"""NumPy-only FR3 kinematics and bounded velocity-curl command integration."""
import numpy as np
import yaml


def rotation(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    x, y, z = axis
    k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def bounded(v, limit):
    return v * min(1., limit / max(float(np.linalg.norm(v)), 1e-12))


class Kinematics:
    """fr3_link0 -> fr3_link8; geometry copied from local franka_description."""
    def __init__(self, directory):
        with open(directory + '/kinematics.yaml') as f:
            geometry = yaml.safe_load(f)
        with open(directory + '/joint_limits.yaml') as f:
            limits = yaml.safe_load(f)
        self.origins = []
        for i in range(1, 9):
            k = geometry[f'joint{i}']['kinematic']
            t = np.eye(4)
            t[:3, 3] = [k[a] for a in ('x', 'y', 'z')]
            t[:3, :3] = (rotation([0, 0, 1], k['yaw']) @
                          rotation([0, 1, 0], k['pitch']) @
                          rotation([1, 0, 0], k['roll']))
            self.origins.append(t)
        self.lower = np.array([limits[f'joint{i}']['limit']['lower'] for i in range(1, 8)]) + .02
        self.upper = np.array([limits[f'joint{i}']['limit']['upper'] for i in range(1, 8)]) - .02

    def evaluate(self, q):
        t = np.eye(4)
        axes, points = [], []
        for i, origin in enumerate(self.origins):
            t = t @ origin
            if i < 7:
                axes.append(t[:3, 2].copy())
                points.append(t[:3, 3].copy())
                r = np.eye(4)
                r[:3, :3] = rotation([0, 0, 1], q[i])
                t = t @ r
        p = t[:3, 3]
        j = np.vstack((np.array([np.cross(a, p - o) for a, o in zip(axes, points)]).T,
                       np.array(axes).T))
        return p, j


class ForceCore:
    def __init__(self, model, params):
        self.model, self.p = model, params
        self.q = self.master = self.anchor = self.master_anchor = None

    def valid(self, q):
        pos, _ = self.model.evaluate(q)
        return (np.all(np.isfinite(q)) and np.all(q >= self.model.lower) and
                np.all(q <= self.model.upper) and
                np.all(pos >= self.p['workspace_min']) and np.all(pos <= self.p['workspace_max']))

    def initialize(self, master, actual):
        if not self.valid(actual):
            raise ValueError('Actual robot state outside configured joint/workspace limits')
        self.q = actual.copy()
        self.anchor = actual.copy()
        self.master = master.copy()
        self.master_anchor = master.copy()

    def step(self, master, dt, alpha):
        if not np.all(np.isfinite(master)) or not 0 < dt <= self.p['timeout']:
            raise ValueError('Invalid master sample or control interval')
        dq = (master - self.master) / dt
        self.master = master.copy()
        if np.max(np.abs(dq)) > self.p['master_speed_limit']:
            raise ValueError('Master sample jump / excessive speed')
        pos, j = self.model.evaluate(self.q)
        nominal_v = j[:3] @ dq
        force = self.p['field_b'] * np.array([-nominal_v[1], nominal_v[0], 0.]) * alpha
        limited_force = bounded(force, self.p['force_limit'])
        disturbance = bounded(self.p['mobility'] * limited_force, self.p['disturbance_speed_limit'])
        # Full 6D Jacobian: request zero additional angular velocity.
        twist = np.r_[disturbance, np.zeros(3)]
        extra = j.T @ np.linalg.solve(j @ j.T + self.p['damping']**2 * np.eye(6), twist)
        delta = (dq + extra) * dt
        scale = min(1., self.p['joint_speed_limit'] * dt / max(np.max(np.abs(delta)), 1e-12))
        nominal_position, _ = self.model.evaluate(self.anchor + master - self.master_anchor)
        for _ in range(24):
            candidate = self.q + delta * scale
            candidate_pos, _ = self.model.evaluate(candidate)
            # Keep the command continuous while retaining instantaneous joint,
            # workspace, and speed bounds.  A cumulative displacement cap
            # makes a long F1 trial latch-stop with "No feasible bounded
            # target" even though the requested velocity is valid.
            if (self.valid(candidate) and np.linalg.norm(candidate_pos-pos) <= self.p['speed_limit']*dt + 1e-12):
                break
            scale *= .5
        else:
            raise ValueError('No feasible bounded target; stop and realign master')
        applied_v = (candidate_pos - pos) / dt
        self.q = candidate
        return dict(nominal_velocity=nominal_v.tolist(), theoretical_force=force.tolist(),
                    limited_virtual_force=limited_force.tolist(), requested_disturbance=disturbance.tolist(),
                    applied_velocity=applied_v.tolist(), effective_velocity_change=(applied_v-nominal_v).tolist(),
                    command_position=candidate_pos.tolist(), command_joints=self.q.tolist(), scale=scale)
