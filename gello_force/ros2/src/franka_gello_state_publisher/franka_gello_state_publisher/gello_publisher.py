"""GELLO velocity-curl overlay, with explicit trial boundaries and latched stop."""
import csv
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, SetBool
from franka_gello_state_publisher.gello_hardware import GelloHardware
from franka_gello_state_publisher.gello_parameter_config import GelloParameterConfig
from franka_gello_state_publisher.force_core import Kinematics, ForceCore

DEFAULTS = dict(publishing_rate=25, field_b=20.0, mobility=0.05,
    force_limit=5.0, disturbance_speed_limit=0.12, speed_limit=0.20,
    joint_speed_limit=0.5, master_speed_limit=4.0, displacement_limit=0.12,
    damping=0.03, timeout=0.25, tracking_error_limit=0.25, ramp_time=0.5,
    workspace_min=[0.15, -0.60, 0.10], workspace_max=[0.80, 0.60, 0.85],
    level='f0', state_topic='/franka/joint_states', log_file='/tmp/gello_force.csv')
NAMES = [f'fr3_joint{i}' for i in range(1, 8)]


class GelloPublisher(Node):
    def __init__(self):
        super().__init__('gello_publisher')
        self.p = {k: self.declare_parameter(k, v, ParameterDescriptor(read_only=True)).value
                  for k, v in DEFAULTS.items()}
        # Keep a direct attribute for the callbacks/logging below.  The
        # configuration is stored in ``self.p`` but older code also uses
        # ``self.level`` when selecting F0/F1 behavior.
        self.level = str(self.p['level'])
        for k, default in DEFAULTS.items():
            if isinstance(default, (int, float)) and (not np.isfinite(self.p[k]) or (k != 'field_b' and self.p[k] <= 0)):
                raise ValueError(f'Invalid positive parameter: {k}')
        if self.level not in ('f0', 'f1'):
            raise ValueError('level must be f0 or f1')
        lo, hi = np.array(self.p['workspace_min']), np.array(self.p['workspace_max'])
        if lo.shape != (3,) or hi.shape != (3,) or not np.all(np.isfinite([lo, hi])) or not np.all(lo < hi):
            raise ValueError('Invalid workspace bounds')
        model = Kinematics(get_package_share_directory('franka_gello_state_publisher') + '/config')
        self.core = ForceCore(model, self.p)
        self.actual = None
        self.actual_time = None
        self.feedback_anchored = False
        self.actual_velocity = np.zeros(3)
        self.previous_actual_pos = None
        self.last_tick = None
        # Start publishing immediately, like the gain and delay experiments.
        # F0 keeps alpha at zero; F1 keeps the curl field fully enabled.
        self.active = True
        self.stopped = False
        self.target_alpha = float(self.level == 'f1')
        self.alpha = self.target_alpha
        self.trial = 0
        self.catch = False
        path = Path(self.p['log_file']).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.log = path.open('a', buffering=1)
        self.writer = csv.writer(self.log)
        if path.stat().st_size == 0:
            self.writer.writerow(['ros_time_ns', 'monotonic_s', 'event', 'trial', 'level', 'alpha', 'data_json'])
        self.arm = self.create_publisher(JointState, 'gello/joint_states', 1)
        self.gripper = self.create_publisher(Float32, 'gripper/gripper_client/target_gripper_width_percent', 1)
        self.create_subscription(JointState, self.p['state_topic'], self.state, qos_profile_sensor_data)
        self.create_service(Trigger, '~/stop', self.stop)
        hardware_params = {p.descriptor.name: self.declare_parameter(p.descriptor.name, p.default, p.descriptor).value
                           for p in GelloParameterConfig()}
        self.gello_hardware = GelloHardware(hardware_params, self.get_logger())
        self.record('startup', self.p)
        self.timer = self.create_timer(1. / self.p['publishing_rate'], self.tick)
        self.get_logger().info(f'Force ready: level={self.level}, field held continuously. Stop is latched.')

    def record(self, event, data):
        self.writer.writerow([self.get_clock().now().nanoseconds, time.monotonic(), event,
                              self.trial, self.level, self.alpha, json.dumps(data)])

    def state(self, msg):
        try:
            q = np.array([msg.position[msg.name.index(n)] for n in NAMES])
            if not np.all(np.isfinite(q)):
                raise ValueError('Non-finite feedback')
            now = time.monotonic()
            pos, _ = self.core.model.evaluate(q)
            if self.actual_time is not None and now > self.actual_time:
                self.actual_velocity = (pos-self.previous_actual_pos)/(now-self.actual_time)
            self.actual, self.actual_time, self.previous_actual_pos = q, now, pos
        except (ValueError, IndexError):
            return  # Freshness watchdog will latch a stop if valid feedback ceases.

    def start(self, request, response):
        if self.stopped or self.active or self.alpha > 0 or self.actual_time is None or time.monotonic()-self.actual_time > self.p['timeout']:
            response.success, response.message = False, 'Need fresh feedback, idle state and no latched stop'
        else:
            self.active, self.catch = True, False
            self.core.q = None  # Anchor at actual pose on the next master sample.
            self.trial += 1
            self.target_alpha = float(self.level == 'f1')
            response.success, response.message = True, 'Trial started; master anchored at current robot pose'
            self.record('start_trial', {})
        return response

    def end(self, request, response):
        self.active = False
        self.target_alpha = 0.
        response.success, response.message = True, 'Trial ended; holding target, ramping field to zero'
        self.record('end_trial', {})
        return response

    def withdraw(self, request, response):
        response.success = self.active and not self.stopped and self.level == 'f1'
        response.message = 'Field withdrawn immediately; position remains continuous' if response.success else 'Requires active F1 trial'
        if response.success:
            self.alpha = self.target_alpha = 0.
            self.catch = True
            self.record('catch', {})
        return response

    def set_level(self, request, response):
        response.success = not self.active and self.alpha == 0. and not self.stopped
        response.message = 'Set level between trials only'
        if response.success:
            self.level = 'f1' if request.data else 'f0'
            self.record('set_level', {})
        return response

    def stop(self, request, response):
        self.latch('Service stop')
        response.success, response.message = True, 'Stop latched; restart node to re-arm'
        return response

    def latch(self, reason):
        if not self.stopped:
            self.stopped, self.active = True, False
            self.alpha = self.target_alpha = 0.
            # Hold the last already bounded target, never jump to a new target.
            self.record('stop', {'reason': reason})
            self.get_logger().error(reason)

    def tick(self):
        now = time.monotonic()
        dt = 1./self.p['publishing_rate'] if self.last_tick is None else now-self.last_tick
        self.last_tick = now
        try:
            # Publish an initial target even before the robot state broadcaster
            # has produced its first sample.  Waiting for feedback here forms
            # a startup deadlock: the impedance controller waits for a valid
            # target while this node waits for the controller's feedback.
            # Once feedback arrives, the normal watchdog and tracking checks
            # take over on the next ticks.
            if not self.stopped:
                master, width = self.gello_hardware.get_joint_and_gripper_positions()
                master = np.asarray(master, dtype=float)
                if master.shape != (7,) or not np.all(np.isfinite(master)) or not np.isfinite(width):
                    raise ValueError('Invalid GELLO input')
                if self.actual_time is None:
                    # Match delay/gain startup: continuously publish the GELLO
                    # pose so the controller can start and publish feedback.
                    self.core.q = master.copy()
                    self.core.master = master.copy()
                    self.core.anchor = master.copy()
                    self.core.master_anchor = master.copy()
                    self.gripper.publish(Float32(data=float(np.clip(width, 0., 1.))))
                else:
                    if now-self.actual_time > self.p['timeout'] or dt > self.p['timeout']:
                        raise ValueError('Feedback/control watchdog timeout')
                    actual_joints = self.actual
                    actual_position = self.previous_actual_pos
                    actual_velocity = self.actual_velocity
                    if not self.feedback_anchored:
                        # Keep the startup target anchored to GELLO.  This is
                        # the same reset behavior as delay/gain: the robot
                        # first returns to the operator handle pose instead of
                        # rebasing the target to its current robot pose.
                        self.feedback_anchored = True
                if self.actual_time is not None and self.feedback_anchored:
                    if self.active and self.level == 'f0':
                        # Baseline is the same relative joint follow as delay/gain.
                        self.core.q = self.core.q + (master - self.core.master)
                        self.core.master = master.copy()
                        data = dict(command_joints=self.core.q.tolist(), nominal_velocity=np.zeros(3).tolist(),
                                    requested_disturbance=np.zeros(3).tolist())
                    elif self.active:
                        data = self.core.step(master, dt, self.alpha)
                    else:
                        self.core.master = master.copy()
                        data = None
                    if self.active and data is not None:
                        data.update(actual_position=np.asarray(actual_position).tolist(),
                                actual_velocity=np.asarray(actual_velocity).tolist(),
                                actual_joints=np.asarray(actual_joints).tolist(),
                                master_joints=master.tolist(), catch=self.catch)
                        self.record('sample', data)
                        self.gripper.publish(Float32(data=float(np.clip(width, 0., 1.))))
                
        except Exception as exc:
            self.latch(str(exc))
        if self.core.q is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'fr3_link0'
            msg.name, msg.position = NAMES, self.core.q.tolist()
            self.arm.publish(msg)
    def destroy_node(self):
        try:
            if hasattr(self, 'gello_hardware'):
                self.gello_hardware.disable_torque()
            self.log.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GelloPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
