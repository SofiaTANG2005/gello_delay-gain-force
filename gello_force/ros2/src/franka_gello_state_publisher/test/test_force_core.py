from pathlib import Path
import numpy as np
import pytest
import yaml
from franka_gello_state_publisher.force_core import ForceCore, Kinematics

ROOT = Path(__file__).resolve().parents[1]
Q = np.array([0., -.5, 0., -2., 0., 1.6, .7])


def core():
    params = yaml.safe_load((ROOT/'config/force_f1.yaml').read_text())['SINGLE']
    c = ForceCore(Kinematics(str(ROOT/'config')), params)
    c.initialize(Q, Q)
    return c


def test_jacobian_finite_difference():
    c = core()
    pos, j = c.model.evaluate(Q)
    numeric = np.column_stack([(c.model.evaluate(Q+np.eye(7)[i]*1e-7)[0]-pos)/1e-7 for i in range(7)])
    np.testing.assert_allclose(j[:3], numeric, atol=1e-7)


def test_f0_baseline_and_no_static_drift():
    c = core()
    target = Q + np.array([.001, 0., 0., 0., 0., 0., 0.])
    c.step(target, .04, 0.)
    np.testing.assert_allclose(c.q, target)
    before = c.q.copy()
    c.step(target, .04, 1.)
    np.testing.assert_allclose(c.q, before)


def test_direction_orthogonality_and_catch_continuity():
    a, b = core(), core()
    b.p['field_b'] *= -1
    target = Q.copy(); target[0] += .002
    da, db = a.step(target, .04, 1.), b.step(target, .04, 1.)
    v, f = np.array(da['nominal_velocity']), np.array(da['theoretical_force'])
    assert abs(v @ f) < 1e-12
    assert np.cross(v, f)[2] > 0
    np.testing.assert_allclose(f, -np.array(db['theoretical_force']))
    before = a.q.copy()
    d = a.step(target, .04, 0.)
    np.testing.assert_allclose(a.q, before)  # removal must not erase accumulated offset
    np.testing.assert_allclose(d['requested_disturbance'], 0.)


def test_limits():
    c = core(); c.p['field_b'] = 10000.
    target = Q.copy(); target[0] += .1
    before = c.q.copy()
    d = c.step(target, .04, 1.)
    assert np.linalg.norm(d['limited_virtual_force']) <= c.p['force_limit'] + 1e-12
    assert np.linalg.norm(d['requested_disturbance']) <= c.p['disturbance_speed_limit'] + 1e-12
    assert np.linalg.norm(d['applied_velocity']) <= c.p['speed_limit'] + 1e-10
    assert np.max(np.abs(c.q-before)) <= .04*c.p['joint_speed_limit'] + 1e-12
    assert c.valid(c.q)


def test_workspace_and_bad_samples():
    c = core()
    with pytest.raises(ValueError): c.step(Q, 1., 1.)
    with pytest.raises(ValueError): c.step(Q + 1., .04, 1.)
    with pytest.raises(ValueError): c.step(Q * np.nan, .04, 1.)
    c.p['workspace_max'] = [0., 0., 0.]
    with pytest.raises(ValueError): c.initialize(Q, Q)
