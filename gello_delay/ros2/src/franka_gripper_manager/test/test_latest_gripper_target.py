from unittest.mock import Mock, patch

from franka_gripper_manager.franka_gripper_client import GripperClient


def make_client() -> GripperClient:
    client = GripperClient.__new__(GripperClient)
    client._max_width = 0.08
    client._GRIPPER_COMMAND_DEADBAND_PERCENT = 0.02
    client._MIN_COMMAND_INTERVAL_SEC = 0.1
    client._TARGET_SETTLE_SEC = 0.2
    client._latest_target_changed_time = 0.0
    client._latest_motion_direction = 0
    client._last_command_time = 0.0
    client._goal_request_pending = False
    client._active_goal_handle = None
    client._active_gripper_command = None
    client._active_goal_direction = 0
    client._cancel_requested = False
    return client


def test_idle_client_dispatches_latest_target() -> None:
    client = make_client()
    client._latest_gripper_command = 0.06
    client._last_gripper_command = 0.04
    client._send_gripper_command = Mock()

    client._process_latest_gripper_command()

    client._send_gripper_command.assert_called_once_with(0.06)


def test_active_stale_goal_is_canceled() -> None:
    client = make_client()
    client._latest_gripper_command = 0.02
    client._last_gripper_command = 0.06
    client._active_goal_handle = Mock()
    client._active_gripper_command = 0.06
    client._active_goal_direction = 1
    client._latest_motion_direction = -1
    client._request_active_goal_cancel = Mock()

    client._process_latest_gripper_command()

    client._request_active_goal_cancel.assert_called_once_with()


def test_active_goal_inside_deadband_is_not_canceled() -> None:
    client = make_client()
    client._latest_gripper_command = 0.0605
    client._last_gripper_command = 0.06
    client._active_goal_handle = Mock()
    client._active_gripper_command = 0.06
    client._active_goal_direction = 1
    client._latest_motion_direction = -1
    client._request_active_goal_cancel = Mock()

    client._process_latest_gripper_command()

    client._request_active_goal_cancel.assert_not_called()


def test_active_goal_is_not_canceled_before_rate_limit() -> None:
    client = make_client()
    client._latest_gripper_command = 0.02
    client._last_gripper_command = 0.06
    client._last_command_time = 10.0
    client._active_goal_handle = Mock()
    client._active_gripper_command = 0.06
    client._active_goal_direction = 1
    client._latest_motion_direction = -1
    client._request_active_goal_cancel = Mock()

    with patch(
        "franka_gripper_manager.franka_gripper_client.time.monotonic", return_value=10.05
    ):
        client._process_latest_gripper_command()

    client._request_active_goal_cancel.assert_not_called()


def test_active_goal_is_not_canceled_for_same_direction_update() -> None:
    client = make_client()
    client._latest_gripper_command = 0.07
    client._last_gripper_command = 0.04
    client._active_goal_handle = Mock()
    client._active_gripper_command = 0.04
    client._active_goal_direction = 1
    client._latest_motion_direction = 1
    client._request_active_goal_cancel = Mock()

    client._process_latest_gripper_command()

    client._request_active_goal_cancel.assert_not_called()


def test_idle_client_waits_for_stable_target() -> None:
    client = make_client()
    client._latest_gripper_command = 0.06
    client._last_gripper_command = 0.04
    client._latest_target_changed_time = 10.0
    client._send_gripper_command = Mock()

    with patch(
        "franka_gripper_manager.franka_gripper_client.time.monotonic", return_value=10.1
    ):
        client._process_latest_gripper_command()

    client._send_gripper_command.assert_not_called()
