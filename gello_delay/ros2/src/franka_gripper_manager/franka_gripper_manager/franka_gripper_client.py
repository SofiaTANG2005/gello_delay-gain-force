import rclpy
import time
from rclpy.node import Node
from franka_msgs.action import Move
from franka_msgs.action import Homing
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from std_msgs.msg import Float32

DEFAULT_MOVE_ACTION_TOPIC = "franka_gripper/move"
DEFAULT_HOMING_ACTION_TOPIC = "franka_gripper/homing"
DEFAULT_JOINT_STATES_TOPIC = "franka_gripper/joint_states"
DEFAULT_GRIPPER_COMMAND_TOPIC = "gripper/gripper_client/target_gripper_width_percent"
DEFAULT_GRIPPER_COMMAND_DEADBAND_PERCENT = 0.02
DEFAULT_GRIPPER_MOVE_SPEED = 0.1
DEFAULT_MIN_COMMAND_INTERVAL_SEC = 0.0
DEFAULT_TARGET_SETTLE_SEC = 0.0
LATEST_COMMAND_CHECK_INTERVAL_SEC = 0.02


class GripperClient(Node):
    def __init__(self):
        super().__init__("gripper_client")

        self.declare_parameter("move_action_topic", DEFAULT_MOVE_ACTION_TOPIC)
        self.declare_parameter("homing_action_topic", DEFAULT_HOMING_ACTION_TOPIC)
        self.declare_parameter("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)
        self.declare_parameter(
            "gripper_command_deadband_percent", DEFAULT_GRIPPER_COMMAND_DEADBAND_PERCENT
        )
        self.declare_parameter("gripper_move_speed", DEFAULT_GRIPPER_MOVE_SPEED)
        self.declare_parameter("min_command_interval_sec", DEFAULT_MIN_COMMAND_INTERVAL_SEC)
        self.declare_parameter("target_settle_sec", DEFAULT_TARGET_SETTLE_SEC)
        self.declare_parameter("sync_to_first_command", True)

        move_action_topic = (
            self.get_parameter("move_action_topic").get_parameter_value().string_value
        )
        homing_action_topic = (
            self.get_parameter("homing_action_topic").get_parameter_value().string_value
        )
        gripper_command_topic = (
            self.get_parameter("gripper_command_topic").get_parameter_value().string_value
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self._GRIPPER_COMMAND_DEADBAND_PERCENT = (
            self.get_parameter("gripper_command_deadband_percent")
            .get_parameter_value()
            .double_value
        )
        self._GRIPPER_MOVE_SPEED = self.get_parameter(
            "gripper_move_speed"
        ).get_parameter_value().double_value
        self._MIN_COMMAND_INTERVAL_SEC = (
            self.get_parameter("min_command_interval_sec").get_parameter_value().double_value
        )
        self._TARGET_SETTLE_SEC = (
            self.get_parameter("target_settle_sec").get_parameter_value().double_value
        )
        self._sync_to_first_command = (
            self.get_parameter("sync_to_first_command").get_parameter_value().bool_value
        )

        self._ACTION_SERVER_TIMEOUT = 10.0
        self._MIN_GRIPPER_WIDTH_PERCENT = 0.0
        self._MAX_GRIPPER_WIDTH_PERCENT = 1.0
        self._max_width = 0.0
        self._current_gripper_width = 0.0
        self._latest_gripper_command = None
        self._settle_reference_command = None
        self._latest_target_changed_time = 0.0
        self._latest_motion_direction = 0
        self._last_gripper_command = None
        self._last_command_time = 0.0
        self._goal_request_pending = False
        self._active_goal_handle = None
        self._active_gripper_command = None
        self._active_goal_direction = 0
        self._active_goal_started_time = None
        self._cancel_requested = False

        self.get_logger().info("Initializing gripper client...")
        self._home_gripper(homing_action_topic)
        self._get_max_gripper_width(joint_states_topic)
        self._gripper_state_subscription = self.create_subscription(
            JointState, joint_states_topic, self._gripper_state_callback, 10
        )

        self.get_logger().info("Subscribing to gripper commands...")
        self._gripper_command_subscription = self.create_subscription(
            Float32, gripper_command_topic, self._gripper_command_callback, 10
        )
        self._action_client = ActionClient(self, Move, move_action_topic)

        self.get_logger().info("Waiting for gripper move action server...")
        if not self._action_client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError(
                f"Move action server not available after {self._ACTION_SERVER_TIMEOUT} seconds!"
            )

        # Process the most recent command independently of the subscription callback. This makes
        # sure a command received while an action is active is sent as soon as cancellation and
        # rate limiting allow it, even if no additional GELLO message arrives afterwards.
        self._latest_command_timer = self.create_timer(
            LATEST_COMMAND_CHECK_INTERVAL_SEC, self._process_latest_gripper_command
        )

        self.get_logger().info(
            "Gripper client initialized "
            f"(speed={self._GRIPPER_MOVE_SPEED:.3f} m/s, "
            f"deadband={self._GRIPPER_COMMAND_DEADBAND_PERCENT:.3f}, "
            f"min_interval={self._MIN_COMMAND_INTERVAL_SEC:.3f} s, "
            f"settle={self._TARGET_SETTLE_SEC:.3f} s)"
        )

    def _home_gripper(self, homing_action_topic: str) -> None:
        self.get_logger().info("Starting gripper homing...")
        homing_client = ActionClient(self, Homing, homing_action_topic)

        self.get_logger().info(f"Waiting for homing action server {homing_action_topic}...")
        if not homing_client.wait_for_server(timeout_sec=self._ACTION_SERVER_TIMEOUT):
            raise RuntimeError(
                f"Homing action server not available after {self._ACTION_SERVER_TIMEOUT} seconds!"
            )

        self.get_logger().info("Homing action server found!")
        goal_msg = Homing.Goal()
        future = homing_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Homing action rejected!")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        time.sleep(2)

        if result.result.success:
            self.get_logger().info("Gripper homing successful!")
        else:
            raise RuntimeError("Gripper homing failed!")

    def _get_max_gripper_width(self, joint_states_topic: str) -> None:
        self.get_logger().info("Readout maximum gripper width...")
        future = rclpy.task.Future()

        def joint_state_callback(msg):
            _INDEX_FINGER_LEFT = 0
            self._max_width = 2 * msg.position[_INDEX_FINGER_LEFT]
            self._current_gripper_width = self._max_width
            self.get_logger().info(f"Maximum gripper width determined: {self._max_width}")
            future.set_result(True)

        self.get_logger().info(f"Subscribing to {joint_states_topic}...")
        gripper_subscription = self.create_subscription(
            JointState, joint_states_topic, joint_state_callback, 10
        )

        self.get_logger().info(f"Waiting for {joint_states_topic}...")
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info(f"Unsubscribing from {joint_states_topic}")
        self.destroy_subscription(gripper_subscription)

    def _gripper_state_callback(self, msg: JointState) -> None:
        """Track the physical gripper width for direction-aware latest-target handling."""
        if msg.position:
            self._current_gripper_width = 2 * msg.position[0]

    def _gripper_command_callback(self, msg: Float32) -> None:
        new_open_width_percent = max(
            self._MIN_GRIPPER_WIDTH_PERCENT,
            min(self._MAX_GRIPPER_WIDTH_PERCENT, msg.data),
        )
        new_open_width = self._max_width * new_open_width_percent
        now = time.monotonic()

        deadband_width = self._max_width * self._GRIPPER_COMMAND_DEADBAND_PERCENT
        if self._settle_reference_command is None:
            self._settle_reference_command = new_open_width
            self._latest_target_changed_time = now
        elif abs(new_open_width - self._settle_reference_command) >= deadband_width:
            self._latest_motion_direction = (
                1 if new_open_width > self._settle_reference_command else -1
            )
            self._settle_reference_command = new_open_width
            self._latest_target_changed_time = now

        self._latest_gripper_command = new_open_width

        if self._last_gripper_command is None:
            if self._sync_to_first_command:
                self._last_gripper_command = new_open_width
                self.get_logger().info(
                    "Synced initial gripper command without moving: "
                    f"{new_open_width_percent:.3f}"
                )
                return

        self._process_latest_gripper_command()

    def _process_latest_gripper_command(self) -> None:
        """Cancel stale work and dispatch the latest GELLO gripper target when possible."""
        if self._latest_gripper_command is None:
            return

        deadband_width = self._max_width * self._GRIPPER_COMMAND_DEADBAND_PERCENT
        now = time.monotonic()
        target_is_stable = (
            now - self._latest_target_changed_time >= self._TARGET_SETTLE_SEC
        )

        # A send_goal request has not been accepted yet, so there is no goal handle to cancel.
        # The goal response callback will compare it with the latest target again.
        if self._goal_request_pending:
            return

        if self._active_goal_handle is not None:
            enough_time_elapsed = now - self._last_command_time >= self._MIN_COMMAND_INTERVAL_SEC
            target_reversed = (
                self._active_goal_direction != 0
                and self._latest_motion_direction != 0
                and self._active_goal_direction != self._latest_motion_direction
            )
            if (
                not self._cancel_requested
                and enough_time_elapsed
                and target_is_stable
                and target_reversed
                and self._active_gripper_command is not None
                and abs(self._latest_gripper_command - self._active_gripper_command)
                >= deadband_width
            ):
                self._request_active_goal_cancel()
            return

        enough_time_elapsed = now - self._last_command_time >= self._MIN_COMMAND_INTERVAL_SEC
        command_changed = self._last_gripper_command is None or (
            abs(self._latest_gripper_command - self._last_gripper_command) >= deadband_width
        )
        if enough_time_elapsed and target_is_stable and command_changed:
            self._send_gripper_command(self._latest_gripper_command)

    def _send_gripper_command(self, gripper_position: float) -> None:
        goal_msg = Move.Goal()
        goal_msg.width = gripper_position
        goal_msg.speed = self._GRIPPER_MOVE_SPEED
        self._last_command_time = time.monotonic()
        self._last_gripper_command = gripper_position
        self._active_gripper_command = gripper_position
        position_error = gripper_position - self._current_gripper_width
        self._active_goal_direction = 1 if position_error > 0 else -1 if position_error < 0 else 0
        self._active_goal_started_time = self._last_command_time
        self._goal_request_pending = True
        self.get_logger().info(
            f"Sending gripper goal: width={gripper_position:.4f} m, "
            f"speed={self._GRIPPER_MOVE_SPEED:.3f} m/s"
        )
        self._future = self._action_client.send_goal_async(goal_msg)
        self._future.add_done_callback(self._gripper_response_callback)

    def _gripper_response_callback(self, future: rclpy.task.Future) -> None:
        self._goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send gripper goal: {e}")
            self._active_gripper_command = None
            return

        if not goal_handle.accepted:
            self.get_logger().error(f"Gripper goal rejected with status: {goal_handle.status}")
            self._active_gripper_command = None
            return

        self._active_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)
        self._process_latest_gripper_command()

    def _request_active_goal_cancel(self) -> None:
        """Stop an obsolete Move action before sending the newest target."""
        if self._active_goal_handle is None or self._cancel_requested:
            return

        self._cancel_requested = True
        self.get_logger().info(
            "Canceling stale gripper goal in favor of the latest GELLO target: "
            f"active={self._active_gripper_command:.4f} m, "
            f"latest={self._latest_gripper_command:.4f} m, "
            f"current={self._current_gripper_width:.4f} m"
        )
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future: rclpy.task.Future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to cancel stale gripper goal: {e}")
            return

        if not response.goals_canceling:
            self.get_logger().warning(
                "Stale gripper goal could not be canceled; waiting for it to finish"
            )

    def _get_result_callback(self, future: rclpy.task.Future) -> None:
        was_cancel_requested = self._cancel_requested
        elapsed = (
            time.monotonic() - self._active_goal_started_time
            if self._active_goal_started_time is not None
            else 0.0
        )
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as e:
            self.get_logger().error(f"Failed to get gripper result: {e}")
        else:
            if was_cancel_requested:
                self.get_logger().info(
                    f"Stale gripper goal finished after cancellation request ({elapsed:.3f} s)"
                )
            elif result.success:
                self.get_logger().info(f"Result after {elapsed:.3f} s: {result}")
            else:
                self.get_logger().error("Result: {0}".format(result))
        finally:
            # A gripper Move action can report success when the fingers make
            # contact with an object, even though the requested width has not
            # been reached.  For delay experiments, keep issuing the same
            # closing target so contact pressure continues to build instead
            # of stopping at the first contact width.
            # Keep a close command alive until the operator opens the gripper.
            # Do not use measured width as a stop condition: an object may
            # prevent the fingers reaching the requested width, but the user
            # explicitly wants continued closing force in that case.
            retry_closing = not was_cancel_requested and self._active_goal_direction < 0
            self._active_goal_handle = None
            self._active_gripper_command = None
            self._active_goal_direction = 0
            self._active_goal_started_time = None
            self._cancel_requested = False
            if was_cancel_requested:
                # The stopped goal may not have reached its requested width. Clear the deadband
                # reference so the latest target is always reissued, even if it moved back to the
                # width of the canceled goal while cancellation was in progress.
                self._last_gripper_command = None
            elif retry_closing:
                self._last_gripper_command = None

        # Dispatch the newest cached target immediately if the rate limit has elapsed. Otherwise
        # the timer will retry, so intermediate GELLO targets never build up in a queue.
        self._process_latest_gripper_command()


def main(args=None):
    rclpy.init(args=args)
    gripper_client = GripperClient()
    rclpy.spin(gripper_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
