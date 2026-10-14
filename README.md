# GELLO delay and gain experiments

ROS 2 Humble experiments for GELLO teleoperation of a Franka FR3 robot.

- [`gello_delay/`](gello_delay/README.md): adds configurable command delays of 0, 250, 500, and 750 ms.
- [`gello_gain/`](gello_gain/README.md): scales joint displacement around the startup pose with gains 0.2, 0.5, 1.0, 1.5, and 2.0; gripper commands are not scaled.

Each experiment is an independent colcon workspace overlay and includes its own startup script. Both depend on ROS 2 Humble and the separately built baseline FR3 controller workspace described in the experiment documentation.

> Real-robot safety: test with small, slow motions, keep the emergency stop available, and ensure the robot workspace is clear.
