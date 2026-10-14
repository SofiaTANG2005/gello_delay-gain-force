# GELLO 增益真机实验说明

本目录是在 `gello_basic_launch` baseline 基础上增加位置增益映射的独立 ROS 2 工作区。

映射关系为：

```text
q_robot = q_anchor + gain × (q_master - q_anchor)
```

其中 `q_anchor` 是 GELLO 发布器启动后采集的第一帧主端关节位置。夹爪不进行增益缩放。

当前实验水平：

| 模式 | 配置文件 | 增益 |
|---|---|---:|
| G0 | `gain_g0.yaml` | 1.0 |
| G1 | `gain_g1.yaml` | 0.2 |
| G2 | `gain_g2.yaml` | 0.5 |
| G3 | `gain_g3.yaml` | 1.5 |
| G4 | `gain_g4.yaml` | 2.0 |

## 1. 真机启动前检查

确认 ROS 2 Humble：

```bash
ls /opt/ros/humble/setup.bash
```

确认 GELLO 串口：

```bash
ls -l /dev/serial/by-id/
```

应看到类似：

```text
usb-1a86_USB_Serial-if00-port0 -> ../../ttyUSB0
```

配置文件中的串口字段应与设备名一致：

```bash
grep com_port /data/tang_projects/gello_gain/ros2/src/franka_gello_state_publisher/config/gain_*.yaml
```

确认 FR3 控制柜网络连通（默认 IP 为 `172.16.0.2`）：

```bash
ping -c 1 172.16.0.2
```

启动真机前确认急停、工作空间和碰撞安全设置，首次测试只做小幅度慢速运动。

## 2. 构建增益工作区

```bash
cd /data/tang_projects/gello_gain/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

每次重新打开终端后，启动脚本会自动加载 ROS 2 Humble、baseline 工作区和增益工作区。

## 3. 启动真机

### G0（1.0×，基线增益）

```bash
cd /data/tang_projects/gello_gain
./start_gain.sh g0
```

其余实验水平：

```bash
cd /data/tang_projects/gello_gain
./start_gain.sh g1  # 0.2×
./start_gain.sh g2  # 0.5×
./start_gain.sh g3  # 1.5×
./start_gain.sh g4  # 2.0×
```

启动脚本会依次启动：

1. 增益版 GELLO 发布器；
2. FR3 真机控制器（`fr3_rate_medium.yaml`）；
3. Franka Hand 夹爪节点。

启动日志中应看到对应的增益值：

```text
position gain: 1.000
```

或：

```text
position gain: 0.200 / 0.500 / 1.500 / 2.000
```

按 `Ctrl-C` 会停止本次启动的 GELLO、FR3 和夹爪进程。

## 4. 运行状态验证

另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /data/tang_projects/gello_basic_launch/gello_software/ros2/install/setup.bash
source /data/tang_projects/gello_gain/ros2/install/setup.bash
```

检查 GELLO 主端关节话题：

```bash
ros2 topic hz /gello/joint_states
ros2 topic echo --once /gello/joint_states
```

正常情况下发布频率约为 25 Hz，消息应包含七个 FR3 关节位置。

检查控制器：

```bash
ros2 control list_controllers
```

至少应确认以下控制器为 `active`：

```text
joint_state_broadcaster
joint_impedance_controller
```

检查实际机械臂状态：

```bash
ros2 topic echo --once /franka/joint_states
```

## 5. 配置文件位置

增益配置位于：

```text
/data/tang_projects/gello_gain/ros2/src/franka_gello_state_publisher/config/gain_g0.yaml
/data/tang_projects/gello_gain/ros2/src/franka_gello_state_publisher/config/gain_g1.yaml
/data/tang_projects/gello_gain/ros2/src/franka_gello_state_publisher/config/gain_g2.yaml
/data/tang_projects/gello_gain/ros2/src/franka_gello_state_publisher/config/gain_g3.yaml
/data/tang_projects/gello_gain/ros2/src/franka_gello_state_publisher/config/gain_g4.yaml
```

如果更换 GELLO USB 设备，请修改五个文件中的：

```yaml
com_port: "usb-设备在/dev/serial/by-id中的文件名"
```

FR3 机器人 IP 和控制参数沿用 baseline：

```text
/data/tang_projects/gello_basic_launch/gello_software/ros2/src/franka_fr3_arm_controllers/config/fr3_rate_medium.yaml
```

## 6. 常见问题

### `/opt/ros//setup.bash` 不存在

不要使用未设置的 `$ROS_DISTRO`，直接使用：

```bash
source /opt/ros/humble/setup.bash
```

### 找不到 GELLO 串口

```bash
lsusb
ls -l /dev/serial/by-id/
```

如果设备名变化，更新五个 `gain_g*.yaml` 的 `com_port`。

### 找不到 FR3 配置或控制器

确认 baseline 工作区存在并已构建：

```bash
ls /data/tang_projects/gello_basic_launch/gello_software/ros2/install/setup.bash
```

然后重新构建 baseline 工作区，再运行增益启动脚本。

### 机械臂不动作

先检查：

```bash
ping -c 1 172.16.0.2
ros2 control list_controllers
```

同时确认控制柜已启动、急停已释放、机器人处于可执行状态，并由现场人员监护。
