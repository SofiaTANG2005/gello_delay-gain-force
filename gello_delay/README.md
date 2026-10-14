# GELLO 延迟实验

本目录提取 baseline 中的时间戳缓冲延迟实现。发布器把主端采样放入队列，按 `delay` 秒取出历史样本；只发布到期样本，不累积过期命令。急停时按 Ctrl-C 停止全部节点。

实验条件：

| 模式 | 配置 | 注入延迟 |
|---|---|---:|
| D0 / baseline | `delay_baseline.yaml` | 0 s（系统固有延迟） |
| D1 / 250ms | `delay_250ms.yaml` | 0.25 s |
| D2 / 500ms | `delay_500ms.yaml` | 0.5 s |
| D3 / 750ms | `delay_750ms.yaml` | 0.75 s |

## 构建

```bash
cd /data/tang_projects/gello_delay/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 真机启动

确认串口和网络后启动：

```bash
ls -l /dev/serial/by-id/
ping -c 1 172.16.0.2
```

D0 基线：

```bash
cd /data/tang_projects/gello_delay
./start_delay.sh d0
```

D1（额外 250 ms）：

```bash
cd /data/tang_projects/gello_delay
./start_delay.sh d1
```

D2（额外 500 ms）：

```bash
cd /data/tang_projects/gello_delay
./start_delay.sh d2
```

D3（额外 750 ms）：

```bash
cd /data/tang_projects/gello_delay
./start_delay.sh d3
```

脚本会加载 ROS 2 Humble、delay 工作区和 baseline FR3 工作区，并启动 GELLO、FR3 控制器及 Franka Hand。默认 FR3 配置为 `fr3_rate_medium.yaml`，机器人 IP 沿用 baseline 配置。

## 验证与测量

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /gello/joint_states
ros2 topic echo --once /gello/joint_states
ros2 control list_controllers
```

记录设定延迟、主端时间戳、`/gello/joint_states` 发布时间、实际机械臂状态和夹爪状态。夹爪响应延迟需单独测量，不能与该注入量混合。

## 配置

配置文件位于：

```text
/data/tang_projects/gello_delay/ros2/src/franka_gello_state_publisher/config/delay_baseline.yaml
/data/tang_projects/gello_delay/ros2/src/franka_gello_state_publisher/config/delay_250ms.yaml
/data/tang_projects/gello_delay/ros2/src/franka_gello_state_publisher/config/delay_500ms.yaml
/data/tang_projects/gello_delay/ros2/src/franka_gello_state_publisher/config/delay_750ms.yaml
```

若串口设备名变化，修改四个文件的 `com_port`。首次真机测试应小幅、低速，并保持急停可用。
