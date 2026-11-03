# GELLO 遥操作与实验测试命令

路径已统一为 `/data/tang_projects`。真机启动脚本会自动加载 ROS 2 Humble 和对应工作区；测试终端需单独 source 一次。

每次只启动一种实验模式；切换模式前先用 Ctrl-C 停止上一组节点。启动脚本使用配置中指定的串口名，不会自动检测或切换到其他串口。

## 首次构建工作区（或修改代码/新增配置后）

以下构建命令每个工作区首次使用时执行一次即可。之后若没有修改代码、launch、`setup.py`、`package.xml` 或新增配置文件，不需要重复构建。

普通 baseline：

```bash
cd /data/tang_projects/gello_basic_launch/gello_software/ros2
source /opt/ros/humble/setup.bash
source /data/tang_projects/franka_ros2_ws/install/local_setup.bash
colcon build --symlink-install
```

增益工作区：

```bash
source /opt/ros/humble/setup.bash
source /data/tang_projects/franka_ros2_ws/install/local_setup.bash
cd /data/tang_projects/gello_gain/ros2
colcon build --symlink-install
```

延迟工作区：

```bash
source /opt/ros/humble/setup.bash
source /data/tang_projects/franka_ros2_ws/install/local_setup.bash
cd /data/tang_projects/gello_delay/ros2
colcon build --symlink-install
```

等效指令扰动工作区：

```bash
cd /data/tang_projects/gello_force/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

构建完成后，各工作区均直接使用对应的 `start_*.sh` 启动，不需要再次执行 `colcon build`。

## 普通 baseline 遥操作

```bash
cd /data/tang_projects/gello_basic_launch
export FRANKA_ROS2_WS=/data/tang_projects/franka_ros2_ws
./start_gello.sh                 # 原始模式：25 Hz，发布周期 40 ms；端到端延迟需实测
./start_gello.sh --rate low      # 25 Hz，额外 0.5 s 延迟（不是限速）
```

## 延迟实验

```bash
cd /data/tang_projects/gello_delay
./start_delay.sh d0               # D0，额外延迟 0 s（机械臂和夹爪）
./start_delay.sh d1               # D1，额外延迟 0.2 s（25 Hz 离散采样）
./start_delay.sh d2               # D2，额外延迟 0.4 s（25 Hz 离散采样）
./start_delay.sh d3               # D3，额外延迟 0.6 s（25 Hz 离散采样）
```

## 增益实验

```bash
cd /data/tang_projects/gello_gain
./start_gain.sh g0                 # G0：关节位移 1.0×（基线增益）
./start_gain.sh g1                 # G1：关节位移 0.2×
./start_gain.sh g2                 # G2：关节位移 0.5×
./start_gain.sh g3                 # G3：关节位移 1.5×
./start_gain.sh g4                 # G4：关节位移 2.0×（夹爪均不缩放）
```

## 速度扰动实验（Force）

`gello_force` 与 gain/delay 一样是独立 overlay，包含两个水平：

| 模式 | 命令 | 行为 |
|---|---|---|
| F0 | `./start_force.sh f0` | 无扰动，1:1 相对关节跟随 |
| F1 | `./start_force.sh f1` | 叠加基座 XY 平面横向扰动 |

```text
F_virtual = alpha * B * [-vy, vx, 0]
v_disturb = mobility * F_virtual        # 经过虚拟力/扰动速度限幅
v_command = v_nominal + v_disturb        # 再经合速度/关节/空间限制
```

`v_nominal` 来自主端关节速度及当前目标姿态的 FR3 Jacobian；机器人反馈速度另行记录。`B` 正负控制旋向，不是固定世界坐标方向。默认 `B=20 N·s/m`、`mobility=0.05 m/(N·s)`，未限幅时扰动速度与原 XY 速度等大，约偏转 45°。这是速度映射扰动，不发送真实末端力或主端力反馈。

```bash
cd /data/tang_projects/gello_force/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install

cd /data/tang_projects/gello_force
./start_force.sh f0               # 无扰动
# 或停止上一组节点后：
./start_force.sh f1               # 明显的横向扰动
```

启动脚本会清理旧控制链路、先让机器人回到当前 GELLO 姿态，再进入正常跟随。F1 从正常跟随阶段开始施加扰动，不自动渐入或撤除：

```bash
source /opt/ros/humble/setup.bash
source /data/tang_projects/gello_force/ros2/install/local_setup.bash
# 软件锁存停止；重启节点才可重新使能：
ros2 service call /gello_publisher/stop std_srvs/srv/Trigger '{}'
```

需要切换水平时停止当前节点，再运行另一个命令。软件锁存停止后必须重启节点才能重新使能。

默认限值为：虚拟力 5 N、扰动速度 0.12 m/s、合成法兰速度 0.20 m/s、关节速度 0.5 rad/s；工作空间为基座坐标 `[0.15,-0.60,0.10]` 至 `[0.80,0.60,0.85] m`。末端是 `fr3_link8` 法兰，不是指尖。软件 stop 不能代替 Franka 硬件急停。

配置文件：`gello_force/ros2/src/franka_gello_state_publisher/config/force_baselinef0.yaml`、`force_f1.yaml`。修改 `field_b` 的正负切换方向。日志追加到 `/tmp/gello_force_f0.csv`、`/tmp/gello_force_f1.csv`，记录理论/限幅虚拟力、请求与最终指令速度、反馈速度和轨迹、试次及 catch 事件；没有真实外力测量。

理论基础为 [Shadmehr & Mussa-Ivaldi (1994)](https://doi.org/10.1523/JNEUROSCI.14-05-03208.1994)。适应后反向后效支持预测性补偿，但本实现属于运动学映射扰动，不能直接宣称复现了真实动力学力场学习。近年理论补充与实验解释见 [Force 完整说明](gello_force/README_force.md)，其中也列出全部配置、日志语义和验证命令。

## 测试终端环境

另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /data/tang_projects/gello_basic_launch/gello_software/ros2/install/local_setup.bash
```

如需显式加载 Franka overlay：

```bash
source /data/tang_projects/franka_ros2_ws/install/setup.bash
```

检查话题和控制器：

```bash
ros2 topic hz /gello/joint_states
ros2 topic echo --once /gello/joint_states
ros2 control list_controllers
```

## gello_test 数据采集

```bash
cd /data/tang_projects/gello_test
python3 show_ee_pose.py
python3 record_joint_states.py --duration 10
python3 record_ee_pose.py --duration 10
```

处理一次到达试次：

```bash
python3 analysis/process_trial.py \
  --input data/demo07/ee_pose.csv \
  --output-dir data/demo07 \
  --target 0.54 0.23 0.19
```

运行完整流水线：

```bash
./run_pipeline.sh
```

测试脚本默认读取当前 ROS 话题，不修改任何遥操作工作区。若 Franka 工作区路径不同，可设置：

```bash
export FRANKA_ROS2_SETUP=/实际路径/franka_ros2_ws/install/setup.bash
export GELLO_ROS2_SETUP=/data/tang_projects/gello_basic_launch/gello_software/ros2/install/local_setup.bash
```

首次真机测试应小幅、低速运动并保持急停可用。
