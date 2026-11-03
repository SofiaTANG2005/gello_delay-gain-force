# GELLO Force：速度相关横向指令扰动

本工作区沿用 `gello_gain` / `gello_delay` 的独立 ROS overlay 结构，包名仍为 `franka_gello_state_publisher`，只允许同时运行一种实验工作区。使用现有 GELLO 串口驱动、Franka 关节阻抗控制器及夹爪接口。新增的 Python 运动学仅依赖 NumPy 和 YAML，不依赖 Pinocchio，也不回退到前两个关节的旋转映射。

## 理论与本实现的区别

经典 curl force field 为

\[
F = B\begin{bmatrix}0&-1\\1&0\end{bmatrix}v,\quad F_x=-Bv_y,\quad F_y=Bv_x.
\]

`B` 的单位是 N·s/m，正号表示在基座 XY 坐标平面将速度方向旋转 +90° 得到力方向，负号相反。“固定方向”指固定旋向，不是始终向世界坐标的左方施力。从 +Z 看向原点，+X 转向 +Y 为逆时针。`F·v=0` 表示理想力场本身瞬时不做功；真实机器人受操作者、控制器和阻尼等共同影响，不能据此保证实际速率不变。

本版本执行用户要求的**速度矢量叠加**，没有下发真实末端力或关节外加力矩：

```text
qdot_master = (q_master[k] - q_master[k-1]) / dt
v_nominal   = J_linear(q_command) @ qdot_master
F_virtual   = alpha * B * [-v_nominal_y, v_nominal_x, 0]
v_disturb   = clip_norm(mobility * clip_norm(F_virtual, force_limit), disturbance_speed_limit)
v_requested = v_nominal + v_disturb
qdot_extra  = J.T @ inv(J @ J.T + damping² I) @ [v_disturb, 0, 0, 0]
q_next      = q_command + scale * (qdot_master + qdot_extra) * dt
```

这里 `mobility` 是虚拟力到速度的转换系数，单位 m/(N·s)；`B*mobility` 才是无量纲速度扰动比例。使用未扰动的主端运动估计从端名义速度，避免对已经扰动后的速度再次反馈放大。实际从端速度由机器人关节反馈计算并单独记录，不参与力场驱动。全 6D Jacobian 请求额外角速度为零，阻尼逆解仍可能有姿态和速度误差。

无限幅且纯 XY 运动时，`v_requested=(I+mobility*B*R90)v_nominal`，速率增大为原来的 `sqrt(1+(mobility*B)²)`，偏转角为 `atan(mobility*B)`。因此不能沿用“速度大小不变”的说法。这是运动学扰动实验，不能直接等同于经典动力学力场学习；恒定外力、恒定扰动速度与本实验均不同。

运动学参数复制自当前 `franka_description/robots/fr3/{kinematics,joint_limits}.yaml`，末端定义为 **fr3_link8 法兰原点**，坐标系为 **fr3_link0**。不是夹爪指尖 TCP；若实验以工具尖端为参照，需要先扩展工具固定变换及相应 Jacobian。机器人型号或几何参数变化时应同步更新配置。

## 两个实验水平

| 模式 | 启动参数 | 扰动 |
|---|---|---|
| F0 | `baselinef0`（也接受 `f0`、`baseline`） | alpha=0，普通相对关节跟随 |
| F1 | `f1` | 默认 B=20、mobility=0.05，正向 XY curl |

默认 F1 比例为 1.0：名义速度为 `[0.10,0,0] m/s` 时，理论虚拟力为 `[0,2,0] N`、扰动速度为 `[0,0.10,0] m/s`，理想合速度为 `[0.10,0.10,0] m/s`，约 45° 偏转。实际值受逆解、限幅和机器人跟踪影响。将 `field_b` 改为负值即可反向。

启动时先清理旧的 GELLO/Franka 控制进程，并持续发布当前 GELLO 关节目标，使机器人先回到操作杆姿态。收到机器人反馈后进入正常跟随；启动复位只发生一次，后续不会反复重置。F0 使用与 delay/gain 相同的相对关节跟随：

```text
q_robot[k+1] = q_robot[k] + (q_master[k+1] - q_master[k])
```

F1 在同一名义关节增量上叠加水平扰动速度。主端停止时 `dq_master=0`，因此不会继续累积扰动；已经产生的 F1 位置偏移会保留。F0/F1 都只发布位置目标到现有 Franka 关节阻抗控制器，不直接发送力矩。

## 构建与启动

串口依赖为 `pyserial` 与 `dynamixel-sdk`。当前工作区将它们安装在 `ros2/python_deps`，启动脚本自动加载；迁移到新机器且系统没有这些库时，先执行：

```bash
python3 -m pip install --target /data/tang_projects/gello_force/ros2/python_deps --no-deps pyserial dynamixel-sdk
```


```bash
cd /data/tang_projects/gello_force/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select franka_gello_state_publisher

cd /data/tang_projects/gello_force
./start_force.sh f0
# 或先停止上一组节点，再运行：
./start_force.sh f1
```

脚本加载 Franka、basic 和 force 环境（force 最后），启动机器人、GELLO 和夹爪。机器人 IP 沿用 `fr3_rate_medium.yaml` 的 `172.16.0.2`，GELLO 为 25 Hz。可设置 `GELLO_BASIC_ROOT`、`FRANKA_ROS2_WS` 改变基础路径。配置文件为 `ros2/src/franka_gello_state_publisher/config/force_baselinef0.yaml` 与 `force_f1.yaml`；串口和实验参数均在其中修改。配置是启动时只读参数，不能在运动中用 `ros2 param set` 修改。

启动后会先发布 GELLO 目标，再等待 `/franka/joint_states` 的完整七关节反馈；反馈到达后继续正常跟随。F1 从正常跟随阶段开始施加完整扰动，不渐入、不自动撤除。启动脚本会自动检查 force overlay 的包路径，避免误加载 delay/gain 的同名包。

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /data/tang_projects/gello_force/ros2/install/local_setup.bash

# 软件锁存停止；重启节点才可重新使能：
ros2 service call /gello_publisher/stop std_srvs/srv/Trigger '{}'
```

程序运行期间持续保持所选 F0/F1 状态。切换水平需要停止当前节点后重新运行对应启动命令；唯一的运行时控制是锁存停止服务。

## 限制与停止

默认保护：虚拟力 5 N、扰动速度 0.12 m/s、合成目标法兰速度 0.20 m/s、关节目标速度 0.5 rad/s、主端采样速度 4 rad/s。关节位置使用 FR3 限位并留 0.02 rad 裕量；法兰工作空间盒为 `[0.15,-0.60,0.10]` 至 `[0.80,0.60,0.85] m`。反馈/定时器超时为 0.25 s，关节跟踪误差阈值为 0.25 rad。`displacement_limit` 仍保留在配置中用于记录兼容性，但当前 F1 求解不以累计位移作为停止条件。

每步用共同缩放系数检查最终关节目标、目标法兰速度及工作空间，而非只限扰动分量；若无法找到可行增量则锁存停止。较大 `B` 不能绕过这些上限。工作空间盒仅约束法兰点，不是整臂碰撞检测，也不能保证低层插值路径或真实运动严格满足所有上限。运动到关节/工作空间边界、主端采样跳变或反馈超时，可能触发锁存停止，需要结束实验并重新就位。

软件 stop/异常停止会停止更新运动与夹爪目标，并持续发布最后一个受限关节目标；它是**软件保持**，不是断电、制动或安全认证急停。若节点崩溃、串口调用阻塞或进程退出，需依赖原控制器的指令超时处理和 Franka 硬件急停。Ctrl-C 关闭整组节点，也不是硬件急停。实际反馈越界或跟踪异常后，保持旧目标并不能保证机械臂瞬间停下。现场须保留机器人原有碰撞保护和硬件急停，不修改原控制器保护阈值。

## 日志与验证

默认追加 `/tmp/gello_force_f0.csv` 或 `/tmp/gello_force_f1.csv`，包含 ROS/单调时钟、试次编号、水平、alpha、启动参数与边界/catch/stop 事件。每个活动试次样本的 `data_json` 包含：

- 主端关节角、目标关节角、机器人反馈关节角；
- 名义速度、理论虚拟力、限幅虚拟力、请求扰动速度；
- 最终目标速度、相对名义速度的有效变化、共同缩放系数；
- 目标法兰位置、反馈法兰位置与反馈差分速度、catch 标记。

`effective_velocity_change` 包含整个指令的限幅/逆解误差，不能当作真实外力或独立实测扰动；实际速度差也不能简单归因为外力。本包没有真实力传感测量，虚拟力上限不是实际接触力上限。反馈为最新接收样本、速度按接收间隔差分，未进行硬件时钟同步。

```bash
ros2 topic hz /gello/joint_states
ros2 topic echo --once /franka/joint_states
ros2 control list_controllers

cd /data/tang_projects
source /opt/ros/humble/setup.bash
PYTHONPATH=gello_force/ros2/python_deps:gello_force/ros2/src/franka_gello_state_publisher:$PYTHONPATH \
  python3 -m pytest -q gello_force/ros2/src/franka_gello_state_publisher/test
```

## 运动学习依据与后效解释

Shadmehr & Mussa-Ivaldi (1994), *Adaptive Representation of Dynamics During Learning of a Motor Task*, Journal of Neuroscience 14(5), 3208–3224. https://doi.org/10.1523/JNEUROSCI.14-05-03208.1994 。经典机器人力场范式中，适应后撤除扰动出现相反方向的轨迹偏差，支持预测性补偿及内部动力学表征。

Smith, Ghazizadeh & Shadmehr (2006), *Interacting Adaptive Processes with Different Timescales Underlie Short-Term Motor Learning*, PLoS Biology 4(6), e179. https://doi.org/10.1371/journal.pbio.0040179 。不同时间尺度的适应过程说明训练、保持和洗脱应分开设计。

近年理论补充：Tsay, Kim, Haith & Ivry (2022), *Understanding implicit sensorimotor adaptation as a process of proprioceptive re-alignment*, eLife 11:e76639. https://doi.org/10.7554/eLife.76639 。该工作讨论隐式感觉运动适应的解释框架，不是本遥操作速度叠加方案的直接验证，也不构成近几年 curl 力场文献的系统综述。

建议比较 F0 基线、F1 适应、随机 catch 和 F0 洗脱，记录早期横向偏差及撤除后的反向偏差。后效是预测性补偿的重要证据，但还需排除有意识策略、残余偏移、视觉映射变化、限幅和跟踪误差；本版本只能研究该运动学映射的适应，不能仅凭轨迹后效宣称形成了真实外力动力学内部模型。程序不人为生成反向后效，后效是否出现由操作者适应与实验条件决定。
