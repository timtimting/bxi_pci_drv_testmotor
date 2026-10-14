# ROS 2 Humble 电机控制

本分支提供 `bxi_motor_msgs`（类型化消息）与 `bxi_motor_control`（节点及 launch）。
节点直接复用 motor_console 的 C 后台，不通过终端文本或 ROS 定时器驱动 PDO。
后台持续目标 1 kHz；ROS 反馈默认 100 Hz，可以通过 `feedback_hz` 设置 1..1000。
普通 Linux/ROS 不保证硬实时。ROS 命令进入原有后台队列，建议目标更新频率先用 50..100 Hz，
ROS 话题的频率与 PDO 频率独立；队列满会拒绝，不保证每个 ROS 消息都成为单独的 PDO 周期。

## 构建和启动

需要 ROS 2 Humble、colcon、EtherLab 开发库，以及本仓库的 BXI 静态库。
在仓库根目录：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --base-paths ros2 --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DETHERLAB_ROOT=/usr/local/etherlab
source install/setup.bash
ros2 launch bxi_motor_control motors.launch.py interface:=enp86s0
```

默认无需 PCI/CAN，电机应已由外部电源供电。启动时自动扫描、MIT PDO/DC 预热、保持伺服失能，
完成后订阅控制话题。若需要通过 BXI 控制电源：

```bash
ros2 launch bxi_motor_control motors.launch.py init_pci:=true auto_power_on:=true
```

运行用户需要访问 EtherLab `/dev/EtherCAT0`；使用 BXI 电源功能还需相应设备权限。
launch 不自动申请 sudo；需要权限时在配置好 ROS 环境且有设备权限的运行终端启动。
节点独占 master 0，不能与 motor_console 同时控制同一主站。

启动参数：`interface`、`config_file`、`init_pci`、`auto_power_on`、`auto_scan`、
`feedback_hz`、`default_hold_ms`。`auto_scan=false` 可在不访问电机的情况下启动节点，
首次 ENABLE/MIT 命令会建立后台会话。`default_hold_ms` 默认 1000，范围 1..60000；
ENABLE/MIT 消息 duration_ms=0 时使用此默认值，超时伺服失能，PDO 后台保留。
持续控制需要在到期前更新命令；duration_ms>0 时使用消息指定期限。

## 话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/bxi_motor_control/command` | `bxi_motor_msgs/msg/MotorCommand` | 使能、失能、MIT、Pn082、保存、通信测试、电源 |
| `/bxi_motor_control/states` | `bxi_motor_msgs/msg/MotorStateArray` | 所有已缓存从站反馈和最新 PDO 目标 |
| `/bxi_motor_control/command_status` | `std_msgs/msg/String` | 请求接收或拒绝，非电机执行完成确认 |

command 使用 reliable、深度 1；states 使用 SensorDataQoS（best effort）。
MotorCommand 中 slave_id=1..255 为终端拓扑编号；0 表示 all。
动作 action：1 ENABLE、2 DISABLE、3 MIT、4 PN082、5 SAVE、6 QUALITY、7 POWER_ON、8 POWER_OFF。
电源动作只在 `init_pci=true` 时可用。参数写入和保存保留原有失能检查。

### 使能 / 失能

```bash
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 1, slave_id: 1, duration_ms: 1000}'
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 2, slave_id: 0}'
```

首次使能 KP/KD/前馈为零，不保持位置。

### MIT 力位混合控制

```bash
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 3, slave_id: 1, position: 0.0, velocity: 0.0, torque: 0.0, kp: 50.0, kd: 10.0, duration_ms: 1000}'
ros2 topic echo /bxi_motor_control/states
ros2 topic echo /bxi_motor_control/command_status
```

position 为输出端绝对 rad，velocity 为 rad/s，torque 为前馈 N·m，KP/KD 是直接发送的 float。
示例会请求绝对零位置，使用前按实际机构修改目标与增益。
状态中的 sample_age_ms 表示 PDO 样本年龄；valid 要求 MIT 浮点格式且样本年龄不超过 100ms。
节点未初始化或缓存缺失时数组可为空，旧样本标记为无效。header.stamp 是 ROS 发布时刻，
不是从站采样时刻；状态字/错误码和目标反馈用于判断执行结果。

### Pn082 与通信质量

```bash
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 4, slave_id: 1, pn082: 10}'
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 5, slave_id: 1}'
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 6, duration_ms: 10000}'
```

Pn082 是原始整数编码，不是目录中的 N·m/Arms 系数；上例 10 仅为语法示例。
QUALITY 必须在已有会话中执行，结果在 launch 后台日志中输出，当前未作为 ROS 统计话题发布。
退出节点或 Ctrl-C 将停止后台、发送失能并释放主站；若节点负责打开 BXI 电源，同时下电。

## 验证

构建后，在仓库根目录且已 source install/setup.bash 的终端运行：

```bash
ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/ros_topics_smoke_test.py
```

该测试自动启动 auto_scan=false/init_pci=false 的节点，验证反馈发布、命令传递、拒绝状态与退出，
不访问电机。它需要本机 DDS 通信权限。完整实机检查仍需验证扫描、使能、反馈、定时失能以及 1kHz 质量统计。
若 EtherLab 安装在非标准目录，除构建路径外，还需要将其 lib 目录加入运行环境的 LD_LIBRARY_PATH。
