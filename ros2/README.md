# ROS 2 Humble 电机控制

主控制/反馈接口与 ELF3 程序一致：使用原 `communication` 消息类型，通过关节名称索引数组。
节点复用 motor_console 后台，持续目标 1 kHz PDO；上层发送频率独立，建议先用 50..100 Hz。
ROS 反馈默认 100 Hz。普通 Linux/ROS 不保证硬实时。

## 构建和启动

```bash
source /opt/ros/humble/setup.bash
/usr/bin/colcon build --base-paths ros2 --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DETHERLAB_ROOT=/usr/local/etherlab
source install/setup.bash
ros2 launch bxi_motor_control motors.launch.py interface:=enp86s0
```

默认自动扫描并预热 MIT、保持失能，不初始化 PCI。电机需外部供电。
需要 BXI 上下电时加 `init_pci:=true auto_power_on:=true`。运行用户需要 EtherLab/BXI 设备权限，
launch 不自动调用 sudo。不能与 motor_console 或其他驱动同时占用同一 EtherCAT master。

## 名称映射

编辑 `ros2/bxi_motor_control/config/joints.yaml`，重新构建安装配置；也可通过启动参数
`joints_file:=/绝对路径/joints.yaml` 使用外部文件，无需重新构建。

```yaml
bxi_motor_control:
  ros__parameters:
    joint_names: [l_shoulder_y_joint, l_shoulder_x_joint, l_shoulder_z_joint, l_elbow_y_joint, l_wrist_x_joint, l_wrist_y_joint, l_wrist_z_joint, r_shoulder_y_joint, r_shoulder_x_joint, r_shoulder_z_joint, r_elbow_y_joint, r_wrist_x_joint, r_wrist_y_joint, r_wrist_z_joint]
    slave_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
    topic_prefix: /hardware/
```

名称必须与上层 `actuators_name` 完全一致。默认映射已由用户确认：左侧从站 1..7，
右侧 8..14，各侧为肩 Y/X/Z、肘 Y、腕 X/Y/Z。两组列表须等长且不能重复。
如果硬件接线改变，应同步修改配置。14 台模组该映射的预期 WKC 为 42。
SAFE-OP 等待按每台 7 秒分配预算（14 台为 98 秒），上限扩展为 120 秒；
它是超时预算，不是固定延时，所有从站就绪后会立即继续。

## 与 ELF3 对齐的话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/hardware/actuators_cmds` | `communication/msg/ActuatorCmds` | 批量 MIT 参数输入 |
| `/hardware/actuator_states` | `communication/msg/ActuatorStates` | 按名称输出反馈 |
| `/bxi_motor_control/command` | `bxi_motor_msgs/msg/MotorCommand` | 使能、失能、Pn082、保存、质量测试、电源等管理命令 |
| `/bxi_motor_control/states` | `bxi_motor_msgs/msg/MotorStateArray` | 详细 PDO 诊断、样本年龄、状态字及目标 |
| `/bxi_motor_control/command_status` | `std_msgs/msg/String` | 请求接收/拒绝信息，不是执行成功确认 |

主参数接口使用 best effort、volatile、深度 1，与 ELF3 发布端匹配。
管理命令保持 reliable、深度 1，详细诊断反馈使用 SensorDataQoS。
`topic_prefix` 可更改主接口的话题前缀。

命令字段：`header`、`actuators_name[]`、`pos[]`、`vel[]`、`torque[]`、`kp[]`、`kd[]`。
所有数组必须与名称等长，名称不能重复，映射关节的数值须有限，KP/KD 非负。
默认 `ignore_unmapped_joints=true`，允许 ELF3 整机命令帧中携带腿部/腰部等名称，
只挑选这 14 个手臂关节下发；没有任何匹配关节时拒绝。设为 false 可严格拒绝未知名称。
整帧复制到后台队列，只占一个队列位置；后台先验证所有目标从站，再在一次 PDO 发送前
更新全部目标，失能从站会执行使能握手。故障、重复名称或映射关节的非法数值不允许部分更新。
只更新消息包含的关节；未包含关节保持原目标，原保持期限继续计时。
后台队列满、PDO 未就绪或样本过期会拒绝提交。先完成扫描预热再发送数组命令。
上层 QoS 和后台有界队列不保证每个 ROS 帧都被电机执行一次，以反馈判断实际状态。

示例在零增益/零前馈下使能，不保持位置：

```bash
ros2 topic pub --once /hardware/actuators_cmds communication/msg/ActuatorCmds '{actuators_name: [l_shoulder_y_joint], pos: [0.0], vel: [0.0], torque: [0.0], kp: [0.0], kd: [0.0]}'
ros2 topic echo /hardware/actuator_states --qos-reliability best_effort
```

实际控制时按机构选择目标位置和增益，持续控制应定期发布。例如上层 50 Hz 更新参数，
底层仍在其间每 1 ms 重发最新参数。每帧刷新所选电机的 `default_hold_ms`（默认 1000ms，
范围 1..60000）期限，到期失能，后台 PDO 保留；可在 launch 设置 `default_hold_ms:=200`。
不直接使用消息 header.stamp 调度电机动作。

反馈字段：`header`、`name[]`、`position[]`、`velocity[]`、`effort[]`、
`driver_temperature[]`、`motor_temperature[]`，按配置顺序输出有效从站。
position 为输出端 rad，velocity 为 rad/s，effort 为反馈 N·m。
当前 PDO 没有温度信息，因此温度数组填 NaN。无效或超过 100ms 的样本不出现在名称反馈中，
详细 `/states` 则保留 valid/sample_age_ms。header.stamp 是发布时刻，不是电机采样时刻。

## 管理命令

MotorCommand：slave_id=1..255，0 表示 all；action=1 ENABLE、2 DISABLE、3 单台 MIT、
4 PN082、5 SAVE、6 QUALITY、7 POWER_ON、8 POWER_OFF。ENABLE/MIT 的 duration_ms=0
使用 default_hold_ms；QUALITY 的 duration_ms=0 默认 10000ms。
电源动作要求 init_pci=true，Pn082 写入和保存要求目标失能。

```bash
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 2, slave_id: 0}'
ros2 topic pub --once /bxi_motor_control/command bxi_motor_msgs/msg/MotorCommand '{action: 6, duration_ms: 10000}'
```

Pn082 接收原始整数编码，不能直接填写目录中的 N·m/Arms。通信质量结果在 launch 日志输出。
退出或 Ctrl-C 会停止后台、发送失能并释放主站；节点负责 BXI 上电时也会下电。

## 消息来源与验证

`ros2/communication` 的 msg/srv 来自工作区中 BXI 原 `communication` 包（BSD 声明保留），
包括其余接口，避免覆盖已有包后导致 ELF3 的其他消息/服务缺失。
若工作区已有同名 communication 源包，只构建一份；两端必须使用相同消息定义。

无硬件 ROS 冒烟测试（禁用扫描和 PCI）：

```bash
ROS_LOCALHOST_ONLY=1 /usr/bin/python3 tests/ros_topics_smoke_test.py
```

启动参数还包括 config_file、init_pci、auto_power_on、auto_scan、feedback_hz。
EtherLab 库目录写入安装 RUNPATH；迁移安装结果到其他机器时应重新构建或提供对应目录。
旧构建遇到 libethercat.so.1 缺失可临时设置 LD_LIBRARY_PATH=/usr/local/etherlab/lib:/usr/local/etherlab/lib64。
