# x12_4.0 电机 CAN 协议提取

本文只提取 `x12_4.0` 调试最重要的三个功能：

1. 置零
2. 使能
3. MIT 控制

## 1. 通用用法

### 1.1 CAN ID

| 类型 | CAN ID |
| --- | --- |
| 单电机普通命令发送 | `0x140 + id` |
| 单电机普通命令回复 | `0x240 + id` |
| MIT 控制命令发送 | `0x400 + id` |
| MIT 控制命令回复 | `0x500 + id` |

其中 `id` 为电机 ID，范围为 `1 ~ 32`。

例如 `id=1`：

```text
普通命令发送 ID = 0x141
普通命令回复 ID = 0x241
MIT 命令发送 ID = 0x401
MIT 命令回复 ID = 0x501
```

### 1.2 数据格式

CAN 使用 Classic CAN 标准帧：

```text
波特率：1 Mbps
帧格式：标准帧
数据长度：8 字节
```

普通命令一般使用：

```text
CAN ID = 0x140 + id
DATA[0] = 命令字
DATA[1..7] = 参数
```

普通命令回复一般使用：

```text
CAN ID = 0x240 + id
DATA[0] = 命令字
DATA[1..7] = 回复参数
```

## 2. MIT 通信范围

MIT 控制帧包含目标位置、目标速度、前馈力矩、`kp`、`kd`。

| 参数 | 含义 | 通信范围 | 位宽 |
| --- | --- | --- | --- |
| `p_des` | 目标位置 | `-12.566 ~ +12.566 rad` | 16 bit |
| `v_des` | 目标速度 | `-45 ~ +45 rad/s` | 12 bit |
| `kp` | 位置误差系数 | `0 ~ 500` | 12 bit |
| `kd` | 速度误差系数 | `0 ~ 5` | 12 bit |
| `t_ff` | 前馈力矩 | `-Motor Max Torque ~ +Motor Max Torque Nm` | 12 bit |

说明：

- `kd=0~5` 为供应商确认范围。
- `t_ff` 的范围由电机最大力矩决定。
- 对 `x12_4.0`，当前按 `Motor Max Torque = 320Nm` 使用。

MIT 控制关系可理解为：

```text
IqRef = kp * (p_des - p_actual) + kd * (v_des - v_actual) + t_ff
```

因此 `t_ff=0` 只表示前馈力矩为 0，不表示最终输出力矩为 0。只要 `kp>0` 且存在位置误差，电机仍会输出力矩。

## 3. 置零

置零流程：

```text
1. 电机上电
2. 发送使能 / 保持命令
3. 发送当前零位设置命令
4. 发送系统复位命令
5. 复位后重新读取位置，确认零位是否生效
```

组合调试命令：

```text
maita_zero_reset <bus> <id>
```

该命令内部依次发送：

```text
0x81 -> 0x64 -> 0x76
```

### 3.1 使能 / 保持命令：0x81

置零前先进入 Stop/Hold 状态。

发送：

```text
CAN ID = 0x140 + id
DATA   = 81 00 00 00 00 00 00 00
```

回复：

```text
CAN ID = 0x240 + id
DATA[0] = 0x81
DATA[1] = temperature，int8，单位 ℃
DATA[2..3] = iq，int16，小端，单位 0.01A/LSB
DATA[4..5] = speed，int16，小端，单位 dps
DATA[6..7] = angle，int16，小端，单位 degree
```

### 3.2 当前零位设置命令：0x64

将当前位置写入 ROM 作为零点。

发送：

```text
CAN ID = 0x140 + id
DATA   = 64 00 00 00 00 00 00 00
```

回复：

```text
CAN ID = 0x240 + id
DATA[0]    = 0x64
DATA[1..3] = 0x00
DATA[4..7] = encoder_offset，int32，小端
```

解析：

```text
encoder_offset = DATA[4] | DATA[5] << 8 | DATA[6] << 16 | DATA[7] << 24
```

### 3.3 系统复位命令：0x76

零位写入后需要复位生效。

发送：

```text
CAN ID = 0x140 + id
DATA   = 76 00 00 00 00 00 00 00
```

回复：

```text
无回复预期
```

## 4. 使能

使能流程：

```text
1. 电机上电
2. 发送使能 / 保持命令
3. 发送一帧 0 命令 MIT 控制帧
4. 根据 MIT 回复确认电机通信和状态
```

### 4.1 使能 / 保持命令：0x81

发送：

```text
CAN ID = 0x140 + id
DATA   = 81 00 00 00 00 00 00 00
```

回复：

```text
CAN ID = 0x240 + id
DATA[0] = 0x81
DATA[1] = temperature，int8，单位 ℃
DATA[2..3] = iq，int16，小端，单位 0.01A/LSB
DATA[4..5] = speed，int16，小端，单位 dps
DATA[6..7] = angle，int16，小端，单位 degree
```

注意：`0x81` 对应 Stop/Hold，更接近保持状态，不等价于完全无力矩。

### 4.2 发送一帧 0 命令 MIT

使能后发送一帧 MIT 零命令，用于建立 MIT 通信并读取当前位置、速度、力矩。

MIT 零命令物理量：

```text
p_des = 0
v_des = 0
kp    = 0
kd    = 0
t_ff  = 0
```

按 `x12_4.0` 的通信范围打包后：

```text
CAN ID = 0x400 + id
DATA   = 80 00 80 00 00 00 08 00
```

回复：

```text
CAN ID = 0x500 + id
DATA[0]      = 电机 ID
DATA[1..2]   = 当前实际位置 p
DATA[3] 和 DATA[4]高4位 = 当前实际速度 v
DATA[4]低4位 和 DATA[5] = 当前实际力矩 t
DATA[6..7]   = 保留
```

## 5. MIT 控制命令

MIT 控制命令发送到：

```text
CAN ID = 0x400 + id
```

MIT 控制命令回复来自：

```text
CAN ID = 0x500 + id
```

### 5.1 MIT 发送帧格式

MIT 发送数据为大端打包：

```text
DATA[0] = p_des[15:8]
DATA[1] = p_des[7:0]
DATA[2] = v_des[11:4]
DATA[3] = v_des[3:0] << 4 | kp[11:8]
DATA[4] = kp[7:0]
DATA[5] = kd[11:4]
DATA[6] = kd[3:0] << 4 | t_ff[11:8]
DATA[7] = t_ff[7:0]
```

物理量转原始值：

```text
p_raw  = (p_des + 12.566) / 25.132 * 65535
v_raw  = (v_des + 45)     / 90     * 4095
kp_raw = kp               / 500    * 4095
kd_raw = kd               / 5      * 4095
t_raw  = (t_ff + T_MAX)   / (2*T_MAX) * 4095
```

其中：

```text
T_MAX = Motor Max Torque
x12_4.0 建议 T_MAX = 320Nm
```

### 5.2 MIT 回复帧格式

MIT 回复数据为大端打包：

```text
DATA[0] = 电机 ID
DATA[1] = p[15:8]
DATA[2] = p[7:0]
DATA[3] = v[11:4]
DATA[4] = v[3:0] << 4 | t[11:8]
DATA[5] = t[7:0]
DATA[6] = 0x00
DATA[7] = 0x00
```

原始值组合：

```text
p_raw = DATA[1] << 8 | DATA[2]
v_raw = DATA[3] << 4 | DATA[4] >> 4
t_raw = (DATA[4] & 0x0f) << 8 | DATA[5]
```

原始值转物理量：

```text
pos_rad   = p_raw / 65535 * 25.132 - 12.566
vel_rad_s = v_raw / 4095  * 90     - 45
torque_Nm = t_raw / 4095  * (2*T_MAX) - T_MAX
```

示例：

```text
CAN ID = 0x501
DATA   = 01 86 f0 7f f7 ff 00 00
```

解析：

```text
id    = 0x01
p_raw = 0x86f0
v_raw = 0x7ff
t_raw = 0x7ff
```

若 `T_MAX=320Nm`，则力矩按 `-320 ~ +320Nm` 映射。

### 5.3 MIT 位置控制示例

目标：

```text
p_des = 0.1 rad
v_des = 0 rad/s
kp    = 2
kd    = 0.3
t_ff  = 0 Nm
T_MAX = 320Nm
```

发送：

```text
CAN ID = 0x400 + id
DATA   = 按 5.1 公式打包
```

对应调试命令：

```text
maita_pos <bus> <id> 0.1 2 0.3 1000 320
```

完整 MIT 调试命令：

```text
mit_set <bus> <id> <p_des> <t_ff> <v_des> <kp> <kd> <timeout_ms> <T_MAX>
```

### 5.4 MIT 连续发送测试

如果怀疑电机底层 MIT 解析或控制不正常，可以连续发送相同 MIT 帧进行测试。

连续发送命令：

```text
mit_stream <bus> <id> <p_des> <t_ff> <v_des> <kp> <kd> <hz> <duration_ms> [T_MAX]
```

说明：

```text
hz          发送频率，范围 1 ~ 1000Hz
duration_ms 持续发送时间，单位 ms
T_MAX       可选；不填时使用原来的 MIT 力矩通信量程 18Nm
```

只测试 MIT 前馈扭矩时，建议将位置和速度闭环增益清零：

```text
p_des = 0
v_des = 0
kp    = 0
kd    = 0
t_ff  = 测试扭矩
```

例如按 100Hz 连续发送 `0.1Nm` 前馈扭矩，持续 2s：

```text
mit_stream <bus> <id> 0 0.1 0 0 0 100 2000
```

如果需要使用 x12_4.0 的 `±320Nm` 通信范围，则显式传入最后一个参数：

```text
mit_stream <bus> <id> 0 0.1 0 0 0 100 2000 320
```
