# x12_4.0 电机 CAN 协议提取

`test/maita_arm` 从 `test/maita_motor` 派生，使用上游 `arm64` 分支的
ARM64 PCI/CAN 驱动库和头文件。请在 AArch64 设备上执行 `make`，然后运行
`sudo ./build/motor_console`；x86_64 主机不能链接该驱动库。此版本驱动定义
`CANFD_DEVICE_NUM=7`（`bus=0..6`），原有电机拓扑配置仍只使用 `bus=0..4`；
实际可用端口和供电接线须在设备上核对；首次测试可先用
`power_on`、`maita_temp <bus> <id>` 查询单台已知电机，再运行全端口 `maita_scan`。

## 调试前注意事项

1. **使能后发送 MIT 零控制帧。** 执行 `maita_enable <bus> <id>`（`0x81` Stop/Hold）后，
   在本测试流程中紧接着发送一帧 `p_des=0`、`v_des=0`、`kp=0`、`kd=0`、`t_ff=0` 的 MIT 控制帧。
   终端可用 `maita_info <bus> <id>` 发送该帧并查看位置、速度、力矩回复；它不是纯只读查询，
   也不是写零点的 `maita_zero`，且只发送一帧，不负责持续通信。
2. **置零前先失能。** 确认电机静止、机构安全，先执行 `maita_disable <bus> <id>` 并确认收到
   `0x80` 回复，再执行 `maita_zero <bus> <id>`。后者只提醒、不自动失能；
   `maita_zero_reset` 则会自动先失能，再写零位并发送复位。复位后必须重新检查位置。
3. **温度暂不可依赖。** 当前程序不从 MIT 回复解析温度，`maita_info` 只显示位置、速度和力矩。
   上电后可用 `maita_temp 0 4` 查询总线 0、ID 4 的温度；该命令另行发送 `0x9A` 状态查询。
   现场尚未确认其温度值可靠，
   无回复或异常值不能当作 `0℃`，也不能用于温度保护判断。
4. **MIT 通信范围。** 以下是本程序对 `x12_4.0` 使用的编码范围，不等于机械行程或安全工作范围：

   | 参数 | 范围 | 位宽 |
   | --- | --- | --- |
   | 位置 `p_des` | `-12.566 ~ +12.566 rad` | 16 bit |
   | 速度 `v_des` | `-45 ~ +45 rad/s` | 12 bit |
   | 位置增益 `kp` | `0 ~ 500` | 12 bit |
   | 速度增益 `kd` | `0 ~ 50` | 12 bit |
   | 前馈力矩 `t_ff` | `-320 ~ +320 Nm`（当前 x12 配置） | 12 bit |

5. **增益调试。** `kp=150`、`kd=15` 在上述通信范围内，可作为调试值，
   但并非出厂默认值或保证稳定的配置；先核对当前位置和限位，从小位移、低风险工况开始验证。

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

### 1.3 在终端扫描所有端口和电机 ID

在同一个 `motor_console` 终端中执行 `maita_scan`（别名 `motor_scan`），扫描 PCI/CAN 的全部
7 个端口（驱动索引 `bus=0..6`），每个端口遍历脉塔协议的全部合法电机 ID `1..32`，共 224 个地址。
扫描范围不依赖 `motor_console.yaml` 中的电机列表，不再构建或运行独立的扫描程序。
这里的 ID 是电机逻辑 ID，不是遍历所有 CAN 仲裁 ID，也不会向 ID 0 或广播地址发送命令。

确认机构、负载以及同一路供电的其他电机安全后，构建并启动终端；不要同时运行其他 PCI/CAN 控制程序：

```bash
make
sudo ./build/motor_console
```

在终端中执行：

```text
power_on
maita_scan
```

未上电时扫描会拒绝执行，提示先运行 `power_on`。已上电则直接扫描，不重新上电、不重新初始化驱动。
每个地址只发送读取状态命令：

```text
TX: 标准帧，CAN ID = 0x140 + id，DLC = 8
DATA: 9a 00 00 00 00 00 00 00
RX: 同一 bus，CAN ID = 0x240 + id，DLC = 8，DATA[0] = 9a
```

沿用本分支的 Classic CAN、1Mbps 通信前提，不发送 MIT、使能、停止、置零或参数写入命令。
收到符合条件的回复时，显示端口、ID、原始数据、温度、电压、抱闸状态和错误码；
故障中的电机只要有效回复，仍会列入 `responding_ids`。无回复、发送失败分别标为 `no_reply`、`tx_failed`。
扩展帧、远程帧、CAN FD、错误命令字及其他地址的报文不会被当作本次查询的有效回复。
最后按端口汇总，例如（仅为格式示例）：

```text
[bus0] responding_ids: 1 3; responding=2 checked=32/32 CAN_errors=0
[bus1] responding_ids: none; responding=0 checked=32/32 CAN_errors=0
```

默认每次等待 100ms，没有有效回复时最多尝试 2 次；大多地址无回复时全扫约需 46 秒。
命令会等待扫描完成再返回提示符。可调整等待与尝试次数：

```text
maita_scan 200 3
motor_scan 200 3
help all
```

语法：`maita_scan [timeout_ms] [attempts]`，参数范围分别为 `1..5000` 和 `1..10`。
本分支的 `motor_scan` 使用相同的脉塔扫描流程，不再发送旧的 BXI 探测帧。
扫描结束保持供电及终端原有的 PCI/CAN 连接，可以立即查询扫描到的电机，例如：

```text
maita_temp 0 1
```

扫描只查询状态，不负责保持位置或失能；请在电机静止且机构安全时执行。
`Ctrl-C`/`SIGTERM` 会中断扫描，并沿用终端原有的退出及安全下电流程。
中断结果会标为 `incomplete`，未扫描地址不会算成无回复。
`CAN_errors` 是本次扫描期间收到的 CAN 错误帧数量，不是电机错误码，也不是驱动打印的硬件发送错误计数。

`no_reply` 只代表查询窗口内没有收到有效回复，不能证明地址上没有电机；还需检查电源、
波特率、接线、终端电阻及协议/固件兼容性。同一端口多个电机使用相同 ID 时，
本扫描无法区分它们，且可能发生总线冲突。请仅在脉塔兼容设备总线上使用。

## 2. MIT 通信范围

MIT 控制帧包含目标位置、目标速度、前馈力矩、`kp`、`kd`。

| 参数 | 含义 | 通信范围 | 位宽 |
| --- | --- | --- | --- |
| `p_des` | 目标位置 | `-12.566 ~ +12.566 rad` | 16 bit |
| `v_des` | 目标速度 | `-45 ~ +45 rad/s` | 12 bit |
| `kp` | 位置误差系数 | `0 ~ 500` | 12 bit |
| `kd` | 速度误差系数 | `0 ~ 50` | 12 bit |
| `t_ff` | 前馈力矩 | `-Motor Max Torque ~ +Motor Max Torque Nm` | 12 bit |

说明：

- 脉塔 `mit_set`、`maita_mit`、`maita_pos`、`mit_stream` 统一按 `kd=0~50` 编码，不改变命令参数顺序。
- `kd` 按 `round(clamp(kd, 0, 50) / 50 * 4095)` 换算；电机固件必须使用相同量程，不能混用旧的 `0~5` 编码。
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
1. 确认电机静止、机构安全，然后上电
2. 发送 0x80 失能命令，并确认收到回复
3. 发送 0x64 当前零位设置命令
4. 发送 0x76 系统复位命令
5. 确认电机实际重启后重新读取位置，检查零位是否生效
```

在终端中逐步执行（以 bus=0、id=4 为例）：

```text
power_on
maita_disable 0 4
maita_zero 0 4
maita_reset 0 4
maita_info 0 4
```

`maita_zero` 会提示先执行 `maita_disable <bus> <id>`，但**不会自动失能或检查当前是否失能**。
只有 `maita_disable` 成功回复后才继续置零；`maita_zero` 收到 `0x64` 回复和
`maita_reset` 显示发送成功，都不能单独证明零位已生效，其中 `0x76` 没有回复。
`maita_info` 会发送一帧零力矩 MIT 指令以取得位置反馈，并非纯只读查询。

也可使用组合调试命令 `maita_zero_reset <bus> <id>`，它内部自动依次发送并检查前两步回复：

```text
0x80 -> 0x64 -> 0x76
```

组合命令发送 `0x76` 后仅等待指定时间，仍需再次读取位置确认零位。

### 3.1 失能命令：0x80

置零前先关闭电机输出并确认失能命令得到回复，不要用 `0x81` Stop/Hold 代替失能。

发送：

```text
CAN ID = 0x140 + id
DATA   = 80 00 00 00 00 00 00 00
```

回复：

```text
CAN ID = 0x240 + id
DATA[0] = 0x80
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
kd_raw = kd               / 50     * 4095
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
