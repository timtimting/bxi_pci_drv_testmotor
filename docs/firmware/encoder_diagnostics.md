# 上下电错圈定位

当前日志：断电前电机端 0.951134 圈，上电后 6.951052 圈，差约 +6 圈。
10 次固定点复测均重复该差值。不是目标跟踪误差 0.092338 圈导致的误判。

按提供的 `bxi_motor_source.zip`，启动逻辑（Core/Src/main.c）为：

```c
enc2_pos = LDC1614_GetPos_Usr() * GR;  // 输出编码器折算电机圈数
enc1_pos = Encoder.position;          // 同步前输入编码器位置
turns = enc2_pos - enc1_pos;
Encoder.turns = round(turns);
Encoder.position = Encoder.turns + Encoder.cnt_usr / ENCODER_CPR;
```

`SYNC: enc1=-0.05 enc2=7.08 turns=7.13 pos=6.95` 全部以电机端圈数表示，
其中 `enc2` 已乘减速比，`turns` 打印的是取整之前的差值，实际赋给整圈计数的是 7。
`pos=6.95` 在发送 MIT 回复之前，固件已经计算出来，因此本次 +6 圈跳变发生在固件启动恢复路径。
同步前 -0.05 与原位置 0.951134 的单圈相位相容，但这些仅为两位小数的启动打印，
没有原始编码器读数，不能据此排除输入编码器异常或判定输出编码器硬件坏了。

`CON: fail` 在参考源码由 `USR_CONFIG_read_config()` 非零返回触发：-1 表示 CRC 不匹配，
启动时会回退默认配置；-2 表示配置版本不匹配。原日志没有返回值，不能区分这两者。
`ENC2: CONFIG fail CALIB unknown (normal mode)` 的完整文案不在本源码快照里，
说明实机打印逻辑与快照不完全一致；必须按实机对应源码确认。
日志中 code=1.1.2、config=1.0 本身也不能确定编译期期望的配置版本。

## 已加入的主机采集

- 每轮失能后、断电前发送菜单字符串 `p`，获取 SYSTEM INFO。
- 上电读取 MIT 位置后再次发送 `p`，与原基准比较。
- 每个失败点额外复测 10 次，均发送 `p`，不置零、不使能、不运动。
- `.csv.boot.log` 记录正常轮次的连续启动文本、查询响应及编码器比较。
- `.csv.retries.log` 记录失败点 10 次连续启动文本与查询响应，参考原失败轮断电前快照。
- ENC1 pos 是电机端圈数；ENC2 pos 是输出端圈数，报告同时给输出端角度差和折算电机端角度差。
- ENC1 raw 是计数；补丁固件会报告 CPR，旧固件按参考源码 65536 CPR 解释。
- 原始单圈角度差取最短圆周差以正确处理 0/360 边界；多圈位置差不取模，以保留错圈。
- 老固件 `p` 的位置仅三位小数，诊断值精度低于 MIT 数据；不要替代 MIT 的通过/失败判据。
- 缺字段明确显示 unavailable，不推算一个假的原始角度。每次 `p` 查询预留 1.5 秒响应时间。

旧日志重组（从 hex 字段恢复连续文本，不拼接重复的 text 字段）：

```bash
python3 tools/decode_zero_log.py /tmp/zero-cycle-19160-1792037799828879.csv.retries.log
```

输出同路径追加 `.readable.txt`，原文件不变。新格式日志本身已经是连续可读文本。

## 固件只读诊断补丁

`encoder_diagnostics.patch` 基于提供的源码包，增强现有菜单 `p`，不增加控制或保存动作：

- ENC1 原始码、CPR、校正码、用户码、整圈计数、用户零偏，以及更高精度位置。
- ENC2 校准前 raw 角度、校准后角度、四通道原始值、整圈计数、用户零偏、方向和校准标志。
- 编译期 GR 与配置 reduction_ratio。
- 启动 CONFIG_READ 返回值、存储和期望配置版本。
- 启动 BOOT_RESTORE 取整前后及两编码器输入值，便于定位错误在哪一步出现。

在匹配的固件源码根目录（包含 Core/）先检查补丁：

```bash
patch --dry-run -p1 -i /绝对路径/encoder_diagnostics.patch
patch -p1 -i /绝对路径/encoder_diagnostics.patch
```

补丁保留原源码 CRLF 行尾。源码版本或行尾不匹配时先人工核对，不能盲目强行应用。
需要重新构建并刷入固件后才能获得新增原始字段；此项目没有自动刷写固件。
本环境无 ARM 编译器，补丁只验证能匹配提供的源码，未完成固件编译或实机验证。

手动只读查询可以在 motor_console 已上电时执行 `motor_dbg 01`，发送 Esc 回菜单，
再输入 `p`，用反引号退出直通。不要发送 `x`、`u`、`d` 等会改零位或配置的命令，
以免改变失败点。新诊断不是一次原子硬件采样，分析时应保持电机静止。
