# BXI PCI 电机统一控制终端

本项目提供 BXI PCIe CAN-FD 控制卡的电机统一控制终端。

唯一对外程序为 `motor_console`。它把电机上下电、在线扫描、MIT 控制、状态管理、
CAN 统计和固件烧录集成在同一个交互终端中。

## 快速开始

在项目根目录编译并检查配置：

```bash
cd ~/bxi_ws/bxi_pci_drv
make -j4
./build/motor_console --check-config
```

启动终端：

```bash
sudo ./build/motor_console
```

## 1. 安全说明

操作真实电机前请确认：

- 机器人已经固定，电机运动不会伤人或损坏机构。
- 急停开关有效并且操作人员可以随时触发。
- CAN Bus、ID、电机型号和逻辑 `index` 与实际接线一致。
- 固件文件与电机型号匹配。
- 执行零位校准前，电机必须处于失能状态。
- 执行位置控制前，先使用较小的 KP、KD 和转矩进行单电机验证。
- 烧录前必须失能全部电机，烧录过程中不要断电或退出程序。

`motor_console` 会记录总电源、在线和使能状态，并拒绝一部分存在状态冲突的命令。
这些软件保护不能代替急停和硬件限位。

## 2. 编译

```bash
make
```

生成以下程序：

```text
build/motor_console   统一交互终端
```

仅检查统一配置文件，不初始化 PCI/CAN，也不操作电机：

```bash
./build/motor_console --check-config
```

## 3. 启动统一终端

```bash
sudo ./build/motor_console
```

也可以进入 `build/` 目录后直接启动：

```bash
cd build
sudo ./motor_console
```

指定其他配置文件：

```bash
sudo ./build/motor_console --config /path/to/motor_console.yaml
```

启动时临时指定语言（优先于配置文件）：

```bash
sudo ./build/motor_console --language zh
sudo ./build/motor_console --language en
```

启动时程序只加载配置、初始化 PCI/CAN 并拉起终端，不会自动上电、使能、复位或
烧录电机。

默认配置语言为中文，因此默认提示符：

```text
电机[已下电]>
```

电机上电后提示符变为：

```text
电机[已上电]>
```

## 4. 终端编辑功能

终端支持：

- `Left` / `Right`：前后移动光标。
- `Backspace` / `Delete`：删除字符。
- `Up` / `Down`：选择历史命令。
- `Tab`：补全命令或显示候选项。
- 终端常规复制、粘贴快捷键，例如 `Ctrl+Shift+C`、`Ctrl+Shift+V`。
- `Ctrl+C`：中止当前操作并退出；如果软件记录的电机电源仍开启，会尝试安全下电。

## 5. 中英文切换

终端默认语言由配置文件决定。当前模板默认中文：

```yaml
language: zh
```

终端内即时切换，不需要重启：

```text
language zh        切换到中文
language en        Switch to English
lang zh            language 的简写
lang en
```

命令名称在中英文模式下保持一致，方便脚本和操作记录复用。

涉及配置电机序号的命令统一使用两位十进制 `index00` 参数，例如 `00`、`08`、`30`。

## 6. 查看帮助

程序启动前查看英文帮助：

```bash
./build/motor_console -h
./build/motor_console --help
./build/motor_console -h all
```

查看中文帮助：

```bash
./build/motor_console --language zh --help
```

终端内查看帮助。默认输出精简分组命令，详细说明使用 `all`：

```text
help
-h
?
help all
-h all
```

## 7. 推荐操作流程

### 7.1 上电、扫描和查看状态

```text
电机[已下电]> power_on
电机[已上电]> motor_list
电机[已上电]> can_status
```

`power_on` 会执行：

1. 打开电机总电源。
2. 按 `power_on_wait_ms` 等待软启动。
3. 被动监听电机上电输出，识别已在线电机。
4. 只向已确认在线电机发送 `can_warmup_count` 次零 MIT 帧，让 CAN TX error 通过成功 ACK 逐步恢复。
5. 对已在线电机识别程序/配置版本；版本读取失败只提示，不阻塞上电。
6. 如果被动监听没有发现电机，再进入主动扫描；空总线可能产生 CAN TX error。

典型输出：

```text
power_on: start 电源已开启 wait=2s
version_scan: start total=31
version_scan: done total=31 success=31 failed=0
power_on: done total=31 success=31 failed=0
```

如果有电机未回复，会自动选择更少的一侧输出，避免刷屏：

- 离线电机较少：输出 `offline`，方便直接定位未回复关节。
- 在线电机较少：输出 `online`，方便确认当前实际连上的电机。

```text
power_on: start 电源已开启 wait=2s
[motor08]: offline name=左腿 bus=1 id=6
[motor29]: offline name=头部 bus=0 id=4
power_on: done total=31 success=29 failed=2
```

大部分电机未回复时，输出会变成：

```text
power_on: start 电源已开启 wait=2s
[motor00]: online name=腰部左边 bus=0 id=1
[motor03]: online name=左腿髋部pitch、y轴 bus=1 id=1
power_on: done total=31 success=2 failed=29
```

也可以手动重新扫描：

```text
motor_scan
motor_scan 1500
```

### 7.2 校准零位

给全部电机发送 MIT 零位校准帧：

```text
mit_zero_set_all
```

给单台电机发送零位校准帧：

```text
mit_zero_set_single 00
```

零位校准要求目标电机处于失能状态。如果软件记录该电机仍然使能，命令会被拒绝。

### 7.3 使能和失能

```text
mit_enable_all
mit_disable_all

mit_enable_single 00
mit_disable_single 00
```

只有收到电机回复后，软件才会把该电机记录为已使能或已失能。可以通过
`motor_list` 检查状态。

### 7.4 单电机 MIT 控制

```text
mit_set <index00> <pos> <torque> <vel> <kp> <kd>
```

示例：控制 `index 00` 到位置 `0.5`，目标速度和前馈转矩为 0：

```text
mit_set 00 0.5 0 0 10 1
```

参数顺序：

| 参数 | 含义 |
|---|---|
| `index` | 配置文件中的逻辑电机序号 |
| `pos` | 目标位置 |
| `torque` | 前馈转矩 |
| `vel` | 目标速度 |
| `kp` | 位置增益 |
| `kd` | 速度增益 |

目标电机必须已经在线并确认使能。所有参数都会根据配置中的 MIT 范围进行检查，超限
命令不会发送。

### 7.5 全部电机软启动回零

```text
stand_up
```

该命令会选择当前在线且已使能的电机；离线或未使能的电机会跳过，不影响本次执行。程序将：

1. 目标位置设置为 0。
2. 速度和前馈转矩设置为 0。
3. KD 使用配置中的 `home_kd`。
4. KP 在 `home_soft_start_ms` 时间内从 0 缓慢增加到 `home_kp`。

建议先降低 `home_kp` 并在机构安全状态下验证。

### 7.6 安全下电

```text
mit_disable_all
power_off
quit
```

如果仍有已知使能电机，`power_off` 会先发送全部失能命令，再关闭总电源。电源开启时
直接执行 `quit` / `q` / `qq` 会自动执行一次 `power_off`，下电成功后再退出。

## 8. CAN 输出和统计

实时输出由配置文件控制：

```yaml
live_output: on
```

MIT 回复会解析并显示：

- 电机逻辑 `index`
- CAN Bus 和电机 ID
- 位置
- 速度
- 转矩
- MOS 温度
- 电机温度

查看每路 CAN 软件统计：

```text
can_status
```

清零统计：

```text
can_status reset
```

统计内容包括：

- 发送帧数和发送失败数。
- 接收帧数。
- 命令期望回复数和匹配回复数。
- 解码到的电机帧数。
- 回复超时和估算丢包率。

当前公开驱动接口没有提供 CAN 控制器 TEC、REC 和 bus-off 状态，因此这里显示的是
软件发送错误与命令回复丢失统计，不是底层控制器硬件错误寄存器。

快速查看单个电机原始回复解析：

```text
can_dbg 00
```

`can_dbg` 会：

1. 读取 `0x6d mit_aux_enable`。
2. 发送一帧零 MIT 探测帧。
3. 打印该电机的 MIT 回复解析结果。

新版固件输出示例：

```text
motor_reply: start total=1 index=00 bus=0 id=1 timeout_ms=3000 probe
[motor00]: mit_decode=aux mit_aux_enable=1 reason=register_0x6d_enabled
[motor00]: pos= 0.02041 vel=-0.01099 torque=-0.01954 aux=0x3:vbus value=24.0V raw=240
motor_reply: done total=1 success=1 failed=0 frames=1
```

## 9. 固件烧录

### 9.1 准备固件

默认固件目录是：

```text
firmware/
```

对应文件应放在：

```text
firmware/bxi_motor_50.bin
firmware/bxi_motor_50L.bin
firmware/bxi_motor_70.bin
firmware/bxi_motor_85.bin
```

如果需要按电机程序主版本区分固件，也可以放置：

```text
firmware/bxi_motor_50_v0.1.2.bin
firmware/bxi_motor_50L_v0.1.2.bin
firmware/bxi_motor_70_v0.1.2.bin
firmware/bxi_motor_85_v0.1.2.bin
firmware/bxi_motor_50_v1.1.2_new.bin
firmware/bxi_motor_50L_v1.1.2_new.bin
firmware/bxi_motor_70_v1.1.2_new.bin
firmware/bxi_motor_85_v1.1.2_new.bin
```

如果固件经常更新，也可以把固件放在内网下载站中，不必提交到项目里。
在 `config/motor_console.yaml` 中配置下载站地址：

```yaml
firmware_dir: firmware
firmware_sync_on_start: off
firmware_base_url: "http://your-download-server/motor"
firmware_cache_dir: firmware_cache
```

程序正常启动时会先根据默认烧录计划 `config/flash_plan_default.yaml` 拉取一次所需固件：

- `firmware_sync_on_start: off`：关闭启动同步，直接使用项目自带 `firmware_dir`
- `firmware_sync_on_start: on`：启用启动同步
- 全部固件拉取成功：本次运行烧录优先使用 `firmware_cache_dir`
- 任意固件拉取失败：本次运行自动回退项目自带 `firmware_dir`
- `firmware_base_url` 留空：不联网，直接使用项目自带 `firmware_dir`

下载文件名由 `config/motor_console.yaml` 中 `motor_types.firmware` /
`firmware_v0` / `firmware_v1` 决定。
例如：

```yaml
motor_types:
  - type: "50L"
    firmware: bxi_motor_50L.bin
    firmware_v0: bxi_motor_50L_v0.1.2.bin
    firmware_v1: bxi_motor_50L_v1.1.2_new.bin
```

当烧录配置里写 `version: 50L` 时，会下载：

```text
http://your-download-server/motor/bxi_motor_50L.bin
```

如果运行时已经识别出当前电机是 0.x 或 1.x 程序/配置版本，烧录时会优先使用
`firmware_v0` 或 `firmware_v1` 对应文件。

下载成功后缓存为：

```text
firmware_cache/bxi_motor_50L.bin
```

下载依赖设备上安装 `curl` 或 `wget`；两者有一个即可。

终端配置 `config/motor_console.yaml` 只负责电机型号和 MIT 打包/解包范围。
默认烧录版本放在 `config/flash_plan_default.yaml`。
`Kp` 通用范围为 `[0,500]`，`p_des` 通用范围为 `[-12.5,12.5]`，
`v_des` 通用范围为 `[-45,45]`。

### 9.2 烧录单台电机

```text
power_on
mit_disable_all
flash_single <index00> [version|firmware.bin] [cycle]
```

示例：

```text
flash_single 00
flash_single 08 50L
flash_single 08 test.bin
```

`flash_single` 不写固件参数时，会优先使用运行时从电机输出中识别到的型号
（例如 `PRO/HW/Bxi_motor_50L` -> `50L`）选择固件；没有识别到时，回退使用
`config/flash_plan_default.yaml` 中该 index 对应的 `version`。如果程序启动时远程
固件全部拉取成功，会优先使用 `firmware_cache_dir`；否则使用 `firmware_dir`。
`50L` 会根据当前电机版本优先解析为 `motor_types.firmware_v0` 或
`motor_types.firmware_v1`；没有版本信息时使用 `motor_types.firmware`。

### 9.3 按烧录策略文件烧录多个电机

默认烧录全部电机可以直接使用：

```text
flash_all
```

如果需要反复烧录几个固定电机，可以复制默认配置后删改 `targets`：

```text
cp config/flash_plan_default.yaml config/flash_plan_debug.yaml
```

然后修改 `config/flash_plan_debug.yaml`：

```yaml
firmware_dir: firmware
firmware_base_url: "http://your-download-server/motor"
firmware_cache_dir: firmware_cache

targets:
  - index: 0
    bus: 0
    id: 1
    version: 70

  - index: 8
    bus: 1
    id: 6
    version: 50L

  - index: 29
    bus: 0
    id: 4
    version: bxi_motor_50.bin
```

执行：

```text
power_on
mit_disable_all
flash_all config/flash_plan_debug.yaml
```

说明：

- 烧录配置描述固件路径和烧录目标：`firmware_dir`、`index`、`bus`、`id`、`version`。
- `flash_all` 会按目标逐个烧录；如果某台电机已经从上电输出中识别到实际型号，
  会优先使用识别型号对应的固件，否则使用烧录配置里的 `version`。
  同时会根据已识别到的 0.x/1.x 程序或配置版本，在
  `firmware_v0` / `firmware_v1` 中选择文件；未识别版本时使用 `firmware`。
- `firmware_base_url` 可选；配置后，程序启动时会尝试把默认烧录计划所需固件下载到
  `firmware_cache_dir`。
- `version` 支持型号简写，例如 `50L` 会下载/查找 `motor_types` 中配置的文件名。
- `version` 也可以写完整文件名，例如 `bxi_motor_50.bin`、`test.bin`。
- 安全限制：`firmware_dir` / `firmware_cache_dir` 必须是相对目录，不能包含绝对路径或
  `..`；`firmware_base_url` 必须是 `http://` 或 `https://`；`version`
  和手动固件参数不能包含 `/` 或 `..`。
- 终端语言、电机输出状态识别、MIT 参数和电机型号配置都放在 `config/motor_console.yaml`。

### 9.4 烧录全部电机

```text
power_on
mit_disable_all
flash_all
```

烧录前程序会预检所有目标及固件文件。任何文件缺失时，整个批次会在操作第一台电机
之前停止。已经识别到实际型号的电机，会自动覆盖默认烧录配置中的 `version` 来选择
对应固件；手动输入固件参数时，以手动参数为准。

如果普通烧录无法进入升级状态，可以增加 `cycle`：

```text
flash_single 00 cycle
flash_all cycle
flash_all config/flash_plan_debug.yaml cycle
```

`cycle` 会操作电机总电源，使用前应确认其他机构处于安全状态。

## 10. 配置文件

默认配置文件：

```text
config/motor_console.yaml
```

配置包含：

- 界面语言。
- 上电等待及扫描超时。
- 实时 CAN 输出和 CAN-FD 开关。
- MIT 位置、速度、转矩、KP 和 KD 范围。
- 全部电机软启动回零参数。

电机逻辑序号、CAN Bus、ID 和默认烧录版本放在 `config/flash_plan_default.yaml`。
当前默认拓扑：

| 位置 | CAN 范围 | Index 范围 |
|---|---|---|
| 腰部 | `bus0/id1-3` | `0-2` |
| 左腿 | `bus1/id1-6` | `3-8` |
| 右腿 | `bus2/id1-6` | `9-14` |
| 左手 | `bus3/id1-7` | `15-21` |
| 右手 | `bus4/id1-7` | `22-28` |
| 头部 | `bus0/id4-5` | `29-30` |
| 左手夹爪 | `bus5/id1` | `31` |
| 右手夹爪 | `bus6/id1` | `32` |

夹爪使用 `bxi_motor_50L_hand.bin`（电机固件 `USE_ENC2=0`）并采用物理量 CAN
通信：位置为 `0..90 mm`（实际机械行程约 `0..80.55 mm`），速度为
`-641..641 mm/s`，力矩字段为夹爪力 `-24..24 N`，`Kp` 范围为 `0..5`，`Kd` 范围为 `0..1`。
夹爪固件内部先将夹持力换算为电机侧力矩，再按 `KT` 换算为 q 轴电流；上位机
不应把夹爪的力值当作普通关节的 N·m。示例：

```text
power_on
motor_list
mit_enable_single 31
motor_set 31 20 0 100 20 1
motor_set 32 20 0 100 20 1
mit_disable_single 31
mit_disable_single 32
power_off
```

首次调试建议只连接并操作一只夹爪，低速、低力矩开始，并保持急停可用。

## 11. 当前命令

终端里可以执行 `help` / `-h` / `?` 查看精简分组命令；执行
`help all` / `-h all` 查看每个命令的详细说明。实际测试时建议优先使用下面这些常用命令。

### 11.1 常用操作

| 命令 | 参数 | 说明 |
|---|---|---|
| `help` / `-h` / `?` | `[all]` | 显示终端帮助；带 `all` 显示详细说明 |
| `power_on` | 无 | 电机上电、等待软启动、扫描全部电机，并读取在线电机版本 |
| `power_off` | 无 | 失能已知使能电机并关闭总电源 |
| `motor_probe` | 无 | 上电、接收并显示配置电机回复，最后无论结果如何都会自动下电 |
| `motor_list` | 无 | 显示电机 index、bus、id、型号、在线/使能状态、程序/配置版本和最后反馈 |
| `motor_scan` | `[timeout_ms]` | 重新扫描配置电机 |
| `can_status` | `[reset]` | 显示或清零 CAN 软件统计 |
| `quit` / `exit` / `q` / `qq` | 无 | 退出终端；电源开启时会先自动下电 |

### 11.2 MIT 控制

| 命令 | 参数 | 说明 |
|---|---|---|
| `mit_zero_set_all` | 无 | 给全部电机发送 MIT 零位校准帧 |
| `mit_zero_set_single` | `<index00>` | 给单台电机发送 MIT 零位校准帧 |
| `mit_enable_all` | 无 | 使能全部电机 |
| `mit_disable_all` | 无 | 失能全部电机 |
| `mit_enable_single` | `<index00>` | 使能单台电机 |
| `mit_disable_single` | `<index00>` | 失能单台电机 |
| `mit_set` | `<index00> <pos> <torque> <vel> <kp> <kd>` | 给单台电机发送一次 MIT 控制帧 |
| `stand_up` | 无 | 全部在线且已使能电机软启动回零 |
| `reg_read` | `<index00\|all> <reg_index> [wait_ms]` | 读取寄存器；`all` 按默认烧录配置逐台发送 |
| `reg_write` | `<index00\|all> <reg_index> <type> <value> [wait_ms]` | 写入寄存器；`type` 由用户指定；`all` 按默认烧录配置逐台发送；要求没有已知使能电机 |
| `reg_save` | `<index00\|all> [wait_ms]` | 保存寄存器配置；`all` 按默认烧录配置逐台发送；要求没有已知使能电机 |
| `reg_info` | `<index00\|all> [wait_ms]` | 读取该电机支持的寄存器总数和可写数量 |

寄存器命令只做通用协议收发和解析，不在上位机保存“寄存器地址对应参数名/类型”的完整表：
`reg_read` 使用 `CAN_CMD_REG_READ=0x17`，`reg_write` 使用
`CAN_CMD_REG_WRITE=0x18`，`reg_save` 使用 `CAN_CMD_REG_SAVE=0x19`，
`reg_info` 使用 `CAN_CMD_REG_INFO=0x1A`。寄存器地址范围是 `0x00..0xFF`。
`reg_read` 请求 `data[0..3]=address`；`reg_write` 请求
`data[0..3]=address | (value_type << 8)`、`data[4..7]=value_raw`。
`reg_write` 的 `value_type` 由用户通过命令参数指定，电机端负责校验地址和类型是否匹配；
如果不匹配，回复 `status=5(TYPE_MISMATCH)`。回复会解析 `status`、`value_type` 和 value；
`value_type` 使用 `bit8..10`，目前 `0=int`、`1=bool`、`2=float`、`3=uint32`、`4=version`。
`reg_info` 回复的 `value_raw` 低 16 位是寄存器总数，高 16 位是可写数量。
配置版本地址 `0x7C` 仍由上位机固定识别，用于上电后的版本扫描；如果没有版本回复，
该电机按最老版本/无版本信息处理，但普通寄存器命令仍可发送，由电机端返回实际状态。

常用寄存器示例：

```text
reg_read 00 0x7c     # 读取配置版本
reg_read 00 0x6d     # 读取 MIT AUX 轮询开关，0=旧版 NTC，1=AUX 轮询
reg_write 00 0x6d bool 1  # 本次上电周期启用 MIT AUX 轮询回复
reg_write 00 0x6d bool 0  # 切回旧版 NTC 回复
reg_read 00 0x01          # 读取 0x01，类型和值以电机回复为准
reg_write 00 0x01 int 10  # 按 int 类型写入 0x01；类型是否正确由电机回复确认
reg_info 00          # 读取寄存器数量信息
reg_save 00          # 保存配置到电机 Flash
```

`0x6d mit_aux_enable` 是运行时开关，电机上电默认值为 0，且不会保存到 Flash。
需要 MIT AUX 轮询遥测时，每次上电后都需要重新写 1。

### 11.3 烧录

| 命令 | 参数 | 说明 |
|---|---|---|
| `flash_single` | `<index00> [version\|firmware.bin] [cycle]` | 烧录单台；不写固件时优先按识别型号选固件，否则使用默认 version |
| `flash_all` | `[plan.yaml] [cycle]` | 按烧录配置逐台烧录；已识别型号会优先覆盖 plan version |
| `flash_debug` | `<index00>\|<bus0-6 id0-7> <version\|firmware.bin> [cycle]` | 两位数按 index；一位 bus+一位 id 按 bus/id 调试烧录 |

旧 `flash_plan <plan.yaml>` 合并为 `flash_all <plan.yaml>`。
旧 `flash_file <index00> <version|firmware.bin>` 合并为
`flash_single <index00> <version|firmware.bin>`。

推荐默认烧录：

```text
power_on
mit_disable_all
flash_all
```

调试少量电机：

```text
cp config/flash_plan_default.yaml config/flash_plan_debug.yaml
# 删除不需要烧录的 targets，或修改 version
flash_all config/flash_plan_debug.yaml
```

### 11.4 调试和配置

| 命令 | 参数 | 说明 |
|---|---|---|
| `motor_dbg` | `<index00>` | 按 index 进入单电机直通调试；所有按键直接发给电机，反引号 `` ` `` 退出 |
| `language` / `lang` | `zh\|en` | 切换中英文 |

## 12. 源码结构

`motor_console` 的源码主入口在 `src/main.c`。为了复用 Boot/YMODEM/MIT
底层实现，它仍然在同一个编译单元内包含 `src/lib/runtime.c` 和下面这些功能模块。
`src/lib/runtime.c` 是 `motor_console` 的内部实现文件，不作为独立程序构建。
`src/` 根目录只保留入口文件 `main.c`，其他功能文件放在 `src/lib/` 下，由
`main.c` include，不要单独加入 CMake 编译：

| 文件 | 内容 |
|---|---|
| `src/main.c` | 程序入口、统一 include 内部模块 |
| `src/lib/core.c` | 通用工具、配置路径、电机查找和状态保护 |
| `src/lib/display.c` | help、配置、电机列表等输出 |
| `src/lib/control.c` | 上下电、扫描、MIT 控制、CAN 统计 |
| `src/lib/flash.c` | 单台/全部/计划烧录和烧录配置解析 |
| `src/lib/debug.c` | 单电机直通调试、`can_dbg` 单电机回复解析 |
| `src/lib/terminal.c` | 命令分发、配置重载、交互终端循环 |
| `src/lib/bxi_motor_comm.c` / `src/lib/bxi_motor_comm.h` | MIT 帧打包、特殊命令帧、回复解析和默认参数范围 |

### 12.1 MIT 轮询解析相关函数

| 函数 | 位置 | 说明 |
|---|---|---|
| `bxi_motor_unpack_reply` | `src/lib/bxi_motor_comm.c` | 新版 MIT 回复解析；解析位置、速度、力矩和 AUX 轮询遥测 |
| `bxi_motor_unpack_reply_legacy_ntc` | `src/lib/bxi_motor_comm.c` | 旧版 MIT 回复解析；最后两个字节按 NTC 温度解析 |
| `mit_reply_frame_matches` | `src/lib/runtime.c` | 判断 CAN 帧是否匹配某个电机的 MIT 回复 |
| `console_unpack_motor_reply_by_aux_enable` | `src/lib/debug.c` | 根据 `mit_aux_enable` 选择新版 AUX 或旧版 NTC 解析 |
| `console_print_raw_motor_can_frame` | `src/lib/debug.c` | `can_dbg/motor_reply` 的单电机回复输出 |

### 12.2 寄存器协议相关函数

| 函数 | 位置 | 说明 |
|---|---|---|
| `console_read_motor_register` | `src/lib/debug.c` | `can_dbg` 前置读取 `0x6d` 的 MIT AUX 开关 |
| `console_scan_online_versions` | `src/lib/control.c` | `power_on` 后读取在线电机版本，并缓存到运行时状态 |
| `reg_value_type_from_wire` | `src/lib/runtime.c` | 从 `reply_header` 解析 `value_type` |
| `reg_request_header` | `src/lib/runtime.c` | 生成写寄存器请求头：`address | value_type << 8` |
| `reg_reply_status` | `src/lib/runtime.c` | 从 `reply_header` 解析寄存器返回状态 |
| `reg_reply_value_type` | `src/lib/runtime.c` | 从 `reply_header` 解析 `bit8..10` 的 `value_type` |
| `print_register_value` | `src/lib/runtime.c` | 按 `value_type` 打印 int/bool/float/uint32/version |
| `print_register_debug_frame` | `src/lib/runtime.c` | `reg_read/reg_write/reg_save/reg_info` 回复的主要解析和输出函数 |
| `console_reg_send_one` | `src/lib/control.c` | `reg_read/reg_write/reg_save/reg_info` 对单台电机的实际发送函数 |
| `console_parse_register_type_arg` | `src/lib/control.c` | 解析用户输入的 `type`：int/bool/float/uint/version 或 0..4 |
| `console_parse_register_value_arg` | `src/lib/control.c` | `reg_write` 输入值解析，并按用户指定类型转成 32 位 raw value |

### 上下电零位识别随机测试（CAN）

从下电状态执行，单次测试一台普通旋转关节（不支持夹爪）：

```text
zero_cycle_test 00
```

交互测试期间按 `Ctrl+X`（无需回车）退出本次测试，避免与终端的 `q` 退出程序命令混淆。程序发送失能并尝试下电，保存已完成
轮次的 CSV、日志与汇总，报告 STOPPED 后返回电机终端，之后可重新开始测试。
运动、复测和反馈等待期间会轮询按键；正在执行已有上电、扫描等操作时，等当前操作返回再收尾退出。
此按键功能用于交互终端；非 TTY 输入不接管标准输入。Ctrl-C 仍保留原有退出程序行为。

默认读取 `config/zero_cycle_test.yaml`（相对当前工作目录）；也可执行
`zero_cycle_test 00 /路径/测试.yaml` 指定其他配置，配置路径支持 Tab 补全文件和目录。
配置中的 `gear_ratio` 必须等于实机固件的实际减速比，0 会拒绝运行。
参考 `/home/tim-dxt/bxi_ws/bxi_motor_source.zip` 的普通关节固件：MIT 目标与
`Encoder.position / GR * 2π` 比较，回复位置也由此计算，因此总线位置是输出端 rad，
上位机 MIT 打包本身不换算减速比。源码 `USE_ENC2=1` 的参考 GR 为 `7056/361`，
约 19.54570637；不能据此认定所有实机固件均相同。

测试配置角度和误差用电机端“圈”，下发时换算成输出端弧度。
`max_motor_turns` 最大 9，且必须严格小于减速比一半，以保持在输出端零位 ±半圈以内。
若单圈识别算法采用其他可识别区间，应按实际固件调整测试范围。
以参考减速比为例，电机端 ±9 圈约对应输出端 ±2.8932 rad。

首次上电仅发送一次置零，验证反馈接近零；随后分成 36 个等宽角度区间，每组随机打乱区间顺序，
逐区间随机取非固定角度，每 36 次覆盖所有区间。`seed` 可固定以复现目标序列。
按 `motor_turns_per_second` 平滑插值，每次都检查新鲜 MIT 回复，并等待位置和速度
满足稳定条件；目标到位后失能，再记录断电前的实际位置。默认断电间隔 `off_ms: 1000`，
包括已有 power_off 的 300ms 等待；上电启动、扫描和预热耗时额外计算。
重新上电后保持失能，以零增益 MIT 查询实际位置，与断电前位置直接比较，不做整圈取模。

`count` 和 `duration_s` 为次数与总时长限制，0 关闭对应限制，至少一个非零；
两者都配置时先达到的限制生效。时长在周期边界判断，不会中途截断一个上下电周期，
因此实际耗时可能超过配置时长一个周期。`move_timeout_ms` 限制每次运动和稳定等待。
`tolerance_motor_turns` 是电机端绝对误差，配置过小低于 MIT 量化能力会拒绝运行。
KP/KD 为原 MIT 输出端控制单位，需按待测电机负载设置。

位置恢复超差、通信超时或中断会停止测试并尝试下电；运动未到位按下述分级重试与当前位置兜底流程处理。CSV 写入
`/tmp/zero-cycle-<pid>-<time>.csv`，包含目标角度、断电前后输出端位置、
电机端误差及 PASS/FAIL。断电前已失能，假设断电期间机械位置不变；
外部负载造成的位移也会表现为位置不一致。
电源开关为整组共享，测试会影响所有连接的电机。此命令会修改待测电机的零位，
不自动恢复旧零位，不额外发送保存寄存器命令；是否持久化由固件置零命令决定。

新增输出端逐度扫描模式：

```text
zero_cycle_test 03 config/zero_cycle_degree_scan.yaml
```

首次置零后，按输出端 −1°、−2°……−175° 逐点执行上下电比较；随后以最高 5°/s 缓慢返回原零位，并再次上下电验证零位。验证通过才继续 +1°、+2°……+175°，共 350 个角度点。每个点默认连续上下电 10 次，期间保持失能，不重新运动；完成 10 次后才移到下一个角度。多电机模式还会始终与该点第一次断电前的基准比较。回零验证失败时，单台模式停止，多电机模式锁定为仅上下电。

```yaml
scan_repeats_per_angle: 10  # 每个角度上下电次数
scan_start_direction: negative  # negative 先负向；positive 先正向
scan_rounds: 1             # 完整扫描轮数；0 持续循环，Ctrl+X 停止
count: 0                   # 总上下电比较次数上限；0 不额外限制
duration_s: 0              # 总时长上限；0 不额外限制
```

一轮扫描指 −1°…−175°、回零验证、+1°…+175°，并非机械连续旋转 360°。比较次数为 `350 × scan_repeats_per_angle × scan_rounds`：每点 10 次、扫描 1 轮为 3500 次，3 轮为 10500 次。两轮扫描之间同样缓慢回零并上下电验证，再从 −1° 开始，不重新置零。回零验证、单台异常点额外复测不计入主比较次数。有限轮数完成后下电；`count`/`duration_s` 可提前结束，可能截断某个点的 10 次。仅逐度模式允许所有上限均为 0，以供持续循环。
起始方向优先级为命令行 `--direction positive|negative` > YAML `scan_start_direction` > 默认 `negative`。指定正向时每轮顺序改为 +1°…+175°、缓慢回零验证、−1°…−175°；多轮测试均沿用选定方向。方向参数仅适用于 `degree_scan`，不改变配置文件，也不自动将随机模式切换为逐度模式。开始时会显示最终方向及参数来源，CSV 同步记录。例如：

```text
zero_cycle_test 03 config/zero_cycle_degree_scan.yaml --direction positive
zero_cycle_group_test auto config/zero_cycle_degree_scan.yaml --direction positive
zero_cycle_group_test 0:2,1:1 config/zero_cycle_degree_scan.yaml --direction negative
```

配置使用 `test_mode: degree_scan`，`scan_step_output_deg: 1`、`scan_limit_output_deg: 175` 和 `scan_return_output_deg_per_second: 5` 分别控制间隔、单侧终点和回零速度。角度终点必须小于输出端半圈，且是间隔的整数倍；扫描范围独立于随机模式的电机端 ±9 圈限制。`count` 或 `duration_s` 仍可提前终止扫描。此模式不插入异常点回访；失败后的原地复测与恢复零位检查保留。

扫描配置将到位容差设为电机端 0.01 圈，以区分相邻 1° 测试点；上下电比较容差仍为 0.1 圈。未到位兜底的轮次会继续标明未到位，实际反馈位置保存在 CSV 中。CSV 另含 `target_output_deg` 目标输出端角度列。原配置 `config/zero_cycle_test.yaml` 使用 `test_mode: random`，保持随机测试。
逐度模式回零另用 `scan_return_tolerance_output_deg: 1`，允许输出端距原零位 ±1°（可配置范围大于 0、不超过 5°）。此容差同时用于回零运动、失能后读数、重启后的零位范围和编码器相位核对；上下电前后的差值仍按 `tolerance_motor_turns` 判断。普通角度点继续使用 `arrival_tolerance_motor_turns: 0.01`，不会因放宽回零而降低逐点测试精度。单台与多电机均适用，随机模式沿用原到位容差。

多电机同步测试可直接自动发现：

```text
zero_cycle_group_test
zero_cycle_group_test auto config/zero_cycle_degree_scan.yaml
```

不带参数时读取 `config/zero_cycle_test.yaml`；指定配置可切换逐度扫描等测试参数。命令使用终端启动时加载的机器人配置中的总线、ID、型号及各型号 MIT 范围，上电后主动探测全部已配置地址，输出每条总线的配置数量、在线数量及各台型号核对结果。自动选择所有在线且型号匹配的普通旋转电机，离线电机不加入，手部电机跳过。发现未知型号、型号不一致、无可测电机或超过 32 台时下电退出，不置零、不使能。扫描不探测配置之外的地址，因此在线数量是已配置地址范围内的数量。

发现阶段只扫描；随后下电并重新进入测试启动流程，再次核对本次上电的 `HW/PRO` 输出与配置型号，整组通过后才置零。缺少新鲜型号输出也会拒绝开始，可检查上电日志和 `power_on_wait_ms`。型号核对比较的是固件报告型号，并非硬件身份鉴定。减速比、KP/KD、测试角度和容差从测试 YAML 读取，不能仅凭 `motor_85` 等型号文本确定减速比；不同减速比应分组测试。

也可按机器人配置的实际 `bus:id` 手动选择（数字按现有终端解析规则；建议使用十进制，无前导零）：

```text
zero_cycle_group_test 0:2,1:1,1:2 config/zero_cycle_degree_scan.yaml
```

以上仅为选择格式示例，请替换为实际接线；必须在下电状态启动，最多选择 32 台，拒绝未知或重复地址及手部电机。全组共用同一份减速比、KP/KD、角度、速度和容差配置，因此所选电机必须适用这些参数。每台首次置零后执行相同目标角度；随机和逐度模式均支持。逐度模式在负向结束后全组缓慢回零、上下电验证，再开始正向。

全组 MIT 按约 10ms 周期轮询发送，先向所有成员发送再收集回复；回复必须同时匹配 bus 和 CAN ID。任一台通信超时、未确认使能、初始置零失败或运动超时，则终止并下电；多电机模式不执行单台模式的增益重试和未到位兜底。

任一台上下电位置超出 `tolerance_motor_turns` 后，全组锁定为仅上下电模式：不再使能、运动、置零或执行补偿，也不自动恢复旋转。剩余轮次仍受 `count`/`duration_s` 限制，`count` 是全组上下电轮数。持续使用首次失败轮次各台断电前的独立基准，避免把稳定的错误位置当作新基准；另外记录每次上下电自身的变化。每轮分别显示各台成功/失败累计和全组累计；全组所有电机一致才算该轮成功。

CSV 位于 `/tmp/zero-group-<pid>-<time>.csv`，每行对应一台电机一次比较，包含 bus、id、配置 index、阶段、目标角度、冻结基准及断电前后读数。每台上电文本单独保存为 `.csv.bus-<bus>.id-<id>.boot.log`；首次位置异常后额外发送只读 `p` 查询编码器诊断。中途按 Ctrl+X 停止并下电，返回终端。

零位测试默认 100 轮，隐藏运动进度及常规上下电/扫描/握手日志，只保留每轮目标、重启后的比较结果与错误；CSV 继续记录。
单台与多电机测试在开始、每轮结果及结束时显示本地日期时间（含时区）和累计耗时（小时/分/秒）；单台失败点复测也显示累计时间。CSV 注释和各台上电日志保存 `TIME` 记录及 `elapsed_s`，可用于事后对照。累计耗时通过单调时钟计算，包含上电、运动、断电等待、诊断和复测；自动多电机模式还包含初始扫描与型号核对。系统时间校正不会使累计耗时跳变。
每轮结果后显示本次运行累计成功、失败和总比较次数，并实时写入 CSV 注释及 boot.log 的 RUNNING_TOTAL。
该累计包含随机点与异常点回访；原地10次复测、原零位重启验证仍分别记录，不混入主轮次累计。

当上下电位置差超限时，随机测试停止推进，并在同一个机械位置额外复测 10 次。
复测只做上下电和零增益 MIT 位置查询，不发送置零、使能或运动命令。
每次读数都与失败轮断电前的原始位置比较（不随复测结果更新基准），即使仍然错圈也继续采集。
读取失败会记录 ERROR 并继续下一次；下电失败或用户中断时提前停止。
复测结束后保留原失败记录；默认通过软件坐标补偿返回原零位继续测试，设置 continue_after_position_failure=0 时下电退出。

日志保存在原 CSV 路径追加 `.cycle-N.retries.log` 的文件中，包含失败点目标、原基准、
10 次重新上电的电机端圈数/角度、误差和 PASS/FAIL/ERROR、完成及有效读取数量。
另外按复测次数分段记录目标电机上报的连续 CAN 启动文本；
终端静默不会屏蔽这份采集。采集窗口覆盖每次上电/扫描/查询及查询后额外 1 秒。
仅记录该电机实际经 CAN 上报的输出，不包含固件未发送到 CAN 的 UART 或内部变量。

每轮断电前和重启后发送只读菜单字符串 `p`，比较 ENC1 原始码/电机端位置与
ENC2 角度/输出端位置，写入 `.csv.boot.log`；失败点复测也采集并解析这些字段。
日志重组工具、字段单位和固件原始角度诊断补丁见 [编码器错圈诊断](docs/firmware/encoder_diagnostics.md)。

零位测试使能时现在持续发送零增益 MIT 保活，并通过只读菜单字节 `0xC8` 获取新鲜
`STATE : 02 MOTOR` 状态后才开始轨迹。不再把任意 CAN 回复当作使能成功，也不在使能后
固定停止通信一秒。2 秒内无法确认 MOTOR 时停止、失能并保存诊断，不自动清故障或校准。

单次 MIT 反馈总等待保持 100ms，期间每 10ms 重发相同目标（最多 10 帧），回复解析规则不变。
避免一帧缺回复后停止发送长达 100ms；重发后恢复记录 `MIT_RECOVERED`。
持续缺少合法反馈时停止，不能用当前位置兜底掩盖通信故障。
超时会输出预期 CAN ID、数据首字节及收包计数，并在 `.boot.log` 写入 `MIT_TIMEOUT`、
`MIT_TX` 及最多 8 帧 `MIT_RX` 原始数据，便于区分无回复、回复 ID/长度不符或解码失败。

运动改为五次平滑轨迹，同时发送匹配的目标速度；`motor_turns_per_second` 为轨迹峰值速度上限，
`motor_acceleration_turns_per_second2` 为加速度上限（默认 20 圈/秒²），起止速度、加速度均为零。
原先移动位置而目标速度一直为 0，会让 KD 对运动产生额外阻尼；匹配速度可降低这部分跟踪滞后。
到位不再复用上下电一致性容差：

- `arrival_tolerance_motor_turns`：运动到位位置容差，默认 0.1 电机圈。
- `settle_speed_motor_turns_per_second`：稳定速度阈值，默认 0.1 电机圈/秒，考虑量化下限。
- `tolerance_motor_turns`：仅用于初始置零确认和上下电位置一致性判断。

每秒运动细节写入 `.boot.log` 的 `MOTION` 行，不刷终端；到位超时打印目标、实际位置、
误差和速度，失能后采集 `MOVE_FAILURE_AFTER_DISABLE` 系统信息。
电机完全不动时先检查 `STATE`、故障/校准标志和母线电压，不能仅靠提高 KP 处理。


### 未到位的分级重试与当前位置兜底

轨迹结束后等待 `arrival_wait_ms`（默认 5000ms）；尚未满足位置和速度稳定条件时，
终端输出目标、实际位置、误差及速度，再按 `retry_gain_factor`（默认 1.5）提高 KP/KD。
最多 `move_retry_count`（默认 2）次，每次观察 `retry_hold_ms`（默认 3000ms），
增益用 500ms 平滑切换，不改变本轮目标、不注入额外前馈转矩。
KP/KD 受 `retry_kp_max: 100`、`retry_kd_max: 7.5` 及电机协议上限共同约束。
例如 KP=50 时两次为 75、100；KD=5 时，协议上限允许则到 7.5，否则保持在协议上限。
这些值是可配置的测试增益，不能保证克服机械卡滞、故障或不正确的标定。
`move_timeout_ms` 仍是整段运动含重试的总时限，达到后不再继续加增益。

重试耗尽仍未到位时，保持当前位置并用原始 KP/KD 等待最多 5 秒，确认位置和速度稳定后，
失能并重新读取实际位置作为断电基准，继续本轮上下电零位识别测试。
如果当前位置也不能稳定，或通信失败、使能未确认、用户中断，则停止并下电，不进行兜底比较。
原先到达的目标和实际测试位置均保留；每轮显示目标到位与否和增益重试次数，
CSV 新增 `target_reached`、`gain_retries` 两列，汇总显示 `current_position_fallbacks`。
`PASS` 只表示上下电位置一致，不能表示随机目标已达到或已覆盖。


### 错圈补偿、异常点回访与全程汇总

默认 `continue_after_position_failure: 1`。每次上下电位置不一致时，先在原位置完成 10 次
固定点复测，然后读取最新固件位置，以“断电前的原坐标位置在原地未变”为前提，计算软件坐标偏移：

```text
原坐标位置 = 固件反馈位置 - 软件偏移
新的软件偏移 = 复测后的固件反馈位置 - 失败轮断电前的原坐标位置
后续 MIT 目标 = 原坐标目标 + 软件偏移
```

例如原坐标为 +6.235 电机圈、重启后固件读数为 -6.765 圈，则偏移为 -13 圈。
回原零位时发送固件目标 -13 圈，运动位移为 -6.235 圈。不会向驱动发送 ZERO、保存或修改编码器零偏。
返回原零位必须真正到位，不使用当前位置兜底；失能后再次读取位置和 `p` 编码器数据。
可用的 ENC1/ENC2 原始相位若与首次零位矛盾则停止，无法到位或通信异常也停止。
原坐标的估计依赖断电期间无机械位移、运行中错位表现为固定坐标偏移等前提，软件补偿不代表修复固件。
原坐标目标仍限制在 ±max_motor_turns；偏移后的命令还必须落在 MIT 编码范围内，越界则停止，绝不截断或取模。

默认 `revisit_after_random_cycles: 3`：记录异常实际位置后，继续三个随机点的上下电测试，
再轮流返回已记录异常点，按相同流程到位、读取、上下电并判断是否复现。
回访指向的是原先实际断电位置，不是当时未完全到达的随机目标；没有回到该点会单独标记，
不计为同点复现。相邻异常点以 arrival_tolerance_motor_turns 为分组容差，最多记录 128 个点。
随机目标使用独立序号，因此插入回访不会跳过原先的随机区间覆盖。
次数/时长上限包含随机点与回访轮次；10 次原地重启是独立诊断统计；上限可能使某些点来不及回访。
即使继续流程执行完，只要发生过零位丢失，最终仍报告 COMPLETED_WITH_POSITION_FAILURES，不能变成全程 PASS。

结束或中止时打印并保存 `.csv.summary.csv`：总比较次数、成功/失败、运动兜底、每个异常点的
原坐标和角度、记录后的比较/失败次数、安排返回/返回复现/返回未到位次数，以及原地 10 次重启的独立统计。
主 CSV 增加软件偏移、原坐标断电前后位置、random/revisit 类型、实际/请求异常点编号、
revisit_hit 及 ENC2 原始角度；缺失原始角度写 NaN，不会用校准角度替代。

要对整场测试所有轮次按实际位置和原始角度统计分布（包括异常点被记录之前的成功轮次）：

```bash
python3 tools/summarize_zero_cycles.py /tmp/zero-cycle-<pid>-<time>.csv
```

生成 `.csv.analysis.txt`，按原坐标电机位置和断电前 ENCO 原始角度分箱，列出失败次数/比较次数、
回访失败复现次数、最大错位及对应轮次。默认位置分箱 0.1 电机圈、原始角度分箱 1°，
可用 `--motor-bin-turns` 与 `--enco-bin-deg` 调整；分箱相同不代表两个角度完全相同。
旧 CSV 无原坐标字段时需要显式传入 `--gear-ratio`；旧固件无 ENCO 原始角度时该维度无法统计。
只分析一场测试，避免把不同初始零位的多场原坐标混在一起。

恢复到原零位后，现在必须额外上下电一次验证，不能直接带旧软件偏移进入下一随机点。
本次检查以断电前的补偿坐标为原位置参考，同时要求重启后的固件原始位置本身接近 0，
以及两者差值不超过 tolerance_motor_turns；可用的原始编码器相位也不能与初始零位矛盾。
不会通过更新偏移把这次检查改成 PASS。只有验证通过才清除旧软件偏移，再继续随机/异常点回访。
验证未通过或无法读到位置时，状态标记 PAUSED_ZERO_RECHECK_FAILED，失能、下电，
保存日志和汇总并返回终端，等待人工处理；不会自动开始下一轮，也不自动重新置零。
该额外零位验证单独记录在 boot.log 的 ORIGINAL_ZERO_REBOOT_RESULT 及主 CSV 注释中，
不混入随机点/回访轮次成功计数。暂停后不提供自动原地续跑命令，重新启动测试会执行初始置零流程。
