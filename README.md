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

该命令要求配置中的全部电机都在线且已使能。程序将：

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
| `reg_write` | `<index00\|all> <reg_index> <value> [wait_ms]` | 写入寄存器；`all` 按默认烧录配置逐台发送；要求没有已知使能电机 |
| `reg_save` | `<index00\|all> [wait_ms]` | 保存寄存器配置；`all` 按默认烧录配置逐台发送；要求没有已知使能电机 |
| `reg_info` | `<index00\|all> [wait_ms]` | 读取该电机支持的寄存器总数和可写数量 |

寄存器命令会根据上电识别到的程序/配置版本，自动选择 0.x 旧版或 1.x 新版寄存器表：
`reg_read` 使用 `CAN_CMD_REG_READ=0x17`，`reg_write` 使用
`CAN_CMD_REG_WRITE=0x18`，`reg_save` 使用 `CAN_CMD_REG_SAVE=0x19`，
`reg_info` 使用 `CAN_CMD_REG_INFO=0x1A`。寄存器地址范围是 `0x00..0xFF`。
`reg_read` 请求 `data[0..3]=address`；`reg_write` 请求
`data[0..3]=address | (value_type << 8)`、`data[4..7]=value_raw`。
`reg_write` 会根据寄存器地址自动填充 `value_type`，回复会解析
`status`、`value_type` 和 value；`value_type` 使用 `bit8..10`，
目前 `0=int`、`1=bool`、`2=float`、`3=uint32`、`4=version`。
`reg_info` 回复的 `value_raw` 低 16 位是寄存器总数，高 16 位是可写数量。
如果某台电机尚未识别到版本信息，寄存器命令会拒绝向该电机发送，
避免使用错误寄存器表误写配置。

常用寄存器示例：

```text
reg_read 00 0x7c     # 读取配置版本
reg_read 00 0x6d     # 读取 MIT AUX 轮询开关，0=旧版 NTC，1=AUX 轮询
reg_write 00 0x6d 1  # 本次上电周期启用 MIT AUX 轮询回复
reg_write 00 0x6d 0  # 切回旧版 NTC 回复
reg_read 00 0x01     # 读取 motor_pole_pairs
reg_write 00 0x01 10 # 写入 motor_pole_pairs
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
| `flash_debug` | `<index00>\|<bus0-4 id0-7> <version\|firmware.bin> [cycle]` | 两位数按 index；一位 bus+一位 id 按 bus/id 调试烧录 |

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
| `console_read_motor_register` | `src/lib/debug.c` | `can_dbg` 前置读取 `0x6d mit_aux_enable` |
| `console_scan_online_versions` | `src/lib/control.c` | `power_on` 后读取在线电机版本，并缓存到运行时状态 |
| `reg_config_meta_for_motor` | `src/lib/runtime.c` | 根据电机版本选择 0.x/1.x 寄存器名称和 `value_type` |
| `reg_value_type_from_wire` | `src/lib/runtime.c` | 从 `reply_header` 解析 `value_type` |
| `reg_request_header` | `src/lib/runtime.c` | 生成写寄存器请求头：`address | value_type << 8` |
| `reg_reply_status` | `src/lib/runtime.c` | 从 `reply_header` 解析寄存器返回状态 |
| `reg_reply_value_type` | `src/lib/runtime.c` | 从 `reply_header` 解析 `bit8..10` 的 `value_type` |
| `print_register_value` | `src/lib/runtime.c` | 按 `value_type` 打印 int/bool/float/uint32/version |
| `print_register_debug_frame` | `src/lib/runtime.c` | `reg_read/reg_write/reg_save/reg_info` 回复的主要解析和输出函数 |
| `console_reg_send_one` | `src/lib/control.c` | `reg_read/reg_write/reg_save/reg_info` 对单台电机的实际发送函数 |
| `console_parse_register_value_arg` | `src/lib/control.c` | `reg_write` 输入值解析，并按寄存器类型转成 32 位 raw value |
