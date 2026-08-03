# BXI PCI 电机统一控制终端

本项目提供 BXI PCIe CAN-FD 控制卡的电机统一控制终端。

唯一对外程序为 `motor_console`。它把电机上下电、在线扫描、MIT 控制、状态管理、
CAN 统计和固件烧录集成在同一个交互终端中。

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

默认语言为英文，因此默认提示符：

```text
motor[POWER-OFF]>
```

电机上电后提示符变为：

```text
motor[POWER-ON]>
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

终端默认语言为英文，由配置文件决定：

```yaml
language: en
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
```

查看中文帮助：

```bash
./build/motor_console --language zh --help
```

终端内查看帮助：

```text
help
-h
?
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
5. 如果被动监听没有发现电机，再进入主动扫描；空总线可能产生 CAN TX error。

典型输出：

```text
power_on: start 电源已开启 wait=2s
power_on: done total=31 success=31 failed=0
```

如果有电机未回复，只会额外输出未回复的电机，并按照烧录配置中的 `name` 标出关节位置：

```text
power_on: start 电源已开启 wait=2s
[motor08]: offline name=左腿 bus=1 id=6
[motor29]: offline name=头部 bus=0 id=4
power_on: done total=31 success=29 failed=2
```

也可以手动重新扫描：

```text
motor_scan
motor_scan 1500
```

### 7.2 校准零位

给全部电机发送 MIT 零位校准帧：

```text
mit_zero_set
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
motor_set <index00> <pos> <torque> <vel> <kp> <kd>
```

示例：控制 `index 00` 到位置 `0.5`，目标速度和前馈转矩为 0：

```text
motor_set 00 0.5 0 0 10 1
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

下载文件名由 `config/motor_console.yaml` 中 `motor_types.firmware` 决定。
例如：

```yaml
motor_types:
  - type: "50L"
    firmware: bxi_motor_50L.bin
```

当烧录配置里写 `version: 50L` 时，会下载：

```text
http://your-download-server/motor/bxi_motor_50L.bin
```

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

`flash_single` 不写固件参数时，使用 `config/flash_plan_default.yaml` 中该 index
对应的 `version`。如果程序启动时远程固件全部拉取成功，会优先使用
`firmware_cache_dir`；否则使用 `firmware_dir`。`50L` 会自动解析为
`motor_types.firmware` 中配置的文件名。

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
- `firmware_base_url` 可选；配置后，程序启动时会尝试把默认烧录计划所需固件下载到
  `firmware_cache_dir`。
- `version` 支持型号简写，例如 `50L` 会下载/查找 `motor_types.firmware`
  中配置的文件名。
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
之前停止。

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

终端里可以执行 `help` / `-h` / `?` 查看推荐命令。实际测试时建议优先使用下面这些常用命令。

### 11.1 常用操作

| 命令 | 参数 | 说明 |
|---|---|---|
| `help` / `-h` / `?` | 无 | 显示终端帮助 |
| `power_on` | 无 | 电机上电、等待软启动、扫描全部电机 |
| `power_off` | 无 | 失能已知使能电机并关闭总电源 |
| `motor_list` | 无 | 显示电机 index、bus、id、型号、在线/使能状态和最后反馈 |
| `motor_scan` | `[timeout_ms]` | 重新扫描配置电机 |
| `can_status` | `[reset]` | 显示或清零 CAN 软件统计 |
| `quit` / `exit` / `q` / `qq` | 无 | 退出终端；电源开启时会先自动下电 |

### 11.2 MIT 控制

| 命令 | 参数 | 说明 |
|---|---|---|
| `mit_zero_set` | 无 | 给全部电机发送 MIT 零位校准帧 |
| `mit_zero_set_single` | `<index00>` | 给单台电机发送 MIT 零位校准帧 |
| `mit_enable_all` | 无 | 使能全部电机 |
| `mit_disable_all` | 无 | 失能全部电机 |
| `mit_enable_single` | `<index00>` | 使能单台电机 |
| `mit_disable_single` | `<index00>` | 失能单台电机 |
| `motor_set` | `<index00> <pos> <torque> <vel> <kp> <kd>` | 给单台电机发送一次 MIT 控制帧 |
| `stand_up` | 无 | 全部在线且已使能电机软启动回零 |
| `reg_read` | `<index00\|all> <reg_index> [wait_ms]` | 读取寄存器；`all` 按默认烧录配置逐台发送 |
| `reg_write` | `<index00\|all> <reg_index> <value> [wait_ms]` | 写入寄存器；`all` 按默认烧录配置逐台发送；要求没有已知使能电机 |
| `reg_save` | `<index00\|all> [wait_ms]` | 保存寄存器配置；`all` 按默认烧录配置逐台发送；要求没有已知使能电机 |

寄存器命令按底层电机程序的 CAN 配置协议发送：
`reg_write` 使用 `CAN_CMD_SET_CONFIG=0x11`，`reg_read` 使用
`CAN_CMD_GET_CONFIG=0x12`，`reg_save` 使用 `CAN_CMD_UPDATE_CONFIGS=0x13`。
读写回复会打印寄存器地址和 raw/uint/int/float 四种值。

### 11.3 烧录

| 命令 | 参数 | 说明 |
|---|---|---|
| `flash_single` | `<index00> [version\|firmware.bin] [cycle]` | 烧录单台；可省略固件使用默认 version |
| `flash_all` | `[plan.yaml] [cycle]` | 按烧录配置烧录多个目标；省略配置时使用默认配置 |
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
| `src/lib/debug.c` | 单电机直通调试 |
| `src/lib/terminal.c` | 命令分发、配置重载、交互终端循环 |
| `src/lib/bxi_motor_comm.c` / `src/lib/bxi_motor_comm.h` | MIT 帧打包、特殊命令帧、回复解析和默认参数范围 |
