# BXI PCI 电机统一控制终端

本项目提供 BXI PCIe CAN-FD 控制卡的用户态测试、调试和电机固件烧录工具。

推荐使用 `motor_console`。它把电机上下电、在线扫描、MIT 控制、状态管理、CAN
统计和固件烧录集成在同一个交互终端中。原有的 `motor_test`、`motor_flash` 和
`drv_test` 仍然保留，方便单项调试和兼容已有脚本。

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
build/motor_console   推荐使用的统一交互终端
build/motor_test      原有电机调试工具
build/motor_flash     原有电机烧录工具
build/drv_test        原有板卡测试工具
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
sudo ./build/motor_console --config /path/to/motor_firmware.yaml
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

命令名称在中英文模式下保持一致，方便脚本和操作记录复用。Bootloader 和 YMODEM
底层协议的原始回显保持固件端原文，便于定位升级问题。

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
3. 向配置中的全部电机发送安全探测帧。
4. 输出收到回复的电机逻辑序号和解析后的反馈。
5. 按 Bus 输出发送失败及回复超时警告。

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
mit_zero_set_single 0
```

零位校准要求目标电机处于失能状态。如果软件记录该电机仍然使能，命令会被拒绝。

### 7.3 使能和失能

```text
mit_enable_all
mit_disable_all

mit_enable_single 0
mit_disable_single 0
```

只有收到电机回复后，软件才会把该电机记录为已使能或已失能。可以通过
`motor_list` 检查状态。

### 7.4 单电机 MIT 控制

```text
motor_set <index> <pos> <torque> <vel> <kp> <kd>
```

示例：控制 `index 0` 到位置 `0.5`，目标速度和前馈转矩为 0：

```text
motor_set 0 0.5 0 0 10 1
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
直接执行 `quit` 会被拒绝。

## 8. CAN 输出和统计

实时输出默认由配置控制：

```yaml
live_output: on
```

终端内切换：

```text
can_monitor on
can_monitor off
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

默认目录和文件映射：

```yaml
firmware_dir: firmware

firmware_files:
  50: motor_50.bin
  50L: motor_50L.bin
  70: motor_70.bin
  85: motor_85.bin
```

对应文件应放在：

```text
firmware/motor_50.bin
firmware/motor_50L.bin
firmware/motor_70.bin
firmware/motor_85.bin
```

### 9.2 烧录单台电机

```text
power_on
mit_disable_all
flash_single <index>
```

示例：

```text
flash_single 0
```

### 9.3 烧录全部电机

```text
power_on
mit_disable_all
flash_all
```

烧录前程序会预检所有目标及固件文件。任何文件缺失时，整个批次会在操作第一台电机
之前停止。

### 9.4 进入 Boot 的流程

默认流程：

1. 探测电机是否已经位于 Boot 菜单。
2. 如果不在 Boot，向应用程序发送复位命令 `r`。
3. 在 Boot 窗口连续发送 `m`。
4. 收到 `Commands:` 后确认进入 Boot。
5. 发送 `u` 并通过 CAN/YMODEM 传输固件。

如果软复位无法进入 Boot，可以增加 `cycle`：

```text
flash_single 0 cycle
flash_all cycle
```

`cycle` 会操作电机总电源，使用前应确认其他机构处于安全状态。

## 10. 配置文件

默认配置文件：

```text
config/motor_firmware.yaml
```

显示当前配置：

```text
config_show
```

下电状态下重新加载：

```text
config_reload
config_reload /path/to/config.yaml
```

配置包含：

- 界面语言。
- 电机逻辑序号、CAN Bus、ID 和型号。
- 固件目录和四种型号的固件文件名。
- 上电等待及扫描超时。
- 实时 CAN 输出和 CAN-FD 开关。
- MIT 位置、速度、转矩、KP 和 KD 范围。
- 全部电机软启动回零参数。

当前电机拓扑：

| 位置 | CAN 范围 | Index 范围 |
|---|---|---|
| 腰部 | `bus0/id1-3` | `0-2` |
| 左腿 | `bus1/id1-6` | `3-8` |
| 右腿 | `bus2/id1-6` | `9-14` |
| 左手 | `bus3/id1-7` | `15-21` |
| 右手 | `bus4/id1-7` | `22-28` |
| 头部 | `bus0/id4-5` | `29-30` |

## 11. 命令速查

| 命令 | 参数 | 说明 |
|---|---|---|
| `help` / `-h` / `?` | 无 | 显示终端帮助 |
| `language` / `lang` | `zh\|en` | 切换中英文 |
| `power_on` | 无 | 上电、等待并扫描全部电机 |
| `power_off` | 无 | 失能并关闭总电源 |
| `motor_scan` | `[timeout_ms]` | 扫描全部配置电机 |
| `motor_list` | 无 | 显示电机状态和最后反馈 |
| `mit_zero_set` | 无 | 全部电机校准零位 |
| `mit_zero_set_single` | `<index>` | 单电机校准零位 |
| `mit_enable_all` | 无 | 使能全部电机 |
| `mit_disable_all` | 无 | 失能全部电机 |
| `mit_enable_single` | `<index>` | 使能单台电机 |
| `mit_disable_single` | `<index>` | 失能单台电机 |
| `motor_set` | `<index> <pos> <torque> <vel> <kp> <kd>` | 单次 MIT 控制 |
| `stand_up` | 无 | 全部在线电机软启动回零 |
| `flash_single` | `<index> [cycle]` | 烧录单台电机 |
| `flash_all` | `[cycle]` | 烧录全部电机 |
| `can_status` | `[reset]` | 显示或清零 CAN 软件统计 |
| `can_monitor` | `on\|off` | 开关实时 CAN 输出 |
| `config_show` | 无 | 显示当前配置 |
| `config_reload` | `[path]` | 重新加载配置 |
| `quit` / `exit` / `q` | 无 | 退出终端 |

## 12. 原有工具

原有工具仍可单独使用：

```bash
sudo ./build/motor_test --help
sudo ./build/motor_flash --help
sudo ./build/drv_test
```

涉及完整机器人操作时优先使用 `motor_console`，避免不同程序分别维护电源和使能状态。
