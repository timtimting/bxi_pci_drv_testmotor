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

`build/motor_console` 现在是 `run_motor_console.sh` 的启动副本：每次启动前自动
检测 EtherLab 开发库、增量构建，并检查内核主站；若未启动，则调用 `/etc/init.d/ethercat start`，再运行内部程序。
通过 `sudo` 启动时，构建由原登录用户完成。
启动脚本用醒目的“EtherLab 启动检查与编译：开始／完成／失败”分隔线标识构建输出。
更新启动脚本后需先执行一次 `make`，将新脚本复制到 `build/motor_console`；此后只需上述启动命令。

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

生成以下启动入口：

```text
build/motor_console   统一交互终端启动入口
```

实际可执行文件位于 `build/internal/motor_console`，由启动入口调用，不建议直接运行。

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
- 终端常规复制、粘贴快捷键，例如 `Ctrl+Shift+C`、`Ctrl+Shift+V`。程序支持 bracketed paste；粘贴内容只进入当前输入行，换行转为空格，不会自动执行命令，仍需检查并按 `Enter`。快捷键由外部终端决定；若按下后完全没有输入，请检查终端按键绑定或使用 `Shift+Insert`/右键粘贴。
- `Ctrl+C`：中止当前操作并退出；如果软件记录的电机电源仍开启，会尝试安全下电。

### 命令输出时间戳

程序日志逐行添加 `[cmd=编号 +秒.毫秒s]`，例如：

```text
[cmd=1 +0.000s] ethercat_enable: 已提交后台队列（不代表执行成功）
[cmd=1 +0.012s] ethercat_enable: 正在读取使能前 SDO 配置
[cmd=1 +0.420s] ethercat_enable: 正在等待 SAFE-OP，最长 7s
[cmd=1 +1.205s] ethercat_enable: SAFE-OP 已确认
```

以上耗时仅为格式示例。交互命令以按回车后进入命令分发的时刻为零点，不是网卡实际发帧时刻；
独立命令行模式以开始解析启动参数为零点。采用单调时钟，不受系统校时影响，命令编号在本进程内递增。
时间在日志产生时采集，不在终端显示时采集；后台排队等待包含在耗时内。
每条后台命令保留自己的编号及起点，新命令不会重置旧命令的日志时间；定时失能及运行中故障日志
关联该从站最近一次成功提交的使能/位置控制。多次打印拼成的一行只加一次前缀。
提示符、键盘回显和补全不加前缀；无独立命令上下文的 CAN 接收日志沿用最近一次命令的起点。
本功能覆盖本程序生成的标准输出/错误文本；底层库直接输出及采集到的原始内核日志保留原格式。
EtherCAT 日志仍使用有界异步队列，不在 PDO 循环中等待终端输出。

交互终端中，命令输出结束后自动恢复提示符；后台日志输出时先移开提示符，
本批完整日志输出后再显示，例如 `电机[已上电]> `。提示符按当前电源状态及语言显示，
不会固定为“已上电”。正在编辑的命令及光标位置会保留；提示符不带时间戳。
后台命令仍可排队，不会为了等待执行完成而阻塞输入；提示符仅表示可输入，不表示使能或运动成功。
独立命令行模式及非 TTY 输出不启用后台提示符重绘。

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

## 开璇 EtherCAT ESI

厂商 ESI 文件位于 `assets/ethercat/KaiserDrive_KDE_ECAT_V1.2.xml`，已纳入 Git
跟踪。`motor_console` 启动时会读取该文件并打印 Vendor ID、Product Code 和
Revision，用于确认项目内的 ESI 资产可用；该读取不会初始化 EtherCAT 主站，也不会
向电机发送任何 EtherCAT、CAN 或 MIT 控制命令。

EtherCAT 命令通过 EtherLab (IgH) 主站运行，使用配置的 master 0。运行前需安装
EtherLab 主站内核模块和用户态开发库，并在 `/etc/ethercat.conf` 中将
`MASTER0_DEVICE` 绑定到 `enp86s0` 的 MAC 地址；`ethercat_*` 命令仍保留网卡参数，
程序会将其 MAC 与 EtherLab master 0 的绑定进行校验。

### EtherCAT 扫描

编译时检测到 EtherLab 后，以下独立命令只读扫描从站，不配置 PDO，也不发送使能或运动指令：

```bash
sudo ./build/motor_console --ethercat-scan enp86s0
```

所有 `--ethercat-*` 命令均通过 EtherLab master 0 访问网卡上的实际 EtherCAT 从站；不会读取
`config/motor_console.yaml` 中的 CAN 电机配置，也不会初始化 BXI PCI/CAN。电机身份和
拓扑序号以从站 EEPROM 返回的 Vendor ID、Product Code、Revision 和 `slave N` 为准。

终端交互命令默认使用网卡 `enp86s0`，所以可直接输入 `ethercat_scan`；需要换网卡时将网卡名放在命令最后。
交互扫描列出拓扑后，会为检测到的开璇从站配置 PDO/DC，等待 SAFE-OP 和 OP，再以约 1 kHz 持续交换 PDO；
扫描完成预热或报错后才返回终端提示符；等待期间输入的内容不会执行。报错会区分 OP 等待与 CiA402 失能确认，并打印 AL/WKC/PDO 状态。
OP 最长等待 2 秒；如果仍失败，请提供该次扫描的分阶段输出和从站状态，不能仅凭旧的合并报错确定具体原因。
全程保持控制字 `0x0000` 和伺服失能，不发送使能或运动指令。首次扫描因此仍可能等待约 5 秒，
但预热成功后再执行 `ethercat_enable 1` 可复用后台主站，避免重复等待 SAFE-OP。
若拓扑含非开璇从站，交互扫描仅显示拓扑并跳过预热；独立 `--ethercat-scan` 始终只读。
预热使用默认 `Sync0=1 ms`、`shift=0`；若需其他周期或相位，应在扫描前直接使用带参数的 `ethercat_enable`。
旧的“网卡在前”格式仍兼容。独立命令行方式不会初始化 BXI PCI/CAN，
更适合只连接 EtherCAT 电机的电脑。启动前自动检测 `/usr/local/etherlab`、
`/opt/etherlab` 或系统路径，缺少开发库时直接报错，不会运行不支持 EtherCAT 的程序：

```bash
sudo ./build/motor_console
```

交互终端中执行 `power_on` 后可输入 `ethercat_scan`。启动入口也支持直接传入命令行参数，
例如 `sudo ./build/motor_console --ethercat-scan enp86s0`。若 EtherLab 安装在其他路径，
可用未提权的脚本入口指定前缀：

```bash
ETHERLAB_ROOT=/custom/etherlab ./run_motor_console.sh
```

脚本会在 `/dev/EtherCAT0` 或主站模块缺失时启动 EtherLab 服务；不会安装 EtherLab、修改主站网卡配置或给电机上电。
`--check-config` 和 `--help` 不会启动主站。若服务启动后仍不可用，脚本会报错退出。
若仍需手动构建，可指定安装前缀：

```bash
cmake -S . -B build -DETHERLAB_ROOT=/opt/etherlab
cmake --build build -j
```

开璇 EtherCAT 力位混合控制（《KGU系列谐波减速模组说明_EtherCAT版_V1.2》，混合力控章节）：

```text
mit <slave_id|all> <pos_rad> <vel_rad_s> <tff_Nm> <kp> <kd> [hold_ms] [network_interface]
info <slave_id|all> [network_interface]
```

`ethercat_mit` 与 `mit` 相同，`info` 与 `ethercat_info` 相同。五个控制参数均为输出端物理量：

| 参数 | 含义 | 单位 |
| --- | --- | --- |
| `pos_rad` | 输出端目标绝对位置 | rad |
| `vel_rad_s` | 输出端目标速度 | rad/s |
| `tff_Nm` | 前馈力矩 | N·m |
| `kp` | 位置误差增益，必须非负 | N·m/rad |
| `kd` | 速度误差增益，必须非负 | N·m/(rad/s) |

控制律为 `目标力矩 = (目标位置 - 实际位置) * kp + (目标速度 - 实际速度) * kd + tff`。
`hold_ms` 为 0 或省略时持续控制；1..60000 时到期失能。网卡默认沿用 EtherCAT 命令默认值。
手册建议从 KP=50、KD=10 开始调试，具体增益和目标需按实际机构选择。
例如 `mit 1 0.0 0.0 0.0 50 10 1000` 会控制从站 1 到输出端绝对零位置，持续 1 秒。

MIT 会话以 `0x6060=10` 和专用 PDO（输出 23 字节、输入 16 字节）启动，目标对象为
`0x4010..0x4014`，反馈对象为 `0x4000..0x4002`。位置不再通过 Pn051 换算。
首次使用时，应在新启动的终端中先执行 `mit`；如果已经执行过 `ethercat_scan`、CSP 或 PP
并建立了后台会话，请退出终端再启动 MIT。MIT 与 CSP/PP 之间切换也需要退出重启终端。
同一 MIT 会话内可以更新目标、失能、再次执行 MIT 使能，退出或 Ctrl-C 会失能。
所有开璇从站使用该会话的 PDO 格式，只有选中的从站接收使能和目标。

`info` 显示 Pn082（SDO `0x2082:00`，INT16）原值及 `原值 × 0.1 A/N·m`。
会话内该参数为初始化时的 SDO 快照，Pn082 写入回读后更新缓存；实时值可通过 `ethercat_pnread 1 82` 查询。
手册要求先标定转矩系数并写入 Pn082，否则力矩反馈不精确；MIT 命令不会自动修改 Pn082。
手册描述默认值为 10，但随附 ESI 默认值为 0，应以实际读取值和标定结果为准。

修改转矩系数可用 `ethercat_pn082`，或简写 `pn082`：

```text
ethercat_disable 1
pn082 1 10
info 1
# 如需掉电保存，再执行：
ethercat_save 1
```

语法为 `ethercat_pn082 <slave_id|all> <原始值:0..32767> [network_interface]`。
原始值按 INT16 非负范围校验，单位为 0.1 A/N·m；例如 10 为 1 A/N·m，25 为 2.5 A/N·m。
这里的范围是对象的非负编码范围，实际标定值以驱动器及模组要求为准。
目标从站须确认失能，命令先读取原值、写入 `0x2082:00` 的两字节小端整数，再回读验证，
成功后更新 `info` 缓存。命令不会自动使能或保存；写入失败时以后台结果和回读值为准。
独立命令行也支持 `sudo ./build/motor_console --ethercat-pn082 enp86s0 1 10`。

原 CAN MIT 控制继续使用 `mit_set <index00> <pos> <torque> <vel> <kp> <kd>`；
交互命令 `mit` 现在专用于开璇 EtherCAT，参数顺序按上述格式使用。

交互终端命令例子：`ethercat_info 1`、`ethercat_pn077 1 0`、`ethercat_save 1`、
`ethercat_enable 1`。如需选用别的网卡，将网卡名放最后，例如 `ethercat_info 1 enp5s0`；
使能命令可用独立命名参数 `sync0_cycle_ms=<周期>` 配置周期，支持 `0.5/1/2/4/5/8 ms`；
主站 Sync0 与 EtherCAT 循环会一起使用该周期。`hold_ms`、`sync0_shift_ns` 仍为可选位置参数，网卡覆盖参数放在最后。

若未检测到 EtherLab，直接运行 `make` 仍可能构建不含 EtherCAT 的程序，但会给出警告；
启动入口会停止并提示安装开发库。`--check-config` 和 `--help` 不要求 EtherLab。
master 0 必须在系统层绑定到命令指定网卡。
`ethercat_enable` 会在使能尝试期间临时启用 EtherLab `debug=1`；失败时把本次尝试的内核日志
保存到 `/tmp/ethercat-enable-failure-*.log`，成功时删除临时日志并恢复 `debug=0`。此功能需要以
root 运行且允许读取 `/dev/kmsg`；EtherLab 命令不在默认路径时，可通过 `ETHERLAB_CLI` 指定其绝对路径。
周期 PDO 先处理上周期回包，再发送本周期数据；进入 SAFE-OP 后最多等待 250 ms，只有连续 3 个周期获得完整 WKC 才继续使能。
EtherLab 的 LRW 回包可能覆盖域缓冲区中的输出字节，因此控制字和目标位置保存在独立输出缓冲区，每周期发送前重新写入域缓冲区。

### EtherCAT 安全使能测试

使能指定拓扑从站（例如 `slave 1`）并持续保持当前的位置：

```bash
sudo ./build/motor_console --ethercat-enable enp86s0 1
```

可在保持时间后指定主站 Sync0 相位偏移（单位 ns，范围 `-4000000` 至 `4000000`；默认 `0`）。例如指定 `+100 us`：

```bash
sudo ./build/motor_console --ethercat-enable enp86s0 1 0 100000
```

其中 `0` 表示持续运行直到 `Ctrl-C`，`100000` 是传给 EtherLab DC 配置的主站 shift。周期参数可独立指定，不需要填写 hold 或 shift：

```bash
sudo ./build/motor_console --ethercat-enable enp86s0 1 sync0_cycle_ms=0.5
```

交互终端同样可输入 `ethercat_enable 1 sync0_cycle_ms=0.5`。所有交互式 `ethercat_*` 命令进入同一个后台队列，提交后立即返回提示符；“已提交”不等于执行成功，请等待后台结果。命令按提交顺序执行，最多等待 16 条，队列满时拒绝新命令。参数错误、从站或网卡不匹配不会停止已有主站。

尚无周期会话时，首次使能会打印各阶段耗时（毫秒）和初始化总耗时，包括主站申请、从站发现、PDO/DC 配置、SDO 快照、主站激活、SAFE-OP 等待、PDO/WKC、OP 及 CiA402 使能。等待 SAFE-OP 时约每秒输出主站响应数、链路状态及各从站的 EtherCAT AL 状态；这能区分主站激活耗时与状态等待，但不能进一步显示 EtherLab 内核配置状态机的内部步骤。`7s` 是等待超时上限，不是固定延时。已运行的后台主站上再次执行 `ethercat_enable` 不重走这些初始化阶段。

使能成功后，后台持续交换 PDO。`ethercat_position` 接收绝对输出端角度，按各从站 Pn051 换算后更新原循环的电机端目标；Pn051 无效、目标超出计数范围、未使能或有故障时拒绝更新。首次执行位置命令且尚无周期会话时，先后台使能并保持当前位置，再提交目标。位置是阶跃目标，不是限速轨迹。`ethercat_disable` 只失能指定从站，保留主站和 PDO；随后可以重新 `ethercat_enable`，不会重开主站。`enable`/`position` 的 `hold_ms` 沿用定时失能语义：正数到期后发送失能但不关闭主站，省略或为 0 则持续保持到显式失能；新命令覆盖所选从站的保持期限。

`ethercat_scan` 在周期会话中显示已有拓扑和最新状态，不重新扫描；`ethercat_info` 显示会话初始化时缓存的 SDO 配置及最新 PDO，查询未缓存的从站直接拒绝。`ethercat_pnread` 使用异步 SDO 请求，等待响应时继续 PDO 通信。每次请求有超时，失败或仍忙时不会重用在途请求。设零、参数写入及保存命令在会话中要求目标从站已确认失能，请先执行 `ethercat_disable`，不会自动失能其他电机；保存参数的 100ms 等待期间也继续交换 PDO。Pn028/044/106/107 写入回读后更新 `motion snapshot`，其他 SDO 配置仍为使能前快照；最新参数值可用 `ethercat_pnread` 查看。尚无周期会话时，交互扫描会预热 PDO 并保持失能；只读 `ethercat_info` 仍不会自动启动 PDO 或使能。

`power_off`、退出终端或 `Ctrl-C` 会取消待执行命令，失能并释放主站；PDO 通信失败也会关闭当前会话并取消待执行命令，避免恢复后执行旧运动目标。普通命令按队列处理，不能代替硬件急停。独立 `--ethercat-*` 命令行模式仍前台运行，生命周期不变。`0.5` 表示 `500000 ns`，同时改变 Sync0 和主站 PDO 周期；不写 Pn077。运行中的周期/shift 不能更改，需要退出终端后重新配置；省略这些参数时沿用当前会话设置。

使能总线上全部从站并保持当前位置 5 秒后自动失能：

```bash
sudo ./build/motor_console --ethercat-enable enp86s0 all 5000
```

该命令仅支持扫描到的开璇 `Kaiserdrive_ECAT` 和实测 `13 B` 输出、`14 B` 输入 PDO
布局。它按指定周期（默认 `1 ms`）配置从站的 DC Sync0，再将目标位置写为每台电机的当前反馈位置，最后依次发送 CiA-402 控制字
`0x0006`、`0x0007`、`0x000F`。命令持续发送对应周期的 PDO；到时会发送 `0x0000` 失能。交互会话定时失能后继续通信，通信失败或退出时关闭主站；独立命令行模式到时或 `Ctrl-C` 后关闭主站。不要从其他进程同时启动第二个 EtherCAT 主站。

若使能流程失败，程序会在失能和关闭主站前自动输出现场诊断：EtherCAT AL 状态、PDO 原始字节、
WKC 与循环间隔、CiA-402 状态和 ESC DC 激活/周期寄存器。失败快照不执行可能阻塞的 SDO 上传；
驱动器错误码优先从 PDO 读取。主动按 `Ctrl-C` 或执行 `ethercat_disable` 不触发失败诊断。若处于 `fault`，先排除
供电、机械急停和编码器等故障；确认安全后按手册发送 `0x0080` 复位故障，再重新使能。

失能指定从站或总线上全部从站：

```bash
sudo ./build/motor_console --ethercat-disable enp86s0 1
sudo ./build/motor_console --ethercat-disable enp86s0 all
```

交互模式下 `ethercat_disable <slave_id|all>` 提交后台失能请求，执行后确认所选从站的 CiA-402 失能状态，其他从站不受影响。已有周期会话保留 OP 和 PDO 通信；要释放 EtherLab master 请退出终端或执行 `power_off`。不要在另一个终端同时启动占用 master 0 的 EtherCAT 主站。

### EtherCAT CSP 位控

交互命令 `ethercat_position <slave_id|all> <output_rad> [hold_ms]` 和独立命令
`--ethercat-position <interface> <slave_id|all> <output_rad> [hold_ms]` 中的 `output_rad`
均为减速器输出端的**绝对角度**（rad）。每台从站读取其 Pn051 减速比，将目标换算为
`0x607A` 的电机端位置计数：`round(output_rad × Pn051 × 1048576 / 2π)`。
例如 Pn051=101 时，输出端转 `2π rad` 对应电机端 101 圈、105906176 count。
`all` 对每台从站分别换算并在全部校验通过后才更新目标；Pn051 读取失败、非正数或计数超出
int32 范围时拒绝发送位置目标。

以下仅为命令语法示例，目标 0 是输出端绝对零位，执行前必须确认当前位置及机械行程：

```bash
sudo ./build/motor_console --ethercat-position enp86s0 1 0
```

独立命令行模式使用 CSP（`0x6060=8`），先配置目标位置，再按 `0x0006`、`0x0007`、`0x000F`
使能，随后以默认 1 ms 周期持续发送 `0x607A`。目标仍是阶跃，不做限速轨迹；角度沿用
`0x6064` 的编码器零位和方向，`output_pos` 为电机位置除以 Pn051 的换算值，不是独立输出编码器反馈。
**此改动改变旧位置命令的单位**：原先的电机端目标数值不能原样使用；请先用 `ethercat_info 1`
确认当前 `output_pos`，从附近的小幅目标开始，并预留机械限位空间。旧电机端目标若需换算，
新输入约为 `旧目标电机端弧度 / Pn051`；`ethercat_info` 的目标 `pos` 仍是电机端角度，
新增 `output_pos` 显示相应的输出端换算角度。

### EtherCAT Profile Position（PP）位控

本项目同时支持两种 CiA-402 位置模式：

- `ethercat_position`：CSP（`0x6060=8`），后台循环每个周期持续发送目标位置。
- `ethercat_position_pp`：PP（`0x6060=1`），通过控制字 bit4 的新设定点握手发送目标；状态字 bit12 表示设定点确认，bit10 表示到位。

PP 命令的目标同样是减速器**输出端绝对角度**（rad），仍按每台从站的 Pn051 换算到 `0x607A` 电机端计数：
`round(output_rad × Pn051 × 1048576 / 2π)`。`profile_velocity` 必填，`profile_accel`、`profile_decel` 默认均为 `100`，`hold_ms` 默认持续运行：

```text
ethercat_position_pp <slave_id|all> <output_rad> <profile_velocity> [profile_accel] [profile_decel] [hold_ms] [network_interface]
```

交互终端也支持无下划线别名 `pp`，参数完全相同。若键盘无法输入 `_`，可直接输入 `pp 1 0.02 1000 100 100`；其他 EtherCAT 命令可用 `ethercat<Tab>` 自动补出下划线，或在英文输入法下使用 `Shift+-`。

三个 profile 参数以 ESI 中的原始 `UDINT` 写入并回读 `0x6081`、`0x6083`、`0x6084`，本程序不擅自解释单位；请按电机用户手册选择范围和单位。`profile_accel` 对应 PP 加速度，`profile_decel` 对应 PP 减速度，与通用限制参数 Pn106/Pn107 不同。PP 模式首次切换或修改 profile 参数前必须先执行 `ethercat_disable`。已经处于 PP 使能状态时，可以重复执行 `ethercat_position_pp` 或 `pp` 只发送新的设定点，但三个 profile 数值必须与已写入值相同；省略加/减速度按默认 100 计算。若参数不一致，将拒绝发送位置目标；要改参数，先失能。

推荐先用很小的输出端角度和保守 profile 参数逐步验证：

```text
power_on
ethercat_scan
ethercat_info 1

# CSP：扫描成功后先使能，再发送输出端目标
ethercat_enable 1
ethercat_position 1 0.01 1000
ethercat_info 1

# 切换到 PP：必须先失能；首次 PP 命令会写参数、重新进入 operation-enabled 并发送设定点
ethercat_disable 1
ethercat_position_pp 1 0.02 1000 100 100 1000
ethercat_info 1

# PP 下发送第二个输出端目标，仍使用上一次已写入的 profile 参数
ethercat_position_pp 1 0.01 1000

# 切回 CSP：先失能，再由 enable 将模式恢复为 CSP
ethercat_disable 1
ethercat_enable 1
ethercat_position 1 0.01 1000
```

调试时以每条命令的后台结果为准：关注 `PP 设定点已确认`、`setpoint_ack=yes`，以及 `ethercat_info` 的实时 `SW`、实际输出端角度和目标输出端角度。`hold_ms` 到期只发送失能并保留后台 PDO 主站。

独立 CLI 调试方式如下；它会独占一次主站会话，完成后关闭主站：

```bash
sudo ./build/motor_console --ethercat-position-pp enp86s0 1 0.02 1000 100 100 1000
```

PP 失败时，先执行 `ethercat_info 1` 查看 `SW`、`error_code`、PDO error 和当前模式；如果当前仍是 CSP 或已 fault，先 `ethercat_disable 1`，确认故障原因后再重试。不要在真实机构上直接使用大角度或大 profile 数值。

### EtherCAT 零位设置

在机械位置已摆正时可设置当前机械位置为零。交互会话中先执行 `ethercat_disable 1` 并等待确认，再执行 `ethercat_zero 1`，不关闭主站。下列独立命令行方式只能在没有其他主站进程占用时使用：

```bash
sudo ./build/motor_console --ethercat-zero enp86s0 1
sudo ./build/motor_console --ethercat-zero enp86s0 all
```

命令会先确认伺服失能，再通过 SDO 对 `Pn101 (0x2101:00, int16)` 写入 `0→1→0`。这只记录
零位请求；根据开璇手册，完成后必须同时重启执行器主电和 USB 电源，`0x6064` 的零位才会生效。

### EtherCAT 信息查询

读取指定从站或全部从站的可安全查询信息：

```bash
sudo ./build/motor_console --ethercat-info enp86s0 1
sudo ./build/motor_console --ethercat-info enp86s0 all
```

命令按组输出从站身份、驱动参数（`Pn001/002/028/044/051/070/075/077/079/085/088/106/107/150`）、PDO/DC 与 SM2/SM3
同步信息、CiA-402 状态、实际反馈和目标值。位置同时输出电机端角度与 `电机端角度 / Pn051` 的减速器输出端角度；
减速比读取失败或非正数时输出端角度显示 `?`。`Pn088` 用于解释实际速度单位，`Pn002` 显示监控项选择，
当前监控值来自 `0x3000`。Pn150 显示编码器配置：`0` 为双编码器，`1` 为高速端单多圈编码器。
Pn051 显示减速比设定值。单项读取失败显示 `?`。读取不会配置 PDO、请求 OP、使能或运动电机。

`motion` 行显示 Pn028 电机端转速上限（rpm）、Pn106 加速度限制值（手册未注明单位）、
Pn107 加速度限制开关（0 关闭、1 开启）及 Pn044 速度指令滤波档位（0 关闭，1～9 对应 200～1000us）。
对应对象分别为 `0x2028:00`、`0x2106:00`、`0x2107:00`、`0x2044:00`，按 16 位有符号整数读取。
后台会话中显示为 `motion snapshot`，初始使用使能前 SDO 缓存。四项参数写入后使用回读值更新，
并标记 `[write/readback updated]`；写入后未能回读的项显示 `?`，不继续显示旧值。
`info` 本身不额外读取 SDO 或中断 PDO。

需要读取其他 Pn 参数时，可指定编号（支持 `150` 或 `Pn150`）：

```text
ethercat_pnread 1 Pn150
ethercat_pnread all 150
```

该命令仅对当前开璇驱动器按十进制 Pn 编号（0～9999）的各位数字编码对象地址：
如 Pn028 → `0x2028:00`、Pn106 → `0x2106:00`、Pn150 → `0x2150:00`，
不是直接把十进制编号加到 `0x2000`。显示数据长度、原始字节和小端整数解释；
数据类型与单位请以对应参数手册为准。它只读 SDO，不配置 PDO 或请求 OP；若后台使能循环正在运行，
交互终端会通过现有后台主站异步读取，不失能、不重开主站。也可从 shell 执行 `sudo ./build/motor_console --ethercat-pnread enp86s0 1 Pn150`（不要与运行中的交互会话同时占用主站）。

### EtherCAT 转速、加速度与滤波配置

| 交互命令 | 数值范围 | 作用 | SDO 对象 |
| --- | --- | --- | --- |
| `ethercat_pn028 <slave_id\|all> <value> [network_interface]` | 0～5000 | 电机端转速上限，rpm | `0x2028:00` |
| `ethercat_pn106 <slave_id\|all> <value> [network_interface]` | 0～300 | 加速度限制值，手册未注明单位 | `0x2106:00` |
| `ethercat_pn107 <slave_id\|all> <value> [network_interface]` | 0、1 | 加速度限制开关，1 开启、0 关闭 | `0x2107:00` |
| `ethercat_pn044 <slave_id\|all> <value> [network_interface]` | 0～9 | 速度指令滤波档位 | `0x2044:00` |

四项均在后台执行，复用已有主站及 PDO 循环。先显式执行 `ethercat_disable 1` 并等待失能确认，
再按需写入。以下数值仅为语法示例，不是针对你的机械系统推荐的调参值：

```text
ethercat_pn028 1 1000
ethercat_pn106 1 150
ethercat_pn107 1 1
ethercat_pn044 1 9
ethercat_info 1
ethercat_pnread 1 Pn028
```

每条命令检查从站型号、失能状态、范围及原值，然后以 16 位有符号整数写入并回读。
`all` 会在写入前预检全部目标，但多从站写入不是原子事务；中途失败时已写入的值不会自动回滚，
应逐台回读确认。命令不会自动使能、保存参数或更改 PDO 周期。
需要持久化时另行执行 `ethercat_save 1`，等待保存确认，再按厂商流程重启并回读验证。
手册标注这些参数在线生效，但程序为安全起见要求先失能。不要直接调到最大限速或盲目关闭加速度限制。

同样支持独立命令行模式，不初始化 PCI/CAN；不得与已占用主站的交互会话同时运行：

```bash
sudo ./build/motor_console --ethercat-pn028 enp86s0 1 1000
sudo ./build/motor_console --ethercat-pn106 enp86s0 1 150
sudo ./build/motor_console --ethercat-pn107 enp86s0 1 1
sudo ./build/motor_console --ethercat-pn044 enp86s0 1 9
```

### EtherCAT Pn077 配置

驱动器失能时，可通过 SDO 写入 Pn077（仅允许 `0` 或 `1`）：

```text
ethercat_pn077 1 0
```

指定其他网卡时把它放在末尾，例如 `ethercat_pn077 1 0 enp5s0`。命令也可从 shell 执行：`sudo ./build/motor_console --ethercat-pn077 enp86s0 1 0`。它会确认目标为开璇驱动器、检查伺服未使能，写入 SDO `0x2077:00` 并回读验证。根据手册，`0` 表示 DC 时钟偏移 `100 us`，`1` 表示不偏移；修改需要重启执行器生效。该电机参数不同于主站的 Sync0 shift；此命令不会使能电机或改变 Sync0 配置。

终端中按 `Tab` 可补全 EtherCAT 命令、当前电脑 `/sys/class/net` 中的网卡名、`all`、
从站序号 `1..100` 以及常用的 `hold_ms` 值。`hold_ms` 也接受手工输入的 `1..60000`。

根据厂商提供的流程，可在驱动器失能时通过 `ethercat_save` 触发参数保存：

```text
ethercat_save 1
```

该命令向 Pn097（SDO `0x2097:00`）依次写入 `1`、`0`，并确认回到 `0`。命令成功只表示保存触发序列已确认发送；仍需按厂商要求重启执行器，再用 `ethercat_info 1` 验证参数是否持久化。也支持 `all` 选择全部从站、末尾网卡覆盖（如 `ethercat_save all enp5s0`）以及独立命令 `sudo ./build/motor_console --ethercat-save enp86s0 1`。

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
电机[已上电]> motor_scan
电机[已上电]> motor_list
电机[已上电]> can_status
```

`power_on` 会执行：

1. 打开电机总电源。
2. 按 `power_on_wait_ms` 等待软启动。
3. 结束上电流程，不读取配置文件扫描 CAN，也不发送 CAN 帧。

需要发现配置中的 CAN 电机时，电机上电后手动执行 `motor_scan`；该命令仍按配置文件
中的电机列表扫描。`motor_probe` 保留完整流程：上电、主动扫描配置电机，然后自动下电。

典型输出：

```text
power_on: start 电源已开启 wait=2s
power_on: done power=on; CAN scan skipped (run motor_scan explicitly)
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

常用快捷写法：

```text
enable              # 使能全部电机
enable 00           # 使能单台电机
disable             # 失能全部电机
disable 00          # 失能单台电机
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

也可以使用更短的命令名：

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

### 7.7 直通调试

```text
debug 00
```

`debug 00` 等价于 `motor_dbg 00`；进入后每个按键都会立即发送给目标电机，按反引号 `` ` `` 退出。

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
| `power_on` | 无 | 电机上电并等待软启动；不扫描 CAN |
| `power_off` | 无 | 失能已知使能电机并关闭总电源 |
| `motor_probe` | 无 | 上电、接收并显示配置电机回复，最后无论结果如何都会自动下电 |
| `motor_list` | 无 | 显示电机 index、bus、id、型号、在线/使能状态、程序/配置版本和最后反馈 |
| `motor_scan` | `[timeout_ms]` | 按配置文件中的电机列表扫描 CAN 电机 |
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
| `enable` / `disable` | `[<index00>\|all]` | 快捷使能/失能；省略参数或使用 `all` 表示全部电机 |
| `mit` | `<slave_id\|all> <pos_rad> <vel_rad_s> <tff_Nm> <kp> <kd> [hold_ms] [network_interface]` | 开璇 EtherCAT 力位混合控制 |
| `debug` | `<index00>` | `motor_dbg` 的快捷写法 |
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
