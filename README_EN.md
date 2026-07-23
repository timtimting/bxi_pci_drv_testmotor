# BXI Motor Control and Firmware Tools

[简体中文](README.md) | [English](README_EN.md)

This project provides Linux user-space tools for the BXI PCIe CAN/CAN-FD controller.
`motor_console` is the recommended unified terminal for power control, discovery,
MIT commands, state monitoring, and firmware flashing.

> Safety: these commands operate real motors. Keep the emergency stop available,
> clear the mechanism, and validate one motor before running group commands.

## Build

```bash
git clone https://github.com/bxirobotics/bxi_pci_drv.git
cd bxi_pci_drv
make
```

Built programs:

- `build/motor_console` — unified interactive terminal;
- `build/motor_flash` — legacy flashing/debug tool;
- `build/motor_test` — legacy MIT/register/CAN test tool;
- `build/drv_test` — legacy board test.

## Configuration

The default configuration is `config/motor_firmware.yaml`. Validate it without
initializing PCI/CAN hardware:

```bash
./build/motor_console --check-config
```

Use another file:

```bash
sudo ./build/motor_console --config /path/to/motor_firmware.yaml
```

The configuration contains motor indexes, buses, IDs, models, firmware files,
timings, MIT limits, live-output settings, and the KP/KD values used by the
soft-start zero-position command.

Current topology:

| Body group | CAN location | Indexes |
| --- | --- | --- |
| Waist | bus0/id1-3 | 0-2 |
| Left leg | bus1 | 3-8 |
| Right leg | bus2 | 9-14 |
| Left arm | bus3 | 15-21 |
| Right arm | bus4 | 22-28 |
| Head | bus0/id4-5 | 29-30 |

## Start the console

```bash
sudo ./build/motor_console
```

Startup loads the configuration and initializes PCI/CAN. It does not power,
enable, reset, control, or flash a motor. The editor supports pasted text,
Left/Right, Delete/Backspace, Up/Down history, and Tab completion. Use `help`,
`-h`, or `?` inside the terminal.

## Typical session

```text
motor[POWER-OFF]> power_on
motor[POWER-ON]> motor_list
motor[POWER-ON]> mit_zero_set
motor[POWER-ON]> mit_enable_all
motor[POWER-ON]> motor_set 0 0.5 0 0 10 1
motor[POWER-ON]> stand_up
motor[POWER-ON]> mit_disable_all
motor[POWER-ON]> can_status
motor[POWER-ON]> power_off
motor[POWER-OFF]> quit
```

`power_on` waits for soft start, probes every configured motor across all five
buses, and prints responding indexes and decoded feedback.

## Command reference

| Command | Arguments | Notes |
| --- | --- | --- |
| `power_on` | none | Power on, wait, and scan all configured motors |
| `power_off` | none | Disable known enabled motors, then power off |
| `motor_scan` | `[timeout_ms]` | Probe all configured motors |
| `motor_list` | none | Show power and per-motor runtime state |
| `mit_zero_set` | none | Calibrate zero on all disabled motors |
| `mit_zero_set_single` | `<index>` | Calibrate one disabled motor |
| `mit_enable_all` | none | Enable all configured motors |
| `mit_disable_all` | none | Disable all configured motors |
| `mit_enable_single` | `<index>` | Enable one motor |
| `mit_disable_single` | `<index>` | Disable one motor |
| `motor_set` | `<index> <pos> <torque> <vel> <kp> <kd>` | Send one validated MIT command |
| `stand_up` | none | Ramp KP and move every online/enabled motor to zero |
| `flash_single` | `<index> [cycle]` | Flash one motor selected by model |
| `flash_all` | `[cycle]` | Preflight and flash all configured motors |
| `can_status` | `[reset]` | Show or reset per-bus software statistics |
| `can_monitor` | `on\|off` | Toggle live decoded CAN output |
| `config_show` | none | Show the active configuration summary |
| `config_reload` | `[path]` | Reload while power is off and motors are disabled |
| `help` / `-h` / `?` | none | Show built-in help |
| `quit` / `exit` / `q` | none | Exit; motor power must be off |

## Firmware flashing

Default configured files:

```text
firmware/motor_50.bin
firmware/motor_50L.bin
firmware/motor_70.bin
firmware/motor_85.bin
```

Examples:

```text
motor[POWER-ON]> flash_single 0
motor[POWER-ON]> flash_all
```

The normal path probes the bootloader, sends application reset `r`, catches the
boot window with `m`, waits for `Commands:`, sends `u`, and transfers the file
using YMODEM over CAN. Use `cycle` if soft reset cannot enter boot:

```text
motor[POWER-ON]> flash_single 0 cycle
motor[POWER-ON]> flash_all cycle
```

Flashing requires power and refuses to start while a motor is known to be enabled.
All selected targets and files are checked before the first motor is modified.

## CAN statistics

```text
motor[POWER-ON]> can_status
motor[POWER-ON]> can_status reset
motor[POWER-ON]> can_monitor on
```

Statistics include TX/RX counts, send failures, expected and matched replies,
pending replies, timeouts, and estimated response loss per bus. The public driver
API does not expose hardware TEC, REC, or bus-off registers, so these are software
command/response statistics rather than controller error-register values.

## Troubleshooting

- Run `./build/motor_console --check-config` first.
- If no motors reply, check the emergency stop, motor power, PCIe device, CAN
  wiring, termination, and then try `motor_scan 3000` plus `can_status`.
- If boot entry fails, ensure motors are disabled and retry with `cycle`.
- If firmware preflight fails, verify `firmware_dir`, the four configured file
  names, and file permissions. Relative paths use the process working directory.
