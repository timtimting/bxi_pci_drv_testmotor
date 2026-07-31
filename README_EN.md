# BXI Motor Control and Firmware Tools

[简体中文](README.md) | [English](README_EN.md)

This project provides one Linux user-space program for the BXI PCIe CAN/CAN-FD
controller: `motor_console`, a unified terminal for power control, discovery,
MIT commands, state monitoring, and firmware flashing.

> Safety: these commands operate real motors. Keep the emergency stop available,
> clear the mechanism, and validate one motor before running group commands.

## Build

```bash
git clone https://github.com/bxirobotics/bxi_pci_drv.git
cd bxi_pci_drv
make
```

Built program:

- `build/motor_console` — unified interactive terminal.

## Configuration

The default terminal configuration is `config/motor_console.yaml`. Validate it without
initializing PCI/CAN hardware:

```bash
./build/motor_console --check-config
```

Use another file:

```bash
sudo ./build/motor_console --config /path/to/motor_console.yaml
```

The terminal configuration contains timings, state-detection patterns, MIT
limits, live-output settings, and the KP/KD values used by the soft-start
zero-position command. Motor `index`/`bus`/`id`/`version` topology is loaded
from `config/flash_plan_default.yaml`.

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
motor[POWER-ON]> motor_set 00 0.5 0 0 10 1
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
| `mit_zero_set_single` | `<index00>` | Calibrate one disabled motor |
| `mit_enable_all` | none | Enable all configured motors |
| `mit_disable_all` | none | Disable all configured motors |
| `mit_enable_single` | `<index00>` | Enable one motor |
| `mit_disable_single` | `<index00>` | Disable one motor |
| `motor_set` | `<index00> <pos> <torque> <vel> <kp> <kd>` | Send one validated MIT command |
| `stand_up` | none | Ramp KP and move every online/enabled motor to zero |
| `reg_read` | `<index00\|all> <reg_index> [wait_ms]` | Read registers; `all` sends to every target in the default flash plan |
| `reg_write` | `<index00\|all> <reg_index> <value> [wait_ms]` | Write registers; `all` sends to every target in the default flash plan; refused while any motor is known enabled |
| `reg_save` | `<index00\|all> [wait_ms]` | Save register configs; `all` sends to every target in the default flash plan; refused while any motor is known enabled |
| `flash_single` | `<index00> [version\|firmware.bin] [cycle]` | Flash one motor; omit firmware to use the default version |
| `flash_all` | `[plan.yaml] [cycle]` | Flash multiple targets; omit plan to use the default one |
| `flash_debug` | `<index00>\|<bus0-4 id0-7> <version\|firmware.bin> [cycle]` | Two digits select index; one-digit bus + one-digit id select Bus/ID |
| `can_status` | `[reset]` | Show or reset per-bus software statistics |
| `help` / `-h` / `?` | none | Show built-in help |
| `quit` / `exit` / `q` | none | Exit; motor power must be off |

Register commands follow the motor firmware CAN config protocol:
`reg_write` sends `CAN_CMD_SET_CONFIG=0x11`, `reg_read` sends
`CAN_CMD_GET_CONFIG=0x12`, and `reg_save` sends `CAN_CMD_UPDATE_CONFIGS=0x13`.
Read/write replies print the register address and raw/uint/int/float values.

## Firmware flashing

Terminal settings and MIT limits live in `config/motor_console.yaml`.
The default full-robot flash versions live in `config/flash_plan_default.yaml`.

```text
firmware/bxi_motor_50.bin
firmware/bxi_motor_50L.bin
firmware/bxi_motor_70.bin
firmware/bxi_motor_85.bin
```

If firmware is updated frequently, keep it on an internal download server and
configure the startup download URL:

```yaml
firmware_dir: firmware
firmware_sync_on_start: off
firmware_base_url: "http://your-download-server/motor"
firmware_cache_dir: firmware_cache
```

Set `firmware_sync_on_start: on` to enable startup sync. With the default `off`,
the console uses bundled `firmware_dir` and does not contact the download
server. When enabled, the console fetches the firmware versions required by the
default flash plan `config/flash_plan_default.yaml`. If every required firmware
file is downloaded successfully, flashing in this run prefers
`firmware_cache_dir`. If any download fails, the run falls back to bundled
`firmware_dir`. Leaving `firmware_base_url` empty also keeps the old local-only
behavior. The download filename comes from
`config/motor_console.yaml -> motor_types -> firmware`; for example,
`version: 50L` downloads the configured `bxi_motor_50L.bin` from
`http://your-download-server/motor/` into the cache. The target machine needs
either `curl` or `wget`.

Examples:

```text
motor[POWER-ON]> flash_single 00
motor[POWER-ON]> flash_single 08 50L
motor[POWER-ON]> flash_all
motor[POWER-ON]> flash_all config/flash_plan_default.yaml
```

For repeated bring-up work, copy `config/flash_plan_default.yaml`, edit
`firmware_dir` and `targets`, then run `flash_all <your-plan.yaml>`. The flash
plan contains the firmware directory plus selected motors: `index`, `bus`, `id`,
and firmware `version`.

Safety note: `firmware_dir` and `firmware_cache_dir` must be relative
directories without absolute paths or `..`. `firmware_base_url` must start with
`http://` or `https://`. `version` and manual firmware arguments cannot contain
`/` or `..`.

The normal path probes the bootloader, sends application reset `r`, catches the
boot window with `m`, waits for `Commands:`, sends `u`, and transfers the file
using YMODEM over CAN. Use `cycle` if soft reset cannot enter boot:

```text
motor[POWER-ON]> flash_single 00 cycle
motor[POWER-ON]> flash_all cycle
```

Flashing requires power and refuses to start while a motor is known to be enabled.
All selected targets and files are checked before the first motor is modified.

## CAN statistics

```text
motor[POWER-ON]> can_status
motor[POWER-ON]> can_status reset
```

Statistics include TX/RX counts, send failures, expected and matched replies,
pending replies, timeouts, and estimated response loss per bus. The public driver
API does not expose hardware TEC, REC, or bus-off registers, so these are software
command/response statistics rather than controller error-register values.

## Source layout

`src/main.c` is the source entry point for the `motor_console` executable. It includes `src/lib/runtime.c` and
the feature modules below in the same translation unit to reuse the existing
bootloader, YMODEM, CAN and MIT helpers. `src/lib/runtime.c` is an internal
implementation file and is not built as a standalone program. The `src/` root
keeps only `main.c`; feature files live under `src/lib/` and are included by
`main.c`, so do not add them as standalone CMake sources:

| File | Area |
|---|---|
| `src/main.c` | Program entry point and internal module includes |
| `src/lib/core.c` | Common helpers, config path resolution, motor lookup |
| `src/lib/display.c` | Help text, config summary, motor list output |
| `src/lib/control.c` | Power, scan, MIT control and CAN statistics |
| `src/lib/flash.c` | Flash commands and flash-plan parsing |
| `src/lib/debug.c` | Pass-through motor debug mode |
| `src/lib/terminal.c` | Command dispatch, config reload and terminal loop |
| `src/lib/bxi_motor_comm.c` / `src/lib/bxi_motor_comm.h` | MIT frame packing, special command frames, reply decoding and default limits |

## Troubleshooting

- Run `./build/motor_console --check-config` first.
- If no motors reply, check the emergency stop, motor power, PCIe device, CAN
  wiring, termination, and then try `motor_scan 3000` plus `can_status`.
- If boot entry fails, ensure motors are disabled and retry with `cycle`.
- If firmware preflight fails, verify the `version` values in the flash plan,
  the `firmware/` files, and file permissions.
