# Documentation

| Section | Contents |
|---|---|
| [`guide/`](guide/) | How to build, flash, run SITL, drive and test, for users and developers |
| [`boards/`](boards/) | One page per supported board: connectors, ports and wiring |
| [`adr/`](adr/README.md) | Architecture decision records: why the firmware is built the way it is |
| [`specs/`](specs/README.md) | Milestone specs: what each milestone set out to build |
| [`agents/`](agents/) | Workflow rules for the issue tracker, triage labels and handoff between sessions, written for AI coding agents and followed by human contributors too |
| [`../GLOSSARY.md`](../GLOSSARY.md) | The project vocabulary; docs, code and issues use its terms |

## Guides

| Guide | Contents |
|---|---|
| [Building](guide/building.md) | Devcontainer, host tools, presets, Kconfig |
| [Flashing](guide/flashing.md) | SWD, the serial flasher, rebooting into the bootloader |
| [SITL](guide/sitl.md) | Options, report, motion profiles, transports |
| [Link monitor](guide/link-monitor.md) | Live view, checks, attitude, parameters, recording a log |
| [Driving the wheels](guide/driving.md) | Safety conditions, arming, motor bring-up, driving, odometry, remote control |
| [Local display](guide/local-display.md) | Pages, user key, local menu |
| [Logging](guide/logging.md) | Recording, profiles, dropouts, opening logs, replay |
| [Testing](guide/testing.md) | Test layers, `ctest`, estimator replay, fuzzing, bring-up image, hardware checks |
| [Delegating to agy](guide/delegating-to-agy.md) | Sandboxing the Antigravity CLI for ticket work: install, login, running, checking the sandbox |

## Boards

| Board | ADR |
|---|---|
| [C50C V1.1](boards/c50c-v1_1.md) | [ADR-0024](adr/0024-c50c-v1_1-board.md) |

## Repository layout

| Directory | Contents | ADRs |
|---|---|---|
| `docs/` | User guides (`guide/`), board pages (`boards/`), ADRs (`adr/`), milestone specs (`specs/`) and agent workflow docs (`agents/`); this index is [`docs/README.md`](README.md) | 0021 |
| `boards/` | Board definitions (TOML) and board defconfigs | 0010, 0011, 0017 |
| `soc/` | One directory per SoC, grouped by vendor and family (`soc/st/stm32f4/stm32f407vet6/`): the SoC database (AF map, DMA request map, Flash sector table, as `soc.json`), its stm32-data source and the Kconfig SoC defaults (`defconfig`) | 0011, 0017, 0020 |
| `schema/msg/` | Topic schemas (`.msg`) | 0006 |
| `schema/enum/` | Enums whose values cross the wire (`.enum`) | 0006 |
| `cmake/` | Toolchain, Kconfig integration, generator invocation and the layering rules | 0009, 0017 |
| `hal/` | Interface layer: HAL interfaces (C++20 concepts and runtime interfaces) | 0009 |
| `platform/stm32f4/` | Platform layer: STM32F4 implementation (ST LL), startup, linker script, the internal Flash code the bootloader shares, the serial port parameters, and the USB gamepad over TinyUSB (`usb/`) | 0009, 0020 |
| `platform/sitl/` | Platform layer for SITL and tests: FreeRTOS POSIX port, UDP and pty transports | 0009, 0018 |
| `drivers/` | Driver layer: device drivers (ICM20948, SSD1306) and the shared IMU parameters | 0009, 0014 |
| `core/` | Bus, work queues, parameter runtime and storage, timing monitor, app descriptor, and the mode, arming and reboot types shared by modules and Links | 0008, 0012, 0015, 0023 |
| `modules/` | Module layer: `commander/` (modes, arming, emergency stop), `actuators/` (actuator control, motor bring-up), `remote_control/` (gamepad input and its mapping), `control/` (control chain), `vehicle_model/` (vehicle models, wheel kinematics), `estimation/` (attitude estimator, local odometry), `link/` (custom protocol, topic map, logging over the Link), `local_display/` (display, local menu, user key) | 0002, 0005, 0013, 0014, 0019, 0022 |
| `lib/estimation/` | Estimation math components: `models/`, `filters/`, `math/` | 0014 |
| `bootloader/` | The Vehkernel bootloader: a port of ArduPilot's `AP_Bootloader` structure, its STM32F4 build in the BOOT region and its SITL build | 0023 |
| `tools/socdb/` | Offline converter that regenerates an SoC database from the stm32-data source committed under `soc/<vendor>/<family>/<soc>/source/` | 0011, 0016 |
| `tools/kconfig/` | Layered Kconfig solve and `menuconfig` with overlay saving | 0017 |
| `tools/generators/` | Schema, board and parameter generators; the schema generator also emits the log profiles | 0006, 0011, 0012, 0019 |
| `tools/link/` | Link monitor: live view, checks, parameters, vehicle commands and recording ULog files | 0002, 0006, 0019 |
| `tools/log/` | ULog reading and writing, estimator replay (`replay.py`), comparison (`compare.py`) and cutting (`cut.py`) | 0018, 0019 |
| `tools/flash/` | Serial flasher through the STM32 system bootloader | 0020, 0023 |
| `tools/firmware/` | Fills the app descriptor after linking and writes the `.apj` | 0023 |
| `tools/bootloader/` | ArduPilot's `uploader.py`, pinned, and the script listing upstream bootloader changes | 0023 |
| `tools/bringup/` | Bring-up runner for the hardware cases over SWD | 0018 |
| `tools/agy/` | `agy_watch.py`: shows an agy conversation's progress from a copy, without disturbing a delegation ([Delegating to agy](guide/delegating-to-agy.md)) | |
| `app/` | Entry points: firmware image and SITL executable | 0017 |
| `external/` | External code, pinned as git submodules: FreeRTOS kernel, CMSIS core, STM32F4 device headers and LL drivers, TinyUSB. A tool's own pinned external files sit beside it, as `tools/bootloader/external/` | 0004, 0016 |
| `bridge/` | ROS2 Bridge for the companion computer; a placeholder, nothing is built yet | 0002, 0003 |
| `sim/` | SITL's simulated devices (ICM20948 with motion profiles, SSD1306, NOR Flash, motors and encoders) and the ground vehicle they move; the external simulator interface comes later | 0018 |
| `test/` | `unit/` and `fuzz/` (L1), `replay/` (L2, the `vehkernel-replay` executable), `sitl/` (L3), `hardware/` (the L4 bring-up image), `cmake/` (build system checks); `ros2/` (L5) is a placeholder. L0 static checks run from root config files and CI | 0018 |
| `.devcontainer/` | The pinned development image used locally and by CI | 0016 |
| `.github/` | CI workflow, issue forms and the pull request template | 0018 |

Parameter definitions (`params.toml`) live in the directory of their module or driver, so they move or disappear together with their module (ADR-0012). Log profiles span several modules' topics and live beside the schema, in `schema/log_profiles.toml` (ADR-0019).

