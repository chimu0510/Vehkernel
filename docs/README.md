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
| [Link monitor](guide/link-monitor.md) | Live view, checks, attitude, parameters |
| [Driving the wheels](guide/driving.md) | Safety conditions, arming, motor bring-up, driving |
| [Local display](guide/local-display.md) | Pages, user key, local menu |
| [Testing](guide/testing.md) | Test layers, `ctest`, bring-up image, hardware checks |

## Boards

| Board | ADR |
|---|---|
| [C50C V1.1](boards/c50c-v1_1.md) | [ADR-0024](adr/0024-c50c-v1_1-board.md) |

## Repository layout

| Directory | Contents | ADRs |
|---|---|---|
| `docs/` | User guides (`guide/`), board pages (`boards/`), ADRs (`adr/`), milestone specs (`specs/`) and agent workflow docs (`agents/`); this index is [`docs/README.md`](README.md) | 0021 |
| `boards/` | Board definitions (TOML) and board defconfigs | 0010, 0011, 0017 |
| `mcu/` | One subdirectory per MCU: the MCU database (AF map, DMA request map, Flash sector table, as JSON) and the Kconfig MCU defaults (`defconfig`) | 0011, 0017, 0020 |
| `schema/msg/` | Topic schemas (`.msg`) | 0006 |
| `cmake/` | Toolchains, Kconfig integration, generator invocation | 0017 |
| `hal/` | Interface layer: HAL interfaces (C++20 concepts and runtime interfaces) | 0009 |
| `platform/stm32f4/` | Platform layer: STM32F4 implementation (ST LL) | 0009 |
| `platform/sitl/` | Platform implementation for SITL and tests | 0009, 0018 |
| `drivers/` | Driver layer: device drivers | 0009 |
| `core/` | Bus, work queues, parameter runtime, timing monitor | 0008, 0012, 0015 |
| `modules/` | Module layer: estimators, control chain, vehicle models, Links, logging | 0002, 0013, 0014, 0019 |
| `lib/estimation/` | Estimation math components: `models/`, `filters/`, `math/` | 0014 |
| `bootloader/` | The Vehkernel bootloader: a port of ArduPilot's `AP_Bootloader` structure, and its SITL build | 0023 |
| `tools/mcudb/` | Offline converter that regenerates an MCU database from the stm32-data source committed under `mcu/<mcu>/source/` | 0011, 0016 |
| `tools/kconfig/` | Layered Kconfig solve and `menuconfig` with overlay saving | 0017 |
| `tools/generators/` | Schema (including ULog format definitions), board, parameter and log profile generators | 0006, 0011, 0012, 0019 |
| `tools/link/` | Link monitor: live view, checks, parameters and vehicle commands | 0002, 0006 |
| `tools/flash/` | Serial flasher through the STM32 system bootloader | 0020 |
| `tools/bootloader/` | ArduPilot's `uploader.py`, pinned, and the script listing upstream bootloader changes | 0023 |
| `tools/bringup/` | Bring-up runner for the hardware cases over SWD | 0018 |
| `app/` | Entry points: firmware image and SITL executable | — |
| `third_party/` | Pinned submodules: FreeRTOS kernel, CMSIS core, STM32F4 device headers and LL drivers | 0004, 0016 |
| `bridge/` | ROS2 Bridge for the companion computer | 0002, 0003 |
| `sim/` | Built-in simulator and external simulator interface | 0018 |
| `test/` | `unit/` (L1), `replay/` (L2), `sitl/` (L3), `hardware/` (L4 hardware tests), `ros2/` (L5); L0 static checks run from root config files and CI | 0018 |
| `.devcontainer/` | The pinned development image used locally and by CI | 0016 |
| `.github/` | CI workflow, issue forms and the pull request template | 0018 |

Parameter definitions (`params.toml`) live in the directory of their module or driver, and log profiles live under the logging module, so they move or disappear together with their module (ADR-0012, ADR-0019).

