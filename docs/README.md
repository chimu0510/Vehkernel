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
| [Companion computer](guide/companion-computer.md) | Host tools bundle, unpacking, parameters, flashing |
| [Estimation](guide/estimation.md) | Attitude estimators, heading fusion, magnetometer calibration, board orientation check |
| [Driving the wheels](guide/driving.md) | Safety conditions, arming, motor bring-up, driving, odometry, remote control |
| [Local display](guide/local-display.md) | Pages, user key, local menu |
| [Logging](guide/logging.md) | Recording, profiles, dropouts, opening logs, replay |
| [Testing](guide/testing.md) | Test layers, `ctest`, estimator replay, fuzzing, bring-up image, hardware checks |
| [Examples](guide/examples.md) | Building, flashing, SITL and lessons for the board examples |
| [Delegating to agy](guide/delegating-to-agy.md) | Sandboxing the Antigravity CLI for ticket work: install, login, running, checking the sandbox |

## Boards

| Board | ADR |
|---|---|
| [C50C V1.1](boards/c50c-v1_1.md) | [ADR-0024](adr/0024-c50c-v1_1-board.md) |

## Repository layout

| Directory | Contents |
|---|---|
| `app/` | Entry points: firmware image and SITL executable |
| `boards/` | Board definitions and board defconfigs |
| `bootloader/` | The Vehkernel bootloader |
| `bridge/` | ROS2 Bridge for the companion computer; a placeholder |
| `cmake/` | Toolchain, Kconfig integration, generator invocation and the layering rules |
| `core/` | Bus, work queues, parameter runtime and the types shared by modules and Links |
| `docs/` | This documentation |
| `drivers/` | Driver layer: device drivers |
| `examples/` | Board examples, one component at a time |
| `external/` | External code, pinned as git submodules |
| `hal/` | Interface layer: HAL interfaces |
| `lib/` | Estimation math components |
| `modules/` | Module layer: one directory per module |
| `platform/` | Platform layer: one directory per platform, SITL among them |
| `schema/` | Topic schemas, enums whose values cross the wire, and log profiles |
| `sim/` | SITL's simulated devices and the ground vehicle they move |
| `soc/` | One directory per SoC, grouped by vendor and family |
| `test/` | Tests, by layer ([Testing](guide/testing.md)) |
| `tools/` | Host tools, one directory per purpose |
| `.devcontainer/` | The pinned development image used locally and by CI |
| `.github/` | CI workflow, issue forms and the pull request template |

Parameter definitions (`params.toml`) live in the directory of their module or driver, so they move or disappear together with their module (ADR-0012). Log profiles span several modules' topics and live beside the schema, in `schema/log_profiles.toml` (ADR-0019).
