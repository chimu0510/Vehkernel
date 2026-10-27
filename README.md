# Vehkernel

[![CI](https://github.com/chimu0510/Vehkernel/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/chimu0510/Vehkernel/actions/workflows/ci.yml)
[![License: GPL-3.0-or-later](https://img.shields.io/badge/license-GPL--3.0--or--later-blue.svg)](LICENSE)

**Vehkernel** (Vehicle + Kernel) is open-source MCU firmware with ROS2 support, built as a common core for any vehicle: one system drives many vehicles and frames. The MCU handles real-time state estimation, low-level control and safety, and keeps running without a companion computer; global localization and navigation are left to ROS2 applications above it. The first test platform is a UGV.

> [!WARNING]
> Vehkernel is in early development and has not been validated on a moving vehicle. It drives motors: test with the wheels off the ground, output power limited and an emergency stop within reach ([Driving the wheels](docs/guide/driving.md)). Interfaces, parameters and the wire protocol may change without notice.

## Status

The bootloader (spec 0008) is in progress; the hardware check of the first wheel motion (spec 0006) is deferred. What runs today, in SITL and on the C50C V1.1 unless noted:

- FreeRTOS with the bus and rate-tiered work queues, and the ICM20948 driver publishing IMU samples at 200 Hz.
- A Link with the custom protocol to the development host, over serial on the board and UDP or a pty in SITL, with the Link monitor showing every topic live.
- A Mahony attitude estimator, checked in CI against SITL's ground truth.
- Parameters defined in TOML, set over the Link or the local menu, and stored in internal Flash.
- Modes, arming checks and an emergency stop gating four motor outputs with wheel speed loops; checked in SITL, with the board's hardware check still to come.
- The local display and local menu on the board's OLED and user key.
- ArduPilot's app descriptor and `.apj` file, reboot into the bootloader over the Link, and the Vehkernel bootloader in S0 of the board, flashed by ArduPilot's `uploader.py` and checked against it in SITL; its hardware check is still to come.

Decided but not built yet: the ROS2 Bridge (`bridge/` is a placeholder), the vehicle models and the control chain above the wheel speed loops, and the ROS2 (L5) test layer (`test/ros2/` is a placeholder), and boards other than the C50C V1.1.

Every decision is recorded in [`docs/adr/`](docs/adr/), each milestone in [`docs/specs/`](docs/specs/), and [`GLOSSARY.md`](GLOSSARY.md) defines the vocabulary.

## Target hardware

Vehkernel targets the boards ArduPilot and PX4 run on, and follows their conventions for board definitions, bootloaders and tools. Boards are added one at a time, each with its own board definition, ADR and spec ([ADR-0010](docs/adr/0010-board-revision-policy.md)).

| Item | Supported now |
|---|---|
| MCU | STM32F407VET6 |
| Board | C50C V1.1 (ICM20948, USB gamepad, no RS-485), the first development board ([board page](docs/boards/c50c-v1_1.md), [ADR-0024](docs/adr/0024-c50c-v1_1-board.md)) |
| Companion computer | Raspberry Pi 5 (ROS2), planned: the Bridge does not exist yet, so the development host talks to the board over a Link with the Link monitor |

## Architecture highlights

The design as the ADRs decide it; [Status](#status) says which parts run today.

- **Role**: the MCU is the companion computer's real-time coprocessor and safety gatekeeper, not an autopilot ([0001](docs/adr/0001-mcu-responsibility-boundary.md)).
- **Communication**:
  - A publish/subscribe bus is the internal backbone and ROS2 is just one Link; a Link is made of a topic map, a protocol and a transport, all replaceable ([0002](docs/adr/0002-bus-backbone-and-pluggable-links.md)).
  - Messages are defined in a single schema and code is generated from it ([0006](docs/adr/0006-schema-and-wire-encoding.md)).
- **ROS2 interface**: a vehicle-level topic API with three setpoint levels; ros2_control is optional ([0003](docs/adr/0003-vehicle-level-ros2-api.md)).
- **Safety and local operation**:
  - The mode decides who is in control, and the remote control can always take over; arming checks, an emergency stop, an output watchdog, a fault stop and the hardware watchdog guard the outputs ([0005](docs/adr/0005-setpoint-arbitration.md)).
  - A local display shows the vehicle's status, and its local menu changes whitelisted parameters only while disarmed ([0022](docs/adr/0022-local-display-and-local-menu.md)).
- **Runtime**: FreeRTOS, a C++20 subset, and rate-tiered work queues ([0004](docs/adr/0004-freertos.md), [0007](docs/adr/0007-cpp20-subset.md), [0008](docs/adr/0008-bus-and-work-queues.md)).
- **Hardware support**:
  - Board definitions are written in TOML; the generator validates pins against the SoC database and solves DMA assignments ([0011](docs/adr/0011-toml-board-definition-toolchain.md)).
  - The HAL picks its form of polymorphism by when each choice is made ([0009](docs/adr/0009-layered-hal.md)).
  - Firmware updates use the ArduPilot and PX4 bootloader protocol: boards that ship with that bootloader keep it, and boards without one get a compatible Vehkernel bootloader ([0023](docs/adr/0023-bootloader.md)).
- **Configuration**:
  - Kconfig decides which features are compiled in; parameters decide which one is enabled at runtime ([0017](docs/adr/0017-kconfig-build-configuration.md), [0012](docs/adr/0012-parameter-system.md)).
  - Third-party packages are allowed but must be pinned ([0016](docs/adr/0016-dependency-policy.md)).
- **Control and estimation**:
  - Every frame shares one control chain and differs only in its vehicle model; frames have a class and a type, and the vehicle type (ground, surface, underwater) also sets depth control and failsafe defaults ([0013](docs/adr/0013-control-chain-and-vehicle-models.md)).
  - Estimators are replaceable, and the math components form a separate library ([0014](docs/adr/0014-estimation-architecture.md)).
  - Timing is split into selectable dimensions ([0015](docs/adr/0015-selectable-timing.md)).
- **Quality and observability**:
  - Six test layers; SITL runs the same RTOS ([0018](docs/adr/0018-test-layers-and-sitl.md)).
  - Logs use the ULog format with replaceable sinks ([0019](docs/adr/0019-logging.md)).
  - Internal Flash is split into bootloader, parameter, event log and firmware regions, and is never erased while armed ([0020](docs/adr/0020-internal-flash-layout-and-write-policy.md)).
- **Language**: everything in the repository is written in English ([0021](docs/adr/0021-english-project-language.md)).

## Quick start

Vehkernel builds in a container: `cmake` and `ctest` run inside the image that `.devcontainer/Dockerfile` builds, and fail on the host ([Building](docs/guide/building.md#development-environment)). Clone with the submodules, build the image, then build and run SITL for 5 s inside it:

```bash
git clone --recurse-submodules https://github.com/chimu0510/Vehkernel.git
cd Vehkernel
docker build -t vehkernel-dev -f .devcontainer/Dockerfile .
vk() { docker run --rm -it -u "$(id -u):$(id -g)" -v "$PWD:$PWD" -w "$PWD" --network host vehkernel-dev "$@"; }
vk cmake --preset sitl
vk cmake --build --preset sitl
vk build/sitl/app/vehkernel-sitl --duration 5
```

In VS Code, "Reopen in Container" instead, and run the commands without `vk`.

To watch it live, start SITL with `--duration 0` and connect the Link monitor from another terminal:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600
```

## Documentation

| Guide | Contents |
|---|---|
| [Building](docs/guide/building.md) | Devcontainer, host tools, presets, Kconfig |
| [Flashing](docs/guide/flashing.md) | SWD, the serial flasher, rebooting into the bootloader |
| [SITL](docs/guide/sitl.md) | Options, report, motion profiles, transports |
| [Link monitor](docs/guide/link-monitor.md) | Live view, checks, attitude, parameters |
| [Driving the wheels](docs/guide/driving.md) | Safety conditions, arming, motor bring-up, driving |
| [Local display](docs/guide/local-display.md) | Pages, user key, local menu |
| [Logging](docs/guide/logging.md) | Recording, profiles, dropouts, opening logs, replay |
| [Testing](docs/guide/testing.md) | Test layers, `ctest`, bring-up image, hardware checks |
| [C50C V1.1](docs/boards/c50c-v1_1.md) | The first board: connectors, ports and wiring |

The [documentation index](docs/README.md) also lists the decision records, the milestone specs and the repository layout.

## Contributing

Contributions are welcome: bug reports, board support, drivers, tests and documentation. Start with [CONTRIBUTING.md](CONTRIBUTING.md), which covers the workflow, the coding rules and how decisions are recorded. Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md).

Report security and safety issues privately as described in [SECURITY.md](SECURITY.md), not in public issues.

## License

GNU GPL version 3 or any later version (SPDX: `GPL-3.0-or-later`); see [LICENSE](LICENSE) for the full text.
