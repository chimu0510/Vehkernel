# Vehkernel

[![CI](https://github.com/chimu0510/VehKernel/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/chimu0510/VehKernel/actions/workflows/ci.yml)
[![License: GPL-3.0-or-later](https://img.shields.io/badge/license-GPL--3.0--or--later-blue.svg)](LICENSE)

**Vehkernel** (Vehicle + Kernel) is open-source MCU firmware with ROS2 support, built as a common core for any vehicle: one system drives many vehicles and frames. The MCU handles real-time state estimation, low-level control and safety, and keeps running without a companion computer; global localization and navigation are left to ROS2 applications above it. The first test platform is a UGV.

> [!WARNING]
> Vehkernel is in early development. It has driven a vehicle only in short runs on the ground at limited output power, and its state estimate has not been checked over a drive. It drives motors: test first with the wheels off the ground, output power limited and an emergency stop within reach ([Driving the wheels](docs/guide/driving.md)). Interfaces, parameters and the wire protocol may change without notice.

## Status

Built, in SITL and on the C50C V1.1: attitude estimation, body velocity control for a wheeled vehicle, a USB gamepad, a Link to the development host, logging and replay, parameters, the local display and the bootloader.

Not built yet: the ROS2 Bridge, vehicle models other than the wheeled one, the ROS2 test layer, and boards other than the C50C V1.1.

What each milestone built and how far it was checked on hardware is in [`docs/specs/`](docs/specs/); open work is in the [issues](https://github.com/chimu0510/VehKernel/issues).

## Target hardware

Vehkernel targets the boards ArduPilot and PX4 run on, and follows their conventions for board definitions, bootloaders and tools. Boards are added one at a time, each with its own board definition, ADR and spec ([ADR-0010](docs/adr/0010-board-revision-policy.md)).

| Item | Supported now |
|---|---|
| MCU | STM32F407VET6 |
| Board | C50C V1.1, the first development board ([board page](docs/boards/c50c-v1_1.md)) |
| Companion computer | Raspberry Pi 5: the host tools bundle runs on it and talks to the board over a Link. ROS2 is planned: the Bridge does not exist yet |

## Design

- **Role**: the MCU is the companion computer's real-time coprocessor and safety gatekeeper, not an autopilot.
- **Communication**: a publish/subscribe bus is the internal backbone and ROS2 is just one Link. Messages are defined in a single schema and code is generated from it.
- **Safety**: the mode decides who is in control and the remote control can always take over. Arming checks, an emergency stop and watchdogs guard the outputs.
- **Vehicles**: every frame shares one control chain and differs only in its vehicle model.

The [ADR index](docs/adr/README.md) lists every decision and why it was made.

## Quick start

Vehkernel builds in a container: `cmake` and `ctest` run inside the image that `.devcontainer/Dockerfile` builds, and fail on the host ([Building](docs/guide/building.md#development-environment)). Clone with the submodules, build the image, then build and run SITL for 5 s inside it:

```bash
git clone --recurse-submodules https://github.com/chimu0510/VehKernel.git
cd VehKernel
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

The [documentation index](docs/README.md) lists every guide, the board pages, the decision records, the milestone specs and the repository layout. [`GLOSSARY.md`](GLOSSARY.md) defines the vocabulary. To learn the hardware one component at a time, start with the [board examples](docs/guide/examples.md).

## Contributing

Contributions are welcome: bug reports, board support, drivers, tests and documentation. Start with [CONTRIBUTING.md](CONTRIBUTING.md), which covers the workflow, the coding rules and how decisions are recorded. Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md).

Report security and safety issues privately as described in [SECURITY.md](SECURITY.md), not in public issues.

## License

GNU GPL version 3 or any later version (SPDX: `GPL-3.0-or-later`); see [LICENSE](LICENSE) for the full text.
