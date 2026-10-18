# Vehkernel

**Vehkernel** (Vehicle + Kernel) is open-source MCU firmware with ROS2 support, built as a common core for any vehicle: one system drives many vehicles and frames. The MCU handles real-time state estimation, low-level control and safety, and keeps running without a companion computer; global localization and navigation are left to ROS2 applications above it. The first test platform is a UGV.

**Status: first IMU sample.** Every decision is recorded in [`docs/adr/`](docs/adr/), and [`GLOSSARY.md`](GLOSSARY.md) defines the vocabulary. The firmware runs FreeRTOS with the bus and rate-tiered work queues; on the C50C V1.1 the ICM20948 driver publishes IMU samples at 200 Hz over software I2C, and SITL runs the same driver against a simulated ICM20948. Estimation, control and Links come next.

## Target hardware

| Item | Supported |
|---|---|
| MCU | STM32F407VET6 |
| Board | C50C V1.1 (ICM20948, USB gamepad, no RS-485) |
| Companion computer | Raspberry Pi 5 (ROS2) |

Other board revisions and boards get their own board definitions when needed ([ADR-0010](docs/adr/0010-board-revision-policy.md)).

## Architecture highlights

- **Role**: the MCU is the companion computer's real-time coprocessor and safety gatekeeper, not an autopilot ([0001](docs/adr/0001-mcu-responsibility-boundary.md)).
- **Communication**:
  - A publish/subscribe bus is the internal backbone and ROS2 is just one Link; a Link is made of a topic map, a protocol and a transport, all replaceable ([0002](docs/adr/0002-bus-backbone-and-pluggable-links.md)).
  - Messages are defined in a single schema and code is generated from it ([0006](docs/adr/0006-schema-and-wire-encoding.md)).
- **ROS2 interface**:
  - A vehicle-level topic API with three setpoint levels; ros2_control is optional ([0003](docs/adr/0003-vehicle-level-ros2-api.md)).
  - The mode decides who is in control, and the remote control can always take over ([0005](docs/adr/0005-setpoint-arbitration.md)).
- **Runtime**: FreeRTOS, a C++20 subset, and rate-tiered work queues ([0004](docs/adr/0004-freertos.md), [0007](docs/adr/0007-cpp20-subset.md), [0008](docs/adr/0008-bus-and-work-queues.md)).
- **Hardware support**:
  - Board definitions are written in TOML; the generator validates pins against the MCU database and solves DMA assignments ([0011](docs/adr/0011-toml-board-definition-toolchain.md)).
  - The HAL picks its form of polymorphism by when each choice is made ([0009](docs/adr/0009-layered-hal.md)).
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

## Building

The development environment is [`.devcontainer/`](.devcontainer/); every tool that affects build output is pinned ([ADR-0016](docs/adr/0016-dependency-policy.md)). Third-party C code (FreeRTOS, CMSIS, ST LL) is pinned as git submodules under `third_party/`, so clone with `git clone --recurse-submodules`, or run `git submodule update --init` in an existing clone. Open the repo with VS Code Dev Containers or the `devcontainer` CLI, then use one CMake preset per build target:

| Preset | Output |
|---|---|
| `c50c-v1_1` | C50C V1.1 firmware: `vehkernel.elf`, `vehkernel.hex` (boot stub and firmware), `vehkernel-boot.bin` (S0) and `vehkernel.bin` (firmware only, from S4) |
| `sitl` | SITL executable for the development host: `vehkernel-sitl [--duration <seconds>] [--link udp|pty] [--udp <address>:<port>]` runs the firmware on the FreeRTOS POSIX port with a Link over UDP (or a pty), stops after the duration (default 1 s, 0 runs until interrupted) and prints a report (see below) |
| `test` | Host tests, run with `ctest --preset test` |

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
```

Every firmware build prints the usage of each memory region; outgrowing a region fails the link (ADR-0017, ADR-0020).

Flash the C50C over SWD with an ST-Link on header HDR2 (pin 4 GND, pin 6 PA13/SWDIO, pin 8 PA14/SWCLK; pin 2 is 3.3 V for the probe's voltage sense only, the board stays on its own supply). OpenOCD runs on the host, since the probe is a USB device:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/c50c-v1_1/app/vehkernel.elf verify reset exit"
```

The bring-up test build (L4, [ADR-0018](docs/adr/0018-test-layers-and-sitl.md)) runs the hardware bring-up cases at boot instead of the application: it reads the ICM20948's WHO_AM_I over software I2C and measures the IMU read rate. It never configures actuator outputs, so it is safe with motors connected. Build it in the devcontainer, then run the bring-up runner on the host, which flashes it over SWD, reads the result block back and exits non-zero if a case fails or no results arrive:

```bash
cmake --preset c50c-v1_1-bringup
cmake --build --preset c50c-v1_1-bringup
tools/bringup/run_bringup.py build/c50c-v1_1-bringup/app/vehkernel.elf
```

Only the boot stub (S0) and firmware (S4 onward) sectors are written; the parameter and event log sectors in between are left untouched. The STM32 built-in USB DFU is not practical on this board, because PA11/PA12 go to the USB gamepad port, not to a Type-C port.

Kconfig decides what each build contains (ADR-0017). The layers are the MCU defaults (`mcu/<mcu>/defconfig`, or `platform/sitl/defconfig` for SITL), the board defaults (`boards/<board>/defconfig`), and a local debug overlay, `build/<preset>/debug.config`, which is never committed. Edit the overlay by hand or interactively; the next build reconfigures:

```bash
cmake --build --preset c50c-v1_1 --target menuconfig
```

SITL runs the firmware with the real ICM20948 driver reading a simulated ICM20948 on a simulated I2C bus. Run it in the devcontainer for a given time:

```bash
cmake --preset sitl
cmake --build --preset sitl
build/sitl/app/vehkernel-sitl --duration 5
```

At the end of a run SITL prints one line per topic instance and per work item, as space-separated `key=value` pairs (topics never published are listed too, with `publications=0`):

```text
link transport=udp address=127.0.0.1:15600
report duration_s=5.000
topic name=imu_sample instance=0 publications=978 rate_hz=200.01 last_update_us=5000977
topic name=work_item_status instance=0 publications=5 rate_hz=1.00 last_update_us=5000580
work_item name=icm20948 queue=wq:ctrl runs=1000 overruns=0 run_time_last_us=4 run_time_max_us=12 run_time_mean_us=2 jitter_max_us=1137
work_item name=link0 queue=wq:io runs=1000 overruns=0 run_time_last_us=5 run_time_max_us=227 run_time_mean_us=21 jitter_max_us=4447
work_item name=timing_monitor queue=wq:slow runs=5 overruns=0 run_time_last_us=2 run_time_max_us=6 run_time_mean_us=3 jitter_max_us=809
```

`rate_hz` is the average since the topic's first publication. `test/sitl/check_report.py` runs SITL and checks the report; CI runs it through `ctest` (the `sitl_smoke` test), failing when `imu_sample` is not published at 200 Hz within 5 % or any work item overruns.

### Watching live data with the Link monitor

SITL streams topics over a Link (ADR-0002): UDP on `127.0.0.1:15600` by default, or the address given with `--udp` (port 0 picks a free one; SITL prints a `link` line with the port it bound). The Link monitor on the development host connects, performs the handshake and compares every topic's schema hash with the local build. In a terminal it then shows a live view, refreshed twice a second: every topic's rate and latest values, the Link statistics of both ends (the MCU's come from the `link_status` topic) and every work item's timing. Start SITL without a time limit in one terminal and the monitor in another:

```bash
build/sitl/app/vehkernel-sitl --duration 0
tools/link/link_monitor.py --udp 127.0.0.1:15600
```

The monitor decodes topics with the Python module the schema generator writes into the build tree, `build/sitl/generated/schema/python` by default; pass `--msg-dir` for another build. With `--check` it runs for `--duration` seconds, prints a summary as `key=value` lines and exits non-zero when the handshake fails, a schema hash differs, frames are lost or a rate given with `--expect-rate` is missed:

```bash
tools/link/link_monitor.py --check --duration 5 --expect-rate imu_sample:0=200
```

```text
link handshake=ok version=1 mismatches=0 frames=1032 crc_errors=0 malformed=0 lost=0 rtt_us=5132 clock_offset_us=-213339289143
topic name=imu_sample instance=0 messages=1004 rate_hz=199.99
topic name=link_status instance=0 messages=5 rate_hz=1.00
topic name=work_item_status instance=0 messages=5 rate_hz=1.00
mcu_link instance=0 frames_sent=1050 bytes_sent=50272 frames_received=12 bytes_received=182 drops=0 crc_errors=0 malformed=0 lost=0 handshake_state=connected
work_item name=icm20948 runs=1000 overruns=0 run_time_last_us=5 run_time_max_us=11 run_time_mean_us=2 jitter_max_us=1166
work_item name=link0 runs=1000 overruns=0 run_time_last_us=31 run_time_max_us=87 run_time_mean_us=19 jitter_max_us=1156
```

UDP is SITL's default Link transport. To test a tool against a real serial API (termios, baud rate, framing) without hardware, start SITL with `--link pty` instead: it creates a pseudo-terminal, prints its path in the `link` line (`link transport=pty path=/dev/pts/3`), and the pty disappears when SITL exits. The monitor opens it like a board's serial port:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --link pty
tools/link/link_monitor.py --serial /dev/pts/3
```

`rate_hz` here follows the MCU's timestamps, so host scheduling does not skew it. The `link` line is the development host's side of the Link, `mcu_link` the MCU's: frames the transport could not take count as `drops`, frames missing from the peer's sequence as `lost`. The check also fails when the MCU dropped frames. CI runs SITL and the monitor's check together through `ctest` (the `sitl_link` and `sitl_link_pty` tests, `test/sitl/check_link.py`), which also fail when the Link work item overruns or the pty is left behind.

The `sitl` and `test` presets also run clang-tidy and treat every warning as an error. CI builds every preset and runs the tests inside the same devcontainer image.

## Directory layout

This is the initial skeleton and may change during implementation.

| Directory | Contents | ADRs |
|---|---|---|
| `docs/` | ADRs (`adr/`), milestone specs (`specs/`) and agent docs (`agents/`) | 0021 |
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
| `tools/mcudb/` | Offline converter that regenerates an MCU database from the stm32-data source committed under `mcu/<mcu>/source/` | 0011, 0016 |
| `tools/kconfig/` | Layered Kconfig solve and `menuconfig` with overlay saving | 0017 |
| `tools/generators/` | Schema (including ULog format definitions), board, parameter and log profile generators | 0006, 0011, 0012, 0019 |
| `app/` | Entry points: firmware image and SITL executable | — |
| `third_party/` | Pinned submodules: FreeRTOS kernel, CMSIS core, STM32F4 device headers and LL drivers | 0004, 0016 |
| `bridge/` | ROS2 Bridge for the companion computer | 0002, 0003 |
| `sim/` | Built-in simulator and external simulator interface | 0018 |
| `test/` | `unit/` (L1), `replay/` (L2), `sitl/` (L3), `hardware/` (L4 hardware tests), `ros2/` (L5); L0 static checks run from root config files and CI | 0018 |

Parameter definitions (`params.toml`) live in the directory of their module or driver, and log profiles live under the logging module, so they move or disappear together with their module (ADR-0012, ADR-0019).

## License

GNU GPL version 3 or any later version (SPDX: `GPL-3.0-or-later`); see [LICENSE](LICENSE) for the full text.
