# Vehkernel

**Vehkernel** (Vehicle + Kernel) is open-source MCU firmware with ROS2 support, built as a common core for any vehicle: one system drives many vehicles and frames. The MCU handles real-time state estimation, low-level control and safety, and keeps running without a companion computer; global localization and navigation are left to ROS2 applications above it. The first test platform is a UGV.

**Status: first wheel motion, awaiting its hardware check.** Every decision is recorded in [`docs/adr/`](docs/adr/), and [`GLOSSARY.md`](GLOSSARY.md) defines the vocabulary. The firmware runs FreeRTOS with the bus and rate-tiered work queues; on the C50C V1.1 the ICM20948 driver publishes IMU samples at 200 Hz over software I2C, and SITL runs the same driver against a simulated ICM20948. A Link with the custom protocol streams topics to the development host, over the Type-C port on the board and over UDP or a pty in SITL, where the Link monitor shows them live. An attitude estimator (a Mahony filter) turns the IMU samples into `vehicle_attitude`, which CI checks against SITL's ground truth. Parameters are defined in TOML, listed, read and set over the Link, and stored in internal Flash across power cycles. Modes, arming checks and an emergency stop gate four motor outputs, each with a wheel speed loop on its encoder, driven over the Link in test mode and checked in SITL; the hardware check on the C50C comes next.

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
  - A local display shows the vehicle's status, and its local menu changes whitelisted parameters only while disarmed ([0022](docs/adr/0022-local-display-and-local-menu.md)).
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

The firmware runs a Link on SERIAL1, the Type-C port (USART3 to the CH9102F USB serial adapter), 8N1 with DMA both ways, at the `SERIAL1_BAUD` parameter: 921600 baud by default, as the board definition's `baud` for SERIAL1 sets it. A changed rate takes effect at the next boot; pass the same rate to the monitor with `--baud`. A rate the port cannot reach falls back to the board default. Plug a USB cable into the Type-C port and the adapter appears on the host as a serial device, usually `/dev/ttyACM0` (your user needs the `dialout` group). Run the Link monitor on the host, with the Python decoders of the firmware build:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1/generated/schema/python
```

The hardware check of the serial Link runs the monitor's check for a minute; it passes when `imu_sample` arrives at 200 Hz with no CRC errors and no lost frames:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1/generated/schema/python \
    --check --duration 60 --expect-rate imu_sample:0=200
```

On the board, parameters are listed, read and set the same way, with the metadata of the firmware build. They are stored in sectors S1 and S2 of internal Flash, which flashing over SWD leaves untouched, so they survive power cycles and firmware updates:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1/generated/schema/python \
    --param-meta build/c50c-v1_1/generated/params/params.json param list
```

The hardware check of the parameters (L4): set `ATT_KP` and `SENS_BOARD_ROT`, power-cycle the board, and `param get` returns both while the attitude follows the new board orientation; then set `SERIAL1_BAUD` to 460800, reboot, and connect with `--baud 460800`. The `param_storage` line of the work item timing shows the cost of each Flash write.

The attitude estimate is in the same view: lift the board's front edge and pitch goes positive, lower its right edge and roll goes positive, turn it clockwise seen from above and yaw goes positive (body frame x forward, y right, z down). Two rotations bring the IMU's axes into the body frame (ADR-0014). The mounting rotation is how the IMU chip sits on the board, a hardware fact: `YAW_90` on the C50C V1.1, measured this way with the board's front edge forward and set as `rotation` in `boards/c50c-v1_1/board.toml`. Correct it there only if the board itself is described wrongly. The board orientation is how the board sits in the vehicle, the `SENS_BOARD_ROT` parameter, `NONE` by default and numbered as ArduPilot's and PX4's rotation table: a board mounted turned or upside down in a vehicle needs only this parameter, which takes effect at the next boot.

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

`rate_hz` is the average since the topic's first publication. `test/sitl/check_report.py` runs SITL and checks the report; CI runs it through `ctest` (the `sitl_smoke` test), failing when `imu_sample` is not published at 200 Hz within 5 % or any work item overruns. With the UDP Link, overruns count over a check window after start-up, and an interval in which most work items overran at once is a host stall of the whole SITL process, not counted. SITL tests run serially under `ctest -j`, since they judge timing on the shared host.

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

The attitude estimator (ADR-0014) runs on `wq:ctrl` right after the IMU: a Mahony filter that aligns roll and pitch from the accelerometer once the vehicle has been still for 1 s (yaw starts at 0, and the gyro bias is the mean gyro reading), then publishes `vehicle_attitude` for every IMU sample. The Link sends it at 50 Hz, and the monitor shows it as roll, pitch and yaw in degrees, in the live view and as an `attitude` line in the `--check` summary:

```text
attitude name=vehicle_attitude instance=0 roll_deg=-0.01 pitch_deg=0.01 yaw_deg=-0.02
```

Without a magnetometer, yaw is relative to the start and drifts slowly with the remaining gyro bias.

The filter's gains and its accelerometer rejection are parameters (ADR-0012), `ATT_KP`, `ATT_KI` and `ATT_ACC_REJ`, defined in `modules/estimation/params.toml` next to the estimator. The parameter generator turns every `params.toml` the build contains into plain structs, a parameter table and a metadata file, `build/<preset>/generated/params/params.json`. A change applies from the next IMU sample.

The Link monitor lists, reads and sets parameters over the Link with its `param` command, describing them from that metadata file (`build/sitl/generated/params/params.json` by default, or `--param-meta`). Enum values may be given by name. It warns when the metadata does not match the parameter set hash the MCU reports at the handshake, and exits non-zero when the MCU refuses a change, printing the reason:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 param list
```

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 param set ATT_KP 2.5
```

```text
param name=ATT_KP value=2.5 type=FLOAT unit=1/s default=1 min=0 max=10 description="Attitude correction gain: how fast roll and pitch follow the accelerometer."
```

Parameters that differ from their defaults are stored in internal Flash, entry by entry in two alternating sectors, and survive power loss (ADR-0012, ADR-0020). SITL keeps them in a simulated NOR Flash that follows the same rules. It starts blank at every run, so runs are reproducible; with `--param-file <path>` the Flash lives in that file and the parameters survive a restart:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --param-file /tmp/vk-params.bin
```

The monitor's live view and `--check` summary show `parameter_status`: the sector in use and its fill, changes not yet in Flash (`unsaved`), `reboot_required` changes waiting for a restart (`pending_reboot`), and stored entries skipped at boot because their parameter is gone, changed type, is out of range or was cut short by a power loss. `param list` and `param get` mark a parameter `pending_reboot=1` or `unsaved=1` the same way.

CI runs `test/sitl/check_params.py` (the `sitl_params` test): the `param` command against SITL, refusals included; `ATT_KP` changed while the estimator runs under the shake profile; and values set over the Link found again after SITL restarts with the same `--param-file`.

SITL's simulated IMU follows a scripted motion chosen with `--motion`, and SITL publishes its true attitude on `vehicle_attitude_groundtruth`, which the Link also carries, so the monitor shows the estimate and the truth side by side. Each profile starts moving 3 s after boot, once the estimator has aligned, and ends by 10 s:

| Profile | Motion |
|---|---|
| `still` | Level and at rest (the default) |
| `tilt` | 20° of roll, then 20° of pitch, at 20°/s |
| `rotate` | 90° turns about x, y and z in turn, at 45°/s |
| `shake` | Level, with 0.5 s bursts of 6 m/s² forward acceleration every 1.5 s |

```bash
build/sitl/app/vehkernel-sitl --duration 0 --motion rotate
```

CI checks the estimate against the truth with `test/sitl/check_attitude.py` (the `sitl_attitude_tilt`, `sitl_attitude_rotate` and `sitl_attitude_shake` tests): the tilt error (roll and pitch) must stay within 2° and the whole attitude error, which bounds yaw, within 5°.

UDP is SITL's default Link transport. To test a tool against a real serial API (termios, baud rate, framing) without hardware, start SITL with `--link pty` instead: it creates a pseudo-terminal, prints its path in the `link` line (`link transport=pty path=/dev/pts/3`), and the pty disappears when SITL exits. The monitor opens it like a board's serial port:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --link pty
tools/link/link_monitor.py --serial /dev/pts/3
```

`rate_hz` here follows the MCU's timestamps, so host scheduling does not skew it. The `link` line is the development host's side of the Link, `mcu_link` the MCU's: frames the transport could not take count as `drops`, frames missing from the peer's sequence as `lost`. The check also fails when the MCU dropped frames. CI runs SITL and the monitor's check together through `ctest` (the `sitl_link` and `sitl_link_pty` tests, `test/sitl/check_link.py`), which also fail when the Link work item overruns or the pty is left behind.

### Driving the wheels

Actuators move only in test mode, armed, and only on direct-actuator setpoints: one wheel speed in rad/s per motor output (ADR-0005, ADR-0018). Until a remote control exists, these conditions apply:

- The vehicle boots in `STANDBY`; `mode test` makes the Link that asked the source in control. The mode changes only while disarmed.
- `arm` runs the arming checks in turn and prints the first that fails: the board verified at boot (the IMU answered and, on the C50C, the revision-ID pins read 0), the emergency stop released, `TEST`, the source in control, no setpoint still active, no bring-up case running, actuator power present on boards that sense it, and actuator control started.
- `estop on` from any connected Link forces every output off at once. Released within 5 s with `estop off`, the wheels resume from the next setpoint; engaged for 5 s, the vehicle disarms.
- Setpoints that stop for longer than `COM_SP_TIMEOUT` (0.5 s) stop and disarm the vehicle; an armed vehicle whose targets stay zero or unset for `COM_DISARM_IDLE` (10 s) disarms.
- Outputs are capped at `MOT_OUT_MAX` (30 % duty by default) and change by at most `MOT_SLEWRATE` of full duty per second, though a stop is never slowed. A watchdog turns them off when actuator control stalls for three control periods.

The C50C reads its drive power circuit on `PE4`: arming needs it on for 0.5 s, and losing it disarms. On the bench, with the sense line unwired, skip that check with `param set COM_ARM_CHECK 0`; the vehicle status then reports it skipped, and `arm` warns. Boards without such an input skip it on their own, so make sure the motor power can be cut by hand.

Lift the wheels off the ground first. Then check that each motor turns its own encoder the right way: the bring-up case pulses each output in turn and prints, for any that fails, which parameter to change. `MOTn_REV` and `MOTn_ENC_REV` are locked while armed, so disarm before changing them:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 mode test
tools/link/link_monitor.py --udp 127.0.0.1:15600 arm
tools/link/link_monitor.py --udp 127.0.0.1:15600 bringup motors
```

```text
bringup output=1 result=pass encoder=1 sign=+1
bringup output=2 result=reversed encoder=2 sign=-1  fix: set MOT2_REV=REVERSED if the wheel turned backwards, MOT2_ENC_REV=REVERSED if it turned forwards
```

`drive` arms if needed, sends the wheel speeds at 20 Hz for the given time, then stops and disarms:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 drive 5,5,5,5 --duration 3
```

The live view shows `vehicle_status`, `wheel_encoders` and `actuator_outputs`. On the board, use `--serial /dev/ttyACM0` instead of `--udp`. SITL simulates four motors and encoders; `--actuator-power-off` starts it with the simulated power cut, `sim-power on|off` cuts or restores it while it runs, and `--reversed-motor <output>` wires a motor the wrong way round. CI drives the simulated wheels through every one of these paths (`sitl_arming`, `sitl_actuators` and `sitl_bringup`).

### Local display

The local display shows the vehicle's status on the board's display, three pages of 21 columns by 8 lines refreshed at most five times a second (ADR-0022): status (mode, arming, the emergency stop, board verification, actuator power, skipped arming checks, the last arming refusal and disarm reason, output watchdog trips), attitude (roll, pitch and yaw in degrees) and Link (each Link's handshake, rate scale, drops and lost frames). SITL simulates an SSD1306; `--display <path>` writes what it shows to a PBM image after every change, which any image viewer opens:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --display /tmp/vk-display.pbm
```

CI reads the status page back from that image (`sitl_display`).

The user key drives the display and the local menu (ADR-0022). A short press (released before 1 s) switches pages, armed or disarmed. A long press (held 1 s) while disarmed opens the menu, and **arming is refused with "local menu open" while it is open**. The menu lists the menu parameters by name, then EXIT; the value of the one under the cursor is on the bottom line:

- In the list, a short press moves to the next item; a long press edits it, or on EXIT closes the menu.
- In an edit, the screen shows the current value (NOW) and a staged value (NEW). A short press moves the caret through `+`, `-`, `DEF`, `SAVE` and `CANCEL`; a long press runs the one under it. `+` and `-` step by the parameter's menu step within its range, or through an enum's values, and repeat every 0.2 s while the key stays held. `DEF` stages the default. Nothing is written until `SAVE`, which goes through the same parameter checks as a Link and shows `SAVED` or why it was refused; every connected Link sees the change. A changed parameter that needs a reboot shows `REBOOT`.
- The menu closes by itself after 30 s without a key press, discarding a staged edit, and at once if the vehicle is armed.

The menu parameters are `SENS_BOARD_ROT`, `SERIAL1_BAUD` (on the board), `ATT_KP`, `ATT_KI` and `ATT_ACC_REJ`; a parameter enters the menu only with `menu = true` and a `menu_step` in its definition. Nothing in the `MOT` or `COM` groups ever does: the menu never arms, changes the mode, touches the emergency stop or moves an actuator. Changing `SERIAL1_BAUD` from the menu recovers a board set to a rate the host cannot use.

In SITL the Link monitor presses the simulated user key, then prints the menu's state, which the live view also shows:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 key long
tools/link/link_monitor.py --udp 127.0.0.1:15600 key short
tools/link/link_monitor.py --udp 127.0.0.1:15600 key hold 5
```

CI opens the menu, saves parameters, sees arming refused while it is open and the menu time out (`sitl_local_menu`).

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
