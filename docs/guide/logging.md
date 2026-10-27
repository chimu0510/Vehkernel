# Logging

Vehkernel records telemetry and sensor data in the standard ULog format (`.ulg`) over a Link (ADR-0019, spec 0011). A log profile selects which topics are streamed and at what rates, and standard tools such as [pyulog](https://github.com/PX4/pyulog), [PlotJuggler](https://plotjuggler.io) and Lichtblick open the resulting files directly. Replay (L2, ADR-0018) reruns the firmware's estimator work items on the development host faster than real time, reproducing the recorded estimate and enabling fair comparison against ground truth or alternative algorithms.

## Recording over a Link

The Link monitor's `record` command connects to SITL or a hardware board, performs the handshake, reads the vehicle's parameters, requests logging to start, and writes the stream to a `.ulg` file on the host. Run the commands in the development container ([Building](building.md#development-environment); with Docker, prefix each command with `vk`), SITL in one terminal and the recorder in another:

```bash
build/sitl/app/vehkernel-sitl --duration 0
tools/link/link_monitor.py --udp 127.0.0.1:15600 record log.ulg --profile estimator_replay --duration 10
```

On a serial connection to hardware (for example [C50C V1.1](../boards/c50c-v1_1.md)), run the recorder on the host, which has the board's USB devices ([Container or host](building.md#container-or-host)):

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1/generated/schema/python \
    record log.ulg --profile estimator_replay --duration 60
```

The recorder stops when `--duration` expires or on `Ctrl-C`, and fails with exit code 1 when the serial port goes away. The MCU stops logging when the recorder asks it to, and on its own when the host stays silent past the Link's peer timeout.

CI records SITL for 30 s with `test/sitl/check_log.py` (the `sitl_log` test): pyulog must open the file, it must hold no dropouts, and every profile topic SITL publishes must arrive within 5 % of its rate.

### What is written to the log

Before streaming high-rate data, the recorder writes:
- **Formats (`F`)**: message layouts generated from schema definitions (`.msg`).
- **Info (`I`)**: from the MCU's `log_info`, the board name (`sys_name`), `board_id`, the firmware version (`ver_sw`), the git commit hash (`ver_git`) and the profile's `bandwidth`; and the schema hash of every topic the MCU declared at the handshake (`sys_schema_<topic>`).
- **Initial parameters (`P`)**: every parameter read from the MCU before logging starts.

During the session:
- **Topic data (`D`)**: messages with their original MCU timestamps in microseconds.
- **Parameter updates (`P`)**: written dynamically at the recorded timestamp when any parameter changes on the MCU.
- **Dropouts (`O`)**: transport frame sequence gaps or MCU buffer overruns.
- **Sync markers (`S`)**: written periodically every second to keep files seekable.

## Log profiles

Log profiles are declared in `schema/log_profiles.toml` and emitted by the profile generator (ADR-0019). The MCU refuses `log_start` while armed, when an unknown profile is requested, when another Link is already logging, or when the profile's estimated bandwidth together with the Link's status streams (those at 1 Hz or less) exceeds 80 % of the port capacity. While logging, the normal topic map leaves out the topics the profile carries and is scaled into what remains of the port.

The `estimator_replay` profile records estimator inputs and outputs:

| Topic | Rate | Notes |
|---|---|---|
| `imu_sample:0` | `every_sample` (200 Hz) | Gyro and accelerometer inputs |
| `body_velocity_measurement:0` | `every_sample` (200 Hz) | Wheel encoder velocity inputs |
| `vehicle_attitude` | 50 Hz | Mahony filter attitude estimate |
| `vehicle_local_odometry` | 50 Hz | Local dead-reckoning estimate |
| `actuator_outputs` | 50 Hz | The duty applied to each output and its speed target |
| `vehicle_status` | 5 Hz (`as_published`) | The mode, arming and the vehicle's health |
| `work_item_status` | 1 Hz (`as_published`) | Each work item's timing statistics |
| `link_status` | 1 Hz (`as_published`) | Link transmission statistics |
| `parameter_update` | `as_published` | Notifies when a parameter changes |
| `estimator_restart` | `as_published` | Published once when estimators restart |
| `vehicle_attitude_groundtruth` | 50 Hz (SITL only) | True attitude in simulation |
| `vehicle_local_odometry_groundtruth` | 50 Hz (SITL only) | True pose in simulation |

On the C50C the profile takes about 33.9 KB/s with the Link's framing, about 45 % of SERIAL1 at 921600 baud together with the normal topic map; SITL's ground truth topics bring it to about 39.8 KB/s.

## Stillness after starting

When logging begins, the MCU Link sends `RESTART_ESTIMATORS` on `vehicle_command`. The commander accepts this only while disarmed and publishes `estimator_restart`:
1. The attitude estimator returns to its aligning state and drops any IMU sample taken before the restart. It collects accelerometer and gyro samples to determine initial tilt and calibrate gyro biases. This alignment requires the vehicle to remain stationary for **1.0 second**.
2. Local odometry clears its accumulated position and fixes its coordinate origin at the position where the attitude estimator finishes aligning.

Keep the vehicle still for at least a second after starting a log, before arming or driving; 1.5 s leaves a margin. Moving it while the attitude estimator aligns spoils the gyro bias and the initial tilt, and every estimate after them.

## Dropouts

A logged stream tracks publication sequence numbers and queue generations:
- If a high-rate topic is published faster than the Link can transmit, the MCU emits a `log_dropout` control frame specifying the topic, instance and count of dropped samples.
- If frames are lost on the physical transport (e.g. over serial or UDP), the host recorder detects gaps in the frame sequence.

Both cases are recorded as ULog dropout (`O`) messages indicating the gap duration in milliseconds. A clean log records zero dropouts.

## Opening and inspecting logs

### pyulog

Standard `pyulog` tools inspect and extract data from recorded `.ulg` files.

View metadata, info messages, message counts and dropouts:

```bash
ulog_info log.ulg
```

Print recorded parameters:

```bash
ulog_params log.ulg
```

Export topics to CSV files:

```bash
ulog2csv log.ulg
```

### PlotJuggler

[PlotJuggler](https://plotjuggler.io) loads Vehkernel ULog files natively through its ULog plugin.

1. Launch PlotJuggler:
   ```bash
   plotjuggler
   ```
2. Select **File > Load Data** and choose your `.ulg` file.
3. Drag curves from the topic tree into plotting panes (for example, `vehicle_attitude/q[0..3]` and `vehicle_local_odometry/position[0..1]`).

## Estimator replay and comparison

The replay toolchain (spec 0011, ADR-0018 L2) executes the firmware's estimator algorithms on recorded data on the development host:
- `test/replay/main.cpp` (`vehkernel-replay`) links `AttitudeEstimator` and `LocalOdometry` unchanged on a work queue running against a simulated replay clock.
- `tools/log/replay.py` feeds initial parameters and logged sensor inputs (`imu_sample`, `body_velocity_measurement`, `parameter_update`, `estimator_restart`) from a recorded log into the replay binary and writes its output to a new `.ulg` file.
- `tools/log/compare.py` compares the recorded and replayed files.

### Running replay

Replay a recorded log to an output log:

```bash
tools/log/replay.py log.ulg replayed.ulg
```

Compare the replayed log against the recorded log:

```bash
tools/log/compare.py log.ulg replayed.ulg
```

### Pass criteria

1. **Faithfulness**:
   - For every `vehicle_attitude` and `vehicle_local_odometry` sample in the evaluation window (from `estimator_restart` onward), the replayed sample with the same `timestamp_sample` must match the recorded sample field-for-field.
   - For SITL logs, where the replay binary and SITL executable share the same host toolchain, the match is exact (`--tolerance 0.0`).
   - For board logs, differences due to hardware FPU fused multiply-add and newlib `libm` are accepted within a tight measured tolerance.
2. **Estimation metrics**:
   - `compare.py` aligns the logged ground truth into the local odometry frame at the restart point and computes the mean and maximum attitude error, final position error, and final heading error.
   - CI runs `sitl_replay` (`test/sitl/check_replay.py`), which records a SITL square drive over UDP, replays it, checks exact faithfulness, verifies zero dropouts, and verifies that the odometry error is within `check_odometry.py`'s bounds against ground truth.
