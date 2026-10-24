# Link monitor

`tools/link/link_monitor.py` runs on the development host and talks to the firmware over a Link (ADR-0002), in [SITL](sitl.md) or on a board. It performs the handshake and compares every topic's schema hash with the local build. In a terminal it then shows a live view, refreshed twice a second: every topic's rate and latest values, the Link statistics of both ends (the MCU's come from the `link_status` topic) and every work item's timing. Its commands set parameters, change the mode, arm, drive and reboot the vehicle.

## Connecting

**To SITL.** Start SITL without a time limit in one terminal and the monitor in another:

```bash
build/sitl/app/vehkernel-sitl --duration 0
tools/link/link_monitor.py --udp 127.0.0.1:15600
```

**To a board.** Plug a USB cable into a port that runs a Link, and the board's USB serial adapter appears on the host as a serial device, usually `/dev/ttyACM0` (your user needs the `dialout` group). Pass the Python decoders of the firmware build:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1/generated/schema/python
```

A serial Link runs at its port's `SERIALn_BAUD` parameter, which the board definition's `baud` sets by default. A changed rate takes effect at the next boot; pass the same rate to the monitor with `--baud`. A rate the port cannot reach falls back to the board default. Opening the port asserts DTR and RTS, which on a download port would hold BOOT0 high, so the monitor releases both at once; a board reset while it is connected still starts the firmware. Which ports run a Link is on the board page, for example [C50C V1.1](../boards/c50c-v1_1.md).

The monitor decodes topics with the Python module the schema generator writes into the build tree, `build/sitl/generated/schema/python` by default; pass `--msg-dir` for another build.

## Checks

With `--check` the monitor runs for `--duration` seconds, prints a summary as `key=value` lines and exits non-zero when the handshake fails, a schema hash differs, frames are lost or arrive with CRC errors, or a rate given with `--expect-rate` is missed (within 5 %, or `--tolerance`). Shortened:

```bash
tools/link/link_monitor.py --check --duration 5 --expect-rate imu_sample:0=200
```

```text
link handshake=ok version=4 mismatches=0 params=26 param_set_hash=0x71B05295612E4195 frames=2121 crc_errors=0 malformed=0 lost=0 rtt_us=4183 clock_offset_us=-45587244269
topic name=imu_sample instance=0 messages=1004 rate_hz=200.01
topic name=link_status instance=0 messages=5 rate_hz=1.00
topic name=vehicle_attitude instance=0 messages=250 rate_hz=49.95
topic name=work_item_status instance=0 messages=5 rate_hz=1.00
mcu_link instance=0 frames_sent=1666 bytes_sent=87998 frames_received=9 bytes_received=138 drops=0 crc_errors=0 malformed=0 lost=0 handshake_state=connected rate_scale=100
vehicle_status mode=STANDBY armed=0 source=none board_verified=1 emergency_stop=0 actuator_power=powered skipped_checks=none arming_refusal=none disarm_reason=none output_watchdog_trips=0 reset_cause=power_on fault=none fault_address=0x00000000
work_item name=icm20948 runs=1000 overruns=0 run_time_last_us=14 run_time_max_us=40 run_time_mean_us=16 jitter_max_us=2885
work_item name=link0 runs=1000 overruns=0 run_time_last_us=65 run_time_max_us=407 run_time_mean_us=60 jitter_max_us=2876
```

`rate_hz` here follows the MCU's timestamps, so host scheduling does not skew it. The `link` line is the development host's side of the Link, with the parameter count and parameter set hash the MCU reported at the handshake; `mcu_link` is the MCU's side, with the rate scale it sends topics at: frames the transport could not take count as `drops`, frames missing from the peer's sequence as `lost`. The check also fails when the MCU dropped frames.

The hardware check of a serial Link runs the check for a minute; it passes when `imu_sample` arrives at 200 Hz with no CRC errors and no lost frames:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1/generated/schema/python \
    --check --duration 60 --expect-rate imu_sample:0=200
```

## Attitude

The attitude estimator (ADR-0014) runs on `wq:ctrl` right after the IMU: a Mahony filter that aligns roll and pitch from the accelerometer once the vehicle has been still for 1 s (yaw starts at 0, and the gyro bias is the mean gyro reading), then publishes `vehicle_attitude` for every IMU sample. The Link sends it at 50 Hz, and the monitor shows it as roll, pitch and yaw in degrees, in the live view and as an `attitude` line in the `--check` summary; in SITL the true attitude follows ([Motion profiles](sitl.md#motion-profiles)):

```text
attitude name=vehicle_attitude instance=0 roll_deg=-0.00 pitch_deg=-0.00 yaw_deg=-0.08
attitude name=vehicle_attitude_groundtruth instance=0 roll_deg=0.00 pitch_deg=0.00 yaw_deg=0.00
```

Without a magnetometer, yaw is relative to the start and drifts slowly with the remaining gyro bias.

On a board: lift its front edge and pitch goes positive, lower its right edge and roll goes positive, turn it clockwise seen from above and yaw goes positive (body frame x forward, y right, z down). Two rotations bring the IMU's axes into the body frame (ADR-0014):

- The **mounting rotation** is how the IMU chip sits on the board, a hardware fact, set as `rotation` in the board definition and measured this way with the board's front edge forward. Correct it there only if the board itself is described wrongly.
- The **board orientation** is how the board sits in the vehicle, the `SENS_BOARD_ROT` parameter, `NONE` by default and numbered as ArduPilot's and PX4's rotation table: a board mounted turned or upside down in a vehicle needs only this parameter, which takes effect at the next boot.

## Parameters

Parameters are defined in a `params.toml` next to their module or driver (ADR-0012), for example the attitude estimator's gains and accelerometer rejection, `ATT_KP`, `ATT_KI` and `ATT_ACC_REJ`, in `modules/estimation/params.toml`. The parameter generator turns every `params.toml` the build contains into plain structs, a parameter table and a metadata file, `build/<preset>/generated/params/params.json`. A change to the estimator's parameters applies from the next IMU sample.

The `param` command lists, reads and sets parameters, describing them from that metadata file (`build/sitl/generated/params/params.json` by default, or `--param-meta`). Enum values may be given by name. It warns when the metadata does not match the parameter set hash the MCU reports at the handshake, and exits non-zero when the MCU refuses a change, printing the reason:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 param list
tools/link/link_monitor.py --udp 127.0.0.1:15600 param set ATT_KP 2.5
```

```text
param name=ATT_KP value=2.5 type=FLOAT unsaved=1 unit=1/s default=1 min=0 max=10 description="Attitude correction gain: how fast roll and pitch follow the accelerometer."
```

On a board, pass the metadata of the firmware build:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1/generated/schema/python \
    --param-meta build/c50c-v1_1/generated/params/params.json param list
```

Parameters that differ from their defaults are stored in internal Flash, entry by entry in two alternating sectors, and survive power loss and firmware updates (ADR-0012, ADR-0020). SITL keeps them in a simulated NOR Flash that follows the same rules. It starts blank at every run, so runs are reproducible; with `--param-file <path>` the Flash lives in that file and the parameters survive a restart:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --param-file /tmp/vk-params.bin
```

The live view and the `--check` summary show the `parameter_status` topic as a `param_status` line: the sector in use and its fill, changes not yet in Flash (`unsaved`), `reboot_required` changes waiting for a restart (`pending_reboot`), and stored entries skipped at boot because their parameter is gone, changed type, is out of range or was cut short by a power loss. `param list`, `param get` and `param set` mark a parameter `pending_reboot=1` or `unsaved=1` the same way.

CI runs `test/sitl/check_params.py` (the `sitl_params` test): the `param` command against SITL, refusals included; `ATT_KP` changed while the estimator runs under the shake profile; and values set over the Link found again after SITL restarts with the same `--param-file`.

The hardware check of the parameters (L4): set `ATT_KP` and `SENS_BOARD_ROT`, power-cycle the board, and `param get` returns both while the attitude follows the new board orientation; then set `SERIAL1_BAUD` to 460800, reboot, and connect with `--baud 460800`. The `param_storage` line of the work item timing shows the cost of each Flash write.

## Commands

| Command | Guide |
|---|---|
| `param list`, `param get <name>`, `param set <name> <value>` | [Parameters](#parameters) |
| `mode`, `arm`, `disarm`, `estop`, `drive`, `bringup motors`, `sim-power` | [Driving the wheels](driving.md) |
| `key short\|long\|hold <seconds>` | [Local display](local-display.md) |
| `reboot`, `reboot --bootloader` | [Flashing](flashing.md#rebooting-into-the-bootloader) |
