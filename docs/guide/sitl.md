# SITL

SITL runs the firmware on the FreeRTOS POSIX port on the development host (ADR-0018), with the real drivers reading simulated devices: the ICM20948 driver reads a simulated ICM20948 on a simulated I2C bus. Build and run it in the devcontainer:

```bash
cmake --preset sitl
cmake --build --preset sitl
build/sitl/app/vehkernel-sitl --duration 5
```

## Options

| Option | Effect |
|---|---|
| `--duration <seconds>` | Stops after the duration and prints a report; default 1 s, 0 runs until interrupted |
| `--link udp\|pty` | The Link transport, UDP by default ([Transports](#transports)) |
| `--udp <address>:<port>` | The UDP address, `127.0.0.1:15600` by default; port 0 picks a free one |
| `--motion <profile>` | What moves the simulated IMU: `vehicle`, the default ([Ground vehicle](#ground-vehicle)), or a scripted profile ([Motion profiles](#motion-profiles)) |
| `--slip <fraction>` | The fraction of its wheels' motion the ground vehicle loses, 0 by default ([Ground vehicle](#ground-vehicle)) |
| `--param-file <path>` | Keeps the simulated parameter Flash in a file ([Parameters](link-monitor.md#parameters)) |
| `--display <path>` | Writes the simulated display to a PBM image ([Local display](local-display.md)) |
| `--actuator-power-off`, `--reversed-motor <output>` | Simulated actuator faults ([Driving the wheels](driving.md)) |
| `--fault assert\|fail` | Fails a `configASSERT` or calls `fail()` once an output has been driven for 0.5 s ([Faults and reset cause](#faults-and-reset-cause)) |
| `--reset-cause <cause>[:<fault>]` | The reset cause the run reports, `power_on` by default ([Faults and reset cause](#faults-and-reset-cause)) |

## Report

At the end of a run SITL prints one line per topic instance and per work item, as space-separated `key=value` pairs (topics never published are listed too, with `publications=0`). Shortened:

```text
link transport=udp address=127.0.0.1:15600
report duration_s=5.000
topic name=actuator_setpoint instance=0 publications=0 rate_hz=0.00 last_update_us=0
topic name=imu_sample instance=0 publications=979 rate_hz=200.00 last_update_us=5000196
topic name=vehicle_attitude instance=0 publications=778 rate_hz=200.04 last_update_us=5000240
topic name=work_item_status instance=0 publications=5 rate_hz=1.00 last_update_us=5000532
work_item name=icm20948 queue=wq:ctrl runs=1000 overruns=0 run_time_last_us=29 run_time_max_us=106 run_time_mean_us=16 jitter_max_us=2113
work_item name=mahony_attitude queue=wq:ctrl runs=979 overruns=0 run_time_last_us=14 run_time_max_us=31 run_time_mean_us=10 jitter_max_us=59
work_item name=commander queue=wq:ctrl runs=1000 overruns=0 run_time_last_us=15 run_time_max_us=26 run_time_mean_us=10 jitter_max_us=2156
work_item name=local_display queue=wq:slow runs=100 overruns=0 run_time_last_us=24 run_time_max_us=53 run_time_mean_us=20 jitter_max_us=1125
work_item name=link0 queue=wq:io runs=1000 overruns=0 run_time_last_us=9 run_time_max_us=26 run_time_mean_us=9 jitter_max_us=2900
work_item name=timing_monitor queue=wq:slow runs=5 overruns=0 run_time_last_us=35 run_time_max_us=47 run_time_mean_us=36 jitter_max_us=949
```

`rate_hz` is the average since the topic's first publication. `test/sitl/check_report.py` runs SITL and checks the report; CI runs it through `ctest` (the `sitl_smoke` test), failing when `imu_sample` is not published at 200 Hz within 5 %, or any work item overruns or never runs. With the UDP Link, overruns count over a check window after start-up, and an interval in which most work items overran at once is a host stall of the whole SITL process, not counted. SITL tests run serially under `ctest -j`, since they judge timing on the shared host.

## Faults and reset cause

SITL has no hardware watchdog and no real faults, but runs the fault stop of ADR-0005 for a failed `configASSERT` and for `fail()`: it turns every simulated output off, prints one line and ends the run at once with exit status 12:

```text
fault kind=ASSERT address=0xc2a4e1f3 duty=0.000,0.000,0.000,0.000
```

`--fault assert` or `--fault fail` raises one on purpose once an output has been driven for 0.5 s. A run starts as if powered on; whoever starts SITL again after a reset or a fault gives the reset cause the board would read at boot, with `--reset-cause watchdog` or `--reset-cause fault:assert`, for example. `vehicle_status` then reports it, and after a watchdog or fault reset arming is refused as an unexpected reset; `--reset-cause software` stands for the deliberate reboot that clears it. CI checks both with `test/sitl/check_fault.py` (the `sitl_fault` test).

## Ground vehicle

By default the simulated board sits in a ground vehicle that its motors move (ADR-0018). The vehicle is the one the firmware's vehicle description gives, read at boot from the same parameters: the frame type, `WHL_RADIUS`, `WHL_TRACK` and the output of each wheel ([Vehicle description](driving.md#vehicle-description)). The wheels' true speeds move it through the frame's wheel kinematics; the IMU's gyro feels its turns and its accelerometer its accelerations. A description no model can drive leaves the board at rest.

`--slip 0.25` makes the vehicle move and turn only 75 % as far as its wheels say, as when its tyres skid; the wheel encoders still read the wheels.

SITL publishes the vehicle's true pose and velocity on `vehicle_local_odometry_groundtruth`: position in the local level frame (north-east-down, origin and heading zero at boot), attitude, and velocity and yaw rate in the body frame. Its true attitude also goes out on `vehicle_attitude_groundtruth`. The Link carries both at 50 Hz.

`test/sitl/check_body_velocity.py` (`sitl_body_velocity`) drives it by body velocity setpoints: refused in `STANDBY`, the wheel speed targets of the kinematics in `TEST`, never both setpoint levels at once, the setpoint timeout, and the Link monitor's `drive-body`.

`test/sitl/check_body_motion.py` (`sitl_body_motion`) checks the control chain against the true motion: on a circle at 0.3 m/s and 0.5 rad/s, with and without `--slip 0.25`, the true yaw rate's mean from 4 s on must be within 3 % of the commanded rate (the wheels alone would be 25 % short with slip, so only the gyro loop passes it), and a twist beyond the wheels' reach must keep its curvature within 5 % while `allocation_status` reports the limited axes.

CI drives the vehicle with direct-actuator setpoints in `test/sitl/check_vehicle.py` (`sitl_vehicle`): driving forward, the true distance must match the wheels' turn times `WHL_RADIUS` within 2 %; turning in place, the true heading change must match `WHL_RADIUS × (left - right) / WHL_TRACK` within 2 %, and the attitude estimator's yaw must follow the true heading within 3°; with `--slip 0.25`, the distance must be 75 % of the wheels' within 2 %.

## Motion profiles

With `--motion` and a profile, SITL's simulated IMU follows a scripted motion instead, the vehicle is not simulated, and SITL publishes its true attitude on `vehicle_attitude_groundtruth`, which the Link also carries, so the [Link monitor](link-monitor.md) shows the estimate and the truth side by side. Each profile starts moving 3 s after boot, once the estimator has aligned, and ends by 10 s:

| Profile | Motion |
|---|---|
| `still` | Level and at rest |
| `tilt` | 20° of roll, then 20° of pitch, at 20°/s |
| `rotate` | 90° turns about x, y and z in turn, at 45°/s |
| `shake` | Level, with 0.5 s bursts of 6 m/s² forward acceleration every 1.5 s |

```bash
build/sitl/app/vehkernel-sitl --duration 0 --motion rotate
```

CI checks the estimate against the truth with `test/sitl/check_attitude.py` (the `sitl_attitude_tilt`, `sitl_attitude_rotate` and `sitl_attitude_shake` tests): the tilt error (roll and pitch) must stay within 2° and the whole attitude error, which bounds yaw, within 5°.

## Transports

UDP is SITL's default Link transport (ADR-0002): on `127.0.0.1:15600` by default, or the address given with `--udp`; SITL prints a `link` line with the port it bound.

To test a tool against a real serial API (termios, baud rate, framing) without hardware, start SITL with `--link pty` instead: it creates a pseudo-terminal, prints its path in the `link` line (`link transport=pty path=/dev/pts/3`), and the pty disappears when SITL exits. The monitor opens it like a board's serial port:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --link pty
tools/link/link_monitor.py --serial /dev/pts/3
```

CI runs SITL and the monitor's check together through `ctest` (the `sitl_link` and `sitl_link_pty` tests, `test/sitl/check_link.py`), which also fail when a work item, the Link's included, overruns or never runs during the check, or the pty is left behind.
