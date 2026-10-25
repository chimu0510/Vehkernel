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

### Body velocity scenarios

CI checks spec 0009 against the ground vehicle's truth with these scenarios, each driven over the Link. Every tolerance is set where a correct firmware passes with margin and a sign error, a wrong geometry or a missing loop fails; each script gives the same reasons in its header.

| Test | Scenario | Tolerance, and why |
|---|---|---|
| `sitl_vehicle` (`check_vehicle.py`) | Direct-actuator setpoints: forward, a turn in place, and forward with `--slip 0.25` | Distance and heading change within 2 % of what the wheels turned, slip 75 % of it: room for encoder steps and interpolating the 50 Hz truth. The estimated yaw within 3° of the true heading, through the gyro alone |
| `sitl_body_velocity` (`check_body_velocity.py`) | Body velocity refused in `STANDBY`; in `TEST`, the kinematics' wheel speed targets; direct-actuator and body velocity setpoints switched; the setpoint timeout; the `drive-body` command | Targets exact to 0.001 rad/s, with the yaw rate loop opened (P and I at 0); wheel speeds' mean within 5 %: about 32 encoder counts a cycle at 2 rad/s. No sample may mix the levels |
| `sitl_body_motion` (`check_body_motion.py`) | A circle at 0.3 m/s and 0.5 rad/s, with and without `--slip 0.25`; 0.4 m/s at 2 rad/s with `MOT_MAX_SPD` 10 | Yaw rate mean within 3 % from 4 s on, once the loop's integral has settled: the wheels alone would be 25 % short with slip. Saturated: curvature within 5 %, vx and yaw rate reported limited, scale near 5/6 |
| `sitl_odometry` (`check_odometry.py`) | 0.6 m, a 90° turn in place, 0.6 m; turning in place with `--slip 0.25` | Odometry within 3 % of the distance plus 2 cm and 3° of the truth, measured speed within 2 %. With slip, its heading within 3° while the wheels' heading runs a third ahead |
| `sitl_routes` (`check_routes.py`) | Forward 1 m at 0.25 m/s; a square of 0.5 m sides and right turns in place | Forward: within 5 % of 1 m, feedforward with the wheels' 0.1 s lag at either end; odometry within 3 % of the truth; heading within 2°. Square: the odometry ends within 5 cm and 3° of the truth, and the truth turned a full circle within 15° |

A source not in control cannot send setpoints in these scenarios, since SITL has a single Link; the Link work item's unit tests check that refusal with two Links.

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
