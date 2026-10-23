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
| `--motion <profile>` | The scripted motion of the simulated IMU ([Motion profiles](#motion-profiles)) |
| `--param-file <path>` | Keeps the simulated parameter Flash in a file ([Parameters](link-monitor.md#parameters)) |
| `--display <path>` | Writes the simulated display to a PBM image ([Local display](local-display.md)) |
| `--actuator-power-off`, `--reversed-motor <output>` | Simulated actuator faults ([Driving the wheels](driving.md)) |

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

## Motion profiles

SITL's simulated IMU follows a scripted motion chosen with `--motion`, and SITL publishes its true attitude on `vehicle_attitude_groundtruth`, which the Link also carries, so the [Link monitor](link-monitor.md) shows the estimate and the truth side by side. Each profile starts moving 3 s after boot, once the estimator has aligned, and ends by 10 s:

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

## Transports

UDP is SITL's default Link transport (ADR-0002): on `127.0.0.1:15600` by default, or the address given with `--udp`; SITL prints a `link` line with the port it bound.

To test a tool against a real serial API (termios, baud rate, framing) without hardware, start SITL with `--link pty` instead: it creates a pseudo-terminal, prints its path in the `link` line (`link transport=pty path=/dev/pts/3`), and the pty disappears when SITL exits. The monitor opens it like a board's serial port:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --link pty
tools/link/link_monitor.py --serial /dev/pts/3
```

CI runs SITL and the monitor's check together through `ctest` (the `sitl_link` and `sitl_link_pty` tests, `test/sitl/check_link.py`), which also fail when a work item, the Link's included, overruns or never runs during the check, or the pty is left behind.
