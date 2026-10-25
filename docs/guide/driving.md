# Driving the wheels

Actuators move only in test mode, armed, and only on direct-actuator setpoints: one wheel speed in rad/s per motor output (ADR-0005, ADR-0018). The commands below use the [Link monitor](link-monitor.md) against SITL; on a board, use `--serial /dev/ttyACM0` instead of `--udp`, with the firmware build's `--msg-dir` and `--param-meta` ([Connecting](link-monitor.md#connecting)).

## Safety conditions

Until a remote control exists, these conditions apply:

- The vehicle boots in `STANDBY`; `mode test` makes the Link that asked the source in control. The mode changes only while disarmed.
- `arm` runs the arming checks in turn and prints the first that fails: the board verified at boot (the IMU answered and, on boards that have them, the revision-ID pins read their value), the emergency stop released, `TEST`, the source in control, no setpoint still active, no bring-up case running, actuator power present on boards that sense it, actuator control started, the local menu closed, and no watchdog or fault reset since the last deliberate reset (`reboot`, the reset pin or a power cycle). A reboot already accepted refuses arming before any of these.
- `estop on` from any connected Link forces every output off at once. Released within 5 s with `estop off`, the wheels resume from the next setpoint; engaged for 5 s, the vehicle disarms.
- Setpoints that stop for longer than `COM_SP_TIMEOUT` (0.5 s) stop and disarm the vehicle; an armed vehicle whose targets stay zero or unset for `COM_DISARM_IDLE` (10 s) disarms.
- Outputs are capped at `MOT_OUT_MAX` (30 % duty by default) and change by at most `MOT_SLEWRATE` of full duty per second, though a stop is never slowed. A watchdog turns them off when actuator control stalls for three control periods. A fault, a failed assertion or a stack overflow turns every output off at once and the hardware watchdog then resets the board, as does a scheduler that stops getting idle; `vehicle_status` and the local display report the reset cause after boot (ADR-0005).

## Actuator power

On a board that senses its actuator power circuit, arming needs it on for 0.5 s, and losing it disarms. On the bench, with the sense line unwired, skip that check with `param set COM_ARM_CHECK 0`; the vehicle status then reports it skipped, and `arm` warns. Boards without such an input skip it on their own, so make sure the motor power can be cut by hand. The C50C senses it on `PE4` ([C50C V1.1](../boards/c50c-v1_1.md)).

## Motor bring-up

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

## Vehicle description

The vehicle model turns a body motion demand into wheel speeds, and the wheel speeds back into the body velocity (ADR-0013). Parameters describe the vehicle; all are `reboot_required`, read once at boot, when the model is selected:

| Parameter | Meaning |
|---|---|
| `VEHICLE_TYPE` | The environment: `GROUND` |
| `FRAME_CLASS` | The allocation math, which selects the vehicle model: `WHEELED` |
| `FRAME_TYPE` | The geometry: `DIFF_2WD` (a left and a right wheel) or `SKID_4WD` (a front and a rear pair, skid-steered) |
| `WHL_RADIUS`, `WHL_TRACK` | The wheel radius and the track width, in metres |
| `WHL_LEFT_OUT`, `WHL_RIGHT_OUT` | The output that drives the left and the right wheel; the front pair for `SKID_4WD` |
| `WHL_REAR_L_OUT`, `WHL_REAR_R_OUT` | The output of each rear wheel, `SKID_4WD` only |

Outputs the frame does not use stay uncontrolled. A description no compiled-in model can drive, such as two wheels on one output, selects no model. Only frame classes and types whose model is compiled in (Kconfig `VEHICLE_MODEL_*`) are offered. The body frame is x forward, y right and z down, so a positive yaw rate turns right: on `DIFF_2WD` the left wheel then runs faster than the right by `yaw rate × WHL_TRACK / WHL_RADIUS`.

## Driving

`drive` requests `TEST` and arms if needed, sends one wheel speed in rad/s per output at 20 Hz for the given time (`nan` leaves an output uncontrolled; put `--` before a negative first speed), then stops and disarms:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 drive 5,5,5,5 --duration 3
```

The live view shows `vehicle_status`, `wheel_encoders` and `actuator_outputs`.

## Fault stop check

The hardware check of ADR-0005: a fault while driving turns every output off within one interrupt, and the board resets and reports why. Lift the wheels off the ground and limit the motor supply. Build the firmware with a fault test in the local debug overlay, `build/c50c-v1_1/debug.config` ([Building](building.md)), one of:

```text
CONFIG_FAULT_TEST_ASSERT=y
CONFIG_FAULT_TEST_USAGE_FAULT=y
CONFIG_FAULT_TEST_STALL=y
```

Flash it, disconnect any debugger (a debugger holds the core at the fault, and its halt also freezes the PWM timers, which turns the outputs off by itself), and drive. Half a second after the wheels start, `wq:ctrl` fails an assertion, executes an undefined instruction, or stalls; on a scope or by eye every PWM output drops to zero at once (for the stall, within three control periods, through the output watchdog), and about 2 s later the board resets. After boot `vehicle_status` reports `reset_cause=fault fault=assert` (or `usage_fault`), or `reset_cause=watchdog` for the stall, the local display shows `RST FAULT` or `RST WDOG`, and `arm` is refused as an unexpected reset until `reboot`. Remove the line from `debug.config` afterwards.

## In SITL

SITL simulates four motors and encoders; `--actuator-power-off` starts it with the simulated power cut, `sim-power on|off` cuts or restores it while it runs, and `--reversed-motor <output>` wires a motor the wrong way round. CI drives the simulated wheels through every one of these paths (`sitl_arming`, `sitl_actuators` and `sitl_bringup`), and through the fault stop with `--fault` ([SITL](sitl.md#faults-and-reset-cause), `sitl_fault`).
