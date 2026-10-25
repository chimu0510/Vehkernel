# Driving the wheels

Actuators move only in test mode, armed, on one of two setpoint levels (ADR-0005, ADR-0018): direct-actuator setpoints, one wheel speed in rad/s per motor output, or body velocity setpoints, the motion of the vehicle as a whole, which the control chain allocates to the wheels through the vehicle model (ADR-0013). The commands below use the [Link monitor](link-monitor.md) against SITL; on a board, use `--serial /dev/ttyACM0` instead of `--udp`, with the firmware build's `--msg-dir` and `--param-meta` ([Connecting](link-monitor.md#connecting)).

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

A positive wheel speed must roll the vehicle forward on every side. The motor bring-up case only makes each motor agree with its own encoder, so check the direction by eye with `drive-body` and the wheels lifted. On a side whose motor is mounted mirrored, which turns it the other way for forward, flip the output as a whole: both `MOTn_REV` and `MOTn_ENC_REV`. On the C50C's four-wheel differential swing-arm vehicle that is output 2, the right wheel.

Outputs the frame does not use stay uncontrolled. A description no compiled-in model can drive, such as two wheels on one output, selects no model. Only frame classes and types whose model is compiled in (Kconfig `VEHICLE_MODEL_*`) are offered. The body frame is x forward, y right and z down, so a positive yaw rate turns right: on `DIFF_2WD` the left wheel then runs faster than the right by `yaw rate × WHL_TRACK / WHL_RADIUS`.

## Driving

`drive` requests `TEST` and arms if needed, sends one wheel speed in rad/s per output at 20 Hz for the given time (`nan` leaves an output uncontrolled; put `--` before a negative first speed), then stops and disarms:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 drive 5,5,5,5 --duration 3
```

`drive-body` does the same with a body velocity: the forward and right speed in m/s and the yaw rate in rad/s, positive turning right. `vy` is ignored by frames that cannot move sideways. The vehicle model turns it into a wheel speed target per output, scaled down uniformly when a wheel would exceed `MOT_MAX_SPD`, so the path keeps its curvature:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 drive-body 0.3,0,0.5 --duration 3
```

The yaw rate is closed on the gyro, a PI loop with feedforward (`RATE_YAW_P`, `RATE_YAW_I`, `RATE_YAW_FF`), so the vehicle turns at the commanded rate even when its wheels slip. With the wheels lifted the vehicle cannot turn, so set `RATE_YAW_P` and `RATE_YAW_I` to 0 for a lifted check, or the loop winds up against it. `allocation_status` shows the demand, the scale factor and the limited axes.

The two levels never drive at once: the one whose setpoint arrived last drives, until it times out. Every safety condition above holds for body velocity setpoints as for direct-actuator ones.

The live view shows `vehicle_status`, `wheel_encoders` and `actuator_outputs`, and the odometry below.

## Odometry

The vehicle model measures the body velocity from the wheel speeds and publishes it on `body_velocity_measurement` (ADR-0014), with its variance: each wheel's encoder quantization and the residual, the rim speed the wheels disagree by, which grows with slip. Local odometry dead-reckons from it in the local level frame, north-east-down with its origin where the vehicle stood when the attitude estimator aligned, and publishes `vehicle_local_odometry`: position, attitude, body velocity, yaw rate and their variances. The heading is always the attitude estimator's, never the wheels' speed difference, so a skid does not turn the track. Both go to the Link at 50 Hz; the live view shows them on its `body velocity` and `odometry` lines, and `--check` prints them.

## Remote control

A gamepad drives the vehicle in `MANUAL` (spec 0010). It publishes `manual_control_input` (its sticks, triggers and buttons, and whether it is connected), which the Link carries at 50 Hz and the live view shows on its `gamepad` line.

| Button or stick | Does |
|---|---|
| `A` | Takes over in `MANUAL` from any mode, armed or not |
| `START` held 1 s | Arms, with both sticks centred |
| `BACK` held 1 s | Disarms |
| `B` | Engages the emergency stop |
| `Y` held 1 s | Releases the emergency stop |
| Left stick forward and back | Forward speed, up to `MAN_VX_MAX` (0.5 m/s) |
| Right stick left and right | Yaw rate, up to `MAN_YAW_MAX` (1 rad/s), right turning right |

Stick travel inside `MAN_DEADZONE` (8 %) counts as centred. Arming in `MANUAL` is refused while the gamepad is not connected (`RC_NOT_CONNECTED`) or a stick is off centre (`STICKS_NOT_CENTRED`). Taking over while armed does not disarm: the gamepad's sticks drive from the next cycle, and setpoints from a Link are then dropped and counted on `vehicle_status`. A Link cannot request `MANUAL`, and `TEST` can be entered only while disarmed. If the gamepad stops reporting for `RC_TIMEOUT` (0.5 s) or is unplugged while armed in `MANUAL`, the vehicle stops and disarms with `RC_LOST`; it does not re-arm when the gamepad returns. The idle disarm (`COM_DISARM_IDLE`) applies in `MANUAL` too.

In SITL the gamepad is simulated and set with `sim-gamepad`; for example, take over and arm, then drive forward at half speed for 2 s:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 sim-gamepad --buttons A --duration 0.3
```

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 sim-gamepad --buttons START --duration 1.2
```

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 sim-gamepad --left 0,0.54 --duration 2
```

`sim-gamepad --off` unplugs it. After `--duration` the gamepad stays connected, centred and released.

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
