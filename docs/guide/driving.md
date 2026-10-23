# Driving the wheels

Actuators move only in test mode, armed, and only on direct-actuator setpoints: one wheel speed in rad/s per motor output (ADR-0005, ADR-0018). The commands below use the [Link monitor](link-monitor.md) against SITL; on a board, use `--serial /dev/ttyACM0` instead of `--udp`, with the firmware build's `--msg-dir` and `--param-meta` ([Connecting](link-monitor.md#connecting)).

## Safety conditions

Until a remote control exists, these conditions apply:

- The vehicle boots in `STANDBY`; `mode test` makes the Link that asked the source in control. The mode changes only while disarmed.
- `arm` runs the arming checks in turn and prints the first that fails: the board verified at boot (the IMU answered and, on boards that have them, the revision-ID pins read their value), the emergency stop released, `TEST`, the source in control, no setpoint still active, no bring-up case running, actuator power present on boards that sense it, actuator control started, and the local menu closed. A reboot already accepted refuses arming before any of these.
- `estop on` from any connected Link forces every output off at once. Released within 5 s with `estop off`, the wheels resume from the next setpoint; engaged for 5 s, the vehicle disarms.
- Setpoints that stop for longer than `COM_SP_TIMEOUT` (0.5 s) stop and disarm the vehicle; an armed vehicle whose targets stay zero or unset for `COM_DISARM_IDLE` (10 s) disarms.
- Outputs are capped at `MOT_OUT_MAX` (30 % duty by default) and change by at most `MOT_SLEWRATE` of full duty per second, though a stop is never slowed. A watchdog turns them off when actuator control stalls for three control periods.

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

## Driving

`drive` requests `TEST` and arms if needed, sends one wheel speed in rad/s per output at 20 Hz for the given time (`nan` leaves an output uncontrolled; put `--` before a negative first speed), then stops and disarms:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 drive 5,5,5,5 --duration 3
```

The live view shows `vehicle_status`, `wheel_encoders` and `actuator_outputs`.

## In SITL

SITL simulates four motors and encoders; `--actuator-power-off` starts it with the simulated power cut, `sim-power on|off` cuts or restores it while it runs, and `--reversed-motor <output>` wires a motor the wrong way round. CI drives the simulated wheels through every one of these paths (`sitl_arming`, `sitl_actuators` and `sitl_bringup`).
