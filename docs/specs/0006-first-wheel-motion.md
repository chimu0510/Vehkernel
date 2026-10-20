# Spec 0006: first wheel motion

**Status**: in progress. Issue: [#43](https://github.com/chimu0510/Vehkernel/issues/43); tickets #44 to #47.

This file and the issue carry the same spec; update both when the spec changes.

## Problem Statement

The fifth milestone left Vehkernel with attitude estimation and a parameter system on the C50C V1.1 and in SITL, with parameters listed, read and set over the Link and stored in internal Flash (#36). The firmware still cannot move anything. The C50C has four motor drivers on TIM8 and four quadrature encoders on TIM2 to TIM5, but no code touches them: there is no arming, no mode, no setpoint can reach the MCU, no actuator control exists, and nothing publishes `arming_state`, which parameter storage and `locked_when_armed` already follow.

ADR-0005 decides that the mode chooses who is in control and that the remote control can always take over, ADR-0013 ends the control chain in actuator control (a wheel speed PI with encoder feedback), and ADR-0018 asks for a test mode in which the test runner drives the actuators to check motor-to-encoder mapping and direction. None of it exists, and every later milestone stands on it: the vehicle model and body velocity control need wheel speed control underneath, the remote control needs arming and modes to take over from, and the Bridge's arming and mode services need the same requests over a Link.

Moving wheels is also the first time the firmware can hurt someone or something. A motor or encoder wired the other way round turns a speed loop into positive feedback, which runs the wheel to full output at once. The milestone must therefore bring the safety mechanisms in with the first motion, not after it.

## Solution

Deliver the sixth milestone: **the first wheel motion, from a direct-actuator setpoint sent over the Link to wheels turning at a closed-loop speed**. Afterwards:

- The vehicle boots in `STANDBY`. A Link can request `TEST` mode and then arming; both are control frames of the custom protocol at version 3, answered with the new state or a reason for refusal.
- In test mode the development host sends direct-actuator setpoints, one wheel speed in rad/s per actuator, as an inbound topic on the Link. Setpoint arbitration accepts them only from the Link that requested the mode.
- Actuator control runs on `wq:ctrl` at the base rate: it reads the encoders, closes a speed PI per wheel, and drives the PWM outputs and direction lines. It publishes the wheel angles and speeds and the outputs it applied.
- The emergency stop (the C50C's enable switch, shared with connector CN6) must be closed to arm and disarms at once when opened. A setpoint timeout stops and disarms; an armed vehicle with no setpoint disarms after a while; output is capped by a parameter that is low by default (ADR-0005).
- A bring-up case in test mode pulses each motor open loop and reports which encoder moved and in which direction, so a reversed motor or encoder is found before the speed loop runs (ADR-0018).
- The board definition declares the actuator outputs and their encoders; parameters prefixed `MOT` give each output's reversal and encoder resolution and the shared speed loop tuning.
- SITL runs the same code against a first-order motor and encoder model and a simulated emergency stop; CI drives the wheels, cuts the setpoints and the emergency stop, and finds a reversed motor with the bring-up case.
- On the C50C V1.1, with the wheels lifted, each wheel holds a commanded speed and stops when the emergency stop opens or the USB cable is pulled.

## User Stories

1. As a firmware developer, I want the vehicle to boot in `STANDBY`, where no source is in control and arming is refused, so that nothing can move until a source has deliberately taken control.
2. As a Link user, I want to request `TEST` mode over the Link, so that the development host becomes the source in control for bring-up and testing (ADR-0018).
3. As a firmware developer, I want the mode to change only while disarmed, so that the source in control never changes under moving wheels.
4. As a Link user, I want to arm and disarm over the Link and get the new state or a reason for refusal, so that a script knows exactly why the vehicle will not arm.
5. As a firmware developer, I want arming refused unless the mode is `TEST`, the emergency stop is closed, the requesting Link has completed its handshake, every setpoint is zero or unset, no bring-up case is running and the actuators and encoders started, each with its own reason, so that arming is only possible from a known safe state.
6. As a vehicle user, I want opening the emergency stop to disarm the vehicle within one control cycle, so that a hardware switch stops the wheels whatever the software above it is doing (ADR-0005).
7. As a vehicle user, I want the vehicle to stop and disarm when the source in control stops sending setpoints for longer than `COM_SP_TIMEOUT`, so that a crashed script or a pulled cable stops the wheels (ADR-0005 failsafe action).
8. As a vehicle user, I want the vehicle to disarm when no setpoint arrives within `COM_DISARM_IDLE` after arming, or when every target stays zero or unset for that long, so that a vehicle is never left armed and forgotten.
9. As a firmware developer, I want the arming state published on `arming_state` and the mode, arming state and source in control on `vehicle_status`, so that parameter storage keeps following `arming_state` and the host sees the whole state.
10. As a Link user, I want to send direct-actuator setpoints as an inbound topic, one wheel speed in rad/s per actuator, so that setpoints share the schema, the generated codecs and the path the Bridge's setpoints will take (ADR-0002, ADR-0006).
11. As a Link user, I want an actuator whose setpoint is NaN to be left uncontrolled with no output, so that I can drive one wheel and leave the others alone.
12. As a firmware developer, I want setpoints from any Link other than the source in control rejected and counted, so that a second tool cannot drive the vehicle (ADR-0005).
13. As a firmware developer, I want inbound topics the topic map does not declare dropped and counted, so that a peer cannot publish arbitrary topics on the bus.
14. As a firmware developer, I want actuator control to close a speed PI per wheel on the encoders, with the target clamped to `MOT_MAX_SPD`, the output clamped to `MOT_OUT_MAX` and the integrator stopped when the output saturates, so that a wheel holds its speed under load and recovers cleanly from saturation (ADR-0013).
15. As a firmware developer, I want a wheel's integrator cleared whenever its setpoint becomes NaN or the vehicle disarms, so that a wheel never jumps when control resumes.
16. As a firmware developer, I want disarming to set every PWM output to zero and both direction lines low, so that disarmed motors coast and draw no current.
17. As a firmware developer, I want each wheel's angle and speed published on `wheel_encoders` and each applied output on `actuator_outputs` every control cycle, so that tuning and the next milestone's body velocity measurement have their inputs.
18. As a firmware developer, I want encoder counts handled as 16-bit differences on every timer, so that 16-bit and 32-bit timers wrap correctly with one code path.
19. As a board author, I want to declare each actuator output with its PWM channel, direction lines and paired encoder in the board definition, checked by the board generator, so that actuator wiring is a hardware fact compared line by line with the schematic (ADR-0011).
20. As a board author, I want to declare the emergency stop input and its active level in the board definition, so that the firmware reads it the same way on every board (ADR-0005).
21. As a vehicle builder, I want `MOTn_REV`, `MOTn_ENC_REV` and `MOTn_CPR` per output, with the encoder resolution's default from the board definition, so that a different motor or wiring needs no rebuild.
22. As a firmware developer, I want the per-output parameters locked while armed and the speed loop gains changeable while armed, so that wiring cannot change under moving wheels while tuning still can (ADR-0012).
23. As a C50C user, I want to run a bring-up case that pulses each motor and reports whether its own encoder moved in the right direction, which encoder moved and the sign of its count, so that I know which `MOTn_REV` or `MOTn_ENC_REV` to set before running the speed loop (ADR-0018).
24. As a firmware developer, I want the bring-up case to run only in test mode while armed, to ignore setpoints and the setpoint timeout while it runs, and to abort and disarm on the emergency stop or a disarm request, so that it is guarded like any other motion.
25. As a Link user, I want the Link monitor to change the mode, arm, disarm, drive the wheels for a given time and run the bring-up case, printing reasons and exiting non-zero on refusal, so that scripts and tests can drive the vehicle.
26. As a Link user, I want the Link monitor's live view to show `vehicle_status`, `wheel_encoders` and `actuator_outputs`, so that I see the vehicle's state while it moves.
27. As a SITL user, I want SITL to simulate the motors and encoders with a first-order model, with a motor that can be set to run reversed, so that actuator control and the bring-up case are tested without hardware.
28. As a SITL user, I want to start SITL with the emergency stop open and to open or close it while running, so that CI tests arming refusal and an emergency stop under motion.
29. As a maintainer, I want CI to arm, drive, time out, cut the emergency stop and find a reversed motor in SITL (L3), so that every safety path is covered on every push.
30. As a C50C user, I want a documented hardware check with the wheels lifted, so that mapping, direction, speed control, the emergency stop, the setpoint timeout and the PWM frequency are confirmed on the board (ADR-0018 L4).
31. As a new contributor, I want the README to explain how to arm and drive the wheels in SITL and on the board, and the safety conditions that apply, so that I can make a wheel turn in minutes and safely.

## Implementation Decisions

- **Modes (module)**: `STANDBY` and `TEST`. The vehicle boots in `STANDBY`. A mode request is accepted only while disarmed. In `TEST` the source in control is the Link instance that sent the mode request, and only direct-actuator setpoints are accepted (ADR-0005, ADR-0018). Requesting `STANDBY` releases control.
- **Arming (module)**: an arm request is refused, with one reason each, unless: the mode is `TEST`; the emergency stop is closed; the request comes from the source in control after its handshake; every current setpoint is NaN or zero; no bring-up case is running; actuator outputs and encoders started. The vehicle disarms on a disarm request, when the emergency stop opens (checked every `wq:ctrl` cycle), on the failsafe action, and on idle. Disarming clears every setpoint.
- **Setpoint timers**: after arming the vehicle waits for its first input: if no setpoint arrives and no bring-up case starts within `COM_DISARM_IDLE`, it disarms. After the first setpoint, a gap longer than `COM_SP_TIMEOUT` between setpoints from the source in control triggers the failsafe action, stop and disarm. While armed, every target staying NaN or zero for `COM_DISARM_IDLE` also disarms. While a bring-up case runs neither timer counts.
- **Emergency stop**: read on every `wq:ctrl` cycle before actuator control runs; open while armed disarms in that cycle and the outputs are zeroed in the same cycle. Its state is in `vehicle_status`.
- **Topics (schema)**:
  - `actuator_setpoint`: `timestamp`, `float32[4] wheel_speed` in rad/s, NaN for an uncontrolled actuator. Inbound only.
  - `vehicle_status`: `timestamp`, mode, armed, source in control (none or a Link instance), emergency stop closed, the last arming refusal reason and the last disarm reason. Sent over the Link at 5 Hz and on change.
  - `arming_state`: unchanged; now published by arming on every change.
  - `wheel_encoders`: `timestamp`, `float32[4] angle` in rad (accumulated since boot), `float32[4] speed` in rad/s, and a valid mask. Published every control cycle.
  - `actuator_outputs`: `timestamp`, `float32[4] output`, the applied duty in [-1, 1], the sign giving the direction, and the target speeds after clamping. Published every control cycle.
  - `link_status` gains a count of inbound topic data dropped because the map does not declare it and a count rejected by setpoint arbitration.
  - Arrays have four elements, the most actuators any board has today; elements beyond a board's actuator count are NaN or zero.
- **Actuator control (module)**: on `wq:ctrl`, triggered with the IMU at the base rate (200 Hz on the C50C). Per output: read the encoder as a 16-bit count, take `int16_t(now - previous)`, apply `MOTn_ENC_REV`, convert with `MOTn_CPR` to an angle increment and a speed over the cycle's measured dt. The target is clamped to `±MOT_MAX_SPD`; the output is `MOT_SPD_FF·target + MOT_SPD_P·error + integral`, clamped to `±MOT_OUT_MAX`; the integrator does not grow towards the saturated side. `MOTn_REV` flips the output sign before it reaches the driver. A NaN target or disarmed: output zero, both direction lines low, integrator cleared. Armed with a zero target: the PI holds zero speed; there is no brake mode.
- **Board definition**: a new `actuators` array, one entry per output number, giving the PWM pin role, the two direction pin roles and the encoder's two pin roles; the generator checks that the PWM role is a timer channel, the direction lines are GPIO outputs and the encoder's two roles are CH1 and CH2 of one timer, and emits typed actuator constants and `BOARD_HAS_ACTUATORS` (ADR-0011). A new `estop` entry names the input pin role and its active level. The board defaults gain `MOT_CPR`, applied to every `MOTn_CPR`. The C50C's four outputs are 1 = A, 2 = B, 3 = C, 4 = D as the pins already declare them.
- **Parameters**:
  - Group `MOT`, per output n = 1..4: `MOTn_REV` and `MOTn_ENC_REV` (int32 booleans, default 0), `MOTn_CPR` (float, counts per wheel turn after the gearbox and quadrature decoding, default from the board definition). All three `locked_when_armed`.
  - Group `MOT`, shared: `MOT_SPD_P`, `MOT_SPD_I`, `MOT_SPD_FF` (float, changeable while armed), `MOT_MAX_SPD` (float, rad/s), `MOT_OUT_MAX` (float, 0 to 1, default 0.3).
  - Group `COM`: `COM_SP_TIMEOUT` (float, s, default 0.5) and `COM_DISARM_IDLE` (float, s, default 10).
  - The generator already accepts a prefix followed by a digit, so `MOT1_REV` needs no change to its name rule.
- **Interface layer contracts**: a PWM output (set a duty in [0, 1] on a channel at a frequency fixed at start), a quadrature encoder (read a 16-bit count), and the existing GPIO for direction lines and the emergency stop. `core/` and modules reach them only through these contracts (ADR-0009).
- **Platform layer, STM32F4**: TIM8 PWM at 10 kHz on its four channels, with the direction lines as GPIO outputs; TIM2, TIM3, TIM4 and TIM5 in encoder mode on CH1 and CH2; the emergency stop as a GPIO input. Outputs start with zero duty and both direction lines low.
- **Platform layer, SITL**: a first-order motor model per output: steady speed proportional to duty, 50 rad/s at full duty, time constant 0.1 s, an encoder with a fixed counts-per-turn feeding the same 16-bit counter. A motor can be set to run reversed or to drive another output's encoder, for the bring-up tests. The emergency stop starts closed; `--estop-open` starts it open, and a SITL-only control frame opens or closes it while running. The model lives in `sim/`, like the ICM20948 model and the NOR Flash model, so unit tests use it too.
- **Bring-up case, motor-to-encoder mapping and direction**: started by `bringup_run`, accepted only in `TEST` while armed. For each output in turn it applies a fixed open-loop duty (20 %, within `MOT_OUT_MAX`) for 0.3 s with every other output off, and records the count change of every encoder. Per output it reports one of: pass, no encoder moved, own encoder moved the wrong way, another encoder moved; with the encoder that moved most and the sign of its count. Setpoints are ignored and the timers stop while it runs; the emergency stop and a disarm request abort it and disarm. The result arrives as `bringup_result`; the vehicle stays armed afterwards.
- **Custom protocol, version 3**: the MCU and the host tools move to version 3; the MCU offers only version 3.
  - `mode_request` (mode) → `mode_reply` (mode now in effect) or `mode_reject` (reason: armed, unknown mode, not handshaken).
  - `arming_request` (arm or disarm) → `arming_reply` (armed state now in effect) or `arming_reject` (the reasons above).
  - `bringup_run` (case id) → `bringup_result` (case id, result per output, encoder that moved, sign) or `bringup_reject` (reason).
  - `TopicData` is now accepted from the host for topics the topic map declares inbound.
  - The protocol module stays independent of the transport and of FreeRTOS.
- **Topic map**: each entry gains a direction. The default map declares `actuator_setpoint` inbound and adds `vehicle_status` at 5 Hz, `wheel_encoders` at 50 Hz and `actuator_outputs` at 50 Hz outbound. Inbound topics are not rate limited; the timers use the time of the last accepted setpoint.
- **Setpoint arbitration (module)**: an inbound `actuator_setpoint` is accepted only from the source in control, only in a mode that allows direct-actuator setpoints, only while armed and no bring-up case runs; otherwise it is counted and dropped. An accepted setpoint is published on the bus for actuator control.
- **Link monitor**: `mode <standby|test>`, `arm`, `disarm`, `drive <w1,w2,w3,w4> --duration <s>` (requests `TEST` and arming if needed, sends `actuator_setpoint` at 20 Hz, sends all NaN at the end and disarms), and `bringup motors` (prints the result per output and the parameter to change). Refusals print the reason and exit non-zero. The live view and `--check` show `vehicle_status`, `wheel_encoders` and `actuator_outputs`.
- **Kconfig**: `ACTUATOR_CONTROL` depends on `BOARD_HAS_ACTUATORS`, SITL provides it with its model; `BRINGUP_MOTORS` depends on it. Without them the firmware has modes and arming but refuses to arm for lack of actuators.
- **Bring-up image**: unchanged. The IMU cases stay in the SWD bring-up image, which still never configures actuator pins (ADR-0018).

## Testing Decisions

- **Good tests check external behaviour only**: what a request returns, what `vehicle_status`, `wheel_encoders` and `actuator_outputs` show, what the simulated motors do and what the Link monitor prints. The state machine's internals are not asserted.
- **Seam A: SITL (L3), the main seam.** `test/sitl/check_actuators.py` runs SITL and the Link monitor library and checks, in CI:
  1. With the emergency stop open, arming is refused with its reason.
  2. Arming in `STANDBY` is refused; a mode request while armed is refused.
  3. In `TEST`, armed, 10 rad/s on all four outputs: within 1 s every wheel's speed in `wheel_encoders` is within ±5 % of the target.
  4. Setpoints stop: within `COM_SP_TIMEOUT` plus one cycle the vehicle disarms and `actuator_outputs` is all zero.
  5. Opening the emergency stop while the wheels turn disarms in the next cycle and zeroes the outputs.
  6. With output 2 set to run reversed in the model, `bringup motors` reports output 2 reversed and the others passing; after `MOT2_REV=1` it reports all passing.
  7. Armed with no setpoint, the vehicle disarms after `COM_DISARM_IDLE`.
  The existing SITL, Link, attitude and parameter checks keep passing.
- **Seam B: actuator control (L1, GoogleTest).** Against the motor model: the PI reaches and holds a target; the target is clamped to `MOT_MAX_SPD` and the output to `MOT_OUT_MAX`; the integrator does not wind up in saturation and recovers without overshoot beyond a bound; NaN zeroes the output and clears the integrator; `MOTn_REV` and `MOTn_ENC_REV` flip the right signs; 16-bit wrap in both directions gives the right speed on a counter near its limits.
- **Seam C: modes, arming and arbitration (L1).** Every arming refusal reason; mode changes refused while armed; each disarm cause, including the emergency stop in the cycle it opens; the two timers at their boundaries (just under and just over `COM_SP_TIMEOUT` and `COM_DISARM_IDLE`), and the all-zero idle disarm; setpoints from another Link, in `STANDBY` or while disarmed are dropped and counted; `arming_state` published on every change.
- **Seam D: bring-up case (L1).** Against the motor model with each wiring fault: a reversed motor, a reversed encoder, a dead motor, swapped encoders; each gives the right result, encoder and sign; the emergency stop and a disarm request abort it.
- **Seam E: protocol and generators (L0 and L1).** The version 3 frames over a fake transport, inbound `TopicData` accepted only when declared; board generator golden output for `actuators` and `estop`, and a diagnostic each for a PWM role that is not a timer channel, encoder channels on two timers and an unknown pin role.
- **Seam F: hardware (L4, manual).** On the C50C with the wheels lifted, `MOT_OUT_MAX` at its default: `bringup motors`, setting `MOTn_REV`/`MOTn_ENC_REV` until every output passes; each output alone at +5 and −5 rad/s holds within ±10 % in steady state, with no `wq:ctrl` overruns in the timing monitor; opening CN6 while turning stops at once; pulling the USB cable while turning stops within 0.5 s; an oscilloscope shows 10 kHz on the PWM pins. The emergency stop's active level and `MOT_CPR` for the stock motors are measured first (a wheel turned by hand one turn) and committed to the board definition. Results are recorded on the ticket.
- **Prior art**: `test/sitl/check_params.py` and `check_attitude.py` for SITL with the Link monitor library; the parameter and Link unit tests over a fake transport; the ICM20948 register model and NOR Flash model in `sim/`, which the motor model follows; the board generator golden tests and diagnostic cases.

## Out of Scope

- The vehicle model (`WHEELED`, `ACKERMANN`, `THRUSTER`), `VEHICLE_TYPE`, `FRAME_CLASS` and frame presets (ADR-0013).
- Body velocity and attitude setpoints, velocity, rate and attitude control, and body velocity measurements and odometry.
- The remote control (SBUS, RC PWM inputs, USB gamepad), RC takeover and RC modes, and companion mode.
- The Bridge and its arming and mode services (ADR-0003).
- The built-in vehicle simulator's body dynamics; SITL simulates the motors and encoders only.
- Servo outputs, including the C50C's steering servos, and alt config 1.
- Battery monitoring and low-battery failsafe actions.
- Pre-arm checks beyond those listed, such as the estimator's health or calibration.
- Moving the IMU bring-up cases into test mode, and further bring-up cases (revision-ID pins, PWM frequency as a case).
- Logging of the new topics (ADR-0019).

## Further Notes

- **Why direct actuator before the vehicle model**: ADR-0013 lets direct-actuator setpoints skip the first three layers of the control chain, so actuator control can be built and proven alone. The bring-up case must exist before any closed loop runs on hardware, and the vehicle model then only adds `wheel speeds = J·twist` above a layer that already works.
- **Why wheel speed, not duty**: the vehicle model hands actuator control wheel speeds in rad/s, so direct-actuator setpoints use the same unit and the input of actuator control is the same whichever path feeds it. It also matches a velocity joint interface of an optional ros2_control adapter (ADR-0003). Open-loop duty exists only inside the bring-up case.
- **Why test mode may move wheels before the remote control exists**: ADR-0005's takeover rule cannot hold without a remote control, so ADR-0005 now allows motion only in test mode under the emergency stop, a low output cap, a setpoint timeout and idle disarm.
- **Facts to settle at bring-up**: the emergency stop's active level on `PE4` and the encoder resolution of the stock motors are not in the schematic notes or the vendor firmware sources at hand; the L4 ticket measures them before any motion.
- **Protocol version 3 only**: as with version 2, the custom protocol has no peers outside this repository, so the MCU does not keep a version 2 path.
