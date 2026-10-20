# Setpoint arbitration: the mode decides who is in control, and RC can always take over

Several sources may send setpoints at the same time: the companion computer, the remote control, a ground station, and in test mode the test runner on the development host (ADR-0018). Instead of a fixed source priority, the current mode decides which source is in control, as with PX4's offboard mode. A mode decides two things: which source is in control, and which setpoint levels that source may use (ADR-0003). In companion mode, for example, only setpoints from the Bridge are accepted, at any of the three levels; the RC manual mode accepts only body velocity. Direct actuator is a setpoint level, not a mode.

At boot the vehicle is in `STANDBY`, where no source is in control and nothing can be armed. The mode can change only while disarmed. The source in control is the Link (or, later, the remote control or the Bridge) that requested the mode; setpoints from any other source are rejected and counted.

Two rules apply regardless of mode:

- **The remote control can always take over or trigger an emergency stop.** This is the last line of defence for standalone operation and safety.
- **If the source in control times out, the vehicle falls back to its configured failsafe action. The default depends on the vehicle type and can be overridden by the frame preset (ADR-0013).** A ground vehicle stops, for example; an underwater vehicle holds position or surfaces.

**Emergency stop**

A board may have a hardware emergency stop input, declared in the board definition with its active level (on the C50C, the enable switch on `PE4`, shared with connector CN6). It is independent of the remote control and of every Link, and it works as follows:

- **Opening stops at once; closing must settle.** The input is sampled every control cycle. A single open sample counts as open. Closed counts only after it has read closed without a break for a settle time (0.5 s), so contact bounce or a loose connector cannot make the vehicle armable.
- **Opening disarms.** Opening it while armed disarms in the same cycle, with the emergency stop as the disarm reason. Closing it again never re-arms: arming needs a new request from the source in control, with every setpoint cleared.
- **Enforced twice.** Arming disarms the vehicle, and actuator control separately reads the same sample and forces its outputs off while the input is open, whatever the arming state says. A fault in the arming logic therefore cannot keep a motor running against an open emergency stop.
- **No bypass.** No parameter disables it or masks it from the arming checks. A board without an emergency stop input cannot arm in test mode (see below).

The emergency stop may be a firmware input only, without cutting motor power in hardware, as on the C50C's enable switch. The firmware therefore also guards against itself stalling: the platform layer turns every actuator output off when actuator control has not refreshed it within a few control periods, so a stalled `wq:ctrl` cannot leave a motor running on its last duty while nothing reads the emergency stop.

**Before the remote control exists**

Until the remote control is implemented, actuators may move only in test mode, under four conditions that stand in for the RC takeover rule:

- Only direct-actuator setpoints are accepted, as test mode always requires (ADR-0018).
- The board must have an emergency stop input, and it gates arming as above.
- Actuator output is capped by a parameter, low by default.
- If the source in control times out, the failsafe action is stop and disarm; if no setpoint arrives within a set time after arming, or all targets stay zero for that time, the vehicle disarms as well.

When the remote control exists these conditions stay; it adds a takeover path, it does not replace them.

## Considered Options

- **Fixed priority (RC > ground station > companion computer)**: rejected. While the companion computer is driving autonomously, any higher-priority source could interrupt it at will, and the priority order cannot express "who is driving right now".
- **Emergency stop that keeps the vehicle armed** (PX4's kill switch and ArduPilot's `MOTOR_ESTOP`): rejected. Both force the outputs to their disarmed values but leave the vehicle armed, so that an aircraft can restart its motors in the air the moment the switch is released; PX4 disarms only after the kill switch has stayed on for 5 s. A ground or surface vehicle gains nothing from an instant restart, and a stopped vehicle that is still armed resumes motion when the switch closes, which an operator walking up to it does not expect. Vehkernel disarms at once and takes from both only the enforcement at the output stage.
- **A safety button in addition to arming** (PX4's `COM_SAFETY_MODE`, ArduPilot's `BRD_SAFETYENABLE`): not adopted. A push button held for about a second turns "safety" off before arming is allowed, and options decide whether disarming turns it back on. It protects propellers while a drone is handled on the bench. Here the emergency stop already gates arming, and a vehicle model that needs it later can add it as one more arming check.
- **A parameter or arming-check mask that bypasses the emergency stop** (PX4's `CBRK_IO_SAFETY`, ArduPilot's `ARMING_CHECK` bit for the hardware safety switch): rejected while test mode is the only way to move actuators; the emergency stop is then the only safeguard independent of the software above it.
