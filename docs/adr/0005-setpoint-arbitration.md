# Setpoint arbitration: the mode decides who is in control, and RC can always take over

Several sources may send setpoints at the same time: the companion computer, the remote control, a ground station, and in test mode the test runner on the development host (ADR-0018). Instead of a fixed source priority, the current mode decides which source is in control, as with PX4's offboard mode. A mode decides two things: which source is in control, and which setpoint levels that source may use (ADR-0003). In companion mode, for example, only setpoints from the Bridge are accepted, at any of the three levels; the RC manual mode accepts only body velocity. Direct actuator is a setpoint level, not a mode.

At boot the vehicle is in `STANDBY`, where no source is in control and nothing can be armed. The mode can change only while disarmed. The source in control is the Link (or, later, the remote control or the Bridge) that requested the mode; setpoints from any other source are rejected and counted.

Two rules apply regardless of mode:

- **The remote control can always take over or trigger an emergency stop.** This is the last line of defence for standalone operation and safety.
- **If the source in control times out, the vehicle falls back to its configured failsafe action. The default depends on the vehicle type and can be overridden by the frame preset (ADR-0013).** A ground vehicle stops, for example; an underwater vehicle holds position or surfaces.

**Emergency stop**

A board may have a hardware emergency stop input, declared in the board definition with its active level (on the C50C, the enable switch on `PE4`, shared with connector CN6). While it is open the vehicle cannot be armed, and opening it while armed disarms at once, checked every control cycle. It is independent of the remote control and of every Link.

**Before the remote control exists**

Until the remote control is implemented, actuators may move only in test mode, under four conditions that stand in for the RC takeover rule:

- Only direct-actuator setpoints are accepted, as test mode always requires (ADR-0018).
- The board must have an emergency stop input, and it gates arming as above.
- Actuator output is capped by a parameter, low by default.
- If the source in control times out, the failsafe action is stop and disarm; if no setpoint arrives within a set time after arming, or all targets stay zero for that time, the vehicle disarms as well.

When the remote control exists these conditions stay; it adds a takeover path, it does not replace them.

## Considered Options

- **Fixed priority (RC > ground station > companion computer)**: rejected. While the companion computer is driving autonomously, any higher-priority source could interrupt it at will, and the priority order cannot express "who is driving right now".
