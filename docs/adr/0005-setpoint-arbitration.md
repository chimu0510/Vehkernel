# Setpoint arbitration: the mode decides who is in control, and RC can always take over

Several sources may send setpoints at the same time: the companion computer, the remote control, a ground station, and in test mode the test runner on the development host (ADR-0018). Instead of a fixed source priority, the current mode decides which source is in control, as with PX4's offboard mode. A mode decides two things: which source is in control, and which setpoint levels that source may use (ADR-0003). In companion mode, for example, only setpoints from the Bridge are accepted, at any of the three levels; the RC manual mode accepts only body velocity. Direct actuator is a setpoint level, not a mode.

At boot the vehicle is in `STANDBY`, where no source is in control and nothing can be armed. The mode can change only while disarmed. The source in control is the Link (or, later, the remote control or the Bridge) that requested the mode; setpoints from any other source are rejected and counted.

Two rules apply regardless of mode:

- **The remote control can always take over or trigger an emergency stop.** This is the last line of defence for standalone operation and safety.
- **If the source in control times out, the vehicle falls back to its configured failsafe action. The default depends on the vehicle type and can be overridden by the frame preset (ADR-0013).** A ground vehicle stops, for example; an underwater vehicle holds position or surfaces.

**Emergency stop and actuator power sense**

The emergency stop is a switch that cuts the actuators' power circuit in hardware; the MCU is never in that path. A board may also let the MCU read whether that circuit is closed: on the C50C, `PE4` reads the drive power circuit that the switch on connector CN6 cuts. Other boards, such as the MicoAir743, have no such input. The board definition declares the input and its powered level where there is one (ADR-0011).

- **Actuator power is an arming check.** Arming needs the circuit to have read powered without a break for a settle time (0.5 s), so contact bounce or a loose connector cannot make the vehicle armable.
- **Losing power disarms.** A single unpowered sample while armed disarms in that cycle, with actuator power lost as the reason. The motors have already stopped in hardware; disarming keeps the speed loops from winding up against stopped motors, so the wheels cannot jump when power returns. Restoring power never re-arms: arming needs a new request from the source in control, with every setpoint cleared.
- **It can be skipped.** See arming checks below. On a board without the input the check is not available and passes.

The firmware cannot assume a hardware cut exists or is used, so it also guards against itself stalling: the platform layer turns every actuator output off when actuator control has not refreshed it within a few control periods, so a stalled `wq:ctrl` cannot leave a motor running on its last duty.

**Arming checks**

Arming checks are of two kinds:

- **Checks of hardware state**, such as actuator power now and battery or sensor health later. Each has a bit in the bitmask parameter `ARM_CHECK`, all set by default, as in ArduPilot's `ARMING_CHECK`. Clearing a bit skips that check: it neither refuses arming nor disarms, but its state is still published. Skipping is for bench testing and for boards whose input is unwired or unreliable. `ARM_CHECK` is locked while armed, the vehicle status reports which checks are skipped, and tools warn on arming while any is.
- **Checks that guard the arming logic itself**: the mode, the source in control, setpoints cleared, no bring-up case running, actuators started. They cannot be skipped.

**Before the remote control exists**

Until the remote control is implemented, actuators may move only in test mode, under four conditions that stand in for the RC takeover rule:

- Only direct-actuator setpoints are accepted, as test mode always requires (ADR-0018).
- Actuator power is checked where the board senses it, unless skipped. On a board without the input, the operator must be able to cut actuator power by hand; the documentation says so, since the firmware cannot check it.
- Actuator output is capped by a parameter, low by default.
- If the source in control times out, the failsafe action is stop and disarm; if no setpoint arrives within a set time after arming, or all targets stay zero for that time, the vehicle disarms as well.

When the remote control exists these conditions stay; it adds a takeover path, it does not replace them.

## Considered Options

- **Fixed priority (RC > ground station > companion computer)**: rejected. While the companion computer is driving autonomously, any higher-priority source could interrupt it at will, and the priority order cannot express "who is driving right now".
- **Staying armed when actuator power or a kill switch stops the motors** (PX4's kill switch and ArduPilot's `MOTOR_ESTOP`): rejected. Both force the outputs to their disarmed values but leave the vehicle armed, so that an aircraft can restart its motors in the air the moment the switch is released; PX4 disarms only after the kill switch has stayed on for 5 s. A ground or surface vehicle gains nothing from an instant restart, and a stopped vehicle that is still armed resumes motion when power returns, which an operator walking up to it does not expect.
- **A safety button in addition to arming** (PX4's `COM_SAFETY_MODE`, ArduPilot's `BRD_SAFETYENABLE`): not adopted. A push button held for about a second turns "safety" off before arming is allowed, and options decide whether disarming turns it back on. It protects propellers while a drone is handled on the bench. Here arming already needs a deliberate request from the source in control, and a vehicle model that needs a button later can add it as one more arming check.
- **Actuator power as a check that cannot be skipped**: rejected. Boards differ, and on the bench the sense line may be unwired or the drivers fed from a supply that bypasses it; a check nobody can skip gets patched out of the firmware instead.
- **PX4's circuit breakers** (`CBRK_*` parameters that disable a check only when set to a magic number): not adopted. The magic number guards against setting one by accident; here `ARM_CHECK` is locked while armed, and skipped checks are reported and warned about at every arming.
