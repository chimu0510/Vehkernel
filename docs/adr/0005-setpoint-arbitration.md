# Setpoint arbitration: the mode decides who is in control, and RC can always take over

Several sources may send setpoints at the same time: the companion computer, the remote control, a ground station, and in test mode the test runner on the development host (ADR-0018). Instead of a fixed source priority, the current mode decides which source is in control, as with PX4's offboard mode. A mode decides two things: which source is in control, and which setpoint levels that source may use (ADR-0003). In companion mode, for example, only setpoints from the Bridge are accepted, at any of the three levels; the RC manual mode accepts only body velocity. Direct actuator is a setpoint level, not a mode.

At boot the vehicle is in `STANDBY`, where no source is in control and nothing can be armed. The mode can change only while disarmed. The source in control is the Link (or, later, the remote control or the Bridge) that requested the mode; setpoints from any other source are rejected and counted.

Two rules apply regardless of mode:

- **The remote control can always take over or trigger the emergency stop.** This is the last line of defence for standalone operation and safety.
- **If the source in control times out, the vehicle falls back to its configured failsafe action. The default depends on the vehicle type and can be overridden by the frame preset (ADR-0013).** A ground vehicle stops, for example; an underwater vehicle holds position or surfaces.

**Emergency stop**

The emergency stop follows ArduPilot's `MOTOR_ESTOP` and PX4's kill switch. It is a firmware function, triggered by the remote control or by a request over any connected Link, whoever is in control:

- **Outputs off at the output stage.** While it is engaged, actuator control forces every output to its disarmed value, whatever the mode, the source in control and the setpoints, and clears the speed loops' integrators (as ArduPilot Rover relaxes its integrators).
- **The vehicle stays armed for a short time.** If the emergency stop is released within 5 s, output resumes from the next setpoint, as with PX4. If it stays engaged for 5 s, the vehicle disarms with the emergency stop as the reason, as PX4 does after its kill switch has been on for 5 s. The 5 s is a constant.
- **Arming is refused while it is engaged**, as ArduPilot's arming checks refuse while the motors are emergency stopped.

Any connected Link may engage or release it, as PX4 accepts kill and unkill from any source. The remote control will be one more source when it exists.

**Output watchdog**

The firmware also guards against itself stalling: the platform layer turns every actuator output off when actuator control has not refreshed it within a few control periods, so a stalled `wq:ctrl` cannot leave a motor running on its last duty, and the emergency stop, which acts in actuator control, still takes effect.

**Arming checks**

Arming checks are of two kinds:

- **Checks of hardware state**, which can be skipped. Each has a bit in the bitmask parameter `COM_ARM_CHECK`, all set by default, as in ArduPilot's `ARMING_CHECK`. Clearing a bit skips that check: it neither refuses arming nor disarms, but its state is still published. Skipping is for bench testing and for boards whose input is unwired or unreliable. `COM_ARM_CHECK` is locked while armed, the vehicle status reports which checks are skipped, and tools warn on arming while any is. A check whose input a board lacks is not available and passes.
- **Checks that cannot be skipped**: the board verified at boot (ADR-0010), the emergency stop released, the mode, the source in control, setpoints cleared, no bring-up case running, actuators started, the local menu closed (ADR-0022), and no accepted reboot pending (ADR-0023).

**Board-specific: actuator power sense**

Some boards let the MCU read whether the actuators' power circuit is closed; on the C50C, `PE4` reads the drive power circuit. This is a fact of those boards, declared in their board definition with its powered level (ADR-0011), and has nothing to do with the emergency stop. Where it exists it is a check of hardware state with its own bit in `COM_ARM_CHECK`:

- Arming needs the circuit to have read powered without a break for a settle time (0.5 s), so contact bounce or a loose connector cannot make the vehicle armable.
- A single unpowered sample while armed disarms in that cycle, with actuator power lost as the reason, so the speed loops do not wind up against motors that have lost power and the wheels cannot jump when it returns. Power returning never re-arms.
- With the bit cleared nothing is checked.

**Before the remote control exists**

Until the remote control is implemented, actuators may move only in test mode, under these conditions, which stand in for the RC takeover rule:

- Only direct-actuator setpoints are accepted, as test mode always requires (ADR-0018).
- The emergency stop can be engaged from any connected Link.
- Actuator output is capped by a parameter, low by default.
- If the source in control times out, the failsafe action is stop and disarm; if no setpoint arrives within a set time after arming, or all targets stay zero for that time, the vehicle disarms as well.

When the remote control exists these conditions stay; it adds a takeover path, it does not replace them.

## Considered Options

- **Fixed priority (RC > ground station > companion computer)**: rejected. While the companion computer is driving autonomously, any higher-priority source could interrupt it at will, and the priority order cannot express "who is driving right now".
- **An emergency stop that disarms at once**: not adopted, to keep ArduPilot's and PX4's semantics, which ground stations and operators already know: engaging stops the outputs at once, and a release within a few seconds resumes without a new arming sequence. The 5 s auto-disarm bounds how long a stopped vehicle stays armed.
- **A safety button in addition to arming** (PX4's `COM_SAFETY_MODE`, ArduPilot's `BRD_SAFETYENABLE`): not adopted. A push button held for about a second turns "safety" off before arming is allowed, and options decide whether disarming turns it back on. It protects propellers while a drone is handled on the bench. Here arming already needs a deliberate request from the source in control, and a vehicle model that needs a button later can add it as one more arming check.
- **Board-specific checks that cannot be skipped**: rejected. Boards differ, and on the bench a sense line may be unwired or the drivers fed from a supply that bypasses it; a check nobody can skip gets patched out of the firmware instead.
- **PX4's circuit breakers** (`CBRK_*` parameters that disable a check only when set to a magic number): not adopted. The magic number guards against setting one by accident; here `COM_ARM_CHECK` is locked while armed, and skipped checks are reported and warned about at every arming.
