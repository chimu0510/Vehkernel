# Setpoint arbitration: the mode decides who is in control, and RC can always take over

Several sources may send setpoints at the same time: the companion computer, the remote control, a ground station, and in test mode the test runner on the development host (ADR-0018). Instead of a fixed source priority, the current mode decides which source is in control, as with PX4's offboard mode. A mode decides two things: which source is in control, and which setpoint levels that source may use (ADR-0003). In companion mode, for example, only setpoints from the Bridge are accepted, at any of the three levels; the RC manual mode accepts only body velocity. Direct actuator is a setpoint level, not a mode.

At boot the vehicle is in `STANDBY`, where no source is in control and nothing can be armed. For now the mode can change only while disarmed; see **Modes while armed** below for what changes when the remote control and companion mode arrive. The source in control is the Link (or, later, the remote control or the Bridge) that requested the mode; setpoints from any other source are rejected and counted.

Two rules apply regardless of mode:

- **The remote control can always take over or trigger the emergency stop.** This is the last line of defence for standalone operation and safety.
- **If the source in control times out, the vehicle falls back to its configured failsafe action. The default depends on the vehicle type and can be overridden by the frame preset (ADR-0013).** A ground vehicle stops, for example; an underwater vehicle holds position or surfaces.

**Emergency stop**

The emergency stop follows ArduPilot's `MOTOR_ESTOP` and PX4's kill switch. It is a firmware function, triggered by the remote control or by a request over any connected Link, whoever is in control:

- **Outputs off at the output stage.** While it is engaged, actuator control forces every output to its disarmed value, whatever the mode, the source in control and the setpoints, and clears the speed loops' integrators (as ArduPilot Rover relaxes its integrators).
- **The vehicle stays armed for a short time.** If the emergency stop is released within 5 s, output resumes from the next setpoint, as with PX4. If it stays engaged for 5 s, the vehicle disarms with the emergency stop as the reason, as PX4 does after its kill switch has been on for 5 s. The 5 s is a constant.
- **Arming is refused while it is engaged**, as ArduPilot's arming checks refuse while the motors are emergency stopped.

Any connected Link may engage or release it, as PX4 accepts kill and unkill from any source. The remote control will be one more source when it exists.

**Output watchdog, fault stop and hardware watchdog**

The firmware also guards against itself stalling or faulting, at three levels, as Betaflight stops its motors in its fault handlers and ArduPilot enables the hardware watchdog:

- **Output watchdog.** The platform layer turns every actuator output off when actuator control has not refreshed it within a few control periods, so a stalled `wq:ctrl` cannot leave a motor running on its last duty, and the emergency stop, which acts in actuator control, still takes effect. It runs from a timer interrupt above every interrupt that may call FreeRTOS (TIM6 on the STM32F4).
- **Fault stop.** A fault (HardFault, MemManage, BusFault, UsageFault, or an exception or interrupt without a handler), a failed `configASSERT`, `fail()` and the FreeRTOS stack overflow hook run above that interrupt or with it masked, so the output watchdog never fires for them. Each ends in the fault stop, which first turns every actuator output off by writing registers only, as Betaflight's `systemFaultAction()` calls `stopMotors()` before anything else. It uses neither FreeRTOS nor the bus, so it works from any context. It then records the fault and where it happened, so it survives the reset, and stops: at a breakpoint when a debugger is attached, otherwise waiting for the hardware watchdog to reset the board. In SITL it ends the run.
- **Hardware watchdog.** The MCU's independent watchdog (the IWDG on the STM32) starts just before the scheduler and is refreshed from the FreeRTOS idle hook, the lowest priority there is, so a scheduler that never gets idle, a task or an interrupt that never returns, or a stopped CPU resets the board. Its timeout is about 2 s, as ArduPilot's: longer than any wait the firmware allows itself, such as erasing a parameter sector, and short enough that a wedged board restarts on its own. It is frozen while a debugger halts the core, so SWD debugging keeps working, and a reset stops it, so the bootloader runs without it. Before the scheduler starts it is not running, and nothing drives an output then: actuator control runs only on `wq:ctrl`. The fault stop starts it if it is not running yet, so every fault ends in a reset unless a debugger holds the core.

**Reset cause**

At boot the platform reads what caused the last reset: power-on, the reset pin, a software reset (a reboot request, ADR-0023), the hardware watchdog, or the fault stop, with the fault and its address. The commander publishes it on `vehicle_status`, the local display shows it, and the Link tools print it. After a watchdog or fault reset arming is refused as an unexpected reset until the next deliberate reset: a reboot request, the reset pin or a power cycle. ArduPilot likewise reports a watchdog reset as an internal error, and an internal error fails its arming checks until the board reboots. A vehicle that stopped itself is not driven again before someone has seen why.

**Arming checks**

Arming checks are of two kinds:

- **Checks of hardware state**, which can be skipped. Each has a bit in the bitmask parameter `COM_ARM_CHECK`, all set by default, as in ArduPilot's `ARMING_CHECK`. Clearing a bit skips that check: it neither refuses arming nor disarms, but its state is still published. Skipping is for bench testing and for boards whose input is unwired or unreliable. `COM_ARM_CHECK` is locked while armed, the vehicle status reports which checks are skipped, and tools warn on arming while any is. A check whose input a board lacks is not available and passes.
- **Checks that cannot be skipped**: the board verified at boot (ADR-0010), the emergency stop released, the mode, the source in control, setpoints cleared, no bring-up case running, actuators started, the local menu closed (ADR-0022), no accepted reboot pending (ADR-0023), and no watchdog or fault reset since the last deliberate reset.

**Board-specific: actuator power sense**

Some boards let the MCU read whether the actuators' power circuit is closed; on the C50C, `PE4` reads the drive power circuit. This is a fact of those boards, declared in their board definition with its powered level (ADR-0011), and has nothing to do with the emergency stop. Where it exists it is a check of hardware state with its own bit in `COM_ARM_CHECK`:

- Arming needs the circuit to have read powered without a break for a settle time (0.5 s), so contact bounce or a loose connector cannot make the vehicle armable.
- A single unpowered sample while armed disarms in that cycle, with actuator power lost as the reason, so the speed loops do not wind up against motors that have lost power and the wheels cannot jump when it returns. Power returning never re-arms.
- With the bit cleared nothing is checked.

**Before the remote control exists**

Until the remote control is implemented, actuators may move only in test mode, under these conditions, which stand in for the RC takeover rule:

- Only direct-actuator and body velocity setpoints are accepted (ADR-0018). The two levels never drive at once: the newer one replaces the other until it times out. Body velocity goes through the control chain and the vehicle model (ADR-0013), and every condition below holds for it as for direct-actuator setpoints.
- The emergency stop can be engaged from any connected Link.
- Actuator output is capped by a parameter, low by default.
- If the source in control times out, the failsafe action is stop and disarm; if no setpoint arrives within a set time after arming, or all targets stay zero for that time, the vehicle disarms as well.

When the remote control exists these conditions stay; it adds a takeover path, it does not replace them.

**Modes while armed (decided, built with the remote control or companion mode)**

The rule that the mode changes only while disarmed holds while `STANDBY` and `TEST` are the only modes. Once the remote control and companion mode exist, taking over means switching modes while armed, and a failsafe action is usually a mode too, so the rule becomes per mode, as in ArduPilot (`Rover::set_mode()` asks each mode's `enter()`) and PX4 (per-mode `can_run` checks):

- Each mode declares whether it may be entered while armed, and what it needs: the source in control, the setpoint levels it accepts and the estimator health it requires.
- `TEST` is still entered only while disarmed.
- The remote control can always switch to its manual mode or engage the emergency stop, and a failsafe can always switch to its action, whatever the current mode allows.

Requests then travel on the bus, as PX4's `vehicle_command` and `vehicle_command_ack` do and as `bringup_command` and `bringup_status` already do here: a queued command topic carrying the source, a sequence number and the request, and an acknowledgement topic matched by source and sequence. The commander takes requests on its own cycle, every source (each Link, the remote control, the Bridge) publishes to the same topic, and requests and their results are logged with the other topics (ADR-0019). Setpoint arbitration moves to the consumer: setpoints are tagged with their source, and those from a source not in control are dropped. Replies then arrive one control cycle later instead of at once. Until a second kind of source exists, Links keep calling the vehicle control service directly.

## Considered Options

- **Fixed priority (RC > ground station > companion computer)**: rejected. While the companion computer is driving autonomously, any higher-priority source could interrupt it at will, and the priority order cannot express "who is driving right now".
- **An emergency stop that disarms at once**: not adopted, to keep ArduPilot's and PX4's semantics, which ground stations and operators already know: engaging stops the outputs at once, and a release within a few seconds resumes without a new arming sequence. The 5 s auto-disarm bounds how long a stopped vehicle stays armed.
- **A safety button in addition to arming** (PX4's `COM_SAFETY_MODE`, ArduPilot's `BRD_SAFETYENABLE`): not adopted. A push button held for about a second turns "safety" off before arming is allowed, and options decide whether disarming turns it back on. It protects propellers while a drone is handled on the bench. Here arming already needs a deliberate request from the source in control, and a vehicle model that needs a button later can add it as one more arming check.
- **Board-specific checks that cannot be skipped**: rejected. Boards differ, and on the bench a sense line may be unwired or the drivers fed from a supply that bypasses it; a check nobody can skip gets patched out of the firmware instead.
- **Resuming armed after a watchdog reset** (ArduPilot restores the armed state when a watchdog reset happens in flight, so a copter keeps flying): rejected. A ground, surface or underwater vehicle is safe stopped, so after a watchdog or fault reset it boots disarmed and refuses arming.
- **Acknowledging an unexpected reset with its own request over a Link**: not adopted for now. A reboot request is already a deliberate act every Link tool can send, and the next boot reads a software reset; an acknowledgement would need a new control message for the same effect.
- **Resetting at once from the fault stop**: not adopted. Waiting for the hardware watchdog keeps the core where a debugger finds the fault, keeps one path to the reset, and stops a fault at boot from becoming a fast reset loop; the outputs are already off by then.
- **Refreshing the hardware watchdog from a high-priority task or the output watchdog's interrupt**: rejected. Either keeps refreshing while the work queues starve, which is what the hardware watchdog must catch. ArduPilot likewise refreshes it from its main loop, not from a timer.
- **PX4's circuit breakers** (`CBRK_*` parameters that disable a check only when set to a magic number): not adopted. The magic number guards against setting one by accident; here `COM_ARM_CHECK` is locked while armed, and skipped checks are reported and warned about at every arming.

## Consequences

- Any code that holds the CPU for longer than the hardware watchdog's timeout without letting the idle task run resets the board, armed or not. A long blocking wait, such as erasing a large Flash sector, must stay well inside it.
- A bootloader that runs before the firmware must leave the reset flags and the fault record alone, or the firmware reports a later cause; on the C50C this is recorded in ADR-0024.
- SITL has no hardware watchdog and no real faults. Its tests raise a failed assertion or `fail()` on request and are given the reset cause at start, as whoever restarts SITL after a reset would.
