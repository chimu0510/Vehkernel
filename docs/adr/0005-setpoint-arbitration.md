# Setpoint arbitration: the mode decides who is in control, and RC can always take over

Several sources may send setpoints at the same time: the companion computer, the remote control, a ground station, and in test mode the test runner on the development host (ADR-0018). Instead of a fixed source priority, the current mode decides which source is in control, as with PX4's offboard mode. A mode decides two things: which source is in control, and which setpoint levels that source may use (ADR-0003). In companion mode, for example, only setpoints from the Bridge are accepted, at any of the three levels; the RC manual mode accepts only body velocity. Direct actuator is a setpoint level, not a mode.

At boot the vehicle is in `STANDBY`, where no source is in control and nothing can be armed. Whether a mode can be entered while armed is the mode's own rule; see **Modes while armed** below. The source in control is the Link (or, later, the remote control or the Bridge) that requested the mode; setpoints from any other source are dropped and counted (see **Requests and setpoint arbitration on the bus**).

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

**Test mode and the manual mode**

Actuators move only in `TEST` or `MANUAL`, under these conditions:

- In `TEST` only direct-actuator and body velocity setpoints are accepted (ADR-0018). The two levels never drive at once: the newer one replaces the other until it times out. Body velocity goes through the control chain and the vehicle model (ADR-0013), and every condition below holds for it as for direct-actuator setpoints.
- In `MANUAL` the remote control drives by body velocity from its sticks; it takes over from any mode with one button, arms and disarms with deliberate holds, and arming needs it connected with its sticks centred (spec 0010).
- The emergency stop can be engaged from any connected Link and from the remote control.
- Actuator output is capped by a parameter, low by default.
- If the source in control times out, the failsafe action is stop and disarm; if no setpoint arrives within a set time after arming, or all targets stay zero for that time, the vehicle disarms as well. In `MANUAL`, the remote control lost (`RC_TIMEOUT` without input, or disconnected) stops and disarms with `RC_LOST`. A failsafe action other than stop and disarm waits until a vehicle needs one.

The remote control's input is repeated at a steady rate while it is connected, as a gamepad that sends only changes would otherwise fall silent while held still; silence then always means it is lost.

**Modes while armed**

A global rule that the mode changes only while disarmed would contradict the remote control's takeover, and a failsafe action is usually a mode too, so the rule is per mode, as in ArduPilot (`Rover::set_mode()` asks each mode's `enter()`) and PX4 (per-mode `can_run` checks). Each mode declares, in one table every consumer shares (`core::mode_rules`), whether it can be armed, whether it may be entered while armed, which sources may request it and so be in control in it, the setpoint levels it accepts, and its extra arming checks:

| Mode | Armable | Entered while armed | Requested by | Setpoint levels | Extra arming checks |
|---|---|---|---|---|---|
| `STANDBY` | no | no | any source | none | |
| `TEST` | yes | no | a Link | direct actuator, body velocity | |
| `MANUAL` | yes | yes | the remote control | body velocity | the remote control connected, its sticks centred |

- Entering a mode other than `STANDBY` makes the requesting source the source in control. Entering `MANUAL` while armed takes over at once without disarming; the setpoint timeout then counts from the takeover, so the old source's silence is not held against the new one. A bring-up case running at that moment ends with a disarm, since it drives the outputs open loop.
- The remote control can always switch to `MANUAL` or engage the emergency stop, whatever the current mode allows; a failsafe action that is a mode will be allowed the same way when one exists.
- A mode needing estimator health declares it here when one does.

**Requests and setpoint arbitration on the bus**

Requests travel on the bus, as PX4's `vehicle_command` and `vehicle_command_ack` do. Every source (each Link now, the remote control and the Bridge later) publishes its mode, arming, emergency stop, bring-up and reboot requests on one queued topic, `vehicle_command`, with the source, its own sequence number, the request and its argument. A source that falls silent says so there too, which aborts a bring-up case it started. The commander takes the requests on its own cycle, in order, and answers each on `vehicle_command_ack`, matched by source and sequence, with its result and the state after it (the mode, armed, the emergency stop and the skipped checks), which is all a source needs for its reply. A Link replies to its host once the answer arrives, a control cycle later than a direct call would; one the commander has not answered within 200 ms goes unanswered, and the host repeats it, as requests are idempotent. Nothing calls into the commander from another work queue, so it takes no critical sections, and requests and their results are logged with the other topics (ADR-0019). A bring-up case the commander accepts still goes to actuator control on `bringup_command`.

Setpoint arbitration happens at the consumer. Setpoints are tagged with their source: a Link writes its own instance into each one it publishes, whatever the host wrote there. Each consumer (actuator control for direct-actuator setpoints, the control chain for body velocity) drops a setpoint unless its source is the one in control and the mode accepts its level, judged on the newest `vehicle_status`; arming and the emergency stop then decide whether it drives. The commander applies the same rule to follow the setpoints for the timeout and the idle disarm, and counts the ones it drops on `vehicle_status`.

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
