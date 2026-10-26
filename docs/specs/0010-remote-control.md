# Spec 0010: the remote control (a USB gamepad, the manual mode, takeover and the RC failsafe)

**Status**: in progress. Issue: [#120](https://github.com/chimu0510/Vehkernel/issues/120); tickets #121 to #126, with the hardware check in #126.

Until this spec is done, this file and its issue carry the same spec; update both in the same edit.

## Problem Statement

Spec 0009 left the C50C V1.1 able to drive the vehicle as a whole by body velocity, but only in `TEST` mode and only from the development host over a Link, with the wheels lifted. The vehicle cannot yet drive on the ground: ADR-0005 makes the remote control the last line of defence (it can always take over or trigger the emergency stop), and until it exists actuators move only in `TEST`, capped low. Nobody can drive the vehicle standalone, take it over from the host, or stop it from a hand-held device, and nothing happens when such a device is lost.

Three decisions of ADR-0005 wait for this milestone. The remote control's manual mode accepts body velocity only. Modes change only while disarmed today, which contradicts "the remote control can always take over" (#82). And requests reach the commander through a synchronous interface each Link calls, which does not scale to a second kind of source (#76).

The C50C V1.1 has a USB host port (CN8, `PA11` and `PA12`, USB OTG FS) for a gamepad, and an SBUS input and four PWM receiver inputs (ADR-0024). The gamepad on hand is a Flydigi pad, which enumerates as an Xbox 360 controller (XInput, vendor class `0xFF`, `045E:028E`), as WHEELTEC's own firmware treats it.

## Solution

Deliver the tenth milestone: **a USB gamepad as the remote control, a `MANUAL` mode it drives by body velocity, takeover from any mode while armed, the emergency stop from the gamepad, and a failsafe when the gamepad is lost**. Afterwards:

- A Flydigi, or any Xbox 360-compatible, pad plugged into the C50C's USB host port is read through TinyUSB and an XInput driver, and published as `manual_control_input`: sticks, triggers, buttons and whether it is connected.
- In `MANUAL` the left stick drives forward and back and the right stick turns, as a body velocity setpoint through the control chain of spec 0009, scaled by `MAN_VX_MAX` and `MAN_YAW_MAX`.
- Buttons on the gamepad arm and disarm, take over in `MANUAL` from any mode, armed or not, and engage and release the emergency stop.
- Each mode says whether it can be entered while armed (#82). `TEST` still only while disarmed; the remote control can always switch to `MANUAL`.
- Losing the gamepad in `MANUAL` (no report for `RC_TIMEOUT`, or unplugged) stops and disarms, as a setpoint timeout does in `TEST`.
- Mode, arming, emergency stop, bring-up and reboot requests travel on the bus as `vehicle_command` and `vehicle_command_ack`, from every source alike, and setpoint arbitration moves to the consumers (#76).
- SITL simulates a gamepad over the Link, so CI drives `MANUAL`, takeover and the failsafe.
- On the C50C V1.1 the vehicle drives on the ground with the gamepad, and the odometry's error is measured there (deferred from spec 0009).

## User Stories

1. As an operator, I want to plug a USB gamepad into the board and drive the vehicle with its sticks, so that the vehicle drives standalone, with no development host (ADR-0001).
2. As an operator, I want the left stick to drive forward and back and the right stick to turn, at speeds set by parameters, so that the vehicle drives like any RC ground vehicle.
3. As an operator, I want to arm and disarm from the gamepad with a deliberate hold, so that a bump does neither.
4. As an operator, I want one button to take over in `MANUAL` whatever mode the vehicle is in, armed or not, so that I can always take the vehicle back from the host (ADR-0005).
5. As an operator, I want an emergency stop button on the gamepad, so that the last line of defence is in my hand (ADR-0005).
6. As an operator, I want the vehicle to stop and disarm when the gamepad is unplugged, out of range or silent, so that a lost remote control never leaves the vehicle driving.
7. As an operator, I want arming in `MANUAL` refused while the gamepad is not connected or a stick is off centre, so that the vehicle never jumps when it arms.
8. As a developer on the bench, I want `TEST` to keep working as in spec 0009, and to be entered only while disarmed, so that tests and bring-up cases are unchanged.
9. As a firmware developer, I want each mode to declare whether it may be entered while armed and what it needs, so that `MANUAL`, `TEST` and, later, companion mode follow one rule (#82).
10. As a firmware developer, I want every request (mode, arming, emergency stop, bring-up, reboot) to travel on the bus with its source and an acknowledgement, so that the remote control, the Links and, later, the Bridge reach the commander the same way and every request is logged (#76, ADR-0019).
11. As a firmware developer, I want setpoints tagged with their source and dropped by their consumer when that source is not in control, so that setpoint arbitration no longer needs a call into the commander from every Link.
12. As a vehicle integrator, I want any Xbox 360-compatible pad to work, matched by its interface rather than one product ID, so that a replacement pad needs no firmware change.
13. As a firmware developer, I want SITL to simulate a gamepad over the Link, so that `MANUAL`, takeover, the gamepad's emergency stop and the failsafe are checked in CI without hardware (ADR-0018).
14. As a developer, I want the Link monitor to show the gamepad's state and to drive SITL's simulated gamepad, so that I can check the remote control without a board.
15. As a C50C user, I want a documented hardware check driving on the ground with the gamepad, with the odometry's error measured over a known route, so that spec 0009's control chain and odometry are confirmed on a vehicle that moves (ADR-0018 L4).

## Implementation Decisions

- **USB host stack** (ADR-0016): TinyUSB, pinned as a submodule under `external/`, MIT-licensed and so compatible with GPL-3.0-or-later; ST's USB Host Library is not used for its licence. It runs the C50C's USB OTG FS core as a host on `PA11` and `PA12`, from its own FreeRTOS task above the work queues, as the vendor firmware runs its host stack. Kconfig `RC_USB_GAMEPAD` builds it in; boards declare the USB host port in their board definition (ADR-0011).
- **XInput driver**: a TinyUSB application class driver that claims an interface of class `0xFF`, subclass `0x5D`, protocol `0x01` (the Xbox 360 wired gamepad protocol), whatever the vendor and product IDs, and reads its 20-byte input report: buttons in bytes 2 and 3, the triggers in 4 and 5, the four stick axes as signed 16-bit values in 6 to 13. HID-class gamepads and the Xbox One protocol are out of scope. Rumble is not used.
- **Remote control input topic**: `manual_control_input` (schema, ADR-0006), as PX4's `manual_control_setpoint`: timestamp, source (an enum, `USB_GAMEPAD` now; SBUS later), connected, the sticks normalized to -1 to 1 (left x and y, right x and y, y positive forward), the triggers 0 to 1, and the buttons as a bitmask in a fixed layout that every source maps into (the Xbox 360 controller's). Published at least 20 times a second while the gamepad is connected, its latest state repeated when the gamepad sends nothing new (an XInput pad reports only changes, so a pad held still would otherwise look lost), and once when it disconnects. The work item that publishes it knows nothing of modes.
- **Mapping** (parameters `MAN_*`, ADR-0012): in `MANUAL`, `vx = MAN_VX_MAX · left y` and `yaw rate = MAN_YAW_MAX · right x` (right stick right turns right, z down), each after a deadband `MAN_DEADZONE` around the centre; `vy` stays 0 until a frame can move sideways. Defaults 0.5 m/s, 1.0 rad/s and 0.08. The result is a body velocity setpoint from the remote control as source, through the control chain unchanged.
- **Buttons** (fixed, in the topic's bitmask layout):
  - Arm: hold `START` for 1 s while both sticks are centred; disarm: hold `BACK` for 1 s.
  - Take over in `MANUAL`: press `A`, from any mode, armed or not.
  - Emergency stop: press `B` to engage; hold `Y` for 1 s to release.
  - Buttons act on edges and holds measured in the commander's own cycle, so a held button does not repeat.
- **Modes** (ADR-0005, #82): `MANUAL` joins `STANDBY` and `TEST` in the mode enum. Each mode declares whether it may be entered while armed and what it needs: its source in control, the setpoint levels it accepts and its arming checks. `TEST` may be entered only while disarmed, from a Link, and accepts direct-actuator and body velocity setpoints. `MANUAL` may be entered at any time, from the remote control only, and accepts body velocity setpoints from the remote control only. Entering `MANUAL` while armed makes the remote control the source in control at once, and the next cycle's setpoint comes from the sticks. ADR-0005 is edited to say so and its "Modes while armed" section becomes the decision as built.
- **Arming in `MANUAL`**: two more checks that cannot be skipped, the remote control connected and both sticks inside the deadband, in addition to every check of ADR-0005.
- **RC failsafe** (ADR-0005, ADR-0012): in `MANUAL`, no `manual_control_input` for `RC_TIMEOUT` (default 0.5 s) or a disconnect stops and disarms, with a new disarm reason `RC_LOST`. The idle disarm (`COM_DISARM_IDLE`) applies in `MANUAL` too. A failsafe action other than stop and disarm, such as hold, is out of scope: the user chose stop and disarm as the default, and the parameter that will choose another waits for a second action.
- **Requests on the bus** (#76, ADR-0005, ADR-0008): a queued `vehicle_command` topic carries the source, a sequence number, the request (mode, arming, emergency stop, bring-up, reboot) and its argument; the commander takes requests on its own cycle and answers on `vehicle_command_ack` with the source, sequence and result. The Link waits for the acknowledgement before it replies to the host, so its replies come one control cycle later. `core::VehicleControl` and its critical sections go. A bring-up request travels on `vehicle_command` like the others; the commander still hands an accepted case to actuator control on `bringup_command`.
- **Setpoint arbitration at the consumer** (#76): `actuator_setpoint` and `body_velocity_setpoint` carry their source; Links publish what the host sends, tagged with the Link, and the control chain and actuator control drop setpoints from a source the commander's `vehicle_status` does not show in control, or at a level its mode does not accept. The commander applies the same rule to follow the setpoints for the timeout and the idle disarm, and counts the ones it drops on `vehicle_status`; `link_status` no longer counts them. ADR-0005 and ADR-0008 are edited.
- **SITL** (ADR-0018): a simulated gamepad behind the same topic, driven by a `sim_gamepad` Link frame (connected, sticks, buttons), as `sim_user_key` drives the user key. The Link monitor gains `sim-gamepad` to set it, and a `gamepad` line in its live view and `--check` summary for `manual_control_input`.
- **Output cap**: `MOT_OUT_MAX` stays 0.3 by default; the ground drive sets it for the test vehicle and records the value on the hardware check ticket.

## Testing Decisions

- **Good tests check external behaviour only**: the modes, arming results and setpoints the firmware publishes for given gamepad input and requests, what the SITL vehicle does, and what the Link tools report.
- **Seam A: SITL (L3).** New scenarios in `test/sitl/`, run in CI, with the simulated gamepad:
  - Arm with `START` held 1 s, drive a square with the sticks, disarm with `BACK`: the vehicle follows `vehicle_local_odometry_groundtruth` within set tolerances, and arming is refused with a stick off centre or the gamepad disconnected.
  - Takeover: the host drives in `TEST`, armed; `A` switches to `MANUAL` without disarming, the host's setpoints are then dropped and counted, and the sticks drive.
  - RC failsafe: the gamepad disconnects, or stops reporting, while driving in `MANUAL`: the vehicle stops and disarms with `RC_LOST` after `RC_TIMEOUT`.
  - Emergency stop from the gamepad: `B` zeroes the outputs at once, `Y` held 1 s releases it, and 5 s engaged disarms, as from a Link.
  - Every existing SITL scenario still passes over requests on the bus.
- **Seam B: modules (L1).** The XInput report parser against recorded reports; the mapping with its deadband; button holds and edges; per-mode entry rules and arming checks; the command and acknowledgement round trip; arbitration at the consumer, including a setpoint from a Link dropped in `MANUAL`.
- **Seam C: generators (L0, L1).** The new topics and enums.
- **Seam D: hardware (L4, manual).** On the C50C V1.1 with the four-wheel differential swing-arm vehicle of #110 and the Flydigi pad, on the ground:
  - The pad enumerates and `manual_control_input` follows the sticks and buttons.
  - Arm, drive, turn and disarm with the pad; directions and speeds as the parameters say.
  - Take over from a host driving in `TEST`; the emergency stop from the pad; unplug the pad while driving: the vehicle stops and disarms.
  - Drive a measured 5 m straight and a square, and record the odometry's position and heading error against the measured ground truth.
  - No work item overruns with the USB host running.
  Results are recorded on the ticket.
- **Prior art**: `test/sitl/check_body_velocity.py` and `check_routes.py` for driving SITL by body velocity against the truth; the `sim_user_key` frame and `check_local_menu.py` for simulated input over the Link; `bringup_command` and `bringup_status` for a command and its result on the bus.

## Out of Scope

- SBUS and PWM receivers, and HID-class or Xbox One gamepads: the topic leaves room for them.
- Failsafe actions other than stop and disarm, and per-mode failsafe parameters beyond `RC_TIMEOUT`.
- Companion mode, the ROS2 Bridge, `/cmd_vel` and MAVLink (ADR-0002, ADR-0003).
- Rumble and the gamepad's LEDs.
- The user key as an emergency stop source (ADR-0022), and showing the gamepad on the local display.
- Per-topic schema handling (#83).

## Further Notes

- **Why TinyUSB**: the vendor's ST USB Host Library is proven on this board, but its ST licence is not GPL-compatible, and TinyUSB's host stack is portable, MIT-licensed and runs on FreeRTOS. The XInput driver is small and is written here rather than pinned.
- **Why stop and disarm on RC loss**: the user chose it; it matches `TEST`'s setpoint timeout, and a ground vehicle is safe stopped (ADR-0005).
- **Why requests move to the bus now**: the remote control is the first source that is not a Link, which is when #76 said to do it; doing it before `MANUAL` keeps one way into the commander.
