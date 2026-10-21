# Spec 0007: local display and local menu

**Status**: in progress. Issue: [#51](https://github.com/chimu0510/Vehkernel/issues/51); tickets #52 to #57, with the hardware check in #57.

This file and the issue carry the same spec; update both when the spec changes.

## Problem Statement

The sixth milestone left Vehkernel driving four wheels in test mode, with modes, arming checks and an emergency stop, all checked in SITL; its hardware check on the C50C V1.1 (#50) waits for a person with the board. Everything the vehicle knows about itself, from the mode and the arming refusal to the attitude, is visible only through a Link, so a board on the bench without a development host says nothing, and a wrong `SERIAL1_BAUD` or `SENS_BOARD_ROT` can be fixed only by a host that can still reach it.

The C50C has an OLED and a user key, S3, which the board definition declares as pins but no code drives. ADR-0022 decides what they may do: a local display shows the vehicle's status, and a local menu changes whitelisted parameters only while disarmed, with arming refused while it is open; it never arms, changes the mode, touches the emergency stop or moves an actuator. None of it exists.

This milestone runs while #50 is still open, so it must not depend on, or change, anything that hardware check may still change: the actuator platform, the `MOT` parameters and the board's actuator facts (`cpr`, the `PE4` powered level).

## Solution

Deliver the seventh milestone: **the vehicle's status on the local display, and a local menu that changes menu parameters with the user key, in SITL and on the C50C V1.1**. Afterwards:

- The local display shows three pages, status, attitude and Link, refreshed at most 5 Hz from topics on the bus, on `wq:slow`. A short press of the user key switches pages.
- A long press while disarmed opens the local menu, which lists the menu parameters by name with their current values. A parameter is edited in a staged value and written only on Save, through the same parameter service as a Link, so the same checks apply and every Link sees the change.
- Arming is refused with the reason `LOCAL_MENU_OPEN` while the menu is open; the menu closes after 30 s without a key press.
- Parameter definitions mark menu parameters with `menu = true` and a `menu_step`; the `MOT` and `COM` groups declare `menu = false` and the generator rejects any menu parameter inside them. The first menu parameters are `SENS_BOARD_ROT`, `SERIAL1_BAUD`, `ATT_KP`, `ATT_KI` and `ATT_ACC_REJ`.
- The board definition declares the display as a board device, an SSD1306 on a write-only software SPI bus, and the user key with its active level and pull.
- SITL runs the same display driver against an SSD1306 model, which can write the screen to an image file, and the user key is pressed over a SITL-only Link frame; CI opens the menu, edits a parameter, sees arming refused and the menu time out.
- On the C50C V1.1 the OLED shows the pages and the menu changes `SERIAL1_BAUD` and `SENS_BOARD_ROT` without a host.

## User Stories

1. As a C50C user, I want the OLED to show the mode, whether the vehicle is armed, the emergency stop, the board verification, actuator power and the last arming refusal, so that I can see why a vehicle will not arm without connecting a host.
2. As a C50C user, I want an attitude page with roll, pitch and yaw in degrees, so that I can check the board orientation by tilting the vehicle.
3. As a C50C user, I want a Link page showing whether a Link is connected, its rate scale and its drop counts, so that I can tell a cable problem from a firmware problem.
4. As a C50C user, I want a short press of the user key to switch pages, armed or disarmed, so that the key is useful while the wheels turn without being able to change anything.
5. As a firmware developer, I want the local display to run on `wq:slow`, refresh at most 5 Hz and redraw only the pages of the screen that changed, so that a display busy-waiting on GPIO lines never delays `wq:ctrl` or a Link (ADR-0022).
6. As a firmware developer, I want the timing monitor to report the display's work item, so that its cost on the board is measured, not guessed (ADR-0015).
7. As a firmware developer, I want a display that is missing to change nothing, so that a board without its OLED fitted still verifies and arms (ADR-0010, ADR-0022).
8. As a C50C user, I want a long press while disarmed to open the local menu, so that I can change a parameter on the bench without a host.
9. As a vehicle user, I want arming refused with the reason `LOCAL_MENU_OPEN` while the menu is open, so that nobody arms a vehicle whose parameters are being edited at it.
10. As a vehicle user, I want the menu to close after 30 s without a key press, discarding any staged edit, so that a menu left open does not block arming from a Link for long.
11. As a vehicle user, I want a long press while armed never to open the menu, and an armed vehicle to close an open menu at once, so that the menu is never open while armed.
12. As a C50C user, I want the menu to list the menu parameters by name with their values, enumerations by value name, so that I can read `SENS_BOARD_ROT = YAW_90` rather than a number.
13. As a C50C user, I want an edit to change only a staged value, with `+` and `-` stepping by `menu_step` within the range, `Default` staging the default, `Save` writing it and `Cancel` discarding it, so that nothing is written until I confirm.
14. As a vehicle user, I want every Save to need the key released first, so that a key held down or stuck writes at most once (ADR-0022).
15. As a firmware developer, I want the menu to write only through the parameter service, with a rejection shown with its reason, so that the menu has no path around the type, range and `locked_when_armed` checks (ADR-0012).
16. As a Link user, I want a change made in the menu to arrive as `parameter_update` like any other, and the menu to reload a value a Link changed, so that the host and the board never disagree for long.
17. As a C50C user, I want a changed `reboot_required` parameter marked as needing a reboot, so that I know a power cycle is due.
18. As a C50C user, I want to change `SERIAL1_BAUD` from the menu, so that a board set to a rate my host cannot use can be recovered without SWD.
19. As a parameter author, I want `menu = true` and `menu_step` in a parameter definition, checked by the generator, so that a parameter enters the menu only by a deliberate change in its own file.
20. As a maintainer, I want the `MOT` and `COM` groups to declare `menu = false` and the generator to reject a menu parameter inside them, so that nothing that drives actuators, an arming check or a failsafe can ever be changed from the menu.
21. As a firmware developer, I want enumeration value names compiled into the firmware for menu parameters only, so that the Flash cost stays bounded (ADR-0012).
22. As a board author, I want to declare a display as a board device with its controller type, bus and pins, and the user key with its active level, checked by the board generator, so that both are hardware facts compared with the schematic (ADR-0011).
23. As a board author, I want a GPIO input pin to declare a pull resistor, so that the C50C's user key reads high when released as in the vendor firmware.
24. As a firmware developer, I want the user key debounced and a key pressed at boot ignored until released, so that a bouncing or stuck key does nothing on its own.
25. As a SITL user, I want SITL to run the same display driver against an SSD1306 model and write the screen to an image file on request, so that I can see the display without hardware.
26. As a SITL user, I want to press the user key over the Link, so that tests and I can operate the menu in SITL.
27. As a Link user, I want the menu's state on a `local_menu` topic and in the Link monitor's live view, so that a script knows whether the menu is open and what it last wrote.
28. As a maintainer, I want CI to open the menu, change a parameter, see arming refused while it is open, see it time out and see it stay shut while armed, in SITL (L3), so that the boundary in ADR-0022 is checked on every push.
29. As a C50C user, I want a documented hardware check of the display and the menu, so that the pages, the key and the timing are confirmed on the board (ADR-0018 L4).
30. As a new contributor, I want the README to describe the pages, the key presses and what the menu can and cannot change, so that I can use it in minutes.

## Implementation Decisions

- **Hardware facts (C50C V1.1)**, from the vendor firmware WHEELTEC_C50X 2026.05.29 (`HARDWARE/oled.c`, `key.c`) and the schematic: the OLED is an SSD1306, 128 × 64, on a 6-pin connector (U11 or J4) carrying GND, 3V3 and four signals, clock `PD7`, data `PD6`, reset `PD5` and data/command `PD4`; there is no chip select and no line back to the MCU, so the bus is write-only and the display cannot be detected. The vendor firmware bit-bangs SPI mode 0, most significant bit first, uses page addressing and resets the controller with reset held low for 100 ms. The user key S3 on `PC5` reads low when pressed; the vendor firmware enables the internal pull-up.
- **Board definition**:
  - `devices` gains the display: `{ device = "DISPLAY0", kind = "DISPLAY", type = "SSD1306", bus = "SOFT_SPI", pins = ["OLED_SCL", "OLED_SDA", "OLED_RST", "OLED_DC"] }`. The generator accepts the bus `SOFT_SPI` for a `DISPLAY` with four `GPIO_OUTPUT` pins in the order clock, data, reset, data/command, and emits `BOARD_HAS_DISPLAY`, `BOARD_HAS_SSD1306` and typed pins, as it does for `SOFT_I2C`.
  - A new optional `user_key = { pin = "USER_KEY", pressed = "LOW" }`, whose pin must be a `GPIO_INPUT`, emitting `BOARD_HAS_USER_KEY`.
  - Pin entries gain an optional `pull` (`UP` or `DOWN`), accepted only on `GPIO_INPUT` pins; without it an input has no pull, as today. The C50C declares `pull = "UP"` on `USER_KEY`.
  - Nothing else in the C50C's board definition changes, in particular not its actuators, `cpr` or `actuator_power`.
- **Parameter generator**:
  - A parameter may set `menu = true`; then `menu_step` (positive, in the parameter's unit) is required for an `INT32` without `values` and for a `FLOAT`, and rejected for an enumeration, which steps through its values. A parameter file may set `menu = false` at file level; the generator then rejects `menu = true` in it.
  - `modules/actuators/params.toml` and `modules/commander/params.toml` gain `menu = false`, and no other change.
  - The generator emits, for menu parameters only, the list of their indices and their enumeration value names, compiled into the firmware; the metadata file gains `menu` and `menu_step`. The parameter set hash covers them, as it covers all metadata.
  - Menu parameters: `SENS_BOARD_ROT` and `SERIAL1_BAUD` (enumerations), `ATT_KP` (step 0.1), `ATT_KI` (step 0.01) and `ATT_ACC_REJ` (step 0.01).
- **Interface layer contracts**: a write-only display bus (pulse reset, write command bytes, write data bytes), satisfied on the STM32F4 by a software SPI on four GPIO lines and in SITL by the SSD1306 model; and the existing GPIO input, now with the declared pull applied at start (ADR-0009).
- **SSD1306 driver (driver layer)**: initialises the controller with the vendor firmware's command sequence and turns the display on; holds a 1 KB frame buffer; writes only the pages (8 rows of 128 columns) that changed since the last write, at most two pages per run so each run's busy-wait stays bounded. It reports nothing about presence: the bus cannot answer.
- **Font**: a 6 × 8 ASCII font, giving 21 columns and 8 lines, defined in the repository from a public-domain source; the vendor's font table is not copied.
- **Local display (module)**: a work item on `wq:slow` running every 200 ms. It reads the latest `vehicle_status`, `vehicle_attitude`, `link_status` and `local_menu`, renders the current page into the frame buffer and lets the driver write the changed pages. Pages:
  - Status: mode, armed or disarmed, the emergency stop, board verified, actuator power, the skipped arming checks and the last arming refusal or disarm reason, by name.
  - Attitude: roll, pitch and yaw in degrees, one decimal, from the quaternion in `vehicle_attitude`.
  - Link: per Link instance, the handshake state, `rate_scale`, and the `drops` and `lost` counts.
  - While the menu is open, the menu replaces the pages.
- **User key (module)**: sampled every 10 ms on `wq:slow`. A level held for three samples in a row counts (30 ms debounce). A key pressed at boot is ignored until released. A press released before 1 s is a short press; a press held for 1 s is a long press, reported once at 1 s while held; the next event needs a release first.
- **Local menu (module)**:
  - Closed: a short press switches pages; a long press opens the menu if `arming_state` shows disarmed, and is ignored while armed.
  - List: the menu parameters by name with their current values, then `Exit`. A short press moves to the next item; a long press on a parameter starts an edit, on `Exit` closes the menu.
  - Edit: shows the parameter, its current value and the staged value, and the actions `+`, `-`, `Default`, `Save` and `Cancel`. A short press moves to the next action; a long press runs it. `+` and `-` step the staged value by `menu_step`, clamped to the range, or to the next or previous enumeration value; held on, they repeat every 0.2 s after the long press. `Default` stages the default. `Save` calls the parameter service with the staged value, shows `OK` or the rejection's name, and returns to the list; `Cancel` returns without writing. A `reboot_required` parameter whose value differs from the value at boot shows `REBOOT`.
  - A `parameter_update` for a menu parameter reloads its current value; a staged value is kept, and `Save` writes it (the last write wins).
  - 30 s without a key press closes the menu and discards a staged edit (a constant).
  - If `arming_state` shows armed while the menu is open, which only a race between an arm request and a long press can cause, the menu closes at once and discards a staged edit.
  - It publishes `local_menu` and has no other output: no setpoints, mode, arming or emergency stop requests.
- **Topics (schema)**:
  - `local_menu`: `timestamp`, `state` (0 closed, 1 list, 2 edit), `item` (the parameter index, or 65535 for `Exit` or none), the last `Save`'s result (a parameter service result, 255 for none) and the parameter it wrote, and a `reboot_required` flag set once any menu parameter needs a reboot. Published on every change and at 1 Hz; outbound on the Link at 5 Hz.
- **Commander**: subscribes to `local_menu` and refuses an arm request while its state is not closed, with the new arming refusal `LOCAL_MENU_OPEN` (9), a check that cannot be skipped (ADR-0005). This is the only change to the commander; no actuator code changes.
- **Custom protocol**: stays at version 3. The new arming refusal is one more reason value, and the new frame is SITL-only:
  - `sim_user_key` (pressed or released), SITL only, like `sim_actuator_power`.
- **Platform layer, STM32F4**: the software SPI clocks bits on the four GPIO lines with at least 100 ns per half period, within the SSD1306's 10 MHz serial clock; the user key's pull is applied from the board definition. `BOARD_HAS_DISPLAY` and `BOARD_HAS_USER_KEY` turn on `LOCAL_DISPLAY` and `LOCAL_MENU` in the C50C's defconfig.
- **Platform layer, SITL**: the SSD1306 model lives in `sim/`, like the ICM20948 and NOR Flash models, and decodes the command and data bytes into its own 128 × 64 image, which unit tests read. `--display <path>` writes that image as a PBM file after each write. The user key is a simulated input, released at start, set by `sim_user_key`.
- **Kconfig**: `LOCAL_DISPLAY` depends on `BOARD_HAS_DISPLAY || PLATFORM_SITL`; `LOCAL_MENU` depends on `LOCAL_DISPLAY` and `BOARD_HAS_USER_KEY || PLATFORM_SITL`. Both are on by default where they can be. Without `LOCAL_MENU` the commander never sees the menu open.
- **Link monitor**: `key short|long`, and `key hold <s>` for a press held for a given time, sent as `sim_user_key` frames (SITL only, refused by the board); the live view shows `local_menu`.
- **Isolation from #50**: no change to the actuator platform, actuator control, the `MOT` parameters' values or the board's actuator facts. The commander gains one arming check; the `MOT` and `COM` parameter files gain one file-level line. If #50 changes any of them, nothing here needs to follow.

## Testing Decisions

- **Good tests check external behaviour only**: what the SSD1306 model shows, what `local_menu` and `vehicle_status` report, which parameter values a Link reads back and what the generators emit or reject. The menu's internal states are not asserted.
- **Seam A: SITL (L3), the main seam.** `test/sitl/check_local_menu.py` runs SITL with `--param-file` and the Link monitor library and checks, in CI:
  1. A long press opens the menu; `local_menu` reports it open; an arm request is refused with `LOCAL_MENU_OPEN`.
  2. Editing `SENS_BOARD_ROT` to `YAW_90` and saving: a Link reads `YAW_90`, `parameter_update` arrives and `local_menu` reports the result and `reboot_required`; after a restart with the same parameter file the value is still `YAW_90`.
  3. Stepping `ATT_KP` with `+` past its maximum stays at the maximum; `Cancel` leaves the value unchanged; `Default` and `Save` restore the default.
  4. 30 s without a key press closes the menu and discards a staged edit; arming is then accepted in `TEST`.
  5. Armed in `TEST`, a long press does not open the menu and short presses only switch pages.
  6. A value a Link sets while the menu shows it appears in the menu; the image written with `--display` shows it.
  The existing SITL checks keep passing.
- **Seam B: rendering (L1, GoogleTest).** The display module and the driver against the SSD1306 model: each page renders the given topic values as expected, compared with golden images; only changed pages are written, at most two per run; the initialisation sequence matches the vendor firmware's.
- **Seam C: user key and local menu (L1).** Debounce at its boundary (two samples and three); a key pressed at boot is ignored until released; short and long presses at the 1 s boundary; a held key confirms `Save` once; the repeat of `+` while held; every action on an integer, a float and an enumeration parameter, with clamping to the range; a rejected `Save` shows the result; the 30 s close at its boundary; closing on arming; a `parameter_update` reloads the value and keeps the staged one; `local_menu` on every change.
- **Seam D: commander (L1).** `LOCAL_MENU_OPEN` refuses arming in each open state and not when closed; it cannot be skipped by `COM_ARM_CHECK`; the existing arming tests keep passing unchanged.
- **Seam E: generators (L0 and L1).** Board generator golden output for the display device, `user_key` and `pull`, and a diagnostic each for `SOFT_SPI` with the wrong number of pins or a pin that is not a `GPIO_OUTPUT`, `user_key` on an output pin and `pull` on an output pin. Parameter generator golden output for menu parameters and their enumeration names, and a diagnostic each for `menu = true` without `menu_step`, `menu_step` on an enumeration and `menu = true` in a file with `menu = false`.
- **Seam F: hardware (L4, manual).** On the C50C V1.1, wheels not involved: the pages appear after boot and follow the vehicle (tilt for attitude, unplug the USB cable for the Link page); a short press switches pages; the menu changes `SENS_BOARD_ROT` and, after a reboot, the attitude follows it; the menu changes `SERIAL1_BAUD` to 57600 and back, and after each reboot the Link monitor connects at the new rate; arming over the Link is refused while the menu is open; the timing monitor reports the display's work item with no `wq:ctrl` or `wq:io` overruns; the board verifies and arms with the OLED unplugged. Results are recorded on the ticket.
- **Prior art**: `test/sitl/check_params.py` for SITL parameter checks with a parameter file; the ICM20948 and NOR Flash models in `sim/`, which the SSD1306 model follows; `sim_actuator_power` for a SITL-only control frame; the board and parameter generator golden tests and diagnostic cases.

## Out of Scope

- Logging (ADR-0019), which moves to spec 0008.
- The C50C V1.2 board definition, whose display data, reset and data/command lines moved to `PD4`, `PE1` and `PE0`.
- Arming, mode changes and the emergency stop from the menu or the key, rebooting from the menu and resetting every parameter (ADR-0022).
- Menu parameters beyond the five listed, and any parameter of the `MOT` or `COM` groups.
- Display brightness or contrast as a parameter, graphics beyond text, and pages for later milestones such as battery.
- Displays on I2C or hardware SPI, and detecting whether a display is fitted.
- Calibration started from the menu.

## Further Notes

- **Why now**: the display and menu touch none of what #50 may still change, and they make that hardware check easier: the status page shows an arming refusal without reading the Link, and `SERIAL1_BAUD` can be recovered at the board.
- **Why `SERIAL1_BAUD` in the menu**: it is `reboot_required`, and a rate the host cannot use cuts the only Link, so today recovery needs SWD; the menu restores it at the board.
- **Why the parameter name as the label**: names are at most 16 characters and already in the firmware, and the 6 × 8 font gives 21 columns, so a separate label would cost Flash and add a second name for the same parameter (ADR-0012).
- **Why no detection**: the C50C's display has no line back to the MCU, so an unplugged OLED looks the same as a working one; ADR-0022 already keeps the display out of board verification and arming, so nothing needs to know.
- **Protocol version 3 kept**: the only new frame is SITL-only and the new arming refusal is a new value of an existing field; host tools come from the same build (ADR-0002).
