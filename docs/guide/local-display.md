# Local display and local menu

A board with a display and a user key shows the vehicle's status on the display, and its local menu changes a few parameters without a development host (ADR-0022). The board's display and key are on its board page, for example [C50C V1.1](../boards/c50c-v1_1.md).

## Pages

The local display shows three pages of 21 columns by 8 lines, whose values change at most five times a second, while what the user key changes is on the panel within 50 ms:

| Page | Shows |
|---|---|
| Status | Mode, arming, the emergency stop, board verification, actuator power, skipped arming checks, the last arming refusal and disarm reason, output watchdog trips |
| Attitude | Roll, pitch and yaw in degrees |
| Link | Each Link's handshake, rate scale, drops and lost frames |

## User key and local menu

A short press (released before 1 s) switches pages, armed or disarmed. A long press (held 1 s) while disarmed opens the menu, and **arming is refused with "local menu open" while it is open**. The menu lists the menu parameters by name, then EXIT; the value of the one under the cursor is on the bottom line:

- In the list, a short press moves to the next item; a long press edits it, or on EXIT closes the menu.
- In an edit, the screen shows the current value (NOW) and a staged value (NEW). A short press moves the caret through `+`, `-`, `DEF`, `SAVE` and `CANCEL`; a long press runs the one under it. `+` and `-` step by the parameter's menu step within its range, or through an enum's values, and repeat every 0.2 s while the key stays held. `DEF` stages the default. Nothing is written until `SAVE`, which goes through the same parameter checks as a Link and shows `SAVED` or why it was refused; every connected Link sees the change. A changed parameter that needs a reboot shows `REBOOT`.
- The menu closes by itself after 30 s without a key press, discarding a staged edit, and at once if the vehicle is armed.

## Menu parameters

The menu parameters are `SENS_BOARD_ROT`, `SERIAL1_BAUD` and `SERIAL2_BAUD` (on the board), `ATT_KP`, `ATT_KI` and `ATT_ACC_REJ`; a parameter enters the menu only with `menu = true` and a `menu_step` in its definition. Nothing in the `MOT` or `COM` groups ever does: the menu never arms, changes the mode, touches the emergency stop or moves an actuator. Changing `SERIAL1_BAUD` or `SERIAL2_BAUD` from the menu recovers a board set to a rate the host cannot use.

## In SITL

SITL simulates an SSD1306; `--display <path>` writes what it shows to a PBM image after every change, which any image viewer opens:

```bash
build/sitl/app/vehkernel-sitl --duration 0 --display /tmp/vk-display.pbm
```

The [Link monitor](link-monitor.md) presses the simulated user key, then prints the menu's state, which the live view also shows:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 key long
tools/link/link_monitor.py --udp 127.0.0.1:15600 key short
tools/link/link_monitor.py --udp 127.0.0.1:15600 key hold 5
```

CI reads the status page back from the image (`sitl_display`), and opens the menu, saves parameters, sees arming refused while it is open and the menu time out (`sitl_local_menu`).
