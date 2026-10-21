# Local display and local menu: status always, whitelisted parameters only while disarmed

A board with a display and a user key shows the vehicle's status on the display and offers a local menu that changes parameters, so a vehicle can be checked and set up on the bench without a development host. The C50C V1.1 has both: an OLED on four GPIO lines and the user key S3 (board definition). The local menu is one more client of the parameter service (ADR-0012) and nothing else: it is never a source of setpoints, mode changes or arming requests, and nothing it does can make an actuator move. Betaflight's CMS menu is the model: it opens only while disarmed, and arming is refused while it is open.

**Local display**

- The local display subscribes to topics on the bus and publishes nothing the control chain reads. Producing modules never know it exists, as with the logger (ADR-0019).
- It runs on the lowest-priority work queue, never on `wq:ctrl`. A display on GPIO lines busy-waits for every byte, so it refreshes at a bounded rate and redraws only what changed; the timing monitor measures its cost (ADR-0015).
- Pages: status (mode, arming state, emergency stop, refused and skipped arming checks, board verified), attitude, and Link status. Later milestones add their own pages, such as battery.
- A display that is missing or does not answer is reported over a Link where its bus can tell; on a write-only bus, such as the C50C's, the firmware cannot know. Either way it affects nothing else: it is not part of verifying the board at boot (ADR-0010) and not an arming check.
- While armed the user key only switches status pages.

**Local menu**

- **Only while disarmed.** The local menu opens only while the vehicle is disarmed.
- **Arming is refused while it is open.** "Local menu closed" is an arming check that cannot be skipped (ADR-0005). The menu closes itself after 30 s without a key press, discarding any unconfirmed edit, so a menu left open on the bench does not block arming from a Link for long; the refusal reason tells the Link why. The 30 s is a constant.
- **One gate.** Every change goes through the same parameter service as a Link, with the same type, range and safety attribute checks. A rejection is shown with its reason. Each change publishes `parameter_update`, so every connected Link sees it, and storage follows ADR-0020; the vehicle is disarmed, so the spare sector may be erased.
- **A whitelist, not a blacklist.** Only parameters whose definition sets `menu = true` appear in the local menu, so a new parameter stays out of it until someone decides otherwise. A parameter group that drives actuators or holds arming checks, failsafe actions or setpoint timeouts declares `menu = false` for the whole file, and the parameter generator rejects `menu = true` inside it; today these are `MOT` and `COM`. The first whitelist is `SENS_BOARD_ROT`, `SERIAL1_BAUD` and the attitude estimator's `ATT_KP`, `ATT_KI` and `ATT_ACC_REJ`, whose worst case is a poor attitude estimate, not motion.
- **Staged edits.** A short press moves to the next item or value, a long press selects or confirms. A value is staged in the menu and written only when confirmed; each confirm needs the key released first, so a key held down or stuck confirms at most once. Numeric parameters step by their `menu_step` within their range, which the parameter generator requires with `menu = true`; enumerations step through their value names. When a Link changes a parameter the menu shows, the menu reloads it from `parameter_update`, and the last confirmed write wins.
- **Reset to default.** Each menu parameter can be reset to its default, through the same gate.
- **Reboot required.** A changed `reboot_required` parameter is marked as needing a reboot; the menu does not reboot the board.
- **Key handling.** The user key is debounced. A key that reads pressed at boot is ignored until it has been released.

**Not in the local menu**

Arming and disarming, mode changes, engaging or releasing the emergency stop, starting bring-up cases, anything that moves an actuator, rebooting, and resetting every parameter at once.

**Firmware metadata**

The local menu shows each parameter by its name, which the firmware already holds. Enumeration value names are compiled into the firmware for menu parameters only; every other parameter keeps only what validation needs (ADR-0012). The whitelist bounds the Flash this costs.

**Board definition**

A display is a board device in the board definition (ADR-0011), with its controller type and peripheral bus, and the user key is a pin role. Board revisions that move display pins are separate board definitions (ADR-0010): the C50C V1.2 moved the display's reset and data/command lines.

## Considered Options

- **Arming, mode changes or the emergency stop from the local menu**: rejected. The menu would become a source in setpoint arbitration (ADR-0005), operated by one key with no second confirmation, by someone standing next to the wheels.
- **The user key as an emergency stop source**: not adopted for now. Engaging is the safe direction, but the key is shared with the menu, so a press would mean two things, and an on-board key is out of reach on a moving vehicle. Revisit with the remote control.
- **Editing while armed parameters that are not `locked_when_armed`**: rejected. Live tuning over a Link has a tool and an operator at a distance; the local menu has neither.
- **A blacklist of parameters the menu must not show**: rejected. Every new parameter would appear in the menu by default, and forgetting to list one fails unsafe.
- **Every parameter's enumeration names in the firmware**: rejected for the reasons in ADR-0012; only menu parameters need them on the board.
- **A reboot item in the menu**: not adopted in the first version. The menu marks what needs a reboot, and a power cycle does the rest.
- **Resetting every parameter from the menu**: rejected. It would also reset parameters outside the whitelist, such as `MOT` and `COM`, which the menu must not change.
