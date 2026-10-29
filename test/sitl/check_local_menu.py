#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL local menu test (ADR-0018, ADR-0022, spec 0007): the menu on SITL's user key.

Presses SITL's simulated user key over the Link and checks, against
local_menu, the parameters read over the Link and the display image:

1. A long press opens the menu, and arming is refused with "local menu open".
2. SENS_BOARD_ROT edited to YAW_90 and saved reads YAW_90 over the Link,
   local_menu reports the save and a reboot required, and the value is still
   there after SITL restarts with the same parameter file.
3. ATT_KP stepped up past its maximum stays at the maximum; Cancel writes
   nothing; Default and Save restore the default.
4. 30 s without a key press close the menu and discard a staged edit; arming
   is then accepted in TEST.
5. Armed, a long press does not open the menu, and a short press only
   switches the page.
6. A value a Link sets while the menu shows it appears on the display.

    check_local_menu.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_attitude import free_udp_port  # noqa: E402
from display_text import screen_text  # noqa: E402
from vklink.monitor import LinkMonitor, UdpConnection, load_messages  # noqa: E402
from vklink.session import ParamReject, ParamType  # noqa: E402

HANDSHAKE_S = 4.0
REPLY_S = 3.0
SHORT_S = 0.1
LONG_S = 1.1        # past the 1 s long press, short of its first repeat
SETTLE_S = 0.4      # for local_menu and the display (5 Hz) to follow
IDLE_CLOSE_S = 30.0
BOOT_S = 3.0        # board verified and actuator power settled
# The menu parameters SITL compiles in, in table order, then Exit.
POSITIONS = {"ATT_ACC_REJ": 0, "ATT_KI": 1, "ATT_KP": 2, "EKF_MAG_DECL": 3, "EKF_MAG_TYPE": 4, "SENS_BOARD_ROT": 5}
# Actions in an edit, in the order a short press moves through them.
ACTIONS = ["+", "-", "default", "save", "cancel"]
MODE_TEST = 1


class Check:
    def __init__(self):
        self.failures: list[str] = []

    def expect(self, condition: bool, what: str) -> None:
        print(f"{'ok  ' if condition else 'FAIL'} {what}")
        if not condition:
            self.failures.append(what)


class Menu:
    """SITL with its Link, display image and simulated user key."""

    def __init__(self, args, param_file: Path, image: Path):
        self.port = free_udp_port()
        self.image = image
        self.process = subprocess.Popen(
            [args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{self.port}", "--param-file", str(param_file),
             "--display", str(image)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.5)
        self.connection = UdpConnection("127.0.0.1", self.port)
        self.monitor = LinkMonitor(self.connection, load_messages(args.msg_dir))
        if not self.monitor.handshake(HANDSHAKE_S):
            self.close()
            raise RuntimeError("no handshake with SITL")
        self.monitor.run(BOOT_S)

    def close(self) -> None:
        self.connection.close()
        self.process.terminate()
        self.process.wait(timeout=10)

    def press(self, duration_s: float) -> None:
        self.monitor.press_key(duration_s)
        self.monitor.run(SETTLE_S)

    def short(self, times: int = 1) -> None:
        for _ in range(times):
            self.press(SHORT_S)

    def long(self) -> None:
        self.press(LONG_S)

    def state(self):
        self.monitor.run(0.25)
        menus = self.monitor.latest("local_menu")
        return menus[0][1] if menus else None

    def screen(self) -> list[str]:
        self.monitor.run(SETTLE_S)
        return screen_text(self.image)

    def param(self, name: str):
        reply = self.monitor.get_param(name, REPLY_S)
        return None if reply is None or isinstance(reply, ParamReject) else reply.value

    def edit(self, name: str) -> None:
        """Opens the menu and edits `name`; the + action is selected."""
        self.long()
        self.short(POSITIONS[name])
        self.long()

    def action(self, current: str, wanted: str) -> None:
        """Moves from the action `current` to `wanted` and runs it."""
        self.short((ACTIONS.index(wanted) - ACTIONS.index(current)) % len(ACTIONS))
        self.long()

    def arm(self, arm: bool):
        return self.monitor.request_arming(arm, REPLY_S)


def run(args, check: Check, workdir: Path) -> None:
    param_file, image = workdir / "params.bin", workdir / "display.pbm"
    menu = Menu(args, param_file, image)
    try:
        default_kp = menu.param("ATT_KP")

        print("1. a long press opens the menu, and arming is refused")
        menu.long()
        state = menu.state()
        check.expect(state is not None and state.state == 1, "local_menu reports the list open")
        check.expect(menu.screen()[0].startswith("MENU"), "the display shows the menu")
        reply = menu.monitor.request_mode(MODE_TEST, REPLY_S)
        check.expect(reply is not None and not reply.refused, "TEST accepted while the menu is open")
        reply = menu.arm(True)
        check.expect(reply is not None and reply.refused and reply.reason_text == "local menu open",
                     "arming refused with 'local menu open'")

        print("2. SENS_BOARD_ROT to YAW_90 from the menu")
        menu.short(POSITIONS["SENS_BOARD_ROT"])
        menu.long()
        menu.action("+", "+")
        menu.long()                      # NONE, YAW_45, YAW_90
        check.expect(menu.screen()[3] == "NEW YAW_90", "the edit stages YAW_90")
        check.expect(menu.param("SENS_BOARD_ROT") == 0, "nothing written before Save")
        menu.action("+", "save")
        check.expect(menu.param("SENS_BOARD_ROT") == 2, "the Link reads YAW_90 after Save")
        state = menu.state()
        check.expect(state is not None and state.state == 1 and state.last_result == 0, "local_menu reports the save")
        check.expect(state is not None and state.reboot_required, "local_menu reports a reboot required")

        print("3. ATT_KP: + past the maximum, Cancel, Default and Save")
        menu.short((POSITIONS["ATT_KP"] - POSITIONS["SENS_BOARD_ROT"]) % (len(POSITIONS) + 1))
        menu.long()
        menu.press(20.0)                 # + held: one step, then a repeat every 0.2 s
        check.expect(menu.screen()[3] == "NEW 10.0", "+ held stops at the maximum, 10.0")
        menu.action("+", "cancel")
        check.expect(menu.param("ATT_KP") == default_kp, "Cancel writes nothing")
        menu.long()
        menu.action("+", "+")
        menu.action("+", "save")
        bumped = menu.param("ATT_KP")
        check.expect(bumped is not None and abs(bumped - (default_kp + 0.1)) < 1e-4, "+ and Save write one step up")
        menu.long()
        menu.action("+", "default")
        menu.action("default", "save")
        check.expect(menu.param("ATT_KP") == default_kp, "Default and Save restore the default")

        print("6. a value set over the Link appears on the display")
        menu.short((POSITIONS["ATT_KI"] - POSITIONS["ATT_KP"]) % (len(POSITIONS) + 1))
        reply = menu.monitor.set_param("ATT_KI", ParamType.FLOAT, 0.2, REPLY_S)
        check.expect(reply is not None and not isinstance(reply, ParamReject), "ATT_KI set over the Link")
        check.expect(menu.screen()[7] == "= 0.20", "the menu shows ATT_KI = 0.20")

        print("4. 30 s without a key press close the menu and discard the edit")
        menu.long()
        menu.action("+", "+")
        menu.monitor.run(IDLE_CLOSE_S + 1.0)
        state = menu.state()
        check.expect(state is not None and state.state == 0, "local_menu reports the menu closed")
        check.expect(abs(menu.param("ATT_KI") - 0.2) < 1e-4, "the staged edit was discarded")
        reply = menu.arm(True)
        check.expect(reply is not None and not reply.refused, "arming accepted once the menu is closed")

        print("5. armed, the key only switches pages")
        menu.long()
        state = menu.state()
        check.expect(state is not None and state.state == 0, "a long press does not open the menu while armed")
        menu.short()
        check.expect(menu.screen()[0].startswith("ATTITUDE"), "a short press switches to the attitude page")
        menu.arm(False)
    finally:
        menu.close()

    print("2. SENS_BOARD_ROT survives a restart")
    menu = Menu(args, param_file, image)
    try:
        check.expect(menu.param("SENS_BOARD_ROT") == 2, "SENS_BOARD_ROT is YAW_90 after the restart")
    finally:
        menu.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    args = parser.parse_args()
    check = Check()
    with tempfile.TemporaryDirectory() as tmp:
        try:
            run(args, check, Path(tmp))
        except RuntimeError as error:
            check.expect(False, str(error))
    if check.failures:
        print(f"FAIL: {len(check.failures)} check(s) failed")
        return 1
    print("PASS: the local menu follows the boundary of ADR-0022")
    return 0


if __name__ == "__main__":
    sys.exit(main())
