"""Mode, arming and emergency stop requests (spec 0006): frames, the monitor and the command line."""

import subprocess
import sys
from pathlib import Path

from test_monitor import FakeMcu
from test_params import MONITOR, VKMSG
from vklink import BringupReply, ControlReply, FrameReader, FrameType, HostSession, McuControl, McuSession


def connected_pair(control: McuControl):
    mcu = McuSession(control=control)
    host = HostSession()
    exchange(mcu, host.make_hello())
    return mcu, host


def exchange(mcu: McuSession, wire: bytes) -> list[ControlReply]:
    replies = []
    for frame in FrameReader().feed(wire):
        for reply in mcu.handle(frame, now_us=1000):
            for answer in FrameReader().feed(reply):
                parsed = ControlReply.parse(answer)
                if parsed is not None:
                    replies.append(parsed)
    return replies


def test_mode_request_frames():
    control = McuControl()
    mcu, host = connected_pair(control)
    assert exchange(mcu, host.make_mode_request(1)) == [ControlReply(FrameType.MODE_REPLY, 1)]
    control.mode_result = 1
    [reject] = exchange(mcu, host.make_mode_request(0))
    assert reject.refused and reject.reason_text == "armed" and reject.mode == 1
    [unknown] = exchange(mcu, host.make_mode_request(9))
    assert unknown.reason_text == "unknown mode"


def test_arming_reply_carries_the_skipped_checks_and_a_reject_its_reason():
    control = McuControl(skipped=1)
    mcu, host = connected_pair(control)
    assert exchange(mcu, host.make_arming_request(True)) == [ControlReply(FrameType.ARMING_REPLY, 1, skipped=1)]
    control.arming_result = 7
    [reject] = exchange(mcu, host.make_arming_request(True))
    assert reject.refused and reject.reason_text == "actuator power off"
    # Disarming is never refused.
    assert exchange(mcu, host.make_arming_request(False))[0].value == 0


def test_estop_and_sim_power_frames():
    control = McuControl()
    mcu, host = connected_pair(control)
    assert exchange(mcu, host.make_estop_request(True)) == [ControlReply(FrameType.ESTOP_REPLY, 1)]
    assert control.emergency_stop
    assert exchange(mcu, host.make_sim_actuator_power(False)) == []
    assert not control.sim_actuator_power


def run_cli(port: int, tmp_path: Path, *command: str) -> subprocess.CompletedProcess:
    (tmp_path / "vkmsg.py").write_text(VKMSG, encoding="utf-8")
    return subprocess.run(
        [sys.executable, str(MONITOR), "--udp", f"127.0.0.1:{port}", "--msg-dir", str(tmp_path),
         "--handshake-timeout", "2", *command],
        capture_output=True, text=True, timeout=30, check=False)


def test_cli_changes_mode_arms_and_engages_the_emergency_stop(tmp_path: Path):
    control = McuControl()
    with FakeMcu(control=control) as mcu:
        mode = run_cli(mcu.port, tmp_path, "mode", "test")
        armed = run_cli(mcu.port, tmp_path, "arm")
        estop = run_cli(mcu.port, tmp_path, "estop", "on")
        disarmed = run_cli(mcu.port, tmp_path, "disarm")
    assert (mode.returncode, mode.stdout) == (0, "mode=TEST\n"), mode.stderr
    assert (armed.returncode, armed.stdout, armed.stderr) == (0, "armed=1 skipped_checks=none\n", "")
    assert (estop.returncode, estop.stdout) == (0, "emergency_stop=1\n")
    assert (disarmed.returncode, disarmed.stdout) == (0, "armed=0 skipped_checks=none\n")


def test_cli_warns_on_skipped_checks_and_fails_with_the_reason(tmp_path: Path):
    control = McuControl(skipped=1)
    with FakeMcu(control=control) as mcu:
        armed = run_cli(mcu.port, tmp_path, "arm")
        control.arming_result = 3
        control.armed = False
        refused = run_cli(mcu.port, tmp_path, "arm")
        control.mode_result = 1
        mode = run_cli(mcu.port, tmp_path, "mode", "standby")
    assert armed.returncode == 0
    assert "warning: arming checks skipped: ACT_POWER" in armed.stderr
    assert refused.returncode == 1
    assert "FAIL: arm refused: wrong mode" in refused.stderr
    assert mode.returncode == 1
    assert "FAIL: mode refused: armed" in mode.stderr


def test_cli_fails_when_the_mcu_does_not_answer(tmp_path: Path):
    with FakeMcu() as mcu:
        result = run_cli(mcu.port, tmp_path, "estop", "on")
    assert result.returncode == 1
    assert "FAIL: no reply from the MCU to estop" in result.stderr


def test_bringup_frames_and_cli(tmp_path: Path):
    control = McuControl(bringup_outputs=((0, 1, 1), (2, 2, -1)))
    mcu, host = connected_pair(control)
    [result] = [BringupReply.parse(f) for w in mcu.handle(next(iter(FrameReader().feed(host.make_bringup_run(1)))),
                                                         now_us=1000) for f in FrameReader().feed(w)]
    assert result.outputs[1].result == 2 and result.outputs[1].sign == -1 and not result.aborted

    with FakeMcu(control=control) as mcu:
        failed = run_cli(mcu.port, tmp_path, "bringup", "motors")
        control.bringup_outputs = ((0, 1, 1), (0, 2, 1))
        passed = run_cli(mcu.port, tmp_path, "bringup", "motors")
        control.bringup_result = 4
        refused = run_cli(mcu.port, tmp_path, "bringup", "motors")
    assert failed.returncode == 1
    assert "bringup output=2 result=reversed encoder=2 sign=-1  fix: set MOT2_REV=REVERSED" in failed.stdout
    assert passed.returncode == 0, passed.stderr
    assert passed.stdout.splitlines()[0] == "bringup output=1 result=pass encoder=1 sign=+1"
    assert refused.returncode == 1 and "FAIL: bringup refused: not armed" in refused.stderr
