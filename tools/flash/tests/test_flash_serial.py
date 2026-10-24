# SPDX-License-Identifier: GPL-3.0-or-later
"""Serial flasher tests, through its command line, with fakes for stm32flash,
uploader.py and the Link monitor that record their calls, and a build
directory holding only what the tool reads. The update tests run the tool in
this process, so the download circuit's restart is recorded rather than
driven on a port."""

import importlib.util
import json
import os
import stat
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
TOOL = HERE.parent / "flash_serial.py"
SEQUENCE = "-dtr,rts,,dtr,,-rts:-dtr,rts,,-rts"


def make_build(tmp_path: Path, firmware_size: int = 1000, download: str | None = "DTR_RTS") -> Path:
    build = tmp_path / "build"
    (build / "generated" / "board").mkdir(parents=True)
    (build / "app").mkdir()
    flash = [
        {"region": "BOOT", "start": 0x08000000, "size": 16384},
        {"region": "PARAMS", "start": 0x08004000, "size": 32768},
        {"region": "EVENTLOG", "start": 0x0800C000, "size": 16384},
        {"region": "FIRMWARE", "start": 0x08010000, "size": 458752},
    ]
    serial = [{"port": "SERIAL1", "download": None}, {"port": "SERIAL2", "download": download}]
    (build / "generated" / "board" / "board.json").write_text(
        json.dumps({"name": "c50c-v1_1", "flash": flash, "serial": serial}))
    (build / "app" / "vehkernel.bin").write_bytes(b"\0" * firmware_size)
    (build / "app" / "vehkernel.hex").write_text(":00000001FF\n")
    (build / "app" / "vehkernel_with_bl.hex").write_text(":00000001FF\n")
    (build / "app" / "vehkernel.apj").write_text("{}\n")
    (build / "generated" / "schema" / "python").mkdir(parents=True)
    return build


def fake_stm32flash(tmp_path: Path, fail_on: str = "", short_read: bool = False) -> Path:
    """Logs each call's arguments; -r writes a file of the size -S asks for."""
    script = tmp_path / "stm32flash"
    script.write_text(f"""#!{sys.executable}
import sys
args = sys.argv[1:]
with open({str(tmp_path / 'calls.log')!r}, 'a') as log:
    log.write(' '.join(args) + '\\n')
if {fail_on!r} and {fail_on!r} in args:
    sys.exit(1)
if '-r' in args:
    size = int(args[args.index('-S') + 1].split(':')[1])
    open(args[args.index('-r') + 1], 'wb').write(b'\\xff' * (size // 2 if {short_read!r} else size))
""")
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    return script


def run(tmp_path: Path, *args: str, fail_on: str = "", short_read: bool = False, build: Path | None = None):
    build = build or make_build(tmp_path)
    tool = fake_stm32flash(tmp_path, fail_on, short_read)
    result = subprocess.run(
        [sys.executable, str(TOOL), "--build", str(build), "--stm32flash", str(tool), "--port", "/dev/fake", *args],
        capture_output=True, text=True, check=False, cwd=tmp_path,
        env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    log = tmp_path / "calls.log"
    calls = log.read_text().splitlines() if log.exists() else []
    return result, calls


def test_new_board_backs_up_the_whole_chip_before_erasing_it(tmp_path):
    result, calls = run(tmp_path, "new-board", "--backup", "vendor.bin")
    assert result.returncode == 0, result.stderr
    assert [call.split(" /dev/fake")[0].split(f"{SEQUENCE} ")[1] for call in calls] == [
        "-S 0x08000000:524288 -r vendor.bin",
        "-o",
        f"-w {tmp_path / 'build' / 'app' / 'vehkernel_with_bl.hex'} -v",
    ]
    assert (tmp_path / "vendor.bin").stat().st_size == 524288


def test_new_board_erases_nothing_when_the_backup_fails(tmp_path):
    result, calls = run(tmp_path, "new-board", "--backup", "vendor.bin", fail_on="-r")
    assert result.returncode == 1
    assert len(calls) == 1 and "-r vendor.bin" in calls[0]


def test_new_board_erases_nothing_when_the_backup_is_short(tmp_path):
    result, calls = run(tmp_path, "new-board", "--backup", "vendor.bin", short_read=True)
    assert result.returncode == 1
    assert "incomplete; nothing was erased" in result.stderr
    assert len(calls) == 1


def test_new_board_never_overwrites_an_existing_backup(tmp_path):
    (tmp_path / "vendor.bin").write_bytes(b"vendor")
    result, calls = run(tmp_path, "new-board", "--backup", "vendor.bin")
    assert result.returncode == 2
    assert calls == []
    assert (tmp_path / "vendor.bin").read_bytes() == b"vendor"


def test_new_board_without_a_backup_only_erases_and_writes(tmp_path):
    result, calls = run(tmp_path, "new-board", "--no-backup")
    assert result.returncode == 0, result.stderr
    assert [" -o " in call or call.endswith(" -o /dev/fake") for call in calls] == [True, False]


def test_a_missing_build_is_a_usage_error(tmp_path):
    result, calls = run(tmp_path, "backup", "chip.bin", build=tmp_path / "nowhere")
    assert result.returncode == 2
    assert "a firmware build?" in result.stderr
    assert calls == []


def test_a_board_without_a_download_circuit_is_refused(tmp_path):
    result, calls = run(tmp_path, "new-board", "--no-backup", build=make_build(tmp_path, download=None))
    assert result.returncode == 2
    assert "declares no serial port with a download circuit" in result.stderr
    assert calls == []


def test_new_board_takes_the_sequence_from_the_board_definition(tmp_path):
    result, calls = run(tmp_path, "new-board", "--no-backup")
    assert result.returncode == 0, result.stderr
    assert "the board's download port is SERIAL2, with a DTR_RTS circuit" in result.stdout
    assert all(call.startswith(f"-b 115200 -i {SEQUENCE} ") for call in calls)


# update: the Link monitor and uploader.py are fakes that log their arguments.

def fake_link(tmp_path: Path, outcome: str) -> Path:
    """accept: the firmware accepts; silent: no handshake; refuse: refused while armed."""
    script = tmp_path / "link_monitor.py"
    script.write_text(f"""import sys
with open({str(tmp_path / 'link.log')!r}, 'a') as log:
    log.write(' '.join(sys.argv[1:]) + '\\n')
if {outcome!r} == 'accept':
    print('reboot=bootloader')
elif {outcome!r} == 'refuse':
    sys.exit(print('FAIL: reboot refused: armed', file=sys.stderr) or 1)
else:
    sys.exit(print('FAIL: no handshake with the MCU within 5 s', file=sys.stderr) or 1)
""")
    return script


def fake_uploader(tmp_path: Path, outcome: str) -> Path:
    """Waits for the bootloader as uploader.py does. flash: finds it and writes
    the image; after-restart: finds it only once the board was restarted;
    refuse: is refused for another board and keeps waiting; fail: exits 1."""
    script = tmp_path / "uploader.py"
    restarted = tmp_path / "restarted"
    script.write_text(f"""import os, sys, time
with open({str(tmp_path / 'uploader.log')!r}, 'a') as log:
    log.write(' '.join(sys.argv[1:]) + '\\n')
print('Loaded firmware for 5643,0, size: 1000 bytes, waiting for the bootloader...')
outcome = {outcome!r}
if outcome == 'after-restart':
    while not os.path.exists({str(restarted)!r}):
        time.sleep(0.01)
if outcome == 'refuse':
    print('WARNING: Firmware not suitable for this board (board_type=56 board_id=22083)')
    time.sleep(60)
if outcome == 'fail':
    sys.exit(1)
print('Found board 5643,0 bootloader rev 5 on /dev/fake')
print('Erase  : [====================] 100.0%')
print('Program: [====================] 100.0%')
print('Verify : [====================] 100.0%')
""")
    return script


@pytest.fixture()
def flasher(monkeypatch):
    """The tool as a module, with the download circuit's restart recorded."""
    spec = importlib.util.spec_from_file_location("flash_serial", TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    restarts = []

    def reset(port, circuit):
        restarts.append((port, circuit))
        (Path(os.environ["VK_TEST_DIR"]) / "restarted").touch()

    monkeypatch.setattr(module, "reset", reset)
    module.restarts = restarts
    return module


def update(flasher, tmp_path: Path, monkeypatch, link: str, uploader: str):
    monkeypatch.setenv("VK_TEST_DIR", str(tmp_path))
    build = make_build(tmp_path)
    code = flasher.main(["--build", str(build), "--port", "/dev/fake",
                         "--link-monitor", str(fake_link(tmp_path, link)),
                         "--uploader", str(fake_uploader(tmp_path, uploader)), "update"])
    logs = {}
    for name in ("link", "uploader"):
        log = tmp_path / f"{name}.log"
        logs[name] = log.read_text().splitlines() if log.exists() else []
    return code, logs, build


def test_update_asks_over_the_link_then_runs_uploader_with_the_apj(flasher, tmp_path, monkeypatch, capsys):
    code, logs, build = update(flasher, tmp_path, monkeypatch, link="accept", uploader="flash")
    assert code == 0, capsys.readouterr().err
    assert logs["link"] == [f"--serial /dev/fake --baud 921600 --msg-dir {build / 'generated' / 'schema' / 'python'} "
                            "reboot --bootloader"]
    assert logs["uploader"] == [f"--port /dev/fake --no-extf {build / 'app' / 'vehkernel.apj'}"]
    assert flasher.restarts == []
    assert "parameters are kept" in capsys.readouterr().out


def test_update_falls_back_to_the_download_circuit_when_no_link_answers(flasher, tmp_path, monkeypatch, capsys):
    code, logs, _build = update(flasher, tmp_path, monkeypatch, link="silent", uploader="after-restart")
    assert code == 0, capsys.readouterr().err
    assert len(logs["link"]) == 1
    # The restart comes once uploader.py waits, so the boot window catches it.
    assert flasher.restarts == [("/dev/fake", "DTR_RTS")]
    assert len(logs["uploader"]) == 1 and "vehkernel.apj" in logs["uploader"][0]
    assert "No Link answered" in capsys.readouterr().out


def test_update_never_restarts_a_board_that_refuses(flasher, tmp_path, monkeypatch, capsys):
    code, logs, _build = update(flasher, tmp_path, monkeypatch, link="refuse", uploader="flash")
    assert code == 1
    assert "disarm it first" in capsys.readouterr().err
    assert flasher.restarts == []
    assert logs["uploader"] == []


def test_update_stops_uploader_when_the_bootloader_refuses_the_image(flasher, tmp_path, monkeypatch, capsys):
    code, _logs, _build = update(flasher, tmp_path, monkeypatch, link="accept", uploader="refuse")
    assert code == 1
    assert "it is for another board" in capsys.readouterr().err


def test_update_reports_a_failed_upload(flasher, tmp_path, monkeypatch, capsys):
    code, _logs, _build = update(flasher, tmp_path, monkeypatch, link="accept", uploader="fail")
    assert code == 1
    assert "uploader.py failed with exit code 1" in capsys.readouterr().err


def test_update_gives_up_when_no_bootloader_answers(flasher, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(flasher, "UPLOAD_TIMEOUT_S", 0.5)
    monkeypatch.setattr(flasher, "reset", lambda port, circuit: None)  # the board never comes back
    code, _logs, _build = update(flasher, tmp_path, monkeypatch, link="silent", uploader="after-restart")
    assert code == 1
    assert "did not finish within" in capsys.readouterr().err


def test_update_needs_the_apj(flasher, tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("VK_TEST_DIR", str(tmp_path))
    build = make_build(tmp_path)
    (build / "app" / "vehkernel.apj").unlink()
    code = flasher.main(["--build", str(build), "--port", "/dev/fake", "update"])
    assert code == 2
    assert "vehkernel.apj is missing" in capsys.readouterr().err
