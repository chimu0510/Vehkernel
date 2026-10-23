"""Serial flasher tests, through its command line, with a fake stm32flash that
records its calls and a build directory holding only what the tool reads."""

import json
import os
import stat
import subprocess
import sys
from pathlib import Path

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


def test_update_writes_the_firmware_from_its_region_only(tmp_path):
    result, calls = run(tmp_path, "update")
    assert result.returncode == 0, result.stderr
    assert len(calls) == 1
    assert calls[0].startswith(f"-b 115200 -i {SEQUENCE} -w ")
    assert "vehkernel.bin -v -S 0x08010000 /dev/fake" in calls[0]


def test_update_refuses_an_image_larger_than_its_region(tmp_path):
    result, calls = run(tmp_path, "update", build=make_build(tmp_path, firmware_size=458753))
    assert result.returncode == 2
    assert "FIRMWARE region holds 458752" in result.stderr
    assert calls == []


def test_new_board_backs_up_the_whole_chip_before_erasing_it(tmp_path):
    result, calls = run(tmp_path, "new-board", "--backup", "vendor.bin")
    assert result.returncode == 0, result.stderr
    assert [call.split(" /dev/fake")[0].split(f"{SEQUENCE} ")[1] for call in calls] == [
        "-S 0x08000000:524288 -r vendor.bin",
        "-o",
        f"-w {tmp_path / 'build' / 'app' / 'vehkernel.hex'} -v",
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
    result, calls = run(tmp_path, "update", build=tmp_path / "nowhere")
    assert result.returncode == 2
    assert "a firmware build?" in result.stderr
    assert calls == []


def test_the_sequence_comes_from_the_board_definition(tmp_path):
    result, calls = run(tmp_path, "update")
    assert result.returncode == 0, result.stderr
    assert "download port SERIAL2, DTR_RTS circuit" in result.stdout
    assert f"-i {SEQUENCE} " in calls[0]


def test_a_board_without_a_download_circuit_is_refused(tmp_path):
    result, calls = run(tmp_path, "update", build=make_build(tmp_path, download=None))
    assert result.returncode == 2
    assert "declares no serial port with a download circuit" in result.stderr
    assert calls == []
