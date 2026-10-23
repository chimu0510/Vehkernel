"""finalize_image.py tests, through its command line, on small images that
hold an app descriptor as the firmware's linker places it."""

import base64
import json
import os
import struct
import subprocess
import sys
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
TOOL = HERE.parent / "finalize_image.py"
SIGNATURE = bytes([0x40, 0xA2, 0xE4, 0xF1, 0x64, 0x68, 0x91, 0x06])
BOARD_ID = 22083


def descriptor(board_id: int = BOARD_ID, major: int = 0, minor: int = 1) -> bytes:
    """As the compiler emits it: signature, 16 zero bytes, version, board ID, reserved."""
    return SIGNATURE + bytes(16) + struct.pack("<BBH", major, minor, board_id) + bytes(8)


def ap_crc32(data: bytes) -> int:
    """ArduPilot's crc32_small from 0, by way of zlib's CRC-32, which inverts before and after."""
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


def ap_check(image: bytes, board_id: int) -> str:
    """ArduPilot's check_good_firmware_unsigned (libraries/AP_CheckFirmware), in Python."""
    offset = image.find(SIGNATURE)
    if offset < 0:
        return "FAIL_REASON_NO_APP_SIG"
    crc1, crc2, size, _git, _major, _minor, image_board = struct.unpack_from("<IIIIBBH", image, offset + 8)
    if size > len(image):
        return "FAIL_REASON_BAD_LENGTH_APP"
    if image_board != board_id:
        return "FAIL_REASON_BAD_BOARD_ID"
    len1 = offset + 8
    if crc1 != ap_crc32(image[:len1]) or crc2 != ap_crc32(image[offset + 24:size]):
        return "FAIL_REASON_BAD_CRC"
    return "CHECK_FW_OK"


def make_files(tmp_path: Path, board_id_in_image: int = BOARD_ID, with_descriptor: bool = True):
    vectors = bytes(range(256)) * 2
    body = bytes((i * 7) & 0xFF for i in range(3000))
    image = vectors + (descriptor(board_id_in_image) if with_descriptor else b"") + body
    (tmp_path / "vehkernel.bin").write_bytes(image)
    # An ELF holds the same bytes among its headers and other sections.
    (tmp_path / "vehkernel.elf").write_bytes(b"\x7fELF" + bytes(100) + image + b"debug info")
    board = {"name": "c50c-v1_1", "board_id": BOARD_ID,
             "flash": [{"region": "BOOT", "start": 0x08000000, "size": 16384},
                       {"region": "FIRMWARE", "start": 0x08010000, "size": 458752}]}
    (tmp_path / "board.json").write_text(json.dumps(board))


def run(tmp_path: Path, version: str = "0.1"):
    return subprocess.run(
        [sys.executable, str(TOOL), "--elf", str(tmp_path / "vehkernel.elf"), "--bin", str(tmp_path / "vehkernel.bin"),
         "--board-json", str(tmp_path / "board.json"), "--apj", str(tmp_path / "vehkernel.apj"),
         "--version", version, "--source-dir", str(tmp_path)],
        capture_output=True, text=True, check=False, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})


def test_the_filled_image_passes_ardupilots_check(tmp_path):
    make_files(tmp_path)
    result = run(tmp_path)
    assert result.returncode == 0, result.stderr
    image = (tmp_path / "vehkernel.bin").read_bytes()
    assert ap_check(image, BOARD_ID) == "CHECK_FW_OK"
    assert ap_check(image, BOARD_ID + 1) == "FAIL_REASON_BAD_BOARD_ID"
    _crc1, _crc2, size, git = struct.unpack_from("<IIII", image, image.find(SIGNATURE) + 8)
    assert size == len(image)
    assert git == 0  # outside a git checkout


def test_a_changed_byte_fails_the_check(tmp_path):
    make_files(tmp_path)
    assert run(tmp_path).returncode == 0
    image = bytearray((tmp_path / "vehkernel.bin").read_bytes())
    image[-1] ^= 0xFF
    assert ap_check(bytes(image), BOARD_ID) == "FAIL_REASON_BAD_CRC"


def test_the_elf_gets_the_same_descriptor(tmp_path):
    make_files(tmp_path)
    assert run(tmp_path).returncode == 0
    image = (tmp_path / "vehkernel.bin").read_bytes()
    elf = (tmp_path / "vehkernel.elf").read_bytes()
    filled = image[image.find(SIGNATURE):image.find(SIGNATURE) + 36]
    assert filled in elf
    assert elf.endswith(b"debug info")


def test_the_apj_decodes_to_the_image(tmp_path):
    make_files(tmp_path)
    assert run(tmp_path).returncode == 0
    apj = json.loads((tmp_path / "vehkernel.apj").read_text())
    image = (tmp_path / "vehkernel.bin").read_bytes()
    assert zlib.decompress(base64.b64decode(apj["image"])) == image
    assert apj["board_id"] == BOARD_ID
    assert apj["magic"] == "APJFWv1"
    assert apj["image_size"] == len(image)
    assert apj["image_maxsize"] == 458752
    assert apj["version"] == "0.1"
    assert apj["summary"] == "c50c-v1_1"
    assert apj["board_revision"] == 0
    assert apj["extf_image_size"] == 0
    assert apj["git_identity"] == "unknown"


def test_finalizing_twice_gives_the_same_files(tmp_path):
    make_files(tmp_path)
    assert run(tmp_path).returncode == 0
    first = {name: (tmp_path / name).read_bytes() for name in ("vehkernel.bin", "vehkernel.elf", "vehkernel.apj")}
    assert run(tmp_path).returncode == 0
    assert first == {name: (tmp_path / name).read_bytes() for name in first}


def test_an_image_for_another_board_is_refused_and_nothing_is_written(tmp_path):
    make_files(tmp_path, board_id_in_image=56)
    before = (tmp_path / "vehkernel.bin").read_bytes()
    result = run(tmp_path)
    assert result.returncode == 1
    assert "board ID 56 is not the board's 22083" in result.stderr
    assert (tmp_path / "vehkernel.bin").read_bytes() == before
    assert not (tmp_path / "vehkernel.apj").exists()


def test_a_wrong_version_is_refused(tmp_path):
    make_files(tmp_path)
    result = run(tmp_path, version="0.2")
    assert result.returncode == 1
    assert "version 0.1 is not 0.2" in result.stderr


def test_an_image_without_a_descriptor_is_refused(tmp_path):
    make_files(tmp_path, with_descriptor=False)
    result = run(tmp_path)
    assert result.returncode == 1
    assert "no app descriptor in the firmware image" in result.stderr
