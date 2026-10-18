"""The C++ and Python encoders generated from the schema agree byte for byte
(ADR-0006). wire_vectors prints every message encoded by C++, filled with the
values of wire_values.hpp; this test computes the same values in Python,
encodes them with the generated Python module and compares, then decodes the
C++ bytes in Python and compares the fields."""

import importlib.util
import os
import struct
import subprocess
from pathlib import Path

import pytest

VECTORS = os.environ.get("VK_WIRE_VECTORS")
PYTHON_DIR = os.environ.get("VK_MSG_PYTHON_DIR")

pytestmark = pytest.mark.skipif(not VECTORS or not PYTHON_DIR,
                                reason="run through ctest, which sets VK_WIRE_VECTORS and VK_MSG_PYTHON_DIR")


def load_vkmsg():
    spec = importlib.util.spec_from_file_location("vkmsg", Path(PYTHON_DIR) / "vkmsg.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wire_value(code: str, k: int, j: int):
    """Mirror of vk::test::wire_value() in wire_values.hpp."""
    negative = (k + j) % 2 == 1
    if code == "?":
        return negative
    if code in "fd":
        magnitude = k + 0.25 + j * 0.5
        return -magnitude if negative else magnitude
    size = struct.calcsize(code)
    m = (k * 37 + j * 11 + 3) % 100
    m = {1: m, 2: m * 300 + m, 4: m * 100_000 + m, 8: m * 1_000_000_000_000 + m}[size]
    return -m if code.islower() and negative else m


def expected_message(cls):
    codes = [c for c in cls._STRUCT.format if not c.isdigit() and c != "<"]
    counts = []
    fmt = cls._STRUCT.format.lstrip("<")
    number = ""
    for c in fmt:
        if c.isdigit():
            number += c
        else:
            counts.append(int(number) if number else 0)
            number = ""
    values = {}
    for k, (name, code, count) in enumerate(zip(cls.FIELDS, codes, counts)):
        if count:
            values[name] = tuple(wire_value(code, k, j) for j in range(count))
        else:
            values[name] = wire_value(code, k, 0)
    return cls(**values)


def cpp_vectors() -> dict[str, bytes]:
    output = subprocess.run([VECTORS], capture_output=True, text=True, check=True).stdout
    return {name: bytes.fromhex(data) for name, data in (line.split() for line in output.splitlines())}


def test_every_topic_is_covered():
    vkmsg = load_vkmsg()
    assert set(cpp_vectors()) == set(vkmsg.BY_NAME)


@pytest.mark.parametrize("direction", ["python-encodes", "python-decodes"])
def test_cpp_and_python_agree(direction: str):
    vkmsg = load_vkmsg()
    for name, data in cpp_vectors().items():
        cls = vkmsg.BY_NAME[name]
        expected = expected_message(cls)
        assert len(data) == cls.ENCODED_SIZE, name
        if direction == "python-encodes":
            assert expected.encode() == data, name
        else:
            assert cls.decode(data) == expected, name
