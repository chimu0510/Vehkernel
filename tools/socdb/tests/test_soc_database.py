# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for the SoC database and its offline converter, run through the command line."""

import json
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
CONVERTER = REPO / "tools" / "socdb" / "convert_stm32_data.py"
F407 = REPO / "soc" / "st" / "stm32f4" / "stm32f407vet6"


def run_converter(*args):
    return subprocess.run([sys.executable, str(CONVERTER), *map(str, args)],
                          capture_output=True, text=True, check=False)


@pytest.fixture(scope="module")
def database():
    return json.loads((F407 / "soc.json").read_text(encoding="utf-8"))


def test_committed_database_matches_a_fresh_conversion():
    result = run_converter(F407, "--check")
    assert result.returncode == 0, result.stderr


def test_check_fails_when_the_database_is_out_of_date(tmp_path):
    copy = tmp_path / "stm32f407vet6"
    shutil.copytree(F407, copy)
    (copy / "soc.json").write_text("{}\n", encoding="utf-8")
    result = run_converter(copy, "--check")
    assert result.returncode == 1
    assert "out of date" in result.stderr


def test_conversion_writes_the_same_bytes_as_committed(tmp_path):
    copy = tmp_path / "stm32f407vet6"
    shutil.copytree(F407, copy)
    (copy / "soc.json").unlink()
    assert run_converter(copy).returncode == 0
    assert (copy / "soc.json").read_bytes() == (F407 / "soc.json").read_bytes()


def test_unknown_package_is_reported(tmp_path):
    copy = tmp_path / "stm32f407vet6"
    shutil.copytree(F407, copy)
    manifest_path = copy / "source" / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["package"] = "STM32F407ZGTx"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    result = run_converter(copy)
    assert result.returncode == 1
    assert "package 'STM32F407ZGTx' not found" in result.stderr


def test_records_identity_and_source(database):
    assert database["format"] == 1
    assert database["soc"] == "STM32F407VET6"
    assert database["family"] == "STM32F4"
    assert database["package"] == "LQFP100"
    source = database["source"]
    assert source["project"] == "embassy-rs/stm32-data-generated"
    assert len(source["commit"]) == 40
    assert source["path"] == "data/chips/STM32F407VE.json"


def test_lists_the_82_gpios_of_the_lqfp100_package(database):
    pins = database["pins"]
    assert len(pins) == 82
    assert pins[0] == "PA0" and pins[-1] == "PH1"
    assert "PF0" not in pins
    assert set(database["af"]) == set(pins)


@pytest.mark.parametrize("pin, function, af", [
    ("PA9", "USART1_TX", 7),
    ("PA10", "USART1_RX", 7),
    ("PB7", "TIM4_CH2", 2),
    ("PB8", "I2C1_SCL", 4),
    ("PB9", "I2C1_SDA", 4),
    ("PA8", "TIM1_CH1", 1),
    ("PC6", "USART6_TX", 8),
])
def test_alternate_functions_match_the_datasheet(database, pin, function, af):
    assert {"function": function, "af": af} in database["af"][pin]


def test_analog_functions_are_kept_apart_from_alternate_functions(database):
    assert "ADC1_IN0" in database["analog"]["PA0"]
    assert all(entry["function"] != "ADC1_IN0" for entry in database["af"]["PA0"])


@pytest.mark.parametrize("request_name, controller, stream, channel", [
    ("USART1_TX", "DMA2", 7, 4),
    ("USART1_RX", "DMA2", 2, 4),
    ("USART1_RX", "DMA2", 5, 4),
    ("TIM4_CH2", "DMA1", 3, 2),
])
def test_dma_requests_match_the_reference_manual(database, request_name, controller, stream, channel):
    assert {"controller": controller, "stream": stream, "channel": channel} in database["dma"][request_name]


def test_flash_sector_table(database):
    flash = database["flash"]
    assert flash["base"] == 0x08000000
    assert flash["size"] == 512 * 1024
    assert flash["write_size"] == 4
    assert [s["size"] // 1024 for s in flash["sectors"]] == [16, 16, 16, 16, 64, 128, 128, 128]
    assert [s["index"] for s in flash["sectors"]] == list(range(8))
    assert flash["sectors"][4]["address"] == 0x08010000


def test_ram_regions_include_non_executable_ccm(database):
    ram = {region["name"]: region for region in database["ram"]}
    assert ram["CCMRAM"] == {"name": "CCMRAM", "address": 0x10000000, "size": 64 * 1024, "executable": False}
    assert ram["SRAM"]["size"] + ram["SRAM2"]["size"] == 128 * 1024


def test_peripheral_instances_carry_address_kind_and_bus_clock(database):
    assert database["peripherals"]["USART1"] == {"address": 0x40011000, "kind": "usart", "bus_clock": "PCLK2"}
    assert database["peripherals"]["TIM4"]["address"] == 0x40000800
    assert database["peripherals"]["I2C1"]["bus_clock"] == "PCLK1"
