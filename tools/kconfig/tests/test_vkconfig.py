# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for the layered Kconfig solve, run through tools/kconfig/vkconfig.py."""

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
TOOL = REPO / "tools" / "kconfig" / "vkconfig.py"
KCONFIG = REPO / "Kconfig"


def solve(tmp_path: Path, fragments: list[str], board_symbols: list[str] = ()):
    board = tmp_path / "Kconfig.board"
    board.write_text("".join(f"config {s}\n\tbool\n\tdefault y\n" for s in board_symbols), encoding="utf-8")
    args = [sys.executable, str(TOOL), "generate", "--kconfig", str(KCONFIG), "--board-kconfig", str(board),
            "--output-dir", str(tmp_path / "out")]
    for number, text in enumerate(fragments):
        path = tmp_path / f"layer{number}.config"
        path.write_text(text, encoding="utf-8")
        args += ["--fragment", str(path)]
    return subprocess.run(args, capture_output=True, text=True, check=False)


def config(tmp_path: Path) -> str:
    return (tmp_path / "out" / ".config").read_text(encoding="utf-8")


def test_writes_config_header_and_cmake_values(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_SITL=y\n"])
    assert result.returncode == 0, result.stderr
    header = (tmp_path / "out" / "vk" / "config.h").read_text(encoding="utf-8")
    assert "#define CONFIG_PLATFORM_SITL 1" in header
    cmake = (tmp_path / "out" / "kconfig.cmake").read_text(encoding="utf-8")
    assert 'set(CONFIG_PLATFORM_SITL "y")' in cmake


def test_later_layers_override_earlier_ones(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_STM32F4=y\n", "# CONFIG_VEHICLE_MODEL_WHEELED is not set\n",
                              "CONFIG_VEHICLE_MODEL_ACKERMANN=y\n"])
    assert result.returncode == 0, result.stderr
    assert "# CONFIG_VEHICLE_MODEL_WHEELED is not set" in config(tmp_path)
    assert "CONFIG_VEHICLE_MODEL_ACKERMANN=y" in config(tmp_path)


def test_select_pulls_in_helper_libraries(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_SITL=y\nCONFIG_ESTIMATOR_MAHONY=y\n"])
    assert result.returncode == 0, result.stderr
    assert "CONFIG_MATH_QUATERNION=y" in config(tmp_path)


def test_board_symbols_satisfy_dependencies(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_STM32F4=y\n", "CONFIG_DRIVER_CAN=y\n"], ["BOARD_HAS_CAN"])
    assert result.returncode == 0, result.stderr
    assert "CONFIG_DRIVER_CAN=y" in config(tmp_path)


def test_unmet_dependency_names_the_fragment_line_and_dependency(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_SITL=y\n", "# overlay\nCONFIG_DRIVER_CAN=y\n"])
    assert result.returncode == 1
    assert f"{tmp_path / 'layer1.config'}:2: error: CONFIG_DRIVER_CAN=y did not take effect" in result.stderr
    assert "it depends on BOARD_HAS_CAN" in result.stderr


def test_unknown_option_is_an_error(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_SITL=y\nCONFIG_NO_SUCH_OPTION=y\n"])
    assert result.returncode == 1
    assert "CONFIG_NO_SUCH_OPTION is not a known option" in result.stderr


def test_selected_only_helpers_cannot_be_set_directly(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_SITL=y\n# CONFIG_ESTIMATOR_MAHONY is not set\n"
                              "# CONFIG_ESTIMATOR_EKF is not set\n"
                              "# CONFIG_LOCAL_DISPLAY is not set\nCONFIG_MATH_QUATERNION=y\n"])
    assert result.returncode == 1
    assert "CONFIG_MATH_QUATERNION=y did not take effect" in result.stderr


def test_icm20948_driver_needs_a_board_with_an_icm20948(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_STM32F4=y\n", "CONFIG_DRIVER_ICM20948=y\n"])
    assert result.returncode == 1
    assert "CONFIG_DRIVER_ICM20948=y did not take effect" in result.stderr


def test_icm20948_driver_is_on_for_a_board_with_an_icm20948_and_in_sitl(tmp_path):
    (tmp_path / "board").mkdir()
    (tmp_path / "sitl").mkdir()
    assert solve(tmp_path / "board", ["CONFIG_PLATFORM_STM32F4=y\n"], ["BOARD_HAS_ICM20948"]).returncode == 0
    assert "CONFIG_DRIVER_ICM20948=y" in config(tmp_path / "board")
    assert solve(tmp_path / "sitl", ["CONFIG_PLATFORM_SITL=y\n"]).returncode == 0
    assert "CONFIG_DRIVER_ICM20948=y" in config(tmp_path / "sitl")


def test_bringup_build_needs_an_icm20948_on_software_i2c(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_STM32F4=y\n", "CONFIG_BRINGUP_TESTS=y\n"], ["BOARD_HAS_ICM20948"])
    assert result.returncode == 1
    assert "CONFIG_BRINGUP_TESTS=y did not take effect" in result.stderr


def test_serial_link_needs_a_board_serial_port_for_a_link(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_STM32F4=y\n", "CONFIG_LINK_SERIAL=y\n"])
    assert result.returncode == 1
    assert "CONFIG_LINK_SERIAL=y did not take effect" in result.stderr
    assert "BOARD_HAS_LINK_PORT" in result.stderr


def test_serial_link_is_on_for_a_board_with_a_link_port(tmp_path):
    result = solve(tmp_path, ["CONFIG_PLATFORM_STM32F4=y\n"], ["BOARD_HAS_LINK_PORT"])
    assert result.returncode == 0, result.stderr
    assert "CONFIG_LINK_SERIAL=y" in config(tmp_path)
