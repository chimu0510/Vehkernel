# SPDX-License-Identifier: GPL-3.0-or-later
"""SITL's simulated gamepad (spec 0010): the sim_gamepad frame, the gamepad line and the sim-gamepad command."""

import struct
import types
from pathlib import Path

from test_control import connected_pair, exchange, run_cli
from test_monitor import FakeMcu
from vklink import McuControl
from vklink.enums import GamepadButton
from vklink.monitor import LinkMonitor, TopicStats, gamepad_line
from vklink.view import render


def test_sim_gamepad_frame_carries_sticks_triggers_and_buttons():
    control = McuControl()
    mcu, host = connected_pair(control)
    buttons = (1 << GamepadButton.A) | (1 << GamepadButton.START)
    assert exchange(mcu, host.make_sim_gamepad(True, (0.5, -1.0, 2.0, 0.0), (0.0, 1.0), buttons)) == []
    connected, *rest = struct.unpack("<B4h2BH", control.sim_gamepad)
    assert connected == 1
    assert rest[:4] == [16384, -32767, 32767, 0]  # clamped to full travel
    assert rest[4:6] == [0, 255]
    assert rest[6] == buttons
    exchange(mcu, host.make_sim_gamepad(False))
    assert struct.unpack("<B4h2BH", control.sim_gamepad) == (0, 0, 0, 0, 0, 0, 0, 0)


def test_gamepad_line_names_the_buttons_and_the_live_view_shows_it():
    gamepad = types.SimpleNamespace(timestamp=1, connected=True, sticks=(0.0, 0.5, -0.25, 0.0), triggers=(0.0, 1.0),
                                    buttons=(1 << GamepadButton.B) | (1 << GamepadButton.BACK))
    assert gamepad_line(gamepad) == ("gamepad connected=1 sticks=0.000,0.500,-0.250,0.000 triggers=0.000,1.000 "
                                     "buttons=BACK,B")
    connection = types.SimpleNamespace(send=lambda data: None, receive=lambda timeout: b"", close=lambda: None)
    monitor = LinkMonitor(connection, types.SimpleNamespace(TOPICS={}, BY_NAME={}))
    monitor.topics[("manual_control_input", 0)] = TopicStats("manual_control_input", 0)
    monitor.topics[("manual_control_input", 0)].add(gamepad)
    assert gamepad_line(gamepad) in monitor.summary()
    assert "gamepad        connected=1" in "\n".join(render(monitor, "udp", width=200))


def test_cli_sim_gamepad_sends_the_state_and_needs_valid_arguments(tmp_path: Path):
    control = McuControl()
    with FakeMcu(control=control) as mcu:
        sent = run_cli(mcu.port, tmp_path, "sim-gamepad", "--left", "0,0.5", "--buttons", "start,a")
        bad_stick = run_cli(mcu.port, tmp_path, "sim-gamepad", "--left", "0,2")
        bad_button = run_cli(mcu.port, tmp_path, "sim-gamepad", "--buttons", "Z")
    # The fake MCU has no manual_control_input to show back.
    assert sent.returncode == 1 and "no manual_control_input" in sent.stderr
    connected, lx, ly, *_, buttons = struct.unpack("<B4h2BH", control.sim_gamepad)
    assert (connected, lx, ly) == (1, 0, 16384)
    assert buttons == (1 << GamepadButton.START) | (1 << GamepadButton.A)
    assert bad_stick.returncode == 2 and "expected a stick position" in bad_stick.stderr
    assert bad_button.returncode == 2 and "unknown button 'Z'" in bad_button.stderr
