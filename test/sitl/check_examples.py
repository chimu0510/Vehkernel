#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL test for board examples (spec 0013 Seam C).

Runs a SITL example executable and verifies its report and simulated output.
For 03-oled:
- Verifies the counter rate in the report (10 Hz within tolerance).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 the title, line 2 the last reported counter, every other line blank.
For 04-work-queues:
- Verifies fast (10 Hz) and slow (2 Hz) counter rates over the span between first and last report.
- Verifies no gaps in either counter's sequence.
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 the title, line 2 the last fast counter, line 4 the last slow counter.
For 05-bus:
- Verifies published counter rate (10 Hz) over the span between first and last report.
- Verifies no gaps in published counter sequence.
- Verifies that every received value matches the published ones, each once and in order (allowing 1-2 unreceived at stop).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 the title, line 2 the last received count, line 4 the total received count.
For 06-imu:
- Verifies reported sample rate against 200 Hz within tolerance.
- Verifies that the reported down axis is +Z on the level ground vehicle at rest.
- Verifies reported specific force components (ax ~ 0, ay ~ 0, az ~ -9.81 m/s^2).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 title, lines 2-4 specific force, line 5 sample rate, line 6 down axis.
For 07-magnetometer:
- Verifies reported sample rate against 100 Hz within tolerance.
- Verifies that reported heading tracks simulated vehicle true yaw within tolerance (3.0 deg) over a rotating motion.
- Verifies that heading sweeps through at least most of a turn (at least 270 deg).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 title, line 1 blank, lines 2-4 magnetic field components (BX, BY, BZ), line 5 norm, line 6 sample rate, line 7 heading.
For 08-serial-link:
- Connects over UDP to the bound port.
- Verifies handshake succeeds.
- Verifies reported imu_sample rate matches 200 Hz within tolerance (20 %).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 title, line 2 STATE: CONNECTED, line 3 TX FRAMES, line 4 RX FRAMES, line 5 DROPS: 0, line 6 SCALE: 100 %.
For 09-parameters:
- Starts SITL on a simulated NOR Flash file with UDP transport.
- Connects over UDP and verifies handshake succeeds.
- Sets ATT_KP to a non-default value (2.5) over the Link.
- Waits with wait_until_saved() for parameter_status showing changes stored (unsaved == 0).
- Verifies stdout report and OLED display lines (title, PARAM: ATT_KP, VALUE: 2.50, SAVED: YES, BYTES USED, SECTOR).
- Restarts SITL on the same parameter file.
- Reads ATT_KP back over the Link, verifying persistence across restart.
- Verifies OLED display shows persisted value 2.50, SAVED: YES, and parameter_status reports 0 skipped entries.
For 10-motor-outputs:
- Runs example with key presses at 1.0, 2.2, 3.4, 4.6, 5.8, 7.0 s and power cut at 8.3 s.
- Verifies that no applied duty on any output ever exceeds 20 % in magnitude (max_duty cap, exact 0.200).
- Verifies that output 1 follows each commanded demand (+10 %, +20 %, -10 %, -20 %).
- Verifies that outputs 2 to 4 stay at zero throughout the run.
- Verifies that output 1 returns to 0 within the demand timeout bound once demand stops.
- Verifies that every output reads 0 after actuator power is cut, within the reaction bound.
- Verifies display_on=1 and OLED screen lines (title, STEP: 0, DEMAND: +0.00, OUTPUT 1: +0.00, POWER: OFF).
"""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
from pathlib import Path
import re
import select
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT / "tools" / "link") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "tools" / "link"))

from display_text import screen_text  # noqa: E402
from sitl_support import free_udp_port, wait_until_saved  # noqa: E402
from vklink.monitor import LinkMonitor, UdpConnection, load_messages  # noqa: E402
from vklink.session import ParamType, ParamValue  # noqa: E402


def fixed_line_matches(line: str, prefix: str, value: float, suffix: str) -> bool:
    """Whether a display line shows `value` to two decimals between `prefix` and `suffix`.

    The report prints the value to three decimals and the display rounds the full value to two,
    so the two can differ by one in the last digit: at most 0.0005 + 0.005 apart.
    """
    if not (line.startswith(prefix) and line.endswith(suffix)):
        return False
    try:
        shown = float(line[len(prefix):len(line) - len(suffix)])
    except ValueError:
        return False
    return abs(shown - value) <= 0.006


def line_matches(line: str, expected) -> bool:
    """Whether a display line matches `expected`: a text, a (prefix, value, suffix) number to two
    decimals, or a list of either.

    A list holds what the last two display updates drew: SITL stops at a set time, which can fall
    between an item's report and its next draw, so the screen may still show the previous values.
    """
    if isinstance(expected, list):
        return any(line_matches(line, alternative) for alternative in expected)
    if isinstance(expected, tuple):
        return fixed_line_matches(line, *expected)
    return line == expected


def check_03_oled(stdout: str, display_file: Path, duration_s: float, tolerance: float) -> list[str]:
    failures: list[str] = []

    # 1. Check counter rate from stdout report
    counters = [int(m.group(1)) for m in re.finditer(r"^counter=(\d+)", stdout, re.MULTILINE)]
    if not counters:
        failures.append("no counter reported in stdout")
    else:
        for i in range(len(counters) - 1):
            if counters[i + 1] != counters[i] + 1:
                failures.append(f"counter gap: {counters[i]} followed by {counters[i + 1]}")
                break

        measured_rate = len(counters) / duration_s
        expected_rate = 10.0
        if abs(measured_rate - expected_rate) > expected_rate * tolerance:
            failures.append(
                f"counter rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                f"(tolerance {tolerance:.0%})"
            )

    # 2. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 3. Judge whole frames from the captured display image:
    # line 0 the title, line 2 the last reported counter, every other line blank
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if not counters:
                failures.append("cannot verify display content without reported counters")
            else:
                expected_lines = {
                    0: "VEHKERNEL 03-OLED",
                    1: "",
                    2: f"COUNTER: {counters[-1]}",
                    3: "",
                    4: "",
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected in expected_lines.items():
                    ok = line_matches(lines[line_idx], expected)
                    if not ok:
                        failures.append(
                            f"display line {line_idx}: expected {expected!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def check_04_work_queues(
    stdout: str,
    stdout_lines: list[tuple[str, float]],
    display_file: Path,
    tolerance: float,
) -> list[str]:
    failures: list[str] = []

    # 1. Parse counters and arrival timestamps
    fast_reports: list[tuple[int, float]] = []
    slow_reports: list[tuple[int, float]] = []

    for line, t in stdout_lines:
        m_fast = re.match(r"^fast=(\d+)", line)
        if m_fast:
            fast_reports.append((int(m_fast.group(1)), t))
        m_slow = re.match(r"^slow=(\d+)", line)
        if m_slow:
            slow_reports.append((int(m_slow.group(1)), t))

    # 2. Check fast counter: no gaps and rate
    if not fast_reports:
        failures.append("no fast counter reported in stdout")
    else:
        for i in range(len(fast_reports) - 1):
            if fast_reports[i + 1][0] != fast_reports[i][0] + 1:
                failures.append(
                    f"fast counter gap: {fast_reports[i][0]} followed by {fast_reports[i + 1][0]}"
                )
                break

        if len(fast_reports) < 2:
            failures.append("too few fast reports to compute rate")
        else:
            span = fast_reports[-1][1] - fast_reports[0][1]
            if span <= 0:
                failures.append(f"invalid fast time span: {span:.3f} s")
            else:
                measured_rate = (len(fast_reports) - 1) / span
                expected_rate = 10.0
                if abs(measured_rate - expected_rate) > expected_rate * tolerance:
                    failures.append(
                        f"fast rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                        f"(tolerance {tolerance:.0%})"
                    )

    # 3. Check slow counter: no gaps and rate
    if not slow_reports:
        failures.append("no slow counter reported in stdout")
    else:
        for i in range(len(slow_reports) - 1):
            if slow_reports[i + 1][0] != slow_reports[i][0] + 1:
                failures.append(
                    f"slow counter gap: {slow_reports[i][0]} followed by {slow_reports[i + 1][0]}"
                )
                break

        if len(slow_reports) < 2:
            failures.append("too few slow reports to compute rate")
        else:
            span = slow_reports[-1][1] - slow_reports[0][1]
            if span <= 0:
                failures.append(f"invalid slow time span: {span:.3f} s")
            else:
                measured_rate = (len(slow_reports) - 1) / span
                expected_rate = 2.0
                if abs(measured_rate - expected_rate) > expected_rate * tolerance:
                    failures.append(
                        f"slow rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                        f"(tolerance {tolerance:.0%})"
                    )

    # 4. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 5. Judge whole frames from captured display image:
    # line 0 the title, line 2 the last reported fast counter, line 4 the last reported slow counter,
    # every other line blank
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if not fast_reports or not slow_reports:
                failures.append("cannot verify display content without reported counters")
            else:
                expected_lines = {
                    0: "VEHKERNEL 04-WQ",
                    1: "",
                    2: [f"FAST: {n}" for n, _ in fast_reports[-2:]],
                    3: "",
                    4: [f"SLOW: {n}" for n, _ in slow_reports[-2:]],
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected in expected_lines.items():
                    ok = line_matches(lines[line_idx], expected)
                    if not ok:
                        failures.append(
                            f"display line {line_idx}: expected {expected!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def check_05_bus(
    stdout: str,
    stdout_lines: list[tuple[str, float]],
    display_file: Path,
    tolerance: float,
) -> list[str]:
    failures: list[str] = []

    # 1. Parse published and received reports with timestamps
    published_reports: list[tuple[int, float]] = []
    received_reports: list[tuple[int, float]] = []

    for line, t in stdout_lines:
        m_pub = re.match(r"^published=(\d+)", line)
        if m_pub:
            published_reports.append((int(m_pub.group(1)), t))
        m_recv = re.match(r"^received=(\d+)", line)
        if m_recv:
            received_reports.append((int(m_recv.group(1)), t))

    # 2. Check published counter: no gaps and rate
    if not published_reports:
        failures.append("no published counter reported in stdout")
    else:
        for i in range(len(published_reports) - 1):
            if published_reports[i + 1][0] != published_reports[i][0] + 1:
                failures.append(
                    f"published counter gap: {published_reports[i][0]} followed by {published_reports[i + 1][0]}"
                )
                break

        if len(published_reports) < 2:
            failures.append("too few published reports to compute rate")
        else:
            span = published_reports[-1][1] - published_reports[0][1]
            if span <= 0:
                failures.append(f"invalid published time span: {span:.3f} s")
            else:
                measured_rate = (len(published_reports) - 1) / span
                expected_rate = 10.0
                if abs(measured_rate - expected_rate) > expected_rate * tolerance:
                    failures.append(
                        f"publication rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                        f"(tolerance {tolerance:.0%})"
                    )

    # 3. Check received values: exactly the published ones, each once and in order
    if not received_reports:
        failures.append("no received counter reported in stdout")
    else:
        # Check that the received values are exactly the published ones, each once and in order.
        # Allow the last one or two published just before the stop not to be received yet,
        # because the publisher runs on higher-priority wq:io and may publish just before
        # SITL stops before the subscriber on wq:slow has been scheduled to drain the queue.
        pub_values = [p[0] for p in published_reports]
        recv_values = [r[0] for r in received_reports]

        lag = len(pub_values) - len(recv_values)
        if lag < 0:
            failures.append(
                f"more values received ({len(recv_values)}) than published ({len(pub_values)})"
            )
        elif lag > 2:
            failures.append(
                f"too many published values not received yet: {lag} unreceived (published {len(pub_values)}, "
                f"received {len(recv_values)})"
            )
        else:
            for i, (recv, pub) in enumerate(zip(recv_values, pub_values)):
                if recv != pub:
                    failures.append(
                        f"received value mismatch at index {i}: expected {pub}, got {recv}"
                    )
                    break

    # 4. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 5. Judge whole frames from captured display image:
    # line 0 the title, line 2 the last received count, line 4 the total received count,
    # every other line blank
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if not received_reports:
                failures.append("cannot verify display content without reported received counters")
            else:
                expected_lines = {
                    0: "VEHKERNEL 05-BUS",
                    1: "",
                    2: [f"COUNT: {n}" for n, _ in received_reports[-2:]],
                    3: "",
                    4: [f"RECEIVED: {len(received_reports) - k}" for k in (0, 1)],
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected in expected_lines.items():
                    ok = line_matches(lines[line_idx], expected)
                    if not ok:
                        failures.append(
                            f"display line {line_idx}: expected {expected!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def check_06_imu(
    stdout: str,
    stdout_lines: list[tuple[str, float]],
    display_file: Path,
    tolerance: float,
) -> list[str]:
    failures: list[str] = []

    # 1. Check sample rate reported once per second
    rates = [float(m.group(1)) for m in re.finditer(r"^rate=([0-9.]+)", stdout, re.MULTILINE)]
    if not rates:
        failures.append("no rate reported in stdout")
    else:
        expected_rate = 200.0
        # The mean of the once-a-second rates: a host stall on a loaded runner can starve one
        # second alone, which says nothing about the driver.
        mean_rate = sum(rates) / len(rates)
        if abs(mean_rate - expected_rate) > expected_rate * tolerance:
            failures.append(
                f"mean sample rate {mean_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                f"(tolerance {tolerance:.0%}; per second {rates})"
            )

    # 2. Check down axis reported at display rate
    downs = [m.group(1) for m in re.finditer(r"^down=([^\s]+)", stdout, re.MULTILINE)]
    if not downs:
        failures.append("no down axis reported in stdout")
    else:
        expected_down = "+Z"
        for d in downs:
            if d != expected_down:
                failures.append(f"expected down axis {expected_down!r}, got {d!r}")
                break

    # 3. Check specific force components reported at display rate
    ax_reports = [float(m.group(1)) for m in re.finditer(r"^ax=([+-]?[0-9.]+)", stdout, re.MULTILINE)]
    ay_reports = [float(m.group(1)) for m in re.finditer(r"^ay=([+-]?[0-9.]+)", stdout, re.MULTILINE)]
    az_reports = [float(m.group(1)) for m in re.finditer(r"^az=([+-]?[0-9.]+)", stdout, re.MULTILINE)]

    if not ax_reports or not ay_reports or not az_reports:
        failures.append("specific force not reported in stdout")
    else:
        last_ax = ax_reports[-1]
        last_ay = ay_reports[-1]
        last_az = az_reports[-1]
        if abs(last_ax) > 1.0:
            failures.append(f"ax {last_ax:.3f} m/s^2 unexpectedly far from 0")
        if abs(last_ay) > 1.0:
            failures.append(f"ay {last_ay:.3f} m/s^2 unexpectedly far from 0")
        if abs(last_az - (-9.81)) > 1.0:
            failures.append(f"az {last_az:.3f} m/s^2 unexpectedly far from -9.81 m/s^2")

    # 4. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 5. Judge whole frames from captured display image:
    # line 0 the title, lines 2-4 specific force, line 5 rate, line 6 down, lines 1 and 7 blank
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if not rates or not downs or not ax_reports or not ay_reports or not az_reports:
                failures.append("cannot verify display content without reported values")
            else:
                expected_lines = {
                    0: "VEHKERNEL 06-IMU",
                    1: "",
                    2: [("AX: ", v, " M/S2") for v in ax_reports[-2:]],
                    3: [("AY: ", v, " M/S2") for v in ay_reports[-2:]],
                    4: [("AZ: ", v, " M/S2") for v in az_reports[-2:]],
                    5: [f"RATE: {round(r)} HZ" for r in rates[-2:]],
                    6: [f"DOWN: {d}" for d in downs[-2:]],
                    7: "",
                }
                for line_idx, expected in expected_lines.items():
                    ok = line_matches(lines[line_idx], expected)
                    if not ok:
                        failures.append(
                            f"display line {line_idx}: expected {expected!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def check_07_magnetometer(
    stdout: str,
    stdout_lines: list[tuple[str, float]],
    display_file: Path,
    tolerance: float,
) -> list[str]:
    failures: list[str] = []

    # 1. Check magnetometer sample rate reported once per second against 100 Hz within tolerance
    rates = [float(m.group(1)) for m in re.finditer(r"^rate=([0-9.]+)", stdout, re.MULTILINE)]
    if not rates:
        failures.append("no rate reported in stdout")
    else:
        expected_rate = 100.0  # AK09916 magnetometer rate in SITL (spec 0012, sim/icm20948.cpp)
        # The mean of the once-a-second rates: a host stall on a loaded runner can starve one
        # second alone, which says nothing about the driver.
        mean_rate = sum(rates) / len(rates)
        if abs(mean_rate - expected_rate) > expected_rate * tolerance:
            failures.append(
                f"mean sample rate {mean_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                f"(tolerance {tolerance:.0%}; per second {rates})"
            )

    # 2. Check heading follows simulated vehicle true yaw during rotation
    headings: list[tuple[float, float]] = []
    heading_times: list[int] = []
    yaws: list[float] = []
    yaw_times: list[int] = []
    bx_reports: list[float] = []
    by_reports: list[float] = []
    bz_reports: list[float] = []
    norm_reports: list[float] = []

    for line, t in stdout_lines:
        if m := re.match(r"^heading_us=(\d+)", line):
            heading_times.append(int(m.group(1)))
        elif m := re.match(r"^heading=([+-]?[0-9.]+)", line):
            headings.append((float(m.group(1)), t))
        elif m := re.match(r"^true_yaw_us=(\d+)", line):
            yaw_times.append(int(m.group(1)))
        elif m := re.match(r"^true_yaw=([+-]?[0-9.]+)", line):
            yaws.append(float(m.group(1)))
        elif m := re.match(r"^bx=([+-]?[0-9.]+)", line):
            bx_reports.append(float(m.group(1)))
        elif m := re.match(r"^by=([+-]?[0-9.]+)", line):
            by_reports.append(float(m.group(1)))
        elif m := re.match(r"^bz=([+-]?[0-9.]+)", line):
            bz_reports.append(float(m.group(1)))
        elif m := re.match(r"^norm=([+-]?[0-9.]+)", line):
            norm_reports.append(float(m.group(1)))

    if not headings:
        failures.append("no heading reported in stdout")
    if not yaws:
        failures.append("no true_yaw reported in stdout")
    if len(heading_times) != len(headings) or len(yaw_times) != len(yaws):
        failures.append("a heading or true_yaw was reported without its time")

    def wrap_180(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    # Each heading is compared with the true yaw at the time of the magnetometer sample it comes
    # from, interpolated between the true yaws reported around it (both on SITL's clock), so the
    # host's scheduling of the two reporting work items does not matter. The rotate profile turns
    # at a constant rate, so the interpolation is exact. What is left: the field noise (0.3 uT on
    # 30 uT, about 0.6 deg) and the sample's age, at most one AK09916 period plus one driver read
    # period, 15 ms (test/sitl/check_magnetometer.py), 0.7 deg at 45 deg/s. 3 deg covers both; a
    # heading one display period (11.25 deg) off fails.
    max_heading_error_deg = 3.0
    compared = 0

    if headings and yaws and len(heading_times) == len(headings) and len(yaw_times) == len(yaws):
        unwrapped_yaws: list[float] = []
        for y in yaws:
            unwrapped_yaws.append(y if not unwrapped_yaws else unwrapped_yaws[-1] + wrap_180(y - unwrapped_yaws[-1]))

        for (h_val, _), h_us in zip(headings, heading_times):
            k = bisect.bisect_left(yaw_times, h_us)
            if k == 0 or k >= len(yaw_times) or yaw_times[k] == yaw_times[k - 1]:
                continue  # outside the true yaws reported, or nothing to interpolate between
            if abs(wrap_180(unwrapped_yaws[k] - unwrapped_yaws[k - 1])) < 1.0:
                continue  # not turning yet
            f = (h_us - yaw_times[k - 1]) / (yaw_times[k] - yaw_times[k - 1])
            true_yaw = unwrapped_yaws[k - 1] + f * (unwrapped_yaws[k] - unwrapped_yaws[k - 1])
            error = abs(wrap_180(h_val - true_yaw))
            compared += 1
            if error > max_heading_error_deg:
                failures.append(
                    f"heading error {error:.2f} deg exceeds tolerance {max_heading_error_deg:.1f} deg "
                    f"(heading={h_val:.2f} at {h_us} us, true yaw {wrap_180(true_yaw):.2f})"
                )
                break

        # The run turns for about 9 s at 4 reports a second; on a loaded runner fewer arrive.
        if compared < 10:
            failures.append(f"only {compared} headings compared during turning (expected at least 10)")

        # Check that heading sweeps through at least most of a turn over the run.
        # A full rotation is 360 deg; sweeping at least 270 deg (3/4 of a turn) satisfies this criterion.
        unwrapped: list[float] = []
        for h_val, _ in headings:
            if not unwrapped:
                unwrapped.append(h_val)
            else:
                diff = wrap_180(h_val - unwrapped[-1])
                unwrapped.append(unwrapped[-1] + diff)
        sweep_span = max(unwrapped) - min(unwrapped)
        if sweep_span < 270.0:
            failures.append(
                f"heading swept {sweep_span:.1f} deg, less than required turn sweep of at least 270 deg"
            )

    # 3. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 4. Judge whole frames from captured display image:
    # line 0 title, line 1 blank, lines 2-4 field components, line 5 norm, line 6 rate, line 7 heading
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if (
                not rates
                or not headings
                or not bx_reports
                or not by_reports
                or not bz_reports
                or not norm_reports
            ):
                failures.append("cannot verify display content without reported values")
            else:
                expected_lines = {
                    0: "VEHKERNEL 07-MAG",
                    1: "",
                    2: [("BX: ", v, " UT") for v in bx_reports[-2:]],
                    3: [("BY: ", v, " UT") for v in by_reports[-2:]],
                    4: [("BZ: ", v, " UT") for v in bz_reports[-2:]],
                    5: [("NORM: ", v, " UT") for v in norm_reports[-2:]],
                    6: [f"RATE: {round(r)} HZ" for r in rates[-2:]],
                    7: [("HEADING: ", h, " DEG") for h, _ in headings[-2:]],
                }
                for line_idx, expected in expected_lines.items():
                    ok = line_matches(lines[line_idx], expected)
                    if not ok:
                        failures.append(
                            f"display line {line_idx}: expected {expected!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def check_08_serial_link(
    stdout: str,
    display_file: Path,
    monitor,
    tolerance: float,
) -> list[str]:
    failures: list[str] = []

    # 1. Check handshake
    if not monitor.connected:
        failures.append("no handshake with SITL Link")

    # 2. Check imu_sample instance 0 rate
    imu_stats = monitor.topics.get(("imu_sample", 0))
    if imu_stats is None or imu_stats.messages < 2:
        failures.append("topic imu_sample:0 never arrived or arrived with too few samples")
    else:
        expected_rate = 200.0
        measured_rate = imu_stats.rate_hz
        if abs(measured_rate - expected_rate) > expected_rate * tolerance:
            failures.append(
                f"imu_sample:0 rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                f"(tolerance {tolerance:.0%})"
            )

    # 3. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 4. Judge whole frames from captured display image:
    # line 0 title, line 2 state, line 3 tx frames, line 4 rx frames, line 5 drops, line 6 scale
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            expected_lines = {
                0: "VEHKERNEL 08-LINK",
                1: "",
                2: "STATE: CONNECTED",
                5: "DROPS: 0",
                6: "SCALE: 100 %",
                7: "",
            }
            for line_idx, expected in expected_lines.items():
                if lines[line_idx] != expected:
                    failures.append(
                        f"display line {line_idx}: expected {expected!r}, got {lines[line_idx]!r}"
                    )
            if not lines[3].startswith("TX FRAMES: "):
                failures.append(f"display line 3: expected prefix 'TX FRAMES: ', got {lines[3]!r}")
            if not lines[4].startswith("RX FRAMES: "):
                failures.append(f"display line 4: expected prefix 'RX FRAMES: ', got {lines[4]!r}")
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


# Each 09-parameters run: long enough to connect, set and see the change stored, then ends by itself.
RUN_S = 8.0


def check_09_parameters(target: Path, msg_dir: str) -> list[str]:
    failures: list[str] = []

    with tempfile.TemporaryDirectory() as tmp:
        param_file = Path(tmp) / "params.bin"
        display1 = Path(tmp) / "display1.pbm"
        display2 = Path(tmp) / "display2.pbm"

        # 1. First run: Start on a fresh parameter file, set ATT_KP to 2.5 over the Link
        port1 = free_udp_port()
        cmd1 = [
            str(target),
            "--duration", str(RUN_S),
            "--display", str(display1),
            "--udp", f"127.0.0.1:{port1}",
            "--param-file", str(param_file),
        ]
        proc1 = subprocess.Popen(cmd1, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        assert proc1.stdout is not None
        first_line1 = proc1.stdout.readline()
        print(first_line1, end="")
        m1 = re.search(r"address=127\.0\.0\.1:(\d+)", first_line1)
        if not m1:
            proc1.kill()
            _, err1 = proc1.communicate(timeout=10)
            return [f"could not parse bound UDP port from SITL output: {err1}"]

        conn1 = UdpConnection("127.0.0.1", port1)
        monitor1 = LinkMonitor(conn1, load_messages(msg_dir))
        try:
            if not monitor1.handshake(timeout_s=4.0):
                failures.append("Run 1: no handshake with SITL Link")
            else:
                reply = monitor1.set_param("ATT_KP", ParamType.FLOAT, 2.5, timeout_s=3.0)
                if not isinstance(reply, ParamValue) or abs(reply.value - 2.5) > 1e-4:
                    failures.append(f"Run 1: failed to set ATT_KP to 2.5: {reply}")
                else:
                    saved_status = wait_until_saved(monitor1, timeout_s=6.0)
                    if saved_status is None:
                        failures.append("Run 1: parameter changes were not stored (wait_until_saved timed out)")
                    else:
                        print(f"stored: bytes_used={saved_status.bytes_used} sector={saved_status.active_sector}")
                    # Allow display work item (at 4 Hz) to update the display with the new saved state
                    monitor1.run(0.5)
        finally:
            conn1.close()
            # Let the run end by itself, so it prints its last report lines.
            out1, err1 = proc1.communicate(timeout=RUN_S + 30)

        display_on1 = re.search(r"^display_on=(\d+)", out1, re.MULTILINE)
        if not display_on1 or int(display_on1.group(1)) != 1:
            failures.append("Run 1: display_on=1 not reported")

        if "param=ATT_KP" not in out1:
            failures.append("Run 1: stdout report does not contain param=ATT_KP")
        if not re.search(r"value=2\.500", out1):
            failures.append("Run 1: stdout report does not contain value=2.500")

        if not display1.exists():
            failures.append("Run 1: SITL wrote no display image")
        else:
            try:
                lines1 = screen_text(display1)
                print("Run 1 display:")
                for idx, line in enumerate(lines1):
                    print(f"  [{idx}] |{line}|")

                expected_lines1 = {
                    0: "VEHKERNEL 09-PARAMS",
                    1: "",
                    2: "PARAM: ATT_KP",
                    4: "SAVED: YES",
                    7: "",
                }
                for line_idx, expected in expected_lines1.items():
                    if lines1[line_idx] != expected:
                        failures.append(
                            f"Run 1 display line {line_idx}: expected {expected!r}, got {lines1[line_idx]!r}"
                        )
                if not fixed_line_matches(lines1[3], "VALUE: ", 2.5, ""):
                    failures.append(f"Run 1 display line 3: expected 'VALUE: 2.50', got {lines1[3]!r}")
                if not lines1[5].startswith("BYTES USED: "):
                    failures.append(f"Run 1 display line 5: expected prefix 'BYTES USED: ', got {lines1[5]!r}")
                if not lines1[6].startswith("SECTOR: "):
                    failures.append(f"Run 1 display line 6: expected prefix 'SECTOR: ', got {lines1[6]!r}")
            except Exception as e:
                failures.append(f"Run 1: failed to read display image: {e}")

        # 2. Second run: Restart on the same parameter file and read value back
        port2 = free_udp_port()
        cmd2 = [
            str(target),
            "--duration", str(RUN_S),
            "--display", str(display2),
            "--udp", f"127.0.0.1:{port2}",
            "--param-file", str(param_file),
        ]
        proc2 = subprocess.Popen(cmd2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        assert proc2.stdout is not None
        first_line2 = proc2.stdout.readline()
        print(first_line2, end="")
        m2 = re.search(r"address=127\.0\.0\.1:(\d+)", first_line2)
        if not m2:
            proc2.kill()
            _, err2 = proc2.communicate(timeout=10)
            return failures + [f"Run 2: could not parse bound UDP port: {err2}"]

        conn2 = UdpConnection("127.0.0.1", port2)
        monitor2 = LinkMonitor(conn2, load_messages(msg_dir))
        try:
            if not monitor2.handshake(timeout_s=4.0):
                failures.append("Run 2: no handshake with SITL Link after restart")
            else:
                reply2 = monitor2.get_param("ATT_KP", timeout_s=3.0)
                if not isinstance(reply2, ParamValue) or abs(reply2.value - 2.5) > 1e-4:
                    failures.append(
                        f"Run 2: ATT_KP after restart is {getattr(reply2, 'value', reply2)}, expected 2.5"
                    )
                else:
                    print(f"Run 2: ATT_KP read back successfully as {reply2.value}")

                monitor2.run(1.5)
                stats = monitor2.topics.get(("parameter_status", 0))
                if stats and stats.latest is not None:
                    status2 = stats.latest
                    skipped = (status2.skipped_unknown + status2.skipped_wrong_type +
                               status2.skipped_out_of_range + status2.skipped_corrupt)
                    print(f"Run 2 status: bytes_used={status2.bytes_used} sector={status2.active_sector} "
                          f"unsaved={status2.unsaved} skipped={skipped}")
                    if skipped != 0 or status2.unsaved != 0 or status2.failed:
                        failures.append(f"Run 2: parameter_status after restart showed errors: {status2}")
                else:
                    failures.append("Run 2: no parameter_status received after restart")
        finally:
            conn2.close()
            out2, err2 = proc2.communicate(timeout=RUN_S + 30)

        display_on2 = re.search(r"^display_on=(\d+)", out2, re.MULTILINE)
        if not display_on2 or int(display_on2.group(1)) != 1:
            failures.append("Run 2: display_on=1 not reported")

        if not display2.exists():
            failures.append("Run 2: SITL wrote no display image")
        else:
            try:
                lines2 = screen_text(display2)
                print("Run 2 display:")
                for idx, line in enumerate(lines2):
                    print(f"  [{idx}] |{line}|")

                expected_lines2 = {
                    0: "VEHKERNEL 09-PARAMS",
                    1: "",
                    2: "PARAM: ATT_KP",
                    4: "SAVED: YES",
                    7: "",
                }
                for line_idx, expected in expected_lines2.items():
                    if lines2[line_idx] != expected:
                        failures.append(
                            f"Run 2 display line {line_idx}: expected {expected!r}, got {lines2[line_idx]!r}"
                        )
                if not fixed_line_matches(lines2[3], "VALUE: ", 2.5, ""):
                    failures.append(f"Run 2 display line 3: expected 'VALUE: 2.50', got {lines2[3]!r}")
                if not lines2[5].startswith("BYTES USED: "):
                    failures.append(f"Run 2 display line 5: expected prefix 'BYTES USED: ', got {lines2[5]!r}")
                if not lines2[6].startswith("SECTOR: "):
                    failures.append(f"Run 2 display line 6: expected prefix 'SECTOR: ', got {lines2[6]!r}")
            except Exception as e:
                failures.append(f"Run 2: failed to read display image: {e}")

    return failures


@dataclass(frozen=True)
class TimelineStep:
    label: str
    key_press_s: float | None
    check_start_s: float
    check_end_s: float
    expected_demand: float
    expected_out1: float
    out1_tol: float = 0.02


TIMELINE_10_MOTOR_OUTPUTS: list[TimelineStep] = [
    TimelineStep("boot (0 %)", None, 0.4, 0.9, 0.00, 0.00, 0.005),
    TimelineStep("step 1 (+10 %)", 1.0, 1.6, 2.1, 0.10, 0.10, 0.02),
    TimelineStep("step 2 (+20 %)", 2.2, 2.8, 3.3, 0.20, 0.20, 0.02),
    TimelineStep("step 3 (-10 %)", 3.4, 4.0, 4.5, -0.10, -0.10, 0.02),
    TimelineStep("step 4 (-20 %)", 4.6, 5.2, 5.7, -0.20, -0.20, 0.02),
    TimelineStep("step 5 (0 %, demand stopped)", 5.8, 6.4, 6.9, 0.00, 0.00, 0.005),
    TimelineStep("step 6 (+10 %, held)", 7.0, 7.6, 8.1, 0.10, 0.10, 0.02),
]
POWER_CUT_TIME_10_MOTOR_OUTPUTS = 8.3
POST_CUT_CHECK_START_10_MOTOR_OUTPUTS = 8.7
POST_CUT_CHECK_END_10_MOTOR_OUTPUTS = 9.2
SCENARIO_DURATION_10_MOTOR_OUTPUTS = 9.5


def check_10_motor_outputs(
    stdout: str,
    stdout_lines: list[tuple[str, float]],
    display_file: Path,
) -> list[str]:
    """L3 check for 10-motor-outputs (spec 0013, ADR-0005, spec 0006).

    Checks:
    - No applied duty on any output ever exceeds 20 % in magnitude (max_duty cap, exact 0.200).
    - Output 1 follows each commanded demand (+10 %, +20 %, -10 %, -20 %).
    - Outputs 2 to 4 stay at 0 throughout the run.
    - Applied duty returns to 0 within the demand timeout bound once demand stops.
    - Every output reads 0 after power cut, within the reaction bound.
    - OLED display shows step 0, demand +0.00, output 1 +0.00, power OFF.
    """
    failures: list[str] = []

    # Bounds taken from the code:
    # - Cap: max_duty = 0.2F in main.cpp and vk::params::mot.out_max lowered to
    #   std::min(mot.out_max, max_duty) in RAM before ActuatorTask starts (spec 0013, ADR-0005).
    #   Enforced by ActuatorControl::step() clamping duty to [-loop.out_max, loop.out_max].
    #   No tolerance is added: applied duty must never exceed 0.200 in magnitude.
    # - Key debounce time: Debouncer::debounce_count = 3 consecutive agreeing samples
    #   sampled every 10 ms (30 ms debounce time, from 02-user-key and main.cpp).
    # - Speed slew rate: MOT_SLEWRATE default = 2.0 (1/s, full duty in 0.5 s, from
    #   modules/actuators/params.toml and actuator_control.cpp). Ramping 0.10 duty
    #   takes 50 ms; ramping 0.30 duty takes 150 ms.
    # - Motor model lag: Motor::Config::time_constant_s = 0.1 s in sim/include/vk/sim/motor.hpp.
    #   Within 3 time constants (0.3 s), the speed loop settles to within 5 % of target.
    # - Setpoint timeout: COM_SP_TIMEOUT default = 0.5 s in modules/commander/params.toml.
    #   In this example, stopping is disarmed immediately on step 5 (back to 0 %) by
    #   publishing armed = false, returning outputs to 0 within 0.4 s (well within COM_SP_TIMEOUT).
    # - Output watchdog interval: OutputWatchdog::periods = 3 base periods (15 ms at 200 Hz,
    #   from hal/include/vk/hal/output_watchdog.hpp). Refreshed every cycle by ActuatorTask on wq:ctrl.
    # - Actuator power cut reaction: power cut occurs at 8.3 s. ActuatorPowerSense reads
    #   unpowered in 1 control cycle (5 ms, DemandItem on wq:ctrl), resetting step to 0,
    #   publishing armed = false, and emitting demand=0.000; simulated motor power is cut
    #   immediately in sim/src/motor.cpp; and display reports at 4 Hz (250 ms). Within 0.4 s
    #   (by 8.7 s), every output reads 0.

    MAX_DUTY_CAP = 0.200  # Exact 20 % duty, enforced by MOT_OUT_MAX clamp

    # 1. Parse report entries and timestamps
    parsed_lines: list[tuple[float, str, float]] = []
    for line, t in stdout_lines:
        line_str = line.strip()
        m_dem = re.match(r"^demand=([+-]?\d+(?:\.\d+)?)", line_str)
        if m_dem:
            parsed_lines.append((t, "demand", float(m_dem.group(1))))
            continue
        m_out1 = re.match(r"^out1=([+-]?\d+(?:\.\d+)?)", line_str)
        if m_out1:
            parsed_lines.append((t, "out1", float(m_out1.group(1))))
            continue
        m_out2 = re.match(r"^out2=([+-]?\d+(?:\.\d+)?)", line_str)
        if m_out2:
            parsed_lines.append((t, "out2", float(m_out2.group(1))))
            continue
        m_out3 = re.match(r"^out3=([+-]?\d+(?:\.\d+)?)", line_str)
        if m_out3:
            parsed_lines.append((t, "out3", float(m_out3.group(1))))
            continue
        m_out4 = re.match(r"^out4=([+-]?\d+(?:\.\d+)?)", line_str)
        if m_out4:
            parsed_lines.append((t, "out4", float(m_out4.group(1))))
            continue
        m_pow = re.match(r"^power=(\d+)", line_str)
        if m_pow:
            parsed_lines.append((t, "power", float(m_pow.group(1))))
            continue

    if not parsed_lines:
        failures.append("no report lines captured from SITL output")
        return failures

    t0 = parsed_lines[0][0]

    # 2. Check maximum duty cap (<= 0.200 in magnitude) across all outputs with no tolerance added
    for t, kind, val in parsed_lines:
        if kind in ("out1", "out2", "out3", "out4"):
            if abs(val) > MAX_DUTY_CAP:
                failures.append(
                    f"output {kind} exceeded 20 % duty cap: {val:.3f} at t={t - t0:.2f} s"
                )

    # 3. Check outputs 2, 3, 4 stay at 0
    for t, kind, val in parsed_lines:
        if kind in ("out2", "out3", "out4"):
            if abs(val) > 1e-4:
                failures.append(
                    f"output {kind} was not zero: {val:.3f} at t={t - t0:.2f} s"
                )

    # 4. Check demand sequence: derived from timeline plus power cut disarm
    expected_seq = [s.expected_demand for s in TIMELINE_10_MOTOR_OUTPUTS] + [0.00]
    demands = [val for _, kind, val in parsed_lines if kind == "demand"]
    if len(demands) < len(expected_seq):
        failures.append(
            f"expected at least {len(expected_seq)} demand transitions, got {len(demands)}: {demands}"
        )
    else:
        for idx, expected in enumerate(expected_seq):
            if abs(demands[idx] - expected) > 0.005:
                failures.append(
                    f"demand transition {idx}: expected {expected:.2f}, got {demands[idx]:.2f}"
                )

    # 5. Check output 1 follows each demand in steady-state windows
    for step in TIMELINE_10_MOTOR_OUTPUTS:
        samples = [
            val for t, kind, val in parsed_lines
            if kind == "out1" and step.check_start_s <= (t - t0) <= step.check_end_s
        ]
        if not samples:
            failures.append(
                f"no out1 samples in window [{step.check_start_s:.1f}, {step.check_end_s:.1f}] for {step.label}"
            )
        else:
            last_sample = samples[-1]
            if abs(last_sample - step.expected_out1) > step.out1_tol:
                failures.append(
                    f"{step.label}: out1 reached {last_sample:.3f}, expected {step.expected_out1:.3f} "
                    f"(tol {step.out1_tol:.3f})"
                )

    # 6. Check power cut: every output reads 0 after power cut, power reads 0
    post_cut_out1 = [
        val for t, kind, val in parsed_lines
        if kind == "out1" and (t - t0) >= POST_CUT_CHECK_START_10_MOTOR_OUTPUTS
    ]
    if not post_cut_out1:
        failures.append(
            f"no out1 samples after power cut (t >= {POST_CUT_CHECK_START_10_MOTOR_OUTPUTS:.1f} s)"
        )
    else:
        for val in post_cut_out1:
            if abs(val) > 0.005:
                failures.append(f"out1 did not read 0 after power cut: {val:.3f}")
                break

    post_cut_powers = [
        val for t, kind, val in parsed_lines
        if kind == "power" and (t - t0) >= POST_CUT_CHECK_START_10_MOTOR_OUTPUTS
    ]
    if not post_cut_powers or any(p != 0.0 for p in post_cut_powers):
        failures.append("power did not report 0 after power cut")

    # 7. Check display_on=1 and OLED frame content
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            expected_lines = {
                0: "VEHKERNEL 10-MOTORS",
                1: "",
                2: "STEP: 0",
                3: ("DEMAND: ", 0.0, ""),
                4: ("OUTPUT 1: ", 0.0, ""),
                5: "POWER: OFF",
                6: "",
                7: "",
            }
            for line_idx, expected in expected_lines.items():
                ok = line_matches(lines[line_idx], expected)
                if not ok:
                    failures.append(
                        f"display line {line_idx}: expected {expected!r}, got {lines[line_idx]!r}"
                    )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("executable", help="the example executable to run (e.g. example-03-oled)")
    parser.add_argument("--duration", type=float, default=3.0, help="duration to run SITL in seconds")
    parser.add_argument("--tolerance", type=float, default=0.20, help="allowed relative rate error")
    parser.add_argument("--msg-dir", default=None, help="directory of the generated vkmsg.py")
    args = parser.parse_args()

    target = Path(args.executable)
    name = target.name
    if name.startswith("example-"):
        ex_name = name.removeprefix("example-")
    else:
        ex_name = name

    if ex_name == "09-parameters":
        if not args.msg_dir:
            print("FAIL: 09-parameters needs --msg-dir, the build's generated schema Python", file=sys.stderr)
            return 1
        failures = check_09_parameters(target, args.msg_dir)
        if failures:
            for f in failures:
                print(f"FAIL: {f}", file=sys.stderr)
            return 1
        print(f"PASS: {name} completed successfully")
        return 0

    with tempfile.TemporaryDirectory() as tmp:
        display_file = Path(tmp) / "display.pbm"
        duration = (
            10.0 if ex_name == "07-magnetometer"
            else (5.0 if ex_name == "08-serial-link"
                  else (SCENARIO_DURATION_10_MOTOR_OUTPUTS if ex_name == "10-motor-outputs"
                        else args.duration))
        )
        cmd = [str(target), "--duration", str(duration), "--display", str(display_file)]
        if ex_name == "07-magnetometer":
            cmd.extend(["--motion", "rotate"])
        if ex_name == "08-serial-link":
            cmd.extend(["--udp", "127.0.0.1:0"])
        if ex_name == "10-motor-outputs":
            key_times = [
                f"{s.key_press_s:.1f}"
                for s in TIMELINE_10_MOTOR_OUTPUTS
                if s.key_press_s is not None
            ]
            cmd.extend([
                "--key-presses", ",".join(key_times),
                "--power-cut-at", f"{POWER_CUT_TIME_10_MOTOR_OUTPUTS:.1f}",
            ])

        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        stdout_lines: list[tuple[str, float]] = []
        monitor = None

        if ex_name == "08-serial-link":
            assert proc.stdout is not None
            first_line = proc.stdout.readline()
            stdout_lines.append((first_line, time.monotonic()))
            print(first_line, end="")
            m = re.search(r"address=127\.0\.0\.1:(\d+)", first_line)
            if not m:
                proc.kill()
                _, stderr = proc.communicate(timeout=10)
                print(stderr, file=sys.stderr)
                print("FAIL: could not parse bound UDP port from SITL output", file=sys.stderr)
                return 1
            port = int(m.group(1))

            msg_dir = args.msg_dir
            if not msg_dir:
                proc.kill()
                print("FAIL: 08-serial-link needs --msg-dir, the build's generated schema Python", file=sys.stderr)
                return 1

            from vklink.monitor import LinkMonitor, UdpConnection, load_messages

            connection = UdpConnection("127.0.0.1", port)
            monitor = LinkMonitor(connection, load_messages(msg_dir))
            try:
                if monitor.handshake(timeout_s=2.0):
                    check_duration = max(1.0, duration - 2.0)
                    monitor.run(check_duration)
            finally:
                connection.close()

            out_rest, stderr = proc.communicate(timeout=30)
            for line in out_rest.splitlines(keepends=True):
                stdout_lines.append((line, time.monotonic()))
                print(line, end="")
        else:
            deadline = time.monotonic() + duration + 30
            assert proc.stdout is not None
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    proc.kill()
                    raise subprocess.TimeoutExpired(cmd, duration + 30)
                rlist, _, _ = select.select([proc.stdout], [], [], remaining)
                if not rlist:
                    proc.kill()
                    raise subprocess.TimeoutExpired(cmd, duration + 30)
                line = proc.stdout.readline()
                if not line:
                    break
                t = time.monotonic()
                stdout_lines.append((line, t))
                print(line, end="")

            _, stderr = proc.communicate(timeout=30)

        if proc.returncode != 0:
            print(stderr, file=sys.stderr)
            print(f"FAIL: {name} exited with {proc.returncode}", file=sys.stderr)
            return 1

        full_stdout = "".join(line for line, _ in stdout_lines)

        if ex_name == "03-oled":
            failures = check_03_oled(full_stdout, display_file, args.duration, args.tolerance)
        elif ex_name == "04-work-queues":
            failures = check_04_work_queues(full_stdout, stdout_lines, display_file, args.tolerance)
        elif ex_name == "05-bus":
            failures = check_05_bus(full_stdout, stdout_lines, display_file, args.tolerance)
        elif ex_name == "06-imu":
            failures = check_06_imu(full_stdout, stdout_lines, display_file, args.tolerance)
        elif ex_name == "07-magnetometer":
            failures = check_07_magnetometer(full_stdout, stdout_lines, display_file, args.tolerance)
        elif ex_name == "08-serial-link":
            assert monitor is not None
            failures = check_08_serial_link(full_stdout, display_file, monitor, args.tolerance)
        elif ex_name == "10-motor-outputs":
            failures = check_10_motor_outputs(full_stdout, stdout_lines, display_file)
        else:
            failures = [f"unknown SITL example {ex_name}"]

    if failures:
        for f in failures:
            print(f"FAIL: {f}", file=sys.stderr)
        return 1

    print(f"PASS: {name} completed successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
