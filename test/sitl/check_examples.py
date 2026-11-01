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
"""

from __future__ import annotations

import argparse
import bisect
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
                    if isinstance(expected, tuple):
                        ok = fixed_line_matches(lines[line_idx], *expected)
                    else:
                        ok = lines[line_idx] == expected
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
                    2: f"FAST: {fast_reports[-1][0]}",
                    3: "",
                    4: f"SLOW: {slow_reports[-1][0]}",
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected in expected_lines.items():
                    if isinstance(expected, tuple):
                        ok = fixed_line_matches(lines[line_idx], *expected)
                    else:
                        ok = lines[line_idx] == expected
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
                    2: f"COUNT: {received_reports[-1][0]}",
                    3: "",
                    4: f"RECEIVED: {len(received_reports)}",
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected in expected_lines.items():
                    if isinstance(expected, tuple):
                        ok = fixed_line_matches(lines[line_idx], *expected)
                    else:
                        ok = lines[line_idx] == expected
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
        for r in rates:
            if abs(r - expected_rate) > expected_rate * tolerance:
                failures.append(
                    f"sample rate {r:.2f} Hz outside expected {expected_rate:.2f} Hz "
                    f"(tolerance {tolerance:.0%})"
                )
                break

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
                last_rate = int(round(rates[-1]))
                last_down = downs[-1]

                expected_lines = {
                    0: "VEHKERNEL 06-IMU",
                    1: "",
                    2: ("AX: ", last_ax, " M/S2"),
                    3: ("AY: ", last_ay, " M/S2"),
                    4: ("AZ: ", last_az, " M/S2"),
                    5: f"RATE: {last_rate} HZ",
                    6: f"DOWN: {last_down}",
                    7: "",
                }
                for line_idx, expected in expected_lines.items():
                    if isinstance(expected, tuple):
                        ok = fixed_line_matches(lines[line_idx], *expected)
                    else:
                        ok = lines[line_idx] == expected
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
        for r in rates:
            if abs(r - expected_rate) > expected_rate * tolerance:
                failures.append(
                    f"sample rate {r:.2f} Hz outside expected {expected_rate:.2f} Hz "
                    f"(tolerance {tolerance:.0%})"
                )
                break

    # 2. Check heading follows simulated vehicle true yaw during rotation
    headings: list[tuple[float, float]] = []
    yaws: list[tuple[float, float]] = []
    bx_reports: list[float] = []
    by_reports: list[float] = []
    bz_reports: list[float] = []
    norm_reports: list[float] = []

    for line, t in stdout_lines:
        m_h = re.match(r"^heading=([+-]?[0-9.]+)", line)
        if m_h:
            headings.append((float(m_h.group(1)), t))
        m_y = re.match(r"^true_yaw=([+-]?[0-9.]+)", line)
        if m_y:
            yaws.append((float(m_y.group(1)), t))
        m_bx = re.match(r"^bx=([+-]?[0-9.]+)", line)
        if m_bx:
            bx_reports.append(float(m_bx.group(1)))
        m_by = re.match(r"^by=([+-]?[0-9.]+)", line)
        if m_by:
            by_reports.append(float(m_by.group(1)))
        m_bz = re.match(r"^bz=([+-]?[0-9.]+)", line)
        if m_bz:
            bz_reports.append(float(m_bz.group(1)))
        m_norm = re.match(r"^norm=([+-]?[0-9.]+)", line)
        if m_norm:
            norm_reports.append(float(m_norm.group(1)))

    if not headings:
        failures.append("no heading reported in stdout")
    if not yaws:
        failures.append("no true_yaw reported in stdout")

    def wrap_180(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    # In SITL, the rotate profile turns at 45 deg/s, and both DisplayItem and TrueYawItem report at 4 Hz
    # (a 250 ms period), turning 11.25 deg between display updates.
    # A correct heading has an error against its corresponding true yaw under 1.0 deg (sensor noise on a
    # 30 uT field is 0.3 uT, atan(0.3 / 30) = 0.57 deg). A heading lagging by one display period has an
    # error of at least 11.25 - 1.0 = 10.25 deg.
    # We choose a tolerance of 3.0 deg: this provides ample margin for host thread scheduling jitter on a
    # loaded CI runner while decisively rejecting any heading lagging by a display period (>= 10.25 deg).
    max_heading_error_deg = 3.0
    compared = 0

    if headings and yaws:
        first_heading_time = headings[0][1]
        yaw_times = [t for _, t in yaws]

        for h_val, h_t in headings:
            # Skip headings before the vehicle starts turning (first second of alignment/rest).
            if h_t < first_heading_time + 1.0:
                continue

            # Compare with true yaw over a window: take smallest error against true yaw reported
            # just before and just after the heading (covering sample acquisition and display latency).
            i = bisect.bisect_left(yaw_times, h_t)
            candidates = [yaws[k][0] for k in (i - 1, i) if 0 <= k < len(yaws)]
            if not candidates:
                continue

            sample_min_error = min(abs(wrap_180(h_val - y_val)) for y_val in candidates)
            compared += 1
            if sample_min_error > max_heading_error_deg:
                failures.append(
                    f"heading error {sample_min_error:.2f} deg exceeds tolerance {max_heading_error_deg:.1f} deg "
                    f"(heading={h_val:.2f}, candidate true yaws={candidates})"
                )
                break

        if compared < 20:
            failures.append(f"only {compared} headings compared during turning (expected at least 20)")

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
                last_rate = int(round(rates[-1]))
                last_bx = bx_reports[-1]
                last_by = by_reports[-1]
                last_bz = bz_reports[-1]
                last_norm = norm_reports[-1]
                last_heading = headings[-1][0]

                expected_lines = {
                    0: "VEHKERNEL 07-MAG",
                    1: "",
                    2: ("BX: ", last_bx, " UT"),
                    3: ("BY: ", last_by, " UT"),
                    4: ("BZ: ", last_bz, " UT"),
                    5: ("NORM: ", last_norm, " UT"),
                    6: f"RATE: {last_rate} HZ",
                    7: ("HEADING: ", last_heading, " DEG"),
                }
                for line_idx, expected in expected_lines.items():
                    if isinstance(expected, tuple):
                        ok = fixed_line_matches(lines[line_idx], *expected)
                    else:
                        ok = lines[line_idx] == expected
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

    with tempfile.TemporaryDirectory() as tmp:
        display_file = Path(tmp) / "display.pbm"
        duration = 10.0 if ex_name == "07-magnetometer" else (5.0 if ex_name == "08-serial-link" else args.duration)
        cmd = [str(target), "--duration", str(duration), "--display", str(display_file)]
        if ex_name == "07-magnetometer":
            cmd.extend(["--motion", "rotate"])
        if ex_name == "08-serial-link":
            cmd.extend(["--udp", "127.0.0.1:0"])

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
