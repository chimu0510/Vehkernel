#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Magnetometer calibration from a recorded ULog file (spec 0012, ADR-0014).

Reads magnetometer_sample and logged CAL_MAG_* parameters from a .ulg file,
undoes the logged calibration, and fits an axis-aligned ellipse to the
horizontal field (hard-iron offsets and soft-iron scales for x and y), keeping z.
With --3d it fits an axis-aligned ellipsoid across all three axes.

Refuses logs covering less than 360 deg of heading or with residual exceeding
a threshold, and outputs new parameters as `link_monitor.py param set` commands
or applies them live over the Link with --apply.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import struct
import sys
from typing import Any, Sequence

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "log"))
sys.path.insert(0, str(REPO / "tools" / "link"))

from vklink.bundle import bundle_root  # noqa: E402
import vkulog  # noqa: E402
from vkulog.reader import DataMessage, ParameterChange  # noqa: E402

# Default maximum residual threshold in µT.
# The AK09916 sensor has a resolution of 0.15 µT (quantization error ~0.043 µT RMS),
# and the SITL simulated magnetometer introduces 0.3 µT uniform noise (~0.173 µT RMS).
# Under normal operation with an axis-aligned elliptical horizontal field, the expected
# RMS residual is around 0.2 to 0.4 µT. A threshold of 2.5 µT (~8-10x sensor noise)
# absorbs typical mechanical vibration and slight non-elliptical disturbances while
# reliably refusing corrupted fields, outlier clouds, or poor fits.
DEFAULT_MAX_RESIDUAL_UT = 2.5

# Minimum required samples for calibration.
MIN_SAMPLES = 100

# Minimum heading coverage required in degrees.
MIN_COVERAGE_DEG = 360.0

# Allowed parameter scale range per params.toml.
MIN_SCALE = 0.5
MAX_SCALE = 2.0

CAL_MAG_PARAM_NAMES = {
    "CAL_MAG_XOFF",
    "CAL_MAG_YOFF",
    "CAL_MAG_ZOFF",
    "CAL_MAG_XSCALE",
    "CAL_MAG_YSCALE",
    "CAL_MAG_ZSCALE",
}

DEFAULT_MSG_DIR = REPO / "build" / "sitl" / "generated" / "schema" / "python"


class CalibrationRefused(Exception):
    """Raised when a log cannot be calibrated (coverage, residual, sample count, etc.)."""


@dataclass(frozen=True)
class FitResult:
    offsets: tuple[float, float, float]  # (x, y, z) in µT
    scales: tuple[float, float, float]   # (x, y, z) scale factors
    semi_axes: tuple[float, ...]         # (a, b) or (a, b, c) in µT
    mean_radius: float                  # r in µT
    coverage_deg: float                 # heading coverage in degrees
    residual_ut: float                  # RMS residual in µT
    residual_fraction: float            # residual as fraction of r
    is_3d: bool                         # True if 3D ellipsoid fit
    kept_z: bool                        # True if 2D fit kept logged z


def solve_linear_system(a_mat: list[list[float]], b_vec: list[float]) -> list[float]:
    """Solves A x = b using Gaussian elimination with partial pivoting."""
    n = len(b_vec)
    aug = [row[:] + [b_vec[i]] for i, row in enumerate(a_mat)]

    for col in range(n):
        max_row = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[max_row][col]) < 1e-12:
            raise ValueError("Singular matrix in least squares fit")
        aug[col], aug[max_row] = aug[max_row], aug[col]

        pivot = aug[col][col]
        for row in range(col + 1, n):
            factor = aug[row][col] / pivot
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]

    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = sum(aug[i][j] * x[j] for j in range(i + 1, n))
        x[i] = (aug[i][n] - s) / aug[i][i]
    return x


def undo_calibration(
    samples: Sequence[tuple[float, float, float]],
    initial_cal: dict[str, float],
) -> list[tuple[float, float, float]]:
    """Undoes logged driver calibration: raw = field / scale + offset."""
    xoff = initial_cal.get("CAL_MAG_XOFF", 0.0)
    yoff = initial_cal.get("CAL_MAG_YOFF", 0.0)
    zoff = initial_cal.get("CAL_MAG_ZOFF", 0.0)
    xscale = initial_cal.get("CAL_MAG_XSCALE", 1.0)
    yscale = initial_cal.get("CAL_MAG_YSCALE", 1.0)
    zscale = initial_cal.get("CAL_MAG_ZSCALE", 1.0)

    raw_samples = []
    for fx, fy, fz in samples:
        rx = fx / xscale + xoff
        ry = fy / yscale + yoff
        rz = fz / zscale + zoff
        raw_samples.append((rx, ry, rz))
    return raw_samples


def fit_ellipse_2d(raw_xy: Sequence[tuple[float, float]]) -> tuple[float, float, float, float, float, float]:
    """Fits x^2 + A y^2 + B x + C y + D = 0 by linear least squares.

    Returns:
        (offset_x, offset_y, scale_x, scale_y, semi_a, semi_b)
    """
    p = [[0.0] * 4 for _ in range(4)]
    q = [0.0] * 4

    for x, y in raw_xy:
        row = [y**2, x, y, 1.0]
        v = -x**2
        for r in range(4):
            q[r] += row[r] * v
            for c in range(4):
                p[r][c] += row[r] * row[c]

    u = solve_linear_system(p, q)
    a_param, b_param, c_param, d_param = u

    if a_param <= 0.0:
        raise ValueError(f"Fitted equation is not an ellipse (A = {a_param:.4e} <= 0)")

    k = (b_param**2) / 4.0 + (c_param**2) / (4.0 * a_param) - d_param
    if k <= 0.0:
        raise ValueError(f"Fitted equation is degenerate (K = {k:.4e} <= 0)")

    x0 = -b_param / 2.0
    y0 = -c_param / (2.0 * a_param)
    semi_a = math.sqrt(k)
    semi_b = math.sqrt(k / a_param)

    r = (semi_a + semi_b) / 2.0
    scale_x = r / semi_a
    scale_y = r / semi_b

    return x0, y0, scale_x, scale_y, semi_a, semi_b


def fit_ellipsoid_3d(
    raw_xyz: Sequence[tuple[float, float, float]]
) -> tuple[float, float, float, float, float, float, float, float, float]:
    """Fits x^2 + A y^2 + B z^2 + C x + D y + E z + F = 0 by linear least squares.

    Returns:
        (offset_x, offset_y, offset_z, scale_x, scale_y, scale_z, semi_a, semi_b, semi_c)
    """
    p = [[0.0] * 6 for _ in range(6)]
    q = [0.0] * 6

    for x, y, z in raw_xyz:
        row = [y**2, z**2, x, y, z, 1.0]
        v = -x**2
        for r in range(6):
            q[r] += row[r] * v
            for c in range(6):
                p[r][c] += row[r] * row[c]

    u = solve_linear_system(p, q)
    a_param, b_param, c_param, d_param, e_param, f_param = u

    if a_param <= 0.0 or b_param <= 0.0:
        raise ValueError(
            f"Fitted equation is not an ellipsoid (A = {a_param:.4e}, B = {b_param:.4e})"
        )

    k = (
        (c_param**2) / 4.0
        + (d_param**2) / (4.0 * a_param)
        + (e_param**2) / (4.0 * b_param)
        - f_param
    )
    if k <= 0.0:
        raise ValueError(f"Fitted equation is degenerate (K = {k:.4e} <= 0)")

    x0 = -c_param / 2.0
    y0 = -d_param / (2.0 * a_param)
    z0 = -e_param / (2.0 * b_param)
    semi_a = math.sqrt(k)
    semi_b = math.sqrt(k / a_param)
    semi_c = math.sqrt(k / b_param)

    r = (semi_a + semi_b + semi_c) / 3.0
    scale_x = r / semi_a
    scale_y = r / semi_b
    scale_z = r / semi_c

    return x0, y0, z0, scale_x, scale_y, scale_z, semi_a, semi_b, semi_c


def compute_coverage(calibrated_xy: Sequence[tuple[float, float]]) -> float:
    """Computes total heading coverage in degrees by unwrapping atan2(y, x) from first to last sample."""
    if not calibrated_xy:
        return 0.0

    thetas = [math.atan2(y, x) for x, y in calibrated_xy]
    unwrapped = [thetas[0]]
    for t in thetas[1:]:
        diff = (t - unwrapped[-1] + math.pi) % (2.0 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + diff)

    total_turn_rad = abs(unwrapped[-1] - unwrapped[0])
    return math.degrees(total_turn_rad)


def compute_residual_2d(calibrated_xy: Sequence[tuple[float, float]], r: float) -> tuple[float, float]:
    """Computes RMS residual of ||(x, y)|| - r in µT and as a fraction of r."""
    if not calibrated_xy or r <= 0.0:
        return 0.0, 0.0
    sq_err_sum = sum((math.hypot(x, y) - r) ** 2 for x, y in calibrated_xy)
    rms_ut = math.sqrt(sq_err_sum / len(calibrated_xy))
    return rms_ut, rms_ut / r


def compute_residual_3d(calibrated_xyz: Sequence[tuple[float, float, float]], r: float) -> tuple[float, float]:
    """Computes RMS residual of ||(x, y, z)|| - r in µT and as a fraction of r."""
    if not calibrated_xyz or r <= 0.0:
        return 0.0, 0.0
    sq_err_sum = sum((math.sqrt(x**2 + y**2 + z**2) - r) ** 2 for x, y, z in calibrated_xyz)
    rms_ut = math.sqrt(sq_err_sum / len(calibrated_xyz))
    return rms_ut, rms_ut / r


def calibrate_magnetometer(
    samples: Sequence[tuple[float, float, float]],
    initial_cal: dict[str, float],
    is_3d: bool = False,
    max_residual: float = DEFAULT_MAX_RESIDUAL_UT,
) -> FitResult:
    """Calibrates magnetometer samples, checking coverage, residual, scale bounds, and sample count."""
    if len(samples) < MIN_SAMPLES:
        raise CalibrationRefused(
            f"Too few samples: got {len(samples)}, minimum required is {MIN_SAMPLES}"
        )

    raw_samples = undo_calibration(samples, initial_cal)

    if not is_3d:
        raw_xy = [(s[0], s[1]) for s in raw_samples]
        try:
            x0, y0, sx, sy, semi_a, semi_b = fit_ellipse_2d(raw_xy)
        except ValueError as err:
            raise CalibrationRefused(f"2D ellipse fit failed: {err}") from err

        for axis_name, scale_val in (("X", sx), ("Y", sy)):
            if not (MIN_SCALE <= scale_val <= MAX_SCALE):
                raise CalibrationRefused(
                    f"New {axis_name} scale factor {scale_val:.4f} is outside allowed range [{MIN_SCALE}, {MAX_SCALE}]"
                )

        # Calibrate horizontal field around fitted centre
        cal_xy = [((rx - x0) * sx, (ry - y0) * sy) for rx, ry in raw_xy]
        coverage_deg = compute_coverage(cal_xy)
        if coverage_deg < MIN_COVERAGE_DEG:
            raise CalibrationRefused(
                f"Heading coverage {coverage_deg:.1f} deg is less than required {MIN_COVERAGE_DEG:.1f} deg"
            )

        mean_radius = (semi_a + semi_b) / 2.0
        res_ut, res_frac = compute_residual_2d(cal_xy, mean_radius)
        if res_ut > max_residual:
            raise CalibrationRefused(
                f"Residual {res_ut:.2f} µT ({res_frac * 100:.1f}% of radius) exceeds threshold {max_residual:.2f} µT"
            )

        # z keeps logged offset and scale
        zoff = initial_cal.get("CAL_MAG_ZOFF", 0.0)
        zscale = initial_cal.get("CAL_MAG_ZSCALE", 1.0)

        return FitResult(
            offsets=(x0, y0, zoff),
            scales=(sx, sy, zscale),
            semi_axes=(semi_a, semi_b),
            mean_radius=mean_radius,
            coverage_deg=coverage_deg,
            residual_ut=res_ut,
            residual_fraction=res_frac,
            is_3d=False,
            kept_z=True,
        )

    # 3D fit
    try:
        x0, y0, z0, sx, sy, sz, semi_a, semi_b, semi_c = fit_ellipsoid_3d(raw_samples)
    except ValueError as err:
        raise CalibrationRefused(f"3D ellipsoid fit failed: {err}") from err

    for axis_name, scale_val in (("X", sx), ("Y", sy), ("Z", sz)):
        if not (MIN_SCALE <= scale_val <= MAX_SCALE):
            raise CalibrationRefused(
                f"New {axis_name} scale factor {scale_val:.4f} is outside allowed range [{MIN_SCALE}, {MAX_SCALE}]"
            )

    cal_xyz = [
        ((rx - x0) * sx, (ry - y0) * sy, (rz - z0) * sz)
        for rx, ry, rz in raw_samples
    ]
    # In 3D, require 360 deg turn about z as well
    coverage_deg = compute_coverage([(cx, cy) for cx, cy, _ in cal_xyz])
    if coverage_deg < MIN_COVERAGE_DEG:
        raise CalibrationRefused(
            f"Heading coverage about z is {coverage_deg:.1f} deg, less than required {MIN_COVERAGE_DEG:.1f} deg"
        )

    mean_radius = (semi_a + semi_b + semi_c) / 3.0
    res_ut, res_frac = compute_residual_3d(cal_xyz, mean_radius)
    if res_ut > max_residual:
        raise CalibrationRefused(
            f"Residual {res_ut:.2f} µT ({res_frac * 100:.1f}% of radius) exceeds threshold {max_residual:.2f} µT"
        )

    return FitResult(
        offsets=(x0, y0, z0),
        scales=(sx, sy, sz),
        semi_axes=(semi_a, semi_b, semi_c),
        mean_radius=mean_radius,
        coverage_deg=coverage_deg,
        residual_ut=res_ut,
        residual_fraction=res_frac,
        is_3d=True,
        kept_z=False,
    )


def read_log_samples(log_path: str | Path) -> tuple[list[tuple[float, float, float]], dict[str, float]]:
    """Reads magnetometer_sample:0 records and initial CAL_MAG_* parameters from a ULog file."""
    try:
        reader = vkulog.ULogReader(log_path)
    except Exception as err:
        raise OSError(f"Could not open ULog file {log_path}: {err}") from err

    # Check for parameter changes during the log
    for change in reader.parameter_changes:
        if change.name in CAL_MAG_PARAM_NAMES:
            raise CalibrationRefused(
                f"Parameter {change.name} changed during the log; calibration samples would mix two calibrations"
            )

    initial_cal = {
        "CAL_MAG_XOFF": float(reader.parameters.get("CAL_MAG_XOFF", 0.0)),
        "CAL_MAG_YOFF": float(reader.parameters.get("CAL_MAG_YOFF", 0.0)),
        "CAL_MAG_ZOFF": float(reader.parameters.get("CAL_MAG_ZOFF", 0.0)),
        "CAL_MAG_XSCALE": float(reader.parameters.get("CAL_MAG_XSCALE", 1.0)),
        "CAL_MAG_YSCALE": float(reader.parameters.get("CAL_MAG_YSCALE", 1.0)),
        "CAL_MAG_ZSCALE": float(reader.parameters.get("CAL_MAG_ZSCALE", 1.0)),
    }

    samples: list[tuple[float, float, float]] = []
    for record in reader.records:
        if isinstance(record, DataMessage):
            sub = reader.subscriptions.get(record.msg_id)
            if sub is not None and sub.message_name == "magnetometer_sample" and sub.multi_id == 0:
                fmt = reader.formats.get("magnetometer_sample")
                if fmt is not None:
                    decoded = fmt.decode(record.data)
                    field = decoded["field"]
                    samples.append((float(field[0]), float(field[1]), float(field[2])))

    return samples, initial_cal


def apply_parameters_over_link(
    result: FitResult,
    serial_port: str | None,
    baud: int,
    udp_target: str | None,
    msg_dir: str | Path,
) -> None:
    """Applies calibrated parameters to the MCU over Serial or UDP Link."""
    from vklink.monitor import LinkMonitor, SerialConnection, UdpConnection, load_messages
    from vklink.session import ParamType, ParamValue

    conn: SerialConnection | UdpConnection
    if serial_port:
        conn = SerialConnection(serial_port, baud)
    elif udp_target:
        host, _, port_str = udp_target.partition(":")
        conn = UdpConnection(host or "127.0.0.1", int(port_str or 15600))
    else:
        raise ValueError("--apply requires --serial <port> or --udp <host:port>")

    try:
        messages = load_messages(msg_dir)
        monitor = LinkMonitor(conn, messages)
        if not monitor.handshake(timeout_s=5.0):
            raise ConnectionError("No handshake with the MCU within 5.0 s")

        params_to_set: list[tuple[str, float]] = [
            ("CAL_MAG_XOFF", result.offsets[0]),
            ("CAL_MAG_YOFF", result.offsets[1]),
            ("CAL_MAG_XSCALE", result.scales[0]),
            ("CAL_MAG_YSCALE", result.scales[1]),
        ]
        if result.is_3d:
            params_to_set.insert(2, ("CAL_MAG_ZOFF", result.offsets[2]))
            params_to_set.append(("CAL_MAG_ZSCALE", result.scales[2]))

        for name, value in params_to_set:
            reply = monitor.set_param(name, ParamType.FLOAT, value, timeout_s=3.0)
            if not isinstance(reply, ParamValue):
                raise RuntimeError(f"Setting {name} failed: {reply}")
            print(f"Set {name} = {reply.value:.4g}")
    finally:
        conn.close()


def print_fit_result(
    result: FitResult,
    serial_port: str | None,
    udp_target: str | None,
) -> None:
    """Prints calibration fit summary and param set commands."""
    mode_str = "3D ellipsoid" if result.is_3d else "2D horizontal ellipse"
    print(f"Fit ({mode_str}):")
    if result.is_3d:
        print(f"  offsets:   x = {result.offsets[0]:.3f} µT, y = {result.offsets[1]:.3f} µT, z = {result.offsets[2]:.3f} µT")
        print(f"  scales:    x = {result.scales[0]:.4f}, y = {result.scales[1]:.4f}, z = {result.scales[2]:.4f}")
        print(
            f"  semi-axes: a = {result.semi_axes[0]:.2f} µT, b = {result.semi_axes[1]:.2f} µT, "
            f"c = {result.semi_axes[2]:.2f} µT, mean radius r = {result.mean_radius:.2f} µT"
        )
    else:
        print(f"  offsets:   x = {result.offsets[0]:.3f} µT, y = {result.offsets[1]:.3f} µT")
        print(f"  scales:    x = {result.scales[0]:.4f}, y = {result.scales[1]:.4f}")
        print(
            f"  semi-axes: a = {result.semi_axes[0]:.2f} µT, b = {result.semi_axes[1]:.2f} µT, "
            f"mean radius r = {result.mean_radius:.2f} µT"
        )
    print(f"  coverage:  {result.coverage_deg:.1f} deg")
    print(f"  residual:  {result.residual_ut:.2f} µT ({result.residual_fraction * 100:.1f}% of r)")

    if result.kept_z:
        print(
            f"Note: kept z offset {result.offsets[2]:.3f} µT and scale {result.scales[2]:.4f} "
            f"(a ground vehicle turning about z does not observe them)."
        )

    # Determine command prefix
    if serial_port:
        target_flag = f"--serial {serial_port}"
    elif udp_target:
        target_flag = f"--udp {udp_target}"
    else:
        target_flag = "--serial <port>"

    print()
    print(f"tools/link/link_monitor.py {target_flag} param set CAL_MAG_XOFF {result.offsets[0]:.3f}")
    print(f"tools/link/link_monitor.py {target_flag} param set CAL_MAG_YOFF {result.offsets[1]:.3f}")
    if result.is_3d:
        print(f"tools/link/link_monitor.py {target_flag} param set CAL_MAG_ZOFF {result.offsets[2]:.3f}")
    print(f"tools/link/link_monitor.py {target_flag} param set CAL_MAG_XSCALE {result.scales[0]:.4f}")
    print(f"tools/link/link_monitor.py {target_flag} param set CAL_MAG_YSCALE {result.scales[1]:.4f}")
    if result.is_3d:
        print(f"tools/link/link_monitor.py {target_flag} param set CAL_MAG_ZSCALE {result.scales[2]:.4f}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("log", help="path to recorded .ulg log file")
    parser.add_argument("--3d", dest="three_d", action="store_true", help="fit an axis-aligned ellipsoid in 3D")
    parser.add_argument(
        "--max-residual",
        type=float,
        default=DEFAULT_MAX_RESIDUAL_UT,
        help=f"maximum allowable RMS residual in µT (default: {DEFAULT_MAX_RESIDUAL_UT})",
    )
    parser.add_argument("--serial", metavar="PORT", help="Link serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=921600, help="serial baud rate (default: 921600)")
    parser.add_argument("--udp", metavar="HOST:PORT", help="Link UDP address (e.g. 127.0.0.1:15600)")
    parser.add_argument("--apply", action="store_true", help="apply fitted parameters to the MCU over the Link")
    bundle = bundle_root(Path(__file__))
    if "VK_MSG_PYTHON_DIR" in os.environ:
        default_msg_dir = os.environ["VK_MSG_PYTHON_DIR"]
    elif bundle is not None:
        default_msg_dir = str(bundle / "generated" / "schema" / "python")
    else:
        default_msg_dir = str(DEFAULT_MSG_DIR)

    parser.add_argument(
        "--msg-dir",
        default=default_msg_dir,
        help="path to directory containing generated vkmsg.py schema",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
    except SystemExit as exit_exc:
        return exit_exc.code if exit_exc.code is not None else 2

    # Exit codes: 0 done, 1 refused, 2 usage or I/O error
    try:
        samples, initial_cal = read_log_samples(args.log)
    except CalibrationRefused as refused:
        print(f"Refused: {refused}", file=sys.stderr)
        return 1
    except (OSError, ValueError) as io_err:
        print(f"error: {io_err}", file=sys.stderr)
        return 2

    try:
        result = calibrate_magnetometer(
            samples,
            initial_cal,
            is_3d=args.three_d,
            max_residual=args.max_residual,
        )
    except CalibrationRefused as refused:
        print(f"Refused: {refused}", file=sys.stderr)
        return 1

    print_fit_result(result, args.serial, args.udp)

    if args.apply:
        try:
            apply_parameters_over_link(result, args.serial, args.baud, args.udp, args.msg_dir)
        except Exception as apply_err:
            print(f"FAIL: could not apply parameters: {apply_err}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
