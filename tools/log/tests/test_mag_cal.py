# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for tools/log/mag_cal.py (spec 0012, ADR-0014)."""

import math
from pathlib import Path
import random
import re
import struct
import sys

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "log"))
sys.path.insert(0, str(REPO / "tools" / "link"))

from mag_cal import (  # noqa: E402
    DEFAULT_MAX_RESIDUAL_UT,
    CalibrationRefused,
    calibrate_magnetometer,
    main,
    read_log_samples,
    undo_calibration,
)
import vkulog  # noqa: E402


def make_magnetometer_payload(
    timestamp_us: int,
    device_instance: int = 0,
    field: tuple[float, float, float] = (30.0, 0.0, 30.0),
    overflows: int = 0,
    failures: int = 0,
) -> bytes:
    # Format: uint64 timestamp, uint8 device_instance, float[3] field, uint32 overflows, uint32 failures
    return struct.pack("<QB3fII", timestamp_us, device_instance, *field, overflows, failures)


def write_test_log(
    path: Path,
    samples: list[tuple[float, float, float]],
    initial_cal: dict[str, float] | None = None,
    parameter_changes: list[tuple[str, float]] | None = None,
) -> None:
    """Helper to write a synthetic ULog file containing magnetometer_sample:0 and parameters."""
    with vkulog.ULogWriter(path, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["magnetometer_sample"])

        if initial_cal is not None:
            for name, val in initial_cal.items():
                w.write_parameter(name, val)

        w.write_add_logged(1, "magnetometer_sample", 0)

        # Write samples at 100 Hz (10 ms interval)
        for i, (fx, fy, fz) in enumerate(samples):
            ts = 1_000_000 + i * 10_000
            w.write_data(1, make_magnetometer_payload(ts, 0, (fx, fy, fz)))

        # Parameter change during the log if requested
        if parameter_changes:
            for name, val in parameter_changes:
                w.write_parameter(name, val)


def test_2d_fit_recovers_known_offset_and_scale(tmp_path: Path):
    """2D ellipse fit recovers known horizontal hard-iron offsets and soft-iron scales under noise.

    Tolerance justification:
    - Uniform noise of amplitude 0.15 µT is added to each axis, matching the AK09916 sensor resolution.
    - With N = 360 samples over 2 full turns (720 deg), the theoretical standard error on center
      offsets is ~0.007 µT, and on scales is ~0.02%.
    - An offset tolerance of 0.5 µT and scale tolerance of 1.0% (0.01) provide a tight, justifiable bound
      that readily absorbs the finite-sample random noise.
    """
    rng = random.Random(42)
    true_x_off = 15.0  # µT
    true_y_off = -10.0  # µT
    true_z_off = 5.0   # µT (unobserved in 2D, kept from log)
    true_x_scale = 1.15
    true_y_scale = 0.88
    earth_r = 30.0  # µT horizontal field

    # Simulated sensor noise amplitude matching sensor LSB resolution (0.15 µT)
    noise_amp = 0.15

    samples = []
    # 2 full turns (720 deg) with 360 samples
    for i in range(360):
        theta = 4.0 * math.pi * i / 360.0
        # True Earth field in body frame
        bx = earth_r * math.cos(theta)
        by = earth_r * math.sin(theta)
        bz = 25.0

        # Physical distortion on sensor: field = (body * sitl_scale) + sitl_offset
        # Driver applies: calibrated = (raw - cal_offset) * cal_scale
        # So when calibrated = body, cal_offset = sitl_offset, and cal_scale scales semi-axes to mean radius.
        raw_x = (bx / true_x_scale) + true_x_off + rng.uniform(-noise_amp, noise_amp)
        raw_y = (by / true_y_scale) + true_y_off + rng.uniform(-noise_amp, noise_amp)
        raw_z = true_z_off + bz + rng.uniform(-noise_amp, noise_amp)

        # Driver logging with default calibration (offset 0, scale 1)
        samples.append((raw_x, raw_y, raw_z))

    log_file = tmp_path / "synthetic_2d.ulg"
    initial_cal = {
        "CAL_MAG_XOFF": 0.0,
        "CAL_MAG_YOFF": 0.0,
        "CAL_MAG_ZOFF": true_z_off,
        "CAL_MAG_XSCALE": 1.0,
        "CAL_MAG_YSCALE": 1.0,
        "CAL_MAG_ZSCALE": 1.0,
    }
    write_test_log(log_file, samples, initial_cal)

    read_samps, read_cal = read_log_samples(log_file)
    result = calibrate_magnetometer(read_samps, read_cal, is_3d=False)

    # Semi-axes in raw data: a = earth_r / true_x_scale, b = earth_r / true_y_scale
    raw_a = earth_r / true_x_scale
    raw_b = earth_r / true_y_scale
    mean_r = (raw_a + raw_b) / 2.0
    expected_x_scale = mean_r / raw_a
    expected_y_scale = mean_r / raw_b

    # Tolerances: 0.5 µT on offsets, 1.0% (0.01) relative on scales
    assert abs(result.offsets[0] - true_x_off) < 0.5
    assert abs(result.offsets[1] - true_y_off) < 0.5
    assert result.offsets[2] == true_z_off  # z kept exactly
    assert abs(result.scales[0] - expected_x_scale) / expected_x_scale < 0.01
    assert abs(result.scales[1] - expected_y_scale) / expected_y_scale < 0.01
    assert result.scales[2] == 1.0  # z scale kept exactly

    assert result.coverage_deg >= 700.0  # ~720 deg coverage
    assert result.residual_ut < DEFAULT_MAX_RESIDUAL_UT
    assert result.kept_z is True


def test_3d_fit_recovers_known_offset_and_scale(tmp_path: Path):
    """3D ellipsoid fit (--3d) recovers known x, y, and z offsets and scales under noise.

    Tolerance justification:
    - Uniform noise of 0.15 µT is added to each axis.
    - With N = 400 samples covering 3D sphere with 720 deg turn about z.
    - Offsets within 0.5 µT and scales within 1.5% absorb the noise with ample margin.
    """
    rng = random.Random(123)
    true_off = (12.0, -8.0, 6.0)
    true_semi = (33.0, 27.0, 30.0)  # a, b, c in µT
    noise_amp = 0.15

    samples = []
    # Grid of spherical coordinates providing full 3D coverage and >360 deg turn about z
    n_theta = 20
    n_phi = 20
    for i in range(n_theta):
        theta = math.pi * (i + 0.5) / n_theta
        for j in range(n_phi):
            phi = 4.0 * math.pi * (i * n_phi + j) / (n_theta * n_phi)  # wraps around z twice
            x = true_off[0] + true_semi[0] * math.sin(theta) * math.cos(phi) + rng.uniform(-noise_amp, noise_amp)
            y = true_off[1] + true_semi[1] * math.sin(theta) * math.sin(phi) + rng.uniform(-noise_amp, noise_amp)
            z = true_off[2] + true_semi[2] * math.cos(theta) + rng.uniform(-noise_amp, noise_amp)
            samples.append((x, y, z))

    log_file = tmp_path / "synthetic_3d.ulg"
    write_test_log(log_file, samples)

    read_samps, read_cal = read_log_samples(log_file)
    result = calibrate_magnetometer(read_samps, read_cal, is_3d=True)

    mean_r = sum(true_semi) / 3.0
    expected_scales = tuple(mean_r / s for s in true_semi)

    for i in range(3):
        assert abs(result.offsets[i] - true_off[i]) < 0.5
        assert abs(result.scales[i] - expected_scales[i]) / expected_scales[i] < 0.015

    assert result.coverage_deg >= 360.0
    assert result.residual_ut < DEFAULT_MAX_RESIDUAL_UT
    assert result.kept_z is False
    assert result.is_3d is True


def test_undo_logged_calibration(tmp_path: Path):
    """The tool undoes non-default initial calibration logged in the file before fitting."""
    # Suppose raw field had true offset 20.0 and scale 0.9.
    # The driver logged samples using an earlier imperfect calibration:
    # CAL_MAG_XOFF = 5.0, CAL_MAG_XSCALE = 1.1.
    initial_cal = {
        "CAL_MAG_XOFF": 5.0,
        "CAL_MAG_YOFF": -3.0,
        "CAL_MAG_ZOFF": 0.0,
        "CAL_MAG_XSCALE": 1.1,
        "CAL_MAG_YSCALE": 0.95,
        "CAL_MAG_ZSCALE": 1.0,
    }
    true_raw_xoff = 20.0
    true_raw_yoff = -15.0
    radius = 30.0

    samples = []
    for i in range(200):
        th = 2.0 * math.pi * i / 100.0  # 2 full turns
        raw_x = true_raw_xoff + radius * math.cos(th)
        raw_y = true_raw_yoff + radius * math.sin(th)
        raw_z = 25.0

        # Logged field produced by driver: (raw - cal_offset) * cal_scale
        log_fx = (raw_x - initial_cal["CAL_MAG_XOFF"]) * initial_cal["CAL_MAG_XSCALE"]
        log_fy = (raw_y - initial_cal["CAL_MAG_YOFF"]) * initial_cal["CAL_MAG_YSCALE"]
        log_fz = (raw_z - initial_cal["CAL_MAG_ZOFF"]) * initial_cal["CAL_MAG_ZSCALE"]
        samples.append((log_fx, log_fy, log_fz))

    log_file = tmp_path / "non_default_cal.ulg"
    write_test_log(log_file, samples, initial_cal)

    read_samps, read_cal = read_log_samples(log_file)
    result = calibrate_magnetometer(read_samps, read_cal, is_3d=False)

    # Recovered offsets must match the true raw offsets, not the logged offsets
    assert abs(result.offsets[0] - true_raw_xoff) < 0.1
    assert abs(result.offsets[1] - true_raw_yoff) < 0.1
    assert abs(result.scales[0] - 1.0) < 0.01
    assert abs(result.scales[1] - 1.0) < 0.01


def test_refuse_insufficient_coverage(tmp_path: Path):
    """A log covering only half a turn (180 deg) is refused with coverage reported."""
    samples = []
    for i in range(150):
        # Only sweep from 0 to pi (180 deg)
        th = math.pi * i / 150.0
        x = 30.0 * math.cos(th)
        y = 30.0 * math.sin(th)
        samples.append((x, y, 20.0))

    log_file = tmp_path / "half_turn.ulg"
    write_test_log(log_file, samples)

    read_samps, read_cal = read_log_samples(log_file)
    with pytest.raises(CalibrationRefused) as excinfo:
        calibrate_magnetometer(read_samps, read_cal, is_3d=False)

    msg = str(excinfo.value)
    assert "Heading coverage" in msg
    assert "less than required" in msg
    # The samples span 149/150 of half a turn, 178.8 deg.
    coverage = float(re.search(r"Heading coverage ([0-9.]+) deg", msg).group(1))
    assert 178.0 < coverage < 180.0


def test_refuse_high_residual_outlier_cloud(tmp_path: Path):
    """A field corrupted by an outlier cloud is refused due to high RMS residual."""
    samples = []
    for i in range(200):
        th = 4.0 * math.pi * i / 200.0
        x = 30.0 * math.cos(th)
        y = 30.0 * math.sin(th)
        # Add high-error outlier cloud between sample 50 and 80 (e.g. 15 µT magnetic pulse)
        if 50 <= i <= 80:
            x += 15.0
            y += 15.0
        samples.append((x, y, 20.0))

    log_file = tmp_path / "outliers.ulg"
    write_test_log(log_file, samples)

    read_samps, read_cal = read_log_samples(log_file)
    with pytest.raises(CalibrationRefused) as excinfo:
        calibrate_magnetometer(read_samps, read_cal, is_3d=False)

    msg = str(excinfo.value)
    assert "Residual" in msg
    assert "exceeds threshold" in msg


def test_refuse_wrong_shape_straight_line(tmp_path: Path):
    """A drive straight in a line does not form an ellipse and is refused."""
    samples = []
    for i in range(150):
        # Sensor stays facing roughly the same heading with slight drift
        x = 25.0 + 0.1 * i
        y = 10.0 + 0.05 * i
        samples.append((x, y, 20.0))

    log_file = tmp_path / "straight_line.ulg"
    write_test_log(log_file, samples)

    read_samps, read_cal = read_log_samples(log_file)
    with pytest.raises(CalibrationRefused):
        calibrate_magnetometer(read_samps, read_cal, is_3d=False)


def test_refuse_parameter_changed_during_log(tmp_path: Path):
    """A log where CAL_MAG_* changes dynamically is refused to avoid mixing calibrations."""
    samples = [(30.0 * math.cos(0.01 * i), 30.0 * math.sin(0.01 * i), 20.0) for i in range(150)]
    log_file = tmp_path / "param_change.ulg"
    write_test_log(
        log_file,
        samples,
        parameter_changes=[("CAL_MAG_XOFF", 12.0)],
    )

    with pytest.raises(CalibrationRefused) as excinfo:
        read_log_samples(log_file)

    msg = str(excinfo.value)
    assert "changed during the log" in msg
    assert "mix two calibrations" in msg


def test_refuse_too_few_samples(tmp_path: Path):
    """Fewer than 100 samples is refused."""
    samples = [(30.0 * math.cos(0.01 * i), 30.0 * math.sin(0.01 * i), 20.0) for i in range(50)]
    log_file = tmp_path / "few_samples.ulg"
    write_test_log(log_file, samples)

    read_samps, read_cal = read_log_samples(log_file)
    with pytest.raises(CalibrationRefused) as excinfo:
        calibrate_magnetometer(read_samps, read_cal)

    assert "Too few samples" in str(excinfo.value)


def test_refuse_scale_out_of_range(tmp_path: Path):
    """An extremely elongated ellipse whose scale falls outside [0.5, 2.0] is refused."""
    samples = []
    for i in range(200):
        th = 4.0 * math.pi * i / 200.0
        # semi_a = 5.0, semi_b = 40.0 => mean r = 22.5 => scale_x = 22.5 / 5.0 = 4.5 > 2.0
        x = 5.0 * math.cos(th)
        y = 40.0 * math.sin(th)
        samples.append((x, y, 20.0))

    with pytest.raises(CalibrationRefused) as excinfo:
        calibrate_magnetometer(samples, {})

    assert "outside allowed range" in str(excinfo.value)


def test_cli_execution_and_exit_codes(tmp_path: Path, capsys):
    """CLI exit codes: 0 for success, 1 for refused, 2 for file/usage error."""
    # 1. Non-existent file => exit code 2
    assert main(["non_existent.ulg"]) == 2

    # 2. Refused log (too few samples) => exit code 1
    short_log = tmp_path / "short.ulg"
    write_test_log(short_log, [(10.0, 10.0, 10.0)] * 10)
    assert main([str(short_log)]) == 1
    captured = capsys.readouterr()
    assert "Refused:" in captured.err

    # 3. Successful calibration => exit code 0 and param set commands printed
    good_samples = []
    for i in range(200):
        th = 4.0 * math.pi * i / 200.0
        good_samples.append((12.0 + 30.0 * math.cos(th), -8.0 + 30.0 * math.sin(th), 20.0))
    good_log = tmp_path / "good.ulg"
    write_test_log(good_log, good_samples)

    assert main([str(good_log), "--serial", "/dev/ttyACM0"]) == 0
    captured = capsys.readouterr()
    assert "Fit (2D horizontal ellipse):" in captured.out
    assert "param set CAL_MAG_XOFF" in captured.out
    assert "--serial /dev/ttyACM0" in captured.out
