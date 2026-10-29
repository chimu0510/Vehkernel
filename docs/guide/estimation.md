# Estimation

Vehkernel estimates vehicle attitude and odometry on `wq:ctrl` directly following IMU sensor acquisition (ADR-0014, spec 0012). The MCU performs local estimation only—global localization, SLAM, and path planning belong to ROS2 on the companion computer (ADR-0001).

Two attitude estimators are built into the firmware:
1. **EKF Attitude Estimator**: an error-state (multiplicative) extended Kalman filter estimating orientation (NED quaternion) and 3-axis gyroscope biases, propagating real error covariances, fusing gravity and magnetic heading observations with innovation gating and field strength checks, and learning gyroscope bias during rest via zero rate updates.
2. **Mahony Filter**: a complementary filter with fixed gains (`ATT_KP`, `ATT_KI`), the first attitude estimator (spec 0004), kept beside the EKF for comparison and as the primary estimator on a vehicle where the EKF is not tuned yet.

Both run on every IMU sample. The primary estimator publishes `vehicle_attitude`, which the control chain and the local odometry use; the other one publishes `estimator_attitude` (instance 0 for Mahony, instance 1 for EKF), for comparison live, in a log and in replay. The Link and the `estimator_replay` log profile carry both at 50 Hz.

## Selecting the primary estimator (`EST_PRIMARY`)

The `EST_PRIMARY` parameter chooses which estimator publishes `vehicle_attitude`:

| Value | Name | Description |
|---|---|---|
| `1` | `MAHONY` | Mahony complementary filter |
| `2` | `EKF` | Error-state Kalman filter (default) |

`EST_PRIMARY` requires a reboot to take effect (`reboot_required`). If the selected estimator is not built into the firmware, the other one is primary: SITL prints a line saying so at start, and on a board `estimator_id` on `vehicle_attitude` shows which one it is.

Both estimators publish their identifier in the `estimator_id` field of their attitude message:
- `estimator_id = 1`: Mahony filter
- `estimator_id = 2`: EKF

Local odometry reads `vehicle_attitude` without modification, transparently following whichever estimator is currently primary.

## Heading fusion and vehicle types (`EKF_MAG_TYPE`)

Without heading fusion, the yaw is relative to the heading at alignment (at boot, or after an estimator restart) and drifts with the gyro bias that the zero rate update has not removed. Magnetometer fusion anchors yaw to magnetic north (corrected to true north via `EKF_MAG_DECL`).

Heading fusion is governed by `EKF_MAG_TYPE` (`reboot_required`):

| Value | Name | Description |
|---|---|---|
| `0` | `AUTO` | Resolves at boot from `VEHICLE_TYPE` (default) |
| `1` | `OFF` | Magnetometer heading fusion disabled; yaw is relative to start |
| `2` | `HEADING` | Levelled magnetometer heading fused into EKF yaw state |

When set to `AUTO`, the firmware enables heading fusion depending on vehicle operating domain (ADR-0013):
- **`GROUND`**: resolves to `OFF`. Ground vehicles frequently operate indoors or near steel structures, reinforced concrete, and electric traction motors where magnetic anomalies induce severe yaw distortion.
- **`SURFACE` and `UNDERWATER`**: will resolve to `HEADING` when these vehicle types are added: a vessel usually operates away from structural magnetic disturbances, and nothing else bounds its yaw drift. `GROUND` is the only vehicle type now.

### Declination (`EKF_MAG_DECL`)

Magnetic north differs from true geographic north by the local magnetic declination angle. Because the MCU has no GPS receiver or global World Magnetic Model (WMM) lookup table on board (ADR-0001), the local declination angle in degrees is supplied via the `EKF_MAG_DECL` parameter:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 param set EKF_MAG_DECL 12.4
```

With heading fusion on, the yaw in `vehicle_attitude.q`, and so the heading in `vehicle_local_odometry.q`, is relative to true north. With heading fusion off, it is relative to the heading at alignment.

## Measurement rejection and rest updates

The EKF tests every sensor observation against the filter's own error covariance before fusion:

### Accelerometer gravity rejection

Accelerometer specific force provides tilt correction (roll and pitch). Specific forces differing from standard gravity (9.80665 m/s²) by more than `EKF_ACC_REJ` (default 0.10 g) are dropped without fusion. Measurements within this window are evaluated against a chi-squared innovation gate (`EKF_ACC_GATE`, default 5.0 standard deviations) using observation noise `EKF_ACC_NOISE` (default 2.0 m/s²).

### Magnetometer field strength rejection (`EKF_MAG_REJ`)

During initial 1.0 s still alignment, the EKF records the reference magnetic field strength. During operation, any magnetometer sample whose total field magnitude differs from this reference by more than the fraction `EKF_MAG_REJ` (default 0.20, or 20 %) is immediately rejected before innovation gating. This protects against transient disturbances, such as driving over an iron manhole or near structural steel.

Samples passing field strength validation are levelled using current roll and pitch estimates and tested against `EKF_MAG_GATE` (default 3.0 standard deviations) with heading noise `EKF_MAG_NOISE` (default 0.10 rad). The heading measurement Jacobian touches the yaw error state only, preventing magnetic disturbances from directly inducing artificial roll or pitch tilt.

### Zero rate update (`zero_rate_fused`)

On level ground without bank angle changes, z-axis gyroscope bias is not observable from gravity or wheel odometry. When the vehicle remains completely stationary, the gyroscope directly measures its own bias.

The EKF triggers a zero rate update when:
1. The IMU has been still for at least 0.5 s, with the alignment's thresholds (raw gyro norm below 0.05 rad/s and specific force within 0.05 g of gravity).
2. The latest `body_velocity_measurement:0` (no older than 50 ms) indicates linear speed and yaw rate near zero.

Gyro readings are averaged over 0.2 s and each average is fused as a measurement of the gyro bias. The body velocity condition matters: driving straight at a constant speed looks still to the IMU alone, and a zero rate update then would learn the steering corrections as bias.

## Attitude diagnostic counters

`vehicle_attitude` and `estimator_attitude` provide health counters to verify estimation integrity in real time:

| Field | Meaning |
|---|---|
| `accel_rejected` | Count of accelerometer gravity vectors rejected by `EKF_ACC_REJ` or `EKF_ACC_GATE`. |
| `mag_rejected` | Count of magnetometer samples rejected by field strength check `EKF_MAG_REJ` or `EKF_MAG_GATE`. |
| `mag_fused` | Count of valid magnetometer heading observations fused into the attitude state. |
| `zero_rate_fused` | Count of stationary zero rate updates fused to calibrate gyroscope bias. |
| `mag_missing` | Boolean warning flag set when heading fusion is requested (`EKF_MAG_TYPE=HEADING`), but no magnetometer sample arrived during alignment. The commander reflects this in `vehicle_status.mag_missing`. |

## Magnetometer calibration

The ICM20948 driver publishes calibrated magnetic field in microtesla (µT) in the vehicle body frame on `magnetometer_sample`. Uncalibrated raw measurements are subject to two primary errors:
- **Hard-iron distortion**: permanent vehicle magnetic fields (screws, motor magnets, battery leads) that shift the origin away from zero (`CAL_MAG_XOFF`, `CAL_MAG_YOFF`, `CAL_MAG_ZOFF`).
- **Soft-iron distortion**: ferromagnetic materials that distort the magnetic sphere into an ellipsoid with unequal axis scaling (`CAL_MAG_XSCALE`, `CAL_MAG_YSCALE`, `CAL_MAG_ZSCALE`).

The driver applies calibration per body axis:
$$\text{field} = (\text{raw} - \text{offset}) \times \text{scale}$$

Calibration is performed entirely on the development host using recorded ULog data. The MCU executes no fitting routines.

### Step-by-step calibration procedure

1. **Connect and start recording**:
   Connect the Link monitor to the vehicle and start recording with the `estimator_replay` profile:
   ```bash
   tools/link/link_monitor.py --serial /dev/ttyACM0 record mag_cal.ulg --profile estimator_replay
   ```
2. **Initial stationary alignment**:
   Keep the vehicle completely stationary for at least 1.5 seconds while the estimators align initial tilt and reference field strength.
3. **Turn the vehicle in place**:
   Rotate the vehicle slowly around its vertical axis for two complete revolutions (at least 720° of turn). Keep rotation speed smooth and slow (around 0.5 to 0.6 rad/s, or 30°/s).
4. **Stop recording**:
   Stop the vehicle, keep it still for 1 second, and terminate the recorder with `Ctrl-C`.
5. **Run the calibration tool**:
   Run `tools/log/mag_cal.py` on the recorded file:
   ```bash
   tools/log/mag_cal.py mag_cal.ulg
   ```
   The tool reads the recorded samples, undoes the logged `CAL_MAG_*` parameters, fits an axis-aligned ellipse to the horizontal plane, checks heading coverage (>= 360°) and residual, and prints the fit summary:
   ```text
   Fit (2D horizontal ellipse):
     offsets:   x = 12.350 µT, y = -7.820 µT
     scales:    x = 0.9381, y = 1.0694
     semi-axes: a = 32.14 µT, b = 28.20 µT, mean radius r = 30.17 µT
     coverage:  724.2 deg
     residual:  0.24 µT (0.8% of r)
   Note: kept z offset 0.000 µT and scale 1.0000 (a ground vehicle turning about z does not observe them).

   tools/link/link_monitor.py --serial <port> param set CAL_MAG_XOFF 12.350
   tools/link/link_monitor.py --serial <port> param set CAL_MAG_YOFF -7.820
   tools/link/link_monitor.py --serial <port> param set CAL_MAG_XSCALE 0.9381
   tools/link/link_monitor.py --serial <port> param set CAL_MAG_YSCALE 1.0694
   ```
6. **Apply parameters**:
   Apply the parameters automatically over the Link with `--apply`:
   ```bash
   tools/log/mag_cal.py mag_cal.ulg --apply --serial /dev/ttyACM0
   ```
   For SITL over UDP:
   ```bash
   tools/log/mag_cal.py mag_cal.ulg --apply --udp 127.0.0.1:15600
   ```

### 3D ellipsoid calibration (`--3d`)

For a vehicle that can be turned about every axis (for example a small underwater vehicle turned by hand), pass `--3d` to fit an axis-aligned ellipsoid:

```bash
tools/log/mag_cal.py mag_cal.ulg --3d --apply --serial /dev/ttyACM0
```

This fits and applies offsets and scales for all three axes (`CAL_MAG_XOFF`, `CAL_MAG_YOFF`, `CAL_MAG_ZOFF`, `CAL_MAG_XSCALE`, `CAL_MAG_YSCALE`, `CAL_MAG_ZSCALE`).

### Recalibrating after board rotation (`SENS_BOARD_ROT`)

`CAL_MAG_*` parameters are defined in the **vehicle body frame** (forward, right, down). The driver applies `SENS_BOARD_ROT` before applying calibration offsets and scales.

If the board orientation `SENS_BOARD_ROT` is changed, the physical magnetometer axes project differently onto the vehicle's forward, right, and down axes. The vehicle must be recalibrated after any change to `SENS_BOARD_ROT`.

## Estimation parameters

Parameters are declared in `modules/estimation/params.toml` (`ATT_*`), `modules/estimation/primary/params.toml` (`EST_PRIMARY`), `modules/estimation/ekf/params.toml` (`EKF_*`) and `drivers/cal/params.toml` (`CAL_MAG_*`):

| Parameter | Type | Default | Unit | Description |
|---|---|---|---|---|
| `EST_PRIMARY` | `INT32` | `2` (`EKF`) | - | Primary attitude estimator (`1` Mahony, `2` EKF). Reboot required. |
| `EKF_MAG_TYPE` | `INT32` | `0` (`AUTO`) | - | Heading fusion mode (`0` AUTO, `1` OFF, `2` HEADING). Reboot required. |
| `EKF_MAG_DECL` | `FLOAT` | `0.0` | deg | Local magnetic declination angle from true north. |
| `EKF_MAG_REJ` | `FLOAT` | `0.20` | - | Fraction of magnetic field magnitude change triggering rejection. |
| `EKF_MAG_NOISE` | `FLOAT` | `0.10` | rad | Magnetometer heading innovation noise standard deviation. |
| `EKF_MAG_GATE` | `FLOAT` | `3.0` | SD | Chi-squared innovation gate for magnetometer heading updates. |
| `EKF_ACC_NOISE` | `FLOAT` | `2.0` | m/s² | Accelerometer gravity innovation noise standard deviation. |
| `EKF_ACC_REJ` | `FLOAT` | `0.10` | g | Specific force deviation from gravity triggering rejection. |
| `EKF_ACC_GATE` | `FLOAT` | `5.0` | SD | Chi-squared innovation gate for accelerometer tilt updates. |
| `EKF_GYR_NOISE` | `FLOAT` | `1.1e-3` | rad/s/√Hz | Gyroscope continuous rate noise spectral density. |
| `EKF_GYR_B_NOISE` | `FLOAT` | `7e-5` | rad/s²/√Hz | Gyroscope bias random walk continuous noise spectral density. |
| `ATT_KP` | `FLOAT` | `1.0` | 1/s | Mahony filter proportional attitude correction gain. |
| `ATT_KI` | `FLOAT` | `0.05` | 1/s² | Mahony filter integral gyroscope bias correction gain. |
| `ATT_ACC_REJ` | `FLOAT` | `0.1` | g | Mahony filter gravity specific force rejection threshold. |
| `CAL_MAG_XOFF` | `FLOAT` | `0.0` | µT | Hard-iron offset along body X axis. |
| `CAL_MAG_YOFF` | `FLOAT` | `0.0` | µT | Hard-iron offset along body Y axis. |
| `CAL_MAG_ZOFF` | `FLOAT` | `0.0` | µT | Hard-iron offset along body Z axis. |
| `CAL_MAG_XSCALE` | `FLOAT` | `1.0` | - | Soft-iron scale factor along body X axis (0.5 to 2.0). |
| `CAL_MAG_YSCALE` | `FLOAT` | `1.0` | - | Soft-iron scale factor along body Y axis (0.5 to 2.0). |
| `CAL_MAG_ZSCALE` | `FLOAT` | `1.0` | - | Soft-iron scale factor along body Z axis (0.5 to 2.0). |
