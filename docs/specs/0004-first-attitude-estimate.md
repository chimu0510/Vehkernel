# Spec 0004: first attitude estimate

**Status**: done. Issue: [#30](https://github.com/chimu0510/Vehkernel/issues/30); tickets #31 to #35. The IMU mounting rotation and the estimate were confirmed on the C50C in [#35](https://github.com/chimu0510/Vehkernel/issues/35).

This spec is done and no longer edited; later decisions live in the ADRs and the code, which win where they differ.

## Problem Statement

The third milestone left Vehkernel streaming IMU samples at 200 Hz from the C50C V1.1 over USB and from SITL over UDP, visible live in the Link monitor (#23). The firmware still knows nothing about the vehicle's state: samples are published and sent, but nothing turns them into an attitude. Every later layer needs one. Angular rate control closes on the gyro, attitude and heading control need an attitude, local odometry dead-reckons with the heading from attitude estimation (ADR-0014), and the Bridge publishes it to ROS2 (ADR-0003). ADR-0014 fixes the structure (estimator modules over a math component library) and names the first implementation, a Mahony complementary filter with quaternion kinematics and an accelerometer model, but `lib/estimation/` is empty. Two hardware facts are still open as well: the IMU's mounting rotation on the C50C is `NONE` until measured (the board definition says so), and the CPU cost of estimation on the F407 is unknown.

## Solution

Deliver the fourth milestone: **the first attitude estimate, from IMU samples to a `vehicle_attitude` topic seen live on the development host**. Afterwards:

- `lib/estimation/` holds the first math components: vector and quaternion math, the quaternion kinematics process model, the accelerometer (gravity direction) measurement model, and the Mahony filter. They depend on neither the bus nor the HAL and are unit-tested on the development host.
- An attitude estimator runs as a work item on `wq:ctrl`, woken by each `imu_sample` publication and draining the queue on every run (ADR-0008, ADR-0015). It aligns roll and pitch from the accelerometer at start, estimates the gyro bias while the vehicle is still, then publishes `vehicle_attitude` for every sample.
- SITL's simulated IMU follows a scripted motion with a known true attitude, published as `vehicle_attitude_groundtruth`, so CI checks the estimate against the truth over the Link.
- The Link monitor shows roll, pitch and yaw in degrees.
- On the C50C V1.1 the IMU mounting rotation is measured and recorded in the board definition, and the estimator runs at 200 Hz with no overruns.

## User Stories

1. As an estimator author, I want vector and quaternion math in `lib/estimation/math/` that never allocates, so that estimators share one tested implementation (ADR-0014).
2. As an estimator author, I want quaternion kinematics as a process model, so that the attitude propagates from the gyro the same way in every filter.
3. As an estimator author, I want an accelerometer measurement model predicting the direction of gravity in the body frame from an attitude, so that filters correct roll and pitch from the accelerometer.
4. As an estimator author, I want the Mahony filter as a filter algorithm in `lib/estimation/filters/`, with a proportional term for the attitude correction and an integral term that estimates the gyro bias, so that the first estimator is one algorithm plus models (ADR-0014).
5. As an estimator author, I want the filter to skip the accelerometer correction when the measured specific force differs from gravity by more than a threshold, so that linear acceleration does not tilt the estimate.
6. As a maintainer, I want the math components and the filter tested on the development host against synthetic motion with a known true attitude, so that their behaviour is locked without hardware.
7. As a maintainer, I want the library to depend on neither the bus nor the HAL, enforced by the layer rules, so that it can be replayed against recorded data later (ADR-0009, ADR-0018).
8. As a firmware developer, I want the attitude estimator to run as a work item on `wq:ctrl` woken by `imu_sample`, so that estimation follows the IMU in the event chain of ADR-0008.
9. As a firmware developer, I want the estimator to drain every queued IMU sample on each run, so that no sample is skipped when the work queue runs late (ADR-0015).
10. As a firmware developer, I want the estimator to initialize roll and pitch from the accelerometer and yaw to zero, so that the estimate is right from its first output rather than converging from level.
11. As a firmware developer, I want the gyro bias estimated from the first seconds in which the vehicle is still, so that yaw drift starts small without a stored calibration.
12. As a firmware developer, I want the estimator to report whether it is aligned, so that later consumers (the control chain, arming checks) do not use an attitude before it is valid.
13. As a firmware developer, I want `vehicle_attitude` to carry the attitude quaternion, the bias-corrected angular velocity, the gyro bias, a variance per axis and the estimator ID, so that it follows ADR-0014's standard state topics with covariances.
14. As a firmware developer, I want the filter gains, the accelerometer rejection threshold and the stillness thresholds to be compile-time defaults until the parameter system exists, so that this milestone does not depend on parameters.
15. As a firmware developer, I want the estimator selected in Kconfig (`ESTIMATOR_MAHONY`, which already pulls in `MATH_QUATERNION`), so that the build contains only the estimators it uses (ADR-0017).
16. As a SITL user, I want the simulated IMU to follow a scripted motion (at rest, tilted, rotating about each axis), so that the estimator is exercised in SITL as on a moving vehicle.
17. As a SITL user, I want SITL to publish the true attitude of the simulated IMU as `vehicle_attitude_groundtruth`, so that tools compare the estimate with the truth.
18. As a SITL user, I want to choose the motion on the command line, so that I can watch the estimate in the Link monitor under a known motion.
19. As a Link user, I want `vehicle_attitude` sent over the Link at 50 Hz, and in SITL the ground truth as well, so that the development host sees the attitude live without spending the whole Link on it.
20. As a Link user, I want the Link monitor to show roll, pitch and yaw in degrees for attitude topics, so that I read the attitude at a glance.
21. As a maintainer, I want CI to run SITL through a scripted motion and fail when the roll or pitch error exceeds 2° or the yaw error exceeds 5° (L3), so that the whole estimation path is covered on every push.
22. As a maintainer, I want the estimator's run time in the timing monitor with no overruns in SITL and on the C50C, so that its CPU cost is known.
23. As a C50C user, I want the IMU mounting rotation measured by tilting and turning the board and recorded in the board definition, so that roll, pitch and yaw follow the body frame (x forward, y right, z down).
24. As a C50C user, I want a documented hardware check, tilting the board by known angles and turning it, that records the estimate's response on the ticket, so that the estimator is confirmed on the board.
25. As a new contributor, I want the README to explain how to watch the attitude live from SITL and from the board, so that I can see the estimator work in minutes.

## Implementation Decisions

- **Math library (`lib/estimation/math/`)**: `Vector3` and `Quaternion` value types in `float`, with the operations filters need: products, normalization, rotation of a vector, conversion to and from roll, pitch and yaw (ZYX, aerospace convention), and integration of an angular rate over a time step. Header-only where small; no allocation, no exceptions (ADR-0007). The quaternion maps body to the local level frame (north-east-down, yaw relative to the start without a magnetometer).
- **Models (`lib/estimation/models/`)**: quaternion kinematics, `q_next = q ⊗ exp(ω·dt/2)`, and the accelerometer model, which predicts the specific force direction in the body frame for a given attitude (at rest the accelerometer reads `-g` along the down axis rotated into the body frame).
- **Filter (`lib/estimation/filters/`)**: Mahony with gains `Kp` and `Ki`. The error is the cross product of the measured and predicted gravity directions; `Ki` integrates it into the gyro bias estimate, and `Kp` feeds it back into the rate. The accelerometer correction is skipped when `| |a| - g | > 0.1 g` (default). Defaults: `Kp = 1.0`, `Ki = 0.05` at 200 Hz, revisited at bring-up. The filter takes the time step from successive sample timestamps, not from a fixed rate.
- **Estimator (module layer, `modules/estimation/`)**: the attitude estimator is a work item on `wq:ctrl`, triggered by `imu_sample` instance 0. Each run drains the queue and steps the filter once per sample. States: `Aligning` (collecting still samples), `Running`. Alignment needs 1 s of stillness (gyro norm below 0.05 rad/s and accelerometer norm within 0.05 g of gravity); it sets roll and pitch from the mean accelerometer, yaw to zero and the initial gyro bias to the mean gyro. Movement during alignment restarts it. While aligning nothing is published on `vehicle_attitude`.
- **Topics (schema)**: `vehicle_attitude`: `timestamp`, `timestamp_sample`, `estimator_id`, `q[4]`, `angular_velocity[3]` (bias corrected, body frame), `gyro_bias[3]`, `variance[3]` (roll, pitch, yaw in rad², from a simple documented model: roll and pitch constant once aligned, yaw growing linearly with time since alignment because nothing observes it), and `accel_rejected` (samples whose correction was skipped since boot). `vehicle_attitude_groundtruth`: `timestamp`, `q[4]`, `angular_velocity[3]`, published by SITL only. One estimator exists, so it publishes to the standard topic directly; choosing a primary estimator by parameter comes with the parameter system (ADR-0014).
- **SITL motion**: a motion profile drives the simulated ICM20948: a true attitude and angular rate over time, from which the simulator derives the gyro reading (plus the existing noise and a constant bias) and the specific force (gravity rotated into the body frame, plus linear acceleration where the profile has it). Profiles: `still` (default, level), `tilt` (step to 20° roll, then 20° pitch), `rotate` (turns about each axis in turn at 45°/s), `shake` (still attitude with linear acceleration bursts, to exercise accelerometer rejection). `--motion <profile>` chooses one. The truth is published on `vehicle_attitude_groundtruth` at the IMU rate.
- **Topic map**: the default map adds `vehicle_attitude` at 50 Hz, and `vehicle_attitude_groundtruth` at 50 Hz when present.
- **Link monitor**: messages with a field `q` are shown as roll, pitch and yaw in degrees in the live view and in `--check` summary lines (`attitude name=... roll_deg=... pitch_deg=... yaw_deg=...`).
- **L3 check**: `test/sitl/check_attitude.py` runs SITL with a profile and the Link monitor library, pairs each `vehicle_attitude` with the ground truth nearest in time, and fails when the largest roll or pitch error after alignment exceeds 2°, the yaw error exceeds 5°, the estimator never aligns, or any work item overruns during the check. CI runs it with `tilt`, `rotate` and `shake`.
- **C50C mounting rotation**: measured with the board level, nose up and right side down, and turned clockwise seen from above, from the raw `imu_sample` axes in the Link monitor. The result goes into the board definition's `rotation`, which the ICM20948 driver already applies (ADR-0014); the open question in `board.toml` is closed.
- **Kconfig**: `ESTIMATOR_MAHONY` (exists) gains the dependency on a driver publishing `imu_sample` (`DRIVER_ICM20948`).

## Testing Decisions

- **Good tests check external behaviour only**: filter outputs for given inputs, published topics, the Link monitor's output and SITL's report. Internal state such as the error integral is not asserted directly.
- **Seam A: math library (L1, GoogleTest).** Quaternion identities and conversions against known angles, including the gimbal lock edge; integration of a constant rate over many steps against the closed form; the accelerometer model at known attitudes.
- **Seam B: filter (L1).** Synthetic trajectories with a known true attitude: a static tilt converges within 1° in a few seconds; a constant rotation is tracked; a constant gyro bias is estimated by the integral term within 0.005 rad/s on roll and pitch axes; linear acceleration bursts are rejected and leave the attitude within 2°; uneven time steps are handled.
- **Seam C: estimator work item (L1).** On a fake clock work queue with published `imu_sample` messages: nothing is published while aligning; alignment sets roll and pitch from the accelerometer and the gyro bias from the mean gyro; movement restarts alignment; every queued sample is consumed; `vehicle_attitude` carries the expected fields.
- **Seam D: SITL (L3).** `check_attitude.py` with the `tilt`, `rotate` and `shake` profiles in CI, through the Link.
- **Seam E: hardware (L4, manual).** On the C50C: the mounting rotation measurement, then level, 30° roll, 30° pitch and a 90° turn, read from the Link monitor, with the estimator's timing. Results are recorded on the ticket.
- **Prior art**: the Link unit tests over a fake transport, `test/sitl/check_link.py`, `tools/link/vklink/monitor.py`, and the work queue tests on a fake clock.

## Out of Scope

- Local odometry and body velocity measurements: no source exists yet (no encoders, no vehicle model, no setpoints over a Link).
- The magnetometer (AK09916 inside the ICM20948); yaw is relative to the start and drifts.
- EKF or ESKF, several estimators side by side, diagnostic topics and choosing the primary estimator by parameter.
- The external estimator.
- Accelerometer and gyro calibration that is stored; the gyro bias is estimated at every boot.
- Parameters: gains and thresholds are compile-time defaults.
- L2 replay: it needs recorded ULog data, which needs logging (ADR-0019).
- IMU_EVENT triggering, FIFO batch reads and changing the IMU rate.
- Vehicle physics in SITL beyond the IMU motion profiles, and the external simulator interface.
- The control chain, modes, arming and actuators.

## Further Notes

- **Why attitude before parameters**: the estimator is self-contained, every later layer needs it, and its constants are few; the parameter system is the larger milestone and will then have gains, the Link baud rate and the IMU rotation to carry, which shape its design with real cases.
- **Yaw without a magnetometer** is relative to the start and drifts with the residual gyro bias; the yaw variance grows accordingly so consumers can tell. Ground vehicles indoors often leave the magnetometer off anyway (ADR-0014).
- **CPU budget**: the software I2C read already costs about 10 % of the CPU at 200 Hz (ADR-0015); one Mahony step in single precision on the Cortex-M4 FPU is expected to cost a few microseconds. The timing monitor confirms it on hardware.
- **Frames**: the body frame is x forward, y right, z down; the local level frame is north-east-down with yaw zero at alignment.
