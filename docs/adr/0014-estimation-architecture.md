# Estimation split into replaceable estimator modules and a math component library

Estimation has two levels: replaceable estimator modules that can run side by side, and a math component library that depends on neither the bus nor the HAL. Algorithms can then be compared, back each other up, and be tested on the development host. The MCU does only local estimation (ADR-0001):

- IMU processing: reading with the sampling method of ADR-0015 (per sample or batched FIFO), calibration, rotation by the mounting rotation from the board definition and then by the board orientation parameter, filtering. The mounting rotation is a hardware fact (ADR-0011); how the board sits in the vehicle is not, so it is a parameter. The IMU driver composes both at boot and applies them, so `imu_sample` is always in the body frame and no estimator rotates again. The board orientation is `SENS_BOARD_ROT`, `reboot_required`, numbered as ArduPilot's and PX4's rotation table.
- Attitude estimation.
- Local odometry: dead reckoning from body velocity measurements and the heading from attitude estimation. The heading comes from attitude estimation (gyro integration, fused with the magnetometer when enabled), never from the left/right wheel speed difference.

Outputs include both the calibrated raw data and the fused results; the fused results carry covariances. The per-sample IMU topic carries no noise figures: as ArduPilot (`EK3_GYRO_P_NSE`) and PX4 (`EKF2_GYR_NOISE`) do, sensor noise is a parameter of the estimator that uses it, which keeps `imu_sample` small at high rates, and the Bridge fills the covariances of ROS2's `sensor_msgs/Imu` from parameters (ADR-0003). The ICM20948's DMP is not used: it is closed on-chip firmware with opaque timing and filtering, and provides no covariance. Magnetometer use is a parameter whose default depends on the vehicle type and can be overridden by the frame preset (ADR-0013): `EKF_MAG_TYPE` `AUTO` turns heading fusion off for `GROUND` and is meant to turn it on for `SURFACE` and `UNDERWATER`. The magnetometer is calibrated on the development host from a log (`tools/log/mag_cal.py`), not by a procedure on the MCU, and its calibration parameters are in the body frame, so changing `SENS_BOARD_ROT` needs a new calibration.

**Measurement sources and odometry state are separate**

Odometry is not tied to any one sensor; otherwise the firmware would degrade into wheeled-vehicle firmware. It has two parts:

- **Body velocity measurement**: a standard topic with multiple instances; each message carries a source ID, timestamp and covariance. Sources include vehicle models with wheel speed feedback (ADR-0013), drivers for velocity sensors such as a DVL, and velocities sent by the companion computer over a Link (for example from VIO). Data arriving over a Link must already be in the body frame, with timestamps converted to MCU time by time synchronization (ADR-0002).
- **Local odometry**: an estimator subscribes to body velocity measurements and dead-reckons with the heading from attitude estimation, publishing `vehicle_local_odometry`. Parameters select which sources are used.

When the companion computer's pose is clearly better (for example from VIO), the **external estimator** can be enabled: it converts the pose received over a Link to MCU time and republishes it, and is selected as the primary estimator through parameters like any other estimator. If the companion computer disconnects, health checks fall back to the MCU's own estimator, keeping standalone operation (ADR-0001). With the external estimator enabled, the `odom → base_link` TF is published by either the Bridge or the VIO node, never both (ADR-0003).

**Level 1: estimator modules**

- Each estimator is an independent work item that publishes standard state topics (such as `vehicle_attitude` and `vehicle_local_odometry`) tagged with an estimator ID. The control chain subscribes only to the standard topics.
- A parameter selects the primary estimator. Several estimators may run at once: the selected one publishes to the standard topics and the others to diagnostic topics, for algorithm comparison and as a fallback when the primary estimator becomes unhealthy. For attitude this is `EST_PRIMARY`, `EKF` by default: the primary attitude estimator publishes `vehicle_attitude` and the other one `estimator_attitude` on its slot's instance (0 Mahony, 1 EKF). The automatic fallback is not built yet.

**Level 2: math component library**

Split by mathematical model, under `lib/estimation/`:

- Process models, for example quaternion kinematics and wheel kinematics.
- Measurement models, for example accelerometer, magnetometer, wheel speed and depth sensor.
- Filter algorithms, for example complementary filter, EKF and ESKF.
- Non-allocating math utilities.

An estimator = one algorithm + several models. The library depends on neither the bus nor the HAL, so it can be unit tested and replayed against recorded data on the development host.

The initial implementation covers attitude estimation with a complementary filter (Mahony) + quaternion kinematics + accelerometer model, local odometry by dead reckoning, and two body velocity sources: the `WHEELED` vehicle model and velocities injected over a Link. For the external estimator, only the interface and schema are defined at first. Directories and interfaces follow this structure from the start. Attitude estimation came first, with spec 0004; local odometry and the body velocity sources follow with the `WHEELED` vehicle model.

Spec 0012 adds the second attitude estimator, the primary one by default: an error-state EKF (`filters/attitude_eskf.hpp`) on the attitude and the gyro bias, with the accelerometer as a gravity measurement, the magnetometer as a heading measurement whose Jacobian touches only the yaw (heading fusion, as PX4 EKF2's for ground vehicles), and the gyro at rest as a measurement of its own bias (the zero rate update, as PX4 EKF2's `ZeroGyroUpdate`). It estimates attitude only, not position or velocity, which stay with the ROS2 side (ADR-0001). Its variances come from its covariance, so `vehicle_attitude` and the local odometry built on it carry variances that follow what the filter has seen. Two lessons from implementing it:

- PX4 and ArduPilot give gyro noise as a rate standard deviation per sample; this filter takes noise densities, and their defaults taken as densities made the process noise about 200 times too large.
- On level ground nothing but the zero rate update observes the z gyro bias, so without it the yaw drifts with whatever bias the alignment left; at rest means a still IMU and a body velocity near zero, as driving straight at a constant speed looks still to the IMU alone.

The attitude estimator's alignment time and stillness thresholds are compile-time constants for now; they become parameters when a vehicle must align with motors or pumps running, as ArduPilot's `INS_STILL_THRESH` is (ADR-0012). Its variance models stay internal.

**Restarting the estimators**

A log over the Link starts long after boot, and replay cannot rebuild an estimator's state from what it publishes, so a log holds the estimators' start (spec 0011). On an `estimator_restart` publication from the commander (ADR-0005), the estimators return to their initial state: each attitude estimator returns to aligning and needs the vehicle still for 1 s, publishing nothing while aligning; local odometry clears its position and its dead reckoning, and its origin becomes where the vehicle stands once the attitude estimator runs again. An attitude estimator drops an input sample (IMU, magnetometer, body velocity) whose timestamp is before the restart's, even if the sample reaches it after the restart: a sample is published a little after the time it was taken at, so the order of arrival would otherwise decide whether the last sample before the restart joins the alignment, and a replay, which orders inputs by timestamp, could not follow it.
