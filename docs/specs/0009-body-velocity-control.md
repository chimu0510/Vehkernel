# Spec 0009: body velocity control (the WHEELED vehicle model, local odometry and SITL vehicle physics)

**Status**: done. Issue: [#103](https://github.com/chimu0510/Vehkernel/issues/103); tickets #104 to #110. The hardware check in #110 confirmed it on the C50C V1.1 on 2026-10-26, wheels lifted, on WHEELTEC's four-wheel differential swing-arm vehicle (two motors, outputs 1 and 2; its preset is recorded on #110).

This spec is done and no longer edited; later decisions live in the ADRs and the code, which win where they differ.

## Problem Statement

Spec 0006 left the C50C V1.1 driving each wheel at a speed the development host gives it, in `TEST` mode, through direct-actuator setpoints. The firmware does not know it is on a vehicle: it has no vehicle model, so it cannot turn a body motion demand (forward speed and turn rate) into wheel speeds, and it has no odometry, so it cannot say how the vehicle moved. Both are the MCU's job under ADR-0001: local estimation and low-level control. ADR-0013 decides one shared control chain with frame differences only in the vehicle model, and ADR-0014 decides local odometry by dead reckoning from body velocity measurements and the attitude estimator's heading. None of it exists, and SITL has no vehicle to check it against: it simulates motors and encoders, and scripted IMU motion, but no vehicle that moves.

The two milestones after this one need it. The remote control (spec 0010) drives the vehicle by body velocity, and the ROS2 Bridge drives it through `/cmd_vel` and reads `/odom` (ADR-0003). The test vehicle on hand is a two-wheel differential drive on outputs 1 and 2 (#50).

## Solution

Deliver the ninth milestone: **the WHEELED vehicle model for two- and four-wheel differential frames, body velocity setpoints with the angular rate closed on the gyro, body velocity measurements from the wheels, local odometry, and a SITL vehicle that moves**. Afterwards:

- Parameters describe the vehicle: `VEHICLE_TYPE` (`GROUND`), `FRAME_CLASS` (`WHEELED`) and `FRAME_TYPE` (`DIFF_2WD` or `SKID_4WD`), the wheel radius, the track width and which output drives which wheel. The vehicle model is selected at boot from them (ADR-0013).
- A body velocity setpoint (forward speed and yaw rate) arrives over a Link in `TEST` mode, goes through setpoint arbitration, and the control chain turns it into wheel speed targets for actuator control. The yaw rate is closed on the gyro. On saturation the whole twist is scaled down uniformly and the controllers are told, so their integrators do not wind up (#84).
- The vehicle model publishes the body velocity it measures from the wheel speeds, with a covariance that grows with the wheels' disagreement.
- A local odometry estimator dead-reckons from that measurement and the attitude estimator's heading, and publishes `vehicle_local_odometry`.
- SITL simulates a ground vehicle: the simulated motors move it, the simulated IMU feels its rotation, and it publishes the true odometry, so CI checks the control chain and the odometry against ground truth.
- The Link monitor drives by body velocity and shows the odometry.
- On the C50C V1.1, with the wheels lifted, a body velocity setpoint turns the wheels in the right directions at the right speeds.

## User Stories

1. As a vehicle integrator, I want to describe my vehicle with parameters (type, frame class, frame type, wheel radius, track and output mapping), so that one firmware image drives any supported frame on the board (ADR-0009, ADR-0013).
2. As a vehicle integrator, I want a two-wheel differential frame preset, so that the test vehicle runs with no geometry beyond its wheel radius and track.
3. As a vehicle integrator, I want a four-wheel skid-steer preset with the same math, so that the board's four outputs drive a skid-steer vehicle without code changes.
4. As a vehicle integrator, I want to say which output drives which wheel, so that the wiring of a vehicle never needs a firmware change.
5. As a developer on the bench, I want to send a forward speed and a yaw rate in `TEST` mode, so that I can drive the vehicle as a whole rather than wheel by wheel.
6. As a developer on the bench, I want every safety condition of `TEST` mode to hold for body velocity setpoints too (output cap, slew rate, setpoint timeout, idle disarm, emergency stop), so that the new level adds no new way to move unsafely (ADR-0005).
7. As a developer, I want the yaw rate closed on the gyro, so that the vehicle turns at the commanded rate even when wheels slip (ADR-0013).
8. As a developer, I want a twist the wheels cannot reach to be scaled down uniformly, so that the vehicle keeps its path curvature when saturated (ADR-0013).
9. As a developer, I want the controllers to stop integrating on a saturated axis, so that they do not wind up while the vehicle cannot follow (#84).
10. As an estimator developer, I want the vehicle model to publish the body velocity it measures from the wheels, with a covariance, so that odometry and later estimators use it like any other measurement source (ADR-0014).
11. As an estimator developer, I want the measurement's covariance to grow when the wheels disagree with the rigid-body motion, so that slip shows up as uncertainty.
12. As a ROS2 user, I want the MCU to publish local odometry (position, heading, body velocity and their covariances), so that the Bridge can publish `/odom` and TF later without computing anything (ADR-0003).
13. As a ROS2 user, I want the odometry's heading to come from the attitude estimator, never from the wheel speed difference, so that it does not drift with every skid (ADR-0014).
14. As a firmware developer, I want SITL to simulate a ground vehicle that the simulated motors move, so that the control chain is tested end to end without hardware (ADR-0018).
15. As a firmware developer, I want SITL to publish the vehicle's true odometry, so that CI measures the odometry's error, not just its existence.
16. As a firmware developer, I want CI to drive the SITL vehicle along a square and a circle and check where it ends, so that a sign error or a wrong geometry fails the build.
17. As a developer, I want the Link monitor to drive by body velocity and show the odometry live, so that I can check a vehicle without writing a script.
18. As a firmware developer, I want the vehicle model math in the math library with unit tests, so that allocation and its pseudo-inverse are checked on the development host (ADR-0014).
19. As a C50C user, I want a documented hardware check with the wheels lifted, so that directions, speeds and saturation are confirmed on the board before the vehicle drives on the ground in spec 0010 (ADR-0018 L4).

## Implementation Decisions

- **Vehicle description** (ADR-0012, ADR-0013):
  - `VEHICLE_TYPE` (`GROUND`, the only value built now), `FRAME_CLASS` (`WHEELED`) and `FRAME_TYPE` (`DIFF_2WD`, `SKID_4WD`), all `reboot_required` and resolved first at boot, named as ADR-0013 names them (each prefix is its own parameter file, as ADR-0012 requires); a value whose vehicle model is not compiled in (Kconfig `VEHICLE_MODEL_*`) is refused by the parameter generator's enum filtering (ADR-0017).
  - Geometry: the wheel radius and the track width, in metres, `WHL_RADIUS` and `WHL_TRACK`. The frame type sets each wheel's position and drive direction from them; `CUSTOM` geometry is out of scope.
  - Output mapping: which actuator output drives each wheel of the frame type (`WHL_LEFT_OUT` and `WHL_RIGHT_OUT` for `DIFF_2WD`; `SKID_4WD` adds the rear pair, `WHL_REAR_L_OUT` and `WHL_REAR_R_OUT`). Outputs the frame does not use are left uncontrolled, as `nan` leaves them today.
  - Board and frame preset defaults follow ADR-0012's layering. The test vehicle's preset (`DIFF_2WD`, outputs 1 left and 2 right, its measured radius and track) is recorded on the hardware check ticket, not in the board definition: the board does not know the vehicle it is in.
- **Vehicle model interface**: one virtual interface (ADR-0009, runtime choice), selected at boot by `FRAME_CLASS`: allocation from a body motion demand (vx, vy, yaw rate) to wheel speed targets, and the measurement of body velocity from wheel speeds. The math lives in `lib/estimation/` as a non-allocating kinematics model: the wheel matrix J from each wheel's position and radius, `wheel speeds = J·twist`, and `twist = J⁺·wheel speeds` with the least-squares residual (ADR-0013, ADR-0014).
- **Setpoint level**: a body velocity setpoint topic (vx, vy, yaw rate, in the body frame; `vy` ignored by frames that cannot move sideways), inbound over Links like `actuator_setpoint`, accepted by setpoint arbitration in `TEST` from the source in control. **`TEST` accepts body velocity setpoints as well as direct-actuator ones**; ADR-0005 and ADR-0018 are edited in this spec to say so. Every `TEST` safety condition applies unchanged, and a direct-actuator setpoint and a body velocity setpoint never drive at once: the newer level replaces the other until it times out.
- **Control chain** (ADR-0013), on `wq:ctrl` after the attitude estimator:
  - Velocity control: vx and vy pass through to allocation as feedforward; the wheel speed loops of spec 0006 already close each wheel's speed.
  - Angular rate control: a PI loop on the gyro's yaw rate (the attitude estimator's `angular_velocity`, bias removed) with feedforward of the target, so the turn rate holds despite slip. Its gains are `RATE_YAW_P`, `RATE_YAW_I` and `RATE_YAW_FF`; integrators reset on disarm and on the emergency stop, as the wheel speed loops' do.
  - Allocation: the vehicle model turns the demand into wheel speed targets. When any target exceeds `MOT_MAX_SPD`, the whole twist is scaled down by one factor (ADR-0013), and the allocation reports per axis whether it was limited; the controllers freeze their integrators on a limited axis, as ArduPilot's `AP_MotorsUGV` limit flags and PX4's control allocator do (#84). ADR-0013 is edited to record the feedback.
- **Body velocity measurement** (ADR-0014): the vehicle model publishes a body velocity measurement topic with multiple instances (source ID, timestamp, vx, vy, yaw rate, covariance), from the wheel encoders' speeds through J⁺; the residual feeds the covariance. Only the wheel source exists now; velocities over a Link (VIO) come with the Bridge.
- **Local odometry** (ADR-0014): an estimator work item that subscribes to the body velocity measurement and `vehicle_attitude`, dead-reckons position in the local level frame (north-east-down, origin at boot) with the attitude's heading, and publishes `vehicle_local_odometry` (position, heading as a quaternion, body velocity, yaw rate, covariances, estimator ID), first field `timestamp`. The math (the dead-reckoning step and its covariance growth) lives in `lib/estimation/`. A parameter resets nothing; a Link command to reset the origin is out of scope.
- **Schema** (ADR-0006): new topics for the body velocity setpoint (`body_velocity_setpoint`), the control chain's wheel speed targets for actuator control (`actuator_demand`, stamped with the time its setpoint was accepted, so actuator control drives the level accepted last), the body velocity measurement, the allocation status and `vehicle_local_odometry`, plus SITL's `vehicle_local_odometry_groundtruth`; enums such as the frame type in `schema/enum/`. The Link's default topic map sends the odometry at 50 Hz and the measurement at 50 Hz.
- **SITL vehicle** (ADR-0018): a built-in ground vehicle model in `sim/` selected by `VEHICLE_TYPE` and `FRAME_TYPE`: the simulated motors' wheel speeds move a rigid body by the frame's kinematics, with optional wheel slip; the vehicle's yaw rate and accelerations drive the simulated ICM20948 in place of the scripted motion profiles (which stay for the attitude tests); the true pose is published as `vehicle_local_odometry_groundtruth`. SITL gets the frame's wheel radius and track from the same parameters as the firmware.
- **Link monitor**: a `drive-body <vx>,<vy>,<yaw_rate> --duration` command, like `drive`; the live view and `--check` show `vehicle_local_odometry` and the measurement.
- **Rates and costs**: the control chain runs at the base rate (200 Hz on the C50C). The timing monitor reports the new work items; none may overrun on the C50C.

## Testing Decisions

- **Good tests check external behaviour only**: the wheel speed targets and topics the firmware publishes for given setpoints and measurements, where the SITL vehicle ends up, and what the Link monitor reports.
- **Seam A: SITL (L3), the main seam.** New scenarios in `test/sitl/`, run in CI:
  - Forward 1 m at a fixed speed: the true distance and the odometry agree within a few percent.
  - A square of four sides and four 90° turns: the odometry ends within a set distance and heading error of the truth.
  - A circle at constant twist: the true yaw rate holds the commanded rate within a tolerance, also with wheel slip enabled, which shows the gyro loop at work.
  - A twist beyond the wheels' reach: the true path keeps the commanded curvature, and the allocation reports the limited axes.
  - Body velocity setpoints are refused outside `TEST` and from a source not in control, and stop the vehicle on timeout, as direct-actuator setpoints do.
- **Seam B: math library (L1).** J and J⁺ for `DIFF_2WD` and `SKID_4WD`; round trip twist → wheels → twist; the residual is zero for consistent wheels and grows for inconsistent ones; uniform scaling keeps the ratio of vx to yaw rate; the dead-reckoning step for straight, turning and stationary motion.
- **Seam C: modules (L1).** The vehicle model selected by parameters; output mapping; the angular rate PI with frozen integrators on a limited axis and reset on disarm; arbitration of the new setpoint level beside the direct-actuator level.
- **Seam D: generators (L0, L1).** The new schema topics and enums; the parameter enum filtered by the compiled-in vehicle models.
- **Seam E: hardware (L4, manual).** On the C50C V1.1, wheels lifted, the two-motor test vehicle:
  - Measure and record the wheel radius and track; set the `DIFF_2WD` preset.
  - Forward vx: both wheels turn forward at `vx / radius`; reverse turns both backward.
  - Positive yaw rate (z down: a right turn) turns the left wheel forward faster than the right, by the track; with the angular rate loop's P and I set to zero for the lifted test, since a lifted vehicle cannot rotate and the loop would wind up against it; with them restored, the limited-axis report freezes the integrator.
  - A twist beyond `MOT_MAX_SPD` is scaled uniformly; the wheel speed ratio holds.
  - The odometry integrates the wheels' motion while the heading stays still (the board does not rotate); no work item overruns.
  - The emergency stop, the setpoint timeout and the output cap act on body velocity setpoints as on direct-actuator ones.
  Results are recorded on the ticket.
- **Prior art**: `test/sitl/check_actuators.py` and `check_arming.py` for SITL scenarios over the Link; the attitude estimator's SITL checks against `vehicle_attitude_groundtruth`; the motor model in `sim/`.

## Out of Scope

- The remote control, its manual mode, RC takeover and RC failsafe, and modes entered while armed (#82) and requests on the bus (#76): spec 0010, with the USB gamepad.
- Driving on the ground and measuring the odometry's error there: spec 0010's hardware check, once the remote control can take over.
- The ROS2 Bridge, companion mode, `/cmd_vel`, `/odom` and TF, and per-topic schema handling (#83).
- Mecanum and omni wheel presets, `CUSTOM` geometry, the `ACKERMANN` and `THRUSTER` vehicle models, and surface and underwater vehicle types.
- Attitude and depth setpoints, and position control: navigation stays above the MCU (ADR-0001).
- A magnetometer, an EKF, the external estimator and velocities over a Link (ADR-0014).
- Logging (ADR-0019) and the replay test layer (L2).
- The oscilloscope checks of #101.

## Further Notes

- **Why `TEST` takes body velocity now**: without a remote control there is no source but the development host, and ADR-0005 already bounds `TEST` with an output cap, timeouts and the emergency stop. Opening a new mode before the remote control exists would mean deciding its safety without the last line of defence; spec 0010 adds the manual mode with the remote control behind it.
- **Why the lifted check has no gyro loop**: on a lifted vehicle the gyro reads no rotation whatever the wheels do, so a closed yaw rate loop only winds up. The SITL circle scenario, with slip, is where the loop is checked; the ground drive in spec 0010 checks it on the vehicle.
- **Two-wheel test vehicle**: `DIFF_2WD` is the frame on hand; `SKID_4WD` shares the math and costs only its preset, so the board's four outputs are covered by the same code.
