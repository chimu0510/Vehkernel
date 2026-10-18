# One shared control chain; frame differences live only in the vehicle model

PX4 writes a separate module for each rover type: `rover_differential`, `rover_mecanum` and `rover_ackermann` each carry their own rate, attitude, speed, position and actuator control, the same chain repeated three times. Vehkernel follows ArduPilot Rover instead: every frame shares one control chain, and frame differences appear only in the final vehicle model (allocation).

The control chain, in order:

1. Setpoints enter at the levels of ADR-0003: body velocity enters velocity control and angular rate control; attitude or depth enters attitude/depth control. Angular rate is always closed on the gyro.
2. These controllers output a body motion demand (twist or wrench).
3. The vehicle model allocates the demand to each actuator.
4. Actuator control (wheel speed PI with encoder feedback) drives PWM and servo outputs.

Direct-actuator setpoints skip steps 1 to 3.

**Two-level frames**

- **Frame class**: classified by allocation math, not by vehicle appearance, for example `WHEELED`, `ACKERMANN`, `THRUSTER`. It selects the vehicle model and which layers of the control chain are enabled (the depth layer is decided by the vehicle type).
- **Frame type**: a named geometry within the class, for example `DIFF_2WD`, `SKID_4WD`, `MECANUM_4`, `OMNI_3`, `CUSTOM`.
- **Frame preset**: the full parameter set of one real vehicle configuration: vehicle type + class + type + dimensions + actuator mapping + tuning.

**Vehicle type**

The `VEHICLE_TYPE` parameter describes the environment the vehicle operates in: `GROUND`, `SURFACE` or `UNDERWATER`, independent of the frame. Frame classes follow allocation math and cannot tell surface from underwater, so environment-dependent behaviour is decided by the vehicle type:

- Whether the control chain enables the depth layer.
- The defaults of the timeout failsafe action (ADR-0005) and of magnetometer use (ADR-0014); these remain separate parameters that a frame preset can override.
- The `MAV_TYPE` reported by the MAVLink protocol.

A twin-thruster differential USV, for example, is `SURFACE` + `THRUSTER` + `DIFF_2`; a UUV with the same layout differs only in `UNDERWATER`.

`VEHICLE_TYPE` and `FRAME_CLASS` are `reboot_required`: they are resolved first at boot, and then decide the defaults of other parameters and which vehicle model is selected (ADR-0012).

**Vehicle models**

All vehicle models implement one virtual interface and are selected at boot by frame class. Vehicle models with wheel speed feedback (`WHEELED`, `ACKERMANN`) also publish body velocity measurements; integrating them into odometry is the estimator's job (ADR-0014).

- **`WHEELED`**: differential, skid-steer, mecanum and omni wheels are all linear at the velocity level, so one generic matrix model covers them. Each wheel is described by its position, roller angle and radius, forming the matrix J:
  - For control, `wheel speeds = J·twist`.
  - For body velocity measurements, the pseudo-inverse of the same matrix, `twist = J⁺·wheel speeds`; the least-squares residual goes into the covariance as a slip indicator.
  - On saturation the whole twist is scaled down uniformly, keeping the path curvature.
- **`ACKERMANN`**: nonlinear, a separate model using the bicycle model.
- **`THRUSTER`**: allocates at the force and torque level, `thrust = B⁺·wrench` (like PX4's control allocator). Only the interface is defined for now.

## Considered Options

- **Classifying frames by appearance** (ArduPilot's Rover/Boat): rejected. A boat's allocation depends on its thruster layout; a twin-thruster differential USV is really `THRUSTER` + `DIFF_2`, and surface versus underwater is decided by the vehicle type.
