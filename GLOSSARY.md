# Vehkernel

Open-source MCU firmware with ROS2 support that drives many vehicles and frames with one system. The MCU handles real-time estimation, low-level control and safety and can run without a companion computer; global localization and navigation belong to ROS2 applications above it.

## Language

### Vehicles and hardware

**Vehicle**:
The whole machine controlled by the firmware, for example a UGV, USV or UUV.
_Avoid_: car, robot, platform

**Vehicle Type**:
The environment a vehicle operates in: ground, surface or underwater, independent of the frame.
_Avoid_: vehicle kind, vehicle category

**Frame**:
A vehicle's motion configuration, made of a frame class and a frame type.
_Avoid_: car type, chassis type

**Frame Class**:
A category of frames defined by their allocation math, for example WHEELED, ACKERMANN, THRUSTER.
_Avoid_: vehicle class

**Frame Type**:
A named geometry within a frame class, for example MECANUM_4, DIFF_2WD.

**Vehicle Model**:
The math that allocates a body motion demand to each actuator, one per frame class.
_Avoid_: mixer, kinematics solver, kinematic model

**Control Chain**:
The sequence of control layers, shared by every frame, from setpoint to actuator output.
_Avoid_: control loop

**Estimator**:
A module combining one filter algorithm with several math models that publishes standard state topics.
_Avoid_: AHRS, EKF

**Primary Estimator**:
The estimator, selected by parameter, whose output goes to the standard state topics; other estimators running alongside publish only to diagnostic topics.
_Avoid_: main EKF

**Body Velocity Measurement**:
A velocity measurement in the body frame, with its source and uncertainty; the source may be a vehicle model, a velocity sensor or the companion computer.
_Avoid_: wheel odometry, wheel odom

**Local Odometry**:
The local pose and velocity an estimator derives from body velocity measurements and heading, not tied to any one sensor.
_Avoid_: wheel odometry

**External Estimator**:
An estimator whose output is a pose sent down by the companion computer, for example from VIO.
_Avoid_: VIO passthrough

**Body Motion Demand**:
The body-level demand the control chain hands to the vehicle model, such as a twist or a wrench.
_Avoid_: body command

**Actuator**:
A component that turns control output into motion, for example a motor, servo or thruster.

**Actuator Control**:
The last layer of the control chain, which turns each actuator's demand into an output signal.
_Avoid_: motor driver

**Board**:
The circuit board that runs the firmware, for example the C50C.
_Avoid_: mainboard, target

**Board Revision**:
A hardware revision of a board, named as the vendor firmware reports it, for example C50C V1.1.
_Avoid_: V4.1

**Board Definition**:
The TOML file holding a board's hardware facts: pins, peripherals, board devices and Flash layout.
_Avoid_: hwdef, board config

**Mounting Rotation**:
The rotation of a board device's axes relative to the board, a hardware fact declared in the board definition.
_Avoid_: IMU orientation, sensor rotation

**Board Orientation**:
The rotation of the board relative to the vehicle's body frame, set by parameter for each installation.
_Avoid_: board rotation, AHRS orientation

**Alt Config**:
An alternative use of a pin under a particular frame, declared in the board definition.
_Avoid_: AF, pin mux

**MCU Database**:
An MCU's pin, DMA and Flash sector data, against which the generator validates board definitions.
_Avoid_: pinmap, chip table

**Peripheral Bus**:
A hardware bus between the MCU and board devices, such as I2C or SPI.
_Avoid_: bus

**Remote Control**:
The input device an operator uses to drive the vehicle directly, for example a USB gamepad or an SBUS receiver.
_Avoid_: controller

**Ground Station**:
A tool outside the vehicle that monitors or commands it over a Link, for example QGroundControl.
_Avoid_: GCS

**Companion Computer**:
The computer connected to the MCU that runs ROS2, for example a Raspberry Pi 5.
_Avoid_: host, Pi

**Development Host**:
The computer that runs builds, tests, SITL and flashing tools, such as a developer's machine or a CI runner.
_Avoid_: PC, workstation

### Operation

**Standalone**:
The state in which the MCU keeps the vehicle running on its own estimation, control and remote control input, without a companion computer.
_Avoid_: offline mode

**Setpoint**:
A control target sent to the MCU, at one of three levels: body velocity, attitude/depth or direct actuator.
_Avoid_: command

**Mode**:
The operating state that decides which source is in control and which setpoint levels it may use. The vehicle boots in `STANDBY`, where no source is in control.

**Arming**:
The state that decides whether actuators may produce output, either armed or disarmed.
_Avoid_: unlock, lock

**Emergency Stop**:
A switch that cuts the actuators' power circuit in hardware, independent of the MCU, for example the switch on the C50C's connector CN6. The MCU sees it only through actuator power sense, on boards that have it.
_Avoid_: kill switch

**Actuator Power Sense**:
A board input from which the MCU reads whether the actuators' power circuit is closed, for example `PE4` on the C50C.
_Avoid_: enable switch, emergency stop input

**Arming Check**:
A condition that must hold for an arm request to be accepted. Checks of hardware state, such as actuator power, can be skipped with `ARM_CHECK`; checks that guard the arming logic cannot.

**Failsafe Action**:
The action a vehicle takes automatically when the source in control times out or fails, for example stopping, holding position or surfacing.
_Avoid_: failsafe mode

**Setpoint Arbitration**:
The mechanism by which the MCU decides, from the mode and safety state, whose setpoints are in control.
_Avoid_: command arbitration, priority switching

### Logging

**Log Sink**:
A destination for log data, for example an SD card, SPI NOR Flash or a stream to the companion computer.
_Avoid_: logger device

**Log Profile**:
A set of topics to log, each with its own rate; profiles can be combined.
_Avoid_: log level

**Batch Message**:
A topic that packs several consecutive samples into one message, for high-rate data.
_Avoid_: FIFO topic

### Parameters

**Parameter**:
A setting that can be changed at runtime and survives power loss.
_Avoid_: setting, config

**Parameter Group**:
The set of parameters belonging to one module or driver.
_Avoid_: PG

**Board Defaults**:
The parameter and Kconfig defaults provided by a board definition.

**Timing Preset**:
A set of defaults for the trigger source, base rate, dividers and sampling method.
_Avoid_: loop rate

**Frame Preset**:
The parameter defaults for one real vehicle configuration, including its vehicle type, frame and dimensions.
_Avoid_: airframe

### Communication

**Bus**:
The publish/subscribe mechanism through which modules inside the MCU exchange data; the firmware's backbone.
_Avoid_: uORB, message queue

**Topic**:
A typed, named data stream on the bus, which may have several instances.

**Work Queue**:
A rate-tiered unit of execution that runs several work items.
_Avoid_: thread

**Work Item**:
A module attached to a work queue and triggered by a timer or a topic publication.
_Avoid_: task, job

**Link**:
A communication connection between the MCU and an external system, made of a topic map, a protocol and a transport.
_Avoid_: channel, interface

**Protocol**:
The part of a Link that encodes topics into a specific protocol, for example the custom protocol, XRCE-DDS or MAVLink.
_Avoid_: driver

**Transport**:
The physical or logical channel that carries a Link's bytes, for example UART, CAN or USB.
_Avoid_: interface, port

**Topic Map**:
The configuration declaring which topics travel over a Link, in which direction and at what rate; inbound topics are accepted only when the map declares them.

**Schema**:
The single source of truth for topics and message types, from which MCU, Bridge and tool code is generated.

**Schema Hash**:
A hash computed from the schema for each message type, used to confirm that both ends of a connection share the same message definitions.

**Bridge**:
The node on the companion computer that turns a Link into ROS2 topics, services and parameters.
_Avoid_: driver node, agent

### Testing

**Test Layer**:
One of six levels of testing by scope, numbered L0 to L5.

**Bring-up**:
A test that confirms the wiring, mapping and direction of a new board or new wiring.
_Avoid_: power-on test

**Test Mode**:
A mode in which the test runner on the development host is in control, used to run bring-up cases.

### Software layers

**Module Layer**:
The layer holding functional modules such as estimators, the control chain, vehicle models, Links and logging.
_Avoid_: L3, application layer

**Driver Layer**:
The layer holding drivers for board and external devices.
_Avoid_: L2

**Interface Layer**:
The layer that defines the HAL contracts.
_Avoid_: L1

**Platform Layer**:
The implementation of the interface layer for a specific MCU or for SITL.
_Avoid_: L0, BSP

**Capability**:
A platform-specific interface the interface layer offers as a concrete function, for example PWM output or a quadrature encoder.
