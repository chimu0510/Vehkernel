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

**Board Device**:
A device mounted on the board and declared in its board definition with its peripheral bus and mounting rotation, for example the IMU or the display.
_Avoid_: onboard sensor, peripheral

**Port**:
A logical serial port a board definition declares, such as `SERIAL1`, mapped to one UART of that board; its use, such as a Link or an RC receiver, is set by board default and later by parameter.
_Avoid_: COM port, UART number

**Download Circuit**:
A board circuit through which a USB serial adapter's DTR and RTS lines drive BOOT0 and NRST, so the development host enters the system bootloader without touching the board, for example on the C50C's SERIAL2.
_Avoid_: auto-reset, one-key download

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

### Firmware update

**Bootloader**:
The program at the start of Flash that checks and starts the firmware and writes new firmware over the ArduPilot and PX4 bootloader protocol.
_Avoid_: BL, loader

**System Bootloader**:
The STM32's own bootloader in ROM, entered through BOOT0 or by a jump from the firmware, used only to install the bootloader and to rescue a board.
_Avoid_: ROM bootloader, DFU mode

**Board ID**:
The number that tells which board a firmware image is for, numbered as in the list ArduPilot and PX4 share.
_Avoid_: board type, APJ ID

**App Descriptor**:
The record inside a firmware image that gives its board ID, size, CRCs and version, checked by the bootloader before it starts the firmware.
_Avoid_: firmware header

**Boot Window**:
The short wait at power-on and reset during which the bootloader accepts a flashing tool before it starts the firmware; a plain reboot skips it.
_Avoid_: bootloader timeout

**Boot Stub**:
The minimal program in the bootloader region that only jumps to the firmware, used on a board until its bootloader is installed.
_Avoid_: mini bootloader

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
A firmware function, triggered by the remote control or any Link, that forces every actuator output off at once whatever the mode and the source in control, and disarms the vehicle if it stays engaged for 5 s, as ArduPilot's motor emergency stop and PX4's kill switch.
_Avoid_: kill switch, e-stop button

**Arming Check**:
A condition that must hold for an arm request to be accepted. Checks of hardware state can be skipped by bits of `COM_ARM_CHECK`; the others cannot.

**Actuator Power Sense**:
A board-specific input from which the MCU reads whether the actuators' power circuit is closed, for example `PE4` on the C50C; where a board has it, it is a skippable arming check.
_Avoid_: enable switch, emergency stop input

**Failsafe Action**:
The action a vehicle takes automatically when the source in control times out or fails, for example stopping, holding position or surfacing.
_Avoid_: failsafe mode

**Setpoint Arbitration**:
The mechanism by which the MCU decides, from the mode and safety state, whose setpoints are in control.
_Avoid_: command arbitration, priority switching

**Commander**:
The module that holds the mode, arming, the arming checks, the emergency stop and setpoint arbitration, and decides when the vehicle disarms.
_Avoid_: flight manager, state machine

**Source in Control**:
The one setpoint source whose setpoints the current mode accepts: the Link, remote control or Bridge that requested the mode.
_Avoid_: owner, master

**Board Verification**:
The check at boot that the board matches the board revision its firmware image was built for; until it passes, actuators stay disabled.
_Avoid_: self-test, boot check

**Output Watchdog**:
The platform-level guard that turns every actuator output off when actuator control stops refreshing it.
_Avoid_: motor timeout

**Fault Stop**:
The platform's path for a fault, a failed assertion or a stack overflow: it turns every actuator output off by writing registers only, records the fault and stops until the hardware watchdog resets the board.
_Avoid_: panic, crash handler

**Hardware Watchdog**:
The MCU's independent watchdog, refreshed only when the scheduler gets idle, which resets the board when it does not.
_Avoid_: IWDG (except for the STM32 peripheral itself), dog

**Reset Cause**:
What caused the last reset, read by the platform at boot and published on `vehicle_status`; a watchdog or fault reset is an unexpected reset, after which arming is refused until the next deliberate reset.
_Avoid_: boot reason

### Local interface

**Local Display**:
A display on the board that shows the vehicle's status from topics on the bus and hosts the local menu; it never feeds the control chain.
_Avoid_: OSD, screen, HMI

**Local Menu**:
The menu on the local display, operated with the user key, that changes whitelisted parameters only while disarmed; arming is refused while it is open, and it never arms, changes the mode or moves an actuator.
_Avoid_: CMS, settings menu, UI

**User Key**:
A push button on the board that operates the local display and the local menu, for example S3 on the C50C; it is not an emergency stop.
_Avoid_: button, mode key

**Menu Parameter**:
A parameter whose definition sets `menu = true`, so that it appears in the local menu.
_Avoid_: quick setting

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

**Event Log**:
The record of the moments around an event, kept in RAM and written to internal Flash only when the event occurs.
_Avoid_: crash log, black box

### Parameters

**Parameter**:
A setting that can be changed at runtime and survives power loss.
_Avoid_: setting, config

**Parameter Group**:
The set of parameters belonging to one module or driver.
_Avoid_: PG

**Safety Attribute**:
A property of a parameter definition that limits when a change takes effect or is allowed: `reboot_required`, or `locked_when_armed`.

**Parameter Set Hash**:
A 64-bit hash over the parameter metadata file the build writes, reported at the handshake so a tool can tell whether its parameter metadata matches the firmware.
_Avoid_: parameter version

**Board Defaults**:
The parameter, Kconfig, port and timing preset defaults provided by a board definition; a frame preset and the user may override the parameter defaults.

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
The physical or logical channel that carries a Link's bytes, for example UART, CAN or USB, or UDP and a pty in SITL.
_Avoid_: interface, port

**Topic Map**:
The configuration declaring which topics travel over a Link, in which direction and at what rate; inbound topics are accepted only when the map declares them.

**Schema**:
The single source of truth for topics, message types and the enums whose values cross the wire, from which MCU, Bridge and tool code is generated.

**Schema Hash**:
A hash computed from the schema for each message type, used to confirm that both ends of a connection share the same message definitions.

**Bridge**:
The node on the companion computer that turns a Link into ROS2 topics, services and parameters.
_Avoid_: driver node, agent

**Custom Protocol**:
Vehkernel's own Link protocol, with COBS and CRC16 framing, sequence numbers, time synchronization and version negotiation.
_Avoid_: binary protocol, VK protocol

**Handshake**:
The exchange that opens a Link: the protocol version is negotiated and every topic's schema hash is compared.
_Avoid_: connect, login

**Control Message**:
A message of the custom protocol itself rather than a topic, such as the handshake, time sync, parameter access, arming and mode requests, bring-up and reboot; its compatibility follows the protocol version, not the schema hash.
_Avoid_: command frame

**Parameter Service**:
The protocol-independent service through which Links and the local menu list, read and set parameters.
_Avoid_: parameter server

**Reboot Service**:
The protocol-independent service that reboots a disarmed board, or reboots it into its bootloader.
_Avoid_: reset command

### Testing

**Test Layer**:
One of six levels of testing by scope, numbered L0 to L5.

**Bring-up**:
A test that confirms the wiring, mapping and direction of a new board or new wiring.
_Avoid_: power-on test

**Bring-up Case**:
One bring-up test, such as reading the IMU's identity or checking a motor's direction against its encoder.

**Bring-up Image**:
A separate firmware image that runs the bring-up cases needing no actuators and reports over SWD; it never configures actuator outputs.
_Avoid_: test firmware, factory image

**Bring-up Runner**:
The development host tool that flashes the bring-up image over SWD and reads its results back.
_Avoid_: test harness

**Hardware Check**:
The last ticket of a spec, in which a person confirms the milestone on the board and records the readings.
_Avoid_: HIL test, board verification

**SITL**:
Software in the loop: the complete firmware running on the development host on the FreeRTOS POSIX port, with simulated devices in place of the board.
_Avoid_: simulator build, emulator

**Debug Overlay**:
The last of the three Kconfig layers, after the MCU defaults and the board defaults, given as a config fragment at configure time, for example the one that selects the bring-up image.
_Avoid_: product config, frame overlay

**Test Mode**:
A mode in which the test runner on the development host is in control, accepting only direct-actuator setpoints, used for bring-up cases and actuator tests.

### Software layers

**Module Layer**:
The layer holding functional modules such as the commander, estimators, actuator control, Links and the local display, and later the control chain, vehicle models and logging.
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
A platform-specific feature the interface layer offers as a small interface of its own, for example `PwmOutput` or `QuadratureEncoder`, rather than as a general peripheral contract such as a timer.

### Observability

**Timing Monitor**:
The core service that measures every work item's run time, jitter and overruns and publishes them on the bus.
_Avoid_: profiler, scheduler stats

**Link Monitor**:
The development host tool that connects over a Link to show live topics and to send parameter changes and vehicle commands.
_Avoid_: GCS, console
