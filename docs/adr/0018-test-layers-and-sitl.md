# Six test layers; SITL runs the same RTOS

Tests are split into six layers, from static checks to ROS2 integration. SITL runs the complete firmware on the FreeRTOS POSIX port, so scheduling behaves as on hardware and the whole ROS2 stack can be tested without hardware. The layers:

- **L0 static checks**: `-Werror`, clang-tidy, generator determinism (the same input produces the same output) and board definition validation.
- **L1 unit tests**: GoogleTest + gmock on the development host, as in Betaflight and PX4.
  - Driver tests point `platform::` at a fake peripheral bus containing a register model of the device (ADR-0009).
  - Link decoders are fuzzed with libFuzzer.
  - Parameter storage is tested against a fake Flash that simulates power loss at any point during a write.
- **L2 replay**: re-run estimators on recorded data and compare against reference results. The data is ULog, recorded with the estimator replay log profile (ADR-0019).
- **L3 SITL scenarios**: the complete firmware runs on the development host, driven by scenario scripts (modelled on ArduPilot's `rover.py`), for example driving a square to check odometry error, cutting a Link to check the failsafe action, and RC takeover.
- **L4 hardware tests**: the firmware offers a test mode that runs bring-up cases and reports results over a Link, while a runner on the development host flashes the board and collects results. Test mode is a mode (ADR-0005): the runner is in control and only direct-actuator setpoints are accepted. The first cases:
  - IMU wiring, by reading WHO_AM_I.
  - Revision-ID pins.
  - Motor-to-encoder mapping and direction.
  - PWM frequency, measured by hand with an oscilloscope.
  - Parameter storage.

  Tests that turn the wheels require the wheels to be lifted off the ground first, with output power limited.

  Bring-up cases live in two places. Cases that need no actuators and must work before any Link does, such as IMU wiring on a newly powered board, run in a separate bring-up image that the runner flashes over SWD and whose results it reads back over SWD; that image never configures actuator pins. Cases that move actuators run in the main firmware's test mode, started and reported over a Link, so they are guarded by arming, its checks and the emergency stop (ADR-0005). Motor-to-encoder mapping and direction runs before any closed-loop wheel speed control, because a reversed motor or encoder turns the speed loop into positive feedback. PWM frequency stays a manual measurement, since the MCU cannot observe its own output.
- **L5 ROS2 integration**: SITL + Bridge + Nav2, run with launch_testing.

**SITL**

- SITL uses the official FreeRTOS POSIX port, so work queues, the bus and scheduling behave as on hardware.
- Two kinds of simulator, modelled on PX4's SIH / `gz_bridge` and ArduPilot's `SIM_Rover` / `SIM_JSON`:
  - **Built-in lightweight simulator**: a physics model selected by vehicle type and frame class (surface and underwater thruster vehicles have different dynamics), running in the same process. CI uses it by default.
  - **External simulator interface**: a simulator-agnostic protocol with messages defined in the schema (ADR-0006), for Gazebo, Webots or a custom simulator.
- SITL Links connect to the real Bridge, so the complete ROS2 stack runs without hardware. UDP is the default transport in SITL and CI: it needs no device files or permissions, leaves nothing behind when a process dies, and suits the Bridge and Nav2 running in separate processes or containers (L5). A pty transport is optional, for testing tools against a real serial API (termios, baud rate changes, framing errors) without hardware.

**CI**

On every push:
1. Build every board and check its size budget.
2. L0 and L1.
3. L2, once recorded replay data exists.
4. L3 with the built-in simulator.

L4 moves to a self-hosted runner later; until then it runs manually.
