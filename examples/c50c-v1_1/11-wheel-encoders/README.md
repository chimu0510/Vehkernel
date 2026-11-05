# 11-wheel-encoders: Quadrature encoders for motor outputs 1 to 4 and wheel_encoders

## What you learn

- How quadrature encoders are sampled and published across the Vehkernel control and estimation stack ([ADR-0013](../../../docs/adr/0013-control-chain-and-vehicle-models.md), [spec 0006](../../../docs/specs/0006-first-wheel-motion.md), [spec 0013](../../../docs/specs/0013-board-examples.md)):
  - Hardware quadrature decoding: On the STM32F407, timer encoder mode on TIM2 through TIM5 decodes quadrature pulses on dual input pins for motor channels 1 through 4 (connectors U23, U22, U21, U12 from [`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)).
  - Control cycle sampling: [`vk::actuators::ActuatorTask`](../../../modules/actuators/include/vk/actuators/actuator_task.hpp) runs on `wq:ctrl` at the base control rate (200 Hz, 5000 µs interval). On each cycle, it reads each channel's 16-bit encoder counter via [`MotorChannel::encoder->count()`](../../../hal/include/vk/hal/actuator.hpp). [`vk::actuators::ActuatorControl::step()`](../../../modules/actuators/src/actuator_control.cpp) takes 16-bit wrapping differences between consecutive readings (`static_cast<std::int16_t>(counts[i] - wheel.last_count)`), converts deltas to accumulated angle (rad) and angular velocity (rad/s), and publishes [`vk::msg::WheelEncoders`](../../../schema/msg/wheel_encoders.msg) at 200 Hz carrying `angle` and `speed` for all four channels.
  - Why the message carries angle and speed rather than raw counts: odometry, body velocity estimators, and control loops ([ADR-0013](../../../docs/adr/0013-control-chain-and-vehicle-models.md), [ADR-0014](../../../docs/adr/0014-estimation-architecture.md)) operate on standard SI physical units (radians and radians per second).
- Computing signed counts from `angle` and `MOTn_CPR`:
  - The `wheel_encoders` message carries no count field. `main.cpp` computes each wheel's signed count from its accumulated angle and configured counts per turn:
    `count = round(angle / (2 * pi) * MOTn_CPR)`
    reading `vk::params::mot.mot1_cpr` to `mot4_cpr`.
  - One wheel turn is `MOTn_CPR` counts: 36000 on the C50C's stock motors ([`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml) gives the reasoning: 500-line GMR encoders with 1:18 gearboxes under 4x quadrature decoding produce 500 * 4 * 18 = 36000 counts per wheel turn), and 20000 in SITL ([`sim/include/vk/sim/motor.hpp`](../../../sim/include/vk/sim/motor.hpp)).
- Parameter defaults versus stored calibration:
  - This example starts no parameter storage (`example::start_parameters()` is not called; [spec 0013](../../../docs/specs/0013-board-examples.md) specifies that only `09-parameters` starts storage).
  - Therefore, the example runs purely with the parameters' compiled-in defaults rather than board-stored calibrations from the `PARAMS` Flash region.
  - In particular, `MOTn_ENC_REV` (encoder direction reversal) as calibrated and stored for the firmware does not apply. Depending on raw motor wiring and timer phase, a wheel turned forward may count in the negative direction under the default parameters; the lesson demonstrates the raw default behaviour.
- The example drives nothing:
  - In this example, `example::start_actuators()` initializes `ActuatorTask` on `wq:ctrl`, but `main.cpp` never publishes `arming_state` or `actuator_demand`.
  - Because no arming state is asserted, `ActuatorTask` remains continuously disarmed, its duty stays 0, and the output watchdog, started by `example::start_actuators()`, remains satisfied.
  - The wheels are turned by hand on the bench to observe count and speed changes on the display.
  - To see the counts move under motor power, the same `wheel_encoders` topic is published by the firmware under full setpoint arbitration ([ADR-0005](../../../docs/adr/0005-setpoint-arbitration.md)); in SITL, the L3 automated test drives the simulated motors with the test-only option `--wheel-speeds 3,-6,9,-12`. The default duty limit `MOT_OUT_MAX = 0.3` ([`modules/actuators/params.toml`](../../../modules/actuators/params.toml)) times the motor model's full-scale speed of 50.0 rad/s ([`sim/include/vk/sim/motor.hpp`](../../../sim/include/vk/sim/motor.hpp)) limits achievable speed to 15.0 rad/s (0.3 * 50.0 = 15.0 rad/s); demanded speeds are kept within this limit so outputs do not saturate.
  - This example does not combine with [`10-motor-outputs`](../10-motor-outputs/README.md), which is a separate executable image ([ADR-0025](../../../docs/adr/0025-board-examples.md)).
- Display and reporting work item:
  - A single work item on `wq:slow` runs at 4 Hz (250 ms interval). It drains the `wheel_encoders` subscription, keeps the newest message, computes signed counts in floats, reports `encoders_us`, `speed1` through `speed4`, and `count1` through `count4` for test evaluation, and updates the SSD1306 OLED display using stack-allocated `Line` buffers without dynamic allocation ([ADR-0007](../../../docs/adr/0007-cpp20-subset.md)).

## Before this

- [`01-led`](../01-led/README.md): System clock initialization (`vk::hal::SystemClock::now_us()`), GPIO output, and the build and flash workflow.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation ([ADR-0007](../../../docs/adr/0007-cpp20-subset.md)).
- [`04-work-queues`](../04-work-queues/README.md): Rate-tiered FreeRTOS work queues (`wq:slow`), periodic scheduling with `schedule_periodic()`, and cooperative work items.
- [`05-bus`](../05-bus/README.md): Publish/subscribe bus backbone, schema-generated topics, and `vk::core::Subscription<T>`.
- [`09-parameters`](../09-parameters/README.md): Parameter system, compiled-in parameter defaults, and the `vk::params` definitions.
- [`10-motor-outputs`](../10-motor-outputs/README.md): Actuator control on outputs 1 to 4, `ActuatorTask` initialization via `example::start_actuators()`, and output safety.

## Hardware

- **Quadrature encoders A to D**:
  - The C50C V1.1 provides four quadrature encoder inputs on dedicated timer channels ([`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)):
    - Encoder A (output 1): connector U23, pins `PA15` (TIM2_CH1) and `PB3` (TIM2_CH2).
    - Encoder B (output 2): connector U22, pins `PA6` (TIM3_CH1) and `PA7` (TIM3_CH2).
    - Encoder C (output 3): connector U21, pins `PB6` (TIM4_CH1) and `PB7` (TIM4_CH2).
    - Encoder D (output 4): connector U12, pins `PA0` (TIM5_CH1) and `PA1` (TIM5_CH2).
  - The stock motors have 500-line GMR encoders and 1:18 gearboxes: with 4x quadrature decoding, 500 * 4 * 18 = 36000 counts per wheel turn.
- **Turning the wheels by hand**:
  - The example drives nothing: `ActuatorTask` runs disarmed, duty stays 0, and no demand is commanded.
  - Place the vehicle on a stand so that all four wheels rotate freely in the air, and turn each wheel by hand on the bench to observe the encoder response.

## Build and flash

### Building for the board

Build with the `c50c-v1_1-examples` preset, then flash this example's image ([Board examples](../../../docs/guide/examples.md#flashing-an-example)):

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-11-wheel-encoders.apj
```

### Running in SITL

`11-wheel-encoders` also runs on the development host with the `sitl-examples` preset ([Running in SITL](../../../docs/guide/examples.md#running-in-sitl)).

Run the SITL executable directly, specifying run duration, display capture path, and simulated wheel speeds:

```bash
build/sitl-examples/app/examples/example-11-wheel-encoders \
    --duration 6.5 \
    --display /tmp/display.pbm \
    --wheel-speeds 3,-6,9,-12
```

When finished, [return to the firmware](../../../docs/guide/examples.md#returning-to-the-firmware).

## What you see

### On the board

1. **Initial state (Boot)**:
   - When the board boots, `ActuatorTask` starts in the disarmed state.
   - The SSD1306 OLED displays:
     ```text
     VEHKERNEL 11-ENCODERS

     1 +0 +0.00
     2 +0 +0.00
     3 +0 +0.00
     4 +0 +0.00
     ```
   - Duty stays 0 on all motor outputs.
2. **Turning a wheel by hand**:
   - Turn wheel 1 forward by hand:
     - The display updates at 4 Hz.
     - While rotating, the speed column reflects the rotational velocity in rad/s (e.g. `+4.50`).
     - The count column updates with the signed accumulated counts. Rotating wheel 1 one complete revolution changes its count by approximately 36000.
   - Turn wheel 1 backward by hand:
     - The speed column displays a negative value (e.g. `-3.20`), and the count decrements.
   - Repeat for wheels 2, 3, and 4 to verify that each independent quadrature timer channel decodes correctly.
   - Note on direction: Because this example runs with parameter defaults without loading stored calibrations, `MOTn_ENC_REV` is not active; a wheel may count in the opposite direction from what the firmware shows.

### In SITL simulation

When running in SITL with `--wheel-speeds 3,-6,9,-12` for 6.5 seconds:
- **0.0 s to 1.0 s**:
  - The simulated vehicle is disarmed; `speed1` through `speed4` report `0.000` rad/s and `count1` through `count4` report `0`.
- **1.0 s to 4.0 s**:
  - At 1.0 s, SITL scenario automation arms the vehicle and publishes `actuator_demand` with wheel speeds `[+3.0, -6.0, +9.0, -12.0]` rad/s.
  - The motors accelerate to their demanded targets, and `ActuatorTask` publishes `wheel_encoders` at 200 Hz.
  - Between 2.5 s and 4.0 s, the reported speeds reach steady state matching the demanded targets:
    - Wheel 1: about +3.00 rad/s
    - Wheel 2: about -6.00 rad/s
    - Wheel 3: about +9.00 rad/s
    - Wheel 4: about -12.00 rad/s
- **4.0 s to 5.5 s**:
  - At 4.0 s, the vehicle disarms and demand stops. Motor duties drop immediately to 0.0, and the motors coast to a stop with time constant tau = 0.1 s.
- **5.5 s to 6.5 s**:
  - By 5.5 s (15 time constants), wheel speeds are 0.00 rad/s, and encoder counts remain completely static.
  - The final counts reflect the total angle accumulated across the 3.0-second driven window (count = round(speed * 3.0 * 20000 / (2 * pi))):
    - Wheel 1: +28648
    - Wheel 2: -57296
    - Wheel 3: +85944
    - Wheel 4: -114592
- At 6.5 s:
  - SITL writes the final display image to `--display`, outputs `display_on=1`, and terminates.

## The code

The complete source code is in `main.cpp`:

- **Encoder Display Work Item (`EncoderDisplayItem`)**:
  - Inherits `vk::core::WorkItem` and runs periodically on `wq:slow` at 4 Hz (`interval_us = 250'000`).
  - Holds a subscription to `vk::msg::WheelEncoders` (`encoders_sub_`) and stores the newest received message in `last_encoders_`.
  - Reads counts per revolution defaults from `vk::params::mot.mot1_cpr` through `mot4_cpr`.
  - Computes signed counts in floats for each wheel as `std::lround(last_encoders_.angle[i] / (2.0F * std::numbers::pi_v<float>) * cpr[i])`.
  - Emits report lines `encoders_us`, `speed1` through `speed4`, and `count1` through `count4` via `example::report()`.
  - Clears and formats the 21-column SSD1306 `TextScreen` using `Line` buffers without dynamic memory allocation:
    - Line 0: Title `VEHKERNEL 11-ENCODERS`
    - Line 1: Blank separator
    - Lines 2 to 5: Wheel number, signed count via `Line::add_signed()`, and signed speed via `Line::add_fixed()`
  - Draws the text screen onto `display_.frame()` and updates display pages.
- **Static Work Item Instance (`display_item`)**:
  - Defined in an anonymous namespace using `vk::core::StaticInstance` to allocate work item storage statically in BSS without dynamic heap allocation ([ADR-0007](../../../docs/adr/0007-cpp20-subset.md)).
- **Application Entry Point (`main()`)**:
  - Calls `example::init(argc, argv)` to initialize clocking, fault handlers, and platform peripherals.
  - Calls `example::start_actuators()` to initialize timer PWM channels, direction GPIOs, timer encoder interfaces, `ActuatorTask` on `wq:ctrl`, and the output watchdog.
  - Obtains references to `example::display()` and `vk::core::work_queue(WorkQueueTier::Slow)`.
  - Emplaces and starts `display_item` on `wq:slow`.
  - Calls `example::start_scheduler()` to enter the FreeRTOS scheduler.

## Exercises

1. **Calculate revolutions and degrees**: In `EncoderDisplayItem::run()`, calculate revolutions as `turns = last_encoders_.angle[i] / (2.0F * std::numbers::pi_v<float>)` and angular position in degrees as `deg = turns * 360.0F`. Format and display the angle in degrees on the OLED: for example `1 +360.00 +0.00` (15 characters, fitting within the 21 columns). Verify on the bench that turning a wheel through one full mechanical rotation advances the displayed angle by 360 degrees.
2. **Observe encoder resolution**: Compare the sensitivity of the C50C's 36000-count encoder with a simulated motor. Compute the angular resolution in radians per count: 2 * pi / 36000 (about 0.000175 rad/count, or 0.01 deg per count). Turn the wheel by a tiny increment by hand and observe how quickly the count increments.
3. **Inspect report streaming in SITL**: Run the SITL example with custom wheel speeds within the 15 rad/s output limit (e.g. `--wheel-speeds 2,-4,6,-8`) and inspect the 4 Hz output in your terminal. Verify how the reported speeds settle to the commanded values and how counts scale proportionally with speed.

## Read further

- [ADR-0002: Bus backbone and pluggable Links](../../../docs/adr/0002-bus-backbone-and-pluggable-links.md)
- [ADR-0004: FreeRTOS usage](../../../docs/adr/0004-freertos.md)
- [ADR-0005: Setpoint arbitration, arming and output watchdog](../../../docs/adr/0005-setpoint-arbitration.md)
- [ADR-0007: C++20 subset](../../../docs/adr/0007-cpp20-subset.md)
- [ADR-0008: Bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0009: Layered HAL](../../../docs/adr/0009-layered-hal.md)
- [ADR-0013: Control chain and vehicle models](../../../docs/adr/0013-control-chain-and-vehicle-models.md)
- [ADR-0014: Estimation architecture](../../../docs/adr/0014-estimation-architecture.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [Spec 0006: First wheel motion](../../../docs/specs/0006-first-wheel-motion.md)
- [Spec 0013: Board examples](../../../docs/specs/0013-board-examples.md)
- [C50C V1.1 board definition (`board.toml`)](../../../boards/c50c-v1_1/board.toml)
- [Wheel encoders message definition (`wheel_encoders.msg`)](../../../schema/msg/wheel_encoders.msg)
- [Actuator task header (`actuator_task.hpp`)](../../../modules/actuators/include/vk/actuators/actuator_task.hpp)
- [Actuator control implementation (`actuator_control.cpp`)](../../../modules/actuators/src/actuator_control.cpp)
- [Examples guide](../../../docs/guide/examples.md)
- [Contributing guidelines (`CONTRIBUTING.md`)](../../../CONTRIBUTING.md)
