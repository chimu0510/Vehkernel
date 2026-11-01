# 10-motor-outputs: Actuator control on outputs 1 to 4, output watchdog, and actuator power sense

## What you learn

- How actuator control operates as the final execution stage of the Vehkernel control stack ([ADR-0013](../../../docs/adr/0013-control-chain-and-vehicle-models.md), [spec 0006](../../../docs/specs/0006-first-wheel-motion.md)):
  - Motor outputs: drives four independent DC motor H-bridges via timer PWM and dual direction GPIO lines (`OUTPUT1` to `OUTPUT4`). On the STM32F407, TIM8 provides 4-channel PWM at `MOT_PWM_FREQ` (10 kHz default), paired with quadrature encoders on TIM2 through TIM5 ([`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)).
  - Wheel speed PI control loops: [`vk::actuators::ActuatorControl`](../../../modules/actuators/include/vk/actuators/actuator_control.hpp) closes an independent speed loop per wheel based on encoder pulse rate differences. Targets are clamped to `MOT_MAX_SPD` (30.0 rad/s default). The duty is computed as `MOT_SPD_FF * target + MOT_SPD_P * error + integral` and clamped to `MOT_OUT_MAX` (0.3 default), with anti-windup clamping to prevent integrator saturation.
  - Slew rate limiting: `MOT_SLEWRATE` (default 2.0 1/s, full duty change in 0.5 s) bounds the rate of change of output duty to protect gearboxes and limit current surges. Stopping (transition to zero or disarmed) is never slew-limited and takes effect immediately.
  - Work item execution: [`vk::actuators::ActuatorTask`](../../../modules/actuators/include/vk/actuators/actuator_task.hpp) runs periodically on `wq:ctrl` at the base control rate (200 Hz, 5000 µs interval).
- The three-tier output safety architecture ([ADR-0005](../../../docs/adr/0005-setpoint-arbitration.md)):
  - **Output watchdog** ([`vk::hal::OutputWatchdog`](../../../hal/include/vk/hal/output_watchdog.hpp)): runs on a dedicated high-priority hardware timer interrupt (TIM6 on STM32F4, at interrupt priority 2 above FreeRTOS) or a high-priority task in SITL. If `ActuatorTask` stalls and fails to refresh outputs within 3 base control periods (15 ms), the output watchdog trips and shuts down all motor channels directly via hardware registers without using FreeRTOS or the bus.
  - **Fault stop**: unhandled exceptions, HardFaults, failed assertions, or FreeRTOS stack overflows force the timer Main Output Enable (MOE) clear and all direction lines low before halting the core or resetting via the hardware watchdog (IWDG).
  - **Actuator power sense**: physical sensing of the actuator supply circuit on `PE4` (`ACT_POWER_SENSE`, active high). Arming requires power to read continuously present for 0.5 s; losing power while armed disarms in that very cycle to prevent motor runaway or integrator windup against unpowered wheels.
- Demand conversion and safety cap:
  - This example commands wheel speeds through `vk::msg::ActuatorDemand` ([`actuator_demand.msg`](../../../schema/msg/actuator_demand.msg)). With the feedforward gain `MOT_SPD_FF = 0.02 s/rad` matching the full scale speed of 50.0 rad/s (`1.0 / 0.02 = 50.0`), commanded duty is converted directly to target wheel speed: `target_speed = duty * 50.0F`.
  - Clamping only the speed target is not enough: the speed loop's PI term can drive the duty higher (up to `MOT_OUT_MAX`, default 0.3). The duty limit enforced by actuator control is `MOT_OUT_MAX`, read from `vk::params::mot.out_max` on every run (`modules/actuators/src/actuator_task.cpp`).
  - Before the actuator task starts, `main.cpp` lowers `vk::params::mot.out_max` in RAM to `std::min(vk::params::mot.out_max, max_duty)`, with `max_duty = 0.2F` as the named constant, ensuring the speed loop's own clamp bounds the applied duty to 20 % in either direction. Commanded step duty in `main.cpp` is clamped to the same constant.
  - **Non-persistent RAM parameters**: This RAM value is never stored in Flash because the example starts no parameter storage (`example::start_parameters()` is not called). For the same reason, the example runs with the parameters' compiled-in defaults, not the calibrated values stored on the board (e.g. motor reversal, encoder counts, speed loop gains), so a motor may turn the other way than under the firmware; the duty cap holds either way.
- Why this example feeds the actuator task directly in place of the commander, and why the firmware never does that:
  - In production firmware ([ADR-0005](../../../docs/adr/0005-setpoint-arbitration.md)), all arming checks, mode management, and setpoint arbitration are strictly centralized in `CommanderTask`. Setpoints from Links or remote control are validated against the current mode and active control source; actuators only move in `TEST` or `MANUAL` mode; and arming is refused unless every check passes (board verified, emergency stop released, mode valid, actuator power present, zero/NaN setpoints). Only `CommanderTask` ever publishes `arming_state` ([`arming_state.msg`](../../../schema/msg/arming_state.msg)).
  - `ActuatorTask` requires an armed state (`armed = true` in `ArmingState`) to enable its speed loops and drive PWM outputs. In this example, `main.cpp` feeds `ActuatorTask` directly without instantiating the commander module, mode state machine, or Link communications. To satisfy `ActuatorTask`'s safety interlock, `main.cpp` publishes `ArmingState` with `armed = true` while a non-zero demand step is active, and `armed = false` when demand stops or power is cut. The firmware never does this: allowing arbitrary modules to claim armed state would bypass setpoint arbitration, mode validation, and safety interlocks.
- What stops the outputs when demand stops:
  - In this example, the outputs stop because the example publishes `armed = false` (both on the step back to 0 and when actuator power is cut).
  - As verified by the source code (`actuator_task.cpp` and `output_watchdog.hpp`), `ActuatorTask` has no internal demand timeout—in production firmware, setpoint timeout (`COM_SP_TIMEOUT = 0.5 s`) is strictly enforced by `CommanderTask`, which disarms the vehicle.
  - Furthermore, because `ActuatorTask` runs periodically at 200 Hz on `wq:ctrl` and refreshes the output watchdog on every cycle, the output watchdog acts only if `wq:ctrl` stalls or crashes; it does not trip when setpoint publications merely cease.
  - If demand were left published while armed, `ActuatorTask` would keep driving the motors indefinitely. This is one key reason why production firmware never permits application modules to publish `arming_state` directly.
- Monitoring actuator state:
  - Subscribing to `vk::msg::ActuatorOutputs` ([`actuator_outputs.msg`](../../../schema/msg/actuator_outputs.msg)) published by `ActuatorTask` at 200 Hz to display applied duty on the OLED and emit SITL report lines at 4 Hz.

## Before this

- [`01-led`](../01-led/README.md): System clock initialization (`vk::hal::SystemClock::now_us()`), GPIO output, and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line with `Debouncer`.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation ([ADR-0007](../../../docs/adr/0007-cpp20-subset.md)).
- [`04-work-queues`](../04-work-queues/README.md): Rate-tiered FreeRTOS work queues (`wq:ctrl`, `wq:slow`), periodic scheduling with `schedule_periodic()`, and cooperative scheduling.
- [`05-bus`](../05-bus/README.md): Publish/subscribe bus backbone, schema-generated topics, `vk::core::Publication<T>`, `vk::core::Subscription<T>`.

## Hardware

> [!WARNING]
> Moving wheels can cause injury or damage. Follow the safety precautions of [`CONTRIBUTING.md`](../../../CONTRIBUTING.md): **"Changes that move actuators also need the hardware check described there, done with the wheels off the ground and output power limited."**

- **Precaution 1: Wheels off the ground**:
  - Before connecting drive power or running this example, lift the vehicle so that all four wheels are elevated and spin completely freely in the air without touching the bench, floor, or any obstacles.
  - Never place the vehicle on the ground or bench surface while this example is running.
- **Precaution 2: Output power limited**:
  - Power the motor drive circuit from a bench power supply with current limiting enabled (e.g. capped at 0.5 A to 1.0 A), or use a low-voltage battery during testing.
- **Precaution 3: Cutting the drive power**:
  - The drive power reaches the motor drivers through connector CN6, and the board senses it on `PE4` (`ACT_POWER_SENSE`): high while the drive supply is connected, low when it is not ([C50C V1.1](../../../docs/boards/c50c-v1_1.md#actuator-power-sense)). Keep the bench supply's output switch, or the drive supply's connector, within reach, so you can cut the drive power by hand at any time; the example then stops every output.
- **Board wiring**:
  - Outputs 1 to 4 are the board's four motor drivers with their encoders; their pins are in [`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml) (`actuators` and the `MOTOR_*` and `ENC_*` pins) and the connectors in the [board page](../../../docs/boards/c50c-v1_1.md). This example drives output 1 only.

## Build and flash

### Building for the board

Build the example using the `c50c-v1_1-examples` preset inside the development container:

```bash
cmake --preset c50c-v1_1-examples
cmake --build --preset c50c-v1_1-examples
```

Flash the compiled example (`.apj`) over the one-key download port (`SERIAL2`) using the serial flasher tool:

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-10-motor-outputs.apj
```

Because this example runs no Link and no commander, `flash_serial.py update` restarts the board through SERIAL2's hardware auto-download circuit (pulsing NRST with BOOT0 asserted).

### Running in SITL

`10-motor-outputs` compiles and runs on the development host using the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

Run the SITL executable directly, specifying run duration, simulated key presses, and a power cut time:

```bash
build/sitl-examples/app/examples/example-10-motor-outputs \
    --duration 9.5 \
    --display /tmp/display.pbm \
    --key-presses 1.0,2.2,3.4,4.6,5.8,7.0 \
    --power-cut-at 8.3
```

To run the automated L3 scenario test in CI:

```bash
ctest --preset sitl-examples -R example_10_motor_outputs
```

### Returning to the firmware

When finished testing, restore the standard vehicle firmware:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

The bootloader preserves the non-volatile parameter storage (`PARAMS`), keeping all sensor calibrations intact.

## What you see

### On the board

1. **Initial state (Boot)**:
   - With drive power switched on at CN6, the OLED displays:
     ```text
     VEHKERNEL 10-MOTORS

     STEP: 0
     DEMAND: +0.00
     OUTPUT 1: +0.00
     POWER: ON
     ```
   - Motor outputs are disarmed (`duty = 0.00`); all wheels remain stationary.
2. **Pressing user key S3**:
   - **Press 1 (Step 1)**: Output 1 spins forward at +10 % duty (`DEMAND: +0.10`, `OUTPUT 1: +0.10`). Wheels 2, 3, and 4 stay at zero.
   - **Press 2 (Step 2)**: Output 1 accelerates forward to +20 % duty (`DEMAND: +0.20`, `OUTPUT 1: +0.20`), reaching the safety cap.
   - **Press 3 (Step 3)**: Output 1 slows down, reverses direction smoothly following the slew rate, and spins backward at -10 % duty (`DEMAND: -0.10`, `OUTPUT 1: -0.10`).
   - **Press 4 (Step 4)**: Output 1 accelerates backward to -20 % duty (`DEMAND: -0.20`, `OUTPUT 1: -0.20`), reaching the safety cap in reverse.
   - **Press 5 (Step 0)**: Returns to Step 0 (0 % duty, `DEMAND: +0.00`). The example publishes `armed = false`; output 1 immediately coasts to a complete stop (`OUTPUT 1: +0.00`).
   - **Press 6 (Step 1)**: Re-advances to Step 1 (+10 % duty).
3. **Cutting actuator power**:
   - If connector CN6 is unplugged or switched off while motor 1 is spinning:
   - `PE4` senses the power loss within 5 ms on `wq:ctrl`.
   - The example immediately publishes `armed = false`, resets its step counter to 0, and stops publishing demand.
   - The OLED updates to `POWER: OFF` and `OUTPUT 1: +0.00`.
   - Motor 1 coasts to a halt. While power reads `OFF`, pressing user key S3 will not advance steps or command any output.

> [!IMPORTANT]
> In this example, the outputs stop because the example publishes `armed = false` (both on the step back to 0 and on a power cut). The hardware output watchdog acts only if `wq:ctrl` stops refreshing it. Because `ActuatorTask` has no internal setpoint timeout, a demand left published would keep driving indefinitely without stopping—one fundamental reason why production firmware strictly centralizes all arming decisions and setpoint timeouts in `CommanderTask`.

### In SITL simulation

When running with `--key-presses 1.0,2.2,3.4,4.6,5.8,7.0` and `--power-cut-at 8.3`, standard output reports:
- `demand=0.000` at start.
- `out1=0.000`, `out2=0.000`, `out3=0.000`, `out4=0.000`, `power=1` at 4 Hz.
- At 1.0 s: key press 1 advances to `demand=0.100`. `out1` settles to `0.100`.
- At 2.2 s: key press 2 advances to `demand=0.200`. `out1` settles to `0.200` (capped at 0.200).
- At 3.4 s: key press 3 advances to `demand=-0.100`. `out1` settles to `-0.100`.
- At 4.6 s: key press 4 advances to `demand=-0.200`. `out1` settles to `-0.200` (capped at -0.200).
- At 5.8 s: key press 5 wraps to `demand=0.000` (demand stopped). The example disarms (`armed = false`); `out1` returns to `0.000`.
- At 7.0 s: key press 6 advances to `demand=0.100`. `out1` settles to `0.100` and is held.
- At 8.3 s: power cut triggers while the non-zero demand is held. The example disarms, reports `demand=0.000`, and standard output reports `power=0` and `out1=0.000`.
- At 9.5 s: SITL terminates and reports `display_on=1`.

## The code

The complete source code is in `main.cpp`:

- **Constants and Safety Cap (`max_duty`, `duty_steps`, `vk::params::mot.out_max`)**:
  - `max_duty = 0.2F`: Clamps commanded step duty in `main.cpp`, and lowers `vk::params::mot.out_max` in RAM before starting `ActuatorTask` to `std::min(vk::params::mot.out_max, max_duty)`. The speed loop in `ActuatorControl` clamps duty to `MOT_OUT_MAX` on every run, ensuring PI output never exceeds 20 % in magnitude in either direction.
  - **Non-persistent parameters**: `example::start_parameters()` is not called, so nothing writes to Flash; the example runs purely with compiled-in defaults.
  - `full_scale_speed_rad_s = 50.0F`: Converts duty to rad/s for the actuator task (`rad/s = duty * 50.0F`), matching the motor model and `MOT_SPD_FF = 0.02 s/rad`.
  - `duty_steps`: The fixed array of 5 steps (`0.0F`, `0.10F`, `0.20F`, `-0.10F`, `-0.20F`), wrapping cyclically.
- **Debouncing (`Debouncer`)**:
  - Plain 30 ms debouncer: samples `example::user_key_pressed()` every 10 ms; 3 consecutive agreeing samples confirm a key press transition.
- **Control Work Item (`DemandItem`)**:
  - Runs on `wq:ctrl` at 200 Hz (`interval_us = 5'000`).
  - Samples the key debouncer every 10 ms (every two 5 ms ticks).
  - Checks `example::actuator_power()` every cycle. If power is lost while driving, it immediately resets `step_index_ = 0`, `commanded_duty_ = 0.0F`, emits `demand=0.000`, and publishes `ArmingState` with `armed = false`.
  - When non-zero demand is held and power is present: publishes `ArmingState{.armed = true}` and streams `vk::msg::ActuatorDemand` at 200 Hz with output 1 set to `commanded_duty * full_scale_speed_rad_s` and outputs 2 to 4 set to `0.0F`.
  - When demand is zero (step 0 or wrap back to 0): publishes `ArmingState{.armed = false}` and stops publishing demand.
  - **Plain stopping mechanism**: In this example, outputs stop because `DemandItem` publishes `armed = false`. The output watchdog acts only if `wq:ctrl` stalls or crashes and stops refreshing the watchdog timer. If demand were left published, `ActuatorTask` would continue driving without timeout—which is why the firmware never allows modules other than `CommanderTask` to publish `arming_state`.
- **Display and Reporting Work Item (`DisplayItem`)**:
  - Runs on `wq:slow` at 4 Hz (`interval_us = 250'000`).
  - Subscribes to `vk::msg::ActuatorOutputs` to inspect applied duties (`output[0]` to `output[3]`).
  - Emits report lines `out1`, `out2`, `out3`, `out4`, and `power` for automated test evaluation.
  - Formats lines with stack-allocated `Line` buffers and updates the SSD1306 OLED display.
- **Platform Support Code (`examples/support/`)**:
  - `example::start_actuators()`: encapsulates initialization of timer PWM channels, direction lines, quadrature encoders, `ActuatorTask` on `wq:ctrl`, and `vk::platform::start_output_watchdog()`.
  - `example::actuator_power()`: returns power sense state (`PE4` on hardware, simulated power in SITL).
  - `example::user_key_pressed()`: returns user key state (`PC5` on hardware, simulated key in SITL).
  - In SITL, the support code parses `--key-presses` and `--power-cut-at` options and executes timed scenario events using an internal `ScenarioItem` work item, keeping `main.cpp` completely free of simulator-specific code.

## Exercises

1. **Observe slew rate limiting**: Modify `duty_steps` in `main.cpp` to jump directly from `-0.20F` to `+0.20F` on a single key press. In SITL or on hardware, observe how `OUTPUT 1` on the display or report ramps gradually across 0.20 s (`delta_duty / MOT_SLEWRATE = 0.40 / 2.0 = 0.20 s`) rather than stepping discontinuously.
2. **Test duty clamping**: Change a step in `duty_steps` to `0.50F` (50 % duty). Verify that `std::clamp(duty, -max_duty, max_duty)` clamps the commanded duty to `+0.20F` and the OLED displays `DEMAND: +0.20`, preventing accidental overdrive.
3. **Exercise output watchdog fault handling**: In `main.cpp`, add a busy loop or long sleep inside `DemandItem::run()` when a specific step is reached. Observe that `ActuatorTask` on `wq:ctrl` starves, causing the hardware output watchdog timer on TIM6 (or the watchdog task in SITL) to trip after 15 ms and force all motor outputs off.

## Read further

- [ADR-0002: Bus backbone and pluggable Links](../../../docs/adr/0002-bus-backbone-and-pluggable-links.md)
- [ADR-0004: FreeRTOS usage](../../../docs/adr/0004-freertos.md)
- [ADR-0005: Setpoint arbitration, arming and output watchdog](../../../docs/adr/0005-setpoint-arbitration.md)
- [ADR-0008: Bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0009: Layered HAL](../../../docs/adr/0009-layered-hal.md)
- [ADR-0013: Control chain and vehicle models](../../../docs/adr/0013-control-chain-and-vehicle-models.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [Spec 0006: First wheel motion](../../../docs/specs/0006-first-wheel-motion.md)
- [Spec 0013: Board examples](../../../docs/specs/0013-board-examples.md)
- [C50C V1.1 board definition (`board.toml`)](../../../boards/c50c-v1_1/board.toml)
- [C50C V1.1 board guide](../../../docs/boards/c50c-v1_1.md#actuator-power-sense)
- [Driving the wheels guide](../../../docs/guide/driving.md)
- [Examples guide](../../../docs/guide/examples.md)
- [Contributing guidelines (`CONTRIBUTING.md`)](../../../CONTRIBUTING.md)
- [Actuator task header (`actuator_task.hpp`)](../../../modules/actuators/include/vk/actuators/actuator_task.hpp)
- [Actuator control header (`actuator_control.hpp`)](../../../modules/actuators/include/vk/actuators/actuator_control.hpp)
- [Actuator demand message (`actuator_demand.msg`)](../../../schema/msg/actuator_demand.msg)
- [Actuator outputs message (`actuator_outputs.msg`)](../../../schema/msg/actuator_outputs.msg)
- [Arming state message (`arming_state.msg`)](../../../schema/msg/arming_state.msg)
- [Output watchdog header (`output_watchdog.hpp`)](../../../hal/include/vk/hal/output_watchdog.hpp)
