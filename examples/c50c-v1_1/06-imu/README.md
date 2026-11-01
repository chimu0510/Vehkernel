# 06-imu: ICM20948 IMU driver publishing imu_sample over software I2C

## What you learn

- How the ICM20948 9-axis motion tracking device (`IMU0`) is interfaced over software I2C on the C50C V1.1 (ADR-0015, [`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#imu)).
- Why the C50C V1.1 runs the IMU over software I2C rather than hardware I2C1: `PB9` is wired to SCL and `PB8` to SDA through the RS0102 level shifter (U19), which is the reverse pinout of STM32F407 hardware I2C1 (ADR-0015, [`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)).
- How one software I2C read of accelerometer, gyroscope and temperature takes ~0.50 ms of CPU time, achieving the board default rate of 200 Hz (`vk::board::timing::base_rate_hz`) at ~10 % CPU load (ADR-0015).
- How the driver checks `WHO_AM_I` (`0xEA`), resets the device, configures sample rate and full-scale ranges, and handles bus errors and reprobing (ADR-0009).
- How the driver converts raw sensor registers into SI units (m/s² and rad/s) and transforms them from the sensor frame ($x$ forward, $y$ left, $z$ up) into the vehicle body frame ($x$ forward, $y$ right, $z$ down) by composing mounting rotation (`YAW_90`) with board orientation (`SENS_BOARD_ROT`) (ADR-0014, [`GLOSSARY.md`](../../../GLOSSARY.md#vehicles-and-hardware)).
- The sign convention for specific force and identifying which way is down: an accelerometer measures contact force minus gravity ($f = a - g$). In the vehicle body frame ($x$ forward, $y$ right, $z$ down, as defined in [`schema/msg/imu_sample.msg`](../../../schema/msg/imu_sample.msg) and [`docs/guide/link-monitor.md`](../../../docs/guide/link-monitor.md#attitude)), gravity acts in the $+z$ direction (downward). At rest on level ground ($a = 0$), the upward contact reaction force from the ground produces $f_z = -g \approx -9.81\text{ m/s}^2$ along the body $z$-axis (`AZ` reads about -9.81, [`accelerometer_model.hpp`](../../../lib/estimation/models/include/vk/estimation/models/accelerometer_model.hpp)). Because specific force at rest measures the upward reaction, the direction pointing toward the Earth (down) is opposite to the measured specific force vector, so the down axis is $+Z$ (`DOWN: +Z`).
- How `imu_sample` topics are declared with static circular buffers (`@queue 4`, `@instances 2`) to ensure that high-rate samples are not overwritten when consumers are preempted (ADR-0008, ADR-0015, [`schema/msg/imu_sample.msg`](../../../schema/msg/imu_sample.msg)).
- Subscribing to `imu_sample` on `wq:ctrl` via event-driven `trigger_on()`, draining the queue with `copy()`, and sharing state safely across rate tiers using short critical sections (`vk::core::CriticalSection`, ADR-0008).
- Rendering live specific force components, measured sample rate in Hz, and the down axis on the local SSD1306 OLED display (`DISPLAY0`) at 4 Hz on `wq:slow` (ADR-0022).
- What this example leaves out: runtime parameter storage (`SENS_BOARD_ROT`, gyro/accel calibration, introduced in `09-parameters`), attitude estimation filters (Mahony and EKF), and the internal AK09916 magnetometer (introduced in `07-magnetometer`).
- Simulating the IMU in SITL under FreeRTOS POSIX and verifying sample rate, down axis, and display content in automated L3 tests (ADR-0018).

## Before this

- [`01-led`](../01-led/README.md): System clock initialization (`vk::hal::SystemClock::now_us()`), GPIO output, and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation (ADR-0007).
- [`04-work-queues`](../04-work-queues/README.md): Rate-tiered FreeRTOS work queues, periodic scheduling with `schedule_periodic()`, and cooperative scheduling within a queue.
- [`05-bus`](../05-bus/README.md): Publish/subscribe bus backbone, schema-generated topics, `vk::core::Publication<T>`, `vk::core::Subscription<T>`, and event-driven wakeups with `trigger_on()`.

## Hardware

- **Board**: C50C V1.1 robot controller ([`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)).
- **IMU (`IMU0`)**: TDK InvenSense ICM20948 motion sensor:
  - `IMU_SCL` (`PB9`): I2C clock line driven by software I2C.
  - `IMU_SDA` (`PB8`): I2C data line driven by software I2C.
  - `IMU_INT` (`PD3`): Data-ready interrupt line (uncovered in timer-triggered 200 Hz mode, ADR-0015).
  - Level shifter: RS0102 (U19) interfacing PB9/PB8 to the sensor's I2C lines.
  - Bus address: `0x68`.
  - Mounting rotation: `YAW_90` relative to the board forward axis ([`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#imu)).
- **Display (`DISPLAY0`)**: 0.96-inch 128 x 64 monochrome OLED display based on the SSD1306 controller, connected via software SPI:
  - `OLED_SCL` (`PD7`): SPI clock line.
  - `OLED_SDA` (`PD6`): SPI MOSI (data) line.
  - `OLED_RST` (`PD5`): Active-low reset line.
  - `OLED_DC` (`PD4`): Data/command select line.
- **Connection**: Connect your development host to the board's one-key download port (`SERIAL2`).
- **Safety**: Motor outputs and actuators remain unconfigured and inactive.

## Build and flash

### Building for the board

Build the example with the `c50c-v1_1-examples` preset inside the development container:

```bash
cmake --preset c50c-v1_1-examples
cmake --build --preset c50c-v1_1-examples
```

Flash the compiled example (`.apj`) over the one-key download port using the serial flasher tool:

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-06-imu.apj
```

Global options (`--port`, `--build`) go before the `update` subcommand. An example runs no Link, so `update` restarts the board through `SERIAL2`'s download circuit instead; on `SERIAL1` the user must power-cycle the board while `update` waits (see [C50C V1.1](../../../docs/boards/c50c-v1_1.md#serial-ports)).

When finished with this example, return the board to standard firmware:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

### Running in SITL

`06-imu` also builds and runs on your development host in SITL simulation with the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

Run the example executable directly, specifying duration and capturing the display output:

```bash
build/sitl-examples/app/examples/example-06-imu --duration 3 --display /tmp/display.pbm
```

To run the automated L3 scenario check in SITL:

```bash
ctest --preset sitl-examples
```

## What you see

At boot, the FreeRTOS scheduler starts. The ICM20948 driver initializes on `wq:ctrl`, probes `WHO_AM_I` (`0xEA`), resets the chip, configures sampling, and begins periodic reads at 200 Hz. Each sample publication on `imu_sample` wakes the subscriber work item on `wq:ctrl`, which drains queued samples, stores the latest specific force, and counts received samples. The display work item runs periodically at 4 Hz on `wq:slow`, updating the OLED screen:

- Line 0: Header text `VEHKERNEL 06-IMU`.
- Line 1: Blank.
- Line 2: Specific force $x$-component in m/s², formatted as `AX: +0.00 M/S2`.
- Line 3: Specific force $y$-component in m/s², formatted as `AY: +0.00 M/S2`.
- Line 4: Specific force $z$-component in m/s², formatted as `AZ: -9.81 M/S2`.
- Line 5: Measured sample rate in Hz, formatted as `RATE: 200 HZ`.
- Line 6: Which body axis points down, formatted as `DOWN: +Z`.
- Line 7: Blank.

### Observing sensor motion on the bench

- **Level at rest**: With the board resting flat on a level bench, `AX` and `AY` are near `+0.00 M/S2` while `AZ` reads approximately `-9.81 M/S2` (the upward reaction force supporting the board against gravity). The down axis names the body axis facing the ground: `DOWN: +Z`.
- **Tilted forward (pitch)**: Raising the front edge tilts gravity's upward reaction force towards the rear ($-x$), causing `AX` to go negative, while $+X$ tilts upward and $-X$ points downward.
- **Tilted right (roll)**: Lowering the right edge tilts the board so body $+Y$ faces the ground; gravity's upward reaction force tilts toward $-y$ (`AY` goes negative) and the down axis names the downward-facing side (`DOWN: +Y`).
- **Turned on its side**: Tilting the board onto its right side causes body $+Y$ to face the ground, so `AY` reads approximately `-9.81 M/S2` and the OLED displays `DOWN: +Y`. Resting the board on its front nose displays `DOWN: +X`, while turning the board upside down (wheels up) displays `DOWN: -Z`.

In SITL, standard output prints report lines:
- `down=<axis>` emitted on each display update at 4 Hz (e.g. `down=+Z`).
- `ax=<val>`, `ay=<val>`, `az=<val>` emitted on each display update.
- `rate=<Hz>` emitted once per second (e.g. `rate=200`).
- `display_on=1` emitted when SITL shuts down.

The captured PBM display image confirms whole frames: line 0 shows the header, lines 2-4 show specific force components, line 5 shows measured sample rate, line 6 shows down axis, and lines 1 and 7 remain blank.

## The code

The complete source code is in `main.cpp`:

- **Includes and namespaces**:
  - `<line.hpp>`: stack-based string, integer, and fixed-point float formatter (`example::Line`).
  - `<support.hpp>`: provides platform abstraction (`example::init()`, `example::display()`, `example::imu()`, `example::report()`, `example::start_scheduler()`).
  - `<vk/core/bus.hpp>`: bus subscription and topic lookup (`vk::core::Subscription`, `vk::core::topic()`).
  - `<vk/core/critical_section.hpp>`: short critical sections (`vk::core::CriticalSection`) for atomic cross-queue state reads.
  - `<vk/core/static_instance.hpp>`: static object storage without heap allocation (ADR-0007).
  - `<vk/core/work_queue.hpp>`: rate-tiered work queues and items (`vk::core::WorkItem`, `vk::core::WorkQueueTier`).
  - `<vk/drivers/icm20948.hpp>`: ICM20948 IMU driver.
  - `<vk/drivers/ssd1306.hpp>`: SSD1306 OLED display driver.
  - `<vk/local_display/pages.hpp>`: `vk::local_display::TextScreen` 21 x 8 text buffer.
  - `<vk/msg/imu_sample.hpp>`: schema-generated IMU message definition.
- **Platform abstraction (`example::imu()`)**:
  - Implemented in `examples/support/stm32f4/support.hpp` for hardware and `examples/support/sitl/support.hpp` for SITL simulation.
  - On the C50C V1.1 board, `example::imu()` initializes the software I2C bus on `PB9` and `PB8` (`vk::platform::SoftI2cOn<Imu::pins>`), configures the `Icm20948` driver with address `0x68`, mounting rotation `YAW_90`, default board orientation `Rotation::None`, and read rate 200 Hz (`vk::board::timing::base_rate_hz`), and places it on `wq:ctrl` (`WorkQueueTier::Ctrl`).
  - In SITL, `example::imu()` attaches the driver to `vk::platform::imu_bus()` and `vk::platform::simulated_imu()`.
- **Down axis detection (`down_axis()`)**:
  - Determines which body axis points down by finding the specific force component with the greatest absolute magnitude and taking the sign opposite to the reading.
  - Because an accelerometer measures specific force (the upward contact reaction supporting the vehicle against gravity, rather than the downward gravitational acceleration itself), the body axis pointing toward the ground has the opposite sign to the measured specific force. For example, when resting level, `AZ` is negative (~-9.81 m/s²), indicating that body $+Z$ points down (`+Z`).
- **Subscriber work item (`ImuSubscriberItem`)**:
  - Subclasses `vk::core::WorkItem("imu_sub", queue)` and attaches to `wq:ctrl`.
  - In `start()`, registers as an event-driven topic listener using `trigger_on(vk::core::topic<vk::msg::ImuSample>(0))`.
  - In `run()`, drains all buffered samples in the queued topic:
    ```cpp
    vk::msg::ImuSample sample{};
    while (subscription_.copy(sample)) {
        const vk::core::CriticalSection lock;
        state_.accel = sample.accel;
        ++state_.samples;
    }
    ```
  - Provides thread-safe snapshot reading via `state()` protected by `CriticalSection`.
- **Display work item (`DisplayItem`)**:
  - Subclasses `vk::core::WorkItem("display", queue)` and attaches to `wq:slow`.
  - Schedules periodic execution at 4 Hz (`schedule_periodic(250'000)`).
  - In `run()`:
    1. Reads `state()` atomically from the subscriber.
    2. Once per second (`now - last_rate_time_us_ >= 1'000'000`), computes `rate_hz_ = delta_samples * 1e6 / elapsed_us` and emits `example::report("rate", ...)`.
    3. Identifies the down axis with `down_axis(state.accel)`.
    4. Emits `example::report("down", down)`, `ax`, `ay`, and `az`.
    5. Renders lines 0, 2, 3, 4, 5, 6 into `screen_` using `Line().add_fixed()` and `Line().add()`, draws into `display_.frame()`, and flushes modified pages via `display_.update()`.
- **Why work queues separate IMU processing from display rendering**:
  - The ICM20948 driver and subscriber run on `wq:ctrl`, the highest-priority rate tier (ADR-0008). One software I2C read takes ~0.50 ms every 5 ms.
  - The OLED display driver runs on `wq:slow`. Updating a dirty display frame over software SPI takes up to ~5.5 ms on the MCU.
  - Because `wq:ctrl` preempts `wq:slow`, display bit-banging never delays high-rate sensor acquisition or estimation.
- **Fixed-point float formatting (`Line::add_fixed`)**:
  - Formats numbers with sign, integer part, and two decimal places (e.g. `+0.00`, `-9.81`) without heap allocation (`malloc` or `new`) or `printf` (ADR-0007).
- **What this example leaves out**:
  - The parameter system: Uses hardcoded defaults (`Rotation::None` for `SENS_BOARD_ROT`, zero calibration offsets) rather than non-volatile Flash storage (introduced in `09-parameters`).
  - Magnetometer: Does not configure the internal AK09916 or publish `magnetometer_sample` (introduced in `07-magnetometer`).
  - Estimators: Does not run Mahony or EKF attitude estimators.

## Exercises

1. **Calculate total acceleration magnitude**: Extend `DisplayItem::run()` to compute the Euclidean norm $\sqrt{a_x^2 + a_y^2 + a_z^2}$ using `std::sqrt`. Display `NORM:` on line 7 and verify it remains close to $9.81\text{ m/s}^2$ when the board is motionless.
2. **Detect free-fall**: Check if the acceleration magnitude drops below $3.0\text{ m/s}^2$ and display `FREE FALL` on line 1 of the OLED.
3. **Inspect driver health**: Call `example::imu().status()` to query `read_failures` and `samples`, and verify that `read_failures` remains zero during normal operation.
4. **Tip the board onto each side**: Tip the board onto each of its six sides (flat, upside down, left side, right side, nose down, and tail down). Check which axis the display names in each orientation: `+Z` (level at rest), `-Z` (upside down), `+Y` (on its right side), `-Y` (on its left side), `+X` (nose down), and `-X` (tail down).

## Read further

- [ADR-0002: Bus backbone and pluggable Links](../../../docs/adr/0002-bus-backbone-and-pluggable-links.md)
- [ADR-0004: FreeRTOS as the RTOS](../../../docs/adr/0004-freertos.md)
- [ADR-0006: Schema and wire encoding](../../../docs/adr/0006-schema-and-wire-encoding.md)
- [ADR-0008: Bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0009: Layered HAL](../../../docs/adr/0009-layered-hal.md)
- [ADR-0014: Estimation architecture](../../../docs/adr/0014-estimation-architecture.md)
- [ADR-0015: Selectable timing](../../../docs/adr/0015-selectable-timing.md)
- [ADR-0018: Test layers and SITL](../../../docs/adr/0018-test-layers-and-sitl.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [C50C V1.1 board definition (`board.toml`)](../../../boards/c50c-v1_1/board.toml)
- [C50C V1.1 board guide](../../../docs/boards/c50c-v1_1.md)
- [Link monitor guide (`link-monitor.md`)](../../../docs/guide/link-monitor.md#attitude)
- [ICM20948 driver header (`icm20948.hpp`)](../../../drivers/icm20948/include/vk/drivers/icm20948.hpp)
- [IMU sample message (`imu_sample.msg`)](../../../schema/msg/imu_sample.msg)
- [Accelerometer measurement model (`accelerometer_model.hpp`)](../../../lib/estimation/models/include/vk/estimation/models/accelerometer_model.hpp)
- [Building guide](../../../docs/guide/building.md)
- [Examples guide](../../../docs/guide/examples.md)
