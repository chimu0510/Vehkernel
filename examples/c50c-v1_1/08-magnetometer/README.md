# 08-magnetometer: AK09916 magnetometer read through ICM20948 FIFO; OLED shows magnetic field, magnitude and heading; read over the Link at each heading

## What you learn

- How the internal Asahi Kasei AK09916 3-axis magnetometer inside the ICM20948 (`IMU0`) is read over software I2C on the C50C V1.1 (ADR-0015, [`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#imu)).
- How the ICM20948 internal I2C master configures and reads the AK09916: checks `WHO_AM_I` (`0x09` at register `0x01`) on slave 4, configures continuous measurement mode 4 at nominal 100 Hz, and reads the 9-byte measurement block (`ST1`, `HXL`, `HXH`, `HYL`, `HYH`, `HZL`, `HZH`, `TMPS`, `ST2`) via slave 0 (`drivers/icm20948/src/icm20948.cpp`, [`docs/specs/0012-ekf-attitude-and-magnetometer.md`](../../../docs/specs/0012-ekf-attitude-and-magnetometer.md#reading-the-magnetometer)).
- Why magnetometer records are read through the ICM20948 FIFO: the master reads `ST1` to `ST2` on every cycle, and reading `ST2` clears data-ready. Reading `EXT_SLV_SENS_DATA_00` directly at 200 Hz would miss about 1 sample in 9. Transferring 9-byte records to the FIFO at 281.25 Hz (`I2C_MST_ODR_CONFIG` 2, `GYRO_SMPLRT_DIV` 3) reads each sample once. On the board the AK09916 still skips a measurement that ends while the master reads `ST1` to `ST2`, about 6 %, so the rate is about 94 Hz rather than the nominal 100 Hz (spec 0012).
- How the driver converts raw sensor counts to microtesla (µT) using the AK09916 sensitivity of 0.15 µT per LSB, handles AK09916 inverted axes, and maps measurements into the vehicle body frame ($x$ forward, $y$ right, $z$ down) using mounting rotation `YAW_90` and board orientation `SENS_BOARD_ROT` (ADR-0014, [`GLOSSARY.md`](../../../GLOSSARY.md#vehicles-and-hardware)).
- Deriving the magnetic heading formula for a level board:
  - In local level NED (North-East-Down) coordinates, the Earth's magnetic field vector $\mathbf{B}$ points toward magnetic north ($B_N > 0$) with downward vertical component $B_D > 0$ in the northern hemisphere: $\mathbf{B}_{NED} = [B_N, 0, B_D]^T$.
  - When the vehicle is level ($roll = 0, pitch = 0$), the vehicle body horizontal plane ($xy$) is aligned with the local horizontal plane (North-East). Let $\psi$ denote the vehicle yaw heading measured clockwise from magnetic north to the forward body axis ($+x$).
  - Rotating $\mathbf{B}_{NED}$ into the body frame about the $z$-axis by heading angle $\psi$:
    $$\begin{bmatrix} B_x \\ B_y \\ B_z \end{bmatrix} = \begin{bmatrix} \cos\psi & \sin\psi & 0 \\ -\sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} B_N \\ 0 \\ B_D \end{bmatrix} = \begin{bmatrix} B_N \cos\psi \\ -B_N \sin\psi \\ B_D \end{bmatrix}$$
  - From this system:
    $$B_x = B_N \cos\psi$$
    $$B_y = -B_N \sin\psi \implies -B_y = B_N \sin\psi$$
  - Taking the ratio:
    $$\frac{-B_y}{B_x} = \frac{B_N \sin\psi}{B_N \cos\psi} = \tan\psi$$
  - Therefore, the magnetic heading is:
    $$\psi = \operatorname{atan2}(-B_y, B_x)$$
  - Converted to degrees:
    $$\psi_{\text{deg}} = \operatorname{atan2}(-B_y, B_x) \times \frac{180^\circ}{\pi}$$
  - This formula matches `heading_from_field` in [`test/sitl/check_magnetometer.py`](../../../test/sitl/check_magnetometer.py#L110): `atan2(-levelled[1], levelled[0])`.
- What this example leaves out and why:
  - **No tilt compensation**: `atan2(-y, x)` is only valid when the board is level. Tilting the board in roll or pitch projects the vertical magnetic field ($B_z \approx 30\text{ \mu T}$) onto body $x$ and $y$, distorting the computed heading. The firmware's EKF attitude estimator performs 3D attitude levelling using gravity before fusing heading observations (`lib/estimation/models/include/vk/estimation/models/magnetometer_model.hpp`, [`docs/guide/estimation.md`](../../../docs/guide/estimation.md#heading-fusion-and-vehicle-types-ekf_mag_type)).
  - **No calibration**: Real magnetometers experience hard-iron distortions (constant offsets from onboard magnets and screws) and soft-iron distortions (ellipsoidal deformation from surrounding ferromagnetic material). The firmware applies axis offsets (`CAL_MAG_XOFF`, `CAL_MAG_YOFF`, `CAL_MAG_ZOFF`) and scale factors (`CAL_MAG_XSCALE`, `CAL_MAG_YSCALE`, `CAL_MAG_ZSCALE`). Calibration is fitted on the development host from logs using [`tools/log/mag_cal.py`](../../../tools/log/mag_cal.py) ([`docs/guide/estimation.md`](../../../docs/guide/estimation.md#magnetometer-calibration)).
  - **No declination**: Magnetic north differs from true geographic north by local magnetic declination (for example, a round made-up declination of $+10.0^\circ$). The MCU does not store global magnetic models; true heading correction is configured via `EKF_MAG_DECL` ([`docs/guide/estimation.md`](../../../docs/guide/estimation.md#declination-ekf_mag_decl)).
- Subscribing to `magnetometer_sample` on `wq:ctrl` via event-driven `trigger_on()`, draining the queue with `copy()`, and sharing state safely across rate tiers using short critical sections (`vk::core::CriticalSection`, ADR-0008).
- Rendering live magnetic field components, total magnitude (norm), measured sample rate in Hz, and magnetic heading in degrees on the local SSD1306 OLED display (`DISPLAY0`) at 4 Hz on `wq:slow` (ADR-0022).
- Reading the field at a chosen heading from the development host with `link_monitor.py mag-read`, which averages `magnetometer_sample` over the Link instead of leaving you to read a display that changes four times a second.
- Simulating the AK09916 in SITL under FreeRTOS POSIX and verifying sample rate, heading tracking against simulated vehicle true yaw, and display content in automated L3 tests (ADR-0018).

## Before this

- [`01-led`](../01-led/README.md): System clock initialization, GPIO output, and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation (ADR-0007).
- [`04-work-queues`](../04-work-queues/README.md): Rate-tiered FreeRTOS work queues, periodic scheduling with `schedule_periodic()`, and cooperative scheduling within a queue.
- [`05-bus`](../05-bus/README.md): Publish/subscribe bus backbone, schema-generated topics, `vk::core::Publication<T>`, `vk::core::Subscription<T>`, and event-driven wakeups with `trigger_on()`.
- [`06-imu`](../06-imu/README.md): ICM20948 accelerometer and gyroscope publishing `imu_sample` over software I2C; specific force and down axis.
- [`07-serial-link`](../07-serial-link/README.md): The Link on `SERIAL1` and `SERIAL2`, topic maps, and `link_monitor.py`.

## Hardware

- **Board**: C50C V1.1 robot controller ([`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)).
- **IMU (`IMU0`)**: TDK InvenSense ICM20948 motion tracking sensor with internal Asahi Kasei AK09916 magnetometer:
  - `IMU_SCL` (`PB9`): I2C clock line driven by software I2C.
  - `IMU_SDA` (`PB8`): I2C data line driven by software I2C.
  - Level shifter: RS0102 (U19) interfacing PB9/PB8 to the sensor's I2C lines.
  - Bus address: `0x68`.
  - AK09916 internal I2C address: `0x0C` (accessed through ICM20948 master slave 0 and slave 4).
  - Mounting rotation: `YAW_90` relative to the board forward axis ([`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#imu)).
- **Display (`DISPLAY0`)**: 0.96-inch 128 x 64 monochrome OLED display based on the SSD1306 controller, connected via software SPI:
  - `OLED_SCL` (`PD7`): SPI clock line.
  - `OLED_SDA` (`PD6`): SPI MOSI line.
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
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-08-magnetometer.apj
```

Global options (`--port`, `--build`) go before the `update` subcommand. This example's Link has no control access, so it ignores `update`'s reboot request, and `update` restarts the board through `SERIAL2`'s download circuit instead; on `SERIAL1` the user must power-cycle the board while `update` waits (see [C50C V1.1](../../../docs/boards/c50c-v1_1.md#serial-ports)).

When finished with this example, return the board to standard firmware:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

### Running in SITL

`08-magnetometer` also builds and runs on your development host in SITL simulation with the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

Run the example executable directly, specifying duration, rotating motion, the Link's UDP address, and capturing the display output:

```bash
build/sitl-examples/app/examples/example-08-magnetometer --duration 10 --motion rotate --udp 127.0.0.1:15600 --display /tmp/display.pbm
```

In a second terminal, read the field over the Link. While SITL rotates, the horizontal field averages out and `bz` stays:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 --msg-dir build/sitl-examples/generated/schema/python mag-read --duration 5
```

To run the automated L3 scenario check in SITL:

```bash
ctest --preset sitl-examples
```

## What you see

At boot, the FreeRTOS scheduler starts. The ICM20948 driver initializes on `wq:ctrl`, configures its internal I2C master, probes the AK09916, sets continuous mode 4 (100 Hz), routes ST1-ST2 reads into the FIFO, and publishes `magnetometer_sample` at 100 Hz. Each sample wakes `MagSubscriberItem` on `wq:ctrl`, which records the latest field components. `DisplayItem` runs periodically at 4 Hz on `wq:slow`, updating the OLED:

- Line 0: Header text `VEHKERNEL 08-MAG`.
- Line 1: Blank.
- Line 2: Magnetic field $x$-component in µT, formatted as `BX: +30.00 UT`.
- Line 3: Magnetic field $y$-component in µT, formatted as `BY: +0.00 UT`.
- Line 4: Magnetic field $z$-component in µT, formatted as `BZ: +30.00 UT`.
- Line 5: Total field magnitude in µT, formatted as `NORM: +42.43 UT`.
- Line 6: Measured sample rate in Hz, formatted as `RATE: 100 HZ`.
- Line 7: Magnetic heading in degrees, formatted as `HEADING: +0.00 DEG`.

### Observing magnetic heading on the bench

- **Pointing North on level bench**: When the front of the board faces magnetic north, `BX` reads positive horizontal field, `BY` reads near zero, and `HEADING` displays near `+0.00 DEG`.
- **Turning East (90° turn clockwise)**: Turning the board 90° clockwise points $+x$ East, so magnetic north is to the vehicle's left ($-y$). `BY` goes negative, `BX` approaches zero, and `HEADING` displays near `+90.00 DEG`.
- **Turning South (180° turn)**: Facing south, magnetic north points behind ($-x$). `BX` goes negative, `BY` reads near zero, and `HEADING` reads near `+180.00 DEG` (or `-180.00 DEG`).
- **Turning West (270° or -90°)**: Facing west, magnetic north points to the right ($+y$). `BY` goes positive, `BX` approaches zero, and `HEADING` displays near `-90.00 DEG`.
- **Tilting the board (demonstrating why EKF levelling is required)**: Pitching the board up or down on the bench tilts the downward vertical field component into $B_x$, causing `HEADING` to drift even though the vehicle's yaw did not change. This demonstrates why the firmware fuses heading only after levelling the field with estimated roll and pitch.

### Reading a heading over the Link

Connect a USB cable to `SERIAL1` or `SERIAL2`, turn the board to a heading and hold it still and level, then read:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1-examples/generated/schema/python mag-read --label north
```

It averages `magnetometer_sample` for two seconds (`--duration` changes that) and prints one line, with values like these:

```text
mag_read label=north samples=200 rate_hz=100.00 bx=30.00 by=0.00 bz=30.00 norm=42.43 heading_deg=0.00 heading_spread_deg=0.40 failures=0
```

- `rate_hz` comes from the MCU's timestamps, so host scheduling does not skew it. The C50C V1.1 reads about 94 Hz, as the top of this lesson explains; the 100 above is SITL's.
- `heading_deg` comes from the mean field, by the same `atan2(-y, x)` as the OLED.
- `heading_spread_deg` is how far the furthest sample's heading was from it. A large value means the board moved during the read, or a disturbance.
- `--label` is only a name in the line. Repeat the command with `east`, `south` and `west` at the other headings.

The example does not load the stored parameters, so the field carries no calibration: expect offsets from the board's and the vehicle's own magnetism in the absolute heading.

In SITL, standard output prints report lines:
- `heading=<deg>`, `bx=<val>`, `by=<val>`, `bz=<val>`, `norm=<val>` emitted on each display update at 4 Hz.
- `rate=<Hz>` emitted once per second.
- `true_yaw=<deg>` emitted alongside heading by SITL support code during rotation.
- `display_on=1` emitted when SITL shuts down.

## The code

The complete source code is in `main.cpp`:

- **Includes and namespaces**:
  - `<line.hpp>`: stack-based string, integer, and fixed-point float formatter (`example::Line`).
  - `<support.hpp>`: platform abstraction (`example::init()`, `example::display()`, `example::imu()`, `example::report()`, `example::start_scheduler()`).
  - `<vk/core/bus.hpp>`: bus subscription and topic lookup (`vk::core::Subscription`, `vk::core::topic()`).
  - `<vk/core/critical_section.hpp>`: short critical sections (`vk::core::CriticalSection`) for atomic cross-queue state reads.
  - `<vk/core/static_instance.hpp>`: static object storage without heap allocation (ADR-0007).
  - `<vk/core/work_queue.hpp>`: rate-tiered work queues and items (`vk::core::WorkItem`, `vk::core::WorkQueueTier`).
  - `<vk/drivers/icm20948.hpp>`: ICM20948 driver with AK09916 magnetometer support (`DRIVER_ICM20948_MAG`).
  - `<vk/drivers/ssd1306.hpp>`: SSD1306 OLED display driver.
  - `<vk/link/link.hpp>`, `<vk/link/topic_map.hpp>`: the Link and its topic map.
  - `<vk/local_display/pages.hpp>`: `vk::local_display::TextScreen` 21 x 8 text buffer.
  - `<vk/msg/link_status.hpp>`: schema-generated Link statistics message definition.
  - `<vk/msg/magnetometer_sample.hpp>`: schema-generated magnetometer message definition.
- **Subscriber work item (`MagSubscriberItem`)**:
  - Subclasses `vk::core::WorkItem("mag_sub", queue)` and attaches to `wq:ctrl`.
  - In `start()`, registers as an event-driven topic listener using `trigger_on(vk::core::topic<vk::msg::MagnetometerSample>(0))`.
  - In `run()`, drains all buffered samples in the queued topic (`@queue 4`):
    ```cpp
    vk::msg::MagnetometerSample sample{};
    while (subscription_.copy(sample)) {
        const vk::core::CriticalSection lock;
        state_.field = sample.field;
        ++state_.samples;
    }
    ```
  - Provides thread-safe snapshot reading via `state()` protected by `CriticalSection`.
- **Display work item (`DisplayItem`)**:
  - Subclasses `vk::core::WorkItem("display", queue)` and attaches to `wq:slow`.
  - Schedules periodic execution at 4 Hz (`schedule_periodic(250'000)`).
  - In `run()`:
    1. Reads `state()` atomically from the subscriber.
    2. Once per second (`now - last_rate_time_us_ >= 1'000'000`), computes sample rate `rate_hz_ = delta_samples * 1e6 / elapsed_us` and emits `example::report("rate", ...)`.
    3. Computes total field magnitude: $\text{norm} = \sqrt{B_x^2 + B_y^2 + B_z^2}$.
    4. Computes magnetic heading in degrees: $\psi = \operatorname{atan2}(-B_y, B_x) \times \frac{180^\circ}{\pi}$.
    5. Emits `example::report("heading", heading_deg)`, `bx`, `by`, `bz`, and `norm`.
    6. Renders lines 0, 2, 3, 4, 5, 6, 7 into `screen_` using `Line().add_fixed()` and `Line().add()`, draws into `display_.frame()`, and flushes modified pages via `display_.update()`.
- **Topic map (`ExampleStreams`, `example_topic_map`)**:
  - Sends every `magnetometer_sample` (`vk::link::every_sample`, at the topic's nominal rate) and `link_status` once a second, one stream object per Link, as `07-serial-link` does for `imu_sample`.
- **Main entry point (`main`)**:
  - Initializes platform support with `example::init(argc, argv)`.
  - Instantiates `MagSubscriberItem` on `wq:ctrl` and `DisplayItem` on `wq:slow`.
  - Starts the Links with `example::start_links(example_topic_map)`, then the work items and the IMU driver.
  - Starts the FreeRTOS scheduler with `example::start_scheduler()`.

## Exercises

1. **Horizontal field strength**: Extend `DisplayItem::run()` to calculate horizontal field magnitude $B_h = \sqrt{B_x^2 + B_y^2}$. Verify on the bench that $B_h$ remains approximately constant as the vehicle turns on a level surface.
2. **Magnetic disturbance indicator**: Compare total field magnitude $\text{norm}$ against expected Earth field strength (for example, a made-up nominal strength of $45.0\text{ \mu T}$). If $|\text{norm} - 45.0| > 9.0\text{ \mu T}$ (a 20 % disturbance, as `EKF_MAG_REJ` checks), render `DISTURBED` on line 1 of the OLED display.
3. **Cardinal directions**: Map `heading_deg` into 8 cardinal compass directions (`N`, `NE`, `E`, `SE`, `S`, `SW`, `W`, `NW`) and display the text direction next to the heading on line 7.

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
- [Spec 0012: The EKF attitude estimator and the magnetometer](../../../docs/specs/0012-ekf-attitude-and-magnetometer.md)
- [Spec 0013: Board examples](../../../docs/specs/0013-board-examples.md)
- [Estimation guide (`docs/guide/estimation.md`)](../../../docs/guide/estimation.md)
- [Building guide (`docs/guide/building.md`)](../../../docs/guide/building.md)
- [Examples guide (`docs/guide/examples.md`)](../../../docs/guide/examples.md)
- [C50C V1.1 board definition (`board.toml`)](../../../boards/c50c-v1_1/board.toml)
- [C50C V1.1 board guide (`docs/boards/c50c-v1_1.md`)](../../../docs/boards/c50c-v1_1.md#imu)
- [ICM20948 driver header (`icm20948.hpp`)](../../../drivers/icm20948/include/vk/drivers/icm20948.hpp)
- [Magnetometer sample message (`magnetometer_sample.msg`)](../../../schema/msg/magnetometer_sample.msg)
- [Magnetometer measurement model (`magnetometer_model.hpp`)](../../../lib/estimation/models/include/vk/estimation/models/magnetometer_model.hpp)
- [Magnetometer calibration tool (`mag_cal.py`)](../../../tools/log/mag_cal.py)
- [Glossary (`GLOSSARY.md`)](../../../GLOSSARY.md)
