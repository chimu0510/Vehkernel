# 03-oled: Drawing a title and a counter on the OLED

## What you learn

- How to drive the on-board 128 x 64 SSD1306 monochrome OLED display (`DISPLAY0`) over software SPI using the display driver (`vk::drivers::Ssd1306`).
- How the write-only display bus contract (`vk::hal::DisplayBus`) abstracts the software SPI bit-banging (`vk::platform::SoftSpiDisplayOn`).
- Using the local display module's text screen (`vk::local_display::TextScreen`) to format and render lines onto a display frame (`vk::drivers::MonoFrame`).
- Formatting numbers and strings into a fixed line buffer (`Line`) without `snprintf` or heap allocation (ADR-0007).
- Pacing updates without drift using absolute monotonic time from `vk::hal::SystemClock::now_us()` and `example::sleep_until_us()`.
- The SSD1306 display controller lifecycle: how `Ssd1306::update()` manages the 100 ms reset hold and transmits initialisation commands over successive calls.
- Running the exact same example code under SITL host simulation against the simulated display (`vk::sim::Ssd1306`), verifying `display_on=1` and whole frames with `check_examples.py`.

## Before this

- [`01-led`](../01-led/README.md): Blinking the on-board RGB LED, using the system clock (`vk::hal::SystemClock::now_us()`), and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line.

## Hardware

- **Board**: C50C V1.1 robot controller.
- **Display (`DISPLAY0`)**: 0.96-inch 128 x 64 monochrome OLED display module based on the SSD1306 controller (`boards/c50c-v1_1/board.toml`).
- **Pins**: The display is connected via a 4-wire software SPI interface:
  - `OLED_SCL` (`PB13`): SPI clock.
  - `OLED_SDA` (`PB15`): SPI MOSI (data).
  - `OLED_RST` (`PB14`): Active-low hardware reset.
  - `OLED_DC` (`PB12`): Data/command select line (low for commands, high for display data).
- **Connection**: Connect your development host to the board's one-key download port (`SERIAL2`).
- **Safety**: This example drives only the local OLED display. Motor outputs and actuators are not configured.

## Build and flash

### Building for the board

Build the example with the `c50c-v1_1-examples` preset inside the development container:

```bash
cmake --preset c50c-v1_1-examples
cmake --build --preset c50c-v1_1-examples
```

Flash the compiled example (`.apj`) over the one-key download port using the serial flasher tool:

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-03-oled.apj
```

Global options (`--port`, `--build`) go before the `update` subcommand. An example runs no Link, so `update` finds no Link to ask and restarts the board through `SERIAL2`'s download circuit instead; on `SERIAL1` the user must power-cycle the board while `update` waits (see [C50C V1.1](../../docs/boards/c50c-v1_1.md#serial-ports)).

When finished with this example, return the board to the standard firmware using the default build:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

### Running in SITL

`03-oled` also builds and runs on your development host in SITL simulation with the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

Run the example executable directly, specifying a run duration and a file to capture the display image:

```bash
build/sitl-examples/app/examples/example-03-oled --duration 3 --display /tmp/display.pbm
```

To run the automated L3 scenario check in SITL:

```bash
ctest --preset sitl-examples
```

## What you see

At boot, the OLED display initializes automatically through the loop's initial update calls.
The display shows:
- Line 0: the header text `VEHKERNEL 03-OLED`.
- Line 2: a counter formatted as `COUNTER: <n>`, starting at `0` and incrementing by 1 every 100 ms (10 Hz).
- All other lines remain blank.

In SITL, each counter increment emits a `counter=<n>` report line to standard output. When the run finishes, it reports `display_on=1` from the simulated SSD1306's `displaying()`. The captured PBM image confirms that whole frames were received: line 0 displays the title `VEHKERNEL 03-OLED`, line 2 displays the final counter value, and every other line is blank.

## The code

The complete source code is in `main.cpp`:

- **Includes**: `<support.hpp>` provides the platform abstraction (`example::init()`, `example::display()`, `example::sleep_until_us()`, `example::report()`, `example::running()`). `<vk/drivers/ssd1306.hpp>` provides the `vk::drivers::Ssd1306` driver. `<vk/hal/clock.hpp>` provides the system clock. `<vk/local_display/pages.hpp>` provides `vk::local_display::TextScreen`.
- **Text formatting (`Line`)**: Formats text and unsigned integers into a fixed-size `std::array<char, 21>` buffer without pulling in newlib heap allocation or `snprintf` (`_sbrk`).
- **Text screen (`TextScreen`)**: The example reuses the `local_display` module's text screen: `screen.clear()` clears all 8 lines to spaces, `screen.put(line, col, text)` writes strings into the grid, and `screen.draw(frame)` renders the glyphs into `MonoFrame`.
- **Display initialisation and first frame**: The loop calls `display.update()` directly without manual delays. On the first call, `Ssd1306::update()` holds the hardware reset line active for 100 ms. Once 100 ms has passed, the subsequent call brings the controller out of reset, sends the vendor initialisation sequence (charge pump, contrast, addressing), and marks all 8 pages as stale, immediately flushing the entire first frame to the panel. Subsequent calls update only the lines that changed.
- **Drift-free pacing**: In each iteration, `next_us += step_us` advances the target deadline by exactly 100 000 µs (10 Hz), and `example::sleep_until_us(next_us)` pauses until that timestamp arrives (busy-wait on the board, `nanosleep` in SITL). Pacing by absolute time prevents drawing duration from causing timing drift.
- **Reporting**: `example::report("counter", counter)` emits report lines in SITL and is a no-op on the board.

## Exercises

1. **Change the update rate**: Alter `step_us` from `100'000` (10 Hz) to `50'000` (20 Hz) or `500'000` (2 Hz) and observe the paced refresh rate.
2. **Add multiple text lines**: Use `screen.put()` to draw additional text on lines 4 and 6 of the display.
3. **Right-align text**: Experiment with `screen.put_right(line, text)` to right-align status indicators or labels.

## Read further

- [ADR-0007: C++ subset for the firmware](../../docs/adr/0007-cxx-subset.md)
- [ADR-0009: Layered HAL](../../docs/adr/0009-layered-hal.md)
- [ADR-0018: Test layers and SITL](../../docs/adr/0018-test-layers-and-sitl.md)
- [ADR-0022: Local display and local menu](../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../docs/adr/0025-board-examples.md)
- [SSD1306 driver](../../drivers/ssd1306/include/vk/drivers/ssd1306.hpp)
- [TextScreen (`pages.hpp`)](../../modules/local_display/include/vk/local_display/pages.hpp)
- [Simulated SSD1306](../../sim/include/vk/sim/ssd1306.hpp)
- [Building guide](../../docs/guide/building.md)
- [Examples guide](../../docs/guide/examples.md)
- [C50C V1.1 board page](../../docs/boards/c50c-v1_1.md)
