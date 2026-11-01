# 02-user-key: Cycling LED colours with the user key

## What you learn

- How to configure and read MCU pins as GPIO inputs with internal pull resistors using the platform layer (`vk::platform::InputLine`, `user_key`).
- Accessing board definition hardware facts from `board.hpp` (`vk::board::user_key::pin` and `vk::board::user_key::pressed_high`) instead of hard-coding pin assignments or active polarity.
- Understanding switch bounce in mechanical push-buttons and implementing a deterministic sampling debouncer.
- Detecting state transitions (rising edge of debounced pressed state) so each button actuation triggers exactly one colour change.
- Combining GPIO inputs and outputs to cycle through RGB LED colours.

## Before this

- [`01-led`](../01-led/README.md): Blinking the on-board RGB LED, driving pins as GPIO push-pull outputs, using the system clock (`vk::hal::SystemClock::now_us()`), and the build and flash workflow.

## Hardware

- **Board**: C50C V1.1 robot controller.
- **Connection**: Connect your development host to the board's one-key download port (`SERIAL2`). The download circuit on `SERIAL2` allows the flasher to restart the board into the bootloader automatically.
- **User key (S3)**: The user key S3 is connected to pin `PC5` (`boards/c50c-v1_1/board.toml`). The board definition specifies an internal pull-up resistor (`pull = "UP"`) and an active-low pressed level (`pressed = "LOW"`). When S3 is pressed, it pulls `PC5` to ground.
- **LED and light strips**: The `LIGHT_R`, `LIGHT_G` and `LIGHT_B` pins drive the AT8236 RGB light strip drivers as well as the on-board LED (`boards/c50c-v1_1/board.toml`), so a connected light strip changes colour along with the on-board LED.
- **Safety**: This example drives only the on-board LED and any connected light strip, and reads push-button S3. Motor outputs and actuators are not configured.

## Build and flash

Build the example with the `c50c-v1_1-examples` preset inside the development container:

```bash
cmake --preset c50c-v1_1-examples
cmake --build --preset c50c-v1_1-examples
```

Flash the compiled example (`.apj`) over the one-key download port using the serial flasher tool:

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-02-user-key.apj
```

Global options (`--port`, `--build`) go before the `update` subcommand. An example runs no Link, so `update` finds no Link to ask and restarts the board through `SERIAL2`'s download circuit instead; on `SERIAL1` the user must power-cycle the board while `update` waits (see [C50C V1.1](../../../docs/boards/c50c-v1_1.md#serial-ports)).

When finished with this example, return the board to the standard firmware using the default build:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

As with flashing the example, because the running example has no Link, `update` restarts the board through `SERIAL2`'s download circuit (or waits for a power-cycle on `SERIAL1`).

## What you see

At boot, the on-board RGB LED starts off.
Each debounced press of user key S3 advances the LED to the next colour in an 8-state sequence:
1. Red (primary)
2. Green (primary)
3. Blue (primary)
4. Yellow (Red + Green)
5. Cyan (Green + Blue)
6. Magenta (Red + Blue)
7. White (Red + Green + Blue)
8. Off

Holding the button down keeps the LED on the selected colour without advancing; releasing the button does not trigger a colour change. The next press advances to the subsequent colour. After Off, the sequence wraps back to Red.

## The code

The complete source code is in `main.cpp`:

- **Includes**: `<board.hpp>` provides the board's pin and hardware definitions, `<support.hpp>` provides platform initialization (`example::init()`), `<vk/hal/clock.hpp>` provides monotonic microsecond time, and `<vk/platform/gpio.hpp>` provides GPIO line templates.
- **Pin adaptation**: Reuses the LED pins `LIGHT_R`, `LIGHT_G` and `LIGHT_B` as push-pull GPIO outputs (`vk::platform::PushPull`) introduced in `01-led`.
- **User key input line**: `using UserKeyLine = vk::platform::InputLine<vk::board::user_key::pin>;`. `InputLine::init()` enables the GPIO clock, sets the pin mode to input, and configures the internal pull resistor declared in `board.toml` (`vk::hal::Pull::Up`).
- **Reading pressed level**: `is_key_pressed()` compares `UserKeyLine::is_high()` against `vk::board::user_key::pressed_high`. On C50C V1.1, `pressed_high` is `false` (active low), so `is_key_pressed()` returns `true` when the pin reads low. Pin assignment and active polarity are never hard-coded.
- **Fixed colour palette**: The `colors` array defines eight distinct states: Off, the three primary colours (Red, Green, Blue), and their combinations (Yellow, Cyan, Magenta, White).
- **Debouncer**: Mechanical switch contacts bounce between open and closed for a few milliseconds when pressed or released. `Debouncer` samples the key every 10 ms. It tracks consecutive agreeing samples; when 3 samples in a row (30 ms) agree on a new state, `debounced_` updates. When transitioning from released to pressed, `update()` returns `true` once; releasing the button returns `false`.
- **Main entry point `main()`**: `example::init()` configures clocks. `UserKeyLine::init()` sets up the button input. The LED lines are initialized low with `init(false)`, and `set_color` sets the initial colour state (off). The loop samples the button every 10 ms with `delay_ms(10)`. When `debouncer.update()` registers a debounced press, `color_index` increments modulo 8 and `set_color()` displays the new colour.

## Exercises

1. **Debounce tuning**: Change `debounce_count` in `Debouncer` to 1. With one sample a single bounce that lands on a 10 ms sample counts as a press: watch for a click that skips a colour. Then shorten the sampling period in `main()` to 1 ms and compare.
2. **Reverse stepping**: Modify the loop or `main()` so that presses cycle backward through the palette, or skip secondary combinations.
3. **Trigger on release**: Modify `Debouncer::update()` so that the colour change occurs when the key is released rather than when pressed.

## Read further

- [ADR-0009: Layered HAL](../../../docs/adr/0009-layered-hal.md)
- [ADR-0011: TOML board definitions](../../../docs/adr/0011-toml-board-definition-toolchain.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- Firmware debouncer: [`modules/local_display/include/vk/local_display/user_key.hpp`](../../../modules/local_display/include/vk/local_display/user_key.hpp) — the production firmware implementation of `vk::local_display::UserKey`, which handles short presses, long presses (1 s), auto-repeat (200 ms), and ignores a key held at boot until released.
- [Building guide](../../../docs/guide/building.md)
- [Flashing guide](../../../docs/guide/flashing.md)
- [C50C V1.1 board page](../../../docs/boards/c50c-v1_1.md)
