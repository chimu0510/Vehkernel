# 01-led: Blinking the on-board RGB LED

## What you learn

- How a board example is built as a separate executable in the `FIRMWARE` Flash region and flashed through the bootloader (ADR-0025, `flash:FIRMWARE`).
- Driving MCU pins as GPIO push-pull outputs through the platform layer.
- Querying monotonic microsecond time using the system clock (`vk::hal::SystemClock::now_us()`).
- Why the C50C V1.1 board definition assigns `LIGHT_R`, `LIGHT_G` and `LIGHT_B` timer functions, and how this example drives the same physical pins as GPIO outputs until a PWM driver is implemented.

## Before this

None: `01-led` is the first example in the series.

## Hardware

- **Board**: C50C V1.1 robot controller.
- **Connection**: Connect your development host to the board's one-key download port (`SERIAL2`). The download circuit on `SERIAL2` allows the flasher to restart the board into the bootloader automatically.
- **LED and light strips**: The `LIGHT_R`, `LIGHT_G` and `LIGHT_B` pins drive the AT8236 RGB light strip drivers as well as the on-board LED (`boards/c50c-v1_1/board.toml`), so a connected light strip lights along with the on-board LED.
- **Safety**: This example drives only the on-board LED and any connected light strip. Motor outputs and actuators are not configured.

## Build and flash

Build with the `c50c-v1_1-examples` preset, then flash this example's image ([Board examples](../../../docs/guide/examples.md#flashing-an-example)):

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-01-led.apj
```

When finished, [return to the firmware](../../../docs/guide/examples.md#returning-to-the-firmware).

## What you see

The on-board RGB LED (and any attached light strip) blinks continuously, cycling through each primary colour:
1. Red for 500 ms
2. Green for 500 ms
3. Blue for 500 ms
4. Off for 500 ms

## The code

The complete source code is in `main.cpp`:

- **Includes**: `<board.hpp>` provides the board's pin definitions, `<support.hpp>` provides platform initialization (`example::init()`), `<vk/hal/clock.hpp>` provides monotonic time, and `<vk/platform/gpio.hpp>` provides STM32F4 GPIO line templates.
- **Pin adaptation**: In `boards/c50c-v1_1/board.toml`, pins `PE6`, `PB15` and `PB14` (red, green and blue) are assigned timer channels (`TIM9_CH2`, `TIM12_CH2`, `TIM12_CH1`) for future PWM colour blending. Because that driver is not yet available, we extract each pin's `port` and `number` from `vk::board::pins::LIGHT_*` to define concrete `vk::hal::GpioOutput` types, wrapping each with `vk::platform::PushPull`.
- **Delay function**: `delay_ms()` samples `vk::hal::SystemClock::now_us()` and busy-waits until the elapsed duration in microseconds is reached.
- **Colour helper**: `set_color(r, g, b)` writes logical high or low levels to the three LED colour channels.
- **Entry point `main()`**: `example::init()` configures the clock tree and DWT cycle counter. `init(false)` then configures each GPIO pin as a push-pull output with the initial state driving low (off). The main loop cycles through red, green, blue, and off with `delay_ms()` calls between transitions.

## Exercises

1. **Change blink frequency**: Adjust the 500 ms delay arguments in `main()`'s calls to `delay_ms()` (for example to 100 ms) to observe a faster strobe effect.
2. **Colour mixing**: Modify the arguments in `set_color()` in `main()` to create secondary colours: Yellow (Red + Green), Cyan (Green + Blue), Magenta (Red + Blue), or White (all three).
3. **Morse code**: Implement a short helper function that blinks an SOS pattern (`... --- ...`).

## Read further

- [ADR-0009: Layered HAL](../../../docs/adr/0009-layered-hal.md)
- [ADR-0011: TOML board definitions](../../../docs/adr/0011-toml-board-definition-toolchain.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [Building guide](../../../docs/guide/building.md)
- [Flashing guide](../../../docs/guide/flashing.md)
- [C50C V1.1 board page](../../../docs/boards/c50c-v1_1.md)
