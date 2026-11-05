# 12-usb-gamepad: The USB host on CN8 and an XInput gamepad

## What you learn

- How the gamepad reaches the bus ([spec 0010](../../../docs/specs/0010-remote-control.md)): TinyUSB runs the STM32's USB OTG FS core as a host in its own FreeRTOS task, an XInput class driver reads the pad's reports, and a work item publishes `manual_control_input`.
- What the topic promises: 50 times a second while the pad is connected, its latest state repeated, since an XInput pad reports only changes; and once with `connected` false when it goes. So silence means that the remote control is lost.
- What the topic does not know: it carries sticks, triggers and buttons, and knows nothing of modes. Arming, taking over and driving are the commander's work ([Remote control](../../../docs/guide/driving.md#remote-control)). This example has no commander, so no button here arms or moves anything.

## Before this

- [`03-oled`](../03-oled/README.md): the SSD1306 display, `TextScreen` and `Line`.
- [`04-work-queues`](../04-work-queues/README.md): work items at a rate.
- [`05-bus`](../05-bus/README.md): topics and subscriptions.

## Hardware

- An XInput gamepad, the Xbox 360 wired protocol, plugged into CN8. It works whatever its vendor and product IDs. HID-class pads and the Xbox One protocol are out of scope ([spec 0010](../../../docs/specs/0010-remote-control.md)). A pad with a mode switch must be in its Xbox 360 mode: the Flydigi pad checked in #124 then enumerates as `045E:028E`.
- CN8 is the USB OTG FS core on `PA11` and `PA12` ([`board.toml`](../../../boards/c50c-v1_1/board.toml)). Its VBUS comes straight from the board's 5 V rail, and nothing switches it ([C50C V1.1](../../../docs/boards/c50c-v1_1.md#usb-gamepad)).
- The example does not start the actuator task, so the outputs are not driven.

## Build and flash

Build with the `c50c-v1_1-examples` preset, then flash this example's image ([Board examples](../../../docs/guide/examples.md#flashing-an-example)):

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-12-usb-gamepad.apj
```

It does not run in SITL: its subject is the USB host, which SITL replaces with a simulated gamepad ([SITL](../../../docs/guide/sitl.md)).

When finished, [return to the firmware](../../../docs/guide/examples.md#returning-to-the-firmware).

## What you see

Without a pad, the OLED shows `PAD: NONE` and zeros:

```text
VEHKERNEL 12-GAMEPAD

PAD: NONE
LEFT  +0.00 +0.00
RIGHT +0.00 +0.00
TRIG  0.00 0.00
```

Plug in the pad and the line changes to `PAD: CONNECTED`. Then:

- `LEFT` and `RIGHT` show each stick's x and y, from -1 to 1: x positive to the right, y positive forward.
- `TRIG` shows the left and the right trigger, from 0 to 1.
- The last two lines name the buttons held: `A`, `B`, `X`, `Y`, `LB`, `RB`, `ST` (start) and `BK` (back) on the first; `U`, `D`, `L`, `R` (the D-pad), `LS`, `RS` (the sticks pressed) and `GD` (guide) on the second.

Unplug the pad and the screen returns to `PAD: NONE` and zeros, from the one message with `connected` false.

## The code

Everything is in `main.cpp`:

- `front_buttons` and `other_buttons` give each button of the `gamepad_button` enum a short name for the screen. `buttons` in the topic is a bitmask, with the enum's value as the bit number.
- `pressed()` turns the bitmask into a `Line` of the names held.
- `GamepadDisplayItem` runs on `wq:slow` at 10 Hz, so that the sticks feel live. `run()` drains its subscription and keeps the newest message in `last_input_`. Before the first message, `last_input_` is all zeros and `connected` false, which is what the screen shows as no pad.
- `main()` calls `example::init()`, then `example::start_usb_gamepad()`, which starts the USB task and the publishing work item on `wq:io`, as the firmware does ([`usb_gamepad.cpp`](../../../platform/stm32f4/src/usb_gamepad.cpp)). It then starts the display item and the scheduler.

## Exercises

1. Show each stick as 0 while it is inside 0.08 of its centre, the default of `MAN_DEADZONE`, and compare a stick at rest before and after.
2. Keep the time of the newest message, and show its age on line 1. While the pad is connected it stays small, since the topic repeats 50 times a second. After you unplug the pad it grows: this is the silence that the commander's failsafe judges against `RC_TIMEOUT`.
3. Count how many messages `run()` drains on each run, and show it. At 50 Hz and 10 Hz, expect about five.

## Read further

- [Spec 0010: remote control](../../../docs/specs/0010-remote-control.md)
- [Remote control](../../../docs/guide/driving.md#remote-control), how the firmware drives from this topic
- [C50C V1.1: USB gamepad](../../../docs/boards/c50c-v1_1.md#usb-gamepad)
- [`manual_control_input.msg`](../../../schema/msg/manual_control_input.msg) and [`gamepad_button.enum`](../../../schema/enum/gamepad_button.enum)
- [`usb_gamepad.cpp`](../../../platform/stm32f4/src/usb_gamepad.cpp) and [`xinput.hpp`](../../../platform/stm32f4/include/vk/platform/xinput.hpp)
- [ADR-0008: bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0025: board examples](../../../docs/adr/0025-board-examples.md)
- [Examples guide](../../../docs/guide/examples.md)
