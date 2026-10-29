# Spec 0013: board examples

**Status**: deferred. Issue: [#156](https://github.com/chimu0510/Vehkernel/issues/156); tickets #157 to #169, with the hardware check in #169. Deferred by the maintainer at publication, on 2026-10-29: spec 0012 stays the active spec until the maintainer names this one; every ticket is left.

Until this spec is done, this file and its issue carry the same spec; update both in the same edit.

## Problem Statement

The firmware uses every component of the C50C V1.1: the OLED, the user key, the ICM20948, the motor outputs and their encoders, the serial Links, the parameter storage and the USB gamepad. But it uses them all at once, inside one image of about thirty work items. A user who wants to learn how one of them is driven has to read `app/firmware_main.cpp`, the module it lives in and the platform code under it, and cannot run that component alone on the board.

- **No program does one thing.** The firmware image (ADR-0017) has every module the board supports. The bring-up image (ADR-0018) runs automated pass or fail cases over SWD and teaches nothing about the API.
- **No order to learn in.** The components build on each other (the IMU driver is a work item that publishes on the bus, the Link carries topics, the actuator task needs the output watchdog), but nothing says which to read first.
- **No lesson.** The guides describe how to build, flash and drive the vehicle, not how to write firmware for the board.

## Solution

Deliver the thirteenth milestone: **an `examples/` directory with one small example per component of the C50C V1.1, in the components' dependency order**. Afterwards:

- Each example is a separate executable for the board, as the bring-up image is: one `main.cpp` that exercises one component, flashed like the firmware and replaced by it afterwards.
- The examples are numbered in dependency order, from a blinking LED to the USB gamepad. Each one uses only components an earlier example introduced, so the order is both the order they were developed in and the order a user learns the board in. The first examples use the platform layer and the drivers directly; later ones introduce the work queues, the bus and the modules, so the order also teaches the layering (ADR-0009).
- Each example has a lesson, its `README.md`: what it teaches, which examples come first, the hardware it needs, how to build and flash it, what to expect, a walk through the code and exercises.
- `examples/c50c-v1_1/examples.toml` declares the order and the components each example introduces and uses, and an L0 check keeps it in step with the board definition: every component of the board is introduced by an example or listed as not covered, with a reason.
- One preset, `c50c-v1_1-examples`, builds every example of the board, and CI builds it on every push. The examples whose components SITL simulates also build for SITL, with `sitl-examples`, and CI runs them as L3 scenarios.

## User Stories

1. As a new C50C user, I want a numbered list of examples, one per component, so that I can learn the board one component at a time in an order that works.
2. As a new C50C user, I want each example to do one thing in one short `main.cpp`, so that I can read all of it and see exactly how that component is used.
3. As a new C50C user, I want each example's lesson to say what to expect on the board, so that I know whether it worked.
4. As a new C50C user, I want the lessons to point to the ADRs and the guides behind each component, so that I know where to read further.
5. As a user without a board, I want the examples SITL can run to run on my development host, so that I can learn the bus, the IMU, the Link and the parameters before I own the hardware.
6. As a firmware developer, I want every example built and size-checked in CI, so that a change to a driver or a module cannot leave an example broken.
7. As a firmware developer, I want the dependency order declared in one file and checked against the board definition, so that a new component is not forgotten and an example never uses a component a later one introduces.
8. As a board maintainer adding a new board (ADR-0010), I want the examples' layout and check to be per board, so that the new board gets its own ordered examples.
9. As a vehicle operator, I want the motor example to cap its output and stop when it loses its demand, so that a learning user cannot drive the vehicle off the bench.
10. As a C50C user, I want a documented hardware check of every example, so that each lesson is confirmed on the real board (ADR-0018 L4).

## Implementation Decisions

- **What an example is** (decided with the maintainer, recorded in a new ADR-0025): a separate executable built for one board, like the bring-up image and the bootloader (ADR-0017), not a variant of the firmware image and not a mode selected at boot. One example per component, one function per example. ADR-0017's sentence on separate executables is edited to name the examples and to point to ADR-0025.
- **Layout**:
  - `examples/README.md`: the index of every board's examples, linked from the README and from a new guide, `docs/guide/examples.md`, which says how to build, flash, run in SITL and go back to the firmware.
  - `examples/c50c-v1_1/examples.toml`: the board's examples in order (below).
  - `examples/c50c-v1_1/examples.config`: the board's debug overlay for the examples (ADR-0017): `CONFIG_EXAMPLES=y` and nothing else, so the examples see the board's Kconfig and the firmware's parameter set.
  - `examples/c50c-v1_1/NN-name/main.cpp` and `README.md`, `NN` from `01`, one directory per example.
  - `examples/support/`: what an example needs from its platform and that would distract from the lesson, kept small: on the STM32F4 the clock tree and the FreeRTOS start, and on SITL the simulated devices and the report (below). An example takes its devices from `board.hpp` on the board and from this support code in SITL, through the HAL concepts (ADR-0009).
- **Building** (ADR-0017, decided with the maintainer):
  - Preset `c50c-v1_1-examples` inherits `c50c-v1_1` and sets `VK_DEBUG_OVERLAY` to the board's `examples.config`. With `CONFIG_EXAMPLES`, `app/CMakeLists.txt` adds `examples/<board>/` in place of the firmware entry point, as it does for the bring-up image, and the build makes one executable per example, `example-NN-name.elf`, with its `.bin`, `.hex` and `.apj` finalized as the firmware's are (ADR-0023).
  - Every example links against the FIRMWARE region, so the link fails when one is too large, and it is flashed with the bootloader (`tools/flash/`) like the firmware. Flashing the firmware again leaves the board as it was; the parameter storage keeps its contents, since the examples carry the firmware's parameter set.
  - Preset `sitl-examples` inherits `sitl` and builds, for the SITL platform, the examples that declare `sitl = true`, as host executables. A test preset of the same name runs them.
  - The C++ subset and the coding rules apply to examples as to the firmware (ADR-0007, `CONTRIBUTING.md`); `sitl-examples` runs clang-tidy with every warning as an error, as `sitl` does. Each `main.cpp` has the project's comment density, plus a reference to its lesson at the top; the teaching is in the lesson, not in the comments.
  - CI adds `c50c-v1_1-examples` and `sitl-examples` to its preset loop and runs `ctest --preset sitl-examples`.
- **The dependency order** (decided with the maintainer): `examples.toml` lists the board's examples in order, each with:
  - `name` (`NN-name`, matching its directory, numbered from 01 without gaps in list order), a one-line `summary`;
  - `introduces` and `uses`: the components the example teaches for the first time and the ones it relies on. A component is either a board component, named as the board definition names it (`device:IMU0`, `pin:LIGHT_R`, `output:1`, `encoder:1`, `serial:SERIAL1`, `flash:PARAMS`, `user_key`, `actuator_power`), or a firmware component the file declares in its `[firmware]` table (`work_queue`, `bus`, `link`, `parameters`, `actuator_control`), each with one line of description;
  - `sitl`: true when the example also runs in SITL.
  - A `[not_covered]` table lists each board component no example introduces, with the reason, such as "no driver yet".
  - While the examples are written, a `[pending]` table lists the components of the examples not yet written, so that the check passes from the first ticket on; each example's ticket moves its components out of it, and the last one removes it.
- **The L0 check** (`tools/examples/check_examples.py`, run by `ctest --preset test` on every board's `examples.toml`, with the board generator's parsed board definition):
  - Every board component named exists in the board definition, and every device, actuator output, encoder, serial port and flash region of the board definition is introduced by exactly one example or listed in `[not_covered]`, not both. So is every pin role, except that a pin is covered by the device, output, encoder, serial port, `user_key` or `actuator_power` entry that names it.
  - Every component an example uses is introduced by that example or an earlier one; a firmware component used is declared in `[firmware]`.
  - The example directories under `examples/<board>/` are exactly those listed, and each holds a `main.cpp` and a `README.md` with the lesson's sections.
  - Every example with `sitl = true` is built by `sitl-examples`, and only those.
  - Its own unit tests, on small example files, cover each failure.
- **The C50C V1.1 examples** (decided with the maintainer), each with the component it introduces:
  1. `01-led`: GPIO output and the system clock; blinks the on-board RGB LED. The board definition gives `LIGHT_R`, `LIGHT_G` and `LIGHT_B` timer functions for a PWM driver that does not exist yet, so the example drives the same pins as GPIO outputs, on or off, and its lesson says so. A colour mix by PWM waits for that driver.
  2. `02-user-key`: GPIO input with the board's pull-up and pressed level (`user_key`); each press of S3 changes the LED's colour, with the key debounced in the example.
  3. `03-oled`: the software SPI and the SSD1306 driver (`DISPLAY0`); draws text and a counter. SITL: the simulated SSD1306.
  4. `04-work-queues`: FreeRTOS, the work queues and work items at a rate (ADR-0004, ADR-0008); two work items at different rates count on the OLED. SITL.
  5. `05-bus`: topics, publications and subscriptions (ADR-0002, ADR-0008); one work item publishes a counter that another, woken by it, draws on the OLED. SITL.
  6. `06-imu`: the software I2C and the ICM20948 driver (`IMU0`), publishing `imu_sample`; the OLED shows the specific force and the rate, and which way is down. SITL.
  7. `07-magnetometer`: the AK09916 read through the ICM20948's FIFO and `magnetometer_sample` (spec 0012); the OLED shows the field and its heading. Its ticket is blocked by #150. SITL.
  8. `08-serial-link`: the Link on `SERIAL1` and `SERIAL2` (ADR-0002, ADR-0006); the IMU's topics seen with `link_monitor.py` on the development host. SITL, over UDP.
  9. `09-parameters`: the parameter system and its storage in the `PARAMS` region (ADR-0012, ADR-0020); a parameter set over the Link is shown on the OLED and kept across a reset. SITL, on the simulated NOR Flash.
  10. `10-motor-outputs`: actuator control on outputs 1 to 4, the output watchdog and the actuator power sense (ADR-0005, spec 0006); the user key steps one output through a few demands, capped in the example at 20 % duty, and the outputs stop when the demand stops or the drive power is cut. The example feeds the actuator task itself, without the commander or arming; its lesson says why the firmware never does, and that the wheels must be off the ground. SITL, on the simulated motors.
  11. `11-wheel-encoders`: the encoders of outputs 1 to 4 and `wheel_encoders`; the OLED shows each wheel's count and speed, turned by hand or by the demands of `10-motor-outputs`. SITL.
  12. `12-usb-gamepad`: the USB host on CN8 (`pin:USB_DM`, `pin:USB_DP`) and the XInput gamepad (spec 0010), publishing `manual_control_input`; the OLED shows the sticks and the buttons. Board only.
  - Not covered, in `[not_covered]`: the revision-ID pins (board verification, shown by the firmware and the bring-up image), CAN, the RC receiver inputs and SBUS, the steering servos and alt config 1, the analog inputs, `SERIAL3` (Bluetooth), and the `BOOT` and `EVENTLOG` regions. Each gets an example when a spec adds its support.
- **Running in SITL** (ADR-0018, decided with the maintainer): an example with `sitl = true` builds unchanged for SITL against the simulated devices of `sim/`, which `examples/support/` provides. It prints `key=value` report lines, as SITL does, through a small report function from the support code that does nothing on the board. One L3 ctest case per SITL example, through `test/sitl/check_examples.py`, runs it for a few seconds, drives the simulator where needed (the gamepad, the drive power, the Link) and checks the report: for example the counters' rates, the IMU's down axis, a parameter kept across a restart, an output stopped when its demand stops.
- **The lesson** (decided with the maintainer): each example's `README.md` has these sections, which the L0 check looks for: *What you learn*, *Before this* (the earlier examples it builds on), *Hardware* (what to connect, and the safety precautions where outputs move), *Build and flash*, *What you see*, *The code* (a walk through `main.cpp` in order, naming its lines), *Exercises* (two or three changes to try), and *Read further* (the ADRs, the guides and the modules behind the component). `examples/README.md` lists the examples in order, with their summaries and whether they run in SITL.
- **The bring-up image** (decided with the maintainer): unchanged. It stays the automated L4 pass or fail check of a new board; the examples are lessons. They share the mechanism, a debug overlay that replaces the firmware entry point, and neither moves.
- **Only supported components** (decided with the maintainer): this spec adds no driver and no platform support. Where an example needs something the firmware lacks, the example works around it and its lesson says so, as `01-led` does for the PWM, or the component is not covered.
- **Documentation**: `docs/guide/examples.md`, the examples' index, the lessons, ADR-0025 and the edit to ADR-0017; `GLOSSARY.md` gains **Example** (a separate executable for one board that exercises one component, with its lesson) and **Lesson**; the Building guide lists the two presets; the README links the examples.

## Testing Decisions

- **Seam A: the L0 check (L0, L1).** `check_examples.py` passes on the C50C's `examples.toml` and fails, in its unit tests, on: an unknown board component; a board component neither introduced nor listed as not covered, or both; one introduced twice; a component used before it is introduced; an undeclared firmware component; a gap in the numbering; a directory not listed or listed without its files; a lesson missing a section.
- **Seam B: the build (L0).** `c50c-v1_1-examples` builds every example of the board within its Flash and RAM budgets, and `sitl-examples` builds the SITL ones with clang-tidy clean, in CI on every push.
- **Seam C: the SITL examples (L3).** `ctest --preset sitl-examples` runs every SITL example and checks its report, as each ticket describes. Tests that check a rate or a value use tolerances that hold on a loaded CI runner.
- **Seam D: hardware (L4, manual).** On the C50C V1.1, every example flashed and worked through by its lesson, with the motor examples' wheels off the ground and the drive power limited: what the lesson says to expect is what the board does. The firmware is flashed back afterwards and boots with its parameters kept. Results are recorded on the ticket, and a lesson that was wrong is corrected there.
- **Prior art**: the bring-up image and its overlay (ADR-0018); Zephyr's `samples/` with a `sample.yaml` per sample and its twister builds; ESP-IDF's `examples/` with a README per example; ArduPilot's `libraries/*/examples/` built by `waf examples`; PX4's `src/examples/`.

## Out of Scope

- New drivers or platform support: the RGB LED's PWM, CAN, the RC receiver and SBUS, the servos, the analog inputs and Bluetooth get their examples with the specs that support them.
- Examples for a board other than the C50C V1.1, or for SITL alone.
- An automated runner that flashes the examples and judges them on the board; the bring-up image remains the automated L4 check.
- Examples of the estimators, the control chain, logging, the local menu or the Bridge: they are modules over the components, documented by their guides.
- A selectable example inside the firmware image, and examples as a mode (ADR-0005).

## Further Notes

- The examples follow the firmware's API as it is. A change to a driver's or a module's interface updates the examples that use it, and their lessons, in the same pull request; CI makes a broken example visible.
- A new board (ADR-0010) gets its examples in its own spec, with its own `examples.toml` and the same check.
