# Board examples

Vehkernel provides a structured set of board examples under `examples/` ([ADR-0025](../adr/0025-board-examples.md), [spec 0013](../specs/0013-board-examples.md)). Rather than studying the full firmware with its complete module stack, each example isolates a single board component or subsystem, demonstrating its use in a minimal `main.cpp` accompanied by an instructional lesson.

The examples are numbered in hardware and layering dependency order: earlier examples introduce fundamental platform primitives (GPIO, clocks), while later examples build upon them to introduce drivers, work queues, the bus, Links, parameters and actuators.

## Layout

- [`examples/README.md`](../../examples/README.md): index of board examples and supported hardware.
- `examples/<board>/examples.toml`: ordered declaration of examples, introduced components, prerequisites, and SITL compatibility.
- `examples/<board>/examples.config`: Kconfig debug overlay (`CONFIG_EXAMPLES=y`).
- `examples/<board>/NN-name/`: one directory per example, containing:
  - `main.cpp`: compact example entry point.
  - `README.md`: lesson covering objectives, wiring, code walkthrough and exercises.
- `examples/support/`: platform support code for clock initialization, FreeRTOS start and test reporting.

## Building the examples

Board examples are built with the `<board>-examples` preset, such as `c50c-v1_1-examples`. Run `cmake` and `cmake --build` inside the development container ([Building](building.md)):

```bash
cmake --preset c50c-v1_1-examples
cmake --build --preset c50c-v1_1-examples
```

The build produces one finalized executable per example under `build/<board>-examples/app/examples/`:
- `example-NN-name.elf`
- `example-NN-name.bin`
- `example-NN-name.hex`
- `example-NN-name.apj`

Every example links against the board's `FIRMWARE` Flash region and includes ArduPilot's app descriptor, enforcing size budgets at link time.

## Flashing an example

Examples are flashed over the bootloader using the serial flasher tool ([Flashing](flashing.md)). Connect the host to the board's one-key download port (SERIAL2):

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-01-led.apj
```

Global options (`--port`, `--build`) go before the `update` subcommand. An example runs no Link, or a Link with no control access that ignores the request, so `update` gets no reply and restarts the board through SERIAL2's download circuit instead; on SERIAL1 the user must power-cycle the board while `update` waits (see [C50C V1.1](../boards/c50c-v1_1.md#serial-ports)).

Because examples share the board's Flash layout and Kconfig parameters, flashing an example leaves the parameter sectors (`PARAMS`) untouched.

## Running in SITL

Examples that declare `sitl = true` in `examples.toml` build for the development host with the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

The build compiles the SITL examples with clang-tidy into host executables under `build/sitl-examples/app/examples/`. Run an example directly, with options to specify duration and capture the display:

```bash
build/sitl-examples/app/examples/example-03-oled --duration 3 --display /tmp/display.pbm
```

To run the automated L3 scenario checks for every SITL example, or for one with `-R`:

```bash
ctest --preset sitl-examples
ctest --preset sitl-examples -R example_07_serial_link
```

## Returning to the firmware

To return the board to normal vehicle firmware, rebuild the firmware preset and flash with the default build:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

As with flashing an example, when the running example does not answer the Link's reboot request, `update` restarts the board through SERIAL2's download circuit (or waits for a power-cycle on SERIAL1). Since parameter storage is preserved across updates, all vehicle calibration and runtime parameters are retained.
