# Examples are separate executables per board, ordered by a checked examples.toml

The firmware image (ADR-0017) compiles in every module a board supports, and the bring-up image (ADR-0018) runs automated pass/fail checks over SWD. Neither teaches a user how to use a single component of the board: a user wanting to understand the OLED, the IMU, the motor outputs or the parameter storage must navigate the full application and its thirty work items, and cannot run that component alone on the board.

The decision: **each board gets an `examples/` directory of small, single-purpose examples, built as separate executables in the dependency order declared by `examples.toml`**.

- **Separate executables**: like the bring-up image (ADR-0018) and the bootloader (ADR-0023), each example is a standalone executable built for one board, not a variant of the firmware image and not a runtime mode. Each example links into the `FIRMWARE` Flash region and is finalized with ArduPilot's app descriptor (`.elf`, `.bin`, `.hex`, `.apj`), so it is flashed with the bootloader (`tools/flash/`) and replaced by the firmware afterwards.
- **Dependency order**: `examples/<board>/examples.toml` orders the examples from the simplest peripheral (a blinking LED) up to complex subsystems (motor control, USB gamepad). Each example introduces one or more components and uses only components introduced by earlier examples. This order reflects both the hardware dependency chain and the software layering (ADR-0009).
- **Checked coverage**: an L0 check (`tools/examples/check_examples.py`, run by `ctest --preset test`) validates `examples.toml` against the parsed board definition (ADR-0011). Every board device, actuator output, encoder, serial port and flash region, and every pin role not covered by them, must be introduced by exactly one example or listed in `[not_covered]` with a reason. While examples are written, components of upcoming examples live in `[pending]`.
- **Lessons**: every example directory holds a `main.cpp` and a `README.md` lesson covering what you learn, prerequisites, hardware, build and flash commands, expected observations, a line-by-line code walkthrough, exercises, and references to relevant ADRs and guides.
- **Support code**: `examples/support/` provides what an example needs from its platform that would distract from the lesson: on hardware the clock tree and the FreeRTOS start, and on SITL the simulated devices and a report function that does nothing on hardware.
- **SITL simulation**: examples whose components SITL simulates declare `sitl = true`. Preset `sitl-examples` builds them as host executables, and CI runs them as L3 scenario tests.

## Considered Options

- **Examples selectable at boot inside the firmware image**: rejected. Bundling example logic into the production firmware wastes Flash, requires runtime branching or special parameters, and violates the rule that each board has exactly one firmware image (ADR-0017).
- **Unit test mocks instead of board executables**: rejected. While unit tests (L1) verify platform and driver contracts on the development host, board examples teach how the actual hardware behaves on the bench and must run on the MCU.
- **A single shared examples tree across all boards**: rejected. Boards differ in pinouts, peripherals and capabilities (ADR-0010). Each board definition has its own hardware facts and therefore its own ordered `examples.toml`.

## Consequences

- CI builds `c50c-v1_1-examples` on every push, ensuring that changes to platform contracts, drivers or modules never leave an example broken.
- Flashing an example overwrites the firmware in Flash, but parameters remain intact because examples carry the firmware's parameter set and link to the same `FIRMWARE` region without touching `PARAMS` (ADR-0020). Flashing the firmware again restores full operation.
