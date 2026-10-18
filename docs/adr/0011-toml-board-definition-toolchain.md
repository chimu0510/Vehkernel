# TOML board definitions with an MCU database and a DMA solver

The real value of ArduPilot's hwdef is its MCU database, pin validation and DMA solver, not the `.dat` format. PX4, by contrast, assembles pins by hand in `board_config.h` and fills in DMA streams and channels by hand in `timer_config.cpp`, with no conflict checking at all. Vehkernel adopts ArduPilot's toolchain but writes board definitions in TOML, the same format as parameter definitions; the board generator (`tools/generators/vkboard/`) turns them into everything else that depends on the board.

- **One pin per line**: TOML 1.0 requires inline tables on a single line, so the format itself guarantees one pin per line, ready for line-by-line comparison with the schematic. The generator catches only impossible configurations, not legal but wrong ones such as swapped encoder A/B channels, so the schematic comparison is still required.
- **The board definition is the only home of hardware facts**: pins, board devices, storage devices (ADR-0019), the Flash layout (ADR-0020) and the timing preset (ADR-0015). Ports are declared with their peripheral and pins but not their use, which the `SERIALn_FUNCTION` parameter assigns at runtime, so the same logical port may map to a different UART on another board without any parameter change.
- **MCU database**: converted offline from the STM32Cube data and committed together with its source, so builds need no network and are reproducible.
- **DMA solver**: the F4 has no DMAMUX, so each request can use only a fixed set of stream/channel pairs. The solver must be deterministic and report its assignment.

## Considered Options

- **A line-based format modelled on ArduPilot's `hwdef.dat`**: adopted at first, later dropped. The value lies in the toolchain behind it; a custom syntax needs a hand-written parser, while TOML inline tables give one pin per line just as well.
- **YAML board definitions**: rejected. A tree describing a flat table is verbose and hard to compare line by line with a schematic.
