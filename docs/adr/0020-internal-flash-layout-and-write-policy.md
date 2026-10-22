# Internal Flash layout and write policy: write but never erase while armed

The STM32F407 Flash has a single bank. During an erase or write, every instruction or constant fetch from Flash stalls, including ISRs and `wq:ctrl`. Writing one 32-bit word stalls for only about 16 µs, but erasing a sector takes hundreds of milliseconds up to 2 seconds, long enough to stall the control chain. Parameter storage (ADR-0012) and the event log (ADR-0019) both write internal Flash, so they need one layout and one set of write rules.

**Layout**

| Sector | Size | Use |
|---|---|---|
| S0 | 16 KB | Bootloader (ADR-0023); until it exists, a stub that jumps to the firmware |
| S1–S2 | 16 KB × 2 | Parameter storage, two alternating sectors (ADR-0012) |
| S3 | 16 KB | Event log (ADR-0019) |
| S4–S7 | 64 KB + 128 KB × 3 | Firmware, 448 KB contiguous; VTOR points the vector table at S4 |

- The board TOML declares the layout, and the sector table comes from the MCU database (ADR-0011). The generator derives the linker script memory regions from it and uses the firmware region size as the Flash budget of ADR-0017.
- The table is the C50C's. Every board declares its own layout. A board that keeps an ArduPilot or PX4 bootloader declares the layout that bootloader expects, for example a 128 KB bootloader region on STM32H743 boards (ADR-0023).
- The bootloader erases only the firmware region, so the parameter and event log sectors survive firmware updates. Until the bootloader exists, SWD and the STM32 system bootloader write the firmware region only, and a new board's whole chip is erased once.

**Write policy**

- **Erase only while disarmed.** At boot, before arming is allowed, the spare parameter sector is erased.
- **While armed, write but never erase.** Writes happen one word at a time in batches, with a cap on words per run so that each stall is bounded. The timing monitor measures and publishes the stall time (ADR-0015).
- **Parameters**: when the active sector fills up, the current values are compacted into the erased spare sector in batches, which is allowed while armed; erasing the old sector waits until disarm. If both sectors run out of space while armed, new changes stay in RAM, marked unsaved, and are written after disarm.
- **Event log**: when an event occurs, the RAM ring buffer is written to S3 in batches. If S3 still holds a log that has not been read out, the old log is kept, the new event is not written, and this is reported over a Link. S3 is erased, while disarmed, only after the log has been read out over a Link or explicitly cleared by the user.

## Considered Options

- **No Flash writes at all while armed**: rejected. Parameter changes and event logs would wait until disarm, yet events usually happen while armed, and a power loss then destroys exactly the data that matters most.
- **Running the control path from SRAM**: rejected. The F407's CCM cannot execute code, so the control chain, FreeRTOS, ISRs and drivers would all have to move into normal SRAM, taking memory DMA needs and costing maintenance.
- **Firmware starting at S0 with the parameter sectors in between**: rejected. It gains 16 KB but splits the firmware in two and forces a new layout once a bootloader is added.
