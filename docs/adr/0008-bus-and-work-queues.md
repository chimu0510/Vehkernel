# Bus semantics and a rate-tiered Work Queue execution model

The bus follows PX4's uORB, but everything is allocated statically at compile time to fit the limited RAM of MCUs such as the STM32F407, and execution is split into a few rate-tiered work queues instead of one task per module. Bus rules:

- **Semantics**: by default a topic keeps only its latest value. Subscribers check `updated()` against a generation counter and read with `copy()`. A schema annotation `@queue N` turns a topic into a queue where needed.
- **Allocation**: the generator creates all topics and buffers statically at compile time, including multiple instances (for example several IMUs). Nothing calls `malloc` at runtime.
- **Concurrency**: publish and read are protected by short critical sections. ISRs never publish; they notify the corresponding driver work item, which publishes. Work items on different queues talk only through the bus, never by calling each other: requests to the commander travel on `vehicle_command` and `vehicle_command_ack` (ADR-0005), so no work item locks another's state.
- **Timestamps**: every message carries a 64-bit monotonic microsecond `timestamp`, always as its first field (ADR-0006).
- **Observability**: the bus can list every topic with its publish rate and last update time, and expose them over a Link.

Rather than one task per module, execution is divided into rate tiers, each tier a work queue backed by one FreeRTOS task. Modules are work items attached to a work queue and triggered by a timer or by a topic publication. With the `IMU_EVENT` trigger (the trigger source is selectable, see ADR-0015) the control path is an event chain: the IMU driver publishes and wakes the estimator, the estimator publishes and wakes the controllers, then allocation and output follow, all on the highest-priority `wq:ctrl`. This implements the rate tiers of ADR-0001 and saves scarce RAM. Stacks go into core-coupled RAM where the MCU has it (CCM on the F4, DTCM on the H7), which the peripheral DMA controllers cannot reach, leaving the RAM they reach for DMA buffers. Bus buffers stay in normal RAM for now: on the C50C the firmware uses about a fifth of the F407's 128 KB of SRAM, so DMA buffers have room to spare. They move into core-coupled RAM when a board's normal RAM passes about 70 % in use, or with the first board whose DMA-reachable RAM is tight; that needs the core-coupled section zero-filled at startup, and no DMA transfer may then read or write a topic's storage directly.

## Considered Options

- **One FreeRTOS task per module**: rejected. Every task needs its own stack, which an MCU like the F407 cannot afford, and the tasks would have to synchronize with each other.
- **Publishing directly from ISRs**: rejected. It complicates the locking rules and lengthens ISRs.
