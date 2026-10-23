# Spec 0002: first IMU sample end to end

**Status**: done. Issue: [#14](https://github.com/chimu0510/Vehkernel/issues/14); tickets #15 to #22. The IMU wiring and read rate were confirmed on hardware in [#20](https://github.com/chimu0510/Vehkernel/issues/20), and IMU samples at 200 Hz on the C50C in [#22](https://github.com/chimu0510/Vehkernel/issues/22).

This spec is done and no longer edited; later decisions live in the ADRs and the code, which win where they differ.

## Problem Statement

The first milestone left Vehkernel with a build system, a board generator and a C50C V1.1 image that boots from S4 behind the S0 stub and then idles in WFI (confirmed on hardware, #10). Nothing runs yet: there is no RTOS, no bus, no work queues, no driver, and no way to see data move. Two facts the architecture depends on are still unconfirmed on real hardware: that the ICM20948 answers on PB9/PB8 over software I2C, and how fast that software I2C read really is (ADR-0015 estimates about 200 Hz). Every later module (estimators, control chain, Links) needs the bus and work queues to exist and needs real IMU samples to arrive on them.

## Solution

Deliver the second milestone: **the first IMU sample travels end to end**. Afterwards:

- The firmware starts FreeRTOS, configures the 168 MHz clock tree from the board definition, and runs a rate-tiered work queue that polls the ICM20948 over software I2C and publishes each sample on the bus as a typed topic.
- The same firmware runs as SITL on the FreeRTOS POSIX port, where the same ICM20948 driver talks to a simulated device on a simulated peripheral bus. A SITL run prints the publish rate of every topic and the timing monitor statistics of every work item.
- Topics are defined in `.msg` schema files; a schema generator emits typed C++ topics with static storage and a schema hash per message.
- A bring-up test build runs the first L4 cases on the board: read WHO_AM_I (expect `0xEA`) and measure the achievable software I2C read rate. A runner on the development host flashes the board over SWD, reads the results back and reports pass or fail.

## User Stories

1. As a firmware developer, I want FreeRTOS integrated on STM32F4, so that modules run as tasks with priorities instead of in a bare loop.
2. As a firmware developer, I want the same FreeRTOS kernel running on its POSIX port in SITL, so that work queues, the bus and scheduling behave as on hardware (ADR-0018).
3. As a firmware developer, I want the FreeRTOS kernel pinned to an exact release in the repository, so that builds stay offline and reproducible (ADR-0016).
4. As a firmware developer, I want the clock tree configured at boot from the board definition's solved PLL values, so that no clock constant is hard-coded in platform code.
5. As a firmware developer, I want a monotonic 64-bit microsecond clock, so that every topic message carries the timestamp ADR-0008 requires.
6. As a firmware developer, I want to define a topic once in a `.msg` file, so that its C++ type, storage and schema hash are generated from one source (ADR-0006).
7. As a firmware developer, I want the schema generator to reject a message whose first field is not a 64-bit `timestamp`, so that the rule of ADR-0006 is enforced at build time.
8. As a firmware developer, I want `@queue N` in the schema to turn a topic into a queue, so that batched IMU samples are not overwritten (ADR-0008, ADR-0015).
9. As a firmware developer, I want a schema hash per message, so that later protocols can compare definitions at the handshake.
10. As a firmware developer, I want schema errors reported with file and line, so that I can fix a definition quickly.
11. As a firmware developer, I want topics allocated statically at build time, so that nothing calls `malloc` at runtime (ADR-0007, ADR-0008).
12. As a firmware developer, I want topics to support several instances, so that a second IMU later needs no new topic type.
13. As a module author, I want to publish a typed message with one call, so that producing data is trivial.
14. As a module author, I want a subscription to tell me whether a topic was updated since I last read it, so that I process each sample once.
15. As a module author, I want to drain a queued topic in order, so that no sample of a burst is lost.
16. As a module author, I want my module to be a work item triggered by a timer or by a topic publication, so that I never manage tasks or stacks myself (ADR-0008).
17. As a module author, I want work queues split by rate tier, so that slow work never delays the control path.
18. As a firmware developer, I want ISRs never to publish, so that locking stays simple; ISRs notify a work item instead (ADR-0008).
19. As a firmware developer, I want the timing monitor to record run time, jitter and overruns for every work item, so that timing problems are visible (ADR-0015).
20. As a firmware developer, I want the timing monitor to publish its statistics on the bus, so that later Links and logs can carry them.
21. As a firmware developer, I want to list every topic with its publish rate and last update time, so that the bus is observable (ADR-0008).
22. As a driver author, I want an interface-layer contract for an I2C bus, so that drivers work unchanged on hardware I2C, software I2C and simulation (ADR-0009).
23. As a driver author, I want an interface-layer contract for GPIO pins built on the generated typed pins, so that a pin can only be used as its board role allows.
24. As a driver author, I want a software I2C implementation on two GPIO pins, so that the C50C IMU works despite its swapped wiring (ADR-0015).
25. As a driver author, I want software I2C to drive the lines open-drain, so that the level shifter and the device can pull them low safely.
26. As a C50C user, I want the ICM20948 driver to verify WHO_AM_I before configuring the device, so that a wrong or missing IMU is detected at boot.
27. As a C50C user, I want the IMU driver to configure accelerometer and gyroscope ranges and output rate, so that samples have known scaling.
28. As a C50C user, I want each IMU sample published with SI units, so that consumers never deal with raw register values.
29. As a C50C user, I want the IMU sample rotated by the mounting rotation from the board definition, so that consumers always see body-frame data (ADR-0014).
30. As a C50C user, I want the IMU read rate to come from the board's timing preset (200 Hz, timer-triggered, per-sample), so that the board definition stays the single source of timing defaults (ADR-0015).
31. As a firmware developer, I want the board generator to expose the board's default parameters to C++ as constants, so that the firmware can use them before the parameter system exists.
32. As a firmware developer, I want the IMU driver to report read failures and keep retrying, so that a loose cable does not stop the firmware.
33. As a firmware developer, I want the CPU to sleep in the idle task, so that an idle firmware wastes no power, while the debugger can still attach (DBGMCU kept on).
34. As a SITL user, I want the real ICM20948 driver to run against a simulated device, so that SITL exercises the same driver code as the board.
35. As a SITL user, I want the simulated device to produce plausible accelerometer and gyroscope data, so that downstream modules see realistic values.
36. As a SITL user, I want to run SITL for a given duration and get a report of topic rates and work item timing, so that I can check behaviour without hardware.
37. As a maintainer, I want CI to run SITL and fail if the IMU topic rate differs from the configured rate, so that scheduling regressions are caught automatically.
38. As a maintainer, I want unit tests for the bus, work queues and timing monitor at their public interfaces, so that their semantics are locked.
39. As a maintainer, I want the ICM20948 driver tested against a register model of the device on a fake peripheral bus, so that driver logic is covered without hardware (ADR-0018 L1).
40. As a maintainer, I want the simulated device in SITL and the register model in unit tests to be the same code, so that there is one model to maintain.
41. As a maintainer, I want schema generator golden tests like the board generator's, so that its outputs and diagnostics are locked.
42. As a maintainer, I want the generated topic code compiled by the test preset, so that generator output is checked by a real compiler.
43. As a maintainer, I want the layering rules to keep holding: drivers depend on the interface layer and core, never on the platform layer (ADR-0009).
44. As a maintainer, I want every new option in Kconfig with correct dependencies, for example the ICM20948 driver depending on the board having the device, so that builds only contain what the board supports (ADR-0017).
45. As a maintainer, I want Flash, RAM and CCM budgets to keep holding with FreeRTOS and the new modules linked in, so that growth stays visible.
46. As a board bring-up engineer, I want a bring-up test build that runs the L4 cases at boot, so that I can check new hardware quickly (ADR-0018).
47. As a board bring-up engineer, I want the first case to read the ICM20948 WHO_AM_I over software I2C and expect `0xEA`, so that the IMU wiring of ADR-0015 is confirmed on hardware.
48. As a board bring-up engineer, I want the second case to measure the duration of one IMU read and the achievable read rate, so that the 200 Hz estimate of ADR-0015 is replaced by a measurement.
49. As a board bring-up engineer, I want the results stored in a versioned block at a fixed RAM symbol, so that a debugger can read them without any Link.
50. As a board bring-up engineer, I want a runner on the development host that flashes the bring-up build over SWD, waits, reads the result block and prints a report, so that one command runs the whole check.
51. As a board bring-up engineer, I want the runner to exit non-zero when a case fails or the block is missing, so that it can later run on a self-hosted CI runner.
52. As a board bring-up engineer, I want the bring-up build never to drive motors or other actuators, so that it is safe to run with motors connected.
53. As a new contributor, I want the README to explain how to run SITL and the bring-up runner, so that I can reproduce both checks.

## Implementation Decisions

- **FreeRTOS**: the FreeRTOS kernel is pinned to an exact release and kept in the repository (submodule or vendored copy) so builds need no network (ADR-0016). The STM32 build uses the Cortex-M4F port; SITL uses the POSIX port. Each platform supplies its FreeRTOS configuration. Only `core/` and the platform layer call FreeRTOS directly (ADR-0004, ADR-0009).
- **Platform layer, STM32F4**: boot-time clock setup from the board's solved clock tree (HSE 8 MHz → 168 MHz, Flash wait states, APB prescalers); the FreeRTOS tick; a monotonic 64-bit microsecond clock that uses no timer the C50C board definition already assigns; GPIO configuration for the generated typed pins; software I2C on two GPIO pins driven open-drain at up to 400 kHz. The DBGMCU debug-in-sleep setting stays.
- **Platform layer, SITL**: FreeRTOS POSIX port, host time as the microsecond clock, and a simulated peripheral bus to which simulated devices attach.
- **Interface layer**: contracts for a microsecond clock, GPIO pins and an I2C bus (addressed write, read and register read), checked with C++20 concepts on the platform side. Drivers depend only on these contracts and `core/`.
- **Schema generator**: a command-line generator, following the board generator's conventions, that reads `.msg` files with annotations (`@queue N`, units, nominal rate), enforces a leading `uint64 timestamp`, and emits typed C++ topic structs, a static topic registry with storage for each instance, and a schema hash per message. Encoders and decoders for protocols are not generated yet. It runs as a CMake custom command and its output is not committed (ADR-0017).
- **First topics**: an IMU sample topic (timestamp, device instance, accelerometer in m/s², gyroscope in rad/s, temperature), declared with a queue large enough for one batch; and a work item status topic carrying the timing monitor statistics.
- **Bus (core)**: latest-value topics by default and queued topics with `@queue N`; subscriptions track a generation counter for `updated()` and read with `copy()`; queued subscriptions drain in order and report missed samples; short critical sections protect publish and read; no ISR publishes. The bus can enumerate topics with publish rate and last update time.
- **Work queues (core)**: a small fixed set of rate-tiered work queues, each one FreeRTOS task, including `wq:ctrl` at the highest priority. Work items run on a timer period or when a subscribed topic is published.
- **Timing monitor (core)**: measures each work item's run time, jitter and overruns using the microsecond clock and publishes them on the work item status topic.
- **Board generator extension**: board default parameters (including the timing preset) are also emitted as compile-time constants in the board header, until the parameter system exists (ADR-0012).
- **ICM20948 driver (driver layer)**: verifies WHO_AM_I, resets and configures the device, reads accelerometer, gyroscope and temperature per sample over the I2C contract, converts to SI units, applies the board device's mounting rotation, and publishes the IMU sample topic. It runs as a timer-triggered work item at the board's base rate. Read failures are counted and retried without stopping the firmware. The magnetometer inside the ICM20948 is not used yet.
- **ICM20948 device model**: one register-level model of the ICM20948, used both as the fake device in unit tests and as the simulated device in SITL, producing configurable accelerometer and gyroscope values.
- **Firmware entry points**: the firmware and SITL entry points initialise the platform, start the work queues and the IMU driver, and start the scheduler. The idle task sleeps with WFI on hardware.
- **SITL report**: SITL accepts a run duration, then stops and prints each topic's publish count and rate and each work item's timing statistics in a stable, parseable format.
- **Bring-up test build**: a Kconfig option builds a bring-up firmware instead of the normal application. At boot it runs the bring-up cases, writes a versioned result block (magic, version, per-case status and measured values) to a fixed RAM symbol, and idles. It never configures actuator outputs.
- **Bring-up runner**: a development-host tool that builds or takes the bring-up image, flashes it over SWD with OpenOCD, waits, reads the result block, prints a report and exits non-zero on failure or a missing block.
- **Kconfig**: new options for the ICM20948 driver (depending on the board having an ICM20948, or on SITL), software I2C (depending on a board device on `SOFT_I2C`, or SITL), and the bring-up test build.

## Testing Decisions

- **Good tests check external behaviour only**: generator command-line output, the public interfaces of the bus, work queues and drivers, SITL's printed report, and the bring-up runner's report. Internal state such as generation counters or scheduler internals is never asserted directly.
- **Seam A: schema generator command line.** Golden tests with fixed `.msg` inputs and expected outputs, and one failing input per validation rule checking file and line, following the board generator's test layout. Generated code for every golden case is compiled by the test preset.
- **Seam B: host unit tests (L1, GoogleTest).** The bus (latest value, `updated()`, queued topics, multiple instances, missed-sample reporting), work queues and the timing monitor at their public interfaces, and the ICM20948 driver against the device register model on a fake peripheral bus: WHO_AM_I mismatch, configuration, scaling, rotation and read-failure handling.
- **Seam C: SITL smoke test (L3).** CI runs the SITL executable for a fixed duration and checks from its report that the IMU sample topic is published at the configured rate within a tolerance and that no work item overran.
- **Seam D: hardware bring-up (L4, manual for now).** The runner flashes the C50C over SWD and reads the result block. WHO_AM_I must equal `0xEA`; the measured read duration and achievable rate are recorded on the ticket and used to confirm or correct the board's timing preset.
- **Prior art**: the board generator's golden tests (seam A), the existing GoogleTest suite and CMake smoke tests (seams B and C), and the SWD boot verification recorded on #10 (seam D).

## Out of Scope

- Links, protocols, encoders and decoders, the Bridge and anything ROS2.
- Estimators, the control chain, vehicle models and actuator control.
- The parameter system runtime and parameter storage; board defaults are compile-time constants only.
- Logging and log sinks.
- Modes, setpoint arbitration and arming.
- Hardware I2C, SPI, DMA-driven transfers, interrupt-driven IMU triggering and FIFO batch reading; the C50C uses the timer-triggered per-sample path.
- The ICM20948 magnetometer and any other device driver (encoders, motors, RC input, CAN, USB, OLED).
- The external simulator interface and vehicle physics in SITL.
- Running L4 on a self-hosted CI runner.

## Further Notes

- **Interim deviation from ADR-0018 (results over a Link)**: bring-up results are read from a RAM block over SWD, because no Link exists yet. Once the first Link lands, results move to it and ADR-0018's design applies unchanged.
- **Interim deviation from ADR-0018 (test mode as a mode)**: the bring-up runs as a separate Kconfig build, because modes and setpoint arbitration (ADR-0005) are out of scope. It becomes a proper test mode once modes exist.
- The C50C IMU sits behind the RS0102 level shifter at 7-bit address `0x68` (the vendor firmware's 8-bit `0xD0`), with PB9 = SCL and PB8 = SDA, the reverse of hardware I2C1 (ADR-0015). Its data-ready interrupt on PD3 is declared but not used in this milestone.
- The boot path, VTOR and BOOT0 behaviour were confirmed on hardware in #10; this milestone builds on that image layout.
- The measured software I2C rate may change the C50C timing preset; if it does, update the board definition and the example in ADR-0015 together.

