# 04-work-queues: Rate-tiered FreeRTOS work queues and timed work items

## What you learn

- How Vehkernel uses FreeRTOS tasks to implement rate-tiered work queues (`vk::core::WorkQueueTier`, `vk::core::WorkQueue`) rather than dedicating one task per module (ADR-0004, ADR-0008).
- Why rate-tiered work queues conserve MCU SRAM by sharing a single task stack across multiple cooperative work items.
- Defining timed work items by subclassing `vk::core::WorkItem` and implementing the `run()` lifecycle method.
- Scheduling work items periodically with `schedule_periodic(interval_us)` at distinct rates (10 Hz and 2 Hz).
- Why attaching both work items to `wq:slow` (`vk::core::WorkQueueTier::Slow`) enables safe state sharing without mutexes or critical sections, and why display rendering belongs on `wq:slow` (ADR-0022).
- Starting the FreeRTOS scheduler through `example::start_scheduler()` (`vk::core::start_scheduler()`).
- Simulating rate-tiered execution under SITL host simulation and verifying rate accuracy, gap-free sequencing, and display output in automated L3 checks.

## Before this

- [`01-led`](../01-led/README.md): System clock initialization (`vk::hal::SystemClock::now_us()`), basic GPIO output, and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation (ADR-0007).

## Hardware

- **Board**: C50C V1.1 robot controller (`boards/c50c-v1_1/board.toml`).
- **Display (`DISPLAY0`)**: 0.96-inch 128 x 64 monochrome OLED display based on the SSD1306 controller, connected via software SPI:
  - `OLED_SCL` (`PD7`): SPI clock.
  - `OLED_SDA` (`PD6`): SPI MOSI (data).
  - `OLED_RST` (`PD5`): Active-low reset.
  - `OLED_DC` (`PD4`): Data/command select line.
- **Connection**: Connect your development host to the board's one-key download port (`SERIAL2`).
- **Safety**: This example exercises the RTOS scheduler and drives the local OLED display. Motor outputs and actuators remain unconfigured and inactive.

## Build and flash

### Building for the board

Build the example with the `c50c-v1_1-examples` preset inside the development container:

```bash
cmake --preset c50c-v1_1-examples
cmake --build --preset c50c-v1_1-examples
```

Flash the compiled example (`.apj`) over the one-key download port using the serial flasher tool:

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-04-work-queues.apj
```

Global options (`--port`, `--build`) go before the `update` subcommand. An example runs no Link, so `update` restarts the board through `SERIAL2`'s download circuit instead; on `SERIAL1` the user must power-cycle the board while `update` waits (see [C50C V1.1](../../../docs/boards/c50c-v1_1.md#serial-ports)).

When finished with this example, return the board to the standard firmware using the default build:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

### Running in SITL

`04-work-queues` also builds and runs on your development host in SITL simulation with the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

Run the example executable directly, specifying duration and capturing the display output:

```bash
build/sitl-examples/app/examples/example-04-work-queues --duration 3 --display /tmp/display.pbm
```

To run the automated L3 scenario check in SITL:

```bash
ctest --preset sitl-examples
```

## What you see

At boot, the FreeRTOS scheduler starts and both work items begin running at their scheduled intervals.
The OLED display initializes automatically through the faster item's initial update cycles and displays:
- Line 0: the header text `VEHKERNEL 04-WQ`.
- Line 2: the faster item's counter formatted as `FAST: <n>`, starting at `1` and incrementing at 10 Hz (every 100 ms).
- Line 4: the slower item's counter formatted as `SLOW: <n>`, starting at `1` and incrementing at 2 Hz (every 500 ms).
- All other lines remain blank.

In SITL, standard output prints report lines on each run:
- `fast=<n>` emitted 10 times per second.
- `slow=<n>` emitted 2 times per second.

When the configured duration expires, the simulated display reports `display_on=1`. The captured PBM image confirms whole frames: line 0 shows `VEHKERNEL 04-WQ`, line 2 shows the final reported fast count, line 4 shows the final reported slow count, and lines 1, 3, 5, 6, 7 are blank.

## The code

The complete source code is in `main.cpp`:

- **Includes and namespaces**: `<line.hpp>` provides the heap-free string and integer formatter (`example::Line`). `<support.hpp>` provides the platform abstraction (`example::init()`, `example::display()`, `example::report()`, `example::start_scheduler()`). `<vk/core/work_queue.hpp>` provides `vk::core::WorkItem` and `vk::core::work_queue()`. `<vk/core/static_instance.hpp>` provides static object storage without dynamic allocation (ADR-0007).
- **Work items**:
  - `SlowItem`: Subclasses `vk::core::WorkItem("slow", queue)`. Defines `interval_us = 500'000` (2 Hz). In `start()`, it schedules itself with `schedule_periodic(interval_us)`. In `run()`, it increments `count_` and emits `example::report("slow", count_)`.
  - `FastItem`: Subclasses `vk::core::WorkItem("fast", queue)`. Holds a reference to `SlowItem` and the display driver. Defines `interval_us = 100'000` (10 Hz). In `start()`, it schedules itself with `schedule_periodic(interval_us)`. In `run()`, it increments `count_`, emits `example::report("fast", count_)`, formats the screen lines using `Line()`, renders into `display.frame()`, and flushes modified pages via `display.update(queue().now_us(), vk::drivers::MonoFrame::pages)`.
- **Why one work queue (`wq:slow`)**:
  - In Vehkernel, execution is divided into three rate tiers: `wq:ctrl` (highest priority, control loops and IMU), `wq:io` (intermediate priority, communication Links), and `wq:slow` (lowest priority, housekeeping, display and menus). Each queue is backed by exactly one FreeRTOS task (ADR-0008).
  - Attaching both `SlowItem` and `FastItem` to `wq:slow` demonstrates cooperative multitasking: work items on the same queue execute sequentially to completion without preempting each other, sharing a single task stack and saving precious SRAM.
  - Because work items on the same queue cannot preempt each other, `FastItem` can read `SlowItem::count()` directly without mutexes, spinlocks or critical sections.
  - If work items ran on different work queues (such as `wq:ctrl` and `wq:slow`), direct state access would violate thread safety; cross-queue communication is reserved for the bus (`05-bus`). Furthermore, writing a full screen to the SSD1306 over software SPI busy-waits for up to about 5.5 ms ([C50C V1.1](../../../docs/boards/c50c-v1_1.md#local-display-and-user-key)), which belongs on `wq:slow` rather than the high-priority real-time control queue (`wq:ctrl`).
  - Attaching `SlowItem` before `FastItem` ensures that when both are due simultaneously (every 500 ms), `SlowItem` runs first and `FastItem` runs second, so the display frame rendered by `FastItem` always shows both counters up to date.
- **Static storage**: Using `vk::core::StaticInstance` ensures work items live in static memory without dynamic allocation (`malloc` or `new`), adhering to ADR-0007.
- **Starting the scheduler**: `example::start_scheduler()` hands CPU control over to FreeRTOS. On hardware it starts the FreeRTOS scheduler and never returns. In SITL, it configures a stop task matching `--duration`, runs until stopped, reports `display_on`, and exits cleanly with status 0.

## Exercises

1. **Inspect timing statistics**: Call `stats()` on `slow_item` or `fast_item` to inspect `runs`, `overruns`, and `jitter_max_us` tracked by `vk::core::WorkItem`.
2. **Alter update rates**: Change `FastItem::interval_us` to `50'000` (20 Hz) or `SlowItem::interval_us` to `1'000'000` (1 Hz) and observe the paced output and display refresh.
3. **Explore event triggers**: Check `core/include/vk/core/work_queue.hpp` for `schedule_now()` and `trigger_on()`, which allow work items to wake immediately or upon topic publication rather than on a timer, preparing for `05-bus`.

## Read further

- [ADR-0004: FreeRTOS as the RTOS](../../../docs/adr/0004-freertos.md)
- [ADR-0007: C++ subset for the firmware](../../../docs/adr/0007-cpp20-subset.md)
- [ADR-0008: Bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0018: Test layers and SITL](../../../docs/adr/0018-test-layers-and-sitl.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [Work queues (`work_queue.hpp`)](../../../core/include/vk/core/work_queue.hpp)
- [Scheduler (`scheduler.hpp`)](../../../core/include/vk/core/scheduler.hpp)
- [StaticInstance (`static_instance.hpp`)](../../../core/include/vk/core/static_instance.hpp)
- [Building guide](../../../docs/guide/building.md)
- [Examples guide](../../../docs/guide/examples.md)
- [C50C V1.1 board page](../../../docs/boards/c50c-v1_1.md)
