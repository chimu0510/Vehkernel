# 05-bus: Publish/subscribe bus topics, publications and subscriptions

## What you learn

- How Vehkernel uses an internal publish/subscribe bus as its communications backbone rather than direct function calls or shared variables between modules (ADR-0002, ADR-0008).
- How topics are defined in ROS-style `.msg` schema files (`schema/msg/example_counter.msg`) with code generated at build time (ADR-0006).
- Why work items on different work queues (`wq:io` and `wq:slow`) must communicate through the bus rather than sharing memory (ADR-0008).
- Publishing messages with `vk::core::Publication<T>` into statically allocated circular buffers.
- Triggering event-driven work item execution upon publication using `trigger_on(vk::core::TopicBase&)` without timer polling.
- Subscribing to and draining queued topics (`@queue 4`) with `vk::core::Subscription<T>::copy()`.
- Why queued topics guarantee that no samples are missed even when the subscriber executes on a lower-priority work queue.
- Rendering live bus data on the SSD1306 local OLED display (`DISPLAY0`).
- Simulating rate-tiered bus communication in SITL and verifying sample ordering, rate consistency, and display output in automated L3 tests.

## Before this

- [`01-led`](../01-led/README.md): System clock initialization (`vk::hal::SystemClock::now_us()`), GPIO output, and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation (ADR-0007).
- [`04-work-queues`](../04-work-queues/README.md): Rate-tiered FreeRTOS work queues, periodic scheduling with `schedule_periodic()`, and why work items on the same work queue can share state safely without locks.

## Hardware

- **Board**: C50C V1.1 robot controller (`boards/c50c-v1_1/board.toml`).
- **Display (`DISPLAY0`)**: the SSD1306 OLED of [`03-oled`](../03-oled/README.md#hardware).
- **Connection**: Connect your development host to the board's one-key download port (`SERIAL2`).
- **Safety**: This example exercises the bus and drives the local OLED display. Motor outputs and actuators remain unconfigured and inactive.

## Build and flash

### Building for the board

Build with the `c50c-v1_1-examples` preset, then flash this example's image ([Board examples](../../../docs/guide/examples.md#flashing-an-example)):

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-05-bus.apj
```

### Running in SITL

`05-bus` also runs on the development host with the `sitl-examples` preset ([Running in SITL](../../../docs/guide/examples.md#running-in-sitl)).

Run the example executable directly, specifying duration and capturing the display output:

```bash
build/sitl-examples/app/examples/example-05-bus --duration 3 --display /tmp/display.pbm
```

When finished, [return to the firmware](../../../docs/guide/examples.md#returning-to-the-firmware).

## What you see

At boot, the FreeRTOS scheduler starts. The subscriber work item attaches as an event listener to the `example_counter` topic. The publisher work item begins running at 10 Hz on `wq:io`. Each publication notifies the subscriber on `wq:slow`, which wakes, drains all queued samples, and renders the updated screen.

The OLED display displays:
- Line 0: the header text `VEHKERNEL 05-BUS`.
- Line 2: the latest received counter value formatted as `COUNT: <n>`, starting at `1` and incrementing at 10 Hz (every 100 ms).
- Line 4: the cumulative count of samples received formatted as `RECEIVED: <n>`.
- All other lines remain blank.

In SITL, standard output prints report lines as samples are produced and consumed:
- `published=<n>` emitted 10 times per second by `PublisherItem` on `wq:io`.
- `received=<n>` emitted for each sample read by `SubscriberItem` on `wq:slow`.

When the configured duration expires, the simulated display reports `display_on=1`. The captured PBM image confirms whole frames: line 0 shows `VEHKERNEL 05-BUS`, line 2 shows `COUNT: <n>`, line 4 shows `RECEIVED: <n>`, and lines 1, 3, 5, 6, 7 are blank.

## The code

The complete source code is in `main.cpp`:

- **Includes and dependencies**:
  - `<line.hpp>`: provides the stack-based string and integer formatter (`example::Line`).
  - `<support.hpp>`: provides platform abstraction (`example::init()`, `example::display()`, `example::report()`, `example::start_scheduler()`).
  - `<vk/core/bus.hpp>`: provides the bus infrastructure: `vk::core::Topic`, `vk::core::Publication`, `vk::core::Subscription`, and `vk::core::topic()`.
  - `<vk/core/static_instance.hpp>`: provides static object storage without heap allocation (ADR-0007).
  - `<vk/core/work_queue.hpp>`: provides `vk::core::WorkItem`, `vk::core::WorkQueue`, and `vk::core::work_queue()`.
  - `<vk/msg/example_counter.hpp>`: generated header from `schema/msg/example_counter.msg` defining the `vk::msg::ExampleCounter` message struct.
- **The message schema (`schema/msg/example_counter.msg`)**:
  - The schema file defines `uint64 timestamp` and `uint32 count`, annotated with `@queue 4` and `@rate 10`.
  - The build-time schema generator (`tools/generators/generate_schema.py`, ADR-0006) emits `vk::msg::ExampleCounter`, including static metadata: `topic_name = "example_counter"`, `queue_length = 4`, `instances = 1`, and `schema_hash`.
  - Each topic's storage (`topic_instances<T>`) is statically allocated in RAM at build time; dynamic allocation (`malloc` or `new`) is never used (ADR-0007, ADR-0008).
- **Publisher work item (`PublisherItem`)**:
  - Subclasses `vk::core::WorkItem("publisher", queue)` and attaches to `wq:io` (`vk::core::WorkQueueTier::Io`).
  - In `start()`, schedules periodic execution at 10 Hz (`schedule_periodic(100'000)`).
  - In `run()`, increments `count_`, reports `published=<count>` via `example::report()`, populates `vk::msg::ExampleCounter` with `timestamp = queue().now_us()` and `count = count_`, and publishes it through `publication_.publish(msg)`.
- **Subscriber work item (`SubscriberItem`)**:
  - Subclasses `vk::core::WorkItem("subscriber", queue)` and attaches to `wq:slow` (`vk::core::WorkQueueTier::Slow`).
  - Holds an instance of `vk::core::Subscription<vk::msg::ExampleCounter> subscription_{0}`.
  - In `start()`, registers as an event-driven listener using `trigger_on(vk::core::topic<vk::msg::ExampleCounter>(0))`.
  - In `run()`, drains the subscription queue:
    ```cpp
    vk::msg::ExampleCounter sample{};
    while (subscription_.copy(sample)) {
        latest_count_ = sample.count;
        ++received_count_;
        example::report("received", sample.count);
    }
    ```
  - Formats lines 0, 2, and 4 on `screen_` using `Line()`, copies them to `display_.frame()`, and flushes modified pages via `display_.update(queue().now_us(), vk::drivers::MonoFrame::pages)`.
- **Why two different work queues**:
  - In `04-work-queues`, both work items ran on `wq:slow`. Because items on the same work queue run cooperatively and sequentially within a single FreeRTOS task, they could safely access each other's state without mutexes or critical sections.
  - In this example, `PublisherItem` runs on `wq:io` (backed by a FreeRTOS task at priority `configMAX_PRIORITIES - 4`) while `SubscriberItem` runs on `wq:slow` (priority `configMAX_PRIORITIES - 6`).
  - Because `wq:io` and `wq:slow` are distinct FreeRTOS tasks that preempt each other, sharing raw variables would introduce data races.
  - Data between queues travels exclusively through the bus (ADR-0008). Publications and reads are protected by short critical sections (`vk::core::CriticalSection`), ensuring thread safety without locking or deadlock risk.
- **Queued topic semantics (`@queue 4`)**:
  - By default, Vehkernel topics retain only the most recently published value (`queue_length = 1`).
  - Specifying `@queue 4` turns `example_counter` into a static circular queue with depth 4.
  - When `PublisherItem` publishes on `wq:io`, it calls `notify_listeners()`, which schedules `SubscriberItem` on `wq:slow`. If `wq:slow` is temporarily blocked (for instance, rendering an OLED frame over software SPI takes up to ~5.5 ms on the C50C V1.1), subsequent publications accumulate in the circular buffer.
  - When `SubscriberItem::run()` executes, `subscription_.copy(sample)` returns `true` for each buffered sample from oldest to newest, incrementing the internal subscription read pointer until all queued samples are consumed.
- **Startup sequence**:
  - `subscriber_item->start()` is called before `publisher_item->start()` to ensure the topic listener is registered before the first message is published.
  - `example::start_scheduler()` activates the FreeRTOS tasks for all rate tiers.

## Exercises

1. **Inspect topic status**: Call `vk::core::topic<vk::msg::ExampleCounter>(0).status()` to inspect `publications` and `rate_hz` recorded by the bus observability mechanism (ADR-0008, `bus.hpp`).
2. **Observe queue overflows**: Check `subscription_.missed()`. Add an artificial busy delay inside `SubscriberItem::run()` exceeding 400 ms (four publisher periods) to force the queue to overwrite unread messages, and observe how `missed()` tracks lost samples.
3. **Compare with latest-value semantics**: Consider the behavior if `@queue 4` were omitted in `example_counter.msg` (reverting to latest-value depth 1). If `wq:slow` were delayed past two publication cycles, `copy()` would only yield the newest sample, skipping intermediate counter values.

## Read further

- [ADR-0002: Bus backbone and pluggable Links](../../../docs/adr/0002-bus-backbone-and-pluggable-links.md)
- [ADR-0004: FreeRTOS as the RTOS](../../../docs/adr/0004-freertos.md)
- [ADR-0006: Schema and wire encoding](../../../docs/adr/0006-schema-and-wire-encoding.md)
- [ADR-0008: Bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [Bus header (`bus.hpp`)](../../../core/include/vk/core/bus.hpp)
- [Work queues (`work_queue.hpp`)](../../../core/include/vk/core/work_queue.hpp)
- [Example counter message (`example_counter.msg`)](../../../schema/msg/example_counter.msg)
- [Building guide](../../../docs/guide/building.md)
- [Examples guide](../../../docs/guide/examples.md)
- [C50C V1.1 board page](../../../docs/boards/c50c-v1_1.md)
