# 09-parameters: Parameter system and persistent storage in non-volatile Flash (PARAMS)

## What you learn

- How the Vehkernel parameter architecture ([ADR-0012](../../../docs/adr/0012-parameter-system.md)) combines direct C++ struct access in RAM with robust non-volatile Flash storage. In RAM, parameter groups generated from `params.toml` are plain structs that modules read directly with zero lookup overhead. In Flash, non-default values are stored entry by entry, keyed by 32-bit FNV-1a parameter name hashes (`vk::core::param_hash`), ensuring that firmware updates and group layout changes never invalidate stored user calibrations or settings.
- How parameters are defined in TOML files, specifying name, type (`INT32` or `FLOAT`), unit, range (`min`, `max`), default value, and safety attributes. The parameter generator translates definitions into plain structs (`vk::params::...`), accessors, parameter tables, and build metadata (`build/<preset>/generated/params/params.json`).
- Safety attributes: `reboot_required` (parameters that only take effect upon a subsequent MCU reboot) and `locked_when_armed` (parameters that refuse changes while the vehicle is armed).
- The internal Flash layout and write policy on single-bank STM32F407 MCUs ([ADR-0020](../../../docs/adr/0020-internal-flash-layout-and-write-policy.md)):
  - Sector allocation: two alternating 16 KB sectors, S1 and S2 (32 KB total `PARAMS` region).
  - Sector header: 8 bytes consisting of a 4-byte magic word (`0x52415056` "VPAR") and a 4-byte generation counter. The magic word is written last so an incomplete header is never treated as valid; the valid sector with the highest generation is the active sector.
  - Append-only log entries: 12 bytes per entry (word 0: name hash; word 1: 32-bit value bits; word 2: type byte, `0xFF` padding, and CRC16-CCITT checksum over the preceding 10 bytes).
  - Write policy: erase only while disarmed; at boot, before arming is possible, the spare sector is erased. While armed, writes occur in bounded word batches (6 words per run on `wq:slow`) to bound Flash stall times (~16 µs per 32-bit word). When the active sector fills up, current non-default values are compacted into the spare sector in batches; erasing the old sector waits until disarm.
- Link parameter access ([ADR-0002](../../../docs/adr/0002-bus-backbone-and-pluggable-links.md)): `ParameterAccess` in `vk::link::Link::Config` provides a protocol-independent parameter service. Custom protocol control frames (`param_list`, `param_get`, `param_set`) allow host tools such as `link_monitor.py` to inspect and update parameters over serial ports (`SERIAL1`, `SERIAL2`) or UDP in SITL simulation.
- Bus communication and event notification: `vk::msg::ParameterUpdate` is published on the bus whenever a parameter value changes, allowing consumers to re-read updated values event-driven rather than polling. `vk::msg::ParameterStatus` is published periodically at 1.0 Hz on the bus and streamed over the Link, reporting active sector, bytes used, unsaved change count, pending reboot count, and boot skip counters.
- Visualizing parameter state on the SSD1306 OLED display (`DISPLAY0`) at 4 Hz on `wq:slow` ([ADR-0022](../../../docs/adr/0022-local-display-and-local-menu.md)): parameter name, current value formatted with fixed decimal places via `Line::add_float()`, save status (`SAVED: YES` or `SAVED: NO`), bytes used in the active Flash sector, and active sector index (0 or 1).
- Why `ATT_KP` is chosen: `ATT_KP` is an existing firmware parameter defined in [`modules/estimation/params.toml`](../../../modules/estimation/params.toml) (the Mahony attitude filter's attitude correction gain, default 1.0 1/s, range 0.0 to 10.0 1/s). Because `09-parameters` does not instantiate or run the attitude estimator or any vehicle control modules, changing `ATT_KP` changes nothing on the vehicle while the example runs.
- **Important note on persistence**: Any parameter modified over the Link is written directly to the board's non-volatile `PARAMS` Flash sectors (or to the simulated Flash file in SITL) and persists across reboots and firmware updates. When returning to standard vehicle firmware, the changed `ATT_KP` will take effect in the firmware's attitude estimator. To restore the standard default, run `tools/link/link_monitor.py ... param set ATT_KP 1.0`.
- What this example leaves out: vehicle control commands and commander requests (`ControlAccess`, [ADR-0005](../../../docs/adr/0005-setpoint-arbitration.md)), attitude estimators ([ADR-0014](../../../docs/adr/0014-estimation-architecture.md)), actuator control, and log recording over Link.
- Why returning to firmware via `flash_serial.py update` restarts the board through SERIAL2's hardware download circuit: `update` attempts to request a reboot into the bootloader over the Link. The Link passes reboot requests on to the commander as a `vehicle_command`, but only when its `ControlAccess` is enabled; `09-parameters` leaves it disabled and runs no commander, so the Link ignores the request and `link_monitor.py` gets no reply, and `flash_serial.py` automatically restarts the board through SERIAL2's hardware download circuit (asserting DTR and RTS to pulse NRST with BOOT0 high). On SERIAL1, the user must power-cycle the board when prompted.

## Before this

- [`01-led`](../01-led/README.md): System clock initialization (`vk::hal::SystemClock::now_us()`), GPIO output, and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation ([ADR-0007](../../../docs/adr/0007-cpp20-subset.md)).
- [`04-work-queues`](../04-work-queues/README.md): Rate-tiered FreeRTOS work queues (`wq:ctrl`, `wq:io`, `wq:slow`), periodic scheduling with `schedule_periodic()`, and cooperative scheduling within a queue.
- [`05-bus`](../05-bus/README.md): Publish/subscribe bus backbone, schema-generated topics, `vk::core::Publication<T>`, `vk::core::Subscription<T>`, and event-driven wakeups with `trigger_on()`.
- [`08-serial-link`](../08-serial-link/README.md): Custom protocol Link over serial ports (`SERIAL1`, `SERIAL2`) or UDP in SITL, handshake version negotiation, time synchronization, and streaming topics.

## Hardware

- **Board**: C50C V1.1 robot controller ([`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)).
- **Internal Flash layout (`STM32F407VG`)** ([ADR-0020](../../../docs/adr/0020-internal-flash-layout-and-write-policy.md)):
  - Sector 0 (`S0`, 16 KB): Vehkernel bootloader region (`flash:BOOT`).
  - Sectors 1 and 2 (`S1`, `S2`, 16 KB × 2): Parameter storage alternating sectors (`flash:PARAMS`).
  - Sector 3 (`S3`, 16 KB): Event log region (`flash:EVENTLOG`).
  - Sectors 4 to 7 (`S4`–`S7`, 448 KB contiguous): Firmware region (`flash:FIRMWARE`).
- **Serial port 1 (`SERIAL1`)**: Type-C port:
  - Connector: USB Type-C.
  - Bridge: WCH CH9102F USB-to-UART bridge.
  - Peripheral: `USART3`, TX `PB10`, RX `PB11` ([`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#serial-ports)).
  - Default baud rate: 921,600 baud 8N1 (`vk::board::defaults::SERIAL1_BAUD`).
- **Serial port 2 (`SERIAL2`)**: One-key download port:
  - Connector: USB Type-C with auto-download circuit (DTR/RTS driving NRST and BOOT0).
  - Bridge: WCH CH9102F USB-to-UART bridge.
  - Peripheral: `USART1`, TX `PA9`, RX `PA10` ([`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#serial-ports)).
  - Default baud rate: 921,600 baud 8N1 (`vk::board::defaults::SERIAL2_BAUD`).
- **Display (`DISPLAY0`)**: 0.96-inch 128 x 64 monochrome OLED on software SPI (`PD7` SCL, `PD6` SDA, `PD5` RST, `PD4` DC).
- **Safety**: Motors and actuator outputs remain unconfigured and inactive.

## Build and flash

### Building for the board

Build the example with the `c50c-v1_1-examples` preset inside the development container:

```bash
cmake --preset c50c-v1_1-examples
cmake --build --preset c50c-v1_1-examples
```

Flash the compiled example (`.apj`) over the one-key download port (`SERIAL2`) using the serial flasher tool:

```bash
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-09-parameters.apj
```

Global options (`--port`, `--build`) precede the `update` subcommand. `flash_serial.py update` attempts to request a reboot into the bootloader over the Link. Because this example leaves `ControlAccess` disabled and runs no commander, the Link ignores the request and no reply comes, and `flash_serial.py` automatically restarts the board through SERIAL2's hardware download circuit (asserting DTR and RTS to pulse NRST with BOOT0 high). On SERIAL1, power-cycle the board when prompted.

### Connecting with the Link monitor on the board

Connect a USB cable to either `SERIAL1` or `SERIAL2`. The USB serial adapter appears on the host as `/dev/ttyACM0` (or `/dev/ttyACM1`). List all parameters using `link_monitor.py`:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 \
    --msg-dir build/c50c-v1_1-examples/generated/schema/python \
    --param-meta build/c50c-v1_1-examples/generated/params/params.json param list
```

Read the current value of `ATT_KP`:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 \
    --msg-dir build/c50c-v1_1-examples/generated/schema/python \
    --param-meta build/c50c-v1_1-examples/generated/params/params.json param get ATT_KP
```

Set `ATT_KP` to `2.5`:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 \
    --msg-dir build/c50c-v1_1-examples/generated/schema/python \
    --param-meta build/c50c-v1_1-examples/generated/params/params.json param set ATT_KP 2.5
```

### Running in SITL

`09-parameters` builds and runs in SITL simulation on the development host with the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

Run the example executable directly in one terminal, specifying duration 0 (runs until interrupted), listening for UDP connections, and backing parameter storage with a file:

```bash
build/sitl-examples/app/examples/example-09-parameters --duration 0 --udp 127.0.0.1:15600 --param-file /tmp/vk-params.bin
```

In a second terminal, inspect and set parameters over UDP:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 param get ATT_KP
tools/link/link_monitor.py --udp 127.0.0.1:15600 param set ATT_KP 2.5
```

To run the automated L3 scenario check in SITL:

```bash
ctest --preset sitl-examples -R example_09_parameters
```

### Returning to the firmware

When finished with this example, return the board to standard vehicle firmware:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

As with flashing an example, when its Link reboot request gets no reply, it restarts the board through SERIAL2's download circuit. Parameter sectors (`PARAMS`) remain intact.

> [!IMPORTANT]
> The modified `ATT_KP` remains stored in Flash. If you changed `ATT_KP` during testing, restore its default value of `1.0` using `link_monitor.py`:
> ```bash
> tools/link/link_monitor.py --serial /dev/ttyACM0 param set ATT_KP 1.0
> ```

## What you see

When connecting to `09-parameters`, `link_monitor.py` completes the handshake, reporting the firmware's parameter count and matching parameter set hash.

Reading `ATT_KP` before making changes prints:

```text
param name=ATT_KP value=1 type=FLOAT unit=1/s default=1 min=0 max=10 description="Attitude correction gain: how fast roll and pitch follow the accelerometer."
```

Setting `ATT_KP` to `2.5` updates the value in RAM immediately and returns a reply marking it unsaved until written to Flash:

```text
param name=ATT_KP value=2.5 type=FLOAT unsaved=1 unit=1/s default=1 min=0 max=10 description="Attitude correction gain: how fast roll and pitch follow the accelerometer."
```

Querying `param get ATT_KP` shortly afterwards (after the background storage item on `wq:slow` commits the entry to Flash) returns the confirmed value without `unsaved=1`:

```text
param name=ATT_KP value=2.5 type=FLOAT unit=1/s default=1 min=0 max=10 description="Attitude correction gain: how fast roll and pitch follow the accelerometer."
```

### Observing the SSD1306 OLED display

The OLED screen updates at 4 Hz on `wq:slow`:

- Line 0: Header text `VEHKERNEL 09-PARAMS`.
- Line 1: Blank.
- Line 2: Monitored parameter name: `PARAM: ATT_KP`.
- Line 3: Parameter value: `VALUE: 1.00`, transitioning to `VALUE: 2.50` upon update.
- Line 4: Non-volatile save status: `SAVED: YES`, transitioning briefly to `SAVED: NO` while unsaved changes are queued, returning to `SAVED: YES` once written to Flash.
- Line 5: Flash sector utilization: `BYTES USED: 8` at initial boot (8-byte sector header), increasing to `BYTES USED: 20` once the 12-byte entry is committed.
- Line 6: Active Flash sector index: `SECTOR: 0` (or `SECTOR: 1`).
- Line 7: Blank.

### Observing persistence across reset

To confirm persistence:
1. In SITL, terminate the running process (`Ctrl+C`) and restart it pointing to the same parameter file (`--param-file /tmp/vk-params.bin`).
2. On hardware, disconnect USB or reset the board.
Upon reboot, the OLED immediately displays `VALUE: 2.50` and `SAVED: YES` at boot, before any host connects or sends Link commands, demonstrating that `ParameterStorage::load()` successfully read the non-default value from the `PARAMS` region.

In SITL simulation, standard output prints:
- `link transport=udp address=127.0.0.1:<port>` when the UDP Link starts.
- `param=ATT_KP`, `value=2.500`, `saved=1`, `bytes_used=20`, `sector=0` emitted by the display work item at 4 Hz.
- `display_on=1` emitted when SITL shuts down.

## The code

The complete source code is in `main.cpp`:

- **Dedicated parameter topic map (`ExampleStreams`, `example_topic_map`)**:
  - The example defines a minimal topic map streaming only what is relevant to parameter management:
    ```cpp
    struct ExampleStreams {
        vk::link::TopicStreamOf<vk::msg::ParameterStatus> param_status{status_rate, 0};
        vk::link::TopicStreamOf<vk::msg::ParameterUpdate> param_update{vk::link::every_sample, 0, 1.0F};
        std::array<vk::link::TopicStreamOf<vk::msg::LinkStatus>, vk::msg::LinkStatus::instances> links{
            vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 0},
            vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 1},
        };
        std::array<vk::link::TopicStream*, 2 + vk::msg::LinkStatus::instances> map{
            &param_status, &param_update, &links[0], &links[1],
        };
    };
    ```
  - `param_status`: streams `vk::msg::ParameterStatus` once per second (`status_rate = 1.0F`), reporting Flash sector usage, unsaved parameter count, and boot verification results.
  - `param_update`: streams `vk::msg::ParameterUpdate` on every publication (`every_sample`), notifying external Link monitors whenever a parameter changes.
  - `links`: two `TopicStreamOf<msg::LinkStatus>` streams at 1.0 Hz, one per Link instance.
- **Platform parameter storage in support code (`example::start_parameters`)**:
  - `main.cpp` calls `example::start_parameters()`, which encapsulates platform-specific Flash drivers in support code (`examples/support/`).
  - On the board (`examples/support/stm32f4/support.hpp`), it maps `vk::platform::InternalFlash` to `vk::board::flash::PARAMS` (sectors S1 and S2).
  - In SITL (`examples/support/sitl/support.hpp`), it binds `vk::sim::NorFlash` to the file passed with `--param-file <path>` (or in-memory blank Flash if omitted).
  - It constructs `vk::core::ParameterStorage` on `wq:slow`, calls `param_storage->load()` at boot (which validates headers, loads non-default values into RAM structs, and erases the spare sector), and calls `param_storage->start()` to run background Flash writes.
- **Link parameter integration (`example::start_links`)**:
  - Once `example::start_parameters()` has run, the support code gives each Link the parameter access the firmware gives it: the parameter set, its set hash and the storage, for the flags of each `param_value`. Without it, as in `08-serial-link`, the Link answers no parameter request.
  - Allows `link_monitor.py param list`, `param get`, and `param set` control frames to query and update the runtime parameters.
- **Display work item (`DisplayItem`)**:
  - Subclasses `vk::core::WorkItem("display", queue)` and attaches to `wq:slow`.
  - Configured with `trigger_on(vk::core::topic<vk::msg::ParameterUpdate>(0))` and `schedule_periodic(250'000)`.
  - When `parameter_update` is published on the bus, `DisplayItem` wakes up and re-reads `vk::params::att.kp` directly from RAM without polling.
  - Subscribes to `parameter_status` to observe `unsaved`, `bytes_used`, and `active_sector`.
  - Formats all information using stack-allocated `Line` buffers without dynamic memory allocation ([ADR-0007](../../../docs/adr/0007-cpp20-subset.md)).

## Exercises

1. **Observe log compaction**: Connect with `link_monitor.py` and repeatedly set `ATT_KP` to alternating values (e.g., `2.0`, `2.5`, `3.0`). Notice that on each change, `BYTES USED` increases by 12 bytes (`entry_bytes`). When the active sector reaches capacity, observe that `ParameterStorage` compacts current values into the spare sector and increments `SECTOR` between 0 and 1.
2. **Test out-of-range rejection**: Try setting `ATT_KP` outside its declared range `[0.0, 10.0]`:
   ```bash
   tools/link/link_monitor.py --udp 127.0.0.1:15600 param set ATT_KP 50.0
   ```
   Verify that the Link returns an error indicating the value was rejected as out of range, and observe that the OLED display retains its previous valid value.
3. **Verify persistence across restart**: Set `ATT_KP` to `3.5` in SITL with `--param-file /tmp/test-params.bin`. Stop the process, verify the file size, and restart SITL with the same `--param-file`. Query `param get ATT_KP` and verify `3.5` is returned and shown on the OLED display at boot. Finally, restore the default `1.0`.

## Read further

- [ADR-0002: Bus backbone and pluggable Links](../../../docs/adr/0002-bus-backbone-and-pluggable-links.md)
- [ADR-0004: FreeRTOS usage](../../../docs/adr/0004-freertos.md)
- [ADR-0006: Schema and wire encoding](../../../docs/adr/0006-schema-and-wire-encoding.md)
- [ADR-0008: Bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0009: Layered HAL](../../../docs/adr/0009-layered-hal.md)
- [ADR-0012: Parameter system](../../../docs/adr/0012-parameter-system.md)
- [ADR-0018: Test layers and SITL](../../../docs/adr/0018-test-layers-and-sitl.md)
- [ADR-0020: Internal Flash layout and write policy](../../../docs/adr/0020-internal-flash-layout-and-write-policy.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [C50C V1.1 board definition (`board.toml`)](../../../boards/c50c-v1_1/board.toml)
- [C50C V1.1 board guide](../../../docs/boards/c50c-v1_1.md#serial-ports)
- [Link monitor guide (`link-monitor.md`)](../../../docs/guide/link-monitor.md)
- [Building guide](../../../docs/guide/building.md)
- [Flashing guide](../../../docs/guide/flashing.md)
- [Examples guide](../../../docs/guide/examples.md)
- [Parameters header (`parameters.hpp`)](../../../core/include/vk/core/parameters.hpp)
- [Parameter storage header (`parameter_storage.hpp`)](../../../core/include/vk/core/parameter_storage.hpp)
- [Link header (`link.hpp`)](../../../modules/link/include/vk/link/link.hpp)
- [Topic map header (`topic_map.hpp`)](../../../modules/link/include/vk/link/topic_map.hpp)
- [Parameter status message (`parameter_status.msg`)](../../../schema/msg/parameter_status.msg)
- [Parameter update message (`parameter_update.msg`)](../../../schema/msg/parameter_update.msg)
- [Link status message (`link_status.msg`)](../../../schema/msg/link_status.msg)
