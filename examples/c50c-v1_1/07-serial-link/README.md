# 08-serial-link: Custom protocol Link over serial ports SERIAL1 and SERIAL2 carrying IMU topics

## What you learn

- How the Vehkernel custom binary protocol (ADR-0002, ADR-0006) streams topics between the flight controller MCU and a development host or companion computer over high-speed serial ports (`SERIAL1`, `SERIAL2`) or UDP in SITL simulation.
- How wire frames are framed using Consistent Overhead Byte Stuffing (COBS) with delimiter `0x00` and protected by CRC16-CCITT error detection ([`modules/link/include/vk/link/frame.hpp`](../../../modules/link/include/vk/link/frame.hpp)).
- How the connection handshake establishes communication: the host starts with a `Hello` frame offering supported protocol versions; the MCU accepts the highest matching version, replies with `HelloReply` (carrying the parameter count and parameter set hash), and paces `TopicInfo` frames describing each topic's ID, instances, and schema hash ([`modules/link/include/vk/link/session.hpp`](../../../modules/link/include/vk/link/session.hpp), ADR-0002).
- How bidirectional time synchronization (`TimeSyncRequest` / `TimeSyncReply`) measures round-trip time and estimates clock offset between host and MCU without altering MCU monotonic time.
- How topic maps (`TopicMap`, [`modules/link/include/vk/link/topic_map.hpp`](../../../modules/link/include/vk/link/topic_map.hpp)) define which topics flow out over a Link: queue-draining topics (`every_sample`, such as `imu_sample` at 200 Hz) and rate-limited periodic topics (`MaxRate`, such as `link_status` at 1.0 Hz).
- How Links run as periodic work items on `wq:io` at 200 Hz (`Link::default_interval_us = 5'000`, ADR-0008) without blocking the work queue, dropping frames into `link_status.drops` when the transport buffer is full.
- How baud rate scaling (`scale_to_baud`, ADR-0002) prevents Link traffic from exceeding port bandwidth: when total stream traffic exceeds 80 % of port capacity (`max_port_load = 0.8F`), streams faster than 1 Hz are scaled down by a common percentage factor (never below 1 Hz). At the default baud rate of 921,600 baud (92,160 bytes/s), 80 % capacity is ~73.7 kB/s, so `imu_sample` (200 Hz, ~10 kB/s) and `link_status` (1 Hz) run unscaled at 100 % rate scale.
- How DMA UARTs on STM32F407 (`vk::platform::DmaUart`) manage transmit and receive without per-byte CPU interrupts, utilizing 4096-byte circular transmit buffers (`serial_transmit_buffer_size = 4096`).
- Monitoring live topics, rates, and Link health on the development host using `link_monitor.py` ([`docs/guide/link-monitor.md`](../../../docs/guide/link-monitor.md)).
- Visualizing Link health (`link_status`) on the local SSD1306 OLED display (`DISPLAY0`) at 4 Hz on `wq:slow` (ADR-0022): connection state (`WAITING`, `CONNECTED`, `SILENT`), frame counters (`TX FRAMES`, `RX FRAMES`), buffer drops, and rate scale.
- What this example leaves out: parameter access and storage (`ParameterAccess`, introduced in `09-parameters`), vehicle control and commander requests (`ControlAccess`, ADR-0005), attitude estimators, and log recording over Link (spec 0011).
- Why returning to firmware via `flash_serial.py update` restarts the board through SERIAL2's hardware download circuit: `update` attempts to request a reboot into the bootloader over the Link. The Link passes reboot requests on to the commander as a `vehicle_command`, but only when its `ControlAccess` is enabled; `08-serial-link` leaves it disabled and runs no commander, so the Link ignores the request and `link_monitor.py` gets no reply, and `flash_serial.py` automatically restarts the board through SERIAL2's hardware download circuit (asserting DTR and RTS to pulse NRST with BOOT0 high). On SERIAL1, the user must power-cycle the board when prompted.

## Before this

- [`01-led`](../01-led/README.md): System clock initialization (`vk::hal::SystemClock::now_us()`), GPIO output, and the build and flash workflow.
- [`02-user-key`](../02-user-key/README.md): Reading user key S3 as a debounced GPIO input line.
- [`03-oled`](../03-oled/README.md): SSD1306 OLED display driver (`DISPLAY0`) on software SPI, text rendering with `TextScreen`, and `Line` buffer formatting without heap allocation (ADR-0007).
- [`04-work-queues`](../04-work-queues/README.md): Rate-tiered FreeRTOS work queues (`wq:ctrl`, `wq:io`, `wq:slow`), periodic scheduling with `schedule_periodic()`, and cooperative scheduling within a queue.
- [`05-bus`](../05-bus/README.md): Publish/subscribe bus backbone, schema-generated topics, `vk::core::Publication<T>`, `vk::core::Subscription<T>`, and event-driven wakeups with `trigger_on()`.
- [`06-imu`](../06-imu/README.md): ICM20948 IMU driver (`IMU0`) publishing `imu_sample` at 200 Hz over software I2C on `wq:ctrl`.

## Hardware

- **Board**: C50C V1.1 robot controller ([`boards/c50c-v1_1/board.toml`](../../../boards/c50c-v1_1/board.toml)).
- **Serial port 1 (`SERIAL1`)**: Type-C port:
  - Connector: USB Type-C.
  - Bridge: WCH CH9102F USB-to-UART bridge.
  - Peripheral: `USART3`, TX `PB10`, RX `PB11` ([`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#serial-ports)).
  - DMA streams: TX `DMA1_Stream4` (channel 7), RX `DMA1_Stream1` (channel 4).
  - Default baud rate: 921,600 baud 8N1 (`vk::board::defaults::SERIAL1_BAUD`).
- **Serial port 2 (`SERIAL2`)**: One-key download port:
  - Connector: USB Type-C with auto-download circuit (DTR/RTS driving NRST and BOOT0).
  - Bridge: WCH CH9102F USB-to-UART bridge.
  - Peripheral: `USART1`, TX `PA9`, RX `PA10` ([`docs/boards/c50c-v1_1.md`](../../../docs/boards/c50c-v1_1.md#serial-ports)).
  - DMA streams: TX `DMA2_Stream7` (channel 4), RX `DMA2_Stream2` (channel 4).
  - Default baud rate: 921,600 baud 8N1 (`vk::board::defaults::SERIAL2_BAUD`).
- **IMU (`IMU0`)**: ICM20948 on software I2C (`PB9` SCL, `PB8` SDA, address `0x68`) publishing `imu_sample` at 200 Hz.
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
tools/flash/flash_serial.py --port /dev/ttyACM0 --build build/c50c-v1_1-examples update --image build/c50c-v1_1-examples/app/examples/example-08-serial-link.apj
```

Global options (`--port`, `--build`) go before the `update` subcommand. `flash_serial.py update` first requests a reboot into the bootloader over the Link. Because this example leaves the Link's `ControlAccess` disabled and runs no commander, the Link ignores the request and no reply comes, and `flash_serial.py` automatically restarts the board through SERIAL2's hardware download circuit (asserting DTR and RTS to pulse NRST with BOOT0 high). On SERIAL1, power-cycle the board while `update` waits (see [C50C V1.1](../../../docs/boards/c50c-v1_1.md#serial-ports)).

### Connecting with the Link monitor on the board

Connect a USB cable to either `SERIAL1` or `SERIAL2`. The USB serial adapter appears on the host as `/dev/ttyACM0` (or `/dev/ttyACM1`). Start `link_monitor.py` specifying the board's serial port and generated schema decoders:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1-examples/generated/schema/python
```

Run an automated 5-second check verifying the handshake and the 200 Hz `imu_sample` rate:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --msg-dir build/c50c-v1_1-examples/generated/schema/python \
    --check --duration 5 --expect-rate imu_sample:0=200
```

### Running in SITL

`08-serial-link` also builds and runs in SITL simulation on the development host with the `sitl-examples` preset:

```bash
cmake --preset sitl-examples
cmake --build --preset sitl-examples
```

Run the example executable directly in one terminal, specifying duration 0 (runs until interrupted) and listening for UDP connections:

```bash
build/sitl-examples/app/examples/example-08-serial-link --duration 0 --udp 127.0.0.1:15600
```

In a second terminal, connect with the Link monitor:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600
```

Or run an automated rate verification check over UDP:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600 --check --duration 5 --expect-rate imu_sample:0=200
```

To run the automated L3 scenario check in SITL:

```bash
ctest --preset sitl-examples -R example_08_serial_link
```

### Returning to the firmware

When finished with this example, return the board to standard firmware:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
tools/flash/flash_serial.py --port /dev/ttyACM0 update
```

As with flashing an example, when its Link reboot request gets no reply, it restarts the board through SERIAL2's download circuit. Parameter sectors (`PARAMS`) remain intact.

## What you see

When connected to `08-serial-link`, `link_monitor.py` completes the handshake, exchanges time synchronization, and displays a live view refreshed twice a second.

With `--check`, `link_monitor.py` prints summary `key=value` lines and confirms the negotiated session:

```text
link handshake=ok version=5 mismatches=0 params=0 param_set_hash=0x0000000000000000 frames=1005 crc_errors=0 malformed=0 lost=0 rtt_us=312 clock_offset_us=-1234
topic name=imu_sample instance=0 messages=1000 rate_hz=200.00
topic name=link_status instance=0 messages=5 rate_hz=1.00
mcu_link instance=0 frames_sent=1005 bytes_sent=48240 frames_received=9 bytes_received=138 drops=0 crc_errors=0 malformed=0 lost=0 handshake_state=connected rate_scale=100
```

- `link handshake=ok`: the host and MCU agreed on protocol version 5. Because this example does not include parameter storage, `params=0` and `param_set_hash=0x0000000000000000`.
- `topic name=imu_sample`: the IMU's high-rate sensor readings arrive at 200 Hz (`rate_hz=200.00`), matching `vk::board::timing::base_rate_hz`.
- `topic name=link_status`: MCU Link statistics published once per second (`rate_hz=1.00`).
- `mcu_link`: reports MCU-side counters: frames and bytes sent/received, zero drops (`drops=0`), zero CRC errors, handshake state `connected`, and `rate_scale=100` (unscaled bandwidth).

### Observing the SSD1306 OLED display

The OLED screen updates at 4 Hz on `wq:slow` using information from the `link_status` topic on the bus:

- Line 0: Header text `VEHKERNEL 08-LINK`.
- Line 1: Blank.
- Line 2: Connection state: `STATE: WAITING` before a host connects, changing to `STATE: CONNECTED` once the handshake completes. If the host stops transmitting, the state transitions to `STATE: SILENT` after the peer timeout.
- Line 3: Total frames transmitted by the MCU: `TX FRAMES: <count>`.
- Line 4: Total frames received by the MCU: `RX FRAMES: <count>`.
- Line 5: Transmit frames dropped due to full buffers: `DROPS: 0`.
- Line 6: Effective topic rate scale: `SCALE: 100 %`.
- Line 7: Blank.

In SITL simulation, standard output prints:
- `link transport=udp address=127.0.0.1:<port>` when the UDP Link starts.
- `state=CONNECTED`, `tx=<count>`, `rx=<count>`, `drops=0`, `scale=100` emitted by the display work item at 4 Hz.
- `display_on=1` emitted when SITL shuts down.

## The code

The complete source code is in `main.cpp`:

- **Dedicated minimal topic map (`ExampleStreams`, `example_topic_map`)**:
  - The example declares its own topic map rather than using the full application map:
    ```cpp
    struct ExampleStreams {
        explicit ExampleStreams(float imu_rate_hz) : imu(vk::link::every_sample, 0, imu_rate_hz) {}

        vk::link::TopicStreamOf<vk::msg::ImuSample> imu;
        std::array<vk::link::TopicStreamOf<vk::msg::LinkStatus>, vk::msg::LinkStatus::instances> links{
            vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 0},
            vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 1},
        };
        std::array<vk::link::TopicStream*, 1 + vk::msg::LinkStatus::instances> map{
            &imu, &links[0], &links[1],
        };
    };
    ```
  - `imu`: configured with `every_sample` to drain all published samples of `imu_sample` instance 0.
  - `links`: two `TopicStreamOf<msg::LinkStatus>` streams at 1.0 Hz (`status_rate`), one per Link instance.
  - Each Link instance (`link = 0` or `link = 1`) gets its own `ExampleStreams` instance so each can maintain independent subscription state and rate scaling.
- **Platform abstraction in support code (`example::start_links`)**:
  - `main.cpp` calls `example::start_links(example_topic_map)`, remaining free of platform-specific code.
  - On the STM32F4 board (`examples/support/stm32f4/support.hpp`), `start_links` initializes DMA UARTs on `SERIAL1` (`USART3`) and `SERIAL2` (`USART1`) at 921,600 baud (`vk::board::defaults::SERIAL1_BAUD`, `vk::board::defaults::SERIAL2_BAUD`), attaches `vk::hal::UartTransport`, and creates `vk::link::Link` instances on `wq:io` (`WorkQueueTier::Io`).
  - In SITL simulation (`examples/support/sitl/support.hpp`), `start_links` opens `vk::platform::UdpTransport` on the address and port specified by `--udp <address>:<port>` (default `127.0.0.1:15600`, or port 0 for an ephemeral port), prints `link transport=udp address=...` to standard output, and starts `host_link` on `wq:io`.
- **Work queue prioritization (ADR-0008)**:
  - The ICM20948 IMU driver runs on `wq:ctrl` (highest rate tier) at 200 Hz.
  - The Links run on `wq:io` (middle rate tier) at 200 Hz (every 5 ms). Draining DMA receive buffers, pacing handshake frames, and transmitting topic packets never delays sensor acquisition on `wq:ctrl`.
  - The OLED display work item runs on `wq:slow` (lowest rate tier) at 4 Hz.
- **Display work item (`DisplayItem`)**:
  - Subclasses `vk::core::WorkItem("display", queue)` and attaches to `wq:slow`.
  - Subscribes to `vk::msg::LinkStatus` instance 0 (`status_sub_{0}`).
  - Reads the latest status without blocking, formats connection state (`WAITING`, `CONNECTED`, `SILENT`), frame counts, drops, and rate scale using stack-allocated `Line` buffers, and updates the OLED display via `display_.update()`.
- **What this example leaves out**:
  - Parameter system: `Link::Config` provides no `ParameterAccess` (`parameters = {}`), so parameter queries return 0 parameters (introduced in `09-parameters`).
  - Commander and vehicle commands: `Link::Config` leaves `ControlAccess` disabled (`control = {}`), so mode changes, arming requests, and software reboot requests are not serviced (ADR-0005).
  - Attitude estimation: does not run Mahony or EKF filters; `imu_sample` carries raw calibrated specific force and angular velocity.
  - Magnetometer: does not stream `magnetometer_sample`.

## Exercises

1. **Test rate scaling at lower baud**: In `examples/support/stm32f4/support.hpp`, change `SERIAL1_BAUD` to `115200`. Calculate the port bandwidth (11,520 bytes/s, 80 % load threshold 9,216 bytes/s). Run `link_monitor.py` and observe `rate_scale` in `mcu_link` scale down below 100 % so traffic fits the port.
2. **Observe peer timeout**: Connect with `link_monitor.py`, verify `STATE: CONNECTED` on the OLED, and then terminate `link_monitor.py` with `Ctrl+C`. Watch the OLED state transition to `STATE: SILENT` after the default 3-second peer timeout (`Session::default_peer_timeout_us = 3'000'000`). Reconnect `link_monitor.py` and verify it transitions back to `STATE: CONNECTED`.
3. **Connect to both serial ports**: On the C50C V1.1 board, plug two USB Type-C cables into `SERIAL1` and `SERIAL2`. Run two terminal sessions with `link_monitor.py --serial /dev/ttyACM0` and `link_monitor.py --serial /dev/ttyACM1`. Observe that both serial Links stream `imu_sample` simultaneously without interference.

## Read further

- [ADR-0002: Bus backbone and pluggable Links](../../../docs/adr/0002-bus-backbone-and-pluggable-links.md)
- [ADR-0006: Schema and wire encoding](../../../docs/adr/0006-schema-and-wire-encoding.md)
- [ADR-0008: Bus and work queues](../../../docs/adr/0008-bus-and-work-queues.md)
- [ADR-0009: Layered HAL](../../../docs/adr/0009-layered-hal.md)
- [ADR-0018: Test layers and SITL](../../../docs/adr/0018-test-layers-and-sitl.md)
- [ADR-0022: Local display and local menu](../../../docs/adr/0022-local-display-and-local-menu.md)
- [ADR-0025: Board examples](../../../docs/adr/0025-board-examples.md)
- [C50C V1.1 board definition (`board.toml`)](../../../boards/c50c-v1_1/board.toml)
- [C50C V1.1 board guide](../../../docs/boards/c50c-v1_1.md#serial-ports)
- [Link monitor guide (`link-monitor.md`)](../../../docs/guide/link-monitor.md)
- [Building guide](../../../docs/guide/building.md)
- [Flashing guide](../../../docs/guide/flashing.md)
- [Examples guide](../../../docs/guide/examples.md)
- [Link header (`link.hpp`)](../../../modules/link/include/vk/link/link.hpp)
- [Topic map header (`topic_map.hpp`)](../../../modules/link/include/vk/link/topic_map.hpp)
- [IMU sample message (`imu_sample.msg`)](../../../schema/msg/imu_sample.msg)
- [Link status message (`link_status.msg`)](../../../schema/msg/link_status.msg)
