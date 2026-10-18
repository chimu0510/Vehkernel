# Spec 0003: first Link to the development host

**Status**: done. Issue: [#23](https://github.com/chimu0510/Vehkernel/issues/23); tickets #24 to #29. The serial Link was confirmed on the C50C over USB in [#28](https://github.com/chimu0510/Vehkernel/issues/28).

This file and the issue carry the same spec; update both when the spec changes.

## Problem Statement

The second milestone left Vehkernel publishing IMU samples at 200 Hz on the C50C V1.1 and in SITL (#14). The only way to see that data is to halt the CPU over SWD and decode RAM by hand, and SITL can only print a report when it exits. Nothing can talk to the firmware while it runs. Every later milestone (parameters, modes and arming, actuators, the Bridge, logging) needs a Link: tuning needs live data, the Bridge is a Link's peer, and ADR-0018 already expects bring-up results to travel over one. ADR-0002 and ADR-0006 define the custom protocol and its encoding, but none of it exists.

## Solution

Deliver the third milestone: **the first Link, from the firmware to the development host**. Afterwards:

- The schema generator also emits field-by-field `encode()` / `decode()` for the custom protocol's packed little-endian encoding, in C++ for the MCU and in Python for host tools, both from the same schema.
- A Link (ADR-0002) runs on the firmware: a topic map choosing which topics leave and at what rate, the custom protocol (COBS + CRC16 framing, sequence numbers, version negotiation, schema hash comparison at the handshake, time synchronization), and a transport.
- On the C50C V1.1 the transport is SERIAL1 (USART3 to the Type-C CH9102F) with DMA. In SITL the transport is UDP by default; a pty transport is available as an option.
- A Link monitor on the development host connects over a serial port or UDP, performs the handshake, reports schema mismatches, and shows every received topic live in the terminal with its rate and latest values.
- CI runs SITL with its UDP Link and checks, from the Link monitor, that the handshake succeeds and `imu_sample` arrives at the configured rate.

## User Stories

1. As a firmware developer, I want the schema generator to emit `encode()` and `decode()` for every message, so that no protocol code is written by hand (ADR-0006).
2. As a firmware developer, I want the encoding to be packed little-endian, fields in schema order, with no padding, so that the wire format never depends on the compiler or platform (ADR-0006).
3. As a firmware developer, I want structs never to be `memcpy`'d onto the wire, so that internal structs stay free to change layout (ADR-0006).
4. As a tool author, I want Python decoders generated from the same schema, so that host tools never drift from the firmware.
5. As a maintainer, I want a cross-language test where C++ encodes and Python decodes the same messages, so that both generated sides agree byte for byte.
6. As a firmware developer, I want each frame protected by COBS framing and a CRC16, so that a receiver resynchronizes after any corruption (ADR-0002).
7. As a firmware developer, I want each frame to carry a sequence number, so that the receiver can count lost frames.
8. As a Link user, I want both sides to agree on the highest protocol version they share, so that the protocol can evolve without breaking older peers (ADR-0002).
9. As a Link user, I want the handshake to compare every topic's schema hash and name the topics that differ, so that a firmware and a tool built from different schemas never misread data (ADR-0006).
10. As a Link user, I want time synchronization between the development host and the MCU, so that tools can convert MCU timestamps to host time (ADR-0002).
11. As a firmware developer, I want a topic map that says which topics leave through a Link and at what rate, so that bandwidth is spent on what matters (ADR-0002).
12. As a firmware developer, I want a topic map entry to be able to send every sample of a queued topic, so that `imu_sample` arrives at its full 200 Hz.
13. As a firmware developer, I want a Link to drop and count messages when its transport is full, so that a slow host never blocks a work queue.
14. As a firmware developer, I want each Link to publish its statistics (frames and bytes sent and received, drops, CRC errors, lost frames) as a topic, so that Link health is observable on the bus and over the Link itself.
15. As a firmware developer, I want the Link to run as a work item, so that it follows the execution model of ADR-0008 and is timed by the timing monitor.
16. As a driver author, I want a UART contract in the interface layer, so that a Link transport works on any MCU's UART (ADR-0009).
17. As a C50C user, I want the Link on SERIAL1, the Type-C port, so that one USB cable gives live data.
18. As a C50C user, I want UART transmit and receive to use DMA, so that the Link costs little CPU at high rates.
19. As a firmware developer, I want the Link's baud rate and port to be compile-time defaults until the parameter system exists, so that this milestone does not depend on parameters.
20. As a SITL user, I want SITL's Link on UDP by default, so that tools, the Bridge and CI connect without device files or permissions.
21. As a SITL user, I want to choose the UDP address and port on the command line, so that several SITL instances can run at once.
22. As a SITL user, I want an optional pty transport, so that tools can be tested against a real serial API (termios, baud rate, framing errors) without hardware.
23. As a Link user, I want a Link monitor on the development host that connects over a serial port or UDP, so that I can watch live data from the board or from SITL.
24. As a Link user, I want the monitor to show each topic's rate, lost frames and latest values, refreshing in the terminal, so that I see the firmware's state at a glance.
25. As a Link user, I want the monitor to print schema mismatches and protocol errors clearly and exit non-zero on a failed handshake, so that it can be used in scripts.
26. As a Link user, I want the monitor to show the Link statistics and the work item timing statistics, so that both ends' health is visible.
27. As a maintainer, I want unit tests for COBS, CRC16, framing, the handshake and time synchronization at their public interfaces, so that protocol behaviour is locked.
28. As a maintainer, I want the protocol tested against corrupted, truncated and concatenated frames, so that resynchronization is proven.
29. As a maintainer, I want the Link tested over a fake transport on the development host, so that topic map rates, drops and statistics are covered without SITL.
30. As a maintainer, I want CI to run SITL with its UDP Link and check through the Link monitor that `imu_sample` arrives at 200 Hz, so that the whole Link path is covered on every push (L3).
31. As a maintainer, I want schema generator golden tests to cover the new C++ and Python outputs, so that generated code changes are reviewed.
32. As a maintainer, I want every new option in Kconfig with correct dependencies, for example the serial Link depending on a board port with the `LINK` function, or SITL (ADR-0017).
33. As a maintainer, I want Flash, RAM and CCM budgets to keep holding with the Link linked in, so that growth stays visible.
34. As a C50C user, I want a documented hardware check that runs the Link monitor over USB for a minute with no CRC errors or lost frames, so that the serial Link is confirmed on the board.
35. As a new contributor, I want the README to explain how to connect the Link monitor to the board and to SITL, so that I can see live data in minutes.

## Implementation Decisions

- **Encoding (schema generator)**: the generator emits, per message, `encode()` into a byte span and `decode()` from one, field by field in schema order, packed little-endian with no padding, plus the encoded size as a constant. It also emits a Python module per schema with the same encoders, decoders and schema hashes for host tools. Topics get a stable numeric topic ID, derived from the sorted schema, which the handshake maps to names and hashes.
- **Frame format**: every frame is COBS-encoded and terminated by a zero byte. Before COBS encoding a frame holds: protocol version, frame type (topic data or a control message), sequence number, and for topic data the topic ID, instance and encoded message; a CRC16 (CRC-16/CCITT-FALSE) over all of it comes last. Maximum frame size is fixed at build time; larger messages are rejected by the generator.
- **Control messages**: `hello` (the protocol versions a side supports), `hello_reply` (the chosen version and every topic's ID, name and schema hash), `time_sync_request` and `time_sync_reply` (host send time, MCU receive and send times in MCU microseconds). The host starts the handshake; the MCU sends topic data only after a successful handshake and stops when the peer is silent past a timeout.
- **Protocol module (module layer)**: COBS, CRC16, framing, sequence numbers, the handshake state machine and time sync replies, independent of the transport and of FreeRTOS, so it is unit-tested on the development host.
- **Topic map**: a compile-time table per Link until the parameter system exists: topic, instance and either a maximum rate or "every sample" for queued topics. The first map sends `imu_sample` at every sample, `work_item_status` and `link_status` at 1 Hz.
- **Link work item**: one work item on `wq:io` per Link. It subscribes to the topics in its map, encodes due messages into the transport, drains received bytes and runs the protocol. When the transport cannot take a frame it drops it and counts the drop; it never blocks.
- **Link statistics**: a `link_status` topic (frames and bytes sent and received, drops, CRC errors, lost frames from sequence gaps, handshake state), one instance per Link.
- **Interface layer**: a UART contract (configure baud rate; non-blocking write that accepts what fits; non-blocking read of what has arrived), checked with C++20 concepts and bound like the I2C contract, so a transport is not a template.
- **Platform layer, STM32F4**: USART with DMA for transmit and receive, using the DMA streams the board generator solved, a receive ring buffer and a transmit buffer in normal SRAM (not CCM, ADR-0008). Pins come from the board's port definition (SERIAL1 on the C50C). Default 921600 baud, 8N1, no flow control.
- **Platform layer, SITL**: a UDP transport (bind address and port from the command line, default `127.0.0.1:15600`; replies go to the address the last `hello` came from) and an optional pty transport (`--link pty` creates a pseudo-terminal and prints its path). UDP is the default in SITL and CI; pty is for testing tools against a real serial API.
- **Link monitor (development host)**: a Python tool under `tools/link/` using the generated Python decoders. It connects to a serial port or UDP address, performs the handshake and time sync, reports mismatching schema hashes by topic name, and refreshes a terminal table of topics (rate, lost frames, latest values), Link statistics and work item timing. A `--check` mode runs for a fixed time, prints a parseable summary and exits non-zero when the handshake fails or an expected topic rate is not met.
- **Kconfig**: `LINK` (the custom protocol and Link work item), `LINK_SERIAL` (depends on a board serial port with the `LINK` function), `LINK_UDP` and `LINK_PTY` (SITL only).
- **Entry points**: the firmware starts one Link on SERIAL1; SITL starts one Link on the chosen transport.

## Testing Decisions

- **Good tests check external behaviour only**: generator outputs, the bytes on a fake transport, the Link monitor's output and exit code, and SITL's report. Internal state such as the handshake state variable is not asserted directly.
- **Seam A: schema generator command line.** Golden tests for the C++ and Python encoder outputs, an encoded-size check per type, and a failing input for a message larger than the maximum frame. Generated C++ is compiled by the `test` preset.
- **Seam B: host unit tests (L1, GoogleTest and pytest).** COBS and CRC16 against known vectors; framing with corrupted, truncated, concatenated and oversized input; the handshake and time sync as message exchanges; the Link over a fake transport (topic map rates, every-sample queued topics, drops when full, statistics). A cross-language test encodes messages in C++ and decodes them with the generated Python module.
- **Seam C: SITL over UDP (L3).** CI starts SITL with its UDP Link and runs the Link monitor in `--check` mode against it: the handshake must succeed with no schema mismatch, `imu_sample` must arrive at 200 Hz within 5 %, and no frames may be lost on the loopback.
- **Seam D: hardware (L4, manual).** The Link monitor runs over USB against the C50C for 60 s: `imu_sample` at 200 Hz, no CRC errors, no lost frames. Results are recorded on the ticket.
- **Prior art**: the schema generator's golden tests, the bus and work queue unit tests, `test/sitl/check_report.py`, and the bring-up runner's pytest with a fake OpenOCD.

## Out of Scope

- The attitude estimator and every other estimator (next milestone).
- Topics sent from the development host to the MCU, setpoints, setpoint arbitration, modes and arming. This milestone carries only control messages inbound.
- The parameter system and parameter transfer over the Link; Link settings are compile-time defaults.
- XRCE-DDS, MAVLink and any protocol other than the custom one.
- The Bridge and ROS2.
- Logging and ULog streaming.
- CAN and USB device transports; a Link on SERIAL2 or SERIAL3.
- Moving bring-up results onto the Link, and the test mode of ADR-0018.
- libFuzzer targets for the decoders: the devcontainer has no clang compiler yet; framing robustness is covered by unit tests with corrupted input for now.

## Further Notes

- **UDP by default, pty optional in SITL**: UDP needs no device files or permissions, leaves nothing behind when a process dies, and is what the Bridge and Nav2 in separate processes or containers (L5) will use. A pty keeps a path to test tools against a real serial API (termios, baud rate changes, framing errors) without hardware. ADR-0018 records this.
- **921600 baud**: `imu_sample` at 200 Hz is about 10 KB/s with framing, about 11 % of the link; the CH9102F supports it. The rate becomes a parameter once the parameter system exists.
- **Time sync** is request and response only in this milestone; the host computes offset and drift. The MCU does not adjust its clock.
- The topic IDs are local to one build; peers always learn them from the handshake, never assume them.
