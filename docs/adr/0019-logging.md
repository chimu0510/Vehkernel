# ULog logging with replaceable sinks; backpressure handled in the logger

Logs use the ULog format because existing tools open it directly and its packed little-endian encoding matches our schema. Sinks are replaceable through the `LogSink` interface and several can run at once; when there is too much data the logger degrades on its own, and modules producing data never know the logger exists.

**Format: ULog**

- No tools to write: pyulog, PlotJuggler and Lichtblick all open `.ulg` files.
- Fits existing decisions: the first field of every message is `timestamp` (ADR-0006, ADR-0008), and the custom protocol also uses packed little-endian (ADR-0006), so logging and the custom protocol share one encoding with no re-encoding. The generator derives the ULog format definitions from the schema.
- Message types used: `P` (parameters), `I` (board, board revision, firmware version, schema hash), `L` (text messages, also forwarded to ROS2 `/rosout`), `O` (dropouts) and `S` (sync).
- Logged topics must be fixed length.
- The Bridge records the clock offset between the MCU and the companion computer, so ULog can be aligned in Lichtblick with MCAP recorded by rosbag2.

**Sinks**

Every sink implements the `LogSink` virtual interface, and several can be enabled at once:

- SD card: FatFs, one `.ulg` file per session.
- SPI NOR Flash: block-wise append, no file system.
- Streaming to the companion computer, which the Bridge saves.
- RAM ring buffer: written to the event log sector of internal Flash, in batches, only when an event occurs (ADR-0020).
- Serial logger.

The board TOML declares storage devices, which become `BOARD_HAS_*` Kconfig symbols, so Kconfig decides which sinks are compiled in. The bitmask parameter `LOG_BACKEND` decides which are enabled, with defaults from the board definition. The C50C has no SD card or SPI Flash, so it defaults to streaming and uses internal Flash only for the event log (ADR-0020); this is merely its board default.

**What to log**

- The bitmask parameter `LOG_PROFILE` combines these profiles (like PX4's `SDLOG_PROFILE`): default, estimator replay, system identification, high-rate control and debug.
- Each profile is defined in TOML: which topics to log, at what rate.
- `LOG_MODE` decides when to log: always, while armed, or on events.
- The generator estimates each profile's bandwidth, and the firmware checks it again at boot against the actual baud rate.

**High-rate data and backpressure**

- Topics that must be logged at full rate use `@queue` or batch messages; otherwise the generator reports an error.
- Batch messages follow PX4's `SensorGyroFifo`: `int16[32]` plus `samples`, `dt` and `scale`, published when N samples have accumulated or T milliseconds have passed, whichever comes first. The control chain still subscribes to the per-sample topic, declared with `@queue` (ADR-0015).
- Backpressure is handled entirely in the logger, and producing modules never know the logger exists:
  - The logger detects missed samples from subscription generation counters and writes `O` messages.
  - Before writing to the ring buffer it reserves space with a non-blocking `try_reserve`; on failure it skips the sample and counts the drop.
  - Under heavy load it degrades by profile priority.

## Considered Options

- **Producers calling `logger_has_headroom()`**: rejected. High-priority drivers would depend on the logger, breaking bus decoupling and putting the check on the `wq:ctrl` hot path.
- **Betaflight's compressed format, a custom format, or MCAP generated on the MCU**: rejected. The first two need dedicated tools; MCAP's indexes and compressed chunks are too complex for the MCU, and rosbag2 on the companion computer already produces it.
