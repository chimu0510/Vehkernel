# Spec 0011: logging over the Link and estimator replay (ULog, log profiles and the L2 replay layer)

**Status**: in progress. Issue: [#137](https://github.com/chimu0510/Vehkernel/issues/137); tickets #138 to #143, with the hardware check in #143.

Until this spec is done, this file and its issue carry the same spec; update both in the same edit.

## Problem Statement

The firmware estimates attitude with a Mahony filter and dead-reckons local odometry (specs 0004, 0009), and the next estimation work is an EKF (ADR-0014). There is no way to record what the estimators saw and did, and so no way to develop or judge a new estimator on real data:

- The Link monitor shows topics live, but keeps nothing. Its topic map sends the wheel data and the estimates at 50 Hz, a quarter of the rate the estimators run at, so even a capture of the stream could not reproduce them.
- An estimator can be checked only in SITL, against simulated sensors, or on the board, by eye. The heading and square checks of spec 0010 were deferred to the EKF for this reason.
- ADR-0019 decides the log format (ULog), the sinks and the log profiles, and ADR-0018 defines L2, the replay layer. Neither exists. The C50C has no SD card or SPI Flash, so its board default is streaming to the host (ADR-0019).

## Solution

Deliver the eleventh milestone: **logging over a Link to a ULog file on the host, and replay of the estimators on recorded logs**. Afterwards:

- A host asks the MCU over a Link, while disarmed, to start logging with a log profile. The MCU adds that profile's topics to the Link's stream at the profile's rates, every sample where the profile says so, and tells the host what it misses. It then restarts the estimators, so that the log holds their start.
- The host writes a standard `.ulg` file: the formats from the schema, the board and firmware information, the parameters at the start, every logged message with its MCU timestamp, and the dropouts. pyulog, PlotJuggler and Lichtblick open it.
- One log profile exists: estimator replay. Its inputs (`imu_sample`, `body_velocity_measurement`) go at every sample, and its outputs (`vehicle_attitude`, `vehicle_local_odometry`) at 50 Hz. On the C50C it fits SERIAL1 at 921600 baud alongside the normal topic map, at about 45 % of the port.
- A replay program runs the firmware's own estimator work items on the development host, fed from a log in recorded order with the logged parameters, and writes what they publish to a new log.
- Replaying an estimator on a SITL log reproduces the logged estimate exactly, and on a board log within a tight tolerance. A changed or new estimator is then judged by its error against SITL's ground truth or a reference log.
- CI records SITL drives, replays them and checks both. One short log recorded on the C50C is kept in the repository, so replay also runs on real sensor noise.

## User Stories

1. As an estimation developer, I want to record a drive on the board to a file on the host, so that I can study what the estimators saw and did.
2. As an estimation developer, I want the inputs of the estimators recorded at every sample, so that a replay sees exactly what the board saw.
3. As an estimation developer, I want to rerun the firmware's estimators on a recorded log on my development host, faster than real time, so that I can change an estimator and see the effect without driving again.
4. As an estimation developer, I want the replay of an unchanged estimator to match the recorded estimate, exactly for SITL and within a tight tolerance for the board, so that I can trust a difference a change makes is the change's, not the replay's.
5. As an estimation developer, I want a new estimator, such as an EKF, judged by its error against the truth on the same logs as the old one, so that the comparison is fair (ADR-0014).
6. As a developer, I want logs that standard tools open (pyulog, PlotJuggler, Lichtblick), so that I need no tool of our own to plot them (ADR-0019).
7. As a developer, I want every dropout marked in the log, on the MCU or on the transport, so that a gap in the data is never mistaken for the vehicle standing still.
8. As a developer, I want the log to carry the board, the firmware version and git hash, the schema hash and the parameters, so that a log says what produced it.
9. As a developer, I want logging refused when a profile does not fit the port, so that a slow port never silently thins a log.
10. As a firmware developer, I want the topics a profile logs and their rates declared in one TOML file, and the formats generated from the schema, so that a new topic or profile needs no hand-written encoding (ADR-0006, ADR-0019).
11. As a firmware developer, I want CI to record SITL drives and replay them, so that a change that breaks the estimators or their replay fails in CI (ADR-0018 L2).
12. As a C50C user, I want a documented hardware check that records a drive on the board and replays it, so that logging is confirmed on the real port and sensors (ADR-0018 L4).

## Implementation Decisions

- **Where the log is put together** (ADR-0019, decided with the maintainer): the MCU chooses what is logged and at what rate, and the host writes the file. The topics travel as the Link's topic data, in the schema's packed little-endian encoding, which ULog's data messages share, so nothing is encoded twice. The MCU has no `LogSink` and no ring buffer in this milestone; those come with the first sink on the MCU itself (SD card, SPI Flash, the event log). ADR-0019 is edited to say that the streaming sink is the Link's own stream, started by the host.
- **Log profiles** (ADR-0019): one TOML file, `schema/log_profiles.toml`, lists each profile's topics and their rates; a rate is a frequency, `every_sample`, which needs a queued topic (`@queue`, ADR-0008), or `as_published`. An entry may carry a Kconfig condition (`kconfig = "PLATFORM_SITL"`), so that SITL-only topics are left out of the board's table. The generator checks each topic against the schema and emits the profile tables for the firmware and the host, and each profile's bandwidth on the wire. The one profile now is `estimator_replay`:
  - every sample: `imu_sample` (instance 0), `body_velocity_measurement` (instance 0);
  - 50 Hz: `vehicle_attitude`, `vehicle_local_odometry`, `actuator_outputs`;
  - as published: `vehicle_status` (5 Hz), `work_item_status` and `link_status` (1 Hz), and `parameter_update` (on each change);
  - SITL adds `vehicle_attitude_groundtruth` and `vehicle_local_odometry_groundtruth` at 50 Hz.

  About 33 KB/s with the Link's framing, about 45 % of SERIAL1 at 921600 baud.
- **`body_velocity_measurement` queued**: logging every sample of it needs a queue, so the schema gives it `@queue 4`, as `imu_sample` has. Its subscribers drain it each run as the IMU's do.
- **Starting and stopping** (protocol version 5, ADR-0002): four control messages, `log_start`, `log_info`, `log_stop` and `log_dropout`.
  - `log_start` carries the profile bitmask; the MCU answers with `log_info`: the board name (`board::name`) and ID, the firmware version and git hash from the app descriptor (ADR-0023), and the profile's estimated bandwidth. The schema hash of every logged topic is already known to the host from the handshake.
  - The MCU refuses `log_start` with a reason when the vehicle is armed, when the profile is unknown, when its bandwidth together with the part of the Link's own topic map that is never scaled (its 1 Hz status topics) exceeds 80 % of the port (ADR-0002), or when another Link is logging. The rest of the topic map is scaled down to the room the profile leaves, and its streams of the profile's topics are held back while logging, so nothing is sent twice. `log_info` carries the result, `OK` or the refusal.
  - `log_stop` ends it, and so does the end of the Link session.
  - While logging, the profile's streams are never rate-scaled; the topic map's own streams still are, after the profile's are counted.
- **Restarting the estimators** (decided with the maintainer): a log over the Link starts long after boot, and replay cannot rebuild an estimator's state from what it publishes (`DeadReckoning` keeps north and east as doubles and publishes floats). So a log holds the estimators' start.
  - Once the profile's streams have started, the Link sends a new request on `vehicle_command`, `RESTART_ESTIMATORS`. The commander accepts it only while disarmed and publishes `estimator_restart`.
  - The attitude estimator returns to aligning, and needs the vehicle still for 1 s, as at boot. Local odometry clears its position, and its origin becomes where the vehicle stands when the attitude estimator has aligned again.
  - Nothing else changes: arming behaves as it does in the first second after boot.
  - The Logging guide tells the operator to leave the vehicle still for a second after starting a log.
- **Dropouts** (ADR-0019): a logged stream remembers its subscription's generation. A gap (publications overwritten before the Link sent them, or frames the transport could not take) becomes a `log_dropout` control message with the topic, the instance and the number of messages missed. The host also turns gaps in the frames' sequence numbers into dropouts. Each becomes a ULog `O` message.
- **Host recorder** (`tools/link/`): `link_monitor.py record <file.ulg> --profile estimator_replay [--duration s]`, and the library behind it. It writes:
  - the header and `I` messages from `log_info` (board, board ID, firmware version, git hash, schema hash);
  - `F` and `A` messages generated from the schema;
  - the parameters read over the Link before logging starts, as `P` messages, and again as `P` messages at the time of change for each parameter that changes while logging. `parameter_update` is logged and names the parameter by its hash, and the recorder reads the new value over the Link;
  - every logged message as a `D` message with its own MCU timestamp, in arrival order;
  - `O` for dropouts, and `S` sync messages.

  The writer and a small reader live in `tools/log/vkulog/`, so the replay tools need no third-party parser. pyulog is pinned as a test dependency (ADR-0016) only, to check that standard tools read our files.
- **Replay program** (`vehkernel-replay`, a host executable in the `test` preset, ADR-0018 L2): it builds the firmware's estimator work items (`AttitudeEstimator`, `LocalOdometry`) unchanged, on a `WorkQueue` with a replay clock.
  - Before the first input, it applies the log's parameters; a parameter that changed while logging is applied, and `parameter_update` published, at its logged time.
  - It then publishes each logged input in timestamp order, sets the clock to its timestamp and runs the queue.
  - The order between topics follows the order the work items run in on `wq:ctrl`: the IMU driver, and so the attitude estimator, before the vehicle model's measurement and local odometry. The implementer confirms this order from the firmware, and SITL's exactness checks it.
  - What the estimators publish goes to stdout.
  - `tools/log/replay.py <in.ulg> <out.ulg>` feeds the program from a log and writes its output as a log with the same information and parameters.
  - Replay runs as fast as the host allows.
- **Pass rule** (decided with the maintainer):
  - **Faithful first.** For every logged `vehicle_attitude` and `vehicle_local_odometry` sample, the replayed one with the same `timestamp_sample` must match it. For a SITL log the match is exact, field for field: SITL and the replay program are built by the same host toolchain.
  - For a board log the match is within a tolerance. The board's float maths differs from the host's: GCC fuses multiply-adds on the Cortex-M4 FPU, and newlib's libm is not glibc's. The tolerance is measured in the hardware check and written into `replay_board_log`; about 1e-5 relative on the attitude and under a millimetre on the position is expected.
  - Either way, a mismatch means the replay is not faithful (order, parameters, or a hidden input), and fails.
  - **Then metrics.** An estimator changed since the log was recorded, or a new one, is judged by its error against the log's ground truth (SITL) or against a reference log: attitude error in degrees, and position and heading error after the drive.
  - `tools/log/compare.py` reports both.
- **SITL logs in CI** (L2): `sitl_replay` records SITL's ground vehicle over its UDP Link with `estimator_replay`, driving a square with turns (as `check_remote_control.py` does), then replays the log. The test passes when:
  - the replay is exact, from the estimators' restart to the end of the log;
  - the replayed odometry is within the bounds `check_odometry.py` already uses against the ground truth;
  - the log has no dropouts.
- **A board log in the repository** (decided with the maintainer): one log recorded on the C50C in the hardware check, under 1 MB, kept in `test/replay/data/` and listed in `REUSE.toml`. `replay_board_log` replays it and checks the match within the tolerance measured in the hardware check. When the estimators change on purpose, the reference is re-recorded, or the test is switched to metrics against the previous estimate, and the commit says which.
- **Documentation**: a Logging guide, `docs/guide/logging.md` (recording, profiles, dropouts, opening a log, replay and comparison). ADR-0019 and ADR-0018 are edited for what is built, and `GLOSSARY.md` gains Replay.

## Testing Decisions

- **Seam A: the Link (L1, L3).**
  - Unit tests for `log_start` and its refusals (armed, unknown profile, over budget, already logging on another Link) and for `log_info`'s contents.
  - Unit tests for `RESTART_ESTIMATORS`: refused while armed; accepted, it returns the attitude estimator to aligning and clears local odometry.
  - Unit tests for the profile's streams at their rates, unscaled, and for a dropout reported when publications are overwritten before sending.
  - The Link's fuzz targets cover the new control messages.
- **Seam B: the recorder and the ULog files (L1).**
  - pytest round trips through `vkulog`.
  - pyulog reads a recorded file, with its formats, parameters, information and dropouts.
  - Sequence gaps become `O` messages.
- **Seam C: the generator (L0, L1).** A profile naming an unknown topic, `every_sample` on an unqueued topic, or a malformed rate fails the build; the bandwidth estimate matches a hand count for `estimator_replay`.
- **Seam D: replay (L2).**
  - `sitl_replay` and `replay_board_log` as above.
  - A unit test feeds the replay program a hand-made sequence and checks the input order between topics.
  - A unit test changes a parameter partway through a hand-made log and checks that the replay applies it at that time.
- **Seam E: hardware (L4, manual).** On the C50C V1.1, with the vehicle of #126 and the pad:
  - Record `estimator_replay` over SERIAL1 while driving on the ground for about a minute, with a straight run and turns. The check passes with no dropouts, no work item overruns and the Link at its expected rate.
  - The log opens in PlotJuggler or pyulog.
  - Its replay matches within a tolerance, which this check measures and records. The largest difference seen, with a margin, becomes the tolerance of `replay_board_log`.
  - A short cut of it, from the estimators' restart, is committed as the board log.

  Results are recorded on the ticket.
- **Prior art**: `check_remote_control.py` and `check_odometry.py` for driving SITL and judging the odometry; the attitude estimator's unit tests for running a work item on a fake clock; PX4's `replay` module and `ekf2` replay; pyulog's ULog writer and reader.

## Out of Scope

- MCU sinks: SD card, SPI NOR Flash, the RAM ring buffer and the event log in internal Flash (ADR-0019, ADR-0020). The event log gets its own spec once there are events worth keeping.
- `LOG_PROFILE`, `LOG_MODE` and `LOG_BACKEND` as parameters: they choose what an MCU sink records, and only a host starts logging now.
- Profiles other than `estimator_replay`: default, system identification, high-rate control and debug come when first needed.
- Batch messages (ADR-0019): `imu_sample` at 200 Hz fits without them.
- Text log messages (ULog `L`) and forwarding them to `/rosout`.
- The Bridge's recording and aligning ULog with rosbag2 (ADR-0019, ADR-0003).
- The EKF itself, and the magnetometer (ADR-0014): the next spec, built on this one.
- Replay of anything other than the estimators, such as the control chain.

## Further Notes

- The ULog format is documented by PX4 ("ULog File Format"). Vehkernel writes version 1, without the optional appended-data section.
- The deferred heading and square checks of spec 0010 become possible with this spec's board log and the EKF that follows.
