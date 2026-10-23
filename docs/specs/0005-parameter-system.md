# Spec 0005: parameter system

**Status**: done. Issue: [#36](https://github.com/chimu0510/Vehkernel/issues/36); tickets #37 to #42. Parameter storage in internal Flash and the five parameters were confirmed on the C50C in [#42](https://github.com/chimu0510/Vehkernel/issues/42).

This spec is done and no longer edited; later decisions live in the ADRs and the code, which win where they differ.

## Problem Statement

The fourth milestone left Vehkernel estimating attitude on the C50C V1.1 and in SITL, visible live in the Link monitor (#30). Every setting the firmware has is still a compile-time constant: the Mahony gains and the accelerometer rejection threshold, the Link's baud rate, and the board defaults the board generator emits as constants "until the parameter system exists". Changing a gain means editing code, rebuilding and reflashing over SWD. A board mounted turned in a vehicle has no correct place to say so: the only rotation is the IMU's mounting rotation in the board definition, which is a fact of the PCB, not of the vehicle it sits in.

ADR-0012 and ADR-0020 decide how parameters are defined, accessed, stored and written to internal Flash, but none of it exists: no parameter definitions, no parameter generator, no Flash contract in the HAL, no storage in sectors S1 and S2, and no way to read or change a parameter over a Link. Every later milestone needs it. The control chain resolves `VEHICLE_TYPE` and `FRAME_CLASS` first (ADR-0013), modes and failsafe actions are configured by parameters (ADR-0005), ports get their use from `SERIALn_FUNCTION` (ADR-0011), the primary estimator is chosen by parameter (ADR-0014), logging is configured by `LOG_*` parameters (ADR-0019), and the Bridge maps MCU parameters to ROS2 parameters (ADR-0003).

## Solution

Deliver the fifth milestone: **parameters defined in TOML, stored in internal Flash, and read and changed over the Link**. Afterwards:

- Each module or driver that has settings declares them in its own parameter definition file. A parameter generator turns the definitions compiled into a build into plain structs that modules read directly, a parameter table the firmware validates against, and a metadata file for host tools (ADR-0012).
- Parameters are int32 or float32. Every change is validated against the type, the range and the safety attributes, applied, published as `parameter_update`, and stored.
- Non-default values are stored entry by entry in two alternating 16 KB sectors of internal Flash, keyed by a hash of the name, and survive power loss at any point of a write. Erasing follows ADR-0020: only while disarmed.
- The custom protocol, at version 2, lists, reads and changes parameters over any Link. The handshake reports a parameter set hash so host tools know whether their metadata matches the firmware.
- The Link monitor lists, reads and sets parameters, showing descriptions, units and enum names from the metadata file.
- Five parameters exist and work: the board orientation `SENS_BOARD_ROT`, the Mahony gains `ATT_KP` and `ATT_KI`, the accelerometer rejection threshold `ATT_ACC_REJ`, and the Link baud rate `SERIAL1_BAUD`, whose default comes from the board definition.
- SITL keeps parameters in a simulated Flash that behaves like the F407's, in memory by default or in a file given on the command line.
- On the C50C V1.1, a parameter set over USB survives a power cycle.

## User Stories

1. As a module author, I want to declare my module's parameters in a TOML file next to the module, so that its parameters move or disappear together with the module (ADR-0012).
2. As a module author, I want to give each parameter a type, a default, a range, a unit, a description and its safety attributes in one place, so that the firmware, the tools and the documentation share one definition.
3. As a module author, I want my module to read its parameters as plain struct members with no lookup, so that reading a parameter on `wq:ctrl` costs nothing.
4. As a module author, I want to be told when one of my parameters changes, through `parameter_update` on the bus, so that I apply a new value without polling.
5. As a module author, I want booleans, enums and bitmasks to be int32 parameters whose value names come from the definition, so that two types cover every setting and match MAVLink.
6. As a module author, I want each parameter definition file to declare its name prefix and the generator to reject a name without it, so that parameter names stay grouped by module.
7. As a module author, I want the build to fail when two parameter definitions anywhere in the repository have the same name hash, compiled in or not, so that a removed parameter's stored value can never land in a new one.
8. As a module author, I want the build to fail when a name is longer than 16 characters, a default is out of range or an enum default is not a value name, so that bad definitions never reach the firmware.
9. As a board author, I want the board definition to give board defaults by parameter name, such as the Link port's baud rate, so that one parameter definition serves boards with different hardware (ADR-0012 default layering).
10. As a firmware developer, I want parameters loaded from Flash before drivers and Links start, so that settings read at boot, such as the baud rate and the board orientation, take effect at the first sample.
11. As a firmware developer, I want every change validated against its type and range and rejected with a reason, so that no tool or Link can put a parameter out of range.
12. As a firmware developer, I want parameters marked `locked_when_armed` to reject changes while armed, so that safety-relevant settings cannot change under a moving vehicle.
13. As a firmware developer, I want the arming state to come from an `arming_state` topic on the bus, treated as disarmed until something publishes it, so that parameter storage and the safety attributes work now and the arming milestone only has to publish the topic.
14. As a firmware developer, I want a change to a `reboot_required` parameter to be stored and reported as pending until reboot, so that I know a restart is needed.
15. As a vehicle user, I want my settings to survive power loss, including loss in the middle of a write, so that the vehicle never boots with half-written settings.
16. As a vehicle user, I want a firmware update that adds, removes or reorders parameters to keep every setting I made, so that updates never reset my configuration (ADR-0012).
17. As a vehicle user, I want stored entries that are unknown, of the wrong type, out of range or corrupt to be skipped and reported, so that a bad entry never stops the vehicle from booting.
18. As a firmware developer, I want Flash erased only while disarmed and writes kept in small batches, so that the control chain is never stalled by Flash (ADR-0020).
19. As a firmware developer, I want changes that cannot be written while armed to stay in RAM, marked unsaved, and be written after disarm, so that no change is lost and no erase happens while armed.
20. As a firmware developer, I want the parameter storage's state (entries used, unsaved changes, skipped entries at boot) published on the bus and sent over the Link, so that I can see it in the Link monitor.
21. As a Link user, I want to list every parameter with its value over the Link, so that I see the whole configuration of a running vehicle.
22. As a Link user, I want to read and set a parameter by name over the Link, so that I tune the vehicle without rebuilding or reflashing.
23. As a Link user, I want the handshake to report a parameter set hash, and the Link monitor to warn when its metadata does not match, so that I know when descriptions or ranges shown may be wrong while names still work.
24. As a Link user, I want the Link monitor to show each parameter's description, unit, range and enum value names, so that I set parameters without reading the source.
25. As a Link user, I want a set that is rejected to print the reason and exit non-zero, so that scripts and tests can rely on it.
26. As a tool author, I want the parameter metadata in a JSON file in the build tree next to the generated schema decoders, so that every host tool reads parameters the same way.
27. As a C50C user, I want to set the board orientation with `SENS_BOARD_ROT` when the board is mounted turned in the vehicle, so that the attitude follows the vehicle without editing the board definition.
28. As a C50C user, I want `SENS_BOARD_ROT` numbered as ArduPilot's and PX4's rotation table, so that the values are familiar and map to MAVLink later.
29. As a firmware developer, I want the IMU driver to compose the mounting rotation and the board orientation once at boot, so that `imu_sample` is always in the body frame and no estimator rotates again (ADR-0014).
30. As an estimator tuner, I want to change `ATT_KP`, `ATT_KI` and `ATT_ACC_REJ` while the estimator runs, so that I see the effect of a gain immediately.
31. As a C50C user, I want the Link baud rate to be `SERIAL1_BAUD`, with its default from the board definition, so that a slower adapter or a long cable works without a rebuild.
32. As a SITL user, I want SITL to start from defaults unless I give it a parameter file, so that every run in CI is reproducible.
33. As a SITL user, I want SITL to keep parameters in a file given with `--param-file`, so that I can test a restart with stored parameters.
34. As a maintainer, I want the SITL Flash to behave like NOR Flash (erased to all ones, programming only clears bits, programming a programmed word is an error), so that storage bugs show up without hardware.
35. As a maintainer, I want parameter storage tested against a fake Flash that loses power at any word of a write, so that recovery is proven for every interruption (ADR-0018).
36. As a maintainer, I want CI to set, read and reject parameters over the Link in SITL, change a gain and see the estimator respond, and restart SITL with a parameter file and find the values kept (L3), so that the whole parameter path is covered on every push.
37. As a C50C user, I want a documented hardware check that sets a parameter over USB, power-cycles the board and reads it back, so that storage in internal Flash is confirmed on the board (ADR-0018, L4 "parameter storage").
38. As a new contributor, I want the README to explain how to list and set parameters from SITL and from the board, so that I can change a setting in minutes.

## Implementation Decisions

- **Parameter definition files**: one TOML file per module or driver, next to it. The file declares its prefix and a list of parameters, each with name, type (`INT32` or `FLOAT`), default, range (minimum and maximum), unit, description, optional enum values (name and number) or bitmask bits, and the flags `reboot_required` and `locked_when_armed`. Booleans are int32 enums with two values.
- **Parameter generator**: a command-line generator following the board and schema generators' conventions, run as a CMake custom command after the Kconfig solve; its output is not committed (ADR-0017). It reads every parameter definition in the repository and checks all of them: prefixes declared once, every name with its file's prefix, names at most 16 characters and upper case, no two names with the same FNV-1a 32-bit hash, defaults in range, enum defaults among the value names. For the definitions compiled into the build, filtered by Kconfig including their enum values, it emits: one plain struct per parameter group holding the values, the parameter table (name, hash, type, default, range, flags, location of the value), the board defaults resolved against the definitions, the metadata file, and the parameter set hash. Diagnostics give file, line and parameter name.
- **Default layering**: parameter definition, then board definition. A board default for a parameter that has a definition must be in range or be an enum value name, else configuration fails. Board defaults without a definition in this milestone (`CTRL_RATE_HZ`, `CTRL_TRIGGER`, `IMU_SAMPLING`, `SERIALn_FUNCTION`) stay compile-time constants, as now. A serial port in the board definition gains an optional baud rate, which becomes the `SERIALn_BAUD` board default the way its function becomes `SERIALn_FUNCTION`. Frame presets come with the vehicle models.
- **Metadata file**: JSON in the build tree next to the generated schema Python module, holding every compiled-in parameter with type, default, range, unit, description, enum value names, bitmask bits and flags. The parameter set hash is the first 64 bits of SHA-256 over a canonical form of this content, sorted by name, like the schema hash (ADR-0006).
- **Parameter runtime (core)**: owns the parameter table and the group structs. Get and set by name or index; set validates type, range and `locked_when_armed` against the latest `arming_state`, writes the struct member, publishes `parameter_update`, and hands the change to storage. A `reboot_required` change is stored and its value written, but it is reported as pending reboot; its consumers read it only at boot. It is protocol independent, so the custom protocol now and MAVLink later use the same service (ADR-0012).
- **Topics (schema)**:
  - `parameter_update`: `timestamp`, the hash of the changed parameter and a change counter. Consumers check `updated()` and re-read their struct.
  - `arming_state`: `timestamp` and `armed`. Nothing publishes it in this milestone; a topic never published reads as disarmed.
  - `parameter_status`: `timestamp`, parameter count, the active sector and its bytes used, unsaved changes, changes pending reboot, and the entries skipped at boot by reason. Sent over the Link at 1 Hz.
- **Storage (core)**: in the board definition's `PARAMS` region, two alternating 16 KB sectors (ADR-0012, ADR-0020).
  - Each sector starts with a header (magic and a generation counter) written last when a sector becomes active, so a sector whose header is incomplete is never used. The active sector is the one with a valid header and the highest generation.
  - After the header, entries of 12 bytes, three words, appended in order: name hash (u32), value (4 bytes), type (u8), a reserved byte, and a CRC-16/CCITT-FALSE over the first 10 bytes. Only non-default values are written; setting a parameter back to its default appends an entry with the default value, which compaction later drops. The last entry for a hash wins. The scan stops at the first entry that is all ones.
  - At boot: entries are filled into the structs by hash. Unknown hashes, a stored type different from the definition, values out of range and entries failing their CRC are skipped and counted by reason in `parameter_status`.
  - At boot, before arming is allowed, the spare sector is erased if it is not blank.
  - Writes run on `wq:slow` with a cap on words per run, so each stall is bounded and visible in the timing monitor.
  - When the active sector is full, the current non-default values are compacted into the erased spare sector in batches, which is allowed while armed, and the spare's header is written last. Erasing the old sector waits until disarm. When both sectors are out of space while armed, changes stay in RAM, counted as unsaved, and are written after disarm.
- **Flash contract (interface layer)**: erase a sector of the parameter region, program one 32-bit word, read. Programming only clears bits; programming a word that is not erased is an error the contract reports. `core/` reaches Flash only through this contract (ADR-0009).
- **Platform layer, STM32F4**: Flash controller unlock, sector erase and 32-bit programming of the `PARAMS` sectors only, with the controller's error flags reported.
- **Platform layer, SITL**: a NOR Flash model of the two parameter sectors: erased to all ones, programming only clears bits, programming a programmed word is an error. In memory and blank at start by default; `--param-file <path>` keeps it in that file across runs. The same model, with power loss injectable after any word, serves the unit tests.
- **Custom protocol, version 2**: the MCU and the host tools move to protocol version 2; no released peer speaks version 1, so the MCU offers only version 2. `hello_reply` gains the parameter set hash and the parameter count. New control frames:
  - `param_list_request`: start index; the MCU replies with `param_value` for every parameter from that index on, paced by the Link work item, which never blocks.
  - `param_get`: a name; the MCU replies with `param_value`.
  - `param_set`: a name, a type and a value; the MCU replies with `param_value` holding the value now in effect, or with `param_reject`.
  - `param_value`: index, count, name, type, value, and flags (pending reboot, unsaved).
  - `param_reject`: name and a reason (unknown name, wrong type, out of range, locked while armed).
  - Names travel as a length and up to 16 characters. The protocol module stays independent of the transport and of FreeRTOS.
- **Rotation**: `hal::Rotation` takes explicit numbers from ArduPilot's and PX4's rotation table (`Roll270` 20, `Pitch90` 24, `Pitch270` 25; the others already match). The board definition keeps rotation names; `SENS_BOARD_ROT` offers exactly the rotations `hal::Rotation` has, by those numbers.
- **The five parameters**:
  - `SENS_BOARD_ROT` (sensors group, prefix `SENS`): int32 enum, default `NONE`, `reboot_required`. The ICM20948 driver composes the board's mounting rotation and the board orientation into one rotation at boot and applies it to the accelerometer and gyroscope.
  - `ATT_KP`, `ATT_KI`, `ATT_ACC_REJ` (attitude estimator, prefix `ATT`): float, defaults 1.0, 0.05 and 0.1 g, the current compile-time values. The estimator applies a change on `parameter_update` between samples. The alignment stillness thresholds stay compile-time constants.
  - `SERIAL1_BAUD` (serial ports group, prefix `SERIAL`): int32 enum of the supported rates (57600 to 921600), `reboot_required`, default 921600 from the C50C board definition. The firmware's Link opens SERIAL1 at this rate.
- **Link monitor**: `param list`, `param get <name>` and `param set <name> <value>` over serial or UDP, reading the metadata file from the build tree or from `--param-meta`. It accepts enum value names as values, shows descriptions, units and ranges, warns when the parameter set hash differs from its metadata, and exits non-zero on a rejected set or a failed handshake. The live view and `--check` show `parameter_status`.
- **Topic map**: the default map adds `parameter_status` at 1 Hz.
- **Kconfig**: `PARAM_STORAGE_FLASH` (exists, depends on `BOARD_HAS_FLASH_PARAMS`) selects Flash storage; SITL provides it with its Flash model. Without it parameters live in RAM and start from defaults at every boot.
- **Entry points**: the firmware and SITL load parameters before starting the IMU driver and the Link, and register the storage work item on `wq:slow`.

## Testing Decisions

- **Good tests check external behaviour only**: what a set returns and what `parameter_update` publishes, what a reboot finds in the structs, what the Link carries and what the Link monitor prints. Sector layout and entry bytes are asserted only through what a later boot reads back, except in the generator's golden tests.
- **Seam A: SITL (L3), the main seam.** A script runs SITL and the Link monitor and checks: `param list` returns all five parameters with their metadata and the parameter set hash matches; `param get` and `param set` round-trip; an out-of-range set and an unknown name are rejected with their reason; changing `ATT_KP` changes how fast the estimate converges after a `tilt` step; a `reboot_required` set is reported pending; after a restart with the same `--param-file`, the set values are back and `parameter_status` shows no skipped entries. CI runs it.
- **Seam B: storage (L1, GoogleTest).** Against the NOR Flash model with power loss injected after every word of every write in a sequence of sets and a compaction, the next boot finds either the old or the new value of each parameter, never anything else; compaction keeps every non-default value; entries with a bad CRC, an unknown hash, a wrong type or an out-of-range value are skipped and counted; with `arming_state` armed, no erase happens, compaction into the spare still works, and changes beyond both sectors stay unsaved until disarm; a parameter set back to its default disappears after compaction.
- **Seam C: parameter runtime and protocol (L1).** Set and get by name and index, range and type rejection, `locked_when_armed` against a published `arming_state`, `parameter_update` published once per change, and the version 2 parameter frames over a fake transport, including the `param_list_request` stream and the parameter set hash in `hello_reply`.
- **Seam D: generator (L0 and L1).** Golden output for valid definitions; one diagnostic case each for a hash collision between a compiled-in and a not compiled-in definition, a duplicated prefix, a missing prefix, a name over 16 characters, an out-of-range default, an unknown enum default and an out-of-range board default; byte-identical output for identical input.
- **Seam E: hardware (L4, manual).** On the C50C over USB: `param set` of `ATT_KP` and `SENS_BOARD_ROT`, a power cycle, `param get` returns the values and the attitude follows the new board orientation; then `SERIAL1_BAUD` set to 460800, a reboot, and the Link monitor connects at the new rate. Results are recorded on the ticket.
- **Prior art**: `test/sitl/check_attitude.py` and `check_link.py` for SITL with the Link monitor library; the Link unit tests over a fake transport; the ICM20948 register model shared by unit tests and SITL, which the Flash model follows; the board and schema generator golden tests and diagnostic cases.

## Out of Scope

- The Bridge and ROS2 parameters (ADR-0003, ADR-0012), and the MAVLink parameter protocol.
- Arming, modes and anything that publishes `arming_state`; the armed paths are tested by publishing the topic in unit tests.
- `VEHICLE_TYPE`, `FRAME_CLASS`, frame presets and the defaults that depend on the vehicle type (ADR-0013).
- Turning `CTRL_RATE_HZ`, `CTRL_TRIGGER`, `IMU_SAMPLING` and `SERIALn_FUNCTION` into parameters, and runtime port assignment; they stay board-default constants until their features exist.
- Topic map rates as parameters, the primary estimator chosen by parameter, and the alignment thresholds as parameters.
- Metadata in the firmware or sent over the Link (PX4's component information).
- Parameter types other than int32 and float32, and string parameters.
- Stored sensor calibration.
- The event log in S3, which shares the Flash contract and write rules but belongs to the logging milestone (ADR-0019, ADR-0020).
- Parameter import and export files, and resetting all parameters to defaults from a tool.

## Further Notes

- **Why parameters now**: spec 0004 put attitude first so that parameters would have real cases to shape them: gains changed at runtime, a baud rate read at boot with a board default, and a board orientation that must reach the IMU driver before the first sample. All three are in this milestone.
- **Why protocol version 2 only**: the custom protocol has no peers outside this repository, and the Link monitor ships with the firmware build; keeping version 1 would mean a second handshake path that nothing uses.
- **Board orientation versus mounting rotation**: the board definition's `rotation` stays the IMU's rotation on the PCB, a hardware fact (ADR-0011); `SENS_BOARD_ROT` is how the board sits in the vehicle. The README's advice to fix a wrongly tilting estimate in `board.toml` changes accordingly.
- **Entry size**: 12 bytes leaves room for about 1360 entries per sector, so compaction is rare; with a few hundred parameters it still leaves room for many changes.
- **CPU and stall cost**: one 32-bit program stalls Flash fetches for about 16 µs (ADR-0020); with a cap of a few entries per `wq:slow` run, a set costs well under a millisecond of stall, and the timing monitor confirms it on the C50C.
