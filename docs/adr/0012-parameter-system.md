# Parameter system: direct struct access, per-entry storage keyed by name hash

In RAM, parameters are plain structs that modules read directly; in Flash they are stored entry by entry, keyed by a hash of the parameter name, so changing the layout of a parameter group never invalidates user settings. This combines Betaflight's access speed, PX4's storage robustness, and the metadata of PX4 and ArduPilot.

- **Definition**: one TOML file per module or driver, giving the description, type, unit, range, default and safety attributes. It shares the board definition toolchain, whose versions are pinned by ADR-0016.
- **Access**: the generator turns each parameter group into a plain struct in RAM; modules read members directly, with no lookup. Every change publishes `parameter_update` on the bus.
- **Storage**: entries keyed by the name hash, storing only non-default values. At boot the values are filled back into the structs by hash and unknown hashes are skipped, so no layout change ever resets user settings. Flash uses two alternating 16 KB sectors with append-only writes, which survives power loss. Sector placement and the write limits while armed are in ADR-0020.
- **Default layering**, lowest first: parameter definition (which may give different defaults per vehicle type, ADR-0013) → board definition → frame preset → user value. `VEHICLE_TYPE` and `FRAME_CLASS` are resolved first (ADR-0013).
- **Safety attributes**: `reboot_required`, and `locked_when_armed` (no changes while armed).
- **Names**: at most 16 characters, upper case with a module prefix, for example `RD_WHEEL_TRACK`, for compatibility with the MAVLink parameter protocol. Every Link and ROS2 use the same name.
- **ROS2**: the MCU is the single source of truth. On connection the Bridge pulls every parameter and declares it as a ROS2 parameter from its metadata. The MCU validates each change against the range and safety attributes and returns the reason when it rejects one.

## Considered Options

- **Betaflight's whole-block PG storage**: its storage is rejected. Any struct change bumps the version and resets the whole group to defaults. Only its direct struct access is adopted.
- **Unlimited name length, with the MAVLink protocol generating a short-name mapping**: rejected. One parameter would have two names, complicating documentation, debugging and search.
