# Build-time configuration is solved by Kconfig; CMake only builds

Selecting modules and drivers involves constraints: dependencies (`depends on`), automatic selection (`select`) and mutually exclusive choices (`choice`). Rather than reimplementing these in CMake scripts or board TOML lists, Vehkernel solves them with Kconfig (kconfiglib) during CMake configuration, as PX4 does. CMake, with one preset per build target, only builds.

- **Kconfig decides what the firmware contains; parameters decide what is enabled after boot.** `VEHICLE_MODEL_WHEELED=y` is Kconfig, `FRAME_CLASS = WHEELED` is a parameter. Kconfig holds no runtime choices. Parameter enum values are filtered by the Kconfig result, so options not compiled in never appear.
- **Hardware facts live only in the board TOML.** The board generator exports them as `BOARD_HAS_*` symbols, like the `DT_HAS_*` symbols Zephyr derives from devicetree, and drivers `depends on` them.
- **Layers are MCU defaults, board defaults and a debug overlay**, with every constraint re-validated after each merge. Each board produces exactly one firmware image (ADR-0009) with every vehicle model the board supports compiled in and selected by parameters; there is no per-frame or per-product layer.
- **`select` only for helper libraries that have no dependencies of their own**, such as `MATH_QUATERNION`; everything else uses `depends on`, because `select` does not check the dependencies of what it selects.
- Generators run as CMake custom commands, except the board generator, which runs at configure time because a wrong board definition must fail configuration and Kconfig needs its symbols. Generated code is never committed.
- CMake target dependencies enforce the layering rules (ADR-0009), and every build fails when it exceeds a board memory budget.

## Consequences

- PX4 and Zephyr both generate a single configuration header, so changing any option triggers a large rebuild. Linux solves this with `fixdep`, which kconfiglib lacks. If needed later, per-module headers containing only the relevant options can be generated; the first version does not do this.
- If Flash ever cannot hold every vehicle model, a new ADR introduces a reduced firmware image, rather than splitting builds per frame in Kconfig.
