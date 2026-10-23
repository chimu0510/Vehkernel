# Build-time configuration is solved by Kconfig; CMake only builds

The build system is CMake + `CMakePresets.json` + Ninja, with one preset per build target, for example `c50c-v1_1`, `sitl` and `test`. Selecting modules and drivers involves constraints: dependencies (`depends on`), automatic selection (`select`) and mutually exclusive choices (`choice`). Rather than reimplementing these in CMake scripts or board TOML lists, Vehkernel uses Kconfig, solved with kconfiglib during CMake configuration; CMake only builds. PX4 does the same with `default.px4board` fragments and `cmake/kconfig.cmake`.

**Rules**

- **Kconfig decides what the firmware contains; parameters decide what is enabled after boot.** `VEHICLE_MODEL_WHEELED=y` is Kconfig, `FRAME_CLASS = WHEELED` is a parameter. Kconfig holds no runtime choices. The parameter generator filters parameter enum values by the Kconfig result, so options not compiled in never appear.
- **Hardware facts live only in the board TOML.** The board generator exports hardware capabilities as Kconfig symbols such as `BOARD_HAS_CAN` and `BOARD_IMU_BUS_SOFT_I2C`, like the `DT_HAS_*` symbols Zephyr derives from devicetree. Drivers `depends on` these symbols.
- **Three configuration layers**: MCU defaults → board defaults → debug overlay. Kconfig re-validates every constraint after each merge. Each board produces exactly one firmware image (ADR-0009) with every vehicle model the board supports compiled in and selected by parameters; there is no per-frame or per-product overlay layer.
- **`select` only for helper libraries that have no dependencies of their own**, such as `MATH_QUATERNION`; everything else uses `depends on`, because `select` does not check the dependencies of what it selects.
- Generators (schema, parameters, and the ULog format definitions and log profile bandwidth estimates of ADR-0019) run as CMake custom commands. The board generator is the exception and runs at configure time: a wrong board definition must fail configuration, and Kconfig needs the board's symbols before the build starts. Generated code is never committed.
- CMake target dependencies enforce the layering rules (ADR-0009).
- Every build checks the usage of Flash and of every RAM region the MCU has, such as CCM on the F4, and fails when the board budget is exceeded: each budget is a linker memory region the board generator emits, so the link itself fails.

## Consequences

- PX4 and Zephyr both generate a single configuration header, so changing any option triggers a large rebuild. Linux solves this with `fixdep`, which kconfiglib lacks. If needed later, per-module headers containing only the relevant options can be generated; the first version does not do this.
- If Flash ever cannot hold every vehicle model, a new ADR introduces a reduced firmware image, rather than splitting builds per frame in Kconfig.
