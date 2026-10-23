# Building

## Development environment

The development environment is [`.devcontainer/`](../../.devcontainer/); every tool that affects build output is pinned ([ADR-0016](../adr/0016-dependency-policy.md)). Third-party C code (FreeRTOS, CMSIS, ST LL) is pinned as git submodules under `third_party/`, so clone with `git clone --recurse-submodules`, or run `git submodule update --init` in an existing clone. Open the repo with VS Code Dev Containers or the `devcontainer` CLI.

Tools that need the development host's USB devices run on the host, outside the devcontainer, at these versions:

| Tool | Version | Used for |
|---|---|---|
| OpenOCD | 0.12.0 | Flashing over SWD ([Flashing](flashing.md)) |
| [`stm32flash`](https://sourceforge.net/projects/stm32flash/) | 0.7 (`sudo apt install stm32flash`) | Flashing through the STM32 system bootloader ([Flashing](flashing.md)) |

## Presets

There is one CMake preset per build target:

| Preset | Output |
|---|---|
| `c50c-v1_1` | C50C V1.1 firmware: `vehkernel.elf`, `vehkernel.hex` (boot stub and firmware), `vehkernel-boot.bin` (S0), `vehkernel.bin` (firmware only, from S4) and `vehkernel.apj` (ArduPilot's firmware file, ADR-0023). Every image carries ArduPilot's app descriptor, filled after linking with its CRCs, size, git hash, version and board ID |
| `c50c-v1_1-bringup` | The C50C V1.1 bring-up image ([Testing](testing.md#bring-up-image)) |
| `sitl` | SITL executable for the development host ([SITL](sitl.md)) |
| `test` | Host tests, run with `ctest --preset test` ([Testing](testing.md)) |

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
```

Every firmware build prints the usage of each memory region; outgrowing a region fails the link (ADR-0017, ADR-0020).

The `sitl` and `test` presets also run clang-tidy and treat every warning as an error. CI builds every preset and runs the tests inside the same devcontainer image.

## Build configuration (Kconfig)

Kconfig decides what each build contains (ADR-0017). The layers are the MCU defaults (`mcu/<mcu>/defconfig`, or `platform/sitl/defconfig` for SITL), the board defaults (`boards/<board>/defconfig`), and the debug overlays: the local `build/<preset>/debug.config`, which is never committed, then any fragments listed in the `VK_DEBUG_OVERLAY` cache variable. The `c50c-v1_1-bringup` preset uses the latter for the committed `test/hardware/bringup.config`. Edit the local overlay by hand or interactively; the next build reconfigures:

```bash
cmake --build --preset c50c-v1_1 --target menuconfig
```
