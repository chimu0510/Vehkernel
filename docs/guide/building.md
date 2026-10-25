# Building

## Development environment

The development environment is [`.devcontainer/`](../../.devcontainer/); every tool that affects build output is pinned ([ADR-0016](../adr/0016-dependency-policy.md)). Third-party C code (FreeRTOS, CMSIS, ST LL) is pinned as git submodules under `third_party/`, so clone with `git clone --recurse-submodules`, or run `git submodule update --init` in an existing clone. Open the repo with VS Code Dev Containers or the `devcontainer` CLI.

Tools that need the development host's USB devices run on the host, outside the devcontainer, at these versions:

| Tool | Version | Used for |
|---|---|---|
| OpenOCD | 0.12.0 | Flashing over SWD ([Flashing](flashing.md)) |
| [`stm32flash`](https://sourceforge.net/projects/stm32flash/) | 0.7 (`sudo apt install stm32flash`) | Flashing through the STM32 system bootloader ([Flashing](flashing.md)) |
| [pyserial](https://pypi.org/project/pyserial/) | 3.5, as `pyproject.toml` pins it | ArduPilot's `uploader.py`, which updates the firmware through the bootloader ([Flashing](flashing.md)) |

Disable ModemManager on the host (`sudo systemctl disable --now ModemManager`), and remove `brltty` if no braille display is used; `uploader.py` warns about both. ModemManager probes new `/dev/ttyACM*` devices and may drive their DTR and RTS lines, which on a download port can reset the board or hold BOOT0 high.

## Presets

There is one CMake preset per build target:

| Preset | Output |
|---|---|
| `c50c-v1_1` | C50C V1.1 firmware and bootloader. In `app/`: `vehkernel.elf`, `vehkernel.hex` and `vehkernel.bin` (the firmware alone, from S4), `vehkernel.apj` (ArduPilot's firmware file, ADR-0023) and `vehkernel_with_bl.hex` (the bootloader and the firmware, for a new board). In `bootloader/`: `vehkernel-bootloader.elf` and `.bin` (S0). Every firmware image carries ArduPilot's app descriptor, filled after linking with its CRCs, size, git hash, version and board ID |
| `c50c-v1_1-bringup` | The C50C V1.1 bring-up image ([Testing](testing.md#bring-up-image)) |
| `sitl` | SITL executable for the development host ([SITL](sitl.md)) |
| `test` | Host tests, run with `ctest --preset test` ([Testing](testing.md)) |
| `fuzz` | The Link fuzz targets, built with Clang, libFuzzer and the sanitizers, run with `ctest --preset fuzz` ([Testing](testing.md#fuzzing-the-link-decoders)) |

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
```

Every firmware and bootloader build prints the usage of each memory region; outgrowing a region fails the link (ADR-0017, ADR-0020). The bootloader's budget is the BOOT region, 16 KB on the C50C.

The `sitl`, `test` and `fuzz` presets also run clang-tidy and treat every warning as an error. CI builds every preset and runs the tests inside the same devcontainer image.

## Build configuration (Kconfig)

Kconfig decides what each build contains (ADR-0017). The layers are the SoC defaults (`soc/<vendor>/<family>/<soc>/defconfig`, or `platform/sitl/defconfig` for SITL), the board defaults (`boards/<board>/defconfig`), and the debug overlays: the local `build/<preset>/debug.config`, which is never committed, then any fragments listed in the `VK_DEBUG_OVERLAY` cache variable. The `c50c-v1_1-bringup` preset uses the latter for the committed `test/hardware/bringup.config`. Edit the local overlay by hand or interactively; the next build reconfigures:

```bash
cmake --build --preset c50c-v1_1 --target menuconfig
```
