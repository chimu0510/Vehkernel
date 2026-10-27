# Building

## Development environment

Vehkernel builds in a container. Every tool that affects build output, from the Arm toolchain, CMake and Ninja to kconfiglib, clang-tidy and the Python tools, is pinned in the image that [`.devcontainer/Dockerfile`](../../.devcontainer/Dockerfile) builds ([ADR-0016](../adr/0016-dependency-policy.md)). CI builds in the same image. **Run `cmake` and `ctest` inside the container, not on the host.** On the host, configuring fails, for example with `No module named 'kconfiglib'`, or builds with a different compiler.

Third-party C code (FreeRTOS, CMSIS, ST LL, TinyUSB) is pinned as git submodules under `external/`. Clone with them, or run `git submodule update --init` in an existing clone:

```bash
git clone --recurse-submodules https://github.com/chimu0510/Vehkernel.git
```

There are two ways into the container.

**VS Code.** Open the repository with Dev Containers ("Reopen in Container"). Its terminal is inside the container, so the commands in these guides run as written.

**Docker on the command line**, as CI does. Build the image once from the repository root:

```bash
docker build -t vehkernel-dev -f .devcontainer/Dockerfile .
```

Then run each command through `docker run`. A shell function keeps that short:

```bash
vk() { docker run --rm -it -u "$(id -u):$(id -g)" -v "$PWD:$PWD" -w "$PWD" --network host vehkernel-dev "$@"; }
```

```bash
vk cmake --preset test
vk cmake --build --preset test
vk ctest --preset test
```

`ctest` runs what its own preset built: configure and build `test` before `ctest --preset test`.

The function's options:

- `-u` makes the build tree belong to your user.
- `-v "$PWD:$PWD"` mounts the repository at the same path as on the host. The build tree then records host paths, so host tools and editors can read it, including the Link monitor's `--msg-dir`, the flashing tools and `compile_commands.json`.
- `--network host` lets a Link monitor on the host reach SITL's UDP port.
- `-it` gives the command a terminal, for `menuconfig` and for stopping SITL with Ctrl+C. Leave it out in scripts, which have none.

Chain commands with `vk bash -c "cmake --preset sitl && cmake --build --preset sitl"`.

Rebuild the image after `.devcontainer/Dockerfile`, `pyproject.toml`, `uv.lock` or `.python-version` change; an outdated image lacks newer tools. A build directory configured anywhere else, on the host or at another mount path such as CI's `/work`, fails to configure again: delete it, for example `rm -rf build/sitl`, and configure again in the container.

### Container or host

| In the container | On the host |
|---|---|
| `cmake`, `ctest` and every preset's build | The Link monitor on a serial port (`--serial /dev/ttyACM0`), including recording a log from a board |
| SITL, and the SITL and unit tests; estimator replay and comparison (`tools/log/`) and pyulog's tools | Flashing: `tools/flash/flash_serial.py`, `stm32flash`, ArduPilot's `uploader.py` and OpenOCD |
| clang-tidy, the generators and the license check (`reuse lint`) | `tools/bringup/run_bringup.py`, which flashes over SWD |

The host tools need the board's USB devices. They read the build tree that the container wrote, and need only Python 3 with pyserial besides the tools below, at these versions:

| Tool | Version | Used for |
|---|---|---|
| OpenOCD | 0.12.0 | Flashing over SWD ([Flashing](flashing.md)) |
| [`stm32flash`](https://sourceforge.net/projects/stm32flash/) | 0.7 (`sudo apt install stm32flash`) | Flashing through the STM32 system bootloader ([Flashing](flashing.md)) |
| [pyserial](https://pypi.org/project/pyserial/) | 3.5, as `pyproject.toml` pins it (`sudo apt install python3-serial`) | The Link monitor on a serial port ([Link monitor](link-monitor.md)), and ArduPilot's `uploader.py`, which updates the firmware through the bootloader ([Flashing](flashing.md)) |

On the host, your user needs the `dialout` group for the serial ports (`sudo usermod -aG dialout $USER`, then log in again). Disable ModemManager (`sudo systemctl disable --now ModemManager`), and remove `brltty` if no braille display is used; `uploader.py` warns about both. ModemManager probes new `/dev/ttyACM*` devices and may drive their DTR and RTS lines, which on a download port can reset the board or hold BOOT0 high.

## Presets

There is one CMake preset per build target:

| Preset | Output |
|---|---|
| `c50c-v1_1` | C50C V1.1 firmware and bootloader. In `app/`: `vehkernel.elf`, `vehkernel.hex` and `vehkernel.bin` (the firmware alone, from S4), `vehkernel.apj` (ArduPilot's firmware file, ADR-0023) and `vehkernel_with_bl.hex` (the bootloader and the firmware, for a new board). In `bootloader/`: `vehkernel-bootloader.elf` and `.bin` (S0). Every firmware image carries ArduPilot's app descriptor, filled after linking with its CRCs, size, git hash, version and board ID |
| `c50c-v1_1-bringup` | The C50C V1.1 bring-up image ([Testing](testing.md#bring-up-image)) |
| `sitl` | SITL executable for the development host ([SITL](sitl.md)) |
| `test` | Host tests, run with `ctest --preset test` ([Testing](testing.md)), and `vehkernel-replay` in `test/`, the replay executable that `tools/log/replay.py` runs ([Logging](logging.md#estimator-replay-and-comparison)) |
| `fuzz` | The Link fuzz targets, built with Clang, libFuzzer and the sanitizers, run with `ctest --preset fuzz` ([Testing](testing.md#fuzzing-the-link-decoders)) |

Configure and build a preset in the container; with Docker, prefix each command with `vk`:

```bash
cmake --preset c50c-v1_1
cmake --build --preset c50c-v1_1
```

Every firmware and bootloader build prints the usage of each memory region; outgrowing a region fails the link (ADR-0017, ADR-0020). The bootloader's budget is the BOOT region, 16 KB on the C50C. The link also prints two warnings from newlib's `nosys` stubs, `_getpid is not implemented and will always fail` and `_kill is not implemented and will always fail`. They are expected: the C library links in its signal handling, and the firmware has no processes or signals for these stubs to serve.

The `sitl`, `test` and `fuzz` presets also run clang-tidy and treat every warning as an error. CI builds every preset and runs the tests in the same image.

## Build configuration (Kconfig)

Kconfig decides what each build contains (ADR-0017). The layers are the SoC defaults (`soc/<vendor>/<family>/<soc>/defconfig`, or `platform/sitl/defconfig` for SITL), the board defaults (`boards/<board>/defconfig`), and the debug overlays: the local `build/<preset>/debug.config`, which is never committed, then any fragments listed in the `VK_DEBUG_OVERLAY` cache variable. The `c50c-v1_1-bringup` preset uses the latter for the committed `test/hardware/bringup.config`. Edit the local overlay by hand or interactively (in the container, with `-it` for the terminal); the next build reconfigures:

```bash
cmake --build --preset c50c-v1_1 --target menuconfig
```
