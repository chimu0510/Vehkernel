# Spec 0014: the host tools on an arm64 companion computer

**Status**: in progress. Issue: [#171](https://github.com/chimu0510/Vehkernel/issues/171); tickets #172 to #176, with the hardware check in #176.

Until this spec is done, this file and its issue carry the same spec; update both in the same edit.

## Problem Statement

The vehicles' companion computers are arm64: the maintainer's UGV carries a Raspberry Pi 5, and a Jetson is the other usual choice. The project's tools assume a development host:

- **The host tools need a build.** The Link monitor, the log tools and the flashing tool are Python with the standard library and pyserial, so they run on arm64. But they read files that only `cmake --build` writes: the generated messages (`vkmsg.py`), the parameter metadata (`params.json`), the board's Flash layout (`board.json`) and the firmware images. On the vehicle's Raspberry Pi 5, `link_monitor.py param set SENS_BOARD_ROT YAW_180` fails with `vkmsg.py does not exist; build a preset first`, so parameters cannot be set, the vehicle cannot be monitored, and a hardware check cannot be run from the computer that sits on the vehicle.
- **The build needs an amd64 machine.** The development container supports linux/amd64 only (ADR-0016, #146): its Arm toolchain is the x86_64 archive. On an arm64 computer nothing can be built, so the files above cannot be made there either.
- **A stale build tree misleads.** The Link monitor reads `build/sitl/` unless told otherwise; on a development host where that tree is older than the firmware on the board, it reports schema hash mismatches for topics that are fine (found on 2026-10-30).

## Solution

Deliver the fourteenth milestone: **the host tools on an arm64 companion computer, from a bundle built with the firmware, and the development container on arm64**. Afterwards:

- Each board preset's build also writes a host tools bundle (ADR-0026): the host tools and the build's own generated metadata and firmware images, laid out as a build tree, with a manifest. CI keeps it as an artifact for every push.
- On the companion computer, an unpacked bundle runs the Link monitor (parameters, monitoring, recording a log) and flashes the firmware it carries, with Python 3.10 or newer and pyserial, and `stm32flash` from the distribution.
- The development container builds and tests on linux/arm64 as on linux/amd64, and CI runs both.
- The Link monitor, on a schema hash mismatch in the repository, names any other build tree whose hashes match the MCU's.

## User Stories

1. As a vehicle operator, I want to set parameters and watch the vehicle from its companion computer, so that I do not need a development host beside the vehicle.
2. As a vehicle operator, I want to flash the firmware from the companion computer, so that an update needs only the vehicle and a file.
3. As a vehicle operator, I want the tools on the companion computer to match the firmware on the board, so that a mismatch cannot come from copying the wrong files.
4. As a developer on an arm64 machine (a Raspberry Pi 5, a Jetson, Apple Silicon), I want to build, run SITL and run the tests in the development container, so that I can work on the computer I have.
5. As a developer, I want CI to keep a bundle for every commit, so that I can give a vehicle the tools and the firmware of any commit.
6. As a developer, I want the Link monitor to tell me which build tree matches the board when the one it reads does not, so that a stale tree does not look like a firmware fault.
7. As a contributor, I want CI to fail when a host tool uses a Python feature the companion computers lack, so that the bundle keeps working on them.

## Implementation Decisions

- **The bundle** (ADR-0026):
  - Contents: `tools/link/` (`link_monitor.py`, `vklink/`), `tools/log/` without `replay.py` (`vkulog/`, `compare.py`, `cut.py`, `mag_cal.py`), `tools/flash/` (`flash_serial.py`, `run_uploader.py`) with `tools/bootloader/external/uploader.py`, and from the build tree `generated/schema/python/`, `generated/params/params.json`, `generated/board/board.json` on a board, and `app/vehkernel.{bin,apj,elf}` (and `vehkernel_with_bl.hex` where the board has the Vehkernel bootloader). Tests, `__pycache__` and the generators stay out.
  - Layout: the tools under `tools/` at their repository paths, the build outputs at their build tree paths, so `--build <bundle>` and `--msg-dir <bundle>/generated/schema/python` work as with a build tree. `manifest.json`: `preset`, `board`, `commit` (`git describe --always --dirty`), `protocol_version`, `param_set_hash`, `python_min` (`3.10`).
  - Defaults: each tool finds the directory it was started from; if `manifest.json` is two levels up (`<bundle>/tools/<tool dir>/`), that bundle is its default build directory and message directory, before the repository's `build/sitl/` default and after the explicit options and `VK_MSG_PYTHON_DIR`.
  - Making it: a CMake target `bundle` (part of `all` on every preset, `sitl` included for the check against SITL) writes `build/<preset>/bundle/` and `build/<preset>/vehkernel-<preset>-<commit>.tar.gz`, with the file modes of the tools kept (executable scripts stay executable) and a fixed file order, so two builds of one commit pack the same files.
  - `tools/bundle/README.md` inside the bundle says how to use it: Python and pyserial, `stm32flash` from apt, the commands for parameters, monitoring, recording and flashing.
- **CI**: the board presets' bundles are uploaded as one artifact per push (`bundles`), kept as long as GitHub's default. A job runs the bundled tools' unit tests (`tools/link/tests`, `tools/log/tests` without the replay tests, `tools/flash/tests`) with Python 3.10 and pyserial, from an unpacked bundle, outside the development container. The tests travel as a test-only copy taken from the checkout, not in the bundle; the job's packages are pinned in one file, `tools/bundle/tests/requirements-py310.txt` (ADR-0016), and include pytest's own dependencies and pyulog and numpy, which two tests need and the bundled tools do not. The build job's container runs as root on the runner user's checkout, so it marks the checkout a git `safe.directory`; without it `git describe` fails and the bundles are named `unknown` (found in #173).
- **Python 3.10**: the host tools in the bundle use nothing newer than 3.10 (no `tomllib`, `typing.Self`, `ExceptionGroup`); `tomllib` stays in the generators, which run in the container. The test job above is the check.
- **The development container on arm64** (ADR-0016): the Dockerfile picks `arm-gnu-toolchain-<version>-x86_64-arm-none-eabi.tar.xz` or `...-aarch64-arm-none-eabi.tar.xz` and its pinned SHA256 by `TARGETARCH`, and stops on any other architecture with the message of #146 updated. uv comes the same way, from its release tarball pinned by SHA256 per architecture, not from its image: BuildKit resolves every image a Dockerfile references before the first step, and the uv image has no variant for other architectures, so the build would fail on image resolution before the stop step (#175). On arm64 the OS packages come from ports.ubuntu.com, which apt does not map to the Ubuntu snapshot service, so the Dockerfile maps it to the service's `/ubuntu/` path, which holds every architecture (the service has no `ubuntu-ports` path). The other layers (g++, clang, the Python tools, gtest) are checked to build for linux/arm64. CI adds the same build-and-test job on GitHub's `ubuntu-24.04-arm` runner; the firmware images of the two need not be identical, and the bundle artifact comes from the amd64 job.
- **Stale build trees**: on a mismatch, `link_monitor.py` looks through `build/*/generated/schema/python/` in the repository and prints the `--msg-dir` of each tree whose hashes for the mismatched topics equal the MCU's. It skips the message directory in use and trees whose `vkmsg.py` is missing or fails to load, prints paths relative to the current directory when they lie under it and absolute otherwise, and prints beside the mismatch warnings, in the non-interactive modes only (#174).
- **Documentation**: a Companion computer guide, `docs/guide/companion-computer.md` (getting a bundle from CI or from a build, unpacking it, the tools' commands, flashing, the Python floor); the Building guide for arm64 hosts; the Flashing and Link monitor guides for running from a bundle. `GLOSSARY.md` gains Host Tools Bundle. ADR-0026 is new, and ADR-0016 is edited for arm64.

## Testing Decisions

- **Seam A: the bundle (L0).** A test unpacks a freshly built bundle into an empty directory and checks its file list against the contents above, the manifest's fields against the build, that the tools' defaults resolve to the bundle, and that two builds of one commit pack the same files with the same modes.
- **Seam B: the tools on Python 3.10 (L1).** The CI job runs the bundled tools' unit tests on Python 3.10 from the unpacked bundle.
- **Seam C: a bundle against SITL (L3).** From an unpacked bundle of the `sitl` preset, outside the build tree, `link_monitor.py --udp` handshakes with SITL without a mismatch, reads and sets a parameter with its metadata, and records a short log.
- **Seam D: the stale tree hint (L1).** A unit test with two message directories, one matching the MCU's hashes, checks the hint names the matching one.
- **Seam E: arm64 (CI).** The development container builds on `ubuntu-24.04-arm`, every preset builds there, and `ctest --preset test` passes.
- **Seam F: hardware (L4, manual).** On the maintainer's Raspberry Pi 5, with a bundle downloaded from CI: the Link monitor's `--check` with no mismatch, `param get` and `param set` with metadata, a short recorded log, and `flash_serial.py update` of the bundle's firmware; and the development container built and the `test` preset passing on the Raspberry Pi 5. Results are recorded on the ticket.
- **Prior art**: the existing tests of `tools/link`, `tools/log` and `tools/flash`; `test/sitl/check_link.py` for a SITL handshake.

## Out of Scope

- The MCU serving its own metadata over the Link (ADR-0026, considered options).
- Building or packaging the Bridge for arm64; the Bridge's spec decides.
- Cross-compiling SITL or `vehkernel-replay` for arm64: they build natively in the container on an arm64 host.
- Replay (`tools/log/replay.py`) on the companion computer.
- Installing the tools as a Python package (pip, wheels) or a Debian package.
- arm32 (armhf) companion computers.

## Further Notes

- ADR-0025 is reserved for spec 0013's examples, which is deferred, so this spec's ADR is ADR-0026.
- Until the bundle exists, copying `build/<preset>/generated/` and `app/` from a development host to the same paths on the companion computer works around the problem.
