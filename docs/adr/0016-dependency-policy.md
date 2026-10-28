# Third-party tools and packages are allowed, pinned and declared centrally

Build and development-host tools may use third-party packages, but versions must be pinned and declared centrally. Generators were once restricted to the Python standard library, but over-restriction means missing mature ecosystems such as kconfiglib and Jinja2 and rewriting worse versions ourselves. The rules:

- Versions are pinned and declared centrally: a lock file for Python packages, CMake FetchContent or submodules at a fixed version for C++ libraries, and exact versions for toolchains.
- A devcontainer gives local machines and CI the same environment. It supports linux/amd64 hosts only, and its image build stops at once on any other architecture.
- Tools that need the development host's USB devices, such as OpenOCD for a debug probe and `stm32flash` for the STM32 system bootloader, run outside the devcontainer. The building guide (`docs/guide/building.md`) names them with the versions the project uses.
- External data that affects build output, such as the SoC database, is converted offline and committed, so builds need no network and stay reproducible.

Unmaintained packages such as kconfiglib may be kept as a copy in the repo when necessary.

**Where external code lives**: C and C++ code from elsewhere is pinned as git submodules under `external/`, the name Android's AOSP and many CMake projects use; a tool's own pinned external files sit beside it in an `external/` directory of its own, as `tools/bootloader/external/uploader.py` does. External code is never edited in place: it is updated by moving the pin, and REUSE and GitHub's language statistics treat it as vendored.

## Considered Options

- **arm64 hosts for the devcontainer** (Apple Silicon, aarch64 Linux): deferred. There is no current need: the companion computer flashes firmware built on an amd64 host, since the flashing tools run outside the container. Supporting them takes selecting the Arm toolchain archive and its SHA256 per `TARGETARCH`, and building the image for `linux/arm64` in CI.
- **`third_party/` as the directory name** (Chromium, TensorFlow): replaced by `external/`. `modules/` (ArduPilot, Zephyr) and `lib/` (Betaflight) were taken by the module layer and the math libraries, and `vendor/` names the ignored directory that holds vendor datasheets and firmware backups.
