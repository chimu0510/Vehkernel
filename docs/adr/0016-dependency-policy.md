# Third-party tools and packages are allowed, pinned and declared centrally

Build and development-host tools may use third-party packages, but versions must be pinned and declared centrally. Generators were once restricted to the Python standard library, but over-restriction means missing mature ecosystems such as kconfiglib and Jinja2 and rewriting worse versions ourselves. The rules:

- Versions are pinned and declared centrally: a lock file for Python packages, CMake FetchContent or submodules at a fixed version for C++ libraries, and exact versions for toolchains. In the devcontainer, OS packages, the host compiler among them, come from the Ubuntu snapshot service at one timestamp in the Dockerfile, and container images are referenced by digest, never by tag alone. A tool downloaded as an archive, such as the Arm toolchain or uv, is pinned by version and by SHA256 for each architecture; uv is taken that way rather than from its image, because BuildKit resolves every referenced image before the first step and would fail on an unsupported architecture before the stop step below.
- A devcontainer gives local machines and CI the same environment. It supports linux/amd64 and linux/arm64 hosts, so a developer can build on an arm64 machine, such as a Raspberry Pi 5, a Jetson or Apple Silicon, as on an x86 one: the Dockerfile selects the Arm toolchain and uv archives and their SHA256 by `TARGETARCH`, and on arm64 maps ports.ubuntu.com, which apt leaves out, to the snapshot service's `/ubuntu/` path; every other layer already exists for both, and CI builds and tests on both (spec 0014). Its image build stops at once on any other architecture.
- Tools that need the development host's USB devices, such as OpenOCD for a debug probe and `stm32flash` for the STM32 system bootloader, run outside the devcontainer. The building guide (`docs/guide/building.md`) names them with the versions the project uses.
- External data that affects build output, such as the SoC database, is converted offline and committed, so builds need no network and stay reproducible.

Unmaintained packages such as kconfiglib may be kept as a copy in the repo when necessary.

**Where external code lives**: C and C++ code from elsewhere is pinned as git submodules under `external/`, the name Android's AOSP and many CMake projects use; a tool's own pinned external files sit beside it in an `external/` directory of its own, as `tools/bootloader/external/uploader.py` does. External code is never edited in place: it is updated by moving the pin, and REUSE and GitHub's language statistics treat it as vendored.

## Considered Options

- **amd64 hosts only for the devcontainer**: dropped on 2026-10-30 (spec 0014). It held while every developer used an x86 machine and the companion computer only flashed firmware built elsewhere. The vehicles' companion computers are arm64, and the maintainer works on them; the host tools themselves travel in a bundle (ADR-0026), but building and testing there needs the devcontainer on arm64.
- **Pinning each apt package to an exact version** (`g++-13=<version>`): rejected for packages from `noble-updates`, which drops a version once a newer one supersedes it, so the pin would later break the image build. The snapshot pins the whole set instead.
- **`third_party/` as the directory name** (Chromium, TensorFlow): replaced by `external/`. `modules/` (ArduPilot, Zephyr) and `lib/` (Betaflight) were taken by the module layer and the math libraries, and `vendor/` names the ignored directory that holds vendor datasheets and firmware backups.
