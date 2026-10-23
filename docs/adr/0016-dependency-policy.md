# Third-party tools and packages are allowed, pinned and declared centrally

Build and development-host tools may use third-party packages, but versions must be pinned and declared centrally. Generators were once restricted to the Python standard library, but over-restriction means missing mature ecosystems such as kconfiglib and Jinja2 and rewriting worse versions ourselves. The rules:

- Versions are pinned and declared centrally: a lock file for Python packages, CMake FetchContent or submodules at a fixed version for C++ libraries, and exact versions for toolchains.
- A devcontainer gives local machines and CI the same environment.
- Tools that need the development host's USB devices, such as OpenOCD for a debug probe and `stm32flash` for the STM32 system bootloader, run outside the devcontainer. The README names them with the versions the project uses.
- External data that affects build output, such as the MCU database, is converted offline and committed, so builds need no network and stay reproducible.

Unmaintained packages such as kconfiglib may be kept as a copy in the repo when necessary.
