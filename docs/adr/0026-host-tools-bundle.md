# The host tools reach a companion computer as a bundle built with the firmware

The Link monitor, the log tools and the flashing tool are Python with only the standard library and pyserial, so they run on any architecture. What they cannot do without a build is know the firmware: the Link monitor decodes topics and checks their schema hashes with the generated `vkmsg.py`, gives parameter names their types, units, ranges and enum values from the generated `params.json`, and the flashing tool takes the Flash layout from the generated `board.json` and the images from `app/`. All of it is produced by `cmake --build` in the development container, which a companion computer such as a Raspberry Pi 5 or a Jetson does not run. On 2026-10-30 the maintainer, setting a parameter from the vehicle's Raspberry Pi 5, got `vkmsg.py does not exist; build a preset first`.

The decision: **each firmware build also produces a host tools bundle**, architecture-independent and tied to that build, that a companion computer unpacks and uses without building anything.

- **What it holds**: the host tools (`tools/link/`, the Python log tools of `tools/log/` that need no native program, `tools/flash/` with the pinned `uploader.py`), and the build's own outputs that the tools read, at the same relative paths as in the build tree: `generated/schema/python/`, `generated/params/params.json`, `generated/board/board.json` on a board, and the firmware images in `app/`. A `manifest.json` gives the preset, the board, the git commit, the protocol version, the parameter set hash and the minimum Python version.
- **Its layout is a build tree's**: the tools already take `--build <dir>` and `--msg-dir <dir>`, and in the bundle their defaults point at the bundle itself, so the same commands work in the repository and from an unpacked bundle, and `flash_serial.py update` can flash from the companion computer the image the bundle carries.
- **How it is made and shipped**: a CMake target writes it to `build/<preset>/bundle/` and packs it as `vehkernel-<preset>-<git describe>.tar.gz`. CI builds it for each board preset on every push and keeps it as an artifact, so the companion computer can fetch the bundle that matches a commit instead of the developer copying files by hand.
- **Python**: the bundled tools run on the oldest Python of the companion computers in use: 3.10, Ubuntu 22.04's, as on JetPack 6. The development container keeps its pinned Python (ADR-0016); CI runs the bundled tools' tests on 3.10 as well.
- **Finding the right metadata**: a tool reading a build tree whose schema hashes differ from the MCU's says so (it does already) and, in the repository, names any other build tree whose hashes match, since the usual cause is a stale tree of another preset.

## Considered Options

- **Building the tools on the companion computer**: rejected. The generators need the build configuration (Kconfig) and, on a board, its definition; running them means running the build, which is what the companion computer should not need.
- **Copying `build/<preset>/generated/` by hand**: a workaround until the bundle exists, not a design. Nothing ties the copy to the firmware on the board, and the flashing tool needs more than the generated messages.
- **The MCU serving its own metadata over the Link**, as PX4 serves parameter and message metadata to QGroundControl through MAVLink's component information: deferred. It removes the need for any file on the companion computer, but adds to the protocol (ADR-0002 lets a version only add) and to the Flash the firmware spends. The bundle comes first; the protocol can follow when a client, such as the Bridge, cannot be given a bundle.
- **Cross-compiling for linux/arm64**: not needed for this. The bundle holds no native program. The native programs a developer runs, SITL and `vehkernel-replay`, build natively in the development container on an arm64 host (ADR-0016). The Bridge, the first native program meant for the companion computer, is built the ROS2 way on its target or in ROS2's arm64 images, and its spec decides how.

## Consequences

- The bundle and the firmware image of one build always agree, so `flash_serial.py update` from an unpacked bundle flashes a firmware the bundle's Link monitor can read.
- The host tools gain a compatibility floor below the development container's Python: a tool may not use a feature newer than 3.10, which CI checks.
- `tools/log/replay.py` stays on the development host: it needs `vehkernel-replay`, a native program. `compare.py`, `cut.py` and `mag_cal.py` travel in the bundle.
