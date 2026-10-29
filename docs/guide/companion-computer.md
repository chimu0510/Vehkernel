# Companion computer

A companion computer, such as a Raspberry Pi 5 or an NVIDIA Jetson, runs vehicle applications (like ROS2) and talks to Vehkernel over a Link (ADR-0001, ADR-0002). Companion computers are usually arm64 Linux systems, and they need not build anything to use the tools: no CMake, no development container.

To monitor, configure, log, and flash the board from the companion computer, each firmware build produces an architecture-independent **host tools bundle** (ADR-0026, spec 0014).

## The host tools bundle

The host tools (the Link monitor `tools/link/link_monitor.py`, log utilities in `tools/log/`, and the serial flasher `tools/flash/flash_serial.py`) are pure Python scripts requiring only the standard library and `pyserial`. However, to function they need build outputs:
- Generated Python message decoders (`generated/schema/python/vkmsg.py`).
- Parameter metadata (`generated/params/params.json`), providing names, types, units, ranges, and enum values.
- The board Flash layout (`generated/board/board.json`) and firmware images (`app/vehkernel.bin`, `vehkernel.apj`, `vehkernel.elf`, `vehkernel.hex`, and `vehkernel_with_bl.hex`).

The host tools bundle gathers these files at their standard repository paths, alongside `manifest.json` recording the preset, board, git commit, protocol version, parameter set hash, and minimum Python version.

### Python compatibility floor

Bundled tools support Python 3.10 or newer (the default on Ubuntu 22.04 LTS and JetPack 6). They do not use features newer than 3.10 (such as `tomllib`, `typing.Self`, or `ExceptionGroup`).

Install `pyserial` on the companion computer:

```bash
sudo apt install python3-serial
```

(Raspberry Pi OS and recent Ubuntu refuse a system-wide `pip install`; the distribution's package, or `pip install pyserial` in a virtual environment, does the same.)

For flashing a new board over a download port through the STM32 system bootloader, install `stm32flash`:

```bash
sudo apt install stm32flash
```

## Getting a bundle

### From CI

CI builds the bundle for every board preset on each push and keeps it as the `bundles` artifact (ticket #173). Download the archive `vehkernel-<preset>-<commit>.tar.gz` that matches the firmware commit.

### From a development build

On a development host, building any board preset or `sitl` automatically creates the bundle directory and archive:

```bash
cmake --build --preset c50c-v1_1
```

This writes:
- The unpacked directory: `build/c50c-v1_1/bundle/`
- The compressed archive: `build/c50c-v1_1/vehkernel-c50c-v1_1-<commit>.tar.gz`

Copy the archive to the companion computer:

```bash
scp build/c50c-v1_1/vehkernel-c50c-v1_1-*.tar.gz pi@companion:/home/pi/
```

## Unpacking and running

On the companion computer, unpack the archive:

```bash
tar -xzf vehkernel-c50c-v1_1-*.tar.gz
cd vehkernel-c50c-v1_1-*
```

When run from inside the unpacked bundle, the tools automatically locate `manifest.json` two levels above their own directories. Their defaults for `--msg-dir`, `--param-meta`, and `--build` point to the bundle itself, so no extra path options are needed.

### Monitoring and checks

Start the live monitor over the serial Link (defaulting to 921600 baud):

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0
```

Run a 5-second health check:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --check --duration 5 --expect-rate imu_sample:0=200
```

### Parameters

List all parameters with their descriptions, units, and values:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 param list
```

Read or set parameters by name (enum values can be given by name):

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 param get ATT_KP
tools/link/link_monitor.py --serial /dev/ttyACM0 param set ATT_KP 2.5
tools/link/link_monitor.py --serial /dev/ttyACM0 param set SENS_BOARD_ROT YAW_90
```

### Recording logs

Record a ULog file over the Link with a selected profile ([Logging](logging.md)):

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 record drive.ulg --profile estimator_replay --duration 30
```

The recorded log can be inspected or calibrated on the companion computer using the bundled log tools (e.g. `tools/log/mag_cal.py`). Note that `replay.py` requires native binary `vehkernel-replay` and remains on the development host.

### Flashing firmware

A board bundle carries the firmware images in `app/`. To update the firmware through the Vehkernel bootloader ([Flashing](flashing.md)):

```bash
tools/flash/flash_serial.py update
```

The tool asks the board over the Link to reboot into its bootloader, then flashes `app/vehkernel.apj` via `uploader.py`. Stored parameters are preserved.

For a new or corrupted board, write the bootloader and firmware through the download port with:

```bash
tools/flash/flash_serial.py new-board
```

## Workaround before bundles

Before host tools bundles were introduced, the workaround was copying `build/<preset>/generated/` and `app/` by hand from a development host to identical relative paths on the companion computer, and passing `--msg-dir` and `--param-meta` explicitly. The bundle replaces this workaround with self-contained, reproducible, commit-tagged archives.
