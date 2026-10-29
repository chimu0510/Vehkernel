# Vehkernel Host Tools Bundle

This bundle contains the host tools, generated schema decoders, parameter metadata, and firmware images for a single build of Vehkernel (ADR-0026, spec 0014). It runs on any architecture without building the firmware.

## Requirements

- Python 3.10 or newer
- pyserial: `sudo apt install python3-serial` (or `pip install pyserial` in a virtual environment)
- stm32flash (for initial flashing of a new board over a download port): `sudo apt install stm32flash`

## Running from the bundle

Run every command from the unpacked bundle directory. The tools detect the bundle and automatically use its generated message definitions, parameter metadata, and firmware images as defaults without extra `--msg-dir`, `--param-meta`, or `--build` flags.

### Monitoring and checks

Connect to SITL over UDP or to a board over serial:

```bash
tools/link/link_monitor.py --udp 127.0.0.1:15600
tools/link/link_monitor.py --serial /dev/ttyACM0
```

Run a check for 5 seconds expecting IMU samples at 200 Hz:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 --check --duration 5 --expect-rate imu_sample:0=200
```

### Parameters

List all parameters with descriptions and units:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 param list
```

Read or set a parameter:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 param get ATT_KP
tools/link/link_monitor.py --serial /dev/ttyACM0 param set ATT_KP 2.5
tools/link/link_monitor.py --serial /dev/ttyACM0 param set SENS_BOARD_ROT YAW_90
```

### Recording logs

Record a ULog file over the Link with a log profile:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 record drive.ulg --profile estimator_replay --duration 30
```

### Flashing

On a board bundle carrying firmware images in `app/`, update the firmware through the Vehkernel bootloader (parameters are preserved):

```bash
tools/flash/flash_serial.py update
```

To back up and flash a new board through its download circuit:

```bash
tools/flash/flash_serial.py new-board
```
