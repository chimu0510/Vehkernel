# Flashing

Every board is flashed through its bootloader (ADR-0023). A board without an ArduPilot or PX4 bootloader of its own, such as the C50C, gets the Vehkernel bootloader once, through the STM32 system bootloader or a debug probe, and every later update goes through it with ArduPilot's `uploader.py`. The board's connectors and ports are on its board page, for example [C50C V1.1](../boards/c50c-v1_1.md).

## The Vehkernel bootloader

The Vehkernel bootloader sits in the BOOT region, S0 on the C50C, and speaks the bootloader protocol of ArduPilot and PX4 at revision 5 on the ports the board definition lists: SERIAL2 and SERIAL1 on the C50C, both at 115200 8N1, polled. After the first good command it answers on that port only.

At every start it reads RTC backup register 0, where the firmware leaves its request, and clears it:

- After `reboot --bootloader` (`RTC_BOOT_HOLD`) it stays until it is flashed or reset.
- After a plain `reboot` (`RTC_BOOT_FAST`) it starts the firmware at once.
- Otherwise it listens for 1 s, the boot window, then starts the firmware. Power-cycling the board while `uploader.py` waits is therefore always a way in.

It starts the firmware only when the app descriptor is present and its CRCs, size and board ID match, and when its reset handler lies in the FIRMWARE region; otherwise it stays, so a cut-off update or an image for another board never runs. An update erases and writes the FIRMWARE region only (S4 to S7), so the stored parameters and the event log survive it.

## A new board: `new-board`

On a board with a download circuit, such as the C50C's one-key download port, the adapter's RTS drives BOOT0 and its DTR, with RTS, drives NRST, so the STM32 system bootloader is entered and left without touching the board; the board definition declares this circuit on the port (`download = "DTR_RTS"`). `tools/flash/flash_serial.py` runs `stm32flash` on the host with that sequence and takes the Flash layout from the build's board definition. It reads `build/c50c-v1_1` and `/dev/ttyACM0` unless given `--build` and `--port`, which go before the command, as in `flash_serial.py --port /dev/ttyACM1 reset`. Device numbers follow the order the cables were plugged in; the board page says how to tell the ports apart.

A new board, or one still running its vendor firmware, is flashed once with `new-board`. The command backs up the whole chip into `<board>_flash_<time>.bin` in the current directory, or the file given with `--backup`. It then erases the whole chip and writes `vehkernel_with_bl.hex`: the Vehkernel bootloader in S0 and the firmware from S4. Nothing of the vendor firmware, its settings or old parameters is left. The command takes about 2 minutes at 115200 baud. Nothing is erased if the backup fails or comes back short, and an existing backup file is never overwritten; `--no-backup` erases without one. Keep each board's backup outside the repository: vendor firmware differs from board to board.

```bash
tools/flash/flash_serial.py new-board
```

`backup <file>` only reads the chip, and `reset` restarts the board through the download circuit. The system bootloader runs at 115200 baud unless given `--baud`. The same commands rescue a board whose bootloader is damaged.

## Updates: `update`

Every later update goes through the Vehkernel bootloader:

```bash
tools/flash/flash_serial.py update
```

1. It asks the firmware over its Link on the port to reboot into the bootloader, as `link_monitor.py --serial <port> reboot --bootloader` does (the Link at 921600 baud unless given `--link-baud`). A firmware that refuses, because it is armed, stops the update there.
2. When no Link answers, for example because the firmware does not run, it starts `uploader.py` first and, once it waits for the bootloader, restarts the board through the download circuit, so the boot window catches it.
   `uploader.py` usually prints `Attempting reboot on <port> with baudrate=57600...` once before `Found board`: it looked for the bootloader before the board had finished resetting. That is expected.
3. The pinned `uploader.py` (`tools/bootloader/external/`) writes `vehkernel.apj`, verifies it and starts the firmware. It runs through `tools/flash/run_uploader.py`, which releases DTR and RTS each time `uploader.py` opens the port: on a download port, RTS asserted holds BOOT0 high, and the reset after `reboot --bootloader` would then start the STM32 system bootloader instead of the board's.

The tool fails with exit code 1 when `stm32flash` or `uploader.py` fails, the firmware refuses to reboot, the bootloader refuses an image built for another board, or no bootloader answers within 3 minutes, and with 2 when an image does not fit its region, the build, `stm32flash` or `uploader.py` is missing, or the backup file already exists. `uploader.py` needs pyserial on the host ([Building](building.md)).

`update` also works on a port without a download circuit, such as the C50C's SERIAL1 (`--port` its Type-C adapter): over the Link, or, when no Link answers, by power-cycling the board while `update` waits. `uploader.py` may also be run directly:

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 reboot --bootloader
python3 tools/bootloader/external/uploader.py --port /dev/ttyACM0 --no-extf build/c50c-v1_1/app/vehkernel.apj
```

## Over SWD

Connect an ST-Link to the board's SWD header. OpenOCD runs on the host, outside the development container, since the probe is a USB device. A new board gets the bootloader and the firmware; later, the firmware alone:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/c50c-v1_1/app/vehkernel_with_bl.hex verify reset exit"
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/c50c-v1_1/app/vehkernel.elf verify reset exit"
```

Only the bootloader (S0) and firmware (S4 onward) sectors are written; the parameter and event log sectors in between are left untouched. The firmware needs the bootloader in S0 to start after a reset.

## Rebooting into the bootloader

The [Link monitor](link-monitor.md) restarts a disarmed board over any Link with `reboot`, or into the bootloader with `reboot --bootloader`, ready for `uploader.py` (ADR-0023); both are refused while armed, and the board resets about 0.1 s after it replies. The request goes into RTC backup register 0 with ArduPilot's values (`RTC_BOOT_HOLD` for the bootloader, `RTC_BOOT_FAST` otherwise), or PX4's on a board that keeps PX4's bootloader. In SITL a reboot ends the run with exit status 10, or 11 for the bootloader (`sitl_reboot`).

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 reboot --bootloader
```
