# Flashing

Every board is flashed through its bootloader (ADR-0023); a debug probe and the STM32 system bootloader install the bootloader on a new board and rescue a damaged one. Until the Vehkernel bootloader runs on the C50C (spec 0008), the C50C is flashed over SWD or through the system bootloader. The board's connectors and ports are on its board page, for example [C50C V1.1](../boards/c50c-v1_1.md).

## Over SWD

Connect an ST-Link to the board's SWD header. OpenOCD runs on the host, outside the devcontainer, since the probe is a USB device:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/c50c-v1_1/app/vehkernel.elf verify reset exit"
```

Only the boot stub (S0) and firmware (S4 onward) sectors are written; the parameter and event log sectors in between are left untouched.

## Over the serial download port

On a board with a download circuit, such as the C50C's one-key download port, the adapter's RTS drives BOOT0 and its DTR, with RTS, drives NRST, so the STM32 system bootloader is entered and left without touching the board; the board definition declares this circuit on the port (`download = "DTR_RTS"`). `tools/flash/flash_serial.py` runs `stm32flash` on the host with that sequence and takes the Flash layout from the build's board definition. It reads `build/c50c-v1_1` and `/dev/ttyACM0` unless given `--build` and `--port`.

A new board, or one still running its vendor firmware, is flashed once with `new-board`. The command backs up the whole chip into `<board>_flash_<time>.bin` in the current directory, or the file given with `--backup`. It then erases the whole chip and writes `vehkernel.hex`, the boot stub and the firmware. Nothing of the vendor firmware, its settings or old parameters is left. The command takes about 2 minutes at 115200 baud. Nothing is erased if the backup fails or comes back short, and an existing backup file is never overwritten; `--no-backup` erases without one. Keep each board's backup outside the repository: vendor firmware differs from board to board.

```bash
tools/flash/flash_serial.py new-board
```

Every later update writes `vehkernel.bin` from the FIRMWARE region (S4) on, so the stored parameters survive. An update takes about 20 s:

```bash
tools/flash/flash_serial.py update
```

`backup <file>` only reads the chip, and `reset` restarts the firmware. An update on a board that still has its vendor firmware in S0 leaves that firmware running, so a new board needs `new-board` first. The tool fails with exit code 1 when `stm32flash` fails or a backup comes back short, and with 2 when an image does not fit its region, the build or `stm32flash` is missing, or the backup file already exists. The bootloader runs at 115200 baud unless given `--baud`.

## Rebooting into the bootloader

The [Link monitor](link-monitor.md) restarts a disarmed board over any Link with `reboot`, or into the bootloader with `reboot --bootloader`, ready for ArduPilot's `uploader.py` (ADR-0023; a pinned copy is in `tools/bootloader/third_party/`); both are refused while armed, and the board resets about 0.1 s after it replies. The request goes into RTC backup register 0 with ArduPilot's values (`RTC_BOOT_HOLD` for the bootloader, `RTC_BOOT_FAST` otherwise), or PX4's on a board that keeps PX4's bootloader. In SITL a reboot ends the run with exit status 10, or 11 for the bootloader (`sitl_reboot`).

```bash
tools/link/link_monitor.py --serial /dev/ttyACM0 reboot --bootloader
```
