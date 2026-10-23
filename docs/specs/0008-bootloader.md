# Spec 0008: the bootloader

**Status**: in progress. Issue: [#61](https://github.com/chimu0510/Vehkernel/issues/61); tickets #62 to #67, with the hardware check in #67.

## Problem Statement

The seventh milestone left Vehkernel on the C50C V1.1 with a local display and menu, and two Links: SERIAL1 on the Type-C port and SERIAL2 on the one-key download port. Firmware reaches the board in two ways. SWD needs a debug probe. `tools/flash/flash_serial.py` drives the STM32 system bootloader through the download port's DTR/RTS circuit. S0 holds only a stub that jumps to the firmware.

Neither way checks what it writes. Firmware built for another board, or an image cut short by a power loss, is started as if it were good. Neither way works through the tools that flash ArduPilot and PX4 boards, and neither works on a board without the C50C's download circuit. ADR-0023 decides to join the ArduPilot and PX4 ecosystem: their bootloader protocol, their firmware file and app descriptor, and the board IDs they share. Boards that ship with their bootloader keep it, and boards without one, such as the C50C, get a Vehkernel bootloader that speaks the same protocol. None of it exists.

This milestone must not depend on, or change, anything the open hardware check of spec 0006 (#50) may still change: the actuator platform, the `MOT` parameters and the board's actuator facts.

## Solution

Deliver the eighth milestone: **a Vehkernel bootloader on the C50C V1.1 that ArduPilot's `uploader.py` flashes, and firmware that carries ArduPilot's app descriptor and asks to be rebooted into the bootloader over a Link**. Afterwards:

- The board definition gives the board ID and says which bootloader the board has: the Vehkernel bootloader, with the ports it listens on, or a kept ArduPilot or PX4 bootloader. The C50C V1.1 has board ID 22083 and the Vehkernel bootloader on SERIAL2 and SERIAL1.
- Every firmware image carries an app descriptor in ArduPilot's unsigned layout, filled in after linking. The build writes `vehkernel.apj`, and for a board with the Vehkernel bootloader also `vehkernel_with_bl.hex`.
- The Vehkernel bootloader sits in the BOOT region and speaks the bootloader protocol at revision 5 on the board's listed ports. It erases only the FIRMWARE region. It starts the firmware only when the app descriptor's CRCs, size and board ID match.
- A Link request reboots the board, or reboots it into the bootloader, while disarmed. The firmware leaves the request where the board's bootloader looks for it.
- In SITL, the same bootloader core runs over a pty against simulated Flash, and CI flashes it with a pinned copy of ArduPilot's `uploader.py`.
- On the C50C V1.1, `flash_serial.py new-board` installs the bootloader and firmware once. Later updates go through the bootloader with `uploader.py`, and stored parameters survive them.

## User Stories

1. As a C50C user, I want to update the firmware with ArduPilot's `uploader.py` and a `.apj` file, so that Vehkernel boards use the same tool as ArduPilot and PX4 boards (ADR-0023).
2. As a C50C user, I want an update to keep my stored parameters, so that flashing never resets my settings (ADR-0012, ADR-0020).
3. As a C50C user, I want firmware built for another board to be refused before anything is erased, so that a wrong file cannot leave my board without firmware.
4. As a C50C user, I want a board whose update was cut short to stay in the bootloader, so that I can flash it again without a debug probe.
5. As a C50C user, I want a Link command that reboots the board into the bootloader, so that I can update it without power-cycling it or knowing the DTR/RTS sequence.
6. As a C50C user, I want that command refused while armed, so that an update can never start under a moving vehicle (ADR-0005).
7. As a C50C user, I want a plain reboot command too, so that `reboot_required` parameters take effect without a power cycle.
8. As a C50C user, I want a short window at every boot in which the bootloader accepts `uploader.py`, so that a board whose firmware cannot run is still flashed by power-cycling it.
9. As a C50C user, I want one command that installs the bootloader and firmware on a new board after backing up the vendor firmware, so that converting a board is one step.
10. As a C50C user, I want the bootloader on both Link ports, so that either cable updates the board.
11. As a board author, I want to declare the board ID and the bootloader in the board definition, checked by the board generator, so that both are hardware facts in one file (ADR-0011).
12. As a board author, I want the generator to refuse a board ID that a board in ArduPilot's list uses, for a board with the Vehkernel bootloader, so that its firmware is never accepted by a board ArduPilot or PX4 supports (ADR-0023).
13. As a board author, I want the generator to require a listed board ID for a board that keeps an ArduPilot or PX4 bootloader, so that the ground station tools know it.
14. As a board author, I want to declare a port's DTR/RTS download circuit in the board definition, so that the serial flasher takes the sequence from the board, not from its own code.
15. As a firmware developer, I want the bootloader's protocol and boot decisions tested on the development host, so that compatibility is checked in CI, not on a bench (ADR-0018).
16. As a firmware developer, I want CI to flash the SITL bootloader with ArduPilot's `uploader.py`, so that compatibility with the real tool is tested, not assumed.
17. As a firmware developer, I want the bootloader to fit the 16 KB BOOT region with its size checked at every build, so that it never grows into the parameter sectors (ADR-0017, ADR-0020).
18. As a firmware developer, I want a script that lists the upstream changes to `AP_Bootloader` and `AP_CheckFirmware` since the commit the port follows, so that the port can follow protocol changes (ADR-0023).
19. As a firmware developer, I want the bring-up image to carry an app descriptor too, so that the bootloader starts it after the bring-up runner flashes it.
20. As a C50C user, I want a documented hardware check of the bootloader, so that installation, updates, refusals and recovery are confirmed on the board (ADR-0018 L4).

## Implementation Decisions

- **Board definition** (ADR-0011, ADR-0023):
  - A top-level `board_id`, an integer from 1 to 65535, and a `bootloader` table, both required on a board with a Flash layout, since it builds a firmware image (ADR-0020); a board definition without one, such as a generator test case, may leave them out.
  - `bootloader.kind` is `VEHKERNEL`, `ARDUPILOT` or `PX4`. With `VEHKERNEL`, `ports` lists the logical serial ports it listens on in order, like ArduPilot's `SERIAL_ORDER`, and an optional `stay = { pin = "<role>", active = "LOW" | "HIGH" }` names a `GPIO_INPUT` role that keeps the board in the bootloader while it reads that level.
  - A serial port entry gains an optional `download = "DTR_RTS"` for the C50C's circuit: RTS asserted holds BOOT0 high, and DTR released with RTS asserted holds NRST low.
  - A copy of ArduPilot's `Tools/AP_Bootloader/board_types.txt` is committed with the upstream commit it came from (ADR-0016). The generator rejects a `VEHKERNEL` board whose ID is in it, lies in a range its comments reserve for a vendor, or lies in the range 1000 to 19999 ArduPilot allocates; an `ARDUPILOT` or `PX4` board whose ID is not in it; and two board definitions in sibling directories with the same ID.
  - The generator emits the board ID, the bootloader kind and ports, the stay input and the download circuit in `board.hpp`, `board.json` and Kconfig (`BOARD_BOOTLOADER_VEHKERNEL`, `BOARD_BOOTLOADER_ARDUPILOT`, `BOARD_BOOTLOADER_PX4`).
  - The C50C V1.1 declares `board_id = 22083` (0x5643, outside ArduPilot's managed range 1000 to 19999 and unused), `bootloader = { kind = "VEHKERNEL", ports = ["SERIAL2", "SERIAL1"] }` and `download = "DTR_RTS"` on SERIAL2. Nothing about its actuators changes.
- **App descriptor**:
  - ArduPilot's unsigned layout (`AP_CheckFirmware.h`, 36 bytes): signature `40 a2 e4 f1 64 68 91 06`, `image_crc1`, `image_crc2`, `image_size`, `git_hash`, `version_major`, `version_minor`, `board_id` and 8 reserved bytes.
  - It sits in a `.app_descriptor` section right after the vector table, as ArduPilot's linker scripts place it.
  - A post-link step, like ArduPilot's `set_app_descriptor`, fills it in the binary and the ELF: the CRCs with ArduPilot's `crc32_small` over the image before `image_crc1` and from `version_major` to the end, the size, the first 32 bits of the git hash and the board ID. The firmware version comes from `project(VERSION)` in CMake, starting at 0.1.
  - The bring-up image carries one too.
- **Firmware files**: `vehkernel.apj` follows ArduPilot's JSON (`chibios.py`): `board_id`, `magic`, `description`, `image` (zlib, base64), `extf_image`, `summary`, `version`, `image_size`, `extf_image_size`, `flash_total`, `image_maxsize` and `flash_free` (from the FIRMWARE region), `extflash_total`, `extflash_free`, `git_identity` and `board_revision`. Like ArduPilot's default build it leaves out `build_time`, so one commit always gives the same file, and `USBID`, which a board without MCU USB has none of. A `VEHKERNEL` board also gets `vehkernel_with_bl.hex`: the bootloader and the firmware.
- **Reboot service** (ADR-0023, ADR-0002):
  - A protocol-independent request with two actions, reboot and reboot to the bootloader, refused while armed with the reason in the reply.
  - The custom protocol carries it as a control message. This raises the protocol to version 4. MAVLink's mapping comes with MAVLink.
  - Before resetting, the platform writes the board's bootloader value to RTC backup register 0, where that bootloader reads it: ArduPilot's `RTC_BOOT_HOLD` (0xb0070001) for `VEHKERNEL` and `ARDUPILOT`, PX4's `BOOT_RTC_SIGNATURE` (0xb007b007) for `PX4`. A plain reboot writes ArduPilot's `RTC_BOOT_FAST` (0xb0070002), as ArduPilot does, so the bootloader starts the firmware without its boot window, or clears the register for `PX4`.
  - The reset happens 0.1 s after the request is accepted, from the commander's cycle, so the reply leaves first, as ArduPilot acknowledges before rebooting. Arming is refused meanwhile with `REBOOTING`. An unknown action is refused by the Link without asking the commander, as an unknown mode is.
  - The Link monitor gains `reboot` and `reboot --bootloader`.
  - In SITL, a reboot ends the process with an exit code that tells the two actions apart.
- **Vehkernel bootloader, core**:
  - A port of `AP_Bootloader`'s structure (`bl_protocol.cpp`, `AP_Bootloader.cpp`, `AP_CheckFirmware`) in the C++20 subset, without ChibiOS, FreeRTOS or ArduPilot.
  - It is split into protocol handling, the boot decision and the descriptor check. Each works through a narrow platform interface modelled on `AP_Bootloader`'s `support.h`: byte ports with a timeout, sector erase and word write within the FIRMWARE region, the RTC backup value, milliseconds, the MCU ID, description and serial number, and an optional LED.
  - Protocol revision 5 commands: `GET_SYNC`, `GET_DEVICE` (bootloader revision, board ID, board revision 0, firmware size), `CHIP_ERASE` (the FIRMWARE region), `PROG_MULTI`, `READ_MULTI`, `GET_CRC`, `GET_SN`, `GET_CHIP`, `GET_CHIP_DES`, `SET_BAUD` and `BOOT`. The external Flash commands answer `INVALID`, as ArduPilot's bootloader does on a board without external Flash. Erasing needs `GET_SYNC` and `GET_DEVICE` first, as in ArduPilot.
  - The boot decision follows ArduPilot. With `RTC_BOOT_HOLD` set, or the stay input active, it stays until flashed or reset. With `RTC_BOOT_FAST` set by a plain reboot and a valid firmware, it starts the firmware at once. Otherwise, with a valid firmware it listens for 1 s (a constant) and then starts it, and with an invalid one it stays. It clears the register.
  - It records the upstream ArduPilot commit it follows: `f48fabc` (2026-10-18). `tools/bootloader/upstream_diff.py` lists the upstream changes to `Tools/AP_Bootloader` and `libraries/AP_CheckFirmware` since that commit, given an ArduPilot checkout.
- **Vehkernel bootloader, STM32F4**:
  - A separate executable built by every preset of a `VEHKERNEL` board and linked into the BOOT region, whose size the link enforces (ADR-0017).
  - Bare metal: the clock tree from the board definition, polled UARTs on the listed ports at 115200 8N1, as ArduPilot's `BOOTLOADER_BAUDRATE` and `uploader.py`, until `SET_BAUD`, SysTick milliseconds, and Flash through the platform's internal Flash code (ADR-0020).
  - It starts the firmware by setting VTOR, the stack pointer and the reset handler, as the boot stub does today, and replaces the stub.
- **Serial flasher** (`tools/flash/flash_serial.py`):
  - It takes the DTR/RTS sequence from the board definition's `download`.
  - `new-board` writes `vehkernel_with_bl.hex` after the backup and the erase.
  - `update` reboots the board into the bootloader over its Link, or through the download circuit when no Link answers, then runs `uploader.py` with `vehkernel.apj`.
  - `backup` and `reset` stay.
- **Pinned `uploader.py`**: ArduPilot's `Tools/scripts/uploader.py` at the same upstream commit is committed under `tools/bootloader/third_party/` with its licence (ADR-0016), and the tools and tests use it.

## Testing Decisions

- **Good tests check external behaviour only**: what the bootloader answers on its port, what it leaves in the simulated Flash, which image it starts, what `uploader.py` reports, what a Link reboot request returns, and what the generators emit or reject.
- **Seam A: SITL (L3), the main seam.** `test/sitl/check_bootloader.py` runs the SITL bootloader over a pty against simulated Flash, and checks with the pinned `uploader.py`, in CI:
  1. A SITL firmware `.apj` with the C50C's board ID flashes, verifies and boots, and the bootloader reports the image started.
  2. A `.apj` with another board ID is refused before anything is erased.
  3. With the simulated Flash holding stored parameter sectors, an update leaves them unchanged.
  4. A flash cut off midway leaves an invalid image. At the next start the bootloader stays and accepts a new flash.
  5. With `RTC_BOOT_HOLD` set it stays. With a valid image and no request, it starts the image after the boot window.
  A SITL firmware run asked to reboot into the bootloader over its Link exits with the bootloader code, and is refused while armed. The existing SITL checks keep passing.
- **Seam B: bootloader core (L1).** Each command and its replies against fake ports and Flash. Erase is refused without `GET_SYNC` and `GET_DEVICE`. Writes outside the FIRMWARE region are refused. `GET_CRC` matches the image. The boot decision covers each case at the 1 s boundary. The descriptor check covers each failure: no signature, bad size, bad CRC, wrong board ID.
- **Seam C: app descriptor and files (L1, pytest).** The post-link step on a small test image gives the CRCs `uploader.py`'s `crc32` and ArduPilot's descriptor check expect. The `.apj` has every field and decodes to the image. `vehkernel_with_bl.hex` holds both images at their addresses.
- **Seam D: reboot service (L1).** The commander or the service refuses while armed and accepts while disarmed. The platform writes each bootloader kind's value, and clears it for a plain reboot.
- **Seam E: generators (L0 and L1).** Golden output for the C50C with `board_id`, `bootloader` and `download`, and a diagnostic each for: a missing `board_id`, an ID out of range, a `VEHKERNEL` ID in `board_types.txt`, an `ARDUPILOT` ID not in it, a duplicate ID, an unknown `kind`, `ports` naming an unknown port, `stay` on an output pin, and `download` with an unknown value.
- **Seam F: serial flasher (pytest).** With fakes for `stm32flash`, `uploader.py` and the Link: `new-board` writes `vehkernel_with_bl.hex`; `update` asks for the bootloader over the Link and runs `uploader.py` with the `.apj`, and falls back to the download circuit when the Link does not answer; the sequence comes from `board.json`.
- **Seam G: hardware (L4, manual).** On the C50C V1.1, wheels not involved:
  - `new-board` installs the bootloader and firmware, and the board verifies.
  - `update` over SERIAL2 and over SERIAL1 keeps a changed parameter.
  - `reboot --bootloader` is refused while armed.
  - A `.apj` with another board ID is refused.
  - Pulling the cable mid-update leaves the board in the bootloader, and a second `update` recovers it.
  - A power cycle with `uploader.py` waiting flashes the board through the boot window.
  - The bootloader adds about 1 s to boot, measured.
  - The BOOT region usage is reported.
  Results are recorded on the ticket.
- **Prior art**: `test/sitl/check_link.py` for SITL over a pty; the simulated NOR Flash in `sim/` and its power-loss tests (ADR-0018); the board generator's golden and diagnostic cases; `tools/flash/tests` for faked tools.

## Out of Scope

- USB CDC in the Vehkernel bootloader: the C50C has no MCU USB. It comes with the first board that needs it.
- MAVLink's mapping of the reboot service, which comes with the MAVLink protocol (ADR-0002).
- Boards that keep an ArduPilot or PX4 bootloader beyond what the generator checks: each comes with its own ADR and spec (ADR-0010).
- Signed firmware, DroneCAN updates, Ethernet, SD card and external Flash (ADR-0023).
- Jumping from the firmware into the STM32 system bootloader.
- Registering a board ID with ArduPilot.
- Logging (ADR-0019), which spec 0007 named as spec 0008 and moves to a later spec.

## Further Notes

- **Why board ID 22083**: ArduPilot manages 1000 to 19999 and asks that IDs there be allocated only through its list. Other vendors use larger numbers, such as ModalAI's 41775. 22083 (0x5643) is outside that range and in no entry of the list at commit `f48fabc`.
- **Why 1 s**: on a board without USB power sensing, ArduPilot's bootloader waits `HAL_BOOTLOADER_TIMEOUT`, 5 s by default, before starting a good firmware; its CAN bootloader waits 1 s. Vehkernel waits 1 s to keep the boot short. `uploader.py` keeps sending `GET_SYNC`, so a power cycle while it waits is caught, and a reboot request holds the bootloader without any window.
- **Why the SITL bootloader matters**: the hardware check runs once, but CI runs `uploader.py` against the port on every push, so a protocol difference is caught where it is made.
- **Protocol version 4**: the reboot request is a new control message, so the custom protocol's version rises (ADR-0002). Host tools come from the same build.
