---
status: proposed
---

# Bootloader: the ArduPilot and PX4 protocol, kept where a board has it, ported where it does not

Flight controllers that ArduPilot and PX4 support ship with a bootloader that speaks one protocol: PX4 wrote it, and ArduPilot's `AP_Bootloader` keeps it compatible and shares its list of board IDs. QGroundControl and Mission Planner flash boards through it over USB, and ArduPilot's `uploader.py` and PX4's `px4_uploader.py` over USB or a serial port. Vehkernel is not limited to its first development board, so it joins this ecosystem rather than defining its own. Boards that already have such a bootloader keep it. Boards without one, such as the C50C, get a Vehkernel bootloader that speaks the same protocol. The STM32 system bootloader in ROM is kept only to install that bootloader and to rescue a board.

**Protocol and firmware file**

- The bootloader protocol is the one in PX4's `bl.c` and ArduPilot's `bl_protocol.cpp`, at protocol revision 5. It erases only the firmware region, so stored parameters survive an update (ADR-0012, ADR-0020).
- Every firmware image carries an **app descriptor** in ArduPilot's layout (`AP_CheckFirmware`): CRCs of the image, its size, the git hash, the version and the board ID. ArduPilot's bootloader and the Vehkernel bootloader check it before starting the firmware; PX4's bootloader does not.
- A build also produces an `.apj` file, the JSON firmware file ArduPilot's tools read: the board ID, the compressed image, its size, the largest image the board takes and the git identity. A board that uses the Vehkernel bootloader also gets a `_with_bl.hex` holding the bootloader and the firmware, for its first installation.

**Board ID**

- The board definition declares the board ID (ADR-0011). The board generator rejects two board definitions with the same ID.
- A board that ArduPilot or PX4 already supports uses the ID its existing firmware uses, for example 56 for the Pixhawk 6C and 1166 for the MicoAir743, so the ground station tools know it.
- A development board, such as the C50C, picks an ID that is not registered with ArduPilot and is not registered later. A board sold to users would be registered in ArduPilot's `board_types.txt`; none is planned.

**Boards with an ArduPilot or PX4 bootloader**

- The bootloader stays. Vehkernel never overwrites it, so the board can go back to ArduPilot or PX4 at any time.
- The board definition says whether it is ArduPilot's or PX4's, since they differ in how the firmware asks them to stay in the bootloader.
- The board definition declares the Flash layout that bootloader expects: the size of its region, where the firmware starts and where parameters are stored, for example a 128 KB bootloader region on the STM32H743 boards. The firmware links at that start address and carries the app descriptor.

**Boards without one: the Vehkernel bootloader**

- It is a port of `AP_Bootloader`'s structure in Vehkernel's style: protocol handling, boot sequence and app descriptor check above a small platform interface (`AP_Bootloader`'s `support.h`), written in the C++20 subset (ADR-0007) on Vehkernel's platform layer (ADR-0009). It needs neither ArduPilot nor ChibiOS to build, and runs without FreeRTOS.
- The board generator configures it from the board definition: the ports it listens on, the firmware start, the board ID, its LED and an optional stay-in-bootloader input. No second file describes the board.
- It fits the BOOT region of ADR-0020. On the C50C that is S0, 16 KB, as ArduPilot's bootloaders on STM32F405 boards do.
- The port records the upstream ArduPilot commit it follows. A script lists the upstream changes to `Tools/AP_Bootloader` and `libraries/AP_CheckFirmware` since that commit, and the Vehkernel bootloader adopts those that change the protocol. CI flashes the bootloader running in SITL with a pinned `uploader.py` (ADR-0018 L3), so compatibility is tested, not assumed.

**Interfaces**

- The protocol runs on UART and on USB CDC. The board definition lists the ports the Vehkernel bootloader listens on, like ArduPilot's `SERIAL_ORDER`; a board without MCU USB, such as the C50C, lists its UARTs only.
- QGroundControl and Mission Planner find a board by its USB IDs, so a board without MCU USB is flashed with `uploader.py` or `px4_uploader.py` given its serial port.
- DroneCAN firmware update, Ethernet and SD card updates, which `AP_Bootloader` also offers, are left to later decisions.

**Entering the bootloader**

All of these are refused or ignored while armed (ADR-0005).

- **Reboot service.** One protocol-independent service reboots the board, or reboots it into the bootloader, as the parameter service does for parameters (ADR-0012). The custom protocol carries it as control messages (ADR-0002). MAVLink maps it to `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` with `param1` 1 or 3, as ArduPilot and PX4 do. The firmware leaves the request in an RTC backup register, where the board's bootloader looks for it: ArduPilot's `RTC_BOOT_HOLD` value, which the Vehkernel bootloader also uses, or PX4's `BOOT_RTC_SIGNATURE` on a board that keeps PX4's bootloader.
- **Boot window.** As in ArduPilot, the bootloader waits briefly at every boot before starting a valid firmware, and stays when the firmware is invalid. Replugging USB or power-cycling the board is enough to flash it, even when the firmware cannot run.
- **Board inputs.** A board may declare a hardware way in: a stay-in-bootloader input, as ArduPilot's `HAL_GPIO_PIN_STAY_IN_BOOTLOADER`, or a download circuit such as the C50C's, where the adapter's RTS drives BOOT0 and its DTR drives NRST.
- **System bootloader.** The firmware can restart into the STM32 system bootloader, as ArduPilot's `boot_to_dfu`, to rescue a board whose bootloader is damaged.

**Validation at boot**

The bootloader starts the firmware only when its app descriptor is present and its CRCs, size and board ID match. Otherwise it stays in the bootloader, so a flash interrupted by a power loss, or firmware for another board, never runs. Signed firmware, ArduPilot's signed app descriptor, is left to a later decision.

**The STM32 system bootloader**

It installs the bootloader and firmware (`_with_bl.hex`) on a new board, after backing up the vendor firmware, and rescues a board whose bootloader is damaged; SWD does the same with a debug probe. Firmware updates go through the bootloader of this decision, not through the system bootloader.

## Considered Options

- **A protocol of Vehkernel's own, integrated with the Link**: rejected. Every tool that flashes ArduPilot and PX4 boards would stop working, and boards that ship with their bootloader would have to have it replaced.
- **The protocol with a firmware file and descriptor of Vehkernel's own**: rejected. The tools would connect, then refuse the image or flash it unchecked.
- **Replacing the bootloader on boards that already have one**: rejected. Each board would need DFU or SWD before its first update, and would lose the way back to ArduPilot or PX4.
- **Building `AP_Bootloader` from the ArduPilot tree with a `hwdef-bl.dat` for each board**: rejected. The board's pins, clock and ports would live in a second file beside the board definition (ADR-0011), and the build would need ArduPilot and ChibiOS.
- **Copying `AP_Bootloader`'s protocol files unchanged, with a shim for ChibiOS**: rejected. It would keep upstream diffs small, but put a second style and an RTOS shim into the platform layer. Following upstream with a diff script and testing with `uploader.py` gives the same compatibility.
- **The STM32 system bootloader for every update**: rejected. It cannot check the board or the image, listens only on the ports the ROM supports, and is entered through BOOT0, which most boards cannot drive from the host.
- **Registering board IDs with ArduPilot now**: not adopted. The boards in use either have IDs already or are development boards.
- **Signed firmware now**: deferred. The app descriptor leaves room for ArduPilot's signed layout.
