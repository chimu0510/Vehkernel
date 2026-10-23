# Third-party bootloader tools

`uploader.py` is ArduPilot's `Tools/scripts/uploader.py`, copied unchanged at
ArduPilot commit `f48fabc4261e25b1dc48a4a58bbf79966790921c` (2026-10-18). It
carries PX4's BSD 3-clause licence in its header. Vehkernel's tools and tests
flash boards with it (ADR-0016, ADR-0023); it needs pyserial, which
`pyproject.toml` pins.

To update it, copy the file again from a newer ArduPilot checkout, change the
commit above, and run the bootloader tests, which flash the SITL bootloader
with it.
