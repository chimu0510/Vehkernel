# Boards are added one at a time with their own ADR and spec, verify their revision at boot, and allow alt configs per pin

Vehkernel targets the boards ArduPilot and PX4 run on, from flight controllers such as the Pixhawk 6C to robot controllers such as the C50C. The framework's rules follow those projects and are not bound to one board's resources. A board is added only when one is on hand: it gets its board definition (ADR-0011), an ADR recording what it needs from the framework, and a spec for its bring-up. Each board revision gets its own board definition. No shared base definition is extracted before a second revision of the same board exists.

**Adding a board**

- The board's ADR records the facts the framework's rules leave to each board: its MCU family and what the platform layer needs for it, its Flash layout and where parameters are stored (ADR-0020), whether it keeps an ArduPilot or PX4 bootloader and its board ID (ADR-0023), and how its revision is verified at boot.
- Its spec covers the board definition, the platform work and the hardware checks (ADR-0018 L4).
- The C50C V1.1, the first board, came before this rule; its ADR, ADR-0024, was written after its bring-up.

**Boot verification**: one firmware image per board revision. At boot the firmware checks what the board definition says it can: that the IMU answers as the declared device and, on boards that have them, that the revision-ID pins read the declared value. If a check fails, actuators stay disabled and the error is reported over a Link and on the local display. ADR-0024 gives the C50C V1.1's checks.

**Alt configs**: a pin may serve different purposes under different frames. On the C50C, for example, PB7 is TIM4_CH2 for encoder C normally, but drives a servo on the high-end Ackermann car. A board definition may declare an alt config for a pin (like ArduPilot's `ALT()` with `BRD_ALT_CONFIG`), selected at boot by the frame parameters; the generator checks at build time that no combination produces a pin or timer conflict.
