# Support only the board revisions on hand, verify the revision at boot, allow alt configs per pin

The C50C has several hardware revisions, called V1.0, V1.1 and V1.2 by the vendor firmware; the vendor documents also use the names V4.0 to V4.2, and how the two naming schemes map onto each other is unconfirmed. Board definitions are written only for the revision on hand: C50C V1.1, with an ICM20948, a USB gamepad and no RS-485. Other revisions get their own board definitions when compatibility is needed. No shared base definition is extracted before a second revision exists.

**Boot verification**: one firmware image per revision. At boot the firmware checks that the IMU is an ICM20948 and that the revision-ID pins PE2, PE3 and PC13 read 0. If either check fails, actuators stay disabled and the error is reported over a Link and on the OLED.

**Alt configs**: a pin may serve different purposes under different frames. PB7, for example, is TIM4_CH2 for encoder C normally, but drives a servo on the high-end Ackermann car. A board definition may declare an alt config for a pin (like ArduPilot's `ALT()` with `BRD_ALT_CONFIG`), selected at boot by the frame parameters; the generator checks at build time that no combination produces a pin or timer conflict.

## Consequences

- The vendor resource allocation table lists PC13, PE2 and PE3 as unused, although they are the revision-ID pins, and its OLED pins do not reflect the V1.2 change. Board definitions are written from the schematic, the vendor firmware and bench measurements.
