# Timing as selectable dimensions, not a fixed scheme

Timing is not hard-coded into one scheme. It is split into three independently selectable dimensions, with defaults from a timing preset, because the right timing depends on the hardware. Popular firmware projects time their loops differently:

- Betaflight: gyro-driven, divided down with `pid_process_denom`.
- PX4: sensor publications wake the controllers.
- ArduPilot: a fixed-rate main loop set by `SCHED_LOOP_RATE` plus a scheduler table that records each task's rate, maximum run time and priority.

Each suits different hardware. The three dimensions:

1. **Trigger source**: `IMU_EVENT` (data-ready or FIFO watermark interrupt) or `TIMER` (fixed rate).
2. **Base rate + dividers**: `CTRL_RATE_HZ` is the base rate; estimation, angular rate control, and velocity and attitude control each set a divider. Timed work items set their own rates.
3. **Sensor sampling method**: per-sample reads or batched FIFO reads. Drivers declare their maximum ODR and the peripheral bus cost of each read; the board definition declares the peripheral bus type. With batched reads the driver still publishes each sample individually, so the IMU's per-sample topic must be declared with `@queue N`, N at least the largest batch (ADR-0008), and the estimator drains the queue on every run.

A **timing preset** groups values for the three dimensions. Its defaults follow the layering of ADR-0012: provided by the board definition, overridable by the frame preset.

At boot the firmware estimates the CPU budget from driver capabilities and peripheral bus costs; if a configuration is not feasible, it drops to the highest feasible rate and raises a warning. At runtime the timing monitor tracks each work item's run time, jitter and overruns and publishes them on the bus.

## Consequences

- C50C V1.1 is just one combination. The vendor firmware says SCL = PB9 and SDA = PB8, but on the F407 hardware I2C1 uses PB8 = SCL and PB9 = SDA, so the board may be wired the other way round. This is unverified, so the board defaults are provisional for two cases:
  - **Wiring confirmed swapped**: only software I2C works. One IMU read takes about 135 SCL cycles, about 0.34 ms at 400 kHz, with the CPU busy-waiting; the provisional board default is about 200 Hz.
  - **Wiring matches hardware I2C1**: switch to hardware I2C, optionally with DMA and no busy-waiting; the default rate is set by measurement.

  The final values live in the board TOML; this ADR only gives an example. A board with the IMU on hardware SPI likewise changes only its board defaults, not the architecture.
- The actual wiring is confirmed by a bring-up test: try both pin assignments with software I2C and read the ICM20948's WHO_AM_I, which must be `0xEA`.
