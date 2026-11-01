# Examples

Board examples teach Vehkernel and its target hardware one component at a time ([ADR-0025](../docs/adr/0025-board-examples.md), [spec 0013](../docs/specs/0013-board-examples.md)). Each example is a standalone executable that exercises a single peripheral or subsystem, accompanied by a lesson in its `README.md`.

For building, flashing, SITL simulation and returning to the firmware, see the [Examples guide](../docs/guide/examples.md).

## Boards

- [C50C V1.1](c50c-v1_1/examples.toml)

### C50C V1.1 examples

The examples are numbered in dependency order:

| Example | Summary | SITL |
|---|---|---|
| [01-led](c50c-v1_1/01-led/README.md) | Blinks the on-board RGB LED with GPIO outputs and the system clock | No |
| [02-user-key](c50c-v1_1/02-user-key/README.md) | GPIO input with internal pull-up; S3 changes LED colour | No |
| [03-oled](c50c-v1_1/03-oled/README.md) | Draws a title and a counter on the OLED through software SPI and the SSD1306 driver | Yes |
| [04-work-queues](c50c-v1_1/04-work-queues/README.md) | Counts at 10 Hz and 2 Hz in two work items on one work queue and shows both on the OLED | Yes |
| [05-bus](c50c-v1_1/05-bus/README.md) | One work item publishes a counter that another, woken by it, draws on the OLED | Yes |
| `06-imu` *(pending)* | Software I2C ICM20948 IMU driver (`IMU0`) publishing `imu_sample` | Yes |
| `07-magnetometer` *(pending)* | AK09916 magnetometer via ICM20948 FIFO and `magnetometer_sample` | Yes |
| `08-serial-link` *(pending)* | Custom protocol Link over serial ports `SERIAL1` and `SERIAL2` | Yes |
| `09-parameters` *(pending)* | Parameter system and persistent storage in internal Flash `PARAMS` | Yes |
| `10-motor-outputs` *(pending)* | Actuator control outputs 1 to 4 with output watchdog and power sense | Yes |
| `11-wheel-encoders` *(pending)* | Quadrature encoders for motor outputs 1 to 4 and `wheel_encoders` | Yes |
| `12-usb-gamepad` *(pending)* | USB host on CN8 and XInput gamepad publishing `manual_control_input` | No |
