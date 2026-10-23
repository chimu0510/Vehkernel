# FreeRTOS as the RTOS

The firmware runs on FreeRTOS because it is the most stable, has the most resources and is the most mature on STM32F4. Multi-board support is therefore ours to build: the firmware builds its own board definitions and HAL (borrowing the toolchain idea of ArduPilot's hwdef and the layering idea of AP_HAL, see ADR-0009 and ADR-0011), and vehicle logic never calls the RTOS or the vendor HAL directly.

## Considered Options

- **Zephyr**: rejected. Its devicetree would solve board definitions, which FreeRTOS does not, but it is less stable and less mature on STM32F4, with fewer resources.
