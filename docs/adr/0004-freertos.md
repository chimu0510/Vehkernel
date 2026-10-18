# FreeRTOS as the RTOS

FreeRTOS was chosen over Zephyr because it is the most stable, has the most resources and is the most mature on STM32F4. The cost is that multi-board support is ours to build: Zephyr's devicetree solves board definitions, FreeRTOS does not. The firmware therefore builds its own board definitions and HAL (borrowing the toolchain idea of ArduPilot's hwdef and the layering idea of AP_HAL, see ADR-0009 and ADR-0011), and vehicle logic never calls the RTOS or the vendor HAL directly.
