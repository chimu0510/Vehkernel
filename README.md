# ROS2-UGV

[![C50C Firmware](https://img.shields.io/badge/Firmware-STM32F407%20(C50C)-blue.svg)](#2-embedded-firmware-examples)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-green.svg)](#4-ros-2-workspace-ros2_ws)
[![Cross-Compilation](https://img.shields.io/badge/Cross--Compile-aarch64%20(Pi%205)-orange.svg)](#arm64-cross-compilation-target-deployment)
[![Architecture](https://img.shields.io/badge/ADR-0001--0004-purple.svg)](docs/adr/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

A modular, production-grade, and education-friendly autonomous Ground Vehicle (UGV) stack designed for WHEELTEC differential Drive-Steer (BS) mobile robots. The platform pairs a low-level **C50C Control Board** (STM32F407VET6) with an onboard **Raspberry Pi 5** single-board computer running **ROS 2 Jazzy**.

The project serves both as an autonomous robot development foundation and as an educational reference, emphasizing decoupled architectures, rigorous unit testing, deterministic failsafe behaviors, and clean domain abstractions.

---

## System Highlights

* **Unified Domain Abstraction**: Strict single-source terminology adhering to [`CONTEXT.md`](CONTEXT.md) and Architecture Decision Records ([`docs/adr/`](docs/adr/)).
* **Software-Defined Heading ([ADR-0001](docs/adr/0001-heading-defined-in-software.md))**: Swap the physical front between the pneumatic *Drive End* and passive *Omni End* via Flash-persisted settings without rewiring or altering kinematic sign conventions.
* **MCU-Level Kinematics & Yaw-Rate Stabilization ([ADR-0002](docs/adr/0002-kinematics-and-yaw-loop-on-mcu.md))**: Exchange only planar body velocity ($V_x, \omega_z$ in `base_link`) between the SBC and MCU; the MCU handles wheel kinematics and runs a fast 100 Hz $\omega_z$ closed loop against gyro feedback to eliminate track drift.
* **Deterministic Framing & Protocol Codegen ([ADR-0003](docs/adr/0003-custom-serial-protocol.md))**: COBS byte-stuffing and CRC16-CCITT packetization driven by a single [`protocol_def.yaml`](protocol/protocol_def.yaml), generating identical C, C++, and Python bindings.
* **Deterministic Arbitration & Ramp Failsafe ([ADR-0004](docs/adr/0004-control-source-arbitration-and-failsafe.md))**: Fixed hardware safety hierarchy (`SBUS Remote > USB Gamepad > Autonomous ROS 2`). Smooth ramp deceleration prevents tip-over upon connection loss or low battery (< 20.0 V).
* **Attitude Filter Workbench ([`tools/ahrs_bench`](tools/ahrs_bench/))**: Full-stack web-based telemetry and real-time tuning UI comparing Mahony vs. Error-State Kalman Filter (ES-EKF) both hardware-in-the-loop and offline simulated.

---

## Repository Structure

```text
.
├── CONTEXT.md               # Unified project vocabulary and domain dictionary
├── AGENTS.md                # AI agent operations, skills, and coding workflows
├── docs/                    # Architecture Decision Records (ADRs) and design specs
│   └── adr/                 # ADR-0001 to ADR-0004
├── protocol/                # Cross-platform binary communication protocol
│   ├── protocol_def.yaml    # Protocol schema definition
│   ├── codegen.py           # Code generator for C, C++, and Python
│   ├── include/ugv_protocol/# COBS and CRC16 core frame codec
│   └── tests/               # Decoupled protocol unit tests
├── examples/                # C50C STM32F407 embedded firmware suite
│   ├── common/              # Board pin definitions (board.h), drivers, and math
│   ├── bringup/             # Layered unit bring-up routines (Layer 0 to 4)
│   └── demos/               # Integrated firmware (chassis_ctrl, ahrs_bench, etc.)
├── ros2_ws/                 # ROS 2 Jazzy workspace
│   ├── src/                 # Onboard target packages (ugv_base, ugv_bringup, ugv_navigation)
│   ├── host_tools/          # Development host tools (e.g. ugv_model_viewer)
│   └── scripts/             # aarch64 cross-compilation pipeline with Docker & QEMU
├── tools/                   # Developer tooling & utilities
│   ├── c50c.py              # CLI flasher (UART/SWD) and serial monitor
│   ├── calibrate_chassis.py # Kinematics and gyro yaw-rate calibration utility
│   ├── ahrs_bench/          # Web workbench for IMU/EKF filter analysis
│   └── setup.sh             # Dependency installer and environment doctor
└── third_party/             # Pinned submodules (STM32F4 HAL, CMSIS)
```

---

## Quick Start

### 1. Environment Setup

Run the automated doctor script to configure build dependencies (ARM GCC toolchain, CMake, Ninja, Python packages, and udev rules):

```bash
# Check dependencies and install required tools
./tools/setup.sh
```

Ensure git submodules for CMSIS and STM32F4 HAL are initialized:

```bash
git submodule update --init --depth 1
```

---

### 2. Embedded Firmware (`examples/`)

The C50C firmware is organized into a modular bring-up hierarchy (`examples/bringup/`) and full-system production programs (`examples/demos/`).

#### Build All Targets

```bash
cmake -S examples -B examples/build -G Ninja
ninja -C examples/build
```

#### Flash to C50C Control Board

Use the [`tools/c50c.py`](tools/c50c.py) utility to flash via USB Type-C (UART1 with automatic DTR/RTS bootloader toggle) or SWD:

```bash
# Flash bring-up verification example via UART
./tools/c50c.py flash 01_board_info

# Flash production chassis control firmware
./tools/c50c.py flash chassis_ctrl

# Launch serial console monitor with auto-reset
./tools/c50c.py monitor --reset
```

#### Bring-up Verification Layers

| Layer | Example | Description |
| :--- | :--- | :--- |
| **0 Core** | `01_board_info`, `02_uart` | Clock tree, HSE 8MHz, hardware ID, USART1/3 loopback |
| **1 Basic I/O** | `11_led`, `12_key_buzzer`, `13_flash_params` | RGB LED, user button, buzzer, internal Flash sector 7 storage |
| **2 Peripherals** | `21_oled`, `22_imu`, `23_battery_adc` | SSD1306 OLED, ICM-20948 9-DOF IMU, battery voltage monitoring |
| **3 Actuation** | `31_encoder`, `32_motor` | 4-channel quadrature encoders, dual DC motor drive & polarity |
| **4 Interface** | `41_sbus`, `42_can` | SBUS 100kbit/s receiver frame rates, CAN loopback |

---

### 3. Binary Protocol & Unit Tests

The communication protocol between the MCU and SBC uses consistent overhead byte stuffing (COBS) and CRC16-CCITT.

#### Generate Bindings

```bash
./protocol/codegen.py
```

#### Run Unit Tests

All unit tests are fully decoupled and can be executed independently on any development host:

```bash
# Python protocol codec and arbitration test suite
python3 -m unittest protocol/tests/test_protocol.py

# Host C/C++ unit test suite (CTest)
cmake -S protocol -B protocol/build
cmake --build protocol/build
ctest --test-dir protocol/build --output-on-failure
```

---

### 4. ROS 2 Workspace (`ros2_ws/`)

#### Native Host Build (Simulation & Visual Tools)

```bash
cd ros2_ws
colcon build --base-paths host_tools
source install/setup.bash

# Run chassis 3D model viewer and heading configurator
ros2 launch ugv_model_viewer view_model.launch.py
```

#### ARM64 Cross-Compilation (Target Deployment)

For deploying onto a Raspberry Pi 5 without compiling on the robot:

```bash
# Initialize Docker/QEMU build container and Sysroot (one-time setup)
./ros2_ws/scripts/setup_qemu_environment.sh

# Cross-compile for ARM64 and automatically deploy via SSH/SCP
./ros2_ws/scripts/cross_compile.sh dest 22 ubuntu@192.168.1.100:/home/ubuntu/ugv_ws
```

---

### 5. AHRS Bench (`tools/ahrs_bench/`)

Evaluate sensor fusion performance, tune Mahony and ES-EKF parameters, and compare attitude estimates in real time:

```bash
# Flash the ahrs_bench firmware to C50C
./tools/c50c.py flash ahrs_bench

# Start the web server (defaults to http://localhost:8008)
./tools/ahrs_bench/server.py

# Or run in purely synthetic simulation mode without hardware
./tools/ahrs_bench/server.py --sim drive
```

---

## Domain Vocabulary

To ensure engineering consistency across firmware, communications, and ROS 2 layers, please follow the terms defined in [`CONTEXT.md`](CONTEXT.md):

* **Drive End**: The physical end with driving wheels and pneumatic tires.
* **Omni End**: The physical end with passive omnidirectional wheels.
* **Software-Defined Heading**: The forward direction set by software (`Drive End Forward` or `Omni End Forward`).
* **Body Velocity**: Linear velocity $V_x$ and angular velocity $\omega_z$ expressed in `base_link`.
* **Failsafe**: Ramp deceleration to zero velocity followed by active motor disarm upon signal timeout or low battery.

---

## Contributing & Development Workflow

1. Always consult [`CONTEXT.md`](CONTEXT.md) and [`AGENTS.md`](AGENTS.md) before refactoring or naming new modules.
2. Maintain high test decoupling: host unit tests must not depend on physical hardware state.
3. Keep firmware additions modular by registering units in `examples/bringup/` or full applications in `examples/demos/`.

---

## License

This project is free software: you can redistribute it and/or modify it under the terms of the [GNU General Public License](https://www.gnu.org/licenses/gpl-3.0) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

See the [LICENSE](LICENSE) file for the full license text.

Third-party vendor libraries and submodules:
* **STM32F4 HAL & CMSIS** (`third_party/`): STMicroelectronics Apache-2.0 / BSD-3-Clause.
* **ROS 2 Workspace Packages** (`ros2_ws/`): GPL-3.0-or-later.