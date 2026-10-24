# Testing

Tests are split into six layers, L0 to L5 (ADR-0018). The directories under `test/` follow them: `unit/` (L1), `replay/` (L2), `sitl/` (L3), `hardware/` (L4) and `ros2/` (L5); L0 static checks run from the root config files and CI.

## Host tests and SITL tests

Unit tests and SITL tests run through `ctest` in the devcontainer:

```bash
cmake --preset test
cmake --build --preset test
ctest --preset test
```

The SITL tests (`sitl_*`) start the SITL executable and check it from the development host with the scripts in `test/sitl/`; each guide names the tests that cover it ([SITL](sitl.md), [Link monitor](link-monitor.md), [Driving the wheels](driving.md), [Local display](local-display.md)). SITL tests run serially under `ctest -j`, since they judge timing on the shared host. CI builds every preset and runs the tests inside the devcontainer image.

## Bring-up image

The bring-up test build (L4) runs the hardware bring-up cases at boot instead of the application: it reads the ICM20948's WHO_AM_I over software I2C and measures the IMU read rate. It never configures actuator outputs, so it is safe with motors connected. Build it in the devcontainer, then run the bring-up runner on the host, which flashes it over SWD, reads the result block back and exits non-zero if a case fails or no results arrive:

```bash
cmake --preset c50c-v1_1-bringup
cmake --build --preset c50c-v1_1-bringup
tools/bringup/run_bringup.py build/c50c-v1_1-bringup/app/vehkernel.elf
```

Bring-up cases that move actuators run in the main firmware's test mode instead ([Driving the wheels](driving.md#motor-bring-up)).

## Hardware checks

Each milestone ends with a hardware check on the board, recorded on its spec's last ticket (`docs/specs/`). The checks a person can repeat:

| Check | Guide |
|---|---|
| Serial Link at 200 Hz for a minute | [Link monitor](link-monitor.md#checks) |
| Attitude follows the board | [Link monitor](link-monitor.md#attitude) |
| Parameters survive a power cycle | [Link monitor](link-monitor.md#parameters) |
| Motor and encoder mapping | [Driving the wheels](driving.md#motor-bring-up) |
| Fault stop and hardware watchdog | [Driving the wheels](driving.md#fault-stop-check) |
