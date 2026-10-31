# SPDX-License-Identifier: GPL-3.0-or-later
# The setup state of one magnetometer behind an IMU's I2C master (spec 0012),
# once a second, so a magnetometer that publishes no samples still shows why.
@instances 2
@rate 1
uint64 timestamp
uint8 device_instance                    # which magnetometer, 0 for the first
uint8 state  @enum magnetometer_state
uint8 last_failure  @enum magnetometer_failure
uint8 i2c_mst_status                     # I2C_MST_STATUS at the last setup step that read it
uint8 who_am_i                           # last magnetometer WHO_AM_I read, 0 before any
uint16 fifo_count  @unit B               # last FIFO count read while running
uint32 samples                           # published since boot
uint32 failures                          # as in magnetometer_sample
uint32 overflows                         # as in magnetometer_sample
uint32 setups                            # setups that reached RUNNING
