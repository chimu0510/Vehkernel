# SPDX-License-Identifier: GPL-3.0-or-later
# One magnetometer sample, rotated into the body frame and calibrated (ADR-0014).
# Queued so a batch of samples read at once is never overwritten (ADR-0015);
# the nominal rate is 100 Hz (spec 0012).
@queue 4
@instances 2
uint64 timestamp                 # time the sample was taken
uint8 device_instance            # which magnetometer, 0 for the first
float32[3] field  @unit uT       # magnetic field in the body frame, x forward, y right, z down
uint32 overflows                 # sensor overflow events detected
uint32 failures                  # sensor communication or configuration failures
