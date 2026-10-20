# The vehicle's mode, arming state and arming checks (ADR-0005), published by
# the commander five times a second and whenever they change.
@rate 5
uint64 timestamp
uint8 mode                               # 0 STANDBY, 1 TEST
bool armed
uint8 source                             # Link instance in control, 255 for none
bool board_verified                      # the IMU answered and the revision-ID pins matched (ADR-0010)
bool emergency_stop                      # engaged
uint8 actuator_power                     # 0 not available, 1 unpowered, 2 settling, 3 powered
bool actuator_power_raw                  # the last sample read powered
uint32 skipped_checks                    # bits of COM_ARM_CHECK cleared
uint8 arming_refusal                     # the last arm request's refusal, 0 for none
uint8 disarm_reason                      # why the vehicle last disarmed, 0 for never
uint32 output_watchdog_trips             # times the output watchdog turned the outputs off
