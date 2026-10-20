# A bring-up case the commander accepted (ADR-0018), for actuator control to
# run: published once per bringup_run request that passes its checks.
uint64 timestamp
uint8 bringup_case                      # 1 motor-to-encoder mapping and direction
uint8 source                            # Link instance that asked for it
