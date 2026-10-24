# A topic with fields that take the values of enums.
uint64 timestamp
int8 gear  @enum gear                 # the gear in effect
int16[2] requested  @enum gear
uint32 fault  @enum fault_code
float32 speed  @unit m/s
