# Every scalar type, a fixed-size array and units.
# A second documentation line.
uint64 timestamp          # time of the sample
bool flag
int8 i8
int16 i16
int32 i32
int64 i64
uint8 u8
uint16 u16
uint32 u32
float32 speed  @unit m/s  # forward speed
float64 precise
float32[3] vector  @unit rad/s
