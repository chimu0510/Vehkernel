# Whether actuators may produce output (ADR-0005). A topic never published
# reads as disarmed. Parameter storage and locked_when_armed parameters follow
# it (ADR-0012, ADR-0020).
uint64 timestamp
bool armed
