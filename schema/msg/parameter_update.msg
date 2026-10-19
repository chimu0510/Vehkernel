# Published on every parameter change (ADR-0012). Consumers check updated()
# and read their parameter group again.
uint64 timestamp
uint32 name_hash                         # param_hash() of the changed parameter's name
uint32 change_count                      # changes since boot, this one included
