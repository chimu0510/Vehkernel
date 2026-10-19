# The state of parameter storage in Flash (ADR-0012, ADR-0020), once a second.
@rate 1
uint64 timestamp
uint16 parameter_count
uint8 active_sector                      # sector holding the log, 0 or 1 of the parameter region
uint32 bytes_used     @unit B            # of the active sector, its header included
uint32 sector_size    @unit B
uint16 unsaved                           # parameters whose value is not in Flash yet
uint16 pending_reboot                    # reboot_required parameters changed since boot
uint16 skipped_unknown                   # entries at boot: hash of no parameter
uint16 skipped_wrong_type                # entries at boot: type differs from the definition
uint16 skipped_out_of_range              # entries at boot: value outside the range
uint16 skipped_corrupt                   # entries at boot: CRC wrong, a write cut short
bool failed                              # Flash reported an error; nothing is written until reboot
