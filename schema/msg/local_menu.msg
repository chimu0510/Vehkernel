# The local menu's state (ADR-0022), published by the local menu on every
# change and once a second. A topic never published reads as closed, and the
# commander refuses arming while it is open.
@rate 1
uint64 timestamp
uint8 state                              # 0 closed, 1 list, 2 edit
uint16 item                              # parameter index of the item in the list or the edit, 65535 for Exit or none
uint8 last_result                        # result of the last Save (vk::core::ParamResult), 255 for none
uint16 last_parameter                    # parameter index the last Save wrote, 65535 for none
bool reboot_required                     # a menu parameter differs from its value at boot and needs a reboot
