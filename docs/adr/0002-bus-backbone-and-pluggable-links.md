# The internal bus is the backbone; ROS2 is just one Link

The MCU must run standalone, so ROS2 cannot be the firmware's internal backbone. Modules exchange data only through a publish/subscribe bus (like PX4's uORB), and all external communication goes through Links. A Link has three independent parts: a topic map, a protocol and a transport, and several Links can run at once. Every topic is defined in one schema, from which MCU, Bridge and tool code is generated.

The first protocol is a lightweight custom protocol: COBS + CRC16 framing, with sequence numbers, timestamps, version negotiation and time synchronization. Version negotiation covers the protocol itself only (framing and control messages such as the handshake, time sync, parameter access, arming and mode requests, bring-up, reboot and logging); both sides pick the highest version they share. A version only adds to the one before: new frame types, or fields appended to a frame's body; it never changes or removes what an older version defined. A host therefore offers every version from the oldest it still speaks, and refuses to send a frame the negotiated version does not have, saying which version it needs; the MCU offers only its own. A host updated past a board's firmware still reaches it, above all to reboot it into the bootloader and update it (ADR-0023). Topic compatibility is decided per topic by the schema hash (ADR-0006). The firmware is therefore not tied to a ROS2 release, and configuration, tuning and logging tools work over the same link when no companion computer is present.

## Considered Options

- **A host that offers only its own protocol version**: replaced. Every version bump stopped the next update of each board at the handshake, since the host could not ask older firmware to reboot into the bootloader (#144).
- **A reboot into the bootloader that needs no handshake or version**: rejected. Firmware already on boards would not have it, and it would be a second way into the board beside the reboot service.

- **micro-ROS (rclc), making the MCU a ROS2 node**: rejected. The rclc executor would share the MCU with the control chain, it needs an Agent, it ties the firmware to a ROS2 release, and no tool works without the companion computer.
- **XRCE-DDS through Micro-XRCE-DDS-Client directly, as PX4 does, without rclc**: planned as the second protocol, generated from the same schema, for users who want to talk to a standard Agent.
- **zenoh-pico + rmw_zenoh**: not on the MCU for now, because the MCU would have to emulate rmw_zenoh's wire format and serial support is still immature. zenoh belongs above the companion computer, on unreliable or multi-hop links between the vehicle and shore, ground stations or fleets.
- **MAVLink**: an optional protocol, so a standalone vehicle can talk to a telemetry radio and QGroundControl directly. Vehkernel borrows MAVLink's ecosystem, not its navigation role.

## Consequences

- A setpoint arriving on any Link goes through setpoint arbitration; it never drives actuators directly.
- A Link on a transport with a baud rate scales its outbound topic rates by one common factor when the topic map would exceed 80 % of the port's capacity, so a lower baud rate costs rate, not frames; 1 Hz status topics are never scaled, and the factor is reported in `link_status`. Rates per topic as parameters come later if this proves too coarse (ADR-0012).
- Protocol version 5 adds logging over the Link: `log_start`, `log_info`, `log_stop` and `log_dropout` (spec 0011, ADR-0019).
