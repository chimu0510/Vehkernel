# `.msg`-style schema; the custom protocol uses packed little-endian encoding

Every topic is defined in a ROS `.msg`-style schema, with extension annotations such as units and rates. The schema is the single source of truth. Generators produce the MCU structs and codecs, the Bridge decoders and ROS2 message conversion, tool-side parsers and field documentation, and compute a schema hash for each message. The custom protocol compares the hashes one by one during the handshake: the MCU announces every topic with its hash, and the host compares them with its own. A host that consumes the data refuses the connection if any of them differs, reporting which messages differ. The Link monitor, a diagnostic tool, stays connected so that it can name the messages that differ, never decodes them, and fails its check. The protocol version itself is negotiable (ADR-0002), the schema is not. Protocols that talk to standard tools, such as MAVLink and XRCE-DDS, leave compatibility to their own protocol and skip the hash handshake.

The first field of every topic is a 64-bit `timestamp` (ADR-0008), enforced by the generator. Encoding belongs to each protocol. The custom protocol uses packed little-endian: fields in schema order with no padding. Internal structs and the wire format are kept separate; the generator emits field-by-field `encode()` / `decode()` and never `memcpy`s a struct.

**Enums**

Enums whose values cross the wire, such as the commander's mode, arming result and disarm reason, are declared in the schema too, so their values are kept in one place instead of by hand in the firmware, the local display and the Link tools. Each enum has a `.enum` file of its own under `schema/enum/`, named like a message file, because many of them belong to no topic: the reasons in a mode, arming, reboot or bring-up reply travel in protocol messages, not in a topic. The file gives the integer type of the values and one `NAME = value` line for each, with an optional short name for narrow screens:

```
# Why the vehicle last disarmed.
@type uint8
NONE = 0                @short -
EMERGENCY_STOP = 2      @short ESTOP   # engaged for 5 s while armed
```

A field takes an enum's values with `@enum <enum>`, for example `uint8 mode  @enum mode`, and the generator checks that the field's integer type holds every value. The field keeps its integer type in the generated struct, so a value the schema does not name, from a peer built from a newer schema, still decodes and can be shown as a number. The generator emits a C++ `enum class` for each enum with a `constexpr` table of its values, their names and their short names (`vk/msg/enums.hpp`), and a Python `IntEnum` for the tools; the Link tools run without a build, so they keep a copy of the Python module in `tools/link/vklink/enums.py`, which the generator tests compare with the schema.

Enum values do not enter the schema hash, and neither does the `@enum` annotation: the hash covers what decides the wire layout, and an enum value changes no field's size or position. Values are therefore append-only: a new value may be added, but an existing one is never renumbered or reused for another meaning. A peer that does not know a new value shows it as a number. Renumbering one would be an incompatible change the hash cannot see, and needs a protocol version change instead (ADR-0002).

## Considered Options

- **CDR**: used only by the future XRCE-DDS protocol, generated from the same schema. Not used by the custom protocol, because alignment padding wastes bandwidth with no benefit on the MCU.
- **varint / tagged fields (protobuf-style)**: rejected. Robot data is mostly floating point, so there is no bandwidth to save, and the schema hash replaces "skip unknown fields" compatibility.
- **Enum values in the schema hash**: rejected. Only the enums a topic field takes could enter it; the reasons in protocol messages would still be covered by the protocol version alone. Adding a value, the common change, would then break every connection to tools built before it, for a value they can show as a number anyway.
- **Enum fields typed as the enum in the generated struct**: rejected. A value the schema does not name, from a newer peer, would turn into an enumerator the code does not expect; an integer field keeps it visible as a number, and the code casts it where it needs the enum.
- **Enums as constants inside the `.msg` file of their topic** (ROS and PX4 style, `uint8 MODE_STANDBY = 0`): rejected. Most of the commander's enums belong to no topic, and an enum that two topics share would have no file to live in.
- **A hand-written description for each value** (`@text "emergency stop engaged"`): deferred. The tools derive a readable text from the value name (`EMERGENCY_STOP` reads "emergency stop"), which kept every text close enough; a description would be one more string to keep in step.
- **`memcpy` of structs**: rejected. Padding differs between compilers and platforms, and the internal structs would be locked to the protocol.
