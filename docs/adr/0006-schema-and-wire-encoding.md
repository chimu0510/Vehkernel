# `.msg`-style schema; the custom protocol uses packed little-endian encoding

Every topic is defined in a ROS `.msg`-style schema, with extension annotations such as units and rates. The schema is the single source of truth. Generators produce the MCU structs and codecs, the Bridge decoders and ROS2 message conversion, tool-side parsers and field documentation, and compute a schema hash for each message. The custom protocol compares the hashes one by one during the handshake and refuses the connection if any of them differs, reporting which messages differ; the protocol version itself is negotiable (ADR-0002), the schema is not. Protocols that talk to standard tools, such as MAVLink and XRCE-DDS, leave compatibility to their own protocol and skip the hash handshake.

The first field of every topic is a 64-bit `timestamp` (ADR-0008), enforced by the generator. Encoding belongs to each protocol. The custom protocol uses packed little-endian: fields in schema order with no padding. Internal structs and the wire format are kept separate; the generator emits field-by-field `encode()` / `decode()` and never `memcpy`s a struct.

## Considered Options

- **CDR**: used only by the future XRCE-DDS protocol, generated from the same schema. Not used by the custom protocol, because alignment padding wastes bandwidth with no benefit on the MCU.
- **varint / tagged fields (protobuf-style)**: rejected. Robot data is mostly floating point, so there is no bandwidth to save, and the schema hash replaces "skip unknown fields" compatibility.
- **`memcpy` of structs**: rejected. Padding differs between compilers and platforms, and the internal structs would be locked to the protocol.
