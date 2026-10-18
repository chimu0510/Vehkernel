# Statistics of one Link since boot (ADR-0002). Each Link publishes on its
# own instance, and the topic map sends it to the peer.
@instances 2
@rate 1
uint64 timestamp
uint32 frames_sent
uint32 bytes_sent
uint32 frames_received                   # with a valid CRC
uint32 bytes_received
uint32 drops                             # frames the transport could not take
uint32 crc_errors                        # received frames dropped for a wrong CRC
uint32 malformed                         # received bytes that were not a frame
uint32 lost                              # frames missing from the peer's sequence
uint8 handshake_state                    # 0 waiting for hello, 1 connected, 2 peer silent
