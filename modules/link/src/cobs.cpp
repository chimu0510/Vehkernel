#include <vk/link/cobs.hpp>

namespace vk::link {

std::size_t cobs_encode(std::span<const std::uint8_t> in, std::span<std::uint8_t> out)
{
    if (out.size() < cobs_max_encoded_size(in.size())) {
        return 0;
    }
    std::size_t code_at = 0;  // where the current block's length code goes
    std::size_t at = 1;
    std::uint8_t code = 1;
    for (std::size_t i = 0; i < in.size(); ++i) {
        const std::uint8_t byte = in[i];
        if (byte != 0) {
            out[at++] = byte;
            ++code;
        }
        if (byte == 0 || code == 0xFF) {
            out[code_at] = code;
            code = 1;
            code_at = at;
            // A full block that ends the input needs no further block.
            if (byte == 0 || i + 1 < in.size()) {
                ++at;
            } else {
                return at;
            }
        }
    }
    out[code_at] = code;
    return at;
}

std::size_t cobs_decode(std::span<const std::uint8_t> in, std::span<std::uint8_t> out)
{
    std::size_t at = 0;
    std::size_t i = 0;
    while (i < in.size()) {
        const std::uint8_t code = in[i++];
        if (code == 0 || i + code - 1 > in.size()) {
            return 0;
        }
        for (std::uint8_t n = 1; n < code; ++n) {
            if (in[i] == 0 || at == out.size()) {
                return 0;
            }
            out[at++] = in[i++];
        }
        // A block shorter than 255 ends in a zero, except the last one.
        if (code != 0xFF && i < in.size()) {
            if (at == out.size()) {
                return 0;
            }
            out[at++] = 0;
        }
    }
    return at;
}

}  // namespace vk::link
