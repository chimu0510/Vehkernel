#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace ugv::base {

class SerialTransport {
public:
    SerialTransport();
    ~SerialTransport();

    bool open(const std::string& device, int baud_rate);
    void close();
    bool is_open() const;

    ssize_t write(const uint8_t* data, size_t size);
    ssize_t read(uint8_t* buffer, size_t max_size, int timeout_ms = 10);

private:
    int fd_{-1};
};

} // namespace ugv::base
