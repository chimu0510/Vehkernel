// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/sim/i2c_bus.hpp>

#include <vk/hal/i2c.hpp>

namespace vk::sim {

static_assert(hal::I2cBus<I2cBus>);

void I2cBus::attach(std::uint8_t address, I2cDeviceModel& device)
{
    detach(address);
    for (auto& slot : slots_) {
        if (slot.device == nullptr) {
            slot = {.address = address, .device = &device};
            return;
        }
    }
}

void I2cBus::detach(std::uint8_t address)
{
    for (auto& slot : slots_) {
        if (slot.device != nullptr && slot.address == address) {
            slot = {};
        }
    }
}

I2cDeviceModel* I2cBus::find(std::uint8_t address)
{
    for (auto& slot : slots_) {
        if (slot.device != nullptr && slot.address == address) {
            return slot.device;
        }
    }
    return nullptr;
}

bool I2cBus::write(std::uint8_t address, std::span<const std::uint8_t> data)
{
    I2cDeviceModel* device = find(address);
    return device != nullptr && device->write(data);
}

bool I2cBus::read(std::uint8_t address, std::span<std::uint8_t> data)
{
    I2cDeviceModel* device = find(address);
    return device != nullptr && device->read(data);
}

bool I2cBus::read_register(std::uint8_t address, std::uint8_t reg, std::span<std::uint8_t> data)
{
    I2cDeviceModel* device = find(address);
    const std::array<std::uint8_t, 1> pointer{reg};
    return device != nullptr && device->write(pointer) && device->read(data);
}

bool RegisterDevice::write(std::span<const std::uint8_t> data)
{
    if (data.empty()) {
        return true;
    }
    pointer_ = data[0];
    for (const std::uint8_t value : data.subspan(1)) {
        write_register(pointer_++, value);
    }
    return true;
}

bool RegisterDevice::read(std::span<std::uint8_t> data)
{
    for (auto& byte : data) {
        byte = read_register(pointer_++);
    }
    return true;
}

}  // namespace vk::sim
