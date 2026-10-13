#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace ugv::base {

class ClockSync {
public:
    explicit ClockSync(double alpha = 0.05);

    void update(uint64_t host_time_ns, uint32_t mcu_timestamp_us);

    rclcpp::Time to_ros_time(uint32_t mcu_timestamp_us);

    bool is_synchronized() const { return initialized_; }
    int64_t get_offset_ns() const { return offset_ns_; }

private:
    uint64_t unwrap_mcu_us(uint32_t mcu_timestamp_us);

    double alpha_{0.05};
    bool initialized_{false};
    uint32_t last_mcu_us_{0};
    uint64_t wrap_count_{0};
    int64_t offset_ns_{0};
};

} // namespace ugv::base
