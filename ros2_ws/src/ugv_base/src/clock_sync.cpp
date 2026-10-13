#include "ugv_base/clock_sync.hpp"

namespace ugv::base {

ClockSync::ClockSync(double alpha) : alpha_(alpha) {}

uint64_t ClockSync::unwrap_mcu_us(uint32_t mcu_timestamp_us) {
    if (!initialized_) {
        last_mcu_us_ = mcu_timestamp_us;
        return mcu_timestamp_us;
    }

    if (mcu_timestamp_us < last_mcu_us_ && (last_mcu_us_ - mcu_timestamp_us) > 0x80000000U) {
        wrap_count_++;
    }
    last_mcu_us_ = mcu_timestamp_us;

    return (wrap_count_ << 32) | static_cast<uint64_t>(mcu_timestamp_us);
}

void ClockSync::update(uint64_t host_time_ns, uint32_t mcu_timestamp_us) {
    uint64_t full_mcu_us = unwrap_mcu_us(mcu_timestamp_us);
    uint64_t mcu_time_ns = full_mcu_us * 1000ULL;

    int64_t current_offset = static_cast<int64_t>(host_time_ns) - static_cast<int64_t>(mcu_time_ns);

    if (!initialized_) {
        offset_ns_ = current_offset;
        initialized_ = true;
    } else {
        // Exponential Moving Average filter
        offset_ns_ = static_cast<int64_t>(alpha_ * current_offset + (1.0 - alpha_) * offset_ns_);
    }
}

rclcpp::Time ClockSync::to_ros_time(uint32_t mcu_timestamp_us) {
    if (!initialized_) {
        return rclcpp::Clock().now();
    }
    uint64_t full_mcu_us = (wrap_count_ << 32) | static_cast<uint64_t>(mcu_timestamp_us);
    uint64_t synched_ns = full_mcu_us * 1000ULL + static_cast<uint64_t>(offset_ns_);
    return rclcpp::Time(static_cast<int64_t>(synched_ns), RCL_ROS_TIME);
}

} // namespace ugv::base
