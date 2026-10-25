// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/vehicle_model/velocity_measurement_task.hpp>

#include <vk/msg/enums.hpp>
#include <vk/params/params.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <numbers>

namespace vk::vehicle_model {

VelocityMeasurementTask::VelocityMeasurementTask(const Config& config, core::WorkQueue& queue)
    : WorkItem("velocity_measurement", queue), config_(config)
{
}

void VelocityMeasurementTask::start()
{
    trigger_on(core::topic<msg::WheelEncoders>(0));
}

void VelocityMeasurementTask::run()
{
    msg::WheelEncoders encoders{};
    while (encoders_.copy(encoders)) {
        const std::uint64_t interval_us = last_sample_us_ == 0 ? 0 : encoders.timestamp - last_sample_us_;
        last_sample_us_ = encoders.timestamp;
        if (config_.model == nullptr || interval_us == 0) {
            continue;
        }
        const auto& mot = params::mot;
        const std::array<float, max_outputs> cpr{mot.mot1_cpr, mot.mot2_cpr, mot.mot3_cpr, mot.mot4_cpr};
        const float interval_s = static_cast<float>(interval_us) * 1e-6F;
        std::array<float, max_outputs> speeds{};
        std::array<float, max_outputs> noise{};
        for (std::size_t i = 0; i < max_outputs; ++i) {
            const bool valid = (encoders.valid & (1U << i)) != 0;
            speeds[i] = valid ? encoders.speed[i] : std::numeric_limits<float>::quiet_NaN();
            noise[i] = 2.0F * std::numbers::pi_v<float> / cpr[i] / (interval_s * std::sqrt(12.0F));
        }
        const auto measurement = config_.model->measure(speeds, noise);
        if (!measurement) {
            continue;
        }
        msg::BodyVelocityMeasurement out{};
        out.timestamp = queue().now_us();
        out.timestamp_sample = encoders.timestamp;
        out.source = static_cast<std::uint8_t>(msg::VelocitySource::Wheels);
        out.vx = measurement->twist.vx;
        out.vy = measurement->twist.vy;
        out.yaw_rate = measurement->twist.yaw_rate;
        out.variance = measurement->variance;
        out.residual = measurement->residual;
        publication_.publish(out);
    }
}

}  // namespace vk::vehicle_model
