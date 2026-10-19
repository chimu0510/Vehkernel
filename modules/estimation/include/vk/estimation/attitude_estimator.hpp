// The attitude estimator (ADR-0014): the Mahony filter as a work item on
// wq:ctrl, woken by each imu_sample publication (ADR-0008).
//
// Each run drains every queued sample (ADR-0015). It first aligns: once the
// vehicle has been still for the alignment time, roll and pitch come from
// the mean accelerometer reading, yaw is 0 and the gyro bias is the mean
// gyro reading; movement restarts the wait. Nothing is published while
// aligning. Then every sample steps the filter and is published on
// vehicle_attitude.
//
// The filter gains and the accelerometer rejection are the ATT_* parameters
// (ADR-0012), read at construction and again after each parameter_update, so
// a change applies from the next sample.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/estimation/filters/mahony.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/parameter_update.hpp>
#include <vk/msg/vehicle_attitude.hpp>

#include <cstdint>

namespace vk::estimation {

class AttitudeEstimator final : public core::WorkItem {
public:
    static constexpr std::uint8_t estimator_id = 1;

    // Compile-time settings; the gains in `filter` are replaced by the ATT_* parameters.
    struct Config {
        std::uint8_t imu_instance = 0;           // imu_sample instance to read
        MahonyConfig filter{};
        std::uint64_t alignment_us = 1'000'000;  // stillness needed to align
        float still_gyro = 0.05F;                // [rad/s] largest gyro norm counted as still
        float still_accel = 0.05F;               // largest | |a| - g | counted as still, as a fraction of g
        float level_variance = 3e-4F;            // [rad^2] roll and pitch, about (1 deg)^2
        float yaw_variance_rate = 1e-5F;         // [rad^2/s] yaw variance growth after alignment
    };

    enum class State : std::uint8_t {
        Aligning,
        Running,
    };

    AttitudeEstimator(const Config& config, core::WorkQueue& queue);

    // Listens to the IMU topic.
    void start();

    [[nodiscard]] State state() const { return state_; }

protected:
    void run() override;

private:
    void apply_parameters();
    void align(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel);
    void estimate(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel, std::uint64_t now);

    Config config_;
    core::Subscription<msg::ImuSample> imu_;
    core::Subscription<msg::ParameterUpdate> parameter_update_;
    core::Publication<msg::VehicleAttitude> attitude_;
    MahonyFilter filter_;
    State state_ = State::Aligning;

    // Alignment: sums since the vehicle became still.
    std::uint64_t still_since_us_ = 0;
    std::uint32_t still_samples_ = 0;
    Vector3 gyro_sum_;
    Vector3 accel_sum_;
    std::uint64_t aligned_at_us_ = 0;
};

}  // namespace vk::estimation
