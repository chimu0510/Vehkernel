// SPDX-License-Identifier: GPL-3.0-or-later
// Replay program (ADR-0018 L2, spec 0011, spec 0012): builds attitude estimators
// (AttitudeEstimator, EkfAttitudeEstimator) and LocalOdometry unchanged on a WorkQueue
// with a replay clock.
//
// Work item order on wq:ctrl:
// 1. IMU driver (ICM20948): publishes imu_sample and magnetometer_sample at the same run timestamp.
// 2. Attitude estimators (AttitudeEstimator, EkfAttitudeEstimator): woken by imu_sample,
//    drain magnetometer_sample, publish vehicle_attitude and estimator_attitude.
// 3. Vehicle model (VelocityMeasurementTask): computes body velocity measurement
//    from wheel encoders, publishes body_velocity_measurement, which wakes local_odometry.
// 4. Local odometry (LocalOdometry): dead-reckons along the attitude's heading,
//    publishes vehicle_local_odometry.
//
// When records share the same timestamp, inputs are published in this order:
// parameter changes -> estimator_restart -> magnetometer_sample (instance 0) -> imu_sample -> body_velocity_measurement.
//
// Stdin/stdout simple framing:
// - Record 'P' (0x50): Parameter
//   timestamp_us (uint64, LE)
//   type (uint8, 1=Int32, 2=Float)
//   name_len (uint8)
//   name (name_len bytes)
//   value (4 bytes, int32 or float, LE)
// - Record 'D' (0x44): Data
//   name_len (uint8)
//   name (name_len bytes)
//   instance (uint8)
//   payload_len (uint16, LE)
//   payload (payload_len bytes)
#include <vk/config.h>
#include <vk/core/bus.hpp>
#include <vk/core/parameters.hpp>
#include <vk/core/work_queue.hpp>
#ifdef CONFIG_ESTIMATOR_MAHONY
#include <vk/estimation/attitude_estimator.hpp>
#endif
#ifdef CONFIG_ESTIMATOR_EKF
#include <vk/estimation/ekf_attitude_estimator.hpp>
#endif
#include <vk/estimation/local_odometry.hpp>
#include <vk/msg/body_velocity_measurement.hpp>
#include <vk/msg/estimator_attitude.hpp>
#include <vk/msg/estimator_restart.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/magnetometer_sample.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_local_odometry.hpp>
#include <vk/params/params.hpp>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <string_view>

namespace {

std::uint64_t current_time_us = 0;

std::uint64_t replay_clock() noexcept
{
    return current_time_us;
}

bool read_exact(void* dst, std::size_t bytes)
{
    auto* p = static_cast<std::uint8_t*>(dst);
    while (bytes > 0) {
        const std::size_t n = std::fread(p, 1, bytes, stdin);
        if (n == 0) {
            return false;
        }
        p += n;
        bytes -= n;
    }
    return true;
}

bool write_exact(const void* src, std::size_t bytes)
{
    const auto* p = static_cast<const std::uint8_t*>(src);
    while (bytes > 0) {
        const std::size_t n = std::fwrite(p, 1, bytes, stdout);
        if (n == 0) {
            return false;
        }
        p += n;
        bytes -= n;
    }
    return true;
}

void write_stdout_record(std::string_view name, std::uint8_t instance, std::span<const std::uint8_t> payload)
{
    const std::uint8_t magic = 'D';
    const auto nlen = static_cast<std::uint8_t>(name.size());
    const auto plen = static_cast<std::uint16_t>(payload.size());
    write_exact(&magic, 1);
    write_exact(&nlen, 1);
    write_exact(name.data(), nlen);
    write_exact(&instance, 1);
    write_exact(&plen, 2);
    write_exact(payload.data(), plen);
}

}  // namespace

int main()
{
    vk::core::Parameters params(vk::params::table(), &replay_clock);

    // Initial parameter setup before work items are started.
    // Stdin loop will first receive initial parameters with timestamp 0.
    // Work items are constructed after initial parameters are applied,
    // so they read logged EST_PRIMARY, EKF_MAG_TYPE, etc. (spec 0012).
    vk::core::WorkQueue queue("wq:ctrl", 1, &replay_clock);

#ifdef CONFIG_ESTIMATOR_MAHONY
    std::optional<vk::estimation::AttitudeEstimator> attitude_estimator;
#endif
#ifdef CONFIG_ESTIMATOR_EKF
    std::optional<vk::estimation::EkfAttitudeEstimator> ekf_attitude_estimator;
#endif
    std::optional<vk::estimation::LocalOdometry> local_odometry;

    bool estimators_started = false;
    auto ensure_estimators_started = [&]() {
        if (estimators_started) {
            return;
        }
        estimators_started = true;
#ifdef CONFIG_ESTIMATOR_MAHONY
        attitude_estimator.emplace(vk::estimation::AttitudeEstimator::Config{.imu_instance = 0}, queue);
        attitude_estimator->start();
#endif
#ifdef CONFIG_ESTIMATOR_EKF
        ekf_attitude_estimator.emplace(vk::estimation::EkfAttitudeEstimator::Config{.imu_instance = 0, .mag_instance = 0},
                                       queue);
        ekf_attitude_estimator->start();
#endif
        local_odometry.emplace(vk::estimation::LocalOdometry::Config{.measurement = 0}, queue);
        local_odometry->start();
    };

    vk::core::Publication<vk::msg::ImuSample> imu_pub{0};
    vk::core::Publication<vk::msg::MagnetometerSample> mag_pub{0};
    vk::core::Publication<vk::msg::BodyVelocityMeasurement> vel_pub{0};
    vk::core::Publication<vk::msg::EstimatorRestart> restart_pub{0};

    vk::core::Subscription<vk::msg::VehicleAttitude> attitude_sub{0};
    vk::core::Subscription<vk::msg::EstimatorAttitude> est_attitude_sub0{0};
    vk::core::Subscription<vk::msg::EstimatorAttitude> est_attitude_sub1{1};
    vk::core::Subscription<vk::msg::VehicleLocalOdometry> odometry_sub{0};

    std::array<std::array<char, 64>, 64> unknown_names{};
    std::size_t unknown_count = 0;

    auto drain_and_write_outputs = [&]() {
        vk::msg::VehicleAttitude att{};
        while (attitude_sub.copy(att)) {
            std::array<std::uint8_t, vk::msg::VehicleAttitude::encoded_size> out_buf{};
            const std::size_t enc_len = att.encode(out_buf);
            if (enc_len > 0) {
                write_stdout_record("vehicle_attitude", 0, std::span(out_buf.data(), enc_len));
            }
        }

        vk::msg::EstimatorAttitude est_att0{};
        while (est_attitude_sub0.copy(est_att0)) {
            std::array<std::uint8_t, vk::msg::EstimatorAttitude::encoded_size> out_buf{};
            const std::size_t enc_len = est_att0.encode(out_buf);
            if (enc_len > 0) {
                write_stdout_record("estimator_attitude", 0, std::span(out_buf.data(), enc_len));
            }
        }

        vk::msg::EstimatorAttitude est_att1{};
        while (est_attitude_sub1.copy(est_att1)) {
            std::array<std::uint8_t, vk::msg::EstimatorAttitude::encoded_size> out_buf{};
            const std::size_t enc_len = est_att1.encode(out_buf);
            if (enc_len > 0) {
                write_stdout_record("estimator_attitude", 1, std::span(out_buf.data(), enc_len));
            }
        }

        vk::msg::VehicleLocalOdometry odom{};
        while (odometry_sub.copy(odom)) {
            std::array<std::uint8_t, vk::msg::VehicleLocalOdometry::encoded_size> out_buf{};
            const std::size_t enc_len = odom.encode(out_buf);
            if (enc_len > 0) {
                write_stdout_record("vehicle_local_odometry", 0, std::span(out_buf.data(), enc_len));
            }
        }
    };

    while (true) {
        std::uint8_t magic = 0;
        if (!read_exact(&magic, 1)) {
            break;  // EOF
        }

        if (magic == 'P') {
            std::uint64_t ts = 0;
            std::uint8_t ptype = 0;
            std::uint8_t nlen = 0;
            if (!read_exact(&ts, 8) || !read_exact(&ptype, 1) || !read_exact(&nlen, 1)) {
                return 1;
            }
            std::array<char, 64> name{};
            if (nlen >= name.size() || !read_exact(name.data(), nlen)) {
                return 1;
            }
            name.at(nlen) = '\0';
            std::uint32_t val_bits = 0;
            if (!read_exact(&val_bits, 4)) {
                return 1;
            }

            if (ts > 0) {
                ensure_estimators_started();
                current_time_us = ts;
            }
            const std::string_view name_view(name.data(), nlen);
            const auto opt_idx = params.find(name_view);
            if (opt_idx) {
                const vk::core::ParamValue val =
                    (ptype == 1) ? vk::core::ParamValue::from_int(std::bit_cast<std::int32_t>(val_bits))
                                 : vk::core::ParamValue::from_float(std::bit_cast<float>(val_bits));
                params.set(*opt_idx, val);
                if (ts > 0) {
                    queue.process();
                    drain_and_write_outputs();
                }
            } else {
                bool already_seen = false;
                for (std::size_t u = 0; u < unknown_count && u < unknown_names.size(); ++u) {
                    if (std::string_view(unknown_names.at(u).data()) == name_view) {
                        already_seen = true;
                        break;
                    }
                }
                if (!already_seen) {
                    if (unknown_count < unknown_names.size()) {
                        name_view.copy(unknown_names.at(unknown_count).data(), name_view.size());
                        unknown_names.at(unknown_count).at(name_view.size()) = '\0';
                    }
                    ++unknown_count;
                    std::fprintf(stderr, "unknown parameter: %.*s\n", static_cast<int>(name_view.size()),
                                 name_view.data());
                }
            }
        } else if (magic == 'D') {
            ensure_estimators_started();
            std::uint8_t nlen = 0;
            if (!read_exact(&nlen, 1)) {
                return 1;
            }
            std::array<char, 64> name{};
            if (nlen >= name.size() || !read_exact(name.data(), nlen)) {
                return 1;
            }
            name.at(nlen) = '\0';
            std::uint8_t instance = 0;
            std::uint16_t plen = 0;
            if (!read_exact(&instance, 1) || !read_exact(&plen, 2)) {
                return 1;
            }
            std::array<std::uint8_t, 512> payload{};
            if (plen > payload.size() || !read_exact(payload.data(), plen)) {
                return 1;
            }

            const std::string_view topic_name(name.data(), nlen);

            if (topic_name == "magnetometer_sample" && instance == 0) {
                vk::msg::MagnetometerSample sample{};
                if (vk::msg::MagnetometerSample::decode(std::span(payload.data(), plen), sample)) {
                    current_time_us = sample.timestamp;
                    mag_pub.publish(sample);
                }
            } else if (topic_name == "imu_sample" && instance == 0) {
                vk::msg::ImuSample sample{};
                if (vk::msg::ImuSample::decode(std::span(payload.data(), plen), sample)) {
                    current_time_us = sample.timestamp;
                    imu_pub.publish(sample);
                    queue.process();
                    drain_and_write_outputs();
                }
            } else if (topic_name == "body_velocity_measurement" && instance == 0) {
                vk::msg::BodyVelocityMeasurement measurement{};
                if (vk::msg::BodyVelocityMeasurement::decode(std::span(payload.data(), plen), measurement)) {
                    current_time_us = measurement.timestamp;
                    vel_pub.publish(measurement);
                    queue.process();
                    drain_and_write_outputs();
                }
            } else if (topic_name == "estimator_restart") {
                vk::msg::EstimatorRestart restart{};
                if (vk::msg::EstimatorRestart::decode(std::span(payload.data(), plen), restart)) {
                    current_time_us = restart.timestamp;
                    restart_pub.publish(restart);
                    queue.process();
                    drain_and_write_outputs();
                }
            }
        } else {
            return 1;
        }
    }

    if (unknown_count > 0) {
        std::fprintf(stderr, "unknown parameters count: %zu\n", unknown_count);
    }
    std::fflush(stdout);
    return 0;
}
