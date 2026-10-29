// SPDX-License-Identifier: GPL-3.0-or-later
// Attitude estimator and magnetometer selection logic (ADR-0013, ADR-0014, spec 0012).
#pragma once

#include <vk/config.h>
#include <vk/core/bus.hpp>
#include <vk/msg/enums.hpp>
#include <vk/msg/estimator_attitude.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/params/params.hpp>

#include <cstdint>

namespace vk::estimation {

enum class PrimaryEstimator : std::uint8_t {
    Mahony = 1,
    Ekf = 2,
};

enum class EkfMagType : std::uint8_t {
    Auto = 0,
    Off = 1,
    Heading = 2,
};

// Pure function to resolve which attitude estimator is primary.
// If the selected estimator is not built in, it falls back to the other one.
// Both estimators calling this with identical built flags will always agree.
constexpr PrimaryEstimator resolve_primary(std::int32_t est_primary, bool mahony_built, bool ekf_built)
{
    if (est_primary == static_cast<std::int32_t>(PrimaryEstimator::Mahony)) {
        if (mahony_built) {
            return PrimaryEstimator::Mahony;
        }
        if (ekf_built) {
            return PrimaryEstimator::Ekf;
        }
    } else if (est_primary == static_cast<std::int32_t>(PrimaryEstimator::Ekf)) {
        if (ekf_built) {
            return PrimaryEstimator::Ekf;
        }
        if (mahony_built) {
            return PrimaryEstimator::Mahony;
        }
    }
    // Default or unknown: prefer EKF if built, else Mahony.
    if (ekf_built) {
        return PrimaryEstimator::Ekf;
    }
    return PrimaryEstimator::Mahony;
}

// True if a fallback occurred because the selected estimator was not built in.
constexpr bool is_primary_fallback(std::int32_t est_primary, PrimaryEstimator selected)
{
    if (est_primary == static_cast<std::int32_t>(PrimaryEstimator::Mahony)) {
        return selected != PrimaryEstimator::Mahony;
    }
    if (est_primary == static_cast<std::int32_t>(PrimaryEstimator::Ekf)) {
        return selected != PrimaryEstimator::Ekf;
    }
    return false;
}

// Reads EST_PRIMARY and the CONFIG_* macros to resolve the primary attitude estimator.
inline PrimaryEstimator primary_estimator()
{
    constexpr bool mahony_built =
#ifdef CONFIG_ESTIMATOR_MAHONY
        true;
#else
        false;
#endif
    constexpr bool ekf_built =
#ifdef CONFIG_ESTIMATOR_EKF
        true;
#else
        false;
#endif
#ifdef CONFIG_ESTIMATOR_EKF
    const std::int32_t est_primary = params::est.primary;
#else
    const std::int32_t est_primary = 1;  // MAHONY
#endif
    return resolve_primary(est_primary, mahony_built, ekf_built);
}

// Resolves EKF_MAG_TYPE (AUTO = 0, OFF = 1, HEADING = 2) for the given VEHICLE_TYPE.
// When EKF_MAG_TYPE is AUTO, GROUND resolves to OFF, and any other vehicle type resolves
// to HEADING (SURFACE and UNDERWATER are the expected ones, ADR-0013, spec 0012).
constexpr EkfMagType resolve_mag_type(EkfMagType mag_type, msg::VehicleType vehicle_type)
{
    if (mag_type == EkfMagType::Auto) {
        if (vehicle_type == msg::VehicleType::Ground) {
            return EkfMagType::Off;
        }
        return EkfMagType::Heading;
    }
    if (mag_type == EkfMagType::Heading) {
        return EkfMagType::Heading;
    }
    return EkfMagType::Off;
}

constexpr EkfMagType resolve_mag_type(std::int32_t mag_type, std::int32_t vehicle_type)
{
    return resolve_mag_type(static_cast<EkfMagType>(mag_type), static_cast<msg::VehicleType>(vehicle_type));
}

// Helper to copy VehicleAttitude to EstimatorAttitude.
inline msg::EstimatorAttitude to_estimator_attitude(const msg::VehicleAttitude& att)
{
    return msg::EstimatorAttitude{
        .timestamp = att.timestamp,
        .timestamp_sample = att.timestamp_sample,
        .estimator_id = att.estimator_id,
        .q = att.q,
        .angular_velocity = att.angular_velocity,
        .gyro_bias = att.gyro_bias,
        .variance = att.variance,
        .accel_rejected = att.accel_rejected,
        .mag_rejected = att.mag_rejected,
        .mag_fused = att.mag_fused,
        .zero_rate_fused = att.zero_rate_fused,
        .mag_missing = att.mag_missing,
    };
}

// Publishes to vehicle_attitude when primary, or estimator_attitude when secondary.
inline void publish_attitude(bool primary, const msg::VehicleAttitude& att,
                             core::Publication<msg::VehicleAttitude>& primary_pub,
                             core::Publication<msg::EstimatorAttitude>& secondary_pub)
{
    if (primary) {
        primary_pub.publish(att);
    } else {
        secondary_pub.publish(to_estimator_attitude(att));
    }
}

}  // namespace vk::estimation
