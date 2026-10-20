#include <vk/actuators/bringup_case.hpp>

#include <algorithm>
#include <cstdlib>

namespace vk::actuators {

void BringupCase::start(std::uint64_t now_us, std::size_t outputs, std::span<const std::uint16_t> counts)
{
    state_ = BringupState::Running;
    outputs_ = std::min({outputs, max_outputs, counts.size()});
    current_ = 0;
    phase_start_us_ = now_us;
    std::copy_n(counts.begin(), outputs_, last_counts_.begin());
    moved_.fill(0);
    results_.fill({});
}

std::array<float, max_outputs> BringupCase::step(std::uint64_t now_us, std::span<const std::uint16_t> counts,
                                                 const Settings& settings)
{
    std::array<float, max_outputs> duties{};
    if (state_ != BringupState::Running) {
        return duties;
    }
    const bool pulsing = now_us - phase_start_us_ < pulse_us;
    for (std::size_t i = 0; i < outputs_ && i < counts.size(); ++i) {
        const auto delta = static_cast<std::int16_t>(static_cast<std::uint16_t>(counts[i] - last_counts_[i]));
        last_counts_[i] = counts[i];
        if (pulsing) {
            moved_[i] += settings.outputs[i].encoder_reversed ? -delta : delta;
        }
    }
    if (pulsing) {
        const float applied = std::min(duty, settings.loop.out_max);
        duties[current_] = settings.outputs[current_].reversed ? -applied : applied;
        return duties;
    }
    if (now_us - phase_start_us_ >= pulse_us + rest_us) {
        evaluate(settings);
        moved_.fill(0);
        phase_start_us_ = now_us;
        if (++current_ == outputs_) {
            state_ = BringupState::Done;
        }
    }
    return duties;
}

void BringupCase::abort()
{
    if (state_ == BringupState::Running) {
        state_ = BringupState::Aborted;
    }
}

void BringupCase::evaluate(const Settings& settings)
{
    std::size_t most = 0;
    bool any = false;
    for (std::size_t i = 0; i < outputs_; ++i) {
        const float threshold = min_turn * settings.outputs[i].counts_per_turn;
        if (static_cast<float>(std::abs(moved_[i])) < threshold) {
            continue;
        }
        if (!any || std::abs(moved_[i]) > std::abs(moved_[most])) {
            most = i;
            any = true;
        }
    }
    OutputResult& result = results_[current_];
    if (!any) {
        result = {.result = BringupResult::NotMoved, .encoder = 0, .sign = 0};
        return;
    }
    const auto sign = static_cast<std::int8_t>(moved_[most] > 0 ? 1 : -1);
    const auto encoder = static_cast<std::uint8_t>(most + 1);
    if (most != current_) {
        result = {.result = BringupResult::OtherEncoder, .encoder = encoder, .sign = sign};
    } else {
        result = {.result = sign > 0 ? BringupResult::Pass : BringupResult::Reversed, .encoder = encoder, .sign = sign};
    }
}

}  // namespace vk::actuators
