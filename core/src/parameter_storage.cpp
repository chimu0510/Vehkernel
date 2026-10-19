#include <vk/core/parameter_storage.hpp>

#include <array>

namespace vk::core {
namespace {

constexpr std::uint32_t erased = 0xFFFFFFFFU;

std::uint16_t crc16(const std::array<std::uint8_t, 10>& bytes)
{
    // CRC-16/CCITT-FALSE, as the custom protocol's frames.
    std::uint16_t crc = 0xFFFFU;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint16_t>(byte << 8U);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0 ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                                       : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

std::uint16_t entry_crc(std::uint32_t hash, std::uint32_t bits, std::uint8_t type)
{
    return crc16({static_cast<std::uint8_t>(hash), static_cast<std::uint8_t>(hash >> 8U),
                  static_cast<std::uint8_t>(hash >> 16U), static_cast<std::uint8_t>(hash >> 24U),
                  static_cast<std::uint8_t>(bits), static_cast<std::uint8_t>(bits >> 8U),
                  static_cast<std::uint8_t>(bits >> 16U), static_cast<std::uint8_t>(bits >> 24U), type, 0xFFU});
}

}  // namespace

ParameterStorage::ParameterStorage(Parameters& parameters, hal::FlashSectors flash, std::span<Slot> slots,
                                   WorkQueue& queue, const Config& config)
    : WorkItem("param_storage", queue), parameters_(&parameters), flash_(flash), slots_(slots), config_(config)
{
    if (slots_.size() < parameters_->count() || flash_.sector_count() < 2 || config_.words_per_run < 3) {
        fail("parameter storage needs a slot per parameter, two sectors and room for an entry per run");
    }
}

void ParameterStorage::start()
{
    schedule_periodic(config_.interval_us);
}

bool ParameterStorage::blank(std::size_t sector) const
{
    for (std::size_t offset = 0; offset < flash_.sector_size(); offset += 4) {
        if (flash_.read(sector, offset) != erased) {
            return false;
        }
    }
    return true;
}

void ParameterStorage::fail_on(hal::FlashResult result)
{
    if (result == hal::FlashResult::Error) {
        failed_ = true;
    }
}

bool ParameterStorage::write_header(std::size_t sector, std::uint32_t generation)
{
    // The magic word goes last: a header cut short is never valid.
    hal::FlashResult result = flash_.program(sector, 4, generation);
    if (result == hal::FlashResult::Ok) {
        result = flash_.program(sector, 0, magic);
    }
    fail_on(result);
    return result == hal::FlashResult::Ok;
}

bool ParameterStorage::write_entry(std::size_t sector, std::size_t offset, std::uint32_t hash, ParamValue value)
{
    const auto type = static_cast<std::uint8_t>(value.type);
    const std::uint32_t tail = type | (0xFFU << 8U) | (static_cast<std::uint32_t>(entry_crc(hash, value.bits, type)) << 16U);
    const std::array<std::uint32_t, 3> words{hash, value.bits, tail};
    for (std::size_t i = 0; i < words.size(); ++i) {
        const hal::FlashResult result = flash_.program(sector, offset + 4 * i, words[i]);
        if (result != hal::FlashResult::Ok) {
            fail_on(result);
            return false;
        }
    }
    return true;
}

void ParameterStorage::scan(std::size_t sector)
{
    std::size_t offset = header_bytes;
    for (; room(offset); offset += entry_bytes) {
        const std::uint32_t hash = flash_.read(sector, offset);
        const std::uint32_t bits = flash_.read(sector, offset + 4);
        const std::uint32_t tail = flash_.read(sector, offset + 8);
        if (hash == erased && bits == erased && tail == erased) {
            break;  // the end of the log
        }
        const auto type = static_cast<std::uint8_t>(tail);
        if (((tail >> 8U) & 0xFFU) != 0xFFU || (tail >> 16U) != entry_crc(hash, bits, type)) {
            ++skipped_[static_cast<std::size_t>(Skip::Corrupt)];
            continue;
        }
        const auto index = parameters_->find_hash(hash);
        if (!index) {
            ++skipped_[static_cast<std::size_t>(Skip::Unknown)];
            continue;
        }
        if (type != static_cast<std::uint8_t>(ParamType::Int32) && type != static_cast<std::uint8_t>(ParamType::Float)) {
            ++skipped_[static_cast<std::size_t>(Skip::WrongType)];
            continue;
        }
        const ParamValue value{.type = type == static_cast<std::uint8_t>(ParamType::Float) ? ParamType::Float
                                                                                           : ParamType::Int32,
                               .bits = bits};
        switch (parameters_->set(*index, value)) {
        case ParamResult::Ok:
            slots_[*index].stored = value;
            break;
        case ParamResult::WrongType:
            ++skipped_[static_cast<std::size_t>(Skip::WrongType)];
            break;
        default:
            ++skipped_[static_cast<std::size_t>(Skip::OutOfRange)];
            break;
        }
    }
    write_offset_ = offset;
}

void ParameterStorage::load()
{
    for (std::size_t i = 0; i < parameters_->count(); ++i) {
        slots_[i].stored = parameters_->info(i).default_value;
    }
    bool found = false;
    for (std::size_t sector = 0; sector < 2; ++sector) {
        const std::uint32_t generation = flash_.read(sector, 4);
        if (flash_.read(sector, 0) == magic && generation != erased && (!found || generation > generation_)) {
            found = true;
            active_ = sector;
            generation_ = generation;
        }
    }
    if (found) {
        scan(active_);
    } else {
        // Nothing stored yet, or nothing readable: start a log in sector 0.
        active_ = 0;
        generation_ = 1;
        write_offset_ = header_bytes;
        if (!(blank(0) || flash_.erase(0)) || !write_header(0, generation_)) {
            failed_ = true;
        }
    }
    // The spare sector is erased now, while disarmed, so a compaction never waits for it (ADR-0020).
    const std::size_t spare = 1 - active_;
    spare_blank_ = blank(spare) || flash_.erase(spare);
    for (std::size_t i = 0; i < parameters_->count(); ++i) {
        slots_[i].boot = parameters_->get(i);
    }
}

void ParameterStorage::run()
{
    const std::uint64_t now = queue().now_us();
    if (now >= next_status_us_) {
        next_status_us_ = now + status_interval_us;
        msg::ParameterStatus message = status();
        message.timestamp = now;
        status_publication_.publish(message);
    }
    if (failed_) {
        return;
    }
    const bool armed = parameters_->armed();
    if (erase_old_ && !armed) {
        // One erase per run: it stalls Flash for long (ADR-0020).
        erase_old_ = false;
        spare_blank_ = flash_.erase(1 - active_);
        return;
    }
    std::size_t budget = config_.words_per_run;
    if (compacting_) {
        compact(budget);
        return;
    }
    save_changes(budget, armed);
}

void ParameterStorage::save_changes(std::size_t& budget, bool armed)
{
    for (std::size_t i = 0; i < parameters_->count() && budget >= 3; ++i) {
        const ParamValue value = parameters_->get(i);
        if (value == slots_[i].stored) {
            continue;
        }
        if (!room(write_offset_)) {
            if (spare_blank_) {
                compacting_ = true;
                compact_index_ = 0;
                compact_offset_ = header_bytes;
            } else if (!armed && !erase_old_) {
                spare_blank_ = flash_.erase(1 - active_);
            }
            // Armed with no erased spare: the change stays unsaved until disarm.
            return;
        }
        const std::size_t offset = write_offset_;
        write_offset_ += entry_bytes;  // a slot a failed write left is not used again
        budget -= 3;
        if (write_entry(active_, offset, parameters_->info(i).hash, value)) {
            slots_[i].stored = value;
        } else if (failed_) {
            return;
        }
    }
}

void ParameterStorage::compact(std::size_t& budget)
{
    const std::size_t target = 1 - active_;
    for (; compact_index_ < parameters_->count() && budget >= 3; ++compact_index_) {
        const ParamInfo& info = parameters_->info(compact_index_);
        const ParamValue value = parameters_->get(compact_index_);
        slots_[compact_index_].compacted = info.default_value;
        if (value == info.default_value) {
            continue;
        }
        if (!room(compact_offset_) || !write_entry(target, compact_offset_, info.hash, value)) {
            failed_ = true;  // more non-default values than a sector holds, or Flash failed
            return;
        }
        slots_[compact_index_].compacted = value;
        compact_offset_ += entry_bytes;
        budget -= 3;
    }
    if (compact_index_ < parameters_->count() || budget < 2) {
        return;
    }
    if (!write_header(target, generation_ + 1)) {
        return;
    }
    // The new sector is the log now; the old one is erased once disarmed.
    for (std::size_t i = 0; i < parameters_->count(); ++i) {
        slots_[i].stored = slots_[i].compacted;
    }
    active_ = target;
    ++generation_;
    write_offset_ = compact_offset_;
    compacting_ = false;
    spare_blank_ = false;
    erase_old_ = true;
}

std::uint8_t ParameterStorage::state(std::size_t index) const
{
    const ParamValue value = parameters_->get(index);
    std::uint8_t flags = 0;
    if ((parameters_->info(index).flags & param_flags::reboot_required) != 0 && value != slots_[index].boot) {
        flags |= param_state::pending_reboot;
    }
    if (value != slots_[index].stored) {
        flags |= param_state::unsaved;
    }
    return flags;
}

msg::ParameterStatus ParameterStorage::status() const
{
    msg::ParameterStatus message{};
    message.parameter_count = static_cast<std::uint16_t>(parameters_->count());
    message.active_sector = static_cast<std::uint8_t>(active_);
    message.bytes_used = static_cast<std::uint32_t>(write_offset_);
    message.sector_size = static_cast<std::uint32_t>(flash_.sector_size());
    for (std::size_t i = 0; i < parameters_->count(); ++i) {
        const std::uint8_t flags = state(i);
        if ((flags & param_state::unsaved) != 0) {
            ++message.unsaved;
        }
        if ((flags & param_state::pending_reboot) != 0) {
            ++message.pending_reboot;
        }
    }
    message.skipped_unknown = skipped_[static_cast<std::size_t>(Skip::Unknown)];
    message.skipped_wrong_type = skipped_[static_cast<std::size_t>(Skip::WrongType)];
    message.skipped_out_of_range = skipped_[static_cast<std::size_t>(Skip::OutOfRange)];
    message.skipped_corrupt = skipped_[static_cast<std::size_t>(Skip::Corrupt)];
    message.failed = failed_;
    return message;
}

}  // namespace vk::core
