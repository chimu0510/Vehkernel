// SPDX-License-Identifier: GPL-3.0-or-later
// Parameter storage in internal Flash (ADR-0012, ADR-0020).
//
// Non-default values are stored entry by entry, keyed by the parameter
// name's hash, in two alternating Flash sectors. Each sector starts with a
// header, a generation counter and a magic word, written last when the sector
// becomes active, so a sector whose header is incomplete is never used; the
// active sector is the valid one with the highest generation. Entries of 12
// bytes follow:
//
//   word 0  name hash (param_hash)
//   word 1  value, its 32 bits
//   word 2  type (u8), 0xFF, CRC-16/CCITT-FALSE over the 10 bytes before it (u16)
//
// The last entry for a hash wins; the scan stops at the first entry that is
// all ones. A write cut short by a power loss fails its CRC and is skipped.
//
// At boot load() fills the parameters from the active sector, skipping and
// counting unknown hashes, wrong types, values out of range and corrupt
// entries, and erases the spare sector. Then the work item, on wq:slow,
// writes every changed value as a new entry, a few words per run so each
// Flash stall stays short. When the active sector is full, the current
// non-default values are compacted into the spare sector, which is allowed
// while armed; the old sector is erased only once disarmed. When neither has
// room while armed, changes stay in RAM, counted as unsaved, and are written
// after disarm. parameter_status reports all of it once a second.
#pragma once

#include <vk/core/parameters.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/flash.hpp>
#include <vk/msg/parameter_status.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::core {

// Flags of a parameter's state, as param_value carries them.
namespace param_state {
inline constexpr std::uint8_t pending_reboot = 1U << 0U;  // reboot_required and changed since boot
inline constexpr std::uint8_t unsaved = 1U << 1U;         // not in Flash yet
}  // namespace param_state

struct ParameterStorageConfig {
    std::uint32_t interval_us = 50'000;  // how often the storage runs
    std::size_t words_per_run = 6;       // Flash words written per run, at least 3: one entry
};

class ParameterStorage final : public WorkItem {
public:
    using Config = ParameterStorageConfig;

    static constexpr std::uint32_t status_interval_us = 1'000'000;
    static constexpr std::size_t header_bytes = 8;
    static constexpr std::size_t entry_bytes = 12;
    static constexpr std::uint32_t magic = 0x52415056U;  // "VPAR"

    // What storage knows of one parameter.
    struct Slot {
        ParamValue stored;     // the value Flash holds, the default without an entry
        ParamValue boot;       // the value after load()
        ParamValue compacted;  // the value written to the spare sector by a compaction in progress
    };

    // `slots` holds one Slot per parameter of `parameters`.
    ParameterStorage(Parameters& parameters, hal::FlashSectors flash, std::span<Slot> slots, WorkQueue& queue,
                     const Config& config = {});

    // Fills the parameters from Flash and prepares the sectors. Call once at
    // boot, before arming is possible and before anything reads parameters.
    void load();

    // Starts the periodic runs.
    void start();

    // param_state flags of one parameter.
    [[nodiscard]] std::uint8_t state(std::size_t index) const;

    [[nodiscard]] msg::ParameterStatus status() const;

protected:
    void run() override;

private:
    enum class Skip : std::uint8_t { Unknown, WrongType, OutOfRange, Corrupt };

    [[nodiscard]] bool blank(std::size_t sector) const;
    [[nodiscard]] bool room(std::size_t offset) const { return offset + entry_bytes <= flash_.sector_size(); }
    bool write_header(std::size_t sector, std::uint32_t generation);
    bool write_entry(std::size_t sector, std::size_t offset, std::uint32_t hash, ParamValue value);
    void scan(std::size_t sector);
    void save_changes(std::size_t& budget, bool armed);
    void compact(std::size_t& budget);
    void fail_on(hal::FlashResult result);

    Parameters* parameters_;
    hal::FlashSectors flash_;
    std::span<Slot> slots_;
    Config config_;
    Publication<msg::ParameterStatus> status_publication_;
    std::uint64_t next_status_us_ = 0;

    std::size_t active_ = 0;
    std::uint32_t generation_ = 0;
    std::size_t write_offset_ = header_bytes;  // next free entry in the active sector
    bool spare_blank_ = false;
    bool erase_old_ = false;  // the spare still holds the sector a compaction left; erase once disarmed
    bool compacting_ = false;
    std::size_t compact_index_ = 0;  // next parameter to copy
    std::size_t compact_offset_ = header_bytes;
    bool failed_ = false;  // Flash reported an error; nothing more is written until reboot
    std::array<std::uint16_t, 4> skipped_{};  // by Skip
};

}  // namespace vk::core
