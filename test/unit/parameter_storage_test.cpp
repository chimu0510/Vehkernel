// Parameter storage in Flash (ADR-0012, ADR-0020), against the NOR Flash
// model: values survive reboots and power loss at any word, compaction,
// entries skipped at boot, and the write rules while armed.
#include <vk/core/bus.hpp>
#include <vk/core/parameter_storage.hpp>
#include <vk/core/parameters.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/flash.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/sim/nor_flash.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <string_view>
#include <vector>

namespace {

using vk::core::ParameterStorage;
using vk::core::ParamInfo;
using vk::core::ParamResult;
using vk::core::Parameters;
using vk::core::ParamType;
using vk::core::ParamValue;
using vk::core::param_hash;
namespace param_state = vk::core::param_state;

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

// RAM of one boot: the parameter group, back at its defaults at every boot.
struct Values {
    std::int32_t count = 3;  // reboot_required
    float gain = 0.5F;
    std::int32_t mode = 0;
};
Values values;

constexpr std::array<std::int32_t, 3> mode_values{-1, 0, 1};

std::array<ParamInfo, 3> make_table()
{
    return {{
        {.name = "T_COUNT", .hash = param_hash("T_COUNT"), .type = ParamType::Int32,
         .flags = vk::core::param_flags::reboot_required, .default_value = ParamValue::from_int(3),
         .min = ParamValue::from_int(-10), .max = ParamValue::from_int(10), .enum_values = {}, .bitmask = 0,
         .int_value = &values.count, .float_value = nullptr},
        {.name = "T_GAIN", .hash = param_hash("T_GAIN"), .type = ParamType::Float, .flags = 0,
         .default_value = ParamValue::from_float(0.5F), .min = ParamValue::from_float(0.0F),
         .max = ParamValue::from_float(100.0F), .enum_values = {}, .bitmask = 0, .int_value = nullptr,
         .float_value = &values.gain},
        {.name = "T_MODE", .hash = param_hash("T_MODE"), .type = ParamType::Int32, .flags = 0,
         .default_value = ParamValue::from_int(0), .min = ParamValue::from_int(-1), .max = ParamValue::from_int(1),
         .enum_values = mode_values, .bitmask = 0, .int_value = &values.mode, .float_value = nullptr},
    }};
}

// Ten entries per sector, so compactions come quickly.
constexpr std::size_t sector_size = ParameterStorage::header_bytes + 10 * ParameterStorage::entry_bytes;
constexpr std::uint32_t interval_us = 1'000;

void arm(bool armed)
{
    vk::msg::ArmingState state{};
    state.timestamp = fake_now;
    state.armed = armed;
    vk::core::Publication<vk::msg::ArmingState>{}.publish(state);
}

// One boot of a vehicle: fresh RAM over the same Flash.
class Boot {
public:
    explicit Boot(vk::sim::NorFlash& flash)
    {
        values = Values{};
        storage_ = &holder_.emplace(parameters_, vk::hal::FlashSectors(flash), slots_, queue_,
                                    ParameterStorage::Config{.interval_us = interval_us, .words_per_run = 6});
        storage_->load();
        storage_->start();
    }

    // Runs the storage `runs` times.
    void run(int runs = 1)
    {
        for (int i = 0; i < runs; ++i) {
            fake_now += interval_us;
            queue_.process();
        }
    }

    // Runs until every change is in Flash; false if that does not happen.
    bool save(int max_runs = 50)
    {
        for (int i = 0; i < max_runs; ++i) {
            if (status().unsaved == 0) {
                return true;
            }
            run();
        }
        return status().unsaved == 0;
    }

    void set(std::string_view name, ParamValue value) { ASSERT_EQ(parameters_.set(name, value), ParamResult::Ok); }
    [[nodiscard]] std::uint8_t state(std::string_view name) const
    {
        return storage_->state(parameters_.find(name).value_or(0));
    }
    [[nodiscard]] vk::msg::ParameterStatus status() const { return storage_->status(); }

private:
    std::array<ParamInfo, 3> table_ = make_table();
    Parameters parameters_{table_, &fake_clock};
    std::array<ParameterStorage::Slot, 3> slots_{};
    vk::core::WorkQueue queue_{"wq:test", 1, &fake_clock};
    std::optional<ParameterStorage> holder_;
    ParameterStorage* storage_ = nullptr;
};

class ParameterStorageTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        arm(false);
    }
    void TearDown() override { arm(false); }

    vk::sim::NorFlash flash{2, sector_size};
};

TEST_F(ParameterStorageTest, BlankFlashGivesTheDefaults)
{
    {
        Boot boot(flash);
        EXPECT_EQ(values.count, 3);
        EXPECT_EQ(values.gain, 0.5F);
        const auto status = boot.status();
        EXPECT_EQ(status.parameter_count, 3U);
        EXPECT_EQ(status.unsaved, 0U);
        EXPECT_EQ(status.bytes_used, ParameterStorage::header_bytes);
        EXPECT_FALSE(status.failed);
    }
    EXPECT_EQ(flash.read(0, 0), ParameterStorage::magic);  // the log starts in sector 0
    Boot again(flash);
    EXPECT_EQ(values.gain, 0.5F);
}

TEST_F(ParameterStorageTest, ChangesSurviveAReboot)
{
    {
        Boot boot(flash);
        boot.set("T_GAIN", ParamValue::from_float(1.25F));
        boot.set("T_COUNT", ParamValue::from_int(7));
        EXPECT_EQ(boot.state("T_GAIN"), param_state::unsaved);
        EXPECT_EQ(boot.state("T_COUNT"), param_state::unsaved | param_state::pending_reboot);
        ASSERT_TRUE(boot.save());
        EXPECT_EQ(boot.state("T_GAIN"), 0);
        EXPECT_EQ(boot.state("T_COUNT"), param_state::pending_reboot);  // stored, but read only at boot
        EXPECT_EQ(boot.status().pending_reboot, 1U);
    }
    Boot again(flash);
    EXPECT_EQ(values.gain, 1.25F);
    EXPECT_EQ(values.count, 7);
    EXPECT_EQ(values.mode, 0);
    EXPECT_EQ(again.state("T_COUNT"), 0);
    EXPECT_EQ(again.status().pending_reboot, 0U);
}

TEST_F(ParameterStorageTest, CompactionKeepsEveryNonDefaultValueAndDropsDefaults)
{
    {
        Boot boot(flash);
        boot.set("T_COUNT", ParamValue::from_int(7));
        boot.set("T_MODE", ParamValue::from_int(-1));
        ASSERT_TRUE(boot.save());
        // Many changes of one gain fill the sector several times over.
        for (int i = 1; i <= 25; ++i) {
            boot.set("T_GAIN", ParamValue::from_float(static_cast<float>(i)));
            ASSERT_TRUE(boot.save()) << i;
        }
        boot.set("T_MODE", ParamValue::from_int(0));  // back to the default
        ASSERT_TRUE(boot.save());
        boot.run(5);  // the old sector is erased after the last compaction
        EXPECT_LT(boot.status().bytes_used, sector_size);
    }
    Boot again(flash);
    EXPECT_EQ(values.count, 7);
    EXPECT_EQ(values.gain, 25.0F);
    EXPECT_EQ(values.mode, 0);
    // A compaction now would copy two values: T_MODE at its default takes no entry.
    for (int i = 0; i < 10; ++i) {
        again.set("T_GAIN", ParamValue::from_float(static_cast<float>(30 + i)));
        ASSERT_TRUE(again.save());
    }
    EXPECT_EQ(again.status().skipped_corrupt, 0U);
}

TEST_F(ParameterStorageTest, PowerLossAtAnyWordLeavesTheOldOrTheNewValue)
{
    // A start from saved values, then changes enough for a compaction.
    const auto prepare = [](vk::sim::NorFlash& f) {
        Boot boot(f);
        boot.set("T_COUNT", ParamValue::from_int(5));
        boot.set("T_GAIN", ParamValue::from_float(2.0F));
        EXPECT_TRUE(boot.save());
    };
    const auto change = [](Boot& boot) {
        boot.set("T_MODE", ParamValue::from_int(1));
        boot.set("T_COUNT", ParamValue::from_int(-4));
        for (int i = 0; i < 12; ++i) {
            boot.set("T_GAIN", ParamValue::from_float(10.0F + static_cast<float>(i)));
            boot.save();
        }
        boot.run(10);
    };
    std::set<float> gains{2.0F};
    for (int i = 0; i < 12; ++i) {
        gains.insert(10.0F + static_cast<float>(i));
    }

    // How many Flash operations the changes take without a power loss.
    std::size_t total = 0;
    {
        vk::sim::NorFlash reference{2, sector_size};
        prepare(reference);
        Boot boot(reference);
        const std::size_t after_boot = reference.operations();
        change(boot);
        // Entries, compactions, headers, and the erases of the sectors compactions left.
        total = reference.operations() - after_boot;
        ASSERT_GT(total, 40U);
    }

    for (std::size_t k = 0; k <= total; ++k) {
        SCOPED_TRACE(k);
        vk::sim::NorFlash f{2, sector_size};
        prepare(f);
        {
            Boot boot(f);
            f.lose_power_after(k);
            change(boot);
        }
        f.power_on();
        Boot after(f);
        EXPECT_TRUE(values.count == 5 || values.count == -4) << values.count;
        EXPECT_TRUE(values.mode == 0 || values.mode == 1) << values.mode;
        EXPECT_EQ(gains.count(values.gain), 1U) << values.gain;
        EXPECT_FALSE(after.status().failed);
        EXPECT_EQ(after.status().skipped_unknown + after.status().skipped_wrong_type +
                      after.status().skipped_out_of_range,
                  0U);
        // Storage goes on after the power loss.
        after.set("T_GAIN", ParamValue::from_float(99.0F));
        ASSERT_TRUE(after.save());
    }
}

// Writes one entry by hand, as storage lays it out, with a CRC that is right or not.
void write_entry(vk::sim::NorFlash& flash, std::size_t slot, std::uint32_t hash, std::uint8_t type,
                 std::uint32_t bits, bool good_crc = true)
{
    const std::array<std::uint8_t, 10> bytes{static_cast<std::uint8_t>(hash), static_cast<std::uint8_t>(hash >> 8U),
                                             static_cast<std::uint8_t>(hash >> 16U),
                                             static_cast<std::uint8_t>(hash >> 24U), static_cast<std::uint8_t>(bits),
                                             static_cast<std::uint8_t>(bits >> 8U),
                                             static_cast<std::uint8_t>(bits >> 16U),
                                             static_cast<std::uint8_t>(bits >> 24U), type, 0xFFU};
    std::uint16_t crc = 0xFFFFU;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint16_t>(byte << 8U);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0 ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                                       : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    if (!good_crc) {
        crc ^= 1U;
    }
    const std::size_t offset = ParameterStorage::header_bytes + slot * ParameterStorage::entry_bytes;
    ASSERT_EQ(flash.program(0, offset, hash), vk::hal::FlashResult::Ok);
    ASSERT_EQ(flash.program(0, offset + 4, bits), vk::hal::FlashResult::Ok);
    ASSERT_EQ(flash.program(0, offset + 8, type | (0xFFU << 8U) | (static_cast<std::uint32_t>(crc) << 16U)),
              vk::hal::FlashResult::Ok);
}

TEST_F(ParameterStorageTest, SkipsAndCountsEntriesItCannotUse)
{
    ASSERT_EQ(flash.program(0, 4, 1), vk::hal::FlashResult::Ok);  // generation 1
    ASSERT_EQ(flash.program(0, 0, ParameterStorage::magic), vk::hal::FlashResult::Ok);
    write_entry(flash, 0, param_hash("T_GAIN"), 2, ParamValue::from_float(1.5F).bits);  // good
    write_entry(flash, 1, param_hash("T_GONE"), 2, ParamValue::from_float(1.0F).bits);  // a removed parameter
    write_entry(flash, 2, param_hash("T_MODE"), 2, ParamValue::from_float(1.0F).bits);  // stored as a float
    write_entry(flash, 3, param_hash("T_COUNT"), 1, ParamValue::from_int(99).bits);     // out of range now
    write_entry(flash, 4, param_hash("T_COUNT"), 1, ParamValue::from_int(4).bits, false);  // corrupt
    write_entry(flash, 5, param_hash("T_GAIN"), 7, ParamValue::from_float(3.0F).bits);  // no such type

    Boot boot(flash);
    EXPECT_EQ(values.gain, 1.5F);
    EXPECT_EQ(values.mode, 0);
    EXPECT_EQ(values.count, 3);
    const auto status = boot.status();
    EXPECT_EQ(status.skipped_unknown, 1U);
    EXPECT_EQ(status.skipped_wrong_type, 2U);
    EXPECT_EQ(status.skipped_out_of_range, 1U);
    EXPECT_EQ(status.skipped_corrupt, 1U);
    // New entries go after the last one, not over it.
    boot.set("T_COUNT", ParamValue::from_int(2));
    ASSERT_TRUE(boot.save());
    EXPECT_EQ(boot.status().bytes_used, ParameterStorage::header_bytes + 7 * ParameterStorage::entry_bytes);
}

TEST_F(ParameterStorageTest, WhileArmedItWritesAndCompactsButNeverErases)
{
    Boot boot(flash);
    arm(true);
    // Fill sector 0, then a compaction into the blank spare: allowed while armed.
    for (int i = 1; i <= 12; ++i) {
        boot.set("T_GAIN", ParamValue::from_float(static_cast<float>(i)));
        ASSERT_TRUE(boot.save()) << i;
    }
    ASSERT_EQ(boot.status().active_sector, 1U);
    boot.run(10);
    EXPECT_EQ(flash.read(0, 0), ParameterStorage::magic);  // the old sector is not erased while armed

    // Fill sector 1 too: with no erased spare, changes stay in RAM.
    for (int i = 13; i <= 25; ++i) {
        boot.set("T_GAIN", ParamValue::from_float(static_cast<float>(i)));
        boot.save(5);
    }
    EXPECT_EQ(boot.status().unsaved, 1U);
    EXPECT_EQ(boot.state("T_GAIN"), param_state::unsaved);
    EXPECT_EQ(flash.read(0, 0), ParameterStorage::magic);

    // Disarmed: the old sector is erased, the values compacted into it, the change written.
    arm(false);
    ASSERT_TRUE(boot.save());
    EXPECT_EQ(boot.status().active_sector, 0U);
    Boot after(flash);
    EXPECT_EQ(values.gain, 25.0F);
}

TEST_F(ParameterStorageTest, PublishesItsStatusOnceASecond)
{
    Boot boot(flash);
    vk::core::Subscription<vk::msg::ParameterStatus> status;
    vk::msg::ParameterStatus message{};
    while (status.copy(message)) {  // what earlier tests left
    }
    boot.run(1);
    ASSERT_TRUE(status.copy(message));
    EXPECT_EQ(message.parameter_count, 3U);
    EXPECT_EQ(message.sector_size, sector_size);
    boot.run(100);  // 0.1 s
    EXPECT_FALSE(status.copy(message));
    boot.run(1000);
    EXPECT_TRUE(status.copy(message));
}

}  // namespace
