// SPDX-License-Identifier: GPL-3.0-or-later
// The Vehkernel bootloader core (ADR-0023) against a fake platform: each
// command and its replies, the guards against erasing on noise, the held
// lead words, the firmware check and the boot decision at its boundaries.
#include <vk/bootloader/bootloader.hpp>
#include <vk/bootloader/firmware_check.hpp>
#include <vk/core/app_descriptor.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

using vk::bootloader::Bootloader;
using vk::bootloader::FirmwareCheck;
namespace proto = vk::bootloader::proto;

constexpr std::uint16_t board_id = 22083;
constexpr std::uint32_t region_size = 4096;

// Bytes arrive from `input`; when none is left, time passes as the timeout says.
class FakePlatform {
public:
    int read_byte(std::uint32_t timeout_ms)
    {
        if (input.empty()) {
            now_ms += timeout_ms == 0 ? 1 : timeout_ms;
            return -1;
        }
        const std::uint8_t byte = input.front();
        input.pop_front();
        return byte;
    }
    void write(std::span<const std::uint8_t> bytes) { output.insert(output.end(), bytes.begin(), bytes.end()); }
    void set_baud(std::uint32_t value) { baud = value; }
    void lock_port() { locked = true; }
    [[nodiscard]] std::uint32_t millis() const { return now_ms; }
    void delay_ms(std::uint32_t ms) { now_ms += ms; }
    [[nodiscard]] std::uint32_t firmware_size() const { return static_cast<std::uint32_t>(flash.size() * 4); }
    [[nodiscard]] std::uint32_t read_word(std::uint32_t offset) const { return flash.at(offset / 4); }
    bool erase_firmware()
    {
        ++erases;
        std::ranges::fill(flash, 0xFFFFFFFF);
        return true;
    }
    bool write_words(std::uint32_t offset, std::span<const std::uint32_t> words)
    {
        if (offset / 4 + words.size() > flash.size()) {
            return false;
        }
        for (std::size_t i = 0; i < words.size(); ++i) {
            flash.at(offset / 4 + i) &= words[i];  // NOR Flash only clears bits
        }
        return true;
    }
    [[nodiscard]] std::uint32_t boot_signal() const { return signal; }
    void set_boot_signal(std::uint32_t value) { signal = value; }
    [[nodiscard]] bool stay_requested() const { return stay; }
    [[nodiscard]] std::uint32_t mcu_id() const { return 0x10076413; }
    [[nodiscard]] std::string_view mcu_description() const { return "STM32F40x,Z"; }
    [[nodiscard]] std::uint32_t serial_word(std::uint32_t offset) const { return 0xA0000000U + offset; }
    [[nodiscard]] std::uint32_t otp_word(std::uint32_t offset) const { return 0xB0000000U + offset; }
    void start_firmware() { ++starts; }
    void report(std::string_view line) { reports.emplace_back(line); }

    std::deque<std::uint8_t> input;
    std::vector<std::uint8_t> output;
    std::vector<std::uint32_t> flash = std::vector<std::uint32_t>(region_size / 4, 0xFFFFFFFF);
    std::uint32_t now_ms = 0;
    std::uint32_t baud = 0;
    bool locked = false;
    int erases = 0;
    int starts = 0;
    std::uint32_t signal = 0;
    bool stay = false;
    std::vector<std::string> reports;
};

static_assert(vk::bootloader::Platform<FakePlatform>);

// ArduPilot's crc32_small, independently of the bootloader's.
std::uint32_t reference_crc(const std::vector<std::uint8_t>& bytes, std::size_t begin, std::size_t end)
{
    std::uint32_t crc = 0;
    for (std::size_t i = begin; i < end; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) != 0U ? (crc >> 1U) ^ 0xEDB88320U : crc >> 1U;
        }
    }
    return crc;
}

// A firmware image as the build finalizes it: vectors, a filled descriptor, code.
std::vector<std::uint8_t> make_image(std::uint16_t id = board_id, std::size_t size = 1000)
{
    std::vector<std::uint8_t> image(size);
    for (std::size_t i = 0; i < size; ++i) {
        image[i] = static_cast<std::uint8_t>((i * 37U) + 11U);
    }
    constexpr std::size_t at = 392;  // after 98 vectors
    std::memcpy(&image[at], vk::core::app_descriptor_signature.data(), 8);
    image[at + 24] = 0;
    image[at + 25] = 1;
    image[at + 26] = static_cast<std::uint8_t>(id);
    image[at + 27] = static_cast<std::uint8_t>(id >> 8U);
    const auto put = [&](std::size_t offset, std::uint32_t value) { std::memcpy(&image[offset], &value, 4); };
    put(at + 16, static_cast<std::uint32_t>(size));
    put(at + 20, 0x5212304U);
    put(at + 8, reference_crc(image, 0, at + 8));
    put(at + 12, reference_crc(image, at + 24, size));
    return image;
}

void load(FakePlatform& p, const std::vector<std::uint8_t>& image)
{
    std::ranges::fill(p.flash, 0xFFFFFFFF);
    std::memcpy(p.flash.data(), image.data(), image.size());
}

class BootloaderTest : public ::testing::Test {
protected:
    // Queues the bytes of a command.
    void send(std::initializer_list<std::uint8_t> bytes) { p.input.insert(p.input.end(), bytes); }
    void send(std::span<const std::uint8_t> bytes) { p.input.insert(p.input.end(), bytes.begin(), bytes.end()); }
    void send_word(std::uint32_t word)
    {
        send({static_cast<std::uint8_t>(word), static_cast<std::uint8_t>(word >> 8U),
              static_cast<std::uint8_t>(word >> 16U), static_cast<std::uint8_t>(word >> 24U)});
    }

    // Runs commands until every queued byte is used, then returns and clears the output.
    std::vector<std::uint8_t> exchange()
    {
        while (!p.input.empty()) {
            bootloader.step();
        }
        return std::exchange(p.output, {});
    }

    // GET_SYNC and the GET_DEVICE queries uploader.py's identify() makes.
    void identify()
    {
        send({proto::get_sync, proto::eoc});
        for (const std::uint8_t arg : {proto::device_bl_rev, proto::device_board_id, proto::device_fw_size}) {
            send({proto::get_device, arg, proto::eoc});
        }
        exchange();
    }

    void upload(const std::vector<std::uint8_t>& image)
    {
        send({proto::chip_erase, proto::eoc});
        for (std::size_t at = 0; at < image.size(); at += 252) {
            const std::size_t n = std::min<std::size_t>(252, image.size() - at);
            std::vector<std::uint8_t> chunk(image.begin() + static_cast<std::ptrdiff_t>(at),
                                            image.begin() + static_cast<std::ptrdiff_t>(at + n));
            chunk.resize((n + 3) / 4 * 4, 0xFF);
            send({proto::prog_multi, static_cast<std::uint8_t>(chunk.size())});
            send(chunk);
            send({proto::eoc});
        }
        exchange();
    }

    FakePlatform p;
    Bootloader<FakePlatform> bootloader{p, {.board_id = board_id, .board_revision = 0}};

    static constexpr std::array<std::uint8_t, 2> ok{proto::insync, proto::ok};
    static constexpr std::array<std::uint8_t, 2> invalid{proto::insync, proto::invalid};
};

std::vector<std::uint8_t> reply(std::initializer_list<std::uint32_t> words, std::uint8_t status = proto::ok)
{
    std::vector<std::uint8_t> out;
    for (const std::uint32_t w : words) {
        for (unsigned b = 0; b < 4; ++b) {
            out.push_back(static_cast<std::uint8_t>(w >> (8U * b)));
        }
    }
    out.push_back(proto::insync);
    out.push_back(status);
    return out;
}

TEST_F(BootloaderTest, GetDeviceAnswersTheRevisionBoardAndSize)
{
    bootloader.begin(0);
    send({proto::get_sync, proto::eoc});
    EXPECT_EQ(exchange(), reply({}));
    send({proto::get_device, proto::device_bl_rev, proto::eoc});
    EXPECT_EQ(exchange(), reply({5}));
    send({proto::get_device, proto::device_board_id, proto::eoc});
    EXPECT_EQ(exchange(), reply({board_id}));
    send({proto::get_device, proto::device_board_rev, proto::eoc});
    EXPECT_EQ(exchange(), reply({0}));
    send({proto::get_device, proto::device_fw_size, proto::eoc});
    EXPECT_EQ(exchange(), reply({region_size}));
    send({proto::get_device, proto::device_extf_size, proto::eoc});
    EXPECT_EQ(exchange(), reply({0}));
    send({proto::get_device, 9, proto::eoc});
    EXPECT_EQ(exchange(), reply({}, proto::invalid));
    EXPECT_TRUE(p.locked);
}

TEST_F(BootloaderTest, EraseAndProgramNeedSyncAndGetDeviceFirst)
{
    bootloader.begin(0);
    send({proto::chip_erase, proto::eoc});
    EXPECT_EQ(exchange(), reply({}, proto::invalid));
    send({proto::get_sync, proto::eoc, proto::chip_erase, proto::eoc});
    EXPECT_EQ(exchange(), (std::vector<std::uint8_t>{proto::insync, proto::ok, proto::insync, proto::invalid}));
    send({proto::prog_multi, 4, 1, 2, 3, 4, proto::eoc});
    EXPECT_EQ(exchange().back(), proto::invalid);
    EXPECT_EQ(p.erases, 0);
}

TEST_F(BootloaderTest, AnUploadHoldsTheLeadWordsBackUntilBoot)
{
    bootloader.begin(0);
    identify();
    const std::vector<std::uint8_t> image = make_image();
    upload(image);
    EXPECT_EQ(p.erases, 1);
    for (std::size_t w = 0; w < proto::reserve_lead_words; ++w) {
        EXPECT_EQ(p.flash.at(w), 0xFFFFFFFFU) << "lead word " << w << " written before BOOT";
    }
    EXPECT_EQ(vk::bootloader::check_firmware(p, board_id), FirmwareCheck::BadCrc);

    // GET_CRC already counts the held words, as uploader.py's verify expects.
    std::vector<std::uint8_t> padded = image;
    padded.resize(region_size, 0xFF);
    send({proto::get_crc, proto::eoc});
    EXPECT_EQ(exchange(), reply({reference_crc(padded, 0, padded.size())}));

    send({proto::boot, proto::eoc});
    EXPECT_EQ(bootloader.step(), Bootloader<FakePlatform>::Step::Boot);
    EXPECT_EQ(p.output, (std::vector<std::uint8_t>(ok.begin(), ok.end())));
    EXPECT_EQ(std::memcmp(p.flash.data(), image.data(), image.size()), 0);
    EXPECT_EQ(vk::bootloader::check_firmware(p, board_id), FirmwareCheck::Ok);
}

TEST_F(BootloaderTest, ProgrammingPastTheRegionOrUnalignedIsRefused)
{
    bootloader.begin(0);
    identify();
    send({proto::chip_erase, proto::eoc});
    exchange();
    send({proto::prog_multi, 3, 1, 2, 3, proto::eoc});
    EXPECT_EQ(exchange().back(), proto::invalid);
    for (std::uint32_t at = 0; at < region_size; at += 252) {
        const std::uint8_t n = static_cast<std::uint8_t>(std::min<std::uint32_t>(252, region_size - at));
        send({proto::prog_multi, n});
        send(std::vector<std::uint8_t>(n, 0));
        send({proto::eoc});
    }
    exchange();
    send({proto::prog_multi, 4, 0, 0, 0, 0, proto::eoc});
    EXPECT_EQ(exchange(), reply({}, proto::invalid));
}

TEST_F(BootloaderTest, ReadMultiOtpSerialAndChipAnswer)
{
    load(p, make_image());
    bootloader.begin(0);
    identify();
    send({proto::read_multi, 8, proto::eoc});
    EXPECT_EQ(exchange(), reply({p.flash[0], p.flash[1]}));
    send({proto::get_otp});
    send_word(8);
    send({proto::eoc});
    EXPECT_EQ(exchange(), reply({0xB0000008U}));
    send({proto::get_sn});
    send_word(4);
    send({proto::eoc});
    EXPECT_EQ(exchange(), reply({0xA0000004U}));
    send({proto::get_chip, proto::eoc});
    EXPECT_EQ(exchange(), reply({0x10076413}));
    send({proto::get_chip_des, proto::eoc});
    std::vector<std::uint8_t> expected = {11, 0, 0, 0};
    for (const char c : std::string_view("STM32F40x,Z")) {
        expected.push_back(static_cast<std::uint8_t>(c));
    }
    expected.insert(expected.end(), ok.begin(), ok.end());
    EXPECT_EQ(exchange(), expected);
}

TEST_F(BootloaderTest, SetBaudRepliesFirstThenChangesTheRate)
{
    bootloader.begin(0);
    identify();
    send({proto::set_baud});
    send_word(921600);
    send({proto::eoc});
    EXPECT_EQ(exchange(), (std::vector<std::uint8_t>(ok.begin(), ok.end())));
    EXPECT_EQ(p.baud, 921600U);
}

TEST_F(BootloaderTest, ExternalFlashCommandsAreInvalidAndUnknownBytesIgnored)
{
    bootloader.begin(0);
    send({proto::extf_get_crc, 0x00, 0x7F, 0x55});
    EXPECT_EQ(exchange(), reply({}, proto::invalid));
}

TEST(FirmwareCheckTest, EachFailureIsTold)
{
    FakePlatform p;
    EXPECT_EQ(vk::bootloader::check_firmware(p, board_id), FirmwareCheck::NoDescriptor);
    load(p, make_image());
    EXPECT_EQ(vk::bootloader::check_firmware(p, board_id), FirmwareCheck::Ok);
    EXPECT_EQ(vk::bootloader::check_firmware(p, board_id + 1), FirmwareCheck::BadBoardId);
    std::vector<std::uint8_t> image = make_image();
    image.back() ^= 0xFFU;
    load(p, image);
    EXPECT_EQ(vk::bootloader::check_firmware(p, board_id), FirmwareCheck::BadCrc);
    image = make_image();
    const std::uint32_t too_big = region_size + 4;
    std::memcpy(&image[392 + 16], &too_big, 4);
    load(p, image);
    EXPECT_EQ(vk::bootloader::check_firmware(p, board_id), FirmwareCheck::BadLength);
}

using Decision = Bootloader<FakePlatform>::Decision;

TEST_F(BootloaderTest, BootDecisionFollowsArduPilot)
{
    const std::vector<std::uint8_t> image = make_image();
    load(p, image);
    EXPECT_EQ(bootloader.boot_decision(), (Decision{.start_at_once = false, .window_ms = 1000}));
    p.signal = vk::bootloader::boot_signal_fast;
    EXPECT_EQ(bootloader.boot_decision(), (Decision{.start_at_once = true, .window_ms = 0}));
    EXPECT_EQ(p.signal, 0U) << "the request is cleared once read";
    p.signal = vk::bootloader::boot_signal_hold;
    EXPECT_EQ(bootloader.boot_decision(), (Decision{.start_at_once = false, .window_ms = 0}));
    p.stay = true;
    EXPECT_EQ(bootloader.boot_decision(), (Decision{.start_at_once = false, .window_ms = 0}));
    p.stay = false;
    p.signal = vk::bootloader::boot_signal_fast;
    p.flash.back() = 0;  // the image changed: its CRC fails
    p.flash.at(100) ^= 1U;
    EXPECT_EQ(bootloader.boot_decision(), (Decision{.start_at_once = false, .window_ms = 0}));
    EXPECT_EQ(p.reports.back(), "stay: bad_crc");
}

TEST_F(BootloaderTest, AValidFirmwareStartsAfterTheWindowAndNotBefore)
{
    load(p, make_image());
    bootloader.begin(vk::bootloader::boot_window_ms);
    while (p.now_ms < vk::bootloader::boot_window_ms - 1) {
        ASSERT_EQ(bootloader.step(), Bootloader<FakePlatform>::Step::Continue);
    }
    while (bootloader.step() == Bootloader<FakePlatform>::Step::Continue) {
    }
    EXPECT_EQ(p.now_ms, vk::bootloader::boot_window_ms);
    bootloader.try_start();
    EXPECT_EQ(p.starts, 1);
}

TEST_F(BootloaderTest, AConversationCancelsTheWindow)
{
    bootloader.begin(vk::bootloader::boot_window_ms);
    identify();
    p.now_ms += 10 * vk::bootloader::boot_window_ms;
    EXPECT_EQ(bootloader.step(), Bootloader<FakePlatform>::Step::Continue);
}

TEST_F(BootloaderTest, NoiseDoesNotCancelTheWindow)
{
    bootloader.begin(vk::bootloader::boot_window_ms);
    send({proto::get_device, 2, 0x00});  // a GET_DEVICE without its EOC
    exchange();
    p.now_ms += vk::bootloader::boot_window_ms;
    EXPECT_EQ(bootloader.step(), Bootloader<FakePlatform>::Step::TimedOut);
}

TEST_F(BootloaderTest, AnInvalidOrUnfinishedImageNeverStarts)
{
    bootloader.try_start();
    EXPECT_EQ(p.starts, 0);
    // An upload cut off before BOOT: everything but the held lead words.
    const std::vector<std::uint8_t> image = make_image();
    bootloader.begin(0);
    identify();
    upload(image);
    bootloader.try_start();
    EXPECT_EQ(p.starts, 0);
    EXPECT_EQ(p.reports.back(), "start refused: bad_crc");
    load(p, image);
    bootloader.try_start();
    EXPECT_EQ(p.starts, 1);
}

}  // namespace
