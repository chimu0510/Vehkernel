// SSD1306 display driver (ADR-0009, ADR-0022).
//
// Drives a 128 x 64 SSD1306 over the write-only display bus contract. It is
// not a work item: the local display calls update() from its own work item,
// with the time, and draws into frame(). update() holds the controller in
// reset for 100 ms, sends the vendor firmware's initialisation sequence and
// then writes only the pages of the frame that changed since they were last
// written, at most `max_pages` per call, so each call's busy-wait on a
// software bus stays bounded. The bus cannot answer, so the driver never
// knows whether a display is fitted, and nothing depends on it.
#pragma once

#include <vk/hal/display_bus.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::drivers {

// A 128 x 64 monochrome image, x to the right and y down from the top left,
// stored as 8 pages of 8 rows; bit n of a byte is row 8 * page + n.
struct MonoFrame {
    static constexpr std::size_t width = 128;
    static constexpr std::size_t height = 64;
    static constexpr std::size_t pages = height / 8;

    std::array<std::array<std::uint8_t, width>, pages> bytes{};

    void clear() { bytes = {}; }
    void set(std::size_t x, std::size_t y, bool lit);
    [[nodiscard]] bool get(std::size_t x, std::size_t y) const;

    friend bool operator==(const MonoFrame&, const MonoFrame&) = default;
};

class Ssd1306 {
public:
    static constexpr std::uint64_t reset_time_us = 100'000;  // as the vendor firmware holds it
    static constexpr std::size_t default_max_pages = 2;

    enum class State : std::uint8_t {
        Idle,       // before the first update()
        Resetting,  // reset line held active
        Ready,      // initialised; pages are written as they change
    };

    explicit Ssd1306(const hal::DisplayDevice& device) : device_(device) {}

    // Advances the reset and initialisation and writes changed pages.
    // Returns how many pages it wrote.
    std::size_t update(std::uint64_t now_us, std::size_t max_pages = default_max_pages);

    [[nodiscard]] MonoFrame& frame() { return frame_; }
    [[nodiscard]] State state() const { return state_; }

private:
    void initialise();
    void write_page(std::size_t page);

    hal::DisplayDevice device_;
    State state_ = State::Idle;
    std::uint64_t reset_until_us_ = 0;
    MonoFrame frame_;
    MonoFrame shown_;                       // what the controller holds
    std::array<bool, MonoFrame::pages> stale_{};  // pages whose shown_ is unknown
    std::size_t next_page_ = 0;             // where the round-robin scan resumes
};

}  // namespace vk::drivers
