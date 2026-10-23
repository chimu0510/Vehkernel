// SPDX-License-Identifier: GPL-3.0-or-later
// The local menu (ADR-0022): the user key's presses become page changes, and
// while disarmed, edits of the menu parameters.
//
// Closed, a short press switches pages and a long press opens the menu if the
// vehicle is disarmed. The list holds the menu parameters, then Exit; a
// short press moves to the next item and a long press edits it, or on Exit
// closes the menu. An edit stages a value: a short press moves to the next
// action and a long press runs it. + and - step the staged value by its
// menu step within the range, or to the next or previous enum value, and
// repeat while the key stays held; Default stages the default; Save writes
// the staged value through the parameter service, as a Link does, and
// returns to the list with the result; Cancel returns without writing.
//
// The menu closes, discarding a staged edit, after 30 s without a key press,
// and at once when the vehicle is armed, which only a race between an arm
// request and a long press can cause, since arming is refused while it is
// open. It never arms, changes the mode or touches the emergency stop. It
// uses neither the bus nor FreeRTOS, so it is tested on the development host.
#pragma once

#include <vk/core/parameters.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/local_display/user_key.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace vk::local_display {

enum class MenuState : std::uint8_t {
    Closed = 0,
    List = 1,
    Edit = 2,
};

enum class MenuAction : std::uint8_t {
    Increase,
    Decrease,
    Default,
    Save,
    Cancel,
};
inline constexpr std::size_t menu_action_count = 5;

class LocalMenu {
public:
    static constexpr std::uint64_t idle_close_us = 30'000'000;  // a constant (ADR-0022)
    static constexpr std::uint16_t no_item = 0xFFFF;
    static constexpr std::uint8_t no_result = 0xFF;

    // `menu` lists the menu parameters of `parameters`' table, in table order.
    LocalMenu(core::Parameters& parameters, std::span<const core::MenuParam> menu);

    // A key event at `now_us`, with the vehicle's arming state.
    void on_key(KeyEvent event, std::uint64_t now_us, bool armed);
    // Every run: closes the menu when idle for too long or when armed.
    void tick(std::uint64_t now_us, bool armed);

    [[nodiscard]] MenuState state() const { return state_; }
    [[nodiscard]] Page page() const { return page_; }
    // The list position: 0 to the number of menu parameters, which is Exit.
    [[nodiscard]] std::size_t position() const { return position_; }
    // The parameter index of the item under the cursor, or no_item for Exit or closed.
    [[nodiscard]] std::uint16_t item() const;
    [[nodiscard]] MenuAction action() const { return action_; }
    [[nodiscard]] core::ParamValue staged() const { return staged_; }
    // The result of the last Save and the parameter it wrote, until the next key press.
    [[nodiscard]] std::optional<core::ParamResult> last_result() const { return last_result_; }
    [[nodiscard]] std::uint16_t last_parameter() const { return last_parameter_; }
    // Whether a reboot_required menu parameter differs from its value when the menu was made.
    [[nodiscard]] bool needs_reboot(std::size_t position) const;
    [[nodiscard]] bool reboot_required() const;

    [[nodiscard]] std::span<const core::MenuParam> entries() const { return menu_; }
    [[nodiscard]] const core::Parameters& parameters() const { return parameters_; }

private:
    static constexpr std::size_t max_entries = 32;

    void close();
    void list_key(KeyEvent event);
    void edit_key(KeyEvent event);
    void step(int direction);
    [[nodiscard]] const core::MenuParam& entry() const { return menu_[position_]; }

    core::Parameters& parameters_;
    std::span<const core::MenuParam> menu_;
    std::array<core::ParamValue, max_entries> boot_values_{};
    MenuState state_ = MenuState::Closed;
    Page page_ = Page::Status;
    std::size_t position_ = 0;
    MenuAction action_ = MenuAction::Increase;
    core::ParamValue staged_{};
    std::optional<core::ParamResult> last_result_;
    std::uint16_t last_parameter_ = no_item;
    std::uint64_t last_key_us_ = 0;
};

// The menu, on the local display while it is open.
void render_menu(const LocalMenu& menu, TextScreen& screen);

// A parameter value as the menu shows it: an enum by name, a number with as
// many decimals as its menu step needs.
struct ValueText {
    std::array<char, TextScreen::columns> text{};
    std::size_t size = 0;

    [[nodiscard]] std::string_view view() const { return {text.data(), size}; }
};
ValueText format_value(const core::ParamInfo& info, const core::MenuParam& entry, core::ParamValue value);

}  // namespace vk::local_display
