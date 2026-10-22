#include <vk/local_display/local_menu.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>

namespace vk::local_display {

namespace {

constexpr std::size_t list_lines = 6;  // lines 1 to 6 hold the list

std::string_view result_name(core::ParamResult result)
{
    switch (result) {
    case core::ParamResult::Ok:
        return "SAVED";
    case core::ParamResult::UnknownName:
        return "UNKNOWN";
    case core::ParamResult::WrongType:
        return "WRONG_TYPE";
    case core::ParamResult::OutOfRange:
        return "OUT_OF_RANGE";
    case core::ParamResult::LockedWhileArmed:
        return "LOCKED";
    }
    return "?";
}

class Text {
public:
    Text& add(std::string_view text)
    {
        for (const char c : text) {
            if (out_.size < out_.text.size()) {
                out_.text.at(out_.size++) = c;
            }
        }
        return *this;
    }

    Text& add(std::uint64_t value)
    {
        std::array<char, 20> digits{};
        std::size_t count = 0;
        do {
            digits.at(count++) = static_cast<char>('0' + (value % 10));
            value /= 10;
        } while (value != 0);
        while (count > 0) {
            add(std::string_view(&digits.at(--count), 1));
        }
        return *this;
    }

    [[nodiscard]] const ValueText& result() const { return out_; }

private:
    ValueText out_;
};

// Decimals a step needs, from 0 to 4: 0.1 needs 1, 0.01 needs 2.
unsigned decimals_of(float step)
{
    unsigned decimals = 0;
    auto scaled = static_cast<double>(step);
    while (decimals < 4 && std::fabs(scaled - std::round(scaled)) > 1e-6 * std::fabs(scaled)) {
        scaled *= 10.0;
        ++decimals;
    }
    return decimals;
}

std::ptrdiff_t enum_position(const core::ParamInfo& info, std::int32_t value)
{
    const auto found = std::ranges::find(info.enum_values, value);
    return found == info.enum_values.end() ? -1 : found - info.enum_values.begin();
}

}  // namespace

ValueText format_value(const core::ParamInfo& info, const core::MenuParam& entry, core::ParamValue value)
{
    Text text;
    if (!info.enum_values.empty()) {
        const std::ptrdiff_t position = enum_position(info, value.as_int());
        if (position >= 0 && static_cast<std::size_t>(position) < entry.value_names.size()) {
            return text.add(entry.value_names[static_cast<std::size_t>(position)]).result();
        }
    }
    if (info.type == core::ParamType::Int32) {
        const std::int32_t number = value.as_int();
        if (number < 0) {
            text.add("-");
        }
        return text.add(static_cast<std::uint64_t>(number < 0 ? -static_cast<std::int64_t>(number) : number)).result();
    }
    const unsigned decimals = decimals_of(entry.step.as_float());
    // A table rather than std::pow, which would cost the firmware 3 KB of Flash.
    constexpr std::array<double, 5> scales{1.0, 10.0, 100.0, 1000.0, 10000.0};
    const double scale = scales[decimals];
    const double rounded = std::round(static_cast<double>(value.as_float()) * scale);
    if (!std::isfinite(rounded)) {
        return text.add("---").result();
    }
    if (rounded < 0.0) {
        text.add("-");
    }
    const auto units = static_cast<std::uint64_t>(std::fabs(rounded));
    const auto whole_scale = static_cast<std::uint64_t>(scale);
    text.add(units / whole_scale);
    if (decimals > 0) {
        text.add(".");
        const std::uint64_t fraction = units % whole_scale;
        for (std::uint64_t digit = whole_scale / 10; digit > 0; digit /= 10) {
            text.add((fraction / digit) % 10);
        }
    }
    return text.result();
}

LocalMenu::LocalMenu(core::Parameters& parameters, std::span<const core::MenuParam> menu)
    : parameters_(parameters), menu_(menu.first(std::min(menu.size(), max_entries)))
{
    for (std::size_t i = 0; i < menu_.size(); ++i) {
        boot_values_.at(i) = parameters_.get(menu_[i].index);
    }
}

std::uint16_t LocalMenu::item() const
{
    if (state_ == MenuState::Closed || position_ >= menu_.size()) {
        return no_item;
    }
    return menu_[position_].index;
}

bool LocalMenu::needs_reboot(std::size_t position) const
{
    if (position >= menu_.size()) {
        return false;
    }
    const core::ParamInfo& info = parameters_.info(menu_[position].index);
    return (info.flags & core::param_flags::reboot_required) != 0 &&
           parameters_.get(menu_[position].index) != boot_values_.at(position);
}

bool LocalMenu::reboot_required() const
{
    for (std::size_t i = 0; i < menu_.size(); ++i) {
        if (needs_reboot(i)) {
            return true;
        }
    }
    return false;
}

void LocalMenu::on_key(KeyEvent event, std::uint64_t now_us, bool armed)
{
    if (event == KeyEvent::None) {
        return;
    }
    last_key_us_ = now_us;
    switch (state_) {
    case MenuState::Closed:
        if (event == KeyEvent::Short) {
            page_ = next_page(page_);
        } else if (event == KeyEvent::Long && !armed) {
            state_ = MenuState::List;
            position_ = 0;
            last_result_.reset();
        }
        return;
    case MenuState::List:
        if (event != KeyEvent::Repeat) {
            last_result_.reset();
        }
        list_key(event);
        return;
    case MenuState::Edit:
        edit_key(event);
        return;
    }
}

void LocalMenu::list_key(KeyEvent event)
{
    if (event == KeyEvent::Short) {
        position_ = (position_ + 1) % (menu_.size() + 1);
    } else if (event == KeyEvent::Long) {
        if (position_ >= menu_.size()) {
            close();
            return;
        }
        state_ = MenuState::Edit;
        action_ = MenuAction::Increase;
        staged_ = parameters_.get(entry().index);
    }
}

void LocalMenu::edit_key(KeyEvent event)
{
    if (event == KeyEvent::Short) {
        action_ = static_cast<MenuAction>((static_cast<std::size_t>(action_) + 1) % menu_action_count);
        return;
    }
    const bool stepping = action_ == MenuAction::Increase || action_ == MenuAction::Decrease;
    if (event == KeyEvent::Repeat) {
        // Only + and - repeat; Save and the others run once per press.
        if (stepping) {
            step(action_ == MenuAction::Increase ? 1 : -1);
        }
        return;
    }
    switch (action_) {
    case MenuAction::Increase:
        step(1);
        return;
    case MenuAction::Decrease:
        step(-1);
        return;
    case MenuAction::Default:
        staged_ = parameters_.info(entry().index).default_value;
        return;
    case MenuAction::Save:
        last_result_ = parameters_.set(entry().index, staged_);
        last_parameter_ = entry().index;
        state_ = MenuState::List;
        return;
    case MenuAction::Cancel:
        state_ = MenuState::List;
        return;
    }
}

void LocalMenu::step(int direction)
{
    const core::ParamInfo& info = parameters_.info(entry().index);
    if (!info.enum_values.empty()) {
        const std::ptrdiff_t last = static_cast<std::ptrdiff_t>(info.enum_values.size()) - 1;
        const std::ptrdiff_t position = std::clamp<std::ptrdiff_t>(
            std::max<std::ptrdiff_t>(enum_position(info, staged_.as_int()), 0) + direction, 0, last);
        staged_ = core::ParamValue::from_int(info.enum_values[static_cast<std::size_t>(position)]);
        return;
    }
    if (info.type == core::ParamType::Int32) {
        const std::int64_t next = static_cast<std::int64_t>(staged_.as_int()) +
                                  (static_cast<std::int64_t>(direction) * entry().step.as_int());
        staged_ = core::ParamValue::from_int(static_cast<std::int32_t>(
            std::clamp<std::int64_t>(next, info.min.as_int(), info.max.as_int())));
        return;
    }
    // Numbers stay on the grid of steps from the minimum, so 0.1 + 0.1 shows 0.2.
    const double minimum = info.min.as_float();
    const double size = entry().step.as_float();
    const double steps = std::round((static_cast<double>(staged_.as_float()) - minimum) / size) + direction;
    const double next = std::clamp(minimum + (steps * size), minimum, static_cast<double>(info.max.as_float()));
    staged_ = core::ParamValue::from_float(static_cast<float>(next));
}

void LocalMenu::tick(std::uint64_t now_us, bool armed)
{
    if (state_ == MenuState::Closed) {
        return;
    }
    if (armed || now_us - last_key_us_ >= idle_close_us) {
        close();
    }
}

void LocalMenu::close()
{
    state_ = MenuState::Closed;
    position_ = 0;
    action_ = MenuAction::Increase;
}

void render_menu(const LocalMenu& menu, TextScreen& screen)
{
    screen.clear();
    const auto entries = menu.entries();
    const core::Parameters& parameters = menu.parameters();
    if (menu.state() == MenuState::List) {
        screen.put(0, 0, "MENU");
        if (menu.last_result()) {
            screen.put_right(0, result_name(*menu.last_result()));
        }
        const std::size_t count = entries.size() + 1;  // and Exit
        const std::size_t first = menu.position() < list_lines ? 0 : menu.position() + 1 - list_lines;
        for (std::size_t line = 0; line < list_lines && first + line < count; ++line) {
            const std::size_t position = first + line;
            screen.put(1 + line, 0, position == menu.position() ? ">" : " ");
            screen.put(1 + line, 1,
                       position < entries.size() ? parameters.info(entries[position].index).name : "EXIT");
        }
        if (menu.position() < entries.size()) {
            const core::MenuParam& entry = entries[menu.position()];
            Text value;
            value.add("= ").add(format_value(parameters.info(entry.index), entry, parameters.get(entry.index)).view());
            if (menu.needs_reboot(menu.position())) {
                value.add(" REBOOT");
            }
            screen.put(7, 0, value.result().view());
        } else {
            screen.put(7, 0, "LONG PRESS: CLOSE");
        }
        return;
    }
    if (menu.state() != MenuState::Edit) {
        return;
    }
    const core::MenuParam& entry = entries[menu.position()];
    const core::ParamInfo& info = parameters.info(entry.index);
    screen.put(0, 0, "EDIT");
    if (menu.needs_reboot(menu.position())) {
        screen.put_right(0, "REBOOT");
    }
    screen.put(1, 0, info.name);
    screen.put(2, 0, Text().add("NOW ").add(format_value(info, entry, parameters.get(entry.index)).view()).result().view());
    screen.put(3, 0, Text().add("NEW ").add(format_value(info, entry, menu.staged()).view()).result().view());
    // The actions, with carets under the one a long press runs.
    screen.put(5, 0, "+ - DEF SAVE CANCEL");
    static constexpr std::array<std::pair<std::size_t, std::string_view>, menu_action_count> carets{
        {{0, "^"}, {2, "^"}, {4, "^^^"}, {8, "^^^^"}, {13, "^^^^^^"}}};
    const auto& caret = carets.at(static_cast<std::size_t>(menu.action()));
    screen.put(6, caret.first, caret.second);
    screen.put(7, 0, "SHORT: NEXT LONG: DO");
}

}  // namespace vk::local_display
