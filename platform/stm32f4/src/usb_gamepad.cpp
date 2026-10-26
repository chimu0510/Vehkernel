// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/platform/platform.hpp>

#include <vk/core/bus.hpp>
#include <vk/core/critical_section.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/msg/enums.hpp>
#include <vk/msg/manual_control_input.hpp>
#include <vk/platform/gpio.hpp>
#include <vk/platform/xinput.hpp>

#include <board.hpp>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "tusb.h"
// After tusb.h, which declares what it uses.
#include "host/usbh_pvt.h"

#include <array>
#include <cstdint>
#include <span>

namespace vk::platform {
namespace {

// Below FreeRTOS's syscall priority (5), since TinyUSB posts to a queue from
// its interrupt; above nothing that must keep time (ADR-0005).
constexpr std::uint32_t otg_fs_irq_priority = 6;
// Between wq:ctrl and wq:io: the interrupt does the time-critical work, and
// the control cycle keeps time.
constexpr UBaseType_t usb_task_priority = configMAX_PRIORITIES - 3;

StaticTask_t usb_tcb;
std::array<StackType_t, 2 * configMINIMAL_STACK_SIZE> usb_stack;

// The one XInput pad TinyUSB has enumerated, touched only from the USB task.
struct Pad {
    std::uint8_t daddr = 0;
    std::uint8_t ep_in = 0;
    std::uint8_t ep_out = 0;
    std::uint16_t in_size = 0;
    bool open = false;
};
Pad pad;
CFG_TUH_MEM_ALIGN std::array<std::uint8_t, 64> in_buffer;
CFG_TUH_MEM_ALIGN std::array<std::uint8_t, 8> out_buffer;

// What the publisher reads, guarded by the critical section.
struct Shared {
    bool connected = false;
    xinput::State state{};
};
Shared shared;

void set_shared(bool connected, const xinput::State& state)
{
    const core::CriticalSection lock;
    shared = {.connected = connected, .state = state};
}

bool receive()
{
    if (!usbh_edpt_claim(pad.daddr, pad.ep_in)) {
        return false;
    }
    const auto size = static_cast<std::uint16_t>(std::min<std::size_t>(pad.in_size, in_buffer.size()));
    if (!usbh_edpt_xfer(pad.daddr, pad.ep_in, in_buffer.data(), size)) {
        usbh_edpt_release(pad.daddr, pad.ep_in);
        return false;
    }
    return true;
}

// The TinyUSB class driver for an XInput gamepad interface.
bool xinput_init()
{
    return true;
}

bool xinput_deinit()
{
    return true;
}

std::uint16_t xinput_open(std::uint8_t /*rhport*/, std::uint8_t daddr, const tusb_desc_interface_t* itf,
                          std::uint16_t max_len)
{
    if (itf->bInterfaceClass != xinput::interface_class || itf->bInterfaceSubClass != xinput::interface_subclass ||
        itf->bInterfaceProtocol != xinput::interface_protocol || pad.open) {
        return 0;
    }
    pad = {.daddr = daddr};
    // The interface, its vendor descriptor and its endpoints, up to the next interface.
    auto length = static_cast<std::uint16_t>(tu_desc_len(itf));
    const auto* p = tu_desc_next(itf);
    while (length < max_len && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
        if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
            const auto* ep = reinterpret_cast<const tusb_desc_endpoint_t*>(p);
            if (!tuh_edpt_open(daddr, ep)) {
                return 0;
            }
            if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
                pad.ep_in = ep->bEndpointAddress;
                pad.in_size = tu_edpt_packet_size(ep);
            } else {
                pad.ep_out = ep->bEndpointAddress;
            }
        }
        length = static_cast<std::uint16_t>(length + tu_desc_len(p));
        p = tu_desc_next(p);
    }
    if (pad.ep_in == 0) {
        return 0;
    }
    pad.open = true;
    return length;
}

bool xinput_set_config(std::uint8_t daddr, std::uint8_t itf_num)
{
    if (pad.ep_out != 0 && usbh_edpt_claim(daddr, pad.ep_out)) {
        std::copy(xinput::led_player_1.begin(), xinput::led_player_1.end(), out_buffer.begin());
        if (!usbh_edpt_xfer(daddr, pad.ep_out, out_buffer.data(), xinput::led_player_1.size())) {
            usbh_edpt_release(daddr, pad.ep_out);
        }
    }
    set_shared(true, xinput::State{});
    static_cast<void>(receive());
    usbh_driver_set_config_complete(daddr, itf_num);
    return true;
}

bool xinput_xfer_cb(std::uint8_t /*daddr*/, std::uint8_t ep_addr, xfer_result_t result, std::uint32_t xferred_bytes)
{
    if (ep_addr != pad.ep_in) {
        return true;  // the LED command went out
    }
    if (result == XFER_RESULT_SUCCESS) {
        const auto report = std::span<const std::uint8_t>(in_buffer).first(
            std::min<std::size_t>(xferred_bytes, in_buffer.size()));
        if (const auto state = xinput::parse(report)) {
            set_shared(true, *state);
        }
    }
    return receive();
}

void xinput_close(std::uint8_t daddr)
{
    if (pad.open && pad.daddr == daddr) {
        pad = {};
        set_shared(false, xinput::State{});
    }
}

const std::array<usbh_class_driver_t, 1> drivers{{{
    .name = "XINPUT",
    .init = &xinput_init,
    .deinit = &xinput_deinit,
    .open = &xinput_open,
    .set_config = &xinput_set_config,
    .xfer_cb = &xinput_xfer_cb,
    .close = &xinput_close,
}}};

void usb_task(void* /*unused*/)
{
    const tusb_rhport_init_t host{.role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_FULL};
    static_cast<void>(tusb_rhport_init(BOARD_TUH_RHPORT, &host));
    for (;;) {
        tuh_task();  // waits for the stack's events
    }
}

// Publishes the pad's state as a gamepad driver does (spec 0010): its latest
// state 50 times a second while connected, since an XInput pad sends only
// changes, and once when it goes.
class GamepadPublisher final : public core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 20'000;

    explicit GamepadPublisher(core::WorkQueue& queue) : WorkItem("usb_gamepad", queue) {}

    void start() { schedule_periodic(interval_us); }

protected:
    void run() override
    {
        Shared now{};
        {
            const core::CriticalSection lock;
            now = shared;
        }
        if (!now.connected && !was_connected_) {
            return;
        }
        was_connected_ = now.connected;
        msg::ManualControlInput input{};
        input.timestamp = queue().now_us();
        input.source = static_cast<std::uint8_t>(msg::ManualControlSource::UsbGamepad);
        input.connected = now.connected;
        if (now.connected) {
            input.sticks = now.state.sticks;
            input.triggers = now.state.triggers;
            input.buttons = now.state.buttons;
        }
        publication_.publish(input);
    }

private:
    core::Publication<msg::ManualControlInput> publication_;
    bool was_connected_ = false;
};

core::StaticInstance<GamepadPublisher> publisher;

}  // namespace

void start_usb_gamepad(core::WorkQueue& queue)
{
    detail::configure_af_pin<board::pins::USB_DM>(LL_GPIO_PULL_NO);
    detail::configure_af_pin<board::pins::USB_DP>(LL_GPIO_PULL_NO);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_OTGFS);
    NVIC_SetPriority(OTG_FS_IRQn, otg_fs_irq_priority);
    xTaskCreateStatic(usb_task, "usb", usb_stack.size(), nullptr, usb_task_priority, usb_stack.data(), &usb_tcb);
    publisher.emplace(queue);
    publisher->start();
}

}  // namespace vk::platform

extern "C" {

// CMSIS's system clock variable, which TinyUSB's DWC2 driver reads to set the
// core's turnaround time. Vehkernel does not use ST's system_stm32f4xx.c, so
// it is defined here from the board definition's clock tree (ADR-0011).
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables): CMSIS declares it writable.
std::uint32_t SystemCoreClock = vk::board::clock::sysclk_hz;

// TinyUSB asks for the application's class drivers when the host stack starts.
usbh_class_driver_t const* usbh_app_driver_get_cb(std::uint8_t* driver_count)
{
    *driver_count = static_cast<std::uint8_t>(vk::platform::drivers.size());
    return vk::platform::drivers.data();
}

void OTG_FS_IRQHandler()
{
    tuh_int_handler(BOARD_TUH_RHPORT, true);
}

}  // extern "C"
