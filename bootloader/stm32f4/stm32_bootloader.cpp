// SPDX-License-Identifier: GPL-3.0-or-later
// The Vehkernel bootloader on an STM32F4 board (ADR-0023), in the BOOT region
// (ADR-0020): bare metal, without FreeRTOS or interrupts other than SysTick.
//
// - The clock tree is the board definition's, set up by the platform's own
//   code; SysTick counts milliseconds.
// - The ports are the board's bootloader ports, polled, at 115200 8N1 as
//   ArduPilot's BOOTLOADER_BAUDRATE and uploader.py until SET_BAUD. The first
//   port a command arrives on answers, and after the first good command it
//   is the only one listened to.
// - Flash goes through the platform's internal Flash code, inside the
//   FIRMWARE region only (FirmwareFlash).
// - The firmware starts as the boot stub that came before this bootloader
//   started it: VTOR, the stack pointer, then the reset handler, after the
//   clocks and peripherals are back as they are after reset. A reset
//   handler outside the FIRMWARE region is refused, as in ArduPilot's
//   jump_to_app.
// - Of the RTC backup registers it reads and clears register 0 only, the
//   firmware's request. RCC_CSR's reset flags and the fault record in
//   registers 1 and 2 are left for the firmware to report (ADR-0005,
//   ADR-0024). It never starts the IWDG: a reset stops the firmware's, so the
//   bootloader runs without it.
#include <vk/bootloader/bootloader.hpp>
#include <vk/platform/firmware_flash.hpp>
#include <vk/platform/gpio.hpp>
#include <vk/platform/internal_flash.hpp>
#include <vk/platform/usart_baud.hpp>

#include "../../platform/stm32f4/src/internal.hpp"

#include <board.hpp>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

// The vector table is in vectors.c; these are its handlers.
extern "C" {
extern std::uint32_t _sidata[], _sdata[], _edata[], _sbss[], _ebss[];
void vk_bootloader_reset();
[[noreturn]] void vk_bootloader_fault();
void SysTick_Handler();
}

namespace {

namespace board = vk::board;

constexpr std::uint32_t bootloader_baud = 115200;  // BOOTLOADER_BAUDRATE

// NOLINTBEGIN(performance-no-int-to-ptr): peripheral registers and Flash live at fixed addresses.
USART_TypeDef* usart_at(std::uint32_t address)
{
    return reinterpret_cast<USART_TypeDef*>(address);
}

const volatile std::uint32_t* word_at(std::uint32_t address)
{
    return reinterpret_cast<const volatile std::uint32_t*>(address);
}
// NOLINTEND(performance-no-int-to-ptr)

struct InternalController {
    static bool erase_sector(std::uint8_t sector) { return vk::platform::detail::flash_erase_sector(sector); }
    static vk::hal::FlashResult program_word(std::uint32_t address, std::uint32_t word)
    {
        return vk::platform::detail::flash_program_word(address, word);
    }
    static std::uint32_t read_word(std::uint32_t address) { return *word_at(address); }
};

using Firmware = vk::platform::FirmwareFlash<board::flash::FIRMWARE, InternalController>;

// One of the board's bootloader ports.
struct Uart {
    std::uint32_t address;
    std::uint32_t pclk_hz;
};

template <class Port>
Uart configure_port()
{
    using Registers = typename Port::registers;
    static_assert(Registers::kind == "usart", "a bootloader port needs a USART or UART");
    static_assert(requires { typename Port::tx; typename Port::rx; }, "a bootloader port needs TX and RX");
    constexpr std::uint32_t pclk = Registers::bus_clock == "PCLK2" ? board::clock::apb2_hz : board::clock::apb1_hz;
    static_assert(vk::platform::usart_baud_reachable(pclk, bootloader_baud));

    switch (Registers::address) {
    case USART1_BASE: LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1); break;
    case USART2_BASE: LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2); break;
    case USART3_BASE: LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3); break;
    case UART4_BASE: LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4); break;
    case UART5_BASE: LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5); break;
    case USART6_BASE: LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6); break;
    default: break;
    }
    vk::platform::detail::configure_af_pin<typename Port::tx>(LL_GPIO_PULL_UP);
    vk::platform::detail::configure_af_pin<typename Port::rx>(LL_GPIO_PULL_UP);  // an unplugged line idles high

    USART_TypeDef* const usart = usart_at(Registers::address);
    usart->CR1 = 0;
    usart->CR2 = 0;  // 1 stop bit
    usart->CR3 = 0;
    usart->BRR = vk::platform::usart_brr(pclk, bootloader_baud);
    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;  // 8 data bits, no parity
    return {.address = Registers::address, .pclk_hz = pclk};
}

template <class... Ports>
struct PortTable {
    static std::array<Uart, sizeof...(Ports)> configure() { return {configure_port<Ports>()...}; }
};

using Ports = board::bootloader::port_list<PortTable>;

// The board's stay-in-bootloader input, if it has one.
template <class Pin>
struct StayInput {
    static void init() { vk::platform::InputLine<Pin>::init(); }
    static bool active() { return vk::platform::InputLine<Pin>::is_high() == board::bootloader::stay_active_high; }
};

template <>
struct StayInput<void> {
    static void init() {}
    static bool active() { return false; }
};

using Stay = StayInput<board::bootloader::stay_pin>;

volatile std::uint32_t milliseconds = 0;

class Stm32Platform {
public:
    Stm32Platform() : ports_(Ports::configure()) { Stay::init(); }

    int read_byte(std::uint32_t timeout_ms)
    {
        const std::uint32_t start = millis();
        do {
            for (std::size_t i = 0; i < ports_.size(); ++i) {
                if (locked_ && i != active_) {
                    continue;
                }
                USART_TypeDef* const usart = usart_at(ports_.at(i).address);
                // Reading SR then DR clears an overrun as well.
                if ((usart->SR & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
                    const auto byte = static_cast<std::uint8_t>(usart->DR);
                    active_ = i;
                    return byte;
                }
            }
        } while (millis() - start < timeout_ms);
        return -1;
    }

    void write(std::span<const std::uint8_t> bytes)
    {
        USART_TypeDef* const usart = usart_at(ports_.at(active_).address);
        for (const std::uint8_t byte : bytes) {
            while ((usart->SR & USART_SR_TXE) == 0U) {
            }
            usart->DR = byte;
        }
    }

    void set_baud(std::uint32_t baud)
    {
        const Uart& port = ports_.at(active_);
        if (vk::platform::usart_baud_reachable(port.pclk_hz, baud)) {
            usart_at(port.address)->BRR = vk::platform::usart_brr(port.pclk_hz, baud);
        }
    }

    void lock_port() { locked_ = true; }

    [[nodiscard]] std::uint32_t millis() const { return milliseconds; }

    void delay_ms(std::uint32_t ms)
    {
        const std::uint32_t start = millis();
        while (millis() - start < ms) {
        }
    }

    [[nodiscard]] std::uint32_t firmware_size() const { return Firmware::size(); }
    [[nodiscard]] std::uint32_t read_word(std::uint32_t offset) const { return Firmware::read(offset); }
    bool erase_firmware() { return Firmware::erase(); }
    bool write_words(std::uint32_t offset, std::span<const std::uint32_t> words)
    {
        return Firmware::write(offset, words);
    }

    // RTC backup register 0, where the firmware leaves its request
    // (platform/stm32f4/src/reset.cpp).
    [[nodiscard]] std::uint32_t boot_signal() const { return RTC->BKP0R; }

    // Unlocked as the firmware does it, with the RTC clocked from the LSI if
    // nothing clocks it yet.
    void set_boot_signal(std::uint32_t value)
    {
        if (RTC->BKP0R == value) {
            return;
        }
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_DBP;
        if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0U) {
            RCC->BDCR |= RCC_BDCR_RTCSEL_1;  // LSI
            RCC->BDCR |= RCC_BDCR_RTCEN;
        }
        RTC->BKP0R = value;
        PWR->CR &= ~PWR_CR_DBP;
    }

    [[nodiscard]] bool stay_requested() const { return Stay::active(); }

    [[nodiscard]] std::uint32_t mcu_id() const { return DBGMCU->IDCODE; }

    // As ArduPilot's get_mcu_desc for the STM32F4 (mcu_f4.h): "<family>,<revision>".
    [[nodiscard]] std::string_view mcu_description() const
    {
        const std::uint32_t idcode = DBGMCU->IDCODE;
        const std::string_view family = (idcode & 0xFFFU) == 0x413U   ? "STM32F40x"
                                        : (idcode & 0xFFFU) == 0x419U ? "STM32F42x"
                                        : (idcode & 0xFFFU) == 0x421U ? "STM32F446XX"
                                                                      : "STM32F???";
        char revision = 0;
        switch (idcode >> 16U) {
        case 0x1000U: revision = 'A'; break;
        case 0x1001U: revision = 'Z'; break;
        case 0x1003U: revision = 'Y'; break;
        case 0x1007U: revision = '1'; break;
        case 0x2001U: revision = '3'; break;
        default: break;
        }
        std::size_t n = 0;
        for (const char ch : family) {
            description_.at(n++) = ch;
        }
        description_.at(n++) = ',';
        if (revision != 0) {
            description_.at(n++) = revision;
        }
        return {description_.data(), n};
    }

    [[nodiscard]] std::uint32_t serial_word(std::uint32_t offset) const
    {
        return offset < 12 ? *word_at(UID_BASE + (offset & ~3U)) : 0xFFFFFFFFU;
    }

    [[nodiscard]] std::uint32_t otp_word(std::uint32_t offset) const
    {
        constexpr std::uint32_t otp_size = 528;  // 512 bytes of OTP and 16 lock bytes (RM0090 3.8)
        return offset < otp_size ? *word_at(FLASH_OTP_BASE + (offset & ~3U)) : 0xFFFFFFFFU;
    }

    void start_firmware()
    {
        const std::uint32_t stack = Firmware::read(0);
        const std::uint32_t entry = Firmware::read(4);
        if (entry < board::flash::FIRMWARE::address ||
            entry >= board::flash::FIRMWARE::address + board::flash::FIRMWARE::size) {
            return;
        }
        // Let the last reply leave before the ports go.
        for (const Uart& port : ports_) {
            while ((usart_at(port.address)->SR & USART_SR_TC) == 0U) {
            }
        }
        __disable_irq();
        SysTick->CTRL = 0;
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
        restore_reset_clocks();
        SCB->VTOR = board::flash::FIRMWARE::address;
        __DSB();
        __ISB();
        // Nothing is enabled to interrupt, so PRIMASK is cleared as after reset.
        __asm__ volatile("msr msp, %0\n\t"
                         "cpsie i\n\t"
                         "bx %1"
                         :
                         : "r"(stack), "r"(entry)
                         : "memory");
    }

    void report(std::string_view /*line*/) {}

private:
    // HSI as the system clock, the PLL and HSE off, every APB peripheral and
    // GPIO port reset and its clock off, as the firmware's start-up expects.
    static void restore_reset_clocks()
    {
        RCC->CR |= RCC_CR_HSION;
        while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
        }
        RCC->CFGR = 0;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
        }
        RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);
        while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
        }
        RCC->PLLCFGR = 0x24003010U;  // its reset value
        constexpr std::uint32_t gpio_ports = 0x1FFU;  // GPIOA to GPIOI
        RCC->APB1RSTR = 0xFFFFFFFFU;
        RCC->APB1RSTR = 0;
        RCC->APB2RSTR = 0xFFFFFFFFU;
        RCC->APB2RSTR = 0;
        RCC->AHB1RSTR = gpio_ports;
        RCC->AHB1RSTR = 0;
        RCC->APB1ENR = 0;
        RCC->APB2ENR = 0;
        RCC->AHB1ENR = RCC_AHB1ENR_CCMDATARAMEN;  // its reset value
    }

    decltype(Ports::configure()) ports_;
    std::size_t active_ = 0;
    bool locked_ = false;
    mutable std::array<char, 16> description_{};
};

static_assert(vk::bootloader::Platform<Stm32Platform>);

void start_systick()
{
    SysTick->LOAD = board::clock::sysclk_hz / 1000U - 1U;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

[[noreturn]] void bootloader_main()
{
    vk::platform::detail::configure_clock_tree();
    start_systick();
    Stm32Platform platform;
    vk::bootloader::Bootloader<Stm32Platform> bootloader(platform, {.board_id = board::board_id, .board_revision = 0});
    bootloader.run();
    vk_bootloader_fault();
}

}  // namespace

extern "C" {

void SysTick_Handler()
{
    milliseconds = milliseconds + 1;
}

// A fault resets the board, which then stays in the bootloader only if the firmware is bad.
void vk_bootloader_fault()
{
    NVIC_SystemReset();
}

void vk_bootloader_reset()
{
    for (std::uint32_t *src = _sidata, *dst = _sdata; dst < _edata;) {
        *dst++ = *src++;
    }
    for (std::uint32_t* dst = _sbss; dst < _ebss;) {
        *dst++ = 0;
    }
    SCB->CPACR |= 0xFU << 20U;  // the FPU, for code the compiler builds with it
    __DSB();
    __ISB();
    bootloader_main();
}
}
