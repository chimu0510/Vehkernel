// A board serial port on an STM32F4 USART, with DMA both ways (ADR-0009).
//
// Receive runs one circular DMA transfer into a ring buffer for good; read()
// copies what the DMA has written since the last call. Transmit queues bytes
// in a second ring buffer, and each call starts the next DMA transfer once
// the previous one is done, so no interrupt is needed: the Link calls often
// enough to keep the line busy. Both buffers sit in normal SRAM, which the
// DMA controllers reach and CCM is not (ADR-0008). Pins, the USART and the
// DMA streams come from the board's port definition.
#pragma once

#include <vk/hal/dma.hpp>
#include <vk/hal/pin.hpp>
#include <vk/hal/uart.hpp>
#include <vk/platform/gpio.hpp>
#include <vk/platform/usart_baud.hpp>

#include <board.hpp>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace vk::platform {

namespace detail {

// NOLINTBEGIN(performance-no-int-to-ptr): peripheral registers live at fixed addresses.
inline USART_TypeDef* usart_at(std::uint32_t address)
{
    return reinterpret_cast<USART_TypeDef*>(address);
}

inline DMA_TypeDef* dma_controller(hal::DmaController controller)
{
    return reinterpret_cast<DMA_TypeDef*>(controller == hal::DmaController::Dma1 ? DMA1_BASE : DMA2_BASE);
}

inline DMA_Stream_TypeDef* dma_stream(hal::DmaController controller, std::uint8_t stream)
{
    constexpr std::uint32_t first_stream = DMA1_Stream0_BASE - DMA1_BASE;
    constexpr std::uint32_t stride = DMA1_Stream1_BASE - DMA1_Stream0_BASE;
    const std::uint32_t base = controller == hal::DmaController::Dma1 ? DMA1_BASE : DMA2_BASE;
    return reinterpret_cast<DMA_Stream_TypeDef*>(base + first_stream + stride * stream);
}
// NOLINTEND(performance-no-int-to-ptr)

// Clears every event flag of one stream (RM0090 10.5.4): streams 0-3 in
// LIFCR, 4-7 in HIFCR, at these bit offsets.
inline void clear_dma_flags(hal::DmaController controller, std::uint8_t stream)
{
    constexpr std::array<std::uint32_t, 4> offsets{0, 6, 16, 22};
    constexpr std::uint32_t all_flags = 0x3DU;  // FEIF, DMEIF, TEIF, HTIF, TCIF
    DMA_TypeDef* const dma = dma_controller(controller);
    const std::uint32_t bits = all_flags << offsets.at(stream % 4U);
    if (stream < 4) {
        dma->LIFCR = bits;
    } else {
        dma->HIFCR = bits;
    }
}

inline void enable_dma_clock(hal::DmaController controller)
{
    LL_AHB1_GRP1_EnableClock(controller == hal::DmaController::Dma1 ? LL_AHB1_GRP1_PERIPH_DMA1
                                                                      : LL_AHB1_GRP1_PERIPH_DMA2);
}

// Enables a USART's clock; false for an address that is not a USART.
inline bool enable_usart_clock(std::uint32_t address)
{
    switch (address) {
    case USART1_BASE:
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
        return true;
    case USART2_BASE:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
        return true;
    case USART3_BASE:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
        return true;
    case UART4_BASE:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
        return true;
    case UART5_BASE:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
        return true;
    case USART6_BASE:
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
        return true;
    default:
        return false;
    }
}

template <hal::AfPinType Pin>
void configure_af_pin(std::uint32_t pull)
{
    enable_gpio_clock(Pin::port);
    GPIO_TypeDef* const port = gpio_port(Pin::port);
    constexpr std::uint32_t mask = 1U << Pin::number;
    if constexpr (Pin::number < 8) {
        LL_GPIO_SetAFPin_0_7(port, mask, Pin::af);
    } else {
        LL_GPIO_SetAFPin_8_15(port, mask, Pin::af);
    }
    LL_GPIO_SetPinOutputType(port, mask, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(port, mask, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(port, mask, pull);
    LL_GPIO_SetPinMode(port, mask, LL_GPIO_MODE_ALTERNATE);
}

}  // namespace detail

template <class Port, std::size_t TxSize = 1024, std::size_t RxSize = 1024>
class DmaUart {
public:
    using Tx = typename Port::tx;
    using Rx = typename Port::rx;
    using TxDma = typename Port::tx_dma;
    using RxDma = typename Port::rx_dma;
    using Registers = typename Port::registers;

    static_assert(Registers::kind == "usart", "a serial port needs a USART or UART");
    static_assert((TxSize & (TxSize - 1)) == 0 && (RxSize & (RxSize - 1)) == 0,
                  "buffer sizes are powers of two, so indices wrap by masking");
    static_assert(!TxDma::shared && !RxDma::shared, "the serial port's DMA streams must be its own");

    static constexpr std::uint32_t pclk_hz =
        Registers::bus_clock == "PCLK2" ? vk::board::clock::apb2_hz : vk::board::clock::apb1_hz;

    // Configures the pins, the USART and both DMA streams, and starts
    // receiving at `baud`; false if the baud rate is out of reach.
    bool configure(std::uint32_t baud)
    {
        if (!usart_baud_reachable(pclk_hz, baud) || !detail::enable_usart_clock(Registers::address)) {
            return false;
        }
        detail::configure_af_pin<Tx>(LL_GPIO_PULL_UP);
        detail::configure_af_pin<Rx>(LL_GPIO_PULL_UP);  // an unplugged line idles high
        detail::enable_dma_clock(TxDma::controller);
        detail::enable_dma_clock(RxDma::controller);

        USART_TypeDef* const usart = detail::usart_at(Registers::address);
        usart->CR1 = 0;
        usart->CR2 = 0;
        usart->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
        usart->BRR = usart_brr(pclk_hz, baud);

        DMA_Stream_TypeDef* const rx = detail::dma_stream(RxDma::controller, RxDma::stream);
        rx->CR = 0;
        while ((rx->CR & DMA_SxCR_EN) != 0U) {
        }
        detail::clear_dma_flags(RxDma::controller, RxDma::stream);
        rx->PAR = reinterpret_cast<std::uint32_t>(&usart->DR);
        rx->M0AR = reinterpret_cast<std::uint32_t>(rx_buffer_.data());
        rx->NDTR = RxSize;
        rx->FCR = 0;  // direct mode
        rx->CR = channel_bits(RxDma::channel) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_PL_1;
        rx->CR |= DMA_SxCR_EN;

        DMA_Stream_TypeDef* const tx = detail::dma_stream(TxDma::controller, TxDma::stream);
        tx->CR = 0;
        while ((tx->CR & DMA_SxCR_EN) != 0U) {
        }
        detail::clear_dma_flags(TxDma::controller, TxDma::stream);
        tx->PAR = reinterpret_cast<std::uint32_t>(&usart->DR);
        tx->FCR = 0;
        tx->CR = channel_bits(TxDma::channel) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_PL_0;

        rx_tail_ = 0;
        tx_head_ = tx_tail_ = tx_in_flight_ = 0;
        usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
        return true;
    }

    [[nodiscard]] std::size_t write_space() const
    {
        // Bytes the DMA has sent are freed by the next read() or write().
        return TxSize - (tx_head_ - tx_tail_);
    }

    std::size_t write(std::span<const std::uint8_t> data)
    {
        service_tx();
        const std::size_t size = std::min(data.size(), write_space());
        for (std::size_t i = 0; i < size; ++i) {
            tx_buffer_[(tx_head_ + i) & (TxSize - 1)] = data[i];
        }
        tx_head_ += size;
        service_tx();
        return size;
    }

    std::size_t read(std::span<std::uint8_t> out)
    {
        service_tx();  // the Link reads every run, so transmission keeps going between writes
        const DMA_Stream_TypeDef* const rx = detail::dma_stream(RxDma::controller, RxDma::stream);
        const std::size_t head = (RxSize - rx->NDTR) & (RxSize - 1);
        __DMB();  // the bytes before `head` are written
        std::size_t size = 0;
        while (rx_tail_ != head && size < out.size()) {
            out[size++] = rx_buffer_[rx_tail_];
            rx_tail_ = (rx_tail_ + 1) & (RxSize - 1);
        }
        return size;
    }

private:
    static constexpr std::uint32_t channel_bits(std::uint8_t channel)
    {
        return static_cast<std::uint32_t>(channel) << DMA_SxCR_CHSEL_Pos;
    }

    // Frees what the finished transfer sent and starts the next one.
    void service_tx()
    {
        DMA_Stream_TypeDef* const tx = detail::dma_stream(TxDma::controller, TxDma::stream);
        if (tx_in_flight_ != 0) {
            if ((tx->CR & DMA_SxCR_EN) != 0U) {
                return;  // still sending
            }
            tx_tail_ += tx_in_flight_;
            tx_in_flight_ = 0;
        }
        if (tx_head_ == tx_tail_) {
            return;
        }
        // One contiguous run: up to the newest byte or the end of the buffer.
        const std::size_t start = tx_tail_ & (TxSize - 1);
        const std::size_t size = std::min(tx_head_ - tx_tail_, TxSize - start);
        __DMB();  // the bytes are in memory before the DMA reads them
        detail::clear_dma_flags(TxDma::controller, TxDma::stream);
        tx->M0AR = reinterpret_cast<std::uint32_t>(&tx_buffer_[start]);
        tx->NDTR = static_cast<std::uint32_t>(size);
        tx_in_flight_ = size;
        tx->CR |= DMA_SxCR_EN;
    }

    std::array<std::uint8_t, TxSize> tx_buffer_{};
    std::array<std::uint8_t, RxSize> rx_buffer_{};
    std::size_t tx_head_ = 0;  // free-running: bytes queued since configure()
    std::size_t tx_tail_ = 0;  // free-running: bytes sent
    std::size_t tx_in_flight_ = 0;
    std::size_t rx_tail_ = 0;
};

}  // namespace vk::platform
