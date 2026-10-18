// Typed pins from the interface layer, as the board generator emits them.
#include <gtest/gtest.h>

#include <vk/hal/pin.hpp>

namespace {

using Port = vk::hal::Port;
using Tx = vk::hal::AfPin<Port::A, 9, 7, "USART1_TX">;
using Led = vk::hal::GpioOutput<Port::A, 1>;
using Adc = vk::hal::AnalogPin<Port::A, 0, "ADC1_IN0">;

static_assert(vk::hal::AfPinType<Tx>);
static_assert(!vk::hal::AfPinType<Led>);
static_assert(vk::hal::PinType<Adc>);
static_assert(vk::hal::distinct_pins<vk::hal::PinList<Tx, Led, Adc>>);
static_assert(!vk::hal::distinct_pins<vk::hal::PinList<Tx, Led, vk::hal::GpioInput<Port::A, 9>>>);
static_assert(vk::hal::distinct_pins<vk::hal::PinList<>>);

}  // namespace

TEST(Pin, AfPinCarriesItsSignalAndAlternateFunction)
{
    EXPECT_EQ(Tx::signal, "USART1_TX");
    EXPECT_EQ(Tx::af, 7U);
    EXPECT_EQ(Tx::port, Port::A);
    EXPECT_EQ(Tx::number, 9U);
}

TEST(Pin, AnalogPinCarriesItsSignal)
{
    EXPECT_EQ(Adc::signal, "ADC1_IN0");
}
