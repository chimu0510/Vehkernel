// Clock tree set-up from the board definition's solved values (ADR-0011).
//
// The board generator has already checked that the values are reachable; this
// file only translates them to register fields, so a board with another
// crystal or system clock needs no change here.
#include "internal.hpp"

#include <board.hpp>

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_pwr.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_system.h>

#include <cstdint>

namespace vk::platform::detail {
namespace {

namespace clk = vk::board::clock;

// Not constexpr: reaching it while evaluating a constant fails the build.
std::uint32_t invalid_divider()
{
    return 0;
}

constexpr std::uint32_t pllm_field(std::uint32_t m)
{
    return m << RCC_PLLCFGR_PLLM_Pos;
}

constexpr std::uint32_t pllp_field(std::uint32_t p)
{
    return ((p / 2U) - 1U) << RCC_PLLCFGR_PLLP_Pos;
}

constexpr std::uint32_t pllq_field(std::uint32_t q)
{
    return q << RCC_PLLCFGR_PLLQ_Pos;
}

constexpr std::uint32_t apb1_field(std::uint32_t div)
{
    switch (div) {
    case 1: return LL_RCC_APB1_DIV_1;
    case 2: return LL_RCC_APB1_DIV_2;
    case 4: return LL_RCC_APB1_DIV_4;
    case 8: return LL_RCC_APB1_DIV_8;
    case 16: return LL_RCC_APB1_DIV_16;
    default: return invalid_divider();
    }
}

constexpr std::uint32_t apb2_field(std::uint32_t div)
{
    switch (div) {
    case 1: return LL_RCC_APB2_DIV_1;
    case 2: return LL_RCC_APB2_DIV_2;
    case 4: return LL_RCC_APB2_DIV_4;
    case 8: return LL_RCC_APB2_DIV_8;
    case 16: return LL_RCC_APB2_DIV_16;
    default: return invalid_divider();
    }
}

constexpr std::uint32_t pllm = pllm_field(clk::pll_m);
constexpr std::uint32_t pllp = pllp_field(clk::pll_p);
constexpr std::uint32_t pllq = pllq_field(clk::pll_q);
constexpr std::uint32_t apb1 = apb1_field(clk::apb1_div);
constexpr std::uint32_t apb2 = apb2_field(clk::apb2_div);
constexpr std::uint32_t flash_latency = clk::flash_wait_states << FLASH_ACR_LATENCY_Pos;

}  // namespace

void configure_clock_tree()
{
    // Voltage scale 1 allows the full 168 MHz.
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    // Raise the Flash wait states before the clock goes up.
    LL_FLASH_SetLatency(flash_latency);
    while (LL_FLASH_GetLatency() != flash_latency) {
    }
    LL_FLASH_EnablePrefetch();
    LL_FLASH_EnableInstCache();
    LL_FLASH_EnableDataCache();

    // A board whose crystal does not start stays here, on HSI, with SWD still usable.
    LL_RCC_HSE_Enable();
    while (LL_RCC_HSE_IsReady() == 0U) {
    }

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, pllm, clk::pll_n, pllp);
    LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, pllm, clk::pll_n, pllq);
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() == 0U) {
    }

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(apb1);
    LL_RCC_SetAPB2Prescaler(apb2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
}

}  // namespace vk::platform::detail
