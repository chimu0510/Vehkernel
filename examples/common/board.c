#include "board.h"

#include "console.h"

static bool s_hse_ok;

/* HSE 8 MHz 或 HSI 16 MHz 經 PLL 到 168 MHz；HSE 起振失敗時改用 HSI，主控台仍可用。 */
static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    s_hse_ok = (HAL_RCC_OscConfig(&osc) == HAL_OK);

    if (!s_hse_ok) {
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
        osc.HSEState = RCC_HSE_OFF;
        osc.HSIState = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM = 16;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
            /* PLL 起不來就留在預設的 HSI 16 MHz。 */
            return;
        }
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;   /* HCLK  168 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;    /* PCLK1  42 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;    /* PCLK2  84 MHz */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

static void gpio_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    led_set(false, false, false);
    buzzer_set(false);

    io.Mode = GPIO_MODE_OUTPUT_PP;
    io.Pull = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    io.Pin = LED_R_PIN;
    HAL_GPIO_Init(LED_R_PORT, &io);
    io.Pin = LED_G_PIN;
    HAL_GPIO_Init(LED_G_PORT, &io);
    io.Pin = LED_B_PIN;
    HAL_GPIO_Init(LED_B_PORT, &io);
    io.Pin = BUZZER_PIN;
    HAL_GPIO_Init(BUZZER_PORT, &io);

    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;
    io.Pin = USER_KEY_PIN;
    HAL_GPIO_Init(USER_KEY_PORT, &io);
    io.Pin = ENABLE_SW_PIN;
    HAL_GPIO_Init(ENABLE_SW_PORT, &io);
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void board_init(void)
{
    HAL_Init();
    clock_init();
    dwt_init();
    gpio_init();
    console_init();
}

bool board_hse_ok(void)
{
    return s_hse_ok;
}

uint32_t micros(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void delay_us(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

static void write_led(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    const GPIO_PinState off_level = (LED_ON_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(port, pin, on ? LED_ON_LEVEL : off_level);
}

void led_set(bool r, bool g, bool b)
{
    write_led(LED_R_PORT, LED_R_PIN, r);
    write_led(LED_G_PORT, LED_G_PIN, g);
    write_led(LED_B_PORT, LED_B_PIN, b);
}

void buzzer_set(bool on)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool user_key_pressed(void)
{
    return HAL_GPIO_ReadPin(USER_KEY_PORT, USER_KEY_PIN) == GPIO_PIN_RESET;
}

bool enable_switch_high(void)
{
    return HAL_GPIO_ReadPin(ENABLE_SW_PORT, ENABLE_SW_PIN) == GPIO_PIN_SET;
}
