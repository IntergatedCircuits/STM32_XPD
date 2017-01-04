#include "xpd.h"
#include "stm32f072b_discovery.h"
#include "stm32f072b_discovery_gyroscope.h"


void SysTick_Handler(void)
{
    XPD_SysTick_IRQHandler();
}


void ClockConfiguration(void)
{
    /* PLL configuration */
    {
        RCC_PLL_InitType pll = {
            .State = OSC_ON,
            .Source = HSI48,
            .Multiplier = 1,
            .Predivider = 1
        };

        XPD_RCC_PLLConfig(&pll);
    }

    /* System clocks configuration */
    {
        XPD_RCC_HCLKConfig(PLL, CLK_DIV1, 1);

        XPD_RCC_PCLKConfig(PCLK1, CLK_DIV1);
    }
}

float xyz[3];

static void msTickCallback(void)
{
}

int main(void)
{
    int i;
    ClockConfiguration();

    /* LEDs */
    BSP_LED_Init(LED_GREEN);
    BSP_LED_On(LED_GREEN);

    BSP_GYRO_Init();

    XPD_Callbacks.Tick = msTickCallback;

    while(1)
    {
        BSP_GYRO_GetXYZ(xyz);
    }
}
