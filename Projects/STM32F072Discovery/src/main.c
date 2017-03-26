#include "xpd.h"
#include "stm32f072b_discovery.h"
#include "stm32f072b_discovery_gyroscope.h"


void SysTick_Handler(void)
{
    XPD_SysTick_IRQHandler();
}


void ClockConfiguration(void)
{
    /* HSI48 configuration */
    {
        RCC_HSI_InitType hsi = {
            .State = OSC_ON,
            .CalibrationValue = 16
        };

        XPD_RCC_HSI48Config(&hsi);
    }

    /* System clocks configuration */
    {
        XPD_RCC_HCLKConfig(HSI48, CLK_DIV1, 1);

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
