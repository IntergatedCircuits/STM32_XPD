/**
  ******************************************************************************
  * @file    main.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-17
  * @brief   STM32 eXtensible Peripheral Drivers TODO Module
  *
  *  This file is part of STM32_XPD.
  *
  *  STM32_XPD is free software: you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation, either version 3 of the License, or
  *  (at your option) any later version.
  *
  *  STM32_XPD is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU General Public License for more details.
  *
  *  You should have received a copy of the GNU General Public License
  *  along with STM32_XPD.  If not, see <http://www.gnu.org/licenses/>.
  */

#include "xpd.h"

// debug variable
volatile uint8_t passedtests = 0;


void millisecCallback(void)
{
    passedtests++;
    XPD_Callbacks.Tick = NULL;
}

void SysTick_Handler(void)
{
    XPD_SysTick_IRQHandler();
}

extern void can_test(void);
extern void gpio_test(void);
extern void tim1_test(void);
extern void tim_dma_test(void);

/* clock configuration: 8 MHz HSE, 168 MHz SYSCLK, 5 cycles flash latency */
void ClockConfiguration(void)
{
    /* HSE configuration */
    {
        RCC_HSE_InitType hse = {OSC_ON};

        XPD_RCC_HSEConfig(&hse);
    }

    /* PLL configuration */
    {
        RCC_PLL_InitType pll = {
            .State = OSC_ON,
            .Source = HSE,
            .M = 8,
            .N = 336,
            .P = 2,
            .Q = 7
        };

        XPD_RCC_PLLConfig(&pll);
    }

    /* System clocks configuration */
    {
        RCC_ClockInitType clocks = {
            .HCLK_Divider = CLK_DIV1,
            .PCLK1_Divider = CLK_DIV4,
            .PCLK2_Divider = CLK_DIV2,
            .SYSCLK_Source = PLL
        };

        XPD_RCC_ClockConfig(HCLK | SYSCLK | PCLK1 | PCLK2, &clocks, 5);
    }
}


int main(void)
{
    ClockConfiguration();

    /* enable all GPIOs */
    RCC->AHB1ENR.w |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
                    | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN
                    | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN
                    | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN
                    | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN;

    /* SysTick callback test */
    XPD_Callbacks.Tick = millisecCallback;

    /* GPIO and EXTI test */
    gpio_test();

    /* CAN test */
    can_test();

    /* TIM1 test */
    tim1_test();

    /* DMA test with TIM */
    tim_dma_test();

    while(1)
    {
    }
}
