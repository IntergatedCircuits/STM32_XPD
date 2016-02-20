/**
  ******************************************************************************
  * @file    gpio_test.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-29
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


void buttonHandler(uint32_t line)
{
    if (line == 0)
        XPD_GPIO_TogglePin(GPIOD, 15);
}

void EXTI0_IRQHandler(void)
{
    XPD_EXTI_IRQHandler(0);
}

void gpio_test(void)
{
    GPIO_InitType gpio;

    /* LEDs init */
    gpio.Mode         = GPIO_MODE_OUTPUT;
    gpio.Output.Speed = HIGH;
    gpio.Output.Type  = GPIO_OUTPUT_PUSHPULL;
    gpio.Pull         = GPIO_PULL_FLOAT;
    XPD_GPIO_InitPin(GPIOD, 12, &gpio);
    XPD_GPIO_InitPin(GPIOD, 13, &gpio);
    XPD_GPIO_InitPin(GPIOD, 14, &gpio);
    XPD_GPIO_InitPin(GPIOD, 15, &gpio);

    /* button init */
    gpio.Mode            = GPIO_MODE_EXTI;
    gpio.Pull            = GPIO_PULL_FLOAT;
    gpio.ExtI.Edge       = EDGE_RISING;
    gpio.ExtI.Reaction   = REACTION_IT;
    gpio.ExtI.ITCallback = buttonHandler;
    XPD_GPIO_InitPin(GPIOA,0,&gpio);

    XPD_NVIC_SetPriorityConfig(EXTI0_IRQn, 2, 2);
    XPD_NVIC_EnableIRQ(EXTI0_IRQn);
}
