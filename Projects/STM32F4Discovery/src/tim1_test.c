/**
  ******************************************************************************
  * @file    tim1_test.c
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
#include "xpd_tim.h"

extern volatile uint8_t passedtests;

static void tim1_init(TIM_HandleType * tim)
{
    GPIO_InitType gpio;

    gpio.AlternateMap = GPIO_TIM1_AF1;
    gpio.Mode = GPIO_MODE_ALTERNATE;
    gpio.Output.Speed = VERY_HIGH;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Pull = GPIO_PULL_FLOAT;
    XPD_GPIO_InitPin(GPIOE, 8, &gpio);
    XPD_GPIO_InitPin(GPIOE, 9, &gpio);
    XPD_GPIO_InitPin(GPIOE, 14, &gpio);
    /* break */
    gpio.Pull = GPIO_PULL_DOWN;
    XPD_GPIO_InitPin(GPIOE, 15, &gpio);

    XPD_NVIC_SetPriorityConfig(TIM1_UP_TIM10_IRQn,3,3);
    XPD_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    XPD_NVIC_SetPriorityConfig(TIM1_BRK_TIM9_IRQn,1,3);
    XPD_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
}

TIM_HandleType tim1 = NEW_TIM_HANDLE(TIM1,(XPD_HandleCallbackType)tim1_init,NULL);


void TIM1_BRK_TIM9_IRQHandler(void)
{
    XPD_TIM_BRK_IRQHandler(&tim1);
}
void TIM1_UP_TIM10_IRQHandler(void)
{
    XPD_TIM_UP_IRQHandler(&tim1);
}

void tim_updated(void * tim)
{
    passedtests++;
    ((TIM_HandleType*)tim)->Callbacks.Update = NULL;
}

void tim_break(void * tim)
{
    passedtests++;
    ((TIM_HandleType*)tim)->Callbacks.Break = NULL;
}

void tim1_test(void)
{
    TIM_Counter_InitType itim;
    TIM_Output_InitType chinit;
    TIM_Output_DriveType brk;

    itim.Prescaler = 16800; // get counter to 10 kHz
    itim.Period    = 10000; // get update to 1 Hz
    itim.Mode      = TIM_COUNTER_UP;
    itim.ClockDivision = CLK_DIV4;
    itim.RepetitionCounter = 0;
    XPD_TIM_Init(&tim1, &itim);

    tim1.Callbacks.Update = tim_updated;
    XPD_TIM_Counter_Start_IT(&tim1);

    XPD_Delay_ms(1500);

    XPD_TIM_Counter_Stop_IT(&tim1);

    chinit.Output = TIM_OUTPUT_PWM1;
    chinit.Channel.ActiveLevel = ACTIVE_HIGH;
    chinit.Channel.IdleState = RESET;
    chinit.CompChannel.ActiveLevel = ACTIVE_HIGH;
    chinit.CompChannel.IdleState = RESET;
    XPD_TIM_Output_Init(&tim1, TIM_CHANNEL_1, &chinit);
    chinit.Output = TIM_OUTPUT_PWM2;
    XPD_TIM_Output_Init(&tim1, TIM_CHANNEL_4, &chinit);

    XPD_TIM_Channel_SetPulse(&tim1, TIM_CHANNEL_1, itim.Period - 4000);

    XPD_TIM_Channel_SetPulse(&tim1, TIM_CHANNEL_4, itim.Period * 1 / 2);

    brk.AutomaticOutput = ENABLE;
    brk.Break.State = ENABLE;
    brk.Break.Polarity = ACTIVE_HIGH;
    brk.LockLevel = 0;
    brk.IdleOffState = ENABLE;
    brk.RunOffState = ENABLE;
    XPD_TIM_Output_DriveConfig(&tim1, &brk);

    tim1.Callbacks.Break = tim_break;
    XPD_TIM_EnableIT(&tim1,B);

    XPD_TIM_Output_SetDeadtime(&tim1, 1008);

    XPD_TIM_Output_Start(&tim1, TIM_CHANNEL_1);

    XPD_TIM_Output_Start(&tim1, TIM_CHANNEL_4);


}
