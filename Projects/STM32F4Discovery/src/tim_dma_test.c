/**
  ******************************************************************************
  * @file    tim_dma_test.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-02-16
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

static void timx_init(TIM_HandleType * tim);

DMA_HandleType dmax = NEW_DMA_HANDLE(DMA1_Stream7);
TIM_HandleType timx = NEW_TIM_HANDLE(TIM3,(XPD_HandleCallbackType)timx_init,NULL);

static void timx_init(TIM_HandleType * tim)
{
    DMA_InitType idma;

    idma.Channel   = 5;
    idma.Direction = DMA_MEMORY2PERIPH;
    idma.Priority  = MEDIUM;
    idma.Mode      = DMA_MODE_NORMAL;
    idma.FIFO.Mode = DISABLE;
    idma.Memory.DataAlignment     = DMA_ALIGN_WORD;
    idma.Memory.Increment         = ENABLE;
    idma.Peripheral.DataAlignment = DMA_ALIGN_WORD;
    idma.Peripheral.Increment     = DISABLE;
    XPD_DMA_Init(&dmax, &idma);

    XPD_DMA_BINDTO(&dmax, &timx, Channel[TIM_CHANNEL_3]);

    XPD_NVIC_SetPriorityConfig(DMA1_Stream7_IRQn, 0, 3);
    XPD_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

static void timx_channelcb(TIM_HandleType * tim)
{
    if (XPD_TIM_Channel_GetActive(tim) == TIM_CHANNEL_3)
        passedtests++;
}

void DMA1_Stream7_IRQHandler(void)
{
    XPD_DMA_IRQHandler(&dmax);
}

uint32_t values[8] = {0,1250,2500,3750,5000,6250,7500,8750};

void tim_dma_test(void)
{
    TIM_Counter_InitType itim;

    itim.Prescaler = 168;   // get counter to 1 MHz
    itim.Period    = 10000; // get update to 100 Hz
    itim.Mode      = TIM_COUNTER_UP;
    itim.ClockDivision = CLK_DIV1;
    itim.RepetitionCounter = 0;
    XPD_TIM_Init(&timx, &itim);

    timx.Callbacks.ChannelEvent = (XPD_HandleCallbackType) timx_channelcb;

    XPD_TIM_Output_Start_DMA(&timx, TIM_CHANNEL_3, &values, 8);
}
