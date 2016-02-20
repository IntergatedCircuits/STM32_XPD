/**
  ******************************************************************************
  * @file    can_test.c
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
#include "xpd_can.h"

extern volatile uint8_t passedtests;

static void can_init(CAN_HandleType *hcan)
{
    GPIO_InitType gpio;
    XPD_CAN2_EnableClock();

    gpio.AlternateMap = GPIO_CAN2_AF9;
    gpio.Mode = GPIO_MODE_ALTERNATE;
    gpio.Output.Speed = HIGH;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Pull = GPIO_PULL_UP;
    XPD_GPIO_InitPin(GPIOB, 5, &gpio);
    XPD_GPIO_InitPin(GPIOB, 13, &gpio);

    XPD_NVIC_SetPriorityConfig(CAN2_TX_IRQn,3,3);
    XPD_NVIC_EnableIRQ(CAN2_TX_IRQn);
    XPD_NVIC_SetPriorityConfig(CAN2_RX0_IRQn,3,3);
    XPD_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}

CAN_HandleType hcanx = NEW_CAN_HANDLE(CAN2,(XPD_HandleCallbackType)can_init,NULL);

CAN_FrameType  rcv;
CAN_FilterType fcan;
CAN_FilterType fcan2;

void CAN2_TX_IRQHandler(void)
{
    XPD_CAN_TX_IRQHandler(&hcanx);
}
void CAN2_RX0_IRQHandler(void)
{
    XPD_CAN_RX0_IRQHandler(&hcanx);
}

static void can_rcv(CAN_HandleType *hcan)
{
    /* message was received through the expected filter */
    if((rcv.Index == fcan2.MatchIndex) && (rcv.DLC == 3))
    {
        passedtests++;
    }
    if((rcv.Index == fcan.MatchIndex) && (rcv.DLC == 4))
    {
        passedtests++;
    }
    /* keep on receiving */
    XPD_CAN_Receive_IT(&hcanx, &rcv, 0);
}


void can_test(void)
{
    CAN_InitType ican;
    CAN_FrameType frm;

    /* initialize peripheral */
    ican.Features.Individual.Mode = CAN_MODE_LOOPBACK;
    ican.Features.Individual.TXFP = 1;
    ican.Timing.Prescaler = 4;
    ican.Timing.BS1 = 12;
    ican.Timing.BS2 = 8;
    ican.Timing.SJW = 4;
    XPD_CAN_Init(&hcanx,&ican);

    /* start receiving on fifo 0 */
    hcanx.Callbacks.Receive[0] = (XPD_HandleCallbackType)can_rcv;
    XPD_CAN_Receive_IT(&hcanx, &rcv, 0);

    /* add two std id match filters (the match index of the first is not stored...) */
    fcan.Mode          = CAN_FILTER_MATCH;
    fcan.Pattern.Value = 1;
    fcan.Pattern.Type  = CAN_IDTYPE_STD_DATA;
    XPD_CAN_FilterInit(&hcanx, 0, &fcan);

    fcan.Pattern.Value = 5;
    XPD_CAN_FilterInit(&hcanx, 0, &fcan);

    /* add an ext id mask filter */
    fcan2.Mode = CAN_FILTER_MASK;
    fcan2.Pattern.Value = 4;
    fcan2.Pattern.Type = CAN_IDTYPE_EXT_DATA;
    fcan2.Mask = 0xF;
    XPD_CAN_FilterInit(&hcanx, 0, &fcan2);

    /* send a loopback data */
    frm.Id.Type = CAN_IDTYPE_EXT_DATA;
    frm.Id.Value = 0x24;
    frm.DLC = 3;
    frm.Data.Byte[0] = 0;
    frm.Data.Byte[1] = 1;
    frm.Data.Byte[2] = 2;
    XPD_CAN_Transmit_IT(&hcanx, &frm);

    frm.Id = fcan.Pattern;
    frm.DLC = 4;
    frm.Data.Byte[3] = 3;
    XPD_CAN_Transmit_IT(&hcanx, &frm);
}
