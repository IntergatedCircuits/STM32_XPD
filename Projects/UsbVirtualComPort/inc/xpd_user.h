/**
  ******************************************************************************
  * @file    xpd_user.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-18
  * @brief   STM32 eXtensible Peripheral Drivers User Header
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
#ifndef __XPD_USER_H_
#define __XPD_USER_H_

#include <xpd_core.h>
#include <xpd_dma.h>
#include <xpd_gpio.h>
#include <xpd_rcc.h>
#include <xpd_usart.h>
#include <xpd_usb.h>

typedef enum
{
    UART_PIN_CFG = 0,
    USB_PIN_CFG,
    PIN_CFG_COUNT
}PinType;

#define UART_TX_PIN     GPIOA, 9
#define UART_RX_PIN     GPIOA, 10
#define USB_DP_PIN      GPIOA, 12
#define USB_DM_PIN      GPIOA, 11

/* Ensure preemption-free USB-UART interrupt handling */
#define NVIC_COMMON_PRIO_USB_USART     0

/* Indexed by PinType */
extern const GPIO_InitType PinConfig[];

extern USART_HandleType uart;
extern USB_HandleType usbHandle;

extern void ClockConfiguration(void);

#endif /* __XPD_USER_H_ */
