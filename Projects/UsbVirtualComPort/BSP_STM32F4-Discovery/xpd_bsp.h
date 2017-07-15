/**
  ******************************************************************************
  * @file    xpd_bsp.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-07-15
  * @brief   STM32 eXtensible Peripheral Drivers USB Virtual COM Port Project
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
#ifndef __XPD_BSP_H_
#define __XPD_BSP_H_

#include <xpd_core.h>
#include <xpd_gpio.h>
#include <xpd_usart.h>
#include <xpd_usb.h>

typedef enum
{
    UART_PIN_CFG = 0,
    USB_PIN_CFG,
    PIN_CFG_COUNT
}PinType;

#define UART_TX_PIN     GPIOA, 2
#define UART_RX_PIN     GPIOA, 3
#define USB_DP_PIN      GPIOA, 12
#define USB_DM_PIN      GPIOA, 11
#define USB_VBUS_PIN    GPIOA, 9

/* Indexed by PinType */
extern const GPIO_InitType PinConfig[];

/* Peripheral handle references */
extern USART_HandleType uart;
extern USB_HandleType usbHandle;

/* System clocks configuration */
void ClockConfiguration(void);

#endif /* __XPD_BSP_H_ */
