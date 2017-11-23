/**
  ******************************************************************************
  * @file    xpd_rcc.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-11-21
  * @brief   STM32 eXtensible Peripheral Drivers RCC Module
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
#ifndef __XPD_RCC_H_
#define __XPD_RCC_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup RCC
 * @{ */

#ifdef RCC_BB
#define RCC_REG_BIT(REG_NAME, BIT_NAME) (RCC_BB->REG_NAME.BIT_NAME)
#else
#define RCC_REG_BIT(REG_NAME, BIT_NAME) (RCC->REG_NAME.b.BIT_NAME)
#endif

/* Disable Backup domain write protection state change timeout */
#define RCC_DBP_TIMEOUT         ((uint32_t)100)
#define RCC_CLOCKSWITCH_TIMEOUT ((uint32_t)5000)   /* 5 s    */
#ifdef LSE_STARTUP_TIMEOUT
#define RCC_LSE_TIMEOUT         LSE_STARTUP_TIMEOUT
#else
#define RCC_LSE_TIMEOUT         ((uint32_t)5000)   /* 5 s    */
#endif
#ifdef HSE_STARTUP_TIMEOUT
#define RCC_HSE_TIMEOUT         HSE_STARTUP_TIMEOUT
#else
#define RCC_HSE_TIMEOUT         ((uint32_t)100)    /* 100 ms */
#endif
#define RCC_HSI_TIMEOUT         ((uint32_t)100)    /* 100 ms */
#define RCC_HSI14_TIMEOUT       ((uint32_t)100)    /* 100 ms */
#define RCC_HSI48_TIMEOUT       ((uint32_t)100)    /* 100 ms */
#define RCC_LSI_TIMEOUT         ((uint32_t)100)    /* 100 ms */
#define RCC_PLL_TIMEOUT         ((uint32_t)100)    /* 100 ms */

/** @defgroup RCC_Peripheral_Control RCC Peripheral Control
 * @{ */

/* Relative position of peripheral control bits in RCC register space */
typedef uint16_t RCC_PositionType;

#define RCC_POS_AHB             0
#define RCC_POS_APB2            32
#define RCC_POS_APB1            64

#define RCC_POS_ADC1            (RCC_POS_APB2 +  9)
#define RCC_POS_CAN             (RCC_POS_APB1 + 25)
#define RCC_POS_CEC             (RCC_POS_APB1 + 30)
#define RCC_POS_COMP            (RCC_POS_APB2 +  0)
#define RCC_POS_CRC             (RCC_POS_AHB  +  6)
#define RCC_POS_CRS             (RCC_POS_APB1 + 27)
#define RCC_POS_DAC             (RCC_POS_APB1 + 29)
#define RCC_POS_DBGMCU          (RCC_POS_APB2 + 22)
#define RCC_POS_DMA1            (RCC_POS_AHB  +  0)
#define RCC_POS_DMA2            (RCC_POS_AHB  +  1)
#define RCC_POS_FLITF           (RCC_POS_AHB  +  4)
#define RCC_POS_GPIOA           (RCC_POS_AHB  + 17)
#define RCC_POS_GPIOB           (RCC_POS_AHB  + 18)
#define RCC_POS_GPIOC           (RCC_POS_AHB  + 19)
#define RCC_POS_GPIOD           (RCC_POS_AHB  + 20)
#define RCC_POS_GPIOE           (RCC_POS_AHB  + 21)
#define RCC_POS_GPIOF           (RCC_POS_AHB  + 22)
#define RCC_POS_I2C1            (RCC_POS_APB1 + 21)
#define RCC_POS_I2C2            (RCC_POS_APB1 + 22)
#define RCC_POS_PWR             (RCC_POS_APB1 + 28)
#define RCC_POS_SPI1            (RCC_POS_APB2 + 12)
#define RCC_POS_SPI2            (RCC_POS_APB1 + 14)
#define RCC_POS_SRAM            (RCC_POS_AHB  +  2)
#define RCC_POS_SYSCFG          (RCC_POS_APB2 +  0)
#define RCC_POS_TIM1            (RCC_POS_APB2 + 11)
#define RCC_POS_TIM14           (RCC_POS_APB1 +  8)
#define RCC_POS_TIM15           (RCC_POS_APB2 + 16)
#define RCC_POS_TIM16           (RCC_POS_APB2 + 17)
#define RCC_POS_TIM17           (RCC_POS_APB2 + 18)
#define RCC_POS_TIM2            (RCC_POS_APB1 +  0)
#define RCC_POS_TIM3            (RCC_POS_APB1 +  1)
#define RCC_POS_TIM6            (RCC_POS_APB1 +  4)
#define RCC_POS_TIM7            (RCC_POS_APB1 +  5)
#define RCC_POS_TSC             (RCC_POS_AHB  + 24)
#define RCC_POS_USART1          (RCC_POS_APB2 + 14)
#define RCC_POS_USART2          (RCC_POS_APB1 + 17)
#define RCC_POS_USART3          (RCC_POS_APB1 + 18)
#define RCC_POS_USART4          (RCC_POS_APB1 + 19)
#define RCC_POS_USART5          (RCC_POS_APB1 + 20)
#define RCC_POS_USART6          (RCC_POS_APB2 +  5)
#define RCC_POS_USART7          (RCC_POS_APB2 +  6)
#define RCC_POS_USART8          (RCC_POS_APB2 +  7)
#define RCC_POS_USB             (RCC_POS_APB1 + 23)
#define RCC_POS_WWDG            (RCC_POS_APB1 + 11)

/** @addtogroup RCC_Peripheral_Control_Exported_Functions
 * @{ */
void            XPD_RCC_ClockEnable         (RCC_PositionType PeriphPos);
void            XPD_RCC_ClockDisable        (RCC_PositionType PeriphPos);
void            XPD_RCC_Reset               (RCC_PositionType PeriphPos);
/** @} */

/** @} */

#include "xpd_rcc_cc.h"
#include "xpd_rcc_crs.h"

/** @} */

#endif /* __XPD_RCC_H_ */
