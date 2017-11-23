/**
  ******************************************************************************
  * @file    xpd_rcc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-16
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
#define RCC_HSI_TIMEOUT         ((uint32_t)2)    /* 2 ms */
#define RCC_MSI_TIMEOUT         ((uint32_t)2)    /* 2 ms */
#define RCC_HSI48_TIMEOUT       ((uint32_t)2)    /* 2 ms */
#define RCC_LSI_TIMEOUT         ((uint32_t)2)    /* 2 ms */
#define RCC_PLL_TIMEOUT         ((uint32_t)2)    /* 2 ms */


/** @defgroup RCC_Peripheral_Control RCC Peripheral Control
 * @{ */

/* Relative position of peripheral control bits in RCC register space */
typedef uint16_t RCC_PositionType;

#define RCC_POS_AHB1            0
#define RCC_POS_AHB2            32
#define RCC_POS_AHB3            64
#define RCC_POS_APB11           128
#define RCC_POS_APB12           160
#define RCC_POS_APB2            192

#define RCC_POS_ADC             (RCC_POS_AHB2  + 13)
#define RCC_POS_AES             (RCC_POS_AHB2  + 16)
#define RCC_POS_CAN1            (RCC_POS_APB11 + 25)
#define RCC_POS_CAN2            (RCC_POS_APB11 + 26)
#define RCC_POS_CRC             (RCC_POS_AHB1  + 12)
#define RCC_POS_CRS             (RCC_POS_APB11 + 24)
#define RCC_POS_DAC1            (RCC_POS_APB11 + 29)
#define RCC_POS_DCMI            (RCC_POS_AHB2  + 14)
#define RCC_POS_DFSDM1          (RCC_POS_APB2  + 24)
#define RCC_POS_DMA1            (RCC_POS_AHB1  +  0)
#define RCC_POS_DMA2            (RCC_POS_AHB1  +  1)
#define RCC_POS_DMA2D           (RCC_POS_AHB1  + 17)
#define RCC_POS_DMAMUX1         (RCC_POS_AHB1  +  2)
#define RCC_POS_DSI             (RCC_POS_APB2  + 27)
#define RCC_POS_FLASH           (RCC_POS_AHB1  +  8)
#define RCC_POS_FMC             (RCC_POS_AHB3  +  0)
#define RCC_POS_FW              (RCC_POS_APB2  +  7)
#define RCC_POS_GFXMMU          (RCC_POS_AHB1  + 18)
#define RCC_POS_GPIOA           (RCC_POS_AHB2  +  0)
#define RCC_POS_GPIOB           (RCC_POS_AHB2  +  1)
#define RCC_POS_GPIOC           (RCC_POS_AHB2  +  2)
#define RCC_POS_GPIOD           (RCC_POS_AHB2  +  3)
#define RCC_POS_GPIOE           (RCC_POS_AHB2  +  4)
#define RCC_POS_GPIOF           (RCC_POS_AHB2  +  5)
#define RCC_POS_GPIOG           (RCC_POS_AHB2  +  6)
#define RCC_POS_GPIOH           (RCC_POS_AHB2  +  7)
#define RCC_POS_GPIOI           (RCC_POS_AHB2  +  8)
#define RCC_POS_HASH            (RCC_POS_AHB2  + 17)
#define RCC_POS_I2C1            (RCC_POS_APB11 + 21)
#define RCC_POS_I2C2            (RCC_POS_APB11 + 22)
#define RCC_POS_I2C3            (RCC_POS_APB11 + 23)
#define RCC_POS_I2C4            (RCC_POS_APB12 +  1)
#define RCC_POS_LCD             (RCC_POS_APB11 +  9)
#define RCC_POS_LPTIM1          (RCC_POS_APB11 + 31)
#define RCC_POS_LPTIM2          (RCC_POS_APB12 +  5)
#define RCC_POS_LPUART1         (RCC_POS_APB12 +  0)
#define RCC_POS_LTDC            (RCC_POS_APB2  + 26)
#define RCC_POS_OPAMP           (RCC_POS_APB11 + 30)
#define RCC_POS_OSPI1           (RCC_POS_AHB3  +  8)
#define RCC_POS_OSPI2           (RCC_POS_AHB3  +  9)
#define RCC_POS_OSPIM           (RCC_POS_AHB2  + 20)
#define RCC_POS_OTG_FS          (RCC_POS_AHB2  + 12)
#define RCC_POS_PWR             (RCC_POS_APB11 + 28)
#define RCC_POS_QSPI            (RCC_POS_AHB3  +  8)
#define RCC_POS_RNG             (RCC_POS_AHB2  + 18)
#define RCC_POS_RTC             (RCC_POS_APB11 + 10)
#define RCC_POS_SAI1            (RCC_POS_APB2  + 21)
#define RCC_POS_SAI2            (RCC_POS_APB2  + 22)
#define RCC_POS_SDMMC1          (RCC_POS_AHB2  + 22)
#define RCC_POS_SDMMC1          (RCC_POS_APB2  + 10)
#define RCC_POS_SPI1            (RCC_POS_APB2  + 12)
#define RCC_POS_SPI2            (RCC_POS_APB11 + 14)
#define RCC_POS_SPI3            (RCC_POS_APB11 + 15)
#define RCC_POS_SRAM1           (RCC_POS_AHB1  +  9)
#define RCC_POS_SRAM2           (RCC_POS_AHB2  +  9)
#define RCC_POS_SRAM3           (RCC_POS_AHB2  + 10)
#define RCC_POS_SWPMI1          (RCC_POS_APB12 +  2)
#define RCC_POS_SYSCFG          (RCC_POS_APB2  +  0)
#define RCC_POS_TIM1            (RCC_POS_APB2  + 11)
#define RCC_POS_TIM15           (RCC_POS_APB2  + 16)
#define RCC_POS_TIM16           (RCC_POS_APB2  + 17)
#define RCC_POS_TIM17           (RCC_POS_APB2  + 18)
#define RCC_POS_TIM2            (RCC_POS_APB11 +  0)
#define RCC_POS_TIM3            (RCC_POS_APB11 +  1)
#define RCC_POS_TIM4            (RCC_POS_APB11 +  2)
#define RCC_POS_TIM5            (RCC_POS_APB11 +  3)
#define RCC_POS_TIM6            (RCC_POS_APB11 +  4)
#define RCC_POS_TIM7            (RCC_POS_APB11 +  5)
#define RCC_POS_TIM8            (RCC_POS_APB2  + 13)
#define RCC_POS_TSC             (RCC_POS_AHB1  + 16)
#define RCC_POS_UART4           (RCC_POS_APB11 + 19)
#define RCC_POS_UART5           (RCC_POS_APB11 + 20)
#define RCC_POS_USART1          (RCC_POS_APB2  + 14)
#define RCC_POS_USART2          (RCC_POS_APB11 + 17)
#define RCC_POS_USART3          (RCC_POS_APB11 + 18)
#define RCC_POS_USB             (RCC_POS_APB11 + 26)
#define RCC_POS_WWDG            (RCC_POS_APB11 + 11)

/** @addtogroup RCC_Peripheral_Control_Exported_Functions
 * @{ */
void            XPD_RCC_ClockEnable         (RCC_PositionType PeriphPos);
void            XPD_RCC_ClockDisable        (RCC_PositionType PeriphPos);
void            XPD_RCC_SleepClockEnable    (RCC_PositionType PeriphPos);
void            XPD_RCC_SleepClockDisable   (RCC_PositionType PeriphPos);
void            XPD_RCC_Reset               (RCC_PositionType PeriphPos);
/** @} */

/** @} */

#include "xpd_rcc_cc.h"
#include "xpd_rcc_crs.h"

/** @} */

#endif /* __XPD_RCC_H_ */
