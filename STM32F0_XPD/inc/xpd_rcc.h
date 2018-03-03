/**
  ******************************************************************************
  * @file    xpd_rcc.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers RCC Module
  *
  * Copyright (c) 2018 Benedek Kupper
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  */
#ifndef __XPD_RCC_H_
#define __XPD_RCC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup RCC
 * @{ */

#ifdef RCC_BB
/**
 * @brief RCC register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         RCC_REG_BIT(REG_NAME, BIT_NAME)     \
    (RCC_BB->REG_NAME.BIT_NAME)
#else
/**
 * @brief RCC register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         RCC_REG_BIT(REG_NAME, BIT_NAME)     \
    (RCC->REG_NAME.b.BIT_NAME)
#endif

/* Timeouts for RCC operations */
#define RCC_DBP_TIMEOUT         ((uint32_t)100)
#define RCC_CLOCKSWITCH_TIMEOUT ((uint32_t)5000)   /* 5 s    */
#ifdef  LSE_STARTUP_TIMEOUT
#define RCC_LSE_TIMEOUT         LSE_STARTUP_TIMEOUT
#else
#define RCC_LSE_TIMEOUT         ((uint32_t)5000)   /* 5 s    */
#endif
#ifdef  HSE_STARTUP_TIMEOUT
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
void            RCC_vClockEnable        (RCC_PositionType PeriphPos);
void            RCC_vClockDisable       (RCC_PositionType PeriphPos);
void            RCC_vReset              (RCC_PositionType PeriphPos);
/** @} */

/** @} */

/** @} */

#include <xpd_mco.h>
#include <xpd_rcc_cc.h>

#ifdef __cplusplus
}
#endif

#endif /* __XPD_RCC_H_ */
