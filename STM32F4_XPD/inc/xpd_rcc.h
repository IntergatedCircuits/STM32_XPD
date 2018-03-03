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
#define RCC_LSI_TIMEOUT         ((uint32_t)100)    /* 100 ms */
#define RCC_PLL_TIMEOUT         ((uint32_t)100)    /* 100 ms */

/** @defgroup RCC_Peripheral_Control RCC Peripheral Control
 * @{ */

/* Relative position of peripheral control bits in RCC register space */
typedef uint16_t RCC_PositionType;

#define RCC_POS_AHB1            0
#define RCC_POS_AHB2            32
#define RCC_POS_AHB3            64
#define RCC_POS_APB1            128
#define RCC_POS_APB2            160

#define RCC_POS_ADC             (RCC_POS_APB2 +  8)
#define RCC_POS_ADC1            (RCC_POS_APB2 +  8)
#define RCC_POS_ADC2            (RCC_POS_APB2 +  9)
#define RCC_POS_ADC3            (RCC_POS_APB2 + 10)
#define RCC_POS_AES             (RCC_POS_AHB2 +  4)
#define RCC_POS_BKPSRAM         (RCC_POS_AHB1 + 18)
#define RCC_POS_CAN1            (RCC_POS_APB1 + 25)
#define RCC_POS_CAN2            (RCC_POS_APB1 + 26)
#define RCC_POS_CAN3            (RCC_POS_APB1 + 27)
#define RCC_POS_CCMDATARAM      (RCC_POS_AHB1 + 20)
#define RCC_POS_CEC             (RCC_POS_APB1 + 27)
#define RCC_POS_CRC             (RCC_POS_AHB1 + 12)
#define RCC_POS_CRYP            (RCC_POS_AHB2 +  4)
#define RCC_POS_DAC             (RCC_POS_APB1 + 29)
#define RCC_POS_DCMI            (RCC_POS_AHB2 +  0)
#define RCC_POS_DFSDM1          (RCC_POS_APB2 + 24)
#define RCC_POS_DFSDM2          (RCC_POS_APB2 + 25)
#define RCC_POS_DMA1            (RCC_POS_AHB1 + 21)
#define RCC_POS_DMA2            (RCC_POS_AHB1 + 22)
#define RCC_POS_DMA2D           (RCC_POS_AHB1 + 23)
#define RCC_POS_DSI             (RCC_POS_APB2 + 27)
#define RCC_POS_ETH_MAC         (RCC_POS_AHB1 + 25)
#define RCC_POS_ETH_MAC_PTP     (RCC_POS_AHB1 + 28)
#define RCC_POS_ETH_MAC_RX      (RCC_POS_AHB1 + 27)
#define RCC_POS_ETH_MAC_TX      (RCC_POS_AHB1 + 26)
#define RCC_POS_EXTI            (RCC_POS_APB2 + 15)
#define RCC_POS_FLITF           (RCC_POS_AHB1 + 15)
#define RCC_POS_FMC             (RCC_POS_AHB3 +  0)
#define RCC_POS_FMP_I2C1        (RCC_POS_APB1 + 24)
#define RCC_POS_FSMC            (RCC_POS_AHB3 +  0)
#define RCC_POS_GPIOA           (RCC_POS_AHB1 +  0)
#define RCC_POS_GPIOB           (RCC_POS_AHB1 +  1)
#define RCC_POS_GPIOC           (RCC_POS_AHB1 +  2)
#define RCC_POS_GPIOD           (RCC_POS_AHB1 +  3)
#define RCC_POS_GPIOE           (RCC_POS_AHB1 +  4)
#define RCC_POS_GPIOF           (RCC_POS_AHB1 +  5)
#define RCC_POS_GPIOG           (RCC_POS_AHB1 +  6)
#define RCC_POS_GPIOH           (RCC_POS_AHB1 +  7)
#define RCC_POS_GPIOI           (RCC_POS_AHB1 +  8)
#define RCC_POS_GPIOJ           (RCC_POS_AHB1 +  9)
#define RCC_POS_GPIOK           (RCC_POS_AHB1 + 10)
#define RCC_POS_HASH            (RCC_POS_AHB2 +  5)
#define RCC_POS_I2C1            (RCC_POS_APB1 + 21)
#define RCC_POS_I2C2            (RCC_POS_APB1 + 22)
#define RCC_POS_I2C3            (RCC_POS_APB1 + 23)
#define RCC_POS_LPTIM1          (RCC_POS_APB1 +  9)
#define RCC_POS_LTDC            (RCC_POS_APB2 + 26)
#define RCC_POS_OTG_FS          (RCC_POS_AHB2 +  7)
#define RCC_POS_OTG_HS          (RCC_POS_AHB1 + 29)
#define RCC_POS_OTG_HS_ULPI     (RCC_POS_AHB1 + 30)
#define RCC_POS_PWR             (RCC_POS_APB1 + 28)
#define RCC_POS_QSPI            (RCC_POS_AHB3 +  1)
#ifdef RCC_AHB1ENR_RNGEN
#define RCC_POS_RNG             (RCC_POS_AHB1 + 31)
#else
#define RCC_POS_RNG             (RCC_POS_AHB2 +  6)
#endif
#define RCC_POS_RTC             (RCC_POS_APB1 + 10)
#define RCC_POS_SAI1            (RCC_POS_APB2 + 22)
#define RCC_POS_SAI2            (RCC_POS_APB2 + 23)
#define RCC_POS_SDIO            (RCC_POS_APB2 + 11)
#define RCC_POS_SPDIFRX         (RCC_POS_APB1 + 16)
#define RCC_POS_SPI1            (RCC_POS_APB2 + 12)
#define RCC_POS_SPI2            (RCC_POS_APB1 + 14)
#define RCC_POS_SPI3            (RCC_POS_APB1 + 15)
#define RCC_POS_SPI4            (RCC_POS_APB2 + 13)
#define RCC_POS_SPI5            (RCC_POS_APB2 + 20)
#define RCC_POS_SPI6            (RCC_POS_APB2 + 21)
#define RCC_POS_SRAM1           (RCC_POS_AHB1 + 16)
#define RCC_POS_SRAM2           (RCC_POS_AHB1 + 17)
#define RCC_POS_SRAM3           (RCC_POS_AHB1 + 19)
#define RCC_POS_SYSCFG          (RCC_POS_APB2 + 14)
#define RCC_POS_TIM1            (RCC_POS_APB2 +  0)
#define RCC_POS_TIM10           (RCC_POS_APB2 + 17)
#define RCC_POS_TIM11           (RCC_POS_APB2 + 18)
#define RCC_POS_TIM12           (RCC_POS_APB1 +  6)
#define RCC_POS_TIM13           (RCC_POS_APB1 +  7)
#define RCC_POS_TIM14           (RCC_POS_APB1 +  8)
#define RCC_POS_TIM2            (RCC_POS_APB1 +  0)
#define RCC_POS_TIM3            (RCC_POS_APB1 +  1)
#define RCC_POS_TIM4            (RCC_POS_APB1 +  2)
#define RCC_POS_TIM5            (RCC_POS_APB1 +  3)
#define RCC_POS_TIM6            (RCC_POS_APB1 +  4)
#define RCC_POS_TIM7            (RCC_POS_APB1 +  5)
#define RCC_POS_TIM8            (RCC_POS_APB2 +  1)
#define RCC_POS_TIM9            (RCC_POS_APB2 + 16)
#define RCC_POS_UART10          (RCC_POS_APB2 +  7)
#define RCC_POS_UART4           (RCC_POS_APB1 + 19)
#define RCC_POS_UART5           (RCC_POS_APB1 + 20)
#define RCC_POS_UART7           (RCC_POS_APB1 + 30)
#define RCC_POS_UART8           (RCC_POS_APB1 + 31)
#define RCC_POS_UART9           (RCC_POS_APB2 +  6)
#define RCC_POS_USART1          (RCC_POS_APB2 +  4)
#define RCC_POS_USART2          (RCC_POS_APB1 + 17)
#define RCC_POS_USART3          (RCC_POS_APB1 + 18)
#define RCC_POS_USART6          (RCC_POS_APB2 +  5)
#define RCC_POS_WWDG            (RCC_POS_APB1 + 11)

/** @addtogroup RCC_Peripheral_Control_Exported_Functions
 * @{ */
void            RCC_vClockEnable        (RCC_PositionType PeriphPos);
void            RCC_vClockDisable       (RCC_PositionType PeriphPos);
void            RCC_vSleepClockEnable   (RCC_PositionType PeriphPos);
void            RCC_vSleepClockDisable  (RCC_PositionType PeriphPos);
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
