/**
  ******************************************************************************
  * @file    xpd_syscfg.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers System Configuration Module
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
#ifndef SYSCFG_REG_BIT

#ifdef SYSCFG_BB
#define         SYSCFG_REG_BIT(REG_NAME, BIT_NAME)          \
    (SYSCFG_BB->REG_NAME.BIT_NAME)
#else
#define         SYSCFG_REG_BIT(REG_NAME, BIT_NAME)          \
    (SYSCFG->REG_NAME.b.BIT_NAME)
#endif /* SYSCFG_BB */

#define __MEM_RMP_FLASH         0
#define __MEM_RMP_ROM           1
#define __MEM_RMP_SRAM          3
/**
 * @brief Maps the selected memory also starting from 0 address
 * @param MEMORY: The system memory to map
 *          @arg FLASH  Main flash
 *          @arg ROM    System memory
 *          @arg SRAM   Embedded SRAM
 */
#define         SYSTEM_MEMORY_REMAP(MEMORY)                 \
    (SYSCFG->CFGR1.b.MEM_MODE = __MEM_RMP_##MEMORY)

#endif /* SYSCFG_REG_BIT */

#if defined(XPD_GPIO_API)

#ifdef SYSCFG_CFGR1_PA11_PA12_RMP
#define __GPIO_RMP_PA9_PA10     0
#define __GPIO_RMP_PA11_PA12    1
/**
 * @brief Sets the PA9 & 10 pins to PA11 & 12 or back.
 * @param PINS: Either PA9_PA10 or PA11_PA12
 */
#define         GPIO_PIN_REMAP(PINS)                        \
    (SYSCFG_REG_BIT(CFGR1, PA11_PA12_RMP) = __GPIO_RMP_##PINS)
#else
#define         GPIO_PIN_REMAP(PINS)    ((void)0)
#endif

#elif defined(XPD_DMA_API)
#ifdef SYSCFG_CFGR1_DMA_RMP

/* Defines for macro functionality */
#ifdef SYSCFG_CFGR1_ADC_DMA_RMP
#define __DMA_RMP_ADC_CH1       0U
#define __DMA_RMP_ADC_CH2       (SYSCFG_CFGR1_ADC_DMA_RMP)
#define __DMA_RMP_ADC_BIT       (SYSCFG_CFGR1_ADC_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_I2C1_DMA_RMP
/* Single bit for both channels */
#define __DMA_RMP_I2C1_RX_CH3   0U
#define __DMA_RMP_I2C1_RX_CH7   (SYSCFG_CFGR1_I2C1_DMA_RMP)
#define __DMA_RMP_I2C1_RX_BIT   (SYSCFG_CFGR1_I2C1_DMA_RMP)
#define __DMA_RMP_I2C1_TX_CH2   0U
#define __DMA_RMP_I2C1_TX_CH6   (SYSCFG_CFGR1_I2C1_DMA_RMP)
#define __DMA_RMP_I2C1_TX_BIT   (SYSCFG_CFGR1_I2C1_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_SPI2_DMA_RMP
/* Single bit for both channels */
#define __DMA_RMP_SPI2_RX_CH4   0U
#define __DMA_RMP_SPI2_RX_CH6   (SYSCFG_CFGR1_SPI2_DMA_RMP)
#define __DMA_RMP_SPI2_RX_BIT   (SYSCFG_CFGR1_SPI2_DMA_RMP)
#define __DMA_RMP_SPI2_TX_CH5   0U
#define __DMA_RMP_SPI2_TX_CH7   (SYSCFG_CFGR1_SPI2_DMA_RMP)
#define __DMA_RMP_SPI2_TX_BIT   (SYSCFG_CFGR1_SPI2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM1_DMA_RMP
#define __DMA_RMP_TIM1_CH2      0U
#define __DMA_RMP_TIM1_CH3      0U
#define __DMA_RMP_TIM1_CH4      0U
#define __DMA_RMP_TIM1_CH6      (SYSCFG_CFGR1_TIM1_DMA_RMP)
#define __DMA_RMP_TIM1_BIT      (SYSCFG_CFGR1_TIM1_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM2_DMA_RMP
#define __DMA_RMP_TIM2_CH3      0U
#define __DMA_RMP_TIM2_CH4      0U
#define __DMA_RMP_TIM2_CH7      (SYSCFG_CFGR1_TIM2_DMA_RMP)
#define __DMA_RMP_TIM2_BIT      (SYSCFG_CFGR1_TIM2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM3_DMA_RMP
#define __DMA_RMP_TIM3_CH4      0U
#define __DMA_RMP_TIM3_CH6      (SYSCFG_CFGR1_TIM3_DMA_RMP)
#define __DMA_RMP_TIM3_BIT      (SYSCFG_CFGR1_TIM3_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM16_DMA_RMP
#ifdef SYSCFG_CFGR1_TIM16_DMA_RMP2
#define __DMA_RMP_TIM16_CH3     0U
#define __DMA_RMP_TIM16_CH4     (SYSCFG_CFGR1_TIM16_DMA_RMP)
#define __DMA_RMP_TIM16_CH6     (SYSCFG_CFGR1_TIM16_DMA_RMP2)
#define __DMA_RMP_TIM16_BIT     (SYSCFG_CFGR1_TIM16_DMA_RMP | SYSCFG_CFGR1_TIM16_DMA_RMP2)
#else
#define __DMA_RMP_TIM16_CH3     0U
#define __DMA_RMP_TIM16_CH4     (SYSCFG_CFGR1_TIM16_DMA_RMP)
#define __DMA_RMP_TIM16_BIT     (SYSCFG_CFGR1_TIM16_DMA_RMP)
#endif
#endif

#ifdef SYSCFG_CFGR1_TIM17_DMA_RMP
#ifdef SYSCFG_CFGR1_TIM17_DMA_RMP2
#define __DMA_RMP_TIM17_CH1     0U
#define __DMA_RMP_TIM17_CH2     (SYSCFG_CFGR1_TIM17_DMA_RMP)
#define __DMA_RMP_TIM17_CH7     (SYSCFG_CFGR1_TIM17_DMA_RMP2)
#define __DMA_RMP_TIM17_BIT     (SYSCFG_CFGR1_TIM17_DMA_RMP | SYSCFG_CFGR1_TIM17_DMA_RMP2)
#else
#define __DMA_RMP_TIM17_CH1     0U
#define __DMA_RMP_TIM17_CH2     (SYSCFG_CFGR1_TIM17_DMA_RMP)
#define __DMA_RMP_TIM17_BIT     (SYSCFG_CFGR1_TIM17_DMA_RMP)
#endif
#endif

#ifdef SYSCFG_CFGR1_USART1TX_DMA_RMP
#define __DMA_RMP_USART1_TX_CH2 0U
#define __DMA_RMP_USART1_TX_CH4 (SYSCFG_CFGR1_USART1TX_DMA_RMP)
#define __DMA_RMP_USART1_TX_BIT (SYSCFG_CFGR1_USART1TX_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_USART1RX_DMA_RMP
#define __DMA_RMP_USART1_RX_CH3 0U
#define __DMA_RMP_USART1_RX_CH5 (SYSCFG_CFGR1_USART1RX_DMA_RMP)
#define __DMA_RMP_USART1_RX_BIT (SYSCFG_CFGR1_USART1RX_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_USART2_DMA_RMP
/* Single bit for both channels */
#define __DMA_RMP_USART2_RX_CH5 0U
#define __DMA_RMP_USART2_RX_CH7 (SYSCFG_CFGR1_USART2_DMA_RMP)
#define __DMA_RMP_USART2_RX_BIT (SYSCFG_CFGR1_USART2_DMA_RMP)
#define __DMA_RMP_USART2_TX_CH4 0U
#define __DMA_RMP_USART2_TX_CH6 (SYSCFG_CFGR1_USART2_DMA_RMP)
#define __DMA_RMP_USART2_TX_BIT (SYSCFG_CFGR1_USART2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_USART3_DMA_RMP
/* Single bit for both channels, channel not available without remap */
#define __DMA_RMP_USART3_RX_CH3 (SYSCFG_CFGR1_USART3_DMA_RMP)
#define __DMA_RMP_USART3_RX_BIT (SYSCFG_CFGR1_USART3_DMA_RMP)
#define __DMA_RMP_USART3_TX_CH2 (SYSCFG_CFGR1_USART3_DMA_RMP)
#define __DMA_RMP_USART3_TX_BIT (SYSCFG_CFGR1_USART3_DMA_RMP)
#endif

/**
 * @brief  Sets the DMA remapping for the given request source.
 * @param  SOURCE: source of DMA request
 * @param  CHANNEL: specifies the DMA channel number to remap to [1..7].
 */
#define         DMA_SOURCE_REMAP(SOURCE, CHANNEL)           \
    (MODIFY_REG(SYSCFG->CFGR1.w, __DMA_RMP_##SOURCE##_BIT, __DMA_RMP_##SOURCE##_CH##CHANNEL))

#endif /* SYSCFG_CFGR1_DMA_RMP */

#endif
