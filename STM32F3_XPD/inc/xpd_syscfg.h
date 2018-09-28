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
#define __MEM_RMP_FMC           4
/**
 * @brief Maps the selected memory also starting from 0 address
 * @param MEMORY: The system memory to map
 *          @arg FLASH  Main flash
 *          @arg ROM    System memory
 *          @arg SRAM   Embedded SRAM
 *          @arg FMC    FMC Bank (Only the first two banks)
 */
#define         SYSTEM_MEMORY_REMAP(MEMORY)                 \
    (SYSCFG->CFGR1.b.MEM_MODE = __MEM_RMP_##MEMORY)

#endif /* SYSCFG_REG_BIT */

#if defined(XPD_DMA_API)
#ifdef SYSCFG_CFGR1_DMA_RMP

/* Defines for macro functionality */
#ifdef SYSCFG_CFGR1_ADC24_DMA_RMP
/* DMA 2 */
#define __DMA_RMP_ADC2_CH1        0U
#define __DMA_RMP_ADC2_CH3        (SYSCFG_CFGR1_ADC24_DMA_RMP)
#define __DMA_RMP_ADC2_BIT        (SYSCFG_CFGR1_ADC24_DMA_RMP)
#define __DMA_RMP_ADC4_CH2        0U
#define __DMA_RMP_ADC4_CH4        (SYSCFG_CFGR1_ADC24_DMA_RMP)
#define __DMA_RMP_ADC4_BIT        (SYSCFG_CFGR1_ADC24_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_DAC2Ch1_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_DAC2Ch1_CH_     0U
#define __DMA_RMP_DAC2Ch1_CH5     (SYSCFG_CFGR1_DAC2Ch1_DMA_RMP)
#define __DMA_RMP_DAC2Ch1_BIT     (SYSCFG_CFGR1_DAC2Ch1_DMA_RMP)
#elif defined(SYSCFG_CFGR1_TIM18DAC2Ch1_DMA_RMP)
/* DMA 1 */
#define __DMA_RMP_DAC2Ch1_CH_     0U
#define __DMA_RMP_DAC2Ch1_CH5     (SYSCFG_CFGR1_TIM18DAC2Ch1_DMA_RMP)
#define __DMA_RMP_DAC2Ch1_BIT     (SYSCFG_CFGR1_TIM18DAC2Ch1_DMA_RMP)
#define __DMA_RMP_TIM18_CH_       0U
#define __DMA_RMP_TIM18_CH5       (SYSCFG_CFGR1_TIM18DAC2Ch1_DMA_RMP)
#define __DMA_RMP_TIM18_BIT       (SYSCFG_CFGR1_TIM18DAC2Ch1_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM16_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_TIM16_CH3       0U
#define __DMA_RMP_TIM16_CH6       (SYSCFG_CFGR1_TIM16_DMA_RMP)
#define __DMA_RMP_TIM16_BIT       (SYSCFG_CFGR1_TIM16_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM17_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_TIM17_CH1       0U
#define __DMA_RMP_TIM17_CH7       (SYSCFG_CFGR1_TIM17_DMA_RMP)
#define __DMA_RMP_TIM17_BIT       (SYSCFG_CFGR1_TIM17_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_DAC1Ch1_CH_     0U
#define __DMA_RMP_DAC1Ch1_CH3     (SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP)
#define __DMA_RMP_DAC1Ch1_BIT     (SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP)
#define __DMA_RMP_TIM6_CH_        0U
#define __DMA_RMP_TIM6_CH3        (SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP)
#define __DMA_RMP_TIM6_BIT        (SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_DAC1Ch2_CH_     0U
#define __DMA_RMP_DAC1Ch2_CH3     (SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP)
#define __DMA_RMP_DAC1Ch2_BIT     (SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP)
#define __DMA_RMP_TIM7_CH_        0U
#define __DMA_RMP_TIM7_CH3        (SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP)
#define __DMA_RMP_TIM7_BIT        (SYSCFG_CFGR1_TIM7DAC1Ch2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR3_DMA_RMP

#ifdef SYSCFG_CFGR3_ADC2_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_ADC2_CH2        (0x03000000 | SYSCFG_CFGR3_ADC2___DMA_RMP_0)
#define __DMA_RMP_ADC2_CH4        (0x03000000 | SYSCFG_CFGR3_ADC2_DMA_RMP)
#define __DMA_RMP_ADC2_BIT        (0x03000000 | SYSCFG_CFGR3_ADC2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR3_SPI1_RX_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_SPI1_RX_CH2     (0x03000000 | SYSCFG_CFGR3_SPI1_RX_DMA_RMP)
#define __DMA_RMP_SPI1_RX_CH4     (0x03000000 | SYSCFG_CFGR3_SPI1_RX___DMA_RMP_0)
#define __DMA_RMP_SPI1_RX_CH6     (0x03000000 | SYSCFG_CFGR3_SPI1_RX___DMA_RMP_1)
#define __DMA_RMP_SPI1_RX_BIT     (0x03000000 | SYSCFG_CFGR3_SPI1_RX_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR3_SPI1_TX_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_SPI1_TX_CH3     (0x03000000 | SYSCFG_CFGR3_SPI1_TX_DMA_RMP)
#define __DMA_RMP_SPI1_TX_CH5     (0x03000000 | SYSCFG_CFGR3_SPI1_TX___DMA_RMP_0)
#define __DMA_RMP_SPI1_TX_CH7     (0x03000000 | SYSCFG_CFGR3_SPI1_TX___DMA_RMP_1)
#define __DMA_RMP_SPI1_TX_BIT     (0x03000000 | SYSCFG_CFGR3_SPI1_TX_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR3_SPI1_RX_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_I2C1_RX_CH3     (0x03000000 | SYSCFG_CFGR3_I2C1_RX___DMA_RMP_0)
#define __DMA_RMP_I2C1_RX_CH5     (0x03000000 | SYSCFG_CFGR3_I2C1_RX___DMA_RMP_1)
#define __DMA_RMP_I2C1_RX_CH7     (0x03000000 | SYSCFG_CFGR3_I2C1_RX_DMA_RMP)
#define __DMA_RMP_I2C1_RX_BIT     (0x03000000 | SYSCFG_CFGR3_I2C1_RX_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR3_I2C1_TX_DMA_RMP
/* DMA 1 */
#define __DMA_RMP_I2C1_TX_CH2     (0x03000000 | SYSCFG_CFGR3_I2C1_TX___DMA_RMP_0)
#define __DMA_RMP_I2C1_TX_CH4     (0x03000000 | SYSCFG_CFGR3_I2C1_TX___DMA_RMP_1)
#define __DMA_RMP_I2C1_TX_CH6     (0x03000000 | SYSCFG_CFGR3_I2C1_TX_DMA_RMP)
#define __DMA_RMP_I2C1_TX_BIT     (0x03000000 | SYSCFG_CFGR3_I2C1_TX_DMA_RMP)
#endif

/**
 * @brief  Sets the DMA remapping for the given request source.
 * @param  SOURCE: source of DMA request
 * @param  CHANNEL: specifies the DMA channel number to remap to [1..7].
 */
#define         DMA_SOURCE_REMAP(SOURCE, CHANNEL)           \
    (MODIFY_REG(((__IO uint32_t *)SYSCFG)[__DMA_RMP_##SOURCE##_BIT >> 24], __DMA_RMP_##SOURCE##_BIT & 0xFFFFFF, __DMA_RMP_##SOURCE##_CH##CHANNEL))

#else

/**
 * @brief  Sets the DMA remapping for the given request source.
 * @param  SOURCE: source of DMA request
 * @param  CHANNEL: specifies the DMA channel number to remap to [1..7].
 */
#define         DMA_SOURCE_REMAP(SOURCE, CHANNEL)           \
    (MODIFY_REG(SYSCFG->CFGR1.w, __DMA_RMP_##SOURCE##_BIT, __DMA_RMP_##SOURCE##_CH##CHANNEL))

#endif

#endif /* SYSCFG_CFGR1_DMA_RMP */

#elif defined(XPD_USB_API)

#ifdef SYSCFG_CFGR1_USB_IT_RMP
/**
 * @brief  Enables or disables the USB interrupt remapping.
 * @param  NEWSTATE: the new remap setting
 */
#define         USB_IT_REMAP(NEWSTATE)                      \
    (SYSCFG_REG_BIT(CFGR1, USB_IT_RMP) = (NEWSTATE))
#endif

#endif
