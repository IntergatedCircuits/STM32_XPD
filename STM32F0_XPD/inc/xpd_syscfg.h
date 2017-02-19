/**
  ******************************************************************************
  * @file    xpd_syscfg.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-02-07
  * @brief   STM32 eXtensible Peripheral Drivers System Configuration Module
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
#ifndef SYSCFG_REG_BIT

#ifdef SYSCFG_BB
#define SYSCFG_REG_BIT(_REG_NAME_, _BIT_NAME_) (SYSCFG_BB->_REG_NAME_._BIT_NAME_)
#else
#define SYSCFG_REG_BIT(_REG_NAME_, _BIT_NAME_) (SYSCFG->_REG_NAME_.b._BIT_NAME_)
#endif /* SYSCFG_BB */

#endif /* SYSCFG_REG_BIT */

#if defined(XPD_DMA_API)
#ifdef SYSCFG_CFGR1_DMA_RMP

/* Defines for macro functionality */
#ifdef SYSCFG_CFGR1_ADC_DMA_RMP
#define DMA_RMP_ADC_CH1         0U
#define DMA_RMP_ADC_CH2         (SYSCFG_CFGR1_ADC_DMA_RMP)
#define DMA_RMP_ADC_BIT         (SYSCFG_CFGR1_ADC_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_I2C1_DMA_RMP
/* Single bit for both channels */
#define DMA_RMP_I2C1_RX_CH3     0U
#define DMA_RMP_I2C1_RX_CH7     (SYSCFG_CFGR1_I2C1_DMA_RMP)
#define DMA_RMP_I2C1_RX_BIT     (SYSCFG_CFGR1_I2C1_DMA_RMP)
#define DMA_RMP_I2C1_TX_CH2     0U
#define DMA_RMP_I2C1_TX_CH6     (SYSCFG_CFGR1_I2C1_DMA_RMP)
#define DMA_RMP_I2C1_TX_BIT     (SYSCFG_CFGR1_I2C1_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_SPI2_DMA_RMP
/* Single bit for both channels */
#define DMA_RMP_SPI2_RX_CH4     0U
#define DMA_RMP_SPI2_RX_CH6     (SYSCFG_CFGR1_SPI2_DMA_RMP)
#define DMA_RMP_SPI2_RX_BIT     (SYSCFG_CFGR1_SPI2_DMA_RMP)
#define DMA_RMP_SPI2_TX_CH5     0U
#define DMA_RMP_SPI2_TX_CH7     (SYSCFG_CFGR1_SPI2_DMA_RMP)
#define DMA_RMP_SPI2_TX_BIT     (SYSCFG_CFGR1_SPI2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM1_DMA_RMP
#define DMA_RMP_TIM1_CH2        0U
#define DMA_RMP_TIM1_CH3        0U
#define DMA_RMP_TIM1_CH4        0U
#define DMA_RMP_TIM1_CH6        (SYSCFG_CFGR1_TIM1_DMA_RMP)
#define DMA_RMP_TIM1_BIT        (SYSCFG_CFGR1_TIM1_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM2_DMA_RMP
#define DMA_RMP_TIM2_CH3        0U
#define DMA_RMP_TIM2_CH4        0U
#define DMA_RMP_TIM2_CH7        (SYSCFG_CFGR1_TIM2_DMA_RMP)
#define DMA_RMP_TIM2_BIT        (SYSCFG_CFGR1_TIM2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM3_DMA_RMP
#define DMA_RMP_TIM3_CH4        0U
#define DMA_RMP_TIM3_CH6        (SYSCFG_CFGR1_TIM3_DMA_RMP)
#define DMA_RMP_TIM3_BIT        (SYSCFG_CFGR1_TIM3_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_TIM16_DMA_RMP
#ifdef SYSCFG_CFGR1_TIM16_DMA_RMP2
#define DMA_RMP_TIM16_CH3       0U
#define DMA_RMP_TIM16_CH4       (SYSCFG_CFGR1_TIM16_DMA_RMP)
#define DMA_RMP_TIM16_CH6       (SYSCFG_CFGR1_TIM16_DMA_RMP2)
#define DMA_RMP_TIM16_BIT       (SYSCFG_CFGR1_TIM16_DMA_RMP | SYSCFG_CFGR1_TIM16_DMA_RMP2)
#else
#define DMA_RMP_TIM16_CH3       0U
#define DMA_RMP_TIM16_CH4       (SYSCFG_CFGR1_TIM16_DMA_RMP)
#define DMA_RMP_TIM16_BIT       (SYSCFG_CFGR1_TIM16_DMA_RMP)
#endif
#endif

#ifdef SYSCFG_CFGR1_TIM17_DMA_RMP
#ifdef SYSCFG_CFGR1_TIM17_DMA_RMP2
#define DMA_RMP_TIM17_CH1       0U
#define DMA_RMP_TIM17_CH2       (SYSCFG_CFGR1_TIM17_DMA_RMP)
#define DMA_RMP_TIM17_CH7       (SYSCFG_CFGR1_TIM17_DMA_RMP2)
#define DMA_RMP_TIM17_BIT       (SYSCFG_CFGR1_TIM17_DMA_RMP | SYSCFG_CFGR1_TIM17_DMA_RMP2)
#else
#define DMA_RMP_TIM17_CH1       0U
#define DMA_RMP_TIM17_CH2       (SYSCFG_CFGR1_TIM17_DMA_RMP)
#define DMA_RMP_TIM17_BIT       (SYSCFG_CFGR1_TIM17_DMA_RMP)
#endif
#endif

#ifdef SYSCFG_CFGR1_USART1TX_DMA_RMP
#define DMA_RMP_USART1_TX_CH2   0U
#define DMA_RMP_USART1_TX_CH4   (SYSCFG_CFGR1_USART1TX_DMA_RMP)
#define DMA_RMP_USART1_TX_BIT   (SYSCFG_CFGR1_USART1TX_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_USART1RX_DMA_RMP
#define DMA_RMP_USART1_RX_CH3   0U
#define DMA_RMP_USART1_RX_CH5   (SYSCFG_CFGR1_USART1RX_DMA_RMP)
#define DMA_RMP_USART1_RX_BIT   (SYSCFG_CFGR1_USART1RX_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_USART2_DMA_RMP
/* Single bit for both channels */
#define DMA_RMP_USART2_RX_CH5   0U
#define DMA_RMP_USART2_RX_CH7   (SYSCFG_CFGR1_USART2_DMA_RMP)
#define DMA_RMP_USART2_RX_BIT   (SYSCFG_CFGR1_USART2_DMA_RMP)
#define DMA_RMP_USART2_TX_CH4   0U
#define DMA_RMP_USART2_TX_CH6   (SYSCFG_CFGR1_USART2_DMA_RMP)
#define DMA_RMP_USART2_TX_BIT   (SYSCFG_CFGR1_USART2_DMA_RMP)
#endif

#ifdef SYSCFG_CFGR1_USART3_DMA_RMP
/* Single bit for both channels, channel not available without remap */
#define DMA_RMP_USART3_RX_CH3   (SYSCFG_CFGR1_USART3_DMA_RMP)
#define DMA_RMP_USART3_RX_BIT   (SYSCFG_CFGR1_USART3_DMA_RMP)
#define DMA_RMP_USART3_TX_CH2   (SYSCFG_CFGR1_USART3_DMA_RMP)
#define DMA_RMP_USART3_TX_BIT   (SYSCFG_CFGR1_USART3_DMA_RMP)
#endif

/**
 * @brief  Sets the DMA remapping for the given request source.
 * @param  SOURCE: source of DMA request
 * @param  CHANNEL: specifies the DMA channel number to remap to [1..7].
 */
#define XPD_DMA_SourceRemap(SOURCE, CHANNEL)                    \
    (MODIFY_REG(SYSCFG->CFGR1.w, DMA_RMP_##SOURCE##_BIT, DMA_RMP_##SOURCE##_CH##CHANNEL))

#endif /* SYSCFG_CFGR1_DMA_RMP */

#endif
