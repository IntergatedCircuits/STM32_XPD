/**
  ******************************************************************************
  * @file    xpd_rcc_gen.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-10-31
  * @brief   STM32 eXtensible Peripheral Drivers RCC Peripherals Module
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
#ifndef XPD_RCC_GEN_H_
#define XPD_RCC_GEN_H_


/** @addtogroup RCC
 * @{ */

/** @defgroup RCC_Peripheral_Control RCC Peripheral Control
 * @{ */

/** @defgroup RCC_Generated_Functions RCC Generated Functions
 * @{ */
#ifdef RCC_APB2ENR_ADCEN
void XPD_ADC1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_ADCRST
void XPD_ADC1_Reset(void);
#endif
#ifdef RCC_APB1ENR_CANEN
void XPD_CAN_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CANRST
void XPD_CAN_Reset(void);
#endif
#ifdef RCC_APB1ENR_CECEN
void XPD_CEC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CECRST
void XPD_CEC_Reset(void);
#endif
#ifdef RCC_APB2ENR_SYSCFGCOMPEN
void XPD_COMP_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBENR_CRCEN
void XPD_CRC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1ENR_CRSEN
void XPD_CRS_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CRSRST
void XPD_CRS_Reset(void);
#endif
#ifdef RCC_APB1ENR_DACEN
void XPD_DAC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_DACRST
void XPD_DAC_Reset(void);
#endif
#ifdef RCC_APB2ENR_DBGMCUEN
void XPD_DBGMCU_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_DBGMCURST
void XPD_DBGMCU_Reset(void);
#endif
#ifdef RCC_AHBENR_DMAEN
void XPD_DMA1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBENR_DMA2EN
void XPD_DMA2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBENR_FLITFEN
void XPD_FLITF_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBENR_GPIOAEN
void XPD_GPIOA_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOARST
void XPD_GPIOA_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOBEN
void XPD_GPIOB_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOBRST
void XPD_GPIOB_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOCEN
void XPD_GPIOC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOCRST
void XPD_GPIOC_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIODEN
void XPD_GPIOD_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIODRST
void XPD_GPIOD_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOEEN
void XPD_GPIOE_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOERST
void XPD_GPIOE_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOFEN
void XPD_GPIOF_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOFRST
void XPD_GPIOF_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C1EN
void XPD_I2C1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C1RST
void XPD_I2C1_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C2EN
void XPD_I2C2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C2RST
void XPD_I2C2_Reset(void);
#endif
#ifdef RCC_APB1ENR_PWREN
void XPD_PWR_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_PWRRST
void XPD_PWR_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI1EN
void XPD_SPI1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI1RST
void XPD_SPI1_Reset(void);
#endif
#ifdef RCC_APB1ENR_SPI2EN
void XPD_SPI2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_SPI2RST
void XPD_SPI2_Reset(void);
#endif
#ifdef RCC_AHBENR_SRAMEN
void XPD_SRAM_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
void XPD_SYSCFG_Reset(void);
#endif
#ifdef RCC_APB2ENR_SYSCFGCOMPEN
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_TIM1EN
void XPD_TIM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM1RST
void XPD_TIM1_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM14EN
void XPD_TIM14_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM14RST
void XPD_TIM14_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM15EN
void XPD_TIM15_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM15RST
void XPD_TIM15_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM16EN
void XPD_TIM16_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM16RST
void XPD_TIM16_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM17EN
void XPD_TIM17_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM17RST
void XPD_TIM17_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM2EN
void XPD_TIM2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM2RST
void XPD_TIM2_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM3EN
void XPD_TIM3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM3RST
void XPD_TIM3_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM6EN
void XPD_TIM6_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM6RST
void XPD_TIM6_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM7EN
void XPD_TIM7_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM7RST
void XPD_TIM7_Reset(void);
#endif
#ifdef RCC_AHBENR_TSCEN
void XPD_TSC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_TSCRST
void XPD_TSC_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART1EN
void XPD_USART1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART1RST
void XPD_USART1_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART2EN
void XPD_USART2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART2RST
void XPD_USART2_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART3EN
void XPD_USART3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART3RST
void XPD_USART3_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART4EN
void XPD_USART4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART4RST
void XPD_USART4_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART5EN
void XPD_USART5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART5RST
void XPD_USART5_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART6EN
void XPD_USART6_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART6RST
void XPD_USART6_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART7EN
void XPD_USART7_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART7RST
void XPD_USART7_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART8EN
void XPD_USART8_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART8RST
void XPD_USART8_Reset(void);
#endif
#ifdef RCC_APB1ENR_USBEN
void XPD_USB_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USBRST
void XPD_USB_Reset(void);
#endif
#ifdef RCC_APB1ENR_WWDGEN
void XPD_WWDG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_WWDGRST
void XPD_WWDG_Reset(void);
#endif

/** @} */

/** @} */

/** @} */

#endif /* XPD_RCC_GEN_H_ */
