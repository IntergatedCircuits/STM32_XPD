/**
  ******************************************************************************
  * @file    xpd_rcc_gen.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-04-14
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
#ifdef RCC_AHB2ENR_ADCEN
void XPD_ADC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_ADCSMEN
void XPD_ADC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_ADCRST
void XPD_ADC_Reset(void);
#endif
#ifdef RCC_AHB2ENR_AESEN
void XPD_AES_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_AESSMEN
void XPD_AES_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_AESRST
void XPD_AES_Reset(void);
#endif
#ifdef RCC_APB1ENR1_CAN1EN
void XPD_CAN1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_CAN1SMEN
void XPD_CAN1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_CAN1RST
void XPD_CAN1_Reset(void);
#endif
#ifdef RCC_APB1ENR1_CAN2EN
void XPD_CAN2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_CAN2SMEN
void XPD_CAN2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_CAN2RST
void XPD_CAN2_Reset(void);
#endif
#ifdef RCC_AHB1ENR_CRCEN
void XPD_CRC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1SMENR_CRCSMEN
void XPD_CRC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_CRCRST
void XPD_CRC_Reset(void);
#endif
#ifdef RCC_APB1ENR1_CRSEN
void XPD_CRS_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_CRSSMEN
void XPD_CRS_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_CRSRST
void XPD_CRS_Reset(void);
#endif
#ifdef RCC_APB1ENR1_DAC1EN
void XPD_DAC1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_DAC1SMEN
void XPD_DAC1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_DAC1RST
void XPD_DAC1_Reset(void);
#endif
#ifdef RCC_AHB2ENR_DCMIEN
void XPD_DCMI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_DCMISMEN
void XPD_DCMI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_DCMIRST
void XPD_DCMI_Reset(void);
#endif
#ifdef RCC_APB2ENR_DFSDM1EN
void XPD_DFSDM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_DFSDM1SMEN
void XPD_DFSDM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_DFSDM1RST
void XPD_DFSDM1_Reset(void);
#endif
#ifdef RCC_AHB1ENR_DMA1EN
void XPD_DMA1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1SMENR_DMA1SMEN
void XPD_DMA1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_DMA1RST
void XPD_DMA1_Reset(void);
#endif
#ifdef RCC_AHB1ENR_DMA2EN
void XPD_DMA2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1SMENR_DMA2SMEN
void XPD_DMA2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_DMA2RST
void XPD_DMA2_Reset(void);
#endif
#ifdef RCC_AHB1ENR_DMA2DEN
void XPD_DMA2D_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1SMENR_DMA2DSMEN
void XPD_DMA2D_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_DMA2DRST
void XPD_DMA2D_Reset(void);
#endif
#ifdef RCC_AHB1ENR_FLASHEN
void XPD_FLASH_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1SMENR_FLASHSMEN
void XPD_FLASH_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_FLASHRST
void XPD_FLASH_Reset(void);
#endif
#ifdef RCC_AHB3ENR_FMCEN
void XPD_FMC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3SMENR_FMCSMEN
void XPD_FMC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3RSTR_FMCRST
void XPD_FMC_Reset(void);
#endif
#ifdef RCC_APB2ENR_FWEN
void XPD_FW_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2ENR_GPIOAEN
void XPD_GPIOA_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOASMEN
void XPD_GPIOA_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOARST
void XPD_GPIOA_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIOBEN
void XPD_GPIOB_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOBSMEN
void XPD_GPIOB_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOBRST
void XPD_GPIOB_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIOCEN
void XPD_GPIOC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOCSMEN
void XPD_GPIOC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOCRST
void XPD_GPIOC_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIODEN
void XPD_GPIOD_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIODSMEN
void XPD_GPIOD_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIODRST
void XPD_GPIOD_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIOEEN
void XPD_GPIOE_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOESMEN
void XPD_GPIOE_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOERST
void XPD_GPIOE_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIOFEN
void XPD_GPIOF_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOFSMEN
void XPD_GPIOF_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOFRST
void XPD_GPIOF_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIOGEN
void XPD_GPIOG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOGSMEN
void XPD_GPIOG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOGRST
void XPD_GPIOG_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIOHEN
void XPD_GPIOH_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOHSMEN
void XPD_GPIOH_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOHRST
void XPD_GPIOH_Reset(void);
#endif
#ifdef RCC_AHB2ENR_GPIOIEN
void XPD_GPIOI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_GPIOISMEN
void XPD_GPIOI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_GPIOIRST
void XPD_GPIOI_Reset(void);
#endif
#ifdef RCC_AHB2ENR_HASHEN
void XPD_HASH_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_HASHSMEN
void XPD_HASH_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_HASHRST
void XPD_HASH_Reset(void);
#endif
#ifdef RCC_APB1ENR1_I2C1EN
void XPD_I2C1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_I2C1SMEN
void XPD_I2C1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_I2C1RST
void XPD_I2C1_Reset(void);
#endif
#ifdef RCC_APB1ENR1_I2C2EN
void XPD_I2C2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_I2C2SMEN
void XPD_I2C2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_I2C2RST
void XPD_I2C2_Reset(void);
#endif
#ifdef RCC_APB1ENR1_I2C3EN
void XPD_I2C3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_I2C3SMEN
void XPD_I2C3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_I2C3RST
void XPD_I2C3_Reset(void);
#endif
#ifdef RCC_APB1ENR2_I2C4EN
void XPD_I2C4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR2_I2C4SMEN
void XPD_I2C4_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR2_I2C4RST
void XPD_I2C4_Reset(void);
#endif
#ifdef RCC_APB1ENR1_LCDEN
void XPD_LCD_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_LCDSMEN
void XPD_LCD_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_LCDRST
void XPD_LCD_Reset(void);
#endif
#ifdef RCC_APB1ENR1_LPTIM1EN
void XPD_LPTIM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_LPTIM1SMEN
void XPD_LPTIM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_LPTIM1RST
void XPD_LPTIM1_Reset(void);
#endif
#ifdef RCC_APB1ENR2_LPTIM2EN
void XPD_LPTIM2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR2_LPTIM2SMEN
void XPD_LPTIM2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR2_LPTIM2RST
void XPD_LPTIM2_Reset(void);
#endif
#ifdef RCC_APB1ENR2_LPUART1EN
void XPD_LPUART1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR2_LPUART1SMEN
void XPD_LPUART1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR2_LPUART1RST
void XPD_LPUART1_Reset(void);
#endif
#ifdef RCC_APB1ENR1_OPAMPEN
void XPD_OPAMP_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_OPAMPSMEN
void XPD_OPAMP_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_OPAMPRST
void XPD_OPAMP_Reset(void);
#endif
#ifdef RCC_AHB2ENR_OTGFSEN
void XPD_OTGFS_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_OTGFSSMEN
void XPD_OTGFS_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_OTGFSRST
void XPD_OTGFS_Reset(void);
#endif
#ifdef RCC_APB1ENR1_PWREN
void XPD_PWR_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_PWRSMEN
void XPD_PWR_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_PWRRST
void XPD_PWR_Reset(void);
#endif
#ifdef RCC_AHB3ENR_QSPIEN
void XPD_QSPI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3SMENR_QSPISMEN
void XPD_QSPI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3RSTR_QSPIRST
void XPD_QSPI_Reset(void);
#endif
#ifdef RCC_AHB2ENR_RNGEN
void XPD_RNG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_RNGSMEN
void XPD_RNG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_RNGRST
void XPD_RNG_Reset(void);
#endif
#ifdef RCC_APB1ENR1_RTCAPBEN
void XPD_RTCAPB_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_RTCAPBSMEN
void XPD_RTCAPB_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_SAI1EN
void XPD_SAI1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_SAI1SMEN
void XPD_SAI1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SAI1RST
void XPD_SAI1_Reset(void);
#endif
#ifdef RCC_APB2ENR_SAI2EN
void XPD_SAI2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_SAI2SMEN
void XPD_SAI2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SAI2RST
void XPD_SAI2_Reset(void);
#endif
#ifdef RCC_APB2ENR_SDMMC1EN
void XPD_SDMMC1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_SDMMC1SMEN
void XPD_SDMMC1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SDMMC1RST
void XPD_SDMMC1_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI1EN
void XPD_SPI1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_SPI1SMEN
void XPD_SPI1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI1RST
void XPD_SPI1_Reset(void);
#endif
#ifdef RCC_APB1ENR1_SPI2EN
void XPD_SPI2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_SPI2SMEN
void XPD_SPI2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_SPI2RST
void XPD_SPI2_Reset(void);
#endif
#ifdef RCC_APB1ENR1_SPI3EN
void XPD_SPI3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_SPI3SMEN
void XPD_SPI3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_SPI3RST
void XPD_SPI3_Reset(void);
#endif
#ifdef RCC_AHB1SMENR_SRAM1SMEN
void XPD_SRAM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2SMENR_SRAM2SMEN
void XPD_SRAM2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1ENR2_SWPMI1EN
void XPD_SWPMI1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR2_SWPMI1SMEN
void XPD_SWPMI1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR2_SWPMI1RST
void XPD_SWPMI1_Reset(void);
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_SYSCFGSMEN
void XPD_SYSCFG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
void XPD_SYSCFG_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM1EN
void XPD_TIM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_TIM1SMEN
void XPD_TIM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM1RST
void XPD_TIM1_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM15EN
void XPD_TIM15_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_TIM15SMEN
void XPD_TIM15_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM15RST
void XPD_TIM15_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM16EN
void XPD_TIM16_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_TIM16SMEN
void XPD_TIM16_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM16RST
void XPD_TIM16_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM17EN
void XPD_TIM17_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_TIM17SMEN
void XPD_TIM17_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM17RST
void XPD_TIM17_Reset(void);
#endif
#ifdef RCC_APB1ENR1_TIM2EN
void XPD_TIM2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_TIM2SMEN
void XPD_TIM2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_TIM2RST
void XPD_TIM2_Reset(void);
#endif
#ifdef RCC_APB1ENR1_TIM3EN
void XPD_TIM3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_TIM3SMEN
void XPD_TIM3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_TIM3RST
void XPD_TIM3_Reset(void);
#endif
#ifdef RCC_APB1ENR1_TIM4EN
void XPD_TIM4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_TIM4SMEN
void XPD_TIM4_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_TIM4RST
void XPD_TIM4_Reset(void);
#endif
#ifdef RCC_APB1ENR1_TIM5EN
void XPD_TIM5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_TIM5SMEN
void XPD_TIM5_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_TIM5RST
void XPD_TIM5_Reset(void);
#endif
#ifdef RCC_APB1ENR1_TIM6EN
void XPD_TIM6_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_TIM6SMEN
void XPD_TIM6_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_TIM6RST
void XPD_TIM6_Reset(void);
#endif
#ifdef RCC_APB1ENR1_TIM7EN
void XPD_TIM7_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_TIM7SMEN
void XPD_TIM7_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_TIM7RST
void XPD_TIM7_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM8EN
void XPD_TIM8_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_TIM8SMEN
void XPD_TIM8_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM8RST
void XPD_TIM8_Reset(void);
#endif
#ifdef RCC_AHB1ENR_TSCEN
void XPD_TSC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1SMENR_TSCSMEN
void XPD_TSC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_TSCRST
void XPD_TSC_Reset(void);
#endif
#ifdef RCC_APB1ENR1_UART4EN
void XPD_UART4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_UART4SMEN
void XPD_UART4_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_UART4RST
void XPD_UART4_Reset(void);
#endif
#ifdef RCC_APB1ENR1_UART5EN
void XPD_UART5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_UART5SMEN
void XPD_UART5_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_UART5RST
void XPD_UART5_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART1EN
void XPD_USART1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2SMENR_USART1SMEN
void XPD_USART1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART1RST
void XPD_USART1_Reset(void);
#endif
#ifdef RCC_APB1ENR1_USART2EN
void XPD_USART2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_USART2SMEN
void XPD_USART2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_USART2RST
void XPD_USART2_Reset(void);
#endif
#ifdef RCC_APB1ENR1_USART3EN
void XPD_USART3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_USART3SMEN
void XPD_USART3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_USART3RST
void XPD_USART3_Reset(void);
#endif
#ifdef RCC_APB1ENR1_USBFSEN
void XPD_USBFS_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_USBFSSMEN
void XPD_USBFS_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR1_USBFSRST
void XPD_USBFS_Reset(void);
#endif
#ifdef RCC_APB1ENR1_WWDGEN
void XPD_WWDG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1SMENR1_WWDGSMEN
void XPD_WWDG_SleepClockCtrl(FunctionalState NewState);
#endif

/** @} */

/** @} */

/** @} */

#endif /* XPD_RCC_GEN_H_ */
