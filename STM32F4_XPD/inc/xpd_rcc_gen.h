/**
  ******************************************************************************
  * @file    xpd_rcc_gen.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2016-11-05
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
#ifdef RCC_APB2RSTR_ADCRST
void XPD_ADC_Reset(void);
#endif
#ifdef RCC_APB2ENR_ADC1EN
void XPD_ADC1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_ADC1LPEN
void XPD_ADC1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_ADC2EN
void XPD_ADC2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_ADC2LPEN
void XPD_ADC2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_ADC3EN
void XPD_ADC3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_ADC3LPEN
void XPD_ADC3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2ENR_AESEN
void XPD_AES_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2LPENR_AESLPEN
void XPD_AES_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_AESRST
void XPD_AES_Reset(void);
#endif
#ifdef RCC_AHB1ENR_BKPSRAMEN
void XPD_BKPSRAM_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_BKPSRAMLPEN
void XPD_BKPSRAM_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1ENR_CAN1EN
void XPD_CAN1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_CAN1LPEN
void XPD_CAN1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CAN1RST
void XPD_CAN1_Reset(void);
#endif
#ifdef RCC_APB1ENR_CAN2EN
void XPD_CAN2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_CAN2LPEN
void XPD_CAN2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CAN2RST
void XPD_CAN2_Reset(void);
#endif
#ifdef RCC_APB1ENR_CAN3EN
void XPD_CAN3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_CAN3LPEN
void XPD_CAN3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CAN3RST
void XPD_CAN3_Reset(void);
#endif
#ifdef RCC_AHB1ENR_CCMDATARAMEN
void XPD_CCMDATARAM_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1ENR_CECEN
void XPD_CEC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_CECLPEN
void XPD_CEC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CECRST
void XPD_CEC_Reset(void);
#endif
#ifdef RCC_AHB1ENR_CRCEN
void XPD_CRC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_CRCLPEN
void XPD_CRC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_CRCRST
void XPD_CRC_Reset(void);
#endif
#ifdef RCC_AHB2ENR_CRYPEN
void XPD_CRYP_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2LPENR_CRYPLPEN
void XPD_CRYP_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_CRYPRST
void XPD_CRYP_Reset(void);
#endif
#ifdef RCC_APB1ENR_DACEN
void XPD_DAC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_DACLPEN
void XPD_DAC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_DACRST
void XPD_DAC_Reset(void);
#endif
#ifdef RCC_AHB2ENR_DCMIEN
void XPD_DCMI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2LPENR_DCMILPEN
void XPD_DCMI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_DCMIRST
void XPD_DCMI_Reset(void);
#endif
#ifdef RCC_APB2ENR_DFSDM1EN
void XPD_DFSDM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_DFSDM1LPEN
void XPD_DFSDM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_DFSDM1RST
void XPD_DFSDM1_Reset(void);
#endif
#ifdef RCC_APB2ENR_DFSDM2EN
void XPD_DFSDM2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_DFSDM2LPEN
void XPD_DFSDM2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_DFSDM2RST
void XPD_DFSDM2_Reset(void);
#endif
#ifdef RCC_AHB1ENR_DMA1EN
void XPD_DMA1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_DMA1LPEN
void XPD_DMA1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_DMA1RST
void XPD_DMA1_Reset(void);
#endif
#ifdef RCC_AHB1ENR_DMA2EN
void XPD_DMA2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_DMA2LPEN
void XPD_DMA2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_DMA2RST
void XPD_DMA2_Reset(void);
#endif
#ifdef RCC_AHB1ENR_DMA2DEN
void XPD_DMA2D_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_DMA2DLPEN
void XPD_DMA2D_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_DMA2DRST
void XPD_DMA2D_Reset(void);
#endif
#ifdef RCC_APB2ENR_DSIEN
void XPD_DSI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_DSILPEN
void XPD_DSI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_DSIRST
void XPD_DSI_Reset(void);
#endif
#ifdef RCC_AHB1ENR_ETHMACEN
void XPD_ETHMAC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_ETHMACLPEN
void XPD_ETHMAC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_ETHMACRST
void XPD_ETHMAC_Reset(void);
#endif
#ifdef RCC_AHB1ENR_ETHMACPTPEN
void XPD_ETHMAC_PTP_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_ETHMACPTPLPEN
void XPD_ETHMAC_PTP_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1ENR_ETHMACRXEN
void XPD_ETHMAC_RX_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_ETHMACRXLPEN
void XPD_ETHMAC_RX_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1ENR_ETHMACTXEN
void XPD_ETHMAC_TX_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_ETHMACTXLPEN
void XPD_ETHMAC_TX_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_EXTITEN
void XPD_EXTI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_EXTITLPEN
void XPD_EXTI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_FLITFLPEN
void XPD_FLITF_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3ENR_FMCEN
void XPD_FMC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3LPENR_FMCLPEN
void XPD_FMC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3RSTR_FMCRST
void XPD_FMC_Reset(void);
#endif
#ifdef RCC_APB1ENR_FMPI2C1EN
void XPD_FMP_I2C1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_FMPI2C1LPEN
void XPD_FMP_I2C1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_FMPI2C1RST
void XPD_FMP_I2C1_Reset(void);
#endif
#ifdef RCC_AHB3ENR_FSMCEN
void XPD_FSMC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3LPENR_FSMCLPEN
void XPD_FSMC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3RSTR_FSMCRST
void XPD_FSMC_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOAEN
void XPD_GPIOA_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOALPEN
void XPD_GPIOA_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOARST
void XPD_GPIOA_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOBEN
void XPD_GPIOB_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOBLPEN
void XPD_GPIOB_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOBRST
void XPD_GPIOB_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOCEN
void XPD_GPIOC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOCLPEN
void XPD_GPIOC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOCRST
void XPD_GPIOC_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIODEN
void XPD_GPIOD_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIODLPEN
void XPD_GPIOD_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIODRST
void XPD_GPIOD_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOEEN
void XPD_GPIOE_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOELPEN
void XPD_GPIOE_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOERST
void XPD_GPIOE_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOFEN
void XPD_GPIOF_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOFLPEN
void XPD_GPIOF_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOFRST
void XPD_GPIOF_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOGEN
void XPD_GPIOG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOGLPEN
void XPD_GPIOG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOGRST
void XPD_GPIOG_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOHEN
void XPD_GPIOH_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOHLPEN
void XPD_GPIOH_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOHRST
void XPD_GPIOH_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOIEN
void XPD_GPIOI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOILPEN
void XPD_GPIOI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOIRST
void XPD_GPIOI_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOJEN
void XPD_GPIOJ_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOJLPEN
void XPD_GPIOJ_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOJRST
void XPD_GPIOJ_Reset(void);
#endif
#ifdef RCC_AHB1ENR_GPIOKEN
void XPD_GPIOK_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_GPIOKLPEN
void XPD_GPIOK_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_GPIOKRST
void XPD_GPIOK_Reset(void);
#endif
#ifdef RCC_AHB2ENR_HASHEN
void XPD_HASH_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2LPENR_HASHLPEN
void XPD_HASH_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_HASHRST
void XPD_HASH_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C1EN
void XPD_I2C1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_I2C1LPEN
void XPD_I2C1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C1RST
void XPD_I2C1_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C2EN
void XPD_I2C2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_I2C2LPEN
void XPD_I2C2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C2RST
void XPD_I2C2_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C3EN
void XPD_I2C3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_I2C3LPEN
void XPD_I2C3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C3RST
void XPD_I2C3_Reset(void);
#endif
#ifdef RCC_APB1ENR_LPTIM1EN
void XPD_LPTIM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_LPTIM1LPEN
void XPD_LPTIM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_LPTIM1RST
void XPD_LPTIM1_Reset(void);
#endif
#ifdef RCC_APB2ENR_LTDCEN
void XPD_LTDC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_LTDCLPEN
void XPD_LTDC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_LTDCRST
void XPD_LTDC_Reset(void);
#endif
#ifdef RCC_AHB2ENR_OTGFSEN
void XPD_OTG_FS_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2LPENR_OTGFSLPEN
void XPD_OTG_FS_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2RSTR_OTGFSRST
void XPD_OTG_FS_Reset(void);
#endif
#ifdef RCC_AHB1RSTR_OTGHRST
void XPD_OTG_HS_Reset(void);
#endif
#ifdef RCC_AHB1ENR_OTGHSEN
void XPD_OTG_HS_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_OTGHSLPEN
void XPD_OTG_HS_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1ENR_OTGHSULPIEN
void XPD_OTG_HS_ULPI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_OTGHSULPILPEN
void XPD_OTG_HS_ULPI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1ENR_PWREN
void XPD_PWR_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_PWRLPEN
void XPD_PWR_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_PWRRST
void XPD_PWR_Reset(void);
#endif
#ifdef RCC_AHB3ENR_QSPIEN
void XPD_QSPI_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3LPENR_QSPILPEN
void XPD_QSPI_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB3RSTR_QSPIRST
void XPD_QSPI_Reset(void);
#endif
#ifdef RCC_AHB1ENR_RNGEN
void XPD_RNG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2ENR_RNGEN
void XPD_RNG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB2LPENR_RNGLPEN
void XPD_RNG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_RNGLPEN
void XPD_RNG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1RSTR_RNGRST
void XPD_RNG_Reset(void);
#endif
#ifdef RCC_AHB2RSTR_RNGRST
void XPD_RNG_Reset(void);
#endif
#ifdef RCC_APB1ENR_RTCAPBEN
void XPD_RTC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_RTCAPBLPEN
void XPD_RTC_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_SAI1EN
void XPD_SAI1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SAI1LPEN
void XPD_SAI1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SAI1RST
void XPD_SAI1_Reset(void);
#endif
#ifdef RCC_APB2ENR_SAI2EN
void XPD_SAI2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SAI2LPEN
void XPD_SAI2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SAI2RST
void XPD_SAI2_Reset(void);
#endif
#ifdef RCC_APB2ENR_SDIOEN
void XPD_SDIO_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SDIOLPEN
void XPD_SDIO_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SDIORST
void XPD_SDIO_Reset(void);
#endif
#ifdef RCC_APB1ENR_SPDIFRXEN
void XPD_SPDIF_RX_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_SPDIFRXLPEN
void XPD_SPDIF_RX_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_SPDIFRXRST
void XPD_SPDIF_RX_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI1EN
void XPD_SPI1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SPI1LPEN
void XPD_SPI1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI1RST
void XPD_SPI1_Reset(void);
#endif
#ifdef RCC_APB1ENR_SPI2EN
void XPD_SPI2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_SPI2LPEN
void XPD_SPI2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_SPI2RST
void XPD_SPI2_Reset(void);
#endif
#ifdef RCC_APB1ENR_SPI3EN
void XPD_SPI3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_SPI3LPEN
void XPD_SPI3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_SPI3RST
void XPD_SPI3_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI4EN
void XPD_SPI4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SPI4LPEN
void XPD_SPI4_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI4RST
void XPD_SPI4_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI5EN
void XPD_SPI5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SPI5LPEN
void XPD_SPI5_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI5RST
void XPD_SPI5_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI6EN
void XPD_SPI6_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SPI6LPEN
void XPD_SPI6_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI6RST
void XPD_SPI6_Reset(void);
#endif
#ifdef RCC_AHB1LPENR_SRAM1LPEN
void XPD_SRAM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_SRAM2LPEN
void XPD_SRAM2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHB1LPENR_SRAM3LPEN
void XPD_SRAM3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_SYSCFGLPEN
void XPD_SYSCFG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
void XPD_SYSCFG_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM1EN
void XPD_TIM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_TIM1LPEN
void XPD_TIM1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM1RST
void XPD_TIM1_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM10EN
void XPD_TIM10_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_TIM10LPEN
void XPD_TIM10_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM10RST
void XPD_TIM10_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM11EN
void XPD_TIM11_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_TIM11LPEN
void XPD_TIM11_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM11RST
void XPD_TIM11_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM12EN
void XPD_TIM12_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM12LPEN
void XPD_TIM12_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM12RST
void XPD_TIM12_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM13EN
void XPD_TIM13_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM13LPEN
void XPD_TIM13_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM13RST
void XPD_TIM13_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM14EN
void XPD_TIM14_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM14LPEN
void XPD_TIM14_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM14RST
void XPD_TIM14_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM2EN
void XPD_TIM2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM2LPEN
void XPD_TIM2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM2RST
void XPD_TIM2_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM3EN
void XPD_TIM3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM3LPEN
void XPD_TIM3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM3RST
void XPD_TIM3_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM4EN
void XPD_TIM4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM4LPEN
void XPD_TIM4_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM4RST
void XPD_TIM4_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM5EN
void XPD_TIM5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM5LPEN
void XPD_TIM5_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM5RST
void XPD_TIM5_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM6EN
void XPD_TIM6_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM6LPEN
void XPD_TIM6_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM6RST
void XPD_TIM6_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM7EN
void XPD_TIM7_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_TIM7LPEN
void XPD_TIM7_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM7RST
void XPD_TIM7_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM8EN
void XPD_TIM8_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_TIM8LPEN
void XPD_TIM8_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM8RST
void XPD_TIM8_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM9EN
void XPD_TIM9_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_TIM9LPEN
void XPD_TIM9_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM9RST
void XPD_TIM9_Reset(void);
#endif
#ifdef RCC_APB2ENR_UART10EN
void XPD_UART10_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_UART10LPEN
void XPD_UART10_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_UART10RST
void XPD_UART10_Reset(void);
#endif
#ifdef RCC_APB1ENR_UART4EN
void XPD_UART4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_UART4LPEN
void XPD_UART4_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_UART4RST
void XPD_UART4_Reset(void);
#endif
#ifdef RCC_APB1ENR_UART5EN
void XPD_UART5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_UART5LPEN
void XPD_UART5_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_UART5RST
void XPD_UART5_Reset(void);
#endif
#ifdef RCC_APB1ENR_UART7EN
void XPD_UART7_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_UART7LPEN
void XPD_UART7_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_UART7RST
void XPD_UART7_Reset(void);
#endif
#ifdef RCC_APB1ENR_UART8EN
void XPD_UART8_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_UART8LPEN
void XPD_UART8_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_UART8RST
void XPD_UART8_Reset(void);
#endif
#ifdef RCC_APB2ENR_UART9EN
void XPD_UART9_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_UART9LPEN
void XPD_UART9_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_UART9RST
void XPD_UART9_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART1EN
void XPD_USART1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_USART1LPEN
void XPD_USART1_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART1RST
void XPD_USART1_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART2EN
void XPD_USART2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_USART2LPEN
void XPD_USART2_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART2RST
void XPD_USART2_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART3EN
void XPD_USART3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_USART3LPEN
void XPD_USART3_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART3RST
void XPD_USART3_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART6EN
void XPD_USART6_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2LPENR_USART6LPEN
void XPD_USART6_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART6RST
void XPD_USART6_Reset(void);
#endif
#ifdef RCC_APB1ENR_WWDGEN
void XPD_WWDG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1LPENR_WWDGLPEN
void XPD_WWDG_SleepClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_WWDGRST
void XPD_WWDG_Reset(void);
#endif

/** @} */

/** @} */

/** @} */

#endif /* XPD_RCC_GEN_H_ */
