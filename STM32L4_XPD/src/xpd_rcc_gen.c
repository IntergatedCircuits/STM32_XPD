/**
  ******************************************************************************
  * @file    xpd_rcc_gen.c
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
#include "xpd_rcc.h"

/** @addtogroup RCC
 * @{ */

/** @addtogroup RCC_Peripheral_Control
 * @{ */

/** @addtogroup RCC_Generated_Functions
 * @{ */
#ifdef RCC_AHB2ENR_ADCEN
/** @brief Sets the new clock state of the ADC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_ADC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,ADCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_ADCSMEN
/** @brief Sets the clock state of the ADC peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_ADC_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,ADCSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_ADCRST
/** @brief Forces and releases a reset on the ADC peripheral. */
void XPD_ADC_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,ADCRST) = 1;
    RCC_REG_BIT(AHB2RSTR,ADCRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_AESEN
/** @brief Sets the new clock state of the AES peripheral.
 *  @param NewState: the new clock state to set */
void XPD_AES_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,AESEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_AESSMEN
/** @brief Sets the clock state of the AES peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_AES_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,AESSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_AESRST
/** @brief Forces and releases a reset on the AES peripheral. */
void XPD_AES_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,AESRST) = 1;
    RCC_REG_BIT(AHB2RSTR,AESRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_CAN1EN
/** @brief Sets the new clock state of the CAN1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CAN1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,CAN1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_CAN1SMEN
/** @brief Sets the clock state of the CAN1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_CAN1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,CAN1SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_CAN1RST
/** @brief Forces and releases a reset on the CAN1 peripheral. */
void XPD_CAN1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,CAN1RST) = 1;
    RCC_REG_BIT(APB1RSTR1,CAN1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_CAN2EN
/** @brief Sets the new clock state of the CAN2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CAN2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,CAN2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_CAN2SMEN
/** @brief Sets the clock state of the CAN2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_CAN2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,CAN2SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_CAN2RST
/** @brief Forces and releases a reset on the CAN2 peripheral. */
void XPD_CAN2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,CAN2RST) = 1;
    RCC_REG_BIT(APB1RSTR1,CAN2RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_CRCEN
/** @brief Sets the new clock state of the CRC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CRC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,CRCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR.w;
}
#endif
#ifdef RCC_AHB1SMENR_CRCSMEN
/** @brief Sets the clock state of the CRC peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_CRC_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1SMENR,CRCSMEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_CRCRST
/** @brief Forces and releases a reset on the CRC peripheral. */
void XPD_CRC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,CRCRST) = 1;
    RCC_REG_BIT(AHB1RSTR,CRCRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_CRSEN
/** @brief Sets the new clock state of the CRS peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CRS_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,CRSEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_CRSSMEN
/** @brief Sets the clock state of the CRS peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_CRS_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,CRSSMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_CRSRST
/** @brief Forces and releases a reset on the CRS peripheral. */
void XPD_CRS_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,CRSRST) = 1;
    RCC_REG_BIT(APB1RSTR1,CRSRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_DAC1EN
/** @brief Sets the new clock state of the DAC1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DAC1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,DAC1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_DAC1SMEN
/** @brief Sets the clock state of the DAC1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_DAC1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,DAC1SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_DAC1RST
/** @brief Forces and releases a reset on the DAC1 peripheral. */
void XPD_DAC1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,DAC1RST) = 1;
    RCC_REG_BIT(APB1RSTR1,DAC1RST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_DCMIEN
/** @brief Sets the new clock state of the DCMI peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DCMI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,DCMIEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_DCMISMEN
/** @brief Sets the clock state of the DCMI peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_DCMI_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,DCMISMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_DCMIRST
/** @brief Forces and releases a reset on the DCMI peripheral. */
void XPD_DCMI_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,DCMIRST) = 1;
    RCC_REG_BIT(AHB2RSTR,DCMIRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_DFSDM1EN
/** @brief Sets the new clock state of the DFSDM1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DFSDM1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,DFSDM1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_DFSDM1SMEN
/** @brief Sets the clock state of the DFSDM1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_DFSDM1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,DFSDM1SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_DFSDM1RST
/** @brief Forces and releases a reset on the DFSDM1 peripheral. */
void XPD_DFSDM1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,DFSDM1RST) = 1;
    RCC_REG_BIT(APB2RSTR,DFSDM1RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA1EN
/** @brief Sets the new clock state of the DMA1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DMA1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,DMA1EN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR.w;
}
#endif
#ifdef RCC_AHB1SMENR_DMA1SMEN
/** @brief Sets the clock state of the DMA1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_DMA1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1SMENR,DMA1SMEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_DMA1RST
/** @brief Forces and releases a reset on the DMA1 peripheral. */
void XPD_DMA1_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA1RST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA1RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA2EN
/** @brief Sets the new clock state of the DMA2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DMA2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,DMA2EN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR.w;
}
#endif
#ifdef RCC_AHB1SMENR_DMA2SMEN
/** @brief Sets the clock state of the DMA2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_DMA2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1SMENR,DMA2SMEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_DMA2RST
/** @brief Forces and releases a reset on the DMA2 peripheral. */
void XPD_DMA2_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA2RST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA2RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA2DEN
/** @brief Sets the new clock state of the DMA2D peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DMA2D_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,DMA2DEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR.w;
}
#endif
#ifdef RCC_AHB1SMENR_DMA2DSMEN
/** @brief Sets the clock state of the DMA2D peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_DMA2D_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1SMENR,DMA2DSMEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_DMA2DRST
/** @brief Forces and releases a reset on the DMA2D peripheral. */
void XPD_DMA2D_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA2DRST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA2DRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_FLASHEN
/** @brief Sets the new clock state of the FLASH peripheral.
 *  @param NewState: the new clock state to set */
void XPD_FLASH_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,FLASHEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR.w;
}
#endif
#ifdef RCC_AHB1SMENR_FLASHSMEN
/** @brief Sets the clock state of the FLASH peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_FLASH_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1SMENR,FLASHSMEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_FLASHRST
/** @brief Forces and releases a reset on the FLASH peripheral. */
void XPD_FLASH_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,FLASHRST) = 1;
    RCC_REG_BIT(AHB1RSTR,FLASHRST) = 0;
}
#endif
#ifdef RCC_AHB3ENR_FMCEN
/** @brief Sets the new clock state of the FMC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_FMC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3ENR,FMCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB3ENR.w;
}
#endif
#ifdef RCC_AHB3SMENR_FMCSMEN
/** @brief Sets the clock state of the FMC peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_FMC_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3SMENR,FMCSMEN) = NewState;
}
#endif
#ifdef RCC_AHB3RSTR_FMCRST
/** @brief Forces and releases a reset on the FMC peripheral. */
void XPD_FMC_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,FMCRST) = 1;
    RCC_REG_BIT(AHB3RSTR,FMCRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_FWEN
/** @brief Sets the new clock state of the FW peripheral.
 *  @param NewState: the new clock state to set */
void XPD_FW_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,FWEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_AHB2ENR_GPIOAEN
/** @brief Sets the new clock state of the GPIOA peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOA_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOAEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOASMEN
/** @brief Sets the clock state of the GPIOA peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOA_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOASMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOARST
/** @brief Forces and releases a reset on the GPIOA peripheral. */
void XPD_GPIOA_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOARST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOARST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIOBEN
/** @brief Sets the new clock state of the GPIOB peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOB_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOBEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOBSMEN
/** @brief Sets the clock state of the GPIOB peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOB_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOBSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOBRST
/** @brief Forces and releases a reset on the GPIOB peripheral. */
void XPD_GPIOB_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOBRST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOBRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIOCEN
/** @brief Sets the new clock state of the GPIOC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOCSMEN
/** @brief Sets the clock state of the GPIOC peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOC_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOCSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOCRST
/** @brief Forces and releases a reset on the GPIOC peripheral. */
void XPD_GPIOC_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOCRST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOCRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIODEN
/** @brief Sets the new clock state of the GPIOD peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOD_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIODEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIODSMEN
/** @brief Sets the clock state of the GPIOD peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOD_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIODSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIODRST
/** @brief Forces and releases a reset on the GPIOD peripheral. */
void XPD_GPIOD_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIODRST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIODRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIOEEN
/** @brief Sets the new clock state of the GPIOE peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOE_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOEEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOESMEN
/** @brief Sets the clock state of the GPIOE peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOE_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOESMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOERST
/** @brief Forces and releases a reset on the GPIOE peripheral. */
void XPD_GPIOE_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOERST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOERST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIOFEN
/** @brief Sets the new clock state of the GPIOF peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOF_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOFEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOFSMEN
/** @brief Sets the clock state of the GPIOF peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOF_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOFSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOFRST
/** @brief Forces and releases a reset on the GPIOF peripheral. */
void XPD_GPIOF_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOFRST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOFRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIOGEN
/** @brief Sets the new clock state of the GPIOG peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOGEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOGSMEN
/** @brief Sets the clock state of the GPIOG peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOG_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOGSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOGRST
/** @brief Forces and releases a reset on the GPIOG peripheral. */
void XPD_GPIOG_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOGRST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOGRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIOHEN
/** @brief Sets the new clock state of the GPIOH peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOH_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOHEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOHSMEN
/** @brief Sets the clock state of the GPIOH peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOH_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOHSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOHRST
/** @brief Forces and releases a reset on the GPIOH peripheral. */
void XPD_GPIOH_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOHRST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOHRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_GPIOIEN
/** @brief Sets the new clock state of the GPIOI peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,GPIOIEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_GPIOISMEN
/** @brief Sets the clock state of the GPIOI peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_GPIOI_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,GPIOISMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_GPIOIRST
/** @brief Forces and releases a reset on the GPIOI peripheral. */
void XPD_GPIOI_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,GPIOIRST) = 1;
    RCC_REG_BIT(AHB2RSTR,GPIOIRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_HASHEN
/** @brief Sets the new clock state of the HASH peripheral.
 *  @param NewState: the new clock state to set */
void XPD_HASH_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,HASHEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_HASHSMEN
/** @brief Sets the clock state of the HASH peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_HASH_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,HASHSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_HASHRST
/** @brief Forces and releases a reset on the HASH peripheral. */
void XPD_HASH_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,HASHRST) = 1;
    RCC_REG_BIT(AHB2RSTR,HASHRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_I2C1EN
/** @brief Sets the new clock state of the I2C1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_I2C1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,I2C1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_I2C1SMEN
/** @brief Sets the clock state of the I2C1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_I2C1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,I2C1SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_I2C1RST
/** @brief Forces and releases a reset on the I2C1 peripheral. */
void XPD_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,I2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR1,I2C1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_I2C2EN
/** @brief Sets the new clock state of the I2C2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_I2C2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,I2C2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_I2C2SMEN
/** @brief Sets the clock state of the I2C2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_I2C2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,I2C2SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_I2C2RST
/** @brief Forces and releases a reset on the I2C2 peripheral. */
void XPD_I2C2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,I2C2RST) = 1;
    RCC_REG_BIT(APB1RSTR1,I2C2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_I2C3EN
/** @brief Sets the new clock state of the I2C3 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_I2C3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,I2C3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_I2C3SMEN
/** @brief Sets the clock state of the I2C3 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_I2C3_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,I2C3SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_I2C3RST
/** @brief Forces and releases a reset on the I2C3 peripheral. */
void XPD_I2C3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,I2C3RST) = 1;
    RCC_REG_BIT(APB1RSTR1,I2C3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR2_I2C4EN
/** @brief Sets the new clock state of the I2C4 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_I2C4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR2,I2C4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR2.w;
}
#endif
#ifdef RCC_APB1SMENR2_I2C4SMEN
/** @brief Sets the clock state of the I2C4 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_I2C4_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR2,I2C4SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR2_I2C4RST
/** @brief Forces and releases a reset on the I2C4 peripheral. */
void XPD_I2C4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR2,I2C4RST) = 1;
    RCC_REG_BIT(APB1RSTR2,I2C4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_LCDEN
/** @brief Sets the new clock state of the LCD peripheral.
 *  @param NewState: the new clock state to set */
void XPD_LCD_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,LCDEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_LCDSMEN
/** @brief Sets the clock state of the LCD peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_LCD_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,LCDSMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_LCDRST
/** @brief Forces and releases a reset on the LCD peripheral. */
void XPD_LCD_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,LCDRST) = 1;
    RCC_REG_BIT(APB1RSTR1,LCDRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_LPTIM1EN
/** @brief Sets the new clock state of the LPTIM1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_LPTIM1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,LPTIM1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_LPTIM1SMEN
/** @brief Sets the clock state of the LPTIM1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_LPTIM1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,LPTIM1SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_LPTIM1RST
/** @brief Forces and releases a reset on the LPTIM1 peripheral. */
void XPD_LPTIM1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,LPTIM1RST) = 1;
    RCC_REG_BIT(APB1RSTR1,LPTIM1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR2_LPTIM2EN
/** @brief Sets the new clock state of the LPTIM2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_LPTIM2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR2,LPTIM2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR2.w;
}
#endif
#ifdef RCC_APB1SMENR2_LPTIM2SMEN
/** @brief Sets the clock state of the LPTIM2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_LPTIM2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR2,LPTIM2SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR2_LPTIM2RST
/** @brief Forces and releases a reset on the LPTIM2 peripheral. */
void XPD_LPTIM2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR2,LPTIM2RST) = 1;
    RCC_REG_BIT(APB1RSTR2,LPTIM2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR2_LPUART1EN
/** @brief Sets the new clock state of the LPUART1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_LPUART1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR2,LPUART1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR2.w;
}
#endif
#ifdef RCC_APB1SMENR2_LPUART1SMEN
/** @brief Sets the clock state of the LPUART1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_LPUART1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR2,LPUART1SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR2_LPUART1RST
/** @brief Forces and releases a reset on the LPUART1 peripheral. */
void XPD_LPUART1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR2,LPUART1RST) = 1;
    RCC_REG_BIT(APB1RSTR2,LPUART1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_OPAMPEN
/** @brief Sets the new clock state of the OPAMP peripheral.
 *  @param NewState: the new clock state to set */
void XPD_OPAMP_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,OPAMPEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_OPAMPSMEN
/** @brief Sets the clock state of the OPAMP peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_OPAMP_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,OPAMPSMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_OPAMPRST
/** @brief Forces and releases a reset on the OPAMP peripheral. */
void XPD_OPAMP_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,OPAMPRST) = 1;
    RCC_REG_BIT(APB1RSTR1,OPAMPRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_OTGFSEN
/** @brief Sets the new clock state of the OTGFS peripheral.
 *  @param NewState: the new clock state to set */
void XPD_OTGFS_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,OTGFSEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_OTGFSSMEN
/** @brief Sets the clock state of the OTGFS peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_OTGFS_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,OTGFSSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_OTGFSRST
/** @brief Forces and releases a reset on the OTGFS peripheral. */
void XPD_OTGFS_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 1;
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_PWREN
/** @brief Sets the new clock state of the PWR peripheral.
 *  @param NewState: the new clock state to set */
void XPD_PWR_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,PWREN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_PWRSMEN
/** @brief Sets the clock state of the PWR peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_PWR_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,PWRSMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_PWRRST
/** @brief Forces and releases a reset on the PWR peripheral. */
void XPD_PWR_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,PWRRST) = 1;
    RCC_REG_BIT(APB1RSTR1,PWRRST) = 0;
}
#endif
#ifdef RCC_AHB3ENR_QSPIEN
/** @brief Sets the new clock state of the QSPI peripheral.
 *  @param NewState: the new clock state to set */
void XPD_QSPI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3ENR,QSPIEN) = NewState;
    NewState = (FunctionalState)RCC->AHB3ENR.w;
}
#endif
#ifdef RCC_AHB3SMENR_QSPISMEN
/** @brief Sets the clock state of the QSPI peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_QSPI_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3SMENR,QSPISMEN) = NewState;
}
#endif
#ifdef RCC_AHB3RSTR_QSPIRST
/** @brief Forces and releases a reset on the QSPI peripheral. */
void XPD_QSPI_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,QSPIRST) = 1;
    RCC_REG_BIT(AHB3RSTR,QSPIRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_RNGEN
/** @brief Sets the new clock state of the RNG peripheral.
 *  @param NewState: the new clock state to set */
void XPD_RNG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,RNGEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR.w;
}
#endif
#ifdef RCC_AHB2SMENR_RNGSMEN
/** @brief Sets the clock state of the RNG peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_RNG_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,RNGSMEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_RNGRST
/** @brief Forces and releases a reset on the RNG peripheral. */
void XPD_RNG_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,RNGRST) = 1;
    RCC_REG_BIT(AHB2RSTR,RNGRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_RTCAPBEN
/** @brief Sets the new clock state of the RTCAPB peripheral.
 *  @param NewState: the new clock state to set */
void XPD_RTCAPB_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,RTCAPBEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_RTCAPBSMEN
/** @brief Sets the clock state of the RTCAPB peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_RTCAPB_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,RTCAPBSMEN) = NewState;
}
#endif
#ifdef RCC_APB2ENR_SAI1EN
/** @brief Sets the new clock state of the SAI1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SAI1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SAI1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_SAI1SMEN
/** @brief Sets the clock state of the SAI1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SAI1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,SAI1SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SAI1RST
/** @brief Forces and releases a reset on the SAI1 peripheral. */
void XPD_SAI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SAI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SAI1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SAI2EN
/** @brief Sets the new clock state of the SAI2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SAI2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SAI2EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_SAI2SMEN
/** @brief Sets the clock state of the SAI2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SAI2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,SAI2SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SAI2RST
/** @brief Forces and releases a reset on the SAI2 peripheral. */
void XPD_SAI2_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SAI2RST) = 1;
    RCC_REG_BIT(APB2RSTR,SAI2RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SDMMC1EN
/** @brief Sets the new clock state of the SDMMC1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SDMMC1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SDMMC1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_SDMMC1SMEN
/** @brief Sets the clock state of the SDMMC1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SDMMC1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,SDMMC1SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SDMMC1RST
/** @brief Forces and releases a reset on the SDMMC1 peripheral. */
void XPD_SDMMC1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SDMMC1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SDMMC1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI1EN
/** @brief Sets the new clock state of the SPI1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SPI1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SPI1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_SPI1SMEN
/** @brief Sets the clock state of the SPI1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SPI1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,SPI1SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SPI1RST
/** @brief Forces and releases a reset on the SPI1 peripheral. */
void XPD_SPI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_SPI2EN
/** @brief Sets the new clock state of the SPI2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SPI2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,SPI2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_SPI2SMEN
/** @brief Sets the clock state of the SPI2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SPI2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,SPI2SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_SPI2RST
/** @brief Forces and releases a reset on the SPI2 peripheral. */
void XPD_SPI2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,SPI2RST) = 1;
    RCC_REG_BIT(APB1RSTR1,SPI2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_SPI3EN
/** @brief Sets the new clock state of the SPI3 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SPI3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,SPI3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_SPI3SMEN
/** @brief Sets the clock state of the SPI3 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SPI3_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,SPI3SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_SPI3RST
/** @brief Forces and releases a reset on the SPI3 peripheral. */
void XPD_SPI3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,SPI3RST) = 1;
    RCC_REG_BIT(APB1RSTR1,SPI3RST) = 0;
}
#endif
#ifdef RCC_AHB1SMENR_SRAM1SMEN
/** @brief Sets the clock state of the SRAM1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SRAM1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1SMENR,SRAM1SMEN) = NewState;
}
#endif
#ifdef RCC_AHB2SMENR_SRAM2SMEN
/** @brief Sets the clock state of the SRAM2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SRAM2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2SMENR,SRAM2SMEN) = NewState;
}
#endif
#ifdef RCC_APB1ENR2_SWPMI1EN
/** @brief Sets the new clock state of the SWPMI1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SWPMI1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR2,SWPMI1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR2.w;
}
#endif
#ifdef RCC_APB1SMENR2_SWPMI1SMEN
/** @brief Sets the clock state of the SWPMI1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SWPMI1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR2,SWPMI1SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR2_SWPMI1RST
/** @brief Forces and releases a reset on the SWPMI1 peripheral. */
void XPD_SWPMI1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR2,SWPMI1RST) = 1;
    RCC_REG_BIT(APB1RSTR2,SWPMI1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
/** @brief Sets the new clock state of the SYSCFG peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SYSCFGEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_SYSCFGSMEN
/** @brief Sets the clock state of the SYSCFG peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_SYSCFG_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,SYSCFGSMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
/** @brief Forces and releases a reset on the SYSCFG peripheral. */
void XPD_SYSCFG_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 1;
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM1EN
/** @brief Sets the new clock state of the TIM1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_TIM1SMEN
/** @brief Sets the clock state of the TIM1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,TIM1SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM1RST
/** @brief Forces and releases a reset on the TIM1 peripheral. */
void XPD_TIM1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM15EN
/** @brief Sets the new clock state of the TIM15 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM15_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM15EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_TIM15SMEN
/** @brief Sets the clock state of the TIM15 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM15_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,TIM15SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM15RST
/** @brief Forces and releases a reset on the TIM15 peripheral. */
void XPD_TIM15_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM15RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM15RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM16EN
/** @brief Sets the new clock state of the TIM16 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM16_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM16EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_TIM16SMEN
/** @brief Sets the clock state of the TIM16 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM16_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,TIM16SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM16RST
/** @brief Forces and releases a reset on the TIM16 peripheral. */
void XPD_TIM16_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM16RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM16RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM17EN
/** @brief Sets the new clock state of the TIM17 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM17_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM17EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_TIM17SMEN
/** @brief Sets the clock state of the TIM17 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM17_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,TIM17SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM17RST
/** @brief Forces and releases a reset on the TIM17 peripheral. */
void XPD_TIM17_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM17RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM17RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_TIM2EN
/** @brief Sets the new clock state of the TIM2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,TIM2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_TIM2SMEN
/** @brief Sets the clock state of the TIM2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,TIM2SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_TIM2RST
/** @brief Forces and releases a reset on the TIM2 peripheral. */
void XPD_TIM2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,TIM2RST) = 1;
    RCC_REG_BIT(APB1RSTR1,TIM2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_TIM3EN
/** @brief Sets the new clock state of the TIM3 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,TIM3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_TIM3SMEN
/** @brief Sets the clock state of the TIM3 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM3_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,TIM3SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_TIM3RST
/** @brief Forces and releases a reset on the TIM3 peripheral. */
void XPD_TIM3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,TIM3RST) = 1;
    RCC_REG_BIT(APB1RSTR1,TIM3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_TIM4EN
/** @brief Sets the new clock state of the TIM4 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,TIM4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_TIM4SMEN
/** @brief Sets the clock state of the TIM4 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM4_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,TIM4SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_TIM4RST
/** @brief Forces and releases a reset on the TIM4 peripheral. */
void XPD_TIM4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,TIM4RST) = 1;
    RCC_REG_BIT(APB1RSTR1,TIM4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_TIM5EN
/** @brief Sets the new clock state of the TIM5 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,TIM5EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_TIM5SMEN
/** @brief Sets the clock state of the TIM5 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM5_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,TIM5SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_TIM5RST
/** @brief Forces and releases a reset on the TIM5 peripheral. */
void XPD_TIM5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,TIM5RST) = 1;
    RCC_REG_BIT(APB1RSTR1,TIM5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_TIM6EN
/** @brief Sets the new clock state of the TIM6 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM6_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,TIM6EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_TIM6SMEN
/** @brief Sets the clock state of the TIM6 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM6_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,TIM6SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_TIM6RST
/** @brief Forces and releases a reset on the TIM6 peripheral. */
void XPD_TIM6_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,TIM6RST) = 1;
    RCC_REG_BIT(APB1RSTR1,TIM6RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_TIM7EN
/** @brief Sets the new clock state of the TIM7 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM7_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,TIM7EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_TIM7SMEN
/** @brief Sets the clock state of the TIM7 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM7_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,TIM7SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_TIM7RST
/** @brief Forces and releases a reset on the TIM7 peripheral. */
void XPD_TIM7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,TIM7RST) = 1;
    RCC_REG_BIT(APB1RSTR1,TIM7RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM8EN
/** @brief Sets the new clock state of the TIM8 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM8_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM8EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_TIM8SMEN
/** @brief Sets the clock state of the TIM8 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TIM8_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,TIM8SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM8RST
/** @brief Forces and releases a reset on the TIM8 peripheral. */
void XPD_TIM8_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_TSCEN
/** @brief Sets the new clock state of the TSC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TSC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,TSCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR.w;
}
#endif
#ifdef RCC_AHB1SMENR_TSCSMEN
/** @brief Sets the clock state of the TSC peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_TSC_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1SMENR,TSCSMEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_TSCRST
/** @brief Forces and releases a reset on the TSC peripheral. */
void XPD_TSC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,TSCRST) = 1;
    RCC_REG_BIT(AHB1RSTR,TSCRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_UART4EN
/** @brief Sets the new clock state of the UART4 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_UART4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,UART4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_UART4SMEN
/** @brief Sets the clock state of the UART4 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_UART4_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,UART4SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_UART4RST
/** @brief Forces and releases a reset on the UART4 peripheral. */
void XPD_UART4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,UART4RST) = 1;
    RCC_REG_BIT(APB1RSTR1,UART4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_UART5EN
/** @brief Sets the new clock state of the UART5 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_UART5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,UART5EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_UART5SMEN
/** @brief Sets the clock state of the UART5 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_UART5_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,UART5SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_UART5RST
/** @brief Forces and releases a reset on the UART5 peripheral. */
void XPD_UART5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,UART5RST) = 1;
    RCC_REG_BIT(APB1RSTR1,UART5RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART1EN
/** @brief Sets the new clock state of the USART1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,USART1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2SMENR_USART1SMEN
/** @brief Sets the clock state of the USART1 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_USART1_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2SMENR,USART1SMEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_USART1RST
/** @brief Forces and releases a reset on the USART1 peripheral. */
void XPD_USART1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART1RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_USART2EN
/** @brief Sets the new clock state of the USART2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,USART2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_USART2SMEN
/** @brief Sets the clock state of the USART2 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_USART2_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,USART2SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_USART2RST
/** @brief Forces and releases a reset on the USART2 peripheral. */
void XPD_USART2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,USART2RST) = 1;
    RCC_REG_BIT(APB1RSTR1,USART2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_USART3EN
/** @brief Sets the new clock state of the USART3 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,USART3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_USART3SMEN
/** @brief Sets the clock state of the USART3 peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_USART3_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,USART3SMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_USART3RST
/** @brief Forces and releases a reset on the USART3 peripheral. */
void XPD_USART3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,USART3RST) = 1;
    RCC_REG_BIT(APB1RSTR1,USART3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_USBFSEN
/** @brief Sets the new clock state of the USBFS peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USBFS_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,USBFSEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_USBFSSMEN
/** @brief Sets the clock state of the USBFS peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_USBFS_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,USBFSSMEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR1_USBFSRST
/** @brief Forces and releases a reset on the USBFS peripheral. */
void XPD_USBFS_Reset(void)
{
    RCC_REG_BIT(APB1RSTR1,USBFSRST) = 1;
    RCC_REG_BIT(APB1RSTR1,USBFSRST) = 0;
}
#endif
#ifdef RCC_APB1ENR1_WWDGEN
/** @brief Sets the new clock state of the WWDG peripheral.
 *  @param NewState: the new clock state to set */
void XPD_WWDG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR1,WWDGEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR1.w;
}
#endif
#ifdef RCC_APB1SMENR1_WWDGSMEN
/** @brief Sets the clock state of the WWDG peripheral during sleep mode.
 *  @param NewState: the new sleep clock state to set */
void XPD_WWDG_SleepClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1SMENR1,WWDGSMEN) = NewState;
}
#endif

/** @} */

/** @} */

/** @} */
