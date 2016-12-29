/**
  ******************************************************************************
  * @file    xpd_rcc_gen.c
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
#include "xpd_rcc.h"

/** @addtogroup RCC
 * @{ */

/** @addtogroup RCC_Peripheral_Control
 * @{ */

/** @addtogroup RCC_Generated_Functions
 * @{ */
#ifdef RCC_APB2ENR_ADCEN
/** @brief Sets the new clock state of the ADC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_ADC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,ADCEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_ADCRST
/** @brief Forces and releases a reset on the ADC peripheral. */
void XPD_ADC_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,ADCRST) = 1;
    RCC_REG_BIT(APB2RSTR,ADCRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_CANEN
/** @brief Sets the new clock state of the CAN peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CAN_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CANEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_CANRST
/** @brief Forces and releases a reset on the CAN peripheral. */
void XPD_CAN_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CANRST) = 1;
    RCC_REG_BIT(APB1RSTR,CANRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_CECEN
/** @brief Sets the new clock state of the CEC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CEC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CECEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_CECRST
/** @brief Forces and releases a reset on the CEC peripheral. */
void XPD_CEC_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CECRST) = 1;
    RCC_REG_BIT(APB1RSTR,CECRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SYSCFGCOMPEN
/** @brief Sets the new clock state of the COMP peripheral. */
void XPD_COMP_ClockCtrl(FunctionalState NewState)
{
#if 0 /* SYSCFG clock is kept on */
    RCC_REG_BIT(APB2ENR,SYSCFGCOMPEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
#endif
}
#endif
#ifdef RCC_AHBENR_CRCEN
/** @brief Sets the new clock state of the CRC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CRC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,CRCEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_APB1ENR_CRSEN
/** @brief Sets the new clock state of the CRS peripheral.
 *  @param NewState: the new clock state to set */
void XPD_CRS_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CRSEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_CRSRST
/** @brief Forces and releases a reset on the CRS peripheral. */
void XPD_CRS_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CRSRST) = 1;
    RCC_REG_BIT(APB1RSTR,CRSRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_DACEN
/** @brief Sets the new clock state of the DAC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DAC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,DACEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_DACRST
/** @brief Forces and releases a reset on the DAC peripheral. */
void XPD_DAC_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,DACRST) = 1;
    RCC_REG_BIT(APB1RSTR,DACRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_DBGMCUEN
/** @brief Sets the new clock state of the DBGMCU peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DBGMCU_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,DBGMCUEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_DBGMCURST
/** @brief Forces and releases a reset on the DBGMCU peripheral. */
void XPD_DBGMCU_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,DBGMCURST) = 1;
    RCC_REG_BIT(APB2RSTR,DBGMCURST) = 0;
}
#endif
#ifdef RCC_AHBENR_DMAEN
/** @brief Sets the new clock state of the DMA peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DMA1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,DMAEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBENR_DMA2EN
/** @brief Sets the new clock state of the DMA2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_DMA2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,DMA2EN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBENR_FLITFEN
/** @brief Sets the new clock state of the FLITF peripheral.
 *  @param NewState: the new clock state to set */
void XPD_FLITF_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,FLITFEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBENR_GPIOAEN
/** @brief Sets the new clock state of the GPIOA peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOA_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOAEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOARST
/** @brief Forces and releases a reset on the GPIOA peripheral. */
void XPD_GPIOA_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOARST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOARST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOBEN
/** @brief Sets the new clock state of the GPIOB peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOB_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOBEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOBRST
/** @brief Forces and releases a reset on the GPIOB peripheral. */
void XPD_GPIOB_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOBRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOBRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOCEN
/** @brief Sets the new clock state of the GPIOC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOCEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOCRST
/** @brief Forces and releases a reset on the GPIOC peripheral. */
void XPD_GPIOC_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOCRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOCRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIODEN
/** @brief Sets the new clock state of the GPIOD peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOD_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIODEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIODRST
/** @brief Forces and releases a reset on the GPIOD peripheral. */
void XPD_GPIOD_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIODRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIODRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOEEN
/** @brief Sets the new clock state of the GPIOE peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOE_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOEEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOERST
/** @brief Forces and releases a reset on the GPIOE peripheral. */
void XPD_GPIOE_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOERST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOERST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOFEN
/** @brief Sets the new clock state of the GPIOF peripheral.
 *  @param NewState: the new clock state to set */
void XPD_GPIOF_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOFEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOFRST
/** @brief Forces and releases a reset on the GPIOF peripheral. */
void XPD_GPIOF_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOFRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOFRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C1EN
/** @brief Sets the new clock state of the I2C1 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_I2C1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_I2C1RST
/** @brief Forces and releases a reset on the I2C1 peripheral. */
void XPD_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C2EN
/** @brief Sets the new clock state of the I2C2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_I2C2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_I2C2RST
/** @brief Forces and releases a reset on the I2C2 peripheral. */
void XPD_I2C2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_PWREN
/** @brief Sets the new clock state of the PWR peripheral.
 *  @param NewState: the new clock state to set */
void XPD_PWR_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,PWREN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_PWRRST
/** @brief Forces and releases a reset on the PWR peripheral. */
void XPD_PWR_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,PWRRST) = 1;
    RCC_REG_BIT(APB1RSTR,PWRRST) = 0;
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
#ifdef RCC_APB2RSTR_SPI1RST
/** @brief Forces and releases a reset on the SPI1 peripheral. */
void XPD_SPI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPI2EN
/** @brief Sets the new clock state of the SPI2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SPI2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,SPI2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_SPI2RST
/** @brief Forces and releases a reset on the SPI2 peripheral. */
void XPD_SPI2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 0;
}
#endif
#ifdef RCC_AHBENR_SRAMEN
/** @brief Sets the new clock state of the SRAM peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SRAM_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,SRAMEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
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
#ifdef RCC_APB2ENR_SYSCFGCOMPEN
/** @brief Sets the new clock state of the SYSCFG peripheral.
 *  @param NewState: the new clock state to set */
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SYSCFGCOMPEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
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
#ifdef RCC_APB2RSTR_TIM1RST
/** @brief Forces and releases a reset on the TIM1 peripheral. */
void XPD_TIM1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM14EN
/** @brief Sets the new clock state of the TIM14 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM14_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM14EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM14RST
/** @brief Forces and releases a reset on the TIM14 peripheral. */
void XPD_TIM14_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 0;
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
#ifdef RCC_APB2RSTR_TIM17RST
/** @brief Forces and releases a reset on the TIM17 peripheral. */
void XPD_TIM17_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM17RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM17RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM2EN
/** @brief Sets the new clock state of the TIM2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM2RST
/** @brief Forces and releases a reset on the TIM2 peripheral. */
void XPD_TIM2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM3EN
/** @brief Sets the new clock state of the TIM3 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM3RST
/** @brief Forces and releases a reset on the TIM3 peripheral. */
void XPD_TIM3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM6EN
/** @brief Sets the new clock state of the TIM6 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM6_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM6EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM6RST
/** @brief Forces and releases a reset on the TIM6 peripheral. */
void XPD_TIM6_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM7EN
/** @brief Sets the new clock state of the TIM7 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TIM7_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM7EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM7RST
/** @brief Forces and releases a reset on the TIM7 peripheral. */
void XPD_TIM7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 0;
}
#endif
#ifdef RCC_AHBENR_TSCEN
/** @brief Sets the new clock state of the TSC peripheral.
 *  @param NewState: the new clock state to set */
void XPD_TSC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,TSCEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_TSCRST
/** @brief Forces and releases a reset on the TSC peripheral. */
void XPD_TSC_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,TSCRST) = 1;
    RCC_REG_BIT(AHBRSTR,TSCRST) = 0;
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
#ifdef RCC_APB2RSTR_USART1RST
/** @brief Forces and releases a reset on the USART1 peripheral. */
void XPD_USART1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART1RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART2EN
/** @brief Sets the new clock state of the USART2 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USART2RST
/** @brief Forces and releases a reset on the USART2 peripheral. */
void XPD_USART2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART2RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART3EN
/** @brief Sets the new clock state of the USART3 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USART3RST
/** @brief Forces and releases a reset on the USART3 peripheral. */
void XPD_USART3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART3RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART4EN
/** @brief Sets the new clock state of the USART4 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USART4RST
/** @brief Forces and releases a reset on the USART4 peripheral. */
void XPD_USART4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART4RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART5EN
/** @brief Sets the new clock state of the USART5 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART5EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USART5RST
/** @brief Forces and releases a reset on the USART5 peripheral. */
void XPD_USART5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART5RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART5RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART6EN
/** @brief Sets the new clock state of the USART6 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART6_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,USART6EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_USART6RST
/** @brief Forces and releases a reset on the USART6 peripheral. */
void XPD_USART6_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART6RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART6RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART7EN
/** @brief Sets the new clock state of the USART7 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART7_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,USART7EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_USART7RST
/** @brief Forces and releases a reset on the USART7 peripheral. */
void XPD_USART7_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART7RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART7RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART8EN
/** @brief Sets the new clock state of the USART8 peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USART8_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,USART8EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_USART8RST
/** @brief Forces and releases a reset on the USART8 peripheral. */
void XPD_USART8_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART8RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART8RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USBEN
/** @brief Sets the new clock state of the USB peripheral.
 *  @param NewState: the new clock state to set */
void XPD_USB_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USBEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USBRST
/** @brief Forces and releases a reset on the USB peripheral. */
void XPD_USB_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USBRST) = 1;
    RCC_REG_BIT(APB1RSTR,USBRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_WWDGEN
/** @brief Sets the new clock state of the WWDG peripheral.
 *  @param NewState: the new clock state to set */
void XPD_WWDG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,WWDGEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_WWDGRST
/** @brief Forces and releases a reset on the WWDG peripheral. */
void XPD_WWDG_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 1;
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 0;
}
#endif

/** @} */

/** @} */

/** @} */
