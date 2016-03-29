/**
  ******************************************************************************
  * @file    xpd_rcc_gen.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-03-20
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
#ifdef RCC_APB2RSTR_ADCRST
/** @brief Forces and releases a reset on the ADC peripheral. */
void XPD_ADC_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,ADCRST) = 1;
    RCC_REG_BIT(APB2RSTR,ADCRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_ADC1EN
/** @brief Sets the new clock state of the ADC1 peripheral. */
void XPD_ADC1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,ADC1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_ADC1LPEN
/** @brief Sets the clock state of the ADC1 peripheral during sleep mode. */
void XPD_ADC1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,ADC1LPEN) = NewState;
}
#endif
#ifdef RCC_APB2ENR_ADC2EN
/** @brief Sets the new clock state of the ADC2 peripheral. */
void XPD_ADC2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,ADC2EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_ADC2LPEN
/** @brief Sets the clock state of the ADC2 peripheral during sleep mode. */
void XPD_ADC2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,ADC2LPEN) = NewState;
}
#endif
#ifdef RCC_APB2ENR_ADC3EN
/** @brief Sets the new clock state of the ADC3 peripheral. */
void XPD_ADC3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,ADC3EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_ADC3LPEN
/** @brief Sets the clock state of the ADC3 peripheral during sleep mode. */
void XPD_ADC3_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,ADC3LPEN) = NewState;
}
#endif
#ifdef RCC_APB1ENR_CAN1EN
/** @brief Sets the new clock state of the CAN1 peripheral. */
void XPD_CAN1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CAN1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_CAN1LPEN
/** @brief Sets the clock state of the CAN1 peripheral during sleep mode. */
void XPD_CAN1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,CAN1LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_CAN1RST
/** @brief Forces and releases a reset on the CAN1 peripheral. */
void XPD_CAN1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CAN1RST) = 1;
    RCC_REG_BIT(APB1RSTR,CAN1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_CAN2EN
/** @brief Sets the new clock state of the CAN2 peripheral. */
void XPD_CAN2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CAN2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_CAN2LPEN
/** @brief Sets the clock state of the CAN2 peripheral during sleep mode. */
void XPD_CAN2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,CAN2LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_CAN2RST
/** @brief Forces and releases a reset on the CAN2 peripheral. */
void XPD_CAN2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CAN2RST) = 1;
    RCC_REG_BIT(APB1RSTR,CAN2RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_CCMDATARAMEN
/** @brief Sets the new clock state of the CCMDATARAM peripheral. */
void XPD_CCMDATARAM_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,CCMDATARAMEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_APB1ENR_CECEN
/** @brief Sets the new clock state of the CEC peripheral. */
void XPD_CEC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CECEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_CECLPEN
/** @brief Sets the clock state of the CEC peripheral during sleep mode. */
void XPD_CEC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,CECLPEN) = NewState;
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
#ifdef RCC_AHB1ENR_CRCEN
/** @brief Sets the new clock state of the CRC peripheral. */
void XPD_CRC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,CRCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_CRCLPEN
/** @brief Sets the clock state of the CRC peripheral during sleep mode. */
void XPD_CRC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,CRCLPEN) = NewState;
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
#ifdef RCC_AHB2ENR_CRYPEN
/** @brief Sets the new clock state of the CRYP peripheral. */
void XPD_CRYP_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,CRYPEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR;
}
#endif
#ifdef RCC_AHB2LPENR_CRYPLPEN
/** @brief Sets the clock state of the CRYP peripheral during sleep mode. */
void XPD_CRYP_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2LPENR,CRYPLPEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_CRYPRST
/** @brief Forces and releases a reset on the CRYP peripheral. */
void XPD_CRYP_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,CRYPRST) = 1;
    RCC_REG_BIT(AHB2RSTR,CRYPRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_DACEN
/** @brief Sets the new clock state of the DAC peripheral. */
void XPD_DAC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,DACEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_DACLPEN
/** @brief Sets the clock state of the DAC peripheral during sleep mode. */
void XPD_DAC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,DACLPEN) = NewState;
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
#ifdef RCC_AHB2ENR_DCMIEN
/** @brief Sets the new clock state of the DCMI peripheral. */
void XPD_DCMI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,DCMIEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR;
}
#endif
#ifdef RCC_AHB2LPENR_DCMILPEN
/** @brief Sets the clock state of the DCMI peripheral during sleep mode. */
void XPD_DCMI_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2LPENR,DCMILPEN) = NewState;
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
#ifdef RCC_AHB1ENR_DMA1EN
/** @brief Sets the new clock state of the DMA1 peripheral. */
void XPD_DMA1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,DMA1EN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_DMA1LPEN
/** @brief Sets the clock state of the DMA1 peripheral during sleep mode. */
void XPD_DMA1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,DMA1LPEN) = NewState;
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
/** @brief Sets the new clock state of the DMA2 peripheral. */
void XPD_DMA2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,DMA2EN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_DMA2LPEN
/** @brief Sets the clock state of the DMA2 peripheral during sleep mode. */
void XPD_DMA2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,DMA2LPEN) = NewState;
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
/** @brief Sets the new clock state of the DMA2D peripheral. */
void XPD_DMA2D_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,DMA2DEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_DMA2DLPEN
/** @brief Sets the clock state of the DMA2D peripheral during sleep mode. */
void XPD_DMA2D_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,DMA2DLPEN) = NewState;
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
#ifdef RCC_APB2ENR_DSIEN
/** @brief Sets the new clock state of the DSI peripheral. */
void XPD_DSI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,DSIEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_DSILPEN
/** @brief Sets the clock state of the DSI peripheral during sleep mode. */
void XPD_DSI_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,DSILPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_DSIRST
/** @brief Forces and releases a reset on the DSI peripheral. */
void XPD_DSI_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,DSIRST) = 1;
    RCC_REG_BIT(APB2RSTR,DSIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACEN
/** @brief Sets the new clock state of the ETHMAC peripheral. */
void XPD_ETHMAC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,ETHMACEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACLPEN
/** @brief Sets the clock state of the ETHMAC peripheral during sleep mode. */
void XPD_ETHMAC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_ETHMACRST
/** @brief Forces and releases a reset on the ETHMAC peripheral. */
void XPD_ETHMAC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,ETHMACRST) = 1;
    RCC_REG_BIT(AHB1RSTR,ETHMACRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACPTPEN
/** @brief Sets the new clock state of the ETHMAC_PTP peripheral. */
void XPD_ETHMAC_PTP_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,ETHMACPTPEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACPTPLPEN
/** @brief Sets the clock state of the ETHMAC_PTP peripheral during sleep mode. */
void XPD_ETHMAC_PTP_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACPTPLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACRXEN
/** @brief Sets the new clock state of the ETHMAC_RX peripheral. */
void XPD_ETHMAC_RX_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,ETHMACRXEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACRXLPEN
/** @brief Sets the clock state of the ETHMAC_RX peripheral during sleep mode. */
void XPD_ETHMAC_RX_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACRXLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACTXEN
/** @brief Sets the new clock state of the ETHMAC_TX peripheral. */
void XPD_ETHMAC_TX_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,ETHMACTXEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACTXLPEN
/** @brief Sets the clock state of the ETHMAC_TX peripheral during sleep mode. */
void XPD_ETHMAC_TX_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACTXLPEN) = NewState;
}
#endif
#ifdef RCC_APB2ENR_EXTITEN
/** @brief Sets the new clock state of the EXTI peripheral. */
void XPD_EXTI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,EXTITEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_EXTITLPEN
/** @brief Sets the clock state of the EXTI peripheral during sleep mode. */
void XPD_EXTI_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,EXTITLPEN) = NewState;
}
#endif
#ifdef RCC_AHB3ENR_FMCEN
/** @brief Sets the new clock state of the FMC peripheral. */
void XPD_FMC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3ENR,FMCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB3ENR;
}
#endif
#ifdef RCC_AHB3LPENR_FMCLPEN
/** @brief Sets the clock state of the FMC peripheral during sleep mode. */
void XPD_FMC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3LPENR,FMCLPEN) = NewState;
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
#ifdef RCC_APB1ENR_FMPI2C1EN
/** @brief Sets the new clock state of the FMP_I2C1 peripheral. */
void XPD_FMP_I2C1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,FMPI2C1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_FMPI2C1LPEN
/** @brief Sets the clock state of the FMP_I2C1 peripheral during sleep mode. */
void XPD_FMP_I2C1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,FMPI2C1LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_FMPI2C1RST
/** @brief Forces and releases a reset on the FMP_I2C1 peripheral. */
void XPD_FMP_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,FMPI2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR,FMPI2C1RST) = 0;
}
#endif
#ifdef RCC_AHB3ENR_FSMCEN
/** @brief Sets the new clock state of the FSMC peripheral. */
void XPD_FSMC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3ENR,FSMCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB3ENR;
}
#endif
#ifdef RCC_AHB3LPENR_FSMCLPEN
/** @brief Sets the clock state of the FSMC peripheral during sleep mode. */
void XPD_FSMC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3LPENR,FSMCLPEN) = NewState;
}
#endif
#ifdef RCC_AHB3RSTR_FSMCRST
/** @brief Forces and releases a reset on the FSMC peripheral. */
void XPD_FSMC_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,FSMCRST) = 1;
    RCC_REG_BIT(AHB3RSTR,FSMCRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOAEN
/** @brief Sets the new clock state of the GPIOA peripheral. */
void XPD_GPIOA_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOAEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOALPEN
/** @brief Sets the clock state of the GPIOA peripheral during sleep mode. */
void XPD_GPIOA_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOALPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOARST
/** @brief Forces and releases a reset on the GPIOA peripheral. */
void XPD_GPIOA_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOARST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOARST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOBEN
/** @brief Sets the new clock state of the GPIOB peripheral. */
void XPD_GPIOB_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOBEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOBLPEN
/** @brief Sets the clock state of the GPIOB peripheral during sleep mode. */
void XPD_GPIOB_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOBLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOBRST
/** @brief Forces and releases a reset on the GPIOB peripheral. */
void XPD_GPIOB_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOBRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOBRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOCEN
/** @brief Sets the new clock state of the GPIOC peripheral. */
void XPD_GPIOC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOCEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOCLPEN
/** @brief Sets the clock state of the GPIOC peripheral during sleep mode. */
void XPD_GPIOC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOCLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOCRST
/** @brief Forces and releases a reset on the GPIOC peripheral. */
void XPD_GPIOC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOCRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOCRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIODEN
/** @brief Sets the new clock state of the GPIOD peripheral. */
void XPD_GPIOD_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIODEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIODLPEN
/** @brief Sets the clock state of the GPIOD peripheral during sleep mode. */
void XPD_GPIOD_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIODLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIODRST
/** @brief Forces and releases a reset on the GPIOD peripheral. */
void XPD_GPIOD_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIODRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIODRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOEEN
/** @brief Sets the new clock state of the GPIOE peripheral. */
void XPD_GPIOE_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOEEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOELPEN
/** @brief Sets the clock state of the GPIOE peripheral during sleep mode. */
void XPD_GPIOE_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOELPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOERST
/** @brief Forces and releases a reset on the GPIOE peripheral. */
void XPD_GPIOE_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOERST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOERST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOFEN
/** @brief Sets the new clock state of the GPIOF peripheral. */
void XPD_GPIOF_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOFEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOFLPEN
/** @brief Sets the clock state of the GPIOF peripheral during sleep mode. */
void XPD_GPIOF_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOFLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOFRST
/** @brief Forces and releases a reset on the GPIOF peripheral. */
void XPD_GPIOF_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOFRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOFRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOGEN
/** @brief Sets the new clock state of the GPIOG peripheral. */
void XPD_GPIOG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOGEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOGLPEN
/** @brief Sets the clock state of the GPIOG peripheral during sleep mode. */
void XPD_GPIOG_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOGLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOGRST
/** @brief Forces and releases a reset on the GPIOG peripheral. */
void XPD_GPIOG_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOGRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOGRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOHEN
/** @brief Sets the new clock state of the GPIOH peripheral. */
void XPD_GPIOH_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOHEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOHLPEN
/** @brief Sets the clock state of the GPIOH peripheral during sleep mode. */
void XPD_GPIOH_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOHLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOHRST
/** @brief Forces and releases a reset on the GPIOH peripheral. */
void XPD_GPIOH_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOHRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOHRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOIEN
/** @brief Sets the new clock state of the GPIOI peripheral. */
void XPD_GPIOI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOIEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOILPEN
/** @brief Sets the clock state of the GPIOI peripheral during sleep mode. */
void XPD_GPIOI_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOILPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOIRST
/** @brief Forces and releases a reset on the GPIOI peripheral. */
void XPD_GPIOI_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOIRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOJEN
/** @brief Sets the new clock state of the GPIOJ peripheral. */
void XPD_GPIOJ_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOJEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOJLPEN
/** @brief Sets the clock state of the GPIOJ peripheral during sleep mode. */
void XPD_GPIOJ_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOJLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOJRST
/** @brief Forces and releases a reset on the GPIOJ peripheral. */
void XPD_GPIOJ_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOJRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOJRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOKEN
/** @brief Sets the new clock state of the GPIOK peripheral. */
void XPD_GPIOK_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,GPIOKEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOKLPEN
/** @brief Sets the clock state of the GPIOK peripheral during sleep mode. */
void XPD_GPIOK_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,GPIOKLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOKRST
/** @brief Forces and releases a reset on the GPIOK peripheral. */
void XPD_GPIOK_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOKRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOKRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_HASHEN
/** @brief Sets the new clock state of the HASH peripheral. */
void XPD_HASH_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,HASHEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR;
}
#endif
#ifdef RCC_AHB2LPENR_HASHLPEN
/** @brief Sets the clock state of the HASH peripheral during sleep mode. */
void XPD_HASH_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2LPENR,HASHLPEN) = NewState;
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
#ifdef RCC_APB1ENR_I2C1EN
/** @brief Sets the new clock state of the I2C1 peripheral. */
void XPD_I2C1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_I2C1LPEN
/** @brief Sets the clock state of the I2C1 peripheral during sleep mode. */
void XPD_I2C1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,I2C1LPEN) = NewState;
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
/** @brief Sets the new clock state of the I2C2 peripheral. */
void XPD_I2C2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_I2C2LPEN
/** @brief Sets the clock state of the I2C2 peripheral during sleep mode. */
void XPD_I2C2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,I2C2LPEN) = NewState;
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
#ifdef RCC_APB1ENR_I2C3EN
/** @brief Sets the new clock state of the I2C3 peripheral. */
void XPD_I2C3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_I2C3LPEN
/** @brief Sets the clock state of the I2C3 peripheral during sleep mode. */
void XPD_I2C3_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,I2C3LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_I2C3RST
/** @brief Forces and releases a reset on the I2C3 peripheral. */
void XPD_I2C3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_LPTIM1EN
/** @brief Sets the new clock state of the LPTIM1 peripheral. */
void XPD_LPTIM1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,LPTIM1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_LPTIM1LPEN
/** @brief Sets the clock state of the LPTIM1 peripheral during sleep mode. */
void XPD_LPTIM1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,LPTIM1LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_LPTIM1RST
/** @brief Forces and releases a reset on the LPTIM1 peripheral. */
void XPD_LPTIM1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,LPTIM1RST) = 1;
    RCC_REG_BIT(APB1RSTR,LPTIM1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_LTDCEN
/** @brief Sets the new clock state of the LTDC peripheral. */
void XPD_LTDC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,LTDCEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_LTDCLPEN
/** @brief Sets the clock state of the LTDC peripheral during sleep mode. */
void XPD_LTDC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,LTDCLPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_LTDCRST
/** @brief Forces and releases a reset on the LTDC peripheral. */
void XPD_LTDC_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,LTDCRST) = 1;
    RCC_REG_BIT(APB2RSTR,LTDCRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_OTGFSEN
/** @brief Sets the new clock state of the OTG_FS peripheral. */
void XPD_OTG_FS_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,OTGFSEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR;
}
#endif
#ifdef RCC_AHB2LPENR_OTGFSLPEN
/** @brief Sets the clock state of the OTG_FS peripheral during sleep mode. */
void XPD_OTG_FS_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2LPENR,OTGFSLPEN) = NewState;
}
#endif
#ifdef RCC_AHB2RSTR_OTGFSRST
/** @brief Forces and releases a reset on the OTG_FS peripheral. */
void XPD_OTG_FS_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 1;
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_OTGHRST
/** @brief Forces and releases a reset on the OTG_HS peripheral. */
void XPD_OTG_HS_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,OTGHRST) = 1;
    RCC_REG_BIT(AHB1RSTR,OTGHRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_OTGHSEN
/** @brief Sets the new clock state of the OTG_HS peripheral. */
void XPD_OTG_HS_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,OTGHSEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_OTGHSLPEN
/** @brief Sets the clock state of the OTG_HS peripheral during sleep mode. */
void XPD_OTG_HS_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSLPEN) = NewState;
}
#endif
#ifdef RCC_AHB1ENR_OTGHSULPIEN
/** @brief Sets the new clock state of the OTG_HS_ULPI peripheral. */
void XPD_OTG_HS_ULPI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1ENR,OTGHSULPIEN) = NewState;
    NewState = (FunctionalState)RCC->AHB1ENR;
}
#endif
#ifdef RCC_AHB1LPENR_OTGHSULPILPEN
/** @brief Sets the clock state of the OTG_HS_ULPI peripheral during sleep mode. */
void XPD_OTG_HS_ULPI_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSULPILPEN) = NewState;
}
#endif
#ifdef RCC_APB1ENR_PWREN
/** @brief Sets the new clock state of the PWR peripheral. */
void XPD_PWR_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,PWREN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_PWRLPEN
/** @brief Sets the clock state of the PWR peripheral during sleep mode. */
void XPD_PWR_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,PWRLPEN) = NewState;
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
#ifdef RCC_AHB3ENR_QSPIEN
/** @brief Sets the new clock state of the QSPI peripheral. */
void XPD_QSPI_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3ENR,QSPIEN) = NewState;
    NewState = (FunctionalState)RCC->AHB3ENR;
}
#endif
#ifdef RCC_AHB3LPENR_QSPILPEN
/** @brief Sets the clock state of the QSPI peripheral during sleep mode. */
void XPD_QSPI_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB3LPENR,QSPILPEN) = NewState;
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
/** @brief Sets the new clock state of the RNG peripheral. */
void XPD_RNG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2ENR,RNGEN) = NewState;
    NewState = (FunctionalState)RCC->AHB2ENR;
}
#endif
#ifdef RCC_AHB2LPENR_RNGLPEN
/** @brief Sets the clock state of the RNG peripheral during sleep mode. */
void XPD_RNG_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(AHB2LPENR,RNGLPEN) = NewState;
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
#ifdef RCC_APB1ENR_RTCAPBEN
/** @brief Sets the new clock state of the RTC peripheral. */
void XPD_RTC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,RTCAPBEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_RTCAPBLPEN
/** @brief Sets the clock state of the RTC peripheral during sleep mode. */
void XPD_RTC_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,RTCAPBLPEN) = NewState;
}
#endif
#ifdef RCC_APB2ENR_SAI1EN
/** @brief Sets the new clock state of the SAI1 peripheral. */
void XPD_SAI1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SAI1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SAI1LPEN
/** @brief Sets the clock state of the SAI1 peripheral during sleep mode. */
void XPD_SAI1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SAI1LPEN) = NewState;
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
/** @brief Sets the new clock state of the SAI2 peripheral. */
void XPD_SAI2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SAI2EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SAI2LPEN
/** @brief Sets the clock state of the SAI2 peripheral during sleep mode. */
void XPD_SAI2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SAI2LPEN) = NewState;
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
#ifdef RCC_APB2ENR_SDIOEN
/** @brief Sets the new clock state of the SDIO peripheral. */
void XPD_SDIO_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SDIOEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SDIOLPEN
/** @brief Sets the clock state of the SDIO peripheral during sleep mode. */
void XPD_SDIO_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SDIOLPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SDIORST
/** @brief Forces and releases a reset on the SDIO peripheral. */
void XPD_SDIO_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SDIORST) = 1;
    RCC_REG_BIT(APB2RSTR,SDIORST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPDIFRXEN
/** @brief Sets the new clock state of the SPDIF_RX peripheral. */
void XPD_SPDIF_RX_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,SPDIFRXEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_SPDIFRXLPEN
/** @brief Sets the clock state of the SPDIF_RX peripheral during sleep mode. */
void XPD_SPDIF_RX_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,SPDIFRXLPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_SPDIFRXRST
/** @brief Forces and releases a reset on the SPDIF_RX peripheral. */
void XPD_SPDIF_RX_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPDIFRXRST) = 1;
    RCC_REG_BIT(APB1RSTR,SPDIFRXRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI1EN
/** @brief Sets the new clock state of the SPI1 peripheral. */
void XPD_SPI1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SPI1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SPI1LPEN
/** @brief Sets the clock state of the SPI1 peripheral during sleep mode. */
void XPD_SPI1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SPI1LPEN) = NewState;
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
/** @brief Sets the new clock state of the SPI2 peripheral. */
void XPD_SPI2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,SPI2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_SPI2LPEN
/** @brief Sets the clock state of the SPI2 peripheral during sleep mode. */
void XPD_SPI2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,SPI2LPEN) = NewState;
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
#ifdef RCC_APB1ENR_SPI3EN
/** @brief Sets the new clock state of the SPI3 peripheral. */
void XPD_SPI3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,SPI3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_SPI3LPEN
/** @brief Sets the clock state of the SPI3 peripheral during sleep mode. */
void XPD_SPI3_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,SPI3LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_SPI3RST
/** @brief Forces and releases a reset on the SPI3 peripheral. */
void XPD_SPI3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI4EN
/** @brief Sets the new clock state of the SPI4 peripheral. */
void XPD_SPI4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SPI4EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SPI4LPEN
/** @brief Sets the clock state of the SPI4 peripheral during sleep mode. */
void XPD_SPI4_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SPI4LPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SPI4RST
/** @brief Forces and releases a reset on the SPI4 peripheral. */
void XPD_SPI4_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI5EN
/** @brief Sets the new clock state of the SPI5 peripheral. */
void XPD_SPI5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SPI5EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SPI5LPEN
/** @brief Sets the clock state of the SPI5 peripheral during sleep mode. */
void XPD_SPI5_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SPI5LPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SPI5RST
/** @brief Forces and releases a reset on the SPI5 peripheral. */
void XPD_SPI5_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI5RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI5RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI6EN
/** @brief Sets the new clock state of the SPI6 peripheral. */
void XPD_SPI6_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SPI6EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SPI6LPEN
/** @brief Sets the clock state of the SPI6 peripheral during sleep mode. */
void XPD_SPI6_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SPI6LPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_SPI6RST
/** @brief Forces and releases a reset on the SPI6 peripheral. */
void XPD_SPI6_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI6RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI6RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
/** @brief Sets the new clock state of the SYSCFG peripheral. */
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SYSCFGEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_SYSCFGLPEN
/** @brief Sets the clock state of the SYSCFG peripheral during sleep mode. */
void XPD_SYSCFG_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,SYSCFGLPEN) = NewState;
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
/** @brief Sets the new clock state of the TIM1 peripheral. */
void XPD_TIM1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_TIM1LPEN
/** @brief Sets the clock state of the TIM1 peripheral during sleep mode. */
void XPD_TIM1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,TIM1LPEN) = NewState;
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
#ifdef RCC_APB2ENR_TIM10EN
/** @brief Sets the new clock state of the TIM10 peripheral. */
void XPD_TIM10_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM10EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_TIM10LPEN
/** @brief Sets the clock state of the TIM10 peripheral during sleep mode. */
void XPD_TIM10_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,TIM10LPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM10RST
/** @brief Forces and releases a reset on the TIM10 peripheral. */
void XPD_TIM10_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM10RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM10RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM11EN
/** @brief Sets the new clock state of the TIM11 peripheral. */
void XPD_TIM11_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM11EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_TIM11LPEN
/** @brief Sets the clock state of the TIM11 peripheral during sleep mode. */
void XPD_TIM11_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,TIM11LPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM11RST
/** @brief Forces and releases a reset on the TIM11 peripheral. */
void XPD_TIM11_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM11RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM11RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM12EN
/** @brief Sets the new clock state of the TIM12 peripheral. */
void XPD_TIM12_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM12EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM12LPEN
/** @brief Sets the clock state of the TIM12 peripheral during sleep mode. */
void XPD_TIM12_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM12LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_TIM12RST
/** @brief Forces and releases a reset on the TIM12 peripheral. */
void XPD_TIM12_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM13EN
/** @brief Sets the new clock state of the TIM13 peripheral. */
void XPD_TIM13_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM13EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM13LPEN
/** @brief Sets the clock state of the TIM13 peripheral during sleep mode. */
void XPD_TIM13_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM13LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_TIM13RST
/** @brief Forces and releases a reset on the TIM13 peripheral. */
void XPD_TIM13_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM14EN
/** @brief Sets the new clock state of the TIM14 peripheral. */
void XPD_TIM14_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM14EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM14LPEN
/** @brief Sets the clock state of the TIM14 peripheral during sleep mode. */
void XPD_TIM14_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM14LPEN) = NewState;
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
#ifdef RCC_APB1ENR_TIM2EN
/** @brief Sets the new clock state of the TIM2 peripheral. */
void XPD_TIM2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM2LPEN
/** @brief Sets the clock state of the TIM2 peripheral during sleep mode. */
void XPD_TIM2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM2LPEN) = NewState;
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
/** @brief Sets the new clock state of the TIM3 peripheral. */
void XPD_TIM3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM3LPEN
/** @brief Sets the clock state of the TIM3 peripheral during sleep mode. */
void XPD_TIM3_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM3LPEN) = NewState;
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
#ifdef RCC_APB1ENR_TIM4EN
/** @brief Sets the new clock state of the TIM4 peripheral. */
void XPD_TIM4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM4LPEN
/** @brief Sets the clock state of the TIM4 peripheral during sleep mode. */
void XPD_TIM4_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM4LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_TIM4RST
/** @brief Forces and releases a reset on the TIM4 peripheral. */
void XPD_TIM4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM5EN
/** @brief Sets the new clock state of the TIM5 peripheral. */
void XPD_TIM5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM5EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM5LPEN
/** @brief Sets the clock state of the TIM5 peripheral during sleep mode. */
void XPD_TIM5_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM5LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_TIM5RST
/** @brief Forces and releases a reset on the TIM5 peripheral. */
void XPD_TIM5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM6EN
/** @brief Sets the new clock state of the TIM6 peripheral. */
void XPD_TIM6_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM6EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM6LPEN
/** @brief Sets the clock state of the TIM6 peripheral during sleep mode. */
void XPD_TIM6_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM6LPEN) = NewState;
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
/** @brief Sets the new clock state of the TIM7 peripheral. */
void XPD_TIM7_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM7EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_TIM7LPEN
/** @brief Sets the clock state of the TIM7 peripheral during sleep mode. */
void XPD_TIM7_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,TIM7LPEN) = NewState;
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
#ifdef RCC_APB2ENR_TIM8EN
/** @brief Sets the new clock state of the TIM8 peripheral. */
void XPD_TIM8_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM8EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_TIM8LPEN
/** @brief Sets the clock state of the TIM8 peripheral during sleep mode. */
void XPD_TIM8_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,TIM8LPEN) = NewState;
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
#ifdef RCC_APB2ENR_TIM9EN
/** @brief Sets the new clock state of the TIM9 peripheral. */
void XPD_TIM9_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM9EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_TIM9LPEN
/** @brief Sets the clock state of the TIM9 peripheral during sleep mode. */
void XPD_TIM9_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,TIM9LPEN) = NewState;
}
#endif
#ifdef RCC_APB2RSTR_TIM9RST
/** @brief Forces and releases a reset on the TIM9 peripheral. */
void XPD_TIM9_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM9RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM9RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART4EN
/** @brief Sets the new clock state of the UART4 peripheral. */
void XPD_UART4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,UART4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_UART4LPEN
/** @brief Sets the clock state of the UART4 peripheral during sleep mode. */
void XPD_UART4_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,UART4LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_UART4RST
/** @brief Forces and releases a reset on the UART4 peripheral. */
void XPD_UART4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART4RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART5EN
/** @brief Sets the new clock state of the UART5 peripheral. */
void XPD_UART5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,UART5EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_UART5LPEN
/** @brief Sets the clock state of the UART5 peripheral during sleep mode. */
void XPD_UART5_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,UART5LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_UART5RST
/** @brief Forces and releases a reset on the UART5 peripheral. */
void XPD_UART5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART5RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART7EN
/** @brief Sets the new clock state of the UART7 peripheral. */
void XPD_UART7_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,UART7EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_UART7LPEN
/** @brief Sets the clock state of the UART7 peripheral during sleep mode. */
void XPD_UART7_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,UART7LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_UART7RST
/** @brief Forces and releases a reset on the UART7 peripheral. */
void XPD_UART7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART7RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART7RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART8EN
/** @brief Sets the new clock state of the UART8 peripheral. */
void XPD_UART8_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,UART8EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_UART8LPEN
/** @brief Sets the clock state of the UART8 peripheral during sleep mode. */
void XPD_UART8_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,UART8LPEN) = NewState;
}
#endif
#ifdef RCC_APB1RSTR_UART8RST
/** @brief Forces and releases a reset on the UART8 peripheral. */
void XPD_UART8_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART8RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART8RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART1EN
/** @brief Sets the new clock state of the USART1 peripheral. */
void XPD_USART1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,USART1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_USART1LPEN
/** @brief Sets the clock state of the USART1 peripheral during sleep mode. */
void XPD_USART1_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,USART1LPEN) = NewState;
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
/** @brief Sets the new clock state of the USART2 peripheral. */
void XPD_USART2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_USART2LPEN
/** @brief Sets the clock state of the USART2 peripheral during sleep mode. */
void XPD_USART2_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,USART2LPEN) = NewState;
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
/** @brief Sets the new clock state of the USART3 peripheral. */
void XPD_USART3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_USART3LPEN
/** @brief Sets the clock state of the USART3 peripheral during sleep mode. */
void XPD_USART3_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,USART3LPEN) = NewState;
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
#ifdef RCC_APB2ENR_USART6EN
/** @brief Sets the new clock state of the USART6 peripheral. */
void XPD_USART6_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,USART6EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR;
}
#endif
#ifdef RCC_APB2LPENR_USART6LPEN
/** @brief Sets the clock state of the USART6 peripheral during sleep mode. */
void XPD_USART6_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB2LPENR,USART6LPEN) = NewState;
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
#ifdef RCC_APB1ENR_WWDGEN
/** @brief Sets the new clock state of the WWDG peripheral. */
void XPD_WWDG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,WWDGEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR;
}
#endif
#ifdef RCC_APB1LPENR_WWDGLPEN
/** @brief Sets the clock state of the WWDG peripheral during sleep mode. */
void XPD_WWDG_ClockCtrlInSleep(FunctionalState NewState)
{
    RCC_REG_BIT(APB1LPENR,WWDGLPEN) = NewState;
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
