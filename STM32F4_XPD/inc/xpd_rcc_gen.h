/**
  ******************************************************************************
  * @file    xpd_rcc_gen.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-10
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
#include "xpd_rcc.h"


/** @addtogroup RCC
 * @{ */


/** @defgroup RCC_Peripheral_Control RCC Peripheral Control
 * @{ */


/** @defgroup RCC_Generated_Functions RCC Generated Functions
 * @{ */

#ifdef RCC_APB2RSTR_ADCRST
/** @brief Forces and releases a reset on the ADC peripheral. */
__STATIC_INLINE void XPD_ADC_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,ADCRST) = 1;
    RCC_REG_BIT(APB2RSTR,ADCRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_ADC1EN
/** @brief Enables the clock of the ADC1 peripheral. */
__STATIC_INLINE void XPD_ADC1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,ADC1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the ADC1 peripheral. */
__STATIC_INLINE void XPD_ADC1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,ADC1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_ADC1LPEN
/** @brief Enables the clock of the ADC1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_ADC1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC1LPEN) = 1;
}
/** @brief Disables the clock of the ADC1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_ADC1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC1LPEN) = 0;
}
#endif
#ifdef RCC_APB2ENR_ADC2EN
/** @brief Enables the clock of the ADC2 peripheral. */
__STATIC_INLINE void XPD_ADC2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,ADC2EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the ADC2 peripheral. */
__STATIC_INLINE void XPD_ADC2_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,ADC2EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_ADC2LPEN
/** @brief Enables the clock of the ADC2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_ADC2_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC2LPEN) = 1;
}
/** @brief Disables the clock of the ADC2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_ADC2_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC2LPEN) = 0;
}
#endif
#ifdef RCC_APB2ENR_ADC3EN
/** @brief Enables the clock of the ADC3 peripheral. */
__STATIC_INLINE void XPD_ADC3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,ADC3EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the ADC3 peripheral. */
__STATIC_INLINE void XPD_ADC3_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,ADC3EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_ADC3LPEN
/** @brief Enables the clock of the ADC3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_ADC3_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC3LPEN) = 1;
}
/** @brief Disables the clock of the ADC3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_ADC3_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC3LPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_BKPSRAMEN
/** @brief Enables the clock of the BKPSRAM peripheral. */
__STATIC_INLINE void XPD_BKPSRAM_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,BKPSRAMEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the BKPSRAM peripheral. */
__STATIC_INLINE void XPD_BKPSRAM_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,BKPSRAMEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_BKPSRAMLPEN
/** @brief Enables the clock of the BKPSRAM peripheral during sleep mode. */
__STATIC_INLINE void XPD_BKPSRAM_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,BKPSRAMLPEN) = 1;
}
/** @brief Disables the clock of the BKPSRAM peripheral during sleep mode. */
__STATIC_INLINE void XPD_BKPSRAM_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,BKPSRAMLPEN) = 0;
}
#endif
#ifdef RCC_APB1ENR_CAN1EN
/** @brief Enables the clock of the CAN1 peripheral. */
__STATIC_INLINE void XPD_CAN1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,CAN1EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the CAN1 peripheral. */
__STATIC_INLINE void XPD_CAN1_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,CAN1EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_CAN1LPEN
/** @brief Enables the clock of the CAN1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_CAN1_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN1LPEN) = 1;
}
/** @brief Disables the clock of the CAN1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_CAN1_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN1LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_CAN1RST
/** @brief Forces and releases a reset on the CAN1 peripheral. */
__STATIC_INLINE void XPD_CAN1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CAN1RST) = 1;
    RCC_REG_BIT(APB1RSTR,CAN1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_CAN2EN
/** @brief Enables the clock of the CAN2 peripheral. */
__STATIC_INLINE void XPD_CAN2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,CAN2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the CAN2 peripheral. */
__STATIC_INLINE void XPD_CAN2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,CAN2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_CAN2LPEN
/** @brief Enables the clock of the CAN2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_CAN2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN2LPEN) = 1;
}
/** @brief Disables the clock of the CAN2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_CAN2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_CAN2RST
/** @brief Forces and releases a reset on the CAN2 peripheral. */
__STATIC_INLINE void XPD_CAN2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CAN2RST) = 1;
    RCC_REG_BIT(APB1RSTR,CAN2RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_CCMDATARAMEN
/** @brief Enables the clock of the CCMDATARAM peripheral. */
__STATIC_INLINE void XPD_CCMDATARAM_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,CCMDATARAMEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the CCMDATARAM peripheral. */
__STATIC_INLINE void XPD_CCMDATARAM_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,CCMDATARAMEN) = 0;
}
#endif
#ifdef RCC_APB1ENR_CECEN
/** @brief Enables the clock of the CEC peripheral. */
__STATIC_INLINE void XPD_CEC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,CECEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the CEC peripheral. */
__STATIC_INLINE void XPD_CEC_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,CECEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_CECLPEN
/** @brief Enables the clock of the CEC peripheral during sleep mode. */
__STATIC_INLINE void XPD_CEC_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CECLPEN) = 1;
}
/** @brief Disables the clock of the CEC peripheral during sleep mode. */
__STATIC_INLINE void XPD_CEC_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CECLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_CECRST
/** @brief Forces and releases a reset on the CEC peripheral. */
__STATIC_INLINE void XPD_CEC_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CECRST) = 1;
    RCC_REG_BIT(APB1RSTR,CECRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_CRCEN
/** @brief Enables the clock of the CRC peripheral. */
__STATIC_INLINE void XPD_CRC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,CRCEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the CRC peripheral. */
__STATIC_INLINE void XPD_CRC_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,CRCEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_CRCLPEN
/** @brief Enables the clock of the CRC peripheral during sleep mode. */
__STATIC_INLINE void XPD_CRC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,CRCLPEN) = 1;
}
/** @brief Disables the clock of the CRC peripheral during sleep mode. */
__STATIC_INLINE void XPD_CRC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,CRCLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_CRCRST
/** @brief Forces and releases a reset on the CRC peripheral. */
__STATIC_INLINE void XPD_CRC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,CRCRST) = 1;
    RCC_REG_BIT(AHB1RSTR,CRCRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_CRYPEN
/** @brief Enables the clock of the CRYP peripheral. */
__STATIC_INLINE void XPD_CRYP_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,CRYPEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the CRYP peripheral. */
__STATIC_INLINE void XPD_CRYP_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,CRYPEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_CRYPLPEN
/** @brief Enables the clock of the CRYP peripheral during sleep mode. */
__STATIC_INLINE void XPD_CRYP_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,CRYPLPEN) = 1;
}
/** @brief Disables the clock of the CRYP peripheral during sleep mode. */
__STATIC_INLINE void XPD_CRYP_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,CRYPLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_CRYPRST
/** @brief Forces and releases a reset on the CRYP peripheral. */
__STATIC_INLINE void XPD_CRYP_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,CRYPRST) = 1;
    RCC_REG_BIT(AHB2RSTR,CRYPRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_DACEN
/** @brief Enables the clock of the DAC peripheral. */
__STATIC_INLINE void XPD_DAC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,DACEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the DAC peripheral. */
__STATIC_INLINE void XPD_DAC_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,DACEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_DACLPEN
/** @brief Enables the clock of the DAC peripheral during sleep mode. */
__STATIC_INLINE void XPD_DAC_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,DACLPEN) = 1;
}
/** @brief Disables the clock of the DAC peripheral during sleep mode. */
__STATIC_INLINE void XPD_DAC_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,DACLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_DACRST
/** @brief Forces and releases a reset on the DAC peripheral. */
__STATIC_INLINE void XPD_DAC_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,DACRST) = 1;
    RCC_REG_BIT(APB1RSTR,DACRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_DCMIEN
/** @brief Enables the clock of the DCMI peripheral. */
__STATIC_INLINE void XPD_DCMI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,DCMIEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the DCMI peripheral. */
__STATIC_INLINE void XPD_DCMI_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,DCMIEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_DCMILPEN
/** @brief Enables the clock of the DCMI peripheral during sleep mode. */
__STATIC_INLINE void XPD_DCMI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,DCMILPEN) = 1;
}
/** @brief Disables the clock of the DCMI peripheral during sleep mode. */
__STATIC_INLINE void XPD_DCMI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,DCMILPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_DCMIRST
/** @brief Forces and releases a reset on the DCMI peripheral. */
__STATIC_INLINE void XPD_DCMI_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,DCMIRST) = 1;
    RCC_REG_BIT(AHB2RSTR,DCMIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA1EN
/** @brief Enables the clock of the DMA1 peripheral. */
__STATIC_INLINE void XPD_DMA1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,DMA1EN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the DMA1 peripheral. */
__STATIC_INLINE void XPD_DMA1_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,DMA1EN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_DMA1LPEN
/** @brief Enables the clock of the DMA1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_DMA1_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA1LPEN) = 1;
}
/** @brief Disables the clock of the DMA1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_DMA1_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA1LPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_DMA1RST
/** @brief Forces and releases a reset on the DMA1 peripheral. */
__STATIC_INLINE void XPD_DMA1_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA1RST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA1RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA2EN
/** @brief Enables the clock of the DMA2 peripheral. */
__STATIC_INLINE void XPD_DMA2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,DMA2EN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the DMA2 peripheral. */
__STATIC_INLINE void XPD_DMA2_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,DMA2EN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_DMA2LPEN
/** @brief Enables the clock of the DMA2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_DMA2_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2LPEN) = 1;
}
/** @brief Disables the clock of the DMA2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_DMA2_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2LPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_DMA2RST
/** @brief Forces and releases a reset on the DMA2 peripheral. */
__STATIC_INLINE void XPD_DMA2_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA2RST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA2RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA2DEN
/** @brief Enables the clock of the DMA2D peripheral. */
__STATIC_INLINE void XPD_DMA2D_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,DMA2DEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the DMA2D peripheral. */
__STATIC_INLINE void XPD_DMA2D_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,DMA2DEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_DMA2DLPEN
/** @brief Enables the clock of the DMA2D peripheral during sleep mode. */
__STATIC_INLINE void XPD_DMA2D_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2DLPEN) = 1;
}
/** @brief Disables the clock of the DMA2D peripheral during sleep mode. */
__STATIC_INLINE void XPD_DMA2D_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2DLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_DMA2DRST
/** @brief Forces and releases a reset on the DMA2D peripheral. */
__STATIC_INLINE void XPD_DMA2D_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA2DRST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA2DRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_DSIEN
/** @brief Enables the clock of the DSI peripheral. */
__STATIC_INLINE void XPD_DSI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,DSIEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the DSI peripheral. */
__STATIC_INLINE void XPD_DSI_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,DSIEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_DSILPEN
/** @brief Enables the clock of the DSI peripheral during sleep mode. */
__STATIC_INLINE void XPD_DSI_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,DSILPEN) = 1;
}
/** @brief Disables the clock of the DSI peripheral during sleep mode. */
__STATIC_INLINE void XPD_DSI_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,DSILPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_DSIRST
/** @brief Forces and releases a reset on the DSI peripheral. */
__STATIC_INLINE void XPD_DSI_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,DSIRST) = 1;
    RCC_REG_BIT(APB2RSTR,DSIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACEN
/** @brief Enables the clock of the ETHMAC peripheral. */
__STATIC_INLINE void XPD_ETHMAC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the ETHMAC peripheral. */
__STATIC_INLINE void XPD_ETHMAC_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACLPEN
/** @brief Enables the clock of the ETHMAC peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACLPEN) = 1;
}
/** @brief Disables the clock of the ETHMAC peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_ETHMACRST
/** @brief Forces and releases a reset on the ETHMAC peripheral. */
__STATIC_INLINE void XPD_ETHMAC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,ETHMACRST) = 1;
    RCC_REG_BIT(AHB1RSTR,ETHMACRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACPTPEN
/** @brief Enables the clock of the ETHMAC_PTP peripheral. */
__STATIC_INLINE void XPD_ETHMAC_PTP_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACPTPEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the ETHMAC_PTP peripheral. */
__STATIC_INLINE void XPD_ETHMAC_PTP_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACPTPEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACPTPLPEN
/** @brief Enables the clock of the ETHMAC_PTP peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_PTP_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACPTPLPEN) = 1;
}
/** @brief Disables the clock of the ETHMAC_PTP peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_PTP_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACPTPLPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACRXEN
/** @brief Enables the clock of the ETHMAC_RX peripheral. */
__STATIC_INLINE void XPD_ETHMAC_RX_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACRXEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the ETHMAC_RX peripheral. */
__STATIC_INLINE void XPD_ETHMAC_RX_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACRXEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACRXLPEN
/** @brief Enables the clock of the ETHMAC_RX peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_RX_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACRXLPEN) = 1;
}
/** @brief Disables the clock of the ETHMAC_RX peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_RX_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACRXLPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACTXEN
/** @brief Enables the clock of the ETHMAC_TX peripheral. */
__STATIC_INLINE void XPD_ETHMAC_TX_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACTXEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the ETHMAC_TX peripheral. */
__STATIC_INLINE void XPD_ETHMAC_TX_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACTXEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACTXLPEN
/** @brief Enables the clock of the ETHMAC_TX peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_TX_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACTXLPEN) = 1;
}
/** @brief Disables the clock of the ETHMAC_TX peripheral during sleep mode. */
__STATIC_INLINE void XPD_ETHMAC_TX_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACTXLPEN) = 0;
}
#endif
#ifdef RCC_AHB3ENR_FMCEN
/** @brief Enables the clock of the FMC peripheral. */
__STATIC_INLINE void XPD_FMC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB3ENR,FMCEN) = 1;
    r = RCC->AHB3ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the FMC peripheral. */
__STATIC_INLINE void XPD_FMC_DisableClock(void)
{
    RCC_REG_BIT(AHB3ENR,FMCEN) = 0;
}
#endif
#ifdef RCC_AHB3LPENR_FMCLPEN
/** @brief Enables the clock of the FMC peripheral during sleep mode. */
__STATIC_INLINE void XPD_FMC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FMCLPEN) = 1;
}
/** @brief Disables the clock of the FMC peripheral during sleep mode. */
__STATIC_INLINE void XPD_FMC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FMCLPEN) = 0;
}
#endif
#ifdef RCC_AHB3RSTR_FMCRST
/** @brief Forces and releases a reset on the FMC peripheral. */
__STATIC_INLINE void XPD_FMC_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,FMCRST) = 1;
    RCC_REG_BIT(AHB3RSTR,FMCRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_FMPI2C1EN
/** @brief Enables the clock of the FMP_I2C1 peripheral. */
__STATIC_INLINE void XPD_FMP_I2C1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,FMPI2C1EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the FMP_I2C1 peripheral. */
__STATIC_INLINE void XPD_FMP_I2C1_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,FMPI2C1EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_FMPI2C1LPEN
/** @brief Enables the clock of the FMP_I2C1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_FMP_I2C1_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,FMPI2C1LPEN) = 1;
}
/** @brief Disables the clock of the FMP_I2C1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_FMP_I2C1_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,FMPI2C1LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_FMPI2C1RST
/** @brief Forces and releases a reset on the FMP_I2C1 peripheral. */
__STATIC_INLINE void XPD_FMP_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,FMPI2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR,FMPI2C1RST) = 0;
}
#endif
#ifdef RCC_AHB3ENR_FSMCEN
/** @brief Enables the clock of the FSMC peripheral. */
__STATIC_INLINE void XPD_FSMC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB3ENR,FSMCEN) = 1;
    r = RCC->AHB3ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the FSMC peripheral. */
__STATIC_INLINE void XPD_FSMC_DisableClock(void)
{
    RCC_REG_BIT(AHB3ENR,FSMCEN) = 0;
}
#endif
#ifdef RCC_AHB3LPENR_FSMCLPEN
/** @brief Enables the clock of the FSMC peripheral during sleep mode. */
__STATIC_INLINE void XPD_FSMC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FSMCLPEN) = 1;
}
/** @brief Disables the clock of the FSMC peripheral during sleep mode. */
__STATIC_INLINE void XPD_FSMC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FSMCLPEN) = 0;
}
#endif
#ifdef RCC_AHB3RSTR_FSMCRST
/** @brief Forces and releases a reset on the FSMC peripheral. */
__STATIC_INLINE void XPD_FSMC_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,FSMCRST) = 1;
    RCC_REG_BIT(AHB3RSTR,FSMCRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOAEN
/** @brief Enables the clock of the GPIOA peripheral. */
__STATIC_INLINE void XPD_GPIOA_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOAEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOA peripheral. */
__STATIC_INLINE void XPD_GPIOA_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOAEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOALPEN
/** @brief Enables the clock of the GPIOA peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOA_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOALPEN) = 1;
}
/** @brief Disables the clock of the GPIOA peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOA_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOALPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOARST
/** @brief Forces and releases a reset on the GPIOA peripheral. */
__STATIC_INLINE void XPD_GPIOA_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOARST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOARST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOBEN
/** @brief Enables the clock of the GPIOB peripheral. */
__STATIC_INLINE void XPD_GPIOB_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOBEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOB peripheral. */
__STATIC_INLINE void XPD_GPIOB_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOBEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOBLPEN
/** @brief Enables the clock of the GPIOB peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOB_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOBLPEN) = 1;
}
/** @brief Disables the clock of the GPIOB peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOB_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOBLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOBRST
/** @brief Forces and releases a reset on the GPIOB peripheral. */
__STATIC_INLINE void XPD_GPIOB_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOBRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOBRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOCEN
/** @brief Enables the clock of the GPIOC peripheral. */
__STATIC_INLINE void XPD_GPIOC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOCEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOC peripheral. */
__STATIC_INLINE void XPD_GPIOC_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOCEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOCLPEN
/** @brief Enables the clock of the GPIOC peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOCLPEN) = 1;
}
/** @brief Disables the clock of the GPIOC peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOCLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOCRST
/** @brief Forces and releases a reset on the GPIOC peripheral. */
__STATIC_INLINE void XPD_GPIOC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOCRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOCRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIODEN
/** @brief Enables the clock of the GPIOD peripheral. */
__STATIC_INLINE void XPD_GPIOD_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIODEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOD peripheral. */
__STATIC_INLINE void XPD_GPIOD_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIODEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIODLPEN
/** @brief Enables the clock of the GPIOD peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOD_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIODLPEN) = 1;
}
/** @brief Disables the clock of the GPIOD peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOD_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIODLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIODRST
/** @brief Forces and releases a reset on the GPIOD peripheral. */
__STATIC_INLINE void XPD_GPIOD_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIODRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIODRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOEEN
/** @brief Enables the clock of the GPIOE peripheral. */
__STATIC_INLINE void XPD_GPIOE_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOEEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOE peripheral. */
__STATIC_INLINE void XPD_GPIOE_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOEEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOELPEN
/** @brief Enables the clock of the GPIOE peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOE_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOELPEN) = 1;
}
/** @brief Disables the clock of the GPIOE peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOE_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOELPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOERST
/** @brief Forces and releases a reset on the GPIOE peripheral. */
__STATIC_INLINE void XPD_GPIOE_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOERST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOERST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOFEN
/** @brief Enables the clock of the GPIOF peripheral. */
__STATIC_INLINE void XPD_GPIOF_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOFEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOF peripheral. */
__STATIC_INLINE void XPD_GPIOF_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOFEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOFLPEN
/** @brief Enables the clock of the GPIOF peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOF_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOFLPEN) = 1;
}
/** @brief Disables the clock of the GPIOF peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOF_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOFLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOFRST
/** @brief Forces and releases a reset on the GPIOF peripheral. */
__STATIC_INLINE void XPD_GPIOF_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOFRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOFRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOGEN
/** @brief Enables the clock of the GPIOG peripheral. */
__STATIC_INLINE void XPD_GPIOG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOGEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOG peripheral. */
__STATIC_INLINE void XPD_GPIOG_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOGEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOGLPEN
/** @brief Enables the clock of the GPIOG peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOG_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOGLPEN) = 1;
}
/** @brief Disables the clock of the GPIOG peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOG_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOGLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOGRST
/** @brief Forces and releases a reset on the GPIOG peripheral. */
__STATIC_INLINE void XPD_GPIOG_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOGRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOGRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOHEN
/** @brief Enables the clock of the GPIOH peripheral. */
__STATIC_INLINE void XPD_GPIOH_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOHEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOH peripheral. */
__STATIC_INLINE void XPD_GPIOH_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOHEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOHLPEN
/** @brief Enables the clock of the GPIOH peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOH_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOHLPEN) = 1;
}
/** @brief Disables the clock of the GPIOH peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOH_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOHLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOHRST
/** @brief Forces and releases a reset on the GPIOH peripheral. */
__STATIC_INLINE void XPD_GPIOH_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOHRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOHRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOIEN
/** @brief Enables the clock of the GPIOI peripheral. */
__STATIC_INLINE void XPD_GPIOI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOIEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOI peripheral. */
__STATIC_INLINE void XPD_GPIOI_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOIEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOILPEN
/** @brief Enables the clock of the GPIOI peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOILPEN) = 1;
}
/** @brief Disables the clock of the GPIOI peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOILPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOIRST
/** @brief Forces and releases a reset on the GPIOI peripheral. */
__STATIC_INLINE void XPD_GPIOI_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOIRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOJEN
/** @brief Enables the clock of the GPIOJ peripheral. */
__STATIC_INLINE void XPD_GPIOJ_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOJEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOJ peripheral. */
__STATIC_INLINE void XPD_GPIOJ_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOJEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOJLPEN
/** @brief Enables the clock of the GPIOJ peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOJ_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOJLPEN) = 1;
}
/** @brief Disables the clock of the GPIOJ peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOJ_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOJLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOJRST
/** @brief Forces and releases a reset on the GPIOJ peripheral. */
__STATIC_INLINE void XPD_GPIOJ_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOJRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOJRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOKEN
/** @brief Enables the clock of the GPIOK peripheral. */
__STATIC_INLINE void XPD_GPIOK_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOKEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the GPIOK peripheral. */
__STATIC_INLINE void XPD_GPIOK_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOKEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOKLPEN
/** @brief Enables the clock of the GPIOK peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOK_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOKLPEN) = 1;
}
/** @brief Disables the clock of the GPIOK peripheral during sleep mode. */
__STATIC_INLINE void XPD_GPIOK_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOKLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOKRST
/** @brief Forces and releases a reset on the GPIOK peripheral. */
__STATIC_INLINE void XPD_GPIOK_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOKRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOKRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_HASHEN
/** @brief Enables the clock of the HASH peripheral. */
__STATIC_INLINE void XPD_HASH_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,HASHEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the HASH peripheral. */
__STATIC_INLINE void XPD_HASH_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,HASHEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_HASHLPEN
/** @brief Enables the clock of the HASH peripheral during sleep mode. */
__STATIC_INLINE void XPD_HASH_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,HASHLPEN) = 1;
}
/** @brief Disables the clock of the HASH peripheral during sleep mode. */
__STATIC_INLINE void XPD_HASH_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,HASHLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_HASHRST
/** @brief Forces and releases a reset on the HASH peripheral. */
__STATIC_INLINE void XPD_HASH_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,HASHRST) = 1;
    RCC_REG_BIT(AHB2RSTR,HASHRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C1EN
/** @brief Enables the clock of the I2C1 peripheral. */
__STATIC_INLINE void XPD_I2C1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,I2C1EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the I2C1 peripheral. */
__STATIC_INLINE void XPD_I2C1_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,I2C1EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_I2C1LPEN
/** @brief Enables the clock of the I2C1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_I2C1_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C1LPEN) = 1;
}
/** @brief Disables the clock of the I2C1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_I2C1_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C1LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_I2C1RST
/** @brief Forces and releases a reset on the I2C1 peripheral. */
__STATIC_INLINE void XPD_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C2EN
/** @brief Enables the clock of the I2C2 peripheral. */
__STATIC_INLINE void XPD_I2C2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,I2C2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the I2C2 peripheral. */
__STATIC_INLINE void XPD_I2C2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,I2C2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_I2C2LPEN
/** @brief Enables the clock of the I2C2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_I2C2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C2LPEN) = 1;
}
/** @brief Disables the clock of the I2C2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_I2C2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_I2C2RST
/** @brief Forces and releases a reset on the I2C2 peripheral. */
__STATIC_INLINE void XPD_I2C2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C3EN
/** @brief Enables the clock of the I2C3 peripheral. */
__STATIC_INLINE void XPD_I2C3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,I2C3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the I2C3 peripheral. */
__STATIC_INLINE void XPD_I2C3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,I2C3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_I2C3LPEN
/** @brief Enables the clock of the I2C3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_I2C3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C3LPEN) = 1;
}
/** @brief Disables the clock of the I2C3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_I2C3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_I2C3RST
/** @brief Forces and releases a reset on the I2C3 peripheral. */
__STATIC_INLINE void XPD_I2C3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_LTDCEN
/** @brief Enables the clock of the LTDC peripheral. */
__STATIC_INLINE void XPD_LTDC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,LTDCEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the LTDC peripheral. */
__STATIC_INLINE void XPD_LTDC_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,LTDCEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_LTDCLPEN
/** @brief Enables the clock of the LTDC peripheral during sleep mode. */
__STATIC_INLINE void XPD_LTDC_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,LTDCLPEN) = 1;
}
/** @brief Disables the clock of the LTDC peripheral during sleep mode. */
__STATIC_INLINE void XPD_LTDC_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,LTDCLPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_LTDCRST
/** @brief Forces and releases a reset on the LTDC peripheral. */
__STATIC_INLINE void XPD_LTDC_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,LTDCRST) = 1;
    RCC_REG_BIT(APB2RSTR,LTDCRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_OTGFSEN
/** @brief Enables the clock of the OTG_FS peripheral. */
__STATIC_INLINE void XPD_OTG_FS_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,OTGFSEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the OTG_FS peripheral. */
__STATIC_INLINE void XPD_OTG_FS_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,OTGFSEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_OTGFSLPEN
/** @brief Enables the clock of the OTG_FS peripheral during sleep mode. */
__STATIC_INLINE void XPD_OTG_FS_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,OTGFSLPEN) = 1;
}
/** @brief Disables the clock of the OTG_FS peripheral during sleep mode. */
__STATIC_INLINE void XPD_OTG_FS_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,OTGFSLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_OTGFSRST
/** @brief Forces and releases a reset on the OTG_FS peripheral. */
__STATIC_INLINE void XPD_OTG_FS_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 1;
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_OTGHRST
/** @brief Forces and releases a reset on the OTG_HS peripheral. */
__STATIC_INLINE void XPD_OTG_HS_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,OTGHRST) = 1;
    RCC_REG_BIT(AHB1RSTR,OTGHRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_OTGHSEN
/** @brief Enables the clock of the OTG_HS peripheral. */
__STATIC_INLINE void XPD_OTG_HS_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,OTGHSEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the OTG_HS peripheral. */
__STATIC_INLINE void XPD_OTG_HS_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,OTGHSEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_OTGHSLPEN
/** @brief Enables the clock of the OTG_HS peripheral during sleep mode. */
__STATIC_INLINE void XPD_OTG_HS_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSLPEN) = 1;
}
/** @brief Disables the clock of the OTG_HS peripheral during sleep mode. */
__STATIC_INLINE void XPD_OTG_HS_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSLPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_OTGHSULPIEN
/** @brief Enables the clock of the OTG_HS_ULPI peripheral. */
__STATIC_INLINE void XPD_OTG_HS_ULPI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,OTGHSULPIEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the OTG_HS_ULPI peripheral. */
__STATIC_INLINE void XPD_OTG_HS_ULPI_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,OTGHSULPIEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_OTGHSULPILPEN
/** @brief Enables the clock of the OTG_HS_ULPI peripheral during sleep mode. */
__STATIC_INLINE void XPD_OTG_HS_ULPI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSULPILPEN) = 1;
}
/** @brief Disables the clock of the OTG_HS_ULPI peripheral during sleep mode. */
__STATIC_INLINE void XPD_OTG_HS_ULPI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSULPILPEN) = 0;
}
#endif
#ifdef RCC_APB1ENR_PWREN
/** @brief Enables the clock of the PWR peripheral. */
__STATIC_INLINE void XPD_PWR_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,PWREN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the PWR peripheral. */
__STATIC_INLINE void XPD_PWR_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,PWREN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_PWRLPEN
/** @brief Enables the clock of the PWR peripheral during sleep mode. */
__STATIC_INLINE void XPD_PWR_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,PWRLPEN) = 1;
}
/** @brief Disables the clock of the PWR peripheral during sleep mode. */
__STATIC_INLINE void XPD_PWR_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,PWRLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_PWRRST
/** @brief Forces and releases a reset on the PWR peripheral. */
__STATIC_INLINE void XPD_PWR_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,PWRRST) = 1;
    RCC_REG_BIT(APB1RSTR,PWRRST) = 0;
}
#endif
#ifdef RCC_AHB3ENR_QSPIEN
/** @brief Enables the clock of the QSPI peripheral. */
__STATIC_INLINE void XPD_QSPI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB3ENR,QSPIEN) = 1;
    r = RCC->AHB3ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the QSPI peripheral. */
__STATIC_INLINE void XPD_QSPI_DisableClock(void)
{
    RCC_REG_BIT(AHB3ENR,QSPIEN) = 0;
}
#endif
#ifdef RCC_AHB3LPENR_QSPILPEN
/** @brief Enables the clock of the QSPI peripheral during sleep mode. */
__STATIC_INLINE void XPD_QSPI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,QSPILPEN) = 1;
}
/** @brief Disables the clock of the QSPI peripheral during sleep mode. */
__STATIC_INLINE void XPD_QSPI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,QSPILPEN) = 0;
}
#endif
#ifdef RCC_AHB3RSTR_QSPIRST
/** @brief Forces and releases a reset on the QSPI peripheral. */
__STATIC_INLINE void XPD_QSPI_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,QSPIRST) = 1;
    RCC_REG_BIT(AHB3RSTR,QSPIRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_RNGEN
/** @brief Enables the clock of the RNG peripheral. */
__STATIC_INLINE void XPD_RNG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,RNGEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the RNG peripheral. */
__STATIC_INLINE void XPD_RNG_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,RNGEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_RNGLPEN
/** @brief Enables the clock of the RNG peripheral during sleep mode. */
__STATIC_INLINE void XPD_RNG_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,RNGLPEN) = 1;
}
/** @brief Disables the clock of the RNG peripheral during sleep mode. */
__STATIC_INLINE void XPD_RNG_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,RNGLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_RNGRST
/** @brief Forces and releases a reset on the RNG peripheral. */
__STATIC_INLINE void XPD_RNG_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,RNGRST) = 1;
    RCC_REG_BIT(AHB2RSTR,RNGRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SAI1EN
/** @brief Enables the clock of the SAI1 peripheral. */
__STATIC_INLINE void XPD_SAI1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SAI1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SAI1 peripheral. */
__STATIC_INLINE void XPD_SAI1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SAI1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SAI1LPEN
/** @brief Enables the clock of the SAI1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SAI1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI1LPEN) = 1;
}
/** @brief Disables the clock of the SAI1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SAI1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SAI1RST
/** @brief Forces and releases a reset on the SAI1 peripheral. */
__STATIC_INLINE void XPD_SAI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SAI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SAI1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SAI2EN
/** @brief Enables the clock of the SAI2 peripheral. */
__STATIC_INLINE void XPD_SAI2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SAI2EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SAI2 peripheral. */
__STATIC_INLINE void XPD_SAI2_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SAI2EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SAI2LPEN
/** @brief Enables the clock of the SAI2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SAI2_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI2LPEN) = 1;
}
/** @brief Disables the clock of the SAI2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SAI2_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI2LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SAI2RST
/** @brief Forces and releases a reset on the SAI2 peripheral. */
__STATIC_INLINE void XPD_SAI2_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SAI2RST) = 1;
    RCC_REG_BIT(APB2RSTR,SAI2RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SDIOEN
/** @brief Enables the clock of the SDIO peripheral. */
__STATIC_INLINE void XPD_SDIO_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SDIOEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SDIO peripheral. */
__STATIC_INLINE void XPD_SDIO_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SDIOEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SDIOLPEN
/** @brief Enables the clock of the SDIO peripheral during sleep mode. */
__STATIC_INLINE void XPD_SDIO_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SDIOLPEN) = 1;
}
/** @brief Disables the clock of the SDIO peripheral during sleep mode. */
__STATIC_INLINE void XPD_SDIO_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SDIOLPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SDIORST
/** @brief Forces and releases a reset on the SDIO peripheral. */
__STATIC_INLINE void XPD_SDIO_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SDIORST) = 1;
    RCC_REG_BIT(APB2RSTR,SDIORST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPDIFRXEN
/** @brief Enables the clock of the SPDIFRX peripheral. */
__STATIC_INLINE void XPD_SPDIFRX_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,SPDIFRXEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SPDIFRX peripheral. */
__STATIC_INLINE void XPD_SPDIFRX_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,SPDIFRXEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_SPDIFRXLPEN
/** @brief Enables the clock of the SPDIFRX peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPDIFRX_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPDIFRXLPEN) = 1;
}
/** @brief Disables the clock of the SPDIFRX peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPDIFRX_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPDIFRXLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_SPDIFRXRST
/** @brief Forces and releases a reset on the SPDIFRX peripheral. */
__STATIC_INLINE void XPD_SPDIFRX_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPDIFRXRST) = 1;
    RCC_REG_BIT(APB1RSTR,SPDIFRXRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI1EN
/** @brief Enables the clock of the SPI1 peripheral. */
__STATIC_INLINE void XPD_SPI1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SPI1 peripheral. */
__STATIC_INLINE void XPD_SPI1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI1LPEN
/** @brief Enables the clock of the SPI1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI1LPEN) = 1;
}
/** @brief Disables the clock of the SPI1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI1RST
/** @brief Forces and releases a reset on the SPI1 peripheral. */
__STATIC_INLINE void XPD_SPI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPI2EN
/** @brief Enables the clock of the SPI2 peripheral. */
__STATIC_INLINE void XPD_SPI2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,SPI2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SPI2 peripheral. */
__STATIC_INLINE void XPD_SPI2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,SPI2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_SPI2LPEN
/** @brief Enables the clock of the SPI2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI2LPEN) = 1;
}
/** @brief Disables the clock of the SPI2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_SPI2RST
/** @brief Forces and releases a reset on the SPI2 peripheral. */
__STATIC_INLINE void XPD_SPI2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPI3EN
/** @brief Enables the clock of the SPI3 peripheral. */
__STATIC_INLINE void XPD_SPI3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,SPI3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SPI3 peripheral. */
__STATIC_INLINE void XPD_SPI3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,SPI3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_SPI3LPEN
/** @brief Enables the clock of the SPI3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI3LPEN) = 1;
}
/** @brief Disables the clock of the SPI3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_SPI3RST
/** @brief Forces and releases a reset on the SPI3 peripheral. */
__STATIC_INLINE void XPD_SPI3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI4EN
/** @brief Enables the clock of the SPI4 peripheral. */
__STATIC_INLINE void XPD_SPI4_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI4EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SPI4 peripheral. */
__STATIC_INLINE void XPD_SPI4_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI4EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI4LPEN
/** @brief Enables the clock of the SPI4 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI4_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI4LPEN) = 1;
}
/** @brief Disables the clock of the SPI4 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI4_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI4LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI4RST
/** @brief Forces and releases a reset on the SPI4 peripheral. */
__STATIC_INLINE void XPD_SPI4_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI5EN
/** @brief Enables the clock of the SPI5 peripheral. */
__STATIC_INLINE void XPD_SPI5_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI5EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SPI5 peripheral. */
__STATIC_INLINE void XPD_SPI5_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI5EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI5LPEN
/** @brief Enables the clock of the SPI5 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI5_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI5LPEN) = 1;
}
/** @brief Disables the clock of the SPI5 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI5_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI5LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI5RST
/** @brief Forces and releases a reset on the SPI5 peripheral. */
__STATIC_INLINE void XPD_SPI5_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI5RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI5RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI6EN
/** @brief Enables the clock of the SPI6 peripheral. */
__STATIC_INLINE void XPD_SPI6_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI6EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SPI6 peripheral. */
__STATIC_INLINE void XPD_SPI6_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI6EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI6LPEN
/** @brief Enables the clock of the SPI6 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI6_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI6LPEN) = 1;
}
/** @brief Disables the clock of the SPI6 peripheral during sleep mode. */
__STATIC_INLINE void XPD_SPI6_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI6LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI6RST
/** @brief Forces and releases a reset on the SPI6 peripheral. */
__STATIC_INLINE void XPD_SPI6_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI6RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI6RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
/** @brief Enables the clock of the SYSCFG peripheral. */
__STATIC_INLINE void XPD_SYSCFG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SYSCFGEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the SYSCFG peripheral. */
__STATIC_INLINE void XPD_SYSCFG_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SYSCFGEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SYSCFGLPEN
/** @brief Enables the clock of the SYSCFG peripheral during sleep mode. */
__STATIC_INLINE void XPD_SYSCFG_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SYSCFGLPEN) = 1;
}
/** @brief Disables the clock of the SYSCFG peripheral during sleep mode. */
__STATIC_INLINE void XPD_SYSCFG_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SYSCFGLPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
/** @brief Forces and releases a reset on the SYSCFG peripheral. */
__STATIC_INLINE void XPD_SYSCFG_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 1;
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM1EN
/** @brief Enables the clock of the TIM1 peripheral. */
__STATIC_INLINE void XPD_TIM1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM1 peripheral. */
__STATIC_INLINE void XPD_TIM1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM1LPEN
/** @brief Enables the clock of the TIM1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM1LPEN) = 1;
}
/** @brief Disables the clock of the TIM1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM1RST
/** @brief Forces and releases a reset on the TIM1 peripheral. */
__STATIC_INLINE void XPD_TIM1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM10EN
/** @brief Enables the clock of the TIM10 peripheral. */
__STATIC_INLINE void XPD_TIM10_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM10EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM10 peripheral. */
__STATIC_INLINE void XPD_TIM10_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM10EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM10LPEN
/** @brief Enables the clock of the TIM10 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM10_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM10LPEN) = 1;
}
/** @brief Disables the clock of the TIM10 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM10_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM10LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM10RST
/** @brief Forces and releases a reset on the TIM10 peripheral. */
__STATIC_INLINE void XPD_TIM10_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM10RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM10RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM11EN
/** @brief Enables the clock of the TIM11 peripheral. */
__STATIC_INLINE void XPD_TIM11_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM11EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM11 peripheral. */
__STATIC_INLINE void XPD_TIM11_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM11EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM11LPEN
/** @brief Enables the clock of the TIM11 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM11_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM11LPEN) = 1;
}
/** @brief Disables the clock of the TIM11 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM11_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM11LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM11RST
/** @brief Forces and releases a reset on the TIM11 peripheral. */
__STATIC_INLINE void XPD_TIM11_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM11RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM11RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM12EN
/** @brief Enables the clock of the TIM12 peripheral. */
__STATIC_INLINE void XPD_TIM12_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM12EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM12 peripheral. */
__STATIC_INLINE void XPD_TIM12_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM12EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM12LPEN
/** @brief Enables the clock of the TIM12 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM12_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM12LPEN) = 1;
}
/** @brief Disables the clock of the TIM12 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM12_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM12LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM12RST
/** @brief Forces and releases a reset on the TIM12 peripheral. */
__STATIC_INLINE void XPD_TIM12_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM13EN
/** @brief Enables the clock of the TIM13 peripheral. */
__STATIC_INLINE void XPD_TIM13_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM13EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM13 peripheral. */
__STATIC_INLINE void XPD_TIM13_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM13EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM13LPEN
/** @brief Enables the clock of the TIM13 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM13_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM13LPEN) = 1;
}
/** @brief Disables the clock of the TIM13 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM13_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM13LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM13RST
/** @brief Forces and releases a reset on the TIM13 peripheral. */
__STATIC_INLINE void XPD_TIM13_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM14EN
/** @brief Enables the clock of the TIM14 peripheral. */
__STATIC_INLINE void XPD_TIM14_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM14EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM14 peripheral. */
__STATIC_INLINE void XPD_TIM14_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM14EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM14LPEN
/** @brief Enables the clock of the TIM14 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM14_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM14LPEN) = 1;
}
/** @brief Disables the clock of the TIM14 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM14_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM14LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM14RST
/** @brief Forces and releases a reset on the TIM14 peripheral. */
__STATIC_INLINE void XPD_TIM14_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM2EN
/** @brief Enables the clock of the TIM2 peripheral. */
__STATIC_INLINE void XPD_TIM2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM2 peripheral. */
__STATIC_INLINE void XPD_TIM2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM2LPEN
/** @brief Enables the clock of the TIM2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM2LPEN) = 1;
}
/** @brief Disables the clock of the TIM2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM2RST
/** @brief Forces and releases a reset on the TIM2 peripheral. */
__STATIC_INLINE void XPD_TIM2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM3EN
/** @brief Enables the clock of the TIM3 peripheral. */
__STATIC_INLINE void XPD_TIM3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM3 peripheral. */
__STATIC_INLINE void XPD_TIM3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM3LPEN
/** @brief Enables the clock of the TIM3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM3LPEN) = 1;
}
/** @brief Disables the clock of the TIM3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM3RST
/** @brief Forces and releases a reset on the TIM3 peripheral. */
__STATIC_INLINE void XPD_TIM3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM4EN
/** @brief Enables the clock of the TIM4 peripheral. */
__STATIC_INLINE void XPD_TIM4_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM4EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM4 peripheral. */
__STATIC_INLINE void XPD_TIM4_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM4EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM4LPEN
/** @brief Enables the clock of the TIM4 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM4_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM4LPEN) = 1;
}
/** @brief Disables the clock of the TIM4 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM4_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM4LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM4RST
/** @brief Forces and releases a reset on the TIM4 peripheral. */
__STATIC_INLINE void XPD_TIM4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM5EN
/** @brief Enables the clock of the TIM5 peripheral. */
__STATIC_INLINE void XPD_TIM5_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM5EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM5 peripheral. */
__STATIC_INLINE void XPD_TIM5_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM5EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM5LPEN
/** @brief Enables the clock of the TIM5 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM5_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM5LPEN) = 1;
}
/** @brief Disables the clock of the TIM5 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM5_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM5LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM5RST
/** @brief Forces and releases a reset on the TIM5 peripheral. */
__STATIC_INLINE void XPD_TIM5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM6EN
/** @brief Enables the clock of the TIM6 peripheral. */
__STATIC_INLINE void XPD_TIM6_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM6EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM6 peripheral. */
__STATIC_INLINE void XPD_TIM6_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM6EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM6LPEN
/** @brief Enables the clock of the TIM6 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM6_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM6LPEN) = 1;
}
/** @brief Disables the clock of the TIM6 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM6_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM6LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM6RST
/** @brief Forces and releases a reset on the TIM6 peripheral. */
__STATIC_INLINE void XPD_TIM6_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM7EN
/** @brief Enables the clock of the TIM7 peripheral. */
__STATIC_INLINE void XPD_TIM7_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM7EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM7 peripheral. */
__STATIC_INLINE void XPD_TIM7_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM7EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM7LPEN
/** @brief Enables the clock of the TIM7 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM7_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM7LPEN) = 1;
}
/** @brief Disables the clock of the TIM7 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM7_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM7LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM7RST
/** @brief Forces and releases a reset on the TIM7 peripheral. */
__STATIC_INLINE void XPD_TIM7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM8EN
/** @brief Enables the clock of the TIM8 peripheral. */
__STATIC_INLINE void XPD_TIM8_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM8EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM8 peripheral. */
__STATIC_INLINE void XPD_TIM8_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM8EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM8LPEN
/** @brief Enables the clock of the TIM8 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM8_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM8LPEN) = 1;
}
/** @brief Disables the clock of the TIM8 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM8_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM8LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM8RST
/** @brief Forces and releases a reset on the TIM8 peripheral. */
__STATIC_INLINE void XPD_TIM8_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM9EN
/** @brief Enables the clock of the TIM9 peripheral. */
__STATIC_INLINE void XPD_TIM9_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM9EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the TIM9 peripheral. */
__STATIC_INLINE void XPD_TIM9_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM9EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM9LPEN
/** @brief Enables the clock of the TIM9 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM9_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM9LPEN) = 1;
}
/** @brief Disables the clock of the TIM9 peripheral during sleep mode. */
__STATIC_INLINE void XPD_TIM9_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM9LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM9RST
/** @brief Forces and releases a reset on the TIM9 peripheral. */
__STATIC_INLINE void XPD_TIM9_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM9RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM9RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART4EN
/** @brief Enables the clock of the UART4 peripheral. */
__STATIC_INLINE void XPD_UART4_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART4EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the UART4 peripheral. */
__STATIC_INLINE void XPD_UART4_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART4EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART4LPEN
/** @brief Enables the clock of the UART4 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART4_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART4LPEN) = 1;
}
/** @brief Disables the clock of the UART4 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART4_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART4LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART4RST
/** @brief Forces and releases a reset on the UART4 peripheral. */
__STATIC_INLINE void XPD_UART4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART4RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART5EN
/** @brief Enables the clock of the UART5 peripheral. */
__STATIC_INLINE void XPD_UART5_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART5EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the UART5 peripheral. */
__STATIC_INLINE void XPD_UART5_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART5EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART5LPEN
/** @brief Enables the clock of the UART5 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART5_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART5LPEN) = 1;
}
/** @brief Disables the clock of the UART5 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART5_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART5LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART5RST
/** @brief Forces and releases a reset on the UART5 peripheral. */
__STATIC_INLINE void XPD_UART5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART5RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART7EN
/** @brief Enables the clock of the UART7 peripheral. */
__STATIC_INLINE void XPD_UART7_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART7EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the UART7 peripheral. */
__STATIC_INLINE void XPD_UART7_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART7EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART7LPEN
/** @brief Enables the clock of the UART7 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART7_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART7LPEN) = 1;
}
/** @brief Disables the clock of the UART7 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART7_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART7LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART7RST
/** @brief Forces and releases a reset on the UART7 peripheral. */
__STATIC_INLINE void XPD_UART7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART7RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART7RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART8EN
/** @brief Enables the clock of the UART8 peripheral. */
__STATIC_INLINE void XPD_UART8_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART8EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the UART8 peripheral. */
__STATIC_INLINE void XPD_UART8_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART8EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART8LPEN
/** @brief Enables the clock of the UART8 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART8_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART8LPEN) = 1;
}
/** @brief Disables the clock of the UART8 peripheral during sleep mode. */
__STATIC_INLINE void XPD_UART8_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART8LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART8RST
/** @brief Forces and releases a reset on the UART8 peripheral. */
__STATIC_INLINE void XPD_UART8_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART8RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART8RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART1EN
/** @brief Enables the clock of the USART1 peripheral. */
__STATIC_INLINE void XPD_USART1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,USART1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the USART1 peripheral. */
__STATIC_INLINE void XPD_USART1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,USART1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_USART1LPEN
/** @brief Enables the clock of the USART1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART1LPEN) = 1;
}
/** @brief Disables the clock of the USART1 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_USART1RST
/** @brief Forces and releases a reset on the USART1 peripheral. */
__STATIC_INLINE void XPD_USART1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART1RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART2EN
/** @brief Enables the clock of the USART2 peripheral. */
__STATIC_INLINE void XPD_USART2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,USART2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the USART2 peripheral. */
__STATIC_INLINE void XPD_USART2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,USART2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_USART2LPEN
/** @brief Enables the clock of the USART2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART2LPEN) = 1;
}
/** @brief Disables the clock of the USART2 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_USART2RST
/** @brief Forces and releases a reset on the USART2 peripheral. */
__STATIC_INLINE void XPD_USART2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART2RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART3EN
/** @brief Enables the clock of the USART3 peripheral. */
__STATIC_INLINE void XPD_USART3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,USART3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the USART3 peripheral. */
__STATIC_INLINE void XPD_USART3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,USART3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_USART3LPEN
/** @brief Enables the clock of the USART3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART3LPEN) = 1;
}
/** @brief Disables the clock of the USART3 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_USART3RST
/** @brief Forces and releases a reset on the USART3 peripheral. */
__STATIC_INLINE void XPD_USART3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART3RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART6EN
/** @brief Enables the clock of the USART6 peripheral. */
__STATIC_INLINE void XPD_USART6_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,USART6EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the USART6 peripheral. */
__STATIC_INLINE void XPD_USART6_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,USART6EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_USART6LPEN
/** @brief Enables the clock of the USART6 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART6_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART6LPEN) = 1;
}
/** @brief Disables the clock of the USART6 peripheral during sleep mode. */
__STATIC_INLINE void XPD_USART6_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART6LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_USART6RST
/** @brief Forces and releases a reset on the USART6 peripheral. */
__STATIC_INLINE void XPD_USART6_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART6RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART6RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_WWDGEN
/** @brief Enables the clock of the WWDG peripheral. */
__STATIC_INLINE void XPD_WWDG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,WWDGEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
/** @brief Disables the clock of the WWDG peripheral. */
__STATIC_INLINE void XPD_WWDG_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,WWDGEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_WWDGLPEN
/** @brief Enables the clock of the WWDG peripheral during sleep mode. */
__STATIC_INLINE void XPD_WWDG_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,WWDGLPEN) = 1;
}
/** @brief Disables the clock of the WWDG peripheral during sleep mode. */
__STATIC_INLINE void XPD_WWDG_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,WWDGLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_WWDGRST
/** @brief Forces and releases a reset on the WWDG peripheral. */
__STATIC_INLINE void XPD_WWDG_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 1;
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 0;
}
#endif

/** @} */

/** @defgroup RCC_Generated_Macros RCC Generated Macros
 * @{ */

#define RCC_ADC_CLOCKSOURCE              PCLK2
#define RCC_ADC1_CLOCKSOURCE             PCLK2
#define RCC_ADC2_CLOCKSOURCE             PCLK2
#define RCC_ADC3_CLOCKSOURCE             PCLK2
#define RCC_CAN1_CLOCKSOURCE             PCLK1
#define RCC_CAN2_CLOCKSOURCE             PCLK1
#define RCC_CEC_CLOCKSOURCE              PCLK1
#define RCC_CRC_CLOCKSOURCE              HCLK
#define RCC_CRYP_CLOCKSOURCE             HCLK
#define RCC_DAC_CLOCKSOURCE              PCLK1
#define RCC_DCMI_CLOCKSOURCE             HCLK
#define RCC_DMA1_CLOCKSOURCE             HCLK
#define RCC_DMA2_CLOCKSOURCE             HCLK
#define RCC_DMA2D_CLOCKSOURCE            HCLK
#define RCC_DSI_CLOCKSOURCE              PCLK2
#define RCC_ETHMAC_CLOCKSOURCE           HCLK
#define RCC_FMC_CLOCKSOURCE              HCLK
#define RCC_FMP_I2C1_CLOCKSOURCE         PCLK1
#define RCC_FSMC_CLOCKSOURCE             HCLK
#define RCC_GPIOA_CLOCKSOURCE            HCLK
#define RCC_GPIOB_CLOCKSOURCE            HCLK
#define RCC_GPIOC_CLOCKSOURCE            HCLK
#define RCC_GPIOD_CLOCKSOURCE            HCLK
#define RCC_GPIOE_CLOCKSOURCE            HCLK
#define RCC_GPIOF_CLOCKSOURCE            HCLK
#define RCC_GPIOG_CLOCKSOURCE            HCLK
#define RCC_GPIOH_CLOCKSOURCE            HCLK
#define RCC_GPIOI_CLOCKSOURCE            HCLK
#define RCC_GPIOJ_CLOCKSOURCE            HCLK
#define RCC_GPIOK_CLOCKSOURCE            HCLK
#define RCC_HASH_CLOCKSOURCE             HCLK
#define RCC_I2C1_CLOCKSOURCE             PCLK1
#define RCC_I2C2_CLOCKSOURCE             PCLK1
#define RCC_I2C3_CLOCKSOURCE             PCLK1
#define RCC_LTDC_CLOCKSOURCE             PCLK2
#define RCC_OTG_FS_CLOCKSOURCE           HCLK
#define RCC_OTG_HS_CLOCKSOURCE           HCLK
#define RCC_PWR_CLOCKSOURCE              PCLK1
#define RCC_QSPI_CLOCKSOURCE             HCLK
#define RCC_RNG_CLOCKSOURCE              HCLK
#define RCC_SAI1_CLOCKSOURCE             PCLK2
#define RCC_SAI2_CLOCKSOURCE             PCLK2
#define RCC_SDIO_CLOCKSOURCE             PCLK2
#define RCC_SPDIFRX_CLOCKSOURCE          PCLK1
#define RCC_SPI1_CLOCKSOURCE             PCLK2
#define RCC_SPI2_CLOCKSOURCE             PCLK1
#define RCC_SPI3_CLOCKSOURCE             PCLK1
#define RCC_SPI4_CLOCKSOURCE             PCLK2
#define RCC_SPI5_CLOCKSOURCE             PCLK2
#define RCC_SPI6_CLOCKSOURCE             PCLK2
#define RCC_SYSCFG_CLOCKSOURCE           PCLK2
#define RCC_TIM1_CLOCKSOURCE             PCLK2
#define RCC_TIM10_CLOCKSOURCE            PCLK2
#define RCC_TIM11_CLOCKSOURCE            PCLK2
#define RCC_TIM12_CLOCKSOURCE            PCLK1
#define RCC_TIM13_CLOCKSOURCE            PCLK1
#define RCC_TIM14_CLOCKSOURCE            PCLK1
#define RCC_TIM2_CLOCKSOURCE             PCLK1
#define RCC_TIM3_CLOCKSOURCE             PCLK1
#define RCC_TIM4_CLOCKSOURCE             PCLK1
#define RCC_TIM5_CLOCKSOURCE             PCLK1
#define RCC_TIM6_CLOCKSOURCE             PCLK1
#define RCC_TIM7_CLOCKSOURCE             PCLK1
#define RCC_TIM8_CLOCKSOURCE             PCLK2
#define RCC_TIM9_CLOCKSOURCE             PCLK2
#define RCC_UART4_CLOCKSOURCE            PCLK1
#define RCC_UART5_CLOCKSOURCE            PCLK1
#define RCC_UART7_CLOCKSOURCE            PCLK1
#define RCC_UART8_CLOCKSOURCE            PCLK1
#define RCC_USART1_CLOCKSOURCE           PCLK2
#define RCC_USART2_CLOCKSOURCE           PCLK1
#define RCC_USART3_CLOCKSOURCE           PCLK1
#define RCC_USART6_CLOCKSOURCE           PCLK2
#define RCC_WWDG_CLOCKSOURCE             PCLK1

/** @} */

/** @} */

/** @} */

#endif /* XPD_RCC_GEN_H_ */
