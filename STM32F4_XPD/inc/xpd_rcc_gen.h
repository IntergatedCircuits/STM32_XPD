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

#ifdef RCC_APB2RSTR_ADCRST
__STATIC_INLINE void XPD_ADC_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,ADCRST) = 1;
    RCC_REG_BIT(APB2RSTR,ADCRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_ADC1EN
__STATIC_INLINE void XPD_ADC1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,ADC1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_ADC1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,ADC1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_ADC1LPEN
__STATIC_INLINE void XPD_ADC1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC1LPEN) = 1;
}
__STATIC_INLINE void XPD_ADC1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC1LPEN) = 0;
}
#endif
#ifdef RCC_APB2ENR_ADC2EN
__STATIC_INLINE void XPD_ADC2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,ADC2EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_ADC2_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,ADC2EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_ADC2LPEN
__STATIC_INLINE void XPD_ADC2_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC2LPEN) = 1;
}
__STATIC_INLINE void XPD_ADC2_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC2LPEN) = 0;
}
#endif
#ifdef RCC_APB2ENR_ADC3EN
__STATIC_INLINE void XPD_ADC3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,ADC3EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_ADC3_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,ADC3EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_ADC3LPEN
__STATIC_INLINE void XPD_ADC3_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC3LPEN) = 1;
}
__STATIC_INLINE void XPD_ADC3_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,ADC3LPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_BKPSRAMEN
__STATIC_INLINE void XPD_BKPSRAM_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,BKPSRAMEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_BKPSRAM_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,BKPSRAMEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_BKPSRAMLPEN
__STATIC_INLINE void XPD_BKPSRAM_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,BKPSRAMLPEN) = 1;
}
__STATIC_INLINE void XPD_BKPSRAM_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,BKPSRAMLPEN) = 0;
}
#endif
#ifdef RCC_APB1ENR_CAN1EN
__STATIC_INLINE void XPD_CAN1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,CAN1EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_CAN1_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,CAN1EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_CAN1LPEN
__STATIC_INLINE void XPD_CAN1_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN1LPEN) = 1;
}
__STATIC_INLINE void XPD_CAN1_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN1LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_CAN1RST
__STATIC_INLINE void XPD_CAN1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CAN1RST) = 1;
    RCC_REG_BIT(APB1RSTR,CAN1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_CAN2EN
__STATIC_INLINE void XPD_CAN2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,CAN2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_CAN2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,CAN2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_CAN2LPEN
__STATIC_INLINE void XPD_CAN2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN2LPEN) = 1;
}
__STATIC_INLINE void XPD_CAN2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CAN2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_CAN2RST
__STATIC_INLINE void XPD_CAN2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CAN2RST) = 1;
    RCC_REG_BIT(APB1RSTR,CAN2RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_CCMDATARAMEN
__STATIC_INLINE void XPD_CCMDATARAM_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,CCMDATARAMEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_CCMDATARAM_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,CCMDATARAMEN) = 0;
}
#endif
#ifdef RCC_APB1ENR_CECEN
__STATIC_INLINE void XPD_CEC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,CECEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_CEC_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,CECEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_CECLPEN
__STATIC_INLINE void XPD_CEC_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CECLPEN) = 1;
}
__STATIC_INLINE void XPD_CEC_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,CECLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_CECRST
__STATIC_INLINE void XPD_CEC_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CECRST) = 1;
    RCC_REG_BIT(APB1RSTR,CECRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_CRCEN
__STATIC_INLINE void XPD_CRC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,CRCEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_CRC_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,CRCEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_CRCLPEN
__STATIC_INLINE void XPD_CRC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,CRCLPEN) = 1;
}
__STATIC_INLINE void XPD_CRC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,CRCLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_CRCRST
__STATIC_INLINE void XPD_CRC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,CRCRST) = 1;
    RCC_REG_BIT(AHB1RSTR,CRCRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_CRYPEN
__STATIC_INLINE void XPD_CRYP_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,CRYPEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_CRYP_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,CRYPEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_CRYPLPEN
__STATIC_INLINE void XPD_CRYP_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,CRYPLPEN) = 1;
}
__STATIC_INLINE void XPD_CRYP_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,CRYPLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_CRYPRST
__STATIC_INLINE void XPD_CRYP_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,CRYPRST) = 1;
    RCC_REG_BIT(AHB2RSTR,CRYPRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_DACEN
__STATIC_INLINE void XPD_DAC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,DACEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_DAC_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,DACEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_DACLPEN
__STATIC_INLINE void XPD_DAC_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,DACLPEN) = 1;
}
__STATIC_INLINE void XPD_DAC_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,DACLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_DACRST
__STATIC_INLINE void XPD_DAC_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,DACRST) = 1;
    RCC_REG_BIT(APB1RSTR,DACRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_DCMIEN
__STATIC_INLINE void XPD_DCMI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,DCMIEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_DCMI_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,DCMIEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_DCMILPEN
__STATIC_INLINE void XPD_DCMI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,DCMILPEN) = 1;
}
__STATIC_INLINE void XPD_DCMI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,DCMILPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_DCMIRST
__STATIC_INLINE void XPD_DCMI_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,DCMIRST) = 1;
    RCC_REG_BIT(AHB2RSTR,DCMIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA1EN
__STATIC_INLINE void XPD_DMA1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,DMA1EN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_DMA1_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,DMA1EN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_DMA1LPEN
__STATIC_INLINE void XPD_DMA1_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA1LPEN) = 1;
}
__STATIC_INLINE void XPD_DMA1_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA1LPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_DMA1RST
__STATIC_INLINE void XPD_DMA1_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA1RST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA1RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA2EN
__STATIC_INLINE void XPD_DMA2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,DMA2EN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_DMA2_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,DMA2EN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_DMA2LPEN
__STATIC_INLINE void XPD_DMA2_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2LPEN) = 1;
}
__STATIC_INLINE void XPD_DMA2_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2LPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_DMA2RST
__STATIC_INLINE void XPD_DMA2_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA2RST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA2RST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_DMA2DEN
__STATIC_INLINE void XPD_DMA2D_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,DMA2DEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_DMA2D_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,DMA2DEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_DMA2DLPEN
__STATIC_INLINE void XPD_DMA2D_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2DLPEN) = 1;
}
__STATIC_INLINE void XPD_DMA2D_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,DMA2DLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_DMA2DRST
__STATIC_INLINE void XPD_DMA2D_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,DMA2DRST) = 1;
    RCC_REG_BIT(AHB1RSTR,DMA2DRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_DSIEN
__STATIC_INLINE void XPD_DSI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,DSIEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_DSI_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,DSIEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_DSILPEN
__STATIC_INLINE void XPD_DSI_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,DSILPEN) = 1;
}
__STATIC_INLINE void XPD_DSI_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,DSILPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_DSIRST
__STATIC_INLINE void XPD_DSI_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,DSIRST) = 1;
    RCC_REG_BIT(APB2RSTR,DSIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACEN
__STATIC_INLINE void XPD_ETHMAC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_ETHMAC_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACLPEN
__STATIC_INLINE void XPD_ETHMAC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACLPEN) = 1;
}
__STATIC_INLINE void XPD_ETHMAC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_ETHMACRST
__STATIC_INLINE void XPD_ETHMAC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,ETHMACRST) = 1;
    RCC_REG_BIT(AHB1RSTR,ETHMACRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACPTPEN
__STATIC_INLINE void XPD_ETHMAC_PTP_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACPTPEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_ETHMAC_PTP_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACPTPEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACPTPLPEN
__STATIC_INLINE void XPD_ETHMAC_PTP_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACPTPLPEN) = 1;
}
__STATIC_INLINE void XPD_ETHMAC_PTP_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACPTPLPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACRXEN
__STATIC_INLINE void XPD_ETHMAC_RX_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACRXEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_ETHMAC_RX_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACRXEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACRXLPEN
__STATIC_INLINE void XPD_ETHMAC_RX_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACRXLPEN) = 1;
}
__STATIC_INLINE void XPD_ETHMAC_RX_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACRXLPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_ETHMACTXEN
__STATIC_INLINE void XPD_ETHMAC_TX_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,ETHMACTXEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_ETHMAC_TX_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,ETHMACTXEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_ETHMACTXLPEN
__STATIC_INLINE void XPD_ETHMAC_TX_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACTXLPEN) = 1;
}
__STATIC_INLINE void XPD_ETHMAC_TX_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,ETHMACTXLPEN) = 0;
}
#endif
#ifdef RCC_AHB3ENR_FMCEN
__STATIC_INLINE void XPD_FMC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB3ENR,FMCEN) = 1;
    r = RCC->AHB3ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_FMC_DisableClock(void)
{
    RCC_REG_BIT(AHB3ENR,FMCEN) = 0;
}
#endif
#ifdef RCC_AHB3LPENR_FMCLPEN
__STATIC_INLINE void XPD_FMC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FMCLPEN) = 1;
}
__STATIC_INLINE void XPD_FMC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FMCLPEN) = 0;
}
#endif
#ifdef RCC_AHB3RSTR_FMCRST
__STATIC_INLINE void XPD_FMC_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,FMCRST) = 1;
    RCC_REG_BIT(AHB3RSTR,FMCRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_FMPI2C1EN
__STATIC_INLINE void XPD_FMP_I2C1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,FMPI2C1EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_FMP_I2C1_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,FMPI2C1EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_FMPI2C1LPEN
__STATIC_INLINE void XPD_FMP_I2C1_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,FMPI2C1LPEN) = 1;
}
__STATIC_INLINE void XPD_FMP_I2C1_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,FMPI2C1LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_FMPI2C1RST
__STATIC_INLINE void XPD_FMP_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,FMPI2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR,FMPI2C1RST) = 0;
}
#endif
#ifdef RCC_AHB3ENR_FSMCEN
__STATIC_INLINE void XPD_FSMC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB3ENR,FSMCEN) = 1;
    r = RCC->AHB3ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_FSMC_DisableClock(void)
{
    RCC_REG_BIT(AHB3ENR,FSMCEN) = 0;
}
#endif
#ifdef RCC_AHB3LPENR_FSMCLPEN
__STATIC_INLINE void XPD_FSMC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FSMCLPEN) = 1;
}
__STATIC_INLINE void XPD_FSMC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,FSMCLPEN) = 0;
}
#endif
#ifdef RCC_AHB3RSTR_FSMCRST
__STATIC_INLINE void XPD_FSMC_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,FSMCRST) = 1;
    RCC_REG_BIT(AHB3RSTR,FSMCRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOAEN
__STATIC_INLINE void XPD_GPIOA_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOAEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOA_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOAEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOALPEN
__STATIC_INLINE void XPD_GPIOA_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOALPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOA_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOALPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOARST
__STATIC_INLINE void XPD_GPIOA_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOARST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOARST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOBEN
__STATIC_INLINE void XPD_GPIOB_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOBEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOB_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOBEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOBLPEN
__STATIC_INLINE void XPD_GPIOB_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOBLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOB_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOBLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOBRST
__STATIC_INLINE void XPD_GPIOB_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOBRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOBRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOCEN
__STATIC_INLINE void XPD_GPIOC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOCEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOC_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOCEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOCLPEN
__STATIC_INLINE void XPD_GPIOC_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOCLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOC_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOCLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOCRST
__STATIC_INLINE void XPD_GPIOC_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOCRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOCRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIODEN
__STATIC_INLINE void XPD_GPIOD_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIODEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOD_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIODEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIODLPEN
__STATIC_INLINE void XPD_GPIOD_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIODLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOD_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIODLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIODRST
__STATIC_INLINE void XPD_GPIOD_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIODRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIODRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOEEN
__STATIC_INLINE void XPD_GPIOE_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOEEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOE_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOEEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOELPEN
__STATIC_INLINE void XPD_GPIOE_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOELPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOE_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOELPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOERST
__STATIC_INLINE void XPD_GPIOE_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOERST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOERST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOFEN
__STATIC_INLINE void XPD_GPIOF_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOFEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOF_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOFEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOFLPEN
__STATIC_INLINE void XPD_GPIOF_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOFLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOF_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOFLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOFRST
__STATIC_INLINE void XPD_GPIOF_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOFRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOFRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOGEN
__STATIC_INLINE void XPD_GPIOG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOGEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOG_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOGEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOGLPEN
__STATIC_INLINE void XPD_GPIOG_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOGLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOG_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOGLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOGRST
__STATIC_INLINE void XPD_GPIOG_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOGRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOGRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOHEN
__STATIC_INLINE void XPD_GPIOH_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOHEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOH_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOHEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOHLPEN
__STATIC_INLINE void XPD_GPIOH_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOHLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOH_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOHLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOHRST
__STATIC_INLINE void XPD_GPIOH_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOHRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOHRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOIEN
__STATIC_INLINE void XPD_GPIOI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOIEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOI_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOIEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOILPEN
__STATIC_INLINE void XPD_GPIOI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOILPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOILPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOIRST
__STATIC_INLINE void XPD_GPIOI_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOIRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOIRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOJEN
__STATIC_INLINE void XPD_GPIOJ_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOJEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOJ_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOJEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOJLPEN
__STATIC_INLINE void XPD_GPIOJ_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOJLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOJ_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOJLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOJRST
__STATIC_INLINE void XPD_GPIOJ_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOJRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOJRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_GPIOKEN
__STATIC_INLINE void XPD_GPIOK_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,GPIOKEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_GPIOK_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,GPIOKEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_GPIOKLPEN
__STATIC_INLINE void XPD_GPIOK_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOKLPEN) = 1;
}
__STATIC_INLINE void XPD_GPIOK_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,GPIOKLPEN) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_GPIOKRST
__STATIC_INLINE void XPD_GPIOK_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,GPIOKRST) = 1;
    RCC_REG_BIT(AHB1RSTR,GPIOKRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_HASHEN
__STATIC_INLINE void XPD_HASH_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,HASHEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_HASH_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,HASHEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_HASHLPEN
__STATIC_INLINE void XPD_HASH_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,HASHLPEN) = 1;
}
__STATIC_INLINE void XPD_HASH_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,HASHLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_HASHRST
__STATIC_INLINE void XPD_HASH_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,HASHRST) = 1;
    RCC_REG_BIT(AHB2RSTR,HASHRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C1EN
__STATIC_INLINE void XPD_I2C1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,I2C1EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_I2C1_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,I2C1EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_I2C1LPEN
__STATIC_INLINE void XPD_I2C1_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C1LPEN) = 1;
}
__STATIC_INLINE void XPD_I2C1_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C1LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_I2C1RST
__STATIC_INLINE void XPD_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C2EN
__STATIC_INLINE void XPD_I2C2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,I2C2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_I2C2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,I2C2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_I2C2LPEN
__STATIC_INLINE void XPD_I2C2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C2LPEN) = 1;
}
__STATIC_INLINE void XPD_I2C2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_I2C2RST
__STATIC_INLINE void XPD_I2C2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C3EN
__STATIC_INLINE void XPD_I2C3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,I2C3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_I2C3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,I2C3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_I2C3LPEN
__STATIC_INLINE void XPD_I2C3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C3LPEN) = 1;
}
__STATIC_INLINE void XPD_I2C3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,I2C3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_I2C3RST
__STATIC_INLINE void XPD_I2C3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_LTDCEN
__STATIC_INLINE void XPD_LTDC_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,LTDCEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_LTDC_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,LTDCEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_LTDCLPEN
__STATIC_INLINE void XPD_LTDC_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,LTDCLPEN) = 1;
}
__STATIC_INLINE void XPD_LTDC_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,LTDCLPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_LTDCRST
__STATIC_INLINE void XPD_LTDC_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,LTDCRST) = 1;
    RCC_REG_BIT(APB2RSTR,LTDCRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_OTGFSEN
__STATIC_INLINE void XPD_OTG_FS_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,OTGFSEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_OTG_FS_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,OTGFSEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_OTGFSLPEN
__STATIC_INLINE void XPD_OTG_FS_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,OTGFSLPEN) = 1;
}
__STATIC_INLINE void XPD_OTG_FS_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,OTGFSLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_OTGFSRST
__STATIC_INLINE void XPD_OTG_FS_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 1;
    RCC_REG_BIT(AHB2RSTR,OTGFSRST) = 0;
}
#endif
#ifdef RCC_AHB1RSTR_OTGHRST
__STATIC_INLINE void XPD_OTG_HS_Reset(void)
{
    RCC_REG_BIT(AHB1RSTR,OTGHRST) = 1;
    RCC_REG_BIT(AHB1RSTR,OTGHRST) = 0;
}
#endif
#ifdef RCC_AHB1ENR_OTGHSEN
__STATIC_INLINE void XPD_OTG_HS_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,OTGHSEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_OTG_HS_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,OTGHSEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_OTGHSLPEN
__STATIC_INLINE void XPD_OTG_HS_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSLPEN) = 1;
}
__STATIC_INLINE void XPD_OTG_HS_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSLPEN) = 0;
}
#endif
#ifdef RCC_AHB1ENR_OTGHSULPIEN
__STATIC_INLINE void XPD_OTG_HS_ULPI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB1ENR,OTGHSULPIEN) = 1;
    r = RCC->AHB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_OTG_HS_ULPI_DisableClock(void)
{
    RCC_REG_BIT(AHB1ENR,OTGHSULPIEN) = 0;
}
#endif
#ifdef RCC_AHB1LPENR_OTGHSULPILPEN
__STATIC_INLINE void XPD_OTG_HS_ULPI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSULPILPEN) = 1;
}
__STATIC_INLINE void XPD_OTG_HS_ULPI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB1LPENR,OTGHSULPILPEN) = 0;
}
#endif
#ifdef RCC_APB1ENR_PWREN
__STATIC_INLINE void XPD_PWR_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,PWREN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_PWR_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,PWREN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_PWRLPEN
__STATIC_INLINE void XPD_PWR_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,PWRLPEN) = 1;
}
__STATIC_INLINE void XPD_PWR_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,PWRLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_PWRRST
__STATIC_INLINE void XPD_PWR_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,PWRRST) = 1;
    RCC_REG_BIT(APB1RSTR,PWRRST) = 0;
}
#endif
#ifdef RCC_AHB3ENR_QSPIEN
__STATIC_INLINE void XPD_QSPI_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB3ENR,QSPIEN) = 1;
    r = RCC->AHB3ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_QSPI_DisableClock(void)
{
    RCC_REG_BIT(AHB3ENR,QSPIEN) = 0;
}
#endif
#ifdef RCC_AHB3LPENR_QSPILPEN
__STATIC_INLINE void XPD_QSPI_EnableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,QSPILPEN) = 1;
}
__STATIC_INLINE void XPD_QSPI_DisableInSleep(void)
{
    RCC_REG_BIT(AHB3LPENR,QSPILPEN) = 0;
}
#endif
#ifdef RCC_AHB3RSTR_QSPIRST
__STATIC_INLINE void XPD_QSPI_Reset(void)
{
    RCC_REG_BIT(AHB3RSTR,QSPIRST) = 1;
    RCC_REG_BIT(AHB3RSTR,QSPIRST) = 0;
}
#endif
#ifdef RCC_AHB2ENR_RNGEN
__STATIC_INLINE void XPD_RNG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(AHB2ENR,RNGEN) = 1;
    r = RCC->AHB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_RNG_DisableClock(void)
{
    RCC_REG_BIT(AHB2ENR,RNGEN) = 0;
}
#endif
#ifdef RCC_AHB2LPENR_RNGLPEN
__STATIC_INLINE void XPD_RNG_EnableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,RNGLPEN) = 1;
}
__STATIC_INLINE void XPD_RNG_DisableInSleep(void)
{
    RCC_REG_BIT(AHB2LPENR,RNGLPEN) = 0;
}
#endif
#ifdef RCC_AHB2RSTR_RNGRST
__STATIC_INLINE void XPD_RNG_Reset(void)
{
    RCC_REG_BIT(AHB2RSTR,RNGRST) = 1;
    RCC_REG_BIT(AHB2RSTR,RNGRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SAI1EN
__STATIC_INLINE void XPD_SAI1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SAI1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SAI1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SAI1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SAI1LPEN
__STATIC_INLINE void XPD_SAI1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI1LPEN) = 1;
}
__STATIC_INLINE void XPD_SAI1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SAI1RST
__STATIC_INLINE void XPD_SAI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SAI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SAI1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SAI2EN
__STATIC_INLINE void XPD_SAI2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SAI2EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SAI2_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SAI2EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SAI2LPEN
__STATIC_INLINE void XPD_SAI2_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI2LPEN) = 1;
}
__STATIC_INLINE void XPD_SAI2_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SAI2LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SAI2RST
__STATIC_INLINE void XPD_SAI2_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SAI2RST) = 1;
    RCC_REG_BIT(APB2RSTR,SAI2RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SDIOEN
__STATIC_INLINE void XPD_SDIO_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SDIOEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SDIO_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SDIOEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SDIOLPEN
__STATIC_INLINE void XPD_SDIO_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SDIOLPEN) = 1;
}
__STATIC_INLINE void XPD_SDIO_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SDIOLPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SDIORST
__STATIC_INLINE void XPD_SDIO_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SDIORST) = 1;
    RCC_REG_BIT(APB2RSTR,SDIORST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPDIFRXEN
__STATIC_INLINE void XPD_SPDIFRX_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,SPDIFRXEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SPDIFRX_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,SPDIFRXEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_SPDIFRXLPEN
__STATIC_INLINE void XPD_SPDIFRX_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPDIFRXLPEN) = 1;
}
__STATIC_INLINE void XPD_SPDIFRX_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPDIFRXLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_SPDIFRXRST
__STATIC_INLINE void XPD_SPDIFRX_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPDIFRXRST) = 1;
    RCC_REG_BIT(APB1RSTR,SPDIFRXRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI1EN
__STATIC_INLINE void XPD_SPI1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SPI1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI1LPEN
__STATIC_INLINE void XPD_SPI1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI1LPEN) = 1;
}
__STATIC_INLINE void XPD_SPI1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI1RST
__STATIC_INLINE void XPD_SPI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPI2EN
__STATIC_INLINE void XPD_SPI2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,SPI2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SPI2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,SPI2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_SPI2LPEN
__STATIC_INLINE void XPD_SPI2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI2LPEN) = 1;
}
__STATIC_INLINE void XPD_SPI2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_SPI2RST
__STATIC_INLINE void XPD_SPI2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPI3EN
__STATIC_INLINE void XPD_SPI3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,SPI3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SPI3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,SPI3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_SPI3LPEN
__STATIC_INLINE void XPD_SPI3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI3LPEN) = 1;
}
__STATIC_INLINE void XPD_SPI3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,SPI3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_SPI3RST
__STATIC_INLINE void XPD_SPI3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI4EN
__STATIC_INLINE void XPD_SPI4_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI4EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SPI4_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI4EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI4LPEN
__STATIC_INLINE void XPD_SPI4_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI4LPEN) = 1;
}
__STATIC_INLINE void XPD_SPI4_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI4LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI4RST
__STATIC_INLINE void XPD_SPI4_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI5EN
__STATIC_INLINE void XPD_SPI5_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI5EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SPI5_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI5EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI5LPEN
__STATIC_INLINE void XPD_SPI5_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI5LPEN) = 1;
}
__STATIC_INLINE void XPD_SPI5_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI5LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI5RST
__STATIC_INLINE void XPD_SPI5_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI5RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI5RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI6EN
__STATIC_INLINE void XPD_SPI6_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SPI6EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SPI6_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SPI6EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SPI6LPEN
__STATIC_INLINE void XPD_SPI6_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI6LPEN) = 1;
}
__STATIC_INLINE void XPD_SPI6_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SPI6LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SPI6RST
__STATIC_INLINE void XPD_SPI6_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI6RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI6RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
__STATIC_INLINE void XPD_SYSCFG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,SYSCFGEN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_SYSCFG_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,SYSCFGEN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_SYSCFGLPEN
__STATIC_INLINE void XPD_SYSCFG_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SYSCFGLPEN) = 1;
}
__STATIC_INLINE void XPD_SYSCFG_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,SYSCFGLPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
__STATIC_INLINE void XPD_SYSCFG_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 1;
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM1EN
__STATIC_INLINE void XPD_TIM1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM1LPEN
__STATIC_INLINE void XPD_TIM1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM1LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM1RST
__STATIC_INLINE void XPD_TIM1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM10EN
__STATIC_INLINE void XPD_TIM10_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM10EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM10_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM10EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM10LPEN
__STATIC_INLINE void XPD_TIM10_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM10LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM10_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM10LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM10RST
__STATIC_INLINE void XPD_TIM10_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM10RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM10RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM11EN
__STATIC_INLINE void XPD_TIM11_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM11EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM11_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM11EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM11LPEN
__STATIC_INLINE void XPD_TIM11_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM11LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM11_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM11LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM11RST
__STATIC_INLINE void XPD_TIM11_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM11RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM11RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM12EN
__STATIC_INLINE void XPD_TIM12_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM12EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM12_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM12EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM12LPEN
__STATIC_INLINE void XPD_TIM12_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM12LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM12_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM12LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM12RST
__STATIC_INLINE void XPD_TIM12_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM13EN
__STATIC_INLINE void XPD_TIM13_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM13EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM13_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM13EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM13LPEN
__STATIC_INLINE void XPD_TIM13_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM13LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM13_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM13LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM13RST
__STATIC_INLINE void XPD_TIM13_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM14EN
__STATIC_INLINE void XPD_TIM14_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM14EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM14_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM14EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM14LPEN
__STATIC_INLINE void XPD_TIM14_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM14LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM14_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM14LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM14RST
__STATIC_INLINE void XPD_TIM14_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM2EN
__STATIC_INLINE void XPD_TIM2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM2LPEN
__STATIC_INLINE void XPD_TIM2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM2LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM2RST
__STATIC_INLINE void XPD_TIM2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM3EN
__STATIC_INLINE void XPD_TIM3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM3LPEN
__STATIC_INLINE void XPD_TIM3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM3LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM3RST
__STATIC_INLINE void XPD_TIM3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM4EN
__STATIC_INLINE void XPD_TIM4_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM4EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM4_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM4EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM4LPEN
__STATIC_INLINE void XPD_TIM4_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM4LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM4_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM4LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM4RST
__STATIC_INLINE void XPD_TIM4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM5EN
__STATIC_INLINE void XPD_TIM5_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM5EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM5_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM5EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM5LPEN
__STATIC_INLINE void XPD_TIM5_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM5LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM5_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM5LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM5RST
__STATIC_INLINE void XPD_TIM5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM6EN
__STATIC_INLINE void XPD_TIM6_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM6EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM6_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM6EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM6LPEN
__STATIC_INLINE void XPD_TIM6_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM6LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM6_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM6LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM6RST
__STATIC_INLINE void XPD_TIM6_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM7EN
__STATIC_INLINE void XPD_TIM7_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,TIM7EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM7_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,TIM7EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_TIM7LPEN
__STATIC_INLINE void XPD_TIM7_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM7LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM7_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,TIM7LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_TIM7RST
__STATIC_INLINE void XPD_TIM7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM8EN
__STATIC_INLINE void XPD_TIM8_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM8EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM8_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM8EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM8LPEN
__STATIC_INLINE void XPD_TIM8_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM8LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM8_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM8LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM8RST
__STATIC_INLINE void XPD_TIM8_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM9EN
__STATIC_INLINE void XPD_TIM9_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,TIM9EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_TIM9_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,TIM9EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_TIM9LPEN
__STATIC_INLINE void XPD_TIM9_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM9LPEN) = 1;
}
__STATIC_INLINE void XPD_TIM9_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,TIM9LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_TIM9RST
__STATIC_INLINE void XPD_TIM9_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM9RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM9RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART4EN
__STATIC_INLINE void XPD_UART4_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART4EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_UART4_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART4EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART4LPEN
__STATIC_INLINE void XPD_UART4_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART4LPEN) = 1;
}
__STATIC_INLINE void XPD_UART4_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART4LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART4RST
__STATIC_INLINE void XPD_UART4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART4RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART5EN
__STATIC_INLINE void XPD_UART5_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART5EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_UART5_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART5EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART5LPEN
__STATIC_INLINE void XPD_UART5_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART5LPEN) = 1;
}
__STATIC_INLINE void XPD_UART5_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART5LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART5RST
__STATIC_INLINE void XPD_UART5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART5RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART7EN
__STATIC_INLINE void XPD_UART7_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART7EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_UART7_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART7EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART7LPEN
__STATIC_INLINE void XPD_UART7_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART7LPEN) = 1;
}
__STATIC_INLINE void XPD_UART7_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART7LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART7RST
__STATIC_INLINE void XPD_UART7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART7RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART7RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART8EN
__STATIC_INLINE void XPD_UART8_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,UART8EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_UART8_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,UART8EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_UART8LPEN
__STATIC_INLINE void XPD_UART8_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART8LPEN) = 1;
}
__STATIC_INLINE void XPD_UART8_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,UART8LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_UART8RST
__STATIC_INLINE void XPD_UART8_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART8RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART8RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART1EN
__STATIC_INLINE void XPD_USART1_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,USART1EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_USART1_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,USART1EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_USART1LPEN
__STATIC_INLINE void XPD_USART1_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART1LPEN) = 1;
}
__STATIC_INLINE void XPD_USART1_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART1LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_USART1RST
__STATIC_INLINE void XPD_USART1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART1RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART2EN
__STATIC_INLINE void XPD_USART2_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,USART2EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_USART2_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,USART2EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_USART2LPEN
__STATIC_INLINE void XPD_USART2_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART2LPEN) = 1;
}
__STATIC_INLINE void XPD_USART2_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART2LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_USART2RST
__STATIC_INLINE void XPD_USART2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART2RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART3EN
__STATIC_INLINE void XPD_USART3_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,USART3EN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_USART3_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,USART3EN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_USART3LPEN
__STATIC_INLINE void XPD_USART3_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART3LPEN) = 1;
}
__STATIC_INLINE void XPD_USART3_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,USART3LPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_USART3RST
__STATIC_INLINE void XPD_USART3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART3RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART6EN
__STATIC_INLINE void XPD_USART6_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB2ENR,USART6EN) = 1;
    r = RCC->APB2ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_USART6_DisableClock(void)
{
    RCC_REG_BIT(APB2ENR,USART6EN) = 0;
}
#endif
#ifdef RCC_APB2LPENR_USART6LPEN
__STATIC_INLINE void XPD_USART6_EnableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART6LPEN) = 1;
}
__STATIC_INLINE void XPD_USART6_DisableInSleep(void)
{
    RCC_REG_BIT(APB2LPENR,USART6LPEN) = 0;
}
#endif
#ifdef RCC_APB2RSTR_USART6RST
__STATIC_INLINE void XPD_USART6_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART6RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART6RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_WWDGEN
__STATIC_INLINE void XPD_WWDG_EnableClock(void)
{
    volatile uint32_t r;
    RCC_REG_BIT(APB1ENR,WWDGEN) = 1;
    r = RCC->APB1ENR.w; /* Produce code delay */
    (void)r;
}
__STATIC_INLINE void XPD_WWDG_DisableClock(void)
{
    RCC_REG_BIT(APB1ENR,WWDGEN) = 0;
}
#endif
#ifdef RCC_APB1LPENR_WWDGLPEN
__STATIC_INLINE void XPD_WWDG_EnableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,WWDGLPEN) = 1;
}
__STATIC_INLINE void XPD_WWDG_DisableInSleep(void)
{
    RCC_REG_BIT(APB1LPENR,WWDGLPEN) = 0;
}
#endif
#ifdef RCC_APB1RSTR_WWDGRST
__STATIC_INLINE void XPD_WWDG_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 1;
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 0;
}
#endif
#endif /* XPD_RCC_GEN_H_ */
