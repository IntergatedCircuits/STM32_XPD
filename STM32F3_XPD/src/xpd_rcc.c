/**
  ******************************************************************************
  * @file    xpd_rcc.c
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-11-21
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

typedef union {
    struct {
        uint16_t bitIndex :  5;
        uint16_t regIndex : 11;
    };
    uint16_t w;
} rccPosType;

#define PPOS    ((rccPosType)PeriphPos)

/** @defgroup RCC_Peripheral_Control_Exported_Functions RCC Peripheral Control Exported Functions
 * @{ */

/**
 * @brief Enables the clock of the peripheral.
 * @param PeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void XPD_RCC_ClockEnable(RCC_PositionType PeriphPos)
{
#ifdef RCC_BB
    __IO uint32_t *pEN = PERIPH_BB(&RCC->AHBENR.w);
    pEN[PeriphPos] = 1;

    /* Read back to ensure effect */
    (void) pEN[PeriphPos];
#else
    __IO uint32_t *pENR = &RCC->AHBENR.w + PPOS.regIndex;
    SET_BIT(*pENR, 1 << PPOS.bitIndex);

    /* Read back to ensure effect */
    (void) *pENR;
#endif
}

/**
 * @brief Disables the clock of the peripheral.
 * @param PeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void XPD_RCC_ClockDisable(RCC_PositionType PeriphPos)
{
#ifdef RCC_BB
    __IO uint32_t *pEN = PERIPH_BB(&RCC->AHBENR.w);
    pEN[PeriphPos] = 0;
#else
    __IO uint32_t *pENR = &RCC->AHBENR.w + PPOS.regIndex;
    CLEAR_BIT(*pENR, 1 << PPOS.bitIndex);
#endif
}

/**
 * @brief Forces and releases a reset on the peripheral.
 * @param PeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void XPD_RCC_Reset(RCC_PositionType PeriphPos)
{
    /* These devices have different layout for RSTR than ENR registers */
#ifdef RCC_BB
    __IO uint32_t *pRST = PERIPH_BB(&RCC->AHBRSTR.w);
    if ((PeriphPos & (~0x1F)) != RCC_POS_AHB)
    {
        pRST = PERIPH_BB(&RCC->CIR.w);
    }
    pRST[PeriphPos] = 1;
    pRST[PeriphPos] = 0;
#else
    __IO uint32_t *pRSTR = &RCC->AHBRSTR.w;

    if ((PeriphPos & (~0x1F)) != RCC_POS_AHB)
    {
        pRSTR = &RCC->CIR.w + PPOS.regIndex;
    }
    SET_BIT  (*pRSTR, 1 << PPOS.bitIndex);
    CLEAR_BIT(*pRSTR, 1 << PPOS.bitIndex);
#endif
}

/** @} */

/** @} */

/** @} */
