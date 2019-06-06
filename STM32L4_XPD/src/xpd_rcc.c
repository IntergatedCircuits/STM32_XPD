/**
  ******************************************************************************
  * @file    xpd_rcc.c
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers RCC Peripherals Module
  *
  * Copyright (c) 2018 Benedek Kupper
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  */
#include <xpd_rcc.h>

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

#define PPOS    ((rccPosType)ePeriphPos)

/** @defgroup RCC_Peripheral_Control_Exported_Functions RCC Peripheral Control Exported Functions
 * @{ */

/**
 * @brief Enables the clock of the peripheral.
 * @param ePeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void RCC_vClockEnable(RCC_PositionType ePeriphPos)
{
#ifdef RCC_BB
    __IO uint32_t *pulEN = PERIPH_BB(&RCC->AHB1ENR.w);
    pulEN[ePeriphPos] = 1;

    /* Read back to ensure effect */
    (void) pulEN[ePeriphPos];
#else
    __IO uint32_t *pulENR = &RCC->AHB1ENR.w + PPOS.regIndex;
    SET_BIT(*pulENR, 1 << PPOS.bitIndex);

    /* Read back to ensure effect */
    (void) *pulENR;
#endif
}

/**
 * @brief Disables the clock of the peripheral.
 * @param ePeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void RCC_vClockDisable(RCC_PositionType ePeriphPos)
{
#ifdef RCC_BB
    __IO uint32_t *pulEN = PERIPH_BB(&RCC->AHB1ENR.w);
    pulEN[ePeriphPos] = 0;
#else
    __IO uint32_t *pulENR = &RCC->AHB1ENR.w + PPOS.regIndex;
    CLEAR_BIT(*pulENR, 1 << PPOS.bitIndex);
#endif
}

/**
 * @brief Enables the clock of the peripheral during sleep mode.
 * @param ePeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void RCC_vSleepClockEnable(RCC_PositionType ePeriphPos)
{
#ifdef RCC_BB
    __IO uint32_t *pulSMEN = PERIPH_BB(&RCC->AHB1SMENR.w);
    pulSMEN[ePeriphPos] = 1;
#else
    __IO uint32_t *pulSMENR = &RCC->AHB1SMENR.w + PPOS.regIndex;
    SET_BIT(*pulSMENR, 1 << PPOS.bitIndex);
#endif
}

/**
 * @brief Disables the clock of the peripheral during sleep mode.
 * @param ePeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void RCC_vSleepClockDisable(RCC_PositionType ePeriphPos)
{
#ifdef RCC_BB
    __IO uint32_t *pulSMEN = PERIPH_BB(&RCC->AHB1SMENR.w);
    pulSMEN[ePeriphPos] = 0;
#else
    __IO uint32_t *pulSMENR = &RCC->AHB1SMENR.w + PPOS.regIndex;
    CLEAR_BIT(*pulSMENR, 1 << PPOS.bitIndex);
#endif
}

/**
 * @brief Forces and releases a reset on the peripheral.
 * @param ePeriphPos: Relative position of the peripheral control bit
 *        in the RCC register space
 */
void RCC_vReset(RCC_PositionType ePeriphPos)
{
#ifdef RCC_BB
    __IO uint32_t *pulRST = PERIPH_BB(&RCC->AHB1RSTR.w);
    pulRST[ePeriphPos] = 1;
    pulRST[ePeriphPos] = 0;
#else
    __IO uint32_t *pulRSTR = &RCC->AHB1RSTR.w + PPOS.regIndex;
    SET_BIT  (*pulRSTR, 1 << PPOS.bitIndex);
    CLEAR_BIT(*pulRSTR, 1 << PPOS.bitIndex);
#endif
}

/** @} */

/** @} */

/** @} */
