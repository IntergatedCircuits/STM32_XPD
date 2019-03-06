/**
  ******************************************************************************
  * @file    xpd_mco.h
  * @author  Benedek Kupper
  * @version 0.1
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Master Clock Output Module
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
#ifndef __XPD_MCO_H_
#define __XPD_MCO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_gpio.h>
#include <xpd_rcc.h>

/** @addtogroup RCC
 * @{ */

/** @defgroup MCO Master Clock Output
 * @{ */

/** @defgroup MCO_Exported_Types MCO Exported Types
 * @{ */

/** @brief RCC master clock output 1 clock source types */
typedef enum
{
    MCO1_CLOCKSOURCE_NOCLOCK     = 0, /*!< MCO clock disabled */
    MCO1_CLOCKSOURCE_HSI14       = 1, /*!< HSI14 clock source */
    MCO1_CLOCKSOURCE_LSI         = 2, /*!< LSI clock source */
#ifdef LSE_VALUE_Hz
    MCO1_CLOCKSOURCE_LSE         = 3, /*!< LSE clock source */
#endif
    MCO1_CLOCKSOURCE_SYSCLK      = 4,
    MCO1_CLOCKSOURCE_HSI         = 5, /*!< HSI clock source */
#ifdef HSE_VALUE_Hz
    MCO1_CLOCKSOURCE_HSE         = 6, /*!< HSE clock source */
#endif
    MCO1_CLOCKSOURCE_PLLCLK_DIV2 = 7, /*!< PLL / 2 clock source */
#ifdef RCC_HSI48_SUPPORT
    MCO1_CLOCKSOURCE_HSI48       = 8, /*!< HSE clock source */
#endif
#ifdef RCC_CFGR_PLLNODIV
    MCO1_CLOCKSOURCE_PLLCLK      = 0x17, /*!< PLL clock source */
#endif
}MCO1_ClockSourceType;

/** @} */

#define MCO1_GPIO_PIN           PA8

/** @defgroup MCO_Exported_Functions MCO Exported Functions
 *  @brief    RCC microcontroller clock outputs
 * @{
 */

/**
 * @brief Configures a master clock output
 * @param eMCOSource: clock source of the MCO
 * @param eMCODiv: the clock division to be applied for the MCO
 */
__STATIC_INLINE void MCO_vInit(
        MCO1_ClockSourceType    eMCOSource,
        ClockDividerType        eMCODiv)
{
    static const GPIO_InitType xMCOPinCfg = {
        .Mode = GPIO_MODE_ALTERNATE,
        .AlternateMap = GPIO_MCO_AF0,
        .Output.Speed = VERY_HIGH,
        .Output.Type = GPIO_OUTPUT_PUSHPULL,
        .Pull = GPIO_PULL_FLOAT,
    };

    {
        GPIO_vInitPin(MCO1_GPIO_PIN, &xMCOPinCfg);

        RCC->CFGR.b.MCO = eMCOSource;
#ifdef RCC_CFGR_PLLNODIV
        RCC_REG_BIT(CFGR,PLLNODIV) = eMCOSource >> 4;
#endif
#ifdef RCC_CFGR_MCOPRE
        RCC->CFGR.b.MCOPRE = eMCODiv;
#endif
    }
}

/**
 * @brief Disables a master clock output
 */
__STATIC_INLINE void MCO_vDeinit(void)
{
        GPIO_vDeinitPin(MCO1_GPIO_PIN);
}

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_MCO_H_ */
