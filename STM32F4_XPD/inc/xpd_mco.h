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
    MCO1_CLOCKSOURCE_HSI = 0, /*!< HSI clock source */
#ifdef LSE_VALUE_Hz
    MCO1_CLOCKSOURCE_LSE = 1, /*!< LSE clock source */
#endif
#ifdef HSE_VALUE_Hz
    MCO1_CLOCKSOURCE_HSE = 2, /*!< HSE clock source */
#endif
    MCO1_CLOCKSOURCE_PLL = 3  /*!< PLL clock source */
}MCO1_ClockSourceType;

#ifdef RCC_CFGR_MCO2
/** @brief RCC master clock output 2 clock source types */
typedef enum
{
    MCO2_CLOCKSOURCE_SYSCLK    = 0, /*!< System clock source */
    MCO2_CLOCKSOURCE_PLLI2SCLK = 1, /*!< PLL I2S clock source */
#ifdef HSE_VALUE_Hz
    MCO2_CLOCKSOURCE_HSE       = 2, /*!< HSE clock source */
#endif
    MCO2_CLOCKSOURCE_PLL       = 3  /*!< PLL clock source */
}MCO2_ClockSourceType;
#endif

/** @} */

#define MCO1_GPIO_PIN           PA8
#define MCO2_GPIO_PIN           PC9

/** @defgroup MCO_Exported_Functions MCO Exported Functions
 *  @brief    RCC microcontroller clock outputs
 * @{
 */

/**
 * @brief Configures a master clock output
 * @param ucMCOx: the number of the MCO
 * @param eMCOSource: clock source of the MCO, either @ref MCO1_ClockSourceType
          or @ref MCO2_ClockSourceType depending on the used MCO
 * @param ucMCODiv: the clock division to be applied for the MCO [1 .. 5]
 */
__STATIC_INLINE void MCO_vInit(
        uint8_t                 ucMCOx,
        MCO1_ClockSourceType    eMCOSource,
        uint8_t                 ucMCODiv)
{
    static const GPIO_InitType xMCOPinCfg = {
        .Mode = GPIO_MODE_ALTERNATE,
        .AlternateMap = GPIO_MCO_AF0,
        .Output.Speed = VERY_HIGH,
        .Output.Type = GPIO_OUTPUT_PUSHPULL,
        .Pull = GPIO_PULL_FLOAT,
    };

    if (ucMCOx == 2)
    {
        GPIO_vInitPin(MCO2_GPIO_PIN, &xMCOPinCfg);

        RCC->CFGR.b.MCO2    = eMCOSource;
        if (ucMCODiv > 1)
        {
            RCC->CFGR.b.MCO2PRE = 4 | (ucMCODiv - 1);
        }
        else
        {
            RCC->CFGR.b.MCO2PRE = 0;
        }

#ifdef RCC_CFGR_MCO2EN
        RCC_REG_BIT(CFGR,MCO2EN) = 1;
#endif
    }
    else
    {
        GPIO_vInitPin(MCO1_GPIO_PIN, &xMCOPinCfg);

        RCC->CFGR.b.MCO1    = eMCOSource;
        if (ucMCODiv > 1)
        {
            RCC->CFGR.b.MCO1PRE = 4 | (ucMCODiv - 1);
        }
        else
        {
            RCC->CFGR.b.MCO1PRE = 0;
        }

#ifdef RCC_CFGR_MCO1EN
        RCC_REG_BIT(CFGR,MCO1EN) = 1;
#endif
    }
}

/**
 * @brief Disables a master clock output
 * @param ucMCOx: the number of the MCO
 */
__STATIC_INLINE void MCO_vDeinit(uint8_t ucMCOx)
{
    if (ucMCOx == 2)
    {
        GPIO_vDeinitPin(MCO2_GPIO_PIN);

#ifdef RCC_CFGR_MCO2EN
        RCC_REG_BIT(CFGR,MCO2EN) = 0;
#endif
    }
    else
    {
        GPIO_vDeinitPin(MCO1_GPIO_PIN);

#ifdef RCC_CFGR_MCO1EN
        RCC_REG_BIT(CFGR,MCO1EN) = 0;
#endif
    }
}

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_MCO_H_ */
