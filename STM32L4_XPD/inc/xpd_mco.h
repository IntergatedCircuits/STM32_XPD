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
#include <xpd_pwr.h>
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
    MCO1_CLOCKSOURCE_NONE   = 0, /*!< no clock source */
    MCO1_CLOCKSOURCE_SYSCLK = 1, /*!< SYSCLK clock source */
    MCO1_CLOCKSOURCE_MSI    = 2, /*!< MSI clock source */
    MCO1_CLOCKSOURCE_HSI    = 3, /*!< HSI clock source */
#ifdef HSE_VALUE_Hz
    MCO1_CLOCKSOURCE_HSE    = 4, /*!< HSE clock source */
#endif
    MCO1_CLOCKSOURCE_PLL    = 5, /*!< PLL clock source */
    MCO1_CLOCKSOURCE_LSI    = 6, /*!< LSI clock source */
#ifdef LSE_VALUE_Hz
    MCO1_CLOCKSOURCE_LSE    = 7, /*!< LSE clock source */
#endif
#ifdef RCC_HSI48_SUPPORT
    MCO1_CLOCKSOURCE_HSI48  = 8, /*!< HSI48 clock source */
#endif
}MCO1_ClockSourceType;

/** @brief RCC low-speed clock output clock source types */
typedef enum
{
    LSCO_CLOCKSOURCE_LSI    = 0, /*!< LSI clock source */
#ifdef LSE_VALUE_Hz
    LSCO_CLOCKSOURCE_LSE    = 1, /*!< LSE clock source */
#endif
}LSCO_ClockSourceType;

/** @} */

#define MCO1_GPIO_PIN           PA8
#define LSCO_GPIO_PIN           PA2

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

        RCC->CFGR.b.MCOSEL = eMCOSource;
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

/**
 * @brief Configures the dedicated low-speed clock output
 * @param eLSCOSource: clock source of the LSCO
 */
__STATIC_INLINE void LSCO_vInit(LSCO_ClockSourceType eLSCOSource)
{
    const GPIO_InitType xLSCOPinCfg = {
        .Mode = GPIO_MODE_ANALOG,
        .AlternateMap = 0,
        .Pull = GPIO_PULL_FLOAT,
        .PowerDownPull = GPIO_PULL_FLOAT,
    };
    uint32_t ulCR1 = PWR->CR1.w;

    /* Access must be granted first */
    PWR_REG_BIT(CR1,DBP) = 1;
    {
        /* LSCO map: PA2 */
        GPIO_vInitPin(LSCO_GPIO_PIN, &xLSCOPinCfg);

        RCC_REG_BIT(BDCR,LSCOSEL) = eLSCOSource;
        RCC_REG_BIT(BDCR,LSCOEN)  = 1;
    }
    /* Restore access state */
    PWR->CR1.w = ulCR1;
}

/**
 * @brief Disables the dedicated low-speed clock output
 */
__STATIC_INLINE void LSCO_vDeinit(void)
{
    uint32_t ulCR1 = PWR->CR1.w;

    /* Access must be granted first */
    PWR_REG_BIT(CR1,DBP) = 1;
    {
        RCC_REG_BIT(BDCR,LSCOEN)  = 0;
    }
    /* Restore access state */
    PWR->CR1.w = ulCR1;
}

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_MCO_H_ */
