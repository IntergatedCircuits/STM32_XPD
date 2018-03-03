/**
  ******************************************************************************
  * @file    xpd_systick.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers SysTick Module
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
#ifndef __XPD_SYSTICK_H_
#define __XPD_SYSTICK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup SysTick
 * @{ */

/** @defgroup SysTick_Exported_Types SysTick Exported Types
 * @{ */

/** @brief SysTick clock source types */
typedef enum
{
    SYSTICK_CLOCKSOURCE_HCLK_DIV8 = 0, /*!< SysTick clock source is HCLK divided by 8 */
    SYSTICK_CLOCKSOURCE_HCLK      = 1  /*!< SysTick clock source is HCLK */
}SysTick_ClockSourceType;

/** @} */

/** @defgroup SysTick_Exported_Functions SysTick Exported Functions
 * @{ */

/**
 * @brief Configures the SysTick timer and interrupt generation
 * @param ClockSource: clock source of the timer
 * @param Period: the amount of timer counts until the counter reset
 * @return ERROR if the Period is too large to fit in the 24 bit register,
 *         OK if successful
 */
__STATIC_INLINE XPD_ReturnType SysTick_eInit(
        SysTick_ClockSourceType eClockSource,
        uint32_t                ulPeriod)
{
    /* check against counter size (24 bits) */
    if ((--ulPeriod) > SysTick_LOAD_RELOAD_Msk)
    {
        return XPD_ERROR;
    }
    else
    {
        /* set reload register */
        SysTick->LOAD = ulPeriod;

        /* reset the SysTick Counter Value */
        SysTick->VAL  = 0;

        /* set clock source */
        SysTick->CTRL.b.CLKSOURCE = eClockSource;

        return XPD_OK;
    }
}

/**
 * @brief Enables the SysTick timer
 */
__STATIC_INLINE void SysTick_vStart(void)
{
    SysTick->CTRL.b.ENABLE = 1;
}

/**
 * @brief Disables the SysTick timer
 */
__STATIC_INLINE void SysTick_vStop(void)
{
    SysTick->CTRL.b.ENABLE = 0;
}

/**
 * @brief Starts the SysTick timer with interrupt generation
 */
__STATIC_INLINE void SysTick_vStart_IT(void)
{
    SysTick->VAL = 0;
    SET_BIT(SysTick->CTRL.w, SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

/**
 * @brief Stops the SysTick timer with interrupt generation
 */
__STATIC_INLINE void SysTick_vStop_IT(void)
{
    CLEAR_BIT(SysTick->CTRL.w, SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}

/**
 * @brief Enables the SysTick interrupt request
 */
__STATIC_INLINE void SysTick_IT_Enable(void)
{
    SysTick->CTRL.b.TICKINT = 1;
}

/**
 * @brief Disables the SysTick interrupt request
 */
__STATIC_INLINE void SysTick_IT_Disable(void)
{
    SysTick->CTRL.b.TICKINT = 0;
}

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_SYSTICK_H_ */
