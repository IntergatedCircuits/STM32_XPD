/**
  ******************************************************************************
  * @file    xpd_rcc_pc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-16
  * @brief   STM32 eXtensible Peripheral Drivers RCC Peripheral Clocks Module
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
#ifndef XPD_RCC_PC_H_
#define XPD_RCC_PC_H_

#include "xpd_common.h"
#include "xpd_config.h"

#if defined(USE_XPD_RTC)
/** @addtogroup RTC
 * @{ */

/** @defgroup RTC_Clock_Source RTC Clock Source
 * @{ */

/** @defgroup RTC_Clock_Source_Exported_Types RTC Clock Source Exported Types
 * @{ */

/** @brief RTC clock source types */
typedef enum
{
    RTC_CLOCKSOURCE_NOCLOCK = 0, /*!< no clock source */
    RTC_CLOCKSOURCE_LSE     = 1, /*!< LSE clock source */
    RTC_CLOCKSOURCE_LSI     = 2, /*!< LSI clock source */
    RTC_CLOCKSOURCE_HSE     = 3, /*!< HSE clock source */
}RTC_ClockSourceType;

/** @defgroup RTC_Clock_Source_Exported_Functions RTC Clock Source Exported Functions
 * @{ */
XPD_ReturnType  XPD_RTC_ClockConfig         (RTC_ClockSourceType ClockSource);
uint32_t        XPD_RTC_GetClockFreq        (void);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_RTC */

#if defined(USE_XPD_TIM)
#include "xpd_tim.h"

/** @addtogroup TIM
 * @{ */

/** @defgroup TIM_Clock_Source TIM Clock Source
 * @{ */

#ifdef RCC_DCKCFGR_TIMPRE
/** @defgroup TIM_Clock_Source_Exported_Types TIM Clock Source Exported Types
 * @{ */

/** @brief TIM clock source types */
typedef enum
{
    TIM_CLOCKSOURCE_PCLKx = 0, /*!< default clock source */
    TIM_CLOCKSOURCE_HCLK  = 1, /*!< clock source from HCLK
                                    @note Only applies if APBx prescaler is DIV1, 2 or 4 */
}TIM_ClockSourceType;

/** @} */
#endif

/** @defgroup TIM_Clock_Source_Exported_Functions TIM Clock Source Exported Functions
 * @{ */
#ifdef RCC_DCKCFGR_TIMPRE
void            XPD_TIM_ClockConfig         (TIM_ClockSourceType ClockSource);
#endif
uint32_t        XPD_TIM_GetClockFreq        (TIM_HandleType * htim);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_TIM */

#endif /* XPD_RCC_PC_H_ */
