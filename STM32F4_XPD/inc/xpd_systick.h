/**
  ******************************************************************************
  * @file    xpd_systick.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-02-14
  * @brief   STM32 eXtensible Peripheral Drivers SysTick Module
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
#ifndef XPD_SYSTICK_H_
#define XPD_SYSTICK_H_

#include "xpd_common.h"
#include "xpd_config.h"

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

/** @addtogroup SysTick_Exported_Functions
 * @{ */

XPD_ReturnType  XPD_SysTick_Init        (uint32_t Period, SysTick_ClockSourceType ClockSource);

void            XPD_SysTick_Enable      (void);
void            XPD_SysTick_Disable     (void);

void            XPD_SysTick_EnableIT    (void);
void            XPD_SysTick_DisableIT   (void);

void            XPD_SysTick_Start_IT    (void);
void            XPD_SysTick_Stop_IT     (void);
/** @} */

/** @} */

#endif /* XPD_SYSTICK_H_ */
