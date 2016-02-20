/**
  ******************************************************************************
  * @file    xpd_systick.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-02-14
  * @brief   STM32 eXtensible Peripheral Drivers TODO Module
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

#include "xpd_systick.h"

/** @defgroup SysTick
 * @{ */

/** @defgroup SysTick_Exported_Functions SysTick Exported Functions
 * @{ */

/**
 * @brief Configures the SysTick timer and interrupt generation
 * @param Period: the amount of timer counts until the counter reset
 * @param ClockSource: clock source of the timer
 * @return ERROR if the Period is too large to fit in the 24 bit register, OK if successful
 */
XPD_ReturnType XPD_SysTick_Init(uint32_t Period, SysTick_ClockSourceType ClockSource)
{
    /* check against counter size (24 bits) */
    if ((--Period) > SysTick_LOAD_RELOAD_Msk)
    {
        return XPD_ERROR;                        /* Reload value impossible */
    }
    else
    {
        SysTick->LOAD.w = Period;                /* set reload register */
        SysTick->VAL.w  = 0;                     /* reset the SysTick Counter Value */
        SysTick->CTRL.b.CLKSOURCE = ClockSource; /* set clock source */

        return XPD_OK;
    }
}

/**
 * @brief Enables the SysTick timer
 */
void XPD_SysTick_Enable(void)
{
    SysTick->CTRL.b.ENABLE = 1;
}

/**
 * @brief Disables the SysTick timer
 */
void XPD_SysTick_Disable(void)
{
    SysTick->CTRL.b.ENABLE = 0;
}

/**
 * @brief Enables the SysTick interrupt request
 */
void XPD_SysTick_EnableIT(void)
{
    SysTick->CTRL.b.TICKINT = 1;
}

/**
 * @brief Disables the SysTick interrupt request
 */
void XPD_SysTick_DisableIT(void)
{
    SysTick->CTRL.b.TICKINT = 0;
}

/**
 * @brief Starts the SysTick timer with interrupt generation
 */
void XPD_SysTick_Start_IT(void)
{
    SysTick->VAL.w  = 0;                     /* reset the SysTick Counter Value */
    SysTick->CTRL.b.TICKINT = 1;             /* enable interrupt generation */
    SysTick->CTRL.b.ENABLE = 1;              /* enable counter */
}

/**
 * @brief Stops the SysTick timer with interrupt generation
 */
void XPD_SysTick_Stop_IT(void)
{
    SysTick->CTRL.b.TICKINT = 0;
}

/** @} */

/** @} */
