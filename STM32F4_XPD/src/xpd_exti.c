/**
  ******************************************************************************
  * @file    xpd_exti.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-17
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

#include "xpd_exti.h"

XPD_ValueCallbackType XPD_EXTI_Callbacks[32] = {
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
};

/** @addtogroup EXTI
 * @{ */

/** @defgroup EXTI_Exported_Functions EXTI Exported Functions
 * @{ */

/**
 * @brief Configures the EXTI line according to the setup parameters.
 * @param Line: the selected EXTI line
 * @param Config: pointer to the setup structure
 */
void XPD_EXTI_Init(uint8_t Line, EXTI_InitType * Config)
{
#ifdef EXTI_BB
    EXTI_BB->IMR[Line] = Config->Reaction;

    EXTI_BB->EMR[Line] = Config->Reaction >> 1;

    EXTI_BB->RTSR[Line] = Config->Edge;

    EXTI_BB->FTSR[Line] = Config->Edge >> 1;
#else
    uint32_t linebit = 1 << Line;

    /* set EXTI line configuration */
    if (Config->Reaction & REACTION_IT)
    {
        SET_BIT(EXTI->IMR, linebit);
    }
    else
    {
        CLEAR_BIT(EXTI->IMR, linebit);
    }

    if (Config->Reaction & REACTION_EVENT)
    {
        SET_BIT(EXTI->EMR, linebit);
    }
    else
    {
        CLEAR_BIT(EXTI->EMR, linebit);
    }

    /* set rising and falling edge configuration */
    if (Config->Edge & EDGE_RISING)
    {
        SET_BIT(EXTI->RTSR, linebit);
    }
    else
    {
        CLEAR_BIT(EXTI->RTSR, linebit);
    }

    if (Config->Edge & EDGE_FALLING)
    {
        SET_BIT(EXTI->FTSR, linebit);
    }
    else
    {
        CLEAR_BIT(EXTI->FTSR, linebit);
    }
#endif

    if (Config->Reaction & REACTION_IT)
    {
        XPD_EXTI_Callbacks[Line] = Config->ITCallback;
    }
    else
    {
        XPD_EXTI_Callbacks[Line] = NULL;
    }
}

/**
 * @brief Gets the pending flag for the line.
 * @param Line: the selected EXTI line
 * @return The EXTI line flag status
 */
FlagStatus XPD_EXTI_GetFlag(uint8_t Line)
{
#ifdef EXTI_BB
    return EXTI_BB->PR[Line];
#else
    return (EXTI->PR >> (uint32_t)Line) & 1;
#endif
}

/**
 * @brief Clears the pending flag for the line.
 * @param Line: the selected EXTI line to clear
 */
void XPD_EXTI_ClearFlag(uint8_t Line)
{
#ifdef EXTI_BB
    EXTI_BB->PR[Line] = 1;
#else
    SET_BIT(EXTI->PR, 1 << (uint32_t)Line);
#endif
}

/**
 * @brief Generates a software triggered interrupt.
 * @param Line: the selected EXTI line to trigger
 */
void XPD_EXTI_GenerateIT(uint8_t Line)
{
#ifdef EXTI_BB
    EXTI_BB->SWIER[Line] = 1;
#else
    SET_BIT(EXTI->SWIER, 1 << (uint32_t)Line);
#endif
}

/**
 * @brief EXTI interrupt handler.
 * @param Line: an interrupt line which may be responsible for the interrupt generation.
 */
void XPD_EXTI_IRQHandler(uint8_t Line)
{
    if (XPD_EXTI_GetFlag(Line))
    {
        XPD_EXTI_ClearFlag(Line);

        XPD_SAFE_CALLBACK(XPD_EXTI_Callbacks[Line], Line);
    }
}

/** @} */

/** @} */
