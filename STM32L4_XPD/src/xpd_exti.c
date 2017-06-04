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

XPD_ValueCallbackType XPD_EXTI_Callbacks[] = {
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
void XPD_EXTI_Init(uint8_t Line, const EXTI_InitType * Config)
{
    /* GPIO callbacks only */
    if (Line < 16)
    {
        if (Config->Reaction & REACTION_IT)
        {
            XPD_EXTI_Callbacks[Line] = Config->ITCallback;
        }
        else
        {
            XPD_EXTI_Callbacks[Line] = NULL;
        }
    }

#ifdef EXTI_BB
    if (Line < 32)
    {
        EXTI_BB->IMR1[Line] = Config->Reaction;

        EXTI_BB->EMR1[Line] = Config->Reaction >> 1;

        EXTI_BB->RTSR1[Line] = Config->Edge;

        EXTI_BB->FTSR1[Line] = Config->Edge >> 1;
    }
    else
    {
        Line -= 32;

        EXTI_BB->IMR2[Line] = Config->Reaction;

        EXTI_BB->EMR2[Line] = Config->Reaction >> 1;

        EXTI_BB->RTSR2[Line] = Config->Edge;

        EXTI_BB->FTSR2[Line] = Config->Edge >> 1;
    }
#else
    if (Line < 32)
    {
        uint32_t linebit = 1 << Line;

        /* set EXTI line configuration */
        if (Config->Reaction & REACTION_IT)
        {
            SET_BIT(EXTI->IMR1, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->IMR1, linebit);
        }

        if (Config->Reaction & REACTION_EVENT)
        {
            SET_BIT(EXTI->EMR1, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->EMR1, linebit);
        }

        /* set rising and falling edge configuration */
        if (Config->Edge & EDGE_RISING)
        {
            SET_BIT(EXTI->RTSR1, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->RTSR1, linebit);
        }

        if (Config->Edge & EDGE_FALLING)
        {
            SET_BIT(EXTI->FTSR1, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->FTSR1, linebit);
        }
    }
    else
    {
        uint32_t linebit = 1 << (Line - 32);

        /* set EXTI line configuration */
        if (Config->Reaction & REACTION_IT)
        {
            SET_BIT(EXTI->IMR2, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->IMR2, linebit);
        }

        if (Config->Reaction & REACTION_EVENT)
        {
            SET_BIT(EXTI->EMR2, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->EMR2, linebit);
        }

        /* set rising and falling edge configuration */
        if (Config->Edge & EDGE_RISING)
        {
            SET_BIT(EXTI->RTSR2, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->RTSR2, linebit);
        }

        if (Config->Edge & EDGE_FALLING)
        {
            SET_BIT(EXTI->FTSR2, linebit);
        }
        else
        {
            CLEAR_BIT(EXTI->FTSR2, linebit);
        }
    }
#endif
}

/**
 * @brief Restores the EXTI line to its default state.
 * @param Line: the selected EXTI line
 */
void XPD_EXTI_Deinit(uint8_t Line)
{
    XPD_EXTI_Callbacks[Line] = NULL;

#ifdef EXTI_BB
    if (Line < 32)
    {
        /* Clear EXTI line configuration */
        EXTI_BB->IMR1[Line] = 0;
        EXTI_BB->EMR1[Line] = 0;

        /* Clear Rising Falling edge configuration */
        EXTI_BB->RTSR1[Line] = 0;
        EXTI_BB->FTSR1[Line] = 0;
    }
    else
    {
        Line -= 32;

        /* Clear EXTI line configuration */
        EXTI_BB->IMR2[Line] = 0;
        EXTI_BB->EMR2[Line] = 0;

        /* Clear Rising Falling edge configuration */
        EXTI_BB->RTSR2[Line] = 0;
        EXTI_BB->FTSR2[Line] = 0;
    }
#else
    if (Line < 32)
    {
        uint32_t linebit = 1 << Line;

        /* Clear EXTI line configuration */
        CLEAR_BIT(EXTI->IMR1, linebit);
        CLEAR_BIT(EXTI->EMR1, linebit);

        /* Clear Rising Falling edge configuration */
        CLEAR_BIT(EXTI->RTSR1, linebit);
        CLEAR_BIT(EXTI->FTSR1, linebit);
    }
    else
    {
        uint32_t linebit = 1 << (Line - 32);

        /* Clear EXTI line configuration */
        CLEAR_BIT(EXTI->IMR2, linebit);
        CLEAR_BIT(EXTI->EMR2, linebit);

        /* Clear Rising Falling edge configuration */
        CLEAR_BIT(EXTI->RTSR2, linebit);
        CLEAR_BIT(EXTI->FTSR2, linebit);
    }
#endif
}

/** @} */

/** @} */
