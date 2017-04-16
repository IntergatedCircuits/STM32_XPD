/**
  ******************************************************************************
  * @file    xpd_exti.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-17
  * @brief   STM32 eXtensible Peripheral Drivers EXTI Module
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
#ifndef __XPD_EXTI_H_
#define __XPD_EXTI_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup EXTI
 * @{ */

/** @defgroup EXTI_Exported_Types EXTI Exported Types
 * @{ */

/** @brief EXTI setup structure */
typedef struct
{
    XPD_ValueCallbackType ITCallback; /*!< Callback for the interrupt line, the passed parameter is the line number */
    EdgeType              Edge;       /*!< The selected edges trigger a reaction */
    ReactionType          Reaction;   /*!< Type of generated reaction for the detected edge */
}EXTI_InitType;

/** @} */

/** @defgroup EXTI_Exported_Variables EXTI Exported Variables
 * @{ */

/** @brief EXTI callbacks container array */
extern XPD_ValueCallbackType XPD_EXTI_Callbacks[];

/** @} */

/** @addtogroup EXTI_Exported_Functions
 * @{ */
void            XPD_EXTI_Init           (uint8_t Line, const EXTI_InitType * Config);
void            XPD_EXTI_Deinit         (uint8_t Line);

/**
 * @brief Gets the pending flag for the line.
 * @param Line: the selected EXTI line
 * @return The EXTI line flag status
 */
__STATIC_INLINE FlagStatus XPD_EXTI_GetFlag(uint8_t Line)
{
#ifdef EXTI_BB
    if (Line < 32)
        return EXTI_BB->PR[Line];
    else
        return EXTI_BB->PR2[Line - 32];
#else
    if (Line < 32)
        return (EXTI->PR >> (uint32_t)Line) & 1;
    else
        return (EXTI->PR2 >> ((uint32_t)Line - 32)) & 1;
#endif
}

/**
 * @brief Clears the pending flag for the line.
 * @param Line: the selected EXTI line to clear
 */
__STATIC_INLINE void XPD_EXTI_ClearFlag(uint8_t Line)
{
#ifdef EXTI_BB
    if (Line < 32)
        EXTI_BB->PR[Line] = 1;
    else
        EXTI_BB->PR2[Line - 32] = 1;
#else
    if (Line < 32)
        EXTI->PR = 1 << (uint32_t)Line;
    else
        EXTI->PR2 = 1 << ((uint32_t)Line - 32);
#endif
}

/**
 * @brief Generates a software triggered interrupt.
 * @param Line: the selected EXTI line to trigger
 */
__STATIC_INLINE void XPD_EXTI_GenerateIT(uint8_t Line)
{
#ifdef EXTI_BB
    if (Line < 32)
        EXTI_BB->SWIER[Line] = 1;
    else
        EXTI_BB->SWIER2[Line - 32] = 1;
#else
    if (Line < 32)
        SET_BIT(EXTI->SWIER, 1 << (uint32_t)Line);
    else
        SET_BIT(EXTI->SWIER2, 1 << ((uint32_t)Line - 32));
#endif
}

/**
 * @brief EXTI interrupt handler.
 * @param Line: an interrupt line which may be responsible for the interrupt generation.
 */
__STATIC_INLINE void XPD_EXTI_IRQHandler(uint8_t Line)
{
    if (XPD_EXTI_GetFlag(Line))
    {
        XPD_EXTI_ClearFlag(Line);

        XPD_SAFE_CALLBACK(XPD_EXTI_Callbacks[Line], Line);
    }
}

/** @} */

/** @} */

#endif /* __XPD_EXTI_H_ */
