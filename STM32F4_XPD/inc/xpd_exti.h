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
#ifndef XPD_EXTI_H_
#define XPD_EXTI_H_

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

/** @addtogroup EXTI_Exported_Functions
 * @{ */
void            XPD_EXTI_Init           (uint8_t Line, EXTI_InitType * Config);

FlagStatus      XPD_EXTI_GetFlag        (uint8_t Line);
void            XPD_EXTI_ClearFlag      (uint8_t Line);

void            XPD_EXTI_GenerateIT     (uint8_t Line);

void            XPD_EXTI_IRQHandler     (uint8_t Line);
/** @} */

/** @defgroup EXTI_Exported_Variables EXTI Exported Variables
 * @{ */

/** @brief EXTI callbacks container array */
extern XPD_ValueCallbackType XPD_EXTI_Callbacks[32];

/** @} */

/** @} */

#endif /* XPD_EXTI_H_ */
