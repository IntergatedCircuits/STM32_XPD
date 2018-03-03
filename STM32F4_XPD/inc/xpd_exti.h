/**
  ******************************************************************************
  * @file    xpd_exti.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers EXTI Module
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
#ifndef __XPD_EXTI_H_
#define __XPD_EXTI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup EXTI
 * @{ */

/** @defgroup EXTI_Exported_Types EXTI Exported Types
 * @{ */

/** @brief EXTI setup structure */
typedef struct
{
    EdgeType        Edge;       /*!< The selected edges trigger a reaction */
    ReactionType    Reaction;   /*!< Type of generated reaction for the detected edge */
}EXTI_InitType;

/** @} */

/** @defgroup EXTI_Exported_Variables EXTI Exported Variables
 * @{ */

/** @brief EXTI GPIO Pin callbacks container array */
extern XPD_ValueCallbackType EXTI_xPinCallbacks[16];

/** @} */

/** @addtogroup EXTI_Exported_Functions
 * @{ */
void            EXTI_vInit           (uint8_t ucLine, const EXTI_InitType * pxConfig);
void            EXTI_vDeinit         (uint8_t ucLine);

/**
 * @brief Gets the pending flag for the line.
 * @param ucLine: the selected EXTI line
 * @return The EXTI line flag status
 */
__STATIC_INLINE FlagStatus EXTI_eGetFlag(uint8_t ucLine)
{
#ifdef EXTI_BB
    return EXTI_BB->PR[ucLine];
#else
    return (EXTI->PR >> ucLine) & 1;
#endif
}

/**
 * @brief Clears the pending flag for the line.
 * @param ucLine: the selected EXTI line to clear
 */
__STATIC_INLINE void EXTI_vClearFlag(uint8_t ucLine)
{
    EXTI->PR = 1 << ucLine;
}

/**
 * @brief Generates a software triggered interrupt.
 * @param ucLine: the selected EXTI line to trigger
 */
__STATIC_INLINE void EXTI_vGenerateIT(uint8_t ucLine)
{
    EXTI->SWIER = 1 << ucLine;
}

/**
 * @brief EXTI interrupt handler.
 * @param ucLine: an interrupt line which may be responsible for the interrupt generation.
 */
__STATIC_INLINE void EXTI_vIRQHandler(uint8_t ucLine)
{
    if (EXTI_eGetFlag(ucLine))
    {
        EXTI_vClearFlag(ucLine);

        /* GPIO callbacks only */
        if (ucLine < 16)
        {
            XPD_SAFE_CALLBACK(EXTI_xPinCallbacks[ucLine], ucLine);
        }
    }
}

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_EXTI_H_ */
