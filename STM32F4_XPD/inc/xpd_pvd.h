/**
  ******************************************************************************
  * @file    xpd_pvd.h
  * @author  Benedek Kupper
  * @version 0.1
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Power Voltage Detector Module
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
#ifndef __XPD_PVD_H_
#define __XPD_PVD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_exti.h>
#include <xpd_pwr.h>

#ifdef PWR_CR_PLS

/** @defgroup PVD Power Voltage Detector
 * @{ */

/** @defgroup PVD_Exported_Types PVD Exported Types
 * @{ */

/** @brief PVD levels */
typedef enum
{
    PWR_PVDLEVEL_2V0 = 0, /*!< 2.0V voltage detector level */
    PWR_PVDLEVEL_2V1 = 1, /*!< 2.1V voltage detector level */
    PWR_PVDLEVEL_2V3 = 2, /*!< 2.3V voltage detector level */
    PWR_PVDLEVEL_2V5 = 3, /*!< 2.5V voltage detector level */
    PWR_PVDLEVEL_2V6 = 4, /*!< 2.6V voltage detector level */
    PWR_PVDLEVEL_2V7 = 5, /*!< 2.7V voltage detector level */
    PWR_PVDLEVEL_2V8 = 6, /*!< 2.8V voltage detector level */
    PWR_PVDLEVEL_2V9 = 7  /*!< 2.9V voltage detector level */
} PWR_PVDLevelType;

/** @} */

/** @defgroup PVD_Exported_Macros PVD Exported Macros
 * @{ */

/** @brief PVD EXTI line number */
#define PVD_EXTI_LINE                   16
/** @} */

/** @defgroup PVD_Exported_Functions PVD Exported Functions
 * @{ */

/**
 * @brief Configures the voltage threshold monitoring by the Power Voltage Detector(PVD).
 * @param eLevel: the monitored voltage level
 * @param eReaction: system reaction upon crossing threshold
 */
__STATIC_INLINE void PVD_vInit(PWR_PVDLevelType eLevel, ReactionType eReaction)
{
    EXTI_InitType xExti = { .Edge = EDGE_RISING };
    xExti.Reaction = eReaction;

    /* Set PLS bits according to PVDLevel value */
    PWR->CR.b.PLS = eLevel;

    /* External interrupt line 16 Connected to the PVD EXTI Line */
    EXTI_vInit(PVD_EXTI_LINE, &xExti);
}

/**
 * @brief Enables the Power Voltage Detector (PVD).
 */
__STATIC_INLINE void PVD_vStart(void)
{
    PWR_REG_BIT(CR,PVDE) = 1;
}

/**
 * @brief Disables the Power Voltage Detector (PVD).
 */
__STATIC_INLINE void PVD_vStop(void)
{
    PWR_REG_BIT(CR,PVDE) = 0;
}

/**
 * @brief Gets the pending flag for the PVD.
 * @return The PVD flag status
 */
__STATIC_INLINE FlagStatus PVD_eGetFlag(void)
{
    return EXTI_eGetFlag(PVD_EXTI_LINE);
}

/**
 * @brief Clears the pending PVD flag.
 */
__STATIC_INLINE void PVD_vClearFlag(void)
{
    EXTI_vClearFlag(PVD_EXTI_LINE);
}

/** @} */

/** @} */

#endif /* PWR_CR_PLS */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_PVD_H_ */
