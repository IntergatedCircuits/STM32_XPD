/**
  ******************************************************************************
  * @file    xpd_nvic.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers NVIC Module
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
#ifndef __XPD_NVIC_H_
#define __XPD_NVIC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup NVIC
 * @{ */

#if (__CORTEX_M >= 3)
/** @defgroup NVIC_Exported_Types NVIC Exported Types
 * @{ */

/** @brief NVIC priority group types used for NVIC_SetPriorityGrouping() */
typedef enum
{
    NVIC_PRIOGROUP_0PRE_4SUB = 7, /*!< 0 preemption priority bits, 4 subpriority bits */
    NVIC_PRIOGROUP_1PRE_3SUB = 6, /*!< 1 preemption priority bits, 3 subpriority bits */
    NVIC_PRIOGROUP_2PRE_2SUB = 5, /*!< 2 preemption priority bits, 2 subpriority bits */
    NVIC_PRIOGROUP_3PRE_1SUB = 4, /*!< 3 preemption priority bits, 1 subpriority bits */
    NVIC_PRIOGROUP_4PRE_0SUB = 3  /*!< 4 preemption priority bits, 0 subpriority bits */
}NVIC_PrioGroupType;

/** @} */

#endif /* (__CORTEX_M >= 3) */

/** @defgroup NVIC_Exported_Macros NVIC Exported Macros
 * @{ */

/**
 * @brief  @ref XPD_NVIC_GetCurrentIRQ() return value when it's called in thread context.
 */
#define ThreadMode_IRQn      (-16)

/**
 * @brief  Returns the currently active interrupt line.
 * @retval The @ref IRQn_Type that is currently being executed
 */
__STATIC_INLINE IRQn_Type NVIC_GetCurrentIRQ(void)
{
    return ((IRQn_Type)(((IPSR_Type)__get_IPSR()).b.ISR) - 16);
}

/**
 * @brief  Enable all exceptions with configurable priority (default).
 */
__STATIC_INLINE void NVIC_EnableAllIRQs(void)
{
    __set_PRIMASK(0);
}

/**
 * @brief  Disable all exceptions with configurable priority.
 */
__STATIC_INLINE void NVIC_DisableAllIRQs(void)
{
    __set_PRIMASK(1);
}

#if (__CORTEX_M >= 3)

/**
 * @brief  NVIC interrupt priority configuration setting macro.
 * @param  IRQn: the selected @ref IRQn_Type line to configure
 * @param  PreemptPriority: the preemption priority value
 * @param  SubPriority: the subpriority value
 */
__STATIC_INLINE void NVIC_SetPriorityConfig(IRQn_Type IRQn,
        uint32_t PreemptPriority,
        uint32_t SubPriority)
{
    NVIC_SetPriority(IRQn,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(),
                    PreemptPriority, SubPriority));
}

#else

/**
 * @brief  NVIC interrupt priority configuration setting macro.
 * @param  IRQn: the selected @ref IRQn_Type line to configure
 * @param  PreemptPriority: unused
 * @param  SubPriority: the priority value
 */
__STATIC_INLINE void NVIC_SetPriorityConfig(IRQn_Type IRQn,
        uint32_t PreemptPriority,
        uint32_t SubPriority)
{
    NVIC_SetPriority(IRQn, SubPriority);
}

#endif /* (__CORTEX_M >= 3) */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_NVIC_H_ */
