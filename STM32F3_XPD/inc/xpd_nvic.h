/**
  ******************************************************************************
  * @file    xpd_nvic.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-02-14
  * @brief   STM32 eXtensible Peripheral Drivers NVIC Module
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
#ifndef XPD_NVIC_H_
#define XPD_NVIC_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup NVIC
 * @{ */

/** @defgroup NVIC_Exported_Types NVIC Exported Types
 * @{ */

/** @brief NVIC priority group types */
typedef enum
{
    NVIC_PRIOGROUP_0PRE_4SUB = 7, /*!< 0 preemption priority bits, 4 subpriority bits */
    NVIC_PRIOGROUP_1PRE_3SUB = 6, /*!< 1 preemption priority bits, 3 subpriority bits */
    NVIC_PRIOGROUP_2PRE_2SUB = 5, /*!< 2 preemption priority bits, 2 subpriority bits */
    NVIC_PRIOGROUP_3PRE_1SUB = 4, /*!< 3 preemption priority bits, 1 subpriority bits */
    NVIC_PRIOGROUP_4PRE_0SUB = 3  /*!< 4 preemption priority bits, 0 subpriority bits */
}NVIC_PrioGroupType;

/** @} */

/** @defgroup NVIC_Exported_Macros NVIC Exported Macros
 * @{ */

/**
 * @brief  NVIC interrupt priority group configuration setting redirection macro.
 * @param  PRIOGROUP: the selected @ref NVIC_PrioGroupType to set
 */
#define         XPD_NVIC_SetPriorityGroup(PRIOGROUP)                        \
    NVIC_SetPriorityGrouping((uint32_t)(PRIOGROUP))

/**
 * @brief  NVIC interrupt priority group configuration reading redirection macro.
 * @return The configured @ref NVIC_PrioGroupType
 */
#define         XPD_NVIC_GetPriorityGroup()                                 \
    ((NVIC_PrioGroupType)NVIC_GetPriorityGrouping())

/**
 * @brief  NVIC interrupt priority enable redirection macro.
 * @param  IRQN: the selected @ref IRQn_Type line to enable
 */
#define         XPD_NVIC_EnableIRQ(IRQN)                                    \
    NVIC_EnableIRQ(IRQN)

/**
 * @brief  NVIC interrupt priority disable redirection macro.
 * @param  IRQN: the selected @ref IRQn_Type line to disable
 */
#define         XPD_NVIC_DisableIRQ(IRQN)                                   \
    NVIC_EnableIRQ(IRQN)

/**
 * @brief  NVIC interrupt priority configuration setting macro.
 * @param  IRQN: the selected @ref IRQn_Type line to configure
 * @param  PREEMPT_PRIO: the preemption priority value
 * @param  SUB_PRIO: the subpriority value
 */
#define         XPD_NVIC_SetPriorityConfig(IRQN,PREEMPT_PRIO,SUB_PRIO)      \
    NVIC_SetPriority((IRQN), NVIC_EncodePriority(NVIC_GetPriorityGrouping(),(PREEMPT_PRIO),(SUB_PRIO)))

/**
 * @brief System reset redirection macro.
 */
#define         XPD_SystemReset()                                           \
    NVIC_SystemReset()

/** @} */

/** @} */

#endif /* XPD_NVIC_H_ */
