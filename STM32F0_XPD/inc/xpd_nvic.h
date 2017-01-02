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

/** @defgroup NVIC_Exported_Macros NVIC Exported Macros
 * @{ */

#define XPD_NVIC_SetPriorityGroup(PRIOGROUP)                                \
    ((void)0)

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
    NVIC_SetPriority((IRQN), (SUB_PRIO))

/**
 * @brief System reset redirection macro.
 */
#define         XPD_SystemReset()                                           \
    NVIC_SystemReset()

/** @} */

/** @} */

#endif /* XPD_NVIC_H_ */
