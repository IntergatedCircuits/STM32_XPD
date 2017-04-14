/**
  ******************************************************************************
  * @file    xpd_utils.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-17
  * @brief   STM32 eXtensible Peripheral Drivers Utilities Module
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
#ifndef XPD_UTILS_H_
#define XPD_UTILS_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup XPD_Utils XPD Utilities
 * @{ */

/** @defgroup XPD_Utils_Exported_Types XPD Exported Types
 * @{ */

/** @brief XPD utilities callbacks structure */
typedef struct {
    XPD_SimpleCallbackType Tick; /*!< SysTick interrupt callback */
}XPD_CallbacksType;

/** @} */

/** @defgroup XPD_Exported_Variables XPD Exported Variables
 * @{ */

/** @brief XPD utilities callbacks container */
extern XPD_CallbacksType XPD_Callbacks;

/** @} */

/** @defgroup XPD_Exported_Macros XPD Exported Macros
 * @{ */

#ifndef XPD_ENTER_CRITICAL
/**
 * @brief Enters a critical section by disabling interrupts. [overrideable]
 * @param HANDLE: pointer to the requester handle
 */
#define XPD_ENTER_CRITICAL(HANDLE)
#endif

#ifndef XPD_EXIT_CRITICAL
/**
 * @brief Leaves a critical section by enabling interrupts. [overrideable]
 * @param HANDLE: pointer to the requester handle
 */
#define XPD_EXIT_CRITICAL(HANDLE)
#endif

/** @brief Timeout value for indefinite waiting */
#define XPD_NO_TIMEOUT      0xFFFFFFFF

/** @} */

/** @addtogroup XPD_Exported_Functions
 * @{ */

/** @addtogroup XPD_Exported_Functions_Init
 * @{ */
void            XPD_Init                (void);
void            XPD_Deinit              (void);
/** @} */
/** @addtogroup XPD_Exported_Functions_Timer
 * @{ */
void            XPD_InitTimer           (void);
void            XPD_IncTimer            (void);
uint32_t        XPD_GetTimer            (void);
void            XPD_SuspendTimer        (void);
void            XPD_ResumeTimer         (void);
void            XPD_Delay_ms            (uint32_t milliseconds);
void            XPD_Delay_us            (uint32_t microseconds);
XPD_ReturnType  XPD_WaitForMatch        (volatile uint32_t * varAddress, uint32_t bitSelector,
                                         uint32_t            match,      uint32_t * mstimeout);
XPD_ReturnType  XPD_WaitForDiff         (volatile uint32_t * varAddress, uint32_t bitSelector,
                                         uint32_t            match,      uint32_t * mstimeout);
/** @} */
/** @addtogroup XPD_Exported_Functions_Stream
 * @{ */
void            XPD_ReadToStream        (volatile uint32_t * reg, DataStreamType * stream);
void            XPD_WriteFromStream     (volatile uint32_t * reg, DataStreamType * stream);
/** @} */
/** @addtogroup XPD_Exported_Functions_IRQ
 * @{ */
void            XPD_SysTick_IRQHandler  (void);
/** @} */

/** @} */

/** @} */

#endif /* XPD_UTILS_H_ */
