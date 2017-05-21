/**
  ******************************************************************************
  * @file    xpd_utils.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-04-26
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
#ifndef __XPD_UTILS_H_
#define __XPD_UTILS_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup XPD_Utils XPD Utilities
 * @{ */

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

/** @addtogroup XPD_Exported_Functions_Boot
 * @{ */
void            XPD_BootTo              (const void * StartAddress);
/** @} */

/** @} */

/** @} */

#endif /* __XPD_UTILS_H_ */
