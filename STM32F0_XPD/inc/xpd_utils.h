/**
  ******************************************************************************
  * @file    xpd_utils.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Utilities
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
#ifndef __XPD_UTILS_H_
#define __XPD_UTILS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup XPD_Utils XPD Utilities
 * @{ */

/** @defgroup XPD_Exported_Macros XPD Exported Macros
 * @{ */

#ifndef XPD_ENTER_CRITICAL
/**
 * @brief Enters a critical section by disabling interrupts.
 * @param HANDLE: pointer to the requester handle
 */
#define XPD_ENTER_CRITICAL(HANDLE)
#endif

#ifndef XPD_EXIT_CRITICAL
/**
 * @brief Leaves a critical section by enabling interrupts.
 * @param HANDLE: pointer to the requester handle
 */
#define XPD_EXIT_CRITICAL(HANDLE)
#endif

/** @} */

/** @addtogroup XPD_Exported_Functions
 * @{ */

/** @addtogroup XPD_Exported_Functions_Timer
 * @{ */
void            XPD_vInitTimer          (void);
void            XPD_vDelay_ms           (uint32_t ulMilliseconds);
void            XPD_vDelay_us           (uint32_t ulMicroseconds);
XPD_ReturnType  XPD_eWaitForMatch       (volatile uint32_t * pulVarAddress, uint32_t ulBitSelector,
                                         uint32_t            ulMatch,       uint32_t * pulTimeout);
XPD_ReturnType  XPD_eWaitForDiff        (volatile uint32_t * pulVarAddress, uint32_t ulBitSelector,
                                         uint32_t            ulMatch,       uint32_t * pulTimeout);
/** @} */

/** @addtogroup XPD_Exported_Functions_Stream
 * @{ */
void            XPD_vReadToStream       (const uint32_t * pulReg, DataStreamType * pxStream);
void            XPD_vWriteFromStream    (uint32_t * pulReg, DataStreamType * pxStream);
/** @} */

/** @addtogroup XPD_Exported_Functions_Init
 * @{ */
void            XPD_vInit               (void);
void            XPD_vDeinit             (void);
void            XPD_vBootTo             (void * pvStartAddress);
/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_UTILS_H_ */
