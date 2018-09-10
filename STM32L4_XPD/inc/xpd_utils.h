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

/** @defgroup XPD_Exported_Types XPD Exported Types
 * @{ */

/** @brief Time service functions structure */
typedef struct
{
    void            (*Init)         (uint32_t ulCoreFreq_Hz);
    /*!< Prepares the time service for operation */

    void            (*Block_ms)     (uint32_t ulBlocktime_ms);
    /*!< Blocks the thread for the specified time */

    XPD_ReturnType  (*MatchBlock_ms)(volatile uint32_t* pulVarAddress,
                                     uint32_t ulBitSelector,
                                     uint32_t ulMatch,
                                     uint32_t * pulTimeout);
    /*!< Blocks until the masked value read from address matches the input ulMatch, or until times out. */

    XPD_ReturnType  (*DiffBlock_ms) (volatile uint32_t* pulVarAddress,
                                     uint32_t ulBitSelector,
                                     uint32_t ulMatch,
                                     uint32_t * pulTimeout);
    /*!< Blocks until the masked value read from address differs from the input ulMatch, or until times out. */

}XPD_TimeServiceType;

/** @} */

/** @defgroup XPD_Exported_Macros XPD Exported Macros
 * @{ */

#ifndef XPD_ENTER_CRITICAL
/**
 * @brief Enters a critical section by disabling interrupts.
 * @param HANDLE: pointer to the requester handle
 */
#define         XPD_ENTER_CRITICAL(HANDLE)
#endif

#ifndef XPD_EXIT_CRITICAL
/**
 * @brief Leaves a critical section by enabling interrupts.
 * @param HANDLE: pointer to the requester handle
 */
#define         XPD_EXIT_CRITICAL(HANDLE)
#endif

/** @} */

/** @addtogroup XPD_Exported_Functions
 * @{ */

/** @addtogroup XPD_Exported_Functions_Timer
 * @{ */
const XPD_TimeServiceType* XPD_pxTimeService(void);
void            XPD_vSetTimeService     (const XPD_TimeServiceType* pxTimeService);
void            XPD_vResetTimeService   (void);

void            XPD_vDelay_us           (uint32_t ulMicroseconds);

/**
 * @brief Initialize the timer of the XPD time service.
 * @param ulCoreFreq_Hz: the new core frequency in Hz
 */
__STATIC_INLINE void XPD_vInitTimer(uint32_t ulCoreFreq_Hz)
{
    XPD_pxTimeService()->Init(ulCoreFreq_Hz);
}

/**
 * @brief Insert a delay of a specified time in the execution.
 * @param ulBlocktime_ms: the amount of delay in ms
 */
__STATIC_INLINE void XPD_vDelay_ms(uint32_t ulBlocktime_ms)
{
    XPD_pxTimeService()->Block_ms(ulBlocktime_ms);
}

/**
 * @brief Waits until the masked value at address matches the input, or until timeout.
 * @param pulVarAddress: the word address that needs to be monitored
 * @param ulBitSelector: a bit mask that selects which bits should be considered
 * @param ulMatch: the expected value to wait for
 * @param pulTimeout: pointer to the timeout in ms
 * @return TIMEOUT if timed out, or OK if ulMatch occurred within the deadline
 */
__STATIC_INLINE XPD_ReturnType XPD_eWaitForMatch(
        volatile uint32_t * pulVarAddress,
        uint32_t            ulBitSelector,
        uint32_t            ulMatch,
        uint32_t *          pulTimeout)
{
    return XPD_pxTimeService()->MatchBlock_ms(pulVarAddress, ulBitSelector,
            ulMatch, pulTimeout);
}

/**
 * @brief Waits until the masked value at address differs from the input, or until timeout.
 * @param pulVarAddress: the word address that needs to be monitored
 * @param ulBitSelector: a bit mask that selects which bits should be considered
 * @param ulMatch: the expected value to wait for
 * @param pulTimeout: pointer to the timeout in ms
 * @return TIMEOUT if timed out, or OK if ulMatch occurred within the deadline
 */
__STATIC_INLINE XPD_ReturnType XPD_eWaitForDiff(
        volatile uint32_t * pulVarAddress,
        uint32_t            ulBitSelector,
        uint32_t            ulMatch,
        uint32_t *          pulTimeout)
{
    return XPD_pxTimeService()->DiffBlock_ms(pulVarAddress, ulBitSelector,
            ulMatch, pulTimeout);
}

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
