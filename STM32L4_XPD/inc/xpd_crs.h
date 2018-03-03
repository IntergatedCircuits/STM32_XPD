/**
  ******************************************************************************
  * @file    xpd_crs.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Clock Recovery System
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
#ifndef __XPD_CRS_H_
#define __XPD_CRS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

#if defined(CRS)

/** @addtogroup RCC
 * @{ */

/** @defgroup CRS Clock Recovery System
 * @{ */

/** @defgroup CRS_Exported_Types CRS Exported Types
 * @{ */

/** @brief CRS synchronization source selection */
typedef enum
{
    CRS_SYNC_SOURCE_NONE     = 0,   /*!< No automatic synchronization */
    CRS_SYNC_SOURCE_GPIO     = 4,   /*!< Synchronization on GPIO input signal */
#ifdef LSE_VALUE_Hz
    CRS_SYNC_SOURCE_LSE      = 1,   /*!< Synchronization on LSE oscillator */
#endif
    CRS_SYNC_SOURCE_USB      = 2,   /*!< Synchronization on USB StartOfFrame signals */
    CRS_SYNC_SOURCE_SOFTWARE = 8,   /*!< Synchronization on software generated events */
}CRS_SyncSourceType;

/** @brief CRS synchronization status flags */
typedef enum
{
    CRS_SYNC_PENDING  = 0,                  /*!< No active synchronization events */
    CRS_SYNC_OK       = CRS_ISR_SYNCOKF,    /*!< |fError| = [0; FELIM * 3) */
    CRS_SYNC_WARNING  = CRS_ISR_SYNCWARNF,  /*!< |fError| = [FELIM * 3; FELIM * 128) */
    CRS_SYNC_MISS     = CRS_ISR_SYNCMISS,   /*!<  fError >=  FELIM * 128 */
    CRS_SYNC_ERROR    = CRS_ISR_SYNCERR,    /*!<  fError <= -FELIM * 128 */
    CRS_SYNC_EXPECTED = CRS_ISR_ESYNCF,     /*!< the frequency error counter reached a zero value */
    CRS_TRIMMING_OVF  = CRS_ISR_TRIMOVF,    /*!< the automatic trimming tries to over- or under-flow the TRIM value */
}CRS_StatusType;

/** @brief HSI48 setup structure */
typedef struct
{
    uint8_t            CalibVal;    /*!< HSI48 calibration value [0..63] (default is 32) */
    CRS_SyncSourceType Source;      /*!< SYNC signal source */
    ClockDividerType   Divider;     /*!< Division factor of the GPIO SYNC signal [DIV1..DIV128] */
    uint32_t           Frequency;   /*!< The GPIO synchronization source frequency in Hz */
    ActiveLevelType    Polarity;    /*!< The input polarity for the GPIO SYNC signal */
    uint16_t           ErrorLimit;  /*!< Boundary for SYNC events. Recommended calculation:
                                         FELIM = (fTARGET = 48MHz / fSYNC) * STEP[%] / 100% / 2
                                         Typical trimming step: 0.14% */
}CRS_InitType;

/** @brief CRS callbacks container structure */
typedef struct {
    XPD_SimpleCallbackType SyncSuccess;  /*!< SYNC OK flag requested interrupt */
    XPD_SimpleCallbackType SyncWarning;  /*!< SYNC WARN flag requested interrupt */
    XPD_SimpleCallbackType SyncExpected; /*!< ESYNC flag requested interrupt */
    XPD_SimpleCallbackType SyncError;    /*!< a SYNC error flag requested interrupt */
}CRS_CalllbacksType;

/** @} */

/** @defgroup CRS_Exported_Variables CRS Exported Variables
 * @{ */

/** @brief CRS callbacks container struct */
extern CRS_CalllbacksType CRS_xCallbacks;

/** @} */

/** @defgroup CRS_Exported_Macros CRS Exported Macros
 * @{ */

#ifdef CRS_BB
/**
 * @brief CRS register bit accessing macro
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         CRS_REG_BIT(REG_NAME, BIT_NAME)     \
    (CRS_BB->REG_NAME.BIT_NAME)
#else
/**
 * @brief CRS register bit accessing macro
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         CRS_REG_BIT(REG_NAME, BIT_NAME)     \
    (CRS->REG_NAME.b.BIT_NAME)
#endif

#define CRS_ISR_SYNCERRF        (CRS_ISR_SYNCERR)
#define CRS_ISR_SYNCMISSF       (CRS_ISR_SYNCMISS)
#define CRS_ERROR_FLAGS_MASK    (CRS_ISR_TRIMOVF | CRS_ISR_SYNCERR | CRS_ISR_SYNCMISS)

/** @brief Default Frequency error limit */
#define CRS_ERRORLIMIT_DEFAULT          0x22

/** @brief Default HSI48 calibration value (step is around 67 kHz) */
#define HSI48_CALIBRATION_DEFAULT_VALUE 0x20

/**
 * @brief  Enable the specified CRS interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg SYNCOK:   SYNC event OK
 *            @arg SYNCWARN: SYNC warning
 *            @arg ESYNC:    Expected SYNC event
 *            @arg ERR:      SYNC errors
 */
#define         CRS_IT_ENABLE(IT_NAME)              \
    (CRS_REG_BIT(CR,IT_NAME##IE) = 1)

/**
 * @brief  Disable the specified CRS interrupt.
 * @param  IT_NAME: specifies the interrupt to disable.
 *         This parameter can be one of the following values:
 *            @arg SYNCOK:   SYNC event OK
 *            @arg SYNCWARN: SYNC warning
 *            @arg ESYNC:    Expected SYNC event
 *            @arg ERR:      SYNC errors
 */
#define         CRS_IT_DISABLE(IT_NAME)             \
    (CRS_REG_BIT(CR,IT_NAME##IE) = 0)

/**
 * @brief  Get the specified CRS flag.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg SYNCOK:   SYNC event OK
 *            @arg SYNCWARN: SYNC warning
 *            @arg ESYNC:    Expected SYNC event
 *            @arg SYNCMISS: SYNC event missed
 *            @arg SYNCERR:  SYNC error
 *            @arg TRIMOVF:  Trimming overflow or underflow
 */
#define         CRS_FLAG_STATUS(FLAG_NAME)          \
    CRS_REG_BIT(ISR,FLAG_NAME##F)

/**
 * @brief  Clear the specified CRS flag.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg SYNCOK:   SYNC event OK
 *            @arg SYNCWARN: SYNC warning
 *            @arg ESYNC:    Expected SYNC event
 *            @arg ERR:      SYNC errors
  */
#define         CRS_FLAG_CLEAR(FLAG_NAME)           \
    (CRS->ICR.w = (((CRS_ISR_##FLAG_NAME##F) & CRS_ERROR_FLAGS_MASK) != 0) ? \
     CRS_ICR_ERRC : (CRS_ISR_##FLAG_NAME##F))

/** @} */

/** @addtogroup CRS_Exported_Functions
 * @{ */
void            CRS_vInit                (const CRS_InitType * pxConfig);
void            CRS_vDeinit              (void);

int32_t         CRS_lGetErrorValue       (void);
XPD_ReturnType  CRS_ePollStatus          (CRS_StatusType * peStatus, uint32_t ulTimeout);
CRS_StatusType  CRS_eGetStatus           (void);
void            CRS_vIRQHandler          (void);

/**
 * @brief Generates a software synchronization event.
 */
__STATIC_INLINE void CRS_vGenerateSync(void)
{
    CRS_REG_BIT(CR,SWSYNC) = 1;
}

/**
 * @brief Sets the new state for the frequency error counter.
 * @param eNewState: the new setting
 */
__STATIC_INLINE void CRS_vFreqErrorCounter(FunctionalState eNewState)
{
    CRS_REG_BIT(CR,CEN) = eNewState;
}

/**
 * @brief Sets the new state for the automatic calibration.
 * @param eNewState: the new setting
 */
__STATIC_INLINE void CRS_vAutoCalibration(FunctionalState eNewState)
{
    CRS_REG_BIT(CR,AUTOTRIMEN) = eNewState;
}

/** @} */

/** @} */

/** @} */

#endif /* CRS */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_CRS_H_ */
