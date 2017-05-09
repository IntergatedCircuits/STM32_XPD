/**
  ******************************************************************************
  * @file    xpd_rcc_crs.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-05-08
  * @brief   STM32 eXtensible Peripheral Drivers Clock Recovery System
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
#ifndef __XPD_RCC_CRS_H_
#define __XPD_RCC_CRS_H_

#ifdef CRS

/** @addtogroup RCC
 * @{ */

/** @defgroup RCC_CRS RCC Clock Recovery System
 * @{ */

/** @defgroup RCC_CRS_Exported_Types RCC CRS Exported Types
 * @{ */

/** @brief CRS synchronization source selection */
typedef enum
{
    CRS_SYNC_SOURCE_NONE     = 0,   /*!< No automatic synchronization */
    CRS_SYNC_SOURCE_GPIO     = 4,   /*!< Synchronization on GPIO input signal */
#ifdef LSE_VALUE
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

/** @} */

/** @defgroup CRS_Exported_Macros CRS Exported Macros
 * @{ */

#ifdef CRS_BB
#define CRS_REG_BIT(REG_NAME, BIT_NAME) (CRS_BB->REG_NAME.BIT_NAME)
#else
#define CRS_REG_BIT(REG_NAME, BIT_NAME) (CRS->REG_NAME.b.BIT_NAME)
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
#define         XPD_CRS_EnableIT(IT_NAME)   \
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
#define         XPD_CRS_DisableIT(IT_NAME)   \
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
#define         XPD_CRS_GetFlag(FLAG_NAME)  \
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
#define         XPD_CRS_ClearFlag(FLAG_NAME)     \
    (CRS->ICR.w = (((CRS_ISR_##FLAG_NAME##F) & CRS_ERROR_FLAGS_MASK) != 0) ? \
     CRS_ICR_ERRC : (CRS_ISR_##FLAG_NAME##F))

/** @} */

/** @addtogroup RCC_CRS_Exported_Functions
 * @{ */
void            XPD_CRS_Init                    (const CRS_InitType * Config);
void            XPD_CRS_Deinit                  (void);

int32_t         XPD_CRS_GetErrorValue           (void);
XPD_ReturnType  XPD_CRS_PollStatus              (CRS_StatusType * Status, uint32_t Timeout);
CRS_StatusType  XPD_CRS_GetStatus               (void);
void            XPD_CRS_IRQHandler              (void);

/**
 * @brief Generates a software synchronization event.
 */
__STATIC_INLINE void XPD_CRS_GenerateSync(void)
{
    CRS_REG_BIT(CR,SWSYNC) = 1;
}

/**
 * @brief Sets the new state for the frequency error counter.
 * @param NewState: the new setting
 */
__STATIC_INLINE void XPD_CRS_FreqErrorCounter(FunctionalState NewState)
{
    CRS_REG_BIT(CR,CEN) = NewState;
}

/**
 * @brief Sets the new state for the automatic calibration.
 * @param NewState: the new setting
 */
__STATIC_INLINE void XPD_CRS_AutoCalibration(FunctionalState NewState)
{
    CRS_REG_BIT(CR,AUTOTRIMEN) = NewState;
}

/** @} */

/** @} */

/** @} */

#endif /* CRS */

#endif /* __XPD_RCC_CRS_H_ */
