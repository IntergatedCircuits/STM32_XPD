/**
  ******************************************************************************
  * @file    xpd_crs.c
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
#include <xpd_crs.h>
#include <xpd_rcc.h>
#include <xpd_utils.h>

#if defined(CRS)

/** @addtogroup RCC
 * @{ */

/** @addtogroup CRS
 * @{ */

CRS_CalllbacksType CRS_xCallbacks = { NULL, NULL, NULL, NULL };

/** @defgroup CRS_Exported_Functions CRS Exported Functions
 * @{ */

/**
 * @brief Initializes the CRS peripheral using the setup configuration
 * @param pxConfig: HSI48 setup configuration
 */
void CRS_vInit(const CRS_InitType * pxConfig)
{
    /* pxConfigure automatic synchronization */
    if (pxConfig->Source != CRS_SYNC_SOURCE_NONE)
    {
        RCC_vClockEnable(RCC_POS_CRS);
        CRS->CR.w = 0;

        switch (pxConfig->Source)
        {
            case CRS_SYNC_SOURCE_USB:
                /* StartOfFrame signal frequency is 1 kHz */
                CRS->CFGR.w = (HSI48_VALUE_Hz / 1000) - 1;
                break;
#ifdef LSE_VALUE_Hz
            case CRS_SYNC_SOURCE_LSE:
                CRS->CFGR.w = (HSI48_VALUE_Hz / LSE_VALUE_Hz) - 1;
                break;
#endif

            default:
                /* GPIO or SW input */
                CRS->CFGR.w = (HSI48_VALUE_Hz / pxConfig->Frequency) - 1;

                CRS->CFGR.b.SYNCDIV = pxConfig->Divider;
                CRS_REG_BIT(CFGR,SYNCPOL) = pxConfig->Polarity;
                break;
        }
        CRS->CFGR.b.SYNCSRC = pxConfig->Source;
        CRS->CFGR.b.FELIM   = pxConfig->ErrorLimit;

        /* Enable Automatic trimming & Frequency error counter */
        SET_BIT(CRS->CR.w, CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
    }
    /* If non-default calibration is used, set it */
    else if (pxConfig->CalibVal != HSI48_CALIBRATION_DEFAULT_VALUE)
    {
        RCC_vClockEnable(RCC_POS_CRS);
        CRS->CR.w = (pxConfig->CalibVal << CRS_CR_TRIM_Pos) & CRS_CR_TRIM_Msk;
    }
    /* No functionality is requested, turn off */
    else
    {
        RCC_vClockDisable(RCC_POS_CRS);
    }
}

/**
 * @brief Restores the CRS peripheral to its default inactive state
 */
void CRS_vDeinit(void)
{
    RCC_vClockDisable(RCC_POS_CRS);
}

/**
 * @brief Returns the signed frequency error
 *        (positive means FEDIR = 0 - the actual frequency is above the target)
 * @return The frequency error as a signed value
 */
int32_t CRS_lGetErrorValue(void)
{
    return (int32_t)CRS->ISR.b.FECAP * (1 - (2 * (int32_t)CRS_REG_BIT(ISR,FEDIR)));
}

/**
 * @brief Polls the input status flags for the specified timeout, and returns with the
 *        first active flag.
 * @param eStatus: Input the flags to monitor, output is the activated flags (if return is OK)
 * @param ulTimeout: Amount of milliseconds to wait
 * @return OK if status flag is activated, TIMEOUT otherwise
 */
XPD_ReturnType CRS_ePollStatus(CRS_StatusType * eStatus, uint32_t ulTimeout)
{
    XPD_ReturnType eResult = XPD_eWaitForDiff(&CRS->ISR.w, *eStatus, 0, &ulTimeout);

    if (eResult == XPD_OK)
    {
        /* Indicate which of the selected flags were set */
        *eStatus &= CRS->ISR.w;

        /* Clear the polled flags */
        if ((*eStatus & CRS_ERROR_FLAGS_MASK) != 0)
        {
            CRS->ICR.w = CRS_ICR_ERRC | *eStatus;
        }
        else
        {
            CRS->ICR.w = *eStatus;
        }
    }
    return eResult;
}

/**
 * @brief Reads the currently active status flags and clears them.
 * @return The active status flags
 */
CRS_StatusType CRS_eGetStatus(void)
{
    CRS_StatusType eStatus = CRS->ISR.w &
            (CRS_ISR_SYNCOKF | CRS_ISR_SYNCWARNF | CRS_ISR_SYNCMISSF |
             CRS_ISR_SYNCERRF | CRS_ISR_ESYNCF | CRS_ISR_TRIMOVF);

    /* Clear the polled flags */
    if ((eStatus & CRS_ERROR_FLAGS_MASK) != 0)
    {
        CRS->ICR.w = CRS_ICR_ERRC | eStatus;
    }
    else
    {
        CRS->ICR.w = eStatus;
    }
    return eStatus;
}

/**
 * @brief CRS interrupt handler that provides HSI48 synchronization callbacks.
 */
void CRS_vIRQHandler(void)
{
    uint32_t ulISR = CRS->ISR.w;
    uint32_t ulCR = CRS->CR.w;

    /* SyncSuccess */
    if (((ulISR & CRS_ISR_SYNCOKF) != 0) && ((ulCR & CRS_CR_SYNCOKIE) != 0))
    {
        CRS_FLAG_CLEAR(SYNCOK);

        XPD_SAFE_CALLBACK(CRS_xCallbacks.SyncSuccess,);
    }
    /* SyncWarning */
    if (((ulISR & CRS_ISR_SYNCWARNF) != 0) && ((ulCR & CRS_CR_SYNCWARNIE) != 0))
    {
        CRS_FLAG_CLEAR(SYNCWARN);

        XPD_SAFE_CALLBACK(CRS_xCallbacks.SyncWarning,);
    }
    /* SyncExpected */
    if (((ulISR & CRS_ISR_ESYNCF) != 0) && ((ulCR & CRS_CR_ESYNCIE) != 0))
    {
        CRS_FLAG_CLEAR(ESYNC);

        XPD_SAFE_CALLBACK(CRS_xCallbacks.SyncExpected,);
    }
    /* SyncErrors */
    if (((ulISR & CRS_ERROR_FLAGS_MASK) != 0) && ((ulCR & CRS_CR_ERRIE) != 0))
    {
        /* Calling XPD_RCC_CRS_GetStatus in callback context
         * will return the active errors */
        XPD_SAFE_CALLBACK(CRS_xCallbacks.SyncError,);

        /* Flag is cleared after callback */
        CRS_FLAG_CLEAR(ERR);
    }
}

/** @} */

/** @} */

/** @} */

#endif /* CRS */
