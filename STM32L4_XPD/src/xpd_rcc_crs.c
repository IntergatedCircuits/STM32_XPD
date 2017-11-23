/**
  ******************************************************************************
  * @file    xpd_rcc_crs.c
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
#include "xpd_rcc.h"
#include "xpd_utils.h"

#if defined(CRS)
/** @addtogroup RCC
 * @{ */

/** @addtogroup RCC_CRS
 * @{ */

/** @defgroup RCC_CRS_Exported_Functions RCC CRS Exported Functions
 * @{ */

/**
 * @brief Initializes the CRS peripheral using the setup configuration
 * @param Config: HSI48 setup configuration
 */
void XPD_CRS_Init(const CRS_InitType * Config)
{
    /* Configure automatic synchronization */
    if (Config->Source != CRS_SYNC_SOURCE_NONE)
    {
        XPD_RCC_ClockEnable(RCC_POS_CRS);
        CRS->CR.w = 0;

        switch (Config->Source)
        {
            case CRS_SYNC_SOURCE_USB:
                /* StartOfFrame signal frequency is 1 kHz */
                CRS->CFGR.w = (HSI48_VALUE / 1000) - 1;
                break;
#ifdef LSE_VALUE
            case CRS_SYNC_SOURCE_LSE:
                CRS->CFGR.w = (HSI48_VALUE / LSE_VALUE) - 1;
                break;
#endif

            default:
                /* GPIO or SW input */
                CRS->CFGR.w = (HSI48_VALUE / Config->Frequency) - 1;

                CRS->CFGR.b.SYNCDIV = Config->Divider;
                CRS_REG_BIT(CFGR,SYNCPOL) = Config->Polarity;
                break;
        }
        CRS->CFGR.b.SYNCSRC = Config->Source;
        CRS->CFGR.b.FELIM   = Config->ErrorLimit;

        /* Enable Automatic trimming & Frequency error counter */
        SET_BIT(CRS->CR.w, CRS_CR_AUTOTRIMEN | CRS_CR_CEN);
    }
    /* If non-default calibration is used, set it */
    else if (Config->CalibVal != HSI48_CALIBRATION_DEFAULT_VALUE)
    {
        XPD_RCC_ClockEnable(RCC_POS_CRS);
        CRS->CR.w = (Config->CalibVal << CRS_CR_TRIM_Pos) & CRS_CR_TRIM_Msk;
    }
    /* No functionality is requested, turn off */
    else
    {
        XPD_RCC_ClockDisable(RCC_POS_CRS);
    }
}

/**
 * @brief Restores the CRS peripheral to its default inactive state
 */
void XPD_CRS_Deinit(void)
{
    XPD_RCC_ClockDisable(RCC_POS_CRS);
}

/**
 * @brief Returns the signed frequency error
 *        (positive means FEDIR = 0 - the actual frequency is above the target)
 * @return The frequency error as a signed value
 */
int32_t XPD_CRS_GetErrorValue(void)
{
    return (int32_t)CRS->ISR.b.FECAP * (1 - (2 * (int32_t)CRS_REG_BIT(ISR,FEDIR)));
}

/**
 * @brief Polls the input status flags for the specified timeout, and returns with the
 *        first active flag.
 * @param Status: Input the flags to monitor, output is the activated flags (if return is OK)
 * @param Timeout: Amount of milliseconds to wait
 * @return OK if status flag is activated, TIMEOUT otherwise
 */
XPD_ReturnType XPD_CRS_PollStatus(CRS_StatusType * Status, uint32_t Timeout)
{
    XPD_ReturnType result = XPD_WaitForDiff(&CRS->ISR.w, *Status, 0, &Timeout);

    if (result == XPD_OK)
    {
        /* Indicate which of the selected flags were set */
        *Status &= CRS->ISR.w;

        /* Clear the polled flags */
        if ((*Status & CRS_ERROR_FLAGS_MASK) != 0)
        {
            CRS->ICR.w = CRS_ICR_ERRC | *Status;
        }
        else
        {
            CRS->ICR.w = *Status;
        }
    }
    return result;
}

/**
 * @brief Reads the currently active status flags and clears them.
 * @return The active status flags
 */
CRS_StatusType XPD_CRS_GetStatus(void)
{
    CRS_StatusType status = CRS->ISR.w &
            (CRS_ISR_SYNCOKF | CRS_ISR_SYNCWARNF | CRS_ISR_SYNCMISSF |
             CRS_ISR_SYNCERRF | CRS_ISR_ESYNCF | CRS_ISR_TRIMOVF);

    /* Clear the polled flags */
    if ((status & CRS_ERROR_FLAGS_MASK) != 0)
    {
        CRS->ICR.w = CRS_ICR_ERRC | status;
    }
    else
    {
        CRS->ICR.w = status;
    }
    return status;
}

/**
 * @brief CRS interrupt handler that provides HSI48 synchronization callbacks.
 */
void XPD_CRS_IRQHandler(void)
{
    uint32_t isr = CRS->ISR.w;
    uint32_t cr = CRS->CR.w;

    /* SyncSuccess */
    if (((isr & CRS_ISR_SYNCOKF) != 0) && ((cr & CRS_CR_SYNCOKIE) != 0))
    {
        XPD_CRS_ClearFlag(SYNCOK);

        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.HSI48.SyncSuccess,);
    }
    /* SyncWarning */
    if (((isr & CRS_ISR_SYNCWARNF) != 0) && ((cr & CRS_CR_SYNCWARNIE) != 0))
    {
        XPD_CRS_ClearFlag(SYNCWARN);

        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.HSI48.SyncWarning,);
    }
    /* SyncExpected */
    if (((isr & CRS_ISR_ESYNCF) != 0) && ((cr & CRS_CR_ESYNCIE) != 0))
    {
        XPD_CRS_ClearFlag(ESYNC);

        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.HSI48.SyncExpected,);
    }
    /* SyncErrors */
    if (((isr & CRS_ERROR_FLAGS_MASK) != 0) && ((cr & CRS_CR_ERRIE) != 0))
    {
        /* Calling XPD_RCC_CRS_GetStatus in callback context
         * will return the active errors */
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.HSI48.SyncError,);

        /* Flag is cleared after callback */
        XPD_CRS_ClearFlag(ERR);
    }
}

/** @} */

/** @} */

/** @} */

#endif /* CRS */
