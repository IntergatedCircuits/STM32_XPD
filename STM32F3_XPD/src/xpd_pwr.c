/**
  ******************************************************************************
  * @file    xpd_pwr.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-11-01
  * @brief   STM32 eXtensible Peripheral Drivers Power Module
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
#include "xpd_pwr.h"

/** @addtogroup PWR
 * @{ */

/** @addtogroup PWR_Core
 * @{ */

/** @defgroup PWR_Exported_Functions PWR Exported Functions
 * @{ */

/**
 * @brief Enters Sleep mode.
 * @note  In Sleep mode, all I/O pins keep the same state as in Run mode.
 * @param WakeUpOn: Specifies if SLEEP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 */
void XPD_PWR_SleepMode(ReactionType WakeUpOn)
{
    /* Clear SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR.b.SLEEPDEEP = 0;

    if (WakeUpOn == REACTION_IT)
    {
        /* Request Wait For Interrupt */
        __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();
    }
}

/**
 * @brief Enters STOP mode.
 * @note  In Stop mode, all I/O pins keep the same state as in Run mode.
 * @note  When exiting Stop mode by issuing an interrupt or a wakeup event,
 *         the HSI RC oscillator is selected as system clock.
 * @note  When the voltage regulator operates in low power mode (Stop 1), an additional
 *         startup delay is incurred when waking up from Stop mode.
 *         By keeping the internal regulator ON during Stop mode (Stop 0), the consumption
 *         is higher although the startup time is reduced.
 * @param WakeUpOn: Specifies if STOP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 * @param Regulator: Specifies the regulator state in STOP mode
 */
void XPD_PWR_StopMode(ReactionType WakeUpOn, PWR_RegulatorType Regulator)
{
    /* Clear PDDS bit */
    PWR_REG_BIT(CR,PDDS) = 0;

    /* Set LPDS bit according to Regulator value */
    PWR_REG_BIT(CR,LPDS) = Regulator;

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR.b.SLEEPDEEP = 1;

    /* Select STOP mode entry */
    if (WakeUpOn == REACTION_IT)
    {
        /* Request Wait For Interrupt */
        __WFI();
    }
    else
    {
        /* Request Wait For Event */
        __SEV();
        __WFE();
        __WFE();
    }

    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR.b.SLEEPDEEP = 0;
}

/**
 * @brief Enters STANDBY mode.
 * @note  In Standby mode, all I/O pins are high impedance except for:
 *          - Reset pad (still available),
 *          - RTC alternate function pins if configured for tamper, time-stamp, RTC
 *            Alarm out, or RTC clock calibration out,
 *          - WKUP pins if enabled.
 */
void XPD_PWR_StandbyMode(void)
{
    /* Select STANDBY mode */
    PWR_REG_BIT(CR,PDDS) = 1;

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR.b.SLEEPDEEP = 1;

    /* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
    __force_stores();
#endif
    /* Request Wait For Interrupt */
    __WFI();
}

/**
 * @brief Enables the WakeUp PINx functionality.
 * @param WakeUpPin: Specifies the Power Wake-Up pin to enable.
 *         Check CSR register for the available EWUP bits.
 */
void XPD_PWR_WakeUpPin_Enable(uint8_t WakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CSR.EWUP1 + WakeUpPin - 1) = ENABLE;
#else
    SET_BIT(PWR->CSR.w, (PWR_CSR_EWUP1 << (WakeUpPin - 1)));
#endif
}

/**
 * @brief Disables the WakeUp PINx functionality.
 * @param WakeUpPin: Specifies the Power Wake-Up pin to disable.
 *         Check CSR register for the available EWUP bits.
 */
void XPD_PWR_WakeUpPin_Disable(uint8_t WakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CSR.EWUP1 + WakeUpPin - 1) = DISABLE;
#else
    CLEAR_BIT(PWR->CSR.w, (PWR_CSR_EWUP1 << (WakeUpPin - 1)));
#endif
}

/** @} */

/** @} */

/** @} */
