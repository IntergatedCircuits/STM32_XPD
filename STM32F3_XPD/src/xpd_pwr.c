/**
  ******************************************************************************
  * @file    xpd_pwr.c
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Power Module
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
#include <xpd_pwr.h>

/** @addtogroup PWR
 * @{ */

/** @addtogroup PWR_Core
 * @{ */

/** @defgroup PWR_Exported_Functions PWR Exported Functions
 * @{ */

/**
 * @brief Enters Sleep mode.
 * @note  In Sleep mode, all I/O pins keep the same state as in Run mode.
 * @param eWakeUpOn: Specifies if SLEEP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 */
void PWR_vSleepMode(ReactionType eWakeUpOn)
{
    /* Clear SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR.b.SLEEPDEEP = 0;

    if (eWakeUpOn == REACTION_IT)
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
 * @note  When the voltage regulator operates in low power mode, an additional
 *         startup delay is incurred when waking up from Stop mode.
 *         By keeping the internal regulator ON during Stop mode, the consumption
 *         is higher although the startup time is reduced.
 * @param eWakeUpOn: Specifies if STOP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 * @param eRegulator: Specifies the regulator state in STOP mode
 */
void PWR_vStopMode(ReactionType eWakeUpOn, PWR_RegulatorType eRegulator)
{
    /* Clear PDDS bit */
    PWR_REG_BIT(CR,PDDS) = 0;

    /* Set LPDS bit according to eRegulator value */
    PWR_REG_BIT(CR,LPDS) = eRegulator;

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR.b.SLEEPDEEP = 1;

    /* Select STOP mode entry */
    if (eWakeUpOn == REACTION_IT)
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
void PWR_vStandbyMode(void)
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
 * @param ucWakeUpPin: Specifies the Power Wake-Up pin to enable.
 *         Check CSR register for the available EWUP bits.
 */
void PWR_vWakeUpPin_Enable(uint8_t ucWakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CSR.EWUP1 + ucWakeUpPin - 1) = 1;
#else
    SET_BIT(PWR->CSR.w, (PWR_CSR_EWUP1 << (ucWakeUpPin - 1)));
#endif
}

/**
 * @brief Disables the WakeUp PINx functionality.
 * @param ucWakeUpPin: Specifies the Power Wake-Up pin to disable.
 *         Check CSR register for the available EWUP bits.
 */
void PWR_vWakeUpPin_Disable(uint8_t ucWakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CSR.EWUP1 + ucWakeUpPin - 1) = 0;
#else
    CLEAR_BIT(PWR->CSR.w, (PWR_CSR_EWUP1 << (ucWakeUpPin - 1)));
#endif
}

/** @} */

/** @} */

/** @} */
