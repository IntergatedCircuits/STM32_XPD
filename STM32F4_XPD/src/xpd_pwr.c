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
#include <xpd_rcc.h>
#include <xpd_utils.h>

/** @addtogroup PWR
 * @{ */

#define PWR_BKPREG_TIMEOUT          1000
#define PWR_VOSRDY_TIMEOUT          1000
#define PWR_OVERDRIVE_TIMEOUT       1000
#define PWR_UNDERDRIVE_TIMEOUT      1000

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
 * @note  When the voltage regulator operates in low power mode (Stop 1), an additional
 *         startup delay is incurred when waking up from Stop mode.
 *         By keeping the internal regulator ON during Stop mode (Stop 0), the consumption
 *         is higher although the startup time is reduced.
 * @param eWakeUpOn: Specifies if STOP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 * @param eRegulator: Specifies the regulator state in STOP mode
 * @note  Using Underdrive mode, the 1.2V domain is preserved in reduced leakage mode. This
 *        mode is only available when the main regulator or the low power regulator
 *        is in low voltage mode
 */
void PWR_vStopMode(ReactionType eWakeUpOn, PWR_RegulatorType eRegulator)
{
#ifdef PWR_CR_UDEN
    /* Enable the Under-drive Mode when selected */
    if (eRegulator & (PWR_CR_LPUDS | PWR_CR_MRUDS) != 0)
    {
        uint32_t ulTimeout = PWR_UNDERDRIVE_TIMEOUT;

        /* Clear Under-drive flag */
        PWR_REG_BIT(CSR,UDSWRDY) = 0;

        /* Enable the Under-drive */
        PWR_REG_BIT(CR,UDEN) = 1;

        /* If Underdrive setting is successful, set STOP mode with underdrive */
        if (XPD_OK == XPD_eWaitForMatch(&RCC->CR.w,
                PWR_CSR_UDSWRDY, PWR_CSR_UDSWRDY, &ulTimeout))
        {
            MODIFY_REG(PWR->CR.w, PWR_CR_LPUDS | PWR_CR_MRUDS, eRegulator);
        }
    }
#endif
    /* Clear PDDS bit */
    PWR_REG_BIT(CR,PDDS) = 0;

    /* Set LPDS bit according to Regulator value */
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
 * @brief Sets the Backup Regulator state.
 * @param eNewState: the new backup regulator state to set
 * @return ERROR if backup access is locked, TIMEOUT when setting timed out, OK when successful
 */
XPD_ReturnType XPD_PWR_BackupRegulatorCtrl(FunctionalState eNewState)
{
    XPD_ReturnType eResult = XPD_ERROR;

    /* BRE can only be written when backup access is enabled */
    if (PWR_REG_BIT(CR,DBP) != 0)
    {
        uint32_t ulTimeout = PWR_BKPREG_TIMEOUT;

        PWR_REG_BIT(CSR,BRE) = eNewState;

        eResult = XPD_eWaitForMatch(&PWR->CSR.w,
                PWR_CSR_BRR, PWR_CSR_BRR * eNewState, &ulTimeout);
    }
    return eResult;
}

/**
 * @brief Enables the WakeUp PINx functionality.
 * @param ucWakeUpPin: Specifies the Power Wake-Up pin to enable.
 */
void PWR_vWakeUpPin_Enable(uint8_t ucWakeUpPin)
{
    switch (ucWakeUpPin)
    {
        case 1:
#ifdef PWR_CSR_EWUP1
            PWR_REG_BIT(CSR, EWUP1) = 1;
#else
            PWR_REG_BIT(CSR, EWUP) = 1;
#endif
            break;
#if defined(PWR_CSR_EWUP2)
        case 2:
            PWR_REG_BIT(CSR, EWUP2) = 1;
            break;
#endif
#if defined(PWR_CSR_EWUP3)
        case 3:
            PWR_REG_BIT(CSR, EWUP3) = 1;
            break;
#endif
    }
}

/**
 * @brief Disables the WakeUp PINx functionality.
 * @param ucWakeUpPin: Specifies the Power Wake-Up pin to disable.
 */
void PWR_vWakeUpPin_Disable(uint8_t ucWakeUpPin)
{
    switch (ucWakeUpPin)
    {
        case 1:
#if defined(PWR_CSR_EWUP1)
            PWR_REG_BIT(CSR, EWUP1) = 0;
#else
            PWR_REG_BIT(CSR, EWUP) = 0;
#endif
            break;
#if defined(PWR_CSR_EWUP2)
        case 2:
            PWR_REG_BIT(CSR, EWUP2) = 0;
            break;
#endif
#if defined(PWR_CSR_EWUP3)
        case 3:
            PWR_REG_BIT(CSR, EWUP3) = 0;
            break;
#endif
    }
}

#ifdef PWR_CSR_WUPP
/**
 * @brief Sets the WakeUp Pin polarity.
 * @param ucWakeUpPin: Specifies the Wake-Up pin to configure.
 * @param eRisingOrFalling: the polarity. Permitted values:
             @arg @ref EdgeType::EDGE_RISING
             @arg @ref EdgeType::EDGE_FALLING
 */
void PWR_vWakeUpPin_SetPolarity(uint8_t ucWakeUpPin, EdgeType eRisingOrFalling)
{
    if (eRisingOrFalling == EDGE_FALLING)
    {
        PWR_REG_BIT(CSR,WUPP) = 1;
    }
    else
    {
        PWR_REG_BIT(CSR,WUPP) = 0;
    }
}
#endif

/** @} */

/** @} */

#ifdef PWR_CR_VOS
/** @addtogroup PWR_Regulator_Voltage_Scaling
 * @{ */

/** @defgroup PWR_Regulator_Voltage_Scaling_Exported_Functions PWR Regulator Voltage Scaling Exported Functions
 * @{ */

/**
 * @brief Sets the new Regulator Voltage Scaling configuration.
 * @param eScaling the new scaling value
 * @return ERROR if operation is blocked, TIMEOUT if setting fails, OK when successful
 */
XPD_ReturnType PWR_eVoltageScaleConfig(PWR_RegVoltScaleType eScaling)
{
    XPD_ReturnType eResult;
    uint32_t ulTimeout = PWR_VOSRDY_TIMEOUT;

#ifdef PWR_CR_VOS_1
    /* Check if the PLL is used as system clock or not */
    if (RCC->CFGR.b.SWS == PLL)
    {
        eResult = XPD_ERROR;
    }
    else
#endif
    {
#ifdef PWR_CR_VOS_1
        bool bPllOn = RCC_REG_BIT(CR,PLLRDY);

        if (bPllOn)
        {
            /* disable the main PLL. */
            RCC_REG_BIT(CR,PLLON) = 0;

            /* wait until PLL is disabled */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_PLLRDY, 0, RCC_PLL_TIMEOUT);
            if (eResult != XPD_OK)
            {
                return eResult;
            }
        }
#endif
        PWR->CR.b.VOS = eScaling;

#ifdef PWR_CR_VOS_1
        if (bPllOn)
        {
            /* enable the main PLL. */
            RCC_REG_BIT(CR,PLLON) = 1;

            /* wait until PLL is enabled */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_PLLRDY, RCC_CR_PLLRDY, RCC_PLL_TIMEOUT);
            if (eResult != XPD_OK)
            {
                return eResult;
            }
        }
#endif
        eResult = XPD_eWaitForMatch(&PWR->CSR.w,
                PWR_CSR_VOSRDY, PWR_CSR_VOSRDY, &ulTimeout);
    }
    return eResult;
}

#if defined(PWR_CR_MRLVDS) && defined(PWR_CR_LPLVDS)
/**
 * @brief Sets the low voltage mode for the selected regulator.
 * @param eRegulator: the regulator to configure
 * @param eNewState: the low voltage mode state to set
 */
void PWR_vRegLowVoltageConfig(PWR_RegulatorType eRegulator, FunctionalState eNewState)
{
    if (eRegulator == PWR_MAINREGULATOR)
    {
        PWR_REG_BIT(CR,MRLVDS) = eNewState;
    }
    else
    {
        PWR_REG_BIT(CR,LPLVDS) = eNewState;
    }
}
#endif

/** @} */

/** @} */
#endif /* PWR_CR_VOS */

#ifdef PWR_CR_ODEN

/** @addtogroup PWR_OverDrive_Mode
 * @{ */

/** @defgroup PWR_OverDrive_Mode_Exported_Functions PWR OverDrive Mode Exported Functions
 * @{ */

/**
 * @brief  Activates the Over-Drive mode, allowing the CPU and the core logic to operate at a higher frequency
 *         than the normal mode for a given voltage scaling (scale 1, scale 2 or scale 3).
 * @note   It is recommended to enter or exit Over-drive mode when the application is not running
 *         critical tasks and when the system clock source is either HSI or HSE.
 *         During the Over-drive switch activation, no peripheral clocks should be enabled.
 * @retval TIMEOUT if setting fails, OK when successful
 */
XPD_ReturnType PWR_vOverDrive_Enable(void)
{
    XPD_ReturnType eResult;
    uint32_t ulTimeout = PWR_OVERDRIVE_TIMEOUT;

    /* Enable the Overdrive to extend the clock frequency to 180 Mhz */
    PWR_REG_BIT(CR, ODEN) = 1;

    /* Wait until Overdrive is ready */
    eResult = XPD_eWaitForMatch(&RCC->CR.w,
            PWR_CSR_ODRDY, PWR_CSR_ODRDY, &ulTimeout);

    if (eResult == XPD_OK)
    {
        /* Enable the Overdrive switch */
        PWR_REG_BIT(CR, ODSWEN) = 1;

        /* Wait until Overdrive SW is ready */
        eResult = XPD_eWaitForMatch(&RCC->CR.w,
                PWR_CSR_ODSWRDY, PWR_CSR_ODSWRDY, &ulTimeout);
    }

    return eResult;
}

/**
 * @brief  Deactivates the Over-Drive mode.
 * @note   It is recommended to enter or exit Over-drive mode when the application is not running
 *         critical tasks and when the system clock source is either HSI or HSE.
 *         During the Over-drive switch activation, no peripheral clocks should be enabled.
 * @retval TIMEOUT if setting fails, OK when successful
 */
XPD_ReturnType PWR_vOverDrive_Disable(void)
{
    XPD_ReturnType eResult;
    uint32_t ulTimeout = PWR_OVERDRIVE_TIMEOUT;

    /* Disable the Over-drive switch */
    PWR_REG_BIT(CR, ODSWEN) = 0;

    /* Wait until Overdrive SW is not ready */
    eResult = XPD_eWaitForMatch(&RCC->CR.w,
            PWR_CSR_ODSWRDY, 0, &ulTimeout);

    if (eResult == XPD_OK)
    {
        /* Disable the Over-drive switch */
        PWR_REG_BIT(CR, ODEN) = 0;

        /* Wait until Overdrive is not ready */
        eResult = XPD_eWaitForMatch(&RCC->CR.w,
                PWR_CSR_ODRDY, 0, &ulTimeout);
    }

    return eResult;
}

/** @} */

/** @} */
#endif /* PWR_CR_ODEN */

/** @} */
