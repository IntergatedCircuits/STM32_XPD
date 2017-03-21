/**
  ******************************************************************************
  * @file    xpd_pwr.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-01-28
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
#include "xpd_rcc.h"
#include "xpd_utils.h"

/** @addtogroup PWR
 * @{ */

#define PWR_BKPREG_TIMEOUT     1000
#define PWR_VOSRDY_TIMEOUT     1000
#define PWR_OVERDRIVE_TIMEOUT  1000
#define PWR_UNDERDRIVE_TIMEOUT 1000

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
 * @note  When the voltage regulator operates in low power mode, an additional
 *         startup delay is incurred when waking up from Stop mode.
 *         By keeping the internal regulator ON during Stop mode, the consumption
 *         is higher although the startup time is reduced.
 * @param WakeUpOn: Specifies if STOP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 * @param Regulator: Specifies the regulator state in STOP mode
 * @note  Using Underdrive mode, the 1.2V domain is preserved in reduced leakage mode. This
 *        mode is only available when the main regulator or the low power regulator
 *        is in low voltage mode
 */
void XPD_PWR_StopMode(ReactionType WakeUpOn, PWR_RegulatorType Regulator)
{
#ifdef PWR_CR_UDEN
    /* Enable the Under-drive Mode when selected */
    if (Regulator & (PWR_CR_LPUDS | PWR_CR_MRUDS) != 0)
    {
        uint32_t timeout = PWR_UNDERDRIVE_TIMEOUT;

        /* Clear Under-drive flag */
        PWR_REG_BIT(CSR,UDSWRDY) = 0;

        /* Enable the Under-drive */
        PWR_REG_BIT(CR,UDEN) = 1;

        /* If Underdrive setting is successful, set STOP mode with underdrive */
        if (XPD_OK == XPD_WaitForMatch(&RCC->CR.w, PWR_CSR_UDSWRDY, PWR_CSR_UDSWRDY, &timeout))
        {
            MODIFY_REG(PWR->CR.w, PWR_CR_LPUDS | PWR_CR_MRUDS, Regulator);
        }
    }
#endif
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
 * @brief Enables or disables access to the backup domain (RTC registers, RTC
 *         backup data registers when present).
 * @param NewState: the new backup access state to set
 * @note  If the HSE divided by 2, 3, ..31 is used as the RTC clock, the
 *         Backup Domain Access should be kept enabled.
 */
void XPD_PWR_BackupAccessCtrl(FunctionalState NewState)
{
    PWR_REG_BIT(CR,DBP) = NewState;
}

/**
 * @brief Sets the Backup Regulator state.
 * @param NewState: the new backup regulator state to set
 * @retval ERROR if backup access is locked, TIMEOUT when setting timed out, OK when successful
 */
XPD_ReturnType XPD_PWR_BackupRegulatorCtrl(FunctionalState NewState)
{
    XPD_ReturnType result = XPD_ERROR;

    /* BRE can only be written when backup access is enabled */
    if (PWR_REG_BIT(CR,DBP) != 0)
    {
        uint32_t timeout = PWR_BKPREG_TIMEOUT;

        PWR_REG_BIT(CSR,BRE) = NewState;

        result = XPD_WaitForMatch(&PWR->CSR.w, PWR_CSR_BRR, PWR_CSR_BRR * NewState, &timeout);
    }
    return result;
}

#ifdef PWR_CR_FPDS
/**
 * @brief Sets the Flash Power Down state in Stop mode.
 * @param NewState: the new Flash power down state to set
 */
void XPD_PWR_FlashPowerDownCtrl(FunctionalState NewState)
{
    PWR_REG_BIT(CR, FPDS) = NewState;
}
#endif

/**
 * @brief Enables the WakeUp PINx functionality.
 * @param WakeUpPin: Specifies the Power Wake-Up pin to enable.
 */
void XPD_PWR_WakeUpPin_Enable(uint8_t WakeUpPin)
{
    switch (WakeUpPin)
    {
        case 1:
#ifdef PWR_CSR_EWUP1
            PWR_REG_BIT(CSR, EWUP1) = ENABLE;
#else
            PWR_REG_BIT(CSR, EWUP) = ENABLE;
#endif
            break;
#if defined(PWR_CSR_EWUP2)
        case 2:
            PWR_REG_BIT(CSR, EWUP2) = ENABLE;
            break;
#endif
#if defined(PWR_CSR_EWUP3)
        case 3:
            PWR_REG_BIT(CSR, EWUP3) = ENABLE;
            break;
#endif
    }
}

/**
 * @brief Disables the WakeUp PINx functionality.
 * @param WakeUpPin: Specifies the Power Wake-Up pin to disable.
 */
void XPD_PWR_WakeUpPin_Disable(uint8_t WakeUpPin)
{
    switch (WakeUpPin)
    {
        case 1:
#if defined(PWR_CSR_EWUP1)
            PWR_REG_BIT(CSR, EWUP1) = DISABLE;
#else
            PWR_REG_BIT(CSR, EWUP) = DISABLE;
#endif
            break;
#if defined(PWR_CSR_EWUP2)
        case 2:
            PWR_REG_BIT(CSR, EWUP2) = DISABLE;
            break;
#endif
#if defined(PWR_CSR_EWUP3)
        case 3:
            PWR_REG_BIT(CSR, EWUP3) = DISABLE;
            break;
#endif
    }
}

#ifdef PWR_CSR_WUPP
/**
 * @brief Sets the WakeUp Pins polarity.
 * @param RisingOrFalling: the polarity. Permitted values:
             @arg @ref EdgeType::EDGE_RISING
             @arg @ref EdgeType::EDGE_FALLING
 */
void XPD_PWR_WakeUpPin_SetPolarity(EdgeType RisingOrFalling)
{
    if (RisingOrFalling == EDGE_FALLING)
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

#ifdef PWR_CR_PLS
/** @addtogroup PWR_Voltage_Detector
 * @{ */

/** @defgroup PWR_PVD_Exported_Functions PWR PVD Exported Functions
 * @{ */

/**
 * @brief Configures the voltage threshold monitoring by the Power Voltage Detector(PVD).
 * @param Config: configuration structure that contains the monitored voltage level
 *         and the EXTI configuration.
 */
void XPD_PWR_PVD_Init(PWR_PVD_InitType * Config)
{
    /* Set PLS bits according to PVDLevel value */
    PWR->CR.b.PLS = Config->Level;

    /* External interrupt line 16 Connected to the PVD EXTI Line */
    XPD_EXTI_Init(PWR_PVD_EXTI_LINE, &Config->ExtI);
}

/**
 * @brief Enables the Power Voltage Detector(PVD).
 */
void XPD_PWR_PVD_Enable(void)
{
    PWR_REG_BIT(CR,PVDE) = 1;
}

/**
 * @brief Disables the Power Voltage Detector(PVD).
 */
void XPD_PWR_PVD_Disable(void)
{
    PWR_REG_BIT(CR,PVDE) = 0;
}

/** @} */

/** @} */

#endif /* PWR_CR_PLS */

/** @addtogroup PWR_Regulator_Voltage_Scaling
 * @{ */

/** @defgroup PWR_Regulator_Voltage_Scaling_Exported_Functions PWR Regulator Voltage Scaling Exported Functions
 * @{ */

/**
 * @brief Returns the current Regulator Voltage Scaling configuration.
 * @return The active voltage scaling
 */
PWR_RegVoltScaleType XPD_PWR_GetVoltageScale(void)
{
    return PWR->CR.b.VOS;
}

/**
 * @brief Sets the new Regulator Voltage Scaling configuration.
 * @param Scaling the new scaling value
 * @return ERROR if operation is blocked, TIMEOUT if setting fails, OK when successful
 */
XPD_ReturnType XPD_PWR_VoltageScaleConfig(PWR_RegVoltScaleType Scaling)
{
    XPD_ReturnType result;
    uint32_t timeout = PWR_VOSRDY_TIMEOUT;

#ifdef PWR_CR_VOS_1
    /* Check if the PLL is used as system clock or not */
    if (XPD_RCC_GetSYSCLKSource() == PLL)
    {
        result = XPD_ERROR;
    }
    else
#endif
    {
#ifdef PWR_CR_VOS_1
        boolean_t pll_on = RCC_REG_BIT(CR,PLLRDY);

        if (pll_on)
        {
            /* disable the main PLL. */
            RCC_REG_BIT(CR,PLLON) = OSC_OFF;

            /* wait until PLL is disabled */
            result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_PLLRDY, 0, RCC_PLL_TIMEOUT);
            if (result != XPD_OK)
            {
                return result;
            }
        }
#endif
        PWR->CR.b.VOS = Scaling;

#ifdef PWR_CR_VOS_1
        if (pll_on)
        {
            /* enable the main PLL. */
            RCC_REG_BIT(CR,PLLON) = OSC_ON;

            /* wait until PLL is enabled */
            result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_PLLRDY, RCC_CR_PLLRDY, RCC_PLL_TIMEOUT);
            if (result != XPD_OK)
            {
                return result;
            }
        }
#endif
        result = XPD_WaitForMatch(&PWR->CSR.w, PWR_CSR_VOSRDY, PWR_CSR_VOSRDY, &timeout);
    }
    return result;
}

#if defined(PWR_CR_MRLVDS) && defined(PWR_CR_LPLVDS)
/**
 * @brief Sets the low voltage mode for the selected regulator.
 * @param Regulator: the regulator to configure
 * @param NewState: the low voltage mode state to set
 */
void XPD_PWR_RegLowVoltageConfig(PWR_RegulatorType Regulator, FunctionalState NewState)
{
    if (Regulator == PWR_MAINREGULATOR)
    {
        PWR_REG_BIT(CR,MRLVDS) = NewState;
    }
    else
    {
        PWR_REG_BIT(CR,LPLVDS) = NewState;
    }
}
#endif

/** @} */

/** @} */

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
XPD_ReturnType XPD_PWR_OverDrive_Enable(void)
{
    XPD_ReturnType result;
    uint32_t timeout = PWR_OVERDRIVE_TIMEOUT;

    /* Enable the Overdrive to extend the clock frequency to 180 Mhz */
    PWR_REG_BIT(CR, ODEN) = ENABLE;

    /* Wait until Overdrive is ready */
    result = XPD_WaitForMatch(&RCC->CR.w, PWR_CSR_ODRDY, PWR_CSR_ODRDY, &timeout);

    if (result == XPD_OK)
    {
        /* Enable the Overdrive switch */
        PWR_REG_BIT(CR, ODSWEN) = ENABLE;

        /* Wait until Overdrive SW is ready */
        result = XPD_WaitForMatch(&RCC->CR.w, PWR_CSR_ODSWRDY, PWR_CSR_ODSWRDY, &timeout);
    }

    return result;
}

/**
 * @brief  Deactivates the Over-Drive mode.
 * @note   It is recommended to enter or exit Over-drive mode when the application is not running
 *         critical tasks and when the system clock source is either HSI or HSE.
 *         During the Over-drive switch activation, no peripheral clocks should be enabled.
 * @retval TIMEOUT if setting fails, OK when successful
 */
XPD_ReturnType XPD_PWR_OverDrive_Disable(void)
{
    XPD_ReturnType result;
    uint32_t timeout = PWR_OVERDRIVE_TIMEOUT;

    /* Disable the Over-drive switch */
    PWR_REG_BIT(CR, ODSWEN) = DISABLE;

    /* Wait until Overdrive SW is not ready */
    result = XPD_WaitForMatch(&RCC->CR.w, PWR_CSR_ODSWRDY, 0, &timeout);

    if (result == XPD_OK)
    {
        /* Disable the Over-drive switch */
        PWR_REG_BIT(CR, ODEN) = DISABLE;

        /* Wait until Overdrive is not ready */
        result = XPD_WaitForMatch(&RCC->CR.w, PWR_CSR_ODRDY, 0, &timeout);
    }

    return result;
}

/** @} */

/** @} */

#endif /* PWR_CR_ODEN */

/** @} */
