/**
  ******************************************************************************
  * @file    xpd_pwr.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-04-10
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
#include "xpd_flash.h"
#include "xpd_rcc.h"
#include "xpd_utils.h"

/** @addtogroup PWR
 * @{ */

#define PWR_BKPREG_TIMEOUT          1000
#define PWR_VOSRDY_TIMEOUT          1000
#define PWR_OVERDRIVE_TIMEOUT       1000
#define PWR_UNDERDRIVE_TIMEOUT      1000
#define PWR_FLAG_SETTING_DELAY_US   50

/** @addtogroup PWR_Core
 * @{ */

/** @defgroup PWR_Exported_Functions PWR Exported Functions
 * @{ */

/**
 * @brief Activates or leaves Low-power Run mode.
 * @param NewState: the new state for Low-power Run mode
 * @note  In Low-power Run mode, all I/O pins keep the same state as in Run mode.
 * @note  The clock frequency must be reduced below 2 MHz before enabling Low-power Run mode.
 * @retval TIMEOUT if Low-power run mode was not left successfully, otherwise OK
 */
XPD_ReturnType XPD_PWR_LowPowerRunMode(FunctionalState NewState)
{
    XPD_ReturnType result = XPD_OK;

    /* Set Regulator parameter */
    PWR_REG_BIT(CR1, LPR) = NewState;

    if (NewState == DISABLE)
    {
        XPD_Delay_us(PWR_FLAG_SETTING_DELAY_US);

        /* If the flag didn't clear after delay, low power run mode was not left */
        if (XPD_PWR_GetFlag(REGLPF) != 0)
        {
            result = XPD_TIMEOUT;
        }
    }

    return result;
}

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
 *         the HSI RC oscillator is selected as system clock if STOPWUCK bit in RCC_CFGR register
 *         is set, otherwise the MSI oscillator is selected.
 * @note  When the voltage regulator operates in low power mode (Stop 1), an additional
 *         startup delay is incurred when waking up from Stop mode.
 *         By keeping the internal regulator ON during Stop mode (Stop 0), the consumption
 *         is higher although the startup time is reduced.
 *         Set the voltage regulator in low-power mode and disable Low-power Run to enter Stop 2 mode.
 * @param WakeUpOn: Specifies if STOP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 * @param Regulator: Specifies the regulator state in STOP mode
 */
void XPD_PWR_StopMode(ReactionType WakeUpOn, PWR_RegulatorType Regulator)
{
    /* Using the main regulator the Stop 0 mode is entered */
    if (Regulator == PWR_MAINREGULATOR)
    {
        PWR->CR1.b.LPMS = 0;
    }
    /* If Low-power Run is enabled, Stop 1 mode can be entered only */
    else if (PWR_REG_BIT(CR1, LPR) != 0)
    {
        PWR->CR1.b.LPMS = 1;
    }
    else
    {
        /* Stop 2 mode */
        PWR->CR1.b.LPMS = 2;
    }

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
 * @note  In Standby mode, the PLL, the HSI, the MSI and the HSE oscillators are switched
 *        off. The voltage regulator is disabled, except when SRAM2 content is preserved
 *        in which case the regulator is in low-power mode.
 *        SRAM1 and register contents are lost except for registers in the Backup domain and
 *        Standby circuitry. SRAM2 content can be preserved if the bit RRS is set in PWR_CR3 register.
 *        The BOR is available.
 * @note  The I/Os can be configured either with a pull-up or pull-down or can be kept in analog state.
 */
void XPD_PWR_StandbyMode(void)
{
    /* Select STANDBY mode */
    PWR->CR1.b.LPMS = 3;

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
 * @brief Enter Shutdown mode.
 * @note  In Shutdown mode, the PLL, the HSI, the MSI, the LSI and the HSE oscillators are switched
 *        off. The voltage regulator is disabled and Vcore domain is powered off.
 *        SRAM1, SRAM2 and registers contents are lost except for registers in the Backup domain.
 *        The BOR is not available.
 */
void XPD_PWR_ShutdownMode(void)
{
    /* Select SHUTDOWN mode */
    PWR->CR1.b.LPMS = 4;

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
 */
void XPD_PWR_WakeUpPin_Enable(uint8_t WakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CR3.EWUP1 + WakeUpPin - 1) = ENABLE;
#else
    SET_BIT(PWR->CR3.w, (PWR_CR3_EWUP1 << (WakeUpPin - 1)));
#endif
}

/**
 * @brief Disables the WakeUp PINx functionality.
 * @param WakeUpPin: Specifies the Power Wake-Up pin to disable.
 */
void XPD_PWR_WakeUpPin_Disable(uint8_t WakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CR3.EWUP1 + WakeUpPin - 1) = DISABLE;
#else
    CLEAR_BIT(PWR->CR3.w, (PWR_CR3_EWUP1 << (WakeUpPin - 1)));
#endif
}

/**
 * @brief Sets the WakeUp Pins polarity.
 * @param WakeUpPin: Specifies the Power Wake-Up pin to configure.
 * @param RisingOrFalling: the polarity. Permitted values:
             @arg @ref EdgeType::EDGE_RISING
             @arg @ref EdgeType::EDGE_FALLING
 */
void XPD_PWR_WakeUpPin_SetPolarity(uint8_t WakeUpPin, EdgeType RisingOrFalling)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CR4.WP1 + WakeUpPin - 1) = (uint32_t)(RisingOrFalling == EDGE_FALLING);
#else
    if (RisingOrFalling == EDGE_FALLING)
    {
        SET_BIT(PWR->CR4.w, PWR_CR4_WP1 << (WakeUpPin - 1));
    }
    else
    {
        CLEAR_BIT(PWR->CR4.w, PWR_CR4_WP1 << (WakeUpPin - 1));
    }
#endif
}

/** @} */

/** @} */

#ifdef PWR_CR1_VOS
/** @addtogroup PWR_Regulator_Voltage_Scaling
 * @{ */

/** @defgroup PWR_Regulator_Voltage_Scaling_Exported_Functions PWR Regulator Voltage Scaling Exported Functions
 * @{ */

/**
 * @brief Sets the new Regulator Voltage Scaling configuration.
 * @param Scaling the new scaling value
 * @return ERROR if operation is blocked, TIMEOUT if setting fails, OK when successful
 */
XPD_ReturnType XPD_PWR_VoltageScaleConfig(PWR_RegVoltScaleType Scaling)
{
    XPD_ReturnType result = XPD_OK;

    if (PWR->CR1.b.VOS != Scaling)
    {
        PWR->CR1.b.VOS = Scaling;

        /* If Set Range 1 */
        if (Scaling == PWR_REGVOLT_SCALE1)
        {
            XPD_Delay_us(PWR_FLAG_SETTING_DELAY_US);

            /* Wait until VOSF is cleared */
            if (XPD_PWR_GetFlag(VOSF) != 0)
            {
                result = XPD_TIMEOUT;
            }
        }
    }
    return result;
}

/** @} */

/** @} */
#endif /* PWR_CR1_VOS */

/** @} */
