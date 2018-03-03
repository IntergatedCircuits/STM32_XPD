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
#include <xpd_flash.h>
#include <xpd_rcc.h>
#include <xpd_utils.h>

/** @addtogroup PWR
 * @{ */

#define PWR_BKPREG_TIMEOUT          1000
#define PWR_VOSRDY_TIMEOUT          1000
#define PWR_OVERDRIVE_TIMEOUT       1000
#define PWR_UNDERDRIVE_TIMEOUT      1000
#define PWR_FLAG_SETTING_DELAY_us   50

/** @addtogroup PWR_Core
 * @{ */

/** @defgroup PWR_Exported_Functions PWR Exported Functions
 * @{ */

/**
 * @brief Activates or leaves Low-power Run mode.
 * @param eNewState: the new state for Low-power Run mode
 * @note  In Low-power Run mode, all I/O pins keep the same state as in Run mode.
 * @note  The clock frequency must be reduced below 2 MHz before enabling Low-power Run mode.
 * @retval TIMEOUT if Low-power run mode was not left successfully, otherwise OK
 */
XPD_ReturnType PWR_eLowPowerRunMode(FunctionalState eNewState)
{
    XPD_ReturnType eResult = XPD_OK;

    /* Set Regulator parameter */
    PWR_REG_BIT(CR1, LPR) = eNewState;

    if (eNewState == DISABLE)
    {
        XPD_vDelay_us(PWR_FLAG_SETTING_DELAY_us);

        /* If the flag didn't clear after delay, low power run mode was not left */
        if (PWR_FLAG_STATUS(REGLPF) != 0)
        {
            eResult = XPD_TIMEOUT;
        }
    }

    return eResult;
}

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
 *         the HSI RC oscillator is selected as system clock if STOPWUCK bit
 *         in RCC_CFGR register is set, otherwise the MSI oscillator is selected.
 * @note  When the voltage regulator operates in low power mode (Stop 1), an additional
 *         startup delay is incurred when waking up from Stop mode.
 *         By keeping the internal regulator ON during Stop mode (Stop 0), the consumption
 *         is higher although the startup time is reduced.
 *         Set the voltage regulator in low-power mode and disable
 *         Low-power Run to enter Stop 2 mode.
 * @param eWakeUpOn: Specifies if STOP mode is exited with WFI or WFE instruction
 *           This parameter can be one of the following values:
 *            @arg REACTION_IT: enter SLEEP mode with WFI instruction
 *            @arg REACTION_EVENT: enter SLEEP mode with WFE instruction
 * @param eRegulator: Specifies the regulator state in STOP mode
 */
void PWR_vStopMode(ReactionType eWakeUpOn, PWR_RegulatorType eRegulator)
{
    /* Using the main regulator the Stop 0 mode is entered */
    if (eRegulator == PWR_MAINREGULATOR)
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
 * @note  In Standby mode, the PLL, the HSI, the MSI and the HSE oscillators are switched
 *        off. The voltage regulator is disabled, except when SRAM2 content is preserved
 *        in which case the regulator is in low-power mode.
 *        SRAM1 and register contents are lost except for registers in the Backup domain and
 *        Standby circuitry. SRAM2 content can be preserved
 *        if the bit RRS is set in PWR_CR3 register.
 *        The BOR is available.
 * @note  The I/Os can be configured either with a pull-up or pull-down
 *        or can be kept in analog state.
 */
void PWR_vStandbyMode(void)
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
 * @note  In Shutdown mode, the PLL, the HSI, the MSI, the LSI and the HSE oscillators
 *        are switched off. The voltage regulator is disabled and Vcore domain is powered off.
 *        SRAM1, SRAM2 and registers contents are lost except for registers in the Backup domain.
 *        The BOR is not available.
 */
void PWR_vShutdownMode(void)
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
 * @param ucWakeUpPin: Specifies the Power Wake-Up pin to enable.
 */
void PWR_vWakeUpPin_Enable(uint8_t ucWakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CR3.EWUP1 + ucWakeUpPin - 1) = ENABLE;
#else
    SET_BIT(PWR->CR3.w, (PWR_CR3_EWUP1 << (ucWakeUpPin - 1)));
#endif
}

/**
 * @brief Disables the WakeUp PINx functionality.
 * @param ucWakeUpPin: Specifies the Power Wake-Up pin to disable.
 */
void PWR_vWakeUpPin_Disable(uint8_t ucWakeUpPin)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CR3.EWUP1 + ucWakeUpPin - 1) = DISABLE;
#else
    CLEAR_BIT(PWR->CR3.w, (PWR_CR3_EWUP1 << (ucWakeUpPin - 1)));
#endif
}

/**
 * @brief Sets the WakeUp Pins polarity.
 * @param ucWakeUpPin: Specifies the Power Wake-Up pin to configure.
 * @param eRisingOrFalling: the polarity. Permitted values:
             @arg @ref EdgeType::EDGE_RISING
             @arg @ref EdgeType::EDGE_FALLING
 */
void PWR_vWakeUpPin_SetPolarity(uint8_t ucWakeUpPin, EdgeType eRisingOrFalling)
{
#ifdef PWR_BB
    *(__IO uint32_t *)(&PWR_BB->CR4.WP1 + ucWakeUpPin - 1) =
            (eRisingOrFalling == EDGE_FALLING) ? 1 : 0;
#else
    if (eRisingOrFalling == EDGE_FALLING)
    {
        SET_BIT(PWR->CR4.w, PWR_CR4_WP1 << (ucWakeUpPin - 1));
    }
    else
    {
        CLEAR_BIT(PWR->CR4.w, PWR_CR4_WP1 << (ucWakeUpPin - 1));
    }
#endif
}

/** @} */

/** @} */

#ifdef PWR_CR1_VOS
/** @addtogroup PWR_Regulator_Voltage_Scaling
 * @{ */

/** @defgroup PWR_Regulator_Voltage_Scaling_Exported_Functions
 * PWR Regulator Voltage Scaling Exported Functions
 * @{ */

/**
 * @brief Sets the new Regulator Voltage Scaling configuration.
 * @param eScaling the new scaling value
 * @return ERROR if operation is blocked, TIMEOUT if setting fails, OK when successful
 */
XPD_ReturnType PWR_eVoltageScaleConfig(PWR_RegVoltScaleType eScaling)
{
    XPD_ReturnType eResult = XPD_OK;

    if (PWR->CR1.b.VOS != eScaling)
    {
        PWR->CR1.b.VOS = eScaling;

        /* If Set Range 1 */
        if (eScaling == PWR_REGVOLT_SCALE1)
        {
            XPD_vDelay_us(PWR_FLAG_SETTING_DELAY_us);

            /* Wait until VOSF is cleared */
            if (PWR_FLAG_STATUS(VOSF) != 0)
            {
                eResult = XPD_TIMEOUT;
            }
        }
    }
    return eResult;
}

/** @} */

/** @} */
#endif /* PWR_CR1_VOS */

/** @} */
