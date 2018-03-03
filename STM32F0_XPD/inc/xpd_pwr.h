/**
  ******************************************************************************
  * @file    xpd_pwr.h
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
#ifndef __XPD_PWR_H_
#define __XPD_PWR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup PWR
 * @{ */

/** @defgroup PWR_Core PWR Core
 * @{ */

/** @defgroup PWR_Exported_Types PWR Exported Types
 * @{ */

/** @brief PWR regulator types */
typedef enum
{
    PWR_MAINREGULATOR     = 0, /*!< Main regulator ON in Sleep/Stop mode */
    PWR_LOWPOWERREGULATOR = 1, /*!< Low Power regulator ON in Sleep/Stop mode */
}PWR_RegulatorType;

/** @} */

/** @defgroup PWR_Exported_Macros PWR Exported Macros
 * @{ */

/**
 * @brief  Get the specified PWR flag.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg WUF:         Wake up flag
 *            @arg SBF:         Standby flag
 *            @arg PVDO:        Power Voltage Detector output flag
 *            @arg VREFINTRDYF: VREFINT reference voltage ready
 */
#define         PWR_FLAG_STATUS(FLAG_NAME)      \
    (PWR_REG_BIT(CSR,FLAG_NAME))

/**
 * @brief  Clear the specified PWR flag.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg WUF:         Wake up flag
 *            @arg SBF:         Standby flag
 */
#define         PWR_FLAG_CLEAR(FLAG_NAME)       \
    (PWR_REG_BIT(CR,C##FLAG_NAME) = 1)

/** @brief PWR VDDIO2 EXTI line number */
#define PWR_VDDIO2_EXTI_LINE            31

#ifdef PWR_BB
/**
 * @brief PWR register bit accessing macro
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         PWR_REG_BIT(REG_NAME, BIT_NAME) \
    (PWR_BB->REG_NAME.BIT_NAME)
#else
/**
 * @brief PWR register bit accessing macro
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         PWR_REG_BIT(REG_NAME, BIT_NAME) \
    (PWR->REG_NAME.b.BIT_NAME)
#endif

/** @} */

/** @addtogroup PWR_Exported_Functions
 * @{ */
void            PWR_vSleepMode           (ReactionType eWakeUpOn);
void            PWR_vStopMode            (ReactionType eWakeUpOn, PWR_RegulatorType eRegulator);
void            PWR_vStandbyMode         (void);

void            PWR_vWakeUpPin_Enable    (uint8_t ucWakeUpPin);
void            PWR_vWakeUpPin_Disable   (uint8_t ucWakeUpPin);

/**
 * @brief Send Event on Pending bit enables disabled interrupts to wake up
 *        a system from WaitForEvent.
 * @param eNewState: the new SEVONPEND value to set
 */
__STATIC_INLINE void PWR_vSEVONPEND(FunctionalState eNewState)
{
    SCB->SCR.b.SEVONPEND = eNewState;
}

/**
 * @brief Sleep on Exit bit enables to enter sleep mode
 *        on return from an ISR to Thread mode.
 * @param eNewState: the new SLEEPONEXIT value to set
 */
__STATIC_INLINE void PWR_vSLEEPONEXIT(FunctionalState eNewState)
{
    SCB->SCR.b.SLEEPONEXIT = eNewState;
}

/**
 * @brief Sleep Deep bit enables to enter deep sleep mode.
 * @param eNewState: the new SLEEPONEXIT value to set
 */
__STATIC_INLINE void PWR_vSLEEPDEEP(FunctionalState eNewState)
{
    SCB->SCR.b.SLEEPDEEP = eNewState;
}

/** @} */

/** @} */

/** @defgroup PWR_Peripherals PWR Peripherals
 * @{ */

/** @defgroup PWR_Peripherals_Exported_Functions PWR Peripherals Exported Functions
 * @{ */

/**
 * @brief Enables or disables access to the backup domain
 *        (RTC registers, RTC backup data registers when present).
 * @param eNewState: the new backup access state to set
 * @note  If the HSE divided by 32 is used as the RTC clock, the
 *         Backup Domain Access should be kept enabled.
 */
__STATIC_INLINE void PWR_vBackupAccess(FunctionalState eNewState)
{
    PWR_REG_BIT(CR,DBP) = eNewState;
}

/** @} */

/** @} */

/** @} */

#ifdef PWR_CR_PLS
#include <xpd_pvd.h>
#endif /* PWR_CR_PLS */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_PWR_H_ */
