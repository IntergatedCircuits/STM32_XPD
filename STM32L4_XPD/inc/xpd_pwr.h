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
#include <xpd_flash.h>

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
 *            @arg WUF1:        Wake up flag 1
 *            @arg WUF2:        Wake up flag 2
 *            @arg WUF3:        Wake up flag 3
 *            @arg WUF4:        Wake up flag 4
 *            @arg WUF5:        Wake up flag 5
 *            @arg WUFI:        Wake-Up Flag Internal
 *            @arg SBF:         Standby Flag
 *            @arg REGLPS:      Low Power Regulator Started
 *            @arg REGLPF:      Low Power Regulator Flag
 *            @arg VOSF:        Voltage Scaling Flag
 *            @arg PVDO:        Power Voltage Detector Output flag
 *            @arg PVMO1:       Peripheral Voltage Monitoring Output 1 (VUSB vs PVM1)
 *            @arg PVMO2:       Peripheral Voltage Monitoring Output 2 (VDDIO2 vs PVM2)
 *            @arg PVMO3:       Peripheral Voltage Monitoring Output 3 (VDDA vs PVM3)
 *            @arg PVMO4:       Peripheral Voltage Monitoring Output 4 (VDDA vs PVM4)
 */
#define         PWR_FLAG_STATUS(FLAG_NAME)      \
    (PWR_REG_BIT(__XPD_PWR_##FLAG_NAME##_SR,FLAG_NAME))

/**
 * @brief  Clear the specified PWR flag.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg WUF1:        Wake up flag 1
 *            @arg WUF2:        Wake up flag 2
 *            @arg WUF3:        Wake up flag 3
 *            @arg WUF4:        Wake up flag 4
 *            @arg WUF5:        Wake up flag 5
 *            @arg SBF:         Standby flag
 */
#define         PWR_FLAG_CLEAR(FLAG_NAME)       \
    (PWR_REG_BIT(SCR,C##FLAG_NAME) = 1)

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

#define __XPD_PWR_WUF1_SR       SR1
#define __XPD_PWR_WUF2_SR       SR1
#define __XPD_PWR_WUF3_SR       SR1
#define __XPD_PWR_WUF4_SR       SR1
#define __XPD_PWR_WUF5_SR       SR1
#define __XPD_PWR_WUFI_SR       SR1
#define __XPD_PWR_SBF_SR        SR1
#define __XPD_PWR_REGLPS_SR     SR2
#define __XPD_PWR_REGLPF_SR     SR2
#define __XPD_PWR_VOSF_SR       SR2
#define __XPD_PWR_PVDO_SR       SR2
#define __XPD_PWR_PVMO1_SR      SR2
#define __XPD_PWR_PVMO2_SR      SR2
#define __XPD_PWR_PVMO3_SR      SR2
#define __XPD_PWR_PVMO4_SR      SR2

/** @} */

/** @addtogroup PWR_Exported_Functions
 * @{ */
XPD_ReturnType  PWR_eLowPowerRunMode    (FunctionalState eNewState);
void            PWR_vSleepMode          (ReactionType eWakeUpOn);
void            PWR_vStopMode           (ReactionType eWakeUpOn, PWR_RegulatorType eRegulator);
void            PWR_vStandbyMode        (void);
void            PWR_vShutdownMode       (void);

void            PWR_vWakeUpPin_Enable   (uint8_t ucWakeUpPin);
void            PWR_vWakeUpPin_Disable  (uint8_t ucWakeUpPin);
void            PWR_vWakeUpPin_Polarity (uint8_t ucWakeUpPin, EdgeType eRisingOrFalling);

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

/** @defgroup PWR_Peripherals_Exported_Types PWR Peripherals Exported Types
 * @{ */

/** @brief PWR battery charge options */
typedef enum
{
    PWR_BAT_CHARGE_NONE = 0, /*!< Battery charging is disabled (default) */
    PWR_BAT_CHARGE_5K   = 1, /*!< Battery charging on VBAT through 5 kOhm resistance from VDD */
    PWR_BAT_CHARGE_1K5  = 3, /*!< Battery charging on VBAT through 1.5 kOhm resistance from VDD */
}PWR_BatteryChargeConfigType;

/** @} */

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
    PWR_REG_BIT(CR1,DBP) = eNewState;
}

#ifdef FLASH_ACR_SLEEP_PD
/**
 * @brief Sets the Flash Power Down state in Stop mode.
 * @param eNewState: the new Flash power down state to set
 */
__STATIC_INLINE void PWR_vFlashPowerDown(FunctionalState eNewState)
{
    FLASH_REG_BIT(ACR, SLEEP_PD) = eNewState;
}
#endif

#ifdef PWR_CR3_APC
/**
 * @brief Enables or disables the Power-down GPIO pull configuration
 *        in Standby and Shutdown modes.
 * @param eNewState: Power-down GPIO pull control
 */
__STATIC_INLINE void PWR_vIOPullPowerDown(FunctionalState eNewState)
{
    PWR_REG_BIT(CR3, APC) = eNewState;
}
#endif

/**
 * @brief Configure the SRAM2 content retention (powered by the low-power regulator)
 *        in Standby mode.
 * @param eNewState: ENABLE -> SRAM2 content is kept in Standby
 */
__STATIC_INLINE void PWR_vSRAM2PowerDown(FunctionalState eNewState)
{
    PWR_REG_BIT(CR3, RRS) = eNewState;
}

/**
 * @brief Set up battery charging via VBAT through internal resistor when VDD is present.
 * @param eConfig: specifies the charging and the resistor impedance.
 */
__STATIC_INLINE void PWR_vBatteryChargeConfig(PWR_BatteryChargeConfigType eConfig)
{
    /* Specify resistor selection */
    MODIFY_REG(PWR->CR4.w, PWR_CR4_VBRS | PWR_CR4_VBE, eConfig << PWR_CR4_VBE_Pos);
}

#ifdef PWR_CR2_USV
/**
 * @brief Enable or disable VDDUSB supply.
 *        Remove VDDUSB electrical and logical isolation, once VDDUSB supply is present.
 * @param eNewState: New VDDUSB supply setting
 */
__STATIC_INLINE void PWR_vVddUSB(FunctionalState eNewState)
{
    PWR_REG_BIT(CR2, USV) = eNewState;
}
#endif

#ifdef PWR_CR2_IOSV
/**
 * @brief Enable or disable VDDIO2 supply.
 *        Remove VDDIO2 electrical and logical isolation, once VDDIO2 supply is present.
 * @param eNewState: New VDDIO2 supply setting
 */
__STATIC_INLINE void PWR_vVddIO2(FunctionalState eNewState)
{
    PWR_REG_BIT(CR2, IOSV) = eNewState;
}
#endif /* PWR_CR2_IOSV */

/**
 * @brief Enable or disable the Internal Wake-up Line.
 * @param eNewState: New internal wake-up line setting
 */
__STATIC_INLINE void PWR_vInternalWakeUpLine(FunctionalState eNewState)
{
    PWR_REG_BIT(CR3, EIWUL) = eNewState;
}

/** @} */

/** @} */

#ifdef PWR_CR1_VOS
/** @defgroup PWR_Regulator_Voltage_Scaling PWR Regulator Voltage Scaling
 * @{ */

/** @defgroup PWR_Regulator_Voltage_Scaling_Exported_Types
 * PWR Regulator Voltage Scaling Exported Types
 * @{ */

/** @brief Regulator Voltage Scaling modes */
typedef enum
{
    PWR_REGVOLT_SCALE1 = 1, /*!< Scale 1 mode(default value at reset): the maximum value of fHCLK = 80 MHz. */
    PWR_REGVOLT_SCALE2 = 2  /*!< Scale 2 mode: the maximum value of fHCLK = 26 MHz. */
}PWR_RegVoltScaleType;

/** @} */

/** @addtogroup PWR_Regulator_Voltage_Scaling_Exported_Functions
 * @{ */
XPD_ReturnType  PWR_eVoltageScaleConfig     (PWR_RegVoltScaleType eScaling);

/**
 * @brief Returns the current Regulator Voltage Scaling configuration.
 * @return The active voltage scaling
 */
__STATIC_INLINE PWR_RegVoltScaleType PWR_eGetVoltageScale(void)
{
    return PWR->CR1.b.VOS;
}

/** @} */

/** @} */
#endif /* PWR_CR1_VOS */

#ifdef PWR_CR2_PVME
/** @defgroup PWR_Peripheral_Voltage_Monitoring
 * PWR Peripheral Voltage Monitoring
 * @{ */

/** @defgroup PWR_Peripheral_Voltage_Monitoring_Exported_Types
 * PWR Peripheral Voltage Monitoring Exported Types
 * @{ */

/** @brief Peripheral Voltage Monitoring selection */
typedef enum
{
#ifdef PWR_CR2_PVME1
    PWR_PVM1_VDDUSB_1V2 = PWR_CR2_PVME1, /*!< VddUSB vs 1.2V Peripheral Voltage Monitor */
#endif
#ifdef PWR_CR2_PVME2
    PWR_PVM2_VDDIO2_0V9 = PWR_CR2_PVME2, /*!< VddIO2 vs 0.9V Peripheral Voltage Monitor */
#endif
    PWR_PVM3_VDDA_1V62  = PWR_CR2_PVME3, /*!< VddA vs 1.62V Peripheral Voltage Monitor */
    PWR_PVM4_VDDA_2V2   = PWR_CR2_PVME4, /*!< VddA vs 2.2V Peripheral Voltage Monitor */
}PWR_PVMType;

/** @} */

/** @defgroup PWR_Peripheral_Voltage_Monitoring_Exported_Macros
 * PWR Peripheral Voltage Monitoring Exported Macros
 * @{ */

/** @brief PVM1 EXTI line number */
#define PWR_PVM1_EXTI_LINE              35

/** @brief PVM2 EXTI line number */
#define PWR_PVM2_EXTI_LINE              36

/** @brief PVM3 EXTI line number */
#define PWR_PVM3_EXTI_LINE              37

/** @brief PVM4 EXTI line number */
#define PWR_PVM4_EXTI_LINE              38

/** @} */

/** @defgroup PWR_Peripheral_Voltage_Monitoring_Exported_Functions
 * PWR Peripheral Voltage Monitoring Exported Functions
 * @{ */

/**
 * @brief Enables the selected Power Voltage Monitors.
 * @param pvm: the list of monitors to enable
 */
__STATIC_INLINE void PWR_vPVM_Enable(PWR_PVMType ePVM)
{
    SET_BIT(PWR->CR2.w, ePVM);
}

/**
 * @brief Disables the selected Power Voltage Monitors.
 * @param ePVM: the list of monitors to disable
 */
__STATIC_INLINE void PWR_vPVM_Disable(PWR_PVMType ePVM)
{
    CLEAR_BIT(PWR->CR2.w, ePVM);
}

/** @} */

/** @} */
#endif /* PWR_CR2_PVME */

/** @} */

#ifdef PWR_CR2_PLS
#include <xpd_pvd.h>
#endif /* PWR_CR_PLS */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_PWR_H_ */
