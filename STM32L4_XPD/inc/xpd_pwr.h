/**
  ******************************************************************************
  * @file    xpd_pwr.h
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
#ifndef __XPD_PWR_H_
#define __XPD_PWR_H_

#include "xpd_common.h"
#include "xpd_config.h"
#include "xpd_flash.h"

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
#define         XPD_PWR_GetFlag(FLAG_NAME)      \
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
#define         XPD_PWR_ClearFlag(FLAG_NAME)    \
    (PWR_REG_BIT(SCR,C##FLAG_NAME) = 1)

#ifdef PWR_BB
#define PWR_REG_BIT(_REG_NAME_, _BIT_NAME_) (PWR_BB->_REG_NAME_._BIT_NAME_)
#else
#define PWR_REG_BIT(_REG_NAME_, _BIT_NAME_) (PWR->_REG_NAME_.b._BIT_NAME_)
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

/** @addtogroup PWR_Exported_Functions PWR Exported Functions
 * @{ */
XPD_ReturnType  XPD_PWR_LowPowerRunMode     (FunctionalState NewState);
void            XPD_PWR_SleepMode           (ReactionType WakeUpOn);
void            XPD_PWR_StopMode            (ReactionType WakeUpOn, PWR_RegulatorType Regulator);
void            XPD_PWR_StandbyMode         (void);
void            XPD_PWR_ShutdownMode        (void);

void            XPD_PWR_WakeUpPin_Enable    (uint8_t WakeUpPin);
void            XPD_PWR_WakeUpPin_Disable   (uint8_t WakeUpPin);
void            XPD_PWR_WakeUpPin_Polarity  (EdgeType RisingOrFalling);
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
 * @param NewState: the new backup access state to set
 * @note  If the HSE divided by 32 is used as the RTC clock, the
 *         Backup Domain Access should be kept enabled.
 */
__STATIC_INLINE void XPD_PWR_BackupAccess(FunctionalState NewState)
{
    PWR_REG_BIT(CR1,DBP) = NewState;
}

#ifdef FLASH_ACR_SLEEP_PD
/**
 * @brief Sets the Flash Power Down state in Stop mode.
 * @param NewState: the new Flash power down state to set
 */
__STATIC_INLINE void XPD_PWR_FlashPowerDown(FunctionalState NewState)
{
    FLASH_REG_BIT(ACR, SLEEP_PD) = NewState;
}
#endif

#ifdef PWR_CR3_APC
/**
 * @brief Enables or disables the Power-down GPIO pull configuration in Standby and Shutdown modes.
 * @param NewState: Power-down GPIO pull control
 */
__STATIC_INLINE void XPD_PWR_IOPullPowerDown(FunctionalState NewState)
{
    PWR_REG_BIT(CR3, APC) = NewState;
}
#endif

/**
 * @brief Configure the SRAM2 content retention (powered by the low-power regulator) in Standby mode.
 * @param NewState: ENABLE -> SRAM2 content is kept in Standby
 */
__STATIC_INLINE void XPD_PWR_SRAM2PowerDown(FunctionalState NewState)
{
    PWR_REG_BIT(CR3, RRS) = NewState;
}

/**
 * @brief Set up battery charging via VBAT through internal resistor when VDD is present.
 * @param Config: specifies the charging and the resistor impedance.
 */
__STATIC_INLINE void XPD_PWR_BatteryChargeConfig(PWR_BatteryChargeConfigType Config)
{
    /* Specify resistor selection */
    MODIFY_REG(PWR->CR4.w, PWR_CR4_VBRS | PWR_CR4_VBE, Config << PWR_CR4_VBE_Pos);
}

#ifdef PWR_CR2_USV
/**
 * @brief Enable or disable VDDUSB supply. Remove VDDUSB electrical and logical isolation, once VDDUSB supply is present.
 * @param NewState: New VDDUSB supply setting
 */
__STATIC_INLINE void XPD_PWR_VddUSB(FunctionalState NewState)
{
    PWR_REG_BIT(CR2, USV) = NewState;
}
#endif

#ifdef PWR_CR2_IOSV
/**
 * @brief Enable or disable VDDIO2 supply. Remove VDDIO2 electrical and logical isolation, once VDDIO2 supply is present.
 * @param NewState: New VDDIO2 supply setting
 */
__STATIC_INLINE void XPD_PWR_VddIO2(FunctionalState NewState)
{
    PWR_REG_BIT(CR2, IOSV) = NewState;
}
#endif /* PWR_CR2_IOSV */

/**
 * @brief Enable or disable the Internal Wake-up Line.
 * @param NewState: New internal wake-up line setting
 */
__STATIC_INLINE void XPD_PWR_InternalWakeUpLine(FunctionalState NewState)
{
    PWR_REG_BIT(CR3, EIWF) = NewState;
}

/** @} */

/** @} */

#ifdef PWR_CR2_PLS
#include "xpd_exti.h"

/** @defgroup PWR_Voltage_Detector PWR Voltage Detector
 * @{ */

/** @defgroup PWR_PVD_Exported_Types PWR PVD Exported Types
 * @{ */

/** @brief PVD levels */
typedef enum
{
    PWR_PVDLEVEL_2V0 = 0, /*!< 2.0V voltage detector level */
    PWR_PVDLEVEL_2V2 = 1, /*!< 2.2V voltage detector level */
    PWR_PVDLEVEL_2V4 = 2, /*!< 2.4V voltage detector level */
    PWR_PVDLEVEL_2V5 = 3, /*!< 2.5V voltage detector level */
    PWR_PVDLEVEL_2V6 = 4, /*!< 2.6V voltage detector level */
    PWR_PVDLEVEL_2V8 = 5, /*!< 2.8V voltage detector level */
    PWR_PVDLEVEL_2V9 = 6, /*!< 2.9V voltage detector level */
    PWR_PVDLEVEL_EXT = 7  /*!< External input analog voltage PVD_IN (compared to VREFINT) */
} PWR_PVDLevelType;

/** @brief PVD configuration structure */
typedef struct
{
    PWR_PVDLevelType Level; /*!< Voltage detector level to trigger reaction */
    EXTI_InitType    ExtI;  /*!< External interrupt configuration */
} PWR_PVD_InitType;

/** @} */

/** @defgroup PWR_PVD_Exported_Macros PWR PVD Exported Macros
 * @{ */

/** @brief PVD EXTI line number */
#define PWR_PVD_EXTI_LINE               16
/** @} */

/** @defgroup PWR_PVD_Exported_Functions PWR PVD Exported Functions
 * @{ */

/**
 * @brief Configures the voltage threshold monitoring by the Power Voltage Detector(PVD).
 * @param Config: configuration structure that contains the monitored voltage level
 *         and the EXTI configuration.
 */
__STATIC_INLINE void XPD_PWR_PVD_Init(const PWR_PVD_InitType * Config)
{
    /* Set PLS bits according to PVDLevel value */
    PWR->CR2.b.PLS = Config->Level;

    /* External interrupt line 16 Connected to the PVD EXTI Line */
    XPD_EXTI_Init(PWR_PVD_EXTI_LINE, &Config->ExtI);
}

/**
 * @brief Enables the Power Voltage Detector (PVD).
 */
__STATIC_INLINE void XPD_PWR_PVD_Enable(void)
{
    PWR_REG_BIT(CR2,PVDE) = 1;
}

/**
 * @brief Disables the Power Voltage Detector (PVD).
 */
__STATIC_INLINE void XPD_PWR_PVD_Disable(void)
{
    PWR_REG_BIT(CR2,PVDE) = 0;
}

/** @} */

/** @} */
#endif /* PWR_CR2_PLS */

#ifdef PWR_CR1_VOS
/** @defgroup PWR_Regulator_Voltage_Scaling PWR Regulator Voltage Scaling
 * @{ */

/** @defgroup PWR_Regulator_Voltage_Scaling_Exported_Types PWR Regulator Voltage Scaling Exported Types
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
XPD_ReturnType  XPD_PWR_VoltageScaleConfig  (PWR_RegVoltScaleType Scaling);

/**
 * @brief Returns the current Regulator Voltage Scaling configuration.
 * @return The active voltage scaling
 */
__STATIC_INLINE PWR_RegVoltScaleType XPD_PWR_GetVoltageScale(void)
{
    return PWR->CR1.b.VOS;
}

/** @} */

/** @} */
#endif /* PWR_CR1_VOS */

#ifdef PWR_CR2_PVME
/** @defgroup PWR_Peripheral_Voltage_Monitoring PWR Peripheral Voltage Monitoring
 * @{ */

/** @defgroup PWR_Peripheral_Voltage_Monitoring_Exported_Types PWR Peripheral Voltage Monitoring Exported Types
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

/** @defgroup PWR_Peripheral_Voltage_Monitoring_Exported_Macros PWR Peripheral Voltage Monitoring Exported Macros
 * @{ */

#ifdef PWR_CR2_PVME1
/** @brief PVM1 EXTI line number */
#define PWR_PVM1_EXTI_LINE              35
#endif

#ifdef PWR_CR2_PVME2
/** @brief PVM2 EXTI line number */
#define PWR_PVM2_EXTI_LINE              36
#endif

/** @brief PVM3 EXTI line number */
#define PWR_PVM3_EXTI_LINE              37

/** @brief PVM4 EXTI line number */
#define PWR_PVM4_EXTI_LINE              38

/** @} */

/** @defgroup PWR_Peripheral_Voltage_Monitoring_Exported_Functions PWR Peripheral Voltage Monitoring Exported Functions
 * @{ */

/**
 * @brief Enables the selected Power Voltage Monitors.
 * @param pvm: the list of monitors to enable
 */
__STATIC_INLINE void XPD_PWR_PVM_Enable(PWR_PVMType pvm)
{
    SET_BIT(PWR->CR2.w, pvm);
}

/**
 * @brief Disables the selected Power Voltage Monitors.
 * @param pvm: the list of monitors to disable
 */
__STATIC_INLINE void XPD_PWR_PVM_Disable(PWR_PVMType pvm)
{
    CLEAR_BIT(PWR->CR2.w, pvm);
}

/** @} */

/** @} */
#endif /* PWR_CR2_PVME */

/** @} */

#endif /* __XPD_PWR_H_ */
