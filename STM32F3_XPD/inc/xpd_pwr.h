/**
  ******************************************************************************
  * @file    xpd_pwr.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-11-01
  * @brief   STM32 eXtensible Peripheral Drivers Power Module Header file.
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
#ifndef XPD_PWR_H_
#define XPD_PWR_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup PWR
 * @{ */

/** @defgroup PWR_Core PWR Core
 * @{ */

/** @defgroup PWR_Exported_Types PWR Exported Types
 * @{ */

/** @brief PWR regulator types */
typedef enum
{
    PWR_MAINREGULATOR     = 0,
    PWR_LOWPOWERREGULATOR = 1
}PWR_RegulatorType;

/** @} */

/** @defgroup PWR_Exported_Macros PWR Exported Macros
 * @{ */

#ifdef PWR_BB
#define PWR_REG_BIT(_REG_NAME_, _BIT_NAME_) (PWR_BB->_REG_NAME_._BIT_NAME_)
#else
#define PWR_REG_BIT(_REG_NAME_, _BIT_NAME_) (PWR->_REG_NAME_.b._BIT_NAME_)
#endif

/** @} */

/** @addtogroup PWR_Exported_Functions PWR Exported Functions
 * @{ */
void        XPD_PWR_EnterStop           (ReactionType WakeUpOn, PWR_RegulatorType Regulator);
void        XPD_PWR_EnterSleep          (ReactionType WakeUpOn);
void        XPD_PWR_EnterStandby        (void);

void        XPD_PWR_LockBackup          (void);
void        XPD_PWR_UnlockBackup        (void);

void        XPD_PWR_EnableWakeUpPin     (uint8_t WakeUpPin);
void        XPD_PWR_DisableWakeUpPin    (uint8_t WakeUpPin);
/** @} */

/** @} */

#ifdef PWR_CR_PLS
#include "xpd_exti.h"

/** @defgroup PWR_Voltage_Detector PWR Voltage Detector
 * @{ */

/** @defgroup PWR_PVD_Exported_Types PWR PVD Exported Types
 * @{ */

/** @brief PVD levels */
typedef enum
{
    PWR_PVDLEVEL_2V2 = 0,
    PWR_PVDLEVEL_2V3 = 1,
    PWR_PVDLEVEL_2V4 = 2,
    PWR_PVDLEVEL_2V5 = 3,
    PWR_PVDLEVEL_2V6 = 4,
    PWR_PVDLEVEL_2V7 = 5,
    PWR_PVDLEVEL_2V8 = 6,
    PWR_PVDLEVEL_2V9 = 7
} PWR_PVDLevelType;

/** @brief PVD configuration structure */
typedef struct
{
    PWR_PVDLevelType Level;
    EXTI_InitType    ExtI;
} PWR_PVDInitType;

/** @} */

/** @defgroup PWR_PVD_Exported_Macros PWR PVD Exported Macros
 * @{ */
#define PWR_PVD_EXTI_LINE               16
/** @} */

/** @addtogroup PWR_PVD_Exported_Functions
 * @{ */
void        XPD_PWR_InitPVD             (PWR_PVDInitType * Config);
void        XPD_PWR_EnablePVD           (void);
void        XPD_PWR_DisablePVD          (void);
/** @} */

/** @} */
#endif /* PWR_CR_PLS */

/** @} */

#endif /* XPD_PWR_H_ */
