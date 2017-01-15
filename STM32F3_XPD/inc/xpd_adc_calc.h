/**
  ******************************************************************************
  * @file    xpd_adc_calc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-01-06
  * @brief   STM32 eXtensible Peripheral Drivers ADC Calculations Module
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
#ifndef XPD_ADC_CALC_H_
#define XPD_ADC_CALC_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @addtogroup ADC
 * @{ */

/** @defgroup ADC_Calculations ADC Calculations
 * @{ */

/** @addtogroup ADC_Calculations_Exported_Functions
 * @{ */
int32_t         XPD_ADC_SetVDDA             (uint16_t vRefintConversion);

int32_t         XPD_ADC_GetValue_mV         (uint16_t channelConversion);
int32_t         XPD_ADC_GetVDDA_mV          (void);
int32_t         XPD_ADC_GetVBAT_mV          (uint16_t vBatConversion);
int32_t         XPD_ADC_GetTemperature      (uint16_t tempConversion);

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
float           XPD_ADC_GetValue_V          (uint16_t channelConversion);
float           XPD_ADC_GetVDDA_V           (void);
float           XPD_ADC_GetVBAT_V           (uint16_t vBatConversion);
float           XPD_ADC_GetTemperature_C    (uint16_t tempConversion);
#endif
/** @} */

/** @} */

/** @} */

#endif /* XPD_ADC_CALC_H_ */
