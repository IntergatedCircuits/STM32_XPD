/**
  ******************************************************************************
  * @file    xpd_adc_calc.h
  * @author  Benedek Kupper
  * @version 1.0
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers ADC Calculations Module
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
#ifndef __XPD_ADC_CALC_H_
#define __XPD_ADC_CALC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @ingroup ADC
 * @defgroup ADC_Calculations ADC Calculations
 * @{ */

/** @addtogroup ADC_Calculations_Exported_Functions
 * @{ */
int32_t         ADC_lCalcVDDA_mV        (uint16_t usVRefintConversion);
int32_t         ADC_lCalcExt_mV         (uint16_t usChannelConversion);
int32_t         ADC_lCalcTemp_C         (uint16_t usTempConversion);
#ifdef ADC_VBAT_SCALER
int32_t         ADC_lCalcVBAT_mV        (uint16_t usVBatConversion);
#endif
int32_t         ADC_lGetVDDA_mV         (void);

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
float           ADC_fCalcVDDA_V         (uint16_t usVRefintConversion);
float           ADC_fCalcExt_V          (uint16_t usChannelConversion);
float           ADC_fCalcTemp_C         (uint16_t usTempConversion);
#ifdef ADC_VBAT_SCALER
float           ADC_fCalcVBAT_V         (uint16_t usVBatConversion);
#endif
float           ADC_fGetVDDA_V          (void);
#endif

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_ADC_CALC_H_ */
