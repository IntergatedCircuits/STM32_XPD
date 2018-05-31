/**
  ******************************************************************************
  * @file    xpd_adc_calc.c
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

#include <xpd_adc_calc.h>

#ifndef VDDA_VALUE_mV
#define VDDA_VALUE_mV       (VDD_VALUE_mV)
#endif

/** @ingroup ADC_Calculations
 * @defgroup ADC_Calculations_Exported_Functions ADC Calculations Exported Functions
 * @{ */

static int32_t lVDDA_mV = (int32_t)VDDA_VALUE_mV;

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
static float   fVDDA_V  = ((float)VDDA_VALUE_mV) / 1000.0;

/**
 * @brief Returns the VDDA value in voltage.
 * @return The VDDA voltage measured in Volts
 */
float ADC_fGetVDDA_V(void)
{
    return fVDDA_V;
}

#ifdef ADC_VBAT_SCALER
/**
 * @brief Converts the VBAT measurement to voltage.
 * @param usVBatConversion: 12 bit right aligned ADC measurement value
 * @return The VBAT value in Volts
 */
float ADC_fCalcVBAT_V(uint16_t usVBatConversion)
{
    /* Convert to V from ADC range, multiply by scaler */
    return (ADC_VBAT_SCALER * (float)usVBatConversion * fVDDA_V) / 4095.0;
}
#endif

/**
 * @brief Converts a channel measurement to voltage.
 * @param usChannelConversion: 12 bit right aligned ADC measurement value
 * @return The channel voltage level
 */
float ADC_fCalcExt_V(uint16_t usChannelConversion)
{
    /* Convert to V from ADC range */
    return ((float)usChannelConversion * fVDDA_V) / 4095.0;
}

/**
 * @brief Converts a temperature sensor measurement to degree Celsius.
 * @param usTempConversion: 12 bit right aligned ADC measurement value
 * @return The temperature sensor's value in degree Celcius
 */
float ADC_fCalcTemp_C(uint16_t usTempConversion)
{
    float temp;
#if defined(ADC_TEMP_CAL_HIGH_C)
    float tempdiff = (float)(ADC_TEMP_CAL_HIGH_C - ADC_TEMP_CAL_LOW_C);
    /*
     * Convert the conversion result as it was measured at 3.3V
     * tempConvCorrected / VDDA_mV = usTempConversion / 3300
     * Calculate temperature from factory calibration data
     * (tempConvCorrected - CAL30)/(CAL110 - CAL30) = (Temp_C - 30)/(110 - 30)
     */
    temp = (tempdiff * (float)usTempConversion * fVDDA_V) / ((float)ADC_VREF_CAL_mV / 1000.0);
    temp -= tempdiff * (float)ADC_CALIB->TEMPSENSOR_LOW;
    temp /= (float)(ADC_CALIB->TEMPSENSOR_HIGH - ADC_CALIB->TEMPSENSOR_LOW);
    temp += (float)ADC_TEMP_CAL_LOW_C;
#else
    /*
     * Calculate temperature using calibration at 30C and average slope:
     * temp = (V30 - Vsense)/Avg_slope + 30
     */
    temp = ADC_TEMPSENSOR_SLOPE(ADC_fCalcExt_V(ADC_CALIB->TEMPSENSOR_LOW - usTempConversion))
            + ADC_TEMP_CAL_LOW_C;
#endif
    return temp;
}
#endif

/**
 * @brief Calculates the VDDA value in voltage and saves it for other calculations.
 * @param usVRefintConversion: 12 bit right aligned ADC measurement value
 * @return The VDDA voltage measured in milliVolts
 */
int32_t ADC_lCalcVDDA_mV(uint16_t usVRefintConversion)
{
    /*
     * The reference voltage is measured at 3.3V, with 12 bit resolution
     * VREFINT_CAL * 3300 / 4095 = usVRefintConversion * VDD / 4095
     */
    lVDDA_mV = ((int32_t)ADC_CALIB->VREFINT * ADC_VREF_CAL_mV) / (int32_t)usVRefintConversion;
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    fVDDA_V  = ((float)ADC_CALIB->VREFINT * ((float)ADC_VREF_CAL_mV / 1000.0)) / (float)usVRefintConversion;
#endif
    return lVDDA_mV;
}

/**
 * @brief Returns the last measured VDDA value in voltage.
 * @return The VDDA voltage measured in milliVolts
 */
int32_t ADC_lGetVDDA_mV(void)
{
    return lVDDA_mV;
}

#ifdef ADC_VBAT_SCALER
/**
 * @brief Converts the VBAT measurement to voltage.
 * @param usVBatConversion: 12 bit right aligned ADC measurement value
 * @return The VBAT value in milliVolts
 */
int32_t ADC_lCalcVBAT_mV(uint16_t usVBatConversion)
{
    /* Convert to mV from ADC range, multiply by scaler */
    return (ADC_VBAT_SCALER * (int32_t)usVBatConversion * lVDDA_mV) / 4095;
}
#endif

/**
 * @brief Converts a channel measurement to voltage.
 * @param usChannelConversion: 12 bit right aligned ADC measurement value
 * @return The channel voltage level
 */
int32_t ADC_lCalcExt_mV(uint16_t usChannelConversion)
{
    /* Convert to mV from ADC range */
    return ((int32_t)usChannelConversion * lVDDA_mV) / 4095;
}

/**
 * @brief Converts a temperature sensor measurement to degree Celsius.
 * @param usTempConversion: 12 bit right aligned ADC measurement value
 * @return The temperature sensor's value in degree Celcius
 */
int32_t ADC_lCalcTemp_C(uint16_t usTempConversion)
{
    int32_t temp;
#if defined(ADC_TEMP_CAL_HIGH_C)
    int32_t tempdiff = ADC_TEMP_CAL_HIGH_C - ADC_TEMP_CAL_LOW_C;
    /*
     * Convert the conversion result as it was measured at 3.3V
     * tempConvCorrected / VDDA_mV = usTempConversion / 3300
     * Calculate temperature from factory calibration data
     * (tempConvCorrected - CAL30)/(CAL110 - CAL30) = (Temp_C - 30)/(110 - 30)
     */
    temp = (tempdiff * (int32_t)usTempConversion * lVDDA_mV) / ADC_VREF_CAL_mV;
    temp -= tempdiff * (int32_t)ADC_CALIB->TEMPSENSOR_LOW;
    temp /= (int32_t)(ADC_CALIB->TEMPSENSOR_HIGH - ADC_CALIB->TEMPSENSOR_LOW);
    temp += ADC_TEMP_CAL_LOW_C;
#else
    /*
     * Calculate temperature using calibration at 30C and average slope:
     * temp = (V30 - Vsense)/Avg_slope + 30
     */
    temp = ADC_TEMPSENSOR_SLOPE(ADC_lCalcExt_mV(ADC_CALIB->TEMPSENSOR_LOW - usTempConversion))
            + ADC_TEMP_CAL_LOW_C;
#endif
    return temp;
}

/** @} */
