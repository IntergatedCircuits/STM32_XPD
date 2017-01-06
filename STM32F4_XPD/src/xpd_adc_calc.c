/**
  ******************************************************************************
  * @file    xpd_adc_calc.c
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

#include "xpd_adc_calc.h"

#if defined(USE_XPD_ADC)

/** @addtogroup ADC
 * @{ */

/** @addtogroup ADC_Calculations
 * @{ */

/** @defgroup ADC_Calculations_Exported_Functions ADC Calculations Exported Functions
 * @{ */

static int32_t VDDA_mV = (int32_t)VDD_VALUE;

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
static float   VDDA_V  = (float)VDD_VALUE;

/**
 * @brief Returns the VDDA value in voltage.
 * @return The VDDA voltage measured in Volts
 */
float XPD_ADC_GetVDDA_V(void)
{
    return VDDA_V;
}

/**
 * @brief Converts the (12 bit right aligned) VBAT measurement to voltage.
 * @return The VBAT value in Volts
 */
float XPD_ADC_GetVBAT_V(uint16_t vBatConversion)
{
    /* Convert to V from ADC range, multiply by 2 */
    return (2.0 * (float)vBatConversion * VDDA_V) / 4095.0;
}

/**
 * @brief Converts a (12 bit right aligned) channel measurement to voltage.
 * @return The channel voltage level
 */
float XPD_ADC_GetValue_V(uint16_t channelConversion)
{
    /* Convert to V from ADC range */
    return ((float)channelConversion * VDDA_V) / 4095.0;
}

/**
 * @brief Converts a (12 bit right aligned) temperature sensor measurement to degree Celsius.
 * @return The temperature sensor's value in degree Celcius
 */
float XPD_ADC_GetTemperature_C(uint16_t tempConversion)
{
    float temp;

    /*
     * Convert the conversion result as it was measured at 3.3V
     * tempConvCorrected / VDDA_mV = tempConversion / 3300
     * Calculate temperature from factory calibration data
     * (tempConvCorrected - CAL30)/(CAL110 - CAL30) = (Temp_C - 30)/(110 - 30)
     */
    temp = ((110.0 - 30.0) * (float)tempConversion * VDDA_V) / 3.3;
    temp -= (110.0 - 30.0) * (float)ADC_TEMPSENSOR->CAL30;
    temp /= (float)(ADC_TEMPSENSOR->CAL110 - ADC_TEMPSENSOR->CAL30);
    temp += 30.0;
    return temp;
}
#endif

/**
 * @brief Calculates the VDDA value in voltage and saves it for other calculations.
 * @return The VDDA voltage measured in milliVolts
 */
int32_t XPD_ADC_SetVDDA(uint16_t vRefintConversion)
{
    /*
     * The reference voltage is measured at 3.3V, with 12 bit resolution
     * VREFINT_CAL * 3300 / 4095 = vRefintConversion * VDD / 4095
     */
    VDDA_mV = ((int32_t)ADC_VREFINT_CAL * 3300) / (int32_t)vRefintConversion;
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    VDDA_V  = ((float)ADC_VREFINT_CAL * 3.3) / (float)vRefintConversion;
#endif
    return VDDA_mV;
}

/**
 * @brief Returns the last measured VDDA value in voltage.
 * @return The VDDA voltage measured in milliVolts
 */
int32_t XPD_ADC_GetVDDA_mV(void)
{
    return VDDA_mV;
}

/**
 * @brief Converts the (12 bit right aligned) VBAT measurement to voltage.
 * @return The VBAT value in milliVolts
 */
int32_t XPD_ADC_GetVBAT_mV(uint16_t vBatConversion)
{
    /* Convert to mV from ADC range, multiply by 2 */
    return (2 * (int32_t)vBatConversion * VDDA_mV) / 4095;
}

/**
 * @brief Converts a (12 bit right aligned) channel measurement to voltage.
 * @return The channel voltage level
 */
int32_t XPD_ADC_GetValue_mV(uint16_t channelConversion)
{
    /* Convert to mV from ADC range, multiply by 2 */
    return ((int32_t)channelConversion * VDDA_mV) / 4095;
}

/**
 * @brief Converts a (12 bit right aligned) temperature sensor measurement to degree Celsius.
 * @return The temperature sensor's value in degree Celcius
 */
int32_t XPD_ADC_GetTemperature(uint16_t tempConversion)
{
    int32_t temp;
    /*
     * Convert the conversion result as it was measured at 3.3V
     * tempConvCorrected / VDDA_mV = tempConversion / 3300
     * Calculate temperature from factory calibration data
     * (tempConvCorrected - CAL30)/(CAL110 - CAL30) = (Temp_C - 30)/(110 - 30)
     */
    temp = ((110 - 30) * (int32_t)tempConversion * VDDA_mV) / 3300;
    temp -= (110 - 30) * (int32_t)ADC_TEMPSENSOR->CAL30;
    temp /= (int32_t)(ADC_TEMPSENSOR->CAL110 - ADC_TEMPSENSOR->CAL30);
    temp += 30;
    return temp;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_ADC */
