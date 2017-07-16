/**
  ******************************************************************************
  * @file    xpd_adc_calc.c
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-07-16
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

static int32_t VDDA_mV = (int32_t)VDDA_VALUE;

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
static float   VDDA_V  = ((float)VDDA_VALUE) / 1000.0;

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
    /* Convert to V from ADC range, multiply by scaler */
    return (ADC_VBAT_SCALER * (float)vBatConversion * VDDA_V) / 4095.0;
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
#if defined(ADC_TEMPSENSOR)
    float tempdiff = (float)(ADC_TEMPSENSOR_CAL_HIGH - ADC_TEMPSENSOR_CAL_LOW);
    /*
     * Convert the conversion result as it was measured at 3.3V
     * tempConvCorrected / VDDA_mV = tempConversion / 3300
     * Calculate temperature from factory calibration data
     * (tempConvCorrected - CAL30)/(CAL110 - CAL30) = (Temp_C - 30)/(110 - 30)
     */
    temp = (tempdiff * (float)tempConversion * VDDA_V) / ((float)ADC_CAL_mV / 1000.0);
    temp -= tempdiff * (float)ADC_TEMPSENSOR->CAL_LOW;
    temp /= (float)(ADC_TEMPSENSOR->CAL_HIGH - ADC_TEMPSENSOR->CAL_LOW);
    temp += (float)ADC_TEMPSENSOR_CAL_LOW;
#elif defined(ADC_TEMPSENSOR_LOW)
    /*
     * Calculate temperature using calibration at 30C and average slope:
     * temp = (V30 - Vsense)/Avg_slope + 30
     */
    temp = ADC_TEMPSENSOR_SLOPE(XPD_ADC_GetValue_V(ADC_TEMPSENSOR_LOW - tempConversion))
            + ADC_TEMPSENSOR_CAL_LOW;
#else
    temp = 0;
#endif
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
    VDDA_mV = ((int32_t)ADC_VREFINT_CAL * ADC_CAL_mV) / (int32_t)vRefintConversion;
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    VDDA_V  = ((float)ADC_VREFINT_CAL * ((float)ADC_CAL_mV / 1000.0)) / (float)vRefintConversion;
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
    /* Convert to mV from ADC range, multiply by scaler */
    return (ADC_VBAT_SCALER * (int32_t)vBatConversion * VDDA_mV) / 4095;
}

/**
 * @brief Converts a (12 bit right aligned) channel measurement to voltage.
 * @return The channel voltage level
 */
int32_t XPD_ADC_GetValue_mV(uint16_t channelConversion)
{
    /* Convert to mV from ADC range */
    return ((int32_t)channelConversion * VDDA_mV) / 4095;
}

/**
 * @brief Converts a (12 bit right aligned) temperature sensor measurement to degree Celsius.
 * @return The temperature sensor's value in degree Celcius
 */
int32_t XPD_ADC_GetTemperature(uint16_t tempConversion)
{
    int32_t temp;
#if defined(ADC_TEMPSENSOR)
    int32_t tempdiff = ADC_TEMPSENSOR_CAL_HIGH - ADC_TEMPSENSOR_CAL_LOW;
    /*
     * Convert the conversion result as it was measured at 3.3V
     * tempConvCorrected / VDDA_mV = tempConversion / 3300
     * Calculate temperature from factory calibration data
     * (tempConvCorrected - CAL30)/(CAL110 - CAL30) = (Temp_C - 30)/(110 - 30)
     */
    temp = (tempdiff * (int32_t)tempConversion * VDDA_mV) / ADC_CAL_mV;
    temp -= tempdiff * (int32_t)ADC_TEMPSENSOR->CAL_LOW;
    temp /= (int32_t)(ADC_TEMPSENSOR->CAL_HIGH - ADC_TEMPSENSOR->CAL_LOW);
    temp += ADC_TEMPSENSOR_CAL_LOW;
#elif defined(ADC_TEMPSENSOR_LOW)
    /*
     * Calculate temperature using calibration at 30C and average slope:
     * temp = (V30 - Vsense)/Avg_slope + 30
     */
    temp = ADC_TEMPSENSOR_SLOPE(XPD_ADC_GetValue_mV(ADC_TEMPSENSOR_LOW - tempConversion))
            + ADC_TEMPSENSOR_CAL_LOW;
#else
    temp = 0;
#endif
    return temp;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_ADC */
