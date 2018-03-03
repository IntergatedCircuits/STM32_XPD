/**
  ******************************************************************************
  * @file    xpd_rcc_pc.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers RCC Peripheral Clocks Module
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
#if   defined(XPD_ADC_API)

/** @addtogroup ADC
 * @{ */

/** @defgroup ADC_Clock_Source ADC Clock Source
 * @{ */

/** @defgroup ADC_Clock_Source_Exported_Types ADC Clock Source Exported Types
 * @{ */

/** @brief ADC clock source types */
typedef enum
{
    ADC_CLOCKSOURCE_PCLK_DIV2 = 1, /*!< PCLK / 2 clock source */
    ADC_CLOCKSOURCE_PCLK_DIV4 = 2, /*!< PCLK / 4 clock source */
    ADC_CLOCKSOURCE_HSI14     = 0, /*!< 14 MHz HSI clock source */
}ADC_ClockSourceType;
/** @} */

/** @addtogroup ADC_Clock_Source_Exported_Functions
 * @{ */
void            ADC_vClockConfig        (ADC_ClockSourceType eClockSource);
uint32_t        ADC_ulClockFreq_Hz      (void);
/** @} */

/** @} */

/** @} */

#elif defined(XPD_CEC_API)

/** @addtogroup CEC
 * @{ */

/** @defgroup CEC_Clock_Source CEC Clock Source
 * @{ */

/** @defgroup CEC_Clock_Source_Exported_Types CEC Clock Source Exported Types
 * @{ */

/** @brief CEC clock source types */
typedef enum
{
    CEC_CLOCKSOURCE_HSI_DIV244 = 0, /*!< HSI / 244 clock source */
#ifdef LSE_VALUE_Hz
    CEC_CLOCKSOURCE_LSE        = 1  /*!< LSE clock source */
#endif
}CEC_ClockSourceType;
/** @} */

/** @addtogroup CEC_Clock_Source_Exported_Functions
 * @{ */
void            CEC_vClockConfig        (CEC_ClockSourceType eClockSource);
uint32_t        CEC_ulClockFreq_Hz      (void);
/** @} */

/** @} */

/** @} */

#elif defined(XPD_I2C_API)

/** @addtogroup I2C
 * @{ */

/** @defgroup I2C_Clock_Source I2C Clock Source
 * @{ */

/** @defgroup I2C_Clock_Source_Exported_Types I2C Clock Source Exported Types
 * @{ */

/** @brief I2C clock source types */
typedef enum
{
    I2C_CLOCKSOURCE_HSI     = 0, /*!< HSI clock source */
    I2C_CLOCKSOURCE_SYSCLK  = 1, /*!< SYSCLK clock source */
}I2C_ClockSourceType;
/** @} */

/** @addtogroup I2C_Clock_Source_Exported_Functions
 * @{ */
void            I2C_vClockConfig        (I2C_HandleType * pxI2C, I2C_ClockSourceType eClockSource);
uint32_t        I2C_ulClockFreq_Hz      (I2C_HandleType * pxI2C);
/** @} */

/** @} */

/** @} */

#elif defined(XPD_RTC_API)
/** @addtogroup RTC
 * @{ */

/** @defgroup RTC_Clock_Source RTC Clock Source
 * @{ */

/** @defgroup RTC_Clock_Source_Exported_Types RTC Clock Source Exported Types
 * @{ */

/** @brief RTC clock source types */
typedef enum
{
    RTC_CLOCKSOURCE_NONE      = 0, /*!< no clock source */
#ifdef LSE_VALUE_Hz
    RTC_CLOCKSOURCE_LSE       = 1, /*!< LSE clock source */
#endif
    RTC_CLOCKSOURCE_LSI       = 2, /*!< LSI clock source */
#ifdef HSE_VALUE_Hz
    RTC_CLOCKSOURCE_HSE_DIV32 = 3, /*!< HSE / 32 clock source */
#endif
}RTC_ClockSourceType;
/** @} */

/** @addtogroup RTC_Clock_Source_Exported_Functions
 * @{ */
XPD_ReturnType  RTC_eClockConfig        (RTC_ClockSourceType eClockSource);
uint32_t        RTC_ulClockFreq_Hz      (void);
/** @} */

/** @} */

/** @} */

#elif defined(XPD_TIM_API)

/** @addtogroup TIM
 * @{ */

/** @defgroup TIM_Clock_Source TIM Clock Source
 * @{ */

/** @addtogroup TIM_Clock_Source_Exported_Functions
 * @{ */
uint32_t        TIM_ulClockFreq_Hz      (TIM_HandleType * pxTIM);
/** @} */

/** @} */

/** @} */

#elif defined(XPD_USART_API)

/** @addtogroup USART
 * @{ */

/** @defgroup USART_Clock_Source USART Clock Source
 * @{ */

/** @defgroup USART_Clock_Source_Exported_Types USART Clock Source Exported Types
 * @{ */

/** @brief USART clock source types */
typedef enum
{
    USART_CLOCKSOURCE_PCLKx   = 0, /*!< default clock source */
    USART_CLOCKSOURCE_SYSCLK  = 1, /*!< SYSCLK clock source */
#ifdef LSE_VALUE_Hz
    USART_CLOCKSOURCE_LSE     = 2, /*!< LSE clock source */
#endif
    USART_CLOCKSOURCE_HSI     = 3, /*!< HSI clock source */
}USART_ClockSourceType;
/** @} */

/** @addtogroup USART_Clock_Source_Exported_Functions
 * @{ */
void            USART_vClockConfig      (USART_HandleType * pxUSART, USART_ClockSourceType eClockSource);
uint32_t        USART_ulClockFreq_Hz    (USART_HandleType * pxUSART);
/** @} */

/** @} */

/** @} */

#elif defined(XPD_USB_API)

/** @addtogroup USB
 * @{ */

/** @defgroup USB_Clock_Source USB Clock Source
 * @{ */

/** @defgroup USB_Clock_Source_Exported_Types USB Clock Source Exported Types
 * @{ */

/** @brief USB clock source types */
typedef enum
{
    USB_CLOCKSOURCE_PLL         = 1, /*!< PLL clock source */
#ifdef RCC_HSI48_SUPPORT
    USB_CLOCKSOURCE_HSI48       = 0, /*!< 48 MHz HSI clock source */
#endif
}USB_ClockSourceType;
/** @} */

/** @addtogroup USB_Clock_Source_Exported_Functions
 * @{ */
void            USB_vClockConfig        (USB_ClockSourceType eClockSource);
/** @} */

/** @} */

/** @} */
#endif
