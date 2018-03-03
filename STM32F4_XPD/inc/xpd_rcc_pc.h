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

#if   defined(XPD_I2C_API)

/** @ingroup I2C
 * @defgroup I2C_Clock_Source I2C Clock Source
 * @{ */

/** @addtogroup I2C_Clock_Source_Exported_Functions
 * @{ */
uint32_t        I2C_ulClockFreq_Hz      (I2C_HandleType * pxI2C);
/** @} */

/** @} */

#elif defined(XPD_RTC_API)

/** @ingroup RTC
 * @defgroup RTC_Clock_Source RTC Clock Source
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
    RTC_CLOCKSOURCE_HSE       = 3, /*!< HSE clock source */
#endif
}RTC_ClockSourceType;
/** @} */

/** @addtogroup RTC_Clock_Source_Exported_Functions
 * @{ */
XPD_ReturnType  RTC_eClockConfig        (RTC_ClockSourceType eClockSource);
uint32_t        RTC_ulClockFreq_Hz      (void);
/** @} */

/** @} */

#elif defined(XPD_TIM_API)

/** @ingroup TIM
 * @defgroup TIM_Clock_Source TIM Clock Source
 * @{ */

#ifdef RCC_DCKCFGR_TIMPRE
/** @defgroup TIM_Clock_Source_Exported_Types TIM Clock Source Exported Types
 * @{ */

/** @brief TIM clock source types */
typedef enum
{
    TIM_CLOCKSOURCE_PCLKx        = 0, /*!< default clock source */
    TIM_CLOCKSOURCE_HCLK         = 1, /*!< clock source from HCLK
                                           @note Only applies if APBx prescaler is DIV1, 2 or 4 */
}TIM_ClockSourceType;
/** @} */
#endif

/** @addtogroup TIM_Clock_Source_Exported_Functions
 * @{ */
#ifdef RCC_DCKCFGR_TIMPRE
void            TIM_vClockConfig        (TIM_ClockSourceType eClockSource);
#endif
uint32_t        TIM_ulClockFreq_Hz      (TIM_HandleType * pxTIM);
/** @} */

/** @} */

#elif defined(XPD_USART_API)

/** @ingroup USART
 * @defgroup USART_Clock_Source USART Clock Source
 * @{ */

/** @addtogroup USART_Clock_Source_Exported_Functions
 * @{ */
uint32_t        USART_ulClockFreq_Hz    (USART_HandleType * pxUSART);
/** @} */

/** @} */
#endif
