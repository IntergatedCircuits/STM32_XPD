/**
  ******************************************************************************
  * @file    xpd_rcc_pc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-16
  * @brief   STM32 eXtensible Peripheral Drivers RCC Peripheral Clocks Module
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
#ifndef XPD_RCC_PC_H_
#define XPD_RCC_PC_H_

#include "xpd_common.h"
#include "xpd_config.h"

#if defined(USE_XPD_ADC)
#include "xpd_adc.h"

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
void            XPD_ADC_ClockConfig         (ADC_ClockSourceType ClockSource);
uint32_t        XPD_ADC_GetClockFreq        (void);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_ADC */

#if defined(USE_XPD_CEC) && defined(RCC_CFGR3_CECSW)
#include "xpd_cec.h"

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
#ifdef LSE_VALUE
    CEC_CLOCKSOURCE_LSE        = 1  /*!< LSE clock source */
#endif
}CEC_ClockSourceType;
/** @} */

/** @addtogroup CEC_Clock_Source_Exported_Functions
 * @{ */
void            XPD_CEC_ClockConfig         (CEC_ClockSourceType ClockSource);
uint32_t        XPD_CEC_GetClockFreq        (void);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_CEC */

#if defined(USE_XPD_I2C)
#include "xpd_i2c.h"

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
void            XPD_I2C_ClockConfig         (I2C_HandleType * hi2c, I2C_ClockSourceType ClockSource);
uint32_t        XPD_I2C_GetClockFreq        (I2C_HandleType * hi2c);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_I2C */

#if defined(USE_XPD_RTC)
/** @addtogroup RTC
 * @{ */

/** @defgroup RTC_Clock_Source RTC Clock Source
 * @{ */

/** @defgroup RTC_Clock_Source_Exported_Types RTC Clock Source Exported Types
 * @{ */

/** @brief RTC clock source types */
typedef enum
{
    RTC_CLOCKSOURCE_NOCLOCK   = 0, /*!< no clock source */
#ifdef LSE_VALUE
    RTC_CLOCKSOURCE_LSE       = 1, /*!< LSE clock source */
#endif
    RTC_CLOCKSOURCE_LSI       = 2, /*!< LSI clock source */
#ifdef HSE_VALUE
    RTC_CLOCKSOURCE_HSE_DIV32 = 3, /*!< HSE / 32 clock source */
#endif
}RTC_ClockSourceType;
/** @} */

/** @addtogroup RTC_Clock_Source_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_RTC_ClockConfig         (RTC_ClockSourceType ClockSource);
uint32_t        XPD_RTC_GetClockFreq        (void);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_RTC */

#if defined(USE_XPD_TIM)
#include "xpd_tim.h"

/** @addtogroup TIM
 * @{ */

/** @defgroup TIM_Clock_Source TIM Clock Source
 * @{ */

/** @addtogroup TIM_Clock_Source_Exported_Functions
 * @{ */
uint32_t        XPD_TIM_GetClockFreq        (TIM_HandleType * htim);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_TIM */

#if defined(USE_XPD_USART)
#include "xpd_usart.h"

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
#ifdef LSE_VALUE
    USART_CLOCKSOURCE_LSE     = 2, /*!< LSE clock source */
#endif
    USART_CLOCKSOURCE_HSI     = 3, /*!< HSI clock source */
}USART_ClockSourceType;
/** @} */

/** @addtogroup USART_Clock_Source_Exported_Functions
 * @{ */
void            XPD_USART_ClockConfig       (USART_HandleType * husart, USART_ClockSourceType ClockSource);
uint32_t        XPD_USART_GetClockFreq      (USART_HandleType * husart);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_USART */

#if defined(USE_XPD_USB) && defined(RCC_CFGR3_USBSW)
#include "xpd_usb.h"

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
    USB_CLOCKSOURCE_HSI48       = 0, /*!< 48 MHz HSI clock source */
}USB_ClockSourceType;
/** @} */

/** @addtogroup USB_Clock_Source_Exported_Functions
 * @{ */
void            XPD_USB_ClockConfig         (USB_ClockSourceType ClockSource);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_USB */

#endif /* XPD_RCC_PC_H_ */
