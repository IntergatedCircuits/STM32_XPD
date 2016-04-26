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
#if defined(RCC_CFGR2_ADC1PRES) || defined(RCC_CFGR2_ADCPRE12)
typedef enum
{
    ADC_CLOCKSOURCE_HCLK          = 0x00,
    ADC_CLOCKSOURCE_PLLCLK_DIV1   = 0x10,
    ADC_CLOCKSOURCE_PLLCLK_DIV2   = 0x11,
    ADC_CLOCKSOURCE_PLLCLK_DIV4   = 0x12,
    ADC_CLOCKSOURCE_PLLCLK_DIV6   = 0x13,
    ADC_CLOCKSOURCE_PLLCLK_DIV8   = 0x14,
    ADC_CLOCKSOURCE_PLLCLK_DIV10  = 0x15,
    ADC_CLOCKSOURCE_PLLCLK_DIV12  = 0x16,
    ADC_CLOCKSOURCE_PLLCLK_DIV16  = 0x17,
    ADC_CLOCKSOURCE_PLLCLK_DIV32  = 0x18,
    ADC_CLOCKSOURCE_PLLCLK_DIV64  = 0x19,
    ADC_CLOCKSOURCE_PLLCLK_DIV128 = 0x1A,
    ADC_CLOCKSOURCE_PLLCLK_DIV256 = 0x1B
}ADC_ClockSourceType;
#elif defined(RCC_CFGR_ADCPRE)
typedef enum
{
    ADC_CLOCKSOURCE_PCLK2_DIV2 = 0,
    ADC_CLOCKSOURCE_PCLK2_DIV4 = 1,
    ADC_CLOCKSOURCE_PCLK2_DIV6 = 2,
    ADC_CLOCKSOURCE_PCLK2_DIV8 = 3
}ADC_ClockSourceType;
#endif
/** @} */

/** @defgroup ADC_Clock_Source_Exported_Functions ADC Clock Source Exported Functions
 * @{ */
#ifdef RCC_CFGR2_ADCPRE12
void            XPD_ADC12_ClockConfig       (ADC_ClockSourceType ClockSource);
uint32_t        XPD_ADC12_GetClockFreq      (void);
#endif

#ifdef RCC_CFGR2_ADC1PRES
void            XPD_ADC1_ClockConfig        (ADC_ClockSourceType ClockSource);
uint32_t        XPD_ADC1_GetClockFreq       (void);

#elif defined(RCC_CFGR_ADCPRE)
void            XPD_ADC1_ClockConfig        (ADC_ClockSourceType ClockSource);
uint32_t        XPD_ADC1_GetClockFreq       (void);
#endif

#ifdef RCC_CFGR2_ADCPRE34
void            XPD_ADC34_ClockConfig       (ADC_ClockSourceType ClockSource);
uint32_t        XPD_ADC34_GetClockFreq      (void);
#endif
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
    CEC_CLOCKSOURCE_HSI_DIV244 = 0,
    CEC_CLOCKSOURCE_LSE        = 1
}CEC_ClockSourceType;
/** @} */

/** @defgroup CEC_Clock_Source_Exported_Functions CEC Clock Source Exported Functions
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

/** @defgroup I2C_Clock_Source_Exported_Functions I2C Clock Source Exported Functions
 * @{ */
void            XPD_I2C_ClockConfig         (I2C_HandleType * hi2c, I2C_ClockSourceType ClockSource);
uint32_t        XPD_I2C_GetClockFreq        (I2C_HandleType * hi2c);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_I2C */

#if defined(USE_XPD_I2C)
#include "xpd_i2s.h"

/** @addtogroup I2S
 * @{ */

/** @defgroup I2S_Clock_Source I2S Clock Source
 * @{ */

#if defined(RCC_CFGR_I2SSRC)
/** @defgroup I2S_Clock_Source_Exported_Types I2S Clock Source Exported Types
 * @{ */

/** @brief I2S clock source types */
typedef enum
{
    I2S_CLOCKSOURCE_SYSCLK = 0, /*!< SYSCLK clock source */
    I2S_CLOCKSOURCE_EXT    = 1, /*!< external clock source */
}I2S_ClockSourceType;
/** @} */
#endif

/** @defgroup I2S_Clock_Source_Exported_Functions I2S Clock Source Exported Functions
 * @{ */
#if defined(RCC_CFGR_I2SSRC)
void            XPD_I2S_ClockConfig         (I2S_ClockSourceType ClockSource);
#endif
uint32_t        XPD_I2S_GetClockFreq        (void);
/** @} */

/** @} */

/** @} */
#endif

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
    RTC_CLOCKSOURCE_LSE       = 1, /*!< LSE clock source */
    RTC_CLOCKSOURCE_LSI       = 2, /*!< LSI clock source */
    RTC_CLOCKSOURCE_HSE_DIV32 = 3, /*!< HSE / 32 clock source */
}RTC_ClockSourceType;
/** @} */

/** @defgroup RTC_Clock_Source_Exported_Functions RTC Clock Source Exported Functions
 * @{ */
XPD_ReturnType  XPD_RTC_ClockConfig         (RTC_ClockSourceType ClockSource);
uint32_t        XPD_RTC_GetClockFreq        (void);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_RTC */

#if defined(USE_XPD_SDADC) && defined(RCC_CFGR_SDADCPRE)
#include "xpd_sdadc.h"
/** @addtogroup SDADC
 * @{ */

/** @defgroup SDADC_Clock_Source SDADC Clock Source
 * @{ */

/** @defgroup SDADC_Clock_Source_Exported_Types SDADC Clock Source Exported Types
 * @{ */

/** @brief SDADC clock source types */
typedef enum
{
    SDADC_CLOCKSOURCE_SYSCLK_DIV1  = 0x00, /*!< SYSCLK clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV2  = 0x10, /*!< SYSCLK / 2 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV4  = 0x11, /*!< SYSCLK / 4 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV6  = 0x12, /*!< SYSCLK / 6 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV8  = 0x13, /*!< SYSCLK / 8 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV10 = 0x14, /*!< SYSCLK / 10 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV12 = 0x15, /*!< SYSCLK / 12 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV14 = 0x16, /*!< SYSCLK / 14 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV16 = 0x17, /*!< SYSCLK / 16 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV20 = 0x18, /*!< SYSCLK / 20 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV24 = 0x19, /*!< SYSCLK / 24 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV28 = 0x1A, /*!< SYSCLK / 28 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV32 = 0x1B, /*!< SYSCLK / 32 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV36 = 0x1C, /*!< SYSCLK / 36 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV40 = 0x1D, /*!< SYSCLK / 40 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV44 = 0x1E, /*!< SYSCLK / 44 clock source */
    SDADC_CLOCKSOURCE_SYSCLK_DIV48 = 0x1F, /*!< SYSCLK / 48 clock source */
}SDADC_ClockSourceType;
/** @} */

/** @defgroup SDADC_Clock_Source_Exported_Functions SDADC Clock Source Exported Functions
 * @{ */
void            XPD_SDADC_ClockConfig       (SDADC_ClockSourceType ClockSource);
uint32_t        XPD_SDADC_GetClockFreq      (void);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_SDADC */

#if defined(USE_XPD_TIM)
#include "xpd_tim.h"

/** @addtogroup TIM
 * @{ */

/** @defgroup TIM_Clock_Source TIM Clock Source
 * @{ */

/** @defgroup TIM_Clock_Source_Exported_Types TIM Clock Source Exported Types
 * @{ */

/** @brief TIM clock source types */
typedef enum
{
    TIM_CLOCKSOURCE_PCLKx        = 0, /*!< default clock source */
    TIM_CLOCKSOURCE_PLLCLK_MUL2  = 1, /*!< clock source is PLL clock * 2 */
}TIM_ClockSourceType;
/** @} */

/** @defgroup TIM_Clock_Source_Exported_Functions TIM Clock Source Exported Functions
 * @{ */

void            XPD_TIM_ClockConfig         (TIM_HandleType * htim, TIM_ClockSourceType ClockSource);
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
    USART_CLOCKSOURCE_LSE     = 2, /*!< LSE clock source */
    USART_CLOCKSOURCE_HSI     = 3, /*!< HSI clock source */
}USART_ClockSourceType;
/** @} */

/** @defgroup USART_Clock_Source_Exported_Functions USART Clock Source Exported Functions
 * @{ */
void            XPD_USART_ClockConfig       (USART_ClockSourceType ClockSource);
uint32_t        XPD_USART_GetClockFreq      (void);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_USART */

#if defined(USE_XPD_USB) && defined(RCC_CFGR_USBPRE)
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
    USB_CLOCKSOURCE_PLL_DIV1_5  = 0, /*!< PLL * 2 / 3 clock source */
}USB_ClockSourceType;
/** @} */

/** @defgroup USBD_Clock_Source_Exported_Functions USBD Clock Source Exported Functions
 * @{ */
void            XPD_USB_ClockConfig         (USB_ClockSourceType ClockSource);
/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_USB */

#endif /* XPD_RCC_PC_H_ */
