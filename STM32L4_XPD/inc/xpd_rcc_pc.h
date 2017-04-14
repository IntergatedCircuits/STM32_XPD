/**
  ******************************************************************************
  * @file    xpd_rcc_pc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-04-12
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
    ADC_CLOCKSOURCE_HCLK            = 0x01, /*!< HCLK clock source */
    ADC_CLOCKSOURCE_HCLK_DIV2       = 0x02, /*!< HCLK / 2 clock source */
    ADC_CLOCKSOURCE_HCLK_DIV4       = 0x03, /*!< HCLK / 4 clock source */
    ADC_CLOCKSOURCE_PLLSAI1         = 0x10, /*!< PLLSAI1 R output clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV2    = 0x11, /*!< PLLSAI1 R output / 2 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV4    = 0x12, /*!< PLLSAI1 R output / 4 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV6    = 0x13, /*!< PLLSAI1 R output / 6 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV8    = 0x14, /*!< PLLSAI1 R output / 8 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV10   = 0x15, /*!< PLLSAI1 R output / 10 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV12   = 0x16, /*!< PLLSAI1 R output / 12 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV16   = 0x17, /*!< PLLSAI1 R output / 16 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV32   = 0x18, /*!< PLLSAI1 R output / 32 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV64   = 0x19, /*!< PLLSAI1 R output / 64 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV128  = 0x1A, /*!< PLLSAI1 R output / 128 clock source */
    ADC_CLOCKSOURCE_PLLSAI1_DIV256  = 0x1B, /*!< PLLSAI1 R output / 256 clock source */
#ifdef RCC_PLLSAI2_SUPPORT
    ADC_CLOCKSOURCE_PLLSAI2         = 0x20, /*!< PLLSAI2 R output clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV2    = 0x21, /*!< PLLSAI2 R output / 2 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV4    = 0x22, /*!< PLLSAI2 R output / 4 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV6    = 0x23, /*!< PLLSAI2 R output / 6 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV8    = 0x24, /*!< PLLSAI2 R output / 8 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV10   = 0x25, /*!< PLLSAI2 R output / 10 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV12   = 0x26, /*!< PLLSAI2 R output / 12 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV16   = 0x27, /*!< PLLSAI2 R output / 16 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV32   = 0x28, /*!< PLLSAI2 R output / 32 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV64   = 0x29, /*!< PLLSAI2 R output / 64 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV128  = 0x2A, /*!< PLLSAI2 R output / 128 clock source */
    ADC_CLOCKSOURCE_PLLSAI2_DIV256  = 0x2B, /*!< PLLSAI2 R output / 256 clock source */
#endif
    ADC_CLOCKSOURCE_SYSCLK        = 0x20, /*!< SYSCLK clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV2   = 0x21, /*!< SYSCLK / 2 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV4   = 0x22, /*!< SYSCLK / 4 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV6   = 0x23, /*!< SYSCLK / 6 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV8   = 0x24, /*!< SYSCLK / 8 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV10  = 0x25, /*!< SYSCLK / 10 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV12  = 0x26, /*!< SYSCLK / 12 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV16  = 0x27, /*!< SYSCLK / 16 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV32  = 0x28, /*!< SYSCLK / 32 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV64  = 0x29, /*!< SYSCLK / 64 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV128 = 0x2A, /*!< SYSCLK / 128 clock source */
    ADC_CLOCKSOURCE_SYSCLK_DIV256 = 0x2B  /*!< SYSCLK / 256 clock source */
}ADC_ClockSourceType;

/** @} */

/** @addtogroup ADC_Clock_Source_Exported_Functions
 * @{ */
void            XPD_ADC_ClockConfig         (ADC_ClockSourceType ClockSource);
uint32_t        XPD_ADC_GetClockFreq        (void);
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
    I2C_CLOCKSOURCE_PCLKx   = 0, /*!< default clock source */
    I2C_CLOCKSOURCE_HSI     = 2, /*!< HSI clock source */
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

#elif defined(XPD_TIM_API)

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
#ifdef LSE_VALUE
    USART_CLOCKSOURCE_LSE     = 3, /*!< LSE clock source */
#endif
    USART_CLOCKSOURCE_HSI     = 2, /*!< HSI clock source */
}USART_ClockSourceType;
/** @} */

/** @addtogroup USART_Clock_Source_Exported_Functions
 * @{ */
void            XPD_USART_ClockConfig       (USART_HandleType * husart, USART_ClockSourceType ClockSource);
uint32_t        XPD_USART_GetClockFreq      (USART_HandleType * husart);
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
#ifdef RCC_HSI48_SUPPORT
    USB_CLOCKSOURCE_HSI48   = 0, /*!< 48MHz HSI clock source */
#else
    USB_CLOCKSOURCE_NONE    = 0, /*!< No clock source */
#endif
    USB_CLOCKSOURCE_PLLSAI1 = 1, /*!< PLLSAI1 Q output clock source */
    USB_CLOCKSOURCE_PLL     = 2, /*!< PLL Q output clock source */
    USB_CLOCKSOURCE_MSI     = 3, /*!< MSI clock source */
}USB_ClockSourceType;
/** @} */

/** @addtogroup USB_Clock_Source_Exported_Functions
 * @{ */
void            XPD_USB_ClockConfig         (USB_ClockSourceType ClockSource);
/** @} */

/** @} */

/** @} */
#endif
