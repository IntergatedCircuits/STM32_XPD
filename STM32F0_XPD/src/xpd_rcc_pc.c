/**
  ******************************************************************************
  * @file    xpd_rcc_pc.c
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
#include "xpd_rcc.h"
#include "xpd_utils.h"

#if defined(USE_XPD_ADC)
#include "xpd_adc.h"

/** @addtogroup ADC
 * @{ */

/** @addtogroup ADC_Clock_Source
 * @{ */

/** @defgroup ADC_Clock_Source_Exported_Functions ADC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the ADCs.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_ADC_ClockConfig(ADC_ClockSourceType ClockSource)
{
    if (ClockSource == ADC_CLOCKSOURCE_HSI14)
    {
        uint32_t timeout = RCC_HSI14_TIMEOUT;

        /* Disable ADC control of the oscillator */
        RCC_REG_BIT(CR2, HSI14DIS) = 1;

        /* Enable the Internal High Speed oscillator */
        RCC_REG_BIT(CR2, HSI14ON) = 1;

        /* Wait until HSI14 is ready */
        if (XPD_OK == XPD_WaitForMatch(&RCC->CR2.w, RCC_CR2_HSI14RDY, RCC_CR2_HSI14RDY, &timeout))
        {
            /* Enable ADC control of the oscillator */
            RCC_REG_BIT(CR2, HSI14DIS) = 0;
        }
    }
    else
    {
        /* Disable the oscillator */
        RCC_REG_BIT(CR2, HSI14DIS) = 1;
        RCC_REG_BIT(CR2, HSI14ON) = 0;
    }

    XPD_ADC1_ClockCtrl(ENABLE);

    /* Peripheral configuration can only be applied when ADC is in OFF state */
    if ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
    {
        ADC1->CFGR2.b.CKMODE = ClockSource;
    }
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t XPD_ADC_GetClockFreq(void)
{
    ADC_ClockSourceType ClockSource = ADC1->CFGR2.b.CKMODE;

    if (ClockSource == ADC_CLOCKSOURCE_HSI14)
    {
        return 14000000;
    }
    else
    {
        return XPD_RCC_GetClockFreq(PCLK1) / (ClockSource * 2);
    }
}

/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_ADC */

#if defined(USE_XPD_CEC)
#include "xpd_cec.h"

/** @addtogroup CEC
 * @{ */

/** @addtogroup CEC_Clock_Source
 * @{ */

/** @defgroup CEC_Clock_Source_Exported_Functions CEC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the CEC.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_CEC_ClockConfig(CEC_ClockSourceType ClockSource)
{
    RCC_REG_BIT(CFGR3,CECSW) = ClockSource;
}

/**
 * @brief Returns the input clock frequency of the CEC.
 * @return The clock frequency of the CEC in Hz
 */
uint32_t XPD_CEC_GetClockFreq(void)
{
#ifdef LSE_VALUE
    if (RCC_REG_BIT(CFGR3,CECSW) != CEC_CLOCKSOURCE_HSI_DIV244)
    {
        return LSE_VALUE;
    }
    else
#endif
    {
        return HSI_VALUE / 244;
    }
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_CEC */

#if defined(USE_XPD_I2C)
#include "xpd_i2c.h"

/** @addtogroup I2C
 * @{ */

/** @addtogroup I2C_Clock_Source
 * @{ */

/** @defgroup I2C_Clock_Source_Exported_Functions I2C Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the selected I2C.
 * @param hi2c: pointer to the I2C handle structure
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_I2C_ClockConfig(I2C_HandleType * hi2c, I2C_ClockSourceType ClockSource)
{
    switch ((uint32_t)hi2c->Inst)
    {
#ifdef RCC_CFGR3_I2C1SW
        case I2C1_BASE:
            RCC_REG_BIT(CFGR3,I2C1SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_I2C2SW
        case I2C2_BASE:
            RCC_REG_BIT(CFGR3,I2C2SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_I2C3SW
        case I2C3_BASE:
            RCC_REG_BIT(CFGR3,I2C3SW) = ClockSource;
            break;
#endif
        default:
            break;
    }
}

/**
 * @brief Returns the input clock frequency of the I2C.
 * @param hi2c: pointer to the I2C handle structure
 * @return The clock frequency of the I2C in Hz
 */
uint32_t XPD_I2C_GetClockFreq(I2C_HandleType * hi2c)
{
    I2C_ClockSourceType source;
    uint32_t freq;

    switch ((uint32_t)hi2c->Inst)
    {
#ifdef RCC_CFGR3_I2C1SW
        case I2C1_BASE:
            source = RCC_REG_BIT(CFGR3,I2C1SW);
            break;
#endif
#ifdef RCC_CFGR3_I2C2SW
        case I2C2_BASE:
            source = RCC_REG_BIT(CFGR3,I2C2SW);
            break;
#endif
#ifdef RCC_CFGR3_I2C3SW
        case I2C3_BASE:
            source = RCC_REG_BIT(CFGR3,I2C3SW);
            break;
#endif
        default:
            source = I2C_CLOCKSOURCE_HSI;
            break;
    }
    /* get the value for the configured source */
    if (source == I2C_CLOCKSOURCE_HSI)
    {
        return HSI_VALUE;
    }
    else
    {
        return XPD_RCC_GetClockFreq(SYSCLK);
    }
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_I2C */

#if defined(USE_XPD_RTC)
#include "xpd_pwr.h"
#include "xpd_rtc.h"
#include "xpd_utils.h"

/** @addtogroup RTC
 * @{ */

/** @addtogroup RTC_Clock_Source
 * @{ */

/** @defgroup RTC_Clock_Source_Exported_Functions RTC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the RTC.
 * @param ClockSource: the new source clock which should be configured
 * @return Result of the operation
 */
XPD_ReturnType XPD_RTC_ClockConfig(RTC_ClockSourceType ClockSource)
{
    uint32_t bdcr;
    XPD_ReturnType result;

    /* enable power clock*/
    XPD_PWR_ClockCtrl(ENABLE);

    /* enable write access to backup domain */
    PWR_REG_BIT(CR,DBP) = 1;

    /* wait for backup domain write protection disable */
    result = XPD_WaitForMatch(&PWR->CR.w, PWR_CR_DBP, PWR_CR_DBP, RCC_DBP_TIMEOUT);
    if (result != XPD_OK)
    {
        return result;
    }

    /* reset the backup domain only if the RTC clock source selection is modified */
    if (RCC->BDCR.b.RTCSEL != ClockSource)
    {
        /* store the content of BDCR register before the reset of backup domain */
        bdcr = (RCC->BDCR.w & ~(RCC_BDCR_RTCSEL));

        /* RTC clock selection can be changed only if the backup domain is reset */
        RCC_REG_BIT(BDCR,BDRST) = 1;
        RCC_REG_BIT(BDCR,BDRST) = 0;

        /* restore the Content of BDCR register */
        RCC->BDCR.w = bdcr;

        /* wait for LSERDY if LSE was enabled */
        if ((bdcr & RCC_BDCR_LSERDY) != 0)
        {
            result = XPD_WaitForMatch(&RCC->BDCR.w, RCC_BDCR_LSERDY, RCC_BDCR_LSERDY, RCC_LSE_TIMEOUT);
            if (result != XPD_OK)
            {
                return result;
            }
        }

        /* set clock source if no error was encountered */
        RCC->BDCR.b.RTCSEL = ClockSource;
    }
    return result;
}

/**
 * @brief Returns the input clock frequency of the RTC.
 * @return The clock frequency of the RTC in Hz
 */
uint32_t XPD_RTC_GetClockFreq(void)
{
    uint32_t srcclk;

    /* Get the current RTC source */
    srcclk = RCC->BDCR.b.RTCSEL;

#ifdef LSE_VALUE
    /* Check if LSE is ready and if RTC clock selection is LSE */
    if ((srcclk == RTC_CLOCKSOURCE_LSE) && (RCC_REG_BIT(BDCR,LSERDY) != 0))
    {
        return LSE_VALUE;
    }
    else
#endif
    /* Check if LSI is ready and if RTC clock selection is LSI */
    if ((srcclk == RTC_CLOCKSOURCE_LSI) && (RCC_REG_BIT(CSR,LSIRDY) != 0))
    {
        return LSI_VALUE;
    }
#ifdef HSE_VALUE
    /* Check if HSE is ready  and if RTC clock selection is HSE / x */
    else if ((srcclk == RTC_CLOCKSOURCE_HSE_DIV32) && (RCC_REG_BIT(CR,HSERDY) != 0))
    {
        return HSE_VALUE / 32;
    }
#endif
    /* Clock not enabled for RTC */
    else
    {
        return 0;
    }
}

/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_RTC */

#if defined(USE_XPD_TIM)
#include "xpd_tim.h"

/** @addtogroup TIM
 * @{ */

/** @addtogroup TIM_Clock_Source
 * @{ */

/** @defgroup TIM_Clock_Source_Exported_Functions TIM Clock Source Exported Functions
 * @{ */

/**
 * @brief Returns the input clock frequency of the timer.
 * @param htim: pointer to the TIM handle structure
 * @return The clock frequency of the timer in Hz
 */
uint32_t XPD_TIM_GetClockFreq(TIM_HandleType * htim)
{
    uint32_t freq;

    {
        {
            freq = XPD_RCC_GetClockFreq(PCLK1);

            /* if APB clock is divided, timer frequency is doubled */
            if ((RCC->CFGR.w & RCC_CFGR_PPRE) != 0)
            {
                freq *= 2;
            }
        }
    }
    return freq;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_TIM */

#if defined(USE_XPD_USART)
#include "xpd_usart.h"

/** @addtogroup USART
 * @{ */

/** @addtogroup USART_Clock_Source
 * @{ */

/** @defgroup USART_Clock_Source_Exported_Functions USART Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the selected USART.
 * @param husart: pointer to the USART handle structure
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_USART_ClockConfig(USART_HandleType * husart, USART_ClockSourceType ClockSource)
{
    switch ((uint32_t)husart->Inst)
    {
#ifdef RCC_CFGR3_USART1SW
        case USART1_BASE:
            RCC->CFGR3.b.USART1SW = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_USART2SW
        case USART2_BASE:
            RCC->CFGR3.b.USART2SW = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_USART3SW
        case USART3_BASE:
            RCC->CFGR3.b.USART3SW = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_UART4SW
        case UART4_BASE:
            RCC->CFGR3.b.UART4SW = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_UART5SW
        case UART5_BASE:
            RCC->CFGR3.b.UART5SW = ClockSource;
            break;
#endif
        default:
            break;
    }
}

/**
 * @brief Returns the input clock frequency of the USART.
 * @param husart: pointer to the USART handle structure
 * @return The clock frequency of the USART in Hz
 */
uint32_t XPD_USART_GetClockFreq(USART_HandleType * husart)
{
    USART_ClockSourceType source;
    uint32_t freq;

    switch ((uint32_t)husart->Inst)
    {
#ifdef RCC_CFGR3_USART1SW
        case USART1_BASE:
            source = RCC->CFGR3.b.USART1SW;
            break;
#endif
#ifdef RCC_CFGR3_USART2SW
        case USART2_BASE:
            source = RCC->CFGR3.b.USART2SW;
            break;
#endif
#ifdef RCC_CFGR3_USART3SW
        case USART3_BASE:
            source = RCC->CFGR3.b.USART3SW;
            break;
#endif
#ifdef RCC_CFGR3_UART4SW
        case UART4_BASE:
            source = RCC->CFGR3.b.UART4SW;
            break;
#endif
#ifdef RCC_CFGR3_UART5SW
        case UART5_BASE:
            source = RCC->CFGR3.b.UART5SW;
            break;
#endif
        default:
            source = USART_CLOCKSOURCE_PCLKx;
            break;
    }
    /* get the value for the configured source */
    switch (source)
    {
        case USART_CLOCKSOURCE_SYSCLK:
            return XPD_RCC_GetClockFreq(SYSCLK);

        case USART_CLOCKSOURCE_HSI:
            return HSI_VALUE;

#ifdef LSE_VALUE
        case USART_CLOCKSOURCE_LSE:
            return LSE_VALUE;
#endif

        default:
            return XPD_RCC_GetClockFreq(PCLK1);
    }
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_USART */

#if defined(USE_XPD_USB)
#include "xpd_usb.h"

/** @addtogroup USB
 * @{ */

/** @addtogroup USB_Clock_Source
 * @{ */

/** @defgroup USB_Clock_Source_Exported_Functions USB Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the USB.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_USB_ClockConfig(USB_ClockSourceType ClockSource)
{
    RCC_REG_BIT(CFGR3, USBSW) = ClockSource;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_USB */
