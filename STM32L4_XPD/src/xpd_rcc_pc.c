/**
  ******************************************************************************
  * @file    xpd_rcc_pc.c
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
#include "xpd_rcc.h"

#ifdef RCC_PLLP_DIV_2_31_SUPPORT
#define RCC_PLLP_FREQ(PLL_NAME) \
    (XPD_RCC_GetOscFreq(XPD_RCC_GetPLLSource()) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * RCC->PLL_NAME##CFGR.b.PLL_NAME##PDIV))
#else
#define RCC_PLLP_FREQ(PLL_NAME) \
    (XPD_RCC_GetOscFreq(XPD_RCC_GetPLLSource()) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * ((RCC->PLL_NAME##CFGR.b.PLL_NAME##P == 0) ? 7 : 17)))
#endif

#define RCC_PLLR_FREQ(PLL_NAME) \
    (XPD_RCC_GetOscFreq(XPD_RCC_GetPLLSource()) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * (2 * (RCC->PLL_NAME##CFGR.b.PLL_NAME##R + 1))))

#define RCC_PLLQ_FREQ(PLL_NAME) \
    (XPD_RCC_GetOscFreq(XPD_RCC_GetPLLSource()) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * (2 * (RCC->PLL_NAME##CFGR.b.PLL_NAME##Q + 1))))

#if defined(USE_XPD_ADC)
#include "xpd_adc.h"

/** @addtogroup ADC
 * @{ */

/** @addtogroup ADC_Clock_Source
 * @{ */

/* gets the clock division value from the prescaler configuration */
static const uint8_t adc_clkPreTable[] = { 0, 1, 3, 5, 7, 9, 11, 15, 31, 63, 127, 255 };

/** @defgroup ADC_Clock_Source_Exported_Functions ADC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock and clock divider for the ADCs.
 * @param ClockSource: the new source clock which should be configured
 * @param ClockDivider: the new clock divider value
 *        Permitted range is [DIV1..DIV4] for HCLK, [DIV1..DIV256] for asynchronous clock sources
 */
void XPD_ADC_ClockConfig(ADC_ClockSourceType ClockSource)
{
    uint32_t adcsel = ClockSource >> 4;
    /* Set dedicated input multiplexer */
    RCC->CCIPR.b.ADCSEL = adcsel;

    /* Enable PLLSAI R when selected, otherwise disable */
    RCC_REG_BIT(PLLSAI1CFGR,PLLSAI1REN) = (uint32_t)(adcsel == 1);
#ifdef RCC_PLLSAI2_SUPPORT
    RCC_REG_BIT(PLLSAI2CFGR,PLLSAI2REN) = (uint32_t)(adcsel == 2);
#endif

    XPD_RCC_ClockEnable(RCC_POS_ADC);

    /* Peripheral configuration can only be applied when ADC is in OFF state */
    if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
        && ((ADC2->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
        && ((ADC3->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
    {
        /* In case of synchronous clock, the divider is set in CKMODE */
        if (ClockSource < ADC_CLOCKSOURCE_PLLSAI1)
        {
            ADC123_COMMON->CCR.b.CKMODE = ClockSource;
        }
        else
        {
            /* Select asynchronous clock, configure prescaler */
            ADC123_COMMON->CCR.b.CKMODE = 0;
            ADC123_COMMON->CCR.b.PRESC  = ClockSource;
        }
    }
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t XPD_ADC_GetClockFreq(void)
{
    uint32_t mode = ADC123_COMMON->CCR.b.CKMODE;
    uint32_t source = RCC->CCIPR.b.ADCSEL;

    if (mode != 0)
    {
        /* Calculate synchronous clock frequency */
        return XPD_RCC_GetClockFreq(HCLK) / (1 << (mode - 1));
    }
    else
    {
        uint32_t freq;

        /* Calculate asynchronous clock frequency */
        switch (source)
        {
            case 3:
                freq = XPD_RCC_GetClockFreq(SYSCLK);
                break;
#ifdef RCC_PLLSAI2_SUPPORT
            case 2:
                freq = RCC_PLLR_FREQ(PLLSAI2);
                break;
#endif
            case 1:
                freq = RCC_PLLR_FREQ(PLLSAI1);
                break;

            default:
                return 0;
        }
        return freq / ((uint32_t)adc_clkPreTable[source & 0xF] + 1);
    }
}

/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_ADC */

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
#ifdef RCC_CCIPR_I2C1SEL
        case I2C1_BASE:
            RCC->CCIPR.b.I2C1SEL = ClockSource;
            break;
#endif
#ifdef RCC_CCIPR_I2C2SEL
        case I2C2_BASE:
            RCC->CCIPR.b.I2C2SEL = ClockSource;
            break;
#endif
#ifdef RCC_CCIPR_I2C3SEL
        case I2C3_BASE:
            RCC->CCIPR.b.I2C3SEL = ClockSource;
            break;
#endif
#ifdef RCC_CCIPR2_I2C4SEL
        case I2C4_BASE:
            RCC->CCIPR2.b.I2C4SEL = ClockSource;
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
#ifdef RCC_CCIPR_I2C1SEL
        case I2C1_BASE:
            source = RCC->CCIPR.b.I2C1SEL;
            break;
#endif
#ifdef RCC_CCIPR_I2C2SEL
        case I2C2_BASE:
            source = RCC->CCIPR.b.I2C2SEL;
            break;
#endif
#ifdef RCC_CCIPR_I2C3SEL
        case I2C3_BASE:
            source = RCC->CCIPR.b.I2C3SEL;
            break;
#endif
#ifdef RCC_CCIPR2_I2C4SEL
        case I2C4_BASE:
            source = RCC->CCIPR2.b.I2C4SEL;
            break;
#endif
        default:
            source = I2C_CLOCKSOURCE_PCLKx;
            break;
    }
    /* get the value for the configured source */
    switch (source)
    {
        case I2C_CLOCKSOURCE_HSI:
            return HSI_VALUE;
        case I2C_CLOCKSOURCE_SYSCLK:
            return XPD_RCC_GetClockFreq(SYSCLK);
        default:
            return XPD_RCC_GetClockFreq((((uint32_t)hi2c->Inst) < APB2PERIPH_BASE) ? PCLK1 : PCLK2);
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

    /* enable write access to backup domain */
    PWR_REG_BIT(CR1,DBP) = 1;

    /* wait for backup domain write protection disable */
    result = XPD_WaitForMatch(&PWR->CR1.w, PWR_CR1_DBP, PWR_CR1_DBP, RCC_DBP_TIMEOUT);
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

    /* decide which APB bus the TIM is on */
    if (((uint32_t)htim->Inst) < APB2PERIPH_BASE)
    {
        {
            freq = XPD_RCC_GetClockFreq(PCLK1);

            /* if APB clock is divided, timer frequency is doubled */
            if ((RCC->CFGR.w & RCC_CFGR_PPRE1) != 0)
            {
                freq *= 2;
            }
        }
    }
    else
    {
        {
            freq = XPD_RCC_GetClockFreq(PCLK2);

            /* if APB clock is divided, timer frequency is doubled */
            if ((RCC->CFGR.w & RCC_CFGR_PPRE2) != 0)
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
#ifdef RCC_CCIPR_USART1SEL
        case USART1_BASE:
            RCC->CCIPR.b.USART1SEL = ClockSource;
            break;
#endif
#ifdef RCC_CCIPR_USART2SEL
        case USART2_BASE:
            RCC->CCIPR.b.USART2SEL = ClockSource;
            break;
#endif
#ifdef RCC_CCIPR_USART3SEL
        case USART3_BASE:
            RCC->CCIPR.b.USART3SEL = ClockSource;
            break;
#endif
#ifdef RCC_CCIPR_UART4SEL
        case UART4_BASE:
            RCC->CCIPR.b.UART4SEL = ClockSource;
            break;
#endif
#ifdef RCC_CCIPR_UART5SEL
        case UART5_BASE:
            RCC->CCIPR.b.UART5SEL = ClockSource;
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
#ifdef RCC_CCIPR_USART1SEL
        case USART1_BASE:
            source = RCC->CCIPR.b.USART1SEL;
            break;
#endif
#ifdef RCC_CCIPR_USART2SEL
        case USART2_BASE:
            source = RCC->CCIPR.b.USART2SEL;
            break;
#endif
#ifdef RCC_CCIPR_USART3SEL
        case USART3_BASE:
            source = RCC->CCIPR.b.USART3SEL;
            break;
#endif
#ifdef RCC_CCIPR_UART4SEL
        case UART4_BASE:
            source = RCC->CCIPR.b.UART4SEL;
            break;
#endif
#ifdef RCC_CCIPR_UART5SEL
        case UART5_BASE:
            source = RCC->CCIPR.b.UART5SEL;
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
            return XPD_RCC_GetClockFreq((((uint32_t)husart->Inst) < APB2PERIPH_BASE) ? PCLK1 : PCLK2);
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
    RCC->CCIPR.b.CLK48SEL = ClockSource;

    switch (ClockSource)
    {
    case USB_CLOCKSOURCE_PLL:
        /* Enable PLL Q output */
        RCC_REG_BIT(PLLCFGR, PLLQEN) = 1;
        break;

    case USB_CLOCKSOURCE_PLLSAI1:
        /* Enable PLLSAI1 Q output */
        RCC_REG_BIT(PLLSAI1CFGR, PLLSAI1QEN) = 1;
        break;

    default:
        break;
    }
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_USB */
