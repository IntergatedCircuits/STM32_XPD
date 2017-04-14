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

#if defined(USE_XPD_ADC)
#include "xpd_adc.h"

/** @addtogroup ADC
 * @{ */

/** @addtogroup ADC_Clock_Source
 * @{ */

#if defined(RCC_CFGR2_ADC1PRES) || defined(RCC_CFGR2_ADCPRE12)

/* gets the clock division value from the prescaler configuration */
static const uint8_t adc_clkPreTable[] = { 0, 1, 3, 5, 7, 9, 11, 15, 31, 63, 127, 255 };

#endif

/** @defgroup ADC_Clock_Source_Exported_Functions ADC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the ADCs.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_ADC_ClockConfig(ADC_ClockSourceType ClockSource)
{
#if defined(RCC_CFGR_ADCPRE)
    RCC->CFGR.b.ADCPRE = ClockSource;
#else
    if (ClockSource < ADC_CLOCKSOURCE_PLLCLK)
    {
#if defined(RCC_CFGR2_ADCPRE12)
        XPD_ADC12_ClockCtrl(ENABLE);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC2->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC12_COMMON->CCR.b.CKMODE = ClockSource;
        }

        RCC->CFGR2.b.ADCPRE12 = 0;
#if defined(RCC_CFGR2_ADCPRE34)
        XPD_ADC34_ClockCtrl(ENABLE);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC3->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC4->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC34_COMMON->CCR.b.CKMODE = ClockSource;
        }

        RCC->CFGR2.b.ADCPRE34 = 0;
#endif
#elif defined(RCC_CFGR2_ADCPRES)
        XPD_ADC1_ClockCtrl(ENABLE);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC1_COMMON->CCR.b.CKMODE = ClockSource;
        }

        RCC->CFGR2.b.ADCPRES = 0;
#endif
    }
    else
    {
#if defined(RCC_CFGR2_ADCPRE12)
        XPD_ADC12_ClockCtrl(ENABLE);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC2->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC12_COMMON->CCR.b.CKMODE = 0;
        }

        RCC->CFGR2.b.ADCPRE12 = ClockSource;
#if defined(RCC_CFGR2_ADCPRE34)
        XPD_ADC34_ClockCtrl(ENABLE);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC3->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC4->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC34_COMMON->CCR.b.CKMODE = 0;
        }

        RCC->CFGR2.b.ADCPRE34 = ClockSource;
#endif
#elif defined(RCC_CFGR2_ADCPRES)
        XPD_ADC1_ClockCtrl(ENABLE);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC1_COMMON->CCR.b.CKMODE = 0;
        }

        RCC->CFGR2.b.ADCPRES = ClockSource;
#endif
    }
#endif
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t XPD_ADC_GetClockFreq(void)
{
#if defined(RCC_CFGR_ADCPRE)
    return XPD_RCC_GetClockFreq(PCLK2) / ((RCC->CFGR.b.ADCPRE + 1) * 2);
#else
#if defined(RCC_CFGR2_ADCPRE12)
    uint32_t mode = ADC12_COMMON->CCR.b.CKMODE;
    uint32_t source = RCC->CFGR2.b.ADCPRE12;
#elif defined(RCC_CFGR2_ADC1PRES)
    uint32_t mode = ADC1_COMMON->CCR.b.CKMODE;
    uint32_t source = RCC->CFGR2.b.ADC1PRES;
#endif
    if (source != 0)
    {
        return XPD_RCC_GetOscFreq(PLL) / ((uint32_t)adc_clkPreTable[source & 0xF] + 1);
    }
    else if (mode != 0)
    {
        return XPD_RCC_GetClockFreq(HCLK) / (1 << (mode - 1));
    }
    else
    {
        return 0;
    }
#endif
}

#if defined(RCC_CFGR2_ADCPRE12) && defined(RCC_CFGR2_ADCPRE34)
/**
 * @brief Sets the new source clock for the ADC1 and ADC2.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_ADC12_ClockConfig(ADC_ClockSourceType ClockSource)
{
    XPD_ADC12_ClockCtrl(ENABLE);

    if (ClockSource < ADC_CLOCKSOURCE_PLLCLK)
    {
        ADC12_COMMON->CCR.b.CKMODE = ClockSource;

        RCC->CFGR2.b.ADCPRE12 = 0;
    }
    else
    {
        ADC12_COMMON->CCR.b.CKMODE = 0;

        RCC->CFGR2.b.ADCPRE12 = ClockSource;
    }
}

/**
 * @brief Returns the input clock frequency of the ADC1 and ADC2.
 * @return The clock frequency of the ADC1 and ADC2 in Hz
 */
uint32_t XPD_ADC12_GetClockFreq(void)
{
    return XPD_ADC_GetClockFreq();
}

/**
 * @brief Sets the new source clock for the ADC3 and ADC4.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_ADC34_ClockConfig(ADC_ClockSourceType ClockSource)
{
    XPD_ADC34_ClockCtrl(ENABLE);

    if (ClockSource < ADC_CLOCKSOURCE_PLLCLK)
    {
        ADC34_COMMON->CCR.b.CKMODE = ClockSource;

        RCC->CFGR2.b.ADCPRE34 = 0;
    }
    else
    {
        ADC34_COMMON->CCR.b.CKMODE = 0;

        RCC->CFGR2.b.ADCPRE34 = ClockSource;
    }
}

/**
 * @brief Returns the input clock frequency of the ADC3 and ADC4.
 * @return The clock frequency of the ADC3 and ADC4 in Hz
 */
uint32_t XPD_ADC34_GetClockFreq(void)
{
    uint32_t mode = ADC34_COMMON->CCR.b.CKMODE;
    uint32_t source = RCC->CFGR2.b.ADCPRE34;

    if (source != 0)
    {
        return XPD_RCC_GetOscFreq(PLL) / ((uint32_t)adc_clkPreTable[source & 0xF] + 1);
    }
    else if (mode != 0)
    {
        return XPD_RCC_GetClockFreq(HCLK) / (1 << (mode - 1));
    }
    else
    {
        return 0;
    }
}
#endif
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

#if defined(USE_XPD_I2S)
#include "xpd_i2s.h"

/** @addtogroup I2S
 * @{ */

/** @addtogroup I2S_Clock_Source
 * @{ */

/** @defgroup I2S_Clock_Source_Exported_Functions I2S Clock Source Exported Functions
 * @{ */

#if defined(RCC_CFGR_I2SSRC)
/**
 * @brief Sets the new source clock for the I2S.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_I2S_ClockConfig(I2S_ClockSourceType ClockSource)
{
    RCC_REG_BIT(CFGR,I2SSRC) = ClockSource;
}
#endif

/**
 * @brief Returns the input clock frequency of the I2S.
 * @return The clock frequency of the I2S in Hz
 */
uint32_t XPD_I2S_GetClockFreq(void)
{
#if defined(RCC_CFGR_I2SSRC)
    if (RCC_REG_BIT(CFGR,I2SSRC) == I2S_CLOCKSOURCE_EXT)
        return EXTERNAL_CLOCK_VALUE;
    else
#endif
        return XPD_RCC_GetClockFreq(SYSCLK);
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_I2S */

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

#if defined(USE_XPD_SDADC)
#include "xpd_sdadc.h"

/** @addtogroup SDADC
 * @{ */

/** @addtogroup SDADC_Clock_Source
 * @{ */

/** @defgroup SDADC_Clock_Source_Exported_Functions SDADC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the SDADC.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_SDADC_ClockConfig(SDADC_ClockSourceType ClockSource)
{
    RCC->CFGR.b.SDADCPRE = ClockSource;
}

/**
 * @brief Returns the input clock frequency of the SDADC.
 * @return The clock frequency of the SDADC in Hz
 */
uint32_t XPD_SDADC_GetClockFreq(void)
{
    uint32_t freq = XPD_RCC_GetClockFreq(SYSCLK);
    SDADC_ClockSourceType source = RCC->CFGR.b.SDADCPRE;

    if (source > SDADC_CLOCKSOURCE_SYSCLK_DIV20)
    {
        freq /= ((source - SDADC_CLOCKSOURCE_SYSCLK_DIV24) * 4) + 24;
    }
    else if (source > SDADC_CLOCKSOURCE_SYSCLK)
    {
        freq /= ((source - SDADC_CLOCKSOURCE_SYSCLK_DIV2) * 2) + 2;
    }
    return freq;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_SDADC */

#if defined(USE_XPD_TIM)
#include "xpd_tim.h"

/** @addtogroup TIM
 * @{ */

/** @addtogroup TIM_Clock_Source
 * @{ */

/** @defgroup TIM_Clock_Source_Exported_Functions TIM Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the selected timer.
 * @param htim: pointer to the TIM handle structure
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_TIM_ClockConfig(TIM_HandleType * htim, TIM_ClockSourceType ClockSource)
{
    switch ((uint32_t)htim->Inst)
    {
#ifdef RCC_CFGR3_TIM1SW
        case TIM1_BASE:
            RCC_REG_BIT(CFGR3, TIM1SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM2SW
        case TIM2_BASE:
            RCC_REG_BIT(CFGR3, TIM2SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM8SW
        case TIM8_BASE:
            RCC_REG_BIT(CFGR3, TIM8SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM15SW
        case TIM15_BASE:
            RCC_REG_BIT(CFGR3, TIM15SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM16SW
        case TIM16_BASE:
            RCC_REG_BIT(CFGR3, TIM16SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM17SW
        case TIM17_BASE:
            RCC_REG_BIT(CFGR3, TIM17SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM20SW
        case TIM20_BASE:
            RCC_REG_BIT(CFGR3, TIM20SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM34SW
        case TIM34_BASE:
            RCC_REG_BIT(CFGR3, TIM34SW) = ClockSource;
            break;
#endif
#ifdef RCC_CFGR3_HRTIMSW
        case HRTIM1_BASE:
            RCC_REG_BIT(CFGR3, HRTIMSW) = ClockSource;
            break;
#endif
        default:
            break;
    }
}

/**
 * @brief Returns the input clock frequency of the timer.
 * @param htim: pointer to the TIM handle structure
 * @return The clock frequency of the timer in Hz
 */
uint32_t XPD_TIM_GetClockFreq(TIM_HandleType * htim)
{
    uint32_t freq;

    switch ((uint32_t)htim->Inst)
    {
#ifdef RCC_CFGR3_TIM1SW
        case TIM1_BASE:
            if (RCC_REG_BIT(CFGR3, TIM1SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM2SW
        case TIM2_BASE:
            if (RCC_REG_BIT(CFGR3, TIM2SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM8SW
        case TIM8_BASE:
            if (RCC_REG_BIT(CFGR3, TIM8SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM15SW
        case TIM15_BASE:
            if (RCC_REG_BIT(CFGR3, TIM15SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM16SW
        case TIM16_BASE:
            if (RCC_REG_BIT(CFGR3, TIM16SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM17SW
        case TIM17_BASE:
            if (RCC_REG_BIT(CFGR3, TIM17SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM20SW
        case TIM20_BASE:
            if (RCC_REG_BIT(CFGR3, TIM20SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM34SW
        case TIM34_BASE:
            if (RCC_REG_BIT(CFGR3, TIM34SW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_HRTIMSW
        case HRTIM1_BASE:
            if (RCC_REG_BIT(CFGR3, HRTIMSW))
            {
                return XPD_RCC_GetOscFreq(PLL) * 2;
            }
            break;
#endif
        default:
            break;
    }
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
    RCC_REG_BIT(CFGR, USBPRE) = ClockSource;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_USB */
