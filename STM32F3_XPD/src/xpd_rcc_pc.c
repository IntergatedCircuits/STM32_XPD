/**
  ******************************************************************************
  * @file    xpd_rcc_pc.c
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
#include <xpd_rcc.h>
#include <xpd_adc.h>
#include <xpd_cec.h>
#include <xpd_i2c.h>
#include <xpd_i2s.h>
#include <xpd_pwr.h>
#include <xpd_rtc.h>
#include <xpd_sdadc.h>
#include <xpd_tim.h>
#include <xpd_usart.h>
#include <xpd_usb.h>
#include <xpd_utils.h>

#if defined(RCC_CFGR2_ADC1PRES) || defined(RCC_CFGR2_ADCPRE12)

/* gets the clock division value from the prescaler configuration */
static const uint8_t ADC_aucClkPreTable[] = { 0, 1, 3, 5, 7, 9, 11, 15, 31, 63, 127, 255 };

#endif

/** @ingroup ADC_Clock_Source
 * @defgroup ADC_Clock_Source_Exported_Functions ADC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the ADCs.
 * @param eClockSource: the new source clock which should be configured
 */
void ADC_vClockConfig(ADC_ClockSourceType eClockSource)
{
#if defined(RCC_CFGR_ADCPRE)
    RCC->CFGR.b.ADCPRE = eClockSource;
#else
    if (eClockSource < ADC_CLOCKSOURCE_PLLCLK)
    {
#if defined(RCC_CFGR2_ADCPRE12)
        RCC_vClockEnable(RCC_POS_ADC12);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC2->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC12_COMMON->CCR.b.CKMODE = eClockSource;
        }

        RCC->CFGR2.b.ADCPRE12 = 0;
#if defined(RCC_CFGR2_ADCPRE34)
        RCC_vClockEnable(RCC_POS_ADC34);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC3->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC4->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC34_COMMON->CCR.b.CKMODE = eClockSource;
        }

        RCC->CFGR2.b.ADCPRE34 = 0;
#endif
#elif defined(RCC_CFGR2_ADCPRES)
        RCC_vClockEnable(RCC_POS_ADC1);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC1_COMMON->CCR.b.CKMODE = eClockSource;
        }

        RCC->CFGR2.b.ADCPRES = 0;
#endif
    }
    else
    {
#if defined(RCC_CFGR2_ADCPRE12)
        RCC_vClockEnable(RCC_POS_ADC12);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC2->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC12_COMMON->CCR.b.CKMODE = 0;
        }

        RCC->CFGR2.b.ADCPRE12 = eClockSource;
#if defined(RCC_CFGR2_ADCPRE34)
        RCC_vClockEnable(RCC_POS_ADC34);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC3->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
            && ((ADC4->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC34_COMMON->CCR.b.CKMODE = 0;
        }

        RCC->CFGR2.b.ADCPRE34 = eClockSource;
#endif
#elif defined(RCC_CFGR2_ADCPRES)
        RCC_vClockEnable(RCC_POS_ADC1);

        /* Peripheral configuration can only be applied when ADC is in OFF state */
        if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0))
        {
            ADC1_COMMON->CCR.b.CKMODE = 0;
        }

        RCC->CFGR2.b.ADCPRES = eClockSource;
#endif
    }
#endif
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t ADC_ulClockFreq_Hz(void)
{
#if defined(RCC_CFGR_ADCPRE)
    return RCC_ulClockFreq_Hz(PCLK2) / ((RCC->CFGR.b.ADCPRE + 1) * 2);
#else
#if defined(RCC_CFGR2_ADCPRE12)
    uint32_t ulMode = ADC12_COMMON->CCR.b.CKMODE;
    uint32_t ulSrc = RCC->CFGR2.b.ADCPRE12;
#elif defined(RCC_CFGR2_ADC1PRES)
    uint32_t ulMode = ADC1_COMMON->CCR.b.CKMODE;
    uint32_t ulSrc = RCC->CFGR2.b.ADC1PRES;
#endif
    if (ulSrc != 0)
    {
        return RCC_ulOscFreq_Hz(PLL) / ((uint32_t)ADC_aucClkPreTable[ulSrc & 0xF] + 1);
    }
    else if (ulMode != 0)
    {
        return RCC_ulClockFreq_Hz(HCLK) / (1 << (ulMode - 1));
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
 * @param eClockSource: the new source clock which should be configured
 */
void ADC12_vClockConfig(ADC_ClockSourceType eClockSource)
{
    RCC_vClockEnable(RCC_POS_ADC12);

    if (eClockSource < ADC_CLOCKSOURCE_PLLCLK)
    {
        ADC12_COMMON->CCR.b.CKMODE = eClockSource;

        RCC->CFGR2.b.ADCPRE12 = 0;
    }
    else
    {
        ADC12_COMMON->CCR.b.CKMODE = 0;

        RCC->CFGR2.b.ADCPRE12 = eClockSource;
    }
}

/**
 * @brief Returns the input clock frequency of the ADC1 and ADC2.
 * @return The clock frequency of the ADC1 and ADC2 in Hz
 */
uint32_t ADC12_ulClockFreq_Hz(void)
{
    return ADC_ulClockFreq_Hz();
}

/**
 * @brief Sets the new source clock for the ADC3 and ADC4.
 * @param eClockSource: the new source clock which should be configured
 */
void ADC34_vClockConfig(ADC_ClockSourceType eClockSource)
{
    RCC_vClockEnable(RCC_POS_ADC34);

    if (eClockSource < ADC_CLOCKSOURCE_PLLCLK)
    {
        ADC34_COMMON->CCR.b.CKMODE = eClockSource;

        RCC->CFGR2.b.ADCPRE34 = 0;
    }
    else
    {
        ADC34_COMMON->CCR.b.CKMODE = 0;

        RCC->CFGR2.b.ADCPRE34 = eClockSource;
    }
}

/**
 * @brief Returns the input clock frequency of the ADC3 and ADC4.
 * @return The clock frequency of the ADC3 and ADC4 in Hz
 */
uint32_t ADC34_ulClockFreq_Hz(void)
{
    uint32_t ulMode = ADC34_COMMON->CCR.b.CKMODE;
    uint32_t ulSrc = RCC->CFGR2.b.ADCPRE34;

    if (ulSrc != 0)
    {
        return RCC_ulOscFreq_Hz(PLL) / ((uint32_t)ADC_aucClkPreTable[ulSrc & 0xF] + 1);
    }
    else if (ulMode != 0)
    {
        return RCC_ulClockFreq_Hz(HCLK) / (1 << (ulMode - 1));
    }
    else
    {
        return 0;
    }
}
#endif
/** @} */

#if defined(CEC)

/** @ingroup CEC_Clock_Source
 * @defgroup CEC_Clock_Source_Exported_Functions CEC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the CEC.
 * @param eClockSource: the new source clock which should be configured
 */
void CEC_vClockConfig(CEC_ClockSourceType eClockSource)
{
    RCC_REG_BIT(CFGR3,CECSW) = eClockSource;
}

/**
 * @brief Returns the input clock frequency of the CEC.
 * @return The clock frequency of the CEC in Hz
 */
uint32_t CEC_ulClockFreq_Hz(void)
{
#ifdef LSE_VALUE_Hz
    if (RCC_REG_BIT(CFGR3,CECSW) != CEC_CLOCKSOURCE_HSI_DIV244)
    {
        return LSE_VALUE_Hz;
    }
    else
#endif
    {
        return HSI_VALUE_Hz / 244;
    }
}

/** @} */

#endif /* defined(CEC) */

/** @ingroup I2C_Clock_Source
 * @defgroup I2C_Clock_Source_Exported_Functions I2C Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the selected I2C.
 * @param pxI2C: pointer to the I2C handle structure
 * @param eClockSource: the new source clock which should be configured
 */
void I2C_vClockConfig(I2C_HandleType * pxI2C, I2C_ClockSourceType eClockSource)
{
    switch ((uint32_t)pxI2C->Inst)
    {
#ifdef RCC_CFGR3_I2C1SW
        case I2C1_BASE:
            RCC_REG_BIT(CFGR3,I2C1SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_I2C2SW
        case I2C2_BASE:
            RCC_REG_BIT(CFGR3,I2C2SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_I2C3SW
        case I2C3_BASE:
            RCC_REG_BIT(CFGR3,I2C3SW) = eClockSource;
            break;
#endif
        default:
            break;
    }
}

/**
 * @brief Returns the input clock frequency of the I2C.
 * @param pxI2C: pointer to the I2C handle structure
 * @return The clock frequency of the I2C in Hz
 */
uint32_t I2C_ulClockFreq_Hz(I2C_HandleType * pxI2C)
{
    I2C_ClockSourceType eSource;

    switch ((uint32_t)pxI2C->Inst)
    {
#ifdef RCC_CFGR3_I2C1SW
        case I2C1_BASE:
            eSource = RCC_REG_BIT(CFGR3,I2C1SW);
            break;
#endif
#ifdef RCC_CFGR3_I2C2SW
        case I2C2_BASE:
            eSource = RCC_REG_BIT(CFGR3,I2C2SW);
            break;
#endif
#ifdef RCC_CFGR3_I2C3SW
        case I2C3_BASE:
            eSource = RCC_REG_BIT(CFGR3,I2C3SW);
            break;
#endif
        default:
            eSource = I2C_CLOCKSOURCE_HSI;
            break;
    }
    /* get the value for the configured source */
    if (eSource == I2C_CLOCKSOURCE_HSI)
    {
        return HSI_VALUE_Hz;
    }
    else
    {
        return RCC_ulClockFreq_Hz(SYSCLK);
    }
}

/** @} */

/** @ingroup I2S_Clock_Source
 * @defgroup I2S_Clock_Source_Exported_Functions I2S Clock Source Exported Functions
 * @{ */

#if defined(RCC_CFGR_I2SSRC)
/**
 * @brief Sets the new source clock for the I2S.
 * @param eClockSource: the new source clock which should be configured
 */
void XPD_I2S_ClockConfig(I2S_ClockSourceType eClockSource)
{
    RCC_REG_BIT(CFGR,I2SSRC) = eClockSource;
}
#endif

/**
 * @brief Returns the input clock frequency of the I2S.
 * @return The clock frequency of the I2S in Hz
 */
uint32_t XPD_I2S_ulClockFreq_Hz(void)
{
#if defined(EXTERNAL_CLOCK_VALUE_Hz) && defined(RCC_CFGR_I2SSRC)
    if (RCC_REG_BIT(CFGR,I2SSRC) == I2S_CLOCKSOURCE_EXT)
    {
        return EXTERNAL_CLOCK_VALUE_Hz;
    }
    else
#endif
    {
        return RCC_ulClockFreq_Hz(SYSCLK);
    }
}

/** @} */

/** @ingroup RTC_Clock_Source
 * @defgroup RTC_Clock_Source_Exported_Functions RTC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the RTC.
 * @param eClockSource: the new source clock which should be configured
 * @return Result of the operation
 */
XPD_ReturnType RTC_eClockConfig(RTC_ClockSourceType eClockSource)
{
    uint32_t ulBDCR;
    XPD_ReturnType eResult;
    uint32_t ulTimeout = RCC_DBP_TIMEOUT;

    /* enable write access to backup domain */
    PWR_REG_BIT(CR,DBP) = 1;

    /* wait for backup domain write protection disable */
    eResult = XPD_eWaitForMatch(&PWR->CR.w,
            PWR_CR_DBP, PWR_CR_DBP, &ulTimeout);

    /* reset the backup domain only if the RTC clock source selection is modified */
    if ((eResult == XPD_OK) && (RCC->BDCR.b.RTCSEL != eClockSource))
    {
        /* store the content of BDCR register before the reset of backup domain */
        ulBDCR = (RCC->BDCR.w & ~(RCC_BDCR_RTCSEL));

        /* RTC clock selection can be changed only if the backup domain is reset */
        RCC_REG_BIT(BDCR,BDRST) = 1;
        RCC_REG_BIT(BDCR,BDRST) = 0;

        /* restore the Content of BDCR register */
        RCC->BDCR.w = ulBDCR;

        /* wait for LSERDY if LSE was enabled */
        if ((ulBDCR & RCC_BDCR_LSERDY) != 0)
        {
            ulTimeout = RCC_LSE_TIMEOUT;

            eResult = XPD_eWaitForMatch(&RCC->BDCR.w,
                    RCC_BDCR_LSERDY, RCC_BDCR_LSERDY, &ulTimeout);
        }

        /* set clock source if no error was encountered */
        RCC->BDCR.b.RTCSEL = eClockSource;
    }
    return eResult;
}

/**
 * @brief Returns the input clock frequency of the RTC.
 * @return The clock frequency of the RTC in Hz
 */
uint32_t RTC_ulClockFreq_Hz(void)
{
    RTC_ClockSourceType eSrcClk;

    /* Get the current RTC source */
    eSrcClk = RCC->BDCR.b.RTCSEL;

#ifdef LSE_VALUE_Hz
    /* Check if LSE is ready and if RTC clock selection is LSE */
    if ((eSrcClk == RTC_CLOCKSOURCE_LSE) && (RCC_REG_BIT(BDCR,LSERDY) != 0))
    {
        return LSE_VALUE_Hz;
    }
    else
#endif
    /* Check if LSI is ready and if RTC clock selection is LSI */
    if ((eSrcClk == RTC_CLOCKSOURCE_LSI) && (RCC_REG_BIT(CSR,LSIRDY) != 0))
    {
        return LSI_VALUE_Hz;
    }
#ifdef HSE_VALUE_Hz
    /* Check if HSE is ready  and if RTC clock selection is HSE / x */
    else if ((eSrcClk == RTC_CLOCKSOURCE_HSE_DIV32) && (RCC_REG_BIT(CR,HSERDY) != 0))
    {
        return HSE_VALUE_Hz / 32;
    }
#endif
    /* Clock not enabled for RTC */
    else
    {
        return 0;
    }
}

/** @} */

#if defined(SDADC1)
/** @ingroup SDADC_Clock_Source
 * @defgroup SDADC_Clock_Source_Exported_Functions SDADC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the SDADC.
 * @param eClockSource: the new source clock which should be configured
 */
void SDADC_vClockConfig(SDADC_ClockSourceType eClockSource)
{
    RCC->CFGR.b.SDADCPRE = eClockSource;
}

/**
 * @brief Returns the input clock frequency of the SDADC.
 * @return The clock frequency of the SDADC in Hz
 */
uint32_t SDADC_ulClockFreq_Hz(void)
{
    uint32_t ulFreq = RCC_ulClockFreq_Hz(SYSCLK);
    SDADC_ClockSourceType eSource = RCC->CFGR.b.SDADCPRE;

    if (eSource > SDADC_CLOCKSOURCE_SYSCLK_DIV20)
    {
        ulFreq /= ((eSource - SDADC_CLOCKSOURCE_SYSCLK_DIV24) * 4) + 24;
    }
    else if (eSource > SDADC_CLOCKSOURCE_SYSCLK)
    {
        ulFreq /= ((eSource - SDADC_CLOCKSOURCE_SYSCLK_DIV2) * 2) + 2;
    }
    return ulFreq;
}

/** @} */
#endif /* SDADC1 */

/** @ingroup TIM_Clock_Source
 * @defgroup TIM_Clock_Source_Exported_Functions TIM Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the selected timer.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eClockSource: the new source clock which should be configured
 */
void XPD_TIM_ClockConfig(TIM_HandleType * pxTIM, TIM_ClockSourceType eClockSource)
{
    switch ((uint32_t)pxTIM->Inst)
    {
#ifdef RCC_CFGR3_TIM1SW
        case TIM1_BASE:
            RCC_REG_BIT(CFGR3, TIM1SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM2SW
        case TIM2_BASE:
            RCC_REG_BIT(CFGR3, TIM2SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM8SW
        case TIM8_BASE:
            RCC_REG_BIT(CFGR3, TIM8SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM15SW
        case TIM15_BASE:
            RCC_REG_BIT(CFGR3, TIM15SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM16SW
        case TIM16_BASE:
            RCC_REG_BIT(CFGR3, TIM16SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM17SW
        case TIM17_BASE:
            RCC_REG_BIT(CFGR3, TIM17SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM20SW
        case TIM20_BASE:
            RCC_REG_BIT(CFGR3, TIM20SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_TIM34SW
        case TIM34_BASE:
            RCC_REG_BIT(CFGR3, TIM34SW) = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_HRTIMSW
        case HRTIM1_BASE:
            RCC_REG_BIT(CFGR3, HRTIMSW) = eClockSource;
            break;
#endif
        default:
            break;
    }
}

/**
 * @brief Returns the input clock frequency of the timer.
 * @param pxTIM: pointer to the TIM handle structure
 * @return The clock frequency of the timer in Hz
 */
uint32_t TIM_ulClockFreq_Hz(TIM_HandleType * pxTIM)
{
    uint32_t ulFreq;

    switch ((uint32_t)pxTIM->Inst)
    {
#ifdef RCC_CFGR3_TIM1SW
        case TIM1_BASE:
            if (RCC_REG_BIT(CFGR3, TIM1SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM2SW
        case TIM2_BASE:
            if (RCC_REG_BIT(CFGR3, TIM2SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM8SW
        case TIM8_BASE:
            if (RCC_REG_BIT(CFGR3, TIM8SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM15SW
        case TIM15_BASE:
            if (RCC_REG_BIT(CFGR3, TIM15SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM16SW
        case TIM16_BASE:
            if (RCC_REG_BIT(CFGR3, TIM16SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM17SW
        case TIM17_BASE:
            if (RCC_REG_BIT(CFGR3, TIM17SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM20SW
        case TIM20_BASE:
            if (RCC_REG_BIT(CFGR3, TIM20SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_TIM34SW
        case TIM34_BASE:
            if (RCC_REG_BIT(CFGR3, TIM34SW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
#ifdef RCC_CFGR3_HRTIMSW
        case HRTIM1_BASE:
            if (RCC_REG_BIT(CFGR3, HRTIMSW))
            {
                return RCC_ulOscFreq_Hz(PLL) * 2;
            }
            break;
#endif
        default:
            break;
    }
    /* decide which APB bus the TIM is on */
    if (((uint32_t)pxTIM->Inst) < APB2PERIPH_BASE)
    {
        {
            ulFreq = RCC_ulClockFreq_Hz(PCLK1);

            /* if APB clock is divided, timer frequency is doubled */
            if ((RCC->CFGR.w & RCC_CFGR_PPRE1) != 0)
            {
                ulFreq *= 2;
            }
        }
    }
    else
    {
        {
            ulFreq = RCC_ulClockFreq_Hz(PCLK2);

            /* if APB clock is divided, timer frequency is doubled */
            if ((RCC->CFGR.w & RCC_CFGR_PPRE2) != 0)
            {
                ulFreq *= 2;
            }
        }
    }
    return ulFreq;
}

/** @} */

/** @ingroup USART_Clock_Source
 * @defgroup USART_Clock_Source_Exported_Functions USART Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the selected USART.
 * @param pxUSART: pointer to the USART handle structure
 * @param eClockSource: the new source clock which should be configured
 */
void USART_vClockConfig(USART_HandleType * pxUSART, USART_ClockSourceType eClockSource)
{
    switch ((uint32_t)pxUSART->Inst)
    {
#ifdef RCC_CFGR3_USART1SW
        case USART1_BASE:
            RCC->CFGR3.b.USART1SW = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_USART2SW
        case USART2_BASE:
            RCC->CFGR3.b.USART2SW = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_USART3SW
        case USART3_BASE:
            RCC->CFGR3.b.USART3SW = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_UART4SW
        case UART4_BASE:
            RCC->CFGR3.b.UART4SW = eClockSource;
            break;
#endif
#ifdef RCC_CFGR3_UART5SW
        case UART5_BASE:
            RCC->CFGR3.b.UART5SW = eClockSource;
            break;
#endif
        default:
            break;
    }
}

/**
 * @brief Returns the input clock frequency of the USART.
 * @param pxUSART: pointer to the USART handle structure
 * @return The clock frequency of the USART in Hz
 */
uint32_t USART_ulClockFreq_Hz(USART_HandleType * pxUSART)
{
    USART_ClockSourceType eSource;

    switch ((uint32_t)pxUSART->Inst)
    {
#ifdef RCC_CFGR3_USART1SW
        case USART1_BASE:
            eSource = RCC->CFGR3.b.USART1SW;
            break;
#endif
#ifdef RCC_CFGR3_USART2SW
        case USART2_BASE:
            eSource = RCC->CFGR3.b.USART2SW;
            break;
#endif
#ifdef RCC_CFGR3_USART3SW
        case USART3_BASE:
            eSource = RCC->CFGR3.b.USART3SW;
            break;
#endif
#ifdef RCC_CFGR3_UART4SW
        case UART4_BASE:
            eSource = RCC->CFGR3.b.UART4SW;
            break;
#endif
#ifdef RCC_CFGR3_UART5SW
        case UART5_BASE:
            eSource = RCC->CFGR3.b.UART5SW;
            break;
#endif
        default:
            eSource = USART_CLOCKSOURCE_PCLKx;
            break;
    }
    /* get the value for the configured source */
    switch (eSource)
    {
        case USART_CLOCKSOURCE_SYSCLK:
            return RCC_ulClockFreq_Hz(SYSCLK);

        case USART_CLOCKSOURCE_HSI:
            return HSI_VALUE_Hz;

#ifdef LSE_VALUE_Hz
        case USART_CLOCKSOURCE_LSE:
            return LSE_VALUE_Hz;
#endif

        default:
            return RCC_ulClockFreq_Hz((((uint32_t)pxUSART->Inst) < APB2PERIPH_BASE) ? PCLK1 : PCLK2);
    }
}

/** @} */

#if defined(USB)

/** @ingroup USB_Clock_Source
 * @defgroup USB_Clock_Source_Exported_Functions USB Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the USB.
 * @param eClockSource: the new source clock which should be configured
 */
void USB_vClockConfig(USB_ClockSourceType eClockSource)
{
    RCC_REG_BIT(CFGR, USBPRE) = eClockSource;
}

/** @} */

#endif /* USB */
