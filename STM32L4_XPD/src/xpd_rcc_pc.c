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
#include <xpd_tim.h>
#include <xpd_usart.h>
#include <xpd_usb.h>
#include <xpd_utils.h>

#ifdef RCC_PLLP_DIV_2_31_SUPPORT
#define RCC_PLLP_FREQ(PLL_NAME) \
    (RCC_ulOscFreq_Hz(XPD_RCC_GetPLLSource()) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * RCC->PLL_NAME##CFGR.b.PLL_NAME##PDIV))
#else
#define RCC_PLLP_FREQ(PLL_NAME) \
    (RCC_ulOscFreq_Hz(XPD_RCC_GetPLLSource()) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * ((RCC->PLL_NAME##CFGR.b.PLL_NAME##P == 0) ? 7 : 17)))
#endif

#define RCC_PLLR_FREQ(PLL_NAME) \
    (RCC_ulOscFreq_Hz(RCC->PLLCFGR.b.PLLSRC - 1) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * (2 * (RCC->PLL_NAME##CFGR.b.PLL_NAME##R + 1))))

#define RCC_PLLQ_FREQ(PLL_NAME) \
    (RCC_ulOscFreq_Hz(RCC->PLLCFGR.b.PLLSRC - 1) * RCC->PLL_NAME##CFGR.b.PLL_NAME##N \
    / (RCC->PLLCFGR.b.PLLM * (2 * (RCC->PLL_NAME##CFGR.b.PLL_NAME##Q + 1))))

/** @ingroup ADC_Clock_Source
 * @defgroup ADC_Clock_Source_Exported_Functions ADC Clock Source Exported Functions
 * @{ */

/* gets the clock division value from the prescaler configuration */
static const uint8_t adc_aucClkPreTable[] = { 0, 1, 3, 5, 7, 9, 11, 15, 31, 63, 127, 255 };

/**
 * @brief Sets the new source clock and clock divider for the ADCs.
 * @param eClockSource: the new source clock which should be configured
 * @param eClockDivider: the new clock divider value
 *        Permitted range is [DIV1..DIV4] for HCLK, [DIV1..DIV256] for asynchronous clock sources
 */
void ADC_vClockConfig(ADC_ClockSourceType eClockSource, ClockDividerType eClockDivider)
{
    uint32_t ulADCSEL = eClockSource >> 4;
    /* Set dedicated input multiplexer */
    RCC->CCIPR.b.ADCSEL = ulADCSEL;

    /* Enable PLLSAI R when selected, otherwise disable */
    RCC_REG_BIT(PLLSAI1CFGR,PLLSAI1REN) = (ulADCSEL == 1) ? 1 : 0;
#ifdef RCC_PLLSAI2_SUPPORT
    RCC_REG_BIT(PLLSAI2CFGR,PLLSAI2REN) = (ulADCSEL == 2) ? 1 : 0;
#endif

    RCC_vClockEnable(RCC_POS_ADC);

    /* Peripheral configuration can only be applied when ADC is in OFF state */
    if (   ((ADC1->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
#ifdef ADC123_COMMON
        && ((ADC2->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
        && ((ADC3->CR.w & (ADC_CR_ADSTART | ADC_CR_ADEN)) == 0)
#endif
        )
    {
        /* In case of synchronous clock, the divider is set in CKMODE */
        if (eClockSource < ADC_CLOCKSOURCE_PLLSAI1)
        {
            ADC_COMMON()->CCR.b.CKMODE = eClockSource;
        }
        else
        {
            /* Select asynchronous clock, configure prescaler */
            ADC_COMMON()->CCR.b.CKMODE = 0;
            ADC_COMMON()->CCR.b.PRESC  = eClockSource;
        }
    }
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t ADC_ulClockFreq_Hz(void)
{
    uint32_t ulCKMODE = ADC_COMMON()->CCR.b.CKMODE;
    uint32_t ulADCSEL = RCC->CCIPR.b.ADCSEL;

    if (ulCKMODE != 0)
    {
        /* Calculate synchronous clock frequency */
        return RCC_ulClockFreq_Hz(HCLK) / (1 << (ulCKMODE - 1));
    }
    else
    {
        uint32_t ulFreq;

        /* Calculate asynchronous clock frequency */
        switch (ulADCSEL)
        {
            case 3:
                ulFreq = RCC_ulClockFreq_Hz(SYSCLK);
                break;
#ifdef RCC_PLLSAI2_SUPPORT
            case 2:
                ulFreq = RCC_PLLR_FREQ(PLLSAI2);
                break;
#endif
            case 1:
                ulFreq = RCC_PLLR_FREQ(PLLSAI1);
                break;

            default:
                return 0;
        }
        return ulFreq / ((uint32_t)adc_aucClkPreTable[ulADCSEL & 0xF] + 1);
    }
}

/** @} */

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
#ifdef RCC_CCIPR_I2C1SEL
        case I2C1_BASE:
            RCC->CCIPR.b.I2C1SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR_I2C2SEL
        case I2C2_BASE:
            RCC->CCIPR.b.I2C2SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR_I2C3SEL
        case I2C3_BASE:
            RCC->CCIPR.b.I2C3SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR2_I2C4SEL
        case I2C4_BASE:
            RCC->CCIPR2.b.I2C4SEL = eClockSource;
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
#ifdef RCC_CCIPR_I2C1SEL
        case I2C1_BASE:
            eSource = RCC->CCIPR.b.I2C1SEL;
            break;
#endif
#ifdef RCC_CCIPR_I2C2SEL
        case I2C2_BASE:
            eSource = RCC->CCIPR.b.I2C2SEL;
            break;
#endif
#ifdef RCC_CCIPR_I2C3SEL
        case I2C3_BASE:
            eSource = RCC->CCIPR.b.I2C3SEL;
            break;
#endif
#ifdef RCC_CCIPR2_I2C4SEL
        case I2C4_BASE:
            eSource = RCC->CCIPR2.b.I2C4SEL;
            break;
#endif
        default:
            eSource = I2C_CLOCKSOURCE_PCLKx;
            break;
    }
    /* get the value for the configured source */
    switch (eSource)
    {
        case I2C_CLOCKSOURCE_HSI:
            return HSI_VALUE_Hz;
        case I2C_CLOCKSOURCE_SYSCLK:
            return RCC_ulClockFreq_Hz(SYSCLK);
        default:
            return RCC_ulClockFreq_Hz((((uint32_t)pxI2C->Inst) < APB2PERIPH_BASE) ? PCLK1 : PCLK2);
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
    PWR_REG_BIT(CR1,DBP) = 1;

    /* wait for backup domain write protection disable */
    eResult = XPD_eWaitForMatch(&PWR->CR1.w,
            PWR_CR1_DBP, PWR_CR1_DBP, &ulTimeout);

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

/** @ingroup TIM_Clock_Source
 * @defgroup TIM_Clock_Source_Exported_Functions TIM Clock Source Exported Functions
 * @{ */

/**
 * @brief Returns the input clock frequency of the timer.
 * @param pxTIM: pointer to the TIM handle structure
 * @return The clock frequency of the timer in Hz
 */
uint32_t TIM_ulClockFreq_Hz(TIM_HandleType * pxTIM)
{
    uint32_t ulFreq;

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
#ifdef RCC_CCIPR_USART1SEL
        case USART1_BASE:
            RCC->CCIPR.b.USART1SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR_USART2SEL
        case USART2_BASE:
            RCC->CCIPR.b.USART2SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR_USART3SEL
        case USART3_BASE:
            RCC->CCIPR.b.USART3SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR_UART4SEL
        case UART4_BASE:
            RCC->CCIPR.b.UART4SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR_UART5SEL
        case UART5_BASE:
            RCC->CCIPR.b.UART5SEL = eClockSource;
            break;
#endif
#ifdef RCC_CCIPR_LPUART1SEL
        case LPUART1_BASE:
            RCC->CCIPR.b.LPUART1SEL = eClockSource;
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
#ifdef RCC_CCIPR_USART1SEL
        case USART1_BASE:
            eSource = RCC->CCIPR.b.USART1SEL;
            break;
#endif
#ifdef RCC_CCIPR_USART2SEL
        case USART2_BASE:
            eSource = RCC->CCIPR.b.USART2SEL;
            break;
#endif
#ifdef RCC_CCIPR_USART3SEL
        case USART3_BASE:
            eSource = RCC->CCIPR.b.USART3SEL;
            break;
#endif
#ifdef RCC_CCIPR_UART4SEL
        case UART4_BASE:
            eSource = RCC->CCIPR.b.UART4SEL;
            break;
#endif
#ifdef RCC_CCIPR_UART5SEL
        case UART5_BASE:
            eSource = RCC->CCIPR.b.UART5SEL;
            break;
#endif
#ifdef RCC_CCIPR_LPUART1SEL
        case LPUART1_BASE:
            eSource = RCC->CCIPR.b.LPUART1SEL;
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

#if defined(USB) || defined(USB_OTG_FS)

/** @ingroup USB_Clock_Source
 * @defgroup USB_Clock_Source_Exported_Functions USB Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the USB.
 * @param eClockSource: the new source clock which should be configured
 */
void USB_vClockConfig(USB_ClockSourceType eClockSource)
{
    RCC->CCIPR.b.CLK48SEL = eClockSource;

    switch (eClockSource)
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

#endif /* USB */
