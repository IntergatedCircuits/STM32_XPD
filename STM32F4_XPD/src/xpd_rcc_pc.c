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
#include <xpd_i2c.h>
#include <xpd_pwr.h>
#include <xpd_rtc.h>
#include <xpd_tim.h>
#include <xpd_usart.h>
#include <xpd_utils.h>

/** @ingroup I2C_Clock_Source
 * @defgroup I2C_Clock_Source_Exported_Functions I2C Clock Source Exported Functions
 * @{ */

/**
 * @brief Returns the input clock frequency of the I2C.
 * @param hi2c: pointer to the I2C handle structure
 * @return The clock frequency of the I2C in Hz
 */
uint32_t I2C_ulClockFreq_Hz(I2C_HandleType * hi2c)
{
    return RCC_ulClockFreq_Hz(PCLK1);
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
XPD_ReturnType XPD_RTC_eClockConfig(RTC_ClockSourceType eClockSource)
{
    uint32_t ulBDCR;
    XPD_ReturnType eResult;
    uint32_t ulTimeout = RCC_DBP_TIMEOUT;

    /* enable write access to backup domain */
    PWR_REG_BIT(CR,DBP) = 1;

    /* wait for backup domain write protection disable */
    eResult = XPD_eWaitForMatch(&PWR->CR.w, PWR_CR_DBP, PWR_CR_DBP, &ulTimeout);

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

#ifdef HSE_VALUE_Hz
        /* if HSE is clock source, must set divider in order to get 1 MHz */
        if (eClockSource == RTC_CLOCKSOURCE_HSE)
        {
            uint32_t ulRTCPRE = HSE_VALUE_Hz / 1000000;

            /* check if 1 MHz can be configured */
            if ((ulRTCPRE < 2) || (ulRTCPRE > 31) ||
                ((ulRTCPRE * 1000000) != HSE_VALUE_Hz))
            {
                return XPD_ERROR;
            }
            else
            {
                RCC->CFGR.b.RTCPRE = ulRTCPRE;
            }
        }
#endif

        /* set clock source if no error was encountered */
        RCC->BDCR.b.RTCSEL = eClockSource;
    }
    return eResult;
}

/**
 * @brief Returns the input clock frequency of the RTC.
 * @return The clock frequency of the RTC in Hz
 */
uint32_t XPD_RTC_ulClockFreq_Hz(void)
{
    uint32_t eSrcClk;

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
    else if ((eSrcClk == RTC_CLOCKSOURCE_HSE) && (RCC_REG_BIT(CR,HSERDY) != 0))
    {
        return HSE_VALUE_Hz / RCC->CFGR.b.RTCPRE;
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

#ifdef RCC_DCKCFGR_TIMPRE
/**
 * @brief Sets the new source clock for the timers.
 * @param eClockSource: the new source clock which should be configured
 */
void TIM_vClockConfig(TIM_ClockSourceType eClockSource)
{
    RCC_REG_BIT(DCKCFGR,TIMPRE) = eClockSource;
}
#endif /* RCC_DCKCFGR_TIMPRE */

/**
 * @brief Returns the input clock frequency of the timer.
 * @param htim: pointer to the TIM handle structure
 * @return The clock frequency of the timer in Hz
 */
uint32_t TIM_ulClockFreq_Hz(TIM_HandleType * htim)
{
    uint32_t ulFreq;

    /* decide which APB bus the TIM is on */
    if (((uint32_t)htim->Inst) < APB2PERIPH_BASE)
    {
#ifdef RCC_DCKCFGR_TIMPRE
        /* if the TIMPRE bit is set, the TIM clock is HCLK when PPREx is DIV1, 2 or 4
         * otherwise it is 4 * PCLKx */
        if (RCC_REG_BIT(DCKCFGR,TIMPRE) == 1)
        {
            if (RCC->CFGR.b.PPRE1 < 6) /* CLK_DIV8 for PPREx */
            {
                ulFreq = RCC_ulClockFreq_Hz(HCLK);
            }
            else
            {
                ulFreq = 4 * RCC_ulClockFreq_Hz(PCLK1);
            }
        }
        else
#endif
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
#ifdef RCC_DCKCFGR_TIMPRE
        /* if the TIMPRE bit is set, the TIM clock is HCLK when PPREx is DIV1, 2 or 4
         * otherwise it is 4 * PCLKx */
        if (RCC_REG_BIT(DCKCFGR,TIMPRE) == 1)
        {
            if (RCC->CFGR.b.PPRE2 < 6) /* CLK_DIV8 for PPREx */
            {
                ulFreq = RCC_ulClockFreq_Hz(HCLK);
            }
            else
            {
                ulFreq = 4 * RCC_ulClockFreq_Hz(PCLK2);
            }
        }
        else
#endif
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
 * @brief Returns the input clock frequency of the USART.
 * @param husart: pointer to the USART handle structure
 * @return The clock frequency of the USART in Hz
 */
uint32_t USART_ulClockFreq_Hz(USART_HandleType * husart)
{
    return RCC_ulClockFreq_Hz((((uint32_t)husart->Inst) < APB2PERIPH_BASE) ? PCLK1 : PCLK2);
}

/** @} */
