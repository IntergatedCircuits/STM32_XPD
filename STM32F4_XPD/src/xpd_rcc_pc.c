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

#if defined(USE_XPD_RTC)
#include "xpd_pwr.h"
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

        /* if HSE is clock source, must set divider in order to get 1 MHz */
        if (ClockSource == RTC_CLOCKSOURCE_HSE)
        {
            uint32_t tmp = HSE_VALUE / 1000000;

            /* check if 1 MHz can be configured */
            if ((tmp < 2) || (tmp > 31) || ((tmp * 1000000) != HSE_VALUE))
            {
                return XPD_ERROR;
            }
            else
            {
                RCC->CFGR.b.RTCPRE = tmp;
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

    /* Check if LSE is ready and if RTC clock selection is LSE */
    if ((srcclk == RTC_CLOCKSOURCE_LSE) && (RCC_REG_BIT(BDCR,LSERDY) != 0))
    {
        return LSE_VALUE;
    }
    /* Check if LSI is ready and if RTC clock selection is LSI */
    else if ((srcclk == RTC_CLOCKSOURCE_LSI) && (RCC_REG_BIT(CSR,LSIRDY) != 0))
    {
        return LSI_VALUE;
    }
    /* Check if HSE is ready  and if RTC clock selection is HSE / x */
    else if ((srcclk == RTC_CLOCKSOURCE_HSE) && (RCC_REG_BIT(CR,HSERDY) != 0))
    {
        return HSE_VALUE / RCC->CFGR.b.RTCPRE;
    }
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

/** @addtogroup TIM
 * @{ */

/** @addtogroup TIM_Clock_Source
 * @{ */

/** @defgroup TIM_Clock_Source_Exported_Functions TIM Clock Source Exported Functions
 * @{ */

#ifdef RCC_DCKCFGR_TIMPRE
/**
 * @brief Sets the new source clock for the timers.
 * @param ClockSource: the new source clock which should be configured
 */
void XPD_TIM_ClockConfig(TIM_ClockSourceType ClockSource)
{
    RCC_REG_BIT(DCKCFGR,TIMPRE) = ClockSource;
}
#endif /* RCC_DCKCFGR_TIMPRE */

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
#ifdef RCC_DCKCFGR_TIMPRE
        /* if the TIMPRE bit is set, the TIM clock is HCLK when PPREx is DIV1, 2 or 4
         * otherwise it is 4 * PCLKx */
        if (RCC_REG_BIT(DCKCFGR,TIMPRE) == 1)
        {
            if (RCC->CFGR.b.PPRE1 < 6) /* CLK_DIV8 for PPREx */
            {
                freq = XPD_RCC_GetClockFreq(HCLK);
            }
            else
            {
                freq = 4 * XPD_RCC_GetClockFreq(PCLK1);
            }
        }
        else
#endif
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
#ifdef RCC_DCKCFGR_TIMPRE
        /* if the TIMPRE bit is set, the TIM clock is HCLK when PPREx is DIV1, 2 or 4
         * otherwise it is 4 * PCLKx */
        if (RCC_REG_BIT(DCKCFGR,TIMPRE) == 1)
        {
            if (RCC->CFGR.b.PPRE2 < 6) /* CLK_DIV8 for PPREx */
            {
                freq = XPD_RCC_GetClockFreq(HCLK);
            }
            else
            {
                freq = 4 * XPD_RCC_GetClockFreq(PCLK2);
            }
        }
        else
#endif
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

/** @addtogroup USART
 * @{ */

/** @addtogroup USART_Clock_Source
 * @{ */

/** @defgroup USART_Clock_Source_Exported_Functions USART Clock Source Exported Functions
 * @{ */

/**
 * @brief Returns the input clock frequency of the USART.
 * @param husart: pointer to the USART handle structure
 * @return The clock frequency of the USART in Hz
 */
uint32_t XPD_USART_GetClockFreq(USART_HandleType * husart)
{
    return XPD_RCC_GetClockFreq((((uint32_t)husart->Inst) < APB2PERIPH_BASE) ? PCLK1 : PCLK2);
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_USART */
