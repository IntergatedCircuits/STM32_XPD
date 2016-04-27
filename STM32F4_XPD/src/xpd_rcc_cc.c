/**
  ******************************************************************************
  * @file    xpd_rcc_cc.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-17
  * @brief   STM32 eXtensible Peripheral Drivers RCC Core Clocks Module
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
#include "xpd_gpio.h"
#include "xpd_pwr.h"
#include "xpd_utils.h"
#include "xpd_flash.h"

/** @addtogroup RCC
 * @{ */

/** @addtogroup RCC_Core_Clocks RCC Core
 * @{ */

/* converts the general clock divider to the selected clock type's format */
static uint32_t rcc_convertClockDivider(RCC_ClockType clockType, ClockDividerType divider)
{
    if (divider == CLK_DIV1)
    {
        return (uint32_t)CLK_DIV1;
    }
    else
    {
        /* actual division values start from 0, with the MSB bit set */
        divider--;
        if (clockType == HCLK)
        {
            if (divider > CLK_DIV16)
            {
                divider--; /* skip DIV32 */
            }
            return (uint32_t)divider | 0x8;
        }
        else /* ((clockType == PCLK1) || (clockType == PCLK2)) || MCO */
        {
            return (uint32_t)divider | 0x4;
        }
    }
}

/** @defgroup RCC_Core_Clocks_Exported_Functions RCC Core Exported Functions
 * @{ */

/** @defgroup RCC_Core_Clocks_Exported_Functions_Oscillators RCC Oscillator Functions
 *  @brief    RCC oscillator and PLL control
 * @{
 */

/**
 * Configures the high speed internal oscillator.
 * @param Config: pointer to the configuration parameters
 * @return Result of the operation
 */
XPD_ReturnType XPD_RCC_HSIConfig(RCC_HSI_InitType * Config)
{
    XPD_ReturnType result = XPD_OK;
    RCC_OscType sysclock = XPD_RCC_GetSYSCLKSource();

    /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
    if (     (sysclock == HSI)
         || ((sysclock == PLL) && (XPD_RCC_GetPLLSource() == HSI))
#ifdef RCC_PLLCFGR_PLLR
         || ((sysclock == PLLR) && (XPD_RCC_GetPLLSource() == HSI))
#endif
    )
    {
        /* When HSI is used as system clock it will not disabled */
        if ((RCC_REG_BIT(CR,HSIRDY) != 0) && (Config->State != OSC_ON))
        {
            result = XPD_ERROR;
        }
        /* Otherwise, just the calibration is allowed */
        else
        {
            /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
            RCC->CR.b.HSICAL = Config->CalibrationValue;
        }
    }
    else
    {
        /* Check the HSI State */
        if (Config->State != OSC_OFF)
        {
            /* Enable the Internal High Speed oscillator (HSI). */
            RCC_REG_BIT(CR,HSION) = OSC_ON;

            /* Wait until HSI is ready */
            result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_HSIRDY, RCC_CR_HSIRDY, RCC_HSI_TIMEOUT);

            /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
            RCC->CR.b.HSICAL = Config->CalibrationValue;
        }
        else
        {
            /* Disable the Internal High Speed oscillator (HSI). */
            XPD_RCC_HSIConfig(OSC_OFF);

            /* Wait until HSI is disabled */
            result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_HSIRDY, 0, RCC_HSI_TIMEOUT);
        }
    }
    return result;
}

/**
 * Configures the high speed external oscillator.
 * @param Config: pointer to the configuration parameters
 * @return Result of the operation
 */
XPD_ReturnType XPD_RCC_HSEConfig(RCC_HSE_InitType * Config)
{
    XPD_ReturnType result = XPD_OK;
    RCC_OscType sysclock = XPD_RCC_GetSYSCLKSource();

    /* When the HSE is used as system clock or clock source for PLL in these cases it is not allowed to be disabled */
    if (     (sysclock == HSE)
         || ((sysclock == PLL) && (XPD_RCC_GetPLLSource() == HSE))
#ifdef RCC_PLLCFGR_PLLR
         || ((sysclock == PLLR) && (XPD_RCC_GetPLLSource() == HSE))
#endif
    )
    {
        if ((RCC_REG_BIT(CR,HSERDY) != 0) && (Config->State == OSC_OFF))
        {
            result = XPD_ERROR;
        }
    }
    else
    {
        /* Reset HSEON and HSEBYP bits before configuring the HSE */
        RCC_REG_BIT(CR,HSEON) = 0;
        RCC_REG_BIT(CR,HSEBYP) = 0;

        /* Wait until HSE is disabled */
        result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_HSERDY, 0, RCC_HSE_TIMEOUT);

        if ((result == XPD_OK) && (Config->State != OSC_OFF))
        {
            switch(Config->State)
            {
            case OSC_ON:
            default:
                RCC_REG_BIT(CR,HSEON) = 1;
                break;

            case OSC_BYPASS:
                RCC_REG_BIT(CR,HSEON) = 1;
                RCC_REG_BIT(CR,HSEBYP) = 1;
                break;
            }

            /* Wait until HSE is ready */
            result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_HSERDY, RCC_CR_HSERDY, RCC_HSE_TIMEOUT);
        }
    }
    return result;
}

/**
 * Configures the phase locked loop.
 * @param Config: pointer to the configuration parameters
 * @return Result of the operation
 */
XPD_ReturnType XPD_RCC_PLLConfig(RCC_PLL_InitType * Config)
{
    XPD_ReturnType result;
    RCC_OscType sysclock = XPD_RCC_GetSYSCLKSource();

    /* Check if the PLL is used as system clock or not */
    if (sysclock == PLL)
    {
        result = XPD_ERROR;
    }
    else
    {
        /* Disable the main PLL. */
        RCC_REG_BIT(CR,PLLON) = OSC_OFF;

        /* Wait until PLL is disabled */
        result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_PLLRDY, 0, RCC_PLL_TIMEOUT);

        if ((result == XPD_OK) && (Config->State != OSC_OFF))
        {
            /* Configure the main PLL clock source and multiplication factor. */
            RCC_REG_BIT(PLLCFGR,PLLSRC) = Config->Source;

            RCC->PLLCFGR.b.PLLM = Config->M;
            RCC->PLLCFGR.b.PLLN = Config->N;
            RCC->PLLCFGR.b.PLLP = (Config->P >> 1) - 1;
            RCC->PLLCFGR.b.PLLQ = Config->Q;
#ifdef RCC_PLLCFGR_PLLR
            RCC->PLLCFGR.b.PLLR = Config->R;
#endif

            /* Enable the main PLL. */
            RCC_REG_BIT(CR,PLLON) = OSC_ON;

            /* Wait until PLL is ready */
            result = XPD_WaitForMatch(&RCC->CR.w, RCC_CR_PLLRDY, RCC_CR_PLLRDY, RCC_PLL_TIMEOUT);
        }
    }
    return result;
}

/**
 * Sets the new state of the low speed internal oscillator.
 * @param NewState: the new operation state
 * @return Result of the operation
 */
XPD_ReturnType XPD_RCC_LSIConfig(RCC_OscStateType NewState)
{
    XPD_ReturnType result = XPD_OK;

    /* Check the LSI State */
    if (NewState != OSC_OFF)
    {
        /* Enable the Internal Low Speed oscillator (LSI). */
        RCC_REG_BIT(CSR,LSION) = OSC_ON;

        /* Wait until LSI is ready */
        result = XPD_WaitForMatch(&RCC->CSR.w, RCC_CSR_LSIRDY, RCC_CSR_LSIRDY, RCC_LSI_TIMEOUT);
    }
    else
    {
        /* Disable the Internal Low Speed oscillator (LSI). */
        RCC_REG_BIT(CSR,LSION) = OSC_OFF;

        /* Wait until LSI is disabled */
        result = XPD_WaitForMatch(&RCC->CSR.w, RCC_CSR_LSIRDY, 0, RCC_LSI_TIMEOUT);
    }
    return result;
}

/**
 * Sets the new state of the low speed external oscillator.
 * @param NewState: the new operation state
 * @return Result of the operation
 */
XPD_ReturnType XPD_RCC_LSEConfig(RCC_OscStateType NewState)
{
    XPD_ReturnType result = XPD_OK;

    /* Enable Power Clock*/
    XPD_PWR_ClockCtrl(ENABLE);

    /* Enable write access to Backup domain */
    PWR_REG_BIT(CR,DBP) = 1;

    /* Wait for Backup domain Write protection disable */
    result = XPD_WaitForMatch(&PWR->CR.w, PWR_CR_DBP, 1, RCC_DBP_TIMEOUT);
    if (result != XPD_OK)
    {
        return result;
    }

    /* Reset LSEON and LSEBYP bits before configuring the LSE */
    RCC_REG_BIT(BDCR,LSEON) = 0;
    RCC_REG_BIT(BDCR,LSEBYP) = 0;

    /* Wait until LSE is disabled */
    result = XPD_WaitForMatch(&RCC->BDCR.w, RCC_BDCR_LSERDY, 0, RCC_LSE_TIMEOUT);

    /* Check the LSE State */
    if ((result == XPD_OK) && (NewState != OSC_OFF))
    {
        switch(NewState)
        {
        case OSC_ON:
        default:
            RCC_REG_BIT(BDCR,LSEON) = 1;
            break;

        case OSC_BYPASS:
            RCC_REG_BIT(BDCR,LSEON) = 1;
            RCC_REG_BIT(BDCR,LSEBYP) = 1;
            break;
        }

        /* Wait until LSE is ready */
        result = XPD_WaitForMatch(&RCC->BDCR.w, RCC_BDCR_LSERDY, RCC_BDCR_LSERDY, RCC_LSE_TIMEOUT);
    }
    return result;
}

/**
 * @brief Gets the input oscillator of the PLL.
 * @return The high speed oscillator connected to the PLL
 */
RCC_OscType XPD_RCC_GetPLLSource(void)
{
    return RCC_REG_BIT(PLLCFGR,PLLSRC);
}

/**
 * @brief Gets the input oscillator of SYSCLK.
 * @return The oscillator connected to SYSCLK
 */
RCC_OscType XPD_RCC_GetSYSCLKSource(void)
{
    return RCC->CFGR.b.SWS;
}

/**
 * @brief Gets the oscillator frequency.
 * @param Oscillator: the selected oscillator
 * @return The frequency of the oscillator in Hz.
 */
uint32_t XPD_RCC_GetOscFreq(RCC_OscType Oscillator)
{
    uint32_t m, n, p;

    switch (Oscillator)
    {
        case HSE:
            return HSE_VALUE;

        case HSI:
            return HSI_VALUE;

        case PLL:
        {
            m = RCC->PLLCFGR.b.PLLM;
            n = RCC->PLLCFGR.b.PLLN;
            p = (RCC->PLLCFGR.b.PLLP + 1) * 2;

            if (XPD_RCC_GetPLLSource() != HSI)
            {
                return HSE_VALUE / m * n / p;
            }
            else
            {
                return HSI_VALUE / m * n / p;
            }
        }
#ifdef RCC_CFGR_SWS_PLLR
        case PLLR:
        {
            m = RCC->PLLCFGR.b.PLLM;
            n = RCC->PLLCFGR.b.PLLN;
            p = RCC->PLLCFGR.b.PLLR;

            if (XPD_RCC_GetPLLSource() != HSI)
            {
                return HSE_VALUE / m * n / p;
            }
            else
            {
                return HSI_VALUE / m * n / p;
            }
        }
#endif

        case LSI:
            return LSI_VALUE;

        case LSE:
            return LSE_VALUE;

        default:
            return 0;
    }
}

static RCC_OscType rcc_readyOscillator = HSI;

/**
 * @brief Gets the oscillator which was detected ready by the interrupt handler.
 * @return The oscillator which is currently ready
 */
RCC_OscType XPD_RCC_GetReadyOsc(void)
{
    return rcc_readyOscillator;
}

/**
 * @brief RCC interrupt handler that provides oscillator ready callback.
 */
void XPD_RCC_IRQHandler(void)
{
    uint32_t cir = RCC->CIR.w;

    /* Check RCC LSERDY flag  */
    if ((cir & (RCC_CIR_LSERDYF | RCC_CIR_LSERDYIE)) == (RCC_CIR_LSERDYF | RCC_CIR_LSERDYIE))
    {
        /* Clear RCC LSERDY pending bit */
        RCC_REG_BIT(CIR,LSERDYC) = 1;

        /* LSE Ready callback */
        rcc_readyOscillator = LSE;
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.OscReady,);
    }
    /* Check RCC LSIRDY flag  */
    if ((cir & (RCC_CIR_LSIRDYF | RCC_CIR_LSIRDYIE)) == (RCC_CIR_LSIRDYF | RCC_CIR_LSIRDYIE))
    {
        /* Clear RCC LSIRDY pending bit */
        RCC_REG_BIT(CIR,LSIRDYC) = 1;

        /* LSI Ready callback */
        rcc_readyOscillator = LSI;
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.OscReady,);
    }
    /* Check RCC PLLRDY flag  */
    if ((cir & (RCC_CIR_PLLRDYF | RCC_CIR_PLLRDYIE)) == (RCC_CIR_PLLRDYF | RCC_CIR_PLLRDYIE))
    {
        /* Clear RCC PLLRDY pending bit */
        RCC_REG_BIT(CIR,PLLRDYC) = 1;

        /* PLL Ready callback */
        rcc_readyOscillator = PLL;
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.OscReady,);
    }
    /* Check RCC HSERDY flag  */
    if ((cir & (RCC_CIR_HSERDYF | RCC_CIR_HSERDYIE)) == (RCC_CIR_HSERDYF | RCC_CIR_HSERDYIE))
    {
        /* Clear RCC HSERDY pending bit */
        RCC_REG_BIT(CIR,HSERDYC) = 1;

        /* HSE Ready callback */
        rcc_readyOscillator = HSE;
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.OscReady,);
    }
    /* Check RCC HSIRDY flag  */
    if ((cir & (RCC_CIR_HSIRDYF | RCC_CIR_HSIRDYIE)) == (RCC_CIR_HSIRDYF | RCC_CIR_HSIRDYIE))
    {
        /* Clear RCC HSIRDY pending bit */
        RCC_REG_BIT(CIR,HSIRDYC) = 1;

        /* HSI Ready callback */
        rcc_readyOscillator = HSI;
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.OscReady,);
    }
}

/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_CSS RCC Clock Security System
 *  @brief    RCC Clock Security System
 * @{
 */

/**
 * @brief Enables the Clock Security System
 */
void XPD_RCC_EnableCSS(void)
{
    RCC_REG_BIT(CR,CSSON) = 1;
}

/**
 * @brief Disables the Clock Security System
 */
void XPD_RCC_DisableCSS(void)
{
    RCC_REG_BIT(CR,CSSON) = 0;
}

/**
 * @brief RCC interrupt handler that provides Clock Security System callback.
 */
void XPD_NMI_IRQHandler(void)
{
    /* Check RCC CSSF flag  */
    if (RCC_REG_BIT(CIR,CSSF) != 0)
    {
        /* Clear RCC CSS pending bit */
        RCC_REG_BIT(CIR,CSSC) = 1;

        /* RCC Clock Security System interrupt user callback */
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.CSS,);
    }
}

/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_Clocks RCC Core Clocks Functions
 *  @brief    RCC core clocks control
 * @{
 */

static const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
#define APBPrescTable (&AHBPrescTable[4])

/**
 * @brief Sets the new configuration for the AHB system clocks and the new matching flash latency.
 * @param SYSCLK_Source: @ref RCC_ClockType::SYSCLK input source selection. Permitted values:
             @arg @ref RCC_OscType::HSI
             @arg @ref RCC_OscType::HSE
             @arg @ref RCC_OscType::PLL
 * @param HCLK_Divider: Clock divider of @ref RCC_ClockType::HCLK clock. Must not be CLK_DIV32.
 * @param FlashLatency: the desired amount of flash wait states
 * @return Result of the operation
 * @note To correctly read data from FLASH memory, the number of wait states must be
 * correctly programmed according to the frequency of the CPU clock (HCLK) of the device.
 */
XPD_ReturnType XPD_RCC_HCLKConfig(RCC_OscType SYSCLK_Source, ClockDividerType HCLK_Divider, uint8_t FlashLatency)
{
    XPD_ReturnType result;

    /* Increasing the CPU frequency */
    if (FlashLatency > XPD_FLASH_GetLatency())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        XPD_FLASH_SetLatency(FlashLatency);

        /* Check that the new number of wait states is taken into account to access the Flash
         memory by reading the FLASH_ACR register */
        if (XPD_FLASH_GetLatency() != FlashLatency)
        {
            return XPD_ERROR;
        }
    }

    RCC->CFGR.b.HPRE = rcc_convertClockDivider(HCLK, HCLK_Divider);

    switch (SYSCLK_Source)
    {
        case HSI:
            /* Check the HSI ready flag */
            if (RCC_REG_BIT(CR,HSIRDY) == 0)
            {
                return XPD_ERROR;
            }
            break;

        case HSE:
            /* Check the HSE ready flag */
            if (RCC_REG_BIT(CR,HSERDY) == 0)
            {
                return XPD_ERROR;
            }
            break;

        case PLL:
#ifdef RCC_CFGR_SWS_PLLR
        case PLLR:
#endif
            /* Check the PLL ready flag */
            if (RCC_REG_BIT(CR,PLLRDY) == 0)
            {
                return XPD_ERROR;
            }
            break;

        default:
            return XPD_ERROR;
    }

    RCC->CFGR.b.SW = SYSCLK_Source;

    /* wait until the settings have been processed */
    result = XPD_WaitForMatch(&RCC->CFGR.w, RCC_CFGR_SWS, SYSCLK_Source << 2, RCC_CLOCKSWITCH_TIMEOUT);
    if (result != XPD_OK)
    {
        return result;
    }

    /* Decreasing the CPU frequency */
    if (FlashLatency != XPD_FLASH_GetLatency())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        XPD_FLASH_SetLatency(FlashLatency);

        /* Check that the new number of wait states is taken into account to access the Flash
         memory by reading the FLASH_ACR register */
        if (XPD_FLASH_GetLatency() != FlashLatency)
        {
            return XPD_ERROR;
        }
    }

    /* Update SystemCoreClock variable */
    SystemCoreClock = XPD_RCC_GetOscFreq(SYSCLK_Source) >> AHBPrescTable[RCC->CFGR.b.HPRE];

    /* Configure the source of time base considering new system clocks settings*/
    XPD_InitTimer();

    return result;
}

/**
 * @brief Sets the new configuration for an APB peripheral clock.
 * @param PCLKx: the selected peripheral clock. Permitted values:
             @arg @ref RCC_ClockType::PCLK1
             @arg @ref RCC_ClockType::PCLK2
 * @param HCLK_Divider: Clock divider of @ref RCC_ClockType::PCLK1 clock. Permitted values:
             @arg @ref ClockDividerType::CLK_DIV1
             @arg @ref ClockDividerType::CLK_DIV2
             @arg @ref ClockDividerType::CLK_DIV4
             @arg @ref ClockDividerType::CLK_DIV8
             @arg @ref ClockDividerType::CLK_DIV16
 * @return Result of the operation
 */
void XPD_RCC_PCLKConfig(RCC_ClockType PCLKx, ClockDividerType PCLK_Divider)
{
    uint32_t pprex = rcc_convertClockDivider(PCLKx, PCLK_Divider);

    switch (PCLKx)
    {
        case PCLK1:
            RCC->CFGR.b.PPRE1 = pprex;
            break;

        case PCLK2:
            RCC->CFGR.b.PPRE2 = pprex;
            break;

        default:
            break;
    }
}

/**
 * @brief Gets the selected core clock frequency.
 * @param SelectedClock: the selected core clock
 * @return The frequency of the core clock in Hz.
 */
uint32_t XPD_RCC_GetClockFreq(RCC_ClockType SelectedClock)
{
    switch (SelectedClock)
    {
    case SYSCLK:
        return SystemCoreClock << AHBPrescTable[RCC->CFGR.b.HPRE];

    case HCLK:
        return SystemCoreClock;

    case PCLK1:
        return SystemCoreClock >> APBPrescTable[RCC->CFGR.b.PPRE1];

    case PCLK2:
        return SystemCoreClock >> APBPrescTable[RCC->CFGR.b.PPRE2];

    default:
        return 0;
    }
}

/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_MCO RCC Clock Outputs
 *  @brief    RCC microcontroller clock outputs
 * @{
 */

/**
 * @brief Configures a master clock output
 * @param MCOx: the number of the MCO
 * @param MCOSource: clock source of the MCO
 *        This parameter has different value sets for different MCO selections:
 *        @arg For MCO1 configuration, use @ref RCC_MCO1_ClockSourceType
 *        @arg For MCO2 configuration, use @ref RCC_MCO2_ClockSourceType
 * @param MCODiv: the clock division to be applied for the MCO
 */
void XPD_RCC_MCOConfig(uint8_t MCOx, uint8_t MCOSource, ClockDividerType MCODiv)
{
    GPIO_InitType gpio;

    gpio.Mode = GPIO_MODE_ALTERNATE;
    gpio.AlternateMap = GPIO_MCO_AF0;
    gpio.Output.Speed = VERY_HIGH;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Pull = GPIO_PULL_FLOAT;

    switch (MCOx)
    {
    case 1:
        /* MCO1 map: PA8 */
        XPD_GPIO_InitPin(GPIOA, 8, &gpio);

        RCC->CFGR.b.MCO1 = MCOSource;
        RCC->CFGR.b.MCO1PRE = rcc_convertClockDivider(0xFF, MCODiv);

#ifdef RCC_CFGR_MCO1EN
        RCC_REG_BIT(CFGR,MCO1EN) = 1;
#endif
        break;

    case 2:
        /* MCO2 map: PC9 */
        XPD_GPIO_InitPin(GPIOC, 9, &gpio);

        RCC->CFGR.b.MCO2 = MCOSource;
        RCC->CFGR.b.MCO2PRE = rcc_convertClockDivider(0xFF, MCODiv);

#ifdef RCC_CFGR_MCO2EN
        RCC_REG_BIT(CFGR,MCO2EN) = 1;
#endif
        break;

    default:
        break;
    }
}

#ifdef RCC_CFGR_MCO1EN
void XPD_RCC_EnableMCO(uint8_t MCOx)
{
    if(MCOx == 1)
    {
        RCC_REG_BIT(CFGR,MCO1EN) = 1;
    }
#ifdef RCC_CFGR_MCO2EN
    else
    {
        RCC_REG_BIT(CFGR,MCO2EN) = 1;
    }
#endif
}

void XPD_RCC_DisableMCO(uint8_t MCOx)
{
    if(MCOx == 1)
    {
        RCC_REG_BIT(CFGR,MCO1EN) = 0;
    }
#ifdef RCC_CFGR_MCO2EN
    else
    {
        RCC_REG_BIT(CFGR,MCO2EN) = 0;
    }
#endif
}
#endif /* RCC_CFGR_MCO1EN */

/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_Reset RCC Collective Reset
 *  @brief    RCC collective clocks and peripherals reset functions
 * @{
 */

/**
 * @brief Resets the clock configuration to the default state.
 */
void XPD_RCC_Deinit(void)
{
    /* Set HSION bit, HSITRIM[4:0] bits to the reset value*/
    RCC_REG_BIT(CR, HSION) = 1;
    RCC->CR.b.HSITRIM = 0x10;

    /* Reset CFGR register */
    RCC->CFGR.w = 0;

    /* Reset HSEON, CSSON, PLLON, PLLI2S */
    RCC_REG_BIT(CR, HSEON) = 0;
    RCC_REG_BIT(CR, CSSON) = 0;
    RCC_REG_BIT(CR, PLLON) = 0;
#ifdef RCC_CR_PLLI2SON
    RCC_REG_BIT(CR, PLLI2SON) = 0;
#endif

    /* Reset PLLCFGR register */
    RCC->PLLCFGR.w = 0x24003010;

    /* Reset HSEBYP bit */
    RCC_REG_BIT(CR,HSEBYP) = 0;

    /* Disable all interrupts */
    RCC->CIR.w = 0x00000000;
}

/**
 * @brief Resets the AHB1 peripherals.
 */
void XPD_RCC_ResetAHB1(void)
{
    RCC->AHB1RSTR.w = 0xFFFFFFFF;
    RCC->AHB1RSTR.w = 0x00000000;
}

/**
 * @brief Resets the AHB2 peripherals.
 */
void XPD_RCC_ResetAHB2(void)
{
    RCC->AHB2RSTR.w = 0xFFFFFFFF;
    RCC->AHB2RSTR.w = 0x00000000;
}

/**
 * @brief Resets the AHB3 peripherals.
 */
void XPD_RCC_ResetAHB3(void)
{
    RCC->AHB3RSTR.w = 0xFFFFFFFF;
    RCC->AHB3RSTR.w = 0x00000000;
}

/**
 * @brief Resets the APB1 peripherals.
 */
void XPD_RCC_ResetAPB1(void)
{
    RCC->APB1RSTR.w = 0xFFFFFFFF;
    RCC->APB1RSTR.w = 0x00000000;
}

/**
 * @brief Resets the APB2 peripherals.
 */
void XPD_RCC_ResetAPB2(void)
{
    RCC->APB2RSTR.w = 0xFFFFFFFF;
    RCC->APB2RSTR.w = 0x00000000;
}
/** @} */

/** @} */

/** @} */

/** @} */

XPD_RCC_CallbacksType XPD_RCC_Callbacks = { NULL, NULL };
