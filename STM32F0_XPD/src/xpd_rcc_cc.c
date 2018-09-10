/**
  ******************************************************************************
  * @file    xpd_rcc_cc.c
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers RCC Core Clocks Module
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
#include <xpd_flash.h>
#include <xpd_pwr.h>
#include <xpd_utils.h>

/** @addtogroup RCC
 * @{ */

/** @addtogroup RCC_Core_Clocks RCC Core
 * @{ */

static const uint8_t rcc_aucAHBPrescTable[] =
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

#define rcc_aucAPBPrescTable (&rcc_aucAHBPrescTable[4])

static RCC_OscType rcc_eReadyOscillator = HSI;

RCC_CallbacksType RCC_xCallbacks = { NULL, NULL };

/* converts the general clock divider to HCLK format */
__STATIC_INLINE uint32_t RCC_prvConvertHCLKDivider(ClockDividerType eDivider)
{
    if (eDivider > CLK_DIV1)
    {
        /* actual division values start from 0, with the MSB bit set */
        eDivider--;
        if (eDivider > CLK_DIV16)
        {
            eDivider--; /* skip DIV32 */
        }
        eDivider |= 0x8;
    }
    return (uint32_t)eDivider;
}

/* converts the general clock divider to PCLK format */
__STATIC_INLINE uint32_t RCC_prvConvertPCLKDivider(ClockDividerType eDivider)
{
    if (eDivider > CLK_DIV1)
    {
        /* actual division values start from 0, with the MSB bit set */
        eDivider = 0x4 | (eDivider - 1);
    }
    return (uint32_t)eDivider;
}

/* Gets the input oscillator of the PLL. */
RCC_OscType RCC_prvGetPLLSource(void)
{
#if defined(RCC_CFGR_PLLSRC_HSI_PREDIV)
    RCC_OscType eOsc = RCC->CFGR.b.PLLSRC;
    if ((eOsc != HSI)
#ifdef RCC_HSI48_SUPPORT
     && (eOsc != HSI48)
#endif
    ){ eOsc--; }
    return eOsc;
#else
    return RCC_REG_BIT(CFGR,PLLSRC);
#endif
}

/* Gets the input oscillator of SYSCLK. */
RCC_OscType RCC_prvGetSYSCLKSource(void)
{
    return RCC->CFGR.b.SWS;
}

/** @defgroup RCC_Core_Clocks_Exported_Functions RCC Core Exported Functions
 * @{ */

/** @defgroup RCC_Core_Clocks_Exported_Functions_Oscillators RCC Oscillator Functions
 *  @brief    RCC oscillator and PLL control
 * @{
 */

/**
 * @brief Enables the high speed internal oscillator.
 * @return Result of the operation
 */
XPD_ReturnType RCC_eHSI_Enable(void)
{
    uint32_t ulTimeout = RCC_HSI_TIMEOUT;

    RCC_REG_BIT(CR,HSION) = ENABLE;

    /* Wait until HSI is ready */
    return XPD_eWaitForMatch(&RCC->CR.w,
            RCC_CR_HSIRDY, RCC_CR_HSIRDY, &ulTimeout);
}

/**
 * @brief If not used by the system, disables the high speed internal oscillator.
 * @return Result of the operation
 */
XPD_ReturnType RCC_eHSI_Disable(void)
{
    XPD_ReturnType eResult = XPD_ERROR;
    RCC_OscType eSYSCLK = RCC_prvGetSYSCLKSource();

    /* When the HSI is used as system clock or clock source for PLL
     * it is not allowed to be disabled */
    if (!(     (eSYSCLK == HSI)
           || ((eSYSCLK == PLL) && (RCC_prvGetPLLSource() == HSI))
          ))
    {
        uint32_t ulTimeout = RCC_HSI_TIMEOUT;
        RCC_REG_BIT(CR,HSION) = DISABLE;

        /* Wait until HSI is disabled */
        eResult = XPD_eWaitForMatch(&RCC->CR.w,
                RCC_CR_HSIRDY, 0, &ulTimeout);
    }
    return eResult;
}

/**
 * @brief Enables the low speed internal oscillator.
 * @return Result of the operation
 */
XPD_ReturnType RCC_eLSI_Enable(void)
{
    uint32_t ulTimeout = RCC_LSI_TIMEOUT;

    /* Enable the Internal Low Speed oscillator (LSI). */
    RCC_REG_BIT(CSR,LSION) = ENABLE;

    /* Wait until LSI is ready */
    return XPD_eWaitForMatch(&RCC->CSR.w,
            RCC_CSR_LSIRDY, RCC_CSR_LSIRDY, &ulTimeout);
}

/**
 * @brief Disables the low speed internal oscillator.
 * @return Result of the operation
 */
XPD_ReturnType RCC_eLSI_Disable(void)
{
    uint32_t ulTimeout = RCC_LSI_TIMEOUT;

    /* Disable the Internal Low Speed oscillator (LSI). */
    RCC_REG_BIT(CSR,LSION) = DISABLE;

    /* Wait until LSI is disabled */
    return XPD_eWaitForMatch(&RCC->CSR.w,
            RCC_CSR_LSIRDY, 0, &ulTimeout);
}

#ifdef RCC_HSI48_SUPPORT
/**
 * @brief Enables the 48 MHz high speed internal oscillator.
 * @return Result of the operation
 */
XPD_ReturnType RCC_eHSI48_Enable(void)
{
    uint32_t ulTimeout = RCC_HSI48_TIMEOUT;
    RCC_REG_BIT(CR2,HSI48ON) = ENABLE;

    /* Wait until HSI is ready */
    return XPD_eWaitForMatch(&RCC->CR2.w, RCC_CR2_HSI48RDY, RCC_CR2_HSI48RDY, &ulTimeout);
}

/**
 * @brief If not used by the system, disables the 48 MHz high speed internal oscillator.
 * @return Result of the operation
 */
XPD_ReturnType RCC_eHSI48_Disable(void)
{
    XPD_ReturnType eResult = XPD_ERROR;
    RCC_OscType eSYSCLK = RCC_prvGetSYSCLKSource();

    /* When the HSI48 is used as system clock or clock source for PLL
     * it is not allowed to be disabled */
    if (!(     (eSYSCLK == HSI48)
           || ((eSYSCLK == PLL) && (RCC_prvGetPLLSource() == HSI48))
          ))
    {
        uint32_t ulTimeout = RCC_HSI48_TIMEOUT;
        RCC_REG_BIT(CR2,HSI48ON) = ENABLE;

        /* Wait until HSI is disabled */
        eResult = XPD_eWaitForMatch(&RCC->CR2.w, RCC_CR2_HSI48RDY, 0, &ulTimeout);
    }
    return eResult;
}
#endif

#ifdef HSE_VALUE_Hz
/**
 * @brief Sets the new state of the high speed external oscillator.
 * @param eOscState: the new oscillator configuration
 * @return Result of the operation
 */
XPD_ReturnType RCC_eHSE_Config(RCC_OscStateType eOscState)
{
    XPD_ReturnType eResult = XPD_ERROR;
    RCC_OscType eSYSCLK = RCC_prvGetSYSCLKSource();

    /* When the HSE is used as system clock or clock source for PLL
     * it is not allowed to be disabled */
    if (     (eSYSCLK == HSE)
         || ((eSYSCLK == PLL) && (RCC_prvGetPLLSource() == HSE))
         )
    {
        /* OK if the configuration is already set */
        if ((RCC_REG_BIT(CR,HSEON)  == (eOscState &  1)) &&
            (RCC_REG_BIT(CR,HSEBYP) == (eOscState >> 1)))
        {
            eResult = XPD_OK;
        }
    }
    else
    {
        uint32_t ulTimeout = RCC_HSE_TIMEOUT;

        if (eOscState == OSC_OFF)
        {
            RCC_REG_BIT(CR,HSEON)  = 0;
            RCC_REG_BIT(CR,HSEBYP) = 0;

            /* Wait until HSE is disabled */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_HSERDY, 0, &ulTimeout);
        }
        else
        {
            RCC_REG_BIT(CR,HSEON)  = 1;
            RCC_REG_BIT(CR,HSEBYP) = eOscState >> 1;

            /* Wait until HSE is ready */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_HSERDY, RCC_CR_HSERDY, &ulTimeout);
        }
    }
    return eResult;
}
#endif

#ifdef LSE_VALUE_Hz
/**
 * @brief Sets the new state of the low speed external oscillator.
 * @param eOscState: the new oscillator configuration
 * @return Result of the operation
 */
XPD_ReturnType RCC_eLSE_Config(RCC_OscStateType eOscState)
{
    XPD_ReturnType eResult = XPD_OK;
    uint32_t ulTimeout = RCC_DBP_TIMEOUT;

    /* Enable write access to Backup domain */
    PWR_REG_BIT(CR,DBP) = 1;

    /* Wait for Backup domain Write protection disable */
    eResult = XPD_eWaitForMatch(&PWR->CR.w,
            PWR_CR_DBP, PWR_CR_DBP, &ulTimeout);
    if (eResult != XPD_OK)
    {
        return eResult;
    }

    ulTimeout = RCC_LSE_TIMEOUT;

    if (eOscState == OSC_OFF)
    {
        RCC_REG_BIT(BDCR,LSEON)  = 0;
        RCC_REG_BIT(BDCR,LSEBYP) = 0;

        /* Wait until HSE is disabled */
        eResult = XPD_eWaitForMatch(&RCC->BDCR.w,
                RCC_BDCR_LSERDY, 0, &ulTimeout);
    }
    else
    {
        RCC_REG_BIT(BDCR,LSEON)  = 1;
        RCC_REG_BIT(BDCR,LSEBYP) = eOscState >> 1;

        /* Wait until HSE is ready */
        eResult = XPD_eWaitForMatch(&RCC->BDCR.w,
                RCC_BDCR_LSERDY, RCC_BDCR_LSERDY, &ulTimeout);
    }
    return eResult;
}
#endif

/**
 * @brief Configures the phase locked loop.
 * @param pxConfig: pointer to the configuration parameters
 * @return Result of the operation
 */
XPD_ReturnType RCC_ePLL_Config(const RCC_PLL_InitType * pxConfig)
{
    XPD_ReturnType eResult = XPD_ERROR;
    RCC_OscType eSYSCLK = RCC_prvGetSYSCLKSource();

    /* Check if the PLL is used as system clock or not */
    if (eSYSCLK != PLL)
    {
        uint32_t ulTimeout = RCC_PLL_TIMEOUT;
        /* Disable the main PLL. */
        RCC_REG_BIT(CR,PLLON) = DISABLE;

        /* Wait until PLL is disabled */
        eResult = XPD_eWaitForMatch(&RCC->CR.w,
                RCC_CR_PLLRDY, 0, &ulTimeout);

        if ((eResult == XPD_OK) && (pxConfig->State != DISABLE))
        {
            /* Configure the main PLL clock source and multiplication factor. */
#if defined(RCC_CFGR_PLLSRC_HSI_PREDIV)
            RCC->CFGR2.b.PREDIV = pxConfig->Predivider - 1;

#ifdef RCC_HSI48_SUPPORT
            if (pxConfig->Source == HSI48)
            {
                RCC->CFGR.b.PLLSRC = 3;
            }
            else
#endif
            {
                RCC->CFGR.b.PLLSRC = pxConfig->Source + 1;
            }
#else
            /* HSI can only be predivided by fixed 2, otherwise throw error */
            if ((pxConfig->Source == HSI) && (pxConfig->Predivider != 2))
            {
                return XPD_ERROR;
            }

            RCC_REG_BIT(CFGR,PLLSRC) = pxConfig->Source;
#endif
            RCC->CFGR.b.PLLMUL = pxConfig->Multiplier - 2;

            /* Enable the main PLL. */
            RCC_REG_BIT(CR,PLLON) = ENABLE;

            /* Wait until PLL is ready */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_PLLRDY, RCC_CR_PLLRDY, &ulTimeout);
        }
    }
    return eResult;
}

/**
 * @brief Gets the oscillator frequency.
 * @param eOscillator: the selected oscillator
 * @return The frequency of the oscillator in Hz.
 */
uint32_t RCC_ulOscFreq_Hz(RCC_OscType eOscillator)
{
    switch (eOscillator)
    {
#ifdef HSE_VALUE_Hz
        case HSE:
            return HSE_VALUE_Hz;
#endif

        case HSI:
            return HSI_VALUE_Hz;

#ifdef RCC_HSI48_SUPPORT
        case HSI48:
            return HSI48_VALUE_Hz;
#endif

        case PLL:
        {
            uint32_t ulFreq = HSI_VALUE_Hz, ulPLLSrc = RCC->CFGR.b.PLLSRC;

            switch (RCC->CFGR.b.PLLSRC)
            {
#ifdef RCC_CFGR_PLLSRC_HSI_PREDIV
#ifdef RCC_HSI48_SUPPORT
                case 3:
                    ulFreq = HSI48_VALUE_Hz;
                    break;
#endif /* RCC_HSI48_SUPPORT */
#ifdef HSE_VALUE_Hz
                case 2:
                    ulFreq = HSE_VALUE_Hz;
                    break;
#endif /* HSE_VALUE_Hz */
#else
#ifdef HSE_VALUE_Hz
                case 1:
                    ulFreq = HSE_VALUE_Hz;
                    break;
#endif /* HSE_VALUE_Hz */
#endif
                default:
                    break;
            }

            if (ulPLLSrc > 0)
            {
                return (ulFreq * (RCC->CFGR.b.PLLMUL + 2)) / (RCC->CFGR2.b.PREDIV + 1);
            }
            else
            {
                /* HSI/2 is PLL input */
                return (ulFreq * (RCC->CFGR.b.PLLMUL + 2)) / 2;
            }
        }

        case LSI:
            return LSI_VALUE_Hz;

#ifdef LSE_VALUE_Hz
        case LSE:
            return LSE_VALUE_Hz;
#endif

        case HSI14:
            return 14000000;

        default:
            return 0;
    }
}

/**
 * @brief Gets the oscillator which was detected ready by the interrupt handler.
 * @return The oscillator which is currently ready
 */
RCC_OscType RCC_eGetReadyOsc(void)
{
    return rcc_eReadyOscillator;
}

/**
 * @brief RCC interrupt handler that provides oscillator ready callback.
 */
void RCC_vIRQHandler(void)
{
    uint32_t ulCIR = RCC->CIR.w;

#ifdef LSE_VALUE_Hz
    /* Check RCC LSERDY flag  */
    if ((ulCIR & (RCC_CIR_LSERDYF | RCC_CIR_LSERDYIE)) == (RCC_CIR_LSERDYF | RCC_CIR_LSERDYIE))
    {
        /* Clear RCC LSERDY pending bit */
        RCC_FLAG_CLEAR(LSERDY);

        /* LSE Ready callback */
        rcc_eReadyOscillator = LSE;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#endif
    /* Check RCC LSIRDY flag  */
    if ((ulCIR & (RCC_CIR_LSIRDYF | RCC_CIR_LSIRDYIE)) == (RCC_CIR_LSIRDYF | RCC_CIR_LSIRDYIE))
    {
        /* Clear RCC LSIRDY pending bit */
        RCC_FLAG_CLEAR(LSIRDY);

        /* LSI Ready callback */
        rcc_eReadyOscillator = LSI;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
    /* Check RCC PLLRDY flag  */
    if ((ulCIR & (RCC_CIR_PLLRDYF | RCC_CIR_PLLRDYIE)) == (RCC_CIR_PLLRDYF | RCC_CIR_PLLRDYIE))
    {
        /* Clear RCC PLLRDY pending bit */
        RCC_FLAG_CLEAR(PLLRDY);

        /* PLL Ready callback */
        rcc_eReadyOscillator = PLL;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#ifdef HSE_VALUE_Hz
    /* Check RCC HSERDY flag  */
    if ((ulCIR & (RCC_CIR_HSERDYF | RCC_CIR_HSERDYIE)) == (RCC_CIR_HSERDYF | RCC_CIR_HSERDYIE))
    {
        /* Clear RCC HSERDY pending bit */
        RCC_FLAG_CLEAR(HSERDY);

        /* HSE Ready callback */
        rcc_eReadyOscillator = HSE;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#endif
    /* Check RCC HSIRDY flag  */
    if ((ulCIR & (RCC_CIR_HSIRDYF | RCC_CIR_HSIRDYIE)) == (RCC_CIR_HSIRDYF | RCC_CIR_HSIRDYIE))
    {
        /* Clear RCC HSIRDY pending bit */
        RCC_FLAG_CLEAR(HSIRDY);

        /* HSI Ready callback */
        rcc_eReadyOscillator = HSI;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
    if ((ulCIR & (RCC_CIR_HSI14RDYF | RCC_CIR_HSI14RDYIE)) == (RCC_CIR_HSI14RDYF | RCC_CIR_HSI14RDYIE))
    {
        /* Clear RCC HSIRDY pending bit */
        RCC_FLAG_CLEAR(HSI14RDY);

        /* HSI Ready callback */
        rcc_eReadyOscillator = HSI14;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#ifdef RCC_HSI48_SUPPORT
    if ((ulCIR & (RCC_CIR_HSI48RDYF | RCC_CIR_HSI48RDYIE)) == (RCC_CIR_HSI48RDYF | RCC_CIR_HSI48RDYIE))
    {
        /* Clear RCC HSIRDY pending bit */
        RCC_FLAG_CLEAR(HSI48RDY);

        /* HSI Ready callback */
        rcc_eReadyOscillator = HSI48;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#endif
}

/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_Clocks RCC Core Clocks Functions
 *  @brief    RCC core clocks control
 * @{
 */

/**
 * @brief Sets the new configuration for the AHB system clocks and the new matching flash latency.
 * @param eSYSCLK_Source: @ref RCC_ClockType::SYSCLK input source selection. Permitted values:
             @arg @ref RCC_OscType::HSI
             @arg @ref RCC_OscType::HSE
             @arg @ref RCC_OscType::PLL
 * @param eHCLK_Divider: Clock divider of @ref RCC_ClockType::HCLK clock. Must not be CLK_DIV32.
 * @param ucFlashLatency: the desired amount of flash wait states
 * @return Result of the operation
 * @note To correctly read data from FLASH memory, the number of wait states must be
 * correctly programmed according to the frequency of the CPU clock (HCLK) of the device.
 */
XPD_ReturnType RCC_eHCLK_Config(
        RCC_OscType         eSYSCLK_Source,
        ClockDividerType    eHCLK_Divider,
        uint8_t             ucFlashLatency)
{
    XPD_ReturnType eResult;
    uint32_t ulClkDiv;
    uint32_t ulTimeout = RCC_CLOCKSWITCH_TIMEOUT;

    /* Checking whether the SYSCLK source is ready to be used */
    switch (eSYSCLK_Source)
    {
        case HSI:
            /* Check the HSI ready flag */
            if (RCC_REG_BIT(CR,HSIRDY) == 0)
            {
                return XPD_ERROR;
            }
            break;

#ifdef HSE_VALUE_Hz
        case HSE:
            /* Check the HSE ready flag */
            if (RCC_REG_BIT(CR,HSERDY) == 0)
            {
                return XPD_ERROR;
            }
            break;
#endif

#ifdef RCC_HSI48_SUPPORT
        case HSI48:
            /* Check the HSI ready flag */
            if (RCC_REG_BIT(CR2,HSI48RDY) == 0)
            {
                return XPD_ERROR;
            }
            break;
#endif

        case PLL:
            /* Check the PLL ready flag */
            if (RCC_REG_BIT(CR,PLLRDY) == 0)
            {
                return XPD_ERROR;
            }
            break;

        default:
            return XPD_ERROR;
    }

    /* Increasing the CPU frequency */
    if (ucFlashLatency > FLASH_ucGetLatency())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        FLASH_vSetLatency(ucFlashLatency);
    }

    /* Set SYSCLK source and HCLK prescaler */
    ulClkDiv = RCC_prvConvertHCLKDivider(eHCLK_Divider);
    RCC->CFGR.b.HPRE = ulClkDiv;
    RCC->CFGR.b.SW   = eSYSCLK_Source;

    /* wait until the settings have been processed */
    eResult = XPD_eWaitForMatch(&RCC->CFGR.w,
            RCC_CFGR_SWS, eSYSCLK_Source << RCC_CFGR_SWS_Pos, &ulTimeout);

    /* Decreasing the CPU frequency */
    if (ucFlashLatency != FLASH_ucGetLatency())
    {
        /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
        FLASH_vSetLatency(ucFlashLatency);
    }

    /* Update SystemCoreClock variable */
    SystemCoreClock = RCC_ulOscFreq_Hz(eSYSCLK_Source) >> rcc_aucAHBPrescTable[ulClkDiv];

    /* Configure the source of time base considering new system clocks settings*/
    XPD_vInitTimer(SystemCoreClock);

    return eResult;
}

/**
 * @brief Sets the new configuration for the APB1 peripheral clock.
 * @param ePCLK_Divider: Clock divider of @ref RCC_ClockType::PCLK1 clock. Permitted values:
             @arg @ref ClockDividerType::CLK_DIV1
             @arg @ref ClockDividerType::CLK_DIV2
             @arg @ref ClockDividerType::CLK_DIV4
             @arg @ref ClockDividerType::CLK_DIV8
             @arg @ref ClockDividerType::CLK_DIV16
 */
void RCC_vPCLK1_Config(ClockDividerType ePCLK_Divider)
{
    RCC->CFGR.b.PPRE = RCC_prvConvertPCLKDivider(ePCLK_Divider);
}

/**
 * @brief Gets the selected core clock frequency.
 * @param eSelectedClock: the selected core clock
 * @return The frequency of the core clock in Hz.
 */
uint32_t RCC_ulClockFreq_Hz(RCC_ClockType eSelectedClock)
{
    switch (eSelectedClock)
    {
        case HCLK:
            return SystemCoreClock;

        case SYSCLK:
            return RCC_ulOscFreq_Hz(RCC_prvGetSYSCLKSource());

        case PCLK1:
            return SystemCoreClock >> rcc_aucAPBPrescTable[RCC->CFGR.b.PPRE];

        default:
            return 0;
    }
}

/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_Reset RCC Collective Reset
 *  @brief    RCC collective clocks and peripherals reset functions
 * @{
 */

/**
 * @brief Resets the clock configuration to the default state.
 */
void RCC_vDeinit(void)
{
    /* Set HSION bit, HSITRIM[4:0] bits to the reset value */
    RCC->CR.w = RCC_CR_HSION | RCC_CR_HSITRIM_4;

    /* Reset CFGR registers */
    RCC->CFGR.w = 0;
    RCC->CFGR2.w = 0;
    RCC->CFGR3.w = 0;

    /* Disable all interrupts */
    RCC->CIR.w = 0;

    SystemCoreClock = HSI_VALUE_Hz;
}

/**
 * @brief Resets the AHB peripherals.
 */
void RCC_vResetAHB(void)
{
    RCC->AHBRSTR.w = ~0;
    RCC->AHBRSTR.w = 0;
}

/**
 * @brief Resets the APB peripherals.
 */
void RCC_vResetAPB(void)
{
    RCC->APB1RSTR.w = RCC->APB2RSTR.w = ~0;
    RCC->APB1RSTR.w = RCC->APB2RSTR.w = 0;
}

/** @} */

/** @} */

/** @} */

/** @} */
