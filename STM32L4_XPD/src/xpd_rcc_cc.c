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

#ifdef RCC_PLLP_DIV_2_31_SUPPORT
#define RCC_SET_PLLP_CFG(PLL_NAME,VALUE)     \
    (RCC->PLL_NAME##CFGR.b.PLL_NAME##PDIV = (VALUE))
#else
#define RCC_SET_PLLP_CFG(PLL_NAME,VALUE)     \
    (RCC_REG_BIT(PLL_NAME##CFGR,PLL_NAME##P) = ((VALUE) == 7) ? 0 : 1)
#endif

static const uint8_t rcc_aucMsiTable[] =
    {1, 2, 4, 8, 16, 24, 32, 48};

static const uint8_t rcc_aucAHBPrescTable[] =
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

#define rcc_aucAPBPrescTable (&rcc_aucAHBPrescTable[4])

static RCC_OscType rcc_eReadyOscillator = MSI;

RCC_CallbacksType RCC_xCallbacks = { NULL, NULL, NULL };

/* gets the appropriate FLASH latency for MSI range */
static uint8_t RCC_prvFlashLatencyForMSI(RCC_MSIFreqType eClockRange)
{
    uint8_t ucLatency = 0;

    if (PWR_eGetVoltageScale() == PWR_REGVOLT_SCALE1)
    {
        if (eClockRange > MSI_32MHz)
        {
            ucLatency = 2; /* 2WS */
        }
        else if (eClockRange > MSI_16MHz)
        {
            ucLatency = 1; /* 1WS */
        }
        /* else MSI <= 16Mhz default FLASH_LATENCY_0 0WS */
    }
    else
    {
        if (eClockRange > MSI_16MHz)
        {
            ucLatency = 3; /* 3WS */
        }
        else if (eClockRange > MSI_8MHz)
        {
            /* MSI 16Mhz */
            ucLatency = 2; /* 2WS */
        }
        else if (eClockRange > MSI_4MHz)
        {
            /* MSI 8Mhz */
            ucLatency = 1; /* 1WS */
        }
        /* else MSI < 8Mhz default FLASH_LATENCY_0 0WS */
    }

    return ucLatency;
}

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
    return RCC->PLLCFGR.b.PLLSRC - 1;
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
 * Configures the multiple speed internal oscillator.
 * @param pxConfig: pointer to the configuration parameters
 * @return Result of the operation
 */
XPD_ReturnType RCC_eMSI_Config(const RCC_MSI_InitType * pxConfig)
{
    XPD_ReturnType eResult = XPD_ERROR;
    RCC_OscType eSYSCLK = RCC_prvGetSYSCLKSource();

    /* Check if MSI is used as system clock */
    if (eSYSCLK == MSI)
    {
        /* Just the calibration and MSI range change are allowed */
        if (pxConfig->State != DISABLE)
        {
            uint8_t ucLatency = RCC_prvFlashLatencyForMSI(pxConfig->ClockFreq);

            /* Increasing the MSI range */
            if (ucLatency > FLASH_ucGetLatency())
            {
                /* Program the new number of wait states
                 * to the LATENCY bits in the FLASH_ACR register */
                FLASH_vSetLatency(ucLatency);
            }

            /* Adjust the Clock range and the calibration value.*/
            RCC_REG_BIT(CR, MSIRGSEL) = 1;
            RCC->CR.b.MSIRANGE   = pxConfig->ClockFreq;

            /* Decreasing the MSI range */
            if (ucLatency != FLASH_ucGetLatency())
            {
                /* Program the new number of wait states
                 * to the LATENCY bits in the FLASH_ACR register */
                FLASH_vSetLatency(ucLatency);
            }

            /* Update SystemCoreClock variable */
            SystemCoreClock = RCC_ulOscFreq_Hz(MSI) >>
                    rcc_aucAHBPrescTable[RCC->CFGR.b.HPRE];

            /* Configure the source of time base
             * considering new system clocks settings */
            XPD_vInitTimer(SystemCoreClock);
            eResult = XPD_OK;
        }
    }
    else
    {
        uint32_t ulTimeout = RCC_MSI_TIMEOUT;

        /* Check the MSI State */
        if (pxConfig->State != DISABLE)
        {
            /* Enable the Internal Medium Speed oscillator (MSI). */
            RCC_REG_BIT(CR,MSION) = ENABLE;

            /* Wait until MSI is ready */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_MSIRDY, RCC_CR_MSIRDY, &ulTimeout);

            /* Adjust the Clock range and the calibration value.*/
            RCC_REG_BIT(CR, MSIRGSEL) = 1;
            RCC->CR.b.MSIRANGE   = pxConfig->ClockFreq;
        }
        else
        {
            /* Disable the Internal Medium Speed oscillator (MSI). */
            RCC_REG_BIT(CR,MSION) = DISABLE;

            /* Wait until HSI is disabled */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_MSIRDY, 0, &ulTimeout);
        }
    }

#if defined(RCC_CR_MSIPLLEN) && (LSE_VALUE_Hz == 32768)
    /* When the LSE is ready and at 32.768kHz,
     * enable MSI calibration based on LSE */
    RCC_REG_BIT(CR,MSIPLLEN) = RCC_REG_BIT(BDCR,LSERDY) & RCC_REG_BIT(CR,MSIRDY);
#endif

    return eResult;
}

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
    RCC_REG_BIT(CRRCR,HSI48ON) = ENABLE;

    /* Wait until HSI is ready */
    return XPD_eWaitForMatch(&RCC->CRRCR.w,
            RCC_CRRCR_HSI48RDY, RCC_CRRCR_HSI48RDY, &ulTimeout);
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
        RCC_REG_BIT(CRRCR,HSI48ON) = ENABLE;

        /* Wait until HSI is disabled */
        eResult = XPD_eWaitForMatch(&RCC->CRRCR.w,
                RCC_CRRCR_HSI48RDY, 0, &ulTimeout);
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
    PWR_REG_BIT(CR1,DBP) = 1;

    /* Wait for Backup domain Write protection disable */
    eResult = XPD_eWaitForMatch(&PWR->CR1.w,
            PWR_CR1_DBP, PWR_CR1_DBP, &ulTimeout);

    if (eResult == XPD_OK)
    {
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
    }

#if defined(RCC_CR_MSIPLLEN) && (LSE_VALUE_Hz == 32768)
    /* When the LSE is ready and at 32.768kHz,
     * enable MSI calibration based on LSE */
    RCC_REG_BIT(CR,MSIPLLEN) = RCC_REG_BIT(BDCR,LSERDY) & RCC_REG_BIT(CR,MSIRDY);
#endif

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

        if (pxConfig->State != DISABLE)
        {
            /* Configure the main PLL clock source and multiplication factor */
            RCC->PLLCFGR.b.PLLSRC = pxConfig->Source + 1;

            RCC->PLLCFGR.b.PLLM = pxConfig->M - 1;
            RCC->PLLCFGR.b.PLLN = pxConfig->N;
            RCC_SET_PLLP_CFG(PLL, pxConfig->P);
            RCC->PLLCFGR.b.PLLQ = (pxConfig->Q / 2) - 1;
            RCC->PLLCFGR.b.PLLR = (pxConfig->R / 2) - 1;

            /* Enable the main PLL */
            RCC_REG_BIT(CR,PLLON) = ENABLE;
            RCC_REG_BIT(PLLCFGR,PLLREN) = ENABLE;

            /* Wait until PLL is ready */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_PLLRDY, RCC_CR_PLLRDY, &ulTimeout);
        }
        else
        {
            /* Disable all PLL outputs to save power if no PLLs on */
            if((RCC_REG_BIT(CR,PLLSAI1RDY) == 0)
#ifdef RCC_PLLSAI2_SUPPORT
            && (RCC_REG_BIT(CR,PLLSAI2RDY) == 0)
#endif
              )
            {
                RCC->PLLCFGR.b.PLLSRC = 0;
            }
            CLEAR_BIT(RCC->PLLCFGR.w,
                RCC_PLLCFGR_PLLPEN | RCC_PLLCFGR_PLLQEN | RCC_PLLCFGR_PLLREN);

            /* Wait until PLL is ready */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_PLLRDY, 0, &ulTimeout);
        }
    }
    return eResult;
}

/**
 * @brief Configures the SAI1 phase locked loop.
 * @param pxConfig: pointer to the configuration parameters
 * @return Result of the operation
 */
XPD_ReturnType RCC_ePLLSAI1_Config(const RCC_PLL_InitType * pxConfig)
{
    XPD_ReturnType eResult;

    /* Clock source and divider M is only applied if no other PLLs are configured yet */
    if (RCC_prvGetPLLSource() == NO_OSC)
    {
        /* Configure the main PLL clock source and multiplication factor. */
        RCC->PLLCFGR.b.PLLSRC = pxConfig->Source + 1;

        RCC->PLLCFGR.b.PLLM = pxConfig->M - 1;
    }

    {
        uint32_t ulTimeout = RCC_PLL_TIMEOUT;

        /* Disable the PLLSAI1 */
        RCC_REG_BIT(CR, PLLSAI1ON) = DISABLE;

        /* Wait until PLLSAI1 is disabled */
        eResult = XPD_eWaitForMatch(&RCC->CR.w,
                RCC_CR_PLLSAI1RDY, 0, &ulTimeout);

        if ((eResult == XPD_OK) && (pxConfig->State != DISABLE))
        {
            /* Set SAI1 PLL parameters */
            RCC->PLLSAI1CFGR.b.PLLSAI1N = pxConfig->N;
            RCC_SET_PLLP_CFG(PLLSAI1, pxConfig->P);
            RCC->PLLSAI1CFGR.b.PLLSAI1Q = (pxConfig->Q >> 1) - 1;
            RCC->PLLSAI1CFGR.b.PLLSAI1R = (pxConfig->R >> 1) - 1;

            /* Enable the PLLSAI1 */
            RCC_REG_BIT(CR, PLLSAI1ON) = ENABLE;

            ulTimeout = RCC_PLL_TIMEOUT;
            /* Wait until PLLSAI1 is ready */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_PLLSAI1RDY, RCC_CR_PLLSAI1RDY, &ulTimeout);
        }
    }
    return eResult;
}

#ifdef RCC_PLLSAI2_SUPPORT
/**
 * @brief Configures the SAI2 phase locked loop.
 * @param pxConfig: pointer to the configuration parameters (Q is not used)
 * @return Result of the operation
 */
XPD_ReturnType RCC_ePLLSAI2_Config(const RCC_PLL_InitType * pxConfig)
{
    XPD_ReturnType eResult;

    /* Clock source and divider M is only applied if no other PLLs are configured yet */
    if (RCC_prvGetPLLSource() == NO_OSC)
    {
        /* Configure the main PLL clock source and multiplication factor. */
        RCC->PLLCFGR.b.PLLSRC = pxConfig->Source + 1;

        RCC->PLLCFGR.b.PLLM = pxConfig->M - 1;
    }

    {
        uint32_t ulTimeout = RCC_PLL_TIMEOUT;

        /* Disable the PLLSAI2 */
        RCC_REG_BIT(CR, PLLSAI2ON) = DISABLE;

        /* Wait until PLLSAI2 is disabled */
        eResult = XPD_eWaitForMatch(&RCC->CR.w,
                RCC_CR_PLLSAI2RDY, 0, &ulTimeout);

        if ((eResult == XPD_OK) && (pxConfig->State != DISABLE))
        {
            /* Set SAI2 PLL parameters */
            RCC->PLLSAI2CFGR.b.PLLSAI2N = pxConfig->N;
            RCC_SET_PLLP_CFG(PLLSAI2, pxConfig->P);
            RCC->PLLSAI2CFGR.b.PLLSAI2R = (pxConfig->R >> 1) - 1;

            /* Enable the PLLSAI2 */
            RCC_REG_BIT(CR, PLLSAI2ON) = ENABLE;

            ulTimeout = RCC_PLL_TIMEOUT;
            /* Wait until PLLSAI2 is ready */
            eResult = XPD_eWaitForMatch(&RCC->CR.w,
                    RCC_CR_PLLSAI2RDY, RCC_CR_PLLSAI2RDY, &ulTimeout);
        }
    }
    return eResult;
}
#endif

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

        case MSI:
        {
            uint32_t ulRange = RCC->CR.b.MSIRANGE;

            if (ulRange < 4)
            {
                return (uint32_t)rcc_aucMsiTable[ulRange] * 100000;
            }
            else
            {
                return (uint32_t)rcc_aucMsiTable[ulRange - 4] * 1000000;
            }
        }

        case PLL:
        {
            uint32_t ulM, ulN, ulR;

            ulM = RCC->PLLCFGR.b.PLLM + 1;
            ulN = RCC->PLLCFGR.b.PLLN;
            ulR =(RCC->PLLCFGR.b.PLLR + 1) * 2;

            /* Recursive call - different branch is entered */
            return RCC_ulOscFreq_Hz(RCC_prvGetPLLSource())
                    * ulN / (ulM * ulR);
        }

        case LSI:
            return LSI_VALUE_Hz;

#ifdef LSE_VALUE_Hz
        case LSE:
            return LSE_VALUE_Hz;
#endif

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
    uint32_t ulCIFR = RCC->CIFR.w;
    uint32_t ulCIER = RCC->CIFR.w;

#ifdef LSE_VALUE_Hz
    /* Check RCC LSERDY flag  */
    if (((ulCIFR & RCC_CIFR_LSERDYF) != 0) && ((ulCIER & RCC_CIER_LSERDYIE) != 0))
    {
        /* Clear RCC LSERDY pending bit */
        RCC_FLAG_CLEAR(LSERDY);

        /* LSE Ready callback */
        rcc_eReadyOscillator = LSE;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#endif
    /* Check RCC LSIRDY flag  */
    if (((ulCIFR & RCC_CIFR_LSIRDYF) != 0) && ((ulCIER & RCC_CIER_LSIRDYIE) != 0))
    {
        /* Clear RCC LSIRDY pending bit */
        RCC_FLAG_CLEAR(LSIRDY);

        /* LSI Ready callback */
        rcc_eReadyOscillator = LSI;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
    /* Check RCC PLLRDY flag  */
    if (((ulCIFR & RCC_CIFR_PLLRDYF) != 0) && ((ulCIER & RCC_CIER_PLLRDYIE) != 0))
    {
        /* Clear RCC PLLRDY pending bit */
        RCC_FLAG_CLEAR(PLLRDY);

        /* PLL Ready callback */
        rcc_eReadyOscillator = PLL;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#ifdef HSE_VALUE_Hz
    /* Check RCC HSERDY flag  */
    if (((ulCIFR & RCC_CIFR_HSERDYF) != 0) && ((ulCIER & RCC_CIER_HSERDYIE) != 0))
    {
        /* Clear RCC HSERDY pending bit */
        RCC_FLAG_CLEAR(HSERDY);

        /* HSE Ready callback */
        rcc_eReadyOscillator = HSE;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
#endif
    /* Check RCC HSIRDY flag  */
    if (((ulCIFR & RCC_CIFR_HSIRDYF) != 0) && ((ulCIER & RCC_CIER_HSIRDYIE) != 0))
    {
        /* Clear RCC HSIRDY pending bit */
        RCC_FLAG_CLEAR(HSIRDY);

        /* HSI Ready callback */
        rcc_eReadyOscillator = HSI;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
    /* Check RCC MSIRDY flag  */
    if (((ulCIFR & RCC_CIFR_MSIRDYF) != 0) && ((ulCIER & RCC_CIER_MSIRDYIE) != 0))
    {
        /* Clear RCC MSIRDY pending bit */
        RCC_FLAG_CLEAR(MSIRDY);

        /* HSI Ready callback */
        rcc_eReadyOscillator = MSI;
        XPD_SAFE_CALLBACK(RCC_xCallbacks.OscReady,);
    }
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
        case MSI:
            /* Check the MSI ready flag */
            if (RCC_REG_BIT(CR,MSIRDY) == 0)
            {
                return XPD_ERROR;
            }
            break;

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
        /* Program the new number of wait states
         * to the LATENCY bits in the FLASH_ACR register */
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
        /* Program the new number of wait states
         * to the LATENCY bits in the FLASH_ACR register */
        FLASH_vSetLatency(ucFlashLatency);
    }

    /* Update SystemCoreClock variable */
    SystemCoreClock = RCC_ulOscFreq_Hz(eSYSCLK_Source) >>
            rcc_aucAHBPrescTable[ulClkDiv];

    /* Configure the source of time base
     * considering new system clocks settings*/
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
    RCC->CFGR.b.PPRE1 = RCC_prvConvertPCLKDivider(ePCLK_Divider);
}

/**
 * @brief Sets the new configuration for the APB2 peripheral clock.
 * @param ePCLK_Divider: Clock divider of @ref RCC_ClockType::PCLK2 clock. Permitted values:
             @arg @ref ClockDividerType::CLK_DIV1
             @arg @ref ClockDividerType::CLK_DIV2
             @arg @ref ClockDividerType::CLK_DIV4
             @arg @ref ClockDividerType::CLK_DIV8
             @arg @ref ClockDividerType::CLK_DIV16
 */
void RCC_vPCLK2_Config(ClockDividerType ePCLK_Divider)
{
    RCC->CFGR.b.PPRE2 = RCC_prvConvertPCLKDivider(ePCLK_Divider);
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
            return SystemCoreClock >> rcc_aucAPBPrescTable[RCC->CFGR.b.PPRE1];

        case PCLK2:
            return SystemCoreClock >> rcc_aucAPBPrescTable[RCC->CFGR.b.PPRE2];

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
    /* Set MSION bit */
    RCC_REG_BIT(CR, MSION) = 1;

    /* Ensure MSIRDY bit is set before writing MSIRANGE value */
    while(RCC_REG_BIT(CR, MSIRDY) == 0);

    /* Set MSIRANGE default value */
    RCC->CR.w = RCC_CR_MSION | RCC_CR_MSIRANGE_6;

    /* Reset CFGR register */
    RCC->CFGR.w = 0;

    /* Reset PLLCFGR register */
    RCC->PLLCFGR.w = RCC_PLLCFGR_PLLN_4;

    /* Reset PLLSAI1CFGR register */
    RCC->PLLSAI1CFGR.w = RCC_PLLSAI1CFGR_PLLSAI1N_4;

#if defined(RCC_PLLSAI2_SUPPORT)
    /* Reset PLLSAI2CFGR register */
    RCC->PLLSAI2CFGR.w = RCC_PLLSAI2CFGR_PLLSAI2N_4;
#endif

    /* Disable all interrupts */
    RCC->CIER.w = 0;

    /* Default MSI clock is 4 MHz */
    SystemCoreClock = 4000000;
}

/**
 * @brief Resets the AHB1 peripherals.
 */
void RCC_vResetAHB1(void)
{
    RCC->AHB1RSTR.w = ~0;
    RCC->AHB1RSTR.w = 0;
}

/**
 * @brief Resets the AHB2 peripherals.
 */
void RCC_vResetAHB2(void)
{
    RCC->AHB2RSTR.w = ~0;
    RCC->AHB2RSTR.w = 0;
}

/**
 * @brief Resets the AHB3 peripherals.
 */
void RCC_vResetAHB3(void)
{
    RCC->AHB3RSTR.w = ~0;
    RCC->AHB3RSTR.w = 0;
}

/**
 * @brief Resets the APB1 peripherals.
 */
void RCC_vResetAPB1(void)
{
    RCC->APB1RSTR1.w = ~0;
    RCC->APB1RSTR1.w = 0;
    RCC->APB1RSTR2.w = ~0;
    RCC->APB1RSTR2.w = 0;
}

/**
 * @brief Resets the APB2 peripherals.
 */
void RCC_vResetAPB2(void)
{
    RCC->APB2RSTR.w = ~0;
    RCC->APB2RSTR.w = 0;
}

/** @} */

/** @} */

/** @} */

/** @} */
