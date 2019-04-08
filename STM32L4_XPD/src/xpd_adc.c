/**
  ******************************************************************************
  * @file    xpd_adc.c
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Analog Digital Converter Module
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

#include <xpd_adc.h>
#include <xpd_rcc.h>
#include <xpd_utils.h>

/** @addtogroup ADC
 * @{ */

/** @addtogroup ADC_Core
 * @{ */

#define ADC_STAB_DELAY_US           10
#define ADC_TEMPSENSOR_DELAY_US     120
#define ADC_CALIBRATION_TIMEOUT     10
#define ADC_CONVERSION_TIME_MAX_CPU_CYCLES ((uint32_t) 156928)
#define ADC_STOP_CONVERSION_TIMEOUT 5

#define ADC_STARTCTRL           (ADC_CR_ADSTART | ADC_CR_JADSTART)
#define ADC_STOPCTRL            (ADC_CR_ADSTP | ADC_CR_JADSTP)
#define ADC_ENDFLAG_SEQUENCE    (ADC_IER_EOSIE | ADC_IER_JEOSIE)
#define ADC_ENDFLAG_CONVERSION  (ADC_IER_EOCIE | ADC_IER_JEOCIE)

#define ADC_SQR_MASK            (ADC_SQR1_SQ1 >> ADC_SQR1_SQ1_Pos)
#define ADC_SQR_SIZE            (ADC_SQR1_SQ2_Pos - ADC_SQR1_SQ1_Pos)
#define ADC_SQR_REGDIR          (1)

#if (ADC_COUNT > 1)

#define ADC_PAIR(HANDLE) \
    ((ADC_TypeDef *)(((uint32_t)(HANDLE)->Inst) ^ 0x100))

#define ADC_MULTI_CFGR(HANDLE, DUAL)    \
    (((DUAL) == ADC_MULTIMODE_SINGE) || (IS_ADC_MULTIMODE_MASTER_INSTANCE((HANDLE)->Inst))) ? \
        (HANDLE)->Inst->CFGR.w : ADC_PAIR(HANDLE)->CFGR.w)

#else

#define ADC_PAIR(HANDLE) NULL

#define ADC_MULTI_CFGR(HANDLE, DUAL)    \
    ((HANDLE)->Inst->CFGR.w)

#endif

#if (ADC_COUNT > 1)
static volatile uint8_t adc_aucUsers[] = {
        0
};

/* Enables the peripheral clock in RCC */
static void ADC_prvClockEnable(ADC_HandleType * pxADC)
{
    uint8_t ucAdcx = ADC_INDEX(pxADC);
    uint8_t ucIndex = 0;

    if (adc_aucUsers[ucIndex] == 0)
    {
        RCC_vClockEnable(RCC_POS_ADC + ucIndex);
    }

    SET_BIT(adc_aucUsers[ucIndex], 1 << ucAdcx);
}

/* Disables the peripheral clock in RCC when neither ADCs are used */
static void ADC_prvClockDisable(ADC_HandleType * pxADC)
{
    uint8_t ucAdcx = ADC_INDEX(pxADC);
    uint8_t ucIndex = 0;

    CLEAR_BIT(adc_aucUsers[ucIndex], 1 << ucAdcx);

    if (adc_aucUsers[ucIndex] == 0)
    {
        RCC_vClockDisable(RCC_POS_ADC + ucIndex);
    }
}
#else
#define ADC_prvClockEnable(HANDLE)    (RCC_vClockEnable(RCC_POS_ADC))
#define ADC_prvClockDisable(HANDLE)   (RCC_vClockDisable(RCC_POS_ADC))
#endif

#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
#define ADC_RESET_ERRORS(HANDLE)    ((HANDLE)->Errors = ADC_ERROR_NONE)
#else
#define ADC_RESET_ERRORS(HANDLE)    ((void)(HANDLE))
#endif

/* Returns the ADC pair's relevant CFGR register value */
__STATIC_INLINE uint32_t ADC_prvGetMultiCfgr(ADC_HandleType * pxADC, uint32_t ulDual)
{
    uint32_t ulCFGR;
#if (ADC_COUNT > 1)
    /* Get relevant register CFGR in ADC instance of ADC master or slave
     * in function of multimode state (for devices with multimode available). */
    if (    (ulDual != ADC_MULTIMODE_SINGE)
         && (!IS_ADC_MULTIMODE_MASTER_INSTANCE(pxADC->Inst)))
    {
        ulCFGR = ADC_PAIR(pxADC)->CFGR.w;
    }
    else
#endif
    {
        ulCFGR = pxADC->Inst->CFGR.w;
    }
    return ulCFGR;
}

static void ADC_prvDmaConversionRedirect(void *pxDMA)
{
    ADC_HandleType* pxADC = (ADC_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    XPD_SAFE_CALLBACK(pxADC->Callbacks.ConvComplete, pxADC);
}

#ifdef __XPD_DMA_ERROR_DETECT
static void ADC_prvDmaErrorRedirect(void *pxDMA)
{
    ADC_HandleType* pxADC = (ADC_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    /* Update error code */
    pxADC->Errors |= ADC_ERROR_DMA;

    XPD_SAFE_CALLBACK(pxADC->Callbacks.Error, pxADC);
}
#endif

/* Enables the peripheral */
static bool ADC_prvEnableInst(ADC_TypeDef * ADCx)
{
    /* Requirements:
     * ADCAL, ADSTP, JADSTP, ADSTART, JADSTART, ADDIS, ADEN = 0 */
    bool eSuccess = (0 == (ADCx->CR.w & (ADC_CR_ADEN | ADC_STARTCTRL
            | ADC_STOPCTRL | ADC_CR_ADCAL | ADC_CR_ADDIS)));
    if (eSuccess)
    {
        SET_BIT(ADCx->CR.w, ADC_CR_ADEN);
    }
    return eSuccess;
}

/* Disables the peripheral */
static void ADC_prvDisableInst(ADC_TypeDef * ADCx)
{
    /* Requirements:
     * ADEN = 1, ADSTART, JADSTART = 0 */
    if (ADC_CR_ADEN == (ADCx->CR.w & (ADC_CR_ADEN | ADC_STARTCTRL)))
    {
        SET_BIT(ADCx->CR.w, ADC_CR_ADDIS);
        SET_BIT(ADCx->ISR.w, ADC_ISR_EOSMP | ADC_ISR_ADRDY);
    }
}

/* Sets the sample time for a channel configuration */
static void ADC_prvSampleTimeConfig(ADC_HandleType * pxADC, const ADC_ChannelInitType * pxChannel)
{
    __IO uint32_t *pulSMPR = &pxADC->Inst->SMPR1.w;
    uint32_t ulNumber = pxChannel->Number;

    /* Channel sampling time configuration */
    if (ulNumber > 10)
    {
        pulSMPR = &pxADC->Inst->SMPR2.w;
        ulNumber -= 10;
    }
    ulNumber *= 3;

    /* set the sample time */
    MODIFY_REG(*pulSMPR, ADC_SMPR1_SMP0 << ulNumber, pxChannel->SampleTime << ulNumber);

#ifdef ADC_SMPR1_SMPPLUS
    /* If sample time is either 2.5 or 3.5, update ADC-wide setting */
    if ((pxChannel->SampleTime & 7) == 0)
    {
        ADC_REG_BIT(pxADC, SMPR1, SMPPLUS) = pxChannel->SampleTime >> 3;
    }
#endif
}

/* Enables an internal channel for conversion */
static void ADC_prvInitInternalChannel(ADC_HandleType * pxADC, uint8_t ucChannel)
{
    uint32_t ulChBit = 0;

    /* Map channel to enable bit */
    switch (ucChannel)
    {
        case ADC1_VBAT_CHANNEL:
            /* enable the VBAT input */
#if (ADC_COUNT > 1)
            if (pxADC->Inst != ADC2)
#endif
            { ulChBit = ADC_CCR_VBATEN; }
            break;

        case ADC1_VREFINT_CHANNEL:
#if (ADC_COUNT > 1)
            if (pxADC->Inst == ADC1)
#endif
            /* enable the VREF input */
            { ulChBit = ADC_CCR_VREFEN; }
            break;

        case ADC1_TEMPSENSOR_CHANNEL:
            /* enable the TS input */
#if (ADC_COUNT > 1)
            if (pxADC->Inst != ADC2)
#endif
            { ulChBit = ADC_CCR_TSEN; }
            break;

        default:
            break;
    }

    /* Check if setting is valid */
    if (((ulChBit != 0) && ((ADC_COMMON(pxADC)->CCR.w & ulChBit) == 0))
        /* Software is allowed to change common parameters only when all ADCs
         * of the common group are disabled */
         && (ADC_REG_BIT(pxADC, CR, ADEN) == 0)
#if (ADC_COUNT > 1)
         && (ADC_PAIR(pxADC)->CR.b.ADEN == 0)
#endif
             )
    {
        SET_BIT(ADC_COMMON(pxADC)->CCR.w, ulChBit);

        if (ucChannel == ADC1_TEMPSENSOR_CHANNEL)
        {
            /* wait temperature sensor stabilization */
            XPD_vDelay_us(ADC_TEMPSENSOR_DELAY_US);
        }
    }
}

/* Stops the further conversions of the selected types */
static void ADC_prvStopConversion(ADC_HandleType * pxADC, uint32_t ulConvFlag)
{
    uint32_t ulTimeout = ADC_STOP_CONVERSION_TIMEOUT;

    /* Verification if ADC is not already stopped */
    if ((pxADC->Inst->CR.w & ADC_STARTCTRL) != 0)
    {
        /* In auto-injection mode, regular group stop ADC_CR_ADSTP is used (not 
         * injected group stop ADC_CR_JADSTP) */
        if ((pxADC->Inst->CFGR.w & (ADC_CFGR_JAUTO | ADC_CFGR_CONT | ADC_CFGR_AUTDLY))
                               == (ADC_CFGR_JAUTO | ADC_CFGR_CONT | ADC_CFGR_AUTDLY))
        {
            uint32_t ulCntr;

            /* Use stop of regular group */
            ulConvFlag = ADC_CR_ADSTART;

            /* Wait until JEOS=1 (maximum Timeout: 4 injected conversions) */
            for (ulCntr = 0; ulCntr < (ADC_CONVERSION_TIME_MAX_CPU_CYCLES * 4); ulCntr++)
            {
                if (ADC_FLAG_STATUS(pxADC, JEOS) == 0)
                {
                    break;
                }
            }

            /* Clear JEOS */
            ADC_FLAG_CLEAR(pxADC, JEOS);
        }

        /* Stop potential conversion on going on regular group */
        if (ulConvFlag != ADC_CR_JADSTART)
        {
            /* Software is allowed to set ADSTP only when ADSTART=1 and ADDIS=0 */
            if ((pxADC->Inst->CR.w & (ADC_CR_ADSTART | ADC_CR_ADDIS)) == ADC_CR_ADSTART)
            {
                /* Stop conversions on regular group */
                ADC_REG_BIT(pxADC, CR, ADSTP) = 1;
            }
        }

        /* Stop potential conversion on going on injected group */
        if (ulConvFlag != ADC_CR_ADSTART)
        {
            /* Software is allowed to set JADSTP only when JADSTART=1 and ADDIS=0 */
            if ((pxADC->Inst->CR.w & (ADC_CR_JADSTART | ADC_CR_ADDIS)) == ADC_CR_JADSTART)
            {
                /* Stop conversions on injected group */
                ADC_REG_BIT(pxADC, CR, JADSTP) = 1;
            }
        }

        /* Wait for conversion effectively stopped */
        XPD_eWaitForMatch(&pxADC->Inst->CR.w, ulConvFlag, 0, &ulTimeout);
    }
}

/* Attempts to set the offset configuration of the input channel list */
static void ADC_prvOffsetConfig(ADC_HandleType * pxADC,
        const ADC_ChannelInitType   axChannels[],
        uint8_t                     ucChannelCount)
{
    union {
        struct {
            __IO uint32_t OFFSET : 12;
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET_CH : 5;
            __IO uint32_t OFFSET_EN : 1;
        } b;
        __IO uint32_t w;
    } *pxOFR;
    uint32_t ulUsedChMask = 0, ulOffsetChMask = 0;
    uint8_t i;

    /* Collect offset related information from list */
    for (i = 0; i < ucChannelCount; i++)
    {
        SET_BIT(ulUsedChMask, axChannels[i].Number);
        if (axChannels[i].Offset > 0)
        {
            SET_BIT(ulOffsetChMask, axChannels[i].Number);
        }
    }

    /* If there are channels that don't need offset, but are previously configured */
    ulUsedChMask = pxADC->OffsetUsage & ulUsedChMask & (~ulOffsetChMask);
    if (ulUsedChMask != 0)
    {
        /* Find the channels in the offsets, clear config */
        for (pxOFR = (void*)&pxADC->Inst->OFR1.w;
             (void*)pxOFR <= (void*)&pxADC->Inst->OFR4.w; pxOFR++)
        {
            uint32_t ulChFlag = 1 << pxOFR->b.OFFSET_CH;
            if ((ulUsedChMask & ulChFlag) != 0)
            {
                pxOFR->w = 0;
                CLEAR_BIT(pxADC->OffsetUsage, ulChFlag);
            }
        }
    }
    /* Iterate until all desired offsets are set */
    for (i = 0; i < ucChannelCount; i++)
    {
        /* If the current channel needs an offset */
        if (axChannels[i].Offset > 0)
        {
            /* For the channels that are missing offset */
            for (pxOFR = (void*)&pxADC->Inst->OFR1.w;
                 (void*)pxOFR <= (void*)&pxADC->Inst->OFR4.w; pxOFR++)
            {
                /* If the offset register is disabled, or already used by this channel */
                if ((pxOFR->w == 0) || ((ulOffsetChMask & (1 << pxOFR->b.OFFSET_CH)) != 0))
                {
                    pxOFR->b.OFFSET    = axChannels[i].Offset;
                    pxOFR->b.OFFSET_CH = axChannels[i].Number;
                    pxOFR->b.OFFSET_EN = 1;
                    SET_BIT(pxADC->OffsetUsage, axChannels[i].Number);
                }
            }
        }
        /* All offsets are configured */
        if (pxADC->OffsetUsage == ulOffsetChMask) { break; }
    }
}

/* Common (regular-injected) channel configuration includes:
 *  - Sample Time
 *  - Watchdogs channel selection
 *  - Channel offsets
 *  - Differential mode
 *  - Internal channels activation */
static uint8_t ADC_prvCommonChannelConfig(
        ADC_HandleType *            pxADC,
        const ADC_ChannelInitType   axChannels[],
        uint8_t                     ucChannelCount)
{
    uint8_t ucAWD1Chs = 0;

    /* No ongoing conversions bound settings */
    if ((pxADC->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        uint8_t i;
        uint32_t aulWatchdogMasks[4] = {0, 0, 0, 0};

        /* Channel sampling time configuration */
        for (i = 0; i < ucChannelCount; i++)
        {
            /* Sample time configuration */
            ADC_prvSampleTimeConfig(pxADC, &axChannels[i]);

            /* Temporarily holds the current channel flag */
            aulWatchdogMasks[1] = 1 << axChannels[i].Number;

            /* Holds all channels' flags */
            aulWatchdogMasks[0] |= aulWatchdogMasks[1];

            /* If a watchdog is used for the channel, set it in the configuration */
            if (axChannels[i].Watchdog == ADC_AWD1)
            {
                pxADC->Inst->CFGR.b.AWD1CH = axChannels[i].Number;
                ucAWD1Chs++;
            }
            /* Store channel flag for channel-wise watchdog configuration */
            else if (axChannels[i].Watchdog != ADC_AWD_NONE)
            {
                aulWatchdogMasks[axChannels[i].Watchdog] |= aulWatchdogMasks[1];
            }
        }

        /* Clear unmonitored channel flags, set monitored ones */
        MODIFY_REG(pxADC->Inst->AWD2CR.w, aulWatchdogMasks[0], aulWatchdogMasks[2]);
        MODIFY_REG(pxADC->Inst->AWD3CR.w, aulWatchdogMasks[0], aulWatchdogMasks[3]);

        /* Set offsets configuration */
        ADC_prvOffsetConfig(pxADC, axChannels, ucChannelCount);

        /* ADC disabled */
        if (ADC_REG_BIT(pxADC, CR, ADEN) == 0)
        {
            /* Channel sampling time configuration */
            for (i = 0; i < ucChannelCount; i++)
            {
                /* Internal channel configuration (if applicable) */
                ADC_prvInitInternalChannel(pxADC, axChannels[i].Number);

                /* Configuration of differential mode */
                if (axChannels[i].Differential == ENABLE)
                {
                    ADC_ChannelInitType xDiffPair = axChannels[i];

                    /* Enable differential mode */
                    SET_BIT(pxADC->Inst->DIFSEL.w, 1 << axChannels[i].Number);

                    /* Pair channel sampling time configuration */
                    xDiffPair.Number++;
                    ADC_prvSampleTimeConfig(pxADC, &xDiffPair);
                }
                else
                {
                    /* Disable differential mode (default mode: single-ended) */
                    CLEAR_BIT(pxADC->Inst->DIFSEL.w, 1 << axChannels[i].Number);
                }
            }
        }
    }
    return ucAWD1Chs;
}

/** @addtogroup ADC_Core_Exported_Functions
 * @{ */

/**
 * @brief Initializes the ADC peripheral using the setup configuration.
 * @param pxADC: pointer to the ADC handle structure
 * @param pxConfig: pointer to ADC setup configuration
 */
void ADC_vInit(ADC_HandleType * pxADC, const ADC_InitType * pxConfig)
{
    /* enable clock */
    ADC_prvClockEnable(pxADC);

    /* Initialize ADC API internal variables */
    pxADC->InjectedConfig = 0;
    pxADC->ConversionCount = 0;
    pxADC->OffsetUsage = 0;

    /* Set used EndFlag */
    if (pxConfig->EndFlagSelection == ADC_EOC_SEQUENCE)
    {
        pxADC->EndFlagSelection = ADC_ENDFLAG_SEQUENCE;
    }
    else
    {
        pxADC->EndFlagSelection = ADC_ENDFLAG_CONVERSION;
    }

    /* Exit deep power down mode */
    ADC_REG_BIT(pxADC,CR,DEEPPWD) = 0;

    /* Enable voltage regulator (if disabled at this step) */
    if (pxADC->Inst->CR.b.ADVREGEN != 1)
    {
        /* ADC voltage regulator enable sequence */
#ifdef ADC_CR_ADVREGEN_1
        pxADC->Inst->CR.b.ADVREGEN = 0;
#endif
        pxADC->Inst->CR.b.ADVREGEN = 1;

        /* Delay for ADC stabilization time */
        XPD_vDelay_us(ADC_STAB_DELAY_US);
    }

    /* ADC regular conversion stopped */
    if (ADC_REG_BIT(pxADC, CR, ADSTART) == 0)
    {
        uint32_t ulCfgr, ulCfgrMask;

        ulCfgrMask = ADC_CFGR_CONT | ADC_CFGR_ALIGN | ADC_CFGR_RES |
                ADC_CFGR_EXTEN | ADC_CFGR_EXTSEL | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM;

        ulCfgr = pxConfig->w & ~(ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM);

        /* Enable discontinuous mode only if continuous mode is disabled */
        if ((pxConfig->DiscontinuousCount != 0) && (pxConfig->ContinuousMode == DISABLE))
        {
            ulCfgr |= ADC_CFGR_DISCEN | ((pxConfig->DiscontinuousCount - 1) << ADC_CFGR_DISCNUM_Pos);
        }

        /* No injected or regular conversion ongoing */
        if ((pxADC->Inst->CR.w & ADC_STARTCTRL) == 0)
        {
            ulCfgrMask |= ADC_CFGR_AUTDLY | ADC_CFGR_DMACFG;
#ifdef ADC_CFGR_DFSDMCFG
            ulCfgrMask |= ADC_CFGR_DFSDMCFG;
#endif

            /* Set oversampling mode configuration */
            if (pxConfig->Oversampling.State == ENABLE)
            {
                MODIFY_REG(pxADC->Inst->CFGR2.w,
                        ADC_CFGR2_ROVSE | ADC_CFGR2_OVSR | ADC_CFGR2_OVSS |
                        ADC_CFGR2_TROVS | ADC_CFGR2_ROVSM,
                        pxConfig->Oversampling.w);
            }
        }

        MODIFY_REG(pxADC->Inst->CFGR.w, ulCfgrMask, ulCfgr);

        /* Set conversion count as flag when scan mode is selected */
        pxADC->ConversionCount = pxConfig->ScanMode;
    }

    /* dependencies initialization */
    XPD_SAFE_CALLBACK(pxADC->Callbacks.DepInit, pxADC);
}

/**
 * @brief Deinitializes the ADC peripheral.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vDeinit(ADC_HandleType * pxADC)
{
    /* Stop potential conversion on going, on regular and injected groups */
    ADC_prvStopConversion(pxADC, ADC_STARTCTRL);

    /* Enable injected queue sequencer after injected conversion stop */
    ADC_REG_BIT(pxADC, CFGR, JQM) = 1;

    /* Disable the ADC peripheral */
    ADC_prvDisableInst(pxADC->Inst);

    /* Reset register IER */
    pxADC->Inst->IER.w = 0;

    /* Reset register ISR */
    pxADC->Inst->ISR.w = ~0;

    /* Reset OFR registers */
    pxADC->Inst->OFR1.w = 0;
    pxADC->Inst->OFR2.w = 0;
    pxADC->Inst->OFR3.w = 0;
    pxADC->Inst->OFR4.w = 0;

    /* Disable voltage regulator */
    CLEAR_BIT(pxADC->Inst->CR.w, ADC_CR_ADVREGEN | ADC_CR_ADCALDIF);
#ifdef ADC_CR_ADVREGEN_1
    SET_BIT(pxADC->Inst->CR.w, ADC_CR_ADVREGEN_1);
#endif

    /* Reset register CFGR */
    pxADC->Inst->CFGR.w = 0;
    pxADC->Inst->CFGR2.w = 0;

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxADC->Callbacks.DepDeinit, pxADC);

    /* disable clock */
    ADC_prvClockDisable(pxADC);
}

/**
 * @brief Initializes the regular ADC channels for conversion using the setup configuration.
 * @param pxADC: pointer to the ADC handle structure
 * @param axChannels: ADC regular channel configuration array pointer
 * @param ucChannelCount: number of channels to configure
 */
void ADC_vChannelConfig(
        ADC_HandleType *            pxADC,
        const ADC_ChannelInitType   axChannels[],
        uint8_t                     ucChannelCount)
{
    /* No regular conversion ongoing */
    if (ADC_REG_BIT(pxADC, CR, ADSTART) == 0)
    {
        uint8_t i, ucAWD1Chs = 0;
        uint32_t ulSeqOffset = ADC_SQR1_SQ1_Pos;
        __IO uint32_t *pulSQR = &pxADC->Inst->SQR1.w;

        /* Sequencer configuration */
        for (i = 0; i < ucChannelCount; i++)
        {
            MODIFY_REG(*pulSQR, ADC_SQR_MASK << ulSeqOffset,
                    axChannels[i].Number << ulSeqOffset);

            /* Advance to next sequence element */
            ulSeqOffset += ADC_SQR_SIZE;

            /* Jump to next register when the current one is filled */
            if (ulSeqOffset > (32 - ADC_SQR_SIZE))
            {
                pulSQR += ADC_SQR_REGDIR;
                ulSeqOffset = 0;
            }
        }

        /* Forward to channel common configurator */
        ucAWD1Chs = ADC_prvCommonChannelConfig(pxADC, axChannels, ucChannelCount);

        /* Configuration of AWD1 */
        if ((pxADC->Inst->CR.w & ADC_STARTCTRL) == 0)
        {
            uint32_t ulClrMask = ADC_CFGR_AWD1SGL;
            uint32_t ulSetMask = 0;

            if (ucAWD1Chs == 0)
            {
                ulClrMask = ADC_CFGR_AWD1EN | ADC_CFGR_AWD1SGL;
            }
            else
            {
                ulSetMask = ADC_CFGR_AWD1EN;

                if (ucAWD1Chs == 1)
                {
                    /* In case of a single channel, set SGL flag
                     * AWD1CH is set in common configurator */
                    ulSetMask = ADC_CFGR_AWD1EN | ADC_CFGR_AWD1SGL;
                }

                /* Disable other group's watchdog if it targeted a single channel */
                if ((pxADC->Inst->CFGR.w & (ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN)) ==
                                           (ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN))
                {
                    ulClrMask = ADC_CFGR_JAWD1EN;
                }
            }
            pxADC->Inst->CFGR.w = (pxADC->Inst->CFGR.w & (~ulClrMask)) | ulSetMask;
        }

        /* Init() sets this variable as flag to indicate ScanMode */
        if (pxADC->ConversionCount > 0)
        {
            pxADC->ConversionCount = ucChannelCount;

            /* Set number of ranks in regular group sequencer */
            pxADC->Inst->SQR1.b.L = pxADC->ConversionCount - 1;
        }
        else
        {
            pxADC->Inst->SQR1.b.L = 0;
        }
    }
}

/**
 * @brief Enables the ADC peripheral and starts conversion if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStart(ADC_HandleType * pxADC)
{
    /* Success if not started */
    if (ADC_REG_BIT(pxADC, CR, ADSTART) == 0)
    {
        /* ADC turn ON */
        if (ADC_prvEnableInst(pxADC->Inst))
        {
            /* Wait until ADRDY flag is set ( < 1us) */
            uint32_t ulTimeout = 1;
            XPD_eWaitForMatch(&pxADC->Inst->ISR.w, ADC_ISR_ADRDY, ADC_ISR_ADRDY, &ulTimeout);
        }

        /* clear regular group conversion flag and overrun flag */
        SET_BIT(pxADC->Inst->ISR.w, ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);

#if (ADC_COUNT > 1)
        /* if ADC is either not in multimode, or the multimode master,
         * enable software conversion of regular channels */
        if (    (ADC_COMMON(pxADC)->CCR.b.DUAL == ADC_MULTIMODE_SINGE)
             || (IS_ADC_MULTIMODE_MASTER_INSTANCE(pxADC->Inst)))
#endif
        {
            ADC_REG_BIT(pxADC, CR, ADSTART) = 1;
        }

        ADC_RESET_ERRORS(pxADC);
    }
}

/**
 * @brief Stops the ADC peripheral.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStop(ADC_HandleType * pxADC)
{
    uint32_t ulTimeout = 1;

    /* 1. Stop potential ongoing conversions */
    ADC_prvStopConversion(pxADC, ADC_STARTCTRL);

    ADC_prvDisableInst(pxADC->Inst);

    XPD_eWaitForMatch(&pxADC->Inst->ISR.w, ADC_ISR_ADRDY, 0, &ulTimeout);
}

/**
 * @brief Polls the status of the ADC operation(s).
 * @param pxADC: pointer to the ADC handle structure
 * @param eOperation: the type of operation to check
 * @param ulTimeout: the timeout in ms for the polling.
 * @return ERROR if there was an input error, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType ADC_ePollStatus(
        ADC_HandleType *    pxADC,
        ADC_OperationType   eOperation,
        uint32_t            ulTimeout)
{
    XPD_ReturnType eResult = XPD_OK;

    if (    (eOperation == ADC_OPERATION_CONVERSION)
         && ((pxADC->EndFlagSelection & ADC_IER_EOSIE) != 0))
    {
        eOperation = ADC_ISR_EOS;
    }
    else if (    (eOperation == ADC_OPERATION_INJCONVERSION)
              && ((pxADC->EndFlagSelection & ADC_IER_JEOSIE) != 0))
    {
        eOperation = ADC_ISR_JEOS;
    }

    /* Polling for single conversion is not allowed when DMA is used,
     * and EOC is raised on end of single conversion */
    if (eOperation == ADC_OPERATION_CONVERSION)
    {
#if (ADC_COUNT > 1)
        /* If multimode used, check common DMA config, otherwise dedicated DMA config */
        if (ADC_COMMON_REG_BIT(pxADC, CCR, DUAL) != 0)
        {
            if (ADC_COMMON_REG_BIT(pxADC, CCR, MDMA) == 1)
            { eResult = XPD_ERROR; }
        }
        else
#endif
        {
            if (ADC_REG_BIT(pxADC, CFGR, DMAEN) == 1)
            { eResult = XPD_ERROR; }
        }
    }

    if (eResult == XPD_OK)
    {
        /* Wait until operation flag is set */
        eResult = XPD_eWaitForMatch(&pxADC->Inst->ISR.w, eOperation, eOperation, &ulTimeout);

        /* Clear end of conversion flag of regular group if low power feature
         * "LowPowerAutoWait " is disabled, to not interfere with this feature
         * until data register is read. */
        if ((eResult == XPD_OK) && ((eOperation & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
             && (ADC_REG_BIT(pxADC, CFGR, AUTDLY) == 0))
        {
            SET_BIT(pxADC->Inst->ISR.w, eOperation);
        }
    }
    return eResult;
}

/**
 * @brief Enables the ADC peripheral and interrupts, and starts conversion
 *        if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStart_IT(ADC_HandleType * pxADC)
{
    /* Choose interrupt source based on EOC config */
    uint32_t ulITs = pxADC->EndFlagSelection;

#ifdef __XPD_ADC_ERROR_DETECT
    /* ADC overrun and end of conversion interrupt for regular group */
    ulITs |= ADC_IER_OVRIE;
#endif

    MODIFY_REG(pxADC->Inst->IER.w, (ADC_IER_EOCIE | ADC_IER_EOSIE | ADC_IER_OVRIE), ulITs);

    ADC_vStart(pxADC);
}

/**
 * @brief Stops the ADC peripheral and disables interrupts.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStop_IT(ADC_HandleType * pxADC)
{
    ADC_vStop(pxADC);

    /* ADC end of conversion interrupt for regular and injected group */
    CLEAR_BIT(pxADC->Inst->IER.w, pxADC->EndFlagSelection | ADC_IER_OVRIE);
}

/**
 * @brief ADC interrupt handler that provides handle callbacks.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vIRQHandler(ADC_HandleType * pxADC)
{
    uint32_t ulISR = pxADC->Inst->ISR.w;
    uint32_t ulIER = pxADC->Inst->IER.w;
#if (ADC_COUNT > 1)
    uint32_t ulDual = ADC_COMMON(pxADC)->CCR.b.DUAL;
#else
    uint32_t ulDual = 0;
#endif

    /* End of conversion flag for regular channels */
    if (    ((ulISR & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
         && ((ulIER & (ADC_IER_EOCIE | ADC_IER_EOSIE)) != 0))
    {
        uint32_t ulCFGR = ADC_prvGetMultiCfgr(pxADC, ulDual);

        /* If the conversion is not continuous / external triggered */
        if ((ulCFGR & (ADC_CFGR_CONT | ADC_CFGR_EXTEN)) == 0)
        {
            /* End of sequence */
            if (((ulISR & ADC_ISR_EOS) != 0) && (ADC_REG_BIT(pxADC, CR, ADSTART) == 0))
            {
                /* Disable the ADC end of conversion interrupt for regular group */
                CLEAR_BIT(pxADC->Inst->IER.w, ADC_IER_EOCIE | ADC_IER_EOSIE);
            }
        }

        /* Clear the ADC flag for regular end of conversion */
        SET_BIT(pxADC->Inst->ISR.w, ADC_ISR_EOC | ADC_ISR_EOS);

        /* Conversion complete callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.ConvComplete, pxADC);
    }

    /* End of conversion flag for injected channels */
    if (    ((ulISR & (ADC_ISR_JEOC | ADC_ISR_JEOS)) != 0)
         && ((ulIER & (ADC_IER_JEOCIE | ADC_IER_JEOSIE)) != 0))
    {
        uint32_t ulCFGR = ADC_prvGetMultiCfgr(pxADC, ulDual);

        /* Injected conversion is not continuous or not automatic,
         * and software triggered */
        if (    (pxADC->Inst->JSQR.b.JEXTEN == 0)
             || (    ((ulCFGR & ADC_CFGR_CONT) == 0)
                  && (pxADC->Inst->CFGR.b.EXTEN == 0)
                  && (pxADC->Inst->CFGR.b.JAUTO == 0)))
        {
            /* End of sequence */
            if ((ulISR & ADC_ISR_JEOS) != 0)
            {
#if (ADC_COUNT > 1)
                if (!(    (ulDual == ADC_MULTIMODE_SINGE)
                       || (ulDual == ADC_MULTIMODE_DUAL_REGSIMULT)
                       || (ulDual == ADC_MULTIMODE_DUAL_INTERLEAVED)
                       || (IS_ADC_MULTIMODE_MASTER_INSTANCE(pxADC->Inst))))
                {
                    ulCFGR = ADC_PAIR(pxADC)->CFGR.w;
                }
                else
#endif
                {
                    ulCFGR = pxADC->Inst->CFGR.w;
                }

                if ((ulCFGR & ADC_CFGR_JQM) == 0)
                {
                    if (ADC_REG_BIT(pxADC, CR, JADSTART) == 0)
                    {
                        /* disable the ADC end of conversion interrupt
                         * for injected group */
                        CLEAR_BIT(pxADC->Inst->IER.w, ADC_IER_JEOC | ADC_IER_JEOS);
                    }
                }
            }
        }
        /* clear the ADC flag for injected end of conversion */
        SET_BIT(pxADC->Inst->ISR.w, ADC_ISR_JEOC | ADC_ISR_JEOS);

        /* injected conversion complete callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.InjConvComplete, pxADC);
    }

    /* Check analog watchdog flag */
    if (    ((ulISR & ADC_ISR_AWD1) != 0)
         && ((ulIER & ADC_IER_AWD1IE) != 0))
    {
        pxADC->ActiveWatchdog = ADC_AWD1;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.Watchdog, pxADC);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        ADC_FLAG_CLEAR(pxADC, AWD1);
    }
    if (    ((ulISR & ADC_ISR_AWD2) != 0)
         && ((ulIER & ADC_IER_AWD2IE) != 0))
    {
        pxADC->ActiveWatchdog = ADC_AWD2;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.Watchdog, pxADC);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        ADC_FLAG_CLEAR(pxADC, AWD2);
    }
    if (    ((ulISR & ADC_ISR_AWD3) != 0)
         && ((ulIER & ADC_IER_AWD3IE) != 0))
    {
        pxADC->ActiveWatchdog = ADC_AWD3;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.Watchdog, pxADC);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        ADC_FLAG_CLEAR(pxADC, AWD3);
    }

#ifdef __XPD_ADC_ERROR_DETECT
    /* Check Overrun flag */
    if (    ((ulISR & ADC_ISR_OVR) != 0)
         && ((ulIER & ADC_IER_OVRIE) != 0))
    {
        /* Update error code */
        pxADC->Errors |= ADC_ERROR_OVERRUN;

        /* Clear the Overrun flag */
        ADC_FLAG_CLEAR(pxADC, OVR);
    }
    /* Check Injected Overrun flag */
    if (    ((ulISR & ADC_ISR_JQOVF) != 0)
         && ((ulIER & ADC_IER_JQOVFIE) != 0))
    {
        /* Update error code */
        pxADC->Errors |= ADC_ERROR_JQOVF;

        /* Clear the Overrun flag */
        ADC_FLAG_CLEAR(pxADC, JQOVF);
    }

    if (pxADC->Errors != ADC_ERROR_NONE)
    {
        /* Error callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.Error, pxADC);
    }
#endif
}

/**
 * @brief Sets up and enables a DMA transfer for the ADC regular conversions.
 * @param pxADC: pointer to the ADC handle structure
 * @param pvAddress: memory address to the conversion data storage
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType ADC_eStart_DMA(ADC_HandleType * pxADC, void * pvAddress)
{
    XPD_ReturnType eResult = XPD_ERROR;

#if (ADC_COUNT > 1)
    /* If multimode is used, multimode function shall be used */
    if (ADC_COMMON(pxADC)->CCR.b.DUAL == ADC_MULTIMODE_SINGE)
#endif
    {
        /* Set up DMA for transfer */
        eResult = DMA_eStart_IT(pxADC->DMA.Conversion,
                (void *)&pxADC->Inst->DR, pvAddress, pxADC->Inst->SQR1.b.L + 1);

        /* If the DMA is currently used, return with error */
        if (eResult == XPD_OK)
        {
            /* Set the callback owner */
            pxADC->DMA.Conversion->Owner = pxADC;

            /* Set the DMA transfer callbacks */
            pxADC->DMA.Conversion->Callbacks.Complete     = ADC_prvDmaConversionRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
            pxADC->DMA.Conversion->Callbacks.Error        = ADC_prvDmaErrorRedirect;
#endif

#ifdef __XPD_ADC_ERROR_DETECT
            /* Enable ADC overrun interrupt */
            ADC_IT_ENABLE(pxADC, OVR);
#endif

            /* Enable ADC DMA mode */
            ADC_REG_BIT(pxADC, CFGR, DMAEN) = 1;

            ADC_vStart(pxADC);
        }
    }

    return eResult;
}

/**
 * @brief Disables the ADC and its DMA transfer.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStop_DMA(ADC_HandleType * pxADC)
{
    /* Disable the Peripheral */
    ADC_vStop(pxADC);

#ifdef __XPD_ADC_ERROR_DETECT
    /* Disable ADC overrun interrupt */
    ADC_IT_DISABLE(pxADC, OVR);
#endif
    /* Disable the selected ADC DMA mode */
    ADC_REG_BIT(pxADC, CFGR, DMAEN) = 0;

    /* Disable the ADC DMA Stream */
    DMA_vStop_IT(pxADC->DMA.Conversion);
}

/**
 * @brief Initializes the analog watchdog using the setup configuration.
 * @param pxADC: pointer to the ADC handle structure
 * @param eWatchdog: the analog watchdog selection
 * @param usLowThd: low threshold of the watchdog
 * @param usHighThd: high threshold of the watchdog
 */
void ADC_vWatchdogConfig(
        ADC_HandleType *    pxADC,
        ADC_WatchdogType    eWatchdog,
        uint16_t            usLowThd,
        uint16_t            usHighThd)
{
    /* Configure when ADC is stopped */
    if ((eWatchdog != ADC_AWD_NONE) && ((pxADC->Inst->CR.w & ADC_STARTCTRL) == 0))
    {
        uint32_t ulScaling   = pxADC->Inst->CFGR.b.RES * 2;
        __IO uint32_t *pulTR = (&pxADC->Inst->TR1.w + eWatchdog - ADC_AWD1);

        /* Analog watchdogs configuration */
        if (eWatchdog == ADC_AWD1)
        {
            /* Shift the offset in function of the selected ADC resolution:
             * Thresholds have to be left-aligned on bit 11, the LSB (right bits)
             * are set to 0 */
            *pulTR = (usHighThd  << (16 + ulScaling)) | (usLowThd  << ulScaling);
        }
        else
        {
            /* Shift the threshold in function of the selected ADC resolution
             * have to be left-aligned on bit 7, the LSB (right bits) are set to 0 */
            if (ulScaling < 5)
            {
                *pulTR = (usHighThd  << (20 - ulScaling)) | (usLowThd  << (4 - ulScaling));
            }
            else
            {
                *pulTR = (usHighThd  << (20 - ulScaling)) | (usLowThd  >> (ulScaling - 4));
            }
        }
    }
}

/**
 * @brief Returns the currently active analog watchdog.
 * @param pxADC: pointer to the ADC handle structure
 * @return The watchdog number if an active watchdog triggered an interrupt, 0 otherwise
 */
ADC_WatchdogType ADC_eWatchdogStatus(ADC_HandleType * pxADC)
{
    /* Update the active watchdog variable with current flag status */
    pxADC->ActiveWatchdog *= pxADC->Inst->ISR.w >> (pxADC->ActiveWatchdog + ADC_ISR_AWD1_Pos - 1);
    return pxADC->ActiveWatchdog;
}

/** @} */

/** @} */

/** @addtogroup ADC_Injected
 * @{ */

/** @defgroup ADC_Injected_Exported_Functions ADC Injected Exported Functions
 * @{ */

/**
 * @brief Initializes a injected ADC channel for conversion using the setup configuration.
 * @param pxADC: pointer to the ADC handle structure
 * @param pxConfig: pointer to ADC injected channel setup configuration
 */
void ADC_vInjectedInit(ADC_HandleType * pxADC, const ADC_InjectedInitType * pxConfig)
{
    /* Save trigger configuration into context queue */
    if (pxConfig->TriggerEdge != EDGE_NONE)
    {
        pxADC->InjectedConfig =
                (pxConfig->TriggerSource << ADC_JSQR_JEXTSEL_Pos) |
                (pxConfig->TriggerEdge   << ADC_JSQR_JEXTEN_Pos);
    }
    else
    {
        pxADC->InjectedConfig = 0;
    }

    /* Set configuration that is restricted to no ongoing injected conversions */
    if (ADC_REG_BIT(pxADC, CR, JADSTART) == 0)
    {
        uint32_t ulCfgr, ulCfgrMask;

        ulCfgrMask = ADC_CFGR_JQM | ADC_CFGR_JDISCEN;

        /* Config only affects bits in the high halfword */
        ulCfgr = pxConfig->w << 16;

        /* Cannot use both auto-injected mode and discontinuous mode simultaneously */
        if (pxConfig->AutoInjection != 0)
        {
            CLEAR_BIT(ulCfgr, ADC_CFGR_JDISCEN);
        }

        /* Set configuration that is restricted to no ongoing conversions */
        if ((pxADC->Inst->CR.w & ADC_STARTCTRL) == 0)
        {
            /* Auto-injection is triggered by end of regular group conversion,
             * external triggers are not allowed */
            ulCfgrMask |= ADC_CFGR_JAUTO;

            /* Set oversampling mode configuration */
            if (pxConfig->Oversampling.State == ENABLE)
            {
                MODIFY_REG(pxADC->Inst->CFGR2.w,
                        ADC_CFGR2_JOVSE | ADC_CFGR2_OVSR | ADC_CFGR2_OVSS |
                        ADC_CFGR2_TROVS | ADC_CFGR2_ROVSM,
                        pxConfig->Oversampling.w | ADC_CFGR2_JOVSE);
            }
        }

        MODIFY_REG(pxADC->Inst->CFGR.w, ulCfgrMask, ulCfgr);
    }
}

/**
 * @brief Initializes the injected ADC channels for conversion using the setup configuration.
 * @param pxADC: pointer to the ADC handle structure
 * @param axChannels: ADC injected channel configuration array pointer
 * @param ucChannelCount: number of channels to configure
 */
void ADC_vInjectedChannelConfig(
        ADC_HandleType *            pxADC,
        const ADC_ChannelInitType   axChannels[],
        uint8_t                     ucChannelCount)
{
    uint8_t i, ucAWD1Chs;
    uint32_t ulSeqOffset = ADC_JSQR_JSQ1_Pos, ulJSQR = 0;

    /* Forward to channel common configurator */
    ucAWD1Chs = ADC_prvCommonChannelConfig(pxADC, axChannels, ucChannelCount);

    /* Configuration of AWD1 */
    if ((pxADC->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        uint32_t ulClrMask = ADC_CFGR_AWD1SGL;
        uint32_t ulSetMask = 0;

        if (ucAWD1Chs == 0)
        {
            ulClrMask = ADC_CFGR_JAWD1EN | ADC_CFGR_AWD1SGL;
        }
        else
        {
            ulSetMask = ADC_CFGR_JAWD1EN;

            if (ucAWD1Chs == 1)
            {
                /* In case of a single channel, set SGL flag
                 * AWD1CH is set in common configurator */
                ulSetMask = ADC_CFGR_JAWD1EN | ADC_CFGR_AWD1SGL;
            }

            /* Disable other group's watchdog if it targeted a single channel */
            if ((pxADC->Inst->CFGR.w & (ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN)) ==
                                       (ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN))
            {
                ulClrMask = ADC_CFGR_AWD1EN;
            }
        }
        pxADC->Inst->CFGR.w = (pxADC->Inst->CFGR.w & (~ulClrMask)) | ulSetMask;
    }

    /* Assemble injected sequence */
    for (i = 0; i < ucChannelCount; i++)
    {
        /* Channel configuration for a given rank */
        MODIFY_REG(ulJSQR, ADC_SQR_MASK << ulSeqOffset, axChannels[i].Number << ulSeqOffset);

        ulSeqOffset += ADC_SQR_SIZE;
    }

    /* Set the injected sequence length */
    MODIFY_REG(ulJSQR, ADC_JSQR_JL, (ucChannelCount - 1) << ADC_JSQR_JL_Pos);

    /* Finally apply the injected setup to the register */
    pxADC->Inst->JSQR.w = ulJSQR | pxADC->InjectedConfig;
}

/**
 * @brief Enables the ADC peripheral and starts injected conversion
 *        if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStart(ADC_HandleType * pxADC)
{
    uint32_t ulDual;

    /* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization */
    if (ADC_REG_BIT(pxADC,CR,JADSTART) == 0)
    {
        /* Enable the Peripheral */
        if (ADC_prvEnableInst(pxADC->Inst))
        {
            /* Delay for ADC stabilization time */
            XPD_vDelay_us(ADC_STAB_DELAY_US);
        }
    }

    /* clear injected group conversion flag */
    SET_BIT(pxADC->Inst->ISR.w, ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_JQOVF);

#if (ADC_COUNT > 1)
    ulDual = ADC_COMMON(pxADC)->CCR.b.DUAL;
    /* if no external trigger present and ADC is either not in multimode,
     * or the multimode master, enable software conversion of injected channels */
    if ((pxADC->Inst->CFGR.b.JAUTO == 0) &&
        (   (ulDual == ADC_MULTIMODE_SINGE)
         || (ulDual == ADC_MULTIMODE_DUAL_REGSIMULT)
         || (ulDual == ADC_MULTIMODE_DUAL_INTERLEAVED)
         || (IS_ADC_MULTIMODE_MASTER_INSTANCE(pxADC->Inst))))
#endif
    {
        ADC_REG_BIT(pxADC,CR,JADSTART) = 1;
    }
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @note  In multimode configuration, the master ADC shall be stopped first.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStop(ADC_HandleType * pxADC)
{
    /* Stop conversion on injected group only */
    ADC_prvStopConversion(pxADC, ADC_CR_JADSTART);

    /* Disable ADC peripheral if injected conversions are effectively stopped */
    if ((pxADC->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        ADC_prvDisableInst(pxADC->Inst);
    }
}

/**
 * @brief Enables the ADC peripheral and interrupts, and starts injected conversion
 *        if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStart_IT(ADC_HandleType * pxADC)
{
    uint32_t ulITs = pxADC->EndFlagSelection & (ADC_IER_JEOCIE | ADC_IER_JEOSIE);
    /* ADC overrun and end of conversion interrupt for regular group */
#ifdef __XPD_ADC_ERROR_DETECT
    ulITs |= ADC_IER_JQOVFIE;
#endif
    SET_BIT(pxADC->Inst->IER.w, ulITs);

    ADC_vInjectedStart(pxADC);
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral and interrupts.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @note  In multimode configuration, the master ADC shall be stopped first.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStop_IT(ADC_HandleType * pxADC)
{
    ADC_vInjectedStop(pxADC);

    CLEAR_BIT(pxADC->Inst->IER.w, ADC_IER_JEOCIE | ADC_IER_JEOSIE | ADC_IER_JQOVFIE);
}

/** @} */

/** @} */

#if (ADC_COUNT > 1)
/** @addtogroup ADC_MultiMode
 * @{ */

/** @defgroup ADC_MultiMode_Exported_Functions Multi ADC Mode Exported Functions
 * @{ */

/**
 * @brief Initializes a multi ADC configuration.
 * @param pxADC: pointer to the ADC handle structure
 * @param pxConfig: pointer to the multi ADC setup configuration
 */
void ADC_vMultiModeInit(ADC_HandleType * pxADC, const ADC_MultiModeInitType * pxConfig)
{
    __IO uint32_t * pulCR = &ADC_PAIR(pxADC)->CR.w;
    ADC_Common_TypeDef *pxCommon = ADC_COMMON(pxADC);

    /* Neither ADC should be running regular conversion */
    if (    ((pxADC->Inst->CR.w & ADC_CR_ADSTART) == 0)
         && ((           *pulCR & ADC_CR_ADSTART) == 0))
    {
        pxCommon->CCR.b.MDMA = pxConfig->DMAAccessMode;
        pxCommon->CCR.b.DMACFG = pxADC->Inst->CFGR.b.DMACFG;

        /* Neither ADC should be on */
        if (    ((pxADC->Inst->CR.w & ADC_CR_ADEN) == 0)
             && ((           *pulCR & ADC_CR_ADEN) == 0))
        {
            pxCommon->CCR.b.DUAL  = pxConfig->Mode;
            pxCommon->CCR.b.DELAY = pxConfig->InterSamplingDelay - 1;
        }
    }
}

/**
 * @brief Sets up and enables a DMA transfer for the multi ADC regular conversions.
 * @param pxADC: pointer to the ADC handle structure
 * @param pvAddress: memory address to the conversion data storage
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType ADC_eMultiModeStart_DMA(ADC_HandleType * pxADC, void * pvAddress)
{
    XPD_ReturnType eResult = XPD_ERROR;

    /* Perform ADC enable and conversion start if no conversion is on going
     * (check on ADC master only) */
    if ((ADC_REG_BIT(pxADC,CR,ADSTART) == 0) && ((ADC_INDEX(pxADC) & 1) == 0))
    {
        /* enable both ADCs */
        ADC_prvEnableInst(pxADC->Inst);
        ADC_prvEnableInst(ADC_PAIR(pxADC));

        /* Set up DMA for transfer */
        eResult = DMA_eStart_IT(pxADC->DMA.Conversion,
                (void *)&ADC_COMMON(pxADC)->CDR.w, pvAddress, pxADC->Inst->SQR1.b.L + 1);

        /* If the DMA is currently used, return with error */
        if (eResult == XPD_OK)
        {
            /* Set the callback owner */
            pxADC->DMA.Conversion->Owner = pxADC;

            /* Set the DMA transfer callbacks */
            pxADC->DMA.Conversion->Callbacks.Complete     = ADC_prvDmaConversionRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
            pxADC->DMA.Conversion->Callbacks.Error        = ADC_prvDmaErrorRedirect;
#endif

#ifdef __XPD_ADC_ERROR_DETECT
            /* Enable ADC overrun interrupt */
            ADC_IT_ENABLE(pxADC, OVR);
#endif

            /* Enable the ADC peripherals: master and slave
             * (in case if not already enabled previously) */
            ADC_PAIR(pxADC)->CR.b.ADEN = 1;

            /* Enable ADC DMA mode */
            ADC_REG_BIT(pxADC, CFGR, DMAEN) = 1;

            ADC_vStart(pxADC);
        }
    }

    return eResult;
}

/**
 * @brief Disables the ADC and the common DMA transfer.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vMultiModeStop_DMA(ADC_HandleType * pxADC)
{
    ADC_TypeDef *pxPair = ADC_PAIR(pxADC);
    uint32_t ulTimeout = ADC_STOP_CONVERSION_TIMEOUT;

    ADC_vStop_DMA(pxADC);

    /* Wait for effective finish of slave conversion */
    if (XPD_OK == XPD_eWaitForMatch(&pxPair->CR.w, ADC_CR_ADSTART, 0, &ulTimeout))
    {
        /* disable pair */
        ADC_prvDisableInst(pxPair);
    }
}

/** @} */

/** @} */
#endif

/** @addtogroup ADC_Calibration
 * @{ */

/** @defgroup ADC_Calibration_Exported_Functions ADC Calibration Exported Functions
 * @{ */

/**
 * @brief Executes an ADC self-calibration sequence.
 * @param pxADC: pointer to the ADC handle structure
 * @param eDifferential: differential mode calibration flag
 * @return Result of the operation
 */
XPD_ReturnType ADC_eCalibrate(ADC_HandleType * pxADC, bool eDifferential)
{
    XPD_ReturnType eResult = XPD_ERROR;

    /* Calibration prerequisite: ADC must be disabled. */
    if (ADC_REG_BIT(pxADC, CR, ADEN) == 0)
    {
        uint32_t ulTimeout = ADC_CALIBRATION_TIMEOUT;

        /* Select calibration mode single ended or differential ended */
        ADC_REG_BIT(pxADC, CR, ADCALDIF) = eDifferential;

        /* Start ADC calibration */
        ADC_REG_BIT(pxADC, CR, ADCAL) = 1;

        /* Wait for ADCAL to return to 0 */
        eResult = XPD_eWaitForDiff(&pxADC->Inst->CR.w,
                ADC_CR_ADCAL, ADC_CR_ADCAL, &ulTimeout);
    }
    return eResult;
}

/** @} */

/** @} */

/** @} */
