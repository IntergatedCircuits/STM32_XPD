/**
  ******************************************************************************
  * @file    xpd_adc.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-02-04
  * @brief   STM32 eXtensible Peripheral Drivers Analog Digital Converter Module
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

#include "xpd_adc.h"
#include "xpd_rcc.h"
#include "xpd_utils.h"

#if defined(USE_XPD_ADC)

/** @addtogroup ADC
 * @{ */

/** @addtogroup ADC_Core
 * @{ */

#define ADC_STAB_DELAY_US           10
#define ADC_TEMPSENSOR_DELAY_US     10
#define ADC_CALIBRATION_TIMEOUT     10
#define ADC_CONVERSION_TIME_MAX_CPU_CYCLES ((uint32_t) 156928)
#define ADC_STOP_CONVERSION_TIMEOUT 11

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
static const XPD_CtrlFnType adc_clkCtrl[] = {
#if (ADC_COUNT == 1)
        XPD_ADC1_ClockCtrl
#elif (ADC_COUNT == 2)
        XPD_ADC12_ClockCtrl
#elif (ADC_COUNT == 4)
        XPD_ADC12_ClockCtrl,
        XPD_ADC34_ClockCtrl
#endif
};

static volatile uint8_t adc_users[] = {
        0,
#if (ADC_COUNT == 4)
        0
#endif
};

static void adc_clockCtrl(ADC_HandleType * hadc, FunctionalState ClockState)
{
    uint8_t adcx = ADC_INDEX(hadc);
#if (ADC_COUNT == 4)
    uint8_t index = adcx / 2;
#else
    uint8_t index = 0;
#endif

    if (ClockState == DISABLE)
    {
        CLEAR_BIT(adc_users[index], 1 << adcx);
    }
    else
    {
        SET_BIT(adc_users[index], 1 << adcx);
    }

    adc_clkCtrl[index]((adc_users[index] > 0) ? ENABLE : DISABLE);
}
#else
#define adc_clockCtrl(HANDLE, STATE)    (XPD_ADC1_ClockCtrl(STATE))
#endif

__STATIC_INLINE uint32_t adc_getMultiCfgr(ADC_HandleType * hadc, uint32_t dual)
{
    uint32_t cfgr;
    /* Get relevant register CFGR in ADC instance of ADC master or slave
     * in function of multimode state (for devices with multimode available). */
    if (    (dual == ADC_MULTIMODE_SINGE)
         || (IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Inst)))
    {
        cfgr = hadc->Inst->CFGR.w;
    }
    else
    {
        cfgr = ADC_PAIR(hadc)->CFGR.w;
    }
    return cfgr;
}

static void adc_dmaConversionRedirect(void *hdma)
{
    ADC_HandleType* hadc = (ADC_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    XPD_SAFE_CALLBACK(hadc->Callbacks.ConvComplete, hadc);
}

#ifdef USE_XPD_DMA_ERROR_DETECT
static void adc_dmaErrorRedirect(void *hdma)
{
    ADC_HandleType* hadc = (ADC_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    XPD_SAFE_CALLBACK(hadc->Callbacks.Error, hadc);
}
#endif

/* Enables the peripheral */
static boolean_t adcx_enable(ADC_TypeDef * ADCx)
{
    /* Requirements:
     * ADCAL, ADSTP, JADSTP, ADSTART, JADSTART, ADDIS, ADEN = 0 */
    boolean_t success = (0 == (ADCx->CR.w & (ADC_CR_ADEN | ADC_STARTCTRL
            | ADC_STOPCTRL | ADC_CR_ADCAL | ADC_CR_ADDIS)));
    if (success)
    {
        SET_BIT(ADCx->CR.w, ADC_CR_ADEN);
    }
    return success;
}

/* Disables the peripheral */
static void adcx_disable(ADC_TypeDef * ADCx)
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
static void adc_sampleTimeConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Channel)
{
    uint32_t regoffset = 0, number = Channel->Number;

    /* Channel sampling time configuration */
    if (number > 10)
    {
        regoffset = 1;
        number -= 10;
    }
    number *= 3;

    /* set the sample time */
    MODIFY_REG((&hadc->Inst->SMPR1.w)[regoffset],
            ADC_SMPR1_SMP0 << number, Channel->SampleTime << number);
}

/* Enables an internal channel for conversion */
static void adc_initInternalChannel(ADC_HandleType * hadc, uint8_t Channel)
{
    uint32_t chbit = 0;

    switch (Channel)
    {
        case ADC1_VBAT_CHANNEL:
            /* enable the VBAT input */
#if (ADC_COUNT > 1)
            if (hadc->Inst == ADC1)
#endif
            { chbit = ADC_CCR_VBATEN; }
            break;

        case ADC1_VREFINT_CHANNEL:
            /* enable the VREF input */
            { chbit = ADC_CCR_VREFEN; }
            break;

        case ADC1_TEMPSENSOR_CHANNEL:
            /* enable the TS input */
#if (ADC_COUNT > 1)
            if (hadc->Inst == ADC1)
#endif
            { chbit = ADC_CCR_TSEN; }
            break;

        default:
            break;
    }

    /* Check if setting is valid */
    if (((chbit != 0) && ((ADC_COMMON(hadc)->CCR.w & chbit) == 0))
        /* Software is allowed to change common parameters only when all ADCs
         * of the common group are disabled */
        &&  (ADC_REG_BIT(hadc, CR, ADEN) == 0)
#if (ADC_COUNT > 1)
         && (ADC_PAIR(hadc)->CR.b.ADEN == 0)
#endif
             )
    {
        SET_BIT(ADC_COMMON(hadc)->CCR.w, chbit);

        if (Channel == ADC1_TEMPSENSOR_CHANNEL)
        {
            /* wait temperature sensor stabilization */
            XPD_Delay_us(ADC_TEMPSENSOR_DELAY_US);
        }
    }
}

/* Stops the further conversions of the selected types */
static void adc_stopConversion(ADC_HandleType * hadc, uint32_t convFlag)
{
    uint32_t timeout = ADC_STOP_CONVERSION_TIMEOUT;

    /* Verification if ADC is not already stopped */
    if ((hadc->Inst->CR.w & ADC_STARTCTRL) != 0)
    {
        /* In auto-injection mode, regular group stop ADC_CR_ADSTP is used (not 
         * injected group stop ADC_CR_JADSTP) */
        if ((hadc->Inst->CFGR.w & (ADC_CFGR_JAUTO | ADC_CFGR_CONT | ADC_CFGR_AUTDLY))
                               == (ADC_CFGR_JAUTO | ADC_CFGR_CONT | ADC_CFGR_AUTDLY))
        {
            uint32_t counter;

            /* Use stop of regular group */
            convFlag = ADC_CR_ADSTART;

            /* Wait until JEOS=1 (maximum Timeout: 4 injected conversions) */
            for (counter = 0; counter < (ADC_CONVERSION_TIME_MAX_CPU_CYCLES * 4); counter++)
            {
                if (XPD_ADC_GetFlag(hadc, JEOS) == 0)
                {
                    break;
                }
            }

            /* Clear JEOS */
            XPD_ADC_ClearFlag(hadc, JEOS);
        }

        /* Stop potential conversion on going on regular group */
        if (convFlag != ADC_CR_JADSTART)
        {
            /* Software is allowed to set ADSTP only when ADSTART=1 and ADDIS=0 */
            if ((hadc->Inst->CR.w & (ADC_CR_ADSTART | ADC_CR_ADDIS)) == ADC_CR_ADSTART)
            {
                /* Stop conversions on regular group */
                ADC_REG_BIT(hadc, CR, ADSTP) = 1;
            }
        }

        /* Stop potential conversion on going on injected group */
        if (convFlag != ADC_CR_ADSTART)
        {
            /* Software is allowed to set JADSTP only when JADSTART=1 and ADDIS=0 */
            if ((hadc->Inst->CR.w & (ADC_CR_JADSTART | ADC_CR_ADDIS)) == ADC_CR_JADSTART)
            {
                /* Stop conversions on injected group */
                ADC_REG_BIT(hadc, CR, JADSTP) = 1;
            }
        }

        /* Wait for conversion effectively stopped */
        XPD_WaitForMatch(&hadc->Inst->CR.w, convFlag, 0, &timeout);
    }
}

/* Attempts to set the offset configuration of the input channel list */
static void adc_offsetConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Channels,
        uint8_t ChannelCount)
{
    union {
        struct {
            __IO uint32_t OFFSET : 12;
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET_CH : 5;
            __IO uint32_t OFFSET_EN : 1;
        } b;
        __IO uint32_t w;
    } *pOFR;
    uint32_t channels = 0, offsetNeeds = 0, i;

    /* Collect offset related information from list */
    for (i = 0; i < ChannelCount; i++)
    {
        SET_BIT(channels, Channels[i].Number);
        if (Channels[i].Offset > 0)
        {
            SET_BIT(offsetNeeds, Channels[i].Number);
        }
    }

    /* If there are channels that don't need offset, but are previously configured */
    channels = hadc->OffsetUsage & channels & (~offsetNeeds);
    if (channels != 0)
    {
        /* Find the channels in the offsets, clear config */
        for (pOFR = (void*)&hadc->Inst->OFR1.w;
                ((uint32_t)pOFR) <= ((uint32_t)&hadc->Inst->OFR4.w);
                pOFR++)
        {
            i = 1 << pOFR->b.OFFSET_CH;
            if ((channels & i) != 0)
            {
                pOFR->w = 0;
                CLEAR_BIT(hadc->OffsetUsage, i);
            }
        }
    }
    /* Iterate until all desired offsets are set */
    for (i = 0; (hadc->OffsetUsage != offsetNeeds) && (i < ChannelCount); i++)
    {
        /* If the current channel needs an offset */
        if (Channels[i].Offset > 0)
        {
            /* For the channels that are missing offset */
            for (pOFR = (void*)&hadc->Inst->OFR1.w;
                    ((uint32_t)pOFR) <= ((uint32_t)&hadc->Inst->OFR4.w);
                    pOFR++)
            {
                /* If the offset register is disabled, or already used by this channel */
                if ((pOFR->w == 0) || ((offsetNeeds & (1 << pOFR->b.OFFSET_CH)) != 0))
                {
                    pOFR->b.OFFSET    = Channels[i].Offset;
                    pOFR->b.OFFSET_CH = Channels[i].Number;
                    pOFR->b.OFFSET_EN = 1;
                    SET_BIT(hadc->OffsetUsage, Channels[i].Number);
                }
            }
        }
    }
}

/** @addtogroup ADC_Core_Exported_Functions
 * @{ */

/**
 * @brief Initializes the ADC peripheral using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to ADC setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_ADC_Init(ADC_HandleType * hadc, const ADC_InitType * Config)
{
    XPD_ReturnType result = XPD_ERROR;

    /* enable clock */
    adc_clockCtrl(hadc, ENABLE);

    /* Initialize ADC API internal variables */
    hadc->InjectedContextQueue = 0;
    hadc->ConversionCount = 0;
    hadc->OffsetUsage = 0;

    /* Set used EndFlag */
    if (Config->EndFlagSelection == ADC_EOC_SEQUENCE)
    {
        hadc->EndFlagSelection = ADC_ENDFLAG_SEQUENCE;
    }
    else
    {
        hadc->EndFlagSelection = ADC_ENDFLAG_CONVERSION;
    }

    /* Enable voltage regulator (if disabled at this step) */
    if (hadc->Inst->CR.b.ADVREGEN != 1)
    {
        /* ADC voltage regulator enable sequence */
#ifdef ADC_CR_ADVREGEN_1
        hadc->Inst->CR.b.ADVREGEN = 0;
#endif
        hadc->Inst->CR.b.ADVREGEN = 1;

        /* Delay for ADC stabilization time */
        XPD_Delay_us(ADC_STAB_DELAY_US);
    }

    /* ADC regular conversion stopped */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        ADC_REG_BIT(hadc, CFGR, CONT)  = Config->ContinuousMode;
        ADC_REG_BIT(hadc, CFGR, ALIGN) = Config->LeftAlignment;
        hadc->Inst->CFGR.b.RES         = Config->Resolution;

        /* external trigger configuration */
        if(Config->Trigger.Source == ADC_TRIGGER_SOFTWARE)
        {
            /* reset the external trigger */
            CLEAR_BIT(hadc->Inst->CFGR.w, ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN);
        }
        else
        {
            /* select external trigger and polarity to start conversion */
            hadc->Inst->CFGR.b.EXTSEL = Config->Trigger.Source;
            hadc->Inst->CFGR.b.EXTEN  = Config->Trigger.Edge;
        }

        /* Enable discontinuous mode only if continuous mode is disabled */
        if ((Config->DiscontinuousCount != 0) && (Config->ContinuousMode == DISABLE))
        {
            ADC_REG_BIT(hadc, CFGR, DISCEN) = 1;
            hadc->Inst->CFGR.b.DISCNUM = Config->DiscontinuousCount - 1;
        }
        else
        {
            CLEAR_BIT(hadc->Inst->CFGR.w, ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM);
        }

        /* No injected or regular conversion ongoing */
        if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
        {
            ADC_REG_BIT(hadc, CFGR, AUTDLY) = Config->LPAutoWait;
            ADC_REG_BIT(hadc, CFGR, DMACFG) = Config->ContinuousDMARequests;
        }

        /* Set conversion count as flag when scan mode is selected */
        hadc->ConversionCount = Config->ScanMode & 1;

        result = XPD_OK;
    }

    /* dependencies initialization */
    XPD_SAFE_CALLBACK(hadc->Callbacks.DepInit, hadc);

    return result;
}

/**
 * @brief Deinitializes the ADC peripheral.
 * @param hadc: pointer to the ADC handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_ADC_Deinit(ADC_HandleType * hadc)
{
    /* Stop potential conversion on going, on regular and injected groups */
    adc_stopConversion(hadc, ADC_STARTCTRL);

    /* Enable injected queue sequencer after injected conversion stop */
    ADC_REG_BIT(hadc, CFGR, JQM) = 1;

    /* Disable the ADC peripheral */
    adcx_disable(hadc->Inst);

    /* Reset register IER */
    hadc->Inst->IER.w = 0;

    /* Reset register ISR */
    hadc->Inst->ISR.w = ~0;

    /* Reset OFR registers */
    hadc->Inst->OFR1.w = 0;
    hadc->Inst->OFR2.w = 0;
    hadc->Inst->OFR3.w = 0;
    hadc->Inst->OFR4.w = 0;

    /* Disable voltage regulator */
    CLEAR_BIT(hadc->Inst->CR.w, ADC_CR_ADVREGEN | ADC_CR_ADCALDIF);
#ifdef ADC_CR_ADVREGEN_1
    SET_BIT(hadc->Inst->CR.w, ADC_CR_ADVREGEN_1);
#endif

    /* Reset register CFGR */
    hadc->Inst->CFGR.w = 0;

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(hadc->Callbacks.DepDeinit, hadc);

    /* disable clock */
    adc_clockCtrl(hadc, DISABLE);

    return XPD_OK;
}

/**
 * @brief Initializes the regular ADC channels for conversion using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Channels: ADC regular channel configuration array pointer
 * @param ChannelCount: number of channels to configure
 */
void XPD_ADC_ChannelConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Channels,
        uint8_t ChannelCount)
{
    /* No regular conversion ongoing */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        uint8_t i;
        uint32_t seqOffset = ADC_SQR1_SQ1_Pos;
        __IO uint32_t *pSQR = &hadc->Inst->SQR1.w;

        for (i = 0; i < ChannelCount; i++)
        {
            /* Sequencer configuration */
            MODIFY_REG(*pSQR, ADC_SQR_MASK << seqOffset, Channels[i].Number << seqOffset);

            /* Advance to next sequence element */
            seqOffset += ADC_SQR_SIZE;

            /* Jump to next register when the current one is filled */
            if (seqOffset > (32 - ADC_SQR_SIZE))
            {
                pSQR += ADC_SQR_REGDIR;
                seqOffset = 0;
            }
        }

        /* No injected conversion ongoing */
        if (ADC_REG_BIT(hadc, CR, JADSTART) == 0)
        {
            uint8_t wdgUsers = 0;
            uint32_t watchdogMasks[4] = {0, 0, 0, 0};

            /* Channel sampling time configuration */
            for (i = 0; i < ChannelCount; i++)
            {
                /* Sample time configuration */
                adc_sampleTimeConfig(hadc, &Channels[i]);

                watchdogMasks[1] = 1 << Channels[i].Number;
                watchdogMasks[0] |= watchdogMasks[1];

                /* If a watchdog is used for the channel, set it in the configuration */
                if (Channels[i].Watchdog == ADC_AWD1)
                {
                    hadc->Inst->CFGR.b.AWD1CH = Channels[i].Number;
                    wdgUsers++;
                }
                /* Store channel flag for channel-wise watchdog configuration */
                else if (Channels[i].Watchdog != ADC_AWD_NONE)
                {
                    watchdogMasks[Channels[i].Watchdog] |= watchdogMasks[1];
                }
            }

            /* Determine watchdog1 configuration based on user count */
            switch (wdgUsers)
            {
                case 0:
                    CLEAR_BIT(hadc->Inst->CFGR.w,
                            ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN);
                    break;

                case 1:
                    CLEAR_BIT(hadc->Inst->CFGR.w,
                            ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN);
                    break;

                default:
                {
                    uint32_t clrMask = ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN;

                    /* Clear single injected channel monitoring if exists */
                    if ((hadc->Inst->CFGR.w & (ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN))
                                           == (ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN) )
                    {
                        clrMask |= ADC_CFGR_JAWD1EN;
                    }
                    /* All regular channels will be monitored,
                     * even if they were not configured so! */
                    MODIFY_REG(hadc->Inst->CFGR.w,
                            clrMask, ADC_CFGR_AWD1EN);
                    break;
                }
            }

            /* Clear unmonitored channel flags, set monitored ones */
            MODIFY_REG(hadc->Inst->AWD2CR.w, watchdogMasks[0], watchdogMasks[2]);
            MODIFY_REG(hadc->Inst->AWD3CR.w, watchdogMasks[0], watchdogMasks[3]);

            /* Set offsets configuration */
            adc_offsetConfig(hadc, Channels, ChannelCount);
        }

        /* ADC disabled */
        if (ADC_REG_BIT(hadc, CR, ADEN) == 0)
        {
            /* Channel sampling time configuration */
            for (i = 0; i < ChannelCount; i++)
            {
                /* Internal channel configuration (if applicable) */
                adc_initInternalChannel(hadc, Channels[i].Number);

                /* Configuration of differential mode */
                if (Channels[i].Differential == ENABLE)
                {
                    ADC_ChannelInitType diffPair = Channels[i];

                    /* Enable differential mode */
                    SET_BIT(hadc->Inst->DIFSEL.w, 1 << Channels[i].Number);

                    /* Pair channel sampling time configuration */
                    diffPair.Number++;
                    adc_sampleTimeConfig(hadc, &diffPair);
                }
                else
                {
                    /* Disable differential mode (default mode: single-ended) */
                    CLEAR_BIT(hadc->Inst->DIFSEL.w, 1 << Channels[i].Number);
                }
            }
        }

        /* Init() sets this variable as flag to indicate ScanMode */
        if (hadc->ConversionCount > 0)
        {
            hadc->ConversionCount = ChannelCount;

            /* Set number of ranks in regular group sequencer */
            hadc->Inst->SQR1.b.L = hadc->ConversionCount - 1;
        }
        else
        {
            hadc->Inst->SQR1.b.L = 0;
        }
    }
}

/**
 * @brief Enables the ADC peripheral and starts conversion if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Start(ADC_HandleType * hadc)
{
    /* Success if not started */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        /* ADC turn ON */
        if (adcx_enable(hadc->Inst))
        {
            /* Wait until ADRDY flag is set ( < 1us) */
            uint32_t timeout = 1;
            XPD_WaitForMatch(&hadc->Inst->ISR.w, ADC_ISR_ADRDY, ADC_ISR_ADRDY, &timeout);
        }

        /* clear regular group conversion flag and overrun flag */
        SET_BIT(hadc->Inst->ISR.w, ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);

        /* if ADC is either not in multimode, or the multimode master,
         * enable software conversion of regular channels */
        if (    (ADC_COMMON(hadc)->CCR.b.DUAL == ADC_MULTIMODE_SINGE)
             || (IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Inst)))
        {
            ADC_REG_BIT(hadc, CR, ADSTART) = 1;
        }

#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
        hadc->Errors = ADC_ERROR_NONE;
#endif
    }
}

/**
 * @brief Stops the ADC peripheral.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Stop(ADC_HandleType * hadc)
{
    uint32_t timeout = 1;

    /* 1. Stop potential conversion on going, on regular and injected groups */
    adc_stopConversion(hadc, ADC_STARTCTRL);

    adcx_disable(hadc->Inst);

    XPD_WaitForMatch(&hadc->Inst->ISR.w, ADC_ISR_ADRDY, 0, &timeout);
}

/**
 * @brief Polls the status of the ADC operation(s).
 * @param hadc: pointer to the ADC handle structure
 * @param Operation: the type of operation to check
 * @param Timeout: the timeout in ms for the polling.
 * @return ERROR if there was an input error, TIMEOUT if timed out, OK if successful
 */
XPD_ReturnType XPD_ADC_PollStatus(ADC_HandleType * hadc, ADC_OperationType Operation, uint32_t Timeout)
{
    XPD_ReturnType result = XPD_OK;

    if ((Operation == ADC_OPERATION_CONVERSION)
            && ((hadc->EndFlagSelection & ADC_IER_EOS) != 0))
    {
        Operation = ADC_ISR_EOS;
    }
    else if ((Operation == ADC_OPERATION_INJCONVERSION)
            && ((hadc->EndFlagSelection & ADC_IER_JEOS) != 0))
    {
        Operation = ADC_ISR_JEOS;
    }

    /* Polling for single conversion is not allowed when DMA is used,
     * and EOC is raised on end of single conversion */
    if (Operation == ADC_OPERATION_CONVERSION)
    {
        /* If multimode used, check common DMA config, otherwise dedicated DMA config */
        if (ADC_COMMON_REG_BIT(hadc, CCR, DUAL) == 0)
        {
            if (ADC_REG_BIT(hadc, CFGR, DMAEN) == 1)
            { result = XPD_ERROR; }
        }
        else
        {
            if (ADC_COMMON_REG_BIT(hadc, CCR, MDMA) == 1)
            { result = XPD_ERROR; }
        }
    }

    if (result == XPD_OK)
    {
        result = XPD_WaitForMatch(&hadc->Inst->ISR.w, Operation, Operation, &Timeout);

        /* Clear end of conversion flag of regular group if low power feature
         * "LowPowerAutoWait " is disabled, to not interfere with this feature
         * until data register is read. */
        if ((result == XPD_OK) && ((Operation & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
             && (ADC_REG_BIT(hadc, CFGR, AUTDLY) == 0))
        {
            SET_BIT(hadc->Inst->ISR.w, Operation);
        }
    }
    return result;
}

/**
 * @brief Enables the ADC peripheral and interrupts, and starts conversion
 *        if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Start_IT(ADC_HandleType * hadc)
{
    /* Choose interrupt source based on EOC config */
    uint32_t its = hadc->EndFlagSelection;

#ifdef USE_XPD_ADC_ERROR_DETECT
    /* ADC overrun and end of conversion interrupt for regular group */
    its |= ADC_IER_OVRIE;
#endif

    MODIFY_REG(hadc->Inst->IER.w, (ADC_IER_EOCIE | ADC_IER_EOSIE | ADC_IER_OVRIE), its);

    XPD_ADC_Start(hadc);
}

/**
 * @brief Stops the ADC peripheral and disables interrupts.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Stop_IT(ADC_HandleType * hadc)
{
    XPD_ADC_Stop(hadc);

    /* ADC end of conversion interrupt for regular and injected group */
    CLEAR_BIT(hadc->Inst->IER.w, hadc->EndFlagSelection | ADC_IER_OVRIE);
}

/**
 * @brief ADC interrupt handler that provides handle callbacks.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_IRQHandler(ADC_HandleType * hadc)
{
    uint32_t isr = hadc->Inst->ISR.w;
    uint32_t ier = hadc->Inst->IER.w;
    uint32_t dual = ADC_COMMON(hadc)->CCR.b.DUAL;

    /* End of conversion flag for regular channels */
    if (    ((isr & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
         && ((ier & (ADC_IER_EOCIE | ADC_IER_EOSIE)) != 0))
    {
        uint32_t cfgr = adc_getMultiCfgr(hadc, dual);

        /* If the conversion is not continuous / external triggered */
        if ((cfgr & (ADC_CFGR_CONT | ADC_CFGR_EXTEN)) == 0)
        {
            /* End of sequence */
            if (((isr & ADC_ISR_EOS) != 0) && (ADC_REG_BIT(hadc, CR, ADSTART) == 0))
            {
                /* Disable the ADC end of conversion interrupt for regular group */
                CLEAR_BIT(hadc->Inst->IER.w, ADC_IER_EOCIE | ADC_IER_EOSIE);
            }
        }

        /* Clear the ADC flag for regular end of conversion */
        SET_BIT(hadc->Inst->ISR.w, ADC_ISR_EOC | ADC_ISR_EOS);

        /* Conversion complete callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.ConvComplete, hadc);
    }

    /* End of conversion flag for injected channels */
    if (    ((isr & (ADC_ISR_JEOC | ADC_ISR_JEOS)) != 0)
         && ((ier & (ADC_IER_JEOCIE | ADC_IER_JEOSIE)) != 0))
    {
        uint32_t cfgr = adc_getMultiCfgr(hadc, dual);

        /* Injected conversion is not continuous or not automatic,
         * and software triggered */
        if (    (hadc->Inst->JSQR.b.JEXTEN == 0)
             || (    ((cfgr & ADC_CFGR_CONT) == 0)
                  && (hadc->Inst->CFGR.b.EXTEN == 0)
                  && (hadc->Inst->CFGR.b.JAUTO == 0)))
        {
            /* End of sequence */
            if ((isr & ADC_ISR_JEOS) != 0)
            {
                if (    (dual == ADC_MULTIMODE_SINGE)
                     || (dual == ADC_MULTIMODE_DUAL_REGSIMULT)
                     || (dual == ADC_MULTIMODE_DUAL_INTERLEAVED)
                     || (IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Inst)))
                {
                    cfgr = hadc->Inst->CFGR.w;
                }
                else
                {
                    cfgr = ADC_PAIR(hadc)->CFGR.w;
                }

                if ((cfgr & ADC_CFGR_JQM) == 0)
                {
                    if (ADC_REG_BIT(hadc, CR, JADSTART) == 0)
                    {
                        /* disable the ADC end of conversion interrupt
                         * for injected group */
                        CLEAR_BIT(hadc->Inst->IER.w, ADC_IER_JEOC | ADC_IER_JEOS);
                    }
                }
            }
        }
        /* clear the ADC flag for injected end of conversion */
        SET_BIT(hadc->Inst->ISR.w, ADC_ISR_JEOC | ADC_ISR_JEOS);

        /* injected conversion complete callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.InjConvComplete, hadc);
    }

    /* Check analog watchdog flag */
    if (    ((isr & ADC_ISR_AWD1) != 0)
         && ((ier & ADC_IER_AWD1IE) != 0))
    {
        hadc->ActiveWatchdog = ADC_AWD1;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Watchdog, hadc);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        XPD_ADC_ClearFlag(hadc, AWD1);
    }
    if (    ((isr & ADC_ISR_AWD2) != 0)
         && ((ier & ADC_IER_AWD2IE) != 0))
    {
        hadc->ActiveWatchdog = ADC_AWD2;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Watchdog, hadc);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        XPD_ADC_ClearFlag(hadc, AWD2);
    }
    if (    ((isr & ADC_ISR_AWD3) != 0)
         && ((ier & ADC_IER_AWD3IE) != 0))
    {
        hadc->ActiveWatchdog = ADC_AWD3;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Watchdog, hadc);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        XPD_ADC_ClearFlag(hadc, AWD3);
    }

#ifdef USE_XPD_ADC_ERROR_DETECT
    /* Check Overrun flag */
    if (    ((isr & ADC_ISR_OVR) != 0)
         && ((ier & ADC_IER_OVRIE) != 0))
    {
        /* Update error code */
        hadc->Errors |= ADC_ERROR_OVERRUN;

        /* Clear the Overrun flag */
        XPD_ADC_ClearFlag(hadc, OVR);
    }
    /* Check Injected Overrun flag */
    if (    ((isr & ADC_ISR_JQOVF) != 0)
         && ((ier & ADC_IER_JQOVFIE) != 0))
    {
        /* Update error code */
        hadc->Errors |= ADC_ERROR_JQOVF;

        /* Clear the Overrun flag */
        XPD_ADC_ClearFlag(hadc, JQOVF);
    }

    if (hadc->Errors != ADC_ERROR_NONE)
    {
        /* Error callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Error, hadc);
    }
#endif
}

/**
 * @brief Sets up and enables a DMA transfer for the ADC regular conversions.
 * @param hadc: pointer to the ADC handle structure
 * @param Address: memory address to the conversion data storage
 * @return ERROR if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType XPD_ADC_Start_DMA(ADC_HandleType * hadc, void * Address)
{
    XPD_ReturnType result = XPD_ERROR;

    /* If multimode is used, multimode function shall be used */
    if (ADC_COMMON(hadc)->CCR.b.DUAL == ADC_MULTIMODE_SINGE)
    {
        /* Set up DMA for transfer */
        result = XPD_DMA_Start_IT(hadc->DMA.Conversion,
                (void *)&hadc->Inst->DR, Address, hadc->Inst->SQR1.b.L + 1);

        /* If the DMA is currently used, return with error */
        if (result == XPD_OK)
        {
            /* Set the callback owner */
            hadc->DMA.Conversion->Owner = hadc;

            /* Set the DMA transfer callbacks */
            hadc->DMA.Conversion->Callbacks.Complete     = adc_dmaConversionRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
            hadc->DMA.Conversion->Callbacks.Error        = adc_dmaErrorRedirect;
#endif

#ifdef USE_XPD_ADC_ERROR_DETECT
            /* Enable ADC overrun interrupt */
            XPD_ADC_EnableIT(hadc, OVR);
#endif

            /* Enable ADC DMA mode */
            ADC_REG_BIT(hadc, CFGR, DMAEN) = 1;

            XPD_ADC_Start(hadc);
        }
    }

    return result;
}

/**
 * @brief Disables the ADC and its DMA transfer.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Stop_DMA(ADC_HandleType * hadc)
{
    /* Disable the Peripheral */
    XPD_ADC_Stop(hadc);

#ifdef USE_XPD_ADC_ERROR_DETECT
    /* Disable ADC overrun interrupt */
    XPD_ADC_DisableIT(hadc, OVR);
#endif

    /* Disable the selected ADC DMA mode */
    ADC_REG_BIT(hadc, CFGR, DMAEN) = 0;

    /* Disable the ADC DMA Stream */
    XPD_DMA_Stop_IT(hadc->DMA.Conversion);
}

/**
 * @brief Initializes the analog watchdog using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Watchdog: the analog watchdog selection
 * @param Config: pointer to analog watchdog setup configuration
 */
void XPD_ADC_WatchdogConfig(ADC_HandleType * hadc, ADC_WatchdogType Watchdog,
        const ADC_WatchdogThresholdType * Config)
{
    /* No injected or regular conversion ongoing */
    if ((Watchdog != ADC_AWD_NONE) && ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0))
    {
        uint32_t scaling = hadc->Inst->CFGR.b.RES * 2;
        uint32_t trx;

        /* Analog watchdogs configuration */
        if (Watchdog == ADC_AWD1)
        {
            /* Shift the offset in function of the selected ADC resolution:
             * Thresholds have to be left-aligned on bit 11, the LSB (right bits)
             * are set to 0 */
            hadc->Inst->TR1.w = (Config->High  << (16 + scaling))
                    | (Config->Low  << scaling);
        }
        else
        {
            /* Shift the threshold in function of the selected ADC resolution
             * have to be left-aligned on bit 7, the LSB (right bits) are set to 0 */
            if (scaling < 5)
            {
                trx = (Config->High  << (20 - scaling))
                        | (Config->Low  << (4 - scaling));
            }
            else
            {
                trx = (Config->High  << (20 - scaling))
                        | (Config->Low  >> (scaling - 4));
            }

            (&hadc->Inst->TR2.w)[Watchdog - ADC_AWD2] = trx;
        }
    }
}

/**
 * @brief Returns the currently active analog watchdog.
 * @param hadc: pointer to the ADC handle structure
 * @return The watchdog number if an active watchdog triggered an interrupt, 0 otherwise
 */
ADC_WatchdogType XPD_ADC_WatchdogStatus(ADC_HandleType * hadc)
{
    /* Update the active watchdog variable with current flag status */
    hadc->ActiveWatchdog *= hadc->Inst->ISR.w >> (hadc->ActiveWatchdog + ADC_ISR_AWD1_Pos - 1);
    return hadc->ActiveWatchdog;
}

/** @} */

/** @} */

/** @addtogroup ADC_Injected
 * @{ */

/** @defgroup ADC_Injected_Exported_Functions ADC Injected Exported Functions
 * @{ */

/**
 * @brief Initializes a injected ADC channel for conversion using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to ADC injected channel setup configuration
 */
void XPD_ADC_Injected_Init(ADC_HandleType * hadc, const ADC_Injected_InitType * Config)
{
    /* Save trigger configuration into context queue */
    if (Config->Trigger.InjSource != ADC_INJTRIGGER_SOFTWARE)
    {
        MODIFY_REG(hadc->InjectedContextQueue,
                ADC_JSQR_JEXTEN | ADC_JSQR_JEXTSEL,
                (Config->Trigger.InjSource << ADC_JSQR_JEXTSEL_Pos) |
                (Config->Trigger.Edge      << ADC_JSQR_JEXTEN_Pos));
    }
    else
    {
        CLEAR_BIT(hadc->InjectedContextQueue,
                ADC_JSQR_JEXTEN | ADC_JSQR_JEXTSEL);
    }

    /* Set configuration that is restricted to no ongoing injected conversions */
    if (ADC_REG_BIT(hadc, CR, JADSTART) == 0)
    {
        hadc->Inst->CFGR.b.JQM     = Config->ContextQueue;
        hadc->Inst->CFGR.b.JDISCEN = Config->DiscontinuousMode & (!Config->AutoInjection);

        /* Set configuration that is restricted to no ongoing conversions */
        if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
        {
            /* Auto-injection is triggered by end of regular group conversion,
             * external triggers are not allowed */
            {
                hadc->Inst->CFGR.b.JAUTO = Config->AutoInjection;
            }
        }
    }
}

/**
 * @brief Initializes the injected ADC channels for conversion using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Channels: ADC injected channel configuration array pointer
 * @param ChannelCount: number of channels to configure
 */
void XPD_ADC_Injected_ChannelConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Channels,
        uint8_t ChannelCount)
{
    uint8_t i;
    uint32_t seqOffset = ADC_JSQR_JSQ1_Pos;

    /* No ongoing conversions bound settings */
    if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        uint8_t wdgUsers = 0;
        uint32_t watchdogMasks[4] = {0, 0, 0, 0};

        /* Channel sampling time configuration */
        for (i = 0; i < ChannelCount; i++)
        {
            /* Sample time configuration */
            adc_sampleTimeConfig(hadc, &Channels[i]);

            watchdogMasks[1] = 1 << Channels[i].Number;
            watchdogMasks[0] |= watchdogMasks[1];

            /* If a watchdog is used for the channel, set it in the configuration */
            if (Channels[i].Watchdog == ADC_AWD1)
            {
                hadc->Inst->CFGR.b.AWD1CH = Channels[i].Number;
                wdgUsers++;
            }
            /* Store channel flag for channel-wise watchdog configuration */
            else if (Channels[i].Watchdog != ADC_AWD_NONE)
            {
                watchdogMasks[Channels[i].Watchdog] |= watchdogMasks[1];
            }
        }
        /* Determine watchdog1 configuration based on user count */
        switch (wdgUsers)
        {
            case 0:
                CLEAR_BIT(hadc->Inst->CFGR.w,
                        ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN);
                break;

            case 1:
                CLEAR_BIT(hadc->Inst->CFGR.w,
                        ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN);
                break;

            default:
            {
                uint32_t clrMask = ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN;

                /* Clear single regular channel monitoring if exists */
                if ((hadc->Inst->CFGR.w & (ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN))
                                       == (ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1EN) )
                {
                    clrMask |= ADC_CFGR_AWD1EN;
                }
                /* All injected channels will be monitored,
                 * even if they were not configured so! */
                MODIFY_REG(hadc->Inst->CFGR.w,
                        clrMask, ADC_CFGR_JAWD1EN);
                break;
            }
        }

        /* Clear unmonitored channel flags, set monitored ones */
        MODIFY_REG(hadc->Inst->AWD2CR.w, watchdogMasks[0], watchdogMasks[2]);
        MODIFY_REG(hadc->Inst->AWD3CR.w, watchdogMasks[0], watchdogMasks[3]);

        /* Set offsets configuration */
        adc_offsetConfig(hadc, Channels, ChannelCount);

        /* ADC disabled */
        if (ADC_REG_BIT(hadc, CR, ADEN) == 0)
        {
            /* Channel sampling time configuration */
            for (i = 0; i < ChannelCount; i++)
            {
                /* Internal channel configuration (if applicable) */
                adc_initInternalChannel(hadc, Channels[i].Number);

                /* Configuration of differential mode */
                if (Channels[i].Differential == ENABLE)
                {
                    ADC_ChannelInitType diffPair = Channels[i];

                    /* Enable differential mode */
                    SET_BIT(hadc->Inst->DIFSEL.w, 1 << Channels[i].Number);

                    /* Pair channel sampling time configuration */
                    diffPair.Number++;
                    adc_sampleTimeConfig(hadc, &diffPair);
                }
                else
                {
                    /* Disable differential mode (default mode: single-ended) */
                    CLEAR_BIT(hadc->Inst->DIFSEL.w, 1 << Channels[i].Number);
                }
            }
        }
    }

    /* Assemble injected sequence */
    for (i = 0; i < ChannelCount; i++)
    {
        /* Channel configuration for a given rank */
        MODIFY_REG(hadc->InjectedContextQueue,
                ADC_SQR_MASK << seqOffset, Channels[i].Number << seqOffset);

        seqOffset += ADC_SQR_SIZE;
    }

    /* Set the injected sequence length */
    MODIFY_REG(hadc->InjectedContextQueue,
            ADC_JSQR_JL, (ChannelCount - 1) << ADC_JSQR_JL_Pos);

    /* Finally apply the injected setup to the register */
    hadc->Inst->JSQR.w = hadc->InjectedContextQueue;
}

/**
 * @brief Enables the ADC peripheral and starts injected conversion
 *        if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Start(ADC_HandleType * hadc)
{
    uint32_t dual = ADC_COMMON(hadc)->CCR.b.DUAL;

    /* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization */
    if (ADC_REG_BIT(hadc,CR,JADSTART) == 0)
    {
        /* Enable the Peripheral */
        if (adcx_enable(hadc->Inst))
        {
            /* Delay for ADC stabilization time */
            XPD_Delay_us(ADC_STAB_DELAY_US);
        }
    }

    /* clear injected group conversion flag */
    SET_BIT(hadc->Inst->ISR.w, ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_JQOVF);

    /* if no external trigger present and ADC is either not in multimode,
     * or the multimode master, enable software conversion of injected channels */
    if ((hadc->Inst->CFGR.b.JAUTO == 0) &&
        (   (dual == ADC_MULTIMODE_SINGE)
         || (dual == ADC_MULTIMODE_DUAL_REGSIMULT)
         || (dual == ADC_MULTIMODE_DUAL_INTERLEAVED)
         || (IS_ADC_MULTIMODE_MASTER_INSTANCE(hadc->Inst))))
    {
        ADC_REG_BIT(hadc,CR,JADSTART) = 1;
    }
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @note  In multimode configuration, the master ADC shall be stopped first.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Stop(ADC_HandleType * hadc)
{
    /* Stop conversion on injected group only */
    adc_stopConversion(hadc, ADC_CR_JADSTART);

    /* Disable ADC peripheral if injected conversions are effectively stopped */
    if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        adcx_disable(hadc->Inst);
    }
}

/**
 * @brief Enables the ADC peripheral and interrupts, and starts injected conversion
 *        if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Start_IT(ADC_HandleType * hadc)
{
    uint32_t its = hadc->EndFlagSelection & (ADC_IER_JEOCIE | ADC_IER_JEOSIE);
    /* ADC overrun and end of conversion interrupt for regular group */
#ifdef USE_XPD_ADC_ERROR_DETECT
    its |= ADC_IER_JQOVFIE;
#endif
    SET_BIT(hadc->Inst->IER.w, its);

    XPD_ADC_Injected_Start(hadc);
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral and interrupts.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @note  In multimode configuration, the master ADC shall be stopped first.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Stop_IT(ADC_HandleType * hadc)
{
    XPD_ADC_Injected_Stop(hadc);

    XPD_ADC_DisableIT(hadc, JEOC);
}

/** @} */

/** @} */

#ifdef ADC12_COMMON
/** @addtogroup ADC_MultiMode
 * @{ */

/** @defgroup ADC_MultiMode_Exported_Functions Multi ADC Mode Exported Functions
 * @{ */

/**
 * @brief Initializes a multi ADC configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to the multi ADC setup configuration
 */
void XPD_ADC_MultiMode_Config(ADC_HandleType * hadc, const ADC_MultiMode_InitType * Config)
{
    __IO uint32_t * pCR = &ADC_PAIR(hadc)->CR.w;
    ADC_Common_TypeDef *common = ADC_COMMON(hadc);

    /* Neither ADC should be running regular conversion */
    if (    ((hadc->Inst->CR.w & ADC_CR_ADSTART) == 0)
         && ((            *pCR & ADC_CR_ADSTART) == 0))
    {
        common->CCR.b.MDMA = Config->DMAAccessMode;
        common->CCR.b.DMACFG = hadc->Inst->CFGR.b.DMACFG;

        /* Neither ADC should be on */
        if (    ((hadc->Inst->CR.w & ADC_CR_ADEN) == 0)
             && ((            *pCR & ADC_CR_ADEN) == 0))
        {
            common->CCR.b.DUAL  = Config->Mode;
            common->CCR.b.DELAY = Config->InterSamplingDelay - 1;
        }
    }
}

/**
 * @brief Sets up and enables a DMA transfer for the multi ADC regular conversions.
 * @param hadc: pointer to the ADC handle structure
 * @param Address: memory address to the conversion data storage
 * @return ERROR if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType XPD_ADC_MultiMode_Start_DMA(ADC_HandleType * hadc, void * Address)
{
    XPD_ReturnType result = XPD_ERROR;

    /* Perform ADC enable and conversion start if no conversion is on going
     * (check on ADC master only) */
    if ((ADC_REG_BIT(hadc,CR,ADSTART) == 0) && ((ADC_INDEX(hadc) & 1) == 0))
    {
        /* enable both ADCs */
        (void)adcx_enable(hadc->Inst);
        (void)adcx_enable(ADC_PAIR(hadc));

        /* Set up DMA for transfer */
        result = XPD_DMA_Start_IT(hadc->DMA.Conversion,
                (void *)&ADC_COMMON(hadc)->CDR.w, Address, hadc->Inst->SQR1.b.L + 1);

        /* If the DMA is currently used, return with error */
        if (result == XPD_OK)
        {
            /* Set the callback owner */
            hadc->DMA.Conversion->Owner = hadc;

            /* Set the DMA transfer callbacks */
            hadc->DMA.Conversion->Callbacks.Complete     = adc_dmaConversionRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
            hadc->DMA.Conversion->Callbacks.Error        = adc_dmaErrorRedirect;
#endif

#ifdef USE_XPD_ADC_ERROR_DETECT
            /* Enable ADC overrun interrupt */
            XPD_ADC_EnableIT(hadc, OVR);
#endif

            /* Enable the ADC peripherals: master and slave
             * (in case if not already enabled previously) */
            ADC_PAIR(hadc)->CR.b.ADEN = 1;

            /* Enable ADC DMA mode */
            ADC_REG_BIT(hadc, CFGR, DMAEN) = 1;

            XPD_ADC_Start(hadc);
        }
    }

    return result;
}

/**
 * @brief Disables the ADC and the common DMA transfer.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_MultiMode_Stop_DMA(ADC_HandleType * hadc)
{
    ADC_TypeDef *pair = ADC_PAIR(hadc);
    uint32_t timeout = ADC_STOP_CONVERSION_TIMEOUT;

    XPD_ADC_Stop_DMA(hadc);

    /* Wait for effective finish of slave conversion */
    if (XPD_OK == XPD_WaitForMatch(&pair->CR.w, ADC_CR_ADSTART, 0, &timeout))
    {
        /* disable pair */
        adcx_disable(pair);
    }
}

/** @} */

/** @} */
#endif

/** @addtogroup ADC_Calibration
 * @{ */

/** @defgroup ADC_Calibration_Exported_Functions ADC Calibration Exported Functions
 * @{ */

XPD_ReturnType XPD_ADC_Calibrate(ADC_HandleType * hadc, boolean_t Differential)
{
    XPD_ReturnType result = XPD_ERROR;

    /* Calibration prerequisite: ADC must be disabled. */
    if (ADC_REG_BIT(hadc, CR, ADEN) == 0)
    {
        uint32_t timeout = ADC_CALIBRATION_TIMEOUT;

        /* Select calibration mode single ended or differential ended */
        ADC_REG_BIT(hadc, CR, ADCALDIF) = Differential;

        /* Start ADC calibration */
        ADC_REG_BIT(hadc, CR, ADCAL) = 1;

        result = XPD_WaitForDiff(&hadc->Inst->CR.w,
                ADC_CR_ADCAL, ADC_CR_ADCAL, &timeout);
    }
    return result;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_ADC */
