/**
  ******************************************************************************
  * @file    xpd_adc.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-02-04
  * @brief   STM32 eXtensible Peripheral Drivers ADC Module
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

#define ADC_STARTCTRL          (ADC_CR_ADSTART | ADC_CR_JADSTART)
#define ADC_STOPCTRL           (ADC_CR_ADSTP | ADC_CR_JADSTP)
#define ADC_ENDFLAG_SEQUENCE   (ADC_IER_EOSIE | ADC_IER_JEOSIE)
#define ADC_ENDFLAG_CONVERSION (ADC_IER_EOCIE | ADC_IER_JEOCIE)

#if defined(ADC34_COMMON)

#define ADC_COUNT 4

#define ADC_INDEX(HANDLE)   \
    ((((((uint32_t)(HANDLE)->Inst) > ADC2_BASE) ? \
    (((uint32_t)(HANDLE)->Inst) - 0x200) : \
        ((uint32_t)(HANDLE)->Inst)) >> 8) & 3)

#define ADC_COMMON(HANDLE)  \
  ((((uint32_t)(HANDLE)->Inst) > ADC2_BASE) ? (ADC34_COMMON) : (ADC12_COMMON))

#define ADC_COMMON_REG_BIT(HANDLE, REG_NAME, BIT_NAME)  \
    ((ADC_COMMON(HANDLE))->REG_NAME.b.BIT_NAME)

#define ADC_PAIR(HANDLE) (((((uint32_t)(HANDLE)->Inst) & 0x100) != 0) ? \
        ((ADC_TypeDef *)(((uint32_t)(HANDLE)->Inst) & (~0x100))) : \
        ((ADC_TypeDef *)(((uint32_t)(HANDLE)->Inst) | 0x100)))

#define ADC_MULTI_CFGR(HANDLE, DUAL)    \
    (((DUAL) == ADC_MULTIMODE_SINGE) || (IS_ADC_MULTIMODE_MASTER_INSTANCE((HANDLE)->Inst))) ? \
        (HANDLE)->Inst->CFGR.w : ADC_PAIR(HANDLE)->CFGR.w)

#elif defined(ADC12_COMMON)

#define ADC_COUNT 2

#define ADC_INDEX(HANDLE)   \
    (((((uint32_t)(HANDLE)->Inst)) >> 8) & 3)

#define ADC_COMMON(HANDLE)  \
    (ADC12_COMMON)

#define ADC_COMMON_REG_BIT(HANDLE, REG_NAME, BIT_NAME)  \
    ((ADC_COMMON(HANDLE))->REG_NAME.b.BIT_NAME)

#define ADC_PAIR(HANDLE) (((((uint32_t)(HANDLE)->Inst) & 0x100) != 0) ? \
        ((ADC_TypeDef *)(((uint32_t)(HANDLE)->Inst) & (~0x100))) : \
        ((ADC_TypeDef *)(((uint32_t)(HANDLE)->Inst) | 0x100)))

#elif defined(ADC1_COMMON)

#define ADC_COUNT 1

#define ADC_INDEX(HANDLE)   \
    (0)

#define ADC_COMMON(HANDLE)  \
    (ADC1_COMMON)

#define ADC_COMMON_REG_BIT(HANDLE, REG_NAME, BIT_NAME)  \
    ((ADC_COMMON(HANDLE))->REG_NAME.b.BIT_NAME)

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

    if (ClockState == DISABLE)
    {
        CLEAR_BIT(adc_users[adcx / 2], 1 << adcx);
    }
    else
    {
        SET_BIT(adc_users[adcx / 2], 1 << adcx);
    }

    adc_clkCtrl[adcx / 2]((adc_users[adcx / 2] > 0) ? ENABLE : DISABLE);
}
#else
static void adc_clockCtrl(ADC_HandleType * hadc, FunctionalState ClockState)
{
    XPD_ADC1_ClockCtrl(ClockState);
}
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

    hadc->ActiveConversions = 0;

    XPD_SAFE_CALLBACK(hadc->Callbacks.ConvComplete, hadc);
}
static void adc_dmaHalfConversionRedirect(void *hdma)
{
    ADC_HandleType* hadc = (ADC_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    hadc->ActiveConversions = (hadc->Inst->SQR1.b.L + 1)/2;

    XPD_SAFE_CALLBACK(hadc->Callbacks.HalfConvComplete, hadc);
}
#ifdef USE_XPD_DMA_ERROR_DETECT
static void adc_dmaErrorRedirect(void *hdma)
{
    ADC_HandleType* hadc = (ADC_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    XPD_SAFE_CALLBACK(hadc->Callbacks.Error, hadc);
}
#endif

/* Enables the peripheral */
bool adcx_enable(ADC_TypeDef * ADCx)
{
    /* Requirements:
     * ADCAL, ADSTP, JADSTP, ADSTART, JADSTART, ADDIS, ADEN = 0 */
    bool success = (0 == (ADCx->CR.w & (ADC_CR_ADEN | ADC_STARTCTRL | ADC_STOPCTRL | ADC_CR_ADCAL | ADC_CR_ADDIS)));
    if (success)
    {
        SET_BIT(ADCx->CR.w, ADC_CR_ADEN);
    }
    return success;
}

/* Disables the peripheral */
bool adcx_disable(ADC_TypeDef * ADCx)
{
    /* Requirements:
     * ADEN = 1, ADSTART, JADSTART = 0 */
    bool success = (ADC_CR_ADEN == (ADCx->CR.w & (ADC_CR_ADEN | ADC_STARTCTRL)));
    if (success)
    {
        SET_BIT(ADCx->CR.w, ADC_CR_ADDIS);
        SET_BIT(ADCx->ISR.w, ADC_ISR_EOSMP | ADC_ISR_ADRDY);
    }
    return success;
}

static void adc_sequencerConfig(ADC_HandleType * hadc, uint8_t Channel, uint8_t Rank)
{
    uint32_t regoffset, rankm, choffset;

    /* channel configuration for a given rank */
    if (Rank < 5)
    {
        regoffset = 0;
        rankm = 0;
    }
    else if (Rank < 10)
    {
        regoffset = 1;
        rankm = 5;
    }
    else if (Rank < 15)
    {
        regoffset = 2;
        rankm = 10;
    }
    else
    {
        regoffset = 3;
        rankm = 15;
    }

    /* set the channel for the selected rank */
    choffset = (Rank - rankm) * 6;
    MODIFY_REG((&hadc->Inst->SQR1.w)[regoffset], ADC_SQR2_SQ5 << choffset, Channel << choffset);
}

static void adc_sampleTimeConfig(ADC_HandleType * hadc, uint8_t Channel, ADC_SampleTimeType SampleTime)
{
    uint32_t regoffset = 0;

    /* Channel sampling time configuration */
    if (Channel > 10)
    {
        regoffset = 1;
        Channel -= 10;
    }
    Channel *= 3;

    /* set the sample time */
    MODIFY_REG((&hadc->Inst->SMPR1.w)[regoffset], ADC_SMPR1_SMP0  << Channel, SampleTime << Channel);
}

static void adc_initInternalChannel(ADC_HandleType * hadc, uint32_t Channel)
{
    uint32_t chbit = 0;

    switch (Channel)
    {
        case ADC_VBAT_CHANNEL:
            /* enable the VBAT input */
            if (hadc->Inst == ADC1)
            { chbit = ADC_CCR_VBATEN; }
            break;

        case ADC_VREFINT_CHANNEL:
            /* enable the VREF input */
            { chbit = ADC_CCR_VREFEN; }
            break;

        case ADC_TEMPSENSOR_CHANNEL:
            /* enable the TS input */
            if (hadc->Inst == ADC1)
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
#if ADC_COUNT > 1
         && (ADC_PAIR(hadc)->CR.b.ADEN == 0)
#endif
             )
    {
        SET_BIT(ADC_COMMON(hadc)->CCR.w, chbit);

        if (Channel == ADC_TEMPSENSOR_CHANNEL)
        {
            /* wait temperature sensor stabilization */
            XPD_Delay_us(ADC_TEMPSENSOR_DELAY_US);
        }
    }
}

static void adc_stopConversion(ADC_HandleType * hadc, uint32_t convFlag)
{
    uint32_t timeout = ADC_STOP_CONVERSION_TIMEOUT;

    /* Verification if ADC is not already stopped (on regular and injected      *
     * groups) to bypass this function if not needed.                           */
    if ((hadc->Inst->CR.w & ADC_STARTCTRL) != 0)
    {
        /* Particular case of continuous auto-injection mode combined with        *
         * auto-delay mode.                                                       *
         * In auto-injection mode, regular group stop ADC_CR_ADSTP is used (not   *
         * injected group stop ADC_CR_JADSTP).                                    *
         * Procedure to be followed: Wait until JEOS=1, clear JEOS, set ADSTP=1   *
         * (see reference manual).                                                */
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

/** @addtogroup ADC_Core_Exported_Functions
 * @{ */

/**
 * @brief Initializes the ADC peripheral using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to ADC setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_ADC_Init(ADC_HandleType * hadc, ADC_InitType * Config)
{
    XPD_ReturnType result = XPD_ERROR;

    /* enable clock */
    adc_clockCtrl(hadc, ENABLE);

#ifdef ADC_BB
    hadc->Inst_BB = ADC_BB(hadc->Inst);
#endif

    XPD_SAFE_CALLBACK(hadc->Callbacks.DepInit, hadc);

    /* Initialize ADC API internal variables */
    hadc->InjectedSetup.ChannelCount = 0;
    hadc->InjectedSetup.ContextQueue = 0;

    hadc->ActiveConversions = 0;

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
        /* Note: The software must wait for the startup time of the ADC       *
         *       voltage regulator before launching a calibration or          *
         *       enabling the ADC. This temporization must be implemented by  *
         *       software and is equal to 10 us in the worst case             *
         *       process/temperature/power supply.                            */

        /* Disable the ADC (if not already disabled) */
        (void)adcx_disable(hadc->Inst);

        /* Check if ADC is effectively disabled *
         * Configuration of ADC parameters if previous preliminary actions    *
        /* are correctly completed.                                           */
        if (ADC_REG_BIT(hadc, CR, ADEN) == 0)
        {
            /* Set the intermediate state before moving the ADC voltage         *
             * regulator to state enable.                                       */
            hadc->Inst->CR.b.ADVREGEN = 0;
            /* Set ADVREGEN bits to 0x01 */
            hadc->Inst->CR.b.ADVREGEN = 1;

            /* Delay for ADC stabilization time.                                */
            XPD_Delay_us(ADC_STAB_DELAY_US);
        }
    }

    /* Configuration of ADC parameters if previous preliminary actions are      *
     * correctly completed and if there is no conversion on going on regular    *
     * group (ADC may already be enabled at this point if HAL_ADC_Init() is     *
     * called to update a parameter on the fly).                                */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        /* Configuration of ADC:                   *
         *  - resolution                           *
         *  - data alignment                       *
         *  - external trigger to start conversion *
         *  - external trigger polarity            *
         *  - continuous conversion mode           *
         *  - overrun                              *
         *  - discontinuous mode                   */
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

        /* Parameters update conditioned to ADC state:                            *
         * Parameters that can be updated when ADC is disabled or enabled without *
         * conversion on going on regular and injected groups:                    *
         *  - DMA continuous request                                              *
         *  - LowPowerAutoWait feature                                            */
        if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
        {
            ADC_REG_BIT(hadc, CFGR, AUTDLY) = Config->LPAutoWait;
            ADC_REG_BIT(hadc, CFGR, DMACFG) = Config->ContinuousDMARequests;
        }

        /* Configuration of regular group sequencer:                              *
         * - if scan mode is disabled, regular channels sequence length is set to *
         *   0x00: 1 channel converted (channel on regular rank 1)                *
         *   Parameter "NbrOfConversion" is discarded.                            *
         *   Note: Scan mode is not present by hardware on this device, but       *
         *   emulated by software for alignment over all STM32 devices.           *
         * - if scan mode is enabled, regular channels sequence length is set to  *
         *   parameter "NbrOfConversion"                                          */
        if (Config->ScanMode == ENABLE)
        {
            /* Set number of ranks in regular group sequencer */
            hadc->Inst->SQR1.b.L = Config->ConversionCount - 1;
        }
        else
        {
            hadc->Inst->SQR1.b.L = 0;
        }

        result = XPD_OK;
    }

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

    /* Disable ADC peripheral if conversions are effectively stopped */
    /* Flush register JSQR: queue sequencer reset when injected queue         */
    /* sequencer is enabled and ADC disabled.                                 */
    /* Enable injected queue sequencer after injected conversion stop         */
    ADC_REG_BIT(hadc, CFGR, JQM) = 1;

    /* Disable the ADC peripheral */
    adcx_disable(hadc->Inst);

    /* Reset register IER */
    hadc->Inst->IER.w = 0;

    /* Reset register ISR */
    hadc->Inst->ISR.w = 0xFFFFFFFF;

    /* Reset OFR registers */
    hadc->Inst->OFR1.w = 0;
    hadc->Inst->OFR2.w = 0;
    hadc->Inst->OFR3.w = 0;
    hadc->Inst->OFR4.w = 0;

    /* Reset register CR */
    /* Bits ADC_CR_JADSTP, ADC_CR_ADSTP, ADC_CR_JADSTART, ADC_CR_ADSTART are  */
    /* in access mode "read-set": no direct reset applicable.                 */
    /* Reset Calibration mode to default setting (single ended):              */
    /* Disable voltage regulator:                                             */
    /* Note: Voltage regulator disable is conditioned to ADC state disabled:  */
    /*       already done above.                                              */
    /* Note: Voltage regulator disable is intended for power saving.          */
    /* Sequence to disable voltage regulator:                                 */
    /* 1. Set the intermediate state before moving the ADC voltage regulator  */
    /*    to disable state.                                                   */
    CLEAR_BIT(hadc->Inst->CR.w, ADC_CR_ADVREGEN | ADC_CR_ADCALDIF);
    /* 2. Set ADVREGEN bits to 0x10 */
    SET_BIT(hadc->Inst->CR.w, ADC_CR_ADVREGEN_1);

    /* Reset register CFGR */
    hadc->Inst->CFGR.w = 0;

    /* disable clock */
    adc_clockCtrl(hadc, DISABLE);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(hadc->Callbacks.DepDeinit, hadc);

    return XPD_OK;
}

/**
 * @brief Initializes a regular ADC channel for conversion using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to ADC regular channel setup configuration
 */
void XPD_ADC_ChannelConfig(ADC_HandleType * hadc, ADC_ChannelInitType * Config)
{
    /* Parameters update conditioned to ADC state:                              */
    /* Parameters that can be updated when ADC is disabled or enabled without   */
    /* conversion on going on regular group:                                    */
    /*  - Channel number                                                        */
    /*  - Channel rank                                                          */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        /* sequencer rank configuration */
        adc_sequencerConfig(hadc, Config->Channel, Config->Rank);

        /* Parameters update conditioned to ADC state:                              */
        /* Parameters that can be updated when ADC is disabled or enabled without   */
        /* conversion on going on regular group:                                    */
        /*  - Channel sampling time                                                 */
        /*  - Channel offset                                                        */
        if (ADC_REG_BIT(hadc, CR, JADSTART) == 0)
        {
            /* Channel sampling time configuration */
            adc_sampleTimeConfig(hadc, Config->Channel, Config->SampleTime);
        }

        /* Parameters update conditioned to ADC state:                              */
        /* Parameters that can be updated only when ADC is disabled:                */
        /*  - Single or differential mode                                           */
        /*  - Internal measurement channels: Vbat/VrefInt/TempSensor                */
        if (hadc->Inst->CR.b.ADEN == 0)
        {
            /* Management of internal measurement channels: VrefInt/TempSensor/Vbat   */
            /* internal measurement paths enable: If internal channel selected,       */
            /* enable dedicated internal buffers and path.                            */
            /* Note: these internal measurement paths can be disabled using           */
            /* HAL_ADC_DeInit().                                                      */
            adc_initInternalChannel(hadc, Config->Channel);

            /* Configuration of differential mode */
            if (Config->Differential != DISABLE)
            {
                /* Enable differential mode */
                SET_BIT(hadc->Inst->DIFSEL.w, 1 << Config->Channel);

                /* Channel sampling time configuration (channel ADC_INx +1              */
                /* corresponding to differential negative input).                       */
                adc_sampleTimeConfig(hadc, Config->Channel + 1, Config->SampleTime);
            }
            else
            {
                /* Disable differential mode (default mode: single-ended) */
                CLEAR_BIT(hadc->Inst->DIFSEL.w, 1 << Config->Channel);
            }
        }
    }
}

/**
 * @brief Sets up or removes an offset configuration for an ADC channel.
 * @param hadc: pointer to the ADC handle structure
 * @param Channel: the selected ADC channel
 * @param Offset: the offset value to set up (or clear if equals 0)
 */
XPD_ReturnType XPD_ADC_ChannelOffsetConfig(ADC_HandleType * hadc, uint32_t Channel, uint16_t Offset)
{
    XPD_ReturnType result = XPD_ERROR;
    union {
        struct {
            __IO uint32_t OFFSET : 12;
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET_CH : 5;
            __IO uint32_t OFFSET_EN : 1;
        } b;
        __IO uint32_t w;
    } * pOFR = &hadc->Inst->OFR1.w;

    if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        uint8_t i;

        /* If offset is 0, remove channel from configuration */
        if (Offset == 0)
        {
            /* Find the channel in the offsets, clear config */
            for (i = 0; i < 4; i++)
            {
                if (pOFR->b.OFFSET_CH == Channel)
                {
                    pOFR->w = 0;
                    result = XPD_OK;
                    break;
                }
                pOFR++;
            }
        }
        /* Add channel offset */
        else
        {
            /* Find an empty offset register, set it up */
            for (i = 0; i < 4; i++)
            {
                if (pOFR->w == 0)
                {
                    pOFR->b.OFFSET = Offset << (hadc->Inst->CFGR.b.RES * 2);
                    pOFR->b.OFFSET_CH = Channel;
                    pOFR->b.OFFSET_EN = 1;
                    result = XPD_OK;
                    break;
                }
                pOFR++;
            }
        }
    }

    return result;
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

        /* initialize number of remaining conversions count */
        hadc->ActiveConversions = hadc->Inst->SQR1.b.L + 1;

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

    (void)adcx_disable(hadc->Inst);

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
 * @brief Enables the ADC peripheral and interrupts, and starts conversion if the trigger is software.
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

            /* Initialize number of remaining conversions count */
            if (hadc->ActiveConversions == 0)
            {
                hadc->ActiveConversions = hadc->Inst->SQR1.b.L + 1;
            }

            /* Decrement the number of conversion when an interrupt occurs */
            hadc->ActiveConversions--;
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
        uint32_t cfgr = adc_getMultiCfgr(&hadc, dual);

        /* Injected conversion is not continuous or not automatic, and software triggered */
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

                /* Particular case if injected contexts queue is enabled:             *
                 * when the last context has been fully processed, JSQR is reset      *
                 * by the hardware. Even if no injected conversion is planned to come *
                 * (queue empty, triggers are ignored), it can start again            *
                 * immediately after setting a new context (JADSTART is still set).   *
                /* Therefore, state of HAL ADC injected group is kept to busy.        */
                if ((cfgr & ADC_CFGR_JQM) == 0)
                {
                    if (ADC_REG_BIT(hadc, CR, JADSTART) == 0)
                    {
                        /* disable the ADC end of conversion interrupt for injected group */
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
        hadc->ActiveWatchdog = 1;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Watchdog, hadc);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        XPD_ADC_ClearFlag(hadc, AWD1);
    }
    if (    ((isr & ADC_ISR_AWD2) != 0)
         && ((ier & ADC_IER_AWD2IE) != 0))
    {
        hadc->ActiveWatchdog = 2;

        /* watchdog callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Watchdog, hadc);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        XPD_ADC_ClearFlag(hadc, AWD2);
    }
    if (    ((isr & ADC_ISR_AWD3) != 0)
         && ((ier & ADC_IER_AWD3IE) != 0))
    {
        hadc->ActiveWatchdog = 3;

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
        hadc->Errors |= ADC_ERROR_OVR;

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
        result = XPD_DMA_Start_IT(hadc->DMA.Conversion, (void *)&hadc->Inst->DR, Address, hadc->Inst->SQR1.b.L + 1);

        /* If the DMA is currently used, return with error */
        if (result == XPD_OK)
        {
            /* Set the callback owner */
            hadc->DMA.Conversion->Owner = hadc;

            /* Set the DMA transfer callbacks */
            hadc->DMA.Conversion->Callbacks.Complete     = adc_dmaConversionRedirect;
            hadc->DMA.Conversion->Callbacks.HalfComplete = adc_dmaHalfConversionRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
            hadc->DMA.Conversion->Callbacks.Error        = adc_dmaErrorRedirect;

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
 * @param Channel: the ADC channel to monitor (only used when single channel monitoring is configured)
 * @param Config: pointer to analog watchdog setup configuration
 */
void XPD_ADC_WatchdogConfig(ADC_HandleType * hadc, uint8_t Channel, ADC_WatchdogInitType * Config)
{
    /* Parameters update conditioned to ADC state:                              *
     * Parameters that can be updated when ADC is disabled or enabled without   *
     * conversion on going on regular and injected groups:                      *
     *  - Analog watchdog channels                                              *
     *  - Analog watchdog thresholds                                            */
    if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        uint32_t scaling = hadc->Inst->CFGR.b.RES * 2;

        /* Analog watchdogs configuration */
        if (Config->Number == 1)
        {
            /* Configuration of analog watchdog:                                    *
             *  - Set the analog watchdog enable mode: regular and or injected      *
             *    groups, one or overall group of channels.                         *
             *  - Set the Analog watchdog channel (is not used if watchdog          *
             *    mode "all channels": ADC_CFGR_AWD1SGL=0).                         */
            MODIFY_REG(hadc->Inst->CFGR.w, (ADC_CFGR_AWD1SGL | ADC_CFGR_JAWD1EN | ADC_CFGR_AWD1EN), Config->Mode);
            hadc->Inst->CFGR.b.AWD1CH = Channel;

            /* Shift the offset in function of the selected ADC resolution:         *
             * Thresholds have to be left-aligned on bit 11, the LSB (right bits)   *
             * are set to 0                                                         */
            hadc->Inst->TR1.w = (Config->Threshold.High  << (16 + scaling)) | (Config->Threshold.Low  << scaling);

            /* Clear the ADC Analog watchdog flag (in case of left enabled by       *
             * previous ADC operations) to be ready to use for HAL_ADC_IRQHandler() *
             * or HAL_ADC_PollForEvent().                                           */
            XPD_ADC_ClearFlag(hadc, AWD1);

            /* Configure ADC Analog watchdog interrupt */
            XPD_ADC_EnableIT(hadc, AWD1);
        }
        /* Case of ADC_ANALOGWATCHDOG_2 and ADC_ANALOGWATCHDOG_3 */
        else
        {
            uint32_t thr;
            /* Shift the threshold in function of the selected ADC resolution *
             * have to be left-aligned on bit 7, the LSB (right bits) are set to 0    */
            if (4 < scaling)
            {
                thr = (Config->Threshold.High  << (20 - scaling)) | (Config->Threshold.Low  >> (scaling - 4));
            }
            else
            {
                thr = (Config->Threshold.High  << (20 - scaling)) | (Config->Threshold.Low  << (4 - scaling));
            }

            if (Config->Number == 2)
            {
                /* Set the Analog watchdog channel or group of channels. This also    *
                 * enables the watchdog.                                              *
                 * Note: Conditionnal register reset, because several channels can be *
                 *       set by successive calls of this function.                    */
                if (Config->Mode != ADC_WATCHDOG_NONE)
                {
                    /* Set the high and low thresholds */
                    hadc->Inst->TR2.w = thr;

                    if ((Config->Mode & ADC_CFGR_AWD1SGL) == 0)
                    {
                        hadc->Inst->AWD2CR.w = ADC_AWD2CR_AWD2CH;
                    }
                    else
                    {
                        SET_BIT(hadc->Inst->AWD2CR.w, 1 << Channel);
                    }
                }
                else
                {
                    hadc->Inst->TR2.w = 0;
                    hadc->Inst->AWD2CR.w = 0;
                }

                /* Clear the ADC Analog watchdog flag (in case of left enabled by       *
                 * previous ADC operations) to be ready to use for HAL_ADC_IRQHandler() *
                 * or HAL_ADC_PollForEvent().                                           */
                XPD_ADC_ClearFlag(hadc, AWD2);

                /* Configure ADC Analog watchdog interrupt */
                XPD_ADC_EnableIT(hadc, AWD2);
            }
            /* (AnalogWDGConfig->WatchdogNumber == ADC_ANALOGWATCHDOG_3) */
            else
            {
                /* Set the Analog watchdog channel or group of channels. This also    *
                 * enables the watchdog.                                              *
                 * Note: Conditionnal register reset, because several channels can be *
                 *       set by successive calls of this function.                    */
                if (Config->Mode != ADC_WATCHDOG_NONE)
                {
                    /* Set the high and low thresholds */
                    hadc->Inst->TR3.w = thr;

                    if ((Config->Mode & ADC_CFGR_AWD1SGL) == 0)
                    {
                        hadc->Inst->AWD3CR.w = ADC_AWD3CR_AWD3CH;
                    }
                    else
                    {
                        SET_BIT(hadc->Inst->AWD3CR.w, 1 << Channel);
                    }
                }
                else
                {
                    hadc->Inst->TR3.w = 0;
                    hadc->Inst->AWD3CR.w = 0;
                }

                /* Clear the ADC Analog watchdog flag (in case of left enabled by       *
                 * previous ADC operations) to be ready to use for HAL_ADC_IRQHandler() *
                 * or HAL_ADC_PollForEvent().                                           */
                XPD_ADC_ClearFlag(hadc, AWD3);

                /* Configure ADC Analog watchdog interrupt */
                XPD_ADC_EnableIT(hadc, AWD3);
            }
        }
    }
}

/**
 * @brief Returns the currently active analog watchdog.
 * @param hadc: pointer to the ADC handle structure
 * @return The watchdog number if an active watchdog triggered an interrupt, 0 otherwise
 */
uint8_t XPD_ADC_WatchdogStatus(ADC_HandleType * hadc)
{
    /* Update the active watchdog variable with current flag status */
    hadc->ActiveWatchdog *= hadc->Inst->ISR.w >> (hadc->ActiveWatchdog + ADC_ISR_AWD1_Pos - 1);
    return hadc->ActiveWatchdog;
}

/**
 * @brief Return the result of the last ADC regular conversion.
 * @param hadc: pointer to the ADC handle structure
 * @return The conversion result
 */
uint16_t XPD_ADC_GetValue(ADC_HandleType * hadc)
{
    return (uint16_t)hadc->Inst->DR;
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
void XPD_ADC_Injected_Init(ADC_HandleType * hadc, ADC_Injected_InitType * Config)
{
    /* Scan mode disabled, or injected conversion is single */
    if ((hadc->Inst->SQR1.b.L == 0) || (Config->ConversionCount == 1))
    {
        uint32_t jsqr = 0;

        /* Only channel is used as Rank 1 */
        jsqr = (Config->Channel << ((1 * 6) + 2));

        if (Config->Trigger.InjSource != ADC_INJTRIGGER_SOFTWARE)
        {
            hadc->Inst->JSQR.b.JEXTEN  = Config->Trigger.Edge;
            hadc->Inst->JSQR.b.JEXTSEL = Config->Trigger.InjSource;
            MODIFY_REG(hadc->Inst->JSQR.w,
                    ADC_JSQR_JEXTEN | ADC_JSQR_JEXTSEL | ADC_JSQR_JSQ1,
                    (Config->Trigger.InjSource << 2) | (Config->Trigger.Edge << 4) | jsqr);
        }
    }
    else
    {
        /* Case of scan mode enabled, several channels to set into injected group *
         * sequencer.                                                             *
         * Procedure to define injected context register JSQR over successive     *
         * calls of this function, for each injected channel rank:                */

        /* 1. Start new context and set parameters related to all injected        *
         *    channels: injected sequence length and trigger                      */
        if (hadc->InjectedSetup.ChannelCount == 0)
        {
            /* Initialize number of channels that will be configured on the context *
             *  being built                                                         */
            hadc->InjectedSetup.ChannelCount = Config->ConversionCount - 1;

            /* Initialize value that will be set into register JSQR */
            hadc->InjectedSetup.ContextQueue = Config->ConversionCount - 1;

            /* Configuration of context register JSQR:                              *
             *  - number of ranks in injected group sequencer                       *
             *  - external trigger to start conversion                              *
             *  - external trigger polarity                                         */

            /* Enable external trigger if trigger selection is different of         *
             * software start.                                                      *
             * Note: This configuration keeps the hardware feature of parameter     *
             *       ExternalTrigInjecConvEdge "trigger edge none" equivalent to    *
             *       software start.                                                */
            if (Config->Trigger.InjSource != ADC_INJTRIGGER_SOFTWARE)
            {
                MODIFY_REG(hadc->InjectedSetup.ContextQueue,
                        ADC_JSQR_JEXTEN | ADC_JSQR_JEXTSEL,
                        (Config->Trigger.InjSource << 2) | (Config->Trigger.Edge << 4));
            }
        }

        /* 2. Continue setting of context under definition with parameter       *
         *    related to each channel: channel rank sequence                    */
        {
            uint32_t channelOffset = (Config->Rank * 6) + 2;
            uint32_t channelMask   = ADC_SQR3_SQ10 << channelOffset;
            uint32_t channelVal    = Config->Channel << channelOffset;

            /* Set the JSQx bits for the selected rank */
            MODIFY_REG(hadc->InjectedSetup.ContextQueue, channelMask, channelVal);
        }

        /* Decrease channel count after setting into temporary JSQR variable */
        hadc->InjectedSetup.ChannelCount--;

        /* 3. End of context setting: If last channel set, then write context
         *    into register JSQR and make it enter into queue */
        if (hadc->InjectedSetup.ChannelCount == 0)
        {
            hadc->Inst->JSQR.w = hadc->InjectedSetup.ContextQueue;
        }
    }

    /* Parameters update conditioned to ADC state:                              *
     * Parameters that can be updated when ADC is disabled or enabled without   *
     * conversion on going on injected group:                                   *
     *  - Injected context queue: Queue disable (active context is kept) or     *
     *    enable (context decremented, up to 2 contexts queued)                 *
     *  - Injected discontinuous mode: can be enabled only if auto-injected     *
     *    mode is disabled.                                                     */
    if (ADC_REG_BIT(hadc, CR, JADSTART) == 0)
    {
        hadc->Inst->CFGR.b.JQM     = Config->ContextQueue;
        hadc->Inst->CFGR.b.JDISCEN = Config->DiscontinuousMode & (!Config->AutoInjection);
    }

    /* Parameters update conditioned to ADC state:                              *
     * Parameters that can be updated when ADC is disabled or enabled without   *
     * conversion on going on regular and injected groups:                      *
     *  - Automatic injected conversion: can be enabled if injected group       *
     *    external triggers are disabled.                                       *
     *  - Channel sampling time                                                 *
     *  - Channel offset                                                        */
    if ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0)
    {
        /* If injected group external triggers are disabled (set to injected      *
         * software start): no constraint                                         */
        if (Config->Trigger.InjSource == ADC_INJTRIGGER_SOFTWARE)
        {
            hadc->Inst->CFGR.b.JAUTO = Config->AutoInjection;
        }
        /* If Automatic injected conversion was intended to be set and could not  *
         * due to injected group external triggers enabled, error is reported.    */
        else
        {
            hadc->Inst->CFGR.b.JAUTO = 0;
        }

        /* Channel sampling time configuration */
        adc_sampleTimeConfig(hadc, Config->Channel, Config->SampleTime);

        /* Parameters update conditioned to ADC state:                              *
         * Parameters that can be updated only when ADC is disabled:                *
         *  - Single or differential mode                                           *
         *  - Internal measurement channels: Vbat/VrefInt/TempSensor                */
        if (ADC_REG_BIT(hadc, CR, ADEN) == 0)
        {
            /* Configuration of differential mode */
            if (Config->Differential == DISABLE)
            {
                /* Disable differential mode (default mode: single-ended) */
                CLEAR_BIT(hadc->Inst->DIFSEL.w, 1 << Config->Channel);
            }
            else
            {
                /* Enable differential mode */
                SET_BIT(hadc->Inst->DIFSEL.w, 1 << Config->Channel);

                /* Channel sampling time configuration (channel ADC_INx +1              *
                 * corresponding to differential negative input).                       */
                adc_sampleTimeConfig(hadc, Config->Channel + 1, Config->SampleTime);
            }

            /* Management of internal measurement channels: VrefInt/TempSensor/Vbat   *
             * internal measurement paths enable: If internal channel selected,       *
             * enable dedicated internal buffers and path.                            *
             * Note: these internal measurement paths can be disabled using           *
             * HAL_ADC_deInit().                                                      */
            adc_initInternalChannel(hadc, Config->Channel);
        }
    }
}

/**
 * @brief Enables the ADC peripheral and starts injected conversion if the trigger is software.
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

    /* if no external trigger present and ADC is either not in multimode, or the multimode master,
     * enable software conversion of injected channels */
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
 * @brief Enables the ADC peripheral and interrupts, and starts injected conversion if the trigger is software.
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
 * @brief Return the result of an ADC injected conversion.
 * @param hadc: pointer to the ADC handle structure
 * @param Rank: injected rank of ADC conversion
 * @return The conversion result
 */
uint16_t XPD_ADC_Injected_GetValue(ADC_HandleType * hadc, uint8_t Rank)
{
    /* clear the flag for injected end of conversion */
    XPD_ADC_ClearFlag(hadc, JEOC);

    return (uint16_t)((&hadc->Inst->JDR1)[Rank - 1]);
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
void XPD_ADC_MultiMode_Init(ADC_HandleType * hadc, ADC_MultiMode_InitType * Config)
{
    __IO uint32_t * pCR = &ADC_PAIR(hadc)->CR.w;
    ADC_Common_TypeDef *common = ADC_COMMON(hadc);

    /* Parameters update conditioned to ADC state:                              *
     * Parameters that can be updated when ADC is disabled or enabled without   *
     * conversion on going on regular group:                                    *
     *  - Multimode DMA configuration                                           *
     *  - Multimode DMA mode                                                    */
    if (    ((hadc->Inst->CR.w & ADC_CR_ADSTART) == 0)
         && ((            *pCR & ADC_CR_ADSTART) == 0))
    {
        /* Pointer to the common control register to which is belonging hadc      *
         * (Depending on STM32F3 product, there may have up to 4 ADC and 2 common *
         * control registers)                                                     */

        /* Configuration of ADC common group ADC1&ADC2, ADC3&ADC4 if available    *
         * (ADC2, ADC3, ADC4 availability depends on STM32 product)               *
         *  - DMA access mode                                                     */
        common->CCR.b.MDMA = Config->DMAAccessMode;
        common->CCR.b.DMACFG = hadc->Inst->CFGR.b.DMACFG;

        /* Parameters that can be updated only when ADC is disabled:              *
         *  - Multimode mode selection                                            *
         *  - Multimode delay                                                     *
         * Note: If ADC is not in the appropriate state to modify these           *
         *       parameters, their setting is bypassed without error reporting    *
         *       (as it can be the expected behaviour in case of intended action  *
         *       to update parameter above (which fulfills the ADC state          *
         *       condition: no conversion on going on group regular)              *
         *       on the fly).                                                     */
        if (    ((hadc->Inst->CR.w & ADC_CR_ADEN) == 0)
             && ((            *pCR & ADC_CR_ADEN) == 0))
        {
            /* Configuration of ADC common group ADC1&ADC2, ADC3&ADC4 if available    *
             * (ADC2, ADC3, ADC4 availability depends on STM32 product)               *
             *  - set the selected multimode                                          *
             *  - DMA access mode                                                     *
             *  - Set delay between two sampling phases                               *
             *    Note: Delay range depends on selected resolution:                   *
             *      from 1 to 12 clock cycles for 12 bits                             *
             *      from 1 to 10 clock cycles for 10 bits,                            *
             *      from 1 to 8 clock cycles for 8 bits                               *
             *      from 1 to 6 clock cycles for 6 bits                               *
             *    If a higher delay is selected, it will be clamped to maximum delay  *
             *    range                                                               */
            common->CCR.b.DUAL  = Config->Mode;
            common->CCR.b.DELAY = Config->InterSamplingDelay;
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

    /* Perform ADC enable and conversion start if no conversion is on going (check on ADC master only) */
    if ((ADC_REG_BIT(hadc,CR,ADSTART) == 0) && ((ADC_INDEX(hadc) & 1) == 0))
    {
        /* enable both ADCs */
        (void)adcx_enable(hadc->Inst);
        (void)adcx_enable(ADC_PAIR(hadc));

        /* Set up DMA for transfer */
        result = XPD_DMA_Start_IT(hadc->DMA.Conversion, (void *)&ADC_COMMON(hadc)->CDR.w, Address, hadc->Inst->SQR1.b.L + 1);

        /* If the DMA is currently used, return with error */
        if (result == XPD_OK)
        {
            /* Set the callback owner */
            hadc->DMA.Conversion->Owner = hadc;

            /* Set the DMA transfer callbacks */
            hadc->DMA.Conversion->Callbacks.Complete     = adc_dmaConversionRedirect;
            hadc->DMA.Conversion->Callbacks.HalfComplete = adc_dmaHalfConversionRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
            hadc->DMA.Conversion->Callbacks.Error        = adc_dmaErrorRedirect;

            /* Enable ADC overrun interrupt */
            XPD_ADC_EnableIT(hadc, OVR);
#endif

            /* Enable the ADC peripherals: master and slave (in case if not already enabled previously) */
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
        (void)adcx_disable(pair);
    }
}

/**
 * @brief Return the result of the last common ADC regular conversions.
 * @return A pair of conversion results in a single word
 */
uint32_t XPD_ADC_MultiMode_GetValues(ADC_HandleType * hadc)
{
    /* return the multi mode conversion values */
    return ADC_COMMON(hadc)->CDR.w;
}

/** @} */

/** @} */
#endif

/** @addtogroup ADC_Calibration
 * @{ */

/** @defgroup ADC_Calibration_Exported_Functions ADC Calibration Exported Functions
 * @{ */

XPD_ReturnType XPD_ADC_Calibrate(ADC_HandleType * hadc, bool Differential)
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

        result = XPD_WaitForDiff(&hadc->Inst->CR.w, ADC_CR_ADCAL, ADC_CR_ADCAL, &timeout);
    }
    return result;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_ADC */
