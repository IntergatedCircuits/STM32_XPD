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

#define ADC_STAB_DELAY_US           1
#define ADC_TEMPSENSOR_DELAY_US     10
#define ADC_CALIBRATION_TIMEOUT     2
#define ADC_STOP_CONVERSION_TIMEOUT 2
#define ADC_ENABLE_TIMEOUT          2
#define ADC_DISABLE_TIMEOUT         2

#define ADC_STARTCTRL           (ADC_CR_ADSTART)
#define ADC_STOPCTRL            (ADC_CR_ADSTP)
#define ADC_ENDFLAG_SEQUENCE    (ADC_IER_EOSIE)
#define ADC_ENDFLAG_CONVERSION  (ADC_IER_EOCIE)

#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
#define ADC_RESET_ERRORS(HANDLE)    ((HANDLE)->Errors = ADC_ERROR_NONE)
#else
#define ADC_RESET_ERRORS(HANDLE)    ((void)(HANDLE))
#endif

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

/* Enables an internal channel for conversion */
static void ADC_prvInitInternalChannel(ADC_HandleType * pxADC, uint8_t ucChannel)
{
    uint32_t ulChBit = 0;

    /* Map channel to enable bit */
    switch (ucChannel)
    {
#ifdef ADC1_VBAT_CHANNEL
        case ADC1_VBAT_CHANNEL:
            /* enable the VBAT input */
            { ulChBit = ADC_CCR_VBATEN; }
            break;
#endif
        case ADC1_VREFINT_CHANNEL:
            /* enable the VREF input */
            { ulChBit = ADC_CCR_VREFEN; }
            break;

        case ADC1_TEMPSENSOR_CHANNEL:
            /* enable the TS input */
            { ulChBit = ADC_CCR_TSEN; }
            break;

        default:
            break;
    }

    /* Check if setting is valid */
    if ((ulChBit != 0) && ((ADC->CCR.w & ulChBit) == 0))
    {
        SET_BIT(ADC->CCR.w, ulChBit);

        if (ucChannel == ADC1_TEMPSENSOR_CHANNEL)
        {
            /* wait temperature sensor stabilization */
            XPD_vDelay_us(ADC_TEMPSENSOR_DELAY_US);
        }
    }
}

/* Stops ongoing conversions */
static void ADC_prvStopConversion(ADC_HandleType * pxADC, uint32_t ulConvFlag)
{
    uint32_t ulTimeout = ADC_STOP_CONVERSION_TIMEOUT;

    /* Verification if ADC is not already stopped (on regular and injected
     * groups) to bypass this function if not needed. */
    if ((pxADC->Inst->CR.w & ADC_STARTCTRL) != 0)
    {
        {
            /* Software is allowed to set ADSTP only when ADSTART=1 and ADDIS=0 */
            if ((pxADC->Inst->CR.w & (ADC_CR_ADSTART | ADC_CR_ADDIS)) == ADC_CR_ADSTART)
            {
                /* Stop conversions on regular group */
                ADC_REG_BIT(pxADC, CR, ADSTP) = 1;
            }
        }

        /* Wait for conversion effectively stopped */
        XPD_eWaitForMatch(&pxADC->Inst->CR.w, ulConvFlag, 0, &ulTimeout);
    }
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
    RCC_vClockEnable(RCC_POS_ADC1);

    /* Set used EndFlag */
    if (pxConfig->EndFlagSelection == ADC_EOC_SEQUENCE)
    {
        pxADC->EndFlagSelection = ADC_ENDFLAG_SEQUENCE;
    }
    else
    {
        pxADC->EndFlagSelection = ADC_ENDFLAG_CONVERSION;
    }

    /* Configuration of ADC parameters if previous preliminary actions are
     * correctly completed and if there is no conversion ongoing on regular group */
    if (ADC_REG_BIT(pxADC, CR, ADSTART) == 0)
    {
        uint32_t ulCfgr, ulCfgrMask;

        ulCfgr = pxConfig->w;

        ulCfgrMask = ADC_CFGR1_CONT | ADC_CFGR1_ALIGN | ADC_CFGR1_RES |
                ADC_CFGR1_EXTEN | ADC_CFGR1_EXTSEL | ADC_CFGR1_DISCEN |
                ADC_CFGR1_WAIT | ADC_CFGR1_AUTOFF | ADC_CFGR1_DMACFG | ADC_CFGR1_SCANDIR;

        /* Enable discontinuous mode only if continuous mode is disabled */
        if (pxConfig->ContinuousMode == DISABLE)
        {
            ulCfgr &= ~ADC_CFGR1_DISCEN;
        }

        MODIFY_REG(pxADC->Inst->CFGR1.w, ulCfgrMask, ulCfgr);
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

    /* Disable the ADC peripheral */
    ADC_prvDisableInst(pxADC->Inst);

    /* Reset register IER */
    pxADC->Inst->IER.w = 0;

    /* Reset register ISR */
    pxADC->Inst->ISR.w = ~0;

    /* Reset register CFGR1 */
    pxADC->Inst->CFGR1.w = 0;

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxADC->Callbacks.DepDeinit, pxADC);

    /* disable clock */
    RCC_vClockDisable(RCC_POS_ADC1);
}

/**
 * @brief Initializes the regular ADC channels for conversion using the setup configuration.
 * @param pxADC: pointer to the ADC handle structure
 * @param axChannels: ADC regular channel configuration array pointer
 * @param ucChannelCount: number of channels to configure
 *
 * @note  This device line does not support channel ordering, the conversion scan goes through the
 *        activated channels either in ascending or descending order, depending on ScanDirection.
 * @note  This device line does not support channel-wise sample time configuration,
 *        simply the first element's setting will be applied for all channels.
 */
void ADC_vChannelConfig(
        ADC_HandleType *            pxADC,
        const ADC_ChannelInitType   axChannels[],
        uint8_t                     ucChannelCount)
{
    /* No ongoing conversion on regular group */
    if (ADC_REG_BIT(pxADC, CR, ADSTART) == 0)
    {
        uint8_t i, ucWdgUsers = 0;
        uint32_t ulCHSELR = 0;

        /* Set common sampling time */
        pxADC->Inst->SMPR.b.SMP = axChannels[0].SampleTime;

        /* Set each active channel's bit */
        for (i = 0; i < ucChannelCount; i++)
        {
            SET_BIT(ulCHSELR, 1 << axChannels[i].Number);

            /* Internal channel configuration (if applicable) */
            ADC_prvInitInternalChannel(pxADC, axChannels[i].Number);

            /* If a watchdog is used for the channel, set it in the configuration */
            if (axChannels[i].Watchdog != ADC_AWD_NONE)
            {
                pxADC->Inst->CFGR1.b.AWD1CH = axChannels[i].Number;
                ucWdgUsers++;
            }
        }

        /* Apply channel selection */
        pxADC->Inst->CHSELR = ulCHSELR;

        /* Set conversion count */
        pxADC->ConversionCount = ucChannelCount;

        /* Determine watchdog configuration based on user count */
        switch (ucWdgUsers)
        {
            case 0:
                CLEAR_BIT(pxADC->Inst->CFGR1.w,
                        ADC_CFGR1_AWD1SGL | ADC_CFGR1_AWD1EN);
                break;

            case 1:
                SET_BIT(pxADC->Inst->CFGR1.w,
                        ADC_CFGR1_AWD1SGL | ADC_CFGR1_AWD1EN);
                break;

            default:
                /* All regular channels will be monitored,
                 * even if they were not configured so! */
                MODIFY_REG(pxADC->Inst->CFGR1.w,
                        ADC_CFGR1_AWD1SGL | ADC_CFGR1_AWD1EN, ADC_CFGR1_AWD1EN);
                break;
        }
    }
}

/**
 * @brief Enables the ADC peripheral and starts conversion if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStart(ADC_HandleType * pxADC)
{
    /* If low power mode AutoPowerOff is enabled,
     * power-on/off phases are performed automatically by hardware */
    if (ADC_REG_BIT(pxADC, CR, ADSTART) == 0)
    {
        /* ADC turn ON */
        if ((ADC_REG_BIT(pxADC, CFGR1, AUTOFF) == 0) && ADC_prvEnableInst(pxADC->Inst))
        {
            /* Wait until ADRDY flag is set ( < 1us) */
            uint32_t ulTimeout = 1;
            XPD_eWaitForMatch(&pxADC->Inst->ISR.w, ADC_ISR_ADRDY, ADC_ISR_ADRDY, &ulTimeout);
        }

        /* clear regular group conversion flag and overrun flag */
        SET_BIT(pxADC->Inst->ISR.w, ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);

        /* if ADC is either not in multimode, or the multimode master,
         * enable software conversion of regular channels */
        {
            ADC_REG_BIT(pxADC, CR, ADSTART) = 1;
        }

#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
        pxADC->Errors = ADC_ERROR_NONE;
#endif
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

    if ((eOperation == ADC_OPERATION_CONVERSION)
            && ((pxADC->EndFlagSelection & ADC_IER_EOSIE) != 0))
    {
        eOperation = ADC_ISR_EOS;
    }

    /* Polling for single conversion is not allowed when DMA is used,
     * and EOC is raised on end of single conversion */
    if (eOperation == ADC_OPERATION_CONVERSION)
    {
        if (ADC_REG_BIT(pxADC, CFGR1, DMAEN) == 1)
        { eResult = XPD_ERROR; }
    }

    /* Wait until operation flag is set */
    if (eResult == XPD_OK)
    {
        eResult = XPD_eWaitForMatch(&pxADC->Inst->ISR.w, eOperation, eOperation, &ulTimeout);

        /* Clear end of conversion flag of regular group if low power feature
         * "LowPowerAutoWait " is disabled, to not interfere with this feature
         * until data register is read. */
        if ((eResult == XPD_OK) && ((eOperation & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
            && (ADC_REG_BIT(pxADC, CFGR1, WAIT) == 0))
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

    /* End of conversion flag for regular channels */
    if (    ((ulISR & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
         && ((ulIER & (ADC_IER_EOCIE | ADC_IER_EOSIE)) != 0))
    {
        uint32_t ulCFGR = pxADC->Inst->CFGR1.w;

        /* If the conversion is not continuous / external triggered */
        if ((ulCFGR & (ADC_CFGR1_CONT | ADC_CFGR1_EXTEN)) == 0)
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

    /* Check analog watchdog flag */
    if (    ((ulISR & ADC_ISR_AWD1) != 0)
         && ((ulIER & ADC_IER_AWD1IE) != 0))
    {
        /* watchdog callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.Watchdog, pxADC);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        ADC_FLAG_CLEAR(pxADC, AWD1);
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

    {
        /* Set up DMA for transfer */
        eResult = DMA_eStart_IT(pxADC->DMA.Conversion,
                (void *)&pxADC->Inst->DR, pvAddress, pxADC->ConversionCount);

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
            ADC_REG_BIT(pxADC, CFGR1, DMAEN) = 1;

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
    ADC_REG_BIT(pxADC, CFGR1, DMAEN) = 0;

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
        uint32_t ulScaling = pxADC->Inst->CFGR1.b.RES * 2;

        /* Analog watchdogs configuration */
        {
            /* Shift the offset in function of the selected ADC resolution:
             * Thresholds have to be left-aligned on bit 11, the LSB (right bits)
             * are set to 0 */
            pxADC->Inst->TR.w = (usHighThd  << (16 + ulScaling))
                               | (usLowThd  << ulScaling);
        }
    }
}

/**
 * @brief Returns the currently active analog watchdog.
 * @param pxADC: pointer to the ADC handle structure
 * @return Set if the watchdog is active, 0 otherwise
 */
ADC_WatchdogType ADC_eWatchdogStatus(ADC_HandleType * pxADC)
{
    return ADC_FLAG_STATUS(pxADC, AWD1);
}

/** @} */

/** @} */

/** @addtogroup ADC_Calibration
 * @{ */

/** @defgroup ADC_Calibration_Exported_Functions ADC Calibration Exported Functions
 * @{ */

/**
 * @brief Executes an ADC self-calibration sequence.
 * @param pxADC: pointer to the ADC handle structure
 * @param eDifferential: unused
 * @return Result of the operation
 */
XPD_ReturnType ADC_eCalibrate(ADC_HandleType * pxADC, bool eDifferential)
{
    XPD_ReturnType eResult = XPD_ERROR;

    /* Calibration prerequisite: ADC must be disabled. */
    if (ADC_REG_BIT(pxADC, CR, ADEN) == 0)
    {
        uint32_t ulTimeout = ADC_CALIBRATION_TIMEOUT;

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
