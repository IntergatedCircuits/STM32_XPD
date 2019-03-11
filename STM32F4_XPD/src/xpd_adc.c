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

/** @addtogroup ADC_Clock_Source
 * @{ */

/** @defgroup ADC_Clock_Source_Exported_Functions ADC Clock Source Exported Functions
 * @{ */

/**
 * @brief Sets the new source clock for the ADCs.
 * @param ClockSource: the new source clock which should be configured
 */
void ADC_vClockConfig(ADC_ClockSourceType ClockSource)
{
    ADC_COMMON()->CCR.b.ADCPRE = ClockSource;
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t ADC_ulClockFreq_Hz(void)
{
    return RCC_ulClockFreq_Hz(PCLK2) / ((ADC_COMMON()->CCR.b.ADCPRE + 1) * 2);
}

/** @} */

/** @} */

/** @addtogroup ADC_Core
 * @{ */

#define ADC_STAB_DELAY_US         3
#define ADC_TEMPSENSOR_DELAY_US   10

#define ADC_SQR_MASK            (ADC_SQR3_SQ1 >> ADC_SQR3_SQ1_Pos)
#define ADC_SQR_SIZE            (ADC_SQR3_SQ2_Pos - ADC_SQR3_SQ1_Pos)
#define ADC_SQR_REGDIR          (-1)

static void ADC_prvDmaConversionRedirect(void *pxDMA)
{
    ADC_HandleType* pxADC = (ADC_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    /* Not externally triggered, not continuous, last element of scan, disable interrupt */
    if (    ((pxADC->Inst->CR2.w & (ADC_CR2_CONT | ADC_CR2_EXTEN)) == 0)
         && ((pxADC->Inst->SQR1.b.L == 0) || (ADC_REG_BIT(pxADC,CR2,EOCS) == 0)))
    {
        ADC_IT_DISABLE(pxADC, EOC);
    }

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

/* Sets the sample time for a channel configuration */
static void ADC_prvSampleTimeConfig(ADC_HandleType * pxADC, const ADC_ChannelInitType * pxChannel)
{
    __IO uint32_t *pulSMPR = &pxADC->Inst->SMPR2.w;
    uint32_t ulNumber = pxChannel->Number;

    /* Channel sampling time configuration */
    if (ulNumber > 10)
    {
        pulSMPR = &pxADC->Inst->SMPR1.w;
        ulNumber -= 10;
    }
    ulNumber *= 3;

    /* set the sample time */
    MODIFY_REG(*pulSMPR, ADC_SMPR2_SMP0 << ulNumber, pxChannel->SampleTime << ulNumber);
}

/* Enables an internal channel for conversion */
static void ADC_prvInitInternalChannel(ADC_HandleType * pxADC, uint8_t ucChannel)
{
#if (ADC_COUNT > 1)
    /* Only ADC1 has access to internal measurement channels */
    if (pxADC->Inst == ADC1)
#endif
    {
        /* if ADC1 channel 18 is selected, enable VBAT */
        if (ucChannel == ADC1_VBAT_CHANNEL)
        {
            /* enable the VBAT input */
            ADC_COMMON_REG_BIT(pxADC,CCR,VBATE) = 1;
        }
        /* if ADC1 channel 16 or 17 is selected, enable Temperature sensor and VREFINT */
        else if ((ucChannel == ADC1_TEMPSENSOR_CHANNEL) ||
                 (ucChannel == ADC1_VREFINT_CHANNEL))
        {
            /* enable the TS-VREF input */
            ADC_COMMON_REG_BIT(pxADC,CCR,TSVREFE) = 1;

            if (ucChannel == ADC1_TEMPSENSOR_CHANNEL)
            {
                /* wait temperature sensor stabilization */
                XPD_vDelay_us(ADC_TEMPSENSOR_DELAY_US);
            }
        }
    }
}

/* Common (regular-injected) channel configuration includes:
 *  - Sample Time
 *  - Watchdogs channel selection
 *  - Internal channels activation */
static uint8_t ADC_prvCommonChannelConfig(
        ADC_HandleType *            pxADC,
        const ADC_ChannelInitType   axChannels[],
        uint8_t                     ucChannelCount)
{
    uint8_t ucAWD1Chs = 0;

    {
        uint8_t i;

        /* Channel sampling time configuration */
        for (i = 0; i < ucChannelCount; i++)
        {
            /* Sample time configuration */
            ADC_prvSampleTimeConfig(pxADC, &axChannels[i]);

            /* Internal channel configuration (if applicable) */
            ADC_prvInitInternalChannel(pxADC, axChannels[i].Number);

            /* If a watchdog is used for the channel, set it in the configuration */
            if (axChannels[i].Watchdog == ADC_AWD1)
            {
                pxADC->Inst->CR1.b.AWDCH = axChannels[i].Number;
                ucAWD1Chs++;
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
    uint32_t ulCR1, ulCR1Mask;

    /* Enable clock */
    RCC_vClockEnable(pxADC->CtrlPos);

    ulCR1Mask = ADC_CR1_SCAN | ADC_CR1_DISCEN | ADC_CR1_DISCNUM | ADC_CR1_RES;

    /* The bit config of CR1 is stored with an offset */
    ulCR1 = (pxConfig->w >> 6) & ~(ADC_CR1_DISCEN | ADC_CR1_JDISCEN | ADC_CR1_DISCNUM);

    if (pxConfig->DiscontinuousCount != 0)
    {
        ulCR1 |= ADC_CR1_DISCEN | ((pxConfig->DiscontinuousCount - 1) << ADC_CR1_DISCNUM_Pos);
    }

    MODIFY_REG(pxADC->Inst->CR1.w, ulCR1Mask, ulCR1);
    MODIFY_REG(pxADC->Inst->CR2.w, ADC_CR2_ALIGN | ADC_CR2_CONT | ADC_CR2_DDS |
            ADC_CR2_EOCS | ADC_CR2_EXTEN | ADC_CR2_EXTSEL,
            pxConfig->w);

    /* dependencies initialization */
    XPD_SAFE_CALLBACK(pxADC->Callbacks.DepInit, pxADC);
}

/**
 * @brief Deinitializes the ADC peripheral.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vDeinit(ADC_HandleType * pxADC)
{
    /* Turn off ADC */
    ADC_REG_BIT(pxADC,CR2,ADON) = 0;

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxADC->Callbacks.DepDeinit, pxADC);

    /* disable clock */
    RCC_vClockDisable(pxADC->CtrlPos);
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
    uint8_t i;
    uint32_t ulSeqOffset = ADC_SQR3_SQ1_Pos;
    __IO uint32_t *pulSQR = &pxADC->Inst->SQR3.w;
    uint8_t ucAWD1Chs = 0;

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
        {
            uint32_t ulClrMask = ADC_CR1_AWDSGL;
            uint32_t ulSetMask = 0;

            if (ucAWD1Chs == 0)
            {
                ulClrMask = ADC_CR1_AWDEN | ADC_CR1_AWDSGL;
            }
            else
            {
                ulSetMask = ADC_CR1_AWDEN;

                if (ucAWD1Chs == 1)
                {
                    /* In case of a single channel, set SGL flag
                     * AWD1CH is set in common configurator */
                    ulSetMask = ADC_CR1_AWDEN | ADC_CR1_AWDSGL;
                }

                /* Disable other group's watchdog if it targeted a single channel */
                if ((pxADC->Inst->CR1.w & (ADC_CR1_AWDSGL | ADC_CR1_JAWDEN)) ==
                                          (ADC_CR1_AWDSGL | ADC_CR1_JAWDEN))
                {
                    ulClrMask = ADC_CR1_JAWDEN;
                }
            }
            pxADC->Inst->CR1.w = (pxADC->Inst->CR1.w & (~ulClrMask)) | ulSetMask;
        }

        /* Set the number of conversions */
        pxADC->Inst->SQR1.b.L = ucChannelCount - 1;
}

/**
 * @brief Enables the ADC peripheral and starts conversion if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStart(ADC_HandleType * pxADC)
{
    /* if not already on, wait until ADC starts up */
    if (ADC_REG_BIT(pxADC,CR2,ADON) == 0)
    {
        ADC_REG_BIT(pxADC,CR2,ADON) = 1;

        XPD_vDelay_us(ADC_STAB_DELAY_US);
    }

    /* initialize number of remaining conversions count */
    pxADC->ActiveConversions = pxADC->Inst->SQR1.b.L + 1;

#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
    pxADC->Errors = ADC_ERROR_NONE;
#endif

    /* if no external trigger present and ADC is either not in multimode, or the multimode master,
     * enable software conversion of regular channels */
    if (    (pxADC->Inst->CR2.b.EXTEN == 0)
         && ((ADC_COMMON(pxADC)->CCR.b.MULTI == 0) || (pxADC->Inst == ADC1)))
    {
        ADC_REG_BIT(pxADC,CR2,SWSTART) = 1;
    }
}

/**
 * @brief Stops the ADC peripheral.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vStop(ADC_HandleType * pxADC)
{
    ADC_REG_BIT(pxADC,CR2,ADON) = 0;
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

    /* Polling for single conversion is not allowed when DMA is used,
     * and EOC is raised on end of single conversion */
    if (eOperation == ADC_OPERATION_CONVERSION)
    {
        if ((pxADC->Inst->CR2.w & (ADC_CR2_EOCS | ADC_CR2_DMA))
                               == (ADC_CR2_EOCS | ADC_CR2_DMA))
        {
            eResult = XPD_ERROR;
        }
    }

    if (eResult == XPD_OK)
    {
        /* Wait until operation flag is set */
        eResult = XPD_eWaitForMatch(&pxADC->Inst->SR.w,
                eOperation, eOperation, &ulTimeout);
        if (eResult == XPD_OK)
        {
            CLEAR_BIT(pxADC->Inst->SR.w, eOperation);
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
    /* ADC overrun and end of conversion interrupt for regular group */
#ifdef __XPD_ADC_ERROR_DETECT
    ADC_IT_ENABLE(pxADC, OVR);
#endif
    ADC_IT_ENABLE(pxADC, EOC);

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
    CLEAR_BIT(pxADC->Inst->CR1.w, ADC_CR1_EOCIE | ADC_CR1_JEOCIE | ADC_CR1_OVRIE);
}

/**
 * @brief ADC interrupt handler that provides handle callbacks.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vIRQHandler(ADC_HandleType * pxADC)
{
    uint32_t ulSR = pxADC->Inst->SR.w;
    uint32_t ulCR1 = pxADC->Inst->CR1.w;

    /* End of conversion flag for regular channels */
    if (    ((ulSR  & ADC_SR_EOC) != 0)
         && ((ulCR1 & ADC_CR1_EOCIE) != 0))
    {
        /* if the conversion is not continuous / external triggered */
        if ((pxADC->Inst->CR2.w & (ADC_CR2_CONT | ADC_CR2_EXTEN)) == 0)
        {
            /* single conversion */
            if (ADC_REG_BIT(pxADC,CR2,EOCS) == 0)
            {
                /* disable the ADC end of conversion interrupt for regular group */
                ADC_IT_DISABLE(pxADC, EOC);
            }
            /* conversion sequence */
            else
            {
                /* initialize number of remaining conversions count */
                if (pxADC->ActiveConversions == 0)
                {
                    pxADC->ActiveConversions = pxADC->Inst->SQR1.b.L + 1;
                }

                /* decrement the number of conversion when an interrupt occurs */
                pxADC->ActiveConversions--;

                /* all conversions finished */
                if (pxADC->ActiveConversions == 0)
                {
                    /* disable the ADC end of conversion interrupt for regular group */
                    ADC_IT_DISABLE(pxADC, EOC);
                }
            }
        }

        /* clear the ADC flag for regular end of conversion */
        ADC_FLAG_CLEAR(pxADC, EOC);

        /* Callback only if single conversion, or end of sequence */
        if ((ADC_REG_BIT(pxADC,CR2,EOCS) == 0) || (pxADC->ActiveConversions == 0))
        {
            /* conversion complete callback */
            XPD_SAFE_CALLBACK(pxADC->Callbacks.ConvComplete, pxADC);
        }
    }

    /* End of conversion flag for injected channels */
    if (    ((ulSR  & ADC_SR_JEOC) != 0)
         && ((ulCR1 & ADC_CR1_JEOCIE) != 0))
    {
        /* injected conversion is not continuous or not automatic, and software triggered */
        if (((ADC_REG_BIT(pxADC,CR2,CONT) & ADC_REG_BIT(pxADC,CR1,JAUTO)) == 0) &&
            (pxADC->Inst->CR2.b.JEXTEN == 0))
        {
            /* disable the ADC end of conversion interrupt for injected group */
            ADC_IT_DISABLE(pxADC, JEOC);
        }

        /* clear the ADC flag for injected end of conversion */
        ADC_FLAG_CLEAR(pxADC, JEOC);

        /* injected conversion complete callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.InjConvComplete, pxADC);
    }

    /* Check analog watchdog flag */
    if (    ((ulSR  & ADC_SR_AWD) != 0)
         && ((ulCR1 & ADC_CR1_AWDIE) != 0))
    {
        /* watchdog callback */
        XPD_SAFE_CALLBACK(pxADC->Callbacks.Watchdog, pxADC);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        ADC_FLAG_CLEAR(pxADC, AWD);
    }

#ifdef __XPD_ADC_ERROR_DETECT
    /* Check Overrun flag */
    if (    ((ulSR  & ADC_SR_OVR) != 0)
         && ((ulCR1 & ADC_CR1_OVRIE) != 0))
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
    XPD_ReturnType eResult;

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
            ADC_REG_BIT(pxADC, CR2, DMA) = 1;

            ADC_vStart(pxADC);
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
    ADC_REG_BIT(pxADC, CR2, DMA) = 0;

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
    if (eWatchdog != ADC_AWD_NONE)
    {
        uint32_t ulScaling   = pxADC->Inst->CR1.b.RES * 2;

        /* Analog watchdogs configuration */
        {
            /* Shift the offset in function of the selected ADC resolution:
             * Thresholds have to be left-aligned on bit 11, the LSB (right bits)
             * are set to 0 */
            pxADC->Inst->HTR = usHighThd << ulScaling;
            pxADC->Inst->LTR = usLowThd  << ulScaling;
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
    return ADC_FLAG_STATUS(pxADC, AWD);
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
    uint32_t ulCRx;

    ulCRx = pxConfig->w << 8;

    /* Cannot use both auto-injected mode and discontinuous mode simultaneously */
    if (pxConfig->AutoInjection != 0)
    {
        CLEAR_BIT(ulCRx, ADC_CR1_JDISCEN);
    }

    MODIFY_REG(pxADC->Inst->CR1.w, ADC_CR1_JAUTO | ADC_CR1_JDISCEN, ulCRx);
    MODIFY_REG(pxADC->Inst->CR1.w, ADC_CR2_JEXTEN | ADC_CR2_JEXTSEL, ulCRx);
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
    uint32_t ulSeqOffset = ADC_JSQR_JSQ1_Pos;

    for (i = 0; i < ucChannelCount; i++)
    {
        /* Channel configuration for a given rank */
        MODIFY_REG(pxADC->Inst->JSQR.w,
                ADC_SQR_MASK << ulSeqOffset, axChannels[i].Number << ulSeqOffset);

        ulSeqOffset += ADC_SQR_SIZE;

        /* Offset configuration */
        (&pxADC->Inst->JOFR1)[i] = axChannels[i].Offset;
    }

    /* Forward to channel common configurator */
    ucAWD1Chs = ADC_prvCommonChannelConfig(pxADC, axChannels, ucChannelCount);

    /* Configuration of AWD1 */
    {
        uint32_t ulClrMask = ADC_CR1_AWDSGL;
        uint32_t ulSetMask = 0;

        if (ucAWD1Chs == 0)
        {
            ulClrMask = ADC_CR1_JAWDEN | ADC_CR1_AWDSGL;
        }
        else
        {
            ulSetMask = ADC_CR1_JAWDEN;

            if (ucAWD1Chs == 1)
            {
                /* In case of a single channel, set SGL flag
                 * AWD1CH is set in common configurator */
                ulSetMask = ADC_CR1_JAWDEN | ADC_CR1_AWDSGL;
            }

            /* Disable other group's watchdog if it targeted a single channel */
            if ((pxADC->Inst->CR1.w & (ADC_CR1_AWDSGL | ADC_CR1_AWDEN)) ==
                                      (ADC_CR1_AWDSGL | ADC_CR1_AWDEN))
            {
                ulClrMask = ADC_CR1_AWDEN;
            }
        }
        pxADC->Inst->CR1.w = (pxADC->Inst->CR1.w & (~ulClrMask)) | ulSetMask;
    }

    /* Set the injected sequence length */
    pxADC->Inst->JSQR.b.JL = ucChannelCount - 1;
}

/**
 * @brief Enables the ADC peripheral and starts injected conversion
 *        if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStart(ADC_HandleType * pxADC)
{
    /* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization */
    if (ADC_REG_BIT(pxADC,CR2,ADON) == 0)
    {
        /* Enable the Peripheral */
        ADC_REG_BIT(pxADC,CR2,ADON) = 1;

        /* Delay for ADC stabilization time */
        /* Compute number of CPU cycles to wait for */
        XPD_vDelay_us(ADC_STAB_DELAY_US);
    }

    /* If no external trigger present and ADC is either not in multimode, or the multimode master,
     * enable software conversion of injected channels */
    if (    ( pxADC->Inst->CR2.b.JEXTEN       == 0)
         && ( ADC_REG_BIT(pxADC, CR1, JAUTO)  == 0)
         && ((ADC_COMMON_REG_BIT(pxADC,CCR,MULTI) == DISABLE) || (pxADC->Inst == ADC1)))
    {
        ADC_REG_BIT(pxADC,CR2,JSWSTART) = 1;
    }
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStop(ADC_HandleType * pxADC)
{
    /* Check against auto injection */
    if (ADC_REG_BIT(pxADC,CR1,JAUTO) == 0)
    {
        /* Stop conversions on regular and injected groups */
        ADC_REG_BIT(pxADC,CR2,ADON) = 0;
    }
}

/**
 * @brief Enables the ADC peripheral and interrupts, and starts injected conversion
 *        if the trigger is software.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStart_IT(ADC_HandleType * pxADC)
{
#ifdef __XPD_ADC_ERROR_DETECT
    ADC_IT_ENABLE(pxADC, OVR);
#endif
    ADC_IT_ENABLE(pxADC, JEOC);

    ADC_vInjectedStart(pxADC);
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral and interrupts.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vInjectedStop_IT(ADC_HandleType * pxADC)
{
    ADC_vInjectedStop(pxADC);

    CLEAR_BIT(pxADC->Inst->CR1.w, ADC_CR1_JEOCIE | ADC_CR1_OVRIE);
}

/** @} */

/** @} */

#ifdef ADC123_COMMON
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
    ADC_Common_TypeDef *pxCommon = ADC_COMMON(pxADC);

    pxCommon->CCR.b.MULTI = pxConfig->Mode;
    pxCommon->CCR.b.DMA   = pxConfig->DMAAccessMode;
    pxCommon->CCR.b.DELAY = pxConfig->InterSamplingDelay - 5;
}

/**
 * @brief Sets up and enables a DMA transfer for the multi ADC regular conversions.
 * @param pxADC: pointer to the ADC handle structure
 * @param pvAddress: memory address to the conversion data storage
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType ADC_eMultiModeStart_DMA(ADC_HandleType * pxADC, void * pvAddress)
{
    XPD_ReturnType eResult;

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

            /* pass the continuous DMA request setting to the common config */
            ADC_COMMON_REG_BIT(pxADC,CCR,DDS) = ADC_REG_BIT(pxADC, CR2, DDS);

            /* Enable ADC DMA mode */
            ADC_REG_BIT(pxADC, CR2, DMA) = 1;

            ADC_vStart(pxADC);
        }
    return eResult;
}

/**
 * @brief Disables the ADC and the common DMA transfer.
 * @param pxADC: pointer to the ADC handle structure
 */
void ADC_vMultiModeStop_DMA(ADC_HandleType * pxADC)
{
    /* disable the selected ADC DMA request after last transfer */
    ADC_COMMON_REG_BIT(pxADC,CCR,DDS) = 0;

    ADC_vStop_DMA(pxADC);
}

/** @} */

/** @} */
#endif /* ADC123_COMMON */

/** @} */
