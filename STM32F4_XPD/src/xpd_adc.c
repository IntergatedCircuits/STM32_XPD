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
#include "xpd_rcc_cc.h"
#include "xpd_utils.h"

#if defined(USE_XPD_ADC)

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
void XPD_ADC_ClockConfig(ADC_ClockSourceType ClockSource)
{
    ADC->CCR.b.ADCPRE = ClockSource;
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t XPD_ADC_GetClockFreq(void)
{
    return XPD_RCC_GetClockFreq(PCLK2) / ((ADC->CCR.b.ADCPRE + 1) * 2);
}

/** @} */

/** @} */

/** @addtogroup ADC_Core
 * @{ */

#define ADC_STAB_DELAY_US         3
#define ADC_TEMPSENSOR_DELAY_US   10

static void adc_dmaConversionRedirect(void *hdma)
{
    ADC_HandleType* hadc = (ADC_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    /* Not externally triggered, not continuous, last element of scan, disable interrupt */
    if (    ((hadc->Inst->CR2.w & (ADC_CR2_CONT | ADC_CR2_EXTEN)) == 0)
         && ((hadc->Inst->SQR1.b.L == 0) || (ADC_REG_BIT(hadc,CR2,EOCS) == 0)))
    {
        XPD_ADC_DisableIT(hadc, EOC);
    }

    XPD_SAFE_CALLBACK(hadc->Callbacks.ConvComplete, hadc);
}

#ifdef USE_XPD_DMA_ERROR_DETECT
static void adc_dmaErrorRedirect(void *hdma)
{
    ADC_HandleType* hadc = (ADC_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    /* Update error code */
    hadc->Errors |= ADC_ERROR_DMA;

    XPD_SAFE_CALLBACK(hadc->Callbacks.Error, hadc);
}
#endif

static void adc_sequencerConfig(ADC_HandleType * hadc, uint8_t Channel, uint8_t Rank)
{
    uint32_t regoffset, rankm, choffset;

    /* channel configuration for a given rank */
    if (Rank < 7)
    {
        regoffset = 2;
        rankm = 1;
    }
    else if (Rank < 13)
    {
        regoffset = 1;
        rankm = 7;
    }
    else
    {
        regoffset = 0;
        rankm = 13;
    }

    /* set the channel for the selected rank */
    choffset = (Rank - rankm) * 5;
    MODIFY_REG((&hadc->Inst->SQR1.w)[regoffset], ADC_SQR3_SQ1 << choffset, Channel << choffset);
}

static void adc_sampleTimeConfig(ADC_HandleType * hadc, uint8_t Channel, ADC_SampleTimeType SampleTime)
{
    uint32_t regoffset = 1;

    /* Channel sampling time configuration */
    if (Channel > 10)
    {
        regoffset = 0;
        Channel -= 10;
    }
    Channel *= 3;

    /* set the sample time */
    MODIFY_REG((&hadc->Inst->SMPR1.w)[regoffset], ADC_SMPR2_SMP0  << Channel, SampleTime << Channel);
}

/* Enables an internal channel for conversion */
static void adc_initInternalChannel(ADC_HandleType * hadc, uint32_t Channel)
{
    /* Only ADC1 has access to internal measurement channels */
    if (hadc->Inst == ADC1)
    {
        /* if ADC1 channel 18 is selected, enable VBAT */
        if (Channel == ADC_VBAT_CHANNEL)
        {
            /* enable the VBAT input */
            ADC_COMMON_REG_BIT(CCR, VBATE) = 1;
        }
        /* if ADC1 channel 16 or 17 is selected, enable Temperature sensor and VREFINT */
        else if ((Channel == ADC_TEMPSENSOR_CHANNEL) || (Channel == ADC_VREFINT_CHANNEL))
        {
            /* enable the TS-VREF input */
            ADC_COMMON_REG_BIT(CCR,TSVREFE) = 1;

            if (Channel == ADC_TEMPSENSOR_CHANNEL)
            {
                /* wait temperature sensor stabilization */
                XPD_Delay_us(ADC_TEMPSENSOR_DELAY_US);
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
    /* enable clock */
    XPD_SAFE_CALLBACK(hadc->ClockCtrl, ENABLE);

#ifdef ADC_BB
    hadc->Inst_BB = ADC_BB(hadc->Inst);
#endif

    XPD_SAFE_CALLBACK(hadc->Callbacks.DepInit, hadc);

    ADC_REG_BIT(hadc,CR1,SCAN)  = Config->ScanMode;
    ADC_REG_BIT(hadc,CR2,ALIGN) = Config->LeftAlignment;
    ADC_REG_BIT(hadc,CR2,CONT)  = Config->ContinuousMode;
    ADC_REG_BIT(hadc,CR2,EOCS)  = Config->EndFlagSelection;
    ADC_REG_BIT(hadc,CR2,DDS)   = Config->ContinuousDMARequests;

    hadc->Inst->CR1.b.RES       = Config->Resolution;

    hadc->Inst->SQR1.b.L        = Config->ConversionCount - 1;

    /* external trigger configuration */
    if(Config->Trigger.Source == ADC_TRIGGER_SOFTWARE)
    {
        /* reset the external trigger */
        CLEAR_BIT(hadc->Inst->CR2.w, ADC_CR2_EXTSEL | ADC_CR2_EXTEN);
    }
    else
    {
        /* select external trigger and polarity to start conversion */
        hadc->Inst->CR2.b.EXTSEL = Config->Trigger.Source;
        hadc->Inst->CR2.b.EXTEN  = Config->Trigger.Edge;
    }

    /* discontinuous mode configuration */
    if (Config->DiscontinuousCount != 0)
    {
        ADC_REG_BIT(hadc,CR1,DISCEN) = 1;

        hadc->Inst->CR1.b.DISCNUM    = Config->DiscontinuousCount - 1;
    }
    else
    {
        CLEAR_BIT(hadc->Inst->CR1.w, ADC_CR1_DISCEN | ADC_CR1_DISCNUM);
    }

    return XPD_OK;
}

/**
 * @brief Deinitializes the ADC peripheral.
 * @param hadc: pointer to the ADC handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_ADC_Deinit(ADC_HandleType * hadc)
{
    ADC_REG_BIT(hadc,CR2,ADON) = 0;

    /* disable clock */
    XPD_SAFE_CALLBACK(hadc->ClockCtrl, DISABLE);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(hadc->Callbacks.DepDeinit, hadc);

    return XPD_OK;
}

/**
 * @brief Initializes a regular ADC channel for conversion using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to ADC regular channel setup configuration
 */
void XPD_ADC_ChannelConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Config)
{
    /* sequencer rank configuration */
    adc_sequencerConfig(hadc, Config->Channel, Config->Rank);

    /* sample time configuration */
    adc_sampleTimeConfig(hadc, Config->Channel, Config->SampleTime);

    /* Internal channel configuration (if applicable) */
    adc_initInternalChannel(hadc, Config->Channel);
}

/**
 * @brief Enables the ADC peripheral and starts conversion if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Start(ADC_HandleType * hadc)
{
    /* if not already on, wait until ADC starts up */
    if (ADC_REG_BIT(hadc,CR2,ADON) == 0)
    {
        ADC_REG_BIT(hadc,CR2,ADON) = 1;

        XPD_Delay_us(ADC_STAB_DELAY_US);
    }

    /* if no external trigger present and ADC is either not in multimode, or the multimode master,
     * enable software conversion of regular channels */
    if (    (hadc->Inst->CR2.b.EXTEN == 0)
         && ((ADC->CCR.b.MULTI == 0) || (hadc->Inst == ADC1)))
    {
        ADC_REG_BIT(hadc,CR2,SWSTART) = 1;
    }

    /* initialize number of remaining conversions count */
    hadc->ActiveConversions = hadc->Inst->SQR1.b.L + 1;

#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
    hadc->Errors = ADC_ERROR_NONE;
#endif
}

/**
 * @brief Stops the ADC peripheral.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Stop(ADC_HandleType * hadc)
{
    ADC_REG_BIT(hadc,CR2,ADON) = 0;
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
    XPD_ReturnType result;

    /* Polling for single conversion is not allowed when DMA is used,
     * and EOC is raised on end of single conversion */
    if (    ( Operation == ADC_OPERATION_CONVERSION )
         && ((hadc->Inst->CR2.w & (ADC_CR2_EOCS | ADC_CR2_DMA)) == (ADC_CR2_EOCS | ADC_CR2_DMA)))
    {
        result = XPD_ERROR;
    }
    else
    {
        /* Wait until operation flag is set */
        result = XPD_WaitForMatch(&hadc->Inst->SR.w, Operation, Operation, &Timeout);
        if (result == XPD_OK)
        {
            CLEAR_BIT(hadc->Inst->SR.w, Operation);
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
    /* ADC overrun and end of conversion interrupt for regular group */
#ifdef USE_XPD_ADC_ERROR_DETECT
    XPD_ADC_EnableIT(hadc, OVR);
#endif
    XPD_ADC_EnableIT(hadc, EOC);

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
    CLEAR_BIT(hadc->Inst->CR1.w, ADC_CR1_EOCIE | ADC_CR1_JEOCIE | ADC_CR1_OVRIE);
}

/**
 * @brief ADC interrupt handler that provides handle callbacks.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_IRQHandler(ADC_HandleType * hadc)
{
    uint32_t sr = hadc->Inst->SR.w;
    uint32_t cr1 = hadc->Inst->CR1.w;

    /* End of conversion flag for regular channels */
    if (    ((sr  & ADC_SR_EOC) != 0)
         && ((cr1 & ADC_CR1_EOCIE) != 0))
    {
        /* if the conversion is not continuous / external triggered */
        if ((hadc->Inst->CR2.w & (ADC_CR2_CONT | ADC_CR2_EXTEN)) == 0)
        {
            /* single conversion */
            if (ADC_REG_BIT(hadc,CR2,EOCS) == 0)
            {
                /* disable the ADC end of conversion interrupt for regular group */
                XPD_ADC_DisableIT(hadc, EOC);
            }
            /* conversion sequence */
            else
            {
                /* initialize number of remaining conversions count */
                if (hadc->ActiveConversions == 0)
                {
                    hadc->ActiveConversions = hadc->Inst->SQR1.b.L + 1;
                }

                /* decrement the number of conversion when an interrupt occurs */
                hadc->ActiveConversions--;

                /* all conversions finished */
                if (hadc->ActiveConversions == 0)
                {
                    /* disable the ADC end of conversion interrupt for regular group */
                    XPD_ADC_DisableIT(hadc, EOC);
                }
            }
        }

        /* clear the ADC flag for regular end of conversion */
        XPD_ADC_ClearFlag(hadc, EOC);

        /* Callback only if single conversion, or end of sequence */
        if ((ADC_REG_BIT(hadc,CR2,EOCS) == 0) || (hadc->ActiveConversions == 0))
        {
            /* conversion complete callback */
            XPD_SAFE_CALLBACK(hadc->Callbacks.ConvComplete, hadc);
        }
    }

    /* End of conversion flag for injected channels */
    if (    ((sr  & ADC_SR_JEOC) != 0)
         && ((cr1 & ADC_CR1_JEOCIE) != 0))
    {
        /* injected conversion is not continuous or not automatic, and software triggered */
        if (((ADC_REG_BIT(hadc,CR2,CONT) & ADC_REG_BIT(hadc,CR1,JAUTO)) == 0) && (hadc->Inst->CR2.b.JEXTEN == 0))
        {
            /* disable the ADC end of conversion interrupt for injected group */
            XPD_ADC_DisableIT(hadc, JEOC);
        }

        /* clear the ADC flag for injected end of conversion */
        XPD_ADC_ClearFlag(hadc, JEOC);

        /* injected conversion complete callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.InjConvComplete, hadc);
    }

    /* Check analog watchdog flag */
    if (    ((sr  & ADC_SR_AWD) != 0)
         && ((cr1 & ADC_CR1_AWDIE) != 0))
    {
        /* watchdog callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Watchdog, hadc);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        XPD_ADC_ClearFlag(hadc, AWD);
    }

#ifdef USE_XPD_ADC_ERROR_DETECT
    /* Check Overrun flag */
    if (    ((sr  & ADC_SR_OVR) != 0)
         && ((cr1 & ADC_CR1_OVRIE) != 0))
    {
        /* Update error code */
        hadc->Errors |= ADC_ERROR_OVERRUN;

        /* Clear the Overrun flag */
        XPD_ADC_ClearFlag(hadc, OVR);

        /* Error callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Error, hadc);
    }
#endif
}

/**
 * @brief Sets up and enables a DMA transfer for the ADC regular conversions.
 * @param hadc: pointer to the ADC handle structure
 * @param Address: memory address to the conversion data storage
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType XPD_ADC_Start_DMA(ADC_HandleType * hadc, void * Address)
{
    XPD_ReturnType result;

    /* Set up DMA for transfer */
    result = XPD_DMA_Start_IT(hadc->DMA.Conversion, (void *)&hadc->Inst->DR, Address, hadc->Inst->SQR1.b.L + 1);

    /* If the DMA is currently used, return with error */
    if (result == XPD_OK)
    {
        /* Set the callback owner */
        hadc->DMA.Conversion->Owner = hadc;

        /* Set the DMA transfer callbacks */
        hadc->DMA.Conversion->Callbacks.Complete     = adc_dmaConversionRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
        hadc->DMA.Conversion->Callbacks.Error        = adc_dmaErrorRedirect;

        /* Enable ADC overrun interrupt */
        XPD_ADC_EnableIT(hadc, OVR);
#endif

        /* Enable ADC DMA mode */
        ADC_REG_BIT(hadc, CR2, DMA) = 1;

        XPD_ADC_Start(hadc);
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

    /* Disable the selected ADC DMA mode */
    ADC_REG_BIT(hadc, CR2, DMA) = 0;

    /* Disable the ADC DMA Stream */
    XPD_DMA_Stop_IT(hadc->DMA.Conversion);
}

/**
 * @brief Initializes the analog watchdog using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Channel: the ADC channel to monitor (only used when single channel monitoring is configured)
 * @param Config: pointer to analog watchdog setup configuration
 */
void XPD_ADC_WatchdogConfig(ADC_HandleType * hadc, uint8_t Channel, const ADC_WatchdogInitType * Config)
{
    /* set the analog watchdog mode */
    MODIFY_REG(hadc->Inst->CR1.w, (ADC_CR1_AWDSGL | ADC_CR1_JAWDEN | ADC_CR1_AWDEN), Config->Mode);

    /* Watchdog activation */
    if (Config->Mode != ADC_WATCHDOG_NONE)
    {
        /* set thresholds */
        hadc->Inst->HTR = Config->Threshold.High;
        hadc->Inst->LTR = Config->Threshold.Low;

        /* select the analog watchdog channel */
        hadc->Inst->CR1.b.AWDCH = Channel;

        XPD_ADC_EnableIT(hadc, AWD);
    }
    else
    {
        /* Watchdog deactivation */
        XPD_ADC_DisableIT(hadc, AWD);
    }
}

/**
 * @brief Returns the currently active analog watchdog.
 * @param hadc: pointer to the ADC handle structure
 * @return Set if the watchdog is active, 0 otherwise
 */
uint8_t XPD_ADC_WatchdogStatus(ADC_HandleType * hadc)
{
    return XPD_ADC_GetFlag(hadc, AWD);
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
    /* sample time configuration */
    adc_sampleTimeConfig(hadc, Config->Channel, Config->SampleTime);

    hadc->Inst->JSQR.b.JL = Config->ConversionCount - 1;

    /* only one of these shall be enabled at a time */
    ADC_REG_BIT(hadc, CR1, JAUTO)   = Config->AutoInjection;
    ADC_REG_BIT(hadc, CR1, JDISCEN) = Config->DiscontinuousMode;

    (&hadc->Inst->JOFR1)[Config->Rank - 1] = Config->Offset;

    /* channel configuration for a given rank */
    {
        uint32_t channelOffset = ((Config->Rank + 3) * 5) - Config->ConversionCount;
        uint32_t channelMask   = ADC_JSQR_JSQ1 << channelOffset;
        uint32_t channelVal    = (uint32_t)((uint8_t)Config->Channel) << channelOffset;

        /* Set the SQx bits for the selected rank */
        MODIFY_REG(hadc->Inst->JSQR.w, channelMask, channelVal);
    }

    /* if the trigger is software, or the injected channel is converted automatically after regular */
    if (   (Config->Trigger.InjSource == ADC_INJTRIGGER_SOFTWARE)
        || (Config->AutoInjection == ENABLE))
    {
        /* reset the external trigger */
        CLEAR_BIT(hadc->Inst->CR2.w, ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    }
    else
    {
        /* external trigger config */
        hadc->Inst->CR2.b.JEXTSEL = Config->Trigger.InjSource;
        hadc->Inst->CR2.b.JEXTEN  = Config->Trigger.Edge;
    }

    if (hadc->Inst == ADC1)
    {
        /* if ADC1 Channel_18 is selected enable VBAT Channel */
        if (Config->Channel == ADC_VBAT_CHANNEL)
        {
            /* Enable the VBAT channel*/
            ADC_COMMON_REG_BIT(CCR, VBATE) = 1;
        }
        /* if ADC1 Channel_16 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT) */
        else if (((Config->Channel == ADC_TEMPSENSOR_CHANNEL) || (Config->Channel == ADC_VREFINT_CHANNEL))
                && (ADC_COMMON_REG_BIT(CCR,TSVREFE) == 0))
        {
            /* Enable the TSVREFE channel*/
            ADC_COMMON_REG_BIT(CCR,TSVREFE) = 1;

            if (Config->Channel == ADC_TEMPSENSOR_CHANNEL)
            {
                /* Delay for temperature sensor stabilization time */
                /* Compute number of CPU cycles to wait for */
                XPD_Delay_us(ADC_TEMPSENSOR_DELAY_US);
            }
        }
    }
}

/**
 * @brief Enables the ADC peripheral and starts injected conversion if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Start(ADC_HandleType * hadc)
{
    /* Check if ADC peripheral is disabled in order to enable it and wait during
     Tstab time the ADC's stabilization */
    if (ADC_REG_BIT(hadc,CR2,ADON) == 0)
    {
        /* Enable the Peripheral */
        ADC_REG_BIT(hadc,CR2,ADON) = 1;

        /* Delay for ADC stabilization time */
        /* Compute number of CPU cycles to wait for */
        XPD_Delay_us(ADC_STAB_DELAY_US);
    }

    /* if no external trigger present and ADC is either not in multimode, or the multimode master,
     * enable software conversion of injected channels */
    if (    ( hadc->Inst->CR2.b.JEXTEN       == 0)
         && ( ADC_REG_BIT(hadc, CR1, JAUTO)  == 0)
         && ((ADC_COMMON_REG_BIT(CCR, MULTI) == DISABLE) || (hadc->Inst == ADC1)))
    {
        ADC_REG_BIT(hadc,CR2,JSWSTART) = 1;
    }
}

/**
 * @brief Enables the ADC peripheral and interrupts, and starts injected conversion if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Start_IT(ADC_HandleType * hadc)
{
#ifdef USE_XPD_ADC_ERROR_DETECT
    XPD_ADC_EnableIT(hadc, OVR);
#endif
    XPD_ADC_EnableIT(hadc, JEOC);

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

/** @addtogroup ADC_MultiMode
 * @{ */

/** @defgroup ADC_MultiMode_Exported_Functions Multi ADC Mode Exported Functions
 * @{ */

/**
 * @brief Initializes a multi ADC configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to the multi ADC setup configuration
 */
void XPD_ADC_MultiMode_Init(ADC_HandleType * hadc, const ADC_MultiMode_InitType * Config)
{
    ADC->CCR.b.MULTI = Config->Mode;
    ADC->CCR.b.DMA   = Config->DMAAccessMode;
    ADC->CCR.b.DELAY = Config->InterSamplingDelay - 5;
}

/**
 * @brief Sets up and enables a DMA transfer for the multi ADC regular conversions.
 * @param hadc: pointer to the ADC handle structure
 * @param Address: memory address to the conversion data storage
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType XPD_ADC_MultiMode_Start_DMA(ADC_HandleType * hadc, void * Address)
{
    XPD_ReturnType result;

    /* Set up DMA for transfer */
    result = XPD_DMA_Start_IT(hadc->DMA.Conversion, (void *)&ADC->CDR.w, Address, hadc->Inst->SQR1.b.L + 1);

    /* If the DMA is currently used, return with error */
    if (result == XPD_OK)
    {
        /* Set the callback owner */
        hadc->DMA.Conversion->Owner = hadc;

        /* Set the DMA transfer callbacks */
        hadc->DMA.Conversion->Callbacks.Complete     = adc_dmaConversionRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
        hadc->DMA.Conversion->Callbacks.Error        = adc_dmaErrorRedirect;

        /* Enable ADC overrun interrupt */
        XPD_ADC_EnableIT(hadc, OVR);
#endif

        /* pass the continuous DMA request setting to the common config */
        ADC_COMMON_REG_BIT(CCR,DDS) = ADC_REG_BIT(hadc, CR2, DDS);

        /* Enable ADC DMA mode */
        ADC_REG_BIT(hadc, CR2, DMA) = 1;

        XPD_ADC_Start(hadc);
    }
    return result;
}

/**
 * @brief Disables the ADC and the common DMA transfer.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_MultiMode_Stop_DMA(ADC_HandleType * hadc)
{
    /* disable the selected ADC DMA request after last transfer */
    ADC_COMMON_REG_BIT(CCR,DDS) = 0;

    XPD_ADC_Stop_DMA(hadc);
}

/**
 * @brief Return the result of the last common ADC regular conversions.
 * @param hadc: pointer to the ADC handle structure
 * @return A pair of conversion results in a single word
 */
uint32_t XPD_ADC_MultiMode_GetValues(ADC_HandleType * hadc)
{
    /* return the multi mode conversion values */
    return ADC->CDR.w;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_ADC */
