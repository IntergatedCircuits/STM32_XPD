/**
  ******************************************************************************
  * @file    xpd_adc.c
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2016-07-06
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
    ADC_COMMON()->CCR.b.ADCPRE = ClockSource;
}

/**
 * @brief Returns the input clock frequency of the ADCs.
 * @return The clock frequency of the ADCs in Hz
 */
uint32_t XPD_ADC_GetClockFreq(void)
{
    return XPD_RCC_GetClockFreq(PCLK2) / ((ADC_COMMON()->CCR.b.ADCPRE + 1) * 2);
}

/** @} */

/** @} */

/** @addtogroup ADC_Core
 * @{ */

#define ADC_STAB_DELAY_US         3
#define ADC_TEMPSENSOR_DELAY_US   10

#define ADC_SQR_MASK            (ADC_SQR3_SQ1 >> ADC_SQR3_SQ1_Pos)
#define ADC_SQR_SIZE            (ADC_SQR3_SQ2_Pos - ADC_SQR3_SQ1_Pos)

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

/* Sets the sample time for a channel configuration */
static void adc_sampleTimeConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Channel)
{
    uint32_t regoffset = 1, number = Channel->Number;

    /* Channel sampling time configuration */
    if (number > 10)
    {
        regoffset = 0;
        number -= 10;
    }
    number *= 3;

    /* set the sample time */
    MODIFY_REG((&hadc->Inst->SMPR1.w)[regoffset],
            ADC_SMPR2_SMP0 << number, Channel->SampleTime << number);
}

/* Enables an internal channel for conversion */
static void adc_initInternalChannel(ADC_HandleType * hadc, uint8_t Channel)
{
#if (ADC_COUNT > 1)
    /* Only ADC1 has access to internal measurement channels */
    if (hadc->Inst == ADC1)
#endif
    {
        /* if ADC1 channel 18 is selected, enable VBAT */
        if (Channel == ADC1_VBAT_CHANNEL)
        {
            /* enable the VBAT input */
            ADC_COMMON_REG_BIT(hadc,CCR,VBATE) = 1;
        }
        /* if ADC1 channel 16 or 17 is selected, enable Temperature sensor and VREFINT */
        else if ((Channel == ADC1_TEMPSENSOR_CHANNEL) || (Channel == ADC1_VREFINT_CHANNEL))
        {
            /* enable the TS-VREF input */
            ADC_COMMON_REG_BIT(hadc,CCR,TSVREFE) = 1;

            if (Channel == ADC1_TEMPSENSOR_CHANNEL)
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
    /* Enable clock */
    XPD_SAFE_CALLBACK(hadc->ClockCtrl, ENABLE);

    /* Apply configuration */
    ADC_REG_BIT(hadc,CR1,SCAN)  = Config->ScanMode;
    ADC_REG_BIT(hadc,CR2,ALIGN) = Config->LeftAlignment;
    ADC_REG_BIT(hadc,CR2,CONT)  = Config->ContinuousMode;
    ADC_REG_BIT(hadc,CR2,EOCS)  = Config->EndFlagSelection;
    ADC_REG_BIT(hadc,CR2,DDS)   = Config->ContinuousDMARequests;

    hadc->Inst->CR1.b.RES       = Config->Resolution;

    /* External trigger configuration */
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

    /* Discontinuous mode configuration */
    if (Config->DiscontinuousCount != 0)
    {
        ADC_REG_BIT(hadc,CR1,DISCEN) = 1;

        hadc->Inst->CR1.b.DISCNUM = Config->DiscontinuousCount - 1;
    }
    else
    {
        CLEAR_BIT(hadc->Inst->CR1.w, ADC_CR1_DISCEN | ADC_CR1_DISCNUM);
    }

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(hadc->Callbacks.DepInit, hadc);

    return XPD_OK;
}

/**
 * @brief Deinitializes the ADC peripheral.
 * @param hadc: pointer to the ADC handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_ADC_Deinit(ADC_HandleType * hadc)
{
    /* Turn off ADC */
    ADC_REG_BIT(hadc,CR2,ADON) = 0;

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(hadc->Callbacks.DepDeinit, hadc);

    /* Disable clock */
    XPD_SAFE_CALLBACK(hadc->ClockCtrl, DISABLE);

    return XPD_OK;
}

/**
 * @brief Initializes the regular ADC channels for conversion using the setup configuration.
 * @param hadc: pointer to the ADC handle structure
 * @param Channels: ADC regular channel configuration array pointer
 * @param ChannelCount: number of channels to configure
 */
void XPD_ADC_ChannelConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Channels, uint8_t ChannelCount)
{
    uint8_t i;

    /* Sequencer configuration */
    for (i = 0; (i < ChannelCount) && (i < 6); i++)
    {
        /* set the channel for the selected rank */
        uint32_t choffset = ADC_SQR3_SQ1_Pos + (ADC_SQR_SIZE * i);

        MODIFY_REG(hadc->Inst->SQR3.w,
                ADC_SQR_MASK << choffset, Channels[i].Number << choffset);
    }
    for (; (i < ChannelCount) && (i < 12); i++)
    {
        /* set the channel for the selected rank */
        uint32_t choffset = ADC_SQR_SIZE * (i - 6);

        MODIFY_REG(hadc->Inst->SQR2.w,
                ADC_SQR_MASK << choffset, Channels[i].Number << choffset);
    }
    for (;  i < ChannelCount; i++)
    {
        /* set the channel for the selected rank */
        uint32_t choffset = ADC_SQR_SIZE * (i - 12);

        MODIFY_REG(hadc->Inst->SQR1.w,
                ADC_SQR_MASK << choffset, Channels[i].Number << choffset);
    }

    for (i = 0; i < ChannelCount; i++)
    {
        /* Sample time configuration */
        adc_sampleTimeConfig(hadc, &Channels[i]);

        /* Internal channel configuration (if applicable) */
        adc_initInternalChannel(hadc, Channels[i].Number);
    }

    /* Set the number of conversions */
    hadc->Inst->SQR1.b.L = ChannelCount - 1;
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

    /* initialize number of remaining conversions count */
    hadc->ActiveConversions = hadc->Inst->SQR1.b.L + 1;

#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
    hadc->Errors = ADC_ERROR_NONE;
#endif

    /* if no external trigger present and ADC is either not in multimode, or the multimode master,
     * enable software conversion of regular channels */
    if (    (hadc->Inst->CR2.b.EXTEN == 0)
         && ((ADC_COMMON(hadc)->CCR.b.MULTI == 0) || (hadc->Inst == ADC1)))
    {
        ADC_REG_BIT(hadc,CR2,SWSTART) = 1;
    }
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
#endif

#ifdef USE_XPD_ADC_ERROR_DETECT
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
    /* Only one of these shall be enabled at a time */
    ADC_REG_BIT(hadc, CR1, JAUTO)   = Config->AutoInjection;
    ADC_REG_BIT(hadc, CR1, JDISCEN) = Config->DiscontinuousMode;

    /* If the trigger is software, or the injected channel is converted automatically after regular */
    if (   (Config->Trigger.InjSource == ADC_INJTRIGGER_SOFTWARE)
        || (Config->AutoInjection == ENABLE))
    {
        /* Reset the external trigger */
        CLEAR_BIT(hadc->Inst->CR2.w, ADC_CR2_JEXTSEL | ADC_CR2_JEXTEN);
    }
    else
    {
        /* External trigger config */
        hadc->Inst->CR2.b.JEXTSEL = Config->Trigger.InjSource;
        hadc->Inst->CR2.b.JEXTEN  = Config->Trigger.Edge;
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

    for (i = 0; i < ChannelCount; i++)
    {
        /* Channel configuration for a given rank */
        uint32_t choffset = ADC_JSQR_JSQ1_Pos + (5 * i);

        /* Set the SQx bits for the selected rank */
        MODIFY_REG(hadc->Inst->JSQR.w,
                0x1F << choffset, Channels[i].Number << choffset);

        /* Sample time configuration */
        adc_sampleTimeConfig(hadc, &Channels[i]);

        /* Offset configuration */
        (&hadc->Inst->JOFR1)[i] = Channels[i].Offset;

        /* Internal channel configuration (if applicable) */
        adc_initInternalChannel(hadc, Channels[i].Number);
    }

    /* Set the injected sequence length */
    hadc->Inst->JSQR.b.JL = ChannelCount - 1;
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

    /* If no external trigger present and ADC is either not in multimode, or the multimode master,
     * enable software conversion of injected channels */
    if (    ( hadc->Inst->CR2.b.JEXTEN       == 0)
         && ( ADC_REG_BIT(hadc, CR1, JAUTO)  == 0)
         && ((ADC_COMMON_REG_BIT(hadc,CCR,MULTI) == DISABLE) || (hadc->Inst == ADC1)))
    {
        ADC_REG_BIT(hadc,CR2,JSWSTART) = 1;
    }
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Stop(ADC_HandleType * hadc)
{
    /* Check against auto injection */
    if (ADC_REG_BIT(hadc,CR1,JAUTO) == 0)
    {
        /* Stop conversions on regular and injected groups */
        ADC_REG_BIT(hadc,CR2,ADON) = 0;
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

    XPD_ADC_Inject_Start(hadc);
}

/**
 * @brief Stop the conversion of injected channels by disabling ADC peripheral and interrupts.
 * @note  If auto-injection is used, the conversion can only be stopped along with the regular group.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Injected_Stop_IT(ADC_HandleType * hadc)
{
    XPD_ADC_Injected_Stop(hadc);

    XPD_ADC_DisableIT(hadc, JEOC);
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
 * @param hadc: pointer to the ADC handle structure
 * @param Config: pointer to the multi ADC setup configuration
 */
void XPD_ADC_MultiMode_Config(ADC_HandleType * hadc, const ADC_MultiMode_InitType * Config)
{
    ADC_COMMON(hadc)->CCR.b.MULTI = Config->Mode;
    ADC_COMMON(hadc)->CCR.b.DMA   = Config->DMAAccessMode;
    ADC_COMMON(hadc)->CCR.b.DELAY = Config->InterSamplingDelay - 5;
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

        /* pass the continuous DMA request setting to the common config */
        ADC_COMMON_REG_BIT(hadc,CCR,DDS) = ADC_REG_BIT(hadc, CR2, DDS);

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
    ADC_COMMON_REG_BIT(hadc,CCR,DDS) = 0;

    XPD_ADC_Stop_DMA(hadc);
}

/** @} */

/** @} */
#endif /* ADC123_COMMON */

/** @} */

#endif /* USE_XPD_ADC */
