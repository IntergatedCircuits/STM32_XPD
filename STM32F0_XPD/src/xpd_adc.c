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

static void adc_clockCtrl(ADC_HandleType * hadc, FunctionalState ClockState)
{
    XPD_ADC1_ClockCtrl(ClockState);
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

    /* Update error code */
    hadc->Errors |= ADC_ERROR_DMA;

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

/* Enables an internal channel for conversion */
static void adc_initInternalChannel(ADC_HandleType * hadc, uint8_t Channel)
{
    uint32_t chbit = 0;

    /* Map channel to enable bit */
    switch (Channel)
    {
#ifdef ADC1_VBAT_CHANNEL
        case ADC1_VBAT_CHANNEL:
            /* enable the VBAT input */
            { chbit = ADC_CCR_VBATEN; }
            break;
#endif
        case ADC1_VREFINT_CHANNEL:
            /* enable the VREF input */
            { chbit = ADC_CCR_VREFEN; }
            break;

        case ADC1_TEMPSENSOR_CHANNEL:
            /* enable the TS input */
            { chbit = ADC_CCR_TSEN; }
            break;

        default:
            break;
    }

    /* Check if setting is valid */
    if ((chbit != 0) && ((ADC->CCR.w & chbit) == 0))
    {
        SET_BIT(ADC->CCR.w, chbit);

        if (Channel == ADC1_TEMPSENSOR_CHANNEL)
        {
            /* wait temperature sensor stabilization */
            XPD_Delay_us(ADC_TEMPSENSOR_DELAY_US);
        }
    }
}

/* Stops ongoing conversions */
static void adc_stopConversion(ADC_HandleType * hadc, uint32_t convFlag)
{
    uint32_t timeout = ADC_STOP_CONVERSION_TIMEOUT;

    /* Verification if ADC is not already stopped (on regular and injected
     * groups) to bypass this function if not needed. */
    if ((hadc->Inst->CR.w & ADC_STARTCTRL) != 0)
    {
        {
            /* Software is allowed to set ADSTP only when ADSTART=1 and ADDIS=0 */
            if ((hadc->Inst->CR.w & (ADC_CR_ADSTART | ADC_CR_ADDIS)) == ADC_CR_ADSTART)
            {
                /* Stop conversions on regular group */
                ADC_REG_BIT(hadc, CR, ADSTP) = 1;
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
XPD_ReturnType XPD_ADC_Init(ADC_HandleType * hadc, const ADC_InitType * Config)
{
    XPD_ReturnType result = XPD_ERROR;

    /* enable clock */
    adc_clockCtrl(hadc, ENABLE);

    hadc->ConversionCount = 0;

    /* Set used EndFlag */
    if (Config->EndFlagSelection == ADC_EOC_SEQUENCE)
    {
        hadc->EndFlagSelection = ADC_ENDFLAG_SEQUENCE;
    }
    else
    {
        hadc->EndFlagSelection = ADC_ENDFLAG_CONVERSION;
    }

    /* Configuration of ADC parameters if previous preliminary actions are
     * correctly completed and if there is no conversion ongoing on regular group */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        /* Configuration of ADC */
        if (ADC_REG_BIT(hadc, CR, ADEN) == 0)
        {
            hadc->Inst->CFGR1.b.RES       = Config->Resolution;
        }
        ADC_REG_BIT(hadc, CFGR1, CONT)    = Config->ContinuousMode;
        ADC_REG_BIT(hadc, CFGR1, ALIGN)   = Config->LeftAlignment;
        ADC_REG_BIT(hadc, CFGR1, SCANDIR) = Config->ScanDirection;
        ADC_REG_BIT(hadc, CFGR1, AUTOFF)  = Config->LPAutoPowerOff;
        ADC_REG_BIT(hadc, CFGR1, WAIT)    = Config->LPAutoWait;
        ADC_REG_BIT(hadc, CFGR1, DMACFG)  = Config->ContinuousDMARequests;

        /* external trigger configuration */
        if (Config->Trigger.Source == ADC_TRIGGER_SOFTWARE)
        {
            /* reset the external trigger */
            CLEAR_BIT(hadc->Inst->CFGR1.w, ADC_CFGR1_EXTSEL | ADC_CFGR1_EXTEN);
        }
        else
        {
            /* select external trigger and polarity to start conversion */
            hadc->Inst->CFGR1.b.EXTSEL = Config->Trigger.Source;
            hadc->Inst->CFGR1.b.EXTEN  = Config->Trigger.Edge;
        }

        /* Enable discontinuous mode only if continuous mode is disabled */
        if ((Config->DiscontinuousCount != 0) && (Config->ContinuousMode == DISABLE))
        {
            ADC_REG_BIT(hadc, CFGR1, DISCEN) = 1;
        }
        else
        {
            ADC_REG_BIT(hadc, CFGR1, DISCEN) = 0;
        }

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

    /* Disable the ADC peripheral */
    adcx_disable(hadc->Inst);

    /* Reset register IER */
    hadc->Inst->IER.w = 0;

    /* Reset register ISR */
    hadc->Inst->ISR.w = ~0;

    /* Reset register CFGR1 */
    hadc->Inst->CFGR1.w = 0;

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
 *
 * @note  This device line does not support channel ordering, the conversion scan goes through the
 *        activated channels either in ascending or descending order, depending on ScanDirection.
 * @note  This device line does not support channel-wise sample time configuration,
 *        simply the first element's setting will be applied for all channels.
 */
void XPD_ADC_ChannelConfig(ADC_HandleType * hadc, const ADC_ChannelInitType * Channels,
        uint8_t ChannelCount)
{
    /* No ongoing conversion on regular group */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        uint8_t i;
        uint8_t wdgUsers = 0;

        /* Set common sampling time */
        hadc->Inst->SMPR.b.SMP = Channels[0].SampleTime;

        /* Initially reset channel selection */
        hadc->Inst->CHSELR = 0;

        /* Set each active channel's bit */
        for (i = 0; i < ChannelCount; i++)
        {
            SET_BIT(hadc->Inst->CHSELR, 1 << Channels[i].Number);

            /* Internal channel configuration (if applicable) */
            adc_initInternalChannel(hadc, Channels[i].Number);

            /* If a watchdog is used for the channel, set it in the configuration */
            if (Channels[i].Watchdog != ADC_AWD_NONE)
            {
                hadc->Inst->CFGR1.b.AWD1CH = Channels[i].Number;
                wdgUsers++;
            }
        }

        /* Set conversion count */
        hadc->ConversionCount = ChannelCount;

        /* Determine watchdog configuration based on user count */
        switch (wdgUsers)
        {
            case 0:
                CLEAR_BIT(hadc->Inst->CFGR1.w,
                        ADC_CFGR1_AWD1SGL | ADC_CFGR1_AWD1EN);
                break;

            case 1:
                SET_BIT(hadc->Inst->CFGR1.w,
                        ADC_CFGR1_AWD1SGL | ADC_CFGR1_AWD1EN);
                break;

            default:
                /* All regular channels will be monitored,
                 * even if they were not configured so! */
                MODIFY_REG(hadc->Inst->CFGR1.w,
                        ADC_CFGR1_AWD1SGL | ADC_CFGR1_AWD1EN, ADC_CFGR1_AWD1EN);
                break;
        }
    }
}

/**
 * @brief Enables the ADC peripheral and starts conversion if the trigger is software.
 * @param hadc: pointer to the ADC handle structure
 */
void XPD_ADC_Start(ADC_HandleType * hadc)
{
    /* If low power mode AutoPowerOff is enabled,
     * power-on/off phases are performed automatically by hardware */
    if (ADC_REG_BIT(hadc, CR, ADSTART) == 0)
    {
        /* ADC turn ON */
        if ((ADC_REG_BIT(hadc, CFGR1, AUTOFF) == 0) && adcx_enable(hadc->Inst))
        {
            /* Wait until ADRDY flag is set ( < 1us) */
            uint32_t timeout = 1;
            XPD_WaitForMatch(&hadc->Inst->ISR.w, ADC_ISR_ADRDY, ADC_ISR_ADRDY, &timeout);
        }

        /* clear regular group conversion flag and overrun flag */
        SET_BIT(hadc->Inst->ISR.w, ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);

        /* if ADC is either not in multimode, or the multimode master,
         * enable software conversion of regular channels */
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

    /* 1. Stop potential ongoing conversions */
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
            && ((hadc->EndFlagSelection & ADC_IER_EOSIE) != 0))
    {
        Operation = ADC_ISR_EOS;
    }

    /* Polling for single conversion is not allowed when DMA is used,
     * and EOC is raised on end of single conversion */
    if (Operation == ADC_OPERATION_CONVERSION)
    {
        if (ADC_REG_BIT(hadc, CFGR1, DMAEN) == 1)
        { result = XPD_ERROR; }
    }

    /* Wait until operation flag is set */
    if (result == XPD_OK)
    {
        result = XPD_WaitForMatch(&hadc->Inst->ISR.w, Operation, Operation, &Timeout);

        /* Clear end of conversion flag of regular group if low power feature
         * "LowPowerAutoWait " is disabled, to not interfere with this feature
         * until data register is read. */
        if ((result == XPD_OK) && ((Operation & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
            && (ADC_REG_BIT(hadc, CFGR1, WAIT) == 0))
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

    /* End of conversion flag for regular channels */
    if (    ((isr & (ADC_ISR_EOC | ADC_ISR_EOS)) != 0)
         && ((ier & (ADC_IER_EOCIE | ADC_IER_EOSIE)) != 0))
    {
        uint32_t cfgr = hadc->Inst->CFGR1.w;

        /* If the conversion is not continuous / external triggered */
        if ((cfgr & (ADC_CFGR1_CONT | ADC_CFGR1_EXTEN)) == 0)
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

    /* Check analog watchdog flag */
    if (    ((isr & ADC_ISR_AWD1) != 0)
         && ((ier & ADC_IER_AWD1IE) != 0))
    {
        /* watchdog callback */
        XPD_SAFE_CALLBACK(hadc->Callbacks.Watchdog, hadc);

        /* clear the watchdog flag only after callback,
         * so watchdog status can be determined */
        XPD_ADC_ClearFlag(hadc, AWD1);
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

    {
        /* Set up DMA for transfer */
        result = XPD_DMA_Start_IT(hadc->DMA.Conversion,
                (void *)&hadc->Inst->DR, Address, hadc->ConversionCount);

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
            ADC_REG_BIT(hadc, CFGR1, DMAEN) = 1;

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
    ADC_REG_BIT(hadc, CFGR1, DMAEN) = 0;

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
    /* Configure when ADC is stopped */
    if ((Watchdog != ADC_AWD_NONE) && ((hadc->Inst->CR.w & ADC_STARTCTRL) == 0))
    {
        uint32_t scaling = hadc->Inst->CFGR1.b.RES * 2;

        /* Analog watchdogs configuration */
        {
            /* Shift the offset in function of the selected ADC resolution:
             * Thresholds have to be left-aligned on bit 11, the LSB (right bits)
             * are set to 0 */
            hadc->Inst->TR.w = (Config->High  << (16 + scaling))
                    | (Config->Low  << scaling);
        }
    }
}

/**
 * @brief Returns the currently active analog watchdog.
 * @param hadc: pointer to the ADC handle structure
 * @return Set if the watchdog is active, 0 otherwise
 */
ADC_WatchdogType XPD_ADC_WatchdogStatus(ADC_HandleType * hadc)
{
    return XPD_ADC_GetFlag(hadc, AWD1);
}

/** @} */

/** @} */

/** @addtogroup ADC_Calibration
 * @{ */

/** @defgroup ADC_Calibration_Exported_Functions ADC Calibration Exported Functions
 * @{ */

/**
 * @brief Executes an ADC self-calibration sequence.
 * @param hadc: pointer to the ADC handle structure
 * @param Differential: unused
 * @return Result of the operation
 */
XPD_ReturnType XPD_ADC_Calibrate(ADC_HandleType * hadc, boolean_t Differential)
{
    XPD_ReturnType result = XPD_ERROR;

    /* Calibration prerequisite: ADC must be disabled. */
    if (ADC_REG_BIT(hadc, CR, ADEN) == 0)
    {
        uint32_t timeout = ADC_CALIBRATION_TIMEOUT;

        /* Start ADC calibration */
        ADC_REG_BIT(hadc, CR, ADCAL) = 1;

        /* Wait for ADCAL to return to 0 */
        result = XPD_WaitForDiff(&hadc->Inst->CR.w,
                ADC_CR_ADCAL, ADC_CR_ADCAL, &timeout);
    }
    return result;
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_ADC */
