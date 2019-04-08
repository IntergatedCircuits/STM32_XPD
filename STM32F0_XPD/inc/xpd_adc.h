/**
  ******************************************************************************
  * @file    xpd_adc.h
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
#ifndef __XPD_ADC_H_
#define __XPD_ADC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_adc_calc.h>
#include <xpd_dma.h>

/** @defgroup ADC
 * @{ */

/** @defgroup ADC_Core ADC Core
 * @{ */

/** @defgroup ADC_Core_Exported_Types ADC Core Exported Types
 * @{ */

/** @brief ADC error types */
typedef enum
{
    ADC_ERROR_NONE      = 0,   /*!< No error */
    ADC_ERROR_OVERRUN   = 1,   /*!< Overrun flag */
    ADC_ERROR_DMA       = 4,   /*!< DMA transfer error */
}ADC_ErrorType;

/** @brief ADC sample times */
typedef enum
{
    ADC_SAMPLETIME_1p5   = 0, /*!< Sampling during 1.5 clock cycles */
    ADC_SAMPLETIME_7p5   = 1, /*!< Sampling during 2.5 clock cycles */
    ADC_SAMPLETIME_13p5  = 2, /*!< Sampling during 13.5 clock cycles */
    ADC_SAMPLETIME_28p5  = 3, /*!< Sampling during 28.5 clock cycles */
    ADC_SAMPLETIME_41p5  = 4, /*!< Sampling during 41.5 clock cycles */
    ADC_SAMPLETIME_55p5  = 5, /*!< Sampling during 55.5 clock cycles */
    ADC_SAMPLETIME_71p5  = 6, /*!< Sampling during 71.5 clock cycles */
    ADC_SAMPLETIME_239p5 = 7  /*!< Sampling during 239.5 clock cycles */
}ADC_SampleTimeType;

/** @brief ADC conversion resolution */
typedef enum
{
    ADC_RESOLUTION_12BIT = 0, /*!< 12 bit resolution */
    ADC_RESOLUTION_10BIT = 1, /*!< 10 bit resolution */
    ADC_RESOLUTION_8BIT  = 2, /*!< 8 bit resolution */
    ADC_RESOLUTION_6BIT  = 3  /*!< 6 bit resolution */
}ADC_ResolutionType;

/** @brief ADC trigger sources */
typedef enum
{
    ADC_TRIGGER_TIM1_TRGO  = 0,  /*!< TIM1 Trigger Out */
    ADC_TRIGGER_TIM1_CC4   = 1,  /*!< TIM1 Channel 4 */
    ADC_TRIGGER_TIM2_TRGO  = 2,  /*!< TIM2 Trigger Out */
    ADC_TRIGGER_TIM3_TRGO  = 3,  /*!< TIM3 Trigger Out */
    ADC_TRIGGER_TIM15_TRGO = 4,  /*!< TIM15 Trigger Out */
}ADC_TriggerSourceType;

/** @brief ADC End of Conversion flag mode */
typedef enum
{
    ADC_EOC_SEQUENCE = 0, /*!< End of Conversion flag is set at the end of a sequence */
    ADC_EOC_SINGLE   = 1  /*!< End of Conversion flag is set at the end of each individual conversion */
}ADC_EOCSelectType;

/** @brief ADC operation types */
typedef enum
{
    ADC_OPERATION_CONVERSION    = ADC_ISR_EOS | ADC_ISR_EOC,    /*!< Regular conversion */
    ADC_OPERATION_WATCHDOG1     = ADC_ISR_AWD1,                 /*!< Analog watchdog 1 */
    ADC_OPERATION_OVERRUN       = ADC_ISR_OVR                   /*!< Overrun */
}ADC_OperationType;

/** @brief ADC regular group scan direction */
typedef enum
{
    ADC_SCAN_FORWARD  = 0, /*!< Active ADC channels are converted starting from number 0 */
    ADC_SCAN_BACKWARD = 1  /*!< Active ADC channels are converted starting from number 18 */
}ADC_ScanDirectionType;

/** @brief ADC setup structure */
typedef struct
{
    union {
    struct {
    uint32_t : 1;
    FunctionalState       ContinuousDMARequests : 1;/*!< Continuous DMA requests, or only for a single EOC flag */
    ADC_ScanDirectionType ScanDirection : 2;        /*!< Defines if channels are converted from channels 0 through 18 or in reverse order */
    ADC_ResolutionType    Resolution : 2;           /*!< A/D conversion resolution */
    FunctionalState       LeftAlignment : 1;        /*!< ENABLE to left-align converted data, otherwise DISABLE */
    ADC_TriggerSourceType TriggerSource : 4;        /*!< Source of the conversion trigger */
    EdgeType              TriggerEdge : 2;          /*!< Trigger edges that initiate conversion */
    FunctionalState       Overrun : 1;              /*!< Enables overwriting the data register with the latest conversion */
    FunctionalState       ContinuousMode : 1;       /*!< Continuous or single mode */
    FunctionalState       LPAutoWait : 1;           /*!< When enabled, new conversion starts only after the user has handled the current conversion. */
    FunctionalState       LPAutoPowerOff : 1;       /*!< When enabled, the ADC automatically powers-off after a conversion
                                                         and automatically starts up when a new conversion is triggered.
                                                         @note: This feature also turns off the ADC dedicated 14 MHz RC oscillator (HSI14) */
    uint32_t              DiscontinuousCount : 1;   /*!< If 1, a single channel is converted on each trigger in the sequence */
    uint32_t : 14;
    ADC_EOCSelectType     EndFlagSelection : 1;     /*!< Specifies when the EOC flag is set and the conversions stop */
    };
    uint32_t w;
    };
}ADC_InitType;

/** @brief ADC analog watchdog selection */
typedef enum
{
    ADC_AWD_NONE = 0, /*!< No watchdog is used */
    ADC_AWD1     = 1, /*!< AWD1 default watchdog selection
                           @note This watchdog can only monitor a single channel, or whole conversion group(s) */
}ADC_WatchdogType;

/** @brief ADC channel setup structure */
typedef struct
{
    uint8_t            Number;       /*!< Number of the ADC channel [0..18] */
    ADC_SampleTimeType SampleTime;   /*!< Common sample time of all ADC channels */
    ADC_WatchdogType   Watchdog;     /*!< Channel monitoring watchdog selection */
}ADC_ChannelInitType;

/** @brief ADC Handle structure */
typedef struct
{
    ADC_TypeDef * Inst;                             /*!< The address of the peripheral instance used by the handle */
#ifdef ADC_BB
    ADC_BitBand_TypeDef * Inst_BB;                  /*!< The address of the peripheral instance in the bit-band region */
#endif
    struct {
        XPD_HandleCallbackType DepInit;             /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;           /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType ConvComplete;        /*!< Conversion(s) complete callback */
        XPD_HandleCallbackType Watchdog;            /*!< Watchdog alert callback */
#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
        XPD_HandleCallbackType Error;               /*!< DMA transfer or overrun error callback */
#endif
    }Callbacks;                                     /*   Handle Callbacks */
    struct {
        DMA_HandleType * Conversion;                /*!< DMA handle for update transfer */
    }DMA;                                           /*   DMA handle references */
    void * Trigger;                                 /*!< Conversion trigger's peripheral handle */
    uint8_t ConversionCount;                        /*!< ADC number of regular conversions */
    uint8_t EndFlagSelection;                       /*!< [Internal] Stores the EOC configuration */
#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
    volatile ADC_ErrorType Errors;                  /*!< Conversion errors */
#endif
}ADC_HandleType;

/** @} */

/** @defgroup ADC_Core_Exported_Macros ADC Core Exported Macros
 * @{ */

/** @brief Number of ADC peripherals */
#define         ADC_COUNT           1

/**
 * @brief  The index of the ADC peripheral managed by the handle.
 * @param  HANDLE: specifies the peripheral handle.
 */
#define         ADC_INDEX(HANDLE)   \
    0

/**
 * @brief  The common ADC registers related to the handle.
 * @param  HANDLE: specifies the peripheral handle.
 */
#define         ADC_COMMON(HANDLE)  \
    (ADC1_COMMON)

#ifdef ADC_BB
/**
 * @brief ADC Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the ADC peripheral instance.
 */
#define         ADC_INST2HANDLE(HANDLE,INSTANCE)                \
    ((HANDLE)->Inst    = (INSTANCE),                            \
     (HANDLE)->Inst_BB = ADC_BB(INSTANCE))

/**
 * @brief ADC register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         ADC_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

/**
 * @brief ADC common register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         ADC_COMMON_REG_BIT(HANDLE, REG_NAME, BIT_NAME)  \
    (ADC_COMMON_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief ADC Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the ADC peripheral instance.
 */
#define         ADC_INST2HANDLE(HANDLE,INSTANCE)                \
    ((HANDLE)->Inst = (INSTANCE))

/**
 * @brief ADC register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         ADC_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)

/**
 * @brief ADC common register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         ADC_COMMON_REG_BIT(HANDLE, REG_NAME, BIT_NAME)  \
    (ADC_COMMON(HANDLE)->REG_NAME.b.BIT_NAME)

#endif /* ADC_BB */

/**
 * @brief  Enable the specified ADC interrupt.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg ADRDY:   A/D converter ready
 *            @arg EOSMP:   End of sampling
 *            @arg EOC:     End of regular conversion
 *            @arg EOS:     End of regular sequence
 *            @arg OVR:     Overrun
 *            @arg AWD1:    Analog watchdog 1
 */
#define         ADC_IT_ENABLE(  HANDLE,  IT_NAME)               \
    (ADC_REG_BIT((HANDLE),IER,IT_NAME ## IE) = 1)

/**
 * @brief  Disable the specified ADC interrupt.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg ADRDY:   A/D converter ready
 *            @arg EOSMP:   End of sampling
 *            @arg EOC:     End of regular conversion
 *            @arg EOS:     End of regular sequence
 *            @arg OVR:     Overrun
 *            @arg AWD1:    Analog watchdog 1
 */
#define         ADC_IT_DISABLE( HANDLE,  IT_NAME)               \
    (ADC_REG_BIT((HANDLE),IER,IT_NAME ## IE) = 0)

/**
 * @brief  Get the specified ADC flag.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg ADRDY:   A/D converter ready
 *            @arg EOSMP:   End of sampling
 *            @arg EOC:     End of regular conversion
 *            @arg EOS:     End of regular sequence
 *            @arg OVR:     Overrun
 *            @arg AWD1:    Analog watchdog 1
 */
#define         ADC_FLAG_STATUS(HANDLE, FLAG_NAME)              \
    (ADC_REG_BIT((HANDLE),ISR,FLAG_NAME))

/**
 * @brief  Clear the specified ADC flag.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg ADRDY:   A/D converter ready
 *            @arg EOSMP:   End of sampling
 *            @arg EOC:     End of regular conversion
 *            @arg EOS:     End of regular sequence
 *            @arg OVR:     Overrun
 *            @arg AWD1:    Analog watchdog 1
 */
#define         ADC_FLAG_CLEAR(HANDLE, FLAG_NAME)               \
    ((HANDLE)->Inst->ISR.w = ADC_ISR_##FLAG_NAME)

/* Compatibility macros */
#define ADC_WATCHDOG_CHANNEL        (ADC_WATCHDOG_REG_CHANNEL)
#define ADC_WATCHDOG_ALL_CHANNELS   (ADC_WATCHDOG_REG_GROUP)

/** @} */

/** @addtogroup ADC_Core_Exported_Functions
 * @{ */
void            ADC_vInit               (ADC_HandleType * pxADC, const ADC_InitType * pxConfig);
void            ADC_vDeinit             (ADC_HandleType * pxADC);
void            ADC_vChannelConfig      (ADC_HandleType * pxADC, const ADC_ChannelInitType axChannels[],
                                         uint8_t ucChannelCount);

void            ADC_vStart              (ADC_HandleType * pxADC);
void            ADC_vStop               (ADC_HandleType * pxADC);
XPD_ReturnType  ADC_ePollStatus         (ADC_HandleType * pxADC, ADC_OperationType eOperation,
                                         uint32_t ulTimeout);

void            ADC_vStart_IT           (ADC_HandleType * pxADC);
void            ADC_vStop_IT            (ADC_HandleType * pxADC);
void            ADC_vIRQHandler         (ADC_HandleType * pxADC);

XPD_ReturnType  ADC_eStart_DMA          (ADC_HandleType * pxADC, void * pvAddress);
void            ADC_vStop_DMA           (ADC_HandleType * pxADC);

void            ADC_vWatchdogConfig     (ADC_HandleType * pxADC, ADC_WatchdogType eWatchdog,
                                         uint16_t usLowThd, uint16_t usHighThd);
ADC_WatchdogType ADC_eWatchdogStatus    (ADC_HandleType * pxADC);

/**
 * @brief Return the result of the last ADC regular conversion.
 * @param pxADC: pointer to the ADC handle structure
 * @return The conversion result
 */
__STATIC_INLINE uint16_t ADC_usGetValue(ADC_HandleType * pxADC)
{
    return (uint16_t)pxADC->Inst->DR;
}

/** @} */

/** @} */

/** @defgroup ADC_Calibration ADC Calibration
 * @{ */

/** @addtogroup ADC_Calibration_Exported_Functions
 * @{ */
XPD_ReturnType  ADC_eCalibrate          (ADC_HandleType * pxADC, bool eDifferential);
/** @} */

/** @} */

/** @} */

#define XPD_ADC_API
#include <xpd_rcc_pc.h>
#undef XPD_ADC_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_ADC_H_ */
