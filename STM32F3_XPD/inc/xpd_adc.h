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
    ADC_ERROR_JQOVF     = 2,   /*!< Injected queue overflow flag */
    ADC_ERROR_DMA       = 4,   /*!< DMA transfer error */
}ADC_ErrorType;

/** @brief ADC sample times */
typedef enum
{
    ADC_SAMPLETIME_1p5   = 0, /*!< Sampling during 1.5 clock cycles */
    ADC_SAMPLETIME_2p5   = 1, /*!< Sampling during 2.5 clock cycles */
    ADC_SAMPLETIME_4p5   = 2, /*!< Sampling during 4.5 clock cycles */
    ADC_SAMPLETIME_7p5   = 3, /*!< Sampling during 7.5 clock cycles */
    ADC_SAMPLETIME_19p5  = 4, /*!< Sampling during 19.5 clock cycles */
    ADC_SAMPLETIME_61p5  = 5, /*!< Sampling during 61.5 clock cycles */
    ADC_SAMPLETIME_181p5 = 6, /*!< Sampling during 181.5 clock cycles */
    ADC_SAMPLETIME_601p5 = 7  /*!< Sampling during 601.5 clock cycles */
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
    /* Aliases for ADC1/2 */
    ADC_TRIGGER_TIM1_CC1   = 0,  /*!< TIM1 Channel 1 */
    ADC_TRIGGER_TIM1_CC2   = 1,  /*!< TIM1 Channel 2 */
    ADC_TRIGGER_TIM1_CC3   = 2,  /*!< TIM1 Channel 3 */
    ADC_TRIGGER_TIM2_CC2   = 3,  /*!< TIM2 Channel 2 */
    ADC_TRIGGER_TIM3_TRGO  = 4,  /*!< TIM3 Trigger Out */
    ADC_TRIGGER_TIM4_CC4   = 5,  /*!< TIM4 Channel 4 */
    ADC_TRIGGER_EXTI11     = 6,  /*!< EXTI Line 11 */
    ADC_TRIGGER_TIM8_TRGO  = 7,  /*!< TIM8 Trigger Out */
    ADC_TRIGGER_TIM8_TRGO2 = 8,  /*!< TIM8 Trigger Out 2 */
    ADC_TRIGGER_TIM1_TRGO  = 9,  /*!< TIM1 Trigger Out */
    ADC_TRIGGER_TIM1_TRGO2 = 10, /*!< TIM1 Trigger Out 2 */
    ADC_TRIGGER_TIM2_TRGO  = 11, /*!< TIM2 Trigger Out */
    ADC_TRIGGER_TIM4_TRGO  = 12, /*!< TIM4 Trigger Out */
    ADC_TRIGGER_TIM6_TRGO  = 13, /*!< TIM6 Trigger Out */
    ADC_TRIGGER_TIM15_TRGO = 14, /*!< TIM15 Trigger Out */
    ADC_TRIGGER_TIM3_CC4   = 15, /*!< TIM3 Channel 4 */

    ADC1_2_TRIGGER_TIM1_CC1   = 0,  /*!< TIM1 Channel 1 */
    ADC1_2_TRIGGER_TIM1_CC2   = 1,  /*!< TIM1 Channel 2 */
    ADC1_2_TRIGGER_TIM1_CC3   = 2,  /*!< TIM1 Channel 3 */
    ADC1_2_TRIGGER_TIM2_CC2   = 3,  /*!< TIM2 Channel 2 */
    ADC1_2_TRIGGER_TIM3_TRGO  = 4,  /*!< TIM3 Trigger Out */
    ADC1_2_TRIGGER_TIM4_CC4   = 5,  /*!< TIM4 Channel 4 */
    ADC1_2_TRIGGER_EXTI11     = 6,  /*!< EXTI Line 11 */
#ifdef HRTIM1
    ADC1_2_TRIGGER_HRTIM_TRG1 = 7,  /*!< HRTIM Trigger 1 */
    ADC1_2_TRIGGER_HRTIM_TRG3 = 8,  /*!< HRTIM Trigger 3 */
#else
    ADC1_2_TRIGGER_TIM8_TRGO  = 7,  /*!< TIM8 Trigger Out */
    ADC1_2_TRIGGER_TIM8_TRGO2 = 8,  /*!< TIM8 Trigger Out 2 */
#endif
    ADC1_2_TRIGGER_TIM1_TRGO  = 9,  /*!< TIM1 Trigger Out */
    ADC1_2_TRIGGER_TIM1_TRGO2 = 10, /*!< TIM1 Trigger Out 2 */
    ADC1_2_TRIGGER_TIM2_TRGO  = 11, /*!< TIM2 Trigger Out */
    ADC1_2_TRIGGER_TIM4_TRGO  = 12, /*!< TIM4 Trigger Out */
    ADC1_2_TRIGGER_TIM6_TRGO  = 13, /*!< TIM6 Trigger Out */
    ADC1_2_TRIGGER_TIM15_TRGO = 14, /*!< TIM15 Trigger Out */
    ADC1_2_TRIGGER_TIM3_CC4   = 15, /*!< TIM3 Channel 4 */

    ADC3_4_TRIGGER_TIM3_CC1   = 0,  /*!< TIM3 Channel 1 */
    ADC3_4_TRIGGER_TIM2_CC3   = 1,  /*!< TIM2 Channel 3 */
    ADC3_4_TRIGGER_TIM1_CC3   = 2,  /*!< TIM1 Channel 3 */
    ADC3_4_TRIGGER_TIM8_CC1   = 3,  /*!< TIM8 Channel 1 */
    ADC3_4_TRIGGER_TIM8_TRGO  = 4,  /*!< TIM8 Trigger Out */
    ADC3_4_TRIGGER_EXTI2      = 5,  /*!< EXTI Line 2 */
    ADC3_4_TRIGGER_TIM4_CC1   = 6,  /*!< TIM4 Channel 1 */
    ADC3_4_TRIGGER_TIM2_TRGO  = 7,  /*!< TIM2 Trigger Out */
    ADC3_4_TRIGGER_TIM8_TRGO2 = 8,  /*!< TIM8 Trigger Out 2 */
    ADC3_4_TRIGGER_TIM1_TRGO  = 9,  /*!< TIM1 Trigger Out */
    ADC3_4_TRIGGER_TIM1_TRGO2 = 10, /*!< TIM1 Trigger Out 2 */
    ADC3_4_TRIGGER_TIM3_TRGO  = 11, /*!< TIM3 Trigger Out */
    ADC3_4_TRIGGER_TIM4_TRGO  = 12, /*!< TIM4 Trigger Out */
    ADC3_4_TRIGGER_TIM7_TRGO  = 13, /*!< TIM7 Trigger Out */
    ADC3_4_TRIGGER_TIM15_TRGO = 14, /*!< TIM15 Trigger Out */
    ADC3_4_TRIGGER_TIM2_CC1   = 15, /*!< TIM2 Channel 1 */
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
    ADC_OPERATION_CONVERSION    = ADC_ISR_EOC,  /*!< Regular conversion */
    ADC_OPERATION_INJCONVERSION = ADC_ISR_JEOC, /*!< Injected conversion */
    ADC_OPERATION_WATCHDOG1     = ADC_ISR_AWD1, /*!< Analog watchdog 1 */
    ADC_OPERATION_WATCHDOG2     = ADC_ISR_AWD2, /*!< Analog watchdog 2 */
    ADC_OPERATION_WATCHDOG3     = ADC_ISR_AWD3, /*!< Analog watchdog 3 */
    ADC_OPERATION_OVERRUN       = ADC_ISR_OVR   /*!< Overrun */
}ADC_OperationType;

/** @brief ADC setup structure */
typedef struct
{
    union {
    struct {
    uint32_t : 1;
    FunctionalState       ContinuousDMARequests : 1;/*!< Continuous DMA requests, or only for a single EOC flag */
    uint32_t : 1;
    ADC_ResolutionType    Resolution : 2;           /*!< A/D conversion resolution */
    FunctionalState       LeftAlignment : 1;        /*!< ENABLE to left-align converted data, otherwise DISABLE */
    ADC_TriggerSourceType TriggerSource : 4;        /*!< Source of the conversion trigger */
    EdgeType              TriggerEdge : 2;          /*!< Trigger edges that initiate conversion */
    FunctionalState       Overrun : 1;              /*!< Enables overwriting the data register with the latest conversion */
    FunctionalState       ContinuousMode : 1;       /*!< Continuous or single mode */
    FunctionalState       LPAutoWait : 1;           /*!< When enabled, new conversion starts only after the user has handled the current conversion. */
    uint32_t : 1;
    uint32_t              DiscontinuousCount : 4;   /*!< If not 0, a subgroup of channels is converted
                                                         on each trigger in loop [0..8] */
    uint32_t : 10;
    ADC_EOCSelectType     EndFlagSelection : 1;     /*!< Specifies when the EOC flag is set and the conversions stop */
    FunctionalState       ScanMode : 1;             /*!< Scan mode converts all configured channels in sequence */
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
    ADC_AWD2     = 2, /*!< AWD2 channel-wise watchdog selection */
    ADC_AWD3     = 3, /*!< AWD3 channel-wise watchdog selection */
}ADC_WatchdogType;

/** @brief ADC channel setup structure */
typedef struct
{
    uint8_t            Number;       /*!< Number of the ADC channel [0..18] */
    ADC_SampleTimeType SampleTime;   /*!< Sample time of the channel */
    uint16_t           Offset;       /*!< Offset is subtracted after conversion of channel */
    ADC_WatchdogType   Watchdog;     /*!< Channel monitoring watchdog selection */
    FunctionalState    Differential; /*!< Channel is differential or single ended */
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
        XPD_HandleCallbackType InjConvComplete;     /*!< Injected conversion(s) complete callback */
        XPD_HandleCallbackType Watchdog;            /*!< Watchdog alert callback */
#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
        XPD_HandleCallbackType Error;               /*!< DMA transfer or overrun error callback */
#endif
    }Callbacks;                                     /*   Handle Callbacks */
    struct {
        DMA_HandleType * Conversion;                /*!< DMA handle for update transfer */
    }DMA;                                           /*   DMA handle references */
    void * Trigger;                                 /*!< Conversion trigger's peripheral handle */
    uint32_t OffsetUsage;                           /*!< [Internal] Bitflag for offset using channel numbers */
    uint8_t InjectedConfig;                         /*!< [Internal] The injected group settings */
    uint8_t ConversionCount;                        /*!< ADC number of regular conversions */
    uint8_t EndFlagSelection;                       /*!< [Internal] Stores the EOC configuration */
    volatile ADC_WatchdogType ActiveWatchdog;       /*!< [Internal] The currently active watchdog number */
#if defined(__XPD_ADC_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
    volatile ADC_ErrorType Errors;                  /*!< Conversion errors */
#endif
}ADC_HandleType;

/** @} */

/** @defgroup ADC_Core_Exported_Macros ADC Core Exported Macros
 * @{ */

#if defined(ADC34_COMMON)
/** @brief Number of ADC peripherals */
#define         ADC_COUNT           4

/**
 * @brief  The index of the ADC peripheral managed by the handle.
 * @param  HANDLE: specifies the peripheral handle.
 */
#define         ADC_INDEX(HANDLE)                   \
    ((((((uint32_t)(HANDLE)->Inst) > ADC2_BASE) ?   \
    (((uint32_t)(HANDLE)->Inst) - 0x200) :          \
     ((uint32_t)(HANDLE)->Inst)) >> 8) & 3)

/**
 * @brief  The common ADC registers related to the handle.
 * @param  HANDLE: specifies the peripheral handle.
 */
#define         ADC_COMMON(HANDLE)                  \
    ((ADC_Common_TypeDef *)(((uint32_t)(HANDLE)->Inst) | 0x300))

#elif defined(ADC12_COMMON)

/** @brief Number of ADC peripherals */
#define         ADC_COUNT           2

/**
 * @brief  The index of the ADC peripheral managed by the handle.
 * @param  HANDLE: specifies the peripheral handle.
 */
#define         ADC_INDEX(HANDLE)   \
    (((((uint32_t)(HANDLE)->Inst)) >> 8) & 3)

/**
 * @brief  The common ADC registers related to the handle.
 * @param  HANDLE: specifies the peripheral handle.
 */
#define         ADC_COMMON(HANDLE)  \
    (ADC12_COMMON)

#else
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

#endif /* ADC34_COMMON */
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
 *            @arg JEOC:    End of injected conversion
 *            @arg JEOS:    End of injected sequence
 *            @arg AWD1:    Analog watchdog 1
 *            @arg AWD2:    Analog watchdog 2
 *            @arg AWD3:    Analog watchdog 3
 *            @arg JQOVF:   Injected queue overflow
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
 *            @arg JEOC:    End of injected conversion
 *            @arg JEOS:    End of injected sequence
 *            @arg AWD1:    Analog watchdog 1
 *            @arg AWD2:    Analog watchdog 2
 *            @arg AWD3:    Analog watchdog 3
 *            @arg JQOVF:   Injected queue overflow
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
 *            @arg JEOC:    End of injected conversion
 *            @arg JEOS:    End of injected sequence
 *            @arg AWD1:    Analog watchdog 1
 *            @arg AWD2:    Analog watchdog 2
 *            @arg AWD3:    Analog watchdog 3
 *            @arg JQOVF:   Injected queue overflow
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
 *            @arg JEOC:    End of injected conversion
 *            @arg JEOS:    End of injected sequence
 *            @arg AWD1:    Analog watchdog 1
 *            @arg AWD2:    Analog watchdog 2
 *            @arg AWD3:    Analog watchdog 3
 *            @arg JQOVF:   Injected queue overflow
 */
#define         ADC_FLAG_CLEAR(HANDLE, FLAG_NAME)               \
    ((HANDLE)->Inst->ISR.w = ADC_ISR_##FLAG_NAME)

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

/** @defgroup ADC_Injected ADC Injected Conversions
 * @{ */

/** @defgroup ADC_Injected_Exported_Types ADC Injected Exported Types
 * @{ */

/** @brief ADC injected trigger sources */
typedef enum
{
    /* Aliases for ADC1/2 */
    ADC_INJTRIGGER_TIM1_TRGO  = 0,  /*!< TIM1 Trigger Out */
    ADC_INJTRIGGER_TIM1_CC4   = 1,  /*!< TIM1 Channel 4 */
    ADC_INJTRIGGER_TIM2_TRGO  = 2,  /*!< TIM2 Trigger Out */
    ADC_INJTRIGGER_TIM2_CC1   = 3,  /*!< TIM2 Channel 1 */
    ADC_INJTRIGGER_TIM3_CC4   = 4,  /*!< TIM3 Channel 4 */
    ADC_INJTRIGGER_TIM4_TRGO  = 5,  /*!< TIM4 Trigger Out */
    ADC_INJTRIGGER_EXTI15     = 6,  /*!< EXTI Line 15 */
    ADC_INJTRIGGER_TIM8_CC4   = 7,  /*!< TIM8 Channel 4 */
    ADC_INJTRIGGER_TIM1_TRGO2 = 8,  /*!< TIM1 Trigger Out 2 */
    ADC_INJTRIGGER_TIM8_TRGO  = 9,  /*!< TIM8 Trigger Out */
    ADC_INJTRIGGER_TIM8_TRGO2 = 10, /*!< TIM8 Trigger Out 2 */
    ADC_INJTRIGGER_TIM3_CC3   = 11, /*!< TIM3 Channel 3 */
    ADC_INJTRIGGER_TIM3_TRGO  = 12, /*!< TIM3 Trigger Out */
    ADC_INJTRIGGER_TIM3_CC1   = 13, /*!< TIM3 Channel 1 */
    ADC_INJTRIGGER_TIM6_TRGO  = 14, /*!< TIM6 Trigger Out */
    ADC_INJTRIGGER_TIM15_TRGO = 15, /*!< TIM15 Trigger Out */

    ADC1_2_INJTRIGGER_TIM1_TRGO  = 0,  /*!< TIM1 Trigger Out */
    ADC1_2_INJTRIGGER_TIM1_CC4   = 1,  /*!< TIM1 Channel 4 */
    ADC1_2_INJTRIGGER_TIM2_TRGO  = 2,  /*!< TIM2 Trigger Out */
    ADC1_2_INJTRIGGER_TIM2_CC1   = 3,  /*!< TIM2 Channel 1 */
    ADC1_2_INJTRIGGER_TIM3_CC4   = 4,  /*!< TIM3 Channel 4 */
    ADC1_2_INJTRIGGER_TIM4_TRGO  = 5,  /*!< TIM4 Trigger Out */
    ADC1_2_INJTRIGGER_EXTI15     = 6,  /*!< EXTI Line 15 */
    ADC1_2_INJTRIGGER_TIM8_CC4   = 7,  /*!< TIM8 Channel 4 */
    ADC1_2_INJTRIGGER_TIM1_TRGO2 = 8,  /*!< TIM1 Trigger Out 2 */
#ifdef HRTIM1
    ADC1_2_INJTRIGGER_HRTIM_TRG2 = 9,  /*!< HRTIM Trigger 2 */
    ADC1_2_INJTRIGGER_HRTIM_TRG4 = 10, /*!< HRTIM Trigger 4 */
#else
    ADC1_2_INJTRIGGER_TIM8_TRGO  = 9,  /*!< TIM8 Trigger Out */
    ADC1_2_INJTRIGGER_TIM8_TRGO2 = 10, /*!< TIM8 Trigger Out 2 */
#endif
    ADC1_2_INJTRIGGER_TIM3_CC3   = 11, /*!< TIM3 Channel 3 */
    ADC1_2_INJTRIGGER_TIM3_TRGO  = 12, /*!< TIM3 Trigger Out */
    ADC1_2_INJTRIGGER_TIM3_CC1   = 13, /*!< TIM3 Channel 1 */
    ADC1_2_INJTRIGGER_TIM6_TRGO  = 14, /*!< TIM6 Trigger Out */
    ADC1_2_INJTRIGGER_TIM15_TRGO = 15, /*!< TIM15 Trigger Out */

    ADC3_4_INJTRIGGER_TIM1_TRGO  = 0,  /*!< TIM1 Trigger Out */
    ADC3_4_INJTRIGGER_TIM1_CC4   = 1,  /*!< TIM1 Channel 4 */
    ADC3_4_INJTRIGGER_TIM4_CC3   = 2,  /*!< TIM4 Channel 3 */
    ADC3_4_INJTRIGGER_TIM8_CC2   = 3,  /*!< TIM8 Channel 2 */
    ADC3_4_INJTRIGGER_TIM8_CC3   = 4,  /*!< TIM8 Channel 3 */
    ADC3_4_INJTRIGGER_TIM8_CC4   = 5,  /*!< TIM8 Channel 4 */
    ADC3_4_INJTRIGGER_TIM4_CC4   = 6,  /*!< TIM4 Channel 4 */
    ADC3_4_INJTRIGGER_TIM4_TRGO  = 7,  /*!< TIM4 Trigger Out */
    ADC3_4_INJTRIGGER_TIM1_TRGO2 = 8,  /*!< TIM1 Trigger Out 2 */
    ADC3_4_INJTRIGGER_TIM8_TRGO  = 9,  /*!< TIM8 Trigger Out */
    ADC3_4_INJTRIGGER_TIM8_TRGO2 = 10, /*!< TIM8 Trigger Out 2 */
    ADC3_4_INJTRIGGER_TIM1_CC3   = 11, /*!< TIM1 Channel 3 */
    ADC3_4_INJTRIGGER_TIM3_TRGO  = 12, /*!< TIM3 Trigger Out */
    ADC3_4_INJTRIGGER_TIM2_TRGO  = 13, /*!< TIM2 Trigger Out */
    ADC3_4_INJTRIGGER_TIM7_TRGO  = 14, /*!< TIM7 Trigger Out */
    ADC3_4_INJTRIGGER_TIM15_TRGO = 15, /*!< TIM15 Trigger Out */
}ADC_InjTriggerSourceType;

/** @brief ADC injected channel setup structure */
typedef struct
{
    union {
    struct {
    ADC_InjTriggerSourceType TriggerSource : 4;     /*!< Source of the conversion trigger */
    FunctionalState          DiscontinuousMode : 1; /*!< Sets discontinuous mode
                                                         @note Cannot be used with auto injection */
    FunctionalState          ContextQueue : 1;      /*!< Context queue feature */
    EdgeType                 TriggerEdge : 2;       /*!< Trigger edges that initiate conversion */
    uint16_t : 1;
    FunctionalState          AutoInjection : 1;     /*!< Automatic injected conversion after regular group
                                                         @note External triggers must be disabled */
    uint16_t : 6;
    };
    uint16_t w;
    };
}ADC_InjectedInitType;

/** @} */

/** @addtogroup ADC_Injected_Exported_Functions
 * @{ */
void            ADC_vInjectedInit           (ADC_HandleType * pxADC,
                                             const ADC_InjectedInitType * pxConfig);
void            ADC_vInjectedChannelConfig  (ADC_HandleType*pxADC,
                                             const ADC_ChannelInitType axChannels[],
                                             uint8_t ucChannelCount);

void            ADC_vInjectedStart          (ADC_HandleType * pxADC);
void            ADC_vInjectedStop           (ADC_HandleType * pxADC);

void            ADC_vInjectedStart_IT       (ADC_HandleType * pxADC);
void            ADC_vInjectedStop_IT        (ADC_HandleType * pxADC);

/**
 * @brief Return the result of an ADC injected conversion.
 * @param pxADC: pointer to the ADC handle structure
 * @param ucIndex: index of channel on injected ADC conversion sequence (0-indexed)
 * @return The conversion result
 */
__STATIC_INLINE uint16_t ADC_usInjectedValue(ADC_HandleType * pxADC, uint8_t ucIndex)
{
    /* clear the flag for injected end of conversion */
    ADC_FLAG_CLEAR(pxADC, JEOC);

    return (uint16_t)((&pxADC->Inst->JDR1)[ucIndex]);
}

/** @} */

/** @} */

#ifdef ADC12_COMMON
/** @defgroup ADC_MultiMode Multi ADC Mode Conversions
 * @{ */

/** @defgroup ADC_MultiMode_Exported_Types Multi ADC Mode Exported Types
 * @{ */

/** @brief Multi ADC modes */
typedef enum
{
/* all three ADCs operate independently */
    ADC_MULTIMODE_SINGE                        = 0, /*!< The ADCs operate independently */
/* ADC 1 and 2 common operation */
    ADC_MULTIMODE_DUAL_REGSIMULT               = 6, /*!< ADC1+2 regular simultaneous mode */
    ADC_MULTIMODE_DUAL_INJECSIMULT             = 5, /*!< ADC1+2 injected simultaneous mode */
    ADC_MULTIMODE_DUAL_REGSIMULT_ALTTRIGGER    = 2, /*!< ADC1+2 regular simultaneous, alternate trigger mode */
    ADC_MULTIMODE_DUAL_REGSIMULT_INJECSIMULT   = 1, /*!< ADC1+2 regular simultaneous + injected simultaneous mode */
    ADC_MULTIMODE_DUAL_REGINTERL_INJECSIMULT   = 3, /*!< ADC1+2 regular interleaved + injected simultaneous mode */
    ADC_MULTIMODE_DUAL_ALTTRIGGER              = 9, /*!< ADC1+2 alternate trigger mode */
    ADC_MULTIMODE_DUAL_INTERLEAVED             = 7, /*!< ADC1+2 interleaved mode */
}ADC_MultiModeType;

/** @brief ADC DMA access modes */
typedef enum
{
    ADC_DMAACCESSMODE_DISABLED   = 0, /*!< DMA mode disabled, each ADC uses its own DMA channel */
    ADC_DMAACCESSMODE_12_10_BITS = 2, /*!< DMA of ADC master is used, for 12 and 10 bit resolutions */
    ADC_DMAACCESSMODE_8_6_BITS   = 3, /*!< DMA of ADC master is used, for 8 and 6 bit resolutions */
}ADC_DMAAccessModeType;

/** @brief Multi ADC setup structure */
typedef struct
{
    ADC_MultiModeType     Mode;               /*!< Multi-mode operation type configuration */
    ADC_DMAAccessModeType DMAAccessMode;      /*!< DMA access mode configuration */
    uint8_t               InterSamplingDelay; /*!< Delay between 2 sampling phases [1..12] */
}ADC_MultiModeInitType;

/** @} */

/** @addtogroup ADC_MultiMode_Exported_Functions
 * @{ */
void            ADC_vMultiModeInit          (ADC_HandleType * pxADC,
                                             const ADC_MultiModeInitType * pxConfig);
XPD_ReturnType  ADC_eMultiModeStart_DMA     (ADC_HandleType * pxADC, void * pvAddress);
void            ADC_vMultiModeStop_DMA      (ADC_HandleType * pxADC);

/**
 * @brief Return the result of the last common ADC regular conversions.
 * @param pxADC: pointer to the ADC handle structure
 * @return A pair of conversion results in a single word
 */
__STATIC_INLINE uint32_t ADC_ulMultiModeValues(ADC_HandleType * pxADC)
{
    /* return the multi mode conversion values */
    return ADC_COMMON(pxADC)->CDR.w;
}

/** @} */

/** @} */
#endif

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
