/**
  ******************************************************************************
  * @file    xpd_adc.h
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
#ifndef __XPD_ADC_H_
#define __XPD_ADC_H_

#include "xpd_common.h"
#include "xpd_config.h"
#include "xpd_adc_calc.h"
#include "xpd_dma.h"

/** @defgroup ADC
 * @{ */

/** @defgroup ADC_Clock_Source ADC Clock Source
 * @{ */

/** @defgroup ADC_Clock_Source_Exported_Types ADC Clock Source Exported Types
 * @{ */

/** @brief ADC clock source types */
typedef enum
{
    ADC_CLOCKSOURCE_PCLK_DIV2 = 0, /*!< PCLK divided by 2 */
    ADC_CLOCKSOURCE_PCLK_DIV4 = 1, /*!< PCLK divided by 4 */
    ADC_CLOCKSOURCE_PCLK_DIV6 = 2, /*!< PCLK divided by 6 */
    ADC_CLOCKSOURCE_PCLK_DIV8 = 3, /*!< PCLK divided by 8 */
}ADC_ClockSourceType;

/** @} */

/** @addtogroup ADC_Clock_Source_Exported_Functions
 * @{ */
void            XPD_ADC_ClockConfig         (ADC_ClockSourceType ClockSource);
uint32_t        XPD_ADC_GetClockFreq        (void);
/** @} */

/** @} */

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
    ADC_SAMPLETIME_3   = 0, /*!< Sampling during 3 clock cycles */
    ADC_SAMPLETIME_15  = 1, /*!< Sampling during 15 clock cycles */
    ADC_SAMPLETIME_28  = 2, /*!< Sampling during 28 clock cycles */
    ADC_SAMPLETIME_56  = 3, /*!< Sampling during 56 clock cycles */
    ADC_SAMPLETIME_84  = 4, /*!< Sampling during 84 clock cycles */
    ADC_SAMPLETIME_112 = 5, /*!< Sampling during 112 clock cycles */
    ADC_SAMPLETIME_144 = 6, /*!< Sampling during 144 clock cycles */
    ADC_SAMPLETIME_480 = 7  /*!< Sampling during 480 clock cycles */
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
    ADC_TRIGGER_TIM1_CC1  = 0,  /*!< TIM1 Channel 1 */
    ADC_TRIGGER_TIM1_CC2  = 1,  /*!< TIM1 Channel 2 */
    ADC_TRIGGER_TIM1_CC3  = 2,  /*!< TIM1 Channel 3 */
    ADC_TRIGGER_TIM2_CC2  = 3,  /*!< TIM2 Channel 2 */
    ADC_TRIGGER_TIM2_CC3  = 4,  /*!< TIM2 Channel 3 */
    ADC_TRIGGER_TIM2_CC4  = 5,  /*!< TIM2 Channel 4 */
    ADC_TRIGGER_TIM2_TRGO = 6,  /*!< TIM2 Trigger Out */
    ADC_TRIGGER_TIM3_CC1  = 7,  /*!< TIM3 Channel 1 */
    ADC_TRIGGER_TIM3_TRGO = 8,  /*!< TIM3 Trigger Out */
    ADC_TRIGGER_TIM4_CC4  = 9,  /*!< TIM4 Channel 4 */
    ADC_TRIGGER_TIM5_CC1  = 10, /*!< TIM5 Channel 1 */
    ADC_TRIGGER_TIM5_CC2  = 11, /*!< TIM5 Channel 2 */
    ADC_TRIGGER_TIM5_CC3  = 12, /*!< TIM5 Channel 3 */
    ADC_TRIGGER_TIM8_CC1  = 13, /*!< TIM8 Channel 1 */
    ADC_TRIGGER_TIM8_TRGO = 14, /*!< TIM8 Trigger Out */
    ADC_TRIGGER_EXTI11    = 15, /*!< EXTI Line 11 */
    ADC_TRIGGER_SOFTWARE  = 16  /*!< Implicit trigger by software on start call */
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
    ADC_OPERATION_CONVERSION    = ADC_SR_EOC,  /*!< Conversion */
#ifdef ADC_SR_JEOC
    ADC_OPERATION_INJCONVERSION = ADC_SR_JEOC, /*!< Injected conversion */
#endif
    ADC_OPERATION_WATCHDOG      = ADC_SR_AWD,  /*!< Analog watchdog */
    ADC_OPERATION_OVERRUN       = ADC_SR_OVR   /*!< Overrun */
}ADC_OperationType;

/** @brief ADC setup structure */
typedef struct
{
    ADC_ResolutionType  Resolution;            /*!< A/D conversion resolution */
    FunctionalState     LeftAlignment;         /*!< ENABLE to left-align converted data, otherwise DISABLE */
    uint8_t             ConversionCount;       /*!< Number of conversions in the sequence [1..16] */
    FunctionalState     ContinuousMode;        /*!< Continuous or single mode */
    FunctionalState     ContinuousDMARequests; /*!< Continuous DMA requests, or only for a single EOC flag */
    FunctionalState     ScanMode;              /*!< Scan mode converts all configured channels in sequence */
    uint8_t             DiscontinuousCount;    /*!< If not 0, a subgroup of channels is converted
                                                    on each trigger in loop [0..8] */
    ADC_EOCSelectType   EndFlagSelection;      /*!< Specifies when the EOC flag is set and the conversions stop */
    struct {
        ADC_TriggerSourceType Source;          /*!< Source of the conversion trigger */
        EdgeType              Edge;            /*!< Trigger edges that initiate conversion */
    }Trigger;
}ADC_InitType;

/** @brief ADC channel setup structure */
typedef struct
{
    uint32_t           Channel;    /*!< Number of the ADC channel [0..18] */
    uint8_t            Rank;       /*!< Channel rank [1 .. 16] */
    ADC_SampleTimeType SampleTime; /*!< Sample time of the channel */
}ADC_ChannelInitType;

/** @brief ADC watchdog channel selection */
typedef enum
{
    ADC_WATCHDOG_NONE           = 0,                                                 /*!< No channels */
    ADC_WATCHDOG_SINGLE_REG     = (ADC_CR1_AWDSGL | ADC_CR1_AWDEN),                  /*!< A single regular channel */
    ADC_WATCHDOG_SINGLE_INJ     = (ADC_CR1_AWDSGL | ADC_CR1_JAWDEN),                 /*!< A single injected channel */
    ADC_WATCHDOG_SINGLE_REG_INJ = (ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_JAWDEN), /*!< A single regular/injected channel */
    ADC_WATCHDOG_ALL_REG        =  ADC_CR1_AWDEN,                                    /*!< All regular channels */
    ADC_WATCHDOG_ALL_INJ        =  ADC_CR1_JAWDEN,                                   /*!< All injected channels */
    ADC_WATCHDOG_ALL_REG_INJ    = (ADC_CR1_AWDEN | ADC_CR1_JAWDEN)                   /*!< All regular and injected channels */
}ADC_WatchdogModeType;

/** @brief ADC watchdog setup structure */
typedef struct
{
    ADC_WatchdogModeType Mode;     /*!< Watchdog channel selection */
    struct {
        uint16_t High;             /*!< Watchdog high threshold */
        uint16_t Low;              /*!< Watchdog low threshold */
    } Threshold;
}ADC_WatchdogInitType;

/** @brief ADC Handle structure */
typedef struct
{
    ADC_TypeDef * Inst;                             /*!< The address of the peripheral instance used by the handle */
#ifdef ADC_BB
    ADC_BitBand_TypeDef * Inst_BB;                  /*!< The address of the peripheral instance in the bit-band region */
#endif
    XPD_CtrlFnType ClockCtrl;                       /*!< Function pointer for RCC clock control */
    struct {
        XPD_HandleCallbackType DepInit;             /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;           /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType ConvComplete;        /*!< Conversion(s) complete callback */
        XPD_HandleCallbackType InjConvComplete;     /*!< Injected conversion(s) complete callback */
        XPD_HandleCallbackType Watchdog;            /*!< Watchdog alert callback */
#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
        XPD_HandleCallbackType Error;               /*!< DMA transfer or overrun error callback */
#endif
    }Callbacks;                                     /*   Handle Callbacks */
    struct {
        DMA_HandleType * Conversion;                /*!< DMA handle for update transfer */
    }DMA;                                           /*   DMA handle references */
    volatile uint8_t ActiveConversions;             /*!< ADC number of current regular conversion rank */
#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
    volatile ADC_ErrorType Errors;                  /*!< Conversion errors */
#endif
}ADC_HandleType;

/** @} */

/** @defgroup ADC_Core_Exported_Macros ADC Core Exported Macros
 * @{ */

#ifdef ADC_BB
/**
 * @brief  ADC Handle initializer macro
 * @param  INSTANCE: specifies the ADC peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_ADC_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)      \
    {.Inst = (INSTANCE), .Inst_BB = ADC_BB(INSTANCE),           \
     .Callbacks.DepInit   = (INIT_FN),                          \
     .Callbacks.DepDeinit = (DEINIT_FN)}

/**
 * @brief ADC register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define         ADC_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

/**
 * @brief ADC common register bit accessing macro
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define         ADC_COMMON_REG_BIT( REG_NAME, BIT_NAME)         \
    (ADC_COMMON_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief  ADC Handle initializer macro
 * @param  INSTANCE: specifies the ADC peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_ADC_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)      \
    {.Inst = (INSTANCE),                                        \
     .Callbacks.DepInit   = (INIT_FN),                          \
     .Callbacks.DepDeinit = (DEINIT_FN)}

/**
 * @brief ADC register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define         ADC_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)

/**
 * @brief ADC common register bit accessing macro
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define         ADC_COMMON_REG_BIT( REG_NAME, BIT_NAME)         \
    (ADC_BASE->REG_NAME.b.BIT_NAME)

#endif /* ADC_BB */

/**
 * @brief  Enable the specified ADC interrupt.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg EOC:     End of conversion
 *            @arg AWD:     Analog watchdog alert
 *            @arg JEOC:    Injected end of conversion
 *            @arg OVR:     Overrun
 */
#define         XPD_ADC_EnableIT(  HANDLE,  IT_NAME)            \
    (ADC_REG_BIT((HANDLE),CR1,IT_NAME##IE) = 1)

/**
 * @brief  Disable the specified ADC interrupt.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg EOC:     End of conversion
 *            @arg AWD:     Analog watchdog alert
 *            @arg JEOC:    Injected end of conversion
 *            @arg OVR:     Overrun
 */
#define         XPD_ADC_DisableIT( HANDLE,  IT_NAME)            \
    (ADC_REG_BIT((HANDLE),CR1,IT_NAME##IE) = 0)

/**
 * @brief  Get the specified ADC flag.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg EOC:     End of conversion
 *            @arg JEOC:    Injected end of conversion
 *            @arg OVR:     Overrun
 *            @arg STRT:    Regular channel start flag
 *            @arg JSTRT:   Injected channel start flag
 */
#define         XPD_ADC_GetFlag(  HANDLE, FLAG_NAME)            \
    (ADC_REG_BIT((HANDLE),SR,FLAG_NAME))

/**
 * @brief  Clear the specified ADC flag.
 * @param  HANDLE: specifies the ADC Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg EOC:     End of conversion
 *            @arg JEOC:    Injected end of conversion
 *            @arg OVR:     Overrun
 *            @arg STRT:    Regular channel start flag
 *            @arg JSTRT:   Injected channel start flag
 */
#define         XPD_ADC_ClearFlag(HANDLE, FLAG_NAME)            \
    (ADC_REG_BIT((HANDLE),SR,FLAG_NAME) = 0)

/* Compatibility macros */
#define         XPD_ADC_Calibrate(HANDLE, DIFFERENTIAL)     ((void)0)

/** @} */

/** @defgroup ADC_Core_Exported_Functions ADC Core Exported Functions
 * @{ */
XPD_ReturnType  XPD_ADC_Init                (ADC_HandleType * hadc, const ADC_InitType * Config);
XPD_ReturnType  XPD_ADC_Deinit              (ADC_HandleType * hadc);
void            XPD_ADC_ChannelConfig       (ADC_HandleType * hadc, const ADC_ChannelInitType * Config);

void            XPD_ADC_Start               (ADC_HandleType * hadc);
void            XPD_ADC_Stop                (ADC_HandleType * hadc);
XPD_ReturnType  XPD_ADC_PollStatus          (ADC_HandleType * hadc, ADC_OperationType Operation, uint32_t Timeout);

void            XPD_ADC_Start_IT            (ADC_HandleType * hadc);
void            XPD_ADC_Stop_IT             (ADC_HandleType * hadc);
void            XPD_ADC_IRQHandler          (ADC_HandleType * hadc);

XPD_ReturnType  XPD_ADC_Start_DMA           (ADC_HandleType * hadc, void * Address);
void            XPD_ADC_Stop_DMA            (ADC_HandleType * hadc);

void            XPD_ADC_WatchdogConfig      (ADC_HandleType * hadc, uint8_t Channel,
                                             const ADC_WatchdogInitType * Config);
uint8_t         XPD_ADC_WatchdogStatus      (ADC_HandleType * hadc);

uint16_t        XPD_ADC_GetValue            (ADC_HandleType * hadc);
/** @} */

/** @} */

/** @defgroup ADC_Injected ADC Injected Conversions
 * @{ */

/** @defgroup ADC_Injected_Exported_Types ADC Injected Exported Types
 * @{ */

/** @brief ADC injected trigger sources */
typedef enum
{
    ADC_INJTRIGGER_TIM1_CC4  = 0,  /*!< TIM1 Channel 4 */
    ADC_INJTRIGGER_TIM1_TRGO = 1,  /*!< TIM1 Trigger Out */
    ADC_INJTRIGGER_TIM2_CC1  = 2,  /*!< TIM2 Channel 1 */
    ADC_INJTRIGGER_TIM2_TRGO = 3,  /*!< TIM2 Trigger Out */
    ADC_INJTRIGGER_TIM3_CC2  = 4,  /*!< TIM3 Channel 2 */
    ADC_INJTRIGGER_TIM3_CC4  = 5,  /*!< TIM3 Channel 4 */
    ADC_INJTRIGGER_TIM4_CC1  = 6,  /*!< TIM4 Channel 1 */
    ADC_INJTRIGGER_TIM4_CC2  = 7,  /*!< TIM4 Channel 2 */
    ADC_INJTRIGGER_TIM4_CC3  = 8,  /*!< TIM4 Channel 3 */
    ADC_INJTRIGGER_TIM4_TRGO = 9,  /*!< TIM4 Trigger Out */
    ADC_INJTRIGGER_TIM5_CC4  = 10, /*!< TIM5 Channel 4 */
    ADC_INJTRIGGER_TIM5_TRGO = 11, /*!< TIM5 Trigger Out */
    ADC_INJTRIGGER_TIM8_CC2  = 12, /*!< TIM8 Channel 2 */
    ADC_INJTRIGGER_TIM8_CC3  = 13, /*!< TIM8 Channel 3 */
    ADC_INJTRIGGER_TIM8_CC4  = 14, /*!< TIM8 Channel 4 */
    ADC_INJTRIGGER_EXTI15    = 15, /*!< EXTI Line 15 */
    ADC_INJTRIGGER_SOFTWARE  = 16  /*!< Implicit trigger by software on start call */
}ADC_InjTriggerSourceType;

/** @brief ADC injected channel setup structure */
typedef struct
{
    uint32_t           Channel;             /*!< The ADC injected channel selection */
    uint8_t            Rank;                /*!< The rank in the injected group sequencer [1..4] */
    uint8_t            ConversionCount;     /*!< Number of injected conversions in the injected group [1..4] */
    uint16_t           Offset;              /*!< The data to be subtracted from the converted value [0..4095] */
    ADC_SampleTimeType SampleTime;          /*!< The sample time value to be set for the selected channel */
    FunctionalState    AutoInjection;       /*!< Automatic injected conversion after regular group
                                                 @note External triggers must be disabled */
    FunctionalState    DiscontinuousMode;   /*!< Sets discontinuous mode
                                                 @note Cannot be used with auto injection */
    struct {
        ADC_InjTriggerSourceType InjSource; /*!< Source of the conversion trigger */
        EdgeType                 Edge;      /*!< Trigger edges that initiate conversion */
    }Trigger;
}ADC_Injected_InitType;

/** @} */

/** @addtogroup ADC_Injected_Exported_Functions
 * @{ */
void            XPD_ADC_Injected_Init       (ADC_HandleType * hadc, ADC_Injected_InitType * Config);
void            XPD_ADC_Injected_Start      (ADC_HandleType * hadc);
void            XPD_ADC_Injected_Start_IT   (ADC_HandleType * hadc);
uint16_t        XPD_ADC_Injected_GetValue   (ADC_HandleType * hadc, uint8_t Rank);
/** @} */

/** @} */

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
    ADC_MULTIMODE_DUAL_REGSIMULT_INJECSIMULT   = 1, /*!< ADC1+2 regular simultaneous + injected simultaneous mode */
    ADC_MULTIMODE_DUAL_REGSIMULT_ALTTRIGGER    = 2, /*!< ADC1+2 regular simultaneous, alternate trigger mode */
    ADC_MULTIMODE_DUAL_ALTTRIGGER              = 9, /*!< ADC1+2 alternate trigger mode */
    ADC_MULTIMODE_DUAL_INTERLEAVED             = 7, /*!< ADC1+2 interleaved mode */
/* ADC 1, 2 and 3 common operation */
    ADC_MULTIMODE_TRIPLE_REGSIMULT             = 6 | 0x10, /*!< ADC1+2+3 regular simultaneous mode */
    ADC_MULTIMODE_TRIPLE_INJECSIMULT           = 5 | 0x10, /*!< ADC1+2+3 injected simultaneous mode */
    ADC_MULTIMODE_TRIPLE_REGSIMULT_INJECSIMULT = 1 | 0x10, /*!< ADC1+2+3 regular simultaneous + injected simultaneous mode */
    ADC_MULTIMODE_TRIPLE_REGSIMULT_ALTTRIGGER  = 2 | 0x10, /*!< ADC1+2+3 regular simultaneous, alternate trigger mode */
    ADC_MULTIMODE_TRIPLE_ALTTRIGGER            = 9 | 0x10, /*!< ADC1+2+3 alternate trigger mode */
    ADC_MULTIMODE_TRIPLE_INTERLEAVED           = 7 | 0x10, /*!< ADC1+2+3 interleaved mode */
}ADC_MultiModeType;

/** @brief ADC DMA access modes */
typedef enum
{
    ADC_DMAACCESSMODE_DISABLED = 0, /*!< DMA mode disabled */
    ADC_DMAACCESSMODE_1        = 1, /*!< DMA mode 1 enabled (2 / 3 half-words one by one - 1 then 2 then 3)*/
    ADC_DMAACCESSMODE_2        = 2, /*!< DMA mode 2 enabled (2 / 3 half-words by pairs - 2&1 then 1&3 then 3&2)*/
    ADC_DMAACCESSMODE_3        = 3  /*!< DMA mode 3 enabled (2 / 3 bytes by pairs - 2&1 then 1&3 then 3&2) */
}ADC_DMAAccessModeType;

/** @brief Multi ADC setup structure */
typedef struct
{
    ADC_MultiModeType     Mode;               /*!< Multi-mode operation type configuration */
    ADC_DMAAccessModeType DMAAccessMode;      /*!< DMA access mode configuration */
    uint8_t               InterSamplingDelay; /*!< Delay between 2 sampling phases [5..20] */
}ADC_MultiMode_InitType;

/** @} */

/** @addtogroup ADC_MultiMode_Exported_Functions
 * @{ */
void            XPD_ADC_MultiMode_Init          (ADC_HandleType * hadc, const ADC_MultiMode_InitType * Config);
XPD_ReturnType  XPD_ADC_MultiMode_Start_DMA     (ADC_HandleType * hadc, void * Address);
void            XPD_ADC_MultiMode_Stop_DMA      (ADC_HandleType * hadc);
uint32_t        XPD_ADC_MultiMode_GetValues     (ADC_HandleType * hadc);
/** @} */

/** @} */

/** @} */

#define XPD_ADC_API
#include "xpd_rcc_gen.h"
#undef XPD_ADC_API

#endif /* __XPD_ADC_H_ */
