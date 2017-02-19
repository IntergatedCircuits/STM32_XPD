/**
  ******************************************************************************
  * @file    xpd_adc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-02-04
  * @brief   STM32 eXtensible Peripheral Drivers TODO Module
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
#ifndef XPD_ADC_H_
#define XPD_ADC_H_

#include "xpd_common.h"
#include "xpd_config.h"
#include "xpd_adc_calc.h"
#include "xpd_dma.h"

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
    ADC_ERROR_OVR       = 1,   /*!< Overrun flag */
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
    ADC_TRIGGER_SOFTWARE   = 8   /*!< Implicit trigger by software on start call */
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
    ADC_OPERATION_CONVERSION    = ADC_ISR_EOS | ADC_ISR_EOC,    /*!< Conversion */
    ADC_OPERATION_WATCHDOG      = ADC_ISR_AWD1,                 /*!< Analog watchdog */
    ADC_OPERATION_OVERRUN       = ADC_ISR_OVR                   /*!< Overrun */
}ADC_OperationType;

typedef enum
{
    ADC_SCAN_FORWARD  = 0,
    ADC_SCAN_BACKWARD = 1
}ADC_ScanDirectionType;

/** @brief ADC setup structure */
typedef struct
{
    ADC_ResolutionType    Resolution;            /*!< A/D conversion resolution */
    FunctionalState       LeftAlignment;         /*!< ENABLE to left-align converted data, otherwise DISABLE */
    FunctionalState       ContinuousMode;        /*!< Continuous or single mode */
    FunctionalState       ContinuousDMARequests; /*!< Continuous DMA requests, or only for a single EOC flag */
    ADC_ScanDirectionType ScanDirection;         /*!< Defines if channels are converted from channels 0 through 18 or in reverse order */
    uint8_t               DiscontinuousCount;    /*!< If 1, a single channel is converted on each trigger in the sequence */
    ADC_EOCSelectType     EndFlagSelection;      /*!< Specifies when the EOC flag is set and the conversions stop */
    FunctionalState       LPAutoWait;            /*!< When enabled, new conversion starts only after the user has handled the current conversion. */
    FunctionalState       LPAutoPowerOff;        /*!< When enabled, the ADC automatically powers-off after a conversion
                                                    and automatically starts up when a new conversion is triggered.
                                                    @note: This feature also turns off the ADC dedicated 14 MHz RC oscillator (HSI14) */
    ADC_SampleTimeType    SampleTime;            /*!< Common sample time of all ADC channels */
    struct {
        ADC_TriggerSourceType Source;            /*!< Source of the conversion trigger */
        EdgeType              Edge;              /*!< Trigger edges that initiate conversion */
    }Trigger;
}ADC_InitType;

/** @brief ADC channel setup structure */
typedef struct
{
    uint32_t           Channel;      /*!< Number of the ADC channel [0..18] */
    uint8_t            Rank;         /*!< Channel availability (set to positive to enable) */
}ADC_ChannelInitType;

/** @brief ADC watchdog channel selection */
typedef enum
{
    ADC_WATCHDOG_NONE           = 0,                                                       /*!< No channels */
    ADC_WATCHDOG_SINGLE_REG     = (ADC_CFGR1_AWDSGL | ADC_CFGR1_AWDEN),                    /*!< A single regular channel */
    ADC_WATCHDOG_ALL_REG        =  ADC_CFGR1_AWDEN,                                        /*!< All regular channels */
}ADC_WatchdogModeType;

/** @brief ADC watchdog setup structure */
typedef struct
{
    ADC_WatchdogModeType Mode;     /*!< Watchdog channel selection */
    struct {
        uint16_t High;             /*!< Watchdog high threshold */
        uint16_t Low;              /*!< Watchdog low threshold */
    } Threshold;
    uint8_t Number;                /*!< Watchdog number */
}ADC_WatchdogInitType;

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
        XPD_HandleCallbackType HalfConvComplete;    /*!< Half of the conversions complete callback */
        XPD_HandleCallbackType Watchdog;            /*!< Watchdog alert callback */
#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
        XPD_HandleCallbackType Error;
#endif
    }Callbacks;                                     /*   Handle Callbacks */
    struct {
        DMA_HandleType * Conversion;                /*!< DMA handle for update transfer */
    }DMA;                                           /*   DMA handle references */
    uint8_t ActiveConversions;                      /*!< ADC number of current regular conversions */
    uint8_t EndFlagSelection;                       /*!< [Internal] Stores the EOC configuration */
#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
    volatile ADC_ErrorType Errors;                  /*!< Conversion errors */
#endif
}ADC_HandleType;

/** @} */

/** @defgroup ADC_Core_Exported_Macros ADC Core Exported Macros
 * @{ */

#if defined(USE_XPD_ADC_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
/**
 * @brief  ADC Handle initializer macro
 * @param  INSTANCE: specifies the ADC peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_ADC_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)               \
    {.Inst      = (INSTANCE),                                            \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL,NULL,NULL},           \
     .Errors    = ADC_ERROR_NONE}
#else
/**
 * @brief  ADC Handle initializer macro
 * @param  INSTANCE: specifies the ADC peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_ADC_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)               \
    {.Inst      = (INSTANCE),                                            \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL,NULL}}
#endif

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
#define         XPD_ADC_EnableIT(  HANDLE,  IT_NAME)            \
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
#define         XPD_ADC_DisableIT( HANDLE,  IT_NAME)            \
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
#define         XPD_ADC_GetFlag(  HANDLE, FLAG_NAME)            \
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
#define         XPD_ADC_ClearFlag(HANDLE, FLAG_NAME)            \
    ((HANDLE)->Inst->ISR.w = ADC_ISR_##FLAG_NAME)

#ifdef ADC_BB
#define ADC_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#else
#define ADC_REG_BIT(HANDLE, REG_NAME, BIT_NAME) (HANDLE->Inst->REG_NAME.b.BIT_NAME)
#endif

/** @} */

/** @defgroup ADC_Core_Exported_Functions ADC Core Exported Functions
 * @{ */
XPD_ReturnType  XPD_ADC_Init                (ADC_HandleType * hadc, ADC_InitType * Config);
XPD_ReturnType  XPD_ADC_Deinit              (ADC_HandleType * hadc);
void            XPD_ADC_ChannelConfig       (ADC_HandleType * hadc, ADC_ChannelInitType * Config);

void            XPD_ADC_Start               (ADC_HandleType * hadc);
void            XPD_ADC_Stop                (ADC_HandleType * hadc);
XPD_ReturnType  XPD_ADC_PollStatus          (ADC_HandleType * hadc, ADC_OperationType Operation, uint32_t Timeout);

void            XPD_ADC_Start_IT            (ADC_HandleType * hadc);
void            XPD_ADC_Stop_IT             (ADC_HandleType * hadc);
void            XPD_ADC_IRQHandler          (ADC_HandleType * hadc);

XPD_ReturnType  XPD_ADC_Start_DMA           (ADC_HandleType * hadc, void * Address);
void            XPD_ADC_Stop_DMA            (ADC_HandleType * hadc);

void            XPD_ADC_WatchdogConfig      (ADC_HandleType * hadc, uint8_t Channel, ADC_WatchdogInitType * Config);
uint8_t         XPD_ADC_WatchdogStatus      (ADC_HandleType * hadc);

uint16_t        XPD_ADC_GetValue            (ADC_HandleType * hadc);
/** @} */

/** @} */

/** @defgroup ADC_Calibration ADC Calibration
 * @{ */

/** @addtogroup ADC_Calibration_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_ADC_Calibrate           (ADC_HandleType * hadc, bool Differential);
/** @} */

/** @} */

/** @} */

#define XPD_ADC_API
#include "xpd_rcc_pc.h"
#undef XPD_ADC_API

#endif /* XPD_ADC_H_ */
