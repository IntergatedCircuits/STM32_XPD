/**
  ******************************************************************************
  * @file    xpd_tim.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Timer Module
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
#ifndef __XPD_TIM_H_
#define __XPD_TIM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_dma.h>
#include <xpd_rcc.h>

#if defined(TIM_CCR5_CCR5) && defined(TIM_CCR6_CCR6)
#define TIM_SUPPORTED_CHANNEL_COUNT   6
#else
#define TIM_SUPPORTED_CHANNEL_COUNT   4
#endif

/** @defgroup TIM
 * @{ */

/** @defgroup TIM_Common TIM Common
 * @{ */

/** @defgroup TIM_Common_Exported_Types TIM Common Exported Types
 * @{ */

/** @brief TIM counter modes */
typedef enum
{
    TIM_COUNTER_UP             = 0, /*!< Timer is counting from 0 to Period - 1 */
    TIM_COUNTER_DOWN           = 1, /*!< Timer is counting from Period - 1 to 0 */
    TIM_COUNTER_CENTERALIGNED1 = 2, /*!< Timer is switching between up- and downcounting,
                                         output channel interrupt is set during down */
    TIM_COUNTER_CENTERALIGNED2 = 4, /*!< Timer is switching between up- and downcounting,
                                         output channel interrupt is set during up */
    TIM_COUNTER_CENTERALIGNED3 = 6  /*!< Timer is switching between up- and downcounting,
                                         output channel interrupt is set during up & down */
}TIM_CounterType;

/** @brief TIM counter setup structure */
typedef struct
{
    uint32_t         Prescaler;         /*!< Clock prescaler for the counter, [1 .. (1 << timer bit size)] */
    uint32_t         Period;            /*!< Counter period, [1 .. (1 << timer bit size)] */
    TIM_CounterType  Mode;              /*!< Counter mode */
    ClockDividerType ClockDivision;     /*!< Division factor for deadtime and sampling clock.
                                             Permitted values: @arg CLK_DIV1 @arg CLK_DIV2 @arg CLK_DIV4 */
    uint8_t          RepetitionCounter; /*!< Specifies how many counter periods trigger an update
                                             @note Valid only for TIM1 and TIM8. */
} TIM_InitType;

/** @brief TIM channels */
typedef enum
{
    TIM_CH1 = 0, /*!< TIM channel 1 */
    TIM_CH2 = 1, /*!< TIM channel 2 */
    TIM_CH3 = 2, /*!< TIM channel 3 */
    TIM_CH4 = 3, /*!< TIM channel 4 */
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
    TIM_CH5 = 4, /*!< TIM channel 5 (limited capabilities: no IT or DMA) */
    TIM_CH6 = 5  /*!< TIM channel 6 (limited capabilities: no IT or DMA) */
#endif
}TIM_ChannelType;

/** @brief TIM event selection */
typedef enum
{
    TIM_EVENT_UPDATE    = 0, /*!< TIM counter value updated event */
    TIM_EVENT_CH1       = 1, /*!< TIM Capture/Compare channel 1 event */
    TIM_EVENT_CH2       = 2, /*!< TIM Capture/Compare channel 2 event */
    TIM_EVENT_CH3       = 3, /*!< TIM Capture/Compare channel 3 event */
    TIM_EVENT_CH4       = 4, /*!< TIM Capture/Compare channel 4 event */
    TIM_EVENT_COM       = 5, /*!< TIM Capture/Compare control update event */
    TIM_EVENT_TRIGGER   = 6, /*!< TIM Trigger event */
}TIM_EventType;

/** @brief TIM Handle structure */
typedef struct
{
    TIM_TypeDef * Inst;                      /*!< The address of the peripheral instance used by the handle */
#ifdef TIM_BB
    TIM_BitBand_TypeDef * Inst_BB;           /*!< The address of the peripheral instance in the bit-band region */
#endif
    struct {
        XPD_HandleCallbackType DepInit;      /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;    /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType Update;       /*!< Timer update callback */
        XPD_HandleCallbackType ChannelEvent; /*!< Channel compare/capture callback */
        XPD_HandleCallbackType Trigger;      /*!< Trigger callback */
        XPD_HandleCallbackType Commutation;  /*!< Commutation callback */
        XPD_HandleCallbackType Break;        /*!< Break callback */
#ifdef __XPD_DMA_ERROR_DETECT
        XPD_HandleCallbackType Error;        /*!< DMA error callback */
#endif
    }Callbacks;                              /*   Handle Callbacks */
    struct {
        DMA_HandleType * Update;             /*!< DMA handle for update transfer */
        DMA_HandleType * Channel[4];         /*!< DMA handles for channel transfers */
        DMA_HandleType * Burst;              /*!< DMA handle for burst transfers */
    }DMA;                                    /*   DMA handle references */
    RCC_PositionType CtrlPos;                /*!< Relative position for reset and clock control */
    volatile TIM_ChannelType ActiveChannel;  /*!< The currently active timer channel */
}TIM_HandleType;

/** @} */

/** @defgroup TIM_Common_Exported_Macros TIM Common Exported Macros
 * @{ */

#ifdef TIM_BB
/**
 * @brief TIM Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the TIM peripheral instance.
 */
#define         TIM_INST2HANDLE(HANDLE,INSTANCE)            \
    ((HANDLE)->Inst    = (INSTANCE),                        \
     (HANDLE)->Inst_BB = TIM_BB(INSTANCE),                  \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief TIM register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME)     \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief TIM Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the TIM peripheral instance.
 */
#define         TIM_INST2HANDLE(HANDLE,INSTANCE)            \
    ((HANDLE)->Inst    = (INSTANCE),                        \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief TIM register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME)     \
    ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)

#endif /* TIM_BB */

/**
 * @brief  Enable the specified TIM interrupt.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg U:       Update
 *            @arg CC1:     Capture/Compare channel 1
 *            @arg CC2:     Capture/Compare channel 2
 *            @arg CC3:     Capture/Compare channel 3
 *            @arg CC4:     Capture/Compare channel 4
 *            @arg COM:     Commutation
 *            @arg T:       Trigger
 *            @arg B:       Break
 */
#define         TIM_IT_ENABLE(HANDLE, IT_NAME)              \
    (TIM_REG_BIT((HANDLE),DIER,IT_NAME##IE) = 1)

/**
 * @brief  Disable the specified TIM interrupt.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg U:       Update
 *            @arg CC1:     Capture/Compare channel 1
 *            @arg CC2:     Capture/Compare channel 2
 *            @arg CC3:     Capture/Compare channel 3
 *            @arg CC4:     Capture/Compare channel 4
 *            @arg COM:     Commutation
 *            @arg T:       Trigger
 *            @arg B:       Break
 */
#define         TIM_IT_DISABLE(HANDLE, IT_NAME)             \
    (TIM_REG_BIT((HANDLE),DIER,IT_NAME##IE) = 0)

/**
 * @brief  Enable the specified TIM DMA request.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  DMA_NAME: specifies the DMA request to enable.
 *         This parameter can be one of the following values:
 *            @arg U:       Update
 *            @arg CC1:     Capture/Compare channel 1
 *            @arg CC2:     Capture/Compare channel 2
 *            @arg CC3:     Capture/Compare channel 3
 *            @arg CC4:     Capture/Compare channel 4
 *            @arg COM:     Commutation
 *            @arg T:       Trigger
 */
#define         TIM_DMA_ENABLE(HANDLE, DMA_NAME)            \
    (TIM_REG_BIT((HANDLE),DIER,DMA_NAME##DE) = 1)

/**
 * @brief  Disable the specified TIM DMA request.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  DMA_NAME: specifies the DMA request to enable.
 *         This parameter can be one of the following values:
 *            @arg U:       Update
 *            @arg CC1:     Capture/Compare channel 1
 *            @arg CC2:     Capture/Compare channel 2
 *            @arg CC3:     Capture/Compare channel 3
 *            @arg CC4:     Capture/Compare channel 4
 *            @arg COM:     Commutation
 *            @arg T:       Trigger
 */
#define         TIM_DMA_DISABLE(HANDLE, DMA_NAME)           \
    (TIM_REG_BIT((HANDLE),DIER,DMA_NAME##DE) = 0)

/**
 * @brief  Get the specified TIM flag.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg U:       Update
 *            @arg CC1:     Capture/Compare channel 1
 *            @arg CC2:     Capture/Compare channel 2
 *            @arg CC3:     Capture/Compare channel 3
 *            @arg CC4:     Capture/Compare channel 4
 *            @arg COM:     Commutation
 *            @arg T:       Trigger
 *            @arg B:       Break
 */
#define         TIM_FLAG_STATUS(HANDLE, FLAG_NAME)          \
    (TIM_REG_BIT((HANDLE),SR,FLAG_NAME##IF))

/**
 * @brief  Clear the specified TIM flag.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg U:       Update
 *            @arg CC1:     Capture/Compare channel 1
 *            @arg CC2:     Capture/Compare channel 2
 *            @arg CC3:     Capture/Compare channel 3
 *            @arg CC4:     Capture/Compare channel 4
 *            @arg COM:     Commutation
 *            @arg T:       Trigger
 *            @arg B:       Break
 */
#define         TIM_FLAG_CLEAR(HANDLE,FLAG_NAME)            \
    (TIM_REG_BIT((HANDLE),SR,FLAG_NAME##IF) = 0)

/**
 * @brief  Generate a TIM event.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  FLAG_NAME: specifies the event to generate.
 *         This parameter can be one of the following values:
 *            @arg U:       Update
 *            @arg CC1:     Capture/Compare channel 1
 *            @arg CC2:     Capture/Compare channel 2
 *            @arg CC3:     Capture/Compare channel 3
 *            @arg CC4:     Capture/Compare channel 4
 *            @arg COM:     Commutation
 *            @arg T:       Trigger
 */
#define         TIM_EVENT_SET(HANDLE,FLAG_NAME)             \
    (TIM_REG_BIT((HANDLE),EGR,FLAG_NAME##G) = 1)

/**
 * @brief Gets or sets the TIM counter direction.
 * @param pxTIM: pointer to the TIM handle structure
 * @return The current counter direction (up/down)
 */
#define         TIM_CNTR_DIRECTION(HANDLE)                  \
    (TIM_REG_BIT((HANDLE), CR1, DIR))

/**
 * @brief Gets or sets the current TIM counter value.
 * @param pxTIM: pointer to the TIM handle structure
 * @return The value of the counter
 */
#define         TIM_CNTR_VALUE(HANDLE)                      \
    ((HANDLE)->Inst->CNT)

/**
 * @brief Gets or sets the current TIM reload value.
 * @param pxTIM: pointer to the TIM handle structure
 * @return The value of the reload register
 */
#define         TIM_CNTR_RELOAD(HANDLE)                     \
    ((HANDLE)->Inst->ARR)

/**
 * @brief Gets or sets the current TIM repetition counter value.
 * @param pxTIM: pointer to the TIM handle structure
 * @return The value of the repetition counter
 */
#define         TIM_CNTR_REPETITION(HANDLE)                 \
    ((HANDLE)->Inst->RCR)

/**
 * @brief  Enable the specified TIM Channel interrupt.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel interrupt to enable.
 */
#define         TIM_CH_IT_ENABLE(HANDLE, CH)                \
    SET_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1IE << (CH))

/**
 * @brief  Disable the specified TIM Channel interrupt.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel interrupt to disable.
 */
#define         TIM_CH_IT_DISABLE(HANDLE, CH)               \
    CLEAR_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1IE << (CH))

/**
 * @brief  Enable the specified TIM Channel DMA transfer requests.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel DMA requests to enable.
 */
#define         TIM_CH_DMA_ENABLE(HANDLE, CH)               \
    SET_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1DE << (CH))

/**
 * @brief  Disable the specified TIM Channel DMA transfer requests.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel DMA requests to disable.
 */
#define         TIM_CH_DMA_DISABLE(HANDLE, CH)              \
    CLEAR_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1DE << (CH))

/**
 * @brief  Get the specified TIM channel flag.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel flag to return.
 */
#define         TIM_CH_FLAG_STATUS(HANDLE, CH)              \
    ((HANDLE)->Inst->SR.w & (TIM_SR_CC1IF << (CH)))

/**
 * @brief  Clear the specified TIM channel flag.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel flag to clear.
 */
#define         TIM_CH_FLAG_CLEAR(HANDLE, CH)               \
    CLEAR_BIT((HANDLE)->Inst->SR.w, TIM_SR_CC1IF << (CH))

#if TIM_SUPPORTED_CHANNEL_COUNT > 5
/**
 * @brief  Gets or sets the current TIM channel value.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel.
 * @return The value of the channel.
 */
#define         TIM_CH_VALUE(HANDLE, CH)                    \
    ((&(HANDLE)->Inst->CCR1)[((CH) < TIM_CH5)               \
                            ? (CH) : ((CH) + 5)])
#else
/**
 * @brief  Gets or sets the current TIM channel value.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel.
 * @return The value of the channel.
 */
#define         TIM_CH_VALUE(HANDLE, CH)                    \
    ((&(HANDLE)->Inst->CCR1)[ CH ]))
#endif

/** @} */

/** @addtogroup TIM_Common_Exported_Functions
 * @{ */
void            TIM_vInit               (TIM_HandleType * pxTIM, const TIM_InitType * pxConfig);
void            TIM_vDeinit             (TIM_HandleType * pxTIM);

void            TIM_vIRQHandler         (TIM_HandleType * pxTIM);


void            TIM_vCounterStart_IT    (TIM_HandleType * pxTIM);
void            TIM_vCounterStop_IT     (TIM_HandleType * pxTIM);

void            TIM_vIRQHandler_UP      (TIM_HandleType * pxTIM);

XPD_ReturnType  TIM_eCounterStart_DMA   (TIM_HandleType * pxTIM, void * pvAddress, uint16_t usLength);
void            TIM_vCounterStop_DMA    (TIM_HandleType * pxTIM);


void            TIM_vChannelStart       (TIM_HandleType * pxTIM, TIM_ChannelType eChannel);
void            TIM_vChannelStop        (TIM_HandleType * pxTIM, TIM_ChannelType eChannel);

void            TIM_vChannelStart_IT    (TIM_HandleType * pxTIM, TIM_ChannelType eChannel);
void            TIM_vChannelStop_IT     (TIM_HandleType * pxTIM, TIM_ChannelType eChannel);

void            TIM_vIRQHandler_CC      (TIM_HandleType * pxTIM);

XPD_ReturnType  TIM_eChannelStart_DMA   (TIM_HandleType * pxTIM, TIM_ChannelType eChannel,
                                         void * pvAddress, uint16_t usLength);
void            TIM_vChannelStop_DMA    (TIM_HandleType * pxTIM, TIM_ChannelType eChannel);


/**
 * @brief Enables the TIM counter.
 * @param pxTIM: pointer to the TIM handle structure
 */
__STATIC_INLINE void TIM_vCounterStart(TIM_HandleType * pxTIM)
{
    TIM_REG_BIT(pxTIM,CR1,CEN) = 1;
}

/**
 * @brief Disables the TIM counter.
 * @param pxTIM: pointer to the TIM handle structure
 */
__STATIC_INLINE void TIM_vCounterStop(TIM_HandleType * pxTIM)
{
    TIM_REG_BIT(pxTIM,CR1,CEN) = 0;
}

/**
 * @brief Enables the selected capture/compare channel.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the channel to enable
 */
__STATIC_INLINE void TIM_vChannelEnable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
#ifdef TIM_BB
    *(&pxTIM->Inst_BB->CCER.CC1E + (4 * eChannel)) = 1;
#else
    SET_BIT(pxTIM->Inst->CCER.w, TIM_CCER_CC1E << (4 * eChannel));
#endif
}

/**
 * @brief Disables the selected capture/compare channel.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the channel to disable
 */
__STATIC_INLINE void TIM_vChannelDisable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
#ifdef TIM_BB
    *(&pxTIM->Inst_BB->CCER.CC1E + (4 * eChannel)) = 0;
#else
    CLEAR_BIT(pxTIM->Inst->CCER.w, TIM_CCER_CC1E << (4 * eChannel));
#endif
}

/**
 * @brief Enables the selected complementary capture/compare channel.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the complementary channel to enable
 */
__STATIC_INLINE void TIM_vCompChannelEnable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
#ifdef TIM_BB
    *(&pxTIM->Inst_BB->CCER.CC1NE + (4 * eChannel)) = 1;
#else
    SET_BIT(pxTIM->Inst->CCER.w, TIM_CCER_CC1NE << (4 * eChannel));
#endif
}

/**
 * @brief Disables the selected complementary capture/compare channel.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the complementary channel to disable
 */
__STATIC_INLINE void TIM_vCompChannelDisable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
#ifdef TIM_BB
    *(&pxTIM->Inst_BB->CCER.CC1NE + (4 * eChannel)) = 0;
#else
    CLEAR_BIT(pxTIM->Inst->CCER.w, TIM_CCER_CC1NE << (4 * eChannel));
#endif
}

/** @} */

/** @} */

/** @defgroup TIM_Burst TIM Burst DMA Transfer Mode
 * @{ */

/** @defgroup TIM_Burst_Exported_Types TIM Burst DMA Exported Types
 * @{ */

/** @brief TIM burst DMA register start indexes */
typedef enum
{
    TIM_CR1_REG_INDEX = 0,
    TIM_CR2_REG_INDEX,
    TIM_SMCR_REG_INDEX,
    TIM_DIER_REG_INDEX,
    TIM_SR_REG_INDEX,
    TIM_EGR_REG_INDEX,
    TIM_CCMR1_REG_INDEX,
    TIM_CCMR2_REG_INDEX,
    TIM_CCER_REG_INDEX,
    TIM_CNT_REG_INDEX,
    TIM_PSC_REG_INDEX,
    TIM_ARR_REG_INDEX,
    TIM_RCR_REG_INDEX,
    TIM_CCR1_REG_INDEX,
    TIM_CCR2_REG_INDEX,
    TIM_CCR3_REG_INDEX,
    TIM_CCR4_REG_INDEX,
    TIM_BDTR_REG_INDEX,
    TIM_DCR_REG_INDEX,
}TIM_BurstRegIndexType;

/** @brief TIM DMA burst setup structure */
typedef struct
{
    TIM_BurstRegIndexType RegIndex; /*!< TIM register offset starting from the timer base */
    TIM_EventType         Source;   /*!< TIM burst DMA and trigger source */
}TIM_BurstInitType;

/** @} */

/** @addtogroup TIM_Burst_Exported_Functions
 * @{ */
XPD_ReturnType  TIM_eBurstStart_DMA     (TIM_HandleType * pxTIM, const TIM_BurstInitType * pxConfig,
                                             void * pvAddress, uint16_t usLength);
void            TIM_vBurstStop_DMA      (TIM_HandleType * pxTIM, TIM_EventType Source);
/** @} */

/** @} */

/** @defgroup TIM_Output TIM Output
 * @{ */

/** @defgroup TIM_Output_Exported_Types TIM Output Exported Types
 * @{ */

/** @brief TIM output modes */
typedef enum
{
    TIM_OUTPUT_TIMING             = 0,     /*!< Output channel is frozen, use for timing base generation */
    TIM_OUTPUT_ACTIVE             = 1,     /*!< Output channel is active on match */
    TIM_OUTPUT_INACTIVE           = 2,     /*!< Output channel is inactive on match */
    TIM_OUTPUT_TOGGLE             = 3,     /*!< Output channel is toggled on match */
    TIM_OUTPUT_FORCEDINACTIVE     = 4,     /*!< Output channel is forced low */
    TIM_OUTPUT_FORCEDACTIVE       = 5,     /*!< Output channel is forced high */
    TIM_OUTPUT_PWM1               = 6,     /*!< Output channel is active when CNT < CCR */
    TIM_OUTPUT_PWM2               = 7,     /*!< Output channel is active when CNT > CCR */
#if (TIM_CCMR1_OC1M > 0xFFFF)
    TIM_OUTPUT_RETRIGERRABLE_OPM1 = 0x100, /*!< Output channel is active after CNT < CCR at the time of trigger reception */
    TIM_OUTPUT_RETRIGERRABLE_OPM2 = 0x101, /*!< Output channel is active after CNT > CCR at the time of trigger reception */
    TIM_OUTPUT_COMBINED_PWM1      = 0x104, /*!< As in PWM1, but OC1REFC is the logical OR between OC1REF and OC2REF */
    TIM_OUTPUT_COMBINED_PWM2      = 0x105, /*!< As in PWM2, but OC1REFC is the logical AND between OC1REF and OC2REF */
    TIM_OUTPUT_ASYMMETRIC_PWM1    = 0x106, /*!< Output channel is active when CNT > CCR */
    TIM_OUTPUT_ASYMMETRIC_PWM2    = 0x107, /*!< Output channel is active when CNT > CCR */
#endif
}TIM_OutputModeType;

/** @brief TIM output channel setup structure */
typedef struct
{
    TIM_OutputModeType  Mode;            /*!< Output channel mode */
    ActiveLevelType     Polarity;        /*!< Output channel active level */
    FlagStatus          IdleState;       /*!< Output channel idle state */
    ActiveLevelType     CompPolarity;    /*!< Complementary output channel active level */
    FlagStatus          CompIdleState;   /*!< Complementary output channel idle state */
}TIM_OutputInitType;

/** @brief TIM Main Output drive setup structure */
typedef struct
{
    uint16_t        DeadCounts;      /*!< The dead time between the complementary outputs in deadtime clock counts */
    FunctionalState AutomaticOutput; /*!< Automatic output (MOE is set by software or update event) */
    FunctionalState IdleOffState;    /*!< Off-state selection for Idle mode */
    FunctionalState RunOffState;     /*!< Off-state selection for Run mode */
}TIM_DriveInitType;

/** @brief TIM Break setup structure */
typedef struct
{
    FunctionalState            State;    /*!< Break function state */
    ActiveLevelType            Polarity; /*!< Break input polarity */
#ifdef TIM_BDTR_BKF
    uint8_t                    Filter;   /*!< Break input capture filter */
#endif
}TIM_BreakInitType;

/** @brief TIM OCxREF clear configuration selection */
typedef enum
{
    TIM_OCREFCLEAR_SOURCE_NONE      = 0, /*!< OCxREF clear disabled */
    TIM_OCREFCLEAR_SOURCE_ETR       = 1, /*!< OCxREF cleared by ETRF high level */
#ifdef TIM_SMCR_OCCS
    TIM_OCREFCLEAR_SOURCE_OCREF_CLR = 2, /*!< OCxREF cleared by OCREF_CLR input high level */
#endif
}TIM_OCRefClearSourceType;

/** @brief TIM Commutation trigger source selection */
typedef enum
{
    TIM_COMSOURCE_NONE      = 0,                             /*!< Capture/compare preload disabled */
    TIM_COMSOURCE_SOFTWARE  = 1,                             /*!< Commutate only on software generated event */
    TIM_COMSOURCE_TRGI_ITR0 = 2,                             /*!< Commutate on Internal Trigger 0 */
    TIM_COMSOURCE_TRGI_ITR1 = TIM_SMCR_TS_0,                 /*!< Commutate on Internal Trigger 1 */
    TIM_COMSOURCE_TRGI_ITR2 = TIM_SMCR_TS_1,                 /*!< Commutate on Internal Trigger 2 */
    TIM_COMSOURCE_TRGI_ITR3 = TIM_SMCR_TS_1 | TIM_SMCR_TS_0, /*!< Commutate on Internal Trigger 3 */
}TIM_CommutationSourceType;

/** @} */

/** @addtogroup TIM_Output_Exported_Functions
 * @{ */
void            TIM_vOutputChannelConfig(TIM_HandleType * pxTIM, TIM_ChannelType eChannel,
                                             const TIM_OutputInitType * pxConfig);

void            TIM_vBreakConfig        (TIM_HandleType * pxTIM, uint8_t ucBreakLine,
                                             const TIM_BreakInitType * pxConfig);

void            TIM_vIRQHandler_BRK     (TIM_HandleType * pxTIM);

void            TIM_vDriveConfig        (TIM_HandleType * pxTIM, const TIM_DriveInitType * pxConfig);

void            TIM_vOCRefClearConfig   (TIM_HandleType * pxTIM, TIM_ChannelType eChannel,
                                         TIM_OCRefClearSourceType OCREF_CLR_IN);

void            TIM_vCommutationConfig  (TIM_HandleType * pxTIM,
                                         TIM_CommutationSourceType ComSource);

void            TIM_vIRQHandler_COM     (TIM_HandleType * pxTIM);

/**
 * @brief Sets the selected lock level to write-protect certain timer configuration registers.
 * @param pxTIM: pointer to the TIM handle structure
 * @param LockLevel: the specified lock level to apply [0..3]
 */
__STATIC_INLINE void TIM_vLockConfig(TIM_HandleType * pxTIM, uint8_t LockLevel)
{
    pxTIM->Inst->BDTR.b.LOCK = LockLevel;
}

/**
 * @brief Manually enables the main output of an advanced timer.
 * @param pxTIM: pointer to the TIM handle structure
 */
__STATIC_INLINE void TIM_vOutputEnable(TIM_HandleType * pxTIM)
{
    TIM_REG_BIT(pxTIM,BDTR,MOE) = 1;
}

/**
 * @brief Manually disables the main output of an advanced timer.
 * @param pxTIM: pointer to the TIM handle structure
 */
__STATIC_INLINE void TIM_vOutputDisable(TIM_HandleType * pxTIM)
{
    TIM_REG_BIT(pxTIM,BDTR,MOE) = 0;
}

/** @} */

/** @} */

/** @defgroup TIM_OnePulseMode TIM One Pulse Mode
 * @{ */

/** @defgroup TIM_OnePulseMode_Exported_Functions TIM One Pulse Mode Exported Functions
 * @{ */

/**
 * @brief Enables the one pulse mode of the timer output.
 * @param pxTIM: pointer to the TIM handle structure
 */
__STATIC_INLINE void TIM_vOnePulseModeStart(TIM_HandleType * pxTIM)
{
    TIM_REG_BIT(pxTIM, CR1, OPM) = 1;
}

/**
 * @brief Disables the one pulse mode of the timer output.
 * @param pxTIM: pointer to the TIM handle structure
 */
__STATIC_INLINE void TIM_vOnePulseModeStop(TIM_HandleType * pxTIM)
{
    TIM_REG_BIT(pxTIM, CR1, OPM) = 0;
}

/** @} */

/** @} */

/** @defgroup TIM_Input TIM Input
 * @{ */

/** @defgroup TIM_Input_Exported_Types TIM Input Exported Types
 * @{ */

/** @brief TIM input channel sources */
typedef enum
{
    TIM_INPUT_OWN_TI  = 1,  /*!< TIM input n is connected to ICn */
    TIM_INPUT_PAIR_TI = 2,  /*!< TIM input n is connected to IC(n ^ (n & 1)) */
    TIM_INPUT_TRC     = 3,  /*!< TIM input n is connected to TRC */
}TIM_InputSourceType;

/** @brief TIM input capture channel setup structure */
typedef struct
{
    TIM_InputSourceType  Source;    /*!< Specifies the input source */
    ActiveLevelType      Polarity;  /*!< Specifies the active edge of the input signal */
    ClockDividerType     Prescaler; /*!< Specifies the input capture prescaler [DIV1..DIV8] */
    uint8_t              Filter;    /*!< Specifies the input capture filter. [0..15] */
}TIM_InputInitType;

/** @} */

/** @addtogroup TIM_Input_Exported_Functions
 * {@ */
void            TIM_vInputChannelConfig (TIM_HandleType * pxTIM, TIM_ChannelType eChannel,
                                         const TIM_InputInitType * pxConfig);

/**
 * @brief Sets the TI1 input source as XOR(Ch1, Ch2, Ch3) instead of default Ch1.
 * @param pxTIM: pointer to the TIM handle structure
 * @param NewState: Input channels XOR selection
 */
__STATIC_INLINE void TIM_vInputChannelsXOR(TIM_HandleType * pxTIM, FunctionalState eNewState)
{
    TIM_REG_BIT(pxTIM,CR2,TI1S) = eNewState;
}

/** @} */

/** @} */

/** @defgroup TIM_MasterSlave TIM Master-Slave
 * @{ */

/** @defgroup TIM_MasterSlave_Exported_Types TIM Master-Slave Exported Types
 * @{ */

/** @brief TIM trigger out types */
typedef enum
{
    TIM_TRGO_RESET  = 0, /*!< TRGO is connected to EGR.UG */
    TIM_TRGO_ENABLE = 1, /*!< TRGO is connected to the timer enable bit CR1.EN */
    TIM_TRGO_UPDATE = 2, /*!< TRGO is connected to the timer update */
    TIM_TRGO_OC1    = 3, /*!< TRGO is connected to the eChannel 1 comparator on match */
    TIM_TRGO_OC1REF = 4, /*!< OC1REF is used for TRGO */
    TIM_TRGO_OC2REF = 5, /*!< OC2REF is used for TRGO */
    TIM_TRGO_OC3REF = 6, /*!< OC3REF is used for TRGO */
    TIM_TRGO_OC4REF = 7  /*!< OC4REF is used for TRGO */
}TIM_TriggerOutputType;

#ifdef TIM_CR2_MMS2
/** @brief TIM trigger out 2 types */
typedef enum
{
    TIM_TRGO2_RESET                        = 0,  /*!< TRGO2 is connected to EGR.UG */
    TIM_TRGO2_ENABLE                       = 1,  /*!< TRGO2 is connected to the timer enable bit CR1.EN */
    TIM_TRGO2_UPDATE                       = 2,  /*!< TRGO2 is connected to the timer update */
    TIM_TRGO2_OC1                          = 3,  /*!< TRGO2 is connected to the Channel 1 comparator on match */
    TIM_TRGO2_OC1REF                       = 4,  /*!< OC1REF is used for TRGO2 */
    TIM_TRGO2_OC2REF                       = 5,  /*!< OC2REF is used for TRGO2 */
    TIM_TRGO2_OC3REF                       = 6,  /*!< OC3REF is used for TRGO2 */
    TIM_TRGO2_OC4REF                       = 7,  /*!< OC4REF is used for TRGO2 */
    TIM_TRGO2_OC5REF                       = 8,  /*!< OC5REF is used for TRGO2 */
    TIM_TRGO2_OC6REF                       = 9,  /*!< OC6REF is used for TRGO2 */
    TIM_TRGO2_OC4REF_RISING_FALLING        = 10, /*!< OC4REF is used for TRGO2 with both edges */
    TIM_TRGO2_OC6REF_RISING_FALLING        = 11, /*!< OC6REF is used for TRGO2 with both edges */
    TIM_TRGO2_OC4REF_RISING_OC6REF_RISING  = 12, /*!< OC4REF with OC6REF is used for TRGO2 */
    TIM_TRGO2_OC4REF_RISING_OC6REF_FALLING = 13, /*!< OC4REF with !OC6REF is used for TRGO2 */
    TIM_TRGO2_OC5REF_RISING_OC6REF_RISING  = 14, /*!< OC5REF with OC6REF is used for TRGO2 */
    TIM_TRGO2_OC5REF_RISING_OC6REF_FALLING = 15, /*!< OC5REF with !OC6REF is used for TRGO2 */
}TIM_TriggerOutput2Type;
#endif

/** @brief TIM master setup structure */
typedef struct
{
    FunctionalState        MasterSlaveMode; /*!< Master/Slave mode configuration */
    TIM_TriggerOutputType  MasterTrigger;   /*!< Trigger output (TRGO) selection */
#ifdef TIM_CR2_MMS2
    TIM_TriggerOutput2Type MasterTrigger2;  /*!< Trigger output 2 (TRGO2) selection */
#endif
}TIM_MasterConfigType;

/** @brief TIM slave operating modes */
typedef enum
{
    TIM_SLAVEMODE_DISABLE        = 0, /* Slave mode disabled, internal clock is fed to prescaler */
    TIM_SLAVEMODE_ENCODER_1      = 1, /* Counts up/down on TI1FP1 edge depending on TI2FP2 level */
    TIM_SLAVEMODE_ENCODER_2      = 2, /* Counts up/down on TI2FP2 edge depending on TI1FP1 level */
    TIM_SLAVEMODE_ENCODER_12     = 3, /* Counts up/down on both TI1FP1 and TI2FP2 edges
                                         depending on the level of the other input */
    TIM_SLAVEMODE_RESET          = 4, /* TRGI rising edge reinitializes the counter and generates an Update event */
    TIM_SLAVEMODE_GATED          = 5, /* TRGI high level keeps the counter clock enabled */
    TIM_SLAVEMODE_TRIGGER        = 6, /* The counter starts at a rising edge of the trigger TRGI */
    TIM_SLAVEMODE_EXTERNAL_CLOCK = 7, /* TRGI rising edge clocks the counter */
#ifdef TIM_SMCR_SMS_3
    TIM_SLAVEMODE_RESET_TRIGGER  = TIM_SMCR_SMS_3, /* TRGI rising edge reinitializes the counter,
                                                      generates an Update event and starts the counter */
#endif
}TIM_SlaveModeType;

/** @brief TIM clock trigger types */
typedef enum
{
    TIM_TRGI_ITR0      = 0,                             /*!< Internal Trigger 0 */
    TIM_TRGI_ITR1      = TIM_SMCR_TS_0,                 /*!< Internal Trigger 1 */
    TIM_TRGI_ITR2      = TIM_SMCR_TS_1,                 /*!< Internal Trigger 2 */
    TIM_TRGI_ITR3      = TIM_SMCR_TS_1 | TIM_SMCR_TS_0, /*!< Internal Trigger 3 */
    TIM_TRGI_TI1       = TIM_SMCR_TS_2 | TIM_SMCR_TS_0, /*!< Filtered Timer Input 1 */
    TIM_TRGI_TI2       = TIM_SMCR_TS_2 | TIM_SMCR_TS_1, /*!< Filtered Timer Input 2 */
    TIM_TRGI_TI1_ED    = TIM_SMCR_TS_2,                 /*!< Timer Input 1 Edge Detector */
    TIM_TRGI_ETR_MODE1 = TIM_SMCR_TS,                   /*!< External Trigger input mode 1 */
    TIM_TRGI_ETR_MODE2 = TIM_SMCR_ECE | TIM_SMCR_TS,    /*!< External Trigger input mode 2 */
}TIM_TriggerInputType;

/** @brief TIM slave setup structure */
typedef struct {
    TIM_SlaveModeType    SlaveMode;    /*!< Slave mode selection  */
    TIM_TriggerInputType SlaveTrigger; /*!< Input Trigger source */
    ActiveLevelType      Polarity;     /*!< Specifies the active edge of the input signal */
    ClockDividerType     Prescaler;    /*!< Specifies the input clock prescaler [DIV1..DIV8] */
    uint8_t              Filter;       /*!< Specifies the input clock filter. [0..15] */
}TIM_SlaveConfigType;

/** @} */

/** @addtogroup TIM_MasterSlave_Exported_Functions
 * @{ */
void            TIM_vMasterConfig       (TIM_HandleType * pxTIM, const TIM_MasterConfigType * pxConfig);
void            TIM_vSlaveConfig        (TIM_HandleType * pxTIM, const TIM_SlaveConfigType * pxConfig);

void            TIM_vIRQHandler_TRG     (TIM_HandleType * pxTIM);
/** @} */

/** @} */

/** @} */

#define XPD_TIM_API
#include <xpd_rcc_pc.h>
#undef XPD_TIM_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_TIM_H_ */
