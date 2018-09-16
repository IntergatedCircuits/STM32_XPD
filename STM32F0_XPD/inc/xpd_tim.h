/**
  ******************************************************************************
  * @file    xpd_tim.h
  * @author  Benedek Kupper
  * @version 0.4
  * @date    2018-09-15
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
    FunctionalState  Preload;           /*!< Reload (Period) is preloaded and only applied after an update (ARPE) */
    FunctionalState  OnePulse;          /*!< After an update counter stops automatically (OPM) */
}TIM_InitType;

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

/** @brief TIM output modes */
typedef enum
{
    TIM_OUTPUT_TIMING             = 0x00,   /*!< Output channel is frozen, use for timing base generation */
    TIM_OUTPUT_ACTIVE             = 0x04,   /*!< Output channel is active on match */
    TIM_OUTPUT_INACTIVE           = 0x08,   /*!< Output channel is inactive on match */
    TIM_OUTPUT_TOGGLE             = 0x0C,   /*!< Output channel is toggled on match */
    TIM_OUTPUT_FORCEDINACTIVE     = 0x10,   /*!< Output channel is forced low */
    TIM_OUTPUT_FORCEDACTIVE       = 0x14,   /*!< Output channel is forced high */
    TIM_OUTPUT_PWM1               = 0x1B,   /*!< Output channel is active when CNT < CCR */
    TIM_OUTPUT_PWM2               = 0x1F,   /*!< Output channel is active when CNT > CCR */
#if (TIM_CCMR1_OC1M > 0xFFFF)
    TIM_OUTPUT_RETRIGERRABLE_OPM1 = 0x4000, /*!< Output channel is active after CNT < CCR at the time of trigger reception */
    TIM_OUTPUT_RETRIGERRABLE_OPM2 = 0x4004, /*!< Output channel is active after CNT > CCR at the time of trigger reception */
    TIM_OUTPUT_COMBINED_PWM1      = 0x4013, /*!< As in PWM1, but OC1REFC is the logical OR between OC1REF and OC2REF */
    TIM_OUTPUT_COMBINED_PWM2      = 0x4017, /*!< As in PWM2, but OC1REFC is the logical AND between OC1REF and OC2REF */
    TIM_OUTPUT_ASYMMETRIC_PWM1    = 0x401B, /*!< Output channel is active when CNT > CCR */
    TIM_OUTPUT_ASYMMETRIC_PWM2    = 0x401F, /*!< Output channel is active when CNT > CCR */
#endif
    TIM_OUTPUT_PRELOAD_FLAG       = 0x02,   /*!< Preloaded value is activated at next update event */
}TIM_OutputModeType;

/** @brief TIM input channel sources */
typedef enum
{
    TIM_INPUT_OWN_TI  = 1,  /*!< TIM input n is connected to ICn */
    TIM_INPUT_PAIR_TI = 2,  /*!< TIM input n is connected to IC(n ^ (n & 1)) */
    TIM_INPUT_TRC     = 3,  /*!< TIM input n is connected to TRC */
}TIM_InputSourceType;

/** @brief TIM capture/compare channel setup structure */
typedef union
{
    struct {
        uint32_t Value;         /*!< Channel initial value */
        union {
        struct {
        uint32_t : 2;
#if (TIM_CCMR1_OC1M > 0xFFFF)
        TIM_OutputModeType  Mode : 15;       /*!< Output channel mode */
#else
        TIM_OutputModeType  Mode : 5;        /*!< Output channel mode */
        uint32_t : 10;
#endif
        ActiveLevelType     Polarity : 1;    /*!< Output channel active level */
        FunctionalState     InvChannel : 1;  /*!< Complementary output channel is used */
        ActiveLevelType     InvPolarity : 1; /*!< Complementary output channel active level */
        uint32_t : 4;
        FlagStatus          IdleState : 1;   /*!< Output channel idle state */
        FlagStatus          InvIdleState : 1;/*!< Complementary output channel idle state */
        uint32_t : 6;
        }__packed;
            uint32_t w; /* Internal use only */
        };
        TIM_ChannelType Number; /*!< Channel number */
    }OC;                        /*!< Output Compare channel setup */
    struct {
        uint32_t Value;         /*!< Channel initial value */
        union {
        struct {
        TIM_InputSourceType Source : 2;      /*!< Specifies the input source */
        ClockDividerType    Prescaler : 2;   /*!< Specifies the input capture prescaler [DIV1..DIV8] */
        uint32_t            Filter : 4;      /*!< Specifies the input capture filter. [0..15] */
        uint32_t : 9;
        ActiveLevelType     Polarity : 1;    /*!< Specifies the active edge of the input signal */
        uint32_t : 14;
        }__packed;
            uint8_t b[4]; /* Internal use only */
        };
        TIM_ChannelType Number; /*!< Channel number */
    }IC;                        /*!< Input Capture channel setup */
}TIM_ChannelInitType;

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
    uint32_t ChannelMask;                    /*!< Configured timer channels mask */
    uint8_t  EventMask;                      /*!< Configured timer channels event mask */
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
 * @brief Gets or sets the TIM counter direction.
 * @param pxTIM: pointer to the TIM handle structure
 * @return The current counter direction (up/down)
 */
#define         TIM_DIRECTION(HANDLE)                       \
    (TIM_REG_BIT((HANDLE), CR1, DIR))

/** @} */

/** @addtogroup TIM_Common_Exported_Functions
 * @{ */
void            TIM_vInit               (TIM_HandleType * pxTIM, const TIM_InitType * pxConfig,
                                         const TIM_ChannelInitType axChannels[], uint8_t ucChannelCount);
void            TIM_vDeinit             (TIM_HandleType * pxTIM);

void            TIM_vIRQHandler         (TIM_HandleType * pxTIM);

void            TIM_vBurstConfig        (TIM_HandleType * pxTIM,
                                         __IO uint32_t * pulStartReg, uint8_t ucRegCount);

void            TIM_vStart              (TIM_HandleType * pxTIM);
void            TIM_vStop               (TIM_HandleType * pxTIM);

XPD_ReturnType  TIM_eOpen_DMA           (TIM_HandleType * pxTIM, TIM_EventType eEvent,
                                         void * pvAddress, uint16_t usLength);
void            TIM_vClose_DMA          (TIM_HandleType * pxTIM, TIM_EventType eEvent);

/**
 * @brief Provides the current TIM counter reference to get or set its value.
 * @param pxTIM: pointer to the TIM handle structure
 * @return Reference to the counter
 */
__STATIC_INLINE __IO uint32_t* TIM_pulCounter(TIM_HandleType * pxTIM)
{
    return &pxTIM->Inst->CNT;
}

/**
 * @brief Provides the current TIM reload reference to get or set its value.
 * @param pxTIM: pointer to the TIM handle structure
 * @return Reference to the reload
 */
__STATIC_INLINE __IO uint32_t* TIM_pulReload(TIM_HandleType * pxTIM)
{
    return &pxTIM->Inst->ARR;
}

/**
 * @brief Provides the current TIM repetition reference to get or set its value.
 * @param pxTIM: pointer to the TIM handle structure
 * @return Reference to the repetition
 */
__STATIC_INLINE __IO uint32_t* TIM_pulRepetition(TIM_HandleType * pxTIM)
{
    return &pxTIM->Inst->RCR;
}

/**
 * @brief Generate a TIM event.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eEvent: specifies the event to generate
 */
__STATIC_INLINE void TIM_vGenerate(TIM_HandleType * pxTIM, TIM_EventType eEvent)
{
    pxTIM->Inst->EGR.w = 1 << eEvent;
}

/** @} */

/** @} */

/** @defgroup TIM_Counter TIM Counter
 * @{ */

/** @addtogroup TIM_Counter_Exported_Functions
 * @{ */
void            TIM_vCounterInit        (TIM_HandleType * pxTIM, const TIM_InitType * pxConfig);

void            TIM_vCounterStart_IT    (TIM_HandleType * pxTIM);
void            TIM_vCounterStop_IT     (TIM_HandleType * pxTIM);

void            TIM_vIRQHandler_UP      (TIM_HandleType * pxTIM);

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
 * @brief Returns the current status of the TIM counter.
 * @param pxTIM: pointer to the TIM handle structure
 * @return TRUE if the counter is running, FALSE if stopped
 */
__STATIC_INLINE boolean_t TIM_bCounterIsRunning(TIM_HandleType * pxTIM)
{
    return TIM_REG_BIT(pxTIM,CR1,CEN);
}

/** @} */

/** @} */

/** @defgroup TIM_Channel TIM Channel Control
 * @{ */

/** @addtogroup TIM_Channel_Exported_Functions
 * @{ */
void            TIM_vIRQHandler_CC      (TIM_HandleType * pxTIM);

/**
 * @brief  Gets or sets the current TIM channel value.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel.
 * @return The value of the channel.
 */
__STATIC_INLINE __IO uint32_t* TIM_pulChannel(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    __IO uint32_t* pulCh = &pxTIM->Inst->CCR1 + eChannel;
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
    if (eChannel > TIM_CH4)
    {
        pulCh += 5;
    }
#endif
    return pulCh;
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
__STATIC_INLINE void TIM_vInvChannelEnable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
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
__STATIC_INLINE void TIM_vInvChannelDisable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
#ifdef TIM_BB
    *(&pxTIM->Inst_BB->CCER.CC1NE + (4 * eChannel)) = 0;
#else
    CLEAR_BIT(pxTIM->Inst->CCER.w, TIM_CCER_CC1NE << (4 * eChannel));
#endif
}

/**
 * @brief Enables the selected capture/compare channel interrupt.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the channel to enable
 */
__STATIC_INLINE void TIM_vChannelITEnable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    SET_BIT(pxTIM->Inst->DIER.w, TIM_DIER_CC1IE << eChannel);
}

/**
 * @brief Disables the selected capture/compare channel interrupt.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the channel to disable
 */
__STATIC_INLINE void TIM_vChannelITDisable(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    CLEAR_BIT(pxTIM->Inst->DIER.w, TIM_DIER_CC1IE << eChannel);
}

/**
 * @brief Indicates the selected capture/compare channel's interrupt request status.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the channel to query
 * @return Status of the interrupt flag
 */
__STATIC_INLINE FlagStatus TIM_vChannelGetFlag(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    return pxTIM->Inst->SR.w & (TIM_SR_CC1IF << eChannel);
}

/**
 * @brief Clears the selected capture/compare channel's interrupt request.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the channel to clear
 */
__STATIC_INLINE void TIM_vChannelClearFlag(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    CLEAR_BIT(pxTIM->Inst->SR.w, (TIM_SR_CC1IF << eChannel));
}

/** @} */

/** @} */

/** @defgroup TIM_Output TIM Output
 * @{ */

/** @defgroup TIM_Output_Exported_Types TIM Output Exported Types
 * @{ */

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

/** @} */

/** @} */

/** @defgroup TIM_Input TIM Input
 * @{ */

/** @defgroup TIM_Input_Exported_Functions TIM Input Exported Functions
 * {@ */

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
