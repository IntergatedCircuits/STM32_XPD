/**
  ******************************************************************************
  * @file    xpd_tim.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2016-05-28
  * @brief   STM32 eXtensible Peripheral Drivers Timer Module
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
#ifndef __XPD_TIM_H_
#define __XPD_TIM_H_

#include "xpd_common.h"
#include "xpd_config.h"
#include "xpd_dma.h"

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
} TIM_Counter_InitType;

/** @brief TIM channels */
typedef enum
{
    TIM_CHANNEL_1 = 0, /*!< TIM channel 1 */
    TIM_CHANNEL_2 = 1, /*!< TIM channel 2 */
    TIM_CHANNEL_3 = 2, /*!< TIM channel 3 */
    TIM_CHANNEL_4 = 3, /*!< TIM channel 4 */
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
    TIM_CHANNEL_5 = 4, /*!< TIM channel 5 (limited capabilities: no IT or DMA) */
    TIM_CHANNEL_6 = 5  /*!< TIM channel 6 (limited capabilities: no IT or DMA) */
#endif
}TIM_ChannelType;

/** @brief TIM Handle structure */
typedef struct
{
    TIM_TypeDef * Inst;                      /*!< The address of the peripheral instance used by the handle */
#ifdef TIM_BB
    TIM_BitBand_TypeDef * Inst_BB;           /*!< The address of the peripheral instance in the bit-band region */
#endif
    XPD_CtrlFnType ClockCtrl;                /*!< Function pointer for RCC clock control */
    struct {
        XPD_HandleCallbackType DepInit;      /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;    /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType Update;       /*!< Timer update callback */
        XPD_HandleCallbackType ChannelEvent; /*!< Channel compare/capture callback */
        XPD_HandleCallbackType Trigger;      /*!< Trigger callback */
        XPD_HandleCallbackType Commutation;  /*!< Commutation callback */
        XPD_HandleCallbackType Break;        /*!< Break callback */
#ifdef USE_XPD_DMA_ERROR_DETECT
        XPD_HandleCallbackType Error;        /*!< DMA error callback */
#endif
    }Callbacks;                              /*   Handle Callbacks */
    struct {
        DMA_HandleType * Update;             /*!< DMA handle for update transfer */
        DMA_HandleType * Channel[4];         /*!< DMA handles for channel transfers */
        DMA_HandleType * Burst;              /*!< DMA handle for burst transfers */
    }DMA;                                    /*   DMA handle references */
    volatile TIM_ChannelType ActiveChannel;  /*!< The currently active timer channel */
}TIM_HandleType;

/** @} */

/** @defgroup TIM_Common_Exported_Macros TIM Common Exported Macros
 * @{ */

#ifdef TIM_BB
/**
 * @brief  TIM Handle initializer macro
 * @param  INSTANCE: specifies the TIM peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_TIM_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)  \
    {.Inst = (INSTANCE), .Inst_BB = TIM_BB(INSTANCE),       \
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl,               \
     .Callbacks.DepInit   = (INIT_FN),                      \
     .Callbacks.DepDeinit = (DEINIT_FN)}

/**
 * @brief TIM register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define         TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME)     \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief  TIM Handle initializer macro
 * @param  INSTANCE: specifies the TIM peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_TIM_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)  \
    {.Inst = (INSTANCE),                                    \
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl,               \
     .Callbacks.DepInit   = (INIT_FN),                      \
     .Callbacks.DepDeinit = (DEINIT_FN)}

/**
 * @brief TIM register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
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
#define         XPD_TIM_EnableIT(HANDLE, IT_NAME)           \
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
#define         XPD_TIM_DisableIT(HANDLE, IT_NAME)          \
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
#define         XPD_TIM_EnableDMA(HANDLE, DMA_NAME)         \
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
#define         XPD_TIM_DisableDMA(HANDLE, DMA_NAME)        \
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
#define         XPD_TIM_GetFlag(HANDLE, FLAG_NAME)          \
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
#define         XPD_TIM_ClearFlag(HANDLE,FLAG_NAME)         \
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
 *            @arg B:       Break
 */
#define         XPD_TIM_GenerateEvent(HANDLE,FLAG_NAME)     \
    (TIM_REG_BIT((HANDLE),EGR,FLAG_NAME##G) = 1)

/**
 * @brief Gets or sets the TIM counter direction.
 * @param htim: pointer to the TIM handle structure
 * @return The current counter direction (up/down)
 */
#define         XPD_TIM_Counter_Direction(HANDLE)           \
    (TIM_REG_BIT((HANDLE), CR1, DIR))

/**
 * @brief Gets or sets the current TIM counter value.
 * @param htim: pointer to the TIM handle structure
 * @return The value of the counter
 */
#define         XPD_TIM_Counter_Value(HANDLE)               \
    ((HANDLE)->Inst->CNT)

/**
 * @brief Gets or sets the current TIM reload value.
 * @param htim: pointer to the TIM handle structure
 * @return The value of the reload register
 */
#define         XPD_TIM_Counter_Reload(HANDLE)              \
    ((HANDLE)->Inst->ARR)

/**
 * @brief Gets or sets the current TIM repetition counter value.
 * @param htim: pointer to the TIM handle structure
 * @return The value of the repetition counter
 */
#define         XPD_TIM_Counter_Repetition(HANDLE)          \
    ((HANDLE)->Inst->RCR)

/**
 * @brief  Enable the specified TIM Channel interrupt.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel interrupt to enable.
 */
#define         XPD_TIM_Channel_EnableIT(HANDLE, CH)        \
    SET_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1IE << (CH))

/**
 * @brief  Disable the specified TIM Channel interrupt.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel interrupt to disable.
 */
#define         XPD_TIM_Channel_DisableIT(HANDLE, CH)       \
    CLEAR_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1IE << (CH))

/**
 * @brief  Enable the specified TIM Channel DMA transfer requests.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel DMA requests to enable.
 */
#define         XPD_TIM_Channel_EnableDMA(HANDLE, CH)       \
    SET_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1DE << (CH))

/**
 * @brief  Disable the specified TIM Channel DMA transfer requests.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel DMA requests to disable.
 */
#define         XPD_TIM_Channel_DisableDMA(HANDLE, CH)      \
    CLEAR_BIT((HANDLE)->Inst->DIER.w, TIM_DIER_CC1DE << (CH))

/**
 * @brief  Get the specified TIM channel flag.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel flag to return.
 */
#define         XPD_TIM_Channel_GetFlag(HANDLE, CH)         \
    ((HANDLE)->Inst->SR.w & (TIM_SR_CC1IF << (CH)))

/**
 * @brief  Clear the specified TIM channel flag.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel flag to clear.
 */
#define         XPD_TIM_Channel_ClearFlag(HANDLE, CH)       \
    CLEAR_BIT((HANDLE)->Inst->SR.w, TIM_SR_CC1IF << (CH))

#if TIM_SUPPORTED_CHANNEL_COUNT > 5
/**
 * @brief  Gets or sets the current TIM channel value.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel.
 * @return The value of the channel.
 */
#define         XPD_TIM_Channel_Value(HANDLE, CH)           \
    ((&(HANDLE)->Inst->CCR1)[((CH) < TIM_CHANNEL_5)         \
                            ? (CH) : ((CH) + 5)])
#else
/**
 * @brief  Gets or sets the current TIM channel value.
 * @param  HANDLE: specifies the TIM Handle.
 * @param  CH: Number of TIM channel.
 * @return The value of the channel.
 */
#define         XPD_TIM_Channel_Value(HANDLE, CH)           \
    ((&(HANDLE)->Inst->CCR1)[ CH ]))
#endif

/** @} */

/** @addtogroup TIM_Common_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_TIM_Init                (TIM_HandleType * htim, const TIM_Counter_InitType * Config);
XPD_ReturnType  XPD_TIM_Deinit              (TIM_HandleType * htim);

void            XPD_TIM_Counter_Start_IT    (TIM_HandleType * htim);
void            XPD_TIM_Counter_Stop_IT     (TIM_HandleType * htim);
XPD_ReturnType  XPD_TIM_Counter_Start_DMA   (TIM_HandleType * htim, void * Address, uint16_t Length);
void            XPD_TIM_Counter_Stop_DMA    (TIM_HandleType * htim);

void            XPD_TIM_Channel_Start       (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Channel_Stop        (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Channel_Start_IT    (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Channel_Stop_IT     (TIM_HandleType * htim, TIM_ChannelType Channel);
XPD_ReturnType  XPD_TIM_Channel_Start_DMA   (TIM_HandleType * htim, TIM_ChannelType Channel,
                                             void * Address, uint16_t Length);
void            XPD_TIM_Channel_Stop_DMA    (TIM_HandleType * htim, TIM_ChannelType Channel);

void            XPD_TIM_UP_IRQHandler       (TIM_HandleType * htim);
void            XPD_TIM_CC_IRQHandler       (TIM_HandleType * htim);
void            XPD_TIM_TRG_COM_IRQHandler  (TIM_HandleType * htim);
void            XPD_TIM_BRK_IRQHandler      (TIM_HandleType * htim);
void            XPD_TIM_IRQHandler          (TIM_HandleType * htim);

/**
 * @brief Enables the TIM counter.
 * @param htim: pointer to the TIM handle structure
 */
__STATIC_INLINE void XPD_TIM_Counter_Start(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,CR1,CEN) = 1;
}

/**
 * @brief Disables the TIM counter.
 * @param htim: pointer to the TIM handle structure
 */
__STATIC_INLINE void XPD_TIM_Counter_Stop(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,CR1,CEN) = 0;
}

/**
 * @brief Enables the selected capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the channel to enable
 */
__STATIC_INLINE void XPD_TIM_Channel_Enable(TIM_HandleType * htim, TIM_ChannelType Channel)
{
#ifdef TIM_BB
    *(&htim->Inst_BB->CCER.CC1E + (4 * Channel)) = 1;
#else
    SET_BIT(htim->Inst->CCER.w, TIM_CCER_CC1E << (4 * Channel));
#endif
}

/**
 * @brief Disables the selected capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the channel to disable
 */
__STATIC_INLINE void XPD_TIM_Channel_Disable(TIM_HandleType * htim, TIM_ChannelType Channel)
{
#ifdef TIM_BB
    *(&htim->Inst_BB->CCER.CC1E + (4 * Channel)) = 0;
#else
    CLEAR_BIT(htim->Inst->CCER.w, TIM_CCER_CC1E << (4 * Channel));
#endif
}

/**
 * @brief Enables the selected complementary capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the complementary channel to enable
 */
__STATIC_INLINE void XPD_TIM_CompChannel_Enable(TIM_HandleType * htim, TIM_ChannelType Channel)
{
#ifdef TIM_BB
    *(&htim->Inst_BB->CCER.CC1NE + (4 * Channel)) = 1;
#else
    SET_BIT(htim->Inst->CCER.w, TIM_CCER_CC1NE << (4 * Channel));
#endif
}

/**
 * @brief Disables the selected complementary capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the complementary channel to disable
 */
__STATIC_INLINE void XPD_TIM_CompChannel_Disable(TIM_HandleType * htim, TIM_ChannelType Channel)
{
#ifdef TIM_BB
    *(&htim->Inst_BB->CCER.CC1NE + (4 * Channel)) = 0;
#else
    CLEAR_BIT(htim->Inst->CCER.w, TIM_CCER_CC1NE << (4 * Channel));
#endif
}

/** @} */

/** @} */

/** @defgroup TIM_Burst TIM Burst DMA Transfer Mode
 * @{ */

/** @defgroup TIM_Burst_Exported_Types TIM Burst DMA Exported Types
 * @{ */

/** @brief TIM burst DMA selection */
typedef enum
{
    TIM_BURSTSOURCE_UPDATE    = 0, /*!< TIM counter value updated event */
    TIM_BURSTSOURCE_CHANNEL_1 = 1, /*!< TIM Capture/Compare channel 1 event */
    TIM_BURSTSOURCE_CHANNEL_2 = 2, /*!< TIM Capture/Compare channel 2 event */
    TIM_BURSTSOURCE_CHANNEL_3 = 3, /*!< TIM Capture/Compare channel 3 event */
    TIM_BURSTSOURCE_CHANNEL_4 = 4, /*!< TIM Capture/Compare channel 4 event */
    TIM_BURSTSOURCE_COM       = 5, /*!< TIM Capture/Compare control update event */
    TIM_BURSTSOURCE_TRIGGER   = 6, /*!< TIM Trigger event */
}TIM_BurstSourceType;

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
}TIM_Burst_RegIndexType;

/** @brief TIM DMA burst setup structure */
typedef struct
{
    TIM_Burst_RegIndexType RegIndex; /*!< TIM register offset starting from the timer base */
    TIM_BurstSourceType    Source;   /*!< TIM burst DMA and trigger source */
}TIM_Burst_InitType;

/** @} */

/** @addtogroup TIM_Burst_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_TIM_Burst_Start_DMA     (TIM_HandleType * htim, const TIM_Burst_InitType * Config,
                                             void * Address, uint16_t Length);
void            XPD_TIM_Burst_Stop_DMA      (TIM_HandleType * htim, TIM_BurstSourceType Source);
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
}TIM_Output_ModeType;

/** @brief TIM output channel setup structure */
typedef struct
{
    TIM_Output_ModeType Mode;            /*!< Output channel mode */
    ActiveLevelType     Polarity;        /*!< Output channel active level */
    FlagStatus          IdleState;       /*!< Output channel idle state */
    ActiveLevelType     CompPolarity;    /*!< Complementary output channel active level */
    FlagStatus          CompIdleState;   /*!< Complementary output channel idle state */
}TIM_Output_InitType;

/** @brief TIM Main Output drive setup structure */
typedef struct
{
    uint16_t        DeadCounts;      /*!< The dead time between the complementary outputs in deadtime clock counts */
    FunctionalState AutomaticOutput; /*!< Automatic output (MOE is set by software or update event) */
    FunctionalState IdleOffState;    /*!< Off-state selection for Idle mode */
    FunctionalState RunOffState;     /*!< Off-state selection for Run mode */
}TIM_Output_DriveType;

/** @brief TIM Break setup structure */
typedef struct
{
    FunctionalState            State;    /*!< Break function state */
    ActiveLevelType            Polarity; /*!< Break input polarity */
#ifdef TIM_BDTR_BKF
    uint8_t                    Filter;   /*!< Break input capture filter */
#endif
}TIM_Output_BreakInitType;

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
void            XPD_TIM_Output_ChannelConfig(TIM_HandleType * htim, TIM_ChannelType Channel,
                                             const TIM_Output_InitType * Config);

void            XPD_TIM_Output_DriveConfig  (TIM_HandleType * htim, const TIM_Output_DriveType * Config);

void            XPD_TIM_Output_BreakConfig  (TIM_HandleType * htim, uint8_t BreakLine,
                                             const TIM_Output_BreakInitType * Config);

void            XPD_TIM_Output_OCRefClearConfig
                                            (TIM_HandleType * htim, TIM_ChannelType Channel,
                                             TIM_OCRefClearSourceType OCREF_CLR_IN);

void            XPD_TIM_Output_CommutationConfig
                                            (TIM_HandleType * htim,
                                             TIM_CommutationSourceType ComSource);

/**
 * @brief Sets the selected lock level to write-protect certain timer configuration registers.
 * @param htim: pointer to the TIM handle structure
 * @param LockLevel: the specified lock level to apply [0..3]
 */
__STATIC_INLINE void XPD_TIM_Output_SetLock(TIM_HandleType * htim, uint8_t LockLevel)
{
    htim->Inst->BDTR.b.LOCK = LockLevel;
}

/**
 * @brief Manually enables the main output of an advanced timer.
 * @param htim: pointer to the TIM handle structure
 */
__STATIC_INLINE void XPD_TIM_Output_Enable(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,BDTR,MOE) = 1;
}

/**
 * @brief Manually disables the main output of an advanced timer.
 * @param htim: pointer to the TIM handle structure
 */
__STATIC_INLINE void XPD_TIM_Output_Disable(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,BDTR,MOE) = 0;
}

/** @} */

/** @} */

/** @defgroup TIM_OnePulseMode TIM One Pulse Mode
 * @{ */

/** @defgroup TIM_OnePulseMode_Exported_Functions TIM One Pulse Mode Exported Functions
 * @{ */

/**
 * @brief Enables the one pulse mode of the timer output.
 * @param htim: pointer to the TIM handle structure
 */
__STATIC_INLINE void XPD_TIM_OnePulseMode_Enable(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim, CR1, OPM) = 1;
}

/**
 * @brief Disables the one pulse mode of the timer output.
 * @param htim: pointer to the TIM handle structure
 */
__STATIC_INLINE void XPD_TIM_OnePulseMode_Disable(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim, CR1, OPM) = 0;
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
}TIM_Input_SourceType;

/** @brief TIM input capture channel setup structure */
typedef struct
{
    TIM_Input_SourceType Source;    /*!< Specifies the input source */
    ActiveLevelType      Polarity;  /*!< Specifies the active edge of the input signal */
    ClockDividerType     Prescaler; /*!< Specifies the input capture prescaler [DIV1..DIV8] */
    uint8_t              Filter;    /*!< Specifies the input capture filter. [0..15] */
}TIM_Input_InitType;

/** @} */

/** @addtogroup TIM_Input_Exported_Functions
 * {@ */
void            XPD_TIM_Input_ChannelConfig (TIM_HandleType * htim, TIM_ChannelType Channel,
                                             const TIM_Input_InitType * Config);

/**
 * @brief Sets the TI1 input source as XOR(Ch1, Ch2, Ch3) instead of default Ch1.
 * @param htim: pointer to the TIM handle structure
 * @param NewState: Input channels XOR selection
 */
__STATIC_INLINE void XPD_TIM_Input_ChannelsXOR(TIM_HandleType * htim, FunctionalState NewState)
{
    TIM_REG_BIT(htim,CR2,TI1S) = NewState;
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
    TIM_TRGO_OC1    = 3, /*!< TRGO is connected to the Channel 1 comparator on match */
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
void            XPD_TIM_MasterConfig        (TIM_HandleType * htim, const TIM_MasterConfigType * Config);
void            XPD_TIM_SlaveConfig         (TIM_HandleType * htim, const TIM_SlaveConfigType * Config);
/** @} */

/** @} */

/** @} */

#define XPD_TIM_API
#include "xpd_rcc_gen.h"
#include "xpd_rcc_pc.h"
#undef XPD_TIM_API

#endif /* __XPD_TIM_H_ */
