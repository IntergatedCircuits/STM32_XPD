/**
  ******************************************************************************
  * @file    xpd_tim.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-04-30
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
#ifndef XPD_TIM_H_
#define XPD_TIM_H_

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
        XPD_HandleCallbackType Break;        /*!< Break callback */
        XPD_HandleCallbackType Commutation;  /*!< Commutation callback */
#ifdef USE_XPD_DMA_ERROR_DETECT
        XPD_HandleCallbackType Error;        /*!< DMA error callback */
#endif
    }Callbacks;                              /*   Handle Callbacks */
    struct {
        DMA_HandleType * Update;             /*!< DMA handle for update transfer */
        DMA_HandleType * Channel[4];         /*!< DMA handles for channel transfers */
        DMA_HandleType * Commutation;        /*!< DMA handle for commutation transfer */
        DMA_HandleType * Trigger;            /*!< DMA handle for trigger transfer */
    }DMA;                                    /*   DMA handle references */
    volatile TIM_ChannelType ActiveChannel;  /*!< The currently active timer channel */
}TIM_HandleType;

/** @} */

/** @defgroup TIM_Common_Exported_Macros TIM Common Exported Macros
 * @{ */

#ifdef USE_XPD_DMA_ERROR_DETECT
/**
 * @brief  TIM Handle initializer macro
 * @param  INSTANCE: specifies the TIM peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_TIM_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)               \
    {.Inst      = (INSTANCE),                                            \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL,NULL,NULL,NULL,NULL}, \
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl}
#else
/**
 * @brief  TIM Handle initializer macro
 * @param  INSTANCE: specifies the TIM peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_TIM_HANDLE(INSTANCE,INIT_FN,DEINIT_FN)               \
    {.Inst      = (INSTANCE),                                            \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL,NULL,NULL,NULL},      \
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl}
#endif

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
#define         XPD_TIM_EnableIT(HANDLE, IT_NAME)   \
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
#define         XPD_TIM_DisableIT(HANDLE, IT_NAME)  \
    (TIM_REG_BIT((HANDLE),DIER,IT_NAME##IE) = 0)

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
#define         XPD_TIM_GetFlag(HANDLE, FLAG_NAME)  \
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
#define         XPD_TIM_ClearFlag(HANDLE,FLAG_NAME) \
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
#define         XPD_TIM_GenerateEvent(HANDLE,FLAG_NAME) \
    (TIM_REG_BIT((HANDLE),EGR,FLAG_NAME##G) = 1)

/**
 * @brief Gets the TIM counter direction.
 * @param htim: pointer to the TIM handle structure
 * @return The current counter direction (up/down)
 */
#define         XPD_TIM_Counter_Direction(HANDLE)       \
    (TIM_REG_BIT((HANDLE), CR1, DIR))

/**
 * @brief Gets the current TIM counter value.
 * @param htim: pointer to the TIM handle structure
 * @return The value of the counter
 */
#define         XPD_TIM_Counter_Value(HANDLE)           \
    ((HANDLE)->Inst->CNT)

/**
 * @brief Gets the current TIM reload value.
 * @param htim: pointer to the TIM handle structure
 * @return The value of the reload register
 */
#define         XPD_TIM_Counter_Reload(HANDLE)          \
    ((HANDLE)->Inst->ARR)

/**
 * @brief Gets the current TIM repetition counter value.
 * @param htim: pointer to the TIM handle structure
 * @return The value of the repetition counter
 */
#define         XPD_TIM_Counter_Repetition(HANDLE)      \
    ((HANDLE)->Inst->RCR)


#define         XPD_TIM_Channel_GetFlag(HANDLE, CH)  \
    ((HANDLE)->Inst->SR.w & (TIM_SR_CC1IF << (CH)))

#define         XPD_TIM_Channel_ClearFlag(HANDLE, CH)\
    SET_BIT((HANDLE)->Inst->SR.w,(TIM_SR_CC1IF << (CH)))

#define         XPD_TIM_Channel_EnableIT(HANDLE, CH) \
    SET_BIT((HANDLE)->Inst->DIER.w,(TIM_DIER_CC1IE << (CH)))

#define         XPD_TIM_Channel_DisableIT(HANDLE, CH)\
    CLEAR_BIT((HANDLE)->Inst->DIER.w,(TIM_DIER_CC1IE << (CH)))

#define         XPD_TIM_Channel_EnableDMA(HANDLE, CH)\
    SET_BIT((HANDLE)->Inst->DIER.w,(TIM_DIER_CC1DE << (CH)))

#define         XPD_TIM_Channel_DisableDMA(HANDLE, CH)\
    CLEAR_BIT((HANDLE)->Inst->DIER.w,(TIM_DIER_CC1DE << (CH)))

#ifdef TIM_BB
#define TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#else
#define TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)
#endif

/** @} */

/** @addtogroup TIM_Common_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_TIM_Init                (TIM_HandleType * htim, TIM_Counter_InitType * Config);
XPD_ReturnType  XPD_TIM_Deinit              (TIM_HandleType * htim);

void            XPD_TIM_Counter_Start       (TIM_HandleType * htim);
void            XPD_TIM_Counter_Stop        (TIM_HandleType * htim);
void            XPD_TIM_Counter_Start_IT    (TIM_HandleType * htim);
void            XPD_TIM_Counter_Stop_IT     (TIM_HandleType * htim);
XPD_ReturnType  XPD_TIM_Counter_Start_DMA   (TIM_HandleType * htim, void * Address, uint16_t Length);
void            XPD_TIM_Counter_Stop_DMA    (TIM_HandleType * htim);

TIM_ChannelType XPD_TIM_Channel_GetActive   (TIM_HandleType * htim);

void            XPD_TIM_Channel_Enable      (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Channel_Disable     (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Channel_SetPulse    (TIM_HandleType * htim, TIM_ChannelType Channel, uint32_t Pulse);
uint32_t        XPD_TIM_Channel_GetPulse    (TIM_HandleType * htim, TIM_ChannelType Channel);

void            XPD_TIM_CompChannel_Enable  (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_CompChannel_Disable (TIM_HandleType * htim, TIM_ChannelType Channel);

void            XPD_TIM_UP_IRQHandler       (TIM_HandleType * htim);
void            XPD_TIM_CC_IRQHandler       (TIM_HandleType * htim);
void            XPD_TIM_TRG_COM_IRQHandler  (TIM_HandleType * htim);
void            XPD_TIM_BRK_IRQHandler      (TIM_HandleType * htim);
void            XPD_TIM_IRQHandler          (TIM_HandleType * htim);
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
    TIM_Output_ModeType Output;      /*!< Output channel mode
                                          @note Cannot be modified at LockLevel {3} */
    struct {
        ActiveLevelType ActiveLevel; /*!< Output channel active level
                                          @note Cannot be modified at LockLevel {2, 3} */
        FlagStatus      IdleState;   /*!< Output channel idle state
                                          @note Cannot be modified at LockLevel {1, 2, 3} */
    } Channel;
    struct {
        ActiveLevelType ActiveLevel; /*!< Complementary output channel active level
                                          @note Cannot be modified at LockLevel {2, 3} */
        FlagStatus      IdleState;   /*!< Complementary output channel idle state
                                          @note Cannot be modified at LockLevel {1, 2, 3} */
    } CompChannel;
} TIM_Output_InitType;

/** @brief TIM Main Output drive setup structure */
typedef struct
{
    uint8_t         LockLevel;       /*!< [0 .. 3] Depending on value, locks selected configuration bits
                                          @note Can only be set once after each reset */
    FunctionalState AutomaticOutput; /*!< Automatic output (MOE is set by software or update event)
                                          @note Cannot be modified from LockLevel 1 */
    FunctionalState IdleOffState;    /*!< Off-state selection for Idle mode
                                          @note Cannot be modified from LockLevel 2 */
    FunctionalState RunOffState;     /*!< Off-state selection for Run mode
                                          @note Cannot be modified from LockLevel 2 */
    struct {
        FunctionalState State;       /*!< Break function state
                                          @note Cannot be modified from LockLevel 1 */
        ActiveLevelType Polarity;    /*!< Break input polarity
                                          @note Cannot be modified from LockLevel 1 */
#ifdef TIM_BDTR_BKF
        uint8_t         Filter;      /*!< Break input capture filter
                                          @note Cannot be modified from LockLevel 1 */
#endif
    }Break;
#ifdef TIM_BDTR_BK2E
    struct {
        FunctionalState State;       /*!< Break2 function state
                                          @note Cannot be modified from LockLevel 1 */
        ActiveLevelType Polarity;    /*!< Break2 input polarity
                                          @note Cannot be modified from LockLevel 1 */
        uint8_t         Filter;      /*!< Break2 input capture filter
                                          @note Cannot be modified from LockLevel 1 */
    }Break2;
#endif
}TIM_Output_DriveType;

/** @} */

/** @addtogroup TIM_Output_Exported_Functions
 * @{ */
void            XPD_TIM_Output_Enable       (TIM_HandleType * htim);
void            XPD_TIM_Output_Disable      (TIM_HandleType * htim);

void            XPD_TIM_Output_Init         (TIM_HandleType * htim, TIM_ChannelType Channel,
                                             TIM_Output_InitType * Config);
void            XPD_TIM_Output_Start        (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Output_Stop         (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Output_Start_IT     (TIM_HandleType * htim, TIM_ChannelType Channel);
void            XPD_TIM_Output_Stop_IT      (TIM_HandleType * htim, TIM_ChannelType Channel);
XPD_ReturnType  XPD_TIM_Output_Start_DMA    (TIM_HandleType * htim, TIM_ChannelType Channel,
                                             void * Address, uint16_t Length);
void            XPD_TIM_Output_Stop_DMA     (TIM_HandleType * htim, TIM_ChannelType Channel);

void            XPD_TIM_Output_SetDeadtime  (TIM_HandleType * htim, uint32_t DeadCounts);
void            XPD_TIM_Output_DriveConfig  (TIM_HandleType * htim, TIM_Output_DriveType * Config);
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
    FunctionalState        MasterMode;     /*!< Master mode configuration */
    TIM_TriggerOutputType  MasterTrigger;  /*!< Trigger output (TRGO) selection */
#ifdef TIM_CR2_MMS2
    TIM_TriggerOutput2Type MasterTrigger2; /*!< Trigger output 2 (TRGO2) selection */
#endif
}TIM_MasterConfigType;

/** @} */

/** @addtogroup TIM_MasterSlave_Exported_Functions
 * @{ */
void            XPD_TIM_MasterConfig        (TIM_HandleType * htim, TIM_MasterConfigType * Config);
/** @} */

/** @} */

/** @} */

#define XPD_TIM_API
#include "xpd_rcc_gen.h"
#include "xpd_rcc_pc.h"
#undef XPD_TIM_API

#endif /* XPD_TIM_H_ */
