/**
  ******************************************************************************
  * @file    xpd_tim.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-26
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

#include "xpd_tim.h"
#include "xpd_utils.h"

#ifdef USE_XPD_TIM

/** @addtogroup TIM
 * @{ */

/** @addtogroup TIM_Common
 * @{ */

#define TIM_CONVERT_CH(Channel)     (((uint32_t)Channel) * 4)

#define TIM_ALL_CHANNELS (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E |\
                          TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE)

#define TIM_ACTIVE_CHANNELS(HANDLE) (HANDLE->Inst->CCER.w & TIM_ALL_CHANNELS)

#ifdef USE_XPD_DMA_ERROR_DETECT
static void tim_dmaErrorRedirect(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    XPD_SAFE_CALLBACK(htim->Callbacks.Error, htim);
}
#endif
static void tim_dmaUpdateRedirect(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    XPD_SAFE_CALLBACK(htim->Callbacks.Update, htim);
}
static void tim_dmaChannelEventRedirect1(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    htim->ActiveChannel = TIM_CHANNEL_1;
    XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent, htim);
}
static void tim_dmaChannelEventRedirect2(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    htim->ActiveChannel = TIM_CHANNEL_2;
    XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent, htim);
}
static void tim_dmaChannelEventRedirect3(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    htim->ActiveChannel = TIM_CHANNEL_3;
    XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent, htim);
}
static void tim_dmaChannelEventRedirect4(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    htim->ActiveChannel = TIM_CHANNEL_4;
    XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent,htim);
}
static const XPD_HandleCallbackType tim_dmaChannelEventRedirects[4] = {
        tim_dmaChannelEventRedirect1,
        tim_dmaChannelEventRedirect2,
        tim_dmaChannelEventRedirect3,
        tim_dmaChannelEventRedirect4,
};
static void tim_dmaTriggerCallbackRedirect(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    XPD_SAFE_CALLBACK(htim->Callbacks.Trigger,htim);
}
static void tim_dmaCommutationCallbackRedirect(void *hdma)
{
    TIM_HandleType* htim = (TIM_HandleType*) ((DMA_HandleType*) hdma)->Owner;

    XPD_SAFE_CALLBACK(htim->Callbacks.Commutation,htim);
}

/** @defgroup TIM_Common_Exported_Functions TIM Common Exported Functions
 *  @brief    TIM common functions (timer, channels control)
 * @{ */

/**
 * @brief Initializes the TIM peripheral using the setup configuration.
 * @param htim: pointer to the TIM handle structure
 * @param Config: pointer to TIM setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_TIM_Init(TIM_HandleType * htim, TIM_CounterInitType * Config)
{
    /* enable clock */
    XPD_SAFE_CALLBACK(htim->ClockCtrl, ENABLE);

#ifdef TIM_BB
    htim->Inst_BB = TIM_BB(htim->Inst);
#endif

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(htim->Callbacks.DepInit,htim);

    TIM_REG_BIT(htim, CR1, DIR) = Config->Mode;
    htim->Inst->CR1.b.CMS       = Config->Mode >> 1;
    htim->Inst->CR1.b.CKD       = Config->ClockDivision;

    htim->Inst->ARR             = (uint32_t)(Config->Period - 1);
    htim->Inst->PSC             = (uint32_t)(Config->Prescaler - 1);

    htim->Inst->RCR             = Config->RepetitionCounter;

    /* Generate an update event to reload the prescaler
     and the repetition counter (only for TIM1 and TIM8) value immediately */
    TIM_REG_BIT(htim,EGR,UG) = 1;

    return XPD_OK;
}

/**
 * @brief Deinitializes the TIM peripheral.
 * @param htim: pointer to the TIM handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_TIM_Deinit(TIM_HandleType * htim)
{
    XPD_TIM_OutputDisable(htim);

    /* disable all channels */
    CLEAR_BIT(htim->Inst->CCER.w, TIM_ALL_CHANNELS);

    /* disable the TIM Peripheral Clock */
    XPD_TIM_CounterStop(htim);

    /* disable clock */
    XPD_SAFE_CALLBACK(htim->ClockCtrl, DISABLE);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(htim->Callbacks.DepDeinit,htim);

    return XPD_OK;
}

/**
 * @brief Enables the TIM counter.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_CounterStart(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,CR1,CEN) = 1;
}

/**
 * @brief Disables the TIM counter.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_CounterStop(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,CR1,CEN) = 0;
}

/**
 * @brief Enables the TIM counter and update interrupt.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_CounterStart_IT(TIM_HandleType * htim)
{
    /* enable the TIM Update interrupt */
    XPD_TIM_EnableIT(htim, U);

    /* enable the counter */
    XPD_TIM_CounterStart(htim);
}

/**
 * @brief Disables the TIM counter and update interrupt.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_CounterStop_IT(TIM_HandleType * htim)
{
    XPD_TIM_DisableIT(htim, U);

    /* disable the counter */
    XPD_TIM_CounterStop(htim);
}

/**
 * @brief Sets up and enables a DMA transfer for the TIM counter.
 * @param htim: pointer to the TIM handle structure
 * @param Address: memory address of the counter data
 * @param Length: the amount of data to be transferred
 */
void XPD_TIM_CounterStart_DMA(TIM_HandleType * htim, void * Address, uint16_t Length)
{
    /* set the DMA complete callback */
    htim->DMA.Update->Callbacks.Complete = tim_dmaUpdateRedirect;

#ifdef USE_XPD_DMA_ERROR_DETECT
    /* set the DMA error callback */
    htim->DMA.Update->Callbacks.Error = tim_dmaErrorRedirect;
#endif

    /* configure the DMA channel */
    htim->DMA.Update->Transfer.DataCount     = Length;
    htim->DMA.Update->Transfer.SourceAddress = Address;
    htim->DMA.Update->Transfer.DestAddress   = (void *)&htim->Inst->ARR;

    XPD_DMA_Start_IT(htim->DMA.Update);

    /* enable the TIM Update DMA request */
    TIM_REG_BIT(htim, DIER, UDE) = 1;

    /* enable the counter */
    XPD_TIM_CounterStart(htim);
}

/**
 * @brief Disables the TIM counter and DMA transfer.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_CounterStop_DMA(TIM_HandleType * htim)
{
    /* disable the update DMA request */
    TIM_REG_BIT(htim, DIER, UDE) = 0;

    XPD_DMA_Stop_IT(htim->DMA.Update);

    /* disable the counter */
    XPD_TIM_CounterStop(htim);
}

/**
 * @brief Gets the TIM counter direction.
 * @param htim: pointer to the TIM handle structure
 * @return The current counter direction (up/down)
 */
TIM_CounterType XPD_TIM_CounterDirection(TIM_HandleType * htim)
{
    return TIM_REG_BIT(htim, CR1, DIR);
}

/**
 * @brief Gets the current TIM counter value.
 * @param htim: pointer to the TIM handle structure
 * @return The value of the counter
 */
uint32_t XPD_TIM_GetCounter(TIM_HandleType * htim)
{
    return htim->Inst->CNT;
}

/**
 * @brief Sets the TIM counter value.
 * @param htim: pointer to the TIM handle structure
 * @param Value: the new value of the counter
 */
void XPD_TIM_SetCounter(TIM_HandleType * htim, uint32_t Value)
{
    htim->Inst->CNT = Value;
}

/**
 * @brief Gets the currently active channel. Use this function to determine
 *        which TIM channel event generated the callback in the callback context.
 * @param htim: pointer to the TIM handle structure
 * @return The TIM channel event responsible for the last interrupt
 */
TIM_ChannelType XPD_TIM_GetCurrentChannel(TIM_HandleType * htim)
{
    return htim->ActiveChannel;
}

/**
 * @brief Enables the selected capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the channel to enable
 */
void XPD_TIM_ChannelEnable(TIM_HandleType *htim, TIM_ChannelType Channel)
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
void XPD_TIM_ChannelDisable(TIM_HandleType *htim, TIM_ChannelType Channel)
{
#ifdef TIM_BB
    *(&htim->Inst_BB->CCER.CC1E + (4 * Channel)) = 0;
#else
    CLEAR_BIT(htim->Inst->CCER.w, TIM_CCER_CC1E << (4 * Channel));
#endif
}

/**
 * @brief Sets the pulse of the selected capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the channel to configure
 * @param Pulse: the new value for the channel
 */
void XPD_TIM_ChannelSetPulse(TIM_HandleType * htim, TIM_ChannelType Channel, uint32_t Pulse)
{
    (&htim->Inst->CCR1)[Channel] = Pulse;
}

/**
 * @brief Gets the pulse of the selected capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the channel to read
 * @return The pulse of the channel
 */
uint32_t XPD_TIM_ChannelGetPulse(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    return (&htim->Inst->CCR1)[Channel];
}

/**
 * @brief Enables the selected complementary capture/compare channel.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the complementary channel to enable
 */
void XPD_TIM_CompChannelEnable(TIM_HandleType *htim, TIM_ChannelType Channel)
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
void XPD_TIM_CompChannelDisable(TIM_HandleType *htim, TIM_ChannelType Channel)
{
#ifdef TIM_BB
    *(&htim->Inst_BB->CCER.CC1NE + (4 * Channel)) = 0;
#else
    CLEAR_BIT(htim->Inst->CCER.w, TIM_CCER_CC1NE << (4 * Channel));
#endif
}

/**
 * @brief TIM update interrupt handler that provides handle callbacks.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_UP_IRQHandler(TIM_HandleType * htim)
{
    /* TIM update event */
    if (TIM_REG_BIT(htim, SR, UIF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim, SR, UIF) = 0;

        XPD_SAFE_CALLBACK(htim->Callbacks.Update,htim);
    }
}

/**
 * @brief TIM channel event interrupt handler that provides handle callbacks.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_CC_IRQHandler(TIM_HandleType * htim)
{
    uint32_t sr = htim->Inst->SR.w;

    /* capture compare 1 event */
    if ((sr & TIM_SR_CC1IF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim,SR,CC1IF) = 0;

        htim->ActiveChannel = TIM_CHANNEL_1;
        XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent,htim);
    }
    /* capture compare 2 event */
    if ((sr & TIM_SR_CC2IF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim,SR,CC2IF) = 0;

        htim->ActiveChannel = TIM_CHANNEL_2;
        XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent,htim);
    }
    /* capture compare 3 event */
    if ((sr & TIM_SR_CC3IF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim,SR,CC3IF) = 0;

        htim->ActiveChannel = TIM_CHANNEL_3;
        XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent,htim);
    }
    /* capture compare 4 event */
    if ((sr & TIM_SR_CC4IF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim,SR,CC4IF) = 0;

        htim->ActiveChannel = TIM_CHANNEL_4;
        XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent,htim);
    }
}

/**
 * @brief TIM break event interrupt handler that provides handle callbacks.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_BRK_IRQHandler(TIM_HandleType * htim)
{
    /* TIM Break input event */
    if (TIM_REG_BIT(htim, SR, BIF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim, SR, BIF) = 0;

        XPD_SAFE_CALLBACK(htim->Callbacks.Break, htim);
    }
}

/**
 * @brief TIM trigger and commutation interrupt handler that provides handle callbacks.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_TRG_COM_IRQHandler(TIM_HandleType * htim)
{
    uint32_t sr = htim->Inst->SR.w;

    /* TIM Trigger detection event */
    if ((sr & TIM_SR_TIF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim,SR,TIF) = 0;

        XPD_SAFE_CALLBACK(htim->Callbacks.Trigger, htim);
    }
    /* TIM commutation event */
    if ((sr & TIM_SR_COMIF) != 0)
    {
        /* clear interrupt flag */
        TIM_REG_BIT(htim,SR,COMIF) = 0;

        XPD_SAFE_CALLBACK(htim->Callbacks.Commutation, htim);
    }
}

/**
 * @brief TIM common interrupt handler that provides handle callbacks (break handling excluded).
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_IRQHandler(TIM_HandleType * htim)
{
    XPD_TIM_UP_IRQHandler(htim);
    XPD_TIM_CC_IRQHandler(htim);
    XPD_TIM_TRG_COM_IRQHandler(htim);
    /* leave out break interrupt, timers that have break also have separate interrupt line */
    /*XPD_TIM_BRK_IRQHandler(htim);*/
}

/** @} */

/** @} */

/** @addtogroup TIM_Output
 * @{ */

/** @defgroup TIM_Output_Exported_Functions TIM Output Exported Functions
 *  @brief    TIM output mode functions for compare channels
 *  @details  These functions provide API for TIM output channel modes (Output Compare/Pulse Width Modulation).
 * @{ */

/**
 * @brief Initializes a compare channel using the setup configuration.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 * @param Config: pointer to TIM compare channel setup configuration
 */
void XPD_TIM_OutputInit(TIM_HandleType * htim, TIM_ChannelType Channel, TIM_OutputChannelInitType * Config)
{
    XPD_EnterCritical(htim);

    /* set the output compare polarities (0 is active high) */
    {
        uint32_t pol = 0;

        if (Config->Channel.ActiveLevel == ACTIVE_LOW)
        {
            pol |= TIM_CCER_CC1P;
        }
        if (Config->CompChannel.ActiveLevel == ACTIVE_LOW)
        {
            pol |= TIM_CCER_CC1NP;
        }

        MODIFY_REG(htim->Inst->CCER.w, 0xF << (Channel * 4), pol << (Channel * 4));
    }
    /* output mode configuration */
    {
        __IO uint8_t pccmr[6];

        /* copy CCMR registers to unaligned array */
        if (Channel < TIM_CHANNEL_3)
        {
            *((uint32_t *)&pccmr[0]) = htim->Inst->CCMR1.w;
        }
        else
        {
            *((uint32_t *)&pccmr[2]) = htim->Inst->CCMR2.w;
        }

        /* add output configuration */
        pccmr[Channel] = (Config->Output << 4);

        /* for PWM modes, enable preload and fast mode */
        if (Config->Output >= TIM_OUTPUT_PWM1)
        {
            pccmr[Channel] |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE;
        }

        /* apply configuration */
        if (Channel < TIM_CHANNEL_3)
        {
            htim->Inst->CCMR1.w = *((uint32_t *)&pccmr[0]);
        }
        else
        {
            htim->Inst->CCMR2.w = *((uint32_t *)&pccmr[2]);
        }
    }

    /* Set the Output Idle states */
    {
        uint32_t idle;

        idle = Config->Channel.IdleState | (Config->CompChannel.IdleState << 1);
        idle = idle << (Channel * 2 + 8);

        MODIFY_REG(htim->Inst->CR2.w,(TIM_CR2_OIS1 | TIM_CR2_OIS1N) << (Channel * 2), idle);
    }

    XPD_ExitCritical(htim);
}

/**
 * @brief Enables the main output of an advanced timer (TIM1 / TIM8).
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_OutputEnable(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,BDTR,MOE) = 1;
}

/**
 * @brief Disables the main output of an advanced timer (TIM1 / TIM8).
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_OutputDisable(TIM_HandleType * htim)
{
    TIM_REG_BIT(htim,BDTR,MOE) = 0;
}

/**
 * @brief Starts the selected output channel (and the timer if required).
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 */
void XPD_TIM_OutputStart(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    uint32_t chstate = TIM_ACTIVE_CHANNELS(htim);

    XPD_TIM_ChannelEnable(htim, Channel);

    XPD_TIM_CompChannelEnable(htim, Channel);

    if (chstate == 0)
    {
        XPD_TIM_CounterStart(htim);
    }
}

/**
 * @brief Stops the selected output channel (and the timer if required).
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 */
void XPD_TIM_OutputStop(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    XPD_TIM_ChannelDisable(htim, Channel);

    XPD_TIM_CompChannelDisable(htim, Channel);

    if (TIM_ACTIVE_CHANNELS(htim) == 0)
    {
        XPD_TIM_CounterStop(htim);
    }
}

/**
 * @brief Starts the selected output channel (and the timer if required)
 *        and enables the channel event interrupt.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 */
void XPD_TIM_OutputStart_IT(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    SET_BIT(htim->Inst->DIER.w, (TIM_DIER_CC1IE << (uint32_t)Channel) | TIM_DIER_BIE);

    XPD_TIM_OutputStart(htim, Channel);
}

/**
 * @brief Stops the selected output channel (and the timer if required)
 *        and disables the channel event interrupt.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 */
void XPD_TIM_OutputStop_IT(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    CLEAR_BIT(htim->Inst->DIER.w, (TIM_DIER_CC1IE << (uint32_t)Channel) | TIM_DIER_BIE);

    XPD_TIM_OutputStop(htim, Channel);
}

/**
 * @brief Starts the selected output channel (and the timer if required)
 *        using a DMA transfer to provide channel pulse values.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 * @param Address: the memory start address of the pulse values
 * @param Length: the memory size of the pulse values
 */
void XPD_TIM_OutputStart_DMA(TIM_HandleType * htim, TIM_ChannelType Channel, void * Address, uint16_t Length)
{
    /* callbacks subscription */
    htim->DMA.Channel[Channel]->Callbacks.Complete = tim_dmaChannelEventRedirects[Channel];

#ifdef USE_XPD_DMA_ERROR_DETECT
    htim->DMA.Channel[Channel]->Callbacks.Error = tim_dmaErrorRedirect;
#endif

    /* configure the DMA channel */
    htim->DMA.Channel[Channel]->Transfer.DataCount     = Length;
    htim->DMA.Channel[Channel]->Transfer.SourceAddress = Address;
    htim->DMA.Channel[Channel]->Transfer.DestAddress   = (void *)&((&htim->Inst->CCR1)[Channel]);

    XPD_DMA_Start_IT(htim->DMA.Channel[Channel]);

    XPD_TIM_ChannelEnableDMA(htim, Channel);

    XPD_TIM_OutputStart(htim, Channel);
}

/**
 * @brief Stops the selected output channel (and the timer if required)
 *        and disables the DMA requests.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 */
void XPD_TIM_OutputStop_DMA(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    XPD_TIM_ChannelDisableDMA(htim, Channel);

    XPD_DMA_Stop_IT(htim->DMA.Channel[Channel]);

    XPD_TIM_OutputStop(htim, Channel);
}

/**
 * @brief Configures the dead time between the complementary outputs.
 * @param htim: pointer to the TIM handle structure
 * @param DeadCounts: The amount of dead time specified in deadtime clock counts
 */
void XPD_TIM_OutputSetDeadtime(TIM_HandleType * htim, uint32_t DeadCounts)
{
    uint8_t deadtime;

    if (DeadCounts < 128)
    {
        /* dead counts = DTG */
        deadtime = DeadCounts;
    }
    else if (DeadCounts < 256)
    {
        /* dead counts =(64+DTG[5:0])*2 */
        deadtime = 0x80 | ((DeadCounts / 2) - 64);
    }
    else if (DeadCounts < 512)
    {
        /* dead counts =(32+DTG[4:0])*8 */
        deadtime = 0xC0 | ((DeadCounts / 8) - 32);
    }
    else if (DeadCounts < 1008)
    {
        /* dead counts =(32+DTG[4:0])*16 */
        deadtime = 0xE0 | ((DeadCounts / 16) - 32);
    }
    else
    {
        /* saturation */
        deadtime = 255;
    }

    htim->Inst->BDTR.b.DTG = deadtime;
}

/**
 * @brief Sets up the break configuration of the timer.
 * @param htim: pointer to the TIM handle structure
 * @param Config: pointer to TIM break setup configuration
 */
void XPD_TIM_OutputBreakConfig(TIM_HandleType * htim, TIM_OutputBreakType * Config)
{
    MODIFY_REG(htim->Inst->BDTR.w, 0xFF00, Config->__BDTR1 << 8);
}

/** @} */

/** @} */

/** @addtogroup TIM_MasterSlave
 * @{ */

/** @defgroup TIM_MasterSlave_Exported_Functions TIM Master-Slave Exported Functions
 * @{ */

/**
 * @brief Sets up the master configuration of the timer.
 * @param htim: pointer to the TIM handle structure
 * @param Config: pointer to TIM master setup configuration
 */
void XPD_TIM_MasterConfig(TIM_HandleType * htim, TIM_MasterConfigType * Config)
{
    /* select the TRGO source */
    htim->Inst->CR2.b.MMS = Config->MasterTrigger;

    /* configure the MSM bit */
    TIM_REG_BIT(htim,SMCR,MSM) = Config->MasterMode;
}

/** @} */

/** @} */

/** @} */
#endif /* USE_XPD_TIM */
