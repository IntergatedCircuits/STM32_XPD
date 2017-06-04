/**
  ******************************************************************************
  * @file    xpd_tim.c
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

#include "xpd_tim.h"
#include "xpd_utils.h"

#ifdef USE_XPD_TIM

/** @addtogroup TIM
 * @{ */

/** @addtogroup TIM_Common
 * @{ */

#if TIM_SUPPORTED_CHANNEL_COUNT > 5
#define TIM_ALL_CHANNELS (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E |\
                          TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC5E | TIM_CCER_CC6E)
#else
#define TIM_ALL_CHANNELS (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E |\
                          TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE)
#endif

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
static const XPD_HandleCallbackType tim_dmaRedirects[] = {
        tim_dmaUpdateRedirect,
        tim_dmaChannelEventRedirect1,
        tim_dmaChannelEventRedirect2,
        tim_dmaChannelEventRedirect3,
        tim_dmaChannelEventRedirect4,
        tim_dmaCommutationCallbackRedirect,
        tim_dmaTriggerCallbackRedirect,
};

/** @defgroup TIM_Common_Exported_Functions TIM Common Exported Functions
 *  @brief    TIM common functions (timer, channels control)
 * @{ */

/**
 * @brief Initializes the TIM peripheral using the setup configuration.
 * @param htim: pointer to the TIM handle structure
 * @param Config: pointer to TIM setup configuration
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_TIM_Init(TIM_HandleType * htim, const TIM_Counter_InitType * Config)
{
    /* enable clock */
    XPD_SAFE_CALLBACK(htim->ClockCtrl, ENABLE);

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(htim->Callbacks.DepInit,htim);

    TIM_REG_BIT(htim, CR1, DIR) = Config->Mode;
    htim->Inst->CR1.b.CMS       = Config->Mode >> 1;
    htim->Inst->CR1.b.CKD       = Config->ClockDivision;

    htim->Inst->ARR             = (uint32_t)(Config->Period - 1);
    htim->Inst->PSC             = (uint32_t)(Config->Prescaler - 1);

    htim->Inst->RCR             = Config->RepetitionCounter;

    /* Generate an update event to reload the prescaler
     and the repetition counter (only for advanced TIM) value immediately */
    XPD_TIM_GenerateEvent(htim, U);
    XPD_TIM_ClearFlag(htim, U);

    return XPD_OK;
}

/**
 * @brief Deinitializes the TIM peripheral.
 * @param htim: pointer to the TIM handle structure
 * @return ERROR if input is incorrect, OK if success
 */
XPD_ReturnType XPD_TIM_Deinit(TIM_HandleType * htim)
{
    XPD_TIM_Output_Disable(htim);

    /* disable all channels */
    CLEAR_BIT(htim->Inst->CCER.w, TIM_ALL_CHANNELS);

    /* disable the TIM Peripheral Clock */
    XPD_TIM_Counter_Stop(htim);

    /* disable clock */
    XPD_SAFE_CALLBACK(htim->ClockCtrl, DISABLE);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(htim->Callbacks.DepDeinit,htim);

    return XPD_OK;
}

/**
 * @brief Enables the TIM counter and update interrupt.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_Counter_Start_IT(TIM_HandleType * htim)
{
    /* enable the TIM Update interrupt */
    XPD_TIM_EnableIT(htim, U);

    /* enable the counter */
    XPD_TIM_Counter_Start(htim);
}

/**
 * @brief Disables the TIM counter and update interrupt.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_Counter_Stop_IT(TIM_HandleType * htim)
{
    XPD_TIM_DisableIT(htim, U);

    /* disable the counter */
    XPD_TIM_Counter_Stop(htim);
}

/**
 * @brief Sets up and enables a DMA transfer for the TIM counter.
 * @param htim: pointer to the TIM handle structure
 * @param Address: memory address of the counter data
 * @param Length: the amount of data to be transferred
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType XPD_TIM_Counter_Start_DMA(TIM_HandleType * htim, void * Address, uint16_t Length)
{
    XPD_ReturnType result;

    /* Set up DMA for transfer */
    result = XPD_DMA_Start_IT(htim->DMA.Update, (void*)&htim->Inst->ARR, Address, Length);

    /* If the DMA is currently used, return with error */
    if (result == XPD_OK)
    {
        /* Set the callback owner */
        htim->DMA.Update->Owner = htim;

        /* Set the DMA transfer callbacks */
        htim->DMA.Update->Callbacks.Complete = tim_dmaUpdateRedirect;
#ifdef USE_XPD_DMA_ERROR_DETECT
        htim->DMA.Update->Callbacks.Error    = tim_dmaErrorRedirect;
#endif

        /* enable the TIM Update DMA request */
        TIM_REG_BIT(htim, DIER, UDE) = 1;

        /* enable the counter */
        XPD_TIM_Counter_Start(htim);
    }

    return result;
}

/**
 * @brief Disables the TIM counter and DMA transfer.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_Counter_Stop_DMA(TIM_HandleType * htim)
{
    /* disable the update DMA request */
    TIM_REG_BIT(htim, DIER, UDE) = 0;

    XPD_DMA_Stop_IT(htim->DMA.Update);

    /* disable the counter */
    XPD_TIM_Counter_Stop(htim);
}

/**
 * @brief Starts the selected timer channel (and the timer if required).
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected capture/compare channel to use
 */
void XPD_TIM_Channel_Start(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    uint32_t ccer = htim->Inst->CCER.w;

    htim->Inst->CCER.w = ccer | ((TIM_CCER_CC1E | TIM_CCER_CC1NE) << (4 * Channel));

    if ((ccer & TIM_ALL_CHANNELS) == 0)
    {
        XPD_TIM_Counter_Start(htim);
    }
}

/**
 * @brief Stops the selected timer channel (and the timer if required).
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected capture/compare channel to use
 */
void XPD_TIM_Channel_Stop(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    uint32_t ccer = htim->Inst->CCER.w & (~((TIM_CCER_CC1E | TIM_CCER_CC1NE) << (4 * Channel)));

    htim->Inst->CCER.w = ccer;

    if ((ccer & TIM_ALL_CHANNELS) == 0)
    {
        XPD_TIM_Counter_Stop(htim);
    }
}

/**
 * @brief Starts the selected timer channel (and the timer if required)
 *        and enables the channel event interrupt.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected capture/compare channel to use
 */
void XPD_TIM_Channel_Start_IT(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    SET_BIT(htim->Inst->DIER.w, (TIM_DIER_CC1IE << (uint32_t)Channel) | TIM_DIER_BIE);

    XPD_TIM_Channel_Start(htim, Channel);
}

/**
 * @brief Stops the selected timer channel (and the timer if required)
 *        and disables the channel event interrupt.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected capture/compare channel to use
 */
void XPD_TIM_Channel_Stop_IT(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    CLEAR_BIT(htim->Inst->DIER.w, (TIM_DIER_CC1IE << (uint32_t)Channel) | TIM_DIER_BIE);

    XPD_TIM_Channel_Stop(htim, Channel);
}

/**
 * @brief Starts the selected timer channel (and the timer if required)
 *        using a DMA transfer to provide channel pulse values.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected capture/compare channel to use
 * @param Address: the memory start address of the pulse values
 * @param Length: the memory size of the pulse values
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType XPD_TIM_Channel_Start_DMA(TIM_HandleType * htim, TIM_ChannelType Channel,
        void * Address, uint16_t Length)
{
    XPD_ReturnType result;

    /* Set up DMA for transfer */
    result = XPD_DMA_Start_IT(htim->DMA.Channel[Channel], (void*)&((&htim->Inst->CCR1)[Channel]),
            Address, Length);

    /* If the DMA is currently used, return with error */
    if (result == XPD_OK)
    {
        /* Set the callback owner */
        htim->DMA.Channel[Channel]->Owner = htim;

        /* set the DMA complete callback */
        htim->DMA.Channel[Channel]->Callbacks.Complete = tim_dmaRedirects[Channel + 1];
#ifdef USE_XPD_DMA_ERROR_DETECT
        htim->DMA.Channel[Channel]->Callbacks.Error    = tim_dmaErrorRedirect;
#endif

        XPD_TIM_Channel_EnableDMA(htim, Channel);

        XPD_TIM_Channel_Start(htim, Channel);
    }
    return result;
}

/**
 * @brief Stops the selected timer channel (and the timer if required)
 *        and disables the DMA requests.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected capture/compare channel to use
 */
void XPD_TIM_Channel_Stop_DMA(TIM_HandleType * htim, TIM_ChannelType Channel)
{
    XPD_TIM_Channel_DisableDMA(htim, Channel);

    XPD_DMA_Stop_IT(htim->DMA.Channel[Channel]);

    XPD_TIM_Channel_Stop(htim, Channel);
}

/**
 * @brief TIM update interrupt handler that provides handle callbacks.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_UP_IRQHandler(TIM_HandleType * htim)
{
    /* TIM update event */
    if (XPD_TIM_GetFlag(htim, U) != 0)
    {
        /* Clear interrupt flag */
        XPD_TIM_ClearFlag(htim, U);

        XPD_SAFE_CALLBACK(htim->Callbacks.Update, htim);
    }
}

/**
 * @brief TIM channel event interrupt handler that provides handle callbacks.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_CC_IRQHandler(TIM_HandleType * htim)
{
    uint32_t sr = (htim->Inst->SR.w & (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF)) >> 1;
    TIM_ChannelType ch;

    /* Iterate through active channel flags */
    for (ch = TIM_CHANNEL_1; sr > 0; ch++, sr >>= 1)
    {
        if ((sr & 1) != 0)
        {
            /* Clear interrupt flag */
            XPD_TIM_Channel_ClearFlag(htim, ch);

            /* Provide channel number for interrupt callback context */
            htim->ActiveChannel = ch;
            XPD_SAFE_CALLBACK(htim->Callbacks.ChannelEvent, htim);
        }
    }
}

/**
 * @brief TIM break event interrupt handler that provides handle callbacks.
 * @param htim: pointer to the TIM handle structure
 */
void XPD_TIM_BRK_IRQHandler(TIM_HandleType * htim)
{
    /* TIM Break input event */
    if (XPD_TIM_GetFlag(htim, B) != 0)
    {
        /* Clear interrupt flag */
        XPD_TIM_ClearFlag(htim, B);

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
        XPD_TIM_ClearFlag(htim, T);

        XPD_SAFE_CALLBACK(htim->Callbacks.Trigger, htim);
    }
    /* TIM commutation event */
    if ((sr & TIM_SR_COMIF) != 0)
    {
        /* clear interrupt flag */
        XPD_TIM_ClearFlag(htim, COM);

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

/** @addtogroup TIM_Burst
 * @{ */

/** @defgroup TIM_Burst_Exported_Functions TIM Burst DMA Exported Functions
 * @{ */

/**
 * @brief Sets up burst DMA transfer for the timer and enables the selected DMA requests.
 * @param htim: pointer to the TIM handle structure
 * @param Config: pointer to the burst transfer configuration
 * @param Address: the memory start address of the pulse values
 * @param Length: the memory size of the pulse values
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType XPD_TIM_Burst_Start_DMA(TIM_HandleType * htim, const TIM_Burst_InitType * Config,
        void * Address, uint16_t Length)
{
    XPD_ReturnType result;
    DMA_HandleType * hdma = htim->DMA.Burst;

    /* Set the DMA start address location - relative to timer start address */
    htim->Inst->DCR.b.DBA = Config->RegIndex;
    /* Set the DMA burst transfer length */
    htim->Inst->DCR.b.DBL = Length;

    /* Set up DMA for transfer */
    result = XPD_DMA_Start_IT(hdma, (void*)&htim->Inst->DMAR, Address, Length);

    /* If the DMA is currently used, return with error */
    if (result == XPD_OK)
    {
        /* Set the callback owner */
        hdma->Owner = htim;

        /* set the DMA complete callback */
        hdma->Callbacks.Complete     = tim_dmaRedirects[Config->Source];
#ifdef USE_XPD_DMA_ERROR_DETECT
        hdma->Callbacks.Error        = tim_dmaErrorRedirect;
#endif

        /* Enable the selected DMA request */
        SET_BIT(htim->Inst->DIER.w, TIM_DIER_UDE << Config->Source);
    }
    return result;
}

/**
 * @brief Stops the selected (burst) DMA request.
 * @param htim: pointer to the TIM handle structure
 * @param Source: pointer to the burst transfer configuration
 */
void XPD_TIM_Burst_Stop_DMA(TIM_HandleType * htim, TIM_BurstSourceType Source)
{
    DMA_HandleType * hdma = htim->DMA.Burst;

    /* Disable the selected DMA request */
    CLEAR_BIT(htim->Inst->DIER.w, TIM_DIER_UDE << Source);

    XPD_DMA_Stop_IT(hdma);
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
void XPD_TIM_Output_ChannelConfig(TIM_HandleType * htim, TIM_ChannelType Channel, const TIM_Output_InitType * Config)
{
    /* set the output compare polarities (0 is active high) */
    {
        uint32_t pol = 0;

        if (Config->Polarity == ACTIVE_LOW)
        {
            pol  = TIM_CCER_CC1P;
        }
        if (Config->CompPolarity == ACTIVE_LOW)
        {
            pol |= TIM_CCER_CC1NP;
        }

        MODIFY_REG(htim->Inst->CCER.w, 0xF << (Channel * 4), pol << (Channel * 4));
    }
    /* output mode configuration */
    {
        __IO uint32_t * pccmr;
        uint32_t chcfg, choffset = (Channel & 1) * 8;

        /* get related CCMR register */
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
        if (Channel < TIM_CHANNEL_5)
        {
            pccmr = &htim->Inst->CCMR3.w;
        }
        else
#endif
        if (Channel < TIM_CHANNEL_3)
        {
            pccmr = &htim->Inst->CCMR1.w;
        }
        else
        {
            pccmr = &htim->Inst->CCMR2.w;
        }

        /* add output configuration */
        chcfg = (Config->Mode << TIM_CCMR1_OC1M_Pos) & TIM_CCMR1_OC1M;

        /* for PWM modes, enable preload and fast mode */
        if (Config->Mode >= TIM_OUTPUT_PWM1)
        {
            chcfg |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE;
        }

        /* apply configuration */
        *pccmr = (*pccmr
                 &((TIM_CCMR1_CC2S | TIM_CCMR1_OC2CE | TIM_CCMR1_OC2FE |
                    TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE) >> choffset))
                | (chcfg << choffset);
    }

    /* Set the Output Idle states */
    {
        uint32_t idle;

        idle = (Config->IdleState & 1) | ((Config->CompIdleState & 1) << 1);
        idle = idle << (Channel * 2 + 8);

        MODIFY_REG(htim->Inst->CR2.w,(TIM_CR2_OIS1 | TIM_CR2_OIS1N) << (Channel * 2), idle);
    }
}

/**
 * @brief Sets up a selected Break feature using the input configuration.
 * @param htim: pointer to the TIM handle structure
 * @param BreakNumber: the Break line to configure (1 or 2)
 * @param Config: pointer to TIM Break setup configuration
 */
void XPD_TIM_Output_BreakConfig(TIM_HandleType * htim, uint8_t BreakLine, const TIM_Output_BreakInitType * Config)
{
#ifdef TIM_BDTR_BK2E
    if (BreakLine > 1)
    {
        /* break2 configuration */
        TIM_REG_BIT(htim, BDTR, BK2E) = Config->State;
        TIM_REG_BIT(htim, BDTR, BK2P) = 1 - Config->Polarity;
        htim->Inst->BDTR.b.BK2F       = Config->Filter;
    }
    else
#endif
    {
        /* break configuration */
        TIM_REG_BIT(htim, BDTR, BKE)  = Config->State;
        TIM_REG_BIT(htim, BDTR, BKP)  = 1 - Config->Polarity;
#ifdef TIM_BDTR_BKF
        htim->Inst->BDTR.b.BKF        = Config->Filter;
#endif
    }
}

/**
 * @brief Sets up the output drive configuration of the timer.
 * @param htim: pointer to the TIM handle structure
 * @param Config: pointer to TIM break setup configuration
 */
void XPD_TIM_Output_DriveConfig(TIM_HandleType * htim, const TIM_Output_DriveType * Config)
{
    uint8_t deadtime;

    /* Revert the configuration characteristic */
    if (Config->DeadCounts < 128)
    {
        /* dead counts = DTG */
        deadtime = Config->DeadCounts;
    }
    else if (Config->DeadCounts < 256)
    {
        /* dead counts =(64+DTG[5:0])*2 */
        deadtime = 0x80 | ((Config->DeadCounts / 2) - 64);
    }
    else if (Config->DeadCounts < 512)
    {
        /* dead counts =(32+DTG[4:0])*8 */
        deadtime = 0xC0 | ((Config->DeadCounts / 8) - 32);
    }
    else if (Config->DeadCounts < 1008)
    {
        /* dead counts =(32+DTG[4:0])*16 */
        deadtime = 0xE0 | ((Config->DeadCounts / 16) - 32);
    }
    else
    {
        /* saturation */
        deadtime = 255;
    }

    htim->Inst->BDTR.b.DTG        = deadtime;
    TIM_REG_BIT(htim, BDTR, AOE)  = Config->AutomaticOutput;
    TIM_REG_BIT(htim, BDTR, OSSI) = Config->IdleOffState;
    TIM_REG_BIT(htim, BDTR, OSSR) = Config->RunOffState;
}

/**
 * @brief Sets up the OCxREF clearing feature for a selected channel
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to configure
 * @param OCREF_CLR_IN: OCREFClear input signal selection
 */
void XPD_TIM_Output_OCRefClearConfig(TIM_HandleType * htim, TIM_ChannelType Channel,
        TIM_OCRefClearSourceType OCREF_CLR_IN)
{
    __IO uint32_t * pccmr;
    uint32_t choffset = (Channel & 1) * 8;

    /* get related CCMR register */
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
    if (Channel < TIM_CHANNEL_5)
    {
        pccmr = &htim->Inst->CCMR3.w;
    }
    else
#endif
    if (Channel < TIM_CHANNEL_3)
    {
        pccmr = &htim->Inst->CCMR1.w;
    }
    else
    {
        pccmr = &htim->Inst->CCMR2.w;
    }

    if (OCREF_CLR_IN == TIM_OCREFCLEAR_SOURCE_NONE)
    {
        /* clear configuration from channel */
        CLEAR_BIT(*pccmr, TIM_CCMR1_OC1CE << choffset);
    }
    else
    {
        /* apply configuration */
        SET_BIT(*pccmr, TIM_CCMR1_OC1CE << choffset);

#ifdef TIM_SMCR_OCCS
        /* select ocref_clr_int source */
        TIM_REG_BIT(htim,SMCR,OCCS) = OCREF_CLR_IN;
#endif
    }
}

/**
 * @brief Configures the commutation source
 * @param htim: pointer to the TIM handle structure
 * @param ComSource
 */
void XPD_TIM_Output_CommutationConfig(TIM_HandleType * htim, TIM_CommutationSourceType ComSource)
{
    if (ComSource > TIM_COMSOURCE_SOFTWARE)
    {
        /* Set TRGI if used for commutation */
        MODIFY_REG(htim->Inst->SMCR.w, TIM_SMCR_TS, ComSource);

        /* Select the Commutation event source */
        SET_BIT(htim->Inst->CR2.w, TIM_CR2_CCPC | TIM_CR2_CCUS);
    }
    else
    {
        /* Select the Commutation event source */
        CLEAR_BIT(htim->Inst->CR2.w, TIM_CR2_CCPC | TIM_CR2_CCUS);

        if (ComSource == TIM_COMSOURCE_SOFTWARE)
        {
            /* Select the Capture Compare preload feature */
            TIM_REG_BIT(htim,CR2,CCPC) = 1;
        }
    }
}

/** @} */

/** @} */

/** @addtogroup TIM_Input
 * @{ */

/** @defgroup TIM_Input_Exported_Functions TIM Input Exported Functions
 *  @brief    TIM input mode functions for capture channels
 *  @details  These functions provide API for TIM input channel modes (Input Capture).
 * @{ */

/**
 * @brief Initializes a compare channel using the setup configuration.
 * @param htim: pointer to the TIM handle structure
 * @param Channel: the selected compare channel to use
 * @param Config: pointer to TIM compare channel setup configuration
 */
void XPD_TIM_Input_ChannelConfig(TIM_HandleType * htim, TIM_ChannelType Channel, const TIM_Input_InitType * Config)
{
    /* Set the input polarity, disable channel */
    {
        uint32_t pol = 0;

        if (Config->Polarity == ACTIVE_LOW)
        {
            pol = TIM_CCER_CC1P;
        }

        MODIFY_REG(htim->Inst->CCER.w, 0xF << (Channel * 4), pol << (Channel * 4));
    }

    /* Input mode configuration */
    {
        __IO uint32_t * pccmr;
        uint32_t chcfg, choffset = (Channel & 1) * 8;

        /* get related CCMR register */
        if (Channel < TIM_CHANNEL_3)
        {
            pccmr = &htim->Inst->CCMR1.w;
        }
        else
        {
            pccmr = &htim->Inst->CCMR2.w;
        }

        /* Assemble input configuration */
        chcfg =   ((Config->Filter << TIM_CCMR1_IC1F_Pos) & TIM_CCMR1_IC1F)
                | ((Config->Prescaler << TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC)
                |  (Config->Source & TIM_CCMR1_CC1S);

        /* apply configuration */
        *pccmr = (*pccmr
                 &((TIM_CCMR1_CC2S | TIM_CCMR1_OC2CE | TIM_CCMR1_OC2FE |
                    TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE) >> choffset))
                | (chcfg << choffset);
    }
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
void XPD_TIM_MasterConfig(TIM_HandleType * htim, const TIM_MasterConfigType * Config)
{
    /* select the TRGO source */
    htim->Inst->CR2.b.MMS = Config->MasterTrigger;

#ifdef TIM_CR2_MMS2
    /* select the TRGO2 source */
    htim->Inst->CR2.b.MMS2 = Config->MasterTrigger2;
#endif

    /* configure the MSM bit - delay TRGI to synchronize with TRGO */
    TIM_REG_BIT(htim,SMCR,MSM) = Config->MasterSlaveMode;
}

/**
 * @brief Sets up the slave configuration of the timer.
 * @param htim: pointer to the TIM handle structure
 * @param Config: pointer to TIM slave setup configuration
 */
void XPD_TIM_SlaveConfig(TIM_HandleType * htim, const TIM_SlaveConfigType * Config)
{
    /* Set the mandatory configuration elements */
    MODIFY_REG(htim->Inst->SMCR.w,
            TIM_SMCR_ECE | TIM_SMCR_ETPS | TIM_SMCR_TS | TIM_SMCR_SMS,
            Config->SlaveMode | Config->SlaveTrigger);

    /* Configure the trigger prescaler, filter, and polarity (where applicable) */
    switch (Config->SlaveTrigger)
    {
        case TIM_TRGI_TI1:
        case TIM_TRGI_TI1_ED:
        {
            /* Set the input polarity, disable channel */
            CLEAR_BIT(htim->Inst->CCER.w, 0xF);

            if (Config->Polarity == ACTIVE_LOW)
            {
                TIM_REG_BIT(htim,CCER,CC1P) = 1;
            }

            /* Set the filter */
            htim->Inst->CCMR1.IC.C1F = Config->Filter;
            break;
        }

        case TIM_TRGI_TI2:
        {
            /* Set the input polarity, disable channel */
            CLEAR_BIT(htim->Inst->CCER.w, 0xF0);

            if (Config->Polarity == ACTIVE_LOW)
            {
                TIM_REG_BIT(htim,CCER,CC2P) = 1;
            }

            /* Set the filter */
            htim->Inst->CCMR1.IC.C2F = Config->Filter;
            break;
        }

        case TIM_TRGI_ETR_MODE1:
        case TIM_TRGI_ETR_MODE2:
        {
            /* Configure the external trigger settings */
            htim->Inst->SMCR.b.ETF  = Config->Filter;
            htim->Inst->SMCR.b.ETPS = Config->Prescaler;
            htim->Inst->SMCR.b.ETP  = Config->Polarity;
            break;
        }

        default:
            break;
    }
}

/** @} */

/** @} */

/** @} */

#endif /* USE_XPD_TIM */
