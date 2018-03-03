/**
  ******************************************************************************
  * @file    xpd_tim.c
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

#include <xpd_tim.h>
#include <xpd_utils.h>

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

#ifdef __XPD_DMA_ERROR_DETECT
static void TIM_prvDmaErrorRedirect(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    XPD_SAFE_CALLBACK(pxTIM->Callbacks.Error, pxTIM);
}
#endif
static void TIM_prvDmaUpdateRedirect(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    XPD_SAFE_CALLBACK(pxTIM->Callbacks.Update, pxTIM);
}
static void TIM_prvDmaChannelEventRedirect1(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    pxTIM->ActiveChannel = TIM_CH1;
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.ChannelEvent, pxTIM);
}
static void TIM_prvDmaChannelEventRedirect2(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    pxTIM->ActiveChannel = TIM_CH2;
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.ChannelEvent, pxTIM);
}
static void TIM_prvDmaChannelEventRedirect3(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    pxTIM->ActiveChannel = TIM_CH3;
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.ChannelEvent, pxTIM);
}
static void TIM_prvDmaChannelEventRedirect4(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    pxTIM->ActiveChannel = TIM_CH4;
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.ChannelEvent,pxTIM);
}
static void TIM_prvDmaTriggerCallbackRedirect(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    XPD_SAFE_CALLBACK(pxTIM->Callbacks.Trigger,pxTIM);
}
static void TIM_prvDmaCommutationCallbackRedirect(void *pxDMA)
{
    TIM_HandleType* pxTIM = (TIM_HandleType*) ((DMA_HandleType*) pxDMA)->Owner;

    XPD_SAFE_CALLBACK(pxTIM->Callbacks.Commutation,pxTIM);
}
static const XPD_HandleCallbackType TIM_apxDmaRedirects[] = {
        TIM_prvDmaUpdateRedirect,
        TIM_prvDmaChannelEventRedirect1,
        TIM_prvDmaChannelEventRedirect2,
        TIM_prvDmaChannelEventRedirect3,
        TIM_prvDmaChannelEventRedirect4,
        TIM_prvDmaCommutationCallbackRedirect,
        TIM_prvDmaTriggerCallbackRedirect,
};

/** @defgroup TIM_Common_Exported_Functions TIM Common Exported Functions
 *  @brief    TIM common functions (timer, channels control)
 * @{ */

/**
 * @brief Initializes the TIM peripheral using the setup configuration.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM setup configuration
 */
void TIM_vInit(TIM_HandleType * pxTIM, const TIM_InitType * pxConfig)
{
    /* enable clock, reset peripheral */
    RCC_vClockEnable(pxTIM->CtrlPos);
    RCC_vReset(pxTIM->CtrlPos);

    TIM_REG_BIT(pxTIM, CR1, DIR) = pxConfig->Mode;
    pxTIM->Inst->CR1.b.CMS       = pxConfig->Mode >> 1;
    pxTIM->Inst->CR1.b.CKD       = pxConfig->ClockDivision;

    pxTIM->Inst->ARR             = (uint32_t)(pxConfig->Period - 1);
    pxTIM->Inst->PSC             = (uint32_t)(pxConfig->Prescaler - 1);

    pxTIM->Inst->RCR             = pxConfig->RepetitionCounter;

    /* Generate an update event to reload the prescaler
     and the repetition counter (only for advanced TIM) value immediately */
    TIM_EVENT_SET(pxTIM, U);
    TIM_FLAG_CLEAR(pxTIM, U);

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.DepInit,pxTIM);
}

/**
 * @brief Deinitializes the TIM peripheral.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vDeinit(TIM_HandleType * pxTIM)
{
    TIM_vOutputDisable(pxTIM);

    /* disable all channels */
    CLEAR_BIT(pxTIM->Inst->CCER.w, TIM_ALL_CHANNELS);

    /* stop the timer */
    TIM_vCounterStop(pxTIM);

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.DepDeinit,pxTIM);

    /* disable clock */
    RCC_vClockDisable(pxTIM->CtrlPos);
}

/**
 * @brief TIM common interrupt handler that provides handle callbacks (break handling excluded).
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler(TIM_HandleType * pxTIM)
{
    TIM_vIRQHandler_UP(pxTIM);
    TIM_vIRQHandler_CC(pxTIM);
    TIM_vIRQHandler_TRG(pxTIM);
    TIM_vIRQHandler_COM(pxTIM);
    /* leave out break interrupt, timers that have break also have separate interrupt line */
    /*TIM_vIRQHandler_BRK(pxTIM);*/
}

/**
 * @brief Enables the TIM counter and update interrupt.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vCounterStart_IT(TIM_HandleType * pxTIM)
{
    /* enable the TIM Update interrupt */
    TIM_IT_ENABLE(pxTIM, U);

    /* enable the counter */
    TIM_vCounterStart(pxTIM);
}

/**
 * @brief Disables the TIM counter and update interrupt.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vCounterStop_IT(TIM_HandleType * pxTIM)
{
    TIM_IT_DISABLE(pxTIM, U);

    /* disable the counter */
    TIM_vCounterStop(pxTIM);
}

/**
 * @brief TIM update interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_UP(TIM_HandleType * pxTIM)
{
    /* TIM update event */
    if (TIM_FLAG_STATUS(pxTIM, U) != 0)
    {
        /* Clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, U);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Update, pxTIM);
    }
}

/**
 * @brief Sets up and enables a DMA transfer for the TIM counter.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pvAddress: memory address of the counter data
 * @param usLength: the amount of data to be transferred
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType TIM_eCounterStart_DMA(TIM_HandleType * pxTIM, void * pvAddress, uint16_t usLength)
{
    XPD_ReturnType eResult;

    /* Set up DMA for transfer */
    eResult = DMA_eStart_IT(pxTIM->DMA.Update,
            (void*)&pxTIM->Inst->ARR, pvAddress, usLength);

    /* If the DMA is currently used, return with error */
    if (eResult == XPD_OK)
    {
        /* Set the callback owner */
        pxTIM->DMA.Update->Owner = pxTIM;

        /* Set the DMA transfer callbacks */
        pxTIM->DMA.Update->Callbacks.Complete = TIM_prvDmaUpdateRedirect;
#ifdef __XPD_DMA_ERROR_DETECT
        pxTIM->DMA.Update->Callbacks.Error    = TIM_prvDmaErrorRedirect;
#endif

        /* enable the TIM Update DMA request */
        TIM_REG_BIT(pxTIM, DIER, UDE) = 1;

        /* enable the counter */
        TIM_vCounterStart(pxTIM);
    }

    return eResult;
}

/**
 * @brief Disables the TIM counter and DMA transfer.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vCounterStop_DMA(TIM_HandleType * pxTIM)
{
    /* disable the update DMA request */
    TIM_REG_BIT(pxTIM, DIER, UDE) = 0;

    DMA_vStop_IT(pxTIM->DMA.Update);

    /* disable the counter */
    TIM_vCounterStop(pxTIM);
}

/**
 * @brief Starts the selected timer channel (and the timer if required).
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected capture/compare channel to use
 */
void TIM_vChannelStart(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    uint32_t ulCCER = pxTIM->Inst->CCER.w;

    pxTIM->Inst->CCER.w = ulCCER | ((TIM_CCER_CC1E | TIM_CCER_CC1NE) << (4 * eChannel));

    if ((ulCCER & TIM_ALL_CHANNELS) == 0)
    {
        TIM_vCounterStart(pxTIM);
    }
}

/**
 * @brief Stops the selected timer channel (and the timer if required).
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected capture/compare channel to use
 */
void TIM_vChannelStop(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    uint32_t ulCCER = pxTIM->Inst->CCER.w & (~((TIM_CCER_CC1E | TIM_CCER_CC1NE) << (4 * eChannel)));

    pxTIM->Inst->CCER.w = ulCCER;

    if ((ulCCER & TIM_ALL_CHANNELS) == 0)
    {
        TIM_vCounterStop(pxTIM);
    }
}

/**
 * @brief Starts the selected timer channel (and the timer if required)
 *        and enables the channel event interrupt.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected capture/compare channel to use
 */
void TIM_vChannelStart_IT(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    SET_BIT(pxTIM->Inst->DIER.w, (TIM_DIER_CC1IE << (uint32_t)eChannel) | TIM_DIER_BIE);

    TIM_vChannelStart(pxTIM, eChannel);
}

/**
 * @brief Stops the selected timer channel (and the timer if required)
 *        and disables the channel event interrupt.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected capture/compare channel to use
 */
void TIM_vChannelStop_IT(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    CLEAR_BIT(pxTIM->Inst->DIER.w, (TIM_DIER_CC1IE << (uint32_t)eChannel) | TIM_DIER_BIE);

    TIM_vChannelStop(pxTIM, eChannel);
}

/**
 * @brief TIM channel event interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_CC(TIM_HandleType * pxTIM)
{
    TIM_ChannelType eChannel = TIM_CH1;
    uint32_t ulFlags = (pxTIM->Inst->SR.w
            & (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF)
            ) >> TIM_SR_CC1IF_Pos;

    /* Iterate through active channel flags */
    while (ulFlags > 0)
    {
        if ((ulFlags & 1) != 0)
        {
            /* Clear interrupt flag */
            TIM_CH_FLAG_CLEAR(pxTIM, eChannel);

            /* Provide channel number for interrupt callback context */
            pxTIM->ActiveChannel = eChannel;
            XPD_SAFE_CALLBACK(pxTIM->Callbacks.ChannelEvent, pxTIM);
        }
        eChannel++;
        ulFlags >>= 1;
    }
}

/**
 * @brief Starts the selected timer channel (and the timer if required)
 *        using a DMA transfer to provide channel pulse values.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected capture/compare channel to use
 * @param pvAddress: the memory start address of the pulse values
 * @param usLength: the memory size of the pulse values
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType TIM_eChannelStart_DMA(
        TIM_HandleType *    pxTIM,
        TIM_ChannelType     eChannel,
        void *              pvAddress,
        uint16_t            usLength)
{
    XPD_ReturnType eResult;

    /* Set up DMA for transfer */
    eResult = DMA_eStart_IT(pxTIM->DMA.Channel[eChannel],
            (void*)&((&pxTIM->Inst->CCR1)[eChannel]), pvAddress, usLength);

    /* If the DMA is currently used, return with error */
    if (eResult == XPD_OK)
    {
        /* Set the callback owner */
        pxTIM->DMA.Channel[eChannel]->Owner = pxTIM;

        /* set the DMA complete callback */
        pxTIM->DMA.Channel[eChannel]->Callbacks.Complete = TIM_apxDmaRedirects[1 + eChannel];
#ifdef __XPD_DMA_ERROR_DETECT
        pxTIM->DMA.Channel[eChannel]->Callbacks.Error    = TIM_prvDmaErrorRedirect;
#endif

        TIM_CH_DMA_ENABLE(pxTIM, eChannel);

        TIM_vChannelStart(pxTIM, eChannel);
    }
    return eResult;
}

/**
 * @brief Stops the selected timer channel (and the timer if required)
 *        and disables the DMA requests.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected capture/compare channel to use
 */
void TIM_vChannelStop_DMA(TIM_HandleType * pxTIM, TIM_ChannelType eChannel)
{
    TIM_CH_DMA_DISABLE(pxTIM, eChannel);

    DMA_vStop_IT(pxTIM->DMA.Channel[eChannel]);

    TIM_vChannelStop(pxTIM, eChannel);
}

/** @} */

/** @} */

/** @addtogroup TIM_Burst
 * @{ */

/** @defgroup TIM_Burst_Exported_Functions TIM Burst DMA Exported Functions
 * @{ */

/**
 * @brief Sets up burst DMA transfer for the timer and enables the selected DMA requests.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to the burst transfer configuration
 * @param pvAddress: the memory start address of the pulse values
 * @param usLength: the memory size of the pulse values
 * @return BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType TIM_eBurstStart_DMA(
        TIM_HandleType *            pxTIM,
        const TIM_BurstInitType *   pxConfig,
        void *                      pvAddress,
        uint16_t                    usLength)
{
    XPD_ReturnType eResult;
    DMA_HandleType * pxDMA = pxTIM->DMA.Burst;

    /* Set the DMA start address location - relative to timer start address */
    pxTIM->Inst->DCR.b.DBA = pxConfig->RegIndex;
    /* Set the DMA burst transfer length */
    pxTIM->Inst->DCR.b.DBL = usLength - 1;

    /* Set up DMA for transfer */
    eResult = DMA_eStart_IT(pxDMA,
            (void*)&pxTIM->Inst->DMAR, pvAddress, usLength);

    /* If the DMA is currently used, return with error */
    if (eResult == XPD_OK)
    {
        /* Set the callback owner */
        pxDMA->Owner = pxTIM;

        /* set the DMA complete callback */
        pxDMA->Callbacks.Complete     = TIM_apxDmaRedirects[pxConfig->Source];
#ifdef __XPD_DMA_ERROR_DETECT
        pxDMA->Callbacks.Error        = TIM_prvDmaErrorRedirect;
#endif

        /* Enable the selected DMA request */
        SET_BIT(pxTIM->Inst->DIER.w, TIM_DIER_UDE << pxConfig->Source);
    }
    return eResult;
}

/**
 * @brief Stops the selected (burst) DMA request.
 * @param pxTIM: pointer to the TIM handle structure
 * @param Source: pointer to the burst transfer configuration
 */
void TIM_vBurstStop_DMA(TIM_HandleType * pxTIM, TIM_EventType Source)
{
    DMA_HandleType * pxDMA = pxTIM->DMA.Burst;

    /* Disable the selected DMA request */
    CLEAR_BIT(pxTIM->Inst->DIER.w, TIM_DIER_UDE << Source);

    DMA_vStop_IT(pxDMA);
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
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected compare channel to use
 * @param pxConfig: pointer to TIM compare channel setup configuration
 */
void TIM_vOutputChannelConfig(
        TIM_HandleType *            pxTIM,
        TIM_ChannelType             eChannel,
        const TIM_OutputInitType *  pxConfig)
{
    /* set the output compare polarities (0 is active high) */
    {
        uint32_t ulPol = 0;

        if (pxConfig->Polarity == ACTIVE_LOW)
        {
            ulPol  = TIM_CCER_CC1P;
        }
        if (pxConfig->CompPolarity == ACTIVE_LOW)
        {
            ulPol |= TIM_CCER_CC1NP;
        }

        MODIFY_REG(pxTIM->Inst->CCER.w, 0xF << (eChannel * 4), ulPol << (eChannel * 4));
    }
    /* output mode configuration */
    {
        __IO uint32_t * pulCCMR;
        uint32_t ulChCfg, ulChOffset = (eChannel & 1) * 8;

        /* get related CCMR register */
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
        if (eChannel > TIM_CH4)
        {
            pulCCMR = &pxTIM->Inst->CCMR3.w;
        }
        else
#endif
        if (eChannel < TIM_CH3)
        {
            pulCCMR = &pxTIM->Inst->CCMR1.w;
        }
        else
        {
            pulCCMR = &pxTIM->Inst->CCMR2.w;
        }

        /* add output configuration */
        ulChCfg = (pxConfig->Mode << TIM_CCMR1_OC1M_Pos) & TIM_CCMR1_OC1M;

        /* for PWM modes, enable preload and fast mode */
        if (pxConfig->Mode >= TIM_OUTPUT_PWM1)
        {
            ulChCfg |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE;
        }

        /* apply configuration */
        *pulCCMR = (*pulCCMR
                 &((TIM_CCMR1_CC2S | TIM_CCMR1_OC2CE | TIM_CCMR1_OC2FE |
                    TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE) >> ulChOffset))
                | (ulChCfg << ulChOffset);
    }

    /* Set the Output Idle states */
    {
        uint32_t ulIdle;

        ulIdle = (pxConfig->IdleState & 1) | ((pxConfig->CompIdleState & 1) << 1);
        ulIdle = ulIdle << (eChannel * 2 + 8);

        MODIFY_REG(pxTIM->Inst->CR2.w,(TIM_CR2_OIS1 | TIM_CR2_OIS1N) << (eChannel * 2), ulIdle);
    }
}

/**
 * @brief Sets up a selected Break feature using the input configuration.
 * @param pxTIM: pointer to the TIM handle structure
 * @param ucBreakLine: the Break line to configure (1 or 2)
 * @param pxConfig: pointer to TIM Break setup configuration
 */
void TIM_vBreakConfig(
        TIM_HandleType *            pxTIM,
        uint8_t                     ucBreakLine,
        const TIM_BreakInitType *   pxConfig)
{
#ifdef TIM_BDTR_BK2E
    if (ucBreakLine > 1)
    {
        /* break2 configuration */
        TIM_REG_BIT(pxTIM, BDTR, BK2E) = pxConfig->State;
        TIM_REG_BIT(pxTIM, BDTR, BK2P) = 1 - pxConfig->Polarity;
        pxTIM->Inst->BDTR.b.BK2F       = pxConfig->Filter;
    }
    else
#endif
    {
        /* break configuration */
        TIM_REG_BIT(pxTIM, BDTR, BKE)  = pxConfig->State;
        TIM_REG_BIT(pxTIM, BDTR, BKP)  = 1 - pxConfig->Polarity;
#ifdef TIM_BDTR_BKF
        pxTIM->Inst->BDTR.b.BKF        = pxConfig->Filter;
#endif
    }
}

/**
 * @brief TIM break event interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_BRK(TIM_HandleType * pxTIM)
{
    /* TIM Break input event */
    if (TIM_FLAG_STATUS(pxTIM, B) != 0)
    {
        /* Clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, B);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Break, pxTIM);
    }
}

/**
 * @brief Sets up the output drive configuration of the timer.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM break setup configuration
 */
void TIM_vDriveConfig(TIM_HandleType * pxTIM, const TIM_DriveInitType * pxConfig)
{
    uint8_t ucDeadtime;

    /* Revert the configuration characteristic */
    if (pxConfig->DeadCounts < 128)
    {
        /* dead counts = DTG */
        ucDeadtime = pxConfig->DeadCounts;
    }
    else if (pxConfig->DeadCounts < 256)
    {
        /* dead counts =(64+DTG[5:0])*2 */
        ucDeadtime = 0x80 | ((pxConfig->DeadCounts / 2) - 64);
    }
    else if (pxConfig->DeadCounts < 512)
    {
        /* dead counts =(32+DTG[4:0])*8 */
        ucDeadtime = 0xC0 | ((pxConfig->DeadCounts / 8) - 32);
    }
    else if (pxConfig->DeadCounts < 1008)
    {
        /* dead counts =(32+DTG[4:0])*16 */
        ucDeadtime = 0xE0 | ((pxConfig->DeadCounts / 16) - 32);
    }
    else
    {
        /* saturation */
        ucDeadtime = 255;
    }

    pxTIM->Inst->BDTR.b.DTG        = ucDeadtime;
    TIM_REG_BIT(pxTIM, BDTR, AOE)  = pxConfig->AutomaticOutput;
    TIM_REG_BIT(pxTIM, BDTR, OSSI) = pxConfig->IdleOffState;
    TIM_REG_BIT(pxTIM, BDTR, OSSR) = pxConfig->RunOffState;
}

/**
 * @brief Sets up the OCxREF clearing feature for a selected channel
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected compare channel to configure
 * @param eOCREF_CLR_IN: OCREFClear input signal selection
 */
void TIM_vOCRefClearConfig(
        TIM_HandleType *            pxTIM,
        TIM_ChannelType             eChannel,
        TIM_OCRefClearSourceType    eOCREF_CLR_IN)
{
    __IO uint32_t * pulCCMR;
    uint32_t ulChOffset = (eChannel & 1) * 8;

    /* get related CCMR register */
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
    if (eChannel > TIM_CH4)
    {
        pulCCMR = &pxTIM->Inst->CCMR3.w;
    }
    else
#endif
    if (eChannel < TIM_CH3)
    {
        pulCCMR = &pxTIM->Inst->CCMR1.w;
    }
    else
    {
        pulCCMR = &pxTIM->Inst->CCMR2.w;
    }

    if (eOCREF_CLR_IN == TIM_OCREFCLEAR_SOURCE_NONE)
    {
        /* clear configuration from channel */
        CLEAR_BIT(*pulCCMR, TIM_CCMR1_OC1CE << ulChOffset);
    }
    else
    {
        /* apply configuration */
        SET_BIT(*pulCCMR, TIM_CCMR1_OC1CE << ulChOffset);

#ifdef TIM_SMCR_OCCS
        /* select ocref_clr_int source */
        TIM_REG_BIT(pxTIM,SMCR,OCCS) = eOCREF_CLR_IN;
#endif
    }
}

/**
 * @brief Configures the commutation source
 * @param pxTIM: pointer to the TIM handle structure
 * @param ComSource
 */
void TIM_vCommutationConfig(TIM_HandleType * pxTIM, TIM_CommutationSourceType eComSource)
{
    if (eComSource > TIM_COMSOURCE_SOFTWARE)
    {
        /* Set TRGI if used for commutation */
        MODIFY_REG(pxTIM->Inst->SMCR.w, TIM_SMCR_TS, eComSource);

        /* Select the Commutation event source */
        SET_BIT(pxTIM->Inst->CR2.w, TIM_CR2_CCPC | TIM_CR2_CCUS);
    }
    else
    {
        /* Select the Commutation event source */
        CLEAR_BIT(pxTIM->Inst->CR2.w, TIM_CR2_CCPC | TIM_CR2_CCUS);

        if (eComSource == TIM_COMSOURCE_SOFTWARE)
        {
            /* Select the Capture Compare preload feature */
            TIM_REG_BIT(pxTIM,CR2,CCPC) = 1;
        }
    }
}

/**
 * @brief TIM commutation interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_COM(TIM_HandleType * pxTIM)
{
    /* TIM commutation event */
    if (TIM_FLAG_STATUS(pxTIM, COM) != 0)
    {
        /* clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, COM);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Commutation, pxTIM);
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
 * @param pxTIM: pointer to the TIM handle structure
 * @param eChannel: the selected compare channel to use
 * @param pxConfig: pointer to TIM compare channel setup configuration
 */
void TIM_vInputChannelConfig(
        TIM_HandleType *            pxTIM,
        TIM_ChannelType             eChannel,
        const TIM_InputInitType *   pxConfig)
{
    /* Set the input polarity, disable channel */
    {
        uint32_t ulPol = 0;

        if (pxConfig->Polarity == ACTIVE_LOW)
        {
            ulPol = TIM_CCER_CC1P;
        }

        MODIFY_REG(pxTIM->Inst->CCER.w, 0xF << (eChannel * 4), ulPol << (eChannel * 4));
    }

    /* Input mode configuration */
    {
        __IO uint32_t * pulCCMR;
        uint32_t ulChCfg, ulChOffset = (eChannel & 1) * 8;

        /* get related CCMR register */
        if (eChannel < TIM_CH3)
        {
            pulCCMR = &pxTIM->Inst->CCMR1.w;
        }
        else
        {
            pulCCMR = &pxTIM->Inst->CCMR2.w;
        }

        /* Assemble input configuration */
        ulChCfg = ((pxConfig->Filter << TIM_CCMR1_IC1F_Pos) & TIM_CCMR1_IC1F)
                | ((pxConfig->Prescaler << TIM_CCMR1_IC1PSC_Pos) & TIM_CCMR1_IC1PSC)
                |  (pxConfig->Source & TIM_CCMR1_CC1S);

        /* apply configuration */
        *pulCCMR = (*pulCCMR
                 &((TIM_CCMR1_CC2S | TIM_CCMR1_OC2CE | TIM_CCMR1_OC2FE |
                    TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE) >> ulChOffset))
                | (ulChCfg << ulChOffset);
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
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM master setup configuration
 */
void TIM_vMasterConfig(TIM_HandleType * pxTIM, const TIM_MasterConfigType * pxConfig)
{
    /* select the TRGO source */
    pxTIM->Inst->CR2.b.MMS = pxConfig->MasterTrigger;

#ifdef TIM_CR2_MMS2
    /* select the TRGO2 source */
    pxTIM->Inst->CR2.b.MMS2 = pxConfig->MasterTrigger2;
#endif

    /* configure the MSM bit - delay TRGI to synchronize with TRGO */
    TIM_REG_BIT(pxTIM,SMCR,MSM) = pxConfig->MasterSlaveMode;
}

/**
 * @brief Sets up the slave configuration of the timer.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM slave setup configuration
 */
void TIM_vSlaveConfig(TIM_HandleType * pxTIM, const TIM_SlaveConfigType * pxConfig)
{
    /* Set the mandatory configuration elements */
    MODIFY_REG(pxTIM->Inst->SMCR.w,
            TIM_SMCR_ECE | TIM_SMCR_ETPS | TIM_SMCR_TS | TIM_SMCR_SMS,
            pxConfig->SlaveMode | pxConfig->SlaveTrigger);

    /* Configure the trigger prescaler, filter, and polarity (where applicable) */
    switch (pxConfig->SlaveTrigger)
    {
        case TIM_TRGI_TI1:
        case TIM_TRGI_TI1_ED:
        {
            /* Set the input polarity, disable channel */
            CLEAR_BIT(pxTIM->Inst->CCER.w, 0xF);

            if (pxConfig->Polarity == ACTIVE_LOW)
            {
                TIM_REG_BIT(pxTIM,CCER,CC1P) = 1;
            }

            /* Set the filter */
            pxTIM->Inst->CCMR1.IC.C1F = pxConfig->Filter;
            break;
        }

        case TIM_TRGI_TI2:
        {
            /* Set the input polarity, disable channel */
            CLEAR_BIT(pxTIM->Inst->CCER.w, 0xF0);

            if (pxConfig->Polarity == ACTIVE_LOW)
            {
                TIM_REG_BIT(pxTIM,CCER,CC2P) = 1;
            }

            /* Set the filter */
            pxTIM->Inst->CCMR1.IC.C2F = pxConfig->Filter;
            break;
        }

        case TIM_TRGI_ETR_MODE1:
        case TIM_TRGI_ETR_MODE2:
        {
            /* Configure the external trigger settings */
            pxTIM->Inst->SMCR.b.ETF  = pxConfig->Filter;
            pxTIM->Inst->SMCR.b.ETPS = pxConfig->Prescaler;
            pxTIM->Inst->SMCR.b.ETP  = pxConfig->Polarity;
            break;
        }

        default:
            break;
    }
}

/**
 * @brief TIM trigger interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_TRG(TIM_HandleType * pxTIM)
{
    /* TIM Trigger detection event */
    if (TIM_FLAG_STATUS(pxTIM, T) != 0)
    {
        /* clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, T);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Trigger, pxTIM);
    }
}

/** @} */

/** @} */

/** @} */
