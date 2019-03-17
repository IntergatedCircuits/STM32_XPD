/**
  ******************************************************************************
  * @file    xpd_tim.c
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

/** @defgroup TIM_Private_Functions TIM Private Functions
 * @{ */

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

/**
 * @brief Enables peripheral clock and initializes common TIM registers.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM setup configuration
 */
static void TIM_prvInit(TIM_HandleType * pxTIM, const TIM_InitType * pxConfig)
{
    /* Enable clock and reset peripheral */
    RCC_vClockEnable(pxTIM->CtrlPos);
    RCC_vReset(pxTIM->CtrlPos);

    pxTIM->Inst->CR1.w = pxConfig->wSettings & (TIM_CR1_UDIS | TIM_CR1_URS | TIM_CR1_OPM
#ifdef TIM_CR1_UIFREMAP
            | TIM_CR1_UIFREMAP
#endif
            | TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD);

    pxTIM->Inst->PSC = (uint32_t)(pxConfig->Prescaler - 1);
    pxTIM->Inst->ARR = (uint32_t)(pxConfig->Period - 1);
    pxTIM->Inst->RCR = pxConfig->RepetitionCounter;
}

/** @} */

/** @defgroup TIM_Common_Exported_Functions TIM Common Exported Functions
 * @{ */

/**
 * @brief Initializes the TIM peripheral and its selected channels.
 * @note  If no channels are used, @ref TIM_vCounterInit can be used instead.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM setup configuration
 * @param axChannels: array of channel configurations
 * @param ucChannelCount: number of channels to configure
 */
void TIM_vInit(
        TIM_HandleType *            pxTIM,
        const TIM_InitType *        pxConfig,
        const TIM_ChannelInitType   axChannels[],
        uint8_t                     ucChannelCount)
{
    uint8_t i;
    uint32_t ulCCER = 0, ulCR2 = 0;
    uint32_t aulCCMR[TIM_SUPPORTED_CHANNEL_COUNT / 2] = {0};

    /* Init the timer common core */
    TIM_prvInit(pxTIM, pxConfig);

    pxTIM->ChannelMask = 0;
    pxTIM->EventMask = 0;

    /* Channels configuration */
    for (i = 0; i < ucChannelCount; i++)
    {
        TIM_ChannelType eChannel = axChannels[i].OC.Number;

        /* Save used channels for fast operations */
        pxTIM->ChannelMask |= (TIM_CCER_CC1E | (axChannels[i].IC.b[2] & TIM_CCER_CC1NE))
                << (eChannel * 4);

        /* Set the capture/compare polarities (0 is active high) */
        ulCCER |= (uint32_t)(axChannels[i].IC.b[2] & (TIM_CCER_CC1P | TIM_CCER_CC1NP))
                << (eChannel * 4);

        /* Set initial channel value */
        *TIM_pulChannel(pxTIM, eChannel) = axChannels[i].OC.Value;

#if TIM_SUPPORTED_CHANNEL_COUNT > 5
        if (eChannel > TIM_CH4) /* CH5 and 6 have reduced functionality */
        {   continue; }
#endif
        pxTIM->EventMask |= TIM_DIER_CC1IE << eChannel;

        /* Set the Output Idle states */
        ulCR2 |= (uint32_t)axChannels[i].IC.b[3] << (eChannel * 2);

        /* Add output configuration */
        aulCCMR[eChannel / 2] |= (axChannels[i].OC.w & 0x1FFFF) << (eChannel & 1) * 8;
    }

    /* Copy assembled settings to registers */
    pxTIM->Inst->CR2.w = ulCR2 << TIM_CR2_OIS1_Pos;
    pxTIM->Inst->CCMR1.w = aulCCMR[0];
    pxTIM->Inst->CCMR2.w = aulCCMR[1];
#if TIM_SUPPORTED_CHANNEL_COUNT > 5
    pxTIM->Inst->CCMR3.w = aulCCMR[2];
#endif
    pxTIM->Inst->CCER.w = ulCCER;

    /* Enable automatic output by default in order to ensure
     * matching behavior of advanced and regular timers */
    TIM_REG_BIT(pxTIM, BDTR, AOE) = 1;

    /* Generate update which loads ARR and CCRx registers */
    TIM_vGenerate(pxTIM, TIM_EVENT_UPDATE);
    while (TIM_FLAG_STATUS(pxTIM, U) == 0);
    TIM_FLAG_CLEAR(pxTIM, U);

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.DepInit, pxTIM);
}

/**
 * @brief Deinitializes the TIM peripheral.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vDeinit(TIM_HandleType * pxTIM)
{
    /* Stop the timer */
    TIM_vStop(pxTIM);

    /* Disable master output */
    TIM_REG_BIT(pxTIM, BDTR, MOE) = 0;

    /* Deinitialize peripheral dependencies */
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.DepDeinit, pxTIM);

    /* Disable clock */
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
    TIM_vIRQHandler_BRK(pxTIM);
}

/**
 * @brief Configures burst DMA transfers.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pulStartReg: pointer to the first register of the burst transfer
 * @param ucRegCount: number of registers to transfer in each burst (set to 0 to disable bursts)
 */
void TIM_vBurstConfig(TIM_HandleType * pxTIM, __IO uint32_t * pulStartReg, uint8_t ucRegCount)
{
    if (ucRegCount > 0)
    {
        pxTIM->Inst->DCR.b.DBA = (uint32_t)(pulStartReg - pxTIM->Inst->CR1.w) / sizeof(uint32_t);
        pxTIM->Inst->DCR.b.DBL = ucRegCount - 1;
    }
    else
    {
        pxTIM->Inst->DCR.w = 0;
    }
}

/**
 * @brief Enables the TIM counter and configured channels.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vStart(TIM_HandleType * pxTIM)
{
    SET_BIT(pxTIM->Inst->CCER.w, pxTIM->ChannelMask);
    TIM_vCounterStart(pxTIM);
}

/**
 * @brief Disables the TIM counter and configured channels.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vStop(TIM_HandleType * pxTIM)
{
    CLEAR_BIT(pxTIM->Inst->CCER.w, pxTIM->ChannelMask);
    TIM_vCounterStop(pxTIM);
}

/**
 * @brief Sets up and enables a DMA transfer for the TIM counter.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eEvent: DMA transfer trigger event
 * @param pvAddress: memory address of the counter data
 * @param usLength: the amount of data to be transferred
 * @return ERROR if burst isn't configured and event doesn't have default register,
 *         BUSY if the DMA is used by other peripheral, OK otherwise
 */
XPD_ReturnType TIM_eOpen_DMA(
        TIM_HandleType *    pxTIM,
        TIM_EventType       eEvent,
        void *              pvAddress,
        uint16_t            usLength)
{
    XPD_ReturnType eResult = XPD_ERROR;
    DMA_HandleType* pxDMA = NULL;
    void* pvRegAddr;

    /* Find DMA parameters by TIM configuration */
    if (pxTIM->Inst->DCR.w > 0)
    {
        pxDMA = pxTIM->DMA.Burst;
        usLength *= pxTIM->Inst->DCR.b.DBL + 1;
        pvRegAddr = (void*)&pxTIM->Inst->DMAR;
    }
    else if (eEvent <= TIM_EVENT_CH4)
    {
        pxDMA = ((void*)pxTIM->DMA.Update) + eEvent;

        if (eEvent == TIM_EVENT_UPDATE)
        {
            pvRegAddr = (void*)TIM_pulReload(pxTIM);
        }
        else
        {
            pvRegAddr = (void*)TIM_pulChannel(pxTIM, eEvent - TIM_EVENT_CH1);
        }
    }

    if (pxDMA != NULL)
    {
        /* Set up DMA for transfer */
        eResult = DMA_eStart_IT(pxDMA, pvRegAddr, pvAddress, usLength);

        /* If the DMA is currently used, return with error */
        if (eResult == XPD_OK)
        {
            /* Set the callback owner */
            pxTIM->DMA.Update->Owner = pxTIM;

            /* Set the DMA transfer callbacks */
            pxTIM->DMA.Update->Callbacks.Complete = TIM_apxDmaRedirects[eEvent];
#ifdef __XPD_DMA_ERROR_DETECT
            pxTIM->DMA.Update->Callbacks.Error    = TIM_prvDmaErrorRedirect;
#endif

            /* enable the TIM event's DMA request */
            SET_BIT(pxTIM->Inst->DIER.w, eEvent << TIM_DIER_UDE_Pos);
        }
    }

    return eResult;
}

/**
 * @brief Disables the DMA transfer.
 * @param pxTIM: pointer to the TIM handle structure
 * @param eEvent: DMA transfer trigger event
 */
void TIM_vClose_DMA(TIM_HandleType * pxTIM, TIM_EventType eEvent)
{
    DMA_HandleType* pxDMA = NULL;

    if (pxTIM->Inst->DCR.w > 0)
    {
        pxDMA = pxTIM->DMA.Burst;
    }
    else if (eEvent <= TIM_EVENT_CH4)
    {
        pxDMA = ((void*)pxTIM->DMA.Update) + eEvent;
    }

    if (pxDMA != NULL)
    {
        /* disable the update DMA request */
        CLEAR_BIT(pxTIM->Inst->DIER.w, eEvent << TIM_DIER_UDE_Pos);

        DMA_vStop_IT(pxDMA);
    }
}

/** @} */

/** @} */

/** @addtogroup TIM_Counter
 * @{ */

/** @defgroup TIM_Counter_Exported_Functions TIM Counter Exported Functions
 * @{ */

/**
 * @brief Initializes the TIM counter using the setup configuration.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM setup configuration
 */
void TIM_vCounterInit(TIM_HandleType * pxTIM, const TIM_InitType * pxConfig)
{
    TIM_prvInit(pxTIM, pxConfig);

    /* Disable all channels */
    pxTIM->ChannelMask = 0;
    pxTIM->Inst->CCER.w = 0;

    pxTIM->EventMask = TIM_EVENT_UPDATE;

    /* Generate update which loads ARR and CCRx registers */
    TIM_vGenerate(pxTIM, TIM_EVENT_UPDATE);
    while (TIM_FLAG_STATUS(pxTIM, U) == 0);
    TIM_FLAG_CLEAR(pxTIM, U);

    /* Dependencies initialization */
    XPD_SAFE_CALLBACK(pxTIM->Callbacks.DepInit, pxTIM);
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
    if ((pxTIM->Inst->SR.w & pxTIM->Inst->DIER.w & TIM_SR_UIF) != 0)
    {
        /* Clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, U);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Update, pxTIM);
    }
}

/** @} */

/** @} */

/** @addtogroup TIM_Channel
 * @{ */

/** @defgroup TIM_Channel_Exported_Functions TIM Channel Exported Functions
 * @{ */

/**
 * @brief TIM channel event interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_CC(TIM_HandleType * pxTIM)
{
    TIM_ChannelType eChannel = TIM_CH1;
    uint32_t ulFlags = (pxTIM->Inst->SR.w & pxTIM->Inst->DIER.w
            & (TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF))
                >> TIM_SR_CC1IF_Pos;

    /* Iterate through active channel flags */
    while (ulFlags > 0)
    {
        if ((ulFlags & 1) != 0)
        {
            /* Clear interrupt flag */
            TIM_vChannelClearFlag(pxTIM, eChannel);

            /* Provide channel number for interrupt callback context */
            pxTIM->ActiveChannel = eChannel;
            XPD_SAFE_CALLBACK(pxTIM->Callbacks.ChannelEvent, pxTIM);
        }
        eChannel++;
        ulFlags >>= 1;
    }
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
 * @brief Sets up the output drive configuration of the timer.
 * @param pxTIM: pointer to the TIM handle structure
 * @param pxConfig: pointer to TIM break setup configuration
 */
void TIM_vDriveConfig(TIM_HandleType * pxTIM, const TIM_DriveInitType * pxConfig)
{
    uint32_t ulBDTR = pxConfig->w;

    /* Break polarity bits are interpreted in reverse */
#ifdef TIM_BDTR_BK2P
    ulBDTR ^= TIM_BDTR_BKP | TIM_BDTR_BK2P;
#else
    ulBDTR ^= TIM_BDTR_BKP;
#endif

    MODIFY_REG(pxTIM->Inst->BDTR.w, TIM_BDTR_DTG | TIM_BDTR_OSSI | TIM_BDTR_OSSR
            | TIM_BDTR_AOE | TIM_BDTR_BKE | TIM_BDTR_BKP
#ifdef TIM_BDTR_BKF
            | TIM_BDTR_BKF
#endif
#ifdef TIM_BDTR_BK2E
            | TIM_BDTR_BK2E | TIM_BDTR_BK2P | TIM_BDTR_BK2F
#endif
            , ulBDTR);
}

/**
 * @brief Calculates dead time register value from dead time in timer counts.
 * @param pxTIM: pointer to the TIM handle structure
 * @param ulDeadtimeCounts: the dead time value in timer counts
 * @return The dead time in register configuration format
 */
uint8_t TIM_ucCalcDeadtime(TIM_HandleType * pxTIM, uint32_t ulDeadtimeCounts)
{
    uint8_t ucDeadtime;

    /* Divide by clock division */
    ulDeadtimeCounts >>= pxTIM->Inst->CR1.b.CKD;

    /* Revert the configuration characteristic */
    if (ulDeadtimeCounts < 128)
    {
        /* dead counts = DTG */
        ucDeadtime = ulDeadtimeCounts;
    }
    else if (ulDeadtimeCounts < 256)
    {
        /* dead counts =(64+DTG[5:0])*2 */
        ucDeadtime = 0x80 | ((ulDeadtimeCounts / 2) - 64);
    }
    else if (ulDeadtimeCounts < 512)
    {
        /* dead counts =(32+DTG[4:0])*8 */
        ucDeadtime = 0xC0 | ((ulDeadtimeCounts / 8) - 32);
    }
    else if (ulDeadtimeCounts < 1008)
    {
        /* dead counts =(32+DTG[4:0])*16 */
        ucDeadtime = 0xE0 | ((ulDeadtimeCounts / 16) - 32);
    }
    else
    {
        /* saturation */
        ucDeadtime = 255;
    }

    return ucDeadtime;
}

/**
 * @brief TIM break event interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_BRK(TIM_HandleType * pxTIM)
{
    /* TIM Break input event */
    if ((pxTIM->Inst->SR.w & pxTIM->Inst->DIER.w & TIM_SR_BIF) != 0)
    {
        /* Clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, B);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Break, pxTIM);
    }
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
    if ((pxTIM->Inst->SR.w & pxTIM->Inst->DIER.w & TIM_SR_COMIF) != 0)
    {
        /* clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, COM);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Commutation, pxTIM);
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
    uint32_t ulSmcr = pxConfig->w;
    uint32_t ulSmcrMask = TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ETF
            | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP;

#ifdef TIM_SMCR_SMS_3
    if (pxConfig->SlaveMode == TIM_SLAVEMODE_RESET_TRIGGER)
    {
        ulSmcr |= TIM_SMCR_SMS_3;
    }
#endif
    MODIFY_REG(pxTIM->Inst->SMCR.w, ulSmcrMask, ulSmcr);
}

/**
 * @brief TIM trigger interrupt handler that provides handle callbacks.
 * @param pxTIM: pointer to the TIM handle structure
 */
void TIM_vIRQHandler_TRG(TIM_HandleType * pxTIM)
{
    /* TIM Trigger detection event */
    if ((pxTIM->Inst->SR.w & pxTIM->Inst->DIER.w & TIM_SR_TIF) != 0)
    {
        /* clear interrupt flag */
        TIM_FLAG_CLEAR(pxTIM, T);

        XPD_SAFE_CALLBACK(pxTIM->Callbacks.Trigger, pxTIM);
    }
}

/** @} */

/** @} */

/** @} */
