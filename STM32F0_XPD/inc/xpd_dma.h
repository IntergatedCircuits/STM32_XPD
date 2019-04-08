/**
  ******************************************************************************
  * @file    xpd_dma.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers DMA Module
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
#ifndef __XPD_DMA_H_
#define __XPD_DMA_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup DMA
 * @{ */

/** @defgroup DMA_Exported_Types DMA Exported Types
 * @{ */

/** @brief DMA transfer direction types */
typedef enum
{
    DMA_PERIPH2MEMORY = 0, /*!< Data is transferred from peripheral to memory */
    DMA_MEMORY2PERIPH = 2, /*!< Data is transferred from memory to peripheral */
    DMA_MEMORY2MEMORY = 1, /*!< Data is transferred from memory to memory */
}DMA_DirectionType;

/** @brief DMA data alignment types */
typedef enum
{
    DMA_ALIGN_BYTE     = 0, /*!< Data is byte aligned */
    DMA_ALIGN_HALFWORD = 1, /*!< Data is half word aligned */
    DMA_ALIGN_WORD     = 2  /*!< Data is word aligned */
}DMA_AlignmentType;

/** @brief DMA transfer mode types */
typedef enum
{
    DMA_MODE_NORMAL    = 0, /*!< Normal DMA transfer */
    DMA_MODE_CIRCULAR  = 1, /*!< Circular DMA transfer */
}DMA_ModeType;

/** @brief DMA transfer error types */
typedef enum
{
    DMA_ERROR_NONE     = 0,  /*!< No error             */
    DMA_ERROR_TRANSFER = 1,  /*!< Transfer error       */
}DMA_ErrorType;

/** @brief DMA operation types */
typedef enum
{
    DMA_OPERATION_TRANSFER     = 0, /*!< DMA transfer operation */
    DMA_OPERATION_HALFTRANSFER = 1  /*!< DMA half transfer operation */
}DMA_OperationType;

/** @brief DMA channel setup structure */
typedef union
{
    struct {
    uint16_t : 3;
    DMA_DirectionType Direction : 2;       /*!< DMA channel direction */
    DMA_ModeType      Mode : 1;            /*!< DMA operating mode */
    FunctionalState   PeriphInc : 1;       /*!< The peripheral address is incremented after each transfer */
    FunctionalState   MemoryInc : 1;       /*!< The memory is incremented after each transfer */
    DMA_AlignmentType PeriphDataAlign : 2; /*!< The peripheral data width */
    DMA_AlignmentType MemoryDataAlign : 2; /*!< The memory data width */
    LevelType         Priority : 2;        /*!< DMA bus arbitration priority level */
    uint16_t : 2;
    };
    uint16_t w;
}DMA_InitType;

/** @brief DMA channel handle structure */
typedef struct
{
    DMA_Channel_TypeDef * Inst;               /*!< The address of the peripheral instance used by the handle */
#ifdef DMA_BB
    DMA_Channel_BitBand_TypeDef * Inst_BB;    /*!< The address of the peripheral instance in the bit-band region */
#endif
    DMA_TypeDef * Base;                       /*!< [Internal] The address of the master DMA used by the handle */
    uint8_t ChannelOffset;                    /*!< [Internal] The offset of the channel in the master DMA */
    struct {
        XPD_HandleCallbackType Complete;      /*!< DMA transfer complete callback */
        XPD_HandleCallbackType HalfComplete;  /*!< DMA transfer half complete callback */
#ifdef __XPD_DMA_ERROR_DETECT
        XPD_HandleCallbackType Error;         /*!< DMA transfer error callback */
#endif
    } Callbacks;                              /*   Handle Callbacks */
    void * Owner;                             /*!< [Internal] The pointer of the peripheral handle which uses this handle */
#ifdef __XPD_DMA_ERROR_DETECT
    volatile DMA_ErrorType Errors;            /*!< Transfer errors */
#endif
}DMA_HandleType;

/** @} */

/** @defgroup DMA_Exported_Macros DMA Exported Macros
 * @{ */

#ifdef DMA_Channel_BB
/**
 * @brief DMA Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the DMA peripheral instance.
 */
#define         DMA_INST2HANDLE(HANDLE, INSTANCE)           \
    ((HANDLE)->Inst    = (INSTANCE),                        \
     (HANDLE)->Inst_BB = DMA_Channel_BB(INSTANCE))

/**
 * @brief DMA register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME)     \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief DMA Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the DMA peripheral instance.
 */
#define         DMA_INST2HANDLE(HANDLE, INSTANCE)           \
    ((HANDLE)->Inst    = (INSTANCE))

/**
 * @brief DMA register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME)     \
    ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)

#endif /* DMA_Channel_BB */

/**
 * @brief  Enable the specified DMA interrupt.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete Interrupt
 *            @arg HT:      Half Transfer Complete Interrupt
 *            @arg TE:      Transfer Error Interrupt
 */
#define         DMA_IT_ENABLE(HANDLE, IT_NAME)              \
    (DMA_REG_BIT((HANDLE), CCR, IT_NAME##IE) = 1)

/**
 * @brief  Disable the specified DMA interrupt.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete Interrupt
 *            @arg HT:      Half Transfer Complete Interrupt
 *            @arg TE:      Transfer Error Interrupt
 */
#define         DMA_IT_DISABLE(HANDLE, IT_NAME)             \
        (DMA_REG_BIT((HANDLE), CCR, IT_NAME##IE) = 0)

/**
 * @brief  Get the specified DMA flag.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete
 *            @arg HT:      Half Transfer Complete
 *            @arg TE:      Transfer Error
 */
#define         DMA_FLAG_STATUS(HANDLE, FLAG_NAME)          \
    (((HANDLE)->Base->ISR.w >> (DMA_ISR_##FLAG_NAME##IF1_Pos\
                + (uint32_t)((HANDLE)->ChannelOffset))) & 1)

/**
 * @brief  Clear the specified DMA flag.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete
 *            @arg HT:      Half Transfer Complete
 *            @arg TE:      Transfer Error
 */
#define         DMA_FLAG_CLEAR(HANDLE, FLAG_NAME)           \
    ((HANDLE)->Base->IFCR.w = (DMA_IFCR_C##FLAG_NAME##IF1   \
                << (uint32_t)((HANDLE)->ChannelOffset)))

#ifdef DMA_CSELR_C1S

/* Additional defines for complete macro functionality */
#define DMA1_CSELR_CH1_DEFAULT      0U
#define DMA1_CSELR_CH2_DEFAULT      0U
#define DMA1_CSELR_CH3_DEFAULT      0U
#define DMA1_CSELR_CH4_DEFAULT      0U
#define DMA1_CSELR_CH5_DEFAULT      0U
#define DMA1_CSELR_CH6_DEFAULT      0U
#define DMA1_CSELR_CH7_DEFAULT      0U
#define DMA2_CSELR_CH1_DEFAULT      0U
#define DMA2_CSELR_CH2_DEFAULT      0U
#define DMA2_CSELR_CH3_DEFAULT      0U
#define DMA2_CSELR_CH4_DEFAULT      0U
#define DMA2_CSELR_CH5_DEFAULT      0U
#define DMA2_CSELR_CH6_DEFAULT      0U
#define DMA2_CSELR_CH7_DEFAULT      0U

/**
 * @brief  Sets the DMA remapping for the given channel.
 * @param  BASE: specifies the name of the DMA base.
 *         This parameter can be one of the following values:
 *            @arg DMA1
 *            @arg DMA2 (if available)
 * @param  CHANNEL: specifies the channel number to remap [1..7].
 * @param  SELECTION: specify the remap target selection. Using DEFAULT resets the remapping.
 */
#define DMA_CHANNEL_REMAP(BASE, CHANNEL, SELECTION)         \
    (MODIFY_REG(BASE->CSELR.w, 0xF << (((CHANNEL) - 1) * 4),\
     BASE##_CSELR_CH##CHANNEL##_##SELECTION))
#endif /* DMA_CSELR_C1S */

/** @} */

/** @addtogroup DMA_Exported_Functions
 * @{ */
void            DMA_vInit           (DMA_HandleType * pxDMA, const DMA_InitType * pxConfig);
void            DMA_vDeinit         (DMA_HandleType * pxDMA);

XPD_ReturnType  DMA_eStart          (DMA_HandleType * pxDMA, void * pvPeriphAddress,
                                     void * pvMemAddress, uint16_t usDataCount);
XPD_ReturnType  DMA_eStart_IT       (DMA_HandleType * pxDMA, void * pvPeriphAddress,
                                     void * pvMemAddress, uint16_t usDataCount);
void            DMA_vStop           (DMA_HandleType * pxDMA);
void            DMA_vStop_IT        (DMA_HandleType * pxDMA);

uint16_t        DMA_usGetStatus     (DMA_HandleType * pxDMA);
XPD_ReturnType  DMA_ePollStatus     (DMA_HandleType * pxDMA, DMA_OperationType eOperation,
                                     uint32_t ulTimeout);

void            DMA_vIRQHandler     (DMA_HandleType * pxDMA);

/**
 * @brief  Provides the circular mode of DMA stream.
 * @param  HANDLE: specifies the DMA Handle.
 */
__STATIC_INLINE bool DMA_eCircularMode(DMA_HandleType * pxDMA)
{
    return DMA_REG_BIT(pxDMA, CCR, CIRC);
}

#ifdef __XPD_DMA_ERROR_DETECT
/**
 * @brief Gets the error state of the DMA stream.
 * @param pxDMA: pointer to the DMA stream handle structure
 * @return Current DMA error state
 */
__STATIC_INLINE DMA_ErrorType DMA_eGetError(DMA_HandleType * pxDMA)
{
    return pxDMA->Errors;
}
#endif

/** @} */

/** @} */

#define XPD_DMA_API
#include <xpd_syscfg.h>
#undef XPD_DMA_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_DMA_H_ */
