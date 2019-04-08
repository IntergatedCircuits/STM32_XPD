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
    DMA_MEMORY2PERIPH = 1, /*!< Data is transferred from memory to peripheral */
    DMA_MEMORY2MEMORY = 2  /*!< Data is transferred from memory to memory
                                @note Must use normal mode with FIFO for this direction */
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
    DMA_MODE_PERIPH_FC = 4, /*!< DMA transfer with peripheral flow control (completion is indicated by peripheral) */
    DMA_MODE_DBUFFER   = 3, /*!< Double-buffered DMA transfer (which is inherently circular) */
}DMA_ModeType;

/** @brief DMA FIFO burst mode types */
typedef enum
{
    DMA_BURST_SINGLE = 0, /*!< FIFOs use single transfer */
    DMA_BURST_INC4   = 1, /*!< FIFOs transfer 4 data items in one burst */
    DMA_BURST_INC8   = 2, /*!< FIFOs transfer 8 data items in one burst */
    DMA_BURST_INC16  = 3  /*!< FIFOs transfer 16 data items in one burst */
}DMA_BurstType;

/** @brief DMA transfer error types */
typedef enum
{
    DMA_ERROR_NONE     = 0,  /*!< No error             */
    DMA_ERROR_TRANSFER = 1,  /*!< Transfer error       */
    DMA_ERROR_FIFO     = 2,  /*!< FIFO error           */
    DMA_ERROR_DIRECTM  = 4,  /*!< Direct Mode error    */
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
    uint32_t : 3;
    DMA_ModeType      Mode : 3;            /*!< DMA operating mode */
    DMA_DirectionType Direction : 2;       /*!< DMA channel direction */
    uint32_t : 1;
    FunctionalState   PeriphInc : 1;       /*!< The peripheral address is incremented after each transfer */
    FunctionalState   MemoryInc : 1;       /*!< The memory is incremented after each transfer */
    DMA_AlignmentType PeriphDataAlign : 2; /*!< The peripheral data width */
    DMA_AlignmentType MemoryDataAlign : 2; /*!< The memory data width */
    FunctionalState   PeriphWordInc : 1;   /*!< When enabled with @ref DMA_InitType::PeriphInc,
                                                the peripheral address is incremented by 32 bits
                                                (instead of what's dictated by @ref DMA_InitType::PeriphDataAlign) */
    LevelType         Priority : 2;        /*!< DMA bus arbitration priority level */
    uint32_t : 3;
    DMA_BurstType     PeriphBurst : 2;     /*!< The peripheral burst size (forced to 0 when FIFO isn't used) */
    DMA_BurstType     MemoryBurst : 2;     /*!< The memory burst size (forced to 0 when FIFO isn't used) */
    uint32_t          Channel : 3;         /*!< Channel selection for the DMA stream [0 .. 7] */
    uint32_t : 1;
    uint32_t          FifoThreshold : 3;   /*!< The number of FIFO quarters to fill before transfer [1 .. 4]
                                                (set to 0 to disable the FIFO) */
    };
    uint32_t w;
}DMA_InitType;

/** @brief DMA stream handle structure */
typedef struct
{
    DMA_Stream_TypeDef * Inst;                /*!< The address of the peripheral instance used by the handle */
#ifdef DMA_Stream_BB
    DMA_Stream_BitBand_TypeDef * Inst_BB;     /*!< The address of the peripheral instance in the bit-band region */
#endif
    DMA_TypeDef * Base;                       /*!< [Internal] The address of the master DMA used by the handle */
    uint8_t StreamOffset;                     /*!< [Internal] The offset of the stream in the master DMA */
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

#ifdef DMA_Stream_BB
/**
 * @brief DMA Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the DMA peripheral instance.
 */
#define         DMA_INST2HANDLE(HANDLE, INSTANCE)           \
    ((HANDLE)->Inst    = (INSTANCE),                        \
     (HANDLE)->Inst_BB = DMA_Stream_BB(INSTANCE))

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

#endif /* DMA_Stream_BB */

/**
 * @brief  Enable the specified DMA interrupt.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete Interrupt
 *            @arg HT:      Half Transfer Complete Interrupt
 *            @arg TE:      Transfer Error Interrupt
 *            @arg DME:     Direct Mode Error Interrupt
 *            @arg FE:      FIFO Error Interrupt
 */
#define         DMA_IT_ENABLE(HANDLE, IT_NAME)              \
    __XPD_DMA_ITConfig_##IT_NAME(HANDLE, 1)

/**
 * @brief  Disable the specified DMA interrupt.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete Interrupt
 *            @arg HT:      Half Transfer Complete Interrupt
 *            @arg TE:      Transfer Error Interrupt
 *            @arg DME:     Direct Mode Error Interrupt
 *            @arg FE:      FIFO Error Interrupt
 */
#define         DMA_IT_DISABLE(HANDLE, IT_NAME)             \
    __XPD_DMA_ITConfig_##IT_NAME(HANDLE, 0)

/**
 * @brief  Get the specified DMA flag.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete
 *            @arg HT:      Half Transfer Complete
 *            @arg TE:      Transfer Error
 *            @arg DME:     Direct Mode Error
 *            @arg FE:      FIFO Error
 */
#define         DMA_FLAG_STATUS(HANDLE, FLAG_NAME)          \
    (((HANDLE)->Base->LISR.w >> (DMA_LISR_##FLAG_NAME##IF0_Pos \
            + (uint32_t)((HANDLE)->StreamOffset))) & 1)

/**
 * @brief  Clear the specified DMA flag.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete
 *            @arg HT:      Half Transfer Complete
 *            @arg TE:      Transfer Error
 *            @arg DME:     Direct Mode Error
 *            @arg FE:      FIFO Error
 */
#define         DMA_FLAG_CLEAR(HANDLE, FLAG_NAME)           \
    ((HANDLE)->Base->LIFCR.w = (DMA_LIFCR_C##FLAG_NAME##IF0 \
            << (uint32_t)((HANDLE)->StreamOffset)))

#define         __XPD_DMA_ITConfig_TC(HANDLE,VALUE)         \
    (DMA_REG_BIT((HANDLE),CR,TCIE) = (VALUE))
#define         __XPD_DMA_ITConfig_TE(HANDLE,VALUE)         \
    (DMA_REG_BIT((HANDLE),CR,TEIE) = (VALUE))
#define         __XPD_DMA_ITConfig_HT(HANDLE,VALUE)         \
    (DMA_REG_BIT((HANDLE),CR,HTIE) = (VALUE))
#define         __XPD_DMA_ITConfig_DME(HANDLE,VALUE)        \
    (DMA_REG_BIT((HANDLE),CR,DMEIE) = (VALUE))
#define         __XPD_DMA_ITConfig_FE(HANDLE,VALUE)         \
    (DMA_REG_BIT((HANDLE),FCR,FEIE) = (VALUE))

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

uint32_t        DMA_ulActiveMemory  (DMA_HandleType * pxDMA);
void            DMA_vSetSwapMemory  (DMA_HandleType * pxDMA, void * pvAddress);

/**
 * @brief  Provides the circular mode of DMA stream.
 * @param  HANDLE: specifies the DMA Handle.
 */
__STATIC_INLINE bool DMA_eCircularMode(DMA_HandleType * pxDMA)
{
    return DMA_REG_BIT(pxDMA, CR, CIRC);
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
