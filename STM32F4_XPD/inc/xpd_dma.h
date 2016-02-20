/**
  ******************************************************************************
  * @file    xpd_dma.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-19
  * @brief   STM32 eXtensible Peripheral Drivers DMA Module
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
#ifndef XPD_DMA_H_
#define XPD_DMA_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup DMA
 * @{ */

/** @defgroup DMA_Exported_Types DMA Exported Types
 * @{ */

/** @brief DMA transfer direction types */
typedef enum
{
    DMA_PERIPH2MEMORY = 0, /*!< Data is transferred from peripheral to memory */
    DMA_MEMORY2PERIPH = 1, /*!< Data is transferred from memory to peripheral */
    DMA_MEMORY2MEMORY = 2  /*!< Data is transferred from memory to memory */
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
    DMA_MODE_PERIPH_FC = 2, /*!< DMA transfer with peripheral flow control */
    DMA_MODE_DBUFFER   = 5  /*!< Double-buffered DMA transfer (which is inherently circular) */
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
typedef struct
{
    uint8_t           Channel;          /*!< Channel selection for the DMA stream [0 .. 7] */
    DMA_DirectionType Direction;        /*!< DMA stream direction */
    DMA_ModeType      Mode;             /*!< DMA operating mode */
    LevelType         Priority;         /*!< DMA bus arbitration priority level */
    struct {
        FunctionalState   Increment;     /*!< The address is incremented after each transfer */
        DMA_AlignmentType DataAlignment; /*!< The data width */
        DMA_BurstType     Burst;         /*!< The burst size */
    }Memory;                             /*  Memory side configuration */
    struct {
        FunctionalState   Increment;     /*!< The address is incremented after each transfer */
        DMA_AlignmentType DataAlignment; /*!< The data width */
        DMA_BurstType     Burst;         /*!< The burst size */
    }Peripheral;                         /*   Peripheral side configuration */
    struct {
        FunctionalState Mode;            /*!< FIFO mode is used */
        uint8_t         Threshold;       /*!< The number of quarters to fill before transfer [1 .. 4] */
    }FIFO;                               /*   FIFO configuration */
}DMA_InitType;

/** @brief DMA transfer setup structure */
typedef struct
{
    void * Peripheral;        /*!< Start address of the peripheral */
    void * Memory;            /*!< Start address of the memory */
    void * SwapMemory;        /*!< Start address of the swap memory (only used in double-buffered mode) */
    uint16_t DataCount;       /*!< The amount of data to transfer */
}DMA_TransferType;

/** @brief DMA stream handle structure */
typedef struct
{
    DMA_Stream_TypeDef * Inst;               /*!< The address of the peripheral instance used by the handle */
#ifdef DMA_Stream_BB
    DMA_Stream_BitBand_TypeDef * Inst_BB;    /*!< The address of the peripheral instance in the bit-band region */
#endif
    struct {
        XPD_HandleCallbackType Complete;     /*!< DMA transfer complete callback */
        XPD_HandleCallbackType HalfComplete; /*!< DMA transfer half complete callback */
        XPD_HandleCallbackType Error;        /*!< DMA transfer error callback */
    } Callbacks;                             /*   Handle Callbacks */
    void * Owner;                            /*!< [Internal] The pointer of the peripheral handle which uses this handle */
    volatile DMA_ErrorType Errors;           /*!< Transfer errors */
#ifdef DMA_BB
    void * Base_BB;                          /*!< [Internal] The address of the master DMA bits used by the handle */
#else
    /* TODO */
#endif
}DMA_HandleType;

/** @} */

/** @addtogroup DMA_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_DMA_Init            (DMA_HandleType * hdma, DMA_InitType * Config);
XPD_ReturnType  XPD_DMA_Deinit          (DMA_HandleType * hdma);

void            XPD_DMA_Enable          (DMA_HandleType * hdma);
void            XPD_DMA_Disable         (DMA_HandleType * hdma);

void            XPD_DMA_Start           (DMA_HandleType * hdma, DMA_TransferType * Config);
void            XPD_DMA_Start_IT        (DMA_HandleType * hdma, DMA_TransferType * Config);
XPD_ReturnType  XPD_DMA_Abort           (DMA_HandleType * hdma);

XPD_ReturnType  XPD_DMA_PollStatus      (DMA_HandleType * hdma, DMA_OperationType Operation, uint32_t Timeout);
DMA_ErrorType   XPD_DMA_GetError        (DMA_HandleType * hdma);

void            XPD_DMA_IRQHandler      (DMA_HandleType * hdma);

uint32_t        XPD_DMA_GetActiveMemory (DMA_HandleType * hdma);
void            XPD_DMA_SetSwapMemory   (DMA_HandleType * hdma, void * Address);
/** @} */

/** @defgroup DMA_Exported_Macros DMA Exported Macros
 * @{ */

/**
 * @brief  DMA Handle initializer macro
 * @param  INSTANCE: specifies the DMA stream instance.
 */
#define         NEW_DMA_HANDLE(INSTANCE)                    \
    {.Inst      = (INSTANCE),                               \
     .Callbacks = {NULL,NULL,NULL},                         \
     .Owner     = NULL}

/**
 * @brief  Binds a DMA handle to a peripheral handle in both directions.
 * @param  HDMA: specifies the DMA handle
 * @param  HANDLE: specifies the peripheral owner handle
 * @param  DMASLOT: specifies the DMA slot in the peripheral handle to set to
 */
#define         XPD_DMA_BINDTO(HDMA,HANDLE,DMASLOT)         \
    do{(HANDLE)->DMA.DMASLOT = (HDMA);                      \
       (HDMA)->Owner = (HANDLE); }while(0)

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
#define         XPD_DMA_EnableIT( HANDLE,  IT_NAME)         \
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
#define         XPD_DMA_DisableIT(HANDLE,   IT_NAME)        \
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
#define         XPD_DMA_GetFlag(  HANDLE, FLAG_NAME)        \
    (((DMA_BitBand_TypeDef *)(HANDLE)->Base_BB)->LISR.FLAG_NAME##IF0)

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
#define         XPD_DMA_ClearFlag(HANDLE, FLAG_NAME)        \
    (((DMA_BitBand_TypeDef *)(HANDLE)->Base_BB)->LIFCR.C##FLAG_NAME##IF0 = 1)

/** @} */

#ifdef DMA_Stream_BB
#define DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#else
#define DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)
#endif

#define         __XPD_DMA_ITConfig_TC(HANDLE,_VALUE_)         \
    (DMA_REG_BIT((HANDLE),CR,TCIE) = (_VALUE_))
#define         __XPD_DMA_ITConfig_TE(HANDLE,_VALUE_)         \
    (DMA_REG_BIT((HANDLE),CR,TEIE) = (_VALUE_))
#define         __XPD_DMA_ITConfig_HT(HANDLE,_VALUE_)         \
    (DMA_REG_BIT((HANDLE),CR,HTIE) = (_VALUE_))
#define         __XPD_DMA_ITConfig_DME(HANDLE,_VALUE_)        \
    (DMA_REG_BIT((HANDLE),CR,DMEIE) = (_VALUE_))
#define         __XPD_DMA_ITConfig_FE(HANDLE,_VALUE_)         \
    (DMA_REG_BIT((HANDLE),FCR,FEIE) = (_VALUE_))

/** @} */

#endif /* XPD_DMA_H_ */
