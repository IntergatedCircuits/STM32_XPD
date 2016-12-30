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
    DMA_MODE_PERIPH_FC = 2, /*!< DMA transfer with peripheral flow control (completion is indicated by peripheral) */
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

/** @brief DMA stream handle structure */
typedef struct
{
    DMA_Stream_TypeDef * Inst;               /*!< The address of the peripheral instance used by the handle */
#ifdef DMA_Stream_BB
    DMA_Stream_BitBand_TypeDef * Inst_BB;    /*!< The address of the peripheral instance in the bit-band region */
#endif
    DMA_TypeDef * Base;                      /*!< [Internal] The address of the master DMA used by the handle */
    uint8_t StreamOffset;                    /*!< [Internal] The offset of the stream in the master DMA */
    struct {
        XPD_HandleCallbackType Complete;     /*!< DMA transfer complete callback */
        XPD_HandleCallbackType HalfComplete; /*!< DMA transfer half complete callback */
#ifdef USE_XPD_DMA_ERROR_DETECT
        XPD_HandleCallbackType Error;        /*!< DMA transfer error callback */
#endif
    } Callbacks;                             /*   Handle Callbacks */
    void * Owner;                            /*!< [Internal] The pointer of the peripheral handle which uses this handle */
    volatile DMA_ErrorType Errors;           /*!< Transfer errors */
}DMA_HandleType;

/** @} */

/** @defgroup DMA_Exported_Macros DMA Exported Macros
 * @{ */

/**
 * @brief  DMA Handle initializer macro
 * @param  INSTANCE: specifies the DMA stream instance.
 */
#ifdef USE_XPD_DMA_ERROR_DETECT
#define         NEW_DMA_HANDLE(INSTANCE)                    \
    {.Inst      = (INSTANCE),                               \
     .Callbacks = {NULL,NULL,NULL},                         \
     .Owner     = NULL}
#else
#define         NEW_DMA_HANDLE(INSTANCE)                    \
    {.Inst      = (INSTANCE),                               \
     .Callbacks = {NULL,NULL},                              \
     .Owner     = NULL}
#endif

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
    ((HANDLE)->Base->LISR.w & (DMA_LISR_##FLAG_NAME##IF0 << (uint32_t)((HANDLE)->StreamOffset)))

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
    ((HANDLE)->Base->LIFCR.w = (DMA_LIFCR_C##FLAG_NAME##IF0 << (uint32_t)((HANDLE)->StreamOffset)))

#ifdef DMA_Stream_BB
#define DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#else
#define DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)
#endif

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
XPD_ReturnType  XPD_DMA_Init            (DMA_HandleType * hdma, DMA_InitType * Config);
XPD_ReturnType  XPD_DMA_Deinit          (DMA_HandleType * hdma);

void            XPD_DMA_Enable          (DMA_HandleType * hdma);
void            XPD_DMA_Disable         (DMA_HandleType * hdma);

XPD_ReturnType  XPD_DMA_Start           (DMA_HandleType * hdma, void * PeriphAddress,
                                         void * MemAddress, uint16_t DataCount);
XPD_ReturnType  XPD_DMA_Start_IT        (DMA_HandleType * hdma, void * PeriphAddress,
                                         void * MemAddress, uint16_t DataCount);
XPD_ReturnType  XPD_DMA_Stop            (DMA_HandleType * hdma);
void            XPD_DMA_Stop_IT         (DMA_HandleType * hdma);

XPD_ReturnType  XPD_DMA_GetStatus       (DMA_HandleType * hdma);
XPD_ReturnType  XPD_DMA_PollStatus      (DMA_HandleType * hdma, DMA_OperationType Operation, uint32_t Timeout);
DMA_ErrorType   XPD_DMA_GetError        (DMA_HandleType * hdma);

void            XPD_DMA_IRQHandler      (DMA_HandleType * hdma);

uint32_t        XPD_DMA_GetActiveMemory (DMA_HandleType * hdma);
void            XPD_DMA_SetSwapMemory   (DMA_HandleType * hdma, void * Address);
/** @} */

/** @} */

#endif /* XPD_DMA_H_ */
