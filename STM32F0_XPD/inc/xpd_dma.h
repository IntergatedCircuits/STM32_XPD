/**
  ******************************************************************************
  * @file    xpd_dma.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-11-01
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
typedef struct
{
    DMA_DirectionType Direction;         /*!< DMA channel direction */
    DMA_ModeType      Mode;              /*!< DMA operating mode */
    LevelType         Priority;          /*!< DMA bus arbitration priority level */
    struct {
        FunctionalState   Increment;     /*!< The address is incremented after each transfer */
        DMA_AlignmentType DataAlignment; /*!< The data width */
    }Memory;                             /*  Memory side configuration */
    struct {
        FunctionalState   Increment;     /*!< The address is incremented after each transfer */
        DMA_AlignmentType DataAlignment; /*!< The data width */
    }Peripheral;                         /*   Peripheral side configuration */
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
#ifdef USE_XPD_DMA_ERROR_DETECT
        XPD_HandleCallbackType Error;         /*!< DMA transfer error callback */
#endif
    } Callbacks;                              /*   Handle Callbacks */
    void * Owner;                             /*!< [Internal] The pointer of the peripheral handle which uses this handle */
    volatile DMA_ErrorType Errors;            /*!< Transfer errors */
}DMA_HandleType;

/** @} */

/** @defgroup DMA_Exported_Macros DMA Exported Macros
 * @{ */

#ifdef USE_XPD_DMA_ERROR_DETECT
/**
 * @brief  DMA Handle initializer macro
 * @param  INSTANCE: specifies the DMA stream instance.
 */
#define         NEW_DMA_HANDLE(INSTANCE)                    \
    {.Inst      = (INSTANCE),                               \
     .Callbacks = {NULL,NULL,NULL},                         \
     .Owner     = NULL}
#else
/**
 * @brief  DMA Handle initializer macro
 * @param  INSTANCE: specifies the DMA stream instance.
 */
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
 */
#define         XPD_DMA_EnableIT( HANDLE,  IT_NAME)         \
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
#define         XPD_DMA_DisableIT(HANDLE,   IT_NAME)        \
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
#define         XPD_DMA_GetFlag(  HANDLE, FLAG_NAME)        \
    (((HANDLE)->Base->ISR.w >> (DMA_ISR_##FLAG_NAME##IF1_Pos + (uint32_t)((HANDLE)->ChannelOffset))) & 1)

/**
 * @brief  Clear the specified DMA flag.
 * @param  HANDLE: specifies the DMA Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg TC:      Transfer Complete
 *            @arg HT:      Half Transfer Complete
 *            @arg TE:      Transfer Error
 */
#define         XPD_DMA_ClearFlag(HANDLE, FLAG_NAME)        \
    ((HANDLE)->Base->IFCR.w = (DMA_IFCR_C##FLAG_NAME##IF1 << (uint32_t)((HANDLE)->ChannelOffset)))

/**
 * @brief  Provides the circular mode of DMA stream.
 * @param  HANDLE: specifies the DMA Handle.
 */
#define         XPD_DMA_CircularMode(HANDLE)                \
        (DMA_REG_BIT((HANDLE), CCR, CIRC))

#ifdef DMA_BB
#define DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#else
#define DMA_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)
#endif /* DMA_BB */

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
#define XPD_DMA_ChannelRemap(BASE, CHANNEL, SELECTION)           \
    (MODIFY_REG(BASE->CSELR.w, 0xF << ((uint32_t)((CHANNEL) - 1) * 4), BASE##_CSELR_CH##CHANNEL##_##SELECTION))
#endif /* DMA_CSELR_C1S */

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
/** @} */

/** @} */

#define XPD_DMA_API
#include "xpd_syscfg.h"
#undef XPD_DMA_API

#endif /* XPD_DMA_H_ */
