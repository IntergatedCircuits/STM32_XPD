/**
  ******************************************************************************
  * @file    xpd_spi.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers SPI Module
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
#ifndef __XPD_SPI_H_
#define __XPD_SPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_dma.h>
#include <xpd_rcc.h>

/** @defgroup SPI
 * @{ */

/** @defgroup SPI_Exported_Types SPI Exported Types
 * @{ */

/** @brief SPI operating modes */
typedef enum
{
    SPI_MODE_SLAVE  = 0, /*!< SPI slave mode */
    SPI_MODE_MASTER = 1  /*!< SPI master mode */
}SPI_ModeType;

/** @brief SPI channel configurations */
typedef enum
{
    SPI_CHANNEL_FULL_DUPLEX     = 0,                 /*!< Full duplex channel (simultaneous transmission and reception) */
    SPI_CHANNEL_HALF_DUPLEX     = SPI_CR1_BIDIMODE,  /*!< Half duplex channel (transmission or reception on single data line) */
    SPI_CHANNEL_SIMPLEX_RECEIVE = SPI_CR1_RXONLY     /*!< Receive only channel */
}SPI_ChannelType;

/** @brief SPI slave select configurations */
typedef enum
{
    SPI_NSS_SOFT        = SPI_CR1_SSM,                          /*!< NSS is disconnected from the pin */
    SPI_NSS_HARD_INPUT  = 0,                                    /*!< NSS is slave chip select or puts master in slave mode */
    SPI_NSS_HARD_OUTPUT = SPI_CR2_SSOE,                         /*!< Single master node, NSS is disconnected from the pin */
#ifdef SPI_CR2_NSSP
    SPI_NSS_HARD_OUTPUT_PULSED = (SPI_CR2_SSOE | SPI_CR2_NSSP), /*!< Single master node, NSS is disconnected from the pin */
#endif
}SPI_NSSType;

/** @brief SPI bit order */
typedef enum
{
    SPI_FORMAT_MSB_FIRST = 0, /*!< Most significant bit is transferred first */
    SPI_FORMAT_LSB_FIRST = 1  /*!< Least significant bit is transferred first */
}SPI_FormatType;

/** @brief SPI setup structure */
typedef struct
{
    SPI_ModeType    Mode;           /*!< Specifies the SPI operating mode. */
    SPI_ChannelType Channel;        /*!< Specifies the SPI channel type. */
    uint8_t         DataSize;       /*!< Specifies the SPI data size in bits. */
    SPI_FormatType  Format;         /*!< Specifies whether data transfers start from MSB or LSB bit. */
    FunctionalState TI_Mode;        /*!< Specifies if the TI mode is enabled or not. */
    SPI_NSSType     NSS;            /*!< Specifies whether the NSS signal is managed by hardware (NSS pin) or by software using the SSI bit. */
    struct {
        ActiveLevelType  Polarity;  /*!< Specifies the serial clock steady state. */
        ClockPhaseType   Phase;     /*!< Specifies the clock active edge for the bit capture. */
        ClockDividerType Prescaler; /*!< Specifies the Baud Rate prescaler value which will be
                                         used to configure the transmit and receive SCK clock.Permitted values:
                                             @arg @ref ClockDividerType::CLK_DIV2
                                             @arg @ref ClockDividerType::CLK_DIV4
                                             @arg @ref ClockDividerType::CLK_DIV8
                                             @arg @ref ClockDividerType::CLK_DIV16
                                             @arg @ref ClockDividerType::CLK_DIV32
                                             @arg @ref ClockDividerType::CLK_DIV64
                                             @arg @ref ClockDividerType::CLK_DIV128
                                             @arg @ref ClockDividerType::CLK_DIV256 */
    } Clock;
#ifdef __XPD_SPI_ERROR_DETECT
    uint8_t  CRC_Length;            /*!< Specifies the CRC length in bits. Permitted values: @arg 8, 16 */
    uint16_t CRC_Polynomial;        /*!< Specifies the CRC polynomial. */
#endif
} SPI_InitType;

/** @brief SPI error types */
typedef enum
{
    SPI_ERROR_NONE      = 0,   /*!< No error */
    SPI_ERROR_MODE      = 1,   /*!< Mode fault */
    SPI_ERROR_CRC       = 2,   /*!< CRC error flag */
    SPI_ERROR_OVERRUN   = 4,   /*!< Overrun flag */
    SPI_ERROR_FRAME     = 8,   /*!< Frame format error */
    SPI_ERROR_DMA       = 16,  /*!< DMA transfer error */
}SPI_ErrorType;

/** @brief SPI Handle structure */
typedef struct
{
    SPI_TypeDef * Inst;                      /*!< The address of the peripheral instance used by the handle */
#ifdef SPI_BB
    SPI_BitBand_TypeDef * Inst_BB;           /*!< The address of the peripheral instance in the bit-band region */
#endif
    struct {
        XPD_HandleCallbackType DepInit;      /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;    /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType Transmit;     /*!< Data stream transmission successful callback */
        XPD_HandleCallbackType Receive;      /*!< Data stream reception successful callback */
#if defined(__XPD_SPI_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
        XPD_HandleCallbackType Error;        /*!< Error callbacks */
#endif
    }Callbacks;                              /*   Handle Callbacks */
    struct {
        DMA_HandleType * Transmit;           /*!< DMA handle for data transmission */
        DMA_HandleType * Receive;            /*!< DMA handle for data reception */
    }DMA;                                    /*   DMA handle references */
    DataStreamType RxStream;                 /*!< Data reception stream */
    DataStreamType TxStream;                 /*!< Data transmission stream */
    RCC_PositionType CtrlPos;                /*!< Relative position for reset and clock control */
#if defined(__XPD_SPI_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
    volatile SPI_ErrorType Errors;           /*!< Transfer errors */
#endif
#ifdef __XPD_SPI_ERROR_DETECT
    uint8_t CRCSize;                         /*!< CRC size in bytes */
#endif
}SPI_HandleType;

/** @} */

/** @defgroup SPI_Exported_Macros SPI Exported Macros
 * @{ */

#ifdef SPI_BB
/**
 * @brief SPI Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the SPI peripheral instance.
 */
#define         SPI_INST2HANDLE(HANDLE,INSTANCE)            \
    ((HANDLE)->Inst    = (INSTANCE),                        \
     (HANDLE)->Inst_BB = SPI_BB(INSTANCE),                  \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief SPI register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         SPI_REG_BIT(_HANDLE_, REG_NAME, BIT_NAME)   \
    ((_HANDLE_)->Inst_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief SPI Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the SPI peripheral instance.
 */
#define         SPI_INST2HANDLE(HANDLE,INSTANCE)            \
    ((HANDLE)->Inst    = (INSTANCE),                        \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief SPI register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         SPI_REG_BIT(_HANDLE_, REG_NAME, BIT_NAME)   \
    ((_HANDLE_)->Inst->REG_NAME.b.BIT_NAME)

#endif /* SPI_BB */

/**
 * @brief  Enable the specified SPI interrupt.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TXE:     Transmit empty
 *            @arg RXNE:    Receive not empty
 *            @arg ERR:     Error
 */
#define         SPI_IT_ENABLE(HANDLE, IT_NAME)               \
    (SPI_REG_BIT((HANDLE),CR2,IT_NAME##IE) = 1)

/**
 * @brief  Disable the specified SPI interrupt.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TXE:     Transmit empty
 *            @arg RXNE:    Receive not empty
 *            @arg ERR:     Error
 */
#define         SPI_IT_DISABLE(HANDLE, IT_NAME)             \
    (SPI_REG_BIT((HANDLE),CR2,IT_NAME##IE) = 0)

/**
 * @brief  Resume the specified SPI DMA requests.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  DMA_NAME: specifies the DMA request to resume.
 *         This parameter can be one of the following values:
 *            @arg T:       Transmit
 *            @arg R:       Receive
 */
#define         SPI_DMA_ENABLE(HANDLE, DMA_NAME)            \
    (SPI_REG_BIT((HANDLE), CR2, DMA_NAME##XDMAEN) = 1)

/**
 * @brief  Halt the specified SPI DMA requests.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  DMA_NAME: specifies the DMA request to halt.
 *         This parameter can be one of the following values:
 *            @arg T:       Transmit
 *            @arg R:       Receive
 */
#define         SPI_DMA_DISABLE(HANDLE, DMA_NAME)           \
    (SPI_REG_BIT((HANDLE), CR2, DMA_NAME##XDMAEN) = 0)

/**
 * @brief  Get the specified SPI flag.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg TXE:     Transmit empty
 *            @arg RXNE:    Receive not empty
 *            @arg UDR:     Underrun
 *            @arg CRCERR:  CRC error
 *            @arg MODF:    Mode fault
 *            @arg OVR:     Overrun
 *            @arg BSY:     Busy
 *            @arg FRE:     TI frame format error
 */
#define         SPI_FLAG_STATUS(HANDLE, FLAG_NAME)              \
    (SPI_REG_BIT((HANDLE),SR,FLAG_NAME))

/**
 * @brief  Clear the specified SPI flag.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg CRCERR:  CRC error
 */
#define         SPI_FLAG_CLEAR(HANDLE, FLAG_NAME)            \
    (SPI_REG_BIT((HANDLE),SR,FLAG_NAME) = 0)

/** @} */

/** @addtogroup SPI_Exported_Functions
 * @{ */
void            SPI_vInit               (SPI_HandleType * pxSPI,
                                         const SPI_InitType * pxConfig);

void            SPI_vDeinit             (SPI_HandleType * pxSPI);

XPD_ReturnType  SPI_eGetStatus          (SPI_HandleType * pxSPI);
XPD_ReturnType  SPI_ePollStatus         (SPI_HandleType * pxSPI,
                                         uint32_t ulTimeout);


XPD_ReturnType  SPI_eSend               (SPI_HandleType * pxSPI,
                                         void * pvTxData,
                                         uint16_t usLength,
                                         uint32_t ulTimeout);

XPD_ReturnType  SPI_eReceive            (SPI_HandleType * pxSPI,
                                         void * pvRxData,
                                         uint16_t usLength,
                                         uint32_t ulTimeout);

XPD_ReturnType  SPI_eSendReceive        (SPI_HandleType * pxSPI,
                                         void * pvTxData,
                                         void * pvRxData,
                                         uint16_t usLength,
                                         uint32_t ulTimeout);


void            SPI_vTransmit_IT        (SPI_HandleType * pxSPI,
                                         void * pvTxData,
                                         uint16_t usLength);

void            SPI_vReceive_IT         (SPI_HandleType * pxSPI,
                                         void * pvRxData,
                                         uint16_t usLength);

void            SPI_vTransmitReceive_IT (SPI_HandleType * pxSPI,
                                         void * pvTxData,
                                         void * pvRxData,
                                         uint16_t usLength);

void            SPI_vIRQHandler         (SPI_HandleType * pxSPI);


XPD_ReturnType  SPI_eTransmit_DMA       (SPI_HandleType * pxSPI,
                                         void * pvTxData,
                                         uint16_t usLength);

XPD_ReturnType  SPI_eReceive_DMA        (SPI_HandleType * pxSPI,
                                         void * pvRxData,
                                         uint16_t usLength);

XPD_ReturnType  SPI_eSendReceive_DMA    (SPI_HandleType * pxSPI,
                                         void * pvTxData,
                                         void * pvRxData,
                                         uint16_t usLength);

void            SPI_vStop_DMA           (SPI_HandleType * pxSPI);
/** @} */

/** @} */

#define XPD_SPI_API
#include <xpd_rcc_pc.h>
#undef XPD_SPI_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_SPI_H_ */
