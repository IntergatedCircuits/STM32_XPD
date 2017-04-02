/**
  ******************************************************************************
  * @file    xpd_spi.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-01-04
  * @brief   STM32 eXtensible Peripheral Drivers SPI Module
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
#ifndef XPD_SPI_H_
#define XPD_SPI_H_

#include "xpd_common.h"
#include "xpd_config.h"
#include "xpd_dma.h"

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
#ifdef USE_XPD_SPI_ERROR_DETECT
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
    XPD_CtrlFnType ClockCtrl;                /*!< Function pointer for RCC clock control */
    struct {
        XPD_HandleCallbackType DepInit;      /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;    /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType Transmit;     /*!< Data stream transmission successful callback */
        XPD_HandleCallbackType Receive;      /*!< Data stream reception successful callback */
#if defined(USE_XPD_SPI_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
        XPD_HandleCallbackType Error;        /*!< Error callbacks */
#endif
    }Callbacks;                              /*   Handle Callbacks */
    struct {
        DMA_HandleType * Transmit;           /*!< DMA handle for data transmission */
        DMA_HandleType * Receive;            /*!< DMA handle for data reception */
    }DMA;                                    /*   DMA handle references */
    DataStreamType RxStream;                 /*!< Data reception stream */
    DataStreamType TxStream;                 /*!< Data transmission stream */
#if defined(USE_XPD_SPI_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
    volatile SPI_ErrorType Errors;           /*!< Transfer errors */
#endif
#ifdef USE_XPD_SPI_ERROR_DETECT
    uint8_t CRCSize;                         /*!< CRC size in bytes */
#endif
}SPI_HandleType;

/** @} */

/** @defgroup SPI_Exported_Macros SPI Exported Macros
 * @{ */

/**
 * @brief  Enable the specified SPI interrupt.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TXE:     Transmit empty
 *            @arg RXNE:    Receive not empty
 *            @arg ERR:     Error
 */
#define         XPD_SPI_EnableIT(HANDLE, IT_NAME)       \
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
#define         XPD_SPI_DisableIT(HANDLE, IT_NAME)      \
    (SPI_REG_BIT((HANDLE),CR2,IT_NAME##IE) = 0)

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
#define         XPD_SPI_GetFlag(HANDLE, FLAG_NAME)      \
    (SPI_REG_BIT((HANDLE),SR,FLAG_NAME))

/**
 * @brief  Clear the specified SPI flag.
 * @param  HANDLE: specifies the SPI Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg CRCERR:  CRC error
 */
#define         XPD_SPI_ClearFlag(HANDLE, FLAG_NAME)    \
    (SPI_REG_BIT((HANDLE),SR,FLAG_NAME) = 0)

#if defined(USE_XPD_SPI_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
/**
 * @brief  SPI Handle initializer macro
 * @param  INSTANCE: specifies the SPI peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_SPI_HANDLE(INSTANCE, INIT_FN, DEINIT_FN)    \
    {.Inst      = (INSTANCE),                                   \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL,NULL},       \
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl,                   \
     .Errors    = SPI_ERROR_NONE}
#else
/**
 * @brief  SPI Handle initializer macro
 * @param  INSTANCE: specifies the SPI peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_SPI_HANDLE(INSTANCE, INIT_FN, DEINIT_FN)    \
    {.Inst      = (INSTANCE),                                   \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL},            \
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl}
#endif

#ifdef SPI_BB
#define SPI_REG_BIT(_HANDLE_, _REG_NAME_, _BIT_NAME_) ((_HANDLE_)->Inst_BB->_REG_NAME_._BIT_NAME_)
#else
#define SPI_REG_BIT(_HANDLE_, _REG_NAME_, _BIT_NAME_) ((_HANDLE_)->Inst->_REG_NAME_.b._BIT_NAME_)
#endif /* SPI_BB */

/** @} */

/** @addtogroup SPI_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_SPI_Init                (SPI_HandleType * hspi, SPI_InitType * Config);
XPD_ReturnType  XPD_SPI_Deinit              (SPI_HandleType * hspi);
void            XPD_SPI_Enable              (SPI_HandleType * hspi);
void            XPD_SPI_Disable             (SPI_HandleType * hspi);

XPD_ReturnType  XPD_SPI_GetStatus           (SPI_HandleType * hspi);
XPD_ReturnType  XPD_SPI_PollStatus          (SPI_HandleType * hspi, uint32_t Timeout);

XPD_ReturnType  XPD_SPI_Transmit            (SPI_HandleType * hspi, void * TxData,
                                             uint16_t Length, uint32_t Timeout);
XPD_ReturnType  XPD_SPI_Receive             (SPI_HandleType * hspi, void * RxData,
                                             uint16_t Length, uint32_t Timeout);
XPD_ReturnType  XPD_SPI_TransmitReceive     (SPI_HandleType * hspi, void * TxData, void * RxData,
                                             uint16_t Length, uint32_t Timeout);

void            XPD_SPI_Transmit_IT         (SPI_HandleType * hspi, void * TxData, uint16_t Length);
void            XPD_SPI_Receive_IT          (SPI_HandleType * hspi, void * RxData, uint16_t Length);
void            XPD_SPI_TransmitReceive_IT  (SPI_HandleType * hspi, void * TxData,
                                             void * RxData, uint16_t Length);

void            XPD_SPI_IRQHandler          (SPI_HandleType * hspi);

XPD_ReturnType  XPD_SPI_Transmit_DMA        (SPI_HandleType * hspi, void * TxData, uint16_t Length);
XPD_ReturnType  XPD_SPI_Receive_DMA         (SPI_HandleType * hspi, void * RxData, uint16_t Length);
XPD_ReturnType  XPD_SPI_TransmitReceive_DMA (SPI_HandleType * hspi, void * TxData,
                                             void * RxData, uint16_t Length);
void            XPD_SPI_Stop_DMA            (SPI_HandleType * hspi);
/** @} */

/** @} */

#define XPD_SPI_API
#include "xpd_rcc_gen.h"
#include "xpd_rcc_pc.h"
#undef XPD_SPI_API

#endif /* XPD_SPI_H_ */
