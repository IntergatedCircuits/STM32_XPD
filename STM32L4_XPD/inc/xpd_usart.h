/**
  ******************************************************************************
  * @file    xpd_usart.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-03-27
  * @brief   STM32 eXtensible Peripheral Drivers USART Module
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
#ifndef XPD_USART_H_
#define XPD_USART_H_

#include "xpd_common.h"
#include "xpd_config.h"
#include "xpd_dma.h"
/** @defgroup USART
 * @{ */

/** @defgroup USART_Common USART Common
 * @{ */

/* Determine which version of the peripheral is available */
#if   defined(USART_ISR_WUF)
#define USART_PERIPHERAL_VERSION    3
#elif defined(USART_ISR_BUSY)
#define USART_PERIPHERAL_VERSION    2
#else
#define USART_PERIPHERAL_VERSION    1
#endif

/** @defgroup USART_Common_Exported_Types USART Common Exported Types
 * @{ */

/** @brief USART stopbit selection */
typedef enum
{
    USART_STOPBITS_0p5 = 1, /*!< STOP character is 0.5 bit long */
    USART_STOPBITS_1   = 0, /*!< STOP character is 1 bit long */
    USART_STOPBITS_1p5 = 3, /*!< STOP character is 1.5 bit long */
    USART_STOPBITS_2   = 2  /*!< STOP character is 2 bit long */
}USART_StopBitsType;

/** @brief USART parity selection */
typedef enum
{
    USART_PARITY_NONE = 0, /*!< Parity bit is not used */
    USART_PARITY_EVEN = 2, /*!< Parity bit addition turns the sum of bits even */
    USART_PARITY_ODD  = 3  /*!< Parity bit addition turns the sum of bits odd */
}USART_ParityType;

/** @brief USART setup structure */
typedef struct
{
    uint32_t              BaudRate;      /*!< The USART communication baud rate */
    FunctionalState       Transmitter;   /*!< Sets the module's transmitter capacity */
    FunctionalState       Receiver;      /*!< Sets the module's receiver capacity */
    uint8_t               DataSize;      /*!< Specifies the USART frame data size in bits */
    USART_StopBitsType    StopBits;      /*!< The number of stop bits transmitted */
    FunctionalState       SingleSample;  /*!< Sets the sampling to 1/bit from default 3/bit */

#ifdef USE_XPD_USART_ERROR_DETECT
    USART_ParityType      Parity;        /*!< Specifies if the last frame bit is the parity and how it is calculated */
    FunctionalState       HaltRxOnError; /*!< Specifies whether reception should continue or stop after an error */
#endif
}USART_InitType;

#if (USART_PERIPHERAL_VERSION > 1)
/** @brief USART inversion selection */
typedef enum
{
    USART_INVERT_NONE           = 0,                  /*!< No logical signals are inverted */
    USART_INVERT_TX             = USART_CR2_TXINV,    /*!< TX signal inversion is selected */
    USART_INVERT_RX             = USART_CR2_RXINV,    /*!< RX signal inversion is selected */
    USART_INVERT_DATA           = USART_CR2_DATAINV,  /*!< Data inversion is selected */
    USART_INVERT_BIT_ORDER      = USART_CR2_MSBFIRST, /*!< Bit order inversion is selected (MSB first) */
    USART_INVERT_PIN_DIRECTIONS = USART_CR2_SWAP,     /*!< TX and RX pins are swapped */
}USART_InversionType;
#endif

/** @brief USART error types */
typedef enum
{
    USART_ERROR_NONE    = 0, /*!< No error */
    USART_ERROR_PARITY  = 1, /*!< Parity error */
    USART_ERROR_NOISE   = 2, /*!< Noise error */
    USART_ERROR_FRAME   = 4, /*!< Frame format error */
    USART_ERROR_OVERRUN = 8, /*!< Overrun flag */
    USART_ERROR_DMA     = 16 /*!< DMA transfer error */
}USART_ErrorType;

/** @brief USART Handle structure */
typedef struct
{
    USART_TypeDef * Inst;                    /*!< The address of the peripheral instance used by the handle */
#ifdef USART_BB
    USART_BitBand_TypeDef * Inst_BB;         /*!< The address of the peripheral instance in the bit-band region */
#endif
    XPD_CtrlFnType ClockCtrl;                /*!< Function pointer for RCC clock control */
    struct {
        XPD_HandleCallbackType DepInit;      /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;    /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType Transmit;     /*!< Data stream transmission successful callback */
        XPD_HandleCallbackType Receive;      /*!< Data stream reception successful callback */
        XPD_HandleCallbackType Break;        /*!< LIN break detection callback */
        XPD_HandleCallbackType Idle;         /*!< Idle callback */
        XPD_HandleCallbackType ClearToSend;  /*!< Clear To Send callback */
#if defined(USE_XPD_USART_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
        XPD_HandleCallbackType Error;        /*!< Error callbacks */
#endif
    }Callbacks;                              /*   Handle Callbacks */
    struct {
        DMA_HandleType * Transmit;           /*!< DMA handle for data transmission */
        DMA_HandleType * Receive;            /*!< DMA handle for data reception */
    }DMA;                                    /*   DMA handle references */
    DataStreamType RxStream;                 /*!< Data reception stream */
    DataStreamType TxStream;                 /*!< Data transmission stream */
#if defined(USE_XPD_USART_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
    volatile USART_ErrorType Errors;         /*!< Transfer errors */
#endif
#ifdef USE_XPD_USART_ERROR_DETECT
    boolean_t haltOnError;                   /*!< [Internal] Halt on error setting */
#endif
}USART_HandleType;

/** @} */

/** @defgroup USART_Common_Exported_Macros USART Common Exported Macros
 * @{ */

/**
 * @brief  Enable the specified USART interrupt.
 * @param  HANDLE: specifies the USART Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg IDLE:    Idle
 *            @arg RXNE:    Receive buffer not empty
 *            @arg TC:      Transmission complete
 *            @arg TXE:     Transmit buffer empty
 *            @arg PE:      Parity error
 *            @arg E:       Noise, frame, overrun error
 *            @arg LBD:     LIN break detection
 *            @arg CTS:     Clear To Send
 *            @arg WU:      Wake Up
 */
#define         XPD_USART_EnableIT(  HANDLE,  IT_NAME)      \
    (__XPD_USART_##IT_NAME##IECtrl(HANDLE, ENABLE))

/**
 * @brief  Disable the specified USART interrupt.
 * @param  HANDLE: specifies the USART Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg IDLE:    Idle
 *            @arg RXNE:    Receive buffer not empty
 *            @arg TC:      Transmission complete
 *            @arg TXE:     Transmit buffer empty
 *            @arg PE:      Parity error
 *            @arg E:       Noise, frame, overrun error
 *            @arg LBD:     LIN break detection
 *            @arg CTS:     Clear To Send
 *            @arg WU:      Wake Up
 */
#define         XPD_USART_DisableIT( HANDLE,  IT_NAME)      \
        (__XPD_USART_##IT_NAME##IECtrl(HANDLE, DISABLE))

#if (USART_PERIPHERAL_VERSION > 1)
/* macros for cross-compatibility */
#define USART_ICR_NECF      USART_ICR_NCF
#define USART_ISR_WU        USART_ISR_WUF
#define USART_ISR_WU_Pos    USART_ISR_WUF_Pos
#define USART_ISR_LBD       USART_ISR_LBDF
#define USART_ISR_LBD_Pos   USART_ISR_LBDF_Pos
#define USART_ICR_RXNECF    0

/**
 * @brief  Get the specified USART flag.
 * @param  HANDLE: specifies the USART Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg REACK:   Receive enable acknowledge
 *            @arg TEACK:   Transmit enable acknowledge
 *            @arg BUSY:    Busy
 *            @arg IDLE:    Idle
 *            @arg RXNE:    Receive buffer not empty
 *            @arg TC:      Transmission complete
 *            @arg TXE:     Transmit buffer empty
 *            @arg PE:      Parity error
 *            @arg FE:      Framing error
 *            @arg NE:      Noise error
 *            @arg ORE:     Overrun error
 *            @arg LBD:     LIN break detection
 *            @arg CTS:     Clear To Send
 *            @arg WU:      Wake Up
 */
#define         XPD_USART_GetFlag(  HANDLE, FLAG_NAME)      \
    (((HANDLE)->Inst->ISR.w >> USART_ISR_##FLAG_NAME##_Pos) & 1)

/**
 * @brief  Clear the specified USART flag.
 * @param  HANDLE: specifies the USART Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg IDLE:    Idle
 *            @arg RXNE:    Receive buffer not empty
 *            @arg TC:      Transmission complete
 *            @arg PE:      Parity error
 *            @arg FE:      Framing error
 *            @arg NE:      Noise error
 *            @arg ORE:     Overrun error
 *            @arg LBD:     LIN break detection
 *            @arg CTS:     Clear To Send
 *            @arg WU:      Wake Up
 */
#define         XPD_USART_ClearFlag(HANDLE, FLAG_NAME)      \
    ((USART_ISR_##FLAG_NAME != USART_ISR_RXNE) ?            \
    ((HANDLE)->Inst->ICR.w = USART_ICR_##FLAG_NAME##CF) :   \
    ((HANDLE)->Inst->RQR.w = USART_RQR_RXFRQ))
#else

/**
 * @brief  Get the specified USART flag.
 * @param  HANDLE: specifies the USART Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg IDLE:    Idle
 *            @arg RXNE:    Receive buffer not empty
 *            @arg TC:      Transmission complete
 *            @arg TXE:     Transmit buffer empty
 *            @arg PE:      Parity error
 *            @arg FE:      Framing error
 *            @arg NE:      Noise error
 *            @arg ORE:     Overrun error
 *            @arg LBD:     LIN break detection
 *            @arg CTS:     Clear To Send
 */
#define         XPD_USART_GetFlag(  HANDLE, FLAG_NAME)      \
    (USART_REG_BIT((HANDLE),SR,FLAG_NAME))

/**
 * @brief  Clear the specified USART flag.
 * @param  HANDLE: specifies the USART Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg IDLE:    Idle
 *            @arg RXNE:    Receive buffer not empty
 *            @arg TC:      Transmission complete
 *            @arg TXE:     Transmit buffer empty
 *            @arg PE:      Parity error
 *            @arg FE:      Framing error
 *            @arg NE:      Noise error
 *            @arg ORE:     Overrun error
 *            @arg LBD:     LIN break detection
 *            @arg CTS:     Clear To Send
 */
#define         XPD_USART_ClearFlag(HANDLE, FLAG_NAME)                              \
    do { if(USART_SR_##FLAG_NAME##_Pos <= USART_SR_RXNE_Pos)                        \
       { volatile uint32_t _x_ = (HANDLE)->Inst->SR.w; _x_ = (HANDLE)->Inst->DR; }  \
       else {(USART_REG_BIT((HANDLE),SR,FLAG_NAME) = 0);} } while(0)

/* Compatibility macros */
#define         XPD_USART_InversionConfig(HANDLE, INVERSIONS)       ((void)0)
#define         XPD_USART_OverrunConfig(HANDLE, MODE)               ((void)0)
#define         XPD_UART_BaudrateModeConfig(HANDLE, MODE)           ((void)0)

#endif /* (USART_PERIPHERAL_VERSION > 1) */

#if defined(USE_XPD_USART_ERROR_DETECT) || defined(USE_XPD_DMA_ERROR_DETECT)
/**
 * @brief  USART Handle initializer macro
 * @param  INSTANCE: specifies the SPI peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_USART_HANDLE(INSTANCE, INIT_FN, DEINIT_FN)          \
    {.Inst      = (INSTANCE),                                           \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL,NULL,NULL,NULL,NULL},\
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl,                           \
     .Errors    = USART_ERROR_NONE}
#else
/**
 * @brief  USART Handle initializer macro
 * @param  INSTANCE: specifies the SPI peripheral instance.
 * @param  INIT_FN: specifies the dependency initialization function to call back.
 * @param  DEINIT_FN: specifies the dependency deinitialization function to call back.
 */
#define         NEW_USART_HANDLE(INSTANCE, INIT_FN, DEINIT_FN)          \
    {.Inst      = (INSTANCE),                                           \
     .Callbacks = {(INIT_FN),(DEINIT_FN),NULL,NULL,NULL,NULL,NULL},     \
     .ClockCtrl = XPD_##INSTANCE##_ClockCtrl}
#endif

#ifdef USART_BB
#define USART_REG_BIT(_HANDLE_, _REG_NAME_, _BIT_NAME_) ((_HANDLE_)->Inst_BB->_REG_NAME_._BIT_NAME_)
#else
#define USART_REG_BIT(_HANDLE_, _REG_NAME_, _BIT_NAME_) ((_HANDLE_)->Inst->_REG_NAME_.b._BIT_NAME_)
#endif /* USART_BB */

#define __XPD_USART_IDLEIECtrl(     HANDLE, NEWSTATE)   \
    (USART_REG_BIT((HANDLE),CR1,IDLEIE) = NEWSTATE)

#define __XPD_USART_RXNEIECtrl(HANDLE, NEWSTATE)        \
    (USART_REG_BIT((HANDLE),CR1,RXNEIE) = NEWSTATE)

#define __XPD_USART_TCIECtrl(HANDLE, NEWSTATE)          \
    (USART_REG_BIT((HANDLE),CR1,TCIE) = NEWSTATE)

#define __XPD_USART_TXEIECtrl(HANDLE, NEWSTATE)         \
    (USART_REG_BIT((HANDLE),CR1,TXEIE) = NEWSTATE)

#define __XPD_USART_PEIECtrl(HANDLE, NEWSTATE)          \
    (USART_REG_BIT((HANDLE),CR1,PEIE) = NEWSTATE)

#define __XPD_USART_LBDIECtrl(HANDLE, NEWSTATE)         \
    (USART_REG_BIT((HANDLE),CR2,LBDIE) = NEWSTATE)

#define __XPD_USART_EIECtrl(HANDLE, NEWSTATE)           \
    (USART_REG_BIT((HANDLE),CR3,EIE) = NEWSTATE)

#define __XPD_USART_CTSIECtrl(HANDLE, NEWSTATE)         \
    (USART_REG_BIT((HANDLE),CR3,CTSIE) = NEWSTATE)

#define __XPD_USART_WUIECtrl(HANDLE, NEWSTATE)         \
    (USART_REG_BIT((HANDLE),CR3,WUFIE) = NEWSTATE)

/** @} */

/** @addtogroup USART_Common_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_USART_Deinit            (USART_HandleType * husart);

void            XPD_USART_Enable            (USART_HandleType * husart);
void            XPD_USART_Disable           (USART_HandleType * husart);

XPD_ReturnType  XPD_USART_Transmit          (USART_HandleType * husart, void * TxData,
                                             uint16_t Length, uint32_t Timeout);
XPD_ReturnType  XPD_USART_Receive           (USART_HandleType * husart, void * RxData,
                                             uint16_t Length, uint32_t Timeout);
XPD_ReturnType  XPD_USART_TransmitReceive   (USART_HandleType * husart, void * TxData, void * RxData,
                                             uint16_t Length, uint32_t Timeout);

void            XPD_USART_Transmit_IT       (USART_HandleType * husart, void * TxData, uint16_t Length);
void            XPD_USART_Receive_IT        (USART_HandleType * husart, void * RxData, uint16_t Length);

void            XPD_USART_IRQHandler        (USART_HandleType * husart);

XPD_ReturnType  XPD_USART_Transmit_DMA      (USART_HandleType * husart, void * TxData, uint16_t Length);
XPD_ReturnType  XPD_USART_Receive_DMA       (USART_HandleType * husart, void * RxData, uint16_t Length);
XPD_ReturnType  XPD_USART_TransmitReceive_DMA(USART_HandleType* husart, void * TxData,
                                             void * RxData, uint16_t Length);
void            XPD_USART_Stop_DMA          (USART_HandleType * husart);

#if (USART_PERIPHERAL_VERSION > 1)
void            XPD_USART_InversionConfig   (USART_HandleType * husart, USART_InversionType Inversions);
void            XPD_USART_OverrunConfig     (USART_HandleType * husart, FunctionalState Mode);
#endif
/** @} */

/** @} */

/** @defgroup UART Universal Asynchronous Receiver-Transmitter
 * @{ */

/** @defgroup UART_Exported_Types UART Exported Types
 * @{ */

/** @brief UART flow control selection */
typedef enum
{
    UART_FLOWCONTROL_NONE    = 0, /*!< No flow control lines are used */
    UART_FLOWCONTROL_RTS     = 1, /*!< Request To Send signal is used */
    UART_FLOWCONTROL_CTS     = 2, /*!< Clear To Send signal is used */
    UART_FLOWCONTROL_RTS_CTS = 3  /*!< Request To Send and Clear To Send signals are used */
}UART_FlowControlType;

/** @brief UART setup structure */
typedef struct {
    UART_FlowControlType FlowControl;   /*!< Hardware flow control select */
    FunctionalState      OverSampling8; /*!< When the over sampling 8 is enabled (instead of 16),
                                             higher baudrate is available */
    FunctionalState      HalfDuplex;    /*!< Half-duplex communication select */
}UART_InitType;

#if (USART_PERIPHERAL_VERSION > 1)
/** @brief UART baudrate detection strategy */
typedef enum
{
    UART_BAUDRATE_FIXED = 0,            /*!< Fixed baudrate is used (auto baudrate detection is disabled) */

#ifdef USART_CR2_ABREN

    UART_BAUDRATE_AUTO_STARTBIT_MODE =  /*!< START bit is measured from falling to rising edge (D0=1) */
            USART_CR2_ABREN,
    UART_BAUDRATE_AUTO_2FALLING_MODE =  /*!< START + LSB=1 bits are measured from falling to falling edge (D0=1, D1=0) */
            USART_CR2_ABREN | USART_CR2_ABRMODE_0,

#ifdef USART_CR2_ABRMODE_1

    UART_BAUDRATE_AUTO_0X7FFRAME_MODE = /*!< START to MSB=0 bits are measured from falling to falling edge */
            USART_CR2_ABREN | USART_CR2_ABRMODE_1,
    UART_BAUDRATE_AUTO_0X55FRAME_MODE = /*!< Combination of the above methods */
            USART_CR2_ABREN | USART_CR2_ABRMODE
#endif
#endif
}UART_BaudrateModeType;
#endif

/** @} */

/** @addtogroup UART_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_UART_Init               (USART_HandleType * husart, const USART_InitType * Common,
                                             const UART_InitType * Config);
#if (USART_PERIPHERAL_VERSION > 1)
void            XPD_UART_BaudrateModeConfig (USART_HandleType * husart, UART_BaudrateModeType Mode);
#endif
/** @} */

/** @} */

/** @defgroup USRT Universal Synchronous Receiver-Transmitter
 * @{ */

/** @defgroup USRT_Exported_Types USRT Exported Types
 * @{ */

/** @brief USRT setup structure */
typedef struct {
    ActiveLevelType   Polarity;      /*!< Steady state of the serial clock */
    ClockPhaseType    Phase;         /*!< The clock transition on which the bit capture is made */
    FunctionalState   LastBit;       /*!< The clock pulse for the last transmitted data bit (MSB)
                                          has to be provided */
}USRT_InitType;

/** @} */

/** @addtogroup USRT_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_USRT_Init               (USART_HandleType * husart, const USART_InitType * Common,
                                             const USRT_InitType * Config);
/** @} */

/** @} */

/** @defgroup LIN Local Interconnect Network
 * @{ */

/** @addtogroup LIN_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_LIN_Init                (USART_HandleType * husart, const USART_InitType * Common,
                                             uint8_t BreakSize);
void            XPD_LIN_SendBreak           (USART_HandleType * husart);
/** @} */

/** @} */

/** @defgroup MSUART MultiSlave UART Bus
 * @{ */

/** @defgroup MSUART_Exported_Types MultiSlave UART Exported Types
 * @{ */

/** @brief MSUART wakeup from Mute selection */
typedef enum
{
    MSUART_UNMUTE_IDLE_LINE     = 0, /*!< Unmute on Idle Line */
    MSUART_UNMUTE_ADDRESSED     = 1, /*!< Unmute on matching Address Mark (address MSB = 1, data MSB = 0) */
}MSUART_UnmuteType;

#if (USART_PERIPHERAL_VERSION > 2)
/** @brief MSUART wakeup selection */
typedef enum
{
    MSUART_WAKEUP_ADDRESSED     = 1, /*!< Wakeup on matching Address Mark (address MSB = 1, data MSB = 0) */
    MSUART_WAKEUP_START_BIT     = 2, /*!< Wakeup on Start bit detection */
    MSUART_WAKEUP_DATA_RECEIVED = 3, /*!< Wakeup on RXNE flag */
}MSUART_WakeUpType;
#endif

/** @brief MSUART setup structure */
typedef struct {
    FunctionalState      OverSampling8;     /*!< When the over sampling 8 is enabled (instead of 16),
                                                 higher baudrate is available */
    FunctionalState      HalfDuplex;        /*!< Half-duplex communication select */
    uint8_t              Address;           /*!< Specifies the slave device address */
    uint8_t              AddressLength;     /*!< Specifies the address length (default 4 or 7 bits) */
    MSUART_UnmuteType    UnmuteMethod;      /*!< Specifies the activation strategy from Mute */

#if (USART_PERIPHERAL_VERSION > 2)
    MSUART_WakeUpType    WakeUpMethod;      /*!< Specifies the activation strategy from STOP mode */
#endif
}MSUART_InitType;
/** @} */

/** @addtogroup MSUART_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_MSUART_Init             (USART_HandleType * husart, const USART_InitType * Common,
                                             const MSUART_InitType * Config);
void            XPD_MSUART_MuteCtrl         (USART_HandleType * husart, FunctionalState NewState);

#if (USART_PERIPHERAL_VERSION > 2)
void            XPD_MSUART_StopModeCtrl     (USART_HandleType * husart, FunctionalState NewState);
#endif
/** @} */

/** @} */

/** @defgroup SmartCard SmartCard interface
 * @{ */

/** @defgroup SmartCard_Exported_Types SmartCard Exported Types
 * @{ */

/** @brief SmartCard setup structure */
typedef struct
{
    uint8_t         GuardTime;    /*!< Delays the raising of TC flag by a number of baud clocks */
    FunctionalState NACK;         /*!< Enable to force frame error at the end of received frame
                                       with detected parity error */
    struct {
        ActiveLevelType Polarity; /*!< Steady state of the serial clock */
        ClockPhaseType  Phase;    /*!< The clock transition on which the bit capture is made */
        uint8_t         Divider;  /*!< Prescaler value that divides the peripheral clock
                                       before SCLK output [2, 4, ... 62] */
    }Clock;                       /*   Output clock settings */

#if (USART_PERIPHERAL_VERSION > 1)
    uint32_t        RxTimeout;    /*!< Specifies the receiver timeout value in baud block count for
                                       Character Wait Time (CWT) and Block Wait Time (BWT). Set to 0 to disable. */
    uint8_t         BlockLength;  /*!< Specifies the SmartCard Block Length in T = 1 Reception mode */
    uint8_t         AutoRetries;  /*!< Specifies the number of retries in receive and transmit mode
                                       before reaching error state [0..7] */
#endif
}SmartCard_InitType;
/** @} */

/** @addtogroup SmartCard_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_SmartCard_Init          (USART_HandleType * husart, const USART_InitType * Common,
                                             const SmartCard_InitType * Config);
/** @} */

/** @} */

/** @defgroup IrDA IrDA interface
 * @{ */

/** @defgroup IrDA_Exported_Types IrDA Exported Types
 * @{ */

/** @brief IrDA setup structure */
typedef struct
{
    uint8_t         Prescaler;    /*!< The prescaler provides time base for pulse detection (pulse > 2*tPrescaler) */
    FunctionalState LowPowerMode; /*!< Low power mode - different modulation scheme */
}IrDA_InitType;

/** @addtogroup IrDA_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_IrDA_Init               (USART_HandleType * husart, const USART_InitType * Common,
                                             const IrDA_InitType * Config);
/** @} */

/** @} */

#if (USART_PERIPHERAL_VERSION > 1)
/** @defgroup RS485 RS485 interface
 * @{ */

/** @defgroup RS485_Exported_Types RS485 Exported Types
 * @{ */

/** @brief RS485 setup structure */
typedef struct
{
    FunctionalState      OverSampling8;   /*!< When the over sampling 8 is enabled (instead of 16),
                                               higher baudrate is available */
    FunctionalState      HalfDuplex;      /*!< Half-duplex communication select */
    struct {
        ActiveLevelType  Polarity;        /*!< Polarity of the Driver Enable signal */
        uint8_t          AssertionTime;   /*!< The time between the activation of the DE signal
                                               and the beginning of the start bit in sample time units */
        uint8_t          DeassertionTime; /*!< The time between the end of the last transmitted stop bit and the
                                               deactivation of the DE (Driver Enable) signal in sample time units */
    }DE;                                  /*   Driver Enable signal configuration */
}RS485_InitType;

/** @addtogroup RS485_Exported_Functions
 * @{ */
XPD_ReturnType  XPD_RS485_Init              (USART_HandleType * husart, const USART_InitType * Common,
                                             const RS485_InitType * Config);
/** @} */

/** @} */
#endif

/** @} */

#define XPD_USART_API
#include "xpd_rcc_gen.h"
#include "xpd_rcc_pc.h"
#undef XPD_USART_API

#endif /* XPD_USART_H_ */
