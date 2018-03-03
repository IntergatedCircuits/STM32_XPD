/**
  ******************************************************************************
  * @file    xpd_usart.h
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers USART Module
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
#ifndef __XPD_USART_H_
#define __XPD_USART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_dma.h>
#include <xpd_rcc.h>

/** @defgroup USART
 * @{ */

/** @defgroup USART_Common USART Common
 * @{ */

/* Determine which version of the peripheral is available */
#if   defined(USART_ISR_WUF)
#define __USART_PERIPHERAL_VERSION    3
#elif defined(USART_ISR_BUSY)
#define __USART_PERIPHERAL_VERSION    2
#else
#define __USART_PERIPHERAL_VERSION    1
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

/** @brief USART communication directions */
typedef enum
{
    USART_DIR_TX    = USART_CR1_TE,                 /*!< Transmitter role only */
    USART_DIR_RX    = USART_CR1_RE,                 /*!< Receiver role only */
    USART_DIR_TX_RX = USART_CR1_TE | USART_CR1_RE,  /*!< Full-duplex communication */
}USART_DirectionType;

/** @brief USART inversion selection */
typedef enum
{
    USART_INVERT_NONE           = 0x00, /*!< No logical signals are inverted */
#ifdef USART_CR2_DATAINV
    USART_INVERT_TX             = 0x04, /*!< TX signal inversion is selected */
    USART_INVERT_RX             = 0x02, /*!< RX signal inversion is selected */
    USART_INVERT_DATA           = 0x08, /*!< Data inversion is selected */
    USART_INVERT_BIT_ORDER      = 0x10, /*!< Bit order inversion is selected (MSB first) */
    USART_INVERT_PIN_DIRECTIONS = 0x01, /*!< TX and RX pins are swapped */
#endif
}USART_InversionType;

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
    struct {
        XPD_HandleCallbackType DepInit;      /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;    /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType Transmit;     /*!< Data stream transmission successful callback */
        XPD_HandleCallbackType Receive;      /*!< Data stream reception successful callback */
        XPD_HandleCallbackType Break;        /*!< LIN break detection callback */
        XPD_HandleCallbackType Idle;         /*!< Idle callback */
        XPD_HandleCallbackType ClearToSend;  /*!< Clear To Send callback */
#if defined(__XPD_USART_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
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
#if defined(__XPD_USART_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
    volatile USART_ErrorType Errors;         /*!< Transfer errors */
#endif
}USART_HandleType;

/** @} */

/** @defgroup USART_Common_Exported_Macros USART Common Exported Macros
 * @{ */

#ifdef USART_BB
/**
 * @brief USART Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the USART peripheral instance.
 */
#define         USART_INST2HANDLE(HANDLE,INSTANCE)              \
    ((HANDLE)->Inst    = (INSTANCE),                            \
     (HANDLE)->Inst_BB = USART_BB(INSTANCE),                    \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief USART register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         USART_REG_BIT(HANDLE, REG_NAME, BIT_NAME)       \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief USART Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the USART peripheral instance.
 */
#define         USART_INST2HANDLE(HANDLE,INSTANCE)              \
    ((HANDLE)->Inst    = (INSTANCE),                            \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief USART register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         USART_REG_BIT(HANDLE, REG_NAME, BIT_NAME)       \
    ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)

#endif /* USART_BB */

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
 *            @arg CM:      Character Match
 */
#define         USART_IT_ENABLE(  HANDLE,  IT_NAME)             \
    (__XPD_USART_##IT_NAME##IECTRL(HANDLE, ENABLE))

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
 *            @arg CM:      Character Match
 */
#define         USART_IT_DISABLE( HANDLE,  IT_NAME)             \
        (__XPD_USART_##IT_NAME##IECTRL(HANDLE, DISABLE))

/**
 * @brief  Resume the specified USART DMA requests.
 * @param  HANDLE: specifies the USART Handle.
 * @param  DMA_NAME: specifies the DMA request to resume.
 *         This parameter can be one of the following values:
 *            @arg T:       Transmit
 *            @arg R:       Receive
 */
#define         USART_DMA_ENABLE(HANDLE, DMA_NAME)              \
    (SPI_REG_BIT((HANDLE), CR3, DMA##DMA_NAME) = 1)

/**
 * @brief  Halt the specified USART DMA requests.
 * @param  HANDLE: specifies the USART Handle.
 * @param  DMA_NAME: specifies the DMA request to halt.
 *         This parameter can be one of the following values:
 *            @arg T:       Transmit
 *            @arg R:       Receive
 */
#define         USART_DMA_DISABLE(HANDLE, DMA_NAME)             \
    (SPI_REG_BIT((HANDLE), CR3, DMA##DMA_NAME) = 0)

#if (__USART_PERIPHERAL_VERSION > 1)
/* macros for cross-compatibility */
#ifndef USART_ICR_NECF
#define USART_ICR_NECF      USART_ICR_NCF
#endif
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
 *            @arg ABRF:    Auto BaudRate detection Finished
 *            @arg ABRE:    Auto BaudRate detection Error
 */
#define         USART_FLAG_STATUS(  HANDLE, FLAG_NAME)          \
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
#define         USART_FLAG_CLEAR(HANDLE, FLAG_NAME)             \
    ((USART_ISR_##FLAG_NAME != USART_ISR_RXNE) ?                \
    ((HANDLE)->Inst->ICR.w = USART_ICR_##FLAG_NAME##CF) :       \
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
#define         USART_FLAG_STATUS(  HANDLE, FLAG_NAME)          \
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
#define         USART_FLAG_CLEAR(HANDLE, FLAG_NAME)             \
    do { if(USART_SR_##FLAG_NAME##_Pos <= USART_SR_RXNE_Pos)    \
          {(void)(HANDLE)->Inst->SR.w;(void)(HANDLE)->Inst->DR;}\
       else{(USART_REG_BIT((HANDLE),SR,FLAG_NAME)=0);}}while(0)

#endif /* (__USART_PERIPHERAL_VERSION > 1) */

#define __XPD_USART_IDLEIECTRL(     HANDLE, NEWSTATE)           \
    (USART_REG_BIT((HANDLE),CR1,IDLEIE) = NEWSTATE)

#define __XPD_USART_RXNEIECTRL(HANDLE, NEWSTATE)                \
    (USART_REG_BIT((HANDLE),CR1,RXNEIE) = NEWSTATE)

#define __XPD_USART_TCIECTRL(HANDLE, NEWSTATE)                  \
    (USART_REG_BIT((HANDLE),CR1,TCIE) = NEWSTATE)

#define __XPD_USART_TXEIECTRL(HANDLE, NEWSTATE)                 \
    (USART_REG_BIT((HANDLE),CR1,TXEIE) = NEWSTATE)

#define __XPD_USART_PEIECTRL(HANDLE, NEWSTATE)                  \
    (USART_REG_BIT((HANDLE),CR1,PEIE) = NEWSTATE)

#define __XPD_USART_CMIECTRL(HANDLE, NEWSTATE)                  \
    (USART_REG_BIT((HANDLE),CR1,CMIE) = NEWSTATE)

#define __XPD_USART_LBDIECTRL(HANDLE, NEWSTATE)                 \
    (USART_REG_BIT((HANDLE),CR2,LBDIE) = NEWSTATE)

#define __XPD_USART_EIECTRL(HANDLE, NEWSTATE)                   \
    (USART_REG_BIT((HANDLE),CR3,EIE) = NEWSTATE)

#define __XPD_USART_CTSIECTRL(HANDLE, NEWSTATE)                 \
    (USART_REG_BIT((HANDLE),CR3,CTSIE) = NEWSTATE)

#define __XPD_USART_WUIECTRL(HANDLE, NEWSTATE)                  \
    (USART_REG_BIT((HANDLE),CR3,WUFIE) = NEWSTATE)

/** @} */

/** @addtogroup USART_Common_Exported_Functions
 * @{ */
void            USART_vDeinit               (USART_HandleType * pxUSART);

void            USART_vSetDirection         (USART_HandleType * pxUSART,
                                             USART_DirectionType eDirection);

XPD_ReturnType  USART_eGetStatus            (USART_HandleType * pxUSART);
XPD_ReturnType  USART_ePollStatus           (USART_HandleType * pxUSART,
                                             uint32_t ulTimeout);

XPD_ReturnType  USART_eTransmit             (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             uint16_t usLength,
                                             uint32_t ulTimeout);

XPD_ReturnType  USART_eSend                 (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             uint16_t usLength,
                                             uint32_t ulTimeout);

XPD_ReturnType  USART_eReceive              (USART_HandleType * pxUSART,
                                             void * pvRxData,
                                             uint16_t usLength,
                                             uint32_t ulTimeout);

XPD_ReturnType  USART_eSendReceive          (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             void * pvRxData,
                                             uint16_t usLength,
                                             uint32_t ulTimeout);


void            USART_vTransmit_IT          (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             uint16_t usLength);

void            USART_vSend_IT              (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             uint16_t usLength);

void            USART_vReceive_IT           (USART_HandleType * pxUSART,
                                             void * pvRxData,
                                             uint16_t usLength);

void            USART_vTransmitReceive_IT   (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             void * pvRxData,
                                             uint16_t usLength);

void            USART_vSendReceive_IT       (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             void * pvRxData,
                                             uint16_t usLength);

void            USART_vIRQHandler           (USART_HandleType * pxUSART);


XPD_ReturnType  USART_eTransmit_DMA         (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             uint16_t usLength);

XPD_ReturnType  USART_eSend_DMA             (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             uint16_t usLength);

XPD_ReturnType  USART_eReceive_DMA          (USART_HandleType * pxUSART,
                                             void * pvRxData,
                                             uint16_t usLength);

XPD_ReturnType  USART_eTransmitReceive_DMA  (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             void * pvRxData,
                                             uint16_t usLength);

XPD_ReturnType  USART_eSendReceive_DMA      (USART_HandleType * pxUSART,
                                             void * pvTxData,
                                             void * pvRxData,
                                             uint16_t usLength);

void            USART_vStop_DMA             (USART_HandleType * pxUSART);

#ifdef USART_CR3_OVRDIS
void            USART_vOverrunEnable        (USART_HandleType * pxUSART);
void            USART_vOverrunDisable       (USART_HandleType * pxUSART);
#endif

#ifdef USART_CR2_ABREN
/**
 * @brief Initiates an auto baudrate detection.
 * @param pxUSART: pointer to the USART handle structure
 */
__STATIC_INLINE void USART_vBaudrateDetect(USART_HandleType * pxUSART)
{
    USART_REG_BIT(pxUSART,CR2,ABREN) = 1;
}
#endif /* USART_CR2_ABREN */

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

/** @brief UART baudrate detection strategy */
typedef enum
{
    UART_BAUDRATE_FIXED = 0,               /*!< Fixed baudrate is used (auto baudrate detection is disabled) */

#ifdef USART_CR2_ABREN
    UART_BAUDRATE_AUTO_STARTBIT_MODE  = 1, /*!< START bit is measured from falling to rising edge (D0=1) */
    UART_BAUDRATE_AUTO_2FALLING_MODE  = 3, /*!< START + LSB=1 bits are measured from falling to falling edge (D0=1, D1=0) */
#ifdef USART_CR2_ABRMODE_1
    UART_BAUDRATE_AUTO_0X7FFRAME_MODE = 5, /*!< START to MSB=0 bits are measured from falling to falling edge */
    UART_BAUDRATE_AUTO_0X55FRAME_MODE = 7, /*!< Combination of the above methods */
#endif
#endif
}UART_BaudrateModeType;

/** @brief UART setup structure */
typedef struct {
    uint32_t              Baudrate;      /*!< The USART communication baud rate */
    USART_DirectionType   Directions;    /*!< Selects the module's transmitter and/or receiver capacity */
    uint8_t               DataSize;      /*!< Specifies the USART frame data size in bits */
    USART_StopBitsType    StopBits;      /*!< The number of stop bits transmitted */
    FunctionalState       SingleSample;  /*!< Sets the sampling to 1/bit from default 3/bit */
    USART_ParityType      Parity;        /*!< Specifies if the last frame bit is the parity and how it is calculated */
    USART_InversionType   Inversions;    /*!< Specifies which inversions to activate */

    UART_FlowControlType  FlowControl;   /*!< Hardware flow control select */
    FunctionalState       OverSampling8; /*!< When the over sampling 8 is enabled (instead of 16),
                                              higher baudrate is available */
    FunctionalState       HalfDuplex;    /*!< Half-duplex communication select */
    UART_BaudrateModeType BaudrateMode;  /*!< Baudrate detection mode */
}UART_InitType;

/** @} */

/** @addtogroup UART_Exported_Functions
 * @{ */
void            USART_vInitAsync            (USART_HandleType * pxUSART,
                                             const UART_InitType * pxConfig);
/** @} */

/** @} */

/** @defgroup USRT Universal Synchronous Receiver-Transmitter
 * @{ */

/** @defgroup USRT_Exported_Types USRT Exported Types
 * @{ */

/** @brief USRT setup structure */
typedef struct {
    uint32_t              Baudrate;      /*!< The USART communication baud rate */
    uint8_t               DataSize;      /*!< Specifies the USART frame data size in bits */
    USART_StopBitsType    StopBits;      /*!< The number of stop bits transmitted */
    FunctionalState       SingleSample;  /*!< Sets the sampling to 1/bit from default 3/bit */
    USART_ParityType      Parity;        /*!< Specifies if the last frame bit is the parity and how it is calculated */
    USART_InversionType   Inversions;    /*!< Specifies which inversions to activate */

    struct {
        ActiveLevelType   Polarity;      /*!< Steady state of the serial clock */
        ClockPhaseType    Phase;         /*!< The clock transition on which the bit capture is made */
        FunctionalState   LastBit;       /*!< The clock pulse for the last transmitted data bit (MSB)
                                              has to be provided */
    }Clock;                              /*   Output clock settings */
}USRT_InitType;

/** @} */

/** @addtogroup USRT_Exported_Functions
 * @{ */
void            USART_vInitSync             (USART_HandleType * pxUSART,
                                             const USRT_InitType * pxConfig);
/** @} */

/** @} */

/** @defgroup LIN Local Interconnect Network
 * @{ */

/** @defgroup USRT_Exported_Types USRT Exported Types
 * @{ */

/** @brief USRT setup structure */
typedef struct {
    uint32_t              Baudrate;      /*!< The USART communication baud rate */
    USART_InversionType   Inversions;    /*!< Specifies which inversions to activate */

    UART_BaudrateModeType BaudrateMode;  /*!< Baudrate detection mode */
    uint8_t               BreakSize;     /*!< Bit size of the break frame for detection [10 .. 11] */
}LIN_InitType;

/** @} */

/** @addtogroup LIN_Exported_Functions
 * @{ */
void            USART_vInitLIN              (USART_HandleType * pxUSART,
                                             const LIN_InitType * pxConfig);
void            USART_vSendBreak            (USART_HandleType * pxUSART);
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

#if (__USART_PERIPHERAL_VERSION > 2)
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
    uint32_t              Baudrate;      /*!< The USART communication baud rate */
    USART_DirectionType   Directions;    /*!< Selects the module's transmitter and/or receiver capacity */
    uint8_t               DataSize;      /*!< Specifies the USART frame data size in bits */
    USART_StopBitsType    StopBits;      /*!< The number of stop bits transmitted */
    FunctionalState       SingleSample;  /*!< Sets the sampling to 1/bit from default 3/bit */
    USART_ParityType      Parity;        /*!< Specifies if the last frame bit is the parity and how it is calculated */
    USART_InversionType   Inversions;    /*!< Specifies which inversions to activate */

    FunctionalState       OverSampling8; /*!< When the over sampling 8 is enabled (instead of 16),
                                              higher baudrate is available */
    FunctionalState       HalfDuplex;    /*!< Half-duplex communication select */
    UART_BaudrateModeType BaudrateMode;  /*!< Baudrate detection mode */
    uint8_t               Address;       /*!< Specifies the slave device address */
    uint8_t               AddressLength; /*!< Specifies the address length (default 4 or 7 bits) */
    MSUART_UnmuteType     UnmuteMethod;  /*!< Specifies the activation strategy from Mute */

#if (__USART_PERIPHERAL_VERSION > 2)
    MSUART_WakeUpType     WakeUpMethod;  /*!< Specifies the activation strategy from STOP mode */
#endif
}MSUART_InitType;
/** @} */

/** @addtogroup MSUART_Exported_Functions
 * @{ */
void            USART_vInitMultiSlave       (USART_HandleType * pxUSART,
                                             const MSUART_InitType * pxConfig);

/**
 * @brief Mutes the MultiSlave UART
 * @param pxUSART: pointer to the USART handle structure
 */
__STATIC_INLINE void USART_vMute(USART_HandleType * pxUSART)
{
#if (__USART_PERIPHERAL_VERSION > 1)
    USART_REG_BIT(pxUSART, CR1, MME) = 1;

    /* Send actual mute request */
    pxUSART->Inst->RQR.w = USART_RQR_MMRQ;
#else
    USART_REG_BIT(pxUSART, CR1, RWU) = 1;
#endif
}

/**
 * @brief Unmutes the MultiSlave UART
 * @param pxUSART: pointer to the USART handle structure
 */
__STATIC_INLINE void USART_vUnmute(USART_HandleType * pxUSART)
{
#if (__USART_PERIPHERAL_VERSION > 1)
    USART_REG_BIT(pxUSART, CR1, MME) = 0;
#else
    USART_REG_BIT(pxUSART, CR1, RWU) = 0;
#endif
}

#if (__USART_PERIPHERAL_VERSION > 2)
/**
 * @brief Enables the Stop mode for the UART
 * @note  The UART is able to wake up the MCU from Stop 1 mode as long as UART clock is HSI or LSE.
 * @note  There shall be no ongoing transfer in the UART when Stop mode is entered.
 * @param pxUSART: pointer to the USART handle structure
 */
__STATIC_INLINE void USART_vStopModeEnable(USART_HandleType * pxUSART)
{
    /* Enable RXNE interrupt when it is the wake up source */
    if(pxUSART->Inst->CR3.b.WUS == MSUART_WAKEUP_DATA_RECEIVED)
    {
        USART_IT_ENABLE(pxUSART, RXNE);
    }
    /* Otherwise enable dedicated interrupt */
    else
    {
        USART_IT_ENABLE(pxUSART, WU);
    }
    USART_REG_BIT(pxUSART, CR1, UESM) = 1;
}

/**
 * @brief Disables the Stop mode for the UART
 * @param pxUSART: pointer to the USART handle structure
 */
__STATIC_INLINE void USART_vStopModeDisable(USART_HandleType * pxUSART)
{
    USART_REG_BIT(pxUSART, CR1, UESM) = 0;
}
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
    uint32_t              Baudrate;      /*!< The USART communication baud rate */
    USART_DirectionType   Directions;    /*!< Selects the module's transmitter and/or receiver capacity */
    uint8_t               DataSize;      /*!< Specifies the USART frame data size in bits */
    FunctionalState       SingleSample;  /*!< Sets the sampling to 1/bit from default 3/bit */
    USART_ParityType      Parity;        /*!< Specifies how the parity is calculated */
    USART_InversionType   Inversions;    /*!< Specifies which inversions to activate */

    uint8_t               GuardTime;     /*!< Delays the raising of TC flag by a number of baud clocks */
    FunctionalState       NACK;          /*!< Enable to force frame error at the end of received frame
                                              with detected parity error */
    struct {
        ActiveLevelType   Polarity;      /*!< Steady state of the serial clock */
        ClockPhaseType    Phase;         /*!< The clock transition on which the bit capture is made */
        uint8_t           Divider;       /*!< Prescaler value that divides the peripheral clock
                                              before SCLK output [2, 4, ... 62] */
    }Clock;                              /*   Output clock settings */
#if (__USART_PERIPHERAL_VERSION > 1)
    uint32_t        RxTimeout;           /*!< Specifies the receiver timeout value in baud block count for
                                              Character Wait Time (CWT) and Block Wait Time (BWT). Set to 0 to disable. */
    uint8_t         BlockLength;         /*!< Specifies the SmartCard Block Length in T = 1 Reception mode */
    uint8_t         AutoRetries;         /*!< Specifies the number of retries in receive and transmit mode
                                              before reaching error state [0 .. 7] */
#endif
}SmartCard_InitType;
/** @} */

/** @addtogroup SmartCard_Exported_Functions
 * @{ */
void            USART_vInitSmartCard        (USART_HandleType * pxUSART,
                                             const SmartCard_InitType * pxConfig);
/** @} */

/** @} */

/** @defgroup IrDA IrDA interface
 * @{ */

/** @defgroup IrDA_Exported_Types IrDA Exported Types
 * @{ */

/** @brief IrDA setup structure */
typedef struct
{
    uint32_t              Baudrate;      /*!< The USART communication baud rate */
    uint8_t               DataSize;      /*!< Specifies the USART frame data size in bits */
    USART_ParityType      Parity;        /*!< Specifies if the last frame bit is the parity and how it is calculated */
    USART_InversionType   Inversions;    /*!< Specifies which inversions to activate */

    uint8_t               Prescaler;     /*!< The prescaler provides time base for pulse detection (pulse > 2*tPrescaler) */
    FunctionalState       LowPowerMode;  /*!< Low power mode - different modulation scheme */
}IrDA_InitType;

/** @addtogroup IrDA_Exported_Functions
 * @{ */
void            USART_vInitIrDA             (USART_HandleType * pxUSART,
                                             const IrDA_InitType * pxConfig);
/** @} */

/** @} */

#if (__USART_PERIPHERAL_VERSION > 1)
/** @defgroup RS485 RS485 interface
 * @{ */

/** @defgroup RS485_Exported_Types RS485 Exported Types
 * @{ */

/** @brief RS485 setup structure */
typedef struct
{
    uint32_t              Baudrate;       /*!< The USART communication baud rate */
    USART_DirectionType   Directions;     /*!< Selects the module's transmitter and/or receiver capacity */
    uint8_t               DataSize;       /*!< Specifies the USART frame data size in bits */
    USART_StopBitsType    StopBits;       /*!< The number of stop bits transmitted */
    FunctionalState       SingleSample;   /*!< Sets the sampling to 1/bit from default 3/bit */
    USART_ParityType      Parity;         /*!< Specifies if the last frame bit is the parity and how it is calculated */
    USART_InversionType   Inversions;     /*!< Specifies which inversions to activate */

    FunctionalState       OverSampling8;  /*!< When the over sampling 8 is enabled (instead of 16),
                                               higher baudrate is available */
    FunctionalState       HalfDuplex;     /*!< Half-duplex communication select */
    UART_BaudrateModeType BaudrateMode;   /*!< Baudrate detection mode */
    struct {
        ActiveLevelType   Polarity;       /*!< Polarity of the Driver Enable signal */
        uint8_t           AssertionTime;  /*!< The time between the activation of the DE signal
                                               and the beginning of the start bit in sample time units */
        uint8_t           DeassertionTime;/*!< The time between the end of the last transmitted stop bit and the
                                               deactivation of the DE (Driver Enable) signal in sample time units */
    }DE;                                  /*   Driver Enable signal configuration */
}RS485_InitType;

/** @addtogroup RS485_Exported_Functions
 * @{ */
void            USART_vInitRS485            (USART_HandleType * pxUSART,
                                             const RS485_InitType * pxConfig);
/** @} */

/** @} */
#endif

/** @} */

#define XPD_USART_API
#include <xpd_rcc_pc.h>
#undef XPD_USART_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_USART_H_ */
