/**
  ******************************************************************************
  * @file    xpd_i2c.h
  * @author  Benedek Kupper
  * @version 0.1
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers I2C Module
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
#ifndef __XPD_I2C_H_
#define __XPD_I2C_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_dma.h>
#include <xpd_rcc.h>

/** @defgroup I2C
 * @{ */

/** @defgroup I2C_Exported_Types I2C Exported Types
 * @{ */

/** @brief I2C addressing modes */
typedef enum
{
    I2C_ADDRESS_7BIT    = 0, /*!< Normal 7 bit addressing */
    I2C_ADDRESS_10BIT   = 1, /*!< 10 bit addressing mode */
}I2C_AddressModeType;

/** @brief I2C error types */
typedef enum
{
    I2C_ERROR_NONE      = 0,                /*!< No error */
    I2C_ERROR_NACK      = I2C_ISR_NACKF,    /*!< Not acknowledge error */
    I2C_ERROR_BUS       = I2C_ISR_BERR,     /*!< Bus error */
    I2C_ERROR_ARBIT     = I2C_ISR_ARLO,     /*!< Arbitration lost error */
    I2C_ERROR_OVERRUN   = I2C_ISR_OVR,      /*!< Overrun/Underrun error */
    I2C_ERROR_TIMEOUT   = I2C_ISR_TIMEOUT,  /*!< Timeout error */
    I2C_ERROR_DMA       = 1,                /*!< DMA transfer error */
}I2C_ErrorType;

/** @brief I2C transfer directions */
typedef enum
{
    I2C_DIRECTION_WRITE = 0, /*!< The master sends data to the receiver slave */
    I2C_DIRECTION_READ  = 1  /*!< The master receives data from the sender slave */
}I2C_DirectionType;

/** @brief I2C transfer context */
typedef struct
{
    union {
    struct {
    uint16_t SlaveAddress_10bit : 10; /*!< The slave's address stored in 10 bits (7 bit addresses are left shifted by one) */
    I2C_DirectionType Direction : 1;  /*!< The transfer direction (from the master's point of view) */
    uint16_t CmdSize : 2;             /*!< The size of the optional slave command */
    uint16_t : 3;
    };
    struct {
    uint16_t : 1;
    uint16_t SlaveAddress_7bit : 7;   /*!< The slave's address stored in 7 bits
                    @warning Do not set in struct initialization, as it will conflict with
                             @ref I2C_TransferType::Slave10bitAddress,
                             @ref I2C_TransferType::CmdSize and
                             @ref I2C_TransferType::Direction,
                             either this or they will be set to 0. */
    uint16_t : 8;
    };
    uint16_t wHeader;   /* [Internal] */
    };
    uint16_t Cmd;       /*!< The command that is sent to the slave for servicing.
                             The command stage is only performed when @ref I2C_TransferType::CmdSize is set. */
    uint8_t *Data;      /*!< Source/target buffer for the transferred data, depending on
                             the current role in the transfer and @ref I2C_TransferType::Direction */
    uint16_t Length;    /*!< The desired length of the data transfer */
}I2C_TransferType;

/** @brief I2C setup structure */
typedef struct
{
    uint32_t BusFreq_Hz;                    /*!< Desired I2C bus frequency [Hz] */
    union {
    struct {
    uint16_t : 4;
    uint16_t : 1;
    I2C_AddressModeType AddressingMode : 1; /*!< Global addressing mode */
    uint16_t : 3;
    FunctionalState NoStretch : 1;          /*!< [Slave] Do not stretch SCL low when waiting for software
                                                 to control the transfer */
    uint16_t : 1;
    FunctionalState GeneralCall : 1;        /*!< [Slave] Acknowledge and handle a General Call (address=00h) */
    uint16_t : 4;
    };
    uint16_t wCfg;                  /* [Internal] */
    };
    union {
        struct {
        uint16_t : 1;
        uint16_t _7bit : 7;         /*!< The address stored in 7 bits (disabled when set to 0) */
        uint16_t : 8;
        };
        struct {
        uint16_t _10bit : 10;       /*!< The address stored in 10 bits (disabled when set to 0) */
        uint16_t : 6;
        };
        uint16_t wValue;
    }OwnAddress1;                   /*!< [Slave] The module's address, either 7 or 10 bit,
                                          depending on @ref I2C_InitType::AddressingMode */
    union {
        struct {
        uint16_t : 1;
        uint16_t _7bit : 7;         /*!< The address stored in 7 bits (disabled when set to 0) */
        uint16_t MaskedBitCount : 3;/*!< The amount of LS bits ignored when matching the 7 bit address */
        uint16_t : 5;
        };
        uint16_t wValue;
    }OwnAddress2;                   /*!< [Slave] The module's secondary address, only used with 7 bit addressing */
}I2C_InitType;

/** @brief I2C Handle structure */
typedef struct
{
    I2C_TypeDef * Inst;                         /*!< The address of the peripheral instance used by the handle */
#ifdef I2C_BB
    I2C_BitBand_TypeDef * Inst_BB;              /*!< The address of the peripheral instance in the bit-band region */
#endif
    struct {
        XPD_HandleCallbackType DepInit;         /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;       /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType MasterComplete;  /*!< Data stream transmission successful callback */
        XPD_HandleCallbackType SlaveAddressed;  /*!< Data stream reception successful callback */
        XPD_HandleCallbackType SlaveComplete;   /*!< Data stream reception successful callback */
        XPD_HandleCallbackType Error;           /*!< Error callbacks */
    }Callbacks;                                 /*   Handle Callbacks */
    struct {
        DMA_HandleType * Transmit;              /*!< DMA handle for data transmission */
        DMA_HandleType * Receive;               /*!< DMA handle for data reception */
    }DMA;                                       /*   DMA handle references */
    XPD_HandleCallbackType IRQHandler;          /*!< Contextual interrupt request handler reference */
    struct {
        const I2C_TransferType * pMaster;       /*!< Current master mode transfer */
        I2C_TransferType Slave;                 /*!< Slave mode transfer */
    }Transfers;                                 /*   Current transfer references */
    DataStreamType Stream;                      /*!< Data transfer management */
    uint16_t DataCtrlBits;                      /*!< Data stage control bits to use */
    RCC_PositionType CtrlPos;                   /*!< Relative position for reset and clock control */
    volatile I2C_ErrorType Errors;              /*!< Transfer errors */
}I2C_HandleType;

/** @} */

/** @defgroup I2C_Exported_Macros I2C Exported Macros
 * @{ */

#ifdef I2C_BB
/**
 * @brief I2C Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the I2C peripheral instance.
 */
#define         I2C_INST2HANDLE(HANDLE,INSTANCE)                \
    ((HANDLE)->Inst    = (INSTANCE),                            \
     (HANDLE)->Inst_BB = I2C_BB(INSTANCE),                      \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief I2C register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         I2C_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)

#else
/**
 * @brief I2C Instance to handle binder macro
 * @param HANDLE: specifies the peripheral handle.
 * @param INSTANCE: specifies the I2C peripheral instance.
 */
#define         I2C_INST2HANDLE(HANDLE,INSTANCE)                \
    ((HANDLE)->Inst    = (INSTANCE),                            \
     (HANDLE)->CtrlPos = RCC_POS_##INSTANCE)

/**
 * @brief I2C register bit accessing macro
 * @param HANDLE: specifies the peripheral handle.
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define         I2C_REG_BIT(HANDLE, REG_NAME, BIT_NAME)         \
    ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)

#endif /* I2C_BB */

#ifndef I2C_ISR_NACK
#define I2C_ISR_NACK            I2C_ISR_NACKF
#define I2C_ISR_NACK_Pos        I2C_ISR_NACKF_Pos
#endif
#ifndef I2C_ISR_STOP
#define I2C_ISR_STOP            I2C_ISR_STOPF
#define I2C_ISR_STOP_Pos        I2C_ISR_STOPF_Pos
#endif
#define I2C_ICR_TXECF           0

/**
 * @brief  Enable the specified I2C interrupt.
 * @param  HANDLE: specifies the I2C Handle.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg TX:      Transmit Interrupt
 *            @arg RX:      Receive Interrupt
 *            @arg ADDR:    Address Match Interrupt
 *            @arg NACK:    Not Acknowledge Received Interrupt
 *            @arg STOP:    Stop Detection Interrupt
 *            @arg TC:      Transfer Complete Interrupt
 *            @arg ERR:     Error Interrupts
 */
#define         I2C_IT_ENABLE(HANDLE, IT_NAME)                  \
    (I2C_REG_BIT((HANDLE),CR1,IT_NAME##IE) = 1)

/**
 * @brief  Disable the specified I2C interrupt.
 * @param  HANDLE: specifies the I2C Handle.
 * @param  IT_NAME: specifies the interrupt to disable.
 *         This parameter can be one of the following values:
 *            @arg TX:      Transmit Interrupt
 *            @arg RX:      Receive Interrupt
 *            @arg ADDR:    Address Match Interrupt
 *            @arg NACK:    Not Acknowledge Received Interrupt
 *            @arg STOP:    Stop Detection Interrupt
 *            @arg TC:      Transfer Complete Interrupt
 *            @arg ERR:     Error Interrupts
 */
#define         I2C_IT_DISABLE(HANDLE, IT_NAME)                 \
    (I2C_REG_BIT((HANDLE),CR1,IT_NAME##IE) = 0)

/**
 * @brief  Get the specified I2C state flag.
 * @param  HANDLE: specifies the I2C Handle.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg TXE:     Transmit data register empty
 *            @arg RXNE:    Receive data register not empty
 *            @arg OVR:     Overrun/Underrun
 *            @arg BUSY:    Bus busy
 *            @arg TXIS:    Transmit interrupt status
 *            @arg ADDR:    Address matched
 *            @arg NACK:    Not Acknowledge received
 *            @arg STOP:    Stop detection
 *            @arg TC:      Transfer Complete
 *            @arg TCR:     Transfer Complete Reload
 *            @arg BERR:    Bus error
 *            @arg ARLO:    Arbitration lost
 *            @arg PECERR:  PEC Error in reception
 *            @arg TIMEOUT: Timeout or tLOW detection
 *            @arg ALERT:   SMBus alert
 *            @arg DIR:     Transfer direction
 * @return The state of the flag.
 */
#define         I2C_FLAG_STATUS(HANDLE, FLAG_NAME)              \
    (((HANDLE)->Inst->ISR.w >> I2C_ISR_##FLAG_NAME##_Pos) & 1)

/**
 * @brief  Clear the specified I2C state flag.
 * @param  HANDLE: specifies the I2C Handle.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg OVR:     Overrun/Underrun
 *            @arg ADDR:    Address matched
 *            @arg NACK:    Not Acknowledge received
 *            @arg STOP:    Stop detection
 *            @arg BERR:    Bus error
 *            @arg ARLO:    Arbitration lost
 *            @arg PECERR:  PEC Error in reception
 *            @arg TIMEOUT: Timeout or tLOW detection
 *            @arg ALERT:   SMBus alert
 */
#define         I2C_FLAG_CLEAR(HANDLE, FLAG_NAME)               \
    ((I2C_ISR_##FLAG_NAME <= I2C_ISR_TXE) ?                     \
    ((HANDLE)->Inst->ISR.w = I2C_ISR_##FLAG_NAME) :             \
    ((HANDLE)->Inst->ICR.w = I2C_ICR_##FLAG_NAME##CF))

/** @} */

/** @addtogroup I2C_Common_Exported_Functions
 * @{ */
void            I2C_vInit                   (I2C_HandleType * pxI2C, const I2C_InitType * pxConfig);
void            I2C_vDeinit                 (I2C_HandleType * pxI2C);

XPD_ReturnType  I2C_eGetStatus              (I2C_HandleType * pxI2C);
XPD_ReturnType  I2C_ePollStatus             (I2C_HandleType * pxI2C, uint32_t ulTimeout);

uint16_t        I2C_usGetRejectedLength     (I2C_HandleType * pxI2C);

void            I2C_vIRQHandler_EV          (I2C_HandleType * pxI2C);
void            I2C_vIRQHandler_ER          (I2C_HandleType * pxI2C);
/** @} */

/** @addtogroup I2C_Master_Exported_Functions
 * @{ */
XPD_ReturnType  I2C_eMasterTransfer         (I2C_HandleType * pxI2C, const I2C_TransferType * pxTransfer,
                                             uint32_t ulTimeout);
void            I2C_vMasterTransfer_IT      (I2C_HandleType * pxI2C, const I2C_TransferType * pxTransfer);
/** @} */

/** @addtogroup I2C_Slave_Exported_Functions
 * @{ */
XPD_ReturnType  I2C_eSlaveListen            (I2C_HandleType * pxI2C, uint8_t ucCmdSize,
                                             uint32_t ulTimeout);
void            I2C_vSlaveListen_IT         (I2C_HandleType * pxI2C, uint8_t ucCmdSize);
void            I2C_vSlaveSuspend_IT        (I2C_HandleType * pxI2C);

XPD_ReturnType  I2C_eSlaveTransferData      (I2C_HandleType * pxI2C, uint32_t ulTimeout);
void            I2C_vSlaveTransferData_IT   (I2C_HandleType * pxI2C);

/**
 * @brief Returns the slave transfer info reference, which specifies the current I2C transfer request,
 *        and where the application has to configure the data used for the transfer.
 * @param pxI2C: pointer to the I2C handle structure
 * @return The slave transfer info reference
 */
__STATIC_INLINE I2C_TransferType * I2C_pxSlaveTransferInfo(I2C_HandleType * pxI2C)
{
    return &pxI2C->Transfers.Slave;
}

/** @} */

/** @} */

#define XPD_I2C_API
#include <xpd_rcc_pc.h>
#include <xpd_syscfg.h>
#undef XPD_I2C_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_I2C_H_ */
