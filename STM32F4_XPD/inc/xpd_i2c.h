/**
  ******************************************************************************
  * @file    xpd_i2c.h
  * @author  Benedek Kupper
  * @version 0.1
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Inter-Interface Communication Module
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

/** @defgroup I2C
 * @{ */

/** @defgroup I2C_Exported_Types I2C Exported Types
 * @{ */

/** @brief I2C Handle structure */
typedef struct
{
    I2C_TypeDef * Inst;                      /*!< The address of the peripheral instance used by the handle */
#ifdef I2C_BB
    I2C_BitBand_TypeDef * Inst_BB;           /*!< The address of the peripheral instance in the bit-band region */
#endif
    struct {
        XPD_HandleCallbackType DepInit;      /*!< Callback to initialize module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType DepDeinit;    /*!< Callback to restore module dependencies (GPIOs, IRQs, DMAs) */
        XPD_HandleCallbackType Transmit;     /*!< Data stream transmission successful callback */
        XPD_HandleCallbackType Receive;      /*!< Data stream reception successful callback */
#if defined(__XPD_I2C_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
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
#if defined(__XPD_I2C_ERROR_DETECT) || defined(__XPD_DMA_ERROR_DETECT)
    //volatile I2C_ErrorType Errors;           /*!< Transfer errors */
#endif
}I2C_HandleType;

/** @} */

/** @addtogroup I2C_Exported_Functions
 * @{ */

/** @} */

/** @} */

#define XPD_I2C_API
#include <xpd_rcc_pc.h>
#undef XPD_I2C_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_I2C_H_ */
