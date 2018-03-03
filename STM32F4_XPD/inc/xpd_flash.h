/**
  ******************************************************************************
  * @file    xpd_flash.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Flash Module
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
#ifndef __XPD_FLASH_H_
#define __XPD_FLASH_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

/** @defgroup FLASH
 * @{ */

/** @defgroup FLASH_Exported_Types FLASH Exported Types
 * @{ */

/** @brief FLASH operation types */
typedef enum
{
  FLASH_OPERATION_NONE        = 0,            /*!< FLASH idle */
  FLASH_OPERATION_ERASE_BLOCK = FLASH_CR_SER, /*!< FLASH sector erase operation */
  FLASH_OPERATION_ERASE_BANK1 = FLASH_CR_MER, /*!< FLASH bank1 mass erase operation */
#ifdef FLASH_CR_MER1
  FLASH_OPERATION_ERASE_BANK2 = FLASH_CR_MER1,/*!< FLASH bank2 mass erase operation */
#endif
  FLASH_OPERATION_PROGRAM     = FLASH_CR_PG,  /*!< FLASH write program operation */
}FLASH_OperationType;

/** @brief FLASH error types */
typedef enum
{
    FLASH_ERROR_NONE      = 0,               /*!< No errors */
    FLASH_ERROR_WRITEPROT = FLASH_SR_WRPERR, /*!< Write protection error */
    FLASH_ERROR_ALIGNMENT = FLASH_SR_PGAERR, /*!< Programming Alignment error */
    FLASH_ERROR_SEQUENCE  = FLASH_SR_PGSERR, /*!< Programming Sequence error */
#ifdef FLASH_SR_PGPERR
    FLASH_ERROR_PARALLEL  = FLASH_SR_PGPERR, /*!< Programming Parallelism error */
#endif
#ifdef FLASH_SR_RDERR
    FLASH_ERROR_READPROT  = FLASH_SR_RDERR,  /*!< Read protection error */
#endif
}FLASH_ErrorType;

/** @brief FLASH callbacks container structure */
typedef struct {
    XPD_SimpleCallbackType EraseComplete;   /*!< Erase operation complete callback */
    XPD_SimpleCallbackType ProgramComplete; /*!< Program operation complete callback */
    XPD_SimpleCallbackType Error;           /*!< Operation error callback */
}FLASH_CallbacksType;

/** @} */

/** @defgroup FLASH_Exported_Variables FLASH Exported Variables
 * @{ */

/** @brief FLASH callbacks container struct */
extern FLASH_CallbacksType FLASH_xCallbacks;

/** @} */

/** @defgroup FLASH_Exported_Macros FLASH Exported Macros
 * @{ */

#ifdef FLASH_BB
/**
 * @brief FLASH register bit accessing macro
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define             FLASH_REG_BIT(REG_NAME, BIT_NAME)   \
    (FLASH_BB->REG_NAME.BIT_NAME)
#else
/**
 * @brief FLASH register bit accessing macro
 * @param REG_NAME: specifies the register name.
 * @param BIT_NAME: specifies the register bit name.
 */
#define             FLASH_REG_BIT(REG_NAME, BIT_NAME)   \
    (FLASH->REG_NAME.b.BIT_NAME)
#endif

/**
 * @brief  Enable the specified FLASH interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg ERR:         Error occurrence
 *            @arg EOP:         End of Operation
 */
#define             FLASH_IT_ENABLE(IT_NAME)            \
    (FLASH_REG_BIT(CR,IT_NAME##IE) = 1)

/**
 * @brief  Enable the specified FLASH interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg ERR:         Error occurrence
 *            @arg EOP:         End of Operation
 */
#define             FLASH_IT_DISABLE(IT_NAME)           \
    (FLASH_REG_BIT(CR,IT_NAME##IE) = 0)

/**
 * @brief  Get the specified FLASH flag.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg BSY:         Busy flag
 *            @arg EOP:         End of Operation flag
 *            @arg WRPERR:      Write protected error flag
 *            @arg RDERR:       Read protected error flag
 *            @arg PGAERR:      Programming Alignment error flag
 *            @arg PGPERR:      Programming Parallelism error flag
 *            @arg PGSERR:      Programming Sequence error flag
 */
#define         FLASH_FLAG_STATUS(FLAG_NAME)            \
    (FLASH_REG_BIT(SR,FLAG_NAME))

/**
 * @brief  Clear the specified FLASH flag.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg EOP:         End of Operation flag
 *            @arg WRPERR:      Write protected error flag
 *            @arg RDERR:       Read protected error flag
 *            @arg PGAERR:      Programming Alignment error flag
 *            @arg PGPERR:      Programming Parallelism error flag
 *            @arg PGSERR:      Programming Sequence error flag
 */
#define         FLASH_FLAG_CLEAR(FLAG_NAME)             \
    (FLASH->SR.w = FLASH_SR_##FLAG_NAME)

#ifndef FLASH_CR_ERRIE
#define FLASH_CR_ERRIE      ((uint32_t)0x02000000)
#endif

/** @} */

/** @defgroup FLASH_Exported_Functions FLASH Exported Functions
 * @{ */
void            FLASH_vUnlock       (void);
void            FLASH_vLock         (void);

XPD_ReturnType  FLASH_eProgram      (void * pvAddress, const uint8_t * pucData, uint16_t usLength);
XPD_ReturnType  FLASH_eProgram_IT   (void * pvAddress, const uint8_t * pucData, uint16_t usLength);

XPD_ReturnType  FLASH_eEraseBank    (uint8_t ucBank);
XPD_ReturnType  FLASH_eEraseBank_IT (uint8_t ucBank);
XPD_ReturnType  FLASH_eErase        (void * pvAddress, uint16_t usKBytes);
XPD_ReturnType  FLASH_eErase_IT     (void * pvAddress, uint16_t usKBytes);

XPD_ReturnType  FLASH_ePollStatus   (uint32_t ulTimeout);
FLASH_ErrorType FLASH_eGetError     (void);
void *          FLASH_pvGetAddress  (void);

void            FLASH_vIRQHandler   (void);

/**
 * @brief Sets the flash memory access latency (in clock cycles).
 * @param ucLatency: the flash access latency
 */
__STATIC_INLINE void FLASH_vSetLatency(uint8_t ucLatency)
{
    FLASH->ACR.b.LATENCY = ucLatency;
}

/**
 * @brief Gets the flash memory access latency (in clock cycles).
 * @return The flash access latency
 */
__STATIC_INLINE uint8_t FLASH_ucGetLatency(void)
{
    return FLASH->ACR.b.LATENCY;
}

#ifdef FLASH_ACR_PRFTEN
/**
 * @brief Sets the new state for the flash prefetch buffer.
 * @param eNewState: the new state
 */
__STATIC_INLINE void FLASH_vPrefetchBuffer(FunctionalState eNewState)
{
    FLASH_REG_BIT(ACR,PRFTEN) = eNewState;
}
#endif

#ifdef FLASH_ACR_ICEN
/**
 * @brief Sets the new state for the instruction cache.
 * @param eNewState: the new state
 */
__STATIC_INLINE void FLASH_vInstCache(FunctionalState eNewState)
{
    FLASH_REG_BIT(ACR,ICEN) = eNewState;
}
#endif

#ifdef FLASH_ACR_DCEN
/**
 * @brief Sets the new state for the data cache.
 * @param eNewState: the new state
 */
__STATIC_INLINE void FLASH_vDataCache(FunctionalState eNewState)
{
    FLASH_REG_BIT(ACR,DCEN) = eNewState;
}
#endif

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_FLASH_H_ */
