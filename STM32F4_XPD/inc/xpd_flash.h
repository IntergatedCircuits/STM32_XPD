/**
  ******************************************************************************
  * @file    xpd_flash.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-09-24
  * @brief   STM32 eXtensible Peripheral Drivers Flash Module
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
#ifndef __XPD_FLASH_H_
#define __XPD_FLASH_H_

#include "xpd_common.h"
#include "xpd_config.h"

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
}XPD_FLASH_CallbacksType;

/** @} */

/** @defgroup FLASH_Exported_Variables FLASH Exported Variables
 * @{ */

/** @brief FLASH callbacks container struct */
extern XPD_FLASH_CallbacksType XPD_FLASH_Callbacks;

/** @} */

/** @defgroup FLASH_Exported_Macros FLASH Exported Macros
 * @{ */

#ifdef FLASH_BB
/**
 * @brief FLASH register bit accessing macro
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
 */
#define             FLASH_REG_BIT(REG_NAME, BIT_NAME)   \
    (FLASH_BB->REG_NAME.BIT_NAME)
#else
/**
 * @brief FLASH register bit accessing macro
 * @param REG: specifies the register name.
 * @param BIT: specifies the register bit name.
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
#define             XPD_FLASH_EnableIT(IT_NAME)         \
    (FLASH_REG_BIT(CR,IT_NAME##IE) = 1)

/**
 * @brief  Enable the specified FLASH interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg ERR:         Error occurrence
 *            @arg EOP:         End of Operation
 */
#define             XPD_FLASH_DisableIT(IT_NAME)        \
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
#define         XPD_FLASH_GetFlag(FLAG_NAME)            \
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
#define         XPD_FLASH_ClearFlag(FLAG_NAME)          \
    (FLASH->SR.w = FLASH_SR_##FLAG_NAME)

#ifndef FLASH_CR_ERRIE
#define FLASH_CR_ERRIE      ((uint32_t)0x02000000)
#endif

/** @} */

/** @defgroup FLASH_Exported_Functions FLASH Exported Functions
 * @{ */
void            XPD_FLASH_Unlock            (void);
void            XPD_FLASH_Lock              (void);

XPD_ReturnType  XPD_FLASH_Program           (void * Address, const void * Data, uint16_t Length);
XPD_ReturnType  XPD_FLASH_Program_IT        (void * Address, const void * Data, uint16_t Length);

XPD_ReturnType  XPD_FLASH_EraseBank         (uint8_t Bank);
XPD_ReturnType  XPD_FLASH_EraseBank_IT      (uint8_t Bank);
XPD_ReturnType  XPD_FLASH_Erase             (void * Address, uint16_t kBytes);
XPD_ReturnType  XPD_FLASH_Erase_IT          (void * Address, uint16_t kBytes);

XPD_ReturnType  XPD_FLASH_PollStatus        (uint32_t Timeout);
FLASH_ErrorType XPD_FLASH_GetError          (void);
uint32_t        XPD_FLASH_GetAddress        (void);

void            XPD_FLASH_IRQHandler        (void);

/**
 * @brief Sets the flash memory access latency (in clock cycles).
 * @param Latency: the flash access latency
 */
__STATIC_INLINE void XPD_FLASH_SetLatency(uint8_t Latency)
{
    FLASH->ACR.b.LATENCY = Latency;
}

/**
 * @brief Gets the flash memory access latency (in clock cycles).
 * @return The flash access latency
 */
__STATIC_INLINE uint8_t XPD_FLASH_GetLatency(void)
{
    return FLASH->ACR.b.LATENCY;
}

#ifdef FLASH_ACR_PRFTEN
/**
 * @brief Sets the new state for the flash prefetch buffer.
 * @param NewState: the new state
 */
__STATIC_INLINE void XPD_FLASH_PrefetchBufferCtrl(FunctionalState NewState)
{
    FLASH_REG_BIT(ACR,PRFTEN) = NewState;
}
#endif

#ifdef FLASH_ACR_ICEN
/**
 * @brief Sets the new state for the instruction cache.
 * @param NewState: the new state
 */
__STATIC_INLINE void XPD_FLASH_InstCacheCtrl(FunctionalState NewState)
{
    FLASH_REG_BIT(ACR,ICEN) = NewState;
}
#endif

#ifdef FLASH_ACR_DCEN
/**
 * @brief Sets the new state for the data cache.
 * @param NewState: the new state
 */
__STATIC_INLINE void XPD_FLASH_DataCacheCtrl(FunctionalState NewState)
{
    FLASH_REG_BIT(ACR,DCEN) = NewState;
}
#endif

/** @} */

/** @} */

#endif /* __XPD_FLASH_H_ */
