/**
  ******************************************************************************
  * @file    xpd_common.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-01
  * @brief   STM32 eXtensible Peripheral Drivers Common Module
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
#ifndef XPD_COMMON_H_
#define XPD_COMMON_H_

#include <stdint.h>

/**
 * @mainpage STM32 eXtensible Peripheral Drivers
 *
 * @section introduction Introduction
 *
 * This peripheral driver package was created as a functional upgrade and simplification of
 * the CMSIS and STM32 HAL Drivers, as they were not prepared to handle register bit operations
 * effectively.
 * Below is a short overview of the changes.
 *
 * @section features Features
 *
 * @subsection feat1 Peripheral structures with bit fields
 *
 * All peripheral mapping structures are expanded with the bit maps of each register
 * using a custom-made code generator. The new core and device headers are incompatible with the
 * old drivers, hence the necessity for new peripheral driver layer development.
 *
 * Keep in mind that there are not enough resources available to review the product of the generator
 * (namely the bits of all peripheral registers) so please precaution is required when using peripherals.
 * This is especially applicable for peripherals which don't have their driver code published yet.
 *
 * @subsection feat2 Native support of bit-band alias peripheral memory access
 *
 * In addition to the regular bit field mapping, the code generator creates peripheral
 * access structures for their bit-band alias access (if any peripheral instance of that type
 * is located in the bit-band mappable memory region).
 *
 * For single instance peripheral types a single bit-band access macro is defined.
 * For peripheral types with multiple instances a parametric macro is created.
 * The drivers add a separate pointer for the alias address to the peripheral handles, making the
 * alias address computation a single-time effort.
 *
 * @subsection feat3 Handle-specific callback architecture
 *
 * In the HAL drivers the callback functions were single instance functions that had to
 * be overwritten to use them. The biggest drawback was that it needed routing when multiple
 * peripherals were used from the same kind. In this driver all callback functions are contained
 * by the handles, thus giving the opportunity to create unique functions for each instance.
 * For compatibility and design flexibility the handle pointers are still provided as input
 * parameters for the callback functions.
 *
 * @subsection feat4 Greatly reduced code symbols amount
 *
 * Due to the creation of the register bit fields it is possible to use universal enumerations
 * for all configurations, as the bit shifting is either provided by the compiler, or it is
 * unnecessary (if bit-banding is used). This results in simpler API usage.
 *
 */

/** @defgroup Common
 * @{ */

/** @defgroup Common_Exported_Types Common Exported Types
 * @{ */

/** @brief Boolean type */
typedef enum
{
  FALSE = 0, /*!< False value */
  TRUE  = 1  /*!< True value */
} bool;

/** @brief Flag status type */
typedef enum
{
  RESET = 0, /*!< Reset value */
  SET   = 1  /*!< Set value */
} FlagStatus;

/** @brief Functional state type */
typedef enum
{
  DISABLE = 0, /*!< Disable value */
  ENABLE  = 1  /*!< Enable value */
} FunctionalState;

/** @brief XPD functions return type */
typedef enum
{
  XPD_OK       = 0, /*!< Operation finished successfully */
  XPD_ERROR    = 1, /*!< Operation encountered an error */
  XPD_BUSY     = 2, /*!< Operation exited because it was already in progress */
  XPD_TIMEOUT  = 3  /*!< Operation timed out */
} XPD_ReturnType;

/** @brief Occurrence reaction type */
typedef enum
{
    REACTION_NONE  = 0, /*!< No reaction */
    REACTION_IT    = 1, /*!< Interrupt generation reaction */
    REACTION_EVENT = 2  /*!< Event generation reaction */
}ReactionType;

/** @brief Signal active level type */
typedef enum
{
    ACTIVE_LOW  = 0, /*!< Signal is active low */
    ACTIVE_HIGH = 1  /*!< Signal is active high */
}ActiveLevelType;

/** @brief General level type */
typedef enum
{
    LOW       = 0, /*!< Low level is selected */
    MEDIUM    = 1, /*!< Medium level is selected */
    HIGH      = 2, /*!< High level is selected */
    VERY_HIGH = 3  /*!< Very high level is selected */
}LevelType;

/** @brief Signal edge type */
typedef enum
{
    EDGE_NONE           = 0, /*!< No edge is selected */
    EDGE_RISING         = 1, /*!< Rising edge is selected */
    EDGE_FALLING        = 2, /*!< Falling edge is selected */
    EDGE_RISING_FALLING = 3  /*!< Rising and falling edges are selected */
}EdgeType;

/** @brief Clock division type */
typedef enum
{
    CLK_DIV1   = 0, /*!< Clock is not divided */
    CLK_DIV2   = 1, /*!< Clock is divided by 2 */
    CLK_DIV4   = 2, /*!< Clock is divided by 4 */
    CLK_DIV8   = 3, /*!< Clock is divided by 8 */
    CLK_DIV16  = 4, /*!< Clock is divided by 16 */
    CLK_DIV32  = 5, /*!< Clock is divided by 32 */
    CLK_DIV64  = 6, /*!< Clock is divided by 64 */
    CLK_DIV128 = 7, /*!< Clock is divided by 128 */
    CLK_DIV256 = 8, /*!< Clock is divided by 256 */
    CLK_DIV512 = 9  /*!< Clock is divided by 512 */
}ClockDividerType;

/**
 * @brief Callback function pointer type with no parameters
 */
typedef void ( *XPD_SimpleCallbackType )    ( void );

/**
 * @brief Callback function pointer type with numeric parameter
 * @param Value: provided data for the callback
 */
typedef void ( *XPD_ValueCallbackType )    ( uint32_t Value );

/**
 * @brief Callback function pointer type with handle pointer parameter
 * @param Handle: pointer of the callback sender handle
 */
typedef void ( *XPD_HandleCallbackType )    ( void * Handle );

/** @} */

/** @defgroup Common_Exported_Macros Common Exported Macros
 * @{ */

#ifndef NULL
#define NULL      ((void *)0)
#endif

/**
 * @brief  Safe function pointer caller that checks it against NULL and calls it with parameters.
 * @param  CALLBACK: the function pointer
 * @param  PARAMETERS: the required parameters of the function
 */
#define XPD_SAFE_CALLBACK(CALLBACK, PARAMETERS)      \
    do{ if ((CALLBACK) != NULL) CALLBACK(PARAMETERS); }while(0)

/** @} */

/** @} */

/* Inherited macros */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#if  defined ( __GNUC__ )
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */

/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined   (__GNUC__)        /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#else
#ifndef __ALIGN_END
#define __ALIGN_END
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#if defined   (__CC_ARM)      /* ARM Compiler */
#define __ALIGN_BEGIN    __align(4)
#elif defined (__ICCARM__)    /* IAR Compiler */
#define __ALIGN_BEGIN
#endif /* __CC_ARM */
#endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */


#endif /* XPD_COMMON_H_ */
