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
 * @mainpage STM32 eXtensible Peripheral Drivers for STM32F4 device family
 *
 * This documentation is part of the STM32_XPD peripheral driver package, available
 * <a href="http://github.com/IntergatedCircuits/STM32_XPD">here</a>.
 *
 * STM32_XPD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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
 * @brief Function pointer type for binary control function reference
 * @param NewState: the state to set
 */
typedef void ( *XPD_CtrlFnType )            ( FunctionalState NewState );

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

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  ((REG) = (((REG) & (~(CLEARMASK))) | ((SETMASK) & (CLEARMASK))))

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
