/**
  ******************************************************************************
  * @file    xpd_gpio.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-04-10
  * @brief   STM32 eXtensible Peripheral Drivers General Purpose I/O Module
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
#ifndef XPD_GPIO_H_
#define XPD_GPIO_H_

#include "xpd_common.h"
#include "xpd_config.h"
#include "xpd_exti.h"

/** @defgroup GPIO
 * @{ */

/** @defgroup GPIO_Exported_Types GPIO Exported Types
 * @{ */

/** @brief GPIO modes */
typedef enum
{
    GPIO_MODE_INPUT     = 0,   /*!< Input mode */
    GPIO_MODE_OUTPUT    = 1,   /*!< Output mode */
    GPIO_MODE_ALTERNATE = 2,   /*!< Alternate function mapped mode */
    GPIO_MODE_ANALOG    = 3,   /*!< Analog mode */
    GPIO_MODE_EXTI      = 0x10 /*!< Input mode with external interrupt/event generation */
} GPIO_ModeType;

/** @brief GPIO output stage types */
typedef enum
{
    GPIO_OUTPUT_PUSHPULL  = 0, /*!< Push-pull output stage */
    GPIO_OUTPUT_OPENDRAIN = 1  /*!< Open-drain output stage */
}GPIO_OutputType;

/** @brief GPIO pull-up/down resistor options */
typedef enum
{
    GPIO_PULL_FLOAT = 0, /*!< Floating idle state */
    GPIO_PULL_UP    = 1, /*!< High idle state (pull-up) */
    GPIO_PULL_DOWN  = 2  /*!< Low idle state (pull-down) */
}GPIO_PullType;

/** @brief GPIO setup structure */
typedef struct
{
    GPIO_ModeType Mode;         /*!< GPIO mode */
    GPIO_PullType Pull;         /*!< GPIO resistor pull direction */
    struct {
        GPIO_OutputType Type;   /*!< GPIO output stage type */
        LevelType       Speed;  /*!< GPIO output switching speed */
    }Output;
    EXTI_InitType ExtI;         /*!< EXTI configuration if this mode is selected */
    uint8_t       AlternateMap; /*!< Mapping to alternate function @ref GPIO_Alternate_function_map */
#ifdef PWR_CR3_APC
    GPIO_PullType PowerDownPull;/*!< GPIO resistor pull direction during Standby or Shutdown mode
                                     @note This configuration is globally controlled by PWR/CR3/APC bit */
#endif
}GPIO_InitType;

/** @} */

/** @addtogroup GPIO_Alternate_function_map
 * @{ */
#define GPIO_ADC_AF ((uint8_t)0x10) /*!< Use this macro with ANALOG mode to set the ASC bit (connect to ADC) */
/** @} */

/** @addtogroup GPIO_Exported_Functions
 * @{ */

/** @addtogroup GPIO_Exported_Functions_Port
 * @{ */
void            XPD_GPIO_InitPort   (GPIO_TypeDef * GPIOx, const GPIO_InitType * Config);
/** @} */

/** @addtogroup GPIO_Exported_Functions_Pin
 * @{ */
void            XPD_GPIO_InitPin    (GPIO_TypeDef * GPIOx, uint8_t Pin, const GPIO_InitType * Config);
void            XPD_GPIO_DeinitPin  (GPIO_TypeDef * GPIOx, uint8_t Pin);
void            XPD_GPIO_LockPin    (GPIO_TypeDef * GPIOx, uint8_t Pin);
/** @} */

/** @addtogroup GPIO_Exported_Functions_Port
 * @{ */
/**
 * @brief Reads the input state of the GPIO port.
 * @param GPIOx: pointer to the GPIO peripheral
 * @return The input state of the port
 */
__STATIC_INLINE uint16_t XPD_GPIO_ReadPort(GPIO_TypeDef * GPIOx)
{
    return GPIOx->IDR;
}

/**
 * @brief Writes the outputs of the GPIO port.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Value: the new output state of the port
 */
__STATIC_INLINE void XPD_GPIO_WritePort(GPIO_TypeDef * GPIOx, uint16_t Value)
{
    GPIOx->ODR = Value;
}

/**
 * @brief Sets and resets the selected output pins of the GPIO port.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param SetBits: the bit positions specify which outputs to set
 * @param ResetBits: the bit positions specify which outputs to reset
 */
__STATIC_INLINE void XPD_GPIO_SetResetPort(GPIO_TypeDef * GPIOx, uint16_t SetBits, uint16_t ResetBits)
{
    GPIOx->BSRR = SetBits | (ResetBits << 16);
}

/** @} */

/** @addtogroup GPIO_Exported_Functions_Pin
 * @{ */

/**
 * @brief Sets the output pin to the selected value.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Pin: selected pin of the port [0 .. 15]
 * @param Value: the new output value
 */
__STATIC_INLINE void XPD_GPIO_WritePin(GPIO_TypeDef * GPIOx, uint8_t Pin, uint8_t Value)
{
#ifdef GPIO_BB
    GPIO_BB(GPIOx)->ODR[Pin] = Value;
#else
    if (Value == 0)
    {
        GPIOx->BSRR = 0x10000 << (uint32_t)Pin;
    }
    else
    {
        GPIOx->BSRR = 1 << (uint32_t)Pin;
    }
#endif
}

/**
 * @brief Reads the selected input pin.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Pin: selected pin of the port [0 .. 15]
 * @return The value of the pin
 */
__STATIC_INLINE uint8_t XPD_GPIO_ReadPin(GPIO_TypeDef * GPIOx, uint8_t Pin)
{
#ifdef GPIO_BB
    return GPIO_BB(GPIOx)->IDR[Pin];
#else
    return (GPIOx->IDR >> (uint32_t)Pin) & 1;
#endif
}

/**
 * @brief Toggles the selected output pin.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Pin: selected pin of the port [0 .. 15]
 */
__STATIC_INLINE void XPD_GPIO_TogglePin(GPIO_TypeDef * GPIOx, uint8_t Pin)
{
#ifdef GPIO_BB
    GPIO_BB(GPIOx)->ODR[Pin]++;
#else
    GPIOx->ODR ^= 1 << (uint32_t)Pin;
#endif
}

/** @} */

/** @} */

/** @} */

#endif /* XPD_GPIO_H_ */
