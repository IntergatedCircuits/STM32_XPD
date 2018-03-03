/**
  ******************************************************************************
  * @file    xpd_gpio.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers General Purpose I/O Module
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
#ifndef __XPD_GPIO_H_
#define __XPD_GPIO_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>
#include <xpd_exti.h>

/** @addtogroup GPIO_Alternate_function_map
 * @{ */
#define GPIO_ADC_AF ((uint8_t)0x10) /*!< Use this macro with ANALOG mode to set the ASC bit (connect to ADC) */
/** @} */

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

#define GPIO_xPinCallbacks          EXTI_xPinCallbacks

/** @addtogroup GPIO_Exported_Functions
 * @{ */

/** @addtogroup GPIO_Exported_Functions_Port
 * @{ */
void            GPIO_vInitPort      (GPIO_TypeDef * pxGPIO, const GPIO_InitType * pxConfig);
/** @} */

/** @addtogroup GPIO_Exported_Functions_Pin
 * @{ */
void            GPIO_vInitPin       (GPIO_TypeDef * pxGPIO, uint8_t ucPin, const GPIO_InitType * pxConfig);
void            GPIO_vDeinitPin     (GPIO_TypeDef * pxGPIO, uint8_t ucPin);
void            GPIO_vLockPin       (GPIO_TypeDef * pxGPIO, uint8_t ucPin);
/** @} */

/** @addtogroup GPIO_Exported_Functions_Port
 * @{ */
/**
 * @brief Reads the input state of the GPIO port.
 * @param pxGPIO: pointer to the GPIO peripheral
 * @return The input state of the port
 */
__STATIC_INLINE uint16_t GPIO_usReadPort(GPIO_TypeDef * pxGPIO)
{
    return pxGPIO->IDR;
}

/**
 * @brief Writes the outputs of the GPIO port.
 * @param pxGPIO: pointer to the GPIO peripheral
 * @param Value: the new output state of the port
 */
__STATIC_INLINE void GPIO_vWritePort(GPIO_TypeDef * pxGPIO, uint16_t Value)
{
    pxGPIO->ODR = Value;
}

/**
 * @brief Sets and resets the selected output pins of the GPIO port.
 * @param pxGPIO: pointer to the GPIO peripheral
 * @param SetBits: the bit positions specify which outputs to set
 * @param ResetBits: the bit positions specify which outputs to reset
 */
__STATIC_INLINE void GPIO_vSetResetPort(GPIO_TypeDef * pxGPIO, uint16_t SetBits, uint16_t ResetBits)
{
    pxGPIO->BSRR = SetBits | (ResetBits << 16);
}

/** @} */

/** @addtogroup GPIO_Exported_Functions_Pin
 * @{ */

/**
 * @brief Sets the output pin to the selected value.
 * @param pxGPIO: pointer to the GPIO peripheral
 * @param ucPin: selected pin of the port [0 .. 15]
 * @param eValue: the new output value
 */
__STATIC_INLINE void GPIO_vWritePin(GPIO_TypeDef * pxGPIO, uint8_t ucPin, FlagStatus eValue)
{
#ifdef GPIO_BB
    GPIO_BB(pxGPIO)->ODR[ucPin] = eValue;
#else
    if (eValue == 0)
    {
        pxGPIO->BSRR = 0x10000 << ucPin;
    }
    else
    {
        pxGPIO->BSRR = 1 << ucPin;
    }
#endif
}

/**
 * @brief Reads the selected input pin.
 * @param pxGPIO: pointer to the GPIO peripheral
 * @param ucPin: selected pin of the port [0 .. 15]
 * @return The value of the pin
 */
__STATIC_INLINE FlagStatus GPIO_eReadPin(GPIO_TypeDef * pxGPIO, uint8_t ucPin)
{
#ifdef GPIO_BB
    return GPIO_BB(pxGPIO)->IDR[ucPin];
#else
    return (pxGPIO->IDR >> ucPin) & 1;
#endif
}

/**
 * @brief Toggles the selected output pin.
 * @param pxGPIO: pointer to the GPIO peripheral
 * @param ucPin: selected pin of the port [0 .. 15]
 */
__STATIC_INLINE void GPIO_vTogglePin(GPIO_TypeDef * pxGPIO, uint8_t ucPin)
{
#ifdef GPIO_BB
    GPIO_BB(pxGPIO)->ODR[ucPin]++;
#else
    pxGPIO->ODR ^= 1 << ucPin;
#endif
}

/** @} */

/** @} */

/** @} */

#define XPD_GPIO_API
#include <xpd_syscfg.h>
#undef XPD_GPIO_API

#ifdef __cplusplus
}
#endif

#endif /* __XPD_GPIO_H_ */
