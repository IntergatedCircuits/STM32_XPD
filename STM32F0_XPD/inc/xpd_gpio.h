/**
  ******************************************************************************
  * @file    xpd_gpio.h
  * @author  Benedek Kupper
  * @version 0.4
  * @date    2019-02-13
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
}GPIO_ModeType;

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

/** @brief GPIO pins */
typedef enum
{
#ifdef GPIOA
    PA0 = 0x00, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
#endif
#ifdef GPIOB
    PB0 = 0x10, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
#endif
#ifdef GPIOC
    PC0 = 0x20, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
#endif
#ifdef GPIOD
    PD0 = 0x30, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
#endif
#ifdef GPIOE
    PE0 = 0x40, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
#endif
#ifdef GPIOF
    PF0 = 0x50, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
#endif
#ifdef GPIOG
    PG0 = 0x60, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
#endif
#ifdef GPIOH
    PH0 = 0x70, PH1, PH2, PH3, PH4, PH5, PH6, PH7, PH8, PH9, PH10, PH11, PH12, PH13, PH14, PH15,
#endif
#ifdef GPIOI
    PI0 = 0x80, PI1, PI2, PI3, PI4, PI5, PI6, PI7, PI8, PI9, PI10, PI11, PI12, PI13, PI14, PI15,
#endif
#ifdef GPIOJ
    PJ0 = 0x90, PJ1, PJ2, PJ3, PJ4, PJ5, PJ6, PJ7, PJ8, PJ9, PJ10, PJ11, PJ12, PJ13, PJ14, PJ15,
#endif
#ifdef GPIOK
    PK0 = 0xA0, PK1, PK2, PK3, PK4, PK5, PK6, PK7, PK8, PK9, PK10, PK11, PK12, PK13, PK14, PK15,
#endif
}GPIO_PinType;

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

#define __GPIO_PIN_BITS             4
#define __GPIO_PIN_MASK             ((1 << __GPIO_PIN_BITS) - 1)

#define __GPIO_PORT_FROM_PIN(PIN)   \
    ((GPIO_TypeDef*)(GPIOA_BASE + (((uint32_t)(PIN) & (~__GPIO_PIN_MASK)) << (10 - __GPIO_PIN_BITS))))

/** @addtogroup GPIO_Exported_Functions
 * @{ */

/** @addtogroup GPIO_Exported_Functions_Port
 * @{ */
void            GPIO_vInitPort      (GPIO_TypeDef * pxGPIO, const GPIO_InitType * pxConfig);
/** @} */

/** @addtogroup GPIO_Exported_Functions_Pin
 * @{ */
void            GPIO_vInitPin       (GPIO_PinType ePin, const GPIO_InitType * pxConfig);
void            GPIO_vDeinitPin     (GPIO_PinType ePin);
void            GPIO_vLockPin       (GPIO_PinType ePin);
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
 * @param ePin: selected pin
 * @param eValue: the new output value
 */
__STATIC_INLINE void GPIO_vWritePin(GPIO_PinType ePin, FlagStatus eValue)
{
    GPIO_TypeDef *pxGPIO = __GPIO_PORT_FROM_PIN(ePin);
    uint8_t ucPin = ePin & __GPIO_PIN_MASK;
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
 * @param ePin: selected pin
 * @return The value of the pin
 */
__STATIC_INLINE FlagStatus GPIO_eReadPin(GPIO_PinType ePin)
{
    GPIO_TypeDef *pxGPIO = __GPIO_PORT_FROM_PIN(ePin);
    uint8_t ucPin = ePin & __GPIO_PIN_MASK;
#ifdef GPIO_BB
    return GPIO_BB(pxGPIO)->IDR[ucPin];
#else
    return (pxGPIO->IDR >> ucPin) & 1;
#endif
}

/**
 * @brief Toggles the selected output pin.
 * @param ePin: selected pin
 */
__STATIC_INLINE void GPIO_vTogglePin(GPIO_PinType ePin)
{
    GPIO_TypeDef *pxGPIO = __GPIO_PORT_FROM_PIN(ePin);
    uint8_t ucPin = ePin & __GPIO_PIN_MASK;
#ifdef GPIO_BB
    GPIO_BB(pxGPIO)->ODR[ucPin]++;
#else
    if (((pxGPIO->ODR >> ucPin) & 1) != 0)
    {   ucPin += 16; }
    pxGPIO->BSRR = 1 << ucPin;
#endif
}

/**
 * @brief Returns the reference to the pin's EXTI callback variable.
 * @param ePin: selected pin
 * @return Reference of the pin's EXTI callback
 */
__STATIC_INLINE XPD_ValueCallbackType * GPIO_pxPinCallback(GPIO_PinType ePin)
{
    return &EXTI_xPinCallbacks[ePin & __GPIO_PIN_MASK];
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
