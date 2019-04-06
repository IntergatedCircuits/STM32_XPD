/**
  ******************************************************************************
  * @file    xpd_gpio.c
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
#include <xpd_gpio.h>
#include <xpd_rcc.h>

#define GPIO_PORT_OFFSET(GPIOX) \
    (((uint32_t)(GPIOX) - (uint32_t)GPIOA_BASE) >> 10)

#ifdef PWR_CR3_APC
static struct {
    __IO uint32_t PUCR;
    __IO uint32_t PDCR;
} * const pwr_pxPullConfig = (void*)&PWR->PUCRA;

#ifdef PWR_BB
static struct {
    __IO uint32_t PUC[32];
    __IO uint32_t PDC[32];
} * const pwr_pxPullPinConfig = (void*)PERIPH_BB(&PWR->PUCRA);
#endif
#endif

/** @defgroup GPIO
 * @{ */

static void GPIO_prvClockEnable(GPIO_TypeDef * pxGPIO)
{
#if defined(GPIOH) && (RCC_POS_GPIOH < RCC_POS_GPIOA)
    if (pxGPIO == GPIOH)
    {
        RCC_vClockEnable(RCC_POS_GPIOH);
    }
    else
#endif
    {
        RCC_vClockEnable(RCC_POS_GPIOA + GPIO_PORT_OFFSET(pxGPIO));
    }
}

/** @defgroup GPIO_Exported_Functions GPIO Exported Functions
 * @{ */

/** @defgroup GPIO_Exported_Functions_Port GPIO Port Management Functions
 *  @brief    GPIO port initialization, collective I/O control
 * @{
 */

/**
 * @brief Initializes the GPIO port based on setup structure.
 * @param pxGPIO: pointer to the GPIO peripheral
 * @param pxConfig: pointer to the setup parameters
 */
void GPIO_vInitPort(GPIO_TypeDef * pxGPIO, const GPIO_InitType * pxConfig)
{
    uint32_t ulTmp;
    uint32_t ulAFR = 0;

    /* enable GPIO clock */
    GPIO_prvClockEnable(pxGPIO);

#ifdef PWR_CR3_APC
    switch (pxConfig->PowerDownPull)
    {
        case GPIO_PULL_UP:
            pwr_pxPullConfig[GPIO_PORT_OFFSET(pxGPIO)].PUCR = 0xFFFF;
            pwr_pxPullConfig[GPIO_PORT_OFFSET(pxGPIO)].PDCR = 0;
            break;
        case GPIO_PULL_DOWN:
            pwr_pxPullConfig[GPIO_PORT_OFFSET(pxGPIO)].PUCR = 0;
            pwr_pxPullConfig[GPIO_PORT_OFFSET(pxGPIO)].PDCR = 0xFFFF;
            break;
        case GPIO_PULL_FLOAT:
        default:
            pwr_pxPullConfig[GPIO_PORT_OFFSET(pxGPIO)].PUCR = 0;
            pwr_pxPullConfig[GPIO_PORT_OFFSET(pxGPIO)].PDCR = 0;
            break;
    }
#endif

    /* alternate function mapping */
    if ((pxConfig->Mode == GPIO_MODE_ALTERNATE) && (pxConfig->AlternateMap != 0))
    {
        for (ulTmp = pxConfig->AlternateMap; ulTmp != 0; ulTmp <<= 4)
        {
            ulAFR |= ulTmp;
        }
        pxGPIO->AFR[0] = ulAFR;
        pxGPIO->AFR[1] = ulAFR;
    }
    else
    {
        pxGPIO->AFR[0] = 0;
        pxGPIO->AFR[1] = 0;
    }

    /* IO direction setup */
    switch (pxConfig->Mode)
    {
        case GPIO_MODE_OUTPUT:
            pxGPIO->MODER.w = 0x55555555;
            break;
        case GPIO_MODE_ALTERNATE:
            pxGPIO->MODER.w = 0xAAAAAAAA;
            break;
        case GPIO_MODE_ANALOG:
            pxGPIO->MODER.w = 0xFFFFFFFF;
            break;
        case GPIO_MODE_INPUT:
        case GPIO_MODE_EXTI:
        default:
            pxGPIO->MODER.w = 0;
            break;
    }

#ifdef GPIO_ASCR_ASC0
    /* Set the ADC connection bit if GPIO_ADC_AF is selected */
    if (pxConfig->AlternateMap == GPIO_ADC_AF)
    {
        pxGPIO->ASCR = 0xFFFF;
    }
    else
    {
        pxGPIO->ASCR = 0x0000;
    }
#endif

    /* output stage configuration */
    if ((pxConfig->Mode == GPIO_MODE_OUTPUT) || (pxConfig->Mode == GPIO_MODE_ALTERNATE))
    {
        /* speed */
        switch (pxConfig->Output.Speed)
        {
            case MEDIUM:
                pxGPIO->OSPEEDR.w = 0x55555555;
                break;
            case HIGH:
                pxGPIO->OSPEEDR.w = 0xAAAAAAAA;
                break;
            case VERY_HIGH:
                pxGPIO->OSPEEDR.w = 0xFFFFFFFF;
                break;
            case LOW:
            default:
                pxGPIO->OSPEEDR.w = 0;
                break;
        }

        /* type */
        if (pxConfig->Output.Type == GPIO_OUTPUT_OPENDRAIN)
            pxGPIO->OTYPER = 0xFFFF;
        else
            pxGPIO->OTYPER = 0;
    }

    /* pull */
    switch (pxConfig->Pull)
    {
        case GPIO_PULL_UP:
            pxGPIO->PUPDR.w = 0x55555555;
            break;
        case GPIO_PULL_DOWN:
            pxGPIO->PUPDR.w = 0xAAAAAAAA;
            break;
        case GPIO_PULL_FLOAT:
        default:
            pxGPIO->PUPDR.w = 0;
            break;
    }

    /* EXTI mode configuration is left out */
}

/** @} */

/** @defgroup GPIO_Exported_Functions_Pin GPIO Pin Handling Functions
 *  @brief    GPIO pin initialization, individual I/O control
 * @{
 */

/**
 * @brief Initializes the GPIO pin based on setup structure.
 * @param ePin: selected pin
 * @param pxConfig: pointer to the setup parameters
 */
void GPIO_vInitPin(GPIO_PinType ePin, const GPIO_InitType * pxConfig)
{
    GPIO_TypeDef *pxGPIO = __GPIO_PORT_FROM_PIN(ePin);
    uint8_t ucPin = ePin & __GPIO_PIN_MASK;
    uint8_t uc2BitPos = ucPin * 2;

    /* enable GPIO clock */
    GPIO_prvClockEnable(pxGPIO);

#ifdef PWR_CR3_APC
    /* configure pull direction in power down */
#ifdef PWR_BB
    pwr_pxPullPinConfig[ePin >> __GPIO_PIN_BITS].PUC[ucPin] = pxConfig->PowerDownPull;
    pwr_pxPullPinConfig[ePin >> __GPIO_PIN_BITS].PDC[ucPin] = pxConfig->PowerDownPull >> 1;
#else
    MODIFY_REG(pwr_pxPullConfig[ePin >> __GPIO_PIN_BITS].PUCR, 1 << ucPin,
                               (uin32_t)pxConfig->PowerDownPull  << ucPin);
    MODIFY_REG(pwr_pxPullConfig[ePin >> __GPIO_PIN_BITS].PDCR, 1 << ucPin,
                         ((uin32_t)pxConfig->PowerDownPull >> 1) << ucPin);
#endif
#endif

    /* alternate mapping */
    if (pxConfig->Mode == GPIO_MODE_ALTERNATE)
    {
        uint8_t ucAFPos = (ucPin & 0x7) << 2;

        MODIFY_REG(pxGPIO->AFR[ucPin >> 3], 0xF << ucAFPos,
                         pxConfig->AlternateMap << ucAFPos);
    }

    /* IO mode */
    MODIFY_REG(pxGPIO->MODER.w, 3 << uc2BitPos,
                   pxConfig->Mode << uc2BitPos);

#ifdef GPIO_ASCR_ASC0
    /* Set the ADC connection bit if GPIO_ADC_AF is selected */
    MODIFY_REG(pxGPIO->ASCR, 1 << ucPin, (pxConfig->AlternateMap >> 4) << ucPin);
#endif

    /* output stage configuration */
    if (    (pxConfig->Mode == GPIO_MODE_OUTPUT)
         || (pxConfig->Mode == GPIO_MODE_ALTERNATE))
    {
        /* speed */
        MODIFY_REG(pxGPIO->OSPEEDR.w, 3 << uc2BitPos,
                 pxConfig->Output.Speed << uc2BitPos);

        /* type */
        MODIFY_REG(pxGPIO->OTYPER, 1 << ucPin,
               pxConfig->Output.Type << ucPin);
    }

    /* Activate the Pull-up or Pull down resistor for the current IO */
    MODIFY_REG(pxGPIO->PUPDR.w, 3 << uc2BitPos,
                   pxConfig->Pull << uc2BitPos);

    /* EXTI configuration */
    if (pxConfig->Mode == GPIO_MODE_EXTI)
    {
        /* set EXTI source */
        uint8_t ucPortPos = (ucPin & 0x03) << 2;

        MODIFY_REG(SYSCFG->EXTICR[ucPin >> 2], 0xF << ucPortPos,
                         (ePin >> __GPIO_PIN_BITS) << ucPortPos);

        /* hand over EXTI configuration */
        EXTI_vInit(ucPin, &pxConfig->ExtI);
    }
}

/**
 * @brief Restores the GPIO pin to the default settings.
 * @param ePin: selected pin
 */
void GPIO_vDeinitPin(GPIO_PinType ePin)
{
    GPIO_TypeDef *pxGPIO = __GPIO_PORT_FROM_PIN(ePin);
    uint8_t ucPin = ePin & __GPIO_PIN_MASK;
#ifdef PWR_CR3_APC
    /* configure pull direction in power down */
#ifdef PWR_BB
    pwr_pxPullPinConfig[ePin >> __GPIO_PIN_BITS].PUC[ucPin] = 0;
    pwr_pxPullPinConfig[ePin >> __GPIO_PIN_BITS].PDC[ucPin] = 0;
#else
    CLEAR_BIT(pwr_pxPullConfig[ePin >> __GPIO_PIN_BITS].PUCR, 1 << ucPin);
    CLEAR_BIT(pwr_pxPullConfig[ePin >> __GPIO_PIN_BITS].PDCR, 1 << ucPin);
#endif
#endif

    /* GPIO Mode Configuration */
    /* Configure IO Direction in Input Floating Mode */
    CLEAR_BIT(pxGPIO->MODER.w, (3 << (ucPin * 2)));

    /* Deactivate the Pull-up and Pull-down resistor for the current IO */
    CLEAR_BIT(pxGPIO->PUPDR.w, (3 << (ucPin * 2)));

    /* Configure the External Interrupt or event for the current IO */
    CLEAR_BIT(SYSCFG->EXTICR[ucPin >> 2], ((uint32_t)0xF) << (4 * (ucPin & 3)));

    EXTI_vDeinit(ucPin);
}

/**
 * @brief Locks the pin's configuration until the next device reset
 * @param ePin: selected pin
 */
void GPIO_vLockPin(GPIO_PinType ePin)
{
    GPIO_TypeDef *pxGPIO = __GPIO_PORT_FROM_PIN(ePin);
    uint8_t ucPin = ePin & __GPIO_PIN_MASK;
    uint32_t ulPinMask = 1 << (uint32_t)ucPin;
    uint32_t ulPinLock = GPIO_LCKR_LCKK | ulPinMask;

    if ((pxGPIO->LCKR.w & ulPinMask) == 0)
    {
        /* Apply lock key write sequence */
        /* LCKK='1' + LCK[x] */
        pxGPIO->LCKR.w = ulPinLock;
        /* LCKK='0' + LCK[x] */
        pxGPIO->LCKR.w = ulPinMask;
        /* LCKK='1' + LCK[x] */
        pxGPIO->LCKR.w = ulPinLock;

        /* read register */
        (void) pxGPIO->LCKR.w;
    }
}

/** @} */

/** @} */

/** @} */
