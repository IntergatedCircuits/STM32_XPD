/**
  ******************************************************************************
  * @file    stm32f3_gpio.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2015-12-30
  * @brief   STM32 eXtensible Peripheral Drivers GPIO Module
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
#include "xpd_gpio.h"
#include "xpd_rcc.h"
#include "xpd_utils.h"

#define GET_GPIO_SOURCE(__GPIOx__) (((uint32_t)(__GPIOx__) - (uint32_t)GPIOA_BASE) >> 10)

/** @defgroup GPIO
 * @{ */

/** @defgroup GPIO_Exported_Functions GPIO Exported Functions
 * @{ */

/** @defgroup GPIO_Exported_Functions_Port GPIO Port Management Functions
 *  @brief    GPIO port initialization, collective I/O control
 * @{
 */

/**
 * @brief Initializes the GPIO port based on setup structure.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Config: pointer to the setup parameters
 */
void XPD_GPIO_InitPort(GPIO_TypeDef *GPIOx, GPIO_InitType *Config)
{
    uint32_t temp = 0;
    uint32_t reg;

    /* alternate function mapping */
    GPIOx->AFR[0] = 0;
    GPIOx->AFR[1] = 0;
    if ((Config->Mode == GPIO_MODE_ALTERNATE) && (Config->AlternateMap != 0))
    {
        for (temp = Config->AlternateMap; temp != 0; temp <<= 4)
        {
            reg |= temp;
        }
        GPIOx->AFR[0] = reg;
        GPIOx->AFR[1] = reg;
    }

    /* IO direction setup */
    switch (Config->Mode)
    {
        case GPIO_MODE_INPUT:
        case GPIO_MODE_EXTI:
        default:
            GPIOx->MODER = 0;
            break;
        case GPIO_MODE_OUTPUT:
            GPIOx->MODER = 0x55555555;
            break;
        case GPIO_MODE_ALTERNATE:
            GPIOx->MODER = 0xAAAAAAAA;
            break;
        case GPIO_MODE_ANALOG:
            GPIOx->MODER = 0xFFFFFFFF;
            break;
    }

    /* output stage configuration */
    if ((Config->Mode == GPIO_MODE_OUTPUT) || (Config->Mode == GPIO_MODE_ALTERNATE))
    {
        /* speed */
        switch (Config->Output.Speed)
        {
            case LOW:
            default:
                GPIOx->OSPEEDR = 0;
                break;
            case MEDIUM:
                GPIOx->OSPEEDR = 0x55555555;
                break;
            case HIGH:
                GPIOx->OSPEEDR = 0xAAAAAAAA;
                break;
            case VERY_HIGH:
                GPIOx->OSPEEDR = 0xFFFFFFFF;
                break;
        }

        /* type */
        if (Config->Output.Type == GPIO_OUTPUT_OPENDRAIN)
            GPIOx->OTYPER = 0xFFFF;
        else
            GPIOx->OTYPER = 0;
    }

    /* pull */
    switch (Config->Pull)
    {
        case GPIO_PULL_FLOAT:
        default:
            GPIOx->PUPDR = 0;
            break;
        case GPIO_PULL_UP:
            GPIOx->PUPDR = 0x55555555;
            break;
        case GPIO_PULL_DOWN:
            GPIOx->PUPDR = 0xAAAAAAAA;
            break;
    }

    /* EXTI mode configuration */
    if (Config->Mode == GPIO_MODE_EXTI)
    {
        /* enable SYSCFG clock */
        XPD_SYSCFG_EnableClock();

        /* bind callbacks */
        for (temp = 0; temp < 16; temp++)
        {
            XPD_EXTI_Callbacks[temp] = Config->ExtI.ITCallback;
        }

        /* set EXTI source */
        temp = GET_GPIO_SOURCE(GPIOx);
        temp |= temp << 4;
        temp |= temp << 4;
        temp |= temp << 4;
        SYSCFG->EXTICR[0] = temp;
        SYSCFG->EXTICR[1] = temp;
        SYSCFG->EXTICR[2] = temp;
        SYSCFG->EXTICR[3] = temp;

        /* set reaction and edge */
        if (Config->ExtI.Reaction & REACTION_IT)
        {
            SET_BIT(EXTI->IMR, 0xFFFF);
        }
        else
        {
            CLEAR_BIT(EXTI->IMR, 0xFFFF);
        }

        if (Config->ExtI.Reaction & REACTION_EVENT)
        {
            SET_BIT(EXTI->EMR, 0xFFFF);
        }
        else
        {
            CLEAR_BIT(EXTI->EMR, 0xFFFF);
        }

        if (Config->ExtI.Edge & EDGE_RISING)
        {
            SET_BIT(EXTI->RTSR, 0xFFFF);
        }
        else
        {
            CLEAR_BIT(EXTI->RTSR, 0xFFFF);
        }

        if (Config->ExtI.Edge & EDGE_FALLING)
        {
            SET_BIT(EXTI->FTSR, 0xFFFF);
        }
        else
        {
            CLEAR_BIT(EXTI->FTSR, 0xFFFF);
        }
    }
}

/** @} */

/** @defgroup GPIO_Exported_Functions_Pin GPIO Pin Handling Functions
 *  @brief    GPIO pin initialization, individual I/O control
 * @{
 */

/**
 * @brief Initializes the GPIO pin based on setup structure.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Pin: selected pin of the port [0 .. 15]
 * @param Config: pointer to the setup parameters
 */
void XPD_GPIO_InitPin(GPIO_TypeDef * GPIOx, uint8_t Pin, GPIO_InitType * Config)
{
    uint32_t temp;
    uint32_t shifter;

    (uint32_t) Pin;

    /* alternate mapping */
    if (Config->Mode == GPIO_MODE_ALTERNATE)
    {
        shifter = (Pin & 0x7) * 4;

        MODIFY_REG(GPIOx->AFR[Pin >> 3], (uint32_t)0xF << shifter, (uint32_t)(Config->AlternateMap) << shifter);
    }

    /* IO mode */
    shifter = Pin * 2;

    MODIFY_REG(GPIOx->MODER, 0x0003 << shifter, Config->Mode << shifter);

    /* output stage configuration */
    if (    (Config->Mode == GPIO_MODE_OUTPUT)
         || (Config->Mode == GPIO_MODE_ALTERNATE))
    {
        /* speed */
        MODIFY_REG(GPIOx->OSPEEDR, 0x0003 << shifter, Config->Output.Speed << shifter);

        /* type */
        MODIFY_REG(GPIOx->OTYPER, 0x0001 << Pin, Config->Output.Type << Pin);
    }

    /* Activate the Pull-up or Pull down resistor for the current IO */
    MODIFY_REG(GPIOx->PUPDR, 0x0003 << shifter, Config->Pull << shifter);

    /* EXTI configuration */
    if (Config->Mode == GPIO_MODE_EXTI)
    {
        /* enable SYSCFG clock */
        XPD_SYSCFG_EnableClock();

        /* set EXTI source */
        shifter = (Pin & 0x03) * 4;
        MODIFY_REG(SYSCFG->EXTICR[Pin >> 2], ((uint32_t) 0x0F) << shifter, GET_GPIO_SOURCE(GPIOx) << shifter);

        /* hand over EXTI configuration */
        XPD_EXTI_Init(Pin, &Config->ExtI);
    }
}

/**
 * @brief Restores the GPIO pin to the default settings.
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Pin: selected pin of the port [0 .. 15]
 */
void XPD_GPIO_DeinitPin(GPIO_TypeDef * GPIOx, uint8_t Pin)
{
    uint32_t temp;

    (uint32_t)Pin;

    /* GPIO Mode Configuration */
    /* Configure IO Direction in Input Floating Mode */
    CLEAR_BIT(GPIOx->MODER, (0x0003 << (Pin * 2)));

    /* Deactivate the Pull-up and Pull-down resistor for the current IO */
    CLEAR_BIT(GPIOx->PUPDR, (0x0003 << (Pin * 2)));

    /*------------------------- EXTI Mode Configuration --------------------*/
    XPD_EXTI_Callbacks[Pin] = NULL;

    /* Configure the External Interrupt or event for the current IO */
    CLEAR_BIT(SYSCFG->EXTICR[Pin >> 2], ((uint32_t) 0x0F) << (4 * (Pin & 0x03)));

#ifdef EXTI_BB
    /* Clear EXTI line configuration */
    EXTI_BB->IMR[Pin] = 0;
    EXTI_BB->EMR[Pin] = 0;

    /* Clear Rising Falling edge configuration */
    EXTI_BB->RTSR[Pin] = 0;
    EXTI_BB->FTSR[Pin] = 0;
#else
    temp = 1 << Pin;

    /* Clear EXTI line configuration */
    CLEAR_BIT(EXTI->IMR, temp);
    CLEAR_BIT(EXTI->EMR, temp);

    /* Clear Rising Falling edge configuration */
    CLEAR_BIT(EXTI->RTSR, temp);
    CLEAR_BIT(EXTI->FTSR, temp);
#endif
}

/**
 * @brief Locks the pin's configuration until the next device reset
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Pin: selected pin of the port [0 .. 15]
 */
void XPD_GPIO_LockPin(GPIO_TypeDef * GPIOx, uint8_t Pin)
{
    uint32_t pinmask = 1 << (uint32_t)Pin;
    uint32_t temp = GPIO_LCKR_LCKK | pinmask;

    if ((GPIOx->LCKR.w & pinmask) == 0)
    {
        XPD_EnterCritical(GPIOx);

        /* Apply lock key write sequence */
        /* LCKK='1' + LCK[x] */
        GPIOx->LCKR.w = temp;
        /* LCKK='0' + LCK[x] */
        GPIOx->LCKR.w = pinmask;
        /* LCKK='1' + LCK[x] */
        GPIOx->LCKR.w = temp;

        /* read register */
        temp = GPIOx->LCKR.w;

        (void)temp;

        XPD_ExitCritical(GPIOx);
    }
}

/** @} */

/** @} */

/** @} */

