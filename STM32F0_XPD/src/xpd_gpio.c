/**
  ******************************************************************************
  * @file    xpd_gpio.c
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
#include "xpd_gpio.h"
#include "xpd_rcc.h"

#ifdef PWR_CR3_APC
static struct {
    __IO uint32_t PUCR;
    __IO uint32_t PDCR;
} * const pwr_pullConfig = &PWR->PUCRA;

#ifdef PWR_BB
static struct {
    __IO uint32_t PUC[32];
    __IO uint32_t PDC[32];
} * const pwr_pullPinConfig = PERIPH_BB(&PWR->PUCRA);
#endif
#endif

/** @defgroup GPIO
 * @{ */

static void gpio_clockEnable(GPIO_TypeDef * GPIOx)
{
#if defined(GPIOH) && (RCC_POS_GPIOH < RCC_POS_GPIOA)
    if (GPIOx == GPIOH)
    {
        XPD_RCC_ClockEnable(RCC_POS_GPIOH);
    }
    else
#endif
    {
        XPD_RCC_ClockEnable(RCC_POS_GPIOA + GPIO_PORT_OFFSET(GPIOx));
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
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Config: pointer to the setup parameters
 */
void XPD_GPIO_InitPort(GPIO_TypeDef * GPIOx, const GPIO_InitType * Config)
{
    uint32_t temp;
    uint32_t reg;

    /* enable GPIO clock */
    gpio_clockEnable(GPIOx);

#ifdef PWR_CR3_APC
    switch (Config->PowerDownPull)
    {
        case GPIO_PULL_UP:
            pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PUCR = 0xFFFF;
            pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PDCR = 0;
            break;
        case GPIO_PULL_DOWN:
            pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PUCR = 0;
            pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PDCR = 0xFFFF;
            break;
        case GPIO_PULL_FLOAT:
        default:
            pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PUCR = 0;
            pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PDCR = 0;
            break;
    }
#endif

    /* alternate function mapping */
    if ((Config->Mode == GPIO_MODE_ALTERNATE) && (Config->AlternateMap != 0))
    {
        for (temp = Config->AlternateMap; temp != 0; temp <<= 4)
        {
            reg |= temp;
        }
        GPIOx->AFR[0] = reg;
        GPIOx->AFR[1] = reg;
    }
    else
    {
        GPIOx->AFR[0] = 0;
        GPIOx->AFR[1] = 0;
    }

    /* IO direction setup */
    switch (Config->Mode)
    {
        case GPIO_MODE_OUTPUT:
            GPIOx->MODER = 0x55555555;
            break;
        case GPIO_MODE_ALTERNATE:
            GPIOx->MODER = 0xAAAAAAAA;
            break;
        case GPIO_MODE_ANALOG:
            GPIOx->MODER = 0xFFFFFFFF;
            break;
        case GPIO_MODE_INPUT:
        case GPIO_MODE_EXTI:
        default:
            GPIOx->MODER = 0;
            break;
    }

#ifdef GPIO_ASCR_ASC0
    /* Set the ADC connection bit if GPIO_ADC_AF is selected */
    if (Config->AlternateMap == GPIO_ADC_AF)
    {
        GPIOx->ASCR = 0xFFFF;
    }
    else
    {
        GPIOx->ASCR = 0x0000;
    }
#endif

    /* output stage configuration */
    if ((Config->Mode == GPIO_MODE_OUTPUT) || (Config->Mode == GPIO_MODE_ALTERNATE))
    {
        /* speed */
        switch (Config->Output.Speed)
        {
            case MEDIUM:
                GPIOx->OSPEEDR = 0x55555555;
                break;
            case HIGH:
                GPIOx->OSPEEDR = 0xAAAAAAAA;
                break;
            case VERY_HIGH:
                GPIOx->OSPEEDR = 0xFFFFFFFF;
                break;
            case LOW:
            default:
                GPIOx->OSPEEDR = 0;
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
        case GPIO_PULL_UP:
            GPIOx->PUPDR = 0x55555555;
            break;
        case GPIO_PULL_DOWN:
            GPIOx->PUPDR = 0xAAAAAAAA;
            break;
        case GPIO_PULL_FLOAT:
        default:
            GPIOx->PUPDR = 0;
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
 * @param GPIOx: pointer to the GPIO peripheral
 * @param Pin: selected pin of the port [0 .. 15]
 * @param Config: pointer to the setup parameters
 */
void XPD_GPIO_InitPin(GPIO_TypeDef * GPIOx, uint8_t Pin, const GPIO_InitType * Config)
{
    uint32_t temp;
    uint32_t shifter;

    (uint32_t) Pin;

    /* enable GPIO clock */
    gpio_clockEnable(GPIOx);

#ifdef PWR_CR3_APC
    /* configure pull direction in power down */
#ifdef PWR_BB
    pwr_pullPinConfig[GPIO_PORT_OFFSET(GPIOx)].PUC[Pin] = Config->PowerDownPull;
    pwr_pullPinConfig[GPIO_PORT_OFFSET(GPIOx)].PDC[Pin] = Config->PowerDownPull >> 1;
#else
    MODIFY_REG(pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PUCR, 1 << Pin, (uin32_t)Config->PowerDownPull << Pin);
    MODIFY_REG(pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PDCR, 1 << Pin, ((uin32_t)Config->PowerDownPull >> 1) << Pin);
#endif
#endif

    /* alternate mapping */
    if (Config->Mode == GPIO_MODE_ALTERNATE)
    {
        shifter = (Pin & 0x7) * 4;

        MODIFY_REG(GPIOx->AFR[Pin >> 3], (uint32_t)0xF << shifter, (uint32_t)(Config->AlternateMap) << shifter);
    }

    /* IO mode */
    shifter = Pin * 2;

    MODIFY_REG(GPIOx->MODER, 0x0003 << shifter, Config->Mode << shifter);

#ifdef GPIO_ASCR_ASC0
    /* Set the ADC connection bit if GPIO_ADC_AF is selected */
    MODIFY_REG(GPIOx->ASCR, 0x0001 << Pin, (uint32_t)(Config->AlternateMap >> 4) << Pin);
#endif

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
        /* set EXTI source */
        shifter = (Pin & 0x03) * 4;
        MODIFY_REG(SYSCFG->EXTICR[Pin >> 2], ((uint32_t) 0x0F) << shifter, GPIO_PORT_OFFSET(GPIOx) << shifter);

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
    (uint32_t)Pin;

#ifdef PWR_CR3_APC
    /* configure pull direction in power down */
#ifdef PWR_BB
    pwr_pullPinConfig[GPIO_PORT_OFFSET(GPIOx)].PUC[Pin] = 0;
    pwr_pullPinConfig[GPIO_PORT_OFFSET(GPIOx)].PDC[Pin] = 0;
#else
    CLEAR_BIT(pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PUCR, 1 << Pin);
    CLEAR_BIT(pwr_pullConfig[GPIO_PORT_OFFSET(GPIOx)].PDCR, 1 << Pin);
#endif
#endif

    /* GPIO Mode Configuration */
    /* Configure IO Direction in Input Floating Mode */
    CLEAR_BIT(GPIOx->MODER, (0x0003 << (Pin * 2)));

    /* Deactivate the Pull-up and Pull-down resistor for the current IO */
    CLEAR_BIT(GPIOx->PUPDR, (0x0003 << (Pin * 2)));

    /* Configure the External Interrupt or event for the current IO */
    CLEAR_BIT(SYSCFG->EXTICR[Pin >> 2], ((uint32_t) 0x0F) << (4 * (Pin & 0x03)));

    XPD_EXTI_Deinit(Pin);
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
    }
}

/** @} */

/** @} */

/** @} */
