/**
  ******************************************************************************
  * @file    xpd_bsp.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-06-05
  * @brief   STM32 eXtensible Peripheral Drivers USB Firmware Upgrade Project
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
#include <xpd_bsp.h>

#include <xpd_rcc.h>

const GPIO_InitType PinConfig[] =
{
    /* USB pins */
    {
        .Mode = GPIO_MODE_ALTERNATE,
        .Pull = GPIO_PULL_FLOAT,
        .Output.Type  = GPIO_OUTPUT_PUSHPULL,
        .Output.Speed = VERY_HIGH,
        .AlternateMap = GPIO_USB_AF14
    },
    /* Outputs:
     * USB 1K5 pullup connector
     * LEDs */
    {
        .Mode = GPIO_MODE_OUTPUT,
        .Pull = GPIO_PULL_FLOAT,
        .Output.Type  = GPIO_OUTPUT_PUSHPULL,
        .Output.Speed = HIGH,
    },
};

/* System clocks configuration */
void ClockConfiguration(void)
{
    /* PLL configuration */
    const RCC_PLL_InitType pll = {
        .State = ENABLE,
        .Source = HSE,
        .Multiplier = 72000000 / HSE_VALUE
    };
    XPD_RCC_HSEConfig(OSC_ON);

    XPD_RCC_PLLConfig(&pll);

    /* System clocks configuration */
    XPD_RCC_HCLKConfig(PLL, CLK_DIV1, 2);

    XPD_RCC_PCLKConfig(PCLK1, CLK_DIV2);
    XPD_RCC_PCLKConfig(PCLK2, CLK_DIV1);
}

/************************* USB ************************************/

#ifdef USB_CONNECT_PIN
/* Callback is used to set the USB FS device 1K5 pullup resistor on DP */
static void usbConnect(FunctionalState state)
{
    XPD_GPIO_WritePin(USB_CONNECT_PIN, state);
}
#endif

/* USB dependencies initialization */
static void usbinit(void * handle)
{
    /* GPIO settings */
    XPD_GPIO_InitPin(USB_DM_PIN, &PinConfig[USB_PIN_CFG]);
    XPD_GPIO_InitPin(USB_DP_PIN, &PinConfig[USB_PIN_CFG]);
#ifdef USB_CONNECT_PIN
    XPD_GPIO_InitPin(USB_CONNECT_PIN, &PinConfig[OUT_PIN_CFG]);

    /* USB 1K5 pullup resistor is external, map GPIO switcher to handle */
    ((USB_HandleType*)handle)->Callbacks.ConnectionStateCtrl = usbConnect;
#endif

    /* USB clock configuration - must be operated from 48 MHz */
    XPD_USB_ClockConfig(USB_CLOCKSOURCE_PLL_DIV1p5);

    /* Remap USB interrupts */
    XPD_USB_ITRemap(ENABLE);

    /* Enable USB FS Interrupt (only EP0 used, USB_HP_IRQn is not used) */
    XPD_NVIC_SetPriorityConfig(USB_LP_IRQn, 0, 0);
    XPD_NVIC_EnableIRQ(USB_LP_IRQn);
}

/* USB dependencies deinitialization */
static void usbdeinit(void * handle)
{
    XPD_GPIO_DeinitPin(USB_DM_PIN);
    XPD_GPIO_DeinitPin(USB_DP_PIN);
#ifdef USB_CONNECT_PIN
    XPD_GPIO_DeinitPin(USB_CONNECT_PIN);
#endif
    XPD_NVIC_DisableIRQ(USB_LP_IRQn);
}

USB_HandleType usbHandle = NEW_USB_HANDLE(USB, usbinit, usbdeinit);

/* USB interrupt handling */
void USB_LP_IRQHandler(void)
{
    XPD_USB_IRQHandler(&usbHandle);
}
