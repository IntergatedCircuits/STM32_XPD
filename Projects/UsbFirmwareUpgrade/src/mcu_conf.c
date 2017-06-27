/**
  ******************************************************************************
  * @file    mcu_conf.c
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
#include <xpd_user.h>
#include <xpd_utils.h>

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

/* Callback is used to set the USB FS device 1K5 pullup resistor on DP */
static void usbConnect(FunctionalState state)
{
    XPD_GPIO_WritePin(USB_CONNECT_PIN, state);
}

/* USB dependencies initialization */
static void usbinit(void * handle)
{
    /* GPIO settings */
    XPD_GPIO_InitPin(USB_DM_PIN, &PinConfig[USB_PIN_CFG]);
    XPD_GPIO_InitPin(USB_DP_PIN, &PinConfig[USB_PIN_CFG]);
    XPD_GPIO_InitPin(USB_CONNECT_PIN, &PinConfig[OUT_PIN_CFG]);

    /* USB clock configuration - must be operated from 48 MHz */
    XPD_USB_ClockConfig(USB_CLOCKSOURCE_PLL_DIV1p5);

    /* USB 1K5 pullup resistor is external, map GPIO switcher to handle */
    ((USB_HandleType*)handle)->Callbacks.ConnectionStateCtrl = usbConnect;

    /* Remap USB interrupts */
    XPD_USB_ITRemap(ENABLE);

    /* Enable USB FS Interrupt (only EP0 used, USB_HP_IRQn is not used) */
    XPD_NVIC_SetPriorityConfig(USB_LP_IRQn, 0, 0);
    XPD_NVIC_EnableIRQ(USB_LP_IRQn);

    /* Wakeup EXTI line setup */
    if (((USB_HandleType*)handle)->LowPowerMode != DISABLE)
    {
        EXTI_InitType wakeup = USB_WAKEUP_EXTI_INIT;
        XPD_EXTI_Init(USB_WAKEUP_EXTI_LINE, &wakeup);

        XPD_NVIC_SetPriorityConfig(USBWakeUp_RMP_IRQn, 0, 0);
        XPD_NVIC_EnableIRQ(USBWakeUp_RMP_IRQn);
    }
}

/* USB dependencies deinitialization */
static void usbdeinit(void * handle)
{
    XPD_GPIO_DeinitPin(USB_DM_PIN);
    XPD_GPIO_DeinitPin(USB_DP_PIN);
    XPD_GPIO_DeinitPin(USB_CONNECT_PIN);
    XPD_NVIC_DisableIRQ(USB_LP_IRQn);
    XPD_NVIC_DisableIRQ(USBWakeUp_RMP_IRQn);
}

USB_HandleType usbHandle = NEW_USB_HANDLE(USB, usbinit, usbdeinit);

/* USB interrupt handling */
void USB_LP_IRQHandler(void)
{
    XPD_USB_IRQHandler(&usbHandle);
}

/* USB wakeup interrupt handling */
void USBWakeUp_RMP_IRQHandler(void)
{
    XPD_EXTI_ClearFlag(USB_WAKEUP_EXTI_LINE);

    /* Re-enable suspended PHY clock */
    XPD_USB_PHY_ClockCtrl(&usbHandle, ENABLE);
}
