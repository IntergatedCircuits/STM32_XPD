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
    /* UART pins */
    {
        .Mode = GPIO_MODE_ALTERNATE,
        .Pull = GPIO_PULL_FLOAT,
        .Output.Type  = GPIO_OUTPUT_PUSHPULL,
        .Output.Speed = VERY_HIGH,
        .AlternateMap = GPIO_USART1_AF7
    },
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

    /* Enable USB FS Interrupt */
    XPD_NVIC_EnableIRQ(USB_LP_IRQn);
    XPD_NVIC_EnableIRQ(USB_HP_IRQn);

    /* Wakeup EXTI line setup */
    if (((USB_HandleType*)handle)->LowPowerMode != DISABLE)
    {
        EXTI_InitType wakeup = USB_WAKEUP_EXTI_INIT;
        XPD_EXTI_Init(USB_WAKEUP_EXTI_LINE, &wakeup);

        XPD_NVIC_EnableIRQ(USBWakeUp_RMP_IRQn);
    }
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
    XPD_NVIC_DisableIRQ(USB_HP_IRQn);
    XPD_NVIC_DisableIRQ(USBWakeUp_RMP_IRQn);
}

USB_HandleType usbHandle = NEW_USB_HANDLE(USB, usbinit, usbdeinit);

/* USB interrupt handling */
void USB_LP_IRQHandler(void)
{
    XPD_USB_IRQHandler(&usbHandle);
}
void USB_HP_IRQHandler(void)
{
    XPD_USB_IRQHandler(&usbHandle);
}
void USBWakeUp_RMP_IRQHandler(void)
{
    XPD_EXTI_ClearFlag(USB_WAKEUP_EXTI_LINE);

    /* Re-enable suspended PHY clock */
    XPD_USB_PHY_ClockCtrl(&usbHandle, ENABLE);

    XPD_USB_IRQHandler(&usbHandle);
}

/************************* UART ************************************/
DMA_HandleType dmauat = NEW_DMA_HANDLE(DMA1_Channel4);
DMA_HandleType dmauar = NEW_DMA_HANDLE(DMA1_Channel5);

/* UART dependencies initialization */
static void uartinit(void * handle)
{
    DMA_InitType dmaSetup =
    {
        .Priority                 = MEDIUM,
        .Mode                     = DMA_MODE_NORMAL,
        .Memory.DataAlignment     = DMA_ALIGN_BYTE,
        .Memory.Increment         = ENABLE,
        .Peripheral.DataAlignment = DMA_ALIGN_BYTE,
        .Peripheral.Increment     = DISABLE,
        .Direction                = DMA_MEMORY2PERIPH,
    };

    /* GPIO settings */
    XPD_GPIO_InitPin(UART_TX_PIN, &PinConfig[UART_PIN_CFG]);
    XPD_GPIO_InitPin(UART_RX_PIN, &PinConfig[UART_PIN_CFG]);

    /* DMA settings */
    XPD_DMA_Init(&dmauat, &dmaSetup);
    dmaSetup.Direction = DMA_PERIPH2MEMORY;
    dmaSetup.Mode      = DMA_MODE_CIRCULAR;
    XPD_DMA_Init(&dmauar, &dmaSetup);

    ((USART_HandleType*)handle)->DMA.Transmit = &dmauat;
    ((USART_HandleType*)handle)->DMA.Receive  = &dmauar;

    XPD_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    XPD_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    /* USART transmit DMA uses TC interrupt for completion callback */
    XPD_NVIC_EnableIRQ(USART1_IRQn);
}

/* UART dependencies deinitialization */
static void uartdeinit(void * handle)
{
    XPD_GPIO_DeinitPin(UART_TX_PIN);
    XPD_GPIO_DeinitPin(UART_RX_PIN);

    XPD_DMA_Deinit(&dmauat);
    XPD_DMA_Deinit(&dmauar);
    XPD_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
    XPD_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    XPD_NVIC_DisableIRQ(USART1_IRQn);
}

/* UART DMA interrupt handling */
void DMA1_Channel4_IRQHandler(void)
{
    XPD_DMA_IRQHandler(&dmauat);
}
void DMA1_Channel5_IRQHandler(void)
{
    XPD_DMA_IRQHandler(&dmauar);
}

USART_HandleType uart = NEW_USART_HANDLE(USART1, uartinit, uartdeinit);

/* UART interrupt handling */
void USART1_IRQHandler(void)
{
    XPD_USART_IRQHandler(&uart);
}
