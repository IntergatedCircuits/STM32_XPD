/**
  ******************************************************************************
  * @file    xpd_bsp.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-07-15
  * @brief   STM32 eXtensible Peripheral Drivers USB Virtual COM Port Project
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

#include <xpd_dma.h>
#include <xpd_gpio.h>
#include <xpd_rcc.h>
#include <xpd_usart.h>
#include <xpd_usb.h>

const GPIO_InitType PinConfig[] =
{
    /* UART pins */
    {
        .Mode = GPIO_MODE_ALTERNATE,
        .Pull = GPIO_PULL_FLOAT,
        .Output.Type  = GPIO_OUTPUT_PUSHPULL,
        .Output.Speed = VERY_HIGH,
        .AlternateMap = GPIO_USART2_AF7
    },
    /* USB pins */
    {
        .Mode = GPIO_MODE_ALTERNATE,
        .Pull = GPIO_PULL_FLOAT,
        .Output.Type  = GPIO_OUTPUT_PUSHPULL,
        .Output.Speed = VERY_HIGH,
        .AlternateMap = GPIO_OTG_FS_AF10
    },
};

/* System clocks configuration */
void ClockConfiguration(void)
{
    const RCC_PLL_InitType pll = {
        .State = OSC_ON,
        .Source = HSE,
        .M = 8,
        .N = 336,
        .P = 2,
        .Q = 7
    };

    /* HSE configuration */
    XPD_RCC_HSEConfig(OSC_ON);

    /* PLL configuration */
    XPD_RCC_PLLConfig(&pll);

    /* System clocks configuration */
    XPD_RCC_HCLKConfig(PLL, CLK_DIV1, 5);

    XPD_RCC_PCLKConfig(PCLK1, CLK_DIV4);
    XPD_RCC_PCLKConfig(PCLK2, CLK_DIV2);
}

/* Ensure preemption-free USB-UART interrupt handling */
#define NVIC_COMMON_PRIO_USB_USART     0

/************************* USB ************************************/

/* USB dependencies initialization */
static void usbinit(void * handle)
{
    /* GPIO settings */
    XPD_GPIO_InitPin(USB_DM_PIN, &PinConfig[USB_PIN_CFG]);
    XPD_GPIO_InitPin(USB_DP_PIN, &PinConfig[USB_PIN_CFG]);

    /* USB clock configuration - must be operated from 48 MHz */

    /* Enable USB FS Interrupt */
    XPD_NVIC_SetPriorityConfig(OTG_FS_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
    XPD_NVIC_EnableIRQ(OTG_FS_IRQn);

    /* Wakeup EXTI line setup */
    if (((USB_HandleType*)handle)->LowPowerMode != DISABLE)
    {
        EXTI_InitType wakeup = USB_WAKEUP_EXTI_INIT;
        XPD_EXTI_Init(USB_OTG_FS_WAKEUP_EXTI_LINE, &wakeup);

        XPD_NVIC_SetPriorityConfig(OTG_FS_WKUP_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
        XPD_NVIC_EnableIRQ(OTG_FS_WKUP_IRQn);
    }
}

/* USB dependencies deinitialization */
static void usbdeinit(void * handle)
{
    XPD_GPIO_DeinitPin(USB_DM_PIN);
    XPD_GPIO_DeinitPin(USB_DP_PIN);
    XPD_EXTI_Deinit(USB_OTG_FS_WAKEUP_EXTI_LINE);
    XPD_NVIC_DisableIRQ(OTG_FS_IRQn);
    XPD_NVIC_DisableIRQ(OTG_FS_WKUP_IRQn);
}

USB_HandleType usbHandle = NEW_USB_HANDLE(USB_OTG_FS,usbinit,usbdeinit);

/* USB interrupt handling */
void OTG_FS_IRQHandler(void)
{
    XPD_USB_IRQHandler(&usbHandle);
}

/* USB wakeup interrupt handling */
void OTG_FS_WKUP_IRQHandler(void)
{
    XPD_EXTI_ClearFlag(USB_OTG_FS_WAKEUP_EXTI_LINE);

    /* Re-enable suspended PHY clock */
    XPD_USB_PHY_ClockCtrl(&usbHandle, ENABLE);
}

/************************* UART ************************************/
DMA_HandleType dmauat = NEW_DMA_HANDLE(DMA1_Stream6);
DMA_HandleType dmauar = NEW_DMA_HANDLE(DMA1_Stream5);

/* UART dependencies initialization */
static void uartinit(void * handle)
{
    DMA_InitType dmaSetup =
    {
        .Channel                  = 4,
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

    XPD_NVIC_SetPriorityConfig(DMA1_Stream5_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
    XPD_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    XPD_NVIC_SetPriorityConfig(DMA1_Stream6_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
    XPD_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    /* USART transmit DMA uses TC interrupt for completion callback */
    XPD_NVIC_SetPriorityConfig(USART2_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
    XPD_NVIC_EnableIRQ(USART2_IRQn);
}

/* UART dependencies deinitialization */
static void uartdeinit(void * handle)
{
    XPD_GPIO_DeinitPin(UART_TX_PIN);
    XPD_GPIO_DeinitPin(UART_RX_PIN);

    XPD_DMA_Deinit(&dmauat);
    XPD_DMA_Deinit(&dmauar);
    XPD_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    XPD_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    XPD_NVIC_DisableIRQ(USART2_IRQn);
}

/* UART DMA interrupt handling */
void DMA1_Stream5_IRQHandler(void)
{
    XPD_DMA_IRQHandler(&dmauar);
    NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);
}
void DMA1_Stream6_IRQHandler(void)
{
    XPD_DMA_IRQHandler(&dmauat);
    NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn);
}

USART_HandleType uart = NEW_USART_HANDLE(USART2, uartinit, uartdeinit);

/* UART interrupt handling */
void USART2_IRQHandler(void)
{
    XPD_USART_IRQHandler(&uart);
}
