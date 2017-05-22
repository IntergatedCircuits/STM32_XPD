/**
  ******************************************************************************
  * @file    mcu_conf.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-04-16
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
#include <xpd_user.h>
#include <xpd_utils.h>

const GPIO_InitType PinConfig[] =
{
    /* UART pins */
    {
        .Mode = GPIO_MODE_ALTERNATE,
        .Pull = GPIO_PULL_FLOAT,
        .Output.Type  = GPIO_OUTPUT_PUSHPULL,
        .Output.Speed = VERY_HIGH,
        .AlternateMap = GPIO_USART1_AF1
    },
    /* USB pins */
    {
        .Mode = GPIO_MODE_ALTERNATE,
        .Pull = GPIO_PULL_FLOAT,
        .Output.Type  = GPIO_OUTPUT_PUSHPULL,
        .Output.Speed = VERY_HIGH,
        .AlternateMap = GPIO_USB_AF2
    },
};

/* System clocks configuration */
void ClockConfiguration(void)
{
    const CRS_InitType crsSetup = {
        .Source     = CRS_SYNC_SOURCE_USB,
        .ErrorLimit = CRS_ERRORLIMIT_DEFAULT
    };

    /* HSI48 configuration */
    XPD_RCC_HSI48Config(OSC_ON);
    XPD_CRS_Init(&crsSetup);

    /* System clocks configuration */
    XPD_RCC_HCLKConfig(HSI48, CLK_DIV1, 1);

    XPD_RCC_PCLKConfig(PCLK1, CLK_DIV1);
}

/************************* USB ************************************/

/* USB dependencies initialization */
static void usbinit(void * handle)
{
    /* GPIO settings */
    XPD_GPIO_InitPin(USB_DM_PIN, &PinConfig[USB_PIN_CFG]);
    XPD_GPIO_InitPin(USB_DP_PIN, &PinConfig[USB_PIN_CFG]);

    /* USB clock configuration - must be operated from 48 MHz */
    XPD_USB_ClockConfig(USB_CLOCKSOURCE_HSI48);

    /* Enable USB FS Interrupt */
    XPD_NVIC_SetPriorityConfig(USB_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
    XPD_NVIC_EnableIRQ(USB_IRQn);

#ifdef XPD_GPIOA_PinRemap
    XPD_GPIOA_PinRemap(11);
#endif

    /* Wakeup EXTI line setup */
    if (((USB_HandleType*)handle)->LowPowerMode != DISABLE)
    {
        EXTI_InitType wakeup = USB_WAKEUP_EXTI_INIT;
        XPD_EXTI_Init(USB_WAKEUP_EXTI_LINE, &wakeup);
    }
}

/* USB dependencies deinitialization */
static void usbdeinit(void * handle)
{
    XPD_GPIO_DeinitPin(USB_DM_PIN);
    XPD_GPIO_DeinitPin(USB_DP_PIN);
    XPD_NVIC_DisableIRQ(USB_IRQn);
}

USB_HandleType usbHandle = NEW_USB_HANDLE(USB, usbinit, usbdeinit);

/* Common interrupt handler for USB core and WKUP line */
void USB_IRQHandler(void)
{
    /* Handle USB interrupts */
    XPD_USB_IRQHandler(&usbHandle);

    /* Handle USB WKUP interrupts */
    XPD_EXTI_ClearFlag(USB_WAKEUP_EXTI_LINE);

    XPD_USB_PHY_ClockCtrl(&usbHandle, ENABLE);
}

/************************* UART ************************************/
DMA_HandleType dmauat = NEW_DMA_HANDLE(DMA1_Channel2);
DMA_HandleType dmauar = NEW_DMA_HANDLE(DMA1_Channel3);

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

    XPD_NVIC_SetPriorityConfig(DMA1_Channel2_3_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
    XPD_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    /* USART transmit DMA uses TC interrupt for completion callback */
    XPD_NVIC_SetPriorityConfig(USART1_IRQn, NVIC_COMMON_PRIO_USB_USART, 0);
    XPD_NVIC_EnableIRQ(USART1_IRQn);
}

/* UART dependencies deinitialization */
static void uartdeinit(void * handle)
{
    XPD_GPIO_DeinitPin(UART_TX_PIN);
    XPD_GPIO_DeinitPin(UART_RX_PIN);

    XPD_DMA_Deinit(&dmauat);
    XPD_DMA_Deinit(&dmauar);
    XPD_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
    XPD_NVIC_DisableIRQ(USART1_IRQn);
}

/* UART DMA interrupt handling */
void DMA1_Channel2_3_IRQHandler(void)
{
    XPD_DMA_IRQHandler(&dmauat);
    XPD_DMA_IRQHandler(&dmauar);
}

USART_HandleType uart = NEW_USART_HANDLE(USART1, uartinit, uartdeinit);

/* UART interrupt handling */
void USART1_IRQHandler(void)
{
    XPD_USART_IRQHandler(&uart);
}
