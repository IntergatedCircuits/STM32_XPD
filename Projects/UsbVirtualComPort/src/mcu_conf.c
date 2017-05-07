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

const PinConfigType PinConfig[] =
{
    /* UART_TX_PIN:
     * used for UART Tx */
    {   {
            .Mode = GPIO_MODE_ALTERNATE,
            .Pull = GPIO_PULL_FLOAT,
            .Output = { .Type = GPIO_OUTPUT_PUSHPULL, .Speed = VERY_HIGH },
            .AlternateMap = GPIO_USART1_AF1
        }, GPIOA, 9
    },
    /* UART_RX_PIN:
     * used for UART Rx */
    {   {
            .Mode = GPIO_MODE_ALTERNATE,
            .Pull = GPIO_PULL_UP,
            .Output = { .Type = GPIO_OUTPUT_PUSHPULL, .Speed = VERY_HIGH },
            .AlternateMap = GPIO_USART1_AF1
        }, GPIOA, 10
    },
    /* USB_DP_PIN:
     * used for USB */
    {   {
            .Mode = GPIO_MODE_ALTERNATE,
            .Pull = GPIO_PULL_FLOAT,
            .Output = { .Type = GPIO_OUTPUT_PUSHPULL, .Speed = VERY_HIGH },
            .AlternateMap = GPIO_USB_AF2
        }, GPIOA, 12
    },
    /* USB_DM_PIN:
     * used for USB */
    {   {
            .Mode = GPIO_MODE_ALTERNATE,
            .Pull = GPIO_PULL_FLOAT,
            .Output = { .Type = GPIO_OUTPUT_PUSHPULL, .Speed = VERY_HIGH },
            .AlternateMap = GPIO_USB_AF2
        }, GPIOA, 11
    },
};

/* System clocks configuration */
void ClockConfiguration(void)
{
    const RCC_HSI_InitType hsi = {
        .State = OSC_ON,
        .CalibrationValue = HSI48_CALIBRATION_DEFAULT_VALUE
    };

    /* HSI48 configuration */
    XPD_RCC_HSI48Config(&hsi);

    /* System clocks configuration */
    XPD_RCC_HCLKConfig(HSI48, CLK_DIV1, 1);

    XPD_RCC_PCLKConfig(PCLK1, CLK_DIV1);
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
    XPD_GPIO_InitPin(PinConfig[UART_TX_PIN].port, PinConfig[UART_TX_PIN].pin, &PinConfig[UART_TX_PIN].config);
    XPD_GPIO_InitPin(PinConfig[UART_RX_PIN].port, PinConfig[UART_RX_PIN].pin, &PinConfig[UART_RX_PIN].config);

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
    XPD_GPIO_DeinitPin(PinConfig[UART_TX_PIN].port, PinConfig[UART_TX_PIN].pin);
    XPD_GPIO_DeinitPin(PinConfig[UART_RX_PIN].port, PinConfig[UART_RX_PIN].pin);

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
