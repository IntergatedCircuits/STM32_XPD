/**
  ******************************************************************************
  * @file    usbd_cdc_if.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-04-16
  * @brief   STM32 eXtensible Peripheral Drivers USB Virtual COM Port Project
  *
  *  @verbatim
  *
  *          ===================================================================
  *                                  USB Virtual COM Port
  *          ===================================================================
  *           This project implements a virtual COM port using USB CDC interface
  *           and the UART peripheral of the device. The received USB packets
  *           are transferred on UART using a 2-page buffer, one is receiving
  *           the USB OUT endpoint data, the other is used by the UART Tx DMA.
  *           The received UART bytes are put in a circular buffer by the Rx DMA
  *           and are monitored by a timer callback to determine when new bytes
  *           have been received that can be sent on USB IN endpoint.
  *  @endverbatim
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
#include <usbd_cdc_if.h>
#include <xpd_user.h>

#define CDC_OUT_DATA_SIZE   256
#define CDC_IN_DATA_SIZE    256

/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

USART_InitType SerialConfig = {
        .BaudRate      = 115200,
        .Transmitter   = ENABLE,
        .Receiver      = ENABLE,
        .DataSize      = 8,
        .StopBits      = USART_STOPBITS_1,
        .SingleSample  = DISABLE,
#ifdef USE_XPD_USART_ERROR_DETECT
        .Parity        = USART_PARITY_NONE,
#endif
};

UART_InitType UartConfig = {
        .FlowControl   = UART_FLOWCONTROL_NONE,
        .OverSampling8 = ENABLE,
        .HalfDuplex    = DISABLE,
};

typedef enum
{
    BUFFER_EMPTY = 0,
    BUFFER_FULL,
    BUFFER_RECEIVING,
    BUFFER_TRANSMITTING
}BufferStatusType;

/* This structure is used for data transfer management
 * between the two communication channels */
struct {
    uint8_t OutData[2][CDC_OUT_DATA_SIZE / 2];
    BufferStatusType OutStatus[2];
    uint16_t OutLength;
    uint8_t InData[CDC_IN_DATA_SIZE];
    uint16_t Index;
}CDC_Memory;

static void CDC_Init(void);
static void CDC_DeInit(void);
static void CDC_USB_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static void CDC_USB_Received(uint8_t* pbuf, uint32_t length);
static void CDC_USB_Transmitted(uint8_t* pbuf, uint32_t length);

static void CDC_UART_Transmitted(void * handle);
static void CDC_ProcessIN(void);

const USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{ CDC_Init, CDC_DeInit, CDC_USB_Control, CDC_USB_Received, CDC_USB_Transmitted };

/**
 * @brief  This interrupt handler periodically updates IN endpoint data
 *         from UART received data
 */
void SysTick_IRQHandler(void)
{
    CDC_ProcessIN();
}

/**
 * @brief  This function is called from USB CDC when the device is connected
 *         or locally when the UART configuration changes
 */
static void CDC_Init(void)
{
    /* Unsubscribe from periodic callback handler during initialization */
    XPD_SysTick_DisableIT();

    /* Initialize UART with the current configuration, reset DMAs */
	(void) XPD_UART_Init(&uart, &SerialConfig, &UartConfig);
    XPD_DMA_Stop(uart.DMA.Transmit);
    XPD_DMA_Stop(uart.DMA.Receive);

    /* Subscribe to UART transmit complete callback */
    uart.Callbacks.Transmit = CDC_UART_Transmitted;

    /* Page 0 is initialized for OUT endpoint reception */
    CDC_Memory.OutStatus[0] = BUFFER_RECEIVING;
    CDC_Memory.OutStatus[1] = BUFFER_EMPTY;
	(void) USBD_CDC_Receive(&hUsbDeviceFS, CDC_Memory.OutData[0], CDC_OUT_DATA_SIZE / 2);

	/* Start circular buffer reception with DMA for IN endpoint */
    CDC_Memory.Index = 0;
    XPD_USART_ClearFlag(&uart, RXNE);
    (void) XPD_USART_Receive_DMA(&uart, CDC_Memory.InData, CDC_IN_DATA_SIZE);

    /* Periodic function is subscribed to timer callback */
    XPD_SysTick_EnableIT();
}

/**
 * @brief  This function is called from USB CDC when the device is disconnected.
 */
static void CDC_DeInit(void)
{
	(void) XPD_USART_Deinit(&uart);
}

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 */
static void CDC_USB_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
	switch (cmd)
	{
    /* Sets the UART configuration */
	case CDC_SET_LINE_CODING:
	    SerialConfig.BaudRate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
	    SerialConfig.DataSize = pbuf[6];

	    /* set the Stop bit */
	    if (pbuf[4] == USART_STOPBITS_2)
	        SerialConfig.StopBits = USART_STOPBITS_2;
	    else
	        SerialConfig.StopBits = USART_STOPBITS_1;

#ifdef USE_XPD_USART_ERROR_DETECT
	    /* set the parity bit*/
	    switch (pbuf[5])
	    {
	    case 1:
	        SerialConfig.Parity = USART_PARITY_ODD;
	        break;
	    case 2:
	        SerialConfig.Parity = USART_PARITY_EVEN;
	        break;
	    default:
	        SerialConfig.Parity = USART_PARITY_NONE;
	        break;
	    }
#endif
	    CDC_Init();
		break;

    /* Returns the current UART configuration */
	case CDC_GET_LINE_CODING:
		pbuf[0] = (uint8_t) (SerialConfig.BaudRate);
		pbuf[1] = (uint8_t) (SerialConfig.BaudRate >> 8);
		pbuf[2] = (uint8_t) (SerialConfig.BaudRate >> 16);
		pbuf[3] = (uint8_t) (SerialConfig.BaudRate >> 24);
		pbuf[4] = SerialConfig.StopBits;
#ifdef USE_XPD_USART_ERROR_DETECT
		pbuf[5] = (SerialConfig.Parity == USART_PARITY_ODD) ? 1 : SerialConfig.Parity;
#endif
		pbuf[6] = SerialConfig.DataSize;
		break;

	case CDC_SET_CONTROL_LINE_STATE:
#if 0 /* RTS and CTS signals are not mapped on the current hardware */
	    UartConfig.FlowControl = ((pbuf[0] & 1) != 0) ? UART_FLOWCONTROL_RTS_CTS : UART_FLOWCONTROL_NONE;

        CDC_Init();
#endif
		break;

    /* Unused commands */
	case CDC_SEND_BREAK:
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
	default:
		break;
	}
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 * @param  pbuf: Buffer of data to be received
 * @param  length: Number of data received (in bytes)
 */
static void CDC_USB_Received(uint8_t * pbuf, uint32_t length)
{
    uint8_t page = (CDC_Memory.OutStatus[0] == BUFFER_RECEIVING) ? 0 : 1;

    /* If UART transmission is ongoing on other page */
    if (CDC_Memory.OutStatus[1 - page] == BUFFER_TRANSMITTING)
    {
        /* Just indicate buffer status */
        CDC_Memory.OutStatus[page] = BUFFER_FULL;
        CDC_Memory.OutLength       = (uint16_t)length;
    }
    else
    {
        /* Switch pages, start transfer on both */
        CDC_Memory.OutStatus[page] = BUFFER_TRANSMITTING;
        (void) XPD_USART_Transmit_DMA(&uart, CDC_Memory.OutData[page], (uint16_t)length);

        CDC_Memory.OutStatus[1 - page] = BUFFER_RECEIVING;
        (void) USBD_CDC_Receive(&hUsbDeviceFS, CDC_Memory.OutData[1 - page], CDC_OUT_DATA_SIZE / 2);
    }
}

/**
 * @brief  Callback for UART data transmit completion
 * @param  handle: Pointer to the callback issuer handle
 */
static void CDC_UART_Transmitted(void * handle)
{
    uint8_t page = (CDC_Memory.OutStatus[0] == BUFFER_TRANSMITTING) ? 0 : 1;

    /* The current page has been transferred over UART */
    CDC_Memory.OutStatus[page] = BUFFER_EMPTY;

    /* If other page is full already */
    if (CDC_Memory.OutStatus[1 - page] == BUFFER_FULL)
    {
        /* Switch pages, start transfer on both */
        CDC_Memory.OutStatus[1 - page] = BUFFER_TRANSMITTING;
        (void) XPD_USART_Transmit_DMA(&uart, CDC_Memory.OutData[1 - page], CDC_Memory.OutLength);

        CDC_Memory.OutStatus[page] = BUFFER_RECEIVING;
        (void) USBD_CDC_Receive(&hUsbDeviceFS, CDC_Memory.OutData[page], CDC_OUT_DATA_SIZE / 2);
    }
}

/**
 * @brief  This function is called when USB CDC interface finished transmission
 * 		   of buffer data.
 * @param  pbuf: Buffer of data to be received
 * @param  length: Number of data transmitted (in bytes)
 */
static void CDC_USB_Transmitted(uint8_t * pbuf, uint32_t length)
{
    /* Increment index with the transmitted data size */
    CDC_Memory.Index += (uint16_t)length;

    /* Prevent overflow */
    if (CDC_Memory.Index >= CDC_IN_DATA_SIZE)
    {
        CDC_Memory.Index = 0;
    }

    /* Call the UART Rx -> USB IN processing */
    CDC_ProcessIN();
}

/**
 * @brief  This function is called periodically from timer callback
 *         or when USB IN transfer completes. It requests new USB IN transfer
 *         when new UART data has been received.
 */
static void CDC_ProcessIN(void)
{
    /* Determine the buffer index of the UART DMA */
    uint16_t rxIndex = CDC_IN_DATA_SIZE - XPD_DMA_GetStatus(uart.DMA.Receive);

    /* If the UART RX index is ahead, transmit the new data */
    if (CDC_Memory.Index < rxIndex)
    {
        /* Return value is ignored, always attempt to send new data */
        (void) USBD_CDC_Transmit(&hUsbDeviceFS,
                &CDC_Memory.InData[CDC_Memory.Index], rxIndex - CDC_Memory.Index);
    }
    /* If the USB IN index is ahead, the buffer has wrapped, transmit until the end */
    else if (CDC_Memory.Index > rxIndex)
    {
        /* Return value is ignored, always attempt to send new data */
        (void) USBD_CDC_Transmit(&hUsbDeviceFS,
                &CDC_Memory.InData[CDC_Memory.Index], CDC_IN_DATA_SIZE - CDC_Memory.Index);
    }
}

