#include "xpd.h"
#include "stm32f072b_discovery.h"

#define GYRO_INT1_PIN                    1                  /* PC.01 */
#define GYRO_INT1_EXTI_IRQn              EXTI0_1_IRQn
#define GYRO_INT2_PIN                    2                  /* PC.02 */
#define GYRO_INT2_EXTI_IRQn              EXTI2_3_IRQn

/* Read/Write command */
#define READWRITE_CMD                         ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD                      ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                            ((uint8_t)0x00)

const uint8_t LED_PIN[] = {
    6,
    8,
    9,
    7};

/**
 * @brief  Configures LED GPIO.
 * @param  Led: Specifies the Led to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED3
 *     @arg LED4
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_Init(Led_TypeDef Led)
{
    GPIO_InitType gpio;

    gpio.Mode = GPIO_MODE_OUTPUT;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Output.Speed = VERY_HIGH;
    gpio.Pull = GPIO_PULL_FLOAT;
    XPD_GPIO_InitPin(GPIOC, LED_PIN[Led], &gpio);
    XPD_GPIO_WritePin(GPIOC, LED_PIN[Led], RESET);
}

/**
 * @brief  Turns selected LED On.
 * @param  Led: Specifies the Led to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED3
 *     @arg LED4
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_On(Led_TypeDef Led)
{
    XPD_GPIO_WritePin(GPIOC, LED_PIN[Led], SET);
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: Specifies the Led to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED3
 *     @arg LED4
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_Off(Led_TypeDef Led)
{
    XPD_GPIO_WritePin(GPIOC, LED_PIN[Led], RESET);
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: Specifies the Led to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED3
 *     @arg LED4
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_Toggle(Led_TypeDef Led)
{
    XPD_GPIO_TogglePin(GPIOC, LED_PIN[Led]);
}

__weak void BSP_PB_PushCallback(uint32_t extiLine)
{
}

void EXTI0_1_IRQHandler(void)
{
    XPD_EXTI_IRQHandler(0);
    XPD_EXTI_IRQHandler(1);
}

/**
 * @brief  Configures Button GPIO and EXTI Line.
 * @param  Button: Specifies the Button to be configured.
 *   This parameter should be: BUTTON_USER
 * @param  Mode: Specifies Button mode.
 *   This parameter can be one of following parameters:
 *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
 *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
 *                            generation capability
 * @retval None
 */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode)
{
    GPIO_InitType gpio;

    if (Mode == BUTTON_MODE_GPIO)
    {
        /* Configure Button pin as input */
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULL_DOWN;
        XPD_GPIO_InitPin(GPIOA, 0, &gpio);
    }

    if (Mode == BUTTON_MODE_EXTI)
    {
        /* Configure Button pin as input with External interrupt */
        gpio.Mode = GPIO_MODE_EXTI;
        gpio.Pull = GPIO_PULL_FLOAT;
        gpio.ExtI.Edge = EDGE_RISING;
        gpio.ExtI.ITCallback = BSP_PB_PushCallback;
        gpio.ExtI.Reaction = REACTION_IT;
        XPD_GPIO_InitPin(GPIOA, 0, &gpio);

        /* Enable and set Button EXTI Interrupt to the lowest priority */
        XPD_NVIC_SetPriorityConfig(EXTI0_1_IRQn, 0x03, 0x03);
        XPD_NVIC_EnableIRQ(EXTI0_1_IRQn);
    }
}

/**
 * @brief  Returns the selected Push Button state.
 * @param  Button: Specifies the Button to be checked.
 *   This parameter should be: BUTTON_USER
 * @retval The Button GPIO pin value.
 */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
    return XPD_GPIO_ReadPin(GPIOA, 0);
}


DMA_HandleType rdma = NEW_DMA_HANDLE(DMA1_Channel4);
DMA_HandleType tdma = NEW_DMA_HANDLE(DMA1_Channel5);

void DMA1_Channel4_5_6_7_IRQHandler(void)
{
    XPD_DMA_IRQHandler(&tdma);
    XPD_DMA_IRQHandler(&rdma);
}

/* Initialize dependencies of SPI */
static void spiInit(void * handle)
{
    SPI_HandleType * hspi = (SPI_HandleType *)handle;
    GPIO_InitType gpio;
    DMA_InitType idma;

    gpio.Mode = GPIO_MODE_ALTERNATE;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Output.Speed = VERY_HIGH;
    gpio.Pull = GPIO_PULL_FLOAT;
    gpio.AlternateMap = GPIO_SPI2_AF0;
    XPD_GPIO_InitPin(GPIOB, 13, &gpio);
    XPD_GPIO_InitPin(GPIOB, 14, &gpio);
    XPD_GPIO_InitPin(GPIOB, 15, &gpio);

    idma.Priority  = MEDIUM;
    idma.Mode      = DMA_MODE_NORMAL;
    idma.Memory.DataAlignment     = DMA_ALIGN_BYTE;
    idma.Memory.Increment         = ENABLE;
    idma.Peripheral.DataAlignment = DMA_ALIGN_BYTE;
    idma.Peripheral.Increment     = DISABLE;
    idma.Direction                = DMA_PERIPH2MEMORY;
    XPD_DMA_Init(&rdma, &idma);
    idma.Direction                = DMA_MEMORY2PERIPH;
    XPD_DMA_Init(&tdma, &idma);

    hspi->DMA.Receive = &rdma;
    hspi->DMA.Transmit = &tdma;

    XPD_NVIC_SetPriorityConfig(SPI2_IRQn, 0, 3);
    XPD_NVIC_EnableIRQ(SPI2_IRQn);

    XPD_NVIC_SetPriorityConfig(DMA1_Channel4_5_6_7_IRQn, 0, 3);
    XPD_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
}

SPI_HandleType spi = NEW_SPI_HANDLE(SPI2, spiInit, NULL);

void SPI2_IRQHandler(void)
{
    XPD_SPI_IRQHandler(&spi);
}

/* SPIx bus function */
static void SPIx_Init(void);
static uint8_t SPIx_WriteRead(uint8_t byte);
static void SPIx_Error(void);

/* Link function for GYRO peripheral */
void GYRO_IO_Init(void);
void GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);


__weak void BSP_GYRO_INT1Callback(uint32_t extiLine)
{
}
__weak void BSP_GYRO_INT2Callback(uint32_t extiLine)
{
}

void EXTI2_3_IRQHandler(void)
{
    XPD_EXTI_IRQHandler(2);
    XPD_EXTI_IRQHandler(3);
}

/**
 * @brief SPI Bus initialization
 * @retval None
 */
static void SPIx_Init(void)
{
    SPI_InitType ispi;
#ifdef USE_XPD_SPI_ERROR_DETECT
    ispi.CRC_Length = 0;
#endif
    ispi.Clock.Prescaler = CLK_DIV8;
    ispi.Clock.Polarity  = ACTIVE_HIGH;
    ispi.Clock.Phase     = CLOCK_PHASE_1EDGE;
    ispi.Channel = SPI_CHANNEL_FULL_DUPLEX;
    ispi.DataSize = 8;
    ispi.Format = SPI_FORMAT_MSB_FIRST;
    ispi.Mode = SPI_MODE_MASTER;
    ispi.NSS = SPI_NSS_SOFT;
    ispi.TI_Mode = DISABLE;

    XPD_SPI_Init(&spi, &ispi);
}

/**
 * @brief  Sends a Byte through the SPI interface and return the Byte received
 *         from the SPI bus.
 * @param  Byte : Byte send.
 * @retval The received byte value
 */
static uint8_t SPIx_WriteRead(uint8_t Byte)
{
    uint8_t receivedbyte = 0;

    /* Send a Byte through the SPI peripheral */
    /* Read byte from the SPI bus */
    if (XPD_SPI_TransmitReceive(&spi, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, 0x1000) != XPD_OK)
    {
        SPIx_Error();
    }

    return receivedbyte;
}

/**
 * @brief SPI1 error treatment function
 * @retval None
 */
static void SPIx_Error(void)
{
    /* De-initialize the SPI comunication BUS */
    XPD_SPI_Deinit(&spi);

    /* Re- Initiaize the SPI comunication BUS */
    SPIx_Init();
}

/**
 * @brief  Configures the GYRO SPI interface.
 * @retval None
 */
void GYRO_IO_Init(void)
{
    GPIO_InitType gpio;
    DMA_InitType idma;

    /* GYRO NCS */
    gpio.Mode = GPIO_MODE_OUTPUT;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Output.Speed = VERY_HIGH;
    gpio.Pull = GPIO_PULL_FLOAT;
    XPD_GPIO_InitPin(GPIOC, 0, &gpio);
    XPD_GPIO_WritePin(GPIOC, 0, SET);

    /* INT1, INT2 */
    gpio.Mode = GPIO_MODE_EXTI;
    gpio.Pull = GPIO_PULL_FLOAT;
    gpio.ExtI.Edge = EDGE_RISING;
    gpio.ExtI.ITCallback = BSP_GYRO_INT1Callback;
    gpio.ExtI.Reaction = REACTION_IT;
    XPD_GPIO_InitPin(GPIOC, 1, &gpio);
    gpio.ExtI.ITCallback = BSP_GYRO_INT2Callback;
    XPD_GPIO_InitPin(GPIOC, 2, &gpio);

    /* Enable and set Interrupt to the lowest priority */
    XPD_NVIC_SetPriorityConfig(EXTI0_1_IRQn, 0x03, 0x03);
    XPD_NVIC_EnableIRQ(EXTI0_1_IRQn);
    XPD_NVIC_SetPriorityConfig(EXTI2_3_IRQn, 0x03, 0x03);
    XPD_NVIC_EnableIRQ(EXTI2_3_IRQn);

    SPIx_Init();
}

/**
 * @brief  Writes one byte to the GYRO.
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the GYRO.
 * @param  WriteAddr : GYRO's internal address to write to.
 * @param  NumByteToWrite: Number of bytes to write.
 * @retval None
 */
void GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    /* Configure the MS bit:
     - When 0, the address will remain unchanged in multiple read/write commands.
     - When 1, the address will be auto incremented in multiple read/write commands.
     */
    if (NumByteToWrite > 1)
    {
        WriteAddr |= (uint8_t) MULTIPLEBYTE_CMD;
    }
    /* Set chip select Low at the start of the transmission */
    XPD_GPIO_WritePin(GPIOC, 0, RESET);

    /* Send the Address of the indexed register */
    SPIx_WriteRead(WriteAddr);

    /* Send the data that will be written into the device (MSB First) */
    while (NumByteToWrite > 0)
    {
        SPIx_WriteRead(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }

    /* Set chip select High at the end of the transmission */
    XPD_GPIO_WritePin(GPIOC, 0, SET);
}

/**
 * @brief  Reads a block of data from the GYROSCOPE.
 * @param  pBuffer : pointer to the buffer that receives the data read from the GYROSCOPE.
 * @param  ReadAddr : GYROSCOPE's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the GYROSCOPE.
 * @retval None
 */
void GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    if (NumByteToRead > 1)
    {
        ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
    }
    else
    {
        ReadAddr |= (uint8_t) READWRITE_CMD;
    }
    /* Set chip select Low at the start of the transmission */
    XPD_GPIO_WritePin(GPIOC, 0, RESET);

    /* Send the Address of the indexed register */
    SPIx_WriteRead(ReadAddr);

    /* Receive the data that will be read from the device (MSB First) */
    while (NumByteToRead > 0)
    {
        /* Send dummy byte (0x00) to generate the SPI clock to GYROSCOPE (Slave device) */
        *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
        NumByteToRead--;
        pBuffer++;
    }

    /* Set chip select High at the end of the transmission */
    XPD_GPIO_WritePin(GPIOC, 0, SET);
}
