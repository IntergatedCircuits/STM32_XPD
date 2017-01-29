#include "xpd_user.h"
#include "stm32f3_discovery.h"

/* Read/Write command */
#define READWRITE_CMD                         ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD                      ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                            ((uint8_t)0x00)

void SysTick_Handler(void)
{
    XPD_SysTick_IRQHandler();
}

void ClockConfiguration(void)
{
    /* PLL configuration */
    {
        RCC_PLL_InitType pll = {
            .State = OSC_ON,
            .Source = HSI,
            .Multiplier = 72000000 / HSI_VALUE
        };

        XPD_RCC_PLLConfig(&pll);
    }

    /* System clocks configuration */
    {
        XPD_RCC_HCLKConfig(PLL, CLK_DIV1, 2);

        XPD_RCC_PCLKConfig(PCLK1, CLK_DIV2);
        XPD_RCC_PCLKConfig(PCLK2, CLK_DIV1);
    }
}


const uint8_t LED_PIN[] = {
    9,
    8,
    10,
    15,
    11,
    14,
    12,
    13};

/**
 * @brief  Configures LED GPIO.
 * @param  Led: Specifies the Led to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED_RED
 *     @arg LED_BLUE
 *     @arg LED_ORANGE
 *     @arg LED_GREEN
 *     @arg LED_GREEN2
 *     @arg LED_ORANGE2
 *     @arg LED_BLUE2
 *     @arg LED_RED2
 * @retval None
 */
void BSP_LED_Init(Led_TypeDef Led)
{
    GPIO_InitType gpio;

    gpio.Mode = GPIO_MODE_OUTPUT;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Output.Speed = VERY_HIGH;
    gpio.Pull = GPIO_PULL_FLOAT;
    XPD_GPIO_InitPin(GPIOE, LED_PIN[Led], &gpio);
    XPD_GPIO_WritePin(GPIOE, LED_PIN[Led], RESET);
}

/**
 * @brief  Turns selected LED On.
 * @param  Led: Specifies the Led to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED_RED
 *     @arg LED_BLUE
 *     @arg LED_ORANGE
 *     @arg LED_GREEN
 *     @arg LED_GREEN2
 *     @arg LED_ORANGE2
 *     @arg LED_BLUE2
 *     @arg LED_RED2
 * @retval None
 */
void BSP_LED_On(Led_TypeDef Led)
{
    XPD_GPIO_WritePin(GPIOE, LED_PIN[Led], SET);
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: Specifies the Led to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED_RED
 *     @arg LED_BLUE
 *     @arg LED_ORANGE
 *     @arg LED_GREEN
 *     @arg LED_GREEN2
 *     @arg LED_ORANGE2
 *     @arg LED_BLUE2
 *     @arg LED_RED2
 * @retval None
 */
void BSP_LED_Off(Led_TypeDef Led)
{
    XPD_GPIO_WritePin(GPIOE, LED_PIN[Led], RESET);
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: Specifies the Led to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED_RED
 *     @arg LED_BLUE
 *     @arg LED_ORANGE
 *     @arg LED_GREEN
 *     @arg LED_GREEN2
 *     @arg LED_ORANGE2
 *     @arg LED_BLUE2
 *     @arg LED_RED2
 * @retval None
 */
void BSP_LED_Toggle(Led_TypeDef Led)
{
    XPD_GPIO_TogglePin(GPIOE, LED_PIN[Led]);
}

__weak void BSP_PB_PushCallback(uint32_t extiLine)
{
}

void EXTI0_IRQHandler(void)
{
    XPD_EXTI_IRQHandler(0);
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
        XPD_NVIC_SetPriorityConfig(EXTI0_IRQn, 0x03, 0x03);
        XPD_NVIC_EnableIRQ(EXTI0_IRQn);
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

/* Initialize dependencies of SPI */
static void spiInit(void * handle)
{
    GPIO_InitType gpio;

    gpio.Mode = GPIO_MODE_ALTERNATE;
    gpio.Output.Type = GPIO_OUTPUT_PUSHPULL;
    gpio.Output.Speed = VERY_HIGH;
    gpio.Pull = GPIO_PULL_FLOAT;
    gpio.AlternateMap = GPIO_SPI1_AF5;
    XPD_GPIO_InitPin(GPIOA, 5, &gpio);
    XPD_GPIO_InitPin(GPIOA, 6, &gpio);
    XPD_GPIO_InitPin(GPIOA, 7, &gpio);

    XPD_NVIC_SetPriorityConfig(SPI1_IRQn, 0, 3);
    XPD_NVIC_EnableIRQ(SPI1_IRQn);
}

static void spiDeInit(void * handle)
{
    XPD_GPIO_DeinitPin(GPIOA, 5);
    XPD_GPIO_DeinitPin(GPIOA, 6);
    XPD_GPIO_DeinitPin(GPIOA, 7);

    XPD_NVIC_DisableIRQ(SPI1_IRQn);
}

SPI_HandleType spi = NEW_SPI_HANDLE(SPI1, spiInit, spiDeInit);

void SPI1_IRQHandler(void)
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

void EXTI1_IRQHandler(void)
{
    XPD_EXTI_IRQHandler(1);
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
    XPD_GPIO_InitPin(GPIOE, 3, &gpio);
    XPD_GPIO_WritePin(GPIOE, 3, SET);

    /* INT1, INT2 */
    gpio.Mode = GPIO_MODE_EXTI;
    gpio.Pull = GPIO_PULL_FLOAT;
    gpio.ExtI.Edge = EDGE_RISING;
    gpio.ExtI.ITCallback = BSP_GYRO_INT1Callback;
    gpio.ExtI.Reaction = REACTION_IT;
    XPD_GPIO_InitPin(GPIOE, 0, &gpio);
    gpio.ExtI.ITCallback = BSP_GYRO_INT2Callback;
    XPD_GPIO_InitPin(GPIOE, 1, &gpio);

    /* Enable and set Interrupt to the lowest priority */
    XPD_NVIC_SetPriorityConfig(EXTI0_IRQn, 0x03, 0x03);
    XPD_NVIC_EnableIRQ(EXTI0_IRQn);
    XPD_NVIC_SetPriorityConfig(EXTI1_IRQn, 0x03, 0x03);
    XPD_NVIC_EnableIRQ(EXTI1_IRQn);

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
    XPD_GPIO_WritePin(GPIOE, 3, RESET);

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
    XPD_GPIO_WritePin(GPIOE, 3, SET);
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
    XPD_GPIO_WritePin(GPIOE, 3, RESET);

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
    XPD_GPIO_WritePin(GPIOE, 3, SET);
}
