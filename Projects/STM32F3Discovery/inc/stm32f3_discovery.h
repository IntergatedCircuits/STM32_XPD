/**
  ******************************************************************************
  * @file    stm32f3_discovery.h
  * @author  MCD Application Team
  * @version V2.1.2
  * @date    13-November-2015
  * @brief   This file contains definitions for STM32F3-Discovery's Leds, push-
  *          buttons hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __STM32F3_DISCOVERY_H
#define __STM32F3_DISCOVERY_H

/**
 * @brief LED Types Definition
 */
typedef enum
{
    LED3 = 0,
    LED4 = 1,
    LED5 = 2,
    LED6 = 3,
    LED7 = 4,
    LED8 = 5,
    LED9 = 6,
    LED10 = 7,

    LED_GREEN = LED6,
    LED_ORANGE = LED5,
    LED_RED = LED3,
    LED_BLUE = LED4,
    LED_GREEN_2 = LED7,
    LED_ORANGE_2 = LED8,
    LED_RED_2 = LED10,
    LED_BLUE_2 = LED9
} Led_TypeDef;

/**
 * @brief BUTTON Types Definition
 */
typedef enum
{
    BUTTON_USER = 0
} Button_TypeDef;

typedef enum
{
    BUTTON_MODE_GPIO = 0,
    BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;


void      BSP_LED_Init(Led_TypeDef Led);
void      BSP_LED_On(Led_TypeDef Led);
void      BSP_LED_Off(Led_TypeDef Led);
void      BSP_LED_Toggle(Led_TypeDef Led);
void      BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
uint32_t  BSP_PB_GetState(Button_TypeDef Button);

void BSP_PB_PushCallback(uint32_t extiLine);
void BSP_GYRO_INT1Callback(uint32_t extiLine);
void BSP_GYRO_INT2Callback(uint32_t extiLine);

#endif /* __STM32F3_DISCOVERY_H */
