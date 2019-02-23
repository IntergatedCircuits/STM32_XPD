/**
  ******************************************************************************
  * @file    stm32f401xc.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   CMSIS STM32F401xCxx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral's registers hardware
  *
  * Copyright (c) 2018 Benedek Kupper
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  */
/**
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#ifndef __STM32F401xC_H
#define __STM32F401xC_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup STM32F401xC
  * @{
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals 
  */
#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1U       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U       /*!< FPU present                                   */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device 
 *        in @ref Library_configuration_section 
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84      /*!< SPI4 global Interrupt                                              */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include "system_stm32f4xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */
typedef struct {
    union {
        struct {
            __IO uint32_t AWD : 1;                          /*!<Analog watchdog flag */
            __IO uint32_t EOC : 1;                          /*!<End of conversion */
            __IO uint32_t JEOC : 1;                         /*!<Injected channel end of conversion */
            __IO uint32_t JSTRT : 1;                        /*!<Injected channel Start flag */
            __IO uint32_t STRT : 1;                         /*!<Regular channel Start flag */
            __IO uint32_t OVR : 1;                          /*!<Overrun flag */
                 uint32_t __RESERVED0 : 26;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< ADC status register,                         Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t AWDCH : 5;                        /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
            __IO uint32_t EOCIE : 1;                        /*!<Interrupt enable for EOC */
            __IO uint32_t AWDIE : 1;                        /*!<AAnalog Watchdog interrupt enable */
            __IO uint32_t JEOCIE : 1;                       /*!<Interrupt enable for injected channels */
            __IO uint32_t SCAN : 1;                         /*!<Scan mode */
            __IO uint32_t AWDSGL : 1;                       /*!<Enable the watchdog on a single channel in scan mode */
            __IO uint32_t JAUTO : 1;                        /*!<Automatic injected group conversion */
            __IO uint32_t DISCEN : 1;                       /*!<Discontinuous mode on regular channels */
            __IO uint32_t JDISCEN : 1;                      /*!<Discontinuous mode on injected channels */
            __IO uint32_t DISCNUM : 3;                      /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t JAWDEN : 1;                       /*!<Analog watchdog enable on injected channels */
            __IO uint32_t AWDEN : 1;                        /*!<Analog watchdog enable on regular channels */
            __IO uint32_t RES : 2;                          /*!<RES[2:0] bits (Resolution) */
            __IO uint32_t OVRIE : 1;                        /*!<overrun interrupt enable */
                 uint32_t __RESERVED1 : 5;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< ADC control register 1,                      Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t ADON : 1;                         /*!<A/D Converter ON / OFF */
            __IO uint32_t CONT : 1;                         /*!<Continuous Conversion */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t DMA : 1;                          /*!<Direct Memory access mode */
            __IO uint32_t DDS : 1;                          /*!<DMA disable selection (Single ADC) */
            __IO uint32_t EOCS : 1;                         /*!<End of conversion selection */
            __IO uint32_t ALIGN : 1;                        /*!<Data Alignment */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t JEXTSEL : 4;                      /*!<JEXTSEL[3:0] bits (External event select for injected group) */
            __IO uint32_t JEXTEN : 2;                       /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
            __IO uint32_t JSWSTART : 1;                     /*!<Start Conversion of injected channels */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t EXTSEL : 4;                       /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
            __IO uint32_t EXTEN : 2;                        /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
            __IO uint32_t SWSTART : 1;                      /*!<Start Conversion of regular channels */
                 uint32_t __RESERVED3 : 1;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< ADC control register 2,                      Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t SMP10 : 3;                        /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
            __IO uint32_t SMP11 : 3;                        /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
            __IO uint32_t SMP12 : 3;                        /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
            __IO uint32_t SMP13 : 3;                        /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
            __IO uint32_t SMP14 : 3;                        /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
            __IO uint32_t SMP15 : 3;                        /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
            __IO uint32_t SMP16 : 3;                        /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
            __IO uint32_t SMP17 : 3;                        /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
            __IO uint32_t SMP18 : 3;                        /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
                 uint32_t __RESERVED0 : 5;
        } b;
        __IO uint32_t w;
    } SMPR1;                                                /*!< ADC sample time register 1,                  Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t SMP0 : 3;                         /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
            __IO uint32_t SMP1 : 3;                         /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
            __IO uint32_t SMP2 : 3;                         /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
            __IO uint32_t SMP3 : 3;                         /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
            __IO uint32_t SMP4 : 3;                         /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
            __IO uint32_t SMP5 : 3;                         /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
            __IO uint32_t SMP6 : 3;                         /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
            __IO uint32_t SMP7 : 3;                         /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
            __IO uint32_t SMP8 : 3;                         /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
            __IO uint32_t SMP9 : 3;                         /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SMPR2;                                                /*!< ADC sample time register 2,                  Address offset: 0x10 */
    __IO uint32_t JOFR1;                                    /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
    __IO uint32_t JOFR2;                                    /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
    __IO uint32_t JOFR3;                                    /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
    __IO uint32_t JOFR4;                                    /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
    __IO uint32_t HTR;                                      /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
    __IO uint32_t LTR;                                      /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t SQ13 : 5;                         /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
            __IO uint32_t SQ14 : 5;                         /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
            __IO uint32_t SQ15 : 5;                         /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
            __IO uint32_t SQ16 : 5;                         /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
            __IO uint32_t L : 4;                            /*!<L[3:0] bits (Regular channel sequence length) */
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } SQR1;                                                 /*!< ADC regular sequence register 1,             Address offset: 0x2C */
    union {
        struct {
            __IO uint32_t SQ7 : 5;                          /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
            __IO uint32_t SQ8 : 5;                          /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
            __IO uint32_t SQ9 : 5;                          /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
            __IO uint32_t SQ10 : 5;                         /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
            __IO uint32_t SQ11 : 5;                         /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
            __IO uint32_t SQ12 : 5;                         /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SQR2;                                                 /*!< ADC regular sequence register 2,             Address offset: 0x30 */
    union {
        struct {
            __IO uint32_t SQ1 : 5;                          /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
            __IO uint32_t SQ2 : 5;                          /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
            __IO uint32_t SQ3 : 5;                          /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
            __IO uint32_t SQ4 : 5;                          /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
            __IO uint32_t SQ5 : 5;                          /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
            __IO uint32_t SQ6 : 5;                          /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SQR3;                                                 /*!< ADC regular sequence register 3,             Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t JSQ1 : 5;                         /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */
            __IO uint32_t JSQ2 : 5;                         /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
            __IO uint32_t JSQ3 : 5;                         /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
            __IO uint32_t JSQ4 : 5;                         /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
            __IO uint32_t JL : 2;                           /*!<JL[1:0] bits (Injected Sequence length) */
                 uint32_t __RESERVED0 : 10;
        } b;
        __IO uint32_t w;
    } JSQR;                                                 /*!< ADC injected sequence register,              Address offset: 0x38*/
    __IO uint32_t JDR1;                                     /*!< ADC injected data register 1,                Address offset: 0x3C */
    __IO uint32_t JDR2;                                     /*!< ADC injected data register 2,                Address offset: 0x40 */
    __IO uint32_t JDR3;                                     /*!< ADC injected data register 3,                Address offset: 0x44 */
    __IO uint32_t JDR4;                                     /*!< ADC injected data register 4,                Address offset: 0x48 */
    __IO uint32_t DR;                                       /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t AWD;                                  /*!<Analog watchdog flag */
        __IO uint32_t EOC;                                  /*!<End of conversion */
        __IO uint32_t JEOC;                                 /*!<Injected channel end of conversion */
        __IO uint32_t JSTRT;                                /*!<Injected channel Start flag */
        __IO uint32_t STRT;                                 /*!<Regular channel Start flag */
        __IO uint32_t OVR;                                  /*!<Overrun flag */
             uint32_t __RESERVED0[26];
    } SR;                                                   /*!< ADC status register,                         Address offset: 0x00 */
    struct {
        __IO uint32_t AWDCH[5];                             /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
        __IO uint32_t EOCIE;                                /*!<Interrupt enable for EOC */
        __IO uint32_t AWDIE;                                /*!<AAnalog Watchdog interrupt enable */
        __IO uint32_t JEOCIE;                               /*!<Interrupt enable for injected channels */
        __IO uint32_t SCAN;                                 /*!<Scan mode */
        __IO uint32_t AWDSGL;                               /*!<Enable the watchdog on a single channel in scan mode */
        __IO uint32_t JAUTO;                                /*!<Automatic injected group conversion */
        __IO uint32_t DISCEN;                               /*!<Discontinuous mode on regular channels */
        __IO uint32_t JDISCEN;                              /*!<Discontinuous mode on injected channels */
        __IO uint32_t DISCNUM[3];                           /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
             uint32_t __RESERVED0[6];
        __IO uint32_t JAWDEN;                               /*!<Analog watchdog enable on injected channels */
        __IO uint32_t AWDEN;                                /*!<Analog watchdog enable on regular channels */
        __IO uint32_t RES[2];                               /*!<RES[2:0] bits (Resolution) */
        __IO uint32_t OVRIE;                                /*!<overrun interrupt enable */
             uint32_t __RESERVED1[5];
    } CR1;                                                  /*!< ADC control register 1,                      Address offset: 0x04 */
    struct {
        __IO uint32_t ADON;                                 /*!<A/D Converter ON / OFF */
        __IO uint32_t CONT;                                 /*!<Continuous Conversion */
             uint32_t __RESERVED0[6];
        __IO uint32_t DMA;                                  /*!<Direct Memory access mode */
        __IO uint32_t DDS;                                  /*!<DMA disable selection (Single ADC) */
        __IO uint32_t EOCS;                                 /*!<End of conversion selection */
        __IO uint32_t ALIGN;                                /*!<Data Alignment */
             uint32_t __RESERVED1[4];
        __IO uint32_t JEXTSEL[4];                           /*!<JEXTSEL[3:0] bits (External event select for injected group) */
        __IO uint32_t JEXTEN[2];                            /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
        __IO uint32_t JSWSTART;                             /*!<Start Conversion of injected channels */
             uint32_t __RESERVED2;
        __IO uint32_t EXTSEL[4];                            /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
        __IO uint32_t EXTEN[2];                             /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
        __IO uint32_t SWSTART;                              /*!<Start Conversion of regular channels */
             uint32_t __RESERVED3;
    } CR2;                                                  /*!< ADC control register 2,                      Address offset: 0x08 */
    struct {
        __IO uint32_t SMP10[3];                             /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
        __IO uint32_t SMP11[3];                             /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
        __IO uint32_t SMP12[3];                             /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
        __IO uint32_t SMP13[3];                             /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
        __IO uint32_t SMP14[3];                             /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
        __IO uint32_t SMP15[3];                             /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
        __IO uint32_t SMP16[3];                             /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
        __IO uint32_t SMP17[3];                             /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
        __IO uint32_t SMP18[3];                             /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
             uint32_t __RESERVED0[5];
    } SMPR1;                                                /*!< ADC sample time register 1,                  Address offset: 0x0C */
    struct {
        __IO uint32_t SMP0[3];                              /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
        __IO uint32_t SMP1[3];                              /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
        __IO uint32_t SMP2[3];                              /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
        __IO uint32_t SMP3[3];                              /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
        __IO uint32_t SMP4[3];                              /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
        __IO uint32_t SMP5[3];                              /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
        __IO uint32_t SMP6[3];                              /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
        __IO uint32_t SMP7[3];                              /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
        __IO uint32_t SMP8[3];                              /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
        __IO uint32_t SMP9[3];                              /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
             uint32_t __RESERVED0[2];
    } SMPR2;                                                /*!< ADC sample time register 2,                  Address offset: 0x10 */
    __IO uint32_t JOFR1[32];                                /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
    __IO uint32_t JOFR2[32];                                /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
    __IO uint32_t JOFR3[32];                                /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
    __IO uint32_t JOFR4[32];                                /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
    __IO uint32_t HTR[32];                                  /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
    __IO uint32_t LTR[32];                                  /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
    struct {
        __IO uint32_t SQ13[5];                              /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
        __IO uint32_t SQ14[5];                              /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
        __IO uint32_t SQ15[5];                              /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
        __IO uint32_t SQ16[5];                              /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
        __IO uint32_t L[4];                                 /*!<L[3:0] bits (Regular channel sequence length) */
             uint32_t __RESERVED0[8];
    } SQR1;                                                 /*!< ADC regular sequence register 1,             Address offset: 0x2C */
    struct {
        __IO uint32_t SQ7[5];                               /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
        __IO uint32_t SQ8[5];                               /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
        __IO uint32_t SQ9[5];                               /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
        __IO uint32_t SQ10[5];                              /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
        __IO uint32_t SQ11[5];                              /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
        __IO uint32_t SQ12[5];                              /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
             uint32_t __RESERVED0[2];
    } SQR2;                                                 /*!< ADC regular sequence register 2,             Address offset: 0x30 */
    struct {
        __IO uint32_t SQ1[5];                               /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
        __IO uint32_t SQ2[5];                               /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
        __IO uint32_t SQ3[5];                               /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
        __IO uint32_t SQ4[5];                               /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
        __IO uint32_t SQ5[5];                               /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
        __IO uint32_t SQ6[5];                               /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
             uint32_t __RESERVED0[2];
    } SQR3;                                                 /*!< ADC regular sequence register 3,             Address offset: 0x34 */
    struct {
        __IO uint32_t JSQ1[5];                              /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */
        __IO uint32_t JSQ2[5];                              /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
        __IO uint32_t JSQ3[5];                              /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
        __IO uint32_t JSQ4[5];                              /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
        __IO uint32_t JL[2];                                /*!<JL[1:0] bits (Injected Sequence length) */
             uint32_t __RESERVED0[10];
    } JSQR;                                                 /*!< ADC injected sequence register,              Address offset: 0x38*/
    __IO uint32_t JDR1[32];                                 /*!< ADC injected data register 1,                Address offset: 0x3C */
    __IO uint32_t JDR2[32];                                 /*!< ADC injected data register 2,                Address offset: 0x40 */
    __IO uint32_t JDR3[32];                                 /*!< ADC injected data register 3,                Address offset: 0x44 */
    __IO uint32_t JDR4[32];                                 /*!< ADC injected data register 4,                Address offset: 0x48 */
    __IO uint32_t DR[32];                                   /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_BitBand_TypeDef;



typedef struct {
    union {
        struct {
            __IO uint32_t AWD1 : 1;                         /*!<ADC1 Analog watchdog flag */
            __IO uint32_t EOC1 : 1;                         /*!<ADC1 End of conversion */
            __IO uint32_t JEOC1 : 1;                        /*!<ADC1 Injected channel end of conversion */
            __IO uint32_t JSTRT1 : 1;                       /*!<ADC1 Injected channel Start flag */
            __IO uint32_t STRT1 : 1;                        /*!<ADC1 Regular channel Start flag */
            __IO uint32_t OVR1 : 1;                         /*!<ADC1 DMA overrun  flag */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t AWD2 : 1;                         /*!<ADC2 Analog watchdog flag */
            __IO uint32_t EOC2 : 1;                         /*!<ADC2 End of conversion */
            __IO uint32_t JEOC2 : 1;                        /*!<ADC2 Injected channel end of conversion */
            __IO uint32_t JSTRT2 : 1;                       /*!<ADC2 Injected channel Start flag */
            __IO uint32_t STRT2 : 1;                        /*!<ADC2 Regular channel Start flag */
            __IO uint32_t OVR2 : 1;                         /*!<ADC2 DMA overrun  flag */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t AWD3 : 1;                         /*!<ADC3 Analog watchdog flag */
            __IO uint32_t EOC3 : 1;                         /*!<ADC3 End of conversion */
            __IO uint32_t JEOC3 : 1;                        /*!<ADC3 Injected channel end of conversion */
            __IO uint32_t JSTRT3 : 1;                       /*!<ADC3 Injected channel Start flag */
            __IO uint32_t STRT3 : 1;                        /*!<ADC3 Regular channel Start flag */
            __IO uint32_t OVR3 : 1;                         /*!<ADC3 DMA overrun  flag */
                 uint32_t __RESERVED2 : 10;
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
    union {
        struct {
            __IO uint32_t MULTI : 5;                        /*!<MULTI[4:0] bits (Multi-ADC mode selection) */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t DELAY : 4;                        /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t DDS : 1;                          /*!<DMA disable selection (Multi-ADC mode) */
            __IO uint32_t DMA : 2;                          /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */
            __IO uint32_t ADCPRE : 2;                       /*!<ADCPRE[1:0] bits (ADC prescaler) */
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t VBATE : 1;                        /*!<VBAT Enable */
            __IO uint32_t TSVREFE : 1;                      /*!<Temperature Sensor and VREFINT Enable */
                 uint32_t __RESERVED3 : 8;
        } b;
        __IO uint32_t w;
    } CCR;                                                  /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
    union {
        __IO uint16_t DATA[2];                              /*!< data pair of regular conversions */
        __IO uint32_t w;
    } CDR;                                                  /*!< ADC common regular data register for dual
                                                            AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;


typedef struct {
    struct {
        __IO uint32_t AWD1;                                 /*!<ADC1 Analog watchdog flag */
        __IO uint32_t EOC1;                                 /*!<ADC1 End of conversion */
        __IO uint32_t JEOC1;                                /*!<ADC1 Injected channel end of conversion */
        __IO uint32_t JSTRT1;                               /*!<ADC1 Injected channel Start flag */
        __IO uint32_t STRT1;                                /*!<ADC1 Regular channel Start flag */
        __IO uint32_t OVR1;                                 /*!<ADC1 DMA overrun  flag */
             uint32_t __RESERVED0[2];
        __IO uint32_t AWD2;                                 /*!<ADC2 Analog watchdog flag */
        __IO uint32_t EOC2;                                 /*!<ADC2 End of conversion */
        __IO uint32_t JEOC2;                                /*!<ADC2 Injected channel end of conversion */
        __IO uint32_t JSTRT2;                               /*!<ADC2 Injected channel Start flag */
        __IO uint32_t STRT2;                                /*!<ADC2 Regular channel Start flag */
        __IO uint32_t OVR2;                                 /*!<ADC2 DMA overrun  flag */
             uint32_t __RESERVED1[2];
        __IO uint32_t AWD3;                                 /*!<ADC3 Analog watchdog flag */
        __IO uint32_t EOC3;                                 /*!<ADC3 End of conversion */
        __IO uint32_t JEOC3;                                /*!<ADC3 Injected channel end of conversion */
        __IO uint32_t JSTRT3;                               /*!<ADC3 Injected channel Start flag */
        __IO uint32_t STRT3;                                /*!<ADC3 Regular channel Start flag */
        __IO uint32_t OVR3;                                 /*!<ADC3 DMA overrun  flag */
             uint32_t __RESERVED2[10];
    } CSR;                                                  /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
    struct {
        __IO uint32_t MULTI[5];                             /*!<MULTI[4:0] bits (Multi-ADC mode selection) */
             uint32_t __RESERVED0[3];
        __IO uint32_t DELAY[4];                             /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */
             uint32_t __RESERVED1;
        __IO uint32_t DDS;                                  /*!<DMA disable selection (Multi-ADC mode) */
        __IO uint32_t DMA[2];                               /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */
        __IO uint32_t ADCPRE[2];                            /*!<ADCPRE[1:0] bits (ADC prescaler) */
             uint32_t __RESERVED2[4];
        __IO uint32_t VBATE;                                /*!<VBAT Enable */
        __IO uint32_t TSVREFE;                              /*!<Temperature Sensor and VREFINT Enable */
             uint32_t __RESERVED3[8];
    } CCR;                                                  /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
    struct {
        __IO uint32_t DATA1[16];                            /*!<1st data of a pair of regular conversions */
        __IO uint32_t DATA2[16];                            /*!<2nd data of a pair of regular conversions */
    } CDR;                                                  /*!< ADC common regular data register for dual
                                                            AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_BitBand_TypeDef;



/** 
  * @brief CRC calculation unit 
  */
typedef struct {
    __IO uint32_t DR;                                       /*!< CRC Data register,             Address offset: 0x00 */
    __IO uint8_t IDR;                                       /*!< CRC Independent data register, Address offset: 0x04 */
         uint8_t __RESERVED0[3];
    union {
        struct {
            __IO uint32_t RESET : 1;                        /*!< RESET bit */
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;


typedef struct {
    __IO uint32_t DR[32];                                   /*!< CRC Data register,             Address offset: 0x00 */
    __IO uint32_t IDR[8];                                   /*!< CRC Independent data register, Address offset: 0x04 */
         uint32_t __RESERVED0[3][8];
    struct {
        __IO uint32_t RESET;                                /*!< RESET bit */
             uint32_t __RESERVED0[31];
    } CR;                                                   /*!< CRC Control register,          Address offset: 0x08 */
} CRC_BitBand_TypeDef;



/**
  * @brief Debug MCU
  */
typedef struct {
    union {
        struct {
            __IO uint32_t DEV_ID : 12;
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t REV_ID : 16;
        } b;
        __IO uint32_t w;
    } IDCODE;                                               /*!< MCU device ID code,               Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t DBG_SLEEP : 1;
            __IO uint32_t DBG_STOP : 1;
            __IO uint32_t DBG_STANDBY : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t TRACE_IOEN : 1;
            __IO uint32_t TRACE_MODE : 2;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< Debug MCU configuration register, Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t FZ_DBG_TIM2_STOP : 1;
            __IO uint32_t FZ_DBG_TIM3_STOP : 1;
            __IO uint32_t FZ_DBG_TIM4_STOP : 1;
            __IO uint32_t FZ_DBG_TIM5_STOP : 1;
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t FZ_DBG_RTC_STOP : 1;
            __IO uint32_t FZ_DBG_WWDG_STOP : 1;
            __IO uint32_t FZ_DBG_IWDG_STOP : 1;
                 uint32_t __RESERVED1 : 8;
            __IO uint32_t FZ_DBG_I2C1_SMBUS_TIMEOUT : 1;
            __IO uint32_t FZ_DBG_I2C2_SMBUS_TIMEOUT : 1;
            __IO uint32_t FZ_DBG_I2C3_SMBUS_TIMEOUT : 1;
                 uint32_t __RESERVED2 : 8;
        } b;
        __IO uint32_t w;
    } APB1FZ;                                               /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t FZ_DBG_TIM1_STOP : 1;
                 uint32_t __RESERVED0 : 15;
            __IO uint32_t FZ_DBG_TIM9_STOP : 1;
            __IO uint32_t FZ_DBG_TIM10_STOP : 1;
            __IO uint32_t FZ_DBG_TIM11_STOP : 1;
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } APB2FZ;                                               /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
} DBGMCU_TypeDef;



/**
  * @brief DMA Controller
  */
typedef struct {
    union {
        struct {
            __IO uint32_t EN : 1;
            __IO uint32_t DMEIE : 1;
            __IO uint32_t TEIE : 1;
            __IO uint32_t HTIE : 1;
            __IO uint32_t TCIE : 1;
            __IO uint32_t PFCTRL : 1;
            __IO uint32_t DIR : 2;
            __IO uint32_t CIRC : 1;
            __IO uint32_t PINC : 1;
            __IO uint32_t MINC : 1;
            __IO uint32_t PSIZE : 2;
            __IO uint32_t MSIZE : 2;
            __IO uint32_t PINCOS : 1;
            __IO uint32_t PL : 2;
            __IO uint32_t DBM : 1;
            __IO uint32_t CT : 1;
            __IO uint32_t ACK : 1;
            __IO uint32_t PBURST : 2;
            __IO uint32_t MBURST : 2;
            __IO uint32_t CHSEL : 3;
                 uint32_t __RESERVED0 : 4;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< DMA stream x configuration register      */
    __IO uint32_t NDTR;                                     /*!< DMA stream x number of data register     */
    __IO uint32_t PAR;                                      /*!< DMA stream x peripheral address register */
    __IO uint32_t M0AR;                                     /*!< DMA stream x memory 0 address register   */
    __IO uint32_t M1AR;                                     /*!< DMA stream x memory 1 address register   */
    union {
        struct {
            __IO uint32_t FTH : 2;
            __IO uint32_t DMDIS : 1;
            __IO uint32_t FS : 3;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t FEIE : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } FCR;                                                  /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN;
        __IO uint32_t DMEIE;
        __IO uint32_t TEIE;
        __IO uint32_t HTIE;
        __IO uint32_t TCIE;
        __IO uint32_t PFCTRL;
        __IO uint32_t DIR[2];
        __IO uint32_t CIRC;
        __IO uint32_t PINC;
        __IO uint32_t MINC;
        __IO uint32_t PSIZE[2];
        __IO uint32_t MSIZE[2];
        __IO uint32_t PINCOS;
        __IO uint32_t PL[2];
        __IO uint32_t DBM;
        __IO uint32_t CT;
        __IO uint32_t ACK;
        __IO uint32_t PBURST[2];
        __IO uint32_t MBURST[2];
        __IO uint32_t CHSEL[3];
             uint32_t __RESERVED0[4];
    } CR;                                                   /*!< DMA stream x configuration register      */
    __IO uint32_t NDTR[32];                                 /*!< DMA stream x number of data register     */
    __IO uint32_t PAR[32];                                  /*!< DMA stream x peripheral address register */
    __IO uint32_t M0AR[32];                                 /*!< DMA stream x memory 0 address register   */
    __IO uint32_t M1AR[32];                                 /*!< DMA stream x memory 1 address register   */
    struct {
        __IO uint32_t FTH[2];
        __IO uint32_t DMDIS;
        __IO uint32_t FS[3];
             uint32_t __RESERVED0;
        __IO uint32_t FEIE;
             uint32_t __RESERVED1[24];
    } FCR;                                                  /*!< DMA stream x FIFO control register       */
} DMA_Stream_BitBand_TypeDef;



typedef struct {
    union {
        struct {
            __IO uint32_t FEIF0 : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DMEIF0 : 1;
            __IO uint32_t TEIF0 : 1;
            __IO uint32_t HTIF0 : 1;
            __IO uint32_t TCIF0 : 1;
            __IO uint32_t FEIF1 : 1;
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t DMEIF1 : 1;
            __IO uint32_t TEIF1 : 1;
            __IO uint32_t HTIF1 : 1;
            __IO uint32_t TCIF1 : 1;
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t FEIF2 : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t DMEIF2 : 1;
            __IO uint32_t TEIF2 : 1;
            __IO uint32_t HTIF2 : 1;
            __IO uint32_t TCIF2 : 1;
            __IO uint32_t FEIF3 : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t DMEIF3 : 1;
            __IO uint32_t TEIF3 : 1;
            __IO uint32_t HTIF3 : 1;
            __IO uint32_t TCIF3 : 1;
                 uint32_t __RESERVED5 : 4;
        } b;
        __IO uint32_t w;
    } LISR;                                                 /*!< DMA low interrupt status register,      Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t FEIF4 : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DMEIF4 : 1;
            __IO uint32_t TEIF4 : 1;
            __IO uint32_t HTIF4 : 1;
            __IO uint32_t TCIF4 : 1;
            __IO uint32_t FEIF5 : 1;
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t DMEIF5 : 1;
            __IO uint32_t TEIF5 : 1;
            __IO uint32_t HTIF5 : 1;
            __IO uint32_t TCIF5 : 1;
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t FEIF6 : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t DMEIF6 : 1;
            __IO uint32_t TEIF6 : 1;
            __IO uint32_t HTIF6 : 1;
            __IO uint32_t TCIF6 : 1;
            __IO uint32_t FEIF7 : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t DMEIF7 : 1;
            __IO uint32_t TEIF7 : 1;
            __IO uint32_t HTIF7 : 1;
            __IO uint32_t TCIF7 : 1;
                 uint32_t __RESERVED5 : 4;
        } b;
        __IO uint32_t w;
    } HISR;                                                 /*!< DMA high interrupt status register,     Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t CFEIF0 : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CDMEIF0 : 1;
            __IO uint32_t CTEIF0 : 1;
            __IO uint32_t CHTIF0 : 1;
            __IO uint32_t CTCIF0 : 1;
            __IO uint32_t CFEIF1 : 1;
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t CDMEIF1 : 1;
            __IO uint32_t CTEIF1 : 1;
            __IO uint32_t CHTIF1 : 1;
            __IO uint32_t CTCIF1 : 1;
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t CFEIF2 : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CDMEIF2 : 1;
            __IO uint32_t CTEIF2 : 1;
            __IO uint32_t CHTIF2 : 1;
            __IO uint32_t CTCIF2 : 1;
            __IO uint32_t CFEIF3 : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t CDMEIF3 : 1;
            __IO uint32_t CTEIF3 : 1;
            __IO uint32_t CHTIF3 : 1;
            __IO uint32_t CTCIF3 : 1;
                 uint32_t __RESERVED5 : 4;
        } b;
        __IO uint32_t w;
    } LIFCR;                                                /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t CFEIF4 : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CDMEIF4 : 1;
            __IO uint32_t CTEIF4 : 1;
            __IO uint32_t CHTIF4 : 1;
            __IO uint32_t CTCIF4 : 1;
            __IO uint32_t CFEIF5 : 1;
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t CDMEIF5 : 1;
            __IO uint32_t CTEIF5 : 1;
            __IO uint32_t CHTIF5 : 1;
            __IO uint32_t CTCIF5 : 1;
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t CFEIF6 : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CDMEIF6 : 1;
            __IO uint32_t CTEIF6 : 1;
            __IO uint32_t CHTIF6 : 1;
            __IO uint32_t CTCIF6 : 1;
            __IO uint32_t CFEIF7 : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t CDMEIF7 : 1;
            __IO uint32_t CTEIF7 : 1;
            __IO uint32_t CHTIF7 : 1;
            __IO uint32_t CTCIF7 : 1;
                 uint32_t __RESERVED5 : 4;
        } b;
        __IO uint32_t w;
    } HIFCR;                                                /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;


typedef struct {
    struct {
        __IO uint32_t FEIF0;
             uint32_t __RESERVED0;
        __IO uint32_t DMEIF0;
        __IO uint32_t TEIF0;
        __IO uint32_t HTIF0;
        __IO uint32_t TCIF0;
        __IO uint32_t FEIF1;
             uint32_t __RESERVED1;
        __IO uint32_t DMEIF1;
        __IO uint32_t TEIF1;
        __IO uint32_t HTIF1;
        __IO uint32_t TCIF1;
             uint32_t __RESERVED2[4];
        __IO uint32_t FEIF2;
             uint32_t __RESERVED3;
        __IO uint32_t DMEIF2;
        __IO uint32_t TEIF2;
        __IO uint32_t HTIF2;
        __IO uint32_t TCIF2;
        __IO uint32_t FEIF3;
             uint32_t __RESERVED4;
        __IO uint32_t DMEIF3;
        __IO uint32_t TEIF3;
        __IO uint32_t HTIF3;
        __IO uint32_t TCIF3;
             uint32_t __RESERVED5[4];
    } LISR;                                                 /*!< DMA low interrupt status register,      Address offset: 0x00 */
    struct {
        __IO uint32_t FEIF4;
             uint32_t __RESERVED0;
        __IO uint32_t DMEIF4;
        __IO uint32_t TEIF4;
        __IO uint32_t HTIF4;
        __IO uint32_t TCIF4;
        __IO uint32_t FEIF5;
             uint32_t __RESERVED1;
        __IO uint32_t DMEIF5;
        __IO uint32_t TEIF5;
        __IO uint32_t HTIF5;
        __IO uint32_t TCIF5;
             uint32_t __RESERVED2[4];
        __IO uint32_t FEIF6;
             uint32_t __RESERVED3;
        __IO uint32_t DMEIF6;
        __IO uint32_t TEIF6;
        __IO uint32_t HTIF6;
        __IO uint32_t TCIF6;
        __IO uint32_t FEIF7;
             uint32_t __RESERVED4;
        __IO uint32_t DMEIF7;
        __IO uint32_t TEIF7;
        __IO uint32_t HTIF7;
        __IO uint32_t TCIF7;
             uint32_t __RESERVED5[4];
    } HISR;                                                 /*!< DMA high interrupt status register,     Address offset: 0x04 */
    struct {
        __IO uint32_t CFEIF0;
             uint32_t __RESERVED0;
        __IO uint32_t CDMEIF0;
        __IO uint32_t CTEIF0;
        __IO uint32_t CHTIF0;
        __IO uint32_t CTCIF0;
        __IO uint32_t CFEIF1;
             uint32_t __RESERVED1;
        __IO uint32_t CDMEIF1;
        __IO uint32_t CTEIF1;
        __IO uint32_t CHTIF1;
        __IO uint32_t CTCIF1;
             uint32_t __RESERVED2[4];
        __IO uint32_t CFEIF2;
             uint32_t __RESERVED3;
        __IO uint32_t CDMEIF2;
        __IO uint32_t CTEIF2;
        __IO uint32_t CHTIF2;
        __IO uint32_t CTCIF2;
        __IO uint32_t CFEIF3;
             uint32_t __RESERVED4;
        __IO uint32_t CDMEIF3;
        __IO uint32_t CTEIF3;
        __IO uint32_t CHTIF3;
        __IO uint32_t CTCIF3;
             uint32_t __RESERVED5[4];
    } LIFCR;                                                /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
    struct {
        __IO uint32_t CFEIF4;
             uint32_t __RESERVED0;
        __IO uint32_t CDMEIF4;
        __IO uint32_t CTEIF4;
        __IO uint32_t CHTIF4;
        __IO uint32_t CTCIF4;
        __IO uint32_t CFEIF5;
             uint32_t __RESERVED1;
        __IO uint32_t CDMEIF5;
        __IO uint32_t CTEIF5;
        __IO uint32_t CHTIF5;
        __IO uint32_t CTCIF5;
             uint32_t __RESERVED2[4];
        __IO uint32_t CFEIF6;
             uint32_t __RESERVED3;
        __IO uint32_t CDMEIF6;
        __IO uint32_t CTEIF6;
        __IO uint32_t CHTIF6;
        __IO uint32_t CTCIF6;
        __IO uint32_t CFEIF7;
             uint32_t __RESERVED4;
        __IO uint32_t CDMEIF7;
        __IO uint32_t CTEIF7;
        __IO uint32_t CHTIF7;
        __IO uint32_t CTCIF7;
             uint32_t __RESERVED5[4];
    } HIFCR;                                                /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_BitBand_TypeDef;



/**
  * @brief External Interrupt/Event Controller
  */
typedef struct {
    __IO uint32_t IMR;                                      /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
    __IO uint32_t EMR;                                      /*!< EXTI Event mask register,                Address offset: 0x04 */
    __IO uint32_t RTSR;                                     /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
    __IO uint32_t FTSR;                                     /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
    __IO uint32_t SWIER;                                    /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
    __IO uint32_t PR;                                       /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;


typedef struct {
    __IO uint32_t IMR[32];                                  /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
    __IO uint32_t EMR[32];                                  /*!< EXTI Event mask register,                Address offset: 0x04 */
    __IO uint32_t RTSR[32];                                 /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
    __IO uint32_t FTSR[32];                                 /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
    __IO uint32_t SWIER[32];                                /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
    __IO uint32_t PR[32];                                   /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_BitBand_TypeDef;



/**
  * @brief FLASH Registers
  */
typedef struct {
    union {
        struct {
            __IO uint32_t LATENCY : 4;
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t PRFTEN : 1;
            __IO uint32_t ICEN : 1;
            __IO uint32_t DCEN : 1;
            __IO uint32_t ICRST : 1;
            __IO uint32_t DCRST : 1;
                 uint32_t __RESERVED1 : 19;
        } b;
        __IO uint32_t w;
    } ACR;                                                  /*!< FLASH access control register,   Address offset: 0x00 */
    __IO uint32_t KEYR;                                     /*!< FLASH key register,              Address offset: 0x04 */
    __IO uint32_t OPTKEYR;                                  /*!< FLASH option key register,       Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t EOP : 1;
            __IO uint32_t SOP : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t WRPERR : 1;
            __IO uint32_t PGAERR : 1;
            __IO uint32_t PGPERR : 1;
            __IO uint32_t PGSERR : 1;
            __IO uint32_t RDERR : 1;
                 uint32_t __RESERVED1 : 7;
            __IO uint32_t BSY : 1;
                 uint32_t __RESERVED2 : 15;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< FLASH status register,           Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t PG : 1;
            __IO uint32_t SER : 1;
            __IO uint32_t MER : 1;
            __IO uint32_t SNB : 5;
            __IO uint32_t PSIZE : 2;
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t STRT : 1;
                 uint32_t __RESERVED1 : 7;
            __IO uint32_t EOPIE : 1;
                 uint32_t __RESERVED2 : 6;
            __IO uint32_t LOCK : 1;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< FLASH control register,          Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t OPTLOCK : 1;
            __IO uint32_t OPTSTRT : 1;
            __IO uint32_t BOR_LEV : 2;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t WDG_SW : 1;
            __IO uint32_t nRST_STOP : 1;
            __IO uint32_t nRST_STDBY : 1;
            __IO uint32_t RDP : 8;
            __IO uint32_t nWRP : 12;
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } OPTCR;                                                /*!< FLASH option control register ,  Address offset: 0x14 */
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t nWRP : 12;
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } OPTCR1;                                               /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;


typedef struct {
    struct {
        __IO uint32_t LATENCY[4];
             uint32_t __RESERVED0[4];
        __IO uint32_t PRFTEN;
        __IO uint32_t ICEN;
        __IO uint32_t DCEN;
        __IO uint32_t ICRST;
        __IO uint32_t DCRST;
             uint32_t __RESERVED1[19];
    } ACR;                                                  /*!< FLASH access control register,   Address offset: 0x00 */
    __IO uint32_t KEYR[32];                                 /*!< FLASH key register,              Address offset: 0x04 */
    __IO uint32_t OPTKEYR[32];                              /*!< FLASH option key register,       Address offset: 0x08 */
    struct {
        __IO uint32_t EOP;
        __IO uint32_t SOP;
             uint32_t __RESERVED0[2];
        __IO uint32_t WRPERR;
        __IO uint32_t PGAERR;
        __IO uint32_t PGPERR;
        __IO uint32_t PGSERR;
        __IO uint32_t RDERR;
             uint32_t __RESERVED1[7];
        __IO uint32_t BSY;
             uint32_t __RESERVED2[15];
    } SR;                                                   /*!< FLASH status register,           Address offset: 0x0C */
    struct {
        __IO uint32_t PG;
        __IO uint32_t SER;
        __IO uint32_t MER;
        __IO uint32_t SNB[5];
        __IO uint32_t PSIZE[2];
             uint32_t __RESERVED0[6];
        __IO uint32_t STRT;
             uint32_t __RESERVED1[7];
        __IO uint32_t EOPIE;
             uint32_t __RESERVED2[6];
        __IO uint32_t LOCK;
    } CR;                                                   /*!< FLASH control register,          Address offset: 0x10 */
    struct {
        __IO uint32_t OPTLOCK;
        __IO uint32_t OPTSTRT;
        __IO uint32_t BOR_LEV[2];
             uint32_t __RESERVED0;
        __IO uint32_t WDG_SW;
        __IO uint32_t nRST_STOP;
        __IO uint32_t nRST_STDBY;
        __IO uint32_t RDP[8];
        __IO uint32_t nWRP[12];
             uint32_t __RESERVED1[4];
    } OPTCR;                                                /*!< FLASH option control register ,  Address offset: 0x14 */
    struct {
             uint32_t __RESERVED0[16];
        __IO uint32_t nWRP[12];
             uint32_t __RESERVED1[4];
    } OPTCR1;                                               /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_BitBand_TypeDef;



/**
  * @brief General Purpose I/O
  */
typedef struct {
    union {
        struct {
            __IO uint32_t MODER0 : 2;
            __IO uint32_t MODER1 : 2;
            __IO uint32_t MODER2 : 2;
            __IO uint32_t MODER3 : 2;
            __IO uint32_t MODER4 : 2;
            __IO uint32_t MODER5 : 2;
            __IO uint32_t MODER6 : 2;
            __IO uint32_t MODER7 : 2;
            __IO uint32_t MODER8 : 2;
            __IO uint32_t MODER9 : 2;
            __IO uint32_t MODER10 : 2;
            __IO uint32_t MODER11 : 2;
            __IO uint32_t MODER12 : 2;
            __IO uint32_t MODER13 : 2;
            __IO uint32_t MODER14 : 2;
            __IO uint32_t MODER15 : 2;
        } b;
        __IO uint32_t w;
    } MODER;                                                /*!< GPIO port mode register,                     Address offset: 0x00      */
    __IO uint32_t OTYPER;                                   /*!< GPIO port output type register,              Address offset: 0x04      */
    union {
        struct {
            __IO uint32_t OSPEEDR0 : 2;
            __IO uint32_t OSPEEDR1 : 2;
            __IO uint32_t OSPEEDR2 : 2;
            __IO uint32_t OSPEEDR3 : 2;
            __IO uint32_t OSPEEDR4 : 2;
            __IO uint32_t OSPEEDR5 : 2;
            __IO uint32_t OSPEEDR6 : 2;
            __IO uint32_t OSPEEDR7 : 2;
            __IO uint32_t OSPEEDR8 : 2;
            __IO uint32_t OSPEEDR9 : 2;
            __IO uint32_t OSPEEDR10 : 2;
            __IO uint32_t OSPEEDR11 : 2;
            __IO uint32_t OSPEEDR12 : 2;
            __IO uint32_t OSPEEDR13 : 2;
            __IO uint32_t OSPEEDR14 : 2;
            __IO uint32_t OSPEEDR15 : 2;
        } b;
        __IO uint32_t w;
    } OSPEEDR;                                              /*!< GPIO port output speed register,             Address offset: 0x08      */
    union {
        struct {
            __IO uint32_t PUPDR0 : 2;
            __IO uint32_t PUPDR1 : 2;
            __IO uint32_t PUPDR2 : 2;
            __IO uint32_t PUPDR3 : 2;
            __IO uint32_t PUPDR4 : 2;
            __IO uint32_t PUPDR5 : 2;
            __IO uint32_t PUPDR6 : 2;
            __IO uint32_t PUPDR7 : 2;
            __IO uint32_t PUPDR8 : 2;
            __IO uint32_t PUPDR9 : 2;
            __IO uint32_t PUPDR10 : 2;
            __IO uint32_t PUPDR11 : 2;
            __IO uint32_t PUPDR12 : 2;
            __IO uint32_t PUPDR13 : 2;
            __IO uint32_t PUPDR14 : 2;
            __IO uint32_t PUPDR15 : 2;
        } b;
        __IO uint32_t w;
    } PUPDR;                                                /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    __I  uint32_t IDR;                                      /*!< GPIO port input data register,         Address offset: 0x10      */
    __IO uint32_t ODR;                                      /*!< GPIO port output data register,        Address offset: 0x14      */
    __IO uint32_t BSRR;                                     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
    union {
        struct {
            __IO uint32_t LCK0 : 1;
            __IO uint32_t LCK1 : 1;
            __IO uint32_t LCK2 : 1;
            __IO uint32_t LCK3 : 1;
            __IO uint32_t LCK4 : 1;
            __IO uint32_t LCK5 : 1;
            __IO uint32_t LCK6 : 1;
            __IO uint32_t LCK7 : 1;
            __IO uint32_t LCK8 : 1;
            __IO uint32_t LCK9 : 1;
            __IO uint32_t LCK10 : 1;
            __IO uint32_t LCK11 : 1;
            __IO uint32_t LCK12 : 1;
            __IO uint32_t LCK13 : 1;
            __IO uint32_t LCK14 : 1;
            __IO uint32_t LCK15 : 1;
            __IO uint32_t LCKK : 1;
                 uint32_t __RESERVED0 : 15;
        } b;
        __IO uint32_t w;
    } LCKR;                                                 /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    __IO uint32_t AFR[2];                                   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;


typedef struct {
    __IO uint32_t MODER[16][2];                             /*!< GPIO port mode register,               Address offset: 0x00      */
    __IO uint32_t OTYPER[32];                               /*!< GPIO port output type register,        Address offset: 0x04      */
    __IO uint32_t OSPEEDR[16][2];                           /*!< GPIO port output speed register,       Address offset: 0x08      */
    __IO uint32_t PUPDR[16][2];                             /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    __IO uint32_t IDR[32];                                  /*!< GPIO port input data register,         Address offset: 0x10      */
    __IO uint32_t ODR[32];                                  /*!< GPIO port output data register,        Address offset: 0x14      */
    __IO uint32_t BS[16];                                   /*!< GPIO port bit set register,            Address offset: 0x18      */
    __IO uint32_t BR[16];                                   /*!< GPIO port bit reset register,          Address offset: 0x1A      */
    struct {
        __IO uint32_t LCK[16];
        __IO uint32_t LCKK;
             uint32_t __RESERVED0[15];
    } LCKR;                                                 /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    __IO uint32_t AFR[2][32];                               /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_BitBand_TypeDef;



/**
  * @brief System configuration controller
  */
typedef struct {
    union {
        struct {
            __IO uint32_t MEM_MODE : 2;                     /*!< SYSCFG_Memory Remap Config */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } MEMRMP;                                               /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t ADC1DC2 : 1;                      /*!< Refer to AN4073 on how to use this bit  */
                 uint32_t __RESERVED1 : 15;
        } b;
        __IO uint32_t w;
    } PMC;                                                  /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
    __IO uint32_t EXTICR[4];                                /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
         uint32_t __RESERVED0[2];
    union {
        struct {
            __IO uint32_t CMP_PD : 1;                       /*!<Compensation cell ready flag */
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t READY : 1;                        /*!<Compensation cell power-down */
                 uint32_t __RESERVED1 : 23;
        } b;
        __IO uint32_t w;
    } CMPCR;                                                /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;


typedef struct {
    struct {
        __IO uint32_t MEM_MODE[2];                          /*!< SYSCFG_Memory Remap Config */
             uint32_t __RESERVED0[30];
    } MEMRMP;                                               /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
    struct {
             uint32_t __RESERVED0[16];
        __IO uint32_t ADC1DC2;                              /*!< Refer to AN4073 on how to use this bit  */
             uint32_t __RESERVED1[15];
    } PMC;                                                  /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
    __IO uint32_t EXTICR[4][32];                            /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
         uint32_t __RESERVED0[2][32];
    struct {
        __IO uint32_t CMP_PD;                               /*!<Compensation cell ready flag */
             uint32_t __RESERVED0[7];
        __IO uint32_t READY;                                /*!<Compensation cell power-down */
             uint32_t __RESERVED1[23];
    } CMPCR;                                                /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_BitBand_TypeDef;



/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct {
    union {
        struct {
            __IO uint32_t PE : 1;                           /*!<Peripheral Enable                             */
            __IO uint32_t SMBUS : 1;                        /*!<SMBus Mode                                    */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SMBTYPE : 1;                      /*!<SMBus Type                                    */
            __IO uint32_t ENARP : 1;                        /*!<ARP Enable                                    */
            __IO uint32_t ENPEC : 1;                        /*!<PEC Enable                                    */
            __IO uint32_t ENGC : 1;                         /*!<General Call Enable                           */
            __IO uint32_t NOSTRETCH : 1;                    /*!<Clock Stretching Disable (Slave mode)         */
            __IO uint32_t START : 1;                        /*!<Start Generation                              */
            __IO uint32_t STOP : 1;                         /*!<Stop Generation                               */
            __IO uint32_t ACK : 1;                          /*!<Acknowledge Enable                            */
            __IO uint32_t POS : 1;                          /*!<Acknowledge/PEC Position (for data reception) */
            __IO uint32_t PEC : 1;                          /*!<Packet Error Checking                         */
            __IO uint32_t ALERT : 1;                        /*!<SMBus Alert                                   */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SWRST : 1;                        /*!<Software Reset                                */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< I2C Control register 1,     Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t FREQ : 6;                         /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t ITERREN : 1;                      /*!<Error Interrupt Enable  */
            __IO uint32_t ITEVTEN : 1;                      /*!<Event Interrupt Enable  */
            __IO uint32_t ITBUFEN : 1;                      /*!<Buffer Interrupt Enable */
            __IO uint32_t DMAEN : 1;                        /*!<DMA Requests Enable     */
            __IO uint32_t LAST : 1;                         /*!<DMA Last Transfer       */
                 uint32_t __RESERVED1 : 19;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< I2C Control register 2,     Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t ADD1 : 10;                        /*!< Interface own address 1 */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t ADDMODE : 1;                      /*!<Addressing Mode (Slave mode) */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } OAR1;                                                 /*!< I2C Own address register 1, Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t ENDUAL : 1;                       /*!<Dual addressing mode enable */
            __IO uint32_t ADD2 : 7;                         /*!< Interface own address 2 */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } OAR2;                                                 /*!< I2C Own address register 2, Address offset: 0x0C */
    __IO uint32_t DR;                                       /*!< I2C Data register,          Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t SB : 1;                           /*!<Start Bit (Master mode)                         */
            __IO uint32_t ADDR : 1;                         /*!<Address sent (master mode)/matched (slave mode) */
            __IO uint32_t BTF : 1;                          /*!<Byte Transfer Finished                          */
            __IO uint32_t ADD10 : 1;                        /*!<10-bit header sent (Master mode)                */
            __IO uint32_t STOPF : 1;                        /*!<Stop detection (Slave mode)                     */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RXNE : 1;                         /*!<Data Register not Empty (receivers)             */
            __IO uint32_t TXE : 1;                          /*!<Data Register Empty (transmitters)              */
            __IO uint32_t BERR : 1;                         /*!<Bus Error                                       */
            __IO uint32_t ARLO : 1;                         /*!<Arbitration Lost (master mode)                  */
            __IO uint32_t AF : 1;                           /*!<Acknowledge Failure                             */
            __IO uint32_t OVR : 1;                          /*!<Overrun/Underrun                                */
            __IO uint32_t PECERR : 1;                       /*!<PEC Error in reception                          */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t TIMEOUT : 1;                      /*!<Timeout or Tlow Error                           */
            __IO uint32_t SMBALERT : 1;                     /*!<SMBus Alert                                     */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } SR1;                                                  /*!< I2C Status register 1,      Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t MSL : 1;                          /*!<Master/Slave                                    */
            __IO uint32_t BUSY : 1;                         /*!<Bus Busy                                        */
            __IO uint32_t TRA : 1;                          /*!<Transmitter/Receiver                            */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t GENCALL : 1;                      /*!<General Call Address (Slave mode)               */
            __IO uint32_t SMBDEFAULT : 1;                   /*!<SMBus Device Default Address (Slave mode)       */
            __IO uint32_t SMBHOST : 1;                      /*!<SMBus Host Header (Slave mode)                  */
            __IO uint32_t DUALF : 1;                        /*!<Dual Flag (Slave mode)                          */
            __IO uint32_t PEC : 8;                          /*!<Packet Error Checking Register                  */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } SR2;                                                  /*!< I2C Status register 2,      Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t CCR : 12;                         /*!<Clock Control Register in Fast/Standard mode (Master mode) */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t DUTY : 1;                         /*!<Fast Mode Duty Cycle                                       */
            __IO uint32_t FS : 1;                           /*!<I2C Master Mode Selection                                  */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CCR;                                                  /*!< I2C Clock control register, Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t TRISE : 6;                        /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */
                 uint32_t __RESERVED0 : 26;
        } b;
        __IO uint32_t w;
    } TRISE;                                                /*!< I2C TRISE register,         Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t DNF : 4;                          /*!<Digital Noise Filter */
            __IO uint32_t ANOFF : 1;                        /*!<Analog Noise Filter OFF */
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } FLTR;                                                 /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PE;                                   /*!<Peripheral Enable                             */
        __IO uint32_t SMBUS;                                /*!<SMBus Mode                                    */
             uint32_t __RESERVED0;
        __IO uint32_t SMBTYPE;                              /*!<SMBus Type                                    */
        __IO uint32_t ENARP;                                /*!<ARP Enable                                    */
        __IO uint32_t ENPEC;                                /*!<PEC Enable                                    */
        __IO uint32_t ENGC;                                 /*!<General Call Enable                           */
        __IO uint32_t NOSTRETCH;                            /*!<Clock Stretching Disable (Slave mode)         */
        __IO uint32_t START;                                /*!<Start Generation                              */
        __IO uint32_t STOP;                                 /*!<Stop Generation                               */
        __IO uint32_t ACK;                                  /*!<Acknowledge Enable                            */
        __IO uint32_t POS;                                  /*!<Acknowledge/PEC Position (for data reception) */
        __IO uint32_t PEC;                                  /*!<Packet Error Checking                         */
        __IO uint32_t ALERT;                                /*!<SMBus Alert                                   */
             uint32_t __RESERVED1;
        __IO uint32_t SWRST;                                /*!<Software Reset                                */
             uint32_t __RESERVED2[16];
    } CR1;                                                  /*!< I2C Control register 1,     Address offset: 0x00 */
    struct {
        __IO uint32_t FREQ[6];                              /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
             uint32_t __RESERVED0[2];
        __IO uint32_t ITERREN;                              /*!<Error Interrupt Enable  */
        __IO uint32_t ITEVTEN;                              /*!<Event Interrupt Enable  */
        __IO uint32_t ITBUFEN;                              /*!<Buffer Interrupt Enable */
        __IO uint32_t DMAEN;                                /*!<DMA Requests Enable     */
        __IO uint32_t LAST;                                 /*!<DMA Last Transfer       */
             uint32_t __RESERVED1[19];
    } CR2;                                                  /*!< I2C Control register 2,     Address offset: 0x04 */
    struct {
        __IO uint32_t ADD1[10];                             /*!< Interface own address 1 */
             uint32_t __RESERVED0[5];
        __IO uint32_t ADDMODE;                              /*!<Addressing Mode (Slave mode) */
             uint32_t __RESERVED1[16];
    } OAR1;                                                 /*!< I2C Own address register 1, Address offset: 0x08 */
    struct {
        __IO uint32_t ENDUAL;                               /*!<Dual addressing mode enable */
        __IO uint32_t ADD2[7];                              /*!< Interface own address 2 */
             uint32_t __RESERVED0[24];
    } OAR2;                                                 /*!< I2C Own address register 2, Address offset: 0x0C */
    __IO uint32_t DR[32];                                   /*!< I2C Data register,          Address offset: 0x10 */
    struct {
        __IO uint32_t SB;                                   /*!<Start Bit (Master mode)                         */
        __IO uint32_t ADDR;                                 /*!<Address sent (master mode)/matched (slave mode) */
        __IO uint32_t BTF;                                  /*!<Byte Transfer Finished                          */
        __IO uint32_t ADD10;                                /*!<10-bit header sent (Master mode)                */
        __IO uint32_t STOPF;                                /*!<Stop detection (Slave mode)                     */
             uint32_t __RESERVED0;
        __IO uint32_t RXNE;                                 /*!<Data Register not Empty (receivers)             */
        __IO uint32_t TXE;                                  /*!<Data Register Empty (transmitters)              */
        __IO uint32_t BERR;                                 /*!<Bus Error                                       */
        __IO uint32_t ARLO;                                 /*!<Arbitration Lost (master mode)                  */
        __IO uint32_t AF;                                   /*!<Acknowledge Failure                             */
        __IO uint32_t OVR;                                  /*!<Overrun/Underrun                                */
        __IO uint32_t PECERR;                               /*!<PEC Error in reception                          */
             uint32_t __RESERVED1;
        __IO uint32_t TIMEOUT;                              /*!<Timeout or Tlow Error                           */
        __IO uint32_t SMBALERT;                             /*!<SMBus Alert                                     */
             uint32_t __RESERVED2[16];
    } SR1;                                                  /*!< I2C Status register 1,      Address offset: 0x14 */
    struct {
        __IO uint32_t MSL;                                  /*!<Master/Slave                                    */
        __IO uint32_t BUSY;                                 /*!<Bus Busy                                        */
        __IO uint32_t TRA;                                  /*!<Transmitter/Receiver                            */
             uint32_t __RESERVED0;
        __IO uint32_t GENCALL;                              /*!<General Call Address (Slave mode)               */
        __IO uint32_t SMBDEFAULT;                           /*!<SMBus Device Default Address (Slave mode)       */
        __IO uint32_t SMBHOST;                              /*!<SMBus Host Header (Slave mode)                  */
        __IO uint32_t DUALF;                                /*!<Dual Flag (Slave mode)                          */
        __IO uint32_t PEC[8];                               /*!<Packet Error Checking Register                  */
             uint32_t __RESERVED1[16];
    } SR2;                                                  /*!< I2C Status register 2,      Address offset: 0x18 */
    struct {
        __IO uint32_t CCR[12];                              /*!<Clock Control Register in Fast/Standard mode (Master mode) */
             uint32_t __RESERVED0[2];
        __IO uint32_t DUTY;                                 /*!<Fast Mode Duty Cycle                                       */
        __IO uint32_t FS;                                   /*!<I2C Master Mode Selection                                  */
             uint32_t __RESERVED1[16];
    } CCR;                                                  /*!< I2C Clock control register, Address offset: 0x1C */
    struct {
        __IO uint32_t TRISE[6];                             /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */
             uint32_t __RESERVED0[26];
    } TRISE;                                                /*!< I2C TRISE register,         Address offset: 0x20 */
    struct {
        __IO uint32_t DNF[4];                               /*!<Digital Noise Filter */
        __IO uint32_t ANOFF;                                /*!<Analog Noise Filter OFF */
             uint32_t __RESERVED0[27];
    } FLTR;                                                 /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_BitBand_TypeDef;



/** 
  * @brief Independent WATCHDOG
  */
typedef struct {
    __IO uint32_t KR;                                       /*!< IWDG Key register,       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t PR : 3;                           /*!<PR[2:0] (Prescaler divider)         */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } PR;                                                   /*!< IWDG Prescaler register, Address offset: 0x04 */
    __IO uint32_t RLR;                                      /*!< IWDG Reload register,    Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t PVU : 1;                          /*!<Watchdog prescaler value update      */
            __IO uint32_t RVU : 1;                          /*!<Watchdog counter reload value update */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;


typedef struct {
    __IO uint32_t KR[32];                                   /*!< IWDG Key register,       Address offset: 0x00 */
    struct {
        __IO uint32_t PR[3];                                /*!<PR[2:0] (Prescaler divider)         */
             uint32_t __RESERVED0[29];
    } PR;                                                   /*!< IWDG Prescaler register, Address offset: 0x04 */
    __IO uint32_t RLR[32];                                  /*!< IWDG Reload register,    Address offset: 0x08 */
    struct {
        __IO uint32_t PVU;                                  /*!<Watchdog prescaler value update      */
        __IO uint32_t RVU;                                  /*!<Watchdog counter reload value update */
             uint32_t __RESERVED0[30];
    } SR;                                                   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_BitBand_TypeDef;



/**
  * @brief Power Control
  */
typedef struct {
    union {
        struct {
            __IO uint32_t LPDS : 1;                         /*!< Low-Power Deepsleep                 */
            __IO uint32_t PDDS : 1;                         /*!< Power Down Deepsleep                */
            __IO uint32_t CWUF : 1;                         /*!< Clear Wakeup Flag                   */
            __IO uint32_t CSBF : 1;                         /*!< Clear Standby Flag                  */
            __IO uint32_t PVDE : 1;                         /*!< Power Voltage Detector Enable       */
            __IO uint32_t PLS : 3;                          /*!< PLS[2:0] bits (PVD Level Selection) */
            __IO uint32_t DBP : 1;                          /*!< Disable Backup Domain write protection                     */
            __IO uint32_t FPDS : 1;                         /*!< Flash power down in Stop mode                              */
            __IO uint32_t LPLVDS : 1;                       /*!< Low Power Regulator Low Voltage in Deep Sleep mode         */
            __IO uint32_t MRLVDS : 1;                       /*!< Main Regulator Low Voltage in Deep Sleep mode              */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t ADCDC1 : 1;                       /*!< Refer to AN4073 on how to use this bit                     */
            __IO uint32_t VOS : 2;                          /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< PWR power control register,        Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t WUF : 1;                          /*!< Wakeup Flag                                      */
            __IO uint32_t SBF : 1;                          /*!< Standby Flag                                     */
            __IO uint32_t PVDO : 1;                         /*!< PVD Output                                       */
            __IO uint32_t BRR : 1;                          /*!< Backup regulator ready                           */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t EWUP : 1;                         /*!< Enable WKUP pin                                  */
            __IO uint32_t BRE : 1;                          /*!< Backup regulator enable                          */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t VOSRDY : 1;                       /*!< Regulator voltage scaling output selection ready */
                 uint32_t __RESERVED2 : 17;
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;


typedef struct {
    struct {
        __IO uint32_t LPDS;                                 /*!< Low-Power Deepsleep                 */
        __IO uint32_t PDDS;                                 /*!< Power Down Deepsleep                */
        __IO uint32_t CWUF;                                 /*!< Clear Wakeup Flag                   */
        __IO uint32_t CSBF;                                 /*!< Clear Standby Flag                  */
        __IO uint32_t PVDE;                                 /*!< Power Voltage Detector Enable       */
        __IO uint32_t PLS[3];                               /*!< PLS[2:0] bits (PVD Level Selection) */
        __IO uint32_t DBP;                                  /*!< Disable Backup Domain write protection                     */
        __IO uint32_t FPDS;                                 /*!< Flash power down in Stop mode                              */
        __IO uint32_t LPLVDS;                               /*!< Low Power Regulator Low Voltage in Deep Sleep mode         */
        __IO uint32_t MRLVDS;                               /*!< Main Regulator Low Voltage in Deep Sleep mode              */
             uint32_t __RESERVED0;
        __IO uint32_t ADCDC1;                               /*!< Refer to AN4073 on how to use this bit                     */
        __IO uint32_t VOS[2];                               /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
             uint32_t __RESERVED1[16];
    } CR;                                                   /*!< PWR power control register,        Address offset: 0x00 */
    struct {
        __IO uint32_t WUF;                                  /*!< Wakeup Flag                                      */
        __IO uint32_t SBF;                                  /*!< Standby Flag                                     */
        __IO uint32_t PVDO;                                 /*!< PVD Output                                       */
        __IO uint32_t BRR;                                  /*!< Backup regulator ready                           */
             uint32_t __RESERVED0[4];
        __IO uint32_t EWUP;                                 /*!< Enable WKUP pin                                  */
        __IO uint32_t BRE;                                  /*!< Backup regulator enable                          */
             uint32_t __RESERVED1[4];
        __IO uint32_t VOSRDY;                               /*!< Regulator voltage scaling output selection ready */
             uint32_t __RESERVED2[17];
    } CSR;                                                  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_BitBand_TypeDef;



/**
  * @brief Reset and Clock Control
  */
typedef struct {
    union {
        struct {
            __IO uint32_t HSION : 1;
            __IO uint32_t HSIRDY : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t HSITRIM : 5;
            __IO uint32_t HSICAL : 8;
            __IO uint32_t HSEON : 1;
            __IO uint32_t HSERDY : 1;
            __IO uint32_t HSEBYP : 1;
            __IO uint32_t CSSON : 1;
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t PLLON : 1;
            __IO uint32_t PLLRDY : 1;
            __IO uint32_t PLLI2SON : 1;
            __IO uint32_t PLLI2SRDY : 1;
                 uint32_t __RESERVED2 : 4;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< RCC clock control register,                                  Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t PLLM : 6;
            __IO uint32_t PLLN : 9;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PLLP : 2;
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t PLLSRC : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t PLLQ : 4;
                 uint32_t __RESERVED3 : 4;
        } b;
        __IO uint32_t w;
    } PLLCFGR;                                              /*!< RCC PLL configuration register,                              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t SW : 2;                           /*!< SW[1:0] bits (System clock Switch) */
            __IO uint32_t SWS : 2;                          /*!< SWS[1:0] bits (System Clock Switch Status) */
            __IO uint32_t HPRE : 4;                         /*!< HPRE[3:0] bits (AHB prescaler) */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t PPRE1 : 3;                        /*!< PRE1[2:0] bits (APB1 prescaler) */
            __IO uint32_t PPRE2 : 3;                        /*!< PRE2[2:0] bits (APB2 prescaler) */
            __IO uint32_t RTCPRE : 5;
            __IO uint32_t MCO1 : 2;
            __IO uint32_t I2SSRC : 1;
            __IO uint32_t MCO1PRE : 3;
            __IO uint32_t MCO2PRE : 3;
            __IO uint32_t MCO2 : 2;
        } b;
        __IO uint32_t w;
    } CFGR;                                                 /*!< RCC clock configuration register,                            Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t LSIRDYF : 1;
            __IO uint32_t LSERDYF : 1;
            __IO uint32_t HSIRDYF : 1;
            __IO uint32_t HSERDYF : 1;
            __IO uint32_t PLLRDYF : 1;
            __IO uint32_t PLLI2SRDYF : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CSSF : 1;
            __IO uint32_t LSIRDYIE : 1;
            __IO uint32_t LSERDYIE : 1;
            __IO uint32_t HSIRDYIE : 1;
            __IO uint32_t HSERDYIE : 1;
            __IO uint32_t PLLRDYIE : 1;
            __IO uint32_t PLLI2SRDYIE : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t LSIRDYC : 1;
            __IO uint32_t LSERDYC : 1;
            __IO uint32_t HSIRDYC : 1;
            __IO uint32_t HSERDYC : 1;
            __IO uint32_t PLLRDYC : 1;
            __IO uint32_t PLLI2SRDYC : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t CSSC : 1;
                 uint32_t __RESERVED3 : 8;
        } b;
        __IO uint32_t w;
    } CIR;                                                  /*!< RCC clock interrupt register,                                Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t GPIOARST : 1;
            __IO uint32_t GPIOBRST : 1;
            __IO uint32_t GPIOCRST : 1;
            __IO uint32_t GPIODRST : 1;
            __IO uint32_t GPIOERST : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t GPIOHRST : 1;
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t CRCRST : 1;
                 uint32_t __RESERVED2 : 8;
            __IO uint32_t DMA1RST : 1;
            __IO uint32_t DMA2RST : 1;
                 uint32_t __RESERVED3 : 9;
        } b;
        __IO uint32_t w;
    } AHB1RSTR;                                             /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    union {
        struct {
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t OTGFSRST : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } AHB2RSTR;                                             /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    __IO uint32_t AHB3RSTR;                                 /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t TIM2RST : 1;
            __IO uint32_t TIM3RST : 1;
            __IO uint32_t TIM4RST : 1;
            __IO uint32_t TIM5RST : 1;
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t WWDGRST : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t SPI2RST : 1;
            __IO uint32_t SPI3RST : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t USART2RST : 1;
                 uint32_t __RESERVED3 : 3;
            __IO uint32_t I2C1RST : 1;
            __IO uint32_t I2C2RST : 1;
            __IO uint32_t I2C3RST : 1;
                 uint32_t __RESERVED4 : 4;
            __IO uint32_t PWRRST : 1;
                 uint32_t __RESERVED5 : 3;
        } b;
        __IO uint32_t w;
    } APB1RSTR;                                             /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t TIM1RST : 1;
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t USART1RST : 1;
            __IO uint32_t USART6RST : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ADCRST : 1;
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SDIORST : 1;
            __IO uint32_t SPI1RST : 1;
            __IO uint32_t SPI4RST : 1;
            __IO uint32_t SYSCFGRST : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t TIM9RST : 1;
            __IO uint32_t TIM10RST : 1;
            __IO uint32_t TIM11RST : 1;
                 uint32_t __RESERVED4 : 13;
        } b;
        __IO uint32_t w;
    } APB2RSTR;                                             /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
         uint32_t __RESERVED1[2];
    union {
        struct {
            __IO uint32_t GPIOAEN : 1;
            __IO uint32_t GPIOBEN : 1;
            __IO uint32_t GPIOCEN : 1;
            __IO uint32_t GPIODEN : 1;
            __IO uint32_t GPIOEEN : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t GPIOHEN : 1;
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t CRCEN : 1;
                 uint32_t __RESERVED2 : 8;
            __IO uint32_t DMA1EN : 1;
            __IO uint32_t DMA2EN : 1;
                 uint32_t __RESERVED3 : 9;
        } b;
        __IO uint32_t w;
    } AHB1ENR;                                              /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    union {
        struct {
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t OTGFSEN : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } AHB2ENR;                                              /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    __IO uint32_t AHB3ENR;                                  /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
         uint32_t __RESERVED2;
    union {
        struct {
            __IO uint32_t TIM2EN : 1;
            __IO uint32_t TIM3EN : 1;
            __IO uint32_t TIM4EN : 1;
            __IO uint32_t TIM5EN : 1;
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t WWDGEN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t SPI2EN : 1;
            __IO uint32_t SPI3EN : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t USART2EN : 1;
                 uint32_t __RESERVED3 : 3;
            __IO uint32_t I2C1EN : 1;
            __IO uint32_t I2C2EN : 1;
            __IO uint32_t I2C3EN : 1;
                 uint32_t __RESERVED4 : 4;
            __IO uint32_t PWREN : 1;
                 uint32_t __RESERVED5 : 3;
        } b;
        __IO uint32_t w;
    } APB1ENR;                                              /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    union {
        struct {
            __IO uint32_t TIM1EN : 1;
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t USART1EN : 1;
            __IO uint32_t USART6EN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ADC1EN : 1;
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SDIOEN : 1;
            __IO uint32_t SPI1EN : 1;
            __IO uint32_t SPI4EN : 1;
            __IO uint32_t SYSCFGEN : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t TIM9EN : 1;
            __IO uint32_t TIM10EN : 1;
            __IO uint32_t TIM11EN : 1;
                 uint32_t __RESERVED4 : 13;
        } b;
        __IO uint32_t w;
    } APB2ENR;                                              /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
         uint32_t __RESERVED3[2];
    union {
        struct {
            __IO uint32_t GPIOALPEN : 1;
            __IO uint32_t GPIOBLPEN : 1;
            __IO uint32_t GPIOCLPEN : 1;
            __IO uint32_t GPIODLPEN : 1;
            __IO uint32_t GPIOELPEN : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t GPIOHLPEN : 1;
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t CRCLPEN : 1;
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t FLITFLPEN : 1;
            __IO uint32_t SRAM1LPEN : 1;
                 uint32_t __RESERVED3 : 4;
            __IO uint32_t DMA1LPEN : 1;
            __IO uint32_t DMA2LPEN : 1;
                 uint32_t __RESERVED4 : 9;
        } b;
        __IO uint32_t w;
    } AHB1LPENR;                                            /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    union {
        struct {
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t OTGFSLPEN : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } AHB2LPENR;                                            /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    __IO uint32_t AHB3LPENR;                                /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
         uint32_t __RESERVED4;
    union {
        struct {
            __IO uint32_t TIM2LPEN : 1;
            __IO uint32_t TIM3LPEN : 1;
            __IO uint32_t TIM4LPEN : 1;
            __IO uint32_t TIM5LPEN : 1;
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t WWDGLPEN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t SPI2LPEN : 1;
            __IO uint32_t SPI3LPEN : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t USART2LPEN : 1;
                 uint32_t __RESERVED3 : 3;
            __IO uint32_t I2C1LPEN : 1;
            __IO uint32_t I2C2LPEN : 1;
            __IO uint32_t I2C3LPEN : 1;
                 uint32_t __RESERVED4 : 4;
            __IO uint32_t PWRLPEN : 1;
                 uint32_t __RESERVED5 : 3;
        } b;
        __IO uint32_t w;
    } APB1LPENR;                                            /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    union {
        struct {
            __IO uint32_t TIM1LPEN : 1;
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t USART1LPEN : 1;
            __IO uint32_t USART6LPEN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ADC1LPEN : 1;
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SDIOLPEN : 1;
            __IO uint32_t SPI1LPEN : 1;
            __IO uint32_t SPI4LPEN : 1;
            __IO uint32_t SYSCFGLPEN : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t TIM9LPEN : 1;
            __IO uint32_t TIM10LPEN : 1;
            __IO uint32_t TIM11LPEN : 1;
                 uint32_t __RESERVED4 : 13;
        } b;
        __IO uint32_t w;
    } APB2LPENR;                                            /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
         uint32_t __RESERVED5[2];
    union {
        struct {
            __IO uint32_t LSEON : 1;
            __IO uint32_t LSERDY : 1;
            __IO uint32_t LSEBYP : 1;
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RTCSEL : 2;
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t RTCEN : 1;
            __IO uint32_t BDRST : 1;
                 uint32_t __RESERVED2 : 15;
        } b;
        __IO uint32_t w;
    } BDCR;                                                 /*!< RCC Backup domain control register,                          Address offset: 0x70 */
    union {
        struct {
            __IO uint32_t LSION : 1;
            __IO uint32_t LSIRDY : 1;
                 uint32_t __RESERVED0 : 22;
            __IO uint32_t RMVF : 1;
            __IO uint32_t BORRSTF : 1;
            __IO uint32_t PINRSTF : 1;
            __IO uint32_t PORRSTF : 1;
            __IO uint32_t SFTRSTF : 1;
            __IO uint32_t IWDGRSTF : 1;
            __IO uint32_t WWDGRSTF : 1;
            __IO uint32_t LPWRRSTF : 1;
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< RCC clock control & status register,                         Address offset: 0x74 */
         uint32_t __RESERVED6[2];
    union {
        struct {
            __IO uint32_t MODPER : 13;
            __IO uint32_t INCSTEP : 15;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t SPREADSEL : 1;
            __IO uint32_t SSCGEN : 1;
        } b;
        __IO uint32_t w;
    } SSCGR;                                                /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
    union {
        struct {
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t PLLI2SN : 9;
                 uint32_t __RESERVED1 : 13;
            __IO uint32_t PLLI2SR : 3;
                 uint32_t __RESERVED2 : 1;
        } b;
        __IO uint32_t w;
    } PLLI2SCFGR;                                           /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
         uint32_t __RESERVED7;
    union {
        struct {
                 uint32_t __RESERVED0 : 24;
            __IO uint32_t TIMPRE : 1;
                 uint32_t __RESERVED1 : 7;
        } b;
        __IO uint32_t w;
    } DCKCFGR;                                              /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t HSION;
        __IO uint32_t HSIRDY;
             uint32_t __RESERVED0;
        __IO uint32_t HSITRIM[5];
        __IO uint32_t HSICAL[8];
        __IO uint32_t HSEON;
        __IO uint32_t HSERDY;
        __IO uint32_t HSEBYP;
        __IO uint32_t CSSON;
             uint32_t __RESERVED1[4];
        __IO uint32_t PLLON;
        __IO uint32_t PLLRDY;
        __IO uint32_t PLLI2SON;
        __IO uint32_t PLLI2SRDY;
             uint32_t __RESERVED2[4];
    } CR;                                                   /*!< RCC clock control register,                                  Address offset: 0x00 */
    struct {
        __IO uint32_t PLLM[6];
        __IO uint32_t PLLN[9];
             uint32_t __RESERVED0;
        __IO uint32_t PLLP[2];
             uint32_t __RESERVED1[4];
        __IO uint32_t PLLSRC;
             uint32_t __RESERVED2;
        __IO uint32_t PLLQ[4];
             uint32_t __RESERVED3[4];
    } PLLCFGR;                                              /*!< RCC PLL configuration register,                              Address offset: 0x04 */
    struct {
        __IO uint32_t SW[2];                                /*!< SW[1:0] bits (System clock Switch) */
        __IO uint32_t SWS[2];                               /*!< SWS[1:0] bits (System Clock Switch Status) */
        __IO uint32_t HPRE[4];                              /*!< HPRE[3:0] bits (AHB prescaler) */
             uint32_t __RESERVED0[2];
        __IO uint32_t PPRE1[3];                             /*!< PRE1[2:0] bits (APB1 prescaler) */
        __IO uint32_t PPRE2[3];                             /*!< PRE2[2:0] bits (APB2 prescaler) */
        __IO uint32_t RTCPRE[5];
        __IO uint32_t MCO1[2];
        __IO uint32_t I2SSRC;
        __IO uint32_t MCO1PRE[3];
        __IO uint32_t MCO2PRE[3];
        __IO uint32_t MCO2[2];
    } CFGR;                                                 /*!< RCC clock configuration register,                            Address offset: 0x08 */
    struct {
        __IO uint32_t LSIRDYF;
        __IO uint32_t LSERDYF;
        __IO uint32_t HSIRDYF;
        __IO uint32_t HSERDYF;
        __IO uint32_t PLLRDYF;
        __IO uint32_t PLLI2SRDYF;
             uint32_t __RESERVED0;
        __IO uint32_t CSSF;
        __IO uint32_t LSIRDYIE;
        __IO uint32_t LSERDYIE;
        __IO uint32_t HSIRDYIE;
        __IO uint32_t HSERDYIE;
        __IO uint32_t PLLRDYIE;
        __IO uint32_t PLLI2SRDYIE;
             uint32_t __RESERVED1[2];
        __IO uint32_t LSIRDYC;
        __IO uint32_t LSERDYC;
        __IO uint32_t HSIRDYC;
        __IO uint32_t HSERDYC;
        __IO uint32_t PLLRDYC;
        __IO uint32_t PLLI2SRDYC;
             uint32_t __RESERVED2;
        __IO uint32_t CSSC;
             uint32_t __RESERVED3[8];
    } CIR;                                                  /*!< RCC clock interrupt register,                                Address offset: 0x0C */
    struct {
        __IO uint32_t GPIOARST;
        __IO uint32_t GPIOBRST;
        __IO uint32_t GPIOCRST;
        __IO uint32_t GPIODRST;
        __IO uint32_t GPIOERST;
             uint32_t __RESERVED0[2];
        __IO uint32_t GPIOHRST;
             uint32_t __RESERVED1[4];
        __IO uint32_t CRCRST;
             uint32_t __RESERVED2[8];
        __IO uint32_t DMA1RST;
        __IO uint32_t DMA2RST;
             uint32_t __RESERVED3[9];
    } AHB1RSTR;                                             /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    struct {
             uint32_t __RESERVED0[7];
        __IO uint32_t OTGFSRST;
             uint32_t __RESERVED1[24];
    } AHB2RSTR;                                             /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    __IO uint32_t AHB3RSTR[32];                             /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
         uint32_t __RESERVED0[32];
    struct {
        __IO uint32_t TIM2RST;
        __IO uint32_t TIM3RST;
        __IO uint32_t TIM4RST;
        __IO uint32_t TIM5RST;
             uint32_t __RESERVED0[7];
        __IO uint32_t WWDGRST;
             uint32_t __RESERVED1[2];
        __IO uint32_t SPI2RST;
        __IO uint32_t SPI3RST;
             uint32_t __RESERVED2;
        __IO uint32_t USART2RST;
             uint32_t __RESERVED3[3];
        __IO uint32_t I2C1RST;
        __IO uint32_t I2C2RST;
        __IO uint32_t I2C3RST;
             uint32_t __RESERVED4[4];
        __IO uint32_t PWRRST;
             uint32_t __RESERVED5[3];
    } APB1RSTR;                                             /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    struct {
        __IO uint32_t TIM1RST;
             uint32_t __RESERVED0[3];
        __IO uint32_t USART1RST;
        __IO uint32_t USART6RST;
             uint32_t __RESERVED1[2];
        __IO uint32_t ADCRST;
             uint32_t __RESERVED2[2];
        __IO uint32_t SDIORST;
        __IO uint32_t SPI1RST;
        __IO uint32_t SPI4RST;
        __IO uint32_t SYSCFGRST;
             uint32_t __RESERVED3;
        __IO uint32_t TIM9RST;
        __IO uint32_t TIM10RST;
        __IO uint32_t TIM11RST;
             uint32_t __RESERVED4[13];
    } APB2RSTR;                                             /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
         uint32_t __RESERVED1[2][32];
    struct {
        __IO uint32_t GPIOAEN;
        __IO uint32_t GPIOBEN;
        __IO uint32_t GPIOCEN;
        __IO uint32_t GPIODEN;
        __IO uint32_t GPIOEEN;
             uint32_t __RESERVED0[2];
        __IO uint32_t GPIOHEN;
             uint32_t __RESERVED1[4];
        __IO uint32_t CRCEN;
             uint32_t __RESERVED2[8];
        __IO uint32_t DMA1EN;
        __IO uint32_t DMA2EN;
             uint32_t __RESERVED3[9];
    } AHB1ENR;                                              /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    struct {
             uint32_t __RESERVED0[7];
        __IO uint32_t OTGFSEN;
             uint32_t __RESERVED1[24];
    } AHB2ENR;                                              /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    __IO uint32_t AHB3ENR[32];                              /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
         uint32_t __RESERVED2[32];
    struct {
        __IO uint32_t TIM2EN;
        __IO uint32_t TIM3EN;
        __IO uint32_t TIM4EN;
        __IO uint32_t TIM5EN;
             uint32_t __RESERVED0[7];
        __IO uint32_t WWDGEN;
             uint32_t __RESERVED1[2];
        __IO uint32_t SPI2EN;
        __IO uint32_t SPI3EN;
             uint32_t __RESERVED2;
        __IO uint32_t USART2EN;
             uint32_t __RESERVED3[3];
        __IO uint32_t I2C1EN;
        __IO uint32_t I2C2EN;
        __IO uint32_t I2C3EN;
             uint32_t __RESERVED4[4];
        __IO uint32_t PWREN;
             uint32_t __RESERVED5[3];
    } APB1ENR;                                              /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    struct {
        __IO uint32_t TIM1EN;
             uint32_t __RESERVED0[3];
        __IO uint32_t USART1EN;
        __IO uint32_t USART6EN;
             uint32_t __RESERVED1[2];
        __IO uint32_t ADC1EN;
             uint32_t __RESERVED2[2];
        __IO uint32_t SDIOEN;
        __IO uint32_t SPI1EN;
        __IO uint32_t SPI4EN;
        __IO uint32_t SYSCFGEN;
             uint32_t __RESERVED3;
        __IO uint32_t TIM9EN;
        __IO uint32_t TIM10EN;
        __IO uint32_t TIM11EN;
             uint32_t __RESERVED4[13];
    } APB2ENR;                                              /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
         uint32_t __RESERVED3[2][32];
    struct {
        __IO uint32_t GPIOALPEN;
        __IO uint32_t GPIOBLPEN;
        __IO uint32_t GPIOCLPEN;
        __IO uint32_t GPIODLPEN;
        __IO uint32_t GPIOELPEN;
             uint32_t __RESERVED0[2];
        __IO uint32_t GPIOHLPEN;
             uint32_t __RESERVED1[4];
        __IO uint32_t CRCLPEN;
             uint32_t __RESERVED2[2];
        __IO uint32_t FLITFLPEN;
        __IO uint32_t SRAM1LPEN;
             uint32_t __RESERVED3[4];
        __IO uint32_t DMA1LPEN;
        __IO uint32_t DMA2LPEN;
             uint32_t __RESERVED4[9];
    } AHB1LPENR;                                            /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    struct {
             uint32_t __RESERVED0[7];
        __IO uint32_t OTGFSLPEN;
             uint32_t __RESERVED1[24];
    } AHB2LPENR;                                            /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    __IO uint32_t AHB3LPENR[32];                            /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
         uint32_t __RESERVED4[32];
    struct {
        __IO uint32_t TIM2LPEN;
        __IO uint32_t TIM3LPEN;
        __IO uint32_t TIM4LPEN;
        __IO uint32_t TIM5LPEN;
             uint32_t __RESERVED0[7];
        __IO uint32_t WWDGLPEN;
             uint32_t __RESERVED1[2];
        __IO uint32_t SPI2LPEN;
        __IO uint32_t SPI3LPEN;
             uint32_t __RESERVED2;
        __IO uint32_t USART2LPEN;
             uint32_t __RESERVED3[3];
        __IO uint32_t I2C1LPEN;
        __IO uint32_t I2C2LPEN;
        __IO uint32_t I2C3LPEN;
             uint32_t __RESERVED4[4];
        __IO uint32_t PWRLPEN;
             uint32_t __RESERVED5[3];
    } APB1LPENR;                                            /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    struct {
        __IO uint32_t TIM1LPEN;
             uint32_t __RESERVED0[3];
        __IO uint32_t USART1LPEN;
        __IO uint32_t USART6LPEN;
             uint32_t __RESERVED1[2];
        __IO uint32_t ADC1LPEN;
             uint32_t __RESERVED2[2];
        __IO uint32_t SDIOLPEN;
        __IO uint32_t SPI1LPEN;
        __IO uint32_t SPI4LPEN;
        __IO uint32_t SYSCFGLPEN;
             uint32_t __RESERVED3;
        __IO uint32_t TIM9LPEN;
        __IO uint32_t TIM10LPEN;
        __IO uint32_t TIM11LPEN;
             uint32_t __RESERVED4[13];
    } APB2LPENR;                                            /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
         uint32_t __RESERVED5[2][32];
    struct {
        __IO uint32_t LSEON;
        __IO uint32_t LSERDY;
        __IO uint32_t LSEBYP;
             uint32_t __RESERVED0[5];
        __IO uint32_t RTCSEL[2];
             uint32_t __RESERVED1[5];
        __IO uint32_t RTCEN;
        __IO uint32_t BDRST;
             uint32_t __RESERVED2[15];
    } BDCR;                                                 /*!< RCC Backup domain control register,                          Address offset: 0x70 */
    struct {
        __IO uint32_t LSION;
        __IO uint32_t LSIRDY;
             uint32_t __RESERVED0[22];
        __IO uint32_t RMVF;
        __IO uint32_t BORRSTF;
        __IO uint32_t PINRSTF;
        __IO uint32_t PORRSTF;
        __IO uint32_t SFTRSTF;
        __IO uint32_t IWDGRSTF;
        __IO uint32_t WWDGRSTF;
        __IO uint32_t LPWRRSTF;
    } CSR;                                                  /*!< RCC clock control & status register,                         Address offset: 0x74 */
         uint32_t __RESERVED6[2][32];
    struct {
        __IO uint32_t MODPER[13];
        __IO uint32_t INCSTEP[15];
             uint32_t __RESERVED0[2];
        __IO uint32_t SPREADSEL;
        __IO uint32_t SSCGEN;
    } SSCGR;                                                /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
    struct {
             uint32_t __RESERVED0[6];
        __IO uint32_t PLLI2SN[9];
             uint32_t __RESERVED1[13];
        __IO uint32_t PLLI2SR[3];
             uint32_t __RESERVED2;
    } PLLI2SCFGR;                                           /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
         uint32_t __RESERVED7[32];
    struct {
             uint32_t __RESERVED0[24];
        __IO uint32_t TIMPRE;
             uint32_t __RESERVED1[7];
    } DCKCFGR;                                              /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_BitBand_TypeDef;



/**
  * @brief Real-Time Clock
  */
typedef struct {
    union {
        struct {
            __IO uint32_t SU : 4;
            __IO uint32_t ST : 3;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t MNU : 4;
            __IO uint32_t MNT : 3;
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t HU : 4;
            __IO uint32_t HT : 2;
            __IO uint32_t PM : 1;
                 uint32_t __RESERVED2 : 9;
        } b;
        __IO uint32_t w;
    } TR;                                                   /*!< RTC time register,                                        Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t DU : 4;
            __IO uint32_t DT : 2;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t MU : 4;
            __IO uint32_t MT : 1;
            __IO uint32_t WDU : 3;
            __IO uint32_t YU : 4;
            __IO uint32_t YT : 4;
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } DR;                                                   /*!< RTC date register,                                        Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t WUCKSEL : 3;
            __IO uint32_t TSEDGE : 1;
            __IO uint32_t REFCKON : 1;
            __IO uint32_t BYPSHAD : 1;
            __IO uint32_t FMT : 1;
            __IO uint32_t DCE : 1;
            __IO uint32_t ALRAE : 1;
            __IO uint32_t ALRBE : 1;
            __IO uint32_t WUTE : 1;
            __IO uint32_t TSE : 1;
            __IO uint32_t ALRAIE : 1;
            __IO uint32_t ALRBIE : 1;
            __IO uint32_t WUTIE : 1;
            __IO uint32_t TSIE : 1;
            __IO uint32_t ADD1H : 1;
            __IO uint32_t SUB1H : 1;
            __IO uint32_t BKP : 1;
            __IO uint32_t COSEL : 1;
            __IO uint32_t POL : 1;
            __IO uint32_t OSEL : 2;
            __IO uint32_t COE : 1;
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< RTC control register,                                     Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t ALRAWF : 1;
            __IO uint32_t ALRBWF : 1;
            __IO uint32_t WUTWF : 1;
            __IO uint32_t SHPF : 1;
            __IO uint32_t INITS : 1;
            __IO uint32_t RSF : 1;
            __IO uint32_t INITF : 1;
            __IO uint32_t INIT : 1;
            __IO uint32_t ALRAF : 1;
            __IO uint32_t ALRBF : 1;
            __IO uint32_t WUTF : 1;
            __IO uint32_t TSF : 1;
            __IO uint32_t TSOVF : 1;
            __IO uint32_t TAMP1F : 1;
            __IO uint32_t TAMP2F : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RECALPF : 1;
                 uint32_t __RESERVED1 : 15;
        } b;
        __IO uint32_t w;
    } ISR;                                                  /*!< RTC initialization and status register,                   Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t PREDIV_S : 15;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PREDIV_A : 7;
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } PRER;                                                 /*!< RTC prescaler register,                                   Address offset: 0x10 */
    __IO uint32_t WUTR;                                     /*!< RTC wakeup timer register,                                Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t DC : 5;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t DCS : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } CALIBR;                                               /*!< RTC calibration register,                                 Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t SU : 4;
            __IO uint32_t ST : 3;
            __IO uint32_t MSK1 : 1;
            __IO uint32_t MNU : 4;
            __IO uint32_t MNT : 3;
            __IO uint32_t MSK2 : 1;
            __IO uint32_t HU : 4;
            __IO uint32_t HT : 2;
            __IO uint32_t PM : 1;
            __IO uint32_t MSK3 : 1;
            __IO uint32_t DU : 4;
            __IO uint32_t DT : 2;
            __IO uint32_t WDSEL : 1;
            __IO uint32_t MSK4 : 1;
        } b;
        __IO uint32_t w;
    } ALRMAR;                                               /*!< RTC alarm A register,                                     Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t SU : 4;
            __IO uint32_t ST : 3;
            __IO uint32_t MSK1 : 1;
            __IO uint32_t MNU : 4;
            __IO uint32_t MNT : 3;
            __IO uint32_t MSK2 : 1;
            __IO uint32_t HU : 4;
            __IO uint32_t HT : 2;
            __IO uint32_t PM : 1;
            __IO uint32_t MSK3 : 1;
            __IO uint32_t DU : 4;
            __IO uint32_t DT : 2;
            __IO uint32_t WDSEL : 1;
            __IO uint32_t MSK4 : 1;
        } b;
        __IO uint32_t w;
    } ALRMBR;                                               /*!< RTC alarm B register,                                     Address offset: 0x20 */
    __IO uint32_t WPR;                                      /*!< RTC write protection register,                            Address offset: 0x24 */
    __IO uint32_t SSR;                                      /*!< RTC sub second register,                                  Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t SUBFS : 15;
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t ADD1S : 1;
        } b;
        __IO uint32_t w;
    } SHIFTR;                                               /*!< RTC shift control register,                               Address offset: 0x2C */
    union {
        struct {
            __IO uint32_t SU : 4;
            __IO uint32_t ST : 3;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t MNU : 4;
            __IO uint32_t MNT : 3;
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t HU : 4;
            __IO uint32_t HT : 2;
            __IO uint32_t PM : 1;
                 uint32_t __RESERVED2 : 9;
        } b;
        __IO uint32_t w;
    } TSTR;                                                 /*!< RTC time stamp time register,                             Address offset: 0x30 */
    union {
        struct {
            __IO uint32_t DU : 4;
            __IO uint32_t DT : 2;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t MU : 4;
            __IO uint32_t MT : 1;
            __IO uint32_t WDU : 3;
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } TSDR;                                                 /*!< RTC time stamp date register,                             Address offset: 0x34 */
    __IO uint32_t TSSSR;                                    /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
    union {
        struct {
            __IO uint32_t CALM : 9;
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t CALW16 : 1;
            __IO uint32_t CALW8 : 1;
            __IO uint32_t CALP : 1;
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CALR;                                                 /*!< RTC calibration register,                                 Address offset: 0x3C */
    union {
        struct {
            __IO uint32_t TAMP1E : 1;
            __IO uint32_t TAMP1TRG : 1;
            __IO uint32_t TAMPIE : 1;
            __IO uint32_t TAMP2E : 1;
            __IO uint32_t TAMP2TRG : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t TAMPTS : 1;
            __IO uint32_t TAMPFREQ : 3;
            __IO uint32_t TAMPFLT : 2;
            __IO uint32_t TAMPPRCH : 2;
            __IO uint32_t TAMPPUDIS : 1;
            __IO uint32_t TAMP1INSEL : 1;
            __IO uint32_t TSINSEL : 1;
            __IO uint32_t ALARMOUTTYPE : 1;
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } TAFCR;                                                /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
    union {
        struct {
            __IO uint32_t SS : 15;
                 uint32_t __RESERVED0 : 9;
            __IO uint32_t MASKSS : 4;
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } ALRMASSR;                                             /*!< RTC alarm A sub second register,                          Address offset: 0x44 */
    union {
        struct {
            __IO uint32_t SS : 15;
                 uint32_t __RESERVED0 : 9;
            __IO uint32_t MASKSS : 4;
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } ALRMBSSR;                                             /*!< RTC alarm B sub second register,                          Address offset: 0x48 */
         uint32_t __RESERVED0;
    __IO uint32_t BKPR[20];                                 /*!< RTC backup registers,                                     Address offset: 0x50-0x8C */
} RTC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t SU[4];
        __IO uint32_t ST[3];
             uint32_t __RESERVED0;
        __IO uint32_t MNU[4];
        __IO uint32_t MNT[3];
             uint32_t __RESERVED1;
        __IO uint32_t HU[4];
        __IO uint32_t HT[2];
        __IO uint32_t PM;
             uint32_t __RESERVED2[9];
    } TR;                                                   /*!< RTC time register,                                        Address offset: 0x00 */
    struct {
        __IO uint32_t DU[4];
        __IO uint32_t DT[2];
             uint32_t __RESERVED0[2];
        __IO uint32_t MU[4];
        __IO uint32_t MT;
        __IO uint32_t WDU[3];
        __IO uint32_t YU[4];
        __IO uint32_t YT[4];
             uint32_t __RESERVED1[8];
    } DR;                                                   /*!< RTC date register,                                        Address offset: 0x04 */
    struct {
        __IO uint32_t WUCKSEL[3];
        __IO uint32_t TSEDGE;
        __IO uint32_t REFCKON;
        __IO uint32_t BYPSHAD;
        __IO uint32_t FMT;
        __IO uint32_t DCE;
        __IO uint32_t ALRAE;
        __IO uint32_t ALRBE;
        __IO uint32_t WUTE;
        __IO uint32_t TSE;
        __IO uint32_t ALRAIE;
        __IO uint32_t ALRBIE;
        __IO uint32_t WUTIE;
        __IO uint32_t TSIE;
        __IO uint32_t ADD1H;
        __IO uint32_t SUB1H;
        __IO uint32_t BKP;
        __IO uint32_t COSEL;
        __IO uint32_t POL;
        __IO uint32_t OSEL[2];
        __IO uint32_t COE;
             uint32_t __RESERVED0[8];
    } CR;                                                   /*!< RTC control register,                                     Address offset: 0x08 */
    struct {
        __IO uint32_t ALRAWF;
        __IO uint32_t ALRBWF;
        __IO uint32_t WUTWF;
        __IO uint32_t SHPF;
        __IO uint32_t INITS;
        __IO uint32_t RSF;
        __IO uint32_t INITF;
        __IO uint32_t INIT;
        __IO uint32_t ALRAF;
        __IO uint32_t ALRBF;
        __IO uint32_t WUTF;
        __IO uint32_t TSF;
        __IO uint32_t TSOVF;
        __IO uint32_t TAMP1F;
        __IO uint32_t TAMP2F;
             uint32_t __RESERVED0;
        __IO uint32_t RECALPF;
             uint32_t __RESERVED1[15];
    } ISR;                                                  /*!< RTC initialization and status register,                   Address offset: 0x0C */
    struct {
        __IO uint32_t PREDIV_S[15];
             uint32_t __RESERVED0;
        __IO uint32_t PREDIV_A[7];
             uint32_t __RESERVED1[9];
    } PRER;                                                 /*!< RTC prescaler register,                                   Address offset: 0x10 */
    __IO uint32_t WUTR[32];                                 /*!< RTC wakeup timer register,                                Address offset: 0x14 */
    struct {
        __IO uint32_t DC[5];
             uint32_t __RESERVED0[2];
        __IO uint32_t DCS;
             uint32_t __RESERVED1[24];
    } CALIBR;                                               /*!< RTC calibration register,                                 Address offset: 0x18 */
    struct {
        __IO uint32_t SU[4];
        __IO uint32_t ST[3];
        __IO uint32_t MSK1;
        __IO uint32_t MNU[4];
        __IO uint32_t MNT[3];
        __IO uint32_t MSK2;
        __IO uint32_t HU[4];
        __IO uint32_t HT[2];
        __IO uint32_t PM;
        __IO uint32_t MSK3;
        __IO uint32_t DU[4];
        __IO uint32_t DT[2];
        __IO uint32_t WDSEL;
        __IO uint32_t MSK4;
    } ALRMAR;                                               /*!< RTC alarm A register,                                     Address offset: 0x1C */
    struct {
        __IO uint32_t SU[4];
        __IO uint32_t ST[3];
        __IO uint32_t MSK1;
        __IO uint32_t MNU[4];
        __IO uint32_t MNT[3];
        __IO uint32_t MSK2;
        __IO uint32_t HU[4];
        __IO uint32_t HT[2];
        __IO uint32_t PM;
        __IO uint32_t MSK3;
        __IO uint32_t DU[4];
        __IO uint32_t DT[2];
        __IO uint32_t WDSEL;
        __IO uint32_t MSK4;
    } ALRMBR;                                               /*!< RTC alarm B register,                                     Address offset: 0x20 */
    __IO uint32_t WPR[32];                                  /*!< RTC write protection register,                            Address offset: 0x24 */
    __IO uint32_t SSR[32];                                  /*!< RTC sub second register,                                  Address offset: 0x28 */
    struct {
        __IO uint32_t SUBFS[15];
             uint32_t __RESERVED0[16];
        __IO uint32_t ADD1S;
    } SHIFTR;                                               /*!< RTC shift control register,                               Address offset: 0x2C */
    struct {
        __IO uint32_t SU[4];
        __IO uint32_t ST[3];
             uint32_t __RESERVED0;
        __IO uint32_t MNU[4];
        __IO uint32_t MNT[3];
             uint32_t __RESERVED1;
        __IO uint32_t HU[4];
        __IO uint32_t HT[2];
        __IO uint32_t PM;
             uint32_t __RESERVED2[9];
    } TSTR;                                                 /*!< RTC time stamp time register,                             Address offset: 0x30 */
    struct {
        __IO uint32_t DU[4];
        __IO uint32_t DT[2];
             uint32_t __RESERVED0[2];
        __IO uint32_t MU[4];
        __IO uint32_t MT;
        __IO uint32_t WDU[3];
             uint32_t __RESERVED1[16];
    } TSDR;                                                 /*!< RTC time stamp date register,                             Address offset: 0x34 */
    __IO uint32_t TSSSR[32];                                /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
    struct {
        __IO uint32_t CALM[9];
             uint32_t __RESERVED0[4];
        __IO uint32_t CALW16;
        __IO uint32_t CALW8;
        __IO uint32_t CALP;
             uint32_t __RESERVED1[16];
    } CALR;                                                 /*!< RTC calibration register,                                 Address offset: 0x3C */
    struct {
        __IO uint32_t TAMP1E;
        __IO uint32_t TAMP1TRG;
        __IO uint32_t TAMPIE;
        __IO uint32_t TAMP2E;
        __IO uint32_t TAMP2TRG;
             uint32_t __RESERVED0[2];
        __IO uint32_t TAMPTS;
        __IO uint32_t TAMPFREQ[3];
        __IO uint32_t TAMPFLT[2];
        __IO uint32_t TAMPPRCH[2];
        __IO uint32_t TAMPPUDIS;
        __IO uint32_t TAMP1INSEL;
        __IO uint32_t TSINSEL;
        __IO uint32_t ALARMOUTTYPE;
             uint32_t __RESERVED1[13];
    } TAFCR;                                                /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
    struct {
        __IO uint32_t SS[15];
             uint32_t __RESERVED0[9];
        __IO uint32_t MASKSS[4];
             uint32_t __RESERVED1[4];
    } ALRMASSR;                                             /*!< RTC alarm A sub second register,                          Address offset: 0x44 */
    struct {
        __IO uint32_t SS[15];
             uint32_t __RESERVED0[9];
        __IO uint32_t MASKSS[4];
             uint32_t __RESERVED1[4];
    } ALRMBSSR;                                             /*!< RTC alarm B sub second register,                          Address offset: 0x48 */
         uint32_t __RESERVED0[32];
    __IO uint32_t BKPR[20][32];                             /*!< RTC backup registers,                                     Address offset: 0x50-0x8C */
} RTC_BitBand_TypeDef;



/**
  * @brief SD host Interface
  */
typedef struct {
    union {
        struct {
            __IO uint32_t PWRCTRL : 2;                      /*!<PWRCTRL[1:0] bits (Power supply control bits) */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } POWER;                                                /*!< SDIO power control register,    Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CLKDIV : 8;                       /*!<Clock divide factor             */
            __IO uint32_t CLKEN : 1;                        /*!<Clock enable bit                */
            __IO uint32_t PWRSAV : 1;                       /*!<Power saving configuration bit  */
            __IO uint32_t BYPASS : 1;                       /*!<Clock divider bypass enable bit */
            __IO uint32_t WIDBUS : 2;                       /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
            __IO uint32_t NEGEDGE : 1;                      /*!<SDIO_CK dephasing selection bit */
            __IO uint32_t HWFC_EN : 1;                      /*!<HW Flow Control enable          */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CLKCR;                                                /*!< SDI clock control register,     Address offset: 0x04 */
    __IO uint32_t ARG;                                      /*!< SDIO argument register,         Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t CMDINDEX : 6;                     /*!<Command Index                               */
            __IO uint32_t WAITRESP : 2;                     /*!<WAITRESP[1:0] bits (Wait for response bits) */
            __IO uint32_t WAITINT : 1;                      /*!<CPSM Waits for Interrupt Request                               */
            __IO uint32_t WAITPEND : 1;                     /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
            __IO uint32_t CPSMEN : 1;                       /*!<Command path state machine (CPSM) Enable bit                   */
            __IO uint32_t SDIOSUSPEND : 1;                  /*!<SD I/O suspend command                                         */
            __IO uint32_t ENCMDCOMPL : 1;                   /*!<Enable CMD completion                                          */
            __IO uint32_t NIEN : 1;                         /*!<Not Interrupt Enable                                           */
            __IO uint32_t CEATACMD : 1;                     /*!<CE-ATA command                                                 */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CMD;                                                  /*!< SDIO command register,          Address offset: 0x0C */
    union {
        struct {
            __I  uint32_t RESPCMD : 6;                      /*!<Response command index */
                 uint32_t __RESERVED0 : 26;
        } b;
        __I  uint32_t w;
    } RESPCMD;                                              /*!< SDIO command response register, Address offset: 0x10 */
    __I  uint32_t RESP1;                                    /*!< SDIO response 1 register,       Address offset: 0x14 */
    __I  uint32_t RESP2;                                    /*!< SDIO response 2 register,       Address offset: 0x18 */
    __I  uint32_t RESP3;                                    /*!< SDIO response 3 register,       Address offset: 0x1C */
    __I  uint32_t RESP4;                                    /*!< SDIO response 4 register,       Address offset: 0x20 */
    __IO uint32_t DTIMER;                                   /*!< SDIO data timer register,       Address offset: 0x24 */
    __IO uint32_t DLEN;                                     /*!< SDIO data length register,      Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t DTEN : 1;                         /*!<Data transfer enabled bit         */
            __IO uint32_t DTDIR : 1;                        /*!<Data transfer direction selection */
            __IO uint32_t DTMODE : 1;                       /*!<Data transfer mode selection      */
            __IO uint32_t DMAEN : 1;                        /*!<DMA enabled bit                   */
            __IO uint32_t DBLOCKSIZE : 4;                   /*!<DBLOCKSIZE[3:0] bits (Data block size) */
            __IO uint32_t RWSTART : 1;                      /*!<Read wait start         */
            __IO uint32_t RWSTOP : 1;                       /*!<Read wait stop          */
            __IO uint32_t RWMOD : 1;                        /*!<Read wait mode          */
            __IO uint32_t SDIOEN : 1;                       /*!<SD I/O enable functions */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DCTRL;                                                /*!< SDIO data control register,     Address offset: 0x2C */
    __I  uint32_t DCOUNT;                                   /*!< SDIO data counter register,     Address offset: 0x30 */
    union {
        struct {
            __I  uint32_t CCRCFAIL : 1;                     /*!<Command response received (CRC check failed)  */
            __I  uint32_t DCRCFAIL : 1;                     /*!<Data block sent/received (CRC check failed)   */
            __I  uint32_t CTIMEOUT : 1;                     /*!<Command response timeout                      */
            __I  uint32_t DTIMEOUT : 1;                     /*!<Data timeout                                  */
            __I  uint32_t TXUNDERR : 1;                     /*!<Transmit FIFO underrun error                  */
            __I  uint32_t RXOVERR : 1;                      /*!<Received FIFO overrun error                   */
            __I  uint32_t CMDREND : 1;                      /*!<Command response received (CRC check passed)  */
            __I  uint32_t CMDSENT : 1;                      /*!<Command sent (no response required)           */
            __I  uint32_t DATAEND : 1;                      /*!<Data end (data counter, SDIDCOUNT, is zero)   */
            __I  uint32_t STBITERR : 1;                     /*!<Start bit not detected on all data signals in wide bus mode */
            __I  uint32_t DBCKEND : 1;                      /*!<Data block sent/received (CRC check passed)   */
            __I  uint32_t CMDACT : 1;                       /*!<Command transfer in progress                  */
            __I  uint32_t TXACT : 1;                        /*!<Data transmit in progress                     */
            __I  uint32_t RXACT : 1;                        /*!<Data receive in progress                      */
            __I  uint32_t TXFIFOHE : 1;                     /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
            __I  uint32_t RXFIFOHF : 1;                     /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
            __I  uint32_t TXFIFOF : 1;                      /*!<Transmit FIFO full                            */
            __I  uint32_t RXFIFOF : 1;                      /*!<Receive FIFO full                             */
            __I  uint32_t TXFIFOE : 1;                      /*!<Transmit FIFO empty                           */
            __I  uint32_t RXFIFOE : 1;                      /*!<Receive FIFO empty                            */
            __I  uint32_t TXDAVL : 1;                       /*!<Data available in transmit FIFO               */
            __I  uint32_t RXDAVL : 1;                       /*!<Data available in receive FIFO                */
            __I  uint32_t SDIOIT : 1;                       /*!<SDIO interrupt received                       */
            __I  uint32_t CEATAEND : 1;                     /*!<CE-ATA command completion signal received for CMD61 */
                 uint32_t __RESERVED0 : 8;
        } b;
        __I  uint32_t w;
    } STA;                                                  /*!< SDIO status register,           Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t CCRCFAILC : 1;                    /*!<CCRCFAIL flag clear bit */
            __IO uint32_t DCRCFAILC : 1;                    /*!<DCRCFAIL flag clear bit */
            __IO uint32_t CTIMEOUTC : 1;                    /*!<CTIMEOUT flag clear bit */
            __IO uint32_t DTIMEOUTC : 1;                    /*!<DTIMEOUT flag clear bit */
            __IO uint32_t TXUNDERRC : 1;                    /*!<TXUNDERR flag clear bit */
            __IO uint32_t RXOVERRC : 1;                     /*!<RXOVERR flag clear bit  */
            __IO uint32_t CMDRENDC : 1;                     /*!<CMDREND flag clear bit  */
            __IO uint32_t CMDSENTC : 1;                     /*!<CMDSENT flag clear bit  */
            __IO uint32_t DATAENDC : 1;                     /*!<DATAEND flag clear bit  */
            __IO uint32_t STBITERRC : 1;                    /*!<STBITERR flag clear bit */
            __IO uint32_t DBCKENDC : 1;                     /*!<DBCKEND flag clear bit  */
                 uint32_t __RESERVED0 : 11;
            __IO uint32_t SDIOITC : 1;                      /*!<SDIOIT flag clear bit   */
            __IO uint32_t CEATAENDC : 1;                    /*!<CEATAEND flag clear bit */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } ICR;                                                  /*!< SDIO interrupt clear register,  Address offset: 0x38 */
    union {
        struct {
            __IO uint32_t CCRCFAILIE : 1;                   /*!<Command CRC Fail Interrupt Enable          */
            __IO uint32_t DCRCFAILIE : 1;                   /*!<Data CRC Fail Interrupt Enable             */
            __IO uint32_t CTIMEOUTIE : 1;                   /*!<Command TimeOut Interrupt Enable           */
            __IO uint32_t DTIMEOUTIE : 1;                   /*!<Data TimeOut Interrupt Enable              */
            __IO uint32_t TXUNDERRIE : 1;                   /*!<Tx FIFO UnderRun Error Interrupt Enable    */
            __IO uint32_t RXOVERRIE : 1;                    /*!<Rx FIFO OverRun Error Interrupt Enable     */
            __IO uint32_t CMDRENDIE : 1;                    /*!<Command Response Received Interrupt Enable */
            __IO uint32_t CMDSENTIE : 1;                    /*!<Command Sent Interrupt Enable              */
            __IO uint32_t DATAENDIE : 1;                    /*!<Data End Interrupt Enable                  */
            __IO uint32_t STBITERRIE : 1;                   /*!<Start Bit Error Interrupt Enable           */
            __IO uint32_t DBCKENDIE : 1;                    /*!<Data Block End Interrupt Enable            */
            __IO uint32_t CMDACTIE : 1;                     /*!<CCommand Acting Interrupt Enable           */
            __IO uint32_t TXACTIE : 1;                      /*!<Data Transmit Acting Interrupt Enable      */
            __IO uint32_t RXACTIE : 1;                      /*!<Data receive acting interrupt enabled      */
            __IO uint32_t TXFIFOHEIE : 1;                   /*!<Tx FIFO Half Empty interrupt Enable        */
            __IO uint32_t RXFIFOHFIE : 1;                   /*!<Rx FIFO Half Full interrupt Enable         */
            __IO uint32_t TXFIFOFIE : 1;                    /*!<Tx FIFO Full interrupt Enable              */
            __IO uint32_t RXFIFOFIE : 1;                    /*!<Rx FIFO Full interrupt Enable              */
            __IO uint32_t TXFIFOEIE : 1;                    /*!<Tx FIFO Empty interrupt Enable             */
            __IO uint32_t RXFIFOEIE : 1;                    /*!<Rx FIFO Empty interrupt Enable             */
            __IO uint32_t TXDAVLIE : 1;                     /*!<Data available in Tx FIFO interrupt Enable */
            __IO uint32_t RXDAVLIE : 1;                     /*!<Data available in Rx FIFO interrupt Enable */
            __IO uint32_t SDIOITIE : 1;                     /*!<SDIO Mode Interrupt Received interrupt Enable */
            __IO uint32_t CEATAENDIE : 1;                   /*!<CE-ATA command completion signal received Interrupt Enable */
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } MASK;                                                 /*!< SDIO mask register,             Address offset: 0x3C */
         uint32_t __RESERVED0[2];
    __I  uint32_t FIFOCNT;                                  /*!< SDIO FIFO counter register,     Address offset: 0x48 */
         uint32_t __RESERVED1[13];
    __IO uint32_t FIFO;                                     /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PWRCTRL[2];                           /*!<PWRCTRL[1:0] bits (Power supply control bits) */
             uint32_t __RESERVED0[30];
    } POWER;                                                /*!< SDIO power control register,    Address offset: 0x00 */
    struct {
        __IO uint32_t CLKDIV[8];                            /*!<Clock divide factor             */
        __IO uint32_t CLKEN;                                /*!<Clock enable bit                */
        __IO uint32_t PWRSAV;                               /*!<Power saving configuration bit  */
        __IO uint32_t BYPASS;                               /*!<Clock divider bypass enable bit */
        __IO uint32_t WIDBUS[2];                            /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
        __IO uint32_t NEGEDGE;                              /*!<SDIO_CK dephasing selection bit */
        __IO uint32_t HWFC_EN;                              /*!<HW Flow Control enable          */
             uint32_t __RESERVED0[17];
    } CLKCR;                                                /*!< SDI clock control register,     Address offset: 0x04 */
    __IO uint32_t ARG[32];                                  /*!< SDIO argument register,         Address offset: 0x08 */
    struct {
        __IO uint32_t CMDINDEX[6];                          /*!<Command Index                               */
        __IO uint32_t WAITRESP[2];                          /*!<WAITRESP[1:0] bits (Wait for response bits) */
        __IO uint32_t WAITINT;                              /*!<CPSM Waits for Interrupt Request                               */
        __IO uint32_t WAITPEND;                             /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
        __IO uint32_t CPSMEN;                               /*!<Command path state machine (CPSM) Enable bit                   */
        __IO uint32_t SDIOSUSPEND;                          /*!<SD I/O suspend command                                         */
        __IO uint32_t ENCMDCOMPL;                           /*!<Enable CMD completion                                          */
        __IO uint32_t NIEN;                                 /*!<Not Interrupt Enable                                           */
        __IO uint32_t CEATACMD;                             /*!<CE-ATA command                                                 */
             uint32_t __RESERVED0[17];
    } CMD;                                                  /*!< SDIO command register,          Address offset: 0x0C */
    struct {
        __I  uint32_t RESPCMD[6];                           /*!<Response command index */
             uint32_t __RESERVED0[26];
    } RESPCMD;                                              /*!< SDIO command response register, Address offset: 0x10 */
    __I  uint32_t RESP1[32];                                /*!< SDIO response 1 register,       Address offset: 0x14 */
    __I  uint32_t RESP2[32];                                /*!< SDIO response 2 register,       Address offset: 0x18 */
    __I  uint32_t RESP3[32];                                /*!< SDIO response 3 register,       Address offset: 0x1C */
    __I  uint32_t RESP4[32];                                /*!< SDIO response 4 register,       Address offset: 0x20 */
    __IO uint32_t DTIMER[32];                               /*!< SDIO data timer register,       Address offset: 0x24 */
    __IO uint32_t DLEN[32];                                 /*!< SDIO data length register,      Address offset: 0x28 */
    struct {
        __IO uint32_t DTEN;                                 /*!<Data transfer enabled bit         */
        __IO uint32_t DTDIR;                                /*!<Data transfer direction selection */
        __IO uint32_t DTMODE;                               /*!<Data transfer mode selection      */
        __IO uint32_t DMAEN;                                /*!<DMA enabled bit                   */
        __IO uint32_t DBLOCKSIZE[4];                        /*!<DBLOCKSIZE[3:0] bits (Data block size) */
        __IO uint32_t RWSTART;                              /*!<Read wait start         */
        __IO uint32_t RWSTOP;                               /*!<Read wait stop          */
        __IO uint32_t RWMOD;                                /*!<Read wait mode          */
        __IO uint32_t SDIOEN;                               /*!<SD I/O enable functions */
             uint32_t __RESERVED0[20];
    } DCTRL;                                                /*!< SDIO data control register,     Address offset: 0x2C */
    __I  uint32_t DCOUNT[32];                               /*!< SDIO data counter register,     Address offset: 0x30 */
    struct {
        __I  uint32_t CCRCFAIL;                             /*!<Command response received (CRC check failed)  */
        __I  uint32_t DCRCFAIL;                             /*!<Data block sent/received (CRC check failed)   */
        __I  uint32_t CTIMEOUT;                             /*!<Command response timeout                      */
        __I  uint32_t DTIMEOUT;                             /*!<Data timeout                                  */
        __I  uint32_t TXUNDERR;                             /*!<Transmit FIFO underrun error                  */
        __I  uint32_t RXOVERR;                              /*!<Received FIFO overrun error                   */
        __I  uint32_t CMDREND;                              /*!<Command response received (CRC check passed)  */
        __I  uint32_t CMDSENT;                              /*!<Command sent (no response required)           */
        __I  uint32_t DATAEND;                              /*!<Data end (data counter, SDIDCOUNT, is zero)   */
        __I  uint32_t STBITERR;                             /*!<Start bit not detected on all data signals in wide bus mode */
        __I  uint32_t DBCKEND;                              /*!<Data block sent/received (CRC check passed)   */
        __I  uint32_t CMDACT;                               /*!<Command transfer in progress                  */
        __I  uint32_t TXACT;                                /*!<Data transmit in progress                     */
        __I  uint32_t RXACT;                                /*!<Data receive in progress                      */
        __I  uint32_t TXFIFOHE;                             /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
        __I  uint32_t RXFIFOHF;                             /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
        __I  uint32_t TXFIFOF;                              /*!<Transmit FIFO full                            */
        __I  uint32_t RXFIFOF;                              /*!<Receive FIFO full                             */
        __I  uint32_t TXFIFOE;                              /*!<Transmit FIFO empty                           */
        __I  uint32_t RXFIFOE;                              /*!<Receive FIFO empty                            */
        __I  uint32_t TXDAVL;                               /*!<Data available in transmit FIFO               */
        __I  uint32_t RXDAVL;                               /*!<Data available in receive FIFO                */
        __I  uint32_t SDIOIT;                               /*!<SDIO interrupt received                       */
        __I  uint32_t CEATAEND;                             /*!<CE-ATA command completion signal received for CMD61 */
             uint32_t __RESERVED0[8];
    } STA;                                                  /*!< SDIO status register,           Address offset: 0x34 */
    struct {
        __IO uint32_t CCRCFAILC;                            /*!<CCRCFAIL flag clear bit */
        __IO uint32_t DCRCFAILC;                            /*!<DCRCFAIL flag clear bit */
        __IO uint32_t CTIMEOUTC;                            /*!<CTIMEOUT flag clear bit */
        __IO uint32_t DTIMEOUTC;                            /*!<DTIMEOUT flag clear bit */
        __IO uint32_t TXUNDERRC;                            /*!<TXUNDERR flag clear bit */
        __IO uint32_t RXOVERRC;                             /*!<RXOVERR flag clear bit  */
        __IO uint32_t CMDRENDC;                             /*!<CMDREND flag clear bit  */
        __IO uint32_t CMDSENTC;                             /*!<CMDSENT flag clear bit  */
        __IO uint32_t DATAENDC;                             /*!<DATAEND flag clear bit  */
        __IO uint32_t STBITERRC;                            /*!<STBITERR flag clear bit */
        __IO uint32_t DBCKENDC;                             /*!<DBCKEND flag clear bit  */
             uint32_t __RESERVED0[11];
        __IO uint32_t SDIOITC;                              /*!<SDIOIT flag clear bit   */
        __IO uint32_t CEATAENDC;                            /*!<CEATAEND flag clear bit */
             uint32_t __RESERVED1[8];
    } ICR;                                                  /*!< SDIO interrupt clear register,  Address offset: 0x38 */
    struct {
        __IO uint32_t CCRCFAILIE;                           /*!<Command CRC Fail Interrupt Enable          */
        __IO uint32_t DCRCFAILIE;                           /*!<Data CRC Fail Interrupt Enable             */
        __IO uint32_t CTIMEOUTIE;                           /*!<Command TimeOut Interrupt Enable           */
        __IO uint32_t DTIMEOUTIE;                           /*!<Data TimeOut Interrupt Enable              */
        __IO uint32_t TXUNDERRIE;                           /*!<Tx FIFO UnderRun Error Interrupt Enable    */
        __IO uint32_t RXOVERRIE;                            /*!<Rx FIFO OverRun Error Interrupt Enable     */
        __IO uint32_t CMDRENDIE;                            /*!<Command Response Received Interrupt Enable */
        __IO uint32_t CMDSENTIE;                            /*!<Command Sent Interrupt Enable              */
        __IO uint32_t DATAENDIE;                            /*!<Data End Interrupt Enable                  */
        __IO uint32_t STBITERRIE;                           /*!<Start Bit Error Interrupt Enable           */
        __IO uint32_t DBCKENDIE;                            /*!<Data Block End Interrupt Enable            */
        __IO uint32_t CMDACTIE;                             /*!<CCommand Acting Interrupt Enable           */
        __IO uint32_t TXACTIE;                              /*!<Data Transmit Acting Interrupt Enable      */
        __IO uint32_t RXACTIE;                              /*!<Data receive acting interrupt enabled      */
        __IO uint32_t TXFIFOHEIE;                           /*!<Tx FIFO Half Empty interrupt Enable        */
        __IO uint32_t RXFIFOHFIE;                           /*!<Rx FIFO Half Full interrupt Enable         */
        __IO uint32_t TXFIFOFIE;                            /*!<Tx FIFO Full interrupt Enable              */
        __IO uint32_t RXFIFOFIE;                            /*!<Rx FIFO Full interrupt Enable              */
        __IO uint32_t TXFIFOEIE;                            /*!<Tx FIFO Empty interrupt Enable             */
        __IO uint32_t RXFIFOEIE;                            /*!<Rx FIFO Empty interrupt Enable             */
        __IO uint32_t TXDAVLIE;                             /*!<Data available in Tx FIFO interrupt Enable */
        __IO uint32_t RXDAVLIE;                             /*!<Data available in Rx FIFO interrupt Enable */
        __IO uint32_t SDIOITIE;                             /*!<SDIO Mode Interrupt Received interrupt Enable */
        __IO uint32_t CEATAENDIE;                           /*!<CE-ATA command completion signal received Interrupt Enable */
             uint32_t __RESERVED0[8];
    } MASK;                                                 /*!< SDIO mask register,             Address offset: 0x3C */
         uint32_t __RESERVED0[2][32];
    __I  uint32_t FIFOCNT[32];                              /*!< SDIO FIFO counter register,     Address offset: 0x48 */
         uint32_t __RESERVED1[13][32];
    __IO uint32_t FIFO[32];                                 /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_BitBand_TypeDef;



/**
  * @brief Serial Peripheral Interface
  */
typedef struct {
    union {
        struct {
            __IO uint32_t CPHA : 1;                         /*!<Clock Phase      */
            __IO uint32_t CPOL : 1;                         /*!<Clock Polarity   */
            __IO uint32_t MSTR : 1;                         /*!<Master Selection */
            __IO uint32_t BR : 3;                           /*!<BR[2:0] bits (Baud Rate Control) */
            __IO uint32_t SPE : 1;                          /*!<SPI Enable                          */
            __IO uint32_t LSBFIRST : 1;                     /*!<Frame Format                        */
            __IO uint32_t SSI : 1;                          /*!<Internal slave select               */
            __IO uint32_t SSM : 1;                          /*!<Software slave management           */
            __IO uint32_t RXONLY : 1;                       /*!<Receive only                        */
            __IO uint32_t DFF : 1;                          /*!<Data Frame Format                   */
            __IO uint32_t CRCNEXT : 1;                      /*!<Transmit CRC next                   */
            __IO uint32_t CRCEN : 1;                        /*!<Hardware CRC calculation enable     */
            __IO uint32_t BIDIOE : 1;                       /*!<Output enable in bidirectional mode */
            __IO uint32_t BIDIMODE : 1;                     /*!<Bidirectional data mode enable      */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t RXDMAEN : 1;                      /*!<Rx Buffer DMA Enable                 */
            __IO uint32_t TXDMAEN : 1;                      /*!<Tx Buffer DMA Enable                 */
            __IO uint32_t SSOE : 1;                         /*!<SS Output Enable                     */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t FRF : 1;                          /*!<Frame Format                         */
            __IO uint32_t ERRIE : 1;                        /*!<Error Interrupt Enable               */
            __IO uint32_t RXNEIE : 1;                       /*!<RX buffer Not Empty Interrupt Enable */
            __IO uint32_t TXEIE : 1;                        /*!<Tx buffer Empty Interrupt Enable     */
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< SPI control register 2,                             Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t RXNE : 1;                         /*!<Receive buffer Not Empty */
            __IO uint32_t TXE : 1;                          /*!<Transmit buffer Empty    */
            __IO uint32_t CHSIDE : 1;                       /*!<Channel side             */
            __IO uint32_t UDR : 1;                          /*!<Underrun flag            */
            __IO uint32_t CRCERR : 1;                       /*!<CRC Error flag           */
            __IO uint32_t MODF : 1;                         /*!<Mode fault               */
            __IO uint32_t OVR : 1;                          /*!<Overrun flag             */
            __IO uint32_t BSY : 1;                          /*!<Busy flag                */
            __IO uint32_t FRE : 1;                          /*!<Frame format error flag  */
                 uint32_t __RESERVED0 : 23;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< SPI status register,                                Address offset: 0x08 */
    __IO uint32_t DR;                                       /*!< SPI data register,                                  Address offset: 0x0C */
    __IO uint32_t CRCPR;                                    /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
    __IO uint32_t RXCRCR;                                   /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
    __IO uint32_t TXCRCR;                                   /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t CHLEN : 1;                        /*!<Channel length (number of bits per audio channel) */
            __IO uint32_t DATLEN : 2;                       /*!<DATLEN[1:0] bits (Data length to be transferred)  */
            __IO uint32_t CKPOL : 1;                        /*!<steady state clock polarity               */
            __IO uint32_t I2SSTD : 2;                       /*!<I2SSTD[1:0] bits (I2S standard selection) */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PCMSYNC : 1;                      /*!<PCM frame synchronization                 */
            __IO uint32_t I2SCFG : 2;                       /*!<I2SCFG[1:0] bits (I2S configuration mode) */
            __IO uint32_t I2SE : 1;                         /*!<I2S Enable         */
            __IO uint32_t I2SMOD : 1;                       /*!<I2S mode selection */
                 uint32_t __RESERVED1 : 20;
        } b;
        __IO uint32_t w;
    } I2SCFGR;                                              /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t I2SDIV : 8;                       /*!<I2S Linear prescaler         */
            __IO uint32_t ODD : 1;                          /*!<Odd factor for the prescaler */
            __IO uint32_t MCKOE : 1;                        /*!<Master Clock Output Enable   */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } I2SPR;                                                /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CPHA;                                 /*!<Clock Phase      */
        __IO uint32_t CPOL;                                 /*!<Clock Polarity   */
        __IO uint32_t MSTR;                                 /*!<Master Selection */
        __IO uint32_t BR[3];                                /*!<BR[2:0] bits (Baud Rate Control) */
        __IO uint32_t SPE;                                  /*!<SPI Enable                          */
        __IO uint32_t LSBFIRST;                             /*!<Frame Format                        */
        __IO uint32_t SSI;                                  /*!<Internal slave select               */
        __IO uint32_t SSM;                                  /*!<Software slave management           */
        __IO uint32_t RXONLY;                               /*!<Receive only                        */
        __IO uint32_t DFF;                                  /*!<Data Frame Format                   */
        __IO uint32_t CRCNEXT;                              /*!<Transmit CRC next                   */
        __IO uint32_t CRCEN;                                /*!<Hardware CRC calculation enable     */
        __IO uint32_t BIDIOE;                               /*!<Output enable in bidirectional mode */
        __IO uint32_t BIDIMODE;                             /*!<Bidirectional data mode enable      */
             uint32_t __RESERVED0[16];
    } CR1;                                                  /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
    struct {
        __IO uint32_t RXDMAEN;                              /*!<Rx Buffer DMA Enable                 */
        __IO uint32_t TXDMAEN;                              /*!<Tx Buffer DMA Enable                 */
        __IO uint32_t SSOE;                                 /*!<SS Output Enable                     */
             uint32_t __RESERVED0;
        __IO uint32_t FRF;                                  /*!<Frame Format                         */
        __IO uint32_t ERRIE;                                /*!<Error Interrupt Enable               */
        __IO uint32_t RXNEIE;                               /*!<RX buffer Not Empty Interrupt Enable */
        __IO uint32_t TXEIE;                                /*!<Tx buffer Empty Interrupt Enable     */
             uint32_t __RESERVED1[24];
    } CR2;                                                  /*!< SPI control register 2,                             Address offset: 0x04 */
    struct {
        __IO uint32_t RXNE;                                 /*!<Receive buffer Not Empty */
        __IO uint32_t TXE;                                  /*!<Transmit buffer Empty    */
        __IO uint32_t CHSIDE;                               /*!<Channel side             */
        __IO uint32_t UDR;                                  /*!<Underrun flag            */
        __IO uint32_t CRCERR;                               /*!<CRC Error flag           */
        __IO uint32_t MODF;                                 /*!<Mode fault               */
        __IO uint32_t OVR;                                  /*!<Overrun flag             */
        __IO uint32_t BSY;                                  /*!<Busy flag                */
        __IO uint32_t FRE;                                  /*!<Frame format error flag  */
             uint32_t __RESERVED0[23];
    } SR;                                                   /*!< SPI status register,                                Address offset: 0x08 */
    __IO uint32_t DR[32];                                   /*!< SPI data register,                                  Address offset: 0x0C */
    __IO uint32_t CRCPR[32];                                /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
    __IO uint32_t RXCRCR[32];                               /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
    __IO uint32_t TXCRCR[32];                               /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
    struct {
        __IO uint32_t CHLEN;                                /*!<Channel length (number of bits per audio channel) */
        __IO uint32_t DATLEN[2];                            /*!<DATLEN[1:0] bits (Data length to be transferred)  */
        __IO uint32_t CKPOL;                                /*!<steady state clock polarity               */
        __IO uint32_t I2SSTD[2];                            /*!<I2SSTD[1:0] bits (I2S standard selection) */
             uint32_t __RESERVED0;
        __IO uint32_t PCMSYNC;                              /*!<PCM frame synchronization                 */
        __IO uint32_t I2SCFG[2];                            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
        __IO uint32_t I2SE;                                 /*!<I2S Enable         */
        __IO uint32_t I2SMOD;                               /*!<I2S mode selection */
             uint32_t __RESERVED1[20];
    } I2SCFGR;                                              /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
    struct {
        __IO uint32_t I2SDIV[8];                            /*!<I2S Linear prescaler         */
        __IO uint32_t ODD;                                  /*!<Odd factor for the prescaler */
        __IO uint32_t MCKOE;                                /*!<Master Clock Output Enable   */
             uint32_t __RESERVED0[22];
    } I2SPR;                                                /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_BitBand_TypeDef;



/**
  * @brief TIM
  */
typedef struct {
    union {
        struct {
            __IO uint32_t CEN : 1;                          /*!<Counter enable        */
            __IO uint32_t UDIS : 1;                         /*!<Update disable        */
            __IO uint32_t URS : 1;                          /*!<Update request source */
            __IO uint32_t OPM : 1;                          /*!<One pulse mode        */
            __IO uint32_t DIR : 1;                          /*!<Direction             */
            __IO uint32_t CMS : 2;                          /*!<CMS[1:0] bits (Center-aligned mode selection) */
            __IO uint32_t ARPE : 1;                         /*!<Auto-reload preload enable     */
            __IO uint32_t CKD : 2;                          /*!<CKD[1:0] bits (clock division) */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< TIM control register 1,              Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CCPC : 1;                         /*!<Capture/Compare Preloaded Control        */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CCUS : 1;                         /*!<Capture/Compare Control Update Selection */
            __IO uint32_t CCDS : 1;                         /*!<Capture/Compare DMA Selection            */
            __IO uint32_t MMS : 3;                          /*!<MMS[2:0] bits (Master Mode Selection) */
            __IO uint32_t TI1S : 1;                         /*!<TI1 Selection */
            __IO uint32_t OIS1 : 1;                         /*!<Output Idle state 1 (OC1 output)  */
            __IO uint32_t OIS1N : 1;                        /*!<Output Idle state 1 (OC1N output) */
            __IO uint32_t OIS2 : 1;                         /*!<Output Idle state 2 (OC2 output)  */
            __IO uint32_t OIS2N : 1;                        /*!<Output Idle state 2 (OC2N output) */
            __IO uint32_t OIS3 : 1;                         /*!<Output Idle state 3 (OC3 output)  */
            __IO uint32_t OIS3N : 1;                        /*!<Output Idle state 3 (OC3N output) */
            __IO uint32_t OIS4 : 1;                         /*!<Output Idle state 4 (OC4 output)  */
                 uint32_t __RESERVED1 : 17;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< TIM control register 2,              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t SMS : 3;                          /*!<SMS[2:0] bits (Slave mode selection)    */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TS : 3;                           /*!<TS[2:0] bits (Trigger selection)        */
            __IO uint32_t MSM : 1;                          /*!<Master/slave mode                       */
            __IO uint32_t ETF : 4;                          /*!<ETF[3:0] bits (External trigger filter) */
            __IO uint32_t ETPS : 2;                         /*!<ETPS[1:0] bits (External trigger prescaler) */
            __IO uint32_t ECE : 1;                          /*!<External clock enable     */
            __IO uint32_t ETP : 1;                          /*!<External trigger polarity */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } SMCR;                                                 /*!< TIM slave mode control register,     Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t UIE : 1;                          /*!<Update interrupt enable */
            __IO uint32_t CC1IE : 1;                        /*!<Capture/Compare 1 interrupt enable   */
            __IO uint32_t CC2IE : 1;                        /*!<Capture/Compare 2 interrupt enable   */
            __IO uint32_t CC3IE : 1;                        /*!<Capture/Compare 3 interrupt enable   */
            __IO uint32_t CC4IE : 1;                        /*!<Capture/Compare 4 interrupt enable   */
            __IO uint32_t COMIE : 1;                        /*!<COM interrupt enable                 */
            __IO uint32_t TIE : 1;                          /*!<Trigger interrupt enable             */
            __IO uint32_t BIE : 1;                          /*!<Break interrupt enable               */
            __IO uint32_t UDE : 1;                          /*!<Update DMA request enable            */
            __IO uint32_t CC1DE : 1;                        /*!<Capture/Compare 1 DMA request enable */
            __IO uint32_t CC2DE : 1;                        /*!<Capture/Compare 2 DMA request enable */
            __IO uint32_t CC3DE : 1;                        /*!<Capture/Compare 3 DMA request enable */
            __IO uint32_t CC4DE : 1;                        /*!<Capture/Compare 4 DMA request enable */
            __IO uint32_t COMDE : 1;                        /*!<COM DMA request enable               */
            __IO uint32_t TDE : 1;                          /*!<Trigger DMA request enable           */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } DIER;                                                 /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t UIF : 1;                          /*!<Update interrupt Flag              */
            __IO uint32_t CC1IF : 1;                        /*!<Capture/Compare 1 interrupt Flag   */
            __IO uint32_t CC2IF : 1;                        /*!<Capture/Compare 2 interrupt Flag   */
            __IO uint32_t CC3IF : 1;                        /*!<Capture/Compare 3 interrupt Flag   */
            __IO uint32_t CC4IF : 1;                        /*!<Capture/Compare 4 interrupt Flag   */
            __IO uint32_t COMIF : 1;                        /*!<COM interrupt Flag                 */
            __IO uint32_t TIF : 1;                          /*!<Trigger interrupt Flag             */
            __IO uint32_t BIF : 1;                          /*!<Break interrupt Flag               */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CC1OF : 1;                        /*!<Capture/Compare 1 Overcapture Flag */
            __IO uint32_t CC2OF : 1;                        /*!<Capture/Compare 2 Overcapture Flag */
            __IO uint32_t CC3OF : 1;                        /*!<Capture/Compare 3 Overcapture Flag */
            __IO uint32_t CC4OF : 1;                        /*!<Capture/Compare 4 Overcapture Flag */
                 uint32_t __RESERVED1 : 19;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< TIM status register,                 Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t UG : 1;                           /*!<Update Generation                         */
            __IO uint32_t CC1G : 1;                         /*!<Capture/Compare 1 Generation              */
            __IO uint32_t CC2G : 1;                         /*!<Capture/Compare 2 Generation              */
            __IO uint32_t CC3G : 1;                         /*!<Capture/Compare 3 Generation              */
            __IO uint32_t CC4G : 1;                         /*!<Capture/Compare 4 Generation              */
            __IO uint32_t COMG : 1;                         /*!<Capture/Compare Control Update Generation */
            __IO uint32_t TG : 1;                           /*!<Trigger Generation                        */
            __IO uint32_t BG : 1;                           /*!<Break Generation                          */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } EGR;                                                  /*!< TIM event generation register,       Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t C1S : 2;                          /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
            __IO uint32_t C1FE : 1;                         /*!< Output Compare 1 Fast enable */
            __IO uint32_t C1PE : 1;                         /*!< Output Compare 1 Preload enable */
            __IO uint32_t C1M : 3;                          /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
            __IO uint32_t C1CE : 1;                         /*!< Output Compare 1 Clear Enable */
            __IO uint32_t C2S : 2;                          /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
            __IO uint32_t C2FE : 1;                         /*!< Output Compare 2 Fast enable */
            __IO uint32_t C2PE : 1;                         /*!< Output Compare 2 Preload enable */
            __IO uint32_t C2M : 3;                          /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
            __IO uint32_t C2CE : 1;                         /*!< Output Compare 2 Clear Enable */
                 uint32_t __RESERVED0 : 16;
        } OC;                                               /*!< TIM CCMR register Output Compare configuration mode */
        struct {
            __IO uint32_t C1S : 2;                          /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
            __IO uint32_t C1PSC : 2;                        /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
            __IO uint32_t C1F : 4;                          /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
            __IO uint32_t C2S : 2;                          /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
            __IO uint32_t C2PSC : 2;                        /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
            __IO uint32_t C2F : 4;                          /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
                 uint32_t __RESERVED0 : 16;
        } IC;                                               /*!< TIM CCMR register Input Capture configuration mode */
        __IO uint32_t w;
    } CCMR1;                                                /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t C3S : 2;                          /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
            __IO uint32_t C3FE : 1;                         /*!< Output Compare 3 Fast enable */
            __IO uint32_t C3PE : 1;                         /*!< Output Compare 3 Preload enable */
            __IO uint32_t C3M : 3;                          /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
            __IO uint32_t C3CE : 1;                         /*!< Output Compare 3 Clear Enable */
            __IO uint32_t C4S : 2;                          /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
            __IO uint32_t C4FE : 1;                         /*!< Output Compare 4 Fast enable */
            __IO uint32_t C4PE : 1;                         /*!< Output Compare 4 Preload enable */
            __IO uint32_t C4M : 3;                          /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
            __IO uint32_t C4CE : 1;                         /*!< Output Compare 4 Clear Enable */
                 uint32_t __RESERVED0 : 16;
        } OC;                                               /*!< TIM CCMR register Output Compare configuration mode */
        struct {
            __IO uint32_t C3S : 2;                          /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
            __IO uint32_t C3PSC : 2;                        /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
            __IO uint32_t C3F : 4;                          /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
            __IO uint32_t C4S : 2;                          /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
            __IO uint32_t C4PSC : 2;                        /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
            __IO uint32_t C4F : 4;                          /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
                 uint32_t __RESERVED0 : 16;
        } IC;                                               /*!< TIM CCMR register Input Capture configuration mode */
        __IO uint32_t w;
    } CCMR2;                                                /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t CC1E : 1;                         /*!<Capture/Compare 1 output enable                 */
            __IO uint32_t CC1P : 1;                         /*!<Capture/Compare 1 output Polarity               */
            __IO uint32_t CC1NE : 1;                        /*!<Capture/Compare 1 Complementary output enable   */
            __IO uint32_t CC1NP : 1;                        /*!<Capture/Compare 1 Complementary output Polarity */
            __IO uint32_t CC2E : 1;                         /*!<Capture/Compare 2 output enable                 */
            __IO uint32_t CC2P : 1;                         /*!<Capture/Compare 2 output Polarity               */
            __IO uint32_t CC2NE : 1;                        /*!<Capture/Compare 2 Complementary output enable   */
            __IO uint32_t CC2NP : 1;                        /*!<Capture/Compare 2 Complementary output Polarity */
            __IO uint32_t CC3E : 1;                         /*!<Capture/Compare 3 output enable                 */
            __IO uint32_t CC3P : 1;                         /*!<Capture/Compare 3 output Polarity               */
            __IO uint32_t CC3NE : 1;                        /*!<Capture/Compare 3 Complementary output enable   */
            __IO uint32_t CC3NP : 1;                        /*!<Capture/Compare 3 Complementary output Polarity */
            __IO uint32_t CC4E : 1;                         /*!<Capture/Compare 4 output enable                 */
            __IO uint32_t CC4P : 1;                         /*!<Capture/Compare 4 output Polarity               */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CC4NP : 1;                        /*!<Capture/Compare 4 Complementary output Polarity */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CCER;                                                 /*!< TIM capture/compare enable register, Address offset: 0x20 */
    __IO uint32_t CNT;                                      /*!< TIM counter register,                Address offset: 0x24 */
    __IO uint32_t PSC;                                      /*!< TIM prescaler,                       Address offset: 0x28 */
    __IO uint32_t ARR;                                      /*!< TIM auto-reload register,            Address offset: 0x2C */
    __IO uint32_t RCR;                                      /*!< TIM repetition counter register,     Address offset: 0x30 */
    __IO uint32_t CCR1;                                     /*!< TIM capture/compare register 1,      Address offset: 0x34 */
    __IO uint32_t CCR2;                                     /*!< TIM capture/compare register 2,      Address offset: 0x38 */
    __IO uint32_t CCR3;                                     /*!< TIM capture/compare register 3,      Address offset: 0x3C */
    __IO uint32_t CCR4;                                     /*!< TIM capture/compare register 4,      Address offset: 0x40 */
    union {
        struct {
            __IO uint32_t DTG : 8;                          /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
            __IO uint32_t LOCK : 2;                         /*!<LOCK[1:0] bits (Lock Configuration) */
            __IO uint32_t OSSI : 1;                         /*!<Off-State Selection for Idle mode */
            __IO uint32_t OSSR : 1;                         /*!<Off-State Selection for Run mode  */
            __IO uint32_t BKE : 1;                          /*!<Break enable                      */
            __IO uint32_t BKP : 1;                          /*!<Break Polarity                    */
            __IO uint32_t AOE : 1;                          /*!<Automatic Output enable           */
            __IO uint32_t MOE : 1;                          /*!<Main Output enable                */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } BDTR;                                                 /*!< TIM break and dead-time register,    Address offset: 0x44 */
    union {
        struct {
            __IO uint32_t DBA : 5;                          /*!<DBA[4:0] bits (DMA Base Address) */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t DBL : 5;                          /*!<DBL[4:0] bits (DMA Burst Length) */
                 uint32_t __RESERVED1 : 19;
        } b;
        __IO uint32_t w;
    } DCR;                                                  /*!< TIM DMA control register,            Address offset: 0x48 */
    __IO uint32_t DMAR;                                     /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    __IO uint32_t OR;                                       /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CEN;                                  /*!<Counter enable        */
        __IO uint32_t UDIS;                                 /*!<Update disable        */
        __IO uint32_t URS;                                  /*!<Update request source */
        __IO uint32_t OPM;                                  /*!<One pulse mode        */
        __IO uint32_t DIR;                                  /*!<Direction             */
        __IO uint32_t CMS[2];                               /*!<CMS[1:0] bits (Center-aligned mode selection) */
        __IO uint32_t ARPE;                                 /*!<Auto-reload preload enable     */
        __IO uint32_t CKD[2];                               /*!<CKD[1:0] bits (clock division) */
             uint32_t __RESERVED0[22];
    } CR1;                                                  /*!< TIM control register 1,              Address offset: 0x00 */
    struct {
        __IO uint32_t CCPC;                                 /*!<Capture/Compare Preloaded Control        */
             uint32_t __RESERVED0;
        __IO uint32_t CCUS;                                 /*!<Capture/Compare Control Update Selection */
        __IO uint32_t CCDS;                                 /*!<Capture/Compare DMA Selection            */
        __IO uint32_t MMS[3];                               /*!<MMS[2:0] bits (Master Mode Selection) */
        __IO uint32_t TI1S;                                 /*!<TI1 Selection */
        __IO uint32_t OIS1;                                 /*!<Output Idle state 1 (OC1 output)  */
        __IO uint32_t OIS1N;                                /*!<Output Idle state 1 (OC1N output) */
        __IO uint32_t OIS2;                                 /*!<Output Idle state 2 (OC2 output)  */
        __IO uint32_t OIS2N;                                /*!<Output Idle state 2 (OC2N output) */
        __IO uint32_t OIS3;                                 /*!<Output Idle state 3 (OC3 output)  */
        __IO uint32_t OIS3N;                                /*!<Output Idle state 3 (OC3N output) */
        __IO uint32_t OIS4;                                 /*!<Output Idle state 4 (OC4 output)  */
             uint32_t __RESERVED1[17];
    } CR2;                                                  /*!< TIM control register 2,              Address offset: 0x04 */
    struct {
        __IO uint32_t SMS[3];                               /*!<SMS[2:0] bits (Slave mode selection)    */
             uint32_t __RESERVED0;
        __IO uint32_t TS[3];                                /*!<TS[2:0] bits (Trigger selection)        */
        __IO uint32_t MSM;                                  /*!<Master/slave mode                       */
        __IO uint32_t ETF[4];                               /*!<ETF[3:0] bits (External trigger filter) */
        __IO uint32_t ETPS[2];                              /*!<ETPS[1:0] bits (External trigger prescaler) */
        __IO uint32_t ECE;                                  /*!<External clock enable     */
        __IO uint32_t ETP;                                  /*!<External trigger polarity */
             uint32_t __RESERVED1[16];
    } SMCR;                                                 /*!< TIM slave mode control register,     Address offset: 0x08 */
    struct {
        __IO uint32_t UIE;                                  /*!<Update interrupt enable */
        __IO uint32_t CC1IE;                                /*!<Capture/Compare 1 interrupt enable   */
        __IO uint32_t CC2IE;                                /*!<Capture/Compare 2 interrupt enable   */
        __IO uint32_t CC3IE;                                /*!<Capture/Compare 3 interrupt enable   */
        __IO uint32_t CC4IE;                                /*!<Capture/Compare 4 interrupt enable   */
        __IO uint32_t COMIE;                                /*!<COM interrupt enable                 */
        __IO uint32_t TIE;                                  /*!<Trigger interrupt enable             */
        __IO uint32_t BIE;                                  /*!<Break interrupt enable               */
        __IO uint32_t UDE;                                  /*!<Update DMA request enable            */
        __IO uint32_t CC1DE;                                /*!<Capture/Compare 1 DMA request enable */
        __IO uint32_t CC2DE;                                /*!<Capture/Compare 2 DMA request enable */
        __IO uint32_t CC3DE;                                /*!<Capture/Compare 3 DMA request enable */
        __IO uint32_t CC4DE;                                /*!<Capture/Compare 4 DMA request enable */
        __IO uint32_t COMDE;                                /*!<COM DMA request enable               */
        __IO uint32_t TDE;                                  /*!<Trigger DMA request enable           */
             uint32_t __RESERVED0[17];
    } DIER;                                                 /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    struct {
        __IO uint32_t UIF;                                  /*!<Update interrupt Flag              */
        __IO uint32_t CC1IF;                                /*!<Capture/Compare 1 interrupt Flag   */
        __IO uint32_t CC2IF;                                /*!<Capture/Compare 2 interrupt Flag   */
        __IO uint32_t CC3IF;                                /*!<Capture/Compare 3 interrupt Flag   */
        __IO uint32_t CC4IF;                                /*!<Capture/Compare 4 interrupt Flag   */
        __IO uint32_t COMIF;                                /*!<COM interrupt Flag                 */
        __IO uint32_t TIF;                                  /*!<Trigger interrupt Flag             */
        __IO uint32_t BIF;                                  /*!<Break interrupt Flag               */
             uint32_t __RESERVED0;
        __IO uint32_t CC1OF;                                /*!<Capture/Compare 1 Overcapture Flag */
        __IO uint32_t CC2OF;                                /*!<Capture/Compare 2 Overcapture Flag */
        __IO uint32_t CC3OF;                                /*!<Capture/Compare 3 Overcapture Flag */
        __IO uint32_t CC4OF;                                /*!<Capture/Compare 4 Overcapture Flag */
             uint32_t __RESERVED1[19];
    } SR;                                                   /*!< TIM status register,                 Address offset: 0x10 */
    struct {
        __IO uint32_t UG;                                   /*!<Update Generation                         */
        __IO uint32_t CC1G;                                 /*!<Capture/Compare 1 Generation              */
        __IO uint32_t CC2G;                                 /*!<Capture/Compare 2 Generation              */
        __IO uint32_t CC3G;                                 /*!<Capture/Compare 3 Generation              */
        __IO uint32_t CC4G;                                 /*!<Capture/Compare 4 Generation              */
        __IO uint32_t COMG;                                 /*!<Capture/Compare Control Update Generation */
        __IO uint32_t TG;                                   /*!<Trigger Generation                        */
        __IO uint32_t BG;                                   /*!<Break Generation                          */
             uint32_t __RESERVED0[24];
    } EGR;                                                  /*!< TIM event generation register,       Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t C1S[2];                           /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
            __IO uint32_t C1FE;                             /*!< Output Compare 1 Fast enable */
            __IO uint32_t C1PE;                             /*!< Output Compare 1 Preload enable */
            __IO uint32_t C1M[3];                           /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
            __IO uint32_t C1CE;                             /*!< Output Compare 1 Clear Enable */
            __IO uint32_t C2S[2];                           /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
            __IO uint32_t C2FE;                             /*!< Output Compare 2 Fast enable */
            __IO uint32_t C2PE;                             /*!< Output Compare 2 Preload enable */
            __IO uint32_t C2M[3];                           /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
            __IO uint32_t C2CE;                             /*!< Output Compare 2 Clear Enable */
                 uint32_t __RESERVED0[16];
        } OC;                                               /*!< TIM CCMR register Output Compare configuration mode */
        struct {
            __IO uint32_t C1S[2];                           /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
            __IO uint32_t C1PSC[2];                         /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
            __IO uint32_t C1F[4];                           /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
            __IO uint32_t C2S[2];                           /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
            __IO uint32_t C2PSC[2];                         /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
            __IO uint32_t C2F[4];                           /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
                 uint32_t __RESERVED0[16];
        } IC;                                               /*!< TIM CCMR register Input Capture configuration mode */
    } CCMR1;                                                /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t C3S[2];                           /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
            __IO uint32_t C3FE;                             /*!< Output Compare 3 Fast enable */
            __IO uint32_t C3PE;                             /*!< Output Compare 3 Preload enable */
            __IO uint32_t C3M[3];                           /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
            __IO uint32_t C3CE;                             /*!< Output Compare 3 Clear Enable */
            __IO uint32_t C4S[2];                           /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
            __IO uint32_t C4FE;                             /*!< Output Compare 4 Fast enable */
            __IO uint32_t C4PE;                             /*!< Output Compare 4 Preload enable */
            __IO uint32_t C4M[3];                           /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
            __IO uint32_t C4CE;                             /*!< Output Compare 4 Clear Enable */
                 uint32_t __RESERVED0[16];
        } OC;                                               /*!< TIM CCMR register Output Compare configuration mode */
        struct {
            __IO uint32_t C3S[2];                           /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
            __IO uint32_t C3PSC[2];                         /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
            __IO uint32_t C3F[4];                           /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
            __IO uint32_t C4S[2];                           /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
            __IO uint32_t C4PSC[2];                         /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
            __IO uint32_t C4F[4];                           /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
                 uint32_t __RESERVED0[16];
        } IC;                                               /*!< TIM CCMR register Input Capture configuration mode */
    } CCMR2;                                                /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
    struct {
        __IO uint32_t CC1E;                                 /*!<Capture/Compare 1 output enable                 */
        __IO uint32_t CC1P;                                 /*!<Capture/Compare 1 output Polarity               */
        __IO uint32_t CC1NE;                                /*!<Capture/Compare 1 Complementary output enable   */
        __IO uint32_t CC1NP;                                /*!<Capture/Compare 1 Complementary output Polarity */
        __IO uint32_t CC2E;                                 /*!<Capture/Compare 2 output enable                 */
        __IO uint32_t CC2P;                                 /*!<Capture/Compare 2 output Polarity               */
        __IO uint32_t CC2NE;                                /*!<Capture/Compare 2 Complementary output enable   */
        __IO uint32_t CC2NP;                                /*!<Capture/Compare 2 Complementary output Polarity */
        __IO uint32_t CC3E;                                 /*!<Capture/Compare 3 output enable                 */
        __IO uint32_t CC3P;                                 /*!<Capture/Compare 3 output Polarity               */
        __IO uint32_t CC3NE;                                /*!<Capture/Compare 3 Complementary output enable   */
        __IO uint32_t CC3NP;                                /*!<Capture/Compare 3 Complementary output Polarity */
        __IO uint32_t CC4E;                                 /*!<Capture/Compare 4 output enable                 */
        __IO uint32_t CC4P;                                 /*!<Capture/Compare 4 output Polarity               */
             uint32_t __RESERVED0;
        __IO uint32_t CC4NP;                                /*!<Capture/Compare 4 Complementary output Polarity */
             uint32_t __RESERVED1[16];
    } CCER;                                                 /*!< TIM capture/compare enable register, Address offset: 0x20 */
    __IO uint32_t CNT[32];                                  /*!< TIM counter register,                Address offset: 0x24 */
    __IO uint32_t PSC[32];                                  /*!< TIM prescaler,                       Address offset: 0x28 */
    __IO uint32_t ARR[32];                                  /*!< TIM auto-reload register,            Address offset: 0x2C */
    __IO uint32_t RCR[32];                                  /*!< TIM repetition counter register,     Address offset: 0x30 */
    __IO uint32_t CCR1[32];                                 /*!< TIM capture/compare register 1,      Address offset: 0x34 */
    __IO uint32_t CCR2[32];                                 /*!< TIM capture/compare register 2,      Address offset: 0x38 */
    __IO uint32_t CCR3[32];                                 /*!< TIM capture/compare register 3,      Address offset: 0x3C */
    __IO uint32_t CCR4[32];                                 /*!< TIM capture/compare register 4,      Address offset: 0x40 */
    struct {
        __IO uint32_t DTG[8];                               /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
        __IO uint32_t LOCK[2];                              /*!<LOCK[1:0] bits (Lock Configuration) */
        __IO uint32_t OSSI;                                 /*!<Off-State Selection for Idle mode */
        __IO uint32_t OSSR;                                 /*!<Off-State Selection for Run mode  */
        __IO uint32_t BKE;                                  /*!<Break enable                      */
        __IO uint32_t BKP;                                  /*!<Break Polarity                    */
        __IO uint32_t AOE;                                  /*!<Automatic Output enable           */
        __IO uint32_t MOE;                                  /*!<Main Output enable                */
             uint32_t __RESERVED0[16];
    } BDTR;                                                 /*!< TIM break and dead-time register,    Address offset: 0x44 */
    struct {
        __IO uint32_t DBA[5];                               /*!<DBA[4:0] bits (DMA Base Address) */
             uint32_t __RESERVED0[3];
        __IO uint32_t DBL[5];                               /*!<DBL[4:0] bits (DMA Burst Length) */
             uint32_t __RESERVED1[19];
    } DCR;                                                  /*!< TIM DMA control register,            Address offset: 0x48 */
    __IO uint32_t DMAR[32];                                 /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    __IO uint32_t OR[32];                                   /*!< TIM option register,                 Address offset: 0x50 */
} TIM_BitBand_TypeDef;



/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct {
    union {
        struct {
            __IO uint32_t PE : 1;                           /*!<Parity Error                 */
            __IO uint32_t FE : 1;                           /*!<Framing Error                */
            __IO uint32_t NE : 1;                           /*!<Noise Error Flag             */
            __IO uint32_t ORE : 1;                          /*!<OverRun Error                */
            __IO uint32_t IDLE : 1;                         /*!<IDLE line detected           */
            __IO uint32_t RXNE : 1;                         /*!<Read Data Register Not Empty */
            __IO uint32_t TC : 1;                           /*!<Transmission Complete        */
            __IO uint32_t TXE : 1;                          /*!<Transmit Data Register Empty */
            __IO uint32_t LBD : 1;                          /*!<LIN Break Detection Flag     */
            __IO uint32_t CTS : 1;                          /*!<CTS Flag                     */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< USART Status register,                   Address offset: 0x00 */
    __IO uint32_t DR;                                       /*!< USART Data register,                     Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t DIV_Fraction : 4;                 /*!<Fraction of USARTDIV */
            __IO uint32_t DIV_Mantissa : 12;                /*!<Mantissa of USARTDIV */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } BRR;                                                  /*!< USART Baud rate register,                Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t SBK : 1;                          /*!<Send Break                             */
            __IO uint32_t RWU : 1;                          /*!<Receiver wakeup                        */
            __IO uint32_t RE : 1;                           /*!<Receiver Enable                        */
            __IO uint32_t TE : 1;                           /*!<Transmitter Enable                     */
            __IO uint32_t IDLEIE : 1;                       /*!<IDLE Interrupt Enable                  */
            __IO uint32_t RXNEIE : 1;                       /*!<RXNE Interrupt Enable                  */
            __IO uint32_t TCIE : 1;                         /*!<Transmission Complete Interrupt Enable */
            __IO uint32_t TXEIE : 1;                        /*!<TXE Interrupt Enable                   */
            __IO uint32_t PEIE : 1;                         /*!<PE Interrupt Enable                    */
            __IO uint32_t PS : 1;                           /*!<Parity Selection                       */
            __IO uint32_t PCE : 1;                          /*!<Parity Control Enable                  */
            __IO uint32_t WAKE : 1;                         /*!<Wakeup method                          */
            __IO uint32_t M : 1;                            /*!<Word length                            */
            __IO uint32_t UE : 1;                           /*!<USART Enable                           */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t OVER8 : 1;                        /*!<USART Oversampling by 8 enable         */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< USART Control register 1,                Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t ADD : 4;                          /*!<Address of the USART node            */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t LBDL : 1;                         /*!<LIN Break Detection Length           */
            __IO uint32_t LBDIE : 1;                        /*!<LIN Break Detection Interrupt Enable */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t LBCL : 1;                         /*!<Last Bit Clock pulse                 */
            __IO uint32_t CPHA : 1;                         /*!<Clock Phase                          */
            __IO uint32_t CPOL : 1;                         /*!<Clock Polarity                       */
            __IO uint32_t CLKEN : 1;                        /*!<Clock Enable                         */
            __IO uint32_t STOP : 2;                         /*!<STOP[1:0] bits (STOP bits) */
            __IO uint32_t LINEN : 1;                        /*!<LIN mode enable */
                 uint32_t __RESERVED2 : 17;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< USART Control register 2,                Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t EIE : 1;                          /*!<Error Interrupt Enable      */
            __IO uint32_t IREN : 1;                         /*!<IrDA mode Enable            */
            __IO uint32_t IRLP : 1;                         /*!<IrDA Low-Power              */
            __IO uint32_t HDSEL : 1;                        /*!<Half-Duplex Selection       */
            __IO uint32_t NACK : 1;                         /*!<Smartcard NACK enable       */
            __IO uint32_t SCEN : 1;                         /*!<Smartcard mode enable       */
            __IO uint32_t DMAR : 1;                         /*!<DMA Enable Receiver         */
            __IO uint32_t DMAT : 1;                         /*!<DMA Enable Transmitter      */
            __IO uint32_t RTSE : 1;                         /*!<RTS Enable                  */
            __IO uint32_t CTSE : 1;                         /*!<CTS Enable                  */
            __IO uint32_t CTSIE : 1;                        /*!<CTS Interrupt Enable        */
            __IO uint32_t ONEBIT : 1;                       /*!<USART One bit method enable */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } CR3;                                                  /*!< USART Control register 3,                Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t PSC : 8;                          /*!<PSC[7:0] bits (Prescaler value) */
            __IO uint32_t GT : 8;                           /*!<Guard time value */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } GTPR;                                                 /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PE;                                   /*!<Parity Error                 */
        __IO uint32_t FE;                                   /*!<Framing Error                */
        __IO uint32_t NE;                                   /*!<Noise Error Flag             */
        __IO uint32_t ORE;                                  /*!<OverRun Error                */
        __IO uint32_t IDLE;                                 /*!<IDLE line detected           */
        __IO uint32_t RXNE;                                 /*!<Read Data Register Not Empty */
        __IO uint32_t TC;                                   /*!<Transmission Complete        */
        __IO uint32_t TXE;                                  /*!<Transmit Data Register Empty */
        __IO uint32_t LBD;                                  /*!<LIN Break Detection Flag     */
        __IO uint32_t CTS;                                  /*!<CTS Flag                     */
             uint32_t __RESERVED0[22];
    } SR;                                                   /*!< USART Status register,                   Address offset: 0x00 */
    __IO uint32_t DR[32];                                   /*!< USART Data register,                     Address offset: 0x04 */
    struct {
        __IO uint32_t DIV_Fraction[4];                      /*!<Fraction of USARTDIV */
        __IO uint32_t DIV_Mantissa[12];                     /*!<Mantissa of USARTDIV */
             uint32_t __RESERVED0[16];
    } BRR;                                                  /*!< USART Baud rate register,                Address offset: 0x08 */
    struct {
        __IO uint32_t SBK;                                  /*!<Send Break                             */
        __IO uint32_t RWU;                                  /*!<Receiver wakeup                        */
        __IO uint32_t RE;                                   /*!<Receiver Enable                        */
        __IO uint32_t TE;                                   /*!<Transmitter Enable                     */
        __IO uint32_t IDLEIE;                               /*!<IDLE Interrupt Enable                  */
        __IO uint32_t RXNEIE;                               /*!<RXNE Interrupt Enable                  */
        __IO uint32_t TCIE;                                 /*!<Transmission Complete Interrupt Enable */
        __IO uint32_t TXEIE;                                /*!<TXE Interrupt Enable                   */
        __IO uint32_t PEIE;                                 /*!<PE Interrupt Enable                    */
        __IO uint32_t PS;                                   /*!<Parity Selection                       */
        __IO uint32_t PCE;                                  /*!<Parity Control Enable                  */
        __IO uint32_t WAKE;                                 /*!<Wakeup method                          */
        __IO uint32_t M;                                    /*!<Word length                            */
        __IO uint32_t UE;                                   /*!<USART Enable                           */
             uint32_t __RESERVED0;
        __IO uint32_t OVER8;                                /*!<USART Oversampling by 8 enable         */
             uint32_t __RESERVED1[16];
    } CR1;                                                  /*!< USART Control register 1,                Address offset: 0x0C */
    struct {
        __IO uint32_t ADD[4];                               /*!<Address of the USART node            */
             uint32_t __RESERVED0;
        __IO uint32_t LBDL;                                 /*!<LIN Break Detection Length           */
        __IO uint32_t LBDIE;                                /*!<LIN Break Detection Interrupt Enable */
             uint32_t __RESERVED1;
        __IO uint32_t LBCL;                                 /*!<Last Bit Clock pulse                 */
        __IO uint32_t CPHA;                                 /*!<Clock Phase                          */
        __IO uint32_t CPOL;                                 /*!<Clock Polarity                       */
        __IO uint32_t CLKEN;                                /*!<Clock Enable                         */
        __IO uint32_t STOP[2];                              /*!<STOP[1:0] bits (STOP bits) */
        __IO uint32_t LINEN;                                /*!<LIN mode enable */
             uint32_t __RESERVED2[17];
    } CR2;                                                  /*!< USART Control register 2,                Address offset: 0x10 */
    struct {
        __IO uint32_t EIE;                                  /*!<Error Interrupt Enable      */
        __IO uint32_t IREN;                                 /*!<IrDA mode Enable            */
        __IO uint32_t IRLP;                                 /*!<IrDA Low-Power              */
        __IO uint32_t HDSEL;                                /*!<Half-Duplex Selection       */
        __IO uint32_t NACK;                                 /*!<Smartcard NACK enable       */
        __IO uint32_t SCEN;                                 /*!<Smartcard mode enable       */
        __IO uint32_t DMAR;                                 /*!<DMA Enable Receiver         */
        __IO uint32_t DMAT;                                 /*!<DMA Enable Transmitter      */
        __IO uint32_t RTSE;                                 /*!<RTS Enable                  */
        __IO uint32_t CTSE;                                 /*!<CTS Enable                  */
        __IO uint32_t CTSIE;                                /*!<CTS Interrupt Enable        */
        __IO uint32_t ONEBIT;                               /*!<USART One bit method enable */
             uint32_t __RESERVED0[20];
    } CR3;                                                  /*!< USART Control register 3,                Address offset: 0x14 */
    struct {
        __IO uint32_t PSC[8];                               /*!<PSC[7:0] bits (Prescaler value) */
        __IO uint32_t GT[8];                                /*!<Guard time value */
             uint32_t __RESERVED0[16];
    } GTPR;                                                 /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_BitBand_TypeDef;



/** 
  * @brief Window WATCHDOG
  */
typedef struct {
    union {
        struct {
            __IO uint32_t T : 7;                            /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
            __IO uint32_t WDGA : 1;                         /*!<Activation bit */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< WWDG Control register,       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t W : 7;                            /*!<W[6:0] bits (7-bit window value) */
            __IO uint32_t WDGTB : 2;                        /*!<WDGTB[1:0] bits (Timer Base) */
            __IO uint32_t EWI : 1;                          /*!<Early Wakeup Interrupt */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } CFR;                                                  /*!< WWDG Configuration register, Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t EWIF : 1;                         /*!<Early Wakeup Interrupt Flag */
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;


typedef struct {
    struct {
        __IO uint32_t T[7];                                 /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
        __IO uint32_t WDGA;                                 /*!<Activation bit */
             uint32_t __RESERVED0[24];
    } CR;                                                   /*!< WWDG Control register,       Address offset: 0x00 */
    struct {
        __IO uint32_t W[7];                                 /*!<W[6:0] bits (7-bit window value) */
        __IO uint32_t WDGTB[2];                             /*!<WDGTB[1:0] bits (Timer Base) */
        __IO uint32_t EWI;                                  /*!<Early Wakeup Interrupt */
             uint32_t __RESERVED0[22];
    } CFR;                                                  /*!< WWDG Configuration register, Address offset: 0x04 */
    struct {
        __IO uint32_t EWIF;                                 /*!<Early Wakeup Interrupt Flag */
             uint32_t __RESERVED0[31];
    } SR;                                                   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_BitBand_TypeDef;



/**
  * @brief USB_OTG_Core_Registers
  */
typedef struct {
    union {
        struct {
            __IO uint32_t SRQSCS : 1;                       /*!< Session request success */
            __IO uint32_t SRQ : 1;                          /*!< Session request */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t HNGSCS : 1;                       /*!< Host set HNP enable */
            __IO uint32_t HNPRQ : 1;                        /*!< HNP request */
            __IO uint32_t HSHNPEN : 1;                      /*!< Host set HNP enable */
            __IO uint32_t DHNPEN : 1;                       /*!< Device HNP enabled */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t CIDSTS : 1;                       /*!< Connector ID status */
            __IO uint32_t DBCT : 1;                         /*!< Long/short debounce time */
            __IO uint32_t ASVLD : 1;                        /*!< A-session valid  */
            __IO uint32_t BSVLD : 1;                        /*!< B-session valid */
                 uint32_t __RESERVED2 : 12;
        } b;
        __IO uint32_t w;
    } GOTGCTL;                                              /*!< USB_OTG Control and Status Register          000h */
    union {
        struct {
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t SEDET : 1;                        /*!< Session end detected                   */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t SRSSCHG : 1;                      /*!< Session request success status change  */
            __IO uint32_t HNSSCHG : 1;                      /*!< Host negotiation success status change */
                 uint32_t __RESERVED2 : 7;
            __IO uint32_t HNGDET : 1;                       /*!< Host negotiation detected              */
            __IO uint32_t ADTOCHG : 1;                      /*!< A-device timeout change                */
            __IO uint32_t DBCDNE : 1;                       /*!< Debounce done                          */
                 uint32_t __RESERVED3 : 12;
        } b;
        __IO uint32_t w;
    } GOTGINT;                                              /*!< USB_OTG Interrupt Register                   004h */
    union {
        struct {
            __IO uint32_t GINT : 1;                         /*!< Global interrupt mask */
            __IO uint32_t HBSTLEN : 4;                      /*!< Burst length/type */
            __IO uint32_t DMAEN : 1;                        /*!< DMA enable */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TXFELVL : 1;                      /*!< TxFIFO empty level */
            __IO uint32_t PTXFELVL : 1;                     /*!< Periodic TxFIFO empty level */
                 uint32_t __RESERVED1 : 23;
        } b;
        __IO uint32_t w;
    } GAHBCFG;                                              /*!< Core AHB Configuration Register              008h */
    union {
        struct {
            __IO uint32_t TOCAL : 3;                        /*!< FS timeout calibration */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t PHYSEL : 1;                       /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SRPCAP : 1;                       /*!< SRP-capable */
            __IO uint32_t HNPCAP : 1;                       /*!< HNP-capable */
            __IO uint32_t TRDT : 4;                         /*!< USB turnaround time */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t PHYLPCS : 1;                      /*!< PHY Low-power clock select */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t ULPIFSLS : 1;                     /*!< ULPI FS/LS select               */
            __IO uint32_t ULPIAR : 1;                       /*!< ULPI Auto-resume                */
            __IO uint32_t ULPICSM : 1;                      /*!< ULPI Clock SuspendM             */
            __IO uint32_t ULPIEVBUSD : 1;                   /*!< ULPI External VBUS Drive        */
            __IO uint32_t ULPIEVBUSI : 1;                   /*!< ULPI external VBUS indicator    */
            __IO uint32_t TSDPS : 1;                        /*!< TermSel DLine pulsing selection */
            __IO uint32_t PCCI : 1;                         /*!< Indicator complement            */
            __IO uint32_t PTCI : 1;                         /*!< Indicator pass through          */
            __IO uint32_t ULPIIPD : 1;                      /*!< ULPI interface protect disable  */
                 uint32_t __RESERVED4 : 3;
            __IO uint32_t FHMOD : 1;                        /*!< Forced host mode                */
            __IO uint32_t FDMOD : 1;                        /*!< Forced peripheral mode          */
            __IO uint32_t CTXPKT : 1;                       /*!< Corrupt Tx packet               */
        } b;
        __IO uint32_t w;
    } GUSBCFG;                                              /*!< Core USB Configuration Register              00Ch */
    union {
        struct {
            __IO uint32_t CSRST : 1;                        /*!< Core soft reset          */
            __IO uint32_t HSRST : 1;                        /*!< HCLK soft reset          */
            __IO uint32_t FCRST : 1;                        /*!< Host frame counter reset */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RXFFLSH : 1;                      /*!< RxFIFO flush             */
            __IO uint32_t TXFFLSH : 1;                      /*!< TxFIFO flush             */
            __IO uint32_t TXFNUM : 5;                       /*!< TxFIFO number */
                 uint32_t __RESERVED1 : 19;
            __IO uint32_t DMAREQ : 1;                       /*!< DMA request signal */
            __IO uint32_t AHBIDL : 1;                       /*!< AHB master idle */
        } b;
        __IO uint32_t w;
    } GRSTCTL;                                              /*!< Core Reset Register                          010h */
    union {
        struct {
            __IO uint32_t CMOD : 1;                         /*!< Current mode of operation                      */
            __IO uint32_t MMIS : 1;                         /*!< Mode mismatch interrupt                        */
            __IO uint32_t OTGINT : 1;                       /*!< OTG interrupt                                  */
            __IO uint32_t SOF : 1;                          /*!< Start of frame                                 */
            __IO uint32_t RXFLVL : 1;                       /*!< RxFIFO nonempty                                */
            __IO uint32_t NPTXFE : 1;                       /*!< Nonperiodic TxFIFO empty                       */
            __IO uint32_t GINAKEFF : 1;                     /*!< Global IN nonperiodic NAK effective            */
            __IO uint32_t BOUTNAKEFF : 1;                   /*!< Global OUT NAK effective                       */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t ESUSP : 1;                        /*!< Early suspend                                  */
            __IO uint32_t USBSUSP : 1;                      /*!< USB suspend                                    */
            __IO uint32_t USBRST : 1;                       /*!< USB reset                                      */
            __IO uint32_t ENUMDNE : 1;                      /*!< Enumeration done                               */
            __IO uint32_t ISOODRP : 1;                      /*!< Isochronous OUT packet dropped interrupt       */
            __IO uint32_t EOPF : 1;                         /*!< End of periodic frame interrupt                */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t IEPINT : 1;                       /*!< IN endpoint interrupt                          */
            __IO uint32_t OEPINT : 1;                       /*!< OUT endpoint interrupt                         */
            __IO uint32_t IISOIXFR : 1;                     /*!< Incomplete isochronous IN transfer             */
            __IO uint32_t PXFR_INCOMPISOOUT : 1;            /*!< Incomplete periodic transfer                   */
            __IO uint32_t DATAFSUSP : 1;                    /*!< Data fetch suspended                           */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t HPRTINT : 1;                      /*!< Host port interrupt                            */
            __IO uint32_t HCINT : 1;                        /*!< Host channels interrupt                        */
            __IO uint32_t PTXFE : 1;                        /*!< Periodic TxFIFO empty                          */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CIDSCHG : 1;                      /*!< Connector ID status change                     */
            __IO uint32_t DISCINT : 1;                      /*!< Disconnect detected interrupt                  */
            __IO uint32_t SRQINT : 1;                       /*!< Session request/new session detected interrupt */
            __IO uint32_t WKUINT : 1;                       /*!< Resume/remote wakeup detected interrupt        */
        } b;
        __IO uint32_t w;
    } GINTSTS;                                              /*!< Core Interrupt Register                      014h */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t MMISM : 1;                        /*!< Mode mismatch interrupt mask                        */
            __IO uint32_t OTGINT : 1;                       /*!< OTG interrupt mask                                  */
            __IO uint32_t SOFM : 1;                         /*!< Start of frame mask                                 */
            __IO uint32_t RXFLVLM : 1;                      /*!< Receive FIFO nonempty mask                          */
            __IO uint32_t NPTXFEM : 1;                      /*!< Nonperiodic TxFIFO empty mask                       */
            __IO uint32_t GINAKEFFM : 1;                    /*!< Global nonperiodic IN NAK effective mask            */
            __IO uint32_t GONAKEFFM : 1;                    /*!< Global OUT NAK effective mask                       */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ESUSPM : 1;                       /*!< Early suspend mask                                  */
            __IO uint32_t USBSUSPM : 1;                     /*!< USB suspend mask                                    */
            __IO uint32_t USBRST : 1;                       /*!< USB reset mask                                      */
            __IO uint32_t ENUMDNEM : 1;                     /*!< Enumeration done mask                               */
            __IO uint32_t ISOODRPM : 1;                     /*!< Isochronous OUT packet dropped interrupt mask       */
            __IO uint32_t EOPFM : 1;                        /*!< End of periodic frame interrupt mask                */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t EPMISM : 1;                       /*!< Endpoint mismatch interrupt mask                    */
            __IO uint32_t IEPINT : 1;                       /*!< IN endpoints interrupt mask                         */
            __IO uint32_t OEPINT : 1;                       /*!< OUT endpoints interrupt mask                        */
            __IO uint32_t IISOIXFRM : 1;                    /*!< Incomplete isochronous IN transfer mask             */
            __IO uint32_t PXFRM_IISOOXFRM : 1;              /*!< Incomplete periodic transfer mask                   */
            __IO uint32_t FSUSPM : 1;                       /*!< Data fetch suspended mask                           */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t PRTIM : 1;                        /*!< Host port interrupt mask                            */
            __IO uint32_t HCIM : 1;                         /*!< Host channels interrupt mask                        */
            __IO uint32_t PTXFEM : 1;                       /*!< Periodic TxFIFO empty mask                          */
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t CIDSCHGM : 1;                     /*!< Connector ID status change mask                     */
            __IO uint32_t DISCINT : 1;                      /*!< Disconnect detected interrupt mask                  */
            __IO uint32_t SRQIM : 1;                        /*!< Session request/new session detected interrupt mask */
            __IO uint32_t WUIM : 1;                         /*!< Resume/remote wakeup detected interrupt mask        */
        } b;
        __IO uint32_t w;
    } GINTMSK;                                              /*!< Core Interrupt Mask Register                 018h */
    union {
        struct {
            __IO uint32_t CHNUM_EPNUM : 4;
            __IO uint32_t BCNT : 11;
            __IO uint32_t DPID : 2;
            __IO uint32_t PKTSTS : 4;
            __IO uint32_t FRMNUM : 4;
                 uint32_t __RESERVED0 : 7;
        } b;
        __IO uint32_t w;
    } GRXSTSR;                                              /*!< Receive Sts Q Read Register                  01Ch */
    union {
        struct {
            __IO uint32_t CHNUM_EPNUM : 4;                  /*!< IN EP interrupt mask bits  */
            __IO uint32_t BCNT : 11;                        /*!< OUT EP interrupt mask bits */
            __IO uint32_t DPID : 2;                         /*!< OUT EP interrupt mask bits */
            __IO uint32_t PKTSTS : 4;                       /*!< OUT EP interrupt mask bits */
            __IO uint32_t FRMNUM : 4;
                 uint32_t __RESERVED0 : 7;
        } b;
        __IO uint32_t w;
    } GRXSTSP;                                              /*!< Receive Sts Q Read & POP Register            020h */
    __IO uint32_t GRXFSIZ;                                  /*!< Receive FIFO Size Register                   024h */
    union {
        struct {
            __IO uint32_t TXFSA : 16;                       /*!< IN endpoint FIFOx transmit RAM start address */
            __IO uint32_t TXFD : 16;                        /*!< IN endpoint TxFIFO depth */
        } b;
        __IO uint32_t w;
    } DIEPTXF0_HNPTXFSIZ;                                   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
    union {
        struct {
            __IO uint32_t NPTXFSAV : 16;
            __IO uint32_t NPTQXSAV : 8;
            __IO uint32_t NPTXQTOP : 7;
                 uint32_t __RESERVED0 : 1;
        } b;
        __IO uint32_t w;
    } HNPTXSTS;                                             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
         uint32_t __RESERVED0[2];
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t PWRDWN : 1;                       /*!< Power down */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t VBUSASEN : 1;                     /*!< Enable the VBUS sensing device */
            __IO uint32_t VBUSBSEN : 1;                     /*!< Enable the VBUS sensing device */
            __IO uint32_t SOFOUTEN : 1;                     /*!< SOF output enable */
            __IO uint32_t NOVBUSSENS : 1;                   /*!< VBUS sensing disable option*/
                 uint32_t __RESERVED2 : 10;
        } b;
        __IO uint32_t w;
    } GCCFG;                                                /*!< General Purpose IO Register                  038h */
    __IO uint32_t CID;                                      /*!< User ID Register                             03Ch */
         uint32_t __RESERVED1[48];
    union {
        struct {
            __IO uint32_t PTXSA : 16;                       /*!< Host periodic TxFIFO start address            */
            __IO uint32_t PTXFD : 16;                       /*!< Host periodic TxFIFO depth                    */
        } b;
        __IO uint32_t w;
    } HPTXFSIZ;                                             /*!< Host Periodic Tx FIFO Size Reg               100h */
    union {
        struct {
            __IO uint32_t INEPTXSA : 16;                    /*!< IN endpoint FIFOx transmit RAM start address */
            __IO uint32_t INEPTXFD : 16;                    /*!< IN endpoint TxFIFO depth */
        } b;
        __IO uint32_t w;
    } DIEPTXF[7];                                           /*!< Periodic Transmit FIFO                       104h-144h */
} USB_OTG_GlobalTypeDef;



/**
  * @brief USB_OTG_device_Registers
  */
typedef struct {
    union {
        struct {
            __IO uint32_t DSPD : 2;                         /*!< Device speed */
            __IO uint32_t NZLSOHSK : 1;                     /*!< Nonzero-length status OUT handshake */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DAD : 7;                          /*!< Device address */
            __IO uint32_t PFIVL : 2;                        /*!< Periodic (micro)frame interval */
                 uint32_t __RESERVED1 : 11;
            __IO uint32_t PERSCHIVL : 2;                    /*!< Periodic scheduling interval */
                 uint32_t __RESERVED2 : 6;
        } b;
        __IO uint32_t w;
    } DCFG;                                                 /*!< dev Configuration Register   800h */
    union {
        struct {
            __IO uint32_t RWUSIG : 1;                       /*!< Remote wakeup signaling */
            __IO uint32_t SDIS : 1;                         /*!< Soft disconnect         */
            __IO uint32_t GINSTS : 1;                       /*!< Global IN NAK status    */
            __IO uint32_t GONSTS : 1;                       /*!< Global OUT NAK status   */
            __IO uint32_t TCTL : 3;                         /*!< Test control */
            __IO uint32_t SGINAK : 1;                       /*!< Set global IN NAK         */
            __IO uint32_t CGINAK : 1;                       /*!< Clear global IN NAK       */
            __IO uint32_t SGONAK : 1;                       /*!< Set global OUT NAK        */
            __IO uint32_t CGONAK : 1;                       /*!< Clear global OUT NAK      */
            __IO uint32_t POPRGDNE : 1;                     /*!< Power-on programming done */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DCTL;                                                 /*!< dev Control Register         804h */
    union {
        struct {
            __IO uint32_t SUSPSTS : 1;                      /*!< Suspend status   */
            __IO uint32_t ENUMSPD : 2;                      /*!< Enumerated speed */
            __IO uint32_t EERR : 1;                         /*!< Erratic error     */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t FNSOF : 14;                       /*!< Frame number of the received SOF */
                 uint32_t __RESERVED1 : 10;
        } b;
        __IO uint32_t w;
    } DSTS;                                                 /*!< dev Status Register (RO)     808h */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t XFRCM : 1;                        /*!< Transfer completed interrupt mask                 */
            __IO uint32_t EPDM : 1;                         /*!< Endpoint disabled interrupt mask                  */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TOM : 1;                          /*!< Timeout condition mask (nonisochronous endpoints) */
            __IO uint32_t ITTXFEMSK : 1;                    /*!< IN token received when TxFIFO empty mask          */
            __IO uint32_t INEPNMM : 1;                      /*!< IN token received with EP mismatch mask           */
            __IO uint32_t INEPNEM : 1;                      /*!< IN endpoint NAK effective mask                    */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t TXFURM : 1;                       /*!< FIFO underrun mask                                */
            __IO uint32_t BIM : 1;                          /*!< BNA interrupt mask                                */
                 uint32_t __RESERVED2 : 22;
        } b;
        __IO uint32_t w;
    } DIEPMSK;                                              /*!< dev IN Endpoint Mask         810h */
    union {
        struct {
            __IO uint32_t XFRCM : 1;                        /*!< Transfer completed interrupt mask */
            __IO uint32_t EPDM : 1;                         /*!< Endpoint disabled interrupt mask               */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t STUPM : 1;                        /*!< SETUP phase done mask                          */
            __IO uint32_t OTEPDM : 1;                       /*!< OUT token received when endpoint disabled mask */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t B2BSTUP : 1;                      /*!< Back-to-back SETUP packets received mask */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t OPEM : 1;                         /*!< OUT packet error mask                          */
            __IO uint32_t BOIM : 1;                         /*!< BNA interrupt mask                             */
                 uint32_t __RESERVED3 : 22;
        } b;
        __IO uint32_t w;
    } DOEPMSK;                                              /*!< dev OUT Endpoint Mask        814h */
    union {
        struct {
            __IO uint32_t IEPINT : 16;                      /*!< IN endpoint interrupt bits  */
            __IO uint32_t OEPINT : 16;                      /*!< OUT endpoint interrupt bits */
        } b;
        __IO uint32_t w;
    } DAINT;                                                /*!< dev All Endpoints Itr Reg    818h */
    union {
        struct {
            __IO uint32_t IEPM : 16;                        /*!< IN EP interrupt mask bits */
            __IO uint32_t OEPM : 16;                        /*!< OUT EP interrupt mask bits */
        } b;
        __IO uint32_t w;
    } DAINTMSK;                                             /*!< dev All Endpoints Itr Mask   81Ch */
         uint32_t __RESERVED1[2];
    __IO uint32_t DVBUSDIS;                                 /*!< dev VBUS discharge Register  828h */
    __IO uint32_t DVBUSPULSE;                               /*!< dev VBUS Pulse Register      82Ch */
    union {
        struct {
            __IO uint32_t NONISOTHREN : 1;                  /*!< Nonisochronous IN endpoints threshold enable */
            __IO uint32_t ISOTHREN : 1;                     /*!< ISO IN endpoint threshold enable */
            __IO uint32_t TXTHRLEN : 9;                     /*!< Transmit threshold length */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RXTHREN : 1;                      /*!< Receive threshold enable */
            __IO uint32_t RXTHRLEN : 9;                     /*!< Receive threshold length */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t ARPEN : 1;                        /*!< Arbiter parking enable */
                 uint32_t __RESERVED2 : 4;
        } b;
        __IO uint32_t w;
    } DTHRCTL;                                              /*!< dev threshold                830h */
    __IO uint32_t DIEPEMPMSK;                               /*!< dev empty msk                834h */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t IEP1INT : 1;                      /*!< IN endpoint 1interrupt bit   */
                 uint32_t __RESERVED1 : 15;
            __IO uint32_t OEP1INT : 1;                      /*!< OUT endpoint 1 interrupt bit */
                 uint32_t __RESERVED2 : 14;
        } b;
        __IO uint32_t w;
    } DEACHINT;                                             /*!< dedicated EP interrupt       838h */
    __IO uint32_t DEACHMSK;                                 /*!< dedicated EP msk             83Ch */
         uint32_t __RESERVED2;
    __IO uint32_t DINEP1MSK;                                /*!< dedicated EP mask            844h */
         uint32_t __RESERVED3[15];
    __IO uint32_t DOUTEP1MSK;                               /*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;



/**
  * @brief USB_OTG_IN_Endpoint-Specific_Register
  */
typedef struct {
    union {
        struct {
            __IO uint32_t MPSIZ : 11;                       /*!< Maximum packet size              */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t USBAEP : 1;                       /*!< USB active endpoint              */
            __IO uint32_t EONUM_DPID : 1;                   /*!< Even/odd frame                   */
            __IO uint32_t NAKSTS : 1;                       /*!< NAK status                       */
            __IO uint32_t EPTYP : 2;                        /*!< Endpoint type                    */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t STALL : 1;                        /*!< STALL handshake                  */
            __IO uint32_t TXFNUM : 4;                       /*!< TxFIFO number                    */
            __IO uint32_t CNAK : 1;                         /*!< Clear NAK                        */
            __IO uint32_t SNAK : 1;                         /*!< Set NAK */
            __IO uint32_t SD0PID_SEVNFRM : 1;               /*!< Set DATA0 PID                    */
            __IO uint32_t SODDFRM : 1;                      /*!< Set odd frame                    */
            __IO uint32_t EPDIS : 1;                        /*!< Endpoint disable                 */
            __IO uint32_t EPENA : 1;                        /*!< Endpoint enable                  */
        } b;
        __IO uint32_t w;
    } DIEPCTL;                                              /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t XFRC : 1;                         /*!< Transfer completed interrupt */
            __IO uint32_t EPDISD : 1;                       /*!< Endpoint disabled interrupt */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TOC : 1;                          /*!< Timeout condition */
            __IO uint32_t ITTXFE : 1;                       /*!< IN token received when TxFIFO is empty */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t INEPNE : 1;                       /*!< IN endpoint NAK effective */
            __IO uint32_t TXFE : 1;                         /*!< Transmit FIFO empty */
            __IO uint32_t TXFIFOUDRN : 1;                   /*!< Transmit Fifo Underrun */
            __IO uint32_t BNA : 1;                          /*!< Buffer not available interrupt */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t PKTDRPSTS : 1;                    /*!< Packet dropped status */
            __IO uint32_t BERR : 1;                         /*!< Babble error interrupt */
            __IO uint32_t NAK : 1;                          /*!< NAK interrupt */
                 uint32_t __RESERVED3 : 18;
        } b;
        __IO uint32_t w;
    } DIEPINT;                                              /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
         uint32_t __RESERVED1;
    union {
        struct {
            __IO uint32_t XFRSIZ : 19;                      /*!< Transfer size */
            __IO uint32_t PKTCNT : 10;                      /*!< Packet count */
            __IO uint32_t MULCNT : 2;                       /*!< Packet count */
                 uint32_t __RESERVED0 : 1;
        } b;
        __IO uint32_t w;
    } DIEPTSIZ;                                             /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
    __IO uint32_t DIEPDMA;                                  /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
    __IO uint32_t DTXFSTS;                                  /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
         uint32_t __RESERVED2;
} USB_OTG_INEndpointTypeDef;



/**
  * @brief USB_OTG_OUT_Endpoint-Specific_Registers
  */
typedef struct {
    union {
        struct {
            __IO uint32_t MPSIZ : 11;                       /*!< Maximum packet size */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t USBAEP : 1;                       /*!< USB active endpoint */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t NAKSTS : 1;                       /*!< NAK status */
            __IO uint32_t EPTYP : 2;                        /*!< Endpoint type */
            __IO uint32_t SNPM : 1;                         /*!< Snoop mode */
            __IO uint32_t STALL : 1;                        /*!< STALL handshake */
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t CNAK : 1;                         /*!< Clear NAK */
            __IO uint32_t SNAK : 1;                         /*!< Set NAK */
            __IO uint32_t SD0PID_SEVNFRM : 1;               /*!< Set DATA0 PID */
            __IO uint32_t SODDFRM : 1;                      /*!< Set odd frame */
            __IO uint32_t EPDIS : 1;                        /*!< Endpoint disable */
            __IO uint32_t EPENA : 1;                        /*!< Endpoint enable */
        } b;
        __IO uint32_t w;
    } DOEPCTL;                                              /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t XFRC : 1;                         /*!< Transfer completed interrupt */
            __IO uint32_t EPDISD : 1;                       /*!< Endpoint disabled interrupt */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t STUP : 1;                         /*!< SETUP phase done */
            __IO uint32_t OTEPDIS : 1;                      /*!< OUT token received when endpoint disabled */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t B2BSTUP : 1;                      /*!< Back-to-back SETUP packets received */
                 uint32_t __RESERVED2 : 7;
            __IO uint32_t NYET : 1;                         /*!< NYET interrupt */
                 uint32_t __RESERVED3 : 17;
        } b;
        __IO uint32_t w;
    } DOEPINT;                                              /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
         uint32_t __RESERVED1;
    union {
        struct {
            __IO uint32_t XFRSIZ : 19;                      /*!< Transfer size */
            __IO uint32_t PKTCNT : 10;                      /*!< Packet count */
            __IO uint32_t STUPCNT : 2;                      /*!< SETUP packet count */
                 uint32_t __RESERVED0 : 1;
        } b;
        __IO uint32_t w;
    } DOEPTSIZ;                                             /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
    __IO uint32_t DOEPDMA;                                  /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
         uint32_t __RESERVED2[2];
} USB_OTG_OUTEndpointTypeDef;



/**
  * @brief USB_OTG_Host_Mode_Register_Structures
  */
typedef struct {
    union {
        struct {
            __IO uint32_t FSLSPCS : 2;                      /*!< FS/LS PHY clock select  */
            __IO uint32_t FSLSS : 1;                        /*!< FS- and LS-only support */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } HCFG;                                                 /*!< Host Configuration Register          400h */
    __IO uint32_t HFIR;                                     /*!< Host Frame Interval Register         404h */
    union {
        struct {
            __IO uint32_t FRNUM : 16;                       /*!< Frame number         */
            __IO uint32_t FTREM : 16;                       /*!< Frame time remaining */
        } b;
        __IO uint32_t w;
    } HFNUM;                                                /*!< Host Frame Nbr/Frame Remaining       408h */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t PTXFSAVL : 16;                    /*!< Periodic transmit data FIFO space available     */
            __IO uint32_t PTXQSAV : 8;                      /*!< Periodic transmit request queue space available */
            __IO uint32_t PTXQTOP : 8;                      /*!< Top of the periodic transmit request queue */
        } b;
        __IO uint32_t w;
    } HPTXSTS;                                              /*!< Host Periodic Tx FIFO/ Queue Status  410h */
    __IO uint32_t HAINT;                                    /*!< Host All Channels Interrupt Register 414h */
    __IO uint32_t HAINTMSK;                                 /*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;



/**
  * @brief USB_OTG_Host_Channel_Specific_Registers
  */
typedef struct {
    union {
        struct {
            __IO uint32_t MPSIZ : 11;                       /*!< Maximum packet size */
            __IO uint32_t EPNUM : 4;                        /*!< Endpoint number */
            __IO uint32_t EPDIR : 1;                        /*!< Endpoint direction */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t LSDEV : 1;                        /*!< Low-speed device */
            __IO uint32_t EPTYP : 2;                        /*!< Endpoint type */
            __IO uint32_t MC : 2;                           /*!< Multi Count (MC) / Error Count (EC) */
            __IO uint32_t DAD : 7;                          /*!< Device address */
            __IO uint32_t ODDFRM : 1;                       /*!< Odd frame */
            __IO uint32_t CHDIS : 1;                        /*!< Channel disable */
            __IO uint32_t CHENA : 1;                        /*!< Channel enable */
        } b;
        __IO uint32_t w;
    } HCCHAR;                                               /*!< Host Channel Characteristics Register    500h */
    union {
        struct {
            __IO uint32_t PRTADDR : 7;                      /*!< Port address */
            __IO uint32_t HUBADDR : 7;                      /*!< Hub address */
            __IO uint32_t XACTPOS : 2;                      /*!< XACTPOS */
            __IO uint32_t COMPLSPLT : 1;                    /*!< Do complete split */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t SPLITEN : 1;                      /*!< Split enable */
        } b;
        __IO uint32_t w;
    } HCSPLT;                                               /*!< Host Channel Split Control Register      504h */
    union {
        struct {
            __IO uint32_t XFRC : 1;                         /*!< Transfer completed */
            __IO uint32_t CHH : 1;                          /*!< Channel halted */
            __IO uint32_t AHBERR : 1;                       /*!< AHB error */
            __IO uint32_t STALL : 1;                        /*!< STALL response received interrupt */
            __IO uint32_t NAK : 1;                          /*!< NAK response received interrupt */
            __IO uint32_t ACK : 1;                          /*!< ACK response received/transmitted interrupt */
            __IO uint32_t NYET : 1;                         /*!< Response received interrupt */
            __IO uint32_t TXERR : 1;                        /*!< Transaction error */
            __IO uint32_t BBERR : 1;                        /*!< Babble error */
            __IO uint32_t FRMOR : 1;                        /*!< Frame overrun */
            __IO uint32_t DTERR : 1;                        /*!< Data toggle error */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } HCINT;                                                /*!< Host Channel Interrupt Register          508h */
    union {
        struct {
            __IO uint32_t XFRCM : 1;                        /*!< Transfer completed mask */
            __IO uint32_t CHHM : 1;                         /*!< Channel halted mask */
            __IO uint32_t AHBERR : 1;                       /*!< AHB error */
            __IO uint32_t STALLM : 1;                       /*!< STALL response received interrupt mask */
            __IO uint32_t NAKM : 1;                         /*!< NAK response received interrupt mask */
            __IO uint32_t ACKM : 1;                         /*!< ACK response received/transmitted interrupt mask */
            __IO uint32_t NYET : 1;                         /*!< response received interrupt mask */
            __IO uint32_t TXERRM : 1;                       /*!< Transaction error mask */
            __IO uint32_t BBERRM : 1;                       /*!< Babble error mask */
            __IO uint32_t FRMORM : 1;                       /*!< Frame overrun mask */
            __IO uint32_t DTERRM : 1;                       /*!< Data toggle error mask */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } HCINTMSK;                                             /*!< Host Channel Interrupt Mask Register     50Ch */
    union {
        struct {
            __IO uint32_t XFRSIZ : 19;                      /*!< Transfer size */
            __IO uint32_t PKTCNT : 10;                      /*!< Packet count */
            __IO uint32_t DPID : 2;                         /*!< Data PID */
            __IO uint32_t DOPING : 1;                       /*!< Do PING */
        } b;
        __IO uint32_t w;
    } HCTSIZ;                                               /*!< Host Channel Transfer Size Register      510h */
    __IO uint32_t HCDMA;                                    /*!< Host Channel DMA Address Register        514h */
         uint32_t __RESERVED0[2];
} USB_OTG_HostChannelTypeDef;



/**
  * @brief USB_OTG
  */
typedef struct {
    union {
        struct {
            __IO uint32_t SRQSCS : 1;                       /*!< Session request success */
            __IO uint32_t SRQ : 1;                          /*!< Session request */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t HNGSCS : 1;                       /*!< Host set HNP enable */
            __IO uint32_t HNPRQ : 1;                        /*!< HNP request */
            __IO uint32_t HSHNPEN : 1;                      /*!< Host set HNP enable */
            __IO uint32_t DHNPEN : 1;                       /*!< Device HNP enabled */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t CIDSTS : 1;                       /*!< Connector ID status */
            __IO uint32_t DBCT : 1;                         /*!< Long/short debounce time */
            __IO uint32_t ASVLD : 1;                        /*!< A-session valid  */
            __IO uint32_t BSVLD : 1;                        /*!< B-session valid */
                 uint32_t __RESERVED2 : 12;
        } b;
        __IO uint32_t w;
    } GOTGCTL;                                              /*!< USB_OTG Control and Status Register          000h */
    union {
        struct {
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t SEDET : 1;                        /*!< Session end detected                   */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t SRSSCHG : 1;                      /*!< Session request success status change  */
            __IO uint32_t HNSSCHG : 1;                      /*!< Host negotiation success status change */
                 uint32_t __RESERVED2 : 7;
            __IO uint32_t HNGDET : 1;                       /*!< Host negotiation detected              */
            __IO uint32_t ADTOCHG : 1;                      /*!< A-device timeout change                */
            __IO uint32_t DBCDNE : 1;                       /*!< Debounce done                          */
                 uint32_t __RESERVED3 : 12;
        } b;
        __IO uint32_t w;
    } GOTGINT;                                              /*!< USB_OTG Interrupt Register                   004h */
    union {
        struct {
            __IO uint32_t GINT : 1;                         /*!< Global interrupt mask */
            __IO uint32_t HBSTLEN : 4;                      /*!< Burst length/type */
            __IO uint32_t DMAEN : 1;                        /*!< DMA enable */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TXFELVL : 1;                      /*!< TxFIFO empty level */
            __IO uint32_t PTXFELVL : 1;                     /*!< Periodic TxFIFO empty level */
                 uint32_t __RESERVED1 : 23;
        } b;
        __IO uint32_t w;
    } GAHBCFG;                                              /*!< Core AHB Configuration Register              008h */
    union {
        struct {
            __IO uint32_t TOCAL : 3;                        /*!< FS timeout calibration */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t PHYSEL : 1;                       /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SRPCAP : 1;                       /*!< SRP-capable */
            __IO uint32_t HNPCAP : 1;                       /*!< HNP-capable */
            __IO uint32_t TRDT : 4;                         /*!< USB turnaround time */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t PHYLPCS : 1;                      /*!< PHY Low-power clock select */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t ULPIFSLS : 1;                     /*!< ULPI FS/LS select               */
            __IO uint32_t ULPIAR : 1;                       /*!< ULPI Auto-resume                */
            __IO uint32_t ULPICSM : 1;                      /*!< ULPI Clock SuspendM             */
            __IO uint32_t ULPIEVBUSD : 1;                   /*!< ULPI External VBUS Drive        */
            __IO uint32_t ULPIEVBUSI : 1;                   /*!< ULPI external VBUS indicator    */
            __IO uint32_t TSDPS : 1;                        /*!< TermSel DLine pulsing selection */
            __IO uint32_t PCCI : 1;                         /*!< Indicator complement            */
            __IO uint32_t PTCI : 1;                         /*!< Indicator pass through          */
            __IO uint32_t ULPIIPD : 1;                      /*!< ULPI interface protect disable  */
                 uint32_t __RESERVED4 : 3;
            __IO uint32_t FHMOD : 1;                        /*!< Forced host mode                */
            __IO uint32_t FDMOD : 1;                        /*!< Forced peripheral mode          */
            __IO uint32_t CTXPKT : 1;                       /*!< Corrupt Tx packet               */
        } b;
        __IO uint32_t w;
    } GUSBCFG;                                              /*!< Core USB Configuration Register              00Ch */
    union {
        struct {
            __IO uint32_t CSRST : 1;                        /*!< Core soft reset          */
            __IO uint32_t HSRST : 1;                        /*!< HCLK soft reset          */
            __IO uint32_t FCRST : 1;                        /*!< Host frame counter reset */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RXFFLSH : 1;                      /*!< RxFIFO flush             */
            __IO uint32_t TXFFLSH : 1;                      /*!< TxFIFO flush             */
            __IO uint32_t TXFNUM : 5;                       /*!< TxFIFO number */
                 uint32_t __RESERVED1 : 19;
            __IO uint32_t DMAREQ : 1;                       /*!< DMA request signal */
            __IO uint32_t AHBIDL : 1;                       /*!< AHB master idle */
        } b;
        __IO uint32_t w;
    } GRSTCTL;                                              /*!< Core Reset Register                          010h */
    union {
        struct {
            __IO uint32_t CMOD : 1;                         /*!< Current mode of operation                      */
            __IO uint32_t MMIS : 1;                         /*!< Mode mismatch interrupt                        */
            __IO uint32_t OTGINT : 1;                       /*!< OTG interrupt                                  */
            __IO uint32_t SOF : 1;                          /*!< Start of frame                                 */
            __IO uint32_t RXFLVL : 1;                       /*!< RxFIFO nonempty                                */
            __IO uint32_t NPTXFE : 1;                       /*!< Nonperiodic TxFIFO empty                       */
            __IO uint32_t GINAKEFF : 1;                     /*!< Global IN nonperiodic NAK effective            */
            __IO uint32_t BOUTNAKEFF : 1;                   /*!< Global OUT NAK effective                       */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t ESUSP : 1;                        /*!< Early suspend                                  */
            __IO uint32_t USBSUSP : 1;                      /*!< USB suspend                                    */
            __IO uint32_t USBRST : 1;                       /*!< USB reset                                      */
            __IO uint32_t ENUMDNE : 1;                      /*!< Enumeration done                               */
            __IO uint32_t ISOODRP : 1;                      /*!< Isochronous OUT packet dropped interrupt       */
            __IO uint32_t EOPF : 1;                         /*!< End of periodic frame interrupt                */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t IEPINT : 1;                       /*!< IN endpoint interrupt                          */
            __IO uint32_t OEPINT : 1;                       /*!< OUT endpoint interrupt                         */
            __IO uint32_t IISOIXFR : 1;                     /*!< Incomplete isochronous IN transfer             */
            __IO uint32_t PXFR_INCOMPISOOUT : 1;            /*!< Incomplete periodic transfer                   */
            __IO uint32_t DATAFSUSP : 1;                    /*!< Data fetch suspended                           */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t HPRTINT : 1;                      /*!< Host port interrupt                            */
            __IO uint32_t HCINT : 1;                        /*!< Host channels interrupt                        */
            __IO uint32_t PTXFE : 1;                        /*!< Periodic TxFIFO empty                          */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CIDSCHG : 1;                      /*!< Connector ID status change                     */
            __IO uint32_t DISCINT : 1;                      /*!< Disconnect detected interrupt                  */
            __IO uint32_t SRQINT : 1;                       /*!< Session request/new session detected interrupt */
            __IO uint32_t WKUINT : 1;                       /*!< Resume/remote wakeup detected interrupt        */
        } b;
        __IO uint32_t w;
    } GINTSTS;                                              /*!< Core Interrupt Register                      014h */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t MMISM : 1;                        /*!< Mode mismatch interrupt mask                        */
            __IO uint32_t OTGINT : 1;                       /*!< OTG interrupt mask                                  */
            __IO uint32_t SOFM : 1;                         /*!< Start of frame mask                                 */
            __IO uint32_t RXFLVLM : 1;                      /*!< Receive FIFO nonempty mask                          */
            __IO uint32_t NPTXFEM : 1;                      /*!< Nonperiodic TxFIFO empty mask                       */
            __IO uint32_t GINAKEFFM : 1;                    /*!< Global nonperiodic IN NAK effective mask            */
            __IO uint32_t GONAKEFFM : 1;                    /*!< Global OUT NAK effective mask                       */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ESUSPM : 1;                       /*!< Early suspend mask                                  */
            __IO uint32_t USBSUSPM : 1;                     /*!< USB suspend mask                                    */
            __IO uint32_t USBRST : 1;                       /*!< USB reset mask                                      */
            __IO uint32_t ENUMDNEM : 1;                     /*!< Enumeration done mask                               */
            __IO uint32_t ISOODRPM : 1;                     /*!< Isochronous OUT packet dropped interrupt mask       */
            __IO uint32_t EOPFM : 1;                        /*!< End of periodic frame interrupt mask                */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t EPMISM : 1;                       /*!< Endpoint mismatch interrupt mask                    */
            __IO uint32_t IEPINT : 1;                       /*!< IN endpoints interrupt mask                         */
            __IO uint32_t OEPINT : 1;                       /*!< OUT endpoints interrupt mask                        */
            __IO uint32_t IISOIXFRM : 1;                    /*!< Incomplete isochronous IN transfer mask             */
            __IO uint32_t PXFRM_IISOOXFRM : 1;              /*!< Incomplete periodic transfer mask                   */
            __IO uint32_t FSUSPM : 1;                       /*!< Data fetch suspended mask                           */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t PRTIM : 1;                        /*!< Host port interrupt mask                            */
            __IO uint32_t HCIM : 1;                         /*!< Host channels interrupt mask                        */
            __IO uint32_t PTXFEM : 1;                       /*!< Periodic TxFIFO empty mask                          */
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t CIDSCHGM : 1;                     /*!< Connector ID status change mask                     */
            __IO uint32_t DISCINT : 1;                      /*!< Disconnect detected interrupt mask                  */
            __IO uint32_t SRQIM : 1;                        /*!< Session request/new session detected interrupt mask */
            __IO uint32_t WUIM : 1;                         /*!< Resume/remote wakeup detected interrupt mask        */
        } b;
        __IO uint32_t w;
    } GINTMSK;                                              /*!< Core Interrupt Mask Register                 018h */
    union {
        struct {
            __IO uint32_t CHNUM_EPNUM : 4;
            __IO uint32_t BCNT : 11;
            __IO uint32_t DPID : 2;
            __IO uint32_t PKTSTS : 4;
            __IO uint32_t FRMNUM : 4;
                 uint32_t __RESERVED0 : 7;
        } b;
        __IO uint32_t w;
    } GRXSTSR;                                              /*!< Receive Sts Q Read Register                  01Ch */
    union {
        struct {
            __IO uint32_t CHNUM_EPNUM : 4;                  /*!< IN EP interrupt mask bits  */
            __IO uint32_t BCNT : 11;                        /*!< OUT EP interrupt mask bits */
            __IO uint32_t DPID : 2;                         /*!< OUT EP interrupt mask bits */
            __IO uint32_t PKTSTS : 4;                       /*!< OUT EP interrupt mask bits */
            __IO uint32_t FRMNUM : 4;
                 uint32_t __RESERVED0 : 7;
        } b;
        __IO uint32_t w;
    } GRXSTSP;                                              /*!< Receive Sts Q Read & POP Register            020h */
    __IO uint32_t GRXFSIZ;                                  /*!< Receive FIFO Size Register                   024h */
    union {
        struct {
            __IO uint32_t TXFSA : 16;                       /*!< IN endpoint FIFOx transmit RAM start address */
            __IO uint32_t TXFD : 16;                        /*!< IN endpoint TxFIFO depth */
        } b;
        __IO uint32_t w;
    } DIEPTXF0_HNPTXFSIZ;                                   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
    union {
        struct {
            __IO uint32_t NPTXFSAV : 16;
            __IO uint32_t NPTQXSAV : 8;
            __IO uint32_t NPTXQTOP : 7;
                 uint32_t __RESERVED0 : 1;
        } b;
        __IO uint32_t w;
    } HNPTXSTS;                                             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
         uint32_t __RESERVED0[2];
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t PWRDWN : 1;                       /*!< Power down */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t VBUSASEN : 1;                     /*!< Enable the VBUS sensing device */
            __IO uint32_t VBUSBSEN : 1;                     /*!< Enable the VBUS sensing device */
            __IO uint32_t SOFOUTEN : 1;                     /*!< SOF output enable */
            __IO uint32_t NOVBUSSENS : 1;                   /*!< VBUS sensing disable option*/
                 uint32_t __RESERVED2 : 10;
        } b;
        __IO uint32_t w;
    } GCCFG;                                                /*!< General Purpose IO Register                  038h */
    __IO uint32_t CID;                                      /*!< User ID Register                             03Ch */
         uint32_t __RESERVED1[48];
    union {
        struct {
            __IO uint32_t PTXSA : 16;                       /*!< Host periodic TxFIFO start address            */
            __IO uint32_t PTXFD : 16;                       /*!< Host periodic TxFIFO depth                    */
        } b;
        __IO uint32_t w;
    } HPTXFSIZ;                                             /*!< Host Periodic Tx FIFO Size Reg               100h */
    union {
        struct {
            __IO uint32_t INEPTXSA : 16;                    /*!< IN endpoint FIFOx transmit RAM start address */
            __IO uint32_t INEPTXFD : 16;                    /*!< IN endpoint TxFIFO depth */
        } b;
        __IO uint32_t w;
    } DIEPTXF[7];                                           /*!< Periodic Transmit FIFO                       104h-144h */
         uint32_t __RESERVED2[184];
    union {
        struct {
            __IO uint32_t FSLSPCS : 2;                      /*!< FS/LS PHY clock select  */
            __IO uint32_t FSLSS : 1;                        /*!< FS- and LS-only support */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } HCFG;                                                 /*!< Host Configuration Register          400h */
    __IO uint32_t HFIR;                                     /*!< Host Frame Interval Register         404h */
    union {
        struct {
            __IO uint32_t FRNUM : 16;                       /*!< Frame number         */
            __IO uint32_t FTREM : 16;                       /*!< Frame time remaining */
        } b;
        __IO uint32_t w;
    } HFNUM;                                                /*!< Host Frame Nbr/Frame Remaining       408h */
         uint32_t __RESERVED3;
    union {
        struct {
            __IO uint32_t PTXFSAVL : 16;                    /*!< Periodic transmit data FIFO space available     */
            __IO uint32_t PTXQSAV : 8;                      /*!< Periodic transmit request queue space available */
            __IO uint32_t PTXQTOP : 8;                      /*!< Top of the periodic transmit request queue */
        } b;
        __IO uint32_t w;
    } HPTXSTS;                                              /*!< Host Periodic Tx FIFO/ Queue Status  410h */
    __IO uint32_t HAINT;                                    /*!< Host All Channels Interrupt Register 414h */
    __IO uint32_t HAINTMSK;                                 /*!< Host All Channels Interrupt Mask     418h */
         uint32_t __RESERVED4[9];
    union {
        struct {
            __IO uint32_t PCSTS : 1;                        /*!< Port connect status */
            __IO uint32_t PCDET : 1;                        /*!< Port connect detected */
            __IO uint32_t PENA : 1;                         /*!< Port enable */
            __IO uint32_t PENCHNG : 1;                      /*!< Port enable/disable change */
            __IO uint32_t POCA : 1;                         /*!< Port overcurrent active */
            __IO uint32_t POCCHNG : 1;                      /*!< Port overcurrent change */
            __IO uint32_t PRES : 1;                         /*!< Port resume */
            __IO uint32_t PSUSP : 1;                        /*!< Port suspend */
            __IO uint32_t PRST : 1;                         /*!< Port reset */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PLSTS : 2;                        /*!< Port line status */
            __IO uint32_t PPWR : 1;                         /*!< Port power */
            __IO uint32_t PTCTL : 4;                        /*!< Port test control */
            __IO uint32_t PSPD : 2;                         /*!< Port speed */
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } HPRT;                                                 /*!< Host Port Control and Status Register 440h */
         uint32_t __RESERVED5[47];
    USB_OTG_HostChannelTypeDef HC[12];                      /*!< Host Channels                        500h-67Fh */
         uint32_t __RESERVED6[96];
    union {
        struct {
            __IO uint32_t DSPD : 2;                         /*!< Device speed */
            __IO uint32_t NZLSOHSK : 1;                     /*!< Nonzero-length status OUT handshake */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DAD : 7;                          /*!< Device address */
            __IO uint32_t PFIVL : 2;                        /*!< Periodic (micro)frame interval */
                 uint32_t __RESERVED1 : 11;
            __IO uint32_t PERSCHIVL : 2;                    /*!< Periodic scheduling interval */
                 uint32_t __RESERVED2 : 6;
        } b;
        __IO uint32_t w;
    } DCFG;                                                 /*!< dev Configuration Register   800h */
    union {
        struct {
            __IO uint32_t RWUSIG : 1;                       /*!< Remote wakeup signaling */
            __IO uint32_t SDIS : 1;                         /*!< Soft disconnect         */
            __IO uint32_t GINSTS : 1;                       /*!< Global IN NAK status    */
            __IO uint32_t GONSTS : 1;                       /*!< Global OUT NAK status   */
            __IO uint32_t TCTL : 3;                         /*!< Test control */
            __IO uint32_t SGINAK : 1;                       /*!< Set global IN NAK         */
            __IO uint32_t CGINAK : 1;                       /*!< Clear global IN NAK       */
            __IO uint32_t SGONAK : 1;                       /*!< Set global OUT NAK        */
            __IO uint32_t CGONAK : 1;                       /*!< Clear global OUT NAK      */
            __IO uint32_t POPRGDNE : 1;                     /*!< Power-on programming done */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DCTL;                                                 /*!< dev Control Register         804h */
    union {
        struct {
            __IO uint32_t SUSPSTS : 1;                      /*!< Suspend status   */
            __IO uint32_t ENUMSPD : 2;                      /*!< Enumerated speed */
            __IO uint32_t EERR : 1;                         /*!< Erratic error     */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t FNSOF : 14;                       /*!< Frame number of the received SOF */
                 uint32_t __RESERVED1 : 10;
        } b;
        __IO uint32_t w;
    } DSTS;                                                 /*!< dev Status Register (RO)     808h */
         uint32_t __RESERVED7;
    union {
        struct {
            __IO uint32_t XFRCM : 1;                        /*!< Transfer completed interrupt mask                 */
            __IO uint32_t EPDM : 1;                         /*!< Endpoint disabled interrupt mask                  */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TOM : 1;                          /*!< Timeout condition mask (nonisochronous endpoints) */
            __IO uint32_t ITTXFEMSK : 1;                    /*!< IN token received when TxFIFO empty mask          */
            __IO uint32_t INEPNMM : 1;                      /*!< IN token received with EP mismatch mask           */
            __IO uint32_t INEPNEM : 1;                      /*!< IN endpoint NAK effective mask                    */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t TXFURM : 1;                       /*!< FIFO underrun mask                                */
            __IO uint32_t BIM : 1;                          /*!< BNA interrupt mask                                */
                 uint32_t __RESERVED2 : 22;
        } b;
        __IO uint32_t w;
    } DIEPMSK;                                              /*!< dev IN Endpoint Mask         810h */
    union {
        struct {
            __IO uint32_t XFRCM : 1;                        /*!< Transfer completed interrupt mask */
            __IO uint32_t EPDM : 1;                         /*!< Endpoint disabled interrupt mask               */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t STUPM : 1;                        /*!< SETUP phase done mask                          */
            __IO uint32_t OTEPDM : 1;                       /*!< OUT token received when endpoint disabled mask */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t B2BSTUP : 1;                      /*!< Back-to-back SETUP packets received mask */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t OPEM : 1;                         /*!< OUT packet error mask                          */
            __IO uint32_t BOIM : 1;                         /*!< BNA interrupt mask                             */
                 uint32_t __RESERVED3 : 22;
        } b;
        __IO uint32_t w;
    } DOEPMSK;                                              /*!< dev OUT Endpoint Mask        814h */
    union {
        struct {
            __IO uint32_t IEPINT : 16;                      /*!< IN endpoint interrupt bits  */
            __IO uint32_t OEPINT : 16;                      /*!< OUT endpoint interrupt bits */
        } b;
        __IO uint32_t w;
    } DAINT;                                                /*!< dev All Endpoints Itr Reg    818h */
    union {
        struct {
            __IO uint32_t IEPM : 16;                        /*!< IN EP interrupt mask bits */
            __IO uint32_t OEPM : 16;                        /*!< OUT EP interrupt mask bits */
        } b;
        __IO uint32_t w;
    } DAINTMSK;                                             /*!< dev All Endpoints Itr Mask   81Ch */
         uint32_t __RESERVED8[2];
    __IO uint32_t DVBUSDIS;                                 /*!< dev VBUS discharge Register  828h */
    __IO uint32_t DVBUSPULSE;                               /*!< dev VBUS Pulse Register      82Ch */
    union {
        struct {
            __IO uint32_t NONISOTHREN : 1;                  /*!< Nonisochronous IN endpoints threshold enable */
            __IO uint32_t ISOTHREN : 1;                     /*!< ISO IN endpoint threshold enable */
            __IO uint32_t TXTHRLEN : 9;                     /*!< Transmit threshold length */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RXTHREN : 1;                      /*!< Receive threshold enable */
            __IO uint32_t RXTHRLEN : 9;                     /*!< Receive threshold length */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t ARPEN : 1;                        /*!< Arbiter parking enable */
                 uint32_t __RESERVED2 : 4;
        } b;
        __IO uint32_t w;
    } DTHRCTL;                                              /*!< dev threshold                830h */
    __IO uint32_t DIEPEMPMSK;                               /*!< dev empty msk                834h */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t IEP1INT : 1;                      /*!< IN endpoint 1interrupt bit   */
                 uint32_t __RESERVED1 : 15;
            __IO uint32_t OEP1INT : 1;                      /*!< OUT endpoint 1 interrupt bit */
                 uint32_t __RESERVED2 : 14;
        } b;
        __IO uint32_t w;
    } DEACHINT;                                             /*!< dedicated EP interrupt       838h */
    __IO uint32_t DEACHMSK;                                 /*!< dedicated EP msk             83Ch */
         uint32_t __RESERVED9;
    __IO uint32_t DINEP1MSK;                                /*!< dedicated EP mask            844h */
         uint32_t __RESERVED10[15];
    __IO uint32_t DOUTEP1MSK;                               /*!< dedicated EP msk             884h */
         uint32_t __RESERVED11[30];
    USB_OTG_INEndpointTypeDef IEP[8];
         uint32_t __RESERVED12[64];
    USB_OTG_OUTEndpointTypeDef OEP[8];
         uint32_t __RESERVED13[128];
    union {
        struct {
            __IO uint32_t STOPCLK : 1;
            __IO uint32_t GATECLK : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t PHYSUSP : 1;
                 uint32_t __RESERVED1 : 27;
        } b;
        __IO uint32_t w;
    } PCGCCTL;
         uint32_t __RESERVED14[127];
    union {
             __IO uint32_t DR;
             uint32_t __RESERVED0[1024];
    } DFIFO[8];
} USB_OTG_TypeDef;



/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            0x08000000U /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            0x20000000U /*!< SRAM1(64 KB) base address in the alias region                              */
#define PERIPH_BASE           0x40000000U /*!< Peripheral base address in the alias region                                */
#define SRAM1_BB_BASE         0x22000000U /*!< SRAM1(64 KB) base address in the bit-band region                           */
#define PERIPH_BB_BASE        0x42000000U /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000U /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x0803FFFFU /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800U /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FU /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000U)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800U)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00U)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800U)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00U)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000U)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400U)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00U)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000U)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400U)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00U)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000U)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000U)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000U)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400U)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000U)
#define ADC1_COMMON_BASE      (APB2PERIPH_BASE + 0x2300U)
/* Legacy define */
#define ADC_BASE               ADC1_COMMON_BASE
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00U)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400U)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800U)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00U)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000U)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400U)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800U)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000U)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00U)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000U)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800U)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00U)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000U)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010U)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028U)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040U)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058U)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070U)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088U)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0U)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8U)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400U)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010U)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028U)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040U)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058U)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070U)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088U)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0U)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8U)


/*!< Debug MCU registers base address */
#define DBGMCU_BASE           0xE0042000U
/*!< USB registers base address */
#define USB_OTG_FS_PERIPH_BASE               0x50000000U

#define USB_OTG_GLOBAL_BASE                  0x000U
#define USB_OTG_DEVICE_BASE                  0x800U
#define USB_OTG_IN_ENDPOINT_BASE             0x900U
#define USB_OTG_OUT_ENDPOINT_BASE            0xB00U
#define USB_OTG_EP_REG_SIZE                  0x20U
#define USB_OTG_HOST_BASE                    0x400U
#define USB_OTG_HOST_PORT_BASE               0x440U
#define USB_OTG_HOST_CHANNEL_BASE            0x500U
#define USB_OTG_HOST_CHANNEL_SIZE            0x20U
#define USB_OTG_PCGCCTL_BASE                 0xE00U
#define USB_OTG_FIFO_BASE                    0x1000U
#define USB_OTG_FIFO_SIZE                    0x1000U

#define UID_BASE                     0x1FFF7A10U           /*!< Unique device ID register base address */
#define FLASHSIZE_BASE               0x1FFF7A22U           /*!< FLASH Size register base address       */
#define PACKAGE_BASE                 0x1FFF7BF0U           /*!< Package size register base address     */
/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */  
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC1_COMMON_BASE)
/* Legacy define */
#define ADC                  ADC1_COMMON
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)


#define ADC_COMMON_BB              ((ADC_Common_BitBand_TypeDef *) PERIPH_BB(ADC_BASE))
#define ADC_BB(inst)               ((ADC_BitBand_TypeDef *) PERIPH_BB(inst))
#define CRC_BB                     ((CRC_BitBand_TypeDef *) PERIPH_BB(CRC_BASE))
#define DMA_Stream_BB(inst)        ((DMA_Stream_BitBand_TypeDef *) PERIPH_BB(inst))
#define DMA_BB(inst)               ((DMA_BitBand_TypeDef *) PERIPH_BB(inst))
#define EXTI_BB                    ((EXTI_BitBand_TypeDef *) PERIPH_BB(EXTI_BASE))
#define FLASH_BB                   ((FLASH_BitBand_TypeDef *) PERIPH_BB(FLASH_R_BASE))
#define GPIO_BB(inst)              ((GPIO_BitBand_TypeDef *) PERIPH_BB(inst))
#define I2C_BB(inst)               ((I2C_BitBand_TypeDef *) PERIPH_BB(inst))
#define IWDG_BB                    ((IWDG_BitBand_TypeDef *) PERIPH_BB(IWDG_BASE))
#define PWR_BB                     ((PWR_BitBand_TypeDef *) PERIPH_BB(PWR_BASE))
#define RCC_BB                     ((RCC_BitBand_TypeDef *) PERIPH_BB(RCC_BASE))
#define RTC_BB                     ((RTC_BitBand_TypeDef *) PERIPH_BB(RTC_BASE))
#define SDIO_BB                    ((SDIO_BitBand_TypeDef *) PERIPH_BB(SDIO_BASE))
#define SPI_BB(inst)               ((SPI_BitBand_TypeDef *) PERIPH_BB(inst))
#define SYSCFG_BB                  ((SYSCFG_BitBand_TypeDef *) PERIPH_BB(SYSCFG_BASE))
#define TIM_BB(inst)               ((TIM_BitBand_TypeDef *) PERIPH_BB(inst))
#define USART_BB(inst)             ((USART_BitBand_TypeDef *) PERIPH_BB(inst))
#define WWDG_BB                    ((WWDG_BitBand_TypeDef *) PERIPH_BB(WWDG_BASE))

/**
  * @}
  */

/** @addtogroup Peripheral_registers_bits
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define ADC_SR_AWD_Pos            (0U)                                         
#define ADC_SR_AWD_Msk            (0x1U << ADC_SR_AWD_Pos)                     /*!< 0x00000001 */
#define ADC_SR_AWD                ADC_SR_AWD_Msk                               /*!<Analog watchdog flag */
#define ADC_SR_EOC_Pos            (1U)                                         
#define ADC_SR_EOC_Msk            (0x1U << ADC_SR_EOC_Pos)                     /*!< 0x00000002 */
#define ADC_SR_EOC                ADC_SR_EOC_Msk                               /*!<End of conversion */
#define ADC_SR_JEOC_Pos           (2U)                                         
#define ADC_SR_JEOC_Msk           (0x1U << ADC_SR_JEOC_Pos)                    /*!< 0x00000004 */
#define ADC_SR_JEOC               ADC_SR_JEOC_Msk                              /*!<Injected channel end of conversion */
#define ADC_SR_JSTRT_Pos          (3U)                                         
#define ADC_SR_JSTRT_Msk          (0x1U << ADC_SR_JSTRT_Pos)                   /*!< 0x00000008 */
#define ADC_SR_JSTRT              ADC_SR_JSTRT_Msk                             /*!<Injected channel Start flag */
#define ADC_SR_STRT_Pos           (4U)                                         
#define ADC_SR_STRT_Msk           (0x1U << ADC_SR_STRT_Pos)                    /*!< 0x00000010 */
#define ADC_SR_STRT               ADC_SR_STRT_Msk                              /*!<Regular channel Start flag */
#define ADC_SR_OVR_Pos            (5U)                                         
#define ADC_SR_OVR_Msk            (0x1U << ADC_SR_OVR_Pos)                     /*!< 0x00000020 */
#define ADC_SR_OVR                ADC_SR_OVR_Msk                               /*!<Overrun flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define ADC_CR1_AWDCH_Pos         (0U)                                         
#define ADC_CR1_AWDCH_Msk         (0x1FU << ADC_CR1_AWDCH_Pos)                 /*!< 0x0000001F */
#define ADC_CR1_AWDCH             ADC_CR1_AWDCH_Msk                            /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define ADC_CR1_AWDCH_0           (0x01U << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000001 */
#define ADC_CR1_AWDCH_1           (0x02U << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000002 */
#define ADC_CR1_AWDCH_2           (0x04U << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000004 */
#define ADC_CR1_AWDCH_3           (0x08U << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000008 */
#define ADC_CR1_AWDCH_4           (0x10U << ADC_CR1_AWDCH_Pos)                 /*!< 0x00000010 */
#define ADC_CR1_EOCIE_Pos         (5U)                                         
#define ADC_CR1_EOCIE_Msk         (0x1U << ADC_CR1_EOCIE_Pos)                  /*!< 0x00000020 */
#define ADC_CR1_EOCIE             ADC_CR1_EOCIE_Msk                            /*!<Interrupt enable for EOC */
#define ADC_CR1_AWDIE_Pos         (6U)                                         
#define ADC_CR1_AWDIE_Msk         (0x1U << ADC_CR1_AWDIE_Pos)                  /*!< 0x00000040 */
#define ADC_CR1_AWDIE             ADC_CR1_AWDIE_Msk                            /*!<AAnalog Watchdog interrupt enable */
#define ADC_CR1_JEOCIE_Pos        (7U)                                         
#define ADC_CR1_JEOCIE_Msk        (0x1U << ADC_CR1_JEOCIE_Pos)                 /*!< 0x00000080 */
#define ADC_CR1_JEOCIE            ADC_CR1_JEOCIE_Msk                           /*!<Interrupt enable for injected channels */
#define ADC_CR1_SCAN_Pos          (8U)                                         
#define ADC_CR1_SCAN_Msk          (0x1U << ADC_CR1_SCAN_Pos)                   /*!< 0x00000100 */
#define ADC_CR1_SCAN              ADC_CR1_SCAN_Msk                             /*!<Scan mode */
#define ADC_CR1_AWDSGL_Pos        (9U)                                         
#define ADC_CR1_AWDSGL_Msk        (0x1U << ADC_CR1_AWDSGL_Pos)                 /*!< 0x00000200 */
#define ADC_CR1_AWDSGL            ADC_CR1_AWDSGL_Msk                           /*!<Enable the watchdog on a single channel in scan mode */
#define ADC_CR1_JAUTO_Pos         (10U)                                        
#define ADC_CR1_JAUTO_Msk         (0x1U << ADC_CR1_JAUTO_Pos)                  /*!< 0x00000400 */
#define ADC_CR1_JAUTO             ADC_CR1_JAUTO_Msk                            /*!<Automatic injected group conversion */
#define ADC_CR1_DISCEN_Pos        (11U)                                        
#define ADC_CR1_DISCEN_Msk        (0x1U << ADC_CR1_DISCEN_Pos)                 /*!< 0x00000800 */
#define ADC_CR1_DISCEN            ADC_CR1_DISCEN_Msk                           /*!<Discontinuous mode on regular channels */
#define ADC_CR1_JDISCEN_Pos       (12U)                                        
#define ADC_CR1_JDISCEN_Msk       (0x1U << ADC_CR1_JDISCEN_Pos)                /*!< 0x00001000 */
#define ADC_CR1_JDISCEN           ADC_CR1_JDISCEN_Msk                          /*!<Discontinuous mode on injected channels */
#define ADC_CR1_DISCNUM_Pos       (13U)                                        
#define ADC_CR1_DISCNUM_Msk       (0x7U << ADC_CR1_DISCNUM_Pos)                /*!< 0x0000E000 */
#define ADC_CR1_DISCNUM           ADC_CR1_DISCNUM_Msk                          /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define ADC_CR1_DISCNUM_0         (0x1U << ADC_CR1_DISCNUM_Pos)                /*!< 0x00002000 */
#define ADC_CR1_DISCNUM_1         (0x2U << ADC_CR1_DISCNUM_Pos)                /*!< 0x00004000 */
#define ADC_CR1_DISCNUM_2         (0x4U << ADC_CR1_DISCNUM_Pos)                /*!< 0x00008000 */
#define ADC_CR1_JAWDEN_Pos        (22U)                                        
#define ADC_CR1_JAWDEN_Msk        (0x1U << ADC_CR1_JAWDEN_Pos)                 /*!< 0x00400000 */
#define ADC_CR1_JAWDEN            ADC_CR1_JAWDEN_Msk                           /*!<Analog watchdog enable on injected channels */
#define ADC_CR1_AWDEN_Pos         (23U)                                        
#define ADC_CR1_AWDEN_Msk         (0x1U << ADC_CR1_AWDEN_Pos)                  /*!< 0x00800000 */
#define ADC_CR1_AWDEN             ADC_CR1_AWDEN_Msk                            /*!<Analog watchdog enable on regular channels */
#define ADC_CR1_RES_Pos           (24U)                                        
#define ADC_CR1_RES_Msk           (0x3U << ADC_CR1_RES_Pos)                    /*!< 0x03000000 */
#define ADC_CR1_RES               ADC_CR1_RES_Msk                              /*!<RES[2:0] bits (Resolution) */
#define ADC_CR1_RES_0             (0x1U << ADC_CR1_RES_Pos)                    /*!< 0x01000000 */
#define ADC_CR1_RES_1             (0x2U << ADC_CR1_RES_Pos)                    /*!< 0x02000000 */
#define ADC_CR1_OVRIE_Pos         (26U)                                        
#define ADC_CR1_OVRIE_Msk         (0x1U << ADC_CR1_OVRIE_Pos)                  /*!< 0x04000000 */
#define ADC_CR1_OVRIE             ADC_CR1_OVRIE_Msk                            /*!<overrun interrupt enable */
  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define ADC_CR2_ADON_Pos          (0U)                                         
#define ADC_CR2_ADON_Msk          (0x1U << ADC_CR2_ADON_Pos)                   /*!< 0x00000001 */
#define ADC_CR2_ADON              ADC_CR2_ADON_Msk                             /*!<A/D Converter ON / OFF */
#define ADC_CR2_CONT_Pos          (1U)                                         
#define ADC_CR2_CONT_Msk          (0x1U << ADC_CR2_CONT_Pos)                   /*!< 0x00000002 */
#define ADC_CR2_CONT              ADC_CR2_CONT_Msk                             /*!<Continuous Conversion */
#define ADC_CR2_DMA_Pos           (8U)                                         
#define ADC_CR2_DMA_Msk           (0x1U << ADC_CR2_DMA_Pos)                    /*!< 0x00000100 */
#define ADC_CR2_DMA               ADC_CR2_DMA_Msk                              /*!<Direct Memory access mode */
#define ADC_CR2_DDS_Pos           (9U)                                         
#define ADC_CR2_DDS_Msk           (0x1U << ADC_CR2_DDS_Pos)                    /*!< 0x00000200 */
#define ADC_CR2_DDS               ADC_CR2_DDS_Msk                              /*!<DMA disable selection (Single ADC) */
#define ADC_CR2_EOCS_Pos          (10U)                                        
#define ADC_CR2_EOCS_Msk          (0x1U << ADC_CR2_EOCS_Pos)                   /*!< 0x00000400 */
#define ADC_CR2_EOCS              ADC_CR2_EOCS_Msk                             /*!<End of conversion selection */
#define ADC_CR2_ALIGN_Pos         (11U)                                        
#define ADC_CR2_ALIGN_Msk         (0x1U << ADC_CR2_ALIGN_Pos)                  /*!< 0x00000800 */
#define ADC_CR2_ALIGN             ADC_CR2_ALIGN_Msk                            /*!<Data Alignment */
#define ADC_CR2_JEXTSEL_Pos       (16U)                                        
#define ADC_CR2_JEXTSEL_Msk       (0xFU << ADC_CR2_JEXTSEL_Pos)                /*!< 0x000F0000 */
#define ADC_CR2_JEXTSEL           ADC_CR2_JEXTSEL_Msk                          /*!<JEXTSEL[3:0] bits (External event select for injected group) */
#define ADC_CR2_JEXTSEL_0         (0x1U << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00010000 */
#define ADC_CR2_JEXTSEL_1         (0x2U << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00020000 */
#define ADC_CR2_JEXTSEL_2         (0x4U << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00040000 */
#define ADC_CR2_JEXTSEL_3         (0x8U << ADC_CR2_JEXTSEL_Pos)                /*!< 0x00080000 */
#define ADC_CR2_JEXTEN_Pos        (20U)                                        
#define ADC_CR2_JEXTEN_Msk        (0x3U << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00300000 */
#define ADC_CR2_JEXTEN            ADC_CR2_JEXTEN_Msk                           /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
#define ADC_CR2_JEXTEN_0          (0x1U << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00100000 */
#define ADC_CR2_JEXTEN_1          (0x2U << ADC_CR2_JEXTEN_Pos)                 /*!< 0x00200000 */
#define ADC_CR2_JSWSTART_Pos      (22U)                                        
#define ADC_CR2_JSWSTART_Msk      (0x1U << ADC_CR2_JSWSTART_Pos)               /*!< 0x00400000 */
#define ADC_CR2_JSWSTART          ADC_CR2_JSWSTART_Msk                         /*!<Start Conversion of injected channels */
#define ADC_CR2_EXTSEL_Pos        (24U)                                        
#define ADC_CR2_EXTSEL_Msk        (0xFU << ADC_CR2_EXTSEL_Pos)                 /*!< 0x0F000000 */
#define ADC_CR2_EXTSEL            ADC_CR2_EXTSEL_Msk                           /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
#define ADC_CR2_EXTSEL_0          (0x1U << ADC_CR2_EXTSEL_Pos)                 /*!< 0x01000000 */
#define ADC_CR2_EXTSEL_1          (0x2U << ADC_CR2_EXTSEL_Pos)                 /*!< 0x02000000 */
#define ADC_CR2_EXTSEL_2          (0x4U << ADC_CR2_EXTSEL_Pos)                 /*!< 0x04000000 */
#define ADC_CR2_EXTSEL_3          (0x8U << ADC_CR2_EXTSEL_Pos)                 /*!< 0x08000000 */
#define ADC_CR2_EXTEN_Pos         (28U)                                        
#define ADC_CR2_EXTEN_Msk         (0x3U << ADC_CR2_EXTEN_Pos)                  /*!< 0x30000000 */
#define ADC_CR2_EXTEN             ADC_CR2_EXTEN_Msk                            /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
#define ADC_CR2_EXTEN_0           (0x1U << ADC_CR2_EXTEN_Pos)                  /*!< 0x10000000 */
#define ADC_CR2_EXTEN_1           (0x2U << ADC_CR2_EXTEN_Pos)                  /*!< 0x20000000 */
#define ADC_CR2_SWSTART_Pos       (30U)                                        
#define ADC_CR2_SWSTART_Msk       (0x1U << ADC_CR2_SWSTART_Pos)                /*!< 0x40000000 */
#define ADC_CR2_SWSTART           ADC_CR2_SWSTART_Msk                          /*!<Start Conversion of regular channels */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define ADC_SMPR1_SMP10_Pos       (0U)                                         
#define ADC_SMPR1_SMP10_Msk       (0x7U << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000007 */
#define ADC_SMPR1_SMP10           ADC_SMPR1_SMP10_Msk                          /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
#define ADC_SMPR1_SMP10_0         (0x1U << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000001 */
#define ADC_SMPR1_SMP10_1         (0x2U << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000002 */
#define ADC_SMPR1_SMP10_2         (0x4U << ADC_SMPR1_SMP10_Pos)                /*!< 0x00000004 */
#define ADC_SMPR1_SMP11_Pos       (3U)                                         
#define ADC_SMPR1_SMP11_Msk       (0x7U << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000038 */
#define ADC_SMPR1_SMP11           ADC_SMPR1_SMP11_Msk                          /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
#define ADC_SMPR1_SMP11_0         (0x1U << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000008 */
#define ADC_SMPR1_SMP11_1         (0x2U << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000010 */
#define ADC_SMPR1_SMP11_2         (0x4U << ADC_SMPR1_SMP11_Pos)                /*!< 0x00000020 */
#define ADC_SMPR1_SMP12_Pos       (6U)                                         
#define ADC_SMPR1_SMP12_Msk       (0x7U << ADC_SMPR1_SMP12_Pos)                /*!< 0x000001C0 */
#define ADC_SMPR1_SMP12           ADC_SMPR1_SMP12_Msk                          /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
#define ADC_SMPR1_SMP12_0         (0x1U << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000040 */
#define ADC_SMPR1_SMP12_1         (0x2U << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000080 */
#define ADC_SMPR1_SMP12_2         (0x4U << ADC_SMPR1_SMP12_Pos)                /*!< 0x00000100 */
#define ADC_SMPR1_SMP13_Pos       (9U)                                         
#define ADC_SMPR1_SMP13_Msk       (0x7U << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000E00 */
#define ADC_SMPR1_SMP13           ADC_SMPR1_SMP13_Msk                          /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
#define ADC_SMPR1_SMP13_0         (0x1U << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000200 */
#define ADC_SMPR1_SMP13_1         (0x2U << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000400 */
#define ADC_SMPR1_SMP13_2         (0x4U << ADC_SMPR1_SMP13_Pos)                /*!< 0x00000800 */
#define ADC_SMPR1_SMP14_Pos       (12U)                                        
#define ADC_SMPR1_SMP14_Msk       (0x7U << ADC_SMPR1_SMP14_Pos)                /*!< 0x00007000 */
#define ADC_SMPR1_SMP14           ADC_SMPR1_SMP14_Msk                          /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
#define ADC_SMPR1_SMP14_0         (0x1U << ADC_SMPR1_SMP14_Pos)                /*!< 0x00001000 */
#define ADC_SMPR1_SMP14_1         (0x2U << ADC_SMPR1_SMP14_Pos)                /*!< 0x00002000 */
#define ADC_SMPR1_SMP14_2         (0x4U << ADC_SMPR1_SMP14_Pos)                /*!< 0x00004000 */
#define ADC_SMPR1_SMP15_Pos       (15U)                                        
#define ADC_SMPR1_SMP15_Msk       (0x7U << ADC_SMPR1_SMP15_Pos)                /*!< 0x00038000 */
#define ADC_SMPR1_SMP15           ADC_SMPR1_SMP15_Msk                          /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
#define ADC_SMPR1_SMP15_0         (0x1U << ADC_SMPR1_SMP15_Pos)                /*!< 0x00008000 */
#define ADC_SMPR1_SMP15_1         (0x2U << ADC_SMPR1_SMP15_Pos)                /*!< 0x00010000 */
#define ADC_SMPR1_SMP15_2         (0x4U << ADC_SMPR1_SMP15_Pos)                /*!< 0x00020000 */
#define ADC_SMPR1_SMP16_Pos       (18U)                                        
#define ADC_SMPR1_SMP16_Msk       (0x7U << ADC_SMPR1_SMP16_Pos)                /*!< 0x001C0000 */
#define ADC_SMPR1_SMP16           ADC_SMPR1_SMP16_Msk                          /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
#define ADC_SMPR1_SMP16_0         (0x1U << ADC_SMPR1_SMP16_Pos)                /*!< 0x00040000 */
#define ADC_SMPR1_SMP16_1         (0x2U << ADC_SMPR1_SMP16_Pos)                /*!< 0x00080000 */
#define ADC_SMPR1_SMP16_2         (0x4U << ADC_SMPR1_SMP16_Pos)                /*!< 0x00100000 */
#define ADC_SMPR1_SMP17_Pos       (21U)                                        
#define ADC_SMPR1_SMP17_Msk       (0x7U << ADC_SMPR1_SMP17_Pos)                /*!< 0x00E00000 */
#define ADC_SMPR1_SMP17           ADC_SMPR1_SMP17_Msk                          /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
#define ADC_SMPR1_SMP17_0         (0x1U << ADC_SMPR1_SMP17_Pos)                /*!< 0x00200000 */
#define ADC_SMPR1_SMP17_1         (0x2U << ADC_SMPR1_SMP17_Pos)                /*!< 0x00400000 */
#define ADC_SMPR1_SMP17_2         (0x4U << ADC_SMPR1_SMP17_Pos)                /*!< 0x00800000 */
#define ADC_SMPR1_SMP18_Pos       (24U)                                        
#define ADC_SMPR1_SMP18_Msk       (0x7U << ADC_SMPR1_SMP18_Pos)                /*!< 0x07000000 */
#define ADC_SMPR1_SMP18           ADC_SMPR1_SMP18_Msk                          /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
#define ADC_SMPR1_SMP18_0         (0x1U << ADC_SMPR1_SMP18_Pos)                /*!< 0x01000000 */
#define ADC_SMPR1_SMP18_1         (0x2U << ADC_SMPR1_SMP18_Pos)                /*!< 0x02000000 */
#define ADC_SMPR1_SMP18_2         (0x4U << ADC_SMPR1_SMP18_Pos)                /*!< 0x04000000 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define ADC_SMPR2_SMP0_Pos        (0U)                                         
#define ADC_SMPR2_SMP0_Msk        (0x7U << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000007 */
#define ADC_SMPR2_SMP0            ADC_SMPR2_SMP0_Msk                           /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
#define ADC_SMPR2_SMP0_0          (0x1U << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000001 */
#define ADC_SMPR2_SMP0_1          (0x2U << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000002 */
#define ADC_SMPR2_SMP0_2          (0x4U << ADC_SMPR2_SMP0_Pos)                 /*!< 0x00000004 */
#define ADC_SMPR2_SMP1_Pos        (3U)                                         
#define ADC_SMPR2_SMP1_Msk        (0x7U << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000038 */
#define ADC_SMPR2_SMP1            ADC_SMPR2_SMP1_Msk                           /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
#define ADC_SMPR2_SMP1_0          (0x1U << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000008 */
#define ADC_SMPR2_SMP1_1          (0x2U << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000010 */
#define ADC_SMPR2_SMP1_2          (0x4U << ADC_SMPR2_SMP1_Pos)                 /*!< 0x00000020 */
#define ADC_SMPR2_SMP2_Pos        (6U)                                         
#define ADC_SMPR2_SMP2_Msk        (0x7U << ADC_SMPR2_SMP2_Pos)                 /*!< 0x000001C0 */
#define ADC_SMPR2_SMP2            ADC_SMPR2_SMP2_Msk                           /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
#define ADC_SMPR2_SMP2_0          (0x1U << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000040 */
#define ADC_SMPR2_SMP2_1          (0x2U << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000080 */
#define ADC_SMPR2_SMP2_2          (0x4U << ADC_SMPR2_SMP2_Pos)                 /*!< 0x00000100 */
#define ADC_SMPR2_SMP3_Pos        (9U)                                         
#define ADC_SMPR2_SMP3_Msk        (0x7U << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000E00 */
#define ADC_SMPR2_SMP3            ADC_SMPR2_SMP3_Msk                           /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
#define ADC_SMPR2_SMP3_0          (0x1U << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000200 */
#define ADC_SMPR2_SMP3_1          (0x2U << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000400 */
#define ADC_SMPR2_SMP3_2          (0x4U << ADC_SMPR2_SMP3_Pos)                 /*!< 0x00000800 */
#define ADC_SMPR2_SMP4_Pos        (12U)                                        
#define ADC_SMPR2_SMP4_Msk        (0x7U << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00007000 */
#define ADC_SMPR2_SMP4            ADC_SMPR2_SMP4_Msk                           /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
#define ADC_SMPR2_SMP4_0          (0x1U << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00001000 */
#define ADC_SMPR2_SMP4_1          (0x2U << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00002000 */
#define ADC_SMPR2_SMP4_2          (0x4U << ADC_SMPR2_SMP4_Pos)                 /*!< 0x00004000 */
#define ADC_SMPR2_SMP5_Pos        (15U)                                        
#define ADC_SMPR2_SMP5_Msk        (0x7U << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00038000 */
#define ADC_SMPR2_SMP5            ADC_SMPR2_SMP5_Msk                           /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
#define ADC_SMPR2_SMP5_0          (0x1U << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00008000 */
#define ADC_SMPR2_SMP5_1          (0x2U << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00010000 */
#define ADC_SMPR2_SMP5_2          (0x4U << ADC_SMPR2_SMP5_Pos)                 /*!< 0x00020000 */
#define ADC_SMPR2_SMP6_Pos        (18U)                                        
#define ADC_SMPR2_SMP6_Msk        (0x7U << ADC_SMPR2_SMP6_Pos)                 /*!< 0x001C0000 */
#define ADC_SMPR2_SMP6            ADC_SMPR2_SMP6_Msk                           /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
#define ADC_SMPR2_SMP6_0          (0x1U << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00040000 */
#define ADC_SMPR2_SMP6_1          (0x2U << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00080000 */
#define ADC_SMPR2_SMP6_2          (0x4U << ADC_SMPR2_SMP6_Pos)                 /*!< 0x00100000 */
#define ADC_SMPR2_SMP7_Pos        (21U)                                        
#define ADC_SMPR2_SMP7_Msk        (0x7U << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00E00000 */
#define ADC_SMPR2_SMP7            ADC_SMPR2_SMP7_Msk                           /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
#define ADC_SMPR2_SMP7_0          (0x1U << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00200000 */
#define ADC_SMPR2_SMP7_1          (0x2U << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00400000 */
#define ADC_SMPR2_SMP7_2          (0x4U << ADC_SMPR2_SMP7_Pos)                 /*!< 0x00800000 */
#define ADC_SMPR2_SMP8_Pos        (24U)                                        
#define ADC_SMPR2_SMP8_Msk        (0x7U << ADC_SMPR2_SMP8_Pos)                 /*!< 0x07000000 */
#define ADC_SMPR2_SMP8            ADC_SMPR2_SMP8_Msk                           /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
#define ADC_SMPR2_SMP8_0          (0x1U << ADC_SMPR2_SMP8_Pos)                 /*!< 0x01000000 */
#define ADC_SMPR2_SMP8_1          (0x2U << ADC_SMPR2_SMP8_Pos)                 /*!< 0x02000000 */
#define ADC_SMPR2_SMP8_2          (0x4U << ADC_SMPR2_SMP8_Pos)                 /*!< 0x04000000 */
#define ADC_SMPR2_SMP9_Pos        (27U)                                        
#define ADC_SMPR2_SMP9_Msk        (0x7U << ADC_SMPR2_SMP9_Pos)                 /*!< 0x38000000 */
#define ADC_SMPR2_SMP9            ADC_SMPR2_SMP9_Msk                           /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
#define ADC_SMPR2_SMP9_0          (0x1U << ADC_SMPR2_SMP9_Pos)                 /*!< 0x08000000 */
#define ADC_SMPR2_SMP9_1          (0x2U << ADC_SMPR2_SMP9_Pos)                 /*!< 0x10000000 */
#define ADC_SMPR2_SMP9_2          (0x4U << ADC_SMPR2_SMP9_Pos)                 /*!< 0x20000000 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define ADC_JOFR1_JOFFSET1_Pos    (0U)                                         
#define ADC_JOFR1_JOFFSET1_Msk    (0xFFFU << ADC_JOFR1_JOFFSET1_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR1_JOFFSET1        ADC_JOFR1_JOFFSET1_Msk                       /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define ADC_JOFR2_JOFFSET2_Pos    (0U)                                         
#define ADC_JOFR2_JOFFSET2_Msk    (0xFFFU << ADC_JOFR2_JOFFSET2_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR2_JOFFSET2        ADC_JOFR2_JOFFSET2_Msk                       /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define ADC_JOFR3_JOFFSET3_Pos    (0U)                                         
#define ADC_JOFR3_JOFFSET3_Msk    (0xFFFU << ADC_JOFR3_JOFFSET3_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR3_JOFFSET3        ADC_JOFR3_JOFFSET3_Msk                       /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define ADC_JOFR4_JOFFSET4_Pos    (0U)                                         
#define ADC_JOFR4_JOFFSET4_Msk    (0xFFFU << ADC_JOFR4_JOFFSET4_Pos)           /*!< 0x00000FFF */
#define ADC_JOFR4_JOFFSET4        ADC_JOFR4_JOFFSET4_Msk                       /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define ADC_HTR_HT_Pos            (0U)                                         
#define ADC_HTR_HT_Msk            (0xFFFU << ADC_HTR_HT_Pos)                   /*!< 0x00000FFF */
#define ADC_HTR_HT                ADC_HTR_HT_Msk                               /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define ADC_LTR_LT_Pos            (0U)                                         
#define ADC_LTR_LT_Msk            (0xFFFU << ADC_LTR_LT_Pos)                   /*!< 0x00000FFF */
#define ADC_LTR_LT                ADC_LTR_LT_Msk                               /*!<Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define ADC_SQR1_SQ13_Pos         (0U)                                         
#define ADC_SQR1_SQ13_Msk         (0x1FU << ADC_SQR1_SQ13_Pos)                 /*!< 0x0000001F */
#define ADC_SQR1_SQ13             ADC_SQR1_SQ13_Msk                            /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
#define ADC_SQR1_SQ13_0           (0x01U << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000001 */
#define ADC_SQR1_SQ13_1           (0x02U << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000002 */
#define ADC_SQR1_SQ13_2           (0x04U << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000004 */
#define ADC_SQR1_SQ13_3           (0x08U << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000008 */
#define ADC_SQR1_SQ13_4           (0x10U << ADC_SQR1_SQ13_Pos)                 /*!< 0x00000010 */
#define ADC_SQR1_SQ14_Pos         (5U)                                         
#define ADC_SQR1_SQ14_Msk         (0x1FU << ADC_SQR1_SQ14_Pos)                 /*!< 0x000003E0 */
#define ADC_SQR1_SQ14             ADC_SQR1_SQ14_Msk                            /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
#define ADC_SQR1_SQ14_0           (0x01U << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000020 */
#define ADC_SQR1_SQ14_1           (0x02U << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000040 */
#define ADC_SQR1_SQ14_2           (0x04U << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000080 */
#define ADC_SQR1_SQ14_3           (0x08U << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000100 */
#define ADC_SQR1_SQ14_4           (0x10U << ADC_SQR1_SQ14_Pos)                 /*!< 0x00000200 */
#define ADC_SQR1_SQ15_Pos         (10U)                                        
#define ADC_SQR1_SQ15_Msk         (0x1FU << ADC_SQR1_SQ15_Pos)                 /*!< 0x00007C00 */
#define ADC_SQR1_SQ15             ADC_SQR1_SQ15_Msk                            /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
#define ADC_SQR1_SQ15_0           (0x01U << ADC_SQR1_SQ15_Pos)                 /*!< 0x00000400 */
#define ADC_SQR1_SQ15_1           (0x02U << ADC_SQR1_SQ15_Pos)                 /*!< 0x00000800 */
#define ADC_SQR1_SQ15_2           (0x04U << ADC_SQR1_SQ15_Pos)                 /*!< 0x00001000 */
#define ADC_SQR1_SQ15_3           (0x08U << ADC_SQR1_SQ15_Pos)                 /*!< 0x00002000 */
#define ADC_SQR1_SQ15_4           (0x10U << ADC_SQR1_SQ15_Pos)                 /*!< 0x00004000 */
#define ADC_SQR1_SQ16_Pos         (15U)                                        
#define ADC_SQR1_SQ16_Msk         (0x1FU << ADC_SQR1_SQ16_Pos)                 /*!< 0x000F8000 */
#define ADC_SQR1_SQ16             ADC_SQR1_SQ16_Msk                            /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
#define ADC_SQR1_SQ16_0           (0x01U << ADC_SQR1_SQ16_Pos)                 /*!< 0x00008000 */
#define ADC_SQR1_SQ16_1           (0x02U << ADC_SQR1_SQ16_Pos)                 /*!< 0x00010000 */
#define ADC_SQR1_SQ16_2           (0x04U << ADC_SQR1_SQ16_Pos)                 /*!< 0x00020000 */
#define ADC_SQR1_SQ16_3           (0x08U << ADC_SQR1_SQ16_Pos)                 /*!< 0x00040000 */
#define ADC_SQR1_SQ16_4           (0x10U << ADC_SQR1_SQ16_Pos)                 /*!< 0x00080000 */
#define ADC_SQR1_L_Pos            (20U)                                        
#define ADC_SQR1_L_Msk            (0xFU << ADC_SQR1_L_Pos)                     /*!< 0x00F00000 */
#define ADC_SQR1_L                ADC_SQR1_L_Msk                               /*!<L[3:0] bits (Regular channel sequence length) */
#define ADC_SQR1_L_0              (0x1U << ADC_SQR1_L_Pos)                     /*!< 0x00100000 */
#define ADC_SQR1_L_1              (0x2U << ADC_SQR1_L_Pos)                     /*!< 0x00200000 */
#define ADC_SQR1_L_2              (0x4U << ADC_SQR1_L_Pos)                     /*!< 0x00400000 */
#define ADC_SQR1_L_3              (0x8U << ADC_SQR1_L_Pos)                     /*!< 0x00800000 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define ADC_SQR2_SQ7_Pos          (0U)                                         
#define ADC_SQR2_SQ7_Msk          (0x1FU << ADC_SQR2_SQ7_Pos)                  /*!< 0x0000001F */
#define ADC_SQR2_SQ7              ADC_SQR2_SQ7_Msk                             /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
#define ADC_SQR2_SQ7_0            (0x01U << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000001 */
#define ADC_SQR2_SQ7_1            (0x02U << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000002 */
#define ADC_SQR2_SQ7_2            (0x04U << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000004 */
#define ADC_SQR2_SQ7_3            (0x08U << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000008 */
#define ADC_SQR2_SQ7_4            (0x10U << ADC_SQR2_SQ7_Pos)                  /*!< 0x00000010 */
#define ADC_SQR2_SQ8_Pos          (5U)                                         
#define ADC_SQR2_SQ8_Msk          (0x1FU << ADC_SQR2_SQ8_Pos)                  /*!< 0x000003E0 */
#define ADC_SQR2_SQ8              ADC_SQR2_SQ8_Msk                             /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
#define ADC_SQR2_SQ8_0            (0x01U << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000020 */
#define ADC_SQR2_SQ8_1            (0x02U << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000040 */
#define ADC_SQR2_SQ8_2            (0x04U << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000080 */
#define ADC_SQR2_SQ8_3            (0x08U << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000100 */
#define ADC_SQR2_SQ8_4            (0x10U << ADC_SQR2_SQ8_Pos)                  /*!< 0x00000200 */
#define ADC_SQR2_SQ9_Pos          (10U)                                        
#define ADC_SQR2_SQ9_Msk          (0x1FU << ADC_SQR2_SQ9_Pos)                  /*!< 0x00007C00 */
#define ADC_SQR2_SQ9              ADC_SQR2_SQ9_Msk                             /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
#define ADC_SQR2_SQ9_0            (0x01U << ADC_SQR2_SQ9_Pos)                  /*!< 0x00000400 */
#define ADC_SQR2_SQ9_1            (0x02U << ADC_SQR2_SQ9_Pos)                  /*!< 0x00000800 */
#define ADC_SQR2_SQ9_2            (0x04U << ADC_SQR2_SQ9_Pos)                  /*!< 0x00001000 */
#define ADC_SQR2_SQ9_3            (0x08U << ADC_SQR2_SQ9_Pos)                  /*!< 0x00002000 */
#define ADC_SQR2_SQ9_4            (0x10U << ADC_SQR2_SQ9_Pos)                  /*!< 0x00004000 */
#define ADC_SQR2_SQ10_Pos         (15U)                                        
#define ADC_SQR2_SQ10_Msk         (0x1FU << ADC_SQR2_SQ10_Pos)                 /*!< 0x000F8000 */
#define ADC_SQR2_SQ10             ADC_SQR2_SQ10_Msk                            /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
#define ADC_SQR2_SQ10_0           (0x01U << ADC_SQR2_SQ10_Pos)                 /*!< 0x00008000 */
#define ADC_SQR2_SQ10_1           (0x02U << ADC_SQR2_SQ10_Pos)                 /*!< 0x00010000 */
#define ADC_SQR2_SQ10_2           (0x04U << ADC_SQR2_SQ10_Pos)                 /*!< 0x00020000 */
#define ADC_SQR2_SQ10_3           (0x08U << ADC_SQR2_SQ10_Pos)                 /*!< 0x00040000 */
#define ADC_SQR2_SQ10_4           (0x10U << ADC_SQR2_SQ10_Pos)                 /*!< 0x00080000 */
#define ADC_SQR2_SQ11_Pos         (20U)                                        
#define ADC_SQR2_SQ11_Msk         (0x1FU << ADC_SQR2_SQ11_Pos)                 /*!< 0x01F00000 */
#define ADC_SQR2_SQ11             ADC_SQR2_SQ11_Msk                            /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
#define ADC_SQR2_SQ11_0           (0x01U << ADC_SQR2_SQ11_Pos)                 /*!< 0x00100000 */
#define ADC_SQR2_SQ11_1           (0x02U << ADC_SQR2_SQ11_Pos)                 /*!< 0x00200000 */
#define ADC_SQR2_SQ11_2           (0x04U << ADC_SQR2_SQ11_Pos)                 /*!< 0x00400000 */
#define ADC_SQR2_SQ11_3           (0x08U << ADC_SQR2_SQ11_Pos)                 /*!< 0x00800000 */
#define ADC_SQR2_SQ11_4           (0x10U << ADC_SQR2_SQ11_Pos)                 /*!< 0x01000000 */
#define ADC_SQR2_SQ12_Pos         (25U)                                        
#define ADC_SQR2_SQ12_Msk         (0x1FU << ADC_SQR2_SQ12_Pos)                 /*!< 0x3E000000 */
#define ADC_SQR2_SQ12             ADC_SQR2_SQ12_Msk                            /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
#define ADC_SQR2_SQ12_0           (0x01U << ADC_SQR2_SQ12_Pos)                 /*!< 0x02000000 */
#define ADC_SQR2_SQ12_1           (0x02U << ADC_SQR2_SQ12_Pos)                 /*!< 0x04000000 */
#define ADC_SQR2_SQ12_2           (0x04U << ADC_SQR2_SQ12_Pos)                 /*!< 0x08000000 */
#define ADC_SQR2_SQ12_3           (0x08U << ADC_SQR2_SQ12_Pos)                 /*!< 0x10000000 */
#define ADC_SQR2_SQ12_4           (0x10U << ADC_SQR2_SQ12_Pos)                 /*!< 0x20000000 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define ADC_SQR3_SQ1_Pos          (0U)                                         
#define ADC_SQR3_SQ1_Msk          (0x1FU << ADC_SQR3_SQ1_Pos)                  /*!< 0x0000001F */
#define ADC_SQR3_SQ1              ADC_SQR3_SQ1_Msk                             /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
#define ADC_SQR3_SQ1_0            (0x01U << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000001 */
#define ADC_SQR3_SQ1_1            (0x02U << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000002 */
#define ADC_SQR3_SQ1_2            (0x04U << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000004 */
#define ADC_SQR3_SQ1_3            (0x08U << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000008 */
#define ADC_SQR3_SQ1_4            (0x10U << ADC_SQR3_SQ1_Pos)                  /*!< 0x00000010 */
#define ADC_SQR3_SQ2_Pos          (5U)                                         
#define ADC_SQR3_SQ2_Msk          (0x1FU << ADC_SQR3_SQ2_Pos)                  /*!< 0x000003E0 */
#define ADC_SQR3_SQ2              ADC_SQR3_SQ2_Msk                             /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
#define ADC_SQR3_SQ2_0            (0x01U << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000020 */
#define ADC_SQR3_SQ2_1            (0x02U << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000040 */
#define ADC_SQR3_SQ2_2            (0x04U << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000080 */
#define ADC_SQR3_SQ2_3            (0x08U << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000100 */
#define ADC_SQR3_SQ2_4            (0x10U << ADC_SQR3_SQ2_Pos)                  /*!< 0x00000200 */
#define ADC_SQR3_SQ3_Pos          (10U)                                        
#define ADC_SQR3_SQ3_Msk          (0x1FU << ADC_SQR3_SQ3_Pos)                  /*!< 0x00007C00 */
#define ADC_SQR3_SQ3              ADC_SQR3_SQ3_Msk                             /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
#define ADC_SQR3_SQ3_0            (0x01U << ADC_SQR3_SQ3_Pos)                  /*!< 0x00000400 */
#define ADC_SQR3_SQ3_1            (0x02U << ADC_SQR3_SQ3_Pos)                  /*!< 0x00000800 */
#define ADC_SQR3_SQ3_2            (0x04U << ADC_SQR3_SQ3_Pos)                  /*!< 0x00001000 */
#define ADC_SQR3_SQ3_3            (0x08U << ADC_SQR3_SQ3_Pos)                  /*!< 0x00002000 */
#define ADC_SQR3_SQ3_4            (0x10U << ADC_SQR3_SQ3_Pos)                  /*!< 0x00004000 */
#define ADC_SQR3_SQ4_Pos          (15U)                                        
#define ADC_SQR3_SQ4_Msk          (0x1FU << ADC_SQR3_SQ4_Pos)                  /*!< 0x000F8000 */
#define ADC_SQR3_SQ4              ADC_SQR3_SQ4_Msk                             /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
#define ADC_SQR3_SQ4_0            (0x01U << ADC_SQR3_SQ4_Pos)                  /*!< 0x00008000 */
#define ADC_SQR3_SQ4_1            (0x02U << ADC_SQR3_SQ4_Pos)                  /*!< 0x00010000 */
#define ADC_SQR3_SQ4_2            (0x04U << ADC_SQR3_SQ4_Pos)                  /*!< 0x00020000 */
#define ADC_SQR3_SQ4_3            (0x08U << ADC_SQR3_SQ4_Pos)                  /*!< 0x00040000 */
#define ADC_SQR3_SQ4_4            (0x10U << ADC_SQR3_SQ4_Pos)                  /*!< 0x00080000 */
#define ADC_SQR3_SQ5_Pos          (20U)                                        
#define ADC_SQR3_SQ5_Msk          (0x1FU << ADC_SQR3_SQ5_Pos)                  /*!< 0x01F00000 */
#define ADC_SQR3_SQ5              ADC_SQR3_SQ5_Msk                             /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
#define ADC_SQR3_SQ5_0            (0x01U << ADC_SQR3_SQ5_Pos)                  /*!< 0x00100000 */
#define ADC_SQR3_SQ5_1            (0x02U << ADC_SQR3_SQ5_Pos)                  /*!< 0x00200000 */
#define ADC_SQR3_SQ5_2            (0x04U << ADC_SQR3_SQ5_Pos)                  /*!< 0x00400000 */
#define ADC_SQR3_SQ5_3            (0x08U << ADC_SQR3_SQ5_Pos)                  /*!< 0x00800000 */
#define ADC_SQR3_SQ5_4            (0x10U << ADC_SQR3_SQ5_Pos)                  /*!< 0x01000000 */
#define ADC_SQR3_SQ6_Pos          (25U)                                        
#define ADC_SQR3_SQ6_Msk          (0x1FU << ADC_SQR3_SQ6_Pos)                  /*!< 0x3E000000 */
#define ADC_SQR3_SQ6              ADC_SQR3_SQ6_Msk                             /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
#define ADC_SQR3_SQ6_0            (0x01U << ADC_SQR3_SQ6_Pos)                  /*!< 0x02000000 */
#define ADC_SQR3_SQ6_1            (0x02U << ADC_SQR3_SQ6_Pos)                  /*!< 0x04000000 */
#define ADC_SQR3_SQ6_2            (0x04U << ADC_SQR3_SQ6_Pos)                  /*!< 0x08000000 */
#define ADC_SQR3_SQ6_3            (0x08U << ADC_SQR3_SQ6_Pos)                  /*!< 0x10000000 */
#define ADC_SQR3_SQ6_4            (0x10U << ADC_SQR3_SQ6_Pos)                  /*!< 0x20000000 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define ADC_JSQR_JSQ1_Pos         (0U)                                         
#define ADC_JSQR_JSQ1_Msk         (0x1FU << ADC_JSQR_JSQ1_Pos)                 /*!< 0x0000001F */
#define ADC_JSQR_JSQ1             ADC_JSQR_JSQ1_Msk                            /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define ADC_JSQR_JSQ1_0           (0x01U << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000001 */
#define ADC_JSQR_JSQ1_1           (0x02U << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000002 */
#define ADC_JSQR_JSQ1_2           (0x04U << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000004 */
#define ADC_JSQR_JSQ1_3           (0x08U << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000008 */
#define ADC_JSQR_JSQ1_4           (0x10U << ADC_JSQR_JSQ1_Pos)                 /*!< 0x00000010 */
#define ADC_JSQR_JSQ2_Pos         (5U)                                         
#define ADC_JSQR_JSQ2_Msk         (0x1FU << ADC_JSQR_JSQ2_Pos)                 /*!< 0x000003E0 */
#define ADC_JSQR_JSQ2             ADC_JSQR_JSQ2_Msk                            /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define ADC_JSQR_JSQ2_0           (0x01U << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000020 */
#define ADC_JSQR_JSQ2_1           (0x02U << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000040 */
#define ADC_JSQR_JSQ2_2           (0x04U << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000080 */
#define ADC_JSQR_JSQ2_3           (0x08U << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000100 */
#define ADC_JSQR_JSQ2_4           (0x10U << ADC_JSQR_JSQ2_Pos)                 /*!< 0x00000200 */
#define ADC_JSQR_JSQ3_Pos         (10U)                                        
#define ADC_JSQR_JSQ3_Msk         (0x1FU << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00007C00 */
#define ADC_JSQR_JSQ3             ADC_JSQR_JSQ3_Msk                            /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define ADC_JSQR_JSQ3_0           (0x01U << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00000400 */
#define ADC_JSQR_JSQ3_1           (0x02U << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00000800 */
#define ADC_JSQR_JSQ3_2           (0x04U << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00001000 */
#define ADC_JSQR_JSQ3_3           (0x08U << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00002000 */
#define ADC_JSQR_JSQ3_4           (0x10U << ADC_JSQR_JSQ3_Pos)                 /*!< 0x00004000 */
#define ADC_JSQR_JSQ4_Pos         (15U)                                        
#define ADC_JSQR_JSQ4_Msk         (0x1FU << ADC_JSQR_JSQ4_Pos)                 /*!< 0x000F8000 */
#define ADC_JSQR_JSQ4             ADC_JSQR_JSQ4_Msk                            /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
#define ADC_JSQR_JSQ4_0           (0x01U << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00008000 */
#define ADC_JSQR_JSQ4_1           (0x02U << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00010000 */
#define ADC_JSQR_JSQ4_2           (0x04U << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00020000 */
#define ADC_JSQR_JSQ4_3           (0x08U << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00040000 */
#define ADC_JSQR_JSQ4_4           (0x10U << ADC_JSQR_JSQ4_Pos)                 /*!< 0x00080000 */
#define ADC_JSQR_JL_Pos           (20U)                                        
#define ADC_JSQR_JL_Msk           (0x3U << ADC_JSQR_JL_Pos)                    /*!< 0x00300000 */
#define ADC_JSQR_JL               ADC_JSQR_JL_Msk                              /*!<JL[1:0] bits (Injected Sequence length) */
#define ADC_JSQR_JL_0             (0x1U << ADC_JSQR_JL_Pos)                    /*!< 0x00100000 */
#define ADC_JSQR_JL_1             (0x2U << ADC_JSQR_JL_Pos)                    /*!< 0x00200000 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define ADC_JDR1_JDATA_Pos        (0U)                                         
#define ADC_JDR1_JDATA_Msk        (0xFFFFU << ADC_JDR1_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR1_JDATA            ADC_JDR1_JDATA_Msk                           /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define ADC_JDR2_JDATA_Pos        (0U)                                         
#define ADC_JDR2_JDATA_Msk        (0xFFFFU << ADC_JDR2_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR2_JDATA            ADC_JDR2_JDATA_Msk                           /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define ADC_JDR3_JDATA_Pos        (0U)                                         
#define ADC_JDR3_JDATA_Msk        (0xFFFFU << ADC_JDR3_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR3_JDATA            ADC_JDR3_JDATA_Msk                           /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define ADC_JDR4_JDATA_Pos        (0U)                                         
#define ADC_JDR4_JDATA_Msk        (0xFFFFU << ADC_JDR4_JDATA_Pos)              /*!< 0x0000FFFF */
#define ADC_JDR4_JDATA            ADC_JDR4_JDATA_Msk                           /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos           (0U)                                         
#define ADC_DR_DATA_Msk           (0xFFFFU << ADC_DR_DATA_Pos)                 /*!< 0x0000FFFF */
#define ADC_DR_DATA               ADC_DR_DATA_Msk                              /*!<Regular data */
#define ADC_DR_ADC2DATA_Pos       (16U)                                        
#define ADC_DR_ADC2DATA_Msk       (0xFFFFU << ADC_DR_ADC2DATA_Pos)             /*!< 0xFFFF0000 */
#define ADC_DR_ADC2DATA           ADC_DR_ADC2DATA_Msk                          /*!<ADC2 data */

/*******************  Bit definition for ADC_CSR register  ********************/
#define ADC_CSR_AWD1_Pos          (0U)                                         
#define ADC_CSR_AWD1_Msk          (0x1U << ADC_CSR_AWD1_Pos)                   /*!< 0x00000001 */
#define ADC_CSR_AWD1              ADC_CSR_AWD1_Msk                             /*!<ADC1 Analog watchdog flag */
#define ADC_CSR_EOC1_Pos          (1U)                                         
#define ADC_CSR_EOC1_Msk          (0x1U << ADC_CSR_EOC1_Pos)                   /*!< 0x00000002 */
#define ADC_CSR_EOC1              ADC_CSR_EOC1_Msk                             /*!<ADC1 End of conversion */
#define ADC_CSR_JEOC1_Pos         (2U)                                         
#define ADC_CSR_JEOC1_Msk         (0x1U << ADC_CSR_JEOC1_Pos)                  /*!< 0x00000004 */
#define ADC_CSR_JEOC1             ADC_CSR_JEOC1_Msk                            /*!<ADC1 Injected channel end of conversion */
#define ADC_CSR_JSTRT1_Pos        (3U)                                         
#define ADC_CSR_JSTRT1_Msk        (0x1U << ADC_CSR_JSTRT1_Pos)                 /*!< 0x00000008 */
#define ADC_CSR_JSTRT1            ADC_CSR_JSTRT1_Msk                           /*!<ADC1 Injected channel Start flag */
#define ADC_CSR_STRT1_Pos         (4U)                                         
#define ADC_CSR_STRT1_Msk         (0x1U << ADC_CSR_STRT1_Pos)                  /*!< 0x00000010 */
#define ADC_CSR_STRT1             ADC_CSR_STRT1_Msk                            /*!<ADC1 Regular channel Start flag */
#define ADC_CSR_OVR1_Pos          (5U)                                         
#define ADC_CSR_OVR1_Msk          (0x1U << ADC_CSR_OVR1_Pos)                   /*!< 0x00000020 */
#define ADC_CSR_OVR1              ADC_CSR_OVR1_Msk                             /*!<ADC1 DMA overrun  flag */

/* Legacy defines */
#define  ADC_CSR_DOVR1                        ADC_CSR_OVR1

/*******************  Bit definition for ADC_CCR register  ********************/
#define ADC_CCR_MULTI_Pos         (0U)                                         
#define ADC_CCR_MULTI_Msk         (0x1FU << ADC_CCR_MULTI_Pos)                 /*!< 0x0000001F */
#define ADC_CCR_MULTI             ADC_CCR_MULTI_Msk                            /*!<MULTI[4:0] bits (Multi-ADC mode selection) */  
#define ADC_CCR_MULTI_0           (0x01U << ADC_CCR_MULTI_Pos)                 /*!< 0x00000001 */
#define ADC_CCR_MULTI_1           (0x02U << ADC_CCR_MULTI_Pos)                 /*!< 0x00000002 */
#define ADC_CCR_MULTI_2           (0x04U << ADC_CCR_MULTI_Pos)                 /*!< 0x00000004 */
#define ADC_CCR_MULTI_3           (0x08U << ADC_CCR_MULTI_Pos)                 /*!< 0x00000008 */
#define ADC_CCR_MULTI_4           (0x10U << ADC_CCR_MULTI_Pos)                 /*!< 0x00000010 */
#define ADC_CCR_DELAY_Pos         (8U)                                         
#define ADC_CCR_DELAY_Msk         (0xFU << ADC_CCR_DELAY_Pos)                  /*!< 0x00000F00 */
#define ADC_CCR_DELAY             ADC_CCR_DELAY_Msk                            /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */  
#define ADC_CCR_DELAY_0           (0x1U << ADC_CCR_DELAY_Pos)                  /*!< 0x00000100 */
#define ADC_CCR_DELAY_1           (0x2U << ADC_CCR_DELAY_Pos)                  /*!< 0x00000200 */
#define ADC_CCR_DELAY_2           (0x4U << ADC_CCR_DELAY_Pos)                  /*!< 0x00000400 */
#define ADC_CCR_DELAY_3           (0x8U << ADC_CCR_DELAY_Pos)                  /*!< 0x00000800 */
#define ADC_CCR_DDS_Pos           (13U)                                        
#define ADC_CCR_DDS_Msk           (0x1U << ADC_CCR_DDS_Pos)                    /*!< 0x00002000 */
#define ADC_CCR_DDS               ADC_CCR_DDS_Msk                              /*!<DMA disable selection (Multi-ADC mode) */
#define ADC_CCR_DMA_Pos           (14U)                                        
#define ADC_CCR_DMA_Msk           (0x3U << ADC_CCR_DMA_Pos)                    /*!< 0x0000C000 */
#define ADC_CCR_DMA               ADC_CCR_DMA_Msk                              /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */  
#define ADC_CCR_DMA_0             (0x1U << ADC_CCR_DMA_Pos)                    /*!< 0x00004000 */
#define ADC_CCR_DMA_1             (0x2U << ADC_CCR_DMA_Pos)                    /*!< 0x00008000 */
#define ADC_CCR_ADCPRE_Pos        (16U)                                        
#define ADC_CCR_ADCPRE_Msk        (0x3U << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00030000 */
#define ADC_CCR_ADCPRE            ADC_CCR_ADCPRE_Msk                           /*!<ADCPRE[1:0] bits (ADC prescaler) */  
#define ADC_CCR_ADCPRE_0          (0x1U << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00010000 */
#define ADC_CCR_ADCPRE_1          (0x2U << ADC_CCR_ADCPRE_Pos)                 /*!< 0x00020000 */
#define ADC_CCR_VBATE_Pos         (22U)                                        
#define ADC_CCR_VBATE_Msk         (0x1U << ADC_CCR_VBATE_Pos)                  /*!< 0x00400000 */
#define ADC_CCR_VBATE             ADC_CCR_VBATE_Msk                            /*!<VBAT Enable */
#define ADC_CCR_TSVREFE_Pos       (23U)                                        
#define ADC_CCR_TSVREFE_Msk       (0x1U << ADC_CCR_TSVREFE_Pos)                /*!< 0x00800000 */
#define ADC_CCR_TSVREFE           ADC_CCR_TSVREFE_Msk                          /*!<Temperature Sensor and VREFINT Enable */

/*******************  Bit definition for ADC_CDR register  ********************/
#define ADC_CDR_DATA1_Pos         (0U)                                         
#define ADC_CDR_DATA1_Msk         (0xFFFFU << ADC_CDR_DATA1_Pos)               /*!< 0x0000FFFF */
#define ADC_CDR_DATA1             ADC_CDR_DATA1_Msk                            /*!<1st data of a pair of regular conversions */
#define ADC_CDR_DATA2_Pos         (16U)                                        
#define ADC_CDR_DATA2_Msk         (0xFFFFU << ADC_CDR_DATA2_Pos)               /*!< 0xFFFF0000 */
#define ADC_CDR_DATA2             ADC_CDR_DATA2_Msk                            /*!<2nd data of a pair of regular conversions */

/* Legacy defines */
#define ADC_CDR_RDATA_MST         ADC_CDR_DATA1
#define ADC_CDR_RDATA_SLV         ADC_CDR_DATA2

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define CRC_DR_DR_Pos       (0U)                                               
#define CRC_DR_DR_Msk       (0xFFFFFFFFU << CRC_DR_DR_Pos)                     /*!< 0xFFFFFFFF */
#define CRC_DR_DR           CRC_DR_DR_Msk                                      /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define CRC_IDR_IDR_Pos     (0U)                                               
#define CRC_IDR_IDR_Msk     (0xFFU << CRC_IDR_IDR_Pos)                         /*!< 0x000000FF */
#define CRC_IDR_IDR         CRC_IDR_IDR_Msk                                    /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define CRC_CR_RESET_Pos    (0U)                                               
#define CRC_CR_RESET_Msk    (0x1U << CRC_CR_RESET_Pos)                         /*!< 0x00000001 */
#define CRC_CR_RESET        CRC_CR_RESET_Msk                                   /*!< RESET bit */


/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/
#define DMA_SxCR_CHSEL_Pos       (25U)                                         
#define DMA_SxCR_CHSEL_Msk       (0x7U << DMA_SxCR_CHSEL_Pos)                  /*!< 0x0E000000 */
#define DMA_SxCR_CHSEL           DMA_SxCR_CHSEL_Msk                            
#define DMA_SxCR_CHSEL_0         0x02000000U                                   
#define DMA_SxCR_CHSEL_1         0x04000000U                                   
#define DMA_SxCR_CHSEL_2         0x08000000U                                   
#define DMA_SxCR_MBURST_Pos      (23U)                                         
#define DMA_SxCR_MBURST_Msk      (0x3U << DMA_SxCR_MBURST_Pos)                 /*!< 0x01800000 */
#define DMA_SxCR_MBURST          DMA_SxCR_MBURST_Msk                           
#define DMA_SxCR_MBURST_0        (0x1U << DMA_SxCR_MBURST_Pos)                 /*!< 0x00800000 */
#define DMA_SxCR_MBURST_1        (0x2U << DMA_SxCR_MBURST_Pos)                 /*!< 0x01000000 */
#define DMA_SxCR_PBURST_Pos      (21U)                                         
#define DMA_SxCR_PBURST_Msk      (0x3U << DMA_SxCR_PBURST_Pos)                 /*!< 0x00600000 */
#define DMA_SxCR_PBURST          DMA_SxCR_PBURST_Msk                           
#define DMA_SxCR_PBURST_0        (0x1U << DMA_SxCR_PBURST_Pos)                 /*!< 0x00200000 */
#define DMA_SxCR_PBURST_1        (0x2U << DMA_SxCR_PBURST_Pos)                 /*!< 0x00400000 */
#define DMA_SxCR_CT_Pos          (19U)                                         
#define DMA_SxCR_CT_Msk          (0x1U << DMA_SxCR_CT_Pos)                     /*!< 0x00080000 */
#define DMA_SxCR_CT              DMA_SxCR_CT_Msk                               
#define DMA_SxCR_DBM_Pos         (18U)                                         
#define DMA_SxCR_DBM_Msk         (0x1U << DMA_SxCR_DBM_Pos)                    /*!< 0x00040000 */
#define DMA_SxCR_DBM             DMA_SxCR_DBM_Msk                              
#define DMA_SxCR_PL_Pos          (16U)                                         
#define DMA_SxCR_PL_Msk          (0x3U << DMA_SxCR_PL_Pos)                     /*!< 0x00030000 */
#define DMA_SxCR_PL              DMA_SxCR_PL_Msk                               
#define DMA_SxCR_PL_0            (0x1U << DMA_SxCR_PL_Pos)                     /*!< 0x00010000 */
#define DMA_SxCR_PL_1            (0x2U << DMA_SxCR_PL_Pos)                     /*!< 0x00020000 */
#define DMA_SxCR_PINCOS_Pos      (15U)                                         
#define DMA_SxCR_PINCOS_Msk      (0x1U << DMA_SxCR_PINCOS_Pos)                 /*!< 0x00008000 */
#define DMA_SxCR_PINCOS          DMA_SxCR_PINCOS_Msk                           
#define DMA_SxCR_MSIZE_Pos       (13U)                                         
#define DMA_SxCR_MSIZE_Msk       (0x3U << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00006000 */
#define DMA_SxCR_MSIZE           DMA_SxCR_MSIZE_Msk                            
#define DMA_SxCR_MSIZE_0         (0x1U << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00002000 */
#define DMA_SxCR_MSIZE_1         (0x2U << DMA_SxCR_MSIZE_Pos)                  /*!< 0x00004000 */
#define DMA_SxCR_PSIZE_Pos       (11U)                                         
#define DMA_SxCR_PSIZE_Msk       (0x3U << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001800 */
#define DMA_SxCR_PSIZE           DMA_SxCR_PSIZE_Msk                            
#define DMA_SxCR_PSIZE_0         (0x1U << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00000800 */
#define DMA_SxCR_PSIZE_1         (0x2U << DMA_SxCR_PSIZE_Pos)                  /*!< 0x00001000 */
#define DMA_SxCR_MINC_Pos        (10U)                                         
#define DMA_SxCR_MINC_Msk        (0x1U << DMA_SxCR_MINC_Pos)                   /*!< 0x00000400 */
#define DMA_SxCR_MINC            DMA_SxCR_MINC_Msk                             
#define DMA_SxCR_PINC_Pos        (9U)                                          
#define DMA_SxCR_PINC_Msk        (0x1U << DMA_SxCR_PINC_Pos)                   /*!< 0x00000200 */
#define DMA_SxCR_PINC            DMA_SxCR_PINC_Msk                             
#define DMA_SxCR_CIRC_Pos        (8U)                                          
#define DMA_SxCR_CIRC_Msk        (0x1U << DMA_SxCR_CIRC_Pos)                   /*!< 0x00000100 */
#define DMA_SxCR_CIRC            DMA_SxCR_CIRC_Msk                             
#define DMA_SxCR_DIR_Pos         (6U)                                          
#define DMA_SxCR_DIR_Msk         (0x3U << DMA_SxCR_DIR_Pos)                    /*!< 0x000000C0 */
#define DMA_SxCR_DIR             DMA_SxCR_DIR_Msk                              
#define DMA_SxCR_DIR_0           (0x1U << DMA_SxCR_DIR_Pos)                    /*!< 0x00000040 */
#define DMA_SxCR_DIR_1           (0x2U << DMA_SxCR_DIR_Pos)                    /*!< 0x00000080 */
#define DMA_SxCR_PFCTRL_Pos      (5U)                                          
#define DMA_SxCR_PFCTRL_Msk      (0x1U << DMA_SxCR_PFCTRL_Pos)                 /*!< 0x00000020 */
#define DMA_SxCR_PFCTRL          DMA_SxCR_PFCTRL_Msk                           
#define DMA_SxCR_TCIE_Pos        (4U)                                          
#define DMA_SxCR_TCIE_Msk        (0x1U << DMA_SxCR_TCIE_Pos)                   /*!< 0x00000010 */
#define DMA_SxCR_TCIE            DMA_SxCR_TCIE_Msk                             
#define DMA_SxCR_HTIE_Pos        (3U)                                          
#define DMA_SxCR_HTIE_Msk        (0x1U << DMA_SxCR_HTIE_Pos)                   /*!< 0x00000008 */
#define DMA_SxCR_HTIE            DMA_SxCR_HTIE_Msk                             
#define DMA_SxCR_TEIE_Pos        (2U)                                          
#define DMA_SxCR_TEIE_Msk        (0x1U << DMA_SxCR_TEIE_Pos)                   /*!< 0x00000004 */
#define DMA_SxCR_TEIE            DMA_SxCR_TEIE_Msk                             
#define DMA_SxCR_DMEIE_Pos       (1U)                                          
#define DMA_SxCR_DMEIE_Msk       (0x1U << DMA_SxCR_DMEIE_Pos)                  /*!< 0x00000002 */
#define DMA_SxCR_DMEIE           DMA_SxCR_DMEIE_Msk                            
#define DMA_SxCR_EN_Pos          (0U)                                          
#define DMA_SxCR_EN_Msk          (0x1U << DMA_SxCR_EN_Pos)                     /*!< 0x00000001 */
#define DMA_SxCR_EN              DMA_SxCR_EN_Msk                               

/* Legacy defines */
#define DMA_SxCR_ACK_Pos         (20U)                                         
#define DMA_SxCR_ACK_Msk         (0x1U << DMA_SxCR_ACK_Pos)                    /*!< 0x00100000 */
#define DMA_SxCR_ACK             DMA_SxCR_ACK_Msk                              

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT_Pos            (0U)                                          
#define DMA_SxNDT_Msk            (0xFFFFU << DMA_SxNDT_Pos)                    /*!< 0x0000FFFF */
#define DMA_SxNDT                DMA_SxNDT_Msk                                 
#define DMA_SxNDT_0              (0x0001U << DMA_SxNDT_Pos)                    /*!< 0x00000001 */
#define DMA_SxNDT_1              (0x0002U << DMA_SxNDT_Pos)                    /*!< 0x00000002 */
#define DMA_SxNDT_2              (0x0004U << DMA_SxNDT_Pos)                    /*!< 0x00000004 */
#define DMA_SxNDT_3              (0x0008U << DMA_SxNDT_Pos)                    /*!< 0x00000008 */
#define DMA_SxNDT_4              (0x0010U << DMA_SxNDT_Pos)                    /*!< 0x00000010 */
#define DMA_SxNDT_5              (0x0020U << DMA_SxNDT_Pos)                    /*!< 0x00000020 */
#define DMA_SxNDT_6              (0x0040U << DMA_SxNDT_Pos)                    /*!< 0x00000040 */
#define DMA_SxNDT_7              (0x0080U << DMA_SxNDT_Pos)                    /*!< 0x00000080 */
#define DMA_SxNDT_8              (0x0100U << DMA_SxNDT_Pos)                    /*!< 0x00000100 */
#define DMA_SxNDT_9              (0x0200U << DMA_SxNDT_Pos)                    /*!< 0x00000200 */
#define DMA_SxNDT_10             (0x0400U << DMA_SxNDT_Pos)                    /*!< 0x00000400 */
#define DMA_SxNDT_11             (0x0800U << DMA_SxNDT_Pos)                    /*!< 0x00000800 */
#define DMA_SxNDT_12             (0x1000U << DMA_SxNDT_Pos)                    /*!< 0x00001000 */
#define DMA_SxNDT_13             (0x2000U << DMA_SxNDT_Pos)                    /*!< 0x00002000 */
#define DMA_SxNDT_14             (0x4000U << DMA_SxNDT_Pos)                    /*!< 0x00004000 */
#define DMA_SxNDT_15             (0x8000U << DMA_SxNDT_Pos)                    /*!< 0x00008000 */

/********************  Bits definition for DMA_SxFCR register  ****************/ 
#define DMA_SxFCR_FEIE_Pos       (7U)                                          
#define DMA_SxFCR_FEIE_Msk       (0x1U << DMA_SxFCR_FEIE_Pos)                  /*!< 0x00000080 */
#define DMA_SxFCR_FEIE           DMA_SxFCR_FEIE_Msk                            
#define DMA_SxFCR_FS_Pos         (3U)                                          
#define DMA_SxFCR_FS_Msk         (0x7U << DMA_SxFCR_FS_Pos)                    /*!< 0x00000038 */
#define DMA_SxFCR_FS             DMA_SxFCR_FS_Msk                              
#define DMA_SxFCR_FS_0           (0x1U << DMA_SxFCR_FS_Pos)                    /*!< 0x00000008 */
#define DMA_SxFCR_FS_1           (0x2U << DMA_SxFCR_FS_Pos)                    /*!< 0x00000010 */
#define DMA_SxFCR_FS_2           (0x4U << DMA_SxFCR_FS_Pos)                    /*!< 0x00000020 */
#define DMA_SxFCR_DMDIS_Pos      (2U)                                          
#define DMA_SxFCR_DMDIS_Msk      (0x1U << DMA_SxFCR_DMDIS_Pos)                 /*!< 0x00000004 */
#define DMA_SxFCR_DMDIS          DMA_SxFCR_DMDIS_Msk                           
#define DMA_SxFCR_FTH_Pos        (0U)                                          
#define DMA_SxFCR_FTH_Msk        (0x3U << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000003 */
#define DMA_SxFCR_FTH            DMA_SxFCR_FTH_Msk                             
#define DMA_SxFCR_FTH_0          (0x1U << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000001 */
#define DMA_SxFCR_FTH_1          (0x2U << DMA_SxFCR_FTH_Pos)                   /*!< 0x00000002 */

/********************  Bits definition for DMA_LISR register  *****************/ 
#define DMA_LISR_TCIF3_Pos       (27U)                                         
#define DMA_LISR_TCIF3_Msk       (0x1U << DMA_LISR_TCIF3_Pos)                  /*!< 0x08000000 */
#define DMA_LISR_TCIF3           DMA_LISR_TCIF3_Msk                            
#define DMA_LISR_HTIF3_Pos       (26U)                                         
#define DMA_LISR_HTIF3_Msk       (0x1U << DMA_LISR_HTIF3_Pos)                  /*!< 0x04000000 */
#define DMA_LISR_HTIF3           DMA_LISR_HTIF3_Msk                            
#define DMA_LISR_TEIF3_Pos       (25U)                                         
#define DMA_LISR_TEIF3_Msk       (0x1U << DMA_LISR_TEIF3_Pos)                  /*!< 0x02000000 */
#define DMA_LISR_TEIF3           DMA_LISR_TEIF3_Msk                            
#define DMA_LISR_DMEIF3_Pos      (24U)                                         
#define DMA_LISR_DMEIF3_Msk      (0x1U << DMA_LISR_DMEIF3_Pos)                 /*!< 0x01000000 */
#define DMA_LISR_DMEIF3          DMA_LISR_DMEIF3_Msk                           
#define DMA_LISR_FEIF3_Pos       (22U)                                         
#define DMA_LISR_FEIF3_Msk       (0x1U << DMA_LISR_FEIF3_Pos)                  /*!< 0x00400000 */
#define DMA_LISR_FEIF3           DMA_LISR_FEIF3_Msk                            
#define DMA_LISR_TCIF2_Pos       (21U)                                         
#define DMA_LISR_TCIF2_Msk       (0x1U << DMA_LISR_TCIF2_Pos)                  /*!< 0x00200000 */
#define DMA_LISR_TCIF2           DMA_LISR_TCIF2_Msk                            
#define DMA_LISR_HTIF2_Pos       (20U)                                         
#define DMA_LISR_HTIF2_Msk       (0x1U << DMA_LISR_HTIF2_Pos)                  /*!< 0x00100000 */
#define DMA_LISR_HTIF2           DMA_LISR_HTIF2_Msk                            
#define DMA_LISR_TEIF2_Pos       (19U)                                         
#define DMA_LISR_TEIF2_Msk       (0x1U << DMA_LISR_TEIF2_Pos)                  /*!< 0x00080000 */
#define DMA_LISR_TEIF2           DMA_LISR_TEIF2_Msk                            
#define DMA_LISR_DMEIF2_Pos      (18U)                                         
#define DMA_LISR_DMEIF2_Msk      (0x1U << DMA_LISR_DMEIF2_Pos)                 /*!< 0x00040000 */
#define DMA_LISR_DMEIF2          DMA_LISR_DMEIF2_Msk                           
#define DMA_LISR_FEIF2_Pos       (16U)                                         
#define DMA_LISR_FEIF2_Msk       (0x1U << DMA_LISR_FEIF2_Pos)                  /*!< 0x00010000 */
#define DMA_LISR_FEIF2           DMA_LISR_FEIF2_Msk                            
#define DMA_LISR_TCIF1_Pos       (11U)                                         
#define DMA_LISR_TCIF1_Msk       (0x1U << DMA_LISR_TCIF1_Pos)                  /*!< 0x00000800 */
#define DMA_LISR_TCIF1           DMA_LISR_TCIF1_Msk                            
#define DMA_LISR_HTIF1_Pos       (10U)                                         
#define DMA_LISR_HTIF1_Msk       (0x1U << DMA_LISR_HTIF1_Pos)                  /*!< 0x00000400 */
#define DMA_LISR_HTIF1           DMA_LISR_HTIF1_Msk                            
#define DMA_LISR_TEIF1_Pos       (9U)                                          
#define DMA_LISR_TEIF1_Msk       (0x1U << DMA_LISR_TEIF1_Pos)                  /*!< 0x00000200 */
#define DMA_LISR_TEIF1           DMA_LISR_TEIF1_Msk                            
#define DMA_LISR_DMEIF1_Pos      (8U)                                          
#define DMA_LISR_DMEIF1_Msk      (0x1U << DMA_LISR_DMEIF1_Pos)                 /*!< 0x00000100 */
#define DMA_LISR_DMEIF1          DMA_LISR_DMEIF1_Msk                           
#define DMA_LISR_FEIF1_Pos       (6U)                                          
#define DMA_LISR_FEIF1_Msk       (0x1U << DMA_LISR_FEIF1_Pos)                  /*!< 0x00000040 */
#define DMA_LISR_FEIF1           DMA_LISR_FEIF1_Msk                            
#define DMA_LISR_TCIF0_Pos       (5U)                                          
#define DMA_LISR_TCIF0_Msk       (0x1U << DMA_LISR_TCIF0_Pos)                  /*!< 0x00000020 */
#define DMA_LISR_TCIF0           DMA_LISR_TCIF0_Msk                            
#define DMA_LISR_HTIF0_Pos       (4U)                                          
#define DMA_LISR_HTIF0_Msk       (0x1U << DMA_LISR_HTIF0_Pos)                  /*!< 0x00000010 */
#define DMA_LISR_HTIF0           DMA_LISR_HTIF0_Msk                            
#define DMA_LISR_TEIF0_Pos       (3U)                                          
#define DMA_LISR_TEIF0_Msk       (0x1U << DMA_LISR_TEIF0_Pos)                  /*!< 0x00000008 */
#define DMA_LISR_TEIF0           DMA_LISR_TEIF0_Msk                            
#define DMA_LISR_DMEIF0_Pos      (2U)                                          
#define DMA_LISR_DMEIF0_Msk      (0x1U << DMA_LISR_DMEIF0_Pos)                 /*!< 0x00000004 */
#define DMA_LISR_DMEIF0          DMA_LISR_DMEIF0_Msk                           
#define DMA_LISR_FEIF0_Pos       (0U)                                          
#define DMA_LISR_FEIF0_Msk       (0x1U << DMA_LISR_FEIF0_Pos)                  /*!< 0x00000001 */
#define DMA_LISR_FEIF0           DMA_LISR_FEIF0_Msk                            

/********************  Bits definition for DMA_HISR register  *****************/ 
#define DMA_HISR_TCIF7_Pos       (27U)                                         
#define DMA_HISR_TCIF7_Msk       (0x1U << DMA_HISR_TCIF7_Pos)                  /*!< 0x08000000 */
#define DMA_HISR_TCIF7           DMA_HISR_TCIF7_Msk                            
#define DMA_HISR_HTIF7_Pos       (26U)                                         
#define DMA_HISR_HTIF7_Msk       (0x1U << DMA_HISR_HTIF7_Pos)                  /*!< 0x04000000 */
#define DMA_HISR_HTIF7           DMA_HISR_HTIF7_Msk                            
#define DMA_HISR_TEIF7_Pos       (25U)                                         
#define DMA_HISR_TEIF7_Msk       (0x1U << DMA_HISR_TEIF7_Pos)                  /*!< 0x02000000 */
#define DMA_HISR_TEIF7           DMA_HISR_TEIF7_Msk                            
#define DMA_HISR_DMEIF7_Pos      (24U)                                         
#define DMA_HISR_DMEIF7_Msk      (0x1U << DMA_HISR_DMEIF7_Pos)                 /*!< 0x01000000 */
#define DMA_HISR_DMEIF7          DMA_HISR_DMEIF7_Msk                           
#define DMA_HISR_FEIF7_Pos       (22U)                                         
#define DMA_HISR_FEIF7_Msk       (0x1U << DMA_HISR_FEIF7_Pos)                  /*!< 0x00400000 */
#define DMA_HISR_FEIF7           DMA_HISR_FEIF7_Msk                            
#define DMA_HISR_TCIF6_Pos       (21U)                                         
#define DMA_HISR_TCIF6_Msk       (0x1U << DMA_HISR_TCIF6_Pos)                  /*!< 0x00200000 */
#define DMA_HISR_TCIF6           DMA_HISR_TCIF6_Msk                            
#define DMA_HISR_HTIF6_Pos       (20U)                                         
#define DMA_HISR_HTIF6_Msk       (0x1U << DMA_HISR_HTIF6_Pos)                  /*!< 0x00100000 */
#define DMA_HISR_HTIF6           DMA_HISR_HTIF6_Msk                            
#define DMA_HISR_TEIF6_Pos       (19U)                                         
#define DMA_HISR_TEIF6_Msk       (0x1U << DMA_HISR_TEIF6_Pos)                  /*!< 0x00080000 */
#define DMA_HISR_TEIF6           DMA_HISR_TEIF6_Msk                            
#define DMA_HISR_DMEIF6_Pos      (18U)                                         
#define DMA_HISR_DMEIF6_Msk      (0x1U << DMA_HISR_DMEIF6_Pos)                 /*!< 0x00040000 */
#define DMA_HISR_DMEIF6          DMA_HISR_DMEIF6_Msk                           
#define DMA_HISR_FEIF6_Pos       (16U)                                         
#define DMA_HISR_FEIF6_Msk       (0x1U << DMA_HISR_FEIF6_Pos)                  /*!< 0x00010000 */
#define DMA_HISR_FEIF6           DMA_HISR_FEIF6_Msk                            
#define DMA_HISR_TCIF5_Pos       (11U)                                         
#define DMA_HISR_TCIF5_Msk       (0x1U << DMA_HISR_TCIF5_Pos)                  /*!< 0x00000800 */
#define DMA_HISR_TCIF5           DMA_HISR_TCIF5_Msk                            
#define DMA_HISR_HTIF5_Pos       (10U)                                         
#define DMA_HISR_HTIF5_Msk       (0x1U << DMA_HISR_HTIF5_Pos)                  /*!< 0x00000400 */
#define DMA_HISR_HTIF5           DMA_HISR_HTIF5_Msk                            
#define DMA_HISR_TEIF5_Pos       (9U)                                          
#define DMA_HISR_TEIF5_Msk       (0x1U << DMA_HISR_TEIF5_Pos)                  /*!< 0x00000200 */
#define DMA_HISR_TEIF5           DMA_HISR_TEIF5_Msk                            
#define DMA_HISR_DMEIF5_Pos      (8U)                                          
#define DMA_HISR_DMEIF5_Msk      (0x1U << DMA_HISR_DMEIF5_Pos)                 /*!< 0x00000100 */
#define DMA_HISR_DMEIF5          DMA_HISR_DMEIF5_Msk                           
#define DMA_HISR_FEIF5_Pos       (6U)                                          
#define DMA_HISR_FEIF5_Msk       (0x1U << DMA_HISR_FEIF5_Pos)                  /*!< 0x00000040 */
#define DMA_HISR_FEIF5           DMA_HISR_FEIF5_Msk                            
#define DMA_HISR_TCIF4_Pos       (5U)                                          
#define DMA_HISR_TCIF4_Msk       (0x1U << DMA_HISR_TCIF4_Pos)                  /*!< 0x00000020 */
#define DMA_HISR_TCIF4           DMA_HISR_TCIF4_Msk                            
#define DMA_HISR_HTIF4_Pos       (4U)                                          
#define DMA_HISR_HTIF4_Msk       (0x1U << DMA_HISR_HTIF4_Pos)                  /*!< 0x00000010 */
#define DMA_HISR_HTIF4           DMA_HISR_HTIF4_Msk                            
#define DMA_HISR_TEIF4_Pos       (3U)                                          
#define DMA_HISR_TEIF4_Msk       (0x1U << DMA_HISR_TEIF4_Pos)                  /*!< 0x00000008 */
#define DMA_HISR_TEIF4           DMA_HISR_TEIF4_Msk                            
#define DMA_HISR_DMEIF4_Pos      (2U)                                          
#define DMA_HISR_DMEIF4_Msk      (0x1U << DMA_HISR_DMEIF4_Pos)                 /*!< 0x00000004 */
#define DMA_HISR_DMEIF4          DMA_HISR_DMEIF4_Msk                           
#define DMA_HISR_FEIF4_Pos       (0U)                                          
#define DMA_HISR_FEIF4_Msk       (0x1U << DMA_HISR_FEIF4_Pos)                  /*!< 0x00000001 */
#define DMA_HISR_FEIF4           DMA_HISR_FEIF4_Msk                            

/********************  Bits definition for DMA_LIFCR register  ****************/ 
#define DMA_LIFCR_CTCIF3_Pos     (27U)                                         
#define DMA_LIFCR_CTCIF3_Msk     (0x1U << DMA_LIFCR_CTCIF3_Pos)                /*!< 0x08000000 */
#define DMA_LIFCR_CTCIF3         DMA_LIFCR_CTCIF3_Msk                          
#define DMA_LIFCR_CHTIF3_Pos     (26U)                                         
#define DMA_LIFCR_CHTIF3_Msk     (0x1U << DMA_LIFCR_CHTIF3_Pos)                /*!< 0x04000000 */
#define DMA_LIFCR_CHTIF3         DMA_LIFCR_CHTIF3_Msk                          
#define DMA_LIFCR_CTEIF3_Pos     (25U)                                         
#define DMA_LIFCR_CTEIF3_Msk     (0x1U << DMA_LIFCR_CTEIF3_Pos)                /*!< 0x02000000 */
#define DMA_LIFCR_CTEIF3         DMA_LIFCR_CTEIF3_Msk                          
#define DMA_LIFCR_CDMEIF3_Pos    (24U)                                         
#define DMA_LIFCR_CDMEIF3_Msk    (0x1U << DMA_LIFCR_CDMEIF3_Pos)               /*!< 0x01000000 */
#define DMA_LIFCR_CDMEIF3        DMA_LIFCR_CDMEIF3_Msk                         
#define DMA_LIFCR_CFEIF3_Pos     (22U)                                         
#define DMA_LIFCR_CFEIF3_Msk     (0x1U << DMA_LIFCR_CFEIF3_Pos)                /*!< 0x00400000 */
#define DMA_LIFCR_CFEIF3         DMA_LIFCR_CFEIF3_Msk                          
#define DMA_LIFCR_CTCIF2_Pos     (21U)                                         
#define DMA_LIFCR_CTCIF2_Msk     (0x1U << DMA_LIFCR_CTCIF2_Pos)                /*!< 0x00200000 */
#define DMA_LIFCR_CTCIF2         DMA_LIFCR_CTCIF2_Msk                          
#define DMA_LIFCR_CHTIF2_Pos     (20U)                                         
#define DMA_LIFCR_CHTIF2_Msk     (0x1U << DMA_LIFCR_CHTIF2_Pos)                /*!< 0x00100000 */
#define DMA_LIFCR_CHTIF2         DMA_LIFCR_CHTIF2_Msk                          
#define DMA_LIFCR_CTEIF2_Pos     (19U)                                         
#define DMA_LIFCR_CTEIF2_Msk     (0x1U << DMA_LIFCR_CTEIF2_Pos)                /*!< 0x00080000 */
#define DMA_LIFCR_CTEIF2         DMA_LIFCR_CTEIF2_Msk                          
#define DMA_LIFCR_CDMEIF2_Pos    (18U)                                         
#define DMA_LIFCR_CDMEIF2_Msk    (0x1U << DMA_LIFCR_CDMEIF2_Pos)               /*!< 0x00040000 */
#define DMA_LIFCR_CDMEIF2        DMA_LIFCR_CDMEIF2_Msk                         
#define DMA_LIFCR_CFEIF2_Pos     (16U)                                         
#define DMA_LIFCR_CFEIF2_Msk     (0x1U << DMA_LIFCR_CFEIF2_Pos)                /*!< 0x00010000 */
#define DMA_LIFCR_CFEIF2         DMA_LIFCR_CFEIF2_Msk                          
#define DMA_LIFCR_CTCIF1_Pos     (11U)                                         
#define DMA_LIFCR_CTCIF1_Msk     (0x1U << DMA_LIFCR_CTCIF1_Pos)                /*!< 0x00000800 */
#define DMA_LIFCR_CTCIF1         DMA_LIFCR_CTCIF1_Msk                          
#define DMA_LIFCR_CHTIF1_Pos     (10U)                                         
#define DMA_LIFCR_CHTIF1_Msk     (0x1U << DMA_LIFCR_CHTIF1_Pos)                /*!< 0x00000400 */
#define DMA_LIFCR_CHTIF1         DMA_LIFCR_CHTIF1_Msk                          
#define DMA_LIFCR_CTEIF1_Pos     (9U)                                          
#define DMA_LIFCR_CTEIF1_Msk     (0x1U << DMA_LIFCR_CTEIF1_Pos)                /*!< 0x00000200 */
#define DMA_LIFCR_CTEIF1         DMA_LIFCR_CTEIF1_Msk                          
#define DMA_LIFCR_CDMEIF1_Pos    (8U)                                          
#define DMA_LIFCR_CDMEIF1_Msk    (0x1U << DMA_LIFCR_CDMEIF1_Pos)               /*!< 0x00000100 */
#define DMA_LIFCR_CDMEIF1        DMA_LIFCR_CDMEIF1_Msk                         
#define DMA_LIFCR_CFEIF1_Pos     (6U)                                          
#define DMA_LIFCR_CFEIF1_Msk     (0x1U << DMA_LIFCR_CFEIF1_Pos)                /*!< 0x00000040 */
#define DMA_LIFCR_CFEIF1         DMA_LIFCR_CFEIF1_Msk                          
#define DMA_LIFCR_CTCIF0_Pos     (5U)                                          
#define DMA_LIFCR_CTCIF0_Msk     (0x1U << DMA_LIFCR_CTCIF0_Pos)                /*!< 0x00000020 */
#define DMA_LIFCR_CTCIF0         DMA_LIFCR_CTCIF0_Msk                          
#define DMA_LIFCR_CHTIF0_Pos     (4U)                                          
#define DMA_LIFCR_CHTIF0_Msk     (0x1U << DMA_LIFCR_CHTIF0_Pos)                /*!< 0x00000010 */
#define DMA_LIFCR_CHTIF0         DMA_LIFCR_CHTIF0_Msk                          
#define DMA_LIFCR_CTEIF0_Pos     (3U)                                          
#define DMA_LIFCR_CTEIF0_Msk     (0x1U << DMA_LIFCR_CTEIF0_Pos)                /*!< 0x00000008 */
#define DMA_LIFCR_CTEIF0         DMA_LIFCR_CTEIF0_Msk                          
#define DMA_LIFCR_CDMEIF0_Pos    (2U)                                          
#define DMA_LIFCR_CDMEIF0_Msk    (0x1U << DMA_LIFCR_CDMEIF0_Pos)               /*!< 0x00000004 */
#define DMA_LIFCR_CDMEIF0        DMA_LIFCR_CDMEIF0_Msk                         
#define DMA_LIFCR_CFEIF0_Pos     (0U)                                          
#define DMA_LIFCR_CFEIF0_Msk     (0x1U << DMA_LIFCR_CFEIF0_Pos)                /*!< 0x00000001 */
#define DMA_LIFCR_CFEIF0         DMA_LIFCR_CFEIF0_Msk                          

/********************  Bits definition for DMA_HIFCR  register  ****************/ 
#define DMA_HIFCR_CTCIF7_Pos     (27U)                                         
#define DMA_HIFCR_CTCIF7_Msk     (0x1U << DMA_HIFCR_CTCIF7_Pos)                /*!< 0x08000000 */
#define DMA_HIFCR_CTCIF7         DMA_HIFCR_CTCIF7_Msk                          
#define DMA_HIFCR_CHTIF7_Pos     (26U)                                         
#define DMA_HIFCR_CHTIF7_Msk     (0x1U << DMA_HIFCR_CHTIF7_Pos)                /*!< 0x04000000 */
#define DMA_HIFCR_CHTIF7         DMA_HIFCR_CHTIF7_Msk                          
#define DMA_HIFCR_CTEIF7_Pos     (25U)                                         
#define DMA_HIFCR_CTEIF7_Msk     (0x1U << DMA_HIFCR_CTEIF7_Pos)                /*!< 0x02000000 */
#define DMA_HIFCR_CTEIF7         DMA_HIFCR_CTEIF7_Msk                          
#define DMA_HIFCR_CDMEIF7_Pos    (24U)                                         
#define DMA_HIFCR_CDMEIF7_Msk    (0x1U << DMA_HIFCR_CDMEIF7_Pos)               /*!< 0x01000000 */
#define DMA_HIFCR_CDMEIF7        DMA_HIFCR_CDMEIF7_Msk                         
#define DMA_HIFCR_CFEIF7_Pos     (22U)                                         
#define DMA_HIFCR_CFEIF7_Msk     (0x1U << DMA_HIFCR_CFEIF7_Pos)                /*!< 0x00400000 */
#define DMA_HIFCR_CFEIF7         DMA_HIFCR_CFEIF7_Msk                          
#define DMA_HIFCR_CTCIF6_Pos     (21U)                                         
#define DMA_HIFCR_CTCIF6_Msk     (0x1U << DMA_HIFCR_CTCIF6_Pos)                /*!< 0x00200000 */
#define DMA_HIFCR_CTCIF6         DMA_HIFCR_CTCIF6_Msk                          
#define DMA_HIFCR_CHTIF6_Pos     (20U)                                         
#define DMA_HIFCR_CHTIF6_Msk     (0x1U << DMA_HIFCR_CHTIF6_Pos)                /*!< 0x00100000 */
#define DMA_HIFCR_CHTIF6         DMA_HIFCR_CHTIF6_Msk                          
#define DMA_HIFCR_CTEIF6_Pos     (19U)                                         
#define DMA_HIFCR_CTEIF6_Msk     (0x1U << DMA_HIFCR_CTEIF6_Pos)                /*!< 0x00080000 */
#define DMA_HIFCR_CTEIF6         DMA_HIFCR_CTEIF6_Msk                          
#define DMA_HIFCR_CDMEIF6_Pos    (18U)                                         
#define DMA_HIFCR_CDMEIF6_Msk    (0x1U << DMA_HIFCR_CDMEIF6_Pos)               /*!< 0x00040000 */
#define DMA_HIFCR_CDMEIF6        DMA_HIFCR_CDMEIF6_Msk                         
#define DMA_HIFCR_CFEIF6_Pos     (16U)                                         
#define DMA_HIFCR_CFEIF6_Msk     (0x1U << DMA_HIFCR_CFEIF6_Pos)                /*!< 0x00010000 */
#define DMA_HIFCR_CFEIF6         DMA_HIFCR_CFEIF6_Msk                          
#define DMA_HIFCR_CTCIF5_Pos     (11U)                                         
#define DMA_HIFCR_CTCIF5_Msk     (0x1U << DMA_HIFCR_CTCIF5_Pos)                /*!< 0x00000800 */
#define DMA_HIFCR_CTCIF5         DMA_HIFCR_CTCIF5_Msk                          
#define DMA_HIFCR_CHTIF5_Pos     (10U)                                         
#define DMA_HIFCR_CHTIF5_Msk     (0x1U << DMA_HIFCR_CHTIF5_Pos)                /*!< 0x00000400 */
#define DMA_HIFCR_CHTIF5         DMA_HIFCR_CHTIF5_Msk                          
#define DMA_HIFCR_CTEIF5_Pos     (9U)                                          
#define DMA_HIFCR_CTEIF5_Msk     (0x1U << DMA_HIFCR_CTEIF5_Pos)                /*!< 0x00000200 */
#define DMA_HIFCR_CTEIF5         DMA_HIFCR_CTEIF5_Msk                          
#define DMA_HIFCR_CDMEIF5_Pos    (8U)                                          
#define DMA_HIFCR_CDMEIF5_Msk    (0x1U << DMA_HIFCR_CDMEIF5_Pos)               /*!< 0x00000100 */
#define DMA_HIFCR_CDMEIF5        DMA_HIFCR_CDMEIF5_Msk                         
#define DMA_HIFCR_CFEIF5_Pos     (6U)                                          
#define DMA_HIFCR_CFEIF5_Msk     (0x1U << DMA_HIFCR_CFEIF5_Pos)                /*!< 0x00000040 */
#define DMA_HIFCR_CFEIF5         DMA_HIFCR_CFEIF5_Msk                          
#define DMA_HIFCR_CTCIF4_Pos     (5U)                                          
#define DMA_HIFCR_CTCIF4_Msk     (0x1U << DMA_HIFCR_CTCIF4_Pos)                /*!< 0x00000020 */
#define DMA_HIFCR_CTCIF4         DMA_HIFCR_CTCIF4_Msk                          
#define DMA_HIFCR_CHTIF4_Pos     (4U)                                          
#define DMA_HIFCR_CHTIF4_Msk     (0x1U << DMA_HIFCR_CHTIF4_Pos)                /*!< 0x00000010 */
#define DMA_HIFCR_CHTIF4         DMA_HIFCR_CHTIF4_Msk                          
#define DMA_HIFCR_CTEIF4_Pos     (3U)                                          
#define DMA_HIFCR_CTEIF4_Msk     (0x1U << DMA_HIFCR_CTEIF4_Pos)                /*!< 0x00000008 */
#define DMA_HIFCR_CTEIF4         DMA_HIFCR_CTEIF4_Msk                          
#define DMA_HIFCR_CDMEIF4_Pos    (2U)                                          
#define DMA_HIFCR_CDMEIF4_Msk    (0x1U << DMA_HIFCR_CDMEIF4_Pos)               /*!< 0x00000004 */
#define DMA_HIFCR_CDMEIF4        DMA_HIFCR_CDMEIF4_Msk                         
#define DMA_HIFCR_CFEIF4_Pos     (0U)                                          
#define DMA_HIFCR_CFEIF4_Msk     (0x1U << DMA_HIFCR_CFEIF4_Pos)                /*!< 0x00000001 */
#define DMA_HIFCR_CFEIF4         DMA_HIFCR_CFEIF4_Msk                          

/******************  Bit definition for DMA_SxPAR register  ********************/
#define DMA_SxPAR_PA_Pos         (0U)                                          
#define DMA_SxPAR_PA_Msk         (0xFFFFFFFFU << DMA_SxPAR_PA_Pos)             /*!< 0xFFFFFFFF */
#define DMA_SxPAR_PA             DMA_SxPAR_PA_Msk                              /*!< Peripheral Address */

/******************  Bit definition for DMA_SxM0AR register  ********************/
#define DMA_SxM0AR_M0A_Pos       (0U)                                          
#define DMA_SxM0AR_M0A_Msk       (0xFFFFFFFFU << DMA_SxM0AR_M0A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM0AR_M0A           DMA_SxM0AR_M0A_Msk                            /*!< Memory Address */

/******************  Bit definition for DMA_SxM1AR register  ********************/
#define DMA_SxM1AR_M1A_Pos       (0U)                                          
#define DMA_SxM1AR_M1A_Msk       (0xFFFFFFFFU << DMA_SxM1AR_M1A_Pos)           /*!< 0xFFFFFFFF */
#define DMA_SxM1AR_M1A           DMA_SxM1AR_M1A_Msk                            /*!< Memory Address */


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)                                         
#define EXTI_IMR_MR0_Msk          (0x1U << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR0              EXTI_IMR_MR0_Msk                             /*!< Interrupt Mask on line 0 */
#define EXTI_IMR_MR1_Pos          (1U)                                         
#define EXTI_IMR_MR1_Msk          (0x1U << EXTI_IMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_IMR_MR1              EXTI_IMR_MR1_Msk                             /*!< Interrupt Mask on line 1 */
#define EXTI_IMR_MR2_Pos          (2U)                                         
#define EXTI_IMR_MR2_Msk          (0x1U << EXTI_IMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_IMR_MR2              EXTI_IMR_MR2_Msk                             /*!< Interrupt Mask on line 2 */
#define EXTI_IMR_MR3_Pos          (3U)                                         
#define EXTI_IMR_MR3_Msk          (0x1U << EXTI_IMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_IMR_MR3              EXTI_IMR_MR3_Msk                             /*!< Interrupt Mask on line 3 */
#define EXTI_IMR_MR4_Pos          (4U)                                         
#define EXTI_IMR_MR4_Msk          (0x1U << EXTI_IMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_IMR_MR4              EXTI_IMR_MR4_Msk                             /*!< Interrupt Mask on line 4 */
#define EXTI_IMR_MR5_Pos          (5U)                                         
#define EXTI_IMR_MR5_Msk          (0x1U << EXTI_IMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_IMR_MR5              EXTI_IMR_MR5_Msk                             /*!< Interrupt Mask on line 5 */
#define EXTI_IMR_MR6_Pos          (6U)                                         
#define EXTI_IMR_MR6_Msk          (0x1U << EXTI_IMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_IMR_MR6              EXTI_IMR_MR6_Msk                             /*!< Interrupt Mask on line 6 */
#define EXTI_IMR_MR7_Pos          (7U)                                         
#define EXTI_IMR_MR7_Msk          (0x1U << EXTI_IMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_IMR_MR7              EXTI_IMR_MR7_Msk                             /*!< Interrupt Mask on line 7 */
#define EXTI_IMR_MR8_Pos          (8U)                                         
#define EXTI_IMR_MR8_Msk          (0x1U << EXTI_IMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_IMR_MR8              EXTI_IMR_MR8_Msk                             /*!< Interrupt Mask on line 8 */
#define EXTI_IMR_MR9_Pos          (9U)                                         
#define EXTI_IMR_MR9_Msk          (0x1U << EXTI_IMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_IMR_MR9              EXTI_IMR_MR9_Msk                             /*!< Interrupt Mask on line 9 */
#define EXTI_IMR_MR10_Pos         (10U)                                        
#define EXTI_IMR_MR10_Msk         (0x1U << EXTI_IMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_IMR_MR10             EXTI_IMR_MR10_Msk                            /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos         (11U)                                        
#define EXTI_IMR_MR11_Msk         (0x1U << EXTI_IMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_IMR_MR11             EXTI_IMR_MR11_Msk                            /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos         (12U)                                        
#define EXTI_IMR_MR12_Msk         (0x1U << EXTI_IMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_IMR_MR12             EXTI_IMR_MR12_Msk                            /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos         (13U)                                        
#define EXTI_IMR_MR13_Msk         (0x1U << EXTI_IMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_IMR_MR13             EXTI_IMR_MR13_Msk                            /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos         (14U)                                        
#define EXTI_IMR_MR14_Msk         (0x1U << EXTI_IMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_IMR_MR14             EXTI_IMR_MR14_Msk                            /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos         (15U)                                        
#define EXTI_IMR_MR15_Msk         (0x1U << EXTI_IMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_IMR_MR15             EXTI_IMR_MR15_Msk                            /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR16_Pos         (16U)                                        
#define EXTI_IMR_MR16_Msk         (0x1U << EXTI_IMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_IMR_MR16             EXTI_IMR_MR16_Msk                            /*!< Interrupt Mask on line 16 */
#define EXTI_IMR_MR17_Pos         (17U)                                        
#define EXTI_IMR_MR17_Msk         (0x1U << EXTI_IMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_IMR_MR17             EXTI_IMR_MR17_Msk                            /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR18_Pos         (18U)                                        
#define EXTI_IMR_MR18_Msk         (0x1U << EXTI_IMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_IMR_MR18             EXTI_IMR_MR18_Msk                            /*!< Interrupt Mask on line 18 */
#define EXTI_IMR_MR19_Pos         (19U)                                        
#define EXTI_IMR_MR19_Msk         (0x1U << EXTI_IMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_IMR_MR19             EXTI_IMR_MR19_Msk                            /*!< Interrupt Mask on line 19 */
#define EXTI_IMR_MR20_Pos         (20U)                                        
#define EXTI_IMR_MR20_Msk         (0x1U << EXTI_IMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_IMR_MR20             EXTI_IMR_MR20_Msk                            /*!< Interrupt Mask on line 20 */
#define EXTI_IMR_MR21_Pos         (21U)                                        
#define EXTI_IMR_MR21_Msk         (0x1U << EXTI_IMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_IMR_MR21             EXTI_IMR_MR21_Msk                            /*!< Interrupt Mask on line 21 */
#define EXTI_IMR_MR22_Pos         (22U)                                        
#define EXTI_IMR_MR22_Msk         (0x1U << EXTI_IMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_IMR_MR22             EXTI_IMR_MR22_Msk                            /*!< Interrupt Mask on line 22 */

/* Reference Defines */
#define  EXTI_IMR_IM0                        EXTI_IMR_MR0
#define  EXTI_IMR_IM1                        EXTI_IMR_MR1
#define  EXTI_IMR_IM2                        EXTI_IMR_MR2
#define  EXTI_IMR_IM3                        EXTI_IMR_MR3
#define  EXTI_IMR_IM4                        EXTI_IMR_MR4
#define  EXTI_IMR_IM5                        EXTI_IMR_MR5
#define  EXTI_IMR_IM6                        EXTI_IMR_MR6
#define  EXTI_IMR_IM7                        EXTI_IMR_MR7
#define  EXTI_IMR_IM8                        EXTI_IMR_MR8
#define  EXTI_IMR_IM9                        EXTI_IMR_MR9
#define  EXTI_IMR_IM10                       EXTI_IMR_MR10
#define  EXTI_IMR_IM11                       EXTI_IMR_MR11
#define  EXTI_IMR_IM12                       EXTI_IMR_MR12
#define  EXTI_IMR_IM13                       EXTI_IMR_MR13
#define  EXTI_IMR_IM14                       EXTI_IMR_MR14
#define  EXTI_IMR_IM15                       EXTI_IMR_MR15
#define  EXTI_IMR_IM16                       EXTI_IMR_MR16
#define  EXTI_IMR_IM17                       EXTI_IMR_MR17
#define  EXTI_IMR_IM18                       EXTI_IMR_MR18
#define  EXTI_IMR_IM19                       EXTI_IMR_MR19
#define  EXTI_IMR_IM20                       EXTI_IMR_MR20
#define  EXTI_IMR_IM21                       EXTI_IMR_MR21
#define  EXTI_IMR_IM22                       EXTI_IMR_MR22
#define EXTI_IMR_IM_Pos           (0U)                                         
#define EXTI_IMR_IM_Msk           (0x7FFFFFU << EXTI_IMR_IM_Pos)               /*!< 0x007FFFFF */
#define EXTI_IMR_IM               EXTI_IMR_IM_Msk                              /*!< Interrupt Mask All */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define EXTI_EMR_MR0_Pos          (0U)                                         
#define EXTI_EMR_MR0_Msk          (0x1U << EXTI_EMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_EMR_MR0              EXTI_EMR_MR0_Msk                             /*!< Event Mask on line 0 */
#define EXTI_EMR_MR1_Pos          (1U)                                         
#define EXTI_EMR_MR1_Msk          (0x1U << EXTI_EMR_MR1_Pos)                   /*!< 0x00000002 */
#define EXTI_EMR_MR1              EXTI_EMR_MR1_Msk                             /*!< Event Mask on line 1 */
#define EXTI_EMR_MR2_Pos          (2U)                                         
#define EXTI_EMR_MR2_Msk          (0x1U << EXTI_EMR_MR2_Pos)                   /*!< 0x00000004 */
#define EXTI_EMR_MR2              EXTI_EMR_MR2_Msk                             /*!< Event Mask on line 2 */
#define EXTI_EMR_MR3_Pos          (3U)                                         
#define EXTI_EMR_MR3_Msk          (0x1U << EXTI_EMR_MR3_Pos)                   /*!< 0x00000008 */
#define EXTI_EMR_MR3              EXTI_EMR_MR3_Msk                             /*!< Event Mask on line 3 */
#define EXTI_EMR_MR4_Pos          (4U)                                         
#define EXTI_EMR_MR4_Msk          (0x1U << EXTI_EMR_MR4_Pos)                   /*!< 0x00000010 */
#define EXTI_EMR_MR4              EXTI_EMR_MR4_Msk                             /*!< Event Mask on line 4 */
#define EXTI_EMR_MR5_Pos          (5U)                                         
#define EXTI_EMR_MR5_Msk          (0x1U << EXTI_EMR_MR5_Pos)                   /*!< 0x00000020 */
#define EXTI_EMR_MR5              EXTI_EMR_MR5_Msk                             /*!< Event Mask on line 5 */
#define EXTI_EMR_MR6_Pos          (6U)                                         
#define EXTI_EMR_MR6_Msk          (0x1U << EXTI_EMR_MR6_Pos)                   /*!< 0x00000040 */
#define EXTI_EMR_MR6              EXTI_EMR_MR6_Msk                             /*!< Event Mask on line 6 */
#define EXTI_EMR_MR7_Pos          (7U)                                         
#define EXTI_EMR_MR7_Msk          (0x1U << EXTI_EMR_MR7_Pos)                   /*!< 0x00000080 */
#define EXTI_EMR_MR7              EXTI_EMR_MR7_Msk                             /*!< Event Mask on line 7 */
#define EXTI_EMR_MR8_Pos          (8U)                                         
#define EXTI_EMR_MR8_Msk          (0x1U << EXTI_EMR_MR8_Pos)                   /*!< 0x00000100 */
#define EXTI_EMR_MR8              EXTI_EMR_MR8_Msk                             /*!< Event Mask on line 8 */
#define EXTI_EMR_MR9_Pos          (9U)                                         
#define EXTI_EMR_MR9_Msk          (0x1U << EXTI_EMR_MR9_Pos)                   /*!< 0x00000200 */
#define EXTI_EMR_MR9              EXTI_EMR_MR9_Msk                             /*!< Event Mask on line 9 */
#define EXTI_EMR_MR10_Pos         (10U)                                        
#define EXTI_EMR_MR10_Msk         (0x1U << EXTI_EMR_MR10_Pos)                  /*!< 0x00000400 */
#define EXTI_EMR_MR10             EXTI_EMR_MR10_Msk                            /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos         (11U)                                        
#define EXTI_EMR_MR11_Msk         (0x1U << EXTI_EMR_MR11_Pos)                  /*!< 0x00000800 */
#define EXTI_EMR_MR11             EXTI_EMR_MR11_Msk                            /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos         (12U)                                        
#define EXTI_EMR_MR12_Msk         (0x1U << EXTI_EMR_MR12_Pos)                  /*!< 0x00001000 */
#define EXTI_EMR_MR12             EXTI_EMR_MR12_Msk                            /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos         (13U)                                        
#define EXTI_EMR_MR13_Msk         (0x1U << EXTI_EMR_MR13_Pos)                  /*!< 0x00002000 */
#define EXTI_EMR_MR13             EXTI_EMR_MR13_Msk                            /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos         (14U)                                        
#define EXTI_EMR_MR14_Msk         (0x1U << EXTI_EMR_MR14_Pos)                  /*!< 0x00004000 */
#define EXTI_EMR_MR14             EXTI_EMR_MR14_Msk                            /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos         (15U)                                        
#define EXTI_EMR_MR15_Msk         (0x1U << EXTI_EMR_MR15_Pos)                  /*!< 0x00008000 */
#define EXTI_EMR_MR15             EXTI_EMR_MR15_Msk                            /*!< Event Mask on line 15 */
#define EXTI_EMR_MR16_Pos         (16U)                                        
#define EXTI_EMR_MR16_Msk         (0x1U << EXTI_EMR_MR16_Pos)                  /*!< 0x00010000 */
#define EXTI_EMR_MR16             EXTI_EMR_MR16_Msk                            /*!< Event Mask on line 16 */
#define EXTI_EMR_MR17_Pos         (17U)                                        
#define EXTI_EMR_MR17_Msk         (0x1U << EXTI_EMR_MR17_Pos)                  /*!< 0x00020000 */
#define EXTI_EMR_MR17             EXTI_EMR_MR17_Msk                            /*!< Event Mask on line 17 */
#define EXTI_EMR_MR18_Pos         (18U)                                        
#define EXTI_EMR_MR18_Msk         (0x1U << EXTI_EMR_MR18_Pos)                  /*!< 0x00040000 */
#define EXTI_EMR_MR18             EXTI_EMR_MR18_Msk                            /*!< Event Mask on line 18 */
#define EXTI_EMR_MR19_Pos         (19U)                                        
#define EXTI_EMR_MR19_Msk         (0x1U << EXTI_EMR_MR19_Pos)                  /*!< 0x00080000 */
#define EXTI_EMR_MR19             EXTI_EMR_MR19_Msk                            /*!< Event Mask on line 19 */
#define EXTI_EMR_MR20_Pos         (20U)                                        
#define EXTI_EMR_MR20_Msk         (0x1U << EXTI_EMR_MR20_Pos)                  /*!< 0x00100000 */
#define EXTI_EMR_MR20             EXTI_EMR_MR20_Msk                            /*!< Event Mask on line 20 */
#define EXTI_EMR_MR21_Pos         (21U)                                        
#define EXTI_EMR_MR21_Msk         (0x1U << EXTI_EMR_MR21_Pos)                  /*!< 0x00200000 */
#define EXTI_EMR_MR21             EXTI_EMR_MR21_Msk                            /*!< Event Mask on line 21 */
#define EXTI_EMR_MR22_Pos         (22U)                                        
#define EXTI_EMR_MR22_Msk         (0x1U << EXTI_EMR_MR22_Pos)                  /*!< 0x00400000 */
#define EXTI_EMR_MR22             EXTI_EMR_MR22_Msk                            /*!< Event Mask on line 22 */

/* Reference Defines */
#define  EXTI_EMR_EM0                        EXTI_EMR_MR0
#define  EXTI_EMR_EM1                        EXTI_EMR_MR1
#define  EXTI_EMR_EM2                        EXTI_EMR_MR2
#define  EXTI_EMR_EM3                        EXTI_EMR_MR3
#define  EXTI_EMR_EM4                        EXTI_EMR_MR4
#define  EXTI_EMR_EM5                        EXTI_EMR_MR5
#define  EXTI_EMR_EM6                        EXTI_EMR_MR6
#define  EXTI_EMR_EM7                        EXTI_EMR_MR7
#define  EXTI_EMR_EM8                        EXTI_EMR_MR8
#define  EXTI_EMR_EM9                        EXTI_EMR_MR9
#define  EXTI_EMR_EM10                       EXTI_EMR_MR10
#define  EXTI_EMR_EM11                       EXTI_EMR_MR11
#define  EXTI_EMR_EM12                       EXTI_EMR_MR12
#define  EXTI_EMR_EM13                       EXTI_EMR_MR13
#define  EXTI_EMR_EM14                       EXTI_EMR_MR14
#define  EXTI_EMR_EM15                       EXTI_EMR_MR15
#define  EXTI_EMR_EM16                       EXTI_EMR_MR16
#define  EXTI_EMR_EM17                       EXTI_EMR_MR17
#define  EXTI_EMR_EM18                       EXTI_EMR_MR18
#define  EXTI_EMR_EM19                       EXTI_EMR_MR19
#define  EXTI_EMR_EM20                       EXTI_EMR_MR20
#define  EXTI_EMR_EM21                       EXTI_EMR_MR21
#define  EXTI_EMR_EM22                       EXTI_EMR_MR22

/******************  Bit definition for EXTI_RTSR register  *******************/
#define EXTI_RTSR_TR0_Pos         (0U)                                         
#define EXTI_RTSR_TR0_Msk         (0x1U << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR0             EXTI_RTSR_TR0_Msk                            /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos         (1U)                                         
#define EXTI_RTSR_TR1_Msk         (0x1U << EXTI_RTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_RTSR_TR1             EXTI_RTSR_TR1_Msk                            /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos         (2U)                                         
#define EXTI_RTSR_TR2_Msk         (0x1U << EXTI_RTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_RTSR_TR2             EXTI_RTSR_TR2_Msk                            /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos         (3U)                                         
#define EXTI_RTSR_TR3_Msk         (0x1U << EXTI_RTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_RTSR_TR3             EXTI_RTSR_TR3_Msk                            /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos         (4U)                                         
#define EXTI_RTSR_TR4_Msk         (0x1U << EXTI_RTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_RTSR_TR4             EXTI_RTSR_TR4_Msk                            /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos         (5U)                                         
#define EXTI_RTSR_TR5_Msk         (0x1U << EXTI_RTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_RTSR_TR5             EXTI_RTSR_TR5_Msk                            /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos         (6U)                                         
#define EXTI_RTSR_TR6_Msk         (0x1U << EXTI_RTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_RTSR_TR6             EXTI_RTSR_TR6_Msk                            /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos         (7U)                                         
#define EXTI_RTSR_TR7_Msk         (0x1U << EXTI_RTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_RTSR_TR7             EXTI_RTSR_TR7_Msk                            /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos         (8U)                                         
#define EXTI_RTSR_TR8_Msk         (0x1U << EXTI_RTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_RTSR_TR8             EXTI_RTSR_TR8_Msk                            /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos         (9U)                                         
#define EXTI_RTSR_TR9_Msk         (0x1U << EXTI_RTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_RTSR_TR9             EXTI_RTSR_TR9_Msk                            /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos        (10U)                                        
#define EXTI_RTSR_TR10_Msk        (0x1U << EXTI_RTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_RTSR_TR10            EXTI_RTSR_TR10_Msk                           /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos        (11U)                                        
#define EXTI_RTSR_TR11_Msk        (0x1U << EXTI_RTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_RTSR_TR11            EXTI_RTSR_TR11_Msk                           /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos        (12U)                                        
#define EXTI_RTSR_TR12_Msk        (0x1U << EXTI_RTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_RTSR_TR12            EXTI_RTSR_TR12_Msk                           /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos        (13U)                                        
#define EXTI_RTSR_TR13_Msk        (0x1U << EXTI_RTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_RTSR_TR13            EXTI_RTSR_TR13_Msk                           /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos        (14U)                                        
#define EXTI_RTSR_TR14_Msk        (0x1U << EXTI_RTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_RTSR_TR14            EXTI_RTSR_TR14_Msk                           /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos        (15U)                                        
#define EXTI_RTSR_TR15_Msk        (0x1U << EXTI_RTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_RTSR_TR15            EXTI_RTSR_TR15_Msk                           /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos        (16U)                                        
#define EXTI_RTSR_TR16_Msk        (0x1U << EXTI_RTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_RTSR_TR16            EXTI_RTSR_TR16_Msk                           /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos        (17U)                                        
#define EXTI_RTSR_TR17_Msk        (0x1U << EXTI_RTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_RTSR_TR17            EXTI_RTSR_TR17_Msk                           /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR18_Pos        (18U)                                        
#define EXTI_RTSR_TR18_Msk        (0x1U << EXTI_RTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_RTSR_TR18            EXTI_RTSR_TR18_Msk                           /*!< Rising trigger event configuration bit of line 18 */
#define EXTI_RTSR_TR19_Pos        (19U)                                        
#define EXTI_RTSR_TR19_Msk        (0x1U << EXTI_RTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_RTSR_TR19            EXTI_RTSR_TR19_Msk                           /*!< Rising trigger event configuration bit of line 19 */
#define EXTI_RTSR_TR20_Pos        (20U)                                        
#define EXTI_RTSR_TR20_Msk        (0x1U << EXTI_RTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_RTSR_TR20            EXTI_RTSR_TR20_Msk                           /*!< Rising trigger event configuration bit of line 20 */
#define EXTI_RTSR_TR21_Pos        (21U)                                        
#define EXTI_RTSR_TR21_Msk        (0x1U << EXTI_RTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_RTSR_TR21            EXTI_RTSR_TR21_Msk                           /*!< Rising trigger event configuration bit of line 21 */
#define EXTI_RTSR_TR22_Pos        (22U)                                        
#define EXTI_RTSR_TR22_Msk        (0x1U << EXTI_RTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_RTSR_TR22            EXTI_RTSR_TR22_Msk                           /*!< Rising trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define EXTI_FTSR_TR0_Pos         (0U)                                         
#define EXTI_FTSR_TR0_Msk         (0x1U << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR0             EXTI_FTSR_TR0_Msk                            /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos         (1U)                                         
#define EXTI_FTSR_TR1_Msk         (0x1U << EXTI_FTSR_TR1_Pos)                  /*!< 0x00000002 */
#define EXTI_FTSR_TR1             EXTI_FTSR_TR1_Msk                            /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos         (2U)                                         
#define EXTI_FTSR_TR2_Msk         (0x1U << EXTI_FTSR_TR2_Pos)                  /*!< 0x00000004 */
#define EXTI_FTSR_TR2             EXTI_FTSR_TR2_Msk                            /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos         (3U)                                         
#define EXTI_FTSR_TR3_Msk         (0x1U << EXTI_FTSR_TR3_Pos)                  /*!< 0x00000008 */
#define EXTI_FTSR_TR3             EXTI_FTSR_TR3_Msk                            /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos         (4U)                                         
#define EXTI_FTSR_TR4_Msk         (0x1U << EXTI_FTSR_TR4_Pos)                  /*!< 0x00000010 */
#define EXTI_FTSR_TR4             EXTI_FTSR_TR4_Msk                            /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos         (5U)                                         
#define EXTI_FTSR_TR5_Msk         (0x1U << EXTI_FTSR_TR5_Pos)                  /*!< 0x00000020 */
#define EXTI_FTSR_TR5             EXTI_FTSR_TR5_Msk                            /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos         (6U)                                         
#define EXTI_FTSR_TR6_Msk         (0x1U << EXTI_FTSR_TR6_Pos)                  /*!< 0x00000040 */
#define EXTI_FTSR_TR6             EXTI_FTSR_TR6_Msk                            /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos         (7U)                                         
#define EXTI_FTSR_TR7_Msk         (0x1U << EXTI_FTSR_TR7_Pos)                  /*!< 0x00000080 */
#define EXTI_FTSR_TR7             EXTI_FTSR_TR7_Msk                            /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos         (8U)                                         
#define EXTI_FTSR_TR8_Msk         (0x1U << EXTI_FTSR_TR8_Pos)                  /*!< 0x00000100 */
#define EXTI_FTSR_TR8             EXTI_FTSR_TR8_Msk                            /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos         (9U)                                         
#define EXTI_FTSR_TR9_Msk         (0x1U << EXTI_FTSR_TR9_Pos)                  /*!< 0x00000200 */
#define EXTI_FTSR_TR9             EXTI_FTSR_TR9_Msk                            /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos        (10U)                                        
#define EXTI_FTSR_TR10_Msk        (0x1U << EXTI_FTSR_TR10_Pos)                 /*!< 0x00000400 */
#define EXTI_FTSR_TR10            EXTI_FTSR_TR10_Msk                           /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos        (11U)                                        
#define EXTI_FTSR_TR11_Msk        (0x1U << EXTI_FTSR_TR11_Pos)                 /*!< 0x00000800 */
#define EXTI_FTSR_TR11            EXTI_FTSR_TR11_Msk                           /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos        (12U)                                        
#define EXTI_FTSR_TR12_Msk        (0x1U << EXTI_FTSR_TR12_Pos)                 /*!< 0x00001000 */
#define EXTI_FTSR_TR12            EXTI_FTSR_TR12_Msk                           /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos        (13U)                                        
#define EXTI_FTSR_TR13_Msk        (0x1U << EXTI_FTSR_TR13_Pos)                 /*!< 0x00002000 */
#define EXTI_FTSR_TR13            EXTI_FTSR_TR13_Msk                           /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos        (14U)                                        
#define EXTI_FTSR_TR14_Msk        (0x1U << EXTI_FTSR_TR14_Pos)                 /*!< 0x00004000 */
#define EXTI_FTSR_TR14            EXTI_FTSR_TR14_Msk                           /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos        (15U)                                        
#define EXTI_FTSR_TR15_Msk        (0x1U << EXTI_FTSR_TR15_Pos)                 /*!< 0x00008000 */
#define EXTI_FTSR_TR15            EXTI_FTSR_TR15_Msk                           /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos        (16U)                                        
#define EXTI_FTSR_TR16_Msk        (0x1U << EXTI_FTSR_TR16_Pos)                 /*!< 0x00010000 */
#define EXTI_FTSR_TR16            EXTI_FTSR_TR16_Msk                           /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos        (17U)                                        
#define EXTI_FTSR_TR17_Msk        (0x1U << EXTI_FTSR_TR17_Pos)                 /*!< 0x00020000 */
#define EXTI_FTSR_TR17            EXTI_FTSR_TR17_Msk                           /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR18_Pos        (18U)                                        
#define EXTI_FTSR_TR18_Msk        (0x1U << EXTI_FTSR_TR18_Pos)                 /*!< 0x00040000 */
#define EXTI_FTSR_TR18            EXTI_FTSR_TR18_Msk                           /*!< Falling trigger event configuration bit of line 18 */
#define EXTI_FTSR_TR19_Pos        (19U)                                        
#define EXTI_FTSR_TR19_Msk        (0x1U << EXTI_FTSR_TR19_Pos)                 /*!< 0x00080000 */
#define EXTI_FTSR_TR19            EXTI_FTSR_TR19_Msk                           /*!< Falling trigger event configuration bit of line 19 */
#define EXTI_FTSR_TR20_Pos        (20U)                                        
#define EXTI_FTSR_TR20_Msk        (0x1U << EXTI_FTSR_TR20_Pos)                 /*!< 0x00100000 */
#define EXTI_FTSR_TR20            EXTI_FTSR_TR20_Msk                           /*!< Falling trigger event configuration bit of line 20 */
#define EXTI_FTSR_TR21_Pos        (21U)                                        
#define EXTI_FTSR_TR21_Msk        (0x1U << EXTI_FTSR_TR21_Pos)                 /*!< 0x00200000 */
#define EXTI_FTSR_TR21            EXTI_FTSR_TR21_Msk                           /*!< Falling trigger event configuration bit of line 21 */
#define EXTI_FTSR_TR22_Pos        (22U)                                        
#define EXTI_FTSR_TR22_Msk        (0x1U << EXTI_FTSR_TR22_Pos)                 /*!< 0x00400000 */
#define EXTI_FTSR_TR22            EXTI_FTSR_TR22_Msk                           /*!< Falling trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)                                         
#define EXTI_SWIER_SWIER0_Msk     (0x1U << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0         EXTI_SWIER_SWIER0_Msk                        /*!< Software Interrupt on line 0 */
#define EXTI_SWIER_SWIER1_Pos     (1U)                                         
#define EXTI_SWIER_SWIER1_Msk     (0x1U << EXTI_SWIER_SWIER1_Pos)              /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1         EXTI_SWIER_SWIER1_Msk                        /*!< Software Interrupt on line 1 */
#define EXTI_SWIER_SWIER2_Pos     (2U)                                         
#define EXTI_SWIER_SWIER2_Msk     (0x1U << EXTI_SWIER_SWIER2_Pos)              /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2         EXTI_SWIER_SWIER2_Msk                        /*!< Software Interrupt on line 2 */
#define EXTI_SWIER_SWIER3_Pos     (3U)                                         
#define EXTI_SWIER_SWIER3_Msk     (0x1U << EXTI_SWIER_SWIER3_Pos)              /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3         EXTI_SWIER_SWIER3_Msk                        /*!< Software Interrupt on line 3 */
#define EXTI_SWIER_SWIER4_Pos     (4U)                                         
#define EXTI_SWIER_SWIER4_Msk     (0x1U << EXTI_SWIER_SWIER4_Pos)              /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4         EXTI_SWIER_SWIER4_Msk                        /*!< Software Interrupt on line 4 */
#define EXTI_SWIER_SWIER5_Pos     (5U)                                         
#define EXTI_SWIER_SWIER5_Msk     (0x1U << EXTI_SWIER_SWIER5_Pos)              /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5         EXTI_SWIER_SWIER5_Msk                        /*!< Software Interrupt on line 5 */
#define EXTI_SWIER_SWIER6_Pos     (6U)                                         
#define EXTI_SWIER_SWIER6_Msk     (0x1U << EXTI_SWIER_SWIER6_Pos)              /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6         EXTI_SWIER_SWIER6_Msk                        /*!< Software Interrupt on line 6 */
#define EXTI_SWIER_SWIER7_Pos     (7U)                                         
#define EXTI_SWIER_SWIER7_Msk     (0x1U << EXTI_SWIER_SWIER7_Pos)              /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7         EXTI_SWIER_SWIER7_Msk                        /*!< Software Interrupt on line 7 */
#define EXTI_SWIER_SWIER8_Pos     (8U)                                         
#define EXTI_SWIER_SWIER8_Msk     (0x1U << EXTI_SWIER_SWIER8_Pos)              /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8         EXTI_SWIER_SWIER8_Msk                        /*!< Software Interrupt on line 8 */
#define EXTI_SWIER_SWIER9_Pos     (9U)                                         
#define EXTI_SWIER_SWIER9_Msk     (0x1U << EXTI_SWIER_SWIER9_Pos)              /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9         EXTI_SWIER_SWIER9_Msk                        /*!< Software Interrupt on line 9 */
#define EXTI_SWIER_SWIER10_Pos    (10U)                                        
#define EXTI_SWIER_SWIER10_Msk    (0x1U << EXTI_SWIER_SWIER10_Pos)             /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10        EXTI_SWIER_SWIER10_Msk                       /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos    (11U)                                        
#define EXTI_SWIER_SWIER11_Msk    (0x1U << EXTI_SWIER_SWIER11_Pos)             /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11        EXTI_SWIER_SWIER11_Msk                       /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos    (12U)                                        
#define EXTI_SWIER_SWIER12_Msk    (0x1U << EXTI_SWIER_SWIER12_Pos)             /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12        EXTI_SWIER_SWIER12_Msk                       /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos    (13U)                                        
#define EXTI_SWIER_SWIER13_Msk    (0x1U << EXTI_SWIER_SWIER13_Pos)             /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13        EXTI_SWIER_SWIER13_Msk                       /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos    (14U)                                        
#define EXTI_SWIER_SWIER14_Msk    (0x1U << EXTI_SWIER_SWIER14_Pos)             /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14        EXTI_SWIER_SWIER14_Msk                       /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos    (15U)                                        
#define EXTI_SWIER_SWIER15_Msk    (0x1U << EXTI_SWIER_SWIER15_Pos)             /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15        EXTI_SWIER_SWIER15_Msk                       /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos    (16U)                                        
#define EXTI_SWIER_SWIER16_Msk    (0x1U << EXTI_SWIER_SWIER16_Pos)             /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16        EXTI_SWIER_SWIER16_Msk                       /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos    (17U)                                        
#define EXTI_SWIER_SWIER17_Msk    (0x1U << EXTI_SWIER_SWIER17_Pos)             /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17        EXTI_SWIER_SWIER17_Msk                       /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER18_Pos    (18U)                                        
#define EXTI_SWIER_SWIER18_Msk    (0x1U << EXTI_SWIER_SWIER18_Pos)             /*!< 0x00040000 */
#define EXTI_SWIER_SWIER18        EXTI_SWIER_SWIER18_Msk                       /*!< Software Interrupt on line 18 */
#define EXTI_SWIER_SWIER19_Pos    (19U)                                        
#define EXTI_SWIER_SWIER19_Msk    (0x1U << EXTI_SWIER_SWIER19_Pos)             /*!< 0x00080000 */
#define EXTI_SWIER_SWIER19        EXTI_SWIER_SWIER19_Msk                       /*!< Software Interrupt on line 19 */
#define EXTI_SWIER_SWIER20_Pos    (20U)                                        
#define EXTI_SWIER_SWIER20_Msk    (0x1U << EXTI_SWIER_SWIER20_Pos)             /*!< 0x00100000 */
#define EXTI_SWIER_SWIER20        EXTI_SWIER_SWIER20_Msk                       /*!< Software Interrupt on line 20 */
#define EXTI_SWIER_SWIER21_Pos    (21U)                                        
#define EXTI_SWIER_SWIER21_Msk    (0x1U << EXTI_SWIER_SWIER21_Pos)             /*!< 0x00200000 */
#define EXTI_SWIER_SWIER21        EXTI_SWIER_SWIER21_Msk                       /*!< Software Interrupt on line 21 */
#define EXTI_SWIER_SWIER22_Pos    (22U)                                        
#define EXTI_SWIER_SWIER22_Msk    (0x1U << EXTI_SWIER_SWIER22_Pos)             /*!< 0x00400000 */
#define EXTI_SWIER_SWIER22        EXTI_SWIER_SWIER22_Msk                       /*!< Software Interrupt on line 22 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define EXTI_PR_PR0_Pos           (0U)                                         
#define EXTI_PR_PR0_Msk           (0x1U << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR0               EXTI_PR_PR0_Msk                              /*!< Pending bit for line 0 */
#define EXTI_PR_PR1_Pos           (1U)                                         
#define EXTI_PR_PR1_Msk           (0x1U << EXTI_PR_PR1_Pos)                    /*!< 0x00000002 */
#define EXTI_PR_PR1               EXTI_PR_PR1_Msk                              /*!< Pending bit for line 1 */
#define EXTI_PR_PR2_Pos           (2U)                                         
#define EXTI_PR_PR2_Msk           (0x1U << EXTI_PR_PR2_Pos)                    /*!< 0x00000004 */
#define EXTI_PR_PR2               EXTI_PR_PR2_Msk                              /*!< Pending bit for line 2 */
#define EXTI_PR_PR3_Pos           (3U)                                         
#define EXTI_PR_PR3_Msk           (0x1U << EXTI_PR_PR3_Pos)                    /*!< 0x00000008 */
#define EXTI_PR_PR3               EXTI_PR_PR3_Msk                              /*!< Pending bit for line 3 */
#define EXTI_PR_PR4_Pos           (4U)                                         
#define EXTI_PR_PR4_Msk           (0x1U << EXTI_PR_PR4_Pos)                    /*!< 0x00000010 */
#define EXTI_PR_PR4               EXTI_PR_PR4_Msk                              /*!< Pending bit for line 4 */
#define EXTI_PR_PR5_Pos           (5U)                                         
#define EXTI_PR_PR5_Msk           (0x1U << EXTI_PR_PR5_Pos)                    /*!< 0x00000020 */
#define EXTI_PR_PR5               EXTI_PR_PR5_Msk                              /*!< Pending bit for line 5 */
#define EXTI_PR_PR6_Pos           (6U)                                         
#define EXTI_PR_PR6_Msk           (0x1U << EXTI_PR_PR6_Pos)                    /*!< 0x00000040 */
#define EXTI_PR_PR6               EXTI_PR_PR6_Msk                              /*!< Pending bit for line 6 */
#define EXTI_PR_PR7_Pos           (7U)                                         
#define EXTI_PR_PR7_Msk           (0x1U << EXTI_PR_PR7_Pos)                    /*!< 0x00000080 */
#define EXTI_PR_PR7               EXTI_PR_PR7_Msk                              /*!< Pending bit for line 7 */
#define EXTI_PR_PR8_Pos           (8U)                                         
#define EXTI_PR_PR8_Msk           (0x1U << EXTI_PR_PR8_Pos)                    /*!< 0x00000100 */
#define EXTI_PR_PR8               EXTI_PR_PR8_Msk                              /*!< Pending bit for line 8 */
#define EXTI_PR_PR9_Pos           (9U)                                         
#define EXTI_PR_PR9_Msk           (0x1U << EXTI_PR_PR9_Pos)                    /*!< 0x00000200 */
#define EXTI_PR_PR9               EXTI_PR_PR9_Msk                              /*!< Pending bit for line 9 */
#define EXTI_PR_PR10_Pos          (10U)                                        
#define EXTI_PR_PR10_Msk          (0x1U << EXTI_PR_PR10_Pos)                   /*!< 0x00000400 */
#define EXTI_PR_PR10              EXTI_PR_PR10_Msk                             /*!< Pending bit for line 10 */
#define EXTI_PR_PR11_Pos          (11U)                                        
#define EXTI_PR_PR11_Msk          (0x1U << EXTI_PR_PR11_Pos)                   /*!< 0x00000800 */
#define EXTI_PR_PR11              EXTI_PR_PR11_Msk                             /*!< Pending bit for line 11 */
#define EXTI_PR_PR12_Pos          (12U)                                        
#define EXTI_PR_PR12_Msk          (0x1U << EXTI_PR_PR12_Pos)                   /*!< 0x00001000 */
#define EXTI_PR_PR12              EXTI_PR_PR12_Msk                             /*!< Pending bit for line 12 */
#define EXTI_PR_PR13_Pos          (13U)                                        
#define EXTI_PR_PR13_Msk          (0x1U << EXTI_PR_PR13_Pos)                   /*!< 0x00002000 */
#define EXTI_PR_PR13              EXTI_PR_PR13_Msk                             /*!< Pending bit for line 13 */
#define EXTI_PR_PR14_Pos          (14U)                                        
#define EXTI_PR_PR14_Msk          (0x1U << EXTI_PR_PR14_Pos)                   /*!< 0x00004000 */
#define EXTI_PR_PR14              EXTI_PR_PR14_Msk                             /*!< Pending bit for line 14 */
#define EXTI_PR_PR15_Pos          (15U)                                        
#define EXTI_PR_PR15_Msk          (0x1U << EXTI_PR_PR15_Pos)                   /*!< 0x00008000 */
#define EXTI_PR_PR15              EXTI_PR_PR15_Msk                             /*!< Pending bit for line 15 */
#define EXTI_PR_PR16_Pos          (16U)                                        
#define EXTI_PR_PR16_Msk          (0x1U << EXTI_PR_PR16_Pos)                   /*!< 0x00010000 */
#define EXTI_PR_PR16              EXTI_PR_PR16_Msk                             /*!< Pending bit for line 16 */
#define EXTI_PR_PR17_Pos          (17U)                                        
#define EXTI_PR_PR17_Msk          (0x1U << EXTI_PR_PR17_Pos)                   /*!< 0x00020000 */
#define EXTI_PR_PR17              EXTI_PR_PR17_Msk                             /*!< Pending bit for line 17 */
#define EXTI_PR_PR18_Pos          (18U)                                        
#define EXTI_PR_PR18_Msk          (0x1U << EXTI_PR_PR18_Pos)                   /*!< 0x00040000 */
#define EXTI_PR_PR18              EXTI_PR_PR18_Msk                             /*!< Pending bit for line 18 */
#define EXTI_PR_PR19_Pos          (19U)                                        
#define EXTI_PR_PR19_Msk          (0x1U << EXTI_PR_PR19_Pos)                   /*!< 0x00080000 */
#define EXTI_PR_PR19              EXTI_PR_PR19_Msk                             /*!< Pending bit for line 19 */
#define EXTI_PR_PR20_Pos          (20U)                                        
#define EXTI_PR_PR20_Msk          (0x1U << EXTI_PR_PR20_Pos)                   /*!< 0x00100000 */
#define EXTI_PR_PR20              EXTI_PR_PR20_Msk                             /*!< Pending bit for line 20 */
#define EXTI_PR_PR21_Pos          (21U)                                        
#define EXTI_PR_PR21_Msk          (0x1U << EXTI_PR_PR21_Pos)                   /*!< 0x00200000 */
#define EXTI_PR_PR21              EXTI_PR_PR21_Msk                             /*!< Pending bit for line 21 */
#define EXTI_PR_PR22_Pos          (22U)                                        
#define EXTI_PR_PR22_Msk          (0x1U << EXTI_PR_PR22_Pos)                   /*!< 0x00400000 */
#define EXTI_PR_PR22              EXTI_PR_PR22_Msk                             /*!< Pending bit for line 22 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY_Pos          (0U)                                    
#define FLASH_ACR_LATENCY_Msk          (0xFU << FLASH_ACR_LATENCY_Pos)         /*!< 0x0000000F */
#define FLASH_ACR_LATENCY              FLASH_ACR_LATENCY_Msk                   
#define FLASH_ACR_LATENCY_0WS          0x00000000U                             
#define FLASH_ACR_LATENCY_1WS          0x00000001U                             
#define FLASH_ACR_LATENCY_2WS          0x00000002U                             
#define FLASH_ACR_LATENCY_3WS          0x00000003U                             
#define FLASH_ACR_LATENCY_4WS          0x00000004U                             
#define FLASH_ACR_LATENCY_5WS          0x00000005U                             
#define FLASH_ACR_LATENCY_6WS          0x00000006U                             
#define FLASH_ACR_LATENCY_7WS          0x00000007U                             

#define FLASH_ACR_PRFTEN_Pos           (8U)                                    
#define FLASH_ACR_PRFTEN_Msk           (0x1U << FLASH_ACR_PRFTEN_Pos)          /*!< 0x00000100 */
#define FLASH_ACR_PRFTEN               FLASH_ACR_PRFTEN_Msk                    
#define FLASH_ACR_ICEN_Pos             (9U)                                    
#define FLASH_ACR_ICEN_Msk             (0x1U << FLASH_ACR_ICEN_Pos)            /*!< 0x00000200 */
#define FLASH_ACR_ICEN                 FLASH_ACR_ICEN_Msk                      
#define FLASH_ACR_DCEN_Pos             (10U)                                   
#define FLASH_ACR_DCEN_Msk             (0x1U << FLASH_ACR_DCEN_Pos)            /*!< 0x00000400 */
#define FLASH_ACR_DCEN                 FLASH_ACR_DCEN_Msk                      
#define FLASH_ACR_ICRST_Pos            (11U)                                   
#define FLASH_ACR_ICRST_Msk            (0x1U << FLASH_ACR_ICRST_Pos)           /*!< 0x00000800 */
#define FLASH_ACR_ICRST                FLASH_ACR_ICRST_Msk                     
#define FLASH_ACR_DCRST_Pos            (12U)                                   
#define FLASH_ACR_DCRST_Msk            (0x1U << FLASH_ACR_DCRST_Pos)           /*!< 0x00001000 */
#define FLASH_ACR_DCRST                FLASH_ACR_DCRST_Msk                     
#define FLASH_ACR_BYTE0_ADDRESS_Pos    (10U)                                   
#define FLASH_ACR_BYTE0_ADDRESS_Msk    (0x10008FU << FLASH_ACR_BYTE0_ADDRESS_Pos) /*!< 0x40023C00 */
#define FLASH_ACR_BYTE0_ADDRESS        FLASH_ACR_BYTE0_ADDRESS_Msk             
#define FLASH_ACR_BYTE2_ADDRESS_Pos    (0U)                                    
#define FLASH_ACR_BYTE2_ADDRESS_Msk    (0x40023C03U << FLASH_ACR_BYTE2_ADDRESS_Pos) /*!< 0x40023C03 */
#define FLASH_ACR_BYTE2_ADDRESS        FLASH_ACR_BYTE2_ADDRESS_Msk             

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP_Pos               (0U)                                    
#define FLASH_SR_EOP_Msk               (0x1U << FLASH_SR_EOP_Pos)              /*!< 0x00000001 */
#define FLASH_SR_EOP                   FLASH_SR_EOP_Msk                        
#define FLASH_SR_SOP_Pos               (1U)                                    
#define FLASH_SR_SOP_Msk               (0x1U << FLASH_SR_SOP_Pos)              /*!< 0x00000002 */
#define FLASH_SR_SOP                   FLASH_SR_SOP_Msk                        
#define FLASH_SR_WRPERR_Pos            (4U)                                    
#define FLASH_SR_WRPERR_Msk            (0x1U << FLASH_SR_WRPERR_Pos)           /*!< 0x00000010 */
#define FLASH_SR_WRPERR                FLASH_SR_WRPERR_Msk                     
#define FLASH_SR_PGAERR_Pos            (5U)                                    
#define FLASH_SR_PGAERR_Msk            (0x1U << FLASH_SR_PGAERR_Pos)           /*!< 0x00000020 */
#define FLASH_SR_PGAERR                FLASH_SR_PGAERR_Msk                     
#define FLASH_SR_PGPERR_Pos            (6U)                                    
#define FLASH_SR_PGPERR_Msk            (0x1U << FLASH_SR_PGPERR_Pos)           /*!< 0x00000040 */
#define FLASH_SR_PGPERR                FLASH_SR_PGPERR_Msk                     
#define FLASH_SR_PGSERR_Pos            (7U)                                    
#define FLASH_SR_PGSERR_Msk            (0x1U << FLASH_SR_PGSERR_Pos)           /*!< 0x00000080 */
#define FLASH_SR_PGSERR                FLASH_SR_PGSERR_Msk                     
#define FLASH_SR_RDERR_Pos            (8U)                                    
#define FLASH_SR_RDERR_Msk            (0x1U << FLASH_SR_RDERR_Pos)             /*!< 0x00000100 */
#define FLASH_SR_RDERR                FLASH_SR_RDERR_Msk                     
#define FLASH_SR_BSY_Pos               (16U)                                   
#define FLASH_SR_BSY_Msk               (0x1U << FLASH_SR_BSY_Pos)              /*!< 0x00010000 */
#define FLASH_SR_BSY                   FLASH_SR_BSY_Msk                        

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG_Pos                (0U)                                    
#define FLASH_CR_PG_Msk                (0x1U << FLASH_CR_PG_Pos)               /*!< 0x00000001 */
#define FLASH_CR_PG                    FLASH_CR_PG_Msk                         
#define FLASH_CR_SER_Pos               (1U)                                    
#define FLASH_CR_SER_Msk               (0x1U << FLASH_CR_SER_Pos)              /*!< 0x00000002 */
#define FLASH_CR_SER                   FLASH_CR_SER_Msk                        
#define FLASH_CR_MER_Pos               (2U)                                    
#define FLASH_CR_MER_Msk               (0x1U << FLASH_CR_MER_Pos)              /*!< 0x00000004 */
#define FLASH_CR_MER                   FLASH_CR_MER_Msk                        
#define FLASH_CR_SNB_Pos               (3U)                                    
#define FLASH_CR_SNB_Msk               (0x1FU << FLASH_CR_SNB_Pos)             /*!< 0x000000F8 */
#define FLASH_CR_SNB                   FLASH_CR_SNB_Msk                        
#define FLASH_CR_SNB_0                 (0x01U << FLASH_CR_SNB_Pos)             /*!< 0x00000008 */
#define FLASH_CR_SNB_1                 (0x02U << FLASH_CR_SNB_Pos)             /*!< 0x00000010 */
#define FLASH_CR_SNB_2                 (0x04U << FLASH_CR_SNB_Pos)             /*!< 0x00000020 */
#define FLASH_CR_SNB_3                 (0x08U << FLASH_CR_SNB_Pos)             /*!< 0x00000040 */
#define FLASH_CR_SNB_4                 (0x10U << FLASH_CR_SNB_Pos)             /*!< 0x00000080 */
#define FLASH_CR_PSIZE_Pos             (8U)                                    
#define FLASH_CR_PSIZE_Msk             (0x3U << FLASH_CR_PSIZE_Pos)            /*!< 0x00000300 */
#define FLASH_CR_PSIZE                 FLASH_CR_PSIZE_Msk                      
#define FLASH_CR_PSIZE_0               (0x1U << FLASH_CR_PSIZE_Pos)            /*!< 0x00000100 */
#define FLASH_CR_PSIZE_1               (0x2U << FLASH_CR_PSIZE_Pos)            /*!< 0x00000200 */
#define FLASH_CR_STRT_Pos              (16U)                                   
#define FLASH_CR_STRT_Msk              (0x1U << FLASH_CR_STRT_Pos)             /*!< 0x00010000 */
#define FLASH_CR_STRT                  FLASH_CR_STRT_Msk                       
#define FLASH_CR_EOPIE_Pos             (24U)                                   
#define FLASH_CR_EOPIE_Msk             (0x1U << FLASH_CR_EOPIE_Pos)            /*!< 0x01000000 */
#define FLASH_CR_EOPIE                 FLASH_CR_EOPIE_Msk                      
#define FLASH_CR_LOCK_Pos              (31U)                                   
#define FLASH_CR_LOCK_Msk              (0x1U << FLASH_CR_LOCK_Pos)             /*!< 0x80000000 */
#define FLASH_CR_LOCK                  FLASH_CR_LOCK_Msk                       

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK_Pos        (0U)                                    
#define FLASH_OPTCR_OPTLOCK_Msk        (0x1U << FLASH_OPTCR_OPTLOCK_Pos)       /*!< 0x00000001 */
#define FLASH_OPTCR_OPTLOCK            FLASH_OPTCR_OPTLOCK_Msk                 
#define FLASH_OPTCR_OPTSTRT_Pos        (1U)                                    
#define FLASH_OPTCR_OPTSTRT_Msk        (0x1U << FLASH_OPTCR_OPTSTRT_Pos)       /*!< 0x00000002 */
#define FLASH_OPTCR_OPTSTRT            FLASH_OPTCR_OPTSTRT_Msk                 

#define FLASH_OPTCR_BOR_LEV_0          0x00000004U                             
#define FLASH_OPTCR_BOR_LEV_1          0x00000008U                             
#define FLASH_OPTCR_BOR_LEV_Pos        (2U)                                    
#define FLASH_OPTCR_BOR_LEV_Msk        (0x3U << FLASH_OPTCR_BOR_LEV_Pos)       /*!< 0x0000000C */
#define FLASH_OPTCR_BOR_LEV            FLASH_OPTCR_BOR_LEV_Msk                 
#define FLASH_OPTCR_WDG_SW_Pos         (5U)                                    
#define FLASH_OPTCR_WDG_SW_Msk         (0x1U << FLASH_OPTCR_WDG_SW_Pos)        /*!< 0x00000020 */
#define FLASH_OPTCR_WDG_SW             FLASH_OPTCR_WDG_SW_Msk                  
#define FLASH_OPTCR_nRST_STOP_Pos      (6U)                                    
#define FLASH_OPTCR_nRST_STOP_Msk      (0x1U << FLASH_OPTCR_nRST_STOP_Pos)     /*!< 0x00000040 */
#define FLASH_OPTCR_nRST_STOP          FLASH_OPTCR_nRST_STOP_Msk               
#define FLASH_OPTCR_nRST_STDBY_Pos     (7U)                                    
#define FLASH_OPTCR_nRST_STDBY_Msk     (0x1U << FLASH_OPTCR_nRST_STDBY_Pos)    /*!< 0x00000080 */
#define FLASH_OPTCR_nRST_STDBY         FLASH_OPTCR_nRST_STDBY_Msk              
#define FLASH_OPTCR_RDP_Pos            (8U)                                    
#define FLASH_OPTCR_RDP_Msk            (0xFFU << FLASH_OPTCR_RDP_Pos)          /*!< 0x0000FF00 */
#define FLASH_OPTCR_RDP                FLASH_OPTCR_RDP_Msk                     
#define FLASH_OPTCR_RDP_0              (0x01U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000100 */
#define FLASH_OPTCR_RDP_1              (0x02U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000200 */
#define FLASH_OPTCR_RDP_2              (0x04U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000400 */
#define FLASH_OPTCR_RDP_3              (0x08U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00000800 */
#define FLASH_OPTCR_RDP_4              (0x10U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00001000 */
#define FLASH_OPTCR_RDP_5              (0x20U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00002000 */
#define FLASH_OPTCR_RDP_6              (0x40U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00004000 */
#define FLASH_OPTCR_RDP_7              (0x80U << FLASH_OPTCR_RDP_Pos)          /*!< 0x00008000 */
#define FLASH_OPTCR_nWRP_Pos           (16U)                                   
#define FLASH_OPTCR_nWRP_Msk           (0xFFFU << FLASH_OPTCR_nWRP_Pos)        /*!< 0x0FFF0000 */
#define FLASH_OPTCR_nWRP               FLASH_OPTCR_nWRP_Msk                    
#define FLASH_OPTCR_nWRP_0             0x00010000U                             
#define FLASH_OPTCR_nWRP_1             0x00020000U                             
#define FLASH_OPTCR_nWRP_2             0x00040000U                             
#define FLASH_OPTCR_nWRP_3             0x00080000U                             
#define FLASH_OPTCR_nWRP_4             0x00100000U                             
#define FLASH_OPTCR_nWRP_5             0x00200000U                             
#define FLASH_OPTCR_nWRP_6             0x00400000U                             
#define FLASH_OPTCR_nWRP_7             0x00800000U                             
#define FLASH_OPTCR_nWRP_8             0x01000000U                             
#define FLASH_OPTCR_nWRP_9             0x02000000U                             
#define FLASH_OPTCR_nWRP_10            0x04000000U                             
#define FLASH_OPTCR_nWRP_11            0x08000000U                             
                                             
/******************  Bits definition for FLASH_OPTCR1 register  ***************/
#define FLASH_OPTCR1_nWRP_Pos          (16U)                                   
#define FLASH_OPTCR1_nWRP_Msk          (0xFFFU << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x0FFF0000 */
#define FLASH_OPTCR1_nWRP              FLASH_OPTCR1_nWRP_Msk                   
#define FLASH_OPTCR1_nWRP_0            (0x001U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00010000 */
#define FLASH_OPTCR1_nWRP_1            (0x002U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00020000 */
#define FLASH_OPTCR1_nWRP_2            (0x004U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00040000 */
#define FLASH_OPTCR1_nWRP_3            (0x008U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00080000 */
#define FLASH_OPTCR1_nWRP_4            (0x010U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00100000 */
#define FLASH_OPTCR1_nWRP_5            (0x020U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00200000 */
#define FLASH_OPTCR1_nWRP_6            (0x040U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00400000 */
#define FLASH_OPTCR1_nWRP_7            (0x080U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x00800000 */
#define FLASH_OPTCR1_nWRP_8            (0x100U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x01000000 */
#define FLASH_OPTCR1_nWRP_9            (0x200U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x02000000 */
#define FLASH_OPTCR1_nWRP_10           (0x400U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x04000000 */
#define FLASH_OPTCR1_nWRP_11           (0x800U << FLASH_OPTCR1_nWRP_Pos)       /*!< 0x08000000 */

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODE0_Pos             (0U)                                  
#define GPIO_MODER_MODE0_Msk             (0x3U << GPIO_MODER_MODE0_Pos)        /*!< 0x00000003 */
#define GPIO_MODER_MODE0                 GPIO_MODER_MODE0_Msk                  
#define GPIO_MODER_MODE0_0               (0x1U << GPIO_MODER_MODE0_Pos)        /*!< 0x00000001 */
#define GPIO_MODER_MODE0_1               (0x2U << GPIO_MODER_MODE0_Pos)        /*!< 0x00000002 */
#define GPIO_MODER_MODE1_Pos             (2U)                                  
#define GPIO_MODER_MODE1_Msk             (0x3U << GPIO_MODER_MODE1_Pos)        /*!< 0x0000000C */
#define GPIO_MODER_MODE1                 GPIO_MODER_MODE1_Msk                  
#define GPIO_MODER_MODE1_0               (0x1U << GPIO_MODER_MODE1_Pos)        /*!< 0x00000004 */
#define GPIO_MODER_MODE1_1               (0x2U << GPIO_MODER_MODE1_Pos)        /*!< 0x00000008 */
#define GPIO_MODER_MODE2_Pos             (4U)                                  
#define GPIO_MODER_MODE2_Msk             (0x3U << GPIO_MODER_MODE2_Pos)        /*!< 0x00000030 */
#define GPIO_MODER_MODE2                 GPIO_MODER_MODE2_Msk                  
#define GPIO_MODER_MODE2_0               (0x1U << GPIO_MODER_MODE2_Pos)        /*!< 0x00000010 */
#define GPIO_MODER_MODE2_1               (0x2U << GPIO_MODER_MODE2_Pos)        /*!< 0x00000020 */
#define GPIO_MODER_MODE3_Pos             (6U)                                  
#define GPIO_MODER_MODE3_Msk             (0x3U << GPIO_MODER_MODE3_Pos)        /*!< 0x000000C0 */
#define GPIO_MODER_MODE3                 GPIO_MODER_MODE3_Msk                  
#define GPIO_MODER_MODE3_0               (0x1U << GPIO_MODER_MODE3_Pos)        /*!< 0x00000040 */
#define GPIO_MODER_MODE3_1               (0x2U << GPIO_MODER_MODE3_Pos)        /*!< 0x00000080 */
#define GPIO_MODER_MODE4_Pos             (8U)                                  
#define GPIO_MODER_MODE4_Msk             (0x3U << GPIO_MODER_MODE4_Pos)        /*!< 0x00000300 */
#define GPIO_MODER_MODE4                 GPIO_MODER_MODE4_Msk                  
#define GPIO_MODER_MODE4_0               (0x1U << GPIO_MODER_MODE4_Pos)        /*!< 0x00000100 */
#define GPIO_MODER_MODE4_1               (0x2U << GPIO_MODER_MODE4_Pos)        /*!< 0x00000200 */
#define GPIO_MODER_MODE5_Pos             (10U)                                 
#define GPIO_MODER_MODE5_Msk             (0x3U << GPIO_MODER_MODE5_Pos)        /*!< 0x00000C00 */
#define GPIO_MODER_MODE5                 GPIO_MODER_MODE5_Msk                  
#define GPIO_MODER_MODE5_0               (0x1U << GPIO_MODER_MODE5_Pos)        /*!< 0x00000400 */
#define GPIO_MODER_MODE5_1               (0x2U << GPIO_MODER_MODE5_Pos)        /*!< 0x00000800 */
#define GPIO_MODER_MODE6_Pos             (12U)                                 
#define GPIO_MODER_MODE6_Msk             (0x3U << GPIO_MODER_MODE6_Pos)        /*!< 0x00003000 */
#define GPIO_MODER_MODE6                 GPIO_MODER_MODE6_Msk                  
#define GPIO_MODER_MODE6_0               (0x1U << GPIO_MODER_MODE6_Pos)        /*!< 0x00001000 */
#define GPIO_MODER_MODE6_1               (0x2U << GPIO_MODER_MODE6_Pos)        /*!< 0x00002000 */
#define GPIO_MODER_MODE7_Pos             (14U)                                 
#define GPIO_MODER_MODE7_Msk             (0x3U << GPIO_MODER_MODE7_Pos)        /*!< 0x0000C000 */
#define GPIO_MODER_MODE7                 GPIO_MODER_MODE7_Msk                  
#define GPIO_MODER_MODE7_0               (0x1U << GPIO_MODER_MODE7_Pos)        /*!< 0x00004000 */
#define GPIO_MODER_MODE7_1               (0x2U << GPIO_MODER_MODE7_Pos)        /*!< 0x00008000 */
#define GPIO_MODER_MODE8_Pos             (16U)                                 
#define GPIO_MODER_MODE8_Msk             (0x3U << GPIO_MODER_MODE8_Pos)        /*!< 0x00030000 */
#define GPIO_MODER_MODE8                 GPIO_MODER_MODE8_Msk                  
#define GPIO_MODER_MODE8_0               (0x1U << GPIO_MODER_MODE8_Pos)        /*!< 0x00010000 */
#define GPIO_MODER_MODE8_1               (0x2U << GPIO_MODER_MODE8_Pos)        /*!< 0x00020000 */
#define GPIO_MODER_MODE9_Pos             (18U)                                 
#define GPIO_MODER_MODE9_Msk             (0x3U << GPIO_MODER_MODE9_Pos)        /*!< 0x000C0000 */
#define GPIO_MODER_MODE9                 GPIO_MODER_MODE9_Msk                  
#define GPIO_MODER_MODE9_0               (0x1U << GPIO_MODER_MODE9_Pos)        /*!< 0x00040000 */
#define GPIO_MODER_MODE9_1               (0x2U << GPIO_MODER_MODE9_Pos)        /*!< 0x00080000 */
#define GPIO_MODER_MODE10_Pos            (20U)                                 
#define GPIO_MODER_MODE10_Msk            (0x3U << GPIO_MODER_MODE10_Pos)       /*!< 0x00300000 */
#define GPIO_MODER_MODE10                GPIO_MODER_MODE10_Msk                 
#define GPIO_MODER_MODE10_0              (0x1U << GPIO_MODER_MODE10_Pos)       /*!< 0x00100000 */
#define GPIO_MODER_MODE10_1              (0x2U << GPIO_MODER_MODE10_Pos)       /*!< 0x00200000 */
#define GPIO_MODER_MODE11_Pos            (22U)                                 
#define GPIO_MODER_MODE11_Msk            (0x3U << GPIO_MODER_MODE11_Pos)       /*!< 0x00C00000 */
#define GPIO_MODER_MODE11                GPIO_MODER_MODE11_Msk                 
#define GPIO_MODER_MODE11_0              (0x1U << GPIO_MODER_MODE11_Pos)       /*!< 0x00400000 */
#define GPIO_MODER_MODE11_1              (0x2U << GPIO_MODER_MODE11_Pos)       /*!< 0x00800000 */
#define GPIO_MODER_MODE12_Pos            (24U)                                 
#define GPIO_MODER_MODE12_Msk            (0x3U << GPIO_MODER_MODE12_Pos)       /*!< 0x03000000 */
#define GPIO_MODER_MODE12                GPIO_MODER_MODE12_Msk                 
#define GPIO_MODER_MODE12_0              (0x1U << GPIO_MODER_MODE12_Pos)       /*!< 0x01000000 */
#define GPIO_MODER_MODE12_1              (0x2U << GPIO_MODER_MODE12_Pos)       /*!< 0x02000000 */
#define GPIO_MODER_MODE13_Pos            (26U)                                 
#define GPIO_MODER_MODE13_Msk            (0x3U << GPIO_MODER_MODE13_Pos)       /*!< 0x0C000000 */
#define GPIO_MODER_MODE13                GPIO_MODER_MODE13_Msk                 
#define GPIO_MODER_MODE13_0              (0x1U << GPIO_MODER_MODE13_Pos)       /*!< 0x04000000 */
#define GPIO_MODER_MODE13_1              (0x2U << GPIO_MODER_MODE13_Pos)       /*!< 0x08000000 */
#define GPIO_MODER_MODE14_Pos            (28U)                                 
#define GPIO_MODER_MODE14_Msk            (0x3U << GPIO_MODER_MODE14_Pos)       /*!< 0x30000000 */
#define GPIO_MODER_MODE14                GPIO_MODER_MODE14_Msk                 
#define GPIO_MODER_MODE14_0              (0x1U << GPIO_MODER_MODE14_Pos)       /*!< 0x10000000 */
#define GPIO_MODER_MODE14_1              (0x2U << GPIO_MODER_MODE14_Pos)       /*!< 0x20000000 */
#define GPIO_MODER_MODE15_Pos            (30U)                                 
#define GPIO_MODER_MODE15_Msk            (0x3U << GPIO_MODER_MODE15_Pos)       /*!< 0xC0000000 */
#define GPIO_MODER_MODE15                GPIO_MODER_MODE15_Msk                 
#define GPIO_MODER_MODE15_0              (0x1U << GPIO_MODER_MODE15_Pos)       /*!< 0x40000000 */
#define GPIO_MODER_MODE15_1              (0x2U << GPIO_MODER_MODE15_Pos)       /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_MODER_MODER0_Pos            (0U)                                  
#define GPIO_MODER_MODER0_Msk            (0x3U << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0                GPIO_MODER_MODER0_Msk                 
#define GPIO_MODER_MODER0_0              (0x1U << GPIO_MODER_MODER0_Pos)       /*!< 0x00000001 */
#define GPIO_MODER_MODER0_1              (0x2U << GPIO_MODER_MODER0_Pos)       /*!< 0x00000002 */
#define GPIO_MODER_MODER1_Pos            (2U)                                  
#define GPIO_MODER_MODER1_Msk            (0x3U << GPIO_MODER_MODER1_Pos)       /*!< 0x0000000C */
#define GPIO_MODER_MODER1                GPIO_MODER_MODER1_Msk                 
#define GPIO_MODER_MODER1_0              (0x1U << GPIO_MODER_MODER1_Pos)       /*!< 0x00000004 */
#define GPIO_MODER_MODER1_1              (0x2U << GPIO_MODER_MODER1_Pos)       /*!< 0x00000008 */
#define GPIO_MODER_MODER2_Pos            (4U)                                  
#define GPIO_MODER_MODER2_Msk            (0x3U << GPIO_MODER_MODER2_Pos)       /*!< 0x00000030 */
#define GPIO_MODER_MODER2                GPIO_MODER_MODER2_Msk                 
#define GPIO_MODER_MODER2_0              (0x1U << GPIO_MODER_MODER2_Pos)       /*!< 0x00000010 */
#define GPIO_MODER_MODER2_1              (0x2U << GPIO_MODER_MODER2_Pos)       /*!< 0x00000020 */
#define GPIO_MODER_MODER3_Pos            (6U)                                  
#define GPIO_MODER_MODER3_Msk            (0x3U << GPIO_MODER_MODER3_Pos)       /*!< 0x000000C0 */
#define GPIO_MODER_MODER3                GPIO_MODER_MODER3_Msk                 
#define GPIO_MODER_MODER3_0              (0x1U << GPIO_MODER_MODER3_Pos)       /*!< 0x00000040 */
#define GPIO_MODER_MODER3_1              (0x2U << GPIO_MODER_MODER3_Pos)       /*!< 0x00000080 */
#define GPIO_MODER_MODER4_Pos            (8U)                                  
#define GPIO_MODER_MODER4_Msk            (0x3U << GPIO_MODER_MODER4_Pos)       /*!< 0x00000300 */
#define GPIO_MODER_MODER4                GPIO_MODER_MODER4_Msk                 
#define GPIO_MODER_MODER4_0              (0x1U << GPIO_MODER_MODER4_Pos)       /*!< 0x00000100 */
#define GPIO_MODER_MODER4_1              (0x2U << GPIO_MODER_MODER4_Pos)       /*!< 0x00000200 */
#define GPIO_MODER_MODER5_Pos            (10U)                                 
#define GPIO_MODER_MODER5_Msk            (0x3U << GPIO_MODER_MODER5_Pos)       /*!< 0x00000C00 */
#define GPIO_MODER_MODER5                GPIO_MODER_MODER5_Msk                 
#define GPIO_MODER_MODER5_0              (0x1U << GPIO_MODER_MODER5_Pos)       /*!< 0x00000400 */
#define GPIO_MODER_MODER5_1              (0x2U << GPIO_MODER_MODER5_Pos)       /*!< 0x00000800 */
#define GPIO_MODER_MODER6_Pos            (12U)                                 
#define GPIO_MODER_MODER6_Msk            (0x3U << GPIO_MODER_MODER6_Pos)       /*!< 0x00003000 */
#define GPIO_MODER_MODER6                GPIO_MODER_MODER6_Msk                 
#define GPIO_MODER_MODER6_0              (0x1U << GPIO_MODER_MODER6_Pos)       /*!< 0x00001000 */
#define GPIO_MODER_MODER6_1              (0x2U << GPIO_MODER_MODER6_Pos)       /*!< 0x00002000 */
#define GPIO_MODER_MODER7_Pos            (14U)                                 
#define GPIO_MODER_MODER7_Msk            (0x3U << GPIO_MODER_MODER7_Pos)       /*!< 0x0000C000 */
#define GPIO_MODER_MODER7                GPIO_MODER_MODER7_Msk                 
#define GPIO_MODER_MODER7_0              (0x1U << GPIO_MODER_MODER7_Pos)       /*!< 0x00004000 */
#define GPIO_MODER_MODER7_1              (0x2U << GPIO_MODER_MODER7_Pos)       /*!< 0x00008000 */
#define GPIO_MODER_MODER8_Pos            (16U)                                 
#define GPIO_MODER_MODER8_Msk            (0x3U << GPIO_MODER_MODER8_Pos)       /*!< 0x00030000 */
#define GPIO_MODER_MODER8                GPIO_MODER_MODER8_Msk                 
#define GPIO_MODER_MODER8_0              (0x1U << GPIO_MODER_MODER8_Pos)       /*!< 0x00010000 */
#define GPIO_MODER_MODER8_1              (0x2U << GPIO_MODER_MODER8_Pos)       /*!< 0x00020000 */
#define GPIO_MODER_MODER9_Pos            (18U)                                 
#define GPIO_MODER_MODER9_Msk            (0x3U << GPIO_MODER_MODER9_Pos)       /*!< 0x000C0000 */
#define GPIO_MODER_MODER9                GPIO_MODER_MODER9_Msk                 
#define GPIO_MODER_MODER9_0              (0x1U << GPIO_MODER_MODER9_Pos)       /*!< 0x00040000 */
#define GPIO_MODER_MODER9_1              (0x2U << GPIO_MODER_MODER9_Pos)       /*!< 0x00080000 */
#define GPIO_MODER_MODER10_Pos           (20U)                                 
#define GPIO_MODER_MODER10_Msk           (0x3U << GPIO_MODER_MODER10_Pos)      /*!< 0x00300000 */
#define GPIO_MODER_MODER10               GPIO_MODER_MODER10_Msk                
#define GPIO_MODER_MODER10_0             (0x1U << GPIO_MODER_MODER10_Pos)      /*!< 0x00100000 */
#define GPIO_MODER_MODER10_1             (0x2U << GPIO_MODER_MODER10_Pos)      /*!< 0x00200000 */
#define GPIO_MODER_MODER11_Pos           (22U)                                 
#define GPIO_MODER_MODER11_Msk           (0x3U << GPIO_MODER_MODER11_Pos)      /*!< 0x00C00000 */
#define GPIO_MODER_MODER11               GPIO_MODER_MODER11_Msk                
#define GPIO_MODER_MODER11_0             (0x1U << GPIO_MODER_MODER11_Pos)      /*!< 0x00400000 */
#define GPIO_MODER_MODER11_1             (0x2U << GPIO_MODER_MODER11_Pos)      /*!< 0x00800000 */
#define GPIO_MODER_MODER12_Pos           (24U)                                 
#define GPIO_MODER_MODER12_Msk           (0x3U << GPIO_MODER_MODER12_Pos)      /*!< 0x03000000 */
#define GPIO_MODER_MODER12               GPIO_MODER_MODER12_Msk                
#define GPIO_MODER_MODER12_0             (0x1U << GPIO_MODER_MODER12_Pos)      /*!< 0x01000000 */
#define GPIO_MODER_MODER12_1             (0x2U << GPIO_MODER_MODER12_Pos)      /*!< 0x02000000 */
#define GPIO_MODER_MODER13_Pos           (26U)                                 
#define GPIO_MODER_MODER13_Msk           (0x3U << GPIO_MODER_MODER13_Pos)      /*!< 0x0C000000 */
#define GPIO_MODER_MODER13               GPIO_MODER_MODER13_Msk                
#define GPIO_MODER_MODER13_0             (0x1U << GPIO_MODER_MODER13_Pos)      /*!< 0x04000000 */
#define GPIO_MODER_MODER13_1             (0x2U << GPIO_MODER_MODER13_Pos)      /*!< 0x08000000 */
#define GPIO_MODER_MODER14_Pos           (28U)                                 
#define GPIO_MODER_MODER14_Msk           (0x3U << GPIO_MODER_MODER14_Pos)      /*!< 0x30000000 */
#define GPIO_MODER_MODER14               GPIO_MODER_MODER14_Msk                
#define GPIO_MODER_MODER14_0             (0x1U << GPIO_MODER_MODER14_Pos)      /*!< 0x10000000 */
#define GPIO_MODER_MODER14_1             (0x2U << GPIO_MODER_MODER14_Pos)      /*!< 0x20000000 */
#define GPIO_MODER_MODER15_Pos           (30U)                                 
#define GPIO_MODER_MODER15_Msk           (0x3U << GPIO_MODER_MODER15_Pos)      /*!< 0xC0000000 */
#define GPIO_MODER_MODER15               GPIO_MODER_MODER15_Msk                
#define GPIO_MODER_MODER15_0             (0x1U << GPIO_MODER_MODER15_Pos)      /*!< 0x40000000 */
#define GPIO_MODER_MODER15_1             (0x2U << GPIO_MODER_MODER15_Pos)      /*!< 0x80000000 */

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos              (0U)                                  
#define GPIO_OTYPER_OT0_Msk              (0x1U << GPIO_OTYPER_OT0_Pos)         /*!< 0x00000001 */
#define GPIO_OTYPER_OT0                  GPIO_OTYPER_OT0_Msk                   
#define GPIO_OTYPER_OT1_Pos              (1U)                                  
#define GPIO_OTYPER_OT1_Msk              (0x1U << GPIO_OTYPER_OT1_Pos)         /*!< 0x00000002 */
#define GPIO_OTYPER_OT1                  GPIO_OTYPER_OT1_Msk                   
#define GPIO_OTYPER_OT2_Pos              (2U)                                  
#define GPIO_OTYPER_OT2_Msk              (0x1U << GPIO_OTYPER_OT2_Pos)         /*!< 0x00000004 */
#define GPIO_OTYPER_OT2                  GPIO_OTYPER_OT2_Msk                   
#define GPIO_OTYPER_OT3_Pos              (3U)                                  
#define GPIO_OTYPER_OT3_Msk              (0x1U << GPIO_OTYPER_OT3_Pos)         /*!< 0x00000008 */
#define GPIO_OTYPER_OT3                  GPIO_OTYPER_OT3_Msk                   
#define GPIO_OTYPER_OT4_Pos              (4U)                                  
#define GPIO_OTYPER_OT4_Msk              (0x1U << GPIO_OTYPER_OT4_Pos)         /*!< 0x00000010 */
#define GPIO_OTYPER_OT4                  GPIO_OTYPER_OT4_Msk                   
#define GPIO_OTYPER_OT5_Pos              (5U)                                  
#define GPIO_OTYPER_OT5_Msk              (0x1U << GPIO_OTYPER_OT5_Pos)         /*!< 0x00000020 */
#define GPIO_OTYPER_OT5                  GPIO_OTYPER_OT5_Msk                   
#define GPIO_OTYPER_OT6_Pos              (6U)                                  
#define GPIO_OTYPER_OT6_Msk              (0x1U << GPIO_OTYPER_OT6_Pos)         /*!< 0x00000040 */
#define GPIO_OTYPER_OT6                  GPIO_OTYPER_OT6_Msk                   
#define GPIO_OTYPER_OT7_Pos              (7U)                                  
#define GPIO_OTYPER_OT7_Msk              (0x1U << GPIO_OTYPER_OT7_Pos)         /*!< 0x00000080 */
#define GPIO_OTYPER_OT7                  GPIO_OTYPER_OT7_Msk                   
#define GPIO_OTYPER_OT8_Pos              (8U)                                  
#define GPIO_OTYPER_OT8_Msk              (0x1U << GPIO_OTYPER_OT8_Pos)         /*!< 0x00000100 */
#define GPIO_OTYPER_OT8                  GPIO_OTYPER_OT8_Msk                   
#define GPIO_OTYPER_OT9_Pos              (9U)                                  
#define GPIO_OTYPER_OT9_Msk              (0x1U << GPIO_OTYPER_OT9_Pos)         /*!< 0x00000200 */
#define GPIO_OTYPER_OT9                  GPIO_OTYPER_OT9_Msk                   
#define GPIO_OTYPER_OT10_Pos             (10U)                                 
#define GPIO_OTYPER_OT10_Msk             (0x1U << GPIO_OTYPER_OT10_Pos)        /*!< 0x00000400 */
#define GPIO_OTYPER_OT10                 GPIO_OTYPER_OT10_Msk                  
#define GPIO_OTYPER_OT11_Pos             (11U)                                 
#define GPIO_OTYPER_OT11_Msk             (0x1U << GPIO_OTYPER_OT11_Pos)        /*!< 0x00000800 */
#define GPIO_OTYPER_OT11                 GPIO_OTYPER_OT11_Msk                  
#define GPIO_OTYPER_OT12_Pos             (12U)                                 
#define GPIO_OTYPER_OT12_Msk             (0x1U << GPIO_OTYPER_OT12_Pos)        /*!< 0x00001000 */
#define GPIO_OTYPER_OT12                 GPIO_OTYPER_OT12_Msk                  
#define GPIO_OTYPER_OT13_Pos             (13U)                                 
#define GPIO_OTYPER_OT13_Msk             (0x1U << GPIO_OTYPER_OT13_Pos)        /*!< 0x00002000 */
#define GPIO_OTYPER_OT13                 GPIO_OTYPER_OT13_Msk                  
#define GPIO_OTYPER_OT14_Pos             (14U)                                 
#define GPIO_OTYPER_OT14_Msk             (0x1U << GPIO_OTYPER_OT14_Pos)        /*!< 0x00004000 */
#define GPIO_OTYPER_OT14                 GPIO_OTYPER_OT14_Msk                  
#define GPIO_OTYPER_OT15_Pos             (15U)                                 
#define GPIO_OTYPER_OT15_Msk             (0x1U << GPIO_OTYPER_OT15_Pos)        /*!< 0x00008000 */
#define GPIO_OTYPER_OT15                 GPIO_OTYPER_OT15_Msk                  

/* Legacy defines */
#define GPIO_OTYPER_OT_0                    GPIO_OTYPER_OT0
#define GPIO_OTYPER_OT_1                    GPIO_OTYPER_OT1
#define GPIO_OTYPER_OT_2                    GPIO_OTYPER_OT2
#define GPIO_OTYPER_OT_3                    GPIO_OTYPER_OT3
#define GPIO_OTYPER_OT_4                    GPIO_OTYPER_OT4
#define GPIO_OTYPER_OT_5                    GPIO_OTYPER_OT5
#define GPIO_OTYPER_OT_6                    GPIO_OTYPER_OT6
#define GPIO_OTYPER_OT_7                    GPIO_OTYPER_OT7
#define GPIO_OTYPER_OT_8                    GPIO_OTYPER_OT8
#define GPIO_OTYPER_OT_9                    GPIO_OTYPER_OT9
#define GPIO_OTYPER_OT_10                   GPIO_OTYPER_OT10
#define GPIO_OTYPER_OT_11                   GPIO_OTYPER_OT11
#define GPIO_OTYPER_OT_12                   GPIO_OTYPER_OT12
#define GPIO_OTYPER_OT_13                   GPIO_OTYPER_OT13
#define GPIO_OTYPER_OT_14                   GPIO_OTYPER_OT14
#define GPIO_OTYPER_OT_15                   GPIO_OTYPER_OT15

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos         (0U)                                  
#define GPIO_OSPEEDR_OSPEED0_Msk         (0x3U << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0             GPIO_OSPEEDR_OSPEED0_Msk              
#define GPIO_OSPEEDR_OSPEED0_0           (0x1U << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000001 */
#define GPIO_OSPEEDR_OSPEED0_1           (0x2U << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000002 */
#define GPIO_OSPEEDR_OSPEED1_Pos         (2U)                                  
#define GPIO_OSPEEDR_OSPEED1_Msk         (0x3U << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x0000000C */
#define GPIO_OSPEEDR_OSPEED1             GPIO_OSPEEDR_OSPEED1_Msk              
#define GPIO_OSPEEDR_OSPEED1_0           (0x1U << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000004 */
#define GPIO_OSPEEDR_OSPEED1_1           (0x2U << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x00000008 */
#define GPIO_OSPEEDR_OSPEED2_Pos         (4U)                                  
#define GPIO_OSPEEDR_OSPEED2_Msk         (0x3U << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000030 */
#define GPIO_OSPEEDR_OSPEED2             GPIO_OSPEEDR_OSPEED2_Msk              
#define GPIO_OSPEEDR_OSPEED2_0           (0x1U << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000010 */
#define GPIO_OSPEEDR_OSPEED2_1           (0x2U << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000020 */
#define GPIO_OSPEEDR_OSPEED3_Pos         (6U)                                  
#define GPIO_OSPEEDR_OSPEED3_Msk         (0x3U << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x000000C0 */
#define GPIO_OSPEEDR_OSPEED3             GPIO_OSPEEDR_OSPEED3_Msk              
#define GPIO_OSPEEDR_OSPEED3_0           (0x1U << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000040 */
#define GPIO_OSPEEDR_OSPEED3_1           (0x2U << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x00000080 */
#define GPIO_OSPEEDR_OSPEED4_Pos         (8U)                                  
#define GPIO_OSPEEDR_OSPEED4_Msk         (0x3U << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000300 */
#define GPIO_OSPEEDR_OSPEED4             GPIO_OSPEEDR_OSPEED4_Msk              
#define GPIO_OSPEEDR_OSPEED4_0           (0x1U << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000100 */
#define GPIO_OSPEEDR_OSPEED4_1           (0x2U << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000200 */
#define GPIO_OSPEEDR_OSPEED5_Pos         (10U)                                 
#define GPIO_OSPEEDR_OSPEED5_Msk         (0x3U << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000C00 */
#define GPIO_OSPEEDR_OSPEED5             GPIO_OSPEEDR_OSPEED5_Msk              
#define GPIO_OSPEEDR_OSPEED5_0           (0x1U << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000400 */
#define GPIO_OSPEEDR_OSPEED5_1           (0x2U << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000800 */
#define GPIO_OSPEEDR_OSPEED6_Pos         (12U)                                 
#define GPIO_OSPEEDR_OSPEED6_Msk         (0x3U << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00003000 */
#define GPIO_OSPEEDR_OSPEED6             GPIO_OSPEEDR_OSPEED6_Msk              
#define GPIO_OSPEEDR_OSPEED6_0           (0x1U << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00001000 */
#define GPIO_OSPEEDR_OSPEED6_1           (0x2U << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00002000 */
#define GPIO_OSPEEDR_OSPEED7_Pos         (14U)                                 
#define GPIO_OSPEEDR_OSPEED7_Msk         (0x3U << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x0000C000 */
#define GPIO_OSPEEDR_OSPEED7             GPIO_OSPEEDR_OSPEED7_Msk              
#define GPIO_OSPEEDR_OSPEED7_0           (0x1U << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00004000 */
#define GPIO_OSPEEDR_OSPEED7_1           (0x2U << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x00008000 */
#define GPIO_OSPEEDR_OSPEED8_Pos         (16U)                                 
#define GPIO_OSPEEDR_OSPEED8_Msk         (0x3U << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00030000 */
#define GPIO_OSPEEDR_OSPEED8             GPIO_OSPEEDR_OSPEED8_Msk              
#define GPIO_OSPEEDR_OSPEED8_0           (0x1U << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00010000 */
#define GPIO_OSPEEDR_OSPEED8_1           (0x2U << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00020000 */
#define GPIO_OSPEEDR_OSPEED9_Pos         (18U)                                 
#define GPIO_OSPEEDR_OSPEED9_Msk         (0x3U << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x000C0000 */
#define GPIO_OSPEEDR_OSPEED9             GPIO_OSPEEDR_OSPEED9_Msk              
#define GPIO_OSPEEDR_OSPEED9_0           (0x1U << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00040000 */
#define GPIO_OSPEEDR_OSPEED9_1           (0x2U << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x00080000 */
#define GPIO_OSPEEDR_OSPEED10_Pos        (20U)                                 
#define GPIO_OSPEEDR_OSPEED10_Msk        (0x3U << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00300000 */
#define GPIO_OSPEEDR_OSPEED10            GPIO_OSPEEDR_OSPEED10_Msk             
#define GPIO_OSPEEDR_OSPEED10_0          (0x1U << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00100000 */
#define GPIO_OSPEEDR_OSPEED10_1          (0x2U << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00200000 */
#define GPIO_OSPEEDR_OSPEED11_Pos        (22U)                                 
#define GPIO_OSPEEDR_OSPEED11_Msk        (0x3U << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00C00000 */
#define GPIO_OSPEEDR_OSPEED11            GPIO_OSPEEDR_OSPEED11_Msk             
#define GPIO_OSPEEDR_OSPEED11_0          (0x1U << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00400000 */
#define GPIO_OSPEEDR_OSPEED11_1          (0x2U << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00800000 */
#define GPIO_OSPEEDR_OSPEED12_Pos        (24U)                                 
#define GPIO_OSPEEDR_OSPEED12_Msk        (0x3U << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x03000000 */
#define GPIO_OSPEEDR_OSPEED12            GPIO_OSPEEDR_OSPEED12_Msk             
#define GPIO_OSPEEDR_OSPEED12_0          (0x1U << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x01000000 */
#define GPIO_OSPEEDR_OSPEED12_1          (0x2U << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x02000000 */
#define GPIO_OSPEEDR_OSPEED13_Pos        (26U)                                 
#define GPIO_OSPEEDR_OSPEED13_Msk        (0x3U << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x0C000000 */
#define GPIO_OSPEEDR_OSPEED13            GPIO_OSPEEDR_OSPEED13_Msk             
#define GPIO_OSPEEDR_OSPEED13_0          (0x1U << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x04000000 */
#define GPIO_OSPEEDR_OSPEED13_1          (0x2U << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x08000000 */
#define GPIO_OSPEEDR_OSPEED14_Pos        (28U)                                 
#define GPIO_OSPEEDR_OSPEED14_Msk        (0x3U << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x30000000 */
#define GPIO_OSPEEDR_OSPEED14            GPIO_OSPEEDR_OSPEED14_Msk             
#define GPIO_OSPEEDR_OSPEED14_0          (0x1U << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x10000000 */
#define GPIO_OSPEEDR_OSPEED14_1          (0x2U << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x20000000 */
#define GPIO_OSPEEDR_OSPEED15_Pos        (30U)                                 
#define GPIO_OSPEEDR_OSPEED15_Msk        (0x3U << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0xC0000000 */
#define GPIO_OSPEEDR_OSPEED15            GPIO_OSPEEDR_OSPEED15_Msk             
#define GPIO_OSPEEDR_OSPEED15_0          (0x1U << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x40000000 */
#define GPIO_OSPEEDR_OSPEED15_1          (0x2U << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_OSPEEDER_OSPEEDR0              GPIO_OSPEEDR_OSPEED0
#define GPIO_OSPEEDER_OSPEEDR0_0            GPIO_OSPEEDR_OSPEED0_0
#define GPIO_OSPEEDER_OSPEEDR0_1            GPIO_OSPEEDR_OSPEED0_1
#define GPIO_OSPEEDER_OSPEEDR1              GPIO_OSPEEDR_OSPEED1
#define GPIO_OSPEEDER_OSPEEDR1_0            GPIO_OSPEEDR_OSPEED1_0
#define GPIO_OSPEEDER_OSPEEDR1_1            GPIO_OSPEEDR_OSPEED1_1
#define GPIO_OSPEEDER_OSPEEDR2              GPIO_OSPEEDR_OSPEED2
#define GPIO_OSPEEDER_OSPEEDR2_0            GPIO_OSPEEDR_OSPEED2_0
#define GPIO_OSPEEDER_OSPEEDR2_1            GPIO_OSPEEDR_OSPEED2_1
#define GPIO_OSPEEDER_OSPEEDR3              GPIO_OSPEEDR_OSPEED3
#define GPIO_OSPEEDER_OSPEEDR3_0            GPIO_OSPEEDR_OSPEED3_0
#define GPIO_OSPEEDER_OSPEEDR3_1            GPIO_OSPEEDR_OSPEED3_1
#define GPIO_OSPEEDER_OSPEEDR4              GPIO_OSPEEDR_OSPEED4
#define GPIO_OSPEEDER_OSPEEDR4_0            GPIO_OSPEEDR_OSPEED4_0
#define GPIO_OSPEEDER_OSPEEDR4_1            GPIO_OSPEEDR_OSPEED4_1
#define GPIO_OSPEEDER_OSPEEDR5              GPIO_OSPEEDR_OSPEED5
#define GPIO_OSPEEDER_OSPEEDR5_0            GPIO_OSPEEDR_OSPEED5_0
#define GPIO_OSPEEDER_OSPEEDR5_1            GPIO_OSPEEDR_OSPEED5_1
#define GPIO_OSPEEDER_OSPEEDR6              GPIO_OSPEEDR_OSPEED6
#define GPIO_OSPEEDER_OSPEEDR6_0            GPIO_OSPEEDR_OSPEED6_0
#define GPIO_OSPEEDER_OSPEEDR6_1            GPIO_OSPEEDR_OSPEED6_1
#define GPIO_OSPEEDER_OSPEEDR7              GPIO_OSPEEDR_OSPEED7
#define GPIO_OSPEEDER_OSPEEDR7_0            GPIO_OSPEEDR_OSPEED7_0
#define GPIO_OSPEEDER_OSPEEDR7_1            GPIO_OSPEEDR_OSPEED7_1
#define GPIO_OSPEEDER_OSPEEDR8              GPIO_OSPEEDR_OSPEED8
#define GPIO_OSPEEDER_OSPEEDR8_0            GPIO_OSPEEDR_OSPEED8_0
#define GPIO_OSPEEDER_OSPEEDR8_1            GPIO_OSPEEDR_OSPEED8_1
#define GPIO_OSPEEDER_OSPEEDR9              GPIO_OSPEEDR_OSPEED9
#define GPIO_OSPEEDER_OSPEEDR9_0            GPIO_OSPEEDR_OSPEED9_0
#define GPIO_OSPEEDER_OSPEEDR9_1            GPIO_OSPEEDR_OSPEED9_1
#define GPIO_OSPEEDER_OSPEEDR10             GPIO_OSPEEDR_OSPEED10
#define GPIO_OSPEEDER_OSPEEDR10_0           GPIO_OSPEEDR_OSPEED10_0
#define GPIO_OSPEEDER_OSPEEDR10_1           GPIO_OSPEEDR_OSPEED10_1
#define GPIO_OSPEEDER_OSPEEDR11             GPIO_OSPEEDR_OSPEED11
#define GPIO_OSPEEDER_OSPEEDR11_0           GPIO_OSPEEDR_OSPEED11_0
#define GPIO_OSPEEDER_OSPEEDR11_1           GPIO_OSPEEDR_OSPEED11_1
#define GPIO_OSPEEDER_OSPEEDR12             GPIO_OSPEEDR_OSPEED12
#define GPIO_OSPEEDER_OSPEEDR12_0           GPIO_OSPEEDR_OSPEED12_0
#define GPIO_OSPEEDER_OSPEEDR12_1           GPIO_OSPEEDR_OSPEED12_1
#define GPIO_OSPEEDER_OSPEEDR13             GPIO_OSPEEDR_OSPEED13
#define GPIO_OSPEEDER_OSPEEDR13_0           GPIO_OSPEEDR_OSPEED13_0
#define GPIO_OSPEEDER_OSPEEDR13_1           GPIO_OSPEEDR_OSPEED13_1
#define GPIO_OSPEEDER_OSPEEDR14             GPIO_OSPEEDR_OSPEED14
#define GPIO_OSPEEDER_OSPEEDR14_0           GPIO_OSPEEDR_OSPEED14_0
#define GPIO_OSPEEDER_OSPEEDR14_1           GPIO_OSPEEDR_OSPEED14_1
#define GPIO_OSPEEDER_OSPEEDR15             GPIO_OSPEEDR_OSPEED15
#define GPIO_OSPEEDER_OSPEEDR15_0           GPIO_OSPEEDR_OSPEED15_0
#define GPIO_OSPEEDER_OSPEEDR15_1           GPIO_OSPEEDR_OSPEED15_1

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos             (0U)                                  
#define GPIO_PUPDR_PUPD0_Msk             (0x3U << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0                 GPIO_PUPDR_PUPD0_Msk                  
#define GPIO_PUPDR_PUPD0_0               (0x1U << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000001 */
#define GPIO_PUPDR_PUPD0_1               (0x2U << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000002 */
#define GPIO_PUPDR_PUPD1_Pos             (2U)                                  
#define GPIO_PUPDR_PUPD1_Msk             (0x3U << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x0000000C */
#define GPIO_PUPDR_PUPD1                 GPIO_PUPDR_PUPD1_Msk                  
#define GPIO_PUPDR_PUPD1_0               (0x1U << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000004 */
#define GPIO_PUPDR_PUPD1_1               (0x2U << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x00000008 */
#define GPIO_PUPDR_PUPD2_Pos             (4U)                                  
#define GPIO_PUPDR_PUPD2_Msk             (0x3U << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000030 */
#define GPIO_PUPDR_PUPD2                 GPIO_PUPDR_PUPD2_Msk                  
#define GPIO_PUPDR_PUPD2_0               (0x1U << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000010 */
#define GPIO_PUPDR_PUPD2_1               (0x2U << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000020 */
#define GPIO_PUPDR_PUPD3_Pos             (6U)                                  
#define GPIO_PUPDR_PUPD3_Msk             (0x3U << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPD3                 GPIO_PUPDR_PUPD3_Msk                  
#define GPIO_PUPDR_PUPD3_0               (0x1U << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000040 */
#define GPIO_PUPDR_PUPD3_1               (0x2U << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x00000080 */
#define GPIO_PUPDR_PUPD4_Pos             (8U)                                  
#define GPIO_PUPDR_PUPD4_Msk             (0x3U << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000300 */
#define GPIO_PUPDR_PUPD4                 GPIO_PUPDR_PUPD4_Msk                  
#define GPIO_PUPDR_PUPD4_0               (0x1U << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000100 */
#define GPIO_PUPDR_PUPD4_1               (0x2U << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000200 */
#define GPIO_PUPDR_PUPD5_Pos             (10U)                                 
#define GPIO_PUPDR_PUPD5_Msk             (0x3U << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPD5                 GPIO_PUPDR_PUPD5_Msk                  
#define GPIO_PUPDR_PUPD5_0               (0x1U << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000400 */
#define GPIO_PUPDR_PUPD5_1               (0x2U << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000800 */
#define GPIO_PUPDR_PUPD6_Pos             (12U)                                 
#define GPIO_PUPDR_PUPD6_Msk             (0x3U << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00003000 */
#define GPIO_PUPDR_PUPD6                 GPIO_PUPDR_PUPD6_Msk                  
#define GPIO_PUPDR_PUPD6_0               (0x1U << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00001000 */
#define GPIO_PUPDR_PUPD6_1               (0x2U << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00002000 */
#define GPIO_PUPDR_PUPD7_Pos             (14U)                                 
#define GPIO_PUPDR_PUPD7_Msk             (0x3U << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPD7                 GPIO_PUPDR_PUPD7_Msk                  
#define GPIO_PUPDR_PUPD7_0               (0x1U << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00004000 */
#define GPIO_PUPDR_PUPD7_1               (0x2U << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x00008000 */
#define GPIO_PUPDR_PUPD8_Pos             (16U)                                 
#define GPIO_PUPDR_PUPD8_Msk             (0x3U << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00030000 */
#define GPIO_PUPDR_PUPD8                 GPIO_PUPDR_PUPD8_Msk                  
#define GPIO_PUPDR_PUPD8_0               (0x1U << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00010000 */
#define GPIO_PUPDR_PUPD8_1               (0x2U << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00020000 */
#define GPIO_PUPDR_PUPD9_Pos             (18U)                                 
#define GPIO_PUPDR_PUPD9_Msk             (0x3U << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPD9                 GPIO_PUPDR_PUPD9_Msk                  
#define GPIO_PUPDR_PUPD9_0               (0x1U << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00040000 */
#define GPIO_PUPDR_PUPD9_1               (0x2U << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x00080000 */
#define GPIO_PUPDR_PUPD10_Pos            (20U)                                 
#define GPIO_PUPDR_PUPD10_Msk            (0x3U << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00300000 */
#define GPIO_PUPDR_PUPD10                GPIO_PUPDR_PUPD10_Msk                 
#define GPIO_PUPDR_PUPD10_0              (0x1U << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00100000 */
#define GPIO_PUPDR_PUPD10_1              (0x2U << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00200000 */
#define GPIO_PUPDR_PUPD11_Pos            (22U)                                 
#define GPIO_PUPDR_PUPD11_Msk            (0x3U << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPD11                GPIO_PUPDR_PUPD11_Msk                 
#define GPIO_PUPDR_PUPD11_0              (0x1U << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00400000 */
#define GPIO_PUPDR_PUPD11_1              (0x2U << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00800000 */
#define GPIO_PUPDR_PUPD12_Pos            (24U)                                 
#define GPIO_PUPDR_PUPD12_Msk            (0x3U << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x03000000 */
#define GPIO_PUPDR_PUPD12                GPIO_PUPDR_PUPD12_Msk                 
#define GPIO_PUPDR_PUPD12_0              (0x1U << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x01000000 */
#define GPIO_PUPDR_PUPD12_1              (0x2U << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x02000000 */
#define GPIO_PUPDR_PUPD13_Pos            (26U)                                 
#define GPIO_PUPDR_PUPD13_Msk            (0x3U << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPD13                GPIO_PUPDR_PUPD13_Msk                 
#define GPIO_PUPDR_PUPD13_0              (0x1U << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x04000000 */
#define GPIO_PUPDR_PUPD13_1              (0x2U << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x08000000 */
#define GPIO_PUPDR_PUPD14_Pos            (28U)                                 
#define GPIO_PUPDR_PUPD14_Msk            (0x3U << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x30000000 */
#define GPIO_PUPDR_PUPD14                GPIO_PUPDR_PUPD14_Msk                 
#define GPIO_PUPDR_PUPD14_0              (0x1U << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x10000000 */
#define GPIO_PUPDR_PUPD14_1              (0x2U << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x20000000 */
#define GPIO_PUPDR_PUPD15_Pos            (30U)                                 
#define GPIO_PUPDR_PUPD15_Msk            (0x3U << GPIO_PUPDR_PUPD15_Pos)       /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPD15                GPIO_PUPDR_PUPD15_Msk                 
#define GPIO_PUPDR_PUPD15_0              (0x1U << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x40000000 */
#define GPIO_PUPDR_PUPD15_1              (0x2U << GPIO_PUPDR_PUPD15_Pos)       /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_PUPDR_PUPDR0                   GPIO_PUPDR_PUPD0
#define GPIO_PUPDR_PUPDR0_0                 GPIO_PUPDR_PUPD0_0
#define GPIO_PUPDR_PUPDR0_1                 GPIO_PUPDR_PUPD0_1
#define GPIO_PUPDR_PUPDR1                   GPIO_PUPDR_PUPD1
#define GPIO_PUPDR_PUPDR1_0                 GPIO_PUPDR_PUPD1_0
#define GPIO_PUPDR_PUPDR1_1                 GPIO_PUPDR_PUPD1_1
#define GPIO_PUPDR_PUPDR2                   GPIO_PUPDR_PUPD2
#define GPIO_PUPDR_PUPDR2_0                 GPIO_PUPDR_PUPD2_0
#define GPIO_PUPDR_PUPDR2_1                 GPIO_PUPDR_PUPD2_1
#define GPIO_PUPDR_PUPDR3                   GPIO_PUPDR_PUPD3
#define GPIO_PUPDR_PUPDR3_0                 GPIO_PUPDR_PUPD3_0
#define GPIO_PUPDR_PUPDR3_1                 GPIO_PUPDR_PUPD3_1
#define GPIO_PUPDR_PUPDR4                   GPIO_PUPDR_PUPD4
#define GPIO_PUPDR_PUPDR4_0                 GPIO_PUPDR_PUPD4_0
#define GPIO_PUPDR_PUPDR4_1                 GPIO_PUPDR_PUPD4_1
#define GPIO_PUPDR_PUPDR5                   GPIO_PUPDR_PUPD5
#define GPIO_PUPDR_PUPDR5_0                 GPIO_PUPDR_PUPD5_0
#define GPIO_PUPDR_PUPDR5_1                 GPIO_PUPDR_PUPD5_1
#define GPIO_PUPDR_PUPDR6                   GPIO_PUPDR_PUPD6
#define GPIO_PUPDR_PUPDR6_0                 GPIO_PUPDR_PUPD6_0
#define GPIO_PUPDR_PUPDR6_1                 GPIO_PUPDR_PUPD6_1
#define GPIO_PUPDR_PUPDR7                   GPIO_PUPDR_PUPD7
#define GPIO_PUPDR_PUPDR7_0                 GPIO_PUPDR_PUPD7_0
#define GPIO_PUPDR_PUPDR7_1                 GPIO_PUPDR_PUPD7_1
#define GPIO_PUPDR_PUPDR8                   GPIO_PUPDR_PUPD8
#define GPIO_PUPDR_PUPDR8_0                 GPIO_PUPDR_PUPD8_0
#define GPIO_PUPDR_PUPDR8_1                 GPIO_PUPDR_PUPD8_1
#define GPIO_PUPDR_PUPDR9                   GPIO_PUPDR_PUPD9
#define GPIO_PUPDR_PUPDR9_0                 GPIO_PUPDR_PUPD9_0
#define GPIO_PUPDR_PUPDR9_1                 GPIO_PUPDR_PUPD9_1
#define GPIO_PUPDR_PUPDR10                  GPIO_PUPDR_PUPD10
#define GPIO_PUPDR_PUPDR10_0                GPIO_PUPDR_PUPD10_0
#define GPIO_PUPDR_PUPDR10_1                GPIO_PUPDR_PUPD10_1
#define GPIO_PUPDR_PUPDR11                  GPIO_PUPDR_PUPD11
#define GPIO_PUPDR_PUPDR11_0                GPIO_PUPDR_PUPD11_0
#define GPIO_PUPDR_PUPDR11_1                GPIO_PUPDR_PUPD11_1
#define GPIO_PUPDR_PUPDR12                  GPIO_PUPDR_PUPD12
#define GPIO_PUPDR_PUPDR12_0                GPIO_PUPDR_PUPD12_0
#define GPIO_PUPDR_PUPDR12_1                GPIO_PUPDR_PUPD12_1
#define GPIO_PUPDR_PUPDR13                  GPIO_PUPDR_PUPD13
#define GPIO_PUPDR_PUPDR13_0                GPIO_PUPDR_PUPD13_0
#define GPIO_PUPDR_PUPDR13_1                GPIO_PUPDR_PUPD13_1
#define GPIO_PUPDR_PUPDR14                  GPIO_PUPDR_PUPD14
#define GPIO_PUPDR_PUPDR14_0                GPIO_PUPDR_PUPD14_0
#define GPIO_PUPDR_PUPDR14_1                GPIO_PUPDR_PUPD14_1
#define GPIO_PUPDR_PUPDR15                  GPIO_PUPDR_PUPD15
#define GPIO_PUPDR_PUPDR15_0                GPIO_PUPDR_PUPD15_0
#define GPIO_PUPDR_PUPDR15_1                GPIO_PUPDR_PUPD15_1

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos                 (0U)                                  
#define GPIO_IDR_ID0_Msk                 (0x1U << GPIO_IDR_ID0_Pos)            /*!< 0x00000001 */
#define GPIO_IDR_ID0                     GPIO_IDR_ID0_Msk                      
#define GPIO_IDR_ID1_Pos                 (1U)                                  
#define GPIO_IDR_ID1_Msk                 (0x1U << GPIO_IDR_ID1_Pos)            /*!< 0x00000002 */
#define GPIO_IDR_ID1                     GPIO_IDR_ID1_Msk                      
#define GPIO_IDR_ID2_Pos                 (2U)                                  
#define GPIO_IDR_ID2_Msk                 (0x1U << GPIO_IDR_ID2_Pos)            /*!< 0x00000004 */
#define GPIO_IDR_ID2                     GPIO_IDR_ID2_Msk                      
#define GPIO_IDR_ID3_Pos                 (3U)                                  
#define GPIO_IDR_ID3_Msk                 (0x1U << GPIO_IDR_ID3_Pos)            /*!< 0x00000008 */
#define GPIO_IDR_ID3                     GPIO_IDR_ID3_Msk                      
#define GPIO_IDR_ID4_Pos                 (4U)                                  
#define GPIO_IDR_ID4_Msk                 (0x1U << GPIO_IDR_ID4_Pos)            /*!< 0x00000010 */
#define GPIO_IDR_ID4                     GPIO_IDR_ID4_Msk                      
#define GPIO_IDR_ID5_Pos                 (5U)                                  
#define GPIO_IDR_ID5_Msk                 (0x1U << GPIO_IDR_ID5_Pos)            /*!< 0x00000020 */
#define GPIO_IDR_ID5                     GPIO_IDR_ID5_Msk                      
#define GPIO_IDR_ID6_Pos                 (6U)                                  
#define GPIO_IDR_ID6_Msk                 (0x1U << GPIO_IDR_ID6_Pos)            /*!< 0x00000040 */
#define GPIO_IDR_ID6                     GPIO_IDR_ID6_Msk                      
#define GPIO_IDR_ID7_Pos                 (7U)                                  
#define GPIO_IDR_ID7_Msk                 (0x1U << GPIO_IDR_ID7_Pos)            /*!< 0x00000080 */
#define GPIO_IDR_ID7                     GPIO_IDR_ID7_Msk                      
#define GPIO_IDR_ID8_Pos                 (8U)                                  
#define GPIO_IDR_ID8_Msk                 (0x1U << GPIO_IDR_ID8_Pos)            /*!< 0x00000100 */
#define GPIO_IDR_ID8                     GPIO_IDR_ID8_Msk                      
#define GPIO_IDR_ID9_Pos                 (9U)                                  
#define GPIO_IDR_ID9_Msk                 (0x1U << GPIO_IDR_ID9_Pos)            /*!< 0x00000200 */
#define GPIO_IDR_ID9                     GPIO_IDR_ID9_Msk                      
#define GPIO_IDR_ID10_Pos                (10U)                                 
#define GPIO_IDR_ID10_Msk                (0x1U << GPIO_IDR_ID10_Pos)           /*!< 0x00000400 */
#define GPIO_IDR_ID10                    GPIO_IDR_ID10_Msk                     
#define GPIO_IDR_ID11_Pos                (11U)                                 
#define GPIO_IDR_ID11_Msk                (0x1U << GPIO_IDR_ID11_Pos)           /*!< 0x00000800 */
#define GPIO_IDR_ID11                    GPIO_IDR_ID11_Msk                     
#define GPIO_IDR_ID12_Pos                (12U)                                 
#define GPIO_IDR_ID12_Msk                (0x1U << GPIO_IDR_ID12_Pos)           /*!< 0x00001000 */
#define GPIO_IDR_ID12                    GPIO_IDR_ID12_Msk                     
#define GPIO_IDR_ID13_Pos                (13U)                                 
#define GPIO_IDR_ID13_Msk                (0x1U << GPIO_IDR_ID13_Pos)           /*!< 0x00002000 */
#define GPIO_IDR_ID13                    GPIO_IDR_ID13_Msk                     
#define GPIO_IDR_ID14_Pos                (14U)                                 
#define GPIO_IDR_ID14_Msk                (0x1U << GPIO_IDR_ID14_Pos)           /*!< 0x00004000 */
#define GPIO_IDR_ID14                    GPIO_IDR_ID14_Msk                     
#define GPIO_IDR_ID15_Pos                (15U)                                 
#define GPIO_IDR_ID15_Msk                (0x1U << GPIO_IDR_ID15_Pos)           /*!< 0x00008000 */
#define GPIO_IDR_ID15                    GPIO_IDR_ID15_Msk                     

/* Legacy defines */
#define GPIO_IDR_IDR_0                      GPIO_IDR_ID0
#define GPIO_IDR_IDR_1                      GPIO_IDR_ID1
#define GPIO_IDR_IDR_2                      GPIO_IDR_ID2
#define GPIO_IDR_IDR_3                      GPIO_IDR_ID3
#define GPIO_IDR_IDR_4                      GPIO_IDR_ID4
#define GPIO_IDR_IDR_5                      GPIO_IDR_ID5
#define GPIO_IDR_IDR_6                      GPIO_IDR_ID6
#define GPIO_IDR_IDR_7                      GPIO_IDR_ID7
#define GPIO_IDR_IDR_8                      GPIO_IDR_ID8
#define GPIO_IDR_IDR_9                      GPIO_IDR_ID9
#define GPIO_IDR_IDR_10                     GPIO_IDR_ID10
#define GPIO_IDR_IDR_11                     GPIO_IDR_ID11
#define GPIO_IDR_IDR_12                     GPIO_IDR_ID12
#define GPIO_IDR_IDR_13                     GPIO_IDR_ID13
#define GPIO_IDR_IDR_14                     GPIO_IDR_ID14
#define GPIO_IDR_IDR_15                     GPIO_IDR_ID15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos                 (0U)                                  
#define GPIO_ODR_OD0_Msk                 (0x1U << GPIO_ODR_OD0_Pos)            /*!< 0x00000001 */
#define GPIO_ODR_OD0                     GPIO_ODR_OD0_Msk                      
#define GPIO_ODR_OD1_Pos                 (1U)                                  
#define GPIO_ODR_OD1_Msk                 (0x1U << GPIO_ODR_OD1_Pos)            /*!< 0x00000002 */
#define GPIO_ODR_OD1                     GPIO_ODR_OD1_Msk                      
#define GPIO_ODR_OD2_Pos                 (2U)                                  
#define GPIO_ODR_OD2_Msk                 (0x1U << GPIO_ODR_OD2_Pos)            /*!< 0x00000004 */
#define GPIO_ODR_OD2                     GPIO_ODR_OD2_Msk                      
#define GPIO_ODR_OD3_Pos                 (3U)                                  
#define GPIO_ODR_OD3_Msk                 (0x1U << GPIO_ODR_OD3_Pos)            /*!< 0x00000008 */
#define GPIO_ODR_OD3                     GPIO_ODR_OD3_Msk                      
#define GPIO_ODR_OD4_Pos                 (4U)                                  
#define GPIO_ODR_OD4_Msk                 (0x1U << GPIO_ODR_OD4_Pos)            /*!< 0x00000010 */
#define GPIO_ODR_OD4                     GPIO_ODR_OD4_Msk                      
#define GPIO_ODR_OD5_Pos                 (5U)                                  
#define GPIO_ODR_OD5_Msk                 (0x1U << GPIO_ODR_OD5_Pos)            /*!< 0x00000020 */
#define GPIO_ODR_OD5                     GPIO_ODR_OD5_Msk                      
#define GPIO_ODR_OD6_Pos                 (6U)                                  
#define GPIO_ODR_OD6_Msk                 (0x1U << GPIO_ODR_OD6_Pos)            /*!< 0x00000040 */
#define GPIO_ODR_OD6                     GPIO_ODR_OD6_Msk                      
#define GPIO_ODR_OD7_Pos                 (7U)                                  
#define GPIO_ODR_OD7_Msk                 (0x1U << GPIO_ODR_OD7_Pos)            /*!< 0x00000080 */
#define GPIO_ODR_OD7                     GPIO_ODR_OD7_Msk                      
#define GPIO_ODR_OD8_Pos                 (8U)                                  
#define GPIO_ODR_OD8_Msk                 (0x1U << GPIO_ODR_OD8_Pos)            /*!< 0x00000100 */
#define GPIO_ODR_OD8                     GPIO_ODR_OD8_Msk                      
#define GPIO_ODR_OD9_Pos                 (9U)                                  
#define GPIO_ODR_OD9_Msk                 (0x1U << GPIO_ODR_OD9_Pos)            /*!< 0x00000200 */
#define GPIO_ODR_OD9                     GPIO_ODR_OD9_Msk                      
#define GPIO_ODR_OD10_Pos                (10U)                                 
#define GPIO_ODR_OD10_Msk                (0x1U << GPIO_ODR_OD10_Pos)           /*!< 0x00000400 */
#define GPIO_ODR_OD10                    GPIO_ODR_OD10_Msk                     
#define GPIO_ODR_OD11_Pos                (11U)                                 
#define GPIO_ODR_OD11_Msk                (0x1U << GPIO_ODR_OD11_Pos)           /*!< 0x00000800 */
#define GPIO_ODR_OD11                    GPIO_ODR_OD11_Msk                     
#define GPIO_ODR_OD12_Pos                (12U)                                 
#define GPIO_ODR_OD12_Msk                (0x1U << GPIO_ODR_OD12_Pos)           /*!< 0x00001000 */
#define GPIO_ODR_OD12                    GPIO_ODR_OD12_Msk                     
#define GPIO_ODR_OD13_Pos                (13U)                                 
#define GPIO_ODR_OD13_Msk                (0x1U << GPIO_ODR_OD13_Pos)           /*!< 0x00002000 */
#define GPIO_ODR_OD13                    GPIO_ODR_OD13_Msk                     
#define GPIO_ODR_OD14_Pos                (14U)                                 
#define GPIO_ODR_OD14_Msk                (0x1U << GPIO_ODR_OD14_Pos)           /*!< 0x00004000 */
#define GPIO_ODR_OD14                    GPIO_ODR_OD14_Msk                     
#define GPIO_ODR_OD15_Pos                (15U)                                 
#define GPIO_ODR_OD15_Msk                (0x1U << GPIO_ODR_OD15_Pos)           /*!< 0x00008000 */
#define GPIO_ODR_OD15                    GPIO_ODR_OD15_Msk                     
/* Legacy defines */
#define GPIO_ODR_ODR_0                       GPIO_ODR_OD0
#define GPIO_ODR_ODR_1                       GPIO_ODR_OD1
#define GPIO_ODR_ODR_2                       GPIO_ODR_OD2
#define GPIO_ODR_ODR_3                       GPIO_ODR_OD3
#define GPIO_ODR_ODR_4                       GPIO_ODR_OD4
#define GPIO_ODR_ODR_5                       GPIO_ODR_OD5
#define GPIO_ODR_ODR_6                       GPIO_ODR_OD6
#define GPIO_ODR_ODR_7                       GPIO_ODR_OD7
#define GPIO_ODR_ODR_8                       GPIO_ODR_OD8
#define GPIO_ODR_ODR_9                       GPIO_ODR_OD9
#define GPIO_ODR_ODR_10                      GPIO_ODR_OD10
#define GPIO_ODR_ODR_11                      GPIO_ODR_OD11
#define GPIO_ODR_ODR_12                      GPIO_ODR_OD12
#define GPIO_ODR_ODR_13                      GPIO_ODR_OD13
#define GPIO_ODR_ODR_14                      GPIO_ODR_OD14
#define GPIO_ODR_ODR_15                      GPIO_ODR_OD15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos                (0U)                                  
#define GPIO_BSRR_BS0_Msk                (0x1U << GPIO_BSRR_BS0_Pos)           /*!< 0x00000001 */
#define GPIO_BSRR_BS0                    GPIO_BSRR_BS0_Msk                     
#define GPIO_BSRR_BS1_Pos                (1U)                                  
#define GPIO_BSRR_BS1_Msk                (0x1U << GPIO_BSRR_BS1_Pos)           /*!< 0x00000002 */
#define GPIO_BSRR_BS1                    GPIO_BSRR_BS1_Msk                     
#define GPIO_BSRR_BS2_Pos                (2U)                                  
#define GPIO_BSRR_BS2_Msk                (0x1U << GPIO_BSRR_BS2_Pos)           /*!< 0x00000004 */
#define GPIO_BSRR_BS2                    GPIO_BSRR_BS2_Msk                     
#define GPIO_BSRR_BS3_Pos                (3U)                                  
#define GPIO_BSRR_BS3_Msk                (0x1U << GPIO_BSRR_BS3_Pos)           /*!< 0x00000008 */
#define GPIO_BSRR_BS3                    GPIO_BSRR_BS3_Msk                     
#define GPIO_BSRR_BS4_Pos                (4U)                                  
#define GPIO_BSRR_BS4_Msk                (0x1U << GPIO_BSRR_BS4_Pos)           /*!< 0x00000010 */
#define GPIO_BSRR_BS4                    GPIO_BSRR_BS4_Msk                     
#define GPIO_BSRR_BS5_Pos                (5U)                                  
#define GPIO_BSRR_BS5_Msk                (0x1U << GPIO_BSRR_BS5_Pos)           /*!< 0x00000020 */
#define GPIO_BSRR_BS5                    GPIO_BSRR_BS5_Msk                     
#define GPIO_BSRR_BS6_Pos                (6U)                                  
#define GPIO_BSRR_BS6_Msk                (0x1U << GPIO_BSRR_BS6_Pos)           /*!< 0x00000040 */
#define GPIO_BSRR_BS6                    GPIO_BSRR_BS6_Msk                     
#define GPIO_BSRR_BS7_Pos                (7U)                                  
#define GPIO_BSRR_BS7_Msk                (0x1U << GPIO_BSRR_BS7_Pos)           /*!< 0x00000080 */
#define GPIO_BSRR_BS7                    GPIO_BSRR_BS7_Msk                     
#define GPIO_BSRR_BS8_Pos                (8U)                                  
#define GPIO_BSRR_BS8_Msk                (0x1U << GPIO_BSRR_BS8_Pos)           /*!< 0x00000100 */
#define GPIO_BSRR_BS8                    GPIO_BSRR_BS8_Msk                     
#define GPIO_BSRR_BS9_Pos                (9U)                                  
#define GPIO_BSRR_BS9_Msk                (0x1U << GPIO_BSRR_BS9_Pos)           /*!< 0x00000200 */
#define GPIO_BSRR_BS9                    GPIO_BSRR_BS9_Msk                     
#define GPIO_BSRR_BS10_Pos               (10U)                                 
#define GPIO_BSRR_BS10_Msk               (0x1U << GPIO_BSRR_BS10_Pos)          /*!< 0x00000400 */
#define GPIO_BSRR_BS10                   GPIO_BSRR_BS10_Msk                    
#define GPIO_BSRR_BS11_Pos               (11U)                                 
#define GPIO_BSRR_BS11_Msk               (0x1U << GPIO_BSRR_BS11_Pos)          /*!< 0x00000800 */
#define GPIO_BSRR_BS11                   GPIO_BSRR_BS11_Msk                    
#define GPIO_BSRR_BS12_Pos               (12U)                                 
#define GPIO_BSRR_BS12_Msk               (0x1U << GPIO_BSRR_BS12_Pos)          /*!< 0x00001000 */
#define GPIO_BSRR_BS12                   GPIO_BSRR_BS12_Msk                    
#define GPIO_BSRR_BS13_Pos               (13U)                                 
#define GPIO_BSRR_BS13_Msk               (0x1U << GPIO_BSRR_BS13_Pos)          /*!< 0x00002000 */
#define GPIO_BSRR_BS13                   GPIO_BSRR_BS13_Msk                    
#define GPIO_BSRR_BS14_Pos               (14U)                                 
#define GPIO_BSRR_BS14_Msk               (0x1U << GPIO_BSRR_BS14_Pos)          /*!< 0x00004000 */
#define GPIO_BSRR_BS14                   GPIO_BSRR_BS14_Msk                    
#define GPIO_BSRR_BS15_Pos               (15U)                                 
#define GPIO_BSRR_BS15_Msk               (0x1U << GPIO_BSRR_BS15_Pos)          /*!< 0x00008000 */
#define GPIO_BSRR_BS15                   GPIO_BSRR_BS15_Msk                    
#define GPIO_BSRR_BR0_Pos                (16U)                                 
#define GPIO_BSRR_BR0_Msk                (0x1U << GPIO_BSRR_BR0_Pos)           /*!< 0x00010000 */
#define GPIO_BSRR_BR0                    GPIO_BSRR_BR0_Msk                     
#define GPIO_BSRR_BR1_Pos                (17U)                                 
#define GPIO_BSRR_BR1_Msk                (0x1U << GPIO_BSRR_BR1_Pos)           /*!< 0x00020000 */
#define GPIO_BSRR_BR1                    GPIO_BSRR_BR1_Msk                     
#define GPIO_BSRR_BR2_Pos                (18U)                                 
#define GPIO_BSRR_BR2_Msk                (0x1U << GPIO_BSRR_BR2_Pos)           /*!< 0x00040000 */
#define GPIO_BSRR_BR2                    GPIO_BSRR_BR2_Msk                     
#define GPIO_BSRR_BR3_Pos                (19U)                                 
#define GPIO_BSRR_BR3_Msk                (0x1U << GPIO_BSRR_BR3_Pos)           /*!< 0x00080000 */
#define GPIO_BSRR_BR3                    GPIO_BSRR_BR3_Msk                     
#define GPIO_BSRR_BR4_Pos                (20U)                                 
#define GPIO_BSRR_BR4_Msk                (0x1U << GPIO_BSRR_BR4_Pos)           /*!< 0x00100000 */
#define GPIO_BSRR_BR4                    GPIO_BSRR_BR4_Msk                     
#define GPIO_BSRR_BR5_Pos                (21U)                                 
#define GPIO_BSRR_BR5_Msk                (0x1U << GPIO_BSRR_BR5_Pos)           /*!< 0x00200000 */
#define GPIO_BSRR_BR5                    GPIO_BSRR_BR5_Msk                     
#define GPIO_BSRR_BR6_Pos                (22U)                                 
#define GPIO_BSRR_BR6_Msk                (0x1U << GPIO_BSRR_BR6_Pos)           /*!< 0x00400000 */
#define GPIO_BSRR_BR6                    GPIO_BSRR_BR6_Msk                     
#define GPIO_BSRR_BR7_Pos                (23U)                                 
#define GPIO_BSRR_BR7_Msk                (0x1U << GPIO_BSRR_BR7_Pos)           /*!< 0x00800000 */
#define GPIO_BSRR_BR7                    GPIO_BSRR_BR7_Msk                     
#define GPIO_BSRR_BR8_Pos                (24U)                                 
#define GPIO_BSRR_BR8_Msk                (0x1U << GPIO_BSRR_BR8_Pos)           /*!< 0x01000000 */
#define GPIO_BSRR_BR8                    GPIO_BSRR_BR8_Msk                     
#define GPIO_BSRR_BR9_Pos                (25U)                                 
#define GPIO_BSRR_BR9_Msk                (0x1U << GPIO_BSRR_BR9_Pos)           /*!< 0x02000000 */
#define GPIO_BSRR_BR9                    GPIO_BSRR_BR9_Msk                     
#define GPIO_BSRR_BR10_Pos               (26U)                                 
#define GPIO_BSRR_BR10_Msk               (0x1U << GPIO_BSRR_BR10_Pos)          /*!< 0x04000000 */
#define GPIO_BSRR_BR10                   GPIO_BSRR_BR10_Msk                    
#define GPIO_BSRR_BR11_Pos               (27U)                                 
#define GPIO_BSRR_BR11_Msk               (0x1U << GPIO_BSRR_BR11_Pos)          /*!< 0x08000000 */
#define GPIO_BSRR_BR11                   GPIO_BSRR_BR11_Msk                    
#define GPIO_BSRR_BR12_Pos               (28U)                                 
#define GPIO_BSRR_BR12_Msk               (0x1U << GPIO_BSRR_BR12_Pos)          /*!< 0x10000000 */
#define GPIO_BSRR_BR12                   GPIO_BSRR_BR12_Msk                    
#define GPIO_BSRR_BR13_Pos               (29U)                                 
#define GPIO_BSRR_BR13_Msk               (0x1U << GPIO_BSRR_BR13_Pos)          /*!< 0x20000000 */
#define GPIO_BSRR_BR13                   GPIO_BSRR_BR13_Msk                    
#define GPIO_BSRR_BR14_Pos               (30U)                                 
#define GPIO_BSRR_BR14_Msk               (0x1U << GPIO_BSRR_BR14_Pos)          /*!< 0x40000000 */
#define GPIO_BSRR_BR14                   GPIO_BSRR_BR14_Msk                    
#define GPIO_BSRR_BR15_Pos               (31U)                                 
#define GPIO_BSRR_BR15_Msk               (0x1U << GPIO_BSRR_BR15_Pos)          /*!< 0x80000000 */
#define GPIO_BSRR_BR15                   GPIO_BSRR_BR15_Msk                    

/* Legacy defines */
#define GPIO_BSRR_BS_0                      GPIO_BSRR_BS0
#define GPIO_BSRR_BS_1                      GPIO_BSRR_BS1
#define GPIO_BSRR_BS_2                      GPIO_BSRR_BS2
#define GPIO_BSRR_BS_3                      GPIO_BSRR_BS3
#define GPIO_BSRR_BS_4                      GPIO_BSRR_BS4
#define GPIO_BSRR_BS_5                      GPIO_BSRR_BS5
#define GPIO_BSRR_BS_6                      GPIO_BSRR_BS6
#define GPIO_BSRR_BS_7                      GPIO_BSRR_BS7
#define GPIO_BSRR_BS_8                      GPIO_BSRR_BS8
#define GPIO_BSRR_BS_9                      GPIO_BSRR_BS9
#define GPIO_BSRR_BS_10                     GPIO_BSRR_BS10
#define GPIO_BSRR_BS_11                     GPIO_BSRR_BS11
#define GPIO_BSRR_BS_12                     GPIO_BSRR_BS12
#define GPIO_BSRR_BS_13                     GPIO_BSRR_BS13
#define GPIO_BSRR_BS_14                     GPIO_BSRR_BS14
#define GPIO_BSRR_BS_15                     GPIO_BSRR_BS15
#define GPIO_BSRR_BR_0                      GPIO_BSRR_BR0
#define GPIO_BSRR_BR_1                      GPIO_BSRR_BR1
#define GPIO_BSRR_BR_2                      GPIO_BSRR_BR2
#define GPIO_BSRR_BR_3                      GPIO_BSRR_BR3
#define GPIO_BSRR_BR_4                      GPIO_BSRR_BR4
#define GPIO_BSRR_BR_5                      GPIO_BSRR_BR5
#define GPIO_BSRR_BR_6                      GPIO_BSRR_BR6
#define GPIO_BSRR_BR_7                      GPIO_BSRR_BR7
#define GPIO_BSRR_BR_8                      GPIO_BSRR_BR8
#define GPIO_BSRR_BR_9                      GPIO_BSRR_BR9
#define GPIO_BSRR_BR_10                     GPIO_BSRR_BR10
#define GPIO_BSRR_BR_11                     GPIO_BSRR_BR11
#define GPIO_BSRR_BR_12                     GPIO_BSRR_BR12
#define GPIO_BSRR_BR_13                     GPIO_BSRR_BR13
#define GPIO_BSRR_BR_14                     GPIO_BSRR_BR14
#define GPIO_BSRR_BR_15                     GPIO_BSRR_BR15
/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos               (0U)                                  
#define GPIO_LCKR_LCK0_Msk               (0x1U << GPIO_LCKR_LCK0_Pos)          /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                   GPIO_LCKR_LCK0_Msk                    
#define GPIO_LCKR_LCK1_Pos               (1U)                                  
#define GPIO_LCKR_LCK1_Msk               (0x1U << GPIO_LCKR_LCK1_Pos)          /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                   GPIO_LCKR_LCK1_Msk                    
#define GPIO_LCKR_LCK2_Pos               (2U)                                  
#define GPIO_LCKR_LCK2_Msk               (0x1U << GPIO_LCKR_LCK2_Pos)          /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                   GPIO_LCKR_LCK2_Msk                    
#define GPIO_LCKR_LCK3_Pos               (3U)                                  
#define GPIO_LCKR_LCK3_Msk               (0x1U << GPIO_LCKR_LCK3_Pos)          /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                   GPIO_LCKR_LCK3_Msk                    
#define GPIO_LCKR_LCK4_Pos               (4U)                                  
#define GPIO_LCKR_LCK4_Msk               (0x1U << GPIO_LCKR_LCK4_Pos)          /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                   GPIO_LCKR_LCK4_Msk                    
#define GPIO_LCKR_LCK5_Pos               (5U)                                  
#define GPIO_LCKR_LCK5_Msk               (0x1U << GPIO_LCKR_LCK5_Pos)          /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                   GPIO_LCKR_LCK5_Msk                    
#define GPIO_LCKR_LCK6_Pos               (6U)                                  
#define GPIO_LCKR_LCK6_Msk               (0x1U << GPIO_LCKR_LCK6_Pos)          /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                   GPIO_LCKR_LCK6_Msk                    
#define GPIO_LCKR_LCK7_Pos               (7U)                                  
#define GPIO_LCKR_LCK7_Msk               (0x1U << GPIO_LCKR_LCK7_Pos)          /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                   GPIO_LCKR_LCK7_Msk                    
#define GPIO_LCKR_LCK8_Pos               (8U)                                  
#define GPIO_LCKR_LCK8_Msk               (0x1U << GPIO_LCKR_LCK8_Pos)          /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                   GPIO_LCKR_LCK8_Msk                    
#define GPIO_LCKR_LCK9_Pos               (9U)                                  
#define GPIO_LCKR_LCK9_Msk               (0x1U << GPIO_LCKR_LCK9_Pos)          /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                   GPIO_LCKR_LCK9_Msk                    
#define GPIO_LCKR_LCK10_Pos              (10U)                                 
#define GPIO_LCKR_LCK10_Msk              (0x1U << GPIO_LCKR_LCK10_Pos)         /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                  GPIO_LCKR_LCK10_Msk                   
#define GPIO_LCKR_LCK11_Pos              (11U)                                 
#define GPIO_LCKR_LCK11_Msk              (0x1U << GPIO_LCKR_LCK11_Pos)         /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                  GPIO_LCKR_LCK11_Msk                   
#define GPIO_LCKR_LCK12_Pos              (12U)                                 
#define GPIO_LCKR_LCK12_Msk              (0x1U << GPIO_LCKR_LCK12_Pos)         /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                  GPIO_LCKR_LCK12_Msk                   
#define GPIO_LCKR_LCK13_Pos              (13U)                                 
#define GPIO_LCKR_LCK13_Msk              (0x1U << GPIO_LCKR_LCK13_Pos)         /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                  GPIO_LCKR_LCK13_Msk                   
#define GPIO_LCKR_LCK14_Pos              (14U)                                 
#define GPIO_LCKR_LCK14_Msk              (0x1U << GPIO_LCKR_LCK14_Pos)         /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                  GPIO_LCKR_LCK14_Msk                   
#define GPIO_LCKR_LCK15_Pos              (15U)                                 
#define GPIO_LCKR_LCK15_Msk              (0x1U << GPIO_LCKR_LCK15_Pos)         /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                  GPIO_LCKR_LCK15_Msk                   
#define GPIO_LCKR_LCKK_Pos               (16U)                                 
#define GPIO_LCKR_LCKK_Msk               (0x1U << GPIO_LCKR_LCKK_Pos)          /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                   GPIO_LCKR_LCKK_Msk                    
/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFSEL0_Pos             (0U)                                  
#define GPIO_AFRL_AFSEL0_Msk             (0xFU << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x0000000F */
#define GPIO_AFRL_AFSEL0                 GPIO_AFRL_AFSEL0_Msk                  
#define GPIO_AFRL_AFSEL0_0               (0x1U << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000001 */
#define GPIO_AFRL_AFSEL0_1               (0x2U << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000002 */
#define GPIO_AFRL_AFSEL0_2               (0x4U << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000004 */
#define GPIO_AFRL_AFSEL0_3               (0x8U << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x00000008 */
#define GPIO_AFRL_AFSEL1_Pos             (4U)                                  
#define GPIO_AFRL_AFSEL1_Msk             (0xFU << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRL_AFSEL1                 GPIO_AFRL_AFSEL1_Msk                  
#define GPIO_AFRL_AFSEL1_0               (0x1U << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000010 */
#define GPIO_AFRL_AFSEL1_1               (0x2U << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000020 */
#define GPIO_AFRL_AFSEL1_2               (0x4U << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000040 */
#define GPIO_AFRL_AFSEL1_3               (0x8U << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x00000080 */
#define GPIO_AFRL_AFSEL2_Pos             (8U)                                  
#define GPIO_AFRL_AFSEL2_Msk             (0xFU << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000F00 */
#define GPIO_AFRL_AFSEL2                 GPIO_AFRL_AFSEL2_Msk                  
#define GPIO_AFRL_AFSEL2_0               (0x1U << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000100 */
#define GPIO_AFRL_AFSEL2_1               (0x2U << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000200 */
#define GPIO_AFRL_AFSEL2_2               (0x4U << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000400 */
#define GPIO_AFRL_AFSEL2_3               (0x8U << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000800 */
#define GPIO_AFRL_AFSEL3_Pos             (12U)                                 
#define GPIO_AFRL_AFSEL3_Msk             (0xFU << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x0000F000 */
#define GPIO_AFRL_AFSEL3                 GPIO_AFRL_AFSEL3_Msk                  
#define GPIO_AFRL_AFSEL3_0               (0x1U << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00001000 */
#define GPIO_AFRL_AFSEL3_1               (0x2U << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00002000 */
#define GPIO_AFRL_AFSEL3_2               (0x4U << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00004000 */
#define GPIO_AFRL_AFSEL3_3               (0x8U << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x00008000 */
#define GPIO_AFRL_AFSEL4_Pos             (16U)                                 
#define GPIO_AFRL_AFSEL4_Msk             (0xFU << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x000F0000 */
#define GPIO_AFRL_AFSEL4                 GPIO_AFRL_AFSEL4_Msk                  
#define GPIO_AFRL_AFSEL4_0               (0x1U << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00010000 */
#define GPIO_AFRL_AFSEL4_1               (0x2U << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00020000 */
#define GPIO_AFRL_AFSEL4_2               (0x4U << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00040000 */
#define GPIO_AFRL_AFSEL4_3               (0x8U << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x00080000 */
#define GPIO_AFRL_AFSEL5_Pos             (20U)                                 
#define GPIO_AFRL_AFSEL5_Msk             (0xFU << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00F00000 */
#define GPIO_AFRL_AFSEL5                 GPIO_AFRL_AFSEL5_Msk                  
#define GPIO_AFRL_AFSEL5_0               (0x1U << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00100000 */
#define GPIO_AFRL_AFSEL5_1               (0x2U << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00200000 */
#define GPIO_AFRL_AFSEL5_2               (0x4U << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00400000 */
#define GPIO_AFRL_AFSEL5_3               (0x8U << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00800000 */
#define GPIO_AFRL_AFSEL6_Pos             (24U)                                 
#define GPIO_AFRL_AFSEL6_Msk             (0xFU << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x0F000000 */
#define GPIO_AFRL_AFSEL6                 GPIO_AFRL_AFSEL6_Msk                  
#define GPIO_AFRL_AFSEL6_0               (0x1U << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x01000000 */
#define GPIO_AFRL_AFSEL6_1               (0x2U << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x02000000 */
#define GPIO_AFRL_AFSEL6_2               (0x4U << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x04000000 */
#define GPIO_AFRL_AFSEL6_3               (0x8U << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x08000000 */
#define GPIO_AFRL_AFSEL7_Pos             (28U)                                 
#define GPIO_AFRL_AFSEL7_Msk             (0xFU << GPIO_AFRL_AFSEL7_Pos)        /*!< 0xF0000000 */
#define GPIO_AFRL_AFSEL7                 GPIO_AFRL_AFSEL7_Msk                  
#define GPIO_AFRL_AFSEL7_0               (0x1U << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x10000000 */
#define GPIO_AFRL_AFSEL7_1               (0x2U << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x20000000 */
#define GPIO_AFRL_AFSEL7_2               (0x4U << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x40000000 */
#define GPIO_AFRL_AFSEL7_3               (0x8U << GPIO_AFRL_AFSEL7_Pos)        /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRL_AFRL0                      GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL0_0                    GPIO_AFRL_AFSEL0_0
#define GPIO_AFRL_AFRL0_1                    GPIO_AFRL_AFSEL0_1
#define GPIO_AFRL_AFRL0_2                    GPIO_AFRL_AFSEL0_2
#define GPIO_AFRL_AFRL0_3                    GPIO_AFRL_AFSEL0_3
#define GPIO_AFRL_AFRL1                      GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL1_0                    GPIO_AFRL_AFSEL1_0
#define GPIO_AFRL_AFRL1_1                    GPIO_AFRL_AFSEL1_1
#define GPIO_AFRL_AFRL1_2                    GPIO_AFRL_AFSEL1_2
#define GPIO_AFRL_AFRL1_3                    GPIO_AFRL_AFSEL1_3
#define GPIO_AFRL_AFRL2                      GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL2_0                    GPIO_AFRL_AFSEL2_0
#define GPIO_AFRL_AFRL2_1                    GPIO_AFRL_AFSEL2_1
#define GPIO_AFRL_AFRL2_2                    GPIO_AFRL_AFSEL2_2
#define GPIO_AFRL_AFRL2_3                    GPIO_AFRL_AFSEL2_3
#define GPIO_AFRL_AFRL3                      GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL3_0                    GPIO_AFRL_AFSEL3_0
#define GPIO_AFRL_AFRL3_1                    GPIO_AFRL_AFSEL3_1
#define GPIO_AFRL_AFRL3_2                    GPIO_AFRL_AFSEL3_2
#define GPIO_AFRL_AFRL3_3                    GPIO_AFRL_AFSEL3_3
#define GPIO_AFRL_AFRL4                      GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL4_0                    GPIO_AFRL_AFSEL4_0
#define GPIO_AFRL_AFRL4_1                    GPIO_AFRL_AFSEL4_1
#define GPIO_AFRL_AFRL4_2                    GPIO_AFRL_AFSEL4_2
#define GPIO_AFRL_AFRL4_3                    GPIO_AFRL_AFSEL4_3
#define GPIO_AFRL_AFRL5                      GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL5_0                    GPIO_AFRL_AFSEL5_0
#define GPIO_AFRL_AFRL5_1                    GPIO_AFRL_AFSEL5_1
#define GPIO_AFRL_AFRL5_2                    GPIO_AFRL_AFSEL5_2
#define GPIO_AFRL_AFRL5_3                    GPIO_AFRL_AFSEL5_3
#define GPIO_AFRL_AFRL6                      GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL6_0                    GPIO_AFRL_AFSEL6_0
#define GPIO_AFRL_AFRL6_1                    GPIO_AFRL_AFSEL6_1
#define GPIO_AFRL_AFRL6_2                    GPIO_AFRL_AFSEL6_2
#define GPIO_AFRL_AFRL6_3                    GPIO_AFRL_AFSEL6_3
#define GPIO_AFRL_AFRL7                      GPIO_AFRL_AFSEL7
#define GPIO_AFRL_AFRL7_0                    GPIO_AFRL_AFSEL7_0
#define GPIO_AFRL_AFRL7_1                    GPIO_AFRL_AFSEL7_1
#define GPIO_AFRL_AFRL7_2                    GPIO_AFRL_AFSEL7_2
#define GPIO_AFRL_AFRL7_3                    GPIO_AFRL_AFSEL7_3

/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFSEL8_Pos             (0U)                                  
#define GPIO_AFRH_AFSEL8_Msk             (0xFU << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x0000000F */
#define GPIO_AFRH_AFSEL8                 GPIO_AFRH_AFSEL8_Msk                  
#define GPIO_AFRH_AFSEL8_0               (0x1U << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000001 */
#define GPIO_AFRH_AFSEL8_1               (0x2U << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000002 */
#define GPIO_AFRH_AFSEL8_2               (0x4U << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000004 */
#define GPIO_AFRH_AFSEL8_3               (0x8U << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x00000008 */
#define GPIO_AFRH_AFSEL9_Pos             (4U)                                  
#define GPIO_AFRH_AFSEL9_Msk             (0xFU << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x000000F0 */
#define GPIO_AFRH_AFSEL9                 GPIO_AFRH_AFSEL9_Msk                  
#define GPIO_AFRH_AFSEL9_0               (0x1U << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000010 */
#define GPIO_AFRH_AFSEL9_1               (0x2U << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000020 */
#define GPIO_AFRH_AFSEL9_2               (0x4U << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000040 */
#define GPIO_AFRH_AFSEL9_3               (0x8U << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x00000080 */
#define GPIO_AFRH_AFSEL10_Pos            (8U)                                  
#define GPIO_AFRH_AFSEL10_Msk            (0xFU << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000F00 */
#define GPIO_AFRH_AFSEL10                GPIO_AFRH_AFSEL10_Msk                 
#define GPIO_AFRH_AFSEL10_0              (0x1U << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000100 */
#define GPIO_AFRH_AFSEL10_1              (0x2U << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000200 */
#define GPIO_AFRH_AFSEL10_2              (0x4U << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000400 */
#define GPIO_AFRH_AFSEL10_3              (0x8U << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000800 */
#define GPIO_AFRH_AFSEL11_Pos            (12U)                                 
#define GPIO_AFRH_AFSEL11_Msk            (0xFU << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x0000F000 */
#define GPIO_AFRH_AFSEL11                GPIO_AFRH_AFSEL11_Msk                 
#define GPIO_AFRH_AFSEL11_0              (0x1U << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00001000 */
#define GPIO_AFRH_AFSEL11_1              (0x2U << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00002000 */
#define GPIO_AFRH_AFSEL11_2              (0x4U << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00004000 */
#define GPIO_AFRH_AFSEL11_3              (0x8U << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x00008000 */
#define GPIO_AFRH_AFSEL12_Pos            (16U)                                 
#define GPIO_AFRH_AFSEL12_Msk            (0xFU << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x000F0000 */
#define GPIO_AFRH_AFSEL12                GPIO_AFRH_AFSEL12_Msk                 
#define GPIO_AFRH_AFSEL12_0              (0x1U << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00010000 */
#define GPIO_AFRH_AFSEL12_1              (0x2U << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00020000 */
#define GPIO_AFRH_AFSEL12_2              (0x4U << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00040000 */
#define GPIO_AFRH_AFSEL12_3              (0x8U << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x00080000 */
#define GPIO_AFRH_AFSEL13_Pos            (20U)                                 
#define GPIO_AFRH_AFSEL13_Msk            (0xFU << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00F00000 */
#define GPIO_AFRH_AFSEL13                GPIO_AFRH_AFSEL13_Msk                 
#define GPIO_AFRH_AFSEL13_0              (0x1U << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00100000 */
#define GPIO_AFRH_AFSEL13_1              (0x2U << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00200000 */
#define GPIO_AFRH_AFSEL13_2              (0x4U << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00400000 */
#define GPIO_AFRH_AFSEL13_3              (0x8U << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00800000 */
#define GPIO_AFRH_AFSEL14_Pos            (24U)                                 
#define GPIO_AFRH_AFSEL14_Msk            (0xFU << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x0F000000 */
#define GPIO_AFRH_AFSEL14                GPIO_AFRH_AFSEL14_Msk                 
#define GPIO_AFRH_AFSEL14_0              (0x1U << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x01000000 */
#define GPIO_AFRH_AFSEL14_1              (0x2U << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x02000000 */
#define GPIO_AFRH_AFSEL14_2              (0x4U << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x04000000 */
#define GPIO_AFRH_AFSEL14_3              (0x8U << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x08000000 */
#define GPIO_AFRH_AFSEL15_Pos            (28U)                                 
#define GPIO_AFRH_AFSEL15_Msk            (0xFU << GPIO_AFRH_AFSEL15_Pos)       /*!< 0xF0000000 */
#define GPIO_AFRH_AFSEL15                GPIO_AFRH_AFSEL15_Msk                 
#define GPIO_AFRH_AFSEL15_0              (0x1U << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x10000000 */
#define GPIO_AFRH_AFSEL15_1              (0x2U << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x20000000 */
#define GPIO_AFRH_AFSEL15_2              (0x4U << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x40000000 */
#define GPIO_AFRH_AFSEL15_3              (0x8U << GPIO_AFRH_AFSEL15_Pos)       /*!< 0x80000000 */

/* Legacy defines */
#define GPIO_AFRH_AFRH0                      GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH0_0                    GPIO_AFRH_AFSEL8_0
#define GPIO_AFRH_AFRH0_1                    GPIO_AFRH_AFSEL8_1
#define GPIO_AFRH_AFRH0_2                    GPIO_AFRH_AFSEL8_2
#define GPIO_AFRH_AFRH0_3                    GPIO_AFRH_AFSEL8_3
#define GPIO_AFRH_AFRH1                      GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH1_0                    GPIO_AFRH_AFSEL9_0
#define GPIO_AFRH_AFRH1_1                    GPIO_AFRH_AFSEL9_1
#define GPIO_AFRH_AFRH1_2                    GPIO_AFRH_AFSEL9_2
#define GPIO_AFRH_AFRH1_3                    GPIO_AFRH_AFSEL9_3
#define GPIO_AFRH_AFRH2                      GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH2_0                    GPIO_AFRH_AFSEL10_0
#define GPIO_AFRH_AFRH2_1                    GPIO_AFRH_AFSEL10_1
#define GPIO_AFRH_AFRH2_2                    GPIO_AFRH_AFSEL10_2
#define GPIO_AFRH_AFRH2_3                    GPIO_AFRH_AFSEL10_3
#define GPIO_AFRH_AFRH3                      GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH3_0                    GPIO_AFRH_AFSEL11_0
#define GPIO_AFRH_AFRH3_1                    GPIO_AFRH_AFSEL11_1
#define GPIO_AFRH_AFRH3_2                    GPIO_AFRH_AFSEL11_2
#define GPIO_AFRH_AFRH3_3                    GPIO_AFRH_AFSEL11_3
#define GPIO_AFRH_AFRH4                      GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH4_0                    GPIO_AFRH_AFSEL12_0
#define GPIO_AFRH_AFRH4_1                    GPIO_AFRH_AFSEL12_1
#define GPIO_AFRH_AFRH4_2                    GPIO_AFRH_AFSEL12_2
#define GPIO_AFRH_AFRH4_3                    GPIO_AFRH_AFSEL12_3
#define GPIO_AFRH_AFRH5                      GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH5_0                    GPIO_AFRH_AFSEL13_0
#define GPIO_AFRH_AFRH5_1                    GPIO_AFRH_AFSEL13_1
#define GPIO_AFRH_AFRH5_2                    GPIO_AFRH_AFSEL13_2
#define GPIO_AFRH_AFRH5_3                    GPIO_AFRH_AFSEL13_3
#define GPIO_AFRH_AFRH6                      GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH6_0                    GPIO_AFRH_AFSEL14_0
#define GPIO_AFRH_AFRH6_1                    GPIO_AFRH_AFSEL14_1
#define GPIO_AFRH_AFRH6_2                    GPIO_AFRH_AFSEL14_2
#define GPIO_AFRH_AFRH6_3                    GPIO_AFRH_AFSEL14_3
#define GPIO_AFRH_AFRH7                      GPIO_AFRH_AFSEL15
#define GPIO_AFRH_AFRH7_0                    GPIO_AFRH_AFSEL15_0
#define GPIO_AFRH_AFRH7_1                    GPIO_AFRH_AFSEL15_1
#define GPIO_AFRH_AFRH7_2                    GPIO_AFRH_AFSEL15_2
#define GPIO_AFRH_AFRH7_3                    GPIO_AFRH_AFSEL15_3

/******************  Bits definition for GPIO_BRR register  ******************/
#define GPIO_BRR_BR0_Pos                 (0U)                                  
#define GPIO_BRR_BR0_Msk                 (0x1U << GPIO_BRR_BR0_Pos)            /*!< 0x00000001 */
#define GPIO_BRR_BR0                     GPIO_BRR_BR0_Msk                      
#define GPIO_BRR_BR1_Pos                 (1U)                                  
#define GPIO_BRR_BR1_Msk                 (0x1U << GPIO_BRR_BR1_Pos)            /*!< 0x00000002 */
#define GPIO_BRR_BR1                     GPIO_BRR_BR1_Msk                      
#define GPIO_BRR_BR2_Pos                 (2U)                                  
#define GPIO_BRR_BR2_Msk                 (0x1U << GPIO_BRR_BR2_Pos)            /*!< 0x00000004 */
#define GPIO_BRR_BR2                     GPIO_BRR_BR2_Msk                      
#define GPIO_BRR_BR3_Pos                 (3U)                                  
#define GPIO_BRR_BR3_Msk                 (0x1U << GPIO_BRR_BR3_Pos)            /*!< 0x00000008 */
#define GPIO_BRR_BR3                     GPIO_BRR_BR3_Msk                      
#define GPIO_BRR_BR4_Pos                 (4U)                                  
#define GPIO_BRR_BR4_Msk                 (0x1U << GPIO_BRR_BR4_Pos)            /*!< 0x00000010 */
#define GPIO_BRR_BR4                     GPIO_BRR_BR4_Msk                      
#define GPIO_BRR_BR5_Pos                 (5U)                                  
#define GPIO_BRR_BR5_Msk                 (0x1U << GPIO_BRR_BR5_Pos)            /*!< 0x00000020 */
#define GPIO_BRR_BR5                     GPIO_BRR_BR5_Msk                      
#define GPIO_BRR_BR6_Pos                 (6U)                                  
#define GPIO_BRR_BR6_Msk                 (0x1U << GPIO_BRR_BR6_Pos)            /*!< 0x00000040 */
#define GPIO_BRR_BR6                     GPIO_BRR_BR6_Msk                      
#define GPIO_BRR_BR7_Pos                 (7U)                                  
#define GPIO_BRR_BR7_Msk                 (0x1U << GPIO_BRR_BR7_Pos)            /*!< 0x00000080 */
#define GPIO_BRR_BR7                     GPIO_BRR_BR7_Msk                      
#define GPIO_BRR_BR8_Pos                 (8U)                                  
#define GPIO_BRR_BR8_Msk                 (0x1U << GPIO_BRR_BR8_Pos)            /*!< 0x00000100 */
#define GPIO_BRR_BR8                     GPIO_BRR_BR8_Msk                      
#define GPIO_BRR_BR9_Pos                 (9U)                                  
#define GPIO_BRR_BR9_Msk                 (0x1U << GPIO_BRR_BR9_Pos)            /*!< 0x00000200 */
#define GPIO_BRR_BR9                     GPIO_BRR_BR9_Msk                      
#define GPIO_BRR_BR10_Pos                (10U)                                 
#define GPIO_BRR_BR10_Msk                (0x1U << GPIO_BRR_BR10_Pos)           /*!< 0x00000400 */
#define GPIO_BRR_BR10                    GPIO_BRR_BR10_Msk                     
#define GPIO_BRR_BR11_Pos                (11U)                                 
#define GPIO_BRR_BR11_Msk                (0x1U << GPIO_BRR_BR11_Pos)           /*!< 0x00000800 */
#define GPIO_BRR_BR11                    GPIO_BRR_BR11_Msk                     
#define GPIO_BRR_BR12_Pos                (12U)                                 
#define GPIO_BRR_BR12_Msk                (0x1U << GPIO_BRR_BR12_Pos)           /*!< 0x00001000 */
#define GPIO_BRR_BR12                    GPIO_BRR_BR12_Msk                     
#define GPIO_BRR_BR13_Pos                (13U)                                 
#define GPIO_BRR_BR13_Msk                (0x1U << GPIO_BRR_BR13_Pos)           /*!< 0x00002000 */
#define GPIO_BRR_BR13                    GPIO_BRR_BR13_Msk                     
#define GPIO_BRR_BR14_Pos                (14U)                                 
#define GPIO_BRR_BR14_Msk                (0x1U << GPIO_BRR_BR14_Pos)           /*!< 0x00004000 */
#define GPIO_BRR_BR14                    GPIO_BRR_BR14_Msk                     
#define GPIO_BRR_BR15_Pos                (15U)                                 
#define GPIO_BRR_BR15_Msk                (0x1U << GPIO_BRR_BR15_Pos)           /*!< 0x00008000 */
#define GPIO_BRR_BR15                    GPIO_BRR_BR15_Msk                     


/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE_Pos            (0U)                                         
#define I2C_CR1_PE_Msk            (0x1U << I2C_CR1_PE_Pos)                     /*!< 0x00000001 */
#define I2C_CR1_PE                I2C_CR1_PE_Msk                               /*!<Peripheral Enable                             */
#define I2C_CR1_SMBUS_Pos         (1U)                                         
#define I2C_CR1_SMBUS_Msk         (0x1U << I2C_CR1_SMBUS_Pos)                  /*!< 0x00000002 */
#define I2C_CR1_SMBUS             I2C_CR1_SMBUS_Msk                            /*!<SMBus Mode                                    */
#define I2C_CR1_SMBTYPE_Pos       (3U)                                         
#define I2C_CR1_SMBTYPE_Msk       (0x1U << I2C_CR1_SMBTYPE_Pos)                /*!< 0x00000008 */
#define I2C_CR1_SMBTYPE           I2C_CR1_SMBTYPE_Msk                          /*!<SMBus Type                                    */
#define I2C_CR1_ENARP_Pos         (4U)                                         
#define I2C_CR1_ENARP_Msk         (0x1U << I2C_CR1_ENARP_Pos)                  /*!< 0x00000010 */
#define I2C_CR1_ENARP             I2C_CR1_ENARP_Msk                            /*!<ARP Enable                                    */
#define I2C_CR1_ENPEC_Pos         (5U)                                         
#define I2C_CR1_ENPEC_Msk         (0x1U << I2C_CR1_ENPEC_Pos)                  /*!< 0x00000020 */
#define I2C_CR1_ENPEC             I2C_CR1_ENPEC_Msk                            /*!<PEC Enable                                    */
#define I2C_CR1_ENGC_Pos          (6U)                                         
#define I2C_CR1_ENGC_Msk          (0x1U << I2C_CR1_ENGC_Pos)                   /*!< 0x00000040 */
#define I2C_CR1_ENGC              I2C_CR1_ENGC_Msk                             /*!<General Call Enable                           */
#define I2C_CR1_NOSTRETCH_Pos     (7U)                                         
#define I2C_CR1_NOSTRETCH_Msk     (0x1U << I2C_CR1_NOSTRETCH_Pos)              /*!< 0x00000080 */
#define I2C_CR1_NOSTRETCH         I2C_CR1_NOSTRETCH_Msk                        /*!<Clock Stretching Disable (Slave mode)         */
#define I2C_CR1_START_Pos         (8U)                                         
#define I2C_CR1_START_Msk         (0x1U << I2C_CR1_START_Pos)                  /*!< 0x00000100 */
#define I2C_CR1_START             I2C_CR1_START_Msk                            /*!<Start Generation                              */
#define I2C_CR1_STOP_Pos          (9U)                                         
#define I2C_CR1_STOP_Msk          (0x1U << I2C_CR1_STOP_Pos)                   /*!< 0x00000200 */
#define I2C_CR1_STOP              I2C_CR1_STOP_Msk                             /*!<Stop Generation                               */
#define I2C_CR1_ACK_Pos           (10U)                                        
#define I2C_CR1_ACK_Msk           (0x1U << I2C_CR1_ACK_Pos)                    /*!< 0x00000400 */
#define I2C_CR1_ACK               I2C_CR1_ACK_Msk                              /*!<Acknowledge Enable                            */
#define I2C_CR1_POS_Pos           (11U)                                        
#define I2C_CR1_POS_Msk           (0x1U << I2C_CR1_POS_Pos)                    /*!< 0x00000800 */
#define I2C_CR1_POS               I2C_CR1_POS_Msk                              /*!<Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC_Pos           (12U)                                        
#define I2C_CR1_PEC_Msk           (0x1U << I2C_CR1_PEC_Pos)                    /*!< 0x00001000 */
#define I2C_CR1_PEC               I2C_CR1_PEC_Msk                              /*!<Packet Error Checking                         */
#define I2C_CR1_ALERT_Pos         (13U)                                        
#define I2C_CR1_ALERT_Msk         (0x1U << I2C_CR1_ALERT_Pos)                  /*!< 0x00002000 */
#define I2C_CR1_ALERT             I2C_CR1_ALERT_Msk                            /*!<SMBus Alert                                   */
#define I2C_CR1_SWRST_Pos         (15U)                                        
#define I2C_CR1_SWRST_Msk         (0x1U << I2C_CR1_SWRST_Pos)                  /*!< 0x00008000 */
#define I2C_CR1_SWRST             I2C_CR1_SWRST_Msk                            /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ_Pos          (0U)                                         
#define I2C_CR2_FREQ_Msk          (0x3FU << I2C_CR2_FREQ_Pos)                  /*!< 0x0000003F */
#define I2C_CR2_FREQ              I2C_CR2_FREQ_Msk                             /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
#define I2C_CR2_FREQ_0            (0x01U << I2C_CR2_FREQ_Pos)                  /*!< 0x00000001 */
#define I2C_CR2_FREQ_1            (0x02U << I2C_CR2_FREQ_Pos)                  /*!< 0x00000002 */
#define I2C_CR2_FREQ_2            (0x04U << I2C_CR2_FREQ_Pos)                  /*!< 0x00000004 */
#define I2C_CR2_FREQ_3            (0x08U << I2C_CR2_FREQ_Pos)                  /*!< 0x00000008 */
#define I2C_CR2_FREQ_4            (0x10U << I2C_CR2_FREQ_Pos)                  /*!< 0x00000010 */
#define I2C_CR2_FREQ_5            (0x20U << I2C_CR2_FREQ_Pos)                  /*!< 0x00000020 */

#define I2C_CR2_ITERREN_Pos       (8U)                                         
#define I2C_CR2_ITERREN_Msk       (0x1U << I2C_CR2_ITERREN_Pos)                /*!< 0x00000100 */
#define I2C_CR2_ITERREN           I2C_CR2_ITERREN_Msk                          /*!<Error Interrupt Enable  */
#define I2C_CR2_ITEVTEN_Pos       (9U)                                         
#define I2C_CR2_ITEVTEN_Msk       (0x1U << I2C_CR2_ITEVTEN_Pos)                /*!< 0x00000200 */
#define I2C_CR2_ITEVTEN           I2C_CR2_ITEVTEN_Msk                          /*!<Event Interrupt Enable  */
#define I2C_CR2_ITBUFEN_Pos       (10U)                                        
#define I2C_CR2_ITBUFEN_Msk       (0x1U << I2C_CR2_ITBUFEN_Pos)                /*!< 0x00000400 */
#define I2C_CR2_ITBUFEN           I2C_CR2_ITBUFEN_Msk                          /*!<Buffer Interrupt Enable */
#define I2C_CR2_DMAEN_Pos         (11U)                                        
#define I2C_CR2_DMAEN_Msk         (0x1U << I2C_CR2_DMAEN_Pos)                  /*!< 0x00000800 */
#define I2C_CR2_DMAEN             I2C_CR2_DMAEN_Msk                            /*!<DMA Requests Enable     */
#define I2C_CR2_LAST_Pos          (12U)                                        
#define I2C_CR2_LAST_Msk          (0x1U << I2C_CR2_LAST_Pos)                   /*!< 0x00001000 */
#define I2C_CR2_LAST              I2C_CR2_LAST_Msk                             /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD1_Pos         (0U)
#define I2C_OAR1_ADD1_Msk         (0x3FFU << I2C_OAR1_ADD1_Pos)                /*!< 0x000003FF */
#define I2C_OAR1_ADD1             I2C_OAR1_ADD1_Msk                            /*!< Interface own address 1 */

#define I2C_OAR1_ADDMODE_Pos      (15U)                                        
#define I2C_OAR1_ADDMODE_Msk      (0x1U << I2C_OAR1_ADDMODE_Pos)               /*!< 0x00008000 */
#define I2C_OAR1_ADDMODE          I2C_OAR1_ADDMODE_Msk                         /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL_Pos       (0U)                                         
#define I2C_OAR2_ENDUAL_Msk       (0x1U << I2C_OAR2_ENDUAL_Pos)                /*!< 0x00000001 */
#define I2C_OAR2_ENDUAL           I2C_OAR2_ENDUAL_Msk                          /*!<Dual addressing mode enable */
#define I2C_OAR2_ADD2_Pos         (1U)                                         
#define I2C_OAR2_ADD2_Msk         (0x7FU << I2C_OAR2_ADD2_Pos)                 /*!< 0x000000FE */
#define I2C_OAR2_ADD2             I2C_OAR2_ADD2_Msk                            /*!< Interface own address 2 */

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR_Pos             (0U)                                         
#define I2C_DR_DR_Msk             (0xFFU << I2C_DR_DR_Pos)                     /*!< 0x000000FF */
#define I2C_DR_DR                 I2C_DR_DR_Msk                                /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB_Pos            (0U)                                         
#define I2C_SR1_SB_Msk            (0x1U << I2C_SR1_SB_Pos)                     /*!< 0x00000001 */
#define I2C_SR1_SB                I2C_SR1_SB_Msk                               /*!<Start Bit (Master mode)                         */
#define I2C_SR1_ADDR_Pos          (1U)                                         
#define I2C_SR1_ADDR_Msk          (0x1U << I2C_SR1_ADDR_Pos)                   /*!< 0x00000002 */
#define I2C_SR1_ADDR              I2C_SR1_ADDR_Msk                             /*!<Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF_Pos           (2U)                                         
#define I2C_SR1_BTF_Msk           (0x1U << I2C_SR1_BTF_Pos)                    /*!< 0x00000004 */
#define I2C_SR1_BTF               I2C_SR1_BTF_Msk                              /*!<Byte Transfer Finished                          */
#define I2C_SR1_ADD10_Pos         (3U)                                         
#define I2C_SR1_ADD10_Msk         (0x1U << I2C_SR1_ADD10_Pos)                  /*!< 0x00000008 */
#define I2C_SR1_ADD10             I2C_SR1_ADD10_Msk                            /*!<10-bit header sent (Master mode)                */
#define I2C_SR1_STOPF_Pos         (4U)                                         
#define I2C_SR1_STOPF_Msk         (0x1U << I2C_SR1_STOPF_Pos)                  /*!< 0x00000010 */
#define I2C_SR1_STOPF             I2C_SR1_STOPF_Msk                            /*!<Stop detection (Slave mode)                     */
#define I2C_SR1_RXNE_Pos          (6U)                                         
#define I2C_SR1_RXNE_Msk          (0x1U << I2C_SR1_RXNE_Pos)                   /*!< 0x00000040 */
#define I2C_SR1_RXNE              I2C_SR1_RXNE_Msk                             /*!<Data Register not Empty (receivers)             */
#define I2C_SR1_TXE_Pos           (7U)                                         
#define I2C_SR1_TXE_Msk           (0x1U << I2C_SR1_TXE_Pos)                    /*!< 0x00000080 */
#define I2C_SR1_TXE               I2C_SR1_TXE_Msk                              /*!<Data Register Empty (transmitters)              */
#define I2C_SR1_BERR_Pos          (8U)                                         
#define I2C_SR1_BERR_Msk          (0x1U << I2C_SR1_BERR_Pos)                   /*!< 0x00000100 */
#define I2C_SR1_BERR              I2C_SR1_BERR_Msk                             /*!<Bus Error                                       */
#define I2C_SR1_ARLO_Pos          (9U)                                         
#define I2C_SR1_ARLO_Msk          (0x1U << I2C_SR1_ARLO_Pos)                   /*!< 0x00000200 */
#define I2C_SR1_ARLO              I2C_SR1_ARLO_Msk                             /*!<Arbitration Lost (master mode)                  */
#define I2C_SR1_AF_Pos            (10U)                                        
#define I2C_SR1_AF_Msk            (0x1U << I2C_SR1_AF_Pos)                     /*!< 0x00000400 */
#define I2C_SR1_AF                I2C_SR1_AF_Msk                               /*!<Acknowledge Failure                             */
#define I2C_SR1_OVR_Pos           (11U)                                        
#define I2C_SR1_OVR_Msk           (0x1U << I2C_SR1_OVR_Pos)                    /*!< 0x00000800 */
#define I2C_SR1_OVR               I2C_SR1_OVR_Msk                              /*!<Overrun/Underrun                                */
#define I2C_SR1_PECERR_Pos        (12U)                                        
#define I2C_SR1_PECERR_Msk        (0x1U << I2C_SR1_PECERR_Pos)                 /*!< 0x00001000 */
#define I2C_SR1_PECERR            I2C_SR1_PECERR_Msk                           /*!<PEC Error in reception                          */
#define I2C_SR1_TIMEOUT_Pos       (14U)                                        
#define I2C_SR1_TIMEOUT_Msk       (0x1U << I2C_SR1_TIMEOUT_Pos)                /*!< 0x00004000 */
#define I2C_SR1_TIMEOUT           I2C_SR1_TIMEOUT_Msk                          /*!<Timeout or Tlow Error                           */
#define I2C_SR1_SMBALERT_Pos      (15U)                                        
#define I2C_SR1_SMBALERT_Msk      (0x1U << I2C_SR1_SMBALERT_Pos)               /*!< 0x00008000 */
#define I2C_SR1_SMBALERT          I2C_SR1_SMBALERT_Msk                         /*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL_Pos           (0U)                                         
#define I2C_SR2_MSL_Msk           (0x1U << I2C_SR2_MSL_Pos)                    /*!< 0x00000001 */
#define I2C_SR2_MSL               I2C_SR2_MSL_Msk                              /*!<Master/Slave                                    */
#define I2C_SR2_BUSY_Pos          (1U)                                         
#define I2C_SR2_BUSY_Msk          (0x1U << I2C_SR2_BUSY_Pos)                   /*!< 0x00000002 */
#define I2C_SR2_BUSY              I2C_SR2_BUSY_Msk                             /*!<Bus Busy                                        */
#define I2C_SR2_TRA_Pos           (2U)                                         
#define I2C_SR2_TRA_Msk           (0x1U << I2C_SR2_TRA_Pos)                    /*!< 0x00000004 */
#define I2C_SR2_TRA               I2C_SR2_TRA_Msk                              /*!<Transmitter/Receiver                            */
#define I2C_SR2_GENCALL_Pos       (4U)                                         
#define I2C_SR2_GENCALL_Msk       (0x1U << I2C_SR2_GENCALL_Pos)                /*!< 0x00000010 */
#define I2C_SR2_GENCALL           I2C_SR2_GENCALL_Msk                          /*!<General Call Address (Slave mode)               */
#define I2C_SR2_SMBDEFAULT_Pos    (5U)                                         
#define I2C_SR2_SMBDEFAULT_Msk    (0x1U << I2C_SR2_SMBDEFAULT_Pos)             /*!< 0x00000020 */
#define I2C_SR2_SMBDEFAULT        I2C_SR2_SMBDEFAULT_Msk                       /*!<SMBus Device Default Address (Slave mode)       */
#define I2C_SR2_SMBHOST_Pos       (6U)                                         
#define I2C_SR2_SMBHOST_Msk       (0x1U << I2C_SR2_SMBHOST_Pos)                /*!< 0x00000040 */
#define I2C_SR2_SMBHOST           I2C_SR2_SMBHOST_Msk                          /*!<SMBus Host Header (Slave mode)                  */
#define I2C_SR2_DUALF_Pos         (7U)                                         
#define I2C_SR2_DUALF_Msk         (0x1U << I2C_SR2_DUALF_Pos)                  /*!< 0x00000080 */
#define I2C_SR2_DUALF             I2C_SR2_DUALF_Msk                            /*!<Dual Flag (Slave mode)                          */
#define I2C_SR2_PEC_Pos           (8U)                                         
#define I2C_SR2_PEC_Msk           (0xFFU << I2C_SR2_PEC_Pos)                   /*!< 0x0000FF00 */
#define I2C_SR2_PEC               I2C_SR2_PEC_Msk                              /*!<Packet Error Checking Register                  */

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR_Pos           (0U)                                         
#define I2C_CCR_CCR_Msk           (0xFFFU << I2C_CCR_CCR_Pos)                  /*!< 0x00000FFF */
#define I2C_CCR_CCR               I2C_CCR_CCR_Msk                              /*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CCR_DUTY_Pos          (14U)                                        
#define I2C_CCR_DUTY_Msk          (0x1U << I2C_CCR_DUTY_Pos)                   /*!< 0x00004000 */
#define I2C_CCR_DUTY              I2C_CCR_DUTY_Msk                             /*!<Fast Mode Duty Cycle                                       */
#define I2C_CCR_FS_Pos            (15U)                                        
#define I2C_CCR_FS_Msk            (0x1U << I2C_CCR_FS_Pos)                     /*!< 0x00008000 */
#define I2C_CCR_FS                I2C_CCR_FS_Msk                               /*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE_Pos       (0U)                                         
#define I2C_TRISE_TRISE_Msk       (0x3FU << I2C_TRISE_TRISE_Pos)               /*!< 0x0000003F */
#define I2C_TRISE_TRISE           I2C_TRISE_TRISE_Msk                          /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************  Bit definition for I2C_FLTR register  *******************/
#define I2C_FLTR_DNF_Pos          (0U)                                         
#define I2C_FLTR_DNF_Msk          (0xFU << I2C_FLTR_DNF_Pos)                   /*!< 0x0000000F */
#define I2C_FLTR_DNF              I2C_FLTR_DNF_Msk                             /*!<Digital Noise Filter */
#define I2C_FLTR_ANOFF_Pos        (4U)                                         
#define I2C_FLTR_ANOFF_Msk        (0x1U << I2C_FLTR_ANOFF_Pos)                 /*!< 0x00000010 */
#define I2C_FLTR_ANOFF            I2C_FLTR_ANOFF_Msk                           /*!<Analog Noise Filter OFF */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define IWDG_KR_KEY_Pos     (0U)                                               
#define IWDG_KR_KEY_Msk     (0xFFFFU << IWDG_KR_KEY_Pos)                       /*!< 0x0000FFFF */
#define IWDG_KR_KEY         IWDG_KR_KEY_Msk                                    /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define IWDG_PR_PR_Pos      (0U)                                               
#define IWDG_PR_PR_Msk      (0x7U << IWDG_PR_PR_Pos)                           /*!< 0x00000007 */
#define IWDG_PR_PR          IWDG_PR_PR_Msk                                     /*!<PR[2:0] (Prescaler divider)         */
#define IWDG_PR_PR_0        (0x1U << IWDG_PR_PR_Pos)                           /*!< 0x01 */
#define IWDG_PR_PR_1        (0x2U << IWDG_PR_PR_Pos)                           /*!< 0x02 */
#define IWDG_PR_PR_2        (0x4U << IWDG_PR_PR_Pos)                           /*!< 0x04 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define IWDG_RLR_RL_Pos     (0U)                                               
#define IWDG_RLR_RL_Msk     (0xFFFU << IWDG_RLR_RL_Pos)                        /*!< 0x00000FFF */
#define IWDG_RLR_RL         IWDG_RLR_RL_Msk                                    /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define IWDG_SR_PVU_Pos     (0U)                                               
#define IWDG_SR_PVU_Msk     (0x1U << IWDG_SR_PVU_Pos)                          /*!< 0x00000001 */
#define IWDG_SR_PVU         IWDG_SR_PVU_Msk                                    /*!<Watchdog prescaler value update      */
#define IWDG_SR_RVU_Pos     (1U)                                               
#define IWDG_SR_RVU_Msk     (0x1U << IWDG_SR_RVU_Pos)                          /*!< 0x00000002 */
#define IWDG_SR_RVU         IWDG_SR_RVU_Msk                                    /*!<Watchdog counter reload value update */



/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define PWR_CR_LPDS_Pos        (0U)                                            
#define PWR_CR_LPDS_Msk        (0x1U << PWR_CR_LPDS_Pos)                       /*!< 0x00000001 */
#define PWR_CR_LPDS            PWR_CR_LPDS_Msk                                 /*!< Low-Power Deepsleep                 */
#define PWR_CR_PDDS_Pos        (1U)                                            
#define PWR_CR_PDDS_Msk        (0x1U << PWR_CR_PDDS_Pos)                       /*!< 0x00000002 */
#define PWR_CR_PDDS            PWR_CR_PDDS_Msk                                 /*!< Power Down Deepsleep                */
#define PWR_CR_CWUF_Pos        (2U)                                            
#define PWR_CR_CWUF_Msk        (0x1U << PWR_CR_CWUF_Pos)                       /*!< 0x00000004 */
#define PWR_CR_CWUF            PWR_CR_CWUF_Msk                                 /*!< Clear Wakeup Flag                   */
#define PWR_CR_CSBF_Pos        (3U)                                            
#define PWR_CR_CSBF_Msk        (0x1U << PWR_CR_CSBF_Pos)                       /*!< 0x00000008 */
#define PWR_CR_CSBF            PWR_CR_CSBF_Msk                                 /*!< Clear Standby Flag                  */
#define PWR_CR_PVDE_Pos        (4U)                                            
#define PWR_CR_PVDE_Msk        (0x1U << PWR_CR_PVDE_Pos)                       /*!< 0x00000010 */
#define PWR_CR_PVDE            PWR_CR_PVDE_Msk                                 /*!< Power Voltage Detector Enable       */

#define PWR_CR_PLS_Pos         (5U)                                            
#define PWR_CR_PLS_Msk         (0x7U << PWR_CR_PLS_Pos)                        /*!< 0x000000E0 */
#define PWR_CR_PLS             PWR_CR_PLS_Msk                                  /*!< PLS[2:0] bits (PVD Level Selection) */
#define PWR_CR_PLS_0           (0x1U << PWR_CR_PLS_Pos)                        /*!< 0x00000020 */
#define PWR_CR_PLS_1           (0x2U << PWR_CR_PLS_Pos)                        /*!< 0x00000040 */
#define PWR_CR_PLS_2           (0x4U << PWR_CR_PLS_Pos)                        /*!< 0x00000080 */

/*!< PVD level configuration */
#define PWR_CR_PLS_LEV0        0x00000000U                                     /*!< PVD level 0 */
#define PWR_CR_PLS_LEV1        0x00000020U                                     /*!< PVD level 1 */
#define PWR_CR_PLS_LEV2        0x00000040U                                     /*!< PVD level 2 */
#define PWR_CR_PLS_LEV3        0x00000060U                                     /*!< PVD level 3 */
#define PWR_CR_PLS_LEV4        0x00000080U                                     /*!< PVD level 4 */
#define PWR_CR_PLS_LEV5        0x000000A0U                                     /*!< PVD level 5 */
#define PWR_CR_PLS_LEV6        0x000000C0U                                     /*!< PVD level 6 */
#define PWR_CR_PLS_LEV7        0x000000E0U                                     /*!< PVD level 7 */
#define PWR_CR_DBP_Pos         (8U)                                            
#define PWR_CR_DBP_Msk         (0x1U << PWR_CR_DBP_Pos)                        /*!< 0x00000100 */
#define PWR_CR_DBP             PWR_CR_DBP_Msk                                  /*!< Disable Backup Domain write protection                     */
#define PWR_CR_FPDS_Pos        (9U)                                            
#define PWR_CR_FPDS_Msk        (0x1U << PWR_CR_FPDS_Pos)                       /*!< 0x00000200 */
#define PWR_CR_FPDS            PWR_CR_FPDS_Msk                                 /*!< Flash power down in Stop mode                              */
#define PWR_CR_LPLVDS_Pos      (10U)                                           
#define PWR_CR_LPLVDS_Msk      (0x1U << PWR_CR_LPLVDS_Pos)                     /*!< 0x00000400 */
#define PWR_CR_LPLVDS          PWR_CR_LPLVDS_Msk                               /*!< Low Power Regulator Low Voltage in Deep Sleep mode         */
#define PWR_CR_MRLVDS_Pos      (11U)                                           
#define PWR_CR_MRLVDS_Msk      (0x1U << PWR_CR_MRLVDS_Pos)                     /*!< 0x00000800 */
#define PWR_CR_MRLVDS          PWR_CR_MRLVDS_Msk                               /*!< Main Regulator Low Voltage in Deep Sleep mode              */
#define PWR_CR_ADCDC1_Pos      (13U)                                           
#define PWR_CR_ADCDC1_Msk      (0x1U << PWR_CR_ADCDC1_Pos)                     /*!< 0x00002000 */
#define PWR_CR_ADCDC1          PWR_CR_ADCDC1_Msk                               /*!< Refer to AN4073 on how to use this bit                     */ 
#define PWR_CR_VOS_Pos         (14U)                                           
#define PWR_CR_VOS_Msk         (0x3U << PWR_CR_VOS_Pos)                        /*!< 0x0000C000 */
#define PWR_CR_VOS             PWR_CR_VOS_Msk                                  /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define PWR_CR_VOS_0           0x00004000U                                     /*!< Bit 0 */
#define PWR_CR_VOS_1           0x00008000U                                     /*!< Bit 1 */

/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define PWR_CSR_WUF_Pos        (0U)                                            
#define PWR_CSR_WUF_Msk        (0x1U << PWR_CSR_WUF_Pos)                       /*!< 0x00000001 */
#define PWR_CSR_WUF            PWR_CSR_WUF_Msk                                 /*!< Wakeup Flag                                      */
#define PWR_CSR_SBF_Pos        (1U)                                            
#define PWR_CSR_SBF_Msk        (0x1U << PWR_CSR_SBF_Pos)                       /*!< 0x00000002 */
#define PWR_CSR_SBF            PWR_CSR_SBF_Msk                                 /*!< Standby Flag                                     */
#define PWR_CSR_PVDO_Pos       (2U)                                            
#define PWR_CSR_PVDO_Msk       (0x1U << PWR_CSR_PVDO_Pos)                      /*!< 0x00000004 */
#define PWR_CSR_PVDO           PWR_CSR_PVDO_Msk                                /*!< PVD Output                                       */
#define PWR_CSR_BRR_Pos        (3U)                                            
#define PWR_CSR_BRR_Msk        (0x1U << PWR_CSR_BRR_Pos)                       /*!< 0x00000008 */
#define PWR_CSR_BRR            PWR_CSR_BRR_Msk                                 /*!< Backup regulator ready                           */
#define PWR_CSR_EWUP_Pos       (8U)                                            
#define PWR_CSR_EWUP_Msk       (0x1U << PWR_CSR_EWUP_Pos)                      /*!< 0x00000100 */
#define PWR_CSR_EWUP           PWR_CSR_EWUP_Msk                                /*!< Enable WKUP pin                                  */
#define PWR_CSR_BRE_Pos        (9U)                                            
#define PWR_CSR_BRE_Msk        (0x1U << PWR_CSR_BRE_Pos)                       /*!< 0x00000200 */
#define PWR_CSR_BRE            PWR_CSR_BRE_Msk                                 /*!< Backup regulator enable                          */
#define PWR_CSR_VOSRDY_Pos     (14U)                                           
#define PWR_CSR_VOSRDY_Msk     (0x1U << PWR_CSR_VOSRDY_Pos)                    /*!< 0x00004000 */
#define PWR_CSR_VOSRDY         PWR_CSR_VOSRDY_Msk                              /*!< Regulator voltage scaling output selection ready */

/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                   (0U)                                
#define RCC_CR_HSION_Msk                   (0x1U << RCC_CR_HSION_Pos)          /*!< 0x00000001 */
#define RCC_CR_HSION                       RCC_CR_HSION_Msk                    
#define RCC_CR_HSIRDY_Pos                  (1U)                                
#define RCC_CR_HSIRDY_Msk                  (0x1U << RCC_CR_HSIRDY_Pos)         /*!< 0x00000002 */
#define RCC_CR_HSIRDY                      RCC_CR_HSIRDY_Msk                   

#define RCC_CR_HSITRIM_Pos                 (3U)                                
#define RCC_CR_HSITRIM_Msk                 (0x1FU << RCC_CR_HSITRIM_Pos)       /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                     RCC_CR_HSITRIM_Msk                  
#define RCC_CR_HSITRIM_0                   (0x01U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                   (0x02U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                   (0x04U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                   (0x08U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                   (0x10U << RCC_CR_HSITRIM_Pos)       /*!< 0x00000080 */

#define RCC_CR_HSICAL_Pos                  (8U)                                
#define RCC_CR_HSICAL_Msk                  (0xFFU << RCC_CR_HSICAL_Pos)        /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                      RCC_CR_HSICAL_Msk                   
#define RCC_CR_HSICAL_0                    (0x01U << RCC_CR_HSICAL_Pos)        /*!< 0x00000100 */
#define RCC_CR_HSICAL_1                    (0x02U << RCC_CR_HSICAL_Pos)        /*!< 0x00000200 */
#define RCC_CR_HSICAL_2                    (0x04U << RCC_CR_HSICAL_Pos)        /*!< 0x00000400 */
#define RCC_CR_HSICAL_3                    (0x08U << RCC_CR_HSICAL_Pos)        /*!< 0x00000800 */
#define RCC_CR_HSICAL_4                    (0x10U << RCC_CR_HSICAL_Pos)        /*!< 0x00001000 */
#define RCC_CR_HSICAL_5                    (0x20U << RCC_CR_HSICAL_Pos)        /*!< 0x00002000 */
#define RCC_CR_HSICAL_6                    (0x40U << RCC_CR_HSICAL_Pos)        /*!< 0x00004000 */
#define RCC_CR_HSICAL_7                    (0x80U << RCC_CR_HSICAL_Pos)        /*!< 0x00008000 */

#define RCC_CR_HSEON_Pos                   (16U)                               
#define RCC_CR_HSEON_Msk                   (0x1U << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */
#define RCC_CR_HSEON                       RCC_CR_HSEON_Msk                    
#define RCC_CR_HSERDY_Pos                  (17U)                               
#define RCC_CR_HSERDY_Msk                  (0x1U << RCC_CR_HSERDY_Pos)         /*!< 0x00020000 */
#define RCC_CR_HSERDY                      RCC_CR_HSERDY_Msk                   
#define RCC_CR_HSEBYP_Pos                  (18U)                               
#define RCC_CR_HSEBYP_Msk                  (0x1U << RCC_CR_HSEBYP_Pos)         /*!< 0x00040000 */
#define RCC_CR_HSEBYP                      RCC_CR_HSEBYP_Msk                   
#define RCC_CR_CSSON_Pos                   (19U)                               
#define RCC_CR_CSSON_Msk                   (0x1U << RCC_CR_CSSON_Pos)          /*!< 0x00080000 */
#define RCC_CR_CSSON                       RCC_CR_CSSON_Msk                    
#define RCC_CR_PLLON_Pos                   (24U)                               
#define RCC_CR_PLLON_Msk                   (0x1U << RCC_CR_PLLON_Pos)          /*!< 0x01000000 */
#define RCC_CR_PLLON                       RCC_CR_PLLON_Msk                    
#define RCC_CR_PLLRDY_Pos                  (25U)                               
#define RCC_CR_PLLRDY_Msk                  (0x1U << RCC_CR_PLLRDY_Pos)         /*!< 0x02000000 */
#define RCC_CR_PLLRDY                      RCC_CR_PLLRDY_Msk                   
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLI2S_SUPPORT                                                     /*!< Support PLLI2S oscillator */

#define RCC_CR_PLLI2SON_Pos                (26U)                               
#define RCC_CR_PLLI2SON_Msk                (0x1U << RCC_CR_PLLI2SON_Pos)       /*!< 0x04000000 */
#define RCC_CR_PLLI2SON                    RCC_CR_PLLI2SON_Msk                 
#define RCC_CR_PLLI2SRDY_Pos               (27U)                               
#define RCC_CR_PLLI2SRDY_Msk               (0x1U << RCC_CR_PLLI2SRDY_Pos)      /*!< 0x08000000 */
#define RCC_CR_PLLI2SRDY                   RCC_CR_PLLI2SRDY_Msk                

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLM_Pos               (0U)                                
#define RCC_PLLCFGR_PLLM_Msk               (0x3FU << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x0000003F */
#define RCC_PLLCFGR_PLLM                   RCC_PLLCFGR_PLLM_Msk                
#define RCC_PLLCFGR_PLLM_0                 (0x01U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000001 */
#define RCC_PLLCFGR_PLLM_1                 (0x02U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000002 */
#define RCC_PLLCFGR_PLLM_2                 (0x04U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000004 */
#define RCC_PLLCFGR_PLLM_3                 (0x08U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000008 */
#define RCC_PLLCFGR_PLLM_4                 (0x10U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000010 */
#define RCC_PLLCFGR_PLLM_5                 (0x20U << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x00000020 */

#define RCC_PLLCFGR_PLLN_Pos               (6U)                                
#define RCC_PLLCFGR_PLLN_Msk               (0x1FFU << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00007FC0 */
#define RCC_PLLCFGR_PLLN                   RCC_PLLCFGR_PLLN_Msk                
#define RCC_PLLCFGR_PLLN_0                 (0x001U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000040 */
#define RCC_PLLCFGR_PLLN_1                 (0x002U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000080 */
#define RCC_PLLCFGR_PLLN_2                 (0x004U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000100 */
#define RCC_PLLCFGR_PLLN_3                 (0x008U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000200 */
#define RCC_PLLCFGR_PLLN_4                 (0x010U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000400 */
#define RCC_PLLCFGR_PLLN_5                 (0x020U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00000800 */
#define RCC_PLLCFGR_PLLN_6                 (0x040U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00001000 */
#define RCC_PLLCFGR_PLLN_7                 (0x080U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00002000 */
#define RCC_PLLCFGR_PLLN_8                 (0x100U << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00004000 */

#define RCC_PLLCFGR_PLLP_Pos               (16U)                               
#define RCC_PLLCFGR_PLLP_Msk               (0x3U << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP                   RCC_PLLCFGR_PLLP_Msk                
#define RCC_PLLCFGR_PLLP_0                 (0x1U << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00010000 */
#define RCC_PLLCFGR_PLLP_1                 (0x2U << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00020000 */

#define RCC_PLLCFGR_PLLSRC_Pos             (22U)                               
#define RCC_PLLCFGR_PLLSRC_Msk             (0x1U << RCC_PLLCFGR_PLLSRC_Pos)    /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC                 RCC_PLLCFGR_PLLSRC_Msk              
#define RCC_PLLCFGR_PLLSRC_HSE_Pos         (22U)                               
#define RCC_PLLCFGR_PLLSRC_HSE_Msk         (0x1U << RCC_PLLCFGR_PLLSRC_HSE_Pos) /*!< 0x00400000 */
#define RCC_PLLCFGR_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE_Msk          
#define RCC_PLLCFGR_PLLSRC_HSI             0x00000000U                         

#define RCC_PLLCFGR_PLLQ_Pos               (24U)                               
#define RCC_PLLCFGR_PLLQ_Msk               (0xFU << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x0F000000 */
#define RCC_PLLCFGR_PLLQ                   RCC_PLLCFGR_PLLQ_Msk                
#define RCC_PLLCFGR_PLLQ_0                 (0x1U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x01000000 */
#define RCC_PLLCFGR_PLLQ_1                 (0x2U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x02000000 */
#define RCC_PLLCFGR_PLLQ_2                 (0x4U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x04000000 */
#define RCC_PLLCFGR_PLLQ_3                 (0x8U << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x08000000 */


/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                    (0U)                                
#define RCC_CFGR_SW_Msk                    (0x3U << RCC_CFGR_SW_Pos)           /*!< 0x00000003 */
#define RCC_CFGR_SW                        RCC_CFGR_SW_Msk                     /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                      (0x1U << RCC_CFGR_SW_Pos)           /*!< 0x00000001 */
#define RCC_CFGR_SW_1                      (0x2U << RCC_CFGR_SW_Pos)           /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                    0x00000000U                         /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                    0x00000001U                         /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                    0x00000002U                         /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                   (2U)                                
#define RCC_CFGR_SWS_Msk                   (0x3U << RCC_CFGR_SWS_Pos)          /*!< 0x0000000C */
#define RCC_CFGR_SWS                       RCC_CFGR_SWS_Msk                    /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                     (0x1U << RCC_CFGR_SWS_Pos)          /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                     (0x2U << RCC_CFGR_SWS_Pos)          /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                   0x00000000U                         /*!< HSI oscillator used as system clock        */
#define RCC_CFGR_SWS_HSE                   0x00000004U                         /*!< HSE oscillator used as system clock        */
#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock                   */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                  (4U)                                
#define RCC_CFGR_HPRE_Msk                  (0xFU << RCC_CFGR_HPRE_Pos)         /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                      RCC_CFGR_HPRE_Msk                   /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                    (0x1U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                    (0x2U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                    (0x4U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                    (0x8U << RCC_CFGR_HPRE_Pos)         /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                 (10U)                               
#define RCC_CFGR_PPRE1_Msk                 (0x7U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                     RCC_CFGR_PPRE1_Msk                  /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                   (0x1U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000400 */
#define RCC_CFGR_PPRE1_1                   (0x2U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00000800 */
#define RCC_CFGR_PPRE1_2                   (0x4U << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001000 */

#define RCC_CFGR_PPRE1_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE1_DIV8                0x00001800U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U                         /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)                               
#define RCC_CFGR_PPRE2_Msk                 (0x7U << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                     RCC_CFGR_PPRE2_Msk                  /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                   (0x1U << RCC_CFGR_PPRE2_Pos)        /*!< 0x00002000 */
#define RCC_CFGR_PPRE2_1                   (0x2U << RCC_CFGR_PPRE2_Pos)        /*!< 0x00004000 */
#define RCC_CFGR_PPRE2_2                   (0x4U << RCC_CFGR_PPRE2_Pos)        /*!< 0x00008000 */

#define RCC_CFGR_PPRE2_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE2_DIV4                0x0000A000U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV8                0x0000C000U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE2_DIV16               0x0000E000U                         /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE_Pos                (16U)                               
#define RCC_CFGR_RTCPRE_Msk                (0x1FU << RCC_CFGR_RTCPRE_Pos)      /*!< 0x001F0000 */
#define RCC_CFGR_RTCPRE                    RCC_CFGR_RTCPRE_Msk                 
#define RCC_CFGR_RTCPRE_0                  (0x01U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00010000 */
#define RCC_CFGR_RTCPRE_1                  (0x02U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00020000 */
#define RCC_CFGR_RTCPRE_2                  (0x04U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00040000 */
#define RCC_CFGR_RTCPRE_3                  (0x08U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00080000 */
#define RCC_CFGR_RTCPRE_4                  (0x10U << RCC_CFGR_RTCPRE_Pos)      /*!< 0x00100000 */

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_Pos                  (21U)                               
#define RCC_CFGR_MCO1_Msk                  (0x3U << RCC_CFGR_MCO1_Pos)         /*!< 0x00600000 */
#define RCC_CFGR_MCO1                      RCC_CFGR_MCO1_Msk                   
#define RCC_CFGR_MCO1_0                    (0x1U << RCC_CFGR_MCO1_Pos)         /*!< 0x00200000 */
#define RCC_CFGR_MCO1_1                    (0x2U << RCC_CFGR_MCO1_Pos)         /*!< 0x00400000 */

#define RCC_CFGR_I2SSRC_Pos                (23U)                               
#define RCC_CFGR_I2SSRC_Msk                (0x1U << RCC_CFGR_I2SSRC_Pos)       /*!< 0x00800000 */
#define RCC_CFGR_I2SSRC                    RCC_CFGR_I2SSRC_Msk                 

#define RCC_CFGR_MCO1PRE_Pos               (24U)                               
#define RCC_CFGR_MCO1PRE_Msk               (0x7U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x07000000 */
#define RCC_CFGR_MCO1PRE                   RCC_CFGR_MCO1PRE_Msk                
#define RCC_CFGR_MCO1PRE_0                 (0x1U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x01000000 */
#define RCC_CFGR_MCO1PRE_1                 (0x2U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x02000000 */
#define RCC_CFGR_MCO1PRE_2                 (0x4U << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x04000000 */

#define RCC_CFGR_MCO2PRE_Pos               (27U)                               
#define RCC_CFGR_MCO2PRE_Msk               (0x7U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x38000000 */
#define RCC_CFGR_MCO2PRE                   RCC_CFGR_MCO2PRE_Msk                
#define RCC_CFGR_MCO2PRE_0                 (0x1U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x08000000 */
#define RCC_CFGR_MCO2PRE_1                 (0x2U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x10000000 */
#define RCC_CFGR_MCO2PRE_2                 (0x4U << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x20000000 */

#define RCC_CFGR_MCO2_Pos                  (30U)                               
#define RCC_CFGR_MCO2_Msk                  (0x3U << RCC_CFGR_MCO2_Pos)         /*!< 0xC0000000 */
#define RCC_CFGR_MCO2                      RCC_CFGR_MCO2_Msk                   
#define RCC_CFGR_MCO2_0                    (0x1U << RCC_CFGR_MCO2_Pos)         /*!< 0x40000000 */
#define RCC_CFGR_MCO2_1                    (0x2U << RCC_CFGR_MCO2_Pos)         /*!< 0x80000000 */

/********************  Bit definition for RCC_CIR register  *******************/
#define RCC_CIR_LSIRDYF_Pos                (0U)                                
#define RCC_CIR_LSIRDYF_Msk                (0x1U << RCC_CIR_LSIRDYF_Pos)       /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                    RCC_CIR_LSIRDYF_Msk                 
#define RCC_CIR_LSERDYF_Pos                (1U)                                
#define RCC_CIR_LSERDYF_Msk                (0x1U << RCC_CIR_LSERDYF_Pos)       /*!< 0x00000002 */
#define RCC_CIR_LSERDYF                    RCC_CIR_LSERDYF_Msk                 
#define RCC_CIR_HSIRDYF_Pos                (2U)                                
#define RCC_CIR_HSIRDYF_Msk                (0x1U << RCC_CIR_HSIRDYF_Pos)       /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                    RCC_CIR_HSIRDYF_Msk                 
#define RCC_CIR_HSERDYF_Pos                (3U)                                
#define RCC_CIR_HSERDYF_Msk                (0x1U << RCC_CIR_HSERDYF_Pos)       /*!< 0x00000008 */
#define RCC_CIR_HSERDYF                    RCC_CIR_HSERDYF_Msk                 
#define RCC_CIR_PLLRDYF_Pos                (4U)                                
#define RCC_CIR_PLLRDYF_Msk                (0x1U << RCC_CIR_PLLRDYF_Pos)       /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF                    RCC_CIR_PLLRDYF_Msk                 
#define RCC_CIR_PLLI2SRDYF_Pos             (5U)                                
#define RCC_CIR_PLLI2SRDYF_Msk             (0x1U << RCC_CIR_PLLI2SRDYF_Pos)    /*!< 0x00000020 */
#define RCC_CIR_PLLI2SRDYF                 RCC_CIR_PLLI2SRDYF_Msk              

#define RCC_CIR_CSSF_Pos                   (7U)                                
#define RCC_CIR_CSSF_Msk                   (0x1U << RCC_CIR_CSSF_Pos)          /*!< 0x00000080 */
#define RCC_CIR_CSSF                       RCC_CIR_CSSF_Msk                    
#define RCC_CIR_LSIRDYIE_Pos               (8U)                                
#define RCC_CIR_LSIRDYIE_Msk               (0x1U << RCC_CIR_LSIRDYIE_Pos)      /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                   RCC_CIR_LSIRDYIE_Msk                
#define RCC_CIR_LSERDYIE_Pos               (9U)                                
#define RCC_CIR_LSERDYIE_Msk               (0x1U << RCC_CIR_LSERDYIE_Pos)      /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE                   RCC_CIR_LSERDYIE_Msk                
#define RCC_CIR_HSIRDYIE_Pos               (10U)                               
#define RCC_CIR_HSIRDYIE_Msk               (0x1U << RCC_CIR_HSIRDYIE_Pos)      /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                   RCC_CIR_HSIRDYIE_Msk                
#define RCC_CIR_HSERDYIE_Pos               (11U)                               
#define RCC_CIR_HSERDYIE_Msk               (0x1U << RCC_CIR_HSERDYIE_Pos)      /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE                   RCC_CIR_HSERDYIE_Msk                
#define RCC_CIR_PLLRDYIE_Pos               (12U)                               
#define RCC_CIR_PLLRDYIE_Msk               (0x1U << RCC_CIR_PLLRDYIE_Pos)      /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE                   RCC_CIR_PLLRDYIE_Msk                
#define RCC_CIR_PLLI2SRDYIE_Pos            (13U)                               
#define RCC_CIR_PLLI2SRDYIE_Msk            (0x1U << RCC_CIR_PLLI2SRDYIE_Pos)   /*!< 0x00002000 */
#define RCC_CIR_PLLI2SRDYIE                RCC_CIR_PLLI2SRDYIE_Msk             

#define RCC_CIR_LSIRDYC_Pos                (16U)                               
#define RCC_CIR_LSIRDYC_Msk                (0x1U << RCC_CIR_LSIRDYC_Pos)       /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                    RCC_CIR_LSIRDYC_Msk                 
#define RCC_CIR_LSERDYC_Pos                (17U)                               
#define RCC_CIR_LSERDYC_Msk                (0x1U << RCC_CIR_LSERDYC_Pos)       /*!< 0x00020000 */
#define RCC_CIR_LSERDYC                    RCC_CIR_LSERDYC_Msk                 
#define RCC_CIR_HSIRDYC_Pos                (18U)                               
#define RCC_CIR_HSIRDYC_Msk                (0x1U << RCC_CIR_HSIRDYC_Pos)       /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                    RCC_CIR_HSIRDYC_Msk                 
#define RCC_CIR_HSERDYC_Pos                (19U)                               
#define RCC_CIR_HSERDYC_Msk                (0x1U << RCC_CIR_HSERDYC_Pos)       /*!< 0x00080000 */
#define RCC_CIR_HSERDYC                    RCC_CIR_HSERDYC_Msk                 
#define RCC_CIR_PLLRDYC_Pos                (20U)                               
#define RCC_CIR_PLLRDYC_Msk                (0x1U << RCC_CIR_PLLRDYC_Pos)       /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC                    RCC_CIR_PLLRDYC_Msk                 
#define RCC_CIR_PLLI2SRDYC_Pos             (21U)                               
#define RCC_CIR_PLLI2SRDYC_Msk             (0x1U << RCC_CIR_PLLI2SRDYC_Pos)    /*!< 0x00200000 */
#define RCC_CIR_PLLI2SRDYC                 RCC_CIR_PLLI2SRDYC_Msk              

#define RCC_CIR_CSSC_Pos                   (23U)                               
#define RCC_CIR_CSSC_Msk                   (0x1U << RCC_CIR_CSSC_Pos)          /*!< 0x00800000 */
#define RCC_CIR_CSSC                       RCC_CIR_CSSC_Msk                    

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define RCC_AHB1RSTR_GPIOARST_Pos          (0U)                                
#define RCC_AHB1RSTR_GPIOARST_Msk          (0x1U << RCC_AHB1RSTR_GPIOARST_Pos) /*!< 0x00000001 */
#define RCC_AHB1RSTR_GPIOARST              RCC_AHB1RSTR_GPIOARST_Msk           
#define RCC_AHB1RSTR_GPIOBRST_Pos          (1U)                                
#define RCC_AHB1RSTR_GPIOBRST_Msk          (0x1U << RCC_AHB1RSTR_GPIOBRST_Pos) /*!< 0x00000002 */
#define RCC_AHB1RSTR_GPIOBRST              RCC_AHB1RSTR_GPIOBRST_Msk           
#define RCC_AHB1RSTR_GPIOCRST_Pos          (2U)                                
#define RCC_AHB1RSTR_GPIOCRST_Msk          (0x1U << RCC_AHB1RSTR_GPIOCRST_Pos) /*!< 0x00000004 */
#define RCC_AHB1RSTR_GPIOCRST              RCC_AHB1RSTR_GPIOCRST_Msk           
#define RCC_AHB1RSTR_GPIODRST_Pos          (3U)                                
#define RCC_AHB1RSTR_GPIODRST_Msk          (0x1U << RCC_AHB1RSTR_GPIODRST_Pos) /*!< 0x00000008 */
#define RCC_AHB1RSTR_GPIODRST              RCC_AHB1RSTR_GPIODRST_Msk           
#define RCC_AHB1RSTR_GPIOERST_Pos          (4U)                                
#define RCC_AHB1RSTR_GPIOERST_Msk          (0x1U << RCC_AHB1RSTR_GPIOERST_Pos) /*!< 0x00000010 */
#define RCC_AHB1RSTR_GPIOERST              RCC_AHB1RSTR_GPIOERST_Msk           
#define RCC_AHB1RSTR_GPIOHRST_Pos          (7U)                                
#define RCC_AHB1RSTR_GPIOHRST_Msk          (0x1U << RCC_AHB1RSTR_GPIOHRST_Pos) /*!< 0x00000080 */
#define RCC_AHB1RSTR_GPIOHRST              RCC_AHB1RSTR_GPIOHRST_Msk           
#define RCC_AHB1RSTR_CRCRST_Pos            (12U)                               
#define RCC_AHB1RSTR_CRCRST_Msk            (0x1U << RCC_AHB1RSTR_CRCRST_Pos)   /*!< 0x00001000 */
#define RCC_AHB1RSTR_CRCRST                RCC_AHB1RSTR_CRCRST_Msk             
#define RCC_AHB1RSTR_DMA1RST_Pos           (21U)                               
#define RCC_AHB1RSTR_DMA1RST_Msk           (0x1U << RCC_AHB1RSTR_DMA1RST_Pos)  /*!< 0x00200000 */
#define RCC_AHB1RSTR_DMA1RST               RCC_AHB1RSTR_DMA1RST_Msk            
#define RCC_AHB1RSTR_DMA2RST_Pos           (22U)                               
#define RCC_AHB1RSTR_DMA2RST_Msk           (0x1U << RCC_AHB1RSTR_DMA2RST_Pos)  /*!< 0x00400000 */
#define RCC_AHB1RSTR_DMA2RST               RCC_AHB1RSTR_DMA2RST_Msk            

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define RCC_AHB2RSTR_OTGFSRST_Pos          (7U)                                
#define RCC_AHB2RSTR_OTGFSRST_Msk          (0x1U << RCC_AHB2RSTR_OTGFSRST_Pos) /*!< 0x00000080 */
#define RCC_AHB2RSTR_OTGFSRST              RCC_AHB2RSTR_OTGFSRST_Msk           
/********************  Bit definition for RCC_AHB3RSTR register  **************/


/********************  Bit definition for RCC_APB1RSTR register  **************/
#define RCC_APB1RSTR_TIM2RST_Pos           (0U)                                
#define RCC_APB1RSTR_TIM2RST_Msk           (0x1U << RCC_APB1RSTR_TIM2RST_Pos)  /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST               RCC_APB1RSTR_TIM2RST_Msk            
#define RCC_APB1RSTR_TIM3RST_Pos           (1U)                                
#define RCC_APB1RSTR_TIM3RST_Msk           (0x1U << RCC_APB1RSTR_TIM3RST_Pos)  /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST               RCC_APB1RSTR_TIM3RST_Msk            
#define RCC_APB1RSTR_TIM4RST_Pos           (2U)                                
#define RCC_APB1RSTR_TIM4RST_Msk           (0x1U << RCC_APB1RSTR_TIM4RST_Pos)  /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST               RCC_APB1RSTR_TIM4RST_Msk            
#define RCC_APB1RSTR_TIM5RST_Pos           (3U)                                
#define RCC_APB1RSTR_TIM5RST_Msk           (0x1U << RCC_APB1RSTR_TIM5RST_Pos)  /*!< 0x00000008 */
#define RCC_APB1RSTR_TIM5RST               RCC_APB1RSTR_TIM5RST_Msk            
#define RCC_APB1RSTR_WWDGRST_Pos           (11U)                               
#define RCC_APB1RSTR_WWDGRST_Msk           (0x1U << RCC_APB1RSTR_WWDGRST_Pos)  /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST               RCC_APB1RSTR_WWDGRST_Msk            
#define RCC_APB1RSTR_SPI2RST_Pos           (14U)                               
#define RCC_APB1RSTR_SPI2RST_Msk           (0x1U << RCC_APB1RSTR_SPI2RST_Pos)  /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST               RCC_APB1RSTR_SPI2RST_Msk            
#define RCC_APB1RSTR_SPI3RST_Pos           (15U)                               
#define RCC_APB1RSTR_SPI3RST_Msk           (0x1U << RCC_APB1RSTR_SPI3RST_Pos)  /*!< 0x00008000 */
#define RCC_APB1RSTR_SPI3RST               RCC_APB1RSTR_SPI3RST_Msk            
#define RCC_APB1RSTR_USART2RST_Pos         (17U)                               
#define RCC_APB1RSTR_USART2RST_Msk         (0x1U << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST             RCC_APB1RSTR_USART2RST_Msk          
#define RCC_APB1RSTR_I2C1RST_Pos           (21U)                               
#define RCC_APB1RSTR_I2C1RST_Msk           (0x1U << RCC_APB1RSTR_I2C1RST_Pos)  /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST               RCC_APB1RSTR_I2C1RST_Msk            
#define RCC_APB1RSTR_I2C2RST_Pos           (22U)                               
#define RCC_APB1RSTR_I2C2RST_Msk           (0x1U << RCC_APB1RSTR_I2C2RST_Pos)  /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST               RCC_APB1RSTR_I2C2RST_Msk            
#define RCC_APB1RSTR_I2C3RST_Pos           (23U)                               
#define RCC_APB1RSTR_I2C3RST_Msk           (0x1U << RCC_APB1RSTR_I2C3RST_Pos)  /*!< 0x00800000 */
#define RCC_APB1RSTR_I2C3RST               RCC_APB1RSTR_I2C3RST_Msk            
#define RCC_APB1RSTR_PWRRST_Pos            (28U)                               
#define RCC_APB1RSTR_PWRRST_Msk            (0x1U << RCC_APB1RSTR_PWRRST_Pos)   /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                RCC_APB1RSTR_PWRRST_Msk             

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define RCC_APB2RSTR_TIM1RST_Pos           (0U)                                
#define RCC_APB2RSTR_TIM1RST_Msk           (0x1U << RCC_APB2RSTR_TIM1RST_Pos)  /*!< 0x00000001 */
#define RCC_APB2RSTR_TIM1RST               RCC_APB2RSTR_TIM1RST_Msk            
#define RCC_APB2RSTR_USART1RST_Pos         (4U)                                
#define RCC_APB2RSTR_USART1RST_Msk         (0x1U << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_USART1RST             RCC_APB2RSTR_USART1RST_Msk          
#define RCC_APB2RSTR_USART6RST_Pos         (5U)                                
#define RCC_APB2RSTR_USART6RST_Msk         (0x1U << RCC_APB2RSTR_USART6RST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_USART6RST             RCC_APB2RSTR_USART6RST_Msk          
#define RCC_APB2RSTR_ADCRST_Pos            (8U)                                
#define RCC_APB2RSTR_ADCRST_Msk            (0x1U << RCC_APB2RSTR_ADCRST_Pos)   /*!< 0x00000100 */
#define RCC_APB2RSTR_ADCRST                RCC_APB2RSTR_ADCRST_Msk             
#define RCC_APB2RSTR_SDIORST_Pos           (11U)                               
#define RCC_APB2RSTR_SDIORST_Msk           (0x1U << RCC_APB2RSTR_SDIORST_Pos)  /*!< 0x00000800 */
#define RCC_APB2RSTR_SDIORST               RCC_APB2RSTR_SDIORST_Msk            
#define RCC_APB2RSTR_SPI1RST_Pos           (12U)                               
#define RCC_APB2RSTR_SPI1RST_Msk           (0x1U << RCC_APB2RSTR_SPI1RST_Pos)  /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST               RCC_APB2RSTR_SPI1RST_Msk            
#define RCC_APB2RSTR_SPI4RST_Pos           (13U)                               
#define RCC_APB2RSTR_SPI4RST_Msk           (0x1U << RCC_APB2RSTR_SPI4RST_Pos)  /*!< 0x00002000 */
#define RCC_APB2RSTR_SPI4RST               RCC_APB2RSTR_SPI4RST_Msk            
#define RCC_APB2RSTR_SYSCFGRST_Pos         (14U)                               
#define RCC_APB2RSTR_SYSCFGRST_Msk         (0x1U << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_SYSCFGRST             RCC_APB2RSTR_SYSCFGRST_Msk          
#define RCC_APB2RSTR_TIM9RST_Pos           (16U)                               
#define RCC_APB2RSTR_TIM9RST_Msk           (0x1U << RCC_APB2RSTR_TIM9RST_Pos)  /*!< 0x00010000 */
#define RCC_APB2RSTR_TIM9RST               RCC_APB2RSTR_TIM9RST_Msk            
#define RCC_APB2RSTR_TIM10RST_Pos          (17U)                               
#define RCC_APB2RSTR_TIM10RST_Msk          (0x1U << RCC_APB2RSTR_TIM10RST_Pos) /*!< 0x00020000 */
#define RCC_APB2RSTR_TIM10RST              RCC_APB2RSTR_TIM10RST_Msk           
#define RCC_APB2RSTR_TIM11RST_Pos          (18U)                               
#define RCC_APB2RSTR_TIM11RST_Msk          (0x1U << RCC_APB2RSTR_TIM11RST_Pos) /*!< 0x00040000 */
#define RCC_APB2RSTR_TIM11RST              RCC_APB2RSTR_TIM11RST_Msk           

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)                                
#define RCC_AHB1ENR_GPIOAEN_Msk            (0x1U << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */
#define RCC_AHB1ENR_GPIOAEN                RCC_AHB1ENR_GPIOAEN_Msk             
#define RCC_AHB1ENR_GPIOBEN_Pos            (1U)                                
#define RCC_AHB1ENR_GPIOBEN_Msk            (0x1U << RCC_AHB1ENR_GPIOBEN_Pos)   /*!< 0x00000002 */
#define RCC_AHB1ENR_GPIOBEN                RCC_AHB1ENR_GPIOBEN_Msk             
#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)                                
#define RCC_AHB1ENR_GPIOCEN_Msk            (0x1U << RCC_AHB1ENR_GPIOCEN_Pos)   /*!< 0x00000004 */
#define RCC_AHB1ENR_GPIOCEN                RCC_AHB1ENR_GPIOCEN_Msk             
#define RCC_AHB1ENR_GPIODEN_Pos            (3U)                                
#define RCC_AHB1ENR_GPIODEN_Msk            (0x1U << RCC_AHB1ENR_GPIODEN_Pos)   /*!< 0x00000008 */
#define RCC_AHB1ENR_GPIODEN                RCC_AHB1ENR_GPIODEN_Msk             
#define RCC_AHB1ENR_GPIOEEN_Pos            (4U)                                
#define RCC_AHB1ENR_GPIOEEN_Msk            (0x1U << RCC_AHB1ENR_GPIOEEN_Pos)   /*!< 0x00000010 */
#define RCC_AHB1ENR_GPIOEEN                RCC_AHB1ENR_GPIOEEN_Msk             
#define RCC_AHB1ENR_GPIOHEN_Pos            (7U)                                
#define RCC_AHB1ENR_GPIOHEN_Msk            (0x1U << RCC_AHB1ENR_GPIOHEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB1ENR_GPIOHEN                RCC_AHB1ENR_GPIOHEN_Msk             
#define RCC_AHB1ENR_CRCEN_Pos              (12U)                               
#define RCC_AHB1ENR_CRCEN_Msk              (0x1U << RCC_AHB1ENR_CRCEN_Pos)     /*!< 0x00001000 */
#define RCC_AHB1ENR_CRCEN                  RCC_AHB1ENR_CRCEN_Msk               
#define RCC_AHB1ENR_DMA1EN_Pos             (21U)                               
#define RCC_AHB1ENR_DMA1EN_Msk             (0x1U << RCC_AHB1ENR_DMA1EN_Pos)    /*!< 0x00200000 */
#define RCC_AHB1ENR_DMA1EN                 RCC_AHB1ENR_DMA1EN_Msk              
#define RCC_AHB1ENR_DMA2EN_Pos             (22U)                               
#define RCC_AHB1ENR_DMA2EN_Msk             (0x1U << RCC_AHB1ENR_DMA2EN_Pos)    /*!< 0x00400000 */
#define RCC_AHB1ENR_DMA2EN                 RCC_AHB1ENR_DMA2EN_Msk              
/********************  Bit definition for RCC_AHB2ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB2_SUPPORT                   /*!< AHB2 Bus is supported */

#define RCC_AHB2ENR_OTGFSEN_Pos            (7U)                                
#define RCC_AHB2ENR_OTGFSEN_Msk            (0x1U << RCC_AHB2ENR_OTGFSEN_Pos)   /*!< 0x00000080 */
#define RCC_AHB2ENR_OTGFSEN                RCC_AHB2ENR_OTGFSEN_Msk             

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_TIM2EN_Pos             (0U)                                
#define RCC_APB1ENR_TIM2EN_Msk             (0x1U << RCC_APB1ENR_TIM2EN_Pos)    /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                 RCC_APB1ENR_TIM2EN_Msk              
#define RCC_APB1ENR_TIM3EN_Pos             (1U)                                
#define RCC_APB1ENR_TIM3EN_Msk             (0x1U << RCC_APB1ENR_TIM3EN_Pos)    /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN                 RCC_APB1ENR_TIM3EN_Msk              
#define RCC_APB1ENR_TIM4EN_Pos             (2U)                                
#define RCC_APB1ENR_TIM4EN_Msk             (0x1U << RCC_APB1ENR_TIM4EN_Pos)    /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN                 RCC_APB1ENR_TIM4EN_Msk              
#define RCC_APB1ENR_TIM5EN_Pos             (3U)                                
#define RCC_APB1ENR_TIM5EN_Msk             (0x1U << RCC_APB1ENR_TIM5EN_Pos)    /*!< 0x00000008 */
#define RCC_APB1ENR_TIM5EN                 RCC_APB1ENR_TIM5EN_Msk              
#define RCC_APB1ENR_WWDGEN_Pos             (11U)                               
#define RCC_APB1ENR_WWDGEN_Msk             (0x1U << RCC_APB1ENR_WWDGEN_Pos)    /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                 RCC_APB1ENR_WWDGEN_Msk              
#define RCC_APB1ENR_SPI2EN_Pos             (14U)                               
#define RCC_APB1ENR_SPI2EN_Msk             (0x1U << RCC_APB1ENR_SPI2EN_Pos)    /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN                 RCC_APB1ENR_SPI2EN_Msk              
#define RCC_APB1ENR_SPI3EN_Pos             (15U)                               
#define RCC_APB1ENR_SPI3EN_Msk             (0x1U << RCC_APB1ENR_SPI3EN_Pos)    /*!< 0x00008000 */
#define RCC_APB1ENR_SPI3EN                 RCC_APB1ENR_SPI3EN_Msk              
#define RCC_APB1ENR_USART2EN_Pos           (17U)                               
#define RCC_APB1ENR_USART2EN_Msk           (0x1U << RCC_APB1ENR_USART2EN_Pos)  /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN               RCC_APB1ENR_USART2EN_Msk            
#define RCC_APB1ENR_I2C1EN_Pos             (21U)                               
#define RCC_APB1ENR_I2C1EN_Msk             (0x1U << RCC_APB1ENR_I2C1EN_Pos)    /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                 RCC_APB1ENR_I2C1EN_Msk              
#define RCC_APB1ENR_I2C2EN_Pos             (22U)                               
#define RCC_APB1ENR_I2C2EN_Msk             (0x1U << RCC_APB1ENR_I2C2EN_Pos)    /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                 RCC_APB1ENR_I2C2EN_Msk              
#define RCC_APB1ENR_I2C3EN_Pos             (23U)                               
#define RCC_APB1ENR_I2C3EN_Msk             (0x1U << RCC_APB1ENR_I2C3EN_Pos)    /*!< 0x00800000 */
#define RCC_APB1ENR_I2C3EN                 RCC_APB1ENR_I2C3EN_Msk              
#define RCC_APB1ENR_PWREN_Pos              (28U)                               
#define RCC_APB1ENR_PWREN_Msk              (0x1U << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                  RCC_APB1ENR_PWREN_Msk               

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define RCC_APB2ENR_TIM1EN_Pos             (0U)                                
#define RCC_APB2ENR_TIM1EN_Msk             (0x1U << RCC_APB2ENR_TIM1EN_Pos)    /*!< 0x00000001 */
#define RCC_APB2ENR_TIM1EN                 RCC_APB2ENR_TIM1EN_Msk              
#define RCC_APB2ENR_USART1EN_Pos           (4U)                                
#define RCC_APB2ENR_USART1EN_Msk           (0x1U << RCC_APB2ENR_USART1EN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_USART1EN               RCC_APB2ENR_USART1EN_Msk            
#define RCC_APB2ENR_USART6EN_Pos           (5U)                                
#define RCC_APB2ENR_USART6EN_Msk           (0x1U << RCC_APB2ENR_USART6EN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_USART6EN               RCC_APB2ENR_USART6EN_Msk            
#define RCC_APB2ENR_ADC1EN_Pos             (8U)                                
#define RCC_APB2ENR_ADC1EN_Msk             (0x1U << RCC_APB2ENR_ADC1EN_Pos)    /*!< 0x00000100 */
#define RCC_APB2ENR_ADC1EN                 RCC_APB2ENR_ADC1EN_Msk              
#define RCC_APB2ENR_SDIOEN_Pos             (11U)                               
#define RCC_APB2ENR_SDIOEN_Msk             (0x1U << RCC_APB2ENR_SDIOEN_Pos)    /*!< 0x00000800 */
#define RCC_APB2ENR_SDIOEN                 RCC_APB2ENR_SDIOEN_Msk              
#define RCC_APB2ENR_SPI1EN_Pos             (12U)                               
#define RCC_APB2ENR_SPI1EN_Msk             (0x1U << RCC_APB2ENR_SPI1EN_Pos)    /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                 RCC_APB2ENR_SPI1EN_Msk              
#define RCC_APB2ENR_SPI4EN_Pos             (13U)                               
#define RCC_APB2ENR_SPI4EN_Msk             (0x1U << RCC_APB2ENR_SPI4EN_Pos)    /*!< 0x00002000 */
#define RCC_APB2ENR_SPI4EN                 RCC_APB2ENR_SPI4EN_Msk              
#define RCC_APB2ENR_SYSCFGEN_Pos           (14U)                               
#define RCC_APB2ENR_SYSCFGEN_Msk           (0x1U << RCC_APB2ENR_SYSCFGEN_Pos)  /*!< 0x00004000 */
#define RCC_APB2ENR_SYSCFGEN               RCC_APB2ENR_SYSCFGEN_Msk            
#define RCC_APB2ENR_TIM9EN_Pos             (16U)                               
#define RCC_APB2ENR_TIM9EN_Msk             (0x1U << RCC_APB2ENR_TIM9EN_Pos)    /*!< 0x00010000 */
#define RCC_APB2ENR_TIM9EN                 RCC_APB2ENR_TIM9EN_Msk              
#define RCC_APB2ENR_TIM10EN_Pos            (17U)                               
#define RCC_APB2ENR_TIM10EN_Msk            (0x1U << RCC_APB2ENR_TIM10EN_Pos)   /*!< 0x00020000 */
#define RCC_APB2ENR_TIM10EN                RCC_APB2ENR_TIM10EN_Msk             
#define RCC_APB2ENR_TIM11EN_Pos            (18U)                               
#define RCC_APB2ENR_TIM11EN_Msk            (0x1U << RCC_APB2ENR_TIM11EN_Pos)   /*!< 0x00040000 */
#define RCC_APB2ENR_TIM11EN                RCC_APB2ENR_TIM11EN_Msk             

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define RCC_AHB1LPENR_GPIOALPEN_Pos        (0U)                                
#define RCC_AHB1LPENR_GPIOALPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOALPEN_Pos) /*!< 0x00000001 */
#define RCC_AHB1LPENR_GPIOALPEN            RCC_AHB1LPENR_GPIOALPEN_Msk         
#define RCC_AHB1LPENR_GPIOBLPEN_Pos        (1U)                                
#define RCC_AHB1LPENR_GPIOBLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOBLPEN_Pos) /*!< 0x00000002 */
#define RCC_AHB1LPENR_GPIOBLPEN            RCC_AHB1LPENR_GPIOBLPEN_Msk         
#define RCC_AHB1LPENR_GPIOCLPEN_Pos        (2U)                                
#define RCC_AHB1LPENR_GPIOCLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOCLPEN_Pos) /*!< 0x00000004 */
#define RCC_AHB1LPENR_GPIOCLPEN            RCC_AHB1LPENR_GPIOCLPEN_Msk         
#define RCC_AHB1LPENR_GPIODLPEN_Pos        (3U)                                
#define RCC_AHB1LPENR_GPIODLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIODLPEN_Pos) /*!< 0x00000008 */
#define RCC_AHB1LPENR_GPIODLPEN            RCC_AHB1LPENR_GPIODLPEN_Msk         
#define RCC_AHB1LPENR_GPIOELPEN_Pos        (4U)                                
#define RCC_AHB1LPENR_GPIOELPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOELPEN_Pos) /*!< 0x00000010 */
#define RCC_AHB1LPENR_GPIOELPEN            RCC_AHB1LPENR_GPIOELPEN_Msk         
#define RCC_AHB1LPENR_GPIOHLPEN_Pos        (7U)                                
#define RCC_AHB1LPENR_GPIOHLPEN_Msk        (0x1U << RCC_AHB1LPENR_GPIOHLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB1LPENR_GPIOHLPEN            RCC_AHB1LPENR_GPIOHLPEN_Msk         
#define RCC_AHB1LPENR_CRCLPEN_Pos          (12U)                               
#define RCC_AHB1LPENR_CRCLPEN_Msk          (0x1U << RCC_AHB1LPENR_CRCLPEN_Pos) /*!< 0x00001000 */
#define RCC_AHB1LPENR_CRCLPEN              RCC_AHB1LPENR_CRCLPEN_Msk           
#define RCC_AHB1LPENR_FLITFLPEN_Pos        (15U)                               
#define RCC_AHB1LPENR_FLITFLPEN_Msk        (0x1U << RCC_AHB1LPENR_FLITFLPEN_Pos) /*!< 0x00008000 */
#define RCC_AHB1LPENR_FLITFLPEN            RCC_AHB1LPENR_FLITFLPEN_Msk         
#define RCC_AHB1LPENR_SRAM1LPEN_Pos        (16U)                               
#define RCC_AHB1LPENR_SRAM1LPEN_Msk        (0x1U << RCC_AHB1LPENR_SRAM1LPEN_Pos) /*!< 0x00010000 */
#define RCC_AHB1LPENR_SRAM1LPEN            RCC_AHB1LPENR_SRAM1LPEN_Msk         
#define RCC_AHB1LPENR_DMA1LPEN_Pos         (21U)                               
#define RCC_AHB1LPENR_DMA1LPEN_Msk         (0x1U << RCC_AHB1LPENR_DMA1LPEN_Pos) /*!< 0x00200000 */
#define RCC_AHB1LPENR_DMA1LPEN             RCC_AHB1LPENR_DMA1LPEN_Msk          
#define RCC_AHB1LPENR_DMA2LPEN_Pos         (22U)                               
#define RCC_AHB1LPENR_DMA2LPEN_Msk         (0x1U << RCC_AHB1LPENR_DMA2LPEN_Pos) /*!< 0x00400000 */
#define RCC_AHB1LPENR_DMA2LPEN             RCC_AHB1LPENR_DMA2LPEN_Msk          


/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define RCC_AHB2LPENR_OTGFSLPEN_Pos        (7U)                                
#define RCC_AHB2LPENR_OTGFSLPEN_Msk        (0x1U << RCC_AHB2LPENR_OTGFSLPEN_Pos) /*!< 0x00000080 */
#define RCC_AHB2LPENR_OTGFSLPEN            RCC_AHB2LPENR_OTGFSLPEN_Msk         

/********************  Bit definition for RCC_AHB3LPENR register  *************/

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define RCC_APB1LPENR_TIM2LPEN_Pos         (0U)                                
#define RCC_APB1LPENR_TIM2LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM2LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB1LPENR_TIM2LPEN             RCC_APB1LPENR_TIM2LPEN_Msk          
#define RCC_APB1LPENR_TIM3LPEN_Pos         (1U)                                
#define RCC_APB1LPENR_TIM3LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM3LPEN_Pos) /*!< 0x00000002 */
#define RCC_APB1LPENR_TIM3LPEN             RCC_APB1LPENR_TIM3LPEN_Msk          
#define RCC_APB1LPENR_TIM4LPEN_Pos         (2U)                                
#define RCC_APB1LPENR_TIM4LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM4LPEN_Pos) /*!< 0x00000004 */
#define RCC_APB1LPENR_TIM4LPEN             RCC_APB1LPENR_TIM4LPEN_Msk          
#define RCC_APB1LPENR_TIM5LPEN_Pos         (3U)                                
#define RCC_APB1LPENR_TIM5LPEN_Msk         (0x1U << RCC_APB1LPENR_TIM5LPEN_Pos) /*!< 0x00000008 */
#define RCC_APB1LPENR_TIM5LPEN             RCC_APB1LPENR_TIM5LPEN_Msk          
#define RCC_APB1LPENR_WWDGLPEN_Pos         (11U)                               
#define RCC_APB1LPENR_WWDGLPEN_Msk         (0x1U << RCC_APB1LPENR_WWDGLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB1LPENR_WWDGLPEN             RCC_APB1LPENR_WWDGLPEN_Msk          
#define RCC_APB1LPENR_SPI2LPEN_Pos         (14U)                               
#define RCC_APB1LPENR_SPI2LPEN_Msk         (0x1U << RCC_APB1LPENR_SPI2LPEN_Pos) /*!< 0x00004000 */
#define RCC_APB1LPENR_SPI2LPEN             RCC_APB1LPENR_SPI2LPEN_Msk          
#define RCC_APB1LPENR_SPI3LPEN_Pos         (15U)                               
#define RCC_APB1LPENR_SPI3LPEN_Msk         (0x1U << RCC_APB1LPENR_SPI3LPEN_Pos) /*!< 0x00008000 */
#define RCC_APB1LPENR_SPI3LPEN             RCC_APB1LPENR_SPI3LPEN_Msk          
#define RCC_APB1LPENR_USART2LPEN_Pos       (17U)                               
#define RCC_APB1LPENR_USART2LPEN_Msk       (0x1U << RCC_APB1LPENR_USART2LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB1LPENR_USART2LPEN           RCC_APB1LPENR_USART2LPEN_Msk        
#define RCC_APB1LPENR_I2C1LPEN_Pos         (21U)                               
#define RCC_APB1LPENR_I2C1LPEN_Msk         (0x1U << RCC_APB1LPENR_I2C1LPEN_Pos) /*!< 0x00200000 */
#define RCC_APB1LPENR_I2C1LPEN             RCC_APB1LPENR_I2C1LPEN_Msk          
#define RCC_APB1LPENR_I2C2LPEN_Pos         (22U)                               
#define RCC_APB1LPENR_I2C2LPEN_Msk         (0x1U << RCC_APB1LPENR_I2C2LPEN_Pos) /*!< 0x00400000 */
#define RCC_APB1LPENR_I2C2LPEN             RCC_APB1LPENR_I2C2LPEN_Msk          
#define RCC_APB1LPENR_I2C3LPEN_Pos         (23U)                               
#define RCC_APB1LPENR_I2C3LPEN_Msk         (0x1U << RCC_APB1LPENR_I2C3LPEN_Pos) /*!< 0x00800000 */
#define RCC_APB1LPENR_I2C3LPEN             RCC_APB1LPENR_I2C3LPEN_Msk          
#define RCC_APB1LPENR_PWRLPEN_Pos          (28U)                               
#define RCC_APB1LPENR_PWRLPEN_Msk          (0x1U << RCC_APB1LPENR_PWRLPEN_Pos) /*!< 0x10000000 */
#define RCC_APB1LPENR_PWRLPEN              RCC_APB1LPENR_PWRLPEN_Msk           

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define RCC_APB2LPENR_TIM1LPEN_Pos         (0U)                                
#define RCC_APB2LPENR_TIM1LPEN_Msk         (0x1U << RCC_APB2LPENR_TIM1LPEN_Pos) /*!< 0x00000001 */
#define RCC_APB2LPENR_TIM1LPEN             RCC_APB2LPENR_TIM1LPEN_Msk          
#define RCC_APB2LPENR_USART1LPEN_Pos       (4U)                                
#define RCC_APB2LPENR_USART1LPEN_Msk       (0x1U << RCC_APB2LPENR_USART1LPEN_Pos) /*!< 0x00000010 */
#define RCC_APB2LPENR_USART1LPEN           RCC_APB2LPENR_USART1LPEN_Msk        
#define RCC_APB2LPENR_USART6LPEN_Pos       (5U)                                
#define RCC_APB2LPENR_USART6LPEN_Msk       (0x1U << RCC_APB2LPENR_USART6LPEN_Pos) /*!< 0x00000020 */
#define RCC_APB2LPENR_USART6LPEN           RCC_APB2LPENR_USART6LPEN_Msk        
#define RCC_APB2LPENR_ADC1LPEN_Pos         (8U)                                
#define RCC_APB2LPENR_ADC1LPEN_Msk         (0x1U << RCC_APB2LPENR_ADC1LPEN_Pos) /*!< 0x00000100 */
#define RCC_APB2LPENR_ADC1LPEN             RCC_APB2LPENR_ADC1LPEN_Msk          
#define RCC_APB2LPENR_SDIOLPEN_Pos         (11U)                               
#define RCC_APB2LPENR_SDIOLPEN_Msk         (0x1U << RCC_APB2LPENR_SDIOLPEN_Pos) /*!< 0x00000800 */
#define RCC_APB2LPENR_SDIOLPEN             RCC_APB2LPENR_SDIOLPEN_Msk          
#define RCC_APB2LPENR_SPI1LPEN_Pos         (12U)                               
#define RCC_APB2LPENR_SPI1LPEN_Msk         (0x1U << RCC_APB2LPENR_SPI1LPEN_Pos) /*!< 0x00001000 */
#define RCC_APB2LPENR_SPI1LPEN             RCC_APB2LPENR_SPI1LPEN_Msk          
#define RCC_APB2LPENR_SPI4LPEN_Pos         (13U)                               
#define RCC_APB2LPENR_SPI4LPEN_Msk         (0x1U << RCC_APB2LPENR_SPI4LPEN_Pos) /*!< 0x00002000 */
#define RCC_APB2LPENR_SPI4LPEN             RCC_APB2LPENR_SPI4LPEN_Msk          
#define RCC_APB2LPENR_SYSCFGLPEN_Pos       (14U)                               
#define RCC_APB2LPENR_SYSCFGLPEN_Msk       (0x1U << RCC_APB2LPENR_SYSCFGLPEN_Pos) /*!< 0x00004000 */
#define RCC_APB2LPENR_SYSCFGLPEN           RCC_APB2LPENR_SYSCFGLPEN_Msk        
#define RCC_APB2LPENR_TIM9LPEN_Pos         (16U)                               
#define RCC_APB2LPENR_TIM9LPEN_Msk         (0x1U << RCC_APB2LPENR_TIM9LPEN_Pos) /*!< 0x00010000 */
#define RCC_APB2LPENR_TIM9LPEN             RCC_APB2LPENR_TIM9LPEN_Msk          
#define RCC_APB2LPENR_TIM10LPEN_Pos        (17U)                               
#define RCC_APB2LPENR_TIM10LPEN_Msk        (0x1U << RCC_APB2LPENR_TIM10LPEN_Pos) /*!< 0x00020000 */
#define RCC_APB2LPENR_TIM10LPEN            RCC_APB2LPENR_TIM10LPEN_Msk         
#define RCC_APB2LPENR_TIM11LPEN_Pos        (18U)                               
#define RCC_APB2LPENR_TIM11LPEN_Msk        (0x1U << RCC_APB2LPENR_TIM11LPEN_Pos) /*!< 0x00040000 */
#define RCC_APB2LPENR_TIM11LPEN            RCC_APB2LPENR_TIM11LPEN_Msk         

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos                 (0U)                                
#define RCC_BDCR_LSEON_Msk                 (0x1U << RCC_BDCR_LSEON_Pos)        /*!< 0x00000001 */
#define RCC_BDCR_LSEON                     RCC_BDCR_LSEON_Msk                  
#define RCC_BDCR_LSERDY_Pos                (1U)                                
#define RCC_BDCR_LSERDY_Msk                (0x1U << RCC_BDCR_LSERDY_Pos)       /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                    RCC_BDCR_LSERDY_Msk                 
#define RCC_BDCR_LSEBYP_Pos                (2U)                                
#define RCC_BDCR_LSEBYP_Msk                (0x1U << RCC_BDCR_LSEBYP_Pos)       /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                    RCC_BDCR_LSEBYP_Msk                 

#define RCC_BDCR_RTCSEL_Pos                (8U)                                
#define RCC_BDCR_RTCSEL_Msk                (0x3U << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                    RCC_BDCR_RTCSEL_Msk                 
#define RCC_BDCR_RTCSEL_0                  (0x1U << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                  (0x2U << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000200 */

#define RCC_BDCR_RTCEN_Pos                 (15U)                               
#define RCC_BDCR_RTCEN_Msk                 (0x1U << RCC_BDCR_RTCEN_Pos)        /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                     RCC_BDCR_RTCEN_Msk                  
#define RCC_BDCR_BDRST_Pos                 (16U)                               
#define RCC_BDCR_BDRST_Msk                 (0x1U << RCC_BDCR_BDRST_Pos)        /*!< 0x00010000 */
#define RCC_BDCR_BDRST                     RCC_BDCR_BDRST_Msk                  

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                  (0U)                                
#define RCC_CSR_LSION_Msk                  (0x1U << RCC_CSR_LSION_Pos)         /*!< 0x00000001 */
#define RCC_CSR_LSION                      RCC_CSR_LSION_Msk                   
#define RCC_CSR_LSIRDY_Pos                 (1U)                                
#define RCC_CSR_LSIRDY_Msk                 (0x1U << RCC_CSR_LSIRDY_Pos)        /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                     RCC_CSR_LSIRDY_Msk                  
#define RCC_CSR_RMVF_Pos                   (24U)                               
#define RCC_CSR_RMVF_Msk                   (0x1U << RCC_CSR_RMVF_Pos)          /*!< 0x01000000 */
#define RCC_CSR_RMVF                       RCC_CSR_RMVF_Msk                    
#define RCC_CSR_BORRSTF_Pos                (25U)                               
#define RCC_CSR_BORRSTF_Msk                (0x1U << RCC_CSR_BORRSTF_Pos)       /*!< 0x02000000 */
#define RCC_CSR_BORRSTF                    RCC_CSR_BORRSTF_Msk                 
#define RCC_CSR_PINRSTF_Pos                (26U)
#define RCC_CSR_PINRSTF_Msk                (0x1U << RCC_CSR_PINRSTF_Pos)       /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                    RCC_CSR_PINRSTF_Msk
#define RCC_CSR_PORRSTF_Pos                (27U)                               
#define RCC_CSR_PORRSTF_Msk                (0x1U << RCC_CSR_PORRSTF_Pos)       /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                    RCC_CSR_PORRSTF_Msk                 
#define RCC_CSR_SFTRSTF_Pos                (28U)                               
#define RCC_CSR_SFTRSTF_Msk                (0x1U << RCC_CSR_SFTRSTF_Pos)       /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                    RCC_CSR_SFTRSTF_Msk                 
#define RCC_CSR_IWDGRSTF_Pos               (29U)
#define RCC_CSR_IWDGRSTF_Msk               (0x1U << RCC_CSR_IWDGRSTF_Pos)      /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                   RCC_CSR_IWDGRSTF_Msk
#define RCC_CSR_WWDGRSTF_Pos               (30U)                               
#define RCC_CSR_WWDGRSTF_Msk               (0x1U << RCC_CSR_WWDGRSTF_Pos)      /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                   RCC_CSR_WWDGRSTF_Msk                
#define RCC_CSR_LPWRRSTF_Pos               (31U)                               
#define RCC_CSR_LPWRRSTF_Msk               (0x1U << RCC_CSR_LPWRRSTF_Pos)      /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                   RCC_CSR_LPWRRSTF_Msk
/* Legacy defines */
#define RCC_CSR_PADRSTF                    RCC_CSR_PINRSTF
#define RCC_CSR_WDGRSTF                    RCC_CSR_IWDGRSTF

/********************  Bit definition for RCC_SSCGR register  *****************/
#define RCC_SSCGR_MODPER_Pos               (0U)                                
#define RCC_SSCGR_MODPER_Msk               (0x1FFFU << RCC_SSCGR_MODPER_Pos)   /*!< 0x00001FFF */
#define RCC_SSCGR_MODPER                   RCC_SSCGR_MODPER_Msk                
#define RCC_SSCGR_INCSTEP_Pos              (13U)                               
#define RCC_SSCGR_INCSTEP_Msk              (0x7FFFU << RCC_SSCGR_INCSTEP_Pos)  /*!< 0x0FFFE000 */
#define RCC_SSCGR_INCSTEP                  RCC_SSCGR_INCSTEP_Msk               
#define RCC_SSCGR_SPREADSEL_Pos            (30U)                               
#define RCC_SSCGR_SPREADSEL_Msk            (0x1U << RCC_SSCGR_SPREADSEL_Pos)   /*!< 0x40000000 */
#define RCC_SSCGR_SPREADSEL                RCC_SSCGR_SPREADSEL_Msk             
#define RCC_SSCGR_SSCGEN_Pos               (31U)                               
#define RCC_SSCGR_SSCGEN_Msk               (0x1U << RCC_SSCGR_SSCGEN_Pos)      /*!< 0x80000000 */
#define RCC_SSCGR_SSCGEN                   RCC_SSCGR_SSCGEN_Msk                

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define RCC_PLLI2SCFGR_PLLI2SN_Pos         (6U)                                
#define RCC_PLLI2SCFGR_PLLI2SN_Msk         (0x1FFU << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00007FC0 */
#define RCC_PLLI2SCFGR_PLLI2SN             RCC_PLLI2SCFGR_PLLI2SN_Msk          
#define RCC_PLLI2SCFGR_PLLI2SN_0           (0x001U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000040 */
#define RCC_PLLI2SCFGR_PLLI2SN_1           (0x002U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000080 */
#define RCC_PLLI2SCFGR_PLLI2SN_2           (0x004U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000100 */
#define RCC_PLLI2SCFGR_PLLI2SN_3           (0x008U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000200 */
#define RCC_PLLI2SCFGR_PLLI2SN_4           (0x010U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000400 */
#define RCC_PLLI2SCFGR_PLLI2SN_5           (0x020U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00000800 */
#define RCC_PLLI2SCFGR_PLLI2SN_6           (0x040U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00001000 */
#define RCC_PLLI2SCFGR_PLLI2SN_7           (0x080U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00002000 */
#define RCC_PLLI2SCFGR_PLLI2SN_8           (0x100U << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00004000 */

#define RCC_PLLI2SCFGR_PLLI2SR_Pos         (28U)                               
#define RCC_PLLI2SCFGR_PLLI2SR_Msk         (0x7U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x70000000 */
#define RCC_PLLI2SCFGR_PLLI2SR             RCC_PLLI2SCFGR_PLLI2SR_Msk          
#define RCC_PLLI2SCFGR_PLLI2SR_0           (0x1U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x10000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_1           (0x2U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x20000000 */
#define RCC_PLLI2SCFGR_PLLI2SR_2           (0x4U << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_DCKCFGR register  ***************/

#define RCC_DCKCFGR_TIMPRE_Pos             (24U)                               
#define RCC_DCKCFGR_TIMPRE_Msk             (0x1U << RCC_DCKCFGR_TIMPRE_Pos)    /*!< 0x01000000 */
#define RCC_DCKCFGR_TIMPRE                 RCC_DCKCFGR_TIMPRE_Msk              


/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM_Pos                 (22U)                                    
#define RTC_TR_PM_Msk                 (0x1U << RTC_TR_PM_Pos)                  /*!< 0x00400000 */
#define RTC_TR_PM                     RTC_TR_PM_Msk                            
#define RTC_TR_HT_Pos                 (20U)                                    
#define RTC_TR_HT_Msk                 (0x3U << RTC_TR_HT_Pos)                  /*!< 0x00300000 */
#define RTC_TR_HT                     RTC_TR_HT_Msk                            
#define RTC_TR_HT_0                   (0x1U << RTC_TR_HT_Pos)                  /*!< 0x00100000 */
#define RTC_TR_HT_1                   (0x2U << RTC_TR_HT_Pos)                  /*!< 0x00200000 */
#define RTC_TR_HU_Pos                 (16U)                                    
#define RTC_TR_HU_Msk                 (0xFU << RTC_TR_HU_Pos)                  /*!< 0x000F0000 */
#define RTC_TR_HU                     RTC_TR_HU_Msk                            
#define RTC_TR_HU_0                   (0x1U << RTC_TR_HU_Pos)                  /*!< 0x00010000 */
#define RTC_TR_HU_1                   (0x2U << RTC_TR_HU_Pos)                  /*!< 0x00020000 */
#define RTC_TR_HU_2                   (0x4U << RTC_TR_HU_Pos)                  /*!< 0x00040000 */
#define RTC_TR_HU_3                   (0x8U << RTC_TR_HU_Pos)                  /*!< 0x00080000 */
#define RTC_TR_MNT_Pos                (12U)                                    
#define RTC_TR_MNT_Msk                (0x7U << RTC_TR_MNT_Pos)                 /*!< 0x00007000 */
#define RTC_TR_MNT                    RTC_TR_MNT_Msk                           
#define RTC_TR_MNT_0                  (0x1U << RTC_TR_MNT_Pos)                 /*!< 0x00001000 */
#define RTC_TR_MNT_1                  (0x2U << RTC_TR_MNT_Pos)                 /*!< 0x00002000 */
#define RTC_TR_MNT_2                  (0x4U << RTC_TR_MNT_Pos)                 /*!< 0x00004000 */
#define RTC_TR_MNU_Pos                (8U)                                     
#define RTC_TR_MNU_Msk                (0xFU << RTC_TR_MNU_Pos)                 /*!< 0x00000F00 */
#define RTC_TR_MNU                    RTC_TR_MNU_Msk                           
#define RTC_TR_MNU_0                  (0x1U << RTC_TR_MNU_Pos)                 /*!< 0x00000100 */
#define RTC_TR_MNU_1                  (0x2U << RTC_TR_MNU_Pos)                 /*!< 0x00000200 */
#define RTC_TR_MNU_2                  (0x4U << RTC_TR_MNU_Pos)                 /*!< 0x00000400 */
#define RTC_TR_MNU_3                  (0x8U << RTC_TR_MNU_Pos)                 /*!< 0x00000800 */
#define RTC_TR_ST_Pos                 (4U)                                     
#define RTC_TR_ST_Msk                 (0x7U << RTC_TR_ST_Pos)                  /*!< 0x00000070 */
#define RTC_TR_ST                     RTC_TR_ST_Msk                            
#define RTC_TR_ST_0                   (0x1U << RTC_TR_ST_Pos)                  /*!< 0x00000010 */
#define RTC_TR_ST_1                   (0x2U << RTC_TR_ST_Pos)                  /*!< 0x00000020 */
#define RTC_TR_ST_2                   (0x4U << RTC_TR_ST_Pos)                  /*!< 0x00000040 */
#define RTC_TR_SU_Pos                 (0U)                                     
#define RTC_TR_SU_Msk                 (0xFU << RTC_TR_SU_Pos)                  /*!< 0x0000000F */
#define RTC_TR_SU                     RTC_TR_SU_Msk                            
#define RTC_TR_SU_0                   (0x1U << RTC_TR_SU_Pos)                  /*!< 0x00000001 */
#define RTC_TR_SU_1                   (0x2U << RTC_TR_SU_Pos)                  /*!< 0x00000002 */
#define RTC_TR_SU_2                   (0x4U << RTC_TR_SU_Pos)                  /*!< 0x00000004 */
#define RTC_TR_SU_3                   (0x8U << RTC_TR_SU_Pos)                  /*!< 0x00000008 */

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT_Pos                 (20U)                                    
#define RTC_DR_YT_Msk                 (0xFU << RTC_DR_YT_Pos)                  /*!< 0x00F00000 */
#define RTC_DR_YT                     RTC_DR_YT_Msk                            
#define RTC_DR_YT_0                   (0x1U << RTC_DR_YT_Pos)                  /*!< 0x00100000 */
#define RTC_DR_YT_1                   (0x2U << RTC_DR_YT_Pos)                  /*!< 0x00200000 */
#define RTC_DR_YT_2                   (0x4U << RTC_DR_YT_Pos)                  /*!< 0x00400000 */
#define RTC_DR_YT_3                   (0x8U << RTC_DR_YT_Pos)                  /*!< 0x00800000 */
#define RTC_DR_YU_Pos                 (16U)                                    
#define RTC_DR_YU_Msk                 (0xFU << RTC_DR_YU_Pos)                  /*!< 0x000F0000 */
#define RTC_DR_YU                     RTC_DR_YU_Msk                            
#define RTC_DR_YU_0                   (0x1U << RTC_DR_YU_Pos)                  /*!< 0x00010000 */
#define RTC_DR_YU_1                   (0x2U << RTC_DR_YU_Pos)                  /*!< 0x00020000 */
#define RTC_DR_YU_2                   (0x4U << RTC_DR_YU_Pos)                  /*!< 0x00040000 */
#define RTC_DR_YU_3                   (0x8U << RTC_DR_YU_Pos)                  /*!< 0x00080000 */
#define RTC_DR_WDU_Pos                (13U)                                    
#define RTC_DR_WDU_Msk                (0x7U << RTC_DR_WDU_Pos)                 /*!< 0x0000E000 */
#define RTC_DR_WDU                    RTC_DR_WDU_Msk                           
#define RTC_DR_WDU_0                  (0x1U << RTC_DR_WDU_Pos)                 /*!< 0x00002000 */
#define RTC_DR_WDU_1                  (0x2U << RTC_DR_WDU_Pos)                 /*!< 0x00004000 */
#define RTC_DR_WDU_2                  (0x4U << RTC_DR_WDU_Pos)                 /*!< 0x00008000 */
#define RTC_DR_MT_Pos                 (12U)                                    
#define RTC_DR_MT_Msk                 (0x1U << RTC_DR_MT_Pos)                  /*!< 0x00001000 */
#define RTC_DR_MT                     RTC_DR_MT_Msk                            
#define RTC_DR_MU_Pos                 (8U)                                     
#define RTC_DR_MU_Msk                 (0xFU << RTC_DR_MU_Pos)                  /*!< 0x00000F00 */
#define RTC_DR_MU                     RTC_DR_MU_Msk                            
#define RTC_DR_MU_0                   (0x1U << RTC_DR_MU_Pos)                  /*!< 0x00000100 */
#define RTC_DR_MU_1                   (0x2U << RTC_DR_MU_Pos)                  /*!< 0x00000200 */
#define RTC_DR_MU_2                   (0x4U << RTC_DR_MU_Pos)                  /*!< 0x00000400 */
#define RTC_DR_MU_3                   (0x8U << RTC_DR_MU_Pos)                  /*!< 0x00000800 */
#define RTC_DR_DT_Pos                 (4U)                                     
#define RTC_DR_DT_Msk                 (0x3U << RTC_DR_DT_Pos)                  /*!< 0x00000030 */
#define RTC_DR_DT                     RTC_DR_DT_Msk                            
#define RTC_DR_DT_0                   (0x1U << RTC_DR_DT_Pos)                  /*!< 0x00000010 */
#define RTC_DR_DT_1                   (0x2U << RTC_DR_DT_Pos)                  /*!< 0x00000020 */
#define RTC_DR_DU_Pos                 (0U)                                     
#define RTC_DR_DU_Msk                 (0xFU << RTC_DR_DU_Pos)                  /*!< 0x0000000F */
#define RTC_DR_DU                     RTC_DR_DU_Msk                            
#define RTC_DR_DU_0                   (0x1U << RTC_DR_DU_Pos)                  /*!< 0x00000001 */
#define RTC_DR_DU_1                   (0x2U << RTC_DR_DU_Pos)                  /*!< 0x00000002 */
#define RTC_DR_DU_2                   (0x4U << RTC_DR_DU_Pos)                  /*!< 0x00000004 */
#define RTC_DR_DU_3                   (0x8U << RTC_DR_DU_Pos)                  /*!< 0x00000008 */

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_COE_Pos                (23U)                                    
#define RTC_CR_COE_Msk                (0x1U << RTC_CR_COE_Pos)                 /*!< 0x00800000 */
#define RTC_CR_COE                    RTC_CR_COE_Msk                           
#define RTC_CR_OSEL_Pos               (21U)                                    
#define RTC_CR_OSEL_Msk               (0x3U << RTC_CR_OSEL_Pos)                /*!< 0x00600000 */
#define RTC_CR_OSEL                   RTC_CR_OSEL_Msk                          
#define RTC_CR_OSEL_0                 (0x1U << RTC_CR_OSEL_Pos)                /*!< 0x00200000 */
#define RTC_CR_OSEL_1                 (0x2U << RTC_CR_OSEL_Pos)                /*!< 0x00400000 */
#define RTC_CR_POL_Pos                (20U)                                    
#define RTC_CR_POL_Msk                (0x1U << RTC_CR_POL_Pos)                 /*!< 0x00100000 */
#define RTC_CR_POL                    RTC_CR_POL_Msk                           
#define RTC_CR_COSEL_Pos              (19U)                                    
#define RTC_CR_COSEL_Msk              (0x1U << RTC_CR_COSEL_Pos)               /*!< 0x00080000 */
#define RTC_CR_COSEL                  RTC_CR_COSEL_Msk                         
#define RTC_CR_BKP_Pos                 (18U)                                   
#define RTC_CR_BKP_Msk                 (0x1U << RTC_CR_BKP_Pos)                /*!< 0x00040000 */
#define RTC_CR_BKP                     RTC_CR_BKP_Msk                          
#define RTC_CR_SUB1H_Pos              (17U)                                    
#define RTC_CR_SUB1H_Msk              (0x1U << RTC_CR_SUB1H_Pos)               /*!< 0x00020000 */
#define RTC_CR_SUB1H                  RTC_CR_SUB1H_Msk                         
#define RTC_CR_ADD1H_Pos              (16U)                                    
#define RTC_CR_ADD1H_Msk              (0x1U << RTC_CR_ADD1H_Pos)               /*!< 0x00010000 */
#define RTC_CR_ADD1H                  RTC_CR_ADD1H_Msk                         
#define RTC_CR_TSIE_Pos               (15U)                                    
#define RTC_CR_TSIE_Msk               (0x1U << RTC_CR_TSIE_Pos)                /*!< 0x00008000 */
#define RTC_CR_TSIE                   RTC_CR_TSIE_Msk                          
#define RTC_CR_WUTIE_Pos              (14U)                                    
#define RTC_CR_WUTIE_Msk              (0x1U << RTC_CR_WUTIE_Pos)               /*!< 0x00004000 */
#define RTC_CR_WUTIE                  RTC_CR_WUTIE_Msk                         
#define RTC_CR_ALRBIE_Pos             (13U)                                    
#define RTC_CR_ALRBIE_Msk             (0x1U << RTC_CR_ALRBIE_Pos)              /*!< 0x00002000 */
#define RTC_CR_ALRBIE                 RTC_CR_ALRBIE_Msk                        
#define RTC_CR_ALRAIE_Pos             (12U)                                    
#define RTC_CR_ALRAIE_Msk             (0x1U << RTC_CR_ALRAIE_Pos)              /*!< 0x00001000 */
#define RTC_CR_ALRAIE                 RTC_CR_ALRAIE_Msk                        
#define RTC_CR_TSE_Pos                (11U)                                    
#define RTC_CR_TSE_Msk                (0x1U << RTC_CR_TSE_Pos)                 /*!< 0x00000800 */
#define RTC_CR_TSE                    RTC_CR_TSE_Msk                           
#define RTC_CR_WUTE_Pos               (10U)                                    
#define RTC_CR_WUTE_Msk               (0x1U << RTC_CR_WUTE_Pos)                /*!< 0x00000400 */
#define RTC_CR_WUTE                   RTC_CR_WUTE_Msk                          
#define RTC_CR_ALRBE_Pos              (9U)                                     
#define RTC_CR_ALRBE_Msk              (0x1U << RTC_CR_ALRBE_Pos)               /*!< 0x00000200 */
#define RTC_CR_ALRBE                  RTC_CR_ALRBE_Msk                         
#define RTC_CR_ALRAE_Pos              (8U)                                     
#define RTC_CR_ALRAE_Msk              (0x1U << RTC_CR_ALRAE_Pos)               /*!< 0x00000100 */
#define RTC_CR_ALRAE                  RTC_CR_ALRAE_Msk                         
#define RTC_CR_DCE_Pos                (7U)                                     
#define RTC_CR_DCE_Msk                (0x1U << RTC_CR_DCE_Pos)                 /*!< 0x00000080 */
#define RTC_CR_DCE                    RTC_CR_DCE_Msk                           
#define RTC_CR_FMT_Pos                (6U)                                     
#define RTC_CR_FMT_Msk                (0x1U << RTC_CR_FMT_Pos)                 /*!< 0x00000040 */
#define RTC_CR_FMT                    RTC_CR_FMT_Msk                           
#define RTC_CR_BYPSHAD_Pos            (5U)                                     
#define RTC_CR_BYPSHAD_Msk            (0x1U << RTC_CR_BYPSHAD_Pos)             /*!< 0x00000020 */
#define RTC_CR_BYPSHAD                RTC_CR_BYPSHAD_Msk                       
#define RTC_CR_REFCKON_Pos            (4U)                                     
#define RTC_CR_REFCKON_Msk            (0x1U << RTC_CR_REFCKON_Pos)             /*!< 0x00000010 */
#define RTC_CR_REFCKON                RTC_CR_REFCKON_Msk                       
#define RTC_CR_TSEDGE_Pos             (3U)                                     
#define RTC_CR_TSEDGE_Msk             (0x1U << RTC_CR_TSEDGE_Pos)              /*!< 0x00000008 */
#define RTC_CR_TSEDGE                 RTC_CR_TSEDGE_Msk                        
#define RTC_CR_WUCKSEL_Pos            (0U)                                     
#define RTC_CR_WUCKSEL_Msk            (0x7U << RTC_CR_WUCKSEL_Pos)             /*!< 0x00000007 */
#define RTC_CR_WUCKSEL                RTC_CR_WUCKSEL_Msk                       
#define RTC_CR_WUCKSEL_0              (0x1U << RTC_CR_WUCKSEL_Pos)             /*!< 0x00000001 */
#define RTC_CR_WUCKSEL_1              (0x2U << RTC_CR_WUCKSEL_Pos)             /*!< 0x00000002 */
#define RTC_CR_WUCKSEL_2              (0x4U << RTC_CR_WUCKSEL_Pos)             /*!< 0x00000004 */

/* Legacy defines */
#define RTC_CR_BCK                     RTC_CR_BKP

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_RECALPF_Pos           (16U)                                    
#define RTC_ISR_RECALPF_Msk           (0x1U << RTC_ISR_RECALPF_Pos)            /*!< 0x00010000 */
#define RTC_ISR_RECALPF               RTC_ISR_RECALPF_Msk                      
#define RTC_ISR_TAMP1F_Pos            (13U)                                    
#define RTC_ISR_TAMP1F_Msk            (0x1U << RTC_ISR_TAMP1F_Pos)             /*!< 0x00002000 */
#define RTC_ISR_TAMP1F                RTC_ISR_TAMP1F_Msk                       
#define RTC_ISR_TAMP2F_Pos            (14U)                                    
#define RTC_ISR_TAMP2F_Msk            (0x1U << RTC_ISR_TAMP2F_Pos)             /*!< 0x00004000 */
#define RTC_ISR_TAMP2F                RTC_ISR_TAMP2F_Msk                       
#define RTC_ISR_TSOVF_Pos             (12U)                                    
#define RTC_ISR_TSOVF_Msk             (0x1U << RTC_ISR_TSOVF_Pos)              /*!< 0x00001000 */
#define RTC_ISR_TSOVF                 RTC_ISR_TSOVF_Msk                        
#define RTC_ISR_TSF_Pos               (11U)                                    
#define RTC_ISR_TSF_Msk               (0x1U << RTC_ISR_TSF_Pos)                /*!< 0x00000800 */
#define RTC_ISR_TSF                   RTC_ISR_TSF_Msk                          
#define RTC_ISR_WUTF_Pos              (10U)                                    
#define RTC_ISR_WUTF_Msk              (0x1U << RTC_ISR_WUTF_Pos)               /*!< 0x00000400 */
#define RTC_ISR_WUTF                  RTC_ISR_WUTF_Msk                         
#define RTC_ISR_ALRBF_Pos             (9U)                                     
#define RTC_ISR_ALRBF_Msk             (0x1U << RTC_ISR_ALRBF_Pos)              /*!< 0x00000200 */
#define RTC_ISR_ALRBF                 RTC_ISR_ALRBF_Msk                        
#define RTC_ISR_ALRAF_Pos             (8U)                                     
#define RTC_ISR_ALRAF_Msk             (0x1U << RTC_ISR_ALRAF_Pos)              /*!< 0x00000100 */
#define RTC_ISR_ALRAF                 RTC_ISR_ALRAF_Msk                        
#define RTC_ISR_INIT_Pos              (7U)                                     
#define RTC_ISR_INIT_Msk              (0x1U << RTC_ISR_INIT_Pos)               /*!< 0x00000080 */
#define RTC_ISR_INIT                  RTC_ISR_INIT_Msk                         
#define RTC_ISR_INITF_Pos             (6U)                                     
#define RTC_ISR_INITF_Msk             (0x1U << RTC_ISR_INITF_Pos)              /*!< 0x00000040 */
#define RTC_ISR_INITF                 RTC_ISR_INITF_Msk                        
#define RTC_ISR_RSF_Pos               (5U)                                     
#define RTC_ISR_RSF_Msk               (0x1U << RTC_ISR_RSF_Pos)                /*!< 0x00000020 */
#define RTC_ISR_RSF                   RTC_ISR_RSF_Msk                          
#define RTC_ISR_INITS_Pos             (4U)                                     
#define RTC_ISR_INITS_Msk             (0x1U << RTC_ISR_INITS_Pos)              /*!< 0x00000010 */
#define RTC_ISR_INITS                 RTC_ISR_INITS_Msk                        
#define RTC_ISR_SHPF_Pos              (3U)                                     
#define RTC_ISR_SHPF_Msk              (0x1U << RTC_ISR_SHPF_Pos)               /*!< 0x00000008 */
#define RTC_ISR_SHPF                  RTC_ISR_SHPF_Msk                         
#define RTC_ISR_WUTWF_Pos             (2U)                                     
#define RTC_ISR_WUTWF_Msk             (0x1U << RTC_ISR_WUTWF_Pos)              /*!< 0x00000004 */
#define RTC_ISR_WUTWF                 RTC_ISR_WUTWF_Msk                        
#define RTC_ISR_ALRBWF_Pos            (1U)                                     
#define RTC_ISR_ALRBWF_Msk            (0x1U << RTC_ISR_ALRBWF_Pos)             /*!< 0x00000002 */
#define RTC_ISR_ALRBWF                RTC_ISR_ALRBWF_Msk                       
#define RTC_ISR_ALRAWF_Pos            (0U)                                     
#define RTC_ISR_ALRAWF_Msk            (0x1U << RTC_ISR_ALRAWF_Pos)             /*!< 0x00000001 */
#define RTC_ISR_ALRAWF                RTC_ISR_ALRAWF_Msk                       

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A_Pos         (16U)                                    
#define RTC_PRER_PREDIV_A_Msk         (0x7FU << RTC_PRER_PREDIV_A_Pos)         /*!< 0x007F0000 */
#define RTC_PRER_PREDIV_A             RTC_PRER_PREDIV_A_Msk                    
#define RTC_PRER_PREDIV_S_Pos         (0U)                                     
#define RTC_PRER_PREDIV_S_Msk         (0x7FFFU << RTC_PRER_PREDIV_S_Pos)       /*!< 0x00007FFF */
#define RTC_PRER_PREDIV_S             RTC_PRER_PREDIV_S_Msk                    

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT_Pos              (0U)                                     
#define RTC_WUTR_WUT_Msk              (0xFFFFU << RTC_WUTR_WUT_Pos)            /*!< 0x0000FFFF */
#define RTC_WUTR_WUT                  RTC_WUTR_WUT_Msk                         

/********************  Bits definition for RTC_CALIBR register  ***************/
#define RTC_CALIBR_DCS_Pos            (7U)                                     
#define RTC_CALIBR_DCS_Msk            (0x1U << RTC_CALIBR_DCS_Pos)             /*!< 0x00000080 */
#define RTC_CALIBR_DCS                RTC_CALIBR_DCS_Msk                       
#define RTC_CALIBR_DC_Pos             (0U)                                     
#define RTC_CALIBR_DC_Msk             (0x1FU << RTC_CALIBR_DC_Pos)             /*!< 0x0000001F */
#define RTC_CALIBR_DC                 RTC_CALIBR_DC_Msk                        

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4_Pos           (31U)                                    
#define RTC_ALRMAR_MSK4_Msk           (0x1U << RTC_ALRMAR_MSK4_Pos)            /*!< 0x80000000 */
#define RTC_ALRMAR_MSK4               RTC_ALRMAR_MSK4_Msk                      
#define RTC_ALRMAR_WDSEL_Pos          (30U)                                    
#define RTC_ALRMAR_WDSEL_Msk          (0x1U << RTC_ALRMAR_WDSEL_Pos)           /*!< 0x40000000 */
#define RTC_ALRMAR_WDSEL              RTC_ALRMAR_WDSEL_Msk                     
#define RTC_ALRMAR_DT_Pos             (28U)                                    
#define RTC_ALRMAR_DT_Msk             (0x3U << RTC_ALRMAR_DT_Pos)              /*!< 0x30000000 */
#define RTC_ALRMAR_DT                 RTC_ALRMAR_DT_Msk                        
#define RTC_ALRMAR_DT_0               (0x1U << RTC_ALRMAR_DT_Pos)              /*!< 0x10000000 */
#define RTC_ALRMAR_DT_1               (0x2U << RTC_ALRMAR_DT_Pos)              /*!< 0x20000000 */
#define RTC_ALRMAR_DU_Pos             (24U)                                    
#define RTC_ALRMAR_DU_Msk             (0xFU << RTC_ALRMAR_DU_Pos)              /*!< 0x0F000000 */
#define RTC_ALRMAR_DU                 RTC_ALRMAR_DU_Msk                        
#define RTC_ALRMAR_DU_0               (0x1U << RTC_ALRMAR_DU_Pos)              /*!< 0x01000000 */
#define RTC_ALRMAR_DU_1               (0x2U << RTC_ALRMAR_DU_Pos)              /*!< 0x02000000 */
#define RTC_ALRMAR_DU_2               (0x4U << RTC_ALRMAR_DU_Pos)              /*!< 0x04000000 */
#define RTC_ALRMAR_DU_3               (0x8U << RTC_ALRMAR_DU_Pos)              /*!< 0x08000000 */
#define RTC_ALRMAR_MSK3_Pos           (23U)                                    
#define RTC_ALRMAR_MSK3_Msk           (0x1U << RTC_ALRMAR_MSK3_Pos)            /*!< 0x00800000 */
#define RTC_ALRMAR_MSK3               RTC_ALRMAR_MSK3_Msk                      
#define RTC_ALRMAR_PM_Pos             (22U)                                    
#define RTC_ALRMAR_PM_Msk             (0x1U << RTC_ALRMAR_PM_Pos)              /*!< 0x00400000 */
#define RTC_ALRMAR_PM                 RTC_ALRMAR_PM_Msk                        
#define RTC_ALRMAR_HT_Pos             (20U)                                    
#define RTC_ALRMAR_HT_Msk             (0x3U << RTC_ALRMAR_HT_Pos)              /*!< 0x00300000 */
#define RTC_ALRMAR_HT                 RTC_ALRMAR_HT_Msk                        
#define RTC_ALRMAR_HT_0               (0x1U << RTC_ALRMAR_HT_Pos)              /*!< 0x00100000 */
#define RTC_ALRMAR_HT_1               (0x2U << RTC_ALRMAR_HT_Pos)              /*!< 0x00200000 */
#define RTC_ALRMAR_HU_Pos             (16U)                                    
#define RTC_ALRMAR_HU_Msk             (0xFU << RTC_ALRMAR_HU_Pos)              /*!< 0x000F0000 */
#define RTC_ALRMAR_HU                 RTC_ALRMAR_HU_Msk                        
#define RTC_ALRMAR_HU_0               (0x1U << RTC_ALRMAR_HU_Pos)              /*!< 0x00010000 */
#define RTC_ALRMAR_HU_1               (0x2U << RTC_ALRMAR_HU_Pos)              /*!< 0x00020000 */
#define RTC_ALRMAR_HU_2               (0x4U << RTC_ALRMAR_HU_Pos)              /*!< 0x00040000 */
#define RTC_ALRMAR_HU_3               (0x8U << RTC_ALRMAR_HU_Pos)              /*!< 0x00080000 */
#define RTC_ALRMAR_MSK2_Pos           (15U)                                    
#define RTC_ALRMAR_MSK2_Msk           (0x1U << RTC_ALRMAR_MSK2_Pos)            /*!< 0x00008000 */
#define RTC_ALRMAR_MSK2               RTC_ALRMAR_MSK2_Msk                      
#define RTC_ALRMAR_MNT_Pos            (12U)                                    
#define RTC_ALRMAR_MNT_Msk            (0x7U << RTC_ALRMAR_MNT_Pos)             /*!< 0x00007000 */
#define RTC_ALRMAR_MNT                RTC_ALRMAR_MNT_Msk                       
#define RTC_ALRMAR_MNT_0              (0x1U << RTC_ALRMAR_MNT_Pos)             /*!< 0x00001000 */
#define RTC_ALRMAR_MNT_1              (0x2U << RTC_ALRMAR_MNT_Pos)             /*!< 0x00002000 */
#define RTC_ALRMAR_MNT_2              (0x4U << RTC_ALRMAR_MNT_Pos)             /*!< 0x00004000 */
#define RTC_ALRMAR_MNU_Pos            (8U)                                     
#define RTC_ALRMAR_MNU_Msk            (0xFU << RTC_ALRMAR_MNU_Pos)             /*!< 0x00000F00 */
#define RTC_ALRMAR_MNU                RTC_ALRMAR_MNU_Msk                       
#define RTC_ALRMAR_MNU_0              (0x1U << RTC_ALRMAR_MNU_Pos)             /*!< 0x00000100 */
#define RTC_ALRMAR_MNU_1              (0x2U << RTC_ALRMAR_MNU_Pos)             /*!< 0x00000200 */
#define RTC_ALRMAR_MNU_2              (0x4U << RTC_ALRMAR_MNU_Pos)             /*!< 0x00000400 */
#define RTC_ALRMAR_MNU_3              (0x8U << RTC_ALRMAR_MNU_Pos)             /*!< 0x00000800 */
#define RTC_ALRMAR_MSK1_Pos           (7U)                                     
#define RTC_ALRMAR_MSK1_Msk           (0x1U << RTC_ALRMAR_MSK1_Pos)            /*!< 0x00000080 */
#define RTC_ALRMAR_MSK1               RTC_ALRMAR_MSK1_Msk                      
#define RTC_ALRMAR_ST_Pos             (4U)                                     
#define RTC_ALRMAR_ST_Msk             (0x7U << RTC_ALRMAR_ST_Pos)              /*!< 0x00000070 */
#define RTC_ALRMAR_ST                 RTC_ALRMAR_ST_Msk                        
#define RTC_ALRMAR_ST_0               (0x1U << RTC_ALRMAR_ST_Pos)              /*!< 0x00000010 */
#define RTC_ALRMAR_ST_1               (0x2U << RTC_ALRMAR_ST_Pos)              /*!< 0x00000020 */
#define RTC_ALRMAR_ST_2               (0x4U << RTC_ALRMAR_ST_Pos)              /*!< 0x00000040 */
#define RTC_ALRMAR_SU_Pos             (0U)                                     
#define RTC_ALRMAR_SU_Msk             (0xFU << RTC_ALRMAR_SU_Pos)              /*!< 0x0000000F */
#define RTC_ALRMAR_SU                 RTC_ALRMAR_SU_Msk                        
#define RTC_ALRMAR_SU_0               (0x1U << RTC_ALRMAR_SU_Pos)              /*!< 0x00000001 */
#define RTC_ALRMAR_SU_1               (0x2U << RTC_ALRMAR_SU_Pos)              /*!< 0x00000002 */
#define RTC_ALRMAR_SU_2               (0x4U << RTC_ALRMAR_SU_Pos)              /*!< 0x00000004 */
#define RTC_ALRMAR_SU_3               (0x8U << RTC_ALRMAR_SU_Pos)              /*!< 0x00000008 */

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4_Pos           (31U)                                    
#define RTC_ALRMBR_MSK4_Msk           (0x1U << RTC_ALRMBR_MSK4_Pos)            /*!< 0x80000000 */
#define RTC_ALRMBR_MSK4               RTC_ALRMBR_MSK4_Msk                      
#define RTC_ALRMBR_WDSEL_Pos          (30U)                                    
#define RTC_ALRMBR_WDSEL_Msk          (0x1U << RTC_ALRMBR_WDSEL_Pos)           /*!< 0x40000000 */
#define RTC_ALRMBR_WDSEL              RTC_ALRMBR_WDSEL_Msk                     
#define RTC_ALRMBR_DT_Pos             (28U)                                    
#define RTC_ALRMBR_DT_Msk             (0x3U << RTC_ALRMBR_DT_Pos)              /*!< 0x30000000 */
#define RTC_ALRMBR_DT                 RTC_ALRMBR_DT_Msk                        
#define RTC_ALRMBR_DT_0               (0x1U << RTC_ALRMBR_DT_Pos)              /*!< 0x10000000 */
#define RTC_ALRMBR_DT_1               (0x2U << RTC_ALRMBR_DT_Pos)              /*!< 0x20000000 */
#define RTC_ALRMBR_DU_Pos             (24U)                                    
#define RTC_ALRMBR_DU_Msk             (0xFU << RTC_ALRMBR_DU_Pos)              /*!< 0x0F000000 */
#define RTC_ALRMBR_DU                 RTC_ALRMBR_DU_Msk                        
#define RTC_ALRMBR_DU_0               (0x1U << RTC_ALRMBR_DU_Pos)              /*!< 0x01000000 */
#define RTC_ALRMBR_DU_1               (0x2U << RTC_ALRMBR_DU_Pos)              /*!< 0x02000000 */
#define RTC_ALRMBR_DU_2               (0x4U << RTC_ALRMBR_DU_Pos)              /*!< 0x04000000 */
#define RTC_ALRMBR_DU_3               (0x8U << RTC_ALRMBR_DU_Pos)              /*!< 0x08000000 */
#define RTC_ALRMBR_MSK3_Pos           (23U)                                    
#define RTC_ALRMBR_MSK3_Msk           (0x1U << RTC_ALRMBR_MSK3_Pos)            /*!< 0x00800000 */
#define RTC_ALRMBR_MSK3               RTC_ALRMBR_MSK3_Msk                      
#define RTC_ALRMBR_PM_Pos             (22U)                                    
#define RTC_ALRMBR_PM_Msk             (0x1U << RTC_ALRMBR_PM_Pos)              /*!< 0x00400000 */
#define RTC_ALRMBR_PM                 RTC_ALRMBR_PM_Msk                        
#define RTC_ALRMBR_HT_Pos             (20U)                                    
#define RTC_ALRMBR_HT_Msk             (0x3U << RTC_ALRMBR_HT_Pos)              /*!< 0x00300000 */
#define RTC_ALRMBR_HT                 RTC_ALRMBR_HT_Msk                        
#define RTC_ALRMBR_HT_0               (0x1U << RTC_ALRMBR_HT_Pos)              /*!< 0x00100000 */
#define RTC_ALRMBR_HT_1               (0x2U << RTC_ALRMBR_HT_Pos)              /*!< 0x00200000 */
#define RTC_ALRMBR_HU_Pos             (16U)                                    
#define RTC_ALRMBR_HU_Msk             (0xFU << RTC_ALRMBR_HU_Pos)              /*!< 0x000F0000 */
#define RTC_ALRMBR_HU                 RTC_ALRMBR_HU_Msk                        
#define RTC_ALRMBR_HU_0               (0x1U << RTC_ALRMBR_HU_Pos)              /*!< 0x00010000 */
#define RTC_ALRMBR_HU_1               (0x2U << RTC_ALRMBR_HU_Pos)              /*!< 0x00020000 */
#define RTC_ALRMBR_HU_2               (0x4U << RTC_ALRMBR_HU_Pos)              /*!< 0x00040000 */
#define RTC_ALRMBR_HU_3               (0x8U << RTC_ALRMBR_HU_Pos)              /*!< 0x00080000 */
#define RTC_ALRMBR_MSK2_Pos           (15U)                                    
#define RTC_ALRMBR_MSK2_Msk           (0x1U << RTC_ALRMBR_MSK2_Pos)            /*!< 0x00008000 */
#define RTC_ALRMBR_MSK2               RTC_ALRMBR_MSK2_Msk                      
#define RTC_ALRMBR_MNT_Pos            (12U)                                    
#define RTC_ALRMBR_MNT_Msk            (0x7U << RTC_ALRMBR_MNT_Pos)             /*!< 0x00007000 */
#define RTC_ALRMBR_MNT                RTC_ALRMBR_MNT_Msk                       
#define RTC_ALRMBR_MNT_0              (0x1U << RTC_ALRMBR_MNT_Pos)             /*!< 0x00001000 */
#define RTC_ALRMBR_MNT_1              (0x2U << RTC_ALRMBR_MNT_Pos)             /*!< 0x00002000 */
#define RTC_ALRMBR_MNT_2              (0x4U << RTC_ALRMBR_MNT_Pos)             /*!< 0x00004000 */
#define RTC_ALRMBR_MNU_Pos            (8U)                                     
#define RTC_ALRMBR_MNU_Msk            (0xFU << RTC_ALRMBR_MNU_Pos)             /*!< 0x00000F00 */
#define RTC_ALRMBR_MNU                RTC_ALRMBR_MNU_Msk                       
#define RTC_ALRMBR_MNU_0              (0x1U << RTC_ALRMBR_MNU_Pos)             /*!< 0x00000100 */
#define RTC_ALRMBR_MNU_1              (0x2U << RTC_ALRMBR_MNU_Pos)             /*!< 0x00000200 */
#define RTC_ALRMBR_MNU_2              (0x4U << RTC_ALRMBR_MNU_Pos)             /*!< 0x00000400 */
#define RTC_ALRMBR_MNU_3              (0x8U << RTC_ALRMBR_MNU_Pos)             /*!< 0x00000800 */
#define RTC_ALRMBR_MSK1_Pos           (7U)                                     
#define RTC_ALRMBR_MSK1_Msk           (0x1U << RTC_ALRMBR_MSK1_Pos)            /*!< 0x00000080 */
#define RTC_ALRMBR_MSK1               RTC_ALRMBR_MSK1_Msk                      
#define RTC_ALRMBR_ST_Pos             (4U)                                     
#define RTC_ALRMBR_ST_Msk             (0x7U << RTC_ALRMBR_ST_Pos)              /*!< 0x00000070 */
#define RTC_ALRMBR_ST                 RTC_ALRMBR_ST_Msk                        
#define RTC_ALRMBR_ST_0               (0x1U << RTC_ALRMBR_ST_Pos)              /*!< 0x00000010 */
#define RTC_ALRMBR_ST_1               (0x2U << RTC_ALRMBR_ST_Pos)              /*!< 0x00000020 */
#define RTC_ALRMBR_ST_2               (0x4U << RTC_ALRMBR_ST_Pos)              /*!< 0x00000040 */
#define RTC_ALRMBR_SU_Pos             (0U)                                     
#define RTC_ALRMBR_SU_Msk             (0xFU << RTC_ALRMBR_SU_Pos)              /*!< 0x0000000F */
#define RTC_ALRMBR_SU                 RTC_ALRMBR_SU_Msk                        
#define RTC_ALRMBR_SU_0               (0x1U << RTC_ALRMBR_SU_Pos)              /*!< 0x00000001 */
#define RTC_ALRMBR_SU_1               (0x2U << RTC_ALRMBR_SU_Pos)              /*!< 0x00000002 */
#define RTC_ALRMBR_SU_2               (0x4U << RTC_ALRMBR_SU_Pos)              /*!< 0x00000004 */
#define RTC_ALRMBR_SU_3               (0x8U << RTC_ALRMBR_SU_Pos)              /*!< 0x00000008 */

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY_Pos               (0U)                                     
#define RTC_WPR_KEY_Msk               (0xFFU << RTC_WPR_KEY_Pos)               /*!< 0x000000FF */
#define RTC_WPR_KEY                   RTC_WPR_KEY_Msk                          

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS_Pos                (0U)                                     
#define RTC_SSR_SS_Msk                (0xFFFFU << RTC_SSR_SS_Pos)              /*!< 0x0000FFFF */
#define RTC_SSR_SS                    RTC_SSR_SS_Msk                           

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS_Pos          (0U)                                     
#define RTC_SHIFTR_SUBFS_Msk          (0x7FFFU << RTC_SHIFTR_SUBFS_Pos)        /*!< 0x00007FFF */
#define RTC_SHIFTR_SUBFS              RTC_SHIFTR_SUBFS_Msk                     
#define RTC_SHIFTR_ADD1S_Pos          (31U)                                    
#define RTC_SHIFTR_ADD1S_Msk          (0x1U << RTC_SHIFTR_ADD1S_Pos)           /*!< 0x80000000 */
#define RTC_SHIFTR_ADD1S              RTC_SHIFTR_ADD1S_Msk                     

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM_Pos               (22U)                                    
#define RTC_TSTR_PM_Msk               (0x1U << RTC_TSTR_PM_Pos)                /*!< 0x00400000 */
#define RTC_TSTR_PM                   RTC_TSTR_PM_Msk                          
#define RTC_TSTR_HT_Pos               (20U)                                    
#define RTC_TSTR_HT_Msk               (0x3U << RTC_TSTR_HT_Pos)                /*!< 0x00300000 */
#define RTC_TSTR_HT                   RTC_TSTR_HT_Msk                          
#define RTC_TSTR_HT_0                 (0x1U << RTC_TSTR_HT_Pos)                /*!< 0x00100000 */
#define RTC_TSTR_HT_1                 (0x2U << RTC_TSTR_HT_Pos)                /*!< 0x00200000 */
#define RTC_TSTR_HU_Pos               (16U)                                    
#define RTC_TSTR_HU_Msk               (0xFU << RTC_TSTR_HU_Pos)                /*!< 0x000F0000 */
#define RTC_TSTR_HU                   RTC_TSTR_HU_Msk                          
#define RTC_TSTR_HU_0                 (0x1U << RTC_TSTR_HU_Pos)                /*!< 0x00010000 */
#define RTC_TSTR_HU_1                 (0x2U << RTC_TSTR_HU_Pos)                /*!< 0x00020000 */
#define RTC_TSTR_HU_2                 (0x4U << RTC_TSTR_HU_Pos)                /*!< 0x00040000 */
#define RTC_TSTR_HU_3                 (0x8U << RTC_TSTR_HU_Pos)                /*!< 0x00080000 */
#define RTC_TSTR_MNT_Pos              (12U)                                    
#define RTC_TSTR_MNT_Msk              (0x7U << RTC_TSTR_MNT_Pos)               /*!< 0x00007000 */
#define RTC_TSTR_MNT                  RTC_TSTR_MNT_Msk                         
#define RTC_TSTR_MNT_0                (0x1U << RTC_TSTR_MNT_Pos)               /*!< 0x00001000 */
#define RTC_TSTR_MNT_1                (0x2U << RTC_TSTR_MNT_Pos)               /*!< 0x00002000 */
#define RTC_TSTR_MNT_2                (0x4U << RTC_TSTR_MNT_Pos)               /*!< 0x00004000 */
#define RTC_TSTR_MNU_Pos              (8U)                                     
#define RTC_TSTR_MNU_Msk              (0xFU << RTC_TSTR_MNU_Pos)               /*!< 0x00000F00 */
#define RTC_TSTR_MNU                  RTC_TSTR_MNU_Msk                         
#define RTC_TSTR_MNU_0                (0x1U << RTC_TSTR_MNU_Pos)               /*!< 0x00000100 */
#define RTC_TSTR_MNU_1                (0x2U << RTC_TSTR_MNU_Pos)               /*!< 0x00000200 */
#define RTC_TSTR_MNU_2                (0x4U << RTC_TSTR_MNU_Pos)               /*!< 0x00000400 */
#define RTC_TSTR_MNU_3                (0x8U << RTC_TSTR_MNU_Pos)               /*!< 0x00000800 */
#define RTC_TSTR_ST_Pos               (4U)                                     
#define RTC_TSTR_ST_Msk               (0x7U << RTC_TSTR_ST_Pos)                /*!< 0x00000070 */
#define RTC_TSTR_ST                   RTC_TSTR_ST_Msk                          
#define RTC_TSTR_ST_0                 (0x1U << RTC_TSTR_ST_Pos)                /*!< 0x00000010 */
#define RTC_TSTR_ST_1                 (0x2U << RTC_TSTR_ST_Pos)                /*!< 0x00000020 */
#define RTC_TSTR_ST_2                 (0x4U << RTC_TSTR_ST_Pos)                /*!< 0x00000040 */
#define RTC_TSTR_SU_Pos               (0U)                                     
#define RTC_TSTR_SU_Msk               (0xFU << RTC_TSTR_SU_Pos)                /*!< 0x0000000F */
#define RTC_TSTR_SU                   RTC_TSTR_SU_Msk                          
#define RTC_TSTR_SU_0                 (0x1U << RTC_TSTR_SU_Pos)                /*!< 0x00000001 */
#define RTC_TSTR_SU_1                 (0x2U << RTC_TSTR_SU_Pos)                /*!< 0x00000002 */
#define RTC_TSTR_SU_2                 (0x4U << RTC_TSTR_SU_Pos)                /*!< 0x00000004 */
#define RTC_TSTR_SU_3                 (0x8U << RTC_TSTR_SU_Pos)                /*!< 0x00000008 */

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU_Pos              (13U)                                    
#define RTC_TSDR_WDU_Msk              (0x7U << RTC_TSDR_WDU_Pos)               /*!< 0x0000E000 */
#define RTC_TSDR_WDU                  RTC_TSDR_WDU_Msk                         
#define RTC_TSDR_WDU_0                (0x1U << RTC_TSDR_WDU_Pos)               /*!< 0x00002000 */
#define RTC_TSDR_WDU_1                (0x2U << RTC_TSDR_WDU_Pos)               /*!< 0x00004000 */
#define RTC_TSDR_WDU_2                (0x4U << RTC_TSDR_WDU_Pos)               /*!< 0x00008000 */
#define RTC_TSDR_MT_Pos               (12U)                                    
#define RTC_TSDR_MT_Msk               (0x1U << RTC_TSDR_MT_Pos)                /*!< 0x00001000 */
#define RTC_TSDR_MT                   RTC_TSDR_MT_Msk                          
#define RTC_TSDR_MU_Pos               (8U)                                     
#define RTC_TSDR_MU_Msk               (0xFU << RTC_TSDR_MU_Pos)                /*!< 0x00000F00 */
#define RTC_TSDR_MU                   RTC_TSDR_MU_Msk                          
#define RTC_TSDR_MU_0                 (0x1U << RTC_TSDR_MU_Pos)                /*!< 0x00000100 */
#define RTC_TSDR_MU_1                 (0x2U << RTC_TSDR_MU_Pos)                /*!< 0x00000200 */
#define RTC_TSDR_MU_2                 (0x4U << RTC_TSDR_MU_Pos)                /*!< 0x00000400 */
#define RTC_TSDR_MU_3                 (0x8U << RTC_TSDR_MU_Pos)                /*!< 0x00000800 */
#define RTC_TSDR_DT_Pos               (4U)                                     
#define RTC_TSDR_DT_Msk               (0x3U << RTC_TSDR_DT_Pos)                /*!< 0x00000030 */
#define RTC_TSDR_DT                   RTC_TSDR_DT_Msk                          
#define RTC_TSDR_DT_0                 (0x1U << RTC_TSDR_DT_Pos)                /*!< 0x00000010 */
#define RTC_TSDR_DT_1                 (0x2U << RTC_TSDR_DT_Pos)                /*!< 0x00000020 */
#define RTC_TSDR_DU_Pos               (0U)                                     
#define RTC_TSDR_DU_Msk               (0xFU << RTC_TSDR_DU_Pos)                /*!< 0x0000000F */
#define RTC_TSDR_DU                   RTC_TSDR_DU_Msk                          
#define RTC_TSDR_DU_0                 (0x1U << RTC_TSDR_DU_Pos)                /*!< 0x00000001 */
#define RTC_TSDR_DU_1                 (0x2U << RTC_TSDR_DU_Pos)                /*!< 0x00000002 */
#define RTC_TSDR_DU_2                 (0x4U << RTC_TSDR_DU_Pos)                /*!< 0x00000004 */
#define RTC_TSDR_DU_3                 (0x8U << RTC_TSDR_DU_Pos)                /*!< 0x00000008 */

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS_Pos              (0U)                                     
#define RTC_TSSSR_SS_Msk              (0xFFFFU << RTC_TSSSR_SS_Pos)            /*!< 0x0000FFFF */
#define RTC_TSSSR_SS                  RTC_TSSSR_SS_Msk                         

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP_Pos             (15U)                                    
#define RTC_CALR_CALP_Msk             (0x1U << RTC_CALR_CALP_Pos)              /*!< 0x00008000 */
#define RTC_CALR_CALP                 RTC_CALR_CALP_Msk                        
#define RTC_CALR_CALW8_Pos            (14U)                                    
#define RTC_CALR_CALW8_Msk            (0x1U << RTC_CALR_CALW8_Pos)             /*!< 0x00004000 */
#define RTC_CALR_CALW8                RTC_CALR_CALW8_Msk                       
#define RTC_CALR_CALW16_Pos           (13U)                                    
#define RTC_CALR_CALW16_Msk           (0x1U << RTC_CALR_CALW16_Pos)            /*!< 0x00002000 */
#define RTC_CALR_CALW16               RTC_CALR_CALW16_Msk                      
#define RTC_CALR_CALM_Pos             (0U)                                     
#define RTC_CALR_CALM_Msk             (0x1FFU << RTC_CALR_CALM_Pos)            /*!< 0x000001FF */
#define RTC_CALR_CALM                 RTC_CALR_CALM_Msk                        
#define RTC_CALR_CALM_0               (0x001U << RTC_CALR_CALM_Pos)            /*!< 0x00000001 */
#define RTC_CALR_CALM_1               (0x002U << RTC_CALR_CALM_Pos)            /*!< 0x00000002 */
#define RTC_CALR_CALM_2               (0x004U << RTC_CALR_CALM_Pos)            /*!< 0x00000004 */
#define RTC_CALR_CALM_3               (0x008U << RTC_CALR_CALM_Pos)            /*!< 0x00000008 */
#define RTC_CALR_CALM_4               (0x010U << RTC_CALR_CALM_Pos)            /*!< 0x00000010 */
#define RTC_CALR_CALM_5               (0x020U << RTC_CALR_CALM_Pos)            /*!< 0x00000020 */
#define RTC_CALR_CALM_6               (0x040U << RTC_CALR_CALM_Pos)            /*!< 0x00000040 */
#define RTC_CALR_CALM_7               (0x080U << RTC_CALR_CALM_Pos)            /*!< 0x00000080 */
#define RTC_CALR_CALM_8               (0x100U << RTC_CALR_CALM_Pos)            /*!< 0x00000100 */

/********************  Bits definition for RTC_TAFCR register  ****************/
#define RTC_TAFCR_ALARMOUTTYPE_Pos    (18U)                                    
#define RTC_TAFCR_ALARMOUTTYPE_Msk    (0x1U << RTC_TAFCR_ALARMOUTTYPE_Pos)     /*!< 0x00040000 */
#define RTC_TAFCR_ALARMOUTTYPE        RTC_TAFCR_ALARMOUTTYPE_Msk               
#define RTC_TAFCR_TSINSEL_Pos         (17U)                                    
#define RTC_TAFCR_TSINSEL_Msk         (0x1U << RTC_TAFCR_TSINSEL_Pos)          /*!< 0x00020000 */
#define RTC_TAFCR_TSINSEL             RTC_TAFCR_TSINSEL_Msk                    
#define RTC_TAFCR_TAMP1INSEL_Pos      (16U)                                    
#define RTC_TAFCR_TAMP1INSEL_Msk      (0x1U << RTC_TAFCR_TAMP1INSEL_Pos)        /*!< 0x00010000 */
#define RTC_TAFCR_TAMP1INSEL          RTC_TAFCR_TAMP1INSEL_Msk                  
#define RTC_TAFCR_TAMPPUDIS_Pos       (15U)                                    
#define RTC_TAFCR_TAMPPUDIS_Msk       (0x1U << RTC_TAFCR_TAMPPUDIS_Pos)        /*!< 0x00008000 */
#define RTC_TAFCR_TAMPPUDIS           RTC_TAFCR_TAMPPUDIS_Msk                  
#define RTC_TAFCR_TAMPPRCH_Pos        (13U)                                    
#define RTC_TAFCR_TAMPPRCH_Msk        (0x3U << RTC_TAFCR_TAMPPRCH_Pos)         /*!< 0x00006000 */
#define RTC_TAFCR_TAMPPRCH            RTC_TAFCR_TAMPPRCH_Msk                   
#define RTC_TAFCR_TAMPPRCH_0          (0x1U << RTC_TAFCR_TAMPPRCH_Pos)         /*!< 0x00002000 */
#define RTC_TAFCR_TAMPPRCH_1          (0x2U << RTC_TAFCR_TAMPPRCH_Pos)         /*!< 0x00004000 */
#define RTC_TAFCR_TAMPFLT_Pos         (11U)                                    
#define RTC_TAFCR_TAMPFLT_Msk         (0x3U << RTC_TAFCR_TAMPFLT_Pos)          /*!< 0x00001800 */
#define RTC_TAFCR_TAMPFLT             RTC_TAFCR_TAMPFLT_Msk                    
#define RTC_TAFCR_TAMPFLT_0           (0x1U << RTC_TAFCR_TAMPFLT_Pos)          /*!< 0x00000800 */
#define RTC_TAFCR_TAMPFLT_1           (0x2U << RTC_TAFCR_TAMPFLT_Pos)          /*!< 0x00001000 */
#define RTC_TAFCR_TAMPFREQ_Pos        (8U)                                     
#define RTC_TAFCR_TAMPFREQ_Msk        (0x7U << RTC_TAFCR_TAMPFREQ_Pos)         /*!< 0x00000700 */
#define RTC_TAFCR_TAMPFREQ            RTC_TAFCR_TAMPFREQ_Msk                   
#define RTC_TAFCR_TAMPFREQ_0          (0x1U << RTC_TAFCR_TAMPFREQ_Pos)         /*!< 0x00000100 */
#define RTC_TAFCR_TAMPFREQ_1          (0x2U << RTC_TAFCR_TAMPFREQ_Pos)         /*!< 0x00000200 */
#define RTC_TAFCR_TAMPFREQ_2          (0x4U << RTC_TAFCR_TAMPFREQ_Pos)         /*!< 0x00000400 */
#define RTC_TAFCR_TAMPTS_Pos          (7U)                                     
#define RTC_TAFCR_TAMPTS_Msk          (0x1U << RTC_TAFCR_TAMPTS_Pos)           /*!< 0x00000080 */
#define RTC_TAFCR_TAMPTS              RTC_TAFCR_TAMPTS_Msk                     
#define RTC_TAFCR_TAMP2TRG_Pos        (4U)                                     
#define RTC_TAFCR_TAMP2TRG_Msk        (0x1U << RTC_TAFCR_TAMP2TRG_Pos)         /*!< 0x00000010 */
#define RTC_TAFCR_TAMP2TRG            RTC_TAFCR_TAMP2TRG_Msk                   
#define RTC_TAFCR_TAMP2E_Pos          (3U)                                     
#define RTC_TAFCR_TAMP2E_Msk          (0x1U << RTC_TAFCR_TAMP2E_Pos)           /*!< 0x00000008 */
#define RTC_TAFCR_TAMP2E              RTC_TAFCR_TAMP2E_Msk                     
#define RTC_TAFCR_TAMPIE_Pos          (2U)                                     
#define RTC_TAFCR_TAMPIE_Msk          (0x1U << RTC_TAFCR_TAMPIE_Pos)           /*!< 0x00000004 */
#define RTC_TAFCR_TAMPIE              RTC_TAFCR_TAMPIE_Msk                     
#define RTC_TAFCR_TAMP1TRG_Pos        (1U)                                     
#define RTC_TAFCR_TAMP1TRG_Msk        (0x1U << RTC_TAFCR_TAMP1TRG_Pos)         /*!< 0x00000002 */
#define RTC_TAFCR_TAMP1TRG            RTC_TAFCR_TAMP1TRG_Msk                   
#define RTC_TAFCR_TAMP1E_Pos          (0U)                                     
#define RTC_TAFCR_TAMP1E_Msk          (0x1U << RTC_TAFCR_TAMP1E_Pos)           /*!< 0x00000001 */
#define RTC_TAFCR_TAMP1E              RTC_TAFCR_TAMP1E_Msk                     

/* Legacy defines */
#define RTC_TAFCR_TAMPINSEL           RTC_TAFCR_TAMP1INSEL

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS_Pos       (24U)                                    
#define RTC_ALRMASSR_MASKSS_Msk       (0xFU << RTC_ALRMASSR_MASKSS_Pos)        /*!< 0x0F000000 */
#define RTC_ALRMASSR_MASKSS           RTC_ALRMASSR_MASKSS_Msk                  
#define RTC_ALRMASSR_MASKSS_0         (0x1U << RTC_ALRMASSR_MASKSS_Pos)        /*!< 0x01000000 */
#define RTC_ALRMASSR_MASKSS_1         (0x2U << RTC_ALRMASSR_MASKSS_Pos)        /*!< 0x02000000 */
#define RTC_ALRMASSR_MASKSS_2         (0x4U << RTC_ALRMASSR_MASKSS_Pos)        /*!< 0x04000000 */
#define RTC_ALRMASSR_MASKSS_3         (0x8U << RTC_ALRMASSR_MASKSS_Pos)        /*!< 0x08000000 */
#define RTC_ALRMASSR_SS_Pos           (0U)                                     
#define RTC_ALRMASSR_SS_Msk           (0x7FFFU << RTC_ALRMASSR_SS_Pos)         /*!< 0x00007FFF */
#define RTC_ALRMASSR_SS               RTC_ALRMASSR_SS_Msk                      

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS_Pos       (24U)                                    
#define RTC_ALRMBSSR_MASKSS_Msk       (0xFU << RTC_ALRMBSSR_MASKSS_Pos)        /*!< 0x0F000000 */
#define RTC_ALRMBSSR_MASKSS           RTC_ALRMBSSR_MASKSS_Msk                  
#define RTC_ALRMBSSR_MASKSS_0         (0x1U << RTC_ALRMBSSR_MASKSS_Pos)        /*!< 0x01000000 */
#define RTC_ALRMBSSR_MASKSS_1         (0x2U << RTC_ALRMBSSR_MASKSS_Pos)        /*!< 0x02000000 */
#define RTC_ALRMBSSR_MASKSS_2         (0x4U << RTC_ALRMBSSR_MASKSS_Pos)        /*!< 0x04000000 */
#define RTC_ALRMBSSR_MASKSS_3         (0x8U << RTC_ALRMBSSR_MASKSS_Pos)        /*!< 0x08000000 */
#define RTC_ALRMBSSR_SS_Pos           (0U)                                     
#define RTC_ALRMBSSR_SS_Msk           (0x7FFFU << RTC_ALRMBSSR_SS_Pos)         /*!< 0x00007FFF */
#define RTC_ALRMBSSR_SS               RTC_ALRMBSSR_SS_Msk                      

/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R_Pos                 (0U)                                     
#define RTC_BKP0R_Msk                 (0xFFFFFFFFU << RTC_BKP0R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP0R                     RTC_BKP0R_Msk                            

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R_Pos                 (0U)                                     
#define RTC_BKP1R_Msk                 (0xFFFFFFFFU << RTC_BKP1R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP1R                     RTC_BKP1R_Msk                            

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R_Pos                 (0U)                                     
#define RTC_BKP2R_Msk                 (0xFFFFFFFFU << RTC_BKP2R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP2R                     RTC_BKP2R_Msk                            

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R_Pos                 (0U)                                     
#define RTC_BKP3R_Msk                 (0xFFFFFFFFU << RTC_BKP3R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP3R                     RTC_BKP3R_Msk                            

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R_Pos                 (0U)                                     
#define RTC_BKP4R_Msk                 (0xFFFFFFFFU << RTC_BKP4R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP4R                     RTC_BKP4R_Msk                            

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R_Pos                 (0U)                                     
#define RTC_BKP5R_Msk                 (0xFFFFFFFFU << RTC_BKP5R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP5R                     RTC_BKP5R_Msk                            

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R_Pos                 (0U)                                     
#define RTC_BKP6R_Msk                 (0xFFFFFFFFU << RTC_BKP6R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP6R                     RTC_BKP6R_Msk                            

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R_Pos                 (0U)                                     
#define RTC_BKP7R_Msk                 (0xFFFFFFFFU << RTC_BKP7R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP7R                     RTC_BKP7R_Msk                            

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R_Pos                 (0U)                                     
#define RTC_BKP8R_Msk                 (0xFFFFFFFFU << RTC_BKP8R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP8R                     RTC_BKP8R_Msk                            

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R_Pos                 (0U)                                     
#define RTC_BKP9R_Msk                 (0xFFFFFFFFU << RTC_BKP9R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP9R                     RTC_BKP9R_Msk                            

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R_Pos                (0U)                                     
#define RTC_BKP10R_Msk                (0xFFFFFFFFU << RTC_BKP10R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP10R                    RTC_BKP10R_Msk                           

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R_Pos                (0U)                                     
#define RTC_BKP11R_Msk                (0xFFFFFFFFU << RTC_BKP11R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP11R                    RTC_BKP11R_Msk                           

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R_Pos                (0U)                                     
#define RTC_BKP12R_Msk                (0xFFFFFFFFU << RTC_BKP12R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP12R                    RTC_BKP12R_Msk                           

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R_Pos                (0U)                                     
#define RTC_BKP13R_Msk                (0xFFFFFFFFU << RTC_BKP13R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP13R                    RTC_BKP13R_Msk                           

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R_Pos                (0U)                                     
#define RTC_BKP14R_Msk                (0xFFFFFFFFU << RTC_BKP14R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP14R                    RTC_BKP14R_Msk                           

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R_Pos                (0U)                                     
#define RTC_BKP15R_Msk                (0xFFFFFFFFU << RTC_BKP15R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP15R                    RTC_BKP15R_Msk                           

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R_Pos                (0U)                                     
#define RTC_BKP16R_Msk                (0xFFFFFFFFU << RTC_BKP16R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP16R                    RTC_BKP16R_Msk                           

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R_Pos                (0U)                                     
#define RTC_BKP17R_Msk                (0xFFFFFFFFU << RTC_BKP17R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP17R                    RTC_BKP17R_Msk                           

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R_Pos                (0U)                                     
#define RTC_BKP18R_Msk                (0xFFFFFFFFU << RTC_BKP18R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP18R                    RTC_BKP18R_Msk                           

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R_Pos                (0U)                                     
#define RTC_BKP19R_Msk                (0xFFFFFFFFU << RTC_BKP19R_Pos)          /*!< 0xFFFFFFFF */
#define RTC_BKP19R                    RTC_BKP19R_Msk                           

/******************** Number of backup registers ******************************/
#define RTC_BKP_NUMBER                       0x000000014U


/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDIO_POWER register  ******************/
#define SDIO_POWER_PWRCTRL_Pos         (0U)                                    
#define SDIO_POWER_PWRCTRL_Msk         (0x3U << SDIO_POWER_PWRCTRL_Pos)        /*!< 0x00000003 */
#define SDIO_POWER_PWRCTRL             SDIO_POWER_PWRCTRL_Msk                  /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define SDIO_POWER_PWRCTRL_0           (0x1U << SDIO_POWER_PWRCTRL_Pos)        /*!< 0x01 */
#define SDIO_POWER_PWRCTRL_1           (0x2U << SDIO_POWER_PWRCTRL_Pos)        /*!< 0x02 */

/******************  Bit definition for SDIO_CLKCR register  ******************/
#define SDIO_CLKCR_CLKDIV_Pos          (0U)                                    
#define SDIO_CLKCR_CLKDIV_Msk          (0xFFU << SDIO_CLKCR_CLKDIV_Pos)        /*!< 0x000000FF */
#define SDIO_CLKCR_CLKDIV              SDIO_CLKCR_CLKDIV_Msk                   /*!<Clock divide factor             */
#define SDIO_CLKCR_CLKEN_Pos           (8U)                                    
#define SDIO_CLKCR_CLKEN_Msk           (0x1U << SDIO_CLKCR_CLKEN_Pos)          /*!< 0x00000100 */
#define SDIO_CLKCR_CLKEN               SDIO_CLKCR_CLKEN_Msk                    /*!<Clock enable bit                */
#define SDIO_CLKCR_PWRSAV_Pos          (9U)                                    
#define SDIO_CLKCR_PWRSAV_Msk          (0x1U << SDIO_CLKCR_PWRSAV_Pos)         /*!< 0x00000200 */
#define SDIO_CLKCR_PWRSAV              SDIO_CLKCR_PWRSAV_Msk                   /*!<Power saving configuration bit  */
#define SDIO_CLKCR_BYPASS_Pos          (10U)                                   
#define SDIO_CLKCR_BYPASS_Msk          (0x1U << SDIO_CLKCR_BYPASS_Pos)         /*!< 0x00000400 */
#define SDIO_CLKCR_BYPASS              SDIO_CLKCR_BYPASS_Msk                   /*!<Clock divider bypass enable bit */

#define SDIO_CLKCR_WIDBUS_Pos          (11U)                                   
#define SDIO_CLKCR_WIDBUS_Msk          (0x3U << SDIO_CLKCR_WIDBUS_Pos)         /*!< 0x00001800 */
#define SDIO_CLKCR_WIDBUS              SDIO_CLKCR_WIDBUS_Msk                   /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define SDIO_CLKCR_WIDBUS_0            (0x1U << SDIO_CLKCR_WIDBUS_Pos)         /*!< 0x0800 */
#define SDIO_CLKCR_WIDBUS_1            (0x2U << SDIO_CLKCR_WIDBUS_Pos)         /*!< 0x1000 */

#define SDIO_CLKCR_NEGEDGE_Pos         (13U)                                   
#define SDIO_CLKCR_NEGEDGE_Msk         (0x1U << SDIO_CLKCR_NEGEDGE_Pos)        /*!< 0x00002000 */
#define SDIO_CLKCR_NEGEDGE             SDIO_CLKCR_NEGEDGE_Msk                  /*!<SDIO_CK dephasing selection bit */
#define SDIO_CLKCR_HWFC_EN_Pos         (14U)                                   
#define SDIO_CLKCR_HWFC_EN_Msk         (0x1U << SDIO_CLKCR_HWFC_EN_Pos)        /*!< 0x00004000 */
#define SDIO_CLKCR_HWFC_EN             SDIO_CLKCR_HWFC_EN_Msk                  /*!<HW Flow Control enable          */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define SDIO_ARG_CMDARG_Pos            (0U)                                    
#define SDIO_ARG_CMDARG_Msk            (0xFFFFFFFFU << SDIO_ARG_CMDARG_Pos)    /*!< 0xFFFFFFFF */
#define SDIO_ARG_CMDARG                SDIO_ARG_CMDARG_Msk                     /*!<Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define SDIO_CMD_CMDINDEX_Pos          (0U)                                    
#define SDIO_CMD_CMDINDEX_Msk          (0x3FU << SDIO_CMD_CMDINDEX_Pos)        /*!< 0x0000003F */
#define SDIO_CMD_CMDINDEX              SDIO_CMD_CMDINDEX_Msk                   /*!<Command Index                               */

#define SDIO_CMD_WAITRESP_Pos          (6U)                                    
#define SDIO_CMD_WAITRESP_Msk          (0x3U << SDIO_CMD_WAITRESP_Pos)         /*!< 0x000000C0 */
#define SDIO_CMD_WAITRESP              SDIO_CMD_WAITRESP_Msk                   /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define SDIO_CMD_WAITRESP_0            (0x1U << SDIO_CMD_WAITRESP_Pos)         /*!< 0x0040 */
#define SDIO_CMD_WAITRESP_1            (0x2U << SDIO_CMD_WAITRESP_Pos)         /*!< 0x0080 */

#define SDIO_CMD_WAITINT_Pos           (8U)                                    
#define SDIO_CMD_WAITINT_Msk           (0x1U << SDIO_CMD_WAITINT_Pos)          /*!< 0x00000100 */
#define SDIO_CMD_WAITINT               SDIO_CMD_WAITINT_Msk                    /*!<CPSM Waits for Interrupt Request                               */
#define SDIO_CMD_WAITPEND_Pos          (9U)                                    
#define SDIO_CMD_WAITPEND_Msk          (0x1U << SDIO_CMD_WAITPEND_Pos)         /*!< 0x00000200 */
#define SDIO_CMD_WAITPEND              SDIO_CMD_WAITPEND_Msk                   /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define SDIO_CMD_CPSMEN_Pos            (10U)                                   
#define SDIO_CMD_CPSMEN_Msk            (0x1U << SDIO_CMD_CPSMEN_Pos)           /*!< 0x00000400 */
#define SDIO_CMD_CPSMEN                SDIO_CMD_CPSMEN_Msk                     /*!<Command path state machine (CPSM) Enable bit                   */
#define SDIO_CMD_SDIOSUSPEND_Pos       (11U)                                   
#define SDIO_CMD_SDIOSUSPEND_Msk       (0x1U << SDIO_CMD_SDIOSUSPEND_Pos)      /*!< 0x00000800 */
#define SDIO_CMD_SDIOSUSPEND           SDIO_CMD_SDIOSUSPEND_Msk                /*!<SD I/O suspend command                                         */
#define SDIO_CMD_ENCMDCOMPL_Pos        (12U)                                   
#define SDIO_CMD_ENCMDCOMPL_Msk        (0x1U << SDIO_CMD_ENCMDCOMPL_Pos)       /*!< 0x00001000 */
#define SDIO_CMD_ENCMDCOMPL            SDIO_CMD_ENCMDCOMPL_Msk                 /*!<Enable CMD completion                                          */
#define SDIO_CMD_NIEN_Pos              (13U)                                   
#define SDIO_CMD_NIEN_Msk              (0x1U << SDIO_CMD_NIEN_Pos)             /*!< 0x00002000 */
#define SDIO_CMD_NIEN                  SDIO_CMD_NIEN_Msk                       /*!<Not Interrupt Enable                                           */
#define SDIO_CMD_CEATACMD_Pos          (14U)                                   
#define SDIO_CMD_CEATACMD_Msk          (0x1U << SDIO_CMD_CEATACMD_Pos)         /*!< 0x00004000 */
#define SDIO_CMD_CEATACMD              SDIO_CMD_CEATACMD_Msk                   /*!<CE-ATA command                                                 */

/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define SDIO_RESPCMD_RESPCMD_Pos       (0U)                                    
#define SDIO_RESPCMD_RESPCMD_Msk       (0x3FU << SDIO_RESPCMD_RESPCMD_Pos)     /*!< 0x0000003F */
#define SDIO_RESPCMD_RESPCMD           SDIO_RESPCMD_RESPCMD_Msk                /*!<Response command index */

/******************  Bit definition for SDIO_RESP0 register  ******************/
#define SDIO_RESP0_CARDSTATUS0_Pos     (0U)                                    
#define SDIO_RESP0_CARDSTATUS0_Msk     (0xFFFFFFFFU << SDIO_RESP0_CARDSTATUS0_Pos) /*!< 0xFFFFFFFF */
#define SDIO_RESP0_CARDSTATUS0         SDIO_RESP0_CARDSTATUS0_Msk              /*!<Card Status */

/******************  Bit definition for SDIO_RESP1 register  ******************/
#define SDIO_RESP1_CARDSTATUS1_Pos     (0U)                                    
#define SDIO_RESP1_CARDSTATUS1_Msk     (0xFFFFFFFFU << SDIO_RESP1_CARDSTATUS1_Pos) /*!< 0xFFFFFFFF */
#define SDIO_RESP1_CARDSTATUS1         SDIO_RESP1_CARDSTATUS1_Msk              /*!<Card Status */

/******************  Bit definition for SDIO_RESP2 register  ******************/
#define SDIO_RESP2_CARDSTATUS2_Pos     (0U)                                    
#define SDIO_RESP2_CARDSTATUS2_Msk     (0xFFFFFFFFU << SDIO_RESP2_CARDSTATUS2_Pos) /*!< 0xFFFFFFFF */
#define SDIO_RESP2_CARDSTATUS2         SDIO_RESP2_CARDSTATUS2_Msk              /*!<Card Status */

/******************  Bit definition for SDIO_RESP3 register  ******************/
#define SDIO_RESP3_CARDSTATUS3_Pos     (0U)                                    
#define SDIO_RESP3_CARDSTATUS3_Msk     (0xFFFFFFFFU << SDIO_RESP3_CARDSTATUS3_Pos) /*!< 0xFFFFFFFF */
#define SDIO_RESP3_CARDSTATUS3         SDIO_RESP3_CARDSTATUS3_Msk              /*!<Card Status */

/******************  Bit definition for SDIO_RESP4 register  ******************/
#define SDIO_RESP4_CARDSTATUS4_Pos     (0U)                                    
#define SDIO_RESP4_CARDSTATUS4_Msk     (0xFFFFFFFFU << SDIO_RESP4_CARDSTATUS4_Pos) /*!< 0xFFFFFFFF */
#define SDIO_RESP4_CARDSTATUS4         SDIO_RESP4_CARDSTATUS4_Msk              /*!<Card Status */

/******************  Bit definition for SDIO_DTIMER register  *****************/
#define SDIO_DTIMER_DATATIME_Pos       (0U)                                    
#define SDIO_DTIMER_DATATIME_Msk       (0xFFFFFFFFU << SDIO_DTIMER_DATATIME_Pos) /*!< 0xFFFFFFFF */
#define SDIO_DTIMER_DATATIME           SDIO_DTIMER_DATATIME_Msk                /*!<Data timeout period. */

/******************  Bit definition for SDIO_DLEN register  *******************/
#define SDIO_DLEN_DATALENGTH_Pos       (0U)                                    
#define SDIO_DLEN_DATALENGTH_Msk       (0x1FFFFFFU << SDIO_DLEN_DATALENGTH_Pos) /*!< 0x01FFFFFF */
#define SDIO_DLEN_DATALENGTH           SDIO_DLEN_DATALENGTH_Msk                /*!<Data length value    */

/******************  Bit definition for SDIO_DCTRL register  ******************/
#define SDIO_DCTRL_DTEN_Pos            (0U)                                    
#define SDIO_DCTRL_DTEN_Msk            (0x1U << SDIO_DCTRL_DTEN_Pos)           /*!< 0x00000001 */
#define SDIO_DCTRL_DTEN                SDIO_DCTRL_DTEN_Msk                     /*!<Data transfer enabled bit         */
#define SDIO_DCTRL_DTDIR_Pos           (1U)                                    
#define SDIO_DCTRL_DTDIR_Msk           (0x1U << SDIO_DCTRL_DTDIR_Pos)          /*!< 0x00000002 */
#define SDIO_DCTRL_DTDIR               SDIO_DCTRL_DTDIR_Msk                    /*!<Data transfer direction selection */
#define SDIO_DCTRL_DTMODE_Pos          (2U)                                    
#define SDIO_DCTRL_DTMODE_Msk          (0x1U << SDIO_DCTRL_DTMODE_Pos)         /*!< 0x00000004 */
#define SDIO_DCTRL_DTMODE              SDIO_DCTRL_DTMODE_Msk                   /*!<Data transfer mode selection      */
#define SDIO_DCTRL_DMAEN_Pos           (3U)                                    
#define SDIO_DCTRL_DMAEN_Msk           (0x1U << SDIO_DCTRL_DMAEN_Pos)          /*!< 0x00000008 */
#define SDIO_DCTRL_DMAEN               SDIO_DCTRL_DMAEN_Msk                    /*!<DMA enabled bit                   */

#define SDIO_DCTRL_DBLOCKSIZE_Pos      (4U)                                    
#define SDIO_DCTRL_DBLOCKSIZE_Msk      (0xFU << SDIO_DCTRL_DBLOCKSIZE_Pos)     /*!< 0x000000F0 */
#define SDIO_DCTRL_DBLOCKSIZE          SDIO_DCTRL_DBLOCKSIZE_Msk               /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define SDIO_DCTRL_DBLOCKSIZE_0        (0x1U << SDIO_DCTRL_DBLOCKSIZE_Pos)     /*!< 0x0010 */
#define SDIO_DCTRL_DBLOCKSIZE_1        (0x2U << SDIO_DCTRL_DBLOCKSIZE_Pos)     /*!< 0x0020 */
#define SDIO_DCTRL_DBLOCKSIZE_2        (0x4U << SDIO_DCTRL_DBLOCKSIZE_Pos)     /*!< 0x0040 */
#define SDIO_DCTRL_DBLOCKSIZE_3        (0x8U << SDIO_DCTRL_DBLOCKSIZE_Pos)     /*!< 0x0080 */

#define SDIO_DCTRL_RWSTART_Pos         (8U)                                    
#define SDIO_DCTRL_RWSTART_Msk         (0x1U << SDIO_DCTRL_RWSTART_Pos)        /*!< 0x00000100 */
#define SDIO_DCTRL_RWSTART             SDIO_DCTRL_RWSTART_Msk                  /*!<Read wait start         */
#define SDIO_DCTRL_RWSTOP_Pos          (9U)                                    
#define SDIO_DCTRL_RWSTOP_Msk          (0x1U << SDIO_DCTRL_RWSTOP_Pos)         /*!< 0x00000200 */
#define SDIO_DCTRL_RWSTOP              SDIO_DCTRL_RWSTOP_Msk                   /*!<Read wait stop          */
#define SDIO_DCTRL_RWMOD_Pos           (10U)                                   
#define SDIO_DCTRL_RWMOD_Msk           (0x1U << SDIO_DCTRL_RWMOD_Pos)          /*!< 0x00000400 */
#define SDIO_DCTRL_RWMOD               SDIO_DCTRL_RWMOD_Msk                    /*!<Read wait mode          */
#define SDIO_DCTRL_SDIOEN_Pos          (11U)                                   
#define SDIO_DCTRL_SDIOEN_Msk          (0x1U << SDIO_DCTRL_SDIOEN_Pos)         /*!< 0x00000800 */
#define SDIO_DCTRL_SDIOEN              SDIO_DCTRL_SDIOEN_Msk                   /*!<SD I/O enable functions */

/******************  Bit definition for SDIO_DCOUNT register  *****************/
#define SDIO_DCOUNT_DATACOUNT_Pos      (0U)                                    
#define SDIO_DCOUNT_DATACOUNT_Msk      (0x1FFFFFFU << SDIO_DCOUNT_DATACOUNT_Pos) /*!< 0x01FFFFFF */
#define SDIO_DCOUNT_DATACOUNT          SDIO_DCOUNT_DATACOUNT_Msk               /*!<Data count value */

/******************  Bit definition for SDIO_STA register  ********************/
#define SDIO_STA_CCRCFAIL_Pos          (0U)                                    
#define SDIO_STA_CCRCFAIL_Msk          (0x1U << SDIO_STA_CCRCFAIL_Pos)         /*!< 0x00000001 */
#define SDIO_STA_CCRCFAIL              SDIO_STA_CCRCFAIL_Msk                   /*!<Command response received (CRC check failed)  */
#define SDIO_STA_DCRCFAIL_Pos          (1U)                                    
#define SDIO_STA_DCRCFAIL_Msk          (0x1U << SDIO_STA_DCRCFAIL_Pos)         /*!< 0x00000002 */
#define SDIO_STA_DCRCFAIL              SDIO_STA_DCRCFAIL_Msk                   /*!<Data block sent/received (CRC check failed)   */
#define SDIO_STA_CTIMEOUT_Pos          (2U)                                    
#define SDIO_STA_CTIMEOUT_Msk          (0x1U << SDIO_STA_CTIMEOUT_Pos)         /*!< 0x00000004 */
#define SDIO_STA_CTIMEOUT              SDIO_STA_CTIMEOUT_Msk                   /*!<Command response timeout                      */
#define SDIO_STA_DTIMEOUT_Pos          (3U)                                    
#define SDIO_STA_DTIMEOUT_Msk          (0x1U << SDIO_STA_DTIMEOUT_Pos)         /*!< 0x00000008 */
#define SDIO_STA_DTIMEOUT              SDIO_STA_DTIMEOUT_Msk                   /*!<Data timeout                                  */
#define SDIO_STA_TXUNDERR_Pos          (4U)                                    
#define SDIO_STA_TXUNDERR_Msk          (0x1U << SDIO_STA_TXUNDERR_Pos)         /*!< 0x00000010 */
#define SDIO_STA_TXUNDERR              SDIO_STA_TXUNDERR_Msk                   /*!<Transmit FIFO underrun error                  */
#define SDIO_STA_RXOVERR_Pos           (5U)                                    
#define SDIO_STA_RXOVERR_Msk           (0x1U << SDIO_STA_RXOVERR_Pos)          /*!< 0x00000020 */
#define SDIO_STA_RXOVERR               SDIO_STA_RXOVERR_Msk                    /*!<Received FIFO overrun error                   */
#define SDIO_STA_CMDREND_Pos           (6U)                                    
#define SDIO_STA_CMDREND_Msk           (0x1U << SDIO_STA_CMDREND_Pos)          /*!< 0x00000040 */
#define SDIO_STA_CMDREND               SDIO_STA_CMDREND_Msk                    /*!<Command response received (CRC check passed)  */
#define SDIO_STA_CMDSENT_Pos           (7U)                                    
#define SDIO_STA_CMDSENT_Msk           (0x1U << SDIO_STA_CMDSENT_Pos)          /*!< 0x00000080 */
#define SDIO_STA_CMDSENT               SDIO_STA_CMDSENT_Msk                    /*!<Command sent (no response required)           */
#define SDIO_STA_DATAEND_Pos           (8U)                                    
#define SDIO_STA_DATAEND_Msk           (0x1U << SDIO_STA_DATAEND_Pos)          /*!< 0x00000100 */
#define SDIO_STA_DATAEND               SDIO_STA_DATAEND_Msk                    /*!<Data end (data counter, SDIDCOUNT, is zero)   */
#define SDIO_STA_STBITERR_Pos          (9U)                                    
#define SDIO_STA_STBITERR_Msk          (0x1U << SDIO_STA_STBITERR_Pos)         /*!< 0x00000200 */
#define SDIO_STA_STBITERR              SDIO_STA_STBITERR_Msk                   /*!<Start bit not detected on all data signals in wide bus mode */
#define SDIO_STA_DBCKEND_Pos           (10U)                                   
#define SDIO_STA_DBCKEND_Msk           (0x1U << SDIO_STA_DBCKEND_Pos)          /*!< 0x00000400 */
#define SDIO_STA_DBCKEND               SDIO_STA_DBCKEND_Msk                    /*!<Data block sent/received (CRC check passed)   */
#define SDIO_STA_CMDACT_Pos            (11U)                                   
#define SDIO_STA_CMDACT_Msk            (0x1U << SDIO_STA_CMDACT_Pos)           /*!< 0x00000800 */
#define SDIO_STA_CMDACT                SDIO_STA_CMDACT_Msk                     /*!<Command transfer in progress                  */
#define SDIO_STA_TXACT_Pos             (12U)                                   
#define SDIO_STA_TXACT_Msk             (0x1U << SDIO_STA_TXACT_Pos)            /*!< 0x00001000 */
#define SDIO_STA_TXACT                 SDIO_STA_TXACT_Msk                      /*!<Data transmit in progress                     */
#define SDIO_STA_RXACT_Pos             (13U)                                   
#define SDIO_STA_RXACT_Msk             (0x1U << SDIO_STA_RXACT_Pos)            /*!< 0x00002000 */
#define SDIO_STA_RXACT                 SDIO_STA_RXACT_Msk                      /*!<Data receive in progress                      */
#define SDIO_STA_TXFIFOHE_Pos          (14U)                                   
#define SDIO_STA_TXFIFOHE_Msk          (0x1U << SDIO_STA_TXFIFOHE_Pos)         /*!< 0x00004000 */
#define SDIO_STA_TXFIFOHE              SDIO_STA_TXFIFOHE_Msk                   /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define SDIO_STA_RXFIFOHF_Pos          (15U)                                   
#define SDIO_STA_RXFIFOHF_Msk          (0x1U << SDIO_STA_RXFIFOHF_Pos)         /*!< 0x00008000 */
#define SDIO_STA_RXFIFOHF              SDIO_STA_RXFIFOHF_Msk                   /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define SDIO_STA_TXFIFOF_Pos           (16U)                                   
#define SDIO_STA_TXFIFOF_Msk           (0x1U << SDIO_STA_TXFIFOF_Pos)          /*!< 0x00010000 */
#define SDIO_STA_TXFIFOF               SDIO_STA_TXFIFOF_Msk                    /*!<Transmit FIFO full                            */
#define SDIO_STA_RXFIFOF_Pos           (17U)                                   
#define SDIO_STA_RXFIFOF_Msk           (0x1U << SDIO_STA_RXFIFOF_Pos)          /*!< 0x00020000 */
#define SDIO_STA_RXFIFOF               SDIO_STA_RXFIFOF_Msk                    /*!<Receive FIFO full                             */
#define SDIO_STA_TXFIFOE_Pos           (18U)                                   
#define SDIO_STA_TXFIFOE_Msk           (0x1U << SDIO_STA_TXFIFOE_Pos)          /*!< 0x00040000 */
#define SDIO_STA_TXFIFOE               SDIO_STA_TXFIFOE_Msk                    /*!<Transmit FIFO empty                           */
#define SDIO_STA_RXFIFOE_Pos           (19U)                                   
#define SDIO_STA_RXFIFOE_Msk           (0x1U << SDIO_STA_RXFIFOE_Pos)          /*!< 0x00080000 */
#define SDIO_STA_RXFIFOE               SDIO_STA_RXFIFOE_Msk                    /*!<Receive FIFO empty                            */
#define SDIO_STA_TXDAVL_Pos            (20U)                                   
#define SDIO_STA_TXDAVL_Msk            (0x1U << SDIO_STA_TXDAVL_Pos)           /*!< 0x00100000 */
#define SDIO_STA_TXDAVL                SDIO_STA_TXDAVL_Msk                     /*!<Data available in transmit FIFO               */
#define SDIO_STA_RXDAVL_Pos            (21U)                                   
#define SDIO_STA_RXDAVL_Msk            (0x1U << SDIO_STA_RXDAVL_Pos)           /*!< 0x00200000 */
#define SDIO_STA_RXDAVL                SDIO_STA_RXDAVL_Msk                     /*!<Data available in receive FIFO                */
#define SDIO_STA_SDIOIT_Pos            (22U)                                   
#define SDIO_STA_SDIOIT_Msk            (0x1U << SDIO_STA_SDIOIT_Pos)           /*!< 0x00400000 */
#define SDIO_STA_SDIOIT                SDIO_STA_SDIOIT_Msk                     /*!<SDIO interrupt received                       */
#define SDIO_STA_CEATAEND_Pos          (23U)                                   
#define SDIO_STA_CEATAEND_Msk          (0x1U << SDIO_STA_CEATAEND_Pos)         /*!< 0x00800000 */
#define SDIO_STA_CEATAEND              SDIO_STA_CEATAEND_Msk                   /*!<CE-ATA command completion signal received for CMD61 */

/*******************  Bit definition for SDIO_ICR register  *******************/
#define SDIO_ICR_CCRCFAILC_Pos         (0U)                                    
#define SDIO_ICR_CCRCFAILC_Msk         (0x1U << SDIO_ICR_CCRCFAILC_Pos)        /*!< 0x00000001 */
#define SDIO_ICR_CCRCFAILC             SDIO_ICR_CCRCFAILC_Msk                  /*!<CCRCFAIL flag clear bit */
#define SDIO_ICR_DCRCFAILC_Pos         (1U)                                    
#define SDIO_ICR_DCRCFAILC_Msk         (0x1U << SDIO_ICR_DCRCFAILC_Pos)        /*!< 0x00000002 */
#define SDIO_ICR_DCRCFAILC             SDIO_ICR_DCRCFAILC_Msk                  /*!<DCRCFAIL flag clear bit */
#define SDIO_ICR_CTIMEOUTC_Pos         (2U)                                    
#define SDIO_ICR_CTIMEOUTC_Msk         (0x1U << SDIO_ICR_CTIMEOUTC_Pos)        /*!< 0x00000004 */
#define SDIO_ICR_CTIMEOUTC             SDIO_ICR_CTIMEOUTC_Msk                  /*!<CTIMEOUT flag clear bit */
#define SDIO_ICR_DTIMEOUTC_Pos         (3U)                                    
#define SDIO_ICR_DTIMEOUTC_Msk         (0x1U << SDIO_ICR_DTIMEOUTC_Pos)        /*!< 0x00000008 */
#define SDIO_ICR_DTIMEOUTC             SDIO_ICR_DTIMEOUTC_Msk                  /*!<DTIMEOUT flag clear bit */
#define SDIO_ICR_TXUNDERRC_Pos         (4U)                                    
#define SDIO_ICR_TXUNDERRC_Msk         (0x1U << SDIO_ICR_TXUNDERRC_Pos)        /*!< 0x00000010 */
#define SDIO_ICR_TXUNDERRC             SDIO_ICR_TXUNDERRC_Msk                  /*!<TXUNDERR flag clear bit */
#define SDIO_ICR_RXOVERRC_Pos          (5U)                                    
#define SDIO_ICR_RXOVERRC_Msk          (0x1U << SDIO_ICR_RXOVERRC_Pos)         /*!< 0x00000020 */
#define SDIO_ICR_RXOVERRC              SDIO_ICR_RXOVERRC_Msk                   /*!<RXOVERR flag clear bit  */
#define SDIO_ICR_CMDRENDC_Pos          (6U)                                    
#define SDIO_ICR_CMDRENDC_Msk          (0x1U << SDIO_ICR_CMDRENDC_Pos)         /*!< 0x00000040 */
#define SDIO_ICR_CMDRENDC              SDIO_ICR_CMDRENDC_Msk                   /*!<CMDREND flag clear bit  */
#define SDIO_ICR_CMDSENTC_Pos          (7U)                                    
#define SDIO_ICR_CMDSENTC_Msk          (0x1U << SDIO_ICR_CMDSENTC_Pos)         /*!< 0x00000080 */
#define SDIO_ICR_CMDSENTC              SDIO_ICR_CMDSENTC_Msk                   /*!<CMDSENT flag clear bit  */
#define SDIO_ICR_DATAENDC_Pos          (8U)                                    
#define SDIO_ICR_DATAENDC_Msk          (0x1U << SDIO_ICR_DATAENDC_Pos)         /*!< 0x00000100 */
#define SDIO_ICR_DATAENDC              SDIO_ICR_DATAENDC_Msk                   /*!<DATAEND flag clear bit  */
#define SDIO_ICR_STBITERRC_Pos         (9U)                                    
#define SDIO_ICR_STBITERRC_Msk         (0x1U << SDIO_ICR_STBITERRC_Pos)        /*!< 0x00000200 */
#define SDIO_ICR_STBITERRC             SDIO_ICR_STBITERRC_Msk                  /*!<STBITERR flag clear bit */
#define SDIO_ICR_DBCKENDC_Pos          (10U)                                   
#define SDIO_ICR_DBCKENDC_Msk          (0x1U << SDIO_ICR_DBCKENDC_Pos)         /*!< 0x00000400 */
#define SDIO_ICR_DBCKENDC              SDIO_ICR_DBCKENDC_Msk                   /*!<DBCKEND flag clear bit  */
#define SDIO_ICR_SDIOITC_Pos           (22U)                                   
#define SDIO_ICR_SDIOITC_Msk           (0x1U << SDIO_ICR_SDIOITC_Pos)          /*!< 0x00400000 */
#define SDIO_ICR_SDIOITC               SDIO_ICR_SDIOITC_Msk                    /*!<SDIOIT flag clear bit   */
#define SDIO_ICR_CEATAENDC_Pos         (23U)                                   
#define SDIO_ICR_CEATAENDC_Msk         (0x1U << SDIO_ICR_CEATAENDC_Pos)        /*!< 0x00800000 */
#define SDIO_ICR_CEATAENDC             SDIO_ICR_CEATAENDC_Msk                  /*!<CEATAEND flag clear bit */

/******************  Bit definition for SDIO_MASK register  *******************/
#define SDIO_MASK_CCRCFAILIE_Pos       (0U)                                    
#define SDIO_MASK_CCRCFAILIE_Msk       (0x1U << SDIO_MASK_CCRCFAILIE_Pos)      /*!< 0x00000001 */
#define SDIO_MASK_CCRCFAILIE           SDIO_MASK_CCRCFAILIE_Msk                /*!<Command CRC Fail Interrupt Enable          */
#define SDIO_MASK_DCRCFAILIE_Pos       (1U)                                    
#define SDIO_MASK_DCRCFAILIE_Msk       (0x1U << SDIO_MASK_DCRCFAILIE_Pos)      /*!< 0x00000002 */
#define SDIO_MASK_DCRCFAILIE           SDIO_MASK_DCRCFAILIE_Msk                /*!<Data CRC Fail Interrupt Enable             */
#define SDIO_MASK_CTIMEOUTIE_Pos       (2U)                                    
#define SDIO_MASK_CTIMEOUTIE_Msk       (0x1U << SDIO_MASK_CTIMEOUTIE_Pos)      /*!< 0x00000004 */
#define SDIO_MASK_CTIMEOUTIE           SDIO_MASK_CTIMEOUTIE_Msk                /*!<Command TimeOut Interrupt Enable           */
#define SDIO_MASK_DTIMEOUTIE_Pos       (3U)                                    
#define SDIO_MASK_DTIMEOUTIE_Msk       (0x1U << SDIO_MASK_DTIMEOUTIE_Pos)      /*!< 0x00000008 */
#define SDIO_MASK_DTIMEOUTIE           SDIO_MASK_DTIMEOUTIE_Msk                /*!<Data TimeOut Interrupt Enable              */
#define SDIO_MASK_TXUNDERRIE_Pos       (4U)                                    
#define SDIO_MASK_TXUNDERRIE_Msk       (0x1U << SDIO_MASK_TXUNDERRIE_Pos)      /*!< 0x00000010 */
#define SDIO_MASK_TXUNDERRIE           SDIO_MASK_TXUNDERRIE_Msk                /*!<Tx FIFO UnderRun Error Interrupt Enable    */
#define SDIO_MASK_RXOVERRIE_Pos        (5U)                                    
#define SDIO_MASK_RXOVERRIE_Msk        (0x1U << SDIO_MASK_RXOVERRIE_Pos)       /*!< 0x00000020 */
#define SDIO_MASK_RXOVERRIE            SDIO_MASK_RXOVERRIE_Msk                 /*!<Rx FIFO OverRun Error Interrupt Enable     */
#define SDIO_MASK_CMDRENDIE_Pos        (6U)                                    
#define SDIO_MASK_CMDRENDIE_Msk        (0x1U << SDIO_MASK_CMDRENDIE_Pos)       /*!< 0x00000040 */
#define SDIO_MASK_CMDRENDIE            SDIO_MASK_CMDRENDIE_Msk                 /*!<Command Response Received Interrupt Enable */
#define SDIO_MASK_CMDSENTIE_Pos        (7U)                                    
#define SDIO_MASK_CMDSENTIE_Msk        (0x1U << SDIO_MASK_CMDSENTIE_Pos)       /*!< 0x00000080 */
#define SDIO_MASK_CMDSENTIE            SDIO_MASK_CMDSENTIE_Msk                 /*!<Command Sent Interrupt Enable              */
#define SDIO_MASK_DATAENDIE_Pos        (8U)                                    
#define SDIO_MASK_DATAENDIE_Msk        (0x1U << SDIO_MASK_DATAENDIE_Pos)       /*!< 0x00000100 */
#define SDIO_MASK_DATAENDIE            SDIO_MASK_DATAENDIE_Msk                 /*!<Data End Interrupt Enable                  */
#define SDIO_MASK_STBITERRIE_Pos       (9U)                                    
#define SDIO_MASK_STBITERRIE_Msk       (0x1U << SDIO_MASK_STBITERRIE_Pos)      /*!< 0x00000200 */
#define SDIO_MASK_STBITERRIE           SDIO_MASK_STBITERRIE_Msk                /*!<Start Bit Error Interrupt Enable           */
#define SDIO_MASK_DBCKENDIE_Pos        (10U)                                   
#define SDIO_MASK_DBCKENDIE_Msk        (0x1U << SDIO_MASK_DBCKENDIE_Pos)       /*!< 0x00000400 */
#define SDIO_MASK_DBCKENDIE            SDIO_MASK_DBCKENDIE_Msk                 /*!<Data Block End Interrupt Enable            */
#define SDIO_MASK_CMDACTIE_Pos         (11U)                                   
#define SDIO_MASK_CMDACTIE_Msk         (0x1U << SDIO_MASK_CMDACTIE_Pos)        /*!< 0x00000800 */
#define SDIO_MASK_CMDACTIE             SDIO_MASK_CMDACTIE_Msk                  /*!<CCommand Acting Interrupt Enable           */
#define SDIO_MASK_TXACTIE_Pos          (12U)                                   
#define SDIO_MASK_TXACTIE_Msk          (0x1U << SDIO_MASK_TXACTIE_Pos)         /*!< 0x00001000 */
#define SDIO_MASK_TXACTIE              SDIO_MASK_TXACTIE_Msk                   /*!<Data Transmit Acting Interrupt Enable      */
#define SDIO_MASK_RXACTIE_Pos          (13U)                                   
#define SDIO_MASK_RXACTIE_Msk          (0x1U << SDIO_MASK_RXACTIE_Pos)         /*!< 0x00002000 */
#define SDIO_MASK_RXACTIE              SDIO_MASK_RXACTIE_Msk                   /*!<Data receive acting interrupt enabled      */
#define SDIO_MASK_TXFIFOHEIE_Pos       (14U)                                   
#define SDIO_MASK_TXFIFOHEIE_Msk       (0x1U << SDIO_MASK_TXFIFOHEIE_Pos)      /*!< 0x00004000 */
#define SDIO_MASK_TXFIFOHEIE           SDIO_MASK_TXFIFOHEIE_Msk                /*!<Tx FIFO Half Empty interrupt Enable        */
#define SDIO_MASK_RXFIFOHFIE_Pos       (15U)                                   
#define SDIO_MASK_RXFIFOHFIE_Msk       (0x1U << SDIO_MASK_RXFIFOHFIE_Pos)      /*!< 0x00008000 */
#define SDIO_MASK_RXFIFOHFIE           SDIO_MASK_RXFIFOHFIE_Msk                /*!<Rx FIFO Half Full interrupt Enable         */
#define SDIO_MASK_TXFIFOFIE_Pos        (16U)                                   
#define SDIO_MASK_TXFIFOFIE_Msk        (0x1U << SDIO_MASK_TXFIFOFIE_Pos)       /*!< 0x00010000 */
#define SDIO_MASK_TXFIFOFIE            SDIO_MASK_TXFIFOFIE_Msk                 /*!<Tx FIFO Full interrupt Enable              */
#define SDIO_MASK_RXFIFOFIE_Pos        (17U)                                   
#define SDIO_MASK_RXFIFOFIE_Msk        (0x1U << SDIO_MASK_RXFIFOFIE_Pos)       /*!< 0x00020000 */
#define SDIO_MASK_RXFIFOFIE            SDIO_MASK_RXFIFOFIE_Msk                 /*!<Rx FIFO Full interrupt Enable              */
#define SDIO_MASK_TXFIFOEIE_Pos        (18U)                                   
#define SDIO_MASK_TXFIFOEIE_Msk        (0x1U << SDIO_MASK_TXFIFOEIE_Pos)       /*!< 0x00040000 */
#define SDIO_MASK_TXFIFOEIE            SDIO_MASK_TXFIFOEIE_Msk                 /*!<Tx FIFO Empty interrupt Enable             */
#define SDIO_MASK_RXFIFOEIE_Pos        (19U)                                   
#define SDIO_MASK_RXFIFOEIE_Msk        (0x1U << SDIO_MASK_RXFIFOEIE_Pos)       /*!< 0x00080000 */
#define SDIO_MASK_RXFIFOEIE            SDIO_MASK_RXFIFOEIE_Msk                 /*!<Rx FIFO Empty interrupt Enable             */
#define SDIO_MASK_TXDAVLIE_Pos         (20U)                                   
#define SDIO_MASK_TXDAVLIE_Msk         (0x1U << SDIO_MASK_TXDAVLIE_Pos)        /*!< 0x00100000 */
#define SDIO_MASK_TXDAVLIE             SDIO_MASK_TXDAVLIE_Msk                  /*!<Data available in Tx FIFO interrupt Enable */
#define SDIO_MASK_RXDAVLIE_Pos         (21U)                                   
#define SDIO_MASK_RXDAVLIE_Msk         (0x1U << SDIO_MASK_RXDAVLIE_Pos)        /*!< 0x00200000 */
#define SDIO_MASK_RXDAVLIE             SDIO_MASK_RXDAVLIE_Msk                  /*!<Data available in Rx FIFO interrupt Enable */
#define SDIO_MASK_SDIOITIE_Pos         (22U)                                   
#define SDIO_MASK_SDIOITIE_Msk         (0x1U << SDIO_MASK_SDIOITIE_Pos)        /*!< 0x00400000 */
#define SDIO_MASK_SDIOITIE             SDIO_MASK_SDIOITIE_Msk                  /*!<SDIO Mode Interrupt Received interrupt Enable */
#define SDIO_MASK_CEATAENDIE_Pos       (23U)                                   
#define SDIO_MASK_CEATAENDIE_Msk       (0x1U << SDIO_MASK_CEATAENDIE_Pos)      /*!< 0x00800000 */
#define SDIO_MASK_CEATAENDIE           SDIO_MASK_CEATAENDIE_Msk                /*!<CE-ATA command completion signal received Interrupt Enable */

/*****************  Bit definition for SDIO_FIFOCNT register  *****************/
#define SDIO_FIFOCNT_FIFOCOUNT_Pos     (0U)                                    
#define SDIO_FIFOCNT_FIFOCOUNT_Msk     (0xFFFFFFU << SDIO_FIFOCNT_FIFOCOUNT_Pos) /*!< 0x00FFFFFF */
#define SDIO_FIFOCNT_FIFOCOUNT         SDIO_FIFOCNT_FIFOCOUNT_Msk              /*!<Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDIO_FIFO register  *******************/
#define SDIO_FIFO_FIFODATA_Pos         (0U)                                    
#define SDIO_FIFO_FIFODATA_Msk         (0xFFFFFFFFU << SDIO_FIFO_FIFODATA_Pos) /*!< 0xFFFFFFFF */
#define SDIO_FIFO_FIFODATA             SDIO_FIFO_FIFODATA_Msk                  /*!<Receive and transmit FIFO data */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
#define SPI_I2S_FULLDUPLEX_SUPPORT                                             /*!< I2S Full-Duplex support */

/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos            (0U)                                       
#define SPI_CR1_CPHA_Msk            (0x1U << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!<Clock Phase      */
#define SPI_CR1_CPOL_Pos            (1U)                                       
#define SPI_CR1_CPOL_Msk            (0x1U << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!<Clock Polarity   */
#define SPI_CR1_MSTR_Pos            (2U)                                       
#define SPI_CR1_MSTR_Msk            (0x1U << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!<Master Selection */

#define SPI_CR1_BR_Pos              (3U)                                       
#define SPI_CR1_BR_Msk              (0x7U << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!<BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1U << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2U << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4U << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */

#define SPI_CR1_SPE_Pos             (6U)                                       
#define SPI_CR1_SPE_Msk             (0x1U << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!<SPI Enable                          */
#define SPI_CR1_LSBFIRST_Pos        (7U)                                       
#define SPI_CR1_LSBFIRST_Msk        (0x1U << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!<Frame Format                        */
#define SPI_CR1_SSI_Pos             (8U)                                       
#define SPI_CR1_SSI_Msk             (0x1U << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!<Internal slave select               */
#define SPI_CR1_SSM_Pos             (9U)                                       
#define SPI_CR1_SSM_Msk             (0x1U << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!<Software slave management           */
#define SPI_CR1_RXONLY_Pos          (10U)                                      
#define SPI_CR1_RXONLY_Msk          (0x1U << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!<Receive only                        */
#define SPI_CR1_DFF_Pos             (11U)                                      
#define SPI_CR1_DFF_Msk             (0x1U << SPI_CR1_DFF_Pos)                  /*!< 0x00000800 */
#define SPI_CR1_DFF                 SPI_CR1_DFF_Msk                            /*!<Data Frame Format                   */
#define SPI_CR1_CRCNEXT_Pos         (12U)                                      
#define SPI_CR1_CRCNEXT_Msk         (0x1U << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!<Transmit CRC next                   */
#define SPI_CR1_CRCEN_Pos           (13U)                                      
#define SPI_CR1_CRCEN_Msk           (0x1U << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!<Hardware CRC calculation enable     */
#define SPI_CR1_BIDIOE_Pos          (14U)                                      
#define SPI_CR1_BIDIOE_Msk          (0x1U << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!<Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)                                      
#define SPI_CR1_BIDIMODE_Msk        (0x1U << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos         (0U)                                       
#define SPI_CR2_RXDMAEN_Msk         (0x1U << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!<Rx Buffer DMA Enable                 */
#define SPI_CR2_TXDMAEN_Pos         (1U)                                       
#define SPI_CR2_TXDMAEN_Msk         (0x1U << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!<Tx Buffer DMA Enable                 */
#define SPI_CR2_SSOE_Pos            (2U)                                       
#define SPI_CR2_SSOE_Msk            (0x1U << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!<SS Output Enable                     */
#define SPI_CR2_FRF_Pos             (4U)                                       
#define SPI_CR2_FRF_Msk             (0x1U << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!<Frame Format                         */
#define SPI_CR2_ERRIE_Pos           (5U)                                       
#define SPI_CR2_ERRIE_Msk           (0x1U << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!<Error Interrupt Enable               */
#define SPI_CR2_RXNEIE_Pos          (6U)                                       
#define SPI_CR2_RXNEIE_Msk          (0x1U << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!<RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)                                       
#define SPI_CR2_TXEIE_Msk           (0x1U << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos             (0U)                                       
#define SPI_SR_RXNE_Msk             (0x1U << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!<Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)                                       
#define SPI_SR_TXE_Msk              (0x1U << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!<Transmit buffer Empty    */
#define SPI_SR_CHSIDE_Pos           (2U)                                       
#define SPI_SR_CHSIDE_Msk           (0x1U << SPI_SR_CHSIDE_Pos)                /*!< 0x00000004 */
#define SPI_SR_CHSIDE               SPI_SR_CHSIDE_Msk                          /*!<Channel side             */
#define SPI_SR_UDR_Pos              (3U)                                       
#define SPI_SR_UDR_Msk              (0x1U << SPI_SR_UDR_Pos)                   /*!< 0x00000008 */
#define SPI_SR_UDR                  SPI_SR_UDR_Msk                             /*!<Underrun flag            */
#define SPI_SR_CRCERR_Pos           (4U)                                       
#define SPI_SR_CRCERR_Msk           (0x1U << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!<CRC Error flag           */
#define SPI_SR_MODF_Pos             (5U)                                       
#define SPI_SR_MODF_Msk             (0x1U << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!<Mode fault               */
#define SPI_SR_OVR_Pos              (6U)                                       
#define SPI_SR_OVR_Msk              (0x1U << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!<Overrun flag             */
#define SPI_SR_BSY_Pos              (7U)                                       
#define SPI_SR_BSY_Msk              (0x1U << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!<Busy flag                */
#define SPI_SR_FRE_Pos              (8U)                                       
#define SPI_SR_FRE_Msk              (0x1U << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos               (0U)                                       
#define SPI_DR_DR_Msk               (0xFFFFU << SPI_DR_DR_Pos)                 /*!< 0x0000FFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)                                       
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFU << SPI_CRCPR_CRCPOLY_Pos)         /*!< 0x0000FFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)                                       
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFU << SPI_RXCRCR_RXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)                                       
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFU << SPI_TXCRCR_TXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define SPI_I2SCFGR_CHLEN_Pos       (0U)                                       
#define SPI_I2SCFGR_CHLEN_Msk       (0x1U << SPI_I2SCFGR_CHLEN_Pos)            /*!< 0x00000001 */
#define SPI_I2SCFGR_CHLEN           SPI_I2SCFGR_CHLEN_Msk                      /*!<Channel length (number of bits per audio channel) */

#define SPI_I2SCFGR_DATLEN_Pos      (1U)                                       
#define SPI_I2SCFGR_DATLEN_Msk      (0x3U << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000006 */
#define SPI_I2SCFGR_DATLEN          SPI_I2SCFGR_DATLEN_Msk                     /*!<DATLEN[1:0] bits (Data length to be transferred)  */
#define SPI_I2SCFGR_DATLEN_0        (0x1U << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000002 */
#define SPI_I2SCFGR_DATLEN_1        (0x2U << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000004 */

#define SPI_I2SCFGR_CKPOL_Pos       (3U)                                       
#define SPI_I2SCFGR_CKPOL_Msk       (0x1U << SPI_I2SCFGR_CKPOL_Pos)            /*!< 0x00000008 */
#define SPI_I2SCFGR_CKPOL           SPI_I2SCFGR_CKPOL_Msk                      /*!<steady state clock polarity               */

#define SPI_I2SCFGR_I2SSTD_Pos      (4U)                                       
#define SPI_I2SCFGR_I2SSTD_Msk      (0x3U << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000030 */
#define SPI_I2SCFGR_I2SSTD          SPI_I2SCFGR_I2SSTD_Msk                     /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define SPI_I2SCFGR_I2SSTD_0        (0x1U << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000010 */
#define SPI_I2SCFGR_I2SSTD_1        (0x2U << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000020 */

#define SPI_I2SCFGR_PCMSYNC_Pos     (7U)                                       
#define SPI_I2SCFGR_PCMSYNC_Msk     (0x1U << SPI_I2SCFGR_PCMSYNC_Pos)          /*!< 0x00000080 */
#define SPI_I2SCFGR_PCMSYNC         SPI_I2SCFGR_PCMSYNC_Msk                    /*!<PCM frame synchronization                 */

#define SPI_I2SCFGR_I2SCFG_Pos      (8U)                                       
#define SPI_I2SCFGR_I2SCFG_Msk      (0x3U << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000300 */
#define SPI_I2SCFGR_I2SCFG          SPI_I2SCFGR_I2SCFG_Msk                     /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define SPI_I2SCFGR_I2SCFG_0        (0x1U << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000100 */
#define SPI_I2SCFGR_I2SCFG_1        (0x2U << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000200 */

#define SPI_I2SCFGR_I2SE_Pos        (10U)                                      
#define SPI_I2SCFGR_I2SE_Msk        (0x1U << SPI_I2SCFGR_I2SE_Pos)             /*!< 0x00000400 */
#define SPI_I2SCFGR_I2SE            SPI_I2SCFGR_I2SE_Msk                       /*!<I2S Enable         */
#define SPI_I2SCFGR_I2SMOD_Pos      (11U)                                      
#define SPI_I2SCFGR_I2SMOD_Msk      (0x1U << SPI_I2SCFGR_I2SMOD_Pos)           /*!< 0x00000800 */
#define SPI_I2SCFGR_I2SMOD          SPI_I2SCFGR_I2SMOD_Msk                     /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define SPI_I2SPR_I2SDIV_Pos        (0U)                                       
#define SPI_I2SPR_I2SDIV_Msk        (0xFFU << SPI_I2SPR_I2SDIV_Pos)            /*!< 0x000000FF */
#define SPI_I2SPR_I2SDIV            SPI_I2SPR_I2SDIV_Msk                       /*!<I2S Linear prescaler         */
#define SPI_I2SPR_ODD_Pos           (8U)                                       
#define SPI_I2SPR_ODD_Msk           (0x1U << SPI_I2SPR_ODD_Pos)                /*!< 0x00000100 */
#define SPI_I2SPR_ODD               SPI_I2SPR_ODD_Msk                          /*!<Odd factor for the prescaler */
#define SPI_I2SPR_MCKOE_Pos         (9U)                                       
#define SPI_I2SPR_MCKOE_Msk         (0x1U << SPI_I2SPR_MCKOE_Pos)              /*!< 0x00000200 */
#define SPI_I2SPR_MCKOE             SPI_I2SPR_MCKOE_Msk                        /*!<Master Clock Output Enable   */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE_Pos           (0U)                              
#define SYSCFG_MEMRMP_MEM_MODE_Msk           (0x3U << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000003 */
#define SYSCFG_MEMRMP_MEM_MODE               SYSCFG_MEMRMP_MEM_MODE_Msk        /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0             (0x1U << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000001 */
#define SYSCFG_MEMRMP_MEM_MODE_1             (0x2U << SYSCFG_MEMRMP_MEM_MODE_Pos) /*!< 0x00000002 */
/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_ADC1DC2_Pos               (16U)                             
#define SYSCFG_PMC_ADC1DC2_Msk               (0x1U << SYSCFG_PMC_ADC1DC2_Pos)  /*!< 0x00010000 */
#define SYSCFG_PMC_ADC1DC2                   SYSCFG_PMC_ADC1DC2_Msk            /*!< Refer to AN4073 on how to use this bit  */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0_Pos             (0U)                              
#define SYSCFG_EXTICR1_EXTI0_Msk             (0xFU << SYSCFG_EXTICR1_EXTI0_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR1_EXTI0                 SYSCFG_EXTICR1_EXTI0_Msk          /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1_Pos             (4U)                              
#define SYSCFG_EXTICR1_EXTI1_Msk             (0xFU << SYSCFG_EXTICR1_EXTI1_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR1_EXTI1                 SYSCFG_EXTICR1_EXTI1_Msk          /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2_Pos             (8U)                              
#define SYSCFG_EXTICR1_EXTI2_Msk             (0xFU << SYSCFG_EXTICR1_EXTI2_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR1_EXTI2                 SYSCFG_EXTICR1_EXTI2_Msk          /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3_Pos             (12U)                             
#define SYSCFG_EXTICR1_EXTI3_Msk             (0xFU << SYSCFG_EXTICR1_EXTI3_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR1_EXTI3                 SYSCFG_EXTICR1_EXTI3_Msk          /*!<EXTI 3 configuration */
/**
  * @brief   EXTI0 configuration  
  */
#define SYSCFG_EXTICR1_EXTI0_PA              0x0000U                           /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB              0x0001U                           /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC              0x0002U                           /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD              0x0003U                           /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE              0x0004U                           /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH              0x0007U                           /*!<PH[0] pin */

/**
  * @brief   EXTI1 configuration  
  */
#define SYSCFG_EXTICR1_EXTI1_PA              0x0000U                           /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB              0x0010U                           /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC              0x0020U                           /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD              0x0030U                           /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE              0x0040U                           /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH              0x0070U                           /*!<PH[1] pin */

/**
  * @brief   EXTI2 configuration  
  */
#define SYSCFG_EXTICR1_EXTI2_PA              0x0000U                           /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB              0x0100U                           /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC              0x0200U                           /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD              0x0300U                           /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE              0x0400U                           /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH              0x0700U                           /*!<PH[2] pin */

/**
  * @brief   EXTI3 configuration  
  */
#define SYSCFG_EXTICR1_EXTI3_PA              0x0000U                           /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB              0x1000U                           /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC              0x2000U                           /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD              0x3000U                           /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE              0x4000U                           /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH              0x7000U                           /*!<PH[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4_Pos             (0U)                              
#define SYSCFG_EXTICR2_EXTI4_Msk             (0xFU << SYSCFG_EXTICR2_EXTI4_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR2_EXTI4                 SYSCFG_EXTICR2_EXTI4_Msk          /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5_Pos             (4U)                              
#define SYSCFG_EXTICR2_EXTI5_Msk             (0xFU << SYSCFG_EXTICR2_EXTI5_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR2_EXTI5                 SYSCFG_EXTICR2_EXTI5_Msk          /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6_Pos             (8U)                              
#define SYSCFG_EXTICR2_EXTI6_Msk             (0xFU << SYSCFG_EXTICR2_EXTI6_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR2_EXTI6                 SYSCFG_EXTICR2_EXTI6_Msk          /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7_Pos             (12U)                             
#define SYSCFG_EXTICR2_EXTI7_Msk             (0xFU << SYSCFG_EXTICR2_EXTI7_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR2_EXTI7                 SYSCFG_EXTICR2_EXTI7_Msk          /*!<EXTI 7 configuration */

/**
  * @brief   EXTI4 configuration  
  */
#define SYSCFG_EXTICR2_EXTI4_PA              0x0000U                           /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB              0x0001U                           /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC              0x0002U                           /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD              0x0003U                           /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE              0x0004U                           /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH              0x0007U                           /*!<PH[4] pin */

/**
  * @brief   EXTI5 configuration  
  */
#define SYSCFG_EXTICR2_EXTI5_PA              0x0000U                           /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB              0x0010U                           /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC              0x0020U                           /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD              0x0030U                           /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE              0x0040U                           /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH              0x0070U                           /*!<PH[5] pin */

/**
  * @brief   EXTI6 configuration  
  */
#define SYSCFG_EXTICR2_EXTI6_PA              0x0000U                           /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB              0x0100U                           /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC              0x0200U                           /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD              0x0300U                           /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE              0x0400U                           /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH              0x0700U                           /*!<PH[6] pin */

/**
  * @brief   EXTI7 configuration  
  */
#define SYSCFG_EXTICR2_EXTI7_PA              0x0000U                           /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB              0x1000U                           /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC              0x2000U                           /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD              0x3000U                           /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE              0x4000U                           /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH              0x7000U                           /*!<PH[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8_Pos             (0U)                              
#define SYSCFG_EXTICR3_EXTI8_Msk             (0xFU << SYSCFG_EXTICR3_EXTI8_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR3_EXTI8                 SYSCFG_EXTICR3_EXTI8_Msk          /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9_Pos             (4U)                              
#define SYSCFG_EXTICR3_EXTI9_Msk             (0xFU << SYSCFG_EXTICR3_EXTI9_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR3_EXTI9                 SYSCFG_EXTICR3_EXTI9_Msk          /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10_Pos            (8U)                              
#define SYSCFG_EXTICR3_EXTI10_Msk            (0xFU << SYSCFG_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR3_EXTI10                SYSCFG_EXTICR3_EXTI10_Msk         /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11_Pos            (12U)                             
#define SYSCFG_EXTICR3_EXTI11_Msk            (0xFU << SYSCFG_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR3_EXTI11                SYSCFG_EXTICR3_EXTI11_Msk         /*!<EXTI 11 configuration */

/**
  * @brief   EXTI8 configuration  
  */
#define SYSCFG_EXTICR3_EXTI8_PA              0x0000U                           /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB              0x0001U                           /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC              0x0002U                           /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD              0x0003U                           /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE              0x0004U                           /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH              0x0007U                           /*!<PH[8] pin */

/**
  * @brief   EXTI9 configuration  
  */
#define SYSCFG_EXTICR3_EXTI9_PA              0x0000U                           /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB              0x0010U                           /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC              0x0020U                           /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD              0x0030U                           /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE              0x0040U                           /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH              0x0070U                           /*!<PH[9] pin */

/**
  * @brief   EXTI10 configuration  
  */
#define SYSCFG_EXTICR3_EXTI10_PA             0x0000U                           /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB             0x0100U                           /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC             0x0200U                           /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD             0x0300U                           /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE             0x0400U                           /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH             0x0700U                           /*!<PH[10] pin */

/**
  * @brief   EXTI11 configuration  
  */
#define SYSCFG_EXTICR3_EXTI11_PA             0x0000U                           /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB             0x1000U                           /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC             0x2000U                           /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD             0x3000U                           /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE             0x4000U                           /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH             0x7000U                           /*!<PH[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12_Pos            (0U)                              
#define SYSCFG_EXTICR4_EXTI12_Msk            (0xFU << SYSCFG_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR4_EXTI12                SYSCFG_EXTICR4_EXTI12_Msk         /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13_Pos            (4U)                              
#define SYSCFG_EXTICR4_EXTI13_Msk            (0xFU << SYSCFG_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR4_EXTI13                SYSCFG_EXTICR4_EXTI13_Msk         /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14_Pos            (8U)                              
#define SYSCFG_EXTICR4_EXTI14_Msk            (0xFU << SYSCFG_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR4_EXTI14                SYSCFG_EXTICR4_EXTI14_Msk         /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15_Pos            (12U)                             
#define SYSCFG_EXTICR4_EXTI15_Msk            (0xFU << SYSCFG_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR4_EXTI15                SYSCFG_EXTICR4_EXTI15_Msk         /*!<EXTI 15 configuration */

/**
  * @brief   EXTI12 configuration  
  */
#define SYSCFG_EXTICR4_EXTI12_PA             0x0000U                           /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB             0x0001U                           /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC             0x0002U                           /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD             0x0003U                           /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE             0x0004U                           /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH             0x0007U                           /*!<PH[12] pin */

/**
  * @brief   EXTI13 configuration  
  */
#define SYSCFG_EXTICR4_EXTI13_PA             0x0000U                           /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB             0x0010U                           /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC             0x0020U                           /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD             0x0030U                           /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE             0x0040U                           /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH             0x0070U                           /*!<PH[13] pin */

/**
  * @brief   EXTI14 configuration  
  */
#define SYSCFG_EXTICR4_EXTI14_PA             0x0000U                           /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB             0x0100U                           /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC             0x0200U                           /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD             0x0300U                           /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE             0x0400U                           /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH             0x0700U                           /*!<PH[14] pin */

/**
  * @brief   EXTI15 configuration  
  */
#define SYSCFG_EXTICR4_EXTI15_PA             0x0000U                           /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB             0x1000U                           /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC             0x2000U                           /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD             0x3000U                           /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE             0x4000U                           /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH             0x7000U                           /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/
#define SYSCFG_CMPCR_CMP_PD_Pos              (0U)                              
#define SYSCFG_CMPCR_CMP_PD_Msk              (0x1U << SYSCFG_CMPCR_CMP_PD_Pos) /*!< 0x00000001 */
#define SYSCFG_CMPCR_CMP_PD                  SYSCFG_CMPCR_CMP_PD_Msk           /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY_Pos               (8U)                              
#define SYSCFG_CMPCR_READY_Msk               (0x1U << SYSCFG_CMPCR_READY_Pos)  /*!< 0x00000100 */
#define SYSCFG_CMPCR_READY                   SYSCFG_CMPCR_READY_Msk            /*!<Compensation cell power-down */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define TIM_CR1_CEN_Pos           (0U)                                         
#define TIM_CR1_CEN_Msk           (0x1U << TIM_CR1_CEN_Pos)                    /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!<Counter enable        */
#define TIM_CR1_UDIS_Pos          (1U)                                         
#define TIM_CR1_UDIS_Msk          (0x1U << TIM_CR1_UDIS_Pos)                   /*!< 0x00000002 */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk                             /*!<Update disable        */
#define TIM_CR1_URS_Pos           (2U)                                         
#define TIM_CR1_URS_Msk           (0x1U << TIM_CR1_URS_Pos)                    /*!< 0x00000004 */
#define TIM_CR1_URS               TIM_CR1_URS_Msk                              /*!<Update request source */
#define TIM_CR1_OPM_Pos           (3U)                                         
#define TIM_CR1_OPM_Msk           (0x1U << TIM_CR1_OPM_Pos)                    /*!< 0x00000008 */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk                              /*!<One pulse mode        */
#define TIM_CR1_DIR_Pos           (4U)                                         
#define TIM_CR1_DIR_Msk           (0x1U << TIM_CR1_DIR_Pos)                    /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                              /*!<Direction             */

#define TIM_CR1_CMS_Pos           (5U)                                         
#define TIM_CR1_CMS_Msk           (0x3U << TIM_CR1_CMS_Pos)                    /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                              /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x1U << TIM_CR1_CMS_Pos)                    /*!< 0x0020 */
#define TIM_CR1_CMS_1             (0x2U << TIM_CR1_CMS_Pos)                    /*!< 0x0040 */

#define TIM_CR1_ARPE_Pos          (7U)                                         
#define TIM_CR1_ARPE_Msk          (0x1U << TIM_CR1_ARPE_Pos)                   /*!< 0x00000080 */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk                             /*!<Auto-reload preload enable     */

#define TIM_CR1_CKD_Pos           (8U)                                         
#define TIM_CR1_CKD_Msk           (0x3U << TIM_CR1_CKD_Pos)                    /*!< 0x00000300 */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk                              /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0             (0x1U << TIM_CR1_CKD_Pos)                    /*!< 0x0100 */
#define TIM_CR1_CKD_1             (0x2U << TIM_CR1_CKD_Pos)                    /*!< 0x0200 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define TIM_CR2_CCPC_Pos          (0U)                                         
#define TIM_CR2_CCPC_Msk          (0x1U << TIM_CR2_CCPC_Pos)                   /*!< 0x00000001 */
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk                             /*!<Capture/Compare Preloaded Control        */
#define TIM_CR2_CCUS_Pos          (2U)                                         
#define TIM_CR2_CCUS_Msk          (0x1U << TIM_CR2_CCUS_Pos)                   /*!< 0x00000004 */
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk                             /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos          (3U)                                         
#define TIM_CR2_CCDS_Msk          (0x1U << TIM_CR2_CCDS_Pos)                   /*!< 0x00000008 */
#define TIM_CR2_CCDS              TIM_CR2_CCDS_Msk                             /*!<Capture/Compare DMA Selection            */

#define TIM_CR2_MMS_Pos           (4U)                                         
#define TIM_CR2_MMS_Msk           (0x7U << TIM_CR2_MMS_Pos)                    /*!< 0x00000070 */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk                              /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0             (0x1U << TIM_CR2_MMS_Pos)                    /*!< 0x0010 */
#define TIM_CR2_MMS_1             (0x2U << TIM_CR2_MMS_Pos)                    /*!< 0x0020 */
#define TIM_CR2_MMS_2             (0x4U << TIM_CR2_MMS_Pos)                    /*!< 0x0040 */

#define TIM_CR2_TI1S_Pos          (7U)                                         
#define TIM_CR2_TI1S_Msk          (0x1U << TIM_CR2_TI1S_Pos)                   /*!< 0x00000080 */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk                             /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos          (8U)                                         
#define TIM_CR2_OIS1_Msk          (0x1U << TIM_CR2_OIS1_Pos)                   /*!< 0x00000100 */
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk                             /*!<Output Idle state 1 (OC1 output)  */
#define TIM_CR2_OIS1N_Pos         (9U)                                         
#define TIM_CR2_OIS1N_Msk         (0x1U << TIM_CR2_OIS1N_Pos)                  /*!< 0x00000200 */
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk                            /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos          (10U)                                        
#define TIM_CR2_OIS2_Msk          (0x1U << TIM_CR2_OIS2_Pos)                   /*!< 0x00000400 */
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk                             /*!<Output Idle state 2 (OC2 output)  */
#define TIM_CR2_OIS2N_Pos         (11U)                                        
#define TIM_CR2_OIS2N_Msk         (0x1U << TIM_CR2_OIS2N_Pos)                  /*!< 0x00000800 */
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk                            /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos          (12U)                                        
#define TIM_CR2_OIS3_Msk          (0x1U << TIM_CR2_OIS3_Pos)                   /*!< 0x00001000 */
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk                             /*!<Output Idle state 3 (OC3 output)  */
#define TIM_CR2_OIS3N_Pos         (13U)                                        
#define TIM_CR2_OIS3N_Msk         (0x1U << TIM_CR2_OIS3N_Pos)                  /*!< 0x00002000 */
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk                            /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos          (14U)                                        
#define TIM_CR2_OIS4_Msk          (0x1U << TIM_CR2_OIS4_Pos)                   /*!< 0x00004000 */
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk                             /*!<Output Idle state 4 (OC4 output)  */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define TIM_SMCR_SMS_Pos          (0U)                                         
#define TIM_SMCR_SMS_Msk          (0x7U << TIM_SMCR_SMS_Pos)                   /*!< 0x00000007 */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk                             /*!<SMS[2:0] bits (Slave mode selection)    */
#define TIM_SMCR_SMS_0            (0x1U << TIM_SMCR_SMS_Pos)                   /*!< 0x0001 */
#define TIM_SMCR_SMS_1            (0x2U << TIM_SMCR_SMS_Pos)                   /*!< 0x0002 */
#define TIM_SMCR_SMS_2            (0x4U << TIM_SMCR_SMS_Pos)                   /*!< 0x0004 */

#define TIM_SMCR_TS_Pos           (4U)                                         
#define TIM_SMCR_TS_Msk           (0x7U << TIM_SMCR_TS_Pos)                    /*!< 0x00000070 */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk                              /*!<TS[2:0] bits (Trigger selection)        */
#define TIM_SMCR_TS_0             (0x1U << TIM_SMCR_TS_Pos)                    /*!< 0x0010 */
#define TIM_SMCR_TS_1             (0x2U << TIM_SMCR_TS_Pos)                    /*!< 0x0020 */
#define TIM_SMCR_TS_2             (0x4U << TIM_SMCR_TS_Pos)                    /*!< 0x0040 */

#define TIM_SMCR_MSM_Pos          (7U)                                         
#define TIM_SMCR_MSM_Msk          (0x1U << TIM_SMCR_MSM_Pos)                   /*!< 0x00000080 */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk                             /*!<Master/slave mode                       */

#define TIM_SMCR_ETF_Pos          (8U)                                         
#define TIM_SMCR_ETF_Msk          (0xFU << TIM_SMCR_ETF_Pos)                   /*!< 0x00000F00 */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk                             /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0            (0x1U << TIM_SMCR_ETF_Pos)                   /*!< 0x0100 */
#define TIM_SMCR_ETF_1            (0x2U << TIM_SMCR_ETF_Pos)                   /*!< 0x0200 */
#define TIM_SMCR_ETF_2            (0x4U << TIM_SMCR_ETF_Pos)                   /*!< 0x0400 */
#define TIM_SMCR_ETF_3            (0x8U << TIM_SMCR_ETF_Pos)                   /*!< 0x0800 */

#define TIM_SMCR_ETPS_Pos         (12U)                                        
#define TIM_SMCR_ETPS_Msk         (0x3U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00003000 */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk                            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0           (0x1U << TIM_SMCR_ETPS_Pos)                  /*!< 0x1000 */
#define TIM_SMCR_ETPS_1           (0x2U << TIM_SMCR_ETPS_Pos)                  /*!< 0x2000 */

#define TIM_SMCR_ECE_Pos          (14U)                                        
#define TIM_SMCR_ECE_Msk          (0x1U << TIM_SMCR_ECE_Pos)                   /*!< 0x00004000 */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk                             /*!<External clock enable     */
#define TIM_SMCR_ETP_Pos          (15U)                                        
#define TIM_SMCR_ETP_Msk          (0x1U << TIM_SMCR_ETP_Pos)                   /*!< 0x00008000 */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk                             /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define TIM_DIER_UIE_Pos          (0U)                                         
#define TIM_DIER_UIE_Msk          (0x1U << TIM_DIER_UIE_Pos)                   /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk                             /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos        (1U)                                         
#define TIM_DIER_CC1IE_Msk        (0x1U << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                           /*!<Capture/Compare 1 interrupt enable   */
#define TIM_DIER_CC2IE_Pos        (2U)                                         
#define TIM_DIER_CC2IE_Msk        (0x1U << TIM_DIER_CC2IE_Pos)                 /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk                           /*!<Capture/Compare 2 interrupt enable   */
#define TIM_DIER_CC3IE_Pos        (3U)                                         
#define TIM_DIER_CC3IE_Msk        (0x1U << TIM_DIER_CC3IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk                           /*!<Capture/Compare 3 interrupt enable   */
#define TIM_DIER_CC4IE_Pos        (4U)                                         
#define TIM_DIER_CC4IE_Msk        (0x1U << TIM_DIER_CC4IE_Pos)                 /*!< 0x00000010 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk                           /*!<Capture/Compare 4 interrupt enable   */
#define TIM_DIER_COMIE_Pos        (5U)                                         
#define TIM_DIER_COMIE_Msk        (0x1U << TIM_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                           /*!<COM interrupt enable                 */
#define TIM_DIER_TIE_Pos          (6U)                                         
#define TIM_DIER_TIE_Msk          (0x1U << TIM_DIER_TIE_Pos)                   /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                             /*!<Trigger interrupt enable             */
#define TIM_DIER_BIE_Pos          (7U)                                         
#define TIM_DIER_BIE_Msk          (0x1U << TIM_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk                             /*!<Break interrupt enable               */
#define TIM_DIER_UDE_Pos          (8U)                                         
#define TIM_DIER_UDE_Msk          (0x1U << TIM_DIER_UDE_Pos)                   /*!< 0x00000100 */
#define TIM_DIER_UDE              TIM_DIER_UDE_Msk                             /*!<Update DMA request enable            */
#define TIM_DIER_CC1DE_Pos        (9U)                                         
#define TIM_DIER_CC1DE_Msk        (0x1U << TIM_DIER_CC1DE_Pos)                 /*!< 0x00000200 */
#define TIM_DIER_CC1DE            TIM_DIER_CC1DE_Msk                           /*!<Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE_Pos        (10U)                                        
#define TIM_DIER_CC2DE_Msk        (0x1U << TIM_DIER_CC2DE_Pos)                 /*!< 0x00000400 */
#define TIM_DIER_CC2DE            TIM_DIER_CC2DE_Msk                           /*!<Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE_Pos        (11U)                                        
#define TIM_DIER_CC3DE_Msk        (0x1U << TIM_DIER_CC3DE_Pos)                 /*!< 0x00000800 */
#define TIM_DIER_CC3DE            TIM_DIER_CC3DE_Msk                           /*!<Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE_Pos        (12U)                                        
#define TIM_DIER_CC4DE_Msk        (0x1U << TIM_DIER_CC4DE_Pos)                 /*!< 0x00001000 */
#define TIM_DIER_CC4DE            TIM_DIER_CC4DE_Msk                           /*!<Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE_Pos        (13U)                                        
#define TIM_DIER_COMDE_Msk        (0x1U << TIM_DIER_COMDE_Pos)                 /*!< 0x00002000 */
#define TIM_DIER_COMDE            TIM_DIER_COMDE_Msk                           /*!<COM DMA request enable               */
#define TIM_DIER_TDE_Pos          (14U)                                        
#define TIM_DIER_TDE_Msk          (0x1U << TIM_DIER_TDE_Pos)                   /*!< 0x00004000 */
#define TIM_DIER_TDE              TIM_DIER_TDE_Msk                             /*!<Trigger DMA request enable           */

/********************  Bit definition for TIM_SR register  ********************/
#define TIM_SR_UIF_Pos            (0U)                                         
#define TIM_SR_UIF_Msk            (0x1U << TIM_SR_UIF_Pos)                     /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                               /*!<Update interrupt Flag              */
#define TIM_SR_CC1IF_Pos          (1U)                                         
#define TIM_SR_CC1IF_Msk          (0x1U << TIM_SR_CC1IF_Pos)                   /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk                             /*!<Capture/Compare 1 interrupt Flag   */
#define TIM_SR_CC2IF_Pos          (2U)                                         
#define TIM_SR_CC2IF_Msk          (0x1U << TIM_SR_CC2IF_Pos)                   /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk                             /*!<Capture/Compare 2 interrupt Flag   */
#define TIM_SR_CC3IF_Pos          (3U)                                         
#define TIM_SR_CC3IF_Msk          (0x1U << TIM_SR_CC3IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk                             /*!<Capture/Compare 3 interrupt Flag   */
#define TIM_SR_CC4IF_Pos          (4U)                                         
#define TIM_SR_CC4IF_Msk          (0x1U << TIM_SR_CC4IF_Pos)                   /*!< 0x00000010 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk                             /*!<Capture/Compare 4 interrupt Flag   */
#define TIM_SR_COMIF_Pos          (5U)                                         
#define TIM_SR_COMIF_Msk          (0x1U << TIM_SR_COMIF_Pos)                   /*!< 0x00000020 */
#define TIM_SR_COMIF              TIM_SR_COMIF_Msk                             /*!<COM interrupt Flag                 */
#define TIM_SR_TIF_Pos            (6U)                                         
#define TIM_SR_TIF_Msk            (0x1U << TIM_SR_TIF_Pos)                     /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk                               /*!<Trigger interrupt Flag             */
#define TIM_SR_BIF_Pos            (7U)                                         
#define TIM_SR_BIF_Msk            (0x1U << TIM_SR_BIF_Pos)                     /*!< 0x00000080 */
#define TIM_SR_BIF                TIM_SR_BIF_Msk                               /*!<Break interrupt Flag               */
#define TIM_SR_CC1OF_Pos          (9U)                                         
#define TIM_SR_CC1OF_Msk          (0x1U << TIM_SR_CC1OF_Pos)                   /*!< 0x00000200 */
#define TIM_SR_CC1OF              TIM_SR_CC1OF_Msk                             /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos          (10U)                                        
#define TIM_SR_CC2OF_Msk          (0x1U << TIM_SR_CC2OF_Pos)                   /*!< 0x00000400 */
#define TIM_SR_CC2OF              TIM_SR_CC2OF_Msk                             /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos          (11U)                                        
#define TIM_SR_CC3OF_Msk          (0x1U << TIM_SR_CC3OF_Pos)                   /*!< 0x00000800 */
#define TIM_SR_CC3OF              TIM_SR_CC3OF_Msk                             /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos          (12U)                                        
#define TIM_SR_CC4OF_Msk          (0x1U << TIM_SR_CC4OF_Pos)                   /*!< 0x00001000 */
#define TIM_SR_CC4OF              TIM_SR_CC4OF_Msk                             /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define TIM_EGR_UG_Pos            (0U)                                         
#define TIM_EGR_UG_Msk            (0x1U << TIM_EGR_UG_Pos)                     /*!< 0x00000001 */
#define TIM_EGR_UG                TIM_EGR_UG_Msk                               /*!<Update Generation                         */
#define TIM_EGR_CC1G_Pos          (1U)                                         
#define TIM_EGR_CC1G_Msk          (0x1U << TIM_EGR_CC1G_Pos)                   /*!< 0x00000002 */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk                             /*!<Capture/Compare 1 Generation              */
#define TIM_EGR_CC2G_Pos          (2U)                                         
#define TIM_EGR_CC2G_Msk          (0x1U << TIM_EGR_CC2G_Pos)                   /*!< 0x00000004 */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk                             /*!<Capture/Compare 2 Generation              */
#define TIM_EGR_CC3G_Pos          (3U)                                         
#define TIM_EGR_CC3G_Msk          (0x1U << TIM_EGR_CC3G_Pos)                   /*!< 0x00000008 */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk                             /*!<Capture/Compare 3 Generation              */
#define TIM_EGR_CC4G_Pos          (4U)                                         
#define TIM_EGR_CC4G_Msk          (0x1U << TIM_EGR_CC4G_Pos)                   /*!< 0x00000010 */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk                             /*!<Capture/Compare 4 Generation              */
#define TIM_EGR_COMG_Pos          (5U)                                         
#define TIM_EGR_COMG_Msk          (0x1U << TIM_EGR_COMG_Pos)                   /*!< 0x00000020 */
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk                             /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos            (6U)                                         
#define TIM_EGR_TG_Msk            (0x1U << TIM_EGR_TG_Pos)                     /*!< 0x00000040 */
#define TIM_EGR_TG                TIM_EGR_TG_Msk                               /*!<Trigger Generation                        */
#define TIM_EGR_BG_Pos            (7U)                                         
#define TIM_EGR_BG_Msk            (0x1U << TIM_EGR_BG_Pos)                     /*!< 0x00000080 */
#define TIM_EGR_BG                TIM_EGR_BG_Msk                               /*!<Break Generation                          */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define TIM_CCMR1_CC1S_Pos        (0U)                                         
#define TIM_CCMR1_CC1S_Msk        (0x3U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0          (0x1U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x0001 */
#define TIM_CCMR1_CC1S_1          (0x2U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x0002 */

#define TIM_CCMR1_OC1FE_Pos       (2U)                                         
#define TIM_CCMR1_OC1FE_Msk       (0x1U << TIM_CCMR1_OC1FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk                          /*!<Output Compare 1 Fast enable                 */
#define TIM_CCMR1_OC1PE_Pos       (3U)                                         
#define TIM_CCMR1_OC1PE_Msk       (0x1U << TIM_CCMR1_OC1PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk                          /*!<Output Compare 1 Preload enable              */

#define TIM_CCMR1_OC1M_Pos        (4U)                                         
#define TIM_CCMR1_OC1M_Msk        (0x7U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                           /*!<OC1M[2:0] bits (Output Compare 1 Mode)       */
#define TIM_CCMR1_OC1M_0          (0x1U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x0010 */
#define TIM_CCMR1_OC1M_1          (0x2U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x0020 */
#define TIM_CCMR1_OC1M_2          (0x4U << TIM_CCMR1_OC1M_Pos)                 /*!< 0x0040 */

#define TIM_CCMR1_OC1CE_Pos       (7U)                                         
#define TIM_CCMR1_OC1CE_Msk       (0x1U << TIM_CCMR1_OC1CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE           TIM_CCMR1_OC1CE_Msk                          /*!<Output Compare 1Clear Enable                 */

#define TIM_CCMR1_CC2S_Pos        (8U)                                         
#define TIM_CCMR1_CC2S_Msk        (0x3U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk                           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0          (0x1U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x0100 */
#define TIM_CCMR1_CC2S_1          (0x2U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x0200 */

#define TIM_CCMR1_OC2FE_Pos       (10U)                                        
#define TIM_CCMR1_OC2FE_Msk       (0x1U << TIM_CCMR1_OC2FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk                          /*!<Output Compare 2 Fast enable                 */
#define TIM_CCMR1_OC2PE_Pos       (11U)                                        
#define TIM_CCMR1_OC2PE_Msk       (0x1U << TIM_CCMR1_OC2PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk                          /*!<Output Compare 2 Preload enable              */

#define TIM_CCMR1_OC2M_Pos        (12U)                                        
#define TIM_CCMR1_OC2M_Msk        (0x7U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR1_OC2M            TIM_CCMR1_OC2M_Msk                           /*!<OC2M[2:0] bits (Output Compare 2 Mode)       */
#define TIM_CCMR1_OC2M_0          (0x1U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x1000 */
#define TIM_CCMR1_OC2M_1          (0x2U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x2000 */
#define TIM_CCMR1_OC2M_2          (0x4U << TIM_CCMR1_OC2M_Pos)                 /*!< 0x4000 */

#define TIM_CCMR1_OC2CE_Pos       (15U)                                        
#define TIM_CCMR1_OC2CE_Msk       (0x1U << TIM_CCMR1_OC2CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE           TIM_CCMR1_OC2CE_Msk                          /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos      (2U)                                         
#define TIM_CCMR1_IC1PSC_Msk      (0x3U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC          TIM_CCMR1_IC1PSC_Msk                         /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0        (0x1U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0004 */
#define TIM_CCMR1_IC1PSC_1        (0x2U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0008 */

#define TIM_CCMR1_IC1F_Pos        (4U)                                         
#define TIM_CCMR1_IC1F_Msk        (0xFU << TIM_CCMR1_IC1F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F            TIM_CCMR1_IC1F_Msk                           /*!<IC1F[3:0] bits (Input Capture 1 Filter)      */
#define TIM_CCMR1_IC1F_0          (0x1U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0010 */
#define TIM_CCMR1_IC1F_1          (0x2U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0020 */
#define TIM_CCMR1_IC1F_2          (0x4U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0040 */
#define TIM_CCMR1_IC1F_3          (0x8U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x0080 */

#define TIM_CCMR1_IC2PSC_Pos      (10U)                                        
#define TIM_CCMR1_IC2PSC_Msk      (0x3U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC          TIM_CCMR1_IC2PSC_Msk                         /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)  */
#define TIM_CCMR1_IC2PSC_0        (0x1U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x0400 */
#define TIM_CCMR1_IC2PSC_1        (0x2U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x0800 */

#define TIM_CCMR1_IC2F_Pos        (12U)                                        
#define TIM_CCMR1_IC2F_Msk        (0xFU << TIM_CCMR1_IC2F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F            TIM_CCMR1_IC2F_Msk                           /*!<IC2F[3:0] bits (Input Capture 2 Filter)       */
#define TIM_CCMR1_IC2F_0          (0x1U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x1000 */
#define TIM_CCMR1_IC2F_1          (0x2U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x2000 */
#define TIM_CCMR1_IC2F_2          (0x4U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x4000 */
#define TIM_CCMR1_IC2F_3          (0x8U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x8000 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define TIM_CCMR2_CC3S_Pos        (0U)                                         
#define TIM_CCMR2_CC3S_Msk        (0x3U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk                           /*!<CC3S[1:0] bits (Capture/Compare 3 Selection)  */
#define TIM_CCMR2_CC3S_0          (0x1U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x0001 */
#define TIM_CCMR2_CC3S_1          (0x2U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x0002 */

#define TIM_CCMR2_OC3FE_Pos       (2U)                                         
#define TIM_CCMR2_OC3FE_Msk       (0x1U << TIM_CCMR2_OC3FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk                          /*!<Output Compare 3 Fast enable           */
#define TIM_CCMR2_OC3PE_Pos       (3U)                                         
#define TIM_CCMR2_OC3PE_Msk       (0x1U << TIM_CCMR2_OC3PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk                          /*!<Output Compare 3 Preload enable        */

#define TIM_CCMR2_OC3M_Pos        (4U)                                         
#define TIM_CCMR2_OC3M_Msk        (0x7U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x00000070 */
#define TIM_CCMR2_OC3M            TIM_CCMR2_OC3M_Msk                           /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0          (0x1U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x0010 */
#define TIM_CCMR2_OC3M_1          (0x2U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x0020 */
#define TIM_CCMR2_OC3M_2          (0x4U << TIM_CCMR2_OC3M_Pos)                 /*!< 0x0040 */

#define TIM_CCMR2_OC3CE_Pos       (7U)                                         
#define TIM_CCMR2_OC3CE_Msk       (0x1U << TIM_CCMR2_OC3CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE           TIM_CCMR2_OC3CE_Msk                          /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos        (8U)                                         
#define TIM_CCMR2_CC4S_Msk        (0x3U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk                           /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0          (0x1U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x0100 */
#define TIM_CCMR2_CC4S_1          (0x2U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x0200 */

#define TIM_CCMR2_OC4FE_Pos       (10U)                                        
#define TIM_CCMR2_OC4FE_Msk       (0x1U << TIM_CCMR2_OC4FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk                          /*!<Output Compare 4 Fast enable    */
#define TIM_CCMR2_OC4PE_Pos       (11U)                                        
#define TIM_CCMR2_OC4PE_Msk       (0x1U << TIM_CCMR2_OC4PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk                          /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos        (12U)                                        
#define TIM_CCMR2_OC4M_Msk        (0x7U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x00007000 */
#define TIM_CCMR2_OC4M            TIM_CCMR2_OC4M_Msk                           /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0          (0x1U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x1000 */
#define TIM_CCMR2_OC4M_1          (0x2U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x2000 */
#define TIM_CCMR2_OC4M_2          (0x4U << TIM_CCMR2_OC4M_Pos)                 /*!< 0x4000 */

#define TIM_CCMR2_OC4CE_Pos       (15U)                                        
#define TIM_CCMR2_OC4CE_Msk       (0x1U << TIM_CCMR2_OC4CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE           TIM_CCMR2_OC4CE_Msk                          /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC_Pos      (2U)                                         
#define TIM_CCMR2_IC3PSC_Msk      (0x3U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC          TIM_CCMR2_IC3PSC_Msk                         /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0        (0x1U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0004 */
#define TIM_CCMR2_IC3PSC_1        (0x2U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0008 */

#define TIM_CCMR2_IC3F_Pos        (4U)                                         
#define TIM_CCMR2_IC3F_Msk        (0xFU << TIM_CCMR2_IC3F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F            TIM_CCMR2_IC3F_Msk                           /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0          (0x1U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0010 */
#define TIM_CCMR2_IC3F_1          (0x2U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0020 */
#define TIM_CCMR2_IC3F_2          (0x4U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0040 */
#define TIM_CCMR2_IC3F_3          (0x8U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x0080 */

#define TIM_CCMR2_IC4PSC_Pos      (10U)                                        
#define TIM_CCMR2_IC4PSC_Msk      (0x3U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC          TIM_CCMR2_IC4PSC_Msk                         /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0        (0x1U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x0400 */
#define TIM_CCMR2_IC4PSC_1        (0x2U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x0800 */

#define TIM_CCMR2_IC4F_Pos        (12U)                                        
#define TIM_CCMR2_IC4F_Msk        (0xFU << TIM_CCMR2_IC4F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F            TIM_CCMR2_IC4F_Msk                           /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0          (0x1U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x1000 */
#define TIM_CCMR2_IC4F_1          (0x2U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x2000 */
#define TIM_CCMR2_IC4F_2          (0x4U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x4000 */
#define TIM_CCMR2_IC4F_3          (0x8U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x8000 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define TIM_CCER_CC1E_Pos         (0U)                                         
#define TIM_CCER_CC1E_Msk         (0x1U << TIM_CCER_CC1E_Pos)                  /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                            /*!<Capture/Compare 1 output enable                 */
#define TIM_CCER_CC1P_Pos         (1U)                                         
#define TIM_CCER_CC1P_Msk         (0x1U << TIM_CCER_CC1P_Pos)                  /*!< 0x00000002 */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk                            /*!<Capture/Compare 1 output Polarity               */
#define TIM_CCER_CC1NE_Pos        (2U)                                         
#define TIM_CCER_CC1NE_Msk        (0x1U << TIM_CCER_CC1NE_Pos)                 /*!< 0x00000004 */
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable   */
#define TIM_CCER_CC1NP_Pos        (3U)                                         
#define TIM_CCER_CC1NP_Msk        (0x1U << TIM_CCER_CC1NP_Pos)                 /*!< 0x00000008 */
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk                           /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos         (4U)                                         
#define TIM_CCER_CC2E_Msk         (0x1U << TIM_CCER_CC2E_Pos)                  /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                            /*!<Capture/Compare 2 output enable                 */
#define TIM_CCER_CC2P_Pos         (5U)                                         
#define TIM_CCER_CC2P_Msk         (0x1U << TIM_CCER_CC2P_Pos)                  /*!< 0x00000020 */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk                            /*!<Capture/Compare 2 output Polarity               */
#define TIM_CCER_CC2NE_Pos        (6U)                                         
#define TIM_CCER_CC2NE_Msk        (0x1U << TIM_CCER_CC2NE_Pos)                 /*!< 0x00000040 */
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable   */
#define TIM_CCER_CC2NP_Pos        (7U)                                         
#define TIM_CCER_CC2NP_Msk        (0x1U << TIM_CCER_CC2NP_Pos)                 /*!< 0x00000080 */
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk                           /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos         (8U)                                         
#define TIM_CCER_CC3E_Msk         (0x1U << TIM_CCER_CC3E_Pos)                  /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                            /*!<Capture/Compare 3 output enable                 */
#define TIM_CCER_CC3P_Pos         (9U)                                         
#define TIM_CCER_CC3P_Msk         (0x1U << TIM_CCER_CC3P_Pos)                  /*!< 0x00000200 */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk                            /*!<Capture/Compare 3 output Polarity               */
#define TIM_CCER_CC3NE_Pos        (10U)                                        
#define TIM_CCER_CC3NE_Msk        (0x1U << TIM_CCER_CC3NE_Pos)                 /*!< 0x00000400 */
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable   */
#define TIM_CCER_CC3NP_Pos        (11U)                                        
#define TIM_CCER_CC3NP_Msk        (0x1U << TIM_CCER_CC3NP_Pos)                 /*!< 0x00000800 */
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk                           /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos         (12U)                                        
#define TIM_CCER_CC4E_Msk         (0x1U << TIM_CCER_CC4E_Pos)                  /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                            /*!<Capture/Compare 4 output enable                 */
#define TIM_CCER_CC4P_Pos         (13U)                                        
#define TIM_CCER_CC4P_Msk         (0x1U << TIM_CCER_CC4P_Pos)                  /*!< 0x00002000 */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk                            /*!<Capture/Compare 4 output Polarity               */
#define TIM_CCER_CC4NP_Pos        (15U)                                        
#define TIM_CCER_CC4NP_Msk        (0x1U << TIM_CCER_CC4NP_Pos)                 /*!< 0x00008000 */
#define TIM_CCER_CC4NP            TIM_CCER_CC4NP_Msk                           /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define TIM_CNT_CNT_Pos           (0U)                                             
#define TIM_CNT_CNT_Msk           (0xFFFFFFFFU << TIM_CNT_CNT_Pos)                 /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT               TIM_CNT_CNT_Msk                                  /*!<Counter Value            */

/*******************  Bit definition for TIM_PSC register  ********************/
#define TIM_PSC_PSC_Pos           (0U)                                         
#define TIM_PSC_PSC_Msk           (0xFFFFU << TIM_PSC_PSC_Pos)                 /*!< 0x0000FFFF */
#define TIM_PSC_PSC               TIM_PSC_PSC_Msk                              /*!<Prescaler Value          */

/*******************  Bit definition for TIM_ARR register  ********************/
#define TIM_ARR_ARR_Pos           (0U)                                         
#define TIM_ARR_ARR_Msk           (0xFFFFFFFFU << TIM_ARR_ARR_Pos)             /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR               TIM_ARR_ARR_Msk                              /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define TIM_RCR_REP_Pos           (0U)                                         
#define TIM_RCR_REP_Msk           (0xFFU << TIM_RCR_REP_Pos)                   /*!< 0x000000FF */
#define TIM_RCR_REP               TIM_RCR_REP_Msk                              /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define TIM_CCR1_CCR1_Pos         (0U)                                         
#define TIM_CCR1_CCR1_Msk         (0xFFFFU << TIM_CCR1_CCR1_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1             TIM_CCR1_CCR1_Msk                            /*!<Capture/Compare 1 Value  */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define TIM_CCR2_CCR2_Pos         (0U)                                         
#define TIM_CCR2_CCR2_Msk         (0xFFFFU << TIM_CCR2_CCR2_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2             TIM_CCR2_CCR2_Msk                            /*!<Capture/Compare 2 Value  */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define TIM_CCR3_CCR3_Pos         (0U)                                         
#define TIM_CCR3_CCR3_Msk         (0xFFFFU << TIM_CCR3_CCR3_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3             TIM_CCR3_CCR3_Msk                            /*!<Capture/Compare 3 Value  */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define TIM_CCR4_CCR4_Pos         (0U)                                         
#define TIM_CCR4_CCR4_Msk         (0xFFFFU << TIM_CCR4_CCR4_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4             TIM_CCR4_CCR4_Msk                            /*!<Capture/Compare 4 Value  */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define TIM_BDTR_DTG_Pos          (0U)                                         
#define TIM_BDTR_DTG_Msk          (0xFFU << TIM_BDTR_DTG_Pos)                  /*!< 0x000000FF */
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk                             /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0            (0x01U << TIM_BDTR_DTG_Pos)                  /*!< 0x0001 */
#define TIM_BDTR_DTG_1            (0x02U << TIM_BDTR_DTG_Pos)                  /*!< 0x0002 */
#define TIM_BDTR_DTG_2            (0x04U << TIM_BDTR_DTG_Pos)                  /*!< 0x0004 */
#define TIM_BDTR_DTG_3            (0x08U << TIM_BDTR_DTG_Pos)                  /*!< 0x0008 */
#define TIM_BDTR_DTG_4            (0x10U << TIM_BDTR_DTG_Pos)                  /*!< 0x0010 */
#define TIM_BDTR_DTG_5            (0x20U << TIM_BDTR_DTG_Pos)                  /*!< 0x0020 */
#define TIM_BDTR_DTG_6            (0x40U << TIM_BDTR_DTG_Pos)                  /*!< 0x0040 */
#define TIM_BDTR_DTG_7            (0x80U << TIM_BDTR_DTG_Pos)                  /*!< 0x0080 */

#define TIM_BDTR_LOCK_Pos         (8U)                                         
#define TIM_BDTR_LOCK_Msk         (0x3U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000300 */
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk                            /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0           (0x1U << TIM_BDTR_LOCK_Pos)                  /*!< 0x0100 */
#define TIM_BDTR_LOCK_1           (0x2U << TIM_BDTR_LOCK_Pos)                  /*!< 0x0200 */

#define TIM_BDTR_OSSI_Pos         (10U)                                        
#define TIM_BDTR_OSSI_Msk         (0x1U << TIM_BDTR_OSSI_Pos)                  /*!< 0x00000400 */
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk                            /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos         (11U)                                        
#define TIM_BDTR_OSSR_Msk         (0x1U << TIM_BDTR_OSSR_Pos)                  /*!< 0x00000800 */
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk                            /*!<Off-State Selection for Run mode  */
#define TIM_BDTR_BKE_Pos          (12U)                                        
#define TIM_BDTR_BKE_Msk          (0x1U << TIM_BDTR_BKE_Pos)                   /*!< 0x00001000 */
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk                             /*!<Break enable                      */
#define TIM_BDTR_BKP_Pos          (13U)                                        
#define TIM_BDTR_BKP_Msk          (0x1U << TIM_BDTR_BKP_Pos)                   /*!< 0x00002000 */
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk                             /*!<Break Polarity                    */
#define TIM_BDTR_AOE_Pos          (14U)                                        
#define TIM_BDTR_AOE_Msk          (0x1U << TIM_BDTR_AOE_Pos)                   /*!< 0x00004000 */
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk                             /*!<Automatic Output enable           */
#define TIM_BDTR_MOE_Pos          (15U)                                        
#define TIM_BDTR_MOE_Msk          (0x1U << TIM_BDTR_MOE_Pos)                   /*!< 0x00008000 */
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk                             /*!<Main Output enable                */

/*******************  Bit definition for TIM_DCR register  ********************/
#define TIM_DCR_DBA_Pos           (0U)                                         
#define TIM_DCR_DBA_Msk           (0x1FU << TIM_DCR_DBA_Pos)                   /*!< 0x0000001F */
#define TIM_DCR_DBA               TIM_DCR_DBA_Msk                              /*!<DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBA_0             (0x01U << TIM_DCR_DBA_Pos)                   /*!< 0x0001 */
#define TIM_DCR_DBA_1             (0x02U << TIM_DCR_DBA_Pos)                   /*!< 0x0002 */
#define TIM_DCR_DBA_2             (0x04U << TIM_DCR_DBA_Pos)                   /*!< 0x0004 */
#define TIM_DCR_DBA_3             (0x08U << TIM_DCR_DBA_Pos)                   /*!< 0x0008 */
#define TIM_DCR_DBA_4             (0x10U << TIM_DCR_DBA_Pos)                   /*!< 0x0010 */

#define TIM_DCR_DBL_Pos           (8U)                                         
#define TIM_DCR_DBL_Msk           (0x1FU << TIM_DCR_DBL_Pos)                   /*!< 0x00001F00 */
#define TIM_DCR_DBL               TIM_DCR_DBL_Msk                              /*!<DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCR_DBL_0             (0x01U << TIM_DCR_DBL_Pos)                   /*!< 0x0100 */
#define TIM_DCR_DBL_1             (0x02U << TIM_DCR_DBL_Pos)                   /*!< 0x0200 */
#define TIM_DCR_DBL_2             (0x04U << TIM_DCR_DBL_Pos)                   /*!< 0x0400 */
#define TIM_DCR_DBL_3             (0x08U << TIM_DCR_DBL_Pos)                   /*!< 0x0800 */
#define TIM_DCR_DBL_4             (0x10U << TIM_DCR_DBL_Pos)                   /*!< 0x1000 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define TIM_DMAR_DMAB_Pos         (0U)                                         
#define TIM_DMAR_DMAB_Msk         (0xFFFFU << TIM_DMAR_DMAB_Pos)               /*!< 0x0000FFFF */
#define TIM_DMAR_DMAB             TIM_DMAR_DMAB_Msk                            /*!<DMA register for burst accesses                    */

/*******************  Bit definition for TIM_OR register  *********************/
#define TIM_OR_TI1_RMP_Pos        (0U)                                          
#define TIM_OR_TI1_RMP_Msk        (0x3U << TIM_OR_TI1_RMP_Pos)                 /*!< 0x00000003 */
#define TIM_OR_TI1_RMP            TIM_OR_TI1_RMP_Msk                           /*!< TI1_RMP[1:0] bits (TIM11 Input Capture 1 remap) */
#define TIM_OR_TI1_RMP_0          (0x1U << TIM_OR_TI1_RMP_Pos)                 /*!< 0x00000001 */
#define TIM_OR_TI1_RMP_1          (0x2U << TIM_OR_TI1_RMP_Pos)                 /*!< 0x00000002 */

#define TIM_OR_TI4_RMP_Pos        (6U)                                         
#define TIM_OR_TI4_RMP_Msk        (0x3U << TIM_OR_TI4_RMP_Pos)                 /*!< 0x000000C0 */
#define TIM_OR_TI4_RMP            TIM_OR_TI4_RMP_Msk                           /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
#define TIM_OR_TI4_RMP_0          (0x1U << TIM_OR_TI4_RMP_Pos)                 /*!< 0x0040 */
#define TIM_OR_TI4_RMP_1          (0x2U << TIM_OR_TI4_RMP_Pos)                 /*!< 0x0080 */
#define TIM_OR_ITR1_RMP_Pos       (10U)                                        
#define TIM_OR_ITR1_RMP_Msk       (0x3U << TIM_OR_ITR1_RMP_Pos)                /*!< 0x00000C00 */
#define TIM_OR_ITR1_RMP           TIM_OR_ITR1_RMP_Msk                          /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
#define TIM_OR_ITR1_RMP_0         (0x1U << TIM_OR_ITR1_RMP_Pos)                /*!< 0x0400 */
#define TIM_OR_ITR1_RMP_1         (0x2U << TIM_OR_ITR1_RMP_Pos)                /*!< 0x0800 */


/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE_Pos               (0U)                                     
#define USART_SR_PE_Msk               (0x1U << USART_SR_PE_Pos)                /*!< 0x00000001 */
#define USART_SR_PE                   USART_SR_PE_Msk                          /*!<Parity Error                 */
#define USART_SR_FE_Pos               (1U)                                     
#define USART_SR_FE_Msk               (0x1U << USART_SR_FE_Pos)                /*!< 0x00000002 */
#define USART_SR_FE                   USART_SR_FE_Msk                          /*!<Framing Error                */
#define USART_SR_NE_Pos               (2U)                                     
#define USART_SR_NE_Msk               (0x1U << USART_SR_NE_Pos)                /*!< 0x00000004 */
#define USART_SR_NE                   USART_SR_NE_Msk                          /*!<Noise Error Flag             */
#define USART_SR_ORE_Pos              (3U)                                     
#define USART_SR_ORE_Msk              (0x1U << USART_SR_ORE_Pos)               /*!< 0x00000008 */
#define USART_SR_ORE                  USART_SR_ORE_Msk                         /*!<OverRun Error                */
#define USART_SR_IDLE_Pos             (4U)                                     
#define USART_SR_IDLE_Msk             (0x1U << USART_SR_IDLE_Pos)              /*!< 0x00000010 */
#define USART_SR_IDLE                 USART_SR_IDLE_Msk                        /*!<IDLE line detected           */
#define USART_SR_RXNE_Pos             (5U)                                     
#define USART_SR_RXNE_Msk             (0x1U << USART_SR_RXNE_Pos)              /*!< 0x00000020 */
#define USART_SR_RXNE                 USART_SR_RXNE_Msk                        /*!<Read Data Register Not Empty */
#define USART_SR_TC_Pos               (6U)                                     
#define USART_SR_TC_Msk               (0x1U << USART_SR_TC_Pos)                /*!< 0x00000040 */
#define USART_SR_TC                   USART_SR_TC_Msk                          /*!<Transmission Complete        */
#define USART_SR_TXE_Pos              (7U)                                     
#define USART_SR_TXE_Msk              (0x1U << USART_SR_TXE_Pos)               /*!< 0x00000080 */
#define USART_SR_TXE                  USART_SR_TXE_Msk                         /*!<Transmit Data Register Empty */
#define USART_SR_LBD_Pos              (8U)                                     
#define USART_SR_LBD_Msk              (0x1U << USART_SR_LBD_Pos)               /*!< 0x00000100 */
#define USART_SR_LBD                  USART_SR_LBD_Msk                         /*!<LIN Break Detection Flag     */
#define USART_SR_CTS_Pos              (9U)                                     
#define USART_SR_CTS_Msk              (0x1U << USART_SR_CTS_Pos)               /*!< 0x00000200 */
#define USART_SR_CTS                  USART_SR_CTS_Msk                         /*!<CTS Flag                     */

/*******************  Bit definition for USART_DR register  *******************/
#define USART_DR_DR_Pos               (0U)                                     
#define USART_DR_DR_Msk               (0x1FFU << USART_DR_DR_Pos)              /*!< 0x000001FF */
#define USART_DR_DR                   USART_DR_DR_Msk                          /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction_Pos    (0U)                                     
#define USART_BRR_DIV_Fraction_Msk    (0xFU << USART_BRR_DIV_Fraction_Pos)     /*!< 0x0000000F */
#define USART_BRR_DIV_Fraction        USART_BRR_DIV_Fraction_Msk               /*!<Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa_Pos    (4U)                                     
#define USART_BRR_DIV_Mantissa_Msk    (0xFFFU << USART_BRR_DIV_Mantissa_Pos)   /*!< 0x0000FFF0 */
#define USART_BRR_DIV_Mantissa        USART_BRR_DIV_Mantissa_Msk               /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK_Pos             (0U)                                     
#define USART_CR1_SBK_Msk             (0x1U << USART_CR1_SBK_Pos)              /*!< 0x00000001 */
#define USART_CR1_SBK                 USART_CR1_SBK_Msk                        /*!<Send Break                             */
#define USART_CR1_RWU_Pos             (1U)                                     
#define USART_CR1_RWU_Msk             (0x1U << USART_CR1_RWU_Pos)              /*!< 0x00000002 */
#define USART_CR1_RWU                 USART_CR1_RWU_Msk                        /*!<Receiver wakeup                        */
#define USART_CR1_RE_Pos              (2U)                                     
#define USART_CR1_RE_Msk              (0x1U << USART_CR1_RE_Pos)               /*!< 0x00000004 */
#define USART_CR1_RE                  USART_CR1_RE_Msk                         /*!<Receiver Enable                        */
#define USART_CR1_TE_Pos              (3U)                                     
#define USART_CR1_TE_Msk              (0x1U << USART_CR1_TE_Pos)               /*!< 0x00000008 */
#define USART_CR1_TE                  USART_CR1_TE_Msk                         /*!<Transmitter Enable                     */
#define USART_CR1_IDLEIE_Pos          (4U)                                     
#define USART_CR1_IDLEIE_Msk          (0x1U << USART_CR1_IDLEIE_Pos)           /*!< 0x00000010 */
#define USART_CR1_IDLEIE              USART_CR1_IDLEIE_Msk                     /*!<IDLE Interrupt Enable                  */
#define USART_CR1_RXNEIE_Pos          (5U)                                     
#define USART_CR1_RXNEIE_Msk          (0x1U << USART_CR1_RXNEIE_Pos)           /*!< 0x00000020 */
#define USART_CR1_RXNEIE              USART_CR1_RXNEIE_Msk                     /*!<RXNE Interrupt Enable                  */
#define USART_CR1_TCIE_Pos            (6U)                                     
#define USART_CR1_TCIE_Msk            (0x1U << USART_CR1_TCIE_Pos)             /*!< 0x00000040 */
#define USART_CR1_TCIE                USART_CR1_TCIE_Msk                       /*!<Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_Pos           (7U)                                     
#define USART_CR1_TXEIE_Msk           (0x1U << USART_CR1_TXEIE_Pos)            /*!< 0x00000080 */
#define USART_CR1_TXEIE               USART_CR1_TXEIE_Msk                      /*!<TXE Interrupt Enable                   */
#define USART_CR1_PEIE_Pos            (8U)                                     
#define USART_CR1_PEIE_Msk            (0x1U << USART_CR1_PEIE_Pos)             /*!< 0x00000100 */
#define USART_CR1_PEIE                USART_CR1_PEIE_Msk                       /*!<PE Interrupt Enable                    */
#define USART_CR1_PS_Pos              (9U)                                     
#define USART_CR1_PS_Msk              (0x1U << USART_CR1_PS_Pos)               /*!< 0x00000200 */
#define USART_CR1_PS                  USART_CR1_PS_Msk                         /*!<Parity Selection                       */
#define USART_CR1_PCE_Pos             (10U)                                    
#define USART_CR1_PCE_Msk             (0x1U << USART_CR1_PCE_Pos)              /*!< 0x00000400 */
#define USART_CR1_PCE                 USART_CR1_PCE_Msk                        /*!<Parity Control Enable                  */
#define USART_CR1_WAKE_Pos            (11U)                                    
#define USART_CR1_WAKE_Msk            (0x1U << USART_CR1_WAKE_Pos)             /*!< 0x00000800 */
#define USART_CR1_WAKE                USART_CR1_WAKE_Msk                       /*!<Wakeup method                          */
#define USART_CR1_M_Pos               (12U)                                    
#define USART_CR1_M_Msk               (0x1U << USART_CR1_M_Pos)                /*!< 0x00001000 */
#define USART_CR1_M                   USART_CR1_M_Msk                          /*!<Word length                            */
#define USART_CR1_UE_Pos              (13U)                                    
#define USART_CR1_UE_Msk              (0x1U << USART_CR1_UE_Pos)               /*!< 0x00002000 */
#define USART_CR1_UE                  USART_CR1_UE_Msk                         /*!<USART Enable                           */
#define USART_CR1_OVER8_Pos           (15U)                                    
#define USART_CR1_OVER8_Msk           (0x1U << USART_CR1_OVER8_Pos)            /*!< 0x00008000 */
#define USART_CR1_OVER8               USART_CR1_OVER8_Msk                      /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD_Pos             (0U)                                     
#define USART_CR2_ADD_Msk             (0xFU << USART_CR2_ADD_Pos)              /*!< 0x0000000F */
#define USART_CR2_ADD                 USART_CR2_ADD_Msk                        /*!<Address of the USART node            */
#define USART_CR2_LBDL_Pos            (5U)                                     
#define USART_CR2_LBDL_Msk            (0x1U << USART_CR2_LBDL_Pos)             /*!< 0x00000020 */
#define USART_CR2_LBDL                USART_CR2_LBDL_Msk                       /*!<LIN Break Detection Length           */
#define USART_CR2_LBDIE_Pos           (6U)                                     
#define USART_CR2_LBDIE_Msk           (0x1U << USART_CR2_LBDIE_Pos)            /*!< 0x00000040 */
#define USART_CR2_LBDIE               USART_CR2_LBDIE_Msk                      /*!<LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos            (8U)                                     
#define USART_CR2_LBCL_Msk            (0x1U << USART_CR2_LBCL_Pos)             /*!< 0x00000100 */
#define USART_CR2_LBCL                USART_CR2_LBCL_Msk                       /*!<Last Bit Clock pulse                 */
#define USART_CR2_CPHA_Pos            (9U)                                     
#define USART_CR2_CPHA_Msk            (0x1U << USART_CR2_CPHA_Pos)             /*!< 0x00000200 */
#define USART_CR2_CPHA                USART_CR2_CPHA_Msk                       /*!<Clock Phase                          */
#define USART_CR2_CPOL_Pos            (10U)                                    
#define USART_CR2_CPOL_Msk            (0x1U << USART_CR2_CPOL_Pos)             /*!< 0x00000400 */
#define USART_CR2_CPOL                USART_CR2_CPOL_Msk                       /*!<Clock Polarity                       */
#define USART_CR2_CLKEN_Pos           (11U)                                    
#define USART_CR2_CLKEN_Msk           (0x1U << USART_CR2_CLKEN_Pos)            /*!< 0x00000800 */
#define USART_CR2_CLKEN               USART_CR2_CLKEN_Msk                      /*!<Clock Enable                         */

#define USART_CR2_STOP_Pos            (12U)                                    
#define USART_CR2_STOP_Msk            (0x3U << USART_CR2_STOP_Pos)             /*!< 0x00003000 */
#define USART_CR2_STOP                USART_CR2_STOP_Msk                       /*!<STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0              (0x1U << USART_CR2_STOP_Pos)             /*!< 0x1000 */
#define USART_CR2_STOP_1              (0x2U << USART_CR2_STOP_Pos)             /*!< 0x2000 */

#define USART_CR2_LINEN_Pos           (14U)                                    
#define USART_CR2_LINEN_Msk           (0x1U << USART_CR2_LINEN_Pos)            /*!< 0x00004000 */
#define USART_CR2_LINEN               USART_CR2_LINEN_Msk                      /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos             (0U)                                     
#define USART_CR3_EIE_Msk             (0x1U << USART_CR3_EIE_Pos)              /*!< 0x00000001 */
#define USART_CR3_EIE                 USART_CR3_EIE_Msk                        /*!<Error Interrupt Enable      */
#define USART_CR3_IREN_Pos            (1U)                                     
#define USART_CR3_IREN_Msk            (0x1U << USART_CR3_IREN_Pos)             /*!< 0x00000002 */
#define USART_CR3_IREN                USART_CR3_IREN_Msk                       /*!<IrDA mode Enable            */
#define USART_CR3_IRLP_Pos            (2U)                                     
#define USART_CR3_IRLP_Msk            (0x1U << USART_CR3_IRLP_Pos)             /*!< 0x00000004 */
#define USART_CR3_IRLP                USART_CR3_IRLP_Msk                       /*!<IrDA Low-Power              */
#define USART_CR3_HDSEL_Pos           (3U)                                     
#define USART_CR3_HDSEL_Msk           (0x1U << USART_CR3_HDSEL_Pos)            /*!< 0x00000008 */
#define USART_CR3_HDSEL               USART_CR3_HDSEL_Msk                      /*!<Half-Duplex Selection       */
#define USART_CR3_NACK_Pos            (4U)                                     
#define USART_CR3_NACK_Msk            (0x1U << USART_CR3_NACK_Pos)             /*!< 0x00000010 */
#define USART_CR3_NACK                USART_CR3_NACK_Msk                       /*!<Smartcard NACK enable       */
#define USART_CR3_SCEN_Pos            (5U)                                     
#define USART_CR3_SCEN_Msk            (0x1U << USART_CR3_SCEN_Pos)             /*!< 0x00000020 */
#define USART_CR3_SCEN                USART_CR3_SCEN_Msk                       /*!<Smartcard mode enable       */
#define USART_CR3_DMAR_Pos            (6U)                                     
#define USART_CR3_DMAR_Msk            (0x1U << USART_CR3_DMAR_Pos)             /*!< 0x00000040 */
#define USART_CR3_DMAR                USART_CR3_DMAR_Msk                       /*!<DMA Enable Receiver         */
#define USART_CR3_DMAT_Pos            (7U)                                     
#define USART_CR3_DMAT_Msk            (0x1U << USART_CR3_DMAT_Pos)             /*!< 0x00000080 */
#define USART_CR3_DMAT                USART_CR3_DMAT_Msk                       /*!<DMA Enable Transmitter      */
#define USART_CR3_RTSE_Pos            (8U)                                     
#define USART_CR3_RTSE_Msk            (0x1U << USART_CR3_RTSE_Pos)             /*!< 0x00000100 */
#define USART_CR3_RTSE                USART_CR3_RTSE_Msk                       /*!<RTS Enable                  */
#define USART_CR3_CTSE_Pos            (9U)                                     
#define USART_CR3_CTSE_Msk            (0x1U << USART_CR3_CTSE_Pos)             /*!< 0x00000200 */
#define USART_CR3_CTSE                USART_CR3_CTSE_Msk                       /*!<CTS Enable                  */
#define USART_CR3_CTSIE_Pos           (10U)                                    
#define USART_CR3_CTSIE_Msk           (0x1U << USART_CR3_CTSIE_Pos)            /*!< 0x00000400 */
#define USART_CR3_CTSIE               USART_CR3_CTSIE_Msk                      /*!<CTS Interrupt Enable        */
#define USART_CR3_ONEBIT_Pos          (11U)                                    
#define USART_CR3_ONEBIT_Msk          (0x1U << USART_CR3_ONEBIT_Pos)           /*!< 0x00000800 */
#define USART_CR3_ONEBIT              USART_CR3_ONEBIT_Msk                     /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos            (0U)                                     
#define USART_GTPR_PSC_Msk            (0xFFU << USART_GTPR_PSC_Pos)            /*!< 0x000000FF */
#define USART_GTPR_PSC                USART_GTPR_PSC_Msk                       /*!<PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_PSC_0              (0x01U << USART_GTPR_PSC_Pos)            /*!< 0x0001 */
#define USART_GTPR_PSC_1              (0x02U << USART_GTPR_PSC_Pos)            /*!< 0x0002 */
#define USART_GTPR_PSC_2              (0x04U << USART_GTPR_PSC_Pos)            /*!< 0x0004 */
#define USART_GTPR_PSC_3              (0x08U << USART_GTPR_PSC_Pos)            /*!< 0x0008 */
#define USART_GTPR_PSC_4              (0x10U << USART_GTPR_PSC_Pos)            /*!< 0x0010 */
#define USART_GTPR_PSC_5              (0x20U << USART_GTPR_PSC_Pos)            /*!< 0x0020 */
#define USART_GTPR_PSC_6              (0x40U << USART_GTPR_PSC_Pos)            /*!< 0x0040 */
#define USART_GTPR_PSC_7              (0x80U << USART_GTPR_PSC_Pos)            /*!< 0x0080 */

#define USART_GTPR_GT_Pos             (8U)                                     
#define USART_GTPR_GT_Msk             (0xFFU << USART_GTPR_GT_Pos)             /*!< 0x0000FF00 */
#define USART_GTPR_GT                 USART_GTPR_GT_Msk                        /*!<Guard time value */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define WWDG_CR_T_Pos           (0U)                                           
#define WWDG_CR_T_Msk           (0x7FU << WWDG_CR_T_Pos)                       /*!< 0x0000007F */
#define WWDG_CR_T               WWDG_CR_T_Msk                                  /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CR_T_0             (0x01U << WWDG_CR_T_Pos)                       /*!< 0x01 */
#define WWDG_CR_T_1             (0x02U << WWDG_CR_T_Pos)                       /*!< 0x02 */
#define WWDG_CR_T_2             (0x04U << WWDG_CR_T_Pos)                       /*!< 0x04 */
#define WWDG_CR_T_3             (0x08U << WWDG_CR_T_Pos)                       /*!< 0x08 */
#define WWDG_CR_T_4             (0x10U << WWDG_CR_T_Pos)                       /*!< 0x10 */
#define WWDG_CR_T_5             (0x20U << WWDG_CR_T_Pos)                       /*!< 0x20 */
#define WWDG_CR_T_6             (0x40U << WWDG_CR_T_Pos)                       /*!< 0x40 */
/* Legacy defines */
#define  WWDG_CR_T0                          WWDG_CR_T_0
#define  WWDG_CR_T1                          WWDG_CR_T_1
#define  WWDG_CR_T2                          WWDG_CR_T_2
#define  WWDG_CR_T3                          WWDG_CR_T_3
#define  WWDG_CR_T4                          WWDG_CR_T_4
#define  WWDG_CR_T5                          WWDG_CR_T_5
#define  WWDG_CR_T6                          WWDG_CR_T_6

#define WWDG_CR_WDGA_Pos        (7U)                                           
#define WWDG_CR_WDGA_Msk        (0x1U << WWDG_CR_WDGA_Pos)                     /*!< 0x00000080 */
#define WWDG_CR_WDGA            WWDG_CR_WDGA_Msk                               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define WWDG_CFR_W_Pos          (0U)                                           
#define WWDG_CFR_W_Msk          (0x7FU << WWDG_CFR_W_Pos)                      /*!< 0x0000007F */
#define WWDG_CFR_W              WWDG_CFR_W_Msk                                 /*!<W[6:0] bits (7-bit window value) */
#define WWDG_CFR_W_0            (0x01U << WWDG_CFR_W_Pos)                      /*!< 0x0001 */
#define WWDG_CFR_W_1            (0x02U << WWDG_CFR_W_Pos)                      /*!< 0x0002 */
#define WWDG_CFR_W_2            (0x04U << WWDG_CFR_W_Pos)                      /*!< 0x0004 */
#define WWDG_CFR_W_3            (0x08U << WWDG_CFR_W_Pos)                      /*!< 0x0008 */
#define WWDG_CFR_W_4            (0x10U << WWDG_CFR_W_Pos)                      /*!< 0x0010 */
#define WWDG_CFR_W_5            (0x20U << WWDG_CFR_W_Pos)                      /*!< 0x0020 */
#define WWDG_CFR_W_6            (0x40U << WWDG_CFR_W_Pos)                      /*!< 0x0040 */
/* Legacy defines */
#define  WWDG_CFR_W0                         WWDG_CFR_W_0
#define  WWDG_CFR_W1                         WWDG_CFR_W_1
#define  WWDG_CFR_W2                         WWDG_CFR_W_2
#define  WWDG_CFR_W3                         WWDG_CFR_W_3
#define  WWDG_CFR_W4                         WWDG_CFR_W_4
#define  WWDG_CFR_W5                         WWDG_CFR_W_5
#define  WWDG_CFR_W6                         WWDG_CFR_W_6

#define WWDG_CFR_WDGTB_Pos      (7U)                                           
#define WWDG_CFR_WDGTB_Msk      (0x3U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000180 */
#define WWDG_CFR_WDGTB          WWDG_CFR_WDGTB_Msk                             /*!<WDGTB[1:0] bits (Timer Base) */
#define WWDG_CFR_WDGTB_0        (0x1U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x0080 */
#define WWDG_CFR_WDGTB_1        (0x2U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x0100 */
/* Legacy defines */
#define  WWDG_CFR_WDGTB0                     WWDG_CFR_WDGTB_0
#define  WWDG_CFR_WDGTB1                     WWDG_CFR_WDGTB_1

#define WWDG_CFR_EWI_Pos        (9U)                                           
#define WWDG_CFR_EWI_Msk        (0x1U << WWDG_CFR_EWI_Pos)                     /*!< 0x00000200 */
#define WWDG_CFR_EWI            WWDG_CFR_EWI_Msk                               /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define WWDG_SR_EWIF_Pos        (0U)                                           
#define WWDG_SR_EWIF_Msk        (0x1U << WWDG_SR_EWIF_Pos)                     /*!< 0x00000001 */
#define WWDG_SR_EWIF            WWDG_SR_EWIF_Msk                               /*!<Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                DBG                                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define DBGMCU_IDCODE_DEV_ID_Pos                     (0U)                      
#define DBGMCU_IDCODE_DEV_ID_Msk                     (0xFFFU << DBGMCU_IDCODE_DEV_ID_Pos) /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID                         DBGMCU_IDCODE_DEV_ID_Msk  
#define DBGMCU_IDCODE_REV_ID_Pos                     (16U)                     
#define DBGMCU_IDCODE_REV_ID_Msk                     (0xFFFFU << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID                         DBGMCU_IDCODE_REV_ID_Msk  

/********************  Bit definition for DBGMCU_CR register  *****************/
#define DBGMCU_CR_DBG_SLEEP_Pos                      (0U)                      
#define DBGMCU_CR_DBG_SLEEP_Msk                      (0x1U << DBGMCU_CR_DBG_SLEEP_Pos) /*!< 0x00000001 */
#define DBGMCU_CR_DBG_SLEEP                          DBGMCU_CR_DBG_SLEEP_Msk   
#define DBGMCU_CR_DBG_STOP_Pos                       (1U)                      
#define DBGMCU_CR_DBG_STOP_Msk                       (0x1U << DBGMCU_CR_DBG_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP                           DBGMCU_CR_DBG_STOP_Msk    
#define DBGMCU_CR_DBG_STANDBY_Pos                    (2U)                      
#define DBGMCU_CR_DBG_STANDBY_Msk                    (0x1U << DBGMCU_CR_DBG_STANDBY_Pos) /*!< 0x00000004 */
#define DBGMCU_CR_DBG_STANDBY                        DBGMCU_CR_DBG_STANDBY_Msk 
#define DBGMCU_CR_TRACE_IOEN_Pos                     (5U)                      
#define DBGMCU_CR_TRACE_IOEN_Msk                     (0x1U << DBGMCU_CR_TRACE_IOEN_Pos) /*!< 0x00000020 */
#define DBGMCU_CR_TRACE_IOEN                         DBGMCU_CR_TRACE_IOEN_Msk  

#define DBGMCU_CR_TRACE_MODE_Pos                     (6U)                      
#define DBGMCU_CR_TRACE_MODE_Msk                     (0x3U << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x000000C0 */
#define DBGMCU_CR_TRACE_MODE                         DBGMCU_CR_TRACE_MODE_Msk  
#define DBGMCU_CR_TRACE_MODE_0                       (0x1U << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x00000040 */
#define DBGMCU_CR_TRACE_MODE_1                       (0x2U << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x00000080 */

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos             (0U)                      
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM2_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB1_FZ_DBG_TIM2_STOP                 DBGMCU_APB1_FZ_DBG_TIM2_STOP_Msk 
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos             (1U)                      
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM3_STOP_Pos) /*!< 0x00000002 */
#define DBGMCU_APB1_FZ_DBG_TIM3_STOP                 DBGMCU_APB1_FZ_DBG_TIM3_STOP_Msk 
#define DBGMCU_APB1_FZ_DBG_TIM4_STOP_Pos             (2U)                      
#define DBGMCU_APB1_FZ_DBG_TIM4_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM4_STOP_Pos) /*!< 0x00000004 */
#define DBGMCU_APB1_FZ_DBG_TIM4_STOP                 DBGMCU_APB1_FZ_DBG_TIM4_STOP_Msk 
#define DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos             (3U)                      
#define DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos) /*!< 0x00000008 */
#define DBGMCU_APB1_FZ_DBG_TIM5_STOP                 DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk 
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos              (10U)                     
#define DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk              (0x1U << DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos) /*!< 0x00000400 */
#define DBGMCU_APB1_FZ_DBG_RTC_STOP                  DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk 
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos             (11U)                     
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_APB1_FZ_DBG_WWDG_STOP                 DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk 
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos             (12U)                     
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP                 DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk 
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos    (21U)                     
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk    (0x1U << DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) /*!< 0x00200000 */
#define DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk 
#define DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos    (22U)                     
#define DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk    (0x1U << DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos) /*!< 0x00400000 */
#define DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk 
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos    (23U)                     
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk    (0x1U << DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos) /*!< 0x00800000 */
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk 
/* Old IWDGSTOP bit definition, maintained for legacy purpose */
#define  DBGMCU_APB1_FZ_DBG_IWDEG_STOP           DBGMCU_APB1_FZ_DBG_IWDG_STOP

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos             (0U)                      
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk             (0x1U << DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP                 DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk 
#define DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos             (16U)                     
#define DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk             (0x1U << DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos) /*!< 0x00010000 */
#define DBGMCU_APB2_FZ_DBG_TIM9_STOP                 DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk 
#define DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos            (17U)                     
#define DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk            (0x1U << DBGMCU_APB2_FZ_DBG_TIM10_STOP_Pos) /*!< 0x00020000 */
#define DBGMCU_APB2_FZ_DBG_TIM10_STOP                DBGMCU_APB2_FZ_DBG_TIM10_STOP_Msk 
#define DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos            (18U)                     
#define DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk            (0x1U << DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos) /*!< 0x00040000 */
#define DBGMCU_APB2_FZ_DBG_TIM11_STOP                DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk 

/******************************************************************************/
/*                                                                            */
/*                                       USB_OTG                              */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for USB_OTG_GOTGCTL register  ***********/
#define USB_OTG_GOTGCTL_SRQSCS_Pos               (0U)                          
#define USB_OTG_GOTGCTL_SRQSCS_Msk               (0x1U << USB_OTG_GOTGCTL_SRQSCS_Pos) /*!< 0x00000001 */
#define USB_OTG_GOTGCTL_SRQSCS                   USB_OTG_GOTGCTL_SRQSCS_Msk    /*!< Session request success */
#define USB_OTG_GOTGCTL_SRQ_Pos                  (1U)                          
#define USB_OTG_GOTGCTL_SRQ_Msk                  (0x1U << USB_OTG_GOTGCTL_SRQ_Pos) /*!< 0x00000002 */
#define USB_OTG_GOTGCTL_SRQ                      USB_OTG_GOTGCTL_SRQ_Msk       /*!< Session request */
#define USB_OTG_GOTGCTL_HNGSCS_Pos               (8U)                          
#define USB_OTG_GOTGCTL_HNGSCS_Msk               (0x1U << USB_OTG_GOTGCTL_HNGSCS_Pos) /*!< 0x00000100 */
#define USB_OTG_GOTGCTL_HNGSCS                   USB_OTG_GOTGCTL_HNGSCS_Msk    /*!< Host set HNP enable */
#define USB_OTG_GOTGCTL_HNPRQ_Pos                (9U)                          
#define USB_OTG_GOTGCTL_HNPRQ_Msk                (0x1U << USB_OTG_GOTGCTL_HNPRQ_Pos) /*!< 0x00000200 */
#define USB_OTG_GOTGCTL_HNPRQ                    USB_OTG_GOTGCTL_HNPRQ_Msk     /*!< HNP request */
#define USB_OTG_GOTGCTL_HSHNPEN_Pos              (10U)                         
#define USB_OTG_GOTGCTL_HSHNPEN_Msk              (0x1U << USB_OTG_GOTGCTL_HSHNPEN_Pos) /*!< 0x00000400 */
#define USB_OTG_GOTGCTL_HSHNPEN                  USB_OTG_GOTGCTL_HSHNPEN_Msk   /*!< Host set HNP enable */
#define USB_OTG_GOTGCTL_DHNPEN_Pos               (11U)                         
#define USB_OTG_GOTGCTL_DHNPEN_Msk               (0x1U << USB_OTG_GOTGCTL_DHNPEN_Pos) /*!< 0x00000800 */
#define USB_OTG_GOTGCTL_DHNPEN                   USB_OTG_GOTGCTL_DHNPEN_Msk    /*!< Device HNP enabled */
#define USB_OTG_GOTGCTL_CIDSTS_Pos               (16U)                         
#define USB_OTG_GOTGCTL_CIDSTS_Msk               (0x1U << USB_OTG_GOTGCTL_CIDSTS_Pos) /*!< 0x00010000 */
#define USB_OTG_GOTGCTL_CIDSTS                   USB_OTG_GOTGCTL_CIDSTS_Msk    /*!< Connector ID status */
#define USB_OTG_GOTGCTL_DBCT_Pos                 (17U)                         
#define USB_OTG_GOTGCTL_DBCT_Msk                 (0x1U << USB_OTG_GOTGCTL_DBCT_Pos) /*!< 0x00020000 */
#define USB_OTG_GOTGCTL_DBCT                     USB_OTG_GOTGCTL_DBCT_Msk      /*!< Long/short debounce time */
#define USB_OTG_GOTGCTL_ASVLD_Pos                (18U)                         
#define USB_OTG_GOTGCTL_ASVLD_Msk                (0x1U << USB_OTG_GOTGCTL_ASVLD_Pos) /*!< 0x00040000 */
#define USB_OTG_GOTGCTL_ASVLD                    USB_OTG_GOTGCTL_ASVLD_Msk     /*!< A-session valid  */
#define USB_OTG_GOTGCTL_BSVLD_Pos                (19U)                         
#define USB_OTG_GOTGCTL_BSVLD_Msk                (0x1U << USB_OTG_GOTGCTL_BSVLD_Pos) /*!< 0x00080000 */
#define USB_OTG_GOTGCTL_BSVLD                    USB_OTG_GOTGCTL_BSVLD_Msk     /*!< B-session valid */

/********************  Bit definition forUSB_OTG_HCFG register  ********************/

#define USB_OTG_HCFG_FSLSPCS_Pos                 (0U)                          
#define USB_OTG_HCFG_FSLSPCS_Msk                 (0x3U << USB_OTG_HCFG_FSLSPCS_Pos) /*!< 0x00000003 */
#define USB_OTG_HCFG_FSLSPCS                     USB_OTG_HCFG_FSLSPCS_Msk      /*!< FS/LS PHY clock select  */
#define USB_OTG_HCFG_FSLSPCS_0                   (0x1U << USB_OTG_HCFG_FSLSPCS_Pos) /*!< 0x00000001 */
#define USB_OTG_HCFG_FSLSPCS_1                   (0x2U << USB_OTG_HCFG_FSLSPCS_Pos) /*!< 0x00000002 */
#define USB_OTG_HCFG_FSLSS_Pos                   (2U)                          
#define USB_OTG_HCFG_FSLSS_Msk                   (0x1U << USB_OTG_HCFG_FSLSS_Pos) /*!< 0x00000004 */
#define USB_OTG_HCFG_FSLSS                       USB_OTG_HCFG_FSLSS_Msk        /*!< FS- and LS-only support */

/********************  Bit definition for USB_OTG_DCFG register  ********************/

#define USB_OTG_DCFG_DSPD_Pos                    (0U)                          
#define USB_OTG_DCFG_DSPD_Msk                    (0x3U << USB_OTG_DCFG_DSPD_Pos) /*!< 0x00000003 */
#define USB_OTG_DCFG_DSPD                        USB_OTG_DCFG_DSPD_Msk         /*!< Device speed */
#define USB_OTG_DCFG_DSPD_0                      (0x1U << USB_OTG_DCFG_DSPD_Pos) /*!< 0x00000001 */
#define USB_OTG_DCFG_DSPD_1                      (0x2U << USB_OTG_DCFG_DSPD_Pos) /*!< 0x00000002 */
#define USB_OTG_DCFG_NZLSOHSK_Pos                (2U)                          
#define USB_OTG_DCFG_NZLSOHSK_Msk                (0x1U << USB_OTG_DCFG_NZLSOHSK_Pos) /*!< 0x00000004 */
#define USB_OTG_DCFG_NZLSOHSK                    USB_OTG_DCFG_NZLSOHSK_Msk     /*!< Nonzero-length status OUT handshake */

#define USB_OTG_DCFG_DAD_Pos                     (4U)                          
#define USB_OTG_DCFG_DAD_Msk                     (0x7FU << USB_OTG_DCFG_DAD_Pos) /*!< 0x000007F0 */
#define USB_OTG_DCFG_DAD                         USB_OTG_DCFG_DAD_Msk          /*!< Device address */
#define USB_OTG_DCFG_DAD_0                       (0x01U << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000010 */
#define USB_OTG_DCFG_DAD_1                       (0x02U << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000020 */
#define USB_OTG_DCFG_DAD_2                       (0x04U << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000040 */
#define USB_OTG_DCFG_DAD_3                       (0x08U << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000080 */
#define USB_OTG_DCFG_DAD_4                       (0x10U << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000100 */
#define USB_OTG_DCFG_DAD_5                       (0x20U << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000200 */
#define USB_OTG_DCFG_DAD_6                       (0x40U << USB_OTG_DCFG_DAD_Pos) /*!< 0x00000400 */

#define USB_OTG_DCFG_PFIVL_Pos                   (11U)                         
#define USB_OTG_DCFG_PFIVL_Msk                   (0x3U << USB_OTG_DCFG_PFIVL_Pos) /*!< 0x00001800 */
#define USB_OTG_DCFG_PFIVL                       USB_OTG_DCFG_PFIVL_Msk        /*!< Periodic (micro)frame interval */
#define USB_OTG_DCFG_PFIVL_0                     (0x1U << USB_OTG_DCFG_PFIVL_Pos) /*!< 0x00000800 */
#define USB_OTG_DCFG_PFIVL_1                     (0x2U << USB_OTG_DCFG_PFIVL_Pos) /*!< 0x00001000 */

#define USB_OTG_DCFG_PERSCHIVL_Pos               (24U)                         
#define USB_OTG_DCFG_PERSCHIVL_Msk               (0x3U << USB_OTG_DCFG_PERSCHIVL_Pos) /*!< 0x03000000 */
#define USB_OTG_DCFG_PERSCHIVL                   USB_OTG_DCFG_PERSCHIVL_Msk    /*!< Periodic scheduling interval */
#define USB_OTG_DCFG_PERSCHIVL_0                 (0x1U << USB_OTG_DCFG_PERSCHIVL_Pos) /*!< 0x01000000 */
#define USB_OTG_DCFG_PERSCHIVL_1                 (0x2U << USB_OTG_DCFG_PERSCHIVL_Pos) /*!< 0x02000000 */

/********************  Bit definition for USB_OTG_PCGCR register  ********************/
#define USB_OTG_PCGCR_STPPCLK_Pos                (0U)                          
#define USB_OTG_PCGCR_STPPCLK_Msk                (0x1U << USB_OTG_PCGCR_STPPCLK_Pos) /*!< 0x00000001 */
#define USB_OTG_PCGCR_STPPCLK                    USB_OTG_PCGCR_STPPCLK_Msk     /*!< Stop PHY clock */
#define USB_OTG_PCGCR_GATEHCLK_Pos               (1U)                          
#define USB_OTG_PCGCR_GATEHCLK_Msk               (0x1U << USB_OTG_PCGCR_GATEHCLK_Pos) /*!< 0x00000002 */
#define USB_OTG_PCGCR_GATEHCLK                   USB_OTG_PCGCR_GATEHCLK_Msk    /*!< Gate HCLK */
#define USB_OTG_PCGCR_PHYSUSP_Pos                (4U)                          
#define USB_OTG_PCGCR_PHYSUSP_Msk                (0x1U << USB_OTG_PCGCR_PHYSUSP_Pos) /*!< 0x00000010 */
#define USB_OTG_PCGCR_PHYSUSP                    USB_OTG_PCGCR_PHYSUSP_Msk     /*!< PHY suspended */

/********************  Bit definition for USB_OTG_GOTGINT register  ********************/
#define USB_OTG_GOTGINT_SEDET_Pos                (2U)                          
#define USB_OTG_GOTGINT_SEDET_Msk                (0x1U << USB_OTG_GOTGINT_SEDET_Pos) /*!< 0x00000004 */
#define USB_OTG_GOTGINT_SEDET                    USB_OTG_GOTGINT_SEDET_Msk     /*!< Session end detected                   */
#define USB_OTG_GOTGINT_SRSSCHG_Pos              (8U)                          
#define USB_OTG_GOTGINT_SRSSCHG_Msk              (0x1U << USB_OTG_GOTGINT_SRSSCHG_Pos) /*!< 0x00000100 */
#define USB_OTG_GOTGINT_SRSSCHG                  USB_OTG_GOTGINT_SRSSCHG_Msk   /*!< Session request success status change  */
#define USB_OTG_GOTGINT_HNSSCHG_Pos              (9U)                          
#define USB_OTG_GOTGINT_HNSSCHG_Msk              (0x1U << USB_OTG_GOTGINT_HNSSCHG_Pos) /*!< 0x00000200 */
#define USB_OTG_GOTGINT_HNSSCHG                  USB_OTG_GOTGINT_HNSSCHG_Msk   /*!< Host negotiation success status change */
#define USB_OTG_GOTGINT_HNGDET_Pos               (17U)                         
#define USB_OTG_GOTGINT_HNGDET_Msk               (0x1U << USB_OTG_GOTGINT_HNGDET_Pos) /*!< 0x00020000 */
#define USB_OTG_GOTGINT_HNGDET                   USB_OTG_GOTGINT_HNGDET_Msk    /*!< Host negotiation detected              */
#define USB_OTG_GOTGINT_ADTOCHG_Pos              (18U)                         
#define USB_OTG_GOTGINT_ADTOCHG_Msk              (0x1U << USB_OTG_GOTGINT_ADTOCHG_Pos) /*!< 0x00040000 */
#define USB_OTG_GOTGINT_ADTOCHG                  USB_OTG_GOTGINT_ADTOCHG_Msk   /*!< A-device timeout change                */
#define USB_OTG_GOTGINT_DBCDNE_Pos               (19U)                         
#define USB_OTG_GOTGINT_DBCDNE_Msk               (0x1U << USB_OTG_GOTGINT_DBCDNE_Pos) /*!< 0x00080000 */
#define USB_OTG_GOTGINT_DBCDNE                   USB_OTG_GOTGINT_DBCDNE_Msk    /*!< Debounce done                          */

/********************  Bit definition for USB_OTG_DCTL register  ********************/
#define USB_OTG_DCTL_RWUSIG_Pos                  (0U)                          
#define USB_OTG_DCTL_RWUSIG_Msk                  (0x1U << USB_OTG_DCTL_RWUSIG_Pos) /*!< 0x00000001 */
#define USB_OTG_DCTL_RWUSIG                      USB_OTG_DCTL_RWUSIG_Msk       /*!< Remote wakeup signaling */
#define USB_OTG_DCTL_SDIS_Pos                    (1U)                          
#define USB_OTG_DCTL_SDIS_Msk                    (0x1U << USB_OTG_DCTL_SDIS_Pos) /*!< 0x00000002 */
#define USB_OTG_DCTL_SDIS                        USB_OTG_DCTL_SDIS_Msk         /*!< Soft disconnect         */
#define USB_OTG_DCTL_GINSTS_Pos                  (2U)                          
#define USB_OTG_DCTL_GINSTS_Msk                  (0x1U << USB_OTG_DCTL_GINSTS_Pos) /*!< 0x00000004 */
#define USB_OTG_DCTL_GINSTS                      USB_OTG_DCTL_GINSTS_Msk       /*!< Global IN NAK status    */
#define USB_OTG_DCTL_GONSTS_Pos                  (3U)                          
#define USB_OTG_DCTL_GONSTS_Msk                  (0x1U << USB_OTG_DCTL_GONSTS_Pos) /*!< 0x00000008 */
#define USB_OTG_DCTL_GONSTS                      USB_OTG_DCTL_GONSTS_Msk       /*!< Global OUT NAK status   */

#define USB_OTG_DCTL_TCTL_Pos                    (4U)                          
#define USB_OTG_DCTL_TCTL_Msk                    (0x7U << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000070 */
#define USB_OTG_DCTL_TCTL                        USB_OTG_DCTL_TCTL_Msk         /*!< Test control */
#define USB_OTG_DCTL_TCTL_0                      (0x1U << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000010 */
#define USB_OTG_DCTL_TCTL_1                      (0x2U << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000020 */
#define USB_OTG_DCTL_TCTL_2                      (0x4U << USB_OTG_DCTL_TCTL_Pos) /*!< 0x00000040 */
#define USB_OTG_DCTL_SGINAK_Pos                  (7U)                          
#define USB_OTG_DCTL_SGINAK_Msk                  (0x1U << USB_OTG_DCTL_SGINAK_Pos) /*!< 0x00000080 */
#define USB_OTG_DCTL_SGINAK                      USB_OTG_DCTL_SGINAK_Msk       /*!< Set global IN NAK         */
#define USB_OTG_DCTL_CGINAK_Pos                  (8U)                          
#define USB_OTG_DCTL_CGINAK_Msk                  (0x1U << USB_OTG_DCTL_CGINAK_Pos) /*!< 0x00000100 */
#define USB_OTG_DCTL_CGINAK                      USB_OTG_DCTL_CGINAK_Msk       /*!< Clear global IN NAK       */
#define USB_OTG_DCTL_SGONAK_Pos                  (9U)                          
#define USB_OTG_DCTL_SGONAK_Msk                  (0x1U << USB_OTG_DCTL_SGONAK_Pos) /*!< 0x00000200 */
#define USB_OTG_DCTL_SGONAK                      USB_OTG_DCTL_SGONAK_Msk       /*!< Set global OUT NAK        */
#define USB_OTG_DCTL_CGONAK_Pos                  (10U)                         
#define USB_OTG_DCTL_CGONAK_Msk                  (0x1U << USB_OTG_DCTL_CGONAK_Pos) /*!< 0x00000400 */
#define USB_OTG_DCTL_CGONAK                      USB_OTG_DCTL_CGONAK_Msk       /*!< Clear global OUT NAK      */
#define USB_OTG_DCTL_POPRGDNE_Pos                (11U)                         
#define USB_OTG_DCTL_POPRGDNE_Msk                (0x1U << USB_OTG_DCTL_POPRGDNE_Pos) /*!< 0x00000800 */
#define USB_OTG_DCTL_POPRGDNE                    USB_OTG_DCTL_POPRGDNE_Msk     /*!< Power-on programming done */

/********************  Bit definition for USB_OTG_HFIR register  ********************/
#define USB_OTG_HFIR_FRIVL_Pos                   (0U)                          
#define USB_OTG_HFIR_FRIVL_Msk                   (0xFFFFU << USB_OTG_HFIR_FRIVL_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HFIR_FRIVL                       USB_OTG_HFIR_FRIVL_Msk        /*!< Frame interval */

/********************  Bit definition for USB_OTG_HFNUM register  ********************/
#define USB_OTG_HFNUM_FRNUM_Pos                  (0U)                          
#define USB_OTG_HFNUM_FRNUM_Msk                  (0xFFFFU << USB_OTG_HFNUM_FRNUM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HFNUM_FRNUM                      USB_OTG_HFNUM_FRNUM_Msk       /*!< Frame number         */
#define USB_OTG_HFNUM_FTREM_Pos                  (16U)                         
#define USB_OTG_HFNUM_FTREM_Msk                  (0xFFFFU << USB_OTG_HFNUM_FTREM_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_HFNUM_FTREM                      USB_OTG_HFNUM_FTREM_Msk       /*!< Frame time remaining */

/********************  Bit definition for USB_OTG_DSTS register  ********************/
#define USB_OTG_DSTS_SUSPSTS_Pos                 (0U)                          
#define USB_OTG_DSTS_SUSPSTS_Msk                 (0x1U << USB_OTG_DSTS_SUSPSTS_Pos) /*!< 0x00000001 */
#define USB_OTG_DSTS_SUSPSTS                     USB_OTG_DSTS_SUSPSTS_Msk      /*!< Suspend status   */

#define USB_OTG_DSTS_ENUMSPD_Pos                 (1U)                          
#define USB_OTG_DSTS_ENUMSPD_Msk                 (0x3U << USB_OTG_DSTS_ENUMSPD_Pos) /*!< 0x00000006 */
#define USB_OTG_DSTS_ENUMSPD                     USB_OTG_DSTS_ENUMSPD_Msk      /*!< Enumerated speed */
#define USB_OTG_DSTS_ENUMSPD_0                   (0x1U << USB_OTG_DSTS_ENUMSPD_Pos) /*!< 0x00000002 */
#define USB_OTG_DSTS_ENUMSPD_1                   (0x2U << USB_OTG_DSTS_ENUMSPD_Pos) /*!< 0x00000004 */
#define USB_OTG_DSTS_EERR_Pos                    (3U)                          
#define USB_OTG_DSTS_EERR_Msk                    (0x1U << USB_OTG_DSTS_EERR_Pos) /*!< 0x00000008 */
#define USB_OTG_DSTS_EERR                        USB_OTG_DSTS_EERR_Msk         /*!< Erratic error     */
#define USB_OTG_DSTS_FNSOF_Pos                   (8U)                          
#define USB_OTG_DSTS_FNSOF_Msk                   (0x3FFFU << USB_OTG_DSTS_FNSOF_Pos) /*!< 0x003FFF00 */
#define USB_OTG_DSTS_FNSOF                       USB_OTG_DSTS_FNSOF_Msk        /*!< Frame number of the received SOF */

/********************  Bit definition for USB_OTG_GAHBCFG register  ********************/
#define USB_OTG_GAHBCFG_GINT_Pos                 (0U)                          
#define USB_OTG_GAHBCFG_GINT_Msk                 (0x1U << USB_OTG_GAHBCFG_GINT_Pos) /*!< 0x00000001 */
#define USB_OTG_GAHBCFG_GINT                     USB_OTG_GAHBCFG_GINT_Msk      /*!< Global interrupt mask */
#define USB_OTG_GAHBCFG_HBSTLEN_Pos              (1U)                          
#define USB_OTG_GAHBCFG_HBSTLEN_Msk              (0xFU << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< 0x0000001E */
#define USB_OTG_GAHBCFG_HBSTLEN                  USB_OTG_GAHBCFG_HBSTLEN_Msk   /*!< Burst length/type */
#define USB_OTG_GAHBCFG_HBSTLEN_0                (0x0U << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< Single */
#define USB_OTG_GAHBCFG_HBSTLEN_1                (0x1U << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR */
#define USB_OTG_GAHBCFG_HBSTLEN_2                (0x3U << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR4 */
#define USB_OTG_GAHBCFG_HBSTLEN_3                (0x5U << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR8 */
#define USB_OTG_GAHBCFG_HBSTLEN_4                (0x7U << USB_OTG_GAHBCFG_HBSTLEN_Pos) /*!< INCR16 */
#define USB_OTG_GAHBCFG_DMAEN_Pos                (5U)                          
#define USB_OTG_GAHBCFG_DMAEN_Msk                (0x1U << USB_OTG_GAHBCFG_DMAEN_Pos) /*!< 0x00000020 */
#define USB_OTG_GAHBCFG_DMAEN                    USB_OTG_GAHBCFG_DMAEN_Msk     /*!< DMA enable */
#define USB_OTG_GAHBCFG_TXFELVL_Pos              (7U)                          
#define USB_OTG_GAHBCFG_TXFELVL_Msk              (0x1U << USB_OTG_GAHBCFG_TXFELVL_Pos) /*!< 0x00000080 */
#define USB_OTG_GAHBCFG_TXFELVL                  USB_OTG_GAHBCFG_TXFELVL_Msk   /*!< TxFIFO empty level */
#define USB_OTG_GAHBCFG_PTXFELVL_Pos             (8U)                          
#define USB_OTG_GAHBCFG_PTXFELVL_Msk             (0x1U << USB_OTG_GAHBCFG_PTXFELVL_Pos) /*!< 0x00000100 */
#define USB_OTG_GAHBCFG_PTXFELVL                 USB_OTG_GAHBCFG_PTXFELVL_Msk  /*!< Periodic TxFIFO empty level */

/********************  Bit definition for USB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_TOCAL_Pos                (0U)                          
#define USB_OTG_GUSBCFG_TOCAL_Msk                (0x7U << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000007 */
#define USB_OTG_GUSBCFG_TOCAL                    USB_OTG_GUSBCFG_TOCAL_Msk     /*!< FS timeout calibration */
#define USB_OTG_GUSBCFG_TOCAL_0                  (0x1U << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000001 */
#define USB_OTG_GUSBCFG_TOCAL_1                  (0x2U << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000002 */
#define USB_OTG_GUSBCFG_TOCAL_2                  (0x4U << USB_OTG_GUSBCFG_TOCAL_Pos) /*!< 0x00000004 */
#define USB_OTG_GUSBCFG_PHYSEL_Pos               (6U)                          
#define USB_OTG_GUSBCFG_PHYSEL_Msk               (0x1U << USB_OTG_GUSBCFG_PHYSEL_Pos) /*!< 0x00000040 */
#define USB_OTG_GUSBCFG_PHYSEL                   USB_OTG_GUSBCFG_PHYSEL_Msk    /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
#define USB_OTG_GUSBCFG_SRPCAP_Pos               (8U)                          
#define USB_OTG_GUSBCFG_SRPCAP_Msk               (0x1U << USB_OTG_GUSBCFG_SRPCAP_Pos) /*!< 0x00000100 */
#define USB_OTG_GUSBCFG_SRPCAP                   USB_OTG_GUSBCFG_SRPCAP_Msk    /*!< SRP-capable */
#define USB_OTG_GUSBCFG_HNPCAP_Pos               (9U)                          
#define USB_OTG_GUSBCFG_HNPCAP_Msk               (0x1U << USB_OTG_GUSBCFG_HNPCAP_Pos) /*!< 0x00000200 */
#define USB_OTG_GUSBCFG_HNPCAP                   USB_OTG_GUSBCFG_HNPCAP_Msk    /*!< HNP-capable */
#define USB_OTG_GUSBCFG_TRDT_Pos                 (10U)                         
#define USB_OTG_GUSBCFG_TRDT_Msk                 (0xFU << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00003C00 */
#define USB_OTG_GUSBCFG_TRDT                     USB_OTG_GUSBCFG_TRDT_Msk      /*!< USB turnaround time */
#define USB_OTG_GUSBCFG_TRDT_0                   (0x1U << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00000400 */
#define USB_OTG_GUSBCFG_TRDT_1                   (0x2U << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00000800 */
#define USB_OTG_GUSBCFG_TRDT_2                   (0x4U << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00001000 */
#define USB_OTG_GUSBCFG_TRDT_3                   (0x8U << USB_OTG_GUSBCFG_TRDT_Pos) /*!< 0x00002000 */
#define USB_OTG_GUSBCFG_PHYLPCS_Pos              (15U)                         
#define USB_OTG_GUSBCFG_PHYLPCS_Msk              (0x1U << USB_OTG_GUSBCFG_PHYLPCS_Pos) /*!< 0x00008000 */
#define USB_OTG_GUSBCFG_PHYLPCS                  USB_OTG_GUSBCFG_PHYLPCS_Msk   /*!< PHY Low-power clock select */
#define USB_OTG_GUSBCFG_ULPIFSLS_Pos             (17U)                         
#define USB_OTG_GUSBCFG_ULPIFSLS_Msk             (0x1U << USB_OTG_GUSBCFG_ULPIFSLS_Pos) /*!< 0x00020000 */
#define USB_OTG_GUSBCFG_ULPIFSLS                 USB_OTG_GUSBCFG_ULPIFSLS_Msk  /*!< ULPI FS/LS select               */
#define USB_OTG_GUSBCFG_ULPIAR_Pos               (18U)                         
#define USB_OTG_GUSBCFG_ULPIAR_Msk               (0x1U << USB_OTG_GUSBCFG_ULPIAR_Pos) /*!< 0x00040000 */
#define USB_OTG_GUSBCFG_ULPIAR                   USB_OTG_GUSBCFG_ULPIAR_Msk    /*!< ULPI Auto-resume                */
#define USB_OTG_GUSBCFG_ULPICSM_Pos              (19U)                         
#define USB_OTG_GUSBCFG_ULPICSM_Msk              (0x1U << USB_OTG_GUSBCFG_ULPICSM_Pos) /*!< 0x00080000 */
#define USB_OTG_GUSBCFG_ULPICSM                  USB_OTG_GUSBCFG_ULPICSM_Msk   /*!< ULPI Clock SuspendM             */
#define USB_OTG_GUSBCFG_ULPIEVBUSD_Pos           (20U)                         
#define USB_OTG_GUSBCFG_ULPIEVBUSD_Msk           (0x1U << USB_OTG_GUSBCFG_ULPIEVBUSD_Pos) /*!< 0x00100000 */
#define USB_OTG_GUSBCFG_ULPIEVBUSD               USB_OTG_GUSBCFG_ULPIEVBUSD_Msk /*!< ULPI External VBUS Drive        */
#define USB_OTG_GUSBCFG_ULPIEVBUSI_Pos           (21U)                         
#define USB_OTG_GUSBCFG_ULPIEVBUSI_Msk           (0x1U << USB_OTG_GUSBCFG_ULPIEVBUSI_Pos) /*!< 0x00200000 */
#define USB_OTG_GUSBCFG_ULPIEVBUSI               USB_OTG_GUSBCFG_ULPIEVBUSI_Msk /*!< ULPI external VBUS indicator    */
#define USB_OTG_GUSBCFG_TSDPS_Pos                (22U)                         
#define USB_OTG_GUSBCFG_TSDPS_Msk                (0x1U << USB_OTG_GUSBCFG_TSDPS_Pos) /*!< 0x00400000 */
#define USB_OTG_GUSBCFG_TSDPS                    USB_OTG_GUSBCFG_TSDPS_Msk     /*!< TermSel DLine pulsing selection */
#define USB_OTG_GUSBCFG_PCCI_Pos                 (23U)                         
#define USB_OTG_GUSBCFG_PCCI_Msk                 (0x1U << USB_OTG_GUSBCFG_PCCI_Pos) /*!< 0x00800000 */
#define USB_OTG_GUSBCFG_PCCI                     USB_OTG_GUSBCFG_PCCI_Msk      /*!< Indicator complement            */
#define USB_OTG_GUSBCFG_PTCI_Pos                 (24U)                         
#define USB_OTG_GUSBCFG_PTCI_Msk                 (0x1U << USB_OTG_GUSBCFG_PTCI_Pos) /*!< 0x01000000 */
#define USB_OTG_GUSBCFG_PTCI                     USB_OTG_GUSBCFG_PTCI_Msk      /*!< Indicator pass through          */
#define USB_OTG_GUSBCFG_ULPIIPD_Pos              (25U)                         
#define USB_OTG_GUSBCFG_ULPIIPD_Msk              (0x1U << USB_OTG_GUSBCFG_ULPIIPD_Pos) /*!< 0x02000000 */
#define USB_OTG_GUSBCFG_ULPIIPD                  USB_OTG_GUSBCFG_ULPIIPD_Msk   /*!< ULPI interface protect disable  */
#define USB_OTG_GUSBCFG_FHMOD_Pos                (29U)                         
#define USB_OTG_GUSBCFG_FHMOD_Msk                (0x1U << USB_OTG_GUSBCFG_FHMOD_Pos) /*!< 0x20000000 */
#define USB_OTG_GUSBCFG_FHMOD                    USB_OTG_GUSBCFG_FHMOD_Msk     /*!< Forced host mode                */
#define USB_OTG_GUSBCFG_FDMOD_Pos                (30U)                         
#define USB_OTG_GUSBCFG_FDMOD_Msk                (0x1U << USB_OTG_GUSBCFG_FDMOD_Pos) /*!< 0x40000000 */
#define USB_OTG_GUSBCFG_FDMOD                    USB_OTG_GUSBCFG_FDMOD_Msk     /*!< Forced peripheral mode          */
#define USB_OTG_GUSBCFG_CTXPKT_Pos               (31U)                         
#define USB_OTG_GUSBCFG_CTXPKT_Msk               (0x1U << USB_OTG_GUSBCFG_CTXPKT_Pos) /*!< 0x80000000 */
#define USB_OTG_GUSBCFG_CTXPKT                   USB_OTG_GUSBCFG_CTXPKT_Msk    /*!< Corrupt Tx packet               */

/********************  Bit definition for USB_OTG_GRSTCTL register  ********************/
#define USB_OTG_GRSTCTL_CSRST_Pos                (0U)                          
#define USB_OTG_GRSTCTL_CSRST_Msk                (0x1U << USB_OTG_GRSTCTL_CSRST_Pos) /*!< 0x00000001 */
#define USB_OTG_GRSTCTL_CSRST                    USB_OTG_GRSTCTL_CSRST_Msk     /*!< Core soft reset          */
#define USB_OTG_GRSTCTL_HSRST_Pos                (1U)                          
#define USB_OTG_GRSTCTL_HSRST_Msk                (0x1U << USB_OTG_GRSTCTL_HSRST_Pos) /*!< 0x00000002 */
#define USB_OTG_GRSTCTL_HSRST                    USB_OTG_GRSTCTL_HSRST_Msk     /*!< HCLK soft reset          */
#define USB_OTG_GRSTCTL_FCRST_Pos                (2U)                          
#define USB_OTG_GRSTCTL_FCRST_Msk                (0x1U << USB_OTG_GRSTCTL_FCRST_Pos) /*!< 0x00000004 */
#define USB_OTG_GRSTCTL_FCRST                    USB_OTG_GRSTCTL_FCRST_Msk     /*!< Host frame counter reset */
#define USB_OTG_GRSTCTL_RXFFLSH_Pos              (4U)                          
#define USB_OTG_GRSTCTL_RXFFLSH_Msk              (0x1U << USB_OTG_GRSTCTL_RXFFLSH_Pos) /*!< 0x00000010 */
#define USB_OTG_GRSTCTL_RXFFLSH                  USB_OTG_GRSTCTL_RXFFLSH_Msk   /*!< RxFIFO flush             */
#define USB_OTG_GRSTCTL_TXFFLSH_Pos              (5U)                          
#define USB_OTG_GRSTCTL_TXFFLSH_Msk              (0x1U << USB_OTG_GRSTCTL_TXFFLSH_Pos) /*!< 0x00000020 */
#define USB_OTG_GRSTCTL_TXFFLSH                  USB_OTG_GRSTCTL_TXFFLSH_Msk   /*!< TxFIFO flush             */


#define USB_OTG_GRSTCTL_TXFNUM_Pos               (6U)                          
#define USB_OTG_GRSTCTL_TXFNUM_Msk               (0x1FU << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x000007C0 */
#define USB_OTG_GRSTCTL_TXFNUM                   USB_OTG_GRSTCTL_TXFNUM_Msk    /*!< TxFIFO number */
#define USB_OTG_GRSTCTL_TXFNUM_0                 (0x01U << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000040 */
#define USB_OTG_GRSTCTL_TXFNUM_1                 (0x02U << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000080 */
#define USB_OTG_GRSTCTL_TXFNUM_2                 (0x04U << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000100 */
#define USB_OTG_GRSTCTL_TXFNUM_3                 (0x08U << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000200 */
#define USB_OTG_GRSTCTL_TXFNUM_4                 (0x10U << USB_OTG_GRSTCTL_TXFNUM_Pos) /*!< 0x00000400 */
#define USB_OTG_GRSTCTL_DMAREQ_Pos               (30U)                         
#define USB_OTG_GRSTCTL_DMAREQ_Msk               (0x1U << USB_OTG_GRSTCTL_DMAREQ_Pos) /*!< 0x40000000 */
#define USB_OTG_GRSTCTL_DMAREQ                   USB_OTG_GRSTCTL_DMAREQ_Msk    /*!< DMA request signal */
#define USB_OTG_GRSTCTL_AHBIDL_Pos               (31U)                         
#define USB_OTG_GRSTCTL_AHBIDL_Msk               (0x1U << USB_OTG_GRSTCTL_AHBIDL_Pos) /*!< 0x80000000 */
#define USB_OTG_GRSTCTL_AHBIDL                   USB_OTG_GRSTCTL_AHBIDL_Msk    /*!< AHB master idle */

/********************  Bit definition for USB_OTG_DIEPMSK register  ********************/
#define USB_OTG_DIEPMSK_XFRCM_Pos                (0U)                          
#define USB_OTG_DIEPMSK_XFRCM_Msk                (0x1U << USB_OTG_DIEPMSK_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DIEPMSK_XFRCM                    USB_OTG_DIEPMSK_XFRCM_Msk     /*!< Transfer completed interrupt mask                 */
#define USB_OTG_DIEPMSK_EPDM_Pos                 (1U)                          
#define USB_OTG_DIEPMSK_EPDM_Msk                 (0x1U << USB_OTG_DIEPMSK_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DIEPMSK_EPDM                     USB_OTG_DIEPMSK_EPDM_Msk      /*!< Endpoint disabled interrupt mask                  */
#define USB_OTG_DIEPMSK_TOM_Pos                  (3U)                          
#define USB_OTG_DIEPMSK_TOM_Msk                  (0x1U << USB_OTG_DIEPMSK_TOM_Pos) /*!< 0x00000008 */
#define USB_OTG_DIEPMSK_TOM                      USB_OTG_DIEPMSK_TOM_Msk       /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPMSK_ITTXFEMSK_Pos            (4U)                          
#define USB_OTG_DIEPMSK_ITTXFEMSK_Msk            (0x1U << USB_OTG_DIEPMSK_ITTXFEMSK_Pos) /*!< 0x00000010 */
#define USB_OTG_DIEPMSK_ITTXFEMSK                USB_OTG_DIEPMSK_ITTXFEMSK_Msk /*!< IN token received when TxFIFO empty mask          */
#define USB_OTG_DIEPMSK_INEPNMM_Pos              (5U)                          
#define USB_OTG_DIEPMSK_INEPNMM_Msk              (0x1U << USB_OTG_DIEPMSK_INEPNMM_Pos) /*!< 0x00000020 */
#define USB_OTG_DIEPMSK_INEPNMM                  USB_OTG_DIEPMSK_INEPNMM_Msk   /*!< IN token received with EP mismatch mask           */
#define USB_OTG_DIEPMSK_INEPNEM_Pos              (6U)                          
#define USB_OTG_DIEPMSK_INEPNEM_Msk              (0x1U << USB_OTG_DIEPMSK_INEPNEM_Pos) /*!< 0x00000040 */
#define USB_OTG_DIEPMSK_INEPNEM                  USB_OTG_DIEPMSK_INEPNEM_Msk   /*!< IN endpoint NAK effective mask                    */
#define USB_OTG_DIEPMSK_TXFURM_Pos               (8U)                          
#define USB_OTG_DIEPMSK_TXFURM_Msk               (0x1U << USB_OTG_DIEPMSK_TXFURM_Pos) /*!< 0x00000100 */
#define USB_OTG_DIEPMSK_TXFURM                   USB_OTG_DIEPMSK_TXFURM_Msk    /*!< FIFO underrun mask                                */
#define USB_OTG_DIEPMSK_BIM_Pos                  (9U)                          
#define USB_OTG_DIEPMSK_BIM_Msk                  (0x1U << USB_OTG_DIEPMSK_BIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DIEPMSK_BIM                      USB_OTG_DIEPMSK_BIM_Msk       /*!< BNA interrupt mask                                */

/********************  Bit definition for USB_OTG_HPTXSTS register  ********************/
#define USB_OTG_HPTXSTS_PTXFSAVL_Pos             (0U)                          
#define USB_OTG_HPTXSTS_PTXFSAVL_Msk             (0xFFFFU << USB_OTG_HPTXSTS_PTXFSAVL_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HPTXSTS_PTXFSAVL                 USB_OTG_HPTXSTS_PTXFSAVL_Msk  /*!< Periodic transmit data FIFO space available     */
#define USB_OTG_HPTXSTS_PTXQSAV_Pos              (16U)                         
#define USB_OTG_HPTXSTS_PTXQSAV_Msk              (0xFFU << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00FF0000 */
#define USB_OTG_HPTXSTS_PTXQSAV                  USB_OTG_HPTXSTS_PTXQSAV_Msk   /*!< Periodic transmit request queue space available */
#define USB_OTG_HPTXSTS_PTXQSAV_0                (0x01U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00010000 */
#define USB_OTG_HPTXSTS_PTXQSAV_1                (0x02U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00020000 */
#define USB_OTG_HPTXSTS_PTXQSAV_2                (0x04U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00040000 */
#define USB_OTG_HPTXSTS_PTXQSAV_3                (0x08U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00080000 */
#define USB_OTG_HPTXSTS_PTXQSAV_4                (0x10U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00100000 */
#define USB_OTG_HPTXSTS_PTXQSAV_5                (0x20U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00200000 */
#define USB_OTG_HPTXSTS_PTXQSAV_6                (0x40U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00400000 */
#define USB_OTG_HPTXSTS_PTXQSAV_7                (0x80U << USB_OTG_HPTXSTS_PTXQSAV_Pos) /*!< 0x00800000 */

#define USB_OTG_HPTXSTS_PTXQTOP_Pos              (24U)                         
#define USB_OTG_HPTXSTS_PTXQTOP_Msk              (0xFFU << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0xFF000000 */
#define USB_OTG_HPTXSTS_PTXQTOP                  USB_OTG_HPTXSTS_PTXQTOP_Msk   /*!< Top of the periodic transmit request queue */
#define USB_OTG_HPTXSTS_PTXQTOP_0                (0x01U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x01000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_1                (0x02U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x02000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_2                (0x04U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x04000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_3                (0x08U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x08000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_4                (0x10U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x10000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_5                (0x20U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x20000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_6                (0x40U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x40000000 */
#define USB_OTG_HPTXSTS_PTXQTOP_7                (0x80U << USB_OTG_HPTXSTS_PTXQTOP_Pos) /*!< 0x80000000 */

/********************  Bit definition for USB_OTG_HAINT register  ********************/
#define USB_OTG_HAINT_HAINT_Pos                  (0U)                          
#define USB_OTG_HAINT_HAINT_Msk                  (0xFFFFU << USB_OTG_HAINT_HAINT_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HAINT_HAINT                      USB_OTG_HAINT_HAINT_Msk       /*!< Channel interrupts */

/********************  Bit definition for USB_OTG_DOEPMSK register  ********************/
#define USB_OTG_DOEPMSK_XFRCM_Pos                (0U)                          
#define USB_OTG_DOEPMSK_XFRCM_Msk                (0x1U << USB_OTG_DOEPMSK_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DOEPMSK_XFRCM                    USB_OTG_DOEPMSK_XFRCM_Msk     /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPMSK_EPDM_Pos                 (1U)                          
#define USB_OTG_DOEPMSK_EPDM_Msk                 (0x1U << USB_OTG_DOEPMSK_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DOEPMSK_EPDM                     USB_OTG_DOEPMSK_EPDM_Msk      /*!< Endpoint disabled interrupt mask               */
#define USB_OTG_DOEPMSK_STUPM_Pos                (3U)                          
#define USB_OTG_DOEPMSK_STUPM_Msk                (0x1U << USB_OTG_DOEPMSK_STUPM_Pos) /*!< 0x00000008 */
#define USB_OTG_DOEPMSK_STUPM                    USB_OTG_DOEPMSK_STUPM_Msk     /*!< SETUP phase done mask                          */
#define USB_OTG_DOEPMSK_OTEPDM_Pos               (4U)                          
#define USB_OTG_DOEPMSK_OTEPDM_Msk               (0x1U << USB_OTG_DOEPMSK_OTEPDM_Pos) /*!< 0x00000010 */
#define USB_OTG_DOEPMSK_OTEPDM                   USB_OTG_DOEPMSK_OTEPDM_Msk    /*!< OUT token received when endpoint disabled mask */
#define USB_OTG_DOEPMSK_B2BSTUP_Pos              (6U)                          
#define USB_OTG_DOEPMSK_B2BSTUP_Msk              (0x1U << USB_OTG_DOEPMSK_B2BSTUP_Pos) /*!< 0x00000040 */
#define USB_OTG_DOEPMSK_B2BSTUP                  USB_OTG_DOEPMSK_B2BSTUP_Msk   /*!< Back-to-back SETUP packets received mask */
#define USB_OTG_DOEPMSK_OPEM_Pos                 (8U)                          
#define USB_OTG_DOEPMSK_OPEM_Msk                 (0x1U << USB_OTG_DOEPMSK_OPEM_Pos) /*!< 0x00000100 */
#define USB_OTG_DOEPMSK_OPEM                     USB_OTG_DOEPMSK_OPEM_Msk      /*!< OUT packet error mask                          */
#define USB_OTG_DOEPMSK_BOIM_Pos                 (9U)                          
#define USB_OTG_DOEPMSK_BOIM_Msk                 (0x1U << USB_OTG_DOEPMSK_BOIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DOEPMSK_BOIM                     USB_OTG_DOEPMSK_BOIM_Msk      /*!< BNA interrupt mask                             */

/********************  Bit definition for USB_OTG_GINTSTS register  ********************/
#define USB_OTG_GINTSTS_CMOD_Pos                 (0U)                          
#define USB_OTG_GINTSTS_CMOD_Msk                 (0x1U << USB_OTG_GINTSTS_CMOD_Pos) /*!< 0x00000001 */
#define USB_OTG_GINTSTS_CMOD                     USB_OTG_GINTSTS_CMOD_Msk      /*!< Current mode of operation                      */
#define USB_OTG_GINTSTS_MMIS_Pos                 (1U)                          
#define USB_OTG_GINTSTS_MMIS_Msk                 (0x1U << USB_OTG_GINTSTS_MMIS_Pos) /*!< 0x00000002 */
#define USB_OTG_GINTSTS_MMIS                     USB_OTG_GINTSTS_MMIS_Msk      /*!< Mode mismatch interrupt                        */
#define USB_OTG_GINTSTS_OTGINT_Pos               (2U)                          
#define USB_OTG_GINTSTS_OTGINT_Msk               (0x1U << USB_OTG_GINTSTS_OTGINT_Pos) /*!< 0x00000004 */
#define USB_OTG_GINTSTS_OTGINT                   USB_OTG_GINTSTS_OTGINT_Msk    /*!< OTG interrupt                                  */
#define USB_OTG_GINTSTS_SOF_Pos                  (3U)                          
#define USB_OTG_GINTSTS_SOF_Msk                  (0x1U << USB_OTG_GINTSTS_SOF_Pos) /*!< 0x00000008 */
#define USB_OTG_GINTSTS_SOF                      USB_OTG_GINTSTS_SOF_Msk       /*!< Start of frame                                 */
#define USB_OTG_GINTSTS_RXFLVL_Pos               (4U)                          
#define USB_OTG_GINTSTS_RXFLVL_Msk               (0x1U << USB_OTG_GINTSTS_RXFLVL_Pos) /*!< 0x00000010 */
#define USB_OTG_GINTSTS_RXFLVL                   USB_OTG_GINTSTS_RXFLVL_Msk    /*!< RxFIFO nonempty                                */
#define USB_OTG_GINTSTS_NPTXFE_Pos               (5U)                          
#define USB_OTG_GINTSTS_NPTXFE_Msk               (0x1U << USB_OTG_GINTSTS_NPTXFE_Pos) /*!< 0x00000020 */
#define USB_OTG_GINTSTS_NPTXFE                   USB_OTG_GINTSTS_NPTXFE_Msk    /*!< Nonperiodic TxFIFO empty                       */
#define USB_OTG_GINTSTS_GINAKEFF_Pos             (6U)                          
#define USB_OTG_GINTSTS_GINAKEFF_Msk             (0x1U << USB_OTG_GINTSTS_GINAKEFF_Pos) /*!< 0x00000040 */
#define USB_OTG_GINTSTS_GINAKEFF                 USB_OTG_GINTSTS_GINAKEFF_Msk  /*!< Global IN nonperiodic NAK effective            */
#define USB_OTG_GINTSTS_BOUTNAKEFF_Pos           (7U)                          
#define USB_OTG_GINTSTS_BOUTNAKEFF_Msk           (0x1U << USB_OTG_GINTSTS_BOUTNAKEFF_Pos) /*!< 0x00000080 */
#define USB_OTG_GINTSTS_BOUTNAKEFF               USB_OTG_GINTSTS_BOUTNAKEFF_Msk /*!< Global OUT NAK effective                       */
#define USB_OTG_GINTSTS_ESUSP_Pos                (10U)                         
#define USB_OTG_GINTSTS_ESUSP_Msk                (0x1U << USB_OTG_GINTSTS_ESUSP_Pos) /*!< 0x00000400 */
#define USB_OTG_GINTSTS_ESUSP                    USB_OTG_GINTSTS_ESUSP_Msk     /*!< Early suspend                                  */
#define USB_OTG_GINTSTS_USBSUSP_Pos              (11U)                         
#define USB_OTG_GINTSTS_USBSUSP_Msk              (0x1U << USB_OTG_GINTSTS_USBSUSP_Pos) /*!< 0x00000800 */
#define USB_OTG_GINTSTS_USBSUSP                  USB_OTG_GINTSTS_USBSUSP_Msk   /*!< USB suspend                                    */
#define USB_OTG_GINTSTS_USBRST_Pos               (12U)                         
#define USB_OTG_GINTSTS_USBRST_Msk               (0x1U << USB_OTG_GINTSTS_USBRST_Pos) /*!< 0x00001000 */
#define USB_OTG_GINTSTS_USBRST                   USB_OTG_GINTSTS_USBRST_Msk    /*!< USB reset                                      */
#define USB_OTG_GINTSTS_ENUMDNE_Pos              (13U)                         
#define USB_OTG_GINTSTS_ENUMDNE_Msk              (0x1U << USB_OTG_GINTSTS_ENUMDNE_Pos) /*!< 0x00002000 */
#define USB_OTG_GINTSTS_ENUMDNE                  USB_OTG_GINTSTS_ENUMDNE_Msk   /*!< Enumeration done                               */
#define USB_OTG_GINTSTS_ISOODRP_Pos              (14U)                         
#define USB_OTG_GINTSTS_ISOODRP_Msk              (0x1U << USB_OTG_GINTSTS_ISOODRP_Pos) /*!< 0x00004000 */
#define USB_OTG_GINTSTS_ISOODRP                  USB_OTG_GINTSTS_ISOODRP_Msk   /*!< Isochronous OUT packet dropped interrupt       */
#define USB_OTG_GINTSTS_EOPF_Pos                 (15U)                         
#define USB_OTG_GINTSTS_EOPF_Msk                 (0x1U << USB_OTG_GINTSTS_EOPF_Pos) /*!< 0x00008000 */
#define USB_OTG_GINTSTS_EOPF                     USB_OTG_GINTSTS_EOPF_Msk      /*!< End of periodic frame interrupt                */
#define USB_OTG_GINTSTS_IEPINT_Pos               (18U)                         
#define USB_OTG_GINTSTS_IEPINT_Msk               (0x1U << USB_OTG_GINTSTS_IEPINT_Pos) /*!< 0x00040000 */
#define USB_OTG_GINTSTS_IEPINT                   USB_OTG_GINTSTS_IEPINT_Msk    /*!< IN endpoint interrupt                          */
#define USB_OTG_GINTSTS_OEPINT_Pos               (19U)                         
#define USB_OTG_GINTSTS_OEPINT_Msk               (0x1U << USB_OTG_GINTSTS_OEPINT_Pos) /*!< 0x00080000 */
#define USB_OTG_GINTSTS_OEPINT                   USB_OTG_GINTSTS_OEPINT_Msk    /*!< OUT endpoint interrupt                         */
#define USB_OTG_GINTSTS_IISOIXFR_Pos             (20U)                         
#define USB_OTG_GINTSTS_IISOIXFR_Msk             (0x1U << USB_OTG_GINTSTS_IISOIXFR_Pos) /*!< 0x00100000 */
#define USB_OTG_GINTSTS_IISOIXFR                 USB_OTG_GINTSTS_IISOIXFR_Msk  /*!< Incomplete isochronous IN transfer             */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos    (21U)                         
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk    (0x1U << USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Pos) /*!< 0x00200000 */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT        USB_OTG_GINTSTS_PXFR_INCOMPISOOUT_Msk /*!< Incomplete periodic transfer                   */
#define USB_OTG_GINTSTS_DATAFSUSP_Pos            (22U)                         
#define USB_OTG_GINTSTS_DATAFSUSP_Msk            (0x1U << USB_OTG_GINTSTS_DATAFSUSP_Pos) /*!< 0x00400000 */
#define USB_OTG_GINTSTS_DATAFSUSP                USB_OTG_GINTSTS_DATAFSUSP_Msk /*!< Data fetch suspended                           */
#define USB_OTG_GINTSTS_HPRTINT_Pos              (24U)                         
#define USB_OTG_GINTSTS_HPRTINT_Msk              (0x1U << USB_OTG_GINTSTS_HPRTINT_Pos) /*!< 0x01000000 */
#define USB_OTG_GINTSTS_HPRTINT                  USB_OTG_GINTSTS_HPRTINT_Msk   /*!< Host port interrupt                            */
#define USB_OTG_GINTSTS_HCINT_Pos                (25U)                         
#define USB_OTG_GINTSTS_HCINT_Msk                (0x1U << USB_OTG_GINTSTS_HCINT_Pos) /*!< 0x02000000 */
#define USB_OTG_GINTSTS_HCINT                    USB_OTG_GINTSTS_HCINT_Msk     /*!< Host channels interrupt                        */
#define USB_OTG_GINTSTS_PTXFE_Pos                (26U)                         
#define USB_OTG_GINTSTS_PTXFE_Msk                (0x1U << USB_OTG_GINTSTS_PTXFE_Pos) /*!< 0x04000000 */
#define USB_OTG_GINTSTS_PTXFE                    USB_OTG_GINTSTS_PTXFE_Msk     /*!< Periodic TxFIFO empty                          */
#define USB_OTG_GINTSTS_CIDSCHG_Pos              (28U)                         
#define USB_OTG_GINTSTS_CIDSCHG_Msk              (0x1U << USB_OTG_GINTSTS_CIDSCHG_Pos) /*!< 0x10000000 */
#define USB_OTG_GINTSTS_CIDSCHG                  USB_OTG_GINTSTS_CIDSCHG_Msk   /*!< Connector ID status change                     */
#define USB_OTG_GINTSTS_DISCINT_Pos              (29U)                         
#define USB_OTG_GINTSTS_DISCINT_Msk              (0x1U << USB_OTG_GINTSTS_DISCINT_Pos) /*!< 0x20000000 */
#define USB_OTG_GINTSTS_DISCINT                  USB_OTG_GINTSTS_DISCINT_Msk   /*!< Disconnect detected interrupt                  */
#define USB_OTG_GINTSTS_SRQINT_Pos               (30U)                         
#define USB_OTG_GINTSTS_SRQINT_Msk               (0x1U << USB_OTG_GINTSTS_SRQINT_Pos) /*!< 0x40000000 */
#define USB_OTG_GINTSTS_SRQINT                   USB_OTG_GINTSTS_SRQINT_Msk    /*!< Session request/new session detected interrupt */
#define USB_OTG_GINTSTS_WKUINT_Pos               (31U)                         
#define USB_OTG_GINTSTS_WKUINT_Msk               (0x1U << USB_OTG_GINTSTS_WKUINT_Pos) /*!< 0x80000000 */
#define USB_OTG_GINTSTS_WKUINT                   USB_OTG_GINTSTS_WKUINT_Msk    /*!< Resume/remote wakeup detected interrupt        */

/********************  Bit definition for USB_OTG_GINTMSK register  ********************/
#define USB_OTG_GINTMSK_MMISM_Pos                (1U)                          
#define USB_OTG_GINTMSK_MMISM_Msk                (0x1U << USB_OTG_GINTMSK_MMISM_Pos) /*!< 0x00000002 */
#define USB_OTG_GINTMSK_MMISM                    USB_OTG_GINTMSK_MMISM_Msk     /*!< Mode mismatch interrupt mask                        */
#define USB_OTG_GINTMSK_OTGINT_Pos               (2U)                          
#define USB_OTG_GINTMSK_OTGINT_Msk               (0x1U << USB_OTG_GINTMSK_OTGINT_Pos) /*!< 0x00000004 */
#define USB_OTG_GINTMSK_OTGINT                   USB_OTG_GINTMSK_OTGINT_Msk    /*!< OTG interrupt mask                                  */
#define USB_OTG_GINTMSK_SOFM_Pos                 (3U)                          
#define USB_OTG_GINTMSK_SOFM_Msk                 (0x1U << USB_OTG_GINTMSK_SOFM_Pos) /*!< 0x00000008 */
#define USB_OTG_GINTMSK_SOFM                     USB_OTG_GINTMSK_SOFM_Msk      /*!< Start of frame mask                                 */
#define USB_OTG_GINTMSK_RXFLVLM_Pos              (4U)                          
#define USB_OTG_GINTMSK_RXFLVLM_Msk              (0x1U << USB_OTG_GINTMSK_RXFLVLM_Pos) /*!< 0x00000010 */
#define USB_OTG_GINTMSK_RXFLVLM                  USB_OTG_GINTMSK_RXFLVLM_Msk   /*!< Receive FIFO nonempty mask                          */
#define USB_OTG_GINTMSK_NPTXFEM_Pos              (5U)                          
#define USB_OTG_GINTMSK_NPTXFEM_Msk              (0x1U << USB_OTG_GINTMSK_NPTXFEM_Pos) /*!< 0x00000020 */
#define USB_OTG_GINTMSK_NPTXFEM                  USB_OTG_GINTMSK_NPTXFEM_Msk   /*!< Nonperiodic TxFIFO empty mask                       */
#define USB_OTG_GINTMSK_GINAKEFFM_Pos            (6U)                          
#define USB_OTG_GINTMSK_GINAKEFFM_Msk            (0x1U << USB_OTG_GINTMSK_GINAKEFFM_Pos) /*!< 0x00000040 */
#define USB_OTG_GINTMSK_GINAKEFFM                USB_OTG_GINTMSK_GINAKEFFM_Msk /*!< Global nonperiodic IN NAK effective mask            */
#define USB_OTG_GINTMSK_GONAKEFFM_Pos            (7U)                          
#define USB_OTG_GINTMSK_GONAKEFFM_Msk            (0x1U << USB_OTG_GINTMSK_GONAKEFFM_Pos) /*!< 0x00000080 */
#define USB_OTG_GINTMSK_GONAKEFFM                USB_OTG_GINTMSK_GONAKEFFM_Msk /*!< Global OUT NAK effective mask                       */
#define USB_OTG_GINTMSK_ESUSPM_Pos               (10U)                         
#define USB_OTG_GINTMSK_ESUSPM_Msk               (0x1U << USB_OTG_GINTMSK_ESUSPM_Pos) /*!< 0x00000400 */
#define USB_OTG_GINTMSK_ESUSPM                   USB_OTG_GINTMSK_ESUSPM_Msk    /*!< Early suspend mask                                  */
#define USB_OTG_GINTMSK_USBSUSPM_Pos             (11U)                         
#define USB_OTG_GINTMSK_USBSUSPM_Msk             (0x1U << USB_OTG_GINTMSK_USBSUSPM_Pos) /*!< 0x00000800 */
#define USB_OTG_GINTMSK_USBSUSPM                 USB_OTG_GINTMSK_USBSUSPM_Msk  /*!< USB suspend mask                                    */
#define USB_OTG_GINTMSK_USBRST_Pos               (12U)                         
#define USB_OTG_GINTMSK_USBRST_Msk               (0x1U << USB_OTG_GINTMSK_USBRST_Pos) /*!< 0x00001000 */
#define USB_OTG_GINTMSK_USBRST                   USB_OTG_GINTMSK_USBRST_Msk    /*!< USB reset mask                                      */
#define USB_OTG_GINTMSK_ENUMDNEM_Pos             (13U)                         
#define USB_OTG_GINTMSK_ENUMDNEM_Msk             (0x1U << USB_OTG_GINTMSK_ENUMDNEM_Pos) /*!< 0x00002000 */
#define USB_OTG_GINTMSK_ENUMDNEM                 USB_OTG_GINTMSK_ENUMDNEM_Msk  /*!< Enumeration done mask                               */
#define USB_OTG_GINTMSK_ISOODRPM_Pos             (14U)                         
#define USB_OTG_GINTMSK_ISOODRPM_Msk             (0x1U << USB_OTG_GINTMSK_ISOODRPM_Pos) /*!< 0x00004000 */
#define USB_OTG_GINTMSK_ISOODRPM                 USB_OTG_GINTMSK_ISOODRPM_Msk  /*!< Isochronous OUT packet dropped interrupt mask       */
#define USB_OTG_GINTMSK_EOPFM_Pos                (15U)                         
#define USB_OTG_GINTMSK_EOPFM_Msk                (0x1U << USB_OTG_GINTMSK_EOPFM_Pos) /*!< 0x00008000 */
#define USB_OTG_GINTMSK_EOPFM                    USB_OTG_GINTMSK_EOPFM_Msk     /*!< End of periodic frame interrupt mask                */
#define USB_OTG_GINTMSK_EPMISM_Pos               (17U)                         
#define USB_OTG_GINTMSK_EPMISM_Msk               (0x1U << USB_OTG_GINTMSK_EPMISM_Pos) /*!< 0x00020000 */
#define USB_OTG_GINTMSK_EPMISM                   USB_OTG_GINTMSK_EPMISM_Msk    /*!< Endpoint mismatch interrupt mask                    */
#define USB_OTG_GINTMSK_IEPINT_Pos               (18U)                         
#define USB_OTG_GINTMSK_IEPINT_Msk               (0x1U << USB_OTG_GINTMSK_IEPINT_Pos) /*!< 0x00040000 */
#define USB_OTG_GINTMSK_IEPINT                   USB_OTG_GINTMSK_IEPINT_Msk    /*!< IN endpoints interrupt mask                         */
#define USB_OTG_GINTMSK_OEPINT_Pos               (19U)                         
#define USB_OTG_GINTMSK_OEPINT_Msk               (0x1U << USB_OTG_GINTMSK_OEPINT_Pos) /*!< 0x00080000 */
#define USB_OTG_GINTMSK_OEPINT                   USB_OTG_GINTMSK_OEPINT_Msk    /*!< OUT endpoints interrupt mask                        */
#define USB_OTG_GINTMSK_IISOIXFRM_Pos            (20U)                         
#define USB_OTG_GINTMSK_IISOIXFRM_Msk            (0x1U << USB_OTG_GINTMSK_IISOIXFRM_Pos) /*!< 0x00100000 */
#define USB_OTG_GINTMSK_IISOIXFRM                USB_OTG_GINTMSK_IISOIXFRM_Msk /*!< Incomplete isochronous IN transfer mask             */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Pos      (21U)                         
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Msk      (0x1U << USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Pos) /*!< 0x00200000 */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM          USB_OTG_GINTMSK_PXFRM_IISOOXFRM_Msk /*!< Incomplete periodic transfer mask                   */
#define USB_OTG_GINTMSK_FSUSPM_Pos               (22U)                         
#define USB_OTG_GINTMSK_FSUSPM_Msk               (0x1U << USB_OTG_GINTMSK_FSUSPM_Pos) /*!< 0x00400000 */
#define USB_OTG_GINTMSK_FSUSPM                   USB_OTG_GINTMSK_FSUSPM_Msk    /*!< Data fetch suspended mask                           */
#define USB_OTG_GINTMSK_PRTIM_Pos                (24U)                         
#define USB_OTG_GINTMSK_PRTIM_Msk                (0x1U << USB_OTG_GINTMSK_PRTIM_Pos) /*!< 0x01000000 */
#define USB_OTG_GINTMSK_PRTIM                    USB_OTG_GINTMSK_PRTIM_Msk     /*!< Host port interrupt mask                            */
#define USB_OTG_GINTMSK_HCIM_Pos                 (25U)                         
#define USB_OTG_GINTMSK_HCIM_Msk                 (0x1U << USB_OTG_GINTMSK_HCIM_Pos) /*!< 0x02000000 */
#define USB_OTG_GINTMSK_HCIM                     USB_OTG_GINTMSK_HCIM_Msk      /*!< Host channels interrupt mask                        */
#define USB_OTG_GINTMSK_PTXFEM_Pos               (26U)                         
#define USB_OTG_GINTMSK_PTXFEM_Msk               (0x1U << USB_OTG_GINTMSK_PTXFEM_Pos) /*!< 0x04000000 */
#define USB_OTG_GINTMSK_PTXFEM                   USB_OTG_GINTMSK_PTXFEM_Msk    /*!< Periodic TxFIFO empty mask                          */
#define USB_OTG_GINTMSK_CIDSCHGM_Pos             (28U)                         
#define USB_OTG_GINTMSK_CIDSCHGM_Msk             (0x1U << USB_OTG_GINTMSK_CIDSCHGM_Pos) /*!< 0x10000000 */
#define USB_OTG_GINTMSK_CIDSCHGM                 USB_OTG_GINTMSK_CIDSCHGM_Msk  /*!< Connector ID status change mask                     */
#define USB_OTG_GINTMSK_DISCINT_Pos              (29U)                         
#define USB_OTG_GINTMSK_DISCINT_Msk              (0x1U << USB_OTG_GINTMSK_DISCINT_Pos) /*!< 0x20000000 */
#define USB_OTG_GINTMSK_DISCINT                  USB_OTG_GINTMSK_DISCINT_Msk   /*!< Disconnect detected interrupt mask                  */
#define USB_OTG_GINTMSK_SRQIM_Pos                (30U)                         
#define USB_OTG_GINTMSK_SRQIM_Msk                (0x1U << USB_OTG_GINTMSK_SRQIM_Pos) /*!< 0x40000000 */
#define USB_OTG_GINTMSK_SRQIM                    USB_OTG_GINTMSK_SRQIM_Msk     /*!< Session request/new session detected interrupt mask */
#define USB_OTG_GINTMSK_WUIM_Pos                 (31U)                         
#define USB_OTG_GINTMSK_WUIM_Msk                 (0x1U << USB_OTG_GINTMSK_WUIM_Pos) /*!< 0x80000000 */
#define USB_OTG_GINTMSK_WUIM                     USB_OTG_GINTMSK_WUIM_Msk      /*!< Resume/remote wakeup detected interrupt mask        */

/********************  Bit definition for USB_OTG_DAINT register  ********************/
#define USB_OTG_DAINT_IEPINT_Pos                 (0U)                          
#define USB_OTG_DAINT_IEPINT_Msk                 (0xFFFFU << USB_OTG_DAINT_IEPINT_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DAINT_IEPINT                     USB_OTG_DAINT_IEPINT_Msk      /*!< IN endpoint interrupt bits  */
#define USB_OTG_DAINT_OEPINT_Pos                 (16U)                         
#define USB_OTG_DAINT_OEPINT_Msk                 (0xFFFFU << USB_OTG_DAINT_OEPINT_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_DAINT_OEPINT                     USB_OTG_DAINT_OEPINT_Msk      /*!< OUT endpoint interrupt bits */

/********************  Bit definition for USB_OTG_HAINTMSK register  ********************/
#define USB_OTG_HAINTMSK_HAINTM_Pos              (0U)                          
#define USB_OTG_HAINTMSK_HAINTM_Msk              (0xFFFFU << USB_OTG_HAINTMSK_HAINTM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HAINTMSK_HAINTM                  USB_OTG_HAINTMSK_HAINTM_Msk   /*!< Channel interrupt mask */

/********************  Bit definition for USB_OTG_GRXSTSP register  ********************/
#define USB_OTG_GRXSTSP_EPNUM_Pos                (0U)                          
#define USB_OTG_GRXSTSP_EPNUM_Msk                (0xFU << USB_OTG_GRXSTSP_EPNUM_Pos) /*!< 0x0000000F */
#define USB_OTG_GRXSTSP_EPNUM                    USB_OTG_GRXSTSP_EPNUM_Msk     /*!< IN EP interrupt mask bits  */
#define USB_OTG_GRXSTSP_BCNT_Pos                 (4U)                          
#define USB_OTG_GRXSTSP_BCNT_Msk                 (0x7FFU << USB_OTG_GRXSTSP_BCNT_Pos) /*!< 0x00007FF0 */
#define USB_OTG_GRXSTSP_BCNT                     USB_OTG_GRXSTSP_BCNT_Msk      /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_DPID_Pos                 (15U)                         
#define USB_OTG_GRXSTSP_DPID_Msk                 (0x3U << USB_OTG_GRXSTSP_DPID_Pos) /*!< 0x00018000 */
#define USB_OTG_GRXSTSP_DPID                     USB_OTG_GRXSTSP_DPID_Msk      /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_PKTSTS_Pos               (17U)                         
#define USB_OTG_GRXSTSP_PKTSTS_Msk               (0xFU << USB_OTG_GRXSTSP_PKTSTS_Pos) /*!< 0x001E0000 */
#define USB_OTG_GRXSTSP_PKTSTS                   USB_OTG_GRXSTSP_PKTSTS_Msk    /*!< OUT EP interrupt mask bits */

/********************  Bit definition for USB_OTG_DAINTMSK register  ********************/
#define USB_OTG_DAINTMSK_IEPM_Pos                (0U)                          
#define USB_OTG_DAINTMSK_IEPM_Msk                (0xFFFFU << USB_OTG_DAINTMSK_IEPM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DAINTMSK_IEPM                    USB_OTG_DAINTMSK_IEPM_Msk     /*!< IN EP interrupt mask bits */
#define USB_OTG_DAINTMSK_OEPM_Pos                (16U)                         
#define USB_OTG_DAINTMSK_OEPM_Msk                (0xFFFFU << USB_OTG_DAINTMSK_OEPM_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_DAINTMSK_OEPM                    USB_OTG_DAINTMSK_OEPM_Msk     /*!< OUT EP interrupt mask bits */

/********************  Bit definition for USB_OTG_GRXFSIZ register  ********************/
#define USB_OTG_GRXFSIZ_RXFD_Pos                 (0U)                          
#define USB_OTG_GRXFSIZ_RXFD_Msk                 (0xFFFFU << USB_OTG_GRXFSIZ_RXFD_Pos) /*!< 0x0000FFFF */
#define USB_OTG_GRXFSIZ_RXFD                     USB_OTG_GRXFSIZ_RXFD_Msk      /*!< RxFIFO depth */

/********************  Bit definition for USB_OTG_DVBUSDIS register  ********************/
#define USB_OTG_DVBUSDIS_VBUSDT_Pos              (0U)                          
#define USB_OTG_DVBUSDIS_VBUSDT_Msk              (0xFFFFU << USB_OTG_DVBUSDIS_VBUSDT_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DVBUSDIS_VBUSDT                  USB_OTG_DVBUSDIS_VBUSDT_Msk   /*!< Device VBUS discharge time */

/********************  Bit definition for OTG register  ********************/
#define USB_OTG_NPTXFSA_Pos                      (0U)                          
#define USB_OTG_NPTXFSA_Msk                      (0xFFFFU << USB_OTG_NPTXFSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_NPTXFSA                          USB_OTG_NPTXFSA_Msk           /*!< Nonperiodic transmit RAM start address */
#define USB_OTG_NPTXFD_Pos                       (16U)                         
#define USB_OTG_NPTXFD_Msk                       (0xFFFFU << USB_OTG_NPTXFD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_NPTXFD                           USB_OTG_NPTXFD_Msk            /*!< Nonperiodic TxFIFO depth               */
#define USB_OTG_TX0FSA_Pos                       (0U)                          
#define USB_OTG_TX0FSA_Msk                       (0xFFFFU << USB_OTG_TX0FSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_TX0FSA                           USB_OTG_TX0FSA_Msk            /*!< Endpoint 0 transmit RAM start address  */
#define USB_OTG_TX0FD_Pos                        (16U)                         
#define USB_OTG_TX0FD_Msk                        (0xFFFFU << USB_OTG_TX0FD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_TX0FD                            USB_OTG_TX0FD_Msk             /*!< Endpoint 0 TxFIFO depth                */

/********************  Bit definition forUSB_OTG_DVBUSPULSE register  ********************/
#define USB_OTG_DVBUSPULSE_DVBUSP_Pos            (0U)                          
#define USB_OTG_DVBUSPULSE_DVBUSP_Msk            (0xFFFU << USB_OTG_DVBUSPULSE_DVBUSP_Pos) /*!< 0x00000FFF */
#define USB_OTG_DVBUSPULSE_DVBUSP                USB_OTG_DVBUSPULSE_DVBUSP_Msk /*!< Device VBUS pulsing time */

/********************  Bit definition for USB_OTG_GNPTXSTS register  ********************/
#define USB_OTG_GNPTXSTS_NPTXFSAV_Pos            (0U)                          
#define USB_OTG_GNPTXSTS_NPTXFSAV_Msk            (0xFFFFU << USB_OTG_GNPTXSTS_NPTXFSAV_Pos) /*!< 0x0000FFFF */
#define USB_OTG_GNPTXSTS_NPTXFSAV                USB_OTG_GNPTXSTS_NPTXFSAV_Msk /*!< Nonperiodic TxFIFO space available */

#define USB_OTG_GNPTXSTS_NPTQXSAV_Pos            (16U)                         
#define USB_OTG_GNPTXSTS_NPTQXSAV_Msk            (0xFFU << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00FF0000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV                USB_OTG_GNPTXSTS_NPTQXSAV_Msk /*!< Nonperiodic transmit request queue space available */
#define USB_OTG_GNPTXSTS_NPTQXSAV_0              (0x01U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00010000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_1              (0x02U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00020000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_2              (0x04U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00040000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_3              (0x08U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00080000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_4              (0x10U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00100000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_5              (0x20U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00200000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_6              (0x40U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00400000 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_7              (0x80U << USB_OTG_GNPTXSTS_NPTQXSAV_Pos) /*!< 0x00800000 */

#define USB_OTG_GNPTXSTS_NPTXQTOP_Pos            (24U)                         
#define USB_OTG_GNPTXSTS_NPTXQTOP_Msk            (0x7FU << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x7F000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP                USB_OTG_GNPTXSTS_NPTXQTOP_Msk /*!< Top of the nonperiodic transmit request queue */
#define USB_OTG_GNPTXSTS_NPTXQTOP_0              (0x01U << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x01000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_1              (0x02U << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x02000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_2              (0x04U << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x04000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_3              (0x08U << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x08000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_4              (0x10U << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x10000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_5              (0x20U << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x20000000 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_6              (0x40U << USB_OTG_GNPTXSTS_NPTXQTOP_Pos) /*!< 0x40000000 */

/********************  Bit definition for USB_OTG_DTHRCTL register  ********************/
#define USB_OTG_DTHRCTL_NONISOTHREN_Pos          (0U)                          
#define USB_OTG_DTHRCTL_NONISOTHREN_Msk          (0x1U << USB_OTG_DTHRCTL_NONISOTHREN_Pos) /*!< 0x00000001 */
#define USB_OTG_DTHRCTL_NONISOTHREN              USB_OTG_DTHRCTL_NONISOTHREN_Msk /*!< Nonisochronous IN endpoints threshold enable */
#define USB_OTG_DTHRCTL_ISOTHREN_Pos             (1U)                          
#define USB_OTG_DTHRCTL_ISOTHREN_Msk             (0x1U << USB_OTG_DTHRCTL_ISOTHREN_Pos) /*!< 0x00000002 */
#define USB_OTG_DTHRCTL_ISOTHREN                 USB_OTG_DTHRCTL_ISOTHREN_Msk  /*!< ISO IN endpoint threshold enable */

#define USB_OTG_DTHRCTL_TXTHRLEN_Pos             (2U)                          
#define USB_OTG_DTHRCTL_TXTHRLEN_Msk             (0x1FFU << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x000007FC */
#define USB_OTG_DTHRCTL_TXTHRLEN                 USB_OTG_DTHRCTL_TXTHRLEN_Msk  /*!< Transmit threshold length */
#define USB_OTG_DTHRCTL_TXTHRLEN_0               (0x001U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000004 */
#define USB_OTG_DTHRCTL_TXTHRLEN_1               (0x002U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000008 */
#define USB_OTG_DTHRCTL_TXTHRLEN_2               (0x004U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000010 */
#define USB_OTG_DTHRCTL_TXTHRLEN_3               (0x008U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000020 */
#define USB_OTG_DTHRCTL_TXTHRLEN_4               (0x010U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000040 */
#define USB_OTG_DTHRCTL_TXTHRLEN_5               (0x020U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000080 */
#define USB_OTG_DTHRCTL_TXTHRLEN_6               (0x040U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000100 */
#define USB_OTG_DTHRCTL_TXTHRLEN_7               (0x080U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000200 */
#define USB_OTG_DTHRCTL_TXTHRLEN_8               (0x100U << USB_OTG_DTHRCTL_TXTHRLEN_Pos) /*!< 0x00000400 */
#define USB_OTG_DTHRCTL_RXTHREN_Pos              (16U)                         
#define USB_OTG_DTHRCTL_RXTHREN_Msk              (0x1U << USB_OTG_DTHRCTL_RXTHREN_Pos) /*!< 0x00010000 */
#define USB_OTG_DTHRCTL_RXTHREN                  USB_OTG_DTHRCTL_RXTHREN_Msk   /*!< Receive threshold enable */

#define USB_OTG_DTHRCTL_RXTHRLEN_Pos             (17U)                         
#define USB_OTG_DTHRCTL_RXTHRLEN_Msk             (0x1FFU << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x03FE0000 */
#define USB_OTG_DTHRCTL_RXTHRLEN                 USB_OTG_DTHRCTL_RXTHRLEN_Msk  /*!< Receive threshold length */
#define USB_OTG_DTHRCTL_RXTHRLEN_0               (0x001U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00020000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_1               (0x002U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00040000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_2               (0x004U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00080000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_3               (0x008U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00100000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_4               (0x010U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00200000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_5               (0x020U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00400000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_6               (0x040U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x00800000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_7               (0x080U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x01000000 */
#define USB_OTG_DTHRCTL_RXTHRLEN_8               (0x100U << USB_OTG_DTHRCTL_RXTHRLEN_Pos) /*!< 0x02000000 */
#define USB_OTG_DTHRCTL_ARPEN_Pos                (27U)                         
#define USB_OTG_DTHRCTL_ARPEN_Msk                (0x1U << USB_OTG_DTHRCTL_ARPEN_Pos) /*!< 0x08000000 */
#define USB_OTG_DTHRCTL_ARPEN                    USB_OTG_DTHRCTL_ARPEN_Msk     /*!< Arbiter parking enable */

/********************  Bit definition for USB_OTG_DIEPEMPMSK register  ********************/
#define USB_OTG_DIEPEMPMSK_INEPTXFEM_Pos         (0U)                          
#define USB_OTG_DIEPEMPMSK_INEPTXFEM_Msk         (0xFFFFU << USB_OTG_DIEPEMPMSK_INEPTXFEM_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DIEPEMPMSK_INEPTXFEM             USB_OTG_DIEPEMPMSK_INEPTXFEM_Msk /*!< IN EP Tx FIFO empty interrupt mask bits */

/********************  Bit definition for USB_OTG_DEACHINT register  ********************/
#define USB_OTG_DEACHINT_IEP1INT_Pos             (1U)                          
#define USB_OTG_DEACHINT_IEP1INT_Msk             (0x1U << USB_OTG_DEACHINT_IEP1INT_Pos) /*!< 0x00000002 */
#define USB_OTG_DEACHINT_IEP1INT                 USB_OTG_DEACHINT_IEP1INT_Msk  /*!< IN endpoint 1interrupt bit   */
#define USB_OTG_DEACHINT_OEP1INT_Pos             (17U)                         
#define USB_OTG_DEACHINT_OEP1INT_Msk             (0x1U << USB_OTG_DEACHINT_OEP1INT_Pos) /*!< 0x00020000 */
#define USB_OTG_DEACHINT_OEP1INT                 USB_OTG_DEACHINT_OEP1INT_Msk  /*!< OUT endpoint 1 interrupt bit */

/********************  Bit definition for USB_OTG_GCCFG register  ********************/
#define USB_OTG_GCCFG_PWRDWN_Pos                 (16U)                         
#define USB_OTG_GCCFG_PWRDWN_Msk                 (0x1U << USB_OTG_GCCFG_PWRDWN_Pos) /*!< 0x00010000 */
#define USB_OTG_GCCFG_PWRDWN                     USB_OTG_GCCFG_PWRDWN_Msk      /*!< Power down */
#define USB_OTG_GCCFG_I2CPADEN_Pos               (17U)                         
#define USB_OTG_GCCFG_I2CPADEN_Msk               (0x1U << USB_OTG_GCCFG_I2CPADEN_Pos) /*!< 0x00020000 */
#define USB_OTG_GCCFG_I2CPADEN                   USB_OTG_GCCFG_I2CPADEN_Msk    /*!< Enable I2C bus connection for the external I2C PHY interface*/ 
#define USB_OTG_GCCFG_VBUSASEN_Pos               (18U)                         
#define USB_OTG_GCCFG_VBUSASEN_Msk               (0x1U << USB_OTG_GCCFG_VBUSASEN_Pos) /*!< 0x00040000 */
#define USB_OTG_GCCFG_VBUSASEN                   USB_OTG_GCCFG_VBUSASEN_Msk    /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_VBUSBSEN_Pos               (19U)                         
#define USB_OTG_GCCFG_VBUSBSEN_Msk               (0x1U << USB_OTG_GCCFG_VBUSBSEN_Pos) /*!< 0x00080000 */
#define USB_OTG_GCCFG_VBUSBSEN                   USB_OTG_GCCFG_VBUSBSEN_Msk    /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_SOFOUTEN_Pos               (20U)                         
#define USB_OTG_GCCFG_SOFOUTEN_Msk               (0x1U << USB_OTG_GCCFG_SOFOUTEN_Pos) /*!< 0x00100000 */
#define USB_OTG_GCCFG_SOFOUTEN                   USB_OTG_GCCFG_SOFOUTEN_Msk    /*!< SOF output enable */
#define USB_OTG_GCCFG_NOVBUSSENS_Pos             (21U)                         
#define USB_OTG_GCCFG_NOVBUSSENS_Msk             (0x1U << USB_OTG_GCCFG_NOVBUSSENS_Pos) /*!< 0x00200000 */
#define USB_OTG_GCCFG_NOVBUSSENS                 USB_OTG_GCCFG_NOVBUSSENS_Msk  /*!< VBUS sensing disable option*/ 

/********************  Bit definition forUSB_OTG_DEACHINTMSK register  ********************/
#define USB_OTG_DEACHINTMSK_IEP1INTM_Pos         (1U)                          
#define USB_OTG_DEACHINTMSK_IEP1INTM_Msk         (0x1U << USB_OTG_DEACHINTMSK_IEP1INTM_Pos) /*!< 0x00000002 */
#define USB_OTG_DEACHINTMSK_IEP1INTM             USB_OTG_DEACHINTMSK_IEP1INTM_Msk /*!< IN Endpoint 1 interrupt mask bit  */
#define USB_OTG_DEACHINTMSK_OEP1INTM_Pos         (17U)                         
#define USB_OTG_DEACHINTMSK_OEP1INTM_Msk         (0x1U << USB_OTG_DEACHINTMSK_OEP1INTM_Pos) /*!< 0x00020000 */
#define USB_OTG_DEACHINTMSK_OEP1INTM             USB_OTG_DEACHINTMSK_OEP1INTM_Msk /*!< OUT Endpoint 1 interrupt mask bit */

/********************  Bit definition for USB_OTG_CID register  ********************/
#define USB_OTG_CID_PRODUCT_ID_Pos               (0U)                          
#define USB_OTG_CID_PRODUCT_ID_Msk               (0xFFFFFFFFU << USB_OTG_CID_PRODUCT_ID_Pos) /*!< 0xFFFFFFFF */
#define USB_OTG_CID_PRODUCT_ID                   USB_OTG_CID_PRODUCT_ID_Msk    /*!< Product ID field */

/********************  Bit definition for USB_OTG_DIEPEACHMSK1 register  ********************/
#define USB_OTG_DIEPEACHMSK1_XFRCM_Pos           (0U)                          
#define USB_OTG_DIEPEACHMSK1_XFRCM_Msk           (0x1U << USB_OTG_DIEPEACHMSK1_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DIEPEACHMSK1_XFRCM               USB_OTG_DIEPEACHMSK1_XFRCM_Msk /*!< Transfer completed interrupt mask                 */
#define USB_OTG_DIEPEACHMSK1_EPDM_Pos            (1U)                          
#define USB_OTG_DIEPEACHMSK1_EPDM_Msk            (0x1U << USB_OTG_DIEPEACHMSK1_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DIEPEACHMSK1_EPDM                USB_OTG_DIEPEACHMSK1_EPDM_Msk /*!< Endpoint disabled interrupt mask                  */
#define USB_OTG_DIEPEACHMSK1_TOM_Pos             (3U)                          
#define USB_OTG_DIEPEACHMSK1_TOM_Msk             (0x1U << USB_OTG_DIEPEACHMSK1_TOM_Pos) /*!< 0x00000008 */
#define USB_OTG_DIEPEACHMSK1_TOM                 USB_OTG_DIEPEACHMSK1_TOM_Msk  /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Pos       (4U)                          
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Msk       (0x1U << USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Pos) /*!< 0x00000010 */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK           USB_OTG_DIEPEACHMSK1_ITTXFEMSK_Msk /*!< IN token received when TxFIFO empty mask          */
#define USB_OTG_DIEPEACHMSK1_INEPNMM_Pos         (5U)                          
#define USB_OTG_DIEPEACHMSK1_INEPNMM_Msk         (0x1U << USB_OTG_DIEPEACHMSK1_INEPNMM_Pos) /*!< 0x00000020 */
#define USB_OTG_DIEPEACHMSK1_INEPNMM             USB_OTG_DIEPEACHMSK1_INEPNMM_Msk /*!< IN token received with EP mismatch mask           */
#define USB_OTG_DIEPEACHMSK1_INEPNEM_Pos         (6U)                          
#define USB_OTG_DIEPEACHMSK1_INEPNEM_Msk         (0x1U << USB_OTG_DIEPEACHMSK1_INEPNEM_Pos) /*!< 0x00000040 */
#define USB_OTG_DIEPEACHMSK1_INEPNEM             USB_OTG_DIEPEACHMSK1_INEPNEM_Msk /*!< IN endpoint NAK effective mask                    */
#define USB_OTG_DIEPEACHMSK1_TXFURM_Pos          (8U)                          
#define USB_OTG_DIEPEACHMSK1_TXFURM_Msk          (0x1U << USB_OTG_DIEPEACHMSK1_TXFURM_Pos) /*!< 0x00000100 */
#define USB_OTG_DIEPEACHMSK1_TXFURM              USB_OTG_DIEPEACHMSK1_TXFURM_Msk /*!< FIFO underrun mask                                */
#define USB_OTG_DIEPEACHMSK1_BIM_Pos             (9U)                          
#define USB_OTG_DIEPEACHMSK1_BIM_Msk             (0x1U << USB_OTG_DIEPEACHMSK1_BIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DIEPEACHMSK1_BIM                 USB_OTG_DIEPEACHMSK1_BIM_Msk  /*!< BNA interrupt mask                                */
#define USB_OTG_DIEPEACHMSK1_NAKM_Pos            (13U)                         
#define USB_OTG_DIEPEACHMSK1_NAKM_Msk            (0x1U << USB_OTG_DIEPEACHMSK1_NAKM_Pos) /*!< 0x00002000 */
#define USB_OTG_DIEPEACHMSK1_NAKM                USB_OTG_DIEPEACHMSK1_NAKM_Msk /*!< NAK interrupt mask                                */

/********************  Bit definition for USB_OTG_HPRT register  ********************/
#define USB_OTG_HPRT_PCSTS_Pos                   (0U)                          
#define USB_OTG_HPRT_PCSTS_Msk                   (0x1U << USB_OTG_HPRT_PCSTS_Pos) /*!< 0x00000001 */
#define USB_OTG_HPRT_PCSTS                       USB_OTG_HPRT_PCSTS_Msk        /*!< Port connect status        */
#define USB_OTG_HPRT_PCDET_Pos                   (1U)                          
#define USB_OTG_HPRT_PCDET_Msk                   (0x1U << USB_OTG_HPRT_PCDET_Pos) /*!< 0x00000002 */
#define USB_OTG_HPRT_PCDET                       USB_OTG_HPRT_PCDET_Msk        /*!< Port connect detected      */
#define USB_OTG_HPRT_PENA_Pos                    (2U)                          
#define USB_OTG_HPRT_PENA_Msk                    (0x1U << USB_OTG_HPRT_PENA_Pos) /*!< 0x00000004 */
#define USB_OTG_HPRT_PENA                        USB_OTG_HPRT_PENA_Msk         /*!< Port enable                */
#define USB_OTG_HPRT_PENCHNG_Pos                 (3U)                          
#define USB_OTG_HPRT_PENCHNG_Msk                 (0x1U << USB_OTG_HPRT_PENCHNG_Pos) /*!< 0x00000008 */
#define USB_OTG_HPRT_PENCHNG                     USB_OTG_HPRT_PENCHNG_Msk      /*!< Port enable/disable change */
#define USB_OTG_HPRT_POCA_Pos                    (4U)                          
#define USB_OTG_HPRT_POCA_Msk                    (0x1U << USB_OTG_HPRT_POCA_Pos) /*!< 0x00000010 */
#define USB_OTG_HPRT_POCA                        USB_OTG_HPRT_POCA_Msk         /*!< Port overcurrent active    */
#define USB_OTG_HPRT_POCCHNG_Pos                 (5U)                          
#define USB_OTG_HPRT_POCCHNG_Msk                 (0x1U << USB_OTG_HPRT_POCCHNG_Pos) /*!< 0x00000020 */
#define USB_OTG_HPRT_POCCHNG                     USB_OTG_HPRT_POCCHNG_Msk      /*!< Port overcurrent change    */
#define USB_OTG_HPRT_PRES_Pos                    (6U)                          
#define USB_OTG_HPRT_PRES_Msk                    (0x1U << USB_OTG_HPRT_PRES_Pos) /*!< 0x00000040 */
#define USB_OTG_HPRT_PRES                        USB_OTG_HPRT_PRES_Msk         /*!< Port resume                */
#define USB_OTG_HPRT_PSUSP_Pos                   (7U)                          
#define USB_OTG_HPRT_PSUSP_Msk                   (0x1U << USB_OTG_HPRT_PSUSP_Pos) /*!< 0x00000080 */
#define USB_OTG_HPRT_PSUSP                       USB_OTG_HPRT_PSUSP_Msk        /*!< Port suspend               */
#define USB_OTG_HPRT_PRST_Pos                    (8U)                          
#define USB_OTG_HPRT_PRST_Msk                    (0x1U << USB_OTG_HPRT_PRST_Pos) /*!< 0x00000100 */
#define USB_OTG_HPRT_PRST                        USB_OTG_HPRT_PRST_Msk         /*!< Port reset                 */

#define USB_OTG_HPRT_PLSTS_Pos                   (10U)                         
#define USB_OTG_HPRT_PLSTS_Msk                   (0x3U << USB_OTG_HPRT_PLSTS_Pos) /*!< 0x00000C00 */
#define USB_OTG_HPRT_PLSTS                       USB_OTG_HPRT_PLSTS_Msk        /*!< Port line status           */
#define USB_OTG_HPRT_PLSTS_0                     (0x1U << USB_OTG_HPRT_PLSTS_Pos) /*!< 0x00000400 */
#define USB_OTG_HPRT_PLSTS_1                     (0x2U << USB_OTG_HPRT_PLSTS_Pos) /*!< 0x00000800 */
#define USB_OTG_HPRT_PPWR_Pos                    (12U)                         
#define USB_OTG_HPRT_PPWR_Msk                    (0x1U << USB_OTG_HPRT_PPWR_Pos) /*!< 0x00001000 */
#define USB_OTG_HPRT_PPWR                        USB_OTG_HPRT_PPWR_Msk         /*!< Port power                 */

#define USB_OTG_HPRT_PTCTL_Pos                   (13U)                         
#define USB_OTG_HPRT_PTCTL_Msk                   (0xFU << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x0001E000 */
#define USB_OTG_HPRT_PTCTL                       USB_OTG_HPRT_PTCTL_Msk        /*!< Port test control          */
#define USB_OTG_HPRT_PTCTL_0                     (0x1U << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00002000 */
#define USB_OTG_HPRT_PTCTL_1                     (0x2U << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00004000 */
#define USB_OTG_HPRT_PTCTL_2                     (0x4U << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00008000 */
#define USB_OTG_HPRT_PTCTL_3                     (0x8U << USB_OTG_HPRT_PTCTL_Pos) /*!< 0x00010000 */

#define USB_OTG_HPRT_PSPD_Pos                    (17U)                         
#define USB_OTG_HPRT_PSPD_Msk                    (0x3U << USB_OTG_HPRT_PSPD_Pos) /*!< 0x00060000 */
#define USB_OTG_HPRT_PSPD                        USB_OTG_HPRT_PSPD_Msk         /*!< Port speed                 */
#define USB_OTG_HPRT_PSPD_0                      (0x1U << USB_OTG_HPRT_PSPD_Pos) /*!< 0x00020000 */
#define USB_OTG_HPRT_PSPD_1                      (0x2U << USB_OTG_HPRT_PSPD_Pos) /*!< 0x00040000 */

/********************  Bit definition for USB_OTG_DOEPEACHMSK1 register  ********************/
#define USB_OTG_DOEPEACHMSK1_XFRCM_Pos           (0U)                          
#define USB_OTG_DOEPEACHMSK1_XFRCM_Msk           (0x1U << USB_OTG_DOEPEACHMSK1_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_DOEPEACHMSK1_XFRCM               USB_OTG_DOEPEACHMSK1_XFRCM_Msk /*!< Transfer completed interrupt mask         */
#define USB_OTG_DOEPEACHMSK1_EPDM_Pos            (1U)                          
#define USB_OTG_DOEPEACHMSK1_EPDM_Msk            (0x1U << USB_OTG_DOEPEACHMSK1_EPDM_Pos) /*!< 0x00000002 */
#define USB_OTG_DOEPEACHMSK1_EPDM                USB_OTG_DOEPEACHMSK1_EPDM_Msk /*!< Endpoint disabled interrupt mask          */
#define USB_OTG_DOEPEACHMSK1_TOM_Pos             (3U)                          
#define USB_OTG_DOEPEACHMSK1_TOM_Msk             (0x1U << USB_OTG_DOEPEACHMSK1_TOM_Pos) /*!< 0x00000008 */
#define USB_OTG_DOEPEACHMSK1_TOM                 USB_OTG_DOEPEACHMSK1_TOM_Msk  /*!< Timeout condition mask                    */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Pos       (4U)                          
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Msk       (0x1U << USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Pos) /*!< 0x00000010 */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK           USB_OTG_DOEPEACHMSK1_ITTXFEMSK_Msk /*!< IN token received when TxFIFO empty mask  */
#define USB_OTG_DOEPEACHMSK1_INEPNMM_Pos         (5U)                          
#define USB_OTG_DOEPEACHMSK1_INEPNMM_Msk         (0x1U << USB_OTG_DOEPEACHMSK1_INEPNMM_Pos) /*!< 0x00000020 */
#define USB_OTG_DOEPEACHMSK1_INEPNMM             USB_OTG_DOEPEACHMSK1_INEPNMM_Msk /*!< IN token received with EP mismatch mask   */
#define USB_OTG_DOEPEACHMSK1_INEPNEM_Pos         (6U)                          
#define USB_OTG_DOEPEACHMSK1_INEPNEM_Msk         (0x1U << USB_OTG_DOEPEACHMSK1_INEPNEM_Pos) /*!< 0x00000040 */
#define USB_OTG_DOEPEACHMSK1_INEPNEM             USB_OTG_DOEPEACHMSK1_INEPNEM_Msk /*!< IN endpoint NAK effective mask            */
#define USB_OTG_DOEPEACHMSK1_TXFURM_Pos          (8U)                          
#define USB_OTG_DOEPEACHMSK1_TXFURM_Msk          (0x1U << USB_OTG_DOEPEACHMSK1_TXFURM_Pos) /*!< 0x00000100 */
#define USB_OTG_DOEPEACHMSK1_TXFURM              USB_OTG_DOEPEACHMSK1_TXFURM_Msk /*!< OUT packet error mask                     */
#define USB_OTG_DOEPEACHMSK1_BIM_Pos             (9U)                          
#define USB_OTG_DOEPEACHMSK1_BIM_Msk             (0x1U << USB_OTG_DOEPEACHMSK1_BIM_Pos) /*!< 0x00000200 */
#define USB_OTG_DOEPEACHMSK1_BIM                 USB_OTG_DOEPEACHMSK1_BIM_Msk  /*!< BNA interrupt mask                        */
#define USB_OTG_DOEPEACHMSK1_BERRM_Pos           (12U)                         
#define USB_OTG_DOEPEACHMSK1_BERRM_Msk           (0x1U << USB_OTG_DOEPEACHMSK1_BERRM_Pos) /*!< 0x00001000 */
#define USB_OTG_DOEPEACHMSK1_BERRM               USB_OTG_DOEPEACHMSK1_BERRM_Msk /*!< Bubble error interrupt mask               */
#define USB_OTG_DOEPEACHMSK1_NAKM_Pos            (13U)                         
#define USB_OTG_DOEPEACHMSK1_NAKM_Msk            (0x1U << USB_OTG_DOEPEACHMSK1_NAKM_Pos) /*!< 0x00002000 */
#define USB_OTG_DOEPEACHMSK1_NAKM                USB_OTG_DOEPEACHMSK1_NAKM_Msk /*!< NAK interrupt mask                        */
#define USB_OTG_DOEPEACHMSK1_NYETM_Pos           (14U)                         
#define USB_OTG_DOEPEACHMSK1_NYETM_Msk           (0x1U << USB_OTG_DOEPEACHMSK1_NYETM_Pos) /*!< 0x00004000 */
#define USB_OTG_DOEPEACHMSK1_NYETM               USB_OTG_DOEPEACHMSK1_NYETM_Msk /*!< NYET interrupt mask                       */

/********************  Bit definition for USB_OTG_HPTXFSIZ register  ********************/
#define USB_OTG_HPTXFSIZ_PTXSA_Pos               (0U)                          
#define USB_OTG_HPTXFSIZ_PTXSA_Msk               (0xFFFFU << USB_OTG_HPTXFSIZ_PTXSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_HPTXFSIZ_PTXSA                   USB_OTG_HPTXFSIZ_PTXSA_Msk    /*!< Host periodic TxFIFO start address            */
#define USB_OTG_HPTXFSIZ_PTXFD_Pos               (16U)                         
#define USB_OTG_HPTXFSIZ_PTXFD_Msk               (0xFFFFU << USB_OTG_HPTXFSIZ_PTXFD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_HPTXFSIZ_PTXFD                   USB_OTG_HPTXFSIZ_PTXFD_Msk    /*!< Host periodic TxFIFO depth                    */

/********************  Bit definition for USB_OTG_DIEPCTL register  ********************/
#define USB_OTG_DIEPCTL_MPSIZ_Pos                (0U)                          
#define USB_OTG_DIEPCTL_MPSIZ_Msk                (0x7FFU << USB_OTG_DIEPCTL_MPSIZ_Pos) /*!< 0x000007FF */
#define USB_OTG_DIEPCTL_MPSIZ                    USB_OTG_DIEPCTL_MPSIZ_Msk     /*!< Maximum packet size              */
#define USB_OTG_DIEPCTL_USBAEP_Pos               (15U)                         
#define USB_OTG_DIEPCTL_USBAEP_Msk               (0x1U << USB_OTG_DIEPCTL_USBAEP_Pos) /*!< 0x00008000 */
#define USB_OTG_DIEPCTL_USBAEP                   USB_OTG_DIEPCTL_USBAEP_Msk    /*!< USB active endpoint              */
#define USB_OTG_DIEPCTL_EONUM_DPID_Pos           (16U)                         
#define USB_OTG_DIEPCTL_EONUM_DPID_Msk           (0x1U << USB_OTG_DIEPCTL_EONUM_DPID_Pos) /*!< 0x00010000 */
#define USB_OTG_DIEPCTL_EONUM_DPID               USB_OTG_DIEPCTL_EONUM_DPID_Msk /*!< Even/odd frame                   */
#define USB_OTG_DIEPCTL_NAKSTS_Pos               (17U)                         
#define USB_OTG_DIEPCTL_NAKSTS_Msk               (0x1U << USB_OTG_DIEPCTL_NAKSTS_Pos) /*!< 0x00020000 */
#define USB_OTG_DIEPCTL_NAKSTS                   USB_OTG_DIEPCTL_NAKSTS_Msk    /*!< NAK status                       */

#define USB_OTG_DIEPCTL_EPTYP_Pos                (18U)                         
#define USB_OTG_DIEPCTL_EPTYP_Msk                (0x3U << USB_OTG_DIEPCTL_EPTYP_Pos) /*!< 0x000C0000 */
#define USB_OTG_DIEPCTL_EPTYP                    USB_OTG_DIEPCTL_EPTYP_Msk     /*!< Endpoint type                    */
#define USB_OTG_DIEPCTL_EPTYP_0                  (0x1U << USB_OTG_DIEPCTL_EPTYP_Pos) /*!< 0x00040000 */
#define USB_OTG_DIEPCTL_EPTYP_1                  (0x2U << USB_OTG_DIEPCTL_EPTYP_Pos) /*!< 0x00080000 */
#define USB_OTG_DIEPCTL_STALL_Pos                (21U)                         
#define USB_OTG_DIEPCTL_STALL_Msk                (0x1U << USB_OTG_DIEPCTL_STALL_Pos) /*!< 0x00200000 */
#define USB_OTG_DIEPCTL_STALL                    USB_OTG_DIEPCTL_STALL_Msk     /*!< STALL handshake                  */

#define USB_OTG_DIEPCTL_TXFNUM_Pos               (22U)                         
#define USB_OTG_DIEPCTL_TXFNUM_Msk               (0xFU << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x03C00000 */
#define USB_OTG_DIEPCTL_TXFNUM                   USB_OTG_DIEPCTL_TXFNUM_Msk    /*!< TxFIFO number                    */
#define USB_OTG_DIEPCTL_TXFNUM_0                 (0x1U << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x00400000 */
#define USB_OTG_DIEPCTL_TXFNUM_1                 (0x2U << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x00800000 */
#define USB_OTG_DIEPCTL_TXFNUM_2                 (0x4U << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x01000000 */
#define USB_OTG_DIEPCTL_TXFNUM_3                 (0x8U << USB_OTG_DIEPCTL_TXFNUM_Pos) /*!< 0x02000000 */
#define USB_OTG_DIEPCTL_CNAK_Pos                 (26U)                         
#define USB_OTG_DIEPCTL_CNAK_Msk                 (0x1U << USB_OTG_DIEPCTL_CNAK_Pos) /*!< 0x04000000 */
#define USB_OTG_DIEPCTL_CNAK                     USB_OTG_DIEPCTL_CNAK_Msk      /*!< Clear NAK                        */
#define USB_OTG_DIEPCTL_SNAK_Pos                 (27U)                         
#define USB_OTG_DIEPCTL_SNAK_Msk                 (0x1U << USB_OTG_DIEPCTL_SNAK_Pos) /*!< 0x08000000 */
#define USB_OTG_DIEPCTL_SNAK                     USB_OTG_DIEPCTL_SNAK_Msk      /*!< Set NAK */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Pos       (28U)                         
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk       (0x1U << USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Pos) /*!< 0x10000000 */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM           USB_OTG_DIEPCTL_SD0PID_SEVNFRM_Msk /*!< Set DATA0 PID                    */
#define USB_OTG_DIEPCTL_SODDFRM_Pos              (29U)                         
#define USB_OTG_DIEPCTL_SODDFRM_Msk              (0x1U << USB_OTG_DIEPCTL_SODDFRM_Pos) /*!< 0x20000000 */
#define USB_OTG_DIEPCTL_SODDFRM                  USB_OTG_DIEPCTL_SODDFRM_Msk   /*!< Set odd frame                    */
#define USB_OTG_DIEPCTL_EPDIS_Pos                (30U)                         
#define USB_OTG_DIEPCTL_EPDIS_Msk                (0x1U << USB_OTG_DIEPCTL_EPDIS_Pos) /*!< 0x40000000 */
#define USB_OTG_DIEPCTL_EPDIS                    USB_OTG_DIEPCTL_EPDIS_Msk     /*!< Endpoint disable                 */
#define USB_OTG_DIEPCTL_EPENA_Pos                (31U)                         
#define USB_OTG_DIEPCTL_EPENA_Msk                (0x1U << USB_OTG_DIEPCTL_EPENA_Pos) /*!< 0x80000000 */
#define USB_OTG_DIEPCTL_EPENA                    USB_OTG_DIEPCTL_EPENA_Msk     /*!< Endpoint enable                  */

/********************  Bit definition for USB_OTG_HCCHAR register  ********************/
#define USB_OTG_HCCHAR_MPSIZ_Pos                 (0U)                          
#define USB_OTG_HCCHAR_MPSIZ_Msk                 (0x7FFU << USB_OTG_HCCHAR_MPSIZ_Pos) /*!< 0x000007FF */
#define USB_OTG_HCCHAR_MPSIZ                     USB_OTG_HCCHAR_MPSIZ_Msk      /*!< Maximum packet size */

#define USB_OTG_HCCHAR_EPNUM_Pos                 (11U)                         
#define USB_OTG_HCCHAR_EPNUM_Msk                 (0xFU << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00007800 */
#define USB_OTG_HCCHAR_EPNUM                     USB_OTG_HCCHAR_EPNUM_Msk      /*!< Endpoint number */
#define USB_OTG_HCCHAR_EPNUM_0                   (0x1U << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00000800 */
#define USB_OTG_HCCHAR_EPNUM_1                   (0x2U << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00001000 */
#define USB_OTG_HCCHAR_EPNUM_2                   (0x4U << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00002000 */
#define USB_OTG_HCCHAR_EPNUM_3                   (0x8U << USB_OTG_HCCHAR_EPNUM_Pos) /*!< 0x00004000 */
#define USB_OTG_HCCHAR_EPDIR_Pos                 (15U)                         
#define USB_OTG_HCCHAR_EPDIR_Msk                 (0x1U << USB_OTG_HCCHAR_EPDIR_Pos) /*!< 0x00008000 */
#define USB_OTG_HCCHAR_EPDIR                     USB_OTG_HCCHAR_EPDIR_Msk      /*!< Endpoint direction */
#define USB_OTG_HCCHAR_LSDEV_Pos                 (17U)                         
#define USB_OTG_HCCHAR_LSDEV_Msk                 (0x1U << USB_OTG_HCCHAR_LSDEV_Pos) /*!< 0x00020000 */
#define USB_OTG_HCCHAR_LSDEV                     USB_OTG_HCCHAR_LSDEV_Msk      /*!< Low-speed device */

#define USB_OTG_HCCHAR_EPTYP_Pos                 (18U)                         
#define USB_OTG_HCCHAR_EPTYP_Msk                 (0x3U << USB_OTG_HCCHAR_EPTYP_Pos) /*!< 0x000C0000 */
#define USB_OTG_HCCHAR_EPTYP                     USB_OTG_HCCHAR_EPTYP_Msk      /*!< Endpoint type */
#define USB_OTG_HCCHAR_EPTYP_0                   (0x1U << USB_OTG_HCCHAR_EPTYP_Pos) /*!< 0x00040000 */
#define USB_OTG_HCCHAR_EPTYP_1                   (0x2U << USB_OTG_HCCHAR_EPTYP_Pos) /*!< 0x00080000 */

#define USB_OTG_HCCHAR_MC_Pos                    (20U)                         
#define USB_OTG_HCCHAR_MC_Msk                    (0x3U << USB_OTG_HCCHAR_MC_Pos) /*!< 0x00300000 */
#define USB_OTG_HCCHAR_MC                        USB_OTG_HCCHAR_MC_Msk         /*!< Multi Count (MC) / Error Count (EC) */
#define USB_OTG_HCCHAR_MC_0                      (0x1U << USB_OTG_HCCHAR_MC_Pos) /*!< 0x00100000 */
#define USB_OTG_HCCHAR_MC_1                      (0x2U << USB_OTG_HCCHAR_MC_Pos) /*!< 0x00200000 */

#define USB_OTG_HCCHAR_DAD_Pos                   (22U)                         
#define USB_OTG_HCCHAR_DAD_Msk                   (0x7FU << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x1FC00000 */
#define USB_OTG_HCCHAR_DAD                       USB_OTG_HCCHAR_DAD_Msk        /*!< Device address */
#define USB_OTG_HCCHAR_DAD_0                     (0x01U << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x00400000 */
#define USB_OTG_HCCHAR_DAD_1                     (0x02U << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x00800000 */
#define USB_OTG_HCCHAR_DAD_2                     (0x04U << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x01000000 */
#define USB_OTG_HCCHAR_DAD_3                     (0x08U << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x02000000 */
#define USB_OTG_HCCHAR_DAD_4                     (0x10U << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x04000000 */
#define USB_OTG_HCCHAR_DAD_5                     (0x20U << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x08000000 */
#define USB_OTG_HCCHAR_DAD_6                     (0x40U << USB_OTG_HCCHAR_DAD_Pos) /*!< 0x10000000 */
#define USB_OTG_HCCHAR_ODDFRM_Pos                (29U)                         
#define USB_OTG_HCCHAR_ODDFRM_Msk                (0x1U << USB_OTG_HCCHAR_ODDFRM_Pos) /*!< 0x20000000 */
#define USB_OTG_HCCHAR_ODDFRM                    USB_OTG_HCCHAR_ODDFRM_Msk     /*!< Odd frame */
#define USB_OTG_HCCHAR_CHDIS_Pos                 (30U)                         
#define USB_OTG_HCCHAR_CHDIS_Msk                 (0x1U << USB_OTG_HCCHAR_CHDIS_Pos) /*!< 0x40000000 */
#define USB_OTG_HCCHAR_CHDIS                     USB_OTG_HCCHAR_CHDIS_Msk      /*!< Channel disable */
#define USB_OTG_HCCHAR_CHENA_Pos                 (31U)                         
#define USB_OTG_HCCHAR_CHENA_Msk                 (0x1U << USB_OTG_HCCHAR_CHENA_Pos) /*!< 0x80000000 */
#define USB_OTG_HCCHAR_CHENA                     USB_OTG_HCCHAR_CHENA_Msk      /*!< Channel enable */

/********************  Bit definition for USB_OTG_HCSPLT register  ********************/

#define USB_OTG_HCSPLT_PRTADDR_Pos               (0U)                          
#define USB_OTG_HCSPLT_PRTADDR_Msk               (0x7FU << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x0000007F */
#define USB_OTG_HCSPLT_PRTADDR                   USB_OTG_HCSPLT_PRTADDR_Msk    /*!< Port address */
#define USB_OTG_HCSPLT_PRTADDR_0                 (0x01U << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000001 */
#define USB_OTG_HCSPLT_PRTADDR_1                 (0x02U << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000002 */
#define USB_OTG_HCSPLT_PRTADDR_2                 (0x04U << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000004 */
#define USB_OTG_HCSPLT_PRTADDR_3                 (0x08U << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000008 */
#define USB_OTG_HCSPLT_PRTADDR_4                 (0x10U << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000010 */
#define USB_OTG_HCSPLT_PRTADDR_5                 (0x20U << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000020 */
#define USB_OTG_HCSPLT_PRTADDR_6                 (0x40U << USB_OTG_HCSPLT_PRTADDR_Pos) /*!< 0x00000040 */

#define USB_OTG_HCSPLT_HUBADDR_Pos               (7U)                          
#define USB_OTG_HCSPLT_HUBADDR_Msk               (0x7FU << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00003F80 */
#define USB_OTG_HCSPLT_HUBADDR                   USB_OTG_HCSPLT_HUBADDR_Msk    /*!< Hub address */
#define USB_OTG_HCSPLT_HUBADDR_0                 (0x01U << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000080 */
#define USB_OTG_HCSPLT_HUBADDR_1                 (0x02U << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000100 */
#define USB_OTG_HCSPLT_HUBADDR_2                 (0x04U << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000200 */
#define USB_OTG_HCSPLT_HUBADDR_3                 (0x08U << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000400 */
#define USB_OTG_HCSPLT_HUBADDR_4                 (0x10U << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00000800 */
#define USB_OTG_HCSPLT_HUBADDR_5                 (0x20U << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00001000 */
#define USB_OTG_HCSPLT_HUBADDR_6                 (0x40U << USB_OTG_HCSPLT_HUBADDR_Pos) /*!< 0x00002000 */

#define USB_OTG_HCSPLT_XACTPOS_Pos               (14U)                         
#define USB_OTG_HCSPLT_XACTPOS_Msk               (0x3U << USB_OTG_HCSPLT_XACTPOS_Pos) /*!< 0x0000C000 */
#define USB_OTG_HCSPLT_XACTPOS                   USB_OTG_HCSPLT_XACTPOS_Msk    /*!< XACTPOS */
#define USB_OTG_HCSPLT_XACTPOS_0                 (0x1U << USB_OTG_HCSPLT_XACTPOS_Pos) /*!< 0x00004000 */
#define USB_OTG_HCSPLT_XACTPOS_1                 (0x2U << USB_OTG_HCSPLT_XACTPOS_Pos) /*!< 0x00008000 */
#define USB_OTG_HCSPLT_COMPLSPLT_Pos             (16U)                         
#define USB_OTG_HCSPLT_COMPLSPLT_Msk             (0x1U << USB_OTG_HCSPLT_COMPLSPLT_Pos) /*!< 0x00010000 */
#define USB_OTG_HCSPLT_COMPLSPLT                 USB_OTG_HCSPLT_COMPLSPLT_Msk  /*!< Do complete split */
#define USB_OTG_HCSPLT_SPLITEN_Pos               (31U)                         
#define USB_OTG_HCSPLT_SPLITEN_Msk               (0x1U << USB_OTG_HCSPLT_SPLITEN_Pos) /*!< 0x80000000 */
#define USB_OTG_HCSPLT_SPLITEN                   USB_OTG_HCSPLT_SPLITEN_Msk    /*!< Split enable */

/********************  Bit definition for USB_OTG_HCINT register  ********************/
#define USB_OTG_HCINT_XFRC_Pos                   (0U)                          
#define USB_OTG_HCINT_XFRC_Msk                   (0x1U << USB_OTG_HCINT_XFRC_Pos) /*!< 0x00000001 */
#define USB_OTG_HCINT_XFRC                       USB_OTG_HCINT_XFRC_Msk        /*!< Transfer completed */
#define USB_OTG_HCINT_CHH_Pos                    (1U)                          
#define USB_OTG_HCINT_CHH_Msk                    (0x1U << USB_OTG_HCINT_CHH_Pos) /*!< 0x00000002 */
#define USB_OTG_HCINT_CHH                        USB_OTG_HCINT_CHH_Msk         /*!< Channel halted */
#define USB_OTG_HCINT_AHBERR_Pos                 (2U)                          
#define USB_OTG_HCINT_AHBERR_Msk                 (0x1U << USB_OTG_HCINT_AHBERR_Pos) /*!< 0x00000004 */
#define USB_OTG_HCINT_AHBERR                     USB_OTG_HCINT_AHBERR_Msk      /*!< AHB error */
#define USB_OTG_HCINT_STALL_Pos                  (3U)                          
#define USB_OTG_HCINT_STALL_Msk                  (0x1U << USB_OTG_HCINT_STALL_Pos) /*!< 0x00000008 */
#define USB_OTG_HCINT_STALL                      USB_OTG_HCINT_STALL_Msk       /*!< STALL response received interrupt */
#define USB_OTG_HCINT_NAK_Pos                    (4U)                          
#define USB_OTG_HCINT_NAK_Msk                    (0x1U << USB_OTG_HCINT_NAK_Pos) /*!< 0x00000010 */
#define USB_OTG_HCINT_NAK                        USB_OTG_HCINT_NAK_Msk         /*!< NAK response received interrupt */
#define USB_OTG_HCINT_ACK_Pos                    (5U)                          
#define USB_OTG_HCINT_ACK_Msk                    (0x1U << USB_OTG_HCINT_ACK_Pos) /*!< 0x00000020 */
#define USB_OTG_HCINT_ACK                        USB_OTG_HCINT_ACK_Msk         /*!< ACK response received/transmitted interrupt */
#define USB_OTG_HCINT_NYET_Pos                   (6U)                          
#define USB_OTG_HCINT_NYET_Msk                   (0x1U << USB_OTG_HCINT_NYET_Pos) /*!< 0x00000040 */
#define USB_OTG_HCINT_NYET                       USB_OTG_HCINT_NYET_Msk        /*!< Response received interrupt */
#define USB_OTG_HCINT_TXERR_Pos                  (7U)                          
#define USB_OTG_HCINT_TXERR_Msk                  (0x1U << USB_OTG_HCINT_TXERR_Pos) /*!< 0x00000080 */
#define USB_OTG_HCINT_TXERR                      USB_OTG_HCINT_TXERR_Msk       /*!< Transaction error */
#define USB_OTG_HCINT_BBERR_Pos                  (8U)                          
#define USB_OTG_HCINT_BBERR_Msk                  (0x1U << USB_OTG_HCINT_BBERR_Pos) /*!< 0x00000100 */
#define USB_OTG_HCINT_BBERR                      USB_OTG_HCINT_BBERR_Msk       /*!< Babble error */
#define USB_OTG_HCINT_FRMOR_Pos                  (9U)                          
#define USB_OTG_HCINT_FRMOR_Msk                  (0x1U << USB_OTG_HCINT_FRMOR_Pos) /*!< 0x00000200 */
#define USB_OTG_HCINT_FRMOR                      USB_OTG_HCINT_FRMOR_Msk       /*!< Frame overrun */
#define USB_OTG_HCINT_DTERR_Pos                  (10U)                         
#define USB_OTG_HCINT_DTERR_Msk                  (0x1U << USB_OTG_HCINT_DTERR_Pos) /*!< 0x00000400 */
#define USB_OTG_HCINT_DTERR                      USB_OTG_HCINT_DTERR_Msk       /*!< Data toggle error */

/********************  Bit definition for USB_OTG_DIEPINT register  ********************/
#define USB_OTG_DIEPINT_XFRC_Pos                 (0U)                          
#define USB_OTG_DIEPINT_XFRC_Msk                 (0x1U << USB_OTG_DIEPINT_XFRC_Pos) /*!< 0x00000001 */
#define USB_OTG_DIEPINT_XFRC                     USB_OTG_DIEPINT_XFRC_Msk      /*!< Transfer completed interrupt */
#define USB_OTG_DIEPINT_EPDISD_Pos               (1U)                          
#define USB_OTG_DIEPINT_EPDISD_Msk               (0x1U << USB_OTG_DIEPINT_EPDISD_Pos) /*!< 0x00000002 */
#define USB_OTG_DIEPINT_EPDISD                   USB_OTG_DIEPINT_EPDISD_Msk    /*!< Endpoint disabled interrupt */
#define USB_OTG_DIEPINT_TOC_Pos                  (3U)                          
#define USB_OTG_DIEPINT_TOC_Msk                  (0x1U << USB_OTG_DIEPINT_TOC_Pos) /*!< 0x00000008 */
#define USB_OTG_DIEPINT_TOC                      USB_OTG_DIEPINT_TOC_Msk       /*!< Timeout condition */
#define USB_OTG_DIEPINT_ITTXFE_Pos               (4U)                          
#define USB_OTG_DIEPINT_ITTXFE_Msk               (0x1U << USB_OTG_DIEPINT_ITTXFE_Pos) /*!< 0x00000010 */
#define USB_OTG_DIEPINT_ITTXFE                   USB_OTG_DIEPINT_ITTXFE_Msk    /*!< IN token received when TxFIFO is empty */
#define USB_OTG_DIEPINT_INEPNE_Pos               (6U)                          
#define USB_OTG_DIEPINT_INEPNE_Msk               (0x1U << USB_OTG_DIEPINT_INEPNE_Pos) /*!< 0x00000040 */
#define USB_OTG_DIEPINT_INEPNE                   USB_OTG_DIEPINT_INEPNE_Msk    /*!< IN endpoint NAK effective */
#define USB_OTG_DIEPINT_TXFE_Pos                 (7U)                          
#define USB_OTG_DIEPINT_TXFE_Msk                 (0x1U << USB_OTG_DIEPINT_TXFE_Pos) /*!< 0x00000080 */
#define USB_OTG_DIEPINT_TXFE                     USB_OTG_DIEPINT_TXFE_Msk      /*!< Transmit FIFO empty */
#define USB_OTG_DIEPINT_TXFIFOUDRN_Pos           (8U)                          
#define USB_OTG_DIEPINT_TXFIFOUDRN_Msk           (0x1U << USB_OTG_DIEPINT_TXFIFOUDRN_Pos) /*!< 0x00000100 */
#define USB_OTG_DIEPINT_TXFIFOUDRN               USB_OTG_DIEPINT_TXFIFOUDRN_Msk /*!< Transmit Fifo Underrun */
#define USB_OTG_DIEPINT_BNA_Pos                  (9U)                          
#define USB_OTG_DIEPINT_BNA_Msk                  (0x1U << USB_OTG_DIEPINT_BNA_Pos) /*!< 0x00000200 */
#define USB_OTG_DIEPINT_BNA                      USB_OTG_DIEPINT_BNA_Msk       /*!< Buffer not available interrupt */
#define USB_OTG_DIEPINT_PKTDRPSTS_Pos            (11U)                         
#define USB_OTG_DIEPINT_PKTDRPSTS_Msk            (0x1U << USB_OTG_DIEPINT_PKTDRPSTS_Pos) /*!< 0x00000800 */
#define USB_OTG_DIEPINT_PKTDRPSTS                USB_OTG_DIEPINT_PKTDRPSTS_Msk /*!< Packet dropped status */
#define USB_OTG_DIEPINT_BERR_Pos                 (12U)                         
#define USB_OTG_DIEPINT_BERR_Msk                 (0x1U << USB_OTG_DIEPINT_BERR_Pos) /*!< 0x00001000 */
#define USB_OTG_DIEPINT_BERR                     USB_OTG_DIEPINT_BERR_Msk      /*!< Babble error interrupt */
#define USB_OTG_DIEPINT_NAK_Pos                  (13U)                         
#define USB_OTG_DIEPINT_NAK_Msk                  (0x1U << USB_OTG_DIEPINT_NAK_Pos) /*!< 0x00002000 */
#define USB_OTG_DIEPINT_NAK                      USB_OTG_DIEPINT_NAK_Msk       /*!< NAK interrupt */

/********************  Bit definition forUSB_OTG_HCINTMSK register  ********************/
#define USB_OTG_HCINTMSK_XFRCM_Pos               (0U)                          
#define USB_OTG_HCINTMSK_XFRCM_Msk               (0x1U << USB_OTG_HCINTMSK_XFRCM_Pos) /*!< 0x00000001 */
#define USB_OTG_HCINTMSK_XFRCM                   USB_OTG_HCINTMSK_XFRCM_Msk    /*!< Transfer completed mask */
#define USB_OTG_HCINTMSK_CHHM_Pos                (1U)                          
#define USB_OTG_HCINTMSK_CHHM_Msk                (0x1U << USB_OTG_HCINTMSK_CHHM_Pos) /*!< 0x00000002 */
#define USB_OTG_HCINTMSK_CHHM                    USB_OTG_HCINTMSK_CHHM_Msk     /*!< Channel halted mask */
#define USB_OTG_HCINTMSK_AHBERR_Pos              (2U)                          
#define USB_OTG_HCINTMSK_AHBERR_Msk              (0x1U << USB_OTG_HCINTMSK_AHBERR_Pos) /*!< 0x00000004 */
#define USB_OTG_HCINTMSK_AHBERR                  USB_OTG_HCINTMSK_AHBERR_Msk   /*!< AHB error */
#define USB_OTG_HCINTMSK_STALLM_Pos              (3U)                          
#define USB_OTG_HCINTMSK_STALLM_Msk              (0x1U << USB_OTG_HCINTMSK_STALLM_Pos) /*!< 0x00000008 */
#define USB_OTG_HCINTMSK_STALLM                  USB_OTG_HCINTMSK_STALLM_Msk   /*!< STALL response received interrupt mask */
#define USB_OTG_HCINTMSK_NAKM_Pos                (4U)                          
#define USB_OTG_HCINTMSK_NAKM_Msk                (0x1U << USB_OTG_HCINTMSK_NAKM_Pos) /*!< 0x00000010 */
#define USB_OTG_HCINTMSK_NAKM                    USB_OTG_HCINTMSK_NAKM_Msk     /*!< NAK response received interrupt mask */
#define USB_OTG_HCINTMSK_ACKM_Pos                (5U)                          
#define USB_OTG_HCINTMSK_ACKM_Msk                (0x1U << USB_OTG_HCINTMSK_ACKM_Pos) /*!< 0x00000020 */
#define USB_OTG_HCINTMSK_ACKM                    USB_OTG_HCINTMSK_ACKM_Msk     /*!< ACK response received/transmitted interrupt mask */
#define USB_OTG_HCINTMSK_NYET_Pos                (6U)                          
#define USB_OTG_HCINTMSK_NYET_Msk                (0x1U << USB_OTG_HCINTMSK_NYET_Pos) /*!< 0x00000040 */
#define USB_OTG_HCINTMSK_NYET                    USB_OTG_HCINTMSK_NYET_Msk     /*!< response received interrupt mask */
#define USB_OTG_HCINTMSK_TXERRM_Pos              (7U)                          
#define USB_OTG_HCINTMSK_TXERRM_Msk              (0x1U << USB_OTG_HCINTMSK_TXERRM_Pos) /*!< 0x00000080 */
#define USB_OTG_HCINTMSK_TXERRM                  USB_OTG_HCINTMSK_TXERRM_Msk   /*!< Transaction error mask */
#define USB_OTG_HCINTMSK_BBERRM_Pos              (8U)                          
#define USB_OTG_HCINTMSK_BBERRM_Msk              (0x1U << USB_OTG_HCINTMSK_BBERRM_Pos) /*!< 0x00000100 */
#define USB_OTG_HCINTMSK_BBERRM                  USB_OTG_HCINTMSK_BBERRM_Msk   /*!< Babble error mask */
#define USB_OTG_HCINTMSK_FRMORM_Pos              (9U)                          
#define USB_OTG_HCINTMSK_FRMORM_Msk              (0x1U << USB_OTG_HCINTMSK_FRMORM_Pos) /*!< 0x00000200 */
#define USB_OTG_HCINTMSK_FRMORM                  USB_OTG_HCINTMSK_FRMORM_Msk   /*!< Frame overrun mask */
#define USB_OTG_HCINTMSK_DTERRM_Pos              (10U)                         
#define USB_OTG_HCINTMSK_DTERRM_Msk              (0x1U << USB_OTG_HCINTMSK_DTERRM_Pos) /*!< 0x00000400 */
#define USB_OTG_HCINTMSK_DTERRM                  USB_OTG_HCINTMSK_DTERRM_Msk   /*!< Data toggle error mask */

/********************  Bit definition for USB_OTG_DIEPTSIZ register  ********************/

#define USB_OTG_DIEPTSIZ_XFRSIZ_Pos              (0U)                          
#define USB_OTG_DIEPTSIZ_XFRSIZ_Msk              (0x7FFFFU << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) /*!< 0x0007FFFF */
#define USB_OTG_DIEPTSIZ_XFRSIZ                  USB_OTG_DIEPTSIZ_XFRSIZ_Msk   /*!< Transfer size */
#define USB_OTG_DIEPTSIZ_PKTCNT_Pos              (19U)                         
#define USB_OTG_DIEPTSIZ_PKTCNT_Msk              (0x3FFU << USB_OTG_DIEPTSIZ_PKTCNT_Pos) /*!< 0x1FF80000 */
#define USB_OTG_DIEPTSIZ_PKTCNT                  USB_OTG_DIEPTSIZ_PKTCNT_Msk   /*!< Packet count */
#define USB_OTG_DIEPTSIZ_MULCNT_Pos              (29U)                         
#define USB_OTG_DIEPTSIZ_MULCNT_Msk              (0x3U << USB_OTG_DIEPTSIZ_MULCNT_Pos) /*!< 0x60000000 */
#define USB_OTG_DIEPTSIZ_MULCNT                  USB_OTG_DIEPTSIZ_MULCNT_Msk   /*!< Packet count */
/********************  Bit definition for USB_OTG_HCTSIZ register  ********************/
#define USB_OTG_HCTSIZ_XFRSIZ_Pos                (0U)                          
#define USB_OTG_HCTSIZ_XFRSIZ_Msk                (0x7FFFFU << USB_OTG_HCTSIZ_XFRSIZ_Pos) /*!< 0x0007FFFF */
#define USB_OTG_HCTSIZ_XFRSIZ                    USB_OTG_HCTSIZ_XFRSIZ_Msk     /*!< Transfer size */
#define USB_OTG_HCTSIZ_PKTCNT_Pos                (19U)                         
#define USB_OTG_HCTSIZ_PKTCNT_Msk                (0x3FFU << USB_OTG_HCTSIZ_PKTCNT_Pos) /*!< 0x1FF80000 */
#define USB_OTG_HCTSIZ_PKTCNT                    USB_OTG_HCTSIZ_PKTCNT_Msk     /*!< Packet count */
#define USB_OTG_HCTSIZ_DOPING_Pos                (31U)                         
#define USB_OTG_HCTSIZ_DOPING_Msk                (0x1U << USB_OTG_HCTSIZ_DOPING_Pos) /*!< 0x80000000 */
#define USB_OTG_HCTSIZ_DOPING                    USB_OTG_HCTSIZ_DOPING_Msk     /*!< Do PING */
#define USB_OTG_HCTSIZ_DPID_Pos                  (29U)                         
#define USB_OTG_HCTSIZ_DPID_Msk                  (0x3U << USB_OTG_HCTSIZ_DPID_Pos) /*!< 0x60000000 */
#define USB_OTG_HCTSIZ_DPID                      USB_OTG_HCTSIZ_DPID_Msk       /*!< Data PID */
#define USB_OTG_HCTSIZ_DPID_0                    (0x1U << USB_OTG_HCTSIZ_DPID_Pos) /*!< 0x20000000 */
#define USB_OTG_HCTSIZ_DPID_1                    (0x2U << USB_OTG_HCTSIZ_DPID_Pos) /*!< 0x40000000 */

/********************  Bit definition for USB_OTG_DIEPDMA register  ********************/
#define USB_OTG_DIEPDMA_DMAADDR_Pos              (0U)                          
#define USB_OTG_DIEPDMA_DMAADDR_Msk              (0xFFFFFFFFU << USB_OTG_DIEPDMA_DMAADDR_Pos) /*!< 0xFFFFFFFF */
#define USB_OTG_DIEPDMA_DMAADDR                  USB_OTG_DIEPDMA_DMAADDR_Msk   /*!< DMA address */

/********************  Bit definition for USB_OTG_HCDMA register  ********************/
#define USB_OTG_HCDMA_DMAADDR_Pos                (0U)                          
#define USB_OTG_HCDMA_DMAADDR_Msk                (0xFFFFFFFFU << USB_OTG_HCDMA_DMAADDR_Pos) /*!< 0xFFFFFFFF */
#define USB_OTG_HCDMA_DMAADDR                    USB_OTG_HCDMA_DMAADDR_Msk     /*!< DMA address */

/********************  Bit definition for USB_OTG_DTXFSTS register  ********************/
#define USB_OTG_DTXFSTS_INEPTFSAV_Pos            (0U)                          
#define USB_OTG_DTXFSTS_INEPTFSAV_Msk            (0xFFFFU << USB_OTG_DTXFSTS_INEPTFSAV_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DTXFSTS_INEPTFSAV                USB_OTG_DTXFSTS_INEPTFSAV_Msk /*!< IN endpoint TxFIFO space available */

/********************  Bit definition for USB_OTG_DIEPTXF register  ********************/
#define USB_OTG_DIEPTXF_INEPTXSA_Pos             (0U)                          
#define USB_OTG_DIEPTXF_INEPTXSA_Msk             (0xFFFFU << USB_OTG_DIEPTXF_INEPTXSA_Pos) /*!< 0x0000FFFF */
#define USB_OTG_DIEPTXF_INEPTXSA                 USB_OTG_DIEPTXF_INEPTXSA_Msk  /*!< IN endpoint FIFOx transmit RAM start address */
#define USB_OTG_DIEPTXF_INEPTXFD_Pos             (16U)                         
#define USB_OTG_DIEPTXF_INEPTXFD_Msk             (0xFFFFU << USB_OTG_DIEPTXF_INEPTXFD_Pos) /*!< 0xFFFF0000 */
#define USB_OTG_DIEPTXF_INEPTXFD                 USB_OTG_DIEPTXF_INEPTXFD_Msk  /*!< IN endpoint TxFIFO depth */

/********************  Bit definition for USB_OTG_DOEPCTL register  ********************/

#define USB_OTG_DOEPCTL_MPSIZ_Pos                (0U)                          
#define USB_OTG_DOEPCTL_MPSIZ_Msk                (0x7FFU << USB_OTG_DOEPCTL_MPSIZ_Pos) /*!< 0x000007FF */
#define USB_OTG_DOEPCTL_MPSIZ                    USB_OTG_DOEPCTL_MPSIZ_Msk     /*!< Maximum packet size */          /*!<Bit 1 */
#define USB_OTG_DOEPCTL_USBAEP_Pos               (15U)                         
#define USB_OTG_DOEPCTL_USBAEP_Msk               (0x1U << USB_OTG_DOEPCTL_USBAEP_Pos) /*!< 0x00008000 */
#define USB_OTG_DOEPCTL_USBAEP                   USB_OTG_DOEPCTL_USBAEP_Msk    /*!< USB active endpoint */
#define USB_OTG_DOEPCTL_NAKSTS_Pos               (17U)                         
#define USB_OTG_DOEPCTL_NAKSTS_Msk               (0x1U << USB_OTG_DOEPCTL_NAKSTS_Pos) /*!< 0x00020000 */
#define USB_OTG_DOEPCTL_NAKSTS                   USB_OTG_DOEPCTL_NAKSTS_Msk    /*!< NAK status */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Pos       (28U)                         
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Msk       (0x1U << USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Pos) /*!< 0x10000000 */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM           USB_OTG_DOEPCTL_SD0PID_SEVNFRM_Msk /*!< Set DATA0 PID */
#define USB_OTG_DOEPCTL_SODDFRM_Pos              (29U)                         
#define USB_OTG_DOEPCTL_SODDFRM_Msk              (0x1U << USB_OTG_DOEPCTL_SODDFRM_Pos) /*!< 0x20000000 */
#define USB_OTG_DOEPCTL_SODDFRM                  USB_OTG_DOEPCTL_SODDFRM_Msk   /*!< Set odd frame */
#define USB_OTG_DOEPCTL_EPTYP_Pos                (18U)                         
#define USB_OTG_DOEPCTL_EPTYP_Msk                (0x3U << USB_OTG_DOEPCTL_EPTYP_Pos) /*!< 0x000C0000 */
#define USB_OTG_DOEPCTL_EPTYP                    USB_OTG_DOEPCTL_EPTYP_Msk     /*!< Endpoint type */
#define USB_OTG_DOEPCTL_EPTYP_0                  (0x1U << USB_OTG_DOEPCTL_EPTYP_Pos) /*!< 0x00040000 */
#define USB_OTG_DOEPCTL_EPTYP_1                  (0x2U << USB_OTG_DOEPCTL_EPTYP_Pos) /*!< 0x00080000 */
#define USB_OTG_DOEPCTL_SNPM_Pos                 (20U)                         
#define USB_OTG_DOEPCTL_SNPM_Msk                 (0x1U << USB_OTG_DOEPCTL_SNPM_Pos) /*!< 0x00100000 */
#define USB_OTG_DOEPCTL_SNPM                     USB_OTG_DOEPCTL_SNPM_Msk      /*!< Snoop mode */
#define USB_OTG_DOEPCTL_STALL_Pos                (21U)                         
#define USB_OTG_DOEPCTL_STALL_Msk                (0x1U << USB_OTG_DOEPCTL_STALL_Pos) /*!< 0x00200000 */
#define USB_OTG_DOEPCTL_STALL                    USB_OTG_DOEPCTL_STALL_Msk     /*!< STALL handshake */
#define USB_OTG_DOEPCTL_CNAK_Pos                 (26U)                         
#define USB_OTG_DOEPCTL_CNAK_Msk                 (0x1U << USB_OTG_DOEPCTL_CNAK_Pos) /*!< 0x04000000 */
#define USB_OTG_DOEPCTL_CNAK                     USB_OTG_DOEPCTL_CNAK_Msk      /*!< Clear NAK */
#define USB_OTG_DOEPCTL_SNAK_Pos                 (27U)                         
#define USB_OTG_DOEPCTL_SNAK_Msk                 (0x1U << USB_OTG_DOEPCTL_SNAK_Pos) /*!< 0x08000000 */
#define USB_OTG_DOEPCTL_SNAK                     USB_OTG_DOEPCTL_SNAK_Msk      /*!< Set NAK */
#define USB_OTG_DOEPCTL_EPDIS_Pos                (30U)                         
#define USB_OTG_DOEPCTL_EPDIS_Msk                (0x1U << USB_OTG_DOEPCTL_EPDIS_Pos) /*!< 0x40000000 */
#define USB_OTG_DOEPCTL_EPDIS                    USB_OTG_DOEPCTL_EPDIS_Msk     /*!< Endpoint disable */
#define USB_OTG_DOEPCTL_EPENA_Pos                (31U)                         
#define USB_OTG_DOEPCTL_EPENA_Msk                (0x1U << USB_OTG_DOEPCTL_EPENA_Pos) /*!< 0x80000000 */
#define USB_OTG_DOEPCTL_EPENA                    USB_OTG_DOEPCTL_EPENA_Msk     /*!< Endpoint enable */

/********************  Bit definition for USB_OTG_DOEPINT register  ********************/
#define USB_OTG_DOEPINT_XFRC_Pos                 (0U)                          
#define USB_OTG_DOEPINT_XFRC_Msk                 (0x1U << USB_OTG_DOEPINT_XFRC_Pos) /*!< 0x00000001 */
#define USB_OTG_DOEPINT_XFRC                     USB_OTG_DOEPINT_XFRC_Msk      /*!< Transfer completed interrupt */
#define USB_OTG_DOEPINT_EPDISD_Pos               (1U)                          
#define USB_OTG_DOEPINT_EPDISD_Msk               (0x1U << USB_OTG_DOEPINT_EPDISD_Pos) /*!< 0x00000002 */
#define USB_OTG_DOEPINT_EPDISD                   USB_OTG_DOEPINT_EPDISD_Msk    /*!< Endpoint disabled interrupt */
#define USB_OTG_DOEPINT_STUP_Pos                 (3U)                          
#define USB_OTG_DOEPINT_STUP_Msk                 (0x1U << USB_OTG_DOEPINT_STUP_Pos) /*!< 0x00000008 */
#define USB_OTG_DOEPINT_STUP                     USB_OTG_DOEPINT_STUP_Msk      /*!< SETUP phase done */
#define USB_OTG_DOEPINT_OTEPDIS_Pos              (4U)                          
#define USB_OTG_DOEPINT_OTEPDIS_Msk              (0x1U << USB_OTG_DOEPINT_OTEPDIS_Pos) /*!< 0x00000010 */
#define USB_OTG_DOEPINT_OTEPDIS                  USB_OTG_DOEPINT_OTEPDIS_Msk   /*!< OUT token received when endpoint disabled */
#define USB_OTG_DOEPINT_B2BSTUP_Pos              (6U)                          
#define USB_OTG_DOEPINT_B2BSTUP_Msk              (0x1U << USB_OTG_DOEPINT_B2BSTUP_Pos) /*!< 0x00000040 */
#define USB_OTG_DOEPINT_B2BSTUP                  USB_OTG_DOEPINT_B2BSTUP_Msk   /*!< Back-to-back SETUP packets received */
#define USB_OTG_DOEPINT_NYET_Pos                 (14U)                         
#define USB_OTG_DOEPINT_NYET_Msk                 (0x1U << USB_OTG_DOEPINT_NYET_Pos) /*!< 0x00004000 */
#define USB_OTG_DOEPINT_NYET                     USB_OTG_DOEPINT_NYET_Msk      /*!< NYET interrupt */

/********************  Bit definition for USB_OTG_DOEPTSIZ register  ********************/

#define USB_OTG_DOEPTSIZ_XFRSIZ_Pos              (0U)                          
#define USB_OTG_DOEPTSIZ_XFRSIZ_Msk              (0x7FFFFU << USB_OTG_DOEPTSIZ_XFRSIZ_Pos) /*!< 0x0007FFFF */
#define USB_OTG_DOEPTSIZ_XFRSIZ                  USB_OTG_DOEPTSIZ_XFRSIZ_Msk   /*!< Transfer size */
#define USB_OTG_DOEPTSIZ_PKTCNT_Pos              (19U)                         
#define USB_OTG_DOEPTSIZ_PKTCNT_Msk              (0x3FFU << USB_OTG_DOEPTSIZ_PKTCNT_Pos) /*!< 0x1FF80000 */
#define USB_OTG_DOEPTSIZ_PKTCNT                  USB_OTG_DOEPTSIZ_PKTCNT_Msk   /*!< Packet count */

#define USB_OTG_DOEPTSIZ_STUPCNT_Pos             (29U)                         
#define USB_OTG_DOEPTSIZ_STUPCNT_Msk             (0x3U << USB_OTG_DOEPTSIZ_STUPCNT_Pos) /*!< 0x60000000 */
#define USB_OTG_DOEPTSIZ_STUPCNT                 USB_OTG_DOEPTSIZ_STUPCNT_Msk  /*!< SETUP packet count */
#define USB_OTG_DOEPTSIZ_STUPCNT_0               (0x1U << USB_OTG_DOEPTSIZ_STUPCNT_Pos) /*!< 0x20000000 */
#define USB_OTG_DOEPTSIZ_STUPCNT_1               (0x2U << USB_OTG_DOEPTSIZ_STUPCNT_Pos) /*!< 0x40000000 */

/********************  Bit definition for PCGCCTL register  ********************/
#define USB_OTG_PCGCCTL_STOPCLK_Pos              (0U)                          
#define USB_OTG_PCGCCTL_STOPCLK_Msk              (0x1U << USB_OTG_PCGCCTL_STOPCLK_Pos) /*!< 0x00000001 */
#define USB_OTG_PCGCCTL_STOPCLK                  USB_OTG_PCGCCTL_STOPCLK_Msk   /*!< SETUP packet count */
#define USB_OTG_PCGCCTL_GATECLK_Pos              (1U)                          
#define USB_OTG_PCGCCTL_GATECLK_Msk              (0x1U << USB_OTG_PCGCCTL_GATECLK_Pos) /*!< 0x00000002 */
#define USB_OTG_PCGCCTL_GATECLK                  USB_OTG_PCGCCTL_GATECLK_Msk   /*!<Bit 0 */
#define USB_OTG_PCGCCTL_PHYSUSP_Pos              (4U)                          
#define USB_OTG_PCGCCTL_PHYSUSP_Msk              (0x1U << USB_OTG_PCGCCTL_PHYSUSP_Pos) /*!< 0x00000010 */
#define USB_OTG_PCGCCTL_PHYSUSP                  USB_OTG_PCGCCTL_PHYSUSP_Msk   /*!<Bit 1 */

/* Legacy define */
/********************  Bit definition for OTG register  ********************/
#define USB_OTG_CHNUM_Pos                        (0U)                          
#define USB_OTG_CHNUM_Msk                        (0xFU << USB_OTG_CHNUM_Pos)   /*!< 0x0000000F */
#define USB_OTG_CHNUM                            USB_OTG_CHNUM_Msk             /*!< Channel number */
#define USB_OTG_CHNUM_0                          (0x1U << USB_OTG_CHNUM_Pos)   /*!< 0x00000001 */
#define USB_OTG_CHNUM_1                          (0x2U << USB_OTG_CHNUM_Pos)   /*!< 0x00000002 */
#define USB_OTG_CHNUM_2                          (0x4U << USB_OTG_CHNUM_Pos)   /*!< 0x00000004 */
#define USB_OTG_CHNUM_3                          (0x8U << USB_OTG_CHNUM_Pos)   /*!< 0x00000008 */
#define USB_OTG_BCNT_Pos                         (4U)                          
#define USB_OTG_BCNT_Msk                         (0x7FFU << USB_OTG_BCNT_Pos)  /*!< 0x00007FF0 */
#define USB_OTG_BCNT                             USB_OTG_BCNT_Msk              /*!< Byte count */

#define USB_OTG_DPID_Pos                         (15U)                         
#define USB_OTG_DPID_Msk                         (0x3U << USB_OTG_DPID_Pos)    /*!< 0x00018000 */
#define USB_OTG_DPID                             USB_OTG_DPID_Msk              /*!< Data PID */
#define USB_OTG_DPID_0                           (0x1U << USB_OTG_DPID_Pos)    /*!< 0x00008000 */
#define USB_OTG_DPID_1                           (0x2U << USB_OTG_DPID_Pos)    /*!< 0x00010000 */

#define USB_OTG_PKTSTS_Pos                       (17U)                         
#define USB_OTG_PKTSTS_Msk                       (0xFU << USB_OTG_PKTSTS_Pos)  /*!< 0x001E0000 */
#define USB_OTG_PKTSTS                           USB_OTG_PKTSTS_Msk            /*!< Packet status */
#define USB_OTG_PKTSTS_0                         (0x1U << USB_OTG_PKTSTS_Pos)  /*!< 0x00020000 */
#define USB_OTG_PKTSTS_1                         (0x2U << USB_OTG_PKTSTS_Pos)  /*!< 0x00040000 */
#define USB_OTG_PKTSTS_2                         (0x4U << USB_OTG_PKTSTS_Pos)  /*!< 0x00080000 */
#define USB_OTG_PKTSTS_3                         (0x8U << USB_OTG_PKTSTS_Pos)  /*!< 0x00100000 */

#define USB_OTG_EPNUM_Pos                        (0U)                          
#define USB_OTG_EPNUM_Msk                        (0xFU << USB_OTG_EPNUM_Pos)   /*!< 0x0000000F */
#define USB_OTG_EPNUM                            USB_OTG_EPNUM_Msk             /*!< Endpoint number */
#define USB_OTG_EPNUM_0                          (0x1U << USB_OTG_EPNUM_Pos)   /*!< 0x00000001 */
#define USB_OTG_EPNUM_1                          (0x2U << USB_OTG_EPNUM_Pos)   /*!< 0x00000002 */
#define USB_OTG_EPNUM_2                          (0x4U << USB_OTG_EPNUM_Pos)   /*!< 0x00000004 */
#define USB_OTG_EPNUM_3                          (0x8U << USB_OTG_EPNUM_Pos)   /*!< 0x00000008 */

#define USB_OTG_FRMNUM_Pos                       (21U)                         
#define USB_OTG_FRMNUM_Msk                       (0xFU << USB_OTG_FRMNUM_Pos)  /*!< 0x01E00000 */
#define USB_OTG_FRMNUM                           USB_OTG_FRMNUM_Msk            /*!< Frame number */
#define USB_OTG_FRMNUM_0                         (0x1U << USB_OTG_FRMNUM_Pos)  /*!< 0x00200000 */
#define USB_OTG_FRMNUM_1                         (0x2U << USB_OTG_FRMNUM_Pos)  /*!< 0x00400000 */
#define USB_OTG_FRMNUM_2                         (0x4U << USB_OTG_FRMNUM_Pos)  /*!< 0x00800000 */
#define USB_OTG_FRMNUM_3                         (0x8U << USB_OTG_FRMNUM_Pos)  /*!< 0x01000000 */
/**
  * @}
  */ 

/** @addtogroup Peripheral_properties
  * @{
  */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC1_COMMON)
/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)


/******************************** DMA Instances *******************************/
#define IS_DMA_STREAM_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Stream0) || \
                                              ((INSTANCE) == DMA1_Stream1) || \
                                              ((INSTANCE) == DMA1_Stream2) || \
                                              ((INSTANCE) == DMA1_Stream3) || \
                                              ((INSTANCE) == DMA1_Stream4) || \
                                              ((INSTANCE) == DMA1_Stream5) || \
                                              ((INSTANCE) == DMA1_Stream6) || \
                                              ((INSTANCE) == DMA1_Stream7) || \
                                              ((INSTANCE) == DMA2_Stream0) || \
                                              ((INSTANCE) == DMA2_Stream1) || \
                                              ((INSTANCE) == DMA2_Stream2) || \
                                              ((INSTANCE) == DMA2_Stream3) || \
                                              ((INSTANCE) == DMA2_Stream4) || \
                                              ((INSTANCE) == DMA2_Stream5) || \
                                              ((INSTANCE) == DMA2_Stream6) || \
                                              ((INSTANCE) == DMA2_Stream7))

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOD) || \
                                        ((INSTANCE) == GPIOE) || \
                                        ((INSTANCE) == GPIOH))

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2) || \
                                       ((INSTANCE) == I2C3))

/******************************* SMBUS Instances ******************************/
#define IS_SMBUS_ALL_INSTANCE         IS_I2C_ALL_INSTANCE

/******************************** I2S Instances *******************************/

#define IS_I2S_ALL_INSTANCE(INSTANCE)  (((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/*************************** I2S Extended Instances ***************************/
#define IS_I2S_EXT_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2S2ext)|| \
                                           ((INSTANCE) == I2S3ext))
/* Legacy Defines */
#define IS_I2S_ALL_INSTANCE_EXT    IS_I2S_EXT_ALL_INSTANCE


/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)


/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3) || \
                                       ((INSTANCE) == SPI4))


/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                   ((INSTANCE) == TIM2)   || \
                                   ((INSTANCE) == TIM3)   || \
                                   ((INSTANCE) == TIM4)   || \
                                   ((INSTANCE) == TIM5)   || \
                                   ((INSTANCE) == TIM9)   || \
                                   ((INSTANCE) == TIM10)  || \
                                   ((INSTANCE) == TIM11))


/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)  || \
                                         ((INSTANCE) == TIM2)  || \
                                         ((INSTANCE) == TIM3)  || \
                                         ((INSTANCE) == TIM4)  || \
                                         ((INSTANCE) == TIM5)  || \
                                         ((INSTANCE) == TIM9)  || \
                                         ((INSTANCE) == TIM10) || \
                                         ((INSTANCE) == TIM11))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM2) || \
                                       ((INSTANCE) == TIM3) || \
                                       ((INSTANCE) == TIM4) || \
                                       ((INSTANCE) == TIM5) || \
                                       ((INSTANCE) == TIM9))

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM2) || \
                                         ((INSTANCE) == TIM3) || \
                                         ((INSTANCE) == TIM4) || \
                                         ((INSTANCE) == TIM5))

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM2) || \
                                       ((INSTANCE) == TIM3) || \
                                       ((INSTANCE) == TIM4) || \
                                       ((INSTANCE) == TIM5))

/******************** TIM Instances : Advanced-control timers *****************/
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM2) || \
                                         ((INSTANCE) == TIM3) || \
                                         ((INSTANCE) == TIM4) || \
                                         ((INSTANCE) == TIM5))

/****************** TIM Instances : DMA requests generation (UDE) *************/
#define IS_TIM_DMA_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM2) || \
                                       ((INSTANCE) == TIM3) || \
                                       ((INSTANCE) == TIM4) || \
                                       ((INSTANCE) == TIM5))

/************ TIM Instances : DMA requests generation (CCxDE) *****************/
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM2) || \
                                          ((INSTANCE) == TIM3) || \
                                          ((INSTANCE) == TIM4) || \
                                          ((INSTANCE) == TIM5))

/************ TIM Instances : DMA requests generation (COMDE) *****************/
#define IS_TIM_CCDMA_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM2) || \
                                          ((INSTANCE) == TIM3) || \
                                          ((INSTANCE) == TIM4) || \
                                          ((INSTANCE) == TIM5)) 

/******************** TIM Instances : DMA burst feature ***********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                             ((INSTANCE) == TIM2) || \
                                             ((INSTANCE) == TIM3) || \
                                             ((INSTANCE) == TIM4) || \
                                             ((INSTANCE) == TIM5))

/****** TIM Instances : master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM2) || \
                                          ((INSTANCE) == TIM3) || \
                                          ((INSTANCE) == TIM4) || \
                                          ((INSTANCE) == TIM5))

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM2) || \
                                         ((INSTANCE) == TIM3) || \
                                         ((INSTANCE) == TIM4) || \
                                         ((INSTANCE) == TIM5) || \
                                         ((INSTANCE) == TIM9))

/********************** TIM Instances : 32 bit Counter ************************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)(((INSTANCE) == TIM2) || \
                                              ((INSTANCE) == TIM5))

/***************** TIM Instances : external trigger input availabe ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                        ((INSTANCE) == TIM2) || \
                                        ((INSTANCE) == TIM3) || \
                                        ((INSTANCE) == TIM4) || \
                                        ((INSTANCE) == TIM5))

/****************** TIM Instances : remapping capability **********************/
#define IS_TIM_REMAP_INSTANCE(INSTANCE) (((INSTANCE) == TIM2)  || \
                                         ((INSTANCE) == TIM5)  || \
                                         ((INSTANCE) == TIM11))

/******************* TIM Instances : output(s) available **********************/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM2) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM3) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM4) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM5) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM9) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2)))           \
    ||                                         \
    (((INSTANCE) == TIM10) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1)))           \
    ||                                         \
    (((INSTANCE) == TIM11) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1))))

/************ TIM Instances : complementary output(s) available ***************/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3))))

/****************** TIM Instances : supporting counting mode selection ********/
#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5))

/****************** TIM Instances : supporting clock division *****************/
#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                                  ((INSTANCE) == TIM2)   || \
                                                  ((INSTANCE) == TIM3)   || \
                                                  ((INSTANCE) == TIM4)   || \
                                                  ((INSTANCE) == TIM5)   || \
                                                  ((INSTANCE) == TIM9)   || \
                                                  ((INSTANCE) == TIM10)  || \
                                                  ((INSTANCE) == TIM11))


/****************** TIM Instances : supporting commutation event generation ***/

#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting OCxREF clear *******************/
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)        (((INSTANCE) == TIM1) || \
                                                       ((INSTANCE) == TIM2) || \
                                                       ((INSTANCE) == TIM3) || \
                                                       ((INSTANCE) == TIM4) || \
                                                       ((INSTANCE) == TIM5))

/****** TIM Instances : supporting external clock mode 1 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5) || \
                                                        ((INSTANCE) == TIM9))

/****** TIM Instances : supporting external clock mode 2 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)|| \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM3) || \
                                                        ((INSTANCE) == TIM4) || \
                                                        ((INSTANCE) == TIM5))

/****************** TIM Instances : supporting repetition counter *************/
#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1))

/****************** TIM Instances : supporting encoder interface **************/
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                      ((INSTANCE) == TIM2) || \
                                                      ((INSTANCE) == TIM3) || \
                                                      ((INSTANCE) == TIM4) || \
                                                      ((INSTANCE) == TIM5) || \
                                                      ((INSTANCE) == TIM9))
/****************** TIM Instances : supporting Hall sensor interface **********/
#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                          ((INSTANCE) == TIM2) || \
                                                          ((INSTANCE) == TIM3) || \
                                                          ((INSTANCE) == TIM4) || \
                                                          ((INSTANCE) == TIM5))
/****************** TIM Instances : supporting the break function *************/
#define IS_TIM_BREAK_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1))

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2) || \
                                     ((INSTANCE) == USART6))

/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                               ((INSTANCE) == USART2) || \
                                               ((INSTANCE) == USART6))

/* Legacy defines */
#define IS_UART_INSTANCE          IS_UART_HALFDUPLEX_INSTANCE

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART6))
/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE          IS_UART_HALFDUPLEX_INSTANCE

/********************* UART Instances : Smart card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2) || \
                                         ((INSTANCE) == USART6))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART6))     

/*********************** PCD Instances ****************************************/
#define IS_PCD_ALL_INSTANCE(INSTANCE) (((INSTANCE) == USB_OTG_FS))

/*********************** HCD Instances ****************************************/
#define IS_HCD_ALL_INSTANCE(INSTANCE) (((INSTANCE) == USB_OTG_FS))

/****************************** SDIO Instances ********************************/
#define IS_SDIO_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SDIO)

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/**
  * @}
  */

/** @addtogroup ADC_Internal_channels
  * @{
  */

#define ADC1_TEMPSENSOR_CHANNEL     16U
#define ADC1_VREFINT_CHANNEL        17U
#define ADC1_VBAT_CHANNEL           18U

/**
  * @}
  */

/** @addtogroup ADC_Calibration_values
  * @{
  */

typedef struct
{
    const uint16_t VREFINT;
    const uint16_t TEMPSENSOR_LOW;
    const uint16_t TEMPSENSOR_HIGH;
}ADC_CalibrationTypeDef;

#define ADC_VREF_CAL_mV     3300

#define ADC_CALIB           ((ADC_CalibrationTypeDef *)((uint32_t)0x1FFF75A8U))
#define ADC_TEMP_CAL_LOW_C  30
#define ADC_TEMP_CAL_HIGH_C 110

#define ADC_VBAT_SCALER     4

/**
  * @}
  */

/** @addtogroup Device_Internal_Oscillators
  * @{
  */

/** @brief Value of the internal high speed oscillator in Hz */
#define HSI_VALUE_Hz        16000000U

/** @brief Approximate value of the internal low speed oscillator in Hz */
#define LSI_VALUE_Hz        32000U

/**
  * @}
  */

/** @addtogroup Device_Programming
  * @{
  */

/** @brief Unique Device ID
 *  @note  Use the macro as it was defined as: uint32_t DEVICE_ID_REG[3] */
#define DEVICE_ID_REG        ((const uint32_t *)UID_BASE)

/** @brief Device Flash Memory Size in kB */
#define DEVICE_FLASH_SIZE_kB (*((const uint16_t *)FLASHSIZE_BASE))

/** @brief System Memory Start Address */
#define SYSTEM_MEMORY_ADDR   ((void *)0x1FFF0000U)

/**
  * @}
  */

/** @addtogroup GPIO_Alternate_function_map
  * @{
  */

/* AF 0 selection */
#define GPIO_RTC_50Hz_AF0      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_MCO_AF0           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_TAMPER_AF0        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_SWJ_AF0           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_TRACE_AF0         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */

/* AF 1 selection */
#define GPIO_TIM1_AF1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_TIM2_AF1          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/* AF 2 selection */
#define GPIO_TIM3_AF2          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_TIM4_AF2          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_TIM5_AF2          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/* AF 3 selection */
#define GPIO_TIM9_AF3          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_TIM10_AF3         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_TIM11_AF3         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/* AF 4 selection */
#define GPIO_I2C1_AF4          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_I2C2_AF4          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_I2C3_AF4          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/* AF 5 selection */
#define GPIO_SPI1_AF5          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping        */
#define GPIO_SPI2_AF5          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_SPI4_AF5          ((uint8_t)0x05)  /* SPI4 Alternate Function mapping        */
#define GPIO_I2S3ext_AF5       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/* AF 6 selection */
#define GPIO_SPI3_AF6          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_I2S2ext_AF6       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */

/* AF 7 selection */
#define GPIO_USART1_AF7        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_USART2_AF7        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_I2S3ext_AF7       ((uint8_t)0x07)  /* I2S3ext_SD Alternate Function mapping */

/* AF 8 selection */
#define GPIO_USART6_AF8        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/* AF 9 selection */
#define GPIO_TIM14_AF9         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */
#define GPIO_I2C2_AF9          ((uint8_t)0x09)  /* I2C2 Alternate Function mapping  */
#define GPIO_I2C3_AF9          ((uint8_t)0x09)  /* I2C3 Alternate Function mapping  */

/* AF 10 selection */
#define GPIO_OTG_FS_AF10       ((uint8_t)0x0A)  /* OTG_FS Alternate Function mapping */

/* AF 12 selection */
#define GPIO_SDIO_AF12         ((uint8_t)0x0C)  /* SDIO Alternate Function mapping */

/* AF 15 selection */
#define GPIO_EVENTOUT_AF15     ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */

/**
  * @}
  */

/****************************** USB Exported Constants ************************/
#define USB_OTG_FS_HOST_MAX_CHANNEL_NBR                8U
#define USB_OTG_FS_MAX_IN_ENDPOINTS                    4U    /* Including EP0 */
#define USB_OTG_FS_MAX_OUT_ENDPOINTS                   4U    /* Including EP0 */
#define USB_OTG_FS_TOTAL_FIFO_SIZE                     1280U /* in Bytes */

/*
 * @brief Specific devices reset values definitions
 */
#define RCC_PLLCFGR_RST_VALUE              0x24003010U
#define RCC_PLLI2SCFGR_RST_VALUE           0x20003000U

#define RCC_MAX_FREQUENCY            84000000U         /*!< Max frequency of family in Hz*/
#define RCC_MAX_FREQUENCY_SCALE3     60000000U         /*!< Maximum frequency for system clock at power scale3, in Hz */
#define RCC_MAX_FREQUENCY_SCALE2    RCC_MAX_FREQUENCY  /*!< Maximum frequency for system clock at power scale2, in Hz */
#define RCC_PLLVCO_OUTPUT_MIN       192000000U       /*!< Frequency min for PLLVCO output, in Hz */
#define RCC_PLLVCO_INPUT_MIN           950000U       /*!< Frequency min for PLLVCO input, in Hz  */
#define RCC_PLLVCO_INPUT_MAX          2100000U       /*!< Frequency max for PLLVCO input, in Hz  */
#define RCC_PLLVCO_OUTPUT_MAX       432000000U       /*!< Frequency max for PLLVCO output, in Hz */

#define RCC_PLLN_MIN_VALUE                192U
#define RCC_PLLN_MAX_VALUE                432U

#define FLASH_SCALE2_LATENCY1_FREQ    30000000U      /*!< HCLK frequency to set FLASH latency 1 in power scale 2  */
#define FLASH_SCALE2_LATENCY2_FREQ    60000000U      /*!< HCLK frequency to set FLASH latency 2 in power scale 2  */

#define FLASH_SCALE3_LATENCY1_FREQ    30000000U      /*!< HCLK frequency to set FLASH latency 1 in power scale 3  */
#define FLASH_SCALE3_LATENCY2_FREQ    60000000U      /*!< HCLK frequency to set FLASH latency 2 in power scale 3  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F401xC_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
