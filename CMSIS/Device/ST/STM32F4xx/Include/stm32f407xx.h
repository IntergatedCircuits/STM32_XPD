/**
  ******************************************************************************
  * @file    stm32f407xx.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2015-12-28
  * @brief   CMSIS STM32F407xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral's registers hardware
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
/**
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f407xx
  * @{
  */
    
#ifndef __STM32F407xx_H
#define __STM32F407xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
  

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals 
  */
#define __CM4_REV                 0x0001  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present                                   */

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
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
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
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                             */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  HASH_RNG_IRQn               = 80,     /*!< Hash and RNG global interrupt                                     */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
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
            __IO uint32_t AWD : 1;                   /*!<Analog watchdog flag */
            __IO uint32_t EOC : 1;                   /*!<End of conversion */
            __IO uint32_t JEOC : 1;                  /*!<Injected channel end of conversion */
            __IO uint32_t JSTRT : 1;                 /*!<Injected channel Start flag */
            __IO uint32_t STRT : 1;                  /*!<Regular channel Start flag */
            __IO uint32_t OVR : 1;                   /*!<Overrun flag */
                 uint32_t __RESERVED0 : 26;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< ADC status register,                         Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t AWDCH : 5;                 /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
            __IO uint32_t EOCIE : 1;                 /*!<Interrupt enable for EOC */
            __IO uint32_t AWDIE : 1;                 /*!<AAnalog Watchdog interrupt enable */
            __IO uint32_t JEOCIE : 1;                /*!<Interrupt enable for injected channels */
            __IO uint32_t SCAN : 1;                  /*!<Scan mode */
            __IO uint32_t AWDSGL : 1;                /*!<Enable the watchdog on a single channel in scan mode */
            __IO uint32_t JAUTO : 1;                 /*!<Automatic injected group conversion */
            __IO uint32_t DISCEN : 1;                /*!<Discontinuous mode on regular channels */
            __IO uint32_t JDISCEN : 1;               /*!<Discontinuous mode on injected channels */
            __IO uint32_t DISCNUM : 3;               /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t JAWDEN : 1;                /*!<Analog watchdog enable on injected channels */
            __IO uint32_t AWDEN : 1;                 /*!<Analog watchdog enable on regular channels */
            __IO uint32_t RES : 2;                   /*!<RES[2:0] bits (Resolution) */
            __IO uint32_t OVRIE : 1;                 /*!<overrun interrupt enable */
                 uint32_t __RESERVED1 : 5;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< ADC control register 1,                      Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t ADON : 1;                  /*!<A/D Converter ON / OFF */
            __IO uint32_t CONT : 1;                  /*!<Continuous Conversion */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t DMA : 1;                   /*!<Direct Memory access mode */
            __IO uint32_t DDS : 1;                   /*!<DMA disable selection (Single ADC) */
            __IO uint32_t EOCS : 1;                  /*!<End of conversion selection */
            __IO uint32_t ALIGN : 1;                 /*!<Data Alignment */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t JEXTSEL : 4;               /*!<JEXTSEL[3:0] bits (External event select for injected group) */
            __IO uint32_t JEXTEN : 2;                /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
            __IO uint32_t JSWSTART : 1;              /*!<Start Conversion of injected channels */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t EXTSEL : 4;                /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
            __IO uint32_t EXTEN : 2;                 /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
            __IO uint32_t SWSTART : 1;               /*!<Start Conversion of regular channels */
                 uint32_t __RESERVED3 : 1;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< ADC control register 2,                      Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t SMP10 : 3;                 /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
            __IO uint32_t SMP11 : 3;                 /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
            __IO uint32_t SMP12 : 3;                 /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
            __IO uint32_t SMP13 : 3;                 /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
            __IO uint32_t SMP14 : 3;                 /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
            __IO uint32_t SMP15 : 3;                 /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
            __IO uint32_t SMP16 : 3;                 /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
            __IO uint32_t SMP17 : 3;                 /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
            __IO uint32_t SMP18 : 3;                 /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
                 uint32_t __RESERVED0 : 5;
        } b;
        __IO uint32_t w;
    } SMPR1;                                 /*!< ADC sample time register 1,                  Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t SMP0 : 3;                  /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
            __IO uint32_t SMP1 : 3;                  /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
            __IO uint32_t SMP2 : 3;                  /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
            __IO uint32_t SMP3 : 3;                  /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
            __IO uint32_t SMP4 : 3;                  /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
            __IO uint32_t SMP5 : 3;                  /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
            __IO uint32_t SMP6 : 3;                  /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
            __IO uint32_t SMP7 : 3;                  /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
            __IO uint32_t SMP8 : 3;                  /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
            __IO uint32_t SMP9 : 3;                  /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SMPR2;                                 /*!< ADC sample time register 2,                  Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t JOFFSET1 : 12;             /*!<Data offset for injected channel 1 */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } JOFR1;                                 /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t JOFFSET2 : 12;             /*!<Data offset for injected channel 2 */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } JOFR2;                                 /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t JOFFSET3 : 12;             /*!<Data offset for injected channel 3 */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } JOFR3;                                 /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t JOFFSET4 : 12;             /*!<Data offset for injected channel 4 */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } JOFR4;                                 /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t HT : 12;                   /*!<Analog watchdog high threshold */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } HTR;                                   /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
    union {
        struct {
            __IO uint32_t LT : 12;                   /*!<Analog watchdog low threshold */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } LTR;                                   /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t SQ13 : 5;                  /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
            __IO uint32_t SQ14 : 5;                  /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
            __IO uint32_t SQ15 : 5;                  /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
            __IO uint32_t SQ16 : 5;                  /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
            __IO uint32_t L : 4;                     /*!<L[3:0] bits (Regular channel sequence length) */
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } SQR1;                                  /*!< ADC regular sequence register 1,             Address offset: 0x2C */
    union {
        struct {
            __IO uint32_t SQ7 : 5;                   /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
            __IO uint32_t SQ8 : 5;                   /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
            __IO uint32_t SQ9 : 5;                   /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
            __IO uint32_t SQ10 : 5;                  /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
            __IO uint32_t SQ11 : 5;                  /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
            __IO uint32_t SQ12 : 5;                  /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SQR2;                                  /*!< ADC regular sequence register 2,             Address offset: 0x30 */
    union {
        struct {
            __IO uint32_t SQ1 : 5;                   /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
            __IO uint32_t SQ2 : 5;                   /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
            __IO uint32_t SQ3 : 5;                   /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
            __IO uint32_t SQ4 : 5;                   /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
            __IO uint32_t SQ5 : 5;                   /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
            __IO uint32_t SQ6 : 5;                   /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SQR3;                                  /*!< ADC regular sequence register 3,             Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t JSQ1 : 5;                  /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */
            __IO uint32_t JSQ2 : 5;                  /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
            __IO uint32_t JSQ3 : 5;                  /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
            __IO uint32_t JSQ4 : 5;                  /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
            __IO uint32_t JL : 2;                    /*!<JL[1:0] bits (Injected Sequence length) */
                 uint32_t __RESERVED0 : 10;
        } b;
        __IO uint32_t w;
    } JSQR;                                  /*!< ADC injected sequence register,              Address offset: 0x38*/
    union {
        struct {
            __IO uint32_t JDATA : 16;                /*!<Injected data */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } JDR1;                                  /*!< ADC injected data register 1,                Address offset: 0x3C */
    union {
        struct {
            __IO uint32_t JDATA : 16;                /*!<Injected data */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } JDR2;                                  /*!< ADC injected data register 2,                Address offset: 0x40 */
    union {
        struct {
            __IO uint32_t JDATA : 16;                /*!<Injected data */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } JDR3;                                  /*!< ADC injected data register 3,                Address offset: 0x44 */
    union {
        struct {
            __IO uint32_t JDATA : 16;                /*!<Injected data */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } JDR4;                                  /*!< ADC injected data register 4,                Address offset: 0x48 */
    union {
        struct {
            __IO uint32_t DATA : 16;                 /*!<Regular data */
            __IO uint32_t ADC2DATA : 16;             /*!<ADC2 data */
        } b;
        __IO uint32_t w;
    } DR;                                    /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t AWD;                       /*!<Analog watchdog flag */
        __IO uint32_t EOC;                       /*!<End of conversion */
        __IO uint32_t JEOC;                      /*!<Injected channel end of conversion */
        __IO uint32_t JSTRT;                     /*!<Injected channel Start flag */
        __IO uint32_t STRT;                      /*!<Regular channel Start flag */
        __IO uint32_t OVR;                       /*!<Overrun flag */
             uint32_t __RESERVED0[26];
    } SR;                                    /*!< ADC status register,                         Address offset: 0x00 */
    struct {
        __IO uint32_t AWDCH[5];                  /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
        __IO uint32_t EOCIE;                     /*!<Interrupt enable for EOC */
        __IO uint32_t AWDIE;                     /*!<AAnalog Watchdog interrupt enable */
        __IO uint32_t JEOCIE;                    /*!<Interrupt enable for injected channels */
        __IO uint32_t SCAN;                      /*!<Scan mode */
        __IO uint32_t AWDSGL;                    /*!<Enable the watchdog on a single channel in scan mode */
        __IO uint32_t JAUTO;                     /*!<Automatic injected group conversion */
        __IO uint32_t DISCEN;                    /*!<Discontinuous mode on regular channels */
        __IO uint32_t JDISCEN;                   /*!<Discontinuous mode on injected channels */
        __IO uint32_t DISCNUM[3];                /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
             uint32_t __RESERVED0[6];
        __IO uint32_t JAWDEN;                    /*!<Analog watchdog enable on injected channels */
        __IO uint32_t AWDEN;                     /*!<Analog watchdog enable on regular channels */
        __IO uint32_t RES[2];                    /*!<RES[2:0] bits (Resolution) */
        __IO uint32_t OVRIE;                     /*!<overrun interrupt enable */
             uint32_t __RESERVED1[5];
    } CR1;                                   /*!< ADC control register 1,                      Address offset: 0x04 */
    struct {
        __IO uint32_t ADON;                      /*!<A/D Converter ON / OFF */
        __IO uint32_t CONT;                      /*!<Continuous Conversion */
             uint32_t __RESERVED0[6];
        __IO uint32_t DMA;                       /*!<Direct Memory access mode */
        __IO uint32_t DDS;                       /*!<DMA disable selection (Single ADC) */
        __IO uint32_t EOCS;                      /*!<End of conversion selection */
        __IO uint32_t ALIGN;                     /*!<Data Alignment */
             uint32_t __RESERVED1[4];
        __IO uint32_t JEXTSEL[4];                /*!<JEXTSEL[3:0] bits (External event select for injected group) */
        __IO uint32_t JEXTEN[2];                 /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
        __IO uint32_t JSWSTART;                  /*!<Start Conversion of injected channels */
             uint32_t __RESERVED2;
        __IO uint32_t EXTSEL[4];                 /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
        __IO uint32_t EXTEN[2];                  /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
        __IO uint32_t SWSTART;                   /*!<Start Conversion of regular channels */
             uint32_t __RESERVED3;
    } CR2;                                   /*!< ADC control register 2,                      Address offset: 0x08 */
    struct {
        __IO uint32_t SMP10[3];                  /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
        __IO uint32_t SMP11[3];                  /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
        __IO uint32_t SMP12[3];                  /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
        __IO uint32_t SMP13[3];                  /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
        __IO uint32_t SMP14[3];                  /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
        __IO uint32_t SMP15[3];                  /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
        __IO uint32_t SMP16[3];                  /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
        __IO uint32_t SMP17[3];                  /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
        __IO uint32_t SMP18[3];                  /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
             uint32_t __RESERVED0[5];
    } SMPR1;                                 /*!< ADC sample time register 1,                  Address offset: 0x0C */
    struct {
        __IO uint32_t SMP0[3];                   /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
        __IO uint32_t SMP1[3];                   /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
        __IO uint32_t SMP2[3];                   /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
        __IO uint32_t SMP3[3];                   /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
        __IO uint32_t SMP4[3];                   /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
        __IO uint32_t SMP5[3];                   /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
        __IO uint32_t SMP6[3];                   /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
        __IO uint32_t SMP7[3];                   /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
        __IO uint32_t SMP8[3];                   /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
        __IO uint32_t SMP9[3];                   /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
             uint32_t __RESERVED0[2];
    } SMPR2;                                 /*!< ADC sample time register 2,                  Address offset: 0x10 */
    struct {
        __IO uint32_t JOFFSET1[12];              /*!<Data offset for injected channel 1 */
             uint32_t __RESERVED0[20];
    } JOFR1;                                 /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
    struct {
        __IO uint32_t JOFFSET2[12];              /*!<Data offset for injected channel 2 */
             uint32_t __RESERVED0[20];
    } JOFR2;                                 /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
    struct {
        __IO uint32_t JOFFSET3[12];              /*!<Data offset for injected channel 3 */
             uint32_t __RESERVED0[20];
    } JOFR3;                                 /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
    struct {
        __IO uint32_t JOFFSET4[12];              /*!<Data offset for injected channel 4 */
             uint32_t __RESERVED0[20];
    } JOFR4;                                 /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
    struct {
        __IO uint32_t HT[12];                    /*!<Analog watchdog high threshold */
             uint32_t __RESERVED0[20];
    } HTR;                                   /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
    struct {
        __IO uint32_t LT[12];                    /*!<Analog watchdog low threshold */
             uint32_t __RESERVED0[20];
    } LTR;                                   /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
    struct {
        __IO uint32_t SQ13[5];                   /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
        __IO uint32_t SQ14[5];                   /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
        __IO uint32_t SQ15[5];                   /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
        __IO uint32_t SQ16[5];                   /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
        __IO uint32_t L[4];                      /*!<L[3:0] bits (Regular channel sequence length) */
             uint32_t __RESERVED0[8];
    } SQR1;                                  /*!< ADC regular sequence register 1,             Address offset: 0x2C */
    struct {
        __IO uint32_t SQ7[5];                    /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
        __IO uint32_t SQ8[5];                    /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
        __IO uint32_t SQ9[5];                    /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
        __IO uint32_t SQ10[5];                   /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
        __IO uint32_t SQ11[5];                   /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
        __IO uint32_t SQ12[5];                   /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
             uint32_t __RESERVED0[2];
    } SQR2;                                  /*!< ADC regular sequence register 2,             Address offset: 0x30 */
    struct {
        __IO uint32_t SQ1[5];                    /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
        __IO uint32_t SQ2[5];                    /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
        __IO uint32_t SQ3[5];                    /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
        __IO uint32_t SQ4[5];                    /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
        __IO uint32_t SQ5[5];                    /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
        __IO uint32_t SQ6[5];                    /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
             uint32_t __RESERVED0[2];
    } SQR3;                                  /*!< ADC regular sequence register 3,             Address offset: 0x34 */
    struct {
        __IO uint32_t JSQ1[5];                   /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */
        __IO uint32_t JSQ2[5];                   /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
        __IO uint32_t JSQ3[5];                   /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
        __IO uint32_t JSQ4[5];                   /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
        __IO uint32_t JL[2];                     /*!<JL[1:0] bits (Injected Sequence length) */
             uint32_t __RESERVED0[10];
    } JSQR;                                  /*!< ADC injected sequence register,              Address offset: 0x38*/
    struct {
        __IO uint32_t JDATA[16];                 /*!<Injected data */
             uint32_t __RESERVED0[16];
    } JDR1;                                  /*!< ADC injected data register 1,                Address offset: 0x3C */
    struct {
        __IO uint32_t JDATA[16];                 /*!<Injected data */
             uint32_t __RESERVED0[16];
    } JDR2;                                  /*!< ADC injected data register 2,                Address offset: 0x40 */
    struct {
        __IO uint32_t JDATA[16];                 /*!<Injected data */
             uint32_t __RESERVED0[16];
    } JDR3;                                  /*!< ADC injected data register 3,                Address offset: 0x44 */
    struct {
        __IO uint32_t JDATA[16];                 /*!<Injected data */
             uint32_t __RESERVED0[16];
    } JDR4;                                  /*!< ADC injected data register 4,                Address offset: 0x48 */
    struct {
        __IO uint32_t DATA[16];                  /*!<Regular data */
        __IO uint32_t ADC2DATA[16];              /*!<ADC2 data */
    } DR;                                    /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_BitBand_TypeDef;




typedef struct {
    union {
        struct {
            __IO uint32_t AWD1 : 1;                  /*!<ADC1 Analog watchdog flag */
            __IO uint32_t EOC1 : 1;                  /*!<ADC1 End of conversion */
            __IO uint32_t JEOC1 : 1;                 /*!<ADC1 Injected channel end of conversion */
            __IO uint32_t JSTRT1 : 1;                /*!<ADC1 Injected channel Start flag */
            __IO uint32_t STRT1 : 1;                 /*!<ADC1 Regular channel Start flag */
            __IO uint32_t DOVR1 : 1;                 /*!<ADC1 DMA overrun  flag */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t AWD2 : 1;                  /*!<ADC2 Analog watchdog flag */
            __IO uint32_t EOC2 : 1;                  /*!<ADC2 End of conversion */
            __IO uint32_t JEOC2 : 1;                 /*!<ADC2 Injected channel end of conversion */
            __IO uint32_t JSTRT2 : 1;                /*!<ADC2 Injected channel Start flag */
            __IO uint32_t STRT2 : 1;                 /*!<ADC2 Regular channel Start flag */
            __IO uint32_t DOVR2 : 1;                 /*!<ADC2 DMA overrun  flag */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t AWD3 : 1;                  /*!<ADC3 Analog watchdog flag */
            __IO uint32_t EOC3 : 1;                  /*!<ADC3 End of conversion */
            __IO uint32_t JEOC3 : 1;                 /*!<ADC3 Injected channel end of conversion */
            __IO uint32_t JSTRT3 : 1;                /*!<ADC3 Injected channel Start flag */
            __IO uint32_t STRT3 : 1;                 /*!<ADC3 Regular channel Start flag */
            __IO uint32_t DOVR3 : 1;                 /*!<ADC3 DMA overrun  flag */
                 uint32_t __RESERVED2 : 10;
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
    union {
        struct {
            __IO uint32_t MULTI : 5;                 /*!<MULTI[4:0] bits (Multi-ADC mode selection) */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t DELAY : 4;                 /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t DDS : 1;                   /*!<DMA disable selection (Multi-ADC mode) */
            __IO uint32_t DMA : 2;                   /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */
            __IO uint32_t ADCPRE : 2;                /*!<ADCPRE[1:0] bits (ADC prescaler) */
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t VBATE : 1;                 /*!<VBAT Enable */
            __IO uint32_t TSVREFE : 1;               /*!<Temperature Sensor and VREFINT Enable */
                 uint32_t __RESERVED3 : 8;
        } b;
        __IO uint32_t w;
    } CCR;                                   /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
    union {
        struct {
            __IO uint32_t DATA1 : 16;                /*!<1st data of a pair of regular conversions */
            __IO uint32_t DATA2 : 16;                /*!<2nd data of a pair of regular conversions */
        } b;
        __IO uint32_t w;
    } CDR;                                   /*!< ADC common regular data register for dual
                                 AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;


typedef struct {
    struct {
        __IO uint32_t AWD1;                      /*!<ADC1 Analog watchdog flag */
        __IO uint32_t EOC1;                      /*!<ADC1 End of conversion */
        __IO uint32_t JEOC1;                     /*!<ADC1 Injected channel end of conversion */
        __IO uint32_t JSTRT1;                    /*!<ADC1 Injected channel Start flag */
        __IO uint32_t STRT1;                     /*!<ADC1 Regular channel Start flag */
        __IO uint32_t DOVR1;                     /*!<ADC1 DMA overrun  flag */
             uint32_t __RESERVED0[2];
        __IO uint32_t AWD2;                      /*!<ADC2 Analog watchdog flag */
        __IO uint32_t EOC2;                      /*!<ADC2 End of conversion */
        __IO uint32_t JEOC2;                     /*!<ADC2 Injected channel end of conversion */
        __IO uint32_t JSTRT2;                    /*!<ADC2 Injected channel Start flag */
        __IO uint32_t STRT2;                     /*!<ADC2 Regular channel Start flag */
        __IO uint32_t DOVR2;                     /*!<ADC2 DMA overrun  flag */
             uint32_t __RESERVED1[2];
        __IO uint32_t AWD3;                      /*!<ADC3 Analog watchdog flag */
        __IO uint32_t EOC3;                      /*!<ADC3 End of conversion */
        __IO uint32_t JEOC3;                     /*!<ADC3 Injected channel end of conversion */
        __IO uint32_t JSTRT3;                    /*!<ADC3 Injected channel Start flag */
        __IO uint32_t STRT3;                     /*!<ADC3 Regular channel Start flag */
        __IO uint32_t DOVR3;                     /*!<ADC3 DMA overrun  flag */
             uint32_t __RESERVED2[10];
    } CSR;                                   /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
    struct {
        __IO uint32_t MULTI[5];                  /*!<MULTI[4:0] bits (Multi-ADC mode selection) */
             uint32_t __RESERVED0[3];
        __IO uint32_t DELAY[4];                  /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */
             uint32_t __RESERVED1;
        __IO uint32_t DDS;                       /*!<DMA disable selection (Multi-ADC mode) */
        __IO uint32_t DMA[2];                    /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */
        __IO uint32_t ADCPRE[2];                 /*!<ADCPRE[1:0] bits (ADC prescaler) */
             uint32_t __RESERVED2[4];
        __IO uint32_t VBATE;                     /*!<VBAT Enable */
        __IO uint32_t TSVREFE;                   /*!<Temperature Sensor and VREFINT Enable */
             uint32_t __RESERVED3[8];
    } CCR;                                   /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
    struct {
        __IO uint32_t DATA1[16];                 /*!<1st data of a pair of regular conversions */
        __IO uint32_t DATA2[16];                 /*!<2nd data of a pair of regular conversions */
    } CDR;                                   /*!< ADC common regular data register for dual
                                 AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_BitBand_TypeDef;




/** 
  * @brief Controller Area Network TxMailBox 
  */


typedef struct {
    union {
        struct {
            __IO uint32_t TXRQ : 1;                  /*!<Transmit Mailbox Request */
            __IO uint32_t RTR : 1;                   /*!<Remote Transmission Request */
            __IO uint32_t IDE : 1;                   /*!<Identifier Extension */
            __IO uint32_t EXID : 18;                 /*!<Extended Identifier */
            __IO uint32_t STID : 11;                 /*!<Standard Identifier or Extended Identifier */
        } b;
        __IO uint32_t w;
    } TIR;                                   /*!< CAN TX mailbox identifier register */
    union {
        struct {
            __IO uint32_t DLC : 4;                   /*!<Data Length Code */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t TGT : 1;                   /*!<Transmit Global Time */
                 uint32_t __RESERVED1 : 7;
            __IO uint32_t TIME : 16;                 /*!<Message Time Stamp */
        } b;
        __IO uint32_t w;
    } TDTR;                                  /*!< CAN mailbox data length control and time stamp register */
    union {
        struct {
            __IO uint32_t DATA0 : 8;                 /*!<Data byte 0 */
            __IO uint32_t DATA1 : 8;                 /*!<Data byte 1 */
            __IO uint32_t DATA2 : 8;                 /*!<Data byte 2 */
            __IO uint32_t DATA3 : 8;                 /*!<Data byte 3 */
        } b;
        __IO uint32_t w;
    } TDLR;                                  /*!< CAN mailbox data low register */
    union {
        struct {
            __IO uint32_t DATA4 : 8;                 /*!<Data byte 4 */
            __IO uint32_t DATA5 : 8;                 /*!<Data byte 5 */
            __IO uint32_t DATA6 : 8;                 /*!<Data byte 6 */
            __IO uint32_t DATA7 : 8;                 /*!<Data byte 7 */
        } b;
        __IO uint32_t w;
    } TDHR;                                  /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;


/** 
  * @brief Controller Area Network FIFOMailBox 
  */
  

typedef struct {
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RTR : 1;                   /*!<Remote Transmission Request */
            __IO uint32_t IDE : 1;                   /*!<Identifier Extension */
            __IO uint32_t EXID : 18;                 /*!<Extended Identifier */
            __IO uint32_t STID : 11;                 /*!<Standard Identifier or Extended Identifier */
        } b;
        __IO uint32_t w;
    } RIR;                                   /*!< CAN receive FIFO mailbox identifier register */
    union {
        struct {
            __IO uint32_t DLC : 4;                   /*!<Data Length Code */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t FMI : 8;                   /*!<Filter Match Index */
            __IO uint32_t TIME : 16;                 /*!<Message Time Stamp */
        } b;
        __IO uint32_t w;
    } RDTR;                                  /*!< CAN receive FIFO mailbox data length control and time stamp register */
    union {
        struct {
            __IO uint32_t DATA0 : 8;                 /*!<Data byte 0 */
            __IO uint32_t DATA1 : 8;                 /*!<Data byte 1 */
            __IO uint32_t DATA2 : 8;                 /*!<Data byte 2 */
            __IO uint32_t DATA3 : 8;                 /*!<Data byte 3 */
        } b;
        __IO uint32_t w;
    } RDLR;                                  /*!< CAN receive FIFO mailbox data low register */
    union {
        struct {
            __IO uint32_t DATA4 : 8;                 /*!<Data byte 4 */
            __IO uint32_t DATA5 : 8;                 /*!<Data byte 5 */
            __IO uint32_t DATA6 : 8;                 /*!<Data byte 6 */
            __IO uint32_t DATA7 : 8;                 /*!<Data byte 7 */
        } b;
        __IO uint32_t w;
    } RDHR;                                  /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;


/** 
  * @brief Controller Area Network FilterRegister 
  */
  

typedef struct {
    __IO uint32_t FR1;                       /*!< CAN Filter bank register 1 */
    __IO uint32_t FR2;                       /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;


/** 
  * @brief Controller Area Network 
  */
  

typedef struct {
    union {
        struct {
            __IO uint32_t INRQ : 1;                  /*!<Initialization Request */
            __IO uint32_t SLEEP : 1;                 /*!<Sleep Mode Request */
            __IO uint32_t TXFP : 1;                  /*!<Transmit FIFO Priority */
            __IO uint32_t RFLM : 1;                  /*!<Receive FIFO Locked Mode */
            __IO uint32_t NART : 1;                  /*!<No Automatic Retransmission */
            __IO uint32_t AWUM : 1;                  /*!<Automatic Wakeup Mode */
            __IO uint32_t ABOM : 1;                  /*!<Automatic Bus-Off Management */
            __IO uint32_t TTCM : 1;                  /*!<Time Triggered Communication Mode */
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t RESET : 1;                 /*!<bxCAN software master reset */
            __IO uint32_t DBF : 1;                   /*!<bxCAN Debug freeze */
                 uint32_t __RESERVED1 : 15;
        } b;
        __IO uint32_t w;
    } MCR;                                   /*!< CAN master control register,         Address offset: 0x00          */
    union {
        struct {
            __IO uint32_t INAK : 1;                  /*!<Initialization Acknowledge */
            __IO uint32_t SLAK : 1;                  /*!<Sleep Acknowledge */
            __IO uint32_t ERRI : 1;                  /*!<Error Interrupt */
            __IO uint32_t WKUI : 1;                  /*!<Wakeup Interrupt */
            __IO uint32_t SLAKI : 1;                 /*!<Sleep Acknowledge Interrupt */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t TXM : 1;                   /*!<Transmit Mode */
            __IO uint32_t RXM : 1;                   /*!<Receive Mode */
            __IO uint32_t SAMP : 1;                  /*!<Last Sample Point */
            __IO uint32_t RX : 1;                    /*!<CAN Rx Signal */
                 uint32_t __RESERVED1 : 20;
        } b;
        __IO uint32_t w;
    } MSR;                                   /*!< CAN master status register,          Address offset: 0x04          */
    union {
        struct {
            __IO uint32_t RQCP0 : 1;                 /*!<Request Completed Mailbox0 */
            __IO uint32_t TXOK0 : 1;                 /*!<Transmission OK of Mailbox0 */
            __IO uint32_t ALST0 : 1;                 /*!<Arbitration Lost for Mailbox0 */
            __IO uint32_t TERR0 : 1;                 /*!<Transmission Error of Mailbox0 */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t ABRQ0 : 1;                 /*!<Abort Request for Mailbox0 */
            __IO uint32_t RQCP1 : 1;                 /*!<Request Completed Mailbox1 */
            __IO uint32_t TXOK1 : 1;                 /*!<Transmission OK of Mailbox1 */
            __IO uint32_t ALST1 : 1;                 /*!<Arbitration Lost for Mailbox1 */
            __IO uint32_t TERR1 : 1;                 /*!<Transmission Error of Mailbox1 */
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t ABRQ1 : 1;                 /*!<Abort Request for Mailbox 1 */
            __IO uint32_t RQCP2 : 1;                 /*!<Request Completed Mailbox2 */
            __IO uint32_t TXOK2 : 1;                 /*!<Transmission OK of Mailbox 2 */
            __IO uint32_t ALST2 : 1;                 /*!<Arbitration Lost for mailbox 2 */
            __IO uint32_t TERR2 : 1;                 /*!<Transmission Error of Mailbox 2 */
                 uint32_t __RESERVED2 : 3;
            __IO uint32_t ABRQ2 : 1;                 /*!<Abort Request for Mailbox 2 */
            __IO uint32_t CODE : 2;                  /*!<Mailbox Code */
            __IO uint32_t TME : 3;                   /*!<TME[2:0] bits */
            __IO uint32_t LOW : 3;                   /*!<LOW[2:0] bits */
        } b;
        __IO uint32_t w;
    } TSR;                                   /*!< CAN transmit status register,        Address offset: 0x08          */
    union {
        struct {
            __IO uint32_t FMP : 2;                   /*!<FIFO Message Pending */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t FULL : 1;                  /*!<FIFO Full */
            __IO uint32_t FOVR : 1;                  /*!<FIFO Overrun */
            __IO uint32_t RFOM : 1;                  /*!<Release FIFO Output Mailbox */
                 uint32_t __RESERVED1 : 26;
        } b;
        __IO uint32_t w;
    } RFR[2];
    union {
        struct {
            __IO uint32_t TMEIE : 1;                 /*!<Transmit Mailbox Empty Interrupt Enable */
            __IO uint32_t FMP0IE : 1;                /*!<FIFO 0 Message Pending Interrupt Enable */
            __IO uint32_t FF0IE : 1;                 /*!<FIFO 0 Full Interrupt Enable */
            __IO uint32_t FOV0IE : 1;                /*!<FIFO 0 Overrun Interrupt Enable */
            __IO uint32_t FMP1IE : 1;                /*!<FIFO 1 Message Pending Interrupt Enable */
            __IO uint32_t FF1IE : 1;                 /*!<FIFO 1 Full Interrupt Enable */
            __IO uint32_t FOV1IE : 1;                /*!<FIFO 1 Overrun Interrupt Enable */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t EWGIE : 1;                 /*!<Error Warning Interrupt Enable */
            __IO uint32_t EPVIE : 1;                 /*!<Error Passive Interrupt Enable */
            __IO uint32_t BOFIE : 1;                 /*!<Bus-Off Interrupt Enable */
            __IO uint32_t LECIE : 1;                 /*!<Last Error Code Interrupt Enable */
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t ERRIE : 1;                 /*!<Error Interrupt Enable */
            __IO uint32_t WKUIE : 1;                 /*!<Wakeup Interrupt Enable */
            __IO uint32_t SLKIE : 1;                 /*!<Sleep Interrupt Enable */
                 uint32_t __RESERVED2 : 14;
        } b;
        __IO uint32_t w;
    } IER;                                   /*!< CAN interrupt enable register,       Address offset: 0x14          */
    union {
        struct {
            __IO uint32_t EWGF : 1;                  /*!<Error Warning Flag */
            __IO uint32_t EPVF : 1;                  /*!<Error Passive Flag */
            __IO uint32_t BOFF : 1;                  /*!<Bus-Off Flag */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t LEC : 3;                   /*!<LEC[2:0] bits (Last Error Code) */
                 uint32_t __RESERVED1 : 9;
            __IO uint32_t TEC : 8;                   /*!<Least significant byte of the 9-bit Transmit Error Counter */
            __IO uint32_t REC : 8;                   /*!<Receive Error Counter */
        } b;
        __IO uint32_t w;
    } ESR;                                   /*!< CAN error status register,           Address offset: 0x18          */
    union {
        struct {
            __IO uint32_t BRP : 10;                  /*!<Baud Rate Prescaler */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t TS1 : 4;                   /*!<Time Segment 1 */
            __IO uint32_t TS2 : 3;                   /*!<Time Segment 2 */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SJW : 2;                   /*!<Resynchronization Jump Width */
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t LBKM : 1;                  /*!<Loop Back Mode (Debug) */
            __IO uint32_t SILM : 1;                  /*!<Silent Mode */
        } b;
        __IO uint32_t w;
    } BTR;                                   /*!< CAN bit timing register,             Address offset: 0x1C          */
         uint32_t __RESERVED0[88];               /*!< Reserved, 0x020 - 0x17F                                            */
    CAN_TxMailBox_TypeDef sTxMailBox[3];     /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
    CAN_FIFOMailBox_TypeDef sFIFOMailBox[2]; /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
         uint32_t __RESERVED1[12];               /*!< Reserved, 0x1D0 - 0x1FF                                            */
    union {
        struct {
            __IO uint32_t FINIT : 1;                 /*!<Filter Init Mode */
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t CAN2SB : 6;                /*!<CAN2 start bank */
                 uint32_t __RESERVED1 : 18;
        } b;
        __IO uint32_t w;
    } FMR;                                   /*!< CAN filter master register,          Address offset: 0x200         */
    __IO uint32_t FM1R;                      /*!< CAN filter mode register,            Address offset: 0x204         */
         uint32_t __RESERVED2;               /*!< Reserved, 0x208                                                    */
    __IO uint32_t FS1R;                      /*!< CAN filter scale register,           Address offset: 0x20C         */
         uint32_t __RESERVED3;               /*!< Reserved, 0x210                                                    */
    __IO uint32_t FFA1R;                     /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
         uint32_t __RESERVED4;               /*!< Reserved, 0x218                                                    */
    __IO uint32_t FA1R;                      /*!< CAN filter activation register,      Address offset: 0x21C         */
         uint32_t __RESERVED5[8];               /*!< Reserved, 0x220-0x23F                                              */
    CAN_FilterRegister_TypeDef sFilterRegister[28];/*!< CAN Filter Register,           Address offset: 0x240-0x31C   */
} CAN_TypeDef;


typedef struct {
    struct {
        __IO uint32_t INRQ;                      /*!<Initialization Request */
        __IO uint32_t SLEEP;                     /*!<Sleep Mode Request */
        __IO uint32_t TXFP;                      /*!<Transmit FIFO Priority */
        __IO uint32_t RFLM;                      /*!<Receive FIFO Locked Mode */
        __IO uint32_t NART;                      /*!<No Automatic Retransmission */
        __IO uint32_t AWUM;                      /*!<Automatic Wakeup Mode */
        __IO uint32_t ABOM;                      /*!<Automatic Bus-Off Management */
        __IO uint32_t TTCM;                      /*!<Time Triggered Communication Mode */
             uint32_t __RESERVED0[7];
        __IO uint32_t RESET;                     /*!<bxCAN software master reset */
        __IO uint32_t DBF;                       /*!<bxCAN Debug freeze */
             uint32_t __RESERVED1[15];
    } MCR;                                   /*!< CAN master control register,         Address offset: 0x00          */
    struct {
        __IO uint32_t INAK;                      /*!<Initialization Acknowledge */
        __IO uint32_t SLAK;                      /*!<Sleep Acknowledge */
        __IO uint32_t ERRI;                      /*!<Error Interrupt */
        __IO uint32_t WKUI;                      /*!<Wakeup Interrupt */
        __IO uint32_t SLAKI;                     /*!<Sleep Acknowledge Interrupt */
             uint32_t __RESERVED0[3];
        __IO uint32_t TXM;                       /*!<Transmit Mode */
        __IO uint32_t RXM;                       /*!<Receive Mode */
        __IO uint32_t SAMP;                      /*!<Last Sample Point */
        __IO uint32_t RX;                        /*!<CAN Rx Signal */
             uint32_t __RESERVED1[20];
    } MSR;                                   /*!< CAN master status register,          Address offset: 0x04          */
    struct {
        struct {
            __IO uint32_t RQCP;                      /*!<Request Completed MailboxX */
            __IO uint32_t TXOK;                      /*!<Transmission OK of MailboxX */
            __IO uint32_t ALST;                      /*!<Arbitration Lost for MailboxX */
            __IO uint32_t TERR;                      /*!<Transmission Error of MailboxX */
                 uint32_t __RESERVED0[3];
            __IO uint32_t ABRQ;                      /*!<Abort Request for MailboxX */
        }MB[3];                                  /*!<State of each mailbox */
        __IO uint32_t CODE[2];                   /*!<Mailbox Code */
        __IO uint32_t TME[3];                    /*!<TME[2:0] bits */
        __IO uint32_t LOW[3];                    /*!<LOW[2:0] bits */
    } TSR;                                   /*!< CAN transmit status register,        Address offset: 0x08          */
    struct {
        __IO uint32_t FMP[2];                    /*!<FIFO Message Pending */
             uint32_t __RESERVED0;
        __IO uint32_t FULL;                      /*!<FIFO Full */
        __IO uint32_t FOVR;                      /*!<FIFO Overrun */
        __IO uint32_t RFOM;                      /*!<Release FIFO Output Mailbox */
             uint32_t __RESERVED1[26];
    } RFR[2];
    struct {
        __IO uint32_t TMEIE;                     /*!<Transmit Mailbox Empty Interrupt Enable */
        __IO uint32_t FMP0IE;                    /*!<FIFO 0 Message Pending Interrupt Enable */
        __IO uint32_t FF0IE;                     /*!<FIFO 0 Full Interrupt Enable */
        __IO uint32_t FOV0IE;                    /*!<FIFO 0 Overrun Interrupt Enable */
        __IO uint32_t FMP1IE;                    /*!<FIFO 1 Message Pending Interrupt Enable */
        __IO uint32_t FF1IE;                     /*!<FIFO 1 Full Interrupt Enable */
        __IO uint32_t FOV1IE;                    /*!<FIFO 1 Overrun Interrupt Enable */
             uint32_t __RESERVED0;
        __IO uint32_t EWGIE;                     /*!<Error Warning Interrupt Enable */
        __IO uint32_t EPVIE;                     /*!<Error Passive Interrupt Enable */
        __IO uint32_t BOFIE;                     /*!<Bus-Off Interrupt Enable */
        __IO uint32_t LECIE;                     /*!<Last Error Code Interrupt Enable */
             uint32_t __RESERVED1[3];
        __IO uint32_t ERRIE;                     /*!<Error Interrupt Enable */
        __IO uint32_t WKUIE;                     /*!<Wakeup Interrupt Enable */
        __IO uint32_t SLKIE;                     /*!<Sleep Interrupt Enable */
             uint32_t __RESERVED2[14];
    } IER;                                   /*!< CAN interrupt enable register,       Address offset: 0x14          */
    struct {
        __IO uint32_t EWGF;                      /*!<Error Warning Flag */
        __IO uint32_t EPVF;                      /*!<Error Passive Flag */
        __IO uint32_t BOFF;                      /*!<Bus-Off Flag */
             uint32_t __RESERVED0;
        __IO uint32_t LEC[3];                    /*!<LEC[2:0] bits (Last Error Code) */
             uint32_t __RESERVED1[9];
        __IO uint32_t TEC[8];                    /*!<Least significant byte of the 9-bit Transmit Error Counter */
        __IO uint32_t REC[8];                    /*!<Receive Error Counter */
    } ESR;                                   /*!< CAN error status register,           Address offset: 0x18          */
    struct {
        __IO uint32_t BRP[10];                   /*!<Baud Rate Prescaler */
             uint32_t __RESERVED0[6];
        __IO uint32_t TS1[4];                    /*!<Time Segment 1 */
        __IO uint32_t TS2[3];                    /*!<Time Segment 2 */
             uint32_t __RESERVED1;
        __IO uint32_t SJW[2];                    /*!<Resynchronization Jump Width */
             uint32_t __RESERVED2[4];
        __IO uint32_t LBKM;                      /*!<Loop Back Mode (Debug) */
        __IO uint32_t SILM;                      /*!<Silent Mode */
    } BTR;                                   /*!< CAN bit timing register,             Address offset: 0x1C          */
         uint32_t __RESERVED0[88][32];           /*!< Reserved, 0x020 - 0x17F                                            */
    struct {
        struct {
            __IO uint32_t TXRQ;                      /*!<Transmit Mailbox Request */
            __IO uint32_t RTR;                       /*!<Remote Transmission Request */
            __IO uint32_t IDE;                       /*!<Identifier Extension */
            __IO uint32_t EXID[18];                  /*!<Extended Identifier */
            __IO uint32_t STID[11];                  /*!<Standard Identifier or Extended Identifier */
        } TIR;                                   /*!< CAN TX mailbox identifier register */
        struct {
            __IO uint32_t DLC[4];                    /*!<Data Length Code */
                 uint32_t __RESERVED0[4];
            __IO uint32_t TGT;                       /*!<Transmit Global Time */
                 uint32_t __RESERVED1[7];
            __IO uint32_t TIME[16];                  /*!<Message Time Stamp */
        } TDTR;                                  /*!< CAN mailbox data length control and time stamp register */
        struct {
            __IO uint32_t DATA0[8];                  /*!<Data byte 0 */
            __IO uint32_t DATA1[8];                  /*!<Data byte 1 */
            __IO uint32_t DATA2[8];                  /*!<Data byte 2 */
            __IO uint32_t DATA3[8];                  /*!<Data byte 3 */
        } TDLR;                                  /*!< CAN mailbox data low register */
        struct {
            __IO uint32_t DATA4[8];                  /*!<Data byte 4 */
            __IO uint32_t DATA5[8];                  /*!<Data byte 5 */
            __IO uint32_t DATA6[8];                  /*!<Data byte 6 */
            __IO uint32_t DATA7[8];                  /*!<Data byte 7 */
        } TDHR;                                  /*!< CAN mailbox data high register */
    } sTxMailBox[3];                            /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
    struct {
        struct {
                 uint32_t __RESERVED0;
            __IO uint32_t RTR;                       /*!<Remote Transmission Request */
            __IO uint32_t IDE;                       /*!<Identifier Extension */
            __IO uint32_t EXID[18];                  /*!<Extended Identifier */
            __IO uint32_t STID[11];                  /*!<Standard Identifier or Extended Identifier */
        } RIR;                                   /*!< CAN receive FIFO mailbox identifier register */
        struct {
            __IO uint32_t DLC[4];                    /*!<Data Length Code */
                 uint32_t __RESERVED0[4];
            __IO uint32_t FMI[8];                    /*!<Filter Match Index */
            __IO uint32_t TIME[16];                  /*!<Message Time Stamp */
        } RDTR;                                  /*!< CAN receive FIFO mailbox data length control and time stamp register */
        struct {
            __IO uint32_t DATA0[8];                  /*!<Data byte 0 */
            __IO uint32_t DATA1[8];                  /*!<Data byte 1 */
            __IO uint32_t DATA2[8];                  /*!<Data byte 2 */
            __IO uint32_t DATA3[8];                  /*!<Data byte 3 */
        } RDLR;                                  /*!< CAN receive FIFO mailbox data low register */
        struct {
            __IO uint32_t DATA4[8];                  /*!<Data byte 4 */
            __IO uint32_t DATA5[8];                  /*!<Data byte 5 */
            __IO uint32_t DATA6[8];                  /*!<Data byte 6 */
            __IO uint32_t DATA7[8];                  /*!<Data byte 7 */
        } RDHR;                                  /*!< CAN receive FIFO mailbox data high register */
    } sFIFOMailBox[2];                          /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
         uint32_t __RESERVED1[12][32];           /*!< Reserved, 0x1D0 - 0x1FF                                            */
    struct {
        __IO uint32_t FINIT;                     /*!<Filter Init Mode */
             uint32_t __RESERVED0[7];
        __IO uint32_t CAN2SB[6];                 /*!<CAN2 start bank */
             uint32_t __RESERVED1[18];
    } FMR;                                   /*!< CAN filter master register,          Address offset: 0x200         */
    __IO uint32_t FM1R[32];                  /*!< CAN filter mode register,            Address offset: 0x204         */
         uint32_t __RESERVED2[32];           /*!< Reserved, 0x208                                                    */
    __IO uint32_t FS1R[32];                  /*!< CAN filter scale register,           Address offset: 0x20C         */
         uint32_t __RESERVED3[32];           /*!< Reserved, 0x210                                                    */
    __IO uint32_t FFA1R[32];                 /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
         uint32_t __RESERVED4[32];           /*!< Reserved, 0x218                                                    */
    __IO uint32_t FA1R[32];                  /*!< CAN filter activation register,      Address offset: 0x21C         */
         uint32_t __RESERVED5[8][32];           /*!< Reserved, 0x220-0x23F                                              */
    struct {
        __IO uint32_t FR1[32];                   /*!< CAN Filter bank register 1 */
        __IO uint32_t FR2[32];                   /*!< CAN Filter bank register 1 */
    } sFilterRegister[28];                       /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_BitBand_TypeDef;



/** 
  * @brief CRC calculation unit 
  */


typedef struct {
    __IO uint32_t DR;                        /*!< CRC Data register,             Address offset: 0x00 */
    __IO uint8_t IDR;                        /*!< CRC Independent data register, Address offset: 0x04 */
         uint8_t __RESERVED0;                /*!< Reserved, 0x05                                      */
         uint16_t __RESERVED1;               /*!< Reserved, 0x06                                      */
    union {
        struct {
            __IO uint32_t RESET : 1;                 /*!< RESET bit */
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;


typedef struct {
    __IO uint32_t DR[32];                    /*!< CRC Data register,             Address offset: 0x00 */
    __IO uint32_t IDR[8];                    /*!< CRC Independent data register, Address offset: 0x04 */
         uint32_t __RESERVED0[8];            /*!< Reserved, 0x05                                      */
         uint32_t __RESERVED1[16];           /*!< Reserved, 0x06                                      */
    struct {
        __IO uint32_t RESET;                     /*!< RESET bit */
             uint32_t __RESERVED0[31];
    } CR;                                    /*!< CRC Control register,          Address offset: 0x08 */
} CRC_BitBand_TypeDef;



/** 
  * @brief Digital to Analog Converter
  */


typedef struct {
    union {
        struct {
            __IO uint32_t EN1 : 1;                   /*!<DAC channel1 enable */
            __IO uint32_t BOFF1 : 1;                 /*!<DAC channel1 output buffer disable */
            __IO uint32_t TEN1 : 1;                  /*!<DAC channel1 Trigger enable */
            __IO uint32_t TSEL1 : 3;                 /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
            __IO uint32_t WAVE1 : 2;                 /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
            __IO uint32_t MAMP1 : 4;                 /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
            __IO uint32_t DMAEN1 : 1;                /*!<DAC channel1 DMA enable */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t EN2 : 1;                   /*!<DAC channel2 enable */
            __IO uint32_t BOFF2 : 1;                 /*!<DAC channel2 output buffer disable */
            __IO uint32_t TEN2 : 1;                  /*!<DAC channel2 Trigger enable */
            __IO uint32_t TSEL2 : 3;                 /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
            __IO uint32_t WAVE2 : 2;                 /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
            __IO uint32_t MAMP2 : 4;                 /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
            __IO uint32_t DMAEN2 : 1;                /*!<DAC channel2 DMA enabled */
                 uint32_t __RESERVED1 : 3;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< DAC control register,                                    Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t SWTRIG1 : 1;               /*!<DAC channel1 software trigger */
            __IO uint32_t SWTRIG2 : 1;               /*!<DAC channel2 software trigger */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } SWTRIGR;                               /*!< DAC software trigger register,                           Address offset: 0x04 */
    struct {
        __IO uint32_t D12R;                      /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L;                      /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R;                       /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR1;                                  /*!< DAC channel 1 data holding registers */
    struct {
        __IO uint32_t D12R;                      /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L;                      /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R;                       /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR2;                                  /*!< DAC channel 2 data holding registers */
    struct {
        __IO uint32_t D12R;                      /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L;                      /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R;                       /*!< DAC channel 8-bit right-aligned data holding register */
    } DHRD;                                  /*!< DAC dual channel data holding registers */
    union {
        struct {
            __IO uint32_t DACC1DOR : 12;             /*!<DAC channel1 data output */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DOR1;                                  /*!< DAC channel1 data output register,                       Address offset: 0x2C */
    union {
        struct {
            __IO uint32_t DACC2DOR : 12;             /*!<DAC channel2 data output */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DOR2;                                  /*!< DAC channel2 data output register,                       Address offset: 0x30 */
    union {
        struct {
                 uint32_t __RESERVED0 : 13;
            __IO uint32_t DMAUDR1 : 1;               /*!<DAC channel1 DMA underrun flag */
                 uint32_t __RESERVED1 : 15;
            __IO uint32_t DMAUDR2 : 1;               /*!<DAC channel2 DMA underrun flag */
                 uint32_t __RESERVED2 : 2;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN1;                       /*!<DAC channel1 enable */
        __IO uint32_t BOFF1;                     /*!<DAC channel1 output buffer disable */
        __IO uint32_t TEN1;                      /*!<DAC channel1 Trigger enable */
        __IO uint32_t TSEL1[3];                  /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
        __IO uint32_t WAVE1[2];                  /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
        __IO uint32_t MAMP1[4];                  /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
        __IO uint32_t DMAEN1;                    /*!<DAC channel1 DMA enable */
             uint32_t __RESERVED0[3];
        __IO uint32_t EN2;                       /*!<DAC channel2 enable */
        __IO uint32_t BOFF2;                     /*!<DAC channel2 output buffer disable */
        __IO uint32_t TEN2;                      /*!<DAC channel2 Trigger enable */
        __IO uint32_t TSEL2[3];                  /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
        __IO uint32_t WAVE2[2];                  /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
        __IO uint32_t MAMP2[4];                  /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
        __IO uint32_t DMAEN2;                    /*!<DAC channel2 DMA enabled */
             uint32_t __RESERVED1[3];
    } CR;                                    /*!< DAC control register,                                    Address offset: 0x00 */
    struct {
        __IO uint32_t SWTRIG1;                   /*!<DAC channel1 software trigger */
        __IO uint32_t SWTRIG2;                   /*!<DAC channel2 software trigger */
             uint32_t __RESERVED0[30];
    } SWTRIGR;                               /*!< DAC software trigger register,                           Address offset: 0x04 */
    struct {
        __IO uint32_t D12R[32];                  /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L[32];                  /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R[32];                   /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR1;                                  /*!< DAC channel 1 data holding registers */
    struct {
        __IO uint32_t D12R[32];                  /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L[32];                  /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R[32];                   /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR2;                                  /*!< DAC channel 2 data holding registers */
    struct {
        __IO uint32_t D12R[32];                  /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L[32];                  /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R[32];                   /*!< DAC channel 8-bit right-aligned data holding register */
    } DHRD;                                  /*!< DAC dual channel data holding registers */
    struct {
        __IO uint32_t DACC1DOR[12];              /*!<DAC channel1 data output */
             uint32_t __RESERVED0[20];
    } DOR1;                                  /*!< DAC channel1 data output register,                       Address offset: 0x2C */
    struct {
        __IO uint32_t DACC2DOR[12];              /*!<DAC channel2 data output */
             uint32_t __RESERVED0[20];
    } DOR2;                                  /*!< DAC channel2 data output register,                       Address offset: 0x30 */
    struct {
             uint32_t __RESERVED0[13];
        __IO uint32_t DMAUDR1;                   /*!<DAC channel1 DMA underrun flag */
             uint32_t __RESERVED1[15];
        __IO uint32_t DMAUDR2;                   /*!<DAC channel2 DMA underrun flag */
             uint32_t __RESERVED2[2];
    } SR;                                    /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_BitBand_TypeDef;



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
    } IDCODE;                                /*!< MCU device ID code,               Address offset: 0x00 */
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
    } CR;                                    /*!< Debug MCU configuration register, Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t DBG_TIM2_STOP : 1;
            __IO uint32_t DBG_TIM3_STOP : 1;
            __IO uint32_t DBG_TIM4_STOP : 1;
            __IO uint32_t DBG_TIM5_STOP : 1;
            __IO uint32_t DBG_TIM6_STOP : 1;
            __IO uint32_t DBG_TIM7_STOP : 1;
            __IO uint32_t DBG_TIM12_STOP : 1;
            __IO uint32_t DBG_TIM13_STOP : 1;
            __IO uint32_t DBG_TIM14_STOP : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DBG_RTC_STOP : 1;
            __IO uint32_t DBG_WWDG_STOP : 1;
            __IO uint32_t DBG_IWDG_STOP : 1;
                 uint32_t __RESERVED1 : 8;
            __IO uint32_t DBG_I2C1_SMBUS_TIMEOUT : 1;
            __IO uint32_t DBG_I2C2_SMBUS_TIMEOUT : 1;
            __IO uint32_t DBG_I2C3_SMBUS_TIMEOUT : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t DBG_CAN1_STOP : 1;
            __IO uint32_t DBG_CAN2_STOP : 1;
                 uint32_t __RESERVED3 : 5;
        } b;
        __IO uint32_t w;
    } APB1FZ;                                /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t DBG_TIM1_STOP : 1;
            __IO uint32_t DBG_TIM8_STOP : 1;
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t DBG_TIM9_STOP : 1;
            __IO uint32_t DBG_TIM10_STOP : 1;
            __IO uint32_t DBG_TIM11_STOP : 1;
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } APB2FZ;                                /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
} DBGMCU_TypeDef;


/** 
  * @brief DCMI
  */


typedef struct {
    union {
        struct {
            __IO uint32_t CAPTURE : 1;
            __IO uint32_t CM : 1;
            __IO uint32_t CROP : 1;
            __IO uint32_t JPEG : 1;
            __IO uint32_t ESS : 1;
            __IO uint32_t PCKPOL : 1;
            __IO uint32_t HSPOL : 1;
            __IO uint32_t VSPOL : 1;
            __IO uint32_t FCRC : 2;
            __IO uint32_t EDM : 2;
            __IO uint32_t CRE : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t ENABLE : 1;
                 uint32_t __RESERVED1 : 17;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< DCMI control register 1,                       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t HSYNC : 1;
            __IO uint32_t VSYNC : 1;
            __IO uint32_t FNE : 1;
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< DCMI status register,                          Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t FRAME_RIS : 1;
            __IO uint32_t OVF_RIS : 1;
            __IO uint32_t ERR_RIS : 1;
            __IO uint32_t VSYNC_RIS : 1;
            __IO uint32_t LINE_RIS : 1;
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } RISR;                                  /*!< DCMI raw interrupt status register,            Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t FRAME_IE : 1;
            __IO uint32_t OVF_IE : 1;
            __IO uint32_t ERR_IE : 1;
            __IO uint32_t VSYNC_IE : 1;
            __IO uint32_t LINE_IE : 1;
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } IER;                                   /*!< DCMI interrupt enable register,                Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t FRAME_MIS : 1;
            __IO uint32_t OVF_MIS : 1;
            __IO uint32_t ERR_MIS : 1;
            __IO uint32_t VSYNC_MIS : 1;
            __IO uint32_t LINE_MIS : 1;
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } MISR;                                  /*!< DCMI masked interrupt status register,         Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t FRAME_ISC : 1;
            __IO uint32_t OVF_ISC : 1;
            __IO uint32_t ERR_ISC : 1;
            __IO uint32_t VSYNC_ISC : 1;
            __IO uint32_t LINE_ISC : 1;
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } ICR;                                   /*!< DCMI interrupt clear register,                 Address offset: 0x14 */
    __IO uint32_t ESCR;                      /*!< DCMI embedded synchronization code register,   Address offset: 0x18 */
    __IO uint32_t ESUR;                      /*!< DCMI embedded synchronization unmask register, Address offset: 0x1C */
    __IO uint32_t CWSTRTR;                   /*!< DCMI crop window start,                        Address offset: 0x20 */
    __IO uint32_t CWSIZER;                   /*!< DCMI crop window size,                         Address offset: 0x24 */
    __IO uint32_t DR;                        /*!< DCMI data register,                            Address offset: 0x28 */
} DCMI_TypeDef;


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
    } CR;                                    /*!< DMA stream x configuration register      */
    __IO uint32_t NDTR;                      /*!< DMA stream x number of data register     */
    __IO uint32_t PAR;                       /*!< DMA stream x peripheral address register */
    __IO uint32_t M0AR;                      /*!< DMA stream x memory 0 address register   */
    __IO uint32_t M1AR;                      /*!< DMA stream x memory 1 address register   */
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
    } FCR;                                   /*!< DMA stream x FIFO control register       */
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
    } CR;                                    /*!< DMA stream x configuration register      */
    __IO uint32_t NDTR[32];                  /*!< DMA stream x number of data register     */
    __IO uint32_t PAR[32];                   /*!< DMA stream x peripheral address register */
    __IO uint32_t M0AR[32];                  /*!< DMA stream x memory 0 address register   */
    __IO uint32_t M1AR[32];                  /*!< DMA stream x memory 1 address register   */
    struct {
        __IO uint32_t FTH[2];
        __IO uint32_t DMDIS;
        __IO uint32_t FS[3];
             uint32_t __RESERVED0;
        __IO uint32_t FEIE;
             uint32_t __RESERVED1[24];
    } FCR;                                   /*!< DMA stream x FIFO control register       */
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
    } LISR;                                  /*!< DMA low interrupt status register,      Address offset: 0x00 */
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
    } HISR;                                  /*!< DMA high interrupt status register,     Address offset: 0x04 */
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
    } LIFCR;                                 /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
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
    } HIFCR;                                 /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
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
    } LISR;                                  /*!< DMA low interrupt status register,      Address offset: 0x00 */
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
    } HISR;                                  /*!< DMA high interrupt status register,     Address offset: 0x04 */
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
    } LIFCR;                                 /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
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
    } HIFCR;                                 /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_BitBand_TypeDef;




/** 
  * @brief Ethernet MAC
  */


typedef struct {
    union {
        struct {
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t RE : 1;                    /* Receiver enable */
            __IO uint32_t TE : 1;                    /* Transmitter enable */
            __IO uint32_t DC : 1;                    /* Defferal check */
            __IO uint32_t BL : 2;                    /* Back-off limit: random integer number (r) of slot time delays before rescheduling
                                                                   a transmission attempt during retries after a collision: 0 =< r <2^k */
            __IO uint32_t APCS : 1;                  /* Automatic Pad/CRC stripping */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t RD : 1;                    /* Retry disable */
            __IO uint32_t IPCO : 1;                  /* IP Checksum offload */
            __IO uint32_t DM : 1;                    /* Duplex mode */
            __IO uint32_t LM : 1;                    /* loopback mode */
            __IO uint32_t ROD : 1;                   /* Receive own disable */
            __IO uint32_t FES : 1;                   /* Fast ethernet speed */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t CSD : 1;                   /* Carrier sense disable (during transmission) */
            __IO uint32_t IFG : 3;                   /* Inter-frame gap */
                 uint32_t __RESERVED3 : 2;
            __IO uint32_t JD : 1;                    /* Jabber disable */
            __IO uint32_t WD : 1;                    /* Watchdog disable */
                 uint32_t __RESERVED4 : 8;
        } b;
        __IO uint32_t w;
    } MACCR;
    union {
        struct {
            __IO uint32_t PM : 1;                    /* Promiscuous mode */
            __IO uint32_t HU : 1;                    /* Hash unicast */
            __IO uint32_t HM : 1;                    /* Hash multicast */
            __IO uint32_t DAIF : 1;                  /* DA Inverse filtering */
            __IO uint32_t PAM : 1;                   /* Pass all mutlicast */
            __IO uint32_t BFD : 1;                   /* Broadcast frame disable */
            __IO uint32_t PCF : 2;                   /* Pass control frames: 3 cases */
            __IO uint32_t SAIF : 1;                  /* SA inverse filtering */
            __IO uint32_t SAF : 1;                   /* Source address filter enable */
            __IO uint32_t HPF : 1;                   /* Hash or perfect filter */
                 uint32_t __RESERVED0 : 20;
            __IO uint32_t RA : 1;                    /* Receive all */
        } b;
        __IO uint32_t w;
    } MACFFR;
    __IO uint32_t MACHTHR;
    __IO uint32_t MACHTLR;
    union {
        struct {
            __IO uint32_t MB : 1;                    /* MII busy */
            __IO uint32_t MW : 1;                    /* MII write */
            __IO uint32_t CR : 3;                    /* CR clock range: 6 cases */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t MR : 5;                    /* MII register in the selected PHY */
            __IO uint32_t PA : 5;                    /* Physical layer address */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } MACMIIAR;
    union {
        struct {
            __IO uint32_t MD : 16;                   /* MII data: read/write data from/to PHY */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } MACMIIDR;
    union {
        struct {
            __IO uint32_t FCBBPA : 1;                /* Flow control busy/backpressure activate */
            __IO uint32_t TFCE : 1;                  /* Transmit flow control enable */
            __IO uint32_t RFCE : 1;                  /* Receive flow control enable */
            __IO uint32_t UPFD : 1;                  /* Unicast pause frame detect */
            __IO uint32_t PLT : 2;                   /* Pause low threshold: 4 cases */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t ZQPD : 1;                  /* Zero-quanta pause disable */
                 uint32_t __RESERVED1 : 8;
            __IO uint32_t PT : 16;                   /* Pause time */
        } b;
        __IO uint32_t w;
    } MACFCR;
    union {
        struct {
            __IO uint32_t VLANTI : 16;               /* VLAN tag identifier (for receive frames) */
            __IO uint32_t VLANTC : 1;                /* 12-bit VLAN tag comparison */
                 uint32_t __RESERVED0 : 15;
        } b;
        __IO uint32_t w;
    } MACVLANTR;                             /*    8 */
         uint32_t __RESERVED0[2];
    __IO uint32_t MACRWUFFR;                 /*   11 */
    union {
        struct {
            __IO uint32_t PD : 1;                    /* Power Down */
            __IO uint32_t MPE : 1;                   /* Magic Packet Enable */
            __IO uint32_t WFE : 1;                   /* Wake-Up Frame Enable */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t MPR : 1;                   /* Magic Packet Received */
            __IO uint32_t WFR : 1;                   /* Wake-Up Frame Received */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t GU : 1;                    /* Global Unicast */
                 uint32_t __RESERVED2 : 21;
            __IO uint32_t WFFRPR : 1;                /* Wake-Up Frame Filter Register Pointer Reset */
        } b;
        __IO uint32_t w;
    } MACPMTCSR;
         uint32_t __RESERVED1[2];
    union {
        struct {
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t PMTS : 1;                  /* PMT status */
            __IO uint32_t MMCS : 1;                  /* MMC status */
            __IO uint32_t MMMCRS : 1;                /* MMC receive status */
            __IO uint32_t MMCTS : 1;                 /* MMC transmit status */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t TSTS : 1;                  /* Time stamp trigger status */
                 uint32_t __RESERVED2 : 22;
        } b;
        __IO uint32_t w;
    } MACSR;                                 /*   15 */
    union {
        struct {
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t PMTIM : 1;                 /* PMT interrupt mask */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t TSTIM : 1;                 /* Time stamp trigger interrupt mask */
                 uint32_t __RESERVED2 : 22;
        } b;
        __IO uint32_t w;
    } MACIMR;
    union {
        struct {
            __IO uint32_t MACA0H : 16;               /* MAC address0 high */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } MACA0HR;
    __IO uint32_t MACA0LR;
    union {
        struct {
            __IO uint32_t MACA1H : 16;               /* MAC address1 high */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t MBC : 6;                   /* Mask byte control: bits to mask for comparison of the MAC Address bytes */
            __IO uint32_t SA : 1;                    /* Source address */
            __IO uint32_t AE : 1;                    /* Address enable */
        } b;
        __IO uint32_t w;
    } MACA1HR;
    __IO uint32_t MACA1LR;
    union {
        struct {
            __IO uint32_t MACA2H : 16;               /* MAC address1 high */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t MBC : 6;                   /* Mask byte control */
            __IO uint32_t SA : 1;                    /* Source address */
            __IO uint32_t AE : 1;                    /* Address enable */
        } b;
        __IO uint32_t w;
    } MACA2HR;
    __IO uint32_t MACA2LR;
    union {
        struct {
            __IO uint32_t MACA3H : 16;               /* MAC address3 high */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t MBC : 6;                   /* Mask byte control */
            __IO uint32_t SA : 1;                    /* Source address */
            __IO uint32_t AE : 1;                    /* Address enable */
        } b;
        __IO uint32_t w;
    } MACA3HR;
    __IO uint32_t MACA3LR;                   /*   24 */
         uint32_t __RESERVED2[40];
    union {
        struct {
            __IO uint32_t CR : 1;                    /* Counters Reset */
            __IO uint32_t CSR : 1;                   /* Counter Stop Rollover */
            __IO uint32_t ROR : 1;                   /* Reset on Read */
            __IO uint32_t MCF : 1;                   /* MMC Counter Freeze */
            __IO uint32_t MCP : 1;                   /* MMC counter preset */
            __IO uint32_t MCFHP : 1;                 /* MMC counter Full-Half preset */
                 uint32_t __RESERVED0 : 26;
        } b;
        __IO uint32_t w;
    } MMCCR;                                 /*   65 */
    union {
        struct {
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RFCES : 1;                 /* Set when Rx crc error counter reaches half the maximum value */
            __IO uint32_t RFAES : 1;                 /* Set when Rx alignment error counter reaches half the maximum value */
                 uint32_t __RESERVED1 : 10;
            __IO uint32_t RGUFS : 1;                 /* Set when Rx good unicast frames counter reaches half the maximum value */
                 uint32_t __RESERVED2 : 14;
        } b;
        __IO uint32_t w;
    } MMCRIR;
    union {
        struct {
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t TGFSCS : 1;                /* Set when Tx good single col counter reaches half the maximum value */
            __IO uint32_t TGFMSCS : 1;               /* Set when Tx good multi col counter reaches half the maximum value */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t TGFS : 1;                  /* Set when Tx good frame count counter reaches half the maximum value */
                 uint32_t __RESERVED2 : 10;
        } b;
        __IO uint32_t w;
    } MMCTIR;
    union {
        struct {
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RFCEM : 1;                 /* Mask the interrupt when Rx crc error counter reaches half the maximum value */
            __IO uint32_t RFAEM : 1;                 /* Mask the interrupt when when Rx alignment error counter reaches half the maximum value */
                 uint32_t __RESERVED1 : 10;
            __IO uint32_t RGUFM : 1;                 /* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value */
                 uint32_t __RESERVED2 : 14;
        } b;
        __IO uint32_t w;
    } MMCRIMR;
    union {
        struct {
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t TGFSCM : 1;                /* Mask the interrupt when Tx good single col counter reaches half the maximum value */
            __IO uint32_t TGFMSCM : 1;               /* Mask the interrupt when Tx good multi col counter reaches half the maximum value */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t TGFM : 1;                  /* Mask the interrupt when Tx good frame count counter reaches half the maximum value */
                 uint32_t __RESERVED2 : 10;
        } b;
        __IO uint32_t w;
    } MMCTIMR;                               /*   69 */
         uint32_t __RESERVED3[14];
    __IO uint32_t MMCTGFSCCR;                /*   84 */
    __IO uint32_t MMCTGFMSCCR;
         uint32_t __RESERVED4[5];
    __IO uint32_t MMCTGFCR;
         uint32_t __RESERVED5[10];
    __IO uint32_t MMCRFCECR;
    __IO uint32_t MMCRFAECR;
         uint32_t __RESERVED6[10];
    __IO uint32_t MMCRGUFCR;
         uint32_t __RESERVED7[334];
    union {
        struct {
            __IO uint32_t TSE : 1;                   /* Time stamp enable */
            __IO uint32_t TSFCU : 1;                 /* Time stamp fine or coarse update */
            __IO uint32_t TSSTI : 1;                 /* Time stamp initialize */
            __IO uint32_t TSSTU : 1;                 /* Time stamp update */
            __IO uint32_t TSITE : 1;                 /* Time stamp interrupt trigger enable */
            __IO uint32_t TSARU : 1;                 /* Addend register update */
                 uint32_t __RESERVED0 : 10;
            __IO uint32_t TSCNT : 2;                 /* Time stamp clock node type */
                 uint32_t __RESERVED1 : 14;
        } b;
        __IO uint32_t w;
    } PTPTSCR;
    union {
        struct {
            __IO uint32_t STSSI : 8;                 /* System time Sub-second increment value */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } PTPSSIR;
    __IO uint32_t PTPTSHR;
    union {
        struct {
            __IO uint32_t STSS : 31;                 /* System Time sub-seconds */
            __IO uint32_t STPNS : 1;                 /* System Time Positive or negative time */
        } b;
        __IO uint32_t w;
    } PTPTSLR;
    __IO uint32_t PTPTSHUR;
    union {
        struct {
            __IO uint32_t TSUSS : 31;                /* Time stamp update sub-seconds */
            __IO uint32_t TSUPNS : 1;                /* Time stamp update Positive or negative time */
        } b;
        __IO uint32_t w;
    } PTPTSLUR;
    __IO uint32_t PTPTSAR;
    __IO uint32_t PTPTTHR;
    __IO uint32_t PTPTTLR;
    __IO uint32_t RESERVED8;
    union {
        struct {
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t TSSO : 1;                  /* Time stamp seconds overflow */
            __IO uint32_t TSTTR : 1;                 /* Time stamp target time reached */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t TSSARFE : 1;               /* Time stamp snapshot for all received frames enable */
            __IO uint32_t TSSSR : 1;                 /* Time stamp Sub-seconds rollover */
            __IO uint32_t TSPTPPSV2E : 1;            /* Time stamp PTP packet snooping for version2 format enable */
            __IO uint32_t TSSPTPOEFE : 1;            /* Time stamp snapshot for PTP over ethernet frames enable */
            __IO uint32_t TSSIPV6FE : 1;             /* Time stamp snapshot for IPv6 frames enable */
            __IO uint32_t TSSIPV4FE : 1;             /* Time stamp snapshot for IPv4 frames enable */
            __IO uint32_t TSSEME : 1;                /* Time stamp snapshot for event message enable */
            __IO uint32_t TSSMRME : 1;               /* Time stamp snapshot for message relevant to master enable */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } PTPTSSR;
         uint32_t __RESERVED8[565];
    union {
        struct {
            __IO uint32_t SR : 1;                    /* Software reset */
            __IO uint32_t DA : 1;                    /* DMA arbitration scheme */
            __IO uint32_t DSL : 5;                   /* Descriptor Skip Length */
            __IO uint32_t EDE : 1;                   /* Enhanced Descriptor Enable */
            __IO uint32_t PBL : 6;                   /* Programmable burst length */
            __IO uint32_t RTPR : 2;                  /* Rx Tx priority ratio */
            __IO uint32_t FB : 1;                    /* Fixed Burst */
            __IO uint32_t RDP : 6;                   /* RxDMA PBL */
            __IO uint32_t USP : 1;                   /* Use separate PBL */
            __IO uint32_t FPM : 1;                   /* 4xPBL mode */
            __IO uint32_t AAB : 1;                   /* Address-Aligned beats */
                 uint32_t __RESERVED0 : 6;
        } b;
        __IO uint32_t w;
    } DMABMR;
    __IO uint32_t DMATPDR;
    __IO uint32_t DMARPDR;
    __IO uint32_t DMARDLAR;
    __IO uint32_t DMATDLAR;
    union {
        struct {
            __IO uint32_t TS : 1;                    /* Transmit status */
            __IO uint32_t TPSS : 1;                  /* Transmit process stopped status */
            __IO uint32_t TBUS : 1;                  /* Transmit buffer unavailable status */
            __IO uint32_t TJTS : 1;                  /* Transmit jabber timeout status */
            __IO uint32_t ROS : 1;                   /* Receive overflow status */
            __IO uint32_t TUS : 1;                   /* Transmit underflow status */
            __IO uint32_t RS : 1;                    /* Receive status */
            __IO uint32_t RBUS : 1;                  /* Receive buffer unavailable status */
            __IO uint32_t RPSS : 1;                  /* Receive process stopped status */
            __IO uint32_t RWTS : 1;                  /* Receive watchdog timeout status */
            __IO uint32_t ETS : 1;                   /* Early transmit status */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t FBES : 1;                  /* Fatal bus error status */
            __IO uint32_t ERS : 1;                   /* Early receive status */
            __IO uint32_t AIS : 1;                   /* Abnormal interrupt summary */
            __IO uint32_t NIS : 1;                   /* Normal interrupt summary */
            __IO uint32_t RPS : 3;                   /* Receive process state */
            __IO uint32_t TPS : 3;                   /* Transmit process state */
            __IO uint32_t EBS : 3;                   /* Error bits status */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t MMCS : 1;                  /* MMC status */
            __IO uint32_t PMTS : 1;                  /* PMT status */
            __IO uint32_t TSTS : 1;                  /* Time-stamp trigger status */
                 uint32_t __RESERVED2 : 2;
        } b;
        __IO uint32_t w;
    } DMASR;
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SR : 1;                    /* Start/stop receive */
            __IO uint32_t OSF : 1;                   /* operate on second frame */
            __IO uint32_t RTC : 2;                   /* receive threshold control */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t FUGF : 1;                  /* Forward undersized good frames */
            __IO uint32_t FEF : 1;                   /* Forward error frames */
                 uint32_t __RESERVED2 : 5;
            __IO uint32_t ST : 1;                    /* Start/stop transmission command */
            __IO uint32_t TTC : 3;                   /* Transmit threshold control */
                 uint32_t __RESERVED3 : 3;
            __IO uint32_t FTF : 1;                   /* Flush transmit FIFO */
            __IO uint32_t TSF : 1;                   /* Transmit store and forward */
                 uint32_t __RESERVED4 : 2;
            __IO uint32_t DFRF : 1;                  /* Disable flushing of received frames */
            __IO uint32_t RSF : 1;                   /* Receive store and forward */
            __IO uint32_t DTCEFD : 1;                /* Disable Dropping of TCP/IP checksum error frames */
                 uint32_t __RESERVED5 : 5;
        } b;
        __IO uint32_t w;
    } DMAOMR;
    union {
        struct {
            __IO uint32_t TIE : 1;                   /* Transmit interrupt enable */
            __IO uint32_t TPSIE : 1;                 /* Transmit process stopped interrupt enable */
            __IO uint32_t TBUIE : 1;                 /* Transmit buffer unavailable interrupt enable */
            __IO uint32_t TJTIE : 1;                 /* Transmit jabber timeout interrupt enable */
            __IO uint32_t ROIE : 1;                  /* Receive Overflow interrupt enable */
            __IO uint32_t TUIE : 1;                  /* Transmit Underflow interrupt enable */
            __IO uint32_t RIE : 1;                   /* Receive interrupt enable */
            __IO uint32_t RBUIE : 1;                 /* Receive buffer unavailable interrupt enable */
            __IO uint32_t RPSIE : 1;                 /* Receive process stopped interrupt enable */
            __IO uint32_t RWTIE : 1;                 /* Receive watchdog timeout interrupt enable */
            __IO uint32_t ETIE : 1;                  /* Early transmit interrupt enable */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t FBEIE : 1;                 /* Fatal bus error interrupt enable */
            __IO uint32_t ERIE : 1;                  /* Early receive interrupt enable */
            __IO uint32_t AISE : 1;                  /* Abnormal interrupt summary enable */
            __IO uint32_t NISE : 1;                  /* Normal interrupt summary enable */
                 uint32_t __RESERVED1 : 15;
        } b;
        __IO uint32_t w;
    } DMAIER;
    union {
        struct {
            __IO uint32_t MFC : 16;                  /* Number of frames missed by the controller */
            __IO uint32_t OMFC : 1;                  /* Overflow bit for missed frame counter */
            __IO uint32_t MFA : 11;                  /* Number of frames missed by the application */
            __IO uint32_t OFOC : 1;                  /* Overflow bit for FIFO overflow counter */
                 uint32_t __RESERVED0 : 3;
        } b;
        __IO uint32_t w;
    } DMAMFBOCR;
    __IO uint32_t DMARSWTR;
         uint32_t __RESERVED9[8];
    __IO uint32_t DMACHTDR;
    __IO uint32_t DMACHRDR;
    __IO uint32_t DMACHTBAR;
    __IO uint32_t DMACHRBAR;
} ETH_TypeDef;


typedef struct {
    struct {
             uint32_t __RESERVED0[2];
        __IO uint32_t RE;                        /* Receiver enable */
        __IO uint32_t TE;                        /* Transmitter enable */
        __IO uint32_t DC;                        /* Defferal check */
        __IO uint32_t BL[2];                     /* Back-off limit: random integer number (r) of slot time delays before rescheduling
                                                               a transmission attempt during retries after a collision: 0 =< r <2^k */
        __IO uint32_t APCS;                      /* Automatic Pad/CRC stripping */
             uint32_t __RESERVED1;
        __IO uint32_t RD;                        /* Retry disable */
        __IO uint32_t IPCO;                      /* IP Checksum offload */
        __IO uint32_t DM;                        /* Duplex mode */
        __IO uint32_t LM;                        /* loopback mode */
        __IO uint32_t ROD;                       /* Receive own disable */
        __IO uint32_t FES;                       /* Fast ethernet speed */
             uint32_t __RESERVED2;
        __IO uint32_t CSD;                       /* Carrier sense disable (during transmission) */
        __IO uint32_t IFG[3];                    /* Inter-frame gap */
             uint32_t __RESERVED3[2];
        __IO uint32_t JD;                        /* Jabber disable */
        __IO uint32_t WD;                        /* Watchdog disable */
             uint32_t __RESERVED4[8];
    } MACCR;
    struct {
        __IO uint32_t PM;                        /* Promiscuous mode */
        __IO uint32_t HU;                        /* Hash unicast */
        __IO uint32_t HM;                        /* Hash multicast */
        __IO uint32_t DAIF;                      /* DA Inverse filtering */
        __IO uint32_t PAM;                       /* Pass all mutlicast */
        __IO uint32_t BFD;                       /* Broadcast frame disable */
        __IO uint32_t PCF[2];                    /* Pass control frames: 3 cases */
        __IO uint32_t SAIF;                      /* SA inverse filtering */
        __IO uint32_t SAF;                       /* Source address filter enable */
        __IO uint32_t HPF;                       /* Hash or perfect filter */
             uint32_t __RESERVED0[20];
        __IO uint32_t RA;                        /* Receive all */
    } MACFFR;
    __IO uint32_t MACHTHR[32];
    __IO uint32_t MACHTLR[32];
    struct {
        __IO uint32_t MB;                        /* MII busy */
        __IO uint32_t MW;                        /* MII write */
        __IO uint32_t CR[3];                     /* CR clock range: 6 cases */
             uint32_t __RESERVED0;
        __IO uint32_t MR[5];                     /* MII register in the selected PHY */
        __IO uint32_t PA[5];                     /* Physical layer address */
             uint32_t __RESERVED1[16];
    } MACMIIAR;
    struct {
        __IO uint32_t MD[16];                    /* MII data: read/write data from/to PHY */
             uint32_t __RESERVED0[16];
    } MACMIIDR;
    struct {
        __IO uint32_t FCBBPA;                    /* Flow control busy/backpressure activate */
        __IO uint32_t TFCE;                      /* Transmit flow control enable */
        __IO uint32_t RFCE;                      /* Receive flow control enable */
        __IO uint32_t UPFD;                      /* Unicast pause frame detect */
        __IO uint32_t PLT[2];                    /* Pause low threshold: 4 cases */
             uint32_t __RESERVED0;
        __IO uint32_t ZQPD;                      /* Zero-quanta pause disable */
             uint32_t __RESERVED1[8];
        __IO uint32_t PT[16];                    /* Pause time */
    } MACFCR;
    struct {
        __IO uint32_t VLANTI[16];                /* VLAN tag identifier (for receive frames) */
        __IO uint32_t VLANTC;                    /* 12-bit VLAN tag comparison */
             uint32_t __RESERVED0[15];
    } MACVLANTR;                             /*    8 */
         uint32_t __RESERVED0[2][32];
    __IO uint32_t MACRWUFFR[32];             /*   11 */
    struct {
        __IO uint32_t PD;                        /* Power Down */
        __IO uint32_t MPE;                       /* Magic Packet Enable */
        __IO uint32_t WFE;                       /* Wake-Up Frame Enable */
             uint32_t __RESERVED0[2];
        __IO uint32_t MPR;                       /* Magic Packet Received */
        __IO uint32_t WFR;                       /* Wake-Up Frame Received */
             uint32_t __RESERVED1[2];
        __IO uint32_t GU;                        /* Global Unicast */
             uint32_t __RESERVED2[21];
        __IO uint32_t WFFRPR;                    /* Wake-Up Frame Filter Register Pointer Reset */
    } MACPMTCSR;
         uint32_t __RESERVED1[2][32];
    struct {
             uint32_t __RESERVED0[3];
        __IO uint32_t PMTS;                      /* PMT status */
        __IO uint32_t MMCS;                      /* MMC status */
        __IO uint32_t MMMCRS;                    /* MMC receive status */
        __IO uint32_t MMCTS;                     /* MMC transmit status */
             uint32_t __RESERVED1[2];
        __IO uint32_t TSTS;                      /* Time stamp trigger status */
             uint32_t __RESERVED2[22];
    } MACSR;                                 /*   15 */
    struct {
             uint32_t __RESERVED0[3];
        __IO uint32_t PMTIM;                     /* PMT interrupt mask */
             uint32_t __RESERVED1[5];
        __IO uint32_t TSTIM;                     /* Time stamp trigger interrupt mask */
             uint32_t __RESERVED2[22];
    } MACIMR;
    struct {
        __IO uint32_t MACA0H[16];                /* MAC address0 high */
             uint32_t __RESERVED0[16];
    } MACA0HR;
    __IO uint32_t MACA0LR[32];
    struct {
        __IO uint32_t MACA1H[16];                /* MAC address1 high */
             uint32_t __RESERVED0[8];
        __IO uint32_t MBC[6];                    /* Mask byte control: bits to mask for comparison of the MAC Address bytes */
        __IO uint32_t SA;                        /* Source address */
        __IO uint32_t AE;                        /* Address enable */
    } MACA1HR;
    __IO uint32_t MACA1LR[32];
    struct {
        __IO uint32_t MACA2H[16];                /* MAC address1 high */
             uint32_t __RESERVED0[8];
        __IO uint32_t MBC[6];                    /* Mask byte control */
        __IO uint32_t SA;                        /* Source address */
        __IO uint32_t AE;                        /* Address enable */
    } MACA2HR;
    __IO uint32_t MACA2LR[32];
    struct {
        __IO uint32_t MACA3H[16];                /* MAC address3 high */
             uint32_t __RESERVED0[8];
        __IO uint32_t MBC[6];                    /* Mask byte control */
        __IO uint32_t SA;                        /* Source address */
        __IO uint32_t AE;                        /* Address enable */
    } MACA3HR;
    __IO uint32_t MACA3LR[32];               /*   24 */
         uint32_t __RESERVED2[40][32];
    struct {
        __IO uint32_t CR;                        /* Counters Reset */
        __IO uint32_t CSR;                       /* Counter Stop Rollover */
        __IO uint32_t ROR;                       /* Reset on Read */
        __IO uint32_t MCF;                       /* MMC Counter Freeze */
        __IO uint32_t MCP;                       /* MMC counter preset */
        __IO uint32_t MCFHP;                     /* MMC counter Full-Half preset */
             uint32_t __RESERVED0[26];
    } MMCCR;                                 /*   65 */
    struct {
             uint32_t __RESERVED0[5];
        __IO uint32_t RFCES;                     /* Set when Rx crc error counter reaches half the maximum value */
        __IO uint32_t RFAES;                     /* Set when Rx alignment error counter reaches half the maximum value */
             uint32_t __RESERVED1[10];
        __IO uint32_t RGUFS;                     /* Set when Rx good unicast frames counter reaches half the maximum value */
             uint32_t __RESERVED2[14];
    } MMCRIR;
    struct {
             uint32_t __RESERVED0[14];
        __IO uint32_t TGFSCS;                    /* Set when Tx good single col counter reaches half the maximum value */
        __IO uint32_t TGFMSCS;                   /* Set when Tx good multi col counter reaches half the maximum value */
             uint32_t __RESERVED1[5];
        __IO uint32_t TGFS;                      /* Set when Tx good frame count counter reaches half the maximum value */
             uint32_t __RESERVED2[10];
    } MMCTIR;
    struct {
             uint32_t __RESERVED0[5];
        __IO uint32_t RFCEM;                     /* Mask the interrupt when Rx crc error counter reaches half the maximum value */
        __IO uint32_t RFAEM;                     /* Mask the interrupt when when Rx alignment error counter reaches half the maximum value */
             uint32_t __RESERVED1[10];
        __IO uint32_t RGUFM;                     /* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value */
             uint32_t __RESERVED2[14];
    } MMCRIMR;
    struct {
             uint32_t __RESERVED0[14];
        __IO uint32_t TGFSCM;                    /* Mask the interrupt when Tx good single col counter reaches half the maximum value */
        __IO uint32_t TGFMSCM;                   /* Mask the interrupt when Tx good multi col counter reaches half the maximum value */
             uint32_t __RESERVED1[5];
        __IO uint32_t TGFM;                      /* Mask the interrupt when Tx good frame count counter reaches half the maximum value */
             uint32_t __RESERVED2[10];
    } MMCTIMR;                               /*   69 */
         uint32_t __RESERVED3[14][32];
    __IO uint32_t MMCTGFSCCR[32];            /*   84 */
    __IO uint32_t MMCTGFMSCCR[32];
         uint32_t __RESERVED4[5][32];
    __IO uint32_t MMCTGFCR[32];
         uint32_t __RESERVED5[10][32];
    __IO uint32_t MMCRFCECR[32];
    __IO uint32_t MMCRFAECR[32];
         uint32_t __RESERVED6[10][32];
    __IO uint32_t MMCRGUFCR[32];
         uint32_t __RESERVED7[334][32];
    struct {
        __IO uint32_t TSE;                       /* Time stamp enable */
        __IO uint32_t TSFCU;                     /* Time stamp fine or coarse update */
        __IO uint32_t TSSTI;                     /* Time stamp initialize */
        __IO uint32_t TSSTU;                     /* Time stamp update */
        __IO uint32_t TSITE;                     /* Time stamp interrupt trigger enable */
        __IO uint32_t TSARU;                     /* Addend register update */
             uint32_t __RESERVED0[10];
        __IO uint32_t TSCNT[2];                  /* Time stamp clock node type */
             uint32_t __RESERVED1[14];
    } PTPTSCR;
    struct {
        __IO uint32_t STSSI[8];                  /* System time Sub-second increment value */
             uint32_t __RESERVED0[24];
    } PTPSSIR;
    __IO uint32_t PTPTSHR[32];
    struct {
        __IO uint32_t STSS[31];                  /* System Time sub-seconds */
        __IO uint32_t STPNS;                     /* System Time Positive or negative time */
    } PTPTSLR;
    __IO uint32_t PTPTSHUR[32];
    struct {
        __IO uint32_t TSUSS[31];                 /* Time stamp update sub-seconds */
        __IO uint32_t TSUPNS;                    /* Time stamp update Positive or negative time */
    } PTPTSLUR;
    __IO uint32_t PTPTSAR[32];
    __IO uint32_t PTPTTHR[32];
    __IO uint32_t PTPTTLR[32];
    __IO uint32_t RESERVED8[32];
    struct {
             uint32_t __RESERVED0[4];
        __IO uint32_t TSSO;                      /* Time stamp seconds overflow */
        __IO uint32_t TSTTR;                     /* Time stamp target time reached */
             uint32_t __RESERVED1[2];
        __IO uint32_t TSSARFE;                   /* Time stamp snapshot for all received frames enable */
        __IO uint32_t TSSSR;                     /* Time stamp Sub-seconds rollover */
        __IO uint32_t TSPTPPSV2E;                /* Time stamp PTP packet snooping for version2 format enable */
        __IO uint32_t TSSPTPOEFE;                /* Time stamp snapshot for PTP over ethernet frames enable */
        __IO uint32_t TSSIPV6FE;                 /* Time stamp snapshot for IPv6 frames enable */
        __IO uint32_t TSSIPV4FE;                 /* Time stamp snapshot for IPv4 frames enable */
        __IO uint32_t TSSEME;                    /* Time stamp snapshot for event message enable */
        __IO uint32_t TSSMRME;                   /* Time stamp snapshot for message relevant to master enable */
             uint32_t __RESERVED2[16];
    } PTPTSSR;
         uint32_t __RESERVED8[565][32];
    struct {
        __IO uint32_t SR;                        /* Software reset */
        __IO uint32_t DA;                        /* DMA arbitration scheme */
        __IO uint32_t DSL[5];                    /* Descriptor Skip Length */
        __IO uint32_t EDE;                       /* Enhanced Descriptor Enable */
        __IO uint32_t PBL[6];                    /* Programmable burst length */
        __IO uint32_t RTPR[2];                   /* Rx Tx priority ratio */
        __IO uint32_t FB;                        /* Fixed Burst */
        __IO uint32_t RDP[6];                    /* RxDMA PBL */
        __IO uint32_t USP;                       /* Use separate PBL */
        __IO uint32_t FPM;                       /* 4xPBL mode */
        __IO uint32_t AAB;                       /* Address-Aligned beats */
             uint32_t __RESERVED0[6];
    } DMABMR;
    __IO uint32_t DMATPDR[32];
    __IO uint32_t DMARPDR[32];
    __IO uint32_t DMARDLAR[32];
    __IO uint32_t DMATDLAR[32];
    struct {
        __IO uint32_t TS;                        /* Transmit status */
        __IO uint32_t TPSS;                      /* Transmit process stopped status */
        __IO uint32_t TBUS;                      /* Transmit buffer unavailable status */
        __IO uint32_t TJTS;                      /* Transmit jabber timeout status */
        __IO uint32_t ROS;                       /* Receive overflow status */
        __IO uint32_t TUS;                       /* Transmit underflow status */
        __IO uint32_t RS;                        /* Receive status */
        __IO uint32_t RBUS;                      /* Receive buffer unavailable status */
        __IO uint32_t RPSS;                      /* Receive process stopped status */
        __IO uint32_t RWTS;                      /* Receive watchdog timeout status */
        __IO uint32_t ETS;                       /* Early transmit status */
             uint32_t __RESERVED0[2];
        __IO uint32_t FBES;                      /* Fatal bus error status */
        __IO uint32_t ERS;                       /* Early receive status */
        __IO uint32_t AIS;                       /* Abnormal interrupt summary */
        __IO uint32_t NIS;                       /* Normal interrupt summary */
        __IO uint32_t RPS[3];                    /* Receive process state */
        __IO uint32_t TPS[3];                    /* Transmit process state */
        __IO uint32_t EBS[3];                    /* Error bits status */
             uint32_t __RESERVED1;
        __IO uint32_t MMCS;                      /* MMC status */
        __IO uint32_t PMTS;                      /* PMT status */
        __IO uint32_t TSTS;                      /* Time-stamp trigger status */
             uint32_t __RESERVED2[2];
    } DMASR;
    struct {
             uint32_t __RESERVED0;
        __IO uint32_t SR;                        /* Start/stop receive */
        __IO uint32_t OSF;                       /* operate on second frame */
        __IO uint32_t RTC[2];                    /* receive threshold control */
             uint32_t __RESERVED1;
        __IO uint32_t FUGF;                      /* Forward undersized good frames */
        __IO uint32_t FEF;                       /* Forward error frames */
             uint32_t __RESERVED2[5];
        __IO uint32_t ST;                        /* Start/stop transmission command */
        __IO uint32_t TTC[3];                    /* Transmit threshold control */
             uint32_t __RESERVED3[3];
        __IO uint32_t FTF;                       /* Flush transmit FIFO */
        __IO uint32_t TSF;                       /* Transmit store and forward */
             uint32_t __RESERVED4[2];
        __IO uint32_t DFRF;                      /* Disable flushing of received frames */
        __IO uint32_t RSF;                       /* Receive store and forward */
        __IO uint32_t DTCEFD;                    /* Disable Dropping of TCP/IP checksum error frames */
             uint32_t __RESERVED5[5];
    } DMAOMR;
    struct {
        __IO uint32_t TIE;                       /* Transmit interrupt enable */
        __IO uint32_t TPSIE;                     /* Transmit process stopped interrupt enable */
        __IO uint32_t TBUIE;                     /* Transmit buffer unavailable interrupt enable */
        __IO uint32_t TJTIE;                     /* Transmit jabber timeout interrupt enable */
        __IO uint32_t ROIE;                      /* Receive Overflow interrupt enable */
        __IO uint32_t TUIE;                      /* Transmit Underflow interrupt enable */
        __IO uint32_t RIE;                       /* Receive interrupt enable */
        __IO uint32_t RBUIE;                     /* Receive buffer unavailable interrupt enable */
        __IO uint32_t RPSIE;                     /* Receive process stopped interrupt enable */
        __IO uint32_t RWTIE;                     /* Receive watchdog timeout interrupt enable */
        __IO uint32_t ETIE;                      /* Early transmit interrupt enable */
             uint32_t __RESERVED0[2];
        __IO uint32_t FBEIE;                     /* Fatal bus error interrupt enable */
        __IO uint32_t ERIE;                      /* Early receive interrupt enable */
        __IO uint32_t AISE;                      /* Abnormal interrupt summary enable */
        __IO uint32_t NISE;                      /* Normal interrupt summary enable */
             uint32_t __RESERVED1[15];
    } DMAIER;
    struct {
        __IO uint32_t MFC[16];                   /* Number of frames missed by the controller */
        __IO uint32_t OMFC;                      /* Overflow bit for missed frame counter */
        __IO uint32_t MFA[11];                   /* Number of frames missed by the application */
        __IO uint32_t OFOC;                      /* Overflow bit for FIFO overflow counter */
             uint32_t __RESERVED0[3];
    } DMAMFBOCR;
    __IO uint32_t DMARSWTR[32];
         uint32_t __RESERVED9[8][32];
    __IO uint32_t DMACHTDR[32];
    __IO uint32_t DMACHRDR[32];
    __IO uint32_t DMACHTBAR[32];
    __IO uint32_t DMACHRBAR[32];
} ETH_BitBand_TypeDef;



/** 
  * @brief External Interrupt/Event Controller
  */


typedef struct {
    __IO uint32_t IMR;                       /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
    __IO uint32_t EMR;                       /*!< EXTI Event mask register,                Address offset: 0x04 */
    __IO uint32_t RTSR;                      /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
    __IO uint32_t FTSR;                      /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
    __IO uint32_t SWIER;                     /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
    __IO uint32_t PR;                        /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;


typedef struct {
    __IO uint32_t IMR[32];                   /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
    __IO uint32_t EMR[32];                   /*!< EXTI Event mask register,                Address offset: 0x04 */
    __IO uint32_t RTSR[32];                  /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
    __IO uint32_t FTSR[32];                  /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
    __IO uint32_t SWIER[32];                 /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
    __IO uint32_t PR[32];                    /*!< EXTI Pending register,                   Address offset: 0x14 */
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
    } ACR;                                   /*!< FLASH access control register,   Address offset: 0x00 */
    __IO uint32_t KEYR;                      /*!< FLASH key register,              Address offset: 0x04 */
    __IO uint32_t OPTKEYR;                   /*!< FLASH option key register,       Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t EOP : 1;
            __IO uint32_t SOP : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t WRPERR : 1;
            __IO uint32_t PGAERR : 1;
            __IO uint32_t PGPERR : 1;
            __IO uint32_t PGSERR : 1;
                 uint32_t __RESERVED1 : 8;
            __IO uint32_t BSY : 1;
                 uint32_t __RESERVED2 : 15;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< FLASH status register,           Address offset: 0x0C */
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
    } CR;                                    /*!< FLASH control register,          Address offset: 0x10 */
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
    } OPTCR;                                 /*!< FLASH option control register ,  Address offset: 0x14 */
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t nWRP : 12;
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } OPTCR1;                                /*!< FLASH option control register 1, Address offset: 0x18 */
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
    } ACR;                                   /*!< FLASH access control register,   Address offset: 0x00 */
    __IO uint32_t KEYR[32];                  /*!< FLASH key register,              Address offset: 0x04 */
    __IO uint32_t OPTKEYR[32];               /*!< FLASH option key register,       Address offset: 0x08 */
    struct {
        __IO uint32_t EOP;
        __IO uint32_t SOP;
             uint32_t __RESERVED0[2];
        __IO uint32_t WRPERR;
        __IO uint32_t PGAERR;
        __IO uint32_t PGPERR;
        __IO uint32_t PGSERR;
             uint32_t __RESERVED1[8];
        __IO uint32_t BSY;
             uint32_t __RESERVED2[15];
    } SR;                                    /*!< FLASH status register,           Address offset: 0x0C */
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
    } CR;                                    /*!< FLASH control register,          Address offset: 0x10 */
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
    } OPTCR;                                 /*!< FLASH option control register ,  Address offset: 0x14 */
    struct {
             uint32_t __RESERVED0[16];
        __IO uint32_t nWRP[12];
             uint32_t __RESERVED1[4];
    } OPTCR1;                                /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_BitBand_TypeDef;




/** 
  * @brief Flexible Static Memory Controller
  */


typedef struct {
    struct {
        union {
            struct {
                __IO uint32_t MBKEN : 1;                 /*!<Memory bank enable bit                 */
                __IO uint32_t MUXEN : 1;                 /*!<Address/data multiplexing enable bit   */
                __IO uint32_t MTYP : 2;                  /*!<MTYP[1:0] bits (Memory type)           */
                __IO uint32_t MWID : 2;                  /*!<MWID[1:0] bits (Memory data bus width) */
                __IO uint32_t FACCEN : 1;                /*!<Flash access enable                    */
                     uint32_t __RESERVED0 : 1;
                __IO uint32_t BURSTEN : 1;               /*!<Burst enable bit                       */
                __IO uint32_t WAITPOL : 1;               /*!<Wait signal polarity bit               */
                __IO uint32_t WRAPMOD : 1;               /*!<Wrapped burst mode support             */
                __IO uint32_t WAITCFG : 1;               /*!<Wait timing configuration              */
                __IO uint32_t WREN : 1;                  /*!<Write enable bit                       */
                __IO uint32_t WAITEN : 1;                /*!<Wait enable bit                        */
                __IO uint32_t EXTMOD : 1;                /*!<Extended mode enable                   */
                __IO uint32_t ASYNCWAIT : 1;             /*!<Asynchronous wait                      */
                __IO uint32_t CPSIZE : 3;                /*!<CRAM page size */
                __IO uint32_t CBURSTRW : 1;              /*!<Write burst enable                     */
                     uint32_t __RESERVED1 : 12;
            } b;
            __IO uint32_t w;
        } BCR;                                   /*!< NOR/PSRAM chip-select control register(BCR) */
        union {
            struct {
                __IO uint32_t ADDSET : 4;                /*!<ADDSET[3:0] bits (Address setup phase duration) */
                __IO uint32_t ADDHLD : 4;                /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
                __IO uint32_t DATAST : 8;                /*!<DATAST [7:0] bits (Data-phase duration) */
                __IO uint32_t BUSTURN : 4;               /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
                __IO uint32_t CLKDIV : 4;                /*!<CLKDIV[3:0] bits (Clock divide ratio) */
                __IO uint32_t DATLAT : 4;                /*!<DATLA[3:0] bits (Data latency) */
                __IO uint32_t ACCMOD : 2;                /*!<ACCMOD[1:0] bits (Access mode) */
                     uint32_t __RESERVED0 : 2;
            } b;
            __IO uint32_t w;
        } BTR;                                   /*!< NOR/PSRAM chip-select timing register(BTR) */
    } BCTR[4];                                  /*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */
} FSMC_Bank1_TypeDef;


/** 
  * @brief Flexible Static Memory Controller Bank1E
  */
  

typedef struct {
    union {
        struct {
            __IO uint32_t ADDSET : 4;                /*!<ADDSET[3:0] bits (Address setup phase duration) */
            __IO uint32_t ADDHLD : 4;                /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
            __IO uint32_t DATAST : 8;                /*!<DATAST [7:0] bits (Data-phase duration) */
            __IO uint32_t BUSTURN : 4;               /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
            __IO uint32_t CLKDIV : 4;                /*!<CLKDIV[3:0] bits (Clock divide ratio) */
            __IO uint32_t DATLAT : 4;                /*!<DATLA[3:0] bits (Data latency) */
            __IO uint32_t ACCMOD : 2;                /*!<ACCMOD[1:0] bits (Access mode) */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } BWTR[7];                                  /*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FSMC_Bank1E_TypeDef;


/** 
  * @brief Flexible Static Memory Controller Bank2
  */
  

typedef struct {
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PWAITEN : 1;               /*!<Wait feature enable bit */
            __IO uint32_t PBKEN : 1;                 /*!<PC Card/NAND Flash memory bank enable bit */
            __IO uint32_t PTYP : 1;                  /*!<Memory type */
            __IO uint32_t PWID : 2;                  /*!<PWID[1:0] bits (NAND Flash databus width) */
            __IO uint32_t ECCEN : 1;                 /*!<ECC computation logic enable bit */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t TCLR : 4;                  /*!<TCLR[3:0] bits (CLE to RE delay) */
            __IO uint32_t TAR : 4;                   /*!<TAR[3:0] bits (ALE to RE delay) */
            __IO uint32_t ECCPS : 3;                 /*!<ECCPS[1:0] bits (ECC page size) */
                 uint32_t __RESERVED2 : 12;
        } b;
        __IO uint32_t w;
    } PCR2;                                  /*!< NAND Flash control register 2,                       Address offset: 0x60 */
    union {
        struct {
            __IO uint32_t IRS : 1;                   /*!<Interrupt Rising Edge status                */
            __IO uint32_t ILS : 1;                   /*!<Interrupt Level status                      */
            __IO uint32_t IFS : 1;                   /*!<Interrupt Falling Edge status               */
            __IO uint32_t IREN : 1;                  /*!<Interrupt Rising Edge detection Enable bit  */
            __IO uint32_t ILEN : 1;                  /*!<Interrupt Level detection Enable bit        */
            __IO uint32_t IFEN : 1;                  /*!<Interrupt Falling Edge detection Enable bit */
            __IO uint32_t FEMPT : 1;                 /*!<FIFO empty */
                 uint32_t __RESERVED0 : 25;
        } b;
        __IO uint32_t w;
    } SR2;                                   /*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
    union {
        struct {
            __IO uint32_t MEMSET2 : 8;               /*!<MEMSET2[7:0] bits (Common memory 2 setup time) */
            __IO uint32_t MEMWAIT2 : 8;              /*!<MEMWAIT2[7:0] bits (Common memory 2 wait time) */
            __IO uint32_t MEMHOLD2 : 8;              /*!<MEMHOLD2[7:0] bits (Common memory 2 hold time) */
            __IO uint32_t MEMHIZ2 : 8;               /*!<MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
        } b;
        __IO uint32_t w;
    } PMEM2;                                 /*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
    union {
        struct {
            __IO uint32_t ATTSET2 : 8;               /*!<ATTSET2[7:0] bits (Attribute memory 2 setup time) */
            __IO uint32_t ATTWAIT2 : 8;              /*!<ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
            __IO uint32_t ATTHOLD2 : 8;              /*!<ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
            __IO uint32_t ATTHIZ2 : 8;               /*!<ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
        } b;
        __IO uint32_t w;
    } PATT2;                                 /*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
         uint32_t __RESERVED0;               /*!< Reserved, 0x70                                                            */
    __IO uint32_t ECCR2;                     /*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
         uint32_t __RESERVED1;               /*!< Reserved, 0x78                                                            */
         uint32_t __RESERVED2;               /*!< Reserved, 0x7C                                                            */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PWAITEN : 1;               /*!<Wait feature enable bit */
            __IO uint32_t PBKEN : 1;                 /*!<PC Card/NAND Flash memory bank enable bit */
            __IO uint32_t PTYP : 1;                  /*!<Memory type */
            __IO uint32_t PWID : 2;                  /*!<PWID[1:0] bits (NAND Flash databus width) */
            __IO uint32_t ECCEN : 1;                 /*!<ECC computation logic enable bit */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t TCLR : 4;                  /*!<TCLR[3:0] bits (CLE to RE delay) */
            __IO uint32_t TAR : 4;                   /*!<TAR[3:0] bits (ALE to RE delay) */
            __IO uint32_t ECCPS : 3;                 /*!<ECCPS[2:0] bits (ECC page size) */
                 uint32_t __RESERVED2 : 12;
        } b;
        __IO uint32_t w;
    } PCR3;                                  /*!< NAND Flash control register 3,                       Address offset: 0x80 */
    union {
        struct {
            __IO uint32_t IRS : 1;                   /*!<Interrupt Rising Edge status                */
            __IO uint32_t ILS : 1;                   /*!<Interrupt Level status                      */
            __IO uint32_t IFS : 1;                   /*!<Interrupt Falling Edge status               */
            __IO uint32_t IREN : 1;                  /*!<Interrupt Rising Edge detection Enable bit  */
            __IO uint32_t ILEN : 1;                  /*!<Interrupt Level detection Enable bit        */
            __IO uint32_t IFEN : 1;                  /*!<Interrupt Falling Edge detection Enable bit */
            __IO uint32_t FEMPT : 1;                 /*!<FIFO empty */
                 uint32_t __RESERVED0 : 25;
        } b;
        __IO uint32_t w;
    } SR3;                                   /*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
    union {
        struct {
            __IO uint32_t MEMSET3 : 8;               /*!<MEMSET3[7:0] bits (Common memory 3 setup time) */
            __IO uint32_t MEMWAIT3 : 8;              /*!<MEMWAIT3[7:0] bits (Common memory 3 wait time) */
            __IO uint32_t MEMHOLD3 : 8;              /*!<MEMHOLD3[7:0] bits (Common memory 3 hold time) */
            __IO uint32_t MEMHIZ3 : 8;               /*!<MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
        } b;
        __IO uint32_t w;
    } PMEM3;                                 /*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
    union {
        struct {
            __IO uint32_t ATTSET3 : 8;               /*!<ATTSET3[7:0] bits (Attribute memory 3 setup time) */
            __IO uint32_t ATTWAIT3 : 8;              /*!<ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
            __IO uint32_t ATTHOLD3 : 8;              /*!<ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
            __IO uint32_t ATTHIZ3 : 8;               /*!<ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
        } b;
        __IO uint32_t w;
    } PATT3;                                 /*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
         uint32_t __RESERVED3;               /*!< Reserved, 0x90                                                            */
    __IO uint32_t ECCR3;                     /*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FSMC_Bank2_3_TypeDef;


/** 
  * @brief Flexible Static Memory Controller Bank4
  */
  

typedef struct {
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PWAITEN : 1;               /*!<Wait feature enable bit */
            __IO uint32_t PBKEN : 1;                 /*!<PC Card/NAND Flash memory bank enable bit */
            __IO uint32_t PTYP : 1;                  /*!<Memory type */
            __IO uint32_t PWID : 2;                  /*!<PWID[1:0] bits (NAND Flash databus width) */
            __IO uint32_t ECCEN : 1;                 /*!<ECC computation logic enable bit */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t TCLR : 4;                  /*!<TCLR[3:0] bits (CLE to RE delay) */
            __IO uint32_t TAR : 4;                   /*!<TAR[3:0] bits (ALE to RE delay) */
            __IO uint32_t ECCPS : 3;                 /*!<ECCPS[2:0] bits (ECC page size) */
                 uint32_t __RESERVED2 : 12;
        } b;
        __IO uint32_t w;
    } PCR4;                                  /*!< PC Card  control register 4,                       Address offset: 0xA0 */
    union {
        struct {
            __IO uint32_t IRS : 1;                   /*!<Interrupt Rising Edge status                 */
            __IO uint32_t ILS : 1;                   /*!<Interrupt Level status                       */
            __IO uint32_t IFS : 1;                   /*!<Interrupt Falling Edge status                */
            __IO uint32_t IREN : 1;                  /*!<Interrupt Rising Edge detection Enable bit   */
            __IO uint32_t ILEN : 1;                  /*!<Interrupt Level detection Enable bit         */
            __IO uint32_t IFEN : 1;                  /*!<Interrupt Falling Edge detection Enable bit  */
            __IO uint32_t FEMPT : 1;                 /*!<FIFO empty */
                 uint32_t __RESERVED0 : 25;
        } b;
        __IO uint32_t w;
    } SR4;                                   /*!< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4 */
    union {
        struct {
            __IO uint32_t MEMSET4 : 8;               /*!<MEMSET4[7:0] bits (Common memory 4 setup time) */
            __IO uint32_t MEMWAIT4 : 8;              /*!<MEMWAIT4[7:0] bits (Common memory 4 wait time) */
            __IO uint32_t MEMHOLD4 : 8;              /*!<MEMHOLD4[7:0] bits (Common memory 4 hold time) */
            __IO uint32_t MEMHIZ4 : 8;               /*!<MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
        } b;
        __IO uint32_t w;
    } PMEM4;                                 /*!< PC Card  Common memory space timing register 4,    Address offset: 0xA8 */
    union {
        struct {
            __IO uint32_t ATTSET4 : 8;               /*!<ATTSET4[7:0] bits (Attribute memory 4 setup time) */
            __IO uint32_t ATTWAIT4 : 8;              /*!<ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
            __IO uint32_t ATTHOLD4 : 8;              /*!<ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
            __IO uint32_t ATTHIZ4 : 8;               /*!<ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
        } b;
        __IO uint32_t w;
    } PATT4;                                 /*!< PC Card  Attribute memory space timing register 4, Address offset: 0xAC */
    union {
        struct {
            __IO uint32_t IOSET4 : 8;                /*!<IOSET4[7:0] bits (I/O 4 setup time) */
            __IO uint32_t IOWAIT4 : 8;               /*!<IOWAIT4[7:0] bits (I/O 4 wait time) */
            __IO uint32_t IOHOLD4 : 8;               /*!<IOHOLD4[7:0] bits (I/O 4 hold time) */
            __IO uint32_t IOHIZ4 : 8;                /*!<IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
        } b;
        __IO uint32_t w;
    } PIO4;                                  /*!< PC Card  I/O space timing register 4,              Address offset: 0xB0 */
} FSMC_Bank4_TypeDef;



/** 
  * @brief General Purpose I/O
  */


typedef struct {
    __IO uint32_t MODER;                     /*!< GPIO port mode register,               Address offset: 0x00      */
    __IO uint32_t OTYPER;                    /*!< GPIO port output type register,        Address offset: 0x04      */
    __IO uint32_t OSPEEDR;                   /*!< GPIO port output speed register,       Address offset: 0x08      */
    __IO uint32_t PUPDR;                     /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    __I  uint32_t IDR;                       /*!< GPIO port input data register,         Address offset: 0x10      */
    __IO uint32_t ODR;                       /*!< GPIO port output data register,        Address offset: 0x14      */
    __IO uint32_t BSRR;                      /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
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
    } LCKR;                                  /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    __IO uint32_t AFR[2];                    /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;


typedef struct {
    __IO uint32_t MODER[16][2];              /*!< GPIO port mode register,               Address offset: 0x00      */
    __IO uint32_t OTYPER[32];                /*!< GPIO port output type register,        Address offset: 0x04      */
    __IO uint32_t OSPEEDR[16][2];            /*!< GPIO port output speed register,       Address offset: 0x08      */
    __IO uint32_t PUPDR[16][2];              /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    __IO uint32_t IDR[32];                   /*!< GPIO port input data register,         Address offset: 0x10      */
    __IO uint32_t ODR[32];                   /*!< GPIO port output data register,        Address offset: 0x14      */
    __IO uint32_t BS[16];                    /*!< GPIO port bit set register,            Address offset: 0x18      */
    __IO uint32_t BR[16];                    /*!< GPIO port bit reset register,          Address offset: 0x1A      */
    struct {
        __IO uint32_t LCK[16];
        __IO uint32_t LCKK;
             uint32_t __RESERVED0[15];
    } LCKR;                                  /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    __IO uint32_t AFR[2][32];                   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_BitBand_TypeDef;



/** 
  * @brief System configuration controller
  */
  

typedef struct {
    union {
        struct {
            __IO uint32_t MEM_MODE : 3;              /*!< SYSCFG_Memory Remap Config */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } MEMRMP;                                /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
    union {
        struct {
                 uint32_t __RESERVED0 : 23;
            __IO uint32_t MII_RMII_SEL : 1;          /*!<Ethernet PHY interface selection */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } PMC;                                   /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
    __IO uint32_t EXTICR[4];                    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
         uint32_t __RESERVED0[2];               /*!< Reserved, 0x18-0x1C                                                          */
    union {
        struct {
            __IO uint32_t CMP_PD : 1;                /*!<Compensation cell ready flag */
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t READY : 1;                 /*!<Compensation cell power-down */
                 uint32_t __RESERVED1 : 23;
        } b;
        __IO uint32_t w;
    } CMPCR;                                 /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;


typedef struct {
    struct {
        __IO uint32_t MEM_MODE[3];               /*!< SYSCFG_Memory Remap Config */
             uint32_t __RESERVED0[29];
    } MEMRMP;                                /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
    struct {
             uint32_t __RESERVED0[23];
        __IO uint32_t MII_RMII_SEL;              /*!<Ethernet PHY interface selection */
             uint32_t __RESERVED1[8];
    } PMC;                                   /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
    __IO uint32_t EXTICR[4][32];                /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
         uint32_t __RESERVED0[2][32];           /*!< Reserved, 0x18-0x1C                                                          */
    struct {
        __IO uint32_t CMP_PD;                    /*!<Compensation cell ready flag */
             uint32_t __RESERVED0[7];
        __IO uint32_t READY;                     /*!<Compensation cell power-down */
             uint32_t __RESERVED1[23];
    } CMPCR;                                 /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_BitBand_TypeDef;



/** 
  * @brief Inter-integrated Circuit Interface
  */


typedef struct {
    union {
        struct {
            __IO uint32_t PE : 1;                    /*!<Peripheral Enable                             */
            __IO uint32_t SMBUS : 1;                 /*!<SMBus Mode                                    */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SMBTYPE : 1;               /*!<SMBus Type                                    */
            __IO uint32_t ENARP : 1;                 /*!<ARP Enable                                    */
            __IO uint32_t ENPEC : 1;                 /*!<PEC Enable                                    */
            __IO uint32_t ENGC : 1;                  /*!<General Call Enable                           */
            __IO uint32_t NOSTRETCH : 1;             /*!<Clock Stretching Disable (Slave mode)  */
            __IO uint32_t START : 1;                 /*!<Start Generation                              */
            __IO uint32_t STOP : 1;                  /*!<Stop Generation                               */
            __IO uint32_t ACK : 1;                   /*!<Acknowledge Enable                            */
            __IO uint32_t POS : 1;                   /*!<Acknowledge/PEC Position (for data reception) */
            __IO uint32_t PEC : 1;                   /*!<Packet Error Checking                         */
            __IO uint32_t ALERT : 1;                 /*!<SMBus Alert                                   */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SWRST : 1;                 /*!<Software Reset                                */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< I2C Control register 1,     Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t FREQ : 6;                  /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t ITERREN : 1;               /*!<Error Interrupt Enable  */
            __IO uint32_t ITEVTEN : 1;               /*!<Event Interrupt Enable  */
            __IO uint32_t ITBUFEN : 1;               /*!<Buffer Interrupt Enable */
            __IO uint32_t DMAEN : 1;                 /*!<DMA Requests Enable     */
            __IO uint32_t LAST : 1;                  /*!<DMA Last Transfer       */
                 uint32_t __RESERVED1 : 19;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< I2C Control register 2,     Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t ADD0 : 1;                  /*!<Bit 0 */
            __IO uint32_t ADD1 : 1;                  /*!<Bit 1 */
            __IO uint32_t ADD2 : 1;                  /*!<Bit 2 */
            __IO uint32_t ADD3 : 1;                  /*!<Bit 3 */
            __IO uint32_t ADD4 : 1;                  /*!<Bit 4 */
            __IO uint32_t ADD5 : 1;                  /*!<Bit 5 */
            __IO uint32_t ADD6 : 1;                  /*!<Bit 6 */
            __IO uint32_t ADD7 : 1;                  /*!<Bit 7 */
            __IO uint32_t ADD8 : 1;                  /*!<Bit 8 */
            __IO uint32_t ADD9 : 1;                  /*!<Bit 9 */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t ADDMODE : 1;               /*!<Addressing Mode (Slave mode) */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } OAR1;                                  /*!< I2C Own address register 1, Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t ENDUAL : 1;                /*!<Dual addressing mode enable */
            __IO uint32_t ADD2 : 7;                  /*!<Interface address           */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } OAR2;                                  /*!< I2C Own address register 2, Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t DR : 8;                    /*!<8-bit Data Register         */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } DR;                                    /*!< I2C Data register,          Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t SB : 1;                    /*!<Start Bit (Master mode)                  */
            __IO uint32_t ADDR : 1;                  /*!<Address sent (master mode)/matched (slave mode) */
            __IO uint32_t BTF : 1;                   /*!<Byte Transfer Finished                          */
            __IO uint32_t ADD10 : 1;                 /*!<10-bit header sent (Master mode)         */
            __IO uint32_t STOPF : 1;                 /*!<Stop detection (Slave mode)              */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RXNE : 1;                  /*!<Data Register not Empty (receivers)      */
            __IO uint32_t TXE : 1;                   /*!<Data Register Empty (transmitters)       */
            __IO uint32_t BERR : 1;                  /*!<Bus Error                                       */
            __IO uint32_t ARLO : 1;                  /*!<Arbitration Lost (master mode)           */
            __IO uint32_t AF : 1;                    /*!<Acknowledge Failure                             */
            __IO uint32_t OVR : 1;                   /*!<Overrun/Underrun                                */
            __IO uint32_t PECERR : 1;                /*!<PEC Error in reception                          */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t TIMEOUT : 1;               /*!<Timeout or Tlow Error                           */
            __IO uint32_t SMBALERT : 1;              /*!<SMBus Alert                                     */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } SR1;                                   /*!< I2C Status register 1,      Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t MSL : 1;                   /*!<Master/Slave                              */
            __IO uint32_t BUSY : 1;                  /*!<Bus Busy                                  */
            __IO uint32_t TRA : 1;                   /*!<Transmitter/Receiver                      */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t GENCALL : 1;               /*!<General Call Address (Slave mode)  */
            __IO uint32_t SMBDEFAULT : 1;            /*!<SMBus Device Default Address (Slave mode) */
            __IO uint32_t SMBHOST : 1;               /*!<SMBus Host Header (Slave mode)     */
            __IO uint32_t DUALF : 1;                 /*!<Dual Flag (Slave mode)             */
            __IO uint32_t PEC : 8;                   /*!<Packet Error Checking Register            */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } SR2;                                   /*!< I2C Status register 2,      Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t CCR : 12;                  /*!<Clock Control Register in Fast/Standard mode (Master mode) */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t DUTY : 1;                  /*!<Fast Mode Duty Cycle                                       */
            __IO uint32_t FS : 1;                    /*!<I2C Master Mode Selection                                  */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CCR;                                   /*!< I2C Clock control register, Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t TRISE : 6;                 /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */
                 uint32_t __RESERVED0 : 26;
        } b;
        __IO uint32_t w;
    } TRISE;                                 /*!< I2C TRISE register,         Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t DNF : 4;                   /*!<Digital Noise Filter */
            __IO uint32_t ANOFF : 1;                 /*!<Analog Noise Filter OFF */
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } FLTR;                                  /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PE;                        /*!<Peripheral Enable                             */
        __IO uint32_t SMBUS;                     /*!<SMBus Mode                                    */
             uint32_t __RESERVED0;
        __IO uint32_t SMBTYPE;                   /*!<SMBus Type                                    */
        __IO uint32_t ENARP;                     /*!<ARP Enable                                    */
        __IO uint32_t ENPEC;                     /*!<PEC Enable                                    */
        __IO uint32_t ENGC;                      /*!<General Call Enable                           */
        __IO uint32_t NOSTRETCH;                 /*!<Clock Stretching Disable (Slave mode)  */
        __IO uint32_t START;                     /*!<Start Generation                              */
        __IO uint32_t STOP;                      /*!<Stop Generation                               */
        __IO uint32_t ACK;                       /*!<Acknowledge Enable                            */
        __IO uint32_t POS;                       /*!<Acknowledge/PEC Position (for data reception) */
        __IO uint32_t PEC;                       /*!<Packet Error Checking                         */
        __IO uint32_t ALERT;                     /*!<SMBus Alert                                   */
             uint32_t __RESERVED1;
        __IO uint32_t SWRST;                     /*!<Software Reset                                */
             uint32_t __RESERVED2[16];
    } CR1;                                   /*!< I2C Control register 1,     Address offset: 0x00 */
    struct {
        __IO uint32_t FREQ[6];                   /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
             uint32_t __RESERVED0[2];
        __IO uint32_t ITERREN;                   /*!<Error Interrupt Enable  */
        __IO uint32_t ITEVTEN;                   /*!<Event Interrupt Enable  */
        __IO uint32_t ITBUFEN;                   /*!<Buffer Interrupt Enable */
        __IO uint32_t DMAEN;                     /*!<DMA Requests Enable     */
        __IO uint32_t LAST;                      /*!<DMA Last Transfer       */
             uint32_t __RESERVED1[19];
    } CR2;                                   /*!< I2C Control register 2,     Address offset: 0x04 */
    struct {
        __IO uint32_t ADD0;                      /*!<Bit 0 */
        __IO uint32_t ADD1;                      /*!<Bit 1 */
        __IO uint32_t ADD2;                      /*!<Bit 2 */
        __IO uint32_t ADD3;                      /*!<Bit 3 */
        __IO uint32_t ADD4;                      /*!<Bit 4 */
        __IO uint32_t ADD5;                      /*!<Bit 5 */
        __IO uint32_t ADD6;                      /*!<Bit 6 */
        __IO uint32_t ADD7;                      /*!<Bit 7 */
        __IO uint32_t ADD8;                      /*!<Bit 8 */
        __IO uint32_t ADD9;                      /*!<Bit 9 */
             uint32_t __RESERVED0[5];
        __IO uint32_t ADDMODE;                   /*!<Addressing Mode (Slave mode) */
             uint32_t __RESERVED1[16];
    } OAR1;                                  /*!< I2C Own address register 1, Address offset: 0x08 */
    struct {
        __IO uint32_t ENDUAL;                    /*!<Dual addressing mode enable */
        __IO uint32_t ADD2[7];                   /*!<Interface address           */
             uint32_t __RESERVED0[24];
    } OAR2;                                  /*!< I2C Own address register 2, Address offset: 0x0C */
    struct {
        __IO uint32_t DR[8];                     /*!<8-bit Data Register         */
             uint32_t __RESERVED0[24];
    } DR;                                    /*!< I2C Data register,          Address offset: 0x10 */
    struct {
        __IO uint32_t SB;                        /*!<Start Bit (Master mode)                  */
        __IO uint32_t ADDR;                      /*!<Address sent (master mode)/matched (slave mode) */
        __IO uint32_t BTF;                       /*!<Byte Transfer Finished                          */
        __IO uint32_t ADD10;                     /*!<10-bit header sent (Master mode)         */
        __IO uint32_t STOPF;                     /*!<Stop detection (Slave mode)              */
             uint32_t __RESERVED0;
        __IO uint32_t RXNE;                      /*!<Data Register not Empty (receivers)      */
        __IO uint32_t TXE;                       /*!<Data Register Empty (transmitters)       */
        __IO uint32_t BERR;                      /*!<Bus Error                                       */
        __IO uint32_t ARLO;                      /*!<Arbitration Lost (master mode)           */
        __IO uint32_t AF;                        /*!<Acknowledge Failure                             */
        __IO uint32_t OVR;                       /*!<Overrun/Underrun                                */
        __IO uint32_t PECERR;                    /*!<PEC Error in reception                          */
             uint32_t __RESERVED1;
        __IO uint32_t TIMEOUT;                   /*!<Timeout or Tlow Error                           */
        __IO uint32_t SMBALERT;                  /*!<SMBus Alert                                     */
             uint32_t __RESERVED2[16];
    } SR1;                                   /*!< I2C Status register 1,      Address offset: 0x14 */
    struct {
        __IO uint32_t MSL;                       /*!<Master/Slave                              */
        __IO uint32_t BUSY;                      /*!<Bus Busy                                  */
        __IO uint32_t TRA;                       /*!<Transmitter/Receiver                      */
             uint32_t __RESERVED0;
        __IO uint32_t GENCALL;                   /*!<General Call Address (Slave mode)  */
        __IO uint32_t SMBDEFAULT;                /*!<SMBus Device Default Address (Slave mode) */
        __IO uint32_t SMBHOST;                   /*!<SMBus Host Header (Slave mode)     */
        __IO uint32_t DUALF;                     /*!<Dual Flag (Slave mode)             */
        __IO uint32_t PEC[8];                    /*!<Packet Error Checking Register            */
             uint32_t __RESERVED1[16];
    } SR2;                                   /*!< I2C Status register 2,      Address offset: 0x18 */
    struct {
        __IO uint32_t CCR[12];                   /*!<Clock Control Register in Fast/Standard mode (Master mode) */
             uint32_t __RESERVED0[2];
        __IO uint32_t DUTY;                      /*!<Fast Mode Duty Cycle                                       */
        __IO uint32_t FS;                        /*!<I2C Master Mode Selection                                  */
             uint32_t __RESERVED1[16];
    } CCR;                                   /*!< I2C Clock control register, Address offset: 0x1C */
    struct {
        __IO uint32_t TRISE[6];                  /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */
             uint32_t __RESERVED0[26];
    } TRISE;                                 /*!< I2C TRISE register,         Address offset: 0x20 */
    struct {
        __IO uint32_t DNF[4];                    /*!<Digital Noise Filter */
        __IO uint32_t ANOFF;                     /*!<Analog Noise Filter OFF */
             uint32_t __RESERVED0[27];
    } FLTR;                                  /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_BitBand_TypeDef;



/** 
  * @brief Independent WATCHDOG
  */


typedef struct {
    union {
        struct {
            __IO uint32_t KEY : 16;                  /*!<Key value (write only, read 0000h)  */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } KR;                                    /*!< IWDG Key register,       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t PR : 3;                    /*!<PR[2:0] (Prescaler divider)         */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } PR;                                    /*!< IWDG Prescaler register, Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t RL : 12;                   /*!<Watchdog counter reload value        */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } RLR;                                   /*!< IWDG Reload register,    Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t PVU : 1;                   /*!<Watchdog prescaler value update      */
            __IO uint32_t RVU : 1;                   /*!<Watchdog counter reload value update */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;


typedef struct {
    struct {
        __IO uint32_t KEY[16];                   /*!<Key value (write only, read 0000h)  */
             uint32_t __RESERVED0[16];
    } KR;                                    /*!< IWDG Key register,       Address offset: 0x00 */
    struct {
        __IO uint32_t PR[3];                     /*!<PR[2:0] (Prescaler divider)         */
             uint32_t __RESERVED0[29];
    } PR;                                    /*!< IWDG Prescaler register, Address offset: 0x04 */
    struct {
        __IO uint32_t RL[12];                    /*!<Watchdog counter reload value        */
             uint32_t __RESERVED0[20];
    } RLR;                                   /*!< IWDG Reload register,    Address offset: 0x08 */
    struct {
        __IO uint32_t PVU;                       /*!<Watchdog prescaler value update      */
        __IO uint32_t RVU;                       /*!<Watchdog counter reload value update */
             uint32_t __RESERVED0[30];
    } SR;                                    /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_BitBand_TypeDef;



/** 
  * @brief Power Control
  */


typedef struct {
    union {
        struct {
            __IO uint32_t LPDS : 1;                  /*!< Low-Power Deepsleep                 */
            __IO uint32_t PDDS : 1;                  /*!< Power Down Deepsleep                */
            __IO uint32_t CWUF : 1;                  /*!< Clear Wakeup Flag                   */
            __IO uint32_t CSBF : 1;                  /*!< Clear Standby Flag                  */
            __IO uint32_t PVDE : 1;                  /*!< Power Voltage Detector Enable       */
            __IO uint32_t PLS : 3;                   /*!< PLS[2:0] bits (PVD Level Selection) */
            __IO uint32_t DBP : 1;                   /*!< Disable Backup Domain write protection               */
            __IO uint32_t FPDS : 1;                  /*!< Flash power down in Stop mode                        */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t VOS : 1;                   /*!< VOS bit (Regulator voltage scaling output selection) */
                 uint32_t __RESERVED1 : 17;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< PWR power control register,        Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t WUF : 1;                   /*!< Wakeup Flag                                      */
            __IO uint32_t SBF : 1;                   /*!< Standby Flag                                     */
            __IO uint32_t PVDO : 1;                  /*!< PVD Output                                       */
            __IO uint32_t BRR : 1;                   /*!< Backup regulator ready                           */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t EWUP : 1;                  /*!< Enable WKUP pin                                  */
            __IO uint32_t BRE : 1;                   /*!< Backup regulator enable                          */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t VOSRDY : 1;                /*!< Regulator voltage scaling output selection ready */
                 uint32_t __RESERVED2 : 17;
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;


typedef struct {
    struct {
        __IO uint32_t LPDS;                      /*!< Low-Power Deepsleep                 */
        __IO uint32_t PDDS;                      /*!< Power Down Deepsleep                */
        __IO uint32_t CWUF;                      /*!< Clear Wakeup Flag                   */
        __IO uint32_t CSBF;                      /*!< Clear Standby Flag                  */
        __IO uint32_t PVDE;                      /*!< Power Voltage Detector Enable       */
        __IO uint32_t PLS[3];                    /*!< PLS[2:0] bits (PVD Level Selection) */
        __IO uint32_t DBP;                       /*!< Disable Backup Domain write protection               */
        __IO uint32_t FPDS;                      /*!< Flash power down in Stop mode                        */
             uint32_t __RESERVED0[4];
        __IO uint32_t VOS;                       /*!< VOS bit (Regulator voltage scaling output selection) */
             uint32_t __RESERVED1[17];
    } CR;                                    /*!< PWR power control register,        Address offset: 0x00 */
    struct {
        __IO uint32_t WUF;                       /*!< Wakeup Flag                                      */
        __IO uint32_t SBF;                       /*!< Standby Flag                                     */
        __IO uint32_t PVDO;                      /*!< PVD Output                                       */
        __IO uint32_t BRR;                       /*!< Backup regulator ready                           */
             uint32_t __RESERVED0[4];
        __IO uint32_t EWUP;                      /*!< Enable WKUP pin                                  */
        __IO uint32_t BRE;                       /*!< Backup regulator enable                          */
             uint32_t __RESERVED1[4];
        __IO uint32_t VOSRDY;                    /*!< Regulator voltage scaling output selection ready */
             uint32_t __RESERVED2[17];
    } CSR;                                   /*!< PWR power control/status register, Address offset: 0x04 */
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
    } CR;                                    /*!< RCC clock control register,                                  Address offset: 0x00 */
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
    } PLLCFGR;                               /*!< RCC PLL configuration register,                              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t SW : 2;                    /*!< SW[1:0] bits (System clock Switch) */
            __IO uint32_t SWS : 2;                   /*!< SWS[1:0] bits (System Clock Switch Status) */
            __IO uint32_t HPRE : 4;                  /*!< HPRE[3:0] bits (AHB prescaler) */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t PPRE1 : 3;                 /*!< PRE1[2:0] bits (APB1 prescaler) */
            __IO uint32_t PPRE2 : 3;                 /*!< PRE2[2:0] bits (APB2 prescaler) */
            __IO uint32_t RTCPRE : 5;
            __IO uint32_t MCO1 : 2;
            __IO uint32_t I2SSRC : 1;
            __IO uint32_t MCO1PRE : 3;
            __IO uint32_t MCO2PRE : 3;
            __IO uint32_t MCO2 : 2;
        } b;
        __IO uint32_t w;
    } CFGR;                                  /*!< RCC clock configuration register,                            Address offset: 0x08 */
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
    } CIR;                                   /*!< RCC clock interrupt register,                                Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t GPIOARST : 1;
            __IO uint32_t GPIOBRST : 1;
            __IO uint32_t GPIOCRST : 1;
            __IO uint32_t GPIODRST : 1;
            __IO uint32_t GPIOERST : 1;
            __IO uint32_t GPIOFRST : 1;
            __IO uint32_t GPIOGRST : 1;
            __IO uint32_t GPIOHRST : 1;
            __IO uint32_t GPIOIRST : 1;
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t CRCRST : 1;
                 uint32_t __RESERVED1 : 8;
            __IO uint32_t DMA1RST : 1;
            __IO uint32_t DMA2RST : 1;
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t ETHMACRST : 1;
                 uint32_t __RESERVED3 : 3;
            __IO uint32_t OTGHRST : 1;
                 uint32_t __RESERVED4 : 2;
        } b;
        __IO uint32_t w;
    } AHB1RSTR;                              /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t DCMIRST : 1;
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RNGRST : 1;
            __IO uint32_t OTGFSRST : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } AHB2RSTR;                              /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t FSMCRST : 1;
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } AHB3RSTR;                              /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
         uint32_t __RESERVED0;               /*!< Reserved, 0x1C                                                                    */
    union {
        struct {
            __IO uint32_t TIM2RST : 1;
            __IO uint32_t TIM3RST : 1;
            __IO uint32_t TIM4RST : 1;
            __IO uint32_t TIM5RST : 1;
            __IO uint32_t TIM6RST : 1;
            __IO uint32_t TIM7RST : 1;
            __IO uint32_t TIM12RST : 1;
            __IO uint32_t TIM13RST : 1;
            __IO uint32_t TIM14RST : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t WWDGRST : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t SPI2RST : 1;
            __IO uint32_t SPI3RST : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t USART2RST : 1;
            __IO uint32_t USART3RST : 1;
            __IO uint32_t UART4RST : 1;
            __IO uint32_t UART5RST : 1;
            __IO uint32_t I2C1RST : 1;
            __IO uint32_t I2C2RST : 1;
            __IO uint32_t I2C3RST : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CAN1RST : 1;
            __IO uint32_t CAN2RST : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t PWRRST : 1;
            __IO uint32_t DACRST : 1;
                 uint32_t __RESERVED5 : 2;
        } b;
        __IO uint32_t w;
    } APB1RSTR;                              /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t TIM1RST : 1;
            __IO uint32_t TIM8RST : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t USART1RST : 1;
            __IO uint32_t USART6RST : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ADCRST : 1;
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SDIORST : 1;
            __IO uint32_t SPI1RST : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t SYSCFGRST : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t TIM9RST : 1;
            __IO uint32_t TIM10RST : 1;
            __IO uint32_t TIM11RST : 1;
                 uint32_t __RESERVED5 : 13;
        } b;
        __IO uint32_t w;
    } APB2RSTR;                              /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
         uint32_t __RESERVED1[2];               /*!< Reserved, 0x28-0x2C                                                               */
    union {
        struct {
            __IO uint32_t GPIOAEN : 1;
            __IO uint32_t GPIOBEN : 1;
            __IO uint32_t GPIOCEN : 1;
            __IO uint32_t GPIODEN : 1;
            __IO uint32_t GPIOEEN : 1;
            __IO uint32_t GPIOFEN : 1;
            __IO uint32_t GPIOGEN : 1;
            __IO uint32_t GPIOHEN : 1;
            __IO uint32_t GPIOIEN : 1;
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t CRCEN : 1;
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t BKPSRAMEN : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t CCMDATARAMEN : 1;
            __IO uint32_t DMA1EN : 1;
            __IO uint32_t DMA2EN : 1;
                 uint32_t __RESERVED3 : 2;
            __IO uint32_t ETHMACEN : 1;
            __IO uint32_t ETHMACTXEN : 1;
            __IO uint32_t ETHMACRXEN : 1;
            __IO uint32_t ETHMACPTPEN : 1;
            __IO uint32_t OTGHSEN : 1;
            __IO uint32_t OTGHSULPIEN : 1;
                 uint32_t __RESERVED4 : 1;
        } b;
        __IO uint32_t w;
    } AHB1ENR;                               /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    union {
        struct {
            __IO uint32_t DCMIEN : 1;
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RNGEN : 1;
            __IO uint32_t OTGFSEN : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } AHB2ENR;                               /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t FSMCEN : 1;
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } AHB3ENR;                               /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
         uint32_t __RESERVED2;               /*!< Reserved, 0x3C                                                                    */
    union {
        struct {
            __IO uint32_t TIM2EN : 1;
            __IO uint32_t TIM3EN : 1;
            __IO uint32_t TIM4EN : 1;
            __IO uint32_t TIM5EN : 1;
            __IO uint32_t TIM6EN : 1;
            __IO uint32_t TIM7EN : 1;
            __IO uint32_t TIM12EN : 1;
            __IO uint32_t TIM13EN : 1;
            __IO uint32_t TIM14EN : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t WWDGEN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t SPI2EN : 1;
            __IO uint32_t SPI3EN : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t USART2EN : 1;
            __IO uint32_t USART3EN : 1;
            __IO uint32_t UART4EN : 1;
            __IO uint32_t UART5EN : 1;
            __IO uint32_t I2C1EN : 1;
            __IO uint32_t I2C2EN : 1;
            __IO uint32_t I2C3EN : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CAN1EN : 1;
            __IO uint32_t CAN2EN : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t PWREN : 1;
            __IO uint32_t DACEN : 1;
                 uint32_t __RESERVED5 : 2;
        } b;
        __IO uint32_t w;
    } APB1ENR;                               /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    union {
        struct {
            __IO uint32_t TIM1EN : 1;
            __IO uint32_t TIM8EN : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t USART1EN : 1;
            __IO uint32_t USART6EN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ADC1EN : 1;
            __IO uint32_t ADC2EN : 1;
            __IO uint32_t ADC3EN : 1;
            __IO uint32_t SDIOEN : 1;
            __IO uint32_t SPI1EN : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SYSCFGEN : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t TIM9EN : 1;
            __IO uint32_t TIM10EN : 1;
            __IO uint32_t TIM11EN : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t SPI5EN : 1;
            __IO uint32_t SPI6EN : 1;
                 uint32_t __RESERVED5 : 10;
        } b;
        __IO uint32_t w;
    } APB2ENR;                               /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
         uint32_t __RESERVED3[2];               /*!< Reserved, 0x48-0x4C                                                               */
    union {
        struct {
            __IO uint32_t GPIOALPEN : 1;
            __IO uint32_t GPIOBLPEN : 1;
            __IO uint32_t GPIOCLPEN : 1;
            __IO uint32_t GPIODLPEN : 1;
            __IO uint32_t GPIOELPEN : 1;
            __IO uint32_t GPIOFLPEN : 1;
            __IO uint32_t GPIOGLPEN : 1;
            __IO uint32_t GPIOHLPEN : 1;
            __IO uint32_t GPIOILPEN : 1;
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t CRCLPEN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t FLITFLPEN : 1;
            __IO uint32_t SRAM1LPEN : 1;
            __IO uint32_t SRAM2LPEN : 1;
            __IO uint32_t BKPSRAMLPEN : 1;
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t DMA1LPEN : 1;
            __IO uint32_t DMA2LPEN : 1;
                 uint32_t __RESERVED3 : 2;
            __IO uint32_t ETHMACLPEN : 1;
            __IO uint32_t ETHMACTXLPEN : 1;
            __IO uint32_t ETHMACRXLPEN : 1;
            __IO uint32_t ETHMACPTPLPEN : 1;
            __IO uint32_t OTGHSLPEN : 1;
            __IO uint32_t OTGHSULPILPEN : 1;
                 uint32_t __RESERVED4 : 1;
        } b;
        __IO uint32_t w;
    } AHB1LPENR;                             /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    union {
        struct {
            __IO uint32_t DCMILPEN : 1;
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RNGLPEN : 1;
            __IO uint32_t OTGFSLPEN : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } AHB2LPENR;                             /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    union {
        struct {
            __IO uint32_t FSMCLPEN : 1;
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } AHB3LPENR;                             /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
         uint32_t __RESERVED4;               /*!< Reserved, 0x5C                                                                    */
    union {
        struct {
            __IO uint32_t TIM2LPEN : 1;
            __IO uint32_t TIM3LPEN : 1;
            __IO uint32_t TIM4LPEN : 1;
            __IO uint32_t TIM5LPEN : 1;
            __IO uint32_t TIM6LPEN : 1;
            __IO uint32_t TIM7LPEN : 1;
            __IO uint32_t TIM12LPEN : 1;
            __IO uint32_t TIM13LPEN : 1;
            __IO uint32_t TIM14LPEN : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t WWDGLPEN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t SPI2LPEN : 1;
            __IO uint32_t SPI3LPEN : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t USART2LPEN : 1;
            __IO uint32_t USART3LPEN : 1;
            __IO uint32_t UART4LPEN : 1;
            __IO uint32_t UART5LPEN : 1;
            __IO uint32_t I2C1LPEN : 1;
            __IO uint32_t I2C2LPEN : 1;
            __IO uint32_t I2C3LPEN : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CAN1LPEN : 1;
            __IO uint32_t CAN2LPEN : 1;
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t PWRLPEN : 1;
            __IO uint32_t DACLPEN : 1;
                 uint32_t __RESERVED5 : 2;
        } b;
        __IO uint32_t w;
    } APB1LPENR;                             /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    union {
        struct {
            __IO uint32_t TIM1LPEN : 1;
            __IO uint32_t TIM8LPEN : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t USART1LPEN : 1;
            __IO uint32_t USART6LPEN : 1;
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ADC1LPEN : 1;
            __IO uint32_t ADC2LPEN : 1;
            __IO uint32_t ADC3LPEN : 1;
            __IO uint32_t SDIOLPEN : 1;
            __IO uint32_t SPI1LPEN : 1;
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SYSCFGLPEN : 1;
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t TIM9LPEN : 1;
            __IO uint32_t TIM10LPEN : 1;
            __IO uint32_t TIM11LPEN : 1;
                 uint32_t __RESERVED4 : 13;
        } b;
        __IO uint32_t w;
    } APB2LPENR;                             /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
         uint32_t __RESERVED5[2];               /*!< Reserved, 0x68-0x6C                                                               */
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
    } BDCR;                                  /*!< RCC Backup domain control register,                          Address offset: 0x70 */
    union {
        struct {
            __IO uint32_t LSION : 1;
            __IO uint32_t LSIRDY : 1;
                 uint32_t __RESERVED0 : 22;
            __IO uint32_t RMVF : 1;
            __IO uint32_t BORRSTF : 1;
            __IO uint32_t PADRSTF : 1;
            __IO uint32_t PORRSTF : 1;
            __IO uint32_t SFTRSTF : 1;
            __IO uint32_t WDGRSTF : 1;
            __IO uint32_t WWDGRSTF : 1;
            __IO uint32_t LPWRRSTF : 1;
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< RCC clock control & status register,                         Address offset: 0x74 */
         uint32_t __RESERVED6[2];               /*!< Reserved, 0x78-0x7C                                                               */
    union {
        struct {
            __IO uint32_t MODPER : 13;
            __IO uint32_t INCSTEP : 15;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t SPREADSEL : 1;
            __IO uint32_t SSCGEN : 1;
        } b;
        __IO uint32_t w;
    } SSCGR;                                 /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
    union {
        struct {
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t PLLI2SN : 9;
                 uint32_t __RESERVED1 : 13;
            __IO uint32_t PLLI2SR : 3;
                 uint32_t __RESERVED2 : 1;
        } b;
        __IO uint32_t w;
    } PLLI2SCFGR;                            /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
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
    } CR;                                    /*!< RCC clock control register,                                  Address offset: 0x00 */
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
    } PLLCFGR;                               /*!< RCC PLL configuration register,                              Address offset: 0x04 */
    struct {
        __IO uint32_t SW[2];                     /*!< SW[1:0] bits (System clock Switch) */
        __IO uint32_t SWS[2];                    /*!< SWS[1:0] bits (System Clock Switch Status) */
        __IO uint32_t HPRE[4];                   /*!< HPRE[3:0] bits (AHB prescaler) */
             uint32_t __RESERVED0[2];
        __IO uint32_t PPRE1[3];                  /*!< PRE1[2:0] bits (APB1 prescaler) */
        __IO uint32_t PPRE2[3];                  /*!< PRE2[2:0] bits (APB2 prescaler) */
        __IO uint32_t RTCPRE[5];
        __IO uint32_t MCO1[2];
        __IO uint32_t I2SSRC;
        __IO uint32_t MCO1PRE[3];
        __IO uint32_t MCO2PRE[3];
        __IO uint32_t MCO2[2];
    } CFGR;                                  /*!< RCC clock configuration register,                            Address offset: 0x08 */
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
    } CIR;                                   /*!< RCC clock interrupt register,                                Address offset: 0x0C */
    struct {
        __IO uint32_t GPIOARST;
        __IO uint32_t GPIOBRST;
        __IO uint32_t GPIOCRST;
        __IO uint32_t GPIODRST;
        __IO uint32_t GPIOERST;
        __IO uint32_t GPIOFRST;
        __IO uint32_t GPIOGRST;
        __IO uint32_t GPIOHRST;
        __IO uint32_t GPIOIRST;
             uint32_t __RESERVED0[3];
        __IO uint32_t CRCRST;
             uint32_t __RESERVED1[8];
        __IO uint32_t DMA1RST;
        __IO uint32_t DMA2RST;
             uint32_t __RESERVED2[2];
        __IO uint32_t ETHMACRST;
             uint32_t __RESERVED3[3];
        __IO uint32_t OTGHRST;
             uint32_t __RESERVED4[2];
    } AHB1RSTR;                              /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
    struct {
        __IO uint32_t DCMIRST;
             uint32_t __RESERVED0[5];
        __IO uint32_t RNGRST;
        __IO uint32_t OTGFSRST;
             uint32_t __RESERVED1[24];
    } AHB2RSTR;                              /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
    struct {
        __IO uint32_t FSMCRST;
             uint32_t __RESERVED0[31];
    } AHB3RSTR;                              /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
         uint32_t __RESERVED0[32];           /*!< Reserved, 0x1C                                                                    */
    struct {
        __IO uint32_t TIM2RST;
        __IO uint32_t TIM3RST;
        __IO uint32_t TIM4RST;
        __IO uint32_t TIM5RST;
        __IO uint32_t TIM6RST;
        __IO uint32_t TIM7RST;
        __IO uint32_t TIM12RST;
        __IO uint32_t TIM13RST;
        __IO uint32_t TIM14RST;
             uint32_t __RESERVED0[2];
        __IO uint32_t WWDGRST;
             uint32_t __RESERVED1[2];
        __IO uint32_t SPI2RST;
        __IO uint32_t SPI3RST;
             uint32_t __RESERVED2;
        __IO uint32_t USART2RST;
        __IO uint32_t USART3RST;
        __IO uint32_t UART4RST;
        __IO uint32_t UART5RST;
        __IO uint32_t I2C1RST;
        __IO uint32_t I2C2RST;
        __IO uint32_t I2C3RST;
             uint32_t __RESERVED3;
        __IO uint32_t CAN1RST;
        __IO uint32_t CAN2RST;
             uint32_t __RESERVED4;
        __IO uint32_t PWRRST;
        __IO uint32_t DACRST;
             uint32_t __RESERVED5[2];
    } APB1RSTR;                              /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
    struct {
        __IO uint32_t TIM1RST;
        __IO uint32_t TIM8RST;
             uint32_t __RESERVED0[2];
        __IO uint32_t USART1RST;
        __IO uint32_t USART6RST;
             uint32_t __RESERVED1[2];
        __IO uint32_t ADCRST;
             uint32_t __RESERVED2[2];
        __IO uint32_t SDIORST;
        __IO uint32_t SPI1RST;
             uint32_t __RESERVED3;
        __IO uint32_t SYSCFGRST;
             uint32_t __RESERVED4;
        __IO uint32_t TIM9RST;
        __IO uint32_t TIM10RST;
        __IO uint32_t TIM11RST;
             uint32_t __RESERVED5[13];
    } APB2RSTR;                              /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
         uint32_t __RESERVED1[2][32];           /*!< Reserved, 0x28-0x2C                                                               */
    struct {
        __IO uint32_t GPIOAEN;
        __IO uint32_t GPIOBEN;
        __IO uint32_t GPIOCEN;
        __IO uint32_t GPIODEN;
        __IO uint32_t GPIOEEN;
        __IO uint32_t GPIOFEN;
        __IO uint32_t GPIOGEN;
        __IO uint32_t GPIOHEN;
        __IO uint32_t GPIOIEN;
             uint32_t __RESERVED0[3];
        __IO uint32_t CRCEN;
             uint32_t __RESERVED1[5];
        __IO uint32_t BKPSRAMEN;
             uint32_t __RESERVED2;
        __IO uint32_t CCMDATARAMEN;
        __IO uint32_t DMA1EN;
        __IO uint32_t DMA2EN;
             uint32_t __RESERVED3[2];
        __IO uint32_t ETHMACEN;
        __IO uint32_t ETHMACTXEN;
        __IO uint32_t ETHMACRXEN;
        __IO uint32_t ETHMACPTPEN;
        __IO uint32_t OTGHSEN;
        __IO uint32_t OTGHSULPIEN;
             uint32_t __RESERVED4;
    } AHB1ENR;                               /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
    struct {
        __IO uint32_t DCMIEN;
             uint32_t __RESERVED0[5];
        __IO uint32_t RNGEN;
        __IO uint32_t OTGFSEN;
             uint32_t __RESERVED1[24];
    } AHB2ENR;                               /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
    struct {
        __IO uint32_t FSMCEN;
             uint32_t __RESERVED0[31];
    } AHB3ENR;                               /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
         uint32_t __RESERVED2[32];           /*!< Reserved, 0x3C                                                                    */
    struct {
        __IO uint32_t TIM2EN;
        __IO uint32_t TIM3EN;
        __IO uint32_t TIM4EN;
        __IO uint32_t TIM5EN;
        __IO uint32_t TIM6EN;
        __IO uint32_t TIM7EN;
        __IO uint32_t TIM12EN;
        __IO uint32_t TIM13EN;
        __IO uint32_t TIM14EN;
             uint32_t __RESERVED0[2];
        __IO uint32_t WWDGEN;
             uint32_t __RESERVED1[2];
        __IO uint32_t SPI2EN;
        __IO uint32_t SPI3EN;
             uint32_t __RESERVED2;
        __IO uint32_t USART2EN;
        __IO uint32_t USART3EN;
        __IO uint32_t UART4EN;
        __IO uint32_t UART5EN;
        __IO uint32_t I2C1EN;
        __IO uint32_t I2C2EN;
        __IO uint32_t I2C3EN;
             uint32_t __RESERVED3;
        __IO uint32_t CAN1EN;
        __IO uint32_t CAN2EN;
             uint32_t __RESERVED4;
        __IO uint32_t PWREN;
        __IO uint32_t DACEN;
             uint32_t __RESERVED5[2];
    } APB1ENR;                               /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
    struct {
        __IO uint32_t TIM1EN;
        __IO uint32_t TIM8EN;
             uint32_t __RESERVED0[2];
        __IO uint32_t USART1EN;
        __IO uint32_t USART6EN;
             uint32_t __RESERVED1[2];
        __IO uint32_t ADC1EN;
        __IO uint32_t ADC2EN;
        __IO uint32_t ADC3EN;
        __IO uint32_t SDIOEN;
        __IO uint32_t SPI1EN;
             uint32_t __RESERVED2;
        __IO uint32_t SYSCFGEN;
             uint32_t __RESERVED3;
        __IO uint32_t TIM9EN;
        __IO uint32_t TIM10EN;
        __IO uint32_t TIM11EN;
             uint32_t __RESERVED4;
        __IO uint32_t SPI5EN;
        __IO uint32_t SPI6EN;
             uint32_t __RESERVED5[10];
    } APB2ENR;                               /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
         uint32_t __RESERVED3[2][32];           /*!< Reserved, 0x48-0x4C                                                               */
    struct {
        __IO uint32_t GPIOALPEN;
        __IO uint32_t GPIOBLPEN;
        __IO uint32_t GPIOCLPEN;
        __IO uint32_t GPIODLPEN;
        __IO uint32_t GPIOELPEN;
        __IO uint32_t GPIOFLPEN;
        __IO uint32_t GPIOGLPEN;
        __IO uint32_t GPIOHLPEN;
        __IO uint32_t GPIOILPEN;
             uint32_t __RESERVED0[3];
        __IO uint32_t CRCLPEN;
             uint32_t __RESERVED1[2];
        __IO uint32_t FLITFLPEN;
        __IO uint32_t SRAM1LPEN;
        __IO uint32_t SRAM2LPEN;
        __IO uint32_t BKPSRAMLPEN;
             uint32_t __RESERVED2[2];
        __IO uint32_t DMA1LPEN;
        __IO uint32_t DMA2LPEN;
             uint32_t __RESERVED3[2];
        __IO uint32_t ETHMACLPEN;
        __IO uint32_t ETHMACTXLPEN;
        __IO uint32_t ETHMACRXLPEN;
        __IO uint32_t ETHMACPTPLPEN;
        __IO uint32_t OTGHSLPEN;
        __IO uint32_t OTGHSULPILPEN;
             uint32_t __RESERVED4;
    } AHB1LPENR;                             /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    struct {
        __IO uint32_t DCMILPEN;
             uint32_t __RESERVED0[5];
        __IO uint32_t RNGLPEN;
        __IO uint32_t OTGFSLPEN;
             uint32_t __RESERVED1[24];
    } AHB2LPENR;                             /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    struct {
        __IO uint32_t FSMCLPEN;
             uint32_t __RESERVED0[31];
    } AHB3LPENR;                             /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
         uint32_t __RESERVED4[32];           /*!< Reserved, 0x5C                                                                    */
    struct {
        __IO uint32_t TIM2LPEN;
        __IO uint32_t TIM3LPEN;
        __IO uint32_t TIM4LPEN;
        __IO uint32_t TIM5LPEN;
        __IO uint32_t TIM6LPEN;
        __IO uint32_t TIM7LPEN;
        __IO uint32_t TIM12LPEN;
        __IO uint32_t TIM13LPEN;
        __IO uint32_t TIM14LPEN;
             uint32_t __RESERVED0[2];
        __IO uint32_t WWDGLPEN;
             uint32_t __RESERVED1[2];
        __IO uint32_t SPI2LPEN;
        __IO uint32_t SPI3LPEN;
             uint32_t __RESERVED2;
        __IO uint32_t USART2LPEN;
        __IO uint32_t USART3LPEN;
        __IO uint32_t UART4LPEN;
        __IO uint32_t UART5LPEN;
        __IO uint32_t I2C1LPEN;
        __IO uint32_t I2C2LPEN;
        __IO uint32_t I2C3LPEN;
             uint32_t __RESERVED3;
        __IO uint32_t CAN1LPEN;
        __IO uint32_t CAN2LPEN;
             uint32_t __RESERVED4;
        __IO uint32_t PWRLPEN;
        __IO uint32_t DACLPEN;
             uint32_t __RESERVED5[2];
    } APB1LPENR;                             /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    struct {
        __IO uint32_t TIM1LPEN;
        __IO uint32_t TIM8LPEN;
             uint32_t __RESERVED0[2];
        __IO uint32_t USART1LPEN;
        __IO uint32_t USART6LPEN;
             uint32_t __RESERVED1[2];
        __IO uint32_t ADC1LPEN;
        __IO uint32_t ADC2LPEN;
        __IO uint32_t ADC3LPEN;
        __IO uint32_t SDIOLPEN;
        __IO uint32_t SPI1LPEN;
             uint32_t __RESERVED2;
        __IO uint32_t SYSCFGLPEN;
             uint32_t __RESERVED3;
        __IO uint32_t TIM9LPEN;
        __IO uint32_t TIM10LPEN;
        __IO uint32_t TIM11LPEN;
             uint32_t __RESERVED4[13];
    } APB2LPENR;                             /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
         uint32_t __RESERVED5[2][32];           /*!< Reserved, 0x68-0x6C                                                               */
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
    } BDCR;                                  /*!< RCC Backup domain control register,                          Address offset: 0x70 */
    struct {
        __IO uint32_t LSION;
        __IO uint32_t LSIRDY;
             uint32_t __RESERVED0[22];
        __IO uint32_t RMVF;
        __IO uint32_t BORRSTF;
        __IO uint32_t PADRSTF;
        __IO uint32_t PORRSTF;
        __IO uint32_t SFTRSTF;
        __IO uint32_t WDGRSTF;
        __IO uint32_t WWDGRSTF;
        __IO uint32_t LPWRRSTF;
    } CSR;                                   /*!< RCC clock control & status register,                         Address offset: 0x74 */
         uint32_t __RESERVED6[2][32];           /*!< Reserved, 0x78-0x7C                                                               */
    struct {
        __IO uint32_t MODPER[13];
        __IO uint32_t INCSTEP[15];
             uint32_t __RESERVED0[2];
        __IO uint32_t SPREADSEL;
        __IO uint32_t SSCGEN;
    } SSCGR;                                 /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
    struct {
             uint32_t __RESERVED0[6];
        __IO uint32_t PLLI2SN[9];
             uint32_t __RESERVED1[13];
        __IO uint32_t PLLI2SR[3];
             uint32_t __RESERVED2;
    } PLLI2SCFGR;                            /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
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
    } TR;                                    /*!< RTC time register,                                        Address offset: 0x00 */
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
    } DR;                                    /*!< RTC date register,                                        Address offset: 0x04 */
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
            __IO uint32_t BCK : 1;
            __IO uint32_t COSEL : 1;
            __IO uint32_t POL : 1;
            __IO uint32_t OSEL : 2;
            __IO uint32_t COE : 1;
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< RTC control register,                                     Address offset: 0x08 */
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
    } ISR;                                   /*!< RTC initialization and status register,                   Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t PREDIV_S : 15;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PREDIV_A : 7;
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } PRER;                                  /*!< RTC prescaler register,                                   Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t WUT : 16;
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } WUTR;                                  /*!< RTC wakeup timer register,                                Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t DC : 5;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t DCS : 1;
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } CALIBR;                                /*!< RTC calibration register,                                 Address offset: 0x18 */
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
    } ALRMAR;                                /*!< RTC alarm A register,                                     Address offset: 0x1C */
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
    } ALRMBR;                                /*!< RTC alarm B register,                                     Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t KEY : 8;
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } WPR;                                   /*!< RTC write protection register,                            Address offset: 0x24 */
    union {
        struct {
            __IO uint32_t SS : 16;
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } SSR;                                   /*!< RTC sub second register,                                  Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t SUBFS : 15;
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t ADD1S : 1;
        } b;
        __IO uint32_t w;
    } SHIFTR;                                /*!< RTC shift control register,                               Address offset: 0x2C */
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
    } TSTR;                                  /*!< RTC time stamp time register,                             Address offset: 0x30 */
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
    } TSDR;                                  /*!< RTC time stamp date register,                             Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t SS : 16;
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } TSSSR;                                 /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
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
    } CALR;                                  /*!< RTC calibration register,                                 Address offset: 0x3C */
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
            __IO uint32_t TAMPINSEL : 1;
            __IO uint32_t TSINSEL : 1;
            __IO uint32_t ALARMOUTTYPE : 1;
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } TAFCR;                                 /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
    union {
        struct {
            __IO uint32_t SS : 15;
                 uint32_t __RESERVED0 : 9;
            __IO uint32_t MASKSS : 4;
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } ALRMASSR;                              /*!< RTC alarm A sub second register,                          Address offset: 0x44 */
    union {
        struct {
            __IO uint32_t SS : 15;
                 uint32_t __RESERVED0 : 9;
            __IO uint32_t MASKSS : 4;
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } ALRMBSSR;                              /*!< RTC alarm B sub second register,                          Address offset: 0x48 */
         uint32_t __RESERVED0;               /*!< Reserved, 0x4C                                                                 */
    __IO uint32_t BKPR[16];                      /*!< RTC backup registers,                                     Address offset: 0x50-0x8C */
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
    } TR;                                    /*!< RTC time register,                                        Address offset: 0x00 */
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
    } DR;                                    /*!< RTC date register,                                        Address offset: 0x04 */
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
        __IO uint32_t BCK;
        __IO uint32_t COSEL;
        __IO uint32_t POL;
        __IO uint32_t OSEL[2];
        __IO uint32_t COE;
             uint32_t __RESERVED0[8];
    } CR;                                    /*!< RTC control register,                                     Address offset: 0x08 */
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
    } ISR;                                   /*!< RTC initialization and status register,                   Address offset: 0x0C */
    struct {
        __IO uint32_t PREDIV_S[15];
             uint32_t __RESERVED0;
        __IO uint32_t PREDIV_A[7];
             uint32_t __RESERVED1[9];
    } PRER;                                  /*!< RTC prescaler register,                                   Address offset: 0x10 */
    struct {
        __IO uint32_t WUT[16];
             uint32_t __RESERVED0[16];
    } WUTR;                                  /*!< RTC wakeup timer register,                                Address offset: 0x14 */
    struct {
        __IO uint32_t DC[5];
             uint32_t __RESERVED0[2];
        __IO uint32_t DCS;
             uint32_t __RESERVED1[24];
    } CALIBR;                                /*!< RTC calibration register,                                 Address offset: 0x18 */
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
    } ALRMAR;                                /*!< RTC alarm A register,                                     Address offset: 0x1C */
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
    } ALRMBR;                                /*!< RTC alarm B register,                                     Address offset: 0x20 */
    struct {
        __IO uint32_t KEY[8];
             uint32_t __RESERVED0[24];
    } WPR;                                   /*!< RTC write protection register,                            Address offset: 0x24 */
    struct {
        __IO uint32_t SS[16];
             uint32_t __RESERVED0[16];
    } SSR;                                   /*!< RTC sub second register,                                  Address offset: 0x28 */
    struct {
        __IO uint32_t SUBFS[15];
             uint32_t __RESERVED0[16];
        __IO uint32_t ADD1S;
    } SHIFTR;                                /*!< RTC shift control register,                               Address offset: 0x2C */
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
    } TSTR;                                  /*!< RTC time stamp time register,                             Address offset: 0x30 */
    struct {
        __IO uint32_t DU[4];
        __IO uint32_t DT[2];
             uint32_t __RESERVED0[2];
        __IO uint32_t MU[4];
        __IO uint32_t MT;
        __IO uint32_t WDU[3];
             uint32_t __RESERVED1[16];
    } TSDR;                                  /*!< RTC time stamp date register,                             Address offset: 0x34 */
    struct {
        __IO uint32_t SS[16];
             uint32_t __RESERVED0[16];
    } TSSSR;                                 /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
    struct {
        __IO uint32_t CALM[9];
             uint32_t __RESERVED0[4];
        __IO uint32_t CALW16;
        __IO uint32_t CALW8;
        __IO uint32_t CALP;
             uint32_t __RESERVED1[16];
    } CALR;                                  /*!< RTC calibration register,                                 Address offset: 0x3C */
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
        __IO uint32_t TAMPINSEL;
        __IO uint32_t TSINSEL;
        __IO uint32_t ALARMOUTTYPE;
             uint32_t __RESERVED1[13];
    } TAFCR;                                 /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
    struct {
        __IO uint32_t SS[15];
             uint32_t __RESERVED0[9];
        __IO uint32_t MASKSS[4];
             uint32_t __RESERVED1[4];
    } ALRMASSR;                              /*!< RTC alarm A sub second register,                          Address offset: 0x44 */
    struct {
        __IO uint32_t SS[15];
             uint32_t __RESERVED0[9];
        __IO uint32_t MASKSS[4];
             uint32_t __RESERVED1[4];
    } ALRMBSSR;                              /*!< RTC alarm B sub second register,                          Address offset: 0x48 */
         uint32_t __RESERVED0[32];           /*!< Reserved, 0x4C                                                                 */
    __IO uint32_t BKPR[16][32];                  /*!< RTC backup registers,                                     Address offset: 0x50-0x8C */
} RTC_BitBand_TypeDef;




/** 
  * @brief SD host Interface
  */


typedef struct {
    union {
        struct {
            __IO uint32_t PWRCTRL : 2;               /*!<PWRCTRL[1:0] bits (Power supply control bits) */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } POWER;                                 /*!< SDIO power control register,    Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CLKDIV : 8;                /*!<Clock divide factor             */
            __IO uint32_t CLKEN : 1;                 /*!<Clock enable bit                */
            __IO uint32_t PWRSAV : 1;                /*!<Power saving configuration bit  */
            __IO uint32_t BYPASS : 1;                /*!<Clock divider bypass enable bit */
            __IO uint32_t WIDBUS : 2;                /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
            __IO uint32_t NEGEDGE : 1;               /*!<SDIO_CK dephasing selection bit */
            __IO uint32_t HWFC_EN : 1;               /*!<HW Flow Control enable          */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CLKCR;                                 /*!< SDI clock control register,     Address offset: 0x04 */
    __IO uint32_t ARG;                       /*!< SDIO argument register,         Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t CMDINDEX : 6;              /*!<Command Index                               */
            __IO uint32_t WAITRESP : 2;              /*!<WAITRESP[1:0] bits (Wait for response bits) */
            __IO uint32_t WAITINT : 1;               /*!<CPSM Waits for Interrupt Request                               */
            __IO uint32_t WAITPEND : 1;              /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
            __IO uint32_t CPSMEN : 1;                /*!<Command path state machine (CPSM) Enable bit                   */
            __IO uint32_t SDIOSUSPEND : 1;           /*!<SD I/O suspend command                                         */
            __IO uint32_t ENCMDCOMPL : 1;            /*!<Enable CMD completion                                          */
            __IO uint32_t NIEN : 1;                  /*!<Not Interrupt Enable */
            __IO uint32_t CEATACMD : 1;              /*!<CE-ATA command       */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CMD;                                   /*!< SDIO command register,          Address offset: 0x0C */
    union {
        struct {
            __I  uint32_t RESPCMD : 6;               /*!<Response command index */
                 uint32_t __RESERVED0 : 26;
        } b;
        __I  uint32_t w;
    } RESPCMD;                               /*!< SDIO command response register, Address offset: 0x10 */
    __I  uint32_t RESP1;                     /*!< SDIO response 1 register,       Address offset: 0x14 */
    __I  uint32_t RESP2;                     /*!< SDIO response 2 register,       Address offset: 0x18 */
    __I  uint32_t RESP3;                     /*!< SDIO response 3 register,       Address offset: 0x1C */
    __I  uint32_t RESP4;                     /*!< SDIO response 4 register,       Address offset: 0x20 */
    __IO uint32_t DTIMER;                    /*!< SDIO data timer register,       Address offset: 0x24 */
    union {
        struct {
            __IO uint32_t DATALENGTH : 25;           /*!<Data length value    */
                 uint32_t __RESERVED0 : 7;
        } b;
        __IO uint32_t w;
    } DLEN;                                  /*!< SDIO data length register,      Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t DTEN : 1;                  /*!<Data transfer enabled bit         */
            __IO uint32_t DTDIR : 1;                 /*!<Data transfer direction selection */
            __IO uint32_t DTMODE : 1;                /*!<Data transfer mode selection      */
            __IO uint32_t DMAEN : 1;                 /*!<DMA enabled bit                   */
            __IO uint32_t DBLOCKSIZE : 4;            /*!<DBLOCKSIZE[3:0] bits (Data block size) */
            __IO uint32_t RWSTART : 1;               /*!<Read wait start         */
            __IO uint32_t RWSTOP : 1;                /*!<Read wait stop          */
            __IO uint32_t RWMOD : 1;                 /*!<Read wait mode          */
            __IO uint32_t SDIOEN : 1;                /*!<SD I/O enable functions */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DCTRL;                                 /*!< SDIO data control register,     Address offset: 0x2C */
    union {
        struct {
            __I  uint32_t DATACOUNT : 25;            /*!<Data count value */
                 uint32_t __RESERVED0 : 7;
        } b;
        __I  uint32_t w;
    } DCOUNT;                                /*!< SDIO data counter register,     Address offset: 0x30 */
    union {
        struct {
            __I  uint32_t CCRCFAIL : 1;              /*!<Command response received (CRC check failed)  */
            __I  uint32_t DCRCFAIL : 1;              /*!<Data block sent/received (CRC check failed)   */
            __I  uint32_t CTIMEOUT : 1;              /*!<Command response timeout                      */
            __I  uint32_t DTIMEOUT : 1;              /*!<Data timeout                                  */
            __I  uint32_t TXUNDERR : 1;              /*!<Transmit FIFO underrun error                  */
            __I  uint32_t RXOVERR : 1;               /*!<Received FIFO overrun error                   */
            __I  uint32_t CMDREND : 1;               /*!<Command response received (CRC check passed)  */
            __I  uint32_t CMDSENT : 1;               /*!<Command sent (no response required)           */
            __I  uint32_t DATAEND : 1;               /*!<Data end (data counter, SDIDCOUNT, is zero)   */
            __I  uint32_t STBITERR : 1;              /*!<Start bit not detected on all data signals in wide bus mode */
            __I  uint32_t DBCKEND : 1;               /*!<Data block sent/received (CRC check passed)   */
            __I  uint32_t CMDACT : 1;                /*!<Command transfer in progress                  */
            __I  uint32_t TXACT : 1;                 /*!<Data transmit in progress                     */
            __I  uint32_t RXACT : 1;                 /*!<Data receive in progress                      */
            __I  uint32_t TXFIFOHE : 1;              /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
            __I  uint32_t RXFIFOHF : 1;              /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
            __I  uint32_t TXFIFOF : 1;               /*!<Transmit FIFO full                            */
            __I  uint32_t RXFIFOF : 1;               /*!<Receive FIFO full                             */
            __I  uint32_t TXFIFOE : 1;               /*!<Transmit FIFO empty                           */
            __I  uint32_t RXFIFOE : 1;               /*!<Receive FIFO empty                            */
            __I  uint32_t TXDAVL : 1;                /*!<Data available in transmit FIFO               */
            __I  uint32_t RXDAVL : 1;                /*!<Data available in receive FIFO                */
            __I  uint32_t SDIOIT : 1;                /*!<SDIO interrupt received                       */
            __I  uint32_t CEATAEND : 1;              /*!<CE-ATA command completion signal received for CMD61 */
                 uint32_t __RESERVED0 : 8;
        } b;
        __I  uint32_t w;
    } STA;                                   /*!< SDIO status register,           Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t CCRCFAILC : 1;             /*!<CCRCFAIL flag clear bit */
            __IO uint32_t DCRCFAILC : 1;             /*!<DCRCFAIL flag clear bit */
            __IO uint32_t CTIMEOUTC : 1;             /*!<CTIMEOUT flag clear bit */
            __IO uint32_t DTIMEOUTC : 1;             /*!<DTIMEOUT flag clear bit */
            __IO uint32_t TXUNDERRC : 1;             /*!<TXUNDERR flag clear bit */
            __IO uint32_t RXOVERRC : 1;              /*!<RXOVERR flag clear bit  */
            __IO uint32_t CMDRENDC : 1;              /*!<CMDREND flag clear bit  */
            __IO uint32_t CMDSENTC : 1;              /*!<CMDSENT flag clear bit  */
            __IO uint32_t DATAENDC : 1;              /*!<DATAEND flag clear bit  */
            __IO uint32_t STBITERRC : 1;             /*!<STBITERR flag clear bit */
            __IO uint32_t DBCKENDC : 1;              /*!<DBCKEND flag clear bit  */
                 uint32_t __RESERVED0 : 11;
            __IO uint32_t SDIOITC : 1;               /*!<SDIOIT flag clear bit   */
            __IO uint32_t CEATAENDC : 1;             /*!<CEATAEND flag clear bit */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } ICR;                                   /*!< SDIO interrupt clear register,  Address offset: 0x38 */
    union {
        struct {
            __IO uint32_t CCRCFAILIE : 1;            /*!<Command CRC Fail Interrupt Enable          */
            __IO uint32_t DCRCFAILIE : 1;            /*!<Data CRC Fail Interrupt Enable             */
            __IO uint32_t CTIMEOUTIE : 1;            /*!<Command TimeOut Interrupt Enable           */
            __IO uint32_t DTIMEOUTIE : 1;            /*!<Data TimeOut Interrupt Enable              */
            __IO uint32_t TXUNDERRIE : 1;            /*!<Tx FIFO UnderRun Error Interrupt Enable    */
            __IO uint32_t RXOVERRIE : 1;             /*!<Rx FIFO OverRun Error Interrupt Enable     */
            __IO uint32_t CMDRENDIE : 1;             /*!<Command Response Received Interrupt Enable */
            __IO uint32_t CMDSENTIE : 1;             /*!<Command Sent Interrupt Enable              */
            __IO uint32_t DATAENDIE : 1;             /*!<Data End Interrupt Enable                  */
            __IO uint32_t STBITERRIE : 1;            /*!<Start Bit Error Interrupt Enable           */
            __IO uint32_t DBCKENDIE : 1;             /*!<Data Block End Interrupt Enable            */
            __IO uint32_t CMDACTIE : 1;              /*!<CCommand Acting Interrupt Enable           */
            __IO uint32_t TXACTIE : 1;               /*!<Data Transmit Acting Interrupt Enable      */
            __IO uint32_t RXACTIE : 1;               /*!<Data receive acting interrupt enabled      */
            __IO uint32_t TXFIFOHEIE : 1;            /*!<Tx FIFO Half Empty interrupt Enable        */
            __IO uint32_t RXFIFOHFIE : 1;            /*!<Rx FIFO Half Full interrupt Enable         */
            __IO uint32_t TXFIFOFIE : 1;             /*!<Tx FIFO Full interrupt Enable              */
            __IO uint32_t RXFIFOFIE : 1;             /*!<Rx FIFO Full interrupt Enable              */
            __IO uint32_t TXFIFOEIE : 1;             /*!<Tx FIFO Empty interrupt Enable             */
            __IO uint32_t RXFIFOEIE : 1;             /*!<Rx FIFO Empty interrupt Enable             */
            __IO uint32_t TXDAVLIE : 1;              /*!<Data available in Tx FIFO interrupt Enable */
            __IO uint32_t RXDAVLIE : 1;              /*!<Data available in Rx FIFO interrupt Enable */
            __IO uint32_t SDIOITIE : 1;              /*!<SDIO Mode Interrupt Received interrupt Enable */
            __IO uint32_t CEATAENDIE : 1;            /*!<CE-ATA command completion signal received Interrupt Enable */
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } MASK;                                  /*!< SDIO mask register,             Address offset: 0x3C */
         uint32_t __RESERVED0[2];               /*!< Reserved, 0x40-0x44                                  */
    union {
        struct {
            __I  uint32_t FIFOCOUNT : 24;            /*!<Remaining number of words to be written to or read from the FIFO */
                 uint32_t __RESERVED0 : 8;
        } b;
        __I  uint32_t w;
    } FIFOCNT;                               /*!< SDIO FIFO counter register,     Address offset: 0x48 */
         uint32_t __RESERVED1[13];               /*!< Reserved, 0x4C-0x7C                                  */
    __IO uint32_t FIFO;                      /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PWRCTRL[2];                /*!<PWRCTRL[1:0] bits (Power supply control bits) */
             uint32_t __RESERVED0[30];
    } POWER;                                 /*!< SDIO power control register,    Address offset: 0x00 */
    struct {
        __IO uint32_t CLKDIV[8];                 /*!<Clock divide factor             */
        __IO uint32_t CLKEN;                     /*!<Clock enable bit                */
        __IO uint32_t PWRSAV;                    /*!<Power saving configuration bit  */
        __IO uint32_t BYPASS;                    /*!<Clock divider bypass enable bit */
        __IO uint32_t WIDBUS[2];                 /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
        __IO uint32_t NEGEDGE;                   /*!<SDIO_CK dephasing selection bit */
        __IO uint32_t HWFC_EN;                   /*!<HW Flow Control enable          */
             uint32_t __RESERVED0[17];
    } CLKCR;                                 /*!< SDI clock control register,     Address offset: 0x04 */
    __IO uint32_t ARG[32];                   /*!< SDIO argument register,         Address offset: 0x08 */
    struct {
        __IO uint32_t CMDINDEX[6];               /*!<Command Index                               */
        __IO uint32_t WAITRESP[2];               /*!<WAITRESP[1:0] bits (Wait for response bits) */
        __IO uint32_t WAITINT;                   /*!<CPSM Waits for Interrupt Request                               */
        __IO uint32_t WAITPEND;                  /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
        __IO uint32_t CPSMEN;                    /*!<Command path state machine (CPSM) Enable bit                   */
        __IO uint32_t SDIOSUSPEND;               /*!<SD I/O suspend command                                         */
        __IO uint32_t ENCMDCOMPL;                /*!<Enable CMD completion                                          */
        __IO uint32_t NIEN;                      /*!<Not Interrupt Enable */
        __IO uint32_t CEATACMD;                  /*!<CE-ATA command       */
             uint32_t __RESERVED0[17];
    } CMD;                                   /*!< SDIO command register,          Address offset: 0x0C */
    struct {
        __I  uint32_t RESPCMD[6];                /*!<Response command index */
             uint32_t __RESERVED0[26];
    } RESPCMD;                               /*!< SDIO command response register, Address offset: 0x10 */
    __I  uint32_t RESP1[32];                 /*!< SDIO response 1 register,       Address offset: 0x14 */
    __I  uint32_t RESP2[32];                 /*!< SDIO response 2 register,       Address offset: 0x18 */
    __I  uint32_t RESP3[32];                 /*!< SDIO response 3 register,       Address offset: 0x1C */
    __I  uint32_t RESP4[32];                 /*!< SDIO response 4 register,       Address offset: 0x20 */
    __IO uint32_t DTIMER[32];                /*!< SDIO data timer register,       Address offset: 0x24 */
    struct {
        __IO uint32_t DATALENGTH[25];            /*!<Data length value    */
             uint32_t __RESERVED0[7];
    } DLEN;                                  /*!< SDIO data length register,      Address offset: 0x28 */
    struct {
        __IO uint32_t DTEN;                      /*!<Data transfer enabled bit         */
        __IO uint32_t DTDIR;                     /*!<Data transfer direction selection */
        __IO uint32_t DTMODE;                    /*!<Data transfer mode selection      */
        __IO uint32_t DMAEN;                     /*!<DMA enabled bit                   */
        __IO uint32_t DBLOCKSIZE[4];             /*!<DBLOCKSIZE[3:0] bits (Data block size) */
        __IO uint32_t RWSTART;                   /*!<Read wait start         */
        __IO uint32_t RWSTOP;                    /*!<Read wait stop          */
        __IO uint32_t RWMOD;                     /*!<Read wait mode          */
        __IO uint32_t SDIOEN;                    /*!<SD I/O enable functions */
             uint32_t __RESERVED0[20];
    } DCTRL;                                 /*!< SDIO data control register,     Address offset: 0x2C */
    struct {
        __I  uint32_t DATACOUNT[25];             /*!<Data count value */
             uint32_t __RESERVED0[7];
    } DCOUNT;                                /*!< SDIO data counter register,     Address offset: 0x30 */
    struct {
        __I  uint32_t CCRCFAIL;                  /*!<Command response received (CRC check failed)  */
        __I  uint32_t DCRCFAIL;                  /*!<Data block sent/received (CRC check failed)   */
        __I  uint32_t CTIMEOUT;                  /*!<Command response timeout                      */
        __I  uint32_t DTIMEOUT;                  /*!<Data timeout                                  */
        __I  uint32_t TXUNDERR;                  /*!<Transmit FIFO underrun error                  */
        __I  uint32_t RXOVERR;                   /*!<Received FIFO overrun error                   */
        __I  uint32_t CMDREND;                   /*!<Command response received (CRC check passed)  */
        __I  uint32_t CMDSENT;                   /*!<Command sent (no response required)           */
        __I  uint32_t DATAEND;                   /*!<Data end (data counter, SDIDCOUNT, is zero)   */
        __I  uint32_t STBITERR;                  /*!<Start bit not detected on all data signals in wide bus mode */
        __I  uint32_t DBCKEND;                   /*!<Data block sent/received (CRC check passed)   */
        __I  uint32_t CMDACT;                    /*!<Command transfer in progress                  */
        __I  uint32_t TXACT;                     /*!<Data transmit in progress                     */
        __I  uint32_t RXACT;                     /*!<Data receive in progress                      */
        __I  uint32_t TXFIFOHE;                  /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
        __I  uint32_t RXFIFOHF;                  /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
        __I  uint32_t TXFIFOF;                   /*!<Transmit FIFO full                            */
        __I  uint32_t RXFIFOF;                   /*!<Receive FIFO full                             */
        __I  uint32_t TXFIFOE;                   /*!<Transmit FIFO empty                           */
        __I  uint32_t RXFIFOE;                   /*!<Receive FIFO empty                            */
        __I  uint32_t TXDAVL;                    /*!<Data available in transmit FIFO               */
        __I  uint32_t RXDAVL;                    /*!<Data available in receive FIFO                */
        __I  uint32_t SDIOIT;                    /*!<SDIO interrupt received                       */
        __I  uint32_t CEATAEND;                  /*!<CE-ATA command completion signal received for CMD61 */
             uint32_t __RESERVED0[8];
    } STA;                                   /*!< SDIO status register,           Address offset: 0x34 */
    struct {
        __IO uint32_t CCRCFAILC;                 /*!<CCRCFAIL flag clear bit */
        __IO uint32_t DCRCFAILC;                 /*!<DCRCFAIL flag clear bit */
        __IO uint32_t CTIMEOUTC;                 /*!<CTIMEOUT flag clear bit */
        __IO uint32_t DTIMEOUTC;                 /*!<DTIMEOUT flag clear bit */
        __IO uint32_t TXUNDERRC;                 /*!<TXUNDERR flag clear bit */
        __IO uint32_t RXOVERRC;                  /*!<RXOVERR flag clear bit  */
        __IO uint32_t CMDRENDC;                  /*!<CMDREND flag clear bit  */
        __IO uint32_t CMDSENTC;                  /*!<CMDSENT flag clear bit  */
        __IO uint32_t DATAENDC;                  /*!<DATAEND flag clear bit  */
        __IO uint32_t STBITERRC;                 /*!<STBITERR flag clear bit */
        __IO uint32_t DBCKENDC;                  /*!<DBCKEND flag clear bit  */
             uint32_t __RESERVED0[11];
        __IO uint32_t SDIOITC;                   /*!<SDIOIT flag clear bit   */
        __IO uint32_t CEATAENDC;                 /*!<CEATAEND flag clear bit */
             uint32_t __RESERVED1[8];
    } ICR;                                   /*!< SDIO interrupt clear register,  Address offset: 0x38 */
    struct {
        __IO uint32_t CCRCFAILIE;                /*!<Command CRC Fail Interrupt Enable          */
        __IO uint32_t DCRCFAILIE;                /*!<Data CRC Fail Interrupt Enable             */
        __IO uint32_t CTIMEOUTIE;                /*!<Command TimeOut Interrupt Enable           */
        __IO uint32_t DTIMEOUTIE;                /*!<Data TimeOut Interrupt Enable              */
        __IO uint32_t TXUNDERRIE;                /*!<Tx FIFO UnderRun Error Interrupt Enable    */
        __IO uint32_t RXOVERRIE;                 /*!<Rx FIFO OverRun Error Interrupt Enable     */
        __IO uint32_t CMDRENDIE;                 /*!<Command Response Received Interrupt Enable */
        __IO uint32_t CMDSENTIE;                 /*!<Command Sent Interrupt Enable              */
        __IO uint32_t DATAENDIE;                 /*!<Data End Interrupt Enable                  */
        __IO uint32_t STBITERRIE;                /*!<Start Bit Error Interrupt Enable           */
        __IO uint32_t DBCKENDIE;                 /*!<Data Block End Interrupt Enable            */
        __IO uint32_t CMDACTIE;                  /*!<CCommand Acting Interrupt Enable           */
        __IO uint32_t TXACTIE;                   /*!<Data Transmit Acting Interrupt Enable      */
        __IO uint32_t RXACTIE;                   /*!<Data receive acting interrupt enabled      */
        __IO uint32_t TXFIFOHEIE;                /*!<Tx FIFO Half Empty interrupt Enable        */
        __IO uint32_t RXFIFOHFIE;                /*!<Rx FIFO Half Full interrupt Enable         */
        __IO uint32_t TXFIFOFIE;                 /*!<Tx FIFO Full interrupt Enable              */
        __IO uint32_t RXFIFOFIE;                 /*!<Rx FIFO Full interrupt Enable              */
        __IO uint32_t TXFIFOEIE;                 /*!<Tx FIFO Empty interrupt Enable             */
        __IO uint32_t RXFIFOEIE;                 /*!<Rx FIFO Empty interrupt Enable             */
        __IO uint32_t TXDAVLIE;                  /*!<Data available in Tx FIFO interrupt Enable */
        __IO uint32_t RXDAVLIE;                  /*!<Data available in Rx FIFO interrupt Enable */
        __IO uint32_t SDIOITIE;                  /*!<SDIO Mode Interrupt Received interrupt Enable */
        __IO uint32_t CEATAENDIE;                /*!<CE-ATA command completion signal received Interrupt Enable */
             uint32_t __RESERVED0[8];
    } MASK;                                  /*!< SDIO mask register,             Address offset: 0x3C */
         uint32_t __RESERVED0[2][32];           /*!< Reserved, 0x40-0x44                                  */
    struct {
        __I  uint32_t FIFOCOUNT[24];             /*!<Remaining number of words to be written to or read from the FIFO */
             uint32_t __RESERVED0[8];
    } FIFOCNT;                               /*!< SDIO FIFO counter register,     Address offset: 0x48 */
         uint32_t __RESERVED1[13][32];           /*!< Reserved, 0x4C-0x7C                                  */
    __IO uint32_t FIFO[32];                  /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_BitBand_TypeDef;



/** 
  * @brief Serial Peripheral Interface
  */


typedef struct {
    union {
        struct {
            __IO uint32_t CPHA : 1;                  /*!<Clock Phase      */
            __IO uint32_t CPOL : 1;                  /*!<Clock Polarity   */
            __IO uint32_t MSTR : 1;                  /*!<Master Selection */
            __IO uint32_t BR : 3;                    /*!<BR[2:0] bits (Baud Rate Control) */
            __IO uint32_t SPE : 1;                   /*!<SPI Enable                          */
            __IO uint32_t LSBFIRST : 1;              /*!<Frame Format                        */
            __IO uint32_t SSI : 1;                   /*!<Internal slave select               */
            __IO uint32_t SSM : 1;                   /*!<Software slave management           */
            __IO uint32_t RXONLY : 1;                /*!<Receive only                        */
            __IO uint32_t DFF : 1;                   /*!<Data Frame Format                   */
            __IO uint32_t CRCNEXT : 1;               /*!<Transmit CRC next                   */
            __IO uint32_t CRCEN : 1;                 /*!<Hardware CRC calculation enable     */
            __IO uint32_t BIDIOE : 1;                /*!<Output enable in bidirectional mode */
            __IO uint32_t BIDIMODE : 1;              /*!<Bidirectional data mode enable      */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t RXDMAEN : 1;               /*!<Rx Buffer DMA Enable                 */
            __IO uint32_t TXDMAEN : 1;               /*!<Tx Buffer DMA Enable                 */
            __IO uint32_t SSOE : 1;                  /*!<SS Output Enable                     */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t FRF : 1;                   /*!<Frame Format                         */
            __IO uint32_t ERRIE : 1;                 /*!<Error Interrupt Enable               */
            __IO uint32_t RXNEIE : 1;                /*!<RX buffer Not Empty Interrupt Enable */
            __IO uint32_t TXEIE : 1;                 /*!<Tx buffer Empty Interrupt Enable     */
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< SPI control register 2,                             Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t RXNE : 1;                  /*!<Receive buffer Not Empty */
            __IO uint32_t TXE : 1;                   /*!<Transmit buffer Empty    */
            __IO uint32_t CHSIDE : 1;                /*!<Channel side             */
            __IO uint32_t UDR : 1;                   /*!<Underrun flag            */
            __IO uint32_t CRCERR : 1;                /*!<CRC Error flag           */
            __IO uint32_t MODF : 1;                  /*!<Mode fault               */
            __IO uint32_t OVR : 1;                   /*!<Overrun flag             */
            __IO uint32_t BSY : 1;                   /*!<Busy flag                */
            __IO uint32_t FRE : 1;                   /*!<Frame format error flag  */
                 uint32_t __RESERVED0 : 23;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< SPI status register,                                Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t DR : 16;                   /*!<Data Register           */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } DR;                                    /*!< SPI data register,                                  Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t CRCPOLY : 16;              /*!<CRC polynomial register */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } CRCPR;                                 /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t RXCRC : 16;                /*!<Rx CRC Register         */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } RXCRCR;                                /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t TXCRC : 16;                /*!<Tx CRC Register         */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } TXCRCR;                                /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t CHLEN : 1;                 /*!<Channel length (number of bits per audio channel) */
            __IO uint32_t DATLEN : 2;                /*!<DATLEN[1:0] bits (Data length to be transferred)  */
            __IO uint32_t CKPOL : 1;                 /*!<steady state clock polarity               */
            __IO uint32_t I2SSTD : 2;                /*!<I2SSTD[1:0] bits (I2S standard selection) */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PCMSYNC : 1;               /*!<PCM frame synchronization                 */
            __IO uint32_t I2SCFG : 2;                /*!<I2SCFG[1:0] bits (I2S configuration mode) */
            __IO uint32_t I2SE : 1;                  /*!<I2S Enable         */
            __IO uint32_t I2SMOD : 1;                /*!<I2S mode selection */
                 uint32_t __RESERVED1 : 20;
        } b;
        __IO uint32_t w;
    } I2SCFGR;                               /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t I2SDIV : 8;                /*!<I2S Linear prescaler         */
            __IO uint32_t ODD : 1;                   /*!<Odd factor for the prescaler */
            __IO uint32_t MCKOE : 1;                 /*!<Master Clock Output Enable   */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } I2SPR;                                 /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CPHA;                      /*!<Clock Phase      */
        __IO uint32_t CPOL;                      /*!<Clock Polarity   */
        __IO uint32_t MSTR;                      /*!<Master Selection */
        __IO uint32_t BR[3];                     /*!<BR[2:0] bits (Baud Rate Control) */
        __IO uint32_t SPE;                       /*!<SPI Enable                          */
        __IO uint32_t LSBFIRST;                  /*!<Frame Format                        */
        __IO uint32_t SSI;                       /*!<Internal slave select               */
        __IO uint32_t SSM;                       /*!<Software slave management           */
        __IO uint32_t RXONLY;                    /*!<Receive only                        */
        __IO uint32_t DFF;                       /*!<Data Frame Format                   */
        __IO uint32_t CRCNEXT;                   /*!<Transmit CRC next                   */
        __IO uint32_t CRCEN;                     /*!<Hardware CRC calculation enable     */
        __IO uint32_t BIDIOE;                    /*!<Output enable in bidirectional mode */
        __IO uint32_t BIDIMODE;                  /*!<Bidirectional data mode enable      */
             uint32_t __RESERVED0[16];
    } CR1;                                   /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
    struct {
        __IO uint32_t RXDMAEN;                   /*!<Rx Buffer DMA Enable                 */
        __IO uint32_t TXDMAEN;                   /*!<Tx Buffer DMA Enable                 */
        __IO uint32_t SSOE;                      /*!<SS Output Enable                     */
             uint32_t __RESERVED0;
        __IO uint32_t FRF;                       /*!<Frame Format                         */
        __IO uint32_t ERRIE;                     /*!<Error Interrupt Enable               */
        __IO uint32_t RXNEIE;                    /*!<RX buffer Not Empty Interrupt Enable */
        __IO uint32_t TXEIE;                     /*!<Tx buffer Empty Interrupt Enable     */
             uint32_t __RESERVED1[24];
    } CR2;                                   /*!< SPI control register 2,                             Address offset: 0x04 */
    struct {
        __IO uint32_t RXNE;                      /*!<Receive buffer Not Empty */
        __IO uint32_t TXE;                       /*!<Transmit buffer Empty    */
        __IO uint32_t CHSIDE;                    /*!<Channel side             */
        __IO uint32_t UDR;                       /*!<Underrun flag            */
        __IO uint32_t CRCERR;                    /*!<CRC Error flag           */
        __IO uint32_t MODF;                      /*!<Mode fault               */
        __IO uint32_t OVR;                       /*!<Overrun flag             */
        __IO uint32_t BSY;                       /*!<Busy flag                */
        __IO uint32_t FRE;                       /*!<Frame format error flag  */
             uint32_t __RESERVED0[23];
    } SR;                                    /*!< SPI status register,                                Address offset: 0x08 */
    struct {
        __IO uint32_t DR[16];                    /*!<Data Register           */
             uint32_t __RESERVED0[16];
    } DR;                                    /*!< SPI data register,                                  Address offset: 0x0C */
    struct {
        __IO uint32_t CRCPOLY[16];               /*!<CRC polynomial register */
             uint32_t __RESERVED0[16];
    } CRCPR;                                 /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
    struct {
        __IO uint32_t RXCRC[16];                 /*!<Rx CRC Register         */
             uint32_t __RESERVED0[16];
    } RXCRCR;                                /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
    struct {
        __IO uint32_t TXCRC[16];                 /*!<Tx CRC Register         */
             uint32_t __RESERVED0[16];
    } TXCRCR;                                /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
    struct {
        __IO uint32_t CHLEN;                     /*!<Channel length (number of bits per audio channel) */
        __IO uint32_t DATLEN[2];                 /*!<DATLEN[1:0] bits (Data length to be transferred)  */
        __IO uint32_t CKPOL;                     /*!<steady state clock polarity               */
        __IO uint32_t I2SSTD[2];                 /*!<I2SSTD[1:0] bits (I2S standard selection) */
             uint32_t __RESERVED0;
        __IO uint32_t PCMSYNC;                   /*!<PCM frame synchronization                 */
        __IO uint32_t I2SCFG[2];                 /*!<I2SCFG[1:0] bits (I2S configuration mode) */
        __IO uint32_t I2SE;                      /*!<I2S Enable         */
        __IO uint32_t I2SMOD;                    /*!<I2S mode selection */
             uint32_t __RESERVED1[20];
    } I2SCFGR;                               /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
    struct {
        __IO uint32_t I2SDIV[8];                 /*!<I2S Linear prescaler         */
        __IO uint32_t ODD;                       /*!<Odd factor for the prescaler */
        __IO uint32_t MCKOE;                     /*!<Master Clock Output Enable   */
             uint32_t __RESERVED0[22];
    } I2SPR;                                 /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_BitBand_TypeDef;



/** 
  * @brief TIM
  */


typedef struct {
    union {
        struct {
            __IO uint32_t CEN : 1;                   /*!<Counter enable        */
            __IO uint32_t UDIS : 1;                  /*!<Update disable        */
            __IO uint32_t URS : 1;                   /*!<Update request source */
            __IO uint32_t OPM : 1;                   /*!<One pulse mode        */
            __IO uint32_t DIR : 1;                   /*!<Direction             */
            __IO uint32_t CMS : 2;                   /*!<CMS[1:0] bits (Center-aligned mode selection) */
            __IO uint32_t ARPE : 1;                  /*!<Auto-reload preload enable     */
            __IO uint32_t CKD : 2;                   /*!<CKD[1:0] bits (clock division) */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< TIM control register 1,              Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CCPC : 1;                  /*!<Capture/Compare Preloaded Control        */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CCUS : 1;                  /*!<Capture/Compare Control Update Selection */
            __IO uint32_t CCDS : 1;                  /*!<Capture/Compare DMA Selection            */
            __IO uint32_t MMS : 3;                   /*!<MMS[2:0] bits (Master Mode Selection) */
            __IO uint32_t TI1S : 1;                  /*!<TI1 Selection */
            __IO uint32_t OIS1 : 1;                  /*!<Output Idle state 1 (OC1 output)  */
            __IO uint32_t OIS1N : 1;                 /*!<Output Idle state 1 (OC1N output) */
            __IO uint32_t OIS2 : 1;                  /*!<Output Idle state 2 (OC2 output)  */
            __IO uint32_t OIS2N : 1;                 /*!<Output Idle state 2 (OC2N output) */
            __IO uint32_t OIS3 : 1;                  /*!<Output Idle state 3 (OC3 output)  */
            __IO uint32_t OIS3N : 1;                 /*!<Output Idle state 3 (OC3N output) */
            __IO uint32_t OIS4 : 1;                  /*!<Output Idle state 4 (OC4 output)  */
                 uint32_t __RESERVED1 : 17;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< TIM control register 2,              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t SMS : 3;                   /*!<SMS[2:0] bits (Slave mode selection)    */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TS : 3;                    /*!<TS[2:0] bits (Trigger selection)        */
            __IO uint32_t MSM : 1;                   /*!<Master/slave mode                       */
            __IO uint32_t ETF : 4;                   /*!<ETF[3:0] bits (External trigger filter) */
            __IO uint32_t ETPS : 2;                  /*!<ETPS[1:0] bits (External trigger prescaler) */
            __IO uint32_t ECE : 1;                   /*!<External clock enable     */
            __IO uint32_t ETP : 1;                   /*!<External trigger polarity */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } SMCR;                                  /*!< TIM slave mode control register,     Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t UIE : 1;                   /*!<Update interrupt enable */
            __IO uint32_t CC1IE : 1;                 /*!<Capture/Compare 1 interrupt enable   */
            __IO uint32_t CC2IE : 1;                 /*!<Capture/Compare 2 interrupt enable   */
            __IO uint32_t CC3IE : 1;                 /*!<Capture/Compare 3 interrupt enable   */
            __IO uint32_t CC4IE : 1;                 /*!<Capture/Compare 4 interrupt enable   */
            __IO uint32_t COMIE : 1;                 /*!<COM interrupt enable                 */
            __IO uint32_t TIE : 1;                   /*!<Trigger interrupt enable             */
            __IO uint32_t BIE : 1;                   /*!<Break interrupt enable               */
            __IO uint32_t UDE : 1;                   /*!<Update DMA request enable            */
            __IO uint32_t CC1DE : 1;                 /*!<Capture/Compare 1 DMA request enable */
            __IO uint32_t CC2DE : 1;                 /*!<Capture/Compare 2 DMA request enable */
            __IO uint32_t CC3DE : 1;                 /*!<Capture/Compare 3 DMA request enable */
            __IO uint32_t CC4DE : 1;                 /*!<Capture/Compare 4 DMA request enable */
            __IO uint32_t COMDE : 1;                 /*!<COM DMA request enable               */
            __IO uint32_t TDE : 1;                   /*!<Trigger DMA request enable           */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } DIER;                                  /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t UIF : 1;                   /*!<Update interrupt Flag              */
            __IO uint32_t CC1IF : 1;                 /*!<Capture/Compare 1 interrupt Flag   */
            __IO uint32_t CC2IF : 1;                 /*!<Capture/Compare 2 interrupt Flag   */
            __IO uint32_t CC3IF : 1;                 /*!<Capture/Compare 3 interrupt Flag   */
            __IO uint32_t CC4IF : 1;                 /*!<Capture/Compare 4 interrupt Flag   */
            __IO uint32_t COMIF : 1;                 /*!<COM interrupt Flag                 */
            __IO uint32_t TIF : 1;                   /*!<Trigger interrupt Flag             */
            __IO uint32_t BIF : 1;                   /*!<Break interrupt Flag               */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CC1OF : 1;                 /*!<Capture/Compare 1 Overcapture Flag */
            __IO uint32_t CC2OF : 1;                 /*!<Capture/Compare 2 Overcapture Flag */
            __IO uint32_t CC3OF : 1;                 /*!<Capture/Compare 3 Overcapture Flag */
            __IO uint32_t CC4OF : 1;                 /*!<Capture/Compare 4 Overcapture Flag */
                 uint32_t __RESERVED1 : 19;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< TIM status register,                 Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t UG : 1;                    /*!<Update Generation                         */
            __IO uint32_t CC1G : 1;                  /*!<Capture/Compare 1 Generation              */
            __IO uint32_t CC2G : 1;                  /*!<Capture/Compare 2 Generation              */
            __IO uint32_t CC3G : 1;                  /*!<Capture/Compare 3 Generation              */
            __IO uint32_t CC4G : 1;                  /*!<Capture/Compare 4 Generation              */
            __IO uint32_t COMG : 1;                  /*!<Capture/Compare Control Update Generation */
            __IO uint32_t TG : 1;                    /*!<Trigger Generation                        */
            __IO uint32_t BG : 1;                    /*!<Break Generation                          */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } EGR;                                   /*!< TIM event generation register,       Address offset: 0x14 */
    union {
        struct {
			__IO uint32_t C1S : 2;                   /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
			__IO uint32_t C1FE : 1;                  /*!< Output Compare 1 Fast enable */
			__IO uint32_t C1PE : 1;                  /*!< Output Compare 1 Preload enable */
			__IO uint32_t C1M : 3;                   /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
			__IO uint32_t C1CE : 1;                  /*!< Output Compare 1 Clear Enable */
			__IO uint32_t C2S : 2;                   /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
			__IO uint32_t C2FE : 1;                  /*!< Output Compare 2 Fast enable */
			__IO uint32_t C2PE : 1;                  /*!< Output Compare 2 Preload enable */
			__IO uint32_t C2M : 3;                   /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
			__IO uint32_t C2CE : 1;                  /*!< Output Compare 2 Clear Enable */
				 uint32_t __RESERVED0 : 16;
        } OC;                                    /*!< TIM CCMR register Output Compare configuration mode */
        struct {
			__IO uint32_t C1S : 2;                   /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
			__IO uint32_t C1PSC : 2;                 /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
			__IO uint32_t C1F : 4;                   /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
			__IO uint32_t C2S : 2;                   /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
			__IO uint32_t C2PSC : 2;                 /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
			__IO uint32_t C2F : 4;                   /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
				 uint32_t __RESERVED0 : 16;
        } IC;                                    /*!< TIM CCMR register Input Capture configuration mode */
        __IO uint32_t w;
    } CCMR1;                                 /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
    union {
        struct {
			__IO uint32_t C3S : 2;                   /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
			__IO uint32_t C3FE : 1;                  /*!< Output Compare 3 Fast enable */
			__IO uint32_t C3PE : 1;                  /*!< Output Compare 3 Preload enable */
			__IO uint32_t C3M : 3;                   /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
			__IO uint32_t C3CE : 1;                  /*!< Output Compare 3 Clear Enable */
			__IO uint32_t C4S : 2;                   /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
			__IO uint32_t C4FE : 1;                  /*!< Output Compare 4 Fast enable */
			__IO uint32_t C4PE : 1;                  /*!< Output Compare 4 Preload enable */
			__IO uint32_t C4M : 3;                   /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
			__IO uint32_t C4CE : 1;                  /*!< Output Compare 4 Clear Enable */
				 uint32_t __RESERVED0 : 16;
        } OC;                                    /*!< TIM CCMR register Output Compare configuration mode */
        struct {
			__IO uint32_t C3S : 2;                   /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
			__IO uint32_t C3PSC : 2;                 /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
			__IO uint32_t C3F : 4;                   /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
			__IO uint32_t C4S : 2;                   /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
			__IO uint32_t C4PSC : 2;                 /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
			__IO uint32_t C4F : 4;                   /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
				 uint32_t __RESERVED0 : 16;
        } IC;                                    /*!< TIM CCMR register Input Capture configuration mode */
        __IO uint32_t w;
    } CCMR2;                                 /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t CC1E : 1;                  /*!<Capture/Compare 1 output enable                 */
            __IO uint32_t CC1P : 1;                  /*!<Capture/Compare 1 output Polarity               */
            __IO uint32_t CC1NE : 1;                 /*!<Capture/Compare 1 Complementary output enable   */
            __IO uint32_t CC1NP : 1;                 /*!<Capture/Compare 1 Complementary output Polarity */
            __IO uint32_t CC2E : 1;                  /*!<Capture/Compare 2 output enable                 */
            __IO uint32_t CC2P : 1;                  /*!<Capture/Compare 2 output Polarity               */
            __IO uint32_t CC2NE : 1;                 /*!<Capture/Compare 2 Complementary output enable   */
            __IO uint32_t CC2NP : 1;                 /*!<Capture/Compare 2 Complementary output Polarity */
            __IO uint32_t CC3E : 1;                  /*!<Capture/Compare 3 output enable                 */
            __IO uint32_t CC3P : 1;                  /*!<Capture/Compare 3 output Polarity               */
            __IO uint32_t CC3NE : 1;                 /*!<Capture/Compare 3 Complementary output enable   */
            __IO uint32_t CC3NP : 1;                 /*!<Capture/Compare 3 Complementary output Polarity */
            __IO uint32_t CC4E : 1;                  /*!<Capture/Compare 4 output enable                 */
            __IO uint32_t CC4P : 1;                  /*!<Capture/Compare 4 output Polarity               */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CC4NP : 1;                 /*!<Capture/Compare 4 Complementary output Polarity */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CCER;                                  /*!< TIM capture/compare enable register, Address offset: 0x20 */
    __IO uint32_t CNT;                       /*!< TIM counter register,                Address offset: 0x24 */
    __IO uint32_t PSC;                       /*!< TIM prescaler,                       Address offset: 0x28 */
    __IO uint32_t ARR;                       /*!< TIM auto-reload register,            Address offset: 0x2C */
    __IO uint32_t RCR;                       /*!< TIM repetition counter register,     Address offset: 0x30 */
    __IO uint32_t CCR1;                      /*!< TIM capture/compare register 1,      Address offset: 0x34 */
    __IO uint32_t CCR2;                      /*!< TIM capture/compare register 2,      Address offset: 0x38 */
    __IO uint32_t CCR3;                      /*!< TIM capture/compare register 3,      Address offset: 0x3C */
    __IO uint32_t CCR4;                      /*!< TIM capture/compare register 4,      Address offset: 0x40 */
    union {
        struct {
            __IO uint32_t DTG : 8;                   /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
            __IO uint32_t LOCK : 2;                  /*!<LOCK[1:0] bits (Lock Configuration) */
            __IO uint32_t OSSI : 1;                  /*!<Off-State Selection for Idle mode */
            __IO uint32_t OSSR : 1;                  /*!<Off-State Selection for Run mode  */
            __IO uint32_t BKE : 1;                   /*!<Break enable                      */
            __IO uint32_t BKP : 1;                   /*!<Break Polarity                    */
            __IO uint32_t AOE : 1;                   /*!<Automatic Output enable           */
            __IO uint32_t MOE : 1;                   /*!<Main Output enable                */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } BDTR;                                  /*!< TIM break and dead-time register,    Address offset: 0x44 */
    union {
        struct {
            __IO uint32_t DBA : 5;                   /*!<DBA[4:0] bits (DMA Base Address) */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t DBL : 5;                   /*!<DBL[4:0] bits (DMA Burst Length) */
                 uint32_t __RESERVED1 : 19;
        } b;
        __IO uint32_t w;
    } DCR;                                   /*!< TIM DMA control register,            Address offset: 0x48 */
    __IO uint32_t DMAR;                      /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    union {
        struct {
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t TI4_RMP : 2;               /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ITR1_RMP : 2;              /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
                 uint32_t __RESERVED2 : 20;
        } b;
        __IO uint32_t w;
    } OR;                                    /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CEN;                       /*!<Counter enable        */
        __IO uint32_t UDIS;                      /*!<Update disable        */
        __IO uint32_t URS;                       /*!<Update request source */
        __IO uint32_t OPM;                       /*!<One pulse mode        */
        __IO uint32_t DIR;                       /*!<Direction             */
        __IO uint32_t CMS[2];                    /*!<CMS[1:0] bits (Center-aligned mode selection) */
        __IO uint32_t ARPE;                      /*!<Auto-reload preload enable     */
        __IO uint32_t CKD[2];                    /*!<CKD[1:0] bits (clock division) */
             uint32_t __RESERVED0[22];
    } CR1;                                   /*!< TIM control register 1,              Address offset: 0x00 */
    struct {
        __IO uint32_t CCPC;                      /*!<Capture/Compare Preloaded Control        */
             uint32_t __RESERVED0;
        __IO uint32_t CCUS;                      /*!<Capture/Compare Control Update Selection */
        __IO uint32_t CCDS;                      /*!<Capture/Compare DMA Selection            */
        __IO uint32_t MMS[3];                    /*!<MMS[2:0] bits (Master Mode Selection) */
        __IO uint32_t TI1S;                      /*!<TI1 Selection */
        __IO uint32_t OIS1;                      /*!<Output Idle state 1 (OC1 output)  */
        __IO uint32_t OIS1N;                     /*!<Output Idle state 1 (OC1N output) */
        __IO uint32_t OIS2;                      /*!<Output Idle state 2 (OC2 output)  */
        __IO uint32_t OIS2N;                     /*!<Output Idle state 2 (OC2N output) */
        __IO uint32_t OIS3;                      /*!<Output Idle state 3 (OC3 output)  */
        __IO uint32_t OIS3N;                     /*!<Output Idle state 3 (OC3N output) */
        __IO uint32_t OIS4;                      /*!<Output Idle state 4 (OC4 output)  */
             uint32_t __RESERVED1[17];
    } CR2;                                   /*!< TIM control register 2,              Address offset: 0x04 */
    struct {
        __IO uint32_t SMS[3];                    /*!<SMS[2:0] bits (Slave mode selection)    */
             uint32_t __RESERVED0;
        __IO uint32_t TS[3];                     /*!<TS[2:0] bits (Trigger selection)        */
        __IO uint32_t MSM;                       /*!<Master/slave mode                       */
        __IO uint32_t ETF[4];                    /*!<ETF[3:0] bits (External trigger filter) */
        __IO uint32_t ETPS[2];                   /*!<ETPS[1:0] bits (External trigger prescaler) */
        __IO uint32_t ECE;                       /*!<External clock enable     */
        __IO uint32_t ETP;                       /*!<External trigger polarity */
             uint32_t __RESERVED1[16];
    } SMCR;                                  /*!< TIM slave mode control register,     Address offset: 0x08 */
    struct {
        __IO uint32_t UIE;                       /*!<Update interrupt enable */
        __IO uint32_t CC1IE;                     /*!<Capture/Compare 1 interrupt enable   */
        __IO uint32_t CC2IE;                     /*!<Capture/Compare 2 interrupt enable   */
        __IO uint32_t CC3IE;                     /*!<Capture/Compare 3 interrupt enable   */
        __IO uint32_t CC4IE;                     /*!<Capture/Compare 4 interrupt enable   */
        __IO uint32_t COMIE;                     /*!<COM interrupt enable                 */
        __IO uint32_t TIE;                       /*!<Trigger interrupt enable             */
        __IO uint32_t BIE;                       /*!<Break interrupt enable               */
        __IO uint32_t UDE;                       /*!<Update DMA request enable            */
        __IO uint32_t CC1DE;                     /*!<Capture/Compare 1 DMA request enable */
        __IO uint32_t CC2DE;                     /*!<Capture/Compare 2 DMA request enable */
        __IO uint32_t CC3DE;                     /*!<Capture/Compare 3 DMA request enable */
        __IO uint32_t CC4DE;                     /*!<Capture/Compare 4 DMA request enable */
        __IO uint32_t COMDE;                     /*!<COM DMA request enable               */
        __IO uint32_t TDE;                       /*!<Trigger DMA request enable           */
             uint32_t __RESERVED0[17];
    } DIER;                                  /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    struct {
        __IO uint32_t UIF;                       /*!<Update interrupt Flag              */
        __IO uint32_t CC1IF;                     /*!<Capture/Compare 1 interrupt Flag   */
        __IO uint32_t CC2IF;                     /*!<Capture/Compare 2 interrupt Flag   */
        __IO uint32_t CC3IF;                     /*!<Capture/Compare 3 interrupt Flag   */
        __IO uint32_t CC4IF;                     /*!<Capture/Compare 4 interrupt Flag   */
        __IO uint32_t COMIF;                     /*!<COM interrupt Flag                 */
        __IO uint32_t TIF;                       /*!<Trigger interrupt Flag             */
        __IO uint32_t BIF;                       /*!<Break interrupt Flag               */
             uint32_t __RESERVED0;
        __IO uint32_t CC1OF;                     /*!<Capture/Compare 1 Overcapture Flag */
        __IO uint32_t CC2OF;                     /*!<Capture/Compare 2 Overcapture Flag */
        __IO uint32_t CC3OF;                     /*!<Capture/Compare 3 Overcapture Flag */
        __IO uint32_t CC4OF;                     /*!<Capture/Compare 4 Overcapture Flag */
             uint32_t __RESERVED1[19];
    } SR;                                    /*!< TIM status register,                 Address offset: 0x10 */
    struct {
        __IO uint32_t UG;                        /*!<Update Generation                         */
        __IO uint32_t CC1G;                      /*!<Capture/Compare 1 Generation              */
        __IO uint32_t CC2G;                      /*!<Capture/Compare 2 Generation              */
        __IO uint32_t CC3G;                      /*!<Capture/Compare 3 Generation              */
        __IO uint32_t CC4G;                      /*!<Capture/Compare 4 Generation              */
        __IO uint32_t COMG;                      /*!<Capture/Compare Control Update Generation */
        __IO uint32_t TG;                        /*!<Trigger Generation                        */
        __IO uint32_t BG;                        /*!<Break Generation                          */
             uint32_t __RESERVED0[24];
    } EGR;                                   /*!< TIM event generation register,       Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t C1S[2];                    /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
            __IO uint32_t C1FE;                      /*!< Output Compare 1 Fast enable */
            __IO uint32_t C1PE;                      /*!< Output Compare 1 Preload enable */
            __IO uint32_t C1M[3];                    /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
            __IO uint32_t C1CE;                      /*!< Output Compare 1 Clear Enable */
            __IO uint32_t C2S[2];                    /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
            __IO uint32_t C2FE;                      /*!< Output Compare 2 Fast enable */
            __IO uint32_t C2PE;                      /*!< Output Compare 2 Preload enable */
            __IO uint32_t C2M[3];                    /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
            __IO uint32_t C2CE;                      /*!< Output Compare 2 Clear Enable */
                 uint32_t __RESERVED0[16];
        } OC;                                    /*!< TIM CCMR register Output Compare configuration mode */
        struct {
            __IO uint32_t C1S[2];                    /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
            __IO uint32_t C1PSC[2];                  /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
            __IO uint32_t C1F[4];                    /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
            __IO uint32_t C2S[2];                    /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
            __IO uint32_t C2PSC[2];                  /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
            __IO uint32_t C2F[4];                    /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
                 uint32_t __RESERVED0[16];
        } IC;                                    /*!< TIM CCMR register Input Capture configuration mode */
    } CCMR1;                                 /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t C3S[2];                    /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
            __IO uint32_t C3FE;                      /*!< Output Compare 3 Fast enable */
            __IO uint32_t C3PE;                      /*!< Output Compare 3 Preload enable */
            __IO uint32_t C3M[3];                    /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
            __IO uint32_t C3CE;                      /*!< Output Compare 3 Clear Enable */
            __IO uint32_t C4S[2];                    /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
            __IO uint32_t C4FE;                      /*!< Output Compare 4 Fast enable */
            __IO uint32_t C4PE;                      /*!< Output Compare 4 Preload enable */
            __IO uint32_t C4M[3];                    /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
            __IO uint32_t C4CE;                      /*!< Output Compare 4 Clear Enable */
                 uint32_t __RESERVED0[16];
        } OC;                                    /*!< TIM CCMR register Output Compare configuration mode */
        struct {
            __IO uint32_t C3S[2];                    /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
            __IO uint32_t C3PSC[2];                  /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
            __IO uint32_t C3F[4];                    /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
            __IO uint32_t C4S[2];                    /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
            __IO uint32_t C4PSC[2];                  /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
            __IO uint32_t C4F[4];                    /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
                 uint32_t __RESERVED0[16];
        } IC;                                    /*!< TIM CCMR register Input Capture configuration mode */
    } CCMR2;                                 /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
    struct {
        __IO uint32_t CC1E;                      /*!<Capture/Compare 1 output enable                 */
        __IO uint32_t CC1P;                      /*!<Capture/Compare 1 output Polarity               */
        __IO uint32_t CC1NE;                     /*!<Capture/Compare 1 Complementary output enable   */
        __IO uint32_t CC1NP;                     /*!<Capture/Compare 1 Complementary output Polarity */
        __IO uint32_t CC2E;                      /*!<Capture/Compare 2 output enable                 */
        __IO uint32_t CC2P;                      /*!<Capture/Compare 2 output Polarity               */
        __IO uint32_t CC2NE;                     /*!<Capture/Compare 2 Complementary output enable   */
        __IO uint32_t CC2NP;                     /*!<Capture/Compare 2 Complementary output Polarity */
        __IO uint32_t CC3E;                      /*!<Capture/Compare 3 output enable                 */
        __IO uint32_t CC3P;                      /*!<Capture/Compare 3 output Polarity               */
        __IO uint32_t CC3NE;                     /*!<Capture/Compare 3 Complementary output enable   */
        __IO uint32_t CC3NP;                     /*!<Capture/Compare 3 Complementary output Polarity */
        __IO uint32_t CC4E;                      /*!<Capture/Compare 4 output enable                 */
        __IO uint32_t CC4P;                      /*!<Capture/Compare 4 output Polarity               */
             uint32_t __RESERVED0;
        __IO uint32_t CC4NP;                     /*!<Capture/Compare 4 Complementary output Polarity */
             uint32_t __RESERVED1[16];
    } CCER;                                  /*!< TIM capture/compare enable register, Address offset: 0x20 */
    __IO uint32_t CNT[32];                   /*!< TIM counter register,                Address offset: 0x24 */
    __IO uint32_t PSC[32];                   /*!< TIM prescaler,                       Address offset: 0x28 */
    __IO uint32_t ARR[32];                   /*!< TIM auto-reload register,            Address offset: 0x2C */
    __IO uint32_t RCR[32];                   /*!< TIM repetition counter register,     Address offset: 0x30 */
    __IO uint32_t CCR1[32];                  /*!< TIM capture/compare register 1,      Address offset: 0x34 */
    __IO uint32_t CCR2[32];                  /*!< TIM capture/compare register 2,      Address offset: 0x38 */
    __IO uint32_t CCR3[32];                  /*!< TIM capture/compare register 3,      Address offset: 0x3C */
    __IO uint32_t CCR4[32];                  /*!< TIM capture/compare register 4,      Address offset: 0x40 */
    struct {
        __IO uint32_t DTG[8];                    /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
        __IO uint32_t LOCK[2];                   /*!<LOCK[1:0] bits (Lock Configuration) */
        __IO uint32_t OSSI;                      /*!<Off-State Selection for Idle mode */
        __IO uint32_t OSSR;                      /*!<Off-State Selection for Run mode  */
        __IO uint32_t BKE;                       /*!<Break enable                      */
        __IO uint32_t BKP;                       /*!<Break Polarity                    */
        __IO uint32_t AOE;                       /*!<Automatic Output enable           */
        __IO uint32_t MOE;                       /*!<Main Output enable                */
             uint32_t __RESERVED0[16];
    } BDTR;                                  /*!< TIM break and dead-time register,    Address offset: 0x44 */
    struct {
        __IO uint32_t DBA[5];                    /*!<DBA[4:0] bits (DMA Base Address) */
             uint32_t __RESERVED0[3];
        __IO uint32_t DBL[5];                    /*!<DBL[4:0] bits (DMA Burst Length) */
             uint32_t __RESERVED1[19];
    } DCR;                                   /*!< TIM DMA control register,            Address offset: 0x48 */
    __IO uint32_t DMAR[32];                  /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    struct {
             uint32_t __RESERVED0[6];
        __IO uint32_t TI4_RMP[2];                /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
             uint32_t __RESERVED1[2];
        __IO uint32_t ITR1_RMP[2];               /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
             uint32_t __RESERVED2[20];
    } OR;                                    /*!< TIM option register,                 Address offset: 0x50 */
} TIM_BitBand_TypeDef;



/** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
 

typedef struct {
    union {
        struct {
            __IO uint32_t PE : 1;                    /*!<Parity Error                 */
            __IO uint32_t FE : 1;                    /*!<Framing Error                */
            __IO uint32_t NE : 1;                    /*!<Noise Error Flag             */
            __IO uint32_t ORE : 1;                   /*!<OverRun Error                */
            __IO uint32_t IDLE : 1;                  /*!<IDLE line detected           */
            __IO uint32_t RXNE : 1;                  /*!<Read Data Register Not Empty */
            __IO uint32_t TC : 1;                    /*!<Transmission Complete        */
            __IO uint32_t TXE : 1;                   /*!<Transmit Data Register Empty */
            __IO uint32_t LBD : 1;                   /*!<LIN Break Detection Flag     */
            __IO uint32_t CTS : 1;                   /*!<CTS Flag                     */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< USART Status register,                   Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t DR : 9;                    /*!<Data value */
                 uint32_t __RESERVED0 : 23;
        } b;
        __IO uint32_t w;
    } DR;                                    /*!< USART Data register,                     Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t DIV_Fraction : 4;          /*!<Fraction of USARTDIV */
            __IO uint32_t DIV_Mantissa : 12;         /*!<Mantissa of USARTDIV */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } BRR;                                   /*!< USART Baud rate register,                Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t SBK : 1;                   /*!<Send Break                             */
            __IO uint32_t RWU : 1;                   /*!<Receiver wakeup                        */
            __IO uint32_t RE : 1;                    /*!<Receiver Enable                        */
            __IO uint32_t TE : 1;                    /*!<Transmitter Enable                     */
            __IO uint32_t IDLEIE : 1;                /*!<IDLE Interrupt Enable                  */
            __IO uint32_t RXNEIE : 1;                /*!<RXNE Interrupt Enable                  */
            __IO uint32_t TCIE : 1;                  /*!<Transmission Complete Interrupt Enable */
            __IO uint32_t TXEIE : 1;                 /*!<PE Interrupt Enable                    */
            __IO uint32_t PEIE : 1;                  /*!<PE Interrupt Enable                    */
            __IO uint32_t PS : 1;                    /*!<Parity Selection                       */
            __IO uint32_t PCE : 1;                   /*!<Parity Control Enable                  */
            __IO uint32_t WAKE : 1;                  /*!<Wakeup method                          */
            __IO uint32_t M : 1;                     /*!<Word length                            */
            __IO uint32_t UE : 1;                    /*!<USART Enable                           */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t OVER8 : 1;                 /*!<USART Oversampling by 8 enable         */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< USART Control register 1,                Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t ADD : 4;                   /*!<Address of the USART node            */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t LBDL : 1;                  /*!<LIN Break Detection Length           */
            __IO uint32_t LBDIE : 1;                 /*!<LIN Break Detection Interrupt Enable */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t LBCL : 1;                  /*!<Last Bit Clock pulse                 */
            __IO uint32_t CPHA : 1;                  /*!<Clock Phase                          */
            __IO uint32_t CPOL : 1;                  /*!<Clock Polarity                       */
            __IO uint32_t CLKEN : 1;                 /*!<Clock Enable                         */
            __IO uint32_t STOP : 2;                  /*!<STOP[1:0] bits (STOP bits) */
            __IO uint32_t LINEN : 1;                 /*!<LIN mode enable */
                 uint32_t __RESERVED2 : 17;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< USART Control register 2,                Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t EIE : 1;                   /*!<Error Interrupt Enable      */
            __IO uint32_t IREN : 1;                  /*!<IrDA mode Enable            */
            __IO uint32_t IRLP : 1;                  /*!<IrDA Low-Power              */
            __IO uint32_t HDSEL : 1;                 /*!<Half-Duplex Selection       */
            __IO uint32_t NACK : 1;                  /*!<Smartcard NACK enable       */
            __IO uint32_t SCEN : 1;                  /*!<Smartcard mode enable       */
            __IO uint32_t DMAR : 1;                  /*!<DMA Enable Receiver         */
            __IO uint32_t DMAT : 1;                  /*!<DMA Enable Transmitter      */
            __IO uint32_t RTSE : 1;                  /*!<RTS Enable                  */
            __IO uint32_t CTSE : 1;                  /*!<CTS Enable                  */
            __IO uint32_t CTSIE : 1;                 /*!<CTS Interrupt Enable        */
            __IO uint32_t ONEBIT : 1;                /*!<USART One bit method enable */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } CR3;                                   /*!< USART Control register 3,                Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t PSC : 8;                   /*!<PSC[7:0] bits (Prescaler value) */
            __IO uint32_t GT : 8;                    /*!<Guard time value */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } GTPR;                                  /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PE;                        /*!<Parity Error                 */
        __IO uint32_t FE;                        /*!<Framing Error                */
        __IO uint32_t NE;                        /*!<Noise Error Flag             */
        __IO uint32_t ORE;                       /*!<OverRun Error                */
        __IO uint32_t IDLE;                      /*!<IDLE line detected           */
        __IO uint32_t RXNE;                      /*!<Read Data Register Not Empty */
        __IO uint32_t TC;                        /*!<Transmission Complete        */
        __IO uint32_t TXE;                       /*!<Transmit Data Register Empty */
        __IO uint32_t LBD;                       /*!<LIN Break Detection Flag     */
        __IO uint32_t CTS;                       /*!<CTS Flag                     */
             uint32_t __RESERVED0[22];
    } SR;                                    /*!< USART Status register,                   Address offset: 0x00 */
    struct {
        __IO uint32_t DR[9];                     /*!<Data value */
             uint32_t __RESERVED0[23];
    } DR;                                    /*!< USART Data register,                     Address offset: 0x04 */
    struct {
        __IO uint32_t DIV_Fraction[4];           /*!<Fraction of USARTDIV */
        __IO uint32_t DIV_Mantissa[12];          /*!<Mantissa of USARTDIV */
             uint32_t __RESERVED0[16];
    } BRR;                                   /*!< USART Baud rate register,                Address offset: 0x08 */
    struct {
        __IO uint32_t SBK;                       /*!<Send Break                             */
        __IO uint32_t RWU;                       /*!<Receiver wakeup                        */
        __IO uint32_t RE;                        /*!<Receiver Enable                        */
        __IO uint32_t TE;                        /*!<Transmitter Enable                     */
        __IO uint32_t IDLEIE;                    /*!<IDLE Interrupt Enable                  */
        __IO uint32_t RXNEIE;                    /*!<RXNE Interrupt Enable                  */
        __IO uint32_t TCIE;                      /*!<Transmission Complete Interrupt Enable */
        __IO uint32_t TXEIE;                     /*!<PE Interrupt Enable                    */
        __IO uint32_t PEIE;                      /*!<PE Interrupt Enable                    */
        __IO uint32_t PS;                        /*!<Parity Selection                       */
        __IO uint32_t PCE;                       /*!<Parity Control Enable                  */
        __IO uint32_t WAKE;                      /*!<Wakeup method                          */
        __IO uint32_t M;                         /*!<Word length                            */
        __IO uint32_t UE;                        /*!<USART Enable                           */
             uint32_t __RESERVED0;
        __IO uint32_t OVER8;                     /*!<USART Oversampling by 8 enable         */
             uint32_t __RESERVED1[16];
    } CR1;                                   /*!< USART Control register 1,                Address offset: 0x0C */
    struct {
        __IO uint32_t ADD[4];                    /*!<Address of the USART node            */
             uint32_t __RESERVED0;
        __IO uint32_t LBDL;                      /*!<LIN Break Detection Length           */
        __IO uint32_t LBDIE;                     /*!<LIN Break Detection Interrupt Enable */
             uint32_t __RESERVED1;
        __IO uint32_t LBCL;                      /*!<Last Bit Clock pulse                 */
        __IO uint32_t CPHA;                      /*!<Clock Phase                          */
        __IO uint32_t CPOL;                      /*!<Clock Polarity                       */
        __IO uint32_t CLKEN;                     /*!<Clock Enable                         */
        __IO uint32_t STOP[2];                   /*!<STOP[1:0] bits (STOP bits) */
        __IO uint32_t LINEN;                     /*!<LIN mode enable */
             uint32_t __RESERVED2[17];
    } CR2;                                   /*!< USART Control register 2,                Address offset: 0x10 */
    struct {
        __IO uint32_t EIE;                       /*!<Error Interrupt Enable      */
        __IO uint32_t IREN;                      /*!<IrDA mode Enable            */
        __IO uint32_t IRLP;                      /*!<IrDA Low-Power              */
        __IO uint32_t HDSEL;                     /*!<Half-Duplex Selection       */
        __IO uint32_t NACK;                      /*!<Smartcard NACK enable       */
        __IO uint32_t SCEN;                      /*!<Smartcard mode enable       */
        __IO uint32_t DMAR;                      /*!<DMA Enable Receiver         */
        __IO uint32_t DMAT;                      /*!<DMA Enable Transmitter      */
        __IO uint32_t RTSE;                      /*!<RTS Enable                  */
        __IO uint32_t CTSE;                      /*!<CTS Enable                  */
        __IO uint32_t CTSIE;                     /*!<CTS Interrupt Enable        */
        __IO uint32_t ONEBIT;                    /*!<USART One bit method enable */
             uint32_t __RESERVED0[20];
    } CR3;                                   /*!< USART Control register 3,                Address offset: 0x14 */
    struct {
        __IO uint32_t PSC[8];                    /*!<PSC[7:0] bits (Prescaler value) */
        __IO uint32_t GT[8];                     /*!<Guard time value */
             uint32_t __RESERVED0[16];
    } GTPR;                                  /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_BitBand_TypeDef;



/** 
  * @brief Window WATCHDOG
  */


typedef struct {
    union {
        struct {
            __IO uint32_t T : 7;                     /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
            __IO uint32_t WDGA : 1;                  /*!<Activation bit */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< WWDG Control register,       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t W : 7;                     /*!<W[6:0] bits (7-bit window value) */
            __IO uint32_t WDGTB : 2;                 /*!<WDGTB[1:0] bits (Timer Base) */
            __IO uint32_t EWI : 1;                   /*!<Early Wakeup Interrupt */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } CFR;                                   /*!< WWDG Configuration register, Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t EWIF : 1;                  /*!<Early Wakeup Interrupt Flag */
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;


typedef struct {
    struct {
        __IO uint32_t T[7];                      /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
        __IO uint32_t WDGA;                      /*!<Activation bit */
             uint32_t __RESERVED0[24];
    } CR;                                    /*!< WWDG Control register,       Address offset: 0x00 */
    struct {
        __IO uint32_t W[7];                      /*!<W[6:0] bits (7-bit window value) */
        __IO uint32_t WDGTB[2];                  /*!<WDGTB[1:0] bits (Timer Base) */
        __IO uint32_t EWI;                       /*!<Early Wakeup Interrupt */
             uint32_t __RESERVED0[22];
    } CFR;                                   /*!< WWDG Configuration register, Address offset: 0x04 */
    struct {
        __IO uint32_t EWIF;                      /*!<Early Wakeup Interrupt Flag */
             uint32_t __RESERVED0[31];
    } SR;                                    /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_BitBand_TypeDef;



/** 
  * @brief RNG
  */
  

typedef struct {
    union {
        struct {
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t RNGEN : 1;
            __IO uint32_t IE : 1;
                 uint32_t __RESERVED1 : 28;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< RNG control register, Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t DRDY : 1;
            __IO uint32_t CECS : 1;
            __IO uint32_t SECS : 1;
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t CEIS : 1;
            __IO uint32_t SEIS : 1;
                 uint32_t __RESERVED1 : 25;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< RNG status register,  Address offset: 0x04 */
    __IO uint32_t DR;                        /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;



 
/** 
  * @brief __USB_OTG_Core_register
  */

typedef struct {
    union {
        struct {
            __IO uint32_t SRQSCS : 1;                /*!< Session request success */
            __IO uint32_t SRQ : 1;                   /*!< Session request */
                 uint32_t __RESERVED0 : 6;
            __IO uint32_t HNGSCS : 1;                /*!< Host negotiation success */
            __IO uint32_t HNPRQ : 1;                 /*!< HNP request */
            __IO uint32_t HSHNPEN : 1;               /*!< Host set HNP enable */
            __IO uint32_t DHNPEN : 1;                /*!< Device HNP enabled */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t CIDSTS : 1;                /*!< Connector ID status */
            __IO uint32_t DBCT : 1;                  /*!< Long/short debounce time */
            __IO uint32_t ASVLD : 1;                 /*!< A-session valid */
            __IO uint32_t BSVLD : 1;                 /*!< B-session valid */
                 uint32_t __RESERVED2 : 12;
        } b;
        __IO uint32_t w;
    } GOTGCTL;                               /*!<  USB_OTG Control and Status Register    000h*/
    union {
        struct {
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t SEDET : 1;                 /*!< Session end detected */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t SRSSCHG : 1;               /*!< Session request success status change */
            __IO uint32_t HNSSCHG : 1;               /*!< Host negotiation success status change */
                 uint32_t __RESERVED2 : 7;
            __IO uint32_t HNGDET : 1;                /*!< Host negotiation detected */
            __IO uint32_t ADTOCHG : 1;               /*!< A-device timeout change */
            __IO uint32_t DBCDNE : 1;                /*!< Debounce done */
                 uint32_t __RESERVED3 : 12;
        } b;
        __IO uint32_t w;
    } GOTGINT;                               /*!<  USB_OTG Interrupt Register             004h*/
    union {
        struct {
            __IO uint32_t GINT : 1;                  /*!< Global interrupt mask */
            __IO uint32_t HBSTLEN : 4;               /*!< Burst length/type */
            __IO uint32_t DMAEN : 1;                 /*!< DMA enable */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TXFELVL : 1;               /*!< TxFIFO empty level */
            __IO uint32_t PTXFELVL : 1;              /*!< Periodic TxFIFO empty level */
                 uint32_t __RESERVED1 : 23;
        } b;
        __IO uint32_t w;
    } GAHBCFG;                               /*!<  Core AHB Configuration Register    008h*/
    union {
        struct {
            __IO uint32_t TOCAL : 3;                 /*!< FS timeout calibration */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t PHYSEL : 1;                /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SRPCAP : 1;                /*!< SRP-capable */
            __IO uint32_t HNPCAP : 1;                /*!< HNP-capable */
            __IO uint32_t TRDT : 4;                  /*!< USB turnaround time */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t PHYLPCS : 1;               /*!< PHY Low-power clock select */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t ULPIFSLS : 1;              /*!< ULPI FS/LS select */
            __IO uint32_t ULPIAR : 1;                /*!< ULPI Auto-resume */
            __IO uint32_t ULPICSM : 1;               /*!< ULPI Clock SuspendM */
            __IO uint32_t ULPIEVBUSD : 1;            /*!< ULPI External VBUS Drive */
            __IO uint32_t ULPIEVBUSI : 1;            /*!< ULPI external VBUS indicator */
            __IO uint32_t TSDPS : 1;                 /*!< TermSel DLine pulsing selection */
            __IO uint32_t PCCI : 1;                  /*!< Indicator complement */
            __IO uint32_t PTCI : 1;                  /*!< Indicator pass through */
            __IO uint32_t ULPIIPD : 1;               /*!< ULPI interface protect disable */
                 uint32_t __RESERVED4 : 3;
            __IO uint32_t FHMOD : 1;                 /*!< Forced host mode */
            __IO uint32_t FDMOD : 1;                 /*!< Forced peripheral mode */
            __IO uint32_t CTXPKT : 1;                /*!< Corrupt Tx packet */
        } b;
        __IO uint32_t w;
    } GUSBCFG;                               /*!<  Core USB Configuration Register    00Ch*/
    union {
        struct {
            __IO uint32_t CSRST : 1;                 /*!< Core soft reset */
            __IO uint32_t HSRST : 1;                 /*!< HCLK soft reset */
            __IO uint32_t FCRST : 1;                 /*!< Host frame counter reset */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RXFFLSH : 1;               /*!< RxFIFO flush */
            __IO uint32_t TXFFLSH : 1;               /*!< TxFIFO flush */
            __IO uint32_t TXFNUM : 5;                /*!< TxFIFO number */
                 uint32_t __RESERVED1 : 19;
            __IO uint32_t DMAREQ : 1;                /*!< DMA request signal */
            __IO uint32_t AHBIDL : 1;                /*!< AHB master idle */
        } b;
        __IO uint32_t w;
    } GRSTCTL;                               /*!<  Core Reset Register                010h*/
    union {
        struct {
            __IO uint32_t CMOD : 1;                  /*!< Current mode of operation */
            __IO uint32_t MMIS : 1;                  /*!< Mode mismatch interrupt */
            __IO uint32_t OTGINT : 1;                /*!< OTG interrupt */
            __IO uint32_t SOF : 1;                   /*!< Start of frame */
            __IO uint32_t RXFLVL : 1;                /*!< RxFIFO nonempty */
            __IO uint32_t NPTXFE : 1;                /*!< Nonperiodic TxFIFO empty */
            __IO uint32_t GINAKEFF : 1;              /*!< Global IN nonperiodic NAK effective */
            __IO uint32_t BOUTNAKEFF : 1;            /*!< Global OUT NAK effective */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t ESUSP : 1;                 /*!< Early suspend */
            __IO uint32_t USBSUSP : 1;               /*!< USB suspend */
            __IO uint32_t USBRST : 1;                /*!< USB reset */
            __IO uint32_t ENUMDNE : 1;               /*!< Enumeration done */
            __IO uint32_t ISOODRP : 1;               /*!< Isochronous OUT packet dropped interrupt */
            __IO uint32_t EOPF : 1;                  /*!< End of periodic frame interrupt */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t IEPINT : 1;                /*!< IN endpoint interrupt */
            __IO uint32_t OEPINT : 1;                /*!< OUT endpoint interrupt */
            __IO uint32_t IISOIXFR : 1;              /*!< Incomplete isochronous IN transfer */
            __IO uint32_t PXFR_INCOMPISOOUT : 1;     /*!< Incomplete periodic transfer */
            __IO uint32_t DATAFSUSP : 1;             /*!< Data fetch suspended */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t HPRTINT : 1;               /*!< Host port interrupt */
            __IO uint32_t HCINT : 1;                 /*!< Host channels interrupt */
            __IO uint32_t PTXFE : 1;                 /*!< Periodic TxFIFO empty */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t CIDSCHG : 1;               /*!< Connector ID status change */
            __IO uint32_t DISCINT : 1;               /*!< Disconnect detected interrupt */
            __IO uint32_t SRQINT : 1;                /*!< Session request/new session detected interrupt */
            __IO uint32_t WKUINT : 1;                /*!< Resume/remote wakeup detected interrupt */
        } b;
        __IO uint32_t w;
    } GINTSTS;                               /*!<  Core Interrupt Register            014h*/
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t MMISM : 1;                 /*!< Mode mismatch interrupt mask */
            __IO uint32_t OTGINT : 1;                /*!< OTG interrupt mask */
            __IO uint32_t SOFM : 1;                  /*!< Start of frame mask */
            __IO uint32_t RXFLVLM : 1;               /*!< Receive FIFO nonempty mask */
            __IO uint32_t NPTXFEM : 1;               /*!< Nonperiodic TxFIFO empty mask */
            __IO uint32_t GINAKEFFM : 1;             /*!< Global nonperiodic IN NAK effective mask */
            __IO uint32_t GONAKEFFM : 1;             /*!< Global OUT NAK effective mask */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t ESUSPM : 1;                /*!< Early suspend mask */
            __IO uint32_t USBSUSPM : 1;              /*!< USB suspend mask */
            __IO uint32_t USBRST : 1;                /*!< USB reset mask */
            __IO uint32_t ENUMDNEM : 1;              /*!< Enumeration done mask */
            __IO uint32_t ISOODRPM : 1;              /*!< Isochronous OUT packet dropped interrupt mask */
            __IO uint32_t EOPFM : 1;                 /*!< End of periodic frame interrupt mask */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t EPMISM : 1;                /*!< Endpoint mismatch interrupt mask */
            __IO uint32_t IEPINT : 1;                /*!< IN endpoints interrupt mask */
            __IO uint32_t OEPINT : 1;                /*!< OUT endpoints interrupt mask */
            __IO uint32_t IISOIXFRM : 1;             /*!< Incomplete isochronous IN transfer mask */
            __IO uint32_t PXFRM_IISOOXFRM : 1;       /*!< Incomplete periodic transfer mask */
            __IO uint32_t FSUSPM : 1;                /*!< Data fetch suspended mask */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t PRTIM : 1;                 /*!< Host port interrupt mask */
            __IO uint32_t HCIM : 1;                  /*!< Host channels interrupt mask */
            __IO uint32_t PTXFEM : 1;                /*!< Periodic TxFIFO empty mask */
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t CIDSCHGM : 1;              /*!< Connector ID status change mask */
            __IO uint32_t DISCINT : 1;               /*!< Disconnect detected interrupt mask */
            __IO uint32_t SRQIM : 1;                 /*!< Session request/new session detected interrupt mask */
            __IO uint32_t WUIM : 1;                  /*!< Resume/remote wakeup detected interrupt mask */
        } b;
        __IO uint32_t w;
    } GINTMSK;                               /*!<  Core Interrupt Mask Register       018h*/
    union {
        struct {
            __IO uint32_t CHNUM : 4;
            __IO uint32_t BCNT : 11;
            __IO uint32_t DPID : 2;
            __IO uint32_t PKTSTS : 4;
            __IO uint32_t FRMNUM : 4;
                 uint32_t __RESERVED0 : 7;
        } b;
        __IO uint32_t w;
    } GRXSTSR;                               /*!<  Receive Sts Q Read Register        01Ch*/
    union {
        struct {
            __IO uint32_t EPNUM : 4;                 /*!< IN EP interrupt mask bits */
            __IO uint32_t BCNT : 11;                 /*!< OUT EP interrupt mask bits */
            __IO uint32_t DPID : 2;                  /*!< OUT EP interrupt mask bits */
            __IO uint32_t PKTSTS : 4;                /*!< OUT EP interrupt mask bits */
                 uint32_t __RESERVED0 : 11;
        } b;
        __IO uint32_t w;
    } GRXSTSP;                               /*!<  Receive Sts Q Read & POP Register  020h*/
    union {
        struct {
            __IO uint32_t RXFD : 16;                 /*!< RxFIFO depth */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } GRXFSIZ;                               /* Receive FIFO Size Register         024h*/
    __IO uint32_t DIEPTXF0_HNPTXFSIZ;        /*!<  EP0 / Non Periodic Tx FIFO Size Register 028h*/
    __IO uint32_t HNPTXSTS;                  /*!<  Non Periodic Tx FIFO/Queue Sts reg 02Ch*/
         uint32_t __RESERVED0[2];               /* Reserved                           030h*/
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t PWRDWN : 1;                /*!< Power down */
            __IO uint32_t I2CPADEN : 1;              /*!< Enable I2C bus connection for the external I2C PHY interface */
            __IO uint32_t VBUSASEN : 1;              /*!< Enable the VBUS sensing device */
            __IO uint32_t VBUSBSEN : 1;              /*!< Enable the VBUS sensing device */
            __IO uint32_t SOFOUTEN : 1;              /*!< SOF output enable */
            __IO uint32_t NOVBUSSENS : 1;            /*!< VBUS sensing disable option */
                 uint32_t __RESERVED1 : 10;
        } b;
        __IO uint32_t w;
    } GCCFG;                                 /* General Purpose IO Register        038h*/
    __IO uint32_t CID;                       /* User ID Register                   03Ch*/
         uint32_t __RESERVED1[48];               /* Reserved                      040h-0FFh*/
    union {
        struct {
            __IO uint32_t PTXSA : 16;                /*!< Host periodic TxFIFO start address */
            __IO uint32_t PTXFD : 16;                /*!< Host periodic TxFIFO depth */
        } b;
        __IO uint32_t w;
    } HPTXFSIZ;                              /* Host Periodic Tx FIFO Size Reg     100h*/
    union {
        struct {
            __IO uint32_t INEPTXSA : 16;             /*!< IN endpoint FIFOx transmit RAM start address */
            __IO uint32_t INEPTXFD : 16;             /*!< IN endpoint TxFIFO depth */
        } b;
        __IO uint32_t w;
    } DIEPTXF;                               /* dev Periodic Transmit FIFO */
} USB_OTG_GlobalTypeDef;



/** 
  * @brief __device_Registers
  */

typedef struct {
    union {
        struct {
            __IO uint32_t DSPD : 2;                  /*!< Device speed */
            __IO uint32_t NZLSOHSK : 1;              /*!< Nonzero-length status OUT handshake */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DAD : 7;                   /*!< Device address */
            __IO uint32_t PFIVL : 2;                 /*!< Periodic (micro)frame interval */
                 uint32_t __RESERVED1 : 11;
            __IO uint32_t PERSCHIVL : 2;             /*!< Periodic scheduling interval */
                 uint32_t __RESERVED2 : 6;
        } b;
        __IO uint32_t w;
    } DCFG;                                  /* dev Configuration Register   800h*/
    union {
        struct {
            __IO uint32_t RWUSIG : 1;                /*!< Remote wakeup signaling */
            __IO uint32_t SDIS : 1;                  /*!< Soft disconnect */
            __IO uint32_t GINSTS : 1;                /*!< Global IN NAK status */
            __IO uint32_t GONSTS : 1;                /*!< Global OUT NAK status */
            __IO uint32_t TCTL : 3;                  /*!< Test control */
            __IO uint32_t SGINAK : 1;                /*!< Set global IN NAK */
            __IO uint32_t CGINAK : 1;                /*!< Clear global IN NAK */
            __IO uint32_t SGONAK : 1;                /*!< Set global OUT NAK */
            __IO uint32_t CGONAK : 1;                /*!< Clear global OUT NAK */
            __IO uint32_t POPRGDNE : 1;              /*!< Power-on programming done */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DCTL;                                  /* dev Control Register         804h*/
    union {
        struct {
            __IO uint32_t SUSPSTS : 1;               /*!< Suspend status */
            __IO uint32_t ENUMSPD : 2;               /*!< Enumerated speed */
            __IO uint32_t EERR : 1;                  /*!< Erratic error */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t FNSOF : 14;                /*!< Frame number of the received SOF */
                 uint32_t __RESERVED1 : 10;
        } b;
        __IO uint32_t w;
    } DSTS;                                  /* dev Status Register (RO)     808h*/
         uint32_t __RESERVED0;               /* Reserved                     80Ch*/
    union {
        struct {
            __IO uint32_t XFRCM : 1;                 /*!< Transfer completed interrupt mask */
            __IO uint32_t EPDM : 1;                  /*!< Endpoint disabled interrupt mask */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TOM : 1;                   /*!< Timeout condition mask (nonisochronous endpoints) */
            __IO uint32_t ITTXFEMSK : 1;             /*!< IN token received when TxFIFO empty mask */
            __IO uint32_t INEPNMM : 1;               /*!< IN token received with EP mismatch mask */
            __IO uint32_t INEPNEM : 1;               /*!< IN endpoint NAK effective mask */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t TXFURM : 1;                /*!< FIFO underrun mask */
            __IO uint32_t BIM : 1;                   /*!< BNA interrupt mask */
                 uint32_t __RESERVED2 : 22;
        } b;
        __IO uint32_t w;
    } DIEPMSK;                               /* dev IN Endpoint Mask         810h*/
    union {
        struct {
            __IO uint32_t XFRCM : 1;                 /*!< Transfer completed interrupt mask */
            __IO uint32_t EPDM : 1;                  /*!< Endpoint disabled interrupt mask */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t STUPM : 1;                 /*!< SETUP phase done mask */
            __IO uint32_t OTEPDM : 1;                /*!< OUT token received when endpoint disabled mask */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t B2BSTUP : 1;               /*!< Back-to-back SETUP packets received mask */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t OPEM : 1;                  /*!< OUT packet error mask */
            __IO uint32_t BOIM : 1;                  /*!< BNA interrupt mask */
                 uint32_t __RESERVED3 : 22;
        } b;
        __IO uint32_t w;
    } DOEPMSK;                               /* dev OUT Endpoint Mask        814h*/
    union {
        struct {
            __IO uint32_t IEPINT : 16;               /*!< IN endpoint interrupt bits */
            __IO uint32_t OEPINT : 16;               /*!< OUT endpoint interrupt bits */
        } b;
        __IO uint32_t w;
    } DAINT;                                 /* dev All Endpoints Itr Reg    818h*/
    union {
        struct {
            __IO uint32_t IEPM : 16;                 /*!< IN EP interrupt mask bits */
            __IO uint32_t OEPM : 16;                 /*!< OUT EP interrupt mask bits */
        } b;
        __IO uint32_t w;
    } DAINTMSK;                              /* dev All Endpoints Itr Mask   81Ch*/
         uint32_t __RESERVED1;               /* Reserved                     820h*/
         uint32_t __RESERVED2;               /* Reserved                     824h*/
    union {
        struct {
            __IO uint32_t VBUSDT : 16;               /*!< Device VBUS discharge time */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } DVBUSDIS;                              /* dev VBUS discharge Register  828h*/
    union {
        struct {
            __IO uint32_t DVBUSP : 12;               /*!< Device VBUS pulsing time */
                 uint32_t __RESERVED0 : 20;
        } b;
        __IO uint32_t w;
    } DVBUSPULSE;                            /* dev VBUS Pulse Register      82Ch*/
    union {
        struct {
            __IO uint32_t NONISOTHREN : 1;           /*!< Nonisochronous IN endpoints threshold enable */
            __IO uint32_t ISOTHREN : 1;              /*!< ISO IN endpoint threshold enable */
            __IO uint32_t TXTHRLEN : 9;              /*!< Transmit threshold length */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t RXTHREN : 1;               /*!< Receive threshold enable */
            __IO uint32_t RXTHRLEN : 9;              /*!< Receive threshold length */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t ARPEN : 1;                 /*!< Arbiter parking enable */
                 uint32_t __RESERVED2 : 4;
        } b;
        __IO uint32_t w;
    } DTHRCTL;                               /* dev thr                      830h*/
    union {
        struct {
            __IO uint32_t INEPTXFEM : 16;            /*!< IN EP Tx FIFO empty interrupt mask bits */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } DIEPEMPMSK;                            /* dev empty msk             834h*/
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t IEP1INT : 1;               /*!< IN endpoint 1interrupt bit */
                 uint32_t __RESERVED1 : 15;
            __IO uint32_t OEP1INT : 1;               /*!< OUT endpoint 1 interrupt bit */
                 uint32_t __RESERVED2 : 14;
        } b;
        __IO uint32_t w;
    } DEACHINT;                              /* dedicated EP interrupt       838h*/
    __IO uint32_t DEACHMSK;                  /* dedicated EP msk             83Ch*/
         uint32_t __RESERVED3;               /* dedicated EP mask           840h*/
    __IO uint32_t DINEP1MSK;                 /* dedicated EP mask           844h*/
         uint32_t __RESERVED4[15];               /* Reserved                 844-87Ch*/
    __IO uint32_t DOUTEP1MSK;                /* dedicated EP msk            884h*/
} USB_OTG_DeviceTypeDef;



/** 
  * @brief __IN_Endpoint-Specific_Register
  */

typedef struct {
    union {
        struct {
            __IO uint32_t MPSIZ : 11;                /*!< Maximum packet size */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t USBAEP : 1;                /*!< USB active endpoint */
            __IO uint32_t EONUM_DPID : 1;            /*!< Even/odd frame */
            __IO uint32_t NAKSTS : 1;                /*!< NAK status */
            __IO uint32_t EPTYP : 2;                 /*!< Endpoint type */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t STALL : 1;                 /*!< STALL handshake */
            __IO uint32_t TXFNUM : 4;                /*!< TxFIFO number */
            __IO uint32_t CNAK : 1;                  /*!< Clear NAK */
            __IO uint32_t SNAK : 1;                  /*!< Set NAK */
            __IO uint32_t SD0PID_SEVNFRM : 1;        /*!< Set DATA0 PID */
            __IO uint32_t SODDFRM : 1;               /*!< Set odd frame */
            __IO uint32_t EPDIS : 1;                 /*!< Endpoint disable */
            __IO uint32_t EPENA : 1;                 /*!< Endpoint enable */
        } b;
        __IO uint32_t w;
    } DIEPCTL;                               /* dev IN Endpoint Control Reg 900h + (ep_num * 20h) + 00h*/
         uint32_t __RESERVED0;               /* Reserved                       900h + (ep_num * 20h) + 04h*/
    union {
        struct {
            __IO uint32_t XFRC : 1;                  /*!< Transfer completed interrupt */
            __IO uint32_t EPDISD : 1;                /*!< Endpoint disabled interrupt */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TOC : 1;                   /*!< Timeout condition */
            __IO uint32_t ITTXFE : 1;                /*!< IN token received when TxFIFO is empty */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t INEPNE : 1;                /*!< IN endpoint NAK effective */
            __IO uint32_t TXFE : 1;                  /*!< Transmit FIFO empty */
            __IO uint32_t TXFIFOUDRN : 1;            /*!< Transmit Fifo Underrun */
            __IO uint32_t BNA : 1;                   /*!< Buffer not available interrupt */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t PKTDRPSTS : 1;             /*!< Packet dropped status */
            __IO uint32_t BERR : 1;                  /*!< Babble error interrupt */
            __IO uint32_t NAK : 1;                   /*!< NAK interrupt */
                 uint32_t __RESERVED3 : 18;
        } b;
        __IO uint32_t w;
    } DIEPINT;                               /* dev IN Endpoint Itr Reg     900h + (ep_num * 20h) + 08h*/
         uint32_t __RESERVED1;               /* Reserved                       900h + (ep_num * 20h) + 0Ch*/
    union {
        struct {
            __IO uint32_t XFRSIZ : 19;               /*!< Transfer size */
            __IO uint32_t PKTCNT : 10;               /*!< Packet count */
            __IO uint32_t MULCNT : 2;                /*!< Packet count */
                 uint32_t __RESERVED0 : 1;
        } b;
        __IO uint32_t w;
    } DIEPTSIZ;                              /* IN Endpoint Txfer Size   900h + (ep_num * 20h) + 10h*/
    __IO uint32_t DIEPDMA;                   /* IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h*/
    union {
        struct {
            __IO uint32_t INEPTFSAV : 16;            /*!< IN endpoint TxFIFO space avail */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } DTXFSTS;                               /*IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h*/
         uint32_t __RESERVED2;               /* Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch*/
} USB_OTG_INEndpointTypeDef;



/** 
  * @brief __OUT_Endpoint-Specific_Registers
  */

typedef struct {
    union {
        struct {
            __IO uint32_t MPSIZ : 11;                /*!< Maximum packet size */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t USBAEP : 1;                /*!< USB active endpoint */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t NAKSTS : 1;                /*!< NAK status */
            __IO uint32_t EPTYP : 2;                 /*!< Endpoint type */
            __IO uint32_t SNPM : 1;                  /*!< Snoop mode */
            __IO uint32_t STALL : 1;                 /*!< STALL handshake */
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t CNAK : 1;                  /*!< Clear NAK */
            __IO uint32_t SNAK : 1;                  /*!< Set NAK */
            __IO uint32_t SD0PID_SEVNFRM : 1;        /*!< Set DATA0 PID */
            __IO uint32_t SODDFRM : 1;               /*!< Set odd frame */
            __IO uint32_t EPDIS : 1;                 /*!< Endpoint disable */
            __IO uint32_t EPENA : 1;                 /*!< Endpoint enable */
        } b;
        __IO uint32_t w;
    } DOEPCTL;                               /* dev OUT Endpoint Control Reg  B00h + (ep_num * 20h) + 00h*/
         uint32_t __RESERVED0;               /* Reserved                      B00h + (ep_num * 20h) + 04h*/
    union {
        struct {
            __IO uint32_t XFRC : 1;                  /*!< Transfer completed interrupt */
            __IO uint32_t EPDISD : 1;                /*!< Endpoint disabled interrupt */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t STUP : 1;                  /*!< SETUP phase done */
            __IO uint32_t OTEPDIS : 1;               /*!< OUT token received when endpoint disabled */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t B2BSTUP : 1;               /*!< Back-to-back SETUP packets received */
                 uint32_t __RESERVED2 : 7;
            __IO uint32_t NYET : 1;                  /*!< NYET interrupt */
                 uint32_t __RESERVED3 : 17;
        } b;
        __IO uint32_t w;
    } DOEPINT;                               /* dev OUT Endpoint Itr Reg      B00h + (ep_num * 20h) + 08h*/
         uint32_t __RESERVED1;               /* Reserved                      B00h + (ep_num * 20h) + 0Ch*/
    union {
        struct {
            __IO uint32_t XFRSIZ : 19;               /*!< Transfer size */
            __IO uint32_t PKTCNT : 10;               /*!< Packet count */
            __IO uint32_t STUPCNT : 2;               /*!< SETUP packet count */
                 uint32_t __RESERVED0 : 1;
        } b;
        __IO uint32_t w;
    } DOEPTSIZ;                              /* dev OUT Endpoint Txfer Size   B00h + (ep_num * 20h) + 10h*/
    __IO uint32_t DOEPDMA;                   /* dev OUT Endpoint DMA Address  B00h + (ep_num * 20h) + 14h*/
         uint32_t __RESERVED2[2];               /* Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch*/
} USB_OTG_OUTEndpointTypeDef;



/** 
  * @brief __Host_Mode_Register_Structures
  */

typedef struct {
    union {
        struct {
            __IO uint32_t FSLSPCS : 2;               /*!< FS/LS PHY clock select */
            __IO uint32_t FSLSS : 1;                 /*!< FS- and LS-only support */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } HCFG;                                  /* Host Configuration Register    400h*/
    union {
        struct {
            __IO uint32_t FRIVL : 16;                /*!< Frame interval */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } HFIR;                                  /* Host Frame Interval Register   404h*/
    union {
        struct {
            __IO uint32_t FRNUM : 16;                /*!< Frame number */
            __IO uint32_t FTREM : 16;                /*!< Frame time remaining */
        } b;
        __IO uint32_t w;
    } HFNUM;                                 /* Host Frame Nbr/Frame Remaining 408h*/
         uint32_t __RESERVED0;               /* Reserved                       40Ch*/
    union {
        struct {
            __IO uint32_t PTXFSAVL : 16;             /*!< Periodic transmit data FIFO space available */
            __IO uint32_t PTXQSAV : 8;               /*!< Periodic transmit request queue space available */
            __IO uint32_t PTXQTOP : 8;               /*!< Top of the periodic transmit request queue */
        } b;
        __IO uint32_t w;
    } HPTXSTS;                               /* Host Periodic Tx FIFO/ Queue Status 410h*/
    union {
        struct {
            __IO uint32_t HAINT : 16;                /*!< Channel interrupts */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } HAINT;                                 /* Host All Channels Interrupt Register 414h*/
    union {
        struct {
            __IO uint32_t HAINTM : 16;               /*!< Channel interrupt mask */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } HAINTMSK;                              /* Host All Channels Interrupt Mask 418h*/
} USB_OTG_HostTypeDef;



/** 
  * @brief __Host_Channel_Specific_Registers
  */

typedef struct {
    union {
        struct {
            __IO uint32_t MPSIZ : 11;                /*!< Maximum packet size */
            __IO uint32_t EPNUM : 4;                 /*!< Endpoint number */
            __IO uint32_t EPDIR : 1;                 /*!< Endpoint direction */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t LSDEV : 1;                 /*!< Low-speed device */
            __IO uint32_t EPTYP : 2;                 /*!< Endpoint type */
            __IO uint32_t MC : 2;                    /*!< Multi Count (MC) / Error Count (EC) */
            __IO uint32_t DAD : 7;                   /*!< Device address */
            __IO uint32_t ODDFRM : 1;                /*!< Odd frame */
            __IO uint32_t CHDIS : 1;                 /*!< Channel disable */
            __IO uint32_t CHENA : 1;                 /*!< Channel enable */
        } b;
        __IO uint32_t w;
    } HCCHAR;
    union {
        struct {
            __IO uint32_t PRTADDR : 7;               /*!< Port address */
            __IO uint32_t HUBADDR : 7;               /*!< Hub address */
            __IO uint32_t XACTPOS : 2;               /*!< XACTPOS */
            __IO uint32_t COMPLSPLT : 1;             /*!< Do complete split */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t SPLITEN : 1;               /*!< Split enable */
        } b;
        __IO uint32_t w;
    } HCSPLT;
    union {
        struct {
            __IO uint32_t XFRC : 1;                  /*!< Transfer completed */
            __IO uint32_t CHH : 1;                   /*!< Channel halted */
            __IO uint32_t AHBERR : 1;                /*!< AHB error */
            __IO uint32_t STALL : 1;                 /*!< STALL response received interrupt */
            __IO uint32_t NAK : 1;                   /*!< NAK response received interrupt */
            __IO uint32_t ACK : 1;                   /*!< ACK response received/transmitted interrupt */
            __IO uint32_t NYET : 1;                  /*!< Response received interrupt */
            __IO uint32_t TXERR : 1;                 /*!< Transaction error */
            __IO uint32_t BBERR : 1;                 /*!< Babble error */
            __IO uint32_t FRMOR : 1;                 /*!< Frame overrun */
            __IO uint32_t DTERR : 1;                 /*!< Data toggle error */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } HCINT;
    union {
        struct {
            __IO uint32_t XFRCM : 1;                 /*!< Transfer completed mask */
            __IO uint32_t CHHM : 1;                  /*!< Channel halted mask */
            __IO uint32_t AHBERR : 1;                /*!< AHB error */
            __IO uint32_t STALLM : 1;                /*!< STALL response received interrupt mask */
            __IO uint32_t NAKM : 1;                  /*!< NAK response received interrupt mask */
            __IO uint32_t ACKM : 1;                  /*!< ACK response received/transmitted interrupt mask */
            __IO uint32_t NYET : 1;                  /*!< response received interrupt mask */
            __IO uint32_t TXERRM : 1;                /*!< Transaction error mask */
            __IO uint32_t BBERRM : 1;                /*!< Babble error mask */
            __IO uint32_t FRMORM : 1;                /*!< Frame overrun mask */
            __IO uint32_t DTERRM : 1;                /*!< Data toggle error mask */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } HCINTMSK;
    union {
        struct {
            __IO uint32_t XFRSIZ : 19;               /*!< Transfer size */
            __IO uint32_t PKTCNT : 10;               /*!< Packet count */
            __IO uint32_t DPID : 2;                  /*!< Data PID */
            __IO uint32_t DOPING : 1;                /*!< Do PING */
        } b;
        __IO uint32_t w;
    } HCTSIZ;
    __IO uint32_t HCDMA;
         uint32_t __RESERVED0[2];
} USB_OTG_HostChannelTypeDef;


    
/** 
  * @brief Peripheral_memory_map
  */
#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       ((uint32_t)0x10000000) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            ((uint32_t)0x20000000) /*!< SRAM1(112 KB) base address in the alias region                             */
#define SRAM2_BASE            ((uint32_t)0x2001C000) /*!< SRAM2(16 KB) base address in the alias region                              */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          ((uint32_t)0x40024000) /*!< Backup SRAM(4 KB) base address in the alias region                         */
#define FSMC_R_BASE           ((uint32_t)0xA0000000) /*!< FSMC registers base address                                                */
#define SRAM1_BB_BASE         ((uint32_t)0x22000000) /*!< SRAM1(112 KB) base address in the bit-band region                             */
#define SRAM2_BB_BASE         ((uint32_t)0x22380000) /*!< SRAM2(16 KB) base address in the bit-band region                              */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region                                */
#define BKPSRAM_BB_BASE       ((uint32_t)0x42480000) /*!< Backup SRAM(4 KB) base address in the bit-band region                         */
#define FLASH_END             ((uint32_t)0x080FFFFF) /*!< FLASH end address */
#define CCMDATARAM_END        ((uint32_t)0x1000FFFF) /*!< CCM data RAM end address */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE


/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2100)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x2200)
#define ADC_BASE              (APB2PERIPH_BASE + 0x2300)
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8)
#define ETH_BASE              (AHB1PERIPH_BASE + 0x8000)
#define ETH_MAC_BASE          (ETH_BASE)
#define ETH_MMC_BASE          (ETH_BASE + 0x0100)
#define ETH_PTP_BASE          (ETH_BASE + 0x0700)
#define ETH_DMA_BASE          (ETH_BASE + 0x1000)

/*!< AHB2 peripherals */
#define DCMI_BASE             (AHB2PERIPH_BASE + 0x50000)
#define RNG_BASE              (AHB2PERIPH_BASE + 0x60800)

/*!< FSMC Bankx registers base address */
#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000)
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104)
#define FSMC_Bank2_3_R_BASE   (FSMC_R_BASE + 0x0060)
#define FSMC_Bank4_R_BASE     (FSMC_R_BASE + 0x00A0)

/* Debug MCU registers base address */
#define DBGMCU_BASE           ((uint32_t )0xE0042000)

/*!< USB registers base address */
#define USB_OTG_HS_PERIPH_BASE               ((uint32_t )0x40040000)
#define USB_OTG_FS_PERIPH_BASE               ((uint32_t )0x50000000)

#define USB_OTG_GLOBAL_BASE                  ((uint32_t )0x000)
#define USB_OTG_DEVICE_BASE                  ((uint32_t )0x800)
#define USB_OTG_IN_ENDPOINT_BASE             ((uint32_t )0x900)
#define USB_OTG_OUT_ENDPOINT_BASE            ((uint32_t )0xB00)
#define USB_OTG_EP_REG_SIZE                  ((uint32_t )0x20)
#define USB_OTG_HOST_BASE                    ((uint32_t )0x400)
#define USB_OTG_HOST_PORT_BASE               ((uint32_t )0x440)
#define USB_OTG_HOST_CHANNEL_BASE            ((uint32_t )0x500)
#define USB_OTG_HOST_CHANNEL_SIZE            ((uint32_t )0x20)
#define USB_OTG_PCGCCTL_BASE                 ((uint32_t )0xE00)
#define USB_OTG_FIFO_BASE                    ((uint32_t )0x1000)
#define USB_OTG_FIFO_SIZE                    ((uint32_t )0x1000)

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
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                ((CAN_TypeDef *) CAN2_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC                 ((ADC_Common_TypeDef *) ADC_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE) 
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
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
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
#define ETH                 ((ETH_TypeDef *) ETH_BASE)  
#define DCMI                ((DCMI_TypeDef *) DCMI_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)
#define FSMC_Bank1          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2_3        ((FSMC_Bank2_3_TypeDef *) FSMC_Bank2_3_R_BASE)
#define FSMC_Bank4          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
#define USB_OTG_HS          ((USB_OTG_GlobalTypeDef *) USB_OTG_HS_PERIPH_BASE)

#define ADC_COMMON_BB             ((ADC_Common_BitBand_TypeDef *) PERIPH_BB(ADC_BASE))
#define ADC_BB(inst)              ((ADC_BitBand_TypeDef *) PERIPH_BB(inst))
#define CAN_BB(inst)              ((CAN_BitBand_TypeDef *) PERIPH_BB(inst))
#define CRC_BB                    ((CRC_BitBand_TypeDef *) PERIPH_BB(CRC_BASE))
#define DAC_BB                    ((DAC_BitBand_TypeDef *) PERIPH_BB(DAC_BASE))
#define DMA_Stream_BB(inst)       ((DMA_Stream_BitBand_TypeDef *) PERIPH_BB(inst))
#define DMA_BB(inst)              ((DMA_BitBand_TypeDef *) PERIPH_BB(inst))
#define ETH_BB                    ((ETH_BitBand_TypeDef *) PERIPH_BB(ETH_BASE))
#define EXTI_BB                   ((EXTI_BitBand_TypeDef *) PERIPH_BB(EXTI_BASE))
#define FLASH_BB                  ((FLASH_BitBand_TypeDef *) PERIPH_BB(FLASH_R_BASE))
#define GPIO_BB(inst)             ((GPIO_BitBand_TypeDef *) PERIPH_BB(inst))
#define I2C_BB(inst)              ((I2C_BitBand_TypeDef *) PERIPH_BB(inst))
#define IWDG_BB                   ((IWDG_BitBand_TypeDef *) PERIPH_BB(IWDG_BASE))
#define PWR_BB                    ((PWR_BitBand_TypeDef *) PERIPH_BB(PWR_BASE))
#define RCC_BB                    ((RCC_BitBand_TypeDef *) PERIPH_BB(RCC_BASE))
#define RTC_BB                    ((RTC_BitBand_TypeDef *) PERIPH_BB(RTC_BASE))
#define SDIO_BB                   ((SDIO_BitBand_TypeDef *) PERIPH_BB(SDIO_BASE))
#define SPI_BB(inst)              ((SPI_BitBand_TypeDef *) PERIPH_BB(inst))
#define SYSCFG_BB                 ((SYSCFG_BitBand_TypeDef *) PERIPH_BB(SYSCFG_BASE))
#define TIM_BB(inst)              ((TIM_BitBand_TypeDef *) PERIPH_BB(inst))
#define USART_BB(inst)            ((USART_BitBand_TypeDef *) PERIPH_BB(inst))
#define WWDG_BB                   ((WWDG_BitBand_TypeDef *) PERIPH_BB(WWDG_BASE))
/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */
  
  /** @addtogroup Peripheral_Registers_Bits_Definition
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
#define  ADC_SR_AWD                          ((uint32_t)0x00000001)       /*!<Analog watchdog flag */
#define  ADC_SR_EOC                          ((uint32_t)0x00000002)       /*!<End of conversion */
#define  ADC_SR_JEOC                         ((uint32_t)0x00000004)       /*!<Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((uint32_t)0x00000008)       /*!<Injected channel Start flag */
#define  ADC_SR_STRT                         ((uint32_t)0x00000010)       /*!<Regular channel Start flag */
#define  ADC_SR_OVR                          ((uint32_t)0x00000020)       /*!<Overrun flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((uint32_t)0x0000001F)        /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  ADC_CR1_EOCIE                       ((uint32_t)0x00000020)        /*!<Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((uint32_t)0x00000040)        /*!<AAnalog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((uint32_t)0x00000080)        /*!<Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((uint32_t)0x00000100)        /*!<Scan mode */
#define  ADC_CR1_AWDSGL                      ((uint32_t)0x00000200)        /*!<Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((uint32_t)0x00000400)        /*!<Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((uint32_t)0x00000800)        /*!<Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((uint32_t)0x00001000)        /*!<Discontinuous mode on injected channels */
#define  ADC_CR1_DISCNUM                     ((uint32_t)0x0000E000)        /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((uint32_t)0x00002000)        /*!<Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((uint32_t)0x00004000)        /*!<Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((uint32_t)0x00008000)        /*!<Bit 2 */
#define  ADC_CR1_JAWDEN                      ((uint32_t)0x00400000)        /*!<Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((uint32_t)0x00800000)        /*!<Analog watchdog enable on regular channels */
#define  ADC_CR1_RES                         ((uint32_t)0x03000000)        /*!<RES[2:0] bits (Resolution) */
#define  ADC_CR1_RES_0                       ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  ADC_CR1_RES_1                       ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  ADC_CR1_OVRIE                       ((uint32_t)0x04000000)         /*!<overrun interrupt enable */
  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((uint32_t)0x00000001)        /*!<A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((uint32_t)0x00000002)        /*!<Continuous Conversion */
#define  ADC_CR2_DMA                         ((uint32_t)0x00000100)        /*!<Direct Memory access mode */
#define  ADC_CR2_DDS                         ((uint32_t)0x00000200)        /*!<DMA disable selection (Single ADC) */
#define  ADC_CR2_EOCS                        ((uint32_t)0x00000400)        /*!<End of conversion selection */
#define  ADC_CR2_ALIGN                       ((uint32_t)0x00000800)        /*!<Data Alignment */
#define  ADC_CR2_JEXTSEL                     ((uint32_t)0x000F0000)        /*!<JEXTSEL[3:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  ADC_CR2_JEXTSEL_3                   ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  ADC_CR2_JEXTEN                      ((uint32_t)0x00300000)        /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
#define  ADC_CR2_JEXTEN_0                    ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  ADC_CR2_JEXTEN_1                    ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  ADC_CR2_JSWSTART                    ((uint32_t)0x00400000)        /*!<Start Conversion of injected channels */
#define  ADC_CR2_EXTSEL                      ((uint32_t)0x0F000000)        /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  ADC_CR2_EXTSEL_3                    ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  ADC_CR2_EXTEN                       ((uint32_t)0x30000000)        /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
#define  ADC_CR2_EXTEN_0                     ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  ADC_CR2_EXTEN_1                     ((uint32_t)0x20000000)        /*!<Bit 1 */
#define  ADC_CR2_SWSTART                     ((uint32_t)0x40000000)        /*!<Start Conversion of regular channels */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((uint32_t)0x00000007)        /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP11                     ((uint32_t)0x00000038)        /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((uint32_t)0x00000008)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((uint32_t)0x00000010)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((uint32_t)0x00000020)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP12                     ((uint32_t)0x000001C0)        /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((uint32_t)0x00000040)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((uint32_t)0x00000080)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((uint32_t)0x00000100)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP13                     ((uint32_t)0x00000E00)        /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((uint32_t)0x00000200)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((uint32_t)0x00000400)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((uint32_t)0x00000800)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP14                     ((uint32_t)0x00007000)        /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((uint32_t)0x00001000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((uint32_t)0x00002000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((uint32_t)0x00004000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP15                     ((uint32_t)0x00038000)        /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((uint32_t)0x00008000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((uint32_t)0x00010000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((uint32_t)0x00020000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP16                     ((uint32_t)0x001C0000)        /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((uint32_t)0x00040000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((uint32_t)0x00080000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((uint32_t)0x00100000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP17                     ((uint32_t)0x00E00000)        /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((uint32_t)0x00200000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((uint32_t)0x00400000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((uint32_t)0x00800000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP18                     ((uint32_t)0x07000000)        /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
#define  ADC_SMPR1_SMP18_0                   ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP18_1                   ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP18_2                   ((uint32_t)0x04000000)        /*!<Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((uint32_t)0x00000007)        /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP1                      ((uint32_t)0x00000038)        /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((uint32_t)0x00000008)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP1_1                    ((uint32_t)0x00000010)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP1_2                    ((uint32_t)0x00000020)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP2                      ((uint32_t)0x000001C0)        /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    ((uint32_t)0x00000040)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP2_1                    ((uint32_t)0x00000080)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP2_2                    ((uint32_t)0x00000100)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP3                      ((uint32_t)0x00000E00)        /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    ((uint32_t)0x00000200)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP3_1                    ((uint32_t)0x00000400)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP3_2                    ((uint32_t)0x00000800)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP4                      ((uint32_t)0x00007000)        /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    ((uint32_t)0x00001000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP4_1                    ((uint32_t)0x00002000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP4_2                    ((uint32_t)0x00004000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP5                      ((uint32_t)0x00038000)        /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    ((uint32_t)0x00008000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP5_1                    ((uint32_t)0x00010000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP5_2                    ((uint32_t)0x00020000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP6                      ((uint32_t)0x001C0000)        /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    ((uint32_t)0x00040000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP6_1                    ((uint32_t)0x00080000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP6_2                    ((uint32_t)0x00100000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP7                      ((uint32_t)0x00E00000)        /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    ((uint32_t)0x00200000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP7_1                    ((uint32_t)0x00400000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP7_2                    ((uint32_t)0x00800000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP8                      ((uint32_t)0x07000000)        /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP8_1                    ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP8_2                    ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP9                      ((uint32_t)0x38000000)        /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    ((uint32_t)0x08000000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP9_1                    ((uint32_t)0x10000000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP9_2                    ((uint32_t)0x20000000)        /*!<Bit 2 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((uint32_t)0x0FFF)            /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((uint32_t)0x0FFF)            /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((uint32_t)0x0FFF)            /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((uint32_t)0x0FFF)            /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((uint32_t)0x0FFF)            /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((uint32_t)0x0FFF)            /*!<Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       ((uint32_t)0x0000001F)        /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_SQR1_SQ13_1                     ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_SQR1_SQ13_2                     ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_SQR1_SQ13_3                     ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  ADC_SQR1_SQ13_4                     ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  ADC_SQR1_SQ14                       ((uint32_t)0x000003E0)        /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     ((uint32_t)0x00000020)        /*!<Bit 0 */
#define  ADC_SQR1_SQ14_1                     ((uint32_t)0x00000040)        /*!<Bit 1 */
#define  ADC_SQR1_SQ14_2                     ((uint32_t)0x00000080)        /*!<Bit 2 */
#define  ADC_SQR1_SQ14_3                     ((uint32_t)0x00000100)        /*!<Bit 3 */
#define  ADC_SQR1_SQ14_4                     ((uint32_t)0x00000200)        /*!<Bit 4 */
#define  ADC_SQR1_SQ15                       ((uint32_t)0x00007C00)        /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     ((uint32_t)0x00000400)        /*!<Bit 0 */
#define  ADC_SQR1_SQ15_1                     ((uint32_t)0x00000800)        /*!<Bit 1 */
#define  ADC_SQR1_SQ15_2                     ((uint32_t)0x00001000)        /*!<Bit 2 */
#define  ADC_SQR1_SQ15_3                     ((uint32_t)0x00002000)        /*!<Bit 3 */
#define  ADC_SQR1_SQ15_4                     ((uint32_t)0x00004000)        /*!<Bit 4 */
#define  ADC_SQR1_SQ16                       ((uint32_t)0x000F8000)        /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     ((uint32_t)0x00008000)        /*!<Bit 0 */
#define  ADC_SQR1_SQ16_1                     ((uint32_t)0x00010000)        /*!<Bit 1 */
#define  ADC_SQR1_SQ16_2                     ((uint32_t)0x00020000)        /*!<Bit 2 */
#define  ADC_SQR1_SQ16_3                     ((uint32_t)0x00040000)        /*!<Bit 3 */
#define  ADC_SQR1_SQ16_4                     ((uint32_t)0x00080000)        /*!<Bit 4 */
#define  ADC_SQR1_L                          ((uint32_t)0x00F00000)        /*!<L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  ADC_SQR1_L_1                        ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  ADC_SQR1_L_2                        ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  ADC_SQR1_L_3                        ((uint32_t)0x00800000)        /*!<Bit 3 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        ((uint32_t)0x0000001F)        /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_SQR2_SQ7_1                      ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_SQR2_SQ7_2                      ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_SQR2_SQ7_3                      ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  ADC_SQR2_SQ7_4                      ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  ADC_SQR2_SQ8                        ((uint32_t)0x000003E0)        /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      ((uint32_t)0x00000020)        /*!<Bit 0 */
#define  ADC_SQR2_SQ8_1                      ((uint32_t)0x00000040)        /*!<Bit 1 */
#define  ADC_SQR2_SQ8_2                      ((uint32_t)0x00000080)        /*!<Bit 2 */
#define  ADC_SQR2_SQ8_3                      ((uint32_t)0x00000100)        /*!<Bit 3 */
#define  ADC_SQR2_SQ8_4                      ((uint32_t)0x00000200)        /*!<Bit 4 */
#define  ADC_SQR2_SQ9                        ((uint32_t)0x00007C00)        /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      ((uint32_t)0x00000400)        /*!<Bit 0 */
#define  ADC_SQR2_SQ9_1                      ((uint32_t)0x00000800)        /*!<Bit 1 */
#define  ADC_SQR2_SQ9_2                      ((uint32_t)0x00001000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ9_3                      ((uint32_t)0x00002000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ9_4                      ((uint32_t)0x00004000)        /*!<Bit 4 */
#define  ADC_SQR2_SQ10                       ((uint32_t)0x000F8000)        /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     ((uint32_t)0x00008000)        /*!<Bit 0 */
#define  ADC_SQR2_SQ10_1                     ((uint32_t)0x00010000)        /*!<Bit 1 */
#define  ADC_SQR2_SQ10_2                     ((uint32_t)0x00020000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ10_3                     ((uint32_t)0x00040000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ10_4                     ((uint32_t)0x00080000)        /*!<Bit 4 */
#define  ADC_SQR2_SQ11                       ((uint32_t)0x01F00000)        /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  ADC_SQR2_SQ11_1                     ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  ADC_SQR2_SQ11_2                     ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ11_3                     ((uint32_t)0x00800000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ11_4                     ((uint32_t)0x01000000)        /*!<Bit 4 */
#define  ADC_SQR2_SQ12                       ((uint32_t)0x3E000000)        /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     ((uint32_t)0x02000000)        /*!<Bit 0 */
#define  ADC_SQR2_SQ12_1                     ((uint32_t)0x04000000)        /*!<Bit 1 */
#define  ADC_SQR2_SQ12_2                     ((uint32_t)0x08000000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ12_3                     ((uint32_t)0x10000000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ12_4                     ((uint32_t)0x20000000)        /*!<Bit 4 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        ((uint32_t)0x0000001F)        /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_SQR3_SQ1_1                      ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_SQR3_SQ1_2                      ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_SQR3_SQ1_3                      ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  ADC_SQR3_SQ1_4                      ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  ADC_SQR3_SQ2                        ((uint32_t)0x000003E0)        /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      ((uint32_t)0x00000020)        /*!<Bit 0 */
#define  ADC_SQR3_SQ2_1                      ((uint32_t)0x00000040)        /*!<Bit 1 */
#define  ADC_SQR3_SQ2_2                      ((uint32_t)0x00000080)        /*!<Bit 2 */
#define  ADC_SQR3_SQ2_3                      ((uint32_t)0x00000100)        /*!<Bit 3 */
#define  ADC_SQR3_SQ2_4                      ((uint32_t)0x00000200)        /*!<Bit 4 */
#define  ADC_SQR3_SQ3                        ((uint32_t)0x00007C00)        /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      ((uint32_t)0x00000400)        /*!<Bit 0 */
#define  ADC_SQR3_SQ3_1                      ((uint32_t)0x00000800)        /*!<Bit 1 */
#define  ADC_SQR3_SQ3_2                      ((uint32_t)0x00001000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ3_3                      ((uint32_t)0x00002000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ3_4                      ((uint32_t)0x00004000)        /*!<Bit 4 */
#define  ADC_SQR3_SQ4                        ((uint32_t)0x000F8000)        /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      ((uint32_t)0x00008000)        /*!<Bit 0 */
#define  ADC_SQR3_SQ4_1                      ((uint32_t)0x00010000)        /*!<Bit 1 */
#define  ADC_SQR3_SQ4_2                      ((uint32_t)0x00020000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ4_3                      ((uint32_t)0x00040000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ4_4                      ((uint32_t)0x00080000)        /*!<Bit 4 */
#define  ADC_SQR3_SQ5                        ((uint32_t)0x01F00000)        /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  ADC_SQR3_SQ5_1                      ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  ADC_SQR3_SQ5_2                      ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ5_3                      ((uint32_t)0x00800000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ5_4                      ((uint32_t)0x01000000)        /*!<Bit 4 */
#define  ADC_SQR3_SQ6                        ((uint32_t)0x3E000000)        /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      ((uint32_t)0x02000000)        /*!<Bit 0 */
#define  ADC_SQR3_SQ6_1                      ((uint32_t)0x04000000)        /*!<Bit 1 */
#define  ADC_SQR3_SQ6_2                      ((uint32_t)0x08000000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ6_3                      ((uint32_t)0x10000000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ6_4                      ((uint32_t)0x20000000)        /*!<Bit 4 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       ((uint32_t)0x0000001F)        /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define  ADC_JSQR_JSQ1_0                     ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ1_1                     ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ1_2                     ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ1_3                     ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ1_4                     ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ2                       ((uint32_t)0x000003E0)        /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     ((uint32_t)0x00000020)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ2_1                     ((uint32_t)0x00000040)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ2_2                     ((uint32_t)0x00000080)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ2_3                     ((uint32_t)0x00000100)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ2_4                     ((uint32_t)0x00000200)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ3                       ((uint32_t)0x00007C00)        /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     ((uint32_t)0x00000400)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ3_1                     ((uint32_t)0x00000800)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ3_2                     ((uint32_t)0x00001000)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ3_3                     ((uint32_t)0x00002000)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ3_4                     ((uint32_t)0x00004000)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ4                       ((uint32_t)0x000F8000)        /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     ((uint32_t)0x00008000)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ4_1                     ((uint32_t)0x00010000)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ4_2                     ((uint32_t)0x00020000)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ4_3                     ((uint32_t)0x00040000)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ4_4                     ((uint32_t)0x00080000)        /*!<Bit 4 */
#define  ADC_JSQR_JL                         ((uint32_t)0x00300000)        /*!<JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  ADC_JSQR_JL_1                       ((uint32_t)0x00200000)        /*!<Bit 1 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((uint32_t)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((uint32_t)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((uint32_t)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((uint32_t)0xFFFF)            /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((uint32_t)0x0000FFFF)        /*!<Regular data */
#define  ADC_DR_ADC2DATA                     ((uint32_t)0xFFFF0000)        /*!<ADC2 data */

/*******************  Bit definition for ADC_CSR register  ********************/
#define  ADC_CSR_AWD1                        ((uint32_t)0x00000001)        /*!<ADC1 Analog watchdog flag */
#define  ADC_CSR_EOC1                        ((uint32_t)0x00000002)        /*!<ADC1 End of conversion */
#define  ADC_CSR_JEOC1                       ((uint32_t)0x00000004)        /*!<ADC1 Injected channel end of conversion */
#define  ADC_CSR_JSTRT1                      ((uint32_t)0x00000008)        /*!<ADC1 Injected channel Start flag */
#define  ADC_CSR_STRT1                       ((uint32_t)0x00000010)        /*!<ADC1 Regular channel Start flag */
#define  ADC_CSR_DOVR1                       ((uint32_t)0x00000020)        /*!<ADC1 DMA overrun  flag */
#define  ADC_CSR_AWD2                        ((uint32_t)0x00000100)        /*!<ADC2 Analog watchdog flag */
#define  ADC_CSR_EOC2                        ((uint32_t)0x00000200)        /*!<ADC2 End of conversion */
#define  ADC_CSR_JEOC2                       ((uint32_t)0x00000400)        /*!<ADC2 Injected channel end of conversion */
#define  ADC_CSR_JSTRT2                      ((uint32_t)0x00000800)        /*!<ADC2 Injected channel Start flag */
#define  ADC_CSR_STRT2                       ((uint32_t)0x00001000)        /*!<ADC2 Regular channel Start flag */
#define  ADC_CSR_DOVR2                       ((uint32_t)0x00002000)        /*!<ADC2 DMA overrun  flag */
#define  ADC_CSR_AWD3                        ((uint32_t)0x00010000)        /*!<ADC3 Analog watchdog flag */
#define  ADC_CSR_EOC3                        ((uint32_t)0x00020000)        /*!<ADC3 End of conversion */
#define  ADC_CSR_JEOC3                       ((uint32_t)0x00040000)        /*!<ADC3 Injected channel end of conversion */
#define  ADC_CSR_JSTRT3                      ((uint32_t)0x00080000)        /*!<ADC3 Injected channel Start flag */
#define  ADC_CSR_STRT3                       ((uint32_t)0x00100000)        /*!<ADC3 Regular channel Start flag */
#define  ADC_CSR_DOVR3                       ((uint32_t)0x00200000)        /*!<ADC3 DMA overrun  flag */

/*******************  Bit definition for ADC_CCR register  ********************/
#define  ADC_CCR_MULTI                       ((uint32_t)0x0000001F)        /*!<MULTI[4:0] bits (Multi-ADC mode selection) */  
#define  ADC_CCR_MULTI_0                     ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  ADC_CCR_MULTI_1                     ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  ADC_CCR_MULTI_2                     ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  ADC_CCR_MULTI_3                     ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  ADC_CCR_MULTI_4                     ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  ADC_CCR_DELAY                       ((uint32_t)0x00000F00)        /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */  
#define  ADC_CCR_DELAY_0                     ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  ADC_CCR_DELAY_1                     ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  ADC_CCR_DELAY_2                     ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  ADC_CCR_DELAY_3                     ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  ADC_CCR_DDS                         ((uint32_t)0x00002000)        /*!<DMA disable selection (Multi-ADC mode) */
#define  ADC_CCR_DMA                         ((uint32_t)0x0000C000)        /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */  
#define  ADC_CCR_DMA_0                       ((uint32_t)0x00004000)        /*!<Bit 0 */
#define  ADC_CCR_DMA_1                       ((uint32_t)0x00008000)        /*!<Bit 1 */
#define  ADC_CCR_ADCPRE                      ((uint32_t)0x00030000)        /*!<ADCPRE[1:0] bits (ADC prescaler) */  
#define  ADC_CCR_ADCPRE_0                    ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  ADC_CCR_ADCPRE_1                    ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  ADC_CCR_VBATE                       ((uint32_t)0x00400000)        /*!<VBAT Enable */
#define  ADC_CCR_TSVREFE                     ((uint32_t)0x00800000)        /*!<Temperature Sensor and VREFINT Enable */

/*******************  Bit definition for ADC_CDR register  ********************/
#define  ADC_CDR_DATA1                      ((uint32_t)0x0000FFFF)         /*!<1st data of a pair of regular conversions */
#define  ADC_CDR_DATA2                      ((uint32_t)0xFFFF0000)         /*!<2nd data of a pair of regular conversions */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/
/*!<CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define  CAN_MCR_INRQ                        ((uint32_t)0x00000001)        /*!<Initialization Request */
#define  CAN_MCR_SLEEP                       ((uint32_t)0x00000002)        /*!<Sleep Mode Request */
#define  CAN_MCR_TXFP                        ((uint32_t)0x00000004)        /*!<Transmit FIFO Priority */
#define  CAN_MCR_RFLM                        ((uint32_t)0x00000008)        /*!<Receive FIFO Locked Mode */
#define  CAN_MCR_NART                        ((uint32_t)0x00000010)        /*!<No Automatic Retransmission */
#define  CAN_MCR_AWUM                        ((uint32_t)0x00000020)        /*!<Automatic Wakeup Mode */
#define  CAN_MCR_ABOM                        ((uint32_t)0x00000040)        /*!<Automatic Bus-Off Management */
#define  CAN_MCR_TTCM                        ((uint32_t)0x00000080)        /*!<Time Triggered Communication Mode */
#define  CAN_MCR_RESET                       ((uint32_t)0x00008000)        /*!<bxCAN software master reset */
#define  CAN_MCR_DBF                         ((uint32_t)0x00010000)        /*!<bxCAN Debug freeze */
/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        ((uint32_t)0x0001)            /*!<Initialization Acknowledge */
#define  CAN_MSR_SLAK                        ((uint32_t)0x0002)            /*!<Sleep Acknowledge */
#define  CAN_MSR_ERRI                        ((uint32_t)0x0004)            /*!<Error Interrupt */
#define  CAN_MSR_WKUI                        ((uint32_t)0x0008)            /*!<Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       ((uint32_t)0x0010)            /*!<Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         ((uint32_t)0x0100)            /*!<Transmit Mode */
#define  CAN_MSR_RXM                         ((uint32_t)0x0200)            /*!<Receive Mode */
#define  CAN_MSR_SAMP                        ((uint32_t)0x0400)            /*!<Last Sample Point */
#define  CAN_MSR_RX                          ((uint32_t)0x0800)            /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define  CAN_TSR_RQCP0                       ((uint32_t)0x00000001)        /*!<Request Completed Mailbox0 */
#define  CAN_TSR_TXOK0                       ((uint32_t)0x00000002)        /*!<Transmission OK of Mailbox0 */
#define  CAN_TSR_ALST0                       ((uint32_t)0x00000004)        /*!<Arbitration Lost for Mailbox0 */
#define  CAN_TSR_TERR0                       ((uint32_t)0x00000008)        /*!<Transmission Error of Mailbox0 */
#define  CAN_TSR_ABRQ0                       ((uint32_t)0x00000080)        /*!<Abort Request for Mailbox0 */
#define  CAN_TSR_RQCP1                       ((uint32_t)0x00000100)        /*!<Request Completed Mailbox1 */
#define  CAN_TSR_TXOK1                       ((uint32_t)0x00000200)        /*!<Transmission OK of Mailbox1 */
#define  CAN_TSR_ALST1                       ((uint32_t)0x00000400)        /*!<Arbitration Lost for Mailbox1 */
#define  CAN_TSR_TERR1                       ((uint32_t)0x00000800)        /*!<Transmission Error of Mailbox1 */
#define  CAN_TSR_ABRQ1                       ((uint32_t)0x00008000)        /*!<Abort Request for Mailbox 1 */
#define  CAN_TSR_RQCP2                       ((uint32_t)0x00010000)        /*!<Request Completed Mailbox2 */
#define  CAN_TSR_TXOK2                       ((uint32_t)0x00020000)        /*!<Transmission OK of Mailbox 2 */
#define  CAN_TSR_ALST2                       ((uint32_t)0x00040000)        /*!<Arbitration Lost for mailbox 2 */
#define  CAN_TSR_TERR2                       ((uint32_t)0x00080000)        /*!<Transmission Error of Mailbox 2 */
#define  CAN_TSR_ABRQ2                       ((uint32_t)0x00800000)        /*!<Abort Request for Mailbox 2 */
#define  CAN_TSR_CODE                        ((uint32_t)0x03000000)        /*!<Mailbox Code */

#define  CAN_TSR_TME                         ((uint32_t)0x1C000000)        /*!<TME[2:0] bits */
#define  CAN_TSR_TME0                        ((uint32_t)0x04000000)        /*!<Transmit Mailbox 0 Empty */
#define  CAN_TSR_TME1                        ((uint32_t)0x08000000)        /*!<Transmit Mailbox 1 Empty */
#define  CAN_TSR_TME2                        ((uint32_t)0x10000000)        /*!<Transmit Mailbox 2 Empty */

#define  CAN_TSR_LOW                         ((uint32_t)0xE0000000)        /*!<LOW[2:0] bits */
#define  CAN_TSR_LOW0                        ((uint32_t)0x20000000)        /*!<Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSR_LOW1                        ((uint32_t)0x40000000)        /*!<Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSR_LOW2                        ((uint32_t)0x80000000)        /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define  CAN_RF0R_FMP0                       ((uint32_t)0x03)               /*!<FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      ((uint32_t)0x08)               /*!<FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      ((uint32_t)0x10)               /*!<FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      ((uint32_t)0x20)               /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       ((uint32_t)0x03)               /*!<FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      ((uint32_t)0x08)               /*!<FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      ((uint32_t)0x10)               /*!<FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      ((uint32_t)0x20)               /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define  CAN_IER_TMEIE                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Empty Interrupt Enable */
#define  CAN_IER_FMPIE0                      ((uint32_t)0x00000002)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE0                       ((uint32_t)0x00000004)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE0                      ((uint32_t)0x00000008)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_FMPIE1                      ((uint32_t)0x00000010)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE1                       ((uint32_t)0x00000020)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE1                      ((uint32_t)0x00000040)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_EWGIE                       ((uint32_t)0x00000100)        /*!<Error Warning Interrupt Enable */
#define  CAN_IER_EPVIE                       ((uint32_t)0x00000200)        /*!<Error Passive Interrupt Enable */
#define  CAN_IER_BOFIE                       ((uint32_t)0x00000400)        /*!<Bus-Off Interrupt Enable */
#define  CAN_IER_LECIE                       ((uint32_t)0x00000800)        /*!<Last Error Code Interrupt Enable */
#define  CAN_IER_ERRIE                       ((uint32_t)0x00008000)        /*!<Error Interrupt Enable */
#define  CAN_IER_WKUIE                       ((uint32_t)0x00010000)        /*!<Wakeup Interrupt Enable */
#define  CAN_IER_SLKIE                       ((uint32_t)0x00020000)        /*!<Sleep Interrupt Enable */
#define  CAN_IER_EWGIE                       ((uint32_t)0x00000100)        /*!<Error warning interrupt enable */
#define  CAN_IER_EPVIE                       ((uint32_t)0x00000200)        /*!<Error passive interrupt enable */
#define  CAN_IER_BOFIE                       ((uint32_t)0x00000400)        /*!<Bus-off interrupt enable */
#define  CAN_IER_LECIE                       ((uint32_t)0x00000800)        /*!<Last error code interrupt enable */
#define  CAN_IER_ERRIE                       ((uint32_t)0x00008000)        /*!<Error interrupt enable */


/********************  Bit definition for CAN_ESR register  *******************/
#define  CAN_ESR_EWGF                        ((uint32_t)0x00000001)        /*!<Error Warning Flag */
#define  CAN_ESR_EPVF                        ((uint32_t)0x00000002)        /*!<Error Passive Flag */
#define  CAN_ESR_BOFF                        ((uint32_t)0x00000004)        /*!<Bus-Off Flag */

#define  CAN_ESR_LEC                         ((uint32_t)0x00000070)        /*!<LEC[2:0] bits (Last Error Code) */
#define  CAN_ESR_LEC_0                       ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  CAN_ESR_LEC_1                       ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  CAN_ESR_LEC_2                       ((uint32_t)0x00000040)        /*!<Bit 2 */

#define  CAN_ESR_TEC                         ((uint32_t)0x00FF0000)        /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESR_REC                         ((uint32_t)0xFF000000)        /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define  CAN_BTR_BRP                         ((uint32_t)0x000003FF)        /*!<Baud Rate Prescaler */
#define  CAN_BTR_TS1                         ((uint32_t)0x000F0000)        /*!<Time Segment 1 */
#define  CAN_BTR_TS1_0                       ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  CAN_BTR_TS1_1                       ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  CAN_BTR_TS1_2                       ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  CAN_BTR_TS1_3                       ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  CAN_BTR_TS2                         ((uint32_t)0x00700000)        /*!<Time Segment 2 */
#define  CAN_BTR_TS2_0                       ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  CAN_BTR_TS2_1                       ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  CAN_BTR_TS2_2                       ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  CAN_BTR_SJW                         ((uint32_t)0x03000000)        /*!<Resynchronization Jump Width */
#define  CAN_BTR_SJW_0                       ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  CAN_BTR_SJW_1                       ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  CAN_BTR_LBKM                        ((uint32_t)0x40000000)        /*!<Loop Back Mode (Debug) */
#define  CAN_BTR_SILM                        ((uint32_t)0x80000000)        /*!<Silent Mode */


/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TI0R_TXRQ                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI0R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI0R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI0R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_TI0R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define  CAN_TDT0R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT0R_TGT                       ((uint32_t)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT0R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define  CAN_TDL0R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL0R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL0R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL0R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define  CAN_TDH0R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH0R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH0R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH0R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define  CAN_TI1R_TXRQ                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI1R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI1R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI1R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_TI1R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define  CAN_TDT1R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT1R_TGT                       ((uint32_t)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT1R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define  CAN_TDL1R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL1R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL1R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL1R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define  CAN_TDH1R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH1R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH1R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH1R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define  CAN_TI2R_TXRQ                       ((uint32_t)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI2R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI2R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI2R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended identifier */
#define  CAN_TI2R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/  
#define  CAN_TDT2R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT2R_TGT                       ((uint32_t)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT2R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define  CAN_TDL2R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL2R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL2R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL2R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define  CAN_TDH2R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH2R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH2R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH2R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define  CAN_RI0R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_RI0R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_RI0R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_RI0R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define  CAN_RDT0R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_RDT0R_FMI                       ((uint32_t)0x0000FF00)        /*!<Filter Match Index */
#define  CAN_RDT0R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define  CAN_RDL0R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_RDL0R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_RDL0R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_RDL0R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define  CAN_RDH0R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_RDH0R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_RDH0R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_RDH0R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define  CAN_RI1R_RTR                        ((uint32_t)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_RI1R_IDE                        ((uint32_t)0x00000004)        /*!<Identifier Extension */
#define  CAN_RI1R_EXID                       ((uint32_t)0x001FFFF8)        /*!<Extended identifier */
#define  CAN_RI1R_STID                       ((uint32_t)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define  CAN_RDT1R_DLC                       ((uint32_t)0x0000000F)        /*!<Data Length Code */
#define  CAN_RDT1R_FMI                       ((uint32_t)0x0000FF00)        /*!<Filter Match Index */
#define  CAN_RDT1R_TIME                      ((uint32_t)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define  CAN_RDL1R_DATA0                     ((uint32_t)0x000000FF)        /*!<Data byte 0 */
#define  CAN_RDL1R_DATA1                     ((uint32_t)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_RDL1R_DATA2                     ((uint32_t)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_RDL1R_DATA3                     ((uint32_t)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define  CAN_RDH1R_DATA4                     ((uint32_t)0x000000FF)        /*!<Data byte 4 */
#define  CAN_RDH1R_DATA5                     ((uint32_t)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_RDH1R_DATA6                     ((uint32_t)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_RDH1R_DATA7                     ((uint32_t)0xFF000000)        /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT                       ((uint32_t)0x01)               /*!<Filter Init Mode */
#define  CAN_FMR_CAN2SB                      ((uint32_t)0x00003F00)        /*!<CAN2 start bank */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        ((uint32_t)0x0FFFFFFF)        /*!<Filter Mode */
#define  CAN_FM1R_FBM0                       ((uint32_t)0x00000001)        /*!<Filter Init Mode bit 0 */
#define  CAN_FM1R_FBM1                       ((uint32_t)0x00000002)        /*!<Filter Init Mode bit 1 */
#define  CAN_FM1R_FBM2                       ((uint32_t)0x00000004)        /*!<Filter Init Mode bit 2 */
#define  CAN_FM1R_FBM3                       ((uint32_t)0x00000008)        /*!<Filter Init Mode bit 3 */
#define  CAN_FM1R_FBM4                       ((uint32_t)0x00000010)        /*!<Filter Init Mode bit 4 */
#define  CAN_FM1R_FBM5                       ((uint32_t)0x00000020)        /*!<Filter Init Mode bit 5 */
#define  CAN_FM1R_FBM6                       ((uint32_t)0x00000040)        /*!<Filter Init Mode bit 6 */
#define  CAN_FM1R_FBM7                       ((uint32_t)0x00000080)        /*!<Filter Init Mode bit 7 */
#define  CAN_FM1R_FBM8                       ((uint32_t)0x00000100)        /*!<Filter Init Mode bit 8 */
#define  CAN_FM1R_FBM9                       ((uint32_t)0x00000200)        /*!<Filter Init Mode bit 9 */
#define  CAN_FM1R_FBM10                      ((uint32_t)0x00000400)        /*!<Filter Init Mode bit 10 */
#define  CAN_FM1R_FBM11                      ((uint32_t)0x00000800)        /*!<Filter Init Mode bit 11 */
#define  CAN_FM1R_FBM12                      ((uint32_t)0x00001000)        /*!<Filter Init Mode bit 12 */
#define  CAN_FM1R_FBM13                      ((uint32_t)0x00002000)        /*!<Filter Init Mode bit 13 */
#define  CAN_FM1R_FBM14                      ((uint32_t)0x00004000)        /*!<Filter Init Mode bit 14 */
#define  CAN_FM1R_FBM15                      ((uint32_t)0x00008000)        /*!<Filter Init Mode bit 15 */
#define  CAN_FM1R_FBM16                      ((uint32_t)0x00010000)        /*!<Filter Init Mode bit 16 */
#define  CAN_FM1R_FBM17                      ((uint32_t)0x00020000)        /*!<Filter Init Mode bit 17 */
#define  CAN_FM1R_FBM18                      ((uint32_t)0x00040000)        /*!<Filter Init Mode bit 18 */
#define  CAN_FM1R_FBM19                      ((uint32_t)0x00080000)        /*!<Filter Init Mode bit 19 */
#define  CAN_FM1R_FBM20                      ((uint32_t)0x00100000)        /*!<Filter Init Mode bit 20 */
#define  CAN_FM1R_FBM21                      ((uint32_t)0x00200000)        /*!<Filter Init Mode bit 21 */
#define  CAN_FM1R_FBM22                      ((uint32_t)0x00400000)        /*!<Filter Init Mode bit 22 */
#define  CAN_FM1R_FBM23                      ((uint32_t)0x00800000)        /*!<Filter Init Mode bit 23 */
#define  CAN_FM1R_FBM24                      ((uint32_t)0x01000000)        /*!<Filter Init Mode bit 24 */
#define  CAN_FM1R_FBM25                      ((uint32_t)0x02000000)        /*!<Filter Init Mode bit 25 */
#define  CAN_FM1R_FBM26                      ((uint32_t)0x04000000)        /*!<Filter Init Mode bit 26 */
#define  CAN_FM1R_FBM27                      ((uint32_t)0x08000000)        /*!<Filter Init Mode bit 27 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        ((uint32_t)0x0FFFFFFF)        /*!<Filter Scale Configuration */
#define  CAN_FS1R_FSC0                       ((uint32_t)0x00000001)        /*!<Filter Scale Configuration bit 0 */
#define  CAN_FS1R_FSC1                       ((uint32_t)0x00000002)        /*!<Filter Scale Configuration bit 1 */
#define  CAN_FS1R_FSC2                       ((uint32_t)0x00000004)        /*!<Filter Scale Configuration bit 2 */
#define  CAN_FS1R_FSC3                       ((uint32_t)0x00000008)        /*!<Filter Scale Configuration bit 3 */
#define  CAN_FS1R_FSC4                       ((uint32_t)0x00000010)        /*!<Filter Scale Configuration bit 4 */
#define  CAN_FS1R_FSC5                       ((uint32_t)0x00000020)        /*!<Filter Scale Configuration bit 5 */
#define  CAN_FS1R_FSC6                       ((uint32_t)0x00000040)        /*!<Filter Scale Configuration bit 6 */
#define  CAN_FS1R_FSC7                       ((uint32_t)0x00000080)        /*!<Filter Scale Configuration bit 7 */
#define  CAN_FS1R_FSC8                       ((uint32_t)0x00000100)        /*!<Filter Scale Configuration bit 8 */
#define  CAN_FS1R_FSC9                       ((uint32_t)0x00000200)        /*!<Filter Scale Configuration bit 9 */
#define  CAN_FS1R_FSC10                      ((uint32_t)0x00000400)        /*!<Filter Scale Configuration bit 10 */
#define  CAN_FS1R_FSC11                      ((uint32_t)0x00000800)        /*!<Filter Scale Configuration bit 11 */
#define  CAN_FS1R_FSC12                      ((uint32_t)0x00001000)        /*!<Filter Scale Configuration bit 12 */
#define  CAN_FS1R_FSC13                      ((uint32_t)0x00002000)        /*!<Filter Scale Configuration bit 13 */
#define  CAN_FS1R_FSC14                      ((uint32_t)0x00004000)        /*!<Filter Scale Configuration bit 14 */
#define  CAN_FS1R_FSC15                      ((uint32_t)0x00008000)        /*!<Filter Scale Configuration bit 15 */
#define  CAN_FS1R_FSC16                      ((uint32_t)0x00010000)        /*!<Filter Scale Configuration bit 16 */
#define  CAN_FS1R_FSC17                      ((uint32_t)0x00020000)        /*!<Filter Scale Configuration bit 17 */
#define  CAN_FS1R_FSC18                      ((uint32_t)0x00040000)        /*!<Filter Scale Configuration bit 18 */
#define  CAN_FS1R_FSC19                      ((uint32_t)0x00080000)        /*!<Filter Scale Configuration bit 19 */
#define  CAN_FS1R_FSC20                      ((uint32_t)0x00100000)        /*!<Filter Scale Configuration bit 20 */
#define  CAN_FS1R_FSC21                      ((uint32_t)0x00200000)        /*!<Filter Scale Configuration bit 21 */
#define  CAN_FS1R_FSC22                      ((uint32_t)0x00400000)        /*!<Filter Scale Configuration bit 22 */
#define  CAN_FS1R_FSC23                      ((uint32_t)0x00800000)        /*!<Filter Scale Configuration bit 23 */
#define  CAN_FS1R_FSC24                      ((uint32_t)0x01000000)        /*!<Filter Scale Configuration bit 24 */
#define  CAN_FS1R_FSC25                      ((uint32_t)0x02000000)        /*!<Filter Scale Configuration bit 25 */
#define  CAN_FS1R_FSC26                      ((uint32_t)0x04000000)        /*!<Filter Scale Configuration bit 26 */
#define  CAN_FS1R_FSC27                      ((uint32_t)0x08000000)        /*!<Filter Scale Configuration bit 27 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                        ((uint32_t)0x0FFFFFFF)        /*!<Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                       ((uint32_t)0x00000001)        /*!<Filter FIFO Assignment bit 0 */
#define  CAN_FFA1R_FFA1                       ((uint32_t)0x00000002)        /*!<Filter FIFO Assignment bit 1 */
#define  CAN_FFA1R_FFA2                       ((uint32_t)0x00000004)        /*!<Filter FIFO Assignment bit 2 */
#define  CAN_FFA1R_FFA3                       ((uint32_t)0x00000008)        /*!<Filter FIFO Assignment bit 3 */
#define  CAN_FFA1R_FFA4                       ((uint32_t)0x00000010)        /*!<Filter FIFO Assignment bit 4 */
#define  CAN_FFA1R_FFA5                       ((uint32_t)0x00000020)        /*!<Filter FIFO Assignment bit 5 */
#define  CAN_FFA1R_FFA6                       ((uint32_t)0x00000040)        /*!<Filter FIFO Assignment bit 6 */
#define  CAN_FFA1R_FFA7                       ((uint32_t)0x00000080)        /*!<Filter FIFO Assignment bit 7 */
#define  CAN_FFA1R_FFA8                       ((uint32_t)0x00000100)        /*!<Filter FIFO Assignment bit 8 */
#define  CAN_FFA1R_FFA9                       ((uint32_t)0x00000200)        /*!<Filter FIFO Assignment bit 9 */
#define  CAN_FFA1R_FFA10                      ((uint32_t)0x00000400)        /*!<Filter FIFO Assignment bit 10 */
#define  CAN_FFA1R_FFA11                      ((uint32_t)0x00000800)        /*!<Filter FIFO Assignment bit 11 */
#define  CAN_FFA1R_FFA12                      ((uint32_t)0x00001000)        /*!<Filter FIFO Assignment bit 12 */
#define  CAN_FFA1R_FFA13                      ((uint32_t)0x00002000)        /*!<Filter FIFO Assignment bit 13 */
#define  CAN_FFA1R_FFA14                      ((uint32_t)0x00004000)        /*!<Filter FIFO Assignment bit 14 */
#define  CAN_FFA1R_FFA15                      ((uint32_t)0x00008000)        /*!<Filter FIFO Assignment bit 15 */
#define  CAN_FFA1R_FFA16                      ((uint32_t)0x00010000)        /*!<Filter FIFO Assignment bit 16 */
#define  CAN_FFA1R_FFA17                      ((uint32_t)0x00020000)        /*!<Filter FIFO Assignment bit 17 */
#define  CAN_FFA1R_FFA18                      ((uint32_t)0x00040000)        /*!<Filter FIFO Assignment bit 18 */
#define  CAN_FFA1R_FFA19                      ((uint32_t)0x00080000)        /*!<Filter FIFO Assignment bit 19 */
#define  CAN_FFA1R_FFA20                      ((uint32_t)0x00100000)        /*!<Filter FIFO Assignment bit 20 */
#define  CAN_FFA1R_FFA21                      ((uint32_t)0x00200000)        /*!<Filter FIFO Assignment bit 21 */
#define  CAN_FFA1R_FFA22                      ((uint32_t)0x00400000)        /*!<Filter FIFO Assignment bit 22 */
#define  CAN_FFA1R_FFA23                      ((uint32_t)0x00800000)        /*!<Filter FIFO Assignment bit 23 */
#define  CAN_FFA1R_FFA24                      ((uint32_t)0x01000000)        /*!<Filter FIFO Assignment bit 24 */
#define  CAN_FFA1R_FFA25                      ((uint32_t)0x02000000)        /*!<Filter FIFO Assignment bit 25 */
#define  CAN_FFA1R_FFA26                      ((uint32_t)0x04000000)        /*!<Filter FIFO Assignment bit 26 */
#define  CAN_FFA1R_FFA27                      ((uint32_t)0x08000000)        /*!<Filter FIFO Assignment bit 27 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                        ((uint32_t)0x0FFFFFFF)        /*!<Filter Active */
#define  CAN_FA1R_FACT0                       ((uint32_t)0x00000001)        /*!<Filter Active bit 0 */
#define  CAN_FA1R_FACT1                       ((uint32_t)0x00000002)        /*!<Filter Active bit 1 */
#define  CAN_FA1R_FACT2                       ((uint32_t)0x00000004)        /*!<Filter Active bit 2 */
#define  CAN_FA1R_FACT3                       ((uint32_t)0x00000008)        /*!<Filter Active bit 3 */
#define  CAN_FA1R_FACT4                       ((uint32_t)0x00000010)        /*!<Filter Active bit 4 */
#define  CAN_FA1R_FACT5                       ((uint32_t)0x00000020)        /*!<Filter Active bit 5 */
#define  CAN_FA1R_FACT6                       ((uint32_t)0x00000040)        /*!<Filter Active bit 6 */
#define  CAN_FA1R_FACT7                       ((uint32_t)0x00000080)        /*!<Filter Active bit 7 */
#define  CAN_FA1R_FACT8                       ((uint32_t)0x00000100)        /*!<Filter Active bit 8 */
#define  CAN_FA1R_FACT9                       ((uint32_t)0x00000200)        /*!<Filter Active bit 9 */
#define  CAN_FA1R_FACT10                      ((uint32_t)0x00000400)        /*!<Filter Active bit 10 */
#define  CAN_FA1R_FACT11                      ((uint32_t)0x00000800)        /*!<Filter Active bit 11 */
#define  CAN_FA1R_FACT12                      ((uint32_t)0x00001000)        /*!<Filter Active bit 12 */
#define  CAN_FA1R_FACT13                      ((uint32_t)0x00002000)        /*!<Filter Active bit 13 */
#define  CAN_FA1R_FACT14                      ((uint32_t)0x00004000)        /*!<Filter Active bit 14 */
#define  CAN_FA1R_FACT15                      ((uint32_t)0x00008000)        /*!<Filter Active bit 15 */
#define  CAN_FA1R_FACT16                      ((uint32_t)0x00010000)        /*!<Filter Active bit 16 */
#define  CAN_FA1R_FACT17                      ((uint32_t)0x00020000)        /*!<Filter Active bit 17 */
#define  CAN_FA1R_FACT18                      ((uint32_t)0x00040000)        /*!<Filter Active bit 18 */
#define  CAN_FA1R_FACT19                      ((uint32_t)0x00080000)        /*!<Filter Active bit 19 */
#define  CAN_FA1R_FACT20                      ((uint32_t)0x00100000)        /*!<Filter Active bit 20 */
#define  CAN_FA1R_FACT21                      ((uint32_t)0x00200000)        /*!<Filter Active bit 21 */
#define  CAN_FA1R_FACT22                      ((uint32_t)0x00400000)        /*!<Filter Active bit 22 */
#define  CAN_FA1R_FACT23                      ((uint32_t)0x00800000)        /*!<Filter Active bit 23 */
#define  CAN_FA1R_FACT24                      ((uint32_t)0x01000000)        /*!<Filter Active bit 24 */
#define  CAN_FA1R_FACT25                      ((uint32_t)0x02000000)        /*!<Filter Active bit 25 */
#define  CAN_FA1R_FACT26                      ((uint32_t)0x04000000)        /*!<Filter Active bit 26 */
#define  CAN_FA1R_FACT27                      ((uint32_t)0x08000000)        /*!<Filter Active bit 27 */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define  CAN_F0R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F0R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F0R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F0R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F0R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F0R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F0R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F0R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F0R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F0R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F0R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F0R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F0R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F0R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F0R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F0R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F0R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F0R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F0R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F0R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F0R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F0R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F0R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F0R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F0R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F0R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F0R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F0R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F0R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F0R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F0R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F0R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define  CAN_F1R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F1R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F1R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F1R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F1R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F1R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F1R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F1R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F1R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F1R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F1R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F1R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F1R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F1R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F1R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F1R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F1R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F1R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F1R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F1R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F1R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F1R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F1R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F1R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F1R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F1R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F1R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F1R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F1R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F1R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F1R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F1R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define  CAN_F2R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F2R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F2R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F2R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F2R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F2R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F2R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F2R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F2R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F2R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F2R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F2R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F2R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F2R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F2R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F2R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F2R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F2R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F2R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F2R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F2R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F2R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F2R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F2R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F2R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F2R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F2R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F2R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F2R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F2R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F2R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F2R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define  CAN_F3R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F3R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F3R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F3R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F3R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F3R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F3R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F3R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F3R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F3R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F3R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F3R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F3R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F3R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F3R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F3R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F3R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F3R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F3R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F3R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F3R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F3R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F3R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F3R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F3R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F3R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F3R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F3R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F3R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F3R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F3R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F3R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define  CAN_F4R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F4R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F4R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F4R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F4R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F4R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F4R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F4R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F4R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F4R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F4R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F4R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F4R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F4R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F4R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F4R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F4R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F4R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F4R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F4R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F4R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F4R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F4R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F4R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F4R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F4R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F4R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F4R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F4R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F4R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F4R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F4R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define  CAN_F5R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F5R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F5R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F5R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F5R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F5R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F5R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F5R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F5R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F5R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F5R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F5R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F5R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F5R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F5R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F5R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F5R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F5R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F5R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F5R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F5R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F5R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F5R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F5R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F5R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F5R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F5R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F5R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F5R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F5R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F5R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F5R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define  CAN_F6R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F6R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F6R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F6R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F6R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F6R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F6R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F6R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F6R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F6R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F6R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F6R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F6R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F6R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F6R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F6R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F6R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F6R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F6R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F6R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F6R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F6R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F6R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F6R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F6R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F6R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F6R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F6R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F6R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F6R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F6R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F6R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define  CAN_F7R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F7R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F7R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F7R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F7R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F7R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F7R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F7R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F7R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F7R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F7R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F7R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F7R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F7R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F7R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F7R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F7R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F7R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F7R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F7R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F7R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F7R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F7R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F7R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F7R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F7R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F7R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F7R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F7R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F7R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F7R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F7R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define  CAN_F8R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F8R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F8R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F8R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F8R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F8R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F8R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F8R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F8R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F8R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F8R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F8R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F8R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F8R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F8R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F8R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F8R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F8R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F8R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F8R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F8R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F8R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F8R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F8R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F8R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F8R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F8R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F8R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F8R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F8R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F8R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F8R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define  CAN_F9R1_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F9R1_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F9R1_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F9R1_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F9R1_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F9R1_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F9R1_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F9R1_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F9R1_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F9R1_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F9R1_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F9R1_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F9R1_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F9R1_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F9R1_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F9R1_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F9R1_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F9R1_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F9R1_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F9R1_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F9R1_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F9R1_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F9R1_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F9R1_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F9R1_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F9R1_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F9R1_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F9R1_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F9R1_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F9R1_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F9R1_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F9R1_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define  CAN_F10R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F10R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F10R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F10R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F10R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F10R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F10R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F10R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F10R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F10R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F10R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F10R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F10R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F10R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F10R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F10R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F10R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F10R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F10R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F10R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F10R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F10R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F10R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F10R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F10R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F10R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F10R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F10R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F10R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F10R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F10R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F10R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define  CAN_F11R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F11R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F11R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F11R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F11R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F11R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F11R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F11R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F11R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F11R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F11R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F11R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F11R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F11R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F11R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F11R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F11R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F11R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F11R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F11R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F11R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F11R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F11R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F11R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F11R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F11R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F11R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F11R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F11R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F11R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F11R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F11R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define  CAN_F12R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F12R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F12R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F12R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F12R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F12R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F12R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F12R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F12R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F12R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F12R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F12R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F12R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F12R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F12R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F12R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F12R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F12R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F12R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F12R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F12R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F12R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F12R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F12R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F12R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F12R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F12R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F12R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F12R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F12R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F12R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F12R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define  CAN_F13R1_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F13R1_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F13R1_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F13R1_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F13R1_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F13R1_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F13R1_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F13R1_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F13R1_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F13R1_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F13R1_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F13R1_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F13R1_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F13R1_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F13R1_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F13R1_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F13R1_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F13R1_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F13R1_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F13R1_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F13R1_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F13R1_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F13R1_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F13R1_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F13R1_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F13R1_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F13R1_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F13R1_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F13R1_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F13R1_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F13R1_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F13R1_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define  CAN_F0R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F0R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F0R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F0R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F0R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F0R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F0R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F0R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F0R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F0R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F0R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F0R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F0R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F0R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F0R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F0R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F0R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F0R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F0R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F0R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F0R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F0R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F0R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F0R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F0R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F0R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F0R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F0R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F0R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F0R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F0R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F0R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define  CAN_F1R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F1R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F1R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F1R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F1R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F1R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F1R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F1R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F1R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F1R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F1R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F1R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F1R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F1R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F1R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F1R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F1R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F1R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F1R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F1R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F1R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F1R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F1R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F1R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F1R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F1R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F1R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F1R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F1R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F1R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F1R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F1R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define  CAN_F2R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F2R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F2R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F2R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F2R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F2R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F2R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F2R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F2R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F2R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F2R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F2R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F2R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F2R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F2R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F2R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F2R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F2R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F2R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F2R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F2R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F2R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F2R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F2R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F2R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F2R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F2R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F2R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F2R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F2R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F2R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F2R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define  CAN_F3R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F3R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F3R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F3R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F3R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F3R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F3R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F3R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F3R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F3R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F3R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F3R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F3R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F3R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F3R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F3R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F3R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F3R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F3R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F3R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F3R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F3R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F3R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F3R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F3R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F3R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F3R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F3R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F3R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F3R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F3R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F3R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define  CAN_F4R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F4R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F4R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F4R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F4R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F4R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F4R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F4R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F4R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F4R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F4R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F4R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F4R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F4R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F4R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F4R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F4R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F4R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F4R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F4R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F4R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F4R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F4R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F4R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F4R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F4R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F4R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F4R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F4R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F4R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F4R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F4R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define  CAN_F5R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F5R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F5R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F5R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F5R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F5R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F5R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F5R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F5R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F5R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F5R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F5R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F5R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F5R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F5R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F5R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F5R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F5R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F5R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F5R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F5R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F5R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F5R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F5R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F5R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F5R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F5R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F5R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F5R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F5R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F5R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F5R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define  CAN_F6R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F6R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F6R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F6R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F6R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F6R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F6R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F6R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F6R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F6R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F6R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F6R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F6R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F6R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F6R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F6R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F6R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F6R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F6R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F6R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F6R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F6R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F6R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F6R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F6R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F6R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F6R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F6R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F6R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F6R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F6R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F6R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define  CAN_F7R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F7R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F7R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F7R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F7R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F7R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F7R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F7R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F7R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F7R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F7R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F7R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F7R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F7R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F7R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F7R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F7R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F7R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F7R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F7R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F7R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F7R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F7R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F7R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F7R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F7R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F7R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F7R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F7R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F7R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F7R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F7R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define  CAN_F8R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F8R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F8R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F8R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F8R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F8R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F8R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F8R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F8R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F8R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F8R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F8R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F8R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F8R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F8R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F8R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F8R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F8R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F8R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F8R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F8R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F8R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F8R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F8R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F8R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F8R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F8R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F8R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F8R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F8R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F8R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F8R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define  CAN_F9R2_FB0                        ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F9R2_FB1                        ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F9R2_FB2                        ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F9R2_FB3                        ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F9R2_FB4                        ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F9R2_FB5                        ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F9R2_FB6                        ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F9R2_FB7                        ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F9R2_FB8                        ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F9R2_FB9                        ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F9R2_FB10                       ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F9R2_FB11                       ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F9R2_FB12                       ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F9R2_FB13                       ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F9R2_FB14                       ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F9R2_FB15                       ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F9R2_FB16                       ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F9R2_FB17                       ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F9R2_FB18                       ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F9R2_FB19                       ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F9R2_FB20                       ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F9R2_FB21                       ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F9R2_FB22                       ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F9R2_FB23                       ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F9R2_FB24                       ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F9R2_FB25                       ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F9R2_FB26                       ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F9R2_FB27                       ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F9R2_FB28                       ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F9R2_FB29                       ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F9R2_FB30                       ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F9R2_FB31                       ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define  CAN_F10R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F10R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F10R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F10R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F10R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F10R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F10R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F10R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F10R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F10R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F10R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F10R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F10R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F10R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F10R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F10R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F10R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F10R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F10R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F10R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F10R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F10R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F10R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F10R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F10R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F10R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F10R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F10R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F10R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F10R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F10R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F10R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define  CAN_F11R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F11R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F11R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F11R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F11R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F11R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F11R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F11R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F11R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F11R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F11R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F11R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F11R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F11R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F11R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F11R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F11R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F11R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F11R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F11R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F11R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F11R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F11R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F11R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F11R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F11R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F11R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F11R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F11R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F11R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F11R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F11R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define  CAN_F12R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F12R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F12R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F12R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F12R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F12R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F12R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F12R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F12R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F12R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F12R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F12R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F12R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F12R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F12R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F12R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F12R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F12R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F12R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F12R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F12R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F12R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F12R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F12R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F12R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F12R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F12R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F12R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F12R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F12R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F12R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F12R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define  CAN_F13R2_FB0                       ((uint32_t)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F13R2_FB1                       ((uint32_t)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F13R2_FB2                       ((uint32_t)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F13R2_FB3                       ((uint32_t)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F13R2_FB4                       ((uint32_t)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F13R2_FB5                       ((uint32_t)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F13R2_FB6                       ((uint32_t)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F13R2_FB7                       ((uint32_t)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F13R2_FB8                       ((uint32_t)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F13R2_FB9                       ((uint32_t)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F13R2_FB10                      ((uint32_t)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F13R2_FB11                      ((uint32_t)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F13R2_FB12                      ((uint32_t)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F13R2_FB13                      ((uint32_t)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F13R2_FB14                      ((uint32_t)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F13R2_FB15                      ((uint32_t)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F13R2_FB16                      ((uint32_t)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F13R2_FB17                      ((uint32_t)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F13R2_FB18                      ((uint32_t)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F13R2_FB19                      ((uint32_t)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F13R2_FB20                      ((uint32_t)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F13R2_FB21                      ((uint32_t)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F13R2_FB22                      ((uint32_t)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F13R2_FB23                      ((uint32_t)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F13R2_FB24                      ((uint32_t)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F13R2_FB25                      ((uint32_t)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F13R2_FB26                      ((uint32_t)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F13R2_FB27                      ((uint32_t)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F13R2_FB28                      ((uint32_t)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F13R2_FB29                      ((uint32_t)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F13R2_FB30                      ((uint32_t)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F13R2_FB31                      ((uint32_t)0x80000000)        /*!<Filter bit 31 */

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((uint32_t)0xFFFFFFFF) /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((uint32_t)0xFF)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((uint32_t)0x01)        /*!< RESET bit */


/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((uint32_t)0x00000001)        /*!<DAC channel1 enable */
#define  DAC_CR_BOFF1                        ((uint32_t)0x00000002)        /*!<DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         ((uint32_t)0x00000004)        /*!<DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((uint32_t)0x00000038)        /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((uint32_t)0x00000008)        /*!<Bit 0 */
#define  DAC_CR_TSEL1_1                      ((uint32_t)0x00000010)        /*!<Bit 1 */
#define  DAC_CR_TSEL1_2                      ((uint32_t)0x00000020)        /*!<Bit 2 */

#define  DAC_CR_WAVE1                        ((uint32_t)0x000000C0)        /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((uint32_t)0x00000040)        /*!<Bit 0 */
#define  DAC_CR_WAVE1_1                      ((uint32_t)0x00000080)        /*!<Bit 1 */

#define  DAC_CR_MAMP1                        ((uint32_t)0x00000F00)        /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  DAC_CR_MAMP1_1                      ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  DAC_CR_MAMP1_2                      ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  DAC_CR_MAMP1_3                      ((uint32_t)0x00000800)        /*!<Bit 3 */

#define  DAC_CR_DMAEN1                       ((uint32_t)0x00001000)        /*!<DAC channel1 DMA enable */
#define  DAC_CR_EN2                          ((uint32_t)0x00010000)        /*!<DAC channel2 enable */
#define  DAC_CR_BOFF2                        ((uint32_t)0x00020000)        /*!<DAC channel2 output buffer disable */
#define  DAC_CR_TEN2                         ((uint32_t)0x00040000)        /*!<DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((uint32_t)0x00380000)        /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((uint32_t)0x00080000)        /*!<Bit 0 */
#define  DAC_CR_TSEL2_1                      ((uint32_t)0x00100000)        /*!<Bit 1 */
#define  DAC_CR_TSEL2_2                      ((uint32_t)0x00200000)        /*!<Bit 2 */

#define  DAC_CR_WAVE2                        ((uint32_t)0x00C00000)        /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((uint32_t)0x00400000)        /*!<Bit 0 */
#define  DAC_CR_WAVE2_1                      ((uint32_t)0x00800000)        /*!<Bit 1 */

#define  DAC_CR_MAMP2                        ((uint32_t)0x0F000000)        /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  DAC_CR_MAMP2_1                      ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  DAC_CR_MAMP2_2                      ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  DAC_CR_MAMP2_3                      ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  DAC_CR_DMAEN2                       ((uint32_t)0x10000000)        /*!<DAC channel2 DMA enabled */

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((uint32_t)0x01)               /*!<DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((uint32_t)0x02)               /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((uint32_t)0x0FFF)            /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((uint32_t)0xFFF0)            /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((uint32_t)0xFF)               /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((uint32_t)0x0FFF)            /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((uint32_t)0xFFF0)            /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((uint32_t)0xFF)               /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((uint32_t)0x00000FFF)        /*!<DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((uint32_t)0x0FFF0000)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((uint32_t)0x0000FFF0)        /*!<DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((uint32_t)0xFFF00000)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((uint32_t)0x00FF)            /*!<DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((uint32_t)0xFF00)            /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((uint32_t)0x0FFF)            /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((uint32_t)0x0FFF)            /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((uint32_t)0x00002000)        /*!<DAC channel1 DMA underrun flag */
#define  DAC_SR_DMAUDR2                      ((uint32_t)0x20000000)        /*!<DAC channel2 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                                    DCMI                                    */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DCMI_CR register  ******************/
#define DCMI_CR_CAPTURE                      ((uint32_t)0x00000001)
#define DCMI_CR_CM                           ((uint32_t)0x00000002)
#define DCMI_CR_CROP                         ((uint32_t)0x00000004)
#define DCMI_CR_JPEG                         ((uint32_t)0x00000008)
#define DCMI_CR_ESS                          ((uint32_t)0x00000010)
#define DCMI_CR_PCKPOL                       ((uint32_t)0x00000020)
#define DCMI_CR_HSPOL                        ((uint32_t)0x00000040)
#define DCMI_CR_VSPOL                        ((uint32_t)0x00000080)
#define DCMI_CR_FCRC_0                       ((uint32_t)0x00000100)
#define DCMI_CR_FCRC_1                       ((uint32_t)0x00000200)
#define DCMI_CR_EDM_0                        ((uint32_t)0x00000400)
#define DCMI_CR_EDM_1                        ((uint32_t)0x00000800)
#define DCMI_CR_CRE                          ((uint32_t)0x00001000)
#define DCMI_CR_ENABLE                       ((uint32_t)0x00004000)

/********************  Bits definition for DCMI_SR register  ******************/
#define DCMI_SR_HSYNC                        ((uint32_t)0x00000001)
#define DCMI_SR_VSYNC                        ((uint32_t)0x00000002)
#define DCMI_SR_FNE                          ((uint32_t)0x00000004)

/********************  Bits definition for DCMI_RISR register  ****************/
#define DCMI_RISR_FRAME_RIS                  ((uint32_t)0x00000001)
#define DCMI_RISR_OVF_RIS                    ((uint32_t)0x00000002)
#define DCMI_RISR_ERR_RIS                    ((uint32_t)0x00000004)
#define DCMI_RISR_VSYNC_RIS                  ((uint32_t)0x00000008)
#define DCMI_RISR_LINE_RIS                   ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_IER register  *****************/
#define DCMI_IER_FRAME_IE                    ((uint32_t)0x00000001)
#define DCMI_IER_OVF_IE                      ((uint32_t)0x00000002)
#define DCMI_IER_ERR_IE                      ((uint32_t)0x00000004)
#define DCMI_IER_VSYNC_IE                    ((uint32_t)0x00000008)
#define DCMI_IER_LINE_IE                     ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_MISR register  ****************/
#define DCMI_MISR_FRAME_MIS                  ((uint32_t)0x00000001)
#define DCMI_MISR_OVF_MIS                    ((uint32_t)0x00000002)
#define DCMI_MISR_ERR_MIS                    ((uint32_t)0x00000004)
#define DCMI_MISR_VSYNC_MIS                  ((uint32_t)0x00000008)
#define DCMI_MISR_LINE_MIS                   ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_ICR register  *****************/
#define DCMI_ICR_FRAME_ISC                   ((uint32_t)0x00000001)
#define DCMI_ICR_OVF_ISC                     ((uint32_t)0x00000002)
#define DCMI_ICR_ERR_ISC                     ((uint32_t)0x00000004)
#define DCMI_ICR_VSYNC_ISC                   ((uint32_t)0x00000008)
#define DCMI_ICR_LINE_ISC                    ((uint32_t)0x00000010)

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/ 
#define DMA_SxCR_CHSEL                       ((uint32_t)0x0E000000)
#define DMA_SxCR_CHSEL_0                     ((uint32_t)0x02000000)
#define DMA_SxCR_CHSEL_1                     ((uint32_t)0x04000000)
#define DMA_SxCR_CHSEL_2                     ((uint32_t)0x08000000) 
#define DMA_SxCR_MBURST                      ((uint32_t)0x01800000)
#define DMA_SxCR_MBURST_0                    ((uint32_t)0x00800000)
#define DMA_SxCR_MBURST_1                    ((uint32_t)0x01000000)
#define DMA_SxCR_PBURST                      ((uint32_t)0x00600000)
#define DMA_SxCR_PBURST_0                    ((uint32_t)0x00200000)
#define DMA_SxCR_PBURST_1                    ((uint32_t)0x00400000)
#define DMA_SxCR_ACK                         ((uint32_t)0x00100000)
#define DMA_SxCR_CT                          ((uint32_t)0x00080000)  
#define DMA_SxCR_DBM                         ((uint32_t)0x00040000)
#define DMA_SxCR_PL                          ((uint32_t)0x00030000)
#define DMA_SxCR_PL_0                        ((uint32_t)0x00010000)
#define DMA_SxCR_PL_1                        ((uint32_t)0x00020000)
#define DMA_SxCR_PINCOS                      ((uint32_t)0x00008000)
#define DMA_SxCR_MSIZE                       ((uint32_t)0x00006000)
#define DMA_SxCR_MSIZE_0                     ((uint32_t)0x00002000)
#define DMA_SxCR_MSIZE_1                     ((uint32_t)0x00004000)
#define DMA_SxCR_PSIZE                       ((uint32_t)0x00001800)
#define DMA_SxCR_PSIZE_0                     ((uint32_t)0x00000800)
#define DMA_SxCR_PSIZE_1                     ((uint32_t)0x00001000)
#define DMA_SxCR_MINC                        ((uint32_t)0x00000400)
#define DMA_SxCR_PINC                        ((uint32_t)0x00000200)
#define DMA_SxCR_CIRC                        ((uint32_t)0x00000100)
#define DMA_SxCR_DIR                         ((uint32_t)0x000000C0)
#define DMA_SxCR_DIR_0                       ((uint32_t)0x00000040)
#define DMA_SxCR_DIR_1                       ((uint32_t)0x00000080)
#define DMA_SxCR_PFCTRL                      ((uint32_t)0x00000020)
#define DMA_SxCR_TCIE                        ((uint32_t)0x00000010)
#define DMA_SxCR_HTIE                        ((uint32_t)0x00000008)
#define DMA_SxCR_TEIE                        ((uint32_t)0x00000004)
#define DMA_SxCR_DMEIE                       ((uint32_t)0x00000002)
#define DMA_SxCR_EN                          ((uint32_t)0x00000001)

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT                            ((uint32_t)0x0000FFFF)
#define DMA_SxNDT_0                          ((uint32_t)0x00000001)
#define DMA_SxNDT_1                          ((uint32_t)0x00000002)
#define DMA_SxNDT_2                          ((uint32_t)0x00000004)
#define DMA_SxNDT_3                          ((uint32_t)0x00000008)
#define DMA_SxNDT_4                          ((uint32_t)0x00000010)
#define DMA_SxNDT_5                          ((uint32_t)0x00000020)
#define DMA_SxNDT_6                          ((uint32_t)0x00000040)
#define DMA_SxNDT_7                          ((uint32_t)0x00000080)
#define DMA_SxNDT_8                          ((uint32_t)0x00000100)
#define DMA_SxNDT_9                          ((uint32_t)0x00000200)
#define DMA_SxNDT_10                         ((uint32_t)0x00000400)
#define DMA_SxNDT_11                         ((uint32_t)0x00000800)
#define DMA_SxNDT_12                         ((uint32_t)0x00001000)
#define DMA_SxNDT_13                         ((uint32_t)0x00002000)
#define DMA_SxNDT_14                         ((uint32_t)0x00004000)
#define DMA_SxNDT_15                         ((uint32_t)0x00008000)

/********************  Bits definition for DMA_SxFCR register  ****************/ 
#define DMA_SxFCR_FEIE                       ((uint32_t)0x00000080)
#define DMA_SxFCR_FS                         ((uint32_t)0x00000038)
#define DMA_SxFCR_FS_0                       ((uint32_t)0x00000008)
#define DMA_SxFCR_FS_1                       ((uint32_t)0x00000010)
#define DMA_SxFCR_FS_2                       ((uint32_t)0x00000020)
#define DMA_SxFCR_DMDIS                      ((uint32_t)0x00000004)
#define DMA_SxFCR_FTH                        ((uint32_t)0x00000003)
#define DMA_SxFCR_FTH_0                      ((uint32_t)0x00000001)
#define DMA_SxFCR_FTH_1                      ((uint32_t)0x00000002)

/********************  Bits definition for DMA_LISR register  *****************/ 
#define DMA_LISR_TCIF3                       ((uint32_t)0x08000000)
#define DMA_LISR_HTIF3                       ((uint32_t)0x04000000)
#define DMA_LISR_TEIF3                       ((uint32_t)0x02000000)
#define DMA_LISR_DMEIF3                      ((uint32_t)0x01000000)
#define DMA_LISR_FEIF3                       ((uint32_t)0x00400000)
#define DMA_LISR_TCIF2                       ((uint32_t)0x00200000)
#define DMA_LISR_HTIF2                       ((uint32_t)0x00100000)
#define DMA_LISR_TEIF2                       ((uint32_t)0x00080000)
#define DMA_LISR_DMEIF2                      ((uint32_t)0x00040000)
#define DMA_LISR_FEIF2                       ((uint32_t)0x00010000)
#define DMA_LISR_TCIF1                       ((uint32_t)0x00000800)
#define DMA_LISR_HTIF1                       ((uint32_t)0x00000400)
#define DMA_LISR_TEIF1                       ((uint32_t)0x00000200)
#define DMA_LISR_DMEIF1                      ((uint32_t)0x00000100)
#define DMA_LISR_FEIF1                       ((uint32_t)0x00000040)
#define DMA_LISR_TCIF0                       ((uint32_t)0x00000020)
#define DMA_LISR_HTIF0                       ((uint32_t)0x00000010)
#define DMA_LISR_TEIF0                       ((uint32_t)0x00000008)
#define DMA_LISR_DMEIF0                      ((uint32_t)0x00000004)
#define DMA_LISR_FEIF0                       ((uint32_t)0x00000001)

/********************  Bits definition for DMA_HISR register  *****************/ 
#define DMA_HISR_TCIF7                       ((uint32_t)0x08000000)
#define DMA_HISR_HTIF7                       ((uint32_t)0x04000000)
#define DMA_HISR_TEIF7                       ((uint32_t)0x02000000)
#define DMA_HISR_DMEIF7                      ((uint32_t)0x01000000)
#define DMA_HISR_FEIF7                       ((uint32_t)0x00400000)
#define DMA_HISR_TCIF6                       ((uint32_t)0x00200000)
#define DMA_HISR_HTIF6                       ((uint32_t)0x00100000)
#define DMA_HISR_TEIF6                       ((uint32_t)0x00080000)
#define DMA_HISR_DMEIF6                      ((uint32_t)0x00040000)
#define DMA_HISR_FEIF6                       ((uint32_t)0x00010000)
#define DMA_HISR_TCIF5                       ((uint32_t)0x00000800)
#define DMA_HISR_HTIF5                       ((uint32_t)0x00000400)
#define DMA_HISR_TEIF5                       ((uint32_t)0x00000200)
#define DMA_HISR_DMEIF5                      ((uint32_t)0x00000100)
#define DMA_HISR_FEIF5                       ((uint32_t)0x00000040)
#define DMA_HISR_TCIF4                       ((uint32_t)0x00000020)
#define DMA_HISR_HTIF4                       ((uint32_t)0x00000010)
#define DMA_HISR_TEIF4                       ((uint32_t)0x00000008)
#define DMA_HISR_DMEIF4                      ((uint32_t)0x00000004)
#define DMA_HISR_FEIF4                       ((uint32_t)0x00000001)

/********************  Bits definition for DMA_LIFCR register  ****************/ 
#define DMA_LIFCR_CTCIF3                     ((uint32_t)0x08000000)
#define DMA_LIFCR_CHTIF3                     ((uint32_t)0x04000000)
#define DMA_LIFCR_CTEIF3                     ((uint32_t)0x02000000)
#define DMA_LIFCR_CDMEIF3                    ((uint32_t)0x01000000)
#define DMA_LIFCR_CFEIF3                     ((uint32_t)0x00400000)
#define DMA_LIFCR_CTCIF2                     ((uint32_t)0x00200000)
#define DMA_LIFCR_CHTIF2                     ((uint32_t)0x00100000)
#define DMA_LIFCR_CTEIF2                     ((uint32_t)0x00080000)
#define DMA_LIFCR_CDMEIF2                    ((uint32_t)0x00040000)
#define DMA_LIFCR_CFEIF2                     ((uint32_t)0x00010000)
#define DMA_LIFCR_CTCIF1                     ((uint32_t)0x00000800)
#define DMA_LIFCR_CHTIF1                     ((uint32_t)0x00000400)
#define DMA_LIFCR_CTEIF1                     ((uint32_t)0x00000200)
#define DMA_LIFCR_CDMEIF1                    ((uint32_t)0x00000100)
#define DMA_LIFCR_CFEIF1                     ((uint32_t)0x00000040)
#define DMA_LIFCR_CTCIF0                     ((uint32_t)0x00000020)
#define DMA_LIFCR_CHTIF0                     ((uint32_t)0x00000010)
#define DMA_LIFCR_CTEIF0                     ((uint32_t)0x00000008)
#define DMA_LIFCR_CDMEIF0                    ((uint32_t)0x00000004)
#define DMA_LIFCR_CFEIF0                     ((uint32_t)0x00000001)

/********************  Bits definition for DMA_HIFCR  register  ****************/ 
#define DMA_HIFCR_CTCIF7                     ((uint32_t)0x08000000)
#define DMA_HIFCR_CHTIF7                     ((uint32_t)0x04000000)
#define DMA_HIFCR_CTEIF7                     ((uint32_t)0x02000000)
#define DMA_HIFCR_CDMEIF7                    ((uint32_t)0x01000000)
#define DMA_HIFCR_CFEIF7                     ((uint32_t)0x00400000)
#define DMA_HIFCR_CTCIF6                     ((uint32_t)0x00200000)
#define DMA_HIFCR_CHTIF6                     ((uint32_t)0x00100000)
#define DMA_HIFCR_CTEIF6                     ((uint32_t)0x00080000)
#define DMA_HIFCR_CDMEIF6                    ((uint32_t)0x00040000)
#define DMA_HIFCR_CFEIF6                     ((uint32_t)0x00010000)
#define DMA_HIFCR_CTCIF5                     ((uint32_t)0x00000800)
#define DMA_HIFCR_CHTIF5                     ((uint32_t)0x00000400)
#define DMA_HIFCR_CTEIF5                     ((uint32_t)0x00000200)
#define DMA_HIFCR_CDMEIF5                    ((uint32_t)0x00000100)
#define DMA_HIFCR_CFEIF5                     ((uint32_t)0x00000040)
#define DMA_HIFCR_CTCIF4                     ((uint32_t)0x00000020)
#define DMA_HIFCR_CHTIF4                     ((uint32_t)0x00000010)
#define DMA_HIFCR_CTEIF4                     ((uint32_t)0x00000008)
#define DMA_HIFCR_CDMEIF4                    ((uint32_t)0x00000004)
#define DMA_HIFCR_CFEIF4                     ((uint32_t)0x00000001)


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        ((uint32_t)0x00000001)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        ((uint32_t)0x00000002)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        ((uint32_t)0x00000004)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        ((uint32_t)0x00000008)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        ((uint32_t)0x00000010)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        ((uint32_t)0x00000020)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        ((uint32_t)0x00000040)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        ((uint32_t)0x00000080)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        ((uint32_t)0x00000100)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        ((uint32_t)0x00000200)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       ((uint32_t)0x00000400)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       ((uint32_t)0x00000800)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       ((uint32_t)0x00001000)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       ((uint32_t)0x00002000)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       ((uint32_t)0x00004000)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       ((uint32_t)0x00008000)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       ((uint32_t)0x00010000)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       ((uint32_t)0x00020000)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       ((uint32_t)0x00040000)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR_MR19                       ((uint32_t)0x00080000)        /*!< Interrupt Mask on line 19 */
#define  EXTI_IMR_MR20                       ((uint32_t)0x00100000)        /*!< Interrupt Mask on line 20 */
#define  EXTI_IMR_MR21                       ((uint32_t)0x00200000)        /*!< Interrupt Mask on line 21 */
#define  EXTI_IMR_MR22                       ((uint32_t)0x00400000)        /*!< Interrupt Mask on line 22 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        ((uint32_t)0x00000001)        /*!< Event Mask on line 0 */
#define  EXTI_EMR_MR1                        ((uint32_t)0x00000002)        /*!< Event Mask on line 1 */
#define  EXTI_EMR_MR2                        ((uint32_t)0x00000004)        /*!< Event Mask on line 2 */
#define  EXTI_EMR_MR3                        ((uint32_t)0x00000008)        /*!< Event Mask on line 3 */
#define  EXTI_EMR_MR4                        ((uint32_t)0x00000010)        /*!< Event Mask on line 4 */
#define  EXTI_EMR_MR5                        ((uint32_t)0x00000020)        /*!< Event Mask on line 5 */
#define  EXTI_EMR_MR6                        ((uint32_t)0x00000040)        /*!< Event Mask on line 6 */
#define  EXTI_EMR_MR7                        ((uint32_t)0x00000080)        /*!< Event Mask on line 7 */
#define  EXTI_EMR_MR8                        ((uint32_t)0x00000100)        /*!< Event Mask on line 8 */
#define  EXTI_EMR_MR9                        ((uint32_t)0x00000200)        /*!< Event Mask on line 9 */
#define  EXTI_EMR_MR10                       ((uint32_t)0x00000400)        /*!< Event Mask on line 10 */
#define  EXTI_EMR_MR11                       ((uint32_t)0x00000800)        /*!< Event Mask on line 11 */
#define  EXTI_EMR_MR12                       ((uint32_t)0x00001000)        /*!< Event Mask on line 12 */
#define  EXTI_EMR_MR13                       ((uint32_t)0x00002000)        /*!< Event Mask on line 13 */
#define  EXTI_EMR_MR14                       ((uint32_t)0x00004000)        /*!< Event Mask on line 14 */
#define  EXTI_EMR_MR15                       ((uint32_t)0x00008000)        /*!< Event Mask on line 15 */
#define  EXTI_EMR_MR16                       ((uint32_t)0x00010000)        /*!< Event Mask on line 16 */
#define  EXTI_EMR_MR17                       ((uint32_t)0x00020000)        /*!< Event Mask on line 17 */
#define  EXTI_EMR_MR18                       ((uint32_t)0x00040000)        /*!< Event Mask on line 18 */
#define  EXTI_EMR_MR19                       ((uint32_t)0x00080000)        /*!< Event Mask on line 19 */
#define  EXTI_EMR_MR20                       ((uint32_t)0x00100000)        /*!< Event Mask on line 20 */
#define  EXTI_EMR_MR21                       ((uint32_t)0x00200000)        /*!< Event Mask on line 21 */
#define  EXTI_EMR_MR22                       ((uint32_t)0x00400000)        /*!< Event Mask on line 22 */

/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       ((uint32_t)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       ((uint32_t)0x00000002)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       ((uint32_t)0x00000004)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       ((uint32_t)0x00000008)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       ((uint32_t)0x00000010)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       ((uint32_t)0x00000020)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       ((uint32_t)0x00000040)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       ((uint32_t)0x00000080)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       ((uint32_t)0x00000100)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       ((uint32_t)0x00000200)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      ((uint32_t)0x00000400)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      ((uint32_t)0x00000800)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      ((uint32_t)0x00001000)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      ((uint32_t)0x00002000)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      ((uint32_t)0x00004000)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      ((uint32_t)0x00008000)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      ((uint32_t)0x00010000)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      ((uint32_t)0x00020000)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      ((uint32_t)0x00040000)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR_TR19                      ((uint32_t)0x00080000)        /*!< Rising trigger event configuration bit of line 19 */
#define  EXTI_RTSR_TR20                      ((uint32_t)0x00100000)        /*!< Rising trigger event configuration bit of line 20 */
#define  EXTI_RTSR_TR21                      ((uint32_t)0x00200000)        /*!< Rising trigger event configuration bit of line 21 */
#define  EXTI_RTSR_TR22                      ((uint32_t)0x00400000)        /*!< Rising trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       ((uint32_t)0x00000001)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       ((uint32_t)0x00000002)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       ((uint32_t)0x00000004)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       ((uint32_t)0x00000008)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       ((uint32_t)0x00000010)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       ((uint32_t)0x00000020)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       ((uint32_t)0x00000040)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       ((uint32_t)0x00000080)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       ((uint32_t)0x00000100)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       ((uint32_t)0x00000200)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      ((uint32_t)0x00000400)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      ((uint32_t)0x00000800)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      ((uint32_t)0x00001000)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      ((uint32_t)0x00002000)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      ((uint32_t)0x00004000)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      ((uint32_t)0x00008000)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      ((uint32_t)0x00010000)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      ((uint32_t)0x00020000)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      ((uint32_t)0x00040000)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR_TR19                      ((uint32_t)0x00080000)        /*!< Falling trigger event configuration bit of line 19 */
#define  EXTI_FTSR_TR20                      ((uint32_t)0x00100000)        /*!< Falling trigger event configuration bit of line 20 */
#define  EXTI_FTSR_TR21                      ((uint32_t)0x00200000)        /*!< Falling trigger event configuration bit of line 21 */
#define  EXTI_FTSR_TR22                      ((uint32_t)0x00400000)        /*!< Falling trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   ((uint32_t)0x00000001)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   ((uint32_t)0x00000002)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   ((uint32_t)0x00000004)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   ((uint32_t)0x00000008)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   ((uint32_t)0x00000010)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   ((uint32_t)0x00000020)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   ((uint32_t)0x00000040)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   ((uint32_t)0x00000080)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   ((uint32_t)0x00000100)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   ((uint32_t)0x00000200)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  ((uint32_t)0x00000400)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  ((uint32_t)0x00000800)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  ((uint32_t)0x00001000)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  ((uint32_t)0x00002000)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  ((uint32_t)0x00004000)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  ((uint32_t)0x00008000)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  ((uint32_t)0x00010000)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  ((uint32_t)0x00020000)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  ((uint32_t)0x00040000)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER_SWIER19                  ((uint32_t)0x00080000)        /*!< Software Interrupt on line 19 */
#define  EXTI_SWIER_SWIER20                  ((uint32_t)0x00100000)        /*!< Software Interrupt on line 20 */
#define  EXTI_SWIER_SWIER21                  ((uint32_t)0x00200000)        /*!< Software Interrupt on line 21 */
#define  EXTI_SWIER_SWIER22                  ((uint32_t)0x00400000)        /*!< Software Interrupt on line 22 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         ((uint32_t)0x00000001)        /*!< Pending bit for line 0 */
#define  EXTI_PR_PR1                         ((uint32_t)0x00000002)        /*!< Pending bit for line 1 */
#define  EXTI_PR_PR2                         ((uint32_t)0x00000004)        /*!< Pending bit for line 2 */
#define  EXTI_PR_PR3                         ((uint32_t)0x00000008)        /*!< Pending bit for line 3 */
#define  EXTI_PR_PR4                         ((uint32_t)0x00000010)        /*!< Pending bit for line 4 */
#define  EXTI_PR_PR5                         ((uint32_t)0x00000020)        /*!< Pending bit for line 5 */
#define  EXTI_PR_PR6                         ((uint32_t)0x00000040)        /*!< Pending bit for line 6 */
#define  EXTI_PR_PR7                         ((uint32_t)0x00000080)        /*!< Pending bit for line 7 */
#define  EXTI_PR_PR8                         ((uint32_t)0x00000100)        /*!< Pending bit for line 8 */
#define  EXTI_PR_PR9                         ((uint32_t)0x00000200)        /*!< Pending bit for line 9 */
#define  EXTI_PR_PR10                        ((uint32_t)0x00000400)        /*!< Pending bit for line 10 */
#define  EXTI_PR_PR11                        ((uint32_t)0x00000800)        /*!< Pending bit for line 11 */
#define  EXTI_PR_PR12                        ((uint32_t)0x00001000)        /*!< Pending bit for line 12 */
#define  EXTI_PR_PR13                        ((uint32_t)0x00002000)        /*!< Pending bit for line 13 */
#define  EXTI_PR_PR14                        ((uint32_t)0x00004000)        /*!< Pending bit for line 14 */
#define  EXTI_PR_PR15                        ((uint32_t)0x00008000)        /*!< Pending bit for line 15 */
#define  EXTI_PR_PR16                        ((uint32_t)0x00010000)        /*!< Pending bit for line 16 */
#define  EXTI_PR_PR17                        ((uint32_t)0x00020000)        /*!< Pending bit for line 17 */
#define  EXTI_PR_PR18                        ((uint32_t)0x00040000)        /*!< Pending bit for line 18 */
#define  EXTI_PR_PR19                        ((uint32_t)0x00080000)        /*!< Pending bit for line 19 */
#define  EXTI_PR_PR20                        ((uint32_t)0x00100000)        /*!< Pending bit for line 20 */
#define  EXTI_PR_PR21                        ((uint32_t)0x00200000)        /*!< Pending bit for line 21 */
#define  EXTI_PR_PR22                        ((uint32_t)0x00400000)        /*!< Pending bit for line 22 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    ((uint32_t)0x0000000F)
#define FLASH_ACR_LATENCY_0WS                ((uint32_t)0x00000000)
#define FLASH_ACR_LATENCY_1WS                ((uint32_t)0x00000001)
#define FLASH_ACR_LATENCY_2WS                ((uint32_t)0x00000002)
#define FLASH_ACR_LATENCY_3WS                ((uint32_t)0x00000003)
#define FLASH_ACR_LATENCY_4WS                ((uint32_t)0x00000004)
#define FLASH_ACR_LATENCY_5WS                ((uint32_t)0x00000005)
#define FLASH_ACR_LATENCY_6WS                ((uint32_t)0x00000006)
#define FLASH_ACR_LATENCY_7WS                ((uint32_t)0x00000007)

#define FLASH_ACR_PRFTEN                     ((uint32_t)0x00000100)
#define FLASH_ACR_ICEN                       ((uint32_t)0x00000200)
#define FLASH_ACR_DCEN                       ((uint32_t)0x00000400)
#define FLASH_ACR_ICRST                      ((uint32_t)0x00000800)
#define FLASH_ACR_DCRST                      ((uint32_t)0x00001000)
#define FLASH_ACR_BYTE0_ADDRESS              ((uint32_t)0x40023C00)
#define FLASH_ACR_BYTE2_ADDRESS              ((uint32_t)0x40023C03)

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP                         ((uint32_t)0x00000001)
#define FLASH_SR_SOP                         ((uint32_t)0x00000002)
#define FLASH_SR_WRPERR                      ((uint32_t)0x00000010)
#define FLASH_SR_PGAERR                      ((uint32_t)0x00000020)
#define FLASH_SR_PGPERR                      ((uint32_t)0x00000040)
#define FLASH_SR_PGSERR                      ((uint32_t)0x00000080)
#define FLASH_SR_BSY                         ((uint32_t)0x00010000)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          ((uint32_t)0x00000001)
#define FLASH_CR_SER                         ((uint32_t)0x00000002)
#define FLASH_CR_MER                         ((uint32_t)0x00000004)
#define FLASH_CR_SNB                         ((uint32_t)0x000000F8)
#define FLASH_CR_SNB_0                       ((uint32_t)0x00000008)
#define FLASH_CR_SNB_1                       ((uint32_t)0x00000010)
#define FLASH_CR_SNB_2                       ((uint32_t)0x00000020)
#define FLASH_CR_SNB_3                       ((uint32_t)0x00000040)
#define FLASH_CR_SNB_4                       ((uint32_t)0x00000080)
#define FLASH_CR_PSIZE                       ((uint32_t)0x00000300)
#define FLASH_CR_PSIZE_0                     ((uint32_t)0x00000100)
#define FLASH_CR_PSIZE_1                     ((uint32_t)0x00000200)
#define FLASH_CR_STRT                        ((uint32_t)0x00010000)
#define FLASH_CR_EOPIE                       ((uint32_t)0x01000000)
#define FLASH_CR_LOCK                        ((uint32_t)0x80000000)

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK                 ((uint32_t)0x00000001)
#define FLASH_OPTCR_OPTSTRT                 ((uint32_t)0x00000002)
#define FLASH_OPTCR_BOR_LEV_0               ((uint32_t)0x00000004)
#define FLASH_OPTCR_BOR_LEV_1               ((uint32_t)0x00000008)
#define FLASH_OPTCR_BOR_LEV                 ((uint32_t)0x0000000C)

#define FLASH_OPTCR_WDG_SW                  ((uint32_t)0x00000020)
#define FLASH_OPTCR_nRST_STOP               ((uint32_t)0x00000040)
#define FLASH_OPTCR_nRST_STDBY              ((uint32_t)0x00000080)
#define FLASH_OPTCR_RDP                     ((uint32_t)0x0000FF00)
#define FLASH_OPTCR_RDP_0                   ((uint32_t)0x00000100)
#define FLASH_OPTCR_RDP_1                   ((uint32_t)0x00000200)
#define FLASH_OPTCR_RDP_2                   ((uint32_t)0x00000400)
#define FLASH_OPTCR_RDP_3                   ((uint32_t)0x00000800)
#define FLASH_OPTCR_RDP_4                   ((uint32_t)0x00001000)
#define FLASH_OPTCR_RDP_5                   ((uint32_t)0x00002000)
#define FLASH_OPTCR_RDP_6                   ((uint32_t)0x00004000)
#define FLASH_OPTCR_RDP_7                   ((uint32_t)0x00008000)
#define FLASH_OPTCR_nWRP                    ((uint32_t)0x0FFF0000)
#define FLASH_OPTCR_nWRP_0                  ((uint32_t)0x00010000)
#define FLASH_OPTCR_nWRP_1                  ((uint32_t)0x00020000)
#define FLASH_OPTCR_nWRP_2                  ((uint32_t)0x00040000)
#define FLASH_OPTCR_nWRP_3                  ((uint32_t)0x00080000)
#define FLASH_OPTCR_nWRP_4                  ((uint32_t)0x00100000)
#define FLASH_OPTCR_nWRP_5                  ((uint32_t)0x00200000)
#define FLASH_OPTCR_nWRP_6                  ((uint32_t)0x00400000)
#define FLASH_OPTCR_nWRP_7                  ((uint32_t)0x00800000)
#define FLASH_OPTCR_nWRP_8                  ((uint32_t)0x01000000)
#define FLASH_OPTCR_nWRP_9                  ((uint32_t)0x02000000)
#define FLASH_OPTCR_nWRP_10                 ((uint32_t)0x04000000)
#define FLASH_OPTCR_nWRP_11                 ((uint32_t)0x08000000)
                                             
/******************  Bits definition for FLASH_OPTCR1 register  ***************/
#define FLASH_OPTCR1_nWRP                    ((uint32_t)0x0FFF0000)
#define FLASH_OPTCR1_nWRP_0                  ((uint32_t)0x00010000)
#define FLASH_OPTCR1_nWRP_1                  ((uint32_t)0x00020000)
#define FLASH_OPTCR1_nWRP_2                  ((uint32_t)0x00040000)
#define FLASH_OPTCR1_nWRP_3                  ((uint32_t)0x00080000)
#define FLASH_OPTCR1_nWRP_4                  ((uint32_t)0x00100000)
#define FLASH_OPTCR1_nWRP_5                  ((uint32_t)0x00200000)
#define FLASH_OPTCR1_nWRP_6                  ((uint32_t)0x00400000)
#define FLASH_OPTCR1_nWRP_7                  ((uint32_t)0x00800000)
#define FLASH_OPTCR1_nWRP_8                  ((uint32_t)0x01000000)
#define FLASH_OPTCR1_nWRP_9                  ((uint32_t)0x02000000)
#define FLASH_OPTCR1_nWRP_10                 ((uint32_t)0x04000000)
#define FLASH_OPTCR1_nWRP_11                 ((uint32_t)0x08000000)

/******************************************************************************/
/*                                                                            */
/*                       Flexible Static Memory Controller                    */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for FSMC_BCR1 register  *******************/
#define  FSMC_BCR1_MBKEN                     ((uint32_t)0x00000001)        /*!<Memory bank enable bit                 */
#define  FSMC_BCR1_MUXEN                     ((uint32_t)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR1_MTYP                      ((uint32_t)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR1_MTYP_0                    ((uint32_t)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR1_MTYP_1                    ((uint32_t)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR1_MWID                      ((uint32_t)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR1_MWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR1_MWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR1_FACCEN                    ((uint32_t)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR1_BURSTEN                   ((uint32_t)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR1_WAITPOL                   ((uint32_t)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR1_WRAPMOD                   ((uint32_t)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR1_WAITCFG                   ((uint32_t)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR1_WREN                      ((uint32_t)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR1_WAITEN                    ((uint32_t)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR1_EXTMOD                    ((uint32_t)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR1_ASYNCWAIT                 ((uint32_t)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR1_CPSIZE                    ((uint32_t)0x00070000)        /*!<CRAM page size */
#define  FSMC_BCR1_CPSIZE_0                  ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BCR1_CPSIZE_1                  ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BCR1_CPSIZE_2                  ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BCR1_CBURSTRW                  ((uint32_t)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR2 register  *******************/
#define  FSMC_BCR2_MBKEN                     ((uint32_t)0x00000001)        /*!<Memory bank enable bit                */
#define  FSMC_BCR2_MUXEN                     ((uint32_t)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR2_MTYP                      ((uint32_t)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR2_MTYP_0                    ((uint32_t)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR2_MTYP_1                    ((uint32_t)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR2_MWID                      ((uint32_t)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR2_MWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR2_MWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR2_FACCEN                    ((uint32_t)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR2_BURSTEN                   ((uint32_t)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR2_WAITPOL                   ((uint32_t)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR2_WRAPMOD                   ((uint32_t)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR2_WAITCFG                   ((uint32_t)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR2_WREN                      ((uint32_t)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR2_WAITEN                    ((uint32_t)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR2_EXTMOD                    ((uint32_t)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR2_ASYNCWAIT                 ((uint32_t)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR2_CPSIZE                    ((uint32_t)0x00070000)        /*!<CRAM page size */
#define  FSMC_BCR2_CPSIZE_0                  ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BCR2_CPSIZE_1                  ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BCR2_CPSIZE_2                  ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BCR2_CBURSTRW                  ((uint32_t)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR3 register  *******************/
#define  FSMC_BCR3_MBKEN                     ((uint32_t)0x00000001)        /*!<Memory bank enable bit                 */
#define  FSMC_BCR3_MUXEN                     ((uint32_t)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR3_MTYP                      ((uint32_t)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR3_MTYP_0                    ((uint32_t)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR3_MTYP_1                    ((uint32_t)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR3_MWID                      ((uint32_t)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR3_MWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR3_MWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR3_FACCEN                    ((uint32_t)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR3_BURSTEN                   ((uint32_t)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR3_WAITPOL                   ((uint32_t)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR3_WRAPMOD                   ((uint32_t)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR3_WAITCFG                   ((uint32_t)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR3_WREN                      ((uint32_t)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR3_WAITEN                    ((uint32_t)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR3_EXTMOD                    ((uint32_t)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR3_ASYNCWAIT                 ((uint32_t)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR3_CPSIZE                    ((uint32_t)0x00070000)        /*!<CRAM page size */
#define  FSMC_BCR3_CPSIZE_0                  ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BCR3_CPSIZE_1                  ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BCR3_CPSIZE_2                  ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BCR3_CBURSTRW                  ((uint32_t)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR4 register  *******************/
#define  FSMC_BCR4_MBKEN                     ((uint32_t)0x00000001)        /*!<Memory bank enable bit */
#define  FSMC_BCR4_MUXEN                     ((uint32_t)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR4_MTYP                      ((uint32_t)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR4_MTYP_0                    ((uint32_t)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR4_MTYP_1                    ((uint32_t)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR4_MWID                      ((uint32_t)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR4_MWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR4_MWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR4_FACCEN                    ((uint32_t)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR4_BURSTEN                   ((uint32_t)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR4_WAITPOL                   ((uint32_t)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR4_WRAPMOD                   ((uint32_t)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR4_WAITCFG                   ((uint32_t)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR4_WREN                      ((uint32_t)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR4_WAITEN                    ((uint32_t)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR4_EXTMOD                    ((uint32_t)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR4_ASYNCWAIT                 ((uint32_t)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR4_CPSIZE                    ((uint32_t)0x00070000)        /*!<CRAM page size */
#define  FSMC_BCR4_CPSIZE_0                  ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BCR4_CPSIZE_1                  ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BCR4_CPSIZE_2                  ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BCR4_CBURSTRW                  ((uint32_t)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BTR1 register  ******************/
#define  FSMC_BTR1_ADDSET                    ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR1_ADDSET_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR1_ADDSET_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR1_ADDSET_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR1_ADDSET_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR1_ADDHLD                    ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR1_ADDHLD_0                  ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR1_ADDHLD_1                  ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR1_ADDHLD_2                  ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR1_ADDHLD_3                  ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR1_DATAST                    ((uint32_t)0x0000FF00)        /*!<DATAST [7:0] bits (Data-phase duration) */
#define  FSMC_BTR1_DATAST_0                  ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR1_DATAST_1                  ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR1_DATAST_2                  ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR1_DATAST_3                  ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BTR1_DATAST_4                  ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BTR1_DATAST_5                  ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BTR1_DATAST_6                  ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BTR1_DATAST_7                  ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BTR1_BUSTURN                   ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR1_BUSTURN_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR1_BUSTURN_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR1_BUSTURN_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR1_BUSTURN_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR1_CLKDIV                    ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR1_CLKDIV_0                  ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR1_CLKDIV_1                  ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR1_CLKDIV_2                  ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR1_CLKDIV_3                  ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR1_DATLAT                    ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR1_DATLAT_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR1_DATLAT_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR1_DATLAT_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR1_DATLAT_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR1_ACCMOD                    ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR1_ACCMOD_0                  ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR1_ACCMOD_1                  ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BTR2 register  *******************/
#define  FSMC_BTR2_ADDSET                    ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR2_ADDSET_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR2_ADDSET_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR2_ADDSET_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR2_ADDSET_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR2_ADDHLD                    ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR2_ADDHLD_0                  ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR2_ADDHLD_1                  ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR2_ADDHLD_2                  ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR2_ADDHLD_3                  ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR2_DATAST                    ((uint32_t)0x0000FF00)        /*!<DATAST [7:0] bits (Data-phase duration) */
#define  FSMC_BTR2_DATAST_0                  ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR2_DATAST_1                  ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR2_DATAST_2                  ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR2_DATAST_3                  ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BTR2_DATAST_4                  ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BTR2_DATAST_5                  ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BTR2_DATAST_6                  ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BTR2_DATAST_7                  ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BTR2_BUSTURN                   ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR2_BUSTURN_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR2_BUSTURN_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR2_BUSTURN_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR2_BUSTURN_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR2_CLKDIV                    ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR2_CLKDIV_0                  ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR2_CLKDIV_1                  ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR2_CLKDIV_2                  ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR2_CLKDIV_3                  ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR2_DATLAT                    ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR2_DATLAT_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR2_DATLAT_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR2_DATLAT_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR2_DATLAT_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR2_ACCMOD                    ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR2_ACCMOD_0                  ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR2_ACCMOD_1                  ((uint32_t)0x20000000)        /*!<Bit 1 */

/*******************  Bit definition for FSMC_BTR3 register  *******************/
#define  FSMC_BTR3_ADDSET                    ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR3_ADDSET_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR3_ADDSET_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR3_ADDSET_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR3_ADDSET_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR3_ADDHLD                    ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR3_ADDHLD_0                  ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR3_ADDHLD_1                  ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR3_ADDHLD_2                  ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR3_ADDHLD_3                  ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR3_DATAST                    ((uint32_t)0x0000FF00)        /*!<DATAST [7:0] bits (Data-phase duration) */
#define  FSMC_BTR3_DATAST_0                  ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR3_DATAST_1                  ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR3_DATAST_2                  ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR3_DATAST_3                  ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BTR3_DATAST_4                  ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BTR3_DATAST_5                  ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BTR3_DATAST_6                  ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BTR3_DATAST_7                  ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BTR3_BUSTURN                   ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR3_BUSTURN_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR3_BUSTURN_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR3_BUSTURN_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR3_BUSTURN_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR3_CLKDIV                    ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR3_CLKDIV_0                  ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR3_CLKDIV_1                  ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR3_CLKDIV_2                  ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR3_CLKDIV_3                  ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR3_DATLAT                    ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR3_DATLAT_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR3_DATLAT_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR3_DATLAT_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR3_DATLAT_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR3_ACCMOD                    ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR3_ACCMOD_0                  ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR3_ACCMOD_1                  ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BTR4 register  *******************/
#define  FSMC_BTR4_ADDSET                    ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR4_ADDSET_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR4_ADDSET_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR4_ADDSET_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR4_ADDSET_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR4_ADDHLD                    ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR4_ADDHLD_0                  ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR4_ADDHLD_1                  ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR4_ADDHLD_2                  ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR4_ADDHLD_3                  ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR4_DATAST                    ((uint32_t)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR4_DATAST_0                  ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR4_DATAST_1                  ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR4_DATAST_2                  ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR4_DATAST_3                  ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BTR4_DATAST_4                  ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BTR4_DATAST_5                  ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BTR4_DATAST_6                  ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BTR4_DATAST_7                  ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BTR4_BUSTURN                   ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR4_BUSTURN_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR4_BUSTURN_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR4_BUSTURN_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR4_BUSTURN_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR4_CLKDIV                    ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR4_CLKDIV_0                  ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR4_CLKDIV_1                  ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR4_CLKDIV_2                  ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR4_CLKDIV_3                  ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR4_DATLAT                    ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR4_DATLAT_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR4_DATLAT_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR4_DATLAT_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR4_DATLAT_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR4_ACCMOD                    ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR4_ACCMOD_0                  ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR4_ACCMOD_1                  ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR1 register  ******************/
#define  FSMC_BWTR1_ADDSET                   ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR1_ADDSET_0                 ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR1_ADDSET_1                 ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR1_ADDSET_2                 ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR1_ADDSET_3                 ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR1_ADDHLD                   ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR1_ADDHLD_0                 ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR1_ADDHLD_1                 ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR1_ADDHLD_2                 ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR1_ADDHLD_3                 ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR1_DATAST                   ((uint32_t)0x0000FF00)        /*!<DATAST [7:0] bits (Data-phase duration) */
#define  FSMC_BWTR1_DATAST_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR1_DATAST_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR1_DATAST_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR1_DATAST_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BWTR1_DATAST_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BWTR1_DATAST_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BWTR1_DATAST_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BWTR1_DATAST_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BWTR1_BUSTURN                  ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR1_BUSTURN_0                ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BWTR1_BUSTURN_1                ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BWTR1_BUSTURN_2                ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BWTR1_BUSTURN_3                ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BWTR1_CLKDIV                   ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR1_CLKDIV_0                 ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR1_CLKDIV_1                 ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FSMC_BWTR1_CLKDIV_2                 ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR1_CLKDIV_3                 ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR1_DATLAT                   ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR1_DATLAT_0                 ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR1_DATLAT_1                 ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR1_DATLAT_2                 ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR1_DATLAT_3                 ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR1_ACCMOD                   ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR1_ACCMOD_0                 ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR1_ACCMOD_1                 ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR2 register  ******************/
#define  FSMC_BWTR2_ADDSET                   ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR2_ADDSET_0                 ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR2_ADDSET_1                 ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR2_ADDSET_2                 ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR2_ADDSET_3                 ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR2_ADDHLD                   ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR2_ADDHLD_0                 ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR2_ADDHLD_1                 ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR2_ADDHLD_2                 ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR2_ADDHLD_3                 ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR2_DATAST                   ((uint32_t)0x0000FF00)        /*!<DATAST [7:0] bits (Data-phase duration) */
#define  FSMC_BWTR2_DATAST_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR2_DATAST_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR2_DATAST_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR2_DATAST_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BWTR2_DATAST_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BWTR2_DATAST_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BWTR2_DATAST_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BWTR2_DATAST_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BWTR2_BUSTURN                  ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR2_BUSTURN_0                ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BWTR2_BUSTURN_1                ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BWTR2_BUSTURN_2                ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BWTR2_BUSTURN_3                ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BWTR2_CLKDIV                   ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR2_CLKDIV_0                 ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR2_CLKDIV_1                 ((uint32_t)0x00200000)        /*!<Bit 1*/
#define  FSMC_BWTR2_CLKDIV_2                 ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR2_CLKDIV_3                 ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR2_DATLAT                   ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR2_DATLAT_0                 ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR2_DATLAT_1                 ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR2_DATLAT_2                 ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR2_DATLAT_3                 ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR2_ACCMOD                   ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR2_ACCMOD_0                 ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR2_ACCMOD_1                 ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR3 register  ******************/
#define  FSMC_BWTR3_ADDSET                   ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR3_ADDSET_0                 ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR3_ADDSET_1                 ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR3_ADDSET_2                 ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR3_ADDSET_3                 ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR3_ADDHLD                   ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR3_ADDHLD_0                 ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR3_ADDHLD_1                 ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR3_ADDHLD_2                 ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR3_ADDHLD_3                 ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR3_DATAST                   ((uint32_t)0x0000FF00)        /*!<DATAST [7:0] bits (Data-phase duration) */
#define  FSMC_BWTR3_DATAST_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR3_DATAST_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR3_DATAST_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR3_DATAST_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BWTR3_DATAST_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BWTR3_DATAST_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BWTR3_DATAST_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BWTR3_DATAST_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BWTR3_BUSTURN                  ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR3_BUSTURN_0                ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BWTR3_BUSTURN_1                ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BWTR3_BUSTURN_2                ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BWTR3_BUSTURN_3                ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BWTR3_CLKDIV                   ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR3_CLKDIV_0                 ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR3_CLKDIV_1                 ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FSMC_BWTR3_CLKDIV_2                 ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR3_CLKDIV_3                 ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR3_DATLAT                   ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR3_DATLAT_0                 ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR3_DATLAT_1                 ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR3_DATLAT_2                 ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR3_DATLAT_3                 ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR3_ACCMOD                   ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR3_ACCMOD_0                 ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR3_ACCMOD_1                 ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR4 register  ******************/
#define  FSMC_BWTR4_ADDSET                   ((uint32_t)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR4_ADDSET_0                 ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR4_ADDSET_1                 ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR4_ADDSET_2                 ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR4_ADDSET_3                 ((uint32_t)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR4_ADDHLD                   ((uint32_t)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR4_ADDHLD_0                 ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR4_ADDHLD_1                 ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR4_ADDHLD_2                 ((uint32_t)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR4_ADDHLD_3                 ((uint32_t)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR4_DATAST                   ((uint32_t)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR4_DATAST_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR4_DATAST_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR4_DATAST_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR4_DATAST_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_BWTR4_DATAST_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_BWTR4_DATAST_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_BWTR4_DATAST_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_BWTR4_DATAST_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_BWTR4_BUSTURN                  ((uint32_t)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround duration) */
#define  FSMC_BWTR4_BUSTURN_0                ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_BWTR4_BUSTURN_1                ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_BWTR4_BUSTURN_2                ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_BWTR4_BUSTURN_3                ((uint32_t)0x00080000)        /*!<Bit 3 */

#define  FSMC_BWTR4_CLKDIV                   ((uint32_t)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR4_CLKDIV_0                 ((uint32_t)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR4_CLKDIV_1                 ((uint32_t)0x00200000)        /*!<Bit 1 */
#define  FSMC_BWTR4_CLKDIV_2                 ((uint32_t)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR4_CLKDIV_3                 ((uint32_t)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR4_DATLAT                   ((uint32_t)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR4_DATLAT_0                 ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR4_DATLAT_1                 ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR4_DATLAT_2                 ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR4_DATLAT_3                 ((uint32_t)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR4_ACCMOD                   ((uint32_t)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR4_ACCMOD_0                 ((uint32_t)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR4_ACCMOD_1                 ((uint32_t)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_PCR2 register  *******************/
#define  FSMC_PCR2_PWAITEN                   ((uint32_t)0x00000002)        /*!<Wait feature enable bit */
#define  FSMC_PCR2_PBKEN                     ((uint32_t)0x00000004)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR2_PTYP                      ((uint32_t)0x00000008)        /*!<Memory type */

#define  FSMC_PCR2_PWID                      ((uint32_t)0x00000030)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR2_PWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_PCR2_PWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FSMC_PCR2_ECCEN                     ((uint32_t)0x00000040)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR2_TCLR                      ((uint32_t)0x00001E00)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR2_TCLR_0                    ((uint32_t)0x00000200)        /*!<Bit 0 */
#define  FSMC_PCR2_TCLR_1                    ((uint32_t)0x00000400)        /*!<Bit 1 */
#define  FSMC_PCR2_TCLR_2                    ((uint32_t)0x00000800)        /*!<Bit 2 */
#define  FSMC_PCR2_TCLR_3                    ((uint32_t)0x00001000)        /*!<Bit 3 */

#define  FSMC_PCR2_TAR                       ((uint32_t)0x0001E000)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR2_TAR_0                     ((uint32_t)0x00002000)        /*!<Bit 0 */
#define  FSMC_PCR2_TAR_1                     ((uint32_t)0x00004000)        /*!<Bit 1 */
#define  FSMC_PCR2_TAR_2                     ((uint32_t)0x00008000)        /*!<Bit 2 */
#define  FSMC_PCR2_TAR_3                     ((uint32_t)0x00010000)        /*!<Bit 3 */

#define  FSMC_PCR2_ECCPS                     ((uint32_t)0x000E0000)        /*!<ECCPS[1:0] bits (ECC page size) */
#define  FSMC_PCR2_ECCPS_0                   ((uint32_t)0x00020000)        /*!<Bit 0 */
#define  FSMC_PCR2_ECCPS_1                   ((uint32_t)0x00040000)        /*!<Bit 1 */
#define  FSMC_PCR2_ECCPS_2                   ((uint32_t)0x00080000)        /*!<Bit 2 */

/******************  Bit definition for FSMC_PCR3 register  *******************/
#define  FSMC_PCR3_PWAITEN                   ((uint32_t)0x00000002)        /*!<Wait feature enable bit */
#define  FSMC_PCR3_PBKEN                     ((uint32_t)0x00000004)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR3_PTYP                      ((uint32_t)0x00000008)        /*!<Memory type */

#define  FSMC_PCR3_PWID                      ((uint32_t)0x00000030)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR3_PWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_PCR3_PWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FSMC_PCR3_ECCEN                     ((uint32_t)0x00000040)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR3_TCLR                      ((uint32_t)0x00001E00)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR3_TCLR_0                    ((uint32_t)0x00000200)        /*!<Bit 0 */
#define  FSMC_PCR3_TCLR_1                    ((uint32_t)0x00000400)        /*!<Bit 1 */
#define  FSMC_PCR3_TCLR_2                    ((uint32_t)0x00000800)        /*!<Bit 2 */
#define  FSMC_PCR3_TCLR_3                    ((uint32_t)0x00001000)        /*!<Bit 3 */

#define  FSMC_PCR3_TAR                       ((uint32_t)0x0001E000)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR3_TAR_0                     ((uint32_t)0x00002000)        /*!<Bit 0 */
#define  FSMC_PCR3_TAR_1                     ((uint32_t)0x00004000)        /*!<Bit 1 */
#define  FSMC_PCR3_TAR_2                     ((uint32_t)0x00008000)        /*!<Bit 2 */
#define  FSMC_PCR3_TAR_3                     ((uint32_t)0x00010000)        /*!<Bit 3 */

#define  FSMC_PCR3_ECCPS                     ((uint32_t)0x000E0000)        /*!<ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR3_ECCPS_0                   ((uint32_t)0x00020000)        /*!<Bit 0 */
#define  FSMC_PCR3_ECCPS_1                   ((uint32_t)0x00040000)        /*!<Bit 1 */
#define  FSMC_PCR3_ECCPS_2                   ((uint32_t)0x00080000)        /*!<Bit 2 */

/******************  Bit definition for FSMC_PCR4 register  *******************/
#define  FSMC_PCR4_PWAITEN                   ((uint32_t)0x00000002)        /*!<Wait feature enable bit */
#define  FSMC_PCR4_PBKEN                     ((uint32_t)0x00000004)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR4_PTYP                      ((uint32_t)0x00000008)        /*!<Memory type */

#define  FSMC_PCR4_PWID                      ((uint32_t)0x00000030)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR4_PWID_0                    ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  FSMC_PCR4_PWID_1                    ((uint32_t)0x00000020)        /*!<Bit 1 */

#define  FSMC_PCR4_ECCEN                     ((uint32_t)0x00000040)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR4_TCLR                      ((uint32_t)0x00001E00)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR4_TCLR_0                    ((uint32_t)0x00000200)        /*!<Bit 0 */
#define  FSMC_PCR4_TCLR_1                    ((uint32_t)0x00000400)        /*!<Bit 1 */
#define  FSMC_PCR4_TCLR_2                    ((uint32_t)0x00000800)        /*!<Bit 2 */
#define  FSMC_PCR4_TCLR_3                    ((uint32_t)0x00001000)        /*!<Bit 3 */

#define  FSMC_PCR4_TAR                       ((uint32_t)0x0001E000)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR4_TAR_0                     ((uint32_t)0x00002000)        /*!<Bit 0 */
#define  FSMC_PCR4_TAR_1                     ((uint32_t)0x00004000)        /*!<Bit 1 */
#define  FSMC_PCR4_TAR_2                     ((uint32_t)0x00008000)        /*!<Bit 2 */
#define  FSMC_PCR4_TAR_3                     ((uint32_t)0x00010000)        /*!<Bit 3 */

#define  FSMC_PCR4_ECCPS                     ((uint32_t)0x000E0000)        /*!<ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR4_ECCPS_0                   ((uint32_t)0x00020000)        /*!<Bit 0 */
#define  FSMC_PCR4_ECCPS_1                   ((uint32_t)0x00040000)        /*!<Bit 1 */
#define  FSMC_PCR4_ECCPS_2                   ((uint32_t)0x00080000)        /*!<Bit 2 */

/*******************  Bit definition for FSMC_SR2 register  *******************/
#define  FSMC_SR2_IRS                        ((uint32_t)0x01)               /*!<Interrupt Rising Edge status                */
#define  FSMC_SR2_ILS                        ((uint32_t)0x02)               /*!<Interrupt Level status                      */
#define  FSMC_SR2_IFS                        ((uint32_t)0x04)               /*!<Interrupt Falling Edge status               */
#define  FSMC_SR2_IREN                       ((uint32_t)0x08)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FSMC_SR2_ILEN                       ((uint32_t)0x10)               /*!<Interrupt Level detection Enable bit        */
#define  FSMC_SR2_IFEN                       ((uint32_t)0x20)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR2_FEMPT                      ((uint32_t)0x40)               /*!<FIFO empty */

/*******************  Bit definition for FSMC_SR3 register  *******************/
#define  FSMC_SR3_IRS                        ((uint32_t)0x01)               /*!<Interrupt Rising Edge status                */
#define  FSMC_SR3_ILS                        ((uint32_t)0x02)               /*!<Interrupt Level status                      */
#define  FSMC_SR3_IFS                        ((uint32_t)0x04)               /*!<Interrupt Falling Edge status               */
#define  FSMC_SR3_IREN                       ((uint32_t)0x08)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FSMC_SR3_ILEN                       ((uint32_t)0x10)               /*!<Interrupt Level detection Enable bit        */
#define  FSMC_SR3_IFEN                       ((uint32_t)0x20)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR3_FEMPT                      ((uint32_t)0x40)               /*!<FIFO empty */

/*******************  Bit definition for FSMC_SR4 register  *******************/
#define  FSMC_SR4_IRS                        ((uint32_t)0x01)               /*!<Interrupt Rising Edge status                 */
#define  FSMC_SR4_ILS                        ((uint32_t)0x02)               /*!<Interrupt Level status                       */
#define  FSMC_SR4_IFS                        ((uint32_t)0x04)               /*!<Interrupt Falling Edge status                */
#define  FSMC_SR4_IREN                       ((uint32_t)0x08)               /*!<Interrupt Rising Edge detection Enable bit   */
#define  FSMC_SR4_ILEN                       ((uint32_t)0x10)               /*!<Interrupt Level detection Enable bit         */
#define  FSMC_SR4_IFEN                       ((uint32_t)0x20)               /*!<Interrupt Falling Edge detection Enable bit  */
#define  FSMC_SR4_FEMPT                      ((uint32_t)0x40)               /*!<FIFO empty */

/******************  Bit definition for FSMC_PMEM2 register  ******************/
#define  FSMC_PMEM2_MEMSET2                  ((uint32_t)0x000000FF)        /*!<MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  FSMC_PMEM2_MEMSET2_0                ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMSET2_1                ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMSET2_2                ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMSET2_3                ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMSET2_4                ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMSET2_5                ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMSET2_6                ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMSET2_7                ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMWAIT2                 ((uint32_t)0x0000FF00)        /*!<MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  FSMC_PMEM2_MEMWAIT2_0               ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMWAIT2_1               ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMWAIT2_2               ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMWAIT2_3               ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMWAIT2_4               ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMWAIT2_5               ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMWAIT2_6               ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMWAIT2_7               ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMHOLD2                 ((uint32_t)0x00FF0000)        /*!<MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  FSMC_PMEM2_MEMHOLD2_0               ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMHOLD2_1               ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMHOLD2_2               ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMHOLD2_3               ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMHOLD2_4               ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMHOLD2_5               ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMHOLD2_6               ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMHOLD2_7               ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMHIZ2                  ((uint32_t)0xFF000000)        /*!<MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  FSMC_PMEM2_MEMHIZ2_0                ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMHIZ2_1                ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMHIZ2_2                ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMHIZ2_3                ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMHIZ2_4                ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMHIZ2_5                ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMHIZ2_6                ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMHIZ2_7                ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PMEM3 register  ******************/
#define  FSMC_PMEM3_MEMSET3                  ((uint32_t)0x000000FF)        /*!<MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  FSMC_PMEM3_MEMSET3_0                ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMSET3_1                ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMSET3_2                ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMSET3_3                ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMSET3_4                ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMSET3_5                ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMSET3_6                ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMSET3_7                ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMWAIT3                 ((uint32_t)0x0000FF00)        /*!<MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  FSMC_PMEM3_MEMWAIT3_0               ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMWAIT3_1               ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMWAIT3_2               ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMWAIT3_3               ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMWAIT3_4               ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMWAIT3_5               ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMWAIT3_6               ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMWAIT3_7               ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMHOLD3                 ((uint32_t)0x00FF0000)        /*!<MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  FSMC_PMEM3_MEMHOLD3_0               ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMHOLD3_1               ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMHOLD3_2               ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMHOLD3_3               ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMHOLD3_4               ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMHOLD3_5               ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMHOLD3_6               ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMHOLD3_7               ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMHIZ3                  ((uint32_t)0xFF000000)        /*!<MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  FSMC_PMEM3_MEMHIZ3_0                ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMHIZ3_1                ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMHIZ3_2                ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMHIZ3_3                ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMHIZ3_4                ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMHIZ3_5                ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMHIZ3_6                ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMHIZ3_7                ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PMEM4 register  ******************/
#define  FSMC_PMEM4_MEMSET4                  ((uint32_t)0x000000FF)        /*!<MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  FSMC_PMEM4_MEMSET4_0                ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMSET4_1                ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMSET4_2                ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMSET4_3                ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMSET4_4                ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMSET4_5                ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMSET4_6                ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMSET4_7                ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMWAIT4                 ((uint32_t)0x0000FF00)        /*!<MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  FSMC_PMEM4_MEMWAIT4_0               ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMWAIT4_1               ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMWAIT4_2               ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMWAIT4_3               ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMWAIT4_4               ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMWAIT4_5               ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMWAIT4_6               ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMWAIT4_7               ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMHOLD4                 ((uint32_t)0x00FF0000)        /*!<MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  FSMC_PMEM4_MEMHOLD4_0               ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMHOLD4_1               ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMHOLD4_2               ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMHOLD4_3               ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMHOLD4_4               ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMHOLD4_5               ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMHOLD4_6               ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMHOLD4_7               ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMHIZ4                  ((uint32_t)0xFF000000)        /*!<MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  FSMC_PMEM4_MEMHIZ4_0                ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMHIZ4_1                ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMHIZ4_2                ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMHIZ4_3                ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMHIZ4_4                ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMHIZ4_5                ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMHIZ4_6                ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMHIZ4_7                ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT2 register  ******************/
#define  FSMC_PATT2_ATTSET2                  ((uint32_t)0x000000FF)        /*!<ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  FSMC_PATT2_ATTSET2_0                ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTSET2_1                ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTSET2_2                ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTSET2_3                ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTSET2_4                ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTSET2_5                ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTSET2_6                ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTSET2_7                ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTWAIT2                 ((uint32_t)0x0000FF00)        /*!<ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  FSMC_PATT2_ATTWAIT2_0               ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTWAIT2_1               ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTWAIT2_2               ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTWAIT2_3               ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTWAIT2_4               ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTWAIT2_5               ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTWAIT2_6               ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTWAIT2_7               ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTHOLD2                 ((uint32_t)0x00FF0000)        /*!<ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  FSMC_PATT2_ATTHOLD2_0               ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTHOLD2_1               ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTHOLD2_2               ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTHOLD2_3               ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTHOLD2_4               ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTHOLD2_5               ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTHOLD2_6               ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTHOLD2_7               ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTHIZ2                  ((uint32_t)0xFF000000)        /*!<ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  FSMC_PATT2_ATTHIZ2_0                ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTHIZ2_1                ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTHIZ2_2                ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTHIZ2_3                ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTHIZ2_4                ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTHIZ2_5                ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTHIZ2_6                ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTHIZ2_7                ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT3 register  ******************/
#define  FSMC_PATT3_ATTSET3                  ((uint32_t)0x000000FF)        /*!<ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  FSMC_PATT3_ATTSET3_0                ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTSET3_1                ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTSET3_2                ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTSET3_3                ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTSET3_4                ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTSET3_5                ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTSET3_6                ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTSET3_7                ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTWAIT3                 ((uint32_t)0x0000FF00)        /*!<ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  FSMC_PATT3_ATTWAIT3_0               ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTWAIT3_1               ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTWAIT3_2               ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTWAIT3_3               ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTWAIT3_4               ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTWAIT3_5               ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTWAIT3_6               ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTWAIT3_7               ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTHOLD3                 ((uint32_t)0x00FF0000)        /*!<ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  FSMC_PATT3_ATTHOLD3_0               ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTHOLD3_1               ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTHOLD3_2               ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTHOLD3_3               ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTHOLD3_4               ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTHOLD3_5               ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTHOLD3_6               ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTHOLD3_7               ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTHIZ3                  ((uint32_t)0xFF000000)        /*!<ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  FSMC_PATT3_ATTHIZ3_0                ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTHIZ3_1                ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTHIZ3_2                ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTHIZ3_3                ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTHIZ3_4                ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTHIZ3_5                ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTHIZ3_6                ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTHIZ3_7                ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT4 register  ******************/
#define  FSMC_PATT4_ATTSET4                  ((uint32_t)0x000000FF)        /*!<ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  FSMC_PATT4_ATTSET4_0                ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTSET4_1                ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTSET4_2                ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTSET4_3                ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTSET4_4                ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTSET4_5                ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTSET4_6                ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTSET4_7                ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTWAIT4                 ((uint32_t)0x0000FF00)        /*!<ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  FSMC_PATT4_ATTWAIT4_0               ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTWAIT4_1               ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTWAIT4_2               ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTWAIT4_3               ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTWAIT4_4               ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTWAIT4_5               ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTWAIT4_6               ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTWAIT4_7               ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTHOLD4                 ((uint32_t)0x00FF0000)        /*!<ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  FSMC_PATT4_ATTHOLD4_0               ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTHOLD4_1               ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTHOLD4_2               ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTHOLD4_3               ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTHOLD4_4               ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTHOLD4_5               ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTHOLD4_6               ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTHOLD4_7               ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTHIZ4                  ((uint32_t)0xFF000000)        /*!<ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  FSMC_PATT4_ATTHIZ4_0                ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTHIZ4_1                ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTHIZ4_2                ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTHIZ4_3                ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTHIZ4_4                ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTHIZ4_5                ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTHIZ4_6                ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTHIZ4_7                ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PIO4 register  *******************/
#define  FSMC_PIO4_IOSET4                    ((uint32_t)0x000000FF)        /*!<IOSET4[7:0] bits (I/O 4 setup time) */
#define  FSMC_PIO4_IOSET4_0                  ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  FSMC_PIO4_IOSET4_1                  ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  FSMC_PIO4_IOSET4_2                  ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  FSMC_PIO4_IOSET4_3                  ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  FSMC_PIO4_IOSET4_4                  ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  FSMC_PIO4_IOSET4_5                  ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  FSMC_PIO4_IOSET4_6                  ((uint32_t)0x00000040)        /*!<Bit 6 */
#define  FSMC_PIO4_IOSET4_7                  ((uint32_t)0x00000080)        /*!<Bit 7 */

#define  FSMC_PIO4_IOWAIT4                   ((uint32_t)0x0000FF00)        /*!<IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  FSMC_PIO4_IOWAIT4_0                 ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  FSMC_PIO4_IOWAIT4_1                 ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  FSMC_PIO4_IOWAIT4_2                 ((uint32_t)0x00000400)        /*!<Bit 2 */
#define  FSMC_PIO4_IOWAIT4_3                 ((uint32_t)0x00000800)        /*!<Bit 3 */
#define  FSMC_PIO4_IOWAIT4_4                 ((uint32_t)0x00001000)        /*!<Bit 4 */
#define  FSMC_PIO4_IOWAIT4_5                 ((uint32_t)0x00002000)        /*!<Bit 5 */
#define  FSMC_PIO4_IOWAIT4_6                 ((uint32_t)0x00004000)        /*!<Bit 6 */
#define  FSMC_PIO4_IOWAIT4_7                 ((uint32_t)0x00008000)        /*!<Bit 7 */

#define  FSMC_PIO4_IOHOLD4                   ((uint32_t)0x00FF0000)        /*!<IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  FSMC_PIO4_IOHOLD4_0                 ((uint32_t)0x00010000)        /*!<Bit 0 */
#define  FSMC_PIO4_IOHOLD4_1                 ((uint32_t)0x00020000)        /*!<Bit 1 */
#define  FSMC_PIO4_IOHOLD4_2                 ((uint32_t)0x00040000)        /*!<Bit 2 */
#define  FSMC_PIO4_IOHOLD4_3                 ((uint32_t)0x00080000)        /*!<Bit 3 */
#define  FSMC_PIO4_IOHOLD4_4                 ((uint32_t)0x00100000)        /*!<Bit 4 */
#define  FSMC_PIO4_IOHOLD4_5                 ((uint32_t)0x00200000)        /*!<Bit 5 */
#define  FSMC_PIO4_IOHOLD4_6                 ((uint32_t)0x00400000)        /*!<Bit 6 */
#define  FSMC_PIO4_IOHOLD4_7                 ((uint32_t)0x00800000)        /*!<Bit 7 */

#define  FSMC_PIO4_IOHIZ4                    ((uint32_t)0xFF000000)        /*!<IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  FSMC_PIO4_IOHIZ4_0                  ((uint32_t)0x01000000)        /*!<Bit 0 */
#define  FSMC_PIO4_IOHIZ4_1                  ((uint32_t)0x02000000)        /*!<Bit 1 */
#define  FSMC_PIO4_IOHIZ4_2                  ((uint32_t)0x04000000)        /*!<Bit 2 */
#define  FSMC_PIO4_IOHIZ4_3                  ((uint32_t)0x08000000)        /*!<Bit 3 */
#define  FSMC_PIO4_IOHIZ4_4                  ((uint32_t)0x10000000)        /*!<Bit 4 */
#define  FSMC_PIO4_IOHIZ4_5                  ((uint32_t)0x20000000)        /*!<Bit 5 */
#define  FSMC_PIO4_IOHIZ4_6                  ((uint32_t)0x40000000)        /*!<Bit 6 */
#define  FSMC_PIO4_IOHIZ4_7                  ((uint32_t)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_ECCR2 register  ******************/
#define  FSMC_ECCR2_ECC2                     ((uint32_t)0xFFFFFFFF)        /*!<ECC result */

/******************  Bit definition for FSMC_ECCR3 register  ******************/
#define  FSMC_ECCR3_ECC3                     ((uint32_t)0xFFFFFFFF)        /*!<ECC result */

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0                    ((uint32_t)0x00000003)
#define GPIO_MODER_MODER0_0                  ((uint32_t)0x00000001)
#define GPIO_MODER_MODER0_1                  ((uint32_t)0x00000002)

#define GPIO_MODER_MODER1                    ((uint32_t)0x0000000C)
#define GPIO_MODER_MODER1_0                  ((uint32_t)0x00000004)
#define GPIO_MODER_MODER1_1                  ((uint32_t)0x00000008)

#define GPIO_MODER_MODER2                    ((uint32_t)0x00000030)
#define GPIO_MODER_MODER2_0                  ((uint32_t)0x00000010)
#define GPIO_MODER_MODER2_1                  ((uint32_t)0x00000020)

#define GPIO_MODER_MODER3                    ((uint32_t)0x000000C0)
#define GPIO_MODER_MODER3_0                  ((uint32_t)0x00000040)
#define GPIO_MODER_MODER3_1                  ((uint32_t)0x00000080)

#define GPIO_MODER_MODER4                    ((uint32_t)0x00000300)
#define GPIO_MODER_MODER4_0                  ((uint32_t)0x00000100)
#define GPIO_MODER_MODER4_1                  ((uint32_t)0x00000200)

#define GPIO_MODER_MODER5                    ((uint32_t)0x00000C00)
#define GPIO_MODER_MODER5_0                  ((uint32_t)0x00000400)
#define GPIO_MODER_MODER5_1                  ((uint32_t)0x00000800)

#define GPIO_MODER_MODER6                    ((uint32_t)0x00003000)
#define GPIO_MODER_MODER6_0                  ((uint32_t)0x00001000)
#define GPIO_MODER_MODER6_1                  ((uint32_t)0x00002000)

#define GPIO_MODER_MODER7                    ((uint32_t)0x0000C000)
#define GPIO_MODER_MODER7_0                  ((uint32_t)0x00004000)
#define GPIO_MODER_MODER7_1                  ((uint32_t)0x00008000)

#define GPIO_MODER_MODER8                    ((uint32_t)0x00030000)
#define GPIO_MODER_MODER8_0                  ((uint32_t)0x00010000)
#define GPIO_MODER_MODER8_1                  ((uint32_t)0x00020000)

#define GPIO_MODER_MODER9                    ((uint32_t)0x000C0000)
#define GPIO_MODER_MODER9_0                  ((uint32_t)0x00040000)
#define GPIO_MODER_MODER9_1                  ((uint32_t)0x00080000)

#define GPIO_MODER_MODER10                   ((uint32_t)0x00300000)
#define GPIO_MODER_MODER10_0                 ((uint32_t)0x00100000)
#define GPIO_MODER_MODER10_1                 ((uint32_t)0x00200000)

#define GPIO_MODER_MODER11                   ((uint32_t)0x00C00000)
#define GPIO_MODER_MODER11_0                 ((uint32_t)0x00400000)
#define GPIO_MODER_MODER11_1                 ((uint32_t)0x00800000)

#define GPIO_MODER_MODER12                   ((uint32_t)0x03000000)
#define GPIO_MODER_MODER12_0                 ((uint32_t)0x01000000)
#define GPIO_MODER_MODER12_1                 ((uint32_t)0x02000000)

#define GPIO_MODER_MODER13                   ((uint32_t)0x0C000000)
#define GPIO_MODER_MODER13_0                 ((uint32_t)0x04000000)
#define GPIO_MODER_MODER13_1                 ((uint32_t)0x08000000)

#define GPIO_MODER_MODER14                   ((uint32_t)0x30000000)
#define GPIO_MODER_MODER14_0                 ((uint32_t)0x10000000)
#define GPIO_MODER_MODER14_1                 ((uint32_t)0x20000000)

#define GPIO_MODER_MODER15                   ((uint32_t)0xC0000000)
#define GPIO_MODER_MODER15_0                 ((uint32_t)0x40000000)
#define GPIO_MODER_MODER15_1                 ((uint32_t)0x80000000)

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT_0                     ((uint32_t)0x00000001)
#define GPIO_OTYPER_OT_1                     ((uint32_t)0x00000002)
#define GPIO_OTYPER_OT_2                     ((uint32_t)0x00000004)
#define GPIO_OTYPER_OT_3                     ((uint32_t)0x00000008)
#define GPIO_OTYPER_OT_4                     ((uint32_t)0x00000010)
#define GPIO_OTYPER_OT_5                     ((uint32_t)0x00000020)
#define GPIO_OTYPER_OT_6                     ((uint32_t)0x00000040)
#define GPIO_OTYPER_OT_7                     ((uint32_t)0x00000080)
#define GPIO_OTYPER_OT_8                     ((uint32_t)0x00000100)
#define GPIO_OTYPER_OT_9                     ((uint32_t)0x00000200)
#define GPIO_OTYPER_OT_10                    ((uint32_t)0x00000400)
#define GPIO_OTYPER_OT_11                    ((uint32_t)0x00000800)
#define GPIO_OTYPER_OT_12                    ((uint32_t)0x00001000)
#define GPIO_OTYPER_OT_13                    ((uint32_t)0x00002000)
#define GPIO_OTYPER_OT_14                    ((uint32_t)0x00004000)
#define GPIO_OTYPER_OT_15                    ((uint32_t)0x00008000)

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDER_OSPEEDR0               ((uint32_t)0x00000003)
#define GPIO_OSPEEDER_OSPEEDR0_0             ((uint32_t)0x00000001)
#define GPIO_OSPEEDER_OSPEEDR0_1             ((uint32_t)0x00000002)

#define GPIO_OSPEEDER_OSPEEDR1               ((uint32_t)0x0000000C)
#define GPIO_OSPEEDER_OSPEEDR1_0             ((uint32_t)0x00000004)
#define GPIO_OSPEEDER_OSPEEDR1_1             ((uint32_t)0x00000008)

#define GPIO_OSPEEDER_OSPEEDR2               ((uint32_t)0x00000030)
#define GPIO_OSPEEDER_OSPEEDR2_0             ((uint32_t)0x00000010)
#define GPIO_OSPEEDER_OSPEEDR2_1             ((uint32_t)0x00000020)

#define GPIO_OSPEEDER_OSPEEDR3               ((uint32_t)0x000000C0)
#define GPIO_OSPEEDER_OSPEEDR3_0             ((uint32_t)0x00000040)
#define GPIO_OSPEEDER_OSPEEDR3_1             ((uint32_t)0x00000080)

#define GPIO_OSPEEDER_OSPEEDR4               ((uint32_t)0x00000300)
#define GPIO_OSPEEDER_OSPEEDR4_0             ((uint32_t)0x00000100)
#define GPIO_OSPEEDER_OSPEEDR4_1             ((uint32_t)0x00000200)

#define GPIO_OSPEEDER_OSPEEDR5               ((uint32_t)0x00000C00)
#define GPIO_OSPEEDER_OSPEEDR5_0             ((uint32_t)0x00000400)
#define GPIO_OSPEEDER_OSPEEDR5_1             ((uint32_t)0x00000800)

#define GPIO_OSPEEDER_OSPEEDR6               ((uint32_t)0x00003000)
#define GPIO_OSPEEDER_OSPEEDR6_0             ((uint32_t)0x00001000)
#define GPIO_OSPEEDER_OSPEEDR6_1             ((uint32_t)0x00002000)

#define GPIO_OSPEEDER_OSPEEDR7               ((uint32_t)0x0000C000)
#define GPIO_OSPEEDER_OSPEEDR7_0             ((uint32_t)0x00004000)
#define GPIO_OSPEEDER_OSPEEDR7_1             ((uint32_t)0x00008000)

#define GPIO_OSPEEDER_OSPEEDR8               ((uint32_t)0x00030000)
#define GPIO_OSPEEDER_OSPEEDR8_0             ((uint32_t)0x00010000)
#define GPIO_OSPEEDER_OSPEEDR8_1             ((uint32_t)0x00020000)

#define GPIO_OSPEEDER_OSPEEDR9               ((uint32_t)0x000C0000)
#define GPIO_OSPEEDER_OSPEEDR9_0             ((uint32_t)0x00040000)
#define GPIO_OSPEEDER_OSPEEDR9_1             ((uint32_t)0x00080000)

#define GPIO_OSPEEDER_OSPEEDR10              ((uint32_t)0x00300000)
#define GPIO_OSPEEDER_OSPEEDR10_0            ((uint32_t)0x00100000)
#define GPIO_OSPEEDER_OSPEEDR10_1            ((uint32_t)0x00200000)

#define GPIO_OSPEEDER_OSPEEDR11              ((uint32_t)0x00C00000)
#define GPIO_OSPEEDER_OSPEEDR11_0            ((uint32_t)0x00400000)
#define GPIO_OSPEEDER_OSPEEDR11_1            ((uint32_t)0x00800000)

#define GPIO_OSPEEDER_OSPEEDR12              ((uint32_t)0x03000000)
#define GPIO_OSPEEDER_OSPEEDR12_0            ((uint32_t)0x01000000)
#define GPIO_OSPEEDER_OSPEEDR12_1            ((uint32_t)0x02000000)

#define GPIO_OSPEEDER_OSPEEDR13              ((uint32_t)0x0C000000)
#define GPIO_OSPEEDER_OSPEEDR13_0            ((uint32_t)0x04000000)
#define GPIO_OSPEEDER_OSPEEDR13_1            ((uint32_t)0x08000000)

#define GPIO_OSPEEDER_OSPEEDR14              ((uint32_t)0x30000000)
#define GPIO_OSPEEDER_OSPEEDR14_0            ((uint32_t)0x10000000)
#define GPIO_OSPEEDER_OSPEEDR14_1            ((uint32_t)0x20000000)

#define GPIO_OSPEEDER_OSPEEDR15              ((uint32_t)0xC0000000)
#define GPIO_OSPEEDER_OSPEEDR15_0            ((uint32_t)0x40000000)
#define GPIO_OSPEEDER_OSPEEDR15_1            ((uint32_t)0x80000000)

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPDR0                    ((uint32_t)0x00000003)
#define GPIO_PUPDR_PUPDR0_0                  ((uint32_t)0x00000001)
#define GPIO_PUPDR_PUPDR0_1                  ((uint32_t)0x00000002)

#define GPIO_PUPDR_PUPDR1                    ((uint32_t)0x0000000C)
#define GPIO_PUPDR_PUPDR1_0                  ((uint32_t)0x00000004)
#define GPIO_PUPDR_PUPDR1_1                  ((uint32_t)0x00000008)

#define GPIO_PUPDR_PUPDR2                    ((uint32_t)0x00000030)
#define GPIO_PUPDR_PUPDR2_0                  ((uint32_t)0x00000010)
#define GPIO_PUPDR_PUPDR2_1                  ((uint32_t)0x00000020)

#define GPIO_PUPDR_PUPDR3                    ((uint32_t)0x000000C0)
#define GPIO_PUPDR_PUPDR3_0                  ((uint32_t)0x00000040)
#define GPIO_PUPDR_PUPDR3_1                  ((uint32_t)0x00000080)

#define GPIO_PUPDR_PUPDR4                    ((uint32_t)0x00000300)
#define GPIO_PUPDR_PUPDR4_0                  ((uint32_t)0x00000100)
#define GPIO_PUPDR_PUPDR4_1                  ((uint32_t)0x00000200)

#define GPIO_PUPDR_PUPDR5                    ((uint32_t)0x00000C00)
#define GPIO_PUPDR_PUPDR5_0                  ((uint32_t)0x00000400)
#define GPIO_PUPDR_PUPDR5_1                  ((uint32_t)0x00000800)

#define GPIO_PUPDR_PUPDR6                    ((uint32_t)0x00003000)
#define GPIO_PUPDR_PUPDR6_0                  ((uint32_t)0x00001000)
#define GPIO_PUPDR_PUPDR6_1                  ((uint32_t)0x00002000)

#define GPIO_PUPDR_PUPDR7                    ((uint32_t)0x0000C000)
#define GPIO_PUPDR_PUPDR7_0                  ((uint32_t)0x00004000)
#define GPIO_PUPDR_PUPDR7_1                  ((uint32_t)0x00008000)

#define GPIO_PUPDR_PUPDR8                    ((uint32_t)0x00030000)
#define GPIO_PUPDR_PUPDR8_0                  ((uint32_t)0x00010000)
#define GPIO_PUPDR_PUPDR8_1                  ((uint32_t)0x00020000)

#define GPIO_PUPDR_PUPDR9                    ((uint32_t)0x000C0000)
#define GPIO_PUPDR_PUPDR9_0                  ((uint32_t)0x00040000)
#define GPIO_PUPDR_PUPDR9_1                  ((uint32_t)0x00080000)

#define GPIO_PUPDR_PUPDR10                   ((uint32_t)0x00300000)
#define GPIO_PUPDR_PUPDR10_0                 ((uint32_t)0x00100000)
#define GPIO_PUPDR_PUPDR10_1                 ((uint32_t)0x00200000)

#define GPIO_PUPDR_PUPDR11                   ((uint32_t)0x00C00000)
#define GPIO_PUPDR_PUPDR11_0                 ((uint32_t)0x00400000)
#define GPIO_PUPDR_PUPDR11_1                 ((uint32_t)0x00800000)

#define GPIO_PUPDR_PUPDR12                   ((uint32_t)0x03000000)
#define GPIO_PUPDR_PUPDR12_0                 ((uint32_t)0x01000000)
#define GPIO_PUPDR_PUPDR12_1                 ((uint32_t)0x02000000)

#define GPIO_PUPDR_PUPDR13                   ((uint32_t)0x0C000000)
#define GPIO_PUPDR_PUPDR13_0                 ((uint32_t)0x04000000)
#define GPIO_PUPDR_PUPDR13_1                 ((uint32_t)0x08000000)

#define GPIO_PUPDR_PUPDR14                   ((uint32_t)0x30000000)
#define GPIO_PUPDR_PUPDR14_0                 ((uint32_t)0x10000000)
#define GPIO_PUPDR_PUPDR14_1                 ((uint32_t)0x20000000)

#define GPIO_PUPDR_PUPDR15                   ((uint32_t)0xC0000000)
#define GPIO_PUPDR_PUPDR15_0                 ((uint32_t)0x40000000)
#define GPIO_PUPDR_PUPDR15_1                 ((uint32_t)0x80000000)

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR_0                       ((uint32_t)0x00000001)
#define GPIO_IDR_IDR_1                       ((uint32_t)0x00000002)
#define GPIO_IDR_IDR_2                       ((uint32_t)0x00000004)
#define GPIO_IDR_IDR_3                       ((uint32_t)0x00000008)
#define GPIO_IDR_IDR_4                       ((uint32_t)0x00000010)
#define GPIO_IDR_IDR_5                       ((uint32_t)0x00000020)
#define GPIO_IDR_IDR_6                       ((uint32_t)0x00000040)
#define GPIO_IDR_IDR_7                       ((uint32_t)0x00000080)
#define GPIO_IDR_IDR_8                       ((uint32_t)0x00000100)
#define GPIO_IDR_IDR_9                       ((uint32_t)0x00000200)
#define GPIO_IDR_IDR_10                      ((uint32_t)0x00000400)
#define GPIO_IDR_IDR_11                      ((uint32_t)0x00000800)
#define GPIO_IDR_IDR_12                      ((uint32_t)0x00001000)
#define GPIO_IDR_IDR_13                      ((uint32_t)0x00002000)
#define GPIO_IDR_IDR_14                      ((uint32_t)0x00004000)
#define GPIO_IDR_IDR_15                      ((uint32_t)0x00008000)
/* Old GPIO_IDR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_IDR_0                    GPIO_IDR_IDR_0
#define GPIO_OTYPER_IDR_1                    GPIO_IDR_IDR_1
#define GPIO_OTYPER_IDR_2                    GPIO_IDR_IDR_2
#define GPIO_OTYPER_IDR_3                    GPIO_IDR_IDR_3
#define GPIO_OTYPER_IDR_4                    GPIO_IDR_IDR_4
#define GPIO_OTYPER_IDR_5                    GPIO_IDR_IDR_5
#define GPIO_OTYPER_IDR_6                    GPIO_IDR_IDR_6
#define GPIO_OTYPER_IDR_7                    GPIO_IDR_IDR_7
#define GPIO_OTYPER_IDR_8                    GPIO_IDR_IDR_8
#define GPIO_OTYPER_IDR_9                    GPIO_IDR_IDR_9
#define GPIO_OTYPER_IDR_10                   GPIO_IDR_IDR_10
#define GPIO_OTYPER_IDR_11                   GPIO_IDR_IDR_11
#define GPIO_OTYPER_IDR_12                   GPIO_IDR_IDR_12
#define GPIO_OTYPER_IDR_13                   GPIO_IDR_IDR_13
#define GPIO_OTYPER_IDR_14                   GPIO_IDR_IDR_14
#define GPIO_OTYPER_IDR_15                   GPIO_IDR_IDR_15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR_0                       ((uint32_t)0x00000001)
#define GPIO_ODR_ODR_1                       ((uint32_t)0x00000002)
#define GPIO_ODR_ODR_2                       ((uint32_t)0x00000004)
#define GPIO_ODR_ODR_3                       ((uint32_t)0x00000008)
#define GPIO_ODR_ODR_4                       ((uint32_t)0x00000010)
#define GPIO_ODR_ODR_5                       ((uint32_t)0x00000020)
#define GPIO_ODR_ODR_6                       ((uint32_t)0x00000040)
#define GPIO_ODR_ODR_7                       ((uint32_t)0x00000080)
#define GPIO_ODR_ODR_8                       ((uint32_t)0x00000100)
#define GPIO_ODR_ODR_9                       ((uint32_t)0x00000200)
#define GPIO_ODR_ODR_10                      ((uint32_t)0x00000400)
#define GPIO_ODR_ODR_11                      ((uint32_t)0x00000800)
#define GPIO_ODR_ODR_12                      ((uint32_t)0x00001000)
#define GPIO_ODR_ODR_13                      ((uint32_t)0x00002000)
#define GPIO_ODR_ODR_14                      ((uint32_t)0x00004000)
#define GPIO_ODR_ODR_15                      ((uint32_t)0x00008000)
/* Old GPIO_ODR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_ODR_0                    GPIO_ODR_ODR_0
#define GPIO_OTYPER_ODR_1                    GPIO_ODR_ODR_1
#define GPIO_OTYPER_ODR_2                    GPIO_ODR_ODR_2
#define GPIO_OTYPER_ODR_3                    GPIO_ODR_ODR_3
#define GPIO_OTYPER_ODR_4                    GPIO_ODR_ODR_4
#define GPIO_OTYPER_ODR_5                    GPIO_ODR_ODR_5
#define GPIO_OTYPER_ODR_6                    GPIO_ODR_ODR_6
#define GPIO_OTYPER_ODR_7                    GPIO_ODR_ODR_7
#define GPIO_OTYPER_ODR_8                    GPIO_ODR_ODR_8
#define GPIO_OTYPER_ODR_9                    GPIO_ODR_ODR_9
#define GPIO_OTYPER_ODR_10                   GPIO_ODR_ODR_10
#define GPIO_OTYPER_ODR_11                   GPIO_ODR_ODR_11
#define GPIO_OTYPER_ODR_12                   GPIO_ODR_ODR_12
#define GPIO_OTYPER_ODR_13                   GPIO_ODR_ODR_13
#define GPIO_OTYPER_ODR_14                   GPIO_ODR_ODR_14
#define GPIO_OTYPER_ODR_15                   GPIO_ODR_ODR_15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS_0                       ((uint32_t)0x00000001)
#define GPIO_BSRR_BS_1                       ((uint32_t)0x00000002)
#define GPIO_BSRR_BS_2                       ((uint32_t)0x00000004)
#define GPIO_BSRR_BS_3                       ((uint32_t)0x00000008)
#define GPIO_BSRR_BS_4                       ((uint32_t)0x00000010)
#define GPIO_BSRR_BS_5                       ((uint32_t)0x00000020)
#define GPIO_BSRR_BS_6                       ((uint32_t)0x00000040)
#define GPIO_BSRR_BS_7                       ((uint32_t)0x00000080)
#define GPIO_BSRR_BS_8                       ((uint32_t)0x00000100)
#define GPIO_BSRR_BS_9                       ((uint32_t)0x00000200)
#define GPIO_BSRR_BS_10                      ((uint32_t)0x00000400)
#define GPIO_BSRR_BS_11                      ((uint32_t)0x00000800)
#define GPIO_BSRR_BS_12                      ((uint32_t)0x00001000)
#define GPIO_BSRR_BS_13                      ((uint32_t)0x00002000)
#define GPIO_BSRR_BS_14                      ((uint32_t)0x00004000)
#define GPIO_BSRR_BS_15                      ((uint32_t)0x00008000)
#define GPIO_BSRR_BR_0                       ((uint32_t)0x00010000)
#define GPIO_BSRR_BR_1                       ((uint32_t)0x00020000)
#define GPIO_BSRR_BR_2                       ((uint32_t)0x00040000)
#define GPIO_BSRR_BR_3                       ((uint32_t)0x00080000)
#define GPIO_BSRR_BR_4                       ((uint32_t)0x00100000)
#define GPIO_BSRR_BR_5                       ((uint32_t)0x00200000)
#define GPIO_BSRR_BR_6                       ((uint32_t)0x00400000)
#define GPIO_BSRR_BR_7                       ((uint32_t)0x00800000)
#define GPIO_BSRR_BR_8                       ((uint32_t)0x01000000)
#define GPIO_BSRR_BR_9                       ((uint32_t)0x02000000)
#define GPIO_BSRR_BR_10                      ((uint32_t)0x04000000)
#define GPIO_BSRR_BR_11                      ((uint32_t)0x08000000)
#define GPIO_BSRR_BR_12                      ((uint32_t)0x10000000)
#define GPIO_BSRR_BR_13                      ((uint32_t)0x20000000)
#define GPIO_BSRR_BR_14                      ((uint32_t)0x40000000)
#define GPIO_BSRR_BR_15                      ((uint32_t)0x80000000)

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0                       ((uint32_t)0x00000001)
#define GPIO_LCKR_LCK1                       ((uint32_t)0x00000002)
#define GPIO_LCKR_LCK2                       ((uint32_t)0x00000004)
#define GPIO_LCKR_LCK3                       ((uint32_t)0x00000008)
#define GPIO_LCKR_LCK4                       ((uint32_t)0x00000010)
#define GPIO_LCKR_LCK5                       ((uint32_t)0x00000020)
#define GPIO_LCKR_LCK6                       ((uint32_t)0x00000040)
#define GPIO_LCKR_LCK7                       ((uint32_t)0x00000080)
#define GPIO_LCKR_LCK8                       ((uint32_t)0x00000100)
#define GPIO_LCKR_LCK9                       ((uint32_t)0x00000200)
#define GPIO_LCKR_LCK10                      ((uint32_t)0x00000400)
#define GPIO_LCKR_LCK11                      ((uint32_t)0x00000800)
#define GPIO_LCKR_LCK12                      ((uint32_t)0x00001000)
#define GPIO_LCKR_LCK13                      ((uint32_t)0x00002000)
#define GPIO_LCKR_LCK14                      ((uint32_t)0x00004000)
#define GPIO_LCKR_LCK15                      ((uint32_t)0x00008000)
#define GPIO_LCKR_LCKK                       ((uint32_t)0x00010000)

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          ((uint32_t)0x00000001)     /*!<Peripheral Enable                             */
#define  I2C_CR1_SMBUS                       ((uint32_t)0x00000002)     /*!<SMBus Mode                                    */
#define  I2C_CR1_SMBTYPE                     ((uint32_t)0x00000008)     /*!<SMBus Type                                    */
#define  I2C_CR1_ENARP                       ((uint32_t)0x00000010)     /*!<ARP Enable                                    */
#define  I2C_CR1_ENPEC                       ((uint32_t)0x00000020)     /*!<PEC Enable                                    */
#define  I2C_CR1_ENGC                        ((uint32_t)0x00000040)     /*!<General Call Enable                           */
#define  I2C_CR1_NOSTRETCH                   ((uint32_t)0x00000080)     /*!<Clock Stretching Disable (Slave mode)  */
#define  I2C_CR1_START                       ((uint32_t)0x00000100)     /*!<Start Generation                              */
#define  I2C_CR1_STOP                        ((uint32_t)0x00000200)     /*!<Stop Generation                               */
#define  I2C_CR1_ACK                         ((uint32_t)0x00000400)     /*!<Acknowledge Enable                            */
#define  I2C_CR1_POS                         ((uint32_t)0x00000800)     /*!<Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         ((uint32_t)0x00001000)     /*!<Packet Error Checking                         */
#define  I2C_CR1_ALERT                       ((uint32_t)0x00002000)     /*!<SMBus Alert                                   */
#define  I2C_CR1_SWRST                       ((uint32_t)0x00008000)     /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        ((uint32_t)0x0000003F)     /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
#define  I2C_CR2_FREQ_0                      ((uint32_t)0x00000001)     /*!<Bit 0 */
#define  I2C_CR2_FREQ_1                      ((uint32_t)0x00000002)     /*!<Bit 1 */
#define  I2C_CR2_FREQ_2                      ((uint32_t)0x00000004)     /*!<Bit 2 */
#define  I2C_CR2_FREQ_3                      ((uint32_t)0x00000008)     /*!<Bit 3 */
#define  I2C_CR2_FREQ_4                      ((uint32_t)0x00000010)     /*!<Bit 4 */
#define  I2C_CR2_FREQ_5                      ((uint32_t)0x00000020)     /*!<Bit 5 */

#define  I2C_CR2_ITERREN                     ((uint32_t)0x00000100)     /*!<Error Interrupt Enable  */
#define  I2C_CR2_ITEVTEN                     ((uint32_t)0x00000200)     /*!<Event Interrupt Enable  */
#define  I2C_CR2_ITBUFEN                     ((uint32_t)0x00000400)     /*!<Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       ((uint32_t)0x00000800)     /*!<DMA Requests Enable     */
#define  I2C_CR2_LAST                        ((uint32_t)0x00001000)     /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_7                     ((uint32_t)0x000000FE)     /*!<Interface Address */
#define  I2C_OAR1_ADD8_9                     ((uint32_t)0x00000300)     /*!<Interface Address */

#define  I2C_OAR1_ADD0                       ((uint32_t)0x00000001)     /*!<Bit 0 */
#define  I2C_OAR1_ADD1                       ((uint32_t)0x00000002)     /*!<Bit 1 */
#define  I2C_OAR1_ADD2                       ((uint32_t)0x00000004)     /*!<Bit 2 */
#define  I2C_OAR1_ADD3                       ((uint32_t)0x00000008)     /*!<Bit 3 */
#define  I2C_OAR1_ADD4                       ((uint32_t)0x00000010)     /*!<Bit 4 */
#define  I2C_OAR1_ADD5                       ((uint32_t)0x00000020)     /*!<Bit 5 */
#define  I2C_OAR1_ADD6                       ((uint32_t)0x00000040)     /*!<Bit 6 */
#define  I2C_OAR1_ADD7                       ((uint32_t)0x00000080)     /*!<Bit 7 */
#define  I2C_OAR1_ADD8                       ((uint32_t)0x00000100)     /*!<Bit 8 */
#define  I2C_OAR1_ADD9                       ((uint32_t)0x00000200)     /*!<Bit 9 */

#define  I2C_OAR1_ADDMODE                    ((uint32_t)0x00008000)     /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     ((uint32_t)0x00000001)        /*!<Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       ((uint32_t)0x000000FE)        /*!<Interface address           */

/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           ((uint32_t)0x000000FF)        /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          ((uint32_t)0x00000001)     /*!<Start Bit (Master mode)                  */
#define  I2C_SR1_ADDR                        ((uint32_t)0x00000002)     /*!<Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         ((uint32_t)0x00000004)     /*!<Byte Transfer Finished                          */
#define  I2C_SR1_ADD10                       ((uint32_t)0x00000008)     /*!<10-bit header sent (Master mode)         */
#define  I2C_SR1_STOPF                       ((uint32_t)0x00000010)     /*!<Stop detection (Slave mode)              */
#define  I2C_SR1_RXNE                        ((uint32_t)0x00000040)     /*!<Data Register not Empty (receivers)      */
#define  I2C_SR1_TXE                         ((uint32_t)0x00000080)     /*!<Data Register Empty (transmitters)       */
#define  I2C_SR1_BERR                        ((uint32_t)0x00000100)     /*!<Bus Error                                       */
#define  I2C_SR1_ARLO                        ((uint32_t)0x00000200)     /*!<Arbitration Lost (master mode)           */
#define  I2C_SR1_AF                          ((uint32_t)0x00000400)     /*!<Acknowledge Failure                             */
#define  I2C_SR1_OVR                         ((uint32_t)0x00000800)     /*!<Overrun/Underrun                                */
#define  I2C_SR1_PECERR                      ((uint32_t)0x00001000)     /*!<PEC Error in reception                          */
#define  I2C_SR1_TIMEOUT                     ((uint32_t)0x00004000)     /*!<Timeout or Tlow Error                           */
#define  I2C_SR1_SMBALERT                    ((uint32_t)0x00008000)     /*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         ((uint32_t)0x00000001)     /*!<Master/Slave                              */
#define  I2C_SR2_BUSY                        ((uint32_t)0x00000002)     /*!<Bus Busy                                  */
#define  I2C_SR2_TRA                         ((uint32_t)0x00000004)     /*!<Transmitter/Receiver                      */
#define  I2C_SR2_GENCALL                     ((uint32_t)0x00000010)     /*!<General Call Address (Slave mode)  */
#define  I2C_SR2_SMBDEFAULT                  ((uint32_t)0x00000020)     /*!<SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     ((uint32_t)0x00000040)     /*!<SMBus Host Header (Slave mode)     */
#define  I2C_SR2_DUALF                       ((uint32_t)0x00000080)     /*!<Dual Flag (Slave mode)             */
#define  I2C_SR2_PEC                         ((uint32_t)0x0000FF00)     /*!<Packet Error Checking Register            */

/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         ((uint32_t)0x00000FFF)     /*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        ((uint32_t)0x00004000)     /*!<Fast Mode Duty Cycle                                       */
#define  I2C_CCR_FS                          ((uint32_t)0x00008000)     /*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define  I2C_TRISE_TRISE                     ((uint32_t)0x0000003F)     /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************  Bit definition for I2C_FLTR register  *******************/
#define  I2C_FLTR_DNF                        ((uint32_t)0x0000000F)     /*!<Digital Noise Filter */
#define  I2C_FLTR_ANOFF                      ((uint32_t)0x00000010)     /*!<Analog Noise Filter OFF */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((uint32_t)0xFFFF)            /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((uint32_t)0x07)               /*!<PR[2:0] (Prescaler divider)         */
#define  IWDG_PR_PR_0                        ((uint32_t)0x01)               /*!<Bit 0 */
#define  IWDG_PR_PR_1                        ((uint32_t)0x02)               /*!<Bit 1 */
#define  IWDG_PR_PR_2                        ((uint32_t)0x04)               /*!<Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((uint32_t)0x0FFF)            /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((uint32_t)0x01)               /*!<Watchdog prescaler value update      */
#define  IWDG_SR_RVU                         ((uint32_t)0x02)               /*!<Watchdog counter reload value update */


/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((uint32_t)0x00000001)     /*!< Low-Power Deepsleep                 */
#define  PWR_CR_PDDS                         ((uint32_t)0x00000002)     /*!< Power Down Deepsleep                */
#define  PWR_CR_CWUF                         ((uint32_t)0x00000004)     /*!< Clear Wakeup Flag                   */
#define  PWR_CR_CSBF                         ((uint32_t)0x00000008)     /*!< Clear Standby Flag                  */
#define  PWR_CR_PVDE                         ((uint32_t)0x00000010)     /*!< Power Voltage Detector Enable       */

#define  PWR_CR_PLS                          ((uint32_t)0x000000E0)     /*!< PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        ((uint32_t)0x00000020)     /*!< Bit 0 */
#define  PWR_CR_PLS_1                        ((uint32_t)0x00000040)     /*!< Bit 1 */
#define  PWR_CR_PLS_2                        ((uint32_t)0x00000080)     /*!< Bit 2 */

/*!< PVD level configuration */
#define  PWR_CR_PLS_LEV0                     ((uint32_t)0x00000000)     /*!< PVD level 0 */
#define  PWR_CR_PLS_LEV1                     ((uint32_t)0x00000020)     /*!< PVD level 1 */
#define  PWR_CR_PLS_LEV2                     ((uint32_t)0x00000040)     /*!< PVD level 2 */
#define  PWR_CR_PLS_LEV3                     ((uint32_t)0x00000060)     /*!< PVD level 3 */
#define  PWR_CR_PLS_LEV4                     ((uint32_t)0x00000080)     /*!< PVD level 4 */
#define  PWR_CR_PLS_LEV5                     ((uint32_t)0x000000A0)     /*!< PVD level 5 */
#define  PWR_CR_PLS_LEV6                     ((uint32_t)0x000000C0)     /*!< PVD level 6 */
#define  PWR_CR_PLS_LEV7                     ((uint32_t)0x000000E0)     /*!< PVD level 7 */

#define  PWR_CR_DBP                          ((uint32_t)0x00000100)     /*!< Disable Backup Domain write protection               */
#define  PWR_CR_FPDS                         ((uint32_t)0x00000200)     /*!< Flash power down in Stop mode                        */
#define  PWR_CR_VOS                          ((uint32_t)0x00004000)     /*!< VOS bit (Regulator voltage scaling output selection) */

/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((uint32_t)0x00000001)     /*!< Wakeup Flag                                      */
#define  PWR_CSR_SBF                         ((uint32_t)0x00000002)     /*!< Standby Flag                                     */
#define  PWR_CSR_PVDO                        ((uint32_t)0x00000004)     /*!< PVD Output                                       */
#define  PWR_CSR_BRR                         ((uint32_t)0x00000008)     /*!< Backup regulator ready                           */
#define  PWR_CSR_EWUP                        ((uint32_t)0x00000100)     /*!< Enable WKUP pin                                  */
#define  PWR_CSR_BRE                         ((uint32_t)0x00000200)     /*!< Backup regulator enable                          */
#define  PWR_CSR_VOSRDY                      ((uint32_t)0x00004000)     /*!< Regulator voltage scaling output selection ready */

/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)

#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)
#define  RCC_CR_HSITRIM_0                    ((uint32_t)0x00000008)/*!<Bit 0 */
#define  RCC_CR_HSITRIM_1                    ((uint32_t)0x00000010)/*!<Bit 1 */
#define  RCC_CR_HSITRIM_2                    ((uint32_t)0x00000020)/*!<Bit 2 */
#define  RCC_CR_HSITRIM_3                    ((uint32_t)0x00000040)/*!<Bit 3 */
#define  RCC_CR_HSITRIM_4                    ((uint32_t)0x00000080)/*!<Bit 4 */

#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)
#define  RCC_CR_HSICAL_0                     ((uint32_t)0x00000100)/*!<Bit 0 */
#define  RCC_CR_HSICAL_1                     ((uint32_t)0x00000200)/*!<Bit 1 */
#define  RCC_CR_HSICAL_2                     ((uint32_t)0x00000400)/*!<Bit 2 */
#define  RCC_CR_HSICAL_3                     ((uint32_t)0x00000800)/*!<Bit 3 */
#define  RCC_CR_HSICAL_4                     ((uint32_t)0x00001000)/*!<Bit 4 */
#define  RCC_CR_HSICAL_5                     ((uint32_t)0x00002000)/*!<Bit 5 */
#define  RCC_CR_HSICAL_6                     ((uint32_t)0x00004000)/*!<Bit 6 */
#define  RCC_CR_HSICAL_7                     ((uint32_t)0x00008000)/*!<Bit 7 */

#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)
#define  RCC_CR_PLLI2SON                     ((uint32_t)0x04000000)
#define  RCC_CR_PLLI2SRDY                    ((uint32_t)0x08000000)

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLM                    ((uint32_t)0x0000003F)
#define  RCC_PLLCFGR_PLLM_0                  ((uint32_t)0x00000001)
#define  RCC_PLLCFGR_PLLM_1                  ((uint32_t)0x00000002)
#define  RCC_PLLCFGR_PLLM_2                  ((uint32_t)0x00000004)
#define  RCC_PLLCFGR_PLLM_3                  ((uint32_t)0x00000008)
#define  RCC_PLLCFGR_PLLM_4                  ((uint32_t)0x00000010)
#define  RCC_PLLCFGR_PLLM_5                  ((uint32_t)0x00000020)

#define  RCC_PLLCFGR_PLLN                     ((uint32_t)0x00007FC0)
#define  RCC_PLLCFGR_PLLN_0                   ((uint32_t)0x00000040)
#define  RCC_PLLCFGR_PLLN_1                   ((uint32_t)0x00000080)
#define  RCC_PLLCFGR_PLLN_2                   ((uint32_t)0x00000100)
#define  RCC_PLLCFGR_PLLN_3                   ((uint32_t)0x00000200)
#define  RCC_PLLCFGR_PLLN_4                   ((uint32_t)0x00000400)
#define  RCC_PLLCFGR_PLLN_5                   ((uint32_t)0x00000800)
#define  RCC_PLLCFGR_PLLN_6                   ((uint32_t)0x00001000)
#define  RCC_PLLCFGR_PLLN_7                   ((uint32_t)0x00002000)
#define  RCC_PLLCFGR_PLLN_8                   ((uint32_t)0x00004000)

#define  RCC_PLLCFGR_PLLP                    ((uint32_t)0x00030000)
#define  RCC_PLLCFGR_PLLP_0                  ((uint32_t)0x00010000)
#define  RCC_PLLCFGR_PLLP_1                  ((uint32_t)0x00020000)

#define  RCC_PLLCFGR_PLLSRC                  ((uint32_t)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSE              ((uint32_t)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSI              ((uint32_t)0x00000000)

#define  RCC_PLLCFGR_PLLQ                    ((uint32_t)0x0F000000)
#define  RCC_PLLCFGR_PLLQ_0                  ((uint32_t)0x01000000)
#define  RCC_PLLCFGR_PLLQ_1                  ((uint32_t)0x02000000)
#define  RCC_PLLCFGR_PLLQ_2                  ((uint32_t)0x04000000)
#define  RCC_PLLCFGR_PLLQ_3                  ((uint32_t)0x08000000)

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00001C00)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00001000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00001000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00001400)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00001800)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00001C00)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x0000E000)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00008000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00008000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x0000A000)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x0000C000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x0000E000)        /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define  RCC_CFGR_RTCPRE                     ((uint32_t)0x001F0000)
#define  RCC_CFGR_RTCPRE_0                   ((uint32_t)0x00010000)
#define  RCC_CFGR_RTCPRE_1                   ((uint32_t)0x00020000)
#define  RCC_CFGR_RTCPRE_2                   ((uint32_t)0x00040000)
#define  RCC_CFGR_RTCPRE_3                   ((uint32_t)0x00080000)
#define  RCC_CFGR_RTCPRE_4                   ((uint32_t)0x00100000)

/*!< MCO1 configuration */
#define  RCC_CFGR_MCO1                       ((uint32_t)0x00600000)
#define  RCC_CFGR_MCO1_0                     ((uint32_t)0x00200000)
#define  RCC_CFGR_MCO1_1                     ((uint32_t)0x00400000)

#define  RCC_CFGR_I2SSRC                     ((uint32_t)0x00800000)

#define  RCC_CFGR_MCO1PRE                    ((uint32_t)0x07000000)
#define  RCC_CFGR_MCO1PRE_0                  ((uint32_t)0x01000000)
#define  RCC_CFGR_MCO1PRE_1                  ((uint32_t)0x02000000)
#define  RCC_CFGR_MCO1PRE_2                  ((uint32_t)0x04000000)

#define  RCC_CFGR_MCO2PRE                    ((uint32_t)0x38000000)
#define  RCC_CFGR_MCO2PRE_0                  ((uint32_t)0x08000000)
#define  RCC_CFGR_MCO2PRE_1                  ((uint32_t)0x10000000)
#define  RCC_CFGR_MCO2PRE_2                  ((uint32_t)0x20000000)

#define  RCC_CFGR_MCO2                       ((uint32_t)0xC0000000)
#define  RCC_CFGR_MCO2_0                     ((uint32_t)0x40000000)
#define  RCC_CFGR_MCO2_1                     ((uint32_t)0x80000000)

/********************  Bit definition for RCC_CIR register  *******************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001)
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002)
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004)
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008)
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010)
#define  RCC_CIR_PLLI2SRDYF                  ((uint32_t)0x00000020)

#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080)
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100)
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200)
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400)
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800)
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000)
#define  RCC_CIR_PLLI2SRDYIE                 ((uint32_t)0x00002000)

#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000)
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000)
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000)
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000)
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000)
#define  RCC_CIR_PLLI2SRDYC                  ((uint32_t)0x00200000)

#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_GPIOARST               ((uint32_t)0x00000001)
#define  RCC_AHB1RSTR_GPIOBRST               ((uint32_t)0x00000002)
#define  RCC_AHB1RSTR_GPIOCRST               ((uint32_t)0x00000004)
#define  RCC_AHB1RSTR_GPIODRST               ((uint32_t)0x00000008)
#define  RCC_AHB1RSTR_GPIOERST               ((uint32_t)0x00000010)
#define  RCC_AHB1RSTR_GPIOFRST               ((uint32_t)0x00000020)
#define  RCC_AHB1RSTR_GPIOGRST               ((uint32_t)0x00000040)
#define  RCC_AHB1RSTR_GPIOHRST               ((uint32_t)0x00000080)
#define  RCC_AHB1RSTR_GPIOIRST               ((uint32_t)0x00000100)
#define  RCC_AHB1RSTR_CRCRST                 ((uint32_t)0x00001000)
#define  RCC_AHB1RSTR_DMA1RST                ((uint32_t)0x00200000)
#define  RCC_AHB1RSTR_DMA2RST                ((uint32_t)0x00400000)
#define  RCC_AHB1RSTR_ETHMACRST              ((uint32_t)0x02000000)
#define  RCC_AHB1RSTR_OTGHRST                ((uint32_t)0x20000000)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_DCMIRST                ((uint32_t)0x00000001)
#define  RCC_AHB2RSTR_RNGRST                 ((uint32_t)0x00000040)
#define  RCC_AHB2RSTR_OTGFSRST               ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3RSTR register  **************/

#define  RCC_AHB3RSTR_FSMCRST                ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define  RCC_APB1RSTR_TIM2RST                ((uint32_t)0x00000001)
#define  RCC_APB1RSTR_TIM3RST                ((uint32_t)0x00000002)
#define  RCC_APB1RSTR_TIM4RST                ((uint32_t)0x00000004)
#define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008)
#define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)
#define  RCC_APB1RSTR_TIM7RST                ((uint32_t)0x00000020)
#define  RCC_APB1RSTR_TIM12RST               ((uint32_t)0x00000040)
#define  RCC_APB1RSTR_TIM13RST               ((uint32_t)0x00000080)
#define  RCC_APB1RSTR_TIM14RST               ((uint32_t)0x00000100)
#define  RCC_APB1RSTR_WWDGRST                ((uint32_t)0x00000800)
#define  RCC_APB1RSTR_SPI2RST                ((uint32_t)0x00004000)
#define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00008000)
#define  RCC_APB1RSTR_USART2RST              ((uint32_t)0x00020000)
#define  RCC_APB1RSTR_USART3RST              ((uint32_t)0x00040000)
#define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)
#define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)
#define  RCC_APB1RSTR_I2C1RST                ((uint32_t)0x00200000)
#define  RCC_APB1RSTR_I2C2RST                ((uint32_t)0x00400000)
#define  RCC_APB1RSTR_I2C3RST                ((uint32_t)0x00800000)
#define  RCC_APB1RSTR_CAN1RST                ((uint32_t)0x02000000)
#define  RCC_APB1RSTR_CAN2RST                ((uint32_t)0x04000000)
#define  RCC_APB1RSTR_PWRRST                 ((uint32_t)0x10000000)
#define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000001)
#define  RCC_APB2RSTR_TIM8RST                ((uint32_t)0x00000002)
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00000010)
#define  RCC_APB2RSTR_USART6RST              ((uint32_t)0x00000020)
#define  RCC_APB2RSTR_ADCRST                 ((uint32_t)0x00000100)
#define  RCC_APB2RSTR_SDIORST                ((uint32_t)0x00000800)
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000)
#define  RCC_APB2RSTR_SYSCFGRST              ((uint32_t)0x00004000)
#define  RCC_APB2RSTR_TIM9RST                ((uint32_t)0x00010000)
#define  RCC_APB2RSTR_TIM10RST               ((uint32_t)0x00020000)
#define  RCC_APB2RSTR_TIM11RST               ((uint32_t)0x00040000)

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 ((uint32_t)0x00000001)
#define  RCC_AHB1ENR_GPIOBEN                 ((uint32_t)0x00000002)
#define  RCC_AHB1ENR_GPIOCEN                 ((uint32_t)0x00000004)
#define  RCC_AHB1ENR_GPIODEN                 ((uint32_t)0x00000008)
#define  RCC_AHB1ENR_GPIOEEN                 ((uint32_t)0x00000010)
#define  RCC_AHB1ENR_GPIOFEN                 ((uint32_t)0x00000020)
#define  RCC_AHB1ENR_GPIOGEN                 ((uint32_t)0x00000040)
#define  RCC_AHB1ENR_GPIOHEN                 ((uint32_t)0x00000080)
#define  RCC_AHB1ENR_GPIOIEN                 ((uint32_t)0x00000100)
#define  RCC_AHB1ENR_CRCEN                   ((uint32_t)0x00001000)
#define  RCC_AHB1ENR_BKPSRAMEN               ((uint32_t)0x00040000)
#define  RCC_AHB1ENR_CCMDATARAMEN            ((uint32_t)0x00100000)
#define  RCC_AHB1ENR_DMA1EN                  ((uint32_t)0x00200000)
#define  RCC_AHB1ENR_DMA2EN                  ((uint32_t)0x00400000)

#define  RCC_AHB1ENR_ETHMACEN                ((uint32_t)0x02000000)
#define  RCC_AHB1ENR_ETHMACTXEN              ((uint32_t)0x04000000)
#define  RCC_AHB1ENR_ETHMACRXEN              ((uint32_t)0x08000000)
#define  RCC_AHB1ENR_ETHMACPTPEN             ((uint32_t)0x10000000)
#define  RCC_AHB1ENR_OTGHSEN                 ((uint32_t)0x20000000)
#define  RCC_AHB1ENR_OTGHSULPIEN             ((uint32_t)0x40000000)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_DCMIEN                  ((uint32_t)0x00000001)
#define  RCC_AHB2ENR_RNGEN                   ((uint32_t)0x00000040)
#define  RCC_AHB2ENR_OTGFSEN                 ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3ENR register  ***************/

#define  RCC_AHB3ENR_FSMCEN                  ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define  RCC_APB1ENR_TIM2EN                  ((uint32_t)0x00000001)
#define  RCC_APB1ENR_TIM3EN                  ((uint32_t)0x00000002)
#define  RCC_APB1ENR_TIM4EN                  ((uint32_t)0x00000004)
#define  RCC_APB1ENR_TIM5EN                  ((uint32_t)0x00000008)
#define  RCC_APB1ENR_TIM6EN                  ((uint32_t)0x00000010)
#define  RCC_APB1ENR_TIM7EN                  ((uint32_t)0x00000020)
#define  RCC_APB1ENR_TIM12EN                 ((uint32_t)0x00000040)
#define  RCC_APB1ENR_TIM13EN                 ((uint32_t)0x00000080)
#define  RCC_APB1ENR_TIM14EN                 ((uint32_t)0x00000100)
#define  RCC_APB1ENR_WWDGEN                  ((uint32_t)0x00000800)
#define  RCC_APB1ENR_SPI2EN                  ((uint32_t)0x00004000)
#define  RCC_APB1ENR_SPI3EN                  ((uint32_t)0x00008000)
#define  RCC_APB1ENR_USART2EN                ((uint32_t)0x00020000)
#define  RCC_APB1ENR_USART3EN                ((uint32_t)0x00040000)
#define  RCC_APB1ENR_UART4EN                 ((uint32_t)0x00080000)
#define  RCC_APB1ENR_UART5EN                 ((uint32_t)0x00100000)
#define  RCC_APB1ENR_I2C1EN                  ((uint32_t)0x00200000)
#define  RCC_APB1ENR_I2C2EN                  ((uint32_t)0x00400000)
#define  RCC_APB1ENR_I2C3EN                  ((uint32_t)0x00800000)
#define  RCC_APB1ENR_CAN1EN                  ((uint32_t)0x02000000)
#define  RCC_APB1ENR_CAN2EN                  ((uint32_t)0x04000000)
#define  RCC_APB1ENR_PWREN                   ((uint32_t)0x10000000)
#define  RCC_APB1ENR_DACEN                   ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000001)
#define  RCC_APB2ENR_TIM8EN                  ((uint32_t)0x00000002)
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00000010)
#define  RCC_APB2ENR_USART6EN                ((uint32_t)0x00000020)
#define  RCC_APB2ENR_ADC1EN                  ((uint32_t)0x00000100)
#define  RCC_APB2ENR_ADC2EN                  ((uint32_t)0x00000200)
#define  RCC_APB2ENR_ADC3EN                  ((uint32_t)0x00000400)
#define  RCC_APB2ENR_SDIOEN                  ((uint32_t)0x00000800)
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000)
#define  RCC_APB2ENR_SYSCFGEN                ((uint32_t)0x00004000)
#define  RCC_APB2ENR_TIM9EN                  ((uint32_t)0x00010000)
#define  RCC_APB2ENR_TIM10EN                 ((uint32_t)0x00020000)
#define  RCC_APB2ENR_TIM11EN                 ((uint32_t)0x00040000)
#define  RCC_APB2ENR_SPI5EN                  ((uint32_t)0x00100000)
#define  RCC_APB2ENR_SPI6EN                  ((uint32_t)0x00200000)

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define  RCC_AHB1LPENR_GPIOALPEN             ((uint32_t)0x00000001)
#define  RCC_AHB1LPENR_GPIOBLPEN             ((uint32_t)0x00000002)
#define  RCC_AHB1LPENR_GPIOCLPEN             ((uint32_t)0x00000004)
#define  RCC_AHB1LPENR_GPIODLPEN             ((uint32_t)0x00000008)
#define  RCC_AHB1LPENR_GPIOELPEN             ((uint32_t)0x00000010)
#define  RCC_AHB1LPENR_GPIOFLPEN             ((uint32_t)0x00000020)
#define  RCC_AHB1LPENR_GPIOGLPEN             ((uint32_t)0x00000040)
#define  RCC_AHB1LPENR_GPIOHLPEN             ((uint32_t)0x00000080)
#define  RCC_AHB1LPENR_GPIOILPEN             ((uint32_t)0x00000100)
#define  RCC_AHB1LPENR_CRCLPEN               ((uint32_t)0x00001000)
#define  RCC_AHB1LPENR_FLITFLPEN             ((uint32_t)0x00008000)
#define  RCC_AHB1LPENR_SRAM1LPEN             ((uint32_t)0x00010000)
#define  RCC_AHB1LPENR_SRAM2LPEN             ((uint32_t)0x00020000)
#define  RCC_AHB1LPENR_BKPSRAMLPEN           ((uint32_t)0x00040000)
#define  RCC_AHB1LPENR_DMA1LPEN              ((uint32_t)0x00200000)
#define  RCC_AHB1LPENR_DMA2LPEN              ((uint32_t)0x00400000)
#define  RCC_AHB1LPENR_ETHMACLPEN            ((uint32_t)0x02000000)
#define  RCC_AHB1LPENR_ETHMACTXLPEN          ((uint32_t)0x04000000)
#define  RCC_AHB1LPENR_ETHMACRXLPEN          ((uint32_t)0x08000000)
#define  RCC_AHB1LPENR_ETHMACPTPLPEN         ((uint32_t)0x10000000)
#define  RCC_AHB1LPENR_OTGHSLPEN             ((uint32_t)0x20000000)
#define  RCC_AHB1LPENR_OTGHSULPILPEN         ((uint32_t)0x40000000)

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define  RCC_AHB2LPENR_DCMILPEN              ((uint32_t)0x00000001)
#define  RCC_AHB2LPENR_RNGLPEN               ((uint32_t)0x00000040)
#define  RCC_AHB2LPENR_OTGFSLPEN             ((uint32_t)0x00000080)

/********************  Bit definition for RCC_AHB3LPENR register  *************/

#define  RCC_AHB3LPENR_FSMCLPEN              ((uint32_t)0x00000001)

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define  RCC_APB1LPENR_TIM2LPEN              ((uint32_t)0x00000001)
#define  RCC_APB1LPENR_TIM3LPEN              ((uint32_t)0x00000002)
#define  RCC_APB1LPENR_TIM4LPEN              ((uint32_t)0x00000004)
#define  RCC_APB1LPENR_TIM5LPEN              ((uint32_t)0x00000008)
#define  RCC_APB1LPENR_TIM6LPEN              ((uint32_t)0x00000010)
#define  RCC_APB1LPENR_TIM7LPEN              ((uint32_t)0x00000020)
#define  RCC_APB1LPENR_TIM12LPEN             ((uint32_t)0x00000040)
#define  RCC_APB1LPENR_TIM13LPEN             ((uint32_t)0x00000080)
#define  RCC_APB1LPENR_TIM14LPEN             ((uint32_t)0x00000100)
#define  RCC_APB1LPENR_WWDGLPEN              ((uint32_t)0x00000800)
#define  RCC_APB1LPENR_SPI2LPEN              ((uint32_t)0x00004000)
#define  RCC_APB1LPENR_SPI3LPEN              ((uint32_t)0x00008000)
#define  RCC_APB1LPENR_USART2LPEN            ((uint32_t)0x00020000)
#define  RCC_APB1LPENR_USART3LPEN            ((uint32_t)0x00040000)
#define  RCC_APB1LPENR_UART4LPEN             ((uint32_t)0x00080000)
#define  RCC_APB1LPENR_UART5LPEN             ((uint32_t)0x00100000)
#define  RCC_APB1LPENR_I2C1LPEN              ((uint32_t)0x00200000)
#define  RCC_APB1LPENR_I2C2LPEN              ((uint32_t)0x00400000)
#define  RCC_APB1LPENR_I2C3LPEN              ((uint32_t)0x00800000)
#define  RCC_APB1LPENR_CAN1LPEN              ((uint32_t)0x02000000)
#define  RCC_APB1LPENR_CAN2LPEN              ((uint32_t)0x04000000)
#define  RCC_APB1LPENR_PWRLPEN               ((uint32_t)0x10000000)
#define  RCC_APB1LPENR_DACLPEN               ((uint32_t)0x20000000)

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define  RCC_APB2LPENR_TIM1LPEN              ((uint32_t)0x00000001)
#define  RCC_APB2LPENR_TIM8LPEN              ((uint32_t)0x00000002)
#define  RCC_APB2LPENR_USART1LPEN            ((uint32_t)0x00000010)
#define  RCC_APB2LPENR_USART6LPEN            ((uint32_t)0x00000020)
#define  RCC_APB2LPENR_ADC1LPEN              ((uint32_t)0x00000100)
#define  RCC_APB2LPENR_ADC2LPEN              ((uint32_t)0x00000200)
#define  RCC_APB2LPENR_ADC3LPEN              ((uint32_t)0x00000400)
#define  RCC_APB2LPENR_SDIOLPEN              ((uint32_t)0x00000800)
#define  RCC_APB2LPENR_SPI1LPEN              ((uint32_t)0x00001000)
#define  RCC_APB2LPENR_SYSCFGLPEN            ((uint32_t)0x00004000)
#define  RCC_APB2LPENR_TIM9LPEN              ((uint32_t)0x00010000)
#define  RCC_APB2LPENR_TIM10LPEN             ((uint32_t)0x00020000)
#define  RCC_APB2LPENR_TIM11LPEN             ((uint32_t)0x00040000)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001)
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002)
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004)

#define  RCC_BDCR_RTCSEL                    ((uint32_t)0x00000300)
#define  RCC_BDCR_RTCSEL_0                  ((uint32_t)0x00000100)
#define  RCC_BDCR_RTCSEL_1                  ((uint32_t)0x00000200)

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000)
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)
#define  RCC_CSR_BORRSTF                     ((uint32_t)0x02000000)
#define  RCC_CSR_PADRSTF                     ((uint32_t)0x04000000)
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000)
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)
#define  RCC_CSR_WDGRSTF                     ((uint32_t)0x20000000)
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)

/********************  Bit definition for RCC_SSCGR register  *****************/
#define  RCC_SSCGR_MODPER                    ((uint32_t)0x00001FFF)
#define  RCC_SSCGR_INCSTEP                   ((uint32_t)0x0FFFE000)
#define  RCC_SSCGR_SPREADSEL                 ((uint32_t)0x40000000)
#define  RCC_SSCGR_SSCGEN                    ((uint32_t)0x80000000)

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define  RCC_PLLI2SCFGR_PLLI2SN              ((uint32_t)0x00007FC0)
#define  RCC_PLLI2SCFGR_PLLI2SN_0            ((uint32_t)0x00000040)
#define  RCC_PLLI2SCFGR_PLLI2SN_1            ((uint32_t)0x00000080)
#define  RCC_PLLI2SCFGR_PLLI2SN_2            ((uint32_t)0x00000100)
#define  RCC_PLLI2SCFGR_PLLI2SN_3            ((uint32_t)0x00000200)
#define  RCC_PLLI2SCFGR_PLLI2SN_4            ((uint32_t)0x00000400)
#define  RCC_PLLI2SCFGR_PLLI2SN_5            ((uint32_t)0x00000800)
#define  RCC_PLLI2SCFGR_PLLI2SN_6            ((uint32_t)0x00001000)
#define  RCC_PLLI2SCFGR_PLLI2SN_7            ((uint32_t)0x00002000)
#define  RCC_PLLI2SCFGR_PLLI2SN_8            ((uint32_t)0x00004000)

#define  RCC_PLLI2SCFGR_PLLI2SR              ((uint32_t)0x70000000)
#define  RCC_PLLI2SCFGR_PLLI2SR_0            ((uint32_t)0x10000000)
#define  RCC_PLLI2SCFGR_PLLI2SR_1            ((uint32_t)0x20000000)
#define  RCC_PLLI2SCFGR_PLLI2SR_2            ((uint32_t)0x40000000)

/******************************************************************************/
/*                                                                            */
/*                                    RNG                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RNG_CR register  *******************/
#define RNG_CR_RNGEN                         ((uint32_t)0x00000004)
#define RNG_CR_IE                            ((uint32_t)0x00000008)

/********************  Bits definition for RNG_SR register  *******************/
#define RNG_SR_DRDY                          ((uint32_t)0x00000001)
#define RNG_SR_CECS                          ((uint32_t)0x00000002)
#define RNG_SR_SECS                          ((uint32_t)0x00000004)
#define RNG_SR_CEIS                          ((uint32_t)0x00000020)
#define RNG_SR_SEIS                          ((uint32_t)0x00000040)

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM                            ((uint32_t)0x00400000)
#define RTC_TR_HT                            ((uint32_t)0x00300000)
#define RTC_TR_HT_0                          ((uint32_t)0x00100000)
#define RTC_TR_HT_1                          ((uint32_t)0x00200000)
#define RTC_TR_HU                            ((uint32_t)0x000F0000)
#define RTC_TR_HU_0                          ((uint32_t)0x00010000)
#define RTC_TR_HU_1                          ((uint32_t)0x00020000)
#define RTC_TR_HU_2                          ((uint32_t)0x00040000)
#define RTC_TR_HU_3                          ((uint32_t)0x00080000)
#define RTC_TR_MNT                           ((uint32_t)0x00007000)
#define RTC_TR_MNT_0                         ((uint32_t)0x00001000)
#define RTC_TR_MNT_1                         ((uint32_t)0x00002000)
#define RTC_TR_MNT_2                         ((uint32_t)0x00004000)
#define RTC_TR_MNU                           ((uint32_t)0x00000F00)
#define RTC_TR_MNU_0                         ((uint32_t)0x00000100)
#define RTC_TR_MNU_1                         ((uint32_t)0x00000200)
#define RTC_TR_MNU_2                         ((uint32_t)0x00000400)
#define RTC_TR_MNU_3                         ((uint32_t)0x00000800)
#define RTC_TR_ST                            ((uint32_t)0x00000070)
#define RTC_TR_ST_0                          ((uint32_t)0x00000010)
#define RTC_TR_ST_1                          ((uint32_t)0x00000020)
#define RTC_TR_ST_2                          ((uint32_t)0x00000040)
#define RTC_TR_SU                            ((uint32_t)0x0000000F)
#define RTC_TR_SU_0                          ((uint32_t)0x00000001)
#define RTC_TR_SU_1                          ((uint32_t)0x00000002)
#define RTC_TR_SU_2                          ((uint32_t)0x00000004)
#define RTC_TR_SU_3                          ((uint32_t)0x00000008)

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT                            ((uint32_t)0x00F00000)
#define RTC_DR_YT_0                          ((uint32_t)0x00100000)
#define RTC_DR_YT_1                          ((uint32_t)0x00200000)
#define RTC_DR_YT_2                          ((uint32_t)0x00400000)
#define RTC_DR_YT_3                          ((uint32_t)0x00800000)
#define RTC_DR_YU                            ((uint32_t)0x000F0000)
#define RTC_DR_YU_0                          ((uint32_t)0x00010000)
#define RTC_DR_YU_1                          ((uint32_t)0x00020000)
#define RTC_DR_YU_2                          ((uint32_t)0x00040000)
#define RTC_DR_YU_3                          ((uint32_t)0x00080000)
#define RTC_DR_WDU                           ((uint32_t)0x0000E000)
#define RTC_DR_WDU_0                         ((uint32_t)0x00002000)
#define RTC_DR_WDU_1                         ((uint32_t)0x00004000)
#define RTC_DR_WDU_2                         ((uint32_t)0x00008000)
#define RTC_DR_MT                            ((uint32_t)0x00001000)
#define RTC_DR_MU                            ((uint32_t)0x00000F00)
#define RTC_DR_MU_0                          ((uint32_t)0x00000100)
#define RTC_DR_MU_1                          ((uint32_t)0x00000200)
#define RTC_DR_MU_2                          ((uint32_t)0x00000400)
#define RTC_DR_MU_3                          ((uint32_t)0x00000800)
#define RTC_DR_DT                            ((uint32_t)0x00000030)
#define RTC_DR_DT_0                          ((uint32_t)0x00000010)
#define RTC_DR_DT_1                          ((uint32_t)0x00000020)
#define RTC_DR_DU                            ((uint32_t)0x0000000F)
#define RTC_DR_DU_0                          ((uint32_t)0x00000001)
#define RTC_DR_DU_1                          ((uint32_t)0x00000002)
#define RTC_DR_DU_2                          ((uint32_t)0x00000004)
#define RTC_DR_DU_3                          ((uint32_t)0x00000008)

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_COE                           ((uint32_t)0x00800000)
#define RTC_CR_OSEL                          ((uint32_t)0x00600000)
#define RTC_CR_OSEL_0                        ((uint32_t)0x00200000)
#define RTC_CR_OSEL_1                        ((uint32_t)0x00400000)
#define RTC_CR_POL                           ((uint32_t)0x00100000)
#define RTC_CR_COSEL                         ((uint32_t)0x00080000)
#define RTC_CR_BCK                           ((uint32_t)0x00040000)
#define RTC_CR_SUB1H                         ((uint32_t)0x00020000)
#define RTC_CR_ADD1H                         ((uint32_t)0x00010000)
#define RTC_CR_TSIE                          ((uint32_t)0x00008000)
#define RTC_CR_WUTIE                         ((uint32_t)0x00004000)
#define RTC_CR_ALRBIE                        ((uint32_t)0x00002000)
#define RTC_CR_ALRAIE                        ((uint32_t)0x00001000)
#define RTC_CR_TSE                           ((uint32_t)0x00000800)
#define RTC_CR_WUTE                          ((uint32_t)0x00000400)
#define RTC_CR_ALRBE                         ((uint32_t)0x00000200)
#define RTC_CR_ALRAE                         ((uint32_t)0x00000100)
#define RTC_CR_DCE                           ((uint32_t)0x00000080)
#define RTC_CR_FMT                           ((uint32_t)0x00000040)
#define RTC_CR_BYPSHAD                       ((uint32_t)0x00000020)
#define RTC_CR_REFCKON                       ((uint32_t)0x00000010)
#define RTC_CR_TSEDGE                        ((uint32_t)0x00000008)
#define RTC_CR_WUCKSEL                       ((uint32_t)0x00000007)
#define RTC_CR_WUCKSEL_0                     ((uint32_t)0x00000001)
#define RTC_CR_WUCKSEL_1                     ((uint32_t)0x00000002)
#define RTC_CR_WUCKSEL_2                     ((uint32_t)0x00000004)

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_RECALPF                      ((uint32_t)0x00010000)
#define RTC_ISR_TAMP1F                       ((uint32_t)0x00002000)
#define RTC_ISR_TAMP2F                       ((uint32_t)0x00004000)
#define RTC_ISR_TSOVF                        ((uint32_t)0x00001000)
#define RTC_ISR_TSF                          ((uint32_t)0x00000800)
#define RTC_ISR_WUTF                         ((uint32_t)0x00000400)
#define RTC_ISR_ALRBF                        ((uint32_t)0x00000200)
#define RTC_ISR_ALRAF                        ((uint32_t)0x00000100)
#define RTC_ISR_INIT                         ((uint32_t)0x00000080)
#define RTC_ISR_INITF                        ((uint32_t)0x00000040)
#define RTC_ISR_RSF                          ((uint32_t)0x00000020)
#define RTC_ISR_INITS                        ((uint32_t)0x00000010)
#define RTC_ISR_SHPF                         ((uint32_t)0x00000008)
#define RTC_ISR_WUTWF                        ((uint32_t)0x00000004)
#define RTC_ISR_ALRBWF                       ((uint32_t)0x00000002)
#define RTC_ISR_ALRAWF                       ((uint32_t)0x00000001)

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A                    ((uint32_t)0x007F0000)
#define RTC_PRER_PREDIV_S                    ((uint32_t)0x00007FFF)

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT                         ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_CALIBR register  ***************/
#define RTC_CALIBR_DCS                       ((uint32_t)0x00000080)
#define RTC_CALIBR_DC                        ((uint32_t)0x0000001F)

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4                      ((uint32_t)0x80000000)
#define RTC_ALRMAR_WDSEL                     ((uint32_t)0x40000000)
#define RTC_ALRMAR_DT                        ((uint32_t)0x30000000)
#define RTC_ALRMAR_DT_0                      ((uint32_t)0x10000000)
#define RTC_ALRMAR_DT_1                      ((uint32_t)0x20000000)
#define RTC_ALRMAR_DU                        ((uint32_t)0x0F000000)
#define RTC_ALRMAR_DU_0                      ((uint32_t)0x01000000)
#define RTC_ALRMAR_DU_1                      ((uint32_t)0x02000000)
#define RTC_ALRMAR_DU_2                      ((uint32_t)0x04000000)
#define RTC_ALRMAR_DU_3                      ((uint32_t)0x08000000)
#define RTC_ALRMAR_MSK3                      ((uint32_t)0x00800000)
#define RTC_ALRMAR_PM                        ((uint32_t)0x00400000)
#define RTC_ALRMAR_HT                        ((uint32_t)0x00300000)
#define RTC_ALRMAR_HT_0                      ((uint32_t)0x00100000)
#define RTC_ALRMAR_HT_1                      ((uint32_t)0x00200000)
#define RTC_ALRMAR_HU                        ((uint32_t)0x000F0000)
#define RTC_ALRMAR_HU_0                      ((uint32_t)0x00010000)
#define RTC_ALRMAR_HU_1                      ((uint32_t)0x00020000)
#define RTC_ALRMAR_HU_2                      ((uint32_t)0x00040000)
#define RTC_ALRMAR_HU_3                      ((uint32_t)0x00080000)
#define RTC_ALRMAR_MSK2                      ((uint32_t)0x00008000)
#define RTC_ALRMAR_MNT                       ((uint32_t)0x00007000)
#define RTC_ALRMAR_MNT_0                     ((uint32_t)0x00001000)
#define RTC_ALRMAR_MNT_1                     ((uint32_t)0x00002000)
#define RTC_ALRMAR_MNT_2                     ((uint32_t)0x00004000)
#define RTC_ALRMAR_MNU                       ((uint32_t)0x00000F00)
#define RTC_ALRMAR_MNU_0                     ((uint32_t)0x00000100)
#define RTC_ALRMAR_MNU_1                     ((uint32_t)0x00000200)
#define RTC_ALRMAR_MNU_2                     ((uint32_t)0x00000400)
#define RTC_ALRMAR_MNU_3                     ((uint32_t)0x00000800)
#define RTC_ALRMAR_MSK1                      ((uint32_t)0x00000080)
#define RTC_ALRMAR_ST                        ((uint32_t)0x00000070)
#define RTC_ALRMAR_ST_0                      ((uint32_t)0x00000010)
#define RTC_ALRMAR_ST_1                      ((uint32_t)0x00000020)
#define RTC_ALRMAR_ST_2                      ((uint32_t)0x00000040)
#define RTC_ALRMAR_SU                        ((uint32_t)0x0000000F)
#define RTC_ALRMAR_SU_0                      ((uint32_t)0x00000001)
#define RTC_ALRMAR_SU_1                      ((uint32_t)0x00000002)
#define RTC_ALRMAR_SU_2                      ((uint32_t)0x00000004)
#define RTC_ALRMAR_SU_3                      ((uint32_t)0x00000008)

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4                      ((uint32_t)0x80000000)
#define RTC_ALRMBR_WDSEL                     ((uint32_t)0x40000000)
#define RTC_ALRMBR_DT                        ((uint32_t)0x30000000)
#define RTC_ALRMBR_DT_0                      ((uint32_t)0x10000000)
#define RTC_ALRMBR_DT_1                      ((uint32_t)0x20000000)
#define RTC_ALRMBR_DU                        ((uint32_t)0x0F000000)
#define RTC_ALRMBR_DU_0                      ((uint32_t)0x01000000)
#define RTC_ALRMBR_DU_1                      ((uint32_t)0x02000000)
#define RTC_ALRMBR_DU_2                      ((uint32_t)0x04000000)
#define RTC_ALRMBR_DU_3                      ((uint32_t)0x08000000)
#define RTC_ALRMBR_MSK3                      ((uint32_t)0x00800000)
#define RTC_ALRMBR_PM                        ((uint32_t)0x00400000)
#define RTC_ALRMBR_HT                        ((uint32_t)0x00300000)
#define RTC_ALRMBR_HT_0                      ((uint32_t)0x00100000)
#define RTC_ALRMBR_HT_1                      ((uint32_t)0x00200000)
#define RTC_ALRMBR_HU                        ((uint32_t)0x000F0000)
#define RTC_ALRMBR_HU_0                      ((uint32_t)0x00010000)
#define RTC_ALRMBR_HU_1                      ((uint32_t)0x00020000)
#define RTC_ALRMBR_HU_2                      ((uint32_t)0x00040000)
#define RTC_ALRMBR_HU_3                      ((uint32_t)0x00080000)
#define RTC_ALRMBR_MSK2                      ((uint32_t)0x00008000)
#define RTC_ALRMBR_MNT                       ((uint32_t)0x00007000)
#define RTC_ALRMBR_MNT_0                     ((uint32_t)0x00001000)
#define RTC_ALRMBR_MNT_1                     ((uint32_t)0x00002000)
#define RTC_ALRMBR_MNT_2                     ((uint32_t)0x00004000)
#define RTC_ALRMBR_MNU                       ((uint32_t)0x00000F00)
#define RTC_ALRMBR_MNU_0                     ((uint32_t)0x00000100)
#define RTC_ALRMBR_MNU_1                     ((uint32_t)0x00000200)
#define RTC_ALRMBR_MNU_2                     ((uint32_t)0x00000400)
#define RTC_ALRMBR_MNU_3                     ((uint32_t)0x00000800)
#define RTC_ALRMBR_MSK1                      ((uint32_t)0x00000080)
#define RTC_ALRMBR_ST                        ((uint32_t)0x00000070)
#define RTC_ALRMBR_ST_0                      ((uint32_t)0x00000010)
#define RTC_ALRMBR_ST_1                      ((uint32_t)0x00000020)
#define RTC_ALRMBR_ST_2                      ((uint32_t)0x00000040)
#define RTC_ALRMBR_SU                        ((uint32_t)0x0000000F)
#define RTC_ALRMBR_SU_0                      ((uint32_t)0x00000001)
#define RTC_ALRMBR_SU_1                      ((uint32_t)0x00000002)
#define RTC_ALRMBR_SU_2                      ((uint32_t)0x00000004)
#define RTC_ALRMBR_SU_3                      ((uint32_t)0x00000008)

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY                          ((uint32_t)0x000000FF)

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS                           ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS                     ((uint32_t)0x00007FFF)
#define RTC_SHIFTR_ADD1S                     ((uint32_t)0x80000000)

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM                          ((uint32_t)0x00400000)
#define RTC_TSTR_HT                          ((uint32_t)0x00300000)
#define RTC_TSTR_HT_0                        ((uint32_t)0x00100000)
#define RTC_TSTR_HT_1                        ((uint32_t)0x00200000)
#define RTC_TSTR_HU                          ((uint32_t)0x000F0000)
#define RTC_TSTR_HU_0                        ((uint32_t)0x00010000)
#define RTC_TSTR_HU_1                        ((uint32_t)0x00020000)
#define RTC_TSTR_HU_2                        ((uint32_t)0x00040000)
#define RTC_TSTR_HU_3                        ((uint32_t)0x00080000)
#define RTC_TSTR_MNT                         ((uint32_t)0x00007000)
#define RTC_TSTR_MNT_0                       ((uint32_t)0x00001000)
#define RTC_TSTR_MNT_1                       ((uint32_t)0x00002000)
#define RTC_TSTR_MNT_2                       ((uint32_t)0x00004000)
#define RTC_TSTR_MNU                         ((uint32_t)0x00000F00)
#define RTC_TSTR_MNU_0                       ((uint32_t)0x00000100)
#define RTC_TSTR_MNU_1                       ((uint32_t)0x00000200)
#define RTC_TSTR_MNU_2                       ((uint32_t)0x00000400)
#define RTC_TSTR_MNU_3                       ((uint32_t)0x00000800)
#define RTC_TSTR_ST                          ((uint32_t)0x00000070)
#define RTC_TSTR_ST_0                        ((uint32_t)0x00000010)
#define RTC_TSTR_ST_1                        ((uint32_t)0x00000020)
#define RTC_TSTR_ST_2                        ((uint32_t)0x00000040)
#define RTC_TSTR_SU                          ((uint32_t)0x0000000F)
#define RTC_TSTR_SU_0                        ((uint32_t)0x00000001)
#define RTC_TSTR_SU_1                        ((uint32_t)0x00000002)
#define RTC_TSTR_SU_2                        ((uint32_t)0x00000004)
#define RTC_TSTR_SU_3                        ((uint32_t)0x00000008)

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU                         ((uint32_t)0x0000E000)
#define RTC_TSDR_WDU_0                       ((uint32_t)0x00002000)
#define RTC_TSDR_WDU_1                       ((uint32_t)0x00004000)
#define RTC_TSDR_WDU_2                       ((uint32_t)0x00008000)
#define RTC_TSDR_MT                          ((uint32_t)0x00001000)
#define RTC_TSDR_MU                          ((uint32_t)0x00000F00)
#define RTC_TSDR_MU_0                        ((uint32_t)0x00000100)
#define RTC_TSDR_MU_1                        ((uint32_t)0x00000200)
#define RTC_TSDR_MU_2                        ((uint32_t)0x00000400)
#define RTC_TSDR_MU_3                        ((uint32_t)0x00000800)
#define RTC_TSDR_DT                          ((uint32_t)0x00000030)
#define RTC_TSDR_DT_0                        ((uint32_t)0x00000010)
#define RTC_TSDR_DT_1                        ((uint32_t)0x00000020)
#define RTC_TSDR_DU                          ((uint32_t)0x0000000F)
#define RTC_TSDR_DU_0                        ((uint32_t)0x00000001)
#define RTC_TSDR_DU_1                        ((uint32_t)0x00000002)
#define RTC_TSDR_DU_2                        ((uint32_t)0x00000004)
#define RTC_TSDR_DU_3                        ((uint32_t)0x00000008)

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS                         ((uint32_t)0x0000FFFF)

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP                        ((uint32_t)0x00008000)
#define RTC_CALR_CALW8                       ((uint32_t)0x00004000)
#define RTC_CALR_CALW16                      ((uint32_t)0x00002000)
#define RTC_CALR_CALM                        ((uint32_t)0x000001FF)
#define RTC_CALR_CALM_0                      ((uint32_t)0x00000001)
#define RTC_CALR_CALM_1                      ((uint32_t)0x00000002)
#define RTC_CALR_CALM_2                      ((uint32_t)0x00000004)
#define RTC_CALR_CALM_3                      ((uint32_t)0x00000008)
#define RTC_CALR_CALM_4                      ((uint32_t)0x00000010)
#define RTC_CALR_CALM_5                      ((uint32_t)0x00000020)
#define RTC_CALR_CALM_6                      ((uint32_t)0x00000040)
#define RTC_CALR_CALM_7                      ((uint32_t)0x00000080)
#define RTC_CALR_CALM_8                      ((uint32_t)0x00000100)

/********************  Bits definition for RTC_TAFCR register  ****************/
#define RTC_TAFCR_ALARMOUTTYPE               ((uint32_t)0x00040000)
#define RTC_TAFCR_TSINSEL                    ((uint32_t)0x00020000)
#define RTC_TAFCR_TAMPINSEL                  ((uint32_t)0x00010000)
#define RTC_TAFCR_TAMPPUDIS                  ((uint32_t)0x00008000)
#define RTC_TAFCR_TAMPPRCH                   ((uint32_t)0x00006000)
#define RTC_TAFCR_TAMPPRCH_0                 ((uint32_t)0x00002000)
#define RTC_TAFCR_TAMPPRCH_1                 ((uint32_t)0x00004000)
#define RTC_TAFCR_TAMPFLT                    ((uint32_t)0x00001800)
#define RTC_TAFCR_TAMPFLT_0                  ((uint32_t)0x00000800)
#define RTC_TAFCR_TAMPFLT_1                  ((uint32_t)0x00001000)
#define RTC_TAFCR_TAMPFREQ                   ((uint32_t)0x00000700)
#define RTC_TAFCR_TAMPFREQ_0                 ((uint32_t)0x00000100)
#define RTC_TAFCR_TAMPFREQ_1                 ((uint32_t)0x00000200)
#define RTC_TAFCR_TAMPFREQ_2                 ((uint32_t)0x00000400)
#define RTC_TAFCR_TAMPTS                     ((uint32_t)0x00000080)
#define RTC_TAFCR_TAMP2TRG                   ((uint32_t)0x00000010)
#define RTC_TAFCR_TAMP2E                     ((uint32_t)0x00000008)
#define RTC_TAFCR_TAMPIE                     ((uint32_t)0x00000004)
#define RTC_TAFCR_TAMP1TRG                   ((uint32_t)0x00000002)
#define RTC_TAFCR_TAMP1E                     ((uint32_t)0x00000001)

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS                  ((uint32_t)0x0F000000)
#define RTC_ALRMASSR_MASKSS_0                ((uint32_t)0x01000000)
#define RTC_ALRMASSR_MASKSS_1                ((uint32_t)0x02000000)
#define RTC_ALRMASSR_MASKSS_2                ((uint32_t)0x04000000)
#define RTC_ALRMASSR_MASKSS_3                ((uint32_t)0x08000000)
#define RTC_ALRMASSR_SS                      ((uint32_t)0x00007FFF)

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS                  ((uint32_t)0x0F000000)
#define RTC_ALRMBSSR_MASKSS_0                ((uint32_t)0x01000000)
#define RTC_ALRMBSSR_MASKSS_1                ((uint32_t)0x02000000)
#define RTC_ALRMBSSR_MASKSS_2                ((uint32_t)0x04000000)
#define RTC_ALRMBSSR_MASKSS_3                ((uint32_t)0x08000000)
#define RTC_ALRMBSSR_SS                      ((uint32_t)0x00007FFF)

/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R                            ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R                           ((uint32_t)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R                           ((uint32_t)0xFFFFFFFF)



/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                  ((uint32_t)0x03)               /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0                ((uint32_t)0x01)               /*!<Bit 0 */
#define  SDIO_POWER_PWRCTRL_1                ((uint32_t)0x02)               /*!<Bit 1 */

/******************  Bit definition for SDIO_CLKCR register  ******************/
#define  SDIO_CLKCR_CLKDIV                   ((uint32_t)0x00FF)            /*!<Clock divide factor             */
#define  SDIO_CLKCR_CLKEN                    ((uint32_t)0x0100)            /*!<Clock enable bit                */
#define  SDIO_CLKCR_PWRSAV                   ((uint32_t)0x0200)            /*!<Power saving configuration bit  */
#define  SDIO_CLKCR_BYPASS                   ((uint32_t)0x0400)            /*!<Clock divider bypass enable bit */

#define  SDIO_CLKCR_WIDBUS                   ((uint32_t)0x1800)            /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                 ((uint32_t)0x0800)            /*!<Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                 ((uint32_t)0x1000)            /*!<Bit 1 */

#define  SDIO_CLKCR_NEGEDGE                  ((uint32_t)0x2000)            /*!<SDIO_CK dephasing selection bit */
#define  SDIO_CLKCR_HWFC_EN                  ((uint32_t)0x4000)            /*!<HW Flow Control enable          */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_CMDARG                     ((uint32_t)0xFFFFFFFF)            /*!<Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDINDEX                   ((uint32_t)0x003F)            /*!<Command Index                               */

#define  SDIO_CMD_WAITRESP                   ((uint32_t)0x00C0)            /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                 ((uint32_t)0x0040)            /*!< Bit 0 */
#define  SDIO_CMD_WAITRESP_1                 ((uint32_t)0x0080)            /*!< Bit 1 */

#define  SDIO_CMD_WAITINT                    ((uint32_t)0x0100)            /*!<CPSM Waits for Interrupt Request                               */
#define  SDIO_CMD_WAITPEND                   ((uint32_t)0x0200)            /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CPSMEN                     ((uint32_t)0x0400)            /*!<Command path state machine (CPSM) Enable bit                   */
#define  SDIO_CMD_SDIOSUSPEND                ((uint32_t)0x0800)            /*!<SD I/O suspend command                                         */
#define  SDIO_CMD_ENCMDCOMPL                 ((uint32_t)0x1000)            /*!<Enable CMD completion                                          */
#define  SDIO_CMD_NIEN                       ((uint32_t)0x2000)            /*!<Not Interrupt Enable */
#define  SDIO_CMD_CEATACMD                   ((uint32_t)0x4000)            /*!<CE-ATA command       */

/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define  SDIO_RESPCMD_RESPCMD                ((uint32_t)0x3F)               /*!<Response command index */

/******************  Bit definition for SDIO_RESP0 register  ******************/
#define  SDIO_RESP0_CARDSTATUS0              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP1 register  ******************/
#define  SDIO_RESP1_CARDSTATUS1              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP2 register  ******************/
#define  SDIO_RESP2_CARDSTATUS2              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP3 register  ******************/
#define  SDIO_RESP3_CARDSTATUS3              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP4 register  ******************/
#define  SDIO_RESP4_CARDSTATUS4              ((uint32_t)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_DTIMER register  *****************/
#define  SDIO_DTIMER_DATATIME                ((uint32_t)0xFFFFFFFF)        /*!<Data timeout period. */

/******************  Bit definition for SDIO_DLEN register  *******************/
#define  SDIO_DLEN_DATALENGTH                ((uint32_t)0x01FFFFFF)        /*!<Data length value    */

/******************  Bit definition for SDIO_DCTRL register  ******************/
#define  SDIO_DCTRL_DTEN                     ((uint32_t)0x0001)            /*!<Data transfer enabled bit         */
#define  SDIO_DCTRL_DTDIR                    ((uint32_t)0x0002)            /*!<Data transfer direction selection */
#define  SDIO_DCTRL_DTMODE                   ((uint32_t)0x0004)            /*!<Data transfer mode selection      */
#define  SDIO_DCTRL_DMAEN                    ((uint32_t)0x0008)            /*!<DMA enabled bit                   */

#define  SDIO_DCTRL_DBLOCKSIZE               ((uint32_t)0x00F0)            /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DCTRL_DBLOCKSIZE_0             ((uint32_t)0x0010)            /*!<Bit 0 */
#define  SDIO_DCTRL_DBLOCKSIZE_1             ((uint32_t)0x0020)            /*!<Bit 1 */
#define  SDIO_DCTRL_DBLOCKSIZE_2             ((uint32_t)0x0040)            /*!<Bit 2 */
#define  SDIO_DCTRL_DBLOCKSIZE_3             ((uint32_t)0x0080)            /*!<Bit 3 */

#define  SDIO_DCTRL_RWSTART                  ((uint32_t)0x0100)            /*!<Read wait start         */
#define  SDIO_DCTRL_RWSTOP                   ((uint32_t)0x0200)            /*!<Read wait stop          */
#define  SDIO_DCTRL_RWMOD                    ((uint32_t)0x0400)            /*!<Read wait mode          */
#define  SDIO_DCTRL_SDIOEN                   ((uint32_t)0x0800)            /*!<SD I/O enable functions */

/******************  Bit definition for SDIO_DCOUNT register  *****************/
#define  SDIO_DCOUNT_DATACOUNT               ((uint32_t)0x01FFFFFF)        /*!<Data count value */

/******************  Bit definition for SDIO_STA register  ********************/
#define  SDIO_STA_CCRCFAIL                   ((uint32_t)0x00000001)        /*!<Command response received (CRC check failed)  */
#define  SDIO_STA_DCRCFAIL                   ((uint32_t)0x00000002)        /*!<Data block sent/received (CRC check failed)   */
#define  SDIO_STA_CTIMEOUT                   ((uint32_t)0x00000004)        /*!<Command response timeout                      */
#define  SDIO_STA_DTIMEOUT                   ((uint32_t)0x00000008)        /*!<Data timeout                                  */
#define  SDIO_STA_TXUNDERR                   ((uint32_t)0x00000010)        /*!<Transmit FIFO underrun error                  */
#define  SDIO_STA_RXOVERR                    ((uint32_t)0x00000020)        /*!<Received FIFO overrun error                   */
#define  SDIO_STA_CMDREND                    ((uint32_t)0x00000040)        /*!<Command response received (CRC check passed)  */
#define  SDIO_STA_CMDSENT                    ((uint32_t)0x00000080)        /*!<Command sent (no response required)           */
#define  SDIO_STA_DATAEND                    ((uint32_t)0x00000100)        /*!<Data end (data counter, SDIDCOUNT, is zero)   */
#define  SDIO_STA_STBITERR                   ((uint32_t)0x00000200)        /*!<Start bit not detected on all data signals in wide bus mode */
#define  SDIO_STA_DBCKEND                    ((uint32_t)0x00000400)        /*!<Data block sent/received (CRC check passed)   */
#define  SDIO_STA_CMDACT                     ((uint32_t)0x00000800)        /*!<Command transfer in progress                  */
#define  SDIO_STA_TXACT                      ((uint32_t)0x00001000)        /*!<Data transmit in progress                     */
#define  SDIO_STA_RXACT                      ((uint32_t)0x00002000)        /*!<Data receive in progress                      */
#define  SDIO_STA_TXFIFOHE                   ((uint32_t)0x00004000)        /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDIO_STA_RXFIFOHF                   ((uint32_t)0x00008000)        /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDIO_STA_TXFIFOF                    ((uint32_t)0x00010000)        /*!<Transmit FIFO full                            */
#define  SDIO_STA_RXFIFOF                    ((uint32_t)0x00020000)        /*!<Receive FIFO full                             */
#define  SDIO_STA_TXFIFOE                    ((uint32_t)0x00040000)        /*!<Transmit FIFO empty                           */
#define  SDIO_STA_RXFIFOE                    ((uint32_t)0x00080000)        /*!<Receive FIFO empty                            */
#define  SDIO_STA_TXDAVL                     ((uint32_t)0x00100000)        /*!<Data available in transmit FIFO               */
#define  SDIO_STA_RXDAVL                     ((uint32_t)0x00200000)        /*!<Data available in receive FIFO                */
#define  SDIO_STA_SDIOIT                     ((uint32_t)0x00400000)        /*!<SDIO interrupt received                       */
#define  SDIO_STA_CEATAEND                   ((uint32_t)0x00800000)        /*!<CE-ATA command completion signal received for CMD61 */

/*******************  Bit definition for SDIO_ICR register  *******************/
#define  SDIO_ICR_CCRCFAILC                  ((uint32_t)0x00000001)        /*!<CCRCFAIL flag clear bit */
#define  SDIO_ICR_DCRCFAILC                  ((uint32_t)0x00000002)        /*!<DCRCFAIL flag clear bit */
#define  SDIO_ICR_CTIMEOUTC                  ((uint32_t)0x00000004)        /*!<CTIMEOUT flag clear bit */
#define  SDIO_ICR_DTIMEOUTC                  ((uint32_t)0x00000008)        /*!<DTIMEOUT flag clear bit */
#define  SDIO_ICR_TXUNDERRC                  ((uint32_t)0x00000010)        /*!<TXUNDERR flag clear bit */
#define  SDIO_ICR_RXOVERRC                   ((uint32_t)0x00000020)        /*!<RXOVERR flag clear bit  */
#define  SDIO_ICR_CMDRENDC                   ((uint32_t)0x00000040)        /*!<CMDREND flag clear bit  */
#define  SDIO_ICR_CMDSENTC                   ((uint32_t)0x00000080)        /*!<CMDSENT flag clear bit  */
#define  SDIO_ICR_DATAENDC                   ((uint32_t)0x00000100)        /*!<DATAEND flag clear bit  */
#define  SDIO_ICR_STBITERRC                  ((uint32_t)0x00000200)        /*!<STBITERR flag clear bit */
#define  SDIO_ICR_DBCKENDC                   ((uint32_t)0x00000400)        /*!<DBCKEND flag clear bit  */
#define  SDIO_ICR_SDIOITC                    ((uint32_t)0x00400000)        /*!<SDIOIT flag clear bit   */
#define  SDIO_ICR_CEATAENDC                  ((uint32_t)0x00800000)        /*!<CEATAEND flag clear bit */

/******************  Bit definition for SDIO_MASK register  *******************/
#define  SDIO_MASK_CCRCFAILIE                ((uint32_t)0x00000001)        /*!<Command CRC Fail Interrupt Enable          */
#define  SDIO_MASK_DCRCFAILIE                ((uint32_t)0x00000002)        /*!<Data CRC Fail Interrupt Enable             */
#define  SDIO_MASK_CTIMEOUTIE                ((uint32_t)0x00000004)        /*!<Command TimeOut Interrupt Enable           */
#define  SDIO_MASK_DTIMEOUTIE                ((uint32_t)0x00000008)        /*!<Data TimeOut Interrupt Enable              */
#define  SDIO_MASK_TXUNDERRIE                ((uint32_t)0x00000010)        /*!<Tx FIFO UnderRun Error Interrupt Enable    */
#define  SDIO_MASK_RXOVERRIE                 ((uint32_t)0x00000020)        /*!<Rx FIFO OverRun Error Interrupt Enable     */
#define  SDIO_MASK_CMDRENDIE                 ((uint32_t)0x00000040)        /*!<Command Response Received Interrupt Enable */
#define  SDIO_MASK_CMDSENTIE                 ((uint32_t)0x00000080)        /*!<Command Sent Interrupt Enable              */
#define  SDIO_MASK_DATAENDIE                 ((uint32_t)0x00000100)        /*!<Data End Interrupt Enable                  */
#define  SDIO_MASK_STBITERRIE                ((uint32_t)0x00000200)        /*!<Start Bit Error Interrupt Enable           */
#define  SDIO_MASK_DBCKENDIE                 ((uint32_t)0x00000400)        /*!<Data Block End Interrupt Enable            */
#define  SDIO_MASK_CMDACTIE                  ((uint32_t)0x00000800)        /*!<CCommand Acting Interrupt Enable           */
#define  SDIO_MASK_TXACTIE                   ((uint32_t)0x00001000)        /*!<Data Transmit Acting Interrupt Enable      */
#define  SDIO_MASK_RXACTIE                   ((uint32_t)0x00002000)        /*!<Data receive acting interrupt enabled      */
#define  SDIO_MASK_TXFIFOHEIE                ((uint32_t)0x00004000)        /*!<Tx FIFO Half Empty interrupt Enable        */
#define  SDIO_MASK_RXFIFOHFIE                ((uint32_t)0x00008000)        /*!<Rx FIFO Half Full interrupt Enable         */
#define  SDIO_MASK_TXFIFOFIE                 ((uint32_t)0x00010000)        /*!<Tx FIFO Full interrupt Enable              */
#define  SDIO_MASK_RXFIFOFIE                 ((uint32_t)0x00020000)        /*!<Rx FIFO Full interrupt Enable              */
#define  SDIO_MASK_TXFIFOEIE                 ((uint32_t)0x00040000)        /*!<Tx FIFO Empty interrupt Enable             */
#define  SDIO_MASK_RXFIFOEIE                 ((uint32_t)0x00080000)        /*!<Rx FIFO Empty interrupt Enable             */
#define  SDIO_MASK_TXDAVLIE                  ((uint32_t)0x00100000)        /*!<Data available in Tx FIFO interrupt Enable */
#define  SDIO_MASK_RXDAVLIE                  ((uint32_t)0x00200000)        /*!<Data available in Rx FIFO interrupt Enable */
#define  SDIO_MASK_SDIOITIE                  ((uint32_t)0x00400000)        /*!<SDIO Mode Interrupt Received interrupt Enable */
#define  SDIO_MASK_CEATAENDIE                ((uint32_t)0x00800000)        /*!<CE-ATA command completion signal received Interrupt Enable */

/*****************  Bit definition for SDIO_FIFOCNT register  *****************/
#define  SDIO_FIFOCNT_FIFOCOUNT              ((uint32_t)0x00FFFFFF)        /*!<Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDIO_FIFO register  *******************/
#define  SDIO_FIFO_FIFODATA                  ((uint32_t)0xFFFFFFFF)        /*!<Receive and transmit FIFO data */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((uint32_t)0x00000001)            /*!<Clock Phase      */
#define  SPI_CR1_CPOL                        ((uint32_t)0x00000002)            /*!<Clock Polarity   */
#define  SPI_CR1_MSTR                        ((uint32_t)0x00000004)            /*!<Master Selection */

#define  SPI_CR1_BR                          ((uint32_t)0x00000038)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((uint32_t)0x00000008)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        ((uint32_t)0x00000010)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        ((uint32_t)0x00000020)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         ((uint32_t)0x00000040)            /*!<SPI Enable                          */
#define  SPI_CR1_LSBFIRST                    ((uint32_t)0x00000080)            /*!<Frame Format                        */
#define  SPI_CR1_SSI                         ((uint32_t)0x00000100)            /*!<Internal slave select               */
#define  SPI_CR1_SSM                         ((uint32_t)0x00000200)            /*!<Software slave management           */
#define  SPI_CR1_RXONLY                      ((uint32_t)0x00000400)            /*!<Receive only                        */
#define  SPI_CR1_DFF                         ((uint32_t)0x00000800)            /*!<Data Frame Format                   */
#define  SPI_CR1_CRCNEXT                     ((uint32_t)0x00001000)            /*!<Transmit CRC next                   */
#define  SPI_CR1_CRCEN                       ((uint32_t)0x00002000)            /*!<Hardware CRC calculation enable     */
#define  SPI_CR1_BIDIOE                      ((uint32_t)0x00004000)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((uint32_t)0x00008000)            /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((uint32_t)0x00000001)               /*!<Rx Buffer DMA Enable                 */
#define  SPI_CR2_TXDMAEN                     ((uint32_t)0x00000002)               /*!<Tx Buffer DMA Enable                 */
#define  SPI_CR2_SSOE                        ((uint32_t)0x00000004)               /*!<SS Output Enable                     */
#define  SPI_CR2_FRF                         ((uint32_t)0x00000010)               /*!<Frame Format                         */
#define  SPI_CR2_ERRIE                       ((uint32_t)0x00000020)               /*!<Error Interrupt Enable               */
#define  SPI_CR2_RXNEIE                      ((uint32_t)0x00000040)               /*!<RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((uint32_t)0x00000080)               /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((uint32_t)0x00000001)               /*!<Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((uint32_t)0x00000002)               /*!<Transmit buffer Empty    */
#define  SPI_SR_CHSIDE                       ((uint32_t)0x00000004)               /*!<Channel side             */
#define  SPI_SR_UDR                          ((uint32_t)0x00000008)               /*!<Underrun flag            */
#define  SPI_SR_CRCERR                       ((uint32_t)0x00000010)               /*!<CRC Error flag           */
#define  SPI_SR_MODF                         ((uint32_t)0x00000020)               /*!<Mode fault               */
#define  SPI_SR_OVR                          ((uint32_t)0x00000040)               /*!<Overrun flag             */
#define  SPI_SR_BSY                          ((uint32_t)0x00000080)               /*!<Busy flag                */
#define  SPI_SR_FRE                          ((uint32_t)0x00000100)               /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((uint32_t)0x0000FFFF)            /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((uint32_t)0x0000FFFF)            /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((uint32_t)0x0000FFFF)            /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((uint32_t)0x0000FFFF)            /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((uint32_t)0x00000001)            /*!<Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  ((uint32_t)0x00000006)            /*!<DATLEN[1:0] bits (Data length to be transferred)  */
#define  SPI_I2SCFGR_DATLEN_0                ((uint32_t)0x00000002)            /*!<Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((uint32_t)0x00000004)            /*!<Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   ((uint32_t)0x00000008)            /*!<steady state clock polarity               */

#define  SPI_I2SCFGR_I2SSTD                  ((uint32_t)0x00000030)            /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((uint32_t)0x00000020)            /*!<Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 ((uint32_t)0x00000080)            /*!<PCM frame synchronization                 */

#define  SPI_I2SCFGR_I2SCFG                  ((uint32_t)0x00000300)            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  SPI_I2SCFGR_I2SE                    ((uint32_t)0x00000400)            /*!<I2S Enable         */
#define  SPI_I2SCFGR_I2SMOD                  ((uint32_t)0x00000800)            /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((uint32_t)0x000000FF)            /*!<I2S Linear prescaler         */
#define  SPI_I2SPR_ODD                       ((uint32_t)0x00000100)            /*!<Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((uint32_t)0x00000200)            /*!<Master Clock Output Enable   */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/  
#define SYSCFG_MEMRMP_MEM_MODE          ((uint32_t)0x00000007) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0        ((uint32_t)0x00000001)
#define SYSCFG_MEMRMP_MEM_MODE_1        ((uint32_t)0x00000002)
#define SYSCFG_MEMRMP_MEM_MODE_2        ((uint32_t)0x00000004)

/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_MII_RMII_SEL         ((uint32_t)0x00800000) /*!<Ethernet PHY interface selection */
/* Old MII_RMII_SEL bit definition, maintained for legacy purpose */
#define SYSCFG_PMC_MII_RMII             SYSCFG_PMC_MII_RMII_SEL

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            ((uint32_t)0x000F) /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            ((uint32_t)0x00F0) /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            ((uint32_t)0x0F00) /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            ((uint32_t)0xF000) /*!<EXTI 3 configuration */
/** 
  * @brief   EXTI0 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI0_PA         ((uint32_t)0x0000) /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         ((uint32_t)0x0001) /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         ((uint32_t)0x0002) /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD         ((uint32_t)0x0003) /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE         ((uint32_t)0x0004) /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF         ((uint32_t)0x0005) /*!<PF[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PG         ((uint32_t)0x0006) /*!<PG[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH         ((uint32_t)0x0007) /*!<PH[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PI         ((uint32_t)0x0008) /*!<PI[0] pin */

/** 
  * @brief   EXTI1 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI1_PA         ((uint32_t)0x0000) /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         ((uint32_t)0x0010) /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         ((uint32_t)0x0020) /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD         ((uint32_t)0x0030) /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE         ((uint32_t)0x0040) /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF         ((uint32_t)0x0050) /*!<PF[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PG         ((uint32_t)0x0060) /*!<PG[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH         ((uint32_t)0x0070) /*!<PH[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PI         ((uint32_t)0x0080) /*!<PI[1] pin */

/** 
  * @brief   EXTI2 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI2_PA         ((uint32_t)0x0000) /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         ((uint32_t)0x0100) /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         ((uint32_t)0x0200) /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD         ((uint32_t)0x0300) /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE         ((uint32_t)0x0400) /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF         ((uint32_t)0x0500) /*!<PF[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PG         ((uint32_t)0x0600) /*!<PG[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH         ((uint32_t)0x0700) /*!<PH[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PI         ((uint32_t)0x0800) /*!<PI[2] pin */

/** 
  * @brief   EXTI3 configuration  
  */ 
#define SYSCFG_EXTICR1_EXTI3_PA         ((uint32_t)0x0000) /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         ((uint32_t)0x1000) /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         ((uint32_t)0x2000) /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD         ((uint32_t)0x3000) /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE         ((uint32_t)0x4000) /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PF         ((uint32_t)0x5000) /*!<PF[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PG         ((uint32_t)0x6000) /*!<PG[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH         ((uint32_t)0x7000) /*!<PH[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PI         ((uint32_t)0x8000) /*!<PI[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            ((uint32_t)0x000F) /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            ((uint32_t)0x00F0) /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            ((uint32_t)0x0F00) /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            ((uint32_t)0xF000) /*!<EXTI 7 configuration */
/** 
  * @brief   EXTI4 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI4_PA         ((uint32_t)0x0000) /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         ((uint32_t)0x0001) /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         ((uint32_t)0x0002) /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD         ((uint32_t)0x0003) /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE         ((uint32_t)0x0004) /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF         ((uint32_t)0x0005) /*!<PF[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PG         ((uint32_t)0x0006) /*!<PG[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH         ((uint32_t)0x0007) /*!<PH[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PI         ((uint32_t)0x0008) /*!<PI[4] pin */

/** 
  * @brief   EXTI5 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI5_PA         ((uint32_t)0x0000) /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         ((uint32_t)0x0010) /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         ((uint32_t)0x0020) /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD         ((uint32_t)0x0030) /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE         ((uint32_t)0x0040) /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF         ((uint32_t)0x0050) /*!<PF[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PG         ((uint32_t)0x0060) /*!<PG[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH         ((uint32_t)0x0070) /*!<PH[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PI         ((uint32_t)0x0080) /*!<PI[5] pin */

/** 
  * @brief   EXTI6 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI6_PA         ((uint32_t)0x0000) /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         ((uint32_t)0x0100) /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         ((uint32_t)0x0200) /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD         ((uint32_t)0x0300) /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE         ((uint32_t)0x0400) /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF         ((uint32_t)0x0500) /*!<PF[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PG         ((uint32_t)0x0600) /*!<PG[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH         ((uint32_t)0x0700) /*!<PH[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PI         ((uint32_t)0x0800) /*!<PI[6] pin */

/** 
  * @brief   EXTI7 configuration  
  */ 
#define SYSCFG_EXTICR2_EXTI7_PA         ((uint32_t)0x0000) /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         ((uint32_t)0x1000) /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         ((uint32_t)0x2000) /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD         ((uint32_t)0x3000) /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE         ((uint32_t)0x4000) /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PF         ((uint32_t)0x5000) /*!<PF[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PG         ((uint32_t)0x6000) /*!<PG[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH         ((uint32_t)0x7000) /*!<PH[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PI         ((uint32_t)0x8000) /*!<PI[7] pin */


/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            ((uint32_t)0x000F) /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            ((uint32_t)0x00F0) /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           ((uint32_t)0x0F00) /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           ((uint32_t)0xF000) /*!<EXTI 11 configuration */
           
/** 
  * @brief   EXTI8 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI8_PA         ((uint32_t)0x0000) /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         ((uint32_t)0x0001) /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         ((uint32_t)0x0002) /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD         ((uint32_t)0x0003) /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE         ((uint32_t)0x0004) /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PF         ((uint32_t)0x0005) /*!<PF[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PG         ((uint32_t)0x0006) /*!<PG[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH         ((uint32_t)0x0007) /*!<PH[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PI         ((uint32_t)0x0008) /*!<PI[8] pin */

/** 
  * @brief   EXTI9 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI9_PA         ((uint32_t)0x0000) /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         ((uint32_t)0x0010) /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         ((uint32_t)0x0020) /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD         ((uint32_t)0x0030) /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE         ((uint32_t)0x0040) /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF         ((uint32_t)0x0050) /*!<PF[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PG         ((uint32_t)0x0060) /*!<PG[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH         ((uint32_t)0x0070) /*!<PH[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PI         ((uint32_t)0x0080) /*!<PI[9] pin */

/** 
  * @brief   EXTI10 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI10_PA        ((uint32_t)0x0000) /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        ((uint32_t)0x0100) /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        ((uint32_t)0x0200) /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD        ((uint32_t)0x0300) /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE        ((uint32_t)0x0400) /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF        ((uint32_t)0x0500) /*!<PF[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PG        ((uint32_t)0x0600) /*!<PG[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH        ((uint32_t)0x0700) /*!<PH[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PI        ((uint32_t)0x0800) /*!<PI[10] pin */

/** 
  * @brief   EXTI11 configuration  
  */ 
#define SYSCFG_EXTICR3_EXTI11_PA        ((uint32_t)0x0000) /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        ((uint32_t)0x1000) /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        ((uint32_t)0x2000) /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD        ((uint32_t)0x3000) /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE        ((uint32_t)0x4000) /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PF        ((uint32_t)0x5000) /*!<PF[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PG        ((uint32_t)0x6000) /*!<PG[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH        ((uint32_t)0x7000) /*!<PH[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PI        ((uint32_t)0x8000) /*!<PI[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12           ((uint32_t)0x000F) /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           ((uint32_t)0x00F0) /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           ((uint32_t)0x0F00) /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           ((uint32_t)0xF000) /*!<EXTI 15 configuration */
/** 
  * @brief   EXTI12 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI12_PA        ((uint32_t)0x0000) /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        ((uint32_t)0x0001) /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        ((uint32_t)0x0002) /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD        ((uint32_t)0x0003) /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE        ((uint32_t)0x0004) /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PF        ((uint32_t)0x0005) /*!<PF[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PG        ((uint32_t)0x0006) /*!<PG[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH        ((uint32_t)0x0007) /*!<PH[12] pin */

/** 
  * @brief   EXTI13 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI13_PA        ((uint32_t)0x0000) /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        ((uint32_t)0x0010) /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        ((uint32_t)0x0020) /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD        ((uint32_t)0x0030) /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE        ((uint32_t)0x0040) /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PF        ((uint32_t)0x0050) /*!<PF[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PG        ((uint32_t)0x0060) /*!<PG[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH        ((uint32_t)0x0070) /*!<PH[13] pin */

/** 
  * @brief   EXTI14 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI14_PA        ((uint32_t)0x0000) /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        ((uint32_t)0x0100) /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        ((uint32_t)0x0200) /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD        ((uint32_t)0x0300) /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE        ((uint32_t)0x0400) /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PF        ((uint32_t)0x0500) /*!<PF[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PG        ((uint32_t)0x0600) /*!<PG[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH        ((uint32_t)0x0700) /*!<PH[14] pin */

/** 
  * @brief   EXTI15 configuration  
  */ 
#define SYSCFG_EXTICR4_EXTI15_PA        ((uint32_t)0x0000) /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        ((uint32_t)0x1000) /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        ((uint32_t)0x2000) /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD        ((uint32_t)0x3000) /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE        ((uint32_t)0x4000) /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PF        ((uint32_t)0x5000) /*!<PF[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PG        ((uint32_t)0x6000) /*!<PG[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH        ((uint32_t)0x7000) /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/  
#define SYSCFG_CMPCR_CMP_PD             ((uint32_t)0x00000001) /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY              ((uint32_t)0x00000100) /*!<Compensation cell power-down */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         ((uint32_t)0x0001)            /*!<Counter enable        */
#define  TIM_CR1_UDIS                        ((uint32_t)0x0002)            /*!<Update disable        */
#define  TIM_CR1_URS                         ((uint32_t)0x0004)            /*!<Update request source */
#define  TIM_CR1_OPM                         ((uint32_t)0x0008)            /*!<One pulse mode        */
#define  TIM_CR1_DIR                         ((uint32_t)0x0010)            /*!<Direction             */

#define  TIM_CR1_CMS                         ((uint32_t)0x0060)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       ((uint32_t)0x0020)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1                       ((uint32_t)0x0040)            /*!<Bit 1 */

#define  TIM_CR1_ARPE                        ((uint32_t)0x0080)            /*!<Auto-reload preload enable     */

#define  TIM_CR1_CKD                         ((uint32_t)0x0300)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       ((uint32_t)0x0100)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1                       ((uint32_t)0x0200)            /*!<Bit 1 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        ((uint32_t)0x0001)            /*!<Capture/Compare Preloaded Control        */
#define  TIM_CR2_CCUS                        ((uint32_t)0x0004)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        ((uint32_t)0x0008)            /*!<Capture/Compare DMA Selection            */

#define  TIM_CR2_MMS                         ((uint32_t)0x0070)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       ((uint32_t)0x0010)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1                       ((uint32_t)0x0020)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2                       ((uint32_t)0x0040)            /*!<Bit 2 */

#define  TIM_CR2_TI1S                        ((uint32_t)0x0080)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1                        ((uint32_t)0x0100)            /*!<Output Idle state 1 (OC1 output)  */
#define  TIM_CR2_OIS1N                       ((uint32_t)0x0200)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        ((uint32_t)0x0400)            /*!<Output Idle state 2 (OC2 output)  */
#define  TIM_CR2_OIS2N                       ((uint32_t)0x0800)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        ((uint32_t)0x1000)            /*!<Output Idle state 3 (OC3 output)  */
#define  TIM_CR2_OIS3N                       ((uint32_t)0x2000)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        ((uint32_t)0x4000)            /*!<Output Idle state 4 (OC4 output)  */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        ((uint32_t)0x0007)            /*!<SMS[2:0] bits (Slave mode selection)    */
#define  TIM_SMCR_SMS_0                      ((uint32_t)0x0001)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1                      ((uint32_t)0x0002)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2                      ((uint32_t)0x0004)            /*!<Bit 2 */

#define  TIM_SMCR_TS                         ((uint32_t)0x0070)            /*!<TS[2:0] bits (Trigger selection)        */
#define  TIM_SMCR_TS_0                       ((uint32_t)0x0010)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1                       ((uint32_t)0x0020)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2                       ((uint32_t)0x0040)            /*!<Bit 2 */

#define  TIM_SMCR_MSM                        ((uint32_t)0x0080)            /*!<Master/slave mode                       */

#define  TIM_SMCR_ETF                        ((uint32_t)0x0F00)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      ((uint32_t)0x0100)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1                      ((uint32_t)0x0200)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2                      ((uint32_t)0x0400)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3                      ((uint32_t)0x0800)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS                       ((uint32_t)0x3000)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     ((uint32_t)0x1000)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1                     ((uint32_t)0x2000)            /*!<Bit 1 */

#define  TIM_SMCR_ECE                        ((uint32_t)0x4000)            /*!<External clock enable     */
#define  TIM_SMCR_ETP                        ((uint32_t)0x8000)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        ((uint32_t)0x0001)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE                      ((uint32_t)0x0002)            /*!<Capture/Compare 1 interrupt enable   */
#define  TIM_DIER_CC2IE                      ((uint32_t)0x0004)            /*!<Capture/Compare 2 interrupt enable   */
#define  TIM_DIER_CC3IE                      ((uint32_t)0x0008)            /*!<Capture/Compare 3 interrupt enable   */
#define  TIM_DIER_CC4IE                      ((uint32_t)0x0010)            /*!<Capture/Compare 4 interrupt enable   */
#define  TIM_DIER_COMIE                      ((uint32_t)0x0020)            /*!<COM interrupt enable                 */
#define  TIM_DIER_TIE                        ((uint32_t)0x0040)            /*!<Trigger interrupt enable             */
#define  TIM_DIER_BIE                        ((uint32_t)0x0080)            /*!<Break interrupt enable               */
#define  TIM_DIER_UDE                        ((uint32_t)0x0100)            /*!<Update DMA request enable            */
#define  TIM_DIER_CC1DE                      ((uint32_t)0x0200)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      ((uint32_t)0x0400)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      ((uint32_t)0x0800)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      ((uint32_t)0x1000)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      ((uint32_t)0x2000)            /*!<COM DMA request enable               */
#define  TIM_DIER_TDE                        ((uint32_t)0x4000)            /*!<Trigger DMA request enable           */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          ((uint32_t)0x0001)            /*!<Update interrupt Flag              */
#define  TIM_SR_CC1IF                        ((uint32_t)0x0002)            /*!<Capture/Compare 1 interrupt Flag   */
#define  TIM_SR_CC2IF                        ((uint32_t)0x0004)            /*!<Capture/Compare 2 interrupt Flag   */
#define  TIM_SR_CC3IF                        ((uint32_t)0x0008)            /*!<Capture/Compare 3 interrupt Flag   */
#define  TIM_SR_CC4IF                        ((uint32_t)0x0010)            /*!<Capture/Compare 4 interrupt Flag   */
#define  TIM_SR_COMIF                        ((uint32_t)0x0020)            /*!<COM interrupt Flag                 */
#define  TIM_SR_TIF                          ((uint32_t)0x0040)            /*!<Trigger interrupt Flag             */
#define  TIM_SR_BIF                          ((uint32_t)0x0080)            /*!<Break interrupt Flag               */
#define  TIM_SR_CC1OF                        ((uint32_t)0x0200)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        ((uint32_t)0x0400)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        ((uint32_t)0x0800)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        ((uint32_t)0x1000)            /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          ((uint32_t)0x01)               /*!<Update Generation                         */
#define  TIM_EGR_CC1G                        ((uint32_t)0x02)               /*!<Capture/Compare 1 Generation              */
#define  TIM_EGR_CC2G                        ((uint32_t)0x04)               /*!<Capture/Compare 2 Generation              */
#define  TIM_EGR_CC3G                        ((uint32_t)0x08)               /*!<Capture/Compare 3 Generation              */
#define  TIM_EGR_CC4G                        ((uint32_t)0x10)               /*!<Capture/Compare 4 Generation              */
#define  TIM_EGR_COMG                        ((uint32_t)0x20)               /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          ((uint32_t)0x40)               /*!<Trigger Generation                        */
#define  TIM_EGR_BG                          ((uint32_t)0x80)               /*!<Break Generation                          */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      ((uint32_t)0x0003)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    ((uint32_t)0x0001)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1                    ((uint32_t)0x0002)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE                     ((uint32_t)0x0004)            /*!<Output Compare 1 Fast enable                 */
#define  TIM_CCMR1_OC1PE                     ((uint32_t)0x0008)            /*!<Output Compare 1 Preload enable              */

#define  TIM_CCMR1_OC1M                      ((uint32_t)0x0070)            /*!<OC1M[2:0] bits (Output Compare 1 Mode)       */
#define  TIM_CCMR1_OC1M_0                    ((uint32_t)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1                    ((uint32_t)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2                    ((uint32_t)0x0040)            /*!<Bit 2 */

#define  TIM_CCMR1_OC1CE                     ((uint32_t)0x0080)            /*!<Output Compare 1Clear Enable                 */

#define  TIM_CCMR1_CC2S                      ((uint32_t)0x0300)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    ((uint32_t)0x0100)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1                    ((uint32_t)0x0200)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE                     ((uint32_t)0x0400)            /*!<Output Compare 2 Fast enable                 */
#define  TIM_CCMR1_OC2PE                     ((uint32_t)0x0800)            /*!<Output Compare 2 Preload enable              */

#define  TIM_CCMR1_OC2M                      ((uint32_t)0x7000)            /*!<OC2M[2:0] bits (Output Compare 2 Mode)       */
#define  TIM_CCMR1_OC2M_0                    ((uint32_t)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1                    ((uint32_t)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2                    ((uint32_t)0x4000)            /*!<Bit 2 */

#define  TIM_CCMR1_OC2CE                     ((uint32_t)0x8000)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    ((uint32_t)0x000C)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  ((uint32_t)0x0004)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  ((uint32_t)0x0008)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F                      ((uint32_t)0x00F0)            /*!<IC1F[3:0] bits (Input Capture 1 Filter)      */
#define  TIM_CCMR1_IC1F_0                    ((uint32_t)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1                    ((uint32_t)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2                    ((uint32_t)0x0040)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3                    ((uint32_t)0x0080)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC                    ((uint32_t)0x0C00)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)  */
#define  TIM_CCMR1_IC2PSC_0                  ((uint32_t)0x0400)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  ((uint32_t)0x0800)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F                      ((uint32_t)0xF000)            /*!<IC2F[3:0] bits (Input Capture 2 Filter)       */
#define  TIM_CCMR1_IC2F_0                    ((uint32_t)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1                    ((uint32_t)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2                    ((uint32_t)0x4000)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3                    ((uint32_t)0x8000)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      ((uint32_t)0x0003)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection)  */
#define  TIM_CCMR2_CC3S_0                    ((uint32_t)0x0001)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1                    ((uint32_t)0x0002)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE                     ((uint32_t)0x0004)            /*!<Output Compare 3 Fast enable           */
#define  TIM_CCMR2_OC3PE                     ((uint32_t)0x0008)            /*!<Output Compare 3 Preload enable        */

#define  TIM_CCMR2_OC3M                      ((uint32_t)0x0070)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    ((uint32_t)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1                    ((uint32_t)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2                    ((uint32_t)0x0040)            /*!<Bit 2 */

#define  TIM_CCMR2_OC3CE                     ((uint32_t)0x0080)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      ((uint32_t)0x0300)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    ((uint32_t)0x0100)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1                    ((uint32_t)0x0200)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE                     ((uint32_t)0x0400)            /*!<Output Compare 4 Fast enable    */
#define  TIM_CCMR2_OC4PE                     ((uint32_t)0x0800)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      ((uint32_t)0x7000)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    ((uint32_t)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1                    ((uint32_t)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2                    ((uint32_t)0x4000)            /*!<Bit 2 */

#define  TIM_CCMR2_OC4CE                     ((uint32_t)0x8000)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    ((uint32_t)0x000C)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  ((uint32_t)0x0004)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  ((uint32_t)0x0008)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F                      ((uint32_t)0x00F0)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    ((uint32_t)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1                    ((uint32_t)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2                    ((uint32_t)0x0040)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3                    ((uint32_t)0x0080)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC                    ((uint32_t)0x0C00)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  ((uint32_t)0x0400)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  ((uint32_t)0x0800)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F                      ((uint32_t)0xF000)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    ((uint32_t)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1                    ((uint32_t)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2                    ((uint32_t)0x4000)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3                    ((uint32_t)0x8000)            /*!<Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       ((uint32_t)0x0001)            /*!<Capture/Compare 1 output enable                 */
#define  TIM_CCER_CC1P                       ((uint32_t)0x0002)            /*!<Capture/Compare 1 output Polarity               */
#define  TIM_CCER_CC1NE                      ((uint32_t)0x0004)            /*!<Capture/Compare 1 Complementary output enable   */
#define  TIM_CCER_CC1NP                      ((uint32_t)0x0008)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       ((uint32_t)0x0010)            /*!<Capture/Compare 2 output enable                 */
#define  TIM_CCER_CC2P                       ((uint32_t)0x0020)            /*!<Capture/Compare 2 output Polarity               */
#define  TIM_CCER_CC2NE                      ((uint32_t)0x0040)            /*!<Capture/Compare 2 Complementary output enable   */
#define  TIM_CCER_CC2NP                      ((uint32_t)0x0080)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       ((uint32_t)0x0100)            /*!<Capture/Compare 3 output enable                 */
#define  TIM_CCER_CC3P                       ((uint32_t)0x0200)            /*!<Capture/Compare 3 output Polarity               */
#define  TIM_CCER_CC3NE                      ((uint32_t)0x0400)            /*!<Capture/Compare 3 Complementary output enable   */
#define  TIM_CCER_CC3NP                      ((uint32_t)0x0800)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       ((uint32_t)0x1000)            /*!<Capture/Compare 4 output enable                 */
#define  TIM_CCER_CC4P                       ((uint32_t)0x2000)            /*!<Capture/Compare 4 output Polarity               */
#define  TIM_CCER_CC4NP                      ((uint32_t)0x8000)            /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         ((uint32_t)0xFFFF)            /*!<Counter Value            */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         ((uint32_t)0xFFFF)            /*!<Prescaler Value          */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         ((uint32_t)0xFFFF)            /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         ((uint32_t)0xFF)               /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       ((uint32_t)0xFFFF)            /*!<Capture/Compare 1 Value  */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       ((uint32_t)0xFFFF)            /*!<Capture/Compare 2 Value  */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       ((uint32_t)0xFFFF)            /*!<Capture/Compare 3 Value  */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       ((uint32_t)0xFFFF)            /*!<Capture/Compare 4 Value  */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        ((uint32_t)0x00FF)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      ((uint32_t)0x0001)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1                      ((uint32_t)0x0002)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2                      ((uint32_t)0x0004)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3                      ((uint32_t)0x0008)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4                      ((uint32_t)0x0010)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5                      ((uint32_t)0x0020)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6                      ((uint32_t)0x0040)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7                      ((uint32_t)0x0080)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK                       ((uint32_t)0x0300)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     ((uint32_t)0x0100)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1                     ((uint32_t)0x0200)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI                       ((uint32_t)0x0400)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       ((uint32_t)0x0800)            /*!<Off-State Selection for Run mode  */
#define  TIM_BDTR_BKE                        ((uint32_t)0x1000)            /*!<Break enable                      */
#define  TIM_BDTR_BKP                        ((uint32_t)0x2000)            /*!<Break Polarity                    */
#define  TIM_BDTR_AOE                        ((uint32_t)0x4000)            /*!<Automatic Output enable           */
#define  TIM_BDTR_MOE                        ((uint32_t)0x8000)            /*!<Main Output enable                */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         ((uint32_t)0x001F)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       ((uint32_t)0x0001)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1                       ((uint32_t)0x0002)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2                       ((uint32_t)0x0004)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3                       ((uint32_t)0x0008)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4                       ((uint32_t)0x0010)            /*!<Bit 4 */

#define  TIM_DCR_DBL                         ((uint32_t)0x1F00)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       ((uint32_t)0x0100)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1                       ((uint32_t)0x0200)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2                       ((uint32_t)0x0400)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3                       ((uint32_t)0x0800)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4                       ((uint32_t)0x1000)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       ((uint32_t)0xFFFF)            /*!<DMA register for burst accesses                    */

/*******************  Bit definition for TIM_OR register  *********************/
#define TIM_OR_TI4_RMP                       ((uint32_t)0x00C0)            /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
#define TIM_OR_TI4_RMP_0                     ((uint32_t)0x0040)            /*!<Bit 0 */
#define TIM_OR_TI4_RMP_1                     ((uint32_t)0x0080)            /*!<Bit 1 */
#define TIM_OR_ITR1_RMP                      ((uint32_t)0x0C00)            /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
#define TIM_OR_ITR1_RMP_0                    ((uint32_t)0x0400)            /*!<Bit 0 */
#define TIM_OR_ITR1_RMP_1                    ((uint32_t)0x0800)            /*!<Bit 1 */


/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         ((uint32_t)0x0001)            /*!<Parity Error                 */
#define  USART_SR_FE                         ((uint32_t)0x0002)            /*!<Framing Error                */
#define  USART_SR_NE                         ((uint32_t)0x0004)            /*!<Noise Error Flag             */
#define  USART_SR_ORE                        ((uint32_t)0x0008)            /*!<OverRun Error                */
#define  USART_SR_IDLE                       ((uint32_t)0x0010)            /*!<IDLE line detected           */
#define  USART_SR_RXNE                       ((uint32_t)0x0020)            /*!<Read Data Register Not Empty */
#define  USART_SR_TC                         ((uint32_t)0x0040)            /*!<Transmission Complete        */
#define  USART_SR_TXE                        ((uint32_t)0x0080)            /*!<Transmit Data Register Empty */
#define  USART_SR_LBD                        ((uint32_t)0x0100)            /*!<LIN Break Detection Flag     */
#define  USART_SR_CTS                        ((uint32_t)0x0200)            /*!<CTS Flag                     */

/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         ((uint32_t)0x01FF)            /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              ((uint32_t)0x000F)            /*!<Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              ((uint32_t)0xFFF0)            /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       ((uint32_t)0x0001)            /*!<Send Break                             */
#define  USART_CR1_RWU                       ((uint32_t)0x0002)            /*!<Receiver wakeup                        */
#define  USART_CR1_RE                        ((uint32_t)0x0004)            /*!<Receiver Enable                        */
#define  USART_CR1_TE                        ((uint32_t)0x0008)            /*!<Transmitter Enable                     */
#define  USART_CR1_IDLEIE                    ((uint32_t)0x0010)            /*!<IDLE Interrupt Enable                  */
#define  USART_CR1_RXNEIE                    ((uint32_t)0x0020)            /*!<RXNE Interrupt Enable                  */
#define  USART_CR1_TCIE                      ((uint32_t)0x0040)            /*!<Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((uint32_t)0x0080)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PEIE                      ((uint32_t)0x0100)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PS                        ((uint32_t)0x0200)            /*!<Parity Selection                       */
#define  USART_CR1_PCE                       ((uint32_t)0x0400)            /*!<Parity Control Enable                  */
#define  USART_CR1_WAKE                      ((uint32_t)0x0800)            /*!<Wakeup method                          */
#define  USART_CR1_M                         ((uint32_t)0x1000)            /*!<Word length                            */
#define  USART_CR1_UE                        ((uint32_t)0x2000)            /*!<USART Enable                           */
#define  USART_CR1_OVER8                     ((uint32_t)0x8000)            /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       ((uint32_t)0x000F)            /*!<Address of the USART node            */
#define  USART_CR2_LBDL                      ((uint32_t)0x0020)            /*!<LIN Break Detection Length           */
#define  USART_CR2_LBDIE                     ((uint32_t)0x0040)            /*!<LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((uint32_t)0x0100)            /*!<Last Bit Clock pulse                 */
#define  USART_CR2_CPHA                      ((uint32_t)0x0200)            /*!<Clock Phase                          */
#define  USART_CR2_CPOL                      ((uint32_t)0x0400)            /*!<Clock Polarity                       */
#define  USART_CR2_CLKEN                     ((uint32_t)0x0800)            /*!<Clock Enable                         */

#define  USART_CR2_STOP                      ((uint32_t)0x3000)            /*!<STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((uint32_t)0x1000)            /*!<Bit 0 */
#define  USART_CR2_STOP_1                    ((uint32_t)0x2000)            /*!<Bit 1 */

#define  USART_CR2_LINEN                     ((uint32_t)0x4000)            /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((uint32_t)0x0001)            /*!<Error Interrupt Enable      */
#define  USART_CR3_IREN                      ((uint32_t)0x0002)            /*!<IrDA mode Enable            */
#define  USART_CR3_IRLP                      ((uint32_t)0x0004)            /*!<IrDA Low-Power              */
#define  USART_CR3_HDSEL                     ((uint32_t)0x0008)            /*!<Half-Duplex Selection       */
#define  USART_CR3_NACK                      ((uint32_t)0x0010)            /*!<Smartcard NACK enable       */
#define  USART_CR3_SCEN                      ((uint32_t)0x0020)            /*!<Smartcard mode enable       */
#define  USART_CR3_DMAR                      ((uint32_t)0x0040)            /*!<DMA Enable Receiver         */
#define  USART_CR3_DMAT                      ((uint32_t)0x0080)            /*!<DMA Enable Transmitter      */
#define  USART_CR3_RTSE                      ((uint32_t)0x0100)            /*!<RTS Enable                  */
#define  USART_CR3_CTSE                      ((uint32_t)0x0200)            /*!<CTS Enable                  */
#define  USART_CR3_CTSIE                     ((uint32_t)0x0400)            /*!<CTS Interrupt Enable        */
#define  USART_CR3_ONEBIT                    ((uint32_t)0x0800)            /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((uint32_t)0x00FF)            /*!<PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    ((uint32_t)0x0001)            /*!<Bit 0 */
#define  USART_GTPR_PSC_1                    ((uint32_t)0x0002)            /*!<Bit 1 */
#define  USART_GTPR_PSC_2                    ((uint32_t)0x0004)            /*!<Bit 2 */
#define  USART_GTPR_PSC_3                    ((uint32_t)0x0008)            /*!<Bit 3 */
#define  USART_GTPR_PSC_4                    ((uint32_t)0x0010)            /*!<Bit 4 */
#define  USART_GTPR_PSC_5                    ((uint32_t)0x0020)            /*!<Bit 5 */
#define  USART_GTPR_PSC_6                    ((uint32_t)0x0040)            /*!<Bit 6 */
#define  USART_GTPR_PSC_7                    ((uint32_t)0x0080)            /*!<Bit 7 */

#define  USART_GTPR_GT                       ((uint32_t)0xFF00)            /*!<Guard time value */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((uint32_t)0x7F)               /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          ((uint32_t)0x01)               /*!<Bit 0 */
#define  WWDG_CR_T1                          ((uint32_t)0x02)               /*!<Bit 1 */
#define  WWDG_CR_T2                          ((uint32_t)0x04)               /*!<Bit 2 */
#define  WWDG_CR_T3                          ((uint32_t)0x08)               /*!<Bit 3 */
#define  WWDG_CR_T4                          ((uint32_t)0x10)               /*!<Bit 4 */
#define  WWDG_CR_T5                          ((uint32_t)0x20)               /*!<Bit 5 */
#define  WWDG_CR_T6                          ((uint32_t)0x40)               /*!<Bit 6 */

#define  WWDG_CR_WDGA                        ((uint32_t)0x80)               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((uint32_t)0x007F)            /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         ((uint32_t)0x0001)            /*!<Bit 0 */
#define  WWDG_CFR_W1                         ((uint32_t)0x0002)            /*!<Bit 1 */
#define  WWDG_CFR_W2                         ((uint32_t)0x0004)            /*!<Bit 2 */
#define  WWDG_CFR_W3                         ((uint32_t)0x0008)            /*!<Bit 3 */
#define  WWDG_CFR_W4                         ((uint32_t)0x0010)            /*!<Bit 4 */
#define  WWDG_CFR_W5                         ((uint32_t)0x0020)            /*!<Bit 5 */
#define  WWDG_CFR_W6                         ((uint32_t)0x0040)            /*!<Bit 6 */

#define  WWDG_CFR_WDGTB                      ((uint32_t)0x0180)            /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     ((uint32_t)0x0080)            /*!<Bit 0 */
#define  WWDG_CFR_WDGTB1                     ((uint32_t)0x0100)            /*!<Bit 1 */

#define  WWDG_CFR_EWI                        ((uint32_t)0x0200)            /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((uint32_t)0x01)               /*!<Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                DBG                                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define  DBGMCU_IDCODE_DEV_ID                ((uint32_t)0x00000FFF)
#define  DBGMCU_IDCODE_REV_ID                ((uint32_t)0xFFFF0000)

/********************  Bit definition for DBGMCU_CR register  *****************/
#define  DBGMCU_CR_DBG_SLEEP                 ((uint32_t)0x00000001)
#define  DBGMCU_CR_DBG_STOP                  ((uint32_t)0x00000002)
#define  DBGMCU_CR_DBG_STANDBY               ((uint32_t)0x00000004)
#define  DBGMCU_CR_TRACE_IOEN                ((uint32_t)0x00000020)

#define  DBGMCU_CR_TRACE_MODE                ((uint32_t)0x000000C0)
#define  DBGMCU_CR_TRACE_MODE_0              ((uint32_t)0x00000040)/*!<Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              ((uint32_t)0x00000080)/*!<Bit 1 */

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
#define  DBGMCU_APB1_FZ_DBG_TIM2_STOP            ((uint32_t)0x00000001)
#define  DBGMCU_APB1_FZ_DBG_TIM3_STOP            ((uint32_t)0x00000002)
#define  DBGMCU_APB1_FZ_DBG_TIM4_STOP            ((uint32_t)0x00000004)
#define  DBGMCU_APB1_FZ_DBG_TIM5_STOP            ((uint32_t)0x00000008)
#define  DBGMCU_APB1_FZ_DBG_TIM6_STOP            ((uint32_t)0x00000010)
#define  DBGMCU_APB1_FZ_DBG_TIM7_STOP            ((uint32_t)0x00000020)
#define  DBGMCU_APB1_FZ_DBG_TIM12_STOP           ((uint32_t)0x00000040)
#define  DBGMCU_APB1_FZ_DBG_TIM13_STOP           ((uint32_t)0x00000080)
#define  DBGMCU_APB1_FZ_DBG_TIM14_STOP           ((uint32_t)0x00000100)
#define  DBGMCU_APB1_FZ_DBG_RTC_STOP             ((uint32_t)0x00000400)
#define  DBGMCU_APB1_FZ_DBG_WWDG_STOP            ((uint32_t)0x00000800)
#define  DBGMCU_APB1_FZ_DBG_IWDG_STOP            ((uint32_t)0x00001000)
#define  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT   ((uint32_t)0x00200000)
#define  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT   ((uint32_t)0x00400000)
#define  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT   ((uint32_t)0x00800000)
#define  DBGMCU_APB1_FZ_DBG_CAN1_STOP            ((uint32_t)0x02000000)
#define  DBGMCU_APB1_FZ_DBG_CAN2_STOP            ((uint32_t)0x04000000)
/* Old IWDGSTOP bit definition, maintained for legacy purpose */
#define  DBGMCU_APB1_FZ_DBG_IWDEG_STOP           DBGMCU_APB1_FZ_DBG_IWDG_STOP

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define  DBGMCU_APB2_FZ_DBG_TIM1_STOP        ((uint32_t)0x00000001)
#define  DBGMCU_APB2_FZ_DBG_TIM8_STOP        ((uint32_t)0x00000002)
#define  DBGMCU_APB2_FZ_DBG_TIM9_STOP        ((uint32_t)0x00010000)
#define  DBGMCU_APB2_FZ_DBG_TIM10_STOP       ((uint32_t)0x00020000)
#define  DBGMCU_APB2_FZ_DBG_TIM11_STOP       ((uint32_t)0x00040000)

/******************************************************************************/
/*                                                                            */
/*                Ethernet MAC Registers bits definitions                     */
/*                                                                            */
/******************************************************************************/
/* Bit definition for Ethernet MAC Control Register register */
#define ETH_MACCR_WD      ((uint32_t)0x00800000)  /* Watchdog disable */
#define ETH_MACCR_JD      ((uint32_t)0x00400000)  /* Jabber disable */
#define ETH_MACCR_IFG     ((uint32_t)0x000E0000)  /* Inter-frame gap */
#define ETH_MACCR_IFG_96Bit     ((uint32_t)0x00000000)  /* Minimum IFG between frames during transmission is 96Bit */
  #define ETH_MACCR_IFG_88Bit     ((uint32_t)0x00020000)  /* Minimum IFG between frames during transmission is 88Bit */
  #define ETH_MACCR_IFG_80Bit     ((uint32_t)0x00040000)  /* Minimum IFG between frames during transmission is 80Bit */
  #define ETH_MACCR_IFG_72Bit     ((uint32_t)0x00060000)  /* Minimum IFG between frames during transmission is 72Bit */
  #define ETH_MACCR_IFG_64Bit     ((uint32_t)0x00080000)  /* Minimum IFG between frames during transmission is 64Bit */        
  #define ETH_MACCR_IFG_56Bit     ((uint32_t)0x000A0000)  /* Minimum IFG between frames during transmission is 56Bit */
  #define ETH_MACCR_IFG_48Bit     ((uint32_t)0x000C0000)  /* Minimum IFG between frames during transmission is 48Bit */
  #define ETH_MACCR_IFG_40Bit     ((uint32_t)0x000E0000)  /* Minimum IFG between frames during transmission is 40Bit */              
#define ETH_MACCR_CSD     ((uint32_t)0x00010000)  /* Carrier sense disable (during transmission) */
#define ETH_MACCR_FES     ((uint32_t)0x00004000)  /* Fast ethernet speed */
#define ETH_MACCR_ROD     ((uint32_t)0x00002000)  /* Receive own disable */
#define ETH_MACCR_LM      ((uint32_t)0x00001000)  /* loopback mode */
#define ETH_MACCR_DM      ((uint32_t)0x00000800)  /* Duplex mode */
#define ETH_MACCR_IPCO    ((uint32_t)0x00000400)  /* IP Checksum offload */
#define ETH_MACCR_RD      ((uint32_t)0x00000200)  /* Retry disable */
#define ETH_MACCR_APCS    ((uint32_t)0x00000080)  /* Automatic Pad/CRC stripping */
#define ETH_MACCR_BL      ((uint32_t)0x00000060)  /* Back-off limit: random integer number (r) of slot time delays before rescheduling
                                                       a transmission attempt during retries after a collision: 0 =< r <2^k */
  #define ETH_MACCR_BL_10    ((uint32_t)0x00000000)  /* k = min (n, 10) */
  #define ETH_MACCR_BL_8     ((uint32_t)0x00000020)  /* k = min (n, 8) */
  #define ETH_MACCR_BL_4     ((uint32_t)0x00000040)  /* k = min (n, 4) */
  #define ETH_MACCR_BL_1     ((uint32_t)0x00000060)  /* k = min (n, 1) */ 
#define ETH_MACCR_DC      ((uint32_t)0x00000010)  /* Defferal check */
#define ETH_MACCR_TE      ((uint32_t)0x00000008)  /* Transmitter enable */
#define ETH_MACCR_RE      ((uint32_t)0x00000004)  /* Receiver enable */

/* Bit definition for Ethernet MAC Frame Filter Register */
#define ETH_MACFFR_RA     ((uint32_t)0x80000000)  /* Receive all */ 
#define ETH_MACFFR_HPF    ((uint32_t)0x00000400)  /* Hash or perfect filter */ 
#define ETH_MACFFR_SAF    ((uint32_t)0x00000200)  /* Source address filter enable */ 
#define ETH_MACFFR_SAIF   ((uint32_t)0x00000100)  /* SA inverse filtering */ 
#define ETH_MACFFR_PCF    ((uint32_t)0x000000C0)  /* Pass control frames: 3 cases */
  #define ETH_MACFFR_PCF_BlockAll                ((uint32_t)0x00000040)  /* MAC filters all control frames from reaching the application */
  #define ETH_MACFFR_PCF_ForwardAll              ((uint32_t)0x00000080)  /* MAC forwards all control frames to application even if they fail the Address Filter */
  #define ETH_MACFFR_PCF_ForwardPassedAddrFilter ((uint32_t)0x000000C0)  /* MAC forwards control frames that pass the Address Filter. */ 
#define ETH_MACFFR_BFD    ((uint32_t)0x00000020)  /* Broadcast frame disable */ 
#define ETH_MACFFR_PAM    ((uint32_t)0x00000010)  /* Pass all mutlicast */ 
#define ETH_MACFFR_DAIF   ((uint32_t)0x00000008)  /* DA Inverse filtering */ 
#define ETH_MACFFR_HM     ((uint32_t)0x00000004)  /* Hash multicast */ 
#define ETH_MACFFR_HU     ((uint32_t)0x00000002)  /* Hash unicast */
#define ETH_MACFFR_PM     ((uint32_t)0x00000001)  /* Promiscuous mode */

/* Bit definition for Ethernet MAC Hash Table High Register */
#define ETH_MACHTHR_HTH   ((uint32_t)0xFFFFFFFF)  /* Hash table high */

/* Bit definition for Ethernet MAC Hash Table Low Register */
#define ETH_MACHTLR_HTL   ((uint32_t)0xFFFFFFFF)  /* Hash table low */

/* Bit definition for Ethernet MAC MII Address Register */
#define ETH_MACMIIAR_PA   ((uint32_t)0x0000F800)  /* Physical layer address */ 
#define ETH_MACMIIAR_MR   ((uint32_t)0x000007C0)  /* MII register in the selected PHY */ 
#define ETH_MACMIIAR_CR   ((uint32_t)0x0000001C)  /* CR clock range: 6 cases */ 
  #define ETH_MACMIIAR_CR_Div42   ((uint32_t)0x00000000)  /* HCLK:60-100 MHz; MDC clock= HCLK/42 */
  #define ETH_MACMIIAR_CR_Div62   ((uint32_t)0x00000004)  /* HCLK:100-150 MHz; MDC clock= HCLK/62 */
  #define ETH_MACMIIAR_CR_Div16   ((uint32_t)0x00000008)  /* HCLK:20-35 MHz; MDC clock= HCLK/16 */
  #define ETH_MACMIIAR_CR_Div26   ((uint32_t)0x0000000C)  /* HCLK:35-60 MHz; MDC clock= HCLK/26 */
  #define ETH_MACMIIAR_CR_Div102  ((uint32_t)0x00000010)  /* HCLK:150-168 MHz; MDC clock= HCLK/102 */  
#define ETH_MACMIIAR_MW   ((uint32_t)0x00000002)  /* MII write */ 
#define ETH_MACMIIAR_MB   ((uint32_t)0x00000001)  /* MII busy */ 
  
/* Bit definition for Ethernet MAC MII Data Register */
#define ETH_MACMIIDR_MD   ((uint32_t)0x0000FFFF)  /* MII data: read/write data from/to PHY */

/* Bit definition for Ethernet MAC Flow Control Register */
#define ETH_MACFCR_PT     ((uint32_t)0xFFFF0000)  /* Pause time */
#define ETH_MACFCR_ZQPD   ((uint32_t)0x00000080)  /* Zero-quanta pause disable */
#define ETH_MACFCR_PLT    ((uint32_t)0x00000030)  /* Pause low threshold: 4 cases */
  #define ETH_MACFCR_PLT_Minus4   ((uint32_t)0x00000000)  /* Pause time minus 4 slot times */
  #define ETH_MACFCR_PLT_Minus28  ((uint32_t)0x00000010)  /* Pause time minus 28 slot times */
  #define ETH_MACFCR_PLT_Minus144 ((uint32_t)0x00000020)  /* Pause time minus 144 slot times */
  #define ETH_MACFCR_PLT_Minus256 ((uint32_t)0x00000030)  /* Pause time minus 256 slot times */      
#define ETH_MACFCR_UPFD   ((uint32_t)0x00000008)  /* Unicast pause frame detect */
#define ETH_MACFCR_RFCE   ((uint32_t)0x00000004)  /* Receive flow control enable */
#define ETH_MACFCR_TFCE   ((uint32_t)0x00000002)  /* Transmit flow control enable */
#define ETH_MACFCR_FCBBPA ((uint32_t)0x00000001)  /* Flow control busy/backpressure activate */

/* Bit definition for Ethernet MAC VLAN Tag Register */
#define ETH_MACVLANTR_VLANTC ((uint32_t)0x00010000)  /* 12-bit VLAN tag comparison */
#define ETH_MACVLANTR_VLANTI ((uint32_t)0x0000FFFF)  /* VLAN tag identifier (for receive frames) */

/* Bit definition for Ethernet MAC Remote Wake-UpFrame Filter Register */ 
#define ETH_MACRWUFFR_D   ((uint32_t)0xFFFFFFFF)  /* Wake-up frame filter register data */
/* Eight sequential Writes to this address (offset 0x28) will write all Wake-UpFrame Filter Registers.
   Eight sequential Reads from this address (offset 0x28) will read all Wake-UpFrame Filter Registers. */
/* Wake-UpFrame Filter Reg0 : Filter 0 Byte Mask
   Wake-UpFrame Filter Reg1 : Filter 1 Byte Mask
   Wake-UpFrame Filter Reg2 : Filter 2 Byte Mask
   Wake-UpFrame Filter Reg3 : Filter 3 Byte Mask
   Wake-UpFrame Filter Reg4 : RSVD - Filter3 Command - RSVD - Filter2 Command - 
                              RSVD - Filter1 Command - RSVD - Filter0 Command
   Wake-UpFrame Filter Re5 : Filter3 Offset - Filter2 Offset - Filter1 Offset - Filter0 Offset
   Wake-UpFrame Filter Re6 : Filter1 CRC16 - Filter0 CRC16
   Wake-UpFrame Filter Re7 : Filter3 CRC16 - Filter2 CRC16 */

/* Bit definition for Ethernet MAC PMT Control and Status Register */ 
#define ETH_MACPMTCSR_WFFRPR ((uint32_t)0x80000000)  /* Wake-Up Frame Filter Register Pointer Reset */
#define ETH_MACPMTCSR_GU     ((uint32_t)0x00000200)  /* Global Unicast */
#define ETH_MACPMTCSR_WFR    ((uint32_t)0x00000040)  /* Wake-Up Frame Received */
#define ETH_MACPMTCSR_MPR    ((uint32_t)0x00000020)  /* Magic Packet Received */
#define ETH_MACPMTCSR_WFE    ((uint32_t)0x00000004)  /* Wake-Up Frame Enable */
#define ETH_MACPMTCSR_MPE    ((uint32_t)0x00000002)  /* Magic Packet Enable */
#define ETH_MACPMTCSR_PD     ((uint32_t)0x00000001)  /* Power Down */

/* Bit definition for Ethernet MAC Status Register */
#define ETH_MACSR_TSTS      ((uint32_t)0x00000200)  /* Time stamp trigger status */
#define ETH_MACSR_MMCTS     ((uint32_t)0x00000040)  /* MMC transmit status */
#define ETH_MACSR_MMMCRS    ((uint32_t)0x00000020)  /* MMC receive status */
#define ETH_MACSR_MMCS      ((uint32_t)0x00000010)  /* MMC status */
#define ETH_MACSR_PMTS      ((uint32_t)0x00000008)  /* PMT status */

/* Bit definition for Ethernet MAC Interrupt Mask Register */
#define ETH_MACIMR_TSTIM     ((uint32_t)0x00000200)  /* Time stamp trigger interrupt mask */
#define ETH_MACIMR_PMTIM     ((uint32_t)0x00000008)  /* PMT interrupt mask */

/* Bit definition for Ethernet MAC Address0 High Register */
#define ETH_MACA0HR_MACA0H   ((uint32_t)0x0000FFFF)  /* MAC address0 high */

/* Bit definition for Ethernet MAC Address0 Low Register */
#define ETH_MACA0LR_MACA0L   ((uint32_t)0xFFFFFFFF)  /* MAC address0 low */

/* Bit definition for Ethernet MAC Address1 High Register */
#define ETH_MACA1HR_AE       ((uint32_t)0x80000000)  /* Address enable */
#define ETH_MACA1HR_SA       ((uint32_t)0x40000000)  /* Source address */
#define ETH_MACA1HR_MBC      ((uint32_t)0x3F000000)  /* Mask byte control: bits to mask for comparison of the MAC Address bytes */
  #define ETH_MACA1HR_MBC_HBits15_8    ((uint32_t)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
  #define ETH_MACA1HR_MBC_HBits7_0     ((uint32_t)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
  #define ETH_MACA1HR_MBC_LBits31_24   ((uint32_t)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
  #define ETH_MACA1HR_MBC_LBits23_16   ((uint32_t)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
  #define ETH_MACA1HR_MBC_LBits15_8    ((uint32_t)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
  #define ETH_MACA1HR_MBC_LBits7_0     ((uint32_t)0x01000000)  /* Mask MAC Address low reg bits [7:0] */ 
#define ETH_MACA1HR_MACA1H   ((uint32_t)0x0000FFFF)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address1 Low Register */
#define ETH_MACA1LR_MACA1L   ((uint32_t)0xFFFFFFFF)  /* MAC address1 low */

/* Bit definition for Ethernet MAC Address2 High Register */
#define ETH_MACA2HR_AE       ((uint32_t)0x80000000)  /* Address enable */
#define ETH_MACA2HR_SA       ((uint32_t)0x40000000)  /* Source address */
#define ETH_MACA2HR_MBC      ((uint32_t)0x3F000000)  /* Mask byte control */
  #define ETH_MACA2HR_MBC_HBits15_8    ((uint32_t)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
  #define ETH_MACA2HR_MBC_HBits7_0     ((uint32_t)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
  #define ETH_MACA2HR_MBC_LBits31_24   ((uint32_t)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
  #define ETH_MACA2HR_MBC_LBits23_16   ((uint32_t)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
  #define ETH_MACA2HR_MBC_LBits15_8    ((uint32_t)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
  #define ETH_MACA2HR_MBC_LBits7_0     ((uint32_t)0x01000000)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA2HR_MACA2H   ((uint32_t)0x0000FFFF)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address2 Low Register */
#define ETH_MACA2LR_MACA2L   ((uint32_t)0xFFFFFFFF)  /* MAC address2 low */

/* Bit definition for Ethernet MAC Address3 High Register */
#define ETH_MACA3HR_AE       ((uint32_t)0x80000000)  /* Address enable */
#define ETH_MACA3HR_SA       ((uint32_t)0x40000000)  /* Source address */
#define ETH_MACA3HR_MBC      ((uint32_t)0x3F000000)  /* Mask byte control */
  #define ETH_MACA3HR_MBC_HBits15_8    ((uint32_t)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
  #define ETH_MACA3HR_MBC_HBits7_0     ((uint32_t)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
  #define ETH_MACA3HR_MBC_LBits31_24   ((uint32_t)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
  #define ETH_MACA3HR_MBC_LBits23_16   ((uint32_t)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
  #define ETH_MACA3HR_MBC_LBits15_8    ((uint32_t)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
  #define ETH_MACA3HR_MBC_LBits7_0     ((uint32_t)0x01000000)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA3HR_MACA3H   ((uint32_t)0x0000FFFF)  /* MAC address3 high */

/* Bit definition for Ethernet MAC Address3 Low Register */
#define ETH_MACA3LR_MACA3L   ((uint32_t)0xFFFFFFFF)  /* MAC address3 low */

/******************************************************************************/
/*                Ethernet MMC Registers bits definition                      */
/******************************************************************************/

/* Bit definition for Ethernet MMC Contol Register */
#define ETH_MMCCR_MCFHP      ((uint32_t)0x00000020)  /* MMC counter Full-Half preset */
#define ETH_MMCCR_MCP        ((uint32_t)0x00000010)  /* MMC counter preset */
#define ETH_MMCCR_MCF        ((uint32_t)0x00000008)  /* MMC Counter Freeze */
#define ETH_MMCCR_ROR        ((uint32_t)0x00000004)  /* Reset on Read */
#define ETH_MMCCR_CSR        ((uint32_t)0x00000002)  /* Counter Stop Rollover */
#define ETH_MMCCR_CR         ((uint32_t)0x00000001)  /* Counters Reset */

/* Bit definition for Ethernet MMC Receive Interrupt Register */
#define ETH_MMCRIR_RGUFS     ((uint32_t)0x00020000)  /* Set when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIR_RFAES     ((uint32_t)0x00000040)  /* Set when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIR_RFCES     ((uint32_t)0x00000020)  /* Set when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Register */
#define ETH_MMCTIR_TGFS      ((uint32_t)0x00200000)  /* Set when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIR_TGFMSCS   ((uint32_t)0x00008000)  /* Set when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIR_TGFSCS    ((uint32_t)0x00004000)  /* Set when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Receive Interrupt Mask Register */
#define ETH_MMCRIMR_RGUFM    ((uint32_t)0x00020000)  /* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIMR_RFAEM    ((uint32_t)0x00000040)  /* Mask the interrupt when when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIMR_RFCEM    ((uint32_t)0x00000020)  /* Mask the interrupt when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Mask Register */
#define ETH_MMCTIMR_TGFM     ((uint32_t)0x00200000)  /* Mask the interrupt when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFMSCM  ((uint32_t)0x00008000)  /* Mask the interrupt when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFSCM   ((uint32_t)0x00004000)  /* Mask the interrupt when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmitted Good Frames after Single Collision Counter Register */
#define ETH_MMCTGFSCCR_TGFSCC     ((uint32_t)0xFFFFFFFF)  /* Number of successfully transmitted frames after a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames after More than a Single Collision Counter Register */
#define ETH_MMCTGFMSCCR_TGFMSCC   ((uint32_t)0xFFFFFFFF)  /* Number of successfully transmitted frames after more than a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames Counter Register */
#define ETH_MMCTGFCR_TGFC    ((uint32_t)0xFFFFFFFF)  /* Number of good frames transmitted. */

/* Bit definition for Ethernet MMC Received Frames with CRC Error Counter Register */
#define ETH_MMCRFCECR_RFCEC  ((uint32_t)0xFFFFFFFF)  /* Number of frames received with CRC error. */

/* Bit definition for Ethernet MMC Received Frames with Alignement Error Counter Register */
#define ETH_MMCRFAECR_RFAEC  ((uint32_t)0xFFFFFFFF)  /* Number of frames received with alignment (dribble) error */

/* Bit definition for Ethernet MMC Received Good Unicast Frames Counter Register */
#define ETH_MMCRGUFCR_RGUFC  ((uint32_t)0xFFFFFFFF)  /* Number of good unicast frames received. */

/******************************************************************************/
/*               Ethernet PTP Registers bits definition                       */
/******************************************************************************/

/* Bit definition for Ethernet PTP Time Stamp Contol Register */
#define ETH_PTPTSCR_TSCNT       ((uint32_t)0x00030000)  /* Time stamp clock node type */
#define ETH_PTPTSSR_TSSMRME     ((uint32_t)0x00008000)  /* Time stamp snapshot for message relevant to master enable */
#define ETH_PTPTSSR_TSSEME      ((uint32_t)0x00004000)  /* Time stamp snapshot for event message enable */
#define ETH_PTPTSSR_TSSIPV4FE   ((uint32_t)0x00002000)  /* Time stamp snapshot for IPv4 frames enable */
#define ETH_PTPTSSR_TSSIPV6FE   ((uint32_t)0x00001000)  /* Time stamp snapshot for IPv6 frames enable */
#define ETH_PTPTSSR_TSSPTPOEFE  ((uint32_t)0x00000800)  /* Time stamp snapshot for PTP over ethernet frames enable */
#define ETH_PTPTSSR_TSPTPPSV2E  ((uint32_t)0x00000400)  /* Time stamp PTP packet snooping for version2 format enable */
#define ETH_PTPTSSR_TSSSR       ((uint32_t)0x00000200)  /* Time stamp Sub-seconds rollover */
#define ETH_PTPTSSR_TSSARFE     ((uint32_t)0x00000100)  /* Time stamp snapshot for all received frames enable */

#define ETH_PTPTSCR_TSARU    ((uint32_t)0x00000020)  /* Addend register update */
#define ETH_PTPTSCR_TSITE    ((uint32_t)0x00000010)  /* Time stamp interrupt trigger enable */
#define ETH_PTPTSCR_TSSTU    ((uint32_t)0x00000008)  /* Time stamp update */
#define ETH_PTPTSCR_TSSTI    ((uint32_t)0x00000004)  /* Time stamp initialize */
#define ETH_PTPTSCR_TSFCU    ((uint32_t)0x00000002)  /* Time stamp fine or coarse update */
#define ETH_PTPTSCR_TSE      ((uint32_t)0x00000001)  /* Time stamp enable */

/* Bit definition for Ethernet PTP Sub-Second Increment Register */
#define ETH_PTPSSIR_STSSI    ((uint32_t)0x000000FF)  /* System time Sub-second increment value */

/* Bit definition for Ethernet PTP Time Stamp High Register */
#define ETH_PTPTSHR_STS      ((uint32_t)0xFFFFFFFF)  /* System Time second */

/* Bit definition for Ethernet PTP Time Stamp Low Register */
#define ETH_PTPTSLR_STPNS    ((uint32_t)0x80000000)  /* System Time Positive or negative time */
#define ETH_PTPTSLR_STSS     ((uint32_t)0x7FFFFFFF)  /* System Time sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp High Update Register */
#define ETH_PTPTSHUR_TSUS    ((uint32_t)0xFFFFFFFF)  /* Time stamp update seconds */

/* Bit definition for Ethernet PTP Time Stamp Low Update Register */
#define ETH_PTPTSLUR_TSUPNS  ((uint32_t)0x80000000)  /* Time stamp update Positive or negative time */
#define ETH_PTPTSLUR_TSUSS   ((uint32_t)0x7FFFFFFF)  /* Time stamp update sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp Addend Register */
#define ETH_PTPTSAR_TSA      ((uint32_t)0xFFFFFFFF)  /* Time stamp addend */

/* Bit definition for Ethernet PTP Target Time High Register */
#define ETH_PTPTTHR_TTSH     ((uint32_t)0xFFFFFFFF)  /* Target time stamp high */

/* Bit definition for Ethernet PTP Target Time Low Register */
#define ETH_PTPTTLR_TTSL     ((uint32_t)0xFFFFFFFF)  /* Target time stamp low */

/* Bit definition for Ethernet PTP Time Stamp Status Register */
#define ETH_PTPTSSR_TSTTR    ((uint32_t)0x00000020)  /* Time stamp target time reached */
#define ETH_PTPTSSR_TSSO     ((uint32_t)0x00000010)  /* Time stamp seconds overflow */

/******************************************************************************/
/*                 Ethernet DMA Registers bits definition                     */
/******************************************************************************/

/* Bit definition for Ethernet DMA Bus Mode Register */
#define ETH_DMABMR_AAB       ((uint32_t)0x02000000)  /* Address-Aligned beats */
#define ETH_DMABMR_FPM        ((uint32_t)0x01000000)  /* 4xPBL mode */
#define ETH_DMABMR_USP       ((uint32_t)0x00800000)  /* Use separate PBL */
#define ETH_DMABMR_RDP       ((uint32_t)0x007E0000)  /* RxDMA PBL */
  #define ETH_DMABMR_RDP_1Beat    ((uint32_t)0x00020000)  /* maximum number of beats to be transferred in one RxDMA transaction is 1 */
  #define ETH_DMABMR_RDP_2Beat    ((uint32_t)0x00040000)  /* maximum number of beats to be transferred in one RxDMA transaction is 2 */
  #define ETH_DMABMR_RDP_4Beat    ((uint32_t)0x00080000)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
  #define ETH_DMABMR_RDP_8Beat    ((uint32_t)0x00100000)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
  #define ETH_DMABMR_RDP_16Beat   ((uint32_t)0x00200000)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
  #define ETH_DMABMR_RDP_32Beat   ((uint32_t)0x00400000)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */                
  #define ETH_DMABMR_RDP_4xPBL_4Beat   ((uint32_t)0x01020000)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
  #define ETH_DMABMR_RDP_4xPBL_8Beat   ((uint32_t)0x01040000)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
  #define ETH_DMABMR_RDP_4xPBL_16Beat  ((uint32_t)0x01080000)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
  #define ETH_DMABMR_RDP_4xPBL_32Beat  ((uint32_t)0x01100000)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
  #define ETH_DMABMR_RDP_4xPBL_64Beat  ((uint32_t)0x01200000)  /* maximum number of beats to be transferred in one RxDMA transaction is 64 */
  #define ETH_DMABMR_RDP_4xPBL_128Beat ((uint32_t)0x01400000)  /* maximum number of beats to be transferred in one RxDMA transaction is 128 */  
#define ETH_DMABMR_FB        ((uint32_t)0x00010000)  /* Fixed Burst */
#define ETH_DMABMR_RTPR      ((uint32_t)0x0000C000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_1_1     ((uint32_t)0x00000000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_2_1     ((uint32_t)0x00004000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_3_1     ((uint32_t)0x00008000)  /* Rx Tx priority ratio */
  #define ETH_DMABMR_RTPR_4_1     ((uint32_t)0x0000C000)  /* Rx Tx priority ratio */  
#define ETH_DMABMR_PBL    ((uint32_t)0x00003F00)  /* Programmable burst length */
  #define ETH_DMABMR_PBL_1Beat    ((uint32_t)0x00000100)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 1 */
  #define ETH_DMABMR_PBL_2Beat    ((uint32_t)0x00000200)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 2 */
  #define ETH_DMABMR_PBL_4Beat    ((uint32_t)0x00000400)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
  #define ETH_DMABMR_PBL_8Beat    ((uint32_t)0x00000800)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
  #define ETH_DMABMR_PBL_16Beat   ((uint32_t)0x00001000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
  #define ETH_DMABMR_PBL_32Beat   ((uint32_t)0x00002000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */                
  #define ETH_DMABMR_PBL_4xPBL_4Beat   ((uint32_t)0x01000100)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
  #define ETH_DMABMR_PBL_4xPBL_8Beat   ((uint32_t)0x01000200)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
  #define ETH_DMABMR_PBL_4xPBL_16Beat  ((uint32_t)0x01000400)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
  #define ETH_DMABMR_PBL_4xPBL_32Beat  ((uint32_t)0x01000800)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
  #define ETH_DMABMR_PBL_4xPBL_64Beat  ((uint32_t)0x01001000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 64 */
  #define ETH_DMABMR_PBL_4xPBL_128Beat ((uint32_t)0x01002000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 128 */
#define ETH_DMABMR_EDE       ((uint32_t)0x00000080)  /* Enhanced Descriptor Enable */
#define ETH_DMABMR_DSL       ((uint32_t)0x0000007C)  /* Descriptor Skip Length */
#define ETH_DMABMR_DA        ((uint32_t)0x00000002)  /* DMA arbitration scheme */
#define ETH_DMABMR_SR        ((uint32_t)0x00000001)  /* Software reset */

/* Bit definition for Ethernet DMA Transmit Poll Demand Register */
#define ETH_DMATPDR_TPD      ((uint32_t)0xFFFFFFFF)  /* Transmit poll demand */

/* Bit definition for Ethernet DMA Receive Poll Demand Register */
#define ETH_DMARPDR_RPD      ((uint32_t)0xFFFFFFFF)  /* Receive poll demand  */

/* Bit definition for Ethernet DMA Receive Descriptor List Address Register */
#define ETH_DMARDLAR_SRL     ((uint32_t)0xFFFFFFFF)  /* Start of receive list */

/* Bit definition for Ethernet DMA Transmit Descriptor List Address Register */
#define ETH_DMATDLAR_STL     ((uint32_t)0xFFFFFFFF)  /* Start of transmit list */

/* Bit definition for Ethernet DMA Status Register */
#define ETH_DMASR_TSTS       ((uint32_t)0x20000000)  /* Time-stamp trigger status */
#define ETH_DMASR_PMTS       ((uint32_t)0x10000000)  /* PMT status */
#define ETH_DMASR_MMCS       ((uint32_t)0x08000000)  /* MMC status */
#define ETH_DMASR_EBS        ((uint32_t)0x03800000)  /* Error bits status */
  /* combination with EBS[2:0] for GetFlagStatus function */
  #define ETH_DMASR_EBS_DescAccess      ((uint32_t)0x02000000)  /* Error bits 0-data buffer, 1-desc. access */
  #define ETH_DMASR_EBS_ReadTransf      ((uint32_t)0x01000000)  /* Error bits 0-write trnsf, 1-read transfr */
  #define ETH_DMASR_EBS_DataTransfTx    ((uint32_t)0x00800000)  /* Error bits 0-Rx DMA, 1-Tx DMA */
#define ETH_DMASR_TPS         ((uint32_t)0x00700000)  /* Transmit process state */
  #define ETH_DMASR_TPS_Stopped         ((uint32_t)0x00000000)  /* Stopped - Reset or Stop Tx Command issued  */
  #define ETH_DMASR_TPS_Fetching        ((uint32_t)0x00100000)  /* Running - fetching the Tx descriptor */
  #define ETH_DMASR_TPS_Waiting         ((uint32_t)0x00200000)  /* Running - waiting for status */
  #define ETH_DMASR_TPS_Reading         ((uint32_t)0x00300000)  /* Running - reading the data from host memory */
  #define ETH_DMASR_TPS_Suspended       ((uint32_t)0x00600000)  /* Suspended - Tx Descriptor unavailabe */
  #define ETH_DMASR_TPS_Closing         ((uint32_t)0x00700000)  /* Running - closing Rx descriptor */
#define ETH_DMASR_RPS         ((uint32_t)0x000E0000)  /* Receive process state */
  #define ETH_DMASR_RPS_Stopped         ((uint32_t)0x00000000)  /* Stopped - Reset or Stop Rx Command issued */
  #define ETH_DMASR_RPS_Fetching        ((uint32_t)0x00020000)  /* Running - fetching the Rx descriptor */
  #define ETH_DMASR_RPS_Waiting         ((uint32_t)0x00060000)  /* Running - waiting for packet */
  #define ETH_DMASR_RPS_Suspended       ((uint32_t)0x00080000)  /* Suspended - Rx Descriptor unavailable */
  #define ETH_DMASR_RPS_Closing         ((uint32_t)0x000A0000)  /* Running - closing descriptor */
  #define ETH_DMASR_RPS_Queuing         ((uint32_t)0x000E0000)  /* Running - queuing the recieve frame into host memory */
#define ETH_DMASR_NIS        ((uint32_t)0x00010000)  /* Normal interrupt summary */
#define ETH_DMASR_AIS        ((uint32_t)0x00008000)  /* Abnormal interrupt summary */
#define ETH_DMASR_ERS        ((uint32_t)0x00004000)  /* Early receive status */
#define ETH_DMASR_FBES       ((uint32_t)0x00002000)  /* Fatal bus error status */
#define ETH_DMASR_ETS        ((uint32_t)0x00000400)  /* Early transmit status */
#define ETH_DMASR_RWTS       ((uint32_t)0x00000200)  /* Receive watchdog timeout status */
#define ETH_DMASR_RPSS       ((uint32_t)0x00000100)  /* Receive process stopped status */
#define ETH_DMASR_RBUS       ((uint32_t)0x00000080)  /* Receive buffer unavailable status */
#define ETH_DMASR_RS         ((uint32_t)0x00000040)  /* Receive status */
#define ETH_DMASR_TUS        ((uint32_t)0x00000020)  /* Transmit underflow status */
#define ETH_DMASR_ROS        ((uint32_t)0x00000010)  /* Receive overflow status */
#define ETH_DMASR_TJTS       ((uint32_t)0x00000008)  /* Transmit jabber timeout status */
#define ETH_DMASR_TBUS       ((uint32_t)0x00000004)  /* Transmit buffer unavailable status */
#define ETH_DMASR_TPSS       ((uint32_t)0x00000002)  /* Transmit process stopped status */
#define ETH_DMASR_TS         ((uint32_t)0x00000001)  /* Transmit status */

/* Bit definition for Ethernet DMA Operation Mode Register */
#define ETH_DMAOMR_DTCEFD    ((uint32_t)0x04000000)  /* Disable Dropping of TCP/IP checksum error frames */
#define ETH_DMAOMR_RSF       ((uint32_t)0x02000000)  /* Receive store and forward */
#define ETH_DMAOMR_DFRF      ((uint32_t)0x01000000)  /* Disable flushing of received frames */
#define ETH_DMAOMR_TSF       ((uint32_t)0x00200000)  /* Transmit store and forward */
#define ETH_DMAOMR_FTF       ((uint32_t)0x00100000)  /* Flush transmit FIFO */
#define ETH_DMAOMR_TTC       ((uint32_t)0x0001C000)  /* Transmit threshold control */
  #define ETH_DMAOMR_TTC_64Bytes       ((uint32_t)0x00000000)  /* threshold level of the MTL Transmit FIFO is 64 Bytes */
  #define ETH_DMAOMR_TTC_128Bytes      ((uint32_t)0x00004000)  /* threshold level of the MTL Transmit FIFO is 128 Bytes */
  #define ETH_DMAOMR_TTC_192Bytes      ((uint32_t)0x00008000)  /* threshold level of the MTL Transmit FIFO is 192 Bytes */
  #define ETH_DMAOMR_TTC_256Bytes      ((uint32_t)0x0000C000)  /* threshold level of the MTL Transmit FIFO is 256 Bytes */
  #define ETH_DMAOMR_TTC_40Bytes       ((uint32_t)0x00010000)  /* threshold level of the MTL Transmit FIFO is 40 Bytes */
  #define ETH_DMAOMR_TTC_32Bytes       ((uint32_t)0x00014000)  /* threshold level of the MTL Transmit FIFO is 32 Bytes */
  #define ETH_DMAOMR_TTC_24Bytes       ((uint32_t)0x00018000)  /* threshold level of the MTL Transmit FIFO is 24 Bytes */
  #define ETH_DMAOMR_TTC_16Bytes       ((uint32_t)0x0001C000)  /* threshold level of the MTL Transmit FIFO is 16 Bytes */
#define ETH_DMAOMR_ST        ((uint32_t)0x00002000)  /* Start/stop transmission command */
#define ETH_DMAOMR_FEF       ((uint32_t)0x00000080)  /* Forward error frames */
#define ETH_DMAOMR_FUGF      ((uint32_t)0x00000040)  /* Forward undersized good frames */
#define ETH_DMAOMR_RTC       ((uint32_t)0x00000018)  /* receive threshold control */
  #define ETH_DMAOMR_RTC_64Bytes       ((uint32_t)0x00000000)  /* threshold level of the MTL Receive FIFO is 64 Bytes */
  #define ETH_DMAOMR_RTC_32Bytes       ((uint32_t)0x00000008)  /* threshold level of the MTL Receive FIFO is 32 Bytes */
  #define ETH_DMAOMR_RTC_96Bytes       ((uint32_t)0x00000010)  /* threshold level of the MTL Receive FIFO is 96 Bytes */
  #define ETH_DMAOMR_RTC_128Bytes      ((uint32_t)0x00000018)  /* threshold level of the MTL Receive FIFO is 128 Bytes */
#define ETH_DMAOMR_OSF       ((uint32_t)0x00000004)  /* operate on second frame */
#define ETH_DMAOMR_SR        ((uint32_t)0x00000002)  /* Start/stop receive */

/* Bit definition for Ethernet DMA Interrupt Enable Register */
#define ETH_DMAIER_NISE      ((uint32_t)0x00010000)  /* Normal interrupt summary enable */
#define ETH_DMAIER_AISE      ((uint32_t)0x00008000)  /* Abnormal interrupt summary enable */
#define ETH_DMAIER_ERIE      ((uint32_t)0x00004000)  /* Early receive interrupt enable */
#define ETH_DMAIER_FBEIE     ((uint32_t)0x00002000)  /* Fatal bus error interrupt enable */
#define ETH_DMAIER_ETIE      ((uint32_t)0x00000400)  /* Early transmit interrupt enable */
#define ETH_DMAIER_RWTIE     ((uint32_t)0x00000200)  /* Receive watchdog timeout interrupt enable */
#define ETH_DMAIER_RPSIE     ((uint32_t)0x00000100)  /* Receive process stopped interrupt enable */
#define ETH_DMAIER_RBUIE     ((uint32_t)0x00000080)  /* Receive buffer unavailable interrupt enable */
#define ETH_DMAIER_RIE       ((uint32_t)0x00000040)  /* Receive interrupt enable */
#define ETH_DMAIER_TUIE      ((uint32_t)0x00000020)  /* Transmit Underflow interrupt enable */
#define ETH_DMAIER_ROIE      ((uint32_t)0x00000010)  /* Receive Overflow interrupt enable */
#define ETH_DMAIER_TJTIE     ((uint32_t)0x00000008)  /* Transmit jabber timeout interrupt enable */
#define ETH_DMAIER_TBUIE     ((uint32_t)0x00000004)  /* Transmit buffer unavailable interrupt enable */
#define ETH_DMAIER_TPSIE     ((uint32_t)0x00000002)  /* Transmit process stopped interrupt enable */
#define ETH_DMAIER_TIE       ((uint32_t)0x00000001)  /* Transmit interrupt enable */

/* Bit definition for Ethernet DMA Missed Frame and Buffer Overflow Counter Register */
#define ETH_DMAMFBOCR_OFOC   ((uint32_t)0x10000000)  /* Overflow bit for FIFO overflow counter */
#define ETH_DMAMFBOCR_MFA    ((uint32_t)0x0FFE0000)  /* Number of frames missed by the application */
#define ETH_DMAMFBOCR_OMFC   ((uint32_t)0x00010000)  /* Overflow bit for missed frame counter */
#define ETH_DMAMFBOCR_MFC    ((uint32_t)0x0000FFFF)  /* Number of frames missed by the controller */

/* Bit definition for Ethernet DMA Current Host Transmit Descriptor Register */
#define ETH_DMACHTDR_HTDAP   ((uint32_t)0xFFFFFFFF)  /* Host transmit descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Descriptor Register */
#define ETH_DMACHRDR_HRDAP   ((uint32_t)0xFFFFFFFF)  /* Host receive descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Transmit Buffer Address Register */
#define ETH_DMACHTBAR_HTBAP  ((uint32_t)0xFFFFFFFF)  /* Host transmit buffer address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Buffer Address Register */
#define ETH_DMACHRBAR_HRBAP  ((uint32_t)0xFFFFFFFF)  /* Host receive buffer address pointer */

/******************************************************************************/
/*                                                                            */
/*                                       USB_OTG			                        */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition forUSB_OTG_GOTGCTL register  ********************/
#define USB_OTG_GOTGCTL_SRQSCS                  ((uint32_t)0x00000001)            /*!< Session request success */
#define USB_OTG_GOTGCTL_SRQ                     ((uint32_t)0x00000002)            /*!< Session request */
#define USB_OTG_GOTGCTL_HNGSCS                  ((uint32_t)0x00000100)            /*!< Host negotiation success */
#define USB_OTG_GOTGCTL_HNPRQ                   ((uint32_t)0x00000200)            /*!< HNP request */
#define USB_OTG_GOTGCTL_HSHNPEN                 ((uint32_t)0x00000400)            /*!< Host set HNP enable */
#define USB_OTG_GOTGCTL_DHNPEN                  ((uint32_t)0x00000800)            /*!< Device HNP enabled */
#define USB_OTG_GOTGCTL_CIDSTS                  ((uint32_t)0x00010000)            /*!< Connector ID status */
#define USB_OTG_GOTGCTL_DBCT                    ((uint32_t)0x00020000)            /*!< Long/short debounce time */
#define USB_OTG_GOTGCTL_ASVLD                   ((uint32_t)0x00040000)            /*!< A-session valid */
#define USB_OTG_GOTGCTL_BSVLD                   ((uint32_t)0x00080000)            /*!< B-session valid */

/********************  Bit definition forUSB_OTG_HCFG register  ********************/

#define USB_OTG_HCFG_FSLSPCS                 ((uint32_t)0x00000003)            /*!< FS/LS PHY clock select */
#define USB_OTG_HCFG_FSLSPCS_0               ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_HCFG_FSLSPCS_1               ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_HCFG_FSLSS                   ((uint32_t)0x00000004)            /*!< FS- and LS-only support */

/********************  Bit definition forUSB_OTG_DCFG register  ********************/

#define USB_OTG_DCFG_DSPD                    ((uint32_t)0x00000003)            /*!< Device speed */
#define USB_OTG_DCFG_DSPD_0                  ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_DCFG_DSPD_1                  ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_DCFG_NZLSOHSK                ((uint32_t)0x00000004)            /*!< Nonzero-length status OUT handshake */

#define USB_OTG_DCFG_DAD                     ((uint32_t)0x000007F0)            /*!< Device address */
#define USB_OTG_DCFG_DAD_0                   ((uint32_t)0x00000010)            /*!<Bit 0 */
#define USB_OTG_DCFG_DAD_1                   ((uint32_t)0x00000020)            /*!<Bit 1 */
#define USB_OTG_DCFG_DAD_2                   ((uint32_t)0x00000040)            /*!<Bit 2 */
#define USB_OTG_DCFG_DAD_3                   ((uint32_t)0x00000080)            /*!<Bit 3 */
#define USB_OTG_DCFG_DAD_4                   ((uint32_t)0x00000100)            /*!<Bit 4 */
#define USB_OTG_DCFG_DAD_5                   ((uint32_t)0x00000200)            /*!<Bit 5 */
#define USB_OTG_DCFG_DAD_6                   ((uint32_t)0x00000400)            /*!<Bit 6 */

#define USB_OTG_DCFG_PFIVL                   ((uint32_t)0x00001800)            /*!< Periodic (micro)frame interval */
#define USB_OTG_DCFG_PFIVL_0                 ((uint32_t)0x00000800)            /*!<Bit 0 */
#define USB_OTG_DCFG_PFIVL_1                 ((uint32_t)0x00001000)            /*!<Bit 1 */

#define USB_OTG_DCFG_PERSCHIVL               ((uint32_t)0x03000000)            /*!< Periodic scheduling interval */
#define USB_OTG_DCFG_PERSCHIVL_0             ((uint32_t)0x01000000)            /*!<Bit 0 */
#define USB_OTG_DCFG_PERSCHIVL_1             ((uint32_t)0x02000000)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_PCGCR register  ********************/
#define USB_OTG_PCGCR_STPPCLK                 ((uint32_t)0x00000001)            /*!< Stop PHY clock */
#define USB_OTG_PCGCR_GATEHCLK                ((uint32_t)0x00000002)            /*!< Gate HCLK */
#define USB_OTG_PCGCR_PHYSUSP                 ((uint32_t)0x00000010)            /*!< PHY suspended */

/********************  Bit definition forUSB_OTG_GOTGINT register  ********************/
#define USB_OTG_GOTGINT_SEDET                   ((uint32_t)0x00000004)            /*!< Session end detected */
#define USB_OTG_GOTGINT_SRSSCHG                 ((uint32_t)0x00000100)            /*!< Session request success status change */
#define USB_OTG_GOTGINT_HNSSCHG                 ((uint32_t)0x00000200)            /*!< Host negotiation success status change */
#define USB_OTG_GOTGINT_HNGDET                  ((uint32_t)0x00020000)            /*!< Host negotiation detected */
#define USB_OTG_GOTGINT_ADTOCHG                 ((uint32_t)0x00040000)            /*!< A-device timeout change */
#define USB_OTG_GOTGINT_DBCDNE                  ((uint32_t)0x00080000)            /*!< Debounce done */

/********************  Bit definition forUSB_OTG_DCTL register  ********************/
#define USB_OTG_DCTL_RWUSIG                  ((uint32_t)0x00000001)            /*!< Remote wakeup signaling */
#define USB_OTG_DCTL_SDIS                    ((uint32_t)0x00000002)            /*!< Soft disconnect */
#define USB_OTG_DCTL_GINSTS                  ((uint32_t)0x00000004)            /*!< Global IN NAK status */
#define USB_OTG_DCTL_GONSTS                  ((uint32_t)0x00000008)            /*!< Global OUT NAK status */

#define USB_OTG_DCTL_TCTL                    ((uint32_t)0x00000070)            /*!< Test control */
#define USB_OTG_DCTL_TCTL_0                  ((uint32_t)0x00000010)            /*!<Bit 0 */
#define USB_OTG_DCTL_TCTL_1                  ((uint32_t)0x00000020)            /*!<Bit 1 */
#define USB_OTG_DCTL_TCTL_2                  ((uint32_t)0x00000040)            /*!<Bit 2 */
#define USB_OTG_DCTL_SGINAK                  ((uint32_t)0x00000080)            /*!< Set global IN NAK */
#define USB_OTG_DCTL_CGINAK                  ((uint32_t)0x00000100)            /*!< Clear global IN NAK */
#define USB_OTG_DCTL_SGONAK                  ((uint32_t)0x00000200)            /*!< Set global OUT NAK */
#define USB_OTG_DCTL_CGONAK                  ((uint32_t)0x00000400)            /*!< Clear global OUT NAK */
#define USB_OTG_DCTL_POPRGDNE                ((uint32_t)0x00000800)            /*!< Power-on programming done */

/********************  Bit definition forUSB_OTG_HFIR register  ********************/
#define USB_OTG_HFIR_FRIVL                   ((uint32_t)0x0000FFFF)            /*!< Frame interval */

/********************  Bit definition forUSB_OTG_HFNUM register  ********************/
#define USB_OTG_HFNUM_FRNUM                   ((uint32_t)0x0000FFFF)            /*!< Frame number */
#define USB_OTG_HFNUM_FTREM                   ((uint32_t)0xFFFF0000)            /*!< Frame time remaining */

/********************  Bit definition forUSB_OTG_DSTS register  ********************/
#define USB_OTG_DSTS_SUSPSTS                 ((uint32_t)0x00000001)            /*!< Suspend status */

#define USB_OTG_DSTS_ENUMSPD                 ((uint32_t)0x00000006)            /*!< Enumerated speed */
#define USB_OTG_DSTS_ENUMSPD_0               ((uint32_t)0x00000002)            /*!<Bit 0 */
#define USB_OTG_DSTS_ENUMSPD_1               ((uint32_t)0x00000004)            /*!<Bit 1 */
#define USB_OTG_DSTS_EERR                    ((uint32_t)0x00000008)            /*!< Erratic error */
#define USB_OTG_DSTS_FNSOF                   ((uint32_t)0x003FFF00)            /*!< Frame number of the received SOF */

/********************  Bit definition forUSB_OTG_GAHBCFG register  ********************/
#define USB_OTG_GAHBCFG_GINT                    ((uint32_t)0x00000001)            /*!< Global interrupt mask */

#define USB_OTG_GAHBCFG_HBSTLEN                 ((uint32_t)0x0000001E)            /*!< Burst length/type */
#define USB_OTG_GAHBCFG_HBSTLEN_0               ((uint32_t)0x00000002)            /*!<Bit 0 */
#define USB_OTG_GAHBCFG_HBSTLEN_1               ((uint32_t)0x00000004)            /*!<Bit 1 */
#define USB_OTG_GAHBCFG_HBSTLEN_2               ((uint32_t)0x00000008)            /*!<Bit 2 */
#define USB_OTG_GAHBCFG_HBSTLEN_3               ((uint32_t)0x00000010)            /*!<Bit 3 */
#define USB_OTG_GAHBCFG_DMAEN                   ((uint32_t)0x00000020)            /*!< DMA enable */
#define USB_OTG_GAHBCFG_TXFELVL                 ((uint32_t)0x00000080)            /*!< TxFIFO empty level */
#define USB_OTG_GAHBCFG_PTXFELVL                ((uint32_t)0x00000100)            /*!< Periodic TxFIFO empty level */

/********************  Bit definition forUSB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_TOCAL                   ((uint32_t)0x00000007)            /*!< FS timeout calibration */
#define USB_OTG_GUSBCFG_TOCAL_0                 ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TOCAL_1                 ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TOCAL_2                 ((uint32_t)0x00000004)            /*!<Bit 2 */
#define USB_OTG_GUSBCFG_PHYSEL                  ((uint32_t)0x00000040)            /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
#define USB_OTG_GUSBCFG_SRPCAP                  ((uint32_t)0x00000100)            /*!< SRP-capable */
#define USB_OTG_GUSBCFG_HNPCAP                  ((uint32_t)0x00000200)            /*!< HNP-capable */

#define USB_OTG_GUSBCFG_TRDT                    ((uint32_t)0x00003C00)            /*!< USB turnaround time */
#define USB_OTG_GUSBCFG_TRDT_0                  ((uint32_t)0x00000400)            /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TRDT_1                  ((uint32_t)0x00000800)            /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TRDT_2                  ((uint32_t)0x00001000)            /*!<Bit 2 */
#define USB_OTG_GUSBCFG_TRDT_3                  ((uint32_t)0x00002000)            /*!<Bit 3 */
#define USB_OTG_GUSBCFG_PHYLPCS                 ((uint32_t)0x00008000)            /*!< PHY Low-power clock select */
#define USB_OTG_GUSBCFG_ULPIFSLS                ((uint32_t)0x00020000)            /*!< ULPI FS/LS select */
#define USB_OTG_GUSBCFG_ULPIAR                  ((uint32_t)0x00040000)            /*!< ULPI Auto-resume */
#define USB_OTG_GUSBCFG_ULPICSM                 ((uint32_t)0x00080000)            /*!< ULPI Clock SuspendM */
#define USB_OTG_GUSBCFG_ULPIEVBUSD              ((uint32_t)0x00100000)            /*!< ULPI External VBUS Drive */
#define USB_OTG_GUSBCFG_ULPIEVBUSI              ((uint32_t)0x00200000)            /*!< ULPI external VBUS indicator */
#define USB_OTG_GUSBCFG_TSDPS                   ((uint32_t)0x00400000)            /*!< TermSel DLine pulsing selection */
#define USB_OTG_GUSBCFG_PCCI                    ((uint32_t)0x00800000)            /*!< Indicator complement */
#define USB_OTG_GUSBCFG_PTCI                    ((uint32_t)0x01000000)            /*!< Indicator pass through */
#define USB_OTG_GUSBCFG_ULPIIPD                 ((uint32_t)0x02000000)            /*!< ULPI interface protect disable */
#define USB_OTG_GUSBCFG_FHMOD                   ((uint32_t)0x20000000)            /*!< Forced host mode */
#define USB_OTG_GUSBCFG_FDMOD                   ((uint32_t)0x40000000)            /*!< Forced peripheral mode */
#define USB_OTG_GUSBCFG_CTXPKT                  ((uint32_t)0x80000000)            /*!< Corrupt Tx packet */

/********************  Bit definition forUSB_OTG_GRSTCTL register  ********************/
#define USB_OTG_GRSTCTL_CSRST                   ((uint32_t)0x00000001)            /*!< Core soft reset */
#define USB_OTG_GRSTCTL_HSRST                   ((uint32_t)0x00000002)            /*!< HCLK soft reset */
#define USB_OTG_GRSTCTL_FCRST                   ((uint32_t)0x00000004)            /*!< Host frame counter reset */
#define USB_OTG_GRSTCTL_RXFFLSH                 ((uint32_t)0x00000010)            /*!< RxFIFO flush */
#define USB_OTG_GRSTCTL_TXFFLSH                 ((uint32_t)0x00000020)            /*!< TxFIFO flush */

#define USB_OTG_GRSTCTL_TXFNUM                  ((uint32_t)0x000007C0)            /*!< TxFIFO number */
#define USB_OTG_GRSTCTL_TXFNUM_0                ((uint32_t)0x00000040)            /*!<Bit 0 */
#define USB_OTG_GRSTCTL_TXFNUM_1                ((uint32_t)0x00000080)            /*!<Bit 1 */
#define USB_OTG_GRSTCTL_TXFNUM_2                ((uint32_t)0x00000100)            /*!<Bit 2 */
#define USB_OTG_GRSTCTL_TXFNUM_3                ((uint32_t)0x00000200)            /*!<Bit 3 */
#define USB_OTG_GRSTCTL_TXFNUM_4                ((uint32_t)0x00000400)            /*!<Bit 4 */
#define USB_OTG_GRSTCTL_DMAREQ                  ((uint32_t)0x40000000)            /*!< DMA request signal */
#define USB_OTG_GRSTCTL_AHBIDL                  ((uint32_t)0x80000000)            /*!< AHB master idle */

/********************  Bit definition forUSB_OTG_DIEPMSK register  ********************/
#define USB_OTG_DIEPMSK_XFRCM                   ((uint32_t)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPMSK_EPDM                    ((uint32_t)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPMSK_TOM                     ((uint32_t)0x00000008)            /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPMSK_ITTXFEMSK               ((uint32_t)0x00000010)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPMSK_INEPNMM                 ((uint32_t)0x00000020)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPMSK_INEPNEM                 ((uint32_t)0x00000040)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPMSK_TXFURM                  ((uint32_t)0x00000100)            /*!< FIFO underrun mask */
#define USB_OTG_DIEPMSK_BIM                     ((uint32_t)0x00000200)            /*!< BNA interrupt mask */

/********************  Bit definition forUSB_OTG_HPTXSTS register  ********************/
#define USB_OTG_HPTXSTS_PTXFSAVL                ((uint32_t)0x0000FFFF)            /*!< Periodic transmit data FIFO space available */

#define USB_OTG_HPTXSTS_PTXQSAV                 ((uint32_t)0x00FF0000)            /*!< Periodic transmit request queue space available */
#define USB_OTG_HPTXSTS_PTXQSAV_0               ((uint32_t)0x00010000)            /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQSAV_1               ((uint32_t)0x00020000)            /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQSAV_2               ((uint32_t)0x00040000)            /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQSAV_3               ((uint32_t)0x00080000)            /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQSAV_4               ((uint32_t)0x00100000)            /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQSAV_5               ((uint32_t)0x00200000)            /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQSAV_6               ((uint32_t)0x00400000)            /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQSAV_7               ((uint32_t)0x00800000)            /*!<Bit 7 */

#define USB_OTG_HPTXSTS_PTXQTOP                 ((uint32_t)0xFF000000)            /*!< Top of the periodic transmit request queue */
#define USB_OTG_HPTXSTS_PTXQTOP_0               ((uint32_t)0x01000000)            /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQTOP_1               ((uint32_t)0x02000000)            /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQTOP_2               ((uint32_t)0x04000000)            /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQTOP_3               ((uint32_t)0x08000000)            /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQTOP_4               ((uint32_t)0x10000000)            /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQTOP_5               ((uint32_t)0x20000000)            /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQTOP_6               ((uint32_t)0x40000000)            /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQTOP_7               ((uint32_t)0x80000000)            /*!<Bit 7 */

/********************  Bit definition forUSB_OTG_HAINT register  ********************/
#define USB_OTG_HAINT_HAINT                   ((uint32_t)0x0000FFFF)            /*!< Channel interrupts */

/********************  Bit definition forUSB_OTG_DOEPMSK register  ********************/
#define USB_OTG_DOEPMSK_XFRCM                   ((uint32_t)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPMSK_EPDM                    ((uint32_t)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPMSK_STUPM                   ((uint32_t)0x00000008)            /*!< SETUP phase done mask */
#define USB_OTG_DOEPMSK_OTEPDM                  ((uint32_t)0x00000010)            /*!< OUT token received when endpoint disabled mask */
#define USB_OTG_DOEPMSK_B2BSTUP                 ((uint32_t)0x00000040)            /*!< Back-to-back SETUP packets received mask */
#define USB_OTG_DOEPMSK_OPEM                    ((uint32_t)0x00000100)            /*!< OUT packet error mask */
#define USB_OTG_DOEPMSK_BOIM                    ((uint32_t)0x00000200)            /*!< BNA interrupt mask */

/********************  Bit definition forUSB_OTG_GINTSTS register  ********************/
#define USB_OTG_GINTSTS_CMOD                    ((uint32_t)0x00000001)            /*!< Current mode of operation */
#define USB_OTG_GINTSTS_MMIS                    ((uint32_t)0x00000002)            /*!< Mode mismatch interrupt */
#define USB_OTG_GINTSTS_OTGINT                  ((uint32_t)0x00000004)            /*!< OTG interrupt */
#define USB_OTG_GINTSTS_SOF                     ((uint32_t)0x00000008)            /*!< Start of frame */
#define USB_OTG_GINTSTS_RXFLVL                  ((uint32_t)0x00000010)            /*!< RxFIFO nonempty */
#define USB_OTG_GINTSTS_NPTXFE                  ((uint32_t)0x00000020)            /*!< Nonperiodic TxFIFO empty */
#define USB_OTG_GINTSTS_GINAKEFF                ((uint32_t)0x00000040)            /*!< Global IN nonperiodic NAK effective */
#define USB_OTG_GINTSTS_BOUTNAKEFF              ((uint32_t)0x00000080)            /*!< Global OUT NAK effective */
#define USB_OTG_GINTSTS_ESUSP                   ((uint32_t)0x00000400)            /*!< Early suspend */
#define USB_OTG_GINTSTS_USBSUSP                 ((uint32_t)0x00000800)            /*!< USB suspend */
#define USB_OTG_GINTSTS_USBRST                  ((uint32_t)0x00001000)            /*!< USB reset */
#define USB_OTG_GINTSTS_ENUMDNE                 ((uint32_t)0x00002000)            /*!< Enumeration done */
#define USB_OTG_GINTSTS_ISOODRP                 ((uint32_t)0x00004000)            /*!< Isochronous OUT packet dropped interrupt */
#define USB_OTG_GINTSTS_EOPF                    ((uint32_t)0x00008000)            /*!< End of periodic frame interrupt */
#define USB_OTG_GINTSTS_IEPINT                  ((uint32_t)0x00040000)            /*!< IN endpoint interrupt */
#define USB_OTG_GINTSTS_OEPINT                  ((uint32_t)0x00080000)            /*!< OUT endpoint interrupt */
#define USB_OTG_GINTSTS_IISOIXFR                ((uint32_t)0x00100000)            /*!< Incomplete isochronous IN transfer */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT       ((uint32_t)0x00200000)            /*!< Incomplete periodic transfer */
#define USB_OTG_GINTSTS_DATAFSUSP               ((uint32_t)0x00400000)            /*!< Data fetch suspended */
#define USB_OTG_GINTSTS_HPRTINT                 ((uint32_t)0x01000000)            /*!< Host port interrupt */
#define USB_OTG_GINTSTS_HCINT                   ((uint32_t)0x02000000)            /*!< Host channels interrupt */
#define USB_OTG_GINTSTS_PTXFE                   ((uint32_t)0x04000000)            /*!< Periodic TxFIFO empty */
#define USB_OTG_GINTSTS_CIDSCHG                 ((uint32_t)0x10000000)            /*!< Connector ID status change */
#define USB_OTG_GINTSTS_DISCINT                 ((uint32_t)0x20000000)            /*!< Disconnect detected interrupt */
#define USB_OTG_GINTSTS_SRQINT                  ((uint32_t)0x40000000)            /*!< Session request/new session detected interrupt */
#define USB_OTG_GINTSTS_WKUINT                  ((uint32_t)0x80000000)            /*!< Resume/remote wakeup detected interrupt */

/********************  Bit definition forUSB_OTG_GINTMSK register  ********************/
#define USB_OTG_GINTMSK_MMISM                   ((uint32_t)0x00000002)            /*!< Mode mismatch interrupt mask */
#define USB_OTG_GINTMSK_OTGINT                  ((uint32_t)0x00000004)            /*!< OTG interrupt mask */
#define USB_OTG_GINTMSK_SOFM                    ((uint32_t)0x00000008)            /*!< Start of frame mask */
#define USB_OTG_GINTMSK_RXFLVLM                 ((uint32_t)0x00000010)            /*!< Receive FIFO nonempty mask */
#define USB_OTG_GINTMSK_NPTXFEM                 ((uint32_t)0x00000020)            /*!< Nonperiodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_GINAKEFFM               ((uint32_t)0x00000040)            /*!< Global nonperiodic IN NAK effective mask */
#define USB_OTG_GINTMSK_GONAKEFFM               ((uint32_t)0x00000080)            /*!< Global OUT NAK effective mask */
#define USB_OTG_GINTMSK_ESUSPM                  ((uint32_t)0x00000400)            /*!< Early suspend mask */
#define USB_OTG_GINTMSK_USBSUSPM                ((uint32_t)0x00000800)            /*!< USB suspend mask */
#define USB_OTG_GINTMSK_USBRST                  ((uint32_t)0x00001000)            /*!< USB reset mask */
#define USB_OTG_GINTMSK_ENUMDNEM                ((uint32_t)0x00002000)            /*!< Enumeration done mask */
#define USB_OTG_GINTMSK_ISOODRPM                ((uint32_t)0x00004000)            /*!< Isochronous OUT packet dropped interrupt mask */
#define USB_OTG_GINTMSK_EOPFM                   ((uint32_t)0x00008000)            /*!< End of periodic frame interrupt mask */
#define USB_OTG_GINTMSK_EPMISM                  ((uint32_t)0x00020000)            /*!< Endpoint mismatch interrupt mask */
#define USB_OTG_GINTMSK_IEPINT                  ((uint32_t)0x00040000)            /*!< IN endpoints interrupt mask */
#define USB_OTG_GINTMSK_OEPINT                  ((uint32_t)0x00080000)            /*!< OUT endpoints interrupt mask */
#define USB_OTG_GINTMSK_IISOIXFRM               ((uint32_t)0x00100000)            /*!< Incomplete isochronous IN transfer mask */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM         ((uint32_t)0x00200000)            /*!< Incomplete periodic transfer mask */
#define USB_OTG_GINTMSK_FSUSPM                  ((uint32_t)0x00400000)            /*!< Data fetch suspended mask */
#define USB_OTG_GINTMSK_PRTIM                   ((uint32_t)0x01000000)            /*!< Host port interrupt mask */
#define USB_OTG_GINTMSK_HCIM                    ((uint32_t)0x02000000)            /*!< Host channels interrupt mask */
#define USB_OTG_GINTMSK_PTXFEM                  ((uint32_t)0x04000000)            /*!< Periodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_CIDSCHGM                ((uint32_t)0x10000000)            /*!< Connector ID status change mask */
#define USB_OTG_GINTMSK_DISCINT                 ((uint32_t)0x20000000)            /*!< Disconnect detected interrupt mask */
#define USB_OTG_GINTMSK_SRQIM                   ((uint32_t)0x40000000)            /*!< Session request/new session detected interrupt mask */
#define USB_OTG_GINTMSK_WUIM                    ((uint32_t)0x80000000)            /*!< Resume/remote wakeup detected interrupt mask */

/********************  Bit definition forUSB_OTG_DAINT register  ********************/
#define USB_OTG_DAINT_IEPINT                  ((uint32_t)0x0000FFFF)            /*!< IN endpoint interrupt bits */
#define USB_OTG_DAINT_OEPINT                  ((uint32_t)0xFFFF0000)            /*!< OUT endpoint interrupt bits */

/********************  Bit definition forUSB_OTG_HAINTMSK register  ********************/
#define USB_OTG_HAINTMSK_HAINTM                  ((uint32_t)0x0000FFFF)            /*!< Channel interrupt mask */

/********************  Bit definition for USB_OTG_GRXSTSP register  ********************/
#define USB_OTG_GRXSTSP_EPNUM                    ((uint32_t)0x0000000F)            /*!< IN EP interrupt mask bits */
#define USB_OTG_GRXSTSP_BCNT                     ((uint32_t)0x00007FF0)            /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_DPID                     ((uint32_t)0x00018000)            /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_PKTSTS                   ((uint32_t)0x001E0000)            /*!< OUT EP interrupt mask bits */

/********************  Bit definition forUSB_OTG_DAINTMSK register  ********************/
#define USB_OTG_DAINTMSK_IEPM                    ((uint32_t)0x0000FFFF)            /*!< IN EP interrupt mask bits */
#define USB_OTG_DAINTMSK_OEPM                    ((uint32_t)0xFFFF0000)            /*!< OUT EP interrupt mask bits */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   ((uint32_t)0x0000000F)            /*!< Channel number */
#define USB_OTG_CHNUM_0                 ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 ((uint32_t)0x00000004)            /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 ((uint32_t)0x00000008)            /*!<Bit 3 */
#define USB_OTG_BCNT                    ((uint32_t)0x00007FF0)            /*!< Byte count */

#define USB_OTG_DPID                    ((uint32_t)0x00018000)            /*!< Data PID */
#define USB_OTG_DPID_0                  ((uint32_t)0x00008000)            /*!<Bit 0 */
#define USB_OTG_DPID_1                  ((uint32_t)0x00010000)            /*!<Bit 1 */

#define USB_OTG_PKTSTS                  ((uint32_t)0x001E0000)            /*!< Packet status */
#define USB_OTG_PKTSTS_0                ((uint32_t)0x00020000)            /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                ((uint32_t)0x00040000)            /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                ((uint32_t)0x00080000)            /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                ((uint32_t)0x00100000)            /*!<Bit 3 */

#define USB_OTG_EPNUM                   ((uint32_t)0x0000000F)            /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 ((uint32_t)0x00000004)            /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 ((uint32_t)0x00000008)            /*!<Bit 3 */

#define USB_OTG_FRMNUM                  ((uint32_t)0x01E00000)            /*!< Frame number */
#define USB_OTG_FRMNUM_0                ((uint32_t)0x00200000)            /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                ((uint32_t)0x00400000)            /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                ((uint32_t)0x00800000)            /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                ((uint32_t)0x01000000)            /*!<Bit 3 */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   ((uint32_t)0x0000000F)            /*!< Channel number */
#define USB_OTG_CHNUM_0                 ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 ((uint32_t)0x00000004)            /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 ((uint32_t)0x00000008)            /*!<Bit 3 */
#define USB_OTG_BCNT                    ((uint32_t)0x00007FF0)            /*!< Byte count */

#define USB_OTG_DPID                    ((uint32_t)0x00018000)            /*!< Data PID */
#define USB_OTG_DPID_0                  ((uint32_t)0x00008000)            /*!<Bit 0 */
#define USB_OTG_DPID_1                  ((uint32_t)0x00010000)            /*!<Bit 1 */

#define USB_OTG_PKTSTS                  ((uint32_t)0x001E0000)            /*!< Packet status */
#define USB_OTG_PKTSTS_0                ((uint32_t)0x00020000)            /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                ((uint32_t)0x00040000)            /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                ((uint32_t)0x00080000)            /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                ((uint32_t)0x00100000)            /*!<Bit 3 */

#define USB_OTG_EPNUM                   ((uint32_t)0x0000000F)            /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 ((uint32_t)0x00000004)            /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 ((uint32_t)0x00000008)            /*!<Bit 3 */

#define USB_OTG_FRMNUM                  ((uint32_t)0x01E00000)            /*!< Frame number */
#define USB_OTG_FRMNUM_0                ((uint32_t)0x00200000)            /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                ((uint32_t)0x00400000)            /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                ((uint32_t)0x00800000)            /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                ((uint32_t)0x01000000)            /*!<Bit 3 */

/********************  Bit definition forUSB_OTG_GRXFSIZ register  ********************/
#define USB_OTG_GRXFSIZ_RXFD                    ((uint32_t)0x0000FFFF)            /*!< RxFIFO depth */

/********************  Bit definition forUSB_OTG_DVBUSDIS register  ********************/
#define USB_OTG_DVBUSDIS_VBUSDT                  ((uint32_t)0x0000FFFF)            /*!< Device VBUS discharge time */

/********************  Bit definition for OTG register  ********************/
#define USB_OTG_NPTXFSA                 ((uint32_t)0x0000FFFF)            /*!< Nonperiodic transmit RAM start address */
#define USB_OTG_NPTXFD                  ((uint32_t)0xFFFF0000)            /*!< Nonperiodic TxFIFO depth */
#define USB_OTG_TX0FSA                  ((uint32_t)0x0000FFFF)            /*!< Endpoint 0 transmit RAM start address */
#define USB_OTG_TX0FD                   ((uint32_t)0xFFFF0000)            /*!< Endpoint 0 TxFIFO depth */

/********************  Bit definition forUSB_OTG_DVBUSPULSE register  ********************/
#define USB_OTG_DVBUSPULSE_DVBUSP                  ((uint32_t)0x00000FFF)            /*!< Device VBUS pulsing time */

/********************  Bit definition forUSB_OTG_GNPTXSTS register  ********************/
#define USB_OTG_GNPTXSTS_NPTXFSAV                ((uint32_t)0x0000FFFF)            /*!< Nonperiodic TxFIFO space available */

#define USB_OTG_GNPTXSTS_NPTQXSAV                ((uint32_t)0x00FF0000)            /*!< Nonperiodic transmit request queue space available */
#define USB_OTG_GNPTXSTS_NPTQXSAV_0              ((uint32_t)0x00010000)            /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_1              ((uint32_t)0x00020000)            /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_2              ((uint32_t)0x00040000)            /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_3              ((uint32_t)0x00080000)            /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_4              ((uint32_t)0x00100000)            /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_5              ((uint32_t)0x00200000)            /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_6              ((uint32_t)0x00400000)            /*!<Bit 6 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_7              ((uint32_t)0x00800000)            /*!<Bit 7 */

#define USB_OTG_GNPTXSTS_NPTXQTOP                ((uint32_t)0x7F000000)            /*!< Top of the nonperiodic transmit request queue */
#define USB_OTG_GNPTXSTS_NPTXQTOP_0              ((uint32_t)0x01000000)            /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_1              ((uint32_t)0x02000000)            /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_2              ((uint32_t)0x04000000)            /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_3              ((uint32_t)0x08000000)            /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_4              ((uint32_t)0x10000000)            /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_5              ((uint32_t)0x20000000)            /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_6              ((uint32_t)0x40000000)            /*!<Bit 6 */

/********************  Bit definition forUSB_OTG_DTHRCTL register  ********************/
#define USB_OTG_DTHRCTL_NONISOTHREN             ((uint32_t)0x00000001)            /*!< Nonisochronous IN endpoints threshold enable */
#define USB_OTG_DTHRCTL_ISOTHREN                ((uint32_t)0x00000002)            /*!< ISO IN endpoint threshold enable */

#define USB_OTG_DTHRCTL_TXTHRLEN                ((uint32_t)0x000007FC)            /*!< Transmit threshold length */
#define USB_OTG_DTHRCTL_TXTHRLEN_0              ((uint32_t)0x00000004)            /*!<Bit 0 */
#define USB_OTG_DTHRCTL_TXTHRLEN_1              ((uint32_t)0x00000008)            /*!<Bit 1 */
#define USB_OTG_DTHRCTL_TXTHRLEN_2              ((uint32_t)0x00000010)            /*!<Bit 2 */
#define USB_OTG_DTHRCTL_TXTHRLEN_3              ((uint32_t)0x00000020)            /*!<Bit 3 */
#define USB_OTG_DTHRCTL_TXTHRLEN_4              ((uint32_t)0x00000040)            /*!<Bit 4 */
#define USB_OTG_DTHRCTL_TXTHRLEN_5              ((uint32_t)0x00000080)            /*!<Bit 5 */
#define USB_OTG_DTHRCTL_TXTHRLEN_6              ((uint32_t)0x00000100)            /*!<Bit 6 */
#define USB_OTG_DTHRCTL_TXTHRLEN_7              ((uint32_t)0x00000200)            /*!<Bit 7 */
#define USB_OTG_DTHRCTL_TXTHRLEN_8              ((uint32_t)0x00000400)            /*!<Bit 8 */
#define USB_OTG_DTHRCTL_RXTHREN                 ((uint32_t)0x00010000)            /*!< Receive threshold enable */

#define USB_OTG_DTHRCTL_RXTHRLEN                ((uint32_t)0x03FE0000)            /*!< Receive threshold length */
#define USB_OTG_DTHRCTL_RXTHRLEN_0              ((uint32_t)0x00020000)            /*!<Bit 0 */
#define USB_OTG_DTHRCTL_RXTHRLEN_1              ((uint32_t)0x00040000)            /*!<Bit 1 */
#define USB_OTG_DTHRCTL_RXTHRLEN_2              ((uint32_t)0x00080000)            /*!<Bit 2 */
#define USB_OTG_DTHRCTL_RXTHRLEN_3              ((uint32_t)0x00100000)            /*!<Bit 3 */
#define USB_OTG_DTHRCTL_RXTHRLEN_4              ((uint32_t)0x00200000)            /*!<Bit 4 */
#define USB_OTG_DTHRCTL_RXTHRLEN_5              ((uint32_t)0x00400000)            /*!<Bit 5 */
#define USB_OTG_DTHRCTL_RXTHRLEN_6              ((uint32_t)0x00800000)            /*!<Bit 6 */
#define USB_OTG_DTHRCTL_RXTHRLEN_7              ((uint32_t)0x01000000)            /*!<Bit 7 */
#define USB_OTG_DTHRCTL_RXTHRLEN_8              ((uint32_t)0x02000000)            /*!<Bit 8 */
#define USB_OTG_DTHRCTL_ARPEN                   ((uint32_t)0x08000000)            /*!< Arbiter parking enable */

/********************  Bit definition forUSB_OTG_DIEPEMPMSK register  ********************/
#define USB_OTG_DIEPEMPMSK_INEPTXFEM               ((uint32_t)0x0000FFFF)            /*!< IN EP Tx FIFO empty interrupt mask bits */

/********************  Bit definition forUSB_OTG_DEACHINT register  ********************/
#define USB_OTG_DEACHINT_IEP1INT                 ((uint32_t)0x00000002)            /*!< IN endpoint 1interrupt bit */
#define USB_OTG_DEACHINT_OEP1INT                 ((uint32_t)0x00020000)            /*!< OUT endpoint 1 interrupt bit */

/********************  Bit definition forUSB_OTG_GCCFG register  ********************/
#define USB_OTG_GCCFG_PWRDWN                  ((uint32_t)0x00010000)            /*!< Power down */
#define USB_OTG_GCCFG_I2CPADEN                ((uint32_t)0x00020000)            /*!< Enable I2C bus connection for the external I2C PHY interface */
#define USB_OTG_GCCFG_VBUSASEN                ((uint32_t)0x00040000)            /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_VBUSBSEN                ((uint32_t)0x00080000)            /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_SOFOUTEN                ((uint32_t)0x00100000)            /*!< SOF output enable */
#define USB_OTG_GCCFG_NOVBUSSENS              ((uint32_t)0x00200000)            /*!< VBUS sensing disable option */

/********************  Bit definition forUSB_OTG_DEACHINTMSK register  ********************/
#define USB_OTG_DEACHINTMSK_IEP1INTM                ((uint32_t)0x00000002)            /*!< IN Endpoint 1 interrupt mask bit */
#define USB_OTG_DEACHINTMSK_OEP1INTM                ((uint32_t)0x00020000)            /*!< OUT Endpoint 1 interrupt mask bit */

/********************  Bit definition forUSB_OTG_CID register  ********************/
#define USB_OTG_CID_PRODUCT_ID              ((uint32_t)0xFFFFFFFF)            /*!< Product ID field */

/********************  Bit definition forUSB_OTG_DIEPEACHMSK1 register  ********************/
#define USB_OTG_DIEPEACHMSK1_XFRCM                   ((uint32_t)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPEACHMSK1_EPDM                    ((uint32_t)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPEACHMSK1_TOM                     ((uint32_t)0x00000008)            /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK               ((uint32_t)0x00000010)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPEACHMSK1_INEPNMM                 ((uint32_t)0x00000020)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPEACHMSK1_INEPNEM                 ((uint32_t)0x00000040)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPEACHMSK1_TXFURM                  ((uint32_t)0x00000100)            /*!< FIFO underrun mask */
#define USB_OTG_DIEPEACHMSK1_BIM                     ((uint32_t)0x00000200)            /*!< BNA interrupt mask */
#define USB_OTG_DIEPEACHMSK1_NAKM                    ((uint32_t)0x00002000)            /*!< NAK interrupt mask */

/********************  Bit definition forUSB_OTG_HPRT register  ********************/
#define USB_OTG_HPRT_PCSTS                   ((uint32_t)0x00000001)            /*!< Port connect status */
#define USB_OTG_HPRT_PCDET                   ((uint32_t)0x00000002)            /*!< Port connect detected */
#define USB_OTG_HPRT_PENA                    ((uint32_t)0x00000004)            /*!< Port enable */
#define USB_OTG_HPRT_PENCHNG                 ((uint32_t)0x00000008)            /*!< Port enable/disable change */
#define USB_OTG_HPRT_POCA                    ((uint32_t)0x00000010)            /*!< Port overcurrent active */
#define USB_OTG_HPRT_POCCHNG                 ((uint32_t)0x00000020)            /*!< Port overcurrent change */
#define USB_OTG_HPRT_PRES                    ((uint32_t)0x00000040)            /*!< Port resume */
#define USB_OTG_HPRT_PSUSP                   ((uint32_t)0x00000080)            /*!< Port suspend */
#define USB_OTG_HPRT_PRST                    ((uint32_t)0x00000100)            /*!< Port reset */

#define USB_OTG_HPRT_PLSTS                   ((uint32_t)0x00000C00)            /*!< Port line status */
#define USB_OTG_HPRT_PLSTS_0                 ((uint32_t)0x00000400)            /*!<Bit 0 */
#define USB_OTG_HPRT_PLSTS_1                 ((uint32_t)0x00000800)            /*!<Bit 1 */
#define USB_OTG_HPRT_PPWR                    ((uint32_t)0x00001000)            /*!< Port power */

#define USB_OTG_HPRT_PTCTL                   ((uint32_t)0x0001E000)            /*!< Port test control */
#define USB_OTG_HPRT_PTCTL_0                 ((uint32_t)0x00002000)            /*!<Bit 0 */
#define USB_OTG_HPRT_PTCTL_1                 ((uint32_t)0x00004000)            /*!<Bit 1 */
#define USB_OTG_HPRT_PTCTL_2                 ((uint32_t)0x00008000)            /*!<Bit 2 */
#define USB_OTG_HPRT_PTCTL_3                 ((uint32_t)0x00010000)            /*!<Bit 3 */

#define USB_OTG_HPRT_PSPD                    ((uint32_t)0x00060000)            /*!< Port speed */
#define USB_OTG_HPRT_PSPD_0                  ((uint32_t)0x00020000)            /*!<Bit 0 */
#define USB_OTG_HPRT_PSPD_1                  ((uint32_t)0x00040000)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_DOEPEACHMSK1 register  ********************/
#define USB_OTG_DOEPEACHMSK1_XFRCM                   ((uint32_t)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPEACHMSK1_EPDM                    ((uint32_t)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPEACHMSK1_TOM                     ((uint32_t)0x00000008)            /*!< Timeout condition mask */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK               ((uint32_t)0x00000010)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DOEPEACHMSK1_INEPNMM                 ((uint32_t)0x00000020)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DOEPEACHMSK1_INEPNEM                 ((uint32_t)0x00000040)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DOEPEACHMSK1_TXFURM                  ((uint32_t)0x00000100)            /*!< OUT packet error mask */
#define USB_OTG_DOEPEACHMSK1_BIM                     ((uint32_t)0x00000200)            /*!< BNA interrupt mask */
#define USB_OTG_DOEPEACHMSK1_BERRM                   ((uint32_t)0x00001000)            /*!< Bubble error interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NAKM                    ((uint32_t)0x00002000)            /*!< NAK interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NYETM                   ((uint32_t)0x00004000)            /*!< NYET interrupt mask */

/********************  Bit definition forUSB_OTG_HPTXFSIZ register  ********************/
#define USB_OTG_HPTXFSIZ_PTXSA                   ((uint32_t)0x0000FFFF)            /*!< Host periodic TxFIFO start address */
#define USB_OTG_HPTXFSIZ_PTXFD                   ((uint32_t)0xFFFF0000)            /*!< Host periodic TxFIFO depth */

/********************  Bit definition forUSB_OTG_DIEPCTL register  ********************/
#define USB_OTG_DIEPCTL_MPSIZ                   ((uint32_t)0x000007FF)            /*!< Maximum packet size */
#define USB_OTG_DIEPCTL_USBAEP                  ((uint32_t)0x00008000)            /*!< USB active endpoint */
#define USB_OTG_DIEPCTL_EONUM_DPID              ((uint32_t)0x00010000)            /*!< Even/odd frame */
#define USB_OTG_DIEPCTL_NAKSTS                  ((uint32_t)0x00020000)            /*!< NAK status */

#define USB_OTG_DIEPCTL_EPTYP                   ((uint32_t)0x000C0000)            /*!< Endpoint type */
#define USB_OTG_DIEPCTL_EPTYP_0                 ((uint32_t)0x00040000)            /*!<Bit 0 */
#define USB_OTG_DIEPCTL_EPTYP_1                 ((uint32_t)0x00080000)            /*!<Bit 1 */
#define USB_OTG_DIEPCTL_STALL                   ((uint32_t)0x00200000)            /*!< STALL handshake */

#define USB_OTG_DIEPCTL_TXFNUM                  ((uint32_t)0x03C00000)            /*!< TxFIFO number */
#define USB_OTG_DIEPCTL_TXFNUM_0                ((uint32_t)0x00400000)            /*!<Bit 0 */
#define USB_OTG_DIEPCTL_TXFNUM_1                ((uint32_t)0x00800000)            /*!<Bit 1 */
#define USB_OTG_DIEPCTL_TXFNUM_2                ((uint32_t)0x01000000)            /*!<Bit 2 */
#define USB_OTG_DIEPCTL_TXFNUM_3                ((uint32_t)0x02000000)            /*!<Bit 3 */
#define USB_OTG_DIEPCTL_CNAK                    ((uint32_t)0x04000000)            /*!< Clear NAK */
#define USB_OTG_DIEPCTL_SNAK                    ((uint32_t)0x08000000)            /*!< Set NAK */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM          ((uint32_t)0x10000000)            /*!< Set DATA0 PID */
#define USB_OTG_DIEPCTL_SODDFRM                 ((uint32_t)0x20000000)            /*!< Set odd frame */
#define USB_OTG_DIEPCTL_EPDIS                   ((uint32_t)0x40000000)            /*!< Endpoint disable */
#define USB_OTG_DIEPCTL_EPENA                   ((uint32_t)0x80000000)            /*!< Endpoint enable */

/********************  Bit definition forUSB_OTG_HCCHAR register  ********************/
#define USB_OTG_HCCHAR_MPSIZ                   ((uint32_t)0x000007FF)            /*!< Maximum packet size */

#define USB_OTG_HCCHAR_EPNUM                   ((uint32_t)0x00007800)            /*!< Endpoint number */
#define USB_OTG_HCCHAR_EPNUM_0                 ((uint32_t)0x00000800)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPNUM_1                 ((uint32_t)0x00001000)            /*!<Bit 1 */
#define USB_OTG_HCCHAR_EPNUM_2                 ((uint32_t)0x00002000)            /*!<Bit 2 */
#define USB_OTG_HCCHAR_EPNUM_3                 ((uint32_t)0x00004000)            /*!<Bit 3 */
#define USB_OTG_HCCHAR_EPDIR                   ((uint32_t)0x00008000)            /*!< Endpoint direction */
#define USB_OTG_HCCHAR_LSDEV                   ((uint32_t)0x00020000)            /*!< Low-speed device */

#define USB_OTG_HCCHAR_EPTYP                   ((uint32_t)0x000C0000)            /*!< Endpoint type */
#define USB_OTG_HCCHAR_EPTYP_0                 ((uint32_t)0x00040000)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPTYP_1                 ((uint32_t)0x00080000)            /*!<Bit 1 */

#define USB_OTG_HCCHAR_MC                      ((uint32_t)0x00300000)            /*!< Multi Count (MC) / Error Count (EC) */
#define USB_OTG_HCCHAR_MC_0                    ((uint32_t)0x00100000)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_MC_1                    ((uint32_t)0x00200000)            /*!<Bit 1 */

#define USB_OTG_HCCHAR_DAD                     ((uint32_t)0x1FC00000)            /*!< Device address */
#define USB_OTG_HCCHAR_DAD_0                   ((uint32_t)0x00400000)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_DAD_1                   ((uint32_t)0x00800000)            /*!<Bit 1 */
#define USB_OTG_HCCHAR_DAD_2                   ((uint32_t)0x01000000)            /*!<Bit 2 */
#define USB_OTG_HCCHAR_DAD_3                   ((uint32_t)0x02000000)            /*!<Bit 3 */
#define USB_OTG_HCCHAR_DAD_4                   ((uint32_t)0x04000000)            /*!<Bit 4 */
#define USB_OTG_HCCHAR_DAD_5                   ((uint32_t)0x08000000)            /*!<Bit 5 */
#define USB_OTG_HCCHAR_DAD_6                   ((uint32_t)0x10000000)            /*!<Bit 6 */
#define USB_OTG_HCCHAR_ODDFRM                  ((uint32_t)0x20000000)            /*!< Odd frame */
#define USB_OTG_HCCHAR_CHDIS                   ((uint32_t)0x40000000)            /*!< Channel disable */
#define USB_OTG_HCCHAR_CHENA                   ((uint32_t)0x80000000)            /*!< Channel enable */

/********************  Bit definition forUSB_OTG_HCSPLT register  ********************/

#define USB_OTG_HCSPLT_PRTADDR                 ((uint32_t)0x0000007F)            /*!< Port address */
#define USB_OTG_HCSPLT_PRTADDR_0               ((uint32_t)0x00000001)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_PRTADDR_1               ((uint32_t)0x00000002)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_PRTADDR_2               ((uint32_t)0x00000004)            /*!<Bit 2 */
#define USB_OTG_HCSPLT_PRTADDR_3               ((uint32_t)0x00000008)            /*!<Bit 3 */
#define USB_OTG_HCSPLT_PRTADDR_4               ((uint32_t)0x00000010)            /*!<Bit 4 */
#define USB_OTG_HCSPLT_PRTADDR_5               ((uint32_t)0x00000020)            /*!<Bit 5 */
#define USB_OTG_HCSPLT_PRTADDR_6               ((uint32_t)0x00000040)            /*!<Bit 6 */

#define USB_OTG_HCSPLT_HUBADDR                 ((uint32_t)0x00003F80)            /*!< Hub address */
#define USB_OTG_HCSPLT_HUBADDR_0               ((uint32_t)0x00000080)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_HUBADDR_1               ((uint32_t)0x00000100)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_HUBADDR_2               ((uint32_t)0x00000200)            /*!<Bit 2 */
#define USB_OTG_HCSPLT_HUBADDR_3               ((uint32_t)0x00000400)            /*!<Bit 3 */
#define USB_OTG_HCSPLT_HUBADDR_4               ((uint32_t)0x00000800)            /*!<Bit 4 */
#define USB_OTG_HCSPLT_HUBADDR_5               ((uint32_t)0x00001000)            /*!<Bit 5 */
#define USB_OTG_HCSPLT_HUBADDR_6               ((uint32_t)0x00002000)            /*!<Bit 6 */

#define USB_OTG_HCSPLT_XACTPOS                 ((uint32_t)0x0000C000)            /*!< XACTPOS */
#define USB_OTG_HCSPLT_XACTPOS_0               ((uint32_t)0x00004000)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_XACTPOS_1               ((uint32_t)0x00008000)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_COMPLSPLT               ((uint32_t)0x00010000)            /*!< Do complete split */
#define USB_OTG_HCSPLT_SPLITEN                 ((uint32_t)0x80000000)            /*!< Split enable */

/********************  Bit definition forUSB_OTG_HCINT register  ********************/
#define USB_OTG_HCINT_XFRC                    ((uint32_t)0x00000001)            /*!< Transfer completed */
#define USB_OTG_HCINT_CHH                     ((uint32_t)0x00000002)            /*!< Channel halted */
#define USB_OTG_HCINT_AHBERR                  ((uint32_t)0x00000004)            /*!< AHB error */
#define USB_OTG_HCINT_STALL                   ((uint32_t)0x00000008)            /*!< STALL response received interrupt */
#define USB_OTG_HCINT_NAK                     ((uint32_t)0x00000010)            /*!< NAK response received interrupt */
#define USB_OTG_HCINT_ACK                     ((uint32_t)0x00000020)            /*!< ACK response received/transmitted interrupt */
#define USB_OTG_HCINT_NYET                    ((uint32_t)0x00000040)            /*!< Response received interrupt */
#define USB_OTG_HCINT_TXERR                   ((uint32_t)0x00000080)            /*!< Transaction error */
#define USB_OTG_HCINT_BBERR                   ((uint32_t)0x00000100)            /*!< Babble error */
#define USB_OTG_HCINT_FRMOR                   ((uint32_t)0x00000200)            /*!< Frame overrun */
#define USB_OTG_HCINT_DTERR                   ((uint32_t)0x00000400)            /*!< Data toggle error */

/********************  Bit definition forUSB_OTG_DIEPINT register  ********************/
#define USB_OTG_DIEPINT_XFRC                    ((uint32_t)0x00000001)            /*!< Transfer completed interrupt */
#define USB_OTG_DIEPINT_EPDISD                  ((uint32_t)0x00000002)            /*!< Endpoint disabled interrupt */
#define USB_OTG_DIEPINT_TOC                     ((uint32_t)0x00000008)            /*!< Timeout condition */
#define USB_OTG_DIEPINT_ITTXFE                  ((uint32_t)0x00000010)            /*!< IN token received when TxFIFO is empty */
#define USB_OTG_DIEPINT_INEPNE                  ((uint32_t)0x00000040)            /*!< IN endpoint NAK effective */
#define USB_OTG_DIEPINT_TXFE                    ((uint32_t)0x00000080)            /*!< Transmit FIFO empty */
#define USB_OTG_DIEPINT_TXFIFOUDRN              ((uint32_t)0x00000100)            /*!< Transmit Fifo Underrun */
#define USB_OTG_DIEPINT_BNA                     ((uint32_t)0x00000200)            /*!< Buffer not available interrupt */
#define USB_OTG_DIEPINT_PKTDRPSTS               ((uint32_t)0x00000800)            /*!< Packet dropped status */
#define USB_OTG_DIEPINT_BERR                    ((uint32_t)0x00001000)            /*!< Babble error interrupt */
#define USB_OTG_DIEPINT_NAK                     ((uint32_t)0x00002000)            /*!< NAK interrupt */

/********************  Bit definition forUSB_OTG_HCINTMSK register  ********************/
#define USB_OTG_HCINTMSK_XFRCM                   ((uint32_t)0x00000001)            /*!< Transfer completed mask */
#define USB_OTG_HCINTMSK_CHHM                    ((uint32_t)0x00000002)            /*!< Channel halted mask */
#define USB_OTG_HCINTMSK_AHBERR                  ((uint32_t)0x00000004)            /*!< AHB error */
#define USB_OTG_HCINTMSK_STALLM                  ((uint32_t)0x00000008)            /*!< STALL response received interrupt mask */
#define USB_OTG_HCINTMSK_NAKM                    ((uint32_t)0x00000010)            /*!< NAK response received interrupt mask */
#define USB_OTG_HCINTMSK_ACKM                    ((uint32_t)0x00000020)            /*!< ACK response received/transmitted interrupt mask */
#define USB_OTG_HCINTMSK_NYET                    ((uint32_t)0x00000040)            /*!< response received interrupt mask */
#define USB_OTG_HCINTMSK_TXERRM                  ((uint32_t)0x00000080)            /*!< Transaction error mask */
#define USB_OTG_HCINTMSK_BBERRM                  ((uint32_t)0x00000100)            /*!< Babble error mask */
#define USB_OTG_HCINTMSK_FRMORM                  ((uint32_t)0x00000200)            /*!< Frame overrun mask */
#define USB_OTG_HCINTMSK_DTERRM                  ((uint32_t)0x00000400)            /*!< Data toggle error mask */

/********************  Bit definition for USB_OTG_DIEPTSIZ register  ********************/

#define USB_OTG_DIEPTSIZ_XFRSIZ                  ((uint32_t)0x0007FFFF)            /*!< Transfer size */
#define USB_OTG_DIEPTSIZ_PKTCNT                  ((uint32_t)0x1FF80000)            /*!< Packet count */
#define USB_OTG_DIEPTSIZ_MULCNT                  ((uint32_t)0x60000000)            /*!< Packet count */
/********************  Bit definition forUSB_OTG_HCTSIZ register  ********************/
#define USB_OTG_HCTSIZ_XFRSIZ                    ((uint32_t)0x0007FFFF)            /*!< Transfer size */
#define USB_OTG_HCTSIZ_PKTCNT                    ((uint32_t)0x1FF80000)            /*!< Packet count */
#define USB_OTG_HCTSIZ_DOPING                    ((uint32_t)0x80000000)            /*!< Do PING */
#define USB_OTG_HCTSIZ_DPID                      ((uint32_t)0x60000000)            /*!< Data PID */
#define USB_OTG_HCTSIZ_DPID_0                    ((uint32_t)0x20000000)            /*!<Bit 0 */
#define USB_OTG_HCTSIZ_DPID_1                    ((uint32_t)0x40000000)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_DIEPDMA register  ********************/
#define USB_OTG_DIEPDMA_DMAADDR                  ((uint32_t)0xFFFFFFFF)            /*!< DMA address */

/********************  Bit definition forUSB_OTG_HCDMA register  ********************/
#define USB_OTG_HCDMA_DMAADDR                    ((uint32_t)0xFFFFFFFF)            /*!< DMA address */

/********************  Bit definition forUSB_OTG_DTXFSTS register  ********************/
#define USB_OTG_DTXFSTS_INEPTFSAV                ((uint32_t)0x0000FFFF)            /*!< IN endpoint TxFIFO space avail */

/********************  Bit definition forUSB_OTG_DIEPTXF register  ********************/
#define USB_OTG_DIEPTXF_INEPTXSA                 ((uint32_t)0x0000FFFF)            /*!< IN endpoint FIFOx transmit RAM start address */
#define USB_OTG_DIEPTXF_INEPTXFD                 ((uint32_t)0xFFFF0000)            /*!< IN endpoint TxFIFO depth */

/********************  Bit definition forUSB_OTG_DOEPCTL register  ********************/

#define USB_OTG_DOEPCTL_MPSIZ                     ((uint32_t)0x000007FF)            /*!< Maximum packet size */          /*!<Bit 1 */
#define USB_OTG_DOEPCTL_USBAEP                    ((uint32_t)0x00008000)            /*!< USB active endpoint */
#define USB_OTG_DOEPCTL_NAKSTS                    ((uint32_t)0x00020000)            /*!< NAK status */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM            ((uint32_t)0x10000000)            /*!< Set DATA0 PID */
#define USB_OTG_DOEPCTL_SODDFRM                   ((uint32_t)0x20000000)            /*!< Set odd frame */
#define USB_OTG_DOEPCTL_EPTYP                     ((uint32_t)0x000C0000)            /*!< Endpoint type */
#define USB_OTG_DOEPCTL_EPTYP_0                   ((uint32_t)0x00040000)            /*!<Bit 0 */
#define USB_OTG_DOEPCTL_EPTYP_1                   ((uint32_t)0x00080000)            /*!<Bit 1 */
#define USB_OTG_DOEPCTL_SNPM                      ((uint32_t)0x00100000)            /*!< Snoop mode */
#define USB_OTG_DOEPCTL_STALL                     ((uint32_t)0x00200000)            /*!< STALL handshake */
#define USB_OTG_DOEPCTL_CNAK                      ((uint32_t)0x04000000)            /*!< Clear NAK */
#define USB_OTG_DOEPCTL_SNAK                      ((uint32_t)0x08000000)            /*!< Set NAK */
#define USB_OTG_DOEPCTL_EPDIS                     ((uint32_t)0x40000000)            /*!< Endpoint disable */
#define USB_OTG_DOEPCTL_EPENA                     ((uint32_t)0x80000000)            /*!< Endpoint enable */

/********************  Bit definition forUSB_OTG_DOEPINT register  ********************/
#define USB_OTG_DOEPINT_XFRC                    ((uint32_t)0x00000001)            /*!< Transfer completed interrupt */
#define USB_OTG_DOEPINT_EPDISD                  ((uint32_t)0x00000002)            /*!< Endpoint disabled interrupt */
#define USB_OTG_DOEPINT_STUP                    ((uint32_t)0x00000008)            /*!< SETUP phase done */
#define USB_OTG_DOEPINT_OTEPDIS                 ((uint32_t)0x00000010)            /*!< OUT token received when endpoint disabled */
#define USB_OTG_DOEPINT_B2BSTUP                 ((uint32_t)0x00000040)            /*!< Back-to-back SETUP packets received */
#define USB_OTG_DOEPINT_NYET                    ((uint32_t)0x00004000)            /*!< NYET interrupt */

/********************  Bit definition forUSB_OTG_DOEPTSIZ register  ********************/

#define USB_OTG_DOEPTSIZ_XFRSIZ                  ((uint32_t)0x0007FFFF)            /*!< Transfer size */
#define USB_OTG_DOEPTSIZ_PKTCNT                  ((uint32_t)0x1FF80000)            /*!< Packet count */

#define USB_OTG_DOEPTSIZ_STUPCNT                 ((uint32_t)0x60000000)            /*!< SETUP packet count */
#define USB_OTG_DOEPTSIZ_STUPCNT_0               ((uint32_t)0x20000000)            /*!<Bit 0 */
#define USB_OTG_DOEPTSIZ_STUPCNT_1               ((uint32_t)0x40000000)            /*!<Bit 1 */

/********************  Bit definition for PCGCCTL register  ********************/
#define USB_OTG_PCGCCTL_STOPCLK                 ((uint32_t)0x00000001)            /*!< SETUP packet count */
#define USB_OTG_PCGCCTL_GATECLK                 ((uint32_t)0x00000002)            /*!<Bit 0 */
#define USB_OTG_PCGCCTL_PHYSUSP                 ((uint32_t)0x00000010)            /*!<Bit 1 */

/**
  * @}
  */ 

/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */
 
/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == ADC1) || \
                                       ((INSTANCE) == ADC2) || \
                                       ((INSTANCE) == ADC3))

/******************************* CAN Instances ********************************/
#define IS_CAN_ALL_INSTANCE(INSTANCE) (((INSTANCE) == CAN1) || \
                                       ((INSTANCE) == CAN2))
 
/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************* DAC Instances ********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DAC)

/******************************* DCMI Instances *******************************/
#define IS_DCMI_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DCMI)

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
                                        ((INSTANCE) == GPIOF) || \
                                        ((INSTANCE) == GPIOG) || \
                                        ((INSTANCE) == GPIOH) || \
                                        ((INSTANCE) == GPIOI))

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2) || \
                                       ((INSTANCE) == I2C3))

/******************************** I2S Instances *******************************/
#define IS_I2S_ALL_INSTANCE(INSTANCE)  (((INSTANCE) == SPI2) || \
                                        ((INSTANCE) == SPI3))

/*************************** I2S Extended Instances ***************************/
#define IS_I2S_ALL_INSTANCE_EXT(PERIPH)  (((INSTANCE) == SPI2)    || \
                                          ((INSTANCE) == SPI3)    || \
                                          ((INSTANCE) == I2S2ext) || \
                                          ((INSTANCE) == I2S3ext))

/******************************* RNG Instances ********************************/
#define IS_RNG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RNG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/*************************** SPI Extended Instances ***************************/
#define IS_SPI_ALL_INSTANCE_EXT(INSTANCE) (((INSTANCE) == SPI1)    || \
                                           ((INSTANCE) == SPI2)    || \
                                           ((INSTANCE) == SPI3)    || \
                                           ((INSTANCE) == I2S2ext) || \
                                           ((INSTANCE) == I2S3ext))

/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                   ((INSTANCE) == TIM2)   || \
                                   ((INSTANCE) == TIM3)   || \
                                   ((INSTANCE) == TIM4)   || \
                                   ((INSTANCE) == TIM5)   || \
                                   ((INSTANCE) == TIM6)   || \
                                   ((INSTANCE) == TIM7)   || \
                                   ((INSTANCE) == TIM8)   || \
                                   ((INSTANCE) == TIM9)   || \
                                   ((INSTANCE) == TIM10)  || \
                                   ((INSTANCE) == TIM11)  || \
                                   ((INSTANCE) == TIM12)  || \
                                   ((INSTANCE) == TIM13)  || \
                                   ((INSTANCE) == TIM14))

/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)  || \
                                         ((INSTANCE) == TIM2)  || \
                                         ((INSTANCE) == TIM3)  || \
                                         ((INSTANCE) == TIM4)  || \
                                         ((INSTANCE) == TIM5)  || \
                                         ((INSTANCE) == TIM8)  || \
                                         ((INSTANCE) == TIM9)  || \
                                         ((INSTANCE) == TIM10) || \
                                         ((INSTANCE) == TIM11) || \
                                         ((INSTANCE) == TIM12) || \
                                         ((INSTANCE) == TIM13) || \
                                         ((INSTANCE) == TIM14))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM2) || \
                                       ((INSTANCE) == TIM3) || \
                                       ((INSTANCE) == TIM4) || \
                                       ((INSTANCE) == TIM5) || \
                                       ((INSTANCE) == TIM8) || \
                                       ((INSTANCE) == TIM9) || \
                                       ((INSTANCE) == TIM12))

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM2) || \
                                         ((INSTANCE) == TIM3) || \
                                         ((INSTANCE) == TIM4) || \
                                         ((INSTANCE) == TIM5) || \
                                         ((INSTANCE) == TIM8))

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM2) || \
                                       ((INSTANCE) == TIM3) || \
                                       ((INSTANCE) == TIM4) || \
                                       ((INSTANCE) == TIM5) || \
                                       ((INSTANCE) == TIM8))

/******************** TIM Instances : Advanced-control timers *****************/
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                            ((INSTANCE) == TIM8))

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM2) || \
                                         ((INSTANCE) == TIM3) || \
                                         ((INSTANCE) == TIM4) || \
                                         ((INSTANCE) == TIM5) || \
                                         ((INSTANCE) == TIM8))

/****************** TIM Instances : DMA requests generation (UDE) *************/
#define IS_TIM_DMA_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                       ((INSTANCE) == TIM2) || \
                                       ((INSTANCE) == TIM3) || \
                                       ((INSTANCE) == TIM4) || \
                                       ((INSTANCE) == TIM5) || \
                                       ((INSTANCE) == TIM6) || \
                                       ((INSTANCE) == TIM7) || \
                                       ((INSTANCE) == TIM8))

/************ TIM Instances : DMA requests generation (CCxDE) *****************/
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM2) || \
                                          ((INSTANCE) == TIM3) || \
                                          ((INSTANCE) == TIM4) || \
                                          ((INSTANCE) == TIM5) || \
                                          ((INSTANCE) == TIM8))

/************ TIM Instances : DMA requests generation (COMDE) *****************/
#define IS_TIM_CCDMA_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM2) || \
                                          ((INSTANCE) == TIM3) || \
                                          ((INSTANCE) == TIM4) || \
                                          ((INSTANCE) == TIM5) || \
                                          ((INSTANCE) == TIM8)) 

/******************** TIM Instances : DMA burst feature ***********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                             ((INSTANCE) == TIM2) || \
                                             ((INSTANCE) == TIM3) || \
                                             ((INSTANCE) == TIM4) || \
                                             ((INSTANCE) == TIM5) || \
                                             ((INSTANCE) == TIM8))

/****** TIM Instances : master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                          ((INSTANCE) == TIM2) || \
                                          ((INSTANCE) == TIM3) || \
                                          ((INSTANCE) == TIM4) || \
                                          ((INSTANCE) == TIM5) || \
                                          ((INSTANCE) == TIM6) || \
                                          ((INSTANCE) == TIM7) || \
                                          ((INSTANCE) == TIM8) || \
                                          ((INSTANCE) == TIM9) || \
                                          ((INSTANCE) == TIM12))

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                         ((INSTANCE) == TIM2) || \
                                         ((INSTANCE) == TIM3) || \
                                         ((INSTANCE) == TIM4) || \
                                         ((INSTANCE) == TIM5) || \
                                         ((INSTANCE) == TIM8) || \
                                         ((INSTANCE) == TIM9) || \
                                         ((INSTANCE) == TIM12))

/********************** TIM Instances : 32 bit Counter ************************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)(((INSTANCE) == TIM2) || \
                                              ((INSTANCE) == TIM5))

/***************** TIM Instances : external trigger input availabe ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                        ((INSTANCE) == TIM2) || \
                                        ((INSTANCE) == TIM3) || \
                                        ((INSTANCE) == TIM4) || \
                                        ((INSTANCE) == TIM5) || \
                                        ((INSTANCE) == TIM8))

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
    (((INSTANCE) == TIM8) &&                   \
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
     (((CHANNEL) == TIM_CHANNEL_1)))           \
    ||                                         \
    (((INSTANCE) == TIM12) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2)))           \
    ||                                         \
    (((INSTANCE) == TIM13) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1)))           \
    ||                                         \
    (((INSTANCE) == TIM14) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1))))

/************ TIM Instances : complementary output(s) available ***************/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
    ||                                          \
    (((INSTANCE) == TIM8) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3))))

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2) || \
                                     ((INSTANCE) == USART3) || \
                                     ((INSTANCE) == USART6))

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3) || \
                                    ((INSTANCE) == UART4)  || \
                                    ((INSTANCE) == UART5)  || \
                                    ((INSTANCE) == USART6))

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART3) || \
                                           ((INSTANCE) == USART6))

/********************* UART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2) || \
                                         ((INSTANCE) == USART3) || \
                                         ((INSTANCE) == USART6))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3) || \
                                    ((INSTANCE) == UART4)  || \
                                    ((INSTANCE) == UART5)  || \
                                    ((INSTANCE) == USART6))     

/*********************** PCD Instances ****************************************/
#define IS_PCD_ALL_INSTANCE(INSTANCE) (((INSTANCE) == USB_OTG_FS) || \
                                        ((INSTANCE) == USB_OTG_HS))

/*********************** HCD Instances ****************************************/
#define IS_HCD_ALL_INSTANCE(INSTANCE) (((INSTANCE) == USB_OTG_FS) || \
                                       ((INSTANCE) == USB_OTG_HS))

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/****************************** SDIO Instances ********************************/
#define IS_SDIO_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SDIO)

/****************************** USB Exported Constants ************************/
#define USB_OTG_FS_HOST_MAX_CHANNEL_NBR                8
#define USB_OTG_FS_MAX_IN_ENDPOINTS                    4    /* Including EP0 */
#define USB_OTG_FS_MAX_OUT_ENDPOINTS                   4    /* Including EP0 */
#define USB_OTG_FS_TOTAL_FIFO_SIZE                     1280 /* in Bytes */

#define USB_OTG_HS_HOST_MAX_CHANNEL_NBR                12
#define USB_OTG_HS_MAX_IN_ENDPOINTS                    6    /* Including EP0 */
#define USB_OTG_HS_MAX_IN_ENDPOINTS                    6    /* Including EP0 */
#define USB_OTG_HS_TOTAL_FIFO_SIZE                     4096 /* in Bytes */

/******************************************************************************/
/*  For a painless codes migration between the STM32F4xx device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */
/*  product lines within the same STM32F4 Family                              */
/******************************************************************************/

/* Aliases for __IRQn */
#define FMC_IRQn              FSMC_IRQn

/* Aliases for __IRQHandler */
#define FMC_IRQHandler        FSMC_IRQHandler

/**
  * @}
  */

/** @defgroup GPIO_Alternate_function_map GPIO Alternate function map
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
#define GPIO_TIM8_AF3          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping  */
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
#define GPIO_I2S3ext_AF5       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/* AF 6 selection */
#define GPIO_SPI3_AF6          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_I2S2ext_AF6       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */

/* AF 7 selection */
#define GPIO_USART1_AF7        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_USART2_AF7        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_USART3_AF7        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_I2S3ext_AF7       ((uint8_t)0x07)  /* I2S3ext_SD Alternate Function mapping */

/* AF 8 selection */
#define GPIO_UART4_AF8         ((uint8_t)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_UART5_AF8         ((uint8_t)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_USART6_AF8        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/* AF 9 selection */
#define GPIO_CAN1_AF9          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_CAN2_AF9          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_TIM12_AF9         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_TIM13_AF9         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_TIM14_AF9         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */

/* AF 10 selection */
#define GPIO_OTG_FS_AF10        ((uint8_t)0xA)  /* OTG_FS Alternate Function mapping */
#define GPIO_OTG_HS_AF10        ((uint8_t)0xA)  /* OTG_HS Alternate Function mapping */

/* AF 11 selection */
#define GPIO_ETH_AF11           ((uint8_t)0x0B)  /* ETHERNET Alternate Function mapping */

/* AF 12 selection */
#define GPIO_FSMC_AF12          ((uint8_t)0xC)  /* FSMC Alternate Function mapping                     */
#define GPIO_OTG_HS_FS_AF12     ((uint8_t)0xC)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_SDIO_AF12          ((uint8_t)0xC)  /* SDIO Alternate Function mapping                     */

/* AF 13 selection */
#define GPIO_DCMI_AF13          ((uint8_t)0x0D)  /* DCMI Alternate Function mapping */

/* AF 15 selection */
#define GPIO_EVENTOUT_AF15      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */


/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F407xx_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
