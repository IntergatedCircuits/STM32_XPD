/**
  ******************************************************************************
  * @file    stm32f301x8.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-01-18
  * @brief   CMSIS STM32F301x8 Devices Peripheral Access Layer Header File.
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
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32f301x8
  * @{
  */

#ifndef __STM32F301x8_H
#define __STM32F301x8_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             0U       /*!< STM32F301x8 devices do not provide an MPU */
#define __NVIC_PRIO_BITS          4U       /*!< STM32F301x8 devices use 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1U       /*!< STM32F301x8 devices provide an FPU */

/**
  * @}
  */
   
/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F301x8 devices Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
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
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line 19          */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line 20                     */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_TSC_IRQn              = 8,      /*!< EXTI Line2 Interrupt and Touch Sense Controller Interrupt         */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 Interrupt                                          */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 Interrupt                                          */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 Interrupt                                          */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 Interrupt                                          */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 Interrupt                                          */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 Interrupt                                          */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 Interrupt                                          */
  ADC1_IRQn                   = 18,     /*!< ADC1 Interrupts                                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt                  */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)        */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt & EXTI Line24 Interrupt (I2C2 wakeup)        */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup)   */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt & EXTI Line26 Interrupt (USART2 wakeup)   */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt & EXTI Line28 Interrupt (USART3 wakeup)   */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line 17 Interrupt                 */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC underrun error Interrupt             */
  COMP2_IRQn                  = 64,     /*!< COMP2 global Interrupt via EXTI Line22                            */
  COMP4_6_IRQn                = 65,     /*!< COMP4 and COMP6 global Interrupt via EXTI Line30 and 32           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 Event Interrupt & EXTI Line27 Interrupt (I2C3 wakeup)        */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 Error Interrupt                                              */
  FPU_IRQn                    = 81,      /*!< Floating point Interrupt                                          */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"            /* Cortex-M4 processor and core peripherals */
#include "system_stm32f3xx.h"    /* STM32F3xx System Header */
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
            __IO uint32_t ADRDY : 1;                        /*!< ADC ready flag */
            __IO uint32_t EOSMP : 1;                        /*!< ADC group regular end of sampling flag */
            __IO uint32_t EOC : 1;                          /*!< ADC group regular end of unitary conversion flag */
            __IO uint32_t EOS : 1;                          /*!< ADC group regular end of sequence conversions flag */
            __IO uint32_t OVR : 1;                          /*!< ADC group regular overrun flag */
            __IO uint32_t JEOC : 1;                         /*!< ADC group injected end of unitary conversion flag */
            __IO uint32_t JEOS : 1;                         /*!< ADC group injected end of sequence conversions flag */
            __IO uint32_t AWD1 : 1;                         /*!< ADC analog watchdog 1 flag */
            __IO uint32_t AWD2 : 1;                         /*!< ADC analog watchdog 2 flag */
            __IO uint32_t AWD3 : 1;                         /*!< ADC analog watchdog 3 flag */
            __IO uint32_t JQOVF : 1;                        /*!< ADC group injected contexts queue overflow flag */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } ISR;                                                  /*!< ADC Interrupt and Status Register,                 Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t ADRDYIE : 1;                      /*!< ADC ready interrupt */
            __IO uint32_t EOSMPIE : 1;                      /*!< ADC group regular end of sampling interrupt */
            __IO uint32_t EOCIE : 1;                        /*!< ADC group regular end of unitary conversion interrupt */
            __IO uint32_t EOSIE : 1;                        /*!< ADC group regular end of sequence conversions interrupt */
            __IO uint32_t OVRIE : 1;                        /*!< ADC group regular overrun interrupt */
            __IO uint32_t JEOCIE : 1;                       /*!< ADC group injected end of unitary conversion interrupt */
            __IO uint32_t JEOSIE : 1;                       /*!< ADC group injected end of sequence conversions interrupt */
            __IO uint32_t AWD1IE : 1;                       /*!< ADC analog watchdog 1 interrupt */
            __IO uint32_t AWD2IE : 1;                       /*!< ADC analog watchdog 2 interrupt */
            __IO uint32_t AWD3IE : 1;                       /*!< ADC analog watchdog 3 interrupt */
            __IO uint32_t JQOVFIE : 1;                      /*!< ADC group injected contexts queue overflow interrupt */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } IER;                                                  /*!< ADC Interrupt Enable Register,                     Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t ADEN : 1;                         /*!< ADC enable */
            __IO uint32_t ADDIS : 1;                        /*!< ADC disable */
            __IO uint32_t ADSTART : 1;                      /*!< ADC group regular conversion start */
            __IO uint32_t JADSTART : 1;                     /*!< ADC group injected conversion start */
            __IO uint32_t ADSTP : 1;                        /*!< ADC group regular conversion stop */
            __IO uint32_t JADSTP : 1;                       /*!< ADC group injected conversion stop */
                 uint32_t __RESERVED0 : 22;
            __IO uint32_t ADVREGEN : 2;                     /*!< ADC voltage regulator enable */
            __IO uint32_t ADCALDIF : 1;                     /*!< ADC differential mode for calibration */
            __IO uint32_t ADCAL : 1;                        /*!< ADC calibration */
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< ADC control register,                              Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t DMAEN : 1;                        /*!< ADC DMA enable */
            __IO uint32_t DMACFG : 1;                       /*!< ADC DMA configuration */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RES : 2;                          /*!< ADC data resolution */
            __IO uint32_t ALIGN : 1;                        /*!< ADC data alignement */
            __IO uint32_t EXTSEL : 4;                       /*!< ADC group regular external trigger source */
            __IO uint32_t EXTEN : 2;                        /*!< ADC group regular external trigger polarity */
            __IO uint32_t OVRMOD : 1;                       /*!< ADC group regular overrun configuration */
            __IO uint32_t CONT : 1;                         /*!< ADC group regular continuous conversion mode */
            __IO uint32_t AUTDLY : 1;                       /*!< ADC low power auto wait */
            __IO uint32_t AUTOFF : 1;                       /*!< ADC low power auto power off */
            __IO uint32_t DISCEN : 1;                       /*!< ADC group regular sequencer discontinuous mode */
            __IO uint32_t DISCNUM : 3;                      /*!< ADC Discontinuous mode channel count */
            __IO uint32_t JDISCEN : 1;                      /*!< ADC Discontinuous mode on injected channels */
            __IO uint32_t JQM : 1;                          /*!< ADC group injected contexts queue mode */
            __IO uint32_t AWD1SGL : 1;                      /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
            __IO uint32_t AWD1EN : 1;                       /*!< ADC analog watchdog 1 enable on scope ADC group regular */
            __IO uint32_t JAWD1EN : 1;                      /*!< ADC analog watchdog 1 enable on scope ADC group injected */
            __IO uint32_t JAUTO : 1;                        /*!< ADC group injected automatic trigger mode */
            __IO uint32_t AWD1CH : 5;                       /*!< ADC analog watchdog 1 monitored channel selection */
                 uint32_t __RESERVED1 : 1;
        } b;
        __IO uint32_t w;
    } CFGR;                                                 /*!< ADC Configuration register,                        Address offset: 0x0C */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t SMP0 : 3;                         /*!< ADC channel 0 sampling time selection  */
            __IO uint32_t SMP1 : 3;                         /*!< ADC channel 1 sampling time selection  */
            __IO uint32_t SMP2 : 3;                         /*!< ADC channel 2 sampling time selection  */
            __IO uint32_t SMP3 : 3;                         /*!< ADC channel 3 sampling time selection  */
            __IO uint32_t SMP4 : 3;                         /*!< ADC channel 4 sampling time selection  */
            __IO uint32_t SMP5 : 3;                         /*!< ADC channel 5 sampling time selection  */
            __IO uint32_t SMP6 : 3;                         /*!< ADC channel 6 sampling time selection  */
            __IO uint32_t SMP7 : 3;                         /*!< ADC channel 7 sampling time selection  */
            __IO uint32_t SMP8 : 3;                         /*!< ADC channel 8 sampling time selection  */
            __IO uint32_t SMP9 : 3;                         /*!< ADC channel 9 sampling time selection  */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SMPR1;                                                /*!< ADC sample time register 1,                        Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t SMP10 : 3;                        /*!< ADC channel 10 sampling time selection  */
            __IO uint32_t SMP11 : 3;                        /*!< ADC channel 11 sampling time selection  */
            __IO uint32_t SMP12 : 3;                        /*!< ADC channel 12 sampling time selection  */
            __IO uint32_t SMP13 : 3;                        /*!< ADC channel 13 sampling time selection  */
            __IO uint32_t SMP14 : 3;                        /*!< ADC channel 14 sampling time selection  */
            __IO uint32_t SMP15 : 3;                        /*!< ADC channel 15 sampling time selection  */
            __IO uint32_t SMP16 : 3;                        /*!< ADC channel 16 sampling time selection  */
            __IO uint32_t SMP17 : 3;                        /*!< ADC channel 17 sampling time selection  */
            __IO uint32_t SMP18 : 3;                        /*!< ADC channel 18 sampling time selection  */
                 uint32_t __RESERVED0 : 5;
        } b;
        __IO uint32_t w;
    } SMPR2;                                                /*!< ADC sample time register 2,                        Address offset: 0x18 */
         uint32_t __RESERVED1;
    union {
        struct {
            __IO uint32_t LT1 : 12;                         /*!< ADC analog watchdog 1 threshold low */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t HT1 : 12;                         /*!< ADC Analog watchdog 1 threshold high */
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } TR1;                                                  /*!< ADC watchdog threshold register 1,                 Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t LT2 : 8;                          /*!< ADC analog watchdog 2 threshold low */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t HT2 : 8;                          /*!< ADC analog watchdog 2 threshold high */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } TR2;                                                  /*!< ADC watchdog threshold register 2,                 Address offset: 0x24 */
    union {
        struct {
            __IO uint32_t LT3 : 8;                          /*!< ADC analog watchdog 3 threshold low */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t HT3 : 8;                          /*!< ADC analog watchdog 3 threshold high */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } TR3;                                                  /*!< ADC watchdog threshold register 3,                 Address offset: 0x28 */
         uint32_t __RESERVED2;
    union {
        struct {
            __IO uint32_t L : 4;                            /*!< ADC group regular sequencer scan length */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t SQ1 : 5;                          /*!< ADC group regular sequencer rank 1 */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SQ2 : 5;                          /*!< ADC group regular sequencer rank 2 */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SQ3 : 5;                          /*!< ADC group regular sequencer rank 3 */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t SQ4 : 5;                          /*!< ADC group regular sequencer rank 4 */
                 uint32_t __RESERVED4 : 3;
        } b;
        __IO uint32_t w;
    } SQR1;                                                 /*!< ADC regular sequence register 1,                   Address offset: 0x30 */
    union {
        struct {
            __IO uint32_t SQ5 : 5;                          /*!< ADC group regular sequencer rank 5 */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SQ6 : 5;                          /*!< ADC group regular sequencer rank 6 */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SQ7 : 5;                          /*!< ADC group regular sequencer rank 7 */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SQ8 : 5;                          /*!< ADC group regular sequencer rank 8 */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t SQ9 : 5;                          /*!< ADC group regular sequencer rank 9 */
                 uint32_t __RESERVED4 : 3;
        } b;
        __IO uint32_t w;
    } SQR2;                                                 /*!< ADC regular sequence register 2,                   Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t SQ10 : 5;                         /*!< ADC group regular sequencer rank 10 */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SQ11 : 5;                         /*!< ADC group regular sequencer rank 11 */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SQ12 : 5;                         /*!< ADC group regular sequencer rank 12 */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SQ13 : 5;                         /*!< ADC group regular sequencer rank 13 */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t SQ14 : 5;                         /*!< ADC group regular sequencer rank 14 */
                 uint32_t __RESERVED4 : 3;
        } b;
        __IO uint32_t w;
    } SQR3;                                                 /*!< ADC regular sequence register 3,                   Address offset: 0x38 */
    union {
        struct {
            __IO uint32_t SQ15 : 5;                         /*!< ADC group regular sequencer rank 15 */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SQ16 : 5;                         /*!< ADC group regular sequencer rank 16 */
                 uint32_t __RESERVED1 : 21;
        } b;
        __IO uint32_t w;
    } SQR4;                                                 /*!< ADC regular sequence register 4,                   Address offset: 0x3C */
    __IO uint32_t DR;                                       /*!< ADC regular data register,                         Address offset: 0x40 */
         uint32_t __RESERVED3[2];
    union {
        struct {
            __IO uint32_t JL : 2;                           /*!< ADC group injected sequencer scan length */
            __IO uint32_t JEXTSEL : 4;                      /*!< ADC group injected external trigger source */
            __IO uint32_t JEXTEN : 2;                       /*!< ADC group injected external trigger polarity */
            __IO uint32_t JSQ1 : 5;                         /*!< ADC group injected sequencer rank 1 */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t JSQ2 : 5;                         /*!< ADC group injected sequencer rank 2 */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t JSQ3 : 5;                         /*!< ADC group injected sequencer rank 3 */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t JSQ4 : 5;                         /*!< ADC group injected sequencer rank 4 */
                 uint32_t __RESERVED3 : 1;
        } b;
        __IO uint32_t w;
    } JSQR;                                                 /*!< ADC injected sequence register,                    Address offset: 0x4C */
         uint32_t __RESERVED4[4];
    union {
        struct {
            __IO uint32_t OFFSET1 : 12;                     /*!< ADC offset number 1 offset level */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET1_CH : 5;                   /*!< ADC offset number 1 channel selection */
            __IO uint32_t OFFSET1_EN : 1;                   /*!< ADC offset number 1 enable */
        } b;
        __IO uint32_t w;
    } OFR1;                                                 /*!< ADC offset register 1,                             Address offset: 0x60 */
    union {
        struct {
            __IO uint32_t OFFSET2 : 12;                     /*!< ADC offset number 2 offset level */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET2_CH : 5;                   /*!< ADC offset number 2 channel selection */
            __IO uint32_t OFFSET2_EN : 1;                   /*!< ADC offset number 2 enable */
        } b;
        __IO uint32_t w;
    } OFR2;                                                 /*!< ADC offset register 2,                             Address offset: 0x64 */
    union {
        struct {
            __IO uint32_t OFFSET3 : 12;                     /*!< ADC offset number 3 offset level */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET3_CH : 5;                   /*!< ADC offset number 3 channel selection */
            __IO uint32_t OFFSET3_EN : 1;                   /*!< ADC offset number 3 enable */
        } b;
        __IO uint32_t w;
    } OFR3;                                                 /*!< ADC offset register 3,                             Address offset: 0x68 */
    union {
        struct {
            __IO uint32_t OFFSET4 : 12;                     /*!< ADC offset number 4 offset level */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET4_CH : 5;                   /*!< ADC offset number 4 channel selection */
            __IO uint32_t OFFSET4_EN : 1;                   /*!< ADC offset number 4 enable */
        } b;
        __IO uint32_t w;
    } OFR4;                                                 /*!< ADC offset register 4,                             Address offset: 0x6C */
         uint32_t __RESERVED5[4];
    __IO uint32_t JDR1;                                     /*!< ADC injected data register 1,                      Address offset: 0x80 */
    __IO uint32_t JDR2;                                     /*!< ADC injected data register 2,                      Address offset: 0x84 */
    __IO uint32_t JDR3;                                     /*!< ADC injected data register 3,                      Address offset: 0x88 */
    __IO uint32_t JDR4;                                     /*!< ADC injected data register 4,                      Address offset: 0x8C */
         uint32_t __RESERVED6[4];
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t AWD2CH : 18;                      /*!< ADC Analog watchdog 2 channel selection */
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } AWD2CR;                                               /*!< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0 */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t AWD3CH : 18;                      /*!< ADC Analog watchdog 2 channel selection */
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } AWD3CR;                                               /*!< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4 */
         uint32_t __RESERVED7[2];
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DIFSEL : 18;                      /*!< ADC differential modes for channels 1 to 18 */
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } DIFSEL;                                               /*!< ADC  Differential Mode Selection Register,         Address offset: 0xB0 */
    union {
        struct {
            __IO uint32_t CALFACT_S : 7;                    /*!< ADC calibration factor in single-ended mode */
                 uint32_t __RESERVED0 : 9;
            __IO uint32_t CALFACT_D : 7;                    /*!< ADC calibration factor in differential mode */
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } CALFACT;                                              /*!< ADC  Calibration Factors,                          Address offset: 0xB4 */
} ADC_TypeDef;



typedef struct {
    union {
        struct {
            __IO uint32_t ADRDY_MST : 1;                    /*!< ADC multimode master ready flag */
            __IO uint32_t EOSMP_MST : 1;                    /*!< ADC multimode master group regular end of sampling flag */
            __IO uint32_t EOC_MST : 1;                      /*!< ADC multimode master group regular end of unitary conversion flag */
            __IO uint32_t EOS_MST : 1;                      /*!< ADC multimode master group regular end of sequence conversions flag */
            __IO uint32_t OVR_MST : 1;                      /*!< ADC multimode master group regular overrun flag */
            __IO uint32_t JEOC_MST : 1;                     /*!< ADC multimode master group injected end of unitary conversion flag */
            __IO uint32_t JEOS_MST : 1;                     /*!< ADC multimode master group injected end of sequence conversions flag */
            __IO uint32_t AWD1_MST : 1;                     /*!< ADC multimode master analog watchdog 1 flag */
            __IO uint32_t AWD2_MST : 1;                     /*!< ADC multimode master analog watchdog 2 flag */
            __IO uint32_t AWD3_MST : 1;                     /*!< ADC multimode master analog watchdog 3 flag */
            __IO uint32_t JQOVF_MST : 1;                    /*!< ADC multimode master group injected contexts queue overflow flag */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t ADRDY_SLV : 1;                    /*!< ADC multimode slave ready flag */
            __IO uint32_t EOSMP_SLV : 1;                    /*!< ADC multimode slave group regular end of sampling flag */
            __IO uint32_t EOC_SLV : 1;                      /*!< ADC multimode slave group regular end of unitary conversion flag */
            __IO uint32_t EOS_SLV : 1;                      /*!< ADC multimode slave group regular end of sequence conversions flag */
            __IO uint32_t OVR_SLV : 1;                      /*!< ADC multimode slave group regular overrun flag */
            __IO uint32_t JEOC_SLV : 1;                     /*!< ADC multimode slave group injected end of unitary conversion flag */
            __IO uint32_t JEOS_SLV : 1;                     /*!< ADC multimode slave group injected end of sequence conversions flag */
            __IO uint32_t AWD1_SLV : 1;                     /*!< ADC multimode slave analog watchdog 1 flag */
            __IO uint32_t AWD2_SLV : 1;                     /*!< ADC multimode slave analog watchdog 2 flag */
            __IO uint32_t AWD3_SLV : 1;                     /*!< ADC multimode slave analog watchdog 3 flag */
            __IO uint32_t JQOVF_SLV : 1;                    /*!< ADC multimode slave group injected contexts queue overflow flag */
                 uint32_t __RESERVED1 : 5;
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< ADC Common status register,                  Address offset: ADC1/3 base address + 0x300 */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t DUAL : 5;                         /*!< ADC multimode mode selection */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t DELAY : 4;                        /*!< ADC multimode delay between 2 sampling phases */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t DMACFG : 1;                       /*!< ADC multimode DMA transfer configuration */
            __IO uint32_t MDMA : 2;                         /*!< ADC multimode DMA transfer enable */
            __IO uint32_t CKMODE : 2;                       /*!< ADC common clock source and prescaler (prescaler only for clock source synchronous) */
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t VREFEN : 1;                       /*!< ADC internal path to VrefInt enable */
            __IO uint32_t TSEN : 1;                         /*!< ADC internal path to temperature sensor enable */
            __IO uint32_t VBATEN : 1;                       /*!< ADC internal path to battery voltage enable */
                 uint32_t __RESERVED3 : 7;
        } b;
        __IO uint32_t w;
    } CCR;                                                  /*!< ADC common control register,                 Address offset: ADC1/3 base address + 0x308 */
    union {
        struct {
            __IO uint32_t RDATA_MST : 16;                   /*!< ADC multimode master group regular conversion data */
            __IO uint32_t RDATA_SLV : 16;                   /*!< ADC multimode slave group regular conversion data */
        } b;
        __IO uint32_t w;
    } CDR;                                                  /*!< ADC common regular data register for dual
                                                            AND triple modes,                            Address offset: ADC1/3 base address + 0x30C */
} ADC_Common_TypeDef;


/**
  * @brief Analog Comparators
  */
typedef struct {
    union {
        struct {
            __IO uint32_t EN : 1;                           /*!< COMPx enable */
            __IO uint32_t SW1 : 1;                          /*!< COMPx SW1 switch control */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t INSEL : 3;                        /*!< COMPx inverting input select */
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t OUTSEL : 4;                       /*!< COMPx output select */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t POL : 1;                          /*!< COMPx output polarity */
                 uint32_t __RESERVED3 : 2;
            __IO uint32_t BLANKING : 3;                     /*!< COMPx blanking */
                 uint32_t __RESERVED4 : 9;
            __IO uint32_t OUT : 1;                          /*!< COMPx output level */
            __IO uint32_t LOCK : 1;                         /*!< COMPx lock */
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< COMP control and status register, Address offset: 0x00 */
} COMP_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN;                                   /*!< COMPx enable */
        __IO uint32_t SW1;                                  /*!< COMPx SW1 switch control */
             uint32_t __RESERVED0[2];
        __IO uint32_t INSEL[3];                             /*!< COMPx inverting input select */
             uint32_t __RESERVED1[3];
        __IO uint32_t OUTSEL[4];                            /*!< COMPx output select */
             uint32_t __RESERVED2;
        __IO uint32_t POL;                                  /*!< COMPx output polarity */
             uint32_t __RESERVED3[2];
        __IO uint32_t BLANKING[3];                          /*!< COMPx blanking */
             uint32_t __RESERVED4[9];
        __IO uint32_t OUT;                                  /*!< COMPx output level */
        __IO uint32_t LOCK;                                 /*!< COMPx lock */
    } CSR;                                                  /*!< COMP control and status register, Address offset: 0x00 */
} COMP_BitBand_TypeDef;



/**
  * @brief CRC calculation unit
  */
typedef struct {
    __IO uint32_t DR;                                       /*!< CRC Data register,                           Address offset: 0x00 */
    __IO uint8_t IDR;                                       /*!< CRC Independent data register,               Address offset: 0x04 */
         uint8_t __RESERVED0[3];
    union {
        struct {
            __IO uint32_t RESET : 1;                        /*!< RESET the CRC computation unit bit */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t POLYSIZE : 2;                     /*!< Polynomial size bits */
            __IO uint32_t REV_IN : 2;                       /*!< REV_IN Reverse Input Data bits */
            __IO uint32_t REV_OUT : 1;                      /*!< REV_OUT Reverse Output Data bits */
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< CRC Control register,                        Address offset: 0x08 */
         uint32_t __RESERVED1;
    __IO uint32_t INIT;                                     /*!< Initial CRC value register,                  Address offset: 0x10 */
    __IO uint32_t POL;                                      /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_TypeDef;


typedef struct {
    __IO uint32_t DR[32];                                   /*!< CRC Data register,                           Address offset: 0x00 */
    __IO uint32_t IDR[8];                                   /*!< CRC Independent data register,               Address offset: 0x04 */
         uint32_t __RESERVED0[3][8];
    struct {
        __IO uint32_t RESET;                                /*!< RESET the CRC computation unit bit */
             uint32_t __RESERVED0[2];
        __IO uint32_t POLYSIZE[2];                          /*!< Polynomial size bits */
        __IO uint32_t REV_IN[2];                            /*!< REV_IN Reverse Input Data bits */
        __IO uint32_t REV_OUT;                              /*!< REV_OUT Reverse Output Data bits */
             uint32_t __RESERVED1[24];
    } CR;                                                   /*!< CRC Control register,                        Address offset: 0x08 */
         uint32_t __RESERVED1[32];
    __IO uint32_t INIT[32];                                 /*!< Initial CRC value register,                  Address offset: 0x10 */
    __IO uint32_t POL[32];                                  /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_BitBand_TypeDef;



/**
  * @brief Digital to Analog Converter
  */
typedef struct {
    union {
        struct {
            __IO uint32_t EN1 : 1;                          /*!< DAC channel1 enable */
            __IO uint32_t BOFF1 : 1;                        /*!< DAC channel1 output buffer disable */
            __IO uint32_t TEN1 : 1;                         /*!< DAC channel1 Trigger enable */
            __IO uint32_t TSEL1 : 3;                        /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
            __IO uint32_t WAVE1 : 2;                        /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
            __IO uint32_t MAMP1 : 4;                        /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
            __IO uint32_t DMAEN1 : 1;                       /*!< DAC channel1 DMA enable */
            __IO uint32_t DMAUDRIE1 : 1;                    /*!< DAC channel1 DMA underrun IT enable */
                 uint32_t __RESERVED0 : 18;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< DAC control register,                                    Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t SWTRIG1 : 1;                      /*!< DAC channel1 software trigger */
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } SWTRIGR;                                              /*!< DAC software trigger register,                           Address offset: 0x04 */
    struct {
        __IO uint32_t D12R;                                 /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L;                                 /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R;                                  /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR1;                                                 /*!< DAC channel 1 data holding registers */
         uint32_t __RESERVED0[3];
    struct {
        __IO uint32_t D12R;                                 /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L;                                 /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R;                                  /*!< DAC channel 8-bit right-aligned data holding register */
    } DHRD;                                                 /*!< DAC dual channel data holding registers */
    __IO uint32_t DOR1;                                     /*!< DAC channel1 data output register,                       Address offset: 0x2C */
         uint32_t __RESERVED1;
    union {
        struct {
                 uint32_t __RESERVED0 : 13;
            __IO uint32_t DMAUDR1 : 1;                      /*!< DAC channel1 DMA underrun flag */
                 uint32_t __RESERVED1 : 18;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN1;                                  /*!< DAC channel1 enable */
        __IO uint32_t BOFF1;                                /*!< DAC channel1 output buffer disable */
        __IO uint32_t TEN1;                                 /*!< DAC channel1 Trigger enable */
        __IO uint32_t TSEL1[3];                             /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
        __IO uint32_t WAVE1[2];                             /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
        __IO uint32_t MAMP1[4];                             /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
        __IO uint32_t DMAEN1;                               /*!< DAC channel1 DMA enable */
        __IO uint32_t DMAUDRIE1;                            /*!< DAC channel1 DMA underrun IT enable */
             uint32_t __RESERVED0[18];
    } CR;                                                   /*!< DAC control register,                                    Address offset: 0x00 */
    struct {
        __IO uint32_t SWTRIG1;                              /*!< DAC channel1 software trigger */
             uint32_t __RESERVED0[31];
    } SWTRIGR;                                              /*!< DAC software trigger register,                           Address offset: 0x04 */
    struct {
        __IO uint32_t D12R[32];                             /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L[32];                             /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R[32];                              /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR1;                                                 /*!< DAC channel 1 data holding registers */
         uint32_t __RESERVED0[3][32];
    struct {
        __IO uint32_t D12R[32];                             /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L[32];                             /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R[32];                              /*!< DAC channel 8-bit right-aligned data holding register */
    } DHRD;                                                 /*!< DAC dual channel data holding registers */
    __IO uint32_t DOR1[32];                                 /*!< DAC channel1 data output register,                       Address offset: 0x2C */
         uint32_t __RESERVED1[32];
    struct {
             uint32_t __RESERVED0[13];
        __IO uint32_t DMAUDR1;                              /*!< DAC channel1 DMA underrun flag */
             uint32_t __RESERVED1[18];
    } SR;                                                   /*!< DAC status register,                                     Address offset: 0x34 */
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
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t FZ_DBG_TIM6_STOP : 1;
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t FZ_DBG_RTC_STOP : 1;
            __IO uint32_t FZ_DBG_WWDG_STOP : 1;
            __IO uint32_t FZ_DBG_IWDG_STOP : 1;
                 uint32_t __RESERVED2 : 8;
            __IO uint32_t FZ_DBG_I2C1_SMBUS_TIMEOUT : 1;
            __IO uint32_t FZ_DBG_I2C2_SMBUS_TIMEOUT : 1;
                 uint32_t __RESERVED3 : 7;
            __IO uint32_t FZ_DBG_I2C3_SMBUS_TIMEOUT : 1;    /********************  Bit definition for DBGMCU_APB2_FZ register  ************/
                 uint32_t __RESERVED4 : 1;
        } b;
        __IO uint32_t w;
    } APB1FZ;                                               /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t FZ_DBG_TIM1_STOP : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t FZ_DBG_TIM15_STOP : 1;
            __IO uint32_t FZ_DBG_TIM16_STOP : 1;
            __IO uint32_t FZ_DBG_TIM17_STOP : 1;
                 uint32_t __RESERVED1 : 27;
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
            __IO uint32_t EN : 1;                           /*!< Channel enable                      */
            __IO uint32_t TCIE : 1;                         /*!< Transfer complete interrupt enable  */
            __IO uint32_t HTIE : 1;                         /*!< Half Transfer interrupt enable      */
            __IO uint32_t TEIE : 1;                         /*!< Transfer error interrupt enable     */
            __IO uint32_t DIR : 1;                          /*!< Data transfer direction             */
            __IO uint32_t CIRC : 1;                         /*!< Circular mode                       */
            __IO uint32_t PINC : 1;                         /*!< Peripheral increment mode           */
            __IO uint32_t MINC : 1;                         /*!< Memory increment mode               */
            __IO uint32_t PSIZE : 2;                        /*!< PSIZE[1:0] bits (Peripheral size)   */
            __IO uint32_t MSIZE : 2;                        /*!< MSIZE[1:0] bits (Memory size)       */
            __IO uint32_t PL : 2;                           /*!< PL[1:0] bits(Channel Priority level)*/
            __IO uint32_t MEM2MEM : 1;                      /*!< Memory to memory mode               */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CCR;                                                  /*!< DMA channel x configuration register                                           */
    __IO uint32_t CNDTR;                                    /*!< DMA channel x number of data register                                          */
    __IO uint32_t CPAR;                                     /*!< DMA channel x peripheral address register                                      */
    __IO uint32_t CMAR;                                     /*!< DMA channel x memory address register                                          */
} DMA_Channel_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN;                                   /*!< Channel enable                      */
        __IO uint32_t TCIE;                                 /*!< Transfer complete interrupt enable  */
        __IO uint32_t HTIE;                                 /*!< Half Transfer interrupt enable      */
        __IO uint32_t TEIE;                                 /*!< Transfer error interrupt enable     */
        __IO uint32_t DIR;                                  /*!< Data transfer direction             */
        __IO uint32_t CIRC;                                 /*!< Circular mode                       */
        __IO uint32_t PINC;                                 /*!< Peripheral increment mode           */
        __IO uint32_t MINC;                                 /*!< Memory increment mode               */
        __IO uint32_t PSIZE[2];                             /*!< PSIZE[1:0] bits (Peripheral size)   */
        __IO uint32_t MSIZE[2];                             /*!< MSIZE[1:0] bits (Memory size)       */
        __IO uint32_t PL[2];                                /*!< PL[1:0] bits(Channel Priority level)*/
        __IO uint32_t MEM2MEM;                              /*!< Memory to memory mode               */
             uint32_t __RESERVED0[17];
    } CCR;                                                  /*!< DMA channel x configuration register                                           */
    __IO uint32_t CNDTR[32];                                /*!< DMA channel x number of data register                                          */
    __IO uint32_t CPAR[32];                                 /*!< DMA channel x peripheral address register                                      */
    __IO uint32_t CMAR[32];                                 /*!< DMA channel x memory address register                                          */
} DMA_Channel_BitBand_TypeDef;



typedef struct {
    union {
        struct {
            __IO uint32_t GIF1 : 1;                         /*!< Channel 1 Global interrupt flag */
            __IO uint32_t TCIF1 : 1;                        /*!< Channel 1 Transfer Complete flag */
            __IO uint32_t HTIF1 : 1;                        /*!< Channel 1 Half Transfer flag */
            __IO uint32_t TEIF1 : 1;                        /*!< Channel 1 Transfer Error flag */
            __IO uint32_t GIF2 : 1;                         /*!< Channel 2 Global interrupt flag */
            __IO uint32_t TCIF2 : 1;                        /*!< Channel 2 Transfer Complete flag */
            __IO uint32_t HTIF2 : 1;                        /*!< Channel 2 Half Transfer flag */
            __IO uint32_t TEIF2 : 1;                        /*!< Channel 2 Transfer Error flag */
            __IO uint32_t GIF3 : 1;                         /*!< Channel 3 Global interrupt flag */
            __IO uint32_t TCIF3 : 1;                        /*!< Channel 3 Transfer Complete flag */
            __IO uint32_t HTIF3 : 1;                        /*!< Channel 3 Half Transfer flag */
            __IO uint32_t TEIF3 : 1;                        /*!< Channel 3 Transfer Error flag */
            __IO uint32_t GIF4 : 1;                         /*!< Channel 4 Global interrupt flag */
            __IO uint32_t TCIF4 : 1;                        /*!< Channel 4 Transfer Complete flag */
            __IO uint32_t HTIF4 : 1;                        /*!< Channel 4 Half Transfer flag */
            __IO uint32_t TEIF4 : 1;                        /*!< Channel 4 Transfer Error flag */
            __IO uint32_t GIF5 : 1;                         /*!< Channel 5 Global interrupt flag */
            __IO uint32_t TCIF5 : 1;                        /*!< Channel 5 Transfer Complete flag */
            __IO uint32_t HTIF5 : 1;                        /*!< Channel 5 Half Transfer flag */
            __IO uint32_t TEIF5 : 1;                        /*!< Channel 5 Transfer Error flag */
            __IO uint32_t GIF6 : 1;                         /*!< Channel 6 Global interrupt flag */
            __IO uint32_t TCIF6 : 1;                        /*!< Channel 6 Transfer Complete flag */
            __IO uint32_t HTIF6 : 1;                        /*!< Channel 6 Half Transfer flag */
            __IO uint32_t TEIF6 : 1;                        /*!< Channel 6 Transfer Error flag */
            __IO uint32_t GIF7 : 1;                         /*!< Channel 7 Global interrupt flag */
            __IO uint32_t TCIF7 : 1;                        /*!< Channel 7 Transfer Complete flag */
            __IO uint32_t HTIF7 : 1;                        /*!< Channel 7 Half Transfer flag */
            __IO uint32_t TEIF7 : 1;                        /*!< Channel 7 Transfer Error flag */
                 uint32_t __RESERVED0 : 4;
        } b;
        __IO uint32_t w;
    } ISR;                                                  /*!< DMA interrupt status register,                            Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CGIF1 : 1;                        /*!< Channel 1 Global interrupt clear */
            __IO uint32_t CTCIF1 : 1;                       /*!< Channel 1 Transfer Complete clear */
            __IO uint32_t CHTIF1 : 1;                       /*!< Channel 1 Half Transfer clear */
            __IO uint32_t CTEIF1 : 1;                       /*!< Channel 1 Transfer Error clear */
            __IO uint32_t CGIF2 : 1;                        /*!< Channel 2 Global interrupt clear */
            __IO uint32_t CTCIF2 : 1;                       /*!< Channel 2 Transfer Complete clear */
            __IO uint32_t CHTIF2 : 1;                       /*!< Channel 2 Half Transfer clear */
            __IO uint32_t CTEIF2 : 1;                       /*!< Channel 2 Transfer Error clear */
            __IO uint32_t CGIF3 : 1;                        /*!< Channel 3 Global interrupt clear */
            __IO uint32_t CTCIF3 : 1;                       /*!< Channel 3 Transfer Complete clear */
            __IO uint32_t CHTIF3 : 1;                       /*!< Channel 3 Half Transfer clear */
            __IO uint32_t CTEIF3 : 1;                       /*!< Channel 3 Transfer Error clear */
            __IO uint32_t CGIF4 : 1;                        /*!< Channel 4 Global interrupt clear */
            __IO uint32_t CTCIF4 : 1;                       /*!< Channel 4 Transfer Complete clear */
            __IO uint32_t CHTIF4 : 1;                       /*!< Channel 4 Half Transfer clear */
            __IO uint32_t CTEIF4 : 1;                       /*!< Channel 4 Transfer Error clear */
            __IO uint32_t CGIF5 : 1;                        /*!< Channel 5 Global interrupt clear */
            __IO uint32_t CTCIF5 : 1;                       /*!< Channel 5 Transfer Complete clear */
            __IO uint32_t CHTIF5 : 1;                       /*!< Channel 5 Half Transfer clear */
            __IO uint32_t CTEIF5 : 1;                       /*!< Channel 5 Transfer Error clear */
            __IO uint32_t CGIF6 : 1;                        /*!< Channel 6 Global interrupt clear */
            __IO uint32_t CTCIF6 : 1;                       /*!< Channel 6 Transfer Complete clear */
            __IO uint32_t CHTIF6 : 1;                       /*!< Channel 6 Half Transfer clear */
            __IO uint32_t CTEIF6 : 1;                       /*!< Channel 6 Transfer Error clear */
            __IO uint32_t CGIF7 : 1;                        /*!< Channel 7 Global interrupt clear */
            __IO uint32_t CTCIF7 : 1;                       /*!< Channel 7 Transfer Complete clear */
            __IO uint32_t CHTIF7 : 1;                       /*!< Channel 7 Half Transfer clear */
            __IO uint32_t CTEIF7 : 1;                       /*!< Channel 7 Transfer Error clear */
                 uint32_t __RESERVED0 : 4;
        } b;
        __IO uint32_t w;
    } IFCR;                                                 /*!< DMA interrupt flag clear register,                        Address offset: 0x04 */
} DMA_TypeDef;


typedef struct {
    struct {
        __IO uint32_t GIF1;                                 /*!< Channel 1 Global interrupt flag */
        __IO uint32_t TCIF1;                                /*!< Channel 1 Transfer Complete flag */
        __IO uint32_t HTIF1;                                /*!< Channel 1 Half Transfer flag */
        __IO uint32_t TEIF1;                                /*!< Channel 1 Transfer Error flag */
        __IO uint32_t GIF2;                                 /*!< Channel 2 Global interrupt flag */
        __IO uint32_t TCIF2;                                /*!< Channel 2 Transfer Complete flag */
        __IO uint32_t HTIF2;                                /*!< Channel 2 Half Transfer flag */
        __IO uint32_t TEIF2;                                /*!< Channel 2 Transfer Error flag */
        __IO uint32_t GIF3;                                 /*!< Channel 3 Global interrupt flag */
        __IO uint32_t TCIF3;                                /*!< Channel 3 Transfer Complete flag */
        __IO uint32_t HTIF3;                                /*!< Channel 3 Half Transfer flag */
        __IO uint32_t TEIF3;                                /*!< Channel 3 Transfer Error flag */
        __IO uint32_t GIF4;                                 /*!< Channel 4 Global interrupt flag */
        __IO uint32_t TCIF4;                                /*!< Channel 4 Transfer Complete flag */
        __IO uint32_t HTIF4;                                /*!< Channel 4 Half Transfer flag */
        __IO uint32_t TEIF4;                                /*!< Channel 4 Transfer Error flag */
        __IO uint32_t GIF5;                                 /*!< Channel 5 Global interrupt flag */
        __IO uint32_t TCIF5;                                /*!< Channel 5 Transfer Complete flag */
        __IO uint32_t HTIF5;                                /*!< Channel 5 Half Transfer flag */
        __IO uint32_t TEIF5;                                /*!< Channel 5 Transfer Error flag */
        __IO uint32_t GIF6;                                 /*!< Channel 6 Global interrupt flag */
        __IO uint32_t TCIF6;                                /*!< Channel 6 Transfer Complete flag */
        __IO uint32_t HTIF6;                                /*!< Channel 6 Half Transfer flag */
        __IO uint32_t TEIF6;                                /*!< Channel 6 Transfer Error flag */
        __IO uint32_t GIF7;                                 /*!< Channel 7 Global interrupt flag */
        __IO uint32_t TCIF7;                                /*!< Channel 7 Transfer Complete flag */
        __IO uint32_t HTIF7;                                /*!< Channel 7 Half Transfer flag */
        __IO uint32_t TEIF7;                                /*!< Channel 7 Transfer Error flag */
             uint32_t __RESERVED0[4];
    } ISR;                                                  /*!< DMA interrupt status register,                            Address offset: 0x00 */
    struct {
        __IO uint32_t CGIF1;                                /*!< Channel 1 Global interrupt clear */
        __IO uint32_t CTCIF1;                               /*!< Channel 1 Transfer Complete clear */
        __IO uint32_t CHTIF1;                               /*!< Channel 1 Half Transfer clear */
        __IO uint32_t CTEIF1;                               /*!< Channel 1 Transfer Error clear */
        __IO uint32_t CGIF2;                                /*!< Channel 2 Global interrupt clear */
        __IO uint32_t CTCIF2;                               /*!< Channel 2 Transfer Complete clear */
        __IO uint32_t CHTIF2;                               /*!< Channel 2 Half Transfer clear */
        __IO uint32_t CTEIF2;                               /*!< Channel 2 Transfer Error clear */
        __IO uint32_t CGIF3;                                /*!< Channel 3 Global interrupt clear */
        __IO uint32_t CTCIF3;                               /*!< Channel 3 Transfer Complete clear */
        __IO uint32_t CHTIF3;                               /*!< Channel 3 Half Transfer clear */
        __IO uint32_t CTEIF3;                               /*!< Channel 3 Transfer Error clear */
        __IO uint32_t CGIF4;                                /*!< Channel 4 Global interrupt clear */
        __IO uint32_t CTCIF4;                               /*!< Channel 4 Transfer Complete clear */
        __IO uint32_t CHTIF4;                               /*!< Channel 4 Half Transfer clear */
        __IO uint32_t CTEIF4;                               /*!< Channel 4 Transfer Error clear */
        __IO uint32_t CGIF5;                                /*!< Channel 5 Global interrupt clear */
        __IO uint32_t CTCIF5;                               /*!< Channel 5 Transfer Complete clear */
        __IO uint32_t CHTIF5;                               /*!< Channel 5 Half Transfer clear */
        __IO uint32_t CTEIF5;                               /*!< Channel 5 Transfer Error clear */
        __IO uint32_t CGIF6;                                /*!< Channel 6 Global interrupt clear */
        __IO uint32_t CTCIF6;                               /*!< Channel 6 Transfer Complete clear */
        __IO uint32_t CHTIF6;                               /*!< Channel 6 Half Transfer clear */
        __IO uint32_t CTEIF6;                               /*!< Channel 6 Transfer Error clear */
        __IO uint32_t CGIF7;                                /*!< Channel 7 Global interrupt clear */
        __IO uint32_t CTCIF7;                               /*!< Channel 7 Transfer Complete clear */
        __IO uint32_t CHTIF7;                               /*!< Channel 7 Half Transfer clear */
        __IO uint32_t CTEIF7;                               /*!< Channel 7 Transfer Error clear */
             uint32_t __RESERVED0[4];
    } IFCR;                                                 /*!< DMA interrupt flag clear register,                        Address offset: 0x04 */
} DMA_BitBand_TypeDef;



/**
  * @brief External Interrupt/Event Controller
  */
typedef struct {
    __IO uint32_t IMR;                                      /*!<EXTI Interrupt mask register,                             Address offset: 0x00 */
    __IO uint32_t EMR;                                      /*!<EXTI Event mask register,                                 Address offset: 0x04 */
    __IO uint32_t RTSR;                                     /*!<EXTI Rising trigger selection register ,                  Address offset: 0x08 */
    __IO uint32_t FTSR;                                     /*!<EXTI Falling trigger selection register,                  Address offset: 0x0C */
    __IO uint32_t SWIER;                                    /*!<EXTI Software interrupt event register,                   Address offset: 0x10 */
    __IO uint32_t PR;                                       /*!<EXTI Pending register,                                    Address offset: 0x14 */
         uint32_t __RESERVED0[2];
    __IO uint32_t IMR2;                                     /*!< EXTI Interrupt mask register,                            Address offset: 0x20 */
    __IO uint32_t EMR2;                                     /*!< EXTI Event mask register,                                Address offset: 0x24 */
    __IO uint32_t RTSR2;                                    /*!< EXTI Rising trigger selection register,                  Address offset: 0x28 */
    __IO uint32_t FTSR2;                                    /*!< EXTI Falling trigger selection register,                 Address offset: 0x2C */
    __IO uint32_t SWIER2;                                   /*!< EXTI Software interrupt event register,                  Address offset: 0x30 */
    __IO uint32_t PR2;                                      /*!< EXTI Pending register,                                   Address offset: 0x34 */
} EXTI_TypeDef;


typedef struct {
    __IO uint32_t IMR[32];                                  /*!<EXTI Interrupt mask register,                             Address offset: 0x00 */
    __IO uint32_t EMR[32];                                  /*!<EXTI Event mask register,                                 Address offset: 0x04 */
    __IO uint32_t RTSR[32];                                 /*!<EXTI Rising trigger selection register ,                  Address offset: 0x08 */
    __IO uint32_t FTSR[32];                                 /*!<EXTI Falling trigger selection register,                  Address offset: 0x0C */
    __IO uint32_t SWIER[32];                                /*!<EXTI Software interrupt event register,                   Address offset: 0x10 */
    __IO uint32_t PR[32];                                   /*!<EXTI Pending register,                                    Address offset: 0x14 */
         uint32_t __RESERVED0[2][32]; 
    __IO uint32_t IMR2[32];                                 /*!< EXTI Interrupt mask register,                            Address offset: 0x20 */
    __IO uint32_t EMR2[32];                                 /*!< EXTI Event mask register,                                Address offset: 0x24 */
    __IO uint32_t RTSR2[32];                                /*!< EXTI Rising trigger selection register,                  Address offset: 0x28 */
    __IO uint32_t FTSR2[32];                                /*!< EXTI Falling trigger selection register,                 Address offset: 0x2C */
    __IO uint32_t SWIER2[32];                               /*!< EXTI Software interrupt event register,                  Address offset: 0x30 */
    __IO uint32_t PR2[32];                                  /*!< EXTI Pending register,                                   Address offset: 0x34 */
} EXTI_BitBand_TypeDef;



/**
  * @brief FLASH Registers
  */
typedef struct {
    union {
        struct {
            __IO uint32_t LATENCY : 3;                      /*!< LATENCY[2:0] bits (Latency) */
            __IO uint32_t HLFCYA : 1;                       /*!< Flash Half Cycle Access Enable */
            __IO uint32_t PRFTBE : 1;                       /*!< Prefetch Buffer Enable */
            __IO uint32_t PRFTBS : 1;                       /*!< Prefetch Buffer Status */
                 uint32_t __RESERVED0 : 26;
        } b;
        __IO uint32_t w;
    } ACR;                                                  /*!< FLASH access control register,              Address offset: 0x00 */
    __IO uint32_t KEYR;                                     /*!< FLASH key register,                         Address offset: 0x04 */
    __IO uint32_t OPTKEYR;                                  /*!< FLASH option key register,                  Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t BSY : 1;                          /*!< Busy */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PGERR : 1;                        /*!< Programming Error */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t WRPERR : 1;                       /*!< Write Protection Error */
            __IO uint32_t EOP : 1;                          /*!< End of operation */
                 uint32_t __RESERVED2 : 26;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< FLASH status register,                      Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t PG : 1;                           /*!< Programming */
            __IO uint32_t PER : 1;                          /*!< Page Erase */
            __IO uint32_t MER : 1;                          /*!< Mass Erase */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t OPTPG : 1;                        /*!< Option Byte Programming */
            __IO uint32_t OPTER : 1;                        /*!< Option Byte Erase */
            __IO uint32_t STRT : 1;                         /*!< Start */
            __IO uint32_t LOCK : 1;                         /*!< Lock */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t OPTWRE : 1;                       /*!< Option Bytes Write Enable */
            __IO uint32_t ERRIE : 1;                        /*!< Error Interrupt Enable */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t EOPIE : 1;                        /*!< End of operation interrupt enable */
            __IO uint32_t OBL_LAUNCH : 1;                   /*!< OptionBytes Loader Launch */
                 uint32_t __RESERVED3 : 18;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< FLASH control register,                     Address offset: 0x10 */
    __IO uint32_t AR;                                       /*!< FLASH address register,                     Address offset: 0x14 */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t OPTERR : 1;                       /*!< Option Byte Error */
            __IO uint32_t RDPRT : 2;                        /*!< Read protection */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t IWDG_SW : 1;                      /*!< IWDG SW */
            __IO uint32_t nRST_STOP : 1;                    /*!< nRST_STOP */
            __IO uint32_t nRST_STDBY : 1;                   /*!< nRST_STDBY */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t nBOOT1 : 1;                       /*!< nBOOT1 */
            __IO uint32_t VDDA_MONITOR : 1;                 /*!< VDDA_MONITOR */
            __IO uint32_t SRAM_PE : 1;                      /*!< SRAM_PE */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t DATA0 : 8;                        /*!< Data0 */
            __IO uint32_t DATA1 : 8;                        /*!< Data1 */
        } b;
        __IO uint32_t w;
    } OBR;                                                  /*!< FLASH Option byte register,                 Address offset: 0x1C */
    __IO uint32_t WRPR;                                     /*!< FLASH Write register,                       Address offset: 0x20 */
} FLASH_TypeDef;


typedef struct {
    struct {
        __IO uint32_t LATENCY[3];                           /*!< LATENCY[2:0] bits (Latency) */
        __IO uint32_t HLFCYA;                               /*!< Flash Half Cycle Access Enable */
        __IO uint32_t PRFTBE;                               /*!< Prefetch Buffer Enable */
        __IO uint32_t PRFTBS;                               /*!< Prefetch Buffer Status */
             uint32_t __RESERVED0[26];
    } ACR;                                                  /*!< FLASH access control register,              Address offset: 0x00 */
    __IO uint32_t KEYR[32];                                 /*!< FLASH key register,                         Address offset: 0x04 */
    __IO uint32_t OPTKEYR[32];                              /*!< FLASH option key register,                  Address offset: 0x08 */
    struct {
        __IO uint32_t BSY;                                  /*!< Busy */
             uint32_t __RESERVED0;
        __IO uint32_t PGERR;                                /*!< Programming Error */
             uint32_t __RESERVED1;
        __IO uint32_t WRPERR;                               /*!< Write Protection Error */
        __IO uint32_t EOP;                                  /*!< End of operation */
             uint32_t __RESERVED2[26];
    } SR;                                                   /*!< FLASH status register,                      Address offset: 0x0C */
    struct {
        __IO uint32_t PG;                                   /*!< Programming */
        __IO uint32_t PER;                                  /*!< Page Erase */
        __IO uint32_t MER;                                  /*!< Mass Erase */
             uint32_t __RESERVED0;
        __IO uint32_t OPTPG;                                /*!< Option Byte Programming */
        __IO uint32_t OPTER;                                /*!< Option Byte Erase */
        __IO uint32_t STRT;                                 /*!< Start */
        __IO uint32_t LOCK;                                 /*!< Lock */
             uint32_t __RESERVED1;
        __IO uint32_t OPTWRE;                               /*!< Option Bytes Write Enable */
        __IO uint32_t ERRIE;                                /*!< Error Interrupt Enable */
             uint32_t __RESERVED2;
        __IO uint32_t EOPIE;                                /*!< End of operation interrupt enable */
        __IO uint32_t OBL_LAUNCH;                           /*!< OptionBytes Loader Launch */
             uint32_t __RESERVED3[18];
    } CR;                                                   /*!< FLASH control register,                     Address offset: 0x10 */
    __IO uint32_t AR[32];                                   /*!< FLASH address register,                     Address offset: 0x14 */
         uint32_t __RESERVED0[32];
    struct {
        __IO uint32_t OPTERR;                               /*!< Option Byte Error */
        __IO uint32_t RDPRT[2];                             /*!< Read protection */
             uint32_t __RESERVED0[5];
        __IO uint32_t IWDG_SW;                              /*!< IWDG SW */
        __IO uint32_t nRST_STOP;                            /*!< nRST_STOP */
        __IO uint32_t nRST_STDBY;                           /*!< nRST_STDBY */
             uint32_t __RESERVED1;
        __IO uint32_t nBOOT1;                               /*!< nBOOT1 */
        __IO uint32_t VDDA_MONITOR;                         /*!< VDDA_MONITOR */
        __IO uint32_t SRAM_PE;                              /*!< SRAM_PE */
             uint32_t __RESERVED2;
        __IO uint32_t DATA0[8];                             /*!< Data0 */
        __IO uint32_t DATA1[8];                             /*!< Data1 */
    } OBR;                                                  /*!< FLASH Option byte register,                 Address offset: 0x1C */
    __IO uint32_t WRPR[32];                                 /*!< FLASH Write register,                       Address offset: 0x20 */
} FLASH_BitBand_TypeDef;



/**
  * @brief Option Bytes Registers
  */
typedef struct {
    union {
        struct {
            __IO uint16_t RDP : 8;                          /*!< Read protection option byte */
            __IO uint16_t nRDP : 8;                         /*!< Read protection complemented option byte */
        } b;
        __IO uint16_t w;
    } RDP;                                                  /*!<FLASH option byte Read protection,             Address offset: 0x00 */
    union {
        struct {
            __IO uint16_t USER : 8;                         /*!< User option byte */
            __IO uint16_t nUSER : 8;                        /*!< User complemented option byte */
        } b;
        __IO uint16_t w;
    } USER;                                                 /*!<FLASH option byte user options,                Address offset: 0x02 */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint16_t WRP0 : 8;                         /*!< Flash memory write protection option bytes */
            __IO uint16_t nWRP0 : 8;                        /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP0;                                                 /*!<FLASH option byte write protection 0,          Address offset: 0x08 */
    union {
        struct {
            __IO uint16_t WRP1 : 8;                         /*!< Flash memory write protection option bytes */
            __IO uint16_t nWRP1 : 8;                        /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP1;                                                 /*!<FLASH option byte write protection 1,          Address offset: 0x0C */
    union {
        struct {
            __IO uint16_t WRP2 : 8;                         /*!< Flash memory write protection option bytes */
            __IO uint16_t nWRP2 : 8;                        /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP2;                                                 /*!<FLASH option byte write protection 2,          Address offset: 0x10 */
    union {
        struct {
            __IO uint16_t WRP3 : 8;                         /*!< Flash memory write protection option bytes */
            __IO uint16_t nWRP3 : 8;                        /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP3;                                                 /*!<FLASH option byte write protection 3,          Address offset: 0x12 */
} OB_TypeDef;


/**
  * @brief General Purpose I/O
  */
typedef struct {
    __IO uint32_t MODER;                                    /*!< GPIO port mode register,               Address offset: 0x00      */
    __IO uint32_t OTYPER;                                   /*!< GPIO port output type register,        Address offset: 0x04      */
    __IO uint32_t OSPEEDR;                                  /*!< GPIO port output speed register,       Address offset: 0x08      */
    __IO uint32_t PUPDR;                                    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
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
    __IO uint32_t BRR;                                      /*!< GPIO bit reset register,               Address offset: 0x28 */
} GPIO_TypeDef;


/**
  * @brief Operational Amplifier (OPAMP)
  */
typedef struct {
    union {
        struct {
            __IO uint32_t EN : 1;                           /*!< OPAMP enable */
            __IO uint32_t FORCEVP : 1;                      /*!< Connect the internal references to the plus input of the OPAMPX */
            __IO uint32_t VPSEL : 2;                        /*!< Non inverting input selection */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t VMSEL : 2;                        /*!< Inverting input selection */
            __IO uint32_t TCMEN : 1;                        /*!< Timer-Controlled Mux mode enable */
            __IO uint32_t VMSSEL : 1;                       /*!< Inverting input secondary selection */
            __IO uint32_t VPSSEL : 2;                       /*!< Non inverting input secondary selection */
            __IO uint32_t CALON : 1;                        /*!< Calibration mode enable */
            __IO uint32_t CALSEL : 2;                       /*!< Calibration selection */
            __IO uint32_t PGGAIN : 4;                       /*!< Gain in PGA mode */
            __IO uint32_t USERTRIM : 1;                     /*!< User trimming enable */
            __IO uint32_t TRIMOFFSETP : 5;                  /*!< Offset trimming value (PMOS) */
            __IO uint32_t TRIMOFFSETN : 5;                  /*!< Offset trimming value (NMOS) */
            __IO uint32_t TSTREF : 1;                       /*!< It enables the switch to put out the internal reference */
            __IO uint32_t OUTCAL : 1;                       /*!< OPAMP ouput status flag */
            __IO uint32_t LOCK : 1;                         /*!< OPAMP lock */
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< OPAMP control and status register,            Address offset: 0x00 */
} OPAMP_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN;                                   /*!< OPAMP enable */
        __IO uint32_t FORCEVP;                              /*!< Connect the internal references to the plus input of the OPAMPX */
        __IO uint32_t VPSEL[2];                             /*!< Non inverting input selection */
             uint32_t __RESERVED0;
        __IO uint32_t VMSEL[2];                             /*!< Inverting input selection */
        __IO uint32_t TCMEN;                                /*!< Timer-Controlled Mux mode enable */
        __IO uint32_t VMSSEL;                               /*!< Inverting input secondary selection */
        __IO uint32_t VPSSEL[2];                            /*!< Non inverting input secondary selection */
        __IO uint32_t CALON;                                /*!< Calibration mode enable */
        __IO uint32_t CALSEL[2];                            /*!< Calibration selection */
        __IO uint32_t PGGAIN[4];                            /*!< Gain in PGA mode */
        __IO uint32_t USERTRIM;                             /*!< User trimming enable */
        __IO uint32_t TRIMOFFSETP[5];                       /*!< Offset trimming value (PMOS) */
        __IO uint32_t TRIMOFFSETN[5];                       /*!< Offset trimming value (NMOS) */
        __IO uint32_t TSTREF;                               /*!< It enables the switch to put out the internal reference */
        __IO uint32_t OUTCAL;                               /*!< OPAMP ouput status flag */
        __IO uint32_t LOCK;                                 /*!< OPAMP lock */
    } CSR;                                                  /*!< OPAMP control and status register,            Address offset: 0x00 */
} OPAMP_BitBand_TypeDef;



/**
  * @brief System configuration controller
  */
typedef struct {
    union {
        struct {
            __IO uint32_t MEM_MODE : 2;                     /*!< SYSCFG_Memory Remap Config */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t TIM1_ITR3_RMP : 1;                /*!< Timer 1 ITR3 selection */
            __IO uint32_t DAC1_TRIG1_RMP : 1;               /*!< DAC1 Trigger1 remap */
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t TIM16_DMA_RMP : 1;                /*!< Timer 16 DMA remap */
            __IO uint32_t TIM17_DMA_RMP : 1;                /*!< Timer 17 DMA remap */
            __IO uint32_t TIM6DAC1Ch1_DMA_RMP : 1;          /*!< Timer 6 / DAC1 Ch1 DMA remap */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t I2C_PB6_FMP : 1;                  /*!< I2C PB6 Fast mode plus */
            __IO uint32_t I2C_PB7_FMP : 1;                  /*!< I2C PB7 Fast mode plus */
            __IO uint32_t I2C_PB8_FMP : 1;                  /*!< I2C PB8 Fast mode plus */
            __IO uint32_t I2C_PB9_FMP : 1;                  /*!< I2C PB9 Fast mode plus */
            __IO uint32_t I2C1_FMP : 1;                     /*!< I2C1 Fast mode plus */
            __IO uint32_t I2C2_FMP : 1;                     /*!< I2C2 Fast mode plus */
            __IO uint32_t ENCODER_MODE : 2;                 /*!< Encoder Mode */
            __IO uint32_t I2C3_FMP : 1;                     /*!< I2C3 Fast mode plus */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t FPU_IE : 6;                       /*!< Floating Point Unit Interrupt Enable */
        } b;
        __IO uint32_t w;
    } CFGR1;                                                /*!< SYSCFG configuration register 1,                      Address offset: 0x00 */
         uint32_t __RESERVED0;
    __IO uint32_t EXTICR[4];                                /*!< SYSCFG external interrupt configuration registers, Address offset: 0x14-0x08 */
    union {
        struct {
            __IO uint32_t LOCKUP_LOCK : 1;                  /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM4 with Break Input of TIMx */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PVD_LOCK : 1;                     /*!< Enables and locks the PVD connection with TIMx Break Input, as well as the PVDE and PLS[2:0] in the PWR_CR register */
                 uint32_t __RESERVED1 : 29;
        } b;
        __IO uint32_t w;
    } CFGR2;                                                /*!< SYSCFG configuration register 2,                      Address offset: 0x18 */
} SYSCFG_TypeDef;


typedef struct {
    struct {
        __IO uint32_t MEM_MODE[2];                          /*!< SYSCFG_Memory Remap Config */
             uint32_t __RESERVED0[4];
        __IO uint32_t TIM1_ITR3_RMP;                        /*!< Timer 1 ITR3 selection */
        __IO uint32_t DAC1_TRIG1_RMP;                       /*!< DAC1 Trigger1 remap */
             uint32_t __RESERVED1[3];
        __IO uint32_t TIM16_DMA_RMP;                        /*!< Timer 16 DMA remap */
        __IO uint32_t TIM17_DMA_RMP;                        /*!< Timer 17 DMA remap */
        __IO uint32_t TIM6DAC1Ch1_DMA_RMP;                  /*!< Timer 6 / DAC1 Ch1 DMA remap */
             uint32_t __RESERVED2[2];
        __IO uint32_t I2C_PB6_FMP;                          /*!< I2C PB6 Fast mode plus */
        __IO uint32_t I2C_PB7_FMP;                          /*!< I2C PB7 Fast mode plus */
        __IO uint32_t I2C_PB8_FMP;                          /*!< I2C PB8 Fast mode plus */
        __IO uint32_t I2C_PB9_FMP;                          /*!< I2C PB9 Fast mode plus */
        __IO uint32_t I2C1_FMP;                             /*!< I2C1 Fast mode plus */
        __IO uint32_t I2C2_FMP;                             /*!< I2C2 Fast mode plus */
        __IO uint32_t ENCODER_MODE[2];                      /*!< Encoder Mode */
        __IO uint32_t I2C3_FMP;                             /*!< I2C3 Fast mode plus */
             uint32_t __RESERVED3;
        __IO uint32_t FPU_IE[6];                            /*!< Floating Point Unit Interrupt Enable */
    } CFGR1;                                                /*!< SYSCFG configuration register 1,                      Address offset: 0x00 */
         uint32_t __RESERVED0[32];
    __IO uint32_t EXTICR[4][32];                            /*!< SYSCFG external interrupt configuration registers, Address offset: 0x14-0x08 */
    struct {
        __IO uint32_t LOCKUP_LOCK;                          /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM4 with Break Input of TIMx */
             uint32_t __RESERVED0;
        __IO uint32_t PVD_LOCK;                             /*!< Enables and locks the PVD connection with TIMx Break Input, as well as the PVDE and PLS[2:0] in the PWR_CR register */
             uint32_t __RESERVED1[29];
    } CFGR2;                                                /*!< SYSCFG configuration register 2,                      Address offset: 0x18 */
} SYSCFG_BitBand_TypeDef;



/**
  * @brief Inter-integrated Circuit Interface
  */
typedef struct {
    union {
        struct {
            __IO uint32_t PE : 1;                           /*!< Peripheral enable */
            __IO uint32_t TXIE : 1;                         /*!< TX interrupt enable */
            __IO uint32_t RXIE : 1;                         /*!< RX interrupt enable */
            __IO uint32_t ADDRIE : 1;                       /*!< Address match interrupt enable */
            __IO uint32_t NACKIE : 1;                       /*!< NACK received interrupt enable */
            __IO uint32_t STOPIE : 1;                       /*!< STOP detection interrupt enable */
            __IO uint32_t TCIE : 1;                         /*!< Transfer complete interrupt enable */
            __IO uint32_t ERRIE : 1;                        /*!< Errors interrupt enable */
            __IO uint32_t DNF : 4;                          /*!< Digital noise filter */
            __IO uint32_t ANFOFF : 1;                       /*!< Analog noise filter OFF */
            __IO uint32_t SWRST : 1;                        /*!< Software reset */
            __IO uint32_t TXDMAEN : 1;                      /*!< DMA transmission requests enable */
            __IO uint32_t RXDMAEN : 1;                      /*!< DMA reception requests enable */
            __IO uint32_t SBC : 1;                          /*!< Slave byte control */
            __IO uint32_t NOSTRETCH : 1;                    /*!< Clock stretching disable */
            __IO uint32_t WUPEN : 1;                        /*!< Wakeup from STOP enable */
            __IO uint32_t GCEN : 1;                         /*!< General call enable */
            __IO uint32_t SMBHEN : 1;                       /*!< SMBus host address enable */
            __IO uint32_t SMBDEN : 1;                       /*!< SMBus device default address enable */
            __IO uint32_t ALERTEN : 1;                      /*!< SMBus alert enable */
            __IO uint32_t PECEN : 1;                        /*!< PEC enable */
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< I2C Control register 1,            Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t SADD : 10;                        /*!< Slave address (master mode) */
            __IO uint32_t RD_WRN : 1;                       /*!< Transfer direction (master mode) */
            __IO uint32_t ADD10 : 1;                        /*!< 10-bit addressing mode (master mode) */
            __IO uint32_t HEAD10R : 1;                      /*!< 10-bit address header only read direction (master mode) */
            __IO uint32_t START : 1;                        /*!< START generation */
            __IO uint32_t STOP : 1;                         /*!< STOP generation (master mode) */
            __IO uint32_t NACK : 1;                         /*!< NACK generation (slave mode) */
            __IO uint32_t NBYTES : 8;                       /*!< Number of bytes */
            __IO uint32_t RELOAD : 1;                       /*!< NBYTES reload mode */
            __IO uint32_t AUTOEND : 1;                      /*!< Automatic end mode (master mode) */
            __IO uint32_t PECBYTE : 1;                      /*!< Packet error checking byte */
                 uint32_t __RESERVED0 : 5;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< I2C Control register 2,            Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t OA1 : 10;                         /*!< Interface own address 1 */
            __IO uint32_t OA1MODE : 1;                      /*!< Own address 1 10-bit mode */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t OA1EN : 1;                        /*!< Own address 1 enable */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } OAR1;                                                 /*!< I2C Own address 1 register,        Address offset: 0x08 */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t OA2 : 7;                          /*!< Interface own address 2                        */
            __IO uint32_t OA2MSK : 3;                       /*!< Own address 2 masks                            */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t OA2EN : 1;                        /*!< Own address 2 enable                           */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } OAR2;                                                 /*!< I2C Own address 2 register,        Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t SCLL : 8;                         /*!< SCL low period (master mode) */
            __IO uint32_t SCLH : 8;                         /*!< SCL high period (master mode) */
            __IO uint32_t SDADEL : 4;                       /*!< Data hold time */
            __IO uint32_t SCLDEL : 4;                       /*!< Data setup time */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t PRESC : 4;                        /*!< Timings prescaler */
        } b;
        __IO uint32_t w;
    } TIMINGR;                                              /*!< I2C Timing register,               Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t TIMEOUTA : 12;                    /*!< Bus timeout A */
            __IO uint32_t TIDLE : 1;                        /*!< Idle clock timeout detection */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t TIMOUTEN : 1;                     /*!< Clock timeout enable */
            __IO uint32_t TIMEOUTB : 12;                    /*!< Bus timeout B*/
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t TEXTEN : 1;                       /*!< Extended clock timeout enable */
        } b;
        __IO uint32_t w;
    } TIMEOUTR;                                             /*!< I2C Timeout register,              Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t TXE : 1;                          /*!< Transmit data register empty */
            __IO uint32_t TXIS : 1;                         /*!< Transmit interrupt status */
            __IO uint32_t RXNE : 1;                         /*!< Receive data register not empty */
            __IO uint32_t ADDR : 1;                         /*!< Address matched (slave mode)*/
            __IO uint32_t NACKF : 1;                        /*!< NACK received flag */
            __IO uint32_t STOPF : 1;                        /*!< STOP detection flag */
            __IO uint32_t TC : 1;                           /*!< Transfer complete (master mode) */
            __IO uint32_t TCR : 1;                          /*!< Transfer complete reload */
            __IO uint32_t BERR : 1;                         /*!< Bus error */
            __IO uint32_t ARLO : 1;                         /*!< Arbitration lost */
            __IO uint32_t OVR : 1;                          /*!< Overrun/Underrun */
            __IO uint32_t PECERR : 1;                       /*!< PEC error in reception */
            __IO uint32_t TIMEOUT : 1;                      /*!< Timeout or Tlow detection flag */
            __IO uint32_t ALERT : 1;                        /*!< SMBus alert */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t BUSY : 1;                         /*!< Bus busy */
            __IO uint32_t DIR : 1;                          /*!< Transfer direction (slave mode) */
            __IO uint32_t ADDCODE : 7;                      /*!< Address match code (slave mode) */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } ISR;                                                  /*!< I2C Interrupt and status register, Address offset: 0x18 */
    union {
        struct {
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t ADDRCF : 1;                       /*!< Address matched clear flag */
            __IO uint32_t NACKCF : 1;                       /*!< NACK clear flag */
            __IO uint32_t STOPCF : 1;                       /*!< STOP detection clear flag */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t BERRCF : 1;                       /*!< Bus error clear flag */
            __IO uint32_t ARLOCF : 1;                       /*!< Arbitration lost clear flag */
            __IO uint32_t OVRCF : 1;                        /*!< Overrun/Underrun clear flag */
            __IO uint32_t PECCF : 1;                        /*!< PAC error clear flag */
            __IO uint32_t TIMOUTCF : 1;                     /*!< Timeout clear flag */
            __IO uint32_t ALERTCF : 1;                      /*!< Alert clear flag */
                 uint32_t __RESERVED2 : 18;
        } b;
        __IO uint32_t w;
    } ICR;                                                  /*!< I2C Interrupt clear register,      Address offset: 0x1C */
    __IO uint32_t PECR;                                     /*!< I2C PEC register,                  Address offset: 0x20 */
    __IO uint32_t RXDR;                                     /*!< I2C Receive data register,         Address offset: 0x24 */
    __IO uint32_t TXDR;                                     /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PE;                                   /*!< Peripheral enable */
        __IO uint32_t TXIE;                                 /*!< TX interrupt enable */
        __IO uint32_t RXIE;                                 /*!< RX interrupt enable */
        __IO uint32_t ADDRIE;                               /*!< Address match interrupt enable */
        __IO uint32_t NACKIE;                               /*!< NACK received interrupt enable */
        __IO uint32_t STOPIE;                               /*!< STOP detection interrupt enable */
        __IO uint32_t TCIE;                                 /*!< Transfer complete interrupt enable */
        __IO uint32_t ERRIE;                                /*!< Errors interrupt enable */
        __IO uint32_t DNF[4];                               /*!< Digital noise filter */
        __IO uint32_t ANFOFF;                               /*!< Analog noise filter OFF */
        __IO uint32_t SWRST;                                /*!< Software reset */
        __IO uint32_t TXDMAEN;                              /*!< DMA transmission requests enable */
        __IO uint32_t RXDMAEN;                              /*!< DMA reception requests enable */
        __IO uint32_t SBC;                                  /*!< Slave byte control */
        __IO uint32_t NOSTRETCH;                            /*!< Clock stretching disable */
        __IO uint32_t WUPEN;                                /*!< Wakeup from STOP enable */
        __IO uint32_t GCEN;                                 /*!< General call enable */
        __IO uint32_t SMBHEN;                               /*!< SMBus host address enable */
        __IO uint32_t SMBDEN;                               /*!< SMBus device default address enable */
        __IO uint32_t ALERTEN;                              /*!< SMBus alert enable */
        __IO uint32_t PECEN;                                /*!< PEC enable */
             uint32_t __RESERVED0[8];
    } CR1;                                                  /*!< I2C Control register 1,            Address offset: 0x00 */
    struct {
        __IO uint32_t SADD[10];                             /*!< Slave address (master mode) */
        __IO uint32_t RD_WRN;                               /*!< Transfer direction (master mode) */
        __IO uint32_t ADD10;                                /*!< 10-bit addressing mode (master mode) */
        __IO uint32_t HEAD10R;                              /*!< 10-bit address header only read direction (master mode) */
        __IO uint32_t START;                                /*!< START generation */
        __IO uint32_t STOP;                                 /*!< STOP generation (master mode) */
        __IO uint32_t NACK;                                 /*!< NACK generation (slave mode) */
        __IO uint32_t NBYTES[8];                            /*!< Number of bytes */
        __IO uint32_t RELOAD;                               /*!< NBYTES reload mode */
        __IO uint32_t AUTOEND;                              /*!< Automatic end mode (master mode) */
        __IO uint32_t PECBYTE;                              /*!< Packet error checking byte */
             uint32_t __RESERVED0[5];
    } CR2;                                                  /*!< I2C Control register 2,            Address offset: 0x04 */
    struct {
        __IO uint32_t OA1[10];                              /*!< Interface own address 1 */
        __IO uint32_t OA1MODE;                              /*!< Own address 1 10-bit mode */
             uint32_t __RESERVED0[4];
        __IO uint32_t OA1EN;                                /*!< Own address 1 enable */
             uint32_t __RESERVED1[16];
    } OAR1;                                                 /*!< I2C Own address 1 register,        Address offset: 0x08 */
    struct {
             uint32_t __RESERVED0;
        __IO uint32_t OA2[7];                               /*!< Interface own address 2                        */
        __IO uint32_t OA2MSK[3];                            /*!< Own address 2 masks                            */
             uint32_t __RESERVED1[4];
        __IO uint32_t OA2EN;                                /*!< Own address 2 enable                           */
             uint32_t __RESERVED2[16];
    } OAR2;                                                 /*!< I2C Own address 2 register,        Address offset: 0x0C */
    struct {
        __IO uint32_t SCLL[8];                              /*!< SCL low period (master mode) */
        __IO uint32_t SCLH[8];                              /*!< SCL high period (master mode) */
        __IO uint32_t SDADEL[4];                            /*!< Data hold time */
        __IO uint32_t SCLDEL[4];                            /*!< Data setup time */
             uint32_t __RESERVED0[4];
        __IO uint32_t PRESC[4];                             /*!< Timings prescaler */
    } TIMINGR;                                              /*!< I2C Timing register,               Address offset: 0x10 */
    struct {
        __IO uint32_t TIMEOUTA[12];                         /*!< Bus timeout A */
        __IO uint32_t TIDLE;                                /*!< Idle clock timeout detection */
             uint32_t __RESERVED0[2];
        __IO uint32_t TIMOUTEN;                             /*!< Clock timeout enable */
        __IO uint32_t TIMEOUTB[12];                         /*!< Bus timeout B*/
             uint32_t __RESERVED1[3];
        __IO uint32_t TEXTEN;                               /*!< Extended clock timeout enable */
    } TIMEOUTR;                                             /*!< I2C Timeout register,              Address offset: 0x14 */
    struct {
        __IO uint32_t TXE;                                  /*!< Transmit data register empty */
        __IO uint32_t TXIS;                                 /*!< Transmit interrupt status */
        __IO uint32_t RXNE;                                 /*!< Receive data register not empty */
        __IO uint32_t ADDR;                                 /*!< Address matched (slave mode)*/
        __IO uint32_t NACKF;                                /*!< NACK received flag */
        __IO uint32_t STOPF;                                /*!< STOP detection flag */
        __IO uint32_t TC;                                   /*!< Transfer complete (master mode) */
        __IO uint32_t TCR;                                  /*!< Transfer complete reload */
        __IO uint32_t BERR;                                 /*!< Bus error */
        __IO uint32_t ARLO;                                 /*!< Arbitration lost */
        __IO uint32_t OVR;                                  /*!< Overrun/Underrun */
        __IO uint32_t PECERR;                               /*!< PEC error in reception */
        __IO uint32_t TIMEOUT;                              /*!< Timeout or Tlow detection flag */
        __IO uint32_t ALERT;                                /*!< SMBus alert */
             uint32_t __RESERVED0;
        __IO uint32_t BUSY;                                 /*!< Bus busy */
        __IO uint32_t DIR;                                  /*!< Transfer direction (slave mode) */
        __IO uint32_t ADDCODE[7];                           /*!< Address match code (slave mode) */
             uint32_t __RESERVED1[8];
    } ISR;                                                  /*!< I2C Interrupt and status register, Address offset: 0x18 */
    struct {
             uint32_t __RESERVED0[3];
        __IO uint32_t ADDRCF;                               /*!< Address matched clear flag */
        __IO uint32_t NACKCF;                               /*!< NACK clear flag */
        __IO uint32_t STOPCF;                               /*!< STOP detection clear flag */
             uint32_t __RESERVED1[2];
        __IO uint32_t BERRCF;                               /*!< Bus error clear flag */
        __IO uint32_t ARLOCF;                               /*!< Arbitration lost clear flag */
        __IO uint32_t OVRCF;                                /*!< Overrun/Underrun clear flag */
        __IO uint32_t PECCF;                                /*!< PAC error clear flag */
        __IO uint32_t TIMOUTCF;                             /*!< Timeout clear flag */
        __IO uint32_t ALERTCF;                              /*!< Alert clear flag */
             uint32_t __RESERVED2[18];
    } ICR;                                                  /*!< I2C Interrupt clear register,      Address offset: 0x1C */
    __IO uint32_t PECR[32];                                 /*!< I2C PEC register,                  Address offset: 0x20 */
    __IO uint32_t RXDR[32];                                 /*!< I2C Receive data register,         Address offset: 0x24 */
    __IO uint32_t TXDR[32];                                 /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_BitBand_TypeDef;



/**
  * @brief Independent WATCHDOG
  */
typedef struct {
    __IO uint32_t KR;                                       /*!< IWDG Key register,       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t PR : 3;                           /*!< PR[2:0] (Prescaler divider) */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } PR;                                                   /*!< IWDG Prescaler register, Address offset: 0x04 */
    __IO uint32_t RLR;                                      /*!< IWDG Reload register,    Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t PVU : 1;                          /*!< Watchdog prescaler value update */
            __IO uint32_t RVU : 1;                          /*!< Watchdog counter reload value update */
            __IO uint32_t WVU : 1;                          /*!< Watchdog counter window value update */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< IWDG Status register,    Address offset: 0x0C */
    __IO uint32_t WINR;                                     /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;


typedef struct {
    __IO uint32_t KR[32];                                   /*!< IWDG Key register,       Address offset: 0x00 */
    struct {
        __IO uint32_t PR[3];                                /*!< PR[2:0] (Prescaler divider) */
             uint32_t __RESERVED0[29];
    } PR;                                                   /*!< IWDG Prescaler register, Address offset: 0x04 */
    __IO uint32_t RLR[32];                                  /*!< IWDG Reload register,    Address offset: 0x08 */
    struct {
        __IO uint32_t PVU;                                  /*!< Watchdog prescaler value update */
        __IO uint32_t RVU;                                  /*!< Watchdog counter reload value update */
        __IO uint32_t WVU;                                  /*!< Watchdog counter window value update */
             uint32_t __RESERVED0[29];
    } SR;                                                   /*!< IWDG Status register,    Address offset: 0x0C */
    __IO uint32_t WINR[32];                                 /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_BitBand_TypeDef;



/**
  * @brief Power Control
  */
typedef struct {
    union {
        struct {
            __IO uint32_t LPDS : 1;                         /*!< Low-power Deepsleep */
            __IO uint32_t PDDS : 1;                         /*!< Power Down Deepsleep */
            __IO uint32_t CWUF : 1;                         /*!< Clear Wakeup Flag */
            __IO uint32_t CSBF : 1;                         /*!< Clear Standby Flag */
            __IO uint32_t PVDE : 1;                         /*!< Power Voltage Detector Enable */
            __IO uint32_t PLS : 3;                          /*!< PLS[2:0] bits (PVD Level Selection) */
            __IO uint32_t DBP : 1;                          /*!< Disable Backup Domain write protection */
                 uint32_t __RESERVED0 : 23;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< PWR power control register,        Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t WUF : 1;                          /*!< Wakeup Flag */
            __IO uint32_t SBF : 1;                          /*!< Standby Flag */
            __IO uint32_t PVDO : 1;                         /*!< PVD Output */
            __IO uint32_t VREFINTRDYF : 1;                  /*!< Internal voltage reference (VREFINT) ready flag */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t EWUP1 : 1;                        /*!< Enable WKUP pin 1 */
            __IO uint32_t EWUP2 : 1;                        /*!< Enable WKUP pin 2 */
            __IO uint32_t EWUP3 : 1;                        /*!< Enable WKUP pin 3 */
                 uint32_t __RESERVED1 : 21;
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;


typedef struct {
    struct {
        __IO uint32_t LPDS;                                 /*!< Low-power Deepsleep */
        __IO uint32_t PDDS;                                 /*!< Power Down Deepsleep */
        __IO uint32_t CWUF;                                 /*!< Clear Wakeup Flag */
        __IO uint32_t CSBF;                                 /*!< Clear Standby Flag */
        __IO uint32_t PVDE;                                 /*!< Power Voltage Detector Enable */
        __IO uint32_t PLS[3];                               /*!< PLS[2:0] bits (PVD Level Selection) */
        __IO uint32_t DBP;                                  /*!< Disable Backup Domain write protection */
             uint32_t __RESERVED0[23];
    } CR;                                                   /*!< PWR power control register,        Address offset: 0x00 */
    struct {
        __IO uint32_t WUF;                                  /*!< Wakeup Flag */
        __IO uint32_t SBF;                                  /*!< Standby Flag */
        __IO uint32_t PVDO;                                 /*!< PVD Output */
        __IO uint32_t VREFINTRDYF;                          /*!< Internal voltage reference (VREFINT) ready flag */
             uint32_t __RESERVED0[4];
        __IO uint32_t EWUP1;                                /*!< Enable WKUP pin 1 */
        __IO uint32_t EWUP2;                                /*!< Enable WKUP pin 2 */
        __IO uint32_t EWUP3;                                /*!< Enable WKUP pin 3 */
             uint32_t __RESERVED1[21];
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
                 uint32_t __RESERVED2 : 6;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< RCC clock control register,                                  Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t SW : 2;                           /*!< SW[1:0] bits (System clock Switch) */
            __IO uint32_t SWS : 2;                          /*!< SWS[1:0] bits (System Clock Switch Status) */
            __IO uint32_t HPRE : 4;                         /*!< HPRE[3:0] bits (AHB prescaler) */
            __IO uint32_t PPRE1 : 3;                        /*!< PRE1[2:0] bits (APB1 prescaler) */
            __IO uint32_t PPRE2 : 3;                        /*!< PRE2[2:0] bits (APB2 prescaler) */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t PLLSRC : 1;                       /*!< PLL entry clock source */
            __IO uint32_t PLLXTPRE : 1;                     /*!< HSE divider for PLL entry */
            __IO uint32_t PLLMUL : 4;                       /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t I2SSRC : 1;                       /*!< I2S external clock source selection */
            __IO uint32_t MCO : 3;                          /*!< MCO[2:0] bits (Microcontroller Clock Output) */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t MCOPRE : 3;                       /*!< MCOPRE[3:0] bits (Microcontroller Clock Output Prescaler) */
            __IO uint32_t PLLNODIV : 1;                     /*!< Do not divide PLL to MCO */
        } b;
        __IO uint32_t w;
    } CFGR;                                                 /*!< RCC clock configuration register,                            Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t LSIRDYF : 1;                      /*!< LSI Ready Interrupt flag */
            __IO uint32_t LSERDYF : 1;                      /*!< LSE Ready Interrupt flag */
            __IO uint32_t HSIRDYF : 1;                      /*!< HSI Ready Interrupt flag */
            __IO uint32_t HSERDYF : 1;                      /*!< HSE Ready Interrupt flag */
            __IO uint32_t PLLRDYF : 1;                      /*!< PLL Ready Interrupt flag */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t CSSF : 1;                         /*!< Clock Security System Interrupt flag */
            __IO uint32_t LSIRDYIE : 1;                     /*!< LSI Ready Interrupt Enable */
            __IO uint32_t LSERDYIE : 1;                     /*!< LSE Ready Interrupt Enable */
            __IO uint32_t HSIRDYIE : 1;                     /*!< HSI Ready Interrupt Enable */
            __IO uint32_t HSERDYIE : 1;                     /*!< HSE Ready Interrupt Enable */
            __IO uint32_t PLLRDYIE : 1;                     /*!< PLL Ready Interrupt Enable */
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t LSIRDYC : 1;                      /*!< LSI Ready Interrupt Clear */
            __IO uint32_t LSERDYC : 1;                      /*!< LSE Ready Interrupt Clear */
            __IO uint32_t HSIRDYC : 1;                      /*!< HSI Ready Interrupt Clear */
            __IO uint32_t HSERDYC : 1;                      /*!< HSE Ready Interrupt Clear */
            __IO uint32_t PLLRDYC : 1;                      /*!< PLL Ready Interrupt Clear */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t CSSC : 1;                         /*!< Clock Security System Interrupt Clear */
                 uint32_t __RESERVED3 : 8;
        } b;
        __IO uint32_t w;
    } CIR;                                                  /*!< RCC clock interrupt register,                                Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t SYSCFGRST : 1;                    /*!< SYSCFG reset */
                 uint32_t __RESERVED0 : 10;
            __IO uint32_t TIM1RST : 1;                      /*!< TIM1 reset */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t USART1RST : 1;                    /*!< USART1 reset */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t TIM15RST : 1;                     /*!< TIM15 reset */
            __IO uint32_t TIM16RST : 1;                     /*!< TIM16 reset */
            __IO uint32_t TIM17RST : 1;                     /*!< TIM17 reset */
                 uint32_t __RESERVED3 : 13;
        } b;
        __IO uint32_t w;
    } APB2RSTR;                                             /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t TIM2RST : 1;                      /*!< Timer 2 reset */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t TIM6RST : 1;                      /*!< Timer 6 reset */
                 uint32_t __RESERVED1 : 6;
            __IO uint32_t WWDGRST : 1;                      /*!< Window Watchdog reset */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SPI2RST : 1;                      /*!< SPI2 reset */
            __IO uint32_t SPI3RST : 1;                      /*!< SPI3 reset */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t USART2RST : 1;                    /*!< USART 2 reset */
            __IO uint32_t USART3RST : 1;                    /*!< USART 3 reset */
                 uint32_t __RESERVED4 : 2;
            __IO uint32_t I2C1RST : 1;                      /*!< I2C 1 reset */
            __IO uint32_t I2C2RST : 1;                      /*!< I2C 2 reset */
                 uint32_t __RESERVED5 : 5;
            __IO uint32_t PWRRST : 1;                       /*!< PWR reset */
            __IO uint32_t DAC1RST : 1;                      /*!< DAC 1 reset */
            __IO uint32_t I2C3RST : 1;                      /*!< I2C 3 reset */
                 uint32_t __RESERVED6 : 1;
        } b;
        __IO uint32_t w;
    } APB1RSTR;                                             /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t DMA1EN : 1;                       /*!< DMA1 clock enable */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SRAMEN : 1;                       /*!< SRAM interface clock enable */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t FLITFEN : 1;                      /*!< FLITF clock enable */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t CRCEN : 1;                        /*!< CRC clock enable */
                 uint32_t __RESERVED3 : 10;
            __IO uint32_t GPIOAEN : 1;                      /*!< GPIOA clock enable */
            __IO uint32_t GPIOBEN : 1;                      /*!< GPIOB clock enable */
            __IO uint32_t GPIOCEN : 1;                      /*!< GPIOC clock enable */
            __IO uint32_t GPIODEN : 1;                      /*!< GPIOD clock enable */
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t GPIOFEN : 1;                      /*!< GPIOF clock enable */
                 uint32_t __RESERVED5 : 1;
            __IO uint32_t TSCEN : 1;                        /*!< TS clock enable */
                 uint32_t __RESERVED6 : 3;
            __IO uint32_t ADC1EN : 1;                       /*!< ADC1 clock enable */
                 uint32_t __RESERVED7 : 3;
        } b;
        __IO uint32_t w;
    } AHBENR;                                               /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t SYSCFGEN : 1;                     /*!< SYSCFG clock enable */
                 uint32_t __RESERVED0 : 10;
            __IO uint32_t TIM1EN : 1;                       /*!< TIM1 clock enable */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t USART1EN : 1;                     /*!< USART1 clock enable */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t TIM15EN : 1;                      /*!< TIM15 clock enable */
            __IO uint32_t TIM16EN : 1;                      /*!< TIM16 clock enable */
            __IO uint32_t TIM17EN : 1;                      /*!< TIM17 clock enable */
                 uint32_t __RESERVED3 : 13;
        } b;
        __IO uint32_t w;
    } APB2ENR;                                              /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t TIM2EN : 1;                       /*!< Timer 2 clock enable */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t TIM6EN : 1;                       /*!< Timer 6 clock enable */
                 uint32_t __RESERVED1 : 6;
            __IO uint32_t WWDGEN : 1;                       /*!< Window Watchdog clock enable */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SPI2EN : 1;                       /*!< SPI2 clock enable */
            __IO uint32_t SPI3EN : 1;                       /*!< SPI3 clock enable */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t USART2EN : 1;                     /*!< USART 2 clock enable */
            __IO uint32_t USART3EN : 1;                     /*!< USART 3 clock enable */
                 uint32_t __RESERVED4 : 2;
            __IO uint32_t I2C1EN : 1;                       /*!< I2C 1 clock enable */
            __IO uint32_t I2C2EN : 1;                       /*!< I2C 2 clock enable */
                 uint32_t __RESERVED5 : 5;
            __IO uint32_t PWREN : 1;                        /*!< PWR clock enable */
            __IO uint32_t DAC1EN : 1;                       /*!< DAC 1 clock enable */
            __IO uint32_t I2C3EN : 1;                       /*!< I2C 3 clock enable */
                 uint32_t __RESERVED6 : 1;
        } b;
        __IO uint32_t w;
    } APB1ENR;                                              /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t LSEON : 1;                        /*!< External Low Speed oscillator enable */
            __IO uint32_t LSERDY : 1;                       /*!< External Low Speed oscillator Ready */
            __IO uint32_t LSEBYP : 1;                       /*!< External Low Speed oscillator Bypass */
            __IO uint32_t LSEDRV : 2;                       /*!< LSEDRV[1:0] bits (LSE Osc. drive capability) */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t RTCSEL : 2;                       /*!< RTCSEL[1:0] bits (RTC clock source selection) */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t RTCEN : 1;                        /*!< RTC clock enable */
            __IO uint32_t BDRST : 1;                        /*!< Backup domain software reset  */
                 uint32_t __RESERVED2 : 15;
        } b;
        __IO uint32_t w;
    } BDCR;                                                 /*!< RCC Backup domain control register,                          Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t LSION : 1;                        /*!< Internal Low Speed oscillator enable */
            __IO uint32_t LSIRDY : 1;                       /*!< Internal Low Speed oscillator Ready */
                 uint32_t __RESERVED0 : 21;
            __IO uint32_t V18PWRRSTF : 1;                   /*!< V1.8 power domain reset flag */
            __IO uint32_t RMVF : 1;                         /*!< Remove reset flag */
            __IO uint32_t OBLRSTF : 1;                      /*!< OBL reset flag */
            __IO uint32_t PINRSTF : 1;                      /*!< PIN reset flag */
            __IO uint32_t PORRSTF : 1;                      /*!< POR/PDR reset flag */
            __IO uint32_t SFTRSTF : 1;                      /*!< Software Reset flag */
            __IO uint32_t IWDGRSTF : 1;                     /*!< Independent Watchdog reset flag */
            __IO uint32_t WWDGRSTF : 1;                     /*!< Window watchdog reset flag */
            __IO uint32_t LPWRRSTF : 1;                     /*!< Low-Power reset flag */
        } b;
        __IO uint32_t w;
    } CSR;                                                  /*!< RCC clock control & status register,                         Address offset: 0x24 */
    union {
        struct {
                 uint32_t __RESERVED0 : 17;
            __IO uint32_t GPIOARST : 1;                     /*!< GPIOA reset */
            __IO uint32_t GPIOBRST : 1;                     /*!< GPIOB reset */
            __IO uint32_t GPIOCRST : 1;                     /*!< GPIOC reset */
            __IO uint32_t GPIODRST : 1;                     /*!< GPIOD reset */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t GPIOFRST : 1;                     /*!< GPIOF reset */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t TSCRST : 1;                       /*!< TSC reset */
                 uint32_t __RESERVED3 : 3;
            __IO uint32_t ADC1RST : 1;                      /*!< ADC1 reset */
                 uint32_t __RESERVED4 : 3;
        } b;
        __IO uint32_t w;
    } AHBRSTR;                                              /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t PREDIV : 4;                       /*!< PREDIV[3:0] bits */
            __IO uint32_t ADC1PRES : 5;                     /*!< ADC1PRES[8:4] bits */
                 uint32_t __RESERVED0 : 23;
        } b;
        __IO uint32_t w;
    } CFGR2;                                                /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
    union {
        struct {
            __IO uint32_t USART1SW : 2;                     /*!< USART1SW[1:0] bits */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t I2C1SW : 1;                       /*!< I2C1SW bits */
            __IO uint32_t I2C2SW : 1;                       /*!< I2C2SW bits */
            __IO uint32_t I2C3SW : 1;                       /*!< I2C3SW bits */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t TIM1SW : 1;                       /*!< TIM1SW bits */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t TIM15SW : 1;                      /*!< TIM15SW bits */
            __IO uint32_t TIM16SW : 1;                      /*!< TIM16SW bits */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t TIM17SW : 1;                      /*!< TIM17SW bits */
                 uint32_t __RESERVED4 : 2;
            __IO uint32_t USART2SW : 2;                     /*!< USART2SW[1:0] bits */
            __IO uint32_t USART3SW : 2;                     /*!< USART3SW[1:0] bits */
                 uint32_t __RESERVED5 : 12;
        } b;
        __IO uint32_t w;
    } CFGR3;                                                /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
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
             uint32_t __RESERVED2[6];
    } CR;                                                   /*!< RCC clock control register,                                  Address offset: 0x00 */
    struct {
        __IO uint32_t SW[2];                                /*!< SW[1:0] bits (System clock Switch) */
        __IO uint32_t SWS[2];                               /*!< SWS[1:0] bits (System Clock Switch Status) */
        __IO uint32_t HPRE[4];                              /*!< HPRE[3:0] bits (AHB prescaler) */
        __IO uint32_t PPRE1[3];                             /*!< PRE1[2:0] bits (APB1 prescaler) */
        __IO uint32_t PPRE2[3];                             /*!< PRE2[2:0] bits (APB2 prescaler) */
             uint32_t __RESERVED0[2];
        __IO uint32_t PLLSRC;                               /*!< PLL entry clock source */
        __IO uint32_t PLLXTPRE;                             /*!< HSE divider for PLL entry */
        __IO uint32_t PLLMUL[4];                            /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
             uint32_t __RESERVED1;
        __IO uint32_t I2SSRC;                               /*!< I2S external clock source selection */
        __IO uint32_t MCO[3];                               /*!< MCO[2:0] bits (Microcontroller Clock Output) */
             uint32_t __RESERVED2;
        __IO uint32_t MCOPRE[3];                            /*!< MCOPRE[3:0] bits (Microcontroller Clock Output Prescaler) */
        __IO uint32_t PLLNODIV;                             /*!< Do not divide PLL to MCO */
    } CFGR;                                                 /*!< RCC clock configuration register,                            Address offset: 0x04 */
    struct {
        __IO uint32_t LSIRDYF;                              /*!< LSI Ready Interrupt flag */
        __IO uint32_t LSERDYF;                              /*!< LSE Ready Interrupt flag */
        __IO uint32_t HSIRDYF;                              /*!< HSI Ready Interrupt flag */
        __IO uint32_t HSERDYF;                              /*!< HSE Ready Interrupt flag */
        __IO uint32_t PLLRDYF;                              /*!< PLL Ready Interrupt flag */
             uint32_t __RESERVED0[2];
        __IO uint32_t CSSF;                                 /*!< Clock Security System Interrupt flag */
        __IO uint32_t LSIRDYIE;                             /*!< LSI Ready Interrupt Enable */
        __IO uint32_t LSERDYIE;                             /*!< LSE Ready Interrupt Enable */
        __IO uint32_t HSIRDYIE;                             /*!< HSI Ready Interrupt Enable */
        __IO uint32_t HSERDYIE;                             /*!< HSE Ready Interrupt Enable */
        __IO uint32_t PLLRDYIE;                             /*!< PLL Ready Interrupt Enable */
             uint32_t __RESERVED1[3];
        __IO uint32_t LSIRDYC;                              /*!< LSI Ready Interrupt Clear */
        __IO uint32_t LSERDYC;                              /*!< LSE Ready Interrupt Clear */
        __IO uint32_t HSIRDYC;                              /*!< HSI Ready Interrupt Clear */
        __IO uint32_t HSERDYC;                              /*!< HSE Ready Interrupt Clear */
        __IO uint32_t PLLRDYC;                              /*!< PLL Ready Interrupt Clear */
             uint32_t __RESERVED2[2];
        __IO uint32_t CSSC;                                 /*!< Clock Security System Interrupt Clear */
             uint32_t __RESERVED3[8];
    } CIR;                                                  /*!< RCC clock interrupt register,                                Address offset: 0x08 */
    struct {
        __IO uint32_t SYSCFGRST;                            /*!< SYSCFG reset */
             uint32_t __RESERVED0[10];
        __IO uint32_t TIM1RST;                              /*!< TIM1 reset */
             uint32_t __RESERVED1[2];
        __IO uint32_t USART1RST;                            /*!< USART1 reset */
             uint32_t __RESERVED2;
        __IO uint32_t TIM15RST;                             /*!< TIM15 reset */
        __IO uint32_t TIM16RST;                             /*!< TIM16 reset */
        __IO uint32_t TIM17RST;                             /*!< TIM17 reset */
             uint32_t __RESERVED3[13];
    } APB2RSTR;                                             /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
    struct {
        __IO uint32_t TIM2RST;                              /*!< Timer 2 reset */
             uint32_t __RESERVED0[3];
        __IO uint32_t TIM6RST;                              /*!< Timer 6 reset */
             uint32_t __RESERVED1[6];
        __IO uint32_t WWDGRST;                              /*!< Window Watchdog reset */
             uint32_t __RESERVED2[2];
        __IO uint32_t SPI2RST;                              /*!< SPI2 reset */
        __IO uint32_t SPI3RST;                              /*!< SPI3 reset */
             uint32_t __RESERVED3;
        __IO uint32_t USART2RST;                            /*!< USART 2 reset */
        __IO uint32_t USART3RST;                            /*!< USART 3 reset */
             uint32_t __RESERVED4[2];
        __IO uint32_t I2C1RST;                              /*!< I2C 1 reset */
        __IO uint32_t I2C2RST;                              /*!< I2C 2 reset */
             uint32_t __RESERVED5[5];
        __IO uint32_t PWRRST;                               /*!< PWR reset */
        __IO uint32_t DAC1RST;                              /*!< DAC 1 reset */
        __IO uint32_t I2C3RST;                              /*!< I2C 3 reset */
             uint32_t __RESERVED6;
    } APB1RSTR;                                             /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
    struct {
        __IO uint32_t DMA1EN;                               /*!< DMA1 clock enable */
             uint32_t __RESERVED0;
        __IO uint32_t SRAMEN;                               /*!< SRAM interface clock enable */
             uint32_t __RESERVED1;
        __IO uint32_t FLITFEN;                              /*!< FLITF clock enable */
             uint32_t __RESERVED2;
        __IO uint32_t CRCEN;                                /*!< CRC clock enable */
             uint32_t __RESERVED3[10];
        __IO uint32_t GPIOAEN;                              /*!< GPIOA clock enable */
        __IO uint32_t GPIOBEN;                              /*!< GPIOB clock enable */
        __IO uint32_t GPIOCEN;                              /*!< GPIOC clock enable */
        __IO uint32_t GPIODEN;                              /*!< GPIOD clock enable */
             uint32_t __RESERVED4;
        __IO uint32_t GPIOFEN;                              /*!< GPIOF clock enable */
             uint32_t __RESERVED5;
        __IO uint32_t TSCEN;                                /*!< TS clock enable */
             uint32_t __RESERVED6[3];
        __IO uint32_t ADC1EN;                               /*!< ADC1 clock enable */
             uint32_t __RESERVED7[3];
    } AHBENR;                                               /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
    struct {
        __IO uint32_t SYSCFGEN;                             /*!< SYSCFG clock enable */
             uint32_t __RESERVED0[10];
        __IO uint32_t TIM1EN;                               /*!< TIM1 clock enable */
             uint32_t __RESERVED1[2];
        __IO uint32_t USART1EN;                             /*!< USART1 clock enable */
             uint32_t __RESERVED2;
        __IO uint32_t TIM15EN;                              /*!< TIM15 clock enable */
        __IO uint32_t TIM16EN;                              /*!< TIM16 clock enable */
        __IO uint32_t TIM17EN;                              /*!< TIM17 clock enable */
             uint32_t __RESERVED3[13];
    } APB2ENR;                                              /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
    struct {
        __IO uint32_t TIM2EN;                               /*!< Timer 2 clock enable */
             uint32_t __RESERVED0[3];
        __IO uint32_t TIM6EN;                               /*!< Timer 6 clock enable */
             uint32_t __RESERVED1[6];
        __IO uint32_t WWDGEN;                               /*!< Window Watchdog clock enable */
             uint32_t __RESERVED2[2];
        __IO uint32_t SPI2EN;                               /*!< SPI2 clock enable */
        __IO uint32_t SPI3EN;                               /*!< SPI3 clock enable */
             uint32_t __RESERVED3;
        __IO uint32_t USART2EN;                             /*!< USART 2 clock enable */
        __IO uint32_t USART3EN;                             /*!< USART 3 clock enable */
             uint32_t __RESERVED4[2];
        __IO uint32_t I2C1EN;                               /*!< I2C 1 clock enable */
        __IO uint32_t I2C2EN;                               /*!< I2C 2 clock enable */
             uint32_t __RESERVED5[5];
        __IO uint32_t PWREN;                                /*!< PWR clock enable */
        __IO uint32_t DAC1EN;                               /*!< DAC 1 clock enable */
        __IO uint32_t I2C3EN;                               /*!< I2C 3 clock enable */
             uint32_t __RESERVED6;
    } APB1ENR;                                              /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
    struct {
        __IO uint32_t LSEON;                                /*!< External Low Speed oscillator enable */
        __IO uint32_t LSERDY;                               /*!< External Low Speed oscillator Ready */
        __IO uint32_t LSEBYP;                               /*!< External Low Speed oscillator Bypass */
        __IO uint32_t LSEDRV[2];                            /*!< LSEDRV[1:0] bits (LSE Osc. drive capability) */
             uint32_t __RESERVED0[3];
        __IO uint32_t RTCSEL[2];                            /*!< RTCSEL[1:0] bits (RTC clock source selection) */
             uint32_t __RESERVED1[5];
        __IO uint32_t RTCEN;                                /*!< RTC clock enable */
        __IO uint32_t BDRST;                                /*!< Backup domain software reset  */
             uint32_t __RESERVED2[15];
    } BDCR;                                                 /*!< RCC Backup domain control register,                          Address offset: 0x20 */
    struct {
        __IO uint32_t LSION;                                /*!< Internal Low Speed oscillator enable */
        __IO uint32_t LSIRDY;                               /*!< Internal Low Speed oscillator Ready */
             uint32_t __RESERVED0[21];
        __IO uint32_t V18PWRRSTF;                           /*!< V1.8 power domain reset flag */
        __IO uint32_t RMVF;                                 /*!< Remove reset flag */
        __IO uint32_t OBLRSTF;                              /*!< OBL reset flag */
        __IO uint32_t PINRSTF;                              /*!< PIN reset flag */
        __IO uint32_t PORRSTF;                              /*!< POR/PDR reset flag */
        __IO uint32_t SFTRSTF;                              /*!< Software Reset flag */
        __IO uint32_t IWDGRSTF;                             /*!< Independent Watchdog reset flag */
        __IO uint32_t WWDGRSTF;                             /*!< Window watchdog reset flag */
        __IO uint32_t LPWRRSTF;                             /*!< Low-Power reset flag */
    } CSR;                                                  /*!< RCC clock control & status register,                         Address offset: 0x24 */
    struct {
             uint32_t __RESERVED0[17];
        __IO uint32_t GPIOARST;                             /*!< GPIOA reset */
        __IO uint32_t GPIOBRST;                             /*!< GPIOB reset */
        __IO uint32_t GPIOCRST;                             /*!< GPIOC reset */
        __IO uint32_t GPIODRST;                             /*!< GPIOD reset */
             uint32_t __RESERVED1;
        __IO uint32_t GPIOFRST;                             /*!< GPIOF reset */
             uint32_t __RESERVED2;
        __IO uint32_t TSCRST;                               /*!< TSC reset */
             uint32_t __RESERVED3[3];
        __IO uint32_t ADC1RST;                              /*!< ADC1 reset */
             uint32_t __RESERVED4[3];
    } AHBRSTR;                                              /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
    struct {
        __IO uint32_t PREDIV[4];                            /*!< PREDIV[3:0] bits */
        __IO uint32_t ADC1PRES[5];                          /*!< ADC1PRES[8:4] bits */
             uint32_t __RESERVED0[23];
    } CFGR2;                                                /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
    struct {
        __IO uint32_t USART1SW[2];                          /*!< USART1SW[1:0] bits */
             uint32_t __RESERVED0[2];
        __IO uint32_t I2C1SW;                               /*!< I2C1SW bits */
        __IO uint32_t I2C2SW;                               /*!< I2C2SW bits */
        __IO uint32_t I2C3SW;                               /*!< I2C3SW bits */
             uint32_t __RESERVED1;
        __IO uint32_t TIM1SW;                               /*!< TIM1SW bits */
             uint32_t __RESERVED2;
        __IO uint32_t TIM15SW;                              /*!< TIM15SW bits */
        __IO uint32_t TIM16SW;                              /*!< TIM16SW bits */
             uint32_t __RESERVED3;
        __IO uint32_t TIM17SW;                              /*!< TIM17SW bits */
             uint32_t __RESERVED4[2];
        __IO uint32_t USART2SW[2];                          /*!< USART2SW[1:0] bits */
        __IO uint32_t USART3SW[2];                          /*!< USART3SW[1:0] bits */
             uint32_t __RESERVED5[12];
    } CFGR3;                                                /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
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
                 uint32_t __RESERVED0 : 1;
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
                 uint32_t __RESERVED1 : 8;
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
         uint32_t __RESERVED0;
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
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t PC13VALUE : 1;
            __IO uint32_t PC13MODE : 1;
            __IO uint32_t PC14VALUE : 1;
            __IO uint32_t PC14MODE : 1;
            __IO uint32_t PC15VALUE : 1;
            __IO uint32_t PC15MODE : 1;
                 uint32_t __RESERVED2 : 8;
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
         uint32_t __RESERVED1;
    __IO uint32_t BKPR[16];                                 /*!< RTC backup registers,                                     Address offset: 0x50-0x8C */
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
             uint32_t __RESERVED0;
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
             uint32_t __RESERVED1[8];
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
         uint32_t __RESERVED0[32];
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
             uint32_t __RESERVED1[2];
        __IO uint32_t PC13VALUE;
        __IO uint32_t PC13MODE;
        __IO uint32_t PC14VALUE;
        __IO uint32_t PC14MODE;
        __IO uint32_t PC15VALUE;
        __IO uint32_t PC15MODE;
             uint32_t __RESERVED2[8];
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
         uint32_t __RESERVED1[32];
    __IO uint32_t BKPR[16][32];                             /*!< RTC backup registers,                                     Address offset: 0x50-0x8C */
} RTC_BitBand_TypeDef;




/**
  * @brief Serial Peripheral Interface
  */
typedef struct {
    union {
        struct {
            __IO uint32_t CPHA : 1;                         /*!< Clock Phase */
            __IO uint32_t CPOL : 1;                         /*!< Clock Polarity */
            __IO uint32_t MSTR : 1;                         /*!< Master Selection */
            __IO uint32_t BR : 3;                           /*!< BR[2:0] bits (Baud Rate Control) */
            __IO uint32_t SPE : 1;                          /*!< SPI Enable */
            __IO uint32_t LSBFIRST : 1;                     /*!< Frame Format */
            __IO uint32_t SSI : 1;                          /*!< Internal slave select */
            __IO uint32_t SSM : 1;                          /*!< Software slave management */
            __IO uint32_t RXONLY : 1;                       /*!< Receive only */
            __IO uint32_t CRCL : 1;                         /*!< CRC Length */
            __IO uint32_t CRCNEXT : 1;                      /*!< Transmit CRC next */
            __IO uint32_t CRCEN : 1;                        /*!< Hardware CRC calculation enable */
            __IO uint32_t BIDIOE : 1;                       /*!< Output enable in bidirectional mode */
            __IO uint32_t BIDIMODE : 1;                     /*!< Bidirectional data mode enable */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< SPI Control register 1,                              Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t RXDMAEN : 1;                      /*!< Rx Buffer DMA Enable */
            __IO uint32_t TXDMAEN : 1;                      /*!< Tx Buffer DMA Enable */
            __IO uint32_t SSOE : 1;                         /*!< SS Output Enable */
            __IO uint32_t NSSP : 1;                         /*!< NSS pulse management Enable */
            __IO uint32_t FRF : 1;                          /*!< Frame Format Enable */
            __IO uint32_t ERRIE : 1;                        /*!< Error Interrupt Enable */
            __IO uint32_t RXNEIE : 1;                       /*!< RX buffer Not Empty Interrupt Enable */
            __IO uint32_t TXEIE : 1;                        /*!< Tx buffer Empty Interrupt Enable */
            __IO uint32_t DS : 4;                           /*!< DS[3:0] Data Size */
            __IO uint32_t FRXTH : 1;                        /*!< FIFO reception Threshold */
            __IO uint32_t LDMARX : 1;                       /*!< Last DMA transfer for reception */
            __IO uint32_t LDMATX : 1;                       /*!< Last DMA transfer for transmission */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< SPI Control register 2,                              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t RXNE : 1;                         /*!< Receive buffer Not Empty */
            __IO uint32_t TXE : 1;                          /*!< Transmit buffer Empty */
            __IO uint32_t CHSIDE : 1;                       /*!< Channel side */
            __IO uint32_t UDR : 1;                          /*!< Underrun flag */
            __IO uint32_t CRCERR : 1;                       /*!< CRC Error flag */
            __IO uint32_t MODF : 1;                         /*!< Mode fault */
            __IO uint32_t OVR : 1;                          /*!< Overrun flag */
            __IO uint32_t BSY : 1;                          /*!< Busy flag */
            __IO uint32_t FRE : 1;                          /*!< TI frame format error */
            __IO uint32_t FRLVL : 2;                        /*!< FIFO Reception Level */
            __IO uint32_t FTLVL : 2;                        /*!< FIFO Transmission Level */
                 uint32_t __RESERVED0 : 19;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< SPI Status register,                                 Address offset: 0x08 */
    __IO uint32_t DR;                                       /*!< SPI data register,                                   Address offset: 0x0C */
    __IO uint32_t CRCPR;                                    /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
    __IO uint32_t RXCRCR;                                   /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
    __IO uint32_t TXCRCR;                                   /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t CHLEN : 1;                        /*!<Channel length (number of bits per audio channel) */
            __IO uint32_t DATLEN : 2;                       /*!<DATLEN[1:0] bits (Data length to be transferred) */
            __IO uint32_t CKPOL : 1;                        /*!<steady state clock polarity */
            __IO uint32_t I2SSTD : 2;                       /*!<I2SSTD[1:0] bits (I2S standard selection) */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PCMSYNC : 1;                      /*!<PCM frame synchronization */
            __IO uint32_t I2SCFG : 2;                       /*!<I2SCFG[1:0] bits (I2S configuration mode) */
            __IO uint32_t I2SE : 1;                         /*!<I2S Enable */
            __IO uint32_t I2SMOD : 1;                       /*!<I2S mode selection */
                 uint32_t __RESERVED1 : 20;
        } b;
        __IO uint32_t w;
    } I2SCFGR;                                              /*!< SPI_I2S configuration register,                      Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t I2SDIV : 8;                       /*!<I2S Linear prescaler */
            __IO uint32_t ODD : 1;                          /*!<Odd factor for the prescaler */
            __IO uint32_t MCKOE : 1;                        /*!<Master Clock Output Enable */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } I2SPR;                                                /*!< SPI_I2S prescaler register,                          Address offset: 0x20 */
} SPI_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CPHA;                                 /*!< Clock Phase */
        __IO uint32_t CPOL;                                 /*!< Clock Polarity */
        __IO uint32_t MSTR;                                 /*!< Master Selection */
        __IO uint32_t BR[3];                                /*!< BR[2:0] bits (Baud Rate Control) */
        __IO uint32_t SPE;                                  /*!< SPI Enable */
        __IO uint32_t LSBFIRST;                             /*!< Frame Format */
        __IO uint32_t SSI;                                  /*!< Internal slave select */
        __IO uint32_t SSM;                                  /*!< Software slave management */
        __IO uint32_t RXONLY;                               /*!< Receive only */
        __IO uint32_t CRCL;                                 /*!< CRC Length */
        __IO uint32_t CRCNEXT;                              /*!< Transmit CRC next */
        __IO uint32_t CRCEN;                                /*!< Hardware CRC calculation enable */
        __IO uint32_t BIDIOE;                               /*!< Output enable in bidirectional mode */
        __IO uint32_t BIDIMODE;                             /*!< Bidirectional data mode enable */
             uint32_t __RESERVED0[16];
    } CR1;                                                  /*!< SPI Control register 1,                              Address offset: 0x00 */
    struct {
        __IO uint32_t RXDMAEN;                              /*!< Rx Buffer DMA Enable */
        __IO uint32_t TXDMAEN;                              /*!< Tx Buffer DMA Enable */
        __IO uint32_t SSOE;                                 /*!< SS Output Enable */
        __IO uint32_t NSSP;                                 /*!< NSS pulse management Enable */
        __IO uint32_t FRF;                                  /*!< Frame Format Enable */
        __IO uint32_t ERRIE;                                /*!< Error Interrupt Enable */
        __IO uint32_t RXNEIE;                               /*!< RX buffer Not Empty Interrupt Enable */
        __IO uint32_t TXEIE;                                /*!< Tx buffer Empty Interrupt Enable */
        __IO uint32_t DS[4];                                /*!< DS[3:0] Data Size */
        __IO uint32_t FRXTH;                                /*!< FIFO reception Threshold */
        __IO uint32_t LDMARX;                               /*!< Last DMA transfer for reception */
        __IO uint32_t LDMATX;                               /*!< Last DMA transfer for transmission */
             uint32_t __RESERVED0[17];
    } CR2;                                                  /*!< SPI Control register 2,                              Address offset: 0x04 */
    struct {
        __IO uint32_t RXNE;                                 /*!< Receive buffer Not Empty */
        __IO uint32_t TXE;                                  /*!< Transmit buffer Empty */
        __IO uint32_t CHSIDE;                               /*!< Channel side */
        __IO uint32_t UDR;                                  /*!< Underrun flag */
        __IO uint32_t CRCERR;                               /*!< CRC Error flag */
        __IO uint32_t MODF;                                 /*!< Mode fault */
        __IO uint32_t OVR;                                  /*!< Overrun flag */
        __IO uint32_t BSY;                                  /*!< Busy flag */
        __IO uint32_t FRE;                                  /*!< TI frame format error */
        __IO uint32_t FRLVL[2];                             /*!< FIFO Reception Level */
        __IO uint32_t FTLVL[2];                             /*!< FIFO Transmission Level */
             uint32_t __RESERVED0[19];
    } SR;                                                   /*!< SPI Status register,                                 Address offset: 0x08 */
    __IO uint32_t DR[32];                                   /*!< SPI data register,                                   Address offset: 0x0C */
    __IO uint32_t CRCPR[32];                                /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
    __IO uint32_t RXCRCR[32];                               /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
    __IO uint32_t TXCRCR[32];                               /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
    struct {
        __IO uint32_t CHLEN;                                /*!<Channel length (number of bits per audio channel) */
        __IO uint32_t DATLEN[2];                            /*!<DATLEN[1:0] bits (Data length to be transferred) */
        __IO uint32_t CKPOL;                                /*!<steady state clock polarity */
        __IO uint32_t I2SSTD[2];                            /*!<I2SSTD[1:0] bits (I2S standard selection) */
             uint32_t __RESERVED0;
        __IO uint32_t PCMSYNC;                              /*!<PCM frame synchronization */
        __IO uint32_t I2SCFG[2];                            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
        __IO uint32_t I2SE;                                 /*!<I2S Enable */
        __IO uint32_t I2SMOD;                               /*!<I2S mode selection */
             uint32_t __RESERVED1[20];
    } I2SCFGR;                                              /*!< SPI_I2S configuration register,                      Address offset: 0x1C */
    struct {
        __IO uint32_t I2SDIV[8];                            /*!<I2S Linear prescaler */
        __IO uint32_t ODD;                                  /*!<Odd factor for the prescaler */
        __IO uint32_t MCKOE;                                /*!<Master Clock Output Enable */
             uint32_t __RESERVED0[22];
    } I2SPR;                                                /*!< SPI_I2S prescaler register,                          Address offset: 0x20 */
} SPI_BitBand_TypeDef;



/**
  * @brief TIM
  */
typedef struct {
    union {
        struct {
            __IO uint32_t CEN : 1;                          /*!<Counter enable */
            __IO uint32_t UDIS : 1;                         /*!<Update disable */
            __IO uint32_t URS : 1;                          /*!<Update request source */
            __IO uint32_t OPM : 1;                          /*!<One pulse mode */
            __IO uint32_t DIR : 1;                          /*!<Direction */
            __IO uint32_t CMS : 2;                          /*!<CMS[1:0] bits (Center-aligned mode selection) */
            __IO uint32_t ARPE : 1;                         /*!<Auto-reload preload enable */
            __IO uint32_t CKD : 2;                          /*!<CKD[1:0] bits (clock division) */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t UIFREMAP : 1;                     /*!<Update interrupt flag remap */
                 uint32_t __RESERVED1 : 20;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< TIM control register 1,              Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CCPC : 1;                         /*!<Capture/Compare Preloaded Control */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CCUS : 1;                         /*!<Capture/Compare Control Update Selection */
            __IO uint32_t CCDS : 1;                         /*!<Capture/Compare DMA Selection */
            __IO uint32_t MMS : 3;                          /*!<MMS[2:0] bits (Master Mode Selection) */
            __IO uint32_t TI1S : 1;                         /*!<TI1 Selection */
            __IO uint32_t OIS1 : 1;                         /*!<Output Idle state 1 (OC1 output) */
            __IO uint32_t OIS1N : 1;                        /*!<Output Idle state 1 (OC1N output) */
            __IO uint32_t OIS2 : 1;                         /*!<Output Idle state 2 (OC2 output) */
            __IO uint32_t OIS2N : 1;                        /*!<Output Idle state 2 (OC2N output) */
            __IO uint32_t OIS3 : 1;                         /*!<Output Idle state 3 (OC3 output) */
            __IO uint32_t OIS3N : 1;                        /*!<Output Idle state 3 (OC3N output) */
            __IO uint32_t OIS4 : 1;                         /*!<Output Idle state 4 (OC4 output) */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t OIS5 : 1;                         /*!<Output Idle state 4 (OC4 output) */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t OIS6 : 1;                         /*!<Output Idle state 4 (OC4 output) */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t MMS2 : 4;                         /*!<MMS[2:0] bits (Master Mode Selection) */
                 uint32_t __RESERVED4 : 8;
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< TIM control register 2,              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t SMS : 3;                          /*!<SMS[2:0] bits (Slave mode selection) */
            __IO uint32_t OCCS : 1;                         /*!< OCREF clear selection */
            __IO uint32_t TS : 3;                           /*!<TS[2:0] bits (Trigger selection) */
            __IO uint32_t MSM : 1;                          /*!<Master/slave mode */
            __IO uint32_t ETF : 4;                          /*!<ETF[3:0] bits (External trigger filter) */
            __IO uint32_t ETPS : 2;                         /*!<ETPS[1:0] bits (External trigger prescaler) */
            __IO uint32_t ECE : 1;                          /*!<External clock enable */
            __IO uint32_t ETP : 1;                          /*!<External trigger polarity */
            __IO uint32_t SMS1 : 1;                         /*!<SMS[3] bit (Slave mode selection) */
                 uint32_t __RESERVED0 : 15;
        } b;
        __IO uint32_t w;
    } SMCR;                                                 /*!< TIM slave mode control register,     Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t UIE : 1;                          /*!<Update interrupt enable */
            __IO uint32_t CC1IE : 1;                        /*!<Capture/Compare 1 interrupt enable */
            __IO uint32_t CC2IE : 1;                        /*!<Capture/Compare 2 interrupt enable */
            __IO uint32_t CC3IE : 1;                        /*!<Capture/Compare 3 interrupt enable */
            __IO uint32_t CC4IE : 1;                        /*!<Capture/Compare 4 interrupt enable */
            __IO uint32_t COMIE : 1;                        /*!<COM interrupt enable */
            __IO uint32_t TIE : 1;                          /*!<Trigger interrupt enable */
            __IO uint32_t BIE : 1;                          /*!<Break interrupt enable */
            __IO uint32_t UDE : 1;                          /*!<Update DMA request enable */
            __IO uint32_t CC1DE : 1;                        /*!<Capture/Compare 1 DMA request enable */
            __IO uint32_t CC2DE : 1;                        /*!<Capture/Compare 2 DMA request enable */
            __IO uint32_t CC3DE : 1;                        /*!<Capture/Compare 3 DMA request enable */
            __IO uint32_t CC4DE : 1;                        /*!<Capture/Compare 4 DMA request enable */
            __IO uint32_t COMDE : 1;                        /*!<COM DMA request enable */
            __IO uint32_t TDE : 1;                          /*!<Trigger DMA request enable */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } DIER;                                                 /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t UIF : 1;                          /*!<Update interrupt Flag */
            __IO uint32_t CC1IF : 1;                        /*!<Capture/Compare 1 interrupt Flag */
            __IO uint32_t CC2IF : 1;                        /*!<Capture/Compare 2 interrupt Flag */
            __IO uint32_t CC3IF : 1;                        /*!<Capture/Compare 3 interrupt Flag */
            __IO uint32_t CC4IF : 1;                        /*!<Capture/Compare 4 interrupt Flag */
            __IO uint32_t COMIF : 1;                        /*!<COM interrupt Flag */
            __IO uint32_t TIF : 1;                          /*!<Trigger interrupt Flag */
            __IO uint32_t BIF : 1;                          /*!<Break interrupt Flag */
            __IO uint32_t B2IF : 1;                         /*!<Break2 interrupt Flag */
            __IO uint32_t CC1OF : 1;                        /*!<Capture/Compare 1 Overcapture Flag */
            __IO uint32_t CC2OF : 1;                        /*!<Capture/Compare 2 Overcapture Flag */
            __IO uint32_t CC3OF : 1;                        /*!<Capture/Compare 3 Overcapture Flag */
            __IO uint32_t CC4OF : 1;                        /*!<Capture/Compare 4 Overcapture Flag */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t CC5IF : 1;                        /*!<Capture/Compare 5 interrupt Flag */
            __IO uint32_t CC6IF : 1;                        /*!<Capture/Compare 6 interrupt Flag */
                 uint32_t __RESERVED1 : 14;
        } b;
        __IO uint32_t w;
    } SR;                                                   /*!< TIM status register,                 Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t UG : 1;                           /*!<Update Generation */
            __IO uint32_t CC1G : 1;                         /*!<Capture/Compare 1 Generation */
            __IO uint32_t CC2G : 1;                         /*!<Capture/Compare 2 Generation */
            __IO uint32_t CC3G : 1;                         /*!<Capture/Compare 3 Generation */
            __IO uint32_t CC4G : 1;                         /*!<Capture/Compare 4 Generation */
            __IO uint32_t COMG : 1;                         /*!<Capture/Compare Control Update Generation */
            __IO uint32_t TG : 1;                           /*!<Trigger Generation */
            __IO uint32_t BG : 1;                           /*!<Break Generation */
            __IO uint32_t B2G : 1;                          /*!<Break Generation */
                 uint32_t __RESERVED0 : 23;
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
            __IO uint32_t C1M1 : 1;                         /*!<OC1M[3] bits (Output Compare 1 Mode) */
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t C2M1 : 1;                         /*!<OC2M[3] bits (Output Compare 2 Mode) */
                 uint32_t __RESERVED1 : 7;
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
            __IO uint32_t C3S : 2;                          /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
            __IO uint32_t C3FE : 1;                         /*!<Output Compare 3 Fast enable */
            __IO uint32_t C3PE : 1;                         /*!<Output Compare 3 Preload enable */
            __IO uint32_t C3M : 3;                          /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
            __IO uint32_t C3CE : 1;                         /*!<Output Compare 3 Clear Enable */
            __IO uint32_t C4S : 2;                          /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
            __IO uint32_t C4FE : 1;                         /*!<Output Compare 4 Fast enable */
            __IO uint32_t C4PE : 1;                         /*!<Output Compare 4 Preload enable */
            __IO uint32_t C4M : 3;                          /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
            __IO uint32_t C4CE : 1;                         /*!<Output Compare 4 Clear Enable */
            __IO uint32_t C3M1 : 1;                         /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
                 uint32_t __RESERVED0 : 7;
            __IO uint32_t C4M1 : 1;                         /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
                 uint32_t __RESERVED1 : 7;
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
            __IO uint32_t CC1E : 1;                         /*!<Capture/Compare 1 output enable */
            __IO uint32_t CC1P : 1;                         /*!<Capture/Compare 1 output Polarity */
            __IO uint32_t CC1NE : 1;                        /*!<Capture/Compare 1 Complementary output enable */
            __IO uint32_t CC1NP : 1;                        /*!<Capture/Compare 1 Complementary output Polarity */
            __IO uint32_t CC2E : 1;                         /*!<Capture/Compare 2 output enable */
            __IO uint32_t CC2P : 1;                         /*!<Capture/Compare 2 output Polarity */
            __IO uint32_t CC2NE : 1;                        /*!<Capture/Compare 2 Complementary output enable */
            __IO uint32_t CC2NP : 1;                        /*!<Capture/Compare 2 Complementary output Polarity */
            __IO uint32_t CC3E : 1;                         /*!<Capture/Compare 3 output enable */
            __IO uint32_t CC3P : 1;                         /*!<Capture/Compare 3 output Polarity */
            __IO uint32_t CC3NE : 1;                        /*!<Capture/Compare 3 Complementary output enable */
            __IO uint32_t CC3NP : 1;                        /*!<Capture/Compare 3 Complementary output Polarity */
            __IO uint32_t CC4E : 1;                         /*!<Capture/Compare 4 output enable */
            __IO uint32_t CC4P : 1;                         /*!<Capture/Compare 4 output Polarity */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CC4NP : 1;                        /*!<Capture/Compare 4 Complementary output Polarity */
            __IO uint32_t CC5E : 1;                         /*!<Capture/Compare 5 output enable */
            __IO uint32_t CC5P : 1;                         /*!<Capture/Compare 5 output Polarity */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t CC6E : 1;                         /*!<Capture/Compare 6 output enable */
            __IO uint32_t CC6P : 1;                         /*!<Capture/Compare 6 output Polarity */
                 uint32_t __RESERVED2 : 10;
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
            __IO uint32_t OSSR : 1;                         /*!<Off-State Selection for Run mode */
            __IO uint32_t BKE : 1;                          /*!<Break enable for Break1 */
            __IO uint32_t BKP : 1;                          /*!<Break Polarity for Break1 */
            __IO uint32_t AOE : 1;                          /*!<Automatic Output enable */
            __IO uint32_t MOE : 1;                          /*!<Main Output enable */
            __IO uint32_t BKF : 4;                          /*!<Break Filter for Break1 */
            __IO uint32_t BK2F : 4;                         /*!<Break Filter for Break2 */
            __IO uint32_t BK2E : 1;                         /*!<Break enable for Break2 */
            __IO uint32_t BK2P : 1;                         /*!<Break Polarity for Break2 */
                 uint32_t __RESERVED0 : 6;
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
    union {
        struct {
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t OC5FE : 1;                        /*!<Output Compare 5 Fast enable */
            __IO uint32_t OC5PE : 1;                        /*!<Output Compare 5 Preload enable */
            __IO uint32_t OC5M : 3;                         /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
            __IO uint32_t OC5CE : 1;                        /*!<Output Compare 5 Clear Enable */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t OC6FE : 1;                        /*!<Output Compare 6 Fast enable */
            __IO uint32_t OC6PE : 1;                        /*!<Output Compare 6 Preload enable */
            __IO uint32_t OC6M : 3;                         /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
            __IO uint32_t OC6CE : 1;                        /*!<Output Compare 6 Clear Enable */
            __IO uint32_t OC5M1 : 1;                        /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
                 uint32_t __RESERVED2 : 7;
            __IO uint32_t OC6M1 : 1;                        /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
                 uint32_t __RESERVED3 : 7;
        } b;
        __IO uint32_t w;
    } CCMR3;                                                /*!< TIM capture/compare mode register 3, Address offset: 0x54 */
    __IO uint32_t CCR5;                                     /*!< TIM capture/compare register 5,      Address offset: 0x58 */
    __IO uint32_t CCR6;                                     /*!< TIM capture/compare register 6,      Address offset: 0x5C */
} TIM_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CEN;                                  /*!<Counter enable */
        __IO uint32_t UDIS;                                 /*!<Update disable */
        __IO uint32_t URS;                                  /*!<Update request source */
        __IO uint32_t OPM;                                  /*!<One pulse mode */
        __IO uint32_t DIR;                                  /*!<Direction */
        __IO uint32_t CMS[2];                               /*!<CMS[1:0] bits (Center-aligned mode selection) */
        __IO uint32_t ARPE;                                 /*!<Auto-reload preload enable */
        __IO uint32_t CKD[2];                               /*!<CKD[1:0] bits (clock division) */
             uint32_t __RESERVED0;
        __IO uint32_t UIFREMAP;                             /*!<Update interrupt flag remap */
             uint32_t __RESERVED1[20];
    } CR1;                                                  /*!< TIM control register 1,              Address offset: 0x00 */
    struct {
        __IO uint32_t CCPC;                                 /*!<Capture/Compare Preloaded Control */
             uint32_t __RESERVED0;
        __IO uint32_t CCUS;                                 /*!<Capture/Compare Control Update Selection */
        __IO uint32_t CCDS;                                 /*!<Capture/Compare DMA Selection */
        __IO uint32_t MMS[3];                               /*!<MMS[2:0] bits (Master Mode Selection) */
        __IO uint32_t TI1S;                                 /*!<TI1 Selection */
        __IO uint32_t OIS1;                                 /*!<Output Idle state 1 (OC1 output) */
        __IO uint32_t OIS1N;                                /*!<Output Idle state 1 (OC1N output) */
        __IO uint32_t OIS2;                                 /*!<Output Idle state 2 (OC2 output) */
        __IO uint32_t OIS2N;                                /*!<Output Idle state 2 (OC2N output) */
        __IO uint32_t OIS3;                                 /*!<Output Idle state 3 (OC3 output) */
        __IO uint32_t OIS3N;                                /*!<Output Idle state 3 (OC3N output) */
        __IO uint32_t OIS4;                                 /*!<Output Idle state 4 (OC4 output) */
             uint32_t __RESERVED1;
        __IO uint32_t OIS5;                                 /*!<Output Idle state 4 (OC4 output) */
             uint32_t __RESERVED2;
        __IO uint32_t OIS6;                                 /*!<Output Idle state 4 (OC4 output) */
             uint32_t __RESERVED3;
        __IO uint32_t MMS2[4];                              /*!<MMS[2:0] bits (Master Mode Selection) */
             uint32_t __RESERVED4[8];
    } CR2;                                                  /*!< TIM control register 2,              Address offset: 0x04 */
    struct {
        __IO uint32_t SMS[3];                               /*!<SMS[2:0] bits (Slave mode selection) */
        __IO uint32_t OCCS;                                 /*!< OCREF clear selection */
        __IO uint32_t TS[3];                                /*!<TS[2:0] bits (Trigger selection) */
        __IO uint32_t MSM;                                  /*!<Master/slave mode */
        __IO uint32_t ETF[4];                               /*!<ETF[3:0] bits (External trigger filter) */
        __IO uint32_t ETPS[2];                              /*!<ETPS[1:0] bits (External trigger prescaler) */
        __IO uint32_t ECE;                                  /*!<External clock enable */
        __IO uint32_t ETP;                                  /*!<External trigger polarity */
        __IO uint32_t SMS1;                                 /*!<SMS[3] bits (Slave mode selection) */
             uint32_t __RESERVED0[15];
    } SMCR;                                                 /*!< TIM slave mode control register,     Address offset: 0x08 */
    struct {
        __IO uint32_t UIE;                                  /*!<Update interrupt enable */
        __IO uint32_t CC1IE;                                /*!<Capture/Compare 1 interrupt enable */
        __IO uint32_t CC2IE;                                /*!<Capture/Compare 2 interrupt enable */
        __IO uint32_t CC3IE;                                /*!<Capture/Compare 3 interrupt enable */
        __IO uint32_t CC4IE;                                /*!<Capture/Compare 4 interrupt enable */
        __IO uint32_t COMIE;                                /*!<COM interrupt enable */
        __IO uint32_t TIE;                                  /*!<Trigger interrupt enable */
        __IO uint32_t BIE;                                  /*!<Break interrupt enable */
        __IO uint32_t UDE;                                  /*!<Update DMA request enable */
        __IO uint32_t CC1DE;                                /*!<Capture/Compare 1 DMA request enable */
        __IO uint32_t CC2DE;                                /*!<Capture/Compare 2 DMA request enable */
        __IO uint32_t CC3DE;                                /*!<Capture/Compare 3 DMA request enable */
        __IO uint32_t CC4DE;                                /*!<Capture/Compare 4 DMA request enable */
        __IO uint32_t COMDE;                                /*!<COM DMA request enable */
        __IO uint32_t TDE;                                  /*!<Trigger DMA request enable */
             uint32_t __RESERVED0[17];
    } DIER;                                                 /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    struct {
        __IO uint32_t UIF;                                  /*!<Update interrupt Flag */
        __IO uint32_t CC1IF;                                /*!<Capture/Compare 1 interrupt Flag */
        __IO uint32_t CC2IF;                                /*!<Capture/Compare 2 interrupt Flag */
        __IO uint32_t CC3IF;                                /*!<Capture/Compare 3 interrupt Flag */
        __IO uint32_t CC4IF;                                /*!<Capture/Compare 4 interrupt Flag */
        __IO uint32_t COMIF;                                /*!<COM interrupt Flag */
        __IO uint32_t TIF;                                  /*!<Trigger interrupt Flag */
        __IO uint32_t BIF;                                  /*!<Break interrupt Flag */
        __IO uint32_t B2IF;                                 /*!<Break2 interrupt Flag */
        __IO uint32_t CC1OF;                                /*!<Capture/Compare 1 Overcapture Flag */
        __IO uint32_t CC2OF;                                /*!<Capture/Compare 2 Overcapture Flag */
        __IO uint32_t CC3OF;                                /*!<Capture/Compare 3 Overcapture Flag */
        __IO uint32_t CC4OF;                                /*!<Capture/Compare 4 Overcapture Flag */
             uint32_t __RESERVED0[3];
        __IO uint32_t CC5IF;                                /*!<Capture/Compare 5 interrupt Flag */
        __IO uint32_t CC6IF;                                /*!<Capture/Compare 6 interrupt Flag */
             uint32_t __RESERVED1[14];
    } SR;                                                   /*!< TIM status register,                 Address offset: 0x10 */
    struct {
        __IO uint32_t UG;                                   /*!<Update Generation */
        __IO uint32_t CC1G;                                 /*!<Capture/Compare 1 Generation */
        __IO uint32_t CC2G;                                 /*!<Capture/Compare 2 Generation */
        __IO uint32_t CC3G;                                 /*!<Capture/Compare 3 Generation */
        __IO uint32_t CC4G;                                 /*!<Capture/Compare 4 Generation */
        __IO uint32_t COMG;                                 /*!<Capture/Compare Control Update Generation */
        __IO uint32_t TG;                                   /*!<Trigger Generation */
        __IO uint32_t BG;                                   /*!<Break Generation */
        __IO uint32_t B2G;                                  /*!<Break Generation */
             uint32_t __RESERVED0[23];
    } EGR;                                                  /*!< TIM event generation register,       Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t C1S[2];                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
            __IO uint32_t C1FE;                             /*!<Output Compare 1 Fast enable */
            __IO uint32_t C1PE;                             /*!<Output Compare 1 Preload enable */
            __IO uint32_t C1M[3];                           /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
            __IO uint32_t C1CE;                             /*!<Output Compare 1Clear Enable */
            __IO uint32_t C2S[2];                           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
            __IO uint32_t C2FE;                             /*!<Output Compare 2 Fast enable */
            __IO uint32_t C2PE;                             /*!<Output Compare 2 Preload enable */
            __IO uint32_t C2M[3];                           /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
            __IO uint32_t C2CE;                             /*!<Output Compare 2 Clear Enable */
            __IO uint32_t C1M1;                             /*!<OC1M[3] bits (Output Compare 1 Mode) */
                 uint32_t __RESERVED0[7];
            __IO uint32_t C2M1;                             /*!<OC2M[3] bits (Output Compare 2 Mode) */
                 uint32_t __RESERVED1[7];
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
            __IO uint32_t C3M1;                             /*!<OC3M[3] bits (Output Compare 3 Mode) */
                 uint32_t __RESERVED0[7];
            __IO uint32_t C4M1;                             /*!<OC4M[3] bits (Output Compare 4 Mode) */
                 uint32_t __RESERVED1[7];
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
        __IO uint32_t CC1E;                                 /*!<Capture/Compare 1 output enable */
        __IO uint32_t CC1P;                                 /*!<Capture/Compare 1 output Polarity */
        __IO uint32_t CC1NE;                                /*!<Capture/Compare 1 Complementary output enable */
        __IO uint32_t CC1NP;                                /*!<Capture/Compare 1 Complementary output Polarity */
        __IO uint32_t CC2E;                                 /*!<Capture/Compare 2 output enable */
        __IO uint32_t CC2P;                                 /*!<Capture/Compare 2 output Polarity */
        __IO uint32_t CC2NE;                                /*!<Capture/Compare 2 Complementary output enable */
        __IO uint32_t CC2NP;                                /*!<Capture/Compare 2 Complementary output Polarity */
        __IO uint32_t CC3E;                                 /*!<Capture/Compare 3 output enable */
        __IO uint32_t CC3P;                                 /*!<Capture/Compare 3 output Polarity */
        __IO uint32_t CC3NE;                                /*!<Capture/Compare 3 Complementary output enable */
        __IO uint32_t CC3NP;                                /*!<Capture/Compare 3 Complementary output Polarity */
        __IO uint32_t CC4E;                                 /*!<Capture/Compare 4 output enable */
        __IO uint32_t CC4P;                                 /*!<Capture/Compare 4 output Polarity */
             uint32_t __RESERVED0;
        __IO uint32_t CC4NP;                                /*!<Capture/Compare 4 Complementary output Polarity */
        __IO uint32_t CC5E;                                 /*!<Capture/Compare 5 output enable */
        __IO uint32_t CC5P;                                 /*!<Capture/Compare 5 output Polarity */
             uint32_t __RESERVED1[2];
        __IO uint32_t CC6E;                                 /*!<Capture/Compare 6 output enable */
        __IO uint32_t CC6P;                                 /*!<Capture/Compare 6 output Polarity */
             uint32_t __RESERVED2[10];
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
        __IO uint32_t OSSR;                                 /*!<Off-State Selection for Run mode */
        __IO uint32_t BKE;                                  /*!<Break enable for Break1 */
        __IO uint32_t BKP;                                  /*!<Break Polarity for Break1 */
        __IO uint32_t AOE;                                  /*!<Automatic Output enable */
        __IO uint32_t MOE;                                  /*!<Main Output enable */
        __IO uint32_t BKF[4];                               /*!<Break Filter for Break1 */
        __IO uint32_t BK2F[4];                              /*!<Break Filter for Break2 */
        __IO uint32_t BK2E;                                 /*!<Break enable for Break2 */
        __IO uint32_t BK2P;                                 /*!<Break Polarity for Break2 */
             uint32_t __RESERVED0[6];
    } BDTR;                                                 /*!< TIM break and dead-time register,    Address offset: 0x44 */
    struct {
        __IO uint32_t DBA[5];                               /*!<DBA[4:0] bits (DMA Base Address) */
             uint32_t __RESERVED0[3];
        __IO uint32_t DBL[5];                               /*!<DBL[4:0] bits (DMA Burst Length) */
             uint32_t __RESERVED1[19];
    } DCR;                                                  /*!< TIM DMA control register,            Address offset: 0x48 */
    __IO uint32_t DMAR[32];                                 /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    __IO uint32_t OR[32];                                   /*!< TIM option register,                 Address offset: 0x50 */
    struct {
             uint32_t __RESERVED0[2];
        __IO uint32_t OC5FE;                                /*!<Output Compare 5 Fast enable */
        __IO uint32_t OC5PE;                                /*!<Output Compare 5 Preload enable */
        __IO uint32_t OC5M[3];                              /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
        __IO uint32_t OC5CE;                                /*!<Output Compare 5 Clear Enable */
             uint32_t __RESERVED1[2];
        __IO uint32_t OC6FE;                                /*!<Output Compare 6 Fast enable */
        __IO uint32_t OC6PE;                                /*!<Output Compare 6 Preload enable */
        __IO uint32_t OC6M[3];                              /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
        __IO uint32_t OC6CE;                                /*!<Output Compare 6 Clear Enable */
        __IO uint32_t OC5M1;                                /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
             uint32_t __RESERVED2[7];
        __IO uint32_t OC6M1;                                /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
             uint32_t __RESERVED3[7];
    } CCMR3;                                                /*!< TIM capture/compare mode register 3, Address offset: 0x54 */
    struct {
        __IO uint32_t CCR5[16];                             /*!<Capture/Compare 5 Value */
             uint32_t __RESERVED0[13];
        __IO uint32_t GC5C1;                                /*!<Group Channel 5 and Channel 1 */
        __IO uint32_t GC5C2;                                /*!<Group Channel 5 and Channel 2 */
        __IO uint32_t GC5C3;                                /*!<Group Channel 5 and Channel 3 */
    } CCR5;                                                 /*!< TIM capture/compare register5,       Address offset: 0x58 */
    __IO uint32_t CCR6[32];                                 /*!< TIM capture/compare register 4,      Address offset: 0x5C */
} TIM_BitBand_TypeDef;



/**
  * @brief Touch Sensing Controller (TSC)
  */
typedef struct {
    union {
        struct {
            __IO uint32_t TSCE : 1;                         /*!<Touch sensing controller enable */
            __IO uint32_t START : 1;                        /*!<Start acquisition */
            __IO uint32_t AM : 1;                           /*!<Acquisition mode */
            __IO uint32_t SYNCPOL : 1;                      /*!<Synchronization pin polarity */
            __IO uint32_t IODEF : 1;                        /*!<IO default mode */
            __IO uint32_t MCV : 3;                          /*!<MCV[2:0] bits (Max Count Value) */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t PGPSC : 3;                        /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
            __IO uint32_t SSPSC : 1;                        /*!<Spread Spectrum Prescaler */
            __IO uint32_t SSE : 1;                          /*!<Spread Spectrum Enable */
            __IO uint32_t SSD : 7;                          /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
            __IO uint32_t CTPL : 4;                         /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
            __IO uint32_t CTPH : 4;                         /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< TSC control register,                                     Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t EOAIE : 1;                        /*!<End of acquisition interrupt enable */
            __IO uint32_t MCEIE : 1;                        /*!<Max count error interrupt enable */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } IER;                                                  /*!< TSC interrupt enable register,                            Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t EOAIC : 1;                        /*!<End of acquisition interrupt clear */
            __IO uint32_t MCEIC : 1;                        /*!<Max count error interrupt clear */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } ICR;                                                  /*!< TSC interrupt clear register,                             Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t EOAF : 1;                         /*!<End of acquisition flag */
            __IO uint32_t MCEF : 1;                         /*!<Max count error flag */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } ISR;                                                  /*!< TSC interrupt status register,                            Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                       /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G1_IO2 : 1;                       /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G1_IO3 : 1;                       /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G1_IO4 : 1;                       /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO1 : 1;                       /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO2 : 1;                       /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO3 : 1;                       /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO4 : 1;                       /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO1 : 1;                       /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO2 : 1;                       /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO3 : 1;                       /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO4 : 1;                       /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO1 : 1;                       /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO2 : 1;                       /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO3 : 1;                       /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO4 : 1;                       /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO1 : 1;                       /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO2 : 1;                       /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO3 : 1;                       /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO4 : 1;                       /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO1 : 1;                       /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO2 : 1;                       /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO3 : 1;                       /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO4 : 1;                       /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO1 : 1;                       /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO2 : 1;                       /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO3 : 1;                       /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO4 : 1;                       /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO1 : 1;                       /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO2 : 1;                       /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO3 : 1;                       /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO4 : 1;                       /*!<GROUP8_IO4 schmitt trigger hysteresis mode */
        } b;
        __IO uint32_t w;
    } IOHCR;                                                /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
         uint32_t __RESERVED0;
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                       /*!<GROUP1_IO1 analog switch enable */
            __IO uint32_t G1_IO2 : 1;                       /*!<GROUP1_IO2 analog switch enable */
            __IO uint32_t G1_IO3 : 1;                       /*!<GROUP1_IO3 analog switch enable */
            __IO uint32_t G1_IO4 : 1;                       /*!<GROUP1_IO4 analog switch enable */
            __IO uint32_t G2_IO1 : 1;                       /*!<GROUP2_IO1 analog switch enable */
            __IO uint32_t G2_IO2 : 1;                       /*!<GROUP2_IO2 analog switch enable */
            __IO uint32_t G2_IO3 : 1;                       /*!<GROUP2_IO3 analog switch enable */
            __IO uint32_t G2_IO4 : 1;                       /*!<GROUP2_IO4 analog switch enable */
            __IO uint32_t G3_IO1 : 1;                       /*!<GROUP3_IO1 analog switch enable */
            __IO uint32_t G3_IO2 : 1;                       /*!<GROUP3_IO2 analog switch enable */
            __IO uint32_t G3_IO3 : 1;                       /*!<GROUP3_IO3 analog switch enable */
            __IO uint32_t G3_IO4 : 1;                       /*!<GROUP3_IO4 analog switch enable */
            __IO uint32_t G4_IO1 : 1;                       /*!<GROUP4_IO1 analog switch enable */
            __IO uint32_t G4_IO2 : 1;                       /*!<GROUP4_IO2 analog switch enable */
            __IO uint32_t G4_IO3 : 1;                       /*!<GROUP4_IO3 analog switch enable */
            __IO uint32_t G4_IO4 : 1;                       /*!<GROUP4_IO4 analog switch enable */
            __IO uint32_t G5_IO1 : 1;                       /*!<GROUP5_IO1 analog switch enable */
            __IO uint32_t G5_IO2 : 1;                       /*!<GROUP5_IO2 analog switch enable */
            __IO uint32_t G5_IO3 : 1;                       /*!<GROUP5_IO3 analog switch enable */
            __IO uint32_t G5_IO4 : 1;                       /*!<GROUP5_IO4 analog switch enable */
            __IO uint32_t G6_IO1 : 1;                       /*!<GROUP6_IO1 analog switch enable */
            __IO uint32_t G6_IO2 : 1;                       /*!<GROUP6_IO2 analog switch enable */
            __IO uint32_t G6_IO3 : 1;                       /*!<GROUP6_IO3 analog switch enable */
            __IO uint32_t G6_IO4 : 1;                       /*!<GROUP6_IO4 analog switch enable */
            __IO uint32_t G7_IO1 : 1;                       /*!<GROUP7_IO1 analog switch enable */
            __IO uint32_t G7_IO2 : 1;                       /*!<GROUP7_IO2 analog switch enable */
            __IO uint32_t G7_IO3 : 1;                       /*!<GROUP7_IO3 analog switch enable */
            __IO uint32_t G7_IO4 : 1;                       /*!<GROUP7_IO4 analog switch enable */
            __IO uint32_t G8_IO1 : 1;                       /*!<GROUP8_IO1 analog switch enable */
            __IO uint32_t G8_IO2 : 1;                       /*!<GROUP8_IO2 analog switch enable */
            __IO uint32_t G8_IO3 : 1;                       /*!<GROUP8_IO3 analog switch enable */
            __IO uint32_t G8_IO4 : 1;                       /*!<GROUP8_IO4 analog switch enable */
        } b;
        __IO uint32_t w;
    } IOASCR;                                               /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
         uint32_t __RESERVED1;
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                       /*!<GROUP1_IO1 sampling mode */
            __IO uint32_t G1_IO2 : 1;                       /*!<GROUP1_IO2 sampling mode */
            __IO uint32_t G1_IO3 : 1;                       /*!<GROUP1_IO3 sampling mode */
            __IO uint32_t G1_IO4 : 1;                       /*!<GROUP1_IO4 sampling mode */
            __IO uint32_t G2_IO1 : 1;                       /*!<GROUP2_IO1 sampling mode */
            __IO uint32_t G2_IO2 : 1;                       /*!<GROUP2_IO2 sampling mode */
            __IO uint32_t G2_IO3 : 1;                       /*!<GROUP2_IO3 sampling mode */
            __IO uint32_t G2_IO4 : 1;                       /*!<GROUP2_IO4 sampling mode */
            __IO uint32_t G3_IO1 : 1;                       /*!<GROUP3_IO1 sampling mode */
            __IO uint32_t G3_IO2 : 1;                       /*!<GROUP3_IO2 sampling mode */
            __IO uint32_t G3_IO3 : 1;                       /*!<GROUP3_IO3 sampling mode */
            __IO uint32_t G3_IO4 : 1;                       /*!<GROUP3_IO4 sampling mode */
            __IO uint32_t G4_IO1 : 1;                       /*!<GROUP4_IO1 sampling mode */
            __IO uint32_t G4_IO2 : 1;                       /*!<GROUP4_IO2 sampling mode */
            __IO uint32_t G4_IO3 : 1;                       /*!<GROUP4_IO3 sampling mode */
            __IO uint32_t G4_IO4 : 1;                       /*!<GROUP4_IO4 sampling mode */
            __IO uint32_t G5_IO1 : 1;                       /*!<GROUP5_IO1 sampling mode */
            __IO uint32_t G5_IO2 : 1;                       /*!<GROUP5_IO2 sampling mode */
            __IO uint32_t G5_IO3 : 1;                       /*!<GROUP5_IO3 sampling mode */
            __IO uint32_t G5_IO4 : 1;                       /*!<GROUP5_IO4 sampling mode */
            __IO uint32_t G6_IO1 : 1;                       /*!<GROUP6_IO1 sampling mode */
            __IO uint32_t G6_IO2 : 1;                       /*!<GROUP6_IO2 sampling mode */
            __IO uint32_t G6_IO3 : 1;                       /*!<GROUP6_IO3 sampling mode */
            __IO uint32_t G6_IO4 : 1;                       /*!<GROUP6_IO4 sampling mode */
            __IO uint32_t G7_IO1 : 1;                       /*!<GROUP7_IO1 sampling mode */
            __IO uint32_t G7_IO2 : 1;                       /*!<GROUP7_IO2 sampling mode */
            __IO uint32_t G7_IO3 : 1;                       /*!<GROUP7_IO3 sampling mode */
            __IO uint32_t G7_IO4 : 1;                       /*!<GROUP7_IO4 sampling mode */
            __IO uint32_t G8_IO1 : 1;                       /*!<GROUP8_IO1 sampling mode */
            __IO uint32_t G8_IO2 : 1;                       /*!<GROUP8_IO2 sampling mode */
            __IO uint32_t G8_IO3 : 1;                       /*!<GROUP8_IO3 sampling mode */
            __IO uint32_t G8_IO4 : 1;                       /*!<GROUP8_IO4 sampling mode */
        } b;
        __IO uint32_t w;
    } IOSCR;                                                /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
         uint32_t __RESERVED2;
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                       /*!<GROUP1_IO1 channel mode */
            __IO uint32_t G1_IO2 : 1;                       /*!<GROUP1_IO2 channel mode */
            __IO uint32_t G1_IO3 : 1;                       /*!<GROUP1_IO3 channel mode */
            __IO uint32_t G1_IO4 : 1;                       /*!<GROUP1_IO4 channel mode */
            __IO uint32_t G2_IO1 : 1;                       /*!<GROUP2_IO1 channel mode */
            __IO uint32_t G2_IO2 : 1;                       /*!<GROUP2_IO2 channel mode */
            __IO uint32_t G2_IO3 : 1;                       /*!<GROUP2_IO3 channel mode */
            __IO uint32_t G2_IO4 : 1;                       /*!<GROUP2_IO4 channel mode */
            __IO uint32_t G3_IO1 : 1;                       /*!<GROUP3_IO1 channel mode */
            __IO uint32_t G3_IO2 : 1;                       /*!<GROUP3_IO2 channel mode */
            __IO uint32_t G3_IO3 : 1;                       /*!<GROUP3_IO3 channel mode */
            __IO uint32_t G3_IO4 : 1;                       /*!<GROUP3_IO4 channel mode */
            __IO uint32_t G4_IO1 : 1;                       /*!<GROUP4_IO1 channel mode */
            __IO uint32_t G4_IO2 : 1;                       /*!<GROUP4_IO2 channel mode */
            __IO uint32_t G4_IO3 : 1;                       /*!<GROUP4_IO3 channel mode */
            __IO uint32_t G4_IO4 : 1;                       /*!<GROUP4_IO4 channel mode */
            __IO uint32_t G5_IO1 : 1;                       /*!<GROUP5_IO1 channel mode */
            __IO uint32_t G5_IO2 : 1;                       /*!<GROUP5_IO2 channel mode */
            __IO uint32_t G5_IO3 : 1;                       /*!<GROUP5_IO3 channel mode */
            __IO uint32_t G5_IO4 : 1;                       /*!<GROUP5_IO4 channel mode */
            __IO uint32_t G6_IO1 : 1;                       /*!<GROUP6_IO1 channel mode */
            __IO uint32_t G6_IO2 : 1;                       /*!<GROUP6_IO2 channel mode */
            __IO uint32_t G6_IO3 : 1;                       /*!<GROUP6_IO3 channel mode */
            __IO uint32_t G6_IO4 : 1;                       /*!<GROUP6_IO4 channel mode */
            __IO uint32_t G7_IO1 : 1;                       /*!<GROUP7_IO1 channel mode */
            __IO uint32_t G7_IO2 : 1;                       /*!<GROUP7_IO2 channel mode */
            __IO uint32_t G7_IO3 : 1;                       /*!<GROUP7_IO3 channel mode */
            __IO uint32_t G7_IO4 : 1;                       /*!<GROUP7_IO4 channel mode */
            __IO uint32_t G8_IO1 : 1;                       /*!<GROUP8_IO1 channel mode */
            __IO uint32_t G8_IO2 : 1;                       /*!<GROUP8_IO2 channel mode */
            __IO uint32_t G8_IO3 : 1;                       /*!<GROUP8_IO3 channel mode */
            __IO uint32_t G8_IO4 : 1;                       /*!<GROUP8_IO4 channel mode */
        } b;
        __IO uint32_t w;
    } IOCCR;                                                /*!< TSC I/O channel control register,                         Address offset: 0x28 */
         uint32_t __RESERVED3;
    union {
        struct {
            __IO uint32_t G1E : 1;                          /*!<Analog IO GROUP1 enable */
            __IO uint32_t G2E : 1;                          /*!<Analog IO GROUP2 enable */
            __IO uint32_t G3E : 1;                          /*!<Analog IO GROUP3 enable */
            __IO uint32_t G4E : 1;                          /*!<Analog IO GROUP4 enable */
            __IO uint32_t G5E : 1;                          /*!<Analog IO GROUP5 enable */
            __IO uint32_t G6E : 1;                          /*!<Analog IO GROUP6 enable */
            __IO uint32_t G7E : 1;                          /*!<Analog IO GROUP7 enable */
            __IO uint32_t G8E : 1;                          /*!<Analog IO GROUP8 enable */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t G1S : 1;                          /*!<Analog IO GROUP1 status */
            __IO uint32_t G2S : 1;                          /*!<Analog IO GROUP2 status */
            __IO uint32_t G3S : 1;                          /*!<Analog IO GROUP3 status */
            __IO uint32_t G4S : 1;                          /*!<Analog IO GROUP4 status */
            __IO uint32_t G5S : 1;                          /*!<Analog IO GROUP5 status */
            __IO uint32_t G6S : 1;                          /*!<Analog IO GROUP6 status */
            __IO uint32_t G7S : 1;                          /*!<Analog IO GROUP7 status */
            __IO uint32_t G8S : 1;                          /*!<Analog IO GROUP8 status */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } IOGCSR;                                               /*!< TSC I/O group control status register,                    Address offset: 0x30 */
    __IO uint32_t IOGXCR[8];                                /*!< TSC I/O group x counter register,                         Address offset: 0x34-50 */
} TSC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t TSCE;                                 /*!<Touch sensing controller enable */
        __IO uint32_t START;                                /*!<Start acquisition */
        __IO uint32_t AM;                                   /*!<Acquisition mode */
        __IO uint32_t SYNCPOL;                              /*!<Synchronization pin polarity */
        __IO uint32_t IODEF;                                /*!<IO default mode */
        __IO uint32_t MCV[3];                               /*!<MCV[2:0] bits (Max Count Value) */
             uint32_t __RESERVED0[4];
        __IO uint32_t PGPSC[3];                             /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
        __IO uint32_t SSPSC;                                /*!<Spread Spectrum Prescaler */
        __IO uint32_t SSE;                                  /*!<Spread Spectrum Enable */
        __IO uint32_t SSD[7];                               /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
        __IO uint32_t CTPL[4];                              /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
        __IO uint32_t CTPH[4];                              /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
    } CR;                                                   /*!< TSC control register,                                     Address offset: 0x00 */
    struct {
        __IO uint32_t EOAIE;                                /*!<End of acquisition interrupt enable */
        __IO uint32_t MCEIE;                                /*!<Max count error interrupt enable */
             uint32_t __RESERVED0[30];
    } IER;                                                  /*!< TSC interrupt enable register,                            Address offset: 0x04 */
    struct {
        __IO uint32_t EOAIC;                                /*!<End of acquisition interrupt clear */
        __IO uint32_t MCEIC;                                /*!<Max count error interrupt clear */
             uint32_t __RESERVED0[30];
    } ICR;                                                  /*!< TSC interrupt clear register,                             Address offset: 0x08 */
    struct {
        __IO uint32_t EOAF;                                 /*!<End of acquisition flag */
        __IO uint32_t MCEF;                                 /*!<Max count error flag */
             uint32_t __RESERVED0[30];
    } ISR;                                                  /*!< TSC interrupt status register,                            Address offset: 0x0C */
    struct {
        __IO uint32_t G1_IO1;                               /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G1_IO2;                               /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G1_IO3;                               /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G1_IO4;                               /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO1;                               /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO2;                               /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO3;                               /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO4;                               /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO1;                               /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO2;                               /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO3;                               /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO4;                               /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO1;                               /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO2;                               /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO3;                               /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO4;                               /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO1;                               /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO2;                               /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO3;                               /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO4;                               /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO1;                               /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO2;                               /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO3;                               /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO4;                               /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO1;                               /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO2;                               /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO3;                               /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO4;                               /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO1;                               /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO2;                               /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO3;                               /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO4;                               /*!<GROUP8_IO4 schmitt trigger hysteresis mode */
    } IOHCR;                                                /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
         uint32_t __RESERVED0[32];
    struct {
        __IO uint32_t G1_IO1;                               /*!<GROUP1_IO1 analog switch enable */
        __IO uint32_t G1_IO2;                               /*!<GROUP1_IO2 analog switch enable */
        __IO uint32_t G1_IO3;                               /*!<GROUP1_IO3 analog switch enable */
        __IO uint32_t G1_IO4;                               /*!<GROUP1_IO4 analog switch enable */
        __IO uint32_t G2_IO1;                               /*!<GROUP2_IO1 analog switch enable */
        __IO uint32_t G2_IO2;                               /*!<GROUP2_IO2 analog switch enable */
        __IO uint32_t G2_IO3;                               /*!<GROUP2_IO3 analog switch enable */
        __IO uint32_t G2_IO4;                               /*!<GROUP2_IO4 analog switch enable */
        __IO uint32_t G3_IO1;                               /*!<GROUP3_IO1 analog switch enable */
        __IO uint32_t G3_IO2;                               /*!<GROUP3_IO2 analog switch enable */
        __IO uint32_t G3_IO3;                               /*!<GROUP3_IO3 analog switch enable */
        __IO uint32_t G3_IO4;                               /*!<GROUP3_IO4 analog switch enable */
        __IO uint32_t G4_IO1;                               /*!<GROUP4_IO1 analog switch enable */
        __IO uint32_t G4_IO2;                               /*!<GROUP4_IO2 analog switch enable */
        __IO uint32_t G4_IO3;                               /*!<GROUP4_IO3 analog switch enable */
        __IO uint32_t G4_IO4;                               /*!<GROUP4_IO4 analog switch enable */
        __IO uint32_t G5_IO1;                               /*!<GROUP5_IO1 analog switch enable */
        __IO uint32_t G5_IO2;                               /*!<GROUP5_IO2 analog switch enable */
        __IO uint32_t G5_IO3;                               /*!<GROUP5_IO3 analog switch enable */
        __IO uint32_t G5_IO4;                               /*!<GROUP5_IO4 analog switch enable */
        __IO uint32_t G6_IO1;                               /*!<GROUP6_IO1 analog switch enable */
        __IO uint32_t G6_IO2;                               /*!<GROUP6_IO2 analog switch enable */
        __IO uint32_t G6_IO3;                               /*!<GROUP6_IO3 analog switch enable */
        __IO uint32_t G6_IO4;                               /*!<GROUP6_IO4 analog switch enable */
        __IO uint32_t G7_IO1;                               /*!<GROUP7_IO1 analog switch enable */
        __IO uint32_t G7_IO2;                               /*!<GROUP7_IO2 analog switch enable */
        __IO uint32_t G7_IO3;                               /*!<GROUP7_IO3 analog switch enable */
        __IO uint32_t G7_IO4;                               /*!<GROUP7_IO4 analog switch enable */
        __IO uint32_t G8_IO1;                               /*!<GROUP8_IO1 analog switch enable */
        __IO uint32_t G8_IO2;                               /*!<GROUP8_IO2 analog switch enable */
        __IO uint32_t G8_IO3;                               /*!<GROUP8_IO3 analog switch enable */
        __IO uint32_t G8_IO4;                               /*!<GROUP8_IO4 analog switch enable */
    } IOASCR;                                               /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
         uint32_t __RESERVED1[32];
    struct {
        __IO uint32_t G1_IO1;                               /*!<GROUP1_IO1 sampling mode */
        __IO uint32_t G1_IO2;                               /*!<GROUP1_IO2 sampling mode */
        __IO uint32_t G1_IO3;                               /*!<GROUP1_IO3 sampling mode */
        __IO uint32_t G1_IO4;                               /*!<GROUP1_IO4 sampling mode */
        __IO uint32_t G2_IO1;                               /*!<GROUP2_IO1 sampling mode */
        __IO uint32_t G2_IO2;                               /*!<GROUP2_IO2 sampling mode */
        __IO uint32_t G2_IO3;                               /*!<GROUP2_IO3 sampling mode */
        __IO uint32_t G2_IO4;                               /*!<GROUP2_IO4 sampling mode */
        __IO uint32_t G3_IO1;                               /*!<GROUP3_IO1 sampling mode */
        __IO uint32_t G3_IO2;                               /*!<GROUP3_IO2 sampling mode */
        __IO uint32_t G3_IO3;                               /*!<GROUP3_IO3 sampling mode */
        __IO uint32_t G3_IO4;                               /*!<GROUP3_IO4 sampling mode */
        __IO uint32_t G4_IO1;                               /*!<GROUP4_IO1 sampling mode */
        __IO uint32_t G4_IO2;                               /*!<GROUP4_IO2 sampling mode */
        __IO uint32_t G4_IO3;                               /*!<GROUP4_IO3 sampling mode */
        __IO uint32_t G4_IO4;                               /*!<GROUP4_IO4 sampling mode */
        __IO uint32_t G5_IO1;                               /*!<GROUP5_IO1 sampling mode */
        __IO uint32_t G5_IO2;                               /*!<GROUP5_IO2 sampling mode */
        __IO uint32_t G5_IO3;                               /*!<GROUP5_IO3 sampling mode */
        __IO uint32_t G5_IO4;                               /*!<GROUP5_IO4 sampling mode */
        __IO uint32_t G6_IO1;                               /*!<GROUP6_IO1 sampling mode */
        __IO uint32_t G6_IO2;                               /*!<GROUP6_IO2 sampling mode */
        __IO uint32_t G6_IO3;                               /*!<GROUP6_IO3 sampling mode */
        __IO uint32_t G6_IO4;                               /*!<GROUP6_IO4 sampling mode */
        __IO uint32_t G7_IO1;                               /*!<GROUP7_IO1 sampling mode */
        __IO uint32_t G7_IO2;                               /*!<GROUP7_IO2 sampling mode */
        __IO uint32_t G7_IO3;                               /*!<GROUP7_IO3 sampling mode */
        __IO uint32_t G7_IO4;                               /*!<GROUP7_IO4 sampling mode */
        __IO uint32_t G8_IO1;                               /*!<GROUP8_IO1 sampling mode */
        __IO uint32_t G8_IO2;                               /*!<GROUP8_IO2 sampling mode */
        __IO uint32_t G8_IO3;                               /*!<GROUP8_IO3 sampling mode */
        __IO uint32_t G8_IO4;                               /*!<GROUP8_IO4 sampling mode */
    } IOSCR;                                                /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
         uint32_t __RESERVED2[32];
    struct {
        __IO uint32_t G1_IO1;                               /*!<GROUP1_IO1 channel mode */
        __IO uint32_t G1_IO2;                               /*!<GROUP1_IO2 channel mode */
        __IO uint32_t G1_IO3;                               /*!<GROUP1_IO3 channel mode */
        __IO uint32_t G1_IO4;                               /*!<GROUP1_IO4 channel mode */
        __IO uint32_t G2_IO1;                               /*!<GROUP2_IO1 channel mode */
        __IO uint32_t G2_IO2;                               /*!<GROUP2_IO2 channel mode */
        __IO uint32_t G2_IO3;                               /*!<GROUP2_IO3 channel mode */
        __IO uint32_t G2_IO4;                               /*!<GROUP2_IO4 channel mode */
        __IO uint32_t G3_IO1;                               /*!<GROUP3_IO1 channel mode */
        __IO uint32_t G3_IO2;                               /*!<GROUP3_IO2 channel mode */
        __IO uint32_t G3_IO3;                               /*!<GROUP3_IO3 channel mode */
        __IO uint32_t G3_IO4;                               /*!<GROUP3_IO4 channel mode */
        __IO uint32_t G4_IO1;                               /*!<GROUP4_IO1 channel mode */
        __IO uint32_t G4_IO2;                               /*!<GROUP4_IO2 channel mode */
        __IO uint32_t G4_IO3;                               /*!<GROUP4_IO3 channel mode */
        __IO uint32_t G4_IO4;                               /*!<GROUP4_IO4 channel mode */
        __IO uint32_t G5_IO1;                               /*!<GROUP5_IO1 channel mode */
        __IO uint32_t G5_IO2;                               /*!<GROUP5_IO2 channel mode */
        __IO uint32_t G5_IO3;                               /*!<GROUP5_IO3 channel mode */
        __IO uint32_t G5_IO4;                               /*!<GROUP5_IO4 channel mode */
        __IO uint32_t G6_IO1;                               /*!<GROUP6_IO1 channel mode */
        __IO uint32_t G6_IO2;                               /*!<GROUP6_IO2 channel mode */
        __IO uint32_t G6_IO3;                               /*!<GROUP6_IO3 channel mode */
        __IO uint32_t G6_IO4;                               /*!<GROUP6_IO4 channel mode */
        __IO uint32_t G7_IO1;                               /*!<GROUP7_IO1 channel mode */
        __IO uint32_t G7_IO2;                               /*!<GROUP7_IO2 channel mode */
        __IO uint32_t G7_IO3;                               /*!<GROUP7_IO3 channel mode */
        __IO uint32_t G7_IO4;                               /*!<GROUP7_IO4 channel mode */
        __IO uint32_t G8_IO1;                               /*!<GROUP8_IO1 channel mode */
        __IO uint32_t G8_IO2;                               /*!<GROUP8_IO2 channel mode */
        __IO uint32_t G8_IO3;                               /*!<GROUP8_IO3 channel mode */
        __IO uint32_t G8_IO4;                               /*!<GROUP8_IO4 channel mode */
    } IOCCR;                                                /*!< TSC I/O channel control register,                         Address offset: 0x28 */
         uint32_t __RESERVED3[32];
    struct {
        __IO uint32_t G1E;                                  /*!<Analog IO GROUP1 enable */
        __IO uint32_t G2E;                                  /*!<Analog IO GROUP2 enable */
        __IO uint32_t G3E;                                  /*!<Analog IO GROUP3 enable */
        __IO uint32_t G4E;                                  /*!<Analog IO GROUP4 enable */
        __IO uint32_t G5E;                                  /*!<Analog IO GROUP5 enable */
        __IO uint32_t G6E;                                  /*!<Analog IO GROUP6 enable */
        __IO uint32_t G7E;                                  /*!<Analog IO GROUP7 enable */
        __IO uint32_t G8E;                                  /*!<Analog IO GROUP8 enable */
             uint32_t __RESERVED0[8];
        __IO uint32_t G1S;                                  /*!<Analog IO GROUP1 status */
        __IO uint32_t G2S;                                  /*!<Analog IO GROUP2 status */
        __IO uint32_t G3S;                                  /*!<Analog IO GROUP3 status */
        __IO uint32_t G4S;                                  /*!<Analog IO GROUP4 status */
        __IO uint32_t G5S;                                  /*!<Analog IO GROUP5 status */
        __IO uint32_t G6S;                                  /*!<Analog IO GROUP6 status */
        __IO uint32_t G7S;                                  /*!<Analog IO GROUP7 status */
        __IO uint32_t G8S;                                  /*!<Analog IO GROUP8 status */
             uint32_t __RESERVED1[8];
    } IOGCSR;                                               /*!< TSC I/O group control status register,                    Address offset: 0x30 */
    __IO uint32_t IOGXCR[8][32];                            /*!< TSC I/O group x counter register,                         Address offset: 0x34-50 */
} TSC_BitBand_TypeDef;



/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct {
    union {
        struct {
            __IO uint32_t UE : 1;                           /*!< USART Enable */
            __IO uint32_t UESM : 1;                         /*!< USART Enable in STOP Mode */
            __IO uint32_t RE : 1;                           /*!< Receiver Enable */
            __IO uint32_t TE : 1;                           /*!< Transmitter Enable */
            __IO uint32_t IDLEIE : 1;                       /*!< IDLE Interrupt Enable */
            __IO uint32_t RXNEIE : 1;                       /*!< RXNE Interrupt Enable */
            __IO uint32_t TCIE : 1;                         /*!< Transmission Complete Interrupt Enable */
            __IO uint32_t TXEIE : 1;                        /*!< TXE Interrupt Enable */
            __IO uint32_t PEIE : 1;                         /*!< PE Interrupt Enable */
            __IO uint32_t PS : 1;                           /*!< Parity Selection */
            __IO uint32_t PCE : 1;                          /*!< Parity Control Enable */
            __IO uint32_t WAKE : 1;                         /*!< Receiver Wakeup method */
            __IO uint32_t M0 : 1;                           /*!< Word length bit 0 */
            __IO uint32_t MME : 1;                          /*!< Mute Mode Enable */
            __IO uint32_t CMIE : 1;                         /*!< Character match interrupt enable */
            __IO uint32_t OVER8 : 1;                        /*!< Oversampling by 8-bit or 16-bit mode */
            __IO uint32_t DEDT : 5;                         /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
            __IO uint32_t DEAT : 5;                         /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
            __IO uint32_t RTOIE : 1;                        /*!< Receive Time Out interrupt enable */
            __IO uint32_t EOBIE : 1;                        /*!< End of Block interrupt enable */
            __IO uint32_t M1 : 1;                           /*!< Word length bit 1 */
                 uint32_t __RESERVED0 : 3;
        } b;
        __IO uint32_t w;
    } CR1;                                                  /*!< USART Control register 1,                 Address offset: 0x00 */
    union {
        struct {
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t ADDM7 : 1;                        /*!< 7-bit or 4-bit Address Detection */
            __IO uint32_t LBDL : 1;                         /*!< LIN Break Detection Length */
            __IO uint32_t LBDIE : 1;                        /*!< LIN Break Detection Interrupt Enable */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t LBCL : 1;                         /*!< Last Bit Clock pulse */
            __IO uint32_t CPHA : 1;                         /*!< Clock Phase */
            __IO uint32_t CPOL : 1;                         /*!< Clock Polarity */
            __IO uint32_t CLKEN : 1;                        /*!< Clock Enable */
            __IO uint32_t STOP : 2;                         /*!< STOP[1:0] bits (STOP bits) */
            __IO uint32_t LINEN : 1;                        /*!< LIN mode enable */
            __IO uint32_t SWAP : 1;                         /*!< SWAP TX/RX pins */
            __IO uint32_t RXINV : 1;                        /*!< RX pin active level inversion */
            __IO uint32_t TXINV : 1;                        /*!< TX pin active level inversion */
            __IO uint32_t DATAINV : 1;                      /*!< Binary data inversion */
            __IO uint32_t MSBFIRST : 1;                     /*!< Most Significant Bit First */
            __IO uint32_t ABREN : 1;                        /*!< Auto Baud-Rate Enable*/
            __IO uint32_t ABRMODE : 2;                      /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
            __IO uint32_t RTOEN : 1;                        /*!< Receiver Time-Out enable */
            __IO uint32_t ADD : 8;                          /*!< Address of the USART node */
        } b;
        __IO uint32_t w;
    } CR2;                                                  /*!< USART Control register 2,                 Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t EIE : 1;                          /*!< Error Interrupt Enable */
            __IO uint32_t IREN : 1;                         /*!< IrDA mode Enable */
            __IO uint32_t IRLP : 1;                         /*!< IrDA Low-Power */
            __IO uint32_t HDSEL : 1;                        /*!< Half-Duplex Selection */
            __IO uint32_t NACK : 1;                         /*!< SmartCard NACK enable */
            __IO uint32_t SCEN : 1;                         /*!< SmartCard mode enable */
            __IO uint32_t DMAR : 1;                         /*!< DMA Enable Receiver */
            __IO uint32_t DMAT : 1;                         /*!< DMA Enable Transmitter */
            __IO uint32_t RTSE : 1;                         /*!< RTS Enable */
            __IO uint32_t CTSE : 1;                         /*!< CTS Enable */
            __IO uint32_t CTSIE : 1;                        /*!< CTS Interrupt Enable */
            __IO uint32_t ONEBIT : 1;                       /*!< One sample bit method enable */
            __IO uint32_t OVRDIS : 1;                       /*!< Overrun Disable */
            __IO uint32_t DDRE : 1;                         /*!< DMA Disable on Reception Error */
            __IO uint32_t DEM : 1;                          /*!< Driver Enable Mode */
            __IO uint32_t DEP : 1;                          /*!< Driver Enable Polarity Selection */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SCARCNT : 3;                      /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
            __IO uint32_t WUS : 2;                          /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
            __IO uint32_t WUFIE : 1;                        /*!< Wake Up Interrupt Enable */
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } CR3;                                                  /*!< USART Control register 3,                 Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t DIV_FRACTION : 4;                 /*!< Fraction of USARTDIV */
            __IO uint32_t DIV_MANTISSA : 12;                /*!< Mantissa of USARTDIV */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } BRR;                                                  /*!< USART Baud rate register,                 Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t PSC : 8;                          /*!< PSC[7:0] bits (Prescaler value) */
            __IO uint32_t GT : 8;                           /*!< GT[7:0] bits (Guard time value) */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } GTPR;                                                 /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t RTO : 24;                         /*!< Receiver Time Out Value */
            __IO uint32_t BLEN : 8;                         /*!< Block Length */
        } b;
        __IO uint32_t w;
    } RTOR;                                                 /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t ABRRQ : 1;                        /*!< Auto-Baud Rate Request */
            __IO uint32_t SBKRQ : 1;                        /*!< Send Break Request */
            __IO uint32_t MMRQ : 1;                         /*!< Mute Mode Request */
            __IO uint32_t RXFRQ : 1;                        /*!< Receive Data flush Request */
            __IO uint32_t TXFRQ : 1;                        /*!< Transmit data flush Request */
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } RQR;                                                  /*!< USART Request register,                   Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t PE : 1;                           /*!< Parity Error */
            __IO uint32_t FE : 1;                           /*!< Framing Error */
            __IO uint32_t NE : 1;                           /*!< Noise detected Flag */
            __IO uint32_t ORE : 1;                          /*!< OverRun Error */
            __IO uint32_t IDLE : 1;                         /*!< IDLE line detected */
            __IO uint32_t RXNE : 1;                         /*!< Read Data Register Not Empty */
            __IO uint32_t TC : 1;                           /*!< Transmission Complete */
            __IO uint32_t TXE : 1;                          /*!< Transmit Data Register Empty */
            __IO uint32_t LBDF : 1;                         /*!< LIN Break Detection Flag */
            __IO uint32_t CTSIF : 1;                        /*!< CTS interrupt flag */
            __IO uint32_t CTS : 1;                          /*!< CTS flag */
            __IO uint32_t RTOF : 1;                         /*!< Receiver Time Out */
            __IO uint32_t EOBF : 1;                         /*!< End Of Block Flag */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t ABRE : 1;                         /*!< Auto-Baud Rate Error */
            __IO uint32_t ABRF : 1;                         /*!< Auto-Baud Rate Flag */
            __IO uint32_t BUSY : 1;                         /*!< Busy Flag */
            __IO uint32_t CMF : 1;                          /*!< Character Match Flag */
            __IO uint32_t SBKF : 1;                         /*!< Send Break Flag */
            __IO uint32_t RWU : 1;                          /*!< Receive Wake Up from mute mode Flag */
            __IO uint32_t WUF : 1;                          /*!< Wake Up from stop mode Flag */
            __IO uint32_t TEACK : 1;                        /*!< Transmit Enable Acknowledge Flag */
            __IO uint32_t REACK : 1;                        /*!< Receive Enable Acknowledge Flag */
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } ISR;                                                  /*!< USART Interrupt and status register,      Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t PECF : 1;                         /*!< Parity Error Clear Flag */
            __IO uint32_t FECF : 1;                         /*!< Framing Error Clear Flag */
            __IO uint32_t NCF : 1;                          /*!< Noise detected Clear Flag */
            __IO uint32_t ORECF : 1;                        /*!< OverRun Error Clear Flag */
            __IO uint32_t IDLECF : 1;                       /*!< IDLE line detected Clear Flag */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TCCF : 1;                         /*!< Transmission Complete Clear Flag */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t LBDCF : 1;                        /*!< LIN Break Detection Clear Flag */
            __IO uint32_t CTSCF : 1;                        /*!< CTS Interrupt Clear Flag */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t RTOCF : 1;                        /*!< Receiver Time Out Clear Flag */
            __IO uint32_t EOBCF : 1;                        /*!< End Of Block Clear Flag */
                 uint32_t __RESERVED3 : 4;
            __IO uint32_t CMCF : 1;                         /*!< Character Match Clear Flag */
                 uint32_t __RESERVED4 : 2;
            __IO uint32_t WUCF : 1;                         /*!< Wake Up from stop mode Clear Flag */
                 uint32_t __RESERVED5 : 11;
        } b;
        __IO uint32_t w;
    } ICR;                                                  /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    __IO uint16_t RDR;                                      /*!< USART Receive Data register,              Address offset: 0x24 */
         uint16_t __RESERVED0;
    __IO uint16_t TDR;                                      /*!< USART Transmit Data register,             Address offset: 0x28 */
} USART_TypeDef;


typedef struct {
    struct {
        __IO uint32_t UE;                                   /*!< USART Enable */
        __IO uint32_t UESM;                                 /*!< USART Enable in STOP Mode */
        __IO uint32_t RE;                                   /*!< Receiver Enable */
        __IO uint32_t TE;                                   /*!< Transmitter Enable */
        __IO uint32_t IDLEIE;                               /*!< IDLE Interrupt Enable */
        __IO uint32_t RXNEIE;                               /*!< RXNE Interrupt Enable */
        __IO uint32_t TCIE;                                 /*!< Transmission Complete Interrupt Enable */
        __IO uint32_t TXEIE;                                /*!< TXE Interrupt Enable */
        __IO uint32_t PEIE;                                 /*!< PE Interrupt Enable */
        __IO uint32_t PS;                                   /*!< Parity Selection */
        __IO uint32_t PCE;                                  /*!< Parity Control Enable */
        __IO uint32_t WAKE;                                 /*!< Receiver Wakeup method */
        __IO uint32_t M0;                                   /*!< Word length bit 0 */
        __IO uint32_t MME;                                  /*!< Mute Mode Enable */
        __IO uint32_t CMIE;                                 /*!< Character match interrupt enable */
        __IO uint32_t OVER8;                                /*!< Oversampling by 8-bit or 16-bit mode */
        __IO uint32_t DEDT[5];                              /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
        __IO uint32_t DEAT[5];                              /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
        __IO uint32_t RTOIE;                                /*!< Receive Time Out interrupt enable */
        __IO uint32_t EOBIE;                                /*!< End of Block interrupt enable */
        __IO uint32_t M1;                                   /*!< Word length bit 1 */
             uint32_t __RESERVED0[3];
    } CR1;                                                  /*!< USART Control register 1,                 Address offset: 0x00 */
    struct {
             uint32_t __RESERVED0[4];
        __IO uint32_t ADDM7;                                /*!< 7-bit or 4-bit Address Detection */
        __IO uint32_t LBDL;                                 /*!< LIN Break Detection Length */
        __IO uint32_t LBDIE;                                /*!< LIN Break Detection Interrupt Enable */
             uint32_t __RESERVED1;
        __IO uint32_t LBCL;                                 /*!< Last Bit Clock pulse */
        __IO uint32_t CPHA;                                 /*!< Clock Phase */
        __IO uint32_t CPOL;                                 /*!< Clock Polarity */
        __IO uint32_t CLKEN;                                /*!< Clock Enable */
        __IO uint32_t STOP[2];                              /*!< STOP[1:0] bits (STOP bits) */
        __IO uint32_t LINEN;                                /*!< LIN mode enable */
        __IO uint32_t SWAP;                                 /*!< SWAP TX/RX pins */
        __IO uint32_t RXINV;                                /*!< RX pin active level inversion */
        __IO uint32_t TXINV;                                /*!< TX pin active level inversion */
        __IO uint32_t DATAINV;                              /*!< Binary data inversion */
        __IO uint32_t MSBFIRST;                             /*!< Most Significant Bit First */
        __IO uint32_t ABREN;                                /*!< Auto Baud-Rate Enable*/
        __IO uint32_t ABRMODE[2];                           /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
        __IO uint32_t RTOEN;                                /*!< Receiver Time-Out enable */
        __IO uint32_t ADD[8];                               /*!< Address of the USART node */
    } CR2;                                                  /*!< USART Control register 2,                 Address offset: 0x04 */
    struct {
        __IO uint32_t EIE;                                  /*!< Error Interrupt Enable */
        __IO uint32_t IREN;                                 /*!< IrDA mode Enable */
        __IO uint32_t IRLP;                                 /*!< IrDA Low-Power */
        __IO uint32_t HDSEL;                                /*!< Half-Duplex Selection */
        __IO uint32_t NACK;                                 /*!< SmartCard NACK enable */
        __IO uint32_t SCEN;                                 /*!< SmartCard mode enable */
        __IO uint32_t DMAR;                                 /*!< DMA Enable Receiver */
        __IO uint32_t DMAT;                                 /*!< DMA Enable Transmitter */
        __IO uint32_t RTSE;                                 /*!< RTS Enable */
        __IO uint32_t CTSE;                                 /*!< CTS Enable */
        __IO uint32_t CTSIE;                                /*!< CTS Interrupt Enable */
        __IO uint32_t ONEBIT;                               /*!< One sample bit method enable */
        __IO uint32_t OVRDIS;                               /*!< Overrun Disable */
        __IO uint32_t DDRE;                                 /*!< DMA Disable on Reception Error */
        __IO uint32_t DEM;                                  /*!< Driver Enable Mode */
        __IO uint32_t DEP;                                  /*!< Driver Enable Polarity Selection */
             uint32_t __RESERVED0;
        __IO uint32_t SCARCNT[3];                           /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
        __IO uint32_t WUS[2];                               /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
        __IO uint32_t WUFIE;                                /*!< Wake Up Interrupt Enable */
             uint32_t __RESERVED1[9];
    } CR3;                                                  /*!< USART Control register 3,                 Address offset: 0x08 */
    struct {
        __IO uint32_t DIV_FRACTION[4];                      /*!< Fraction of USARTDIV */
        __IO uint32_t DIV_MANTISSA[12];                     /*!< Mantissa of USARTDIV */
             uint32_t __RESERVED0[16];
    } BRR;                                                  /*!< USART Baud rate register,                 Address offset: 0x0C */
    struct {
        __IO uint32_t PSC[8];                               /*!< PSC[7:0] bits (Prescaler value) */
        __IO uint32_t GT[8];                                /*!< GT[7:0] bits (Guard time value) */
             uint32_t __RESERVED0[16];
    } GTPR;                                                 /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    struct {
        __IO uint32_t RTO[24];                              /*!< Receiver Time Out Value */
        __IO uint32_t BLEN[8];                              /*!< Block Length */
    } RTOR;                                                 /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    struct {
        __IO uint32_t ABRRQ;                                /*!< Auto-Baud Rate Request */
        __IO uint32_t SBKRQ;                                /*!< Send Break Request */
        __IO uint32_t MMRQ;                                 /*!< Mute Mode Request */
        __IO uint32_t RXFRQ;                                /*!< Receive Data flush Request */
        __IO uint32_t TXFRQ;                                /*!< Transmit data flush Request */
             uint32_t __RESERVED0[27];
    } RQR;                                                  /*!< USART Request register,                   Address offset: 0x18 */
    struct {
        __IO uint32_t PE;                                   /*!< Parity Error */
        __IO uint32_t FE;                                   /*!< Framing Error */
        __IO uint32_t NE;                                   /*!< Noise detected Flag */
        __IO uint32_t ORE;                                  /*!< OverRun Error */
        __IO uint32_t IDLE;                                 /*!< IDLE line detected */
        __IO uint32_t RXNE;                                 /*!< Read Data Register Not Empty */
        __IO uint32_t TC;                                   /*!< Transmission Complete */
        __IO uint32_t TXE;                                  /*!< Transmit Data Register Empty */
        __IO uint32_t LBDF;                                 /*!< LIN Break Detection Flag */
        __IO uint32_t CTSIF;                                /*!< CTS interrupt flag */
        __IO uint32_t CTS;                                  /*!< CTS flag */
        __IO uint32_t RTOF;                                 /*!< Receiver Time Out */
        __IO uint32_t EOBF;                                 /*!< End Of Block Flag */
             uint32_t __RESERVED0;
        __IO uint32_t ABRE;                                 /*!< Auto-Baud Rate Error */
        __IO uint32_t ABRF;                                 /*!< Auto-Baud Rate Flag */
        __IO uint32_t BUSY;                                 /*!< Busy Flag */
        __IO uint32_t CMF;                                  /*!< Character Match Flag */
        __IO uint32_t SBKF;                                 /*!< Send Break Flag */
        __IO uint32_t RWU;                                  /*!< Receive Wake Up from mute mode Flag */
        __IO uint32_t WUF;                                  /*!< Wake Up from stop mode Flag */
        __IO uint32_t TEACK;                                /*!< Transmit Enable Acknowledge Flag */
        __IO uint32_t REACK;                                /*!< Receive Enable Acknowledge Flag */
             uint32_t __RESERVED1[9];
    } ISR;                                                  /*!< USART Interrupt and status register,      Address offset: 0x1C */
    struct {
        __IO uint32_t PECF;                                 /*!< Parity Error Clear Flag */
        __IO uint32_t FECF;                                 /*!< Framing Error Clear Flag */
        __IO uint32_t NCF;                                  /*!< Noise detected Clear Flag */
        __IO uint32_t ORECF;                                /*!< OverRun Error Clear Flag */
        __IO uint32_t IDLECF;                               /*!< IDLE line detected Clear Flag */
             uint32_t __RESERVED0;
        __IO uint32_t TCCF;                                 /*!< Transmission Complete Clear Flag */
             uint32_t __RESERVED1;
        __IO uint32_t LBDCF;                                /*!< LIN Break Detection Clear Flag */
        __IO uint32_t CTSCF;                                /*!< CTS Interrupt Clear Flag */
             uint32_t __RESERVED2;
        __IO uint32_t RTOCF;                                /*!< Receiver Time Out Clear Flag */
        __IO uint32_t EOBCF;                                /*!< End Of Block Clear Flag */
             uint32_t __RESERVED3[4];
        __IO uint32_t CMCF;                                 /*!< Character Match Clear Flag */
             uint32_t __RESERVED4[2];
        __IO uint32_t WUCF;                                 /*!< Wake Up from stop mode Clear Flag */
             uint32_t __RESERVED5[11];
    } ICR;                                                  /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    __IO uint32_t RDR[16];                                  /*!< USART Receive Data register,              Address offset: 0x24 */
         uint32_t __RESERVED0[16];
    __IO uint32_t TDR[16];                                  /*!< USART Transmit Data register,             Address offset: 0x28 */
} USART_BitBand_TypeDef;



/**
  * @brief Window WATCHDOG
  */
typedef struct {
    union {
        struct {
            __IO uint32_t T : 7;                            /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
            __IO uint32_t WDGA : 1;                         /*!<Activation bit */
                 uint32_t __RESERVED0 : 24;
        } b;
        __IO uint32_t w;
    } CR;                                                   /*!< WWDG Control register,       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t W : 7;                            /*!< W[6:0] bits (7-bit window value) */
            __IO uint32_t WDGTB : 2;                        /*!< WDGTB[1:0] bits (Timer Base) */
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
        __IO uint32_t T[7];                                 /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
        __IO uint32_t WDGA;                                 /*!<Activation bit */
             uint32_t __RESERVED0[24];
    } CR;                                                   /*!< WWDG Control register,       Address offset: 0x00 */
    struct {
        __IO uint32_t W[7];                                 /*!< W[6:0] bits (7-bit window value) */
        __IO uint32_t WDGTB[2];                             /*!< WDGTB[1:0] bits (Timer Base) */
        __IO uint32_t EWI;                                  /*!<Early Wakeup Interrupt */
             uint32_t __RESERVED0[22];
    } CFR;                                                  /*!< WWDG Configuration register, Address offset: 0x04 */
    struct {
        __IO uint32_t EWIF;                                 /*!<Early Wakeup Interrupt Flag */
             uint32_t __RESERVED0[31];
    } SR;                                                   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_BitBand_TypeDef;



/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            ((uint32_t)0x08000000U) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000U) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000U) /*!< Peripheral base address in the alias region */
#define SRAM_BB_BASE          ((uint32_t)0x22000000U) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000U) /*!< Peripheral base address in the bit-band region */


/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000U)
#define AHB3PERIPH_BASE       (PERIPH_BASE + 0x10000000U)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x00000000U)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x00001000U)
#define RTC_BASE              (APB1PERIPH_BASE + 0x00002800U)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x00002C00U)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x00003000U)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x00003400U)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x00003800U)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x00003C00U)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x00004000U)
#define USART2_BASE           (APB1PERIPH_BASE + 0x00004400U)
#define USART3_BASE           (APB1PERIPH_BASE + 0x00004800U)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x00005400U)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x00005800U)
#define PWR_BASE              (APB1PERIPH_BASE + 0x00007000U)
#define DAC1_BASE             (APB1PERIPH_BASE + 0x00007400U)
#define DAC_BASE               DAC1_BASE
#define I2C3_BASE             (APB1PERIPH_BASE + 0x00007800U)

/*!< APB2 peripherals */
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x00000000U)
#define COMP2_BASE            (APB2PERIPH_BASE + 0x00000020U)
#define COMP4_BASE            (APB2PERIPH_BASE + 0x00000028U)
#define COMP6_BASE            (APB2PERIPH_BASE + 0x00000030U)
#define COMP_BASE             COMP2_BASE
#define OPAMP2_BASE           (APB2PERIPH_BASE + 0x0000003CU)
#define OPAMP_BASE            OPAMP2_BASE
#define EXTI_BASE             (APB2PERIPH_BASE + 0x00000400U)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x00002C00U)
#define USART1_BASE           (APB2PERIPH_BASE + 0x00003800U)
#define TIM15_BASE            (APB2PERIPH_BASE + 0x00004000U)
#define TIM16_BASE            (APB2PERIPH_BASE + 0x00004400U)
#define TIM17_BASE            (APB2PERIPH_BASE + 0x00004800U)

/*!< AHB1 peripherals */
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x00000000U)
#define DMA1_Channel1_BASE    (AHB1PERIPH_BASE + 0x00000008U)
#define DMA1_Channel2_BASE    (AHB1PERIPH_BASE + 0x0000001CU)
#define DMA1_Channel3_BASE    (AHB1PERIPH_BASE + 0x00000030U)
#define DMA1_Channel4_BASE    (AHB1PERIPH_BASE + 0x00000044U)
#define DMA1_Channel5_BASE    (AHB1PERIPH_BASE + 0x00000058U)
#define DMA1_Channel6_BASE    (AHB1PERIPH_BASE + 0x0000006CU)
#define DMA1_Channel7_BASE    (AHB1PERIPH_BASE + 0x00000080U)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x00001000U)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x00002000U) /*!< Flash registers base address */
#define OB_BASE               ((uint32_t)0x1FFFF800U)         /*!< Flash Option Bytes base address */
#define FLASHSIZE_BASE        ((uint32_t)0x1FFFF7CCU)         /*!< FLASH Size register base address */
#define UID_BASE              ((uint32_t)0x1FFFF7ACU)         /*!< Unique device ID register base address */
#define CRC_BASE              (AHB1PERIPH_BASE + 0x00003000U)
#define TSC_BASE              (AHB1PERIPH_BASE + 0x00004000U)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000U)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400U)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800U)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00U)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400U)

/*!< AHB3 peripherals */
#define ADC1_BASE             (AHB3PERIPH_BASE + 0x00000000U)
#define ADC1_COMMON_BASE      (AHB3PERIPH_BASE + 0x00000300U)

#define DBGMCU_BASE          ((uint32_t)0xE0042000U) /*!< Debug MCU registers base address */
/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define DAC1                ((DAC_TypeDef *) DAC1_BASE)
#define COMP2               ((COMP_TypeDef *) COMP2_BASE)
#define COMP4               ((COMP_TypeDef *) COMP4_BASE)
#define COMP6               ((COMP_TypeDef *) COMP6_BASE)
/* Legacy define */
#define COMP                ((COMP_TypeDef *) COMP_BASE)
#define OPAMP               ((OPAMP_TypeDef *) OPAMP_BASE)
#define OPAMP2              ((OPAMP_TypeDef *) OPAMP2_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define TIM15               ((TIM_TypeDef *) TIM15_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define TIM17               ((TIM_TypeDef *) TIM17_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define TSC                 ((TSC_TypeDef *) TSC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC1_COMMON_BASE)


#define COMP_BB(inst)              ((COMP_BitBand_TypeDef *) PERIPH_BB(inst))
#define CRC_BB                     ((CRC_BitBand_TypeDef *) PERIPH_BB(CRC_BASE))
#define DAC_BB(inst)               ((DAC_BitBand_TypeDef *) PERIPH_BB(inst))
#define DMA_Channel_BB(inst)       ((DMA_Channel_BitBand_TypeDef *) PERIPH_BB(inst))
#define DMA_BB(inst)               ((DMA_BitBand_TypeDef *) PERIPH_BB(inst))
#define EXTI_BB                    ((EXTI_BitBand_TypeDef *) PERIPH_BB(EXTI_BASE))
#define FLASH_BB                   ((FLASH_BitBand_TypeDef *) PERIPH_BB(FLASH_R_BASE))
#define I2C_BB(inst)               ((I2C_BitBand_TypeDef *) PERIPH_BB(inst))
#define IWDG_BB                    ((IWDG_BitBand_TypeDef *) PERIPH_BB(IWDG_BASE))
#define OPAMP_BB(inst)             ((OPAMP_BitBand_TypeDef *) PERIPH_BB(inst))
#define PWR_BB                     ((PWR_BitBand_TypeDef *) PERIPH_BB(PWR_BASE))
#define RCC_BB                     ((RCC_BitBand_TypeDef *) PERIPH_BB(RCC_BASE))
#define RTC_BB                     ((RTC_BitBand_TypeDef *) PERIPH_BB(RTC_BASE))
#define SPI_BB(inst)               ((SPI_BitBand_TypeDef *) PERIPH_BB(inst))
#define SYSCFG_BB                  ((SYSCFG_BitBand_TypeDef *) PERIPH_BB(SYSCFG_BASE))
#define TIM_BB(inst)               ((TIM_BitBand_TypeDef *) PERIPH_BB(inst))
#define TSC_BB                     ((TSC_BitBand_TypeDef *) PERIPH_BB(TSC_BASE))
#define USART_BB(inst)             ((USART_BitBand_TypeDef *) PERIPH_BB(inst))
#define WWDG_BB                    ((WWDG_BitBand_TypeDef *) PERIPH_BB(WWDG_BASE))

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
/*                        Analog to Digital Converter SAR (ADC)               */
/*                                                                            */
/******************************************************************************/

#define ADC5_V1_1                                      /*!< ADC IP version */

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
 */
/* Note: No specific macro feature on this device */

/********************  Bit definition for ADC_ISR register  ********************/
#define ADC_ISR_ADRDY_Pos              (0U)                                    
#define ADC_ISR_ADRDY_Msk              (0x1U << ADC_ISR_ADRDY_Pos)             /*!< 0x00000001 */
#define ADC_ISR_ADRDY                  ADC_ISR_ADRDY_Msk                       /*!< ADC ready flag */
#define ADC_ISR_EOSMP_Pos              (1U)                                    
#define ADC_ISR_EOSMP_Msk              (0x1U << ADC_ISR_EOSMP_Pos)             /*!< 0x00000002 */
#define ADC_ISR_EOSMP                  ADC_ISR_EOSMP_Msk                       /*!< ADC group regular end of sampling flag */
#define ADC_ISR_EOC_Pos                (2U)                                    
#define ADC_ISR_EOC_Msk                (0x1U << ADC_ISR_EOC_Pos)               /*!< 0x00000004 */
#define ADC_ISR_EOC                    ADC_ISR_EOC_Msk                         /*!< ADC group regular end of unitary conversion flag */
#define ADC_ISR_EOS_Pos                (3U)                                    
#define ADC_ISR_EOS_Msk                (0x1U << ADC_ISR_EOS_Pos)               /*!< 0x00000008 */
#define ADC_ISR_EOS                    ADC_ISR_EOS_Msk                         /*!< ADC group regular end of sequence conversions flag */
#define ADC_ISR_OVR_Pos                (4U)                                    
#define ADC_ISR_OVR_Msk                (0x1U << ADC_ISR_OVR_Pos)               /*!< 0x00000010 */
#define ADC_ISR_OVR                    ADC_ISR_OVR_Msk                         /*!< ADC group regular overrun flag */
#define ADC_ISR_JEOC_Pos               (5U)                                    
#define ADC_ISR_JEOC_Msk               (0x1U << ADC_ISR_JEOC_Pos)              /*!< 0x00000020 */
#define ADC_ISR_JEOC                   ADC_ISR_JEOC_Msk                        /*!< ADC group injected end of unitary conversion flag */
#define ADC_ISR_JEOS_Pos               (6U)                                    
#define ADC_ISR_JEOS_Msk               (0x1U << ADC_ISR_JEOS_Pos)              /*!< 0x00000040 */
#define ADC_ISR_JEOS                   ADC_ISR_JEOS_Msk                        /*!< ADC group injected end of sequence conversions flag */
#define ADC_ISR_AWD1_Pos               (7U)                                    
#define ADC_ISR_AWD1_Msk               (0x1U << ADC_ISR_AWD1_Pos)              /*!< 0x00000080 */
#define ADC_ISR_AWD1                   ADC_ISR_AWD1_Msk                        /*!< ADC analog watchdog 1 flag */
#define ADC_ISR_AWD2_Pos               (8U)                                    
#define ADC_ISR_AWD2_Msk               (0x1U << ADC_ISR_AWD2_Pos)              /*!< 0x00000100 */
#define ADC_ISR_AWD2                   ADC_ISR_AWD2_Msk                        /*!< ADC analog watchdog 2 flag */
#define ADC_ISR_AWD3_Pos               (9U)                                    
#define ADC_ISR_AWD3_Msk               (0x1U << ADC_ISR_AWD3_Pos)              /*!< 0x00000200 */
#define ADC_ISR_AWD3                   ADC_ISR_AWD3_Msk                        /*!< ADC analog watchdog 3 flag */
#define ADC_ISR_JQOVF_Pos              (10U)                                   
#define ADC_ISR_JQOVF_Msk              (0x1U << ADC_ISR_JQOVF_Pos)             /*!< 0x00000400 */
#define ADC_ISR_JQOVF                  ADC_ISR_JQOVF_Msk                       /*!< ADC group injected contexts queue overflow flag */

/* Legacy defines */
#define ADC_ISR_ADRD            (ADC_ISR_ADRDY)

/********************  Bit definition for ADC_IER register  ********************/
#define ADC_IER_ADRDYIE_Pos            (0U)                                    
#define ADC_IER_ADRDYIE_Msk            (0x1U << ADC_IER_ADRDYIE_Pos)           /*!< 0x00000001 */
#define ADC_IER_ADRDYIE                ADC_IER_ADRDYIE_Msk                     /*!< ADC ready interrupt */
#define ADC_IER_EOSMPIE_Pos            (1U)                                    
#define ADC_IER_EOSMPIE_Msk            (0x1U << ADC_IER_EOSMPIE_Pos)           /*!< 0x00000002 */
#define ADC_IER_EOSMPIE                ADC_IER_EOSMPIE_Msk                     /*!< ADC group regular end of sampling interrupt */
#define ADC_IER_EOCIE_Pos              (2U)                                    
#define ADC_IER_EOCIE_Msk              (0x1U << ADC_IER_EOCIE_Pos)             /*!< 0x00000004 */
#define ADC_IER_EOCIE                  ADC_IER_EOCIE_Msk                       /*!< ADC group regular end of unitary conversion interrupt */
#define ADC_IER_EOSIE_Pos              (3U)                                    
#define ADC_IER_EOSIE_Msk              (0x1U << ADC_IER_EOSIE_Pos)             /*!< 0x00000008 */
#define ADC_IER_EOSIE                  ADC_IER_EOSIE_Msk                       /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_IER_OVRIE_Pos              (4U)                                    
#define ADC_IER_OVRIE_Msk              (0x1U << ADC_IER_OVRIE_Pos)             /*!< 0x00000010 */
#define ADC_IER_OVRIE                  ADC_IER_OVRIE_Msk                       /*!< ADC group regular overrun interrupt */
#define ADC_IER_JEOCIE_Pos             (5U)                                    
#define ADC_IER_JEOCIE_Msk             (0x1U << ADC_IER_JEOCIE_Pos)            /*!< 0x00000020 */
#define ADC_IER_JEOCIE                 ADC_IER_JEOCIE_Msk                      /*!< ADC group injected end of unitary conversion interrupt */
#define ADC_IER_JEOSIE_Pos             (6U)                                    
#define ADC_IER_JEOSIE_Msk             (0x1U << ADC_IER_JEOSIE_Pos)            /*!< 0x00000040 */
#define ADC_IER_JEOSIE                 ADC_IER_JEOSIE_Msk                      /*!< ADC group injected end of sequence conversions interrupt */
#define ADC_IER_AWD1IE_Pos             (7U)                                    
#define ADC_IER_AWD1IE_Msk             (0x1U << ADC_IER_AWD1IE_Pos)            /*!< 0x00000080 */
#define ADC_IER_AWD1IE                 ADC_IER_AWD1IE_Msk                      /*!< ADC analog watchdog 1 interrupt */
#define ADC_IER_AWD2IE_Pos             (8U)                                    
#define ADC_IER_AWD2IE_Msk             (0x1U << ADC_IER_AWD2IE_Pos)            /*!< 0x00000100 */
#define ADC_IER_AWD2IE                 ADC_IER_AWD2IE_Msk                      /*!< ADC analog watchdog 2 interrupt */
#define ADC_IER_AWD3IE_Pos             (9U)                                    
#define ADC_IER_AWD3IE_Msk             (0x1U << ADC_IER_AWD3IE_Pos)            /*!< 0x00000200 */
#define ADC_IER_AWD3IE                 ADC_IER_AWD3IE_Msk                      /*!< ADC analog watchdog 3 interrupt */
#define ADC_IER_JQOVFIE_Pos            (10U)                                   
#define ADC_IER_JQOVFIE_Msk            (0x1U << ADC_IER_JQOVFIE_Pos)           /*!< 0x00000400 */
#define ADC_IER_JQOVFIE                ADC_IER_JQOVFIE_Msk                     /*!< ADC group injected contexts queue overflow interrupt */

/* Legacy defines */
#define ADC_IER_RDY             (ADC_IER_ADRDYIE)
#define ADC_IER_EOSMP           (ADC_IER_EOSMPIE)
#define ADC_IER_EOC             (ADC_IER_EOCIE)
#define ADC_IER_EOS             (ADC_IER_EOSIE)
#define ADC_IER_OVR             (ADC_IER_OVRIE)
#define ADC_IER_JEOC            (ADC_IER_JEOCIE)
#define ADC_IER_JEOS            (ADC_IER_JEOSIE)
#define ADC_IER_AWD1            (ADC_IER_AWD1IE)
#define ADC_IER_AWD2            (ADC_IER_AWD2IE)
#define ADC_IER_AWD3            (ADC_IER_AWD3IE)
#define ADC_IER_JQOVF           (ADC_IER_JQOVFIE)

/********************  Bit definition for ADC_CR register  ********************/
#define ADC_CR_ADEN_Pos                (0U)                                    
#define ADC_CR_ADEN_Msk                (0x1U << ADC_CR_ADEN_Pos)               /*!< 0x00000001 */
#define ADC_CR_ADEN                    ADC_CR_ADEN_Msk                         /*!< ADC enable */
#define ADC_CR_ADDIS_Pos               (1U)                                    
#define ADC_CR_ADDIS_Msk               (0x1U << ADC_CR_ADDIS_Pos)              /*!< 0x00000002 */
#define ADC_CR_ADDIS                   ADC_CR_ADDIS_Msk                        /*!< ADC disable */
#define ADC_CR_ADSTART_Pos             (2U)                                    
#define ADC_CR_ADSTART_Msk             (0x1U << ADC_CR_ADSTART_Pos)            /*!< 0x00000004 */
#define ADC_CR_ADSTART                 ADC_CR_ADSTART_Msk                      /*!< ADC group regular conversion start */
#define ADC_CR_JADSTART_Pos            (3U)                                    
#define ADC_CR_JADSTART_Msk            (0x1U << ADC_CR_JADSTART_Pos)           /*!< 0x00000008 */
#define ADC_CR_JADSTART                ADC_CR_JADSTART_Msk                     /*!< ADC group injected conversion start */
#define ADC_CR_ADSTP_Pos               (4U)                                    
#define ADC_CR_ADSTP_Msk               (0x1U << ADC_CR_ADSTP_Pos)              /*!< 0x00000010 */
#define ADC_CR_ADSTP                   ADC_CR_ADSTP_Msk                        /*!< ADC group regular conversion stop */
#define ADC_CR_JADSTP_Pos              (5U)                                    
#define ADC_CR_JADSTP_Msk              (0x1U << ADC_CR_JADSTP_Pos)             /*!< 0x00000020 */
#define ADC_CR_JADSTP                  ADC_CR_JADSTP_Msk                       /*!< ADC group injected conversion stop */
#define ADC_CR_ADVREGEN_Pos            (28U)                                   
#define ADC_CR_ADVREGEN_Msk            (0x3U << ADC_CR_ADVREGEN_Pos)           /*!< 0x30000000 */
#define ADC_CR_ADVREGEN                ADC_CR_ADVREGEN_Msk                     /*!< ADC voltage regulator enable */
#define ADC_CR_ADVREGEN_0              (0x1U << ADC_CR_ADVREGEN_Pos)           /*!< 0x10000000 */
#define ADC_CR_ADVREGEN_1              (0x2U << ADC_CR_ADVREGEN_Pos)           /*!< 0x20000000 */
#define ADC_CR_ADCALDIF_Pos            (30U)                                   
#define ADC_CR_ADCALDIF_Msk            (0x1U << ADC_CR_ADCALDIF_Pos)           /*!< 0x40000000 */
#define ADC_CR_ADCALDIF                ADC_CR_ADCALDIF_Msk                     /*!< ADC differential mode for calibration */
#define ADC_CR_ADCAL_Pos               (31U)                                   
#define ADC_CR_ADCAL_Msk               (0x1U << ADC_CR_ADCAL_Pos)              /*!< 0x80000000 */
#define ADC_CR_ADCAL                   ADC_CR_ADCAL_Msk                        /*!< ADC calibration */

/********************  Bit definition for ADC_CFGR register  ******************/
#define ADC_CFGR_DMAEN_Pos             (0U)                                    
#define ADC_CFGR_DMAEN_Msk             (0x1U << ADC_CFGR_DMAEN_Pos)            /*!< 0x00000001 */
#define ADC_CFGR_DMAEN                 ADC_CFGR_DMAEN_Msk                      /*!< ADC DMA enable */
#define ADC_CFGR_DMACFG_Pos            (1U)                                    
#define ADC_CFGR_DMACFG_Msk            (0x1U << ADC_CFGR_DMACFG_Pos)           /*!< 0x00000002 */
#define ADC_CFGR_DMACFG                ADC_CFGR_DMACFG_Msk                     /*!< ADC DMA configuration */

#define ADC_CFGR_RES_Pos               (3U)                                    
#define ADC_CFGR_RES_Msk               (0x3U << ADC_CFGR_RES_Pos)              /*!< 0x00000018 */
#define ADC_CFGR_RES                   ADC_CFGR_RES_Msk                        /*!< ADC data resolution */
#define ADC_CFGR_RES_0                 (0x1U << ADC_CFGR_RES_Pos)              /*!< 0x00000008 */
#define ADC_CFGR_RES_1                 (0x2U << ADC_CFGR_RES_Pos)              /*!< 0x00000010 */

#define ADC_CFGR_ALIGN_Pos             (5U)                                    
#define ADC_CFGR_ALIGN_Msk             (0x1U << ADC_CFGR_ALIGN_Pos)            /*!< 0x00000020 */
#define ADC_CFGR_ALIGN                 ADC_CFGR_ALIGN_Msk                      /*!< ADC data alignement */

#define ADC_CFGR_EXTSEL_Pos            (6U)                                    
#define ADC_CFGR_EXTSEL_Msk            (0xFU << ADC_CFGR_EXTSEL_Pos)           /*!< 0x000003C0 */
#define ADC_CFGR_EXTSEL                ADC_CFGR_EXTSEL_Msk                     /*!< ADC group regular external trigger source */
#define ADC_CFGR_EXTSEL_0              (0x1U << ADC_CFGR_EXTSEL_Pos)           /*!< 0x00000040 */
#define ADC_CFGR_EXTSEL_1              (0x2U << ADC_CFGR_EXTSEL_Pos)           /*!< 0x00000080 */
#define ADC_CFGR_EXTSEL_2              (0x4U << ADC_CFGR_EXTSEL_Pos)           /*!< 0x00000100 */
#define ADC_CFGR_EXTSEL_3              (0x8U << ADC_CFGR_EXTSEL_Pos)           /*!< 0x00000200 */

#define ADC_CFGR_EXTEN_Pos             (10U)                                   
#define ADC_CFGR_EXTEN_Msk             (0x3U << ADC_CFGR_EXTEN_Pos)            /*!< 0x00000C00 */
#define ADC_CFGR_EXTEN                 ADC_CFGR_EXTEN_Msk                      /*!< ADC group regular external trigger polarity */
#define ADC_CFGR_EXTEN_0               (0x1U << ADC_CFGR_EXTEN_Pos)            /*!< 0x00000400 */
#define ADC_CFGR_EXTEN_1               (0x2U << ADC_CFGR_EXTEN_Pos)            /*!< 0x00000800 */

#define ADC_CFGR_OVRMOD_Pos            (12U)                                   
#define ADC_CFGR_OVRMOD_Msk            (0x1U << ADC_CFGR_OVRMOD_Pos)           /*!< 0x00001000 */
#define ADC_CFGR_OVRMOD                ADC_CFGR_OVRMOD_Msk                     /*!< ADC group regular overrun configuration */
#define ADC_CFGR_CONT_Pos              (13U)                                   
#define ADC_CFGR_CONT_Msk              (0x1U << ADC_CFGR_CONT_Pos)             /*!< 0x00002000 */
#define ADC_CFGR_CONT                  ADC_CFGR_CONT_Msk                       /*!< ADC group regular continuous conversion mode */
#define ADC_CFGR_AUTDLY_Pos            (14U)                                   
#define ADC_CFGR_AUTDLY_Msk            (0x1U << ADC_CFGR_AUTDLY_Pos)           /*!< 0x00004000 */
#define ADC_CFGR_AUTDLY                ADC_CFGR_AUTDLY_Msk                     /*!< ADC low power auto wait */

#define ADC_CFGR_DISCEN_Pos            (16U)                                   
#define ADC_CFGR_DISCEN_Msk            (0x1U << ADC_CFGR_DISCEN_Pos)           /*!< 0x00010000 */
#define ADC_CFGR_DISCEN                ADC_CFGR_DISCEN_Msk                     /*!< ADC group regular sequencer discontinuous mode */

#define ADC_CFGR_DISCNUM_Pos           (17U)                                   
#define ADC_CFGR_DISCNUM_Msk           (0x7U << ADC_CFGR_DISCNUM_Pos)          /*!< 0x000E0000 */
#define ADC_CFGR_DISCNUM               ADC_CFGR_DISCNUM_Msk                    /*!< ADC Discontinuous mode channel count */
#define ADC_CFGR_DISCNUM_0             (0x1U << ADC_CFGR_DISCNUM_Pos)          /*!< 0x00020000 */
#define ADC_CFGR_DISCNUM_1             (0x2U << ADC_CFGR_DISCNUM_Pos)          /*!< 0x00040000 */
#define ADC_CFGR_DISCNUM_2             (0x4U << ADC_CFGR_DISCNUM_Pos)          /*!< 0x00080000 */

#define ADC_CFGR_JDISCEN_Pos           (20U)                                   
#define ADC_CFGR_JDISCEN_Msk           (0x1U << ADC_CFGR_JDISCEN_Pos)          /*!< 0x00100000 */
#define ADC_CFGR_JDISCEN               ADC_CFGR_JDISCEN_Msk                    /*!< ADC Discontinuous mode on injected channels */
#define ADC_CFGR_JQM_Pos               (21U)                                   
#define ADC_CFGR_JQM_Msk               (0x1U << ADC_CFGR_JQM_Pos)              /*!< 0x00200000 */
#define ADC_CFGR_JQM                   ADC_CFGR_JQM_Msk                        /*!< ADC group injected contexts queue mode */
#define ADC_CFGR_AWD1SGL_Pos           (22U)                                   
#define ADC_CFGR_AWD1SGL_Msk           (0x1U << ADC_CFGR_AWD1SGL_Pos)          /*!< 0x00400000 */
#define ADC_CFGR_AWD1SGL               ADC_CFGR_AWD1SGL_Msk                    /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
#define ADC_CFGR_AWD1EN_Pos            (23U)                                   
#define ADC_CFGR_AWD1EN_Msk            (0x1U << ADC_CFGR_AWD1EN_Pos)           /*!< 0x00800000 */
#define ADC_CFGR_AWD1EN                ADC_CFGR_AWD1EN_Msk                     /*!< ADC analog watchdog 1 enable on scope ADC group regular */
#define ADC_CFGR_JAWD1EN_Pos           (24U)                                   
#define ADC_CFGR_JAWD1EN_Msk           (0x1U << ADC_CFGR_JAWD1EN_Pos)          /*!< 0x01000000 */
#define ADC_CFGR_JAWD1EN               ADC_CFGR_JAWD1EN_Msk                    /*!< ADC analog watchdog 1 enable on scope ADC group injected */
#define ADC_CFGR_JAUTO_Pos             (25U)                                   
#define ADC_CFGR_JAUTO_Msk             (0x1U << ADC_CFGR_JAUTO_Pos)            /*!< 0x02000000 */
#define ADC_CFGR_JAUTO                 ADC_CFGR_JAUTO_Msk                      /*!< ADC group injected automatic trigger mode */

#define ADC_CFGR_AWD1CH_Pos            (26U)                                   
#define ADC_CFGR_AWD1CH_Msk            (0x1FU << ADC_CFGR_AWD1CH_Pos)          /*!< 0x7C000000 */
#define ADC_CFGR_AWD1CH                ADC_CFGR_AWD1CH_Msk                     /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CFGR_AWD1CH_0              (0x01U << ADC_CFGR_AWD1CH_Pos)          /*!< 0x04000000 */
#define ADC_CFGR_AWD1CH_1              (0x02U << ADC_CFGR_AWD1CH_Pos)          /*!< 0x08000000 */
#define ADC_CFGR_AWD1CH_2              (0x04U << ADC_CFGR_AWD1CH_Pos)          /*!< 0x10000000 */
#define ADC_CFGR_AWD1CH_3              (0x08U << ADC_CFGR_AWD1CH_Pos)          /*!< 0x20000000 */
#define ADC_CFGR_AWD1CH_4              (0x10U << ADC_CFGR_AWD1CH_Pos)          /*!< 0x40000000 */

/* Legacy defines */
#define ADC_CFGR_AUTOFF_Pos            (15U)                                   
#define ADC_CFGR_AUTOFF_Msk            (0x1U << ADC_CFGR_AUTOFF_Pos)           /*!< 0x00008000 */
#define ADC_CFGR_AUTOFF                ADC_CFGR_AUTOFF_Msk                     /*!< ADC low power auto power off */

/********************  Bit definition for ADC_SMPR1 register  *****************/
#define ADC_SMPR1_SMP0_Pos             (0U)                                    
#define ADC_SMPR1_SMP0_Msk             (0x7U << ADC_SMPR1_SMP0_Pos)            /*!< 0x00000007 */
#define ADC_SMPR1_SMP0                 ADC_SMPR1_SMP0_Msk                      /*!< ADC channel 0 sampling time selection  */
#define ADC_SMPR1_SMP0_0               (0x1U << ADC_SMPR1_SMP0_Pos)            /*!< 0x00000001 */
#define ADC_SMPR1_SMP0_1               (0x2U << ADC_SMPR1_SMP0_Pos)            /*!< 0x00000002 */
#define ADC_SMPR1_SMP0_2               (0x4U << ADC_SMPR1_SMP0_Pos)            /*!< 0x00000004 */

#define ADC_SMPR1_SMP1_Pos             (3U)                                    
#define ADC_SMPR1_SMP1_Msk             (0x7U << ADC_SMPR1_SMP1_Pos)            /*!< 0x00000038 */
#define ADC_SMPR1_SMP1                 ADC_SMPR1_SMP1_Msk                      /*!< ADC channel 1 sampling time selection  */
#define ADC_SMPR1_SMP1_0               (0x1U << ADC_SMPR1_SMP1_Pos)            /*!< 0x00000008 */
#define ADC_SMPR1_SMP1_1               (0x2U << ADC_SMPR1_SMP1_Pos)            /*!< 0x00000010 */
#define ADC_SMPR1_SMP1_2               (0x4U << ADC_SMPR1_SMP1_Pos)            /*!< 0x00000020 */

#define ADC_SMPR1_SMP2_Pos             (6U)                                    
#define ADC_SMPR1_SMP2_Msk             (0x7U << ADC_SMPR1_SMP2_Pos)            /*!< 0x000001C0 */
#define ADC_SMPR1_SMP2                 ADC_SMPR1_SMP2_Msk                      /*!< ADC channel 2 sampling time selection  */
#define ADC_SMPR1_SMP2_0               (0x1U << ADC_SMPR1_SMP2_Pos)            /*!< 0x00000040 */
#define ADC_SMPR1_SMP2_1               (0x2U << ADC_SMPR1_SMP2_Pos)            /*!< 0x00000080 */
#define ADC_SMPR1_SMP2_2               (0x4U << ADC_SMPR1_SMP2_Pos)            /*!< 0x00000100 */

#define ADC_SMPR1_SMP3_Pos             (9U)                                    
#define ADC_SMPR1_SMP3_Msk             (0x7U << ADC_SMPR1_SMP3_Pos)            /*!< 0x00000E00 */
#define ADC_SMPR1_SMP3                 ADC_SMPR1_SMP3_Msk                      /*!< ADC channel 3 sampling time selection  */
#define ADC_SMPR1_SMP3_0               (0x1U << ADC_SMPR1_SMP3_Pos)            /*!< 0x00000200 */
#define ADC_SMPR1_SMP3_1               (0x2U << ADC_SMPR1_SMP3_Pos)            /*!< 0x00000400 */
#define ADC_SMPR1_SMP3_2               (0x4U << ADC_SMPR1_SMP3_Pos)            /*!< 0x00000800 */

#define ADC_SMPR1_SMP4_Pos             (12U)                                   
#define ADC_SMPR1_SMP4_Msk             (0x7U << ADC_SMPR1_SMP4_Pos)            /*!< 0x00007000 */
#define ADC_SMPR1_SMP4                 ADC_SMPR1_SMP4_Msk                      /*!< ADC channel 4 sampling time selection  */
#define ADC_SMPR1_SMP4_0               (0x1U << ADC_SMPR1_SMP4_Pos)            /*!< 0x00001000 */
#define ADC_SMPR1_SMP4_1               (0x2U << ADC_SMPR1_SMP4_Pos)            /*!< 0x00002000 */
#define ADC_SMPR1_SMP4_2               (0x4U << ADC_SMPR1_SMP4_Pos)            /*!< 0x00004000 */

#define ADC_SMPR1_SMP5_Pos             (15U)                                   
#define ADC_SMPR1_SMP5_Msk             (0x7U << ADC_SMPR1_SMP5_Pos)            /*!< 0x00038000 */
#define ADC_SMPR1_SMP5                 ADC_SMPR1_SMP5_Msk                      /*!< ADC channel 5 sampling time selection  */
#define ADC_SMPR1_SMP5_0               (0x1U << ADC_SMPR1_SMP5_Pos)            /*!< 0x00008000 */
#define ADC_SMPR1_SMP5_1               (0x2U << ADC_SMPR1_SMP5_Pos)            /*!< 0x00010000 */
#define ADC_SMPR1_SMP5_2               (0x4U << ADC_SMPR1_SMP5_Pos)            /*!< 0x00020000 */

#define ADC_SMPR1_SMP6_Pos             (18U)                                   
#define ADC_SMPR1_SMP6_Msk             (0x7U << ADC_SMPR1_SMP6_Pos)            /*!< 0x001C0000 */
#define ADC_SMPR1_SMP6                 ADC_SMPR1_SMP6_Msk                      /*!< ADC channel 6 sampling time selection  */
#define ADC_SMPR1_SMP6_0               (0x1U << ADC_SMPR1_SMP6_Pos)            /*!< 0x00040000 */
#define ADC_SMPR1_SMP6_1               (0x2U << ADC_SMPR1_SMP6_Pos)            /*!< 0x00080000 */
#define ADC_SMPR1_SMP6_2               (0x4U << ADC_SMPR1_SMP6_Pos)            /*!< 0x00100000 */

#define ADC_SMPR1_SMP7_Pos             (21U)                                   
#define ADC_SMPR1_SMP7_Msk             (0x7U << ADC_SMPR1_SMP7_Pos)            /*!< 0x00E00000 */
#define ADC_SMPR1_SMP7                 ADC_SMPR1_SMP7_Msk                      /*!< ADC channel 7 sampling time selection  */
#define ADC_SMPR1_SMP7_0               (0x1U << ADC_SMPR1_SMP7_Pos)            /*!< 0x00200000 */
#define ADC_SMPR1_SMP7_1               (0x2U << ADC_SMPR1_SMP7_Pos)            /*!< 0x00400000 */
#define ADC_SMPR1_SMP7_2               (0x4U << ADC_SMPR1_SMP7_Pos)            /*!< 0x00800000 */

#define ADC_SMPR1_SMP8_Pos             (24U)                                   
#define ADC_SMPR1_SMP8_Msk             (0x7U << ADC_SMPR1_SMP8_Pos)            /*!< 0x07000000 */
#define ADC_SMPR1_SMP8                 ADC_SMPR1_SMP8_Msk                      /*!< ADC channel 8 sampling time selection  */
#define ADC_SMPR1_SMP8_0               (0x1U << ADC_SMPR1_SMP8_Pos)            /*!< 0x01000000 */
#define ADC_SMPR1_SMP8_1               (0x2U << ADC_SMPR1_SMP8_Pos)            /*!< 0x02000000 */
#define ADC_SMPR1_SMP8_2               (0x4U << ADC_SMPR1_SMP8_Pos)            /*!< 0x04000000 */

#define ADC_SMPR1_SMP9_Pos             (27U)                                   
#define ADC_SMPR1_SMP9_Msk             (0x7U << ADC_SMPR1_SMP9_Pos)            /*!< 0x38000000 */
#define ADC_SMPR1_SMP9                 ADC_SMPR1_SMP9_Msk                      /*!< ADC channel 9 sampling time selection  */
#define ADC_SMPR1_SMP9_0               (0x1U << ADC_SMPR1_SMP9_Pos)            /*!< 0x08000000 */
#define ADC_SMPR1_SMP9_1               (0x2U << ADC_SMPR1_SMP9_Pos)            /*!< 0x10000000 */
#define ADC_SMPR1_SMP9_2               (0x4U << ADC_SMPR1_SMP9_Pos)            /*!< 0x20000000 */

/********************  Bit definition for ADC_SMPR2 register  *****************/
#define ADC_SMPR2_SMP10_Pos            (0U)                                    
#define ADC_SMPR2_SMP10_Msk            (0x7U << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000007 */
#define ADC_SMPR2_SMP10                ADC_SMPR2_SMP10_Msk                     /*!< ADC channel 10 sampling time selection  */
#define ADC_SMPR2_SMP10_0              (0x1U << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000001 */
#define ADC_SMPR2_SMP10_1              (0x2U << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000002 */
#define ADC_SMPR2_SMP10_2              (0x4U << ADC_SMPR2_SMP10_Pos)           /*!< 0x00000004 */

#define ADC_SMPR2_SMP11_Pos            (3U)                                    
#define ADC_SMPR2_SMP11_Msk            (0x7U << ADC_SMPR2_SMP11_Pos)           /*!< 0x00000038 */
#define ADC_SMPR2_SMP11                ADC_SMPR2_SMP11_Msk                     /*!< ADC channel 11 sampling time selection  */
#define ADC_SMPR2_SMP11_0              (0x1U << ADC_SMPR2_SMP11_Pos)           /*!< 0x00000008 */
#define ADC_SMPR2_SMP11_1              (0x2U << ADC_SMPR2_SMP11_Pos)           /*!< 0x00000010 */
#define ADC_SMPR2_SMP11_2              (0x4U << ADC_SMPR2_SMP11_Pos)           /*!< 0x00000020 */

#define ADC_SMPR2_SMP12_Pos            (6U)                                    
#define ADC_SMPR2_SMP12_Msk            (0x7U << ADC_SMPR2_SMP12_Pos)           /*!< 0x000001C0 */
#define ADC_SMPR2_SMP12                ADC_SMPR2_SMP12_Msk                     /*!< ADC channel 12 sampling time selection  */
#define ADC_SMPR2_SMP12_0              (0x1U << ADC_SMPR2_SMP12_Pos)           /*!< 0x00000040 */
#define ADC_SMPR2_SMP12_1              (0x2U << ADC_SMPR2_SMP12_Pos)           /*!< 0x00000080 */
#define ADC_SMPR2_SMP12_2              (0x4U << ADC_SMPR2_SMP12_Pos)           /*!< 0x00000100 */

#define ADC_SMPR2_SMP13_Pos            (9U)                                    
#define ADC_SMPR2_SMP13_Msk            (0x7U << ADC_SMPR2_SMP13_Pos)           /*!< 0x00000E00 */
#define ADC_SMPR2_SMP13                ADC_SMPR2_SMP13_Msk                     /*!< ADC channel 13 sampling time selection  */
#define ADC_SMPR2_SMP13_0              (0x1U << ADC_SMPR2_SMP13_Pos)           /*!< 0x00000200 */
#define ADC_SMPR2_SMP13_1              (0x2U << ADC_SMPR2_SMP13_Pos)           /*!< 0x00000400 */
#define ADC_SMPR2_SMP13_2              (0x4U << ADC_SMPR2_SMP13_Pos)           /*!< 0x00000800 */

#define ADC_SMPR2_SMP14_Pos            (12U)                                   
#define ADC_SMPR2_SMP14_Msk            (0x7U << ADC_SMPR2_SMP14_Pos)           /*!< 0x00007000 */
#define ADC_SMPR2_SMP14                ADC_SMPR2_SMP14_Msk                     /*!< ADC channel 14 sampling time selection  */
#define ADC_SMPR2_SMP14_0              (0x1U << ADC_SMPR2_SMP14_Pos)           /*!< 0x00001000 */
#define ADC_SMPR2_SMP14_1              (0x2U << ADC_SMPR2_SMP14_Pos)           /*!< 0x00002000 */
#define ADC_SMPR2_SMP14_2              (0x4U << ADC_SMPR2_SMP14_Pos)           /*!< 0x00004000 */

#define ADC_SMPR2_SMP15_Pos            (15U)                                   
#define ADC_SMPR2_SMP15_Msk            (0x7U << ADC_SMPR2_SMP15_Pos)           /*!< 0x00038000 */
#define ADC_SMPR2_SMP15                ADC_SMPR2_SMP15_Msk                     /*!< ADC channel 15 sampling time selection  */
#define ADC_SMPR2_SMP15_0              (0x1U << ADC_SMPR2_SMP15_Pos)           /*!< 0x00008000 */
#define ADC_SMPR2_SMP15_1              (0x2U << ADC_SMPR2_SMP15_Pos)           /*!< 0x00010000 */
#define ADC_SMPR2_SMP15_2              (0x4U << ADC_SMPR2_SMP15_Pos)           /*!< 0x00020000 */

#define ADC_SMPR2_SMP16_Pos            (18U)                                   
#define ADC_SMPR2_SMP16_Msk            (0x7U << ADC_SMPR2_SMP16_Pos)           /*!< 0x001C0000 */
#define ADC_SMPR2_SMP16                ADC_SMPR2_SMP16_Msk                     /*!< ADC channel 16 sampling time selection  */
#define ADC_SMPR2_SMP16_0              (0x1U << ADC_SMPR2_SMP16_Pos)           /*!< 0x00040000 */
#define ADC_SMPR2_SMP16_1              (0x2U << ADC_SMPR2_SMP16_Pos)           /*!< 0x00080000 */
#define ADC_SMPR2_SMP16_2              (0x4U << ADC_SMPR2_SMP16_Pos)           /*!< 0x00100000 */

#define ADC_SMPR2_SMP17_Pos            (21U)                                   
#define ADC_SMPR2_SMP17_Msk            (0x7U << ADC_SMPR2_SMP17_Pos)           /*!< 0x00E00000 */
#define ADC_SMPR2_SMP17                ADC_SMPR2_SMP17_Msk                     /*!< ADC channel 17 sampling time selection  */
#define ADC_SMPR2_SMP17_0              (0x1U << ADC_SMPR2_SMP17_Pos)           /*!< 0x00200000 */
#define ADC_SMPR2_SMP17_1              (0x2U << ADC_SMPR2_SMP17_Pos)           /*!< 0x00400000 */
#define ADC_SMPR2_SMP17_2              (0x4U << ADC_SMPR2_SMP17_Pos)           /*!< 0x00800000 */

#define ADC_SMPR2_SMP18_Pos            (24U)                                   
#define ADC_SMPR2_SMP18_Msk            (0x7U << ADC_SMPR2_SMP18_Pos)           /*!< 0x07000000 */
#define ADC_SMPR2_SMP18                ADC_SMPR2_SMP18_Msk                     /*!< ADC channel 18 sampling time selection  */
#define ADC_SMPR2_SMP18_0              (0x1U << ADC_SMPR2_SMP18_Pos)           /*!< 0x01000000 */
#define ADC_SMPR2_SMP18_1              (0x2U << ADC_SMPR2_SMP18_Pos)           /*!< 0x02000000 */
#define ADC_SMPR2_SMP18_2              (0x4U << ADC_SMPR2_SMP18_Pos)           /*!< 0x04000000 */

/********************  Bit definition for ADC_TR1 register  *******************/
#define ADC_TR1_LT1_Pos                (0U)                                    
#define ADC_TR1_LT1_Msk                (0xFFFU << ADC_TR1_LT1_Pos)             /*!< 0x00000FFF */
#define ADC_TR1_LT1                    ADC_TR1_LT1_Msk                         /*!< ADC analog watchdog 1 threshold low */
#define ADC_TR1_LT1_0                  (0x001U << ADC_TR1_LT1_Pos)             /*!< 0x00000001 */
#define ADC_TR1_LT1_1                  (0x002U << ADC_TR1_LT1_Pos)             /*!< 0x00000002 */
#define ADC_TR1_LT1_2                  (0x004U << ADC_TR1_LT1_Pos)             /*!< 0x00000004 */
#define ADC_TR1_LT1_3                  (0x008U << ADC_TR1_LT1_Pos)             /*!< 0x00000008 */
#define ADC_TR1_LT1_4                  (0x010U << ADC_TR1_LT1_Pos)             /*!< 0x00000010 */
#define ADC_TR1_LT1_5                  (0x020U << ADC_TR1_LT1_Pos)             /*!< 0x00000020 */
#define ADC_TR1_LT1_6                  (0x040U << ADC_TR1_LT1_Pos)             /*!< 0x00000040 */
#define ADC_TR1_LT1_7                  (0x080U << ADC_TR1_LT1_Pos)             /*!< 0x00000080 */
#define ADC_TR1_LT1_8                  (0x100U << ADC_TR1_LT1_Pos)             /*!< 0x00000100 */
#define ADC_TR1_LT1_9                  (0x200U << ADC_TR1_LT1_Pos)             /*!< 0x00000200 */
#define ADC_TR1_LT1_10                 (0x400U << ADC_TR1_LT1_Pos)             /*!< 0x00000400 */
#define ADC_TR1_LT1_11                 (0x800U << ADC_TR1_LT1_Pos)             /*!< 0x00000800 */

#define ADC_TR1_HT1_Pos                (16U)                                   
#define ADC_TR1_HT1_Msk                (0xFFFU << ADC_TR1_HT1_Pos)             /*!< 0x0FFF0000 */
#define ADC_TR1_HT1                    ADC_TR1_HT1_Msk                         /*!< ADC Analog watchdog 1 threshold high */
#define ADC_TR1_HT1_0                  (0x001U << ADC_TR1_HT1_Pos)             /*!< 0x00010000 */
#define ADC_TR1_HT1_1                  (0x002U << ADC_TR1_HT1_Pos)             /*!< 0x00020000 */
#define ADC_TR1_HT1_2                  (0x004U << ADC_TR1_HT1_Pos)             /*!< 0x00040000 */
#define ADC_TR1_HT1_3                  (0x008U << ADC_TR1_HT1_Pos)             /*!< 0x00080000 */
#define ADC_TR1_HT1_4                  (0x010U << ADC_TR1_HT1_Pos)             /*!< 0x00100000 */
#define ADC_TR1_HT1_5                  (0x020U << ADC_TR1_HT1_Pos)             /*!< 0x00200000 */
#define ADC_TR1_HT1_6                  (0x040U << ADC_TR1_HT1_Pos)             /*!< 0x00400000 */
#define ADC_TR1_HT1_7                  (0x080U << ADC_TR1_HT1_Pos)             /*!< 0x00800000 */
#define ADC_TR1_HT1_8                  (0x100U << ADC_TR1_HT1_Pos)             /*!< 0x01000000 */
#define ADC_TR1_HT1_9                  (0x200U << ADC_TR1_HT1_Pos)             /*!< 0x02000000 */
#define ADC_TR1_HT1_10                 (0x400U << ADC_TR1_HT1_Pos)             /*!< 0x04000000 */
#define ADC_TR1_HT1_11                 (0x800U << ADC_TR1_HT1_Pos)             /*!< 0x08000000 */

/********************  Bit definition for ADC_TR2 register  *******************/
#define ADC_TR2_LT2_Pos                (0U)                                    
#define ADC_TR2_LT2_Msk                (0xFFU << ADC_TR2_LT2_Pos)              /*!< 0x000000FF */
#define ADC_TR2_LT2                    ADC_TR2_LT2_Msk                         /*!< ADC analog watchdog 2 threshold low */
#define ADC_TR2_LT2_0                  (0x01U << ADC_TR2_LT2_Pos)              /*!< 0x00000001 */
#define ADC_TR2_LT2_1                  (0x02U << ADC_TR2_LT2_Pos)              /*!< 0x00000002 */
#define ADC_TR2_LT2_2                  (0x04U << ADC_TR2_LT2_Pos)              /*!< 0x00000004 */
#define ADC_TR2_LT2_3                  (0x08U << ADC_TR2_LT2_Pos)              /*!< 0x00000008 */
#define ADC_TR2_LT2_4                  (0x10U << ADC_TR2_LT2_Pos)              /*!< 0x00000010 */
#define ADC_TR2_LT2_5                  (0x20U << ADC_TR2_LT2_Pos)              /*!< 0x00000020 */
#define ADC_TR2_LT2_6                  (0x40U << ADC_TR2_LT2_Pos)              /*!< 0x00000040 */
#define ADC_TR2_LT2_7                  (0x80U << ADC_TR2_LT2_Pos)              /*!< 0x00000080 */

#define ADC_TR2_HT2_Pos                (16U)                                   
#define ADC_TR2_HT2_Msk                (0xFFU << ADC_TR2_HT2_Pos)              /*!< 0x00FF0000 */
#define ADC_TR2_HT2                    ADC_TR2_HT2_Msk                         /*!< ADC analog watchdog 2 threshold high */
#define ADC_TR2_HT2_0                  (0x01U << ADC_TR2_HT2_Pos)              /*!< 0x00010000 */
#define ADC_TR2_HT2_1                  (0x02U << ADC_TR2_HT2_Pos)              /*!< 0x00020000 */
#define ADC_TR2_HT2_2                  (0x04U << ADC_TR2_HT2_Pos)              /*!< 0x00040000 */
#define ADC_TR2_HT2_3                  (0x08U << ADC_TR2_HT2_Pos)              /*!< 0x00080000 */
#define ADC_TR2_HT2_4                  (0x10U << ADC_TR2_HT2_Pos)              /*!< 0x00100000 */
#define ADC_TR2_HT2_5                  (0x20U << ADC_TR2_HT2_Pos)              /*!< 0x00200000 */
#define ADC_TR2_HT2_6                  (0x40U << ADC_TR2_HT2_Pos)              /*!< 0x00400000 */
#define ADC_TR2_HT2_7                  (0x80U << ADC_TR2_HT2_Pos)              /*!< 0x00800000 */

/********************  Bit definition for ADC_TR3 register  *******************/
#define ADC_TR3_LT3_Pos                (0U)                                    
#define ADC_TR3_LT3_Msk                (0xFFU << ADC_TR3_LT3_Pos)              /*!< 0x000000FF */
#define ADC_TR3_LT3                    ADC_TR3_LT3_Msk                         /*!< ADC analog watchdog 3 threshold low */
#define ADC_TR3_LT3_0                  (0x01U << ADC_TR3_LT3_Pos)              /*!< 0x00000001 */
#define ADC_TR3_LT3_1                  (0x02U << ADC_TR3_LT3_Pos)              /*!< 0x00000002 */
#define ADC_TR3_LT3_2                  (0x04U << ADC_TR3_LT3_Pos)              /*!< 0x00000004 */
#define ADC_TR3_LT3_3                  (0x08U << ADC_TR3_LT3_Pos)              /*!< 0x00000008 */
#define ADC_TR3_LT3_4                  (0x10U << ADC_TR3_LT3_Pos)              /*!< 0x00000010 */
#define ADC_TR3_LT3_5                  (0x20U << ADC_TR3_LT3_Pos)              /*!< 0x00000020 */
#define ADC_TR3_LT3_6                  (0x40U << ADC_TR3_LT3_Pos)              /*!< 0x00000040 */
#define ADC_TR3_LT3_7                  (0x80U << ADC_TR3_LT3_Pos)              /*!< 0x00000080 */

#define ADC_TR3_HT3_Pos                (16U)                                   
#define ADC_TR3_HT3_Msk                (0xFFU << ADC_TR3_HT3_Pos)              /*!< 0x00FF0000 */
#define ADC_TR3_HT3                    ADC_TR3_HT3_Msk                         /*!< ADC analog watchdog 3 threshold high */
#define ADC_TR3_HT3_0                  (0x01U << ADC_TR3_HT3_Pos)              /*!< 0x00010000 */
#define ADC_TR3_HT3_1                  (0x02U << ADC_TR3_HT3_Pos)              /*!< 0x00020000 */
#define ADC_TR3_HT3_2                  (0x04U << ADC_TR3_HT3_Pos)              /*!< 0x00040000 */
#define ADC_TR3_HT3_3                  (0x08U << ADC_TR3_HT3_Pos)              /*!< 0x00080000 */
#define ADC_TR3_HT3_4                  (0x10U << ADC_TR3_HT3_Pos)              /*!< 0x00100000 */
#define ADC_TR3_HT3_5                  (0x20U << ADC_TR3_HT3_Pos)              /*!< 0x00200000 */
#define ADC_TR3_HT3_6                  (0x40U << ADC_TR3_HT3_Pos)              /*!< 0x00400000 */
#define ADC_TR3_HT3_7                  (0x80U << ADC_TR3_HT3_Pos)              /*!< 0x00800000 */

/********************  Bit definition for ADC_SQR1 register  ******************/
#define ADC_SQR1_L_Pos                 (0U)                                    
#define ADC_SQR1_L_Msk                 (0xFU << ADC_SQR1_L_Pos)                /*!< 0x0000000F */
#define ADC_SQR1_L                     ADC_SQR1_L_Msk                          /*!< ADC group regular sequencer scan length */
#define ADC_SQR1_L_0                   (0x1U << ADC_SQR1_L_Pos)                /*!< 0x00000001 */
#define ADC_SQR1_L_1                   (0x2U << ADC_SQR1_L_Pos)                /*!< 0x00000002 */
#define ADC_SQR1_L_2                   (0x4U << ADC_SQR1_L_Pos)                /*!< 0x00000004 */
#define ADC_SQR1_L_3                   (0x8U << ADC_SQR1_L_Pos)                /*!< 0x00000008 */

#define ADC_SQR1_SQ1_Pos               (6U)                                    
#define ADC_SQR1_SQ1_Msk               (0x1FU << ADC_SQR1_SQ1_Pos)             /*!< 0x000007C0 */
#define ADC_SQR1_SQ1                   ADC_SQR1_SQ1_Msk                        /*!< ADC group regular sequencer rank 1 */
#define ADC_SQR1_SQ1_0                 (0x01U << ADC_SQR1_SQ1_Pos)             /*!< 0x00000040 */
#define ADC_SQR1_SQ1_1                 (0x02U << ADC_SQR1_SQ1_Pos)             /*!< 0x00000080 */
#define ADC_SQR1_SQ1_2                 (0x04U << ADC_SQR1_SQ1_Pos)             /*!< 0x00000100 */
#define ADC_SQR1_SQ1_3                 (0x08U << ADC_SQR1_SQ1_Pos)             /*!< 0x00000200 */
#define ADC_SQR1_SQ1_4                 (0x10U << ADC_SQR1_SQ1_Pos)             /*!< 0x00000400 */

#define ADC_SQR1_SQ2_Pos               (12U)                                   
#define ADC_SQR1_SQ2_Msk               (0x1FU << ADC_SQR1_SQ2_Pos)             /*!< 0x0001F000 */
#define ADC_SQR1_SQ2                   ADC_SQR1_SQ2_Msk                        /*!< ADC group regular sequencer rank 2 */
#define ADC_SQR1_SQ2_0                 (0x01U << ADC_SQR1_SQ2_Pos)             /*!< 0x00001000 */
#define ADC_SQR1_SQ2_1                 (0x02U << ADC_SQR1_SQ2_Pos)             /*!< 0x00002000 */
#define ADC_SQR1_SQ2_2                 (0x04U << ADC_SQR1_SQ2_Pos)             /*!< 0x00004000 */
#define ADC_SQR1_SQ2_3                 (0x08U << ADC_SQR1_SQ2_Pos)             /*!< 0x00008000 */
#define ADC_SQR1_SQ2_4                 (0x10U << ADC_SQR1_SQ2_Pos)             /*!< 0x00010000 */

#define ADC_SQR1_SQ3_Pos               (18U)                                   
#define ADC_SQR1_SQ3_Msk               (0x1FU << ADC_SQR1_SQ3_Pos)             /*!< 0x007C0000 */
#define ADC_SQR1_SQ3                   ADC_SQR1_SQ3_Msk                        /*!< ADC group regular sequencer rank 3 */
#define ADC_SQR1_SQ3_0                 (0x01U << ADC_SQR1_SQ3_Pos)             /*!< 0x00040000 */
#define ADC_SQR1_SQ3_1                 (0x02U << ADC_SQR1_SQ3_Pos)             /*!< 0x00080000 */
#define ADC_SQR1_SQ3_2                 (0x04U << ADC_SQR1_SQ3_Pos)             /*!< 0x00100000 */
#define ADC_SQR1_SQ3_3                 (0x08U << ADC_SQR1_SQ3_Pos)             /*!< 0x00200000 */
#define ADC_SQR1_SQ3_4                 (0x10U << ADC_SQR1_SQ3_Pos)             /*!< 0x00400000 */

#define ADC_SQR1_SQ4_Pos               (24U)                                   
#define ADC_SQR1_SQ4_Msk               (0x1FU << ADC_SQR1_SQ4_Pos)             /*!< 0x1F000000 */
#define ADC_SQR1_SQ4                   ADC_SQR1_SQ4_Msk                        /*!< ADC group regular sequencer rank 4 */
#define ADC_SQR1_SQ4_0                 (0x01U << ADC_SQR1_SQ4_Pos)             /*!< 0x01000000 */
#define ADC_SQR1_SQ4_1                 (0x02U << ADC_SQR1_SQ4_Pos)             /*!< 0x02000000 */
#define ADC_SQR1_SQ4_2                 (0x04U << ADC_SQR1_SQ4_Pos)             /*!< 0x04000000 */
#define ADC_SQR1_SQ4_3                 (0x08U << ADC_SQR1_SQ4_Pos)             /*!< 0x08000000 */
#define ADC_SQR1_SQ4_4                 (0x10U << ADC_SQR1_SQ4_Pos)             /*!< 0x10000000 */

/********************  Bit definition for ADC_SQR2 register  ******************/
#define ADC_SQR2_SQ5_Pos               (0U)                                    
#define ADC_SQR2_SQ5_Msk               (0x1FU << ADC_SQR2_SQ5_Pos)             /*!< 0x0000001F */
#define ADC_SQR2_SQ5                   ADC_SQR2_SQ5_Msk                        /*!< ADC group regular sequencer rank 5 */
#define ADC_SQR2_SQ5_0                 (0x01U << ADC_SQR2_SQ5_Pos)             /*!< 0x00000001 */
#define ADC_SQR2_SQ5_1                 (0x02U << ADC_SQR2_SQ5_Pos)             /*!< 0x00000002 */
#define ADC_SQR2_SQ5_2                 (0x04U << ADC_SQR2_SQ5_Pos)             /*!< 0x00000004 */
#define ADC_SQR2_SQ5_3                 (0x08U << ADC_SQR2_SQ5_Pos)             /*!< 0x00000008 */
#define ADC_SQR2_SQ5_4                 (0x10U << ADC_SQR2_SQ5_Pos)             /*!< 0x00000010 */

#define ADC_SQR2_SQ6_Pos               (6U)                                    
#define ADC_SQR2_SQ6_Msk               (0x1FU << ADC_SQR2_SQ6_Pos)             /*!< 0x000007C0 */
#define ADC_SQR2_SQ6                   ADC_SQR2_SQ6_Msk                        /*!< ADC group regular sequencer rank 6 */
#define ADC_SQR2_SQ6_0                 (0x01U << ADC_SQR2_SQ6_Pos)             /*!< 0x00000040 */
#define ADC_SQR2_SQ6_1                 (0x02U << ADC_SQR2_SQ6_Pos)             /*!< 0x00000080 */
#define ADC_SQR2_SQ6_2                 (0x04U << ADC_SQR2_SQ6_Pos)             /*!< 0x00000100 */
#define ADC_SQR2_SQ6_3                 (0x08U << ADC_SQR2_SQ6_Pos)             /*!< 0x00000200 */
#define ADC_SQR2_SQ6_4                 (0x10U << ADC_SQR2_SQ6_Pos)             /*!< 0x00000400 */

#define ADC_SQR2_SQ7_Pos               (12U)                                   
#define ADC_SQR2_SQ7_Msk               (0x1FU << ADC_SQR2_SQ7_Pos)             /*!< 0x0001F000 */
#define ADC_SQR2_SQ7                   ADC_SQR2_SQ7_Msk                        /*!< ADC group regular sequencer rank 7 */
#define ADC_SQR2_SQ7_0                 (0x01U << ADC_SQR2_SQ7_Pos)             /*!< 0x00001000 */
#define ADC_SQR2_SQ7_1                 (0x02U << ADC_SQR2_SQ7_Pos)             /*!< 0x00002000 */
#define ADC_SQR2_SQ7_2                 (0x04U << ADC_SQR2_SQ7_Pos)             /*!< 0x00004000 */
#define ADC_SQR2_SQ7_3                 (0x08U << ADC_SQR2_SQ7_Pos)             /*!< 0x00008000 */
#define ADC_SQR2_SQ7_4                 (0x10U << ADC_SQR2_SQ7_Pos)             /*!< 0x00010000 */

#define ADC_SQR2_SQ8_Pos               (18U)                                   
#define ADC_SQR2_SQ8_Msk               (0x1FU << ADC_SQR2_SQ8_Pos)             /*!< 0x007C0000 */
#define ADC_SQR2_SQ8                   ADC_SQR2_SQ8_Msk                        /*!< ADC group regular sequencer rank 8 */
#define ADC_SQR2_SQ8_0                 (0x01U << ADC_SQR2_SQ8_Pos)             /*!< 0x00040000 */
#define ADC_SQR2_SQ8_1                 (0x02U << ADC_SQR2_SQ8_Pos)             /*!< 0x00080000 */
#define ADC_SQR2_SQ8_2                 (0x04U << ADC_SQR2_SQ8_Pos)             /*!< 0x00100000 */
#define ADC_SQR2_SQ8_3                 (0x08U << ADC_SQR2_SQ8_Pos)             /*!< 0x00200000 */
#define ADC_SQR2_SQ8_4                 (0x10U << ADC_SQR2_SQ8_Pos)             /*!< 0x00400000 */

#define ADC_SQR2_SQ9_Pos               (24U)                                   
#define ADC_SQR2_SQ9_Msk               (0x1FU << ADC_SQR2_SQ9_Pos)             /*!< 0x1F000000 */
#define ADC_SQR2_SQ9                   ADC_SQR2_SQ9_Msk                        /*!< ADC group regular sequencer rank 9 */
#define ADC_SQR2_SQ9_0                 (0x01U << ADC_SQR2_SQ9_Pos)             /*!< 0x01000000 */
#define ADC_SQR2_SQ9_1                 (0x02U << ADC_SQR2_SQ9_Pos)             /*!< 0x02000000 */
#define ADC_SQR2_SQ9_2                 (0x04U << ADC_SQR2_SQ9_Pos)             /*!< 0x04000000 */
#define ADC_SQR2_SQ9_3                 (0x08U << ADC_SQR2_SQ9_Pos)             /*!< 0x08000000 */
#define ADC_SQR2_SQ9_4                 (0x10U << ADC_SQR2_SQ9_Pos)             /*!< 0x10000000 */

/********************  Bit definition for ADC_SQR3 register  ******************/
#define ADC_SQR3_SQ10_Pos              (0U)                                    
#define ADC_SQR3_SQ10_Msk              (0x1FU << ADC_SQR3_SQ10_Pos)            /*!< 0x0000001F */
#define ADC_SQR3_SQ10                  ADC_SQR3_SQ10_Msk                       /*!< ADC group regular sequencer rank 10 */
#define ADC_SQR3_SQ10_0                (0x01U << ADC_SQR3_SQ10_Pos)            /*!< 0x00000001 */
#define ADC_SQR3_SQ10_1                (0x02U << ADC_SQR3_SQ10_Pos)            /*!< 0x00000002 */
#define ADC_SQR3_SQ10_2                (0x04U << ADC_SQR3_SQ10_Pos)            /*!< 0x00000004 */
#define ADC_SQR3_SQ10_3                (0x08U << ADC_SQR3_SQ10_Pos)            /*!< 0x00000008 */
#define ADC_SQR3_SQ10_4                (0x10U << ADC_SQR3_SQ10_Pos)            /*!< 0x00000010 */

#define ADC_SQR3_SQ11_Pos              (6U)                                    
#define ADC_SQR3_SQ11_Msk              (0x1FU << ADC_SQR3_SQ11_Pos)            /*!< 0x000007C0 */
#define ADC_SQR3_SQ11                  ADC_SQR3_SQ11_Msk                       /*!< ADC group regular sequencer rank 11 */
#define ADC_SQR3_SQ11_0                (0x01U << ADC_SQR3_SQ11_Pos)            /*!< 0x00000040 */
#define ADC_SQR3_SQ11_1                (0x02U << ADC_SQR3_SQ11_Pos)            /*!< 0x00000080 */
#define ADC_SQR3_SQ11_2                (0x04U << ADC_SQR3_SQ11_Pos)            /*!< 0x00000100 */
#define ADC_SQR3_SQ11_3                (0x08U << ADC_SQR3_SQ11_Pos)            /*!< 0x00000200 */
#define ADC_SQR3_SQ11_4                (0x10U << ADC_SQR3_SQ11_Pos)            /*!< 0x00000400 */

#define ADC_SQR3_SQ12_Pos              (12U)                                   
#define ADC_SQR3_SQ12_Msk              (0x1FU << ADC_SQR3_SQ12_Pos)            /*!< 0x0001F000 */
#define ADC_SQR3_SQ12                  ADC_SQR3_SQ12_Msk                       /*!< ADC group regular sequencer rank 12 */
#define ADC_SQR3_SQ12_0                (0x01U << ADC_SQR3_SQ12_Pos)            /*!< 0x00001000 */
#define ADC_SQR3_SQ12_1                (0x02U << ADC_SQR3_SQ12_Pos)            /*!< 0x00002000 */
#define ADC_SQR3_SQ12_2                (0x04U << ADC_SQR3_SQ12_Pos)            /*!< 0x00004000 */
#define ADC_SQR3_SQ12_3                (0x08U << ADC_SQR3_SQ12_Pos)            /*!< 0x00008000 */
#define ADC_SQR3_SQ12_4                (0x10U << ADC_SQR3_SQ12_Pos)            /*!< 0x00010000 */

#define ADC_SQR3_SQ13_Pos              (18U)                                   
#define ADC_SQR3_SQ13_Msk              (0x1FU << ADC_SQR3_SQ13_Pos)            /*!< 0x007C0000 */
#define ADC_SQR3_SQ13                  ADC_SQR3_SQ13_Msk                       /*!< ADC group regular sequencer rank 13 */
#define ADC_SQR3_SQ13_0                (0x01U << ADC_SQR3_SQ13_Pos)            /*!< 0x00040000 */
#define ADC_SQR3_SQ13_1                (0x02U << ADC_SQR3_SQ13_Pos)            /*!< 0x00080000 */
#define ADC_SQR3_SQ13_2                (0x04U << ADC_SQR3_SQ13_Pos)            /*!< 0x00100000 */
#define ADC_SQR3_SQ13_3                (0x08U << ADC_SQR3_SQ13_Pos)            /*!< 0x00200000 */
#define ADC_SQR3_SQ13_4                (0x10U << ADC_SQR3_SQ13_Pos)            /*!< 0x00400000 */

#define ADC_SQR3_SQ14_Pos              (24U)                                   
#define ADC_SQR3_SQ14_Msk              (0x1FU << ADC_SQR3_SQ14_Pos)            /*!< 0x1F000000 */
#define ADC_SQR3_SQ14                  ADC_SQR3_SQ14_Msk                       /*!< ADC group regular sequencer rank 14 */
#define ADC_SQR3_SQ14_0                (0x01U << ADC_SQR3_SQ14_Pos)            /*!< 0x01000000 */
#define ADC_SQR3_SQ14_1                (0x02U << ADC_SQR3_SQ14_Pos)            /*!< 0x02000000 */
#define ADC_SQR3_SQ14_2                (0x04U << ADC_SQR3_SQ14_Pos)            /*!< 0x04000000 */
#define ADC_SQR3_SQ14_3                (0x08U << ADC_SQR3_SQ14_Pos)            /*!< 0x08000000 */
#define ADC_SQR3_SQ14_4                (0x10U << ADC_SQR3_SQ14_Pos)            /*!< 0x10000000 */

/********************  Bit definition for ADC_SQR4 register  ******************/
#define ADC_SQR4_SQ15_Pos              (0U)                                    
#define ADC_SQR4_SQ15_Msk              (0x1FU << ADC_SQR4_SQ15_Pos)            /*!< 0x0000001F */
#define ADC_SQR4_SQ15                  ADC_SQR4_SQ15_Msk                       /*!< ADC group regular sequencer rank 15 */
#define ADC_SQR4_SQ15_0                (0x01U << ADC_SQR4_SQ15_Pos)            /*!< 0x00000001 */
#define ADC_SQR4_SQ15_1                (0x02U << ADC_SQR4_SQ15_Pos)            /*!< 0x00000002 */
#define ADC_SQR4_SQ15_2                (0x04U << ADC_SQR4_SQ15_Pos)            /*!< 0x00000004 */
#define ADC_SQR4_SQ15_3                (0x08U << ADC_SQR4_SQ15_Pos)            /*!< 0x00000008 */
#define ADC_SQR4_SQ15_4                (0x10U << ADC_SQR4_SQ15_Pos)            /*!< 0x00000010 */

#define ADC_SQR4_SQ16_Pos              (6U)                                    
#define ADC_SQR4_SQ16_Msk              (0x1FU << ADC_SQR4_SQ16_Pos)            /*!< 0x000007C0 */
#define ADC_SQR4_SQ16                  ADC_SQR4_SQ16_Msk                       /*!< ADC group regular sequencer rank 16 */
#define ADC_SQR4_SQ16_0                (0x01U << ADC_SQR4_SQ16_Pos)            /*!< 0x00000040 */
#define ADC_SQR4_SQ16_1                (0x02U << ADC_SQR4_SQ16_Pos)            /*!< 0x00000080 */
#define ADC_SQR4_SQ16_2                (0x04U << ADC_SQR4_SQ16_Pos)            /*!< 0x00000100 */
#define ADC_SQR4_SQ16_3                (0x08U << ADC_SQR4_SQ16_Pos)            /*!< 0x00000200 */
#define ADC_SQR4_SQ16_4                (0x10U << ADC_SQR4_SQ16_Pos)            /*!< 0x00000400 */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_RDATA_Pos               (0U)                                    
#define ADC_DR_RDATA_Msk               (0xFFFFU << ADC_DR_RDATA_Pos)           /*!< 0x0000FFFF */
#define ADC_DR_RDATA                   ADC_DR_RDATA_Msk                        /*!< ADC group regular conversion data */
#define ADC_DR_RDATA_0                 (0x0001U << ADC_DR_RDATA_Pos)           /*!< 0x00000001 */
#define ADC_DR_RDATA_1                 (0x0002U << ADC_DR_RDATA_Pos)           /*!< 0x00000002 */
#define ADC_DR_RDATA_2                 (0x0004U << ADC_DR_RDATA_Pos)           /*!< 0x00000004 */
#define ADC_DR_RDATA_3                 (0x0008U << ADC_DR_RDATA_Pos)           /*!< 0x00000008 */
#define ADC_DR_RDATA_4                 (0x0010U << ADC_DR_RDATA_Pos)           /*!< 0x00000010 */
#define ADC_DR_RDATA_5                 (0x0020U << ADC_DR_RDATA_Pos)           /*!< 0x00000020 */
#define ADC_DR_RDATA_6                 (0x0040U << ADC_DR_RDATA_Pos)           /*!< 0x00000040 */
#define ADC_DR_RDATA_7                 (0x0080U << ADC_DR_RDATA_Pos)           /*!< 0x00000080 */
#define ADC_DR_RDATA_8                 (0x0100U << ADC_DR_RDATA_Pos)           /*!< 0x00000100 */
#define ADC_DR_RDATA_9                 (0x0200U << ADC_DR_RDATA_Pos)           /*!< 0x00000200 */
#define ADC_DR_RDATA_10                (0x0400U << ADC_DR_RDATA_Pos)           /*!< 0x00000400 */
#define ADC_DR_RDATA_11                (0x0800U << ADC_DR_RDATA_Pos)           /*!< 0x00000800 */
#define ADC_DR_RDATA_12                (0x1000U << ADC_DR_RDATA_Pos)           /*!< 0x00001000 */
#define ADC_DR_RDATA_13                (0x2000U << ADC_DR_RDATA_Pos)           /*!< 0x00002000 */
#define ADC_DR_RDATA_14                (0x4000U << ADC_DR_RDATA_Pos)           /*!< 0x00004000 */
#define ADC_DR_RDATA_15                (0x8000U << ADC_DR_RDATA_Pos)           /*!< 0x00008000 */

/********************  Bit definition for ADC_JSQR register  ******************/
#define ADC_JSQR_JL_Pos                (0U)                                    
#define ADC_JSQR_JL_Msk                (0x3U << ADC_JSQR_JL_Pos)               /*!< 0x00000003 */
#define ADC_JSQR_JL                    ADC_JSQR_JL_Msk                         /*!< ADC group injected sequencer scan length */
#define ADC_JSQR_JL_0                  (0x1U << ADC_JSQR_JL_Pos)               /*!< 0x00000001 */
#define ADC_JSQR_JL_1                  (0x2U << ADC_JSQR_JL_Pos)               /*!< 0x00000002 */

#define ADC_JSQR_JEXTSEL_Pos           (2U)                                    
#define ADC_JSQR_JEXTSEL_Msk           (0xFU << ADC_JSQR_JEXTSEL_Pos)          /*!< 0x0000003C */
#define ADC_JSQR_JEXTSEL               ADC_JSQR_JEXTSEL_Msk                    /*!< ADC group injected external trigger source */
#define ADC_JSQR_JEXTSEL_0             (0x1U << ADC_JSQR_JEXTSEL_Pos)          /*!< 0x00000004 */
#define ADC_JSQR_JEXTSEL_1             (0x2U << ADC_JSQR_JEXTSEL_Pos)          /*!< 0x00000008 */
#define ADC_JSQR_JEXTSEL_2             (0x4U << ADC_JSQR_JEXTSEL_Pos)          /*!< 0x00000010 */
#define ADC_JSQR_JEXTSEL_3             (0x8U << ADC_JSQR_JEXTSEL_Pos)          /*!< 0x00000020 */

#define ADC_JSQR_JEXTEN_Pos            (6U)                                    
#define ADC_JSQR_JEXTEN_Msk            (0x3U << ADC_JSQR_JEXTEN_Pos)           /*!< 0x000000C0 */
#define ADC_JSQR_JEXTEN                ADC_JSQR_JEXTEN_Msk                     /*!< ADC group injected external trigger polarity */
#define ADC_JSQR_JEXTEN_0              (0x1U << ADC_JSQR_JEXTEN_Pos)           /*!< 0x00000040 */
#define ADC_JSQR_JEXTEN_1              (0x2U << ADC_JSQR_JEXTEN_Pos)           /*!< 0x00000080 */

#define ADC_JSQR_JSQ1_Pos              (8U)                                    
#define ADC_JSQR_JSQ1_Msk              (0x1FU << ADC_JSQR_JSQ1_Pos)            /*!< 0x00001F00 */
#define ADC_JSQR_JSQ1                  ADC_JSQR_JSQ1_Msk                       /*!< ADC group injected sequencer rank 1 */
#define ADC_JSQR_JSQ1_0                (0x01U << ADC_JSQR_JSQ1_Pos)            /*!< 0x00000100 */
#define ADC_JSQR_JSQ1_1                (0x02U << ADC_JSQR_JSQ1_Pos)            /*!< 0x00000200 */
#define ADC_JSQR_JSQ1_2                (0x04U << ADC_JSQR_JSQ1_Pos)            /*!< 0x00000400 */
#define ADC_JSQR_JSQ1_3                (0x08U << ADC_JSQR_JSQ1_Pos)            /*!< 0x00000800 */
#define ADC_JSQR_JSQ1_4                (0x10U << ADC_JSQR_JSQ1_Pos)            /*!< 0x00001000 */

#define ADC_JSQR_JSQ2_Pos              (14U)                                   
#define ADC_JSQR_JSQ2_Msk              (0x1FU << ADC_JSQR_JSQ2_Pos)            /*!< 0x0007C000 */
#define ADC_JSQR_JSQ2                  ADC_JSQR_JSQ2_Msk                       /*!< ADC group injected sequencer rank 2 */
#define ADC_JSQR_JSQ2_0                (0x01U << ADC_JSQR_JSQ2_Pos)            /*!< 0x00004000 */
#define ADC_JSQR_JSQ2_1                (0x02U << ADC_JSQR_JSQ2_Pos)            /*!< 0x00008000 */
#define ADC_JSQR_JSQ2_2                (0x04U << ADC_JSQR_JSQ2_Pos)            /*!< 0x00010000 */
#define ADC_JSQR_JSQ2_3                (0x08U << ADC_JSQR_JSQ2_Pos)            /*!< 0x00020000 */
#define ADC_JSQR_JSQ2_4                (0x10U << ADC_JSQR_JSQ2_Pos)            /*!< 0x00040000 */

#define ADC_JSQR_JSQ3_Pos              (20U)                                   
#define ADC_JSQR_JSQ3_Msk              (0x1FU << ADC_JSQR_JSQ3_Pos)            /*!< 0x01F00000 */
#define ADC_JSQR_JSQ3                  ADC_JSQR_JSQ3_Msk                       /*!< ADC group injected sequencer rank 3 */
#define ADC_JSQR_JSQ3_0                (0x01U << ADC_JSQR_JSQ3_Pos)            /*!< 0x00100000 */
#define ADC_JSQR_JSQ3_1                (0x02U << ADC_JSQR_JSQ3_Pos)            /*!< 0x00200000 */
#define ADC_JSQR_JSQ3_2                (0x04U << ADC_JSQR_JSQ3_Pos)            /*!< 0x00400000 */
#define ADC_JSQR_JSQ3_3                (0x08U << ADC_JSQR_JSQ3_Pos)            /*!< 0x00800000 */
#define ADC_JSQR_JSQ3_4                (0x10U << ADC_JSQR_JSQ3_Pos)            /*!< 0x01000000 */

#define ADC_JSQR_JSQ4_Pos              (26U)                                   
#define ADC_JSQR_JSQ4_Msk              (0x1FU << ADC_JSQR_JSQ4_Pos)            /*!< 0x7C000000 */
#define ADC_JSQR_JSQ4                  ADC_JSQR_JSQ4_Msk                       /*!< ADC group injected sequencer rank 4 */
#define ADC_JSQR_JSQ4_0                (0x01U << ADC_JSQR_JSQ4_Pos)            /*!< 0x04000000 */
#define ADC_JSQR_JSQ4_1                (0x02U << ADC_JSQR_JSQ4_Pos)            /*!< 0x08000000 */
#define ADC_JSQR_JSQ4_2                (0x04U << ADC_JSQR_JSQ4_Pos)            /*!< 0x10000000 */
#define ADC_JSQR_JSQ4_3                (0x08U << ADC_JSQR_JSQ4_Pos)            /*!< 0x20000000 */
#define ADC_JSQR_JSQ4_4                (0x10U << ADC_JSQR_JSQ4_Pos)            /*!< 0x40000000 */


/********************  Bit definition for ADC_OFR1 register  ******************/
#define ADC_OFR1_OFFSET1_Pos           (0U)                                    
#define ADC_OFR1_OFFSET1_Msk           (0xFFFU << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000FFF */
#define ADC_OFR1_OFFSET1               ADC_OFR1_OFFSET1_Msk                    /*!< ADC offset number 1 offset level */
#define ADC_OFR1_OFFSET1_0             (0x001U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000001 */
#define ADC_OFR1_OFFSET1_1             (0x002U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000002 */
#define ADC_OFR1_OFFSET1_2             (0x004U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000004 */
#define ADC_OFR1_OFFSET1_3             (0x008U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000008 */
#define ADC_OFR1_OFFSET1_4             (0x010U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000010 */
#define ADC_OFR1_OFFSET1_5             (0x020U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000020 */
#define ADC_OFR1_OFFSET1_6             (0x040U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000040 */
#define ADC_OFR1_OFFSET1_7             (0x080U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000080 */
#define ADC_OFR1_OFFSET1_8             (0x100U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000100 */
#define ADC_OFR1_OFFSET1_9             (0x200U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000200 */
#define ADC_OFR1_OFFSET1_10            (0x400U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000400 */
#define ADC_OFR1_OFFSET1_11            (0x800U << ADC_OFR1_OFFSET1_Pos)        /*!< 0x00000800 */

#define ADC_OFR1_OFFSET1_CH_Pos        (26U)                                   
#define ADC_OFR1_OFFSET1_CH_Msk        (0x1FU << ADC_OFR1_OFFSET1_CH_Pos)      /*!< 0x7C000000 */
#define ADC_OFR1_OFFSET1_CH            ADC_OFR1_OFFSET1_CH_Msk                 /*!< ADC offset number 1 channel selection */
#define ADC_OFR1_OFFSET1_CH_0          (0x01U << ADC_OFR1_OFFSET1_CH_Pos)      /*!< 0x04000000 */
#define ADC_OFR1_OFFSET1_CH_1          (0x02U << ADC_OFR1_OFFSET1_CH_Pos)      /*!< 0x08000000 */
#define ADC_OFR1_OFFSET1_CH_2          (0x04U << ADC_OFR1_OFFSET1_CH_Pos)      /*!< 0x10000000 */
#define ADC_OFR1_OFFSET1_CH_3          (0x08U << ADC_OFR1_OFFSET1_CH_Pos)      /*!< 0x20000000 */
#define ADC_OFR1_OFFSET1_CH_4          (0x10U << ADC_OFR1_OFFSET1_CH_Pos)      /*!< 0x40000000 */

#define ADC_OFR1_OFFSET1_EN_Pos        (31U)                                   
#define ADC_OFR1_OFFSET1_EN_Msk        (0x1U << ADC_OFR1_OFFSET1_EN_Pos)       /*!< 0x80000000 */
#define ADC_OFR1_OFFSET1_EN            ADC_OFR1_OFFSET1_EN_Msk                 /*!< ADC offset number 1 enable */

/********************  Bit definition for ADC_OFR2 register  ******************/
#define ADC_OFR2_OFFSET2_Pos           (0U)                                    
#define ADC_OFR2_OFFSET2_Msk           (0xFFFU << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000FFF */
#define ADC_OFR2_OFFSET2               ADC_OFR2_OFFSET2_Msk                    /*!< ADC offset number 2 offset level */
#define ADC_OFR2_OFFSET2_0             (0x001U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000001 */
#define ADC_OFR2_OFFSET2_1             (0x002U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000002 */
#define ADC_OFR2_OFFSET2_2             (0x004U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000004 */
#define ADC_OFR2_OFFSET2_3             (0x008U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000008 */
#define ADC_OFR2_OFFSET2_4             (0x010U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000010 */
#define ADC_OFR2_OFFSET2_5             (0x020U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000020 */
#define ADC_OFR2_OFFSET2_6             (0x040U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000040 */
#define ADC_OFR2_OFFSET2_7             (0x080U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000080 */
#define ADC_OFR2_OFFSET2_8             (0x100U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000100 */
#define ADC_OFR2_OFFSET2_9             (0x200U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000200 */
#define ADC_OFR2_OFFSET2_10            (0x400U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000400 */
#define ADC_OFR2_OFFSET2_11            (0x800U << ADC_OFR2_OFFSET2_Pos)        /*!< 0x00000800 */

#define ADC_OFR2_OFFSET2_CH_Pos        (26U)                                   
#define ADC_OFR2_OFFSET2_CH_Msk        (0x1FU << ADC_OFR2_OFFSET2_CH_Pos)      /*!< 0x7C000000 */
#define ADC_OFR2_OFFSET2_CH            ADC_OFR2_OFFSET2_CH_Msk                 /*!< ADC offset number 2 channel selection */
#define ADC_OFR2_OFFSET2_CH_0          (0x01U << ADC_OFR2_OFFSET2_CH_Pos)      /*!< 0x04000000 */
#define ADC_OFR2_OFFSET2_CH_1          (0x02U << ADC_OFR2_OFFSET2_CH_Pos)      /*!< 0x08000000 */
#define ADC_OFR2_OFFSET2_CH_2          (0x04U << ADC_OFR2_OFFSET2_CH_Pos)      /*!< 0x10000000 */
#define ADC_OFR2_OFFSET2_CH_3          (0x08U << ADC_OFR2_OFFSET2_CH_Pos)      /*!< 0x20000000 */
#define ADC_OFR2_OFFSET2_CH_4          (0x10U << ADC_OFR2_OFFSET2_CH_Pos)      /*!< 0x40000000 */

#define ADC_OFR2_OFFSET2_EN_Pos        (31U)                                   
#define ADC_OFR2_OFFSET2_EN_Msk        (0x1U << ADC_OFR2_OFFSET2_EN_Pos)       /*!< 0x80000000 */
#define ADC_OFR2_OFFSET2_EN            ADC_OFR2_OFFSET2_EN_Msk                 /*!< ADC offset number 2 enable */

/********************  Bit definition for ADC_OFR3 register  ******************/
#define ADC_OFR3_OFFSET3_Pos           (0U)                                    
#define ADC_OFR3_OFFSET3_Msk           (0xFFFU << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000FFF */
#define ADC_OFR3_OFFSET3               ADC_OFR3_OFFSET3_Msk                    /*!< ADC offset number 3 offset level */
#define ADC_OFR3_OFFSET3_0             (0x001U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000001 */
#define ADC_OFR3_OFFSET3_1             (0x002U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000002 */
#define ADC_OFR3_OFFSET3_2             (0x004U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000004 */
#define ADC_OFR3_OFFSET3_3             (0x008U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000008 */
#define ADC_OFR3_OFFSET3_4             (0x010U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000010 */
#define ADC_OFR3_OFFSET3_5             (0x020U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000020 */
#define ADC_OFR3_OFFSET3_6             (0x040U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000040 */
#define ADC_OFR3_OFFSET3_7             (0x080U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000080 */
#define ADC_OFR3_OFFSET3_8             (0x100U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000100 */
#define ADC_OFR3_OFFSET3_9             (0x200U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000200 */
#define ADC_OFR3_OFFSET3_10            (0x400U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000400 */
#define ADC_OFR3_OFFSET3_11            (0x800U << ADC_OFR3_OFFSET3_Pos)        /*!< 0x00000800 */

#define ADC_OFR3_OFFSET3_CH_Pos        (26U)                                   
#define ADC_OFR3_OFFSET3_CH_Msk        (0x1FU << ADC_OFR3_OFFSET3_CH_Pos)      /*!< 0x7C000000 */
#define ADC_OFR3_OFFSET3_CH            ADC_OFR3_OFFSET3_CH_Msk                 /*!< ADC offset number 3 channel selection */
#define ADC_OFR3_OFFSET3_CH_0          (0x01U << ADC_OFR3_OFFSET3_CH_Pos)      /*!< 0x04000000 */
#define ADC_OFR3_OFFSET3_CH_1          (0x02U << ADC_OFR3_OFFSET3_CH_Pos)      /*!< 0x08000000 */
#define ADC_OFR3_OFFSET3_CH_2          (0x04U << ADC_OFR3_OFFSET3_CH_Pos)      /*!< 0x10000000 */
#define ADC_OFR3_OFFSET3_CH_3          (0x08U << ADC_OFR3_OFFSET3_CH_Pos)      /*!< 0x20000000 */
#define ADC_OFR3_OFFSET3_CH_4          (0x10U << ADC_OFR3_OFFSET3_CH_Pos)      /*!< 0x40000000 */

#define ADC_OFR3_OFFSET3_EN_Pos        (31U)                                   
#define ADC_OFR3_OFFSET3_EN_Msk        (0x1U << ADC_OFR3_OFFSET3_EN_Pos)       /*!< 0x80000000 */
#define ADC_OFR3_OFFSET3_EN            ADC_OFR3_OFFSET3_EN_Msk                 /*!< ADC offset number 3 enable */

/********************  Bit definition for ADC_OFR4 register  ******************/
#define ADC_OFR4_OFFSET4_Pos           (0U)                                    
#define ADC_OFR4_OFFSET4_Msk           (0xFFFU << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000FFF */
#define ADC_OFR4_OFFSET4               ADC_OFR4_OFFSET4_Msk                    /*!< ADC offset number 4 offset level */
#define ADC_OFR4_OFFSET4_0             (0x001U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000001 */
#define ADC_OFR4_OFFSET4_1             (0x002U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000002 */
#define ADC_OFR4_OFFSET4_2             (0x004U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000004 */
#define ADC_OFR4_OFFSET4_3             (0x008U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000008 */
#define ADC_OFR4_OFFSET4_4             (0x010U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000010 */
#define ADC_OFR4_OFFSET4_5             (0x020U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000020 */
#define ADC_OFR4_OFFSET4_6             (0x040U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000040 */
#define ADC_OFR4_OFFSET4_7             (0x080U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000080 */
#define ADC_OFR4_OFFSET4_8             (0x100U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000100 */
#define ADC_OFR4_OFFSET4_9             (0x200U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000200 */
#define ADC_OFR4_OFFSET4_10            (0x400U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000400 */
#define ADC_OFR4_OFFSET4_11            (0x800U << ADC_OFR4_OFFSET4_Pos)        /*!< 0x00000800 */

#define ADC_OFR4_OFFSET4_CH_Pos        (26U)                                   
#define ADC_OFR4_OFFSET4_CH_Msk        (0x1FU << ADC_OFR4_OFFSET4_CH_Pos)      /*!< 0x7C000000 */
#define ADC_OFR4_OFFSET4_CH            ADC_OFR4_OFFSET4_CH_Msk                 /*!< ADC offset number 4 channel selection */
#define ADC_OFR4_OFFSET4_CH_0          (0x01U << ADC_OFR4_OFFSET4_CH_Pos)      /*!< 0x04000000 */
#define ADC_OFR4_OFFSET4_CH_1          (0x02U << ADC_OFR4_OFFSET4_CH_Pos)      /*!< 0x08000000 */
#define ADC_OFR4_OFFSET4_CH_2          (0x04U << ADC_OFR4_OFFSET4_CH_Pos)      /*!< 0x10000000 */
#define ADC_OFR4_OFFSET4_CH_3          (0x08U << ADC_OFR4_OFFSET4_CH_Pos)      /*!< 0x20000000 */
#define ADC_OFR4_OFFSET4_CH_4          (0x10U << ADC_OFR4_OFFSET4_CH_Pos)      /*!< 0x40000000 */

#define ADC_OFR4_OFFSET4_EN_Pos        (31U)                                   
#define ADC_OFR4_OFFSET4_EN_Msk        (0x1U << ADC_OFR4_OFFSET4_EN_Pos)       /*!< 0x80000000 */
#define ADC_OFR4_OFFSET4_EN            ADC_OFR4_OFFSET4_EN_Msk                 /*!< ADC offset number 4 enable */

/********************  Bit definition for ADC_JDR1 register  ******************/
#define ADC_JDR1_JDATA_Pos             (0U)                                    
#define ADC_JDR1_JDATA_Msk             (0xFFFFU << ADC_JDR1_JDATA_Pos)         /*!< 0x0000FFFF */
#define ADC_JDR1_JDATA                 ADC_JDR1_JDATA_Msk                      /*!< ADC group injected sequencer rank 1 conversion data */
#define ADC_JDR1_JDATA_0               (0x0001U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000001 */
#define ADC_JDR1_JDATA_1               (0x0002U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000002 */
#define ADC_JDR1_JDATA_2               (0x0004U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000004 */
#define ADC_JDR1_JDATA_3               (0x0008U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000008 */
#define ADC_JDR1_JDATA_4               (0x0010U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000010 */
#define ADC_JDR1_JDATA_5               (0x0020U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000020 */
#define ADC_JDR1_JDATA_6               (0x0040U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000040 */
#define ADC_JDR1_JDATA_7               (0x0080U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000080 */
#define ADC_JDR1_JDATA_8               (0x0100U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000100 */
#define ADC_JDR1_JDATA_9               (0x0200U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000200 */
#define ADC_JDR1_JDATA_10              (0x0400U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000400 */
#define ADC_JDR1_JDATA_11              (0x0800U << ADC_JDR1_JDATA_Pos)         /*!< 0x00000800 */
#define ADC_JDR1_JDATA_12              (0x1000U << ADC_JDR1_JDATA_Pos)         /*!< 0x00001000 */
#define ADC_JDR1_JDATA_13              (0x2000U << ADC_JDR1_JDATA_Pos)         /*!< 0x00002000 */
#define ADC_JDR1_JDATA_14              (0x4000U << ADC_JDR1_JDATA_Pos)         /*!< 0x00004000 */
#define ADC_JDR1_JDATA_15              (0x8000U << ADC_JDR1_JDATA_Pos)         /*!< 0x00008000 */

/********************  Bit definition for ADC_JDR2 register  ******************/
#define ADC_JDR2_JDATA_Pos             (0U)                                    
#define ADC_JDR2_JDATA_Msk             (0xFFFFU << ADC_JDR2_JDATA_Pos)         /*!< 0x0000FFFF */
#define ADC_JDR2_JDATA                 ADC_JDR2_JDATA_Msk                      /*!< ADC group injected sequencer rank 2 conversion data */
#define ADC_JDR2_JDATA_0               (0x0001U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000001 */
#define ADC_JDR2_JDATA_1               (0x0002U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000002 */
#define ADC_JDR2_JDATA_2               (0x0004U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000004 */
#define ADC_JDR2_JDATA_3               (0x0008U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000008 */
#define ADC_JDR2_JDATA_4               (0x0010U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000010 */
#define ADC_JDR2_JDATA_5               (0x0020U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000020 */
#define ADC_JDR2_JDATA_6               (0x0040U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000040 */
#define ADC_JDR2_JDATA_7               (0x0080U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000080 */
#define ADC_JDR2_JDATA_8               (0x0100U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000100 */
#define ADC_JDR2_JDATA_9               (0x0200U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000200 */
#define ADC_JDR2_JDATA_10              (0x0400U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000400 */
#define ADC_JDR2_JDATA_11              (0x0800U << ADC_JDR2_JDATA_Pos)         /*!< 0x00000800 */
#define ADC_JDR2_JDATA_12              (0x1000U << ADC_JDR2_JDATA_Pos)         /*!< 0x00001000 */
#define ADC_JDR2_JDATA_13              (0x2000U << ADC_JDR2_JDATA_Pos)         /*!< 0x00002000 */
#define ADC_JDR2_JDATA_14              (0x4000U << ADC_JDR2_JDATA_Pos)         /*!< 0x00004000 */
#define ADC_JDR2_JDATA_15              (0x8000U << ADC_JDR2_JDATA_Pos)         /*!< 0x00008000 */

/********************  Bit definition for ADC_JDR3 register  ******************/
#define ADC_JDR3_JDATA_Pos             (0U)                                    
#define ADC_JDR3_JDATA_Msk             (0xFFFFU << ADC_JDR3_JDATA_Pos)         /*!< 0x0000FFFF */
#define ADC_JDR3_JDATA                 ADC_JDR3_JDATA_Msk                      /*!< ADC group injected sequencer rank 3 conversion data */
#define ADC_JDR3_JDATA_0               (0x0001U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000001 */
#define ADC_JDR3_JDATA_1               (0x0002U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000002 */
#define ADC_JDR3_JDATA_2               (0x0004U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000004 */
#define ADC_JDR3_JDATA_3               (0x0008U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000008 */
#define ADC_JDR3_JDATA_4               (0x0010U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000010 */
#define ADC_JDR3_JDATA_5               (0x0020U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000020 */
#define ADC_JDR3_JDATA_6               (0x0040U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000040 */
#define ADC_JDR3_JDATA_7               (0x0080U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000080 */
#define ADC_JDR3_JDATA_8               (0x0100U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000100 */
#define ADC_JDR3_JDATA_9               (0x0200U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000200 */
#define ADC_JDR3_JDATA_10              (0x0400U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000400 */
#define ADC_JDR3_JDATA_11              (0x0800U << ADC_JDR3_JDATA_Pos)         /*!< 0x00000800 */
#define ADC_JDR3_JDATA_12              (0x1000U << ADC_JDR3_JDATA_Pos)         /*!< 0x00001000 */
#define ADC_JDR3_JDATA_13              (0x2000U << ADC_JDR3_JDATA_Pos)         /*!< 0x00002000 */
#define ADC_JDR3_JDATA_14              (0x4000U << ADC_JDR3_JDATA_Pos)         /*!< 0x00004000 */
#define ADC_JDR3_JDATA_15              (0x8000U << ADC_JDR3_JDATA_Pos)         /*!< 0x00008000 */

/********************  Bit definition for ADC_JDR4 register  ******************/
#define ADC_JDR4_JDATA_Pos             (0U)                                    
#define ADC_JDR4_JDATA_Msk             (0xFFFFU << ADC_JDR4_JDATA_Pos)         /*!< 0x0000FFFF */
#define ADC_JDR4_JDATA                 ADC_JDR4_JDATA_Msk                      /*!< ADC group injected sequencer rank 4 conversion data */
#define ADC_JDR4_JDATA_0               (0x0001U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000001 */
#define ADC_JDR4_JDATA_1               (0x0002U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000002 */
#define ADC_JDR4_JDATA_2               (0x0004U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000004 */
#define ADC_JDR4_JDATA_3               (0x0008U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000008 */
#define ADC_JDR4_JDATA_4               (0x0010U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000010 */
#define ADC_JDR4_JDATA_5               (0x0020U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000020 */
#define ADC_JDR4_JDATA_6               (0x0040U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000040 */
#define ADC_JDR4_JDATA_7               (0x0080U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000080 */
#define ADC_JDR4_JDATA_8               (0x0100U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000100 */
#define ADC_JDR4_JDATA_9               (0x0200U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000200 */
#define ADC_JDR4_JDATA_10              (0x0400U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000400 */
#define ADC_JDR4_JDATA_11              (0x0800U << ADC_JDR4_JDATA_Pos)         /*!< 0x00000800 */
#define ADC_JDR4_JDATA_12              (0x1000U << ADC_JDR4_JDATA_Pos)         /*!< 0x00001000 */
#define ADC_JDR4_JDATA_13              (0x2000U << ADC_JDR4_JDATA_Pos)         /*!< 0x00002000 */
#define ADC_JDR4_JDATA_14              (0x4000U << ADC_JDR4_JDATA_Pos)         /*!< 0x00004000 */
#define ADC_JDR4_JDATA_15              (0x8000U << ADC_JDR4_JDATA_Pos)         /*!< 0x00008000 */

/********************  Bit definition for ADC_AWD2CR register  ****************/
#define ADC_AWD2CR_AWD2CH_Pos          (0U)                                    
#define ADC_AWD2CR_AWD2CH_Msk          (0x7FFFFU << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x0007FFFF */
#define ADC_AWD2CR_AWD2CH              ADC_AWD2CR_AWD2CH_Msk                   /*!< ADC analog watchdog 2 monitored channel selection */
#define ADC_AWD2CR_AWD2CH_0            (0x00001U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000001 */
#define ADC_AWD2CR_AWD2CH_1            (0x00002U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000002 */
#define ADC_AWD2CR_AWD2CH_2            (0x00004U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000004 */
#define ADC_AWD2CR_AWD2CH_3            (0x00008U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000008 */
#define ADC_AWD2CR_AWD2CH_4            (0x00010U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000010 */
#define ADC_AWD2CR_AWD2CH_5            (0x00020U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000020 */
#define ADC_AWD2CR_AWD2CH_6            (0x00040U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000040 */
#define ADC_AWD2CR_AWD2CH_7            (0x00080U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000080 */
#define ADC_AWD2CR_AWD2CH_8            (0x00100U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000100 */
#define ADC_AWD2CR_AWD2CH_9            (0x00200U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000200 */
#define ADC_AWD2CR_AWD2CH_10           (0x00400U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000400 */
#define ADC_AWD2CR_AWD2CH_11           (0x00800U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00000800 */
#define ADC_AWD2CR_AWD2CH_12           (0x01000U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00001000 */
#define ADC_AWD2CR_AWD2CH_13           (0x02000U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00002000 */
#define ADC_AWD2CR_AWD2CH_14           (0x04000U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00004000 */
#define ADC_AWD2CR_AWD2CH_15           (0x08000U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00008000 */
#define ADC_AWD2CR_AWD2CH_16           (0x10000U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00010000 */
#define ADC_AWD2CR_AWD2CH_17           (0x20000U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00020000 */
#define ADC_AWD2CR_AWD2CH_18           (0x40000U << ADC_AWD2CR_AWD2CH_Pos)     /*!< 0x00040000 */

/********************  Bit definition for ADC_AWD3CR register  ****************/
#define ADC_AWD3CR_AWD3CH_Pos          (0U)                                    
#define ADC_AWD3CR_AWD3CH_Msk          (0x7FFFFU << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x0007FFFF */
#define ADC_AWD3CR_AWD3CH              ADC_AWD3CR_AWD3CH_Msk                   /*!< ADC analog watchdog 3 monitored channel selection */
#define ADC_AWD3CR_AWD3CH_0            (0x00001U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000001 */
#define ADC_AWD3CR_AWD3CH_1            (0x00002U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000002 */
#define ADC_AWD3CR_AWD3CH_2            (0x00004U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000004 */
#define ADC_AWD3CR_AWD3CH_3            (0x00008U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000008 */
#define ADC_AWD3CR_AWD3CH_4            (0x00010U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000010 */
#define ADC_AWD3CR_AWD3CH_5            (0x00020U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000020 */
#define ADC_AWD3CR_AWD3CH_6            (0x00040U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000040 */
#define ADC_AWD3CR_AWD3CH_7            (0x00080U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000080 */
#define ADC_AWD3CR_AWD3CH_8            (0x00100U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000100 */
#define ADC_AWD3CR_AWD3CH_9            (0x00200U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000200 */
#define ADC_AWD3CR_AWD3CH_10           (0x00400U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000400 */
#define ADC_AWD3CR_AWD3CH_11           (0x00800U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00000800 */
#define ADC_AWD3CR_AWD3CH_12           (0x01000U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00001000 */
#define ADC_AWD3CR_AWD3CH_13           (0x02000U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00002000 */
#define ADC_AWD3CR_AWD3CH_14           (0x04000U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00004000 */
#define ADC_AWD3CR_AWD3CH_15           (0x08000U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00008000 */
#define ADC_AWD3CR_AWD3CH_16           (0x10000U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00010000 */
#define ADC_AWD3CR_AWD3CH_17           (0x20000U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00020000 */
#define ADC_AWD3CR_AWD3CH_18           (0x40000U << ADC_AWD3CR_AWD3CH_Pos)     /*!< 0x00040000 */

/********************  Bit definition for ADC_DIFSEL register  ****************/
#define ADC_DIFSEL_DIFSEL_Pos          (0U)                                    
#define ADC_DIFSEL_DIFSEL_Msk          (0x7FFFFU << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x0007FFFF */
#define ADC_DIFSEL_DIFSEL              ADC_DIFSEL_DIFSEL_Msk                   /*!< ADC channel differential or single-ended mode */
#define ADC_DIFSEL_DIFSEL_0            (0x00001U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000001 */
#define ADC_DIFSEL_DIFSEL_1            (0x00002U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000002 */
#define ADC_DIFSEL_DIFSEL_2            (0x00004U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000004 */
#define ADC_DIFSEL_DIFSEL_3            (0x00008U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000008 */
#define ADC_DIFSEL_DIFSEL_4            (0x00010U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000010 */
#define ADC_DIFSEL_DIFSEL_5            (0x00020U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000020 */
#define ADC_DIFSEL_DIFSEL_6            (0x00040U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000040 */
#define ADC_DIFSEL_DIFSEL_7            (0x00080U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000080 */
#define ADC_DIFSEL_DIFSEL_8            (0x00100U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000100 */
#define ADC_DIFSEL_DIFSEL_9            (0x00200U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000200 */
#define ADC_DIFSEL_DIFSEL_10           (0x00400U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000400 */
#define ADC_DIFSEL_DIFSEL_11           (0x00800U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00000800 */
#define ADC_DIFSEL_DIFSEL_12           (0x01000U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00001000 */
#define ADC_DIFSEL_DIFSEL_13           (0x02000U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00002000 */
#define ADC_DIFSEL_DIFSEL_14           (0x04000U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00004000 */
#define ADC_DIFSEL_DIFSEL_15           (0x08000U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00008000 */
#define ADC_DIFSEL_DIFSEL_16           (0x10000U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00010000 */
#define ADC_DIFSEL_DIFSEL_17           (0x20000U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00020000 */
#define ADC_DIFSEL_DIFSEL_18           (0x40000U << ADC_DIFSEL_DIFSEL_Pos)     /*!< 0x00040000 */

/********************  Bit definition for ADC_CALFACT register  ***************/
#define ADC_CALFACT_CALFACT_S_Pos      (0U)                                    
#define ADC_CALFACT_CALFACT_S_Msk      (0x7FU << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x0000007F */
#define ADC_CALFACT_CALFACT_S          ADC_CALFACT_CALFACT_S_Msk               /*!< ADC calibration factor in single-ended mode */
#define ADC_CALFACT_CALFACT_S_0        (0x01U << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x00000001 */
#define ADC_CALFACT_CALFACT_S_1        (0x02U << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x00000002 */
#define ADC_CALFACT_CALFACT_S_2        (0x04U << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x00000004 */
#define ADC_CALFACT_CALFACT_S_3        (0x08U << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x00000008 */
#define ADC_CALFACT_CALFACT_S_4        (0x10U << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x00000010 */
#define ADC_CALFACT_CALFACT_S_5        (0x20U << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x00000020 */
#define ADC_CALFACT_CALFACT_S_6        (0x40U << ADC_CALFACT_CALFACT_S_Pos)    /*!< 0x00000040 */

#define ADC_CALFACT_CALFACT_D_Pos      (16U)                                   
#define ADC_CALFACT_CALFACT_D_Msk      (0x7FU << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x007F0000 */
#define ADC_CALFACT_CALFACT_D          ADC_CALFACT_CALFACT_D_Msk               /*!< ADC calibration factor in differential mode */
#define ADC_CALFACT_CALFACT_D_0        (0x01U << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x00010000 */
#define ADC_CALFACT_CALFACT_D_1        (0x02U << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x00020000 */
#define ADC_CALFACT_CALFACT_D_2        (0x04U << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x00040000 */
#define ADC_CALFACT_CALFACT_D_3        (0x08U << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x00080000 */
#define ADC_CALFACT_CALFACT_D_4        (0x10U << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x00100000 */
#define ADC_CALFACT_CALFACT_D_5        (0x20U << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x00200000 */
#define ADC_CALFACT_CALFACT_D_6        (0x40U << ADC_CALFACT_CALFACT_D_Pos)    /*!< 0x00400000 */

/*************************  ADC Common registers  *****************************/
/***************  Bit definition for ADC1_COMMON_CSR register  ***************/
#define ADC1_CSR_ADRDY_MST_Pos          (0U)                                   
#define ADC1_CSR_ADRDY_MST_Msk          (0x1U << ADC1_CSR_ADRDY_MST_Pos)       /*!< 0x00000001 */
#define ADC1_CSR_ADRDY_MST              ADC1_CSR_ADRDY_MST_Msk                 /*!< Master ADC ready */
#define ADC1_CSR_ADRDY_EOSMP_MST_Pos    (1U)                                   
#define ADC1_CSR_ADRDY_EOSMP_MST_Msk    (0x1U << ADC1_CSR_ADRDY_EOSMP_MST_Pos) /*!< 0x00000002 */
#define ADC1_CSR_ADRDY_EOSMP_MST        ADC1_CSR_ADRDY_EOSMP_MST_Msk           /*!< End of sampling phase flag of the master ADC */
#define ADC1_CSR_ADRDY_EOC_MST_Pos      (2U)                                   
#define ADC1_CSR_ADRDY_EOC_MST_Msk      (0x1U << ADC1_CSR_ADRDY_EOC_MST_Pos)   /*!< 0x00000004 */
#define ADC1_CSR_ADRDY_EOC_MST          ADC1_CSR_ADRDY_EOC_MST_Msk             /*!< End of regular conversion of the master ADC */
#define ADC1_CSR_ADRDY_EOS_MST_Pos      (3U)                                   
#define ADC1_CSR_ADRDY_EOS_MST_Msk      (0x1U << ADC1_CSR_ADRDY_EOS_MST_Pos)   /*!< 0x00000008 */
#define ADC1_CSR_ADRDY_EOS_MST          ADC1_CSR_ADRDY_EOS_MST_Msk             /*!< End of regular sequence flag of the master ADC */
#define ADC1_CSR_ADRDY_OVR_MST_Pos      (4U)                                   
#define ADC1_CSR_ADRDY_OVR_MST_Msk      (0x1U << ADC1_CSR_ADRDY_OVR_MST_Pos)   /*!< 0x00000010 */
#define ADC1_CSR_ADRDY_OVR_MST          ADC1_CSR_ADRDY_OVR_MST_Msk             /*!< Overrun flag of the master ADC */
#define ADC1_CSR_ADRDY_JEOC_MST_Pos     (5U)                                   
#define ADC1_CSR_ADRDY_JEOC_MST_Msk     (0x1U << ADC1_CSR_ADRDY_JEOC_MST_Pos)  /*!< 0x00000020 */
#define ADC1_CSR_ADRDY_JEOC_MST         ADC1_CSR_ADRDY_JEOC_MST_Msk            /*!< End of injected conversion of the master ADC */
#define ADC1_CSR_ADRDY_JEOS_MST_Pos     (6U)                                   
#define ADC1_CSR_ADRDY_JEOS_MST_Msk     (0x1U << ADC1_CSR_ADRDY_JEOS_MST_Pos)  /*!< 0x00000040 */
#define ADC1_CSR_ADRDY_JEOS_MST         ADC1_CSR_ADRDY_JEOS_MST_Msk            /*!< End of injected sequence flag of the master ADC */
#define ADC1_CSR_AWD1_MST_Pos           (7U)                                   
#define ADC1_CSR_AWD1_MST_Msk           (0x1U << ADC1_CSR_AWD1_MST_Pos)        /*!< 0x00000080 */
#define ADC1_CSR_AWD1_MST               ADC1_CSR_AWD1_MST_Msk                  /*!< Analog watchdog 1 flag of the master ADC */
#define ADC1_CSR_AWD2_MST_Pos           (8U)                                   
#define ADC1_CSR_AWD2_MST_Msk           (0x1U << ADC1_CSR_AWD2_MST_Pos)        /*!< 0x00000100 */
#define ADC1_CSR_AWD2_MST               ADC1_CSR_AWD2_MST_Msk                  /*!< Analog watchdog 2 flag of the master ADC */
#define ADC1_CSR_AWD3_MST_Pos           (9U)                                   
#define ADC1_CSR_AWD3_MST_Msk           (0x1U << ADC1_CSR_AWD3_MST_Pos)        /*!< 0x00000200 */
#define ADC1_CSR_AWD3_MST               ADC1_CSR_AWD3_MST_Msk                  /*!< Analog watchdog 3 flag of the master ADC */
#define ADC1_CSR_JQOVF_MST_Pos          (10U)                                  
#define ADC1_CSR_JQOVF_MST_Msk          (0x1U << ADC1_CSR_JQOVF_MST_Pos)       /*!< 0x00000400 */
#define ADC1_CSR_JQOVF_MST              ADC1_CSR_JQOVF_MST_Msk                 /*!< Injected context queue overflow flag of the master ADC */
#define ADC1_CSR_ADRDY_SLV_Pos          (16U)                                  
#define ADC1_CSR_ADRDY_SLV_Msk          (0x1U << ADC1_CSR_ADRDY_SLV_Pos)       /*!< 0x00010000 */
#define ADC1_CSR_ADRDY_SLV              ADC1_CSR_ADRDY_SLV_Msk                 /*!< Slave ADC ready */
#define ADC1_CSR_ADRDY_EOSMP_SLV_Pos    (17U)                                  
#define ADC1_CSR_ADRDY_EOSMP_SLV_Msk    (0x1U << ADC1_CSR_ADRDY_EOSMP_SLV_Pos) /*!< 0x00020000 */
#define ADC1_CSR_ADRDY_EOSMP_SLV        ADC1_CSR_ADRDY_EOSMP_SLV_Msk           /*!< End of sampling phase flag of the slave ADC */
#define ADC1_CSR_ADRDY_EOC_SLV_Pos      (18U)                                  
#define ADC1_CSR_ADRDY_EOC_SLV_Msk      (0x1U << ADC1_CSR_ADRDY_EOC_SLV_Pos)   /*!< 0x00040000 */
#define ADC1_CSR_ADRDY_EOC_SLV          ADC1_CSR_ADRDY_EOC_SLV_Msk             /*!< End of regular conversion of the slave ADC */
#define ADC1_CSR_ADRDY_EOS_SLV_Pos      (19U)                                  
#define ADC1_CSR_ADRDY_EOS_SLV_Msk      (0x1U << ADC1_CSR_ADRDY_EOS_SLV_Pos)   /*!< 0x00080000 */
#define ADC1_CSR_ADRDY_EOS_SLV          ADC1_CSR_ADRDY_EOS_SLV_Msk             /*!< End of regular sequence flag of the slave ADC */
#define ADC1_CSR_ADRDY_OVR_SLV_Pos      (20U)                                  
#define ADC1_CSR_ADRDY_OVR_SLV_Msk      (0x1U << ADC1_CSR_ADRDY_OVR_SLV_Pos)   /*!< 0x00100000 */
#define ADC1_CSR_ADRDY_OVR_SLV          ADC1_CSR_ADRDY_OVR_SLV_Msk             /*!< Overrun flag of the slave ADC */
#define ADC1_CSR_ADRDY_JEOC_SLV_Pos     (21U)                                  
#define ADC1_CSR_ADRDY_JEOC_SLV_Msk     (0x1U << ADC1_CSR_ADRDY_JEOC_SLV_Pos)  /*!< 0x00200000 */
#define ADC1_CSR_ADRDY_JEOC_SLV         ADC1_CSR_ADRDY_JEOC_SLV_Msk            /*!< End of injected conversion of the slave ADC */
#define ADC1_CSR_ADRDY_JEOS_SLV_Pos     (22U)                                  
#define ADC1_CSR_ADRDY_JEOS_SLV_Msk     (0x1U << ADC1_CSR_ADRDY_JEOS_SLV_Pos)  /*!< 0x00400000 */
#define ADC1_CSR_ADRDY_JEOS_SLV         ADC1_CSR_ADRDY_JEOS_SLV_Msk            /*!< End of injected sequence flag of the slave ADC */
#define ADC1_CSR_AWD1_SLV_Pos           (23U)                                  
#define ADC1_CSR_AWD1_SLV_Msk           (0x1U << ADC1_CSR_AWD1_SLV_Pos)        /*!< 0x00800000 */
#define ADC1_CSR_AWD1_SLV               ADC1_CSR_AWD1_SLV_Msk                  /*!< Analog watchdog 1 flag of the slave ADC */
#define ADC1_CSR_AWD2_SLV_Pos           (24U)                                  
#define ADC1_CSR_AWD2_SLV_Msk           (0x1U << ADC1_CSR_AWD2_SLV_Pos)        /*!< 0x01000000 */
#define ADC1_CSR_AWD2_SLV               ADC1_CSR_AWD2_SLV_Msk                  /*!< Analog watchdog 2 flag of the slave ADC */
#define ADC1_CSR_AWD3_SLV_Pos           (25U)                                  
#define ADC1_CSR_AWD3_SLV_Msk           (0x1U << ADC1_CSR_AWD3_SLV_Pos)        /*!< 0x02000000 */
#define ADC1_CSR_AWD3_SLV               ADC1_CSR_AWD3_SLV_Msk                  /*!< Analog watchdog 3 flag of the slave ADC */
#define ADC1_CSR_JQOVF_SLV_Pos          (26U)                                  
#define ADC1_CSR_JQOVF_SLV_Msk          (0x1U << ADC1_CSR_JQOVF_SLV_Pos)       /*!< 0x04000000 */
#define ADC1_CSR_JQOVF_SLV              ADC1_CSR_JQOVF_SLV_Msk                 /*!< Injected context queue overflow flag of the slave ADC */

/***************  Bit definition for ADC1_COMMON_CCR register  ***************/
#define ADC1_CCR_MULTI_Pos              (0U)                                   
#define ADC1_CCR_MULTI_Msk              (0x1FU << ADC1_CCR_MULTI_Pos)          /*!< 0x0000001F */
#define ADC1_CCR_MULTI                  ADC1_CCR_MULTI_Msk                     /*!< Multi ADC mode selection */
#define ADC1_CCR_MULTI_0                (0x01U << ADC1_CCR_MULTI_Pos)          /*!< 0x00000001 */
#define ADC1_CCR_MULTI_1                (0x02U << ADC1_CCR_MULTI_Pos)          /*!< 0x00000002 */
#define ADC1_CCR_MULTI_2                (0x04U << ADC1_CCR_MULTI_Pos)          /*!< 0x00000004 */
#define ADC1_CCR_MULTI_3                (0x08U << ADC1_CCR_MULTI_Pos)          /*!< 0x00000008 */
#define ADC1_CCR_MULTI_4                (0x10U << ADC1_CCR_MULTI_Pos)          /*!< 0x00000010 */
#define ADC1_CCR_DELAY_Pos              (8U)                                   
#define ADC1_CCR_DELAY_Msk              (0xFU << ADC1_CCR_DELAY_Pos)           /*!< 0x00000F00 */
#define ADC1_CCR_DELAY                  ADC1_CCR_DELAY_Msk                     /*!< Delay between 2 sampling phases */
#define ADC1_CCR_DELAY_0                (0x1U << ADC1_CCR_DELAY_Pos)           /*!< 0x00000100 */
#define ADC1_CCR_DELAY_1                (0x2U << ADC1_CCR_DELAY_Pos)           /*!< 0x00000200 */
#define ADC1_CCR_DELAY_2                (0x4U << ADC1_CCR_DELAY_Pos)           /*!< 0x00000400 */
#define ADC1_CCR_DELAY_3                (0x8U << ADC1_CCR_DELAY_Pos)           /*!< 0x00000800 */
#define ADC1_CCR_DMACFG_Pos             (13U)                                  
#define ADC1_CCR_DMACFG_Msk             (0x1U << ADC1_CCR_DMACFG_Pos)          /*!< 0x00002000 */
#define ADC1_CCR_DMACFG                 ADC1_CCR_DMACFG_Msk                    /*!< DMA configuration for multi-ADC mode */
#define ADC1_CCR_MDMA_Pos               (14U)                                  
#define ADC1_CCR_MDMA_Msk               (0x3U << ADC1_CCR_MDMA_Pos)            /*!< 0x0000C000 */
#define ADC1_CCR_MDMA                   ADC1_CCR_MDMA_Msk                      /*!< DMA mode for multi-ADC mode */
#define ADC1_CCR_MDMA_0                 (0x1U << ADC1_CCR_MDMA_Pos)            /*!< 0x00004000 */
#define ADC1_CCR_MDMA_1                 (0x2U << ADC1_CCR_MDMA_Pos)            /*!< 0x00008000 */
#define ADC1_CCR_CKMODE_Pos             (16U)                                  
#define ADC1_CCR_CKMODE_Msk             (0x3U << ADC1_CCR_CKMODE_Pos)          /*!< 0x00030000 */
#define ADC1_CCR_CKMODE                 ADC1_CCR_CKMODE_Msk                    /*!< ADC clock mode */
#define ADC1_CCR_CKMODE_0               (0x1U << ADC1_CCR_CKMODE_Pos)          /*!< 0x00010000 */
#define ADC1_CCR_CKMODE_1               (0x2U << ADC1_CCR_CKMODE_Pos)          /*!< 0x00020000 */
#define ADC1_CCR_VREFEN_Pos             (22U)                                  
#define ADC1_CCR_VREFEN_Msk             (0x1U << ADC1_CCR_VREFEN_Pos)          /*!< 0x00400000 */
#define ADC1_CCR_VREFEN                 ADC1_CCR_VREFEN_Msk                    /*!< VREFINT enable */
#define ADC1_CCR_TSEN_Pos               (23U)                                  
#define ADC1_CCR_TSEN_Msk               (0x1U << ADC1_CCR_TSEN_Pos)            /*!< 0x00800000 */
#define ADC1_CCR_TSEN                   ADC1_CCR_TSEN_Msk                      /*!< Temperature sensor enable */
#define ADC1_CCR_VBATEN_Pos             (24U)                                  
#define ADC1_CCR_VBATEN_Msk             (0x1U << ADC1_CCR_VBATEN_Pos)          /*!< 0x01000000 */
#define ADC1_CCR_VBATEN                 ADC1_CCR_VBATEN_Msk                    /*!< VBAT enable */

/***************  Bit definition for ADC1_COMMON_CDR register  ***************/
#define ADC1_CDR_RDATA_MST_Pos          (0U)                                   
#define ADC1_CDR_RDATA_MST_Msk          (0xFFFFU << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x0000FFFF */
#define ADC1_CDR_RDATA_MST              ADC1_CDR_RDATA_MST_Msk                 /*!< Regular Data of the master ADC */
#define ADC1_CDR_RDATA_MST_0            (0x0001U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000001 */
#define ADC1_CDR_RDATA_MST_1            (0x0002U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000002 */
#define ADC1_CDR_RDATA_MST_2            (0x0004U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000004 */
#define ADC1_CDR_RDATA_MST_3            (0x0008U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000008 */
#define ADC1_CDR_RDATA_MST_4            (0x0010U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000010 */
#define ADC1_CDR_RDATA_MST_5            (0x0020U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000020 */
#define ADC1_CDR_RDATA_MST_6            (0x0040U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000040 */
#define ADC1_CDR_RDATA_MST_7            (0x0080U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000080 */
#define ADC1_CDR_RDATA_MST_8            (0x0100U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000100 */
#define ADC1_CDR_RDATA_MST_9            (0x0200U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000200 */
#define ADC1_CDR_RDATA_MST_10           (0x0400U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000400 */
#define ADC1_CDR_RDATA_MST_11           (0x0800U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00000800 */
#define ADC1_CDR_RDATA_MST_12           (0x1000U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00001000 */
#define ADC1_CDR_RDATA_MST_13           (0x2000U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00002000 */
#define ADC1_CDR_RDATA_MST_14           (0x4000U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00004000 */
#define ADC1_CDR_RDATA_MST_15           (0x8000U << ADC1_CDR_RDATA_MST_Pos)    /*!< 0x00008000 */

#define ADC1_CDR_RDATA_SLV_Pos          (16U)                                  
#define ADC1_CDR_RDATA_SLV_Msk          (0xFFFFU << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0xFFFF0000 */
#define ADC1_CDR_RDATA_SLV              ADC1_CDR_RDATA_SLV_Msk                 /*!< Regular Data of the master ADC */
#define ADC1_CDR_RDATA_SLV_0            (0x0001U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00010000 */
#define ADC1_CDR_RDATA_SLV_1            (0x0002U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00020000 */
#define ADC1_CDR_RDATA_SLV_2            (0x0004U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00040000 */
#define ADC1_CDR_RDATA_SLV_3            (0x0008U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00080000 */
#define ADC1_CDR_RDATA_SLV_4            (0x0010U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00100000 */
#define ADC1_CDR_RDATA_SLV_5            (0x0020U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00200000 */
#define ADC1_CDR_RDATA_SLV_6            (0x0040U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00400000 */
#define ADC1_CDR_RDATA_SLV_7            (0x0080U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x00800000 */
#define ADC1_CDR_RDATA_SLV_8            (0x0100U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x01000000 */
#define ADC1_CDR_RDATA_SLV_9            (0x0200U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x02000000 */
#define ADC1_CDR_RDATA_SLV_10           (0x0400U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x04000000 */
#define ADC1_CDR_RDATA_SLV_11           (0x0800U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x08000000 */
#define ADC1_CDR_RDATA_SLV_12           (0x1000U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x10000000 */
#define ADC1_CDR_RDATA_SLV_13           (0x2000U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x20000000 */
#define ADC1_CDR_RDATA_SLV_14           (0x4000U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x40000000 */
#define ADC1_CDR_RDATA_SLV_15           (0x8000U << ADC1_CDR_RDATA_SLV_Pos)    /*!< 0x80000000 */

/********************  Bit definition for ADC_CSR register  *******************/
#define ADC_CSR_ADRDY_MST_Pos          (0U)                                    
#define ADC_CSR_ADRDY_MST_Msk          (0x1U << ADC_CSR_ADRDY_MST_Pos)         /*!< 0x00000001 */
#define ADC_CSR_ADRDY_MST              ADC_CSR_ADRDY_MST_Msk                   /*!< ADC multimode master ready flag */
#define ADC_CSR_EOSMP_MST_Pos          (1U)                                    
#define ADC_CSR_EOSMP_MST_Msk          (0x1U << ADC_CSR_EOSMP_MST_Pos)         /*!< 0x00000002 */
#define ADC_CSR_EOSMP_MST              ADC_CSR_EOSMP_MST_Msk                   /*!< ADC multimode master group regular end of sampling flag */
#define ADC_CSR_EOC_MST_Pos            (2U)                                    
#define ADC_CSR_EOC_MST_Msk            (0x1U << ADC_CSR_EOC_MST_Pos)           /*!< 0x00000004 */
#define ADC_CSR_EOC_MST                ADC_CSR_EOC_MST_Msk                     /*!< ADC multimode master group regular end of unitary conversion flag */
#define ADC_CSR_EOS_MST_Pos            (3U)                                    
#define ADC_CSR_EOS_MST_Msk            (0x1U << ADC_CSR_EOS_MST_Pos)           /*!< 0x00000008 */
#define ADC_CSR_EOS_MST                ADC_CSR_EOS_MST_Msk                     /*!< ADC multimode master group regular end of sequence conversions flag */
#define ADC_CSR_OVR_MST_Pos            (4U)                                    
#define ADC_CSR_OVR_MST_Msk            (0x1U << ADC_CSR_OVR_MST_Pos)           /*!< 0x00000010 */
#define ADC_CSR_OVR_MST                ADC_CSR_OVR_MST_Msk                     /*!< ADC multimode master group regular overrun flag */
#define ADC_CSR_JEOC_MST_Pos           (5U)                                    
#define ADC_CSR_JEOC_MST_Msk           (0x1U << ADC_CSR_JEOC_MST_Pos)          /*!< 0x00000020 */
#define ADC_CSR_JEOC_MST               ADC_CSR_JEOC_MST_Msk                    /*!< ADC multimode master group injected end of unitary conversion flag */
#define ADC_CSR_JEOS_MST_Pos           (6U)                                    
#define ADC_CSR_JEOS_MST_Msk           (0x1U << ADC_CSR_JEOS_MST_Pos)          /*!< 0x00000040 */
#define ADC_CSR_JEOS_MST               ADC_CSR_JEOS_MST_Msk                    /*!< ADC multimode master group injected end of sequence conversions flag */
#define ADC_CSR_AWD1_MST_Pos           (7U)                                    
#define ADC_CSR_AWD1_MST_Msk           (0x1U << ADC_CSR_AWD1_MST_Pos)          /*!< 0x00000080 */
#define ADC_CSR_AWD1_MST               ADC_CSR_AWD1_MST_Msk                    /*!< ADC multimode master analog watchdog 1 flag */
#define ADC_CSR_AWD2_MST_Pos           (8U)                                    
#define ADC_CSR_AWD2_MST_Msk           (0x1U << ADC_CSR_AWD2_MST_Pos)          /*!< 0x00000100 */
#define ADC_CSR_AWD2_MST               ADC_CSR_AWD2_MST_Msk                    /*!< ADC multimode master analog watchdog 2 flag */
#define ADC_CSR_AWD3_MST_Pos           (9U)                                    
#define ADC_CSR_AWD3_MST_Msk           (0x1U << ADC_CSR_AWD3_MST_Pos)          /*!< 0x00000200 */
#define ADC_CSR_AWD3_MST               ADC_CSR_AWD3_MST_Msk                    /*!< ADC multimode master analog watchdog 3 flag */
#define ADC_CSR_JQOVF_MST_Pos          (10U)                                   
#define ADC_CSR_JQOVF_MST_Msk          (0x1U << ADC_CSR_JQOVF_MST_Pos)         /*!< 0x00000400 */
#define ADC_CSR_JQOVF_MST              ADC_CSR_JQOVF_MST_Msk                   /*!< ADC multimode master group injected contexts queue overflow flag */

#define ADC_CSR_ADRDY_SLV_Pos          (16U)                                   
#define ADC_CSR_ADRDY_SLV_Msk          (0x1U << ADC_CSR_ADRDY_SLV_Pos)         /*!< 0x00010000 */
#define ADC_CSR_ADRDY_SLV              ADC_CSR_ADRDY_SLV_Msk                   /*!< ADC multimode slave ready flag */
#define ADC_CSR_EOSMP_SLV_Pos          (17U)                                   
#define ADC_CSR_EOSMP_SLV_Msk          (0x1U << ADC_CSR_EOSMP_SLV_Pos)         /*!< 0x00020000 */
#define ADC_CSR_EOSMP_SLV              ADC_CSR_EOSMP_SLV_Msk                   /*!< ADC multimode slave group regular end of sampling flag */
#define ADC_CSR_EOC_SLV_Pos            (18U)                                   
#define ADC_CSR_EOC_SLV_Msk            (0x1U << ADC_CSR_EOC_SLV_Pos)           /*!< 0x00040000 */
#define ADC_CSR_EOC_SLV                ADC_CSR_EOC_SLV_Msk                     /*!< ADC multimode slave group regular end of unitary conversion flag */
#define ADC_CSR_EOS_SLV_Pos            (19U)                                   
#define ADC_CSR_EOS_SLV_Msk            (0x1U << ADC_CSR_EOS_SLV_Pos)           /*!< 0x00080000 */
#define ADC_CSR_EOS_SLV                ADC_CSR_EOS_SLV_Msk                     /*!< ADC multimode slave group regular end of sequence conversions flag */
#define ADC_CSR_OVR_SLV_Pos            (20U)                                   
#define ADC_CSR_OVR_SLV_Msk            (0x1U << ADC_CSR_OVR_SLV_Pos)           /*!< 0x00100000 */
#define ADC_CSR_OVR_SLV                ADC_CSR_OVR_SLV_Msk                     /*!< ADC multimode slave group regular overrun flag */
#define ADC_CSR_JEOC_SLV_Pos           (21U)                                   
#define ADC_CSR_JEOC_SLV_Msk           (0x1U << ADC_CSR_JEOC_SLV_Pos)          /*!< 0x00200000 */
#define ADC_CSR_JEOC_SLV               ADC_CSR_JEOC_SLV_Msk                    /*!< ADC multimode slave group injected end of unitary conversion flag */
#define ADC_CSR_JEOS_SLV_Pos           (22U)                                   
#define ADC_CSR_JEOS_SLV_Msk           (0x1U << ADC_CSR_JEOS_SLV_Pos)          /*!< 0x00400000 */
#define ADC_CSR_JEOS_SLV               ADC_CSR_JEOS_SLV_Msk                    /*!< ADC multimode slave group injected end of sequence conversions flag */
#define ADC_CSR_AWD1_SLV_Pos           (23U)                                   
#define ADC_CSR_AWD1_SLV_Msk           (0x1U << ADC_CSR_AWD1_SLV_Pos)          /*!< 0x00800000 */
#define ADC_CSR_AWD1_SLV               ADC_CSR_AWD1_SLV_Msk                    /*!< ADC multimode slave analog watchdog 1 flag */
#define ADC_CSR_AWD2_SLV_Pos           (24U)                                   
#define ADC_CSR_AWD2_SLV_Msk           (0x1U << ADC_CSR_AWD2_SLV_Pos)          /*!< 0x01000000 */
#define ADC_CSR_AWD2_SLV               ADC_CSR_AWD2_SLV_Msk                    /*!< ADC multimode slave analog watchdog 2 flag */
#define ADC_CSR_AWD3_SLV_Pos           (25U)                                   
#define ADC_CSR_AWD3_SLV_Msk           (0x1U << ADC_CSR_AWD3_SLV_Pos)          /*!< 0x02000000 */
#define ADC_CSR_AWD3_SLV               ADC_CSR_AWD3_SLV_Msk                    /*!< ADC multimode slave analog watchdog 3 flag */
#define ADC_CSR_JQOVF_SLV_Pos          (26U)                                   
#define ADC_CSR_JQOVF_SLV_Msk          (0x1U << ADC_CSR_JQOVF_SLV_Pos)         /*!< 0x04000000 */
#define ADC_CSR_JQOVF_SLV              ADC_CSR_JQOVF_SLV_Msk                   /*!< ADC multimode slave group injected contexts queue overflow flag */

/* Legacy defines */
#define ADC_CSR_ADRDY_EOSMP_MST   ADC_CSR_EOSMP_MST
#define ADC_CSR_ADRDY_EOC_MST     ADC_CSR_EOC_MST
#define ADC_CSR_ADRDY_EOS_MST     ADC_CSR_EOS_MST
#define ADC_CSR_ADRDY_OVR_MST     ADC_CSR_OVR_MST
#define ADC_CSR_ADRDY_JEOC_MST    ADC_CSR_JEOC_MST
#define ADC_CSR_ADRDY_JEOS_MST    ADC_CSR_JEOS_MST

#define ADC_CSR_ADRDY_EOSMP_SLV   ADC_CSR_EOSMP_SLV
#define ADC_CSR_ADRDY_EOC_SLV     ADC_CSR_EOC_SLV
#define ADC_CSR_ADRDY_EOS_SLV     ADC_CSR_EOS_SLV
#define ADC_CSR_ADRDY_OVR_SLV     ADC_CSR_OVR_SLV
#define ADC_CSR_ADRDY_JEOC_SLV    ADC_CSR_JEOC_SLV
#define ADC_CSR_ADRDY_JEOS_SLV    ADC_CSR_JEOS_SLV

/********************  Bit definition for ADC_CCR register  *******************/
#define ADC_CCR_DUAL_Pos               (0U)                                    
#define ADC_CCR_DUAL_Msk               (0x1FU << ADC_CCR_DUAL_Pos)             /*!< 0x0000001F */
#define ADC_CCR_DUAL                   ADC_CCR_DUAL_Msk                        /*!< ADC multimode mode selection */
#define ADC_CCR_DUAL_0                 (0x01U << ADC_CCR_DUAL_Pos)             /*!< 0x00000001 */
#define ADC_CCR_DUAL_1                 (0x02U << ADC_CCR_DUAL_Pos)             /*!< 0x00000002 */
#define ADC_CCR_DUAL_2                 (0x04U << ADC_CCR_DUAL_Pos)             /*!< 0x00000004 */
#define ADC_CCR_DUAL_3                 (0x08U << ADC_CCR_DUAL_Pos)             /*!< 0x00000008 */
#define ADC_CCR_DUAL_4                 (0x10U << ADC_CCR_DUAL_Pos)             /*!< 0x00000010 */

#define ADC_CCR_DELAY_Pos              (8U)                                    
#define ADC_CCR_DELAY_Msk              (0xFU << ADC_CCR_DELAY_Pos)             /*!< 0x00000F00 */
#define ADC_CCR_DELAY                  ADC_CCR_DELAY_Msk                       /*!< ADC multimode delay between 2 sampling phases */
#define ADC_CCR_DELAY_0                (0x1U << ADC_CCR_DELAY_Pos)             /*!< 0x00000100 */
#define ADC_CCR_DELAY_1                (0x2U << ADC_CCR_DELAY_Pos)             /*!< 0x00000200 */
#define ADC_CCR_DELAY_2                (0x4U << ADC_CCR_DELAY_Pos)             /*!< 0x00000400 */
#define ADC_CCR_DELAY_3                (0x8U << ADC_CCR_DELAY_Pos)             /*!< 0x00000800 */

#define ADC_CCR_DMACFG_Pos             (13U)                                   
#define ADC_CCR_DMACFG_Msk             (0x1U << ADC_CCR_DMACFG_Pos)            /*!< 0x00002000 */
#define ADC_CCR_DMACFG                 ADC_CCR_DMACFG_Msk                      /*!< ADC multimode DMA transfer configuration */

#define ADC_CCR_MDMA_Pos               (14U)                                   
#define ADC_CCR_MDMA_Msk               (0x3U << ADC_CCR_MDMA_Pos)              /*!< 0x0000C000 */
#define ADC_CCR_MDMA                   ADC_CCR_MDMA_Msk                        /*!< ADC multimode DMA transfer enable */
#define ADC_CCR_MDMA_0                 (0x1U << ADC_CCR_MDMA_Pos)              /*!< 0x00004000 */
#define ADC_CCR_MDMA_1                 (0x2U << ADC_CCR_MDMA_Pos)              /*!< 0x00008000 */

#define ADC_CCR_CKMODE_Pos             (16U)                                   
#define ADC_CCR_CKMODE_Msk             (0x3U << ADC_CCR_CKMODE_Pos)            /*!< 0x00030000 */
#define ADC_CCR_CKMODE                 ADC_CCR_CKMODE_Msk                      /*!< ADC common clock source and prescaler (prescaler only for clock source synchronous) */
#define ADC_CCR_CKMODE_0               (0x1U << ADC_CCR_CKMODE_Pos)            /*!< 0x00010000 */
#define ADC_CCR_CKMODE_1               (0x2U << ADC_CCR_CKMODE_Pos)            /*!< 0x00020000 */

#define ADC_CCR_VREFEN_Pos             (22U)                                   
#define ADC_CCR_VREFEN_Msk             (0x1U << ADC_CCR_VREFEN_Pos)            /*!< 0x00400000 */
#define ADC_CCR_VREFEN                 ADC_CCR_VREFEN_Msk                      /*!< ADC internal path to VrefInt enable */
#define ADC_CCR_TSEN_Pos               (23U)                                   
#define ADC_CCR_TSEN_Msk               (0x1U << ADC_CCR_TSEN_Pos)              /*!< 0x00800000 */
#define ADC_CCR_TSEN                   ADC_CCR_TSEN_Msk                        /*!< ADC internal path to temperature sensor enable */
#define ADC_CCR_VBATEN_Pos             (24U)                                   
#define ADC_CCR_VBATEN_Msk             (0x1U << ADC_CCR_VBATEN_Pos)            /*!< 0x01000000 */
#define ADC_CCR_VBATEN                 ADC_CCR_VBATEN_Msk                      /*!< ADC internal path to battery voltage enable */

/* Legacy defines */
#define ADC_CCR_MULTI           (ADC_CCR_DUAL)
#define ADC_CCR_MULTI_0         (ADC_CCR_DUAL_0)
#define ADC_CCR_MULTI_1         (ADC_CCR_DUAL_1)
#define ADC_CCR_MULTI_2         (ADC_CCR_DUAL_2)
#define ADC_CCR_MULTI_3         (ADC_CCR_DUAL_3)
#define ADC_CCR_MULTI_4         (ADC_CCR_DUAL_4)

/********************  Bit definition for ADC_CDR register  *******************/
#define ADC_CDR_RDATA_MST_Pos          (0U)                                    
#define ADC_CDR_RDATA_MST_Msk          (0xFFFFU << ADC_CDR_RDATA_MST_Pos)      /*!< 0x0000FFFF */
#define ADC_CDR_RDATA_MST              ADC_CDR_RDATA_MST_Msk                   /*!< ADC multimode master group regular conversion data */
#define ADC_CDR_RDATA_MST_0            (0x0001U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000001 */
#define ADC_CDR_RDATA_MST_1            (0x0002U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000002 */
#define ADC_CDR_RDATA_MST_2            (0x0004U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000004 */
#define ADC_CDR_RDATA_MST_3            (0x0008U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000008 */
#define ADC_CDR_RDATA_MST_4            (0x0010U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000010 */
#define ADC_CDR_RDATA_MST_5            (0x0020U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000020 */
#define ADC_CDR_RDATA_MST_6            (0x0040U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000040 */
#define ADC_CDR_RDATA_MST_7            (0x0080U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000080 */
#define ADC_CDR_RDATA_MST_8            (0x0100U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000100 */
#define ADC_CDR_RDATA_MST_9            (0x0200U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000200 */
#define ADC_CDR_RDATA_MST_10           (0x0400U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000400 */
#define ADC_CDR_RDATA_MST_11           (0x0800U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00000800 */
#define ADC_CDR_RDATA_MST_12           (0x1000U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00001000 */
#define ADC_CDR_RDATA_MST_13           (0x2000U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00002000 */
#define ADC_CDR_RDATA_MST_14           (0x4000U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00004000 */
#define ADC_CDR_RDATA_MST_15           (0x8000U << ADC_CDR_RDATA_MST_Pos)      /*!< 0x00008000 */

#define ADC_CDR_RDATA_SLV_Pos          (16U)                                   
#define ADC_CDR_RDATA_SLV_Msk          (0xFFFFU << ADC_CDR_RDATA_SLV_Pos)      /*!< 0xFFFF0000 */
#define ADC_CDR_RDATA_SLV              ADC_CDR_RDATA_SLV_Msk                   /*!< ADC multimode slave group regular conversion data */
#define ADC_CDR_RDATA_SLV_0            (0x0001U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00010000 */
#define ADC_CDR_RDATA_SLV_1            (0x0002U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00020000 */
#define ADC_CDR_RDATA_SLV_2            (0x0004U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00040000 */
#define ADC_CDR_RDATA_SLV_3            (0x0008U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00080000 */
#define ADC_CDR_RDATA_SLV_4            (0x0010U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00100000 */
#define ADC_CDR_RDATA_SLV_5            (0x0020U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00200000 */
#define ADC_CDR_RDATA_SLV_6            (0x0040U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00400000 */
#define ADC_CDR_RDATA_SLV_7            (0x0080U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x00800000 */
#define ADC_CDR_RDATA_SLV_8            (0x0100U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x01000000 */
#define ADC_CDR_RDATA_SLV_9            (0x0200U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x02000000 */
#define ADC_CDR_RDATA_SLV_10           (0x0400U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x04000000 */
#define ADC_CDR_RDATA_SLV_11           (0x0800U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x08000000 */
#define ADC_CDR_RDATA_SLV_12           (0x1000U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x10000000 */
#define ADC_CDR_RDATA_SLV_13           (0x2000U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x20000000 */
#define ADC_CDR_RDATA_SLV_14           (0x4000U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x40000000 */
#define ADC_CDR_RDATA_SLV_15           (0x8000U << ADC_CDR_RDATA_SLV_Pos)      /*!< 0x80000000 */

/******************************************************************************/
/*                                                                            */
/*                      Analog Comparators (COMP)                             */
/*                                                                            */
/******************************************************************************/

#define COMP_V1_3_0_0                                  /*!< Comparator IP version */

/**********************  Bit definition for COMP2_CSR register  ***************/
#define COMP2_CSR_COMP2EN_Pos            (0U)                                  
#define COMP2_CSR_COMP2EN_Msk            (0x1U << COMP2_CSR_COMP2EN_Pos)       /*!< 0x00000001 */
#define COMP2_CSR_COMP2EN                COMP2_CSR_COMP2EN_Msk                 /*!< COMP2 enable */
#define COMP2_CSR_COMP2SW1_Pos           (1U)                                   
#define COMP2_CSR_COMP2SW1_Msk           (0x1U << COMP2_CSR_COMP2SW1_Pos)      /*!< 0x00000002 */
#define COMP2_CSR_COMP2SW1               COMP2_CSR_COMP2SW1_Msk                /*!< COMP2 SW1 switch control */
/* Legacy defines */
#define COMP_CSR_COMP2SW1                COMP2_CSR_COMP2SW1
#define COMP2_CSR_COMP2INPDAC_Pos        (1U)                                  
#define COMP2_CSR_COMP2INPDAC_Msk        (0x1U << COMP2_CSR_COMP2INPDAC_Pos)   /*!< 0x00000002 */
#define COMP2_CSR_COMP2INPDAC            COMP2_CSR_COMP2INPDAC_Msk             /*!< COMP2 non inverting input to DAC output */
#define COMP2_CSR_COMP2INSEL_Pos         (4U)                                  
#define COMP2_CSR_COMP2INSEL_Msk         (0x7U << COMP2_CSR_COMP2INSEL_Pos)    /*!< 0x00000070 */
#define COMP2_CSR_COMP2INSEL             COMP2_CSR_COMP2INSEL_Msk              /*!< COMP2 inverting input select */
#define COMP2_CSR_COMP2INSEL_0           (0x00000010U)                         /*!< COMP2 inverting input select bit 0 */
#define COMP2_CSR_COMP2INSEL_1           (0x00000020U)                         /*!< COMP2 inverting input select bit 1 */
#define COMP2_CSR_COMP2INSEL_2           (0x00000040U)                         /*!< COMP2 inverting input select bit 2 */
#define COMP2_CSR_COMP2OUTSEL_Pos        (10U)                                 
#define COMP2_CSR_COMP2OUTSEL_Msk        (0xFU << COMP2_CSR_COMP2OUTSEL_Pos)   /*!< 0x00003C00 */
#define COMP2_CSR_COMP2OUTSEL            COMP2_CSR_COMP2OUTSEL_Msk             /*!< COMP2 output select */
#define COMP2_CSR_COMP2OUTSEL_0          (0x1U << COMP2_CSR_COMP2OUTSEL_Pos)   /*!< 0x00000400 */
#define COMP2_CSR_COMP2OUTSEL_1          (0x2U << COMP2_CSR_COMP2OUTSEL_Pos)   /*!< 0x00000800 */
#define COMP2_CSR_COMP2OUTSEL_2          (0x4U << COMP2_CSR_COMP2OUTSEL_Pos)   /*!< 0x00001000 */
#define COMP2_CSR_COMP2OUTSEL_3          (0x8U << COMP2_CSR_COMP2OUTSEL_Pos)   /*!< 0x00002000 */
#define COMP2_CSR_COMP2POL_Pos           (15U)                                 
#define COMP2_CSR_COMP2POL_Msk           (0x1U << COMP2_CSR_COMP2POL_Pos)      /*!< 0x00008000 */
#define COMP2_CSR_COMP2POL               COMP2_CSR_COMP2POL_Msk                /*!< COMP2 output polarity */
#define COMP2_CSR_COMP2BLANKING_Pos      (18U)                                 
#define COMP2_CSR_COMP2BLANKING_Msk      (0x3U << COMP2_CSR_COMP2BLANKING_Pos) /*!< 0x000C0000 */
#define COMP2_CSR_COMP2BLANKING          COMP2_CSR_COMP2BLANKING_Msk           /*!< COMP2 blanking */
#define COMP2_CSR_COMP2BLANKING_0        (0x1U << COMP2_CSR_COMP2BLANKING_Pos) /*!< 0x00040000 */
#define COMP2_CSR_COMP2BLANKING_1        (0x2U << COMP2_CSR_COMP2BLANKING_Pos) /*!< 0x00080000 */
#define COMP2_CSR_COMP2BLANKING_2        (0x4U << COMP2_CSR_COMP2BLANKING_Pos) /*!< 0x00100000 */
#define COMP2_CSR_COMP2OUT_Pos           (30U)                                 
#define COMP2_CSR_COMP2OUT_Msk           (0x1U << COMP2_CSR_COMP2OUT_Pos)      /*!< 0x40000000 */
#define COMP2_CSR_COMP2OUT               COMP2_CSR_COMP2OUT_Msk                /*!< COMP2 output level */
#define COMP2_CSR_COMP2LOCK_Pos          (31U)                                 
#define COMP2_CSR_COMP2LOCK_Msk          (0x1U << COMP2_CSR_COMP2LOCK_Pos)     /*!< 0x80000000 */
#define COMP2_CSR_COMP2LOCK              COMP2_CSR_COMP2LOCK_Msk               /*!< COMP2 lock */

/**********************  Bit definition for COMP4_CSR register  ***************/
#define COMP4_CSR_COMP4EN_Pos            (0U)                                  
#define COMP4_CSR_COMP4EN_Msk            (0x1U << COMP4_CSR_COMP4EN_Pos)       /*!< 0x00000001 */
#define COMP4_CSR_COMP4EN                COMP4_CSR_COMP4EN_Msk                 /*!< COMP4 enable */
#define COMP4_CSR_COMP4INSEL_Pos         (4U)                                  
#define COMP4_CSR_COMP4INSEL_Msk         (0x7U << COMP4_CSR_COMP4INSEL_Pos)    /*!< 0x00000070 */
#define COMP4_CSR_COMP4INSEL             COMP4_CSR_COMP4INSEL_Msk              /*!< COMP4 inverting input select */
#define COMP4_CSR_COMP4INSEL_0           (0x00000010U)                         /*!< COMP4 inverting input select bit 0 */
#define COMP4_CSR_COMP4INSEL_1           (0x00000020U)                         /*!< COMP4 inverting input select bit 1 */
#define COMP4_CSR_COMP4INSEL_2           (0x00000040U)                         /*!< COMP4 inverting input select bit 2 */
#define COMP4_CSR_COMP4OUTSEL_Pos        (10U)                                 
#define COMP4_CSR_COMP4OUTSEL_Msk        (0xFU << COMP4_CSR_COMP4OUTSEL_Pos)   /*!< 0x00003C00 */
#define COMP4_CSR_COMP4OUTSEL            COMP4_CSR_COMP4OUTSEL_Msk             /*!< COMP4 output select */
#define COMP4_CSR_COMP4OUTSEL_0          (0x1U << COMP4_CSR_COMP4OUTSEL_Pos)   /*!< 0x00000400 */
#define COMP4_CSR_COMP4OUTSEL_1          (0x2U << COMP4_CSR_COMP4OUTSEL_Pos)   /*!< 0x00000800 */
#define COMP4_CSR_COMP4OUTSEL_2          (0x4U << COMP4_CSR_COMP4OUTSEL_Pos)   /*!< 0x00001000 */
#define COMP4_CSR_COMP4OUTSEL_3          (0x8U << COMP4_CSR_COMP4OUTSEL_Pos)   /*!< 0x00002000 */
#define COMP4_CSR_COMP4POL_Pos           (15U)                                 
#define COMP4_CSR_COMP4POL_Msk           (0x1U << COMP4_CSR_COMP4POL_Pos)      /*!< 0x00008000 */
#define COMP4_CSR_COMP4POL               COMP4_CSR_COMP4POL_Msk                /*!< COMP4 output polarity */
#define COMP4_CSR_COMP4BLANKING_Pos      (18U)                                 
#define COMP4_CSR_COMP4BLANKING_Msk      (0x3U << COMP4_CSR_COMP4BLANKING_Pos) /*!< 0x000C0000 */
#define COMP4_CSR_COMP4BLANKING          COMP4_CSR_COMP4BLANKING_Msk           /*!< COMP4 blanking */
#define COMP4_CSR_COMP4BLANKING_0        (0x1U << COMP4_CSR_COMP4BLANKING_Pos) /*!< 0x00040000 */
#define COMP4_CSR_COMP4BLANKING_1        (0x2U << COMP4_CSR_COMP4BLANKING_Pos) /*!< 0x00080000 */
#define COMP4_CSR_COMP4BLANKING_2        (0x4U << COMP4_CSR_COMP4BLANKING_Pos) /*!< 0x00100000 */
#define COMP4_CSR_COMP4OUT_Pos           (30U)                                 
#define COMP4_CSR_COMP4OUT_Msk           (0x1U << COMP4_CSR_COMP4OUT_Pos)      /*!< 0x40000000 */
#define COMP4_CSR_COMP4OUT               COMP4_CSR_COMP4OUT_Msk                /*!< COMP4 output level */
#define COMP4_CSR_COMP4LOCK_Pos          (31U)                                 
#define COMP4_CSR_COMP4LOCK_Msk          (0x1U << COMP4_CSR_COMP4LOCK_Pos)     /*!< 0x80000000 */
#define COMP4_CSR_COMP4LOCK              COMP4_CSR_COMP4LOCK_Msk               /*!< COMP4 lock */

/**********************  Bit definition for COMP6_CSR register  ***************/
#define COMP6_CSR_COMP6EN_Pos            (0U)                                  
#define COMP6_CSR_COMP6EN_Msk            (0x1U << COMP6_CSR_COMP6EN_Pos)       /*!< 0x00000001 */
#define COMP6_CSR_COMP6EN                COMP6_CSR_COMP6EN_Msk                 /*!< COMP6 enable */
#define COMP6_CSR_COMP6INSEL_Pos         (4U)                                  
#define COMP6_CSR_COMP6INSEL_Msk         (0x7U << COMP6_CSR_COMP6INSEL_Pos)    /*!< 0x00000070 */
#define COMP6_CSR_COMP6INSEL             COMP6_CSR_COMP6INSEL_Msk              /*!< COMP6 inverting input select */
#define COMP6_CSR_COMP6INSEL_0           (0x00000010U)                         /*!< COMP6 inverting input select bit 0 */
#define COMP6_CSR_COMP6INSEL_1           (0x00000020U)                         /*!< COMP6 inverting input select bit 1 */
#define COMP6_CSR_COMP6INSEL_2           (0x00000040U)                         /*!< COMP6 inverting input select bit 2 */
#define COMP6_CSR_COMP6OUTSEL_Pos        (10U)                                 
#define COMP6_CSR_COMP6OUTSEL_Msk        (0xFU << COMP6_CSR_COMP6OUTSEL_Pos)   /*!< 0x00003C00 */
#define COMP6_CSR_COMP6OUTSEL            COMP6_CSR_COMP6OUTSEL_Msk             /*!< COMP6 output select */
#define COMP6_CSR_COMP6OUTSEL_0          (0x1U << COMP6_CSR_COMP6OUTSEL_Pos)   /*!< 0x00000400 */
#define COMP6_CSR_COMP6OUTSEL_1          (0x2U << COMP6_CSR_COMP6OUTSEL_Pos)   /*!< 0x00000800 */
#define COMP6_CSR_COMP6OUTSEL_2          (0x4U << COMP6_CSR_COMP6OUTSEL_Pos)   /*!< 0x00001000 */
#define COMP6_CSR_COMP6OUTSEL_3          (0x8U << COMP6_CSR_COMP6OUTSEL_Pos)   /*!< 0x00002000 */
#define COMP6_CSR_COMP6POL_Pos           (15U)                                 
#define COMP6_CSR_COMP6POL_Msk           (0x1U << COMP6_CSR_COMP6POL_Pos)      /*!< 0x00008000 */
#define COMP6_CSR_COMP6POL               COMP6_CSR_COMP6POL_Msk                /*!< COMP6 output polarity */
#define COMP6_CSR_COMP6BLANKING_Pos      (18U)                                 
#define COMP6_CSR_COMP6BLANKING_Msk      (0x3U << COMP6_CSR_COMP6BLANKING_Pos) /*!< 0x000C0000 */
#define COMP6_CSR_COMP6BLANKING          COMP6_CSR_COMP6BLANKING_Msk           /*!< COMP6 blanking */
#define COMP6_CSR_COMP6BLANKING_0        (0x1U << COMP6_CSR_COMP6BLANKING_Pos) /*!< 0x00040000 */
#define COMP6_CSR_COMP6BLANKING_1        (0x2U << COMP6_CSR_COMP6BLANKING_Pos) /*!< 0x00080000 */
#define COMP6_CSR_COMP6BLANKING_2        (0x4U << COMP6_CSR_COMP6BLANKING_Pos) /*!< 0x00100000 */
#define COMP6_CSR_COMP6OUT_Pos           (30U)                                 
#define COMP6_CSR_COMP6OUT_Msk           (0x1U << COMP6_CSR_COMP6OUT_Pos)      /*!< 0x40000000 */
#define COMP6_CSR_COMP6OUT               COMP6_CSR_COMP6OUT_Msk                /*!< COMP6 output level */
#define COMP6_CSR_COMP6LOCK_Pos          (31U)                                 
#define COMP6_CSR_COMP6LOCK_Msk          (0x1U << COMP6_CSR_COMP6LOCK_Pos)     /*!< 0x80000000 */
#define COMP6_CSR_COMP6LOCK              COMP6_CSR_COMP6LOCK_Msk               /*!< COMP6 lock */

/**********************  Bit definition for COMP_CSR register  ****************/
#define COMP_CSR_COMPxEN_Pos            (0U)                                   
#define COMP_CSR_COMPxEN_Msk            (0x1U << COMP_CSR_COMPxEN_Pos)         /*!< 0x00000001 */
#define COMP_CSR_COMPxEN                COMP_CSR_COMPxEN_Msk                   /*!< COMPx enable */
#define COMP_CSR_COMPxSW1_Pos           (1U)                                   
#define COMP_CSR_COMPxSW1_Msk           (0x1U << COMP_CSR_COMPxSW1_Pos)        /*!< 0x00000002 */
#define COMP_CSR_COMPxSW1               COMP_CSR_COMPxSW1_Msk                  /*!< COMPx SW1 switch control */
#define COMP_CSR_COMPxINSEL_Pos         (4U)                                   
#define COMP_CSR_COMPxINSEL_Msk         (0x7U << COMP_CSR_COMPxINSEL_Pos)      /*!< 0x00000070 */
#define COMP_CSR_COMPxINSEL             COMP_CSR_COMPxINSEL_Msk                /*!< COMPx inverting input select */
#define COMP_CSR_COMPxINSEL_0           (0x00000010U)                          /*!< COMPx inverting input select bit 0 */
#define COMP_CSR_COMPxINSEL_1           (0x00000020U)                          /*!< COMPx inverting input select bit 1 */
#define COMP_CSR_COMPxINSEL_2           (0x00000040U)                          /*!< COMPx inverting input select bit 2 */
#define COMP_CSR_COMPxOUTSEL_Pos        (10U)                                  
#define COMP_CSR_COMPxOUTSEL_Msk        (0xFU << COMP_CSR_COMPxOUTSEL_Pos)     /*!< 0x00003C00 */
#define COMP_CSR_COMPxOUTSEL            COMP_CSR_COMPxOUTSEL_Msk               /*!< COMPx output select */
#define COMP_CSR_COMPxOUTSEL_0          (0x1U << COMP_CSR_COMPxOUTSEL_Pos)     /*!< 0x00000400 */
#define COMP_CSR_COMPxOUTSEL_1          (0x2U << COMP_CSR_COMPxOUTSEL_Pos)     /*!< 0x00000800 */
#define COMP_CSR_COMPxOUTSEL_2          (0x4U << COMP_CSR_COMPxOUTSEL_Pos)     /*!< 0x00001000 */
#define COMP_CSR_COMPxOUTSEL_3          (0x8U << COMP_CSR_COMPxOUTSEL_Pos)     /*!< 0x00002000 */
#define COMP_CSR_COMPxPOL_Pos           (15U)                                  
#define COMP_CSR_COMPxPOL_Msk           (0x1U << COMP_CSR_COMPxPOL_Pos)        /*!< 0x00008000 */
#define COMP_CSR_COMPxPOL               COMP_CSR_COMPxPOL_Msk                  /*!< COMPx output polarity */
#define COMP_CSR_COMPxBLANKING_Pos      (18U)                                  
#define COMP_CSR_COMPxBLANKING_Msk      (0x3U << COMP_CSR_COMPxBLANKING_Pos)   /*!< 0x000C0000 */
#define COMP_CSR_COMPxBLANKING          COMP_CSR_COMPxBLANKING_Msk             /*!< COMPx blanking */
#define COMP_CSR_COMPxBLANKING_0        (0x1U << COMP_CSR_COMPxBLANKING_Pos)   /*!< 0x00040000 */
#define COMP_CSR_COMPxBLANKING_1        (0x2U << COMP_CSR_COMPxBLANKING_Pos)   /*!< 0x00080000 */
#define COMP_CSR_COMPxBLANKING_2        (0x4U << COMP_CSR_COMPxBLANKING_Pos)   /*!< 0x00100000 */
#define COMP_CSR_COMPxOUT_Pos           (30U)                                  
#define COMP_CSR_COMPxOUT_Msk           (0x1U << COMP_CSR_COMPxOUT_Pos)        /*!< 0x40000000 */
#define COMP_CSR_COMPxOUT               COMP_CSR_COMPxOUT_Msk                  /*!< COMPx output level */
#define COMP_CSR_COMPxLOCK_Pos          (31U)                                  
#define COMP_CSR_COMPxLOCK_Msk          (0x1U << COMP_CSR_COMPxLOCK_Pos)       /*!< 0x80000000 */
#define COMP_CSR_COMPxLOCK              COMP_CSR_COMPxLOCK_Msk                 /*!< COMPx lock */

/******************************************************************************/
/*                                                                            */
/*                     Operational Amplifier (OPAMP)                          */
/*                                                                            */
/******************************************************************************/
/*********************  Bit definition for OPAMP2_CSR register  ***************/
#define OPAMP2_CSR_OPAMP2EN_Pos       (0U)                                     
#define OPAMP2_CSR_OPAMP2EN_Msk       (0x1U << OPAMP2_CSR_OPAMP2EN_Pos)        /*!< 0x00000001 */
#define OPAMP2_CSR_OPAMP2EN           OPAMP2_CSR_OPAMP2EN_Msk                  /*!< OPAMP2 enable */
#define OPAMP2_CSR_FORCEVP_Pos        (1U)                                     
#define OPAMP2_CSR_FORCEVP_Msk        (0x1U << OPAMP2_CSR_FORCEVP_Pos)         /*!< 0x00000002 */
#define OPAMP2_CSR_FORCEVP            OPAMP2_CSR_FORCEVP_Msk                   /*!< Connect the internal references to the plus input of the OPAMPX */
#define OPAMP2_CSR_VPSEL_Pos          (2U)                                     
#define OPAMP2_CSR_VPSEL_Msk          (0x3U << OPAMP2_CSR_VPSEL_Pos)           /*!< 0x0000000C */
#define OPAMP2_CSR_VPSEL              OPAMP2_CSR_VPSEL_Msk                     /*!< Non inverting input selection */
#define OPAMP2_CSR_VPSEL_0            (0x1U << OPAMP2_CSR_VPSEL_Pos)           /*!< 0x00000004 */
#define OPAMP2_CSR_VPSEL_1            (0x2U << OPAMP2_CSR_VPSEL_Pos)           /*!< 0x00000008 */
#define OPAMP2_CSR_VMSEL_Pos          (5U)                                     
#define OPAMP2_CSR_VMSEL_Msk          (0x3U << OPAMP2_CSR_VMSEL_Pos)           /*!< 0x00000060 */
#define OPAMP2_CSR_VMSEL              OPAMP2_CSR_VMSEL_Msk                     /*!< Inverting input selection */
#define OPAMP2_CSR_VMSEL_0            (0x1U << OPAMP2_CSR_VMSEL_Pos)           /*!< 0x00000020 */
#define OPAMP2_CSR_VMSEL_1            (0x2U << OPAMP2_CSR_VMSEL_Pos)           /*!< 0x00000040 */
#define OPAMP2_CSR_TCMEN_Pos          (7U)                                     
#define OPAMP2_CSR_TCMEN_Msk          (0x1U << OPAMP2_CSR_TCMEN_Pos)           /*!< 0x00000080 */
#define OPAMP2_CSR_TCMEN              OPAMP2_CSR_TCMEN_Msk                     /*!< Timer-Controlled Mux mode enable */
#define OPAMP2_CSR_VMSSEL_Pos         (8U)                                     
#define OPAMP2_CSR_VMSSEL_Msk         (0x1U << OPAMP2_CSR_VMSSEL_Pos)          /*!< 0x00000100 */
#define OPAMP2_CSR_VMSSEL             OPAMP2_CSR_VMSSEL_Msk                    /*!< Inverting input secondary selection */
#define OPAMP2_CSR_VPSSEL_Pos         (9U)                                     
#define OPAMP2_CSR_VPSSEL_Msk         (0x3U << OPAMP2_CSR_VPSSEL_Pos)          /*!< 0x00000600 */
#define OPAMP2_CSR_VPSSEL             OPAMP2_CSR_VPSSEL_Msk                    /*!< Non inverting input secondary selection */
#define OPAMP2_CSR_VPSSEL_0           (0x1U << OPAMP2_CSR_VPSSEL_Pos)          /*!< 0x00000200 */
#define OPAMP2_CSR_VPSSEL_1           (0x2U << OPAMP2_CSR_VPSSEL_Pos)          /*!< 0x00000400 */
#define OPAMP2_CSR_CALON_Pos          (11U)                                    
#define OPAMP2_CSR_CALON_Msk          (0x1U << OPAMP2_CSR_CALON_Pos)           /*!< 0x00000800 */
#define OPAMP2_CSR_CALON              OPAMP2_CSR_CALON_Msk                     /*!< Calibration mode enable */
#define OPAMP2_CSR_CALSEL_Pos         (12U)                                    
#define OPAMP2_CSR_CALSEL_Msk         (0x3U << OPAMP2_CSR_CALSEL_Pos)          /*!< 0x00003000 */
#define OPAMP2_CSR_CALSEL             OPAMP2_CSR_CALSEL_Msk                    /*!< Calibration selection */
#define OPAMP2_CSR_CALSEL_0           (0x1U << OPAMP2_CSR_CALSEL_Pos)          /*!< 0x00001000 */
#define OPAMP2_CSR_CALSEL_1           (0x2U << OPAMP2_CSR_CALSEL_Pos)          /*!< 0x00002000 */
#define OPAMP2_CSR_PGGAIN_Pos         (14U)                                    
#define OPAMP2_CSR_PGGAIN_Msk         (0xFU << OPAMP2_CSR_PGGAIN_Pos)          /*!< 0x0003C000 */
#define OPAMP2_CSR_PGGAIN             OPAMP2_CSR_PGGAIN_Msk                    /*!< Gain in PGA mode */
#define OPAMP2_CSR_PGGAIN_0           (0x1U << OPAMP2_CSR_PGGAIN_Pos)          /*!< 0x00004000 */
#define OPAMP2_CSR_PGGAIN_1           (0x2U << OPAMP2_CSR_PGGAIN_Pos)          /*!< 0x00008000 */
#define OPAMP2_CSR_PGGAIN_2           (0x4U << OPAMP2_CSR_PGGAIN_Pos)          /*!< 0x00010000 */
#define OPAMP2_CSR_PGGAIN_3           (0x8U << OPAMP2_CSR_PGGAIN_Pos)          /*!< 0x00020000 */
#define OPAMP2_CSR_USERTRIM_Pos       (18U)                                    
#define OPAMP2_CSR_USERTRIM_Msk       (0x1U << OPAMP2_CSR_USERTRIM_Pos)        /*!< 0x00040000 */
#define OPAMP2_CSR_USERTRIM           OPAMP2_CSR_USERTRIM_Msk                  /*!< User trimming enable */
#define OPAMP2_CSR_TRIMOFFSETP_Pos    (19U)                                    
#define OPAMP2_CSR_TRIMOFFSETP_Msk    (0x1FU << OPAMP2_CSR_TRIMOFFSETP_Pos)    /*!< 0x00F80000 */
#define OPAMP2_CSR_TRIMOFFSETP        OPAMP2_CSR_TRIMOFFSETP_Msk               /*!< Offset trimming value (PMOS) */
#define OPAMP2_CSR_TRIMOFFSETN_Pos    (24U)                                    
#define OPAMP2_CSR_TRIMOFFSETN_Msk    (0x1FU << OPAMP2_CSR_TRIMOFFSETN_Pos)    /*!< 0x1F000000 */
#define OPAMP2_CSR_TRIMOFFSETN        OPAMP2_CSR_TRIMOFFSETN_Msk               /*!< Offset trimming value (NMOS) */
#define OPAMP2_CSR_TSTREF_Pos         (29U)                                    
#define OPAMP2_CSR_TSTREF_Msk         (0x1U << OPAMP2_CSR_TSTREF_Pos)          /*!< 0x20000000 */
#define OPAMP2_CSR_TSTREF             OPAMP2_CSR_TSTREF_Msk                    /*!< It enables the switch to put out the internal reference */
#define OPAMP2_CSR_OUTCAL_Pos         (30U)                                    
#define OPAMP2_CSR_OUTCAL_Msk         (0x1U << OPAMP2_CSR_OUTCAL_Pos)          /*!< 0x40000000 */
#define OPAMP2_CSR_OUTCAL             OPAMP2_CSR_OUTCAL_Msk                    /*!< OPAMP ouput status flag */
#define OPAMP2_CSR_LOCK_Pos           (31U)                                    
#define OPAMP2_CSR_LOCK_Msk           (0x1U << OPAMP2_CSR_LOCK_Pos)            /*!< 0x80000000 */
#define OPAMP2_CSR_LOCK               OPAMP2_CSR_LOCK_Msk                      /*!< OPAMP lock */

/*********************  Bit definition for OPAMPx_CSR register  ***************/
#define OPAMP_CSR_OPAMPxEN_Pos       (0U)                                      
#define OPAMP_CSR_OPAMPxEN_Msk       (0x1U << OPAMP_CSR_OPAMPxEN_Pos)          /*!< 0x00000001 */
#define OPAMP_CSR_OPAMPxEN           OPAMP_CSR_OPAMPxEN_Msk                    /*!< OPAMP enable */
#define OPAMP_CSR_FORCEVP_Pos        (1U)                                      
#define OPAMP_CSR_FORCEVP_Msk        (0x1U << OPAMP_CSR_FORCEVP_Pos)           /*!< 0x00000002 */
#define OPAMP_CSR_FORCEVP            OPAMP_CSR_FORCEVP_Msk                     /*!< Connect the internal references to the plus input of the OPAMPX */
#define OPAMP_CSR_VPSEL_Pos          (2U)                                      
#define OPAMP_CSR_VPSEL_Msk          (0x3U << OPAMP_CSR_VPSEL_Pos)             /*!< 0x0000000C */
#define OPAMP_CSR_VPSEL              OPAMP_CSR_VPSEL_Msk                       /*!< Non inverting input selection */
#define OPAMP_CSR_VPSEL_0            (0x1U << OPAMP_CSR_VPSEL_Pos)             /*!< 0x00000004 */
#define OPAMP_CSR_VPSEL_1            (0x2U << OPAMP_CSR_VPSEL_Pos)             /*!< 0x00000008 */
#define OPAMP_CSR_VMSEL_Pos          (5U)                                      
#define OPAMP_CSR_VMSEL_Msk          (0x3U << OPAMP_CSR_VMSEL_Pos)             /*!< 0x00000060 */
#define OPAMP_CSR_VMSEL              OPAMP_CSR_VMSEL_Msk                       /*!< Inverting input selection */
#define OPAMP_CSR_VMSEL_0            (0x1U << OPAMP_CSR_VMSEL_Pos)             /*!< 0x00000020 */
#define OPAMP_CSR_VMSEL_1            (0x2U << OPAMP_CSR_VMSEL_Pos)             /*!< 0x00000040 */
#define OPAMP_CSR_TCMEN_Pos          (7U)                                      
#define OPAMP_CSR_TCMEN_Msk          (0x1U << OPAMP_CSR_TCMEN_Pos)             /*!< 0x00000080 */
#define OPAMP_CSR_TCMEN              OPAMP_CSR_TCMEN_Msk                       /*!< Timer-Controlled Mux mode enable */
#define OPAMP_CSR_VMSSEL_Pos         (8U)                                      
#define OPAMP_CSR_VMSSEL_Msk         (0x1U << OPAMP_CSR_VMSSEL_Pos)            /*!< 0x00000100 */
#define OPAMP_CSR_VMSSEL             OPAMP_CSR_VMSSEL_Msk                      /*!< Inverting input secondary selection */
#define OPAMP_CSR_VPSSEL_Pos         (9U)                                      
#define OPAMP_CSR_VPSSEL_Msk         (0x3U << OPAMP_CSR_VPSSEL_Pos)            /*!< 0x00000600 */
#define OPAMP_CSR_VPSSEL             OPAMP_CSR_VPSSEL_Msk                      /*!< Non inverting input secondary selection */
#define OPAMP_CSR_VPSSEL_0           (0x1U << OPAMP_CSR_VPSSEL_Pos)            /*!< 0x00000200 */
#define OPAMP_CSR_VPSSEL_1           (0x2U << OPAMP_CSR_VPSSEL_Pos)            /*!< 0x00000400 */
#define OPAMP_CSR_CALON_Pos          (11U)                                     
#define OPAMP_CSR_CALON_Msk          (0x1U << OPAMP_CSR_CALON_Pos)             /*!< 0x00000800 */
#define OPAMP_CSR_CALON              OPAMP_CSR_CALON_Msk                       /*!< Calibration mode enable */
#define OPAMP_CSR_CALSEL_Pos         (12U)                                     
#define OPAMP_CSR_CALSEL_Msk         (0x3U << OPAMP_CSR_CALSEL_Pos)            /*!< 0x00003000 */
#define OPAMP_CSR_CALSEL             OPAMP_CSR_CALSEL_Msk                      /*!< Calibration selection */
#define OPAMP_CSR_CALSEL_0           (0x1U << OPAMP_CSR_CALSEL_Pos)            /*!< 0x00001000 */
#define OPAMP_CSR_CALSEL_1           (0x2U << OPAMP_CSR_CALSEL_Pos)            /*!< 0x00002000 */
#define OPAMP_CSR_PGGAIN_Pos         (14U)                                     
#define OPAMP_CSR_PGGAIN_Msk         (0xFU << OPAMP_CSR_PGGAIN_Pos)            /*!< 0x0003C000 */
#define OPAMP_CSR_PGGAIN             OPAMP_CSR_PGGAIN_Msk                      /*!< Gain in PGA mode */
#define OPAMP_CSR_PGGAIN_0           (0x1U << OPAMP_CSR_PGGAIN_Pos)            /*!< 0x00004000 */
#define OPAMP_CSR_PGGAIN_1           (0x2U << OPAMP_CSR_PGGAIN_Pos)            /*!< 0x00008000 */
#define OPAMP_CSR_PGGAIN_2           (0x4U << OPAMP_CSR_PGGAIN_Pos)            /*!< 0x00010000 */
#define OPAMP_CSR_PGGAIN_3           (0x8U << OPAMP_CSR_PGGAIN_Pos)            /*!< 0x00020000 */
#define OPAMP_CSR_USERTRIM_Pos       (18U)                                     
#define OPAMP_CSR_USERTRIM_Msk       (0x1U << OPAMP_CSR_USERTRIM_Pos)          /*!< 0x00040000 */
#define OPAMP_CSR_USERTRIM           OPAMP_CSR_USERTRIM_Msk                    /*!< User trimming enable */
#define OPAMP_CSR_TRIMOFFSETP_Pos    (19U)                                     
#define OPAMP_CSR_TRIMOFFSETP_Msk    (0x1FU << OPAMP_CSR_TRIMOFFSETP_Pos)      /*!< 0x00F80000 */
#define OPAMP_CSR_TRIMOFFSETP        OPAMP_CSR_TRIMOFFSETP_Msk                 /*!< Offset trimming value (PMOS) */
#define OPAMP_CSR_TRIMOFFSETN_Pos    (24U)                                     
#define OPAMP_CSR_TRIMOFFSETN_Msk    (0x1FU << OPAMP_CSR_TRIMOFFSETN_Pos)      /*!< 0x1F000000 */
#define OPAMP_CSR_TRIMOFFSETN        OPAMP_CSR_TRIMOFFSETN_Msk                 /*!< Offset trimming value (NMOS) */
#define OPAMP_CSR_TSTREF_Pos         (29U)                                     
#define OPAMP_CSR_TSTREF_Msk         (0x1U << OPAMP_CSR_TSTREF_Pos)            /*!< 0x20000000 */
#define OPAMP_CSR_TSTREF             OPAMP_CSR_TSTREF_Msk                      /*!< It enables the switch to put out the internal reference */
#define OPAMP_CSR_OUTCAL_Pos         (30U)                                     
#define OPAMP_CSR_OUTCAL_Msk         (0x1U << OPAMP_CSR_OUTCAL_Pos)            /*!< 0x40000000 */
#define OPAMP_CSR_OUTCAL             OPAMP_CSR_OUTCAL_Msk                      /*!< OPAMP ouput status flag */
#define OPAMP_CSR_LOCK_Pos           (31U)                                     
#define OPAMP_CSR_LOCK_Msk           (0x1U << OPAMP_CSR_LOCK_Pos)              /*!< 0x80000000 */
#define OPAMP_CSR_LOCK               OPAMP_CSR_LOCK_Msk                        /*!< OPAMP lock */

/******************************************************************************/
/*                                                                            */
/*                     CRC calculation unit (CRC)                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define CRC_DR_DR_Pos            (0U)                                          
#define CRC_DR_DR_Msk            (0xFFFFFFFFU << CRC_DR_DR_Pos)                /*!< 0xFFFFFFFF */
#define CRC_DR_DR                CRC_DR_DR_Msk                                 /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define CRC_IDR_IDR              ((uint8_t)0xFFU)                              /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define CRC_CR_RESET_Pos         (0U)                                          
#define CRC_CR_RESET_Msk         (0x1U << CRC_CR_RESET_Pos)                    /*!< 0x00000001 */
#define CRC_CR_RESET             CRC_CR_RESET_Msk                              /*!< RESET the CRC computation unit bit */
#define CRC_CR_POLYSIZE_Pos      (3U)                                          
#define CRC_CR_POLYSIZE_Msk      (0x3U << CRC_CR_POLYSIZE_Pos)                 /*!< 0x00000018 */
#define CRC_CR_POLYSIZE          CRC_CR_POLYSIZE_Msk                           /*!< Polynomial size bits */
#define CRC_CR_POLYSIZE_0        (0x1U << CRC_CR_POLYSIZE_Pos)                 /*!< 0x00000008 */
#define CRC_CR_POLYSIZE_1        (0x2U << CRC_CR_POLYSIZE_Pos)                 /*!< 0x00000010 */
#define CRC_CR_REV_IN_Pos        (5U)                                          
#define CRC_CR_REV_IN_Msk        (0x3U << CRC_CR_REV_IN_Pos)                   /*!< 0x00000060 */
#define CRC_CR_REV_IN            CRC_CR_REV_IN_Msk                             /*!< REV_IN Reverse Input Data bits */
#define CRC_CR_REV_IN_0          (0x1U << CRC_CR_REV_IN_Pos)                   /*!< 0x00000020 */
#define CRC_CR_REV_IN_1          (0x2U << CRC_CR_REV_IN_Pos)                   /*!< 0x00000040 */
#define CRC_CR_REV_OUT_Pos       (7U)                                          
#define CRC_CR_REV_OUT_Msk       (0x1U << CRC_CR_REV_OUT_Pos)                  /*!< 0x00000080 */
#define CRC_CR_REV_OUT           CRC_CR_REV_OUT_Msk                            /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
#define CRC_INIT_INIT_Pos        (0U)                                          
#define CRC_INIT_INIT_Msk        (0xFFFFFFFFU << CRC_INIT_INIT_Pos)            /*!< 0xFFFFFFFF */
#define CRC_INIT_INIT            CRC_INIT_INIT_Msk                             /*!< Initial CRC value bits */

/*******************  Bit definition for CRC_POL register  ********************/
#define CRC_POL_POL_Pos          (0U)                                          
#define CRC_POL_POL_Msk          (0xFFFFFFFFU << CRC_POL_POL_Pos)              /*!< 0xFFFFFFFF */
#define CRC_POL_POL              CRC_POL_POL_Msk                               /*!< Coefficients of the polynomial */

/******************************************************************************/
/*                                                                            */
/*                 Digital to Analog Converter (DAC)                          */
/*                                                                            */
/******************************************************************************/

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
 */
/* Note: No specific macro feature on this device */


/********************  Bit definition for DAC_CR register  ********************/
#define DAC_CR_EN1_Pos              (0U)                                       
#define DAC_CR_EN1_Msk              (0x1U << DAC_CR_EN1_Pos)                   /*!< 0x00000001 */
#define DAC_CR_EN1                  DAC_CR_EN1_Msk                             /*!< DAC channel1 enable */
#define DAC_CR_BOFF1_Pos            (1U)                                       
#define DAC_CR_BOFF1_Msk            (0x1U << DAC_CR_BOFF1_Pos)                 /*!< 0x00000002 */
#define DAC_CR_BOFF1                DAC_CR_BOFF1_Msk                           /*!< DAC channel1 output buffer disable */
#define DAC_CR_TEN1_Pos             (2U)                                       
#define DAC_CR_TEN1_Msk             (0x1U << DAC_CR_TEN1_Pos)                  /*!< 0x00000004 */
#define DAC_CR_TEN1                 DAC_CR_TEN1_Msk                            /*!< DAC channel1 Trigger enable */

#define DAC_CR_TSEL1_Pos            (3U)                                       
#define DAC_CR_TSEL1_Msk            (0x7U << DAC_CR_TSEL1_Pos)                 /*!< 0x00000038 */
#define DAC_CR_TSEL1                DAC_CR_TSEL1_Msk                           /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
#define DAC_CR_TSEL1_0              (0x1U << DAC_CR_TSEL1_Pos)                 /*!< 0x00000008 */
#define DAC_CR_TSEL1_1              (0x2U << DAC_CR_TSEL1_Pos)                 /*!< 0x00000010 */
#define DAC_CR_TSEL1_2              (0x4U << DAC_CR_TSEL1_Pos)                 /*!< 0x00000020 */

#define DAC_CR_WAVE1_Pos            (6U)                                       
#define DAC_CR_WAVE1_Msk            (0x3U << DAC_CR_WAVE1_Pos)                 /*!< 0x000000C0 */
#define DAC_CR_WAVE1                DAC_CR_WAVE1_Msk                           /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define DAC_CR_WAVE1_0              (0x1U << DAC_CR_WAVE1_Pos)                 /*!< 0x00000040 */
#define DAC_CR_WAVE1_1              (0x2U << DAC_CR_WAVE1_Pos)                 /*!< 0x00000080 */

#define DAC_CR_MAMP1_Pos            (8U)                                       
#define DAC_CR_MAMP1_Msk            (0xFU << DAC_CR_MAMP1_Pos)                 /*!< 0x00000F00 */
#define DAC_CR_MAMP1                DAC_CR_MAMP1_Msk                           /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define DAC_CR_MAMP1_0              (0x1U << DAC_CR_MAMP1_Pos)                 /*!< 0x00000100 */
#define DAC_CR_MAMP1_1              (0x2U << DAC_CR_MAMP1_Pos)                 /*!< 0x00000200 */
#define DAC_CR_MAMP1_2              (0x4U << DAC_CR_MAMP1_Pos)                 /*!< 0x00000400 */
#define DAC_CR_MAMP1_3              (0x8U << DAC_CR_MAMP1_Pos)                 /*!< 0x00000800 */

#define DAC_CR_DMAEN1_Pos           (12U)                                      
#define DAC_CR_DMAEN1_Msk           (0x1U << DAC_CR_DMAEN1_Pos)                /*!< 0x00001000 */
#define DAC_CR_DMAEN1               DAC_CR_DMAEN1_Msk                          /*!< DAC channel1 DMA enable */
#define DAC_CR_DMAUDRIE1_Pos        (13U)                                      
#define DAC_CR_DMAUDRIE1_Msk        (0x1U << DAC_CR_DMAUDRIE1_Pos)             /*!< 0x00002000 */
#define DAC_CR_DMAUDRIE1            DAC_CR_DMAUDRIE1_Msk                       /*!< DAC channel1 DMA underrun IT enable */ 
/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define DAC_SWTRIGR_SWTRIG1_Pos     (0U)                                       
#define DAC_SWTRIGR_SWTRIG1_Msk     (0x1U << DAC_SWTRIGR_SWTRIG1_Pos)          /*!< 0x00000001 */
#define DAC_SWTRIGR_SWTRIG1         DAC_SWTRIGR_SWTRIG1_Msk                    /*!< DAC channel1 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define DAC_DHR12R1_DACC1DHR_Pos    (0U)                                       
#define DAC_DHR12R1_DACC1DHR_Msk    (0xFFFU << DAC_DHR12R1_DACC1DHR_Pos)       /*!< 0x00000FFF */
#define DAC_DHR12R1_DACC1DHR        DAC_DHR12R1_DACC1DHR_Msk                   /*!< DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define DAC_DHR12L1_DACC1DHR_Pos    (4U)                                       
#define DAC_DHR12L1_DACC1DHR_Msk    (0xFFFU << DAC_DHR12L1_DACC1DHR_Pos)       /*!< 0x0000FFF0 */
#define DAC_DHR12L1_DACC1DHR        DAC_DHR12L1_DACC1DHR_Msk                   /*!< DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define DAC_DHR8R1_DACC1DHR_Pos     (0U)                                       
#define DAC_DHR8R1_DACC1DHR_Msk     (0xFFU << DAC_DHR8R1_DACC1DHR_Pos)         /*!< 0x000000FF */
#define DAC_DHR8R1_DACC1DHR         DAC_DHR8R1_DACC1DHR_Msk                    /*!< DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define DAC_DHR12RD_DACC1DHR_Pos    (0U)                                       
#define DAC_DHR12RD_DACC1DHR_Msk    (0xFFFU << DAC_DHR12RD_DACC1DHR_Pos)       /*!< 0x00000FFF */
#define DAC_DHR12RD_DACC1DHR        DAC_DHR12RD_DACC1DHR_Msk                   /*!< DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define DAC_DHR12LD_DACC1DHR_Pos    (4U)                                       
#define DAC_DHR12LD_DACC1DHR_Msk    (0xFFFU << DAC_DHR12LD_DACC1DHR_Pos)       /*!< 0x0000FFF0 */
#define DAC_DHR12LD_DACC1DHR        DAC_DHR12LD_DACC1DHR_Msk                   /*!< DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define DAC_DHR8RD_DACC1DHR_Pos     (0U)                                       
#define DAC_DHR8RD_DACC1DHR_Msk     (0xFFU << DAC_DHR8RD_DACC1DHR_Pos)         /*!< 0x000000FF */
#define DAC_DHR8RD_DACC1DHR         DAC_DHR8RD_DACC1DHR_Msk                    /*!< DAC channel1 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define DAC_DOR1_DACC1DOR_Pos       (0U)                                       
#define DAC_DOR1_DACC1DOR_Msk       (0xFFFU << DAC_DOR1_DACC1DOR_Pos)          /*!< 0x00000FFF */
#define DAC_DOR1_DACC1DOR           DAC_DOR1_DACC1DOR_Msk                      /*!< DAC channel1 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define DAC_SR_DMAUDR1_Pos          (13U)                                      
#define DAC_SR_DMAUDR1_Msk          (0x1U << DAC_SR_DMAUDR1_Pos)               /*!< 0x00002000 */
#define DAC_SR_DMAUDR1              DAC_SR_DMAUDR1_Msk                         /*!< DAC channel1 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU (DBGMCU)                         */
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
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos             (4U)                      
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk             (0x1U << DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos) /*!< 0x00000010 */
#define DBGMCU_APB1_FZ_DBG_TIM6_STOP                 DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk 
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
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos    (30U)                     
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk    (0x1U << DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Pos) /*!< 0x40000000 */
#define DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT        DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT_Msk 

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos             (0U)                      
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk             (0x1U << DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) /*!< 0x00000001 */
#define DBGMCU_APB2_FZ_DBG_TIM1_STOP                 DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk 
#define DBGMCU_APB2_FZ_DBG_TIM15_STOP_Pos            (2U)                      
#define DBGMCU_APB2_FZ_DBG_TIM15_STOP_Msk            (0x1U << DBGMCU_APB2_FZ_DBG_TIM15_STOP_Pos) /*!< 0x00000004 */
#define DBGMCU_APB2_FZ_DBG_TIM15_STOP                DBGMCU_APB2_FZ_DBG_TIM15_STOP_Msk 
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos            (3U)                      
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk            (0x1U << DBGMCU_APB2_FZ_DBG_TIM16_STOP_Pos) /*!< 0x00000008 */
#define DBGMCU_APB2_FZ_DBG_TIM16_STOP                DBGMCU_APB2_FZ_DBG_TIM16_STOP_Msk 
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos            (4U)                      
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk            (0x1U << DBGMCU_APB2_FZ_DBG_TIM17_STOP_Pos) /*!< 0x00000010 */
#define DBGMCU_APB2_FZ_DBG_TIM17_STOP                DBGMCU_APB2_FZ_DBG_TIM17_STOP_Msk 

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller (DMA)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for DMA_ISR register  ********************/
#define DMA_ISR_GIF1_Pos       (0U)                                            
#define DMA_ISR_GIF1_Msk       (0x1U << DMA_ISR_GIF1_Pos)                      /*!< 0x00000001 */
#define DMA_ISR_GIF1           DMA_ISR_GIF1_Msk                                /*!< Channel 1 Global interrupt flag */
#define DMA_ISR_TCIF1_Pos      (1U)                                            
#define DMA_ISR_TCIF1_Msk      (0x1U << DMA_ISR_TCIF1_Pos)                     /*!< 0x00000002 */
#define DMA_ISR_TCIF1          DMA_ISR_TCIF1_Msk                               /*!< Channel 1 Transfer Complete flag */
#define DMA_ISR_HTIF1_Pos      (2U)                                            
#define DMA_ISR_HTIF1_Msk      (0x1U << DMA_ISR_HTIF1_Pos)                     /*!< 0x00000004 */
#define DMA_ISR_HTIF1          DMA_ISR_HTIF1_Msk                               /*!< Channel 1 Half Transfer flag */
#define DMA_ISR_TEIF1_Pos      (3U)                                            
#define DMA_ISR_TEIF1_Msk      (0x1U << DMA_ISR_TEIF1_Pos)                     /*!< 0x00000008 */
#define DMA_ISR_TEIF1          DMA_ISR_TEIF1_Msk                               /*!< Channel 1 Transfer Error flag */
#define DMA_ISR_GIF2_Pos       (4U)                                            
#define DMA_ISR_GIF2_Msk       (0x1U << DMA_ISR_GIF2_Pos)                      /*!< 0x00000010 */
#define DMA_ISR_GIF2           DMA_ISR_GIF2_Msk                                /*!< Channel 2 Global interrupt flag */
#define DMA_ISR_TCIF2_Pos      (5U)                                            
#define DMA_ISR_TCIF2_Msk      (0x1U << DMA_ISR_TCIF2_Pos)                     /*!< 0x00000020 */
#define DMA_ISR_TCIF2          DMA_ISR_TCIF2_Msk                               /*!< Channel 2 Transfer Complete flag */
#define DMA_ISR_HTIF2_Pos      (6U)                                            
#define DMA_ISR_HTIF2_Msk      (0x1U << DMA_ISR_HTIF2_Pos)                     /*!< 0x00000040 */
#define DMA_ISR_HTIF2          DMA_ISR_HTIF2_Msk                               /*!< Channel 2 Half Transfer flag */
#define DMA_ISR_TEIF2_Pos      (7U)                                            
#define DMA_ISR_TEIF2_Msk      (0x1U << DMA_ISR_TEIF2_Pos)                     /*!< 0x00000080 */
#define DMA_ISR_TEIF2          DMA_ISR_TEIF2_Msk                               /*!< Channel 2 Transfer Error flag */
#define DMA_ISR_GIF3_Pos       (8U)                                            
#define DMA_ISR_GIF3_Msk       (0x1U << DMA_ISR_GIF3_Pos)                      /*!< 0x00000100 */
#define DMA_ISR_GIF3           DMA_ISR_GIF3_Msk                                /*!< Channel 3 Global interrupt flag */
#define DMA_ISR_TCIF3_Pos      (9U)                                            
#define DMA_ISR_TCIF3_Msk      (0x1U << DMA_ISR_TCIF3_Pos)                     /*!< 0x00000200 */
#define DMA_ISR_TCIF3          DMA_ISR_TCIF3_Msk                               /*!< Channel 3 Transfer Complete flag */
#define DMA_ISR_HTIF3_Pos      (10U)                                           
#define DMA_ISR_HTIF3_Msk      (0x1U << DMA_ISR_HTIF3_Pos)                     /*!< 0x00000400 */
#define DMA_ISR_HTIF3          DMA_ISR_HTIF3_Msk                               /*!< Channel 3 Half Transfer flag */
#define DMA_ISR_TEIF3_Pos      (11U)                                           
#define DMA_ISR_TEIF3_Msk      (0x1U << DMA_ISR_TEIF3_Pos)                     /*!< 0x00000800 */
#define DMA_ISR_TEIF3          DMA_ISR_TEIF3_Msk                               /*!< Channel 3 Transfer Error flag */
#define DMA_ISR_GIF4_Pos       (12U)                                           
#define DMA_ISR_GIF4_Msk       (0x1U << DMA_ISR_GIF4_Pos)                      /*!< 0x00001000 */
#define DMA_ISR_GIF4           DMA_ISR_GIF4_Msk                                /*!< Channel 4 Global interrupt flag */
#define DMA_ISR_TCIF4_Pos      (13U)                                           
#define DMA_ISR_TCIF4_Msk      (0x1U << DMA_ISR_TCIF4_Pos)                     /*!< 0x00002000 */
#define DMA_ISR_TCIF4          DMA_ISR_TCIF4_Msk                               /*!< Channel 4 Transfer Complete flag */
#define DMA_ISR_HTIF4_Pos      (14U)                                           
#define DMA_ISR_HTIF4_Msk      (0x1U << DMA_ISR_HTIF4_Pos)                     /*!< 0x00004000 */
#define DMA_ISR_HTIF4          DMA_ISR_HTIF4_Msk                               /*!< Channel 4 Half Transfer flag */
#define DMA_ISR_TEIF4_Pos      (15U)                                           
#define DMA_ISR_TEIF4_Msk      (0x1U << DMA_ISR_TEIF4_Pos)                     /*!< 0x00008000 */
#define DMA_ISR_TEIF4          DMA_ISR_TEIF4_Msk                               /*!< Channel 4 Transfer Error flag */
#define DMA_ISR_GIF5_Pos       (16U)                                           
#define DMA_ISR_GIF5_Msk       (0x1U << DMA_ISR_GIF5_Pos)                      /*!< 0x00010000 */
#define DMA_ISR_GIF5           DMA_ISR_GIF5_Msk                                /*!< Channel 5 Global interrupt flag */
#define DMA_ISR_TCIF5_Pos      (17U)                                           
#define DMA_ISR_TCIF5_Msk      (0x1U << DMA_ISR_TCIF5_Pos)                     /*!< 0x00020000 */
#define DMA_ISR_TCIF5          DMA_ISR_TCIF5_Msk                               /*!< Channel 5 Transfer Complete flag */
#define DMA_ISR_HTIF5_Pos      (18U)                                           
#define DMA_ISR_HTIF5_Msk      (0x1U << DMA_ISR_HTIF5_Pos)                     /*!< 0x00040000 */
#define DMA_ISR_HTIF5          DMA_ISR_HTIF5_Msk                               /*!< Channel 5 Half Transfer flag */
#define DMA_ISR_TEIF5_Pos      (19U)                                           
#define DMA_ISR_TEIF5_Msk      (0x1U << DMA_ISR_TEIF5_Pos)                     /*!< 0x00080000 */
#define DMA_ISR_TEIF5          DMA_ISR_TEIF5_Msk                               /*!< Channel 5 Transfer Error flag */
#define DMA_ISR_GIF6_Pos       (20U)                                           
#define DMA_ISR_GIF6_Msk       (0x1U << DMA_ISR_GIF6_Pos)                      /*!< 0x00100000 */
#define DMA_ISR_GIF6           DMA_ISR_GIF6_Msk                                /*!< Channel 6 Global interrupt flag */
#define DMA_ISR_TCIF6_Pos      (21U)                                           
#define DMA_ISR_TCIF6_Msk      (0x1U << DMA_ISR_TCIF6_Pos)                     /*!< 0x00200000 */
#define DMA_ISR_TCIF6          DMA_ISR_TCIF6_Msk                               /*!< Channel 6 Transfer Complete flag */
#define DMA_ISR_HTIF6_Pos      (22U)                                           
#define DMA_ISR_HTIF6_Msk      (0x1U << DMA_ISR_HTIF6_Pos)                     /*!< 0x00400000 */
#define DMA_ISR_HTIF6          DMA_ISR_HTIF6_Msk                               /*!< Channel 6 Half Transfer flag */
#define DMA_ISR_TEIF6_Pos      (23U)                                           
#define DMA_ISR_TEIF6_Msk      (0x1U << DMA_ISR_TEIF6_Pos)                     /*!< 0x00800000 */
#define DMA_ISR_TEIF6          DMA_ISR_TEIF6_Msk                               /*!< Channel 6 Transfer Error flag */
#define DMA_ISR_GIF7_Pos       (24U)                                           
#define DMA_ISR_GIF7_Msk       (0x1U << DMA_ISR_GIF7_Pos)                      /*!< 0x01000000 */
#define DMA_ISR_GIF7           DMA_ISR_GIF7_Msk                                /*!< Channel 7 Global interrupt flag */
#define DMA_ISR_TCIF7_Pos      (25U)                                           
#define DMA_ISR_TCIF7_Msk      (0x1U << DMA_ISR_TCIF7_Pos)                     /*!< 0x02000000 */
#define DMA_ISR_TCIF7          DMA_ISR_TCIF7_Msk                               /*!< Channel 7 Transfer Complete flag */
#define DMA_ISR_HTIF7_Pos      (26U)                                           
#define DMA_ISR_HTIF7_Msk      (0x1U << DMA_ISR_HTIF7_Pos)                     /*!< 0x04000000 */
#define DMA_ISR_HTIF7          DMA_ISR_HTIF7_Msk                               /*!< Channel 7 Half Transfer flag */
#define DMA_ISR_TEIF7_Pos      (27U)                                           
#define DMA_ISR_TEIF7_Msk      (0x1U << DMA_ISR_TEIF7_Pos)                     /*!< 0x08000000 */
#define DMA_ISR_TEIF7          DMA_ISR_TEIF7_Msk                               /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define DMA_IFCR_CGIF1_Pos     (0U)                                            
#define DMA_IFCR_CGIF1_Msk     (0x1U << DMA_IFCR_CGIF1_Pos)                    /*!< 0x00000001 */
#define DMA_IFCR_CGIF1         DMA_IFCR_CGIF1_Msk                              /*!< Channel 1 Global interrupt clear */
#define DMA_IFCR_CTCIF1_Pos    (1U)                                            
#define DMA_IFCR_CTCIF1_Msk    (0x1U << DMA_IFCR_CTCIF1_Pos)                   /*!< 0x00000002 */
#define DMA_IFCR_CTCIF1        DMA_IFCR_CTCIF1_Msk                             /*!< Channel 1 Transfer Complete clear */
#define DMA_IFCR_CHTIF1_Pos    (2U)                                            
#define DMA_IFCR_CHTIF1_Msk    (0x1U << DMA_IFCR_CHTIF1_Pos)                   /*!< 0x00000004 */
#define DMA_IFCR_CHTIF1        DMA_IFCR_CHTIF1_Msk                             /*!< Channel 1 Half Transfer clear */
#define DMA_IFCR_CTEIF1_Pos    (3U)                                            
#define DMA_IFCR_CTEIF1_Msk    (0x1U << DMA_IFCR_CTEIF1_Pos)                   /*!< 0x00000008 */
#define DMA_IFCR_CTEIF1        DMA_IFCR_CTEIF1_Msk                             /*!< Channel 1 Transfer Error clear */
#define DMA_IFCR_CGIF2_Pos     (4U)                                            
#define DMA_IFCR_CGIF2_Msk     (0x1U << DMA_IFCR_CGIF2_Pos)                    /*!< 0x00000010 */
#define DMA_IFCR_CGIF2         DMA_IFCR_CGIF2_Msk                              /*!< Channel 2 Global interrupt clear */
#define DMA_IFCR_CTCIF2_Pos    (5U)                                            
#define DMA_IFCR_CTCIF2_Msk    (0x1U << DMA_IFCR_CTCIF2_Pos)                   /*!< 0x00000020 */
#define DMA_IFCR_CTCIF2        DMA_IFCR_CTCIF2_Msk                             /*!< Channel 2 Transfer Complete clear */
#define DMA_IFCR_CHTIF2_Pos    (6U)                                            
#define DMA_IFCR_CHTIF2_Msk    (0x1U << DMA_IFCR_CHTIF2_Pos)                   /*!< 0x00000040 */
#define DMA_IFCR_CHTIF2        DMA_IFCR_CHTIF2_Msk                             /*!< Channel 2 Half Transfer clear */
#define DMA_IFCR_CTEIF2_Pos    (7U)                                            
#define DMA_IFCR_CTEIF2_Msk    (0x1U << DMA_IFCR_CTEIF2_Pos)                   /*!< 0x00000080 */
#define DMA_IFCR_CTEIF2        DMA_IFCR_CTEIF2_Msk                             /*!< Channel 2 Transfer Error clear */
#define DMA_IFCR_CGIF3_Pos     (8U)                                            
#define DMA_IFCR_CGIF3_Msk     (0x1U << DMA_IFCR_CGIF3_Pos)                    /*!< 0x00000100 */
#define DMA_IFCR_CGIF3         DMA_IFCR_CGIF3_Msk                              /*!< Channel 3 Global interrupt clear */
#define DMA_IFCR_CTCIF3_Pos    (9U)                                            
#define DMA_IFCR_CTCIF3_Msk    (0x1U << DMA_IFCR_CTCIF3_Pos)                   /*!< 0x00000200 */
#define DMA_IFCR_CTCIF3        DMA_IFCR_CTCIF3_Msk                             /*!< Channel 3 Transfer Complete clear */
#define DMA_IFCR_CHTIF3_Pos    (10U)                                           
#define DMA_IFCR_CHTIF3_Msk    (0x1U << DMA_IFCR_CHTIF3_Pos)                   /*!< 0x00000400 */
#define DMA_IFCR_CHTIF3        DMA_IFCR_CHTIF3_Msk                             /*!< Channel 3 Half Transfer clear */
#define DMA_IFCR_CTEIF3_Pos    (11U)                                           
#define DMA_IFCR_CTEIF3_Msk    (0x1U << DMA_IFCR_CTEIF3_Pos)                   /*!< 0x00000800 */
#define DMA_IFCR_CTEIF3        DMA_IFCR_CTEIF3_Msk                             /*!< Channel 3 Transfer Error clear */
#define DMA_IFCR_CGIF4_Pos     (12U)                                           
#define DMA_IFCR_CGIF4_Msk     (0x1U << DMA_IFCR_CGIF4_Pos)                    /*!< 0x00001000 */
#define DMA_IFCR_CGIF4         DMA_IFCR_CGIF4_Msk                              /*!< Channel 4 Global interrupt clear */
#define DMA_IFCR_CTCIF4_Pos    (13U)                                           
#define DMA_IFCR_CTCIF4_Msk    (0x1U << DMA_IFCR_CTCIF4_Pos)                   /*!< 0x00002000 */
#define DMA_IFCR_CTCIF4        DMA_IFCR_CTCIF4_Msk                             /*!< Channel 4 Transfer Complete clear */
#define DMA_IFCR_CHTIF4_Pos    (14U)                                           
#define DMA_IFCR_CHTIF4_Msk    (0x1U << DMA_IFCR_CHTIF4_Pos)                   /*!< 0x00004000 */
#define DMA_IFCR_CHTIF4        DMA_IFCR_CHTIF4_Msk                             /*!< Channel 4 Half Transfer clear */
#define DMA_IFCR_CTEIF4_Pos    (15U)                                           
#define DMA_IFCR_CTEIF4_Msk    (0x1U << DMA_IFCR_CTEIF4_Pos)                   /*!< 0x00008000 */
#define DMA_IFCR_CTEIF4        DMA_IFCR_CTEIF4_Msk                             /*!< Channel 4 Transfer Error clear */
#define DMA_IFCR_CGIF5_Pos     (16U)                                           
#define DMA_IFCR_CGIF5_Msk     (0x1U << DMA_IFCR_CGIF5_Pos)                    /*!< 0x00010000 */
#define DMA_IFCR_CGIF5         DMA_IFCR_CGIF5_Msk                              /*!< Channel 5 Global interrupt clear */
#define DMA_IFCR_CTCIF5_Pos    (17U)                                           
#define DMA_IFCR_CTCIF5_Msk    (0x1U << DMA_IFCR_CTCIF5_Pos)                   /*!< 0x00020000 */
#define DMA_IFCR_CTCIF5        DMA_IFCR_CTCIF5_Msk                             /*!< Channel 5 Transfer Complete clear */
#define DMA_IFCR_CHTIF5_Pos    (18U)                                           
#define DMA_IFCR_CHTIF5_Msk    (0x1U << DMA_IFCR_CHTIF5_Pos)                   /*!< 0x00040000 */
#define DMA_IFCR_CHTIF5        DMA_IFCR_CHTIF5_Msk                             /*!< Channel 5 Half Transfer clear */
#define DMA_IFCR_CTEIF5_Pos    (19U)                                           
#define DMA_IFCR_CTEIF5_Msk    (0x1U << DMA_IFCR_CTEIF5_Pos)                   /*!< 0x00080000 */
#define DMA_IFCR_CTEIF5        DMA_IFCR_CTEIF5_Msk                             /*!< Channel 5 Transfer Error clear */
#define DMA_IFCR_CGIF6_Pos     (20U)                                           
#define DMA_IFCR_CGIF6_Msk     (0x1U << DMA_IFCR_CGIF6_Pos)                    /*!< 0x00100000 */
#define DMA_IFCR_CGIF6         DMA_IFCR_CGIF6_Msk                              /*!< Channel 6 Global interrupt clear */
#define DMA_IFCR_CTCIF6_Pos    (21U)                                           
#define DMA_IFCR_CTCIF6_Msk    (0x1U << DMA_IFCR_CTCIF6_Pos)                   /*!< 0x00200000 */
#define DMA_IFCR_CTCIF6        DMA_IFCR_CTCIF6_Msk                             /*!< Channel 6 Transfer Complete clear */
#define DMA_IFCR_CHTIF6_Pos    (22U)                                           
#define DMA_IFCR_CHTIF6_Msk    (0x1U << DMA_IFCR_CHTIF6_Pos)                   /*!< 0x00400000 */
#define DMA_IFCR_CHTIF6        DMA_IFCR_CHTIF6_Msk                             /*!< Channel 6 Half Transfer clear */
#define DMA_IFCR_CTEIF6_Pos    (23U)                                           
#define DMA_IFCR_CTEIF6_Msk    (0x1U << DMA_IFCR_CTEIF6_Pos)                   /*!< 0x00800000 */
#define DMA_IFCR_CTEIF6        DMA_IFCR_CTEIF6_Msk                             /*!< Channel 6 Transfer Error clear */
#define DMA_IFCR_CGIF7_Pos     (24U)                                           
#define DMA_IFCR_CGIF7_Msk     (0x1U << DMA_IFCR_CGIF7_Pos)                    /*!< 0x01000000 */
#define DMA_IFCR_CGIF7         DMA_IFCR_CGIF7_Msk                              /*!< Channel 7 Global interrupt clear */
#define DMA_IFCR_CTCIF7_Pos    (25U)                                           
#define DMA_IFCR_CTCIF7_Msk    (0x1U << DMA_IFCR_CTCIF7_Pos)                   /*!< 0x02000000 */
#define DMA_IFCR_CTCIF7        DMA_IFCR_CTCIF7_Msk                             /*!< Channel 7 Transfer Complete clear */
#define DMA_IFCR_CHTIF7_Pos    (26U)                                           
#define DMA_IFCR_CHTIF7_Msk    (0x1U << DMA_IFCR_CHTIF7_Pos)                   /*!< 0x04000000 */
#define DMA_IFCR_CHTIF7        DMA_IFCR_CHTIF7_Msk                             /*!< Channel 7 Half Transfer clear */
#define DMA_IFCR_CTEIF7_Pos    (27U)                                           
#define DMA_IFCR_CTEIF7_Msk    (0x1U << DMA_IFCR_CTEIF7_Pos)                   /*!< 0x08000000 */
#define DMA_IFCR_CTEIF7        DMA_IFCR_CTEIF7_Msk                             /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR register  ********************/
#define DMA_CCR_EN_Pos         (0U)                                            
#define DMA_CCR_EN_Msk         (0x1U << DMA_CCR_EN_Pos)                        /*!< 0x00000001 */
#define DMA_CCR_EN             DMA_CCR_EN_Msk                                  /*!< Channel enable                      */
#define DMA_CCR_TCIE_Pos       (1U)                                            
#define DMA_CCR_TCIE_Msk       (0x1U << DMA_CCR_TCIE_Pos)                      /*!< 0x00000002 */
#define DMA_CCR_TCIE           DMA_CCR_TCIE_Msk                                /*!< Transfer complete interrupt enable  */
#define DMA_CCR_HTIE_Pos       (2U)                                            
#define DMA_CCR_HTIE_Msk       (0x1U << DMA_CCR_HTIE_Pos)                      /*!< 0x00000004 */
#define DMA_CCR_HTIE           DMA_CCR_HTIE_Msk                                /*!< Half Transfer interrupt enable      */
#define DMA_CCR_TEIE_Pos       (3U)                                            
#define DMA_CCR_TEIE_Msk       (0x1U << DMA_CCR_TEIE_Pos)                      /*!< 0x00000008 */
#define DMA_CCR_TEIE           DMA_CCR_TEIE_Msk                                /*!< Transfer error interrupt enable     */
#define DMA_CCR_DIR_Pos        (4U)                                            
#define DMA_CCR_DIR_Msk        (0x1U << DMA_CCR_DIR_Pos)                       /*!< 0x00000010 */
#define DMA_CCR_DIR            DMA_CCR_DIR_Msk                                 /*!< Data transfer direction             */
#define DMA_CCR_CIRC_Pos       (5U)                                            
#define DMA_CCR_CIRC_Msk       (0x1U << DMA_CCR_CIRC_Pos)                      /*!< 0x00000020 */
#define DMA_CCR_CIRC           DMA_CCR_CIRC_Msk                                /*!< Circular mode                       */
#define DMA_CCR_PINC_Pos       (6U)                                            
#define DMA_CCR_PINC_Msk       (0x1U << DMA_CCR_PINC_Pos)                      /*!< 0x00000040 */
#define DMA_CCR_PINC           DMA_CCR_PINC_Msk                                /*!< Peripheral increment mode           */
#define DMA_CCR_MINC_Pos       (7U)                                            
#define DMA_CCR_MINC_Msk       (0x1U << DMA_CCR_MINC_Pos)                      /*!< 0x00000080 */
#define DMA_CCR_MINC           DMA_CCR_MINC_Msk                                /*!< Memory increment mode               */

#define DMA_CCR_PSIZE_Pos      (8U)                                            
#define DMA_CCR_PSIZE_Msk      (0x3U << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000300 */
#define DMA_CCR_PSIZE          DMA_CCR_PSIZE_Msk                               /*!< PSIZE[1:0] bits (Peripheral size)   */
#define DMA_CCR_PSIZE_0        (0x1U << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000100 */
#define DMA_CCR_PSIZE_1        (0x2U << DMA_CCR_PSIZE_Pos)                     /*!< 0x00000200 */

#define DMA_CCR_MSIZE_Pos      (10U)                                           
#define DMA_CCR_MSIZE_Msk      (0x3U << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000C00 */
#define DMA_CCR_MSIZE          DMA_CCR_MSIZE_Msk                               /*!< MSIZE[1:0] bits (Memory size)       */
#define DMA_CCR_MSIZE_0        (0x1U << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000400 */
#define DMA_CCR_MSIZE_1        (0x2U << DMA_CCR_MSIZE_Pos)                     /*!< 0x00000800 */

#define DMA_CCR_PL_Pos         (12U)                                           
#define DMA_CCR_PL_Msk         (0x3U << DMA_CCR_PL_Pos)                        /*!< 0x00003000 */
#define DMA_CCR_PL             DMA_CCR_PL_Msk                                  /*!< PL[1:0] bits(Channel Priority level)*/
#define DMA_CCR_PL_0           (0x1U << DMA_CCR_PL_Pos)                        /*!< 0x00001000 */
#define DMA_CCR_PL_1           (0x2U << DMA_CCR_PL_Pos)                        /*!< 0x00002000 */

#define DMA_CCR_MEM2MEM_Pos    (14U)                                           
#define DMA_CCR_MEM2MEM_Msk    (0x1U << DMA_CCR_MEM2MEM_Pos)                   /*!< 0x00004000 */
#define DMA_CCR_MEM2MEM        DMA_CCR_MEM2MEM_Msk                             /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
#define DMA_CNDTR_NDT_Pos      (0U)                                            
#define DMA_CNDTR_NDT_Msk      (0xFFFFU << DMA_CNDTR_NDT_Pos)                  /*!< 0x0000FFFF */
#define DMA_CNDTR_NDT          DMA_CNDTR_NDT_Msk                               /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
#define DMA_CPAR_PA_Pos        (0U)                                            
#define DMA_CPAR_PA_Msk        (0xFFFFFFFFU << DMA_CPAR_PA_Pos)                /*!< 0xFFFFFFFF */
#define DMA_CPAR_PA            DMA_CPAR_PA_Msk                                 /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
#define DMA_CMAR_MA_Pos        (0U)                                            
#define DMA_CMAR_MA_Msk        (0xFFFFFFFFU << DMA_CMAR_MA_Pos)                /*!< 0xFFFFFFFF */
#define DMA_CMAR_MA            DMA_CMAR_MA_Msk                                 /*!< Memory Address                      */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller (EXTI)              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos           (0U)                                        
#define EXTI_IMR_MR0_Msk           (0x1U << EXTI_IMR_MR0_Pos)                  /*!< 0x00000001 */
#define EXTI_IMR_MR0               EXTI_IMR_MR0_Msk                            /*!< Interrupt Mask on line 0 */
#define EXTI_IMR_MR1_Pos           (1U)                                        
#define EXTI_IMR_MR1_Msk           (0x1U << EXTI_IMR_MR1_Pos)                  /*!< 0x00000002 */
#define EXTI_IMR_MR1               EXTI_IMR_MR1_Msk                            /*!< Interrupt Mask on line 1 */
#define EXTI_IMR_MR2_Pos           (2U)                                        
#define EXTI_IMR_MR2_Msk           (0x1U << EXTI_IMR_MR2_Pos)                  /*!< 0x00000004 */
#define EXTI_IMR_MR2               EXTI_IMR_MR2_Msk                            /*!< Interrupt Mask on line 2 */
#define EXTI_IMR_MR3_Pos           (3U)                                        
#define EXTI_IMR_MR3_Msk           (0x1U << EXTI_IMR_MR3_Pos)                  /*!< 0x00000008 */
#define EXTI_IMR_MR3               EXTI_IMR_MR3_Msk                            /*!< Interrupt Mask on line 3 */
#define EXTI_IMR_MR4_Pos           (4U)                                        
#define EXTI_IMR_MR4_Msk           (0x1U << EXTI_IMR_MR4_Pos)                  /*!< 0x00000010 */
#define EXTI_IMR_MR4               EXTI_IMR_MR4_Msk                            /*!< Interrupt Mask on line 4 */
#define EXTI_IMR_MR5_Pos           (5U)                                        
#define EXTI_IMR_MR5_Msk           (0x1U << EXTI_IMR_MR5_Pos)                  /*!< 0x00000020 */
#define EXTI_IMR_MR5               EXTI_IMR_MR5_Msk                            /*!< Interrupt Mask on line 5 */
#define EXTI_IMR_MR6_Pos           (6U)                                        
#define EXTI_IMR_MR6_Msk           (0x1U << EXTI_IMR_MR6_Pos)                  /*!< 0x00000040 */
#define EXTI_IMR_MR6               EXTI_IMR_MR6_Msk                            /*!< Interrupt Mask on line 6 */
#define EXTI_IMR_MR7_Pos           (7U)                                        
#define EXTI_IMR_MR7_Msk           (0x1U << EXTI_IMR_MR7_Pos)                  /*!< 0x00000080 */
#define EXTI_IMR_MR7               EXTI_IMR_MR7_Msk                            /*!< Interrupt Mask on line 7 */
#define EXTI_IMR_MR8_Pos           (8U)                                        
#define EXTI_IMR_MR8_Msk           (0x1U << EXTI_IMR_MR8_Pos)                  /*!< 0x00000100 */
#define EXTI_IMR_MR8               EXTI_IMR_MR8_Msk                            /*!< Interrupt Mask on line 8 */
#define EXTI_IMR_MR9_Pos           (9U)                                        
#define EXTI_IMR_MR9_Msk           (0x1U << EXTI_IMR_MR9_Pos)                  /*!< 0x00000200 */
#define EXTI_IMR_MR9               EXTI_IMR_MR9_Msk                            /*!< Interrupt Mask on line 9 */
#define EXTI_IMR_MR10_Pos          (10U)                                       
#define EXTI_IMR_MR10_Msk          (0x1U << EXTI_IMR_MR10_Pos)                 /*!< 0x00000400 */
#define EXTI_IMR_MR10              EXTI_IMR_MR10_Msk                           /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos          (11U)                                       
#define EXTI_IMR_MR11_Msk          (0x1U << EXTI_IMR_MR11_Pos)                 /*!< 0x00000800 */
#define EXTI_IMR_MR11              EXTI_IMR_MR11_Msk                           /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos          (12U)                                       
#define EXTI_IMR_MR12_Msk          (0x1U << EXTI_IMR_MR12_Pos)                 /*!< 0x00001000 */
#define EXTI_IMR_MR12              EXTI_IMR_MR12_Msk                           /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos          (13U)                                       
#define EXTI_IMR_MR13_Msk          (0x1U << EXTI_IMR_MR13_Pos)                 /*!< 0x00002000 */
#define EXTI_IMR_MR13              EXTI_IMR_MR13_Msk                           /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos          (14U)                                       
#define EXTI_IMR_MR14_Msk          (0x1U << EXTI_IMR_MR14_Pos)                 /*!< 0x00004000 */
#define EXTI_IMR_MR14              EXTI_IMR_MR14_Msk                           /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos          (15U)                                       
#define EXTI_IMR_MR15_Msk          (0x1U << EXTI_IMR_MR15_Pos)                 /*!< 0x00008000 */
#define EXTI_IMR_MR15              EXTI_IMR_MR15_Msk                           /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR16_Pos          (16U)                                       
#define EXTI_IMR_MR16_Msk          (0x1U << EXTI_IMR_MR16_Pos)                 /*!< 0x00010000 */
#define EXTI_IMR_MR16              EXTI_IMR_MR16_Msk                           /*!< Interrupt Mask on line 16 */
#define EXTI_IMR_MR17_Pos          (17U)                                       
#define EXTI_IMR_MR17_Msk          (0x1U << EXTI_IMR_MR17_Pos)                 /*!< 0x00020000 */
#define EXTI_IMR_MR17              EXTI_IMR_MR17_Msk                           /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR19_Pos          (19U)                                       
#define EXTI_IMR_MR19_Msk          (0x1U << EXTI_IMR_MR19_Pos)                 /*!< 0x00080000 */
#define EXTI_IMR_MR19              EXTI_IMR_MR19_Msk                           /*!< Interrupt Mask on line 19 */
#define EXTI_IMR_MR20_Pos          (20U)                                       
#define EXTI_IMR_MR20_Msk          (0x1U << EXTI_IMR_MR20_Pos)                 /*!< 0x00100000 */
#define EXTI_IMR_MR20              EXTI_IMR_MR20_Msk                           /*!< Interrupt Mask on line 20 */
#define EXTI_IMR_MR22_Pos          (22U)                                       
#define EXTI_IMR_MR22_Msk          (0x1U << EXTI_IMR_MR22_Pos)                 /*!< 0x00400000 */
#define EXTI_IMR_MR22              EXTI_IMR_MR22_Msk                           /*!< Interrupt Mask on line 22 */
#define EXTI_IMR_MR23_Pos          (23U)                                       
#define EXTI_IMR_MR23_Msk          (0x1U << EXTI_IMR_MR23_Pos)                 /*!< 0x00800000 */
#define EXTI_IMR_MR23              EXTI_IMR_MR23_Msk                           /*!< Interrupt Mask on line 23 */
#define EXTI_IMR_MR24_Pos          (24U)                                       
#define EXTI_IMR_MR24_Msk          (0x1U << EXTI_IMR_MR24_Pos)                 /*!< 0x01000000 */
#define EXTI_IMR_MR24              EXTI_IMR_MR24_Msk                           /*!< Interrupt Mask on line 24 */
#define EXTI_IMR_MR25_Pos          (25U)                                       
#define EXTI_IMR_MR25_Msk          (0x1U << EXTI_IMR_MR25_Pos)                 /*!< 0x02000000 */
#define EXTI_IMR_MR25              EXTI_IMR_MR25_Msk                           /*!< Interrupt Mask on line 25 */
#define EXTI_IMR_MR27_Pos          (27U)                                       
#define EXTI_IMR_MR27_Msk          (0x1U << EXTI_IMR_MR27_Pos)                 /*!< 0x08000000 */
#define EXTI_IMR_MR27              EXTI_IMR_MR27_Msk                           /*!< Interrupt Mask on line 27 */
#define EXTI_IMR_MR30_Pos          (30U)                                       
#define EXTI_IMR_MR30_Msk          (0x1U << EXTI_IMR_MR30_Pos)                 /*!< 0x40000000 */
#define EXTI_IMR_MR30              EXTI_IMR_MR30_Msk                           /*!< Interrupt Mask on line 30 */

/* References Defines */
#define  EXTI_IMR_IM0 EXTI_IMR_MR0
#define  EXTI_IMR_IM1 EXTI_IMR_MR1
#define  EXTI_IMR_IM2 EXTI_IMR_MR2
#define  EXTI_IMR_IM3 EXTI_IMR_MR3
#define  EXTI_IMR_IM4 EXTI_IMR_MR4
#define  EXTI_IMR_IM5 EXTI_IMR_MR5
#define  EXTI_IMR_IM6 EXTI_IMR_MR6
#define  EXTI_IMR_IM7 EXTI_IMR_MR7
#define  EXTI_IMR_IM8 EXTI_IMR_MR8
#define  EXTI_IMR_IM9 EXTI_IMR_MR9
#define  EXTI_IMR_IM10 EXTI_IMR_MR10
#define  EXTI_IMR_IM11 EXTI_IMR_MR11
#define  EXTI_IMR_IM12 EXTI_IMR_MR12
#define  EXTI_IMR_IM13 EXTI_IMR_MR13
#define  EXTI_IMR_IM14 EXTI_IMR_MR14
#define  EXTI_IMR_IM15 EXTI_IMR_MR15
#define  EXTI_IMR_IM16 EXTI_IMR_MR16
#define  EXTI_IMR_IM17 EXTI_IMR_MR17
#if defined(EXTI_IMR_MR18)
#define  EXTI_IMR_IM18 EXTI_IMR_MR18
#endif
#define  EXTI_IMR_IM19 EXTI_IMR_MR19
#define  EXTI_IMR_IM20 EXTI_IMR_MR20
#if defined(EXTI_IMR_MR21)
#define  EXTI_IMR_IM21 EXTI_IMR_MR21
#endif
#define  EXTI_IMR_IM22 EXTI_IMR_MR22
#define  EXTI_IMR_IM23 EXTI_IMR_MR23
#define  EXTI_IMR_IM24 EXTI_IMR_MR24
#define  EXTI_IMR_IM25 EXTI_IMR_MR25
#if defined(EXTI_IMR_MR26)
#define  EXTI_IMR_IM26 EXTI_IMR_MR26
#endif
#define  EXTI_IMR_IM27 EXTI_IMR_MR27
#if defined(EXTI_IMR_MR28)
#define  EXTI_IMR_IM28 EXTI_IMR_MR28
#endif
#if defined(EXTI_IMR_MR29)
#define  EXTI_IMR_IM29 EXTI_IMR_MR29
#endif
#define  EXTI_IMR_IM30 EXTI_IMR_MR30
#if defined(EXTI_IMR_MR31)
#define  EXTI_IMR_IM31 EXTI_IMR_MR31
#endif

#define EXTI_IMR_IM_Pos            (0U)                                        
#define EXTI_IMR_IM_Msk            (0xFFFFFFFFU << EXTI_IMR_IM_Pos)            /*!< 0xFFFFFFFF */
#define EXTI_IMR_IM                EXTI_IMR_IM_Msk                             /*!< Interrupt Mask All */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define EXTI_EMR_MR0_Pos           (0U)                                        
#define EXTI_EMR_MR0_Msk           (0x1U << EXTI_EMR_MR0_Pos)                  /*!< 0x00000001 */
#define EXTI_EMR_MR0               EXTI_EMR_MR0_Msk                            /*!< Event Mask on line 0 */
#define EXTI_EMR_MR1_Pos           (1U)                                        
#define EXTI_EMR_MR1_Msk           (0x1U << EXTI_EMR_MR1_Pos)                  /*!< 0x00000002 */
#define EXTI_EMR_MR1               EXTI_EMR_MR1_Msk                            /*!< Event Mask on line 1 */
#define EXTI_EMR_MR2_Pos           (2U)                                        
#define EXTI_EMR_MR2_Msk           (0x1U << EXTI_EMR_MR2_Pos)                  /*!< 0x00000004 */
#define EXTI_EMR_MR2               EXTI_EMR_MR2_Msk                            /*!< Event Mask on line 2 */
#define EXTI_EMR_MR3_Pos           (3U)                                        
#define EXTI_EMR_MR3_Msk           (0x1U << EXTI_EMR_MR3_Pos)                  /*!< 0x00000008 */
#define EXTI_EMR_MR3               EXTI_EMR_MR3_Msk                            /*!< Event Mask on line 3 */
#define EXTI_EMR_MR4_Pos           (4U)                                        
#define EXTI_EMR_MR4_Msk           (0x1U << EXTI_EMR_MR4_Pos)                  /*!< 0x00000010 */
#define EXTI_EMR_MR4               EXTI_EMR_MR4_Msk                            /*!< Event Mask on line 4 */
#define EXTI_EMR_MR5_Pos           (5U)                                        
#define EXTI_EMR_MR5_Msk           (0x1U << EXTI_EMR_MR5_Pos)                  /*!< 0x00000020 */
#define EXTI_EMR_MR5               EXTI_EMR_MR5_Msk                            /*!< Event Mask on line 5 */
#define EXTI_EMR_MR6_Pos           (6U)                                        
#define EXTI_EMR_MR6_Msk           (0x1U << EXTI_EMR_MR6_Pos)                  /*!< 0x00000040 */
#define EXTI_EMR_MR6               EXTI_EMR_MR6_Msk                            /*!< Event Mask on line 6 */
#define EXTI_EMR_MR7_Pos           (7U)                                        
#define EXTI_EMR_MR7_Msk           (0x1U << EXTI_EMR_MR7_Pos)                  /*!< 0x00000080 */
#define EXTI_EMR_MR7               EXTI_EMR_MR7_Msk                            /*!< Event Mask on line 7 */
#define EXTI_EMR_MR8_Pos           (8U)                                        
#define EXTI_EMR_MR8_Msk           (0x1U << EXTI_EMR_MR8_Pos)                  /*!< 0x00000100 */
#define EXTI_EMR_MR8               EXTI_EMR_MR8_Msk                            /*!< Event Mask on line 8 */
#define EXTI_EMR_MR9_Pos           (9U)                                        
#define EXTI_EMR_MR9_Msk           (0x1U << EXTI_EMR_MR9_Pos)                  /*!< 0x00000200 */
#define EXTI_EMR_MR9               EXTI_EMR_MR9_Msk                            /*!< Event Mask on line 9 */
#define EXTI_EMR_MR10_Pos          (10U)                                       
#define EXTI_EMR_MR10_Msk          (0x1U << EXTI_EMR_MR10_Pos)                 /*!< 0x00000400 */
#define EXTI_EMR_MR10              EXTI_EMR_MR10_Msk                           /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos          (11U)                                       
#define EXTI_EMR_MR11_Msk          (0x1U << EXTI_EMR_MR11_Pos)                 /*!< 0x00000800 */
#define EXTI_EMR_MR11              EXTI_EMR_MR11_Msk                           /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos          (12U)                                       
#define EXTI_EMR_MR12_Msk          (0x1U << EXTI_EMR_MR12_Pos)                 /*!< 0x00001000 */
#define EXTI_EMR_MR12              EXTI_EMR_MR12_Msk                           /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos          (13U)                                       
#define EXTI_EMR_MR13_Msk          (0x1U << EXTI_EMR_MR13_Pos)                 /*!< 0x00002000 */
#define EXTI_EMR_MR13              EXTI_EMR_MR13_Msk                           /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos          (14U)                                       
#define EXTI_EMR_MR14_Msk          (0x1U << EXTI_EMR_MR14_Pos)                 /*!< 0x00004000 */
#define EXTI_EMR_MR14              EXTI_EMR_MR14_Msk                           /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos          (15U)                                       
#define EXTI_EMR_MR15_Msk          (0x1U << EXTI_EMR_MR15_Pos)                 /*!< 0x00008000 */
#define EXTI_EMR_MR15              EXTI_EMR_MR15_Msk                           /*!< Event Mask on line 15 */
#define EXTI_EMR_MR16_Pos          (16U)                                       
#define EXTI_EMR_MR16_Msk          (0x1U << EXTI_EMR_MR16_Pos)                 /*!< 0x00010000 */
#define EXTI_EMR_MR16              EXTI_EMR_MR16_Msk                           /*!< Event Mask on line 16 */
#define EXTI_EMR_MR17_Pos          (17U)                                       
#define EXTI_EMR_MR17_Msk          (0x1U << EXTI_EMR_MR17_Pos)                 /*!< 0x00020000 */
#define EXTI_EMR_MR17              EXTI_EMR_MR17_Msk                           /*!< Event Mask on line 17 */
#define EXTI_EMR_MR19_Pos          (19U)                                       
#define EXTI_EMR_MR19_Msk          (0x1U << EXTI_EMR_MR19_Pos)                 /*!< 0x00080000 */
#define EXTI_EMR_MR19              EXTI_EMR_MR19_Msk                           /*!< Event Mask on line 19 */
#define EXTI_EMR_MR20_Pos          (20U)                                       
#define EXTI_EMR_MR20_Msk          (0x1U << EXTI_EMR_MR20_Pos)                 /*!< 0x00100000 */
#define EXTI_EMR_MR20              EXTI_EMR_MR20_Msk                           /*!< Event Mask on line 20 */
#define EXTI_EMR_MR22_Pos          (22U)                                       
#define EXTI_EMR_MR22_Msk          (0x1U << EXTI_EMR_MR22_Pos)                 /*!< 0x00400000 */
#define EXTI_EMR_MR22              EXTI_EMR_MR22_Msk                           /*!< Event Mask on line 22 */
#define EXTI_EMR_MR23_Pos          (23U)                                       
#define EXTI_EMR_MR23_Msk          (0x1U << EXTI_EMR_MR23_Pos)                 /*!< 0x00800000 */
#define EXTI_EMR_MR23              EXTI_EMR_MR23_Msk                           /*!< Event Mask on line 23 */
#define EXTI_EMR_MR24_Pos          (24U)                                       
#define EXTI_EMR_MR24_Msk          (0x1U << EXTI_EMR_MR24_Pos)                 /*!< 0x01000000 */
#define EXTI_EMR_MR24              EXTI_EMR_MR24_Msk                           /*!< Event Mask on line 24 */
#define EXTI_EMR_MR25_Pos          (25U)                                       
#define EXTI_EMR_MR25_Msk          (0x1U << EXTI_EMR_MR25_Pos)                 /*!< 0x02000000 */
#define EXTI_EMR_MR25              EXTI_EMR_MR25_Msk                           /*!< Event Mask on line 25 */
#define EXTI_EMR_MR27_Pos          (27U)                                       
#define EXTI_EMR_MR27_Msk          (0x1U << EXTI_EMR_MR27_Pos)                 /*!< 0x08000000 */
#define EXTI_EMR_MR27              EXTI_EMR_MR27_Msk                           /*!< Event Mask on line 27 */
#define EXTI_EMR_MR30_Pos          (30U)                                       
#define EXTI_EMR_MR30_Msk          (0x1U << EXTI_EMR_MR30_Pos)                 /*!< 0x40000000 */
#define EXTI_EMR_MR30              EXTI_EMR_MR30_Msk                           /*!< Event Mask on line 30 */

/* References Defines */
#define  EXTI_EMR_EM0 EXTI_EMR_MR0
#define  EXTI_EMR_EM1 EXTI_EMR_MR1
#define  EXTI_EMR_EM2 EXTI_EMR_MR2
#define  EXTI_EMR_EM3 EXTI_EMR_MR3
#define  EXTI_EMR_EM4 EXTI_EMR_MR4
#define  EXTI_EMR_EM5 EXTI_EMR_MR5
#define  EXTI_EMR_EM6 EXTI_EMR_MR6
#define  EXTI_EMR_EM7 EXTI_EMR_MR7
#define  EXTI_EMR_EM8 EXTI_EMR_MR8
#define  EXTI_EMR_EM9 EXTI_EMR_MR9
#define  EXTI_EMR_EM10 EXTI_EMR_MR10
#define  EXTI_EMR_EM11 EXTI_EMR_MR11
#define  EXTI_EMR_EM12 EXTI_EMR_MR12
#define  EXTI_EMR_EM13 EXTI_EMR_MR13
#define  EXTI_EMR_EM14 EXTI_EMR_MR14
#define  EXTI_EMR_EM15 EXTI_EMR_MR15
#define  EXTI_EMR_EM16 EXTI_EMR_MR16
#define  EXTI_EMR_EM17 EXTI_EMR_MR17
#if defined(EXTI_EMR_MR18)
#define  EXTI_EMR_EM18 EXTI_EMR_MR18
#endif
#define  EXTI_EMR_EM19 EXTI_EMR_MR19
#define  EXTI_EMR_EM20 EXTI_EMR_MR20
#if defined(EXTI_EMR_MR21)
#define  EXTI_EMR_EM21 EXTI_EMR_MR21
#endif
#define  EXTI_EMR_EM22 EXTI_EMR_MR22
#define  EXTI_EMR_EM23 EXTI_EMR_MR23
#define  EXTI_EMR_EM24 EXTI_EMR_MR24
#define  EXTI_EMR_EM25 EXTI_EMR_MR25
#if defined(EXTI_EMR_MR26)
#define  EXTI_EMR_EM26 EXTI_EMR_MR26
#endif
#define  EXTI_EMR_EM27 EXTI_EMR_MR27
#if defined(EXTI_EMR_MR28)
#define  EXTI_EMR_EM28 EXTI_EMR_MR28
#endif
#if defined(EXTI_EMR_MR29)
#define  EXTI_EMR_EM29 EXTI_EMR_MR29
#endif
#define  EXTI_EMR_EM30 EXTI_EMR_MR30
#if defined(EXTI_EMR_MR31)
#define  EXTI_EMR_EM31 EXTI_EMR_MR31
#endif

/******************  Bit definition for EXTI_RTSR register  *******************/
#define EXTI_RTSR_TR0_Pos          (0U)                                        
#define EXTI_RTSR_TR0_Msk          (0x1U << EXTI_RTSR_TR0_Pos)                 /*!< 0x00000001 */
#define EXTI_RTSR_TR0              EXTI_RTSR_TR0_Msk                           /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos          (1U)                                        
#define EXTI_RTSR_TR1_Msk          (0x1U << EXTI_RTSR_TR1_Pos)                 /*!< 0x00000002 */
#define EXTI_RTSR_TR1              EXTI_RTSR_TR1_Msk                           /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos          (2U)                                        
#define EXTI_RTSR_TR2_Msk          (0x1U << EXTI_RTSR_TR2_Pos)                 /*!< 0x00000004 */
#define EXTI_RTSR_TR2              EXTI_RTSR_TR2_Msk                           /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos          (3U)                                        
#define EXTI_RTSR_TR3_Msk          (0x1U << EXTI_RTSR_TR3_Pos)                 /*!< 0x00000008 */
#define EXTI_RTSR_TR3              EXTI_RTSR_TR3_Msk                           /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos          (4U)                                        
#define EXTI_RTSR_TR4_Msk          (0x1U << EXTI_RTSR_TR4_Pos)                 /*!< 0x00000010 */
#define EXTI_RTSR_TR4              EXTI_RTSR_TR4_Msk                           /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos          (5U)                                        
#define EXTI_RTSR_TR5_Msk          (0x1U << EXTI_RTSR_TR5_Pos)                 /*!< 0x00000020 */
#define EXTI_RTSR_TR5              EXTI_RTSR_TR5_Msk                           /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos          (6U)                                        
#define EXTI_RTSR_TR6_Msk          (0x1U << EXTI_RTSR_TR6_Pos)                 /*!< 0x00000040 */
#define EXTI_RTSR_TR6              EXTI_RTSR_TR6_Msk                           /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos          (7U)                                        
#define EXTI_RTSR_TR7_Msk          (0x1U << EXTI_RTSR_TR7_Pos)                 /*!< 0x00000080 */
#define EXTI_RTSR_TR7              EXTI_RTSR_TR7_Msk                           /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos          (8U)                                        
#define EXTI_RTSR_TR8_Msk          (0x1U << EXTI_RTSR_TR8_Pos)                 /*!< 0x00000100 */
#define EXTI_RTSR_TR8              EXTI_RTSR_TR8_Msk                           /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos          (9U)                                        
#define EXTI_RTSR_TR9_Msk          (0x1U << EXTI_RTSR_TR9_Pos)                 /*!< 0x00000200 */
#define EXTI_RTSR_TR9              EXTI_RTSR_TR9_Msk                           /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos         (10U)                                       
#define EXTI_RTSR_TR10_Msk         (0x1U << EXTI_RTSR_TR10_Pos)                /*!< 0x00000400 */
#define EXTI_RTSR_TR10             EXTI_RTSR_TR10_Msk                          /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos         (11U)                                       
#define EXTI_RTSR_TR11_Msk         (0x1U << EXTI_RTSR_TR11_Pos)                /*!< 0x00000800 */
#define EXTI_RTSR_TR11             EXTI_RTSR_TR11_Msk                          /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos         (12U)                                       
#define EXTI_RTSR_TR12_Msk         (0x1U << EXTI_RTSR_TR12_Pos)                /*!< 0x00001000 */
#define EXTI_RTSR_TR12             EXTI_RTSR_TR12_Msk                          /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos         (13U)                                       
#define EXTI_RTSR_TR13_Msk         (0x1U << EXTI_RTSR_TR13_Pos)                /*!< 0x00002000 */
#define EXTI_RTSR_TR13             EXTI_RTSR_TR13_Msk                          /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos         (14U)                                       
#define EXTI_RTSR_TR14_Msk         (0x1U << EXTI_RTSR_TR14_Pos)                /*!< 0x00004000 */
#define EXTI_RTSR_TR14             EXTI_RTSR_TR14_Msk                          /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos         (15U)                                       
#define EXTI_RTSR_TR15_Msk         (0x1U << EXTI_RTSR_TR15_Pos)                /*!< 0x00008000 */
#define EXTI_RTSR_TR15             EXTI_RTSR_TR15_Msk                          /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos         (16U)                                       
#define EXTI_RTSR_TR16_Msk         (0x1U << EXTI_RTSR_TR16_Pos)                /*!< 0x00010000 */
#define EXTI_RTSR_TR16             EXTI_RTSR_TR16_Msk                          /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos         (17U)                                       
#define EXTI_RTSR_TR17_Msk         (0x1U << EXTI_RTSR_TR17_Pos)                /*!< 0x00020000 */
#define EXTI_RTSR_TR17             EXTI_RTSR_TR17_Msk                          /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR19_Pos         (19U)                                       
#define EXTI_RTSR_TR19_Msk         (0x1U << EXTI_RTSR_TR19_Pos)                /*!< 0x00080000 */
#define EXTI_RTSR_TR19             EXTI_RTSR_TR19_Msk                          /*!< Rising trigger event configuration bit of line 19 */
#define EXTI_RTSR_TR20_Pos         (20U)                                       
#define EXTI_RTSR_TR20_Msk         (0x1U << EXTI_RTSR_TR20_Pos)                /*!< 0x00100000 */
#define EXTI_RTSR_TR20             EXTI_RTSR_TR20_Msk                          /*!< Rising trigger event configuration bit of line 20 */
#define EXTI_RTSR_TR22_Pos         (22U)                                       
#define EXTI_RTSR_TR22_Msk         (0x1U << EXTI_RTSR_TR22_Pos)                /*!< 0x00400000 */
#define EXTI_RTSR_TR22             EXTI_RTSR_TR22_Msk                          /*!< Rising trigger event configuration bit of line 22 */
#define EXTI_RTSR_TR30_Pos         (30U)                                       
#define EXTI_RTSR_TR30_Msk         (0x1U << EXTI_RTSR_TR30_Pos)                /*!< 0x40000000 */
#define EXTI_RTSR_TR30             EXTI_RTSR_TR30_Msk                          /*!< Rising trigger event configuration bit of line 30 */

/* References Defines */
#define EXTI_RTSR_RT0 EXTI_RTSR_TR0
#define EXTI_RTSR_RT1 EXTI_RTSR_TR1
#define EXTI_RTSR_RT2 EXTI_RTSR_TR2
#define EXTI_RTSR_RT3 EXTI_RTSR_TR3
#define EXTI_RTSR_RT4 EXTI_RTSR_TR4
#define EXTI_RTSR_RT5 EXTI_RTSR_TR5
#define EXTI_RTSR_RT6 EXTI_RTSR_TR6
#define EXTI_RTSR_RT7 EXTI_RTSR_TR7
#define EXTI_RTSR_RT8 EXTI_RTSR_TR8
#define EXTI_RTSR_RT9 EXTI_RTSR_TR9
#define EXTI_RTSR_RT10 EXTI_RTSR_TR10
#define EXTI_RTSR_RT11 EXTI_RTSR_TR11
#define EXTI_RTSR_RT12 EXTI_RTSR_TR12
#define EXTI_RTSR_RT13 EXTI_RTSR_TR13
#define EXTI_RTSR_RT14 EXTI_RTSR_TR14
#define EXTI_RTSR_RT15 EXTI_RTSR_TR15
#define EXTI_RTSR_RT16 EXTI_RTSR_TR16
#define EXTI_RTSR_RT17 EXTI_RTSR_TR17
#if defined(EXTI_RTSR_TR18)
#define EXTI_RTSR_RT18 EXTI_RTSR_TR18
#endif
#define EXTI_RTSR_RT19 EXTI_RTSR_TR19
#define EXTI_RTSR_RT20 EXTI_RTSR_TR20
#if defined(EXTI_RTSR_TR21)
#define EXTI_RTSR_RT21 EXTI_RTSR_TR21
#endif
#define EXTI_RTSR_RT22 EXTI_RTSR_TR22
#if defined(EXTI_RTSR_TR23)
#define EXTI_RTSR_RT23 EXTI_RTSR_TR23
#endif
#if defined(EXTI_RTSR_TR24)
#define EXTI_RTSR_RT24 EXTI_RTSR_TR24
#endif
#if defined(EXTI_RTSR_TR25)
#define EXTI_RTSR_RT25 EXTI_RTSR_TR25
#endif
#if defined(EXTI_RTSR_TR26)
#define EXTI_RTSR_RT26 EXTI_RTSR_TR26
#endif
#if defined(EXTI_RTSR_TR27)
#define EXTI_RTSR_RT27 EXTI_RTSR_TR27
#endif
#if defined(EXTI_RTSR_TR28)
#define EXTI_RTSR_RT28 EXTI_RTSR_TR28
#endif
#if defined(EXTI_RTSR_TR29)
#define EXTI_RTSR_RT29 EXTI_RTSR_TR29
#endif
#define EXTI_RTSR_RT30 EXTI_RTSR_TR30
#if defined(EXTI_RTSR_TR31)
#define EXTI_RTSR_RT31 EXTI_RTSR_TR31
#endif

/******************  Bit definition for EXTI_FTSR register  *******************/
#define EXTI_FTSR_TR0_Pos          (0U)                                        
#define EXTI_FTSR_TR0_Msk          (0x1U << EXTI_FTSR_TR0_Pos)                 /*!< 0x00000001 */
#define EXTI_FTSR_TR0              EXTI_FTSR_TR0_Msk                           /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos          (1U)                                        
#define EXTI_FTSR_TR1_Msk          (0x1U << EXTI_FTSR_TR1_Pos)                 /*!< 0x00000002 */
#define EXTI_FTSR_TR1              EXTI_FTSR_TR1_Msk                           /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos          (2U)                                        
#define EXTI_FTSR_TR2_Msk          (0x1U << EXTI_FTSR_TR2_Pos)                 /*!< 0x00000004 */
#define EXTI_FTSR_TR2              EXTI_FTSR_TR2_Msk                           /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos          (3U)                                        
#define EXTI_FTSR_TR3_Msk          (0x1U << EXTI_FTSR_TR3_Pos)                 /*!< 0x00000008 */
#define EXTI_FTSR_TR3              EXTI_FTSR_TR3_Msk                           /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos          (4U)                                        
#define EXTI_FTSR_TR4_Msk          (0x1U << EXTI_FTSR_TR4_Pos)                 /*!< 0x00000010 */
#define EXTI_FTSR_TR4              EXTI_FTSR_TR4_Msk                           /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos          (5U)                                        
#define EXTI_FTSR_TR5_Msk          (0x1U << EXTI_FTSR_TR5_Pos)                 /*!< 0x00000020 */
#define EXTI_FTSR_TR5              EXTI_FTSR_TR5_Msk                           /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos          (6U)                                        
#define EXTI_FTSR_TR6_Msk          (0x1U << EXTI_FTSR_TR6_Pos)                 /*!< 0x00000040 */
#define EXTI_FTSR_TR6              EXTI_FTSR_TR6_Msk                           /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos          (7U)                                        
#define EXTI_FTSR_TR7_Msk          (0x1U << EXTI_FTSR_TR7_Pos)                 /*!< 0x00000080 */
#define EXTI_FTSR_TR7              EXTI_FTSR_TR7_Msk                           /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos          (8U)                                        
#define EXTI_FTSR_TR8_Msk          (0x1U << EXTI_FTSR_TR8_Pos)                 /*!< 0x00000100 */
#define EXTI_FTSR_TR8              EXTI_FTSR_TR8_Msk                           /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos          (9U)                                        
#define EXTI_FTSR_TR9_Msk          (0x1U << EXTI_FTSR_TR9_Pos)                 /*!< 0x00000200 */
#define EXTI_FTSR_TR9              EXTI_FTSR_TR9_Msk                           /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos         (10U)                                       
#define EXTI_FTSR_TR10_Msk         (0x1U << EXTI_FTSR_TR10_Pos)                /*!< 0x00000400 */
#define EXTI_FTSR_TR10             EXTI_FTSR_TR10_Msk                          /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos         (11U)                                       
#define EXTI_FTSR_TR11_Msk         (0x1U << EXTI_FTSR_TR11_Pos)                /*!< 0x00000800 */
#define EXTI_FTSR_TR11             EXTI_FTSR_TR11_Msk                          /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos         (12U)                                       
#define EXTI_FTSR_TR12_Msk         (0x1U << EXTI_FTSR_TR12_Pos)                /*!< 0x00001000 */
#define EXTI_FTSR_TR12             EXTI_FTSR_TR12_Msk                          /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos         (13U)                                       
#define EXTI_FTSR_TR13_Msk         (0x1U << EXTI_FTSR_TR13_Pos)                /*!< 0x00002000 */
#define EXTI_FTSR_TR13             EXTI_FTSR_TR13_Msk                          /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos         (14U)                                       
#define EXTI_FTSR_TR14_Msk         (0x1U << EXTI_FTSR_TR14_Pos)                /*!< 0x00004000 */
#define EXTI_FTSR_TR14             EXTI_FTSR_TR14_Msk                          /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos         (15U)                                       
#define EXTI_FTSR_TR15_Msk         (0x1U << EXTI_FTSR_TR15_Pos)                /*!< 0x00008000 */
#define EXTI_FTSR_TR15             EXTI_FTSR_TR15_Msk                          /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos         (16U)                                       
#define EXTI_FTSR_TR16_Msk         (0x1U << EXTI_FTSR_TR16_Pos)                /*!< 0x00010000 */
#define EXTI_FTSR_TR16             EXTI_FTSR_TR16_Msk                          /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos         (17U)                                       
#define EXTI_FTSR_TR17_Msk         (0x1U << EXTI_FTSR_TR17_Pos)                /*!< 0x00020000 */
#define EXTI_FTSR_TR17             EXTI_FTSR_TR17_Msk                          /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR19_Pos         (19U)                                       
#define EXTI_FTSR_TR19_Msk         (0x1U << EXTI_FTSR_TR19_Pos)                /*!< 0x00080000 */
#define EXTI_FTSR_TR19             EXTI_FTSR_TR19_Msk                          /*!< Falling trigger event configuration bit of line 19 */
#define EXTI_FTSR_TR20_Pos         (20U)                                       
#define EXTI_FTSR_TR20_Msk         (0x1U << EXTI_FTSR_TR20_Pos)                /*!< 0x00100000 */
#define EXTI_FTSR_TR20             EXTI_FTSR_TR20_Msk                          /*!< Falling trigger event configuration bit of line 20 */
#define EXTI_FTSR_TR22_Pos         (22U)                                       
#define EXTI_FTSR_TR22_Msk         (0x1U << EXTI_FTSR_TR22_Pos)                /*!< 0x00400000 */
#define EXTI_FTSR_TR22             EXTI_FTSR_TR22_Msk                          /*!< Falling trigger event configuration bit of line 22 */
#define EXTI_FTSR_TR30_Pos         (30U)                                       
#define EXTI_FTSR_TR30_Msk         (0x1U << EXTI_FTSR_TR30_Pos)                /*!< 0x40000000 */
#define EXTI_FTSR_TR30             EXTI_FTSR_TR30_Msk                          /*!< Falling trigger event configuration bit of line 30 */

/* References Defines */
#define EXTI_FTSR_FT0 EXTI_FTSR_TR0
#define EXTI_FTSR_FT1 EXTI_FTSR_TR1
#define EXTI_FTSR_FT2 EXTI_FTSR_TR2
#define EXTI_FTSR_FT3 EXTI_FTSR_TR3
#define EXTI_FTSR_FT4 EXTI_FTSR_TR4
#define EXTI_FTSR_FT5 EXTI_FTSR_TR5
#define EXTI_FTSR_FT6 EXTI_FTSR_TR6
#define EXTI_FTSR_FT7 EXTI_FTSR_TR7
#define EXTI_FTSR_FT8 EXTI_FTSR_TR8
#define EXTI_FTSR_FT9 EXTI_FTSR_TR9
#define EXTI_FTSR_FT10 EXTI_FTSR_TR10
#define EXTI_FTSR_FT11 EXTI_FTSR_TR11
#define EXTI_FTSR_FT12 EXTI_FTSR_TR12
#define EXTI_FTSR_FT13 EXTI_FTSR_TR13
#define EXTI_FTSR_FT14 EXTI_FTSR_TR14
#define EXTI_FTSR_FT15 EXTI_FTSR_TR15
#define EXTI_FTSR_FT16 EXTI_FTSR_TR16
#define EXTI_FTSR_FT17 EXTI_FTSR_TR17
#if defined(EXTI_FTSR_TR18)
#define EXTI_FTSR_FT18 EXTI_FTSR_TR18
#endif
#define EXTI_FTSR_FT19 EXTI_FTSR_TR19
#define EXTI_FTSR_FT20 EXTI_FTSR_TR20
#if defined(EXTI_FTSR_TR21)
#define EXTI_FTSR_FT21 EXTI_FTSR_TR21
#endif
#define EXTI_FTSR_FT22 EXTI_FTSR_TR22
#if defined(EXTI_FTSR_TR23)
#define EXTI_FTSR_FT23 EXTI_FTSR_TR23
#endif
#if defined(EXTI_FTSR_TR24)
#define EXTI_FTSR_FT24 EXTI_FTSR_TR24
#endif
#if defined(EXTI_FTSR_TR25)
#define EXTI_FTSR_FT25 EXTI_FTSR_TR25
#endif
#if defined(EXTI_FTSR_TR26)
#define EXTI_FTSR_FT26 EXTI_FTSR_TR26
#endif
#if defined(EXTI_FTSR_TR27)
#define EXTI_FTSR_FT27 EXTI_FTSR_TR27
#endif
#if defined(EXTI_FTSR_TR28)
#define EXTI_FTSR_FT28 EXTI_FTSR_TR28
#endif
#if defined(EXTI_FTSR_TR29)
#define EXTI_FTSR_FT29 EXTI_FTSR_TR29
#endif
#define EXTI_FTSR_FT30 EXTI_FTSR_TR30
#if defined(EXTI_FTSR_TR31)
#define EXTI_FTSR_FT31 EXTI_FTSR_TR31
#endif

/******************  Bit definition for EXTI_SWIER register  ******************/
#define EXTI_SWIER_SWIER0_Pos      (0U)                                        
#define EXTI_SWIER_SWIER0_Msk      (0x1U << EXTI_SWIER_SWIER0_Pos)             /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0          EXTI_SWIER_SWIER0_Msk                       /*!< Software Interrupt on line 0 */
#define EXTI_SWIER_SWIER1_Pos      (1U)                                        
#define EXTI_SWIER_SWIER1_Msk      (0x1U << EXTI_SWIER_SWIER1_Pos)             /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1          EXTI_SWIER_SWIER1_Msk                       /*!< Software Interrupt on line 1 */
#define EXTI_SWIER_SWIER2_Pos      (2U)                                        
#define EXTI_SWIER_SWIER2_Msk      (0x1U << EXTI_SWIER_SWIER2_Pos)             /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2          EXTI_SWIER_SWIER2_Msk                       /*!< Software Interrupt on line 2 */
#define EXTI_SWIER_SWIER3_Pos      (3U)                                        
#define EXTI_SWIER_SWIER3_Msk      (0x1U << EXTI_SWIER_SWIER3_Pos)             /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3          EXTI_SWIER_SWIER3_Msk                       /*!< Software Interrupt on line 3 */
#define EXTI_SWIER_SWIER4_Pos      (4U)                                        
#define EXTI_SWIER_SWIER4_Msk      (0x1U << EXTI_SWIER_SWIER4_Pos)             /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4          EXTI_SWIER_SWIER4_Msk                       /*!< Software Interrupt on line 4 */
#define EXTI_SWIER_SWIER5_Pos      (5U)                                        
#define EXTI_SWIER_SWIER5_Msk      (0x1U << EXTI_SWIER_SWIER5_Pos)             /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5          EXTI_SWIER_SWIER5_Msk                       /*!< Software Interrupt on line 5 */
#define EXTI_SWIER_SWIER6_Pos      (6U)                                        
#define EXTI_SWIER_SWIER6_Msk      (0x1U << EXTI_SWIER_SWIER6_Pos)             /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6          EXTI_SWIER_SWIER6_Msk                       /*!< Software Interrupt on line 6 */
#define EXTI_SWIER_SWIER7_Pos      (7U)                                        
#define EXTI_SWIER_SWIER7_Msk      (0x1U << EXTI_SWIER_SWIER7_Pos)             /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7          EXTI_SWIER_SWIER7_Msk                       /*!< Software Interrupt on line 7 */
#define EXTI_SWIER_SWIER8_Pos      (8U)                                        
#define EXTI_SWIER_SWIER8_Msk      (0x1U << EXTI_SWIER_SWIER8_Pos)             /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8          EXTI_SWIER_SWIER8_Msk                       /*!< Software Interrupt on line 8 */
#define EXTI_SWIER_SWIER9_Pos      (9U)                                        
#define EXTI_SWIER_SWIER9_Msk      (0x1U << EXTI_SWIER_SWIER9_Pos)             /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9          EXTI_SWIER_SWIER9_Msk                       /*!< Software Interrupt on line 9 */
#define EXTI_SWIER_SWIER10_Pos     (10U)                                       
#define EXTI_SWIER_SWIER10_Msk     (0x1U << EXTI_SWIER_SWIER10_Pos)            /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10         EXTI_SWIER_SWIER10_Msk                      /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos     (11U)                                       
#define EXTI_SWIER_SWIER11_Msk     (0x1U << EXTI_SWIER_SWIER11_Pos)            /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11         EXTI_SWIER_SWIER11_Msk                      /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos     (12U)                                       
#define EXTI_SWIER_SWIER12_Msk     (0x1U << EXTI_SWIER_SWIER12_Pos)            /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12         EXTI_SWIER_SWIER12_Msk                      /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos     (13U)                                       
#define EXTI_SWIER_SWIER13_Msk     (0x1U << EXTI_SWIER_SWIER13_Pos)            /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13         EXTI_SWIER_SWIER13_Msk                      /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos     (14U)                                       
#define EXTI_SWIER_SWIER14_Msk     (0x1U << EXTI_SWIER_SWIER14_Pos)            /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14         EXTI_SWIER_SWIER14_Msk                      /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos     (15U)                                       
#define EXTI_SWIER_SWIER15_Msk     (0x1U << EXTI_SWIER_SWIER15_Pos)            /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15         EXTI_SWIER_SWIER15_Msk                      /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos     (16U)                                       
#define EXTI_SWIER_SWIER16_Msk     (0x1U << EXTI_SWIER_SWIER16_Pos)            /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16         EXTI_SWIER_SWIER16_Msk                      /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos     (17U)                                       
#define EXTI_SWIER_SWIER17_Msk     (0x1U << EXTI_SWIER_SWIER17_Pos)            /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17         EXTI_SWIER_SWIER17_Msk                      /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER19_Pos     (19U)                                       
#define EXTI_SWIER_SWIER19_Msk     (0x1U << EXTI_SWIER_SWIER19_Pos)            /*!< 0x00080000 */
#define EXTI_SWIER_SWIER19         EXTI_SWIER_SWIER19_Msk                      /*!< Software Interrupt on line 19 */
#define EXTI_SWIER_SWIER20_Pos     (20U)                                       
#define EXTI_SWIER_SWIER20_Msk     (0x1U << EXTI_SWIER_SWIER20_Pos)            /*!< 0x00100000 */
#define EXTI_SWIER_SWIER20         EXTI_SWIER_SWIER20_Msk                      /*!< Software Interrupt on line 20 */
#define EXTI_SWIER_SWIER22_Pos     (22U)                                       
#define EXTI_SWIER_SWIER22_Msk     (0x1U << EXTI_SWIER_SWIER22_Pos)            /*!< 0x00400000 */
#define EXTI_SWIER_SWIER22         EXTI_SWIER_SWIER22_Msk                      /*!< Software Interrupt on line 22 */
#define EXTI_SWIER_SWIER30_Pos     (30U)                                       
#define EXTI_SWIER_SWIER30_Msk     (0x1U << EXTI_SWIER_SWIER30_Pos)            /*!< 0x40000000 */
#define EXTI_SWIER_SWIER30         EXTI_SWIER_SWIER30_Msk                      /*!< Software Interrupt on line 30 */

/* References Defines */
#define EXTI_SWIER_SWI0 EXTI_SWIER_SWIER0
#define EXTI_SWIER_SWI1 EXTI_SWIER_SWIER1
#define EXTI_SWIER_SWI2 EXTI_SWIER_SWIER2
#define EXTI_SWIER_SWI3 EXTI_SWIER_SWIER3
#define EXTI_SWIER_SWI4 EXTI_SWIER_SWIER4
#define EXTI_SWIER_SWI5 EXTI_SWIER_SWIER5
#define EXTI_SWIER_SWI6 EXTI_SWIER_SWIER6
#define EXTI_SWIER_SWI7 EXTI_SWIER_SWIER7
#define EXTI_SWIER_SWI8 EXTI_SWIER_SWIER8
#define EXTI_SWIER_SWI9 EXTI_SWIER_SWIER9
#define EXTI_SWIER_SWI10 EXTI_SWIER_SWIER10
#define EXTI_SWIER_SWI11 EXTI_SWIER_SWIER11
#define EXTI_SWIER_SWI12 EXTI_SWIER_SWIER12
#define EXTI_SWIER_SWI13 EXTI_SWIER_SWIER13
#define EXTI_SWIER_SWI14 EXTI_SWIER_SWIER14
#define EXTI_SWIER_SWI15 EXTI_SWIER_SWIER15
#define EXTI_SWIER_SWI16 EXTI_SWIER_SWIER16
#define EXTI_SWIER_SWI17 EXTI_SWIER_SWIER17
#if defined(EXTI_SWIER_SWIER18)
#define EXTI_SWIER_SWI18 EXTI_SWIER_SWIER18
#endif
#define EXTI_SWIER_SWI19 EXTI_SWIER_SWIER19
#define EXTI_SWIER_SWI20 EXTI_SWIER_SWIER20
#if defined(EXTI_SWIER_SWIER21)
#define EXTI_SWIER_SWI21 EXTI_SWIER_SWIER21
#endif
#define EXTI_SWIER_SWI22 EXTI_SWIER_SWIER22
#if defined(EXTI_SWIER_SWIER23)
#define EXTI_SWIER_SWI23 EXTI_SWIER_SWIER23
#endif
#if defined(EXTI_SWIER_SWIER24)
#define EXTI_SWIER_SWI24 EXTI_SWIER_SWIER24
#endif
#if defined(EXTI_SWIER_SWIER25)
#define EXTI_SWIER_SWI25 EXTI_SWIER_SWIER25
#endif
#if defined(EXTI_SWIER_SWIER26)
#define EXTI_SWIER_SWI26 EXTI_SWIER_SWIER26
#endif
#if defined(EXTI_SWIER_SWIER27)
#define EXTI_SWIER_SWI27 EXTI_SWIER_SWIER27
#endif
#if defined(EXTI_SWIER_SWIER28)
#define EXTI_SWIER_SWI28 EXTI_SWIER_SWIER28
#endif
#if defined(EXTI_SWIER_SWIER29)
#define EXTI_SWIER_SWI29 EXTI_SWIER_SWIER29
#endif
#define EXTI_SWIER_SWI30 EXTI_SWIER_SWIER30
#if defined(EXTI_SWIER_SWIER31)
#define EXTI_SWIER_SWI31 EXTI_SWIER_SWIER31
#endif

/*******************  Bit definition for EXTI_PR register  ********************/
#define EXTI_PR_PR0_Pos            (0U)                                        
#define EXTI_PR_PR0_Msk            (0x1U << EXTI_PR_PR0_Pos)                   /*!< 0x00000001 */
#define EXTI_PR_PR0                EXTI_PR_PR0_Msk                             /*!< Pending bit for line 0 */
#define EXTI_PR_PR1_Pos            (1U)                                        
#define EXTI_PR_PR1_Msk            (0x1U << EXTI_PR_PR1_Pos)                   /*!< 0x00000002 */
#define EXTI_PR_PR1                EXTI_PR_PR1_Msk                             /*!< Pending bit for line 1 */
#define EXTI_PR_PR2_Pos            (2U)                                        
#define EXTI_PR_PR2_Msk            (0x1U << EXTI_PR_PR2_Pos)                   /*!< 0x00000004 */
#define EXTI_PR_PR2                EXTI_PR_PR2_Msk                             /*!< Pending bit for line 2 */
#define EXTI_PR_PR3_Pos            (3U)                                        
#define EXTI_PR_PR3_Msk            (0x1U << EXTI_PR_PR3_Pos)                   /*!< 0x00000008 */
#define EXTI_PR_PR3                EXTI_PR_PR3_Msk                             /*!< Pending bit for line 3 */
#define EXTI_PR_PR4_Pos            (4U)                                        
#define EXTI_PR_PR4_Msk            (0x1U << EXTI_PR_PR4_Pos)                   /*!< 0x00000010 */
#define EXTI_PR_PR4                EXTI_PR_PR4_Msk                             /*!< Pending bit for line 4 */
#define EXTI_PR_PR5_Pos            (5U)                                        
#define EXTI_PR_PR5_Msk            (0x1U << EXTI_PR_PR5_Pos)                   /*!< 0x00000020 */
#define EXTI_PR_PR5                EXTI_PR_PR5_Msk                             /*!< Pending bit for line 5 */
#define EXTI_PR_PR6_Pos            (6U)                                        
#define EXTI_PR_PR6_Msk            (0x1U << EXTI_PR_PR6_Pos)                   /*!< 0x00000040 */
#define EXTI_PR_PR6                EXTI_PR_PR6_Msk                             /*!< Pending bit for line 6 */
#define EXTI_PR_PR7_Pos            (7U)                                        
#define EXTI_PR_PR7_Msk            (0x1U << EXTI_PR_PR7_Pos)                   /*!< 0x00000080 */
#define EXTI_PR_PR7                EXTI_PR_PR7_Msk                             /*!< Pending bit for line 7 */
#define EXTI_PR_PR8_Pos            (8U)                                        
#define EXTI_PR_PR8_Msk            (0x1U << EXTI_PR_PR8_Pos)                   /*!< 0x00000100 */
#define EXTI_PR_PR8                EXTI_PR_PR8_Msk                             /*!< Pending bit for line 8 */
#define EXTI_PR_PR9_Pos            (9U)                                        
#define EXTI_PR_PR9_Msk            (0x1U << EXTI_PR_PR9_Pos)                   /*!< 0x00000200 */
#define EXTI_PR_PR9                EXTI_PR_PR9_Msk                             /*!< Pending bit for line 9 */
#define EXTI_PR_PR10_Pos           (10U)                                       
#define EXTI_PR_PR10_Msk           (0x1U << EXTI_PR_PR10_Pos)                  /*!< 0x00000400 */
#define EXTI_PR_PR10               EXTI_PR_PR10_Msk                            /*!< Pending bit for line 10 */
#define EXTI_PR_PR11_Pos           (11U)                                       
#define EXTI_PR_PR11_Msk           (0x1U << EXTI_PR_PR11_Pos)                  /*!< 0x00000800 */
#define EXTI_PR_PR11               EXTI_PR_PR11_Msk                            /*!< Pending bit for line 11 */
#define EXTI_PR_PR12_Pos           (12U)                                       
#define EXTI_PR_PR12_Msk           (0x1U << EXTI_PR_PR12_Pos)                  /*!< 0x00001000 */
#define EXTI_PR_PR12               EXTI_PR_PR12_Msk                            /*!< Pending bit for line 12 */
#define EXTI_PR_PR13_Pos           (13U)                                       
#define EXTI_PR_PR13_Msk           (0x1U << EXTI_PR_PR13_Pos)                  /*!< 0x00002000 */
#define EXTI_PR_PR13               EXTI_PR_PR13_Msk                            /*!< Pending bit for line 13 */
#define EXTI_PR_PR14_Pos           (14U)                                       
#define EXTI_PR_PR14_Msk           (0x1U << EXTI_PR_PR14_Pos)                  /*!< 0x00004000 */
#define EXTI_PR_PR14               EXTI_PR_PR14_Msk                            /*!< Pending bit for line 14 */
#define EXTI_PR_PR15_Pos           (15U)                                       
#define EXTI_PR_PR15_Msk           (0x1U << EXTI_PR_PR15_Pos)                  /*!< 0x00008000 */
#define EXTI_PR_PR15               EXTI_PR_PR15_Msk                            /*!< Pending bit for line 15 */
#define EXTI_PR_PR16_Pos           (16U)                                       
#define EXTI_PR_PR16_Msk           (0x1U << EXTI_PR_PR16_Pos)                  /*!< 0x00010000 */
#define EXTI_PR_PR16               EXTI_PR_PR16_Msk                            /*!< Pending bit for line 16 */
#define EXTI_PR_PR17_Pos           (17U)                                       
#define EXTI_PR_PR17_Msk           (0x1U << EXTI_PR_PR17_Pos)                  /*!< 0x00020000 */
#define EXTI_PR_PR17               EXTI_PR_PR17_Msk                            /*!< Pending bit for line 17 */
#define EXTI_PR_PR19_Pos           (19U)                                       
#define EXTI_PR_PR19_Msk           (0x1U << EXTI_PR_PR19_Pos)                  /*!< 0x00080000 */
#define EXTI_PR_PR19               EXTI_PR_PR19_Msk                            /*!< Pending bit for line 19 */
#define EXTI_PR_PR20_Pos           (20U)                                       
#define EXTI_PR_PR20_Msk           (0x1U << EXTI_PR_PR20_Pos)                  /*!< 0x00100000 */
#define EXTI_PR_PR20               EXTI_PR_PR20_Msk                            /*!< Pending bit for line 20 */
#define EXTI_PR_PR22_Pos           (22U)                                       
#define EXTI_PR_PR22_Msk           (0x1U << EXTI_PR_PR22_Pos)                  /*!< 0x00400000 */
#define EXTI_PR_PR22               EXTI_PR_PR22_Msk                            /*!< Pending bit for line 22 */
#define EXTI_PR_PR30_Pos           (30U)                                       
#define EXTI_PR_PR30_Msk           (0x1U << EXTI_PR_PR30_Pos)                  /*!< 0x40000000 */
#define EXTI_PR_PR30               EXTI_PR_PR30_Msk                            /*!< Pending bit for line 30 */

/* References Defines */
#define EXTI_PR_PIF0 EXTI_PR_PR0
#define EXTI_PR_PIF1 EXTI_PR_PR1
#define EXTI_PR_PIF2 EXTI_PR_PR2
#define EXTI_PR_PIF3 EXTI_PR_PR3
#define EXTI_PR_PIF4 EXTI_PR_PR4
#define EXTI_PR_PIF5 EXTI_PR_PR5
#define EXTI_PR_PIF6 EXTI_PR_PR6
#define EXTI_PR_PIF6 EXTI_PR_PR6
#define EXTI_PR_PIF7 EXTI_PR_PR7
#define EXTI_PR_PIF8 EXTI_PR_PR8
#define EXTI_PR_PIF9 EXTI_PR_PR9
#define EXTI_PR_PIF10 EXTI_PR_PR10
#define EXTI_PR_PIF11 EXTI_PR_PR11
#define EXTI_PR_PIF12 EXTI_PR_PR12
#define EXTI_PR_PIF13 EXTI_PR_PR13
#define EXTI_PR_PIF14 EXTI_PR_PR14
#define EXTI_PR_PIF15 EXTI_PR_PR15
#define EXTI_PR_PIF16 EXTI_PR_PR16
#define EXTI_PR_PIF17 EXTI_PR_PR17
#if defined(EXTI_PR_PR18)
#define EXTI_PR_PIF18 EXTI_PR_PR18
#endif
#define EXTI_PR_PIF19 EXTI_PR_PR19
#define EXTI_PR_PIF20 EXTI_PR_PR20
#if defined(EXTI_PR_PR21)
#define EXTI_PR_PIF21 EXTI_PR_PR21
#endif
#define EXTI_PR_PIF22 EXTI_PR_PR22
#if defined(EXTI_PR_PR23)
#define EXTI_PR_PIF23 EXTI_PR_PR23
#endif
#if defined(EXTI_PR_PR24)
#define EXTI_PR_PIF24 EXTI_PR_PR24
#endif
#if defined(EXTI_PR_PR25)
#define EXTI_PR_PIF25 EXTI_PR_PR25
#endif
#if defined(EXTI_PR_PR26)
#define EXTI_PR_PIF26 EXTI_PR_PR26
#endif
#if defined(EXTI_PR_PR27)
#define EXTI_PR_PIF27 EXTI_PR_PR27
#endif
#if defined(EXTI_PR_PR28)
#define EXTI_PR_PIF28 EXTI_PR_PR28
#endif
#if defined(EXTI_PR_PR29)
#define EXTI_PR_PIF29 EXTI_PR_PR29
#endif
#define EXTI_PR_PIF30 EXTI_PR_PR30
#if defined(EXTI_PR_PR31)
#define EXTI_PR_PIF31 EXTI_PR_PR31
#endif

#define EXTI_32_63_SUPPORT /* EXTI support more than 32 lines */

/*******************  Bit definition for EXTI_IMR2 register  ******************/
#define EXTI_IMR2_MR32_Pos         (0U)                                        
#define EXTI_IMR2_MR32_Msk         (0x1U << EXTI_IMR2_MR32_Pos)                /*!< 0x00000001 */
#define EXTI_IMR2_MR32             EXTI_IMR2_MR32_Msk                          /*!< Interrupt Mask on line 32 */

/* References Defines */

#define EXTI_IMR2_IM32 EXTI_IMR2_MR32
#if defined(EXTI_IMR2_MR33)
#define EXTI_IMR2_IM33 EXTI_IMR2_MR33
#endif
#if defined(EXTI_IMR2_MR34)
#define EXTI_IMR2_IM34 EXTI_IMR2_MR34
#endif
#if defined(EXTI_IMR2_MR35)
#define EXTI_IMR2_IM35 EXTI_IMR2_MR35
#endif

#define EXTI_IMR2_IM_Pos           (0U)                                        
#define EXTI_IMR2_IM_Msk           (0xFU << EXTI_IMR2_IM_Pos)                  /*!< 0x0000000F */
#define EXTI_IMR2_IM               EXTI_IMR2_IM_Msk                            

/*******************  Bit definition for EXTI_EMR2 ****************************/
#define EXTI_EMR2_MR32_Pos         (0U)                                        
#define EXTI_EMR2_MR32_Msk         (0x1U << EXTI_EMR2_MR32_Pos)                /*!< 0x00000001 */
#define EXTI_EMR2_MR32             EXTI_EMR2_MR32_Msk                          /*!< Event Mask on line 32 */

/* References Defines */
#define EXTI_EMR2_EM32 EXTI_EMR2_MR32
#if defined(EXTI_EMR2_MR33)
#define EXTI_EMR2_EM33 EXTI_EMR2_MR33
#endif
#if defined(EXTI_EMR2_MR34)
#define EXTI_EMR2_EM34 EXTI_EMR2_MR34
#endif
#if defined(EXTI_EMR2_MR35)
#define EXTI_EMR2_EM35 EXTI_EMR2_MR35
#endif

/******************  Bit definition for EXTI_RTSR2 register ********************/
#define EXTI_RTSR2_TR32_Pos        (0U)                                        
#define EXTI_RTSR2_TR32_Msk        (0x1U << EXTI_RTSR2_TR32_Pos)               /*!< 0x00000001 */
#define EXTI_RTSR2_TR32            EXTI_RTSR2_TR32_Msk                         /*!< Rising trigger event configuration bit of line 32 */

/* References Defines */
#define EXTI_RTSR2_RT32 EXTI_RTSR2_TR32
#if defined(EXTI_RTSR2_TR33)
#define EXTI_RTSR2_RT33 EXTI_RTSR2_TR33
#endif
#if defined(EXTI_RTSR2_TR34)
#define EXTI_RTSR2_RT34 EXTI_RTSR2_TR34
#endif
#if defined(EXTI_RTSR2_TR35)
#define EXTI_RTSR2_RT35 EXTI_RTSR2_TR35
#endif

/******************  Bit definition for EXTI_FTSR2 register  ******************/
#define EXTI_FTSR2_TR32_Pos        (0U)                                        
#define EXTI_FTSR2_TR32_Msk        (0x1U << EXTI_FTSR2_TR32_Pos)               /*!< 0x00000001 */
#define EXTI_FTSR2_TR32            EXTI_FTSR2_TR32_Msk                         /*!< Falling trigger event configuration bit of line 32 */

/* References Defines */
#define EXTI_FTSR2_FT32 EXTI_FTSR2_TR32
#if defined(EXTI_FTSR2_TR33)
#define EXTI_FTSR2_FT33 EXTI_FTSR2_TR33
#endif
#if defined(EXTI_FTSR2_TR34)
#define EXTI_FTSR2_FT34 EXTI_FTSR2_TR34
#endif
#if defined(EXTI_FTSR2_TR35)
#define EXTI_FTSR2_FT35 EXTI_FTSR2_TR35
#endif

/******************  Bit definition for EXTI_SWIER2 register  *****************/
#define EXTI_SWIER2_SWIER32_Pos    (0U)                                        
#define EXTI_SWIER2_SWIER32_Msk    (0x1U << EXTI_SWIER2_SWIER32_Pos)           /*!< 0x00000001 */
#define EXTI_SWIER2_SWIER32        EXTI_SWIER2_SWIER32_Msk                     /*!< Software Interrupt on line 32 */

/* References Defines */
#define EXTI_SWIER2_SWI32 EXTI_SWIER2_SWIER32
#if defined(EXTI_SWIER2_SWIER33)
#define EXTI_SWIER2_SWI33 EXTI_SWIER2_SWIER33
#endif
#if defined(EXTI_SWIER2_SWIER34)
#define EXTI_SWIER2_SWI34 EXTI_SWIER2_SWIER34
#endif
#if defined(EXTI_SWIER2_SWIER35)
#define EXTI_SWIER2_SWI35 EXTI_SWIER2_SWIER35
#endif

/*******************  Bit definition for EXTI_PR2 register  *******************/
#define EXTI_PR2_PR32_Pos          (0U)                                        
#define EXTI_PR2_PR32_Msk          (0x1U << EXTI_PR2_PR32_Pos)                 /*!< 0x00000001 */
#define EXTI_PR2_PR32              EXTI_PR2_PR32_Msk                           /*!< Pending bit for line 32 */

/* References Defines */
#define EXTI_PR2_PIF32 EXTI_PR2_PR32
#if defined(EXTI_PR2_PR33)
#define EXTI_PR2_PIF33 EXTI_PR2_PR33
#endif
#if defined(EXTI_PR2_PR34)
#define EXTI_PR2_PIF34 EXTI_PR2_PR34
#endif
#if defined(EXTI_PR2_PR35)
#define EXTI_PR2_PIF35 EXTI_PR2_PR35
#endif


/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for FLASH_ACR register  ******************/
#define FLASH_ACR_LATENCY_Pos                (0U)                              
#define FLASH_ACR_LATENCY_Msk                (0x7U << FLASH_ACR_LATENCY_Pos)   /*!< 0x00000007 */
#define FLASH_ACR_LATENCY                    FLASH_ACR_LATENCY_Msk             /*!< LATENCY[2:0] bits (Latency) */
#define FLASH_ACR_LATENCY_0                  (0x1U << FLASH_ACR_LATENCY_Pos)   /*!< 0x00000001 */
#define FLASH_ACR_LATENCY_1                  (0x2U << FLASH_ACR_LATENCY_Pos)   /*!< 0x00000002 */
#define FLASH_ACR_LATENCY_2                  (0x4U << FLASH_ACR_LATENCY_Pos)   /*!< 0x00000004 */

#define FLASH_ACR_HLFCYA_Pos                 (3U)                              
#define FLASH_ACR_HLFCYA_Msk                 (0x1U << FLASH_ACR_HLFCYA_Pos)    /*!< 0x00000008 */
#define FLASH_ACR_HLFCYA                     FLASH_ACR_HLFCYA_Msk              /*!< Flash Half Cycle Access Enable */
#define FLASH_ACR_PRFTBE_Pos                 (4U)                              
#define FLASH_ACR_PRFTBE_Msk                 (0x1U << FLASH_ACR_PRFTBE_Pos)    /*!< 0x00000010 */
#define FLASH_ACR_PRFTBE                     FLASH_ACR_PRFTBE_Msk              /*!< Prefetch Buffer Enable */
#define FLASH_ACR_PRFTBS_Pos                 (5U)                              
#define FLASH_ACR_PRFTBS_Msk                 (0x1U << FLASH_ACR_PRFTBS_Pos)    /*!< 0x00000020 */
#define FLASH_ACR_PRFTBS                     FLASH_ACR_PRFTBS_Msk              /*!< Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define FLASH_KEYR_FKEYR_Pos                 (0U)                              
#define FLASH_KEYR_FKEYR_Msk                 (0xFFFFFFFFU << FLASH_KEYR_FKEYR_Pos) /*!< 0xFFFFFFFF */
#define FLASH_KEYR_FKEYR                     FLASH_KEYR_FKEYR_Msk              /*!< FPEC Key */

#define RDP_KEY_Pos    (0U)                                                    
#define RDP_KEY_Msk    (0xA5U << RDP_KEY_Pos)                                  /*!< 0x000000A5 */
#define RDP_KEY        RDP_KEY_Msk                                             /*!< RDP Key */
#define FLASH_KEY1_Pos                       (0U)                              
#define FLASH_KEY1_Msk                       (0x45670123U << FLASH_KEY1_Pos)   /*!< 0x45670123 */
#define FLASH_KEY1                           FLASH_KEY1_Msk                    /*!< FPEC Key1 */
#define FLASH_KEY2_Pos                       (0U)                              
#define FLASH_KEY2_Msk                       (0xCDEF89ABU << FLASH_KEY2_Pos)   /*!< 0xCDEF89AB */
#define FLASH_KEY2                           FLASH_KEY2_Msk                    /*!< FPEC Key2 */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define FLASH_OPTKEYR_OPTKEYR_Pos            (0U)                              
#define FLASH_OPTKEYR_OPTKEYR_Msk            (0xFFFFFFFFU << FLASH_OPTKEYR_OPTKEYR_Pos) /*!< 0xFFFFFFFF */
#define FLASH_OPTKEYR_OPTKEYR                FLASH_OPTKEYR_OPTKEYR_Msk         /*!< Option Byte Key */

#define  FLASH_OPTKEY1                       FLASH_KEY1                    /*!< Option Byte Key1 */
#define  FLASH_OPTKEY2                       FLASH_KEY2                    /*!< Option Byte Key2 */

/******************  Bit definition for FLASH_SR register  *******************/
#define FLASH_SR_BSY_Pos                     (0U)                              
#define FLASH_SR_BSY_Msk                     (0x1U << FLASH_SR_BSY_Pos)        /*!< 0x00000001 */
#define FLASH_SR_BSY                         FLASH_SR_BSY_Msk                  /*!< Busy */
#define FLASH_SR_PGERR_Pos                   (2U)                              
#define FLASH_SR_PGERR_Msk                   (0x1U << FLASH_SR_PGERR_Pos)      /*!< 0x00000004 */
#define FLASH_SR_PGERR                       FLASH_SR_PGERR_Msk                /*!< Programming Error */
#define FLASH_SR_WRPERR_Pos                  (4U)                              
#define FLASH_SR_WRPERR_Msk                  (0x1U << FLASH_SR_WRPERR_Pos)     /*!< 0x00000010 */
#define FLASH_SR_WRPERR                      FLASH_SR_WRPERR_Msk               /*!< Write Protection Error */
#define FLASH_SR_EOP_Pos                     (5U)                              
#define FLASH_SR_EOP_Msk                     (0x1U << FLASH_SR_EOP_Pos)        /*!< 0x00000020 */
#define FLASH_SR_EOP                         FLASH_SR_EOP_Msk                  /*!< End of operation */

/*******************  Bit definition for FLASH_CR register  *******************/
#define FLASH_CR_PG_Pos                      (0U)                              
#define FLASH_CR_PG_Msk                      (0x1U << FLASH_CR_PG_Pos)         /*!< 0x00000001 */
#define FLASH_CR_PG                          FLASH_CR_PG_Msk                   /*!< Programming */
#define FLASH_CR_PER_Pos                     (1U)                              
#define FLASH_CR_PER_Msk                     (0x1U << FLASH_CR_PER_Pos)        /*!< 0x00000002 */
#define FLASH_CR_PER                         FLASH_CR_PER_Msk                  /*!< Page Erase */
#define FLASH_CR_MER_Pos                     (2U)                              
#define FLASH_CR_MER_Msk                     (0x1U << FLASH_CR_MER_Pos)        /*!< 0x00000004 */
#define FLASH_CR_MER                         FLASH_CR_MER_Msk                  /*!< Mass Erase */
#define FLASH_CR_OPTPG_Pos                   (4U)                              
#define FLASH_CR_OPTPG_Msk                   (0x1U << FLASH_CR_OPTPG_Pos)      /*!< 0x00000010 */
#define FLASH_CR_OPTPG                       FLASH_CR_OPTPG_Msk                /*!< Option Byte Programming */
#define FLASH_CR_OPTER_Pos                   (5U)                              
#define FLASH_CR_OPTER_Msk                   (0x1U << FLASH_CR_OPTER_Pos)      /*!< 0x00000020 */
#define FLASH_CR_OPTER                       FLASH_CR_OPTER_Msk                /*!< Option Byte Erase */
#define FLASH_CR_STRT_Pos                    (6U)                              
#define FLASH_CR_STRT_Msk                    (0x1U << FLASH_CR_STRT_Pos)       /*!< 0x00000040 */
#define FLASH_CR_STRT                        FLASH_CR_STRT_Msk                 /*!< Start */
#define FLASH_CR_LOCK_Pos                    (7U)                              
#define FLASH_CR_LOCK_Msk                    (0x1U << FLASH_CR_LOCK_Pos)       /*!< 0x00000080 */
#define FLASH_CR_LOCK                        FLASH_CR_LOCK_Msk                 /*!< Lock */
#define FLASH_CR_OPTWRE_Pos                  (9U)                              
#define FLASH_CR_OPTWRE_Msk                  (0x1U << FLASH_CR_OPTWRE_Pos)     /*!< 0x00000200 */
#define FLASH_CR_OPTWRE                      FLASH_CR_OPTWRE_Msk               /*!< Option Bytes Write Enable */
#define FLASH_CR_ERRIE_Pos                   (10U)                             
#define FLASH_CR_ERRIE_Msk                   (0x1U << FLASH_CR_ERRIE_Pos)      /*!< 0x00000400 */
#define FLASH_CR_ERRIE                       FLASH_CR_ERRIE_Msk                /*!< Error Interrupt Enable */
#define FLASH_CR_EOPIE_Pos                   (12U)                             
#define FLASH_CR_EOPIE_Msk                   (0x1U << FLASH_CR_EOPIE_Pos)      /*!< 0x00001000 */
#define FLASH_CR_EOPIE                       FLASH_CR_EOPIE_Msk                /*!< End of operation interrupt enable */
#define FLASH_CR_OBL_LAUNCH_Pos              (13U)                             
#define FLASH_CR_OBL_LAUNCH_Msk              (0x1U << FLASH_CR_OBL_LAUNCH_Pos) /*!< 0x00002000 */
#define FLASH_CR_OBL_LAUNCH                  FLASH_CR_OBL_LAUNCH_Msk           /*!< OptionBytes Loader Launch */

/*******************  Bit definition for FLASH_AR register  *******************/
#define FLASH_AR_FAR_Pos                     (0U)                              
#define FLASH_AR_FAR_Msk                     (0xFFFFFFFFU << FLASH_AR_FAR_Pos) /*!< 0xFFFFFFFF */
#define FLASH_AR_FAR                         FLASH_AR_FAR_Msk                  /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define FLASH_OBR_OPTERR_Pos                 (0U)                              
#define FLASH_OBR_OPTERR_Msk                 (0x1U << FLASH_OBR_OPTERR_Pos)    /*!< 0x00000001 */
#define FLASH_OBR_OPTERR                     FLASH_OBR_OPTERR_Msk              /*!< Option Byte Error */
#define FLASH_OBR_RDPRT_Pos                  (1U)                              
#define FLASH_OBR_RDPRT_Msk                  (0x3U << FLASH_OBR_RDPRT_Pos)     /*!< 0x00000006 */
#define FLASH_OBR_RDPRT                      FLASH_OBR_RDPRT_Msk               /*!< Read protection */
#define FLASH_OBR_RDPRT_1                    (0x1U << FLASH_OBR_RDPRT_Pos)     /*!< 0x00000002 */
#define FLASH_OBR_RDPRT_2                    (0x3U << FLASH_OBR_RDPRT_Pos)     /*!< 0x00000006 */

#define FLASH_OBR_USER_Pos                   (8U)                              
#define FLASH_OBR_USER_Msk                   (0x77U << FLASH_OBR_USER_Pos)     /*!< 0x00007700 */
#define FLASH_OBR_USER                       FLASH_OBR_USER_Msk                /*!< User Option Bytes */
#define FLASH_OBR_IWDG_SW_Pos                (8U)                              
#define FLASH_OBR_IWDG_SW_Msk                (0x1U << FLASH_OBR_IWDG_SW_Pos)   /*!< 0x00000100 */
#define FLASH_OBR_IWDG_SW                    FLASH_OBR_IWDG_SW_Msk             /*!< IWDG SW */
#define FLASH_OBR_nRST_STOP_Pos              (9U)                              
#define FLASH_OBR_nRST_STOP_Msk              (0x1U << FLASH_OBR_nRST_STOP_Pos) /*!< 0x00000200 */
#define FLASH_OBR_nRST_STOP                  FLASH_OBR_nRST_STOP_Msk           /*!< nRST_STOP */
#define FLASH_OBR_nRST_STDBY_Pos             (10U)                             
#define FLASH_OBR_nRST_STDBY_Msk             (0x1U << FLASH_OBR_nRST_STDBY_Pos) /*!< 0x00000400 */
#define FLASH_OBR_nRST_STDBY                 FLASH_OBR_nRST_STDBY_Msk          /*!< nRST_STDBY */
#define FLASH_OBR_nBOOT1_Pos                 (12U)                             
#define FLASH_OBR_nBOOT1_Msk                 (0x1U << FLASH_OBR_nBOOT1_Pos)    /*!< 0x00001000 */
#define FLASH_OBR_nBOOT1                     FLASH_OBR_nBOOT1_Msk              /*!< nBOOT1 */
#define FLASH_OBR_VDDA_MONITOR_Pos           (13U)                             
#define FLASH_OBR_VDDA_MONITOR_Msk           (0x1U << FLASH_OBR_VDDA_MONITOR_Pos) /*!< 0x00002000 */
#define FLASH_OBR_VDDA_MONITOR               FLASH_OBR_VDDA_MONITOR_Msk        /*!< VDDA_MONITOR */
#define FLASH_OBR_SRAM_PE_Pos                (14U)                             
#define FLASH_OBR_SRAM_PE_Msk                (0x1U << FLASH_OBR_SRAM_PE_Pos)   /*!< 0x00004000 */
#define FLASH_OBR_SRAM_PE                    FLASH_OBR_SRAM_PE_Msk             /*!< SRAM_PE */
#define FLASH_OBR_DATA0_Pos                  (16U)                             
#define FLASH_OBR_DATA0_Msk                  (0xFFU << FLASH_OBR_DATA0_Pos)    /*!< 0x00FF0000 */
#define FLASH_OBR_DATA0                      FLASH_OBR_DATA0_Msk               /*!< Data0 */
#define FLASH_OBR_DATA1_Pos                  (24U)                             
#define FLASH_OBR_DATA1_Msk                  (0xFFU << FLASH_OBR_DATA1_Pos)    /*!< 0xFF000000 */
#define FLASH_OBR_DATA1                      FLASH_OBR_DATA1_Msk               /*!< Data1 */

/* Legacy defines */
#define FLASH_OBR_WDG_SW FLASH_OBR_IWDG_SW

/******************  Bit definition for FLASH_WRPR register  ******************/
#define FLASH_WRPR_WRP_Pos                   (0U)                              
#define FLASH_WRPR_WRP_Msk                   (0xFFFFFFFFU << FLASH_WRPR_WRP_Pos) /*!< 0xFFFFFFFF */
#define FLASH_WRPR_WRP                       FLASH_WRPR_WRP_Msk                /*!< Write Protect */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for OB_RDP register  **********************/
#define OB_RDP_RDP_Pos       (0U)                                              
#define OB_RDP_RDP_Msk       (0xFFU << OB_RDP_RDP_Pos)                         /*!< 0x000000FF */
#define OB_RDP_RDP           OB_RDP_RDP_Msk                                    /*!< Read protection option byte */
#define OB_RDP_nRDP_Pos      (8U)                                              
#define OB_RDP_nRDP_Msk      (0xFFU << OB_RDP_nRDP_Pos)                        /*!< 0x0000FF00 */
#define OB_RDP_nRDP          OB_RDP_nRDP_Msk                                   /*!< Read protection complemented option byte */

/******************  Bit definition for OB_USER register  *********************/
#define OB_USER_USER_Pos     (16U)                                             
#define OB_USER_USER_Msk     (0xFFU << OB_USER_USER_Pos)                       /*!< 0x00FF0000 */
#define OB_USER_USER         OB_USER_USER_Msk                                  /*!< User option byte */
#define OB_USER_nUSER_Pos    (24U)                                             
#define OB_USER_nUSER_Msk    (0xFFU << OB_USER_nUSER_Pos)                      /*!< 0xFF000000 */
#define OB_USER_nUSER        OB_USER_nUSER_Msk                                 /*!< User complemented option byte */

/******************  Bit definition for FLASH_WRP0 register  ******************/
#define OB_WRP0_WRP0_Pos     (0U)                                              
#define OB_WRP0_WRP0_Msk     (0xFFU << OB_WRP0_WRP0_Pos)                       /*!< 0x000000FF */
#define OB_WRP0_WRP0         OB_WRP0_WRP0_Msk                                  /*!< Flash memory write protection option bytes */
#define OB_WRP0_nWRP0_Pos    (8U)                                              
#define OB_WRP0_nWRP0_Msk    (0xFFU << OB_WRP0_nWRP0_Pos)                      /*!< 0x0000FF00 */
#define OB_WRP0_nWRP0        OB_WRP0_nWRP0_Msk                                 /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP1 register  ******************/
#define OB_WRP1_WRP1_Pos     (16U)                                             
#define OB_WRP1_WRP1_Msk     (0xFFU << OB_WRP1_WRP1_Pos)                       /*!< 0x00FF0000 */
#define OB_WRP1_WRP1         OB_WRP1_WRP1_Msk                                  /*!< Flash memory write protection option bytes */
#define OB_WRP1_nWRP1_Pos    (24U)                                             
#define OB_WRP1_nWRP1_Msk    (0xFFU << OB_WRP1_nWRP1_Pos)                      /*!< 0xFF000000 */
#define OB_WRP1_nWRP1        OB_WRP1_nWRP1_Msk                                 /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP2 register  ******************/
#define OB_WRP2_WRP2_Pos     (0U)                                              
#define OB_WRP2_WRP2_Msk     (0xFFU << OB_WRP2_WRP2_Pos)                       /*!< 0x000000FF */
#define OB_WRP2_WRP2         OB_WRP2_WRP2_Msk                                  /*!< Flash memory write protection option bytes */
#define OB_WRP2_nWRP2_Pos    (8U)                                              
#define OB_WRP2_nWRP2_Msk    (0xFFU << OB_WRP2_nWRP2_Pos)                      /*!< 0x0000FF00 */
#define OB_WRP2_nWRP2        OB_WRP2_nWRP2_Msk                                 /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP3 register  ******************/
#define OB_WRP3_WRP3_Pos     (16U)                                             
#define OB_WRP3_WRP3_Msk     (0xFFU << OB_WRP3_WRP3_Pos)                       /*!< 0x00FF0000 */
#define OB_WRP3_WRP3         OB_WRP3_WRP3_Msk                                  /*!< Flash memory write protection option bytes */
#define OB_WRP3_nWRP3_Pos    (24U)                                             
#define OB_WRP3_nWRP3_Msk    (0xFFU << OB_WRP3_nWRP3_Pos)                      /*!< 0xFF000000 */
#define OB_WRP3_nWRP3        OB_WRP3_nWRP3_Msk                                 /*!< Flash memory write protection complemented option bytes */

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O (GPIO)                      */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for GPIO_MODER register  *****************/
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

/******************  Bit definition for GPIO_OTYPER register  *****************/
#define GPIO_OTYPER_OT_0                 (0x00000001U)                         
#define GPIO_OTYPER_OT_1                 (0x00000002U)                         
#define GPIO_OTYPER_OT_2                 (0x00000004U)                         
#define GPIO_OTYPER_OT_3                 (0x00000008U)                         
#define GPIO_OTYPER_OT_4                 (0x00000010U)                         
#define GPIO_OTYPER_OT_5                 (0x00000020U)                         
#define GPIO_OTYPER_OT_6                 (0x00000040U)                         
#define GPIO_OTYPER_OT_7                 (0x00000080U)                         
#define GPIO_OTYPER_OT_8                 (0x00000100U)                         
#define GPIO_OTYPER_OT_9                 (0x00000200U)                         
#define GPIO_OTYPER_OT_10                (0x00000400U)                         
#define GPIO_OTYPER_OT_11                (0x00000800U)                         
#define GPIO_OTYPER_OT_12                (0x00001000U)                         
#define GPIO_OTYPER_OT_13                (0x00002000U)                         
#define GPIO_OTYPER_OT_14                (0x00004000U)                         
#define GPIO_OTYPER_OT_15                (0x00008000U)                         

/****************  Bit definition for GPIO_OSPEEDR register  ******************/
#define GPIO_OSPEEDER_OSPEEDR0_Pos       (0U)                                  
#define GPIO_OSPEEDER_OSPEEDR0_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR0_Pos)  /*!< 0x00000003 */
#define GPIO_OSPEEDER_OSPEEDR0           GPIO_OSPEEDER_OSPEEDR0_Msk            
#define GPIO_OSPEEDER_OSPEEDR0_0         (0x1U << GPIO_OSPEEDER_OSPEEDR0_Pos)  /*!< 0x00000001 */
#define GPIO_OSPEEDER_OSPEEDR0_1         (0x2U << GPIO_OSPEEDER_OSPEEDR0_Pos)  /*!< 0x00000002 */
#define GPIO_OSPEEDER_OSPEEDR1_Pos       (2U)                                  
#define GPIO_OSPEEDER_OSPEEDR1_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR1_Pos)  /*!< 0x0000000C */
#define GPIO_OSPEEDER_OSPEEDR1           GPIO_OSPEEDER_OSPEEDR1_Msk            
#define GPIO_OSPEEDER_OSPEEDR1_0         (0x1U << GPIO_OSPEEDER_OSPEEDR1_Pos)  /*!< 0x00000004 */
#define GPIO_OSPEEDER_OSPEEDR1_1         (0x2U << GPIO_OSPEEDER_OSPEEDR1_Pos)  /*!< 0x00000008 */
#define GPIO_OSPEEDER_OSPEEDR2_Pos       (4U)                                  
#define GPIO_OSPEEDER_OSPEEDR2_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR2_Pos)  /*!< 0x00000030 */
#define GPIO_OSPEEDER_OSPEEDR2           GPIO_OSPEEDER_OSPEEDR2_Msk            
#define GPIO_OSPEEDER_OSPEEDR2_0         (0x1U << GPIO_OSPEEDER_OSPEEDR2_Pos)  /*!< 0x00000010 */
#define GPIO_OSPEEDER_OSPEEDR2_1         (0x2U << GPIO_OSPEEDER_OSPEEDR2_Pos)  /*!< 0x00000020 */
#define GPIO_OSPEEDER_OSPEEDR3_Pos       (6U)                                  
#define GPIO_OSPEEDER_OSPEEDR3_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR3_Pos)  /*!< 0x000000C0 */
#define GPIO_OSPEEDER_OSPEEDR3           GPIO_OSPEEDER_OSPEEDR3_Msk            
#define GPIO_OSPEEDER_OSPEEDR3_0         (0x1U << GPIO_OSPEEDER_OSPEEDR3_Pos)  /*!< 0x00000040 */
#define GPIO_OSPEEDER_OSPEEDR3_1         (0x2U << GPIO_OSPEEDER_OSPEEDR3_Pos)  /*!< 0x00000080 */
#define GPIO_OSPEEDER_OSPEEDR4_Pos       (8U)                                  
#define GPIO_OSPEEDER_OSPEEDR4_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR4_Pos)  /*!< 0x00000300 */
#define GPIO_OSPEEDER_OSPEEDR4           GPIO_OSPEEDER_OSPEEDR4_Msk            
#define GPIO_OSPEEDER_OSPEEDR4_0         (0x1U << GPIO_OSPEEDER_OSPEEDR4_Pos)  /*!< 0x00000100 */
#define GPIO_OSPEEDER_OSPEEDR4_1         (0x2U << GPIO_OSPEEDER_OSPEEDR4_Pos)  /*!< 0x00000200 */
#define GPIO_OSPEEDER_OSPEEDR5_Pos       (10U)                                 
#define GPIO_OSPEEDER_OSPEEDR5_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR5_Pos)  /*!< 0x00000C00 */
#define GPIO_OSPEEDER_OSPEEDR5           GPIO_OSPEEDER_OSPEEDR5_Msk            
#define GPIO_OSPEEDER_OSPEEDR5_0         (0x1U << GPIO_OSPEEDER_OSPEEDR5_Pos)  /*!< 0x00000400 */
#define GPIO_OSPEEDER_OSPEEDR5_1         (0x2U << GPIO_OSPEEDER_OSPEEDR5_Pos)  /*!< 0x00000800 */
#define GPIO_OSPEEDER_OSPEEDR6_Pos       (12U)                                 
#define GPIO_OSPEEDER_OSPEEDR6_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR6_Pos)  /*!< 0x00003000 */
#define GPIO_OSPEEDER_OSPEEDR6           GPIO_OSPEEDER_OSPEEDR6_Msk            
#define GPIO_OSPEEDER_OSPEEDR6_0         (0x1U << GPIO_OSPEEDER_OSPEEDR6_Pos)  /*!< 0x00001000 */
#define GPIO_OSPEEDER_OSPEEDR6_1         (0x2U << GPIO_OSPEEDER_OSPEEDR6_Pos)  /*!< 0x00002000 */
#define GPIO_OSPEEDER_OSPEEDR7_Pos       (14U)                                 
#define GPIO_OSPEEDER_OSPEEDR7_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR7_Pos)  /*!< 0x0000C000 */
#define GPIO_OSPEEDER_OSPEEDR7           GPIO_OSPEEDER_OSPEEDR7_Msk            
#define GPIO_OSPEEDER_OSPEEDR7_0         (0x1U << GPIO_OSPEEDER_OSPEEDR7_Pos)  /*!< 0x00004000 */
#define GPIO_OSPEEDER_OSPEEDR7_1         (0x2U << GPIO_OSPEEDER_OSPEEDR7_Pos)  /*!< 0x00008000 */
#define GPIO_OSPEEDER_OSPEEDR8_Pos       (16U)                                 
#define GPIO_OSPEEDER_OSPEEDR8_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR8_Pos)  /*!< 0x00030000 */
#define GPIO_OSPEEDER_OSPEEDR8           GPIO_OSPEEDER_OSPEEDR8_Msk            
#define GPIO_OSPEEDER_OSPEEDR8_0         (0x1U << GPIO_OSPEEDER_OSPEEDR8_Pos)  /*!< 0x00010000 */
#define GPIO_OSPEEDER_OSPEEDR8_1         (0x2U << GPIO_OSPEEDER_OSPEEDR8_Pos)  /*!< 0x00020000 */
#define GPIO_OSPEEDER_OSPEEDR9_Pos       (18U)                                 
#define GPIO_OSPEEDER_OSPEEDR9_Msk       (0x3U << GPIO_OSPEEDER_OSPEEDR9_Pos)  /*!< 0x000C0000 */
#define GPIO_OSPEEDER_OSPEEDR9           GPIO_OSPEEDER_OSPEEDR9_Msk            
#define GPIO_OSPEEDER_OSPEEDR9_0         (0x1U << GPIO_OSPEEDER_OSPEEDR9_Pos)  /*!< 0x00040000 */
#define GPIO_OSPEEDER_OSPEEDR9_1         (0x2U << GPIO_OSPEEDER_OSPEEDR9_Pos)  /*!< 0x00080000 */
#define GPIO_OSPEEDER_OSPEEDR10_Pos      (20U)                                 
#define GPIO_OSPEEDER_OSPEEDR10_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR10_Pos) /*!< 0x00300000 */
#define GPIO_OSPEEDER_OSPEEDR10          GPIO_OSPEEDER_OSPEEDR10_Msk           
#define GPIO_OSPEEDER_OSPEEDR10_0        (0x1U << GPIO_OSPEEDER_OSPEEDR10_Pos) /*!< 0x00100000 */
#define GPIO_OSPEEDER_OSPEEDR10_1        (0x2U << GPIO_OSPEEDER_OSPEEDR10_Pos) /*!< 0x00200000 */
#define GPIO_OSPEEDER_OSPEEDR11_Pos      (22U)                                 
#define GPIO_OSPEEDER_OSPEEDR11_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR11_Pos) /*!< 0x00C00000 */
#define GPIO_OSPEEDER_OSPEEDR11          GPIO_OSPEEDER_OSPEEDR11_Msk           
#define GPIO_OSPEEDER_OSPEEDR11_0        (0x1U << GPIO_OSPEEDER_OSPEEDR11_Pos) /*!< 0x00400000 */
#define GPIO_OSPEEDER_OSPEEDR11_1        (0x2U << GPIO_OSPEEDER_OSPEEDR11_Pos) /*!< 0x00800000 */
#define GPIO_OSPEEDER_OSPEEDR12_Pos      (24U)                                 
#define GPIO_OSPEEDER_OSPEEDR12_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR12_Pos) /*!< 0x03000000 */
#define GPIO_OSPEEDER_OSPEEDR12          GPIO_OSPEEDER_OSPEEDR12_Msk           
#define GPIO_OSPEEDER_OSPEEDR12_0        (0x1U << GPIO_OSPEEDER_OSPEEDR12_Pos) /*!< 0x01000000 */
#define GPIO_OSPEEDER_OSPEEDR12_1        (0x2U << GPIO_OSPEEDER_OSPEEDR12_Pos) /*!< 0x02000000 */
#define GPIO_OSPEEDER_OSPEEDR13_Pos      (26U)                                 
#define GPIO_OSPEEDER_OSPEEDR13_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR13_Pos) /*!< 0x0C000000 */
#define GPIO_OSPEEDER_OSPEEDR13          GPIO_OSPEEDER_OSPEEDR13_Msk           
#define GPIO_OSPEEDER_OSPEEDR13_0        (0x1U << GPIO_OSPEEDER_OSPEEDR13_Pos) /*!< 0x04000000 */
#define GPIO_OSPEEDER_OSPEEDR13_1        (0x2U << GPIO_OSPEEDER_OSPEEDR13_Pos) /*!< 0x08000000 */
#define GPIO_OSPEEDER_OSPEEDR14_Pos      (28U)                                 
#define GPIO_OSPEEDER_OSPEEDR14_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR14_Pos) /*!< 0x30000000 */
#define GPIO_OSPEEDER_OSPEEDR14          GPIO_OSPEEDER_OSPEEDR14_Msk           
#define GPIO_OSPEEDER_OSPEEDR14_0        (0x1U << GPIO_OSPEEDER_OSPEEDR14_Pos) /*!< 0x10000000 */
#define GPIO_OSPEEDER_OSPEEDR14_1        (0x2U << GPIO_OSPEEDER_OSPEEDR14_Pos) /*!< 0x20000000 */
#define GPIO_OSPEEDER_OSPEEDR15_Pos      (30U)                                 
#define GPIO_OSPEEDER_OSPEEDR15_Msk      (0x3U << GPIO_OSPEEDER_OSPEEDR15_Pos) /*!< 0xC0000000 */
#define GPIO_OSPEEDER_OSPEEDR15          GPIO_OSPEEDER_OSPEEDR15_Msk           
#define GPIO_OSPEEDER_OSPEEDR15_0        (0x1U << GPIO_OSPEEDER_OSPEEDR15_Pos) /*!< 0x40000000 */
#define GPIO_OSPEEDER_OSPEEDR15_1        (0x2U << GPIO_OSPEEDER_OSPEEDR15_Pos) /*!< 0x80000000 */

/*******************  Bit definition for GPIO_PUPDR register ******************/
#define GPIO_PUPDR_PUPDR0_Pos            (0U)                                  
#define GPIO_PUPDR_PUPDR0_Msk            (0x3U << GPIO_PUPDR_PUPDR0_Pos)       /*!< 0x00000003 */
#define GPIO_PUPDR_PUPDR0                GPIO_PUPDR_PUPDR0_Msk                 
#define GPIO_PUPDR_PUPDR0_0              (0x1U << GPIO_PUPDR_PUPDR0_Pos)       /*!< 0x00000001 */
#define GPIO_PUPDR_PUPDR0_1              (0x2U << GPIO_PUPDR_PUPDR0_Pos)       /*!< 0x00000002 */
#define GPIO_PUPDR_PUPDR1_Pos            (2U)                                  
#define GPIO_PUPDR_PUPDR1_Msk            (0x3U << GPIO_PUPDR_PUPDR1_Pos)       /*!< 0x0000000C */
#define GPIO_PUPDR_PUPDR1                GPIO_PUPDR_PUPDR1_Msk                 
#define GPIO_PUPDR_PUPDR1_0              (0x1U << GPIO_PUPDR_PUPDR1_Pos)       /*!< 0x00000004 */
#define GPIO_PUPDR_PUPDR1_1              (0x2U << GPIO_PUPDR_PUPDR1_Pos)       /*!< 0x00000008 */
#define GPIO_PUPDR_PUPDR2_Pos            (4U)                                  
#define GPIO_PUPDR_PUPDR2_Msk            (0x3U << GPIO_PUPDR_PUPDR2_Pos)       /*!< 0x00000030 */
#define GPIO_PUPDR_PUPDR2                GPIO_PUPDR_PUPDR2_Msk                 
#define GPIO_PUPDR_PUPDR2_0              (0x1U << GPIO_PUPDR_PUPDR2_Pos)       /*!< 0x00000010 */
#define GPIO_PUPDR_PUPDR2_1              (0x2U << GPIO_PUPDR_PUPDR2_Pos)       /*!< 0x00000020 */
#define GPIO_PUPDR_PUPDR3_Pos            (6U)                                  
#define GPIO_PUPDR_PUPDR3_Msk            (0x3U << GPIO_PUPDR_PUPDR3_Pos)       /*!< 0x000000C0 */
#define GPIO_PUPDR_PUPDR3                GPIO_PUPDR_PUPDR3_Msk                 
#define GPIO_PUPDR_PUPDR3_0              (0x1U << GPIO_PUPDR_PUPDR3_Pos)       /*!< 0x00000040 */
#define GPIO_PUPDR_PUPDR3_1              (0x2U << GPIO_PUPDR_PUPDR3_Pos)       /*!< 0x00000080 */
#define GPIO_PUPDR_PUPDR4_Pos            (8U)                                  
#define GPIO_PUPDR_PUPDR4_Msk            (0x3U << GPIO_PUPDR_PUPDR4_Pos)       /*!< 0x00000300 */
#define GPIO_PUPDR_PUPDR4                GPIO_PUPDR_PUPDR4_Msk                 
#define GPIO_PUPDR_PUPDR4_0              (0x1U << GPIO_PUPDR_PUPDR4_Pos)       /*!< 0x00000100 */
#define GPIO_PUPDR_PUPDR4_1              (0x2U << GPIO_PUPDR_PUPDR4_Pos)       /*!< 0x00000200 */
#define GPIO_PUPDR_PUPDR5_Pos            (10U)                                 
#define GPIO_PUPDR_PUPDR5_Msk            (0x3U << GPIO_PUPDR_PUPDR5_Pos)       /*!< 0x00000C00 */
#define GPIO_PUPDR_PUPDR5                GPIO_PUPDR_PUPDR5_Msk                 
#define GPIO_PUPDR_PUPDR5_0              (0x1U << GPIO_PUPDR_PUPDR5_Pos)       /*!< 0x00000400 */
#define GPIO_PUPDR_PUPDR5_1              (0x2U << GPIO_PUPDR_PUPDR5_Pos)       /*!< 0x00000800 */
#define GPIO_PUPDR_PUPDR6_Pos            (12U)                                 
#define GPIO_PUPDR_PUPDR6_Msk            (0x3U << GPIO_PUPDR_PUPDR6_Pos)       /*!< 0x00003000 */
#define GPIO_PUPDR_PUPDR6                GPIO_PUPDR_PUPDR6_Msk                 
#define GPIO_PUPDR_PUPDR6_0              (0x1U << GPIO_PUPDR_PUPDR6_Pos)       /*!< 0x00001000 */
#define GPIO_PUPDR_PUPDR6_1              (0x2U << GPIO_PUPDR_PUPDR6_Pos)       /*!< 0x00002000 */
#define GPIO_PUPDR_PUPDR7_Pos            (14U)                                 
#define GPIO_PUPDR_PUPDR7_Msk            (0x3U << GPIO_PUPDR_PUPDR7_Pos)       /*!< 0x0000C000 */
#define GPIO_PUPDR_PUPDR7                GPIO_PUPDR_PUPDR7_Msk                 
#define GPIO_PUPDR_PUPDR7_0              (0x1U << GPIO_PUPDR_PUPDR7_Pos)       /*!< 0x00004000 */
#define GPIO_PUPDR_PUPDR7_1              (0x2U << GPIO_PUPDR_PUPDR7_Pos)       /*!< 0x00008000 */
#define GPIO_PUPDR_PUPDR8_Pos            (16U)                                 
#define GPIO_PUPDR_PUPDR8_Msk            (0x3U << GPIO_PUPDR_PUPDR8_Pos)       /*!< 0x00030000 */
#define GPIO_PUPDR_PUPDR8                GPIO_PUPDR_PUPDR8_Msk                 
#define GPIO_PUPDR_PUPDR8_0              (0x1U << GPIO_PUPDR_PUPDR8_Pos)       /*!< 0x00010000 */
#define GPIO_PUPDR_PUPDR8_1              (0x2U << GPIO_PUPDR_PUPDR8_Pos)       /*!< 0x00020000 */
#define GPIO_PUPDR_PUPDR9_Pos            (18U)                                 
#define GPIO_PUPDR_PUPDR9_Msk            (0x3U << GPIO_PUPDR_PUPDR9_Pos)       /*!< 0x000C0000 */
#define GPIO_PUPDR_PUPDR9                GPIO_PUPDR_PUPDR9_Msk                 
#define GPIO_PUPDR_PUPDR9_0              (0x1U << GPIO_PUPDR_PUPDR9_Pos)       /*!< 0x00040000 */
#define GPIO_PUPDR_PUPDR9_1              (0x2U << GPIO_PUPDR_PUPDR9_Pos)       /*!< 0x00080000 */
#define GPIO_PUPDR_PUPDR10_Pos           (20U)                                 
#define GPIO_PUPDR_PUPDR10_Msk           (0x3U << GPIO_PUPDR_PUPDR10_Pos)      /*!< 0x00300000 */
#define GPIO_PUPDR_PUPDR10               GPIO_PUPDR_PUPDR10_Msk                
#define GPIO_PUPDR_PUPDR10_0             (0x1U << GPIO_PUPDR_PUPDR10_Pos)      /*!< 0x00100000 */
#define GPIO_PUPDR_PUPDR10_1             (0x2U << GPIO_PUPDR_PUPDR10_Pos)      /*!< 0x00200000 */
#define GPIO_PUPDR_PUPDR11_Pos           (22U)                                 
#define GPIO_PUPDR_PUPDR11_Msk           (0x3U << GPIO_PUPDR_PUPDR11_Pos)      /*!< 0x00C00000 */
#define GPIO_PUPDR_PUPDR11               GPIO_PUPDR_PUPDR11_Msk                
#define GPIO_PUPDR_PUPDR11_0             (0x1U << GPIO_PUPDR_PUPDR11_Pos)      /*!< 0x00400000 */
#define GPIO_PUPDR_PUPDR11_1             (0x2U << GPIO_PUPDR_PUPDR11_Pos)      /*!< 0x00800000 */
#define GPIO_PUPDR_PUPDR12_Pos           (24U)                                 
#define GPIO_PUPDR_PUPDR12_Msk           (0x3U << GPIO_PUPDR_PUPDR12_Pos)      /*!< 0x03000000 */
#define GPIO_PUPDR_PUPDR12               GPIO_PUPDR_PUPDR12_Msk                
#define GPIO_PUPDR_PUPDR12_0             (0x1U << GPIO_PUPDR_PUPDR12_Pos)      /*!< 0x01000000 */
#define GPIO_PUPDR_PUPDR12_1             (0x2U << GPIO_PUPDR_PUPDR12_Pos)      /*!< 0x02000000 */
#define GPIO_PUPDR_PUPDR13_Pos           (26U)                                 
#define GPIO_PUPDR_PUPDR13_Msk           (0x3U << GPIO_PUPDR_PUPDR13_Pos)      /*!< 0x0C000000 */
#define GPIO_PUPDR_PUPDR13               GPIO_PUPDR_PUPDR13_Msk                
#define GPIO_PUPDR_PUPDR13_0             (0x1U << GPIO_PUPDR_PUPDR13_Pos)      /*!< 0x04000000 */
#define GPIO_PUPDR_PUPDR13_1             (0x2U << GPIO_PUPDR_PUPDR13_Pos)      /*!< 0x08000000 */
#define GPIO_PUPDR_PUPDR14_Pos           (28U)                                 
#define GPIO_PUPDR_PUPDR14_Msk           (0x3U << GPIO_PUPDR_PUPDR14_Pos)      /*!< 0x30000000 */
#define GPIO_PUPDR_PUPDR14               GPIO_PUPDR_PUPDR14_Msk                
#define GPIO_PUPDR_PUPDR14_0             (0x1U << GPIO_PUPDR_PUPDR14_Pos)      /*!< 0x10000000 */
#define GPIO_PUPDR_PUPDR14_1             (0x2U << GPIO_PUPDR_PUPDR14_Pos)      /*!< 0x20000000 */
#define GPIO_PUPDR_PUPDR15_Pos           (30U)                                 
#define GPIO_PUPDR_PUPDR15_Msk           (0x3U << GPIO_PUPDR_PUPDR15_Pos)      /*!< 0xC0000000 */
#define GPIO_PUPDR_PUPDR15               GPIO_PUPDR_PUPDR15_Msk                
#define GPIO_PUPDR_PUPDR15_0             (0x1U << GPIO_PUPDR_PUPDR15_Pos)      /*!< 0x40000000 */
#define GPIO_PUPDR_PUPDR15_1             (0x2U << GPIO_PUPDR_PUPDR15_Pos)      /*!< 0x80000000 */

/*******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_0                       (0x00000001U)                         
#define GPIO_IDR_1                       (0x00000002U)                         
#define GPIO_IDR_2                       (0x00000004U)                         
#define GPIO_IDR_3                       (0x00000008U)                         
#define GPIO_IDR_4                       (0x00000010U)                         
#define GPIO_IDR_5                       (0x00000020U)                         
#define GPIO_IDR_6                       (0x00000040U)                         
#define GPIO_IDR_7                       (0x00000080U)                         
#define GPIO_IDR_8                       (0x00000100U)                         
#define GPIO_IDR_9                       (0x00000200U)                         
#define GPIO_IDR_10                      (0x00000400U)                         
#define GPIO_IDR_11                      (0x00000800U)                         
#define GPIO_IDR_12                      (0x00001000U)                         
#define GPIO_IDR_13                      (0x00002000U)                         
#define GPIO_IDR_14                      (0x00004000U)                         
#define GPIO_IDR_15                      (0x00008000U)                         

/******************  Bit definition for GPIO_ODR register  ********************/
#define GPIO_ODR_0                       (0x00000001U)                         
#define GPIO_ODR_1                       (0x00000002U)                         
#define GPIO_ODR_2                       (0x00000004U)                         
#define GPIO_ODR_3                       (0x00000008U)                         
#define GPIO_ODR_4                       (0x00000010U)                         
#define GPIO_ODR_5                       (0x00000020U)                         
#define GPIO_ODR_6                       (0x00000040U)                         
#define GPIO_ODR_7                       (0x00000080U)                         
#define GPIO_ODR_8                       (0x00000100U)                         
#define GPIO_ODR_9                       (0x00000200U)                         
#define GPIO_ODR_10                      (0x00000400U)                         
#define GPIO_ODR_11                      (0x00000800U)                         
#define GPIO_ODR_12                      (0x00001000U)                         
#define GPIO_ODR_13                      (0x00002000U)                         
#define GPIO_ODR_14                      (0x00004000U)                         
#define GPIO_ODR_15                      (0x00008000U)                         

/****************** Bit definition for GPIO_BSRR register  ********************/
#define GPIO_BSRR_BS_0                   (0x00000001U)                         
#define GPIO_BSRR_BS_1                   (0x00000002U)                         
#define GPIO_BSRR_BS_2                   (0x00000004U)                         
#define GPIO_BSRR_BS_3                   (0x00000008U)                         
#define GPIO_BSRR_BS_4                   (0x00000010U)                         
#define GPIO_BSRR_BS_5                   (0x00000020U)                         
#define GPIO_BSRR_BS_6                   (0x00000040U)                         
#define GPIO_BSRR_BS_7                   (0x00000080U)                         
#define GPIO_BSRR_BS_8                   (0x00000100U)                         
#define GPIO_BSRR_BS_9                   (0x00000200U)                         
#define GPIO_BSRR_BS_10                  (0x00000400U)                         
#define GPIO_BSRR_BS_11                  (0x00000800U)                         
#define GPIO_BSRR_BS_12                  (0x00001000U)                         
#define GPIO_BSRR_BS_13                  (0x00002000U)                         
#define GPIO_BSRR_BS_14                  (0x00004000U)                         
#define GPIO_BSRR_BS_15                  (0x00008000U)                         
#define GPIO_BSRR_BR_0                   (0x00010000U)                         
#define GPIO_BSRR_BR_1                   (0x00020000U)                         
#define GPIO_BSRR_BR_2                   (0x00040000U)                         
#define GPIO_BSRR_BR_3                   (0x00080000U)                         
#define GPIO_BSRR_BR_4                   (0x00100000U)                         
#define GPIO_BSRR_BR_5                   (0x00200000U)                         
#define GPIO_BSRR_BR_6                   (0x00400000U)                         
#define GPIO_BSRR_BR_7                   (0x00800000U)                         
#define GPIO_BSRR_BR_8                   (0x01000000U)                         
#define GPIO_BSRR_BR_9                   (0x02000000U)                         
#define GPIO_BSRR_BR_10                  (0x04000000U)                         
#define GPIO_BSRR_BR_11                  (0x08000000U)                         
#define GPIO_BSRR_BR_12                  (0x10000000U)                         
#define GPIO_BSRR_BR_13                  (0x20000000U)                         
#define GPIO_BSRR_BR_14                  (0x40000000U)                         
#define GPIO_BSRR_BR_15                  (0x80000000U)                         

/****************** Bit definition for GPIO_LCKR register  ********************/
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

/****************** Bit definition for GPIO_AFRL register  ********************/
#define GPIO_AFRL_AFRL0_Pos              (0U)                                  
#define GPIO_AFRL_AFRL0_Msk              (0xFU << GPIO_AFRL_AFRL0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRL_AFRL0                  GPIO_AFRL_AFRL0_Msk                   
#define GPIO_AFRL_AFRL1_Pos              (4U)                                  
#define GPIO_AFRL_AFRL1_Msk              (0xFU << GPIO_AFRL_AFRL1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRL_AFRL1                  GPIO_AFRL_AFRL1_Msk                   
#define GPIO_AFRL_AFRL2_Pos              (8U)                                  
#define GPIO_AFRL_AFRL2_Msk              (0xFU << GPIO_AFRL_AFRL2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRL_AFRL2                  GPIO_AFRL_AFRL2_Msk                   
#define GPIO_AFRL_AFRL3_Pos              (12U)                                 
#define GPIO_AFRL_AFRL3_Msk              (0xFU << GPIO_AFRL_AFRL3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRL_AFRL3                  GPIO_AFRL_AFRL3_Msk                   
#define GPIO_AFRL_AFRL4_Pos              (16U)                                 
#define GPIO_AFRL_AFRL4_Msk              (0xFU << GPIO_AFRL_AFRL4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRL_AFRL4                  GPIO_AFRL_AFRL4_Msk                   
#define GPIO_AFRL_AFRL5_Pos              (20U)                                 
#define GPIO_AFRL_AFRL5_Msk              (0xFU << GPIO_AFRL_AFRL5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRL_AFRL5                  GPIO_AFRL_AFRL5_Msk                   
#define GPIO_AFRL_AFRL6_Pos              (24U)                                 
#define GPIO_AFRL_AFRL6_Msk              (0xFU << GPIO_AFRL_AFRL6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRL_AFRL6                  GPIO_AFRL_AFRL6_Msk                   
#define GPIO_AFRL_AFRL7_Pos              (28U)                                 
#define GPIO_AFRL_AFRL7_Msk              (0xFU << GPIO_AFRL_AFRL7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRL_AFRL7                  GPIO_AFRL_AFRL7_Msk                   

/****************** Bit definition for GPIO_AFRH register  ********************/
#define GPIO_AFRH_AFRH0_Pos              (0U)                                  
#define GPIO_AFRH_AFRH0_Msk              (0xFU << GPIO_AFRH_AFRH0_Pos)         /*!< 0x0000000F */
#define GPIO_AFRH_AFRH0                  GPIO_AFRH_AFRH0_Msk                   
#define GPIO_AFRH_AFRH1_Pos              (4U)                                  
#define GPIO_AFRH_AFRH1_Msk              (0xFU << GPIO_AFRH_AFRH1_Pos)         /*!< 0x000000F0 */
#define GPIO_AFRH_AFRH1                  GPIO_AFRH_AFRH1_Msk                   
#define GPIO_AFRH_AFRH2_Pos              (8U)                                  
#define GPIO_AFRH_AFRH2_Msk              (0xFU << GPIO_AFRH_AFRH2_Pos)         /*!< 0x00000F00 */
#define GPIO_AFRH_AFRH2                  GPIO_AFRH_AFRH2_Msk                   
#define GPIO_AFRH_AFRH3_Pos              (12U)                                 
#define GPIO_AFRH_AFRH3_Msk              (0xFU << GPIO_AFRH_AFRH3_Pos)         /*!< 0x0000F000 */
#define GPIO_AFRH_AFRH3                  GPIO_AFRH_AFRH3_Msk                   
#define GPIO_AFRH_AFRH4_Pos              (16U)                                 
#define GPIO_AFRH_AFRH4_Msk              (0xFU << GPIO_AFRH_AFRH4_Pos)         /*!< 0x000F0000 */
#define GPIO_AFRH_AFRH4                  GPIO_AFRH_AFRH4_Msk                   
#define GPIO_AFRH_AFRH5_Pos              (20U)                                 
#define GPIO_AFRH_AFRH5_Msk              (0xFU << GPIO_AFRH_AFRH5_Pos)         /*!< 0x00F00000 */
#define GPIO_AFRH_AFRH5                  GPIO_AFRH_AFRH5_Msk                   
#define GPIO_AFRH_AFRH6_Pos              (24U)                                 
#define GPIO_AFRH_AFRH6_Msk              (0xFU << GPIO_AFRH_AFRH6_Pos)         /*!< 0x0F000000 */
#define GPIO_AFRH_AFRH6                  GPIO_AFRH_AFRH6_Msk                   
#define GPIO_AFRH_AFRH7_Pos              (28U)                                 
#define GPIO_AFRH_AFRH7_Msk              (0xFU << GPIO_AFRH_AFRH7_Pos)         /*!< 0xF0000000 */
#define GPIO_AFRH_AFRH7                  GPIO_AFRH_AFRH7_Msk                   

/****************** Bit definition for GPIO_BRR register  *********************/
#define GPIO_BRR_BR_0                    (0x00000001U)                         
#define GPIO_BRR_BR_1                    (0x00000002U)                         
#define GPIO_BRR_BR_2                    (0x00000004U)                         
#define GPIO_BRR_BR_3                    (0x00000008U)                         
#define GPIO_BRR_BR_4                    (0x00000010U)                         
#define GPIO_BRR_BR_5                    (0x00000020U)                         
#define GPIO_BRR_BR_6                    (0x00000040U)                         
#define GPIO_BRR_BR_7                    (0x00000080U)                         
#define GPIO_BRR_BR_8                    (0x00000100U)                         
#define GPIO_BRR_BR_9                    (0x00000200U)                         
#define GPIO_BRR_BR_10                   (0x00000400U)                         
#define GPIO_BRR_BR_11                   (0x00000800U)                         
#define GPIO_BRR_BR_12                   (0x00001000U)                         
#define GPIO_BRR_BR_13                   (0x00002000U)                         
#define GPIO_BRR_BR_14                   (0x00004000U)                         
#define GPIO_BRR_BR_15                   (0x00008000U)                         

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface (I2C)              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
#define I2C_CR1_PE_Pos               (0U)                                      
#define I2C_CR1_PE_Msk               (0x1U << I2C_CR1_PE_Pos)                  /*!< 0x00000001 */
#define I2C_CR1_PE                   I2C_CR1_PE_Msk                            /*!< Peripheral enable */
#define I2C_CR1_TXIE_Pos             (1U)                                      
#define I2C_CR1_TXIE_Msk             (0x1U << I2C_CR1_TXIE_Pos)                /*!< 0x00000002 */
#define I2C_CR1_TXIE                 I2C_CR1_TXIE_Msk                          /*!< TX interrupt enable */
#define I2C_CR1_RXIE_Pos             (2U)                                      
#define I2C_CR1_RXIE_Msk             (0x1U << I2C_CR1_RXIE_Pos)                /*!< 0x00000004 */
#define I2C_CR1_RXIE                 I2C_CR1_RXIE_Msk                          /*!< RX interrupt enable */
#define I2C_CR1_ADDRIE_Pos           (3U)                                      
#define I2C_CR1_ADDRIE_Msk           (0x1U << I2C_CR1_ADDRIE_Pos)              /*!< 0x00000008 */
#define I2C_CR1_ADDRIE               I2C_CR1_ADDRIE_Msk                        /*!< Address match interrupt enable */
#define I2C_CR1_NACKIE_Pos           (4U)                                      
#define I2C_CR1_NACKIE_Msk           (0x1U << I2C_CR1_NACKIE_Pos)              /*!< 0x00000010 */
#define I2C_CR1_NACKIE               I2C_CR1_NACKIE_Msk                        /*!< NACK received interrupt enable */
#define I2C_CR1_STOPIE_Pos           (5U)                                      
#define I2C_CR1_STOPIE_Msk           (0x1U << I2C_CR1_STOPIE_Pos)              /*!< 0x00000020 */
#define I2C_CR1_STOPIE               I2C_CR1_STOPIE_Msk                        /*!< STOP detection interrupt enable */
#define I2C_CR1_TCIE_Pos             (6U)                                      
#define I2C_CR1_TCIE_Msk             (0x1U << I2C_CR1_TCIE_Pos)                /*!< 0x00000040 */
#define I2C_CR1_TCIE                 I2C_CR1_TCIE_Msk                          /*!< Transfer complete interrupt enable */
#define I2C_CR1_ERRIE_Pos            (7U)                                      
#define I2C_CR1_ERRIE_Msk            (0x1U << I2C_CR1_ERRIE_Pos)               /*!< 0x00000080 */
#define I2C_CR1_ERRIE                I2C_CR1_ERRIE_Msk                         /*!< Errors interrupt enable */
#define I2C_CR1_DNF_Pos              (8U)                                      
#define I2C_CR1_DNF_Msk              (0xFU << I2C_CR1_DNF_Pos)                 /*!< 0x00000F00 */
#define I2C_CR1_DNF                  I2C_CR1_DNF_Msk                           /*!< Digital noise filter */
#define I2C_CR1_ANFOFF_Pos           (12U)                                     
#define I2C_CR1_ANFOFF_Msk           (0x1U << I2C_CR1_ANFOFF_Pos)              /*!< 0x00001000 */
#define I2C_CR1_ANFOFF               I2C_CR1_ANFOFF_Msk                        /*!< Analog noise filter OFF */
#define I2C_CR1_SWRST_Pos            (13U)                                     
#define I2C_CR1_SWRST_Msk            (0x1U << I2C_CR1_SWRST_Pos)               /*!< 0x00002000 */
#define I2C_CR1_SWRST                I2C_CR1_SWRST_Msk                         /*!< Software reset */
#define I2C_CR1_TXDMAEN_Pos          (14U)                                     
#define I2C_CR1_TXDMAEN_Msk          (0x1U << I2C_CR1_TXDMAEN_Pos)             /*!< 0x00004000 */
#define I2C_CR1_TXDMAEN              I2C_CR1_TXDMAEN_Msk                       /*!< DMA transmission requests enable */
#define I2C_CR1_RXDMAEN_Pos          (15U)                                     
#define I2C_CR1_RXDMAEN_Msk          (0x1U << I2C_CR1_RXDMAEN_Pos)             /*!< 0x00008000 */
#define I2C_CR1_RXDMAEN              I2C_CR1_RXDMAEN_Msk                       /*!< DMA reception requests enable */
#define I2C_CR1_SBC_Pos              (16U)                                     
#define I2C_CR1_SBC_Msk              (0x1U << I2C_CR1_SBC_Pos)                 /*!< 0x00010000 */
#define I2C_CR1_SBC                  I2C_CR1_SBC_Msk                           /*!< Slave byte control */
#define I2C_CR1_NOSTRETCH_Pos        (17U)                                     
#define I2C_CR1_NOSTRETCH_Msk        (0x1U << I2C_CR1_NOSTRETCH_Pos)           /*!< 0x00020000 */
#define I2C_CR1_NOSTRETCH            I2C_CR1_NOSTRETCH_Msk                     /*!< Clock stretching disable */
#define I2C_CR1_WUPEN_Pos            (18U)                                     
#define I2C_CR1_WUPEN_Msk            (0x1U << I2C_CR1_WUPEN_Pos)               /*!< 0x00040000 */
#define I2C_CR1_WUPEN                I2C_CR1_WUPEN_Msk                         /*!< Wakeup from STOP enable */
#define I2C_CR1_GCEN_Pos             (19U)                                     
#define I2C_CR1_GCEN_Msk             (0x1U << I2C_CR1_GCEN_Pos)                /*!< 0x00080000 */
#define I2C_CR1_GCEN                 I2C_CR1_GCEN_Msk                          /*!< General call enable */
#define I2C_CR1_SMBHEN_Pos           (20U)                                     
#define I2C_CR1_SMBHEN_Msk           (0x1U << I2C_CR1_SMBHEN_Pos)              /*!< 0x00100000 */
#define I2C_CR1_SMBHEN               I2C_CR1_SMBHEN_Msk                        /*!< SMBus host address enable */
#define I2C_CR1_SMBDEN_Pos           (21U)                                     
#define I2C_CR1_SMBDEN_Msk           (0x1U << I2C_CR1_SMBDEN_Pos)              /*!< 0x00200000 */
#define I2C_CR1_SMBDEN               I2C_CR1_SMBDEN_Msk                        /*!< SMBus device default address enable */
#define I2C_CR1_ALERTEN_Pos          (22U)                                     
#define I2C_CR1_ALERTEN_Msk          (0x1U << I2C_CR1_ALERTEN_Pos)             /*!< 0x00400000 */
#define I2C_CR1_ALERTEN              I2C_CR1_ALERTEN_Msk                       /*!< SMBus alert enable */
#define I2C_CR1_PECEN_Pos            (23U)                                     
#define I2C_CR1_PECEN_Msk            (0x1U << I2C_CR1_PECEN_Pos)               /*!< 0x00800000 */
#define I2C_CR1_PECEN                I2C_CR1_PECEN_Msk                         /*!< PEC enable */

/* Legacy defines */
#define I2C_CR1_DFN I2C_CR1_DNF

/******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_SADD_Pos             (0U)                                      
#define I2C_CR2_SADD_Msk             (0x3FFU << I2C_CR2_SADD_Pos)              /*!< 0x000003FF */
#define I2C_CR2_SADD                 I2C_CR2_SADD_Msk                          /*!< Slave address (master mode) */
#define I2C_CR2_RD_WRN_Pos           (10U)                                     
#define I2C_CR2_RD_WRN_Msk           (0x1U << I2C_CR2_RD_WRN_Pos)              /*!< 0x00000400 */
#define I2C_CR2_RD_WRN               I2C_CR2_RD_WRN_Msk                        /*!< Transfer direction (master mode) */
#define I2C_CR2_ADD10_Pos            (11U)                                     
#define I2C_CR2_ADD10_Msk            (0x1U << I2C_CR2_ADD10_Pos)               /*!< 0x00000800 */
#define I2C_CR2_ADD10                I2C_CR2_ADD10_Msk                         /*!< 10-bit addressing mode (master mode) */
#define I2C_CR2_HEAD10R_Pos          (12U)                                     
#define I2C_CR2_HEAD10R_Msk          (0x1U << I2C_CR2_HEAD10R_Pos)             /*!< 0x00001000 */
#define I2C_CR2_HEAD10R              I2C_CR2_HEAD10R_Msk                       /*!< 10-bit address header only read direction (master mode) */
#define I2C_CR2_START_Pos            (13U)                                     
#define I2C_CR2_START_Msk            (0x1U << I2C_CR2_START_Pos)               /*!< 0x00002000 */
#define I2C_CR2_START                I2C_CR2_START_Msk                         /*!< START generation */
#define I2C_CR2_STOP_Pos             (14U)                                     
#define I2C_CR2_STOP_Msk             (0x1U << I2C_CR2_STOP_Pos)                /*!< 0x00004000 */
#define I2C_CR2_STOP                 I2C_CR2_STOP_Msk                          /*!< STOP generation (master mode) */
#define I2C_CR2_NACK_Pos             (15U)                                     
#define I2C_CR2_NACK_Msk             (0x1U << I2C_CR2_NACK_Pos)                /*!< 0x00008000 */
#define I2C_CR2_NACK                 I2C_CR2_NACK_Msk                          /*!< NACK generation (slave mode) */
#define I2C_CR2_NBYTES_Pos           (16U)                                     
#define I2C_CR2_NBYTES_Msk           (0xFFU << I2C_CR2_NBYTES_Pos)             /*!< 0x00FF0000 */
#define I2C_CR2_NBYTES               I2C_CR2_NBYTES_Msk                        /*!< Number of bytes */
#define I2C_CR2_RELOAD_Pos           (24U)                                     
#define I2C_CR2_RELOAD_Msk           (0x1U << I2C_CR2_RELOAD_Pos)              /*!< 0x01000000 */
#define I2C_CR2_RELOAD               I2C_CR2_RELOAD_Msk                        /*!< NBYTES reload mode */
#define I2C_CR2_AUTOEND_Pos          (25U)                                     
#define I2C_CR2_AUTOEND_Msk          (0x1U << I2C_CR2_AUTOEND_Pos)             /*!< 0x02000000 */
#define I2C_CR2_AUTOEND              I2C_CR2_AUTOEND_Msk                       /*!< Automatic end mode (master mode) */
#define I2C_CR2_PECBYTE_Pos          (26U)                                     
#define I2C_CR2_PECBYTE_Msk          (0x1U << I2C_CR2_PECBYTE_Pos)             /*!< 0x04000000 */
#define I2C_CR2_PECBYTE              I2C_CR2_PECBYTE_Msk                       /*!< Packet error checking byte */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define I2C_OAR1_OA1_Pos             (0U)                                      
#define I2C_OAR1_OA1_Msk             (0x3FFU << I2C_OAR1_OA1_Pos)              /*!< 0x000003FF */
#define I2C_OAR1_OA1                 I2C_OAR1_OA1_Msk                          /*!< Interface own address 1 */
#define I2C_OAR1_OA1MODE_Pos         (10U)                                     
#define I2C_OAR1_OA1MODE_Msk         (0x1U << I2C_OAR1_OA1MODE_Pos)            /*!< 0x00000400 */
#define I2C_OAR1_OA1MODE             I2C_OAR1_OA1MODE_Msk                      /*!< Own address 1 10-bit mode */
#define I2C_OAR1_OA1EN_Pos           (15U)                                     
#define I2C_OAR1_OA1EN_Msk           (0x1U << I2C_OAR1_OA1EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR1_OA1EN               I2C_OAR1_OA1EN_Msk                        /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_OA2_Pos             (1U)                                      
#define I2C_OAR2_OA2_Msk             (0x7FU << I2C_OAR2_OA2_Pos)               /*!< 0x000000FE */
#define I2C_OAR2_OA2                 I2C_OAR2_OA2_Msk                          /*!< Interface own address 2                        */
#define I2C_OAR2_OA2MSK_Pos          (8U)                                      
#define I2C_OAR2_OA2MSK_Msk          (0x7U << I2C_OAR2_OA2MSK_Pos)             /*!< 0x00000700 */
#define I2C_OAR2_OA2MSK              I2C_OAR2_OA2MSK_Msk                       /*!< Own address 2 masks                            */
#define I2C_OAR2_OA2NOMASK           (0x00000000U)                             /*!< No mask                                        */
#define I2C_OAR2_OA2MASK01_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK01_Msk       (0x1U << I2C_OAR2_OA2MASK01_Pos)          /*!< 0x00000100 */
#define I2C_OAR2_OA2MASK01           I2C_OAR2_OA2MASK01_Msk                    /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define I2C_OAR2_OA2MASK02_Pos       (9U)                                      
#define I2C_OAR2_OA2MASK02_Msk       (0x1U << I2C_OAR2_OA2MASK02_Pos)          /*!< 0x00000200 */
#define I2C_OAR2_OA2MASK02           I2C_OAR2_OA2MASK02_Msk                    /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define I2C_OAR2_OA2MASK03_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK03_Msk       (0x3U << I2C_OAR2_OA2MASK03_Pos)          /*!< 0x00000300 */
#define I2C_OAR2_OA2MASK03           I2C_OAR2_OA2MASK03_Msk                    /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define I2C_OAR2_OA2MASK04_Pos       (10U)                                     
#define I2C_OAR2_OA2MASK04_Msk       (0x1U << I2C_OAR2_OA2MASK04_Pos)          /*!< 0x00000400 */
#define I2C_OAR2_OA2MASK04           I2C_OAR2_OA2MASK04_Msk                    /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define I2C_OAR2_OA2MASK05_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK05_Msk       (0x5U << I2C_OAR2_OA2MASK05_Pos)          /*!< 0x00000500 */
#define I2C_OAR2_OA2MASK05           I2C_OAR2_OA2MASK05_Msk                    /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define I2C_OAR2_OA2MASK06_Pos       (9U)                                      
#define I2C_OAR2_OA2MASK06_Msk       (0x3U << I2C_OAR2_OA2MASK06_Pos)          /*!< 0x00000600 */
#define I2C_OAR2_OA2MASK06           I2C_OAR2_OA2MASK06_Msk                    /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define I2C_OAR2_OA2MASK07_Pos       (8U)                                      
#define I2C_OAR2_OA2MASK07_Msk       (0x7U << I2C_OAR2_OA2MASK07_Pos)          /*!< 0x00000700 */
#define I2C_OAR2_OA2MASK07           I2C_OAR2_OA2MASK07_Msk                    /*!< OA2[7:1] is masked, No comparison is done      */
#define I2C_OAR2_OA2EN_Pos           (15U)                                     
#define I2C_OAR2_OA2EN_Msk           (0x1U << I2C_OAR2_OA2EN_Pos)              /*!< 0x00008000 */
#define I2C_OAR2_OA2EN               I2C_OAR2_OA2EN_Msk                        /*!< Own address 2 enable                           */

/*******************  Bit definition for I2C_TIMINGR register *****************/
#define I2C_TIMINGR_SCLL_Pos         (0U)                                      
#define I2C_TIMINGR_SCLL_Msk         (0xFFU << I2C_TIMINGR_SCLL_Pos)           /*!< 0x000000FF */
#define I2C_TIMINGR_SCLL             I2C_TIMINGR_SCLL_Msk                      /*!< SCL low period (master mode) */
#define I2C_TIMINGR_SCLH_Pos         (8U)                                      
#define I2C_TIMINGR_SCLH_Msk         (0xFFU << I2C_TIMINGR_SCLH_Pos)           /*!< 0x0000FF00 */
#define I2C_TIMINGR_SCLH             I2C_TIMINGR_SCLH_Msk                      /*!< SCL high period (master mode) */
#define I2C_TIMINGR_SDADEL_Pos       (16U)                                     
#define I2C_TIMINGR_SDADEL_Msk       (0xFU << I2C_TIMINGR_SDADEL_Pos)          /*!< 0x000F0000 */
#define I2C_TIMINGR_SDADEL           I2C_TIMINGR_SDADEL_Msk                    /*!< Data hold time */
#define I2C_TIMINGR_SCLDEL_Pos       (20U)                                     
#define I2C_TIMINGR_SCLDEL_Msk       (0xFU << I2C_TIMINGR_SCLDEL_Pos)          /*!< 0x00F00000 */
#define I2C_TIMINGR_SCLDEL           I2C_TIMINGR_SCLDEL_Msk                    /*!< Data setup time */
#define I2C_TIMINGR_PRESC_Pos        (28U)                                     
#define I2C_TIMINGR_PRESC_Msk        (0xFU << I2C_TIMINGR_PRESC_Pos)           /*!< 0xF0000000 */
#define I2C_TIMINGR_PRESC            I2C_TIMINGR_PRESC_Msk                     /*!< Timings prescaler */

/******************* Bit definition for I2C_TIMEOUTR register *****************/
#define I2C_TIMEOUTR_TIMEOUTA_Pos    (0U)                                      
#define I2C_TIMEOUTR_TIMEOUTA_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTA_Pos)     /*!< 0x00000FFF */
#define I2C_TIMEOUTR_TIMEOUTA        I2C_TIMEOUTR_TIMEOUTA_Msk                 /*!< Bus timeout A */
#define I2C_TIMEOUTR_TIDLE_Pos       (12U)                                     
#define I2C_TIMEOUTR_TIDLE_Msk       (0x1U << I2C_TIMEOUTR_TIDLE_Pos)          /*!< 0x00001000 */
#define I2C_TIMEOUTR_TIDLE           I2C_TIMEOUTR_TIDLE_Msk                    /*!< Idle clock timeout detection */
#define I2C_TIMEOUTR_TIMOUTEN_Pos    (15U)                                     
#define I2C_TIMEOUTR_TIMOUTEN_Msk    (0x1U << I2C_TIMEOUTR_TIMOUTEN_Pos)       /*!< 0x00008000 */
#define I2C_TIMEOUTR_TIMOUTEN        I2C_TIMEOUTR_TIMOUTEN_Msk                 /*!< Clock timeout enable */
#define I2C_TIMEOUTR_TIMEOUTB_Pos    (16U)                                     
#define I2C_TIMEOUTR_TIMEOUTB_Msk    (0xFFFU << I2C_TIMEOUTR_TIMEOUTB_Pos)     /*!< 0x0FFF0000 */
#define I2C_TIMEOUTR_TIMEOUTB        I2C_TIMEOUTR_TIMEOUTB_Msk                 /*!< Bus timeout B*/
#define I2C_TIMEOUTR_TEXTEN_Pos      (31U)                                     
#define I2C_TIMEOUTR_TEXTEN_Msk      (0x1U << I2C_TIMEOUTR_TEXTEN_Pos)         /*!< 0x80000000 */
#define I2C_TIMEOUTR_TEXTEN          I2C_TIMEOUTR_TEXTEN_Msk                   /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  *********************/
#define I2C_ISR_TXE_Pos              (0U)                                      
#define I2C_ISR_TXE_Msk              (0x1U << I2C_ISR_TXE_Pos)                 /*!< 0x00000001 */
#define I2C_ISR_TXE                  I2C_ISR_TXE_Msk                           /*!< Transmit data register empty */
#define I2C_ISR_TXIS_Pos             (1U)                                      
#define I2C_ISR_TXIS_Msk             (0x1U << I2C_ISR_TXIS_Pos)                /*!< 0x00000002 */
#define I2C_ISR_TXIS                 I2C_ISR_TXIS_Msk                          /*!< Transmit interrupt status */
#define I2C_ISR_RXNE_Pos             (2U)                                      
#define I2C_ISR_RXNE_Msk             (0x1U << I2C_ISR_RXNE_Pos)                /*!< 0x00000004 */
#define I2C_ISR_RXNE                 I2C_ISR_RXNE_Msk                          /*!< Receive data register not empty */
#define I2C_ISR_ADDR_Pos             (3U)                                      
#define I2C_ISR_ADDR_Msk             (0x1U << I2C_ISR_ADDR_Pos)                /*!< 0x00000008 */
#define I2C_ISR_ADDR                 I2C_ISR_ADDR_Msk                          /*!< Address matched (slave mode)*/
#define I2C_ISR_NACKF_Pos            (4U)                                      
#define I2C_ISR_NACKF_Msk            (0x1U << I2C_ISR_NACKF_Pos)               /*!< 0x00000010 */
#define I2C_ISR_NACKF                I2C_ISR_NACKF_Msk                         /*!< NACK received flag */
#define I2C_ISR_STOPF_Pos            (5U)                                      
#define I2C_ISR_STOPF_Msk            (0x1U << I2C_ISR_STOPF_Pos)               /*!< 0x00000020 */
#define I2C_ISR_STOPF                I2C_ISR_STOPF_Msk                         /*!< STOP detection flag */
#define I2C_ISR_TC_Pos               (6U)                                      
#define I2C_ISR_TC_Msk               (0x1U << I2C_ISR_TC_Pos)                  /*!< 0x00000040 */
#define I2C_ISR_TC                   I2C_ISR_TC_Msk                            /*!< Transfer complete (master mode) */
#define I2C_ISR_TCR_Pos              (7U)                                      
#define I2C_ISR_TCR_Msk              (0x1U << I2C_ISR_TCR_Pos)                 /*!< 0x00000080 */
#define I2C_ISR_TCR                  I2C_ISR_TCR_Msk                           /*!< Transfer complete reload */
#define I2C_ISR_BERR_Pos             (8U)                                      
#define I2C_ISR_BERR_Msk             (0x1U << I2C_ISR_BERR_Pos)                /*!< 0x00000100 */
#define I2C_ISR_BERR                 I2C_ISR_BERR_Msk                          /*!< Bus error */
#define I2C_ISR_ARLO_Pos             (9U)                                      
#define I2C_ISR_ARLO_Msk             (0x1U << I2C_ISR_ARLO_Pos)                /*!< 0x00000200 */
#define I2C_ISR_ARLO                 I2C_ISR_ARLO_Msk                          /*!< Arbitration lost */
#define I2C_ISR_OVR_Pos              (10U)                                     
#define I2C_ISR_OVR_Msk              (0x1U << I2C_ISR_OVR_Pos)                 /*!< 0x00000400 */
#define I2C_ISR_OVR                  I2C_ISR_OVR_Msk                           /*!< Overrun/Underrun */
#define I2C_ISR_PECERR_Pos           (11U)                                     
#define I2C_ISR_PECERR_Msk           (0x1U << I2C_ISR_PECERR_Pos)              /*!< 0x00000800 */
#define I2C_ISR_PECERR               I2C_ISR_PECERR_Msk                        /*!< PEC error in reception */
#define I2C_ISR_TIMEOUT_Pos          (12U)                                     
#define I2C_ISR_TIMEOUT_Msk          (0x1U << I2C_ISR_TIMEOUT_Pos)             /*!< 0x00001000 */
#define I2C_ISR_TIMEOUT              I2C_ISR_TIMEOUT_Msk                       /*!< Timeout or Tlow detection flag */
#define I2C_ISR_ALERT_Pos            (13U)                                     
#define I2C_ISR_ALERT_Msk            (0x1U << I2C_ISR_ALERT_Pos)               /*!< 0x00002000 */
#define I2C_ISR_ALERT                I2C_ISR_ALERT_Msk                         /*!< SMBus alert */
#define I2C_ISR_BUSY_Pos             (15U)                                     
#define I2C_ISR_BUSY_Msk             (0x1U << I2C_ISR_BUSY_Pos)                /*!< 0x00008000 */
#define I2C_ISR_BUSY                 I2C_ISR_BUSY_Msk                          /*!< Bus busy */
#define I2C_ISR_DIR_Pos              (16U)                                     
#define I2C_ISR_DIR_Msk              (0x1U << I2C_ISR_DIR_Pos)                 /*!< 0x00010000 */
#define I2C_ISR_DIR                  I2C_ISR_DIR_Msk                           /*!< Transfer direction (slave mode) */
#define I2C_ISR_ADDCODE_Pos          (17U)                                     
#define I2C_ISR_ADDCODE_Msk          (0x7FU << I2C_ISR_ADDCODE_Pos)            /*!< 0x00FE0000 */
#define I2C_ISR_ADDCODE              I2C_ISR_ADDCODE_Msk                       /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  *********************/
#define I2C_ICR_ADDRCF_Pos           (3U)                                      
#define I2C_ICR_ADDRCF_Msk           (0x1U << I2C_ICR_ADDRCF_Pos)              /*!< 0x00000008 */
#define I2C_ICR_ADDRCF               I2C_ICR_ADDRCF_Msk                        /*!< Address matched clear flag */
#define I2C_ICR_NACKCF_Pos           (4U)                                      
#define I2C_ICR_NACKCF_Msk           (0x1U << I2C_ICR_NACKCF_Pos)              /*!< 0x00000010 */
#define I2C_ICR_NACKCF               I2C_ICR_NACKCF_Msk                        /*!< NACK clear flag */
#define I2C_ICR_STOPCF_Pos           (5U)                                      
#define I2C_ICR_STOPCF_Msk           (0x1U << I2C_ICR_STOPCF_Pos)              /*!< 0x00000020 */
#define I2C_ICR_STOPCF               I2C_ICR_STOPCF_Msk                        /*!< STOP detection clear flag */
#define I2C_ICR_BERRCF_Pos           (8U)                                      
#define I2C_ICR_BERRCF_Msk           (0x1U << I2C_ICR_BERRCF_Pos)              /*!< 0x00000100 */
#define I2C_ICR_BERRCF               I2C_ICR_BERRCF_Msk                        /*!< Bus error clear flag */
#define I2C_ICR_ARLOCF_Pos           (9U)                                      
#define I2C_ICR_ARLOCF_Msk           (0x1U << I2C_ICR_ARLOCF_Pos)              /*!< 0x00000200 */
#define I2C_ICR_ARLOCF               I2C_ICR_ARLOCF_Msk                        /*!< Arbitration lost clear flag */
#define I2C_ICR_OVRCF_Pos            (10U)                                     
#define I2C_ICR_OVRCF_Msk            (0x1U << I2C_ICR_OVRCF_Pos)               /*!< 0x00000400 */
#define I2C_ICR_OVRCF                I2C_ICR_OVRCF_Msk                         /*!< Overrun/Underrun clear flag */
#define I2C_ICR_PECCF_Pos            (11U)                                     
#define I2C_ICR_PECCF_Msk            (0x1U << I2C_ICR_PECCF_Pos)               /*!< 0x00000800 */
#define I2C_ICR_PECCF                I2C_ICR_PECCF_Msk                         /*!< PAC error clear flag */
#define I2C_ICR_TIMOUTCF_Pos         (12U)                                     
#define I2C_ICR_TIMOUTCF_Msk         (0x1U << I2C_ICR_TIMOUTCF_Pos)            /*!< 0x00001000 */
#define I2C_ICR_TIMOUTCF             I2C_ICR_TIMOUTCF_Msk                      /*!< Timeout clear flag */
#define I2C_ICR_ALERTCF_Pos          (13U)                                     
#define I2C_ICR_ALERTCF_Msk          (0x1U << I2C_ICR_ALERTCF_Pos)             /*!< 0x00002000 */
#define I2C_ICR_ALERTCF              I2C_ICR_ALERTCF_Msk                       /*!< Alert clear flag */

/******************  Bit definition for I2C_PECR register  ********************/
#define I2C_PECR_PEC_Pos             (0U)                                      
#define I2C_PECR_PEC_Msk             (0xFFU << I2C_PECR_PEC_Pos)               /*!< 0x000000FF */
#define I2C_PECR_PEC                 I2C_PECR_PEC_Msk                          /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define I2C_RXDR_RXDATA_Pos          (0U)                                      
#define I2C_RXDR_RXDATA_Msk          (0xFFU << I2C_RXDR_RXDATA_Pos)            /*!< 0x000000FF */
#define I2C_RXDR_RXDATA              I2C_RXDR_RXDATA_Msk                       /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
#define I2C_TXDR_TXDATA_Pos          (0U)                                      
#define I2C_TXDR_TXDATA_Msk          (0xFFU << I2C_TXDR_TXDATA_Pos)            /*!< 0x000000FF */
#define I2C_TXDR_TXDATA              I2C_TXDR_TXDATA_Msk                       /*!< 8-bit transmit data */


/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG (IWDG)                      */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define IWDG_KR_KEY_Pos      (0U)                                              
#define IWDG_KR_KEY_Msk      (0xFFFFU << IWDG_KR_KEY_Pos)                      /*!< 0x0000FFFF */
#define IWDG_KR_KEY          IWDG_KR_KEY_Msk                                   /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  ********************/
#define IWDG_PR_PR_Pos       (0U)                                              
#define IWDG_PR_PR_Msk       (0x7U << IWDG_PR_PR_Pos)                          /*!< 0x00000007 */
#define IWDG_PR_PR           IWDG_PR_PR_Msk                                    /*!< PR[2:0] (Prescaler divider) */
#define IWDG_PR_PR_0         (0x1U << IWDG_PR_PR_Pos)                          /*!< 0x00000001 */
#define IWDG_PR_PR_1         (0x2U << IWDG_PR_PR_Pos)                          /*!< 0x00000002 */
#define IWDG_PR_PR_2         (0x4U << IWDG_PR_PR_Pos)                          /*!< 0x00000004 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define IWDG_RLR_RL_Pos      (0U)                                              
#define IWDG_RLR_RL_Msk      (0xFFFU << IWDG_RLR_RL_Pos)                       /*!< 0x00000FFF */
#define IWDG_RLR_RL          IWDG_RLR_RL_Msk                                   /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  ********************/
#define IWDG_SR_PVU_Pos      (0U)                                              
#define IWDG_SR_PVU_Msk      (0x1U << IWDG_SR_PVU_Pos)                         /*!< 0x00000001 */
#define IWDG_SR_PVU          IWDG_SR_PVU_Msk                                   /*!< Watchdog prescaler value update */
#define IWDG_SR_RVU_Pos      (1U)                                              
#define IWDG_SR_RVU_Msk      (0x1U << IWDG_SR_RVU_Pos)                         /*!< 0x00000002 */
#define IWDG_SR_RVU          IWDG_SR_RVU_Msk                                   /*!< Watchdog counter reload value update */
#define IWDG_SR_WVU_Pos      (2U)                                              
#define IWDG_SR_WVU_Msk      (0x1U << IWDG_SR_WVU_Pos)                         /*!< 0x00000004 */
#define IWDG_SR_WVU          IWDG_SR_WVU_Msk                                   /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_KR register  ********************/
#define IWDG_WINR_WIN_Pos    (0U)                                              
#define IWDG_WINR_WIN_Msk    (0xFFFU << IWDG_WINR_WIN_Pos)                     /*!< 0x00000FFF */
#define IWDG_WINR_WIN        IWDG_WINR_WIN_Msk                                 /*!< Watchdog counter window value */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
#define PWR_PVD_SUPPORT                       /*!< PWR feature available only on specific devices: Power Voltage Detection feature */
/********************  Bit definition for PWR_CR register  ********************/
#define PWR_CR_LPDS_Pos            (0U)                                        
#define PWR_CR_LPDS_Msk            (0x1U << PWR_CR_LPDS_Pos)                   /*!< 0x00000001 */
#define PWR_CR_LPDS                PWR_CR_LPDS_Msk                             /*!< Low-power Deepsleep */
#define PWR_CR_PDDS_Pos            (1U)                                        
#define PWR_CR_PDDS_Msk            (0x1U << PWR_CR_PDDS_Pos)                   /*!< 0x00000002 */
#define PWR_CR_PDDS                PWR_CR_PDDS_Msk                             /*!< Power Down Deepsleep */
#define PWR_CR_CWUF_Pos            (2U)                                        
#define PWR_CR_CWUF_Msk            (0x1U << PWR_CR_CWUF_Pos)                   /*!< 0x00000004 */
#define PWR_CR_CWUF                PWR_CR_CWUF_Msk                             /*!< Clear Wakeup Flag */
#define PWR_CR_CSBF_Pos            (3U)                                        
#define PWR_CR_CSBF_Msk            (0x1U << PWR_CR_CSBF_Pos)                   /*!< 0x00000008 */
#define PWR_CR_CSBF                PWR_CR_CSBF_Msk                             /*!< Clear Standby Flag */
#define PWR_CR_PVDE_Pos            (4U)                                        
#define PWR_CR_PVDE_Msk            (0x1U << PWR_CR_PVDE_Pos)                   /*!< 0x00000010 */
#define PWR_CR_PVDE                PWR_CR_PVDE_Msk                             /*!< Power Voltage Detector Enable */

#define PWR_CR_PLS_Pos             (5U)                                        
#define PWR_CR_PLS_Msk             (0x7U << PWR_CR_PLS_Pos)                    /*!< 0x000000E0 */
#define PWR_CR_PLS                 PWR_CR_PLS_Msk                              /*!< PLS[2:0] bits (PVD Level Selection) */
#define PWR_CR_PLS_0               (0x1U << PWR_CR_PLS_Pos)                    /*!< 0x00000020 */
#define PWR_CR_PLS_1               (0x2U << PWR_CR_PLS_Pos)                    /*!< 0x00000040 */
#define PWR_CR_PLS_2               (0x4U << PWR_CR_PLS_Pos)                    /*!< 0x00000080 */

/*!< PVD level configuration */
#define PWR_CR_PLS_LEV0            (0x00000000U)                               /*!< PVD level 0 */
#define PWR_CR_PLS_LEV1            (0x00000020U)                               /*!< PVD level 1 */
#define PWR_CR_PLS_LEV2            (0x00000040U)                               /*!< PVD level 2 */
#define PWR_CR_PLS_LEV3            (0x00000060U)                               /*!< PVD level 3 */
#define PWR_CR_PLS_LEV4            (0x00000080U)                               /*!< PVD level 4 */
#define PWR_CR_PLS_LEV5            (0x000000A0U)                               /*!< PVD level 5 */
#define PWR_CR_PLS_LEV6            (0x000000C0U)                               /*!< PVD level 6 */
#define PWR_CR_PLS_LEV7            (0x000000E0U)                               /*!< PVD level 7 */

#define PWR_CR_DBP_Pos             (8U)                                        
#define PWR_CR_DBP_Msk             (0x1U << PWR_CR_DBP_Pos)                    /*!< 0x00000100 */
#define PWR_CR_DBP                 PWR_CR_DBP_Msk                              /*!< Disable Backup Domain write protection */

/*******************  Bit definition for PWR_CSR register  ********************/
#define PWR_CSR_WUF_Pos            (0U)                                        
#define PWR_CSR_WUF_Msk            (0x1U << PWR_CSR_WUF_Pos)                   /*!< 0x00000001 */
#define PWR_CSR_WUF                PWR_CSR_WUF_Msk                             /*!< Wakeup Flag */
#define PWR_CSR_SBF_Pos            (1U)                                        
#define PWR_CSR_SBF_Msk            (0x1U << PWR_CSR_SBF_Pos)                   /*!< 0x00000002 */
#define PWR_CSR_SBF                PWR_CSR_SBF_Msk                             /*!< Standby Flag */
#define PWR_CSR_PVDO_Pos           (2U)                                        
#define PWR_CSR_PVDO_Msk           (0x1U << PWR_CSR_PVDO_Pos)                  /*!< 0x00000004 */
#define PWR_CSR_PVDO               PWR_CSR_PVDO_Msk                            /*!< PVD Output */
#define PWR_CSR_VREFINTRDYF_Pos    (3U)                                        
#define PWR_CSR_VREFINTRDYF_Msk    (0x1U << PWR_CSR_VREFINTRDYF_Pos)           /*!< 0x00000008 */
#define PWR_CSR_VREFINTRDYF        PWR_CSR_VREFINTRDYF_Msk                     /*!< Internal voltage reference (VREFINT) ready flag */

#define PWR_CSR_EWUP1_Pos          (8U)                                        
#define PWR_CSR_EWUP1_Msk          (0x1U << PWR_CSR_EWUP1_Pos)                 /*!< 0x00000100 */
#define PWR_CSR_EWUP1              PWR_CSR_EWUP1_Msk                           /*!< Enable WKUP pin 1 */
#define PWR_CSR_EWUP2_Pos          (9U)                                        
#define PWR_CSR_EWUP2_Msk          (0x1U << PWR_CSR_EWUP2_Pos)                 /*!< 0x00000200 */
#define PWR_CSR_EWUP2              PWR_CSR_EWUP2_Msk                           /*!< Enable WKUP pin 2 */
#define PWR_CSR_EWUP3_Pos          (10U)                                       
#define PWR_CSR_EWUP3_Msk          (0x1U << PWR_CSR_EWUP3_Pos)                 /*!< 0x00000400 */
#define PWR_CSR_EWUP3              PWR_CSR_EWUP3_Msk                           /*!< Enable WKUP pin 3 */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                         (0U)                          
#define RCC_CR_HSION_Msk                         (0x1U << RCC_CR_HSION_Pos)    /*!< 0x00000001 */
#define RCC_CR_HSION                             RCC_CR_HSION_Msk              
#define RCC_CR_HSIRDY_Pos                        (1U)                          
#define RCC_CR_HSIRDY_Msk                        (0x1U << RCC_CR_HSIRDY_Pos)   /*!< 0x00000002 */
#define RCC_CR_HSIRDY                            RCC_CR_HSIRDY_Msk             

#define RCC_CR_HSITRIM_Pos                       (3U)                          
#define RCC_CR_HSITRIM_Msk                       (0x1FU << RCC_CR_HSITRIM_Pos) /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                           RCC_CR_HSITRIM_Msk            
#define RCC_CR_HSITRIM_0                         (0x01U << RCC_CR_HSITRIM_Pos) /*!< 0x00000008 */
#define RCC_CR_HSITRIM_1                         (0x02U << RCC_CR_HSITRIM_Pos) /*!< 0x00000010 */
#define RCC_CR_HSITRIM_2                         (0x04U << RCC_CR_HSITRIM_Pos) /*!< 0x00000020 */
#define RCC_CR_HSITRIM_3                         (0x08U << RCC_CR_HSITRIM_Pos) /*!< 0x00000040 */
#define RCC_CR_HSITRIM_4                         (0x10U << RCC_CR_HSITRIM_Pos) /*!< 0x00000080 */

#define RCC_CR_HSICAL_Pos                        (8U)                          
#define RCC_CR_HSICAL_Msk                        (0xFFU << RCC_CR_HSICAL_Pos)  /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                            RCC_CR_HSICAL_Msk             
#define RCC_CR_HSICAL_0                          (0x01U << RCC_CR_HSICAL_Pos)  /*!< 0x00000100 */
#define RCC_CR_HSICAL_1                          (0x02U << RCC_CR_HSICAL_Pos)  /*!< 0x00000200 */
#define RCC_CR_HSICAL_2                          (0x04U << RCC_CR_HSICAL_Pos)  /*!< 0x00000400 */
#define RCC_CR_HSICAL_3                          (0x08U << RCC_CR_HSICAL_Pos)  /*!< 0x00000800 */
#define RCC_CR_HSICAL_4                          (0x10U << RCC_CR_HSICAL_Pos)  /*!< 0x00001000 */
#define RCC_CR_HSICAL_5                          (0x20U << RCC_CR_HSICAL_Pos)  /*!< 0x00002000 */
#define RCC_CR_HSICAL_6                          (0x40U << RCC_CR_HSICAL_Pos)  /*!< 0x00004000 */
#define RCC_CR_HSICAL_7                          (0x80U << RCC_CR_HSICAL_Pos)  /*!< 0x00008000 */

#define RCC_CR_HSEON_Pos                         (16U)                         
#define RCC_CR_HSEON_Msk                         (0x1U << RCC_CR_HSEON_Pos)    /*!< 0x00010000 */
#define RCC_CR_HSEON                             RCC_CR_HSEON_Msk              
#define RCC_CR_HSERDY_Pos                        (17U)                         
#define RCC_CR_HSERDY_Msk                        (0x1U << RCC_CR_HSERDY_Pos)   /*!< 0x00020000 */
#define RCC_CR_HSERDY                            RCC_CR_HSERDY_Msk             
#define RCC_CR_HSEBYP_Pos                        (18U)                         
#define RCC_CR_HSEBYP_Msk                        (0x1U << RCC_CR_HSEBYP_Pos)   /*!< 0x00040000 */
#define RCC_CR_HSEBYP                            RCC_CR_HSEBYP_Msk             
#define RCC_CR_CSSON_Pos                         (19U)                         
#define RCC_CR_CSSON_Msk                         (0x1U << RCC_CR_CSSON_Pos)    /*!< 0x00080000 */
#define RCC_CR_CSSON                             RCC_CR_CSSON_Msk              
#define RCC_CR_PLLON_Pos                         (24U)                         
#define RCC_CR_PLLON_Msk                         (0x1U << RCC_CR_PLLON_Pos)    /*!< 0x01000000 */
#define RCC_CR_PLLON                             RCC_CR_PLLON_Msk              
#define RCC_CR_PLLRDY_Pos                        (25U)                         
#define RCC_CR_PLLRDY_Msk                        (0x1U << RCC_CR_PLLRDY_Pos)   /*!< 0x02000000 */
#define RCC_CR_PLLRDY                            RCC_CR_PLLRDY_Msk             

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                          (0U)                          
#define RCC_CFGR_SW_Msk                          (0x3U << RCC_CFGR_SW_Pos)     /*!< 0x00000003 */
#define RCC_CFGR_SW                              RCC_CFGR_SW_Msk               /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                            (0x1U << RCC_CFGR_SW_Pos)     /*!< 0x00000001 */
#define RCC_CFGR_SW_1                            (0x2U << RCC_CFGR_SW_Pos)     /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                          (0x00000000U)                 /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                          (0x00000001U)                 /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                          (0x00000002U)                 /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                         (2U)                          
#define RCC_CFGR_SWS_Msk                         (0x3U << RCC_CFGR_SWS_Pos)    /*!< 0x0000000C */
#define RCC_CFGR_SWS                             RCC_CFGR_SWS_Msk              /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                           (0x1U << RCC_CFGR_SWS_Pos)    /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                           (0x2U << RCC_CFGR_SWS_Pos)    /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                         (0x00000000U)                 /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                         (0x00000004U)                 /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                         (0x00000008U)                 /*!< PLL used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                        (4U)                          
#define RCC_CFGR_HPRE_Msk                        (0xFU << RCC_CFGR_HPRE_Pos)   /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                            RCC_CFGR_HPRE_Msk             /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                          (0x1U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                          (0x2U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                          (0x4U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                          (0x8U << RCC_CFGR_HPRE_Pos)   /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                       (0x00000000U)                 /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                       (0x00000080U)                 /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                       (0x00000090U)                 /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                       (0x000000A0U)                 /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                      (0x000000B0U)                 /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                      (0x000000C0U)                 /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                     (0x000000D0U)                 /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                     (0x000000E0U)                 /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                     (0x000000F0U)                 /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                       (8U)                          
#define RCC_CFGR_PPRE1_Msk                       (0x7U << RCC_CFGR_PPRE1_Pos)  /*!< 0x00000700 */
#define RCC_CFGR_PPRE1                           RCC_CFGR_PPRE1_Msk            /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                         (0x1U << RCC_CFGR_PPRE1_Pos)  /*!< 0x00000100 */
#define RCC_CFGR_PPRE1_1                         (0x2U << RCC_CFGR_PPRE1_Pos)  /*!< 0x00000200 */
#define RCC_CFGR_PPRE1_2                         (0x4U << RCC_CFGR_PPRE1_Pos)  /*!< 0x00000400 */

#define RCC_CFGR_PPRE1_DIV1                      (0x00000000U)                 /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2                      (0x00000400U)                 /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4                      (0x00000500U)                 /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8                      (0x00000600U)                 /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16                     (0x00000700U)                 /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                       (11U)                         
#define RCC_CFGR_PPRE2_Msk                       (0x7U << RCC_CFGR_PPRE2_Pos)  /*!< 0x00003800 */
#define RCC_CFGR_PPRE2                           RCC_CFGR_PPRE2_Msk            /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                         (0x1U << RCC_CFGR_PPRE2_Pos)  /*!< 0x00000800 */
#define RCC_CFGR_PPRE2_1                         (0x2U << RCC_CFGR_PPRE2_Pos)  /*!< 0x00001000 */
#define RCC_CFGR_PPRE2_2                         (0x4U << RCC_CFGR_PPRE2_Pos)  /*!< 0x00002000 */

#define RCC_CFGR_PPRE2_DIV1                      (0x00000000U)                 /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2                      (0x00002000U)                 /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4                      (0x00002800U)                 /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8                      (0x00003000U)                 /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16                     (0x00003800U)                 /*!< HCLK divided by 16 */

#define RCC_CFGR_PLLSRC_Pos                      (16U)                         
#define RCC_CFGR_PLLSRC_Msk                      (0x1U << RCC_CFGR_PLLSRC_Pos) /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC                          RCC_CFGR_PLLSRC_Msk           /*!< PLL entry clock source */
#define RCC_CFGR_PLLSRC_HSI_DIV2                 (0x00000000U)                 /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define RCC_CFGR_PLLSRC_HSE_PREDIV               (0x00010000U)                 /*!< HSE/PREDIV clock selected as PLL entry clock source */

#define RCC_CFGR_PLLXTPRE_Pos                    (17U)                         
#define RCC_CFGR_PLLXTPRE_Msk                    (0x1U << RCC_CFGR_PLLXTPRE_Pos) /*!< 0x00020000 */
#define RCC_CFGR_PLLXTPRE                        RCC_CFGR_PLLXTPRE_Msk         /*!< HSE divider for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1        (0x00000000U)                 /*!< HSE/PREDIV clock not divided for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2        (0x00020000U)                 /*!< HSE/PREDIV clock divided by 2 for PLL entry */

/*!< PLLMUL configuration */
#define RCC_CFGR_PLLMUL_Pos                      (18U)                         
#define RCC_CFGR_PLLMUL_Msk                      (0xFU << RCC_CFGR_PLLMUL_Pos) /*!< 0x003C0000 */
#define RCC_CFGR_PLLMUL                          RCC_CFGR_PLLMUL_Msk           /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_PLLMUL_0                        (0x1U << RCC_CFGR_PLLMUL_Pos) /*!< 0x00040000 */
#define RCC_CFGR_PLLMUL_1                        (0x2U << RCC_CFGR_PLLMUL_Pos) /*!< 0x00080000 */
#define RCC_CFGR_PLLMUL_2                        (0x4U << RCC_CFGR_PLLMUL_Pos) /*!< 0x00100000 */
#define RCC_CFGR_PLLMUL_3                        (0x8U << RCC_CFGR_PLLMUL_Pos) /*!< 0x00200000 */

#define RCC_CFGR_PLLMUL2                         (0x00000000U)                 /*!< PLL input clock*2 */
#define RCC_CFGR_PLLMUL3                         (0x00040000U)                 /*!< PLL input clock*3 */
#define RCC_CFGR_PLLMUL4                         (0x00080000U)                 /*!< PLL input clock*4 */
#define RCC_CFGR_PLLMUL5                         (0x000C0000U)                 /*!< PLL input clock*5 */
#define RCC_CFGR_PLLMUL6                         (0x00100000U)                 /*!< PLL input clock*6 */
#define RCC_CFGR_PLLMUL7                         (0x00140000U)                 /*!< PLL input clock*7 */
#define RCC_CFGR_PLLMUL8                         (0x00180000U)                 /*!< PLL input clock*8 */
#define RCC_CFGR_PLLMUL9                         (0x001C0000U)                 /*!< PLL input clock*9 */
#define RCC_CFGR_PLLMUL10                        (0x00200000U)                 /*!< PLL input clock10 */
#define RCC_CFGR_PLLMUL11                        (0x00240000U)                 /*!< PLL input clock*11 */
#define RCC_CFGR_PLLMUL12                        (0x00280000U)                 /*!< PLL input clock*12 */
#define RCC_CFGR_PLLMUL13                        (0x002C0000U)                 /*!< PLL input clock*13 */
#define RCC_CFGR_PLLMUL14                        (0x00300000U)                 /*!< PLL input clock*14 */
#define RCC_CFGR_PLLMUL15                        (0x00340000U)                 /*!< PLL input clock*15 */
#define RCC_CFGR_PLLMUL16                        (0x00380000U)                 /*!< PLL input clock*16 */

/*!< I2S configuration */
#define RCC_CFGR_I2SSRC_Pos                      (23U)                         
#define RCC_CFGR_I2SSRC_Msk                      (0x1U << RCC_CFGR_I2SSRC_Pos) /*!< 0x00800000 */
#define RCC_CFGR_I2SSRC                          RCC_CFGR_I2SSRC_Msk           /*!< I2S external clock source selection */

#define RCC_CFGR_I2SSRC_SYSCLK                   (0x00000000U)                 /*!< System clock selected as I2S clock source */
#define RCC_CFGR_I2SSRC_EXT                      (0x00800000U)                 /*!< External clock selected as I2S clock source */

/*!< MCO configuration */
#define RCC_CFGR_MCO_Pos                         (24U)                         
#define RCC_CFGR_MCO_Msk                         (0x7U << RCC_CFGR_MCO_Pos)    /*!< 0x07000000 */
#define RCC_CFGR_MCO                             RCC_CFGR_MCO_Msk              /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_0                           (0x1U << RCC_CFGR_MCO_Pos)    /*!< 0x01000000 */
#define RCC_CFGR_MCO_1                           (0x2U << RCC_CFGR_MCO_Pos)    /*!< 0x02000000 */
#define RCC_CFGR_MCO_2                           (0x4U << RCC_CFGR_MCO_Pos)    /*!< 0x04000000 */

#define RCC_CFGR_MCO_NOCLOCK                     (0x00000000U)                 /*!< No clock */
#define RCC_CFGR_MCO_LSI                         (0x02000000U)                 /*!< LSI clock selected as MCO source */
#define RCC_CFGR_MCO_LSE                         (0x03000000U)                 /*!< LSE clock selected as MCO source */
#define RCC_CFGR_MCO_SYSCLK                      (0x04000000U)                 /*!< System clock selected as MCO source */
#define RCC_CFGR_MCO_HSI                         (0x05000000U)                 /*!< HSI clock selected as MCO source */
#define RCC_CFGR_MCO_HSE                         (0x06000000U)                 /*!< HSE clock selected as MCO source  */
#define RCC_CFGR_MCO_PLL                         (0x07000000U)                 /*!< PLL clock divided by 2 selected as MCO source */

#define RCC_CFGR_MCOPRE_Pos                      (28U)                         
#define RCC_CFGR_MCOPRE_Msk                      (0x7U << RCC_CFGR_MCOPRE_Pos) /*!< 0x70000000 */
#define RCC_CFGR_MCOPRE                          RCC_CFGR_MCOPRE_Msk           /*!< MCOPRE[3:0] bits (Microcontroller Clock Output Prescaler) */
#define RCC_CFGR_MCOPRE_0                        (0x1U << RCC_CFGR_MCOPRE_Pos) /*!< 0x10000000 */
#define RCC_CFGR_MCOPRE_1                        (0x2U << RCC_CFGR_MCOPRE_Pos) /*!< 0x20000000 */
#define RCC_CFGR_MCOPRE_2                        (0x4U << RCC_CFGR_MCOPRE_Pos) /*!< 0x40000000 */

#define RCC_CFGR_MCOPRE_DIV1                     (0x00000000U)                 /*!< MCO is divided by 1 */
#define RCC_CFGR_MCOPRE_DIV2                     (0x10000000U)                 /*!< MCO is divided by 2 */
#define RCC_CFGR_MCOPRE_DIV4                     (0x20000000U)                 /*!< MCO is divided by 4 */
#define RCC_CFGR_MCOPRE_DIV8                     (0x30000000U)                 /*!< MCO is divided by 8 */
#define RCC_CFGR_MCOPRE_DIV16                    (0x40000000U)                 /*!< MCO is divided by 16 */
#define RCC_CFGR_MCOPRE_DIV32                    (0x50000000U)                 /*!< MCO is divided by 32 */
#define RCC_CFGR_MCOPRE_DIV64                    (0x60000000U)                 /*!< MCO is divided by 64 */
#define RCC_CFGR_MCOPRE_DIV128                   (0x70000000U)                 /*!< MCO is divided by 128 */

#define RCC_CFGR_PLLNODIV_Pos                    (31U)                         
#define RCC_CFGR_PLLNODIV_Msk                    (0x1U << RCC_CFGR_PLLNODIV_Pos) /*!< 0x80000000 */
#define RCC_CFGR_PLLNODIV                        RCC_CFGR_PLLNODIV_Msk         /*!< Do not divide PLL to MCO */

/* Reference defines */
#define RCC_CFGR_MCOSEL                      RCC_CFGR_MCO
#define RCC_CFGR_MCOSEL_0                    RCC_CFGR_MCO_0
#define RCC_CFGR_MCOSEL_1                    RCC_CFGR_MCO_1
#define RCC_CFGR_MCOSEL_2                    RCC_CFGR_MCO_2
#define RCC_CFGR_MCOSEL_NOCLOCK              RCC_CFGR_MCO_NOCLOCK
#define RCC_CFGR_MCOSEL_LSI                  RCC_CFGR_MCO_LSI
#define RCC_CFGR_MCOSEL_LSE                  RCC_CFGR_MCO_LSE
#define RCC_CFGR_MCOSEL_SYSCLK               RCC_CFGR_MCO_SYSCLK
#define RCC_CFGR_MCOSEL_HSI                  RCC_CFGR_MCO_HSI
#define RCC_CFGR_MCOSEL_HSE                  RCC_CFGR_MCO_HSE
#define RCC_CFGR_MCOSEL_PLL_DIV2             RCC_CFGR_MCO_PLL

/*********************  Bit definition for RCC_CIR register  ********************/
#define RCC_CIR_LSIRDYF_Pos                      (0U)                          
#define RCC_CIR_LSIRDYF_Msk                      (0x1U << RCC_CIR_LSIRDYF_Pos) /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                          RCC_CIR_LSIRDYF_Msk           /*!< LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF_Pos                      (1U)                          
#define RCC_CIR_LSERDYF_Msk                      (0x1U << RCC_CIR_LSERDYF_Pos) /*!< 0x00000002 */
#define RCC_CIR_LSERDYF                          RCC_CIR_LSERDYF_Msk           /*!< LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF_Pos                      (2U)                          
#define RCC_CIR_HSIRDYF_Msk                      (0x1U << RCC_CIR_HSIRDYF_Pos) /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                          RCC_CIR_HSIRDYF_Msk           /*!< HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF_Pos                      (3U)                          
#define RCC_CIR_HSERDYF_Msk                      (0x1U << RCC_CIR_HSERDYF_Pos) /*!< 0x00000008 */
#define RCC_CIR_HSERDYF                          RCC_CIR_HSERDYF_Msk           /*!< HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF_Pos                      (4U)                          
#define RCC_CIR_PLLRDYF_Msk                      (0x1U << RCC_CIR_PLLRDYF_Pos) /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF                          RCC_CIR_PLLRDYF_Msk           /*!< PLL Ready Interrupt flag */
#define RCC_CIR_CSSF_Pos                         (7U)                          
#define RCC_CIR_CSSF_Msk                         (0x1U << RCC_CIR_CSSF_Pos)    /*!< 0x00000080 */
#define RCC_CIR_CSSF                             RCC_CIR_CSSF_Msk              /*!< Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE_Pos                     (8U)                          
#define RCC_CIR_LSIRDYIE_Msk                     (0x1U << RCC_CIR_LSIRDYIE_Pos) /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                         RCC_CIR_LSIRDYIE_Msk          /*!< LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE_Pos                     (9U)                          
#define RCC_CIR_LSERDYIE_Msk                     (0x1U << RCC_CIR_LSERDYIE_Pos) /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE                         RCC_CIR_LSERDYIE_Msk          /*!< LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE_Pos                     (10U)                         
#define RCC_CIR_HSIRDYIE_Msk                     (0x1U << RCC_CIR_HSIRDYIE_Pos) /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                         RCC_CIR_HSIRDYIE_Msk          /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE_Pos                     (11U)                         
#define RCC_CIR_HSERDYIE_Msk                     (0x1U << RCC_CIR_HSERDYIE_Pos) /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE                         RCC_CIR_HSERDYIE_Msk          /*!< HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE_Pos                     (12U)                         
#define RCC_CIR_PLLRDYIE_Msk                     (0x1U << RCC_CIR_PLLRDYIE_Pos) /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE                         RCC_CIR_PLLRDYIE_Msk          /*!< PLL Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC_Pos                      (16U)                         
#define RCC_CIR_LSIRDYC_Msk                      (0x1U << RCC_CIR_LSIRDYC_Pos) /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                          RCC_CIR_LSIRDYC_Msk           /*!< LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC_Pos                      (17U)                         
#define RCC_CIR_LSERDYC_Msk                      (0x1U << RCC_CIR_LSERDYC_Pos) /*!< 0x00020000 */
#define RCC_CIR_LSERDYC                          RCC_CIR_LSERDYC_Msk           /*!< LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC_Pos                      (18U)                         
#define RCC_CIR_HSIRDYC_Msk                      (0x1U << RCC_CIR_HSIRDYC_Pos) /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                          RCC_CIR_HSIRDYC_Msk           /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC_Pos                      (19U)                         
#define RCC_CIR_HSERDYC_Msk                      (0x1U << RCC_CIR_HSERDYC_Pos) /*!< 0x00080000 */
#define RCC_CIR_HSERDYC                          RCC_CIR_HSERDYC_Msk           /*!< HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC_Pos                      (20U)                         
#define RCC_CIR_PLLRDYC_Msk                      (0x1U << RCC_CIR_PLLRDYC_Pos) /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC                          RCC_CIR_PLLRDYC_Msk           /*!< PLL Ready Interrupt Clear */
#define RCC_CIR_CSSC_Pos                         (23U)                         
#define RCC_CIR_CSSC_Msk                         (0x1U << RCC_CIR_CSSC_Pos)    /*!< 0x00800000 */
#define RCC_CIR_CSSC                             RCC_CIR_CSSC_Msk              /*!< Clock Security System Interrupt Clear */

/******************  Bit definition for RCC_APB2RSTR register  *****************/
#define RCC_APB2RSTR_SYSCFGRST_Pos               (0U)                          
#define RCC_APB2RSTR_SYSCFGRST_Msk               (0x1U << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00000001 */
#define RCC_APB2RSTR_SYSCFGRST                   RCC_APB2RSTR_SYSCFGRST_Msk    /*!< SYSCFG reset */
#define RCC_APB2RSTR_TIM1RST_Pos                 (11U)                         
#define RCC_APB2RSTR_TIM1RST_Msk                 (0x1U << RCC_APB2RSTR_TIM1RST_Pos) /*!< 0x00000800 */
#define RCC_APB2RSTR_TIM1RST                     RCC_APB2RSTR_TIM1RST_Msk      /*!< TIM1 reset */
#define RCC_APB2RSTR_USART1RST_Pos               (14U)                         
#define RCC_APB2RSTR_USART1RST_Msk               (0x1U << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_USART1RST                   RCC_APB2RSTR_USART1RST_Msk    /*!< USART1 reset */
#define RCC_APB2RSTR_TIM15RST_Pos                (16U)                         
#define RCC_APB2RSTR_TIM15RST_Msk                (0x1U << RCC_APB2RSTR_TIM15RST_Pos) /*!< 0x00010000 */
#define RCC_APB2RSTR_TIM15RST                    RCC_APB2RSTR_TIM15RST_Msk     /*!< TIM15 reset */
#define RCC_APB2RSTR_TIM16RST_Pos                (17U)                         
#define RCC_APB2RSTR_TIM16RST_Msk                (0x1U << RCC_APB2RSTR_TIM16RST_Pos) /*!< 0x00020000 */
#define RCC_APB2RSTR_TIM16RST                    RCC_APB2RSTR_TIM16RST_Msk     /*!< TIM16 reset */
#define RCC_APB2RSTR_TIM17RST_Pos                (18U)                         
#define RCC_APB2RSTR_TIM17RST_Msk                (0x1U << RCC_APB2RSTR_TIM17RST_Pos) /*!< 0x00040000 */
#define RCC_APB2RSTR_TIM17RST                    RCC_APB2RSTR_TIM17RST_Msk     /*!< TIM17 reset */

/******************  Bit definition for RCC_APB1RSTR register  ******************/
#define RCC_APB1RSTR_TIM2RST_Pos                 (0U)                          
#define RCC_APB1RSTR_TIM2RST_Msk                 (0x1U << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST                     RCC_APB1RSTR_TIM2RST_Msk      /*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM6RST_Pos                 (4U)                          
#define RCC_APB1RSTR_TIM6RST_Msk                 (0x1U << RCC_APB1RSTR_TIM6RST_Pos) /*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST                     RCC_APB1RSTR_TIM6RST_Msk      /*!< Timer 6 reset */
#define RCC_APB1RSTR_WWDGRST_Pos                 (11U)                         
#define RCC_APB1RSTR_WWDGRST_Msk                 (0x1U << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST                     RCC_APB1RSTR_WWDGRST_Msk      /*!< Window Watchdog reset */
#define RCC_APB1RSTR_SPI2RST_Pos                 (14U)                         
#define RCC_APB1RSTR_SPI2RST_Msk                 (0x1U << RCC_APB1RSTR_SPI2RST_Pos) /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST                     RCC_APB1RSTR_SPI2RST_Msk      /*!< SPI2 reset */
#define RCC_APB1RSTR_SPI3RST_Pos                 (15U)                         
#define RCC_APB1RSTR_SPI3RST_Msk                 (0x1U << RCC_APB1RSTR_SPI3RST_Pos) /*!< 0x00008000 */
#define RCC_APB1RSTR_SPI3RST                     RCC_APB1RSTR_SPI3RST_Msk      /*!< SPI3 reset */
#define RCC_APB1RSTR_USART2RST_Pos               (17U)                         
#define RCC_APB1RSTR_USART2RST_Msk               (0x1U << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST                   RCC_APB1RSTR_USART2RST_Msk    /*!< USART 2 reset */
#define RCC_APB1RSTR_USART3RST_Pos               (18U)                         
#define RCC_APB1RSTR_USART3RST_Msk               (0x1U << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST                   RCC_APB1RSTR_USART3RST_Msk    /*!< USART 3 reset */
#define RCC_APB1RSTR_I2C1RST_Pos                 (21U)                         
#define RCC_APB1RSTR_I2C1RST_Msk                 (0x1U << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST                     RCC_APB1RSTR_I2C1RST_Msk      /*!< I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST_Pos                 (22U)                         
#define RCC_APB1RSTR_I2C2RST_Msk                 (0x1U << RCC_APB1RSTR_I2C2RST_Pos) /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST                     RCC_APB1RSTR_I2C2RST_Msk      /*!< I2C 2 reset */
#define RCC_APB1RSTR_PWRRST_Pos                  (28U)                         
#define RCC_APB1RSTR_PWRRST_Msk                  (0x1U << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                      RCC_APB1RSTR_PWRRST_Msk       /*!< PWR reset */
#define RCC_APB1RSTR_DAC1RST_Pos                 (29U)                         
#define RCC_APB1RSTR_DAC1RST_Msk                 (0x1U << RCC_APB1RSTR_DAC1RST_Pos) /*!< 0x20000000 */
#define RCC_APB1RSTR_DAC1RST                     RCC_APB1RSTR_DAC1RST_Msk      /*!< DAC 1 reset */
#define RCC_APB1RSTR_I2C3RST_Pos                 (30U)                         
#define RCC_APB1RSTR_I2C3RST_Msk                 (0x1U << RCC_APB1RSTR_I2C3RST_Pos) /*!< 0x40000000 */
#define RCC_APB1RSTR_I2C3RST                     RCC_APB1RSTR_I2C3RST_Msk      /*!< I2C 3 reset */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define RCC_AHBENR_DMA1EN_Pos                    (0U)                          
#define RCC_AHBENR_DMA1EN_Msk                    (0x1U << RCC_AHBENR_DMA1EN_Pos) /*!< 0x00000001 */
#define RCC_AHBENR_DMA1EN                        RCC_AHBENR_DMA1EN_Msk         /*!< DMA1 clock enable */
#define RCC_AHBENR_SRAMEN_Pos                    (2U)                          
#define RCC_AHBENR_SRAMEN_Msk                    (0x1U << RCC_AHBENR_SRAMEN_Pos) /*!< 0x00000004 */
#define RCC_AHBENR_SRAMEN                        RCC_AHBENR_SRAMEN_Msk         /*!< SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN_Pos                   (4U)                          
#define RCC_AHBENR_FLITFEN_Msk                   (0x1U << RCC_AHBENR_FLITFEN_Pos) /*!< 0x00000010 */
#define RCC_AHBENR_FLITFEN                       RCC_AHBENR_FLITFEN_Msk        /*!< FLITF clock enable */
#define RCC_AHBENR_CRCEN_Pos                     (6U)                          
#define RCC_AHBENR_CRCEN_Msk                     (0x1U << RCC_AHBENR_CRCEN_Pos) /*!< 0x00000040 */
#define RCC_AHBENR_CRCEN                         RCC_AHBENR_CRCEN_Msk          /*!< CRC clock enable */
#define RCC_AHBENR_GPIOAEN_Pos                   (17U)                         
#define RCC_AHBENR_GPIOAEN_Msk                   (0x1U << RCC_AHBENR_GPIOAEN_Pos) /*!< 0x00020000 */
#define RCC_AHBENR_GPIOAEN                       RCC_AHBENR_GPIOAEN_Msk        /*!< GPIOA clock enable */
#define RCC_AHBENR_GPIOBEN_Pos                   (18U)                         
#define RCC_AHBENR_GPIOBEN_Msk                   (0x1U << RCC_AHBENR_GPIOBEN_Pos) /*!< 0x00040000 */
#define RCC_AHBENR_GPIOBEN                       RCC_AHBENR_GPIOBEN_Msk        /*!< GPIOB clock enable */
#define RCC_AHBENR_GPIOCEN_Pos                   (19U)                         
#define RCC_AHBENR_GPIOCEN_Msk                   (0x1U << RCC_AHBENR_GPIOCEN_Pos) /*!< 0x00080000 */
#define RCC_AHBENR_GPIOCEN                       RCC_AHBENR_GPIOCEN_Msk        /*!< GPIOC clock enable */
#define RCC_AHBENR_GPIODEN_Pos                   (20U)                         
#define RCC_AHBENR_GPIODEN_Msk                   (0x1U << RCC_AHBENR_GPIODEN_Pos) /*!< 0x00100000 */
#define RCC_AHBENR_GPIODEN                       RCC_AHBENR_GPIODEN_Msk        /*!< GPIOD clock enable */
#define RCC_AHBENR_GPIOFEN_Pos                   (22U)                         
#define RCC_AHBENR_GPIOFEN_Msk                   (0x1U << RCC_AHBENR_GPIOFEN_Pos) /*!< 0x00400000 */
#define RCC_AHBENR_GPIOFEN                       RCC_AHBENR_GPIOFEN_Msk        /*!< GPIOF clock enable */
#define RCC_AHBENR_TSCEN_Pos                     (24U)                         
#define RCC_AHBENR_TSCEN_Msk                     (0x1U << RCC_AHBENR_TSCEN_Pos) /*!< 0x01000000 */
#define RCC_AHBENR_TSCEN                         RCC_AHBENR_TSCEN_Msk          /*!< TS clock enable */
#define RCC_AHBENR_ADC1EN_Pos                    (28U)                         
#define RCC_AHBENR_ADC1EN_Msk                    (0x1U << RCC_AHBENR_ADC1EN_Pos) /*!< 0x10000000 */
#define RCC_AHBENR_ADC1EN                        RCC_AHBENR_ADC1EN_Msk         /*!< ADC1 clock enable */

/*****************  Bit definition for RCC_APB2ENR register  ******************/
#define RCC_APB2ENR_SYSCFGEN_Pos                 (0U)                          
#define RCC_APB2ENR_SYSCFGEN_Msk                 (0x1U << RCC_APB2ENR_SYSCFGEN_Pos) /*!< 0x00000001 */
#define RCC_APB2ENR_SYSCFGEN                     RCC_APB2ENR_SYSCFGEN_Msk      /*!< SYSCFG clock enable */
#define RCC_APB2ENR_TIM1EN_Pos                   (11U)                         
#define RCC_APB2ENR_TIM1EN_Msk                   (0x1U << RCC_APB2ENR_TIM1EN_Pos) /*!< 0x00000800 */
#define RCC_APB2ENR_TIM1EN                       RCC_APB2ENR_TIM1EN_Msk        /*!< TIM1 clock enable */
#define RCC_APB2ENR_USART1EN_Pos                 (14U)                         
#define RCC_APB2ENR_USART1EN_Msk                 (0x1U << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00004000 */
#define RCC_APB2ENR_USART1EN                     RCC_APB2ENR_USART1EN_Msk      /*!< USART1 clock enable */
#define RCC_APB2ENR_TIM15EN_Pos                  (16U)                         
#define RCC_APB2ENR_TIM15EN_Msk                  (0x1U << RCC_APB2ENR_TIM15EN_Pos) /*!< 0x00010000 */
#define RCC_APB2ENR_TIM15EN                      RCC_APB2ENR_TIM15EN_Msk       /*!< TIM15 clock enable */
#define RCC_APB2ENR_TIM16EN_Pos                  (17U)                         
#define RCC_APB2ENR_TIM16EN_Msk                  (0x1U << RCC_APB2ENR_TIM16EN_Pos) /*!< 0x00020000 */
#define RCC_APB2ENR_TIM16EN                      RCC_APB2ENR_TIM16EN_Msk       /*!< TIM16 clock enable */
#define RCC_APB2ENR_TIM17EN_Pos                  (18U)                         
#define RCC_APB2ENR_TIM17EN_Msk                  (0x1U << RCC_APB2ENR_TIM17EN_Pos) /*!< 0x00040000 */
#define RCC_APB2ENR_TIM17EN                      RCC_APB2ENR_TIM17EN_Msk       /*!< TIM17 clock enable */

/******************  Bit definition for RCC_APB1ENR register  ******************/
#define RCC_APB1ENR_TIM2EN_Pos                   (0U)                          
#define RCC_APB1ENR_TIM2EN_Msk                   (0x1U << RCC_APB1ENR_TIM2EN_Pos) /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                       RCC_APB1ENR_TIM2EN_Msk        /*!< Timer 2 clock enable */
#define RCC_APB1ENR_TIM6EN_Pos                   (4U)                          
#define RCC_APB1ENR_TIM6EN_Msk                   (0x1U << RCC_APB1ENR_TIM6EN_Pos) /*!< 0x00000010 */
#define RCC_APB1ENR_TIM6EN                       RCC_APB1ENR_TIM6EN_Msk        /*!< Timer 6 clock enable */
#define RCC_APB1ENR_WWDGEN_Pos                   (11U)                         
#define RCC_APB1ENR_WWDGEN_Msk                   (0x1U << RCC_APB1ENR_WWDGEN_Pos) /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                       RCC_APB1ENR_WWDGEN_Msk        /*!< Window Watchdog clock enable */
#define RCC_APB1ENR_SPI2EN_Pos                   (14U)                         
#define RCC_APB1ENR_SPI2EN_Msk                   (0x1U << RCC_APB1ENR_SPI2EN_Pos) /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN                       RCC_APB1ENR_SPI2EN_Msk        /*!< SPI2 clock enable */
#define RCC_APB1ENR_SPI3EN_Pos                   (15U)                         
#define RCC_APB1ENR_SPI3EN_Msk                   (0x1U << RCC_APB1ENR_SPI3EN_Pos) /*!< 0x00008000 */
#define RCC_APB1ENR_SPI3EN                       RCC_APB1ENR_SPI3EN_Msk        /*!< SPI3 clock enable */
#define RCC_APB1ENR_USART2EN_Pos                 (17U)                         
#define RCC_APB1ENR_USART2EN_Msk                 (0x1U << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN                     RCC_APB1ENR_USART2EN_Msk      /*!< USART 2 clock enable */
#define RCC_APB1ENR_USART3EN_Pos                 (18U)                         
#define RCC_APB1ENR_USART3EN_Msk                 (0x1U << RCC_APB1ENR_USART3EN_Pos) /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN                     RCC_APB1ENR_USART3EN_Msk      /*!< USART 3 clock enable */
#define RCC_APB1ENR_I2C1EN_Pos                   (21U)                         
#define RCC_APB1ENR_I2C1EN_Msk                   (0x1U << RCC_APB1ENR_I2C1EN_Pos) /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                       RCC_APB1ENR_I2C1EN_Msk        /*!< I2C 1 clock enable */
#define RCC_APB1ENR_I2C2EN_Pos                   (22U)                         
#define RCC_APB1ENR_I2C2EN_Msk                   (0x1U << RCC_APB1ENR_I2C2EN_Pos) /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                       RCC_APB1ENR_I2C2EN_Msk        /*!< I2C 2 clock enable */
#define RCC_APB1ENR_PWREN_Pos                    (28U)                         
#define RCC_APB1ENR_PWREN_Msk                    (0x1U << RCC_APB1ENR_PWREN_Pos) /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                        RCC_APB1ENR_PWREN_Msk         /*!< PWR clock enable */
#define RCC_APB1ENR_DAC1EN_Pos                   (29U)                         
#define RCC_APB1ENR_DAC1EN_Msk                   (0x1U << RCC_APB1ENR_DAC1EN_Pos) /*!< 0x20000000 */
#define RCC_APB1ENR_DAC1EN                       RCC_APB1ENR_DAC1EN_Msk        /*!< DAC 1 clock enable */
#define RCC_APB1ENR_I2C3EN_Pos                   (30U)                         
#define RCC_APB1ENR_I2C3EN_Msk                   (0x1U << RCC_APB1ENR_I2C3EN_Pos) /*!< 0x40000000 */
#define RCC_APB1ENR_I2C3EN                       RCC_APB1ENR_I2C3EN_Msk        /*!< I2C 3 clock enable */

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSE_Pos                         (0U)                          
#define RCC_BDCR_LSE_Msk                         (0x7U << RCC_BDCR_LSE_Pos)    /*!< 0x00000007 */
#define RCC_BDCR_LSE                             RCC_BDCR_LSE_Msk              /*!< External Low Speed oscillator [2:0] bits */
#define RCC_BDCR_LSEON_Pos                       (0U)                          
#define RCC_BDCR_LSEON_Msk                       (0x1U << RCC_BDCR_LSEON_Pos)  /*!< 0x00000001 */
#define RCC_BDCR_LSEON                           RCC_BDCR_LSEON_Msk            /*!< External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY_Pos                      (1U)                          
#define RCC_BDCR_LSERDY_Msk                      (0x1U << RCC_BDCR_LSERDY_Pos) /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                          RCC_BDCR_LSERDY_Msk           /*!< External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP_Pos                      (2U)                          
#define RCC_BDCR_LSEBYP_Msk                      (0x1U << RCC_BDCR_LSEBYP_Pos) /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                          RCC_BDCR_LSEBYP_Msk           /*!< External Low Speed oscillator Bypass */

#define RCC_BDCR_LSEDRV_Pos                      (3U)                          
#define RCC_BDCR_LSEDRV_Msk                      (0x3U << RCC_BDCR_LSEDRV_Pos) /*!< 0x00000018 */
#define RCC_BDCR_LSEDRV                          RCC_BDCR_LSEDRV_Msk           /*!< LSEDRV[1:0] bits (LSE Osc. drive capability) */
#define RCC_BDCR_LSEDRV_0                        (0x1U << RCC_BDCR_LSEDRV_Pos) /*!< 0x00000008 */
#define RCC_BDCR_LSEDRV_1                        (0x2U << RCC_BDCR_LSEDRV_Pos) /*!< 0x00000010 */

#define RCC_BDCR_RTCSEL_Pos                      (8U)                          
#define RCC_BDCR_RTCSEL_Msk                      (0x3U << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                          RCC_BDCR_RTCSEL_Msk           /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define RCC_BDCR_RTCSEL_0                        (0x1U << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                        (0x2U << RCC_BDCR_RTCSEL_Pos) /*!< 0x00000200 */

/*!< RTC configuration */
#define RCC_BDCR_RTCSEL_NOCLOCK                  (0x00000000U)                 /*!< No clock */
#define RCC_BDCR_RTCSEL_LSE                      (0x00000100U)                 /*!< LSE oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_LSI                      (0x00000200U)                 /*!< LSI oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_HSE                      (0x00000300U)                 /*!< HSE oscillator clock divided by 32 used as RTC clock */

#define RCC_BDCR_RTCEN_Pos                       (15U)                         
#define RCC_BDCR_RTCEN_Msk                       (0x1U << RCC_BDCR_RTCEN_Pos)  /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                           RCC_BDCR_RTCEN_Msk            /*!< RTC clock enable */
#define RCC_BDCR_BDRST_Pos                       (16U)                         
#define RCC_BDCR_BDRST_Msk                       (0x1U << RCC_BDCR_BDRST_Pos)  /*!< 0x00010000 */
#define RCC_BDCR_BDRST                           RCC_BDCR_BDRST_Msk            /*!< Backup domain software reset  */

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                        (0U)                          
#define RCC_CSR_LSION_Msk                        (0x1U << RCC_CSR_LSION_Pos)   /*!< 0x00000001 */
#define RCC_CSR_LSION                            RCC_CSR_LSION_Msk             /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY_Pos                       (1U)                          
#define RCC_CSR_LSIRDY_Msk                       (0x1U << RCC_CSR_LSIRDY_Pos)  /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                           RCC_CSR_LSIRDY_Msk            /*!< Internal Low Speed oscillator Ready */
#define RCC_CSR_V18PWRRSTF_Pos                   (23U)                         
#define RCC_CSR_V18PWRRSTF_Msk                   (0x1U << RCC_CSR_V18PWRRSTF_Pos) /*!< 0x00800000 */
#define RCC_CSR_V18PWRRSTF                       RCC_CSR_V18PWRRSTF_Msk        /*!< V1.8 power domain reset flag */
#define RCC_CSR_RMVF_Pos                         (24U)                         
#define RCC_CSR_RMVF_Msk                         (0x1U << RCC_CSR_RMVF_Pos)    /*!< 0x01000000 */
#define RCC_CSR_RMVF                             RCC_CSR_RMVF_Msk              /*!< Remove reset flag */
#define RCC_CSR_OBLRSTF_Pos                      (25U)                         
#define RCC_CSR_OBLRSTF_Msk                      (0x1U << RCC_CSR_OBLRSTF_Pos) /*!< 0x02000000 */
#define RCC_CSR_OBLRSTF                          RCC_CSR_OBLRSTF_Msk           /*!< OBL reset flag */
#define RCC_CSR_PINRSTF_Pos                      (26U)                         
#define RCC_CSR_PINRSTF_Msk                      (0x1U << RCC_CSR_PINRSTF_Pos) /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                          RCC_CSR_PINRSTF_Msk           /*!< PIN reset flag */
#define RCC_CSR_PORRSTF_Pos                      (27U)                         
#define RCC_CSR_PORRSTF_Msk                      (0x1U << RCC_CSR_PORRSTF_Pos) /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                          RCC_CSR_PORRSTF_Msk           /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF_Pos                      (28U)                         
#define RCC_CSR_SFTRSTF_Msk                      (0x1U << RCC_CSR_SFTRSTF_Pos) /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                          RCC_CSR_SFTRSTF_Msk           /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF_Pos                     (29U)                         
#define RCC_CSR_IWDGRSTF_Msk                     (0x1U << RCC_CSR_IWDGRSTF_Pos) /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                         RCC_CSR_IWDGRSTF_Msk          /*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF_Pos                     (30U)                         
#define RCC_CSR_WWDGRSTF_Msk                     (0x1U << RCC_CSR_WWDGRSTF_Pos) /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                         RCC_CSR_WWDGRSTF_Msk          /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF_Pos                     (31U)                         
#define RCC_CSR_LPWRRSTF_Msk                     (0x1U << RCC_CSR_LPWRRSTF_Pos) /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                         RCC_CSR_LPWRRSTF_Msk          /*!< Low-Power reset flag */

/*******************  Bit definition for RCC_AHBRSTR register  ****************/
#define RCC_AHBRSTR_GPIOARST_Pos                 (17U)                         
#define RCC_AHBRSTR_GPIOARST_Msk                 (0x1U << RCC_AHBRSTR_GPIOARST_Pos) /*!< 0x00020000 */
#define RCC_AHBRSTR_GPIOARST                     RCC_AHBRSTR_GPIOARST_Msk      /*!< GPIOA reset */
#define RCC_AHBRSTR_GPIOBRST_Pos                 (18U)                         
#define RCC_AHBRSTR_GPIOBRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOBRST_Pos) /*!< 0x00040000 */
#define RCC_AHBRSTR_GPIOBRST                     RCC_AHBRSTR_GPIOBRST_Msk      /*!< GPIOB reset */
#define RCC_AHBRSTR_GPIOCRST_Pos                 (19U)                         
#define RCC_AHBRSTR_GPIOCRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOCRST_Pos) /*!< 0x00080000 */
#define RCC_AHBRSTR_GPIOCRST                     RCC_AHBRSTR_GPIOCRST_Msk      /*!< GPIOC reset */
#define RCC_AHBRSTR_GPIODRST_Pos                 (20U)                         
#define RCC_AHBRSTR_GPIODRST_Msk                 (0x1U << RCC_AHBRSTR_GPIODRST_Pos) /*!< 0x00100000 */
#define RCC_AHBRSTR_GPIODRST                     RCC_AHBRSTR_GPIODRST_Msk      /*!< GPIOD reset */
#define RCC_AHBRSTR_GPIOFRST_Pos                 (22U)                         
#define RCC_AHBRSTR_GPIOFRST_Msk                 (0x1U << RCC_AHBRSTR_GPIOFRST_Pos) /*!< 0x00400000 */
#define RCC_AHBRSTR_GPIOFRST                     RCC_AHBRSTR_GPIOFRST_Msk      /*!< GPIOF reset */
#define RCC_AHBRSTR_TSCRST_Pos                   (24U)                         
#define RCC_AHBRSTR_TSCRST_Msk                   (0x1U << RCC_AHBRSTR_TSCRST_Pos) /*!< 0x01000000 */
#define RCC_AHBRSTR_TSCRST                       RCC_AHBRSTR_TSCRST_Msk        /*!< TSC reset */
#define RCC_AHBRSTR_ADC1RST_Pos                  (28U)                         
#define RCC_AHBRSTR_ADC1RST_Msk                  (0x1U << RCC_AHBRSTR_ADC1RST_Pos) /*!< 0x10000000 */
#define RCC_AHBRSTR_ADC1RST                      RCC_AHBRSTR_ADC1RST_Msk       /*!< ADC1 reset */

/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV configuration */
#define RCC_CFGR2_PREDIV_Pos                     (0U)                          
#define RCC_CFGR2_PREDIV_Msk                     (0xFU << RCC_CFGR2_PREDIV_Pos) /*!< 0x0000000F */
#define RCC_CFGR2_PREDIV                         RCC_CFGR2_PREDIV_Msk          /*!< PREDIV[3:0] bits */
#define RCC_CFGR2_PREDIV_0                       (0x1U << RCC_CFGR2_PREDIV_Pos) /*!< 0x00000001 */
#define RCC_CFGR2_PREDIV_1                       (0x2U << RCC_CFGR2_PREDIV_Pos) /*!< 0x00000002 */
#define RCC_CFGR2_PREDIV_2                       (0x4U << RCC_CFGR2_PREDIV_Pos) /*!< 0x00000004 */
#define RCC_CFGR2_PREDIV_3                       (0x8U << RCC_CFGR2_PREDIV_Pos) /*!< 0x00000008 */

#define RCC_CFGR2_PREDIV_DIV1                    (0x00000000U)                 /*!< PREDIV input clock not divided */
#define RCC_CFGR2_PREDIV_DIV2                    (0x00000001U)                 /*!< PREDIV input clock divided by 2 */
#define RCC_CFGR2_PREDIV_DIV3                    (0x00000002U)                 /*!< PREDIV input clock divided by 3 */
#define RCC_CFGR2_PREDIV_DIV4                    (0x00000003U)                 /*!< PREDIV input clock divided by 4 */
#define RCC_CFGR2_PREDIV_DIV5                    (0x00000004U)                 /*!< PREDIV input clock divided by 5 */
#define RCC_CFGR2_PREDIV_DIV6                    (0x00000005U)                 /*!< PREDIV input clock divided by 6 */
#define RCC_CFGR2_PREDIV_DIV7                    (0x00000006U)                 /*!< PREDIV input clock divided by 7 */
#define RCC_CFGR2_PREDIV_DIV8                    (0x00000007U)                 /*!< PREDIV input clock divided by 8 */
#define RCC_CFGR2_PREDIV_DIV9                    (0x00000008U)                 /*!< PREDIV input clock divided by 9 */
#define RCC_CFGR2_PREDIV_DIV10                   (0x00000009U)                 /*!< PREDIV input clock divided by 10 */
#define RCC_CFGR2_PREDIV_DIV11                   (0x0000000AU)                 /*!< PREDIV input clock divided by 11 */
#define RCC_CFGR2_PREDIV_DIV12                   (0x0000000BU)                 /*!< PREDIV input clock divided by 12 */
#define RCC_CFGR2_PREDIV_DIV13                   (0x0000000CU)                 /*!< PREDIV input clock divided by 13 */
#define RCC_CFGR2_PREDIV_DIV14                   (0x0000000DU)                 /*!< PREDIV input clock divided by 14 */
#define RCC_CFGR2_PREDIV_DIV15                   (0x0000000EU)                 /*!< PREDIV input clock divided by 15 */
#define RCC_CFGR2_PREDIV_DIV16                   (0x0000000FU)                 /*!< PREDIV input clock divided by 16 */

/*!< ADC1PRES configuration */
#define RCC_CFGR2_ADC1PRES_Pos                   (4U)                          
#define RCC_CFGR2_ADC1PRES_Msk                   (0x1FU << RCC_CFGR2_ADC1PRES_Pos) /*!< 0x000001F0 */
#define RCC_CFGR2_ADC1PRES                       RCC_CFGR2_ADC1PRES_Msk        /*!< ADC1PRES[8:4] bits */
#define RCC_CFGR2_ADC1PRES_0                     (0x01U << RCC_CFGR2_ADC1PRES_Pos) /*!< 0x00000010 */
#define RCC_CFGR2_ADC1PRES_1                     (0x02U << RCC_CFGR2_ADC1PRES_Pos) /*!< 0x00000020 */
#define RCC_CFGR2_ADC1PRES_2                     (0x04U << RCC_CFGR2_ADC1PRES_Pos) /*!< 0x00000040 */
#define RCC_CFGR2_ADC1PRES_3                     (0x08U << RCC_CFGR2_ADC1PRES_Pos) /*!< 0x00000080 */
#define RCC_CFGR2_ADC1PRES_4                     (0x10U << RCC_CFGR2_ADC1PRES_Pos) /*!< 0x00000100 */

#define RCC_CFGR2_ADC1PRES_NO                    (0x00000000U)                 /*!< ADC1 clock disabled, ADC1 can use AHB clock */
#define RCC_CFGR2_ADC1PRES_DIV1                  (0x00000100U)                 /*!< ADC1 PLL clock divided by 1 */
#define RCC_CFGR2_ADC1PRES_DIV2                  (0x00000110U)                 /*!< ADC1 PLL clock divided by 2 */
#define RCC_CFGR2_ADC1PRES_DIV4                  (0x00000120U)                 /*!< ADC1 PLL clock divided by 4 */
#define RCC_CFGR2_ADC1PRES_DIV6                  (0x00000130U)                 /*!< ADC1 PLL clock divided by 6 */
#define RCC_CFGR2_ADC1PRES_DIV8                  (0x00000140U)                 /*!< ADC1 PLL clock divided by 8 */
#define RCC_CFGR2_ADC1PRES_DIV10                 (0x00000150U)                 /*!< ADC1 PLL clock divided by 10 */
#define RCC_CFGR2_ADC1PRES_DIV12                 (0x00000160U)                 /*!< ADC1 PLL clock divided by 12 */
#define RCC_CFGR2_ADC1PRES_DIV16                 (0x00000170U)                 /*!< ADC1 PLL clock divided by 16 */
#define RCC_CFGR2_ADC1PRES_DIV32                 (0x00000180U)                 /*!< ADC1 PLL clock divided by 32 */
#define RCC_CFGR2_ADC1PRES_DIV64                 (0x00000190U)                 /*!< ADC1 PLL clock divided by 64 */
#define RCC_CFGR2_ADC1PRES_DIV128                (0x000001A0U)                 /*!< ADC1 PLL clock divided by 128 */
#define RCC_CFGR2_ADC1PRES_DIV256                (0x000001B0U)                 /*!< ADC1 PLL clock divided by 256 */

/*******************  Bit definition for RCC_CFGR3 register  ******************/
#define RCC_CFGR3_USART1SW_Pos                   (0U)                          
#define RCC_CFGR3_USART1SW_Msk                   (0x3U << RCC_CFGR3_USART1SW_Pos) /*!< 0x00000003 */
#define RCC_CFGR3_USART1SW                       RCC_CFGR3_USART1SW_Msk        /*!< USART1SW[1:0] bits */
#define RCC_CFGR3_USART1SW_0                     (0x1U << RCC_CFGR3_USART1SW_Pos) /*!< 0x00000001 */
#define RCC_CFGR3_USART1SW_1                     (0x2U << RCC_CFGR3_USART1SW_Pos) /*!< 0x00000002 */

#define RCC_CFGR3_USART1SW_PCLK1                 (0x00000000U)                 /*!< PCLK1 clock used as USART1 clock source */
#define RCC_CFGR3_USART1SW_SYSCLK                (0x00000001U)                 /*!< System clock selected as USART1 clock source */
#define RCC_CFGR3_USART1SW_LSE                   (0x00000002U)                 /*!< LSE oscillator clock used as USART1 clock source */
#define RCC_CFGR3_USART1SW_HSI                   (0x00000003U)                 /*!< HSI oscillator clock used as USART1 clock source */
/* Legacy defines */
#define  RCC_CFGR3_USART1SW_PCLK             RCC_CFGR3_USART1SW_PCLK1

#define RCC_CFGR3_I2CSW_Pos                      (4U)                          
#define RCC_CFGR3_I2CSW_Msk                      (0x7U << RCC_CFGR3_I2CSW_Pos) /*!< 0x00000070 */
#define RCC_CFGR3_I2CSW                          RCC_CFGR3_I2CSW_Msk           /*!< I2CSW bits */
#define RCC_CFGR3_I2C1SW_Pos                     (4U)                          
#define RCC_CFGR3_I2C1SW_Msk                     (0x1U << RCC_CFGR3_I2C1SW_Pos) /*!< 0x00000010 */
#define RCC_CFGR3_I2C1SW                         RCC_CFGR3_I2C1SW_Msk          /*!< I2C1SW bits */
#define RCC_CFGR3_I2C2SW_Pos                     (5U)                          
#define RCC_CFGR3_I2C2SW_Msk                     (0x1U << RCC_CFGR3_I2C2SW_Pos) /*!< 0x00000020 */
#define RCC_CFGR3_I2C2SW                         RCC_CFGR3_I2C2SW_Msk          /*!< I2C2SW bits */
#define RCC_CFGR3_I2C3SW_Pos                     (6U)                          
#define RCC_CFGR3_I2C3SW_Msk                     (0x1U << RCC_CFGR3_I2C3SW_Pos) /*!< 0x00000040 */
#define RCC_CFGR3_I2C3SW                         RCC_CFGR3_I2C3SW_Msk          /*!< I2C3SW bits */

#define RCC_CFGR3_I2C1SW_HSI                     (0x00000000U)                 /*!< HSI oscillator clock used as I2C1 clock source */
#define RCC_CFGR3_I2C1SW_SYSCLK_Pos              (4U)                          
#define RCC_CFGR3_I2C1SW_SYSCLK_Msk              (0x1U << RCC_CFGR3_I2C1SW_SYSCLK_Pos) /*!< 0x00000010 */
#define RCC_CFGR3_I2C1SW_SYSCLK                  RCC_CFGR3_I2C1SW_SYSCLK_Msk   /*!< System clock selected as I2C1 clock source */
#define RCC_CFGR3_I2C2SW_HSI                     (0x00000000U)                 /*!< HSI oscillator clock used as I2C2 clock source */
#define RCC_CFGR3_I2C2SW_SYSCLK_Pos              (5U)                          
#define RCC_CFGR3_I2C2SW_SYSCLK_Msk              (0x1U << RCC_CFGR3_I2C2SW_SYSCLK_Pos) /*!< 0x00000020 */
#define RCC_CFGR3_I2C2SW_SYSCLK                  RCC_CFGR3_I2C2SW_SYSCLK_Msk   /*!< System clock selected as I2C2 clock source */
#define RCC_CFGR3_I2C3SW_HSI                     (0x00000000U)                 /*!< HSI oscillator clock used as I2C3 clock source */
#define RCC_CFGR3_I2C3SW_SYSCLK_Pos              (6U)                          
#define RCC_CFGR3_I2C3SW_SYSCLK_Msk              (0x1U << RCC_CFGR3_I2C3SW_SYSCLK_Pos) /*!< 0x00000040 */
#define RCC_CFGR3_I2C3SW_SYSCLK                  RCC_CFGR3_I2C3SW_SYSCLK_Msk   /*!< System clock selected as I2C3 clock source */

#define RCC_CFGR3_TIMSW_Pos                      (8U)                          
#define RCC_CFGR3_TIMSW_Msk                      (0x2DU << RCC_CFGR3_TIMSW_Pos) /*!< 0x00002D00 */
#define RCC_CFGR3_TIMSW                          RCC_CFGR3_TIMSW_Msk           /*!< TIMSW bits */
#define RCC_CFGR3_TIM1SW_Pos                     (8U)                          
#define RCC_CFGR3_TIM1SW_Msk                     (0x1U << RCC_CFGR3_TIM1SW_Pos) /*!< 0x00000100 */
#define RCC_CFGR3_TIM1SW                         RCC_CFGR3_TIM1SW_Msk          /*!< TIM1SW bits */
#define RCC_CFGR3_TIM15SW_Pos                    (10U)                         
#define RCC_CFGR3_TIM15SW_Msk                    (0x1U << RCC_CFGR3_TIM15SW_Pos) /*!< 0x00000400 */
#define RCC_CFGR3_TIM15SW                        RCC_CFGR3_TIM15SW_Msk         /*!< TIM15SW bits */
#define RCC_CFGR3_TIM16SW_Pos                    (11U)                         
#define RCC_CFGR3_TIM16SW_Msk                    (0x1U << RCC_CFGR3_TIM16SW_Pos) /*!< 0x00000800 */
#define RCC_CFGR3_TIM16SW                        RCC_CFGR3_TIM16SW_Msk         /*!< TIM16SW bits */
#define RCC_CFGR3_TIM17SW_Pos                    (13U)                         
#define RCC_CFGR3_TIM17SW_Msk                    (0x1U << RCC_CFGR3_TIM17SW_Pos) /*!< 0x00002000 */
#define RCC_CFGR3_TIM17SW                        RCC_CFGR3_TIM17SW_Msk         /*!< TIM17SW bits */
#define RCC_CFGR3_TIM1SW_PCLK2                   (0x00000000U)                 /*!< PCLK2 used as TIM1 clock source */
#define RCC_CFGR3_TIM1SW_PLL_Pos                 (8U)                          
#define RCC_CFGR3_TIM1SW_PLL_Msk                 (0x1U << RCC_CFGR3_TIM1SW_PLL_Pos) /*!< 0x00000100 */
#define RCC_CFGR3_TIM1SW_PLL                     RCC_CFGR3_TIM1SW_PLL_Msk      /*!< PLL clock used as TIM1 clock source */
#define RCC_CFGR3_TIM15SW_PCLK2                  (0x00000000U)                 /*!< PCLK2 used as TIM15 clock source */
#define RCC_CFGR3_TIM15SW_PLL_Pos                (10U)                         
#define RCC_CFGR3_TIM15SW_PLL_Msk                (0x1U << RCC_CFGR3_TIM15SW_PLL_Pos) /*!< 0x00000400 */
#define RCC_CFGR3_TIM15SW_PLL                    RCC_CFGR3_TIM15SW_PLL_Msk     /*!< PLL clock used as TIM15 clock source */
#define RCC_CFGR3_TIM16SW_PCLK2                  (0x00000000U)                 /*!< PCLK2 used as TIM16 clock source */
#define RCC_CFGR3_TIM16SW_PLL_Pos                (11U)                         
#define RCC_CFGR3_TIM16SW_PLL_Msk                (0x1U << RCC_CFGR3_TIM16SW_PLL_Pos) /*!< 0x00000800 */
#define RCC_CFGR3_TIM16SW_PLL                    RCC_CFGR3_TIM16SW_PLL_Msk     /*!< PLL clock used as TIM16 clock source */
#define RCC_CFGR3_TIM17SW_PCLK2                  (0x00000000U)                 /*!< PCLK2 used as TIM17 clock source */
#define RCC_CFGR3_TIM17SW_PLL_Pos                (13U)                         
#define RCC_CFGR3_TIM17SW_PLL_Msk                (0x1U << RCC_CFGR3_TIM17SW_PLL_Pos) /*!< 0x00002000 */
#define RCC_CFGR3_TIM17SW_PLL                    RCC_CFGR3_TIM17SW_PLL_Msk     /*!< PLL clock used as TIM17 clock source */

/* Legacy defines */
#define  RCC_CFGR3_TIM1SW_HCLK                RCC_CFGR3_TIM1SW_PCLK2
#define  RCC_CFGR3_TIM15SW_HCLK               RCC_CFGR3_TIM15SW_PCLK2
#define  RCC_CFGR3_TIM16SW_HCLK               RCC_CFGR3_TIM16SW_PCLK2
#define  RCC_CFGR3_TIM17SW_HCLK               RCC_CFGR3_TIM17SW_PCLK2

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/*
* @brief Specific device feature definitions  (not present on all devices in the STM32F3 serie)
*/
#define RTC_TAMPER1_SUPPORT  /*!< TAMPER 1 feature support */
#define RTC_TAMPER2_SUPPORT  /*!< TAMPER 2 feature support */
#define RTC_BACKUP_SUPPORT   /*!< BACKUP register feature support */
#define RTC_WAKEUP_SUPPORT   /*!< WAKEUP feature support */

/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM_Pos                (22U)                                     
#define RTC_TR_PM_Msk                (0x1U << RTC_TR_PM_Pos)                   /*!< 0x00400000 */
#define RTC_TR_PM                    RTC_TR_PM_Msk                             
#define RTC_TR_HT_Pos                (20U)                                     
#define RTC_TR_HT_Msk                (0x3U << RTC_TR_HT_Pos)                   /*!< 0x00300000 */
#define RTC_TR_HT                    RTC_TR_HT_Msk                             
#define RTC_TR_HT_0                  (0x1U << RTC_TR_HT_Pos)                   /*!< 0x00100000 */
#define RTC_TR_HT_1                  (0x2U << RTC_TR_HT_Pos)                   /*!< 0x00200000 */
#define RTC_TR_HU_Pos                (16U)                                     
#define RTC_TR_HU_Msk                (0xFU << RTC_TR_HU_Pos)                   /*!< 0x000F0000 */
#define RTC_TR_HU                    RTC_TR_HU_Msk                             
#define RTC_TR_HU_0                  (0x1U << RTC_TR_HU_Pos)                   /*!< 0x00010000 */
#define RTC_TR_HU_1                  (0x2U << RTC_TR_HU_Pos)                   /*!< 0x00020000 */
#define RTC_TR_HU_2                  (0x4U << RTC_TR_HU_Pos)                   /*!< 0x00040000 */
#define RTC_TR_HU_3                  (0x8U << RTC_TR_HU_Pos)                   /*!< 0x00080000 */
#define RTC_TR_MNT_Pos               (12U)                                     
#define RTC_TR_MNT_Msk               (0x7U << RTC_TR_MNT_Pos)                  /*!< 0x00007000 */
#define RTC_TR_MNT                   RTC_TR_MNT_Msk                            
#define RTC_TR_MNT_0                 (0x1U << RTC_TR_MNT_Pos)                  /*!< 0x00001000 */
#define RTC_TR_MNT_1                 (0x2U << RTC_TR_MNT_Pos)                  /*!< 0x00002000 */
#define RTC_TR_MNT_2                 (0x4U << RTC_TR_MNT_Pos)                  /*!< 0x00004000 */
#define RTC_TR_MNU_Pos               (8U)                                      
#define RTC_TR_MNU_Msk               (0xFU << RTC_TR_MNU_Pos)                  /*!< 0x00000F00 */
#define RTC_TR_MNU                   RTC_TR_MNU_Msk                            
#define RTC_TR_MNU_0                 (0x1U << RTC_TR_MNU_Pos)                  /*!< 0x00000100 */
#define RTC_TR_MNU_1                 (0x2U << RTC_TR_MNU_Pos)                  /*!< 0x00000200 */
#define RTC_TR_MNU_2                 (0x4U << RTC_TR_MNU_Pos)                  /*!< 0x00000400 */
#define RTC_TR_MNU_3                 (0x8U << RTC_TR_MNU_Pos)                  /*!< 0x00000800 */
#define RTC_TR_ST_Pos                (4U)                                      
#define RTC_TR_ST_Msk                (0x7U << RTC_TR_ST_Pos)                   /*!< 0x00000070 */
#define RTC_TR_ST                    RTC_TR_ST_Msk                             
#define RTC_TR_ST_0                  (0x1U << RTC_TR_ST_Pos)                   /*!< 0x00000010 */
#define RTC_TR_ST_1                  (0x2U << RTC_TR_ST_Pos)                   /*!< 0x00000020 */
#define RTC_TR_ST_2                  (0x4U << RTC_TR_ST_Pos)                   /*!< 0x00000040 */
#define RTC_TR_SU_Pos                (0U)                                      
#define RTC_TR_SU_Msk                (0xFU << RTC_TR_SU_Pos)                   /*!< 0x0000000F */
#define RTC_TR_SU                    RTC_TR_SU_Msk                             
#define RTC_TR_SU_0                  (0x1U << RTC_TR_SU_Pos)                   /*!< 0x00000001 */
#define RTC_TR_SU_1                  (0x2U << RTC_TR_SU_Pos)                   /*!< 0x00000002 */
#define RTC_TR_SU_2                  (0x4U << RTC_TR_SU_Pos)                   /*!< 0x00000004 */
#define RTC_TR_SU_3                  (0x8U << RTC_TR_SU_Pos)                   /*!< 0x00000008 */

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT_Pos                (20U)                                     
#define RTC_DR_YT_Msk                (0xFU << RTC_DR_YT_Pos)                   /*!< 0x00F00000 */
#define RTC_DR_YT                    RTC_DR_YT_Msk                             
#define RTC_DR_YT_0                  (0x1U << RTC_DR_YT_Pos)                   /*!< 0x00100000 */
#define RTC_DR_YT_1                  (0x2U << RTC_DR_YT_Pos)                   /*!< 0x00200000 */
#define RTC_DR_YT_2                  (0x4U << RTC_DR_YT_Pos)                   /*!< 0x00400000 */
#define RTC_DR_YT_3                  (0x8U << RTC_DR_YT_Pos)                   /*!< 0x00800000 */
#define RTC_DR_YU_Pos                (16U)                                     
#define RTC_DR_YU_Msk                (0xFU << RTC_DR_YU_Pos)                   /*!< 0x000F0000 */
#define RTC_DR_YU                    RTC_DR_YU_Msk                             
#define RTC_DR_YU_0                  (0x1U << RTC_DR_YU_Pos)                   /*!< 0x00010000 */
#define RTC_DR_YU_1                  (0x2U << RTC_DR_YU_Pos)                   /*!< 0x00020000 */
#define RTC_DR_YU_2                  (0x4U << RTC_DR_YU_Pos)                   /*!< 0x00040000 */
#define RTC_DR_YU_3                  (0x8U << RTC_DR_YU_Pos)                   /*!< 0x00080000 */
#define RTC_DR_WDU_Pos               (13U)                                     
#define RTC_DR_WDU_Msk               (0x7U << RTC_DR_WDU_Pos)                  /*!< 0x0000E000 */
#define RTC_DR_WDU                   RTC_DR_WDU_Msk                            
#define RTC_DR_WDU_0                 (0x1U << RTC_DR_WDU_Pos)                  /*!< 0x00002000 */
#define RTC_DR_WDU_1                 (0x2U << RTC_DR_WDU_Pos)                  /*!< 0x00004000 */
#define RTC_DR_WDU_2                 (0x4U << RTC_DR_WDU_Pos)                  /*!< 0x00008000 */
#define RTC_DR_MT_Pos                (12U)                                     
#define RTC_DR_MT_Msk                (0x1U << RTC_DR_MT_Pos)                   /*!< 0x00001000 */
#define RTC_DR_MT                    RTC_DR_MT_Msk                             
#define RTC_DR_MU_Pos                (8U)                                      
#define RTC_DR_MU_Msk                (0xFU << RTC_DR_MU_Pos)                   /*!< 0x00000F00 */
#define RTC_DR_MU                    RTC_DR_MU_Msk                             
#define RTC_DR_MU_0                  (0x1U << RTC_DR_MU_Pos)                   /*!< 0x00000100 */
#define RTC_DR_MU_1                  (0x2U << RTC_DR_MU_Pos)                   /*!< 0x00000200 */
#define RTC_DR_MU_2                  (0x4U << RTC_DR_MU_Pos)                   /*!< 0x00000400 */
#define RTC_DR_MU_3                  (0x8U << RTC_DR_MU_Pos)                   /*!< 0x00000800 */
#define RTC_DR_DT_Pos                (4U)                                      
#define RTC_DR_DT_Msk                (0x3U << RTC_DR_DT_Pos)                   /*!< 0x00000030 */
#define RTC_DR_DT                    RTC_DR_DT_Msk                             
#define RTC_DR_DT_0                  (0x1U << RTC_DR_DT_Pos)                   /*!< 0x00000010 */
#define RTC_DR_DT_1                  (0x2U << RTC_DR_DT_Pos)                   /*!< 0x00000020 */
#define RTC_DR_DU_Pos                (0U)                                      
#define RTC_DR_DU_Msk                (0xFU << RTC_DR_DU_Pos)                   /*!< 0x0000000F */
#define RTC_DR_DU                    RTC_DR_DU_Msk                             
#define RTC_DR_DU_0                  (0x1U << RTC_DR_DU_Pos)                   /*!< 0x00000001 */
#define RTC_DR_DU_1                  (0x2U << RTC_DR_DU_Pos)                   /*!< 0x00000002 */
#define RTC_DR_DU_2                  (0x4U << RTC_DR_DU_Pos)                   /*!< 0x00000004 */
#define RTC_DR_DU_3                  (0x8U << RTC_DR_DU_Pos)                   /*!< 0x00000008 */

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_COE_Pos               (23U)                                     
#define RTC_CR_COE_Msk               (0x1U << RTC_CR_COE_Pos)                  /*!< 0x00800000 */
#define RTC_CR_COE                   RTC_CR_COE_Msk                            
#define RTC_CR_OSEL_Pos              (21U)                                     
#define RTC_CR_OSEL_Msk              (0x3U << RTC_CR_OSEL_Pos)                 /*!< 0x00600000 */
#define RTC_CR_OSEL                  RTC_CR_OSEL_Msk                           
#define RTC_CR_OSEL_0                (0x1U << RTC_CR_OSEL_Pos)                 /*!< 0x00200000 */
#define RTC_CR_OSEL_1                (0x2U << RTC_CR_OSEL_Pos)                 /*!< 0x00400000 */
#define RTC_CR_POL_Pos               (20U)                                     
#define RTC_CR_POL_Msk               (0x1U << RTC_CR_POL_Pos)                  /*!< 0x00100000 */
#define RTC_CR_POL                   RTC_CR_POL_Msk                            
#define RTC_CR_COSEL_Pos             (19U)                                     
#define RTC_CR_COSEL_Msk             (0x1U << RTC_CR_COSEL_Pos)                /*!< 0x00080000 */
#define RTC_CR_COSEL                 RTC_CR_COSEL_Msk                          
#define RTC_CR_BCK_Pos               (18U)                                     
#define RTC_CR_BCK_Msk               (0x1U << RTC_CR_BCK_Pos)                  /*!< 0x00040000 */
#define RTC_CR_BCK                   RTC_CR_BCK_Msk                            
#define RTC_CR_SUB1H_Pos             (17U)                                     
#define RTC_CR_SUB1H_Msk             (0x1U << RTC_CR_SUB1H_Pos)                /*!< 0x00020000 */
#define RTC_CR_SUB1H                 RTC_CR_SUB1H_Msk                          
#define RTC_CR_ADD1H_Pos             (16U)                                     
#define RTC_CR_ADD1H_Msk             (0x1U << RTC_CR_ADD1H_Pos)                /*!< 0x00010000 */
#define RTC_CR_ADD1H                 RTC_CR_ADD1H_Msk                          
#define RTC_CR_TSIE_Pos              (15U)                                     
#define RTC_CR_TSIE_Msk              (0x1U << RTC_CR_TSIE_Pos)                 /*!< 0x00008000 */
#define RTC_CR_TSIE                  RTC_CR_TSIE_Msk                           
#define RTC_CR_WUTIE_Pos             (14U)                                     
#define RTC_CR_WUTIE_Msk             (0x1U << RTC_CR_WUTIE_Pos)                /*!< 0x00004000 */
#define RTC_CR_WUTIE                 RTC_CR_WUTIE_Msk                          
#define RTC_CR_ALRBIE_Pos            (13U)                                     
#define RTC_CR_ALRBIE_Msk            (0x1U << RTC_CR_ALRBIE_Pos)               /*!< 0x00002000 */
#define RTC_CR_ALRBIE                RTC_CR_ALRBIE_Msk                         
#define RTC_CR_ALRAIE_Pos            (12U)                                     
#define RTC_CR_ALRAIE_Msk            (0x1U << RTC_CR_ALRAIE_Pos)               /*!< 0x00001000 */
#define RTC_CR_ALRAIE                RTC_CR_ALRAIE_Msk                         
#define RTC_CR_TSE_Pos               (11U)                                     
#define RTC_CR_TSE_Msk               (0x1U << RTC_CR_TSE_Pos)                  /*!< 0x00000800 */
#define RTC_CR_TSE                   RTC_CR_TSE_Msk                            
#define RTC_CR_WUTE_Pos              (10U)                                     
#define RTC_CR_WUTE_Msk              (0x1U << RTC_CR_WUTE_Pos)                 /*!< 0x00000400 */
#define RTC_CR_WUTE                  RTC_CR_WUTE_Msk                           
#define RTC_CR_ALRBE_Pos             (9U)                                      
#define RTC_CR_ALRBE_Msk             (0x1U << RTC_CR_ALRBE_Pos)                /*!< 0x00000200 */
#define RTC_CR_ALRBE                 RTC_CR_ALRBE_Msk                          
#define RTC_CR_ALRAE_Pos             (8U)                                      
#define RTC_CR_ALRAE_Msk             (0x1U << RTC_CR_ALRAE_Pos)                /*!< 0x00000100 */
#define RTC_CR_ALRAE                 RTC_CR_ALRAE_Msk                          
#define RTC_CR_FMT_Pos               (6U)                                      
#define RTC_CR_FMT_Msk               (0x1U << RTC_CR_FMT_Pos)                  /*!< 0x00000040 */
#define RTC_CR_FMT                   RTC_CR_FMT_Msk                            
#define RTC_CR_BYPSHAD_Pos           (5U)                                      
#define RTC_CR_BYPSHAD_Msk           (0x1U << RTC_CR_BYPSHAD_Pos)              /*!< 0x00000020 */
#define RTC_CR_BYPSHAD               RTC_CR_BYPSHAD_Msk                        
#define RTC_CR_REFCKON_Pos           (4U)                                      
#define RTC_CR_REFCKON_Msk           (0x1U << RTC_CR_REFCKON_Pos)              /*!< 0x00000010 */
#define RTC_CR_REFCKON               RTC_CR_REFCKON_Msk                        
#define RTC_CR_TSEDGE_Pos            (3U)                                      
#define RTC_CR_TSEDGE_Msk            (0x1U << RTC_CR_TSEDGE_Pos)               /*!< 0x00000008 */
#define RTC_CR_TSEDGE                RTC_CR_TSEDGE_Msk                         
#define RTC_CR_WUCKSEL_Pos           (0U)                                      
#define RTC_CR_WUCKSEL_Msk           (0x7U << RTC_CR_WUCKSEL_Pos)              /*!< 0x00000007 */
#define RTC_CR_WUCKSEL               RTC_CR_WUCKSEL_Msk                        
#define RTC_CR_WUCKSEL_0             (0x1U << RTC_CR_WUCKSEL_Pos)              /*!< 0x00000001 */
#define RTC_CR_WUCKSEL_1             (0x2U << RTC_CR_WUCKSEL_Pos)              /*!< 0x00000002 */
#define RTC_CR_WUCKSEL_2             (0x4U << RTC_CR_WUCKSEL_Pos)              /*!< 0x00000004 */

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_RECALPF_Pos          (16U)                                     
#define RTC_ISR_RECALPF_Msk          (0x1U << RTC_ISR_RECALPF_Pos)             /*!< 0x00010000 */
#define RTC_ISR_RECALPF              RTC_ISR_RECALPF_Msk                       
#define RTC_ISR_TAMP2F_Pos           (14U)                                     
#define RTC_ISR_TAMP2F_Msk           (0x1U << RTC_ISR_TAMP2F_Pos)              /*!< 0x00004000 */
#define RTC_ISR_TAMP2F               RTC_ISR_TAMP2F_Msk                        
#define RTC_ISR_TAMP1F_Pos           (13U)                                     
#define RTC_ISR_TAMP1F_Msk           (0x1U << RTC_ISR_TAMP1F_Pos)              /*!< 0x00002000 */
#define RTC_ISR_TAMP1F               RTC_ISR_TAMP1F_Msk                        
#define RTC_ISR_TSOVF_Pos            (12U)                                     
#define RTC_ISR_TSOVF_Msk            (0x1U << RTC_ISR_TSOVF_Pos)               /*!< 0x00001000 */
#define RTC_ISR_TSOVF                RTC_ISR_TSOVF_Msk                         
#define RTC_ISR_TSF_Pos              (11U)                                     
#define RTC_ISR_TSF_Msk              (0x1U << RTC_ISR_TSF_Pos)                 /*!< 0x00000800 */
#define RTC_ISR_TSF                  RTC_ISR_TSF_Msk                           
#define RTC_ISR_WUTF_Pos             (10U)                                     
#define RTC_ISR_WUTF_Msk             (0x1U << RTC_ISR_WUTF_Pos)                /*!< 0x00000400 */
#define RTC_ISR_WUTF                 RTC_ISR_WUTF_Msk                          
#define RTC_ISR_ALRBF_Pos            (9U)                                      
#define RTC_ISR_ALRBF_Msk            (0x1U << RTC_ISR_ALRBF_Pos)               /*!< 0x00000200 */
#define RTC_ISR_ALRBF                RTC_ISR_ALRBF_Msk                         
#define RTC_ISR_ALRAF_Pos            (8U)                                      
#define RTC_ISR_ALRAF_Msk            (0x1U << RTC_ISR_ALRAF_Pos)               /*!< 0x00000100 */
#define RTC_ISR_ALRAF                RTC_ISR_ALRAF_Msk                         
#define RTC_ISR_INIT_Pos             (7U)                                      
#define RTC_ISR_INIT_Msk             (0x1U << RTC_ISR_INIT_Pos)                /*!< 0x00000080 */
#define RTC_ISR_INIT                 RTC_ISR_INIT_Msk                          
#define RTC_ISR_INITF_Pos            (6U)                                      
#define RTC_ISR_INITF_Msk            (0x1U << RTC_ISR_INITF_Pos)               /*!< 0x00000040 */
#define RTC_ISR_INITF                RTC_ISR_INITF_Msk                         
#define RTC_ISR_RSF_Pos              (5U)                                      
#define RTC_ISR_RSF_Msk              (0x1U << RTC_ISR_RSF_Pos)                 /*!< 0x00000020 */
#define RTC_ISR_RSF                  RTC_ISR_RSF_Msk                           
#define RTC_ISR_INITS_Pos            (4U)                                      
#define RTC_ISR_INITS_Msk            (0x1U << RTC_ISR_INITS_Pos)               /*!< 0x00000010 */
#define RTC_ISR_INITS                RTC_ISR_INITS_Msk                         
#define RTC_ISR_SHPF_Pos             (3U)                                      
#define RTC_ISR_SHPF_Msk             (0x1U << RTC_ISR_SHPF_Pos)                /*!< 0x00000008 */
#define RTC_ISR_SHPF                 RTC_ISR_SHPF_Msk                          
#define RTC_ISR_WUTWF_Pos            (2U)                                      
#define RTC_ISR_WUTWF_Msk            (0x1U << RTC_ISR_WUTWF_Pos)               /*!< 0x00000004 */
#define RTC_ISR_WUTWF                RTC_ISR_WUTWF_Msk                         
#define RTC_ISR_ALRBWF_Pos           (1U)                                      
#define RTC_ISR_ALRBWF_Msk           (0x1U << RTC_ISR_ALRBWF_Pos)              /*!< 0x00000002 */
#define RTC_ISR_ALRBWF               RTC_ISR_ALRBWF_Msk                        
#define RTC_ISR_ALRAWF_Pos           (0U)                                      
#define RTC_ISR_ALRAWF_Msk           (0x1U << RTC_ISR_ALRAWF_Pos)              /*!< 0x00000001 */
#define RTC_ISR_ALRAWF               RTC_ISR_ALRAWF_Msk                        

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A_Pos        (16U)                                     
#define RTC_PRER_PREDIV_A_Msk        (0x7FU << RTC_PRER_PREDIV_A_Pos)          /*!< 0x007F0000 */
#define RTC_PRER_PREDIV_A            RTC_PRER_PREDIV_A_Msk                     
#define RTC_PRER_PREDIV_S_Pos        (0U)                                      
#define RTC_PRER_PREDIV_S_Msk        (0x7FFFU << RTC_PRER_PREDIV_S_Pos)        /*!< 0x00007FFF */
#define RTC_PRER_PREDIV_S            RTC_PRER_PREDIV_S_Msk                     

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT_Pos             (0U)                                      
#define RTC_WUTR_WUT_Msk             (0xFFFFU << RTC_WUTR_WUT_Pos)             /*!< 0x0000FFFF */
#define RTC_WUTR_WUT                 RTC_WUTR_WUT_Msk                          

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4_Pos          (31U)                                     
#define RTC_ALRMAR_MSK4_Msk          (0x1U << RTC_ALRMAR_MSK4_Pos)             /*!< 0x80000000 */
#define RTC_ALRMAR_MSK4              RTC_ALRMAR_MSK4_Msk                       
#define RTC_ALRMAR_WDSEL_Pos         (30U)                                     
#define RTC_ALRMAR_WDSEL_Msk         (0x1U << RTC_ALRMAR_WDSEL_Pos)            /*!< 0x40000000 */
#define RTC_ALRMAR_WDSEL             RTC_ALRMAR_WDSEL_Msk                      
#define RTC_ALRMAR_DT_Pos            (28U)                                     
#define RTC_ALRMAR_DT_Msk            (0x3U << RTC_ALRMAR_DT_Pos)               /*!< 0x30000000 */
#define RTC_ALRMAR_DT                RTC_ALRMAR_DT_Msk                         
#define RTC_ALRMAR_DT_0              (0x1U << RTC_ALRMAR_DT_Pos)               /*!< 0x10000000 */
#define RTC_ALRMAR_DT_1              (0x2U << RTC_ALRMAR_DT_Pos)               /*!< 0x20000000 */
#define RTC_ALRMAR_DU_Pos            (24U)                                     
#define RTC_ALRMAR_DU_Msk            (0xFU << RTC_ALRMAR_DU_Pos)               /*!< 0x0F000000 */
#define RTC_ALRMAR_DU                RTC_ALRMAR_DU_Msk                         
#define RTC_ALRMAR_DU_0              (0x1U << RTC_ALRMAR_DU_Pos)               /*!< 0x01000000 */
#define RTC_ALRMAR_DU_1              (0x2U << RTC_ALRMAR_DU_Pos)               /*!< 0x02000000 */
#define RTC_ALRMAR_DU_2              (0x4U << RTC_ALRMAR_DU_Pos)               /*!< 0x04000000 */
#define RTC_ALRMAR_DU_3              (0x8U << RTC_ALRMAR_DU_Pos)               /*!< 0x08000000 */
#define RTC_ALRMAR_MSK3_Pos          (23U)                                     
#define RTC_ALRMAR_MSK3_Msk          (0x1U << RTC_ALRMAR_MSK3_Pos)             /*!< 0x00800000 */
#define RTC_ALRMAR_MSK3              RTC_ALRMAR_MSK3_Msk                       
#define RTC_ALRMAR_PM_Pos            (22U)                                     
#define RTC_ALRMAR_PM_Msk            (0x1U << RTC_ALRMAR_PM_Pos)               /*!< 0x00400000 */
#define RTC_ALRMAR_PM                RTC_ALRMAR_PM_Msk                         
#define RTC_ALRMAR_HT_Pos            (20U)                                     
#define RTC_ALRMAR_HT_Msk            (0x3U << RTC_ALRMAR_HT_Pos)               /*!< 0x00300000 */
#define RTC_ALRMAR_HT                RTC_ALRMAR_HT_Msk                         
#define RTC_ALRMAR_HT_0              (0x1U << RTC_ALRMAR_HT_Pos)               /*!< 0x00100000 */
#define RTC_ALRMAR_HT_1              (0x2U << RTC_ALRMAR_HT_Pos)               /*!< 0x00200000 */
#define RTC_ALRMAR_HU_Pos            (16U)                                     
#define RTC_ALRMAR_HU_Msk            (0xFU << RTC_ALRMAR_HU_Pos)               /*!< 0x000F0000 */
#define RTC_ALRMAR_HU                RTC_ALRMAR_HU_Msk                         
#define RTC_ALRMAR_HU_0              (0x1U << RTC_ALRMAR_HU_Pos)               /*!< 0x00010000 */
#define RTC_ALRMAR_HU_1              (0x2U << RTC_ALRMAR_HU_Pos)               /*!< 0x00020000 */
#define RTC_ALRMAR_HU_2              (0x4U << RTC_ALRMAR_HU_Pos)               /*!< 0x00040000 */
#define RTC_ALRMAR_HU_3              (0x8U << RTC_ALRMAR_HU_Pos)               /*!< 0x00080000 */
#define RTC_ALRMAR_MSK2_Pos          (15U)                                     
#define RTC_ALRMAR_MSK2_Msk          (0x1U << RTC_ALRMAR_MSK2_Pos)             /*!< 0x00008000 */
#define RTC_ALRMAR_MSK2              RTC_ALRMAR_MSK2_Msk                       
#define RTC_ALRMAR_MNT_Pos           (12U)                                     
#define RTC_ALRMAR_MNT_Msk           (0x7U << RTC_ALRMAR_MNT_Pos)              /*!< 0x00007000 */
#define RTC_ALRMAR_MNT               RTC_ALRMAR_MNT_Msk                        
#define RTC_ALRMAR_MNT_0             (0x1U << RTC_ALRMAR_MNT_Pos)              /*!< 0x00001000 */
#define RTC_ALRMAR_MNT_1             (0x2U << RTC_ALRMAR_MNT_Pos)              /*!< 0x00002000 */
#define RTC_ALRMAR_MNT_2             (0x4U << RTC_ALRMAR_MNT_Pos)              /*!< 0x00004000 */
#define RTC_ALRMAR_MNU_Pos           (8U)                                      
#define RTC_ALRMAR_MNU_Msk           (0xFU << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000F00 */
#define RTC_ALRMAR_MNU               RTC_ALRMAR_MNU_Msk                        
#define RTC_ALRMAR_MNU_0             (0x1U << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000100 */
#define RTC_ALRMAR_MNU_1             (0x2U << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000200 */
#define RTC_ALRMAR_MNU_2             (0x4U << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000400 */
#define RTC_ALRMAR_MNU_3             (0x8U << RTC_ALRMAR_MNU_Pos)              /*!< 0x00000800 */
#define RTC_ALRMAR_MSK1_Pos          (7U)                                      
#define RTC_ALRMAR_MSK1_Msk          (0x1U << RTC_ALRMAR_MSK1_Pos)             /*!< 0x00000080 */
#define RTC_ALRMAR_MSK1              RTC_ALRMAR_MSK1_Msk                       
#define RTC_ALRMAR_ST_Pos            (4U)                                      
#define RTC_ALRMAR_ST_Msk            (0x7U << RTC_ALRMAR_ST_Pos)               /*!< 0x00000070 */
#define RTC_ALRMAR_ST                RTC_ALRMAR_ST_Msk                         
#define RTC_ALRMAR_ST_0              (0x1U << RTC_ALRMAR_ST_Pos)               /*!< 0x00000010 */
#define RTC_ALRMAR_ST_1              (0x2U << RTC_ALRMAR_ST_Pos)               /*!< 0x00000020 */
#define RTC_ALRMAR_ST_2              (0x4U << RTC_ALRMAR_ST_Pos)               /*!< 0x00000040 */
#define RTC_ALRMAR_SU_Pos            (0U)                                      
#define RTC_ALRMAR_SU_Msk            (0xFU << RTC_ALRMAR_SU_Pos)               /*!< 0x0000000F */
#define RTC_ALRMAR_SU                RTC_ALRMAR_SU_Msk                         
#define RTC_ALRMAR_SU_0              (0x1U << RTC_ALRMAR_SU_Pos)               /*!< 0x00000001 */
#define RTC_ALRMAR_SU_1              (0x2U << RTC_ALRMAR_SU_Pos)               /*!< 0x00000002 */
#define RTC_ALRMAR_SU_2              (0x4U << RTC_ALRMAR_SU_Pos)               /*!< 0x00000004 */
#define RTC_ALRMAR_SU_3              (0x8U << RTC_ALRMAR_SU_Pos)               /*!< 0x00000008 */

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4_Pos          (31U)                                     
#define RTC_ALRMBR_MSK4_Msk          (0x1U << RTC_ALRMBR_MSK4_Pos)             /*!< 0x80000000 */
#define RTC_ALRMBR_MSK4              RTC_ALRMBR_MSK4_Msk                       
#define RTC_ALRMBR_WDSEL_Pos         (30U)                                     
#define RTC_ALRMBR_WDSEL_Msk         (0x1U << RTC_ALRMBR_WDSEL_Pos)            /*!< 0x40000000 */
#define RTC_ALRMBR_WDSEL             RTC_ALRMBR_WDSEL_Msk                      
#define RTC_ALRMBR_DT_Pos            (28U)                                     
#define RTC_ALRMBR_DT_Msk            (0x3U << RTC_ALRMBR_DT_Pos)               /*!< 0x30000000 */
#define RTC_ALRMBR_DT                RTC_ALRMBR_DT_Msk                         
#define RTC_ALRMBR_DT_0              (0x1U << RTC_ALRMBR_DT_Pos)               /*!< 0x10000000 */
#define RTC_ALRMBR_DT_1              (0x2U << RTC_ALRMBR_DT_Pos)               /*!< 0x20000000 */
#define RTC_ALRMBR_DU_Pos            (24U)                                     
#define RTC_ALRMBR_DU_Msk            (0xFU << RTC_ALRMBR_DU_Pos)               /*!< 0x0F000000 */
#define RTC_ALRMBR_DU                RTC_ALRMBR_DU_Msk                         
#define RTC_ALRMBR_DU_0              (0x1U << RTC_ALRMBR_DU_Pos)               /*!< 0x01000000 */
#define RTC_ALRMBR_DU_1              (0x2U << RTC_ALRMBR_DU_Pos)               /*!< 0x02000000 */
#define RTC_ALRMBR_DU_2              (0x4U << RTC_ALRMBR_DU_Pos)               /*!< 0x04000000 */
#define RTC_ALRMBR_DU_3              (0x8U << RTC_ALRMBR_DU_Pos)               /*!< 0x08000000 */
#define RTC_ALRMBR_MSK3_Pos          (23U)                                     
#define RTC_ALRMBR_MSK3_Msk          (0x1U << RTC_ALRMBR_MSK3_Pos)             /*!< 0x00800000 */
#define RTC_ALRMBR_MSK3              RTC_ALRMBR_MSK3_Msk                       
#define RTC_ALRMBR_PM_Pos            (22U)                                     
#define RTC_ALRMBR_PM_Msk            (0x1U << RTC_ALRMBR_PM_Pos)               /*!< 0x00400000 */
#define RTC_ALRMBR_PM                RTC_ALRMBR_PM_Msk                         
#define RTC_ALRMBR_HT_Pos            (20U)                                     
#define RTC_ALRMBR_HT_Msk            (0x3U << RTC_ALRMBR_HT_Pos)               /*!< 0x00300000 */
#define RTC_ALRMBR_HT                RTC_ALRMBR_HT_Msk                         
#define RTC_ALRMBR_HT_0              (0x1U << RTC_ALRMBR_HT_Pos)               /*!< 0x00100000 */
#define RTC_ALRMBR_HT_1              (0x2U << RTC_ALRMBR_HT_Pos)               /*!< 0x00200000 */
#define RTC_ALRMBR_HU_Pos            (16U)                                     
#define RTC_ALRMBR_HU_Msk            (0xFU << RTC_ALRMBR_HU_Pos)               /*!< 0x000F0000 */
#define RTC_ALRMBR_HU                RTC_ALRMBR_HU_Msk                         
#define RTC_ALRMBR_HU_0              (0x1U << RTC_ALRMBR_HU_Pos)               /*!< 0x00010000 */
#define RTC_ALRMBR_HU_1              (0x2U << RTC_ALRMBR_HU_Pos)               /*!< 0x00020000 */
#define RTC_ALRMBR_HU_2              (0x4U << RTC_ALRMBR_HU_Pos)               /*!< 0x00040000 */
#define RTC_ALRMBR_HU_3              (0x8U << RTC_ALRMBR_HU_Pos)               /*!< 0x00080000 */
#define RTC_ALRMBR_MSK2_Pos          (15U)                                     
#define RTC_ALRMBR_MSK2_Msk          (0x1U << RTC_ALRMBR_MSK2_Pos)             /*!< 0x00008000 */
#define RTC_ALRMBR_MSK2              RTC_ALRMBR_MSK2_Msk                       
#define RTC_ALRMBR_MNT_Pos           (12U)                                     
#define RTC_ALRMBR_MNT_Msk           (0x7U << RTC_ALRMBR_MNT_Pos)              /*!< 0x00007000 */
#define RTC_ALRMBR_MNT               RTC_ALRMBR_MNT_Msk                        
#define RTC_ALRMBR_MNT_0             (0x1U << RTC_ALRMBR_MNT_Pos)              /*!< 0x00001000 */
#define RTC_ALRMBR_MNT_1             (0x2U << RTC_ALRMBR_MNT_Pos)              /*!< 0x00002000 */
#define RTC_ALRMBR_MNT_2             (0x4U << RTC_ALRMBR_MNT_Pos)              /*!< 0x00004000 */
#define RTC_ALRMBR_MNU_Pos           (8U)                                      
#define RTC_ALRMBR_MNU_Msk           (0xFU << RTC_ALRMBR_MNU_Pos)              /*!< 0x00000F00 */
#define RTC_ALRMBR_MNU               RTC_ALRMBR_MNU_Msk                        
#define RTC_ALRMBR_MNU_0             (0x1U << RTC_ALRMBR_MNU_Pos)              /*!< 0x00000100 */
#define RTC_ALRMBR_MNU_1             (0x2U << RTC_ALRMBR_MNU_Pos)              /*!< 0x00000200 */
#define RTC_ALRMBR_MNU_2             (0x4U << RTC_ALRMBR_MNU_Pos)              /*!< 0x00000400 */
#define RTC_ALRMBR_MNU_3             (0x8U << RTC_ALRMBR_MNU_Pos)              /*!< 0x00000800 */
#define RTC_ALRMBR_MSK1_Pos          (7U)                                      
#define RTC_ALRMBR_MSK1_Msk          (0x1U << RTC_ALRMBR_MSK1_Pos)             /*!< 0x00000080 */
#define RTC_ALRMBR_MSK1              RTC_ALRMBR_MSK1_Msk                       
#define RTC_ALRMBR_ST_Pos            (4U)                                      
#define RTC_ALRMBR_ST_Msk            (0x7U << RTC_ALRMBR_ST_Pos)               /*!< 0x00000070 */
#define RTC_ALRMBR_ST                RTC_ALRMBR_ST_Msk                         
#define RTC_ALRMBR_ST_0              (0x1U << RTC_ALRMBR_ST_Pos)               /*!< 0x00000010 */
#define RTC_ALRMBR_ST_1              (0x2U << RTC_ALRMBR_ST_Pos)               /*!< 0x00000020 */
#define RTC_ALRMBR_ST_2              (0x4U << RTC_ALRMBR_ST_Pos)               /*!< 0x00000040 */
#define RTC_ALRMBR_SU_Pos            (0U)                                      
#define RTC_ALRMBR_SU_Msk            (0xFU << RTC_ALRMBR_SU_Pos)               /*!< 0x0000000F */
#define RTC_ALRMBR_SU                RTC_ALRMBR_SU_Msk                         
#define RTC_ALRMBR_SU_0              (0x1U << RTC_ALRMBR_SU_Pos)               /*!< 0x00000001 */
#define RTC_ALRMBR_SU_1              (0x2U << RTC_ALRMBR_SU_Pos)               /*!< 0x00000002 */
#define RTC_ALRMBR_SU_2              (0x4U << RTC_ALRMBR_SU_Pos)               /*!< 0x00000004 */
#define RTC_ALRMBR_SU_3              (0x8U << RTC_ALRMBR_SU_Pos)               /*!< 0x00000008 */

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY_Pos              (0U)                                      
#define RTC_WPR_KEY_Msk              (0xFFU << RTC_WPR_KEY_Pos)                /*!< 0x000000FF */
#define RTC_WPR_KEY                  RTC_WPR_KEY_Msk                           

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS_Pos               (0U)                                      
#define RTC_SSR_SS_Msk               (0xFFFFU << RTC_SSR_SS_Pos)               /*!< 0x0000FFFF */
#define RTC_SSR_SS                   RTC_SSR_SS_Msk                            

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS_Pos         (0U)                                      
#define RTC_SHIFTR_SUBFS_Msk         (0x7FFFU << RTC_SHIFTR_SUBFS_Pos)         /*!< 0x00007FFF */
#define RTC_SHIFTR_SUBFS             RTC_SHIFTR_SUBFS_Msk                      
#define RTC_SHIFTR_ADD1S_Pos         (31U)                                     
#define RTC_SHIFTR_ADD1S_Msk         (0x1U << RTC_SHIFTR_ADD1S_Pos)            /*!< 0x80000000 */
#define RTC_SHIFTR_ADD1S             RTC_SHIFTR_ADD1S_Msk                      

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM_Pos              (22U)                                     
#define RTC_TSTR_PM_Msk              (0x1U << RTC_TSTR_PM_Pos)                 /*!< 0x00400000 */
#define RTC_TSTR_PM                  RTC_TSTR_PM_Msk                           
#define RTC_TSTR_HT_Pos              (20U)                                     
#define RTC_TSTR_HT_Msk              (0x3U << RTC_TSTR_HT_Pos)                 /*!< 0x00300000 */
#define RTC_TSTR_HT                  RTC_TSTR_HT_Msk                           
#define RTC_TSTR_HT_0                (0x1U << RTC_TSTR_HT_Pos)                 /*!< 0x00100000 */
#define RTC_TSTR_HT_1                (0x2U << RTC_TSTR_HT_Pos)                 /*!< 0x00200000 */
#define RTC_TSTR_HU_Pos              (16U)                                     
#define RTC_TSTR_HU_Msk              (0xFU << RTC_TSTR_HU_Pos)                 /*!< 0x000F0000 */
#define RTC_TSTR_HU                  RTC_TSTR_HU_Msk                           
#define RTC_TSTR_HU_0                (0x1U << RTC_TSTR_HU_Pos)                 /*!< 0x00010000 */
#define RTC_TSTR_HU_1                (0x2U << RTC_TSTR_HU_Pos)                 /*!< 0x00020000 */
#define RTC_TSTR_HU_2                (0x4U << RTC_TSTR_HU_Pos)                 /*!< 0x00040000 */
#define RTC_TSTR_HU_3                (0x8U << RTC_TSTR_HU_Pos)                 /*!< 0x00080000 */
#define RTC_TSTR_MNT_Pos             (12U)                                     
#define RTC_TSTR_MNT_Msk             (0x7U << RTC_TSTR_MNT_Pos)                /*!< 0x00007000 */
#define RTC_TSTR_MNT                 RTC_TSTR_MNT_Msk                          
#define RTC_TSTR_MNT_0               (0x1U << RTC_TSTR_MNT_Pos)                /*!< 0x00001000 */
#define RTC_TSTR_MNT_1               (0x2U << RTC_TSTR_MNT_Pos)                /*!< 0x00002000 */
#define RTC_TSTR_MNT_2               (0x4U << RTC_TSTR_MNT_Pos)                /*!< 0x00004000 */
#define RTC_TSTR_MNU_Pos             (8U)                                      
#define RTC_TSTR_MNU_Msk             (0xFU << RTC_TSTR_MNU_Pos)                /*!< 0x00000F00 */
#define RTC_TSTR_MNU                 RTC_TSTR_MNU_Msk                          
#define RTC_TSTR_MNU_0               (0x1U << RTC_TSTR_MNU_Pos)                /*!< 0x00000100 */
#define RTC_TSTR_MNU_1               (0x2U << RTC_TSTR_MNU_Pos)                /*!< 0x00000200 */
#define RTC_TSTR_MNU_2               (0x4U << RTC_TSTR_MNU_Pos)                /*!< 0x00000400 */
#define RTC_TSTR_MNU_3               (0x8U << RTC_TSTR_MNU_Pos)                /*!< 0x00000800 */
#define RTC_TSTR_ST_Pos              (4U)                                      
#define RTC_TSTR_ST_Msk              (0x7U << RTC_TSTR_ST_Pos)                 /*!< 0x00000070 */
#define RTC_TSTR_ST                  RTC_TSTR_ST_Msk                           
#define RTC_TSTR_ST_0                (0x1U << RTC_TSTR_ST_Pos)                 /*!< 0x00000010 */
#define RTC_TSTR_ST_1                (0x2U << RTC_TSTR_ST_Pos)                 /*!< 0x00000020 */
#define RTC_TSTR_ST_2                (0x4U << RTC_TSTR_ST_Pos)                 /*!< 0x00000040 */
#define RTC_TSTR_SU_Pos              (0U)                                      
#define RTC_TSTR_SU_Msk              (0xFU << RTC_TSTR_SU_Pos)                 /*!< 0x0000000F */
#define RTC_TSTR_SU                  RTC_TSTR_SU_Msk                           
#define RTC_TSTR_SU_0                (0x1U << RTC_TSTR_SU_Pos)                 /*!< 0x00000001 */
#define RTC_TSTR_SU_1                (0x2U << RTC_TSTR_SU_Pos)                 /*!< 0x00000002 */
#define RTC_TSTR_SU_2                (0x4U << RTC_TSTR_SU_Pos)                 /*!< 0x00000004 */
#define RTC_TSTR_SU_3                (0x8U << RTC_TSTR_SU_Pos)                 /*!< 0x00000008 */

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU_Pos             (13U)                                     
#define RTC_TSDR_WDU_Msk             (0x7U << RTC_TSDR_WDU_Pos)                /*!< 0x0000E000 */
#define RTC_TSDR_WDU                 RTC_TSDR_WDU_Msk                          
#define RTC_TSDR_WDU_0               (0x1U << RTC_TSDR_WDU_Pos)                /*!< 0x00002000 */
#define RTC_TSDR_WDU_1               (0x2U << RTC_TSDR_WDU_Pos)                /*!< 0x00004000 */
#define RTC_TSDR_WDU_2               (0x4U << RTC_TSDR_WDU_Pos)                /*!< 0x00008000 */
#define RTC_TSDR_MT_Pos              (12U)                                     
#define RTC_TSDR_MT_Msk              (0x1U << RTC_TSDR_MT_Pos)                 /*!< 0x00001000 */
#define RTC_TSDR_MT                  RTC_TSDR_MT_Msk                           
#define RTC_TSDR_MU_Pos              (8U)                                      
#define RTC_TSDR_MU_Msk              (0xFU << RTC_TSDR_MU_Pos)                 /*!< 0x00000F00 */
#define RTC_TSDR_MU                  RTC_TSDR_MU_Msk                           
#define RTC_TSDR_MU_0                (0x1U << RTC_TSDR_MU_Pos)                 /*!< 0x00000100 */
#define RTC_TSDR_MU_1                (0x2U << RTC_TSDR_MU_Pos)                 /*!< 0x00000200 */
#define RTC_TSDR_MU_2                (0x4U << RTC_TSDR_MU_Pos)                 /*!< 0x00000400 */
#define RTC_TSDR_MU_3                (0x8U << RTC_TSDR_MU_Pos)                 /*!< 0x00000800 */
#define RTC_TSDR_DT_Pos              (4U)                                      
#define RTC_TSDR_DT_Msk              (0x3U << RTC_TSDR_DT_Pos)                 /*!< 0x00000030 */
#define RTC_TSDR_DT                  RTC_TSDR_DT_Msk                           
#define RTC_TSDR_DT_0                (0x1U << RTC_TSDR_DT_Pos)                 /*!< 0x00000010 */
#define RTC_TSDR_DT_1                (0x2U << RTC_TSDR_DT_Pos)                 /*!< 0x00000020 */
#define RTC_TSDR_DU_Pos              (0U)                                      
#define RTC_TSDR_DU_Msk              (0xFU << RTC_TSDR_DU_Pos)                 /*!< 0x0000000F */
#define RTC_TSDR_DU                  RTC_TSDR_DU_Msk                           
#define RTC_TSDR_DU_0                (0x1U << RTC_TSDR_DU_Pos)                 /*!< 0x00000001 */
#define RTC_TSDR_DU_1                (0x2U << RTC_TSDR_DU_Pos)                 /*!< 0x00000002 */
#define RTC_TSDR_DU_2                (0x4U << RTC_TSDR_DU_Pos)                 /*!< 0x00000004 */
#define RTC_TSDR_DU_3                (0x8U << RTC_TSDR_DU_Pos)                 /*!< 0x00000008 */

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS_Pos             (0U)                                      
#define RTC_TSSSR_SS_Msk             (0xFFFFU << RTC_TSSSR_SS_Pos)             /*!< 0x0000FFFF */
#define RTC_TSSSR_SS                 RTC_TSSSR_SS_Msk                          

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP_Pos            (15U)                                     
#define RTC_CALR_CALP_Msk            (0x1U << RTC_CALR_CALP_Pos)               /*!< 0x00008000 */
#define RTC_CALR_CALP                RTC_CALR_CALP_Msk                         
#define RTC_CALR_CALW8_Pos           (14U)                                     
#define RTC_CALR_CALW8_Msk           (0x1U << RTC_CALR_CALW8_Pos)              /*!< 0x00004000 */
#define RTC_CALR_CALW8               RTC_CALR_CALW8_Msk                        
#define RTC_CALR_CALW16_Pos          (13U)                                     
#define RTC_CALR_CALW16_Msk          (0x1U << RTC_CALR_CALW16_Pos)             /*!< 0x00002000 */
#define RTC_CALR_CALW16              RTC_CALR_CALW16_Msk                       
#define RTC_CALR_CALM_Pos            (0U)                                      
#define RTC_CALR_CALM_Msk            (0x1FFU << RTC_CALR_CALM_Pos)             /*!< 0x000001FF */
#define RTC_CALR_CALM                RTC_CALR_CALM_Msk                         
#define RTC_CALR_CALM_0              (0x001U << RTC_CALR_CALM_Pos)             /*!< 0x00000001 */
#define RTC_CALR_CALM_1              (0x002U << RTC_CALR_CALM_Pos)             /*!< 0x00000002 */
#define RTC_CALR_CALM_2              (0x004U << RTC_CALR_CALM_Pos)             /*!< 0x00000004 */
#define RTC_CALR_CALM_3              (0x008U << RTC_CALR_CALM_Pos)             /*!< 0x00000008 */
#define RTC_CALR_CALM_4              (0x010U << RTC_CALR_CALM_Pos)             /*!< 0x00000010 */
#define RTC_CALR_CALM_5              (0x020U << RTC_CALR_CALM_Pos)             /*!< 0x00000020 */
#define RTC_CALR_CALM_6              (0x040U << RTC_CALR_CALM_Pos)             /*!< 0x00000040 */
#define RTC_CALR_CALM_7              (0x080U << RTC_CALR_CALM_Pos)             /*!< 0x00000080 */
#define RTC_CALR_CALM_8              (0x100U << RTC_CALR_CALM_Pos)             /*!< 0x00000100 */

/********************  Bits definition for RTC_TAFCR register  ****************/
#define RTC_TAFCR_PC15MODE_Pos       (23U)                                     
#define RTC_TAFCR_PC15MODE_Msk       (0x1U << RTC_TAFCR_PC15MODE_Pos)          /*!< 0x00800000 */
#define RTC_TAFCR_PC15MODE           RTC_TAFCR_PC15MODE_Msk                    
#define RTC_TAFCR_PC15VALUE_Pos      (22U)                                     
#define RTC_TAFCR_PC15VALUE_Msk      (0x1U << RTC_TAFCR_PC15VALUE_Pos)         /*!< 0x00400000 */
#define RTC_TAFCR_PC15VALUE          RTC_TAFCR_PC15VALUE_Msk                   
#define RTC_TAFCR_PC14MODE_Pos       (21U)                                     
#define RTC_TAFCR_PC14MODE_Msk       (0x1U << RTC_TAFCR_PC14MODE_Pos)          /*!< 0x00200000 */
#define RTC_TAFCR_PC14MODE           RTC_TAFCR_PC14MODE_Msk                    
#define RTC_TAFCR_PC14VALUE_Pos      (20U)                                     
#define RTC_TAFCR_PC14VALUE_Msk      (0x1U << RTC_TAFCR_PC14VALUE_Pos)         /*!< 0x00100000 */
#define RTC_TAFCR_PC14VALUE          RTC_TAFCR_PC14VALUE_Msk                   
#define RTC_TAFCR_PC13MODE_Pos       (19U)                                     
#define RTC_TAFCR_PC13MODE_Msk       (0x1U << RTC_TAFCR_PC13MODE_Pos)          /*!< 0x00080000 */
#define RTC_TAFCR_PC13MODE           RTC_TAFCR_PC13MODE_Msk                    
#define RTC_TAFCR_PC13VALUE_Pos      (18U)                                     
#define RTC_TAFCR_PC13VALUE_Msk      (0x1U << RTC_TAFCR_PC13VALUE_Pos)         /*!< 0x00040000 */
#define RTC_TAFCR_PC13VALUE          RTC_TAFCR_PC13VALUE_Msk                   
#define RTC_TAFCR_TAMPPUDIS_Pos      (15U)                                     
#define RTC_TAFCR_TAMPPUDIS_Msk      (0x1U << RTC_TAFCR_TAMPPUDIS_Pos)         /*!< 0x00008000 */
#define RTC_TAFCR_TAMPPUDIS          RTC_TAFCR_TAMPPUDIS_Msk                   
#define RTC_TAFCR_TAMPPRCH_Pos       (13U)                                     
#define RTC_TAFCR_TAMPPRCH_Msk       (0x3U << RTC_TAFCR_TAMPPRCH_Pos)          /*!< 0x00006000 */
#define RTC_TAFCR_TAMPPRCH           RTC_TAFCR_TAMPPRCH_Msk                    
#define RTC_TAFCR_TAMPPRCH_0         (0x1U << RTC_TAFCR_TAMPPRCH_Pos)          /*!< 0x00002000 */
#define RTC_TAFCR_TAMPPRCH_1         (0x2U << RTC_TAFCR_TAMPPRCH_Pos)          /*!< 0x00004000 */
#define RTC_TAFCR_TAMPFLT_Pos        (11U)                                     
#define RTC_TAFCR_TAMPFLT_Msk        (0x3U << RTC_TAFCR_TAMPFLT_Pos)           /*!< 0x00001800 */
#define RTC_TAFCR_TAMPFLT            RTC_TAFCR_TAMPFLT_Msk                     
#define RTC_TAFCR_TAMPFLT_0          (0x1U << RTC_TAFCR_TAMPFLT_Pos)           /*!< 0x00000800 */
#define RTC_TAFCR_TAMPFLT_1          (0x2U << RTC_TAFCR_TAMPFLT_Pos)           /*!< 0x00001000 */
#define RTC_TAFCR_TAMPFREQ_Pos       (8U)                                      
#define RTC_TAFCR_TAMPFREQ_Msk       (0x7U << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000700 */
#define RTC_TAFCR_TAMPFREQ           RTC_TAFCR_TAMPFREQ_Msk                    
#define RTC_TAFCR_TAMPFREQ_0         (0x1U << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000100 */
#define RTC_TAFCR_TAMPFREQ_1         (0x2U << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000200 */
#define RTC_TAFCR_TAMPFREQ_2         (0x4U << RTC_TAFCR_TAMPFREQ_Pos)          /*!< 0x00000400 */
#define RTC_TAFCR_TAMPTS_Pos         (7U)                                      
#define RTC_TAFCR_TAMPTS_Msk         (0x1U << RTC_TAFCR_TAMPTS_Pos)            /*!< 0x00000080 */
#define RTC_TAFCR_TAMPTS             RTC_TAFCR_TAMPTS_Msk                      
#define RTC_TAFCR_TAMP2TRG_Pos       (4U)                                      
#define RTC_TAFCR_TAMP2TRG_Msk       (0x1U << RTC_TAFCR_TAMP2TRG_Pos)          /*!< 0x00000010 */
#define RTC_TAFCR_TAMP2TRG           RTC_TAFCR_TAMP2TRG_Msk                    
#define RTC_TAFCR_TAMP2E_Pos         (3U)                                      
#define RTC_TAFCR_TAMP2E_Msk         (0x1U << RTC_TAFCR_TAMP2E_Pos)            /*!< 0x00000008 */
#define RTC_TAFCR_TAMP2E             RTC_TAFCR_TAMP2E_Msk                      
#define RTC_TAFCR_TAMPIE_Pos         (2U)                                      
#define RTC_TAFCR_TAMPIE_Msk         (0x1U << RTC_TAFCR_TAMPIE_Pos)            /*!< 0x00000004 */
#define RTC_TAFCR_TAMPIE             RTC_TAFCR_TAMPIE_Msk                      
#define RTC_TAFCR_TAMP1TRG_Pos       (1U)                                      
#define RTC_TAFCR_TAMP1TRG_Msk       (0x1U << RTC_TAFCR_TAMP1TRG_Pos)          /*!< 0x00000002 */
#define RTC_TAFCR_TAMP1TRG           RTC_TAFCR_TAMP1TRG_Msk                    
#define RTC_TAFCR_TAMP1E_Pos         (0U)                                      
#define RTC_TAFCR_TAMP1E_Msk         (0x1U << RTC_TAFCR_TAMP1E_Pos)            /*!< 0x00000001 */
#define RTC_TAFCR_TAMP1E             RTC_TAFCR_TAMP1E_Msk                      

/* Reference defines */
#define RTC_TAFCR_ALARMOUTTYPE               RTC_TAFCR_PC13VALUE

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS_Pos      (24U)                                     
#define RTC_ALRMASSR_MASKSS_Msk      (0xFU << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x0F000000 */
#define RTC_ALRMASSR_MASKSS          RTC_ALRMASSR_MASKSS_Msk                   
#define RTC_ALRMASSR_MASKSS_0        (0x1U << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x01000000 */
#define RTC_ALRMASSR_MASKSS_1        (0x2U << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x02000000 */
#define RTC_ALRMASSR_MASKSS_2        (0x4U << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x04000000 */
#define RTC_ALRMASSR_MASKSS_3        (0x8U << RTC_ALRMASSR_MASKSS_Pos)         /*!< 0x08000000 */
#define RTC_ALRMASSR_SS_Pos          (0U)                                      
#define RTC_ALRMASSR_SS_Msk          (0x7FFFU << RTC_ALRMASSR_SS_Pos)          /*!< 0x00007FFF */
#define RTC_ALRMASSR_SS              RTC_ALRMASSR_SS_Msk                       

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS_Pos      (24U)                                     
#define RTC_ALRMBSSR_MASKSS_Msk      (0xFU << RTC_ALRMBSSR_MASKSS_Pos)         /*!< 0x0F000000 */
#define RTC_ALRMBSSR_MASKSS          RTC_ALRMBSSR_MASKSS_Msk                   
#define RTC_ALRMBSSR_MASKSS_0        (0x1U << RTC_ALRMBSSR_MASKSS_Pos)         /*!< 0x01000000 */
#define RTC_ALRMBSSR_MASKSS_1        (0x2U << RTC_ALRMBSSR_MASKSS_Pos)         /*!< 0x02000000 */
#define RTC_ALRMBSSR_MASKSS_2        (0x4U << RTC_ALRMBSSR_MASKSS_Pos)         /*!< 0x04000000 */
#define RTC_ALRMBSSR_MASKSS_3        (0x8U << RTC_ALRMBSSR_MASKSS_Pos)         /*!< 0x08000000 */
#define RTC_ALRMBSSR_SS_Pos          (0U)                                      
#define RTC_ALRMBSSR_SS_Msk          (0x7FFFU << RTC_ALRMBSSR_SS_Pos)          /*!< 0x00007FFF */
#define RTC_ALRMBSSR_SS              RTC_ALRMBSSR_SS_Msk                       

/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R_Pos                (0U)                                      
#define RTC_BKP0R_Msk                (0xFFFFFFFFU << RTC_BKP0R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP0R                    RTC_BKP0R_Msk                             

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R_Pos                (0U)                                      
#define RTC_BKP1R_Msk                (0xFFFFFFFFU << RTC_BKP1R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP1R                    RTC_BKP1R_Msk                             

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R_Pos                (0U)                                      
#define RTC_BKP2R_Msk                (0xFFFFFFFFU << RTC_BKP2R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP2R                    RTC_BKP2R_Msk                             

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R_Pos                (0U)                                      
#define RTC_BKP3R_Msk                (0xFFFFFFFFU << RTC_BKP3R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP3R                    RTC_BKP3R_Msk                             

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R_Pos                (0U)                                      
#define RTC_BKP4R_Msk                (0xFFFFFFFFU << RTC_BKP4R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP4R                    RTC_BKP4R_Msk                             

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R_Pos                (0U)                                      
#define RTC_BKP5R_Msk                (0xFFFFFFFFU << RTC_BKP5R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP5R                    RTC_BKP5R_Msk                             

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R_Pos                (0U)                                      
#define RTC_BKP6R_Msk                (0xFFFFFFFFU << RTC_BKP6R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP6R                    RTC_BKP6R_Msk                             

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R_Pos                (0U)                                      
#define RTC_BKP7R_Msk                (0xFFFFFFFFU << RTC_BKP7R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP7R                    RTC_BKP7R_Msk                             

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R_Pos                (0U)                                      
#define RTC_BKP8R_Msk                (0xFFFFFFFFU << RTC_BKP8R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP8R                    RTC_BKP8R_Msk                             

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R_Pos                (0U)                                      
#define RTC_BKP9R_Msk                (0xFFFFFFFFU << RTC_BKP9R_Pos)            /*!< 0xFFFFFFFF */
#define RTC_BKP9R                    RTC_BKP9R_Msk                             

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R_Pos               (0U)                                      
#define RTC_BKP10R_Msk               (0xFFFFFFFFU << RTC_BKP10R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP10R                   RTC_BKP10R_Msk                            

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R_Pos               (0U)                                      
#define RTC_BKP11R_Msk               (0xFFFFFFFFU << RTC_BKP11R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP11R                   RTC_BKP11R_Msk                            

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R_Pos               (0U)                                      
#define RTC_BKP12R_Msk               (0xFFFFFFFFU << RTC_BKP12R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP12R                   RTC_BKP12R_Msk                            

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R_Pos               (0U)                                      
#define RTC_BKP13R_Msk               (0xFFFFFFFFU << RTC_BKP13R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP13R                   RTC_BKP13R_Msk                            

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R_Pos               (0U)                                      
#define RTC_BKP14R_Msk               (0xFFFFFFFFU << RTC_BKP14R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP14R                   RTC_BKP14R_Msk                            

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R_Pos               (0U)                                      
#define RTC_BKP15R_Msk               (0xFFFFFFFFU << RTC_BKP15R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP15R                   RTC_BKP15R_Msk                            

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R_Pos               (0U)                                      
#define RTC_BKP16R_Msk               (0xFFFFFFFFU << RTC_BKP16R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP16R                   RTC_BKP16R_Msk                            

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R_Pos               (0U)                                      
#define RTC_BKP17R_Msk               (0xFFFFFFFFU << RTC_BKP17R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP17R                   RTC_BKP17R_Msk                            

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R_Pos               (0U)                                      
#define RTC_BKP18R_Msk               (0xFFFFFFFFU << RTC_BKP18R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP18R                   RTC_BKP18R_Msk                            

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R_Pos               (0U)                                      
#define RTC_BKP19R_Msk               (0xFFFFFFFFU << RTC_BKP19R_Pos)           /*!< 0xFFFFFFFF */
#define RTC_BKP19R                   RTC_BKP19R_Msk                            

/******************** Number of backup registers ******************************/
#define RTC_BKP_NUMBER                       20

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface (SPI)                   */
/*                                                                            */
/******************************************************************************/

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
 */
#define SPI_I2S_SUPPORT                       /*!< I2S support */
#define SPI_I2S_FULLDUPLEX_SUPPORT            /*!< I2S Full-Duplex support */

/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA_Pos            (0U)                                       
#define SPI_CR1_CPHA_Msk            (0x1U << SPI_CR1_CPHA_Pos)                 /*!< 0x00000001 */
#define SPI_CR1_CPHA                SPI_CR1_CPHA_Msk                           /*!< Clock Phase */
#define SPI_CR1_CPOL_Pos            (1U)                                       
#define SPI_CR1_CPOL_Msk            (0x1U << SPI_CR1_CPOL_Pos)                 /*!< 0x00000002 */
#define SPI_CR1_CPOL                SPI_CR1_CPOL_Msk                           /*!< Clock Polarity */
#define SPI_CR1_MSTR_Pos            (2U)                                       
#define SPI_CR1_MSTR_Msk            (0x1U << SPI_CR1_MSTR_Pos)                 /*!< 0x00000004 */
#define SPI_CR1_MSTR                SPI_CR1_MSTR_Msk                           /*!< Master Selection */
#define SPI_CR1_BR_Pos              (3U)                                       
#define SPI_CR1_BR_Msk              (0x7U << SPI_CR1_BR_Pos)                   /*!< 0x00000038 */
#define SPI_CR1_BR                  SPI_CR1_BR_Msk                             /*!< BR[2:0] bits (Baud Rate Control) */
#define SPI_CR1_BR_0                (0x1U << SPI_CR1_BR_Pos)                   /*!< 0x00000008 */
#define SPI_CR1_BR_1                (0x2U << SPI_CR1_BR_Pos)                   /*!< 0x00000010 */
#define SPI_CR1_BR_2                (0x4U << SPI_CR1_BR_Pos)                   /*!< 0x00000020 */
#define SPI_CR1_SPE_Pos             (6U)                                       
#define SPI_CR1_SPE_Msk             (0x1U << SPI_CR1_SPE_Pos)                  /*!< 0x00000040 */
#define SPI_CR1_SPE                 SPI_CR1_SPE_Msk                            /*!< SPI Enable */
#define SPI_CR1_LSBFIRST_Pos        (7U)                                       
#define SPI_CR1_LSBFIRST_Msk        (0x1U << SPI_CR1_LSBFIRST_Pos)             /*!< 0x00000080 */
#define SPI_CR1_LSBFIRST            SPI_CR1_LSBFIRST_Msk                       /*!< Frame Format */
#define SPI_CR1_SSI_Pos             (8U)                                       
#define SPI_CR1_SSI_Msk             (0x1U << SPI_CR1_SSI_Pos)                  /*!< 0x00000100 */
#define SPI_CR1_SSI                 SPI_CR1_SSI_Msk                            /*!< Internal slave select */
#define SPI_CR1_SSM_Pos             (9U)                                       
#define SPI_CR1_SSM_Msk             (0x1U << SPI_CR1_SSM_Pos)                  /*!< 0x00000200 */
#define SPI_CR1_SSM                 SPI_CR1_SSM_Msk                            /*!< Software slave management */
#define SPI_CR1_RXONLY_Pos          (10U)                                      
#define SPI_CR1_RXONLY_Msk          (0x1U << SPI_CR1_RXONLY_Pos)               /*!< 0x00000400 */
#define SPI_CR1_RXONLY              SPI_CR1_RXONLY_Msk                         /*!< Receive only */
#define SPI_CR1_CRCL_Pos            (11U)                                      
#define SPI_CR1_CRCL_Msk            (0x1U << SPI_CR1_CRCL_Pos)                 /*!< 0x00000800 */
#define SPI_CR1_CRCL                SPI_CR1_CRCL_Msk                           /*!< CRC Length */
#define SPI_CR1_CRCNEXT_Pos         (12U)                                      
#define SPI_CR1_CRCNEXT_Msk         (0x1U << SPI_CR1_CRCNEXT_Pos)              /*!< 0x00001000 */
#define SPI_CR1_CRCNEXT             SPI_CR1_CRCNEXT_Msk                        /*!< Transmit CRC next */
#define SPI_CR1_CRCEN_Pos           (13U)                                      
#define SPI_CR1_CRCEN_Msk           (0x1U << SPI_CR1_CRCEN_Pos)                /*!< 0x00002000 */
#define SPI_CR1_CRCEN               SPI_CR1_CRCEN_Msk                          /*!< Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE_Pos          (14U)                                      
#define SPI_CR1_BIDIOE_Msk          (0x1U << SPI_CR1_BIDIOE_Pos)               /*!< 0x00004000 */
#define SPI_CR1_BIDIOE              SPI_CR1_BIDIOE_Msk                         /*!< Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE_Pos        (15U)                                      
#define SPI_CR1_BIDIMODE_Msk        (0x1U << SPI_CR1_BIDIMODE_Pos)             /*!< 0x00008000 */
#define SPI_CR1_BIDIMODE            SPI_CR1_BIDIMODE_Msk                       /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN_Pos         (0U)                                       
#define SPI_CR2_RXDMAEN_Msk         (0x1U << SPI_CR2_RXDMAEN_Pos)              /*!< 0x00000001 */
#define SPI_CR2_RXDMAEN             SPI_CR2_RXDMAEN_Msk                        /*!< Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN_Pos         (1U)                                       
#define SPI_CR2_TXDMAEN_Msk         (0x1U << SPI_CR2_TXDMAEN_Pos)              /*!< 0x00000002 */
#define SPI_CR2_TXDMAEN             SPI_CR2_TXDMAEN_Msk                        /*!< Tx Buffer DMA Enable */
#define SPI_CR2_SSOE_Pos            (2U)                                       
#define SPI_CR2_SSOE_Msk            (0x1U << SPI_CR2_SSOE_Pos)                 /*!< 0x00000004 */
#define SPI_CR2_SSOE                SPI_CR2_SSOE_Msk                           /*!< SS Output Enable */
#define SPI_CR2_NSSP_Pos            (3U)                                       
#define SPI_CR2_NSSP_Msk            (0x1U << SPI_CR2_NSSP_Pos)                 /*!< 0x00000008 */
#define SPI_CR2_NSSP                SPI_CR2_NSSP_Msk                           /*!< NSS pulse management Enable */
#define SPI_CR2_FRF_Pos             (4U)                                       
#define SPI_CR2_FRF_Msk             (0x1U << SPI_CR2_FRF_Pos)                  /*!< 0x00000010 */
#define SPI_CR2_FRF                 SPI_CR2_FRF_Msk                            /*!< Frame Format Enable */
#define SPI_CR2_ERRIE_Pos           (5U)                                       
#define SPI_CR2_ERRIE_Msk           (0x1U << SPI_CR2_ERRIE_Pos)                /*!< 0x00000020 */
#define SPI_CR2_ERRIE               SPI_CR2_ERRIE_Msk                          /*!< Error Interrupt Enable */
#define SPI_CR2_RXNEIE_Pos          (6U)                                       
#define SPI_CR2_RXNEIE_Msk          (0x1U << SPI_CR2_RXNEIE_Pos)               /*!< 0x00000040 */
#define SPI_CR2_RXNEIE              SPI_CR2_RXNEIE_Msk                         /*!< RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE_Pos           (7U)                                       
#define SPI_CR2_TXEIE_Msk           (0x1U << SPI_CR2_TXEIE_Pos)                /*!< 0x00000080 */
#define SPI_CR2_TXEIE               SPI_CR2_TXEIE_Msk                          /*!< Tx buffer Empty Interrupt Enable */
#define SPI_CR2_DS_Pos              (8U)                                       
#define SPI_CR2_DS_Msk              (0xFU << SPI_CR2_DS_Pos)                   /*!< 0x00000F00 */
#define SPI_CR2_DS                  SPI_CR2_DS_Msk                             /*!< DS[3:0] Data Size */
#define SPI_CR2_DS_0                (0x1U << SPI_CR2_DS_Pos)                   /*!< 0x00000100 */
#define SPI_CR2_DS_1                (0x2U << SPI_CR2_DS_Pos)                   /*!< 0x00000200 */
#define SPI_CR2_DS_2                (0x4U << SPI_CR2_DS_Pos)                   /*!< 0x00000400 */
#define SPI_CR2_DS_3                (0x8U << SPI_CR2_DS_Pos)                   /*!< 0x00000800 */
#define SPI_CR2_FRXTH_Pos           (12U)                                      
#define SPI_CR2_FRXTH_Msk           (0x1U << SPI_CR2_FRXTH_Pos)                /*!< 0x00001000 */
#define SPI_CR2_FRXTH               SPI_CR2_FRXTH_Msk                          /*!< FIFO reception Threshold */
#define SPI_CR2_LDMARX_Pos          (13U)                                      
#define SPI_CR2_LDMARX_Msk          (0x1U << SPI_CR2_LDMARX_Pos)               /*!< 0x00002000 */
#define SPI_CR2_LDMARX              SPI_CR2_LDMARX_Msk                         /*!< Last DMA transfer for reception */
#define SPI_CR2_LDMATX_Pos          (14U)                                      
#define SPI_CR2_LDMATX_Msk          (0x1U << SPI_CR2_LDMATX_Pos)               /*!< 0x00004000 */
#define SPI_CR2_LDMATX              SPI_CR2_LDMATX_Msk                         /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE_Pos             (0U)                                       
#define SPI_SR_RXNE_Msk             (0x1U << SPI_SR_RXNE_Pos)                  /*!< 0x00000001 */
#define SPI_SR_RXNE                 SPI_SR_RXNE_Msk                            /*!< Receive buffer Not Empty */
#define SPI_SR_TXE_Pos              (1U)                                       
#define SPI_SR_TXE_Msk              (0x1U << SPI_SR_TXE_Pos)                   /*!< 0x00000002 */
#define SPI_SR_TXE                  SPI_SR_TXE_Msk                             /*!< Transmit buffer Empty */
#define SPI_SR_CHSIDE_Pos           (2U)                                       
#define SPI_SR_CHSIDE_Msk           (0x1U << SPI_SR_CHSIDE_Pos)                /*!< 0x00000004 */
#define SPI_SR_CHSIDE               SPI_SR_CHSIDE_Msk                          /*!< Channel side */
#define SPI_SR_UDR_Pos              (3U)                                       
#define SPI_SR_UDR_Msk              (0x1U << SPI_SR_UDR_Pos)                   /*!< 0x00000008 */
#define SPI_SR_UDR                  SPI_SR_UDR_Msk                             /*!< Underrun flag */
#define SPI_SR_CRCERR_Pos           (4U)                                       
#define SPI_SR_CRCERR_Msk           (0x1U << SPI_SR_CRCERR_Pos)                /*!< 0x00000010 */
#define SPI_SR_CRCERR               SPI_SR_CRCERR_Msk                          /*!< CRC Error flag */
#define SPI_SR_MODF_Pos             (5U)                                       
#define SPI_SR_MODF_Msk             (0x1U << SPI_SR_MODF_Pos)                  /*!< 0x00000020 */
#define SPI_SR_MODF                 SPI_SR_MODF_Msk                            /*!< Mode fault */
#define SPI_SR_OVR_Pos              (6U)                                       
#define SPI_SR_OVR_Msk              (0x1U << SPI_SR_OVR_Pos)                   /*!< 0x00000040 */
#define SPI_SR_OVR                  SPI_SR_OVR_Msk                             /*!< Overrun flag */
#define SPI_SR_BSY_Pos              (7U)                                       
#define SPI_SR_BSY_Msk              (0x1U << SPI_SR_BSY_Pos)                   /*!< 0x00000080 */
#define SPI_SR_BSY                  SPI_SR_BSY_Msk                             /*!< Busy flag */
#define SPI_SR_FRE_Pos              (8U)                                       
#define SPI_SR_FRE_Msk              (0x1U << SPI_SR_FRE_Pos)                   /*!< 0x00000100 */
#define SPI_SR_FRE                  SPI_SR_FRE_Msk                             /*!< TI frame format error */
#define SPI_SR_FRLVL_Pos            (9U)                                       
#define SPI_SR_FRLVL_Msk            (0x3U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000600 */
#define SPI_SR_FRLVL                SPI_SR_FRLVL_Msk                           /*!< FIFO Reception Level */
#define SPI_SR_FRLVL_0              (0x1U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000200 */
#define SPI_SR_FRLVL_1              (0x2U << SPI_SR_FRLVL_Pos)                 /*!< 0x00000400 */
#define SPI_SR_FTLVL_Pos            (11U)                                      
#define SPI_SR_FTLVL_Msk            (0x3U << SPI_SR_FTLVL_Pos)                 /*!< 0x00001800 */
#define SPI_SR_FTLVL                SPI_SR_FTLVL_Msk                           /*!< FIFO Transmission Level */
#define SPI_SR_FTLVL_0              (0x1U << SPI_SR_FTLVL_Pos)                 /*!< 0x00000800 */
#define SPI_SR_FTLVL_1              (0x2U << SPI_SR_FTLVL_Pos)                 /*!< 0x00001000 */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR_Pos               (0U)                                       
#define SPI_DR_DR_Msk               (0xFFFFU << SPI_DR_DR_Pos)                 /*!< 0x0000FFFF */
#define SPI_DR_DR                   SPI_DR_DR_Msk                              /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY_Pos       (0U)                                       
#define SPI_CRCPR_CRCPOLY_Msk       (0xFFFFU << SPI_CRCPR_CRCPOLY_Pos)         /*!< 0x0000FFFF */
#define SPI_CRCPR_CRCPOLY           SPI_CRCPR_CRCPOLY_Msk                      /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC_Pos        (0U)                                       
#define SPI_RXCRCR_RXCRC_Msk        (0xFFFFU << SPI_RXCRCR_RXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_RXCRCR_RXCRC            SPI_RXCRCR_RXCRC_Msk                       /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC_Pos        (0U)                                       
#define SPI_TXCRCR_TXCRC_Msk        (0xFFFFU << SPI_TXCRCR_TXCRC_Pos)          /*!< 0x0000FFFF */
#define SPI_TXCRCR_TXCRC            SPI_TXCRCR_TXCRC_Msk                       /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define SPI_I2SCFGR_CHLEN_Pos       (0U)                                       
#define SPI_I2SCFGR_CHLEN_Msk       (0x1U << SPI_I2SCFGR_CHLEN_Pos)            /*!< 0x00000001 */
#define SPI_I2SCFGR_CHLEN           SPI_I2SCFGR_CHLEN_Msk                      /*!<Channel length (number of bits per audio channel) */
#define SPI_I2SCFGR_DATLEN_Pos      (1U)                                       
#define SPI_I2SCFGR_DATLEN_Msk      (0x3U << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000006 */
#define SPI_I2SCFGR_DATLEN          SPI_I2SCFGR_DATLEN_Msk                     /*!<DATLEN[1:0] bits (Data length to be transferred) */
#define SPI_I2SCFGR_DATLEN_0        (0x1U << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000002 */
#define SPI_I2SCFGR_DATLEN_1        (0x2U << SPI_I2SCFGR_DATLEN_Pos)           /*!< 0x00000004 */
#define SPI_I2SCFGR_CKPOL_Pos       (3U)                                       
#define SPI_I2SCFGR_CKPOL_Msk       (0x1U << SPI_I2SCFGR_CKPOL_Pos)            /*!< 0x00000008 */
#define SPI_I2SCFGR_CKPOL           SPI_I2SCFGR_CKPOL_Msk                      /*!<steady state clock polarity */
#define SPI_I2SCFGR_I2SSTD_Pos      (4U)                                       
#define SPI_I2SCFGR_I2SSTD_Msk      (0x3U << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000030 */
#define SPI_I2SCFGR_I2SSTD          SPI_I2SCFGR_I2SSTD_Msk                     /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define SPI_I2SCFGR_I2SSTD_0        (0x1U << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000010 */
#define SPI_I2SCFGR_I2SSTD_1        (0x2U << SPI_I2SCFGR_I2SSTD_Pos)           /*!< 0x00000020 */
#define SPI_I2SCFGR_PCMSYNC_Pos     (7U)                                       
#define SPI_I2SCFGR_PCMSYNC_Msk     (0x1U << SPI_I2SCFGR_PCMSYNC_Pos)          /*!< 0x00000080 */
#define SPI_I2SCFGR_PCMSYNC         SPI_I2SCFGR_PCMSYNC_Msk                    /*!<PCM frame synchronization */
#define SPI_I2SCFGR_I2SCFG_Pos      (8U)                                       
#define SPI_I2SCFGR_I2SCFG_Msk      (0x3U << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000300 */
#define SPI_I2SCFGR_I2SCFG          SPI_I2SCFGR_I2SCFG_Msk                     /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define SPI_I2SCFGR_I2SCFG_0        (0x1U << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000100 */
#define SPI_I2SCFGR_I2SCFG_1        (0x2U << SPI_I2SCFGR_I2SCFG_Pos)           /*!< 0x00000200 */
#define SPI_I2SCFGR_I2SE_Pos        (10U)                                      
#define SPI_I2SCFGR_I2SE_Msk        (0x1U << SPI_I2SCFGR_I2SE_Pos)             /*!< 0x00000400 */
#define SPI_I2SCFGR_I2SE            SPI_I2SCFGR_I2SE_Msk                       /*!<I2S Enable */
#define SPI_I2SCFGR_I2SMOD_Pos      (11U)                                      
#define SPI_I2SCFGR_I2SMOD_Msk      (0x1U << SPI_I2SCFGR_I2SMOD_Pos)           /*!< 0x00000800 */
#define SPI_I2SCFGR_I2SMOD          SPI_I2SCFGR_I2SMOD_Msk                     /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define SPI_I2SPR_I2SDIV_Pos        (0U)                                       
#define SPI_I2SPR_I2SDIV_Msk        (0xFFU << SPI_I2SPR_I2SDIV_Pos)            /*!< 0x000000FF */
#define SPI_I2SPR_I2SDIV            SPI_I2SPR_I2SDIV_Msk                       /*!<I2S Linear prescaler */
#define SPI_I2SPR_ODD_Pos           (8U)                                       
#define SPI_I2SPR_ODD_Msk           (0x1U << SPI_I2SPR_ODD_Pos)                /*!< 0x00000100 */
#define SPI_I2SPR_ODD               SPI_I2SPR_ODD_Msk                          /*!<Odd factor for the prescaler */
#define SPI_I2SPR_MCKOE_Pos         (9U)                                       
#define SPI_I2SPR_MCKOE_Msk         (0x1U << SPI_I2SPR_MCKOE_Pos)              /*!< 0x00000200 */
#define SPI_I2SPR_MCKOE             SPI_I2SPR_MCKOE_Msk                        /*!<Master Clock Output Enable */

/******************************************************************************/
/*                                                                            */
/*                        System Configuration(SYSCFG)                        */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for SYSCFG_CFGR1 register  ****************/
#define SYSCFG_CFGR1_MEM_MODE_Pos                (0U)                          
#define SYSCFG_CFGR1_MEM_MODE_Msk                (0x3U << SYSCFG_CFGR1_MEM_MODE_Pos) /*!< 0x00000003 */
#define SYSCFG_CFGR1_MEM_MODE                    SYSCFG_CFGR1_MEM_MODE_Msk     /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_CFGR1_MEM_MODE_0                  (0x00000001U)                 /*!< Bit 0 */
#define SYSCFG_CFGR1_MEM_MODE_1                  (0x00000002U)                 /*!< Bit 1 */
#define SYSCFG_CFGR1_TIM1_ITR3_RMP_Pos           (6U)                          
#define SYSCFG_CFGR1_TIM1_ITR3_RMP_Msk           (0x1U << SYSCFG_CFGR1_TIM1_ITR3_RMP_Pos) /*!< 0x00000040 */
#define SYSCFG_CFGR1_TIM1_ITR3_RMP               SYSCFG_CFGR1_TIM1_ITR3_RMP_Msk /*!< Timer 1 ITR3 selection */
#define SYSCFG_CFGR1_DAC1_TRIG1_RMP_Pos          (7U)                          
#define SYSCFG_CFGR1_DAC1_TRIG1_RMP_Msk          (0x1U << SYSCFG_CFGR1_DAC1_TRIG1_RMP_Pos) /*!< 0x00000080 */
#define SYSCFG_CFGR1_DAC1_TRIG1_RMP              SYSCFG_CFGR1_DAC1_TRIG1_RMP_Msk /*!< DAC1 Trigger1 remap */
#define SYSCFG_CFGR1_DMA_RMP_Pos                 (11U)                         
#define SYSCFG_CFGR1_DMA_RMP_Msk                 (0x7U << SYSCFG_CFGR1_DMA_RMP_Pos) /*!< 0x00003800 */
#define SYSCFG_CFGR1_DMA_RMP                     SYSCFG_CFGR1_DMA_RMP_Msk      /*!< DMA remap mask */
#define SYSCFG_CFGR1_TIM16_DMA_RMP_Pos           (11U)                         
#define SYSCFG_CFGR1_TIM16_DMA_RMP_Msk           (0x1U << SYSCFG_CFGR1_TIM16_DMA_RMP_Pos) /*!< 0x00000800 */
#define SYSCFG_CFGR1_TIM16_DMA_RMP               SYSCFG_CFGR1_TIM16_DMA_RMP_Msk /*!< Timer 16 DMA remap */
#define SYSCFG_CFGR1_TIM17_DMA_RMP_Pos           (12U)                         
#define SYSCFG_CFGR1_TIM17_DMA_RMP_Msk           (0x1U << SYSCFG_CFGR1_TIM17_DMA_RMP_Pos) /*!< 0x00001000 */
#define SYSCFG_CFGR1_TIM17_DMA_RMP               SYSCFG_CFGR1_TIM17_DMA_RMP_Msk /*!< Timer 17 DMA remap */
#define SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Pos     (13U)                         
#define SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Msk     (0x1U << SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Pos) /*!< 0x00002000 */
#define SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP         SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP_Msk /*!< Timer 6 / DAC1 Ch1 DMA remap */
#define SYSCFG_CFGR1_I2C_PB6_FMP_Pos             (16U)                         
#define SYSCFG_CFGR1_I2C_PB6_FMP_Msk             (0x1U << SYSCFG_CFGR1_I2C_PB6_FMP_Pos) /*!< 0x00010000 */
#define SYSCFG_CFGR1_I2C_PB6_FMP                 SYSCFG_CFGR1_I2C_PB6_FMP_Msk  /*!< I2C PB6 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB7_FMP_Pos             (17U)                         
#define SYSCFG_CFGR1_I2C_PB7_FMP_Msk             (0x1U << SYSCFG_CFGR1_I2C_PB7_FMP_Pos) /*!< 0x00020000 */
#define SYSCFG_CFGR1_I2C_PB7_FMP                 SYSCFG_CFGR1_I2C_PB7_FMP_Msk  /*!< I2C PB7 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB8_FMP_Pos             (18U)                         
#define SYSCFG_CFGR1_I2C_PB8_FMP_Msk             (0x1U << SYSCFG_CFGR1_I2C_PB8_FMP_Pos) /*!< 0x00040000 */
#define SYSCFG_CFGR1_I2C_PB8_FMP                 SYSCFG_CFGR1_I2C_PB8_FMP_Msk  /*!< I2C PB8 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB9_FMP_Pos             (19U)                         
#define SYSCFG_CFGR1_I2C_PB9_FMP_Msk             (0x1U << SYSCFG_CFGR1_I2C_PB9_FMP_Pos) /*!< 0x00080000 */
#define SYSCFG_CFGR1_I2C_PB9_FMP                 SYSCFG_CFGR1_I2C_PB9_FMP_Msk  /*!< I2C PB9 Fast mode plus */
#define SYSCFG_CFGR1_I2C1_FMP_Pos                (20U)                         
#define SYSCFG_CFGR1_I2C1_FMP_Msk                (0x1U << SYSCFG_CFGR1_I2C1_FMP_Pos) /*!< 0x00100000 */
#define SYSCFG_CFGR1_I2C1_FMP                    SYSCFG_CFGR1_I2C1_FMP_Msk     /*!< I2C1 Fast mode plus */
#define SYSCFG_CFGR1_I2C2_FMP_Pos                (21U)                         
#define SYSCFG_CFGR1_I2C2_FMP_Msk                (0x1U << SYSCFG_CFGR1_I2C2_FMP_Pos) /*!< 0x00200000 */
#define SYSCFG_CFGR1_I2C2_FMP                    SYSCFG_CFGR1_I2C2_FMP_Msk     /*!< I2C2 Fast mode plus */
#define SYSCFG_CFGR1_ENCODER_MODE_Pos            (22U)                         
#define SYSCFG_CFGR1_ENCODER_MODE_Msk            (0x3U << SYSCFG_CFGR1_ENCODER_MODE_Pos) /*!< 0x00C00000 */
#define SYSCFG_CFGR1_ENCODER_MODE                SYSCFG_CFGR1_ENCODER_MODE_Msk /*!< Encoder Mode */
#define SYSCFG_CFGR1_ENCODER_MODE_0              (0x1U << SYSCFG_CFGR1_ENCODER_MODE_Pos) /*!< 0x00400000 */
#define SYSCFG_CFGR1_ENCODER_MODE_1              (0x2U << SYSCFG_CFGR1_ENCODER_MODE_Pos) /*!< 0x00800000 */
#define SYSCFG_CFGR1_ENCODER_MODE_TIM2_Pos       (22U)                         
#define SYSCFG_CFGR1_ENCODER_MODE_TIM2_Msk       (0x1U << SYSCFG_CFGR1_ENCODER_MODE_TIM2_Pos) /*!< 0x00400000 */
#define SYSCFG_CFGR1_ENCODER_MODE_TIM2           SYSCFG_CFGR1_ENCODER_MODE_TIM2_Msk /*!< TIM2 IC1 and TIM2 IC2 are connected to TIM15 IC1 and TIM15 IC2 respectively */
#define SYSCFG_CFGR1_I2C3_FMP_Pos                (24U)                         
#define SYSCFG_CFGR1_I2C3_FMP_Msk                (0x1U << SYSCFG_CFGR1_I2C3_FMP_Pos) /*!< 0x01000000 */
#define SYSCFG_CFGR1_I2C3_FMP                    SYSCFG_CFGR1_I2C3_FMP_Msk     /*!< I2C3 Fast mode plus */
#define SYSCFG_CFGR1_FPU_IE_Pos                  (26U)                         
#define SYSCFG_CFGR1_FPU_IE_Msk                  (0x3FU << SYSCFG_CFGR1_FPU_IE_Pos) /*!< 0xFC000000 */
#define SYSCFG_CFGR1_FPU_IE                      SYSCFG_CFGR1_FPU_IE_Msk       /*!< Floating Point Unit Interrupt Enable */
#define SYSCFG_CFGR1_FPU_IE_0                    (0x01U << SYSCFG_CFGR1_FPU_IE_Pos) /*!< 0x04000000 */
#define SYSCFG_CFGR1_FPU_IE_1                    (0x02U << SYSCFG_CFGR1_FPU_IE_Pos) /*!< 0x08000000 */
#define SYSCFG_CFGR1_FPU_IE_2                    (0x04U << SYSCFG_CFGR1_FPU_IE_Pos) /*!< 0x10000000 */
#define SYSCFG_CFGR1_FPU_IE_3                    (0x08U << SYSCFG_CFGR1_FPU_IE_Pos) /*!< 0x20000000 */
#define SYSCFG_CFGR1_FPU_IE_4                    (0x10U << SYSCFG_CFGR1_FPU_IE_Pos) /*!< 0x40000000 */
#define SYSCFG_CFGR1_FPU_IE_5                    (0x20U << SYSCFG_CFGR1_FPU_IE_Pos) /*!< 0x80000000 */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0_Pos                 (0U)                          
#define SYSCFG_EXTICR1_EXTI0_Msk                 (0xFU << SYSCFG_EXTICR1_EXTI0_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR1_EXTI0                     SYSCFG_EXTICR1_EXTI0_Msk      /*!< EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1_Pos                 (4U)                          
#define SYSCFG_EXTICR1_EXTI1_Msk                 (0xFU << SYSCFG_EXTICR1_EXTI1_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR1_EXTI1                     SYSCFG_EXTICR1_EXTI1_Msk      /*!< EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2_Pos                 (8U)                          
#define SYSCFG_EXTICR1_EXTI2_Msk                 (0xFU << SYSCFG_EXTICR1_EXTI2_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR1_EXTI2                     SYSCFG_EXTICR1_EXTI2_Msk      /*!< EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3_Pos                 (12U)                         
#define SYSCFG_EXTICR1_EXTI3_Msk                 (0xFU << SYSCFG_EXTICR1_EXTI3_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR1_EXTI3                     SYSCFG_EXTICR1_EXTI3_Msk      /*!< EXTI 3 configuration */

/*!<*
  * @brief  EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA                  (0x00000000U)                 /*!< PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB                  (0x00000001U)                 /*!< PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC                  (0x00000002U)                 /*!< PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD                  (0x00000003U)                 /*!< PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE                  (0x00000004U)                 /*!< PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF                  (0x00000005U)                 /*!< PF[0] pin */

/*!<*
  * @brief  EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA                  (0x00000000U)                 /*!< PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB                  (0x00000010U)                 /*!< PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC                  (0x00000020U)                 /*!< PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD                  (0x00000030U)                 /*!< PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE                  (0x00000040U)                 /*!< PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF                  (0x00000050U)                 /*!< PF[1] pin */

/*!<*
  * @brief  EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA                  (0x00000000U)                 /*!< PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB                  (0x00000100U)                 /*!< PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC                  (0x00000200U)                 /*!< PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD                  (0x00000300U)                 /*!< PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE                  (0x00000400U)                 /*!< PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF                  (0x00000500U)                 /*!< PF[2] pin */

/*!<*
  * @brief  EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA                  (0x00000000U)                 /*!< PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB                  (0x00001000U)                 /*!< PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC                  (0x00002000U)                 /*!< PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD                  (0x00003000U)                 /*!< PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE                  (0x00004000U)                 /*!< PE[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4_Pos                 (0U)                          
#define SYSCFG_EXTICR2_EXTI4_Msk                 (0xFU << SYSCFG_EXTICR2_EXTI4_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR2_EXTI4                     SYSCFG_EXTICR2_EXTI4_Msk      /*!< EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5_Pos                 (4U)                          
#define SYSCFG_EXTICR2_EXTI5_Msk                 (0xFU << SYSCFG_EXTICR2_EXTI5_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR2_EXTI5                     SYSCFG_EXTICR2_EXTI5_Msk      /*!< EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6_Pos                 (8U)                          
#define SYSCFG_EXTICR2_EXTI6_Msk                 (0xFU << SYSCFG_EXTICR2_EXTI6_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR2_EXTI6                     SYSCFG_EXTICR2_EXTI6_Msk      /*!< EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7_Pos                 (12U)                         
#define SYSCFG_EXTICR2_EXTI7_Msk                 (0xFU << SYSCFG_EXTICR2_EXTI7_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR2_EXTI7                     SYSCFG_EXTICR2_EXTI7_Msk      /*!< EXTI 7 configuration */

/*!<*
  * @brief  EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA                  (0x00000000U)                 /*!< PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB                  (0x00000001U)                 /*!< PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC                  (0x00000002U)                 /*!< PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD                  (0x00000003U)                 /*!< PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE                  (0x00000004U)                 /*!< PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF                  (0x00000005U)                 /*!< PF[4] pin */

/*!<*
  * @brief  EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA                  (0x00000000U)                 /*!< PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB                  (0x00000010U)                 /*!< PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC                  (0x00000020U)                 /*!< PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD                  (0x00000030U)                 /*!< PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE                  (0x00000040U)                 /*!< PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF                  (0x00000050U)                 /*!< PF[5] pin */

/*!<*
  * @brief  EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA                  (0x00000000U)                 /*!< PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB                  (0x00000100U)                 /*!< PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC                  (0x00000200U)                 /*!< PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD                  (0x00000300U)                 /*!< PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE                  (0x00000400U)                 /*!< PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF                  (0x00000500U)                 /*!< PF[6] pin */

/*!<*
  * @brief  EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA                  (0x00000000U)                 /*!< PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB                  (0x00001000U)                 /*!< PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC                  (0x00002000U)                 /*!< PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD                  (0x00003000U)                 /*!< PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE                  (0x00004000U)                 /*!< PE[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8_Pos                 (0U)                          
#define SYSCFG_EXTICR3_EXTI8_Msk                 (0xFU << SYSCFG_EXTICR3_EXTI8_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR3_EXTI8                     SYSCFG_EXTICR3_EXTI8_Msk      /*!< EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9_Pos                 (4U)                          
#define SYSCFG_EXTICR3_EXTI9_Msk                 (0xFU << SYSCFG_EXTICR3_EXTI9_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR3_EXTI9                     SYSCFG_EXTICR3_EXTI9_Msk      /*!< EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10_Pos                (8U)                          
#define SYSCFG_EXTICR3_EXTI10_Msk                (0xFU << SYSCFG_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR3_EXTI10                    SYSCFG_EXTICR3_EXTI10_Msk     /*!< EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11_Pos                (12U)                         
#define SYSCFG_EXTICR3_EXTI11_Msk                (0xFU << SYSCFG_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR3_EXTI11                    SYSCFG_EXTICR3_EXTI11_Msk     /*!< EXTI 11 configuration */

/*!<*
  * @brief  EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA                  (0x00000000U)                 /*!< PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB                  (0x00000001U)                 /*!< PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC                  (0x00000002U)                 /*!< PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD                  (0x00000003U)                 /*!< PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE                  (0x00000004U)                 /*!< PE[8] pin */

/*!<*
  * @brief  EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA                  (0x00000000U)                 /*!< PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB                  (0x00000010U)                 /*!< PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC                  (0x00000020U)                 /*!< PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD                  (0x00000030U)                 /*!< PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE                  (0x00000040U)                 /*!< PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF                  (0x00000050U)                 /*!< PF[9] pin */

/*!<*
  * @brief  EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA                 (0x00000000U)                 /*!< PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB                 (0x00000100U)                 /*!< PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC                 (0x00000200U)                 /*!< PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD                 (0x00000300U)                 /*!< PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE                 (0x00000400U)                 /*!< PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF                 (0x00000500U)                 /*!< PF[10] pin */

/*!<*
  * @brief  EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA                 (0x00000000U)                 /*!< PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB                 (0x00001000U)                 /*!< PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC                 (0x00002000U)                 /*!< PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD                 (0x00003000U)                 /*!< PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE                 (0x00004000U)                 /*!< PE[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  *****************/
#define SYSCFG_EXTICR4_EXTI12_Pos                (0U)                          
#define SYSCFG_EXTICR4_EXTI12_Msk                (0xFU << SYSCFG_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define SYSCFG_EXTICR4_EXTI12                    SYSCFG_EXTICR4_EXTI12_Msk     /*!< EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13_Pos                (4U)                          
#define SYSCFG_EXTICR4_EXTI13_Msk                (0xFU << SYSCFG_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define SYSCFG_EXTICR4_EXTI13                    SYSCFG_EXTICR4_EXTI13_Msk     /*!< EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14_Pos                (8U)                          
#define SYSCFG_EXTICR4_EXTI14_Msk                (0xFU << SYSCFG_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define SYSCFG_EXTICR4_EXTI14                    SYSCFG_EXTICR4_EXTI14_Msk     /*!< EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15_Pos                (12U)                         
#define SYSCFG_EXTICR4_EXTI15_Msk                (0xFU << SYSCFG_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */
#define SYSCFG_EXTICR4_EXTI15                    SYSCFG_EXTICR4_EXTI15_Msk     /*!< EXTI 15 configuration */

/*!<*
  * @brief  EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA                 (0x00000000U)                 /*!< PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB                 (0x00000001U)                 /*!< PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC                 (0x00000002U)                 /*!< PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD                 (0x00000003U)                 /*!< PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE                 (0x00000004U)                 /*!< PE[12] pin */

/*!<*
  * @brief  EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA                 (0x00000000U)                 /*!< PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB                 (0x00000010U)                 /*!< PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC                 (0x00000020U)                 /*!< PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD                 (0x00000030U)                 /*!< PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE                 (0x00000040U)                 /*!< PE[13] pin */

/*!<*
  * @brief  EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA                 (0x00000000U)                 /*!< PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB                 (0x00000100U)                 /*!< PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC                 (0x00000200U)                 /*!< PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD                 (0x00000300U)                 /*!< PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE                 (0x00000400U)                 /*!< PE[14] pin */

/*!<*
  * @brief  EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA                 (0x00000000U)                 /*!< PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB                 (0x00001000U)                 /*!< PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC                 (0x00002000U)                 /*!< PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD                 (0x00003000U)                 /*!< PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE                 (0x00004000U)                 /*!< PE[15] pin */

/*****************  Bit definition for SYSCFG_CFGR2 register  ****************/
#define SYSCFG_CFGR2_LOCKUP_LOCK_Pos             (0U)                          
#define SYSCFG_CFGR2_LOCKUP_LOCK_Msk             (0x1U << SYSCFG_CFGR2_LOCKUP_LOCK_Pos) /*!< 0x00000001 */
#define SYSCFG_CFGR2_LOCKUP_LOCK                 SYSCFG_CFGR2_LOCKUP_LOCK_Msk  /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM4 with Break Input of TIMx */
#define SYSCFG_CFGR2_PVD_LOCK_Pos                (2U)                          
#define SYSCFG_CFGR2_PVD_LOCK_Msk                (0x1U << SYSCFG_CFGR2_PVD_LOCK_Pos) /*!< 0x00000004 */
#define SYSCFG_CFGR2_PVD_LOCK                    SYSCFG_CFGR2_PVD_LOCK_Msk     /*!< Enables and locks the PVD connection with TIMx Break Input, as well as the PVDE and PLS[2:0] in the PWR_CR register */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define TIM_CR1_CEN_Pos           (0U)                                         
#define TIM_CR1_CEN_Msk           (0x1U << TIM_CR1_CEN_Pos)                    /*!< 0x00000001 */
#define TIM_CR1_CEN               TIM_CR1_CEN_Msk                              /*!<Counter enable */
#define TIM_CR1_UDIS_Pos          (1U)                                         
#define TIM_CR1_UDIS_Msk          (0x1U << TIM_CR1_UDIS_Pos)                   /*!< 0x00000002 */
#define TIM_CR1_UDIS              TIM_CR1_UDIS_Msk                             /*!<Update disable */
#define TIM_CR1_URS_Pos           (2U)                                         
#define TIM_CR1_URS_Msk           (0x1U << TIM_CR1_URS_Pos)                    /*!< 0x00000004 */
#define TIM_CR1_URS               TIM_CR1_URS_Msk                              /*!<Update request source */
#define TIM_CR1_OPM_Pos           (3U)                                         
#define TIM_CR1_OPM_Msk           (0x1U << TIM_CR1_OPM_Pos)                    /*!< 0x00000008 */
#define TIM_CR1_OPM               TIM_CR1_OPM_Msk                              /*!<One pulse mode */
#define TIM_CR1_DIR_Pos           (4U)                                         
#define TIM_CR1_DIR_Msk           (0x1U << TIM_CR1_DIR_Pos)                    /*!< 0x00000010 */
#define TIM_CR1_DIR               TIM_CR1_DIR_Msk                              /*!<Direction */

#define TIM_CR1_CMS_Pos           (5U)                                         
#define TIM_CR1_CMS_Msk           (0x3U << TIM_CR1_CMS_Pos)                    /*!< 0x00000060 */
#define TIM_CR1_CMS               TIM_CR1_CMS_Msk                              /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0             (0x1U << TIM_CR1_CMS_Pos)                    /*!< 0x00000020 */
#define TIM_CR1_CMS_1             (0x2U << TIM_CR1_CMS_Pos)                    /*!< 0x00000040 */

#define TIM_CR1_ARPE_Pos          (7U)                                         
#define TIM_CR1_ARPE_Msk          (0x1U << TIM_CR1_ARPE_Pos)                   /*!< 0x00000080 */
#define TIM_CR1_ARPE              TIM_CR1_ARPE_Msk                             /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos           (8U)                                         
#define TIM_CR1_CKD_Msk           (0x3U << TIM_CR1_CKD_Pos)                    /*!< 0x00000300 */
#define TIM_CR1_CKD               TIM_CR1_CKD_Msk                              /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0             (0x1U << TIM_CR1_CKD_Pos)                    /*!< 0x00000100 */
#define TIM_CR1_CKD_1             (0x2U << TIM_CR1_CKD_Pos)                    /*!< 0x00000200 */

#define TIM_CR1_UIFREMAP_Pos      (11U)                                        
#define TIM_CR1_UIFREMAP_Msk      (0x1U << TIM_CR1_UIFREMAP_Pos)               /*!< 0x00000800 */
#define TIM_CR1_UIFREMAP          TIM_CR1_UIFREMAP_Msk                         /*!<Update interrupt flag remap */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define TIM_CR2_CCPC_Pos          (0U)                                         
#define TIM_CR2_CCPC_Msk          (0x1U << TIM_CR2_CCPC_Pos)                   /*!< 0x00000001 */
#define TIM_CR2_CCPC              TIM_CR2_CCPC_Msk                             /*!<Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS_Pos          (2U)                                         
#define TIM_CR2_CCUS_Msk          (0x1U << TIM_CR2_CCUS_Pos)                   /*!< 0x00000004 */
#define TIM_CR2_CCUS              TIM_CR2_CCUS_Msk                             /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos          (3U)                                         
#define TIM_CR2_CCDS_Msk          (0x1U << TIM_CR2_CCDS_Pos)                   /*!< 0x00000008 */
#define TIM_CR2_CCDS              TIM_CR2_CCDS_Msk                             /*!<Capture/Compare DMA Selection */

#define TIM_CR2_MMS_Pos           (4U)                                         
#define TIM_CR2_MMS_Msk           (0x7U << TIM_CR2_MMS_Pos)                    /*!< 0x00000070 */
#define TIM_CR2_MMS               TIM_CR2_MMS_Msk                              /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0             (0x1U << TIM_CR2_MMS_Pos)                    /*!< 0x00000010 */
#define TIM_CR2_MMS_1             (0x2U << TIM_CR2_MMS_Pos)                    /*!< 0x00000020 */
#define TIM_CR2_MMS_2             (0x4U << TIM_CR2_MMS_Pos)                    /*!< 0x00000040 */

#define TIM_CR2_TI1S_Pos          (7U)                                         
#define TIM_CR2_TI1S_Msk          (0x1U << TIM_CR2_TI1S_Pos)                   /*!< 0x00000080 */
#define TIM_CR2_TI1S              TIM_CR2_TI1S_Msk                             /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos          (8U)                                         
#define TIM_CR2_OIS1_Msk          (0x1U << TIM_CR2_OIS1_Pos)                   /*!< 0x00000100 */
#define TIM_CR2_OIS1              TIM_CR2_OIS1_Msk                             /*!<Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N_Pos         (9U)                                         
#define TIM_CR2_OIS1N_Msk         (0x1U << TIM_CR2_OIS1N_Pos)                  /*!< 0x00000200 */
#define TIM_CR2_OIS1N             TIM_CR2_OIS1N_Msk                            /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos          (10U)                                        
#define TIM_CR2_OIS2_Msk          (0x1U << TIM_CR2_OIS2_Pos)                   /*!< 0x00000400 */
#define TIM_CR2_OIS2              TIM_CR2_OIS2_Msk                             /*!<Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N_Pos         (11U)                                        
#define TIM_CR2_OIS2N_Msk         (0x1U << TIM_CR2_OIS2N_Pos)                  /*!< 0x00000800 */
#define TIM_CR2_OIS2N             TIM_CR2_OIS2N_Msk                            /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos          (12U)                                        
#define TIM_CR2_OIS3_Msk          (0x1U << TIM_CR2_OIS3_Pos)                   /*!< 0x00001000 */
#define TIM_CR2_OIS3              TIM_CR2_OIS3_Msk                             /*!<Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N_Pos         (13U)                                        
#define TIM_CR2_OIS3N_Msk         (0x1U << TIM_CR2_OIS3N_Pos)                  /*!< 0x00002000 */
#define TIM_CR2_OIS3N             TIM_CR2_OIS3N_Msk                            /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos          (14U)                                        
#define TIM_CR2_OIS4_Msk          (0x1U << TIM_CR2_OIS4_Pos)                   /*!< 0x00004000 */
#define TIM_CR2_OIS4              TIM_CR2_OIS4_Msk                             /*!<Output Idle state 4 (OC4 output) */

#define TIM_CR2_OIS5_Pos          (16U)                                        
#define TIM_CR2_OIS5_Msk          (0x1U << TIM_CR2_OIS5_Pos)                   /*!< 0x00010000 */
#define TIM_CR2_OIS5              TIM_CR2_OIS5_Msk                             /*!<Output Idle state 4 (OC4 output) */
#define TIM_CR2_OIS6_Pos          (18U)                                        
#define TIM_CR2_OIS6_Msk          (0x1U << TIM_CR2_OIS6_Pos)                   /*!< 0x00040000 */
#define TIM_CR2_OIS6              TIM_CR2_OIS6_Msk                             /*!<Output Idle state 4 (OC4 output) */

#define TIM_CR2_MMS2_Pos          (20U)                                        
#define TIM_CR2_MMS2_Msk          (0xFU << TIM_CR2_MMS2_Pos)                   /*!< 0x00F00000 */
#define TIM_CR2_MMS2              TIM_CR2_MMS2_Msk                             /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS2_0            (0x1U << TIM_CR2_MMS2_Pos)                   /*!< 0x00100000 */
#define TIM_CR2_MMS2_1            (0x2U << TIM_CR2_MMS2_Pos)                   /*!< 0x00200000 */
#define TIM_CR2_MMS2_2            (0x4U << TIM_CR2_MMS2_Pos)                   /*!< 0x00400000 */
#define TIM_CR2_MMS2_3            (0x8U << TIM_CR2_MMS2_Pos)                   /*!< 0x00800000 */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define TIM_SMCR_SMS_Pos          (0U)                                         
#define TIM_SMCR_SMS_Msk          (0x10007U << TIM_SMCR_SMS_Pos)               /*!< 0x00010007 */
#define TIM_SMCR_SMS              TIM_SMCR_SMS_Msk                             /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0            (0x00000001U)                                /*!<Bit 0 */
#define TIM_SMCR_SMS_1            (0x00000002U)                                /*!<Bit 1 */
#define TIM_SMCR_SMS_2            (0x00000004U)                                /*!<Bit 2 */
#define TIM_SMCR_SMS_3            (0x00010000U)                                /*!<Bit 3 */

#define TIM_SMCR_OCCS_Pos         (3U)                                         
#define TIM_SMCR_OCCS_Msk         (0x1U << TIM_SMCR_OCCS_Pos)                  /*!< 0x00000008 */
#define TIM_SMCR_OCCS             TIM_SMCR_OCCS_Msk                            /*!< OCREF clear selection */

#define TIM_SMCR_TS_Pos           (4U)                                         
#define TIM_SMCR_TS_Msk           (0x7U << TIM_SMCR_TS_Pos)                    /*!< 0x00000070 */
#define TIM_SMCR_TS               TIM_SMCR_TS_Msk                              /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0             (0x1U << TIM_SMCR_TS_Pos)                    /*!< 0x00000010 */
#define TIM_SMCR_TS_1             (0x2U << TIM_SMCR_TS_Pos)                    /*!< 0x00000020 */
#define TIM_SMCR_TS_2             (0x4U << TIM_SMCR_TS_Pos)                    /*!< 0x00000040 */

#define TIM_SMCR_MSM_Pos          (7U)                                         
#define TIM_SMCR_MSM_Msk          (0x1U << TIM_SMCR_MSM_Pos)                   /*!< 0x00000080 */
#define TIM_SMCR_MSM              TIM_SMCR_MSM_Msk                             /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos          (8U)                                         
#define TIM_SMCR_ETF_Msk          (0xFU << TIM_SMCR_ETF_Pos)                   /*!< 0x00000F00 */
#define TIM_SMCR_ETF              TIM_SMCR_ETF_Msk                             /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0            (0x1U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000100 */
#define TIM_SMCR_ETF_1            (0x2U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000200 */
#define TIM_SMCR_ETF_2            (0x4U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000400 */
#define TIM_SMCR_ETF_3            (0x8U << TIM_SMCR_ETF_Pos)                   /*!< 0x00000800 */

#define TIM_SMCR_ETPS_Pos         (12U)                                        
#define TIM_SMCR_ETPS_Msk         (0x3U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00003000 */
#define TIM_SMCR_ETPS             TIM_SMCR_ETPS_Msk                            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0           (0x1U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00001000 */
#define TIM_SMCR_ETPS_1           (0x2U << TIM_SMCR_ETPS_Pos)                  /*!< 0x00002000 */

#define TIM_SMCR_ECE_Pos          (14U)                                        
#define TIM_SMCR_ECE_Msk          (0x1U << TIM_SMCR_ECE_Pos)                   /*!< 0x00004000 */
#define TIM_SMCR_ECE              TIM_SMCR_ECE_Msk                             /*!<External clock enable */
#define TIM_SMCR_ETP_Pos          (15U)                                        
#define TIM_SMCR_ETP_Msk          (0x1U << TIM_SMCR_ETP_Pos)                   /*!< 0x00008000 */
#define TIM_SMCR_ETP              TIM_SMCR_ETP_Msk                             /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define TIM_DIER_UIE_Pos          (0U)                                         
#define TIM_DIER_UIE_Msk          (0x1U << TIM_DIER_UIE_Pos)                   /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk                             /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos        (1U)                                         
#define TIM_DIER_CC1IE_Msk        (0x1U << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk                           /*!<Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE_Pos        (2U)                                         
#define TIM_DIER_CC2IE_Msk        (0x1U << TIM_DIER_CC2IE_Pos)                 /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk                           /*!<Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE_Pos        (3U)                                         
#define TIM_DIER_CC3IE_Msk        (0x1U << TIM_DIER_CC3IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk                           /*!<Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE_Pos        (4U)                                         
#define TIM_DIER_CC4IE_Msk        (0x1U << TIM_DIER_CC4IE_Pos)                 /*!< 0x00000010 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk                           /*!<Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE_Pos        (5U)                                         
#define TIM_DIER_COMIE_Msk        (0x1U << TIM_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                           /*!<COM interrupt enable */
#define TIM_DIER_TIE_Pos          (6U)                                         
#define TIM_DIER_TIE_Msk          (0x1U << TIM_DIER_TIE_Pos)                   /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                             /*!<Trigger interrupt enable */
#define TIM_DIER_BIE_Pos          (7U)                                         
#define TIM_DIER_BIE_Msk          (0x1U << TIM_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk                             /*!<Break interrupt enable */
#define TIM_DIER_UDE_Pos          (8U)                                         
#define TIM_DIER_UDE_Msk          (0x1U << TIM_DIER_UDE_Pos)                   /*!< 0x00000100 */
#define TIM_DIER_UDE              TIM_DIER_UDE_Msk                             /*!<Update DMA request enable */
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
#define TIM_DIER_COMDE            TIM_DIER_COMDE_Msk                           /*!<COM DMA request enable */
#define TIM_DIER_TDE_Pos          (14U)                                        
#define TIM_DIER_TDE_Msk          (0x1U << TIM_DIER_TDE_Pos)                   /*!< 0x00004000 */
#define TIM_DIER_TDE              TIM_DIER_TDE_Msk                             /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define TIM_SR_UIF_Pos            (0U)                                         
#define TIM_SR_UIF_Msk            (0x1U << TIM_SR_UIF_Pos)                     /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                               /*!<Update interrupt Flag */
#define TIM_SR_CC1IF_Pos          (1U)                                         
#define TIM_SR_CC1IF_Msk          (0x1U << TIM_SR_CC1IF_Pos)                   /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk                             /*!<Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF_Pos          (2U)                                         
#define TIM_SR_CC2IF_Msk          (0x1U << TIM_SR_CC2IF_Pos)                   /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk                             /*!<Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF_Pos          (3U)                                         
#define TIM_SR_CC3IF_Msk          (0x1U << TIM_SR_CC3IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk                             /*!<Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF_Pos          (4U)                                         
#define TIM_SR_CC4IF_Msk          (0x1U << TIM_SR_CC4IF_Pos)                   /*!< 0x00000010 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk                             /*!<Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF_Pos          (5U)                                         
#define TIM_SR_COMIF_Msk          (0x1U << TIM_SR_COMIF_Pos)                   /*!< 0x00000020 */
#define TIM_SR_COMIF              TIM_SR_COMIF_Msk                             /*!<COM interrupt Flag */
#define TIM_SR_TIF_Pos            (6U)                                         
#define TIM_SR_TIF_Msk            (0x1U << TIM_SR_TIF_Pos)                     /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk                               /*!<Trigger interrupt Flag */
#define TIM_SR_BIF_Pos            (7U)                                         
#define TIM_SR_BIF_Msk            (0x1U << TIM_SR_BIF_Pos)                     /*!< 0x00000080 */
#define TIM_SR_BIF                TIM_SR_BIF_Msk                               /*!<Break interrupt Flag */
#define TIM_SR_B2IF_Pos           (8U)                                         
#define TIM_SR_B2IF_Msk           (0x1U << TIM_SR_B2IF_Pos)                    /*!< 0x00000100 */
#define TIM_SR_B2IF               TIM_SR_B2IF_Msk                              /*!<Break2 interrupt Flag */
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
#define TIM_SR_CC5IF_Pos          (16U)                                        
#define TIM_SR_CC5IF_Msk          (0x1U << TIM_SR_CC5IF_Pos)                   /*!< 0x00010000 */
#define TIM_SR_CC5IF              TIM_SR_CC5IF_Msk                             /*!<Capture/Compare 5 interrupt Flag */
#define TIM_SR_CC6IF_Pos          (17U)                                        
#define TIM_SR_CC6IF_Msk          (0x1U << TIM_SR_CC6IF_Pos)                   /*!< 0x00020000 */
#define TIM_SR_CC6IF              TIM_SR_CC6IF_Msk                             /*!<Capture/Compare 6 interrupt Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define TIM_EGR_UG_Pos            (0U)                                         
#define TIM_EGR_UG_Msk            (0x1U << TIM_EGR_UG_Pos)                     /*!< 0x00000001 */
#define TIM_EGR_UG                TIM_EGR_UG_Msk                               /*!<Update Generation */
#define TIM_EGR_CC1G_Pos          (1U)                                         
#define TIM_EGR_CC1G_Msk          (0x1U << TIM_EGR_CC1G_Pos)                   /*!< 0x00000002 */
#define TIM_EGR_CC1G              TIM_EGR_CC1G_Msk                             /*!<Capture/Compare 1 Generation */
#define TIM_EGR_CC2G_Pos          (2U)                                         
#define TIM_EGR_CC2G_Msk          (0x1U << TIM_EGR_CC2G_Pos)                   /*!< 0x00000004 */
#define TIM_EGR_CC2G              TIM_EGR_CC2G_Msk                             /*!<Capture/Compare 2 Generation */
#define TIM_EGR_CC3G_Pos          (3U)                                         
#define TIM_EGR_CC3G_Msk          (0x1U << TIM_EGR_CC3G_Pos)                   /*!< 0x00000008 */
#define TIM_EGR_CC3G              TIM_EGR_CC3G_Msk                             /*!<Capture/Compare 3 Generation */
#define TIM_EGR_CC4G_Pos          (4U)                                         
#define TIM_EGR_CC4G_Msk          (0x1U << TIM_EGR_CC4G_Pos)                   /*!< 0x00000010 */
#define TIM_EGR_CC4G              TIM_EGR_CC4G_Msk                             /*!<Capture/Compare 4 Generation */
#define TIM_EGR_COMG_Pos          (5U)                                         
#define TIM_EGR_COMG_Msk          (0x1U << TIM_EGR_COMG_Pos)                   /*!< 0x00000020 */
#define TIM_EGR_COMG              TIM_EGR_COMG_Msk                             /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos            (6U)                                         
#define TIM_EGR_TG_Msk            (0x1U << TIM_EGR_TG_Pos)                     /*!< 0x00000040 */
#define TIM_EGR_TG                TIM_EGR_TG_Msk                               /*!<Trigger Generation */
#define TIM_EGR_BG_Pos            (7U)                                         
#define TIM_EGR_BG_Msk            (0x1U << TIM_EGR_BG_Pos)                     /*!< 0x00000080 */
#define TIM_EGR_BG                TIM_EGR_BG_Msk                               /*!<Break Generation */
#define TIM_EGR_B2G_Pos           (8U)                                         
#define TIM_EGR_B2G_Msk           (0x1U << TIM_EGR_B2G_Pos)                    /*!< 0x00000100 */
#define TIM_EGR_B2G               TIM_EGR_B2G_Msk                              /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define TIM_CCMR1_CC1S_Pos        (0U)                                         
#define TIM_CCMR1_CC1S_Msk        (0x3U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk                           /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0          (0x1U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_1          (0x2U << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000002 */

#define TIM_CCMR1_OC1FE_Pos       (2U)                                         
#define TIM_CCMR1_OC1FE_Msk       (0x1U << TIM_CCMR1_OC1FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE           TIM_CCMR1_OC1FE_Msk                          /*!<Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE_Pos       (3U)                                         
#define TIM_CCMR1_OC1PE_Msk       (0x1U << TIM_CCMR1_OC1PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE           TIM_CCMR1_OC1PE_Msk                          /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos        (4U)                                         
#define TIM_CCMR1_OC1M_Msk        (0x1007U << TIM_CCMR1_OC1M_Pos)              /*!< 0x00010070 */
#define TIM_CCMR1_OC1M            TIM_CCMR1_OC1M_Msk                           /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0          (0x00000010U)                                /*!<Bit 0 */
#define TIM_CCMR1_OC1M_1          (0x00000020U)                                /*!<Bit 1 */
#define TIM_CCMR1_OC1M_2          (0x00000040U)                                /*!<Bit 2 */
#define TIM_CCMR1_OC1M_3          (0x00010000U)                                /*!<Bit 3 */

#define TIM_CCMR1_OC1CE_Pos       (7U)                                         
#define TIM_CCMR1_OC1CE_Msk       (0x1U << TIM_CCMR1_OC1CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE           TIM_CCMR1_OC1CE_Msk                          /*!<Output Compare 1Clear Enable */

#define TIM_CCMR1_CC2S_Pos        (8U)                                         
#define TIM_CCMR1_CC2S_Msk        (0x3U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk                           /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0          (0x1U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_1          (0x2U << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000200 */

#define TIM_CCMR1_OC2FE_Pos       (10U)                                        
#define TIM_CCMR1_OC2FE_Msk       (0x1U << TIM_CCMR1_OC2FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE           TIM_CCMR1_OC2FE_Msk                          /*!<Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE_Pos       (11U)                                        
#define TIM_CCMR1_OC2PE_Msk       (0x1U << TIM_CCMR1_OC2PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE           TIM_CCMR1_OC2PE_Msk                          /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos        (12U)                                        
#define TIM_CCMR1_OC2M_Msk        (0x1007U << TIM_CCMR1_OC2M_Pos)              /*!< 0x01007000 */
#define TIM_CCMR1_OC2M            TIM_CCMR1_OC2M_Msk                           /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0          (0x00001000U)                                /*!<Bit 0 */
#define TIM_CCMR1_OC2M_1          (0x00002000U)                                /*!<Bit 1 */
#define TIM_CCMR1_OC2M_2          (0x00004000U)                                /*!<Bit 2 */
#define TIM_CCMR1_OC2M_3          (0x01000000U)                                /*!<Bit 3 */

#define TIM_CCMR1_OC2CE_Pos       (15U)                                        
#define TIM_CCMR1_OC2CE_Msk       (0x1U << TIM_CCMR1_OC2CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE           TIM_CCMR1_OC2CE_Msk                          /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos      (2U)                                         
#define TIM_CCMR1_IC1PSC_Msk      (0x3U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC          TIM_CCMR1_IC1PSC_Msk                         /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0        (0x1U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1        (0x2U << TIM_CCMR1_IC1PSC_Pos)               /*!< 0x00000008 */

#define TIM_CCMR1_IC1F_Pos        (4U)                                         
#define TIM_CCMR1_IC1F_Msk        (0xFU << TIM_CCMR1_IC1F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F            TIM_CCMR1_IC1F_Msk                           /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0          (0x1U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_1          (0x2U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_2          (0x4U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_3          (0x8U << TIM_CCMR1_IC1F_Pos)                 /*!< 0x00000080 */

#define TIM_CCMR1_IC2PSC_Pos      (10U)                                        
#define TIM_CCMR1_IC2PSC_Msk      (0x3U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC          TIM_CCMR1_IC2PSC_Msk                         /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0        (0x1U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_1        (0x2U << TIM_CCMR1_IC2PSC_Pos)               /*!< 0x00000800 */

#define TIM_CCMR1_IC2F_Pos        (12U)                                        
#define TIM_CCMR1_IC2F_Msk        (0xFU << TIM_CCMR1_IC2F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F            TIM_CCMR1_IC2F_Msk                           /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0          (0x1U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_1          (0x2U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_2          (0x4U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR1_IC2F_3          (0x8U << TIM_CCMR1_IC2F_Pos)                 /*!< 0x00008000 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define TIM_CCMR2_CC3S_Pos        (0U)                                         
#define TIM_CCMR2_CC3S_Msk        (0x3U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk                           /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0          (0x1U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_1          (0x2U << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000002 */

#define TIM_CCMR2_OC3FE_Pos       (2U)                                         
#define TIM_CCMR2_OC3FE_Msk       (0x1U << TIM_CCMR2_OC3FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE           TIM_CCMR2_OC3FE_Msk                          /*!<Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE_Pos       (3U)                                         
#define TIM_CCMR2_OC3PE_Msk       (0x1U << TIM_CCMR2_OC3PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE           TIM_CCMR2_OC3PE_Msk                          /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos        (4U)                                         
#define TIM_CCMR2_OC3M_Msk        (0x1007U << TIM_CCMR2_OC3M_Pos)              /*!< 0x00010070 */
#define TIM_CCMR2_OC3M            TIM_CCMR2_OC3M_Msk                           /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0          (0x00000010U)                                /*!<Bit 0 */
#define TIM_CCMR2_OC3M_1          (0x00000020U)                                /*!<Bit 1 */
#define TIM_CCMR2_OC3M_2          (0x00000040U)                                /*!<Bit 2 */
#define TIM_CCMR2_OC3M_3          (0x00010000U)                                /*!<Bit 3 */

#define TIM_CCMR2_OC3CE_Pos       (7U)                                         
#define TIM_CCMR2_OC3CE_Msk       (0x1U << TIM_CCMR2_OC3CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE           TIM_CCMR2_OC3CE_Msk                          /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos        (8U)                                         
#define TIM_CCMR2_CC4S_Msk        (0x3U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk                           /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0          (0x1U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_1          (0x2U << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000200 */

#define TIM_CCMR2_OC4FE_Pos       (10U)                                        
#define TIM_CCMR2_OC4FE_Msk       (0x1U << TIM_CCMR2_OC4FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE           TIM_CCMR2_OC4FE_Msk                          /*!<Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE_Pos       (11U)                                        
#define TIM_CCMR2_OC4PE_Msk       (0x1U << TIM_CCMR2_OC4PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE           TIM_CCMR2_OC4PE_Msk                          /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos        (12U)                                        
#define TIM_CCMR2_OC4M_Msk        (0x1007U << TIM_CCMR2_OC4M_Pos)              /*!< 0x01007000 */
#define TIM_CCMR2_OC4M            TIM_CCMR2_OC4M_Msk                           /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0          (0x00001000U)                                /*!<Bit 0 */
#define TIM_CCMR2_OC4M_1          (0x00002000U)                                /*!<Bit 1 */
#define TIM_CCMR2_OC4M_2          (0x00004000U)                                /*!<Bit 2 */
#define TIM_CCMR2_OC4M_3          (0x01000000U)                                /*!<Bit 3 */

#define TIM_CCMR2_OC4CE_Pos       (15U)                                        
#define TIM_CCMR2_OC4CE_Msk       (0x1U << TIM_CCMR2_OC4CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE           TIM_CCMR2_OC4CE_Msk                          /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC_Pos      (2U)                                         
#define TIM_CCMR2_IC3PSC_Msk      (0x3U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC          TIM_CCMR2_IC3PSC_Msk                         /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0        (0x1U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_1        (0x2U << TIM_CCMR2_IC3PSC_Pos)               /*!< 0x00000008 */

#define TIM_CCMR2_IC3F_Pos        (4U)                                         
#define TIM_CCMR2_IC3F_Msk        (0xFU << TIM_CCMR2_IC3F_Pos)                 /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F            TIM_CCMR2_IC3F_Msk                           /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0          (0x1U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_1          (0x2U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_2          (0x4U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_3          (0x8U << TIM_CCMR2_IC3F_Pos)                 /*!< 0x00000080 */

#define TIM_CCMR2_IC4PSC_Pos      (10U)                                        
#define TIM_CCMR2_IC4PSC_Msk      (0x3U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC          TIM_CCMR2_IC4PSC_Msk                         /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0        (0x1U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_1        (0x2U << TIM_CCMR2_IC4PSC_Pos)               /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos        (12U)                                        
#define TIM_CCMR2_IC4F_Msk        (0xFU << TIM_CCMR2_IC4F_Pos)                 /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F            TIM_CCMR2_IC4F_Msk                           /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0          (0x1U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_1          (0x2U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_2          (0x4U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00004000 */
#define TIM_CCMR2_IC4F_3          (0x8U << TIM_CCMR2_IC4F_Pos)                 /*!< 0x00008000 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define TIM_CCER_CC1E_Pos         (0U)                                         
#define TIM_CCER_CC1E_Msk         (0x1U << TIM_CCER_CC1E_Pos)                  /*!< 0x00000001 */
#define TIM_CCER_CC1E             TIM_CCER_CC1E_Msk                            /*!<Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos         (1U)                                         
#define TIM_CCER_CC1P_Msk         (0x1U << TIM_CCER_CC1P_Pos)                  /*!< 0x00000002 */
#define TIM_CCER_CC1P             TIM_CCER_CC1P_Msk                            /*!<Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos        (2U)                                         
#define TIM_CCER_CC1NE_Msk        (0x1U << TIM_CCER_CC1NE_Pos)                 /*!< 0x00000004 */
#define TIM_CCER_CC1NE            TIM_CCER_CC1NE_Msk                           /*!<Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP_Pos        (3U)                                         
#define TIM_CCER_CC1NP_Msk        (0x1U << TIM_CCER_CC1NP_Pos)                 /*!< 0x00000008 */
#define TIM_CCER_CC1NP            TIM_CCER_CC1NP_Msk                           /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos         (4U)                                         
#define TIM_CCER_CC2E_Msk         (0x1U << TIM_CCER_CC2E_Pos)                  /*!< 0x00000010 */
#define TIM_CCER_CC2E             TIM_CCER_CC2E_Msk                            /*!<Capture/Compare 2 output enable */
#define TIM_CCER_CC2P_Pos         (5U)                                         
#define TIM_CCER_CC2P_Msk         (0x1U << TIM_CCER_CC2P_Pos)                  /*!< 0x00000020 */
#define TIM_CCER_CC2P             TIM_CCER_CC2P_Msk                            /*!<Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE_Pos        (6U)                                         
#define TIM_CCER_CC2NE_Msk        (0x1U << TIM_CCER_CC2NE_Pos)                 /*!< 0x00000040 */
#define TIM_CCER_CC2NE            TIM_CCER_CC2NE_Msk                           /*!<Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP_Pos        (7U)                                         
#define TIM_CCER_CC2NP_Msk        (0x1U << TIM_CCER_CC2NP_Pos)                 /*!< 0x00000080 */
#define TIM_CCER_CC2NP            TIM_CCER_CC2NP_Msk                           /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos         (8U)                                         
#define TIM_CCER_CC3E_Msk         (0x1U << TIM_CCER_CC3E_Pos)                  /*!< 0x00000100 */
#define TIM_CCER_CC3E             TIM_CCER_CC3E_Msk                            /*!<Capture/Compare 3 output enable */
#define TIM_CCER_CC3P_Pos         (9U)                                         
#define TIM_CCER_CC3P_Msk         (0x1U << TIM_CCER_CC3P_Pos)                  /*!< 0x00000200 */
#define TIM_CCER_CC3P             TIM_CCER_CC3P_Msk                            /*!<Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE_Pos        (10U)                                        
#define TIM_CCER_CC3NE_Msk        (0x1U << TIM_CCER_CC3NE_Pos)                 /*!< 0x00000400 */
#define TIM_CCER_CC3NE            TIM_CCER_CC3NE_Msk                           /*!<Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP_Pos        (11U)                                        
#define TIM_CCER_CC3NP_Msk        (0x1U << TIM_CCER_CC3NP_Pos)                 /*!< 0x00000800 */
#define TIM_CCER_CC3NP            TIM_CCER_CC3NP_Msk                           /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos         (12U)                                        
#define TIM_CCER_CC4E_Msk         (0x1U << TIM_CCER_CC4E_Pos)                  /*!< 0x00001000 */
#define TIM_CCER_CC4E             TIM_CCER_CC4E_Msk                            /*!<Capture/Compare 4 output enable */
#define TIM_CCER_CC4P_Pos         (13U)                                        
#define TIM_CCER_CC4P_Msk         (0x1U << TIM_CCER_CC4P_Pos)                  /*!< 0x00002000 */
#define TIM_CCER_CC4P             TIM_CCER_CC4P_Msk                            /*!<Capture/Compare 4 output Polarity */
#define TIM_CCER_CC4NP_Pos        (15U)                                        
#define TIM_CCER_CC4NP_Msk        (0x1U << TIM_CCER_CC4NP_Pos)                 /*!< 0x00008000 */
#define TIM_CCER_CC4NP            TIM_CCER_CC4NP_Msk                           /*!<Capture/Compare 4 Complementary output Polarity */
#define TIM_CCER_CC5E_Pos         (16U)                                        
#define TIM_CCER_CC5E_Msk         (0x1U << TIM_CCER_CC5E_Pos)                  /*!< 0x00010000 */
#define TIM_CCER_CC5E             TIM_CCER_CC5E_Msk                            /*!<Capture/Compare 5 output enable */
#define TIM_CCER_CC5P_Pos         (17U)                                        
#define TIM_CCER_CC5P_Msk         (0x1U << TIM_CCER_CC5P_Pos)                  /*!< 0x00020000 */
#define TIM_CCER_CC5P             TIM_CCER_CC5P_Msk                            /*!<Capture/Compare 5 output Polarity */
#define TIM_CCER_CC6E_Pos         (20U)                                        
#define TIM_CCER_CC6E_Msk         (0x1U << TIM_CCER_CC6E_Pos)                  /*!< 0x00100000 */
#define TIM_CCER_CC6E             TIM_CCER_CC6E_Msk                            /*!<Capture/Compare 6 output enable */
#define TIM_CCER_CC6P_Pos         (21U)                                        
#define TIM_CCER_CC6P_Msk         (0x1U << TIM_CCER_CC6P_Pos)                  /*!< 0x00200000 */
#define TIM_CCER_CC6P             TIM_CCER_CC6P_Msk                            /*!<Capture/Compare 6 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define TIM_CNT_CNT_Pos           (0U)                                         
#define TIM_CNT_CNT_Msk           (0xFFFFFFFFU << TIM_CNT_CNT_Pos)             /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT               TIM_CNT_CNT_Msk                              /*!<Counter Value */
#define TIM_CNT_UIFCPY_Pos        (31U)                                        
#define TIM_CNT_UIFCPY_Msk        (0x1U << TIM_CNT_UIFCPY_Pos)                 /*!< 0x80000000 */
#define TIM_CNT_UIFCPY            TIM_CNT_UIFCPY_Msk                           /*!<Update interrupt flag copy */

/*******************  Bit definition for TIM_PSC register  ********************/
#define TIM_PSC_PSC_Pos           (0U)                                         
#define TIM_PSC_PSC_Msk           (0xFFFFU << TIM_PSC_PSC_Pos)                 /*!< 0x0000FFFF */
#define TIM_PSC_PSC               TIM_PSC_PSC_Msk                              /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define TIM_ARR_ARR_Pos           (0U)                                         
#define TIM_ARR_ARR_Msk           (0xFFFFFFFFU << TIM_ARR_ARR_Pos)             /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR               TIM_ARR_ARR_Msk                              /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define TIM_RCR_REP_Pos           (0U)                                         
#define TIM_RCR_REP_Msk           (0xFFFFU << TIM_RCR_REP_Pos)                 /*!< 0x0000FFFF */
#define TIM_RCR_REP               TIM_RCR_REP_Msk                              /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define TIM_CCR1_CCR1_Pos         (0U)                                         
#define TIM_CCR1_CCR1_Msk         (0xFFFFU << TIM_CCR1_CCR1_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1             TIM_CCR1_CCR1_Msk                            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define TIM_CCR2_CCR2_Pos         (0U)                                         
#define TIM_CCR2_CCR2_Msk         (0xFFFFU << TIM_CCR2_CCR2_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2             TIM_CCR2_CCR2_Msk                            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define TIM_CCR3_CCR3_Pos         (0U)                                         
#define TIM_CCR3_CCR3_Msk         (0xFFFFU << TIM_CCR3_CCR3_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3             TIM_CCR3_CCR3_Msk                            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define TIM_CCR4_CCR4_Pos         (0U)                                         
#define TIM_CCR4_CCR4_Msk         (0xFFFFU << TIM_CCR4_CCR4_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4             TIM_CCR4_CCR4_Msk                            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_CCR5 register  *******************/
#define TIM_CCR5_CCR5_Pos         (0U)                                         
#define TIM_CCR5_CCR5_Msk         (0xFFFFFFFFU << TIM_CCR5_CCR5_Pos)           /*!< 0xFFFFFFFF */
#define TIM_CCR5_CCR5             TIM_CCR5_CCR5_Msk                            /*!<Capture/Compare 5 Value */
#define TIM_CCR5_GC5C1_Pos        (29U)                                        
#define TIM_CCR5_GC5C1_Msk        (0x1U << TIM_CCR5_GC5C1_Pos)                 /*!< 0x20000000 */
#define TIM_CCR5_GC5C1            TIM_CCR5_GC5C1_Msk                           /*!<Group Channel 5 and Channel 1 */
#define TIM_CCR5_GC5C2_Pos        (30U)                                        
#define TIM_CCR5_GC5C2_Msk        (0x1U << TIM_CCR5_GC5C2_Pos)                 /*!< 0x40000000 */
#define TIM_CCR5_GC5C2            TIM_CCR5_GC5C2_Msk                           /*!<Group Channel 5 and Channel 2 */
#define TIM_CCR5_GC5C3_Pos        (31U)                                        
#define TIM_CCR5_GC5C3_Msk        (0x1U << TIM_CCR5_GC5C3_Pos)                 /*!< 0x80000000 */
#define TIM_CCR5_GC5C3            TIM_CCR5_GC5C3_Msk                           /*!<Group Channel 5 and Channel 3 */

/*******************  Bit definition for TIM_CCR6 register  *******************/
#define TIM_CCR6_CCR6_Pos         (0U)                                         
#define TIM_CCR6_CCR6_Msk         (0xFFFFU << TIM_CCR6_CCR6_Pos)               /*!< 0x0000FFFF */
#define TIM_CCR6_CCR6             TIM_CCR6_CCR6_Msk                            /*!<Capture/Compare 6 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define TIM_BDTR_DTG_Pos          (0U)                                         
#define TIM_BDTR_DTG_Msk          (0xFFU << TIM_BDTR_DTG_Pos)                  /*!< 0x000000FF */
#define TIM_BDTR_DTG              TIM_BDTR_DTG_Msk                             /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0            (0x01U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000001 */
#define TIM_BDTR_DTG_1            (0x02U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000002 */
#define TIM_BDTR_DTG_2            (0x04U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000004 */
#define TIM_BDTR_DTG_3            (0x08U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000008 */
#define TIM_BDTR_DTG_4            (0x10U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000010 */
#define TIM_BDTR_DTG_5            (0x20U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000020 */
#define TIM_BDTR_DTG_6            (0x40U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000040 */
#define TIM_BDTR_DTG_7            (0x80U << TIM_BDTR_DTG_Pos)                  /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos         (8U)                                         
#define TIM_BDTR_LOCK_Msk         (0x3U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000300 */
#define TIM_BDTR_LOCK             TIM_BDTR_LOCK_Msk                            /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0           (0x1U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1           (0x2U << TIM_BDTR_LOCK_Pos)                  /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos         (10U)                                        
#define TIM_BDTR_OSSI_Msk         (0x1U << TIM_BDTR_OSSI_Pos)                  /*!< 0x00000400 */
#define TIM_BDTR_OSSI             TIM_BDTR_OSSI_Msk                            /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos         (11U)                                        
#define TIM_BDTR_OSSR_Msk         (0x1U << TIM_BDTR_OSSR_Pos)                  /*!< 0x00000800 */
#define TIM_BDTR_OSSR             TIM_BDTR_OSSR_Msk                            /*!<Off-State Selection for Run mode */
#define TIM_BDTR_BKE_Pos          (12U)                                        
#define TIM_BDTR_BKE_Msk          (0x1U << TIM_BDTR_BKE_Pos)                   /*!< 0x00001000 */
#define TIM_BDTR_BKE              TIM_BDTR_BKE_Msk                             /*!<Break enable for Break1 */
#define TIM_BDTR_BKP_Pos          (13U)                                        
#define TIM_BDTR_BKP_Msk          (0x1U << TIM_BDTR_BKP_Pos)                   /*!< 0x00002000 */
#define TIM_BDTR_BKP              TIM_BDTR_BKP_Msk                             /*!<Break Polarity for Break1 */
#define TIM_BDTR_AOE_Pos          (14U)                                        
#define TIM_BDTR_AOE_Msk          (0x1U << TIM_BDTR_AOE_Pos)                   /*!< 0x00004000 */
#define TIM_BDTR_AOE              TIM_BDTR_AOE_Msk                             /*!<Automatic Output enable */
#define TIM_BDTR_MOE_Pos          (15U)                                        
#define TIM_BDTR_MOE_Msk          (0x1U << TIM_BDTR_MOE_Pos)                   /*!< 0x00008000 */
#define TIM_BDTR_MOE              TIM_BDTR_MOE_Msk                             /*!<Main Output enable */

#define TIM_BDTR_BKF_Pos          (16U)                                        
#define TIM_BDTR_BKF_Msk          (0xFU << TIM_BDTR_BKF_Pos)                   /*!< 0x000F0000 */
#define TIM_BDTR_BKF              TIM_BDTR_BKF_Msk                             /*!<Break Filter for Break1 */
#define TIM_BDTR_BK2F_Pos         (20U)                                        
#define TIM_BDTR_BK2F_Msk         (0xFU << TIM_BDTR_BK2F_Pos)                  /*!< 0x00F00000 */
#define TIM_BDTR_BK2F             TIM_BDTR_BK2F_Msk                            /*!<Break Filter for Break2 */

#define TIM_BDTR_BK2E_Pos         (24U)                                        
#define TIM_BDTR_BK2E_Msk         (0x1U << TIM_BDTR_BK2E_Pos)                  /*!< 0x01000000 */
#define TIM_BDTR_BK2E             TIM_BDTR_BK2E_Msk                            /*!<Break enable for Break2 */
#define TIM_BDTR_BK2P_Pos         (25U)                                        
#define TIM_BDTR_BK2P_Msk         (0x1U << TIM_BDTR_BK2P_Pos)                  /*!< 0x02000000 */
#define TIM_BDTR_BK2P             TIM_BDTR_BK2P_Msk                            /*!<Break Polarity for Break2 */

/*******************  Bit definition for TIM_DCR register  ********************/
#define TIM_DCR_DBA_Pos           (0U)                                         
#define TIM_DCR_DBA_Msk           (0x1FU << TIM_DCR_DBA_Pos)                   /*!< 0x0000001F */
#define TIM_DCR_DBA               TIM_DCR_DBA_Msk                              /*!<DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBA_0             (0x01U << TIM_DCR_DBA_Pos)                   /*!< 0x00000001 */
#define TIM_DCR_DBA_1             (0x02U << TIM_DCR_DBA_Pos)                   /*!< 0x00000002 */
#define TIM_DCR_DBA_2             (0x04U << TIM_DCR_DBA_Pos)                   /*!< 0x00000004 */
#define TIM_DCR_DBA_3             (0x08U << TIM_DCR_DBA_Pos)                   /*!< 0x00000008 */
#define TIM_DCR_DBA_4             (0x10U << TIM_DCR_DBA_Pos)                   /*!< 0x00000010 */

#define TIM_DCR_DBL_Pos           (8U)                                         
#define TIM_DCR_DBL_Msk           (0x1FU << TIM_DCR_DBL_Pos)                   /*!< 0x00001F00 */
#define TIM_DCR_DBL               TIM_DCR_DBL_Msk                              /*!<DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCR_DBL_0             (0x01U << TIM_DCR_DBL_Pos)                   /*!< 0x00000100 */
#define TIM_DCR_DBL_1             (0x02U << TIM_DCR_DBL_Pos)                   /*!< 0x00000200 */
#define TIM_DCR_DBL_2             (0x04U << TIM_DCR_DBL_Pos)                   /*!< 0x00000400 */
#define TIM_DCR_DBL_3             (0x08U << TIM_DCR_DBL_Pos)                   /*!< 0x00000800 */
#define TIM_DCR_DBL_4             (0x10U << TIM_DCR_DBL_Pos)                   /*!< 0x00001000 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define TIM_DMAR_DMAB_Pos         (0U)                                         
#define TIM_DMAR_DMAB_Msk         (0xFFFFU << TIM_DMAR_DMAB_Pos)               /*!< 0x0000FFFF */
#define TIM_DMAR_DMAB             TIM_DMAR_DMAB_Msk                            /*!<DMA register for burst accesses */

/*******************  Bit definition for TIM16_OR register  *********************/
#define TIM16_OR_TI1_RMP_Pos      (6U)                                         
#define TIM16_OR_TI1_RMP_Msk      (0x3U << TIM16_OR_TI1_RMP_Pos)               /*!< 0x000000C0 */
#define TIM16_OR_TI1_RMP          TIM16_OR_TI1_RMP_Msk                         /*!<TI1_RMP[1:0] bits (TIM16 Input 1 remap) */
#define TIM16_OR_TI1_RMP_0        (0x1U << TIM16_OR_TI1_RMP_Pos)               /*!< 0x00000040 */
#define TIM16_OR_TI1_RMP_1        (0x2U << TIM16_OR_TI1_RMP_Pos)               /*!< 0x00000080 */

/*******************  Bit definition for TIM1_OR register  *********************/
#define TIM1_OR_ETR_RMP_Pos      (0U)                                          
#define TIM1_OR_ETR_RMP_Msk      (0xFU << TIM1_OR_ETR_RMP_Pos)                 /*!< 0x0000000F */
#define TIM1_OR_ETR_RMP          TIM1_OR_ETR_RMP_Msk                           /*!<ETR_RMP[3:0] bits (TIM1 ETR remap) */
#define TIM1_OR_ETR_RMP_0        (0x1U << TIM1_OR_ETR_RMP_Pos)                 /*!< 0x00000001 */
#define TIM1_OR_ETR_RMP_1        (0x2U << TIM1_OR_ETR_RMP_Pos)                 /*!< 0x00000002 */
#define TIM1_OR_ETR_RMP_2        (0x4U << TIM1_OR_ETR_RMP_Pos)                 /*!< 0x00000004 */
#define TIM1_OR_ETR_RMP_3        (0x8U << TIM1_OR_ETR_RMP_Pos)                 /*!< 0x00000008 */

/******************  Bit definition for TIM_CCMR3 register  *******************/
#define TIM_CCMR3_OC5FE_Pos       (2U)                                         
#define TIM_CCMR3_OC5FE_Msk       (0x1U << TIM_CCMR3_OC5FE_Pos)                /*!< 0x00000004 */
#define TIM_CCMR3_OC5FE           TIM_CCMR3_OC5FE_Msk                          /*!<Output Compare 5 Fast enable */
#define TIM_CCMR3_OC5PE_Pos       (3U)                                         
#define TIM_CCMR3_OC5PE_Msk       (0x1U << TIM_CCMR3_OC5PE_Pos)                /*!< 0x00000008 */
#define TIM_CCMR3_OC5PE           TIM_CCMR3_OC5PE_Msk                          /*!<Output Compare 5 Preload enable */

#define TIM_CCMR3_OC5M_Pos        (4U)                                         
#define TIM_CCMR3_OC5M_Msk        (0x1007U << TIM_CCMR3_OC5M_Pos)              /*!< 0x00010070 */
#define TIM_CCMR3_OC5M            TIM_CCMR3_OC5M_Msk                           /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
#define TIM_CCMR3_OC5M_0          (0x0001U << TIM_CCMR3_OC5M_Pos)              /*!< 0x00000010 */
#define TIM_CCMR3_OC5M_1          (0x0002U << TIM_CCMR3_OC5M_Pos)              /*!< 0x00000020 */
#define TIM_CCMR3_OC5M_2          (0x0004U << TIM_CCMR3_OC5M_Pos)              /*!< 0x00000040 */
#define TIM_CCMR3_OC5M_3          (0x1000U << TIM_CCMR3_OC5M_Pos)              /*!< 0x00010000 */

#define TIM_CCMR3_OC5CE_Pos       (7U)                                         
#define TIM_CCMR3_OC5CE_Msk       (0x1U << TIM_CCMR3_OC5CE_Pos)                /*!< 0x00000080 */
#define TIM_CCMR3_OC5CE           TIM_CCMR3_OC5CE_Msk                          /*!<Output Compare 5 Clear Enable */

#define TIM_CCMR3_OC6FE_Pos       (10U)                                        
#define TIM_CCMR3_OC6FE_Msk       (0x1U << TIM_CCMR3_OC6FE_Pos)                /*!< 0x00000400 */
#define TIM_CCMR3_OC6FE           TIM_CCMR3_OC6FE_Msk                          /*!<Output Compare 6 Fast enable */
#define TIM_CCMR3_OC6PE_Pos       (11U)                                        
#define TIM_CCMR3_OC6PE_Msk       (0x1U << TIM_CCMR3_OC6PE_Pos)                /*!< 0x00000800 */
#define TIM_CCMR3_OC6PE           TIM_CCMR3_OC6PE_Msk                          /*!<Output Compare 6 Preload enable */

#define TIM_CCMR3_OC6M_Pos        (12U)                                        
#define TIM_CCMR3_OC6M_Msk        (0x1007U << TIM_CCMR3_OC6M_Pos)              /*!< 0x01007000 */
#define TIM_CCMR3_OC6M            TIM_CCMR3_OC6M_Msk                           /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
#define TIM_CCMR3_OC6M_0          (0x0001U << TIM_CCMR3_OC6M_Pos)              /*!< 0x00001000 */
#define TIM_CCMR3_OC6M_1          (0x0002U << TIM_CCMR3_OC6M_Pos)              /*!< 0x00002000 */
#define TIM_CCMR3_OC6M_2          (0x0004U << TIM_CCMR3_OC6M_Pos)              /*!< 0x00004000 */
#define TIM_CCMR3_OC6M_3          (0x1000U << TIM_CCMR3_OC6M_Pos)              /*!< 0x01000000 */

#define TIM_CCMR3_OC6CE_Pos       (15U)                                        
#define TIM_CCMR3_OC6CE_Msk       (0x1U << TIM_CCMR3_OC6CE_Pos)                /*!< 0x00008000 */
#define TIM_CCMR3_OC6CE           TIM_CCMR3_OC6CE_Msk                          /*!<Output Compare 6 Clear Enable */

/******************************************************************************/
/*                                                                            */
/*                          Touch Sensing Controller (TSC)                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TSC_CR register  *********************/
#define TSC_CR_TSCE_Pos          (0U)                                          
#define TSC_CR_TSCE_Msk          (0x1U << TSC_CR_TSCE_Pos)                     /*!< 0x00000001 */
#define TSC_CR_TSCE              TSC_CR_TSCE_Msk                               /*!<Touch sensing controller enable */
#define TSC_CR_START_Pos         (1U)                                          
#define TSC_CR_START_Msk         (0x1U << TSC_CR_START_Pos)                    /*!< 0x00000002 */
#define TSC_CR_START             TSC_CR_START_Msk                              /*!<Start acquisition */
#define TSC_CR_AM_Pos            (2U)                                          
#define TSC_CR_AM_Msk            (0x1U << TSC_CR_AM_Pos)                       /*!< 0x00000004 */
#define TSC_CR_AM                TSC_CR_AM_Msk                                 /*!<Acquisition mode */
#define TSC_CR_SYNCPOL_Pos       (3U)                                          
#define TSC_CR_SYNCPOL_Msk       (0x1U << TSC_CR_SYNCPOL_Pos)                  /*!< 0x00000008 */
#define TSC_CR_SYNCPOL           TSC_CR_SYNCPOL_Msk                            /*!<Synchronization pin polarity */
#define TSC_CR_IODEF_Pos         (4U)                                          
#define TSC_CR_IODEF_Msk         (0x1U << TSC_CR_IODEF_Pos)                    /*!< 0x00000010 */
#define TSC_CR_IODEF             TSC_CR_IODEF_Msk                              /*!<IO default mode */

#define TSC_CR_MCV_Pos           (5U)                                          
#define TSC_CR_MCV_Msk           (0x7U << TSC_CR_MCV_Pos)                      /*!< 0x000000E0 */
#define TSC_CR_MCV               TSC_CR_MCV_Msk                                /*!<MCV[2:0] bits (Max Count Value) */
#define TSC_CR_MCV_0             (0x1U << TSC_CR_MCV_Pos)                      /*!< 0x00000020 */
#define TSC_CR_MCV_1             (0x2U << TSC_CR_MCV_Pos)                      /*!< 0x00000040 */
#define TSC_CR_MCV_2             (0x4U << TSC_CR_MCV_Pos)                      /*!< 0x00000080 */

#define TSC_CR_PGPSC_Pos         (12U)                                         
#define TSC_CR_PGPSC_Msk         (0x7U << TSC_CR_PGPSC_Pos)                    /*!< 0x00007000 */
#define TSC_CR_PGPSC             TSC_CR_PGPSC_Msk                              /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
#define TSC_CR_PGPSC_0           (0x1U << TSC_CR_PGPSC_Pos)                    /*!< 0x00001000 */
#define TSC_CR_PGPSC_1           (0x2U << TSC_CR_PGPSC_Pos)                    /*!< 0x00002000 */
#define TSC_CR_PGPSC_2           (0x4U << TSC_CR_PGPSC_Pos)                    /*!< 0x00004000 */

#define TSC_CR_SSPSC_Pos         (15U)                                         
#define TSC_CR_SSPSC_Msk         (0x1U << TSC_CR_SSPSC_Pos)                    /*!< 0x00008000 */
#define TSC_CR_SSPSC             TSC_CR_SSPSC_Msk                              /*!<Spread Spectrum Prescaler */
#define TSC_CR_SSE_Pos           (16U)                                         
#define TSC_CR_SSE_Msk           (0x1U << TSC_CR_SSE_Pos)                      /*!< 0x00010000 */
#define TSC_CR_SSE               TSC_CR_SSE_Msk                                /*!<Spread Spectrum Enable */

#define TSC_CR_SSD_Pos           (17U)                                         
#define TSC_CR_SSD_Msk           (0x7FU << TSC_CR_SSD_Pos)                     /*!< 0x00FE0000 */
#define TSC_CR_SSD               TSC_CR_SSD_Msk                                /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
#define TSC_CR_SSD_0             (0x01U << TSC_CR_SSD_Pos)                     /*!< 0x00020000 */
#define TSC_CR_SSD_1             (0x02U << TSC_CR_SSD_Pos)                     /*!< 0x00040000 */
#define TSC_CR_SSD_2             (0x04U << TSC_CR_SSD_Pos)                     /*!< 0x00080000 */
#define TSC_CR_SSD_3             (0x08U << TSC_CR_SSD_Pos)                     /*!< 0x00100000 */
#define TSC_CR_SSD_4             (0x10U << TSC_CR_SSD_Pos)                     /*!< 0x00200000 */
#define TSC_CR_SSD_5             (0x20U << TSC_CR_SSD_Pos)                     /*!< 0x00400000 */
#define TSC_CR_SSD_6             (0x40U << TSC_CR_SSD_Pos)                     /*!< 0x00800000 */

#define TSC_CR_CTPL_Pos          (24U)                                         
#define TSC_CR_CTPL_Msk          (0xFU << TSC_CR_CTPL_Pos)                     /*!< 0x0F000000 */
#define TSC_CR_CTPL              TSC_CR_CTPL_Msk                               /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
#define TSC_CR_CTPL_0            (0x1U << TSC_CR_CTPL_Pos)                     /*!< 0x01000000 */
#define TSC_CR_CTPL_1            (0x2U << TSC_CR_CTPL_Pos)                     /*!< 0x02000000 */
#define TSC_CR_CTPL_2            (0x4U << TSC_CR_CTPL_Pos)                     /*!< 0x04000000 */
#define TSC_CR_CTPL_3            (0x8U << TSC_CR_CTPL_Pos)                     /*!< 0x08000000 */

#define TSC_CR_CTPH_Pos          (28U)                                         
#define TSC_CR_CTPH_Msk          (0xFU << TSC_CR_CTPH_Pos)                     /*!< 0xF0000000 */
#define TSC_CR_CTPH              TSC_CR_CTPH_Msk                               /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
#define TSC_CR_CTPH_0            (0x1U << TSC_CR_CTPH_Pos)                     /*!< 0x10000000 */
#define TSC_CR_CTPH_1            (0x2U << TSC_CR_CTPH_Pos)                     /*!< 0x20000000 */
#define TSC_CR_CTPH_2            (0x4U << TSC_CR_CTPH_Pos)                     /*!< 0x40000000 */
#define TSC_CR_CTPH_3            (0x8U << TSC_CR_CTPH_Pos)                     /*!< 0x80000000 */

/*******************  Bit definition for TSC_IER register  ********************/
#define TSC_IER_EOAIE_Pos        (0U)                                          
#define TSC_IER_EOAIE_Msk        (0x1U << TSC_IER_EOAIE_Pos)                   /*!< 0x00000001 */
#define TSC_IER_EOAIE            TSC_IER_EOAIE_Msk                             /*!<End of acquisition interrupt enable */
#define TSC_IER_MCEIE_Pos        (1U)                                          
#define TSC_IER_MCEIE_Msk        (0x1U << TSC_IER_MCEIE_Pos)                   /*!< 0x00000002 */
#define TSC_IER_MCEIE            TSC_IER_MCEIE_Msk                             /*!<Max count error interrupt enable */

/*******************  Bit definition for TSC_ICR register  ********************/
#define TSC_ICR_EOAIC_Pos        (0U)                                          
#define TSC_ICR_EOAIC_Msk        (0x1U << TSC_ICR_EOAIC_Pos)                   /*!< 0x00000001 */
#define TSC_ICR_EOAIC            TSC_ICR_EOAIC_Msk                             /*!<End of acquisition interrupt clear */
#define TSC_ICR_MCEIC_Pos        (1U)                                          
#define TSC_ICR_MCEIC_Msk        (0x1U << TSC_ICR_MCEIC_Pos)                   /*!< 0x00000002 */
#define TSC_ICR_MCEIC            TSC_ICR_MCEIC_Msk                             /*!<Max count error interrupt clear */

/*******************  Bit definition for TSC_ISR register  ********************/
#define TSC_ISR_EOAF_Pos         (0U)                                          
#define TSC_ISR_EOAF_Msk         (0x1U << TSC_ISR_EOAF_Pos)                    /*!< 0x00000001 */
#define TSC_ISR_EOAF             TSC_ISR_EOAF_Msk                              /*!<End of acquisition flag */
#define TSC_ISR_MCEF_Pos         (1U)                                          
#define TSC_ISR_MCEF_Msk         (0x1U << TSC_ISR_MCEF_Pos)                    /*!< 0x00000002 */
#define TSC_ISR_MCEF             TSC_ISR_MCEF_Msk                              /*!<Max count error flag */

/*******************  Bit definition for TSC_IOHCR register  ******************/
#define TSC_IOHCR_G1_IO1_Pos     (0U)                                          
#define TSC_IOHCR_G1_IO1_Msk     (0x1U << TSC_IOHCR_G1_IO1_Pos)                /*!< 0x00000001 */
#define TSC_IOHCR_G1_IO1         TSC_IOHCR_G1_IO1_Msk                          /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO2_Pos     (1U)                                          
#define TSC_IOHCR_G1_IO2_Msk     (0x1U << TSC_IOHCR_G1_IO2_Pos)                /*!< 0x00000002 */
#define TSC_IOHCR_G1_IO2         TSC_IOHCR_G1_IO2_Msk                          /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO3_Pos     (2U)                                          
#define TSC_IOHCR_G1_IO3_Msk     (0x1U << TSC_IOHCR_G1_IO3_Pos)                /*!< 0x00000004 */
#define TSC_IOHCR_G1_IO3         TSC_IOHCR_G1_IO3_Msk                          /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO4_Pos     (3U)                                          
#define TSC_IOHCR_G1_IO4_Msk     (0x1U << TSC_IOHCR_G1_IO4_Pos)                /*!< 0x00000008 */
#define TSC_IOHCR_G1_IO4         TSC_IOHCR_G1_IO4_Msk                          /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO1_Pos     (4U)                                          
#define TSC_IOHCR_G2_IO1_Msk     (0x1U << TSC_IOHCR_G2_IO1_Pos)                /*!< 0x00000010 */
#define TSC_IOHCR_G2_IO1         TSC_IOHCR_G2_IO1_Msk                          /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO2_Pos     (5U)                                          
#define TSC_IOHCR_G2_IO2_Msk     (0x1U << TSC_IOHCR_G2_IO2_Pos)                /*!< 0x00000020 */
#define TSC_IOHCR_G2_IO2         TSC_IOHCR_G2_IO2_Msk                          /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO3_Pos     (6U)                                          
#define TSC_IOHCR_G2_IO3_Msk     (0x1U << TSC_IOHCR_G2_IO3_Pos)                /*!< 0x00000040 */
#define TSC_IOHCR_G2_IO3         TSC_IOHCR_G2_IO3_Msk                          /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO4_Pos     (7U)                                          
#define TSC_IOHCR_G2_IO4_Msk     (0x1U << TSC_IOHCR_G2_IO4_Pos)                /*!< 0x00000080 */
#define TSC_IOHCR_G2_IO4         TSC_IOHCR_G2_IO4_Msk                          /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO1_Pos     (8U)                                          
#define TSC_IOHCR_G3_IO1_Msk     (0x1U << TSC_IOHCR_G3_IO1_Pos)                /*!< 0x00000100 */
#define TSC_IOHCR_G3_IO1         TSC_IOHCR_G3_IO1_Msk                          /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO2_Pos     (9U)                                          
#define TSC_IOHCR_G3_IO2_Msk     (0x1U << TSC_IOHCR_G3_IO2_Pos)                /*!< 0x00000200 */
#define TSC_IOHCR_G3_IO2         TSC_IOHCR_G3_IO2_Msk                          /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO3_Pos     (10U)                                         
#define TSC_IOHCR_G3_IO3_Msk     (0x1U << TSC_IOHCR_G3_IO3_Pos)                /*!< 0x00000400 */
#define TSC_IOHCR_G3_IO3         TSC_IOHCR_G3_IO3_Msk                          /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO4_Pos     (11U)                                         
#define TSC_IOHCR_G3_IO4_Msk     (0x1U << TSC_IOHCR_G3_IO4_Pos)                /*!< 0x00000800 */
#define TSC_IOHCR_G3_IO4         TSC_IOHCR_G3_IO4_Msk                          /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO1_Pos     (12U)                                         
#define TSC_IOHCR_G4_IO1_Msk     (0x1U << TSC_IOHCR_G4_IO1_Pos)                /*!< 0x00001000 */
#define TSC_IOHCR_G4_IO1         TSC_IOHCR_G4_IO1_Msk                          /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO2_Pos     (13U)                                         
#define TSC_IOHCR_G4_IO2_Msk     (0x1U << TSC_IOHCR_G4_IO2_Pos)                /*!< 0x00002000 */
#define TSC_IOHCR_G4_IO2         TSC_IOHCR_G4_IO2_Msk                          /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO3_Pos     (14U)                                         
#define TSC_IOHCR_G4_IO3_Msk     (0x1U << TSC_IOHCR_G4_IO3_Pos)                /*!< 0x00004000 */
#define TSC_IOHCR_G4_IO3         TSC_IOHCR_G4_IO3_Msk                          /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO4_Pos     (15U)                                         
#define TSC_IOHCR_G4_IO4_Msk     (0x1U << TSC_IOHCR_G4_IO4_Pos)                /*!< 0x00008000 */
#define TSC_IOHCR_G4_IO4         TSC_IOHCR_G4_IO4_Msk                          /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO1_Pos     (16U)                                         
#define TSC_IOHCR_G5_IO1_Msk     (0x1U << TSC_IOHCR_G5_IO1_Pos)                /*!< 0x00010000 */
#define TSC_IOHCR_G5_IO1         TSC_IOHCR_G5_IO1_Msk                          /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO2_Pos     (17U)                                         
#define TSC_IOHCR_G5_IO2_Msk     (0x1U << TSC_IOHCR_G5_IO2_Pos)                /*!< 0x00020000 */
#define TSC_IOHCR_G5_IO2         TSC_IOHCR_G5_IO2_Msk                          /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO3_Pos     (18U)                                         
#define TSC_IOHCR_G5_IO3_Msk     (0x1U << TSC_IOHCR_G5_IO3_Pos)                /*!< 0x00040000 */
#define TSC_IOHCR_G5_IO3         TSC_IOHCR_G5_IO3_Msk                          /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO4_Pos     (19U)                                         
#define TSC_IOHCR_G5_IO4_Msk     (0x1U << TSC_IOHCR_G5_IO4_Pos)                /*!< 0x00080000 */
#define TSC_IOHCR_G5_IO4         TSC_IOHCR_G5_IO4_Msk                          /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO1_Pos     (20U)                                         
#define TSC_IOHCR_G6_IO1_Msk     (0x1U << TSC_IOHCR_G6_IO1_Pos)                /*!< 0x00100000 */
#define TSC_IOHCR_G6_IO1         TSC_IOHCR_G6_IO1_Msk                          /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO2_Pos     (21U)                                         
#define TSC_IOHCR_G6_IO2_Msk     (0x1U << TSC_IOHCR_G6_IO2_Pos)                /*!< 0x00200000 */
#define TSC_IOHCR_G6_IO2         TSC_IOHCR_G6_IO2_Msk                          /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO3_Pos     (22U)                                         
#define TSC_IOHCR_G6_IO3_Msk     (0x1U << TSC_IOHCR_G6_IO3_Pos)                /*!< 0x00400000 */
#define TSC_IOHCR_G6_IO3         TSC_IOHCR_G6_IO3_Msk                          /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO4_Pos     (23U)                                         
#define TSC_IOHCR_G6_IO4_Msk     (0x1U << TSC_IOHCR_G6_IO4_Pos)                /*!< 0x00800000 */
#define TSC_IOHCR_G6_IO4         TSC_IOHCR_G6_IO4_Msk                          /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO1_Pos     (24U)                                         
#define TSC_IOHCR_G7_IO1_Msk     (0x1U << TSC_IOHCR_G7_IO1_Pos)                /*!< 0x01000000 */
#define TSC_IOHCR_G7_IO1         TSC_IOHCR_G7_IO1_Msk                          /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO2_Pos     (25U)                                         
#define TSC_IOHCR_G7_IO2_Msk     (0x1U << TSC_IOHCR_G7_IO2_Pos)                /*!< 0x02000000 */
#define TSC_IOHCR_G7_IO2         TSC_IOHCR_G7_IO2_Msk                          /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO3_Pos     (26U)                                         
#define TSC_IOHCR_G7_IO3_Msk     (0x1U << TSC_IOHCR_G7_IO3_Pos)                /*!< 0x04000000 */
#define TSC_IOHCR_G7_IO3         TSC_IOHCR_G7_IO3_Msk                          /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO4_Pos     (27U)                                         
#define TSC_IOHCR_G7_IO4_Msk     (0x1U << TSC_IOHCR_G7_IO4_Pos)                /*!< 0x08000000 */
#define TSC_IOHCR_G7_IO4         TSC_IOHCR_G7_IO4_Msk                          /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO1_Pos     (28U)                                         
#define TSC_IOHCR_G8_IO1_Msk     (0x1U << TSC_IOHCR_G8_IO1_Pos)                /*!< 0x10000000 */
#define TSC_IOHCR_G8_IO1         TSC_IOHCR_G8_IO1_Msk                          /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO2_Pos     (29U)                                         
#define TSC_IOHCR_G8_IO2_Msk     (0x1U << TSC_IOHCR_G8_IO2_Pos)                /*!< 0x20000000 */
#define TSC_IOHCR_G8_IO2         TSC_IOHCR_G8_IO2_Msk                          /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO3_Pos     (30U)                                         
#define TSC_IOHCR_G8_IO3_Msk     (0x1U << TSC_IOHCR_G8_IO3_Pos)                /*!< 0x40000000 */
#define TSC_IOHCR_G8_IO3         TSC_IOHCR_G8_IO3_Msk                          /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G8_IO4_Pos     (31U)                                         
#define TSC_IOHCR_G8_IO4_Msk     (0x1U << TSC_IOHCR_G8_IO4_Pos)                /*!< 0x80000000 */
#define TSC_IOHCR_G8_IO4         TSC_IOHCR_G8_IO4_Msk                          /*!<GROUP8_IO4 schmitt trigger hysteresis mode */

/*******************  Bit definition for TSC_IOASCR register  *****************/
#define TSC_IOASCR_G1_IO1_Pos    (0U)                                          
#define TSC_IOASCR_G1_IO1_Msk    (0x1U << TSC_IOASCR_G1_IO1_Pos)               /*!< 0x00000001 */
#define TSC_IOASCR_G1_IO1        TSC_IOASCR_G1_IO1_Msk                         /*!<GROUP1_IO1 analog switch enable */
#define TSC_IOASCR_G1_IO2_Pos    (1U)                                          
#define TSC_IOASCR_G1_IO2_Msk    (0x1U << TSC_IOASCR_G1_IO2_Pos)               /*!< 0x00000002 */
#define TSC_IOASCR_G1_IO2        TSC_IOASCR_G1_IO2_Msk                         /*!<GROUP1_IO2 analog switch enable */
#define TSC_IOASCR_G1_IO3_Pos    (2U)                                          
#define TSC_IOASCR_G1_IO3_Msk    (0x1U << TSC_IOASCR_G1_IO3_Pos)               /*!< 0x00000004 */
#define TSC_IOASCR_G1_IO3        TSC_IOASCR_G1_IO3_Msk                         /*!<GROUP1_IO3 analog switch enable */
#define TSC_IOASCR_G1_IO4_Pos    (3U)                                          
#define TSC_IOASCR_G1_IO4_Msk    (0x1U << TSC_IOASCR_G1_IO4_Pos)               /*!< 0x00000008 */
#define TSC_IOASCR_G1_IO4        TSC_IOASCR_G1_IO4_Msk                         /*!<GROUP1_IO4 analog switch enable */
#define TSC_IOASCR_G2_IO1_Pos    (4U)                                          
#define TSC_IOASCR_G2_IO1_Msk    (0x1U << TSC_IOASCR_G2_IO1_Pos)               /*!< 0x00000010 */
#define TSC_IOASCR_G2_IO1        TSC_IOASCR_G2_IO1_Msk                         /*!<GROUP2_IO1 analog switch enable */
#define TSC_IOASCR_G2_IO2_Pos    (5U)                                          
#define TSC_IOASCR_G2_IO2_Msk    (0x1U << TSC_IOASCR_G2_IO2_Pos)               /*!< 0x00000020 */
#define TSC_IOASCR_G2_IO2        TSC_IOASCR_G2_IO2_Msk                         /*!<GROUP2_IO2 analog switch enable */
#define TSC_IOASCR_G2_IO3_Pos    (6U)                                          
#define TSC_IOASCR_G2_IO3_Msk    (0x1U << TSC_IOASCR_G2_IO3_Pos)               /*!< 0x00000040 */
#define TSC_IOASCR_G2_IO3        TSC_IOASCR_G2_IO3_Msk                         /*!<GROUP2_IO3 analog switch enable */
#define TSC_IOASCR_G2_IO4_Pos    (7U)                                          
#define TSC_IOASCR_G2_IO4_Msk    (0x1U << TSC_IOASCR_G2_IO4_Pos)               /*!< 0x00000080 */
#define TSC_IOASCR_G2_IO4        TSC_IOASCR_G2_IO4_Msk                         /*!<GROUP2_IO4 analog switch enable */
#define TSC_IOASCR_G3_IO1_Pos    (8U)                                          
#define TSC_IOASCR_G3_IO1_Msk    (0x1U << TSC_IOASCR_G3_IO1_Pos)               /*!< 0x00000100 */
#define TSC_IOASCR_G3_IO1        TSC_IOASCR_G3_IO1_Msk                         /*!<GROUP3_IO1 analog switch enable */
#define TSC_IOASCR_G3_IO2_Pos    (9U)                                          
#define TSC_IOASCR_G3_IO2_Msk    (0x1U << TSC_IOASCR_G3_IO2_Pos)               /*!< 0x00000200 */
#define TSC_IOASCR_G3_IO2        TSC_IOASCR_G3_IO2_Msk                         /*!<GROUP3_IO2 analog switch enable */
#define TSC_IOASCR_G3_IO3_Pos    (10U)                                         
#define TSC_IOASCR_G3_IO3_Msk    (0x1U << TSC_IOASCR_G3_IO3_Pos)               /*!< 0x00000400 */
#define TSC_IOASCR_G3_IO3        TSC_IOASCR_G3_IO3_Msk                         /*!<GROUP3_IO3 analog switch enable */
#define TSC_IOASCR_G3_IO4_Pos    (11U)                                         
#define TSC_IOASCR_G3_IO4_Msk    (0x1U << TSC_IOASCR_G3_IO4_Pos)               /*!< 0x00000800 */
#define TSC_IOASCR_G3_IO4        TSC_IOASCR_G3_IO4_Msk                         /*!<GROUP3_IO4 analog switch enable */
#define TSC_IOASCR_G4_IO1_Pos    (12U)                                         
#define TSC_IOASCR_G4_IO1_Msk    (0x1U << TSC_IOASCR_G4_IO1_Pos)               /*!< 0x00001000 */
#define TSC_IOASCR_G4_IO1        TSC_IOASCR_G4_IO1_Msk                         /*!<GROUP4_IO1 analog switch enable */
#define TSC_IOASCR_G4_IO2_Pos    (13U)                                         
#define TSC_IOASCR_G4_IO2_Msk    (0x1U << TSC_IOASCR_G4_IO2_Pos)               /*!< 0x00002000 */
#define TSC_IOASCR_G4_IO2        TSC_IOASCR_G4_IO2_Msk                         /*!<GROUP4_IO2 analog switch enable */
#define TSC_IOASCR_G4_IO3_Pos    (14U)                                         
#define TSC_IOASCR_G4_IO3_Msk    (0x1U << TSC_IOASCR_G4_IO3_Pos)               /*!< 0x00004000 */
#define TSC_IOASCR_G4_IO3        TSC_IOASCR_G4_IO3_Msk                         /*!<GROUP4_IO3 analog switch enable */
#define TSC_IOASCR_G4_IO4_Pos    (15U)                                         
#define TSC_IOASCR_G4_IO4_Msk    (0x1U << TSC_IOASCR_G4_IO4_Pos)               /*!< 0x00008000 */
#define TSC_IOASCR_G4_IO4        TSC_IOASCR_G4_IO4_Msk                         /*!<GROUP4_IO4 analog switch enable */
#define TSC_IOASCR_G5_IO1_Pos    (16U)                                         
#define TSC_IOASCR_G5_IO1_Msk    (0x1U << TSC_IOASCR_G5_IO1_Pos)               /*!< 0x00010000 */
#define TSC_IOASCR_G5_IO1        TSC_IOASCR_G5_IO1_Msk                         /*!<GROUP5_IO1 analog switch enable */
#define TSC_IOASCR_G5_IO2_Pos    (17U)                                         
#define TSC_IOASCR_G5_IO2_Msk    (0x1U << TSC_IOASCR_G5_IO2_Pos)               /*!< 0x00020000 */
#define TSC_IOASCR_G5_IO2        TSC_IOASCR_G5_IO2_Msk                         /*!<GROUP5_IO2 analog switch enable */
#define TSC_IOASCR_G5_IO3_Pos    (18U)                                         
#define TSC_IOASCR_G5_IO3_Msk    (0x1U << TSC_IOASCR_G5_IO3_Pos)               /*!< 0x00040000 */
#define TSC_IOASCR_G5_IO3        TSC_IOASCR_G5_IO3_Msk                         /*!<GROUP5_IO3 analog switch enable */
#define TSC_IOASCR_G5_IO4_Pos    (19U)                                         
#define TSC_IOASCR_G5_IO4_Msk    (0x1U << TSC_IOASCR_G5_IO4_Pos)               /*!< 0x00080000 */
#define TSC_IOASCR_G5_IO4        TSC_IOASCR_G5_IO4_Msk                         /*!<GROUP5_IO4 analog switch enable */
#define TSC_IOASCR_G6_IO1_Pos    (20U)                                         
#define TSC_IOASCR_G6_IO1_Msk    (0x1U << TSC_IOASCR_G6_IO1_Pos)               /*!< 0x00100000 */
#define TSC_IOASCR_G6_IO1        TSC_IOASCR_G6_IO1_Msk                         /*!<GROUP6_IO1 analog switch enable */
#define TSC_IOASCR_G6_IO2_Pos    (21U)                                         
#define TSC_IOASCR_G6_IO2_Msk    (0x1U << TSC_IOASCR_G6_IO2_Pos)               /*!< 0x00200000 */
#define TSC_IOASCR_G6_IO2        TSC_IOASCR_G6_IO2_Msk                         /*!<GROUP6_IO2 analog switch enable */
#define TSC_IOASCR_G6_IO3_Pos    (22U)                                         
#define TSC_IOASCR_G6_IO3_Msk    (0x1U << TSC_IOASCR_G6_IO3_Pos)               /*!< 0x00400000 */
#define TSC_IOASCR_G6_IO3        TSC_IOASCR_G6_IO3_Msk                         /*!<GROUP6_IO3 analog switch enable */
#define TSC_IOASCR_G6_IO4_Pos    (23U)                                         
#define TSC_IOASCR_G6_IO4_Msk    (0x1U << TSC_IOASCR_G6_IO4_Pos)               /*!< 0x00800000 */
#define TSC_IOASCR_G6_IO4        TSC_IOASCR_G6_IO4_Msk                         /*!<GROUP6_IO4 analog switch enable */
#define TSC_IOASCR_G7_IO1_Pos    (24U)                                         
#define TSC_IOASCR_G7_IO1_Msk    (0x1U << TSC_IOASCR_G7_IO1_Pos)               /*!< 0x01000000 */
#define TSC_IOASCR_G7_IO1        TSC_IOASCR_G7_IO1_Msk                         /*!<GROUP7_IO1 analog switch enable */
#define TSC_IOASCR_G7_IO2_Pos    (25U)                                         
#define TSC_IOASCR_G7_IO2_Msk    (0x1U << TSC_IOASCR_G7_IO2_Pos)               /*!< 0x02000000 */
#define TSC_IOASCR_G7_IO2        TSC_IOASCR_G7_IO2_Msk                         /*!<GROUP7_IO2 analog switch enable */
#define TSC_IOASCR_G7_IO3_Pos    (26U)                                         
#define TSC_IOASCR_G7_IO3_Msk    (0x1U << TSC_IOASCR_G7_IO3_Pos)               /*!< 0x04000000 */
#define TSC_IOASCR_G7_IO3        TSC_IOASCR_G7_IO3_Msk                         /*!<GROUP7_IO3 analog switch enable */
#define TSC_IOASCR_G7_IO4_Pos    (27U)                                         
#define TSC_IOASCR_G7_IO4_Msk    (0x1U << TSC_IOASCR_G7_IO4_Pos)               /*!< 0x08000000 */
#define TSC_IOASCR_G7_IO4        TSC_IOASCR_G7_IO4_Msk                         /*!<GROUP7_IO4 analog switch enable */
#define TSC_IOASCR_G8_IO1_Pos    (28U)                                         
#define TSC_IOASCR_G8_IO1_Msk    (0x1U << TSC_IOASCR_G8_IO1_Pos)               /*!< 0x10000000 */
#define TSC_IOASCR_G8_IO1        TSC_IOASCR_G8_IO1_Msk                         /*!<GROUP8_IO1 analog switch enable */
#define TSC_IOASCR_G8_IO2_Pos    (29U)                                         
#define TSC_IOASCR_G8_IO2_Msk    (0x1U << TSC_IOASCR_G8_IO2_Pos)               /*!< 0x20000000 */
#define TSC_IOASCR_G8_IO2        TSC_IOASCR_G8_IO2_Msk                         /*!<GROUP8_IO2 analog switch enable */
#define TSC_IOASCR_G8_IO3_Pos    (30U)                                         
#define TSC_IOASCR_G8_IO3_Msk    (0x1U << TSC_IOASCR_G8_IO3_Pos)               /*!< 0x40000000 */
#define TSC_IOASCR_G8_IO3        TSC_IOASCR_G8_IO3_Msk                         /*!<GROUP8_IO3 analog switch enable */
#define TSC_IOASCR_G8_IO4_Pos    (31U)                                         
#define TSC_IOASCR_G8_IO4_Msk    (0x1U << TSC_IOASCR_G8_IO4_Pos)               /*!< 0x80000000 */
#define TSC_IOASCR_G8_IO4        TSC_IOASCR_G8_IO4_Msk                         /*!<GROUP8_IO4 analog switch enable */

/*******************  Bit definition for TSC_IOSCR register  ******************/
#define TSC_IOSCR_G1_IO1_Pos     (0U)                                          
#define TSC_IOSCR_G1_IO1_Msk     (0x1U << TSC_IOSCR_G1_IO1_Pos)                /*!< 0x00000001 */
#define TSC_IOSCR_G1_IO1         TSC_IOSCR_G1_IO1_Msk                          /*!<GROUP1_IO1 sampling mode */
#define TSC_IOSCR_G1_IO2_Pos     (1U)                                          
#define TSC_IOSCR_G1_IO2_Msk     (0x1U << TSC_IOSCR_G1_IO2_Pos)                /*!< 0x00000002 */
#define TSC_IOSCR_G1_IO2         TSC_IOSCR_G1_IO2_Msk                          /*!<GROUP1_IO2 sampling mode */
#define TSC_IOSCR_G1_IO3_Pos     (2U)                                          
#define TSC_IOSCR_G1_IO3_Msk     (0x1U << TSC_IOSCR_G1_IO3_Pos)                /*!< 0x00000004 */
#define TSC_IOSCR_G1_IO3         TSC_IOSCR_G1_IO3_Msk                          /*!<GROUP1_IO3 sampling mode */
#define TSC_IOSCR_G1_IO4_Pos     (3U)                                          
#define TSC_IOSCR_G1_IO4_Msk     (0x1U << TSC_IOSCR_G1_IO4_Pos)                /*!< 0x00000008 */
#define TSC_IOSCR_G1_IO4         TSC_IOSCR_G1_IO4_Msk                          /*!<GROUP1_IO4 sampling mode */
#define TSC_IOSCR_G2_IO1_Pos     (4U)                                          
#define TSC_IOSCR_G2_IO1_Msk     (0x1U << TSC_IOSCR_G2_IO1_Pos)                /*!< 0x00000010 */
#define TSC_IOSCR_G2_IO1         TSC_IOSCR_G2_IO1_Msk                          /*!<GROUP2_IO1 sampling mode */
#define TSC_IOSCR_G2_IO2_Pos     (5U)                                          
#define TSC_IOSCR_G2_IO2_Msk     (0x1U << TSC_IOSCR_G2_IO2_Pos)                /*!< 0x00000020 */
#define TSC_IOSCR_G2_IO2         TSC_IOSCR_G2_IO2_Msk                          /*!<GROUP2_IO2 sampling mode */
#define TSC_IOSCR_G2_IO3_Pos     (6U)                                          
#define TSC_IOSCR_G2_IO3_Msk     (0x1U << TSC_IOSCR_G2_IO3_Pos)                /*!< 0x00000040 */
#define TSC_IOSCR_G2_IO3         TSC_IOSCR_G2_IO3_Msk                          /*!<GROUP2_IO3 sampling mode */
#define TSC_IOSCR_G2_IO4_Pos     (7U)                                          
#define TSC_IOSCR_G2_IO4_Msk     (0x1U << TSC_IOSCR_G2_IO4_Pos)                /*!< 0x00000080 */
#define TSC_IOSCR_G2_IO4         TSC_IOSCR_G2_IO4_Msk                          /*!<GROUP2_IO4 sampling mode */
#define TSC_IOSCR_G3_IO1_Pos     (8U)                                          
#define TSC_IOSCR_G3_IO1_Msk     (0x1U << TSC_IOSCR_G3_IO1_Pos)                /*!< 0x00000100 */
#define TSC_IOSCR_G3_IO1         TSC_IOSCR_G3_IO1_Msk                          /*!<GROUP3_IO1 sampling mode */
#define TSC_IOSCR_G3_IO2_Pos     (9U)                                          
#define TSC_IOSCR_G3_IO2_Msk     (0x1U << TSC_IOSCR_G3_IO2_Pos)                /*!< 0x00000200 */
#define TSC_IOSCR_G3_IO2         TSC_IOSCR_G3_IO2_Msk                          /*!<GROUP3_IO2 sampling mode */
#define TSC_IOSCR_G3_IO3_Pos     (10U)                                         
#define TSC_IOSCR_G3_IO3_Msk     (0x1U << TSC_IOSCR_G3_IO3_Pos)                /*!< 0x00000400 */
#define TSC_IOSCR_G3_IO3         TSC_IOSCR_G3_IO3_Msk                          /*!<GROUP3_IO3 sampling mode */
#define TSC_IOSCR_G3_IO4_Pos     (11U)                                         
#define TSC_IOSCR_G3_IO4_Msk     (0x1U << TSC_IOSCR_G3_IO4_Pos)                /*!< 0x00000800 */
#define TSC_IOSCR_G3_IO4         TSC_IOSCR_G3_IO4_Msk                          /*!<GROUP3_IO4 sampling mode */
#define TSC_IOSCR_G4_IO1_Pos     (12U)                                         
#define TSC_IOSCR_G4_IO1_Msk     (0x1U << TSC_IOSCR_G4_IO1_Pos)                /*!< 0x00001000 */
#define TSC_IOSCR_G4_IO1         TSC_IOSCR_G4_IO1_Msk                          /*!<GROUP4_IO1 sampling mode */
#define TSC_IOSCR_G4_IO2_Pos     (13U)                                         
#define TSC_IOSCR_G4_IO2_Msk     (0x1U << TSC_IOSCR_G4_IO2_Pos)                /*!< 0x00002000 */
#define TSC_IOSCR_G4_IO2         TSC_IOSCR_G4_IO2_Msk                          /*!<GROUP4_IO2 sampling mode */
#define TSC_IOSCR_G4_IO3_Pos     (14U)                                         
#define TSC_IOSCR_G4_IO3_Msk     (0x1U << TSC_IOSCR_G4_IO3_Pos)                /*!< 0x00004000 */
#define TSC_IOSCR_G4_IO3         TSC_IOSCR_G4_IO3_Msk                          /*!<GROUP4_IO3 sampling mode */
#define TSC_IOSCR_G4_IO4_Pos     (15U)                                         
#define TSC_IOSCR_G4_IO4_Msk     (0x1U << TSC_IOSCR_G4_IO4_Pos)                /*!< 0x00008000 */
#define TSC_IOSCR_G4_IO4         TSC_IOSCR_G4_IO4_Msk                          /*!<GROUP4_IO4 sampling mode */
#define TSC_IOSCR_G5_IO1_Pos     (16U)                                         
#define TSC_IOSCR_G5_IO1_Msk     (0x1U << TSC_IOSCR_G5_IO1_Pos)                /*!< 0x00010000 */
#define TSC_IOSCR_G5_IO1         TSC_IOSCR_G5_IO1_Msk                          /*!<GROUP5_IO1 sampling mode */
#define TSC_IOSCR_G5_IO2_Pos     (17U)                                         
#define TSC_IOSCR_G5_IO2_Msk     (0x1U << TSC_IOSCR_G5_IO2_Pos)                /*!< 0x00020000 */
#define TSC_IOSCR_G5_IO2         TSC_IOSCR_G5_IO2_Msk                          /*!<GROUP5_IO2 sampling mode */
#define TSC_IOSCR_G5_IO3_Pos     (18U)                                         
#define TSC_IOSCR_G5_IO3_Msk     (0x1U << TSC_IOSCR_G5_IO3_Pos)                /*!< 0x00040000 */
#define TSC_IOSCR_G5_IO3         TSC_IOSCR_G5_IO3_Msk                          /*!<GROUP5_IO3 sampling mode */
#define TSC_IOSCR_G5_IO4_Pos     (19U)                                         
#define TSC_IOSCR_G5_IO4_Msk     (0x1U << TSC_IOSCR_G5_IO4_Pos)                /*!< 0x00080000 */
#define TSC_IOSCR_G5_IO4         TSC_IOSCR_G5_IO4_Msk                          /*!<GROUP5_IO4 sampling mode */
#define TSC_IOSCR_G6_IO1_Pos     (20U)                                         
#define TSC_IOSCR_G6_IO1_Msk     (0x1U << TSC_IOSCR_G6_IO1_Pos)                /*!< 0x00100000 */
#define TSC_IOSCR_G6_IO1         TSC_IOSCR_G6_IO1_Msk                          /*!<GROUP6_IO1 sampling mode */
#define TSC_IOSCR_G6_IO2_Pos     (21U)                                         
#define TSC_IOSCR_G6_IO2_Msk     (0x1U << TSC_IOSCR_G6_IO2_Pos)                /*!< 0x00200000 */
#define TSC_IOSCR_G6_IO2         TSC_IOSCR_G6_IO2_Msk                          /*!<GROUP6_IO2 sampling mode */
#define TSC_IOSCR_G6_IO3_Pos     (22U)                                         
#define TSC_IOSCR_G6_IO3_Msk     (0x1U << TSC_IOSCR_G6_IO3_Pos)                /*!< 0x00400000 */
#define TSC_IOSCR_G6_IO3         TSC_IOSCR_G6_IO3_Msk                          /*!<GROUP6_IO3 sampling mode */
#define TSC_IOSCR_G6_IO4_Pos     (23U)                                         
#define TSC_IOSCR_G6_IO4_Msk     (0x1U << TSC_IOSCR_G6_IO4_Pos)                /*!< 0x00800000 */
#define TSC_IOSCR_G6_IO4         TSC_IOSCR_G6_IO4_Msk                          /*!<GROUP6_IO4 sampling mode */
#define TSC_IOSCR_G7_IO1_Pos     (24U)                                         
#define TSC_IOSCR_G7_IO1_Msk     (0x1U << TSC_IOSCR_G7_IO1_Pos)                /*!< 0x01000000 */
#define TSC_IOSCR_G7_IO1         TSC_IOSCR_G7_IO1_Msk                          /*!<GROUP7_IO1 sampling mode */
#define TSC_IOSCR_G7_IO2_Pos     (25U)                                         
#define TSC_IOSCR_G7_IO2_Msk     (0x1U << TSC_IOSCR_G7_IO2_Pos)                /*!< 0x02000000 */
#define TSC_IOSCR_G7_IO2         TSC_IOSCR_G7_IO2_Msk                          /*!<GROUP7_IO2 sampling mode */
#define TSC_IOSCR_G7_IO3_Pos     (26U)                                         
#define TSC_IOSCR_G7_IO3_Msk     (0x1U << TSC_IOSCR_G7_IO3_Pos)                /*!< 0x04000000 */
#define TSC_IOSCR_G7_IO3         TSC_IOSCR_G7_IO3_Msk                          /*!<GROUP7_IO3 sampling mode */
#define TSC_IOSCR_G7_IO4_Pos     (27U)                                         
#define TSC_IOSCR_G7_IO4_Msk     (0x1U << TSC_IOSCR_G7_IO4_Pos)                /*!< 0x08000000 */
#define TSC_IOSCR_G7_IO4         TSC_IOSCR_G7_IO4_Msk                          /*!<GROUP7_IO4 sampling mode */
#define TSC_IOSCR_G8_IO1_Pos     (28U)                                         
#define TSC_IOSCR_G8_IO1_Msk     (0x1U << TSC_IOSCR_G8_IO1_Pos)                /*!< 0x10000000 */
#define TSC_IOSCR_G8_IO1         TSC_IOSCR_G8_IO1_Msk                          /*!<GROUP8_IO1 sampling mode */
#define TSC_IOSCR_G8_IO2_Pos     (29U)                                         
#define TSC_IOSCR_G8_IO2_Msk     (0x1U << TSC_IOSCR_G8_IO2_Pos)                /*!< 0x20000000 */
#define TSC_IOSCR_G8_IO2         TSC_IOSCR_G8_IO2_Msk                          /*!<GROUP8_IO2 sampling mode */
#define TSC_IOSCR_G8_IO3_Pos     (30U)                                         
#define TSC_IOSCR_G8_IO3_Msk     (0x1U << TSC_IOSCR_G8_IO3_Pos)                /*!< 0x40000000 */
#define TSC_IOSCR_G8_IO3         TSC_IOSCR_G8_IO3_Msk                          /*!<GROUP8_IO3 sampling mode */
#define TSC_IOSCR_G8_IO4_Pos     (31U)                                         
#define TSC_IOSCR_G8_IO4_Msk     (0x1U << TSC_IOSCR_G8_IO4_Pos)                /*!< 0x80000000 */
#define TSC_IOSCR_G8_IO4         TSC_IOSCR_G8_IO4_Msk                          /*!<GROUP8_IO4 sampling mode */

/*******************  Bit definition for TSC_IOCCR register  ******************/
#define TSC_IOCCR_G1_IO1_Pos     (0U)                                          
#define TSC_IOCCR_G1_IO1_Msk     (0x1U << TSC_IOCCR_G1_IO1_Pos)                /*!< 0x00000001 */
#define TSC_IOCCR_G1_IO1         TSC_IOCCR_G1_IO1_Msk                          /*!<GROUP1_IO1 channel mode */
#define TSC_IOCCR_G1_IO2_Pos     (1U)                                          
#define TSC_IOCCR_G1_IO2_Msk     (0x1U << TSC_IOCCR_G1_IO2_Pos)                /*!< 0x00000002 */
#define TSC_IOCCR_G1_IO2         TSC_IOCCR_G1_IO2_Msk                          /*!<GROUP1_IO2 channel mode */
#define TSC_IOCCR_G1_IO3_Pos     (2U)                                          
#define TSC_IOCCR_G1_IO3_Msk     (0x1U << TSC_IOCCR_G1_IO3_Pos)                /*!< 0x00000004 */
#define TSC_IOCCR_G1_IO3         TSC_IOCCR_G1_IO3_Msk                          /*!<GROUP1_IO3 channel mode */
#define TSC_IOCCR_G1_IO4_Pos     (3U)                                          
#define TSC_IOCCR_G1_IO4_Msk     (0x1U << TSC_IOCCR_G1_IO4_Pos)                /*!< 0x00000008 */
#define TSC_IOCCR_G1_IO4         TSC_IOCCR_G1_IO4_Msk                          /*!<GROUP1_IO4 channel mode */
#define TSC_IOCCR_G2_IO1_Pos     (4U)                                          
#define TSC_IOCCR_G2_IO1_Msk     (0x1U << TSC_IOCCR_G2_IO1_Pos)                /*!< 0x00000010 */
#define TSC_IOCCR_G2_IO1         TSC_IOCCR_G2_IO1_Msk                          /*!<GROUP2_IO1 channel mode */
#define TSC_IOCCR_G2_IO2_Pos     (5U)                                          
#define TSC_IOCCR_G2_IO2_Msk     (0x1U << TSC_IOCCR_G2_IO2_Pos)                /*!< 0x00000020 */
#define TSC_IOCCR_G2_IO2         TSC_IOCCR_G2_IO2_Msk                          /*!<GROUP2_IO2 channel mode */
#define TSC_IOCCR_G2_IO3_Pos     (6U)                                          
#define TSC_IOCCR_G2_IO3_Msk     (0x1U << TSC_IOCCR_G2_IO3_Pos)                /*!< 0x00000040 */
#define TSC_IOCCR_G2_IO3         TSC_IOCCR_G2_IO3_Msk                          /*!<GROUP2_IO3 channel mode */
#define TSC_IOCCR_G2_IO4_Pos     (7U)                                          
#define TSC_IOCCR_G2_IO4_Msk     (0x1U << TSC_IOCCR_G2_IO4_Pos)                /*!< 0x00000080 */
#define TSC_IOCCR_G2_IO4         TSC_IOCCR_G2_IO4_Msk                          /*!<GROUP2_IO4 channel mode */
#define TSC_IOCCR_G3_IO1_Pos     (8U)                                          
#define TSC_IOCCR_G3_IO1_Msk     (0x1U << TSC_IOCCR_G3_IO1_Pos)                /*!< 0x00000100 */
#define TSC_IOCCR_G3_IO1         TSC_IOCCR_G3_IO1_Msk                          /*!<GROUP3_IO1 channel mode */
#define TSC_IOCCR_G3_IO2_Pos     (9U)                                          
#define TSC_IOCCR_G3_IO2_Msk     (0x1U << TSC_IOCCR_G3_IO2_Pos)                /*!< 0x00000200 */
#define TSC_IOCCR_G3_IO2         TSC_IOCCR_G3_IO2_Msk                          /*!<GROUP3_IO2 channel mode */
#define TSC_IOCCR_G3_IO3_Pos     (10U)                                         
#define TSC_IOCCR_G3_IO3_Msk     (0x1U << TSC_IOCCR_G3_IO3_Pos)                /*!< 0x00000400 */
#define TSC_IOCCR_G3_IO3         TSC_IOCCR_G3_IO3_Msk                          /*!<GROUP3_IO3 channel mode */
#define TSC_IOCCR_G3_IO4_Pos     (11U)                                         
#define TSC_IOCCR_G3_IO4_Msk     (0x1U << TSC_IOCCR_G3_IO4_Pos)                /*!< 0x00000800 */
#define TSC_IOCCR_G3_IO4         TSC_IOCCR_G3_IO4_Msk                          /*!<GROUP3_IO4 channel mode */
#define TSC_IOCCR_G4_IO1_Pos     (12U)                                         
#define TSC_IOCCR_G4_IO1_Msk     (0x1U << TSC_IOCCR_G4_IO1_Pos)                /*!< 0x00001000 */
#define TSC_IOCCR_G4_IO1         TSC_IOCCR_G4_IO1_Msk                          /*!<GROUP4_IO1 channel mode */
#define TSC_IOCCR_G4_IO2_Pos     (13U)                                         
#define TSC_IOCCR_G4_IO2_Msk     (0x1U << TSC_IOCCR_G4_IO2_Pos)                /*!< 0x00002000 */
#define TSC_IOCCR_G4_IO2         TSC_IOCCR_G4_IO2_Msk                          /*!<GROUP4_IO2 channel mode */
#define TSC_IOCCR_G4_IO3_Pos     (14U)                                         
#define TSC_IOCCR_G4_IO3_Msk     (0x1U << TSC_IOCCR_G4_IO3_Pos)                /*!< 0x00004000 */
#define TSC_IOCCR_G4_IO3         TSC_IOCCR_G4_IO3_Msk                          /*!<GROUP4_IO3 channel mode */
#define TSC_IOCCR_G4_IO4_Pos     (15U)                                         
#define TSC_IOCCR_G4_IO4_Msk     (0x1U << TSC_IOCCR_G4_IO4_Pos)                /*!< 0x00008000 */
#define TSC_IOCCR_G4_IO4         TSC_IOCCR_G4_IO4_Msk                          /*!<GROUP4_IO4 channel mode */
#define TSC_IOCCR_G5_IO1_Pos     (16U)                                         
#define TSC_IOCCR_G5_IO1_Msk     (0x1U << TSC_IOCCR_G5_IO1_Pos)                /*!< 0x00010000 */
#define TSC_IOCCR_G5_IO1         TSC_IOCCR_G5_IO1_Msk                          /*!<GROUP5_IO1 channel mode */
#define TSC_IOCCR_G5_IO2_Pos     (17U)                                         
#define TSC_IOCCR_G5_IO2_Msk     (0x1U << TSC_IOCCR_G5_IO2_Pos)                /*!< 0x00020000 */
#define TSC_IOCCR_G5_IO2         TSC_IOCCR_G5_IO2_Msk                          /*!<GROUP5_IO2 channel mode */
#define TSC_IOCCR_G5_IO3_Pos     (18U)                                         
#define TSC_IOCCR_G5_IO3_Msk     (0x1U << TSC_IOCCR_G5_IO3_Pos)                /*!< 0x00040000 */
#define TSC_IOCCR_G5_IO3         TSC_IOCCR_G5_IO3_Msk                          /*!<GROUP5_IO3 channel mode */
#define TSC_IOCCR_G5_IO4_Pos     (19U)                                         
#define TSC_IOCCR_G5_IO4_Msk     (0x1U << TSC_IOCCR_G5_IO4_Pos)                /*!< 0x00080000 */
#define TSC_IOCCR_G5_IO4         TSC_IOCCR_G5_IO4_Msk                          /*!<GROUP5_IO4 channel mode */
#define TSC_IOCCR_G6_IO1_Pos     (20U)                                         
#define TSC_IOCCR_G6_IO1_Msk     (0x1U << TSC_IOCCR_G6_IO1_Pos)                /*!< 0x00100000 */
#define TSC_IOCCR_G6_IO1         TSC_IOCCR_G6_IO1_Msk                          /*!<GROUP6_IO1 channel mode */
#define TSC_IOCCR_G6_IO2_Pos     (21U)                                         
#define TSC_IOCCR_G6_IO2_Msk     (0x1U << TSC_IOCCR_G6_IO2_Pos)                /*!< 0x00200000 */
#define TSC_IOCCR_G6_IO2         TSC_IOCCR_G6_IO2_Msk                          /*!<GROUP6_IO2 channel mode */
#define TSC_IOCCR_G6_IO3_Pos     (22U)                                         
#define TSC_IOCCR_G6_IO3_Msk     (0x1U << TSC_IOCCR_G6_IO3_Pos)                /*!< 0x00400000 */
#define TSC_IOCCR_G6_IO3         TSC_IOCCR_G6_IO3_Msk                          /*!<GROUP6_IO3 channel mode */
#define TSC_IOCCR_G6_IO4_Pos     (23U)                                         
#define TSC_IOCCR_G6_IO4_Msk     (0x1U << TSC_IOCCR_G6_IO4_Pos)                /*!< 0x00800000 */
#define TSC_IOCCR_G6_IO4         TSC_IOCCR_G6_IO4_Msk                          /*!<GROUP6_IO4 channel mode */
#define TSC_IOCCR_G7_IO1_Pos     (24U)                                         
#define TSC_IOCCR_G7_IO1_Msk     (0x1U << TSC_IOCCR_G7_IO1_Pos)                /*!< 0x01000000 */
#define TSC_IOCCR_G7_IO1         TSC_IOCCR_G7_IO1_Msk                          /*!<GROUP7_IO1 channel mode */
#define TSC_IOCCR_G7_IO2_Pos     (25U)                                         
#define TSC_IOCCR_G7_IO2_Msk     (0x1U << TSC_IOCCR_G7_IO2_Pos)                /*!< 0x02000000 */
#define TSC_IOCCR_G7_IO2         TSC_IOCCR_G7_IO2_Msk                          /*!<GROUP7_IO2 channel mode */
#define TSC_IOCCR_G7_IO3_Pos     (26U)                                         
#define TSC_IOCCR_G7_IO3_Msk     (0x1U << TSC_IOCCR_G7_IO3_Pos)                /*!< 0x04000000 */
#define TSC_IOCCR_G7_IO3         TSC_IOCCR_G7_IO3_Msk                          /*!<GROUP7_IO3 channel mode */
#define TSC_IOCCR_G7_IO4_Pos     (27U)                                         
#define TSC_IOCCR_G7_IO4_Msk     (0x1U << TSC_IOCCR_G7_IO4_Pos)                /*!< 0x08000000 */
#define TSC_IOCCR_G7_IO4         TSC_IOCCR_G7_IO4_Msk                          /*!<GROUP7_IO4 channel mode */
#define TSC_IOCCR_G8_IO1_Pos     (28U)                                         
#define TSC_IOCCR_G8_IO1_Msk     (0x1U << TSC_IOCCR_G8_IO1_Pos)                /*!< 0x10000000 */
#define TSC_IOCCR_G8_IO1         TSC_IOCCR_G8_IO1_Msk                          /*!<GROUP8_IO1 channel mode */
#define TSC_IOCCR_G8_IO2_Pos     (29U)                                         
#define TSC_IOCCR_G8_IO2_Msk     (0x1U << TSC_IOCCR_G8_IO2_Pos)                /*!< 0x20000000 */
#define TSC_IOCCR_G8_IO2         TSC_IOCCR_G8_IO2_Msk                          /*!<GROUP8_IO2 channel mode */
#define TSC_IOCCR_G8_IO3_Pos     (30U)                                         
#define TSC_IOCCR_G8_IO3_Msk     (0x1U << TSC_IOCCR_G8_IO3_Pos)                /*!< 0x40000000 */
#define TSC_IOCCR_G8_IO3         TSC_IOCCR_G8_IO3_Msk                          /*!<GROUP8_IO3 channel mode */
#define TSC_IOCCR_G8_IO4_Pos     (31U)                                         
#define TSC_IOCCR_G8_IO4_Msk     (0x1U << TSC_IOCCR_G8_IO4_Pos)                /*!< 0x80000000 */
#define TSC_IOCCR_G8_IO4         TSC_IOCCR_G8_IO4_Msk                          /*!<GROUP8_IO4 channel mode */

/*******************  Bit definition for TSC_IOGCSR register  *****************/
#define TSC_IOGCSR_G1E_Pos       (0U)                                          
#define TSC_IOGCSR_G1E_Msk       (0x1U << TSC_IOGCSR_G1E_Pos)                  /*!< 0x00000001 */
#define TSC_IOGCSR_G1E           TSC_IOGCSR_G1E_Msk                            /*!<Analog IO GROUP1 enable */
#define TSC_IOGCSR_G2E_Pos       (1U)                                          
#define TSC_IOGCSR_G2E_Msk       (0x1U << TSC_IOGCSR_G2E_Pos)                  /*!< 0x00000002 */
#define TSC_IOGCSR_G2E           TSC_IOGCSR_G2E_Msk                            /*!<Analog IO GROUP2 enable */
#define TSC_IOGCSR_G3E_Pos       (2U)                                          
#define TSC_IOGCSR_G3E_Msk       (0x1U << TSC_IOGCSR_G3E_Pos)                  /*!< 0x00000004 */
#define TSC_IOGCSR_G3E           TSC_IOGCSR_G3E_Msk                            /*!<Analog IO GROUP3 enable */
#define TSC_IOGCSR_G4E_Pos       (3U)                                          
#define TSC_IOGCSR_G4E_Msk       (0x1U << TSC_IOGCSR_G4E_Pos)                  /*!< 0x00000008 */
#define TSC_IOGCSR_G4E           TSC_IOGCSR_G4E_Msk                            /*!<Analog IO GROUP4 enable */
#define TSC_IOGCSR_G5E_Pos       (4U)                                          
#define TSC_IOGCSR_G5E_Msk       (0x1U << TSC_IOGCSR_G5E_Pos)                  /*!< 0x00000010 */
#define TSC_IOGCSR_G5E           TSC_IOGCSR_G5E_Msk                            /*!<Analog IO GROUP5 enable */
#define TSC_IOGCSR_G6E_Pos       (5U)                                          
#define TSC_IOGCSR_G6E_Msk       (0x1U << TSC_IOGCSR_G6E_Pos)                  /*!< 0x00000020 */
#define TSC_IOGCSR_G6E           TSC_IOGCSR_G6E_Msk                            /*!<Analog IO GROUP6 enable */
#define TSC_IOGCSR_G7E_Pos       (6U)                                          
#define TSC_IOGCSR_G7E_Msk       (0x1U << TSC_IOGCSR_G7E_Pos)                  /*!< 0x00000040 */
#define TSC_IOGCSR_G7E           TSC_IOGCSR_G7E_Msk                            /*!<Analog IO GROUP7 enable */
#define TSC_IOGCSR_G8E_Pos       (7U)                                          
#define TSC_IOGCSR_G8E_Msk       (0x1U << TSC_IOGCSR_G8E_Pos)                  /*!< 0x00000080 */
#define TSC_IOGCSR_G8E           TSC_IOGCSR_G8E_Msk                            /*!<Analog IO GROUP8 enable */
#define TSC_IOGCSR_G1S_Pos       (16U)                                         
#define TSC_IOGCSR_G1S_Msk       (0x1U << TSC_IOGCSR_G1S_Pos)                  /*!< 0x00010000 */
#define TSC_IOGCSR_G1S           TSC_IOGCSR_G1S_Msk                            /*!<Analog IO GROUP1 status */
#define TSC_IOGCSR_G2S_Pos       (17U)                                         
#define TSC_IOGCSR_G2S_Msk       (0x1U << TSC_IOGCSR_G2S_Pos)                  /*!< 0x00020000 */
#define TSC_IOGCSR_G2S           TSC_IOGCSR_G2S_Msk                            /*!<Analog IO GROUP2 status */
#define TSC_IOGCSR_G3S_Pos       (18U)                                         
#define TSC_IOGCSR_G3S_Msk       (0x1U << TSC_IOGCSR_G3S_Pos)                  /*!< 0x00040000 */
#define TSC_IOGCSR_G3S           TSC_IOGCSR_G3S_Msk                            /*!<Analog IO GROUP3 status */
#define TSC_IOGCSR_G4S_Pos       (19U)                                         
#define TSC_IOGCSR_G4S_Msk       (0x1U << TSC_IOGCSR_G4S_Pos)                  /*!< 0x00080000 */
#define TSC_IOGCSR_G4S           TSC_IOGCSR_G4S_Msk                            /*!<Analog IO GROUP4 status */
#define TSC_IOGCSR_G5S_Pos       (20U)                                         
#define TSC_IOGCSR_G5S_Msk       (0x1U << TSC_IOGCSR_G5S_Pos)                  /*!< 0x00100000 */
#define TSC_IOGCSR_G5S           TSC_IOGCSR_G5S_Msk                            /*!<Analog IO GROUP5 status */
#define TSC_IOGCSR_G6S_Pos       (21U)                                         
#define TSC_IOGCSR_G6S_Msk       (0x1U << TSC_IOGCSR_G6S_Pos)                  /*!< 0x00200000 */
#define TSC_IOGCSR_G6S           TSC_IOGCSR_G6S_Msk                            /*!<Analog IO GROUP6 status */
#define TSC_IOGCSR_G7S_Pos       (22U)                                         
#define TSC_IOGCSR_G7S_Msk       (0x1U << TSC_IOGCSR_G7S_Pos)                  /*!< 0x00400000 */
#define TSC_IOGCSR_G7S           TSC_IOGCSR_G7S_Msk                            /*!<Analog IO GROUP7 status */
#define TSC_IOGCSR_G8S_Pos       (23U)                                         
#define TSC_IOGCSR_G8S_Msk       (0x1U << TSC_IOGCSR_G8S_Pos)                  /*!< 0x00800000 */
#define TSC_IOGCSR_G8S           TSC_IOGCSR_G8S_Msk                            /*!<Analog IO GROUP8 status */

/*******************  Bit definition for TSC_IOGXCR register  *****************/
#define TSC_IOGXCR_CNT_Pos       (0U)                                          
#define TSC_IOGXCR_CNT_Msk       (0x3FFFU << TSC_IOGXCR_CNT_Pos)               /*!< 0x00003FFF */
#define TSC_IOGXCR_CNT           TSC_IOGXCR_CNT_Msk                            /*!<CNT[13:0] bits (Counter value) */

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/

/*
* @brief Specific device feature definitions (not present on all devices in the STM32F3 serie)
*/

/* Support of 7 bits data length feature */
#define USART_7BITS_SUPPORT

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_UE_Pos              (0U)                                     
#define USART_CR1_UE_Msk              (0x1U << USART_CR1_UE_Pos)               /*!< 0x00000001 */
#define USART_CR1_UE                  USART_CR1_UE_Msk                         /*!< USART Enable */
#define USART_CR1_UESM_Pos            (1U)                                     
#define USART_CR1_UESM_Msk            (0x1U << USART_CR1_UESM_Pos)             /*!< 0x00000002 */
#define USART_CR1_UESM                USART_CR1_UESM_Msk                       /*!< USART Enable in STOP Mode */
#define USART_CR1_RE_Pos              (2U)                                     
#define USART_CR1_RE_Msk              (0x1U << USART_CR1_RE_Pos)               /*!< 0x00000004 */
#define USART_CR1_RE                  USART_CR1_RE_Msk                         /*!< Receiver Enable */
#define USART_CR1_TE_Pos              (3U)                                     
#define USART_CR1_TE_Msk              (0x1U << USART_CR1_TE_Pos)               /*!< 0x00000008 */
#define USART_CR1_TE                  USART_CR1_TE_Msk                         /*!< Transmitter Enable */
#define USART_CR1_IDLEIE_Pos          (4U)                                     
#define USART_CR1_IDLEIE_Msk          (0x1U << USART_CR1_IDLEIE_Pos)           /*!< 0x00000010 */
#define USART_CR1_IDLEIE              USART_CR1_IDLEIE_Msk                     /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE_Pos          (5U)                                     
#define USART_CR1_RXNEIE_Msk          (0x1U << USART_CR1_RXNEIE_Pos)           /*!< 0x00000020 */
#define USART_CR1_RXNEIE              USART_CR1_RXNEIE_Msk                     /*!< RXNE Interrupt Enable */
#define USART_CR1_TCIE_Pos            (6U)                                     
#define USART_CR1_TCIE_Msk            (0x1U << USART_CR1_TCIE_Pos)             /*!< 0x00000040 */
#define USART_CR1_TCIE                USART_CR1_TCIE_Msk                       /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_Pos           (7U)                                     
#define USART_CR1_TXEIE_Msk           (0x1U << USART_CR1_TXEIE_Pos)            /*!< 0x00000080 */
#define USART_CR1_TXEIE               USART_CR1_TXEIE_Msk                      /*!< TXE Interrupt Enable */
#define USART_CR1_PEIE_Pos            (8U)                                     
#define USART_CR1_PEIE_Msk            (0x1U << USART_CR1_PEIE_Pos)             /*!< 0x00000100 */
#define USART_CR1_PEIE                USART_CR1_PEIE_Msk                       /*!< PE Interrupt Enable */
#define USART_CR1_PS_Pos              (9U)                                     
#define USART_CR1_PS_Msk              (0x1U << USART_CR1_PS_Pos)               /*!< 0x00000200 */
#define USART_CR1_PS                  USART_CR1_PS_Msk                         /*!< Parity Selection */
#define USART_CR1_PCE_Pos             (10U)                                    
#define USART_CR1_PCE_Msk             (0x1U << USART_CR1_PCE_Pos)              /*!< 0x00000400 */
#define USART_CR1_PCE                 USART_CR1_PCE_Msk                        /*!< Parity Control Enable */
#define USART_CR1_WAKE_Pos            (11U)                                    
#define USART_CR1_WAKE_Msk            (0x1U << USART_CR1_WAKE_Pos)             /*!< 0x00000800 */
#define USART_CR1_WAKE                USART_CR1_WAKE_Msk                       /*!< Receiver Wakeup method */
#define USART_CR1_M0_Pos              (12U)                                    
#define USART_CR1_M0_Msk              (0x1U << USART_CR1_M0_Pos)               /*!< 0x00001000 */
#define USART_CR1_M0                  USART_CR1_M0_Msk                         /*!< Word length bit 0 */
#define USART_CR1_MME_Pos             (13U)                                    
#define USART_CR1_MME_Msk             (0x1U << USART_CR1_MME_Pos)              /*!< 0x00002000 */
#define USART_CR1_MME                 USART_CR1_MME_Msk                        /*!< Mute Mode Enable */
#define USART_CR1_CMIE_Pos            (14U)                                    
#define USART_CR1_CMIE_Msk            (0x1U << USART_CR1_CMIE_Pos)             /*!< 0x00004000 */
#define USART_CR1_CMIE                USART_CR1_CMIE_Msk                       /*!< Character match interrupt enable */
#define USART_CR1_OVER8_Pos           (15U)                                    
#define USART_CR1_OVER8_Msk           (0x1U << USART_CR1_OVER8_Pos)            /*!< 0x00008000 */
#define USART_CR1_OVER8               USART_CR1_OVER8_Msk                      /*!< Oversampling by 8-bit or 16-bit mode */
#define USART_CR1_DEDT_Pos            (16U)                                    
#define USART_CR1_DEDT_Msk            (0x1FU << USART_CR1_DEDT_Pos)            /*!< 0x001F0000 */
#define USART_CR1_DEDT                USART_CR1_DEDT_Msk                       /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define USART_CR1_DEDT_0              (0x01U << USART_CR1_DEDT_Pos)            /*!< 0x00010000 */
#define USART_CR1_DEDT_1              (0x02U << USART_CR1_DEDT_Pos)            /*!< 0x00020000 */
#define USART_CR1_DEDT_2              (0x04U << USART_CR1_DEDT_Pos)            /*!< 0x00040000 */
#define USART_CR1_DEDT_3              (0x08U << USART_CR1_DEDT_Pos)            /*!< 0x00080000 */
#define USART_CR1_DEDT_4              (0x10U << USART_CR1_DEDT_Pos)            /*!< 0x00100000 */
#define USART_CR1_DEAT_Pos            (21U)                                    
#define USART_CR1_DEAT_Msk            (0x1FU << USART_CR1_DEAT_Pos)            /*!< 0x03E00000 */
#define USART_CR1_DEAT                USART_CR1_DEAT_Msk                       /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define USART_CR1_DEAT_0              (0x01U << USART_CR1_DEAT_Pos)            /*!< 0x00200000 */
#define USART_CR1_DEAT_1              (0x02U << USART_CR1_DEAT_Pos)            /*!< 0x00400000 */
#define USART_CR1_DEAT_2              (0x04U << USART_CR1_DEAT_Pos)            /*!< 0x00800000 */
#define USART_CR1_DEAT_3              (0x08U << USART_CR1_DEAT_Pos)            /*!< 0x01000000 */
#define USART_CR1_DEAT_4              (0x10U << USART_CR1_DEAT_Pos)            /*!< 0x02000000 */
#define USART_CR1_RTOIE_Pos           (26U)                                    
#define USART_CR1_RTOIE_Msk           (0x1U << USART_CR1_RTOIE_Pos)            /*!< 0x04000000 */
#define USART_CR1_RTOIE               USART_CR1_RTOIE_Msk                      /*!< Receive Time Out interrupt enable */
#define USART_CR1_EOBIE_Pos           (27U)                                    
#define USART_CR1_EOBIE_Msk           (0x1U << USART_CR1_EOBIE_Pos)            /*!< 0x08000000 */
#define USART_CR1_EOBIE               USART_CR1_EOBIE_Msk                      /*!< End of Block interrupt enable */
#define USART_CR1_M1_Pos              (28U)                                    
#define USART_CR1_M1_Msk              (0x1U << USART_CR1_M1_Pos)               /*!< 0x10000000 */
#define USART_CR1_M1                  USART_CR1_M1_Msk                         /*!< Word length bit 1 */
#define USART_CR1_M_Pos               (12U)                                    
#define USART_CR1_M_Msk               (0x10001U << USART_CR1_M_Pos)            /*!< 0x10001000 */
#define USART_CR1_M                   USART_CR1_M_Msk                          /*!< [M1:M0] Word length */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADDM7_Pos           (4U)                                     
#define USART_CR2_ADDM7_Msk           (0x1U << USART_CR2_ADDM7_Pos)            /*!< 0x00000010 */
#define USART_CR2_ADDM7               USART_CR2_ADDM7_Msk                      /*!< 7-bit or 4-bit Address Detection */
#define USART_CR2_LBDL_Pos            (5U)                                     
#define USART_CR2_LBDL_Msk            (0x1U << USART_CR2_LBDL_Pos)             /*!< 0x00000020 */
#define USART_CR2_LBDL                USART_CR2_LBDL_Msk                       /*!< LIN Break Detection Length */
#define USART_CR2_LBDIE_Pos           (6U)                                     
#define USART_CR2_LBDIE_Msk           (0x1U << USART_CR2_LBDIE_Pos)            /*!< 0x00000040 */
#define USART_CR2_LBDIE               USART_CR2_LBDIE_Msk                      /*!< LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos            (8U)                                     
#define USART_CR2_LBCL_Msk            (0x1U << USART_CR2_LBCL_Pos)             /*!< 0x00000100 */
#define USART_CR2_LBCL                USART_CR2_LBCL_Msk                       /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA_Pos            (9U)                                     
#define USART_CR2_CPHA_Msk            (0x1U << USART_CR2_CPHA_Pos)             /*!< 0x00000200 */
#define USART_CR2_CPHA                USART_CR2_CPHA_Msk                       /*!< Clock Phase */
#define USART_CR2_CPOL_Pos            (10U)                                    
#define USART_CR2_CPOL_Msk            (0x1U << USART_CR2_CPOL_Pos)             /*!< 0x00000400 */
#define USART_CR2_CPOL                USART_CR2_CPOL_Msk                       /*!< Clock Polarity */
#define USART_CR2_CLKEN_Pos           (11U)                                    
#define USART_CR2_CLKEN_Msk           (0x1U << USART_CR2_CLKEN_Pos)            /*!< 0x00000800 */
#define USART_CR2_CLKEN               USART_CR2_CLKEN_Msk                      /*!< Clock Enable */
#define USART_CR2_STOP_Pos            (12U)                                    
#define USART_CR2_STOP_Msk            (0x3U << USART_CR2_STOP_Pos)             /*!< 0x00003000 */
#define USART_CR2_STOP                USART_CR2_STOP_Msk                       /*!< STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0              (0x1U << USART_CR2_STOP_Pos)             /*!< 0x00001000 */
#define USART_CR2_STOP_1              (0x2U << USART_CR2_STOP_Pos)             /*!< 0x00002000 */
#define USART_CR2_LINEN_Pos           (14U)                                    
#define USART_CR2_LINEN_Msk           (0x1U << USART_CR2_LINEN_Pos)            /*!< 0x00004000 */
#define USART_CR2_LINEN               USART_CR2_LINEN_Msk                      /*!< LIN mode enable */
#define USART_CR2_SWAP_Pos            (15U)                                    
#define USART_CR2_SWAP_Msk            (0x1U << USART_CR2_SWAP_Pos)             /*!< 0x00008000 */
#define USART_CR2_SWAP                USART_CR2_SWAP_Msk                       /*!< SWAP TX/RX pins */
#define USART_CR2_RXINV_Pos           (16U)                                    
#define USART_CR2_RXINV_Msk           (0x1U << USART_CR2_RXINV_Pos)            /*!< 0x00010000 */
#define USART_CR2_RXINV               USART_CR2_RXINV_Msk                      /*!< RX pin active level inversion */
#define USART_CR2_TXINV_Pos           (17U)                                    
#define USART_CR2_TXINV_Msk           (0x1U << USART_CR2_TXINV_Pos)            /*!< 0x00020000 */
#define USART_CR2_TXINV               USART_CR2_TXINV_Msk                      /*!< TX pin active level inversion */
#define USART_CR2_DATAINV_Pos         (18U)                                    
#define USART_CR2_DATAINV_Msk         (0x1U << USART_CR2_DATAINV_Pos)          /*!< 0x00040000 */
#define USART_CR2_DATAINV             USART_CR2_DATAINV_Msk                    /*!< Binary data inversion */
#define USART_CR2_MSBFIRST_Pos        (19U)                                    
#define USART_CR2_MSBFIRST_Msk        (0x1U << USART_CR2_MSBFIRST_Pos)         /*!< 0x00080000 */
#define USART_CR2_MSBFIRST            USART_CR2_MSBFIRST_Msk                   /*!< Most Significant Bit First */
#define USART_CR2_ABREN_Pos           (20U)                                    
#define USART_CR2_ABREN_Msk           (0x1U << USART_CR2_ABREN_Pos)            /*!< 0x00100000 */
#define USART_CR2_ABREN               USART_CR2_ABREN_Msk                      /*!< Auto Baud-Rate Enable*/
#define USART_CR2_ABRMODE_Pos         (21U)                                    
#define USART_CR2_ABRMODE_Msk         (0x3U << USART_CR2_ABRMODE_Pos)          /*!< 0x00600000 */
#define USART_CR2_ABRMODE             USART_CR2_ABRMODE_Msk                    /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define USART_CR2_ABRMODE_0           (0x1U << USART_CR2_ABRMODE_Pos)          /*!< 0x00200000 */
#define USART_CR2_ABRMODE_1           (0x2U << USART_CR2_ABRMODE_Pos)          /*!< 0x00400000 */
#define USART_CR2_RTOEN_Pos           (23U)                                    
#define USART_CR2_RTOEN_Msk           (0x1U << USART_CR2_RTOEN_Pos)            /*!< 0x00800000 */
#define USART_CR2_RTOEN               USART_CR2_RTOEN_Msk                      /*!< Receiver Time-Out enable */
#define USART_CR2_ADD_Pos             (24U)                                    
#define USART_CR2_ADD_Msk             (0xFFU << USART_CR2_ADD_Pos)             /*!< 0xFF000000 */
#define USART_CR2_ADD                 USART_CR2_ADD_Msk                        /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos             (0U)                                     
#define USART_CR3_EIE_Msk             (0x1U << USART_CR3_EIE_Pos)              /*!< 0x00000001 */
#define USART_CR3_EIE                 USART_CR3_EIE_Msk                        /*!< Error Interrupt Enable */
#define USART_CR3_IREN_Pos            (1U)                                     
#define USART_CR3_IREN_Msk            (0x1U << USART_CR3_IREN_Pos)             /*!< 0x00000002 */
#define USART_CR3_IREN                USART_CR3_IREN_Msk                       /*!< IrDA mode Enable */
#define USART_CR3_IRLP_Pos            (2U)                                     
#define USART_CR3_IRLP_Msk            (0x1U << USART_CR3_IRLP_Pos)             /*!< 0x00000004 */
#define USART_CR3_IRLP                USART_CR3_IRLP_Msk                       /*!< IrDA Low-Power */
#define USART_CR3_HDSEL_Pos           (3U)                                     
#define USART_CR3_HDSEL_Msk           (0x1U << USART_CR3_HDSEL_Pos)            /*!< 0x00000008 */
#define USART_CR3_HDSEL               USART_CR3_HDSEL_Msk                      /*!< Half-Duplex Selection */
#define USART_CR3_NACK_Pos            (4U)                                     
#define USART_CR3_NACK_Msk            (0x1U << USART_CR3_NACK_Pos)             /*!< 0x00000010 */
#define USART_CR3_NACK                USART_CR3_NACK_Msk                       /*!< SmartCard NACK enable */
#define USART_CR3_SCEN_Pos            (5U)                                     
#define USART_CR3_SCEN_Msk            (0x1U << USART_CR3_SCEN_Pos)             /*!< 0x00000020 */
#define USART_CR3_SCEN                USART_CR3_SCEN_Msk                       /*!< SmartCard mode enable */
#define USART_CR3_DMAR_Pos            (6U)                                     
#define USART_CR3_DMAR_Msk            (0x1U << USART_CR3_DMAR_Pos)             /*!< 0x00000040 */
#define USART_CR3_DMAR                USART_CR3_DMAR_Msk                       /*!< DMA Enable Receiver */
#define USART_CR3_DMAT_Pos            (7U)                                     
#define USART_CR3_DMAT_Msk            (0x1U << USART_CR3_DMAT_Pos)             /*!< 0x00000080 */
#define USART_CR3_DMAT                USART_CR3_DMAT_Msk                       /*!< DMA Enable Transmitter */
#define USART_CR3_RTSE_Pos            (8U)                                     
#define USART_CR3_RTSE_Msk            (0x1U << USART_CR3_RTSE_Pos)             /*!< 0x00000100 */
#define USART_CR3_RTSE                USART_CR3_RTSE_Msk                       /*!< RTS Enable */
#define USART_CR3_CTSE_Pos            (9U)                                     
#define USART_CR3_CTSE_Msk            (0x1U << USART_CR3_CTSE_Pos)             /*!< 0x00000200 */
#define USART_CR3_CTSE                USART_CR3_CTSE_Msk                       /*!< CTS Enable */
#define USART_CR3_CTSIE_Pos           (10U)                                    
#define USART_CR3_CTSIE_Msk           (0x1U << USART_CR3_CTSIE_Pos)            /*!< 0x00000400 */
#define USART_CR3_CTSIE               USART_CR3_CTSIE_Msk                      /*!< CTS Interrupt Enable */
#define USART_CR3_ONEBIT_Pos          (11U)                                    
#define USART_CR3_ONEBIT_Msk          (0x1U << USART_CR3_ONEBIT_Pos)           /*!< 0x00000800 */
#define USART_CR3_ONEBIT              USART_CR3_ONEBIT_Msk                     /*!< One sample bit method enable */
#define USART_CR3_OVRDIS_Pos          (12U)                                    
#define USART_CR3_OVRDIS_Msk          (0x1U << USART_CR3_OVRDIS_Pos)           /*!< 0x00001000 */
#define USART_CR3_OVRDIS              USART_CR3_OVRDIS_Msk                     /*!< Overrun Disable */
#define USART_CR3_DDRE_Pos            (13U)                                    
#define USART_CR3_DDRE_Msk            (0x1U << USART_CR3_DDRE_Pos)             /*!< 0x00002000 */
#define USART_CR3_DDRE                USART_CR3_DDRE_Msk                       /*!< DMA Disable on Reception Error */
#define USART_CR3_DEM_Pos             (14U)                                    
#define USART_CR3_DEM_Msk             (0x1U << USART_CR3_DEM_Pos)              /*!< 0x00004000 */
#define USART_CR3_DEM                 USART_CR3_DEM_Msk                        /*!< Driver Enable Mode */
#define USART_CR3_DEP_Pos             (15U)                                    
#define USART_CR3_DEP_Msk             (0x1U << USART_CR3_DEP_Pos)              /*!< 0x00008000 */
#define USART_CR3_DEP                 USART_CR3_DEP_Msk                        /*!< Driver Enable Polarity Selection */
#define USART_CR3_SCARCNT_Pos         (17U)                                    
#define USART_CR3_SCARCNT_Msk         (0x7U << USART_CR3_SCARCNT_Pos)          /*!< 0x000E0000 */
#define USART_CR3_SCARCNT             USART_CR3_SCARCNT_Msk                    /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
#define USART_CR3_SCARCNT_0           (0x1U << USART_CR3_SCARCNT_Pos)          /*!< 0x00020000 */
#define USART_CR3_SCARCNT_1           (0x2U << USART_CR3_SCARCNT_Pos)          /*!< 0x00040000 */
#define USART_CR3_SCARCNT_2           (0x4U << USART_CR3_SCARCNT_Pos)          /*!< 0x00080000 */
#define USART_CR3_WUS_Pos             (20U)                                    
#define USART_CR3_WUS_Msk             (0x3U << USART_CR3_WUS_Pos)              /*!< 0x00300000 */
#define USART_CR3_WUS                 USART_CR3_WUS_Msk                        /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
#define USART_CR3_WUS_0               (0x1U << USART_CR3_WUS_Pos)              /*!< 0x00100000 */
#define USART_CR3_WUS_1               (0x2U << USART_CR3_WUS_Pos)              /*!< 0x00200000 */
#define USART_CR3_WUFIE_Pos           (22U)                                    
#define USART_CR3_WUFIE_Msk           (0x1U << USART_CR3_WUFIE_Pos)            /*!< 0x00400000 */
#define USART_CR3_WUFIE               USART_CR3_WUFIE_Msk                      /*!< Wake Up Interrupt Enable */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_FRACTION_Pos    (0U)                                     
#define USART_BRR_DIV_FRACTION_Msk    (0xFU << USART_BRR_DIV_FRACTION_Pos)     /*!< 0x0000000F */
#define USART_BRR_DIV_FRACTION        USART_BRR_DIV_FRACTION_Msk               /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_MANTISSA_Pos    (4U)                                     
#define USART_BRR_DIV_MANTISSA_Msk    (0xFFFU << USART_BRR_DIV_MANTISSA_Pos)   /*!< 0x0000FFF0 */
#define USART_BRR_DIV_MANTISSA        USART_BRR_DIV_MANTISSA_Msk               /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos            (0U)                                     
#define USART_GTPR_PSC_Msk            (0xFFU << USART_GTPR_PSC_Pos)            /*!< 0x000000FF */
#define USART_GTPR_PSC                USART_GTPR_PSC_Msk                       /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_GT_Pos             (8U)                                     
#define USART_GTPR_GT_Msk             (0xFFU << USART_GTPR_GT_Pos)             /*!< 0x0000FF00 */
#define USART_GTPR_GT                 USART_GTPR_GT_Msk                        /*!< GT[7:0] bits (Guard time value) */


/*******************  Bit definition for USART_RTOR register  *****************/
#define USART_RTOR_RTO_Pos            (0U)                                     
#define USART_RTOR_RTO_Msk            (0xFFFFFFU << USART_RTOR_RTO_Pos)        /*!< 0x00FFFFFF */
#define USART_RTOR_RTO                USART_RTOR_RTO_Msk                       /*!< Receiver Time Out Value */
#define USART_RTOR_BLEN_Pos           (24U)                                    
#define USART_RTOR_BLEN_Msk           (0xFFU << USART_RTOR_BLEN_Pos)           /*!< 0xFF000000 */
#define USART_RTOR_BLEN               USART_RTOR_BLEN_Msk                      /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define USART_RQR_ABRRQ_Pos           (0U)                                     
#define USART_RQR_ABRRQ_Msk           (0x1U << USART_RQR_ABRRQ_Pos)            /*!< 0x00000001 */
#define USART_RQR_ABRRQ               USART_RQR_ABRRQ_Msk                      /*!< Auto-Baud Rate Request */
#define USART_RQR_SBKRQ_Pos           (1U)                                     
#define USART_RQR_SBKRQ_Msk           (0x1U << USART_RQR_SBKRQ_Pos)            /*!< 0x00000002 */
#define USART_RQR_SBKRQ               USART_RQR_SBKRQ_Msk                      /*!< Send Break Request */
#define USART_RQR_MMRQ_Pos            (2U)                                     
#define USART_RQR_MMRQ_Msk            (0x1U << USART_RQR_MMRQ_Pos)             /*!< 0x00000004 */
#define USART_RQR_MMRQ                USART_RQR_MMRQ_Msk                       /*!< Mute Mode Request */
#define USART_RQR_RXFRQ_Pos           (3U)                                     
#define USART_RQR_RXFRQ_Msk           (0x1U << USART_RQR_RXFRQ_Pos)            /*!< 0x00000008 */
#define USART_RQR_RXFRQ               USART_RQR_RXFRQ_Msk                      /*!< Receive Data flush Request */
#define USART_RQR_TXFRQ_Pos           (4U)                                     
#define USART_RQR_TXFRQ_Msk           (0x1U << USART_RQR_TXFRQ_Pos)            /*!< 0x00000010 */
#define USART_RQR_TXFRQ               USART_RQR_TXFRQ_Msk                      /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define USART_ISR_PE_Pos              (0U)                                     
#define USART_ISR_PE_Msk              (0x1U << USART_ISR_PE_Pos)               /*!< 0x00000001 */
#define USART_ISR_PE                  USART_ISR_PE_Msk                         /*!< Parity Error */
#define USART_ISR_FE_Pos              (1U)                                     
#define USART_ISR_FE_Msk              (0x1U << USART_ISR_FE_Pos)               /*!< 0x00000002 */
#define USART_ISR_FE                  USART_ISR_FE_Msk                         /*!< Framing Error */
#define USART_ISR_NE_Pos              (2U)                                     
#define USART_ISR_NE_Msk              (0x1U << USART_ISR_NE_Pos)               /*!< 0x00000004 */
#define USART_ISR_NE                  USART_ISR_NE_Msk                         /*!< Noise detected Flag */
#define USART_ISR_ORE_Pos             (3U)                                     
#define USART_ISR_ORE_Msk             (0x1U << USART_ISR_ORE_Pos)              /*!< 0x00000008 */
#define USART_ISR_ORE                 USART_ISR_ORE_Msk                        /*!< OverRun Error */
#define USART_ISR_IDLE_Pos            (4U)                                     
#define USART_ISR_IDLE_Msk            (0x1U << USART_ISR_IDLE_Pos)             /*!< 0x00000010 */
#define USART_ISR_IDLE                USART_ISR_IDLE_Msk                       /*!< IDLE line detected */
#define USART_ISR_RXNE_Pos            (5U)                                     
#define USART_ISR_RXNE_Msk            (0x1U << USART_ISR_RXNE_Pos)             /*!< 0x00000020 */
#define USART_ISR_RXNE                USART_ISR_RXNE_Msk                       /*!< Read Data Register Not Empty */
#define USART_ISR_TC_Pos              (6U)                                     
#define USART_ISR_TC_Msk              (0x1U << USART_ISR_TC_Pos)               /*!< 0x00000040 */
#define USART_ISR_TC                  USART_ISR_TC_Msk                         /*!< Transmission Complete */
#define USART_ISR_TXE_Pos             (7U)                                     
#define USART_ISR_TXE_Msk             (0x1U << USART_ISR_TXE_Pos)              /*!< 0x00000080 */
#define USART_ISR_TXE                 USART_ISR_TXE_Msk                        /*!< Transmit Data Register Empty */
#define USART_ISR_LBDF_Pos            (8U)                                     
#define USART_ISR_LBDF_Msk            (0x1U << USART_ISR_LBDF_Pos)             /*!< 0x00000100 */
#define USART_ISR_LBDF                USART_ISR_LBDF_Msk                       /*!< LIN Break Detection Flag */
#define USART_ISR_CTSIF_Pos           (9U)                                     
#define USART_ISR_CTSIF_Msk           (0x1U << USART_ISR_CTSIF_Pos)            /*!< 0x00000200 */
#define USART_ISR_CTSIF               USART_ISR_CTSIF_Msk                      /*!< CTS interrupt flag */
#define USART_ISR_CTS_Pos             (10U)                                    
#define USART_ISR_CTS_Msk             (0x1U << USART_ISR_CTS_Pos)              /*!< 0x00000400 */
#define USART_ISR_CTS                 USART_ISR_CTS_Msk                        /*!< CTS flag */
#define USART_ISR_RTOF_Pos            (11U)                                    
#define USART_ISR_RTOF_Msk            (0x1U << USART_ISR_RTOF_Pos)             /*!< 0x00000800 */
#define USART_ISR_RTOF                USART_ISR_RTOF_Msk                       /*!< Receiver Time Out */
#define USART_ISR_EOBF_Pos            (12U)                                    
#define USART_ISR_EOBF_Msk            (0x1U << USART_ISR_EOBF_Pos)             /*!< 0x00001000 */
#define USART_ISR_EOBF                USART_ISR_EOBF_Msk                       /*!< End Of Block Flag */
#define USART_ISR_ABRE_Pos            (14U)                                    
#define USART_ISR_ABRE_Msk            (0x1U << USART_ISR_ABRE_Pos)             /*!< 0x00004000 */
#define USART_ISR_ABRE                USART_ISR_ABRE_Msk                       /*!< Auto-Baud Rate Error */
#define USART_ISR_ABRF_Pos            (15U)                                    
#define USART_ISR_ABRF_Msk            (0x1U << USART_ISR_ABRF_Pos)             /*!< 0x00008000 */
#define USART_ISR_ABRF                USART_ISR_ABRF_Msk                       /*!< Auto-Baud Rate Flag */
#define USART_ISR_BUSY_Pos            (16U)                                    
#define USART_ISR_BUSY_Msk            (0x1U << USART_ISR_BUSY_Pos)             /*!< 0x00010000 */
#define USART_ISR_BUSY                USART_ISR_BUSY_Msk                       /*!< Busy Flag */
#define USART_ISR_CMF_Pos             (17U)                                    
#define USART_ISR_CMF_Msk             (0x1U << USART_ISR_CMF_Pos)              /*!< 0x00020000 */
#define USART_ISR_CMF                 USART_ISR_CMF_Msk                        /*!< Character Match Flag */
#define USART_ISR_SBKF_Pos            (18U)                                    
#define USART_ISR_SBKF_Msk            (0x1U << USART_ISR_SBKF_Pos)             /*!< 0x00040000 */
#define USART_ISR_SBKF                USART_ISR_SBKF_Msk                       /*!< Send Break Flag */
#define USART_ISR_RWU_Pos             (19U)                                    
#define USART_ISR_RWU_Msk             (0x1U << USART_ISR_RWU_Pos)              /*!< 0x00080000 */
#define USART_ISR_RWU                 USART_ISR_RWU_Msk                        /*!< Receive Wake Up from mute mode Flag */
#define USART_ISR_WUF_Pos             (20U)                                    
#define USART_ISR_WUF_Msk             (0x1U << USART_ISR_WUF_Pos)              /*!< 0x00100000 */
#define USART_ISR_WUF                 USART_ISR_WUF_Msk                        /*!< Wake Up from stop mode Flag */
#define USART_ISR_TEACK_Pos           (21U)                                    
#define USART_ISR_TEACK_Msk           (0x1U << USART_ISR_TEACK_Pos)            /*!< 0x00200000 */
#define USART_ISR_TEACK               USART_ISR_TEACK_Msk                      /*!< Transmit Enable Acknowledge Flag */
#define USART_ISR_REACK_Pos           (22U)                                    
#define USART_ISR_REACK_Msk           (0x1U << USART_ISR_REACK_Pos)            /*!< 0x00400000 */
#define USART_ISR_REACK               USART_ISR_REACK_Msk                      /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define USART_ICR_PECF_Pos            (0U)                                     
#define USART_ICR_PECF_Msk            (0x1U << USART_ICR_PECF_Pos)             /*!< 0x00000001 */
#define USART_ICR_PECF                USART_ICR_PECF_Msk                       /*!< Parity Error Clear Flag */
#define USART_ICR_FECF_Pos            (1U)                                     
#define USART_ICR_FECF_Msk            (0x1U << USART_ICR_FECF_Pos)             /*!< 0x00000002 */
#define USART_ICR_FECF                USART_ICR_FECF_Msk                       /*!< Framing Error Clear Flag */
#define USART_ICR_NCF_Pos             (2U)                                     
#define USART_ICR_NCF_Msk             (0x1U << USART_ICR_NCF_Pos)              /*!< 0x00000004 */
#define USART_ICR_NCF                 USART_ICR_NCF_Msk                        /*!< Noise detected Clear Flag */
#define USART_ICR_ORECF_Pos           (3U)                                     
#define USART_ICR_ORECF_Msk           (0x1U << USART_ICR_ORECF_Pos)            /*!< 0x00000008 */
#define USART_ICR_ORECF               USART_ICR_ORECF_Msk                      /*!< OverRun Error Clear Flag */
#define USART_ICR_IDLECF_Pos          (4U)                                     
#define USART_ICR_IDLECF_Msk          (0x1U << USART_ICR_IDLECF_Pos)           /*!< 0x00000010 */
#define USART_ICR_IDLECF              USART_ICR_IDLECF_Msk                     /*!< IDLE line detected Clear Flag */
#define USART_ICR_TCCF_Pos            (6U)                                     
#define USART_ICR_TCCF_Msk            (0x1U << USART_ICR_TCCF_Pos)             /*!< 0x00000040 */
#define USART_ICR_TCCF                USART_ICR_TCCF_Msk                       /*!< Transmission Complete Clear Flag */
#define USART_ICR_LBDCF_Pos           (8U)                                     
#define USART_ICR_LBDCF_Msk           (0x1U << USART_ICR_LBDCF_Pos)            /*!< 0x00000100 */
#define USART_ICR_LBDCF               USART_ICR_LBDCF_Msk                      /*!< LIN Break Detection Clear Flag */
#define USART_ICR_CTSCF_Pos           (9U)                                     
#define USART_ICR_CTSCF_Msk           (0x1U << USART_ICR_CTSCF_Pos)            /*!< 0x00000200 */
#define USART_ICR_CTSCF               USART_ICR_CTSCF_Msk                      /*!< CTS Interrupt Clear Flag */
#define USART_ICR_RTOCF_Pos           (11U)                                    
#define USART_ICR_RTOCF_Msk           (0x1U << USART_ICR_RTOCF_Pos)            /*!< 0x00000800 */
#define USART_ICR_RTOCF               USART_ICR_RTOCF_Msk                      /*!< Receiver Time Out Clear Flag */
#define USART_ICR_EOBCF_Pos           (12U)                                    
#define USART_ICR_EOBCF_Msk           (0x1U << USART_ICR_EOBCF_Pos)            /*!< 0x00001000 */
#define USART_ICR_EOBCF               USART_ICR_EOBCF_Msk                      /*!< End Of Block Clear Flag */
#define USART_ICR_CMCF_Pos            (17U)                                    
#define USART_ICR_CMCF_Msk            (0x1U << USART_ICR_CMCF_Pos)             /*!< 0x00020000 */
#define USART_ICR_CMCF                USART_ICR_CMCF_Msk                       /*!< Character Match Clear Flag */
#define USART_ICR_WUCF_Pos            (20U)                                    
#define USART_ICR_WUCF_Msk            (0x1U << USART_ICR_WUCF_Pos)             /*!< 0x00100000 */
#define USART_ICR_WUCF                USART_ICR_WUCF_Msk                       /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define USART_RDR_RDR_Pos             (0U)                                     
#define USART_RDR_RDR_Msk             (0x1FFU << USART_RDR_RDR_Pos)            /*!< 0x000001FF */
#define USART_RDR_RDR                 USART_RDR_RDR_Msk                        /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define USART_TDR_TDR_Pos             (0U)                                     
#define USART_TDR_TDR_Msk             (0x1FFU << USART_TDR_TDR_Pos)            /*!< 0x000001FF */
#define USART_TDR_TDR                 USART_TDR_TDR_Msk                        /*!< TDR[8:0] bits (Transmit Data value) */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define WWDG_CR_T_Pos           (0U)                                           
#define WWDG_CR_T_Msk           (0x7FU << WWDG_CR_T_Pos)                       /*!< 0x0000007F */
#define WWDG_CR_T               WWDG_CR_T_Msk                                  /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CR_T_0             (0x01U << WWDG_CR_T_Pos)                       /*!< 0x00000001 */
#define WWDG_CR_T_1             (0x02U << WWDG_CR_T_Pos)                       /*!< 0x00000002 */
#define WWDG_CR_T_2             (0x04U << WWDG_CR_T_Pos)                       /*!< 0x00000004 */
#define WWDG_CR_T_3             (0x08U << WWDG_CR_T_Pos)                       /*!< 0x00000008 */
#define WWDG_CR_T_4             (0x10U << WWDG_CR_T_Pos)                       /*!< 0x00000010 */
#define WWDG_CR_T_5             (0x20U << WWDG_CR_T_Pos)                       /*!< 0x00000020 */
#define WWDG_CR_T_6             (0x40U << WWDG_CR_T_Pos)                       /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CR_T0 WWDG_CR_T_0
#define  WWDG_CR_T1 WWDG_CR_T_1
#define  WWDG_CR_T2 WWDG_CR_T_2
#define  WWDG_CR_T3 WWDG_CR_T_3
#define  WWDG_CR_T4 WWDG_CR_T_4
#define  WWDG_CR_T5 WWDG_CR_T_5
#define  WWDG_CR_T6 WWDG_CR_T_6

#define WWDG_CR_WDGA_Pos        (7U)                                           
#define WWDG_CR_WDGA_Msk        (0x1U << WWDG_CR_WDGA_Pos)                     /*!< 0x00000080 */
#define WWDG_CR_WDGA            WWDG_CR_WDGA_Msk                               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define WWDG_CFR_W_Pos          (0U)                                           
#define WWDG_CFR_W_Msk          (0x7FU << WWDG_CFR_W_Pos)                      /*!< 0x0000007F */
#define WWDG_CFR_W              WWDG_CFR_W_Msk                                 /*!< W[6:0] bits (7-bit window value) */
#define WWDG_CFR_W_0            (0x01U << WWDG_CFR_W_Pos)                      /*!< 0x00000001 */
#define WWDG_CFR_W_1            (0x02U << WWDG_CFR_W_Pos)                      /*!< 0x00000002 */
#define WWDG_CFR_W_2            (0x04U << WWDG_CFR_W_Pos)                      /*!< 0x00000004 */
#define WWDG_CFR_W_3            (0x08U << WWDG_CFR_W_Pos)                      /*!< 0x00000008 */
#define WWDG_CFR_W_4            (0x10U << WWDG_CFR_W_Pos)                      /*!< 0x00000010 */
#define WWDG_CFR_W_5            (0x20U << WWDG_CFR_W_Pos)                      /*!< 0x00000020 */
#define WWDG_CFR_W_6            (0x40U << WWDG_CFR_W_Pos)                      /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CFR_W0 WWDG_CFR_W_0
#define  WWDG_CFR_W1 WWDG_CFR_W_1
#define  WWDG_CFR_W2 WWDG_CFR_W_2
#define  WWDG_CFR_W3 WWDG_CFR_W_3
#define  WWDG_CFR_W4 WWDG_CFR_W_4
#define  WWDG_CFR_W5 WWDG_CFR_W_5
#define  WWDG_CFR_W6 WWDG_CFR_W_6

#define WWDG_CFR_WDGTB_Pos      (7U)                                           
#define WWDG_CFR_WDGTB_Msk      (0x3U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000180 */
#define WWDG_CFR_WDGTB          WWDG_CFR_WDGTB_Msk                             /*!< WDGTB[1:0] bits (Timer Base) */
#define WWDG_CFR_WDGTB_0        (0x1U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000080 */
#define WWDG_CFR_WDGTB_1        (0x2U << WWDG_CFR_WDGTB_Pos)                   /*!< 0x00000100 */

/* Legacy defines */
#define  WWDG_CFR_WDGTB0 WWDG_CFR_WDGTB_0
#define  WWDG_CFR_WDGTB1 WWDG_CFR_WDGTB_1

#define WWDG_CFR_EWI_Pos        (9U)                                           
#define WWDG_CFR_EWI_Msk        (0x1U << WWDG_CFR_EWI_Pos)                     /*!< 0x00000200 */
#define WWDG_CFR_EWI            WWDG_CFR_EWI_Msk                               /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define WWDG_SR_EWIF_Pos        (0U)                                           
#define WWDG_SR_EWIF_Msk        (0x1U << WWDG_SR_EWIF_Pos)                     /*!< 0x00000001 */
#define WWDG_SR_EWIF            WWDG_SR_EWIF_Msk                               /*!<Early Wakeup Interrupt Flag */

/**
  * @}
  */

 /**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */

/****************************** ADC Instances *********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC1_COMMON)

/****************************** COMP Instances ********************************/
#define IS_COMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == COMP2) || \
                                        ((INSTANCE) == COMP4) || \
                                        ((INSTANCE) == COMP6))

#define IS_COMP_COMMON_INSTANCE(COMMON_INSTANCE) (0U)

/******************** COMP Instances with switch on DAC1 Channel1 output ******/
#define IS_COMP_DAC1SWITCH_INSTANCE(INSTANCE) ((INSTANCE) == COMP2)

/******************** COMP Instances with window mode capability **************/
#define IS_COMP_WINDOWMODE_INSTANCE(INSTANCE) (0U)

/****************************** CRC Instances *********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/****************************** DAC Instances *********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DAC1)

#define IS_DAC_CHANNEL_INSTANCE(INSTANCE, CHANNEL) \
    (((INSTANCE) == DAC1) &&                   \
     ((CHANNEL) == DAC_CHANNEL_1))

/****************************** DMA Instances *********************************/
#define IS_DMA_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Channel1) || \
                                       ((INSTANCE) == DMA1_Channel2) || \
                                       ((INSTANCE) == DMA1_Channel3) || \
                                       ((INSTANCE) == DMA1_Channel4) || \
                                       ((INSTANCE) == DMA1_Channel5) || \
                                       ((INSTANCE) == DMA1_Channel6) || \
                                       ((INSTANCE) == DMA1_Channel7))

/****************************** GPIO Instances ********************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE)  (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOC) || \
                                         ((INSTANCE) == GPIOD) || \
                                         ((INSTANCE) == GPIOF))

#define IS_GPIO_AF_INSTANCE(INSTANCE)   (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOC) || \
                                         ((INSTANCE) == GPIOD) || \
                                         ((INSTANCE) == GPIOF))

#define IS_GPIO_LOCK_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOC) || \
                                         ((INSTANCE) == GPIOD) || \
                                         ((INSTANCE) == GPIOF))

/****************************** I2C Instances *********************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2) || \
                                       ((INSTANCE) == I2C3))

/****************** I2C Instances : wakeup capability from stop modes *********/
#define IS_I2C_WAKEUP_FROMSTOP_INSTANCE(INSTANCE) IS_I2C_ALL_INSTANCE(INSTANCE)

/****************************** I2S Instances *********************************/
#define IS_I2S_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))
#define IS_I2S_EXT_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2S2ext) || \
                                           ((INSTANCE) == I2S3ext))

/****************************** OPAMP Instances *******************************/
#define IS_OPAMP_ALL_INSTANCE(INSTANCE) ((INSTANCE) == OPAMP2)

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/****************************** SMBUS Instances *******************************/
#define IS_SMBUS_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                         ((INSTANCE) == I2C2) || \
                                         ((INSTANCE) == I2C3))

/****************************** SPI Instances *********************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/******************* TIM Instances : All supported instances ******************/
#define IS_TIM_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/******************* TIM Instances : at least 1 capture/compare channel *******/
#define IS_TIM_CC1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : at least 2 capture/compare channels *******/
#define IS_TIM_CC2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : at least 3 capture/compare channels *******/
#define IS_TIM_CC3_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2))

/****************** TIM Instances : at least 4 capture/compare channels *******/
#define IS_TIM_CC4_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2))

/****************** TIM Instances : at least 5 capture/compare channels *******/
#define IS_TIM_CC5_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/****************** TIM Instances : at least 6 capture/compare channels *******/
#define IS_TIM_CC6_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)
    
/************************** TIM Instances : Advanced-control timers ***********/

/****************** TIM Instances : Advanced timer instances *******************/
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting clock selection ****************/
#define IS_TIM_CLOCK_SELECT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting external clock mode 1 for ETRF input */
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting external clock mode 2 **********/
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting external clock mode 1 for TIX inputs*/
#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting internal trigger inputs(ITRX) *******/
#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting OCxREF clear *******************/
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting encoder interface **************/
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting Hall interface *****************/
#define IS_TIM_HALL_INTERFACE_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)
  
#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)        
   
/**************** TIM Instances : external trigger input available ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2))    


/****************** TIM Instances : supporting input XOR function *************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting master mode ********************/
#define IS_TIM_MASTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting slave mode *********************/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting synchronization ****************/
#define IS_TIM_SYNCHRO_INSTANCE(INSTANCE)\
    (((INSTANCE) == TIM1)    || \
     ((INSTANCE) == TIM2)    || \
     ((INSTANCE) == TIM6)    || \
     ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting 32 bits counter ****************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)\
    ((INSTANCE) == TIM2)

/****************** TIM Instances : supporting DMA burst **********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE)\
    (((INSTANCE) == TIM1)    || \
     ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
     ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting the break function *************/
#define IS_TIM_BREAK_INSTANCE(INSTANCE)\
      (((INSTANCE) == TIM1)    || \
       ((INSTANCE) == TIM15)   || \
       ((INSTANCE) == TIM16)   || \
       ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting input/output channel(s) ********/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4) ||          \
      ((CHANNEL) == TIM_CHANNEL_5) ||          \
      ((CHANNEL) == TIM_CHANNEL_6)))           \
    ||                                         \
    (((INSTANCE) == TIM2) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM15) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2)))           \
    ||                                         \
    (((INSTANCE) == TIM16) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1)))           \
    ||                                         \
    (((INSTANCE) == TIM17) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1))))

/****************** TIM Instances : supporting complementary output(s) ********/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
    ||                                          \
    (((INSTANCE) == TIM15) &&                   \
      ((CHANNEL) == TIM_CHANNEL_1))             \
    ||                                          \
    (((INSTANCE) == TIM16) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1))              \
    ||                                          \
    (((INSTANCE) == TIM17) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1)))

/****************** TIM Instances : supporting counting mode selection ********/
#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting repetition counter *************/
#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting clock division *****************/
#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting 2 break inputs *****************/
#define IS_TIM_BKIN2_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting ADC triggering through TRGO2 ***/
#define IS_TIM_TRGO2_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting DMA generation on Update events*/
#define IS_TIM_DMA_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting DMA generation on Capture/Compare events */
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting commutation event generation ***/
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting remapping capability ***********/
#define IS_TIM_REMAP_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting combined 3-phase PWM mode ******/
#define IS_TIM_COMBINED3PHASEPWM_INSTANCE(INSTANCE) \
  (((INSTANCE) == TIM1))

/****************************** TSC Instances *********************************/
#define IS_TSC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == TSC)

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2) || \
                                     ((INSTANCE) == USART3))

/****************** USART Instances : Auto Baud Rate detection ****************/
#define IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(INSTANCE) ((INSTANCE) == USART1)

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                      ((INSTANCE) == USART2) || \
                                      ((INSTANCE) == USART3))
                                      
/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                 ((INSTANCE) == USART2) || \
                                                 ((INSTANCE) == USART3))
                                      
/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE(INSTANCE)   ((INSTANCE) == USART1)
                                          
/******************** UART Instances : Wake-up from Stop mode **********************/
#define IS_UART_WAKEUP_FROMSTOP_INSTANCE(INSTANCE)   ((INSTANCE) == USART1)

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART3))

/****************** UART Instances : Driver Enable ****************************/
#define IS_UART_DRIVER_ENABLE_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                  ((INSTANCE) == USART2) || \
                                                  ((INSTANCE) == USART3))

/********************* UART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) ((INSTANCE) == USART1)

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) ((INSTANCE) == USART1)

/******************** UART Instances : Support of continuous communication using DMA ****/
#define IS_UART_DMA_INSTANCE(INSTANCE) (1)
/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/**
  * @}
  */


/******************************************************************************/
/*  For a painless codes migration between the STM32F3xx device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */ 
/*  product lines within the same STM32F3 Family                              */
/******************************************************************************/

/* Aliases for __IRQn */
#define ADC1_2_IRQn      ADC1_IRQn
#define COMP1_2_IRQn     COMP2_IRQn
#define COMP1_2_3_IRQn   COMP2_IRQn
#define COMP_IRQn        COMP2_IRQn
#define COMP4_5_6_IRQn   COMP4_6_IRQn
#define HRTIM1_FLT_IRQn  I2C3_ER_IRQn
#define HRTIM1_TIME_IRQn I2C3_EV_IRQn
#define TIM15_IRQn       TIM1_BRK_TIM15_IRQn
#define TIM18_DAC2_IRQn  TIM1_CC_IRQn
#define TIM17_IRQn       TIM1_TRG_COM_TIM17_IRQn
#define TIM16_IRQn       TIM1_UP_TIM16_IRQn
#define TIM6_DAC1_IRQn   TIM6_DAC_IRQn


/* Aliases for __IRQHandler */
#define ADC1_2_IRQHandler      ADC1_IRQHandler
#define COMP1_2_IRQHandler     COMP2_IRQHandler
#define COMP1_2_3_IRQHandler   COMP2_IRQHandler
#define COMP_IRQHandler        COMP2_IRQHandler
#define COMP4_5_6_IRQHandler   COMP4_6_IRQHandler
#define HRTIM1_FLT_IRQHandler  I2C3_ER_IRQHandler
#define HRTIM1_TIME_IRQHandler I2C3_EV_IRQHandler
#define TIM15_IRQHandler       TIM1_BRK_TIM15_IRQHandler
#define TIM18_DAC2_IRQHandler  TIM1_CC_IRQHandler
#define TIM17_IRQHandler       TIM1_TRG_COM_TIM17_IRQHandler
#define TIM16_IRQHandler       TIM1_UP_TIM16_IRQHandler
#define TIM6_DAC1_IRQHandler   TIM6_DAC_IRQHandler


/** @defgroup ADC_Internal_Channels ADC Internal Channels
  * @{
  */

/* Note: Vopamp1, TempSensor and Vbat internal channels available on ADC1 only */
#define ADC_VOPAMP1_CHANNEL     15U
#define ADC_TEMPSENSOR_CHANNEL  16U
#define ADC_VBAT_CHANNEL        17U

/* Note: VrefInt internal channels available on all ADCs, but only            */
/*       one ADC is allowed to be connected to VrefInt at the same time.      */
#define ADC_VREFINT_CHANNEL     18U

/**
  * @}
  */

/** @defgroup ADC_Calibration_Values ADC Calibration Values
  * @{
  */

typedef struct
{
    __I uint16_t CAL30;
    const uint32_t __RESERVED[2];
    __I uint16_t CAL110;
}ADC_TempSensorCalibrationTypeDef;

#define ADC_TEMPSENSOR       ((ADC_TempSensorCalibrationTypeDef *)((uint32_t)0x1FFFF7B8U))

#define ADC_VREFINT_CAL      (*((__I uint16_t *)((uint32_t)0x1FFFF7BAU)))

#define ADC_VBAT_SCALER      2

/**
  * @}
  */

/** @defgroup Unique_Device_ID Unique Device ID
  * @brief    Use the macro as it was defined as: uint32_t DEVICE_ID_REG[3]
  * @{
  */

#define DEVICE_ID_REG        ((__I uint32_t *)UID_BASE)

/**
  * @}
  */

/** @defgroup Device_Flash_Size Device Flash Memory Size in kB
  * @{
  */

#define DEVICE_FLASH_SIZE_KB (*((__I uint16_t *)FLASHSIZE_BASE))

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
#define GPIO_TIM2_AF1          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */
#define GPIO_TIM15_AF1         ((uint8_t)0x01)  /* TIM15 Alternate Function mapping */
#define GPIO_TIM16_AF1         ((uint8_t)0x01)  /* TIM16 Alternate Function mapping */
#define GPIO_TIM17_AF1         ((uint8_t)0x01)  /* TIM17 Alternate Function mapping */
#define GPIO_EVENTOUT_AF1      ((uint8_t)0x01)  /* EVENTOUT Alternate Function mapping */

/* AF 2 selection */
#define GPIO_TIM1_AF2          ((uint8_t)0x02)  /* TIM1 Alternate Function mapping */
#define GPIO_TIM2_AF2          ((uint8_t)0x02)  /* TIM2 Alternate Function mapping */
#define GPIO_TIM15_AF2         ((uint8_t)0x02)  /* TIM15 Alternate Function mapping */
#define GPIO_I2C3_AF2          ((uint8_t)0x02)  /* I2C3 Alternate Function mapping */

/* AF 3 selection */
#define GPIO_TSC_AF3           ((uint8_t)0x03)  /* TSC Alternate Function mapping  */
#define GPIO_TIM15_AF3         ((uint8_t)0x03)  /* TIM15 Alternate Function mapping */
#define GPIO_I2C3_AF3          ((uint8_t)0x03)  /* I2C3 Alternate Function mapping  */

/* AF 4 selection */
#define GPIO_TIM1_AF4          ((uint8_t)0x04)  /* TIM1 Alternate Function mapping */
#define GPIO_TIM16_AF4         ((uint8_t)0x04)  /* TIM16 Alternate Function mapping */
#define GPIO_TIM17_AF4         ((uint8_t)0x04)  /* TIM17 Alternate Function mapping */
#define GPIO_I2C1_AF4          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_I2C2_AF4          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */

/* AF 5 selection */
#define GPIO_SPI1_AF5          ((uint8_t)0x05)  /* SPI1/I2S1 Alternate Function mapping */
#define GPIO_SPI2_AF5          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping */
#define GPIO_SPI3_AF5          ((uint8_t)0x05)  /* SPI3/I2S3 Alternate Function mapping */
#define GPIO_IR_AF5            ((uint8_t)0x05)  /* IR Alternate Function mapping */

/* AF 6 selection */
#define GPIO_SPI2_AF6          ((uint8_t)0x06)  /* SPI2/I2S2 Alternate Function mapping */
#define GPIO_SPI3_AF6          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping */
#define GPIO_TIM1_AF6          ((uint8_t)0x06)  /* TIM1 Alternate Function mapping */
#define GPIO_IR_AF6            ((uint8_t)0x06)  /* IR Alternate Function mapping */

/* AF 7 selection */
#define GPIO_USART1_AF7        ((uint8_t)0x07)  /* USART1 Alternate Function mapping  */
#define GPIO_USART2_AF7        ((uint8_t)0x07)  /* USART2 Alternate Function mapping  */
#define GPIO_USART3_AF7        ((uint8_t)0x07)  /* USART3 Alternate Function mapping  */
#define GPIO_COMP6_AF7         ((uint8_t)0x07)  /* COMP6 Alternate Function mapping  */

/* AF 8 selection */
#define GPIO_I2C3_AF8          ((uint8_t)0x08)  /* I2C3 Alternate Function mapping  */
#define GPIO_COMP2_AF8         ((uint8_t)0x08)  /* COMP2 Alternate Function mapping  */
#define GPIO_COMP4_AF8         ((uint8_t)0x08)  /* COMP4 Alternate Function mapping  */
#define GPIO_COMP6_AF8         ((uint8_t)0x08)  /* COMP6 Alternate Function mapping  */

/* AF 9 selection */
#define GPIO_TIM1_AF9          ((uint8_t)0x09)  /* TIM1 Alternate Function mapping */
#define GPIO_TIM15_AF9         ((uint8_t)0x09)  /* TIM15 Alternate Function mapping */

/* AF 10 selection */
#define GPIO_TIM2_AF10         ((uint8_t)0x0A)  /* TIM2 Alternate Function mapping */
#define GPIO_TIM17_AF10        ((uint8_t)0x0A)  /* TIM17 Alternate Function mapping */

/* AF 11 selection */
#define GPIO_TIM1_AF11         ((uint8_t)0x0B)  /* TIM1 Alternate Function mapping */

/* AF 12 selection */
#define GPIO_TIM1_AF12         ((uint8_t)0x0C)  /* TIM1 Alternate Function mapping */

/* AF 15 selection */
#define GPIO_EVENTOUT_AF15     ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */

/**
  * @}
  */

/** @defgroup RCC_Internal_Oscillators RCC Internal Oscillators
  * @{
  */

#define HSI_VALUE 8000000U  /* Value of the internal high speed oscillator in Hz */

#define LSI_VALUE 40000U    /* Approximate value of the internal low speed oscillator in Hz */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F301x8_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/