/**
  ******************************************************************************
  * @file    stm32f302xc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2015-12-28
  * @brief   CMSIS STM32F302xC Devices Peripheral Access Layer Header File.
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

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32f302xc
  * @{
  */

#ifndef __STM32F302xC_H
#define __STM32F302xC_H

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
#define __MPU_PRESENT             1       /*!< STM32F302xC devices provide an MPU */
#define __NVIC_PRIO_BITS          4       /*!< STM32F302xC devices use 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1       /*!< STM32F302xC devices provide an FPU */

/**
  * @}
  */
   
/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32F302xC devices Interrupt Number Definition, according to the selected device
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
  ADC1_2_IRQn                 = 18,     /*!< ADC1 & ADC2 Interrupts                                            */
  USB_HP_CAN_TX_IRQn          = 19,     /*!< USB Device High Priority or CAN TX Interrupts                     */
  USB_LP_CAN_RX0_IRQn         = 20,     /*!< USB Device Low Priority or CAN RX0 Interrupts                     */
  CAN_RX1_IRQn                = 21,     /*!< CAN RX1 Interrupt                                                 */
  CAN_SCE_IRQn                = 22,     /*!< CAN SCE Interrupt                                                 */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt                  */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)        */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt & EXTI Line24 Interrupt (I2C2 wakeup)        */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup)   */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt & EXTI Line26 Interrupt (USART2 wakeup)   */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt & EXTI Line28 Interrupt (USART3 wakeup)   */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line 17 Interrupt                 */
  USBWakeUp_IRQn              = 42,     /*!< USB Wakeup Interrupt                                              */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt & EXTI Line34 Interrupt (UART4 wakeup)     */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt & EXTI Line35 Interrupt (UART5 wakeup)     */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC underrun error Interrupt             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  COMP1_2_IRQn                = 64,     /*!< COMP1 and COMP2 global Interrupt via EXTI Line21 and 22           */
  COMP4_6_IRQn                = 65,     /*!< COMP4 and COMP6 global Interrupt via EXTI Line30 and 32           */
  USB_HP_IRQn                 = 74,     /*!< USB High Priority global Interrupt                                */
  USB_LP_IRQn                 = 75,     /*!< USB Low Priority global Interrupt                                 */
  USBWakeUp_RMP_IRQn          = 76,     /*!< USB Wakeup Interrupt remap                                        */
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
            __IO uint32_t ADRDY : 1;                 /*!< ADC Ready (ADRDY) flag  */
            __IO uint32_t EOSMP : 1;                 /*!< ADC End of Sampling flag */
            __IO uint32_t EOC : 1;                   /*!< ADC End of Regular Conversion flag */
            __IO uint32_t EOS : 1;                   /*!< ADC End of Regular sequence of Conversions flag */
            __IO uint32_t OVR : 1;                   /*!< ADC overrun flag */
            __IO uint32_t JEOC : 1;                  /*!< ADC End of Injected Conversion flag */
            __IO uint32_t JEOS : 1;                  /*!< ADC End of Injected sequence of Conversions flag */
            __IO uint32_t AWD1 : 1;                  /*!< ADC Analog watchdog 1 flag */
            __IO uint32_t AWD2 : 1;                  /*!< ADC Analog watchdog 2 flag */
            __IO uint32_t AWD3 : 1;                  /*!< ADC Analog watchdog 3 flag */
            __IO uint32_t JQOVF : 1;                 /*!< ADC Injected Context Queue Overflow flag */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } ISR;                                   /*!< ADC Interrupt and Status Register,                 Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t ADRDY : 1;                 /*!< ADC Ready (ADRDY) interrupt source */
            __IO uint32_t EOSMP : 1;                 /*!< ADC End of Sampling interrupt source */
            __IO uint32_t EOC : 1;                   /*!< ADC End of Regular Conversion interrupt source */
            __IO uint32_t EOS : 1;                   /*!< ADC End of Regular sequence of Conversions interrupt source */
            __IO uint32_t OVR : 1;                   /*!< ADC overrun interrupt source */
            __IO uint32_t JEOC : 1;                  /*!< ADC End of Injected Conversion interrupt source */
            __IO uint32_t JEOS : 1;                  /*!< ADC End of Injected sequence of Conversions interrupt source */
            __IO uint32_t AWD1 : 1;                  /*!< ADC Analog watchdog 1 interrupt source */
            __IO uint32_t AWD2 : 1;                  /*!< ADC Analog watchdog 2 interrupt source */
            __IO uint32_t AWD3 : 1;                  /*!< ADC Analog watchdog 3 interrupt source */
            __IO uint32_t JQOVF : 1;                 /*!< ADC Injected Context Queue Overflow interrupt source */
                 uint32_t __RESERVED0 : 21;
        } b;
        __IO uint32_t w;
    } IER;                                   /*!< ADC Interrupt Enable Register,                     Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t ADEN : 1;                  /*!< ADC Enable control */
            __IO uint32_t ADDIS : 1;                 /*!< ADC Disable command */
            __IO uint32_t ADSTART : 1;               /*!< ADC Start of Regular conversion */
            __IO uint32_t JADSTART : 1;              /*!< ADC Start of injected conversion */
            __IO uint32_t ADSTP : 1;                 /*!< ADC Stop of Regular conversion */
            __IO uint32_t JADSTP : 1;                /*!< ADC Stop of injected conversion */
                 uint32_t __RESERVED0 : 22;
            __IO uint32_t ADVREGEN : 2;              /*!< ADC Voltage regulator Enable */
            __IO uint32_t ADCALDIF : 1;              /*!< ADC Differential Mode for calibration */
            __IO uint32_t ADCAL : 1;                 /*!< ADC Calibration */
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< ADC control register,                              Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t DMAEN : 1;                 /*!< ADC DMA Enable */
            __IO uint32_t DMACFG : 1;                /*!< ADC DMA configuration */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t RES : 2;                   /*!< ADC Data resolution */
            __IO uint32_t ALIGN : 1;                 /*!< ADC Data Alignement */
            __IO uint32_t EXTSEL : 4;                /*!< ADC External trigger selection for regular group */
            __IO uint32_t EXTEN : 2;                 /*!< ADC External trigger enable and polarity selection for regular channels */
            __IO uint32_t OVRMOD : 1;                /*!< ADC overrun mode */
            __IO uint32_t CONT : 1;                  /*!< ADC Single/continuous conversion mode for regular conversion */
            __IO uint32_t AUTDLY : 1;                /*!< ADC Delayed conversion mode */
            __IO uint32_t AUTOFF : 1;                /*!< ADC Auto power OFF */
            __IO uint32_t DISCEN : 1;                /*!< ADC Discontinuous mode for regular channels */
            __IO uint32_t DISCNUM : 3;               /*!< ADC Discontinuous mode channel count */
            __IO uint32_t JDISCEN : 1;               /*!< ADC Discontinous mode on injected channels */
            __IO uint32_t JQM : 1;                   /*!< ADC JSQR Queue mode */
            __IO uint32_t AWD1SGL : 1;               /*!< Eanble the watchdog 1 on a single channel or on all channels */
            __IO uint32_t AWD1EN : 1;                /*!< ADC Analog watchdog 1 enable on regular Channels */
            __IO uint32_t JAWD1EN : 1;               /*!< ADC Analog watchdog 1 enable on injected Channels */
            __IO uint32_t JAUTO : 1;                 /*!< ADC Automatic injected group conversion */
            __IO uint32_t AWD1CH : 5;                /*!< ADC Analog watchdog 1 Channel selection */
                 uint32_t __RESERVED1 : 1;
        } b;
        __IO uint32_t w;
    } CFGR;                                  /*!< ADC Configuration register,                        Address offset: 0x0C */
         uint32_t __RESERVED0;               /*!< Reserved, 0x010                                                         */
    union {
        struct {
            __IO uint32_t SMP0 : 3;                  /*!< ADC Channel 0 Sampling time selection  */
            __IO uint32_t SMP1 : 3;                  /*!< ADC Channel 1 Sampling time selection  */
            __IO uint32_t SMP2 : 3;                  /*!< ADC Channel 2 Sampling time selection  */
            __IO uint32_t SMP3 : 3;                  /*!< ADC Channel 3 Sampling time selection  */
            __IO uint32_t SMP4 : 3;                  /*!< ADC Channel 4 Sampling time selection  */
            __IO uint32_t SMP5 : 3;                  /*!< ADC Channel 5 Sampling time selection  */
            __IO uint32_t SMP6 : 3;                  /*!< ADC Channel 6 Sampling time selection  */
            __IO uint32_t SMP7 : 3;                  /*!< ADC Channel 7 Sampling time selection  */
            __IO uint32_t SMP8 : 3;                  /*!< ADC Channel 8 Sampling time selection  */
            __IO uint32_t SMP9 : 3;                  /*!< ADC Channel 9 Sampling time selection  */
                 uint32_t __RESERVED0 : 2;
        } b;
        __IO uint32_t w;
    } SMPR1;                                 /*!< ADC sample time register 1,                        Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t SMP10 : 3;                 /*!< ADC Channel 10 Sampling time selection  */
            __IO uint32_t SMP11 : 3;                 /*!< ADC Channel 11 Sampling time selection  */
            __IO uint32_t SMP12 : 3;                 /*!< ADC Channel 12 Sampling time selection  */
            __IO uint32_t SMP13 : 3;                 /*!< ADC Channel 13 Sampling time selection  */
            __IO uint32_t SMP14 : 3;                 /*!< ADC Channel 14 Sampling time selection  */
            __IO uint32_t SMP15 : 3;                 /*!< ADC Channel 15 Sampling time selection  */
            __IO uint32_t SMP16 : 3;                 /*!< ADC Channel 16 Sampling time selection  */
            __IO uint32_t SMP17 : 3;                 /*!< ADC Channel 17 Sampling time selection  */
            __IO uint32_t SMP18 : 3;                 /*!< ADC Channel 18 Sampling time selection  */
                 uint32_t __RESERVED0 : 5;
        } b;
        __IO uint32_t w;
    } SMPR2;                                 /*!< ADC sample time register 2,                        Address offset: 0x18 */
         uint32_t __RESERVED1;               /*!< Reserved, 0x01C                                                         */
    union {
        struct {
            __IO uint32_t LT1 : 12;                  /*!< ADC Analog watchdog 1 lower threshold */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t HT1 : 12;                  /*!< ADC Analog watchdog 1 higher threshold */
                 uint32_t __RESERVED1 : 4;
        } b;
        __IO uint32_t w;
    } TR1;                                   /*!< ADC watchdog threshold register 1,                 Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t LT2 : 8;                   /*!< ADC Analog watchdog 2 lower threshold */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t HT2 : 8;                   /*!< ADC Analog watchdog 2 higher threshold */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } TR2;                                   /*!< ADC watchdog threshold register 2,                 Address offset: 0x24 */
    union {
        struct {
            __IO uint32_t LT3 : 8;                   /*!< ADC Analog watchdog 3 lower threshold */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t HT3 : 8;                   /*!< ADC Analog watchdog 3 higher threshold */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } TR3;                                   /*!< ADC watchdog threshold register 3,                 Address offset: 0x28 */
         uint32_t __RESERVED2;               /*!< Reserved, 0x02C                                                         */
    union {
        struct {
            __IO uint32_t L : 4;                     /*!< ADC regular channel sequence lenght */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t SQ1 : 5;                   /*!< ADC 1st conversion in regular sequence */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SQ2 : 5;                   /*!< ADC 2nd conversion in regular sequence */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SQ3 : 5;                   /*!< ADC 3rd conversion in regular sequence */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t SQ4 : 5;                   /*!< ADC 4th conversion in regular sequence */
                 uint32_t __RESERVED4 : 3;
        } b;
        __IO uint32_t w;
    } SQR1;                                  /*!< ADC regular sequence register 1,                   Address offset: 0x30 */
    union {
        struct {
            __IO uint32_t SQ5 : 5;                   /*!< ADC 5th conversion in regular sequence */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SQ6 : 5;                   /*!< ADC 6th conversion in regular sequence */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SQ7 : 5;                   /*!< ADC 7th conversion in regular sequence */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SQ8 : 5;                   /*!< ADC 8th conversion in regular sequence */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t SQ9 : 5;                   /*!< ADC 9th conversion in regular sequence */
                 uint32_t __RESERVED4 : 3;
        } b;
        __IO uint32_t w;
    } SQR2;                                  /*!< ADC regular sequence register 2,                   Address offset: 0x34 */
    union {
        struct {
            __IO uint32_t SQ10 : 5;                  /*!< ADC 10th conversion in regular sequence */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SQ11 : 5;                  /*!< ADC 11th conversion in regular sequence */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t SQ12 : 5;                  /*!< ADC 12th conversion in regular sequence */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t SQ13 : 5;                  /*!< ADC 13th conversion in regular sequence */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t SQ14 : 5;                  /*!< ADC 14th conversion in regular sequence */
                 uint32_t __RESERVED4 : 3;
        } b;
        __IO uint32_t w;
    } SQR3;                                  /*!< ADC regular sequence register 3,                   Address offset: 0x38 */
    union {
        struct {
            __IO uint32_t SQ15 : 5;                  /*!< ADC 15th conversion in regular sequence */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SQ16 : 5;                  /*!< ADC 16th conversion in regular sequence */
                 uint32_t __RESERVED1 : 21;
        } b;
        __IO uint32_t w;
    } SQR4;                                  /*!< ADC regular sequence register 4,                   Address offset: 0x3C */
    __IO uint32_t DR;                        /*!< ADC regular data register,                         Address offset: 0x40 */
         uint32_t __RESERVED3;               /*!< Reserved, 0x044                                                         */
         uint32_t __RESERVED4;               /*!< Reserved, 0x048                                                         */
    union {
        struct {
            __IO uint32_t JL : 2;                    /*!< ADC injected channel sequence length */
            __IO uint32_t JEXTSEL : 4;               /*!< ADC external trigger selection for injected group */
            __IO uint32_t JEXTEN : 2;                /*!< ADC external trigger enable and polarity selection for injected channels */
            __IO uint32_t JSQ1 : 5;                  /*!< ADC 1st conversion in injected sequence */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t JSQ2 : 5;                  /*!< ADC 2nd conversion in injected sequence */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t JSQ3 : 5;                  /*!< ADC 3rd conversion in injected sequence */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t JSQ4 : 5;                  /*!< ADC 4th conversion in injected sequence */
                 uint32_t __RESERVED3 : 1;
        } b;
        __IO uint32_t w;
    } JSQR;                                  /*!< ADC injected sequence register,                    Address offset: 0x4C */
         uint32_t __RESERVED5[4];               /*!< Reserved, 0x050 - 0x05C                                                 */
    union {
        struct {
            __IO uint32_t OFFSET1 : 12;              /*!< ADC data offset 1 for channel programmed into bits OFFSET1_CH[4:0] */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET1_CH : 5;            /*!< ADC Channel selection for the data offset 1 */
            __IO uint32_t OFFSET1_EN : 1;            /*!< ADC offset 1 enable */
        } b;
        __IO uint32_t w;
    } OFR1;                                  /*!< ADC offset register 1,                             Address offset: 0x60 */
    union {
        struct {
            __IO uint32_t OFFSET2 : 12;              /*!< ADC data offset 2 for channel programmed into bits OFFSET2_CH[4:0] */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET2_CH : 5;            /*!< ADC Channel selection for the data offset 2 */
            __IO uint32_t OFFSET2_EN : 1;            /*!< ADC offset 2 enable */
        } b;
        __IO uint32_t w;
    } OFR2;                                  /*!< ADC offset register 2,                             Address offset: 0x64 */
    union {
        struct {
            __IO uint32_t OFFSET3 : 12;              /*!< ADC data offset 3 for channel programmed into bits OFFSET3_CH[4:0] */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET3_CH : 5;            /*!< ADC Channel selection for the data offset 3 */
            __IO uint32_t OFFSET3_EN : 1;            /*!< ADC offset 3 enable */
        } b;
        __IO uint32_t w;
    } OFR3;                                  /*!< ADC offset register 3,                             Address offset: 0x68 */
    union {
        struct {
            __IO uint32_t OFFSET4 : 12;              /*!< ADC data offset 4 for channel programmed into bits OFFSET4_CH[4:0] */
                 uint32_t __RESERVED0 : 14;
            __IO uint32_t OFFSET4_CH : 5;            /*!< ADC Channel selection for the data offset 4 */
            __IO uint32_t OFFSET4_EN : 1;            /*!< ADC offset 4 enable */
        } b;
        __IO uint32_t w;
    } OFR4;                                  /*!< ADC offset register 4,                             Address offset: 0x6C */
         uint32_t __RESERVED6[4];               /*!< Reserved, 0x070 - 0x07C                                                 */
    __IO uint32_t JDR1;                      /*!< ADC injected data register 1,                      Address offset: 0x80 */
    __IO uint32_t JDR2;                      /*!< ADC injected data register 2,                      Address offset: 0x84 */
    __IO uint32_t JDR3;                      /*!< ADC injected data register 3,                      Address offset: 0x88 */
    __IO uint32_t JDR4;                      /*!< ADC injected data register 4,                      Address offset: 0x8C */
         uint32_t __RESERVED7[4];               /*!< Reserved, 0x090 - 0x09C                                                 */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t AWD2CH : 18;               /*!< ADC Analog watchdog 2 channel selection */
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } AWD2CR;                                /*!< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0 */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t AWD3CH : 18;               /*!< ADC Analog watchdog 3 channel selection */
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } AWD3CR;                                /*!< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4 */
         uint32_t __RESERVED8;               /*!< Reserved, 0x0A8                                                         */
         uint32_t __RESERVED9;               /*!< Reserved, 0x0AC                                                         */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DIFSEL : 18;               /*!< ADC differential modes for channels 1 to 18 */
                 uint32_t __RESERVED1 : 13;
        } b;
        __IO uint32_t w;
    } DIFSEL;                                /*!< ADC  Differential Mode Selection Register,         Address offset: 0xB0 */
    union {
        struct {
            __IO uint32_t CALFACT_S : 7;             /*!< ADC calibration factors in single-ended mode */
                 uint32_t __RESERVED0 : 9;
            __IO uint32_t CALFACT_D : 7;             /*!< ADC calibration factors in differential mode */
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } CALFACT;                               /*!< ADC  Calibration Factors,                          Address offset: 0xB4 */
} ADC_TypeDef;



typedef struct {
    union {
        struct {
            __IO uint32_t ADRDY_MST : 1;             /*!< Master ADC ready */
            __IO uint32_t ADRDY_EOSMP_MST : 1;       /*!< End of sampling phase flag of the master ADC */
            __IO uint32_t ADRDY_EOC_MST : 1;         /*!< End of regular conversion of the master ADC */
            __IO uint32_t ADRDY_EOS_MST : 1;         /*!< End of regular sequence flag of the master ADC */
            __IO uint32_t ADRDY_OVR_MST : 1;         /*!< Overrun flag of the master ADC */
            __IO uint32_t ADRDY_JEOC_MST : 1;        /*!< End of injected conversion of the master ADC */
            __IO uint32_t ADRDY_JEOS_MST : 1;        /*!< End of injected sequence flag of the master ADC */
            __IO uint32_t AWD1_MST : 1;              /*!< Analog watchdog 1 flag of the master ADC */
            __IO uint32_t AWD2_MST : 1;              /*!< Analog watchdog 2 flag of the master ADC */
            __IO uint32_t AWD3_MST : 1;              /*!< Analog watchdog 3 flag of the master ADC */
            __IO uint32_t JQOVF_MST : 1;             /*!< Injected context queue overflow flag of the master ADC */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t ADRDY_SLV : 1;             /*!< Slave ADC ready */
            __IO uint32_t ADRDY_EOSMP_SLV : 1;       /*!< End of sampling phase flag of the slave ADC */
            __IO uint32_t ADRDY_EOC_SLV : 1;         /*!< End of regular conversion of the slave ADC */
            __IO uint32_t ADRDY_EOS_SLV : 1;         /*!< End of regular sequence flag of the slave ADC */
            __IO uint32_t ADRDY_OVR_SLV : 1;         /*!< Overrun flag of the slave ADC */
            __IO uint32_t ADRDY_JEOC_SLV : 1;        /*!< End of injected conversion of the slave ADC */
            __IO uint32_t ADRDY_JEOS_SLV : 1;        /*!< End of injected sequence flag of the slave ADC */
            __IO uint32_t AWD1_SLV : 1;              /*!< Analog watchdog 1 flag of the slave ADC */
            __IO uint32_t AWD2_SLV : 1;              /*!< Analog watchdog 2 flag of the slave ADC */
            __IO uint32_t AWD3_SLV : 1;              /*!< Analog watchdog 3 flag of the slave ADC */
            __IO uint32_t JQOVF_SLV : 1;             /*!< Injected context queue overflow flag of the slave ADC */
                 uint32_t __RESERVED1 : 5;
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< ADC Common status register,                  Address offset: ADC1/3 base address + 0x300 */
         uint32_t __RESERVED0;               /*!< Reserved, ADC1/3 base address + 0x304                                                    */
    union {
        struct {
            __IO uint32_t MULTI : 5;                 /*!< Multi ADC mode selection */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t DELAY : 4;                 /*!< Delay between 2 sampling phases */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t DMACFG : 1;                /*!< DMA configuration for multi-ADC mode */
            __IO uint32_t MDMA : 2;                  /*!< DMA mode for multi-ADC mode */
            __IO uint32_t CKMODE : 2;                /*!< ADC clock mode */
                 uint32_t __RESERVED2 : 4;
            __IO uint32_t VREFEN : 1;                /*!< VREFINT enable */
            __IO uint32_t TSEN : 1;                  /*!< Temperature sensor enable */
            __IO uint32_t VBATEN : 1;                /*!< VBAT enable */
                 uint32_t __RESERVED3 : 7;
        } b;
        __IO uint32_t w;
    } CCR;                                   /*!< ADC common control register,                 Address offset: ADC1/3 base address + 0x308 */
    union {
        struct {
            __IO uint32_t RDATA_MST : 16;            /*!< Regular Data of the master ADC */
            __IO uint32_t RDATA_SLV : 16;            /*!< Regular Data of the master ADC */
        } b;
        __IO uint32_t w;
    } CDR;                                   /*!< ADC common regular data register for dual
                                         AND triple modes,                            Address offset: ADC1/3 base address + 0x30C */
} ADC_Common_TypeDef;


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
                 uint32_t __RESERVED1 : 16;
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
                 uint32_t __RESERVED0 : 31;
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
    CAN_FilterRegister_TypeDef sFilterRegister[14];/*!< CAN Filter Register,           Address offset: 0x240-0x31C   */
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
             uint32_t __RESERVED1[16];
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
             uint32_t __RESERVED0[31];
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
    } sFilterRegister[14];                       /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_BitBand_TypeDef;



/**
  * @brief Analog Comparators
  */


typedef struct {
    union {
        struct {
            __IO uint32_t EN : 1;               /*!< COMPx enable */
            __IO uint32_t SW1 : 1;              /*!< COMPx SW1 switch control */
            __IO uint32_t MODE : 2;             /*!< COMPx power mode */
            __IO uint32_t INSEL : 3;            /*!< COMPx inverting input select */
            __IO uint32_t NONINSEL : 1;         /*!< COMPx non inverting input select */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t WNDWEN : 1;           /*!< COMPx window mode enable */
            __IO uint32_t OUTSEL : 4;           /*!< COMPx output select */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t POL : 1;              /*!< COMPx output polarity */
            __IO uint32_t HYST : 2;             /*!< COMPx hysteresis */
            __IO uint32_t BLANKING : 2;         /*!< COMPx blanking */
                 uint32_t __RESERVED2 : 10;
            __IO uint32_t OUT : 1;              /*!< COMPx output level */
            __IO uint32_t LOCK : 1;             /*!< COMPx lock */
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< Comparator control Status register, Address offset: 0x00 */
} COMP_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN;                       /*!< COMPx enable */
        __IO uint32_t SW1;                      /*!< COMPx SW1 switch control */
        __IO uint32_t MODE[2];                  /*!< COMPx power mode */
        __IO uint32_t INSEL[3];                 /*!< COMPx inverting input select */
        __IO uint32_t NONINSEL;                 /*!< COMPx non inverting input select */
             uint32_t __RESERVED0;
        __IO uint32_t WNDWEN;                   /*!< COMPx window mode enable */
        __IO uint32_t OUTSEL[4];                /*!< COMPx output select */
             uint32_t __RESERVED1;
        __IO uint32_t POL;                      /*!< COMPx output polarity */
        __IO uint32_t HYST[2];                  /*!< COMPx hysteresis */
        __IO uint32_t BLANKING[2];              /*!< COMPx blanking */
             uint32_t __RESERVED2[10];
        __IO uint32_t OUT;                      /*!< COMPx output level */
        __IO uint32_t LOCK;                     /*!< COMPx lock */
    } CSR;                                   /*!< Comparator control Status register, Address offset: 0x00 */
} COMP_BitBand_TypeDef;



/**
  * @brief CRC calculation unit
  */


typedef struct {
    __IO uint32_t DR;                        /*!< CRC Data register,                           Address offset: 0x00 */
    __IO uint8_t IDR;                        /*!< CRC Independent data register,               Address offset: 0x04 */
         uint8_t __RESERVED0;                /*!< Reserved,                                                    0x05 */
         uint16_t __RESERVED1;               /*!< Reserved,                                                    0x06 */
    union {
        struct {
            __IO uint32_t RESET : 1;                 /*!< RESET the CRC computation unit bit */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t POLYSIZE : 2;              /*!< Polynomial size bits */
            __IO uint32_t REV_IN : 2;                /*!< REV_IN Reverse Input Data bits */
            __IO uint32_t REV_OUT : 1;               /*!< REV_OUT Reverse Output Data bits */
                 uint32_t __RESERVED1 : 24;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< CRC Control register,                        Address offset: 0x08 */
         uint32_t __RESERVED2;               /*!< Reserved,                                                    0x0C */
    __IO uint32_t INIT;                      /*!< Initial CRC value register,                  Address offset: 0x10 */
    __IO uint32_t POL;                       /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_TypeDef;


typedef struct {
    __IO uint32_t DR[32];                    /*!< CRC Data register,                           Address offset: 0x00 */
    __IO uint32_t IDR[8];                    /*!< CRC Independent data register,               Address offset: 0x04 */
         uint32_t __RESERVED0[8];            /*!< Reserved,                                                    0x05 */
         uint32_t __RESERVED1[16];           /*!< Reserved,                                                    0x06 */
    struct {
        __IO uint32_t RESET;                     /*!< RESET the CRC computation unit bit */
             uint32_t __RESERVED0[2];
        __IO uint32_t POLYSIZE[2];               /*!< Polynomial size bits */
        __IO uint32_t REV_IN[2];                 /*!< REV_IN Reverse Input Data bits */
        __IO uint32_t REV_OUT;                   /*!< REV_OUT Reverse Output Data bits */
             uint32_t __RESERVED1[24];
    } CR;                                    /*!< CRC Control register,                        Address offset: 0x08 */
         uint32_t __RESERVED2[32];           /*!< Reserved,                                                    0x0C */
    __IO uint32_t INIT[32];                  /*!< Initial CRC value register,                  Address offset: 0x10 */
    __IO uint32_t POL[32];                   /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_BitBand_TypeDef;



/**
  * @brief Digital to Analog Converter
  */


typedef struct {
    union {
        struct {
            __IO uint32_t EN1 : 1;                   /*!< DAC channel1 enable */
            __IO uint32_t BOFF1 : 1;                 /*!< DAC channel1 output buffer disable */
            __IO uint32_t TEN1 : 1;                  /*!< DAC channel1 Trigger enable */
            __IO uint32_t TSEL1 : 3;                 /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
            __IO uint32_t WAVE1 : 2;                 /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
            __IO uint32_t MAMP1 : 4;                 /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
            __IO uint32_t DMAEN1 : 1;                /*!< DAC channel1 DMA enable */
            __IO uint32_t DMAUDRIE1 : 1;             /*!< DAC channel1 DMA underrun IT enable */
                 uint32_t __RESERVED0 : 18;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< DAC control register,                                    Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t SWTRIG1 : 1;               /*!< DAC channel1 software trigger */
                 uint32_t __RESERVED0 : 31;
        } b;
        __IO uint32_t w;
    } SWTRIGR;                               /*!< DAC software trigger register,                           Address offset: 0x04 */
    struct {
        __IO uint32_t D12R;                      /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L;                      /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R;                       /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR1;                                  /*!< DAC channel 1 data holding registers */
    __IO uint32_t RESERVED0;                 /*!< Reserved,                                                                0x14 */
    __IO uint32_t RESERVED1;                 /*!< Reserved,                                                                0x18 */
    __IO uint32_t RESERVED2;                 /*!< Reserved,                                                                0x1C */
    struct {
        __IO uint32_t D12R;                      /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L;                      /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R;                       /*!< DAC channel 8-bit right-aligned data holding register */
    } DHRD;                                  /*!< DAC dual channel data holding registers */
    __IO uint32_t DOR1;                      /*!< DAC channel1 data output register,                       Address offset: 0x2C */
    __IO uint32_t RESERVED3;                 /*!< Reserved,                                                                0x30 */
    union {
        struct {
                 uint32_t __RESERVED0 : 13;
            __IO uint32_t DMAUDR1 : 1;               /*!< DAC channel1 DMA underrun flag */
                 uint32_t __RESERVED1 : 18;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN1;                       /*!< DAC channel1 enable */
        __IO uint32_t BOFF1;                     /*!< DAC channel1 output buffer disable */
        __IO uint32_t TEN1;                      /*!< DAC channel1 Trigger enable */
        __IO uint32_t TSEL1[3];                  /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
        __IO uint32_t WAVE1[2];                  /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
        __IO uint32_t MAMP1[4];                  /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
        __IO uint32_t DMAEN1;                    /*!< DAC channel1 DMA enable */
        __IO uint32_t DMAUDRIE1;                 /*!< DAC channel1 DMA underrun IT enable */
             uint32_t __RESERVED0[18];
    } CR;                                    /*!< DAC control register,                                    Address offset: 0x00 */
    struct {
        __IO uint32_t SWTRIG1;                   /*!< DAC channel1 software trigger */
             uint32_t __RESERVED0[31];
    } SWTRIGR;                               /*!< DAC software trigger register,                           Address offset: 0x04 */
    struct {
        __IO uint32_t D12R[32];                  /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L[32];                  /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R[32];                   /*!< DAC channel 8-bit right-aligned data holding register */
    } DHR1;                                  /*!< DAC channel 1 data holding registers */
    __IO uint32_t RESERVED0[32];             /*!< Reserved,                                                                0x14 */
    __IO uint32_t RESERVED1[32];             /*!< Reserved,                                                                0x18 */
    __IO uint32_t RESERVED2[32];             /*!< Reserved,                                                                0x1C */
    struct {
        __IO uint32_t D12R[32];                  /*!< DAC channel 12-bit right-aligned data holding register */
        __IO uint32_t D12L[32];                  /*!< DAC channel 12-bit left-aligned data holding register */
        __IO uint32_t D8R[32];                   /*!< DAC channel 8-bit right-aligned data holding register */
    } DHRD;                                  /*!< DAC dual channel data holding registers */
    __IO uint32_t DOR1[32];                  /*!< DAC channel1 data output register,                       Address offset: 0x2C */
    __IO uint32_t RESERVED3[32];             /*!< Reserved,                                                                0x30 */
    struct {
             uint32_t __RESERVED0[13];
        __IO uint32_t DMAUDR1;                   /*!< DAC channel1 DMA underrun flag */
             uint32_t __RESERVED1[18];
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
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DBG_TIM6_STOP : 1;
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t DBG_RTC_STOP : 1;
            __IO uint32_t DBG_WWDG_STOP : 1;
            __IO uint32_t DBG_IWDG_STOP : 1;
                 uint32_t __RESERVED2 : 8;
            __IO uint32_t DBG_I2C1_SMBUS_TIMEOUT : 1;
            __IO uint32_t DBG_I2C2_SMBUS_TIMEOUT : 1;
                 uint32_t __RESERVED3 : 2;
            __IO uint32_t DBG_CAN_STOP : 1;
                 uint32_t __RESERVED4 : 6;
        } b;
        __IO uint32_t w;
    } APB1FZ;                                /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t DBG_TIM1_STOP : 1;
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t DBG_TIM15_STOP : 1;
            __IO uint32_t DBG_TIM16_STOP : 1;
            __IO uint32_t DBG_TIM17_STOP : 1;
                 uint32_t __RESERVED1 : 27;
        } b;
        __IO uint32_t w;
    } APB2FZ;                                /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
} DBGMCU_TypeDef;


/**
  * @brief DMA Controller
  */


typedef struct {
    union {
        struct {
            __IO uint32_t EN : 1;                    /*!< Channel enable                      */
            __IO uint32_t TCIE : 1;                  /*!< Transfer complete interrupt enable  */
            __IO uint32_t HTIE : 1;                  /*!< Half Transfer interrupt enable      */
            __IO uint32_t TEIE : 1;                  /*!< Transfer error interrupt enable     */
            __IO uint32_t DIR : 1;                   /*!< Data transfer direction             */
            __IO uint32_t CIRC : 1;                  /*!< Circular mode                       */
            __IO uint32_t PINC : 1;                  /*!< Peripheral increment mode           */
            __IO uint32_t MINC : 1;                  /*!< Memory increment mode               */
            __IO uint32_t PSIZE : 2;                 /*!< PSIZE[1:0] bits (Peripheral size)   */
            __IO uint32_t MSIZE : 2;                 /*!< MSIZE[1:0] bits (Memory size)       */
            __IO uint32_t PL : 2;                    /*!< PL[1:0] bits(Channel Priority level)*/
            __IO uint32_t MEM2MEM : 1;               /*!< Memory to memory mode               */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CCR;                                   /*!< DMA channel x configuration register                                           */
    __IO uint32_t CNDTR;                     /*!< DMA channel x number of data register                                          */
    __IO uint32_t CPAR;                      /*!< DMA channel x peripheral address register                                      */
    __IO uint32_t CMAR;                      /*!< DMA channel x memory address register                                          */
} DMA_Channel_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN;                        /*!< Channel enable                      */
        __IO uint32_t TCIE;                      /*!< Transfer complete interrupt enable  */
        __IO uint32_t HTIE;                      /*!< Half Transfer interrupt enable      */
        __IO uint32_t TEIE;                      /*!< Transfer error interrupt enable     */
        __IO uint32_t DIR;                       /*!< Data transfer direction             */
        __IO uint32_t CIRC;                      /*!< Circular mode                       */
        __IO uint32_t PINC;                      /*!< Peripheral increment mode           */
        __IO uint32_t MINC;                      /*!< Memory increment mode               */
        __IO uint32_t PSIZE[2];                  /*!< PSIZE[1:0] bits (Peripheral size)   */
        __IO uint32_t MSIZE[2];                  /*!< MSIZE[1:0] bits (Memory size)       */
        __IO uint32_t PL[2];                     /*!< PL[1:0] bits(Channel Priority level)*/
        __IO uint32_t MEM2MEM;                   /*!< Memory to memory mode               */
             uint32_t __RESERVED0[17];
    } CCR;                                   /*!< DMA channel x configuration register                                           */
    __IO uint32_t CNDTR[32];                 /*!< DMA channel x number of data register                                          */
    __IO uint32_t CPAR[32];                  /*!< DMA channel x peripheral address register                                      */
    __IO uint32_t CMAR[32];                  /*!< DMA channel x memory address register                                          */
} DMA_Channel_BitBand_TypeDef;




typedef struct {
    union {
        struct {
            __IO uint32_t GIF1 : 1;                  /*!< Channel 1 Global interrupt flag */
            __IO uint32_t TCIF1 : 1;                 /*!< Channel 1 Transfer Complete flag */
            __IO uint32_t HTIF1 : 1;                 /*!< Channel 1 Half Transfer flag */
            __IO uint32_t TEIF1 : 1;                 /*!< Channel 1 Transfer Error flag */
            __IO uint32_t GIF2 : 1;                  /*!< Channel 2 Global interrupt flag */
            __IO uint32_t TCIF2 : 1;                 /*!< Channel 2 Transfer Complete flag */
            __IO uint32_t HTIF2 : 1;                 /*!< Channel 2 Half Transfer flag */
            __IO uint32_t TEIF2 : 1;                 /*!< Channel 2 Transfer Error flag */
            __IO uint32_t GIF3 : 1;                  /*!< Channel 3 Global interrupt flag */
            __IO uint32_t TCIF3 : 1;                 /*!< Channel 3 Transfer Complete flag */
            __IO uint32_t HTIF3 : 1;                 /*!< Channel 3 Half Transfer flag */
            __IO uint32_t TEIF3 : 1;                 /*!< Channel 3 Transfer Error flag */
            __IO uint32_t GIF4 : 1;                  /*!< Channel 4 Global interrupt flag */
            __IO uint32_t TCIF4 : 1;                 /*!< Channel 4 Transfer Complete flag */
            __IO uint32_t HTIF4 : 1;                 /*!< Channel 4 Half Transfer flag */
            __IO uint32_t TEIF4 : 1;                 /*!< Channel 4 Transfer Error flag */
            __IO uint32_t GIF5 : 1;                  /*!< Channel 5 Global interrupt flag */
            __IO uint32_t TCIF5 : 1;                 /*!< Channel 5 Transfer Complete flag */
            __IO uint32_t HTIF5 : 1;                 /*!< Channel 5 Half Transfer flag */
            __IO uint32_t TEIF5 : 1;                 /*!< Channel 5 Transfer Error flag */
            __IO uint32_t GIF6 : 1;                  /*!< Channel 6 Global interrupt flag */
            __IO uint32_t TCIF6 : 1;                 /*!< Channel 6 Transfer Complete flag */
            __IO uint32_t HTIF6 : 1;                 /*!< Channel 6 Half Transfer flag */
            __IO uint32_t TEIF6 : 1;                 /*!< Channel 6 Transfer Error flag */
            __IO uint32_t GIF7 : 1;                  /*!< Channel 7 Global interrupt flag */
            __IO uint32_t TCIF7 : 1;                 /*!< Channel 7 Transfer Complete flag */
            __IO uint32_t HTIF7 : 1;                 /*!< Channel 7 Half Transfer flag */
            __IO uint32_t TEIF7 : 1;                 /*!< Channel 7 Transfer Error flag */
                 uint32_t __RESERVED0 : 4;
        } b;
        __IO uint32_t w;
    } ISR;                                   /*!< DMA interrupt status register,                            Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CGIF1 : 1;                 /*!< Channel 1 Global interrupt clear */
            __IO uint32_t CTCIF1 : 1;                /*!< Channel 1 Transfer Complete clear */
            __IO uint32_t CHTIF1 : 1;                /*!< Channel 1 Half Transfer clear */
            __IO uint32_t CTEIF1 : 1;                /*!< Channel 1 Transfer Error clear */
            __IO uint32_t CGIF2 : 1;                 /*!< Channel 2 Global interrupt clear */
            __IO uint32_t CTCIF2 : 1;                /*!< Channel 2 Transfer Complete clear */
            __IO uint32_t CHTIF2 : 1;                /*!< Channel 2 Half Transfer clear */
            __IO uint32_t CTEIF2 : 1;                /*!< Channel 2 Transfer Error clear */
            __IO uint32_t CGIF3 : 1;                 /*!< Channel 3 Global interrupt clear */
            __IO uint32_t CTCIF3 : 1;                /*!< Channel 3 Transfer Complete clear */
            __IO uint32_t CHTIF3 : 1;                /*!< Channel 3 Half Transfer clear */
            __IO uint32_t CTEIF3 : 1;                /*!< Channel 3 Transfer Error clear */
            __IO uint32_t CGIF4 : 1;                 /*!< Channel 4 Global interrupt clear */
            __IO uint32_t CTCIF4 : 1;                /*!< Channel 4 Transfer Complete clear */
            __IO uint32_t CHTIF4 : 1;                /*!< Channel 4 Half Transfer clear */
            __IO uint32_t CTEIF4 : 1;                /*!< Channel 4 Transfer Error clear */
            __IO uint32_t CGIF5 : 1;                 /*!< Channel 5 Global interrupt clear */
            __IO uint32_t CTCIF5 : 1;                /*!< Channel 5 Transfer Complete clear */
            __IO uint32_t CHTIF5 : 1;                /*!< Channel 5 Half Transfer clear */
            __IO uint32_t CTEIF5 : 1;                /*!< Channel 5 Transfer Error clear */
            __IO uint32_t CGIF6 : 1;                 /*!< Channel 6 Global interrupt clear */
            __IO uint32_t CTCIF6 : 1;                /*!< Channel 6 Transfer Complete clear */
            __IO uint32_t CHTIF6 : 1;                /*!< Channel 6 Half Transfer clear */
            __IO uint32_t CTEIF6 : 1;                /*!< Channel 6 Transfer Error clear */
            __IO uint32_t CGIF7 : 1;                 /*!< Channel 7 Global interrupt clear */
            __IO uint32_t CTCIF7 : 1;                /*!< Channel 7 Transfer Complete clear */
            __IO uint32_t CHTIF7 : 1;                /*!< Channel 7 Half Transfer clear */
            __IO uint32_t CTEIF7 : 1;                /*!< Channel 7 Transfer Error clear */
                 uint32_t __RESERVED0 : 4;
        } b;
        __IO uint32_t w;
    } IFCR;                                  /*!< DMA interrupt flag clear register,                        Address offset: 0x04 */
} DMA_TypeDef;


typedef struct {
    struct {
        __IO uint32_t GIF1;                      /*!< Channel 1 Global interrupt flag */
        __IO uint32_t TCIF1;                     /*!< Channel 1 Transfer Complete flag */
        __IO uint32_t HTIF1;                     /*!< Channel 1 Half Transfer flag */
        __IO uint32_t TEIF1;                     /*!< Channel 1 Transfer Error flag */
        __IO uint32_t GIF2;                      /*!< Channel 2 Global interrupt flag */
        __IO uint32_t TCIF2;                     /*!< Channel 2 Transfer Complete flag */
        __IO uint32_t HTIF2;                     /*!< Channel 2 Half Transfer flag */
        __IO uint32_t TEIF2;                     /*!< Channel 2 Transfer Error flag */
        __IO uint32_t GIF3;                      /*!< Channel 3 Global interrupt flag */
        __IO uint32_t TCIF3;                     /*!< Channel 3 Transfer Complete flag */
        __IO uint32_t HTIF3;                     /*!< Channel 3 Half Transfer flag */
        __IO uint32_t TEIF3;                     /*!< Channel 3 Transfer Error flag */
        __IO uint32_t GIF4;                      /*!< Channel 4 Global interrupt flag */
        __IO uint32_t TCIF4;                     /*!< Channel 4 Transfer Complete flag */
        __IO uint32_t HTIF4;                     /*!< Channel 4 Half Transfer flag */
        __IO uint32_t TEIF4;                     /*!< Channel 4 Transfer Error flag */
        __IO uint32_t GIF5;                      /*!< Channel 5 Global interrupt flag */
        __IO uint32_t TCIF5;                     /*!< Channel 5 Transfer Complete flag */
        __IO uint32_t HTIF5;                     /*!< Channel 5 Half Transfer flag */
        __IO uint32_t TEIF5;                     /*!< Channel 5 Transfer Error flag */
        __IO uint32_t GIF6;                      /*!< Channel 6 Global interrupt flag */
        __IO uint32_t TCIF6;                     /*!< Channel 6 Transfer Complete flag */
        __IO uint32_t HTIF6;                     /*!< Channel 6 Half Transfer flag */
        __IO uint32_t TEIF6;                     /*!< Channel 6 Transfer Error flag */
        __IO uint32_t GIF7;                      /*!< Channel 7 Global interrupt flag */
        __IO uint32_t TCIF7;                     /*!< Channel 7 Transfer Complete flag */
        __IO uint32_t HTIF7;                     /*!< Channel 7 Half Transfer flag */
        __IO uint32_t TEIF7;                     /*!< Channel 7 Transfer Error flag */
             uint32_t __RESERVED0[4];
    } ISR;                                   /*!< DMA interrupt status register,                            Address offset: 0x00 */
    struct {
        __IO uint32_t CGIF1;                     /*!< Channel 1 Global interrupt clear */
        __IO uint32_t CTCIF1;                    /*!< Channel 1 Transfer Complete clear */
        __IO uint32_t CHTIF1;                    /*!< Channel 1 Half Transfer clear */
        __IO uint32_t CTEIF1;                    /*!< Channel 1 Transfer Error clear */
        __IO uint32_t CGIF2;                     /*!< Channel 2 Global interrupt clear */
        __IO uint32_t CTCIF2;                    /*!< Channel 2 Transfer Complete clear */
        __IO uint32_t CHTIF2;                    /*!< Channel 2 Half Transfer clear */
        __IO uint32_t CTEIF2;                    /*!< Channel 2 Transfer Error clear */
        __IO uint32_t CGIF3;                     /*!< Channel 3 Global interrupt clear */
        __IO uint32_t CTCIF3;                    /*!< Channel 3 Transfer Complete clear */
        __IO uint32_t CHTIF3;                    /*!< Channel 3 Half Transfer clear */
        __IO uint32_t CTEIF3;                    /*!< Channel 3 Transfer Error clear */
        __IO uint32_t CGIF4;                     /*!< Channel 4 Global interrupt clear */
        __IO uint32_t CTCIF4;                    /*!< Channel 4 Transfer Complete clear */
        __IO uint32_t CHTIF4;                    /*!< Channel 4 Half Transfer clear */
        __IO uint32_t CTEIF4;                    /*!< Channel 4 Transfer Error clear */
        __IO uint32_t CGIF5;                     /*!< Channel 5 Global interrupt clear */
        __IO uint32_t CTCIF5;                    /*!< Channel 5 Transfer Complete clear */
        __IO uint32_t CHTIF5;                    /*!< Channel 5 Half Transfer clear */
        __IO uint32_t CTEIF5;                    /*!< Channel 5 Transfer Error clear */
        __IO uint32_t CGIF6;                     /*!< Channel 6 Global interrupt clear */
        __IO uint32_t CTCIF6;                    /*!< Channel 6 Transfer Complete clear */
        __IO uint32_t CHTIF6;                    /*!< Channel 6 Half Transfer clear */
        __IO uint32_t CTEIF6;                    /*!< Channel 6 Transfer Error clear */
        __IO uint32_t CGIF7;                     /*!< Channel 7 Global interrupt clear */
        __IO uint32_t CTCIF7;                    /*!< Channel 7 Transfer Complete clear */
        __IO uint32_t CHTIF7;                    /*!< Channel 7 Half Transfer clear */
        __IO uint32_t CTEIF7;                    /*!< Channel 7 Transfer Error clear */
             uint32_t __RESERVED0[4];
    } IFCR;                                  /*!< DMA interrupt flag clear register,                        Address offset: 0x04 */
} DMA_BitBand_TypeDef;



/**
  * @brief External Interrupt/Event Controller
  */


typedef struct {
    __IO uint32_t IMR;                       /*!<EXTI Interrupt mask register,                             Address offset: 0x00 */
    __IO uint32_t EMR;                       /*!<EXTI Event mask register,                                 Address offset: 0x04 */
    __IO uint32_t RTSR;                      /*!<EXTI Rising trigger selection register ,                  Address offset: 0x08 */
    __IO uint32_t FTSR;                      /*!<EXTI Falling trigger selection register,                  Address offset: 0x0C */
    __IO uint32_t SWIER;                     /*!<EXTI Software interrupt event register,                   Address offset: 0x10 */
    __IO uint32_t PR;                        /*!<EXTI Pending register,                                    Address offset: 0x14 */
         uint32_t __RESERVED0;               /*!< Reserved, 0x18                                                                */
         uint32_t __RESERVED1;               /*!< Reserved, 0x1C                                                                */
    __IO uint32_t IMR2;                      /*!< EXTI Interrupt mask register,                            Address offset: 0x20 */
    __IO uint32_t EMR2;                      /*!< EXTI Event mask register,                                Address offset: 0x24 */
    __IO uint32_t RTSR2;                     /*!< EXTI Rising trigger selection register,                  Address offset: 0x28 */
    __IO uint32_t FTSR2;                     /*!< EXTI Falling trigger selection register,                 Address offset: 0x2C */
    __IO uint32_t SWIER2;                    /*!< EXTI Software interrupt event register,                  Address offset: 0x30 */
    __IO uint32_t PR2;                       /*!< EXTI Pending register,                                   Address offset: 0x34 */
} EXTI_TypeDef;


typedef struct {
    __IO uint32_t IMR[32];                   /*!<EXTI Interrupt mask register,                             Address offset: 0x00 */
    __IO uint32_t EMR[32];                   /*!<EXTI Event mask register,                                 Address offset: 0x04 */
    __IO uint32_t RTSR[32];                  /*!<EXTI Rising trigger selection register ,                  Address offset: 0x08 */
    __IO uint32_t FTSR[32];                  /*!<EXTI Falling trigger selection register,                  Address offset: 0x0C */
    __IO uint32_t SWIER[32];                 /*!<EXTI Software interrupt event register,                   Address offset: 0x10 */
    __IO uint32_t PR[32];                    /*!<EXTI Pending register,                                    Address offset: 0x14 */
         uint32_t __RESERVED0[32];           /*!< Reserved, 0x18                                                                */
         uint32_t __RESERVED1[32];           /*!< Reserved, 0x1C                                                                */
    __IO uint32_t IMR2[32];                  /*!< EXTI Interrupt mask register,                            Address offset: 0x20 */
    __IO uint32_t EMR2[32];                  /*!< EXTI Event mask register,                                Address offset: 0x24 */
    __IO uint32_t RTSR2[32];                 /*!< EXTI Rising trigger selection register,                  Address offset: 0x28 */
    __IO uint32_t FTSR2[32];                 /*!< EXTI Falling trigger selection register,                 Address offset: 0x2C */
    __IO uint32_t SWIER2[32];                /*!< EXTI Software interrupt event register,                  Address offset: 0x30 */
    __IO uint32_t PR2[32];                   /*!< EXTI Pending register,                                   Address offset: 0x34 */
} EXTI_BitBand_TypeDef;



/**
  * @brief FLASH Registers
  */


typedef struct {
    union {
        struct {
            __IO uint32_t LATENCY : 3;               /*!< LATENCY[2:0] bits (Latency) */
            __IO uint32_t HLFCYA : 1;                /*!< Flash Half Cycle Access Enable */
            __IO uint32_t PRFTBE : 1;                /*!< Prefetch Buffer Enable */
            __IO uint32_t PRFTBS : 1;                /*!< Prefetch Buffer Status */
                 uint32_t __RESERVED0 : 26;
        } b;
        __IO uint32_t w;
    } ACR;                                   /*!< FLASH access control register,              Address offset: 0x00 */
    __IO uint32_t KEYR;                      /*!< FLASH key register,                         Address offset: 0x04 */
    __IO uint32_t OPTKEYR;                   /*!< FLASH option key register,                  Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t BSY : 1;                   /*!< Busy */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PGERR : 1;                 /*!< Programming Error */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t WRPERR : 1;                /*!< Write Protection Error */
            __IO uint32_t EOP : 1;                   /*!< End of operation */
                 uint32_t __RESERVED2 : 26;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< FLASH status register,                      Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t PG : 1;                    /*!< Programming */
            __IO uint32_t PER : 1;                   /*!< Page Erase */
            __IO uint32_t MER : 1;                   /*!< Mass Erase */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t OPTPG : 1;                 /*!< Option Byte Programming */
            __IO uint32_t OPTER : 1;                 /*!< Option Byte Erase */
            __IO uint32_t STRT : 1;                  /*!< Start */
            __IO uint32_t LOCK : 1;                  /*!< Lock */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t OPTWRE : 1;                /*!< Option Bytes Write Enable */
            __IO uint32_t ERRIE : 1;                 /*!< Error Interrupt Enable */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t EOPIE : 1;                 /*!< End of operation interrupt enable */
            __IO uint32_t OBL_LAUNCH : 1;            /*!< OptionBytes Loader Launch */
                 uint32_t __RESERVED3 : 18;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< FLASH control register,                     Address offset: 0x10 */
    __IO uint32_t AR;                        /*!< FLASH address register,                     Address offset: 0x14 */
         uint32_t __RESERVED0;               /*!< Reserved, 0x18                                                   */
    union {
        struct {
            __IO uint32_t OPTERR : 1;                /*!< Option Byte Error */
            __IO uint32_t RDPRT : 2;                 /*!< Read protection */
                 uint32_t __RESERVED0 : 5;
            __IO uint32_t IWDG_SW : 1;               /*!< IWDG SW */
            __IO uint32_t nRST_STOP : 1;             /*!< nRST_STOP */
            __IO uint32_t nRST_STDBY : 1;            /*!< nRST_STDBY */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t nBOOT1 : 1;                /*!< nBOOT1 */
            __IO uint32_t VDDA_MONITOR : 1;          /*!< VDDA_MONITOR */
            __IO uint32_t SRAM_PE : 1;               /*!< SRAM_PE */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t DATA0 : 8;                 /*!< Data0 */
            __IO uint32_t DATA1 : 8;                 /*!< Data1 */
        } b;
        __IO uint32_t w;
    } OBR;                                   /*!< FLASH Option byte register,                 Address offset: 0x1C */
    __IO uint32_t WRPR;                      /*!< FLASH Write register,                       Address offset: 0x20 */
} FLASH_TypeDef;


typedef struct {
    struct {
        __IO uint32_t LATENCY[3];                /*!< LATENCY[2:0] bits (Latency) */
        __IO uint32_t HLFCYA;                    /*!< Flash Half Cycle Access Enable */
        __IO uint32_t PRFTBE;                    /*!< Prefetch Buffer Enable */
        __IO uint32_t PRFTBS;                    /*!< Prefetch Buffer Status */
             uint32_t __RESERVED0[26];
    } ACR;                                   /*!< FLASH access control register,              Address offset: 0x00 */
    __IO uint32_t KEYR[32];                  /*!< FLASH key register,                         Address offset: 0x04 */
    __IO uint32_t OPTKEYR[32];               /*!< FLASH option key register,                  Address offset: 0x08 */
    struct {
        __IO uint32_t BSY;                       /*!< Busy */
             uint32_t __RESERVED0;
        __IO uint32_t PGERR;                     /*!< Programming Error */
             uint32_t __RESERVED1;
        __IO uint32_t WRPERR;                    /*!< Write Protection Error */
        __IO uint32_t EOP;                       /*!< End of operation */
             uint32_t __RESERVED2[26];
    } SR;                                    /*!< FLASH status register,                      Address offset: 0x0C */
    struct {
        __IO uint32_t PG;                        /*!< Programming */
        __IO uint32_t PER;                       /*!< Page Erase */
        __IO uint32_t MER;                       /*!< Mass Erase */
             uint32_t __RESERVED0;
        __IO uint32_t OPTPG;                     /*!< Option Byte Programming */
        __IO uint32_t OPTER;                     /*!< Option Byte Erase */
        __IO uint32_t STRT;                      /*!< Start */
        __IO uint32_t LOCK;                      /*!< Lock */
             uint32_t __RESERVED1;
        __IO uint32_t OPTWRE;                    /*!< Option Bytes Write Enable */
        __IO uint32_t ERRIE;                     /*!< Error Interrupt Enable */
             uint32_t __RESERVED2;
        __IO uint32_t EOPIE;                     /*!< End of operation interrupt enable */
        __IO uint32_t OBL_LAUNCH;                /*!< OptionBytes Loader Launch */
             uint32_t __RESERVED3[18];
    } CR;                                    /*!< FLASH control register,                     Address offset: 0x10 */
    __IO uint32_t AR[32];                    /*!< FLASH address register,                     Address offset: 0x14 */
         uint32_t __RESERVED0[32];           /*!< Reserved, 0x18                                                   */
    struct {
        __IO uint32_t OPTERR;                    /*!< Option Byte Error */
        __IO uint32_t RDPRT[2];                  /*!< Read protection */
             uint32_t __RESERVED0[5];
        __IO uint32_t IWDG_SW;                   /*!< IWDG SW */
        __IO uint32_t nRST_STOP;                 /*!< nRST_STOP */
        __IO uint32_t nRST_STDBY;                /*!< nRST_STDBY */
             uint32_t __RESERVED1;
        __IO uint32_t nBOOT1;                    /*!< nBOOT1 */
        __IO uint32_t VDDA_MONITOR;              /*!< VDDA_MONITOR */
        __IO uint32_t SRAM_PE;                   /*!< SRAM_PE */
             uint32_t __RESERVED2;
        __IO uint32_t DATA0[8];                  /*!< Data0 */
        __IO uint32_t DATA1[8];                  /*!< Data1 */
    } OBR;                                   /*!< FLASH Option byte register,                 Address offset: 0x1C */
    __IO uint32_t WRPR[32];                  /*!< FLASH Write register,                       Address offset: 0x20 */
} FLASH_BitBand_TypeDef;



/**
  * @brief Option Bytes Registers
  */

typedef struct {
    union {
        struct {
            __IO uint32_t RDP : 8;                   /*!< Read protection option byte */
            __IO uint32_t nRDP : 8;                  /*!< Read protection complemented option byte */
        } b;
        __IO uint16_t w;
    } RDP;                                   /*!<FLASH option byte Read protection,             Address offset: 0x00 */
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t USER : 8;                  /*!< User option byte */
            __IO uint32_t nUSER : 8;                 /*!< User complemented option byte */
        } b;
        __IO uint16_t w;
    } USER;                                  /*!<FLASH option byte user options,                Address offset: 0x02 */
         uint16_t __RESERVED0;               /*!< Reserved,                                                     0x04 */
         uint16_t __RESERVED1;               /*!< Reserved,                                                     0x06 */
    union {
        struct {
            __IO uint32_t WRP0 : 8;                  /*!< Flash memory write protection option bytes */
            __IO uint32_t nWRP0 : 8;                 /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP0;                                  /*!<FLASH option byte write protection 0,          Address offset: 0x08 */
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t WRP1 : 8;                  /*!< Flash memory write protection option bytes */
            __IO uint32_t nWRP1 : 8;                 /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP1;                                  /*!<FLASH option byte write protection 1,          Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t WRP2 : 8;                  /*!< Flash memory write protection option bytes */
            __IO uint32_t nWRP2 : 8;                 /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP2;                                  /*!<FLASH option byte write protection 2,          Address offset: 0x10 */
    union {
        struct {
                 uint32_t __RESERVED0 : 16;
            __IO uint32_t WRP3 : 8;                  /*!< Flash memory write protection option bytes */
            __IO uint32_t nWRP3 : 8;                 /*!< Flash memory write protection complemented option bytes */
        } b;
        __IO uint16_t w;
    } WRP3;                                  /*!<FLASH option byte write protection 3,          Address offset: 0x12 */
} OB_TypeDef;


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
    __IO uint32_t BRR;                       /*!< GPIO bit reset register,               Address offset: 0x28 */
} GPIO_TypeDef;


/**
  * @brief Operational Amplifier (OPAMP)
  */


typedef struct {
    union {
        struct {
            __IO uint32_t EN : 1;                    /*!< OPAMP enable */
            __IO uint32_t FORCEVP : 1;               /*!< Connect the internal references to the plus input of the OPAMPX */
            __IO uint32_t VPSEL : 2;                 /*!< Non inverting input selection */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t VMSEL : 2;                 /*!< Inverting input selection */
            __IO uint32_t TCMEN : 1;                 /*!< Timer-Controlled Mux mode enable */
            __IO uint32_t VMSSEL : 1;                /*!< Inverting input secondary selection */
            __IO uint32_t VPSSEL : 2;                /*!< Non inverting input secondary selection */
            __IO uint32_t CALON : 1;                 /*!< Calibration mode enable */
            __IO uint32_t CALSEL : 2;                /*!< Calibration selection */
            __IO uint32_t PGGAIN : 4;                /*!< Gain in PGA mode */
            __IO uint32_t USERTRIM : 1;              /*!< User trimming enable */
            __IO uint32_t TRIMOFFSETP : 5;           /*!< Offset trimming value (PMOS) */
            __IO uint32_t TRIMOFFSETN : 5;           /*!< Offset trimming value (NMOS) */
            __IO uint32_t TSTREF : 1;                /*!< It enables the switch to put out the internal reference */
            __IO uint32_t OUTCAL : 1;                /*!< OPAMP ouput status flag */
            __IO uint32_t LOCK : 1;                  /*!< OPAMP lock */
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< OPAMP control and status register,            Address offset: 0x00 */
} OPAMP_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EN;                        /*!< OPAMP enable */
        __IO uint32_t FORCEVP;                   /*!< Connect the internal references to the plus input of the OPAMPX */
        __IO uint32_t VPSEL[2];                  /*!< Non inverting input selection */
             uint32_t __RESERVED0;
        __IO uint32_t VMSEL[2];                  /*!< Inverting input selection */
        __IO uint32_t TCMEN;                     /*!< Timer-Controlled Mux mode enable */
        __IO uint32_t VMSSEL;                    /*!< Inverting input secondary selection */
        __IO uint32_t VPSSEL[2];                 /*!< Non inverting input secondary selection */
        __IO uint32_t CALON;                     /*!< Calibration mode enable */
        __IO uint32_t CALSEL[2];                 /*!< Calibration selection */
        __IO uint32_t PGGAIN[4];                 /*!< Gain in PGA mode */
        __IO uint32_t USERTRIM;                  /*!< User trimming enable */
        __IO uint32_t TRIMOFFSETP[5];            /*!< Offset trimming value (PMOS) */
        __IO uint32_t TRIMOFFSETN[5];            /*!< Offset trimming value (NMOS) */
        __IO uint32_t TSTREF;                    /*!< It enables the switch to put out the internal reference */
        __IO uint32_t OUTCAL;                    /*!< OPAMP ouput status flag */
        __IO uint32_t LOCK;                      /*!< OPAMP lock */
    } CSR;                                   /*!< OPAMP control and status register,            Address offset: 0x00 */
} OPAMP_BitBand_TypeDef;



/**
  * @brief System configuration controller
  */


typedef struct {
    union {
        struct {
            __IO uint32_t MEM_MODE : 2;              /*!< SYSCFG_Memory Remap Config */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t USB_IT_RMP : 1;            /*!< USB interrupt remap */
            __IO uint32_t TIM1_ITR3_RMP : 1;         /*!< Timer 1 ITR3 selection */
            __IO uint32_t DAC1_TRIG1_RMP : 1;        /*!< DAC1 Trigger1 remap */
            __IO uint32_t ADC24_DMA_RMP : 1;         /*!< ADC2 and ADC4 DMA remap */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t TIM16_DMA_RMP : 1;         /*!< Timer 16 DMA remap */
            __IO uint32_t TIM17_DMA_RMP : 1;         /*!< Timer 17 DMA remap */
            __IO uint32_t TIM6DAC1Ch1_DMA_RMP : 1;   /*!< Timer 6 / DAC1 Ch1 DMA remap */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t I2C_PB6_FMP : 1;           /*!< I2C PB6 Fast mode plus */
            __IO uint32_t I2C_PB7_FMP : 1;           /*!< I2C PB7 Fast mode plus */
            __IO uint32_t I2C_PB8_FMP : 1;           /*!< I2C PB8 Fast mode plus */
            __IO uint32_t I2C_PB9_FMP : 1;           /*!< I2C PB9 Fast mode plus */
            __IO uint32_t I2C1_FMP : 1;              /*!< I2C1 Fast mode plus */
            __IO uint32_t I2C2_FMP : 1;              /*!< I2C2 Fast mode plus */
            __IO uint32_t ENCODER_MODE : 2;          /*!< Encoder Mode */
                 uint32_t __RESERVED3 : 2;
            __IO uint32_t FPU_IE : 6;                /*!< Floating Point Unit Interrupt Enable */
        } b;
        __IO uint32_t w;
    } CFGR1;                                 /*!< SYSCFG configuration register 1,                      Address offset: 0x00 */
         uint32_t __RESERVED0;               /*!< Reserved,                                                             0x04 */
    __IO uint32_t EXTICR[4];                    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x14-0x08 */
    union {
        struct {
            __IO uint32_t LOCKUP_LOCK : 1;           /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM4 with Break Input of TIMx */
            __IO uint32_t SRAM_PARITY_LOCK : 1;      /*!< Enables and locks the SRAM_PARITY error signal with Break Input of TIMx */
            __IO uint32_t PVD_LOCK : 1;              /*!< Enables and locks the PVD connection with TIMx Break Input, as well as the PVDE and PLS[2:0] in the PWR_CR register */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t BYP_ADDR_PAR : 1;          /*!< Disables the adddress parity check on RAM */
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t SRAM_PE : 1;               /*!< SRAM Parity error flag */
                 uint32_t __RESERVED2 : 23;
        } b;
        __IO uint32_t w;
    } CFGR2;                                 /*!< SYSCFG configuration register 2,                      Address offset: 0x18 */
} SYSCFG_TypeDef;


typedef struct {
    struct {
        __IO uint32_t MEM_MODE[2];               /*!< SYSCFG_Memory Remap Config */
             uint32_t __RESERVED0[3];
        __IO uint32_t USB_IT_RMP;                /*!< USB interrupt remap */
        __IO uint32_t TIM1_ITR3_RMP;             /*!< Timer 1 ITR3 selection */
        __IO uint32_t DAC1_TRIG1_RMP;            /*!< DAC1 Trigger1 remap */
        __IO uint32_t ADC24_DMA_RMP;             /*!< ADC2 and ADC4 DMA remap */
             uint32_t __RESERVED1[2];
        __IO uint32_t TIM16_DMA_RMP;             /*!< Timer 16 DMA remap */
        __IO uint32_t TIM17_DMA_RMP;             /*!< Timer 17 DMA remap */
        __IO uint32_t TIM6DAC1Ch1_DMA_RMP;       /*!< Timer 6 / DAC1 Ch1 DMA remap */
             uint32_t __RESERVED2[2];
        __IO uint32_t I2C_PB6_FMP;               /*!< I2C PB6 Fast mode plus */
        __IO uint32_t I2C_PB7_FMP;               /*!< I2C PB7 Fast mode plus */
        __IO uint32_t I2C_PB8_FMP;               /*!< I2C PB8 Fast mode plus */
        __IO uint32_t I2C_PB9_FMP;               /*!< I2C PB9 Fast mode plus */
        __IO uint32_t I2C1_FMP;                  /*!< I2C1 Fast mode plus */
        __IO uint32_t I2C2_FMP;                  /*!< I2C2 Fast mode plus */
        __IO uint32_t ENCODER_MODE[2];           /*!< Encoder Mode */
             uint32_t __RESERVED3[2];
        __IO uint32_t FPU_IE[6];                 /*!< Floating Point Unit Interrupt Enable */
    } CFGR1;                                 /*!< SYSCFG configuration register 1,                      Address offset: 0x00 */
         uint32_t __RESERVED0[32];           /*!< Reserved,                                                             0x04 */
    __IO uint32_t EXTICR[4][32];                /*!< SYSCFG external interrupt configuration registers, Address offset: 0x14-0x08 */
    struct {
        __IO uint32_t LOCKUP_LOCK;               /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM4 with Break Input of TIMx */
        __IO uint32_t SRAM_PARITY_LOCK;          /*!< Enables and locks the SRAM_PARITY error signal with Break Input of TIMx */
        __IO uint32_t PVD_LOCK;                  /*!< Enables and locks the PVD connection with TIMx Break Input, as well as the PVDE and PLS[2:0] in the PWR_CR register */
             uint32_t __RESERVED0;
        __IO uint32_t BYP_ADDR_PAR;              /*!< Disables the adddress parity check on RAM */
             uint32_t __RESERVED1[3];
        __IO uint32_t SRAM_PE;                   /*!< SRAM Parity error flag */
             uint32_t __RESERVED2[23];
    } CFGR2;                                 /*!< SYSCFG configuration register 2,                      Address offset: 0x18 */
} SYSCFG_BitBand_TypeDef;



/**
  * @brief Inter-integrated Circuit Interface
  */


typedef struct {
    union {
        struct {
            __IO uint32_t PE : 1;                    /*!< Peripheral enable */
            __IO uint32_t TXIE : 1;                  /*!< TX interrupt enable */
            __IO uint32_t RXIE : 1;                  /*!< RX interrupt enable */
            __IO uint32_t ADDRIE : 1;                /*!< Address match interrupt enable */
            __IO uint32_t NACKIE : 1;                /*!< NACK received interrupt enable */
            __IO uint32_t STOPIE : 1;                /*!< STOP detection interrupt enable */
            __IO uint32_t TCIE : 1;                  /*!< Transfer complete interrupt enable */
            __IO uint32_t ERRIE : 1;                 /*!< Errors interrupt enable */
            __IO uint32_t DNF : 4;                   /*!< Digital noise filter */
            __IO uint32_t ANFOFF : 1;                /*!< Analog noise filter OFF */
            __IO uint32_t SWRST : 1;                 /*!< Software reset */
            __IO uint32_t TXDMAEN : 1;               /*!< DMA transmission requests enable */
            __IO uint32_t RXDMAEN : 1;               /*!< DMA reception requests enable */
            __IO uint32_t SBC : 1;                   /*!< Slave byte control */
            __IO uint32_t NOSTRETCH : 1;             /*!< Clock stretching disable */
            __IO uint32_t WUPEN : 1;                 /*!< Wakeup from STOP enable */
            __IO uint32_t GCEN : 1;                  /*!< General call enable */
            __IO uint32_t SMBHEN : 1;                /*!< SMBus host address enable */
            __IO uint32_t SMBDEN : 1;                /*!< SMBus device default address enable */
            __IO uint32_t ALERTEN : 1;               /*!< SMBus alert enable */
            __IO uint32_t PECEN : 1;                 /*!< PEC enable */
                 uint32_t __RESERVED0 : 8;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< I2C Control register 1,            Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t SADD : 10;                 /*!< Slave address (master mode) */
            __IO uint32_t RD_WRN : 1;                /*!< Transfer direction (master mode) */
            __IO uint32_t ADD10 : 1;                 /*!< 10-bit addressing mode (master mode) */
            __IO uint32_t HEAD10R : 1;               /*!< 10-bit address header only read direction (master mode) */
            __IO uint32_t START : 1;                 /*!< START generation */
            __IO uint32_t STOP : 1;                  /*!< STOP generation (master mode) */
            __IO uint32_t NACK : 1;                  /*!< NACK generation (slave mode) */
            __IO uint32_t NBYTES : 8;                /*!< Number of bytes */
            __IO uint32_t RELOAD : 1;                /*!< NBYTES reload mode */
            __IO uint32_t AUTOEND : 1;               /*!< Automatic end mode (master mode) */
            __IO uint32_t PECBYTE : 1;               /*!< Packet error checking byte */
                 uint32_t __RESERVED0 : 5;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< I2C Control register 2,            Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t OA1 : 10;                  /*!< Interface own address 1 */
            __IO uint32_t OA1MODE : 1;               /*!< Own address 1 10-bit mode */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t OA1EN : 1;                 /*!< Own address 1 enable */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } OAR1;                                  /*!< I2C Own address 1 register,        Address offset: 0x08 */
    union {
        struct {
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t OA2 : 7;                   /*!< Interface own address 2                        */
            __IO uint32_t OA2MSK : 3;                /*!< Own address 2 masks                            */
                 uint32_t __RESERVED1 : 4;
            __IO uint32_t OA2EN : 1;                 /*!< Own address 2 enable                           */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } OAR2;                                  /*!< I2C Own address 2 register,        Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t SCLL : 8;                  /*!< SCL low period (master mode) */
            __IO uint32_t SCLH : 8;                  /*!< SCL high period (master mode) */
            __IO uint32_t SDADEL : 4;                /*!< Data hold time */
            __IO uint32_t SCLDEL : 4;                /*!< Data setup time */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t PRESC : 4;                 /*!< Timings prescaler */
        } b;
        __IO uint32_t w;
    } TIMINGR;                               /*!< I2C Timing register,               Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t TIMEOUTA : 12;             /*!< Bus timeout A */
            __IO uint32_t TIDLE : 1;                 /*!< Idle clock timeout detection */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t TIMOUTEN : 1;              /*!< Clock timeout enable */
            __IO uint32_t TIMEOUTB : 12;             /*!< Bus timeout B*/
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t TEXTEN : 1;                /*!< Extended clock timeout enable */
        } b;
        __IO uint32_t w;
    } TIMEOUTR;                              /*!< I2C Timeout register,              Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t TXE : 1;                   /*!< Transmit data register empty */
            __IO uint32_t TXIS : 1;                  /*!< Transmit interrupt status */
            __IO uint32_t RXNE : 1;                  /*!< Receive data register not empty */
            __IO uint32_t ADDR : 1;                  /*!< Address matched (slave mode)*/
            __IO uint32_t NACKF : 1;                 /*!< NACK received flag */
            __IO uint32_t STOPF : 1;                 /*!< STOP detection flag */
            __IO uint32_t TC : 1;                    /*!< Transfer complete (master mode) */
            __IO uint32_t TCR : 1;                   /*!< Transfer complete reload */
            __IO uint32_t BERR : 1;                  /*!< Bus error */
            __IO uint32_t ARLO : 1;                  /*!< Arbitration lost */
            __IO uint32_t OVR : 1;                   /*!< Overrun/Underrun */
            __IO uint32_t PECERR : 1;                /*!< PEC error in reception */
            __IO uint32_t TIMEOUT : 1;               /*!< Timeout or Tlow detection flag */
            __IO uint32_t ALERT : 1;                 /*!< SMBus alert */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t BUSY : 1;                  /*!< Bus busy */
            __IO uint32_t DIR : 1;                   /*!< Transfer direction (slave mode) */
            __IO uint32_t ADDCODE : 7;               /*!< Address match code (slave mode) */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } ISR;                                   /*!< I2C Interrupt and status register, Address offset: 0x18 */
    union {
        struct {
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t ADDRCF : 1;                /*!< Address matched clear flag */
            __IO uint32_t NACKCF : 1;                /*!< NACK clear flag */
            __IO uint32_t STOPCF : 1;                /*!< STOP detection clear flag */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t BERRCF : 1;                /*!< Bus error clear flag */
            __IO uint32_t ARLOCF : 1;                /*!< Arbitration lost clear flag */
            __IO uint32_t OVRCF : 1;                 /*!< Overrun/Underrun clear flag */
            __IO uint32_t PECCF : 1;                 /*!< PAC error clear flag */
            __IO uint32_t TIMOUTCF : 1;              /*!< Timeout clear flag */
            __IO uint32_t ALERTCF : 1;               /*!< Alert clear flag */
                 uint32_t __RESERVED2 : 18;
        } b;
        __IO uint32_t w;
    } ICR;                                   /*!< I2C Interrupt clear register,      Address offset: 0x1C */
    __IO uint32_t PECR;                      /*!< I2C PEC register,                  Address offset: 0x20 */
    __IO uint32_t RXDR;                      /*!< I2C Receive data register,         Address offset: 0x24 */
    __IO uint32_t TXDR;                      /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;


typedef struct {
    struct {
        __IO uint32_t PE;                        /*!< Peripheral enable */
        __IO uint32_t TXIE;                      /*!< TX interrupt enable */
        __IO uint32_t RXIE;                      /*!< RX interrupt enable */
        __IO uint32_t ADDRIE;                    /*!< Address match interrupt enable */
        __IO uint32_t NACKIE;                    /*!< NACK received interrupt enable */
        __IO uint32_t STOPIE;                    /*!< STOP detection interrupt enable */
        __IO uint32_t TCIE;                      /*!< Transfer complete interrupt enable */
        __IO uint32_t ERRIE;                     /*!< Errors interrupt enable */
        __IO uint32_t DNF[4];                    /*!< Digital noise filter */
        __IO uint32_t ANFOFF;                    /*!< Analog noise filter OFF */
        __IO uint32_t SWRST;                     /*!< Software reset */
        __IO uint32_t TXDMAEN;                   /*!< DMA transmission requests enable */
        __IO uint32_t RXDMAEN;                   /*!< DMA reception requests enable */
        __IO uint32_t SBC;                       /*!< Slave byte control */
        __IO uint32_t NOSTRETCH;                 /*!< Clock stretching disable */
        __IO uint32_t WUPEN;                     /*!< Wakeup from STOP enable */
        __IO uint32_t GCEN;                      /*!< General call enable */
        __IO uint32_t SMBHEN;                    /*!< SMBus host address enable */
        __IO uint32_t SMBDEN;                    /*!< SMBus device default address enable */
        __IO uint32_t ALERTEN;                   /*!< SMBus alert enable */
        __IO uint32_t PECEN;                     /*!< PEC enable */
             uint32_t __RESERVED0[8];
    } CR1;                                   /*!< I2C Control register 1,            Address offset: 0x00 */
    struct {
        __IO uint32_t SADD[10];                  /*!< Slave address (master mode) */
        __IO uint32_t RD_WRN;                    /*!< Transfer direction (master mode) */
        __IO uint32_t ADD10;                     /*!< 10-bit addressing mode (master mode) */
        __IO uint32_t HEAD10R;                   /*!< 10-bit address header only read direction (master mode) */
        __IO uint32_t START;                     /*!< START generation */
        __IO uint32_t STOP;                      /*!< STOP generation (master mode) */
        __IO uint32_t NACK;                      /*!< NACK generation (slave mode) */
        __IO uint32_t NBYTES[8];                 /*!< Number of bytes */
        __IO uint32_t RELOAD;                    /*!< NBYTES reload mode */
        __IO uint32_t AUTOEND;                   /*!< Automatic end mode (master mode) */
        __IO uint32_t PECBYTE;                   /*!< Packet error checking byte */
             uint32_t __RESERVED0[5];
    } CR2;                                   /*!< I2C Control register 2,            Address offset: 0x04 */
    struct {
        __IO uint32_t OA1[10];                   /*!< Interface own address 1 */
        __IO uint32_t OA1MODE;                   /*!< Own address 1 10-bit mode */
             uint32_t __RESERVED0[4];
        __IO uint32_t OA1EN;                     /*!< Own address 1 enable */
             uint32_t __RESERVED1[16];
    } OAR1;                                  /*!< I2C Own address 1 register,        Address offset: 0x08 */
    struct {
             uint32_t __RESERVED0;
        __IO uint32_t OA2[7];                    /*!< Interface own address 2                        */
        __IO uint32_t OA2MSK[3];                 /*!< Own address 2 masks                            */
             uint32_t __RESERVED1[4];
        __IO uint32_t OA2EN;                     /*!< Own address 2 enable                           */
             uint32_t __RESERVED2[16];
    } OAR2;                                  /*!< I2C Own address 2 register,        Address offset: 0x0C */
    struct {
        __IO uint32_t SCLL[8];                   /*!< SCL low period (master mode) */
        __IO uint32_t SCLH[8];                   /*!< SCL high period (master mode) */
        __IO uint32_t SDADEL[4];                 /*!< Data hold time */
        __IO uint32_t SCLDEL[4];                 /*!< Data setup time */
             uint32_t __RESERVED0[4];
        __IO uint32_t PRESC[4];                  /*!< Timings prescaler */
    } TIMINGR;                               /*!< I2C Timing register,               Address offset: 0x10 */
    struct {
        __IO uint32_t TIMEOUTA[12];              /*!< Bus timeout A */
        __IO uint32_t TIDLE;                     /*!< Idle clock timeout detection */
             uint32_t __RESERVED0[2];
        __IO uint32_t TIMOUTEN;                  /*!< Clock timeout enable */
        __IO uint32_t TIMEOUTB[12];              /*!< Bus timeout B*/
             uint32_t __RESERVED1[3];
        __IO uint32_t TEXTEN;                    /*!< Extended clock timeout enable */
    } TIMEOUTR;                              /*!< I2C Timeout register,              Address offset: 0x14 */
    struct {
        __IO uint32_t TXE;                       /*!< Transmit data register empty */
        __IO uint32_t TXIS;                      /*!< Transmit interrupt status */
        __IO uint32_t RXNE;                      /*!< Receive data register not empty */
        __IO uint32_t ADDR;                      /*!< Address matched (slave mode)*/
        __IO uint32_t NACKF;                     /*!< NACK received flag */
        __IO uint32_t STOPF;                     /*!< STOP detection flag */
        __IO uint32_t TC;                        /*!< Transfer complete (master mode) */
        __IO uint32_t TCR;                       /*!< Transfer complete reload */
        __IO uint32_t BERR;                      /*!< Bus error */
        __IO uint32_t ARLO;                      /*!< Arbitration lost */
        __IO uint32_t OVR;                       /*!< Overrun/Underrun */
        __IO uint32_t PECERR;                    /*!< PEC error in reception */
        __IO uint32_t TIMEOUT;                   /*!< Timeout or Tlow detection flag */
        __IO uint32_t ALERT;                     /*!< SMBus alert */
             uint32_t __RESERVED0;
        __IO uint32_t BUSY;                      /*!< Bus busy */
        __IO uint32_t DIR;                       /*!< Transfer direction (slave mode) */
        __IO uint32_t ADDCODE[7];                /*!< Address match code (slave mode) */
             uint32_t __RESERVED1[8];
    } ISR;                                   /*!< I2C Interrupt and status register, Address offset: 0x18 */
    struct {
             uint32_t __RESERVED0[3];
        __IO uint32_t ADDRCF;                    /*!< Address matched clear flag */
        __IO uint32_t NACKCF;                    /*!< NACK clear flag */
        __IO uint32_t STOPCF;                    /*!< STOP detection clear flag */
             uint32_t __RESERVED1[2];
        __IO uint32_t BERRCF;                    /*!< Bus error clear flag */
        __IO uint32_t ARLOCF;                    /*!< Arbitration lost clear flag */
        __IO uint32_t OVRCF;                     /*!< Overrun/Underrun clear flag */
        __IO uint32_t PECCF;                     /*!< PAC error clear flag */
        __IO uint32_t TIMOUTCF;                  /*!< Timeout clear flag */
        __IO uint32_t ALERTCF;                   /*!< Alert clear flag */
             uint32_t __RESERVED2[18];
    } ICR;                                   /*!< I2C Interrupt clear register,      Address offset: 0x1C */
    __IO uint32_t PECR[32];                  /*!< I2C PEC register,                  Address offset: 0x20 */
    __IO uint32_t RXDR[32];                  /*!< I2C Receive data register,         Address offset: 0x24 */
    __IO uint32_t TXDR[32];                  /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_BitBand_TypeDef;



/**
  * @brief Independent WATCHDOG
  */


typedef struct {
    __IO uint32_t KR;                        /*!< IWDG Key register,       Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t PR : 3;                    /*!< PR[2:0] (Prescaler divider) */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } PR;                                    /*!< IWDG Prescaler register, Address offset: 0x04 */
    __IO uint32_t RLR;                       /*!< IWDG Reload register,    Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t PVU : 1;                   /*!< Watchdog prescaler value update */
            __IO uint32_t RVU : 1;                   /*!< Watchdog counter reload value update */
            __IO uint32_t WVU : 1;                   /*!< Watchdog counter window value update */
                 uint32_t __RESERVED0 : 29;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< IWDG Status register,    Address offset: 0x0C */
    __IO uint32_t WINR;                      /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;


typedef struct {
    __IO uint32_t KR[32];                    /*!< IWDG Key register,       Address offset: 0x00 */
    struct {
        __IO uint32_t PR[3];                     /*!< PR[2:0] (Prescaler divider) */
             uint32_t __RESERVED0[29];
    } PR;                                    /*!< IWDG Prescaler register, Address offset: 0x04 */
    __IO uint32_t RLR[32];                   /*!< IWDG Reload register,    Address offset: 0x08 */
    struct {
        __IO uint32_t PVU;                       /*!< Watchdog prescaler value update */
        __IO uint32_t RVU;                       /*!< Watchdog counter reload value update */
        __IO uint32_t WVU;                       /*!< Watchdog counter window value update */
             uint32_t __RESERVED0[29];
    } SR;                                    /*!< IWDG Status register,    Address offset: 0x0C */
    __IO uint32_t WINR[32];                  /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_BitBand_TypeDef;



/**
  * @brief Power Control
  */


typedef struct {
    union {
        struct {
            __IO uint32_t LPDS : 1;                  /*!< Low-power Deepsleep */
            __IO uint32_t PDDS : 1;                  /*!< Power Down Deepsleep */
            __IO uint32_t CWUF : 1;                  /*!< Clear Wakeup Flag */
            __IO uint32_t CSBF : 1;                  /*!< Clear Standby Flag */
            __IO uint32_t PVDE : 1;                  /*!< Power Voltage Detector Enable */
            __IO uint32_t PLS : 3;                   /*!< PLS[2:0] bits (PVD Level Selection) */
            __IO uint32_t DBP : 1;                   /*!< Disable Backup Domain write protection */
                 uint32_t __RESERVED0 : 23;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< PWR power control register,        Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t WUF : 1;                   /*!< Wakeup Flag */
            __IO uint32_t SBF : 1;                   /*!< Standby Flag */
            __IO uint32_t PVDO : 1;                  /*!< PVD Output */
            __IO uint32_t VREFINTRDYF : 1;           /*!< Internal voltage reference (VREFINT) ready flag */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t EWUP1 : 1;                 /*!< Enable WKUP pin 1 */
            __IO uint32_t EWUP2 : 1;                 /*!< Enable WKUP pin 2 */
            __IO uint32_t EWUP3 : 1;                 /*!< Enable WKUP pin 3 */
                 uint32_t __RESERVED1 : 21;
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;


typedef struct {
    struct {
        __IO uint32_t LPDS;                      /*!< Low-power Deepsleep */
        __IO uint32_t PDDS;                      /*!< Power Down Deepsleep */
        __IO uint32_t CWUF;                      /*!< Clear Wakeup Flag */
        __IO uint32_t CSBF;                      /*!< Clear Standby Flag */
        __IO uint32_t PVDE;                      /*!< Power Voltage Detector Enable */
        __IO uint32_t PLS[3];                    /*!< PLS[2:0] bits (PVD Level Selection) */
        __IO uint32_t DBP;                       /*!< Disable Backup Domain write protection */
             uint32_t __RESERVED0[23];
    } CR;                                    /*!< PWR power control register,        Address offset: 0x00 */
    struct {
        __IO uint32_t WUF;                       /*!< Wakeup Flag */
        __IO uint32_t SBF;                       /*!< Standby Flag */
        __IO uint32_t PVDO;                      /*!< PVD Output */
        __IO uint32_t VREFINTRDYF;               /*!< Internal voltage reference (VREFINT) ready flag */
             uint32_t __RESERVED0[4];
        __IO uint32_t EWUP1;                     /*!< Enable WKUP pin 1 */
        __IO uint32_t EWUP2;                     /*!< Enable WKUP pin 2 */
        __IO uint32_t EWUP3;                     /*!< Enable WKUP pin 3 */
             uint32_t __RESERVED1[21];
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
                 uint32_t __RESERVED2 : 6;
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< RCC clock control register,                                  Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t SW : 2;                    /*!< SW[1:0] bits (System clock Switch) */
            __IO uint32_t SWS : 2;                   /*!< SWS[1:0] bits (System Clock Switch Status) */
            __IO uint32_t HPRE : 4;                  /*!< HPRE[3:0] bits (AHB prescaler) */
            __IO uint32_t PPRE1 : 3;                 /*!< PRE1[2:0] bits (APB1 prescaler) */
            __IO uint32_t PPRE2 : 3;                 /*!< PRE2[2:0] bits (APB2 prescaler) */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t PLLSRC : 1;                /*!< PLL entry clock source */
            __IO uint32_t PLLXTPRE : 1;              /*!< HSE divider for PLL entry */
            __IO uint32_t PLLMUL : 4;                /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
            __IO uint32_t USBPRE : 1;                /*!< USB prescaler */
            __IO uint32_t I2SSRC : 1;                /*!< I2S external clock source selection */
            __IO uint32_t MCO : 3;                   /*!< MCO[2:0] bits (Microcontroller Clock Output) */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t MCOF : 1;                  /*!< Microcontroller Clock Output Flag */
                 uint32_t __RESERVED2 : 3;
        } b;
        __IO uint32_t w;
    } CFGR;                                  /*!< RCC clock configuration register,                            Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t LSIRDYF : 1;               /*!< LSI Ready Interrupt flag */
            __IO uint32_t LSERDYF : 1;               /*!< LSE Ready Interrupt flag */
            __IO uint32_t HSIRDYF : 1;               /*!< HSI Ready Interrupt flag */
            __IO uint32_t HSERDYF : 1;               /*!< HSE Ready Interrupt flag */
            __IO uint32_t PLLRDYF : 1;               /*!< PLL Ready Interrupt flag */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t CSSF : 1;                  /*!< Clock Security System Interrupt flag */
            __IO uint32_t LSIRDYIE : 1;              /*!< LSI Ready Interrupt Enable */
            __IO uint32_t LSERDYIE : 1;              /*!< LSE Ready Interrupt Enable */
            __IO uint32_t HSIRDYIE : 1;              /*!< HSI Ready Interrupt Enable */
            __IO uint32_t HSERDYIE : 1;              /*!< HSE Ready Interrupt Enable */
            __IO uint32_t PLLRDYIE : 1;              /*!< PLL Ready Interrupt Enable */
                 uint32_t __RESERVED1 : 3;
            __IO uint32_t LSIRDYC : 1;               /*!< LSI Ready Interrupt Clear */
            __IO uint32_t LSERDYC : 1;               /*!< LSE Ready Interrupt Clear */
            __IO uint32_t HSIRDYC : 1;               /*!< HSI Ready Interrupt Clear */
            __IO uint32_t HSERDYC : 1;               /*!< HSE Ready Interrupt Clear */
            __IO uint32_t PLLRDYC : 1;               /*!< PLL Ready Interrupt Clear */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t CSSC : 1;                  /*!< Clock Security System Interrupt Clear */
                 uint32_t __RESERVED3 : 8;
        } b;
        __IO uint32_t w;
    } CIR;                                   /*!< RCC clock interrupt register,                                Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t SYSCFGRST : 1;             /*!< SYSCFG reset */
                 uint32_t __RESERVED0 : 10;
            __IO uint32_t TIM1RST : 1;               /*!< TIM1 reset */
            __IO uint32_t SPI1RST : 1;               /*!< SPI1 reset */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t USART1RST : 1;             /*!< USART1 reset */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t TIM15RST : 1;              /*!< TIM15 reset */
            __IO uint32_t TIM16RST : 1;              /*!< TIM16 reset */
            __IO uint32_t TIM17RST : 1;              /*!< TIM17 reset */
                 uint32_t __RESERVED3 : 13;
        } b;
        __IO uint32_t w;
    } APB2RSTR;                              /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t TIM2RST : 1;               /*!< Timer 2 reset */
            __IO uint32_t TIM3RST : 1;               /*!< Timer 3 reset */
            __IO uint32_t TIM4RST : 1;               /*!< Timer 4 reset */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TIM6RST : 1;               /*!< Timer 6 reset */
                 uint32_t __RESERVED1 : 6;
            __IO uint32_t WWDGRST : 1;               /*!< Window Watchdog reset */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SPI2RST : 1;               /*!< SPI2 reset */
            __IO uint32_t SPI3RST : 1;               /*!< SPI3 reset */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t USART2RST : 1;             /*!< USART 2 reset */
            __IO uint32_t USART3RST : 1;             /*!< USART 3 reset */
            __IO uint32_t UART4RST : 1;              /*!< UART 4 reset */
            __IO uint32_t UART5RST : 1;              /*!< UART 5 reset */
            __IO uint32_t I2C1RST : 1;               /*!< I2C 1 reset */
            __IO uint32_t I2C2RST : 1;               /*!< I2C 2 reset */
            __IO uint32_t USBRST : 1;                /*!< USB reset */
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t CANRST : 1;                /*!< CAN reset */
                 uint32_t __RESERVED5 : 2;
            __IO uint32_t PWRRST : 1;                /*!< PWR reset */
            __IO uint32_t DAC1RST : 1;               /*!< DAC 1 reset */
                 uint32_t __RESERVED6 : 2;
        } b;
        __IO uint32_t w;
    } APB1RSTR;                              /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t DMA1EN : 1;                /*!< DMA1 clock enable */
            __IO uint32_t DMA2EN : 1;                /*!< DMA2 clock enable */
            __IO uint32_t SRAMEN : 1;                /*!< SRAM interface clock enable */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t FLITFEN : 1;               /*!< FLITF clock enable */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t CRCEN : 1;                 /*!< CRC clock enable */
                 uint32_t __RESERVED2 : 10;
            __IO uint32_t GPIOAEN : 1;               /*!< GPIOA clock enable */
            __IO uint32_t GPIOBEN : 1;               /*!< GPIOB clock enable */
            __IO uint32_t GPIOCEN : 1;               /*!< GPIOC clock enable */
            __IO uint32_t GPIODEN : 1;               /*!< GPIOD clock enable */
            __IO uint32_t GPIOEEN : 1;               /*!< GPIOE clock enable */
            __IO uint32_t GPIOFEN : 1;               /*!< GPIOF clock enable */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t TSCEN : 1;                 /*!< TS clock enable */
                 uint32_t __RESERVED4 : 3;
            __IO uint32_t ADC12EN : 1;               /*!< ADC1/ ADC2 clock enable */
                 uint32_t __RESERVED5 : 3;
        } b;
        __IO uint32_t w;
    } AHBENR;                                /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t SYSCFGEN : 1;              /*!< SYSCFG clock enable */
                 uint32_t __RESERVED0 : 10;
            __IO uint32_t TIM1EN : 1;                /*!< TIM1 clock enable */
            __IO uint32_t SPI1EN : 1;                /*!< SPI1 clock enable */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t USART1EN : 1;              /*!< USART1 clock enable */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t TIM15EN : 1;               /*!< TIM15 clock enable */
            __IO uint32_t TIM16EN : 1;               /*!< TIM16 clock enable */
            __IO uint32_t TIM17EN : 1;               /*!< TIM17 clock enable */
                 uint32_t __RESERVED3 : 13;
        } b;
        __IO uint32_t w;
    } APB2ENR;                               /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t TIM2EN : 1;                /*!< Timer 2 clock enable */
            __IO uint32_t TIM3EN : 1;                /*!< Timer 3 clock enable */
            __IO uint32_t TIM4EN : 1;                /*!< Timer 4 clock enable */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TIM6EN : 1;                /*!< Timer 6 clock enable */
                 uint32_t __RESERVED1 : 6;
            __IO uint32_t WWDGEN : 1;                /*!< Window Watchdog clock enable */
                 uint32_t __RESERVED2 : 2;
            __IO uint32_t SPI2EN : 1;                /*!< SPI2 clock enable */
            __IO uint32_t SPI3EN : 1;                /*!< SPI3 clock enable */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t USART2EN : 1;              /*!< USART 2 clock enable */
            __IO uint32_t USART3EN : 1;              /*!< USART 3 clock enable */
            __IO uint32_t UART4EN : 1;               /*!< UART 4 clock enable */
            __IO uint32_t UART5EN : 1;               /*!< UART 5 clock enable */
            __IO uint32_t I2C1EN : 1;                /*!< I2C 1 clock enable */
            __IO uint32_t I2C2EN : 1;                /*!< I2C 2 clock enable */
            __IO uint32_t USBEN : 1;                 /*!< USB clock enable */
                 uint32_t __RESERVED4 : 1;
            __IO uint32_t CANEN : 1;                 /*!< CAN clock enable */
                 uint32_t __RESERVED5 : 2;
            __IO uint32_t PWREN : 1;                 /*!< PWR clock enable */
            __IO uint32_t DAC1EN : 1;                /*!< DAC 1 clock enable */
                 uint32_t __RESERVED6 : 2;
        } b;
        __IO uint32_t w;
    } APB1ENR;                               /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t LSEON : 1;                 /*!< External Low Speed oscillator enable */
            __IO uint32_t LSERDY : 1;                /*!< External Low Speed oscillator Ready */
            __IO uint32_t LSEBYP : 1;                /*!< External Low Speed oscillator Bypass */
            __IO uint32_t LSEDRV : 2;                /*!< LSEDRV[1:0] bits (LSE Osc. drive capability) */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t RTCSEL : 2;                /*!< RTCSEL[1:0] bits (RTC clock source selection) */
                 uint32_t __RESERVED1 : 5;
            __IO uint32_t RTCEN : 1;                 /*!< RTC clock enable */
            __IO uint32_t BDRST : 1;                 /*!< Backup domain software reset  */
                 uint32_t __RESERVED2 : 15;
        } b;
        __IO uint32_t w;
    } BDCR;                                  /*!< RCC Backup domain control register,                          Address offset: 0x20 */
    union {
        struct {
            __IO uint32_t LSION : 1;                 /*!< Internal Low Speed oscillator enable */
            __IO uint32_t LSIRDY : 1;                /*!< Internal Low Speed oscillator Ready */
                 uint32_t __RESERVED0 : 21;
            __IO uint32_t V18PWRRSTF : 1;            /*!< V1.8 power domain reset flag */
            __IO uint32_t RMVF : 1;                  /*!< Remove reset flag */
            __IO uint32_t OBLRSTF : 1;               /*!< OBL reset flag */
            __IO uint32_t PINRSTF : 1;               /*!< PIN reset flag */
            __IO uint32_t PORRSTF : 1;               /*!< POR/PDR reset flag */
            __IO uint32_t SFTRSTF : 1;               /*!< Software Reset flag */
            __IO uint32_t IWDGRSTF : 1;              /*!< Independent Watchdog reset flag */
            __IO uint32_t WWDGRSTF : 1;              /*!< Window watchdog reset flag */
            __IO uint32_t LPWRRSTF : 1;              /*!< Low-Power reset flag */
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< RCC clock control & status register,                         Address offset: 0x24 */
    union {
        struct {
                 uint32_t __RESERVED0 : 17;
            __IO uint32_t GPIOARST : 1;              /*!< GPIOA reset */
            __IO uint32_t GPIOBRST : 1;              /*!< GPIOB reset */
            __IO uint32_t GPIOCRST : 1;              /*!< GPIOC reset */
            __IO uint32_t GPIODRST : 1;              /*!< GPIOD reset */
            __IO uint32_t GPIOERST : 1;              /*!< GPIOE reset */
            __IO uint32_t GPIOFRST : 1;              /*!< GPIOF reset */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t TSCRST : 1;                /*!< TSC reset */
                 uint32_t __RESERVED2 : 3;
            __IO uint32_t ADC12RST : 1;              /*!< ADC1 & ADC2 reset */
                 uint32_t __RESERVED3 : 3;
        } b;
        __IO uint32_t w;
    } AHBRSTR;                               /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
    union {
        struct {
            __IO uint32_t PREDIV : 4;                /*!< PREDIV[3:0] bits */
            __IO uint32_t ADCPRE12 : 5;              /*!< ADCPRE12[8:4] bits */
                 uint32_t __RESERVED0 : 23;
        } b;
        __IO uint32_t w;
    } CFGR2;                                 /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
    union {
        struct {
            __IO uint32_t USART1SW : 2;              /*!< USART1SW[1:0] bits */
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t I2C1SW : 1;                /*!< I2C1SW bits */
            __IO uint32_t I2C2SW : 1;                /*!< I2C2SW bits */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t TIM1SW : 1;                /*!< TIM1SW bits */
                 uint32_t __RESERVED2 : 7;
            __IO uint32_t USART2SW : 2;              /*!< USART2SW[1:0] bits */
            __IO uint32_t USART3SW : 2;              /*!< USART3SW[1:0] bits */
            __IO uint32_t UART4SW : 2;               /*!< UART4SW[1:0] bits */
            __IO uint32_t UART5SW : 2;               /*!< UART5SW[1:0] bits */
                 uint32_t __RESERVED3 : 8;
        } b;
        __IO uint32_t w;
    } CFGR3;                                 /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
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
    } CR;                                    /*!< RCC clock control register,                                  Address offset: 0x00 */
    struct {
        __IO uint32_t SW[2];                     /*!< SW[1:0] bits (System clock Switch) */
        __IO uint32_t SWS[2];                    /*!< SWS[1:0] bits (System Clock Switch Status) */
        __IO uint32_t HPRE[4];                   /*!< HPRE[3:0] bits (AHB prescaler) */
        __IO uint32_t PPRE1[3];                  /*!< PRE1[2:0] bits (APB1 prescaler) */
        __IO uint32_t PPRE2[3];                  /*!< PRE2[2:0] bits (APB2 prescaler) */
             uint32_t __RESERVED0[2];
        __IO uint32_t PLLSRC;                    /*!< PLL entry clock source */
        __IO uint32_t PLLXTPRE;                  /*!< HSE divider for PLL entry */
        __IO uint32_t PLLMUL[4];                 /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
        __IO uint32_t USBPRE;                    /*!< USB prescaler */
        __IO uint32_t I2SSRC;                    /*!< I2S external clock source selection */
        __IO uint32_t MCO[3];                    /*!< MCO[2:0] bits (Microcontroller Clock Output) */
             uint32_t __RESERVED1;
        __IO uint32_t MCOF;                      /*!< Microcontroller Clock Output Flag */
             uint32_t __RESERVED2[3];
    } CFGR;                                  /*!< RCC clock configuration register,                            Address offset: 0x04 */
    struct {
        __IO uint32_t LSIRDYF;                   /*!< LSI Ready Interrupt flag */
        __IO uint32_t LSERDYF;                   /*!< LSE Ready Interrupt flag */
        __IO uint32_t HSIRDYF;                   /*!< HSI Ready Interrupt flag */
        __IO uint32_t HSERDYF;                   /*!< HSE Ready Interrupt flag */
        __IO uint32_t PLLRDYF;                   /*!< PLL Ready Interrupt flag */
             uint32_t __RESERVED0[2];
        __IO uint32_t CSSF;                      /*!< Clock Security System Interrupt flag */
        __IO uint32_t LSIRDYIE;                  /*!< LSI Ready Interrupt Enable */
        __IO uint32_t LSERDYIE;                  /*!< LSE Ready Interrupt Enable */
        __IO uint32_t HSIRDYIE;                  /*!< HSI Ready Interrupt Enable */
        __IO uint32_t HSERDYIE;                  /*!< HSE Ready Interrupt Enable */
        __IO uint32_t PLLRDYIE;                  /*!< PLL Ready Interrupt Enable */
             uint32_t __RESERVED1[3];
        __IO uint32_t LSIRDYC;                   /*!< LSI Ready Interrupt Clear */
        __IO uint32_t LSERDYC;                   /*!< LSE Ready Interrupt Clear */
        __IO uint32_t HSIRDYC;                   /*!< HSI Ready Interrupt Clear */
        __IO uint32_t HSERDYC;                   /*!< HSE Ready Interrupt Clear */
        __IO uint32_t PLLRDYC;                   /*!< PLL Ready Interrupt Clear */
             uint32_t __RESERVED2[2];
        __IO uint32_t CSSC;                      /*!< Clock Security System Interrupt Clear */
             uint32_t __RESERVED3[8];
    } CIR;                                   /*!< RCC clock interrupt register,                                Address offset: 0x08 */
    struct {
        __IO uint32_t SYSCFGRST;                 /*!< SYSCFG reset */
             uint32_t __RESERVED0[10];
        __IO uint32_t TIM1RST;                   /*!< TIM1 reset */
        __IO uint32_t SPI1RST;                   /*!< SPI1 reset */
             uint32_t __RESERVED1;
        __IO uint32_t USART1RST;                 /*!< USART1 reset */
             uint32_t __RESERVED2;
        __IO uint32_t TIM15RST;                  /*!< TIM15 reset */
        __IO uint32_t TIM16RST;                  /*!< TIM16 reset */
        __IO uint32_t TIM17RST;                  /*!< TIM17 reset */
             uint32_t __RESERVED3[13];
    } APB2RSTR;                              /*!< RCC APB2 peripheral reset register,                          Address offset: 0x0C */
    struct {
        __IO uint32_t TIM2RST;                   /*!< Timer 2 reset */
        __IO uint32_t TIM3RST;                   /*!< Timer 3 reset */
        __IO uint32_t TIM4RST;                   /*!< Timer 4 reset */
             uint32_t __RESERVED0;
        __IO uint32_t TIM6RST;                   /*!< Timer 6 reset */
             uint32_t __RESERVED1[6];
        __IO uint32_t WWDGRST;                   /*!< Window Watchdog reset */
             uint32_t __RESERVED2[2];
        __IO uint32_t SPI2RST;                   /*!< SPI2 reset */
        __IO uint32_t SPI3RST;                   /*!< SPI3 reset */
             uint32_t __RESERVED3;
        __IO uint32_t USART2RST;                 /*!< USART 2 reset */
        __IO uint32_t USART3RST;                 /*!< USART 3 reset */
        __IO uint32_t UART4RST;                  /*!< UART 4 reset */
        __IO uint32_t UART5RST;                  /*!< UART 5 reset */
        __IO uint32_t I2C1RST;                   /*!< I2C 1 reset */
        __IO uint32_t I2C2RST;                   /*!< I2C 2 reset */
        __IO uint32_t USBRST;                    /*!< USB reset */
             uint32_t __RESERVED4;
        __IO uint32_t CANRST;                    /*!< CAN reset */
             uint32_t __RESERVED5[2];
        __IO uint32_t PWRRST;                    /*!< PWR reset */
        __IO uint32_t DAC1RST;                   /*!< DAC 1 reset */
             uint32_t __RESERVED6[2];
    } APB1RSTR;                              /*!< RCC APB1 peripheral reset register,                          Address offset: 0x10 */
    struct {
        __IO uint32_t DMA1EN;                    /*!< DMA1 clock enable */
        __IO uint32_t DMA2EN;                    /*!< DMA2 clock enable */
        __IO uint32_t SRAMEN;                    /*!< SRAM interface clock enable */
             uint32_t __RESERVED0;
        __IO uint32_t FLITFEN;                   /*!< FLITF clock enable */
             uint32_t __RESERVED1;
        __IO uint32_t CRCEN;                     /*!< CRC clock enable */
             uint32_t __RESERVED2[10];
        __IO uint32_t GPIOAEN;                   /*!< GPIOA clock enable */
        __IO uint32_t GPIOBEN;                   /*!< GPIOB clock enable */
        __IO uint32_t GPIOCEN;                   /*!< GPIOC clock enable */
        __IO uint32_t GPIODEN;                   /*!< GPIOD clock enable */
        __IO uint32_t GPIOEEN;                   /*!< GPIOE clock enable */
        __IO uint32_t GPIOFEN;                   /*!< GPIOF clock enable */
             uint32_t __RESERVED3;
        __IO uint32_t TSCEN;                     /*!< TS clock enable */
             uint32_t __RESERVED4[3];
        __IO uint32_t ADC12EN;                   /*!< ADC1/ ADC2 clock enable */
             uint32_t __RESERVED5[3];
    } AHBENR;                                /*!< RCC AHB peripheral clock register,                           Address offset: 0x14 */
    struct {
        __IO uint32_t SYSCFGEN;                  /*!< SYSCFG clock enable */
             uint32_t __RESERVED0[10];
        __IO uint32_t TIM1EN;                    /*!< TIM1 clock enable */
        __IO uint32_t SPI1EN;                    /*!< SPI1 clock enable */
             uint32_t __RESERVED1;
        __IO uint32_t USART1EN;                  /*!< USART1 clock enable */
             uint32_t __RESERVED2;
        __IO uint32_t TIM15EN;                   /*!< TIM15 clock enable */
        __IO uint32_t TIM16EN;                   /*!< TIM16 clock enable */
        __IO uint32_t TIM17EN;                   /*!< TIM17 clock enable */
             uint32_t __RESERVED3[13];
    } APB2ENR;                               /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x18 */
    struct {
        __IO uint32_t TIM2EN;                    /*!< Timer 2 clock enable */
        __IO uint32_t TIM3EN;                    /*!< Timer 3 clock enable */
        __IO uint32_t TIM4EN;                    /*!< Timer 4 clock enable */
             uint32_t __RESERVED0;
        __IO uint32_t TIM6EN;                    /*!< Timer 6 clock enable */
             uint32_t __RESERVED1[6];
        __IO uint32_t WWDGEN;                    /*!< Window Watchdog clock enable */
             uint32_t __RESERVED2[2];
        __IO uint32_t SPI2EN;                    /*!< SPI2 clock enable */
        __IO uint32_t SPI3EN;                    /*!< SPI3 clock enable */
             uint32_t __RESERVED3;
        __IO uint32_t USART2EN;                  /*!< USART 2 clock enable */
        __IO uint32_t USART3EN;                  /*!< USART 3 clock enable */
        __IO uint32_t UART4EN;                   /*!< UART 4 clock enable */
        __IO uint32_t UART5EN;                   /*!< UART 5 clock enable */
        __IO uint32_t I2C1EN;                    /*!< I2C 1 clock enable */
        __IO uint32_t I2C2EN;                    /*!< I2C 2 clock enable */
        __IO uint32_t USBEN;                     /*!< USB clock enable */
             uint32_t __RESERVED4;
        __IO uint32_t CANEN;                     /*!< CAN clock enable */
             uint32_t __RESERVED5[2];
        __IO uint32_t PWREN;                     /*!< PWR clock enable */
        __IO uint32_t DAC1EN;                    /*!< DAC 1 clock enable */
             uint32_t __RESERVED6[2];
    } APB1ENR;                               /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x1C */
    struct {
        __IO uint32_t LSEON;                     /*!< External Low Speed oscillator enable */
        __IO uint32_t LSERDY;                    /*!< External Low Speed oscillator Ready */
        __IO uint32_t LSEBYP;                    /*!< External Low Speed oscillator Bypass */
        __IO uint32_t LSEDRV[2];                 /*!< LSEDRV[1:0] bits (LSE Osc. drive capability) */
             uint32_t __RESERVED0[3];
        __IO uint32_t RTCSEL[2];                 /*!< RTCSEL[1:0] bits (RTC clock source selection) */
             uint32_t __RESERVED1[5];
        __IO uint32_t RTCEN;                     /*!< RTC clock enable */
        __IO uint32_t BDRST;                     /*!< Backup domain software reset  */
             uint32_t __RESERVED2[15];
    } BDCR;                                  /*!< RCC Backup domain control register,                          Address offset: 0x20 */
    struct {
        __IO uint32_t LSION;                     /*!< Internal Low Speed oscillator enable */
        __IO uint32_t LSIRDY;                    /*!< Internal Low Speed oscillator Ready */
             uint32_t __RESERVED0[21];
        __IO uint32_t V18PWRRSTF;                /*!< V1.8 power domain reset flag */
        __IO uint32_t RMVF;                      /*!< Remove reset flag */
        __IO uint32_t OBLRSTF;                   /*!< OBL reset flag */
        __IO uint32_t PINRSTF;                   /*!< PIN reset flag */
        __IO uint32_t PORRSTF;                   /*!< POR/PDR reset flag */
        __IO uint32_t SFTRSTF;                   /*!< Software Reset flag */
        __IO uint32_t IWDGRSTF;                  /*!< Independent Watchdog reset flag */
        __IO uint32_t WWDGRSTF;                  /*!< Window watchdog reset flag */
        __IO uint32_t LPWRRSTF;                  /*!< Low-Power reset flag */
    } CSR;                                   /*!< RCC clock control & status register,                         Address offset: 0x24 */
    struct {
             uint32_t __RESERVED0[17];
        __IO uint32_t GPIOARST;                  /*!< GPIOA reset */
        __IO uint32_t GPIOBRST;                  /*!< GPIOB reset */
        __IO uint32_t GPIOCRST;                  /*!< GPIOC reset */
        __IO uint32_t GPIODRST;                  /*!< GPIOD reset */
        __IO uint32_t GPIOERST;                  /*!< GPIOE reset */
        __IO uint32_t GPIOFRST;                  /*!< GPIOF reset */
             uint32_t __RESERVED1;
        __IO uint32_t TSCRST;                    /*!< TSC reset */
             uint32_t __RESERVED2[3];
        __IO uint32_t ADC12RST;                  /*!< ADC1 & ADC2 reset */
             uint32_t __RESERVED3[3];
    } AHBRSTR;                               /*!< RCC AHB peripheral reset register,                           Address offset: 0x28 */
    struct {
        __IO uint32_t PREDIV[4];                 /*!< PREDIV[3:0] bits */
        __IO uint32_t ADCPRE12[5];               /*!< ADCPRE12[8:4] bits */
             uint32_t __RESERVED0[23];
    } CFGR2;                                 /*!< RCC clock configuration register 2,                          Address offset: 0x2C */
    struct {
        __IO uint32_t USART1SW[2];               /*!< USART1SW[1:0] bits */
             uint32_t __RESERVED0[2];
        __IO uint32_t I2C1SW;                    /*!< I2C1SW bits */
        __IO uint32_t I2C2SW;                    /*!< I2C2SW bits */
             uint32_t __RESERVED1[2];
        __IO uint32_t TIM1SW;                    /*!< TIM1SW bits */
             uint32_t __RESERVED2[7];
        __IO uint32_t USART2SW[2];               /*!< USART2SW[1:0] bits */
        __IO uint32_t USART3SW[2];               /*!< USART3SW[1:0] bits */
        __IO uint32_t UART4SW[2];                /*!< UART4SW[1:0] bits */
        __IO uint32_t UART5SW[2];                /*!< UART5SW[1:0] bits */
             uint32_t __RESERVED3[8];
    } CFGR3;                                 /*!< RCC clock configuration register 3,                          Address offset: 0x30 */
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
            __IO uint32_t TAMP3F : 1;
            __IO uint32_t RECALPF : 1;
                 uint32_t __RESERVED0 : 15;
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
    __IO uint32_t WUTR;                      /*!< RTC wakeup timer register,                                Address offset: 0x14 */
         uint32_t __RESERVED0;               /*!< Reserved, 0x18                                                                 */
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
    __IO uint32_t WPR;                       /*!< RTC write protection register,                            Address offset: 0x24 */
    __IO uint32_t SSR;                       /*!< RTC sub second register,                                  Address offset: 0x28 */
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
    __IO uint32_t TSSSR;                     /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
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
            __IO uint32_t TAMP3E : 1;
            __IO uint32_t TAMP3TRG : 1;
            __IO uint32_t TAMPTS : 1;
            __IO uint32_t TAMPFREQ : 3;
            __IO uint32_t TAMPFLT : 2;
            __IO uint32_t TAMPPRCH : 2;
            __IO uint32_t TAMPPUDIS : 1;
                 uint32_t __RESERVED0 : 2;
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
         uint32_t __RESERVED1;               /*!< Reserved, 0x4C                                                                 */
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
        __IO uint32_t TAMP3F;
        __IO uint32_t RECALPF;
             uint32_t __RESERVED0[15];
    } ISR;                                   /*!< RTC initialization and status register,                   Address offset: 0x0C */
    struct {
        __IO uint32_t PREDIV_S[15];
             uint32_t __RESERVED0;
        __IO uint32_t PREDIV_A[7];
             uint32_t __RESERVED1[9];
    } PRER;                                  /*!< RTC prescaler register,                                   Address offset: 0x10 */
    __IO uint32_t WUTR[32];                  /*!< RTC wakeup timer register,                                Address offset: 0x14 */
         uint32_t __RESERVED0[32];           /*!< Reserved, 0x18                                                                 */
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
    __IO uint32_t WPR[32];                   /*!< RTC write protection register,                            Address offset: 0x24 */
    __IO uint32_t SSR[32];                   /*!< RTC sub second register,                                  Address offset: 0x28 */
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
    __IO uint32_t TSSSR[32];                 /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
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
        __IO uint32_t TAMP3E;
        __IO uint32_t TAMP3TRG;
        __IO uint32_t TAMPTS;
        __IO uint32_t TAMPFREQ[3];
        __IO uint32_t TAMPFLT[2];
        __IO uint32_t TAMPPRCH[2];
        __IO uint32_t TAMPPUDIS;
             uint32_t __RESERVED0[2];
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
         uint32_t __RESERVED1[32];           /*!< Reserved, 0x4C                                                                 */
    __IO uint32_t BKPR[16][32];                  /*!< RTC backup registers,                                     Address offset: 0x50-0x8C */
} RTC_BitBand_TypeDef;




/**
  * @brief Serial Peripheral Interface
  */


typedef struct {
    union {
        struct {
            __IO uint32_t CPHA : 1;                  /*!< Clock Phase */
            __IO uint32_t CPOL : 1;                  /*!< Clock Polarity */
            __IO uint32_t MSTR : 1;                  /*!< Master Selection */
            __IO uint32_t BR : 3;                    /*!< BR[2:0] bits (Baud Rate Control) */
            __IO uint32_t SPE : 1;                   /*!< SPI Enable */
            __IO uint32_t LSBFIRST : 1;              /*!< Frame Format */
            __IO uint32_t SSI : 1;                   /*!< Internal slave select */
            __IO uint32_t SSM : 1;                   /*!< Software slave management */
            __IO uint32_t RXONLY : 1;                /*!< Receive only */
            __IO uint32_t CRCL : 1;                  /*!< CRC Length */
            __IO uint32_t CRCNEXT : 1;               /*!< Transmit CRC next */
            __IO uint32_t CRCEN : 1;                 /*!< Hardware CRC calculation enable */
            __IO uint32_t BIDIOE : 1;                /*!< Output enable in bidirectional mode */
            __IO uint32_t BIDIMODE : 1;              /*!< Bidirectional data mode enable */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< SPI Control register 1,                              Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t RXDMAEN : 1;               /*!< Rx Buffer DMA Enable */
            __IO uint32_t TXDMAEN : 1;               /*!< Tx Buffer DMA Enable */
            __IO uint32_t SSOE : 1;                  /*!< SS Output Enable */
            __IO uint32_t NSSP : 1;                  /*!< NSS pulse management Enable */
            __IO uint32_t FRF : 1;                   /*!< Frame Format Enable */
            __IO uint32_t ERRIE : 1;                 /*!< Error Interrupt Enable */
            __IO uint32_t RXNEIE : 1;                /*!< RX buffer Not Empty Interrupt Enable */
            __IO uint32_t TXEIE : 1;                 /*!< Tx buffer Empty Interrupt Enable */
            __IO uint32_t DS : 4;                    /*!< DS[3:0] Data Size */
            __IO uint32_t FRXTH : 1;                 /*!< FIFO reception Threshold */
            __IO uint32_t LDMARX : 1;                /*!< Last DMA transfer for reception */
            __IO uint32_t LDMATX : 1;                /*!< Last DMA transfer for transmission */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< SPI Control register 2,                              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t RXNE : 1;                  /*!< Receive buffer Not Empty */
            __IO uint32_t TXE : 1;                   /*!< Transmit buffer Empty */
            __IO uint32_t CHSIDE : 1;                /*!< Channel side */
            __IO uint32_t UDR : 1;                   /*!< Underrun flag */
            __IO uint32_t CRCERR : 1;                /*!< CRC Error flag */
            __IO uint32_t MODF : 1;                  /*!< Mode fault */
            __IO uint32_t OVR : 1;                   /*!< Overrun flag */
            __IO uint32_t BSY : 1;                   /*!< Busy flag */
            __IO uint32_t FRE : 1;                   /*!< TI frame format error */
            __IO uint32_t FRLVL : 2;                 /*!< FIFO Reception Level */
            __IO uint32_t FTLVL : 2;                 /*!< FIFO Transmission Level */
                 uint32_t __RESERVED0 : 19;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< SPI Status register,                                 Address offset: 0x08 */
    __IO uint32_t DR;                        /*!< SPI data register,                                   Address offset: 0x0C */
    __IO uint32_t CRCPR;                     /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
    __IO uint32_t RXCRCR;                    /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
    __IO uint32_t TXCRCR;                    /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t CHLEN : 1;                 /*!<Channel length (number of bits per audio channel) */
            __IO uint32_t DATLEN : 2;                /*!<DATLEN[1:0] bits (Data length to be transferred) */
            __IO uint32_t CKPOL : 1;                 /*!<steady state clock polarity */
            __IO uint32_t I2SSTD : 2;                /*!<I2SSTD[1:0] bits (I2S standard selection) */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t PCMSYNC : 1;               /*!<PCM frame synchronization */
            __IO uint32_t I2SCFG : 2;                /*!<I2SCFG[1:0] bits (I2S configuration mode) */
            __IO uint32_t I2SE : 1;                  /*!<I2S Enable */
            __IO uint32_t I2SMOD : 1;                /*!<I2S mode selection */
                 uint32_t __RESERVED1 : 20;
        } b;
        __IO uint32_t w;
    } I2SCFGR;                               /*!< SPI_I2S configuration register,                      Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t I2SDIV : 8;                /*!<I2S Linear prescaler */
            __IO uint32_t ODD : 1;                   /*!<Odd factor for the prescaler */
            __IO uint32_t MCKOE : 1;                 /*!<Master Clock Output Enable */
                 uint32_t __RESERVED0 : 22;
        } b;
        __IO uint32_t w;
    } I2SPR;                                 /*!< SPI_I2S prescaler register,                          Address offset: 0x20 */
} SPI_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CPHA;                      /*!< Clock Phase */
        __IO uint32_t CPOL;                      /*!< Clock Polarity */
        __IO uint32_t MSTR;                      /*!< Master Selection */
        __IO uint32_t BR[3];                     /*!< BR[2:0] bits (Baud Rate Control) */
        __IO uint32_t SPE;                       /*!< SPI Enable */
        __IO uint32_t LSBFIRST;                  /*!< Frame Format */
        __IO uint32_t SSI;                       /*!< Internal slave select */
        __IO uint32_t SSM;                       /*!< Software slave management */
        __IO uint32_t RXONLY;                    /*!< Receive only */
        __IO uint32_t CRCL;                      /*!< CRC Length */
        __IO uint32_t CRCNEXT;                   /*!< Transmit CRC next */
        __IO uint32_t CRCEN;                     /*!< Hardware CRC calculation enable */
        __IO uint32_t BIDIOE;                    /*!< Output enable in bidirectional mode */
        __IO uint32_t BIDIMODE;                  /*!< Bidirectional data mode enable */
             uint32_t __RESERVED0[16];
    } CR1;                                   /*!< SPI Control register 1,                              Address offset: 0x00 */
    struct {
        __IO uint32_t RXDMAEN;                   /*!< Rx Buffer DMA Enable */
        __IO uint32_t TXDMAEN;                   /*!< Tx Buffer DMA Enable */
        __IO uint32_t SSOE;                      /*!< SS Output Enable */
        __IO uint32_t NSSP;                      /*!< NSS pulse management Enable */
        __IO uint32_t FRF;                       /*!< Frame Format Enable */
        __IO uint32_t ERRIE;                     /*!< Error Interrupt Enable */
        __IO uint32_t RXNEIE;                    /*!< RX buffer Not Empty Interrupt Enable */
        __IO uint32_t TXEIE;                     /*!< Tx buffer Empty Interrupt Enable */
        __IO uint32_t DS[4];                     /*!< DS[3:0] Data Size */
        __IO uint32_t FRXTH;                     /*!< FIFO reception Threshold */
        __IO uint32_t LDMARX;                    /*!< Last DMA transfer for reception */
        __IO uint32_t LDMATX;                    /*!< Last DMA transfer for transmission */
             uint32_t __RESERVED0[17];
    } CR2;                                   /*!< SPI Control register 2,                              Address offset: 0x04 */
    struct {
        __IO uint32_t RXNE;                      /*!< Receive buffer Not Empty */
        __IO uint32_t TXE;                       /*!< Transmit buffer Empty */
        __IO uint32_t CHSIDE;                    /*!< Channel side */
        __IO uint32_t UDR;                       /*!< Underrun flag */
        __IO uint32_t CRCERR;                    /*!< CRC Error flag */
        __IO uint32_t MODF;                      /*!< Mode fault */
        __IO uint32_t OVR;                       /*!< Overrun flag */
        __IO uint32_t BSY;                       /*!< Busy flag */
        __IO uint32_t FRE;                       /*!< TI frame format error */
        __IO uint32_t FRLVL[2];                  /*!< FIFO Reception Level */
        __IO uint32_t FTLVL[2];                  /*!< FIFO Transmission Level */
             uint32_t __RESERVED0[19];
    } SR;                                    /*!< SPI Status register,                                 Address offset: 0x08 */
    __IO uint32_t DR[32];                    /*!< SPI data register,                                   Address offset: 0x0C */
    __IO uint32_t CRCPR[32];                 /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
    __IO uint32_t RXCRCR[32];                /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
    __IO uint32_t TXCRCR[32];                /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
    struct {
        __IO uint32_t CHLEN;                     /*!<Channel length (number of bits per audio channel) */
        __IO uint32_t DATLEN[2];                 /*!<DATLEN[1:0] bits (Data length to be transferred) */
        __IO uint32_t CKPOL;                     /*!<steady state clock polarity */
        __IO uint32_t I2SSTD[2];                 /*!<I2SSTD[1:0] bits (I2S standard selection) */
             uint32_t __RESERVED0;
        __IO uint32_t PCMSYNC;                   /*!<PCM frame synchronization */
        __IO uint32_t I2SCFG[2];                 /*!<I2SCFG[1:0] bits (I2S configuration mode) */
        __IO uint32_t I2SE;                      /*!<I2S Enable */
        __IO uint32_t I2SMOD;                    /*!<I2S mode selection */
             uint32_t __RESERVED1[20];
    } I2SCFGR;                               /*!< SPI_I2S configuration register,                      Address offset: 0x1C */
    struct {
        __IO uint32_t I2SDIV[8];                 /*!<I2S Linear prescaler */
        __IO uint32_t ODD;                       /*!<Odd factor for the prescaler */
        __IO uint32_t MCKOE;                     /*!<Master Clock Output Enable */
             uint32_t __RESERVED0[22];
    } I2SPR;                                 /*!< SPI_I2S prescaler register,                          Address offset: 0x20 */
} SPI_BitBand_TypeDef;



/**
  * @brief TIM
  */

typedef struct {
    union {
        struct {
            __IO uint32_t CEN : 1;                   /*!<Counter enable */
            __IO uint32_t UDIS : 1;                  /*!<Update disable */
            __IO uint32_t URS : 1;                   /*!<Update request source */
            __IO uint32_t OPM : 1;                   /*!<One pulse mode */
            __IO uint32_t DIR : 1;                   /*!<Direction */
            __IO uint32_t CMS : 2;                   /*!<CMS[1:0] bits (Center-aligned mode selection) */
            __IO uint32_t ARPE : 1;                  /*!<Auto-reload preload enable */
            __IO uint32_t CKD : 2;                   /*!<CKD[1:0] bits (clock division) */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t UIFREMAP : 1;              /*!<Update interrupt flag remap */
                 uint32_t __RESERVED1 : 20;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< TIM control register 1,              Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t CCPC : 1;                  /*!<Capture/Compare Preloaded Control */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CCUS : 1;                  /*!<Capture/Compare Control Update Selection */
            __IO uint32_t CCDS : 1;                  /*!<Capture/Compare DMA Selection */
            __IO uint32_t MMS : 3;                   /*!<MMS[2:0] bits (Master Mode Selection) */
            __IO uint32_t TI1S : 1;                  /*!<TI1 Selection */
            __IO uint32_t OIS1 : 1;                  /*!<Output Idle state 1 (OC1 output) */
            __IO uint32_t OIS1N : 1;                 /*!<Output Idle state 1 (OC1N output) */
            __IO uint32_t OIS2 : 1;                  /*!<Output Idle state 2 (OC2 output) */
            __IO uint32_t OIS2N : 1;                 /*!<Output Idle state 2 (OC2N output) */
            __IO uint32_t OIS3 : 1;                  /*!<Output Idle state 3 (OC3 output) */
            __IO uint32_t OIS3N : 1;                 /*!<Output Idle state 3 (OC3N output) */
            __IO uint32_t OIS4 : 1;                  /*!<Output Idle state 4 (OC4 output) */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t OIS5 : 1;                  /*!<Output Idle state 4 (OC4 output) */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t OIS6 : 1;                  /*!<Output Idle state 4 (OC4 output) */
                 uint32_t __RESERVED3 : 1;
            __IO uint32_t MMS2 : 4;                  /*!<MMS[2:0] bits (Master Mode Selection) */
                 uint32_t __RESERVED4 : 8;
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< TIM control register 2,              Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t SMS : 3;                   /*!<SMS[2:0] bits (Slave mode selection) */
            __IO uint32_t OCCS : 1;                  /*!< OCREF clear selection */
            __IO uint32_t TS : 3;                    /*!<TS[2:0] bits (Trigger selection) */
            __IO uint32_t MSM : 1;                   /*!<Master/slave mode */
            __IO uint32_t ETF : 4;                   /*!<ETF[3:0] bits (External trigger filter) */
            __IO uint32_t ETPS : 2;                  /*!<ETPS[1:0] bits (External trigger prescaler) */
            __IO uint32_t ECE : 1;                   /*!<External clock enable */
            __IO uint32_t ETP : 1;                   /*!<External trigger polarity */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } SMCR;                                  /*!< TIM slave mode control register,     Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t UIE : 1;                   /*!<Update interrupt enable */
            __IO uint32_t CC1IE : 1;                 /*!<Capture/Compare 1 interrupt enable */
            __IO uint32_t CC2IE : 1;                 /*!<Capture/Compare 2 interrupt enable */
            __IO uint32_t CC3IE : 1;                 /*!<Capture/Compare 3 interrupt enable */
            __IO uint32_t CC4IE : 1;                 /*!<Capture/Compare 4 interrupt enable */
            __IO uint32_t COMIE : 1;                 /*!<COM interrupt enable */
            __IO uint32_t TIE : 1;                   /*!<Trigger interrupt enable */
            __IO uint32_t BIE : 1;                   /*!<Break interrupt enable */
            __IO uint32_t UDE : 1;                   /*!<Update DMA request enable */
            __IO uint32_t CC1DE : 1;                 /*!<Capture/Compare 1 DMA request enable */
            __IO uint32_t CC2DE : 1;                 /*!<Capture/Compare 2 DMA request enable */
            __IO uint32_t CC3DE : 1;                 /*!<Capture/Compare 3 DMA request enable */
            __IO uint32_t CC4DE : 1;                 /*!<Capture/Compare 4 DMA request enable */
            __IO uint32_t COMDE : 1;                 /*!<COM DMA request enable */
            __IO uint32_t TDE : 1;                   /*!<Trigger DMA request enable */
                 uint32_t __RESERVED0 : 17;
        } b;
        __IO uint32_t w;
    } DIER;                                  /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t UIF : 1;                   /*!<Update interrupt Flag */
            __IO uint32_t CC1IF : 1;                 /*!<Capture/Compare 1 interrupt Flag */
            __IO uint32_t CC2IF : 1;                 /*!<Capture/Compare 2 interrupt Flag */
            __IO uint32_t CC3IF : 1;                 /*!<Capture/Compare 3 interrupt Flag */
            __IO uint32_t CC4IF : 1;                 /*!<Capture/Compare 4 interrupt Flag */
            __IO uint32_t COMIF : 1;                 /*!<COM interrupt Flag */
            __IO uint32_t TIF : 1;                   /*!<Trigger interrupt Flag */
            __IO uint32_t BIF : 1;                   /*!<Break interrupt Flag */
            __IO uint32_t B2IF : 1;                  /*!<Break2 interrupt Flag */
            __IO uint32_t CC1OF : 1;                 /*!<Capture/Compare 1 Overcapture Flag */
            __IO uint32_t CC2OF : 1;                 /*!<Capture/Compare 2 Overcapture Flag */
            __IO uint32_t CC3OF : 1;                 /*!<Capture/Compare 3 Overcapture Flag */
            __IO uint32_t CC4OF : 1;                 /*!<Capture/Compare 4 Overcapture Flag */
                 uint32_t __RESERVED0 : 3;
            __IO uint32_t CC5IF : 1;                 /*!<Capture/Compare 5 interrupt Flag */
            __IO uint32_t CC6IF : 1;                 /*!<Capture/Compare 6 interrupt Flag */
                 uint32_t __RESERVED1 : 14;
        } b;
        __IO uint32_t w;
    } SR;                                    /*!< TIM status register,                 Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t UG : 1;                    /*!<Update Generation */
            __IO uint32_t CC1G : 1;                  /*!<Capture/Compare 1 Generation */
            __IO uint32_t CC2G : 1;                  /*!<Capture/Compare 2 Generation */
            __IO uint32_t CC3G : 1;                  /*!<Capture/Compare 3 Generation */
            __IO uint32_t CC4G : 1;                  /*!<Capture/Compare 4 Generation */
            __IO uint32_t COMG : 1;                  /*!<Capture/Compare Control Update Generation */
            __IO uint32_t TG : 1;                    /*!<Trigger Generation */
            __IO uint32_t BG : 1;                    /*!<Break Generation */
            __IO uint32_t B2G : 1;                   /*!<Break Generation */
                 uint32_t __RESERVED0 : 23;
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
            __IO uint32_t CC1E : 1;                  /*!<Capture/Compare 1 output enable */
            __IO uint32_t CC1P : 1;                  /*!<Capture/Compare 1 output Polarity */
            __IO uint32_t CC1NE : 1;                 /*!<Capture/Compare 1 Complementary output enable */
            __IO uint32_t CC1NP : 1;                 /*!<Capture/Compare 1 Complementary output Polarity */
            __IO uint32_t CC2E : 1;                  /*!<Capture/Compare 2 output enable */
            __IO uint32_t CC2P : 1;                  /*!<Capture/Compare 2 output Polarity */
            __IO uint32_t CC2NE : 1;                 /*!<Capture/Compare 2 Complementary output enable */
            __IO uint32_t CC2NP : 1;                 /*!<Capture/Compare 2 Complementary output Polarity */
            __IO uint32_t CC3E : 1;                  /*!<Capture/Compare 3 output enable */
            __IO uint32_t CC3P : 1;                  /*!<Capture/Compare 3 output Polarity */
            __IO uint32_t CC3NE : 1;                 /*!<Capture/Compare 3 Complementary output enable */
            __IO uint32_t CC3NP : 1;                 /*!<Capture/Compare 3 Complementary output Polarity */
            __IO uint32_t CC4E : 1;                  /*!<Capture/Compare 4 output enable */
            __IO uint32_t CC4P : 1;                  /*!<Capture/Compare 4 output Polarity */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t CC4NP : 1;                 /*!<Capture/Compare 4 Complementary output Polarity */
            __IO uint32_t CC5E : 1;                  /*!<Capture/Compare 5 output enable */
            __IO uint32_t CC5P : 1;                  /*!<Capture/Compare 5 output Polarity */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t CC6E : 1;                  /*!<Capture/Compare 6 output enable */
            __IO uint32_t CC6P : 1;                  /*!<Capture/Compare 6 output Polarity */
                 uint32_t __RESERVED2 : 10;
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
            __IO uint32_t OSSR : 1;                  /*!<Off-State Selection for Run mode */
            __IO uint32_t BKE : 1;                   /*!<Break enable for Break1 */
            __IO uint32_t BKP : 1;                   /*!<Break Polarity for Break1 */
            __IO uint32_t AOE : 1;                   /*!<Automatic Output enable */
            __IO uint32_t MOE : 1;                   /*!<Main Output enable */
            __IO uint32_t BKF : 4;                   /*!<Break Filter for Break1 */
            __IO uint32_t BK2F : 4;                  /*!<Break Filter for Break2 */
            __IO uint32_t BK2E : 1;                  /*!<Break enable for Break2 */
            __IO uint32_t BK2P : 1;                  /*!<Break Polarity for Break2 */
                 uint32_t __RESERVED0 : 6;
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
    __IO uint32_t OR;                        /*!< TIM option register,                 Address offset: 0x50 */
    union {
        struct {
                 uint32_t __RESERVED0 : 2;
            __IO uint32_t OC5FE : 1;                 /*!<Output Compare 5 Fast enable */
            __IO uint32_t OC5PE : 1;                 /*!<Output Compare 5 Preload enable */
            __IO uint32_t OC5M : 3;                  /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
            __IO uint32_t OC5CE : 1;                 /*!<Output Compare 5 Clear Enable */
                 uint32_t __RESERVED1 : 2;
            __IO uint32_t OC6FE : 1;                 /*!<Output Compare 6 Fast enable */
            __IO uint32_t OC6PE : 1;                 /*!<Output Compare 6 Preload enable */
            __IO uint32_t OC6M : 3;                  /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
            __IO uint32_t OC6CE : 1;                 /*!<Output Compare 6 Clear Enable */
                 uint32_t __RESERVED2 : 16;
        } b;
        __IO uint32_t w;
    } CCMR3;                                 /*!< TIM capture/compare mode register 3, Address offset: 0x54 */
    __IO uint32_t CCR5;                      /*!< TIM capture/compare register 5,      Address offset: 0x58 */
    __IO uint32_t CCR6;                      /*!< TIM capture/compare register 6,      Address offset: 0x5C */
} TIM_TypeDef;


typedef struct {
    struct {
        __IO uint32_t CEN;                       /*!<Counter enable */
        __IO uint32_t UDIS;                      /*!<Update disable */
        __IO uint32_t URS;                       /*!<Update request source */
        __IO uint32_t OPM;                       /*!<One pulse mode */
        __IO uint32_t DIR;                       /*!<Direction */
        __IO uint32_t CMS[2];                    /*!<CMS[1:0] bits (Center-aligned mode selection) */
        __IO uint32_t ARPE;                      /*!<Auto-reload preload enable */
        __IO uint32_t CKD[2];                    /*!<CKD[1:0] bits (clock division) */
             uint32_t __RESERVED0;
        __IO uint32_t UIFREMAP;                  /*!<Update interrupt flag remap */
             uint32_t __RESERVED1[20];
    } CR1;                                   /*!< TIM control register 1,              Address offset: 0x00 */
    struct {
        __IO uint32_t CCPC;                      /*!<Capture/Compare Preloaded Control */
             uint32_t __RESERVED0;
        __IO uint32_t CCUS;                      /*!<Capture/Compare Control Update Selection */
        __IO uint32_t CCDS;                      /*!<Capture/Compare DMA Selection */
        __IO uint32_t MMS[3];                    /*!<MMS[2:0] bits (Master Mode Selection) */
        __IO uint32_t TI1S;                      /*!<TI1 Selection */
        __IO uint32_t OIS1;                      /*!<Output Idle state 1 (OC1 output) */
        __IO uint32_t OIS1N;                     /*!<Output Idle state 1 (OC1N output) */
        __IO uint32_t OIS2;                      /*!<Output Idle state 2 (OC2 output) */
        __IO uint32_t OIS2N;                     /*!<Output Idle state 2 (OC2N output) */
        __IO uint32_t OIS3;                      /*!<Output Idle state 3 (OC3 output) */
        __IO uint32_t OIS3N;                     /*!<Output Idle state 3 (OC3N output) */
        __IO uint32_t OIS4;                      /*!<Output Idle state 4 (OC4 output) */
             uint32_t __RESERVED1;
        __IO uint32_t OIS5;                      /*!<Output Idle state 4 (OC4 output) */
             uint32_t __RESERVED2;
        __IO uint32_t OIS6;                      /*!<Output Idle state 4 (OC4 output) */
             uint32_t __RESERVED3;
        __IO uint32_t MMS2[4];                   /*!<MMS[2:0] bits (Master Mode Selection) */
             uint32_t __RESERVED4[8];
    } CR2;                                   /*!< TIM control register 2,              Address offset: 0x04 */
    struct {
        __IO uint32_t SMS[3];                    /*!<SMS[2:0] bits (Slave mode selection) */
        __IO uint32_t OCCS;                      /*!< OCREF clear selection */
        __IO uint32_t TS[3];                     /*!<TS[2:0] bits (Trigger selection) */
        __IO uint32_t MSM;                       /*!<Master/slave mode */
        __IO uint32_t ETF[4];                    /*!<ETF[3:0] bits (External trigger filter) */
        __IO uint32_t ETPS[2];                   /*!<ETPS[1:0] bits (External trigger prescaler) */
        __IO uint32_t ECE;                       /*!<External clock enable */
        __IO uint32_t ETP;                       /*!<External trigger polarity */
             uint32_t __RESERVED0[16];
    } SMCR;                                  /*!< TIM slave mode control register,     Address offset: 0x08 */
    struct {
        __IO uint32_t UIE;                       /*!<Update interrupt enable */
        __IO uint32_t CC1IE;                     /*!<Capture/Compare 1 interrupt enable */
        __IO uint32_t CC2IE;                     /*!<Capture/Compare 2 interrupt enable */
        __IO uint32_t CC3IE;                     /*!<Capture/Compare 3 interrupt enable */
        __IO uint32_t CC4IE;                     /*!<Capture/Compare 4 interrupt enable */
        __IO uint32_t COMIE;                     /*!<COM interrupt enable */
        __IO uint32_t TIE;                       /*!<Trigger interrupt enable */
        __IO uint32_t BIE;                       /*!<Break interrupt enable */
        __IO uint32_t UDE;                       /*!<Update DMA request enable */
        __IO uint32_t CC1DE;                     /*!<Capture/Compare 1 DMA request enable */
        __IO uint32_t CC2DE;                     /*!<Capture/Compare 2 DMA request enable */
        __IO uint32_t CC3DE;                     /*!<Capture/Compare 3 DMA request enable */
        __IO uint32_t CC4DE;                     /*!<Capture/Compare 4 DMA request enable */
        __IO uint32_t COMDE;                     /*!<COM DMA request enable */
        __IO uint32_t TDE;                       /*!<Trigger DMA request enable */
             uint32_t __RESERVED0[17];
    } DIER;                                  /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
    struct {
        __IO uint32_t UIF;                       /*!<Update interrupt Flag */
        __IO uint32_t CC1IF;                     /*!<Capture/Compare 1 interrupt Flag */
        __IO uint32_t CC2IF;                     /*!<Capture/Compare 2 interrupt Flag */
        __IO uint32_t CC3IF;                     /*!<Capture/Compare 3 interrupt Flag */
        __IO uint32_t CC4IF;                     /*!<Capture/Compare 4 interrupt Flag */
        __IO uint32_t COMIF;                     /*!<COM interrupt Flag */
        __IO uint32_t TIF;                       /*!<Trigger interrupt Flag */
        __IO uint32_t BIF;                       /*!<Break interrupt Flag */
        __IO uint32_t B2IF;                      /*!<Break2 interrupt Flag */
        __IO uint32_t CC1OF;                     /*!<Capture/Compare 1 Overcapture Flag */
        __IO uint32_t CC2OF;                     /*!<Capture/Compare 2 Overcapture Flag */
        __IO uint32_t CC3OF;                     /*!<Capture/Compare 3 Overcapture Flag */
        __IO uint32_t CC4OF;                     /*!<Capture/Compare 4 Overcapture Flag */
             uint32_t __RESERVED0[3];
        __IO uint32_t CC5IF;                     /*!<Capture/Compare 5 interrupt Flag */
        __IO uint32_t CC6IF;                     /*!<Capture/Compare 6 interrupt Flag */
             uint32_t __RESERVED1[14];
    } SR;                                    /*!< TIM status register,                 Address offset: 0x10 */
    struct {
        __IO uint32_t UG;                        /*!<Update Generation */
        __IO uint32_t CC1G;                      /*!<Capture/Compare 1 Generation */
        __IO uint32_t CC2G;                      /*!<Capture/Compare 2 Generation */
        __IO uint32_t CC3G;                      /*!<Capture/Compare 3 Generation */
        __IO uint32_t CC4G;                      /*!<Capture/Compare 4 Generation */
        __IO uint32_t COMG;                      /*!<Capture/Compare Control Update Generation */
        __IO uint32_t TG;                        /*!<Trigger Generation */
        __IO uint32_t BG;                        /*!<Break Generation */
        __IO uint32_t B2G;                       /*!<Break Generation */
             uint32_t __RESERVED0[23];
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
        __IO uint32_t CC1E;                      /*!<Capture/Compare 1 output enable */
        __IO uint32_t CC1P;                      /*!<Capture/Compare 1 output Polarity */
        __IO uint32_t CC1NE;                     /*!<Capture/Compare 1 Complementary output enable */
        __IO uint32_t CC1NP;                     /*!<Capture/Compare 1 Complementary output Polarity */
        __IO uint32_t CC2E;                      /*!<Capture/Compare 2 output enable */
        __IO uint32_t CC2P;                      /*!<Capture/Compare 2 output Polarity */
        __IO uint32_t CC2NE;                     /*!<Capture/Compare 2 Complementary output enable */
        __IO uint32_t CC2NP;                     /*!<Capture/Compare 2 Complementary output Polarity */
        __IO uint32_t CC3E;                      /*!<Capture/Compare 3 output enable */
        __IO uint32_t CC3P;                      /*!<Capture/Compare 3 output Polarity */
        __IO uint32_t CC3NE;                     /*!<Capture/Compare 3 Complementary output enable */
        __IO uint32_t CC3NP;                     /*!<Capture/Compare 3 Complementary output Polarity */
        __IO uint32_t CC4E;                      /*!<Capture/Compare 4 output enable */
        __IO uint32_t CC4P;                      /*!<Capture/Compare 4 output Polarity */
             uint32_t __RESERVED0;
        __IO uint32_t CC4NP;                     /*!<Capture/Compare 4 Complementary output Polarity */
        __IO uint32_t CC5E;                      /*!<Capture/Compare 5 output enable */
        __IO uint32_t CC5P;                      /*!<Capture/Compare 5 output Polarity */
             uint32_t __RESERVED1[2];
        __IO uint32_t CC6E;                      /*!<Capture/Compare 6 output enable */
        __IO uint32_t CC6P;                      /*!<Capture/Compare 6 output Polarity */
             uint32_t __RESERVED2[10];
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
        __IO uint32_t OSSR;                      /*!<Off-State Selection for Run mode */
        __IO uint32_t BKE;                       /*!<Break enable for Break1 */
        __IO uint32_t BKP;                       /*!<Break Polarity for Break1 */
        __IO uint32_t AOE;                       /*!<Automatic Output enable */
        __IO uint32_t MOE;                       /*!<Main Output enable */
        __IO uint32_t BKF[4];                    /*!<Break Filter for Break1 */
        __IO uint32_t BK2F[4];                   /*!<Break Filter for Break2 */
        __IO uint32_t BK2E;                      /*!<Break enable for Break2 */
        __IO uint32_t BK2P;                      /*!<Break Polarity for Break2 */
             uint32_t __RESERVED0[6];
    } BDTR;                                  /*!< TIM break and dead-time register,    Address offset: 0x44 */
    struct {
        __IO uint32_t DBA[5];                    /*!<DBA[4:0] bits (DMA Base Address) */
             uint32_t __RESERVED0[3];
        __IO uint32_t DBL[5];                    /*!<DBL[4:0] bits (DMA Burst Length) */
             uint32_t __RESERVED1[19];
    } DCR;                                   /*!< TIM DMA control register,            Address offset: 0x48 */
    __IO uint32_t DMAR[32];                  /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    __IO uint32_t OR[32];                    /*!< TIM option register,                 Address offset: 0x50 */
    struct {
             uint32_t __RESERVED0[2];
        __IO uint32_t OC5FE;                     /*!<Output Compare 5 Fast enable */
        __IO uint32_t OC5PE;                     /*!<Output Compare 5 Preload enable */
        __IO uint32_t OC5M[3];                   /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
        __IO uint32_t OC5CE;                     /*!<Output Compare 5 Clear Enable */
             uint32_t __RESERVED1[2];
        __IO uint32_t OC6FE;                     /*!<Output Compare 6 Fast enable */
        __IO uint32_t OC6PE;                     /*!<Output Compare 6 Preload enable */
        __IO uint32_t OC6M[3];                   /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
        __IO uint32_t OC6CE;                     /*!<Output Compare 6 Clear Enable */
             uint32_t __RESERVED2[16];
    } CCMR3;                                 /*!< TIM capture/compare mode register 3, Address offset: 0x54 */
    struct {
        __IO uint32_t CCR5[16];                  /*!<Capture/Compare 5 Value */
             uint32_t __RESERVED0[13];
        __IO uint32_t GC5C1;                     /*!<Group Channel 5 and Channel 1 */
        __IO uint32_t GC5C2;                     /*!<Group Channel 5 and Channel 2 */
        __IO uint32_t GC5C3;                     /*!<Group Channel 5 and Channel 3 */
    } CCR5;                                  /*!< TIM capture/compare register5,       Address offset: 0x58 */
    __IO uint32_t CCR6[32];                  /*!< TIM capture/compare register 4,      Address offset: 0x5C */
} TIM_BitBand_TypeDef;



/**
  * @brief Touch Sensing Controller (TSC)
  */

typedef struct {
    union {
        struct {
            __IO uint32_t TSCE : 1;                  /*!<Touch sensing controller enable */
            __IO uint32_t START : 1;                 /*!<Start acquisition */
            __IO uint32_t AM : 1;                    /*!<Acquisition mode */
            __IO uint32_t SYNCPOL : 1;               /*!<Synchronization pin polarity */
            __IO uint32_t IODEF : 1;                 /*!<IO default mode */
            __IO uint32_t MCV : 3;                   /*!<MCV[2:0] bits (Max Count Value) */
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t PGPSC : 3;                 /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
            __IO uint32_t SSPSC : 1;                 /*!<Spread Spectrum Prescaler */
            __IO uint32_t SSE : 1;                   /*!<Spread Spectrum Enable */
            __IO uint32_t SSD : 7;                   /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
            __IO uint32_t CTPL : 4;                  /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
            __IO uint32_t CTPH : 4;                  /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
        } b;
        __IO uint32_t w;
    } CR;                                    /*!< TSC control register,                                     Address offset: 0x00 */
    union {
        struct {
            __IO uint32_t EOAIE : 1;                 /*!<End of acquisition interrupt enable */
            __IO uint32_t MCEIE : 1;                 /*!<Max count error interrupt enable */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } IER;                                   /*!< TSC interrupt enable register,                            Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t EOAIC : 1;                 /*!<End of acquisition interrupt clear */
            __IO uint32_t MCEIC : 1;                 /*!<Max count error interrupt clear */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } ICR;                                   /*!< TSC interrupt clear register,                             Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t EOAF : 1;                  /*!<End of acquisition flag */
            __IO uint32_t MCEF : 1;                  /*!<Max count error flag */
                 uint32_t __RESERVED0 : 30;
        } b;
        __IO uint32_t w;
    } ISR;                                   /*!< TSC interrupt status register,                            Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G1_IO2 : 1;                /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G1_IO3 : 1;                /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G1_IO4 : 1;                /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO1 : 1;                /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO2 : 1;                /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO3 : 1;                /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G2_IO4 : 1;                /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO1 : 1;                /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO2 : 1;                /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO3 : 1;                /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G3_IO4 : 1;                /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO1 : 1;                /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO2 : 1;                /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO3 : 1;                /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G4_IO4 : 1;                /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO1 : 1;                /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO2 : 1;                /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO3 : 1;                /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G5_IO4 : 1;                /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO1 : 1;                /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO2 : 1;                /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO3 : 1;                /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G6_IO4 : 1;                /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO1 : 1;                /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO2 : 1;                /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO3 : 1;                /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G7_IO4 : 1;                /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO1 : 1;                /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO2 : 1;                /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO3 : 1;                /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
            __IO uint32_t G8_IO4 : 1;                /*!<GROUP8_IO4 schmitt trigger hysteresis mode */
        } b;
        __IO uint32_t w;
    } IOHCR;                                 /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
         uint32_t __RESERVED0;               /*!< Reserved,                                                 Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                /*!<GROUP1_IO1 analog switch enable */
            __IO uint32_t G1_IO2 : 1;                /*!<GROUP1_IO2 analog switch enable */
            __IO uint32_t G1_IO3 : 1;                /*!<GROUP1_IO3 analog switch enable */
            __IO uint32_t G1_IO4 : 1;                /*!<GROUP1_IO4 analog switch enable */
            __IO uint32_t G2_IO1 : 1;                /*!<GROUP2_IO1 analog switch enable */
            __IO uint32_t G2_IO2 : 1;                /*!<GROUP2_IO2 analog switch enable */
            __IO uint32_t G2_IO3 : 1;                /*!<GROUP2_IO3 analog switch enable */
            __IO uint32_t G2_IO4 : 1;                /*!<GROUP2_IO4 analog switch enable */
            __IO uint32_t G3_IO1 : 1;                /*!<GROUP3_IO1 analog switch enable */
            __IO uint32_t G3_IO2 : 1;                /*!<GROUP3_IO2 analog switch enable */
            __IO uint32_t G3_IO3 : 1;                /*!<GROUP3_IO3 analog switch enable */
            __IO uint32_t G3_IO4 : 1;                /*!<GROUP3_IO4 analog switch enable */
            __IO uint32_t G4_IO1 : 1;                /*!<GROUP4_IO1 analog switch enable */
            __IO uint32_t G4_IO2 : 1;                /*!<GROUP4_IO2 analog switch enable */
            __IO uint32_t G4_IO3 : 1;                /*!<GROUP4_IO3 analog switch enable */
            __IO uint32_t G4_IO4 : 1;                /*!<GROUP4_IO4 analog switch enable */
            __IO uint32_t G5_IO1 : 1;                /*!<GROUP5_IO1 analog switch enable */
            __IO uint32_t G5_IO2 : 1;                /*!<GROUP5_IO2 analog switch enable */
            __IO uint32_t G5_IO3 : 1;                /*!<GROUP5_IO3 analog switch enable */
            __IO uint32_t G5_IO4 : 1;                /*!<GROUP5_IO4 analog switch enable */
            __IO uint32_t G6_IO1 : 1;                /*!<GROUP6_IO1 analog switch enable */
            __IO uint32_t G6_IO2 : 1;                /*!<GROUP6_IO2 analog switch enable */
            __IO uint32_t G6_IO3 : 1;                /*!<GROUP6_IO3 analog switch enable */
            __IO uint32_t G6_IO4 : 1;                /*!<GROUP6_IO4 analog switch enable */
            __IO uint32_t G7_IO1 : 1;                /*!<GROUP7_IO1 analog switch enable */
            __IO uint32_t G7_IO2 : 1;                /*!<GROUP7_IO2 analog switch enable */
            __IO uint32_t G7_IO3 : 1;                /*!<GROUP7_IO3 analog switch enable */
            __IO uint32_t G7_IO4 : 1;                /*!<GROUP7_IO4 analog switch enable */
            __IO uint32_t G8_IO1 : 1;                /*!<GROUP8_IO1 analog switch enable */
            __IO uint32_t G8_IO2 : 1;                /*!<GROUP8_IO2 analog switch enable */
            __IO uint32_t G8_IO3 : 1;                /*!<GROUP8_IO3 analog switch enable */
            __IO uint32_t G8_IO4 : 1;                /*!<GROUP8_IO4 analog switch enable */
        } b;
        __IO uint32_t w;
    } IOASCR;                                /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
         uint32_t __RESERVED1;               /*!< Reserved,                                                 Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                /*!<GROUP1_IO1 sampling mode */
            __IO uint32_t G1_IO2 : 1;                /*!<GROUP1_IO2 sampling mode */
            __IO uint32_t G1_IO3 : 1;                /*!<GROUP1_IO3 sampling mode */
            __IO uint32_t G1_IO4 : 1;                /*!<GROUP1_IO4 sampling mode */
            __IO uint32_t G2_IO1 : 1;                /*!<GROUP2_IO1 sampling mode */
            __IO uint32_t G2_IO2 : 1;                /*!<GROUP2_IO2 sampling mode */
            __IO uint32_t G2_IO3 : 1;                /*!<GROUP2_IO3 sampling mode */
            __IO uint32_t G2_IO4 : 1;                /*!<GROUP2_IO4 sampling mode */
            __IO uint32_t G3_IO1 : 1;                /*!<GROUP3_IO1 sampling mode */
            __IO uint32_t G3_IO2 : 1;                /*!<GROUP3_IO2 sampling mode */
            __IO uint32_t G3_IO3 : 1;                /*!<GROUP3_IO3 sampling mode */
            __IO uint32_t G3_IO4 : 1;                /*!<GROUP3_IO4 sampling mode */
            __IO uint32_t G4_IO1 : 1;                /*!<GROUP4_IO1 sampling mode */
            __IO uint32_t G4_IO2 : 1;                /*!<GROUP4_IO2 sampling mode */
            __IO uint32_t G4_IO3 : 1;                /*!<GROUP4_IO3 sampling mode */
            __IO uint32_t G4_IO4 : 1;                /*!<GROUP4_IO4 sampling mode */
            __IO uint32_t G5_IO1 : 1;                /*!<GROUP5_IO1 sampling mode */
            __IO uint32_t G5_IO2 : 1;                /*!<GROUP5_IO2 sampling mode */
            __IO uint32_t G5_IO3 : 1;                /*!<GROUP5_IO3 sampling mode */
            __IO uint32_t G5_IO4 : 1;                /*!<GROUP5_IO4 sampling mode */
            __IO uint32_t G6_IO1 : 1;                /*!<GROUP6_IO1 sampling mode */
            __IO uint32_t G6_IO2 : 1;                /*!<GROUP6_IO2 sampling mode */
            __IO uint32_t G6_IO3 : 1;                /*!<GROUP6_IO3 sampling mode */
            __IO uint32_t G6_IO4 : 1;                /*!<GROUP6_IO4 sampling mode */
            __IO uint32_t G7_IO1 : 1;                /*!<GROUP7_IO1 sampling mode */
            __IO uint32_t G7_IO2 : 1;                /*!<GROUP7_IO2 sampling mode */
            __IO uint32_t G7_IO3 : 1;                /*!<GROUP7_IO3 sampling mode */
            __IO uint32_t G7_IO4 : 1;                /*!<GROUP7_IO4 sampling mode */
            __IO uint32_t G8_IO1 : 1;                /*!<GROUP8_IO1 sampling mode */
            __IO uint32_t G8_IO2 : 1;                /*!<GROUP8_IO2 sampling mode */
            __IO uint32_t G8_IO3 : 1;                /*!<GROUP8_IO3 sampling mode */
            __IO uint32_t G8_IO4 : 1;                /*!<GROUP8_IO4 sampling mode */
        } b;
        __IO uint32_t w;
    } IOSCR;                                 /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
         uint32_t __RESERVED2;               /*!< Reserved,                                                 Address offset: 0x24 */
    union {
        struct {
            __IO uint32_t G1_IO1 : 1;                /*!<GROUP1_IO1 channel mode */
            __IO uint32_t G1_IO2 : 1;                /*!<GROUP1_IO2 channel mode */
            __IO uint32_t G1_IO3 : 1;                /*!<GROUP1_IO3 channel mode */
            __IO uint32_t G1_IO4 : 1;                /*!<GROUP1_IO4 channel mode */
            __IO uint32_t G2_IO1 : 1;                /*!<GROUP2_IO1 channel mode */
            __IO uint32_t G2_IO2 : 1;                /*!<GROUP2_IO2 channel mode */
            __IO uint32_t G2_IO3 : 1;                /*!<GROUP2_IO3 channel mode */
            __IO uint32_t G2_IO4 : 1;                /*!<GROUP2_IO4 channel mode */
            __IO uint32_t G3_IO1 : 1;                /*!<GROUP3_IO1 channel mode */
            __IO uint32_t G3_IO2 : 1;                /*!<GROUP3_IO2 channel mode */
            __IO uint32_t G3_IO3 : 1;                /*!<GROUP3_IO3 channel mode */
            __IO uint32_t G3_IO4 : 1;                /*!<GROUP3_IO4 channel mode */
            __IO uint32_t G4_IO1 : 1;                /*!<GROUP4_IO1 channel mode */
            __IO uint32_t G4_IO2 : 1;                /*!<GROUP4_IO2 channel mode */
            __IO uint32_t G4_IO3 : 1;                /*!<GROUP4_IO3 channel mode */
            __IO uint32_t G4_IO4 : 1;                /*!<GROUP4_IO4 channel mode */
            __IO uint32_t G5_IO1 : 1;                /*!<GROUP5_IO1 channel mode */
            __IO uint32_t G5_IO2 : 1;                /*!<GROUP5_IO2 channel mode */
            __IO uint32_t G5_IO3 : 1;                /*!<GROUP5_IO3 channel mode */
            __IO uint32_t G5_IO4 : 1;                /*!<GROUP5_IO4 channel mode */
            __IO uint32_t G6_IO1 : 1;                /*!<GROUP6_IO1 channel mode */
            __IO uint32_t G6_IO2 : 1;                /*!<GROUP6_IO2 channel mode */
            __IO uint32_t G6_IO3 : 1;                /*!<GROUP6_IO3 channel mode */
            __IO uint32_t G6_IO4 : 1;                /*!<GROUP6_IO4 channel mode */
            __IO uint32_t G7_IO1 : 1;                /*!<GROUP7_IO1 channel mode */
            __IO uint32_t G7_IO2 : 1;                /*!<GROUP7_IO2 channel mode */
            __IO uint32_t G7_IO3 : 1;                /*!<GROUP7_IO3 channel mode */
            __IO uint32_t G7_IO4 : 1;                /*!<GROUP7_IO4 channel mode */
            __IO uint32_t G8_IO1 : 1;                /*!<GROUP8_IO1 channel mode */
            __IO uint32_t G8_IO2 : 1;                /*!<GROUP8_IO2 channel mode */
            __IO uint32_t G8_IO3 : 1;                /*!<GROUP8_IO3 channel mode */
            __IO uint32_t G8_IO4 : 1;                /*!<GROUP8_IO4 channel mode */
        } b;
        __IO uint32_t w;
    } IOCCR;                                 /*!< TSC I/O channel control register,                         Address offset: 0x28 */
         uint32_t __RESERVED3;               /*!< Reserved,                                                 Address offset: 0x2C */
    union {
        struct {
            __IO uint32_t G1E : 1;                   /*!<Analog IO GROUP1 enable */
            __IO uint32_t G2E : 1;                   /*!<Analog IO GROUP2 enable */
            __IO uint32_t G3E : 1;                   /*!<Analog IO GROUP3 enable */
            __IO uint32_t G4E : 1;                   /*!<Analog IO GROUP4 enable */
            __IO uint32_t G5E : 1;                   /*!<Analog IO GROUP5 enable */
            __IO uint32_t G6E : 1;                   /*!<Analog IO GROUP6 enable */
            __IO uint32_t G7E : 1;                   /*!<Analog IO GROUP7 enable */
            __IO uint32_t G8E : 1;                   /*!<Analog IO GROUP8 enable */
                 uint32_t __RESERVED0 : 8;
            __IO uint32_t G1S : 1;                   /*!<Analog IO GROUP1 status */
            __IO uint32_t G2S : 1;                   /*!<Analog IO GROUP2 status */
            __IO uint32_t G3S : 1;                   /*!<Analog IO GROUP3 status */
            __IO uint32_t G4S : 1;                   /*!<Analog IO GROUP4 status */
            __IO uint32_t G5S : 1;                   /*!<Analog IO GROUP5 status */
            __IO uint32_t G6S : 1;                   /*!<Analog IO GROUP6 status */
            __IO uint32_t G7S : 1;                   /*!<Analog IO GROUP7 status */
            __IO uint32_t G8S : 1;                   /*!<Analog IO GROUP8 status */
                 uint32_t __RESERVED1 : 8;
        } b;
        __IO uint32_t w;
    } IOGCSR;                                /*!< TSC I/O group control status register,                    Address offset: 0x30 */
    __IO uint32_t IOGXCR[8];                    /*!< TSC I/O group x counter register,                         Address offset: 0x34-50 */
} TSC_TypeDef;


typedef struct {
    struct {
        __IO uint32_t TSCE;                      /*!<Touch sensing controller enable */
        __IO uint32_t START;                     /*!<Start acquisition */
        __IO uint32_t AM;                        /*!<Acquisition mode */
        __IO uint32_t SYNCPOL;                   /*!<Synchronization pin polarity */
        __IO uint32_t IODEF;                     /*!<IO default mode */
        __IO uint32_t MCV[3];                    /*!<MCV[2:0] bits (Max Count Value) */
             uint32_t __RESERVED0[4];
        __IO uint32_t PGPSC[3];                  /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
        __IO uint32_t SSPSC;                     /*!<Spread Spectrum Prescaler */
        __IO uint32_t SSE;                       /*!<Spread Spectrum Enable */
        __IO uint32_t SSD[7];                    /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
        __IO uint32_t CTPL[4];                   /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
        __IO uint32_t CTPH[4];                   /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
    } CR;                                    /*!< TSC control register,                                     Address offset: 0x00 */
    struct {
        __IO uint32_t EOAIE;                     /*!<End of acquisition interrupt enable */
        __IO uint32_t MCEIE;                     /*!<Max count error interrupt enable */
             uint32_t __RESERVED0[30];
    } IER;                                   /*!< TSC interrupt enable register,                            Address offset: 0x04 */
    struct {
        __IO uint32_t EOAIC;                     /*!<End of acquisition interrupt clear */
        __IO uint32_t MCEIC;                     /*!<Max count error interrupt clear */
             uint32_t __RESERVED0[30];
    } ICR;                                   /*!< TSC interrupt clear register,                             Address offset: 0x08 */
    struct {
        __IO uint32_t EOAF;                      /*!<End of acquisition flag */
        __IO uint32_t MCEF;                      /*!<Max count error flag */
             uint32_t __RESERVED0[30];
    } ISR;                                   /*!< TSC interrupt status register,                            Address offset: 0x0C */
    struct {
        __IO uint32_t G1_IO1;                    /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G1_IO2;                    /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G1_IO3;                    /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G1_IO4;                    /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO1;                    /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO2;                    /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO3;                    /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G2_IO4;                    /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO1;                    /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO2;                    /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO3;                    /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G3_IO4;                    /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO1;                    /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO2;                    /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO3;                    /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G4_IO4;                    /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO1;                    /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO2;                    /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO3;                    /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G5_IO4;                    /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO1;                    /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO2;                    /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO3;                    /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G6_IO4;                    /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO1;                    /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO2;                    /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO3;                    /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G7_IO4;                    /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO1;                    /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO2;                    /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO3;                    /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
        __IO uint32_t G8_IO4;                    /*!<GROUP8_IO4 schmitt trigger hysteresis mode */
    } IOHCR;                                 /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
         uint32_t __RESERVED0[32];           /*!< Reserved,                                                 Address offset: 0x14 */
    struct {
        __IO uint32_t G1_IO1;                    /*!<GROUP1_IO1 analog switch enable */
        __IO uint32_t G1_IO2;                    /*!<GROUP1_IO2 analog switch enable */
        __IO uint32_t G1_IO3;                    /*!<GROUP1_IO3 analog switch enable */
        __IO uint32_t G1_IO4;                    /*!<GROUP1_IO4 analog switch enable */
        __IO uint32_t G2_IO1;                    /*!<GROUP2_IO1 analog switch enable */
        __IO uint32_t G2_IO2;                    /*!<GROUP2_IO2 analog switch enable */
        __IO uint32_t G2_IO3;                    /*!<GROUP2_IO3 analog switch enable */
        __IO uint32_t G2_IO4;                    /*!<GROUP2_IO4 analog switch enable */
        __IO uint32_t G3_IO1;                    /*!<GROUP3_IO1 analog switch enable */
        __IO uint32_t G3_IO2;                    /*!<GROUP3_IO2 analog switch enable */
        __IO uint32_t G3_IO3;                    /*!<GROUP3_IO3 analog switch enable */
        __IO uint32_t G3_IO4;                    /*!<GROUP3_IO4 analog switch enable */
        __IO uint32_t G4_IO1;                    /*!<GROUP4_IO1 analog switch enable */
        __IO uint32_t G4_IO2;                    /*!<GROUP4_IO2 analog switch enable */
        __IO uint32_t G4_IO3;                    /*!<GROUP4_IO3 analog switch enable */
        __IO uint32_t G4_IO4;                    /*!<GROUP4_IO4 analog switch enable */
        __IO uint32_t G5_IO1;                    /*!<GROUP5_IO1 analog switch enable */
        __IO uint32_t G5_IO2;                    /*!<GROUP5_IO2 analog switch enable */
        __IO uint32_t G5_IO3;                    /*!<GROUP5_IO3 analog switch enable */
        __IO uint32_t G5_IO4;                    /*!<GROUP5_IO4 analog switch enable */
        __IO uint32_t G6_IO1;                    /*!<GROUP6_IO1 analog switch enable */
        __IO uint32_t G6_IO2;                    /*!<GROUP6_IO2 analog switch enable */
        __IO uint32_t G6_IO3;                    /*!<GROUP6_IO3 analog switch enable */
        __IO uint32_t G6_IO4;                    /*!<GROUP6_IO4 analog switch enable */
        __IO uint32_t G7_IO1;                    /*!<GROUP7_IO1 analog switch enable */
        __IO uint32_t G7_IO2;                    /*!<GROUP7_IO2 analog switch enable */
        __IO uint32_t G7_IO3;                    /*!<GROUP7_IO3 analog switch enable */
        __IO uint32_t G7_IO4;                    /*!<GROUP7_IO4 analog switch enable */
        __IO uint32_t G8_IO1;                    /*!<GROUP8_IO1 analog switch enable */
        __IO uint32_t G8_IO2;                    /*!<GROUP8_IO2 analog switch enable */
        __IO uint32_t G8_IO3;                    /*!<GROUP8_IO3 analog switch enable */
        __IO uint32_t G8_IO4;                    /*!<GROUP8_IO4 analog switch enable */
    } IOASCR;                                /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
         uint32_t __RESERVED1[32];           /*!< Reserved,                                                 Address offset: 0x1C */
    struct {
        __IO uint32_t G1_IO1;                    /*!<GROUP1_IO1 sampling mode */
        __IO uint32_t G1_IO2;                    /*!<GROUP1_IO2 sampling mode */
        __IO uint32_t G1_IO3;                    /*!<GROUP1_IO3 sampling mode */
        __IO uint32_t G1_IO4;                    /*!<GROUP1_IO4 sampling mode */
        __IO uint32_t G2_IO1;                    /*!<GROUP2_IO1 sampling mode */
        __IO uint32_t G2_IO2;                    /*!<GROUP2_IO2 sampling mode */
        __IO uint32_t G2_IO3;                    /*!<GROUP2_IO3 sampling mode */
        __IO uint32_t G2_IO4;                    /*!<GROUP2_IO4 sampling mode */
        __IO uint32_t G3_IO1;                    /*!<GROUP3_IO1 sampling mode */
        __IO uint32_t G3_IO2;                    /*!<GROUP3_IO2 sampling mode */
        __IO uint32_t G3_IO3;                    /*!<GROUP3_IO3 sampling mode */
        __IO uint32_t G3_IO4;                    /*!<GROUP3_IO4 sampling mode */
        __IO uint32_t G4_IO1;                    /*!<GROUP4_IO1 sampling mode */
        __IO uint32_t G4_IO2;                    /*!<GROUP4_IO2 sampling mode */
        __IO uint32_t G4_IO3;                    /*!<GROUP4_IO3 sampling mode */
        __IO uint32_t G4_IO4;                    /*!<GROUP4_IO4 sampling mode */
        __IO uint32_t G5_IO1;                    /*!<GROUP5_IO1 sampling mode */
        __IO uint32_t G5_IO2;                    /*!<GROUP5_IO2 sampling mode */
        __IO uint32_t G5_IO3;                    /*!<GROUP5_IO3 sampling mode */
        __IO uint32_t G5_IO4;                    /*!<GROUP5_IO4 sampling mode */
        __IO uint32_t G6_IO1;                    /*!<GROUP6_IO1 sampling mode */
        __IO uint32_t G6_IO2;                    /*!<GROUP6_IO2 sampling mode */
        __IO uint32_t G6_IO3;                    /*!<GROUP6_IO3 sampling mode */
        __IO uint32_t G6_IO4;                    /*!<GROUP6_IO4 sampling mode */
        __IO uint32_t G7_IO1;                    /*!<GROUP7_IO1 sampling mode */
        __IO uint32_t G7_IO2;                    /*!<GROUP7_IO2 sampling mode */
        __IO uint32_t G7_IO3;                    /*!<GROUP7_IO3 sampling mode */
        __IO uint32_t G7_IO4;                    /*!<GROUP7_IO4 sampling mode */
        __IO uint32_t G8_IO1;                    /*!<GROUP8_IO1 sampling mode */
        __IO uint32_t G8_IO2;                    /*!<GROUP8_IO2 sampling mode */
        __IO uint32_t G8_IO3;                    /*!<GROUP8_IO3 sampling mode */
        __IO uint32_t G8_IO4;                    /*!<GROUP8_IO4 sampling mode */
    } IOSCR;                                 /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
         uint32_t __RESERVED2[32];           /*!< Reserved,                                                 Address offset: 0x24 */
    struct {
        __IO uint32_t G1_IO1;                    /*!<GROUP1_IO1 channel mode */
        __IO uint32_t G1_IO2;                    /*!<GROUP1_IO2 channel mode */
        __IO uint32_t G1_IO3;                    /*!<GROUP1_IO3 channel mode */
        __IO uint32_t G1_IO4;                    /*!<GROUP1_IO4 channel mode */
        __IO uint32_t G2_IO1;                    /*!<GROUP2_IO1 channel mode */
        __IO uint32_t G2_IO2;                    /*!<GROUP2_IO2 channel mode */
        __IO uint32_t G2_IO3;                    /*!<GROUP2_IO3 channel mode */
        __IO uint32_t G2_IO4;                    /*!<GROUP2_IO4 channel mode */
        __IO uint32_t G3_IO1;                    /*!<GROUP3_IO1 channel mode */
        __IO uint32_t G3_IO2;                    /*!<GROUP3_IO2 channel mode */
        __IO uint32_t G3_IO3;                    /*!<GROUP3_IO3 channel mode */
        __IO uint32_t G3_IO4;                    /*!<GROUP3_IO4 channel mode */
        __IO uint32_t G4_IO1;                    /*!<GROUP4_IO1 channel mode */
        __IO uint32_t G4_IO2;                    /*!<GROUP4_IO2 channel mode */
        __IO uint32_t G4_IO3;                    /*!<GROUP4_IO3 channel mode */
        __IO uint32_t G4_IO4;                    /*!<GROUP4_IO4 channel mode */
        __IO uint32_t G5_IO1;                    /*!<GROUP5_IO1 channel mode */
        __IO uint32_t G5_IO2;                    /*!<GROUP5_IO2 channel mode */
        __IO uint32_t G5_IO3;                    /*!<GROUP5_IO3 channel mode */
        __IO uint32_t G5_IO4;                    /*!<GROUP5_IO4 channel mode */
        __IO uint32_t G6_IO1;                    /*!<GROUP6_IO1 channel mode */
        __IO uint32_t G6_IO2;                    /*!<GROUP6_IO2 channel mode */
        __IO uint32_t G6_IO3;                    /*!<GROUP6_IO3 channel mode */
        __IO uint32_t G6_IO4;                    /*!<GROUP6_IO4 channel mode */
        __IO uint32_t G7_IO1;                    /*!<GROUP7_IO1 channel mode */
        __IO uint32_t G7_IO2;                    /*!<GROUP7_IO2 channel mode */
        __IO uint32_t G7_IO3;                    /*!<GROUP7_IO3 channel mode */
        __IO uint32_t G7_IO4;                    /*!<GROUP7_IO4 channel mode */
        __IO uint32_t G8_IO1;                    /*!<GROUP8_IO1 channel mode */
        __IO uint32_t G8_IO2;                    /*!<GROUP8_IO2 channel mode */
        __IO uint32_t G8_IO3;                    /*!<GROUP8_IO3 channel mode */
        __IO uint32_t G8_IO4;                    /*!<GROUP8_IO4 channel mode */
    } IOCCR;                                 /*!< TSC I/O channel control register,                         Address offset: 0x28 */
         uint32_t __RESERVED3[32];           /*!< Reserved,                                                 Address offset: 0x2C */
    struct {
        __IO uint32_t G1E;                       /*!<Analog IO GROUP1 enable */
        __IO uint32_t G2E;                       /*!<Analog IO GROUP2 enable */
        __IO uint32_t G3E;                       /*!<Analog IO GROUP3 enable */
        __IO uint32_t G4E;                       /*!<Analog IO GROUP4 enable */
        __IO uint32_t G5E;                       /*!<Analog IO GROUP5 enable */
        __IO uint32_t G6E;                       /*!<Analog IO GROUP6 enable */
        __IO uint32_t G7E;                       /*!<Analog IO GROUP7 enable */
        __IO uint32_t G8E;                       /*!<Analog IO GROUP8 enable */
             uint32_t __RESERVED0[8];
        __IO uint32_t G1S;                       /*!<Analog IO GROUP1 status */
        __IO uint32_t G2S;                       /*!<Analog IO GROUP2 status */
        __IO uint32_t G3S;                       /*!<Analog IO GROUP3 status */
        __IO uint32_t G4S;                       /*!<Analog IO GROUP4 status */
        __IO uint32_t G5S;                       /*!<Analog IO GROUP5 status */
        __IO uint32_t G6S;                       /*!<Analog IO GROUP6 status */
        __IO uint32_t G7S;                       /*!<Analog IO GROUP7 status */
        __IO uint32_t G8S;                       /*!<Analog IO GROUP8 status */
             uint32_t __RESERVED1[8];
    } IOGCSR;                                /*!< TSC I/O group control status register,                    Address offset: 0x30 */
    __IO uint32_t IOGXCR[8][32];                /*!< TSC I/O group x counter register,                         Address offset: 0x34-50 */
} TSC_BitBand_TypeDef;



/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */


typedef struct {
    union {
        struct {
            __IO uint32_t UE : 1;                    /*!< USART Enable */
            __IO uint32_t UESM : 1;                  /*!< USART Enable in STOP Mode */
            __IO uint32_t RE : 1;                    /*!< Receiver Enable */
            __IO uint32_t TE : 1;                    /*!< Transmitter Enable */
            __IO uint32_t IDLEIE : 1;                /*!< IDLE Interrupt Enable */
            __IO uint32_t RXNEIE : 1;                /*!< RXNE Interrupt Enable */
            __IO uint32_t TCIE : 1;                  /*!< Transmission Complete Interrupt Enable */
            __IO uint32_t TXEIE : 1;                 /*!< TXE Interrupt Enable */
            __IO uint32_t PEIE : 1;                  /*!< PE Interrupt Enable */
            __IO uint32_t PS : 1;                    /*!< Parity Selection */
            __IO uint32_t PCE : 1;                   /*!< Parity Control Enable */
            __IO uint32_t WAKE : 1;                  /*!< Receiver Wakeup method */
            __IO uint32_t M : 1;                     /*!< Word length */
            __IO uint32_t MME : 1;                   /*!< Mute Mode Enable */
            __IO uint32_t CMIE : 1;                  /*!< Character match interrupt enable */
            __IO uint32_t OVER8 : 1;                 /*!< Oversampling by 8-bit or 16-bit mode */
            __IO uint32_t DEDT : 5;                  /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
            __IO uint32_t DEAT : 5;                  /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
            __IO uint32_t RTOIE : 1;                 /*!< Receive Time Out interrupt enable */
            __IO uint32_t EOBIE : 1;                 /*!< End of Block interrupt enable */
                 uint32_t __RESERVED0 : 4;
        } b;
        __IO uint32_t w;
    } CR1;                                   /*!< USART Control register 1,                 Address offset: 0x00 */
    union {
        struct {
                 uint32_t __RESERVED0 : 4;
            __IO uint32_t ADDM7 : 1;                 /*!< 7-bit or 4-bit Address Detection */
            __IO uint32_t LBDL : 1;                  /*!< LIN Break Detection Length */
            __IO uint32_t LBDIE : 1;                 /*!< LIN Break Detection Interrupt Enable */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t LBCL : 1;                  /*!< Last Bit Clock pulse */
            __IO uint32_t CPHA : 1;                  /*!< Clock Phase */
            __IO uint32_t CPOL : 1;                  /*!< Clock Polarity */
            __IO uint32_t CLKEN : 1;                 /*!< Clock Enable */
            __IO uint32_t STOP : 2;                  /*!< STOP[1:0] bits (STOP bits) */
            __IO uint32_t LINEN : 1;                 /*!< LIN mode enable */
            __IO uint32_t SWAP : 1;                  /*!< SWAP TX/RX pins */
            __IO uint32_t RXINV : 1;                 /*!< RX pin active level inversion */
            __IO uint32_t TXINV : 1;                 /*!< TX pin active level inversion */
            __IO uint32_t DATAINV : 1;               /*!< Binary data inversion */
            __IO uint32_t MSBFIRST : 1;              /*!< Most Significant Bit First */
            __IO uint32_t ABREN : 1;                 /*!< Auto Baud-Rate Enable*/
            __IO uint32_t ABRMODE : 2;               /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
            __IO uint32_t RTOEN : 1;                 /*!< Receiver Time-Out enable */
            __IO uint32_t ADD : 8;                   /*!< Address of the USART node */
        } b;
        __IO uint32_t w;
    } CR2;                                   /*!< USART Control register 2,                 Address offset: 0x04 */
    union {
        struct {
            __IO uint32_t EIE : 1;                   /*!< Error Interrupt Enable */
            __IO uint32_t IREN : 1;                  /*!< IrDA mode Enable */
            __IO uint32_t IRLP : 1;                  /*!< IrDA Low-Power */
            __IO uint32_t HDSEL : 1;                 /*!< Half-Duplex Selection */
            __IO uint32_t NACK : 1;                  /*!< SmartCard NACK enable */
            __IO uint32_t SCEN : 1;                  /*!< SmartCard mode enable */
            __IO uint32_t DMAR : 1;                  /*!< DMA Enable Receiver */
            __IO uint32_t DMAT : 1;                  /*!< DMA Enable Transmitter */
            __IO uint32_t RTSE : 1;                  /*!< RTS Enable */
            __IO uint32_t CTSE : 1;                  /*!< CTS Enable */
            __IO uint32_t CTSIE : 1;                 /*!< CTS Interrupt Enable */
            __IO uint32_t ONEBIT : 1;                /*!< One sample bit method enable */
            __IO uint32_t OVRDIS : 1;                /*!< Overrun Disable */
            __IO uint32_t DDRE : 1;                  /*!< DMA Disable on Reception Error */
            __IO uint32_t DEM : 1;                   /*!< Driver Enable Mode */
            __IO uint32_t DEP : 1;                   /*!< Driver Enable Polarity Selection */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t SCARCNT : 3;               /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
            __IO uint32_t WUS : 2;                   /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
            __IO uint32_t WUFIE : 1;                 /*!< Wake Up Interrupt Enable */
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } CR3;                                   /*!< USART Control register 3,                 Address offset: 0x08 */
    union {
        struct {
            __IO uint32_t DIV_FRACTION : 4;          /*!< Fraction of USARTDIV */
            __IO uint32_t DIV_MANTISSA : 12;         /*!< Mantissa of USARTDIV */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } BRR;                                   /*!< USART Baud rate register,                 Address offset: 0x0C */
    union {
        struct {
            __IO uint32_t PSC : 8;                   /*!< PSC[7:0] bits (Prescaler value) */
            __IO uint32_t GT : 8;                    /*!< GT[7:0] bits (Guard time value) */
                 uint32_t __RESERVED0 : 16;
        } b;
        __IO uint32_t w;
    } GTPR;                                  /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    union {
        struct {
            __IO uint32_t RTO : 24;                  /*!< Receiver Time Out Value */
            __IO uint32_t BLEN : 8;                  /*!< Block Length */
        } b;
        __IO uint32_t w;
    } RTOR;                                  /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    union {
        struct {
            __IO uint32_t ABRRQ : 1;                 /*!< Auto-Baud Rate Request */
            __IO uint32_t SBKRQ : 1;                 /*!< Send Break Request */
            __IO uint32_t MMRQ : 1;                  /*!< Mute Mode Request */
            __IO uint32_t RXFRQ : 1;                 /*!< Receive Data flush Request */
            __IO uint32_t TXFRQ : 1;                 /*!< Transmit data flush Request */
                 uint32_t __RESERVED0 : 27;
        } b;
        __IO uint32_t w;
    } RQR;                                   /*!< USART Request register,                   Address offset: 0x18 */
    union {
        struct {
            __IO uint32_t PE : 1;                    /*!< Parity Error */
            __IO uint32_t FE : 1;                    /*!< Framing Error */
            __IO uint32_t NE : 1;                    /*!< Noise detected Flag */
            __IO uint32_t ORE : 1;                   /*!< OverRun Error */
            __IO uint32_t IDLE : 1;                  /*!< IDLE line detected */
            __IO uint32_t RXNE : 1;                  /*!< Read Data Register Not Empty */
            __IO uint32_t TC : 1;                    /*!< Transmission Complete */
            __IO uint32_t TXE : 1;                   /*!< Transmit Data Register Empty */
            __IO uint32_t LBDF : 1;                  /*!< LIN Break Detection Flag */
            __IO uint32_t CTSIF : 1;                 /*!< CTS interrupt flag */
            __IO uint32_t CTS : 1;                   /*!< CTS flag */
            __IO uint32_t RTOF : 1;                  /*!< Receiver Time Out */
            __IO uint32_t EOBF : 1;                  /*!< End Of Block Flag */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t ABRE : 1;                  /*!< Auto-Baud Rate Error */
            __IO uint32_t ABRF : 1;                  /*!< Auto-Baud Rate Flag */
            __IO uint32_t BUSY : 1;                  /*!< Busy Flag */
            __IO uint32_t CMF : 1;                   /*!< Character Match Flag */
            __IO uint32_t SBKF : 1;                  /*!< Send Break Flag */
            __IO uint32_t RWU : 1;                   /*!< Receive Wake Up from mute mode Flag */
            __IO uint32_t WUF : 1;                   /*!< Wake Up from stop mode Flag */
            __IO uint32_t TEACK : 1;                 /*!< Transmit Enable Acknowledge Flag */
            __IO uint32_t REACK : 1;                 /*!< Receive Enable Acknowledge Flag */
                 uint32_t __RESERVED1 : 9;
        } b;
        __IO uint32_t w;
    } ISR;                                   /*!< USART Interrupt and status register,      Address offset: 0x1C */
    union {
        struct {
            __IO uint32_t PECF : 1;                  /*!< Parity Error Clear Flag */
            __IO uint32_t FECF : 1;                  /*!< Framing Error Clear Flag */
            __IO uint32_t NCF : 1;                   /*!< Noise detected Clear Flag */
            __IO uint32_t ORECF : 1;                 /*!< OverRun Error Clear Flag */
            __IO uint32_t IDLECF : 1;                /*!< IDLE line detected Clear Flag */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t TCCF : 1;                  /*!< Transmission Complete Clear Flag */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t LBDCF : 1;                 /*!< LIN Break Detection Clear Flag */
            __IO uint32_t CTSCF : 1;                 /*!< CTS Interrupt Clear Flag */
                 uint32_t __RESERVED2 : 1;
            __IO uint32_t RTOCF : 1;                 /*!< Receiver Time Out Clear Flag */
            __IO uint32_t EOBCF : 1;                 /*!< End Of Block Clear Flag */
                 uint32_t __RESERVED3 : 4;
            __IO uint32_t CMCF : 1;                  /*!< Character Match Clear Flag */
                 uint32_t __RESERVED4 : 2;
            __IO uint32_t WUCF : 1;                  /*!< Wake Up from stop mode Clear Flag */
                 uint32_t __RESERVED5 : 11;
        } b;
        __IO uint32_t w;
    } ICR;                                   /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    __IO uint16_t RDR;                       /*!< USART Receive Data register,              Address offset: 0x24 */
         uint16_t __RESERVED0;               /*!< Reserved, 0x26                                                 */
    __IO uint16_t TDR;                       /*!< USART Transmit Data register,             Address offset: 0x28 */
         uint16_t __RESERVED1;               /*!< Reserved, 0x2A                                                 */
} USART_TypeDef;


typedef struct {
    struct {
        __IO uint32_t UE;                        /*!< USART Enable */
        __IO uint32_t UESM;                      /*!< USART Enable in STOP Mode */
        __IO uint32_t RE;                        /*!< Receiver Enable */
        __IO uint32_t TE;                        /*!< Transmitter Enable */
        __IO uint32_t IDLEIE;                    /*!< IDLE Interrupt Enable */
        __IO uint32_t RXNEIE;                    /*!< RXNE Interrupt Enable */
        __IO uint32_t TCIE;                      /*!< Transmission Complete Interrupt Enable */
        __IO uint32_t TXEIE;                     /*!< TXE Interrupt Enable */
        __IO uint32_t PEIE;                      /*!< PE Interrupt Enable */
        __IO uint32_t PS;                        /*!< Parity Selection */
        __IO uint32_t PCE;                       /*!< Parity Control Enable */
        __IO uint32_t WAKE;                      /*!< Receiver Wakeup method */
        __IO uint32_t M;                         /*!< Word length */
        __IO uint32_t MME;                       /*!< Mute Mode Enable */
        __IO uint32_t CMIE;                      /*!< Character match interrupt enable */
        __IO uint32_t OVER8;                     /*!< Oversampling by 8-bit or 16-bit mode */
        __IO uint32_t DEDT[5];                   /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
        __IO uint32_t DEAT[5];                   /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
        __IO uint32_t RTOIE;                     /*!< Receive Time Out interrupt enable */
        __IO uint32_t EOBIE;                     /*!< End of Block interrupt enable */
             uint32_t __RESERVED0[4];
    } CR1;                                   /*!< USART Control register 1,                 Address offset: 0x00 */
    struct {
             uint32_t __RESERVED0[4];
        __IO uint32_t ADDM7;                     /*!< 7-bit or 4-bit Address Detection */
        __IO uint32_t LBDL;                      /*!< LIN Break Detection Length */
        __IO uint32_t LBDIE;                     /*!< LIN Break Detection Interrupt Enable */
             uint32_t __RESERVED1;
        __IO uint32_t LBCL;                      /*!< Last Bit Clock pulse */
        __IO uint32_t CPHA;                      /*!< Clock Phase */
        __IO uint32_t CPOL;                      /*!< Clock Polarity */
        __IO uint32_t CLKEN;                     /*!< Clock Enable */
        __IO uint32_t STOP[2];                   /*!< STOP[1:0] bits (STOP bits) */
        __IO uint32_t LINEN;                     /*!< LIN mode enable */
        __IO uint32_t SWAP;                      /*!< SWAP TX/RX pins */
        __IO uint32_t RXINV;                     /*!< RX pin active level inversion */
        __IO uint32_t TXINV;                     /*!< TX pin active level inversion */
        __IO uint32_t DATAINV;                   /*!< Binary data inversion */
        __IO uint32_t MSBFIRST;                  /*!< Most Significant Bit First */
        __IO uint32_t ABREN;                     /*!< Auto Baud-Rate Enable*/
        __IO uint32_t ABRMODE[2];                /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
        __IO uint32_t RTOEN;                     /*!< Receiver Time-Out enable */
        __IO uint32_t ADD[8];                    /*!< Address of the USART node */
    } CR2;                                   /*!< USART Control register 2,                 Address offset: 0x04 */
    struct {
        __IO uint32_t EIE;                       /*!< Error Interrupt Enable */
        __IO uint32_t IREN;                      /*!< IrDA mode Enable */
        __IO uint32_t IRLP;                      /*!< IrDA Low-Power */
        __IO uint32_t HDSEL;                     /*!< Half-Duplex Selection */
        __IO uint32_t NACK;                      /*!< SmartCard NACK enable */
        __IO uint32_t SCEN;                      /*!< SmartCard mode enable */
        __IO uint32_t DMAR;                      /*!< DMA Enable Receiver */
        __IO uint32_t DMAT;                      /*!< DMA Enable Transmitter */
        __IO uint32_t RTSE;                      /*!< RTS Enable */
        __IO uint32_t CTSE;                      /*!< CTS Enable */
        __IO uint32_t CTSIE;                     /*!< CTS Interrupt Enable */
        __IO uint32_t ONEBIT;                    /*!< One sample bit method enable */
        __IO uint32_t OVRDIS;                    /*!< Overrun Disable */
        __IO uint32_t DDRE;                      /*!< DMA Disable on Reception Error */
        __IO uint32_t DEM;                       /*!< Driver Enable Mode */
        __IO uint32_t DEP;                       /*!< Driver Enable Polarity Selection */
             uint32_t __RESERVED0;
        __IO uint32_t SCARCNT[3];                /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
        __IO uint32_t WUS[2];                    /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
        __IO uint32_t WUFIE;                     /*!< Wake Up Interrupt Enable */
             uint32_t __RESERVED1[9];
    } CR3;                                   /*!< USART Control register 3,                 Address offset: 0x08 */
    struct {
        __IO uint32_t DIV_FRACTION[4];           /*!< Fraction of USARTDIV */
        __IO uint32_t DIV_MANTISSA[12];          /*!< Mantissa of USARTDIV */
             uint32_t __RESERVED0[16];
    } BRR;                                   /*!< USART Baud rate register,                 Address offset: 0x0C */
    struct {
        __IO uint32_t PSC[8];                    /*!< PSC[7:0] bits (Prescaler value) */
        __IO uint32_t GT[8];                     /*!< GT[7:0] bits (Guard time value) */
             uint32_t __RESERVED0[16];
    } GTPR;                                  /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    struct {
        __IO uint32_t RTO[24];                   /*!< Receiver Time Out Value */
        __IO uint32_t BLEN[8];                   /*!< Block Length */
    } RTOR;                                  /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    struct {
        __IO uint32_t ABRRQ;                     /*!< Auto-Baud Rate Request */
        __IO uint32_t SBKRQ;                     /*!< Send Break Request */
        __IO uint32_t MMRQ;                      /*!< Mute Mode Request */
        __IO uint32_t RXFRQ;                     /*!< Receive Data flush Request */
        __IO uint32_t TXFRQ;                     /*!< Transmit data flush Request */
             uint32_t __RESERVED0[27];
    } RQR;                                   /*!< USART Request register,                   Address offset: 0x18 */
    struct {
        __IO uint32_t PE;                        /*!< Parity Error */
        __IO uint32_t FE;                        /*!< Framing Error */
        __IO uint32_t NE;                        /*!< Noise detected Flag */
        __IO uint32_t ORE;                       /*!< OverRun Error */
        __IO uint32_t IDLE;                      /*!< IDLE line detected */
        __IO uint32_t RXNE;                      /*!< Read Data Register Not Empty */
        __IO uint32_t TC;                        /*!< Transmission Complete */
        __IO uint32_t TXE;                       /*!< Transmit Data Register Empty */
        __IO uint32_t LBDF;                      /*!< LIN Break Detection Flag */
        __IO uint32_t CTSIF;                     /*!< CTS interrupt flag */
        __IO uint32_t CTS;                       /*!< CTS flag */
        __IO uint32_t RTOF;                      /*!< Receiver Time Out */
        __IO uint32_t EOBF;                      /*!< End Of Block Flag */
             uint32_t __RESERVED0;
        __IO uint32_t ABRE;                      /*!< Auto-Baud Rate Error */
        __IO uint32_t ABRF;                      /*!< Auto-Baud Rate Flag */
        __IO uint32_t BUSY;                      /*!< Busy Flag */
        __IO uint32_t CMF;                       /*!< Character Match Flag */
        __IO uint32_t SBKF;                      /*!< Send Break Flag */
        __IO uint32_t RWU;                       /*!< Receive Wake Up from mute mode Flag */
        __IO uint32_t WUF;                       /*!< Wake Up from stop mode Flag */
        __IO uint32_t TEACK;                     /*!< Transmit Enable Acknowledge Flag */
        __IO uint32_t REACK;                     /*!< Receive Enable Acknowledge Flag */
             uint32_t __RESERVED1[9];
    } ISR;                                   /*!< USART Interrupt and status register,      Address offset: 0x1C */
    struct {
        __IO uint32_t PECF;                      /*!< Parity Error Clear Flag */
        __IO uint32_t FECF;                      /*!< Framing Error Clear Flag */
        __IO uint32_t NCF;                       /*!< Noise detected Clear Flag */
        __IO uint32_t ORECF;                     /*!< OverRun Error Clear Flag */
        __IO uint32_t IDLECF;                    /*!< IDLE line detected Clear Flag */
             uint32_t __RESERVED0;
        __IO uint32_t TCCF;                      /*!< Transmission Complete Clear Flag */
             uint32_t __RESERVED1;
        __IO uint32_t LBDCF;                     /*!< LIN Break Detection Clear Flag */
        __IO uint32_t CTSCF;                     /*!< CTS Interrupt Clear Flag */
             uint32_t __RESERVED2;
        __IO uint32_t RTOCF;                     /*!< Receiver Time Out Clear Flag */
        __IO uint32_t EOBCF;                     /*!< End Of Block Clear Flag */
             uint32_t __RESERVED3[4];
        __IO uint32_t CMCF;                      /*!< Character Match Clear Flag */
             uint32_t __RESERVED4[2];
        __IO uint32_t WUCF;                      /*!< Wake Up from stop mode Clear Flag */
             uint32_t __RESERVED5[11];
    } ICR;                                   /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    __IO uint32_t RDR[16];                   /*!< USART Receive Data register,              Address offset: 0x24 */
         uint32_t __RESERVED0[16];           /*!< Reserved, 0x26                                                 */
    __IO uint32_t TDR[16];                   /*!< USART Transmit Data register,             Address offset: 0x28 */
         uint32_t __RESERVED1[16];           /*!< Reserved, 0x2A                                                 */
} USART_BitBand_TypeDef;



/** 
  * @brief Universal Serial Bus Full Speed Device
  */
  

typedef struct {
    union {
        struct {
            __IO uint32_t EA : 4;                    /*!<  EndPoint Address */
            __IO uint32_t STAT_TX : 2;               /*!<  EndPoint TX Status */
            __IO uint32_t DTOG_TX : 1;               /*!<  EndPoint Data TOGGLE TX */
            __IO uint32_t CTR_TX : 1;                /*!<  EndPoint Correct TRansfer TX */
            __IO uint32_t KIND : 1;                  /*!<  EndPoint KIND */
            __IO uint32_t TYPE : 2;                  /*!<  EndPoint TYPE */
            __IO uint32_t SETUP : 1;                 /*!<  EndPoint SETUP */
            __IO uint32_t STAT_RX : 2;               /*!<  EndPoint RX Status */
            __IO uint32_t DTOG_RX : 1;               /*!<  EndPoint Data TOGGLE RX */
            __IO uint32_t CTR_RX : 1;                /*!<  EndPoint Correct TRansfer RX */
                 uint32_t __RESERVED1 : 16;
        } b;
        __IO uint32_t w;
    } EP[8];                                     /*!< USB endpoint register */
         uint32_t __RESERVED0[8];
    union {
        struct {
            __IO uint16_t FRES : 1;                  /*!< Force USB RESet */
            __IO uint16_t PDWN : 1;                  /*!< Power DoWN */
            __IO uint16_t LPMODE : 1;                /*!< Low-power MODE */
            __IO uint16_t FSUSP : 1;                 /*!< Force SUSPend */
            __IO uint16_t RESUME : 1;                /*!< RESUME request */
            __IO uint16_t L1RESUME : 1;              /*!< LPM L1 Resume request */
                 uint16_t __RESERVED0 : 1;
            __IO uint16_t L1REQM : 1;                /*!< LPM L1 state request interrupt mask */
            __IO uint16_t ESOFM : 1;                 /*!< Expected Start Of Frame Mask */
            __IO uint16_t SOFM : 1;                  /*!< Start Of Frame Mask */
            __IO uint16_t RESETM : 1;                /*!< RESET Mask   */
            __IO uint16_t SUSPM : 1;                 /*!< SUSPend Mask */
            __IO uint16_t WKUPM : 1;                 /*!< WaKe UP Mask */
            __IO uint16_t ERRM : 1;                  /*!< ERRor Mask */
            __IO uint16_t PMAOVR : 1;                /*!< DMA OVeR/underrun Mask */
            __IO uint16_t CTRM : 1;                  /*!< Correct TRansfer Mask */
        } b;
        __IO uint16_t w;
    } CNTR;                                  /*!< Control register,                       Address offset: 0x40 */
         uint16_t __RESERVED1;
    union {
        struct {
            __IO uint16_t EP_ID : 4;                 /*!< EndPoint IDentifier (read-only bit)  */
            __IO uint16_t DIR : 1;                   /*!< DIRection of transaction (read-only bit)  */
                 uint16_t __RESERVED0 : 2;
            __IO uint16_t L1REQ : 1;                 /*!< LPM L1 state request  */
            __IO uint16_t ESOF : 1;                  /*!< Expected Start Of Frame (clear-only bit) */
            __IO uint16_t SOF : 1;                   /*!< Start Of Frame (clear-only bit) */
            __IO uint16_t RESET : 1;                 /*!< RESET (clear-only bit) */
            __IO uint16_t SUSP : 1;                  /*!< SUSPend (clear-only bit) */
            __IO uint16_t WKUP : 1;                  /*!< WaKe UP (clear-only bit) */
            __IO uint16_t ERR : 1;                   /*!< ERRor (clear-only bit) */
            __IO uint16_t PMAOVR : 1;                /*!< DMA OVeR/underrun (clear-only bit) */
            __IO uint16_t CTR : 1;                   /*!< Correct TRansfer (clear-only bit) */
        } b;
        __IO uint16_t w;
    } ISTR;                                  /*!< Interrupt status register,              Address offset: 0x44 */
         uint16_t __RESERVED2;
    union {
        struct {
            __IO uint16_t FN : 11;                   /*!< Frame Number */
            __IO uint16_t LSOF : 2;                  /*!< Lost SOF */
            __IO uint16_t LCK : 1;                   /*!< LoCKed */
            __IO uint16_t RXDM : 1;                  /*!< status of D- data line */
            __IO uint16_t RXDP : 1;                  /*!< status of D+ data line */
        } b;
        __IO uint16_t w;
    } FNR;                                   /*!< Frame number register,                  Address offset: 0x48 */
         uint16_t __RESERVED3;
    union {
        struct {
            __IO uint16_t ADD : 7;                   /*!< USB device address */
            __IO uint16_t EF : 1;                    /*!< USB device address Enable Function */
                 uint16_t __RESERVED0 : 8;
        } b;
        __IO uint16_t w;
    } DADDR;                                 /*!< Device address register,                Address offset: 0x4C */
         uint16_t __RESERVED4;
    __IO uint16_t BTABLE;                    /*!< Buffer Table address register,          Address offset: 0x50 */
         uint16_t __RESERVED5;
} USB_TypeDef;


typedef struct {
    struct {
        __IO uint32_t EA[4];                     /*!<  EndPoint Address */
        __IO uint32_t STAT_TX[2];                /*!<  EndPoint TX Status */
        __IO uint32_t DTOG_TX;                   /*!<  EndPoint Data TOGGLE TX */
        __IO uint32_t CTR_TX;                    /*!<  EndPoint Correct TRansfer TX */
        __IO uint32_t KIND;                      /*!<  EndPoint KIND */
        __IO uint32_t T_FIELD[2];                /*!<  EndPoint TYPE */
        __IO uint32_t SETUP;                     /*!<  EndPoint SETUP */
        __IO uint32_t STAT_RX[2];                /*!<  EndPoint RX Stauts */
        __IO uint32_t DTOG_RX;                   /*!<  EndPoint Data TOGGLE RX */
        __IO uint32_t CTR_RX;                    /*!<  EndPoint Correct TRansfer RX */
             uint32_t __RESERVED0[16];
    } EP[8];                                     /*!< USB endpoint register */
         uint32_t __RESERVED0[8][32];
    struct {
        __IO uint32_t FRES;                      /*!< Force USB RESet */
        __IO uint32_t PDWN;                      /*!< Power DoWN */
        __IO uint32_t LPMODE;                    /*!< Low-power MODE */
        __IO uint32_t FSUSP;                     /*!< Force SUSPend */
        __IO uint32_t RESUME;                    /*!< RESUME request */
        __IO uint32_t L1RESUME;                  /*!< LPM L1 Resume request */
             uint32_t __RESERVED0;
        __IO uint32_t L1REQM;                    /*!< LPM L1 state request interrupt mask */
        __IO uint32_t ESOFM;                     /*!< Expected Start Of Frame Mask */
        __IO uint32_t SOFM;                      /*!< Start Of Frame Mask */
        __IO uint32_t RESETM;                    /*!< RESET Mask   */
        __IO uint32_t SUSPM;                     /*!< SUSPend Mask */
        __IO uint32_t WKUPM;                     /*!< WaKe UP Mask */
        __IO uint32_t ERRM;                      /*!< ERRor Mask */
        __IO uint32_t PMAOVR;                    /*!< DMA OVeR/underrun Mask */
        __IO uint32_t CTRM;                      /*!< Correct TRansfer Mask */
    } CNTR;                                  /*!< Control register,                       Address offset: 0x40 */
         uint32_t __RESERVED1[16];
    struct {
        __IO uint32_t EP_ID[4];                  /*!< EndPoint IDentifier (read-only bit)  */
        __IO uint32_t DIR;                       /*!< DIRection of transaction (read-only bit)  */
             uint32_t __RESERVED0[2];
        __IO uint32_t L1REQ;                     /*!< LPM L1 state request  */
        __IO uint32_t ESOF;                      /*!< Expected Start Of Frame (clear-only bit) */
        __IO uint32_t SOF;                       /*!< Start Of Frame (clear-only bit) */
        __IO uint32_t RESET;                     /*!< RESET (clear-only bit) */
        __IO uint32_t SUSP;                      /*!< SUSPend (clear-only bit) */
        __IO uint32_t WKUP;                      /*!< WaKe UP (clear-only bit) */
        __IO uint32_t ERR;                       /*!< ERRor (clear-only bit) */
        __IO uint32_t PMAOVR;                    /*!< DMA OVeR/underrun (clear-only bit) */
        __IO uint32_t CTR;                       /*!< Correct TRansfer (clear-only bit) */
    } ISTR;                                  /*!< Interrupt status register,              Address offset: 0x44 */
         uint32_t __RESERVED2[16];
    struct {
        __IO uint32_t FN[11];                    /*!< Frame Number */
        __IO uint32_t LSOF[2];                   /*!< Lost SOF */
        __IO uint32_t LCK;                       /*!< LoCKed */
        __IO uint32_t RXDM;                      /*!< status of D- data line */
        __IO uint32_t RXDP;                      /*!< status of D+ data line */
    } FNR;                                   /*!< Frame number register,                  Address offset: 0x48 */
         uint32_t __RESERVED3[16];
    struct {
        __IO uint32_t ADD[7];                    /*!< USB device address */
        __IO uint32_t EF;                        /*!< USB device address Enable Function */
             uint32_t __RESERVED0[8];
    } DADDR;                                 /*!< Device address register,                Address offset: 0x4C */
         uint32_t __RESERVED4[16];
    __IO uint32_t BTABLE[16];                /*!< Buffer Table address register,          Address offset: 0x50 */
         uint32_t __RESERVED5[16];
} USB_BitBand_TypeDef;



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
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */
#define SRAM_BB_BASE          ((uint32_t)0x22000000) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */


/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000)
#define AHB3PERIPH_BASE       (PERIPH_BASE + 0x10000000)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x00000000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x00000400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x00000800)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x00001000)
#define RTC_BASE              (APB1PERIPH_BASE + 0x00002800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x00002C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x00003000)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x00003400)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x00003800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x00003C00)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x00004000)
#define USART2_BASE           (APB1PERIPH_BASE + 0x00004400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x00004800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x00004C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x00005000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x00005400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x00005800)
#define USB_BASE              (APB1PERIPH_BASE + 0x00005C00) /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR           (APB1PERIPH_BASE + 0x00006000) /*!< USB_IP Packet Memory Area base address */
#define CAN_BASE              (APB1PERIPH_BASE + 0x00006400)
#define PWR_BASE              (APB1PERIPH_BASE + 0x00007000)
#define DAC1_BASE             (APB1PERIPH_BASE + 0x00007400)
#define DAC_BASE               DAC1_BASE

/*!< APB2 peripherals */
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x00000000)
#define COMP1_BASE            (APB2PERIPH_BASE + 0x0000001C)
#define COMP2_BASE            (APB2PERIPH_BASE + 0x00000020)
#define COMP4_BASE            (APB2PERIPH_BASE + 0x00000028)
#define COMP6_BASE            (APB2PERIPH_BASE + 0x00000030)
#define COMP_BASE             COMP1_BASE
#define OPAMP1_BASE           (APB2PERIPH_BASE + 0x00000038)
#define OPAMP2_BASE           (APB2PERIPH_BASE + 0x0000003C)
#define OPAMP_BASE            OPAMP1_BASE
#define EXTI_BASE             (APB2PERIPH_BASE + 0x00000400)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x00002C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x00003000)
#define USART1_BASE           (APB2PERIPH_BASE + 0x00003800)
#define TIM15_BASE            (APB2PERIPH_BASE + 0x00004000)
#define TIM16_BASE            (APB2PERIPH_BASE + 0x00004400)
#define TIM17_BASE            (APB2PERIPH_BASE + 0x00004800)

/*!< AHB1 peripherals */
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x00000000)
#define DMA1_Channel1_BASE    (AHB1PERIPH_BASE + 0x00000008)
#define DMA1_Channel2_BASE    (AHB1PERIPH_BASE + 0x0000001C)
#define DMA1_Channel3_BASE    (AHB1PERIPH_BASE + 0x00000030)
#define DMA1_Channel4_BASE    (AHB1PERIPH_BASE + 0x00000044)
#define DMA1_Channel5_BASE    (AHB1PERIPH_BASE + 0x00000058)
#define DMA1_Channel6_BASE    (AHB1PERIPH_BASE + 0x0000006C)
#define DMA1_Channel7_BASE    (AHB1PERIPH_BASE + 0x00000080)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x00000400)
#define DMA2_Channel1_BASE    (AHB1PERIPH_BASE + 0x00000408)
#define DMA2_Channel2_BASE    (AHB1PERIPH_BASE + 0x0000041C)
#define DMA2_Channel3_BASE    (AHB1PERIPH_BASE + 0x00000430)
#define DMA2_Channel4_BASE    (AHB1PERIPH_BASE + 0x00000444)
#define DMA2_Channel5_BASE    (AHB1PERIPH_BASE + 0x00000458)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x00001000)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x00002000) /*!< Flash registers base address */
#define OB_BASE               ((uint32_t)0x1FFFF800)     /*!< Flash Option Bytes base address */
#define CRC_BASE              (AHB1PERIPH_BASE + 0x00003000)
#define TSC_BASE              (AHB1PERIPH_BASE + 0x00004000)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x00001000)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400)

/*!< AHB3 peripherals */
#define ADC1_BASE             (AHB3PERIPH_BASE + 0x00000000)
#define ADC2_BASE             (AHB3PERIPH_BASE + 0x00000100)
#define ADC1_2_COMMON_BASE    (AHB3PERIPH_BASE + 0x00000300)

#define DBGMCU_BASE          ((uint32_t)0xE0042000) /*!< Debug MCU registers base address */
/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
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
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define CAN                 ((CAN_TypeDef *) CAN_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define DAC1                ((DAC_TypeDef *) DAC1_BASE)
#define COMP                ((COMP_TypeDef *) COMP_BASE)
#define COMP1               ((COMP_TypeDef *) COMP1_BASE)
#define COMP2               ((COMP_TypeDef *) COMP2_BASE)
#define COMP4               ((COMP_TypeDef *) COMP4_BASE)
#define COMP6               ((COMP_TypeDef *) COMP6_BASE)
#define OPAMP1              ((OPAMP_TypeDef *) OPAMP1_BASE)
#define OPAMP               ((OPAMP_TypeDef *) OPAMP_BASE)
#define OPAMP2              ((OPAMP_TypeDef *) OPAMP2_BASE)
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
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
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define TSC                 ((TSC_TypeDef *) TSC_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC1_2_COMMON       ((ADC_Common_TypeDef *) ADC1_2_COMMON_BASE)
#define USB                 ((USB_TypeDef *) USB_BASE)

#define CAN_BB(inst)              ((CAN_BitBand_TypeDef *) PERIPH_BB(inst))
#define COMP_BB(inst)             ((COMP_BitBand_TypeDef *) PERIPH_BB(inst))
#define CRC_BB                    ((CRC_BitBand_TypeDef *) PERIPH_BB(CRC_BASE))
#define DAC_BB(inst)              ((DAC_BitBand_TypeDef *) PERIPH_BB(inst))
#define DMA_Channel_BB(inst)      ((DMA_Channel_BitBand_TypeDef *) PERIPH_BB(inst))
#define DMA_BB(inst)              ((DMA_BitBand_TypeDef *) PERIPH_BB(inst))
#define EXTI_BB                   ((EXTI_BitBand_TypeDef *) PERIPH_BB(EXTI_BASE))
#define FLASH_BB                  ((FLASH_BitBand_TypeDef *) PERIPH_BB(FLASH_R_BASE))
#define I2C_BB(inst)              ((I2C_BitBand_TypeDef *) PERIPH_BB(inst))
#define IWDG_BB                   ((IWDG_BitBand_TypeDef *) PERIPH_BB(IWDG_BASE))
#define OPAMP_BB(inst)            ((OPAMP_BitBand_TypeDef *) PERIPH_BB(inst))
#define PWR_BB                    ((PWR_BitBand_TypeDef *) PERIPH_BB(PWR_BASE))
#define RCC_BB                    ((RCC_BitBand_TypeDef *) PERIPH_BB(RCC_BASE))
#define RTC_BB                    ((RTC_BitBand_TypeDef *) PERIPH_BB(RTC_BASE))
#define SPI_BB(inst)              ((SPI_BitBand_TypeDef *) PERIPH_BB(inst))
#define SYSCFG_BB                 ((SYSCFG_BitBand_TypeDef *) PERIPH_BB(SYSCFG_BASE))
#define TIM_BB(inst)              ((TIM_BitBand_TypeDef *) PERIPH_BB(inst))
#define TSC_BB                    ((TSC_BitBand_TypeDef *) PERIPH_BB(TSC_BASE))
#define USART_BB(inst)            ((USART_BitBand_TypeDef *) PERIPH_BB(inst))
#define USB_BB                    ((USB_BitBand_TypeDef *) PERIPH_BB(USB_BASE))
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
/*                        Analog to Digital Converter SAR (ADC)               */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_ISR register  ********************/
#define ADC_ISR_ADRDY         ((uint32_t)0x00000001) /*!< ADC Ready (ADRDY) flag  */
#define ADC_ISR_EOSMP         ((uint32_t)0x00000002) /*!< ADC End of Sampling flag */
#define ADC_ISR_EOC           ((uint32_t)0x00000004) /*!< ADC End of Regular Conversion flag */
#define ADC_ISR_EOS           ((uint32_t)0x00000008) /*!< ADC End of Regular sequence of Conversions flag */
#define ADC_ISR_OVR           ((uint32_t)0x00000010) /*!< ADC overrun flag */
#define ADC_ISR_JEOC          ((uint32_t)0x00000020) /*!< ADC End of Injected Conversion flag */
#define ADC_ISR_JEOS          ((uint32_t)0x00000040) /*!< ADC End of Injected sequence of Conversions flag */
#define ADC_ISR_AWD1          ((uint32_t)0x00000080) /*!< ADC Analog watchdog 1 flag */
#define ADC_ISR_AWD2          ((uint32_t)0x00000100) /*!< ADC Analog watchdog 2 flag */
#define ADC_ISR_AWD3          ((uint32_t)0x00000200) /*!< ADC Analog watchdog 3 flag */
#define ADC_ISR_JQOVF         ((uint32_t)0x00000400) /*!< ADC Injected Context Queue Overflow flag */

/********************  Bit definition for ADC_IER register  ********************/
#define ADC_IER_ADRDY         ((uint32_t)0x00000001) /*!< ADC Ready (ADRDY) interrupt source */
#define ADC_IER_EOSMP         ((uint32_t)0x00000002) /*!< ADC End of Sampling interrupt source */
#define ADC_IER_EOC           ((uint32_t)0x00000004) /*!< ADC End of Regular Conversion interrupt source */
#define ADC_IER_EOS           ((uint32_t)0x00000008) /*!< ADC End of Regular sequence of Conversions interrupt source */
#define ADC_IER_OVR           ((uint32_t)0x00000010) /*!< ADC overrun interrupt source */
#define ADC_IER_JEOC          ((uint32_t)0x00000020) /*!< ADC End of Injected Conversion interrupt source */
#define ADC_IER_JEOS          ((uint32_t)0x00000040) /*!< ADC End of Injected sequence of Conversions interrupt source */
#define ADC_IER_AWD1          ((uint32_t)0x00000080) /*!< ADC Analog watchdog 1 interrupt source */
#define ADC_IER_AWD2          ((uint32_t)0x00000100) /*!< ADC Analog watchdog 2 interrupt source */
#define ADC_IER_AWD3          ((uint32_t)0x00000200) /*!< ADC Analog watchdog 3 interrupt source */
#define ADC_IER_JQOVF         ((uint32_t)0x00000400) /*!< ADC Injected Context Queue Overflow interrupt source */

/********************  Bit definition for ADC_CR register  ********************/
#define ADC_CR_ADEN          ((uint32_t)0x00000001) /*!< ADC Enable control */
#define ADC_CR_ADDIS         ((uint32_t)0x00000002) /*!< ADC Disable command */
#define ADC_CR_ADSTART       ((uint32_t)0x00000004) /*!< ADC Start of Regular conversion */
#define ADC_CR_JADSTART      ((uint32_t)0x00000008) /*!< ADC Start of injected conversion */
#define ADC_CR_ADSTP         ((uint32_t)0x00000010) /*!< ADC Stop of Regular conversion */
#define ADC_CR_JADSTP        ((uint32_t)0x00000020) /*!< ADC Stop of injected conversion */
#define ADC_CR_ADVREGEN      ((uint32_t)0x30000000) /*!< ADC Voltage regulator Enable */
#define ADC_CR_ADVREGEN_0    ((uint32_t)0x10000000) /*!< ADC ADVREGEN bit 0 */
#define ADC_CR_ADVREGEN_1    ((uint32_t)0x20000000) /*!< ADC ADVREGEN bit 1 */
#define ADC_CR_ADCALDIF      ((uint32_t)0x40000000) /*!< ADC Differential Mode for calibration */
#define ADC_CR_ADCAL         ((uint32_t)0x80000000) /*!< ADC Calibration */

/********************  Bit definition for ADC_CFGR register  ********************/
#define ADC_CFGR_DMAEN     ((uint32_t)0x00000001) /*!< ADC DMA Enable */
#define ADC_CFGR_DMACFG    ((uint32_t)0x00000002) /*!< ADC DMA configuration */

#define ADC_CFGR_RES       ((uint32_t)0x00000018) /*!< ADC Data resolution */
#define ADC_CFGR_RES_0     ((uint32_t)0x00000008) /*!< ADC RES bit 0 */
#define ADC_CFGR_RES_1     ((uint32_t)0x00000010) /*!< ADC RES bit 1 */

#define ADC_CFGR_ALIGN     ((uint32_t)0x00000020) /*!< ADC Data Alignement */

#define ADC_CFGR_EXTSEL   ((uint32_t)0x000003C0) /*!< ADC External trigger selection for regular group */
#define ADC_CFGR_EXTSEL_0 ((uint32_t)0x00000040) /*!< ADC EXTSEL bit 0 */
#define ADC_CFGR_EXTSEL_1 ((uint32_t)0x00000080) /*!< ADC EXTSEL bit 1 */
#define ADC_CFGR_EXTSEL_2 ((uint32_t)0x00000100) /*!< ADC EXTSEL bit 2 */
#define ADC_CFGR_EXTSEL_3 ((uint32_t)0x00000200) /*!< ADC EXTSEL bit 3 */

#define ADC_CFGR_EXTEN     ((uint32_t)0x00000C00) /*!< ADC External trigger enable and polarity selection for regular channels */
#define ADC_CFGR_EXTEN_0   ((uint32_t)0x00000400) /*!< ADC EXTEN bit 0 */
#define ADC_CFGR_EXTEN_1   ((uint32_t)0x00000800) /*!< ADC EXTEN bit 1 */

#define ADC_CFGR_OVRMOD    ((uint32_t)0x00001000) /*!< ADC overrun mode */
#define ADC_CFGR_CONT      ((uint32_t)0x00002000) /*!< ADC Single/continuous conversion mode for regular conversion */
#define ADC_CFGR_AUTDLY   ((uint32_t)0x00004000) /*!< ADC Delayed conversion mode */
#define ADC_CFGR_AUTOFF    ((uint32_t)0x00008000) /*!< ADC Auto power OFF */
#define ADC_CFGR_DISCEN    ((uint32_t)0x00010000) /*!< ADC Discontinuous mode for regular channels */

#define ADC_CFGR_DISCNUM   ((uint32_t)0x000E0000) /*!< ADC Discontinuous mode channel count */
#define ADC_CFGR_DISCNUM_0 ((uint32_t)0x00020000) /*!< ADC DISCNUM bit 0 */
#define ADC_CFGR_DISCNUM_1 ((uint32_t)0x00040000) /*!< ADC DISCNUM bit 1 */
#define ADC_CFGR_DISCNUM_2 ((uint32_t)0x00080000) /*!< ADC DISCNUM bit 2 */

#define ADC_CFGR_JDISCEN   ((uint32_t)0x00100000) /*!< ADC Discontinous mode on injected channels */
#define ADC_CFGR_JQM       ((uint32_t)0x00200000) /*!< ADC JSQR Queue mode */
#define ADC_CFGR_AWD1SGL   ((uint32_t)0x00400000) /*!< Eanble the watchdog 1 on a single channel or on all channels */
#define ADC_CFGR_AWD1EN    ((uint32_t)0x00800000) /*!< ADC Analog watchdog 1 enable on regular Channels */
#define ADC_CFGR_JAWD1EN   ((uint32_t)0x01000000) /*!< ADC Analog watchdog 1 enable on injected Channels */
#define ADC_CFGR_JAUTO     ((uint32_t)0x02000000) /*!< ADC Automatic injected group conversion */

#define ADC_CFGR_AWD1CH    ((uint32_t)0x7C000000) /*!< ADC Analog watchdog 1 Channel selection */
#define ADC_CFGR_AWD1CH_0  ((uint32_t)0x04000000) /*!< ADC AWD1CH bit 0 */
#define ADC_CFGR_AWD1CH_1  ((uint32_t)0x08000000) /*!< ADC AWD1CH bit 1  */
#define ADC_CFGR_AWD1CH_2  ((uint32_t)0x10000000) /*!< ADC AWD1CH bit 2  */
#define ADC_CFGR_AWD1CH_3  ((uint32_t)0x20000000) /*!< ADC AWD1CH bit 3  */
#define ADC_CFGR_AWD1CH_4  ((uint32_t)0x40000000) /*!< ADC AWD1CH bit 4  */

/********************  Bit definition for ADC_SMPR1 register  ********************/
#define ADC_SMPR1_SMP0     ((uint32_t)0x00000007) /*!< ADC Channel 0 Sampling time selection  */
#define ADC_SMPR1_SMP0_0   ((uint32_t)0x00000001) /*!< ADC SMP0 bit 0 */
#define ADC_SMPR1_SMP0_1   ((uint32_t)0x00000002) /*!< ADC SMP0 bit 1 */
#define ADC_SMPR1_SMP0_2   ((uint32_t)0x00000004) /*!< ADC SMP0 bit 2 */

#define ADC_SMPR1_SMP1     ((uint32_t)0x00000038) /*!< ADC Channel 1 Sampling time selection  */
#define ADC_SMPR1_SMP1_0   ((uint32_t)0x00000008) /*!< ADC SMP1 bit 0 */
#define ADC_SMPR1_SMP1_1   ((uint32_t)0x00000010) /*!< ADC SMP1 bit 1 */
#define ADC_SMPR1_SMP1_2   ((uint32_t)0x00000020) /*!< ADC SMP1 bit 2 */

#define ADC_SMPR1_SMP2     ((uint32_t)0x000001C0) /*!< ADC Channel 2 Sampling time selection  */
#define ADC_SMPR1_SMP2_0   ((uint32_t)0x00000040) /*!< ADC SMP2 bit 0 */
#define ADC_SMPR1_SMP2_1   ((uint32_t)0x00000080) /*!< ADC SMP2 bit 1 */
#define ADC_SMPR1_SMP2_2   ((uint32_t)0x00000100) /*!< ADC SMP2 bit 2 */

#define ADC_SMPR1_SMP3     ((uint32_t)0x00000E00) /*!< ADC Channel 3 Sampling time selection  */
#define ADC_SMPR1_SMP3_0   ((uint32_t)0x00000200) /*!< ADC SMP3 bit 0 */
#define ADC_SMPR1_SMP3_1   ((uint32_t)0x00000400) /*!< ADC SMP3 bit 1 */
#define ADC_SMPR1_SMP3_2   ((uint32_t)0x00000800) /*!< ADC SMP3 bit 2 */

#define ADC_SMPR1_SMP4     ((uint32_t)0x00007000) /*!< ADC Channel 4 Sampling time selection  */
#define ADC_SMPR1_SMP4_0   ((uint32_t)0x00001000) /*!< ADC SMP4 bit 0 */
#define ADC_SMPR1_SMP4_1   ((uint32_t)0x00002000) /*!< ADC SMP4 bit 1 */
#define ADC_SMPR1_SMP4_2   ((uint32_t)0x00004000) /*!< ADC SMP4 bit 2 */

#define ADC_SMPR1_SMP5     ((uint32_t)0x00038000) /*!< ADC Channel 5 Sampling time selection  */
#define ADC_SMPR1_SMP5_0   ((uint32_t)0x00008000) /*!< ADC SMP5 bit 0 */
#define ADC_SMPR1_SMP5_1   ((uint32_t)0x00010000) /*!< ADC SMP5 bit 1 */
#define ADC_SMPR1_SMP5_2   ((uint32_t)0x00020000) /*!< ADC SMP5 bit 2 */

#define ADC_SMPR1_SMP6     ((uint32_t)0x001C0000) /*!< ADC Channel 6 Sampling time selection  */
#define ADC_SMPR1_SMP6_0   ((uint32_t)0x00040000) /*!< ADC SMP6 bit 0 */
#define ADC_SMPR1_SMP6_1   ((uint32_t)0x00080000) /*!< ADC SMP6 bit 1 */
#define ADC_SMPR1_SMP6_2   ((uint32_t)0x00100000) /*!< ADC SMP6 bit 2 */

#define ADC_SMPR1_SMP7     ((uint32_t)0x00E00000) /*!< ADC Channel 7 Sampling time selection  */
#define ADC_SMPR1_SMP7_0   ((uint32_t)0x00200000) /*!< ADC SMP7 bit 0 */
#define ADC_SMPR1_SMP7_1   ((uint32_t)0x00400000) /*!< ADC SMP7 bit 1 */
#define ADC_SMPR1_SMP7_2   ((uint32_t)0x00800000) /*!< ADC SMP7 bit 2 */

#define ADC_SMPR1_SMP8     ((uint32_t)0x07000000) /*!< ADC Channel 8 Sampling time selection  */
#define ADC_SMPR1_SMP8_0   ((uint32_t)0x01000000) /*!< ADC SMP8 bit 0 */
#define ADC_SMPR1_SMP8_1   ((uint32_t)0x02000000) /*!< ADC SMP8 bit 1 */
#define ADC_SMPR1_SMP8_2   ((uint32_t)0x04000000) /*!< ADC SMP8 bit 2 */

#define ADC_SMPR1_SMP9     ((uint32_t)0x38000000) /*!< ADC Channel 9 Sampling time selection  */
#define ADC_SMPR1_SMP9_0   ((uint32_t)0x08000000) /*!< ADC SMP9 bit 0 */
#define ADC_SMPR1_SMP9_1   ((uint32_t)0x10000000) /*!< ADC SMP9 bit 1 */
#define ADC_SMPR1_SMP9_2   ((uint32_t)0x20000000) /*!< ADC SMP9 bit 2 */

/********************  Bit definition for ADC_SMPR2 register  ********************/
#define ADC_SMPR2_SMP10     ((uint32_t)0x00000007) /*!< ADC Channel 10 Sampling time selection  */
#define ADC_SMPR2_SMP10_0   ((uint32_t)0x00000001) /*!< ADC SMP10 bit 0 */
#define ADC_SMPR2_SMP10_1   ((uint32_t)0x00000002) /*!< ADC SMP10 bit 1 */
#define ADC_SMPR2_SMP10_2   ((uint32_t)0x00000004) /*!< ADC SMP10 bit 2 */

#define ADC_SMPR2_SMP11     ((uint32_t)0x00000038) /*!< ADC Channel 11 Sampling time selection  */
#define ADC_SMPR2_SMP11_0   ((uint32_t)0x00000008) /*!< ADC SMP11 bit 0 */
#define ADC_SMPR2_SMP11_1   ((uint32_t)0x00000010) /*!< ADC SMP11 bit 1 */
#define ADC_SMPR2_SMP11_2   ((uint32_t)0x00000020) /*!< ADC SMP11 bit 2 */

#define ADC_SMPR2_SMP12     ((uint32_t)0x000001C0) /*!< ADC Channel 12 Sampling time selection  */
#define ADC_SMPR2_SMP12_0   ((uint32_t)0x00000040) /*!< ADC SMP12 bit 0 */
#define ADC_SMPR2_SMP12_1   ((uint32_t)0x00000080) /*!< ADC SMP12 bit 1 */
#define ADC_SMPR2_SMP12_2   ((uint32_t)0x00000100) /*!< ADC SMP12 bit 2 */

#define ADC_SMPR2_SMP13     ((uint32_t)0x00000E00) /*!< ADC Channel 13 Sampling time selection  */
#define ADC_SMPR2_SMP13_0   ((uint32_t)0x00000200) /*!< ADC SMP13 bit 0 */
#define ADC_SMPR2_SMP13_1   ((uint32_t)0x00000400) /*!< ADC SMP13 bit 1 */
#define ADC_SMPR2_SMP13_2   ((uint32_t)0x00000800) /*!< ADC SMP13 bit 2 */

#define ADC_SMPR2_SMP14     ((uint32_t)0x00007000) /*!< ADC Channel 14 Sampling time selection  */
#define ADC_SMPR2_SMP14_0   ((uint32_t)0x00001000) /*!< ADC SMP14 bit 0 */
#define ADC_SMPR2_SMP14_1   ((uint32_t)0x00002000) /*!< ADC SMP14 bit 1 */
#define ADC_SMPR2_SMP14_2   ((uint32_t)0x00004000) /*!< ADC SMP14 bit 2 */

#define ADC_SMPR2_SMP15     ((uint32_t)0x00038000) /*!< ADC Channel 15 Sampling time selection  */
#define ADC_SMPR2_SMP15_0   ((uint32_t)0x00008000) /*!< ADC SMP15 bit 0 */
#define ADC_SMPR2_SMP15_1   ((uint32_t)0x00010000) /*!< ADC SMP15 bit 1 */
#define ADC_SMPR2_SMP15_2   ((uint32_t)0x00020000) /*!< ADC SMP15 bit 2 */

#define ADC_SMPR2_SMP16     ((uint32_t)0x001C0000) /*!< ADC Channel 16 Sampling time selection  */
#define ADC_SMPR2_SMP16_0   ((uint32_t)0x00040000) /*!< ADC SMP16 bit 0 */
#define ADC_SMPR2_SMP16_1   ((uint32_t)0x00080000) /*!< ADC SMP16 bit 1 */
#define ADC_SMPR2_SMP16_2   ((uint32_t)0x00100000) /*!< ADC SMP16 bit 2 */

#define ADC_SMPR2_SMP17     ((uint32_t)0x00E00000) /*!< ADC Channel 17 Sampling time selection  */
#define ADC_SMPR2_SMP17_0   ((uint32_t)0x00200000) /*!< ADC SMP17 bit 0 */
#define ADC_SMPR2_SMP17_1   ((uint32_t)0x00400000) /*!< ADC SMP17 bit 1 */
#define ADC_SMPR2_SMP17_2   ((uint32_t)0x00800000) /*!< ADC SMP17 bit 2 */

#define ADC_SMPR2_SMP18     ((uint32_t)0x07000000) /*!< ADC Channel 18 Sampling time selection  */
#define ADC_SMPR2_SMP18_0   ((uint32_t)0x01000000) /*!< ADC SMP18 bit 0 */
#define ADC_SMPR2_SMP18_1   ((uint32_t)0x02000000) /*!< ADC SMP18 bit 1 */
#define ADC_SMPR2_SMP18_2   ((uint32_t)0x04000000) /*!< ADC SMP18 bit 2 */

/********************  Bit definition for ADC_TR1 register  ********************/
#define ADC_TR1_LT1         ((uint32_t)0x00000FFF) /*!< ADC Analog watchdog 1 lower threshold */
#define ADC_TR1_LT1_0       ((uint32_t)0x00000001) /*!< ADC LT1 bit 0 */
#define ADC_TR1_LT1_1       ((uint32_t)0x00000002) /*!< ADC LT1 bit 1 */
#define ADC_TR1_LT1_2       ((uint32_t)0x00000004) /*!< ADC LT1 bit 2 */
#define ADC_TR1_LT1_3       ((uint32_t)0x00000008) /*!< ADC LT1 bit 3 */
#define ADC_TR1_LT1_4       ((uint32_t)0x00000010) /*!< ADC LT1 bit 4 */
#define ADC_TR1_LT1_5       ((uint32_t)0x00000020) /*!< ADC LT1 bit 5 */
#define ADC_TR1_LT1_6       ((uint32_t)0x00000040) /*!< ADC LT1 bit 6 */
#define ADC_TR1_LT1_7       ((uint32_t)0x00000080) /*!< ADC LT1 bit 7 */
#define ADC_TR1_LT1_8       ((uint32_t)0x00000100) /*!< ADC LT1 bit 8 */
#define ADC_TR1_LT1_9       ((uint32_t)0x00000200) /*!< ADC LT1 bit 9 */
#define ADC_TR1_LT1_10      ((uint32_t)0x00000400) /*!< ADC LT1 bit 10 */
#define ADC_TR1_LT1_11      ((uint32_t)0x00000800) /*!< ADC LT1 bit 11 */

#define ADC_TR1_HT1         ((uint32_t)0x0FFF0000) /*!< ADC Analog watchdog 1 higher threshold */
#define ADC_TR1_HT1_0       ((uint32_t)0x00010000) /*!< ADC HT1 bit 0 */
#define ADC_TR1_HT1_1       ((uint32_t)0x00020000) /*!< ADC HT1 bit 1 */
#define ADC_TR1_HT1_2       ((uint32_t)0x00040000) /*!< ADC HT1 bit 2 */
#define ADC_TR1_HT1_3       ((uint32_t)0x00080000) /*!< ADC HT1 bit 3 */
#define ADC_TR1_HT1_4       ((uint32_t)0x00100000) /*!< ADC HT1 bit 4 */
#define ADC_TR1_HT1_5       ((uint32_t)0x00200000) /*!< ADC HT1 bit 5 */
#define ADC_TR1_HT1_6       ((uint32_t)0x00400000) /*!< ADC HT1 bit 6 */
#define ADC_TR1_HT1_7       ((uint32_t)0x00800000) /*!< ADC HT1 bit 7 */
#define ADC_TR1_HT1_8       ((uint32_t)0x01000000) /*!< ADC HT1 bit 8 */
#define ADC_TR1_HT1_9       ((uint32_t)0x02000000) /*!< ADC HT1 bit 9 */
#define ADC_TR1_HT1_10      ((uint32_t)0x04000000) /*!< ADC HT1 bit 10 */
#define ADC_TR1_HT1_11      ((uint32_t)0x08000000) /*!< ADC HT1 bit 11 */

/********************  Bit definition for ADC_TR2 register  ********************/
#define ADC_TR2_LT2         ((uint32_t)0x000000FF) /*!< ADC Analog watchdog 2 lower threshold */
#define ADC_TR2_LT2_0       ((uint32_t)0x00000001) /*!< ADC LT2 bit 0 */
#define ADC_TR2_LT2_1       ((uint32_t)0x00000002) /*!< ADC LT2 bit 1 */
#define ADC_TR2_LT2_2       ((uint32_t)0x00000004) /*!< ADC LT2 bit 2 */
#define ADC_TR2_LT2_3       ((uint32_t)0x00000008) /*!< ADC LT2 bit 3 */
#define ADC_TR2_LT2_4       ((uint32_t)0x00000010) /*!< ADC LT2 bit 4 */
#define ADC_TR2_LT2_5       ((uint32_t)0x00000020) /*!< ADC LT2 bit 5 */
#define ADC_TR2_LT2_6       ((uint32_t)0x00000040) /*!< ADC LT2 bit 6 */
#define ADC_TR2_LT2_7       ((uint32_t)0x00000080) /*!< ADC LT2 bit 7 */

#define ADC_TR2_HT2         ((uint32_t)0x00FF0000) /*!< ADC Analog watchdog 2 higher threshold */
#define ADC_TR2_HT2_0       ((uint32_t)0x00010000) /*!< ADC HT2 bit 0 */
#define ADC_TR2_HT2_1       ((uint32_t)0x00020000) /*!< ADC HT2 bit 1 */
#define ADC_TR2_HT2_2       ((uint32_t)0x00040000) /*!< ADC HT2 bit 2 */
#define ADC_TR2_HT2_3       ((uint32_t)0x00080000) /*!< ADC HT2 bit 3 */
#define ADC_TR2_HT2_4       ((uint32_t)0x00100000) /*!< ADC HT2 bit 4 */
#define ADC_TR2_HT2_5       ((uint32_t)0x00200000) /*!< ADC HT2 bit 5 */
#define ADC_TR2_HT2_6       ((uint32_t)0x00400000) /*!< ADC HT2 bit 6 */
#define ADC_TR2_HT2_7       ((uint32_t)0x00800000) /*!< ADC HT2 bit 7 */

/********************  Bit definition for ADC_TR3 register  ********************/
#define ADC_TR3_LT3         ((uint32_t)0x000000FF) /*!< ADC Analog watchdog 3 lower threshold */
#define ADC_TR3_LT3_0       ((uint32_t)0x00000001) /*!< ADC LT3 bit 0 */
#define ADC_TR3_LT3_1       ((uint32_t)0x00000002) /*!< ADC LT3 bit 1 */
#define ADC_TR3_LT3_2       ((uint32_t)0x00000004) /*!< ADC LT3 bit 2 */
#define ADC_TR3_LT3_3       ((uint32_t)0x00000008) /*!< ADC LT3 bit 3 */
#define ADC_TR3_LT3_4       ((uint32_t)0x00000010) /*!< ADC LT3 bit 4 */
#define ADC_TR3_LT3_5       ((uint32_t)0x00000020) /*!< ADC LT3 bit 5 */
#define ADC_TR3_LT3_6       ((uint32_t)0x00000040) /*!< ADC LT3 bit 6 */
#define ADC_TR3_LT3_7       ((uint32_t)0x00000080) /*!< ADC LT3 bit 7 */

#define ADC_TR3_HT3         ((uint32_t)0x00FF0000) /*!< ADC Analog watchdog 3 higher threshold */
#define ADC_TR3_HT3_0       ((uint32_t)0x00010000) /*!< ADC HT3 bit 0 */
#define ADC_TR3_HT3_1       ((uint32_t)0x00020000) /*!< ADC HT3 bit 1 */
#define ADC_TR3_HT3_2       ((uint32_t)0x00040000) /*!< ADC HT3 bit 2 */
#define ADC_TR3_HT3_3       ((uint32_t)0x00080000) /*!< ADC HT3 bit 3 */
#define ADC_TR3_HT3_4       ((uint32_t)0x00100000) /*!< ADC HT3 bit 4 */
#define ADC_TR3_HT3_5       ((uint32_t)0x00200000) /*!< ADC HT3 bit 5 */
#define ADC_TR3_HT3_6       ((uint32_t)0x00400000) /*!< ADC HT3 bit 6 */
#define ADC_TR3_HT3_7       ((uint32_t)0x00800000) /*!< ADC HT3 bit 7 */

/********************  Bit definition for ADC_SQR1 register  ********************/
#define ADC_SQR1_L          ((uint32_t)0x0000000F) /*!< ADC regular channel sequence lenght */
#define ADC_SQR1_L_0        ((uint32_t)0x00000001) /*!< ADC L bit 0 */
#define ADC_SQR1_L_1        ((uint32_t)0x00000002) /*!< ADC L bit 1 */
#define ADC_SQR1_L_2        ((uint32_t)0x00000004) /*!< ADC L bit 2 */
#define ADC_SQR1_L_3        ((uint32_t)0x00000008) /*!< ADC L bit 3 */

#define ADC_SQR1_SQ1        ((uint32_t)0x000007C0) /*!< ADC 1st conversion in regular sequence */
#define ADC_SQR1_SQ1_0      ((uint32_t)0x00000040) /*!< ADC SQ1 bit 0 */
#define ADC_SQR1_SQ1_1      ((uint32_t)0x00000080) /*!< ADC SQ1 bit 1 */
#define ADC_SQR1_SQ1_2      ((uint32_t)0x00000100) /*!< ADC SQ1 bit 2 */
#define ADC_SQR1_SQ1_3      ((uint32_t)0x00000200) /*!< ADC SQ1 bit 3 */
#define ADC_SQR1_SQ1_4      ((uint32_t)0x00000400) /*!< ADC SQ1 bit 4 */

#define ADC_SQR1_SQ2        ((uint32_t)0x0001F000) /*!< ADC 2nd conversion in regular sequence */
#define ADC_SQR1_SQ2_0      ((uint32_t)0x00001000) /*!< ADC SQ2 bit 0 */
#define ADC_SQR1_SQ2_1      ((uint32_t)0x00002000) /*!< ADC SQ2 bit 1 */
#define ADC_SQR1_SQ2_2      ((uint32_t)0x00004000) /*!< ADC SQ2 bit 2 */
#define ADC_SQR1_SQ2_3      ((uint32_t)0x00008000) /*!< ADC SQ2 bit 3 */
#define ADC_SQR1_SQ2_4      ((uint32_t)0x00010000) /*!< ADC SQ2 bit 4 */

#define ADC_SQR1_SQ3        ((uint32_t)0x007C0000) /*!< ADC 3rd conversion in regular sequence */
#define ADC_SQR1_SQ3_0      ((uint32_t)0x00040000) /*!< ADC SQ3 bit 0 */
#define ADC_SQR1_SQ3_1      ((uint32_t)0x00080000) /*!< ADC SQ3 bit 1 */
#define ADC_SQR1_SQ3_2      ((uint32_t)0x00100000) /*!< ADC SQ3 bit 2 */
#define ADC_SQR1_SQ3_3      ((uint32_t)0x00200000) /*!< ADC SQ3 bit 3 */
#define ADC_SQR1_SQ3_4      ((uint32_t)0x00400000) /*!< ADC SQ3 bit 4 */

#define ADC_SQR1_SQ4        ((uint32_t)0x1F000000) /*!< ADC 4th conversion in regular sequence */
#define ADC_SQR1_SQ4_0      ((uint32_t)0x01000000) /*!< ADC SQ4 bit 0 */
#define ADC_SQR1_SQ4_1      ((uint32_t)0x02000000) /*!< ADC SQ4 bit 1 */
#define ADC_SQR1_SQ4_2      ((uint32_t)0x04000000) /*!< ADC SQ4 bit 2 */
#define ADC_SQR1_SQ4_3      ((uint32_t)0x08000000) /*!< ADC SQ4 bit 3 */
#define ADC_SQR1_SQ4_4      ((uint32_t)0x10000000) /*!< ADC SQ4 bit 4 */

/********************  Bit definition for ADC_SQR2 register  ********************/
#define ADC_SQR2_SQ5        ((uint32_t)0x0000001F) /*!< ADC 5th conversion in regular sequence */
#define ADC_SQR2_SQ5_0      ((uint32_t)0x00000001) /*!< ADC SQ5 bit 0 */
#define ADC_SQR2_SQ5_1      ((uint32_t)0x00000002) /*!< ADC SQ5 bit 1 */
#define ADC_SQR2_SQ5_2      ((uint32_t)0x00000004) /*!< ADC SQ5 bit 2 */
#define ADC_SQR2_SQ5_3      ((uint32_t)0x00000008) /*!< ADC SQ5 bit 3 */
#define ADC_SQR2_SQ5_4      ((uint32_t)0x00000010) /*!< ADC SQ5 bit 4 */

#define ADC_SQR2_SQ6        ((uint32_t)0x000007C0) /*!< ADC 6th conversion in regular sequence */
#define ADC_SQR2_SQ6_0      ((uint32_t)0x00000040) /*!< ADC SQ6 bit 0 */
#define ADC_SQR2_SQ6_1      ((uint32_t)0x00000080) /*!< ADC SQ6 bit 1 */
#define ADC_SQR2_SQ6_2      ((uint32_t)0x00000100) /*!< ADC SQ6 bit 2 */
#define ADC_SQR2_SQ6_3      ((uint32_t)0x00000200) /*!< ADC SQ6 bit 3 */
#define ADC_SQR2_SQ6_4      ((uint32_t)0x00000400) /*!< ADC SQ6 bit 4 */

#define ADC_SQR2_SQ7        ((uint32_t)0x0001F000) /*!< ADC 7th conversion in regular sequence */
#define ADC_SQR2_SQ7_0      ((uint32_t)0x00001000) /*!< ADC SQ7 bit 0 */
#define ADC_SQR2_SQ7_1      ((uint32_t)0x00002000) /*!< ADC SQ7 bit 1 */
#define ADC_SQR2_SQ7_2      ((uint32_t)0x00004000) /*!< ADC SQ7 bit 2 */
#define ADC_SQR2_SQ7_3      ((uint32_t)0x00008000) /*!< ADC SQ7 bit 3 */
#define ADC_SQR2_SQ7_4      ((uint32_t)0x00010000) /*!< ADC SQ7 bit 4 */

#define ADC_SQR2_SQ8        ((uint32_t)0x007C0000) /*!< ADC 8th conversion in regular sequence */
#define ADC_SQR2_SQ8_0      ((uint32_t)0x00040000) /*!< ADC SQ8 bit 0 */
#define ADC_SQR2_SQ8_1      ((uint32_t)0x00080000) /*!< ADC SQ8 bit 1 */
#define ADC_SQR2_SQ8_2      ((uint32_t)0x00100000) /*!< ADC SQ8 bit 2 */
#define ADC_SQR2_SQ8_3      ((uint32_t)0x00200000) /*!< ADC SQ8 bit 3 */
#define ADC_SQR2_SQ8_4      ((uint32_t)0x00400000) /*!< ADC SQ8 bit 4 */

#define ADC_SQR2_SQ9        ((uint32_t)0x1F000000) /*!< ADC 9th conversion in regular sequence */
#define ADC_SQR2_SQ9_0      ((uint32_t)0x01000000) /*!< ADC SQ9 bit 0 */
#define ADC_SQR2_SQ9_1      ((uint32_t)0x02000000) /*!< ADC SQ9 bit 1 */
#define ADC_SQR2_SQ9_2      ((uint32_t)0x04000000) /*!< ADC SQ9 bit 2 */
#define ADC_SQR2_SQ9_3      ((uint32_t)0x08000000) /*!< ADC SQ9 bit 3 */
#define ADC_SQR2_SQ9_4      ((uint32_t)0x10000000) /*!< ADC SQ9 bit 4 */

/********************  Bit definition for ADC_SQR3 register  ********************/
#define ADC_SQR3_SQ10       ((uint32_t)0x0000001F) /*!< ADC 10th conversion in regular sequence */
#define ADC_SQR3_SQ10_0     ((uint32_t)0x00000001) /*!< ADC SQ10 bit 0 */
#define ADC_SQR3_SQ10_1     ((uint32_t)0x00000002) /*!< ADC SQ10 bit 1 */
#define ADC_SQR3_SQ10_2     ((uint32_t)0x00000004) /*!< ADC SQ10 bit 2 */
#define ADC_SQR3_SQ10_3     ((uint32_t)0x00000008) /*!< ADC SQ10 bit 3 */
#define ADC_SQR3_SQ10_4     ((uint32_t)0x00000010) /*!< ADC SQ10 bit 4 */

#define ADC_SQR3_SQ11       ((uint32_t)0x000007C0) /*!< ADC 11th conversion in regular sequence */
#define ADC_SQR3_SQ11_0     ((uint32_t)0x00000040) /*!< ADC SQ11 bit 0 */
#define ADC_SQR3_SQ11_1     ((uint32_t)0x00000080) /*!< ADC SQ11 bit 1 */
#define ADC_SQR3_SQ11_2     ((uint32_t)0x00000100) /*!< ADC SQ11 bit 2 */
#define ADC_SQR3_SQ11_3     ((uint32_t)0x00000200) /*!< ADC SQ11 bit 3 */
#define ADC_SQR3_SQ11_4     ((uint32_t)0x00000400) /*!< ADC SQ11 bit 4 */

#define ADC_SQR3_SQ12       ((uint32_t)0x0001F000) /*!< ADC 12th conversion in regular sequence */
#define ADC_SQR3_SQ12_0     ((uint32_t)0x00001000) /*!< ADC SQ12 bit 0 */
#define ADC_SQR3_SQ12_1     ((uint32_t)0x00002000) /*!< ADC SQ12 bit 1 */
#define ADC_SQR3_SQ12_2     ((uint32_t)0x00004000) /*!< ADC SQ12 bit 2 */
#define ADC_SQR3_SQ12_3     ((uint32_t)0x00008000) /*!< ADC SQ12 bit 3 */
#define ADC_SQR3_SQ12_4     ((uint32_t)0x00010000) /*!< ADC SQ12 bit 4 */

#define ADC_SQR3_SQ13       ((uint32_t)0x007C0000) /*!< ADC 13th conversion in regular sequence */
#define ADC_SQR3_SQ13_0     ((uint32_t)0x00040000) /*!< ADC SQ13 bit 0 */
#define ADC_SQR3_SQ13_1     ((uint32_t)0x00080000) /*!< ADC SQ13 bit 1 */
#define ADC_SQR3_SQ13_2     ((uint32_t)0x00100000) /*!< ADC SQ13 bit 2 */
#define ADC_SQR3_SQ13_3     ((uint32_t)0x00200000) /*!< ADC SQ13 bit 3 */
#define ADC_SQR3_SQ13_4     ((uint32_t)0x00400000) /*!< ADC SQ13 bit 4 */

#define ADC_SQR3_SQ14       ((uint32_t)0x1F000000) /*!< ADC 14th conversion in regular sequence */
#define ADC_SQR3_SQ14_0     ((uint32_t)0x01000000) /*!< ADC SQ14 bit 0 */
#define ADC_SQR3_SQ14_1     ((uint32_t)0x02000000) /*!< ADC SQ14 bit 1 */
#define ADC_SQR3_SQ14_2     ((uint32_t)0x04000000) /*!< ADC SQ14 bit 2 */
#define ADC_SQR3_SQ14_3     ((uint32_t)0x08000000) /*!< ADC SQ14 bit 3 */
#define ADC_SQR3_SQ14_4     ((uint32_t)0x10000000) /*!< ADC SQ14 bit 4 */

/********************  Bit definition for ADC_SQR4 register  ********************/
#define ADC_SQR4_SQ15       ((uint32_t)0x0000001F) /*!< ADC 15th conversion in regular sequence */
#define ADC_SQR4_SQ15_0     ((uint32_t)0x00000001) /*!< ADC SQ15 bit 0 */
#define ADC_SQR4_SQ15_1     ((uint32_t)0x00000002) /*!< ADC SQ15 bit 1 */
#define ADC_SQR4_SQ15_2     ((uint32_t)0x00000004) /*!< ADC SQ15 bit 2 */
#define ADC_SQR4_SQ15_3     ((uint32_t)0x00000008) /*!< ADC SQ15 bit 3 */
#define ADC_SQR4_SQ15_4     ((uint32_t)0x00000010) /*!< ADC SQ105 bit 4 */

#define ADC_SQR4_SQ16       ((uint32_t)0x000007C0) /*!< ADC 16th conversion in regular sequence */
#define ADC_SQR4_SQ16_0     ((uint32_t)0x00000040) /*!< ADC SQ16 bit 0 */
#define ADC_SQR4_SQ16_1     ((uint32_t)0x00000080) /*!< ADC SQ16 bit 1 */
#define ADC_SQR4_SQ16_2     ((uint32_t)0x00000100) /*!< ADC SQ16 bit 2 */
#define ADC_SQR4_SQ16_3     ((uint32_t)0x00000200) /*!< ADC SQ16 bit 3 */
#define ADC_SQR4_SQ16_4     ((uint32_t)0x00000400) /*!< ADC SQ16 bit 4 */
/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_RDATA        ((uint32_t)0x0000FFFF) /*!< ADC regular Data converted */
#define ADC_DR_RDATA_0      ((uint32_t)0x00000001) /*!< ADC RDATA bit 0 */
#define ADC_DR_RDATA_1      ((uint32_t)0x00000002) /*!< ADC RDATA bit 1 */
#define ADC_DR_RDATA_2      ((uint32_t)0x00000004) /*!< ADC RDATA bit 2 */
#define ADC_DR_RDATA_3      ((uint32_t)0x00000008) /*!< ADC RDATA bit 3 */
#define ADC_DR_RDATA_4      ((uint32_t)0x00000010) /*!< ADC RDATA bit 4 */
#define ADC_DR_RDATA_5      ((uint32_t)0x00000020) /*!< ADC RDATA bit 5 */
#define ADC_DR_RDATA_6      ((uint32_t)0x00000040) /*!< ADC RDATA bit 6 */
#define ADC_DR_RDATA_7      ((uint32_t)0x00000080) /*!< ADC RDATA bit 7 */
#define ADC_DR_RDATA_8      ((uint32_t)0x00000100) /*!< ADC RDATA bit 8 */
#define ADC_DR_RDATA_9      ((uint32_t)0x00000200) /*!< ADC RDATA bit 9 */
#define ADC_DR_RDATA_10     ((uint32_t)0x00000400) /*!< ADC RDATA bit 10 */
#define ADC_DR_RDATA_11     ((uint32_t)0x00000800) /*!< ADC RDATA bit 11 */
#define ADC_DR_RDATA_12     ((uint32_t)0x00001000) /*!< ADC RDATA bit 12 */
#define ADC_DR_RDATA_13     ((uint32_t)0x00002000) /*!< ADC RDATA bit 13 */
#define ADC_DR_RDATA_14     ((uint32_t)0x00004000) /*!< ADC RDATA bit 14 */
#define ADC_DR_RDATA_15     ((uint32_t)0x00008000) /*!< ADC RDATA bit 15 */

/********************  Bit definition for ADC_JSQR register  ********************/
#define ADC_JSQR_JL         ((uint32_t)0x00000003) /*!< ADC injected channel sequence length */
#define ADC_JSQR_JL_0       ((uint32_t)0x00000001) /*!< ADC JL bit 0 */
#define ADC_JSQR_JL_1       ((uint32_t)0x00000002) /*!< ADC JL bit 1 */

#define ADC_JSQR_JEXTSEL    ((uint32_t)0x0000003C) /*!< ADC external trigger selection for injected group */
#define ADC_JSQR_JEXTSEL_0  ((uint32_t)0x00000004) /*!< ADC JEXTSEL bit 0 */
#define ADC_JSQR_JEXTSEL_1  ((uint32_t)0x00000008) /*!< ADC JEXTSEL bit 1 */
#define ADC_JSQR_JEXTSEL_2  ((uint32_t)0x00000010) /*!< ADC JEXTSEL bit 2 */
#define ADC_JSQR_JEXTSEL_3  ((uint32_t)0x00000020) /*!< ADC JEXTSEL bit 3 */

#define ADC_JSQR_JEXTEN     ((uint32_t)0x000000C0) /*!< ADC external trigger enable and polarity selection for injected channels */
#define ADC_JSQR_JEXTEN_0   ((uint32_t)0x00000040) /*!< ADC JEXTEN bit 0 */
#define ADC_JSQR_JEXTEN_1   ((uint32_t)0x00000080) /*!< ADC JEXTEN bit 1 */

#define ADC_JSQR_JSQ1       ((uint32_t)0x00001F00) /*!< ADC 1st conversion in injected sequence */
#define ADC_JSQR_JSQ1_0     ((uint32_t)0x00000100) /*!< ADC JSQ1 bit 0 */
#define ADC_JSQR_JSQ1_1     ((uint32_t)0x00000200) /*!< ADC JSQ1 bit 1 */
#define ADC_JSQR_JSQ1_2     ((uint32_t)0x00000400) /*!< ADC JSQ1 bit 2 */
#define ADC_JSQR_JSQ1_3     ((uint32_t)0x00000800) /*!< ADC JSQ1 bit 3 */
#define ADC_JSQR_JSQ1_4     ((uint32_t)0x00001000) /*!< ADC JSQ1 bit 4 */

#define ADC_JSQR_JSQ2       ((uint32_t)0x0007C000) /*!< ADC 2nd conversion in injected sequence */
#define ADC_JSQR_JSQ2_0     ((uint32_t)0x00004000) /*!< ADC JSQ2 bit 0 */
#define ADC_JSQR_JSQ2_1     ((uint32_t)0x00008000) /*!< ADC JSQ2 bit 1 */
#define ADC_JSQR_JSQ2_2     ((uint32_t)0x00010000) /*!< ADC JSQ2 bit 2 */
#define ADC_JSQR_JSQ2_3     ((uint32_t)0x00020000) /*!< ADC JSQ2 bit 3 */
#define ADC_JSQR_JSQ2_4     ((uint32_t)0x00040000) /*!< ADC JSQ2 bit 4 */

#define ADC_JSQR_JSQ3       ((uint32_t)0x01F00000) /*!< ADC 3rd conversion in injected sequence */
#define ADC_JSQR_JSQ3_0     ((uint32_t)0x00100000) /*!< ADC JSQ3 bit 0 */
#define ADC_JSQR_JSQ3_1     ((uint32_t)0x00200000) /*!< ADC JSQ3 bit 1 */
#define ADC_JSQR_JSQ3_2     ((uint32_t)0x00400000) /*!< ADC JSQ3 bit 2 */
#define ADC_JSQR_JSQ3_3     ((uint32_t)0x00800000) /*!< ADC JSQ3 bit 3 */
#define ADC_JSQR_JSQ3_4     ((uint32_t)0x01000000) /*!< ADC JSQ3 bit 4 */

#define ADC_JSQR_JSQ4       ((uint32_t)0x7C000000) /*!< ADC 4th conversion in injected sequence */
#define ADC_JSQR_JSQ4_0     ((uint32_t)0x04000000) /*!< ADC JSQ4 bit 0 */
#define ADC_JSQR_JSQ4_1     ((uint32_t)0x08000000) /*!< ADC JSQ4 bit 1 */
#define ADC_JSQR_JSQ4_2     ((uint32_t)0x10000000) /*!< ADC JSQ4 bit 2 */
#define ADC_JSQR_JSQ4_3     ((uint32_t)0x20000000) /*!< ADC JSQ4 bit 3 */
#define ADC_JSQR_JSQ4_4     ((uint32_t)0x40000000) /*!< ADC JSQ4 bit 4 */

/********************  Bit definition for ADC_OFR1 register  ********************/
#define ADC_OFR1_OFFSET1    ((uint32_t)0x00000FFF) /*!< ADC data offset 1 for channel programmed into bits OFFSET1_CH[4:0] */
#define ADC_OFR1_OFFSET1_0  ((uint32_t)0x00000001) /*!< ADC OFFSET1 bit 0 */
#define ADC_OFR1_OFFSET1_1  ((uint32_t)0x00000002) /*!< ADC OFFSET1 bit 1 */
#define ADC_OFR1_OFFSET1_2  ((uint32_t)0x00000004) /*!< ADC OFFSET1 bit 2 */
#define ADC_OFR1_OFFSET1_3  ((uint32_t)0x00000008) /*!< ADC OFFSET1 bit 3 */
#define ADC_OFR1_OFFSET1_4  ((uint32_t)0x00000010) /*!< ADC OFFSET1 bit 4 */
#define ADC_OFR1_OFFSET1_5  ((uint32_t)0x00000020) /*!< ADC OFFSET1 bit 5 */
#define ADC_OFR1_OFFSET1_6  ((uint32_t)0x00000040) /*!< ADC OFFSET1 bit 6 */
#define ADC_OFR1_OFFSET1_7  ((uint32_t)0x00000080) /*!< ADC OFFSET1 bit 7 */
#define ADC_OFR1_OFFSET1_8  ((uint32_t)0x00000100) /*!< ADC OFFSET1 bit 8 */
#define ADC_OFR1_OFFSET1_9  ((uint32_t)0x00000200) /*!< ADC OFFSET1 bit 9 */
#define ADC_OFR1_OFFSET1_10 ((uint32_t)0x00000400) /*!< ADC OFFSET1 bit 10 */
#define ADC_OFR1_OFFSET1_11 ((uint32_t)0x00000800) /*!< ADC OFFSET1 bit 11 */

#define ADC_OFR1_OFFSET1_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 1 */
#define ADC_OFR1_OFFSET1_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET1_CH bit 0 */
#define ADC_OFR1_OFFSET1_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET1_CH bit 1 */
#define ADC_OFR1_OFFSET1_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET1_CH bit 2 */
#define ADC_OFR1_OFFSET1_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET1_CH bit 3 */
#define ADC_OFR1_OFFSET1_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET1_CH bit 4 */

#define ADC_OFR1_OFFSET1_EN ((uint32_t)0x80000000) /*!< ADC offset 1 enable */

/********************  Bit definition for ADC_OFR2 register  ********************/
#define ADC_OFR2_OFFSET2    ((uint32_t)0x00000FFF) /*!< ADC data offset 2 for channel programmed into bits OFFSET2_CH[4:0] */
#define ADC_OFR2_OFFSET2_0  ((uint32_t)0x00000001) /*!< ADC OFFSET2 bit 0 */
#define ADC_OFR2_OFFSET2_1  ((uint32_t)0x00000002) /*!< ADC OFFSET2 bit 1 */
#define ADC_OFR2_OFFSET2_2  ((uint32_t)0x00000004) /*!< ADC OFFSET2 bit 2 */
#define ADC_OFR2_OFFSET2_3  ((uint32_t)0x00000008) /*!< ADC OFFSET2 bit 3 */
#define ADC_OFR2_OFFSET2_4  ((uint32_t)0x00000010) /*!< ADC OFFSET2 bit 4 */
#define ADC_OFR2_OFFSET2_5  ((uint32_t)0x00000020) /*!< ADC OFFSET2 bit 5 */
#define ADC_OFR2_OFFSET2_6  ((uint32_t)0x00000040) /*!< ADC OFFSET2 bit 6 */
#define ADC_OFR2_OFFSET2_7  ((uint32_t)0x00000080) /*!< ADC OFFSET2 bit 7 */
#define ADC_OFR2_OFFSET2_8  ((uint32_t)0x00000100) /*!< ADC OFFSET2 bit 8 */
#define ADC_OFR2_OFFSET2_9  ((uint32_t)0x00000200) /*!< ADC OFFSET2 bit 9 */
#define ADC_OFR2_OFFSET2_10 ((uint32_t)0x00000400) /*!< ADC OFFSET2 bit 10 */
#define ADC_OFR2_OFFSET2_11 ((uint32_t)0x00000800) /*!< ADC OFFSET2 bit 11 */

#define ADC_OFR2_OFFSET2_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 2 */
#define ADC_OFR2_OFFSET2_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET2_CH bit 0 */
#define ADC_OFR2_OFFSET2_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET2_CH bit 1 */
#define ADC_OFR2_OFFSET2_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET2_CH bit 2 */
#define ADC_OFR2_OFFSET2_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET2_CH bit 3 */
#define ADC_OFR2_OFFSET2_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET2_CH bit 4 */

#define ADC_OFR2_OFFSET2_EN ((uint32_t)0x80000000) /*!< ADC offset 2 enable */

/********************  Bit definition for ADC_OFR3 register  ********************/
#define ADC_OFR3_OFFSET3    ((uint32_t)0x00000FFF) /*!< ADC data offset 3 for channel programmed into bits OFFSET3_CH[4:0] */
#define ADC_OFR3_OFFSET3_0  ((uint32_t)0x00000001) /*!< ADC OFFSET3 bit 0 */
#define ADC_OFR3_OFFSET3_1  ((uint32_t)0x00000002) /*!< ADC OFFSET3 bit 1 */
#define ADC_OFR3_OFFSET3_2  ((uint32_t)0x00000004) /*!< ADC OFFSET3 bit 2 */
#define ADC_OFR3_OFFSET3_3  ((uint32_t)0x00000008) /*!< ADC OFFSET3 bit 3 */
#define ADC_OFR3_OFFSET3_4  ((uint32_t)0x00000010) /*!< ADC OFFSET3 bit 4 */
#define ADC_OFR3_OFFSET3_5  ((uint32_t)0x00000020) /*!< ADC OFFSET3 bit 5 */
#define ADC_OFR3_OFFSET3_6  ((uint32_t)0x00000040) /*!< ADC OFFSET3 bit 6 */
#define ADC_OFR3_OFFSET3_7  ((uint32_t)0x00000080) /*!< ADC OFFSET3 bit 7 */
#define ADC_OFR3_OFFSET3_8  ((uint32_t)0x00000100) /*!< ADC OFFSET3 bit 8 */
#define ADC_OFR3_OFFSET3_9  ((uint32_t)0x00000200) /*!< ADC OFFSET3 bit 9 */
#define ADC_OFR3_OFFSET3_10 ((uint32_t)0x00000400) /*!< ADC OFFSET3 bit 10 */
#define ADC_OFR3_OFFSET3_11 ((uint32_t)0x00000800) /*!< ADC OFFSET3 bit 11 */

#define ADC_OFR3_OFFSET3_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 3 */
#define ADC_OFR3_OFFSET3_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET3_CH bit 0 */
#define ADC_OFR3_OFFSET3_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET3_CH bit 1 */
#define ADC_OFR3_OFFSET3_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET3_CH bit 2 */
#define ADC_OFR3_OFFSET3_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET3_CH bit 3 */
#define ADC_OFR3_OFFSET3_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET3_CH bit 4 */

#define ADC_OFR3_OFFSET3_EN ((uint32_t)0x80000000) /*!< ADC offset 3 enable */

/********************  Bit definition for ADC_OFR4 register  ********************/
#define ADC_OFR4_OFFSET4    ((uint32_t)0x00000FFF) /*!< ADC data offset 4 for channel programmed into bits OFFSET4_CH[4:0] */
#define ADC_OFR4_OFFSET4_0  ((uint32_t)0x00000001) /*!< ADC OFFSET4 bit 0 */
#define ADC_OFR4_OFFSET4_1  ((uint32_t)0x00000002) /*!< ADC OFFSET4 bit 1 */
#define ADC_OFR4_OFFSET4_2  ((uint32_t)0x00000004) /*!< ADC OFFSET4 bit 2 */
#define ADC_OFR4_OFFSET4_3  ((uint32_t)0x00000008) /*!< ADC OFFSET4 bit 3 */
#define ADC_OFR4_OFFSET4_4  ((uint32_t)0x00000010) /*!< ADC OFFSET4 bit 4 */
#define ADC_OFR4_OFFSET4_5  ((uint32_t)0x00000020) /*!< ADC OFFSET4 bit 5 */
#define ADC_OFR4_OFFSET4_6  ((uint32_t)0x00000040) /*!< ADC OFFSET4 bit 6 */
#define ADC_OFR4_OFFSET4_7  ((uint32_t)0x00000080) /*!< ADC OFFSET4 bit 7 */
#define ADC_OFR4_OFFSET4_8  ((uint32_t)0x00000100) /*!< ADC OFFSET4 bit 8 */
#define ADC_OFR4_OFFSET4_9  ((uint32_t)0x00000200) /*!< ADC OFFSET4 bit 9 */
#define ADC_OFR4_OFFSET4_10 ((uint32_t)0x00000400) /*!< ADC OFFSET4 bit 10 */
#define ADC_OFR4_OFFSET4_11 ((uint32_t)0x00000800) /*!< ADC OFFSET4 bit 11 */

#define ADC_OFR4_OFFSET4_CH     ((uint32_t)0x7C000000) /*!< ADC Channel selection for the data offset 4 */
#define ADC_OFR4_OFFSET4_CH_0  ((uint32_t)0x04000000) /*!< ADC OFFSET4_CH bit 0 */
#define ADC_OFR4_OFFSET4_CH_1  ((uint32_t)0x08000000) /*!< ADC OFFSET4_CH bit 1 */
#define ADC_OFR4_OFFSET4_CH_2  ((uint32_t)0x10000000) /*!< ADC OFFSET4_CH bit 2 */
#define ADC_OFR4_OFFSET4_CH_3  ((uint32_t)0x20000000) /*!< ADC OFFSET4_CH bit 3 */
#define ADC_OFR4_OFFSET4_CH_4  ((uint32_t)0x40000000) /*!< ADC OFFSET4_CH bit 4 */

#define ADC_OFR4_OFFSET4_EN ((uint32_t)0x80000000) /*!< ADC offset 4 enable */

/********************  Bit definition for ADC_JDR1 register  ********************/
#define ADC_JDR1_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR1_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR1_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR1_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR1_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR1_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR1_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR1_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR1_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR1_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR1_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR1_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR1_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR1_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR1_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR1_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR1_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR2 register  ********************/
#define ADC_JDR2_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR2_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR2_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR2_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR2_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR2_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR2_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR2_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR2_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR2_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR2_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR2_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR2_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR2_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR2_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR2_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR2_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR3 register  ********************/
#define ADC_JDR3_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR3_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR3_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR3_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR3_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR3_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR3_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR3_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR3_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR3_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR3_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR3_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR3_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR3_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR3_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR3_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR3_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_JDR4 register  ********************/
#define ADC_JDR4_JDATA      ((uint32_t)0x0000FFFF) /*!< ADC Injected DATA */
#define ADC_JDR4_JDATA_0    ((uint32_t)0x00000001) /*!< ADC JDATA bit 0 */
#define ADC_JDR4_JDATA_1    ((uint32_t)0x00000002) /*!< ADC JDATA bit 1 */
#define ADC_JDR4_JDATA_2    ((uint32_t)0x00000004) /*!< ADC JDATA bit 2 */
#define ADC_JDR4_JDATA_3    ((uint32_t)0x00000008) /*!< ADC JDATA bit 3 */
#define ADC_JDR4_JDATA_4    ((uint32_t)0x00000010) /*!< ADC JDATA bit 4 */
#define ADC_JDR4_JDATA_5    ((uint32_t)0x00000020) /*!< ADC JDATA bit 5 */
#define ADC_JDR4_JDATA_6    ((uint32_t)0x00000040) /*!< ADC JDATA bit 6 */
#define ADC_JDR4_JDATA_7    ((uint32_t)0x00000080) /*!< ADC JDATA bit 7 */
#define ADC_JDR4_JDATA_8    ((uint32_t)0x00000100) /*!< ADC JDATA bit 8 */
#define ADC_JDR4_JDATA_9    ((uint32_t)0x00000200) /*!< ADC JDATA bit 9 */
#define ADC_JDR4_JDATA_10   ((uint32_t)0x00000400) /*!< ADC JDATA bit 10 */
#define ADC_JDR4_JDATA_11   ((uint32_t)0x00000800) /*!< ADC JDATA bit 11 */
#define ADC_JDR4_JDATA_12   ((uint32_t)0x00001000) /*!< ADC JDATA bit 12 */
#define ADC_JDR4_JDATA_13   ((uint32_t)0x00002000) /*!< ADC JDATA bit 13 */
#define ADC_JDR4_JDATA_14   ((uint32_t)0x00004000) /*!< ADC JDATA bit 14 */
#define ADC_JDR4_JDATA_15   ((uint32_t)0x00008000) /*!< ADC JDATA bit 15 */

/********************  Bit definition for ADC_AWD2CR register  ********************/
#define ADC_AWD2CR_AWD2CH    ((uint32_t)0x0007FFFE) /*!< ADC Analog watchdog 2 channel selection */
#define ADC_AWD2CR_AWD2CH_0  ((uint32_t)0x00000002) /*!< ADC AWD2CH bit 0 */
#define ADC_AWD2CR_AWD2CH_1  ((uint32_t)0x00000004) /*!< ADC AWD2CH bit 1 */
#define ADC_AWD2CR_AWD2CH_2  ((uint32_t)0x00000008) /*!< ADC AWD2CH bit 2 */
#define ADC_AWD2CR_AWD2CH_3  ((uint32_t)0x00000010) /*!< ADC AWD2CH bit 3 */
#define ADC_AWD2CR_AWD2CH_4  ((uint32_t)0x00000020) /*!< ADC AWD2CH bit 4 */
#define ADC_AWD2CR_AWD2CH_5  ((uint32_t)0x00000040) /*!< ADC AWD2CH bit 5 */
#define ADC_AWD2CR_AWD2CH_6  ((uint32_t)0x00000080) /*!< ADC AWD2CH bit 6 */
#define ADC_AWD2CR_AWD2CH_7  ((uint32_t)0x00000100) /*!< ADC AWD2CH bit 7 */
#define ADC_AWD2CR_AWD2CH_8  ((uint32_t)0x00000200) /*!< ADC AWD2CH bit 8 */
#define ADC_AWD2CR_AWD2CH_9  ((uint32_t)0x00000400) /*!< ADC AWD2CH bit 9 */
#define ADC_AWD2CR_AWD2CH_10 ((uint32_t)0x00000800) /*!< ADC AWD2CH bit 10 */
#define ADC_AWD2CR_AWD2CH_11 ((uint32_t)0x00001000) /*!< ADC AWD2CH bit 11 */
#define ADC_AWD2CR_AWD2CH_12 ((uint32_t)0x00002000) /*!< ADC AWD2CH bit 12 */
#define ADC_AWD2CR_AWD2CH_13 ((uint32_t)0x00004000) /*!< ADC AWD2CH bit 13 */
#define ADC_AWD2CR_AWD2CH_14 ((uint32_t)0x00008000) /*!< ADC AWD2CH bit 14 */
#define ADC_AWD2CR_AWD2CH_15 ((uint32_t)0x00010000) /*!< ADC AWD2CH bit 15 */
#define ADC_AWD2CR_AWD2CH_16 ((uint32_t)0x00020000) /*!< ADC AWD2CH bit 16 */
#define ADC_AWD2CR_AWD2CH_17 ((uint32_t)0x00030000) /*!< ADC AWD2CH bit 17 */

/********************  Bit definition for ADC_AWD3CR register  ********************/
#define ADC_AWD3CR_AWD3CH    ((uint32_t)0x0007FFFE) /*!< ADC Analog watchdog 2 channel selection */
#define ADC_AWD3CR_AWD3CH_0  ((uint32_t)0x00000002) /*!< ADC AWD3CH bit 0 */
#define ADC_AWD3CR_AWD3CH_1  ((uint32_t)0x00000004) /*!< ADC AWD3CH bit 1 */
#define ADC_AWD3CR_AWD3CH_2  ((uint32_t)0x00000008) /*!< ADC AWD3CH bit 2 */
#define ADC_AWD3CR_AWD3CH_3  ((uint32_t)0x00000010) /*!< ADC AWD3CH bit 3 */
#define ADC_AWD3CR_AWD3CH_4  ((uint32_t)0x00000020) /*!< ADC AWD3CH bit 4 */
#define ADC_AWD3CR_AWD3CH_5  ((uint32_t)0x00000040) /*!< ADC AWD3CH bit 5 */
#define ADC_AWD3CR_AWD3CH_6  ((uint32_t)0x00000080) /*!< ADC AWD3CH bit 6 */
#define ADC_AWD3CR_AWD3CH_7  ((uint32_t)0x00000100) /*!< ADC AWD3CH bit 7 */
#define ADC_AWD3CR_AWD3CH_8  ((uint32_t)0x00000200) /*!< ADC AWD3CH bit 8 */
#define ADC_AWD3CR_AWD3CH_9  ((uint32_t)0x00000400) /*!< ADC AWD3CH bit 9 */
#define ADC_AWD3CR_AWD3CH_10 ((uint32_t)0x00000800) /*!< ADC AWD3CH bit 10 */
#define ADC_AWD3CR_AWD3CH_11 ((uint32_t)0x00001000) /*!< ADC AWD3CH bit 11 */
#define ADC_AWD3CR_AWD3CH_12 ((uint32_t)0x00002000) /*!< ADC AWD3CH bit 12 */
#define ADC_AWD3CR_AWD3CH_13 ((uint32_t)0x00004000) /*!< ADC AWD3CH bit 13 */
#define ADC_AWD3CR_AWD3CH_14 ((uint32_t)0x00008000) /*!< ADC AWD3CH bit 14 */
#define ADC_AWD3CR_AWD3CH_15 ((uint32_t)0x00010000) /*!< ADC AWD3CH bit 15 */
#define ADC_AWD3CR_AWD3CH_16 ((uint32_t)0x00020000) /*!< ADC AWD3CH bit 16 */
#define ADC_AWD3CR_AWD3CH_17 ((uint32_t)0x00030000) /*!< ADC AWD3CH bit 17 */

/********************  Bit definition for ADC_DIFSEL register  ********************/
#define ADC_DIFSEL_DIFSEL    ((uint32_t)0x0007FFFE) /*!< ADC differential modes for channels 1 to 18 */
#define ADC_DIFSEL_DIFSEL_0  ((uint32_t)0x00000002) /*!< ADC DIFSEL bit 0 */
#define ADC_DIFSEL_DIFSEL_1  ((uint32_t)0x00000004) /*!< ADC DIFSEL bit 1 */
#define ADC_DIFSEL_DIFSEL_2  ((uint32_t)0x00000008) /*!< ADC DIFSEL bit 2 */
#define ADC_DIFSEL_DIFSEL_3  ((uint32_t)0x00000010) /*!< ADC DIFSEL bit 3 */
#define ADC_DIFSEL_DIFSEL_4  ((uint32_t)0x00000020) /*!< ADC DIFSEL bit 4 */
#define ADC_DIFSEL_DIFSEL_5  ((uint32_t)0x00000040) /*!< ADC DIFSEL bit 5 */
#define ADC_DIFSEL_DIFSEL_6  ((uint32_t)0x00000080) /*!< ADC DIFSEL bit 6 */
#define ADC_DIFSEL_DIFSEL_7  ((uint32_t)0x00000100) /*!< ADC DIFSEL bit 7 */
#define ADC_DIFSEL_DIFSEL_8  ((uint32_t)0x00000200) /*!< ADC DIFSEL bit 8 */
#define ADC_DIFSEL_DIFSEL_9  ((uint32_t)0x00000400) /*!< ADC DIFSEL bit 9 */
#define ADC_DIFSEL_DIFSEL_10 ((uint32_t)0x00000800) /*!< ADC DIFSEL bit 10 */
#define ADC_DIFSEL_DIFSEL_11 ((uint32_t)0x00001000) /*!< ADC DIFSEL bit 11 */
#define ADC_DIFSEL_DIFSEL_12 ((uint32_t)0x00002000) /*!< ADC DIFSEL bit 12 */
#define ADC_DIFSEL_DIFSEL_13 ((uint32_t)0x00004000) /*!< ADC DIFSEL bit 13 */
#define ADC_DIFSEL_DIFSEL_14 ((uint32_t)0x00008000) /*!< ADC DIFSEL bit 14 */
#define ADC_DIFSEL_DIFSEL_15 ((uint32_t)0x00010000) /*!< ADC DIFSEL bit 15 */
#define ADC_DIFSEL_DIFSEL_16 ((uint32_t)0x00020000) /*!< ADC DIFSEL bit 16 */
#define ADC_DIFSEL_DIFSEL_17 ((uint32_t)0x00030000) /*!< ADC DIFSEL bit 17 */

/********************  Bit definition for ADC_CALFACT register  ********************/
#define ADC_CALFACT_CALFACT_S   ((uint32_t)0x0000007F) /*!< ADC calibration factors in single-ended mode */
#define ADC_CALFACT_CALFACT_S_0 ((uint32_t)0x00000001) /*!< ADC CALFACT_S bit 0 */
#define ADC_CALFACT_CALFACT_S_1 ((uint32_t)0x00000002) /*!< ADC CALFACT_S bit 1 */
#define ADC_CALFACT_CALFACT_S_2 ((uint32_t)0x00000004) /*!< ADC CALFACT_S bit 2 */
#define ADC_CALFACT_CALFACT_S_3 ((uint32_t)0x00000008) /*!< ADC CALFACT_S bit 3 */
#define ADC_CALFACT_CALFACT_S_4 ((uint32_t)0x00000010) /*!< ADC CALFACT_S bit 4 */
#define ADC_CALFACT_CALFACT_S_5 ((uint32_t)0x00000020) /*!< ADC CALFACT_S bit 5 */
#define ADC_CALFACT_CALFACT_S_6 ((uint32_t)0x00000040) /*!< ADC CALFACT_S bit 6 */
#define ADC_CALFACT_CALFACT_D   ((uint32_t)0x007F0000) /*!< ADC calibration factors in differential mode */
#define ADC_CALFACT_CALFACT_D_0 ((uint32_t)0x00010000) /*!< ADC CALFACT_D bit 0 */
#define ADC_CALFACT_CALFACT_D_1 ((uint32_t)0x00020000) /*!< ADC CALFACT_D bit 1 */
#define ADC_CALFACT_CALFACT_D_2 ((uint32_t)0x00040000) /*!< ADC CALFACT_D bit 2 */
#define ADC_CALFACT_CALFACT_D_3 ((uint32_t)0x00080000) /*!< ADC CALFACT_D bit 3 */
#define ADC_CALFACT_CALFACT_D_4 ((uint32_t)0x00100000) /*!< ADC CALFACT_D bit 4 */
#define ADC_CALFACT_CALFACT_D_5 ((uint32_t)0x00200000) /*!< ADC CALFACT_D bit 5 */
#define ADC_CALFACT_CALFACT_D_6 ((uint32_t)0x00400000) /*!< ADC CALFACT_D bit 6 */

/*************************  ADC Common registers  *****************************/
/********************  Bit definition for ADC12_CSR register  ********************/
#define ADC12_CSR_ADRDY_MST         ((uint32_t)0x00000001) /*!< Master ADC ready */
#define ADC12_CSR_ADRDY_EOSMP_MST   ((uint32_t)0x00000002) /*!< End of sampling phase flag of the master ADC */
#define ADC12_CSR_ADRDY_EOC_MST     ((uint32_t)0x00000004) /*!< End of regular conversion of the master ADC */
#define ADC12_CSR_ADRDY_EOS_MST     ((uint32_t)0x00000008) /*!< End of regular sequence flag of the master ADC */
#define ADC12_CSR_ADRDY_OVR_MST     ((uint32_t)0x00000010) /*!< Overrun flag of the master ADC */
#define ADC12_CSR_ADRDY_JEOC_MST    ((uint32_t)0x00000020) /*!< End of injected conversion of the master ADC */
#define ADC12_CSR_ADRDY_JEOS_MST    ((uint32_t)0x00000040) /*!< End of injected sequence flag of the master ADC */
#define ADC12_CSR_AWD1_MST          ((uint32_t)0x00000080) /*!< Analog watchdog 1 flag of the master ADC */
#define ADC12_CSR_AWD2_MST          ((uint32_t)0x00000100) /*!< Analog watchdog 2 flag of the master ADC */
#define ADC12_CSR_AWD3_MST          ((uint32_t)0x00000200) /*!< Analog watchdog 3 flag of the master ADC */
#define ADC12_CSR_JQOVF_MST         ((uint32_t)0x00000400) /*!< Injected context queue overflow flag of the master ADC */
#define ADC12_CSR_ADRDY_SLV         ((uint32_t)0x00010000) /*!< Slave ADC ready */
#define ADC12_CSR_ADRDY_EOSMP_SLV   ((uint32_t)0x00020000) /*!< End of sampling phase flag of the slave ADC */
#define ADC12_CSR_ADRDY_EOC_SLV     ((uint32_t)0x00040000) /*!< End of regular conversion of the slave ADC */
#define ADC12_CSR_ADRDY_EOS_SLV     ((uint32_t)0x00080000) /*!< End of regular sequence flag of the slave ADC */
#define ADC12_CSR_ADRDY_OVR_SLV     ((uint32_t)0x00100000) /*!< Overrun flag of the slave ADC */
#define ADC12_CSR_ADRDY_JEOC_SLV    ((uint32_t)0x00200000) /*!< End of injected conversion of the slave ADC */
#define ADC12_CSR_ADRDY_JEOS_SLV    ((uint32_t)0x00400000) /*!< End of injected sequence flag of the slave ADC */
#define ADC12_CSR_AWD1_SLV          ((uint32_t)0x00800000) /*!< Analog watchdog 1 flag of the slave ADC */
#define ADC12_CSR_AWD2_SLV          ((uint32_t)0x01000000) /*!< Analog watchdog 2 flag of the slave ADC */
#define ADC12_CSR_AWD3_SLV          ((uint32_t)0x02000000) /*!< Analog watchdog 3 flag of the slave ADC */
#define ADC12_CSR_JQOVF_SLV         ((uint32_t)0x04000000) /*!< Injected context queue overflow flag of the slave ADC */

/********************  Bit definition for ADC_CCR register  ********************/
#define ADC12_CCR_MULTI             ((uint32_t)0x0000001F) /*!< Multi ADC mode selection */
#define ADC12_CCR_MULTI_0           ((uint32_t)0x00000001) /*!< MULTI bit 0 */
#define ADC12_CCR_MULTI_1           ((uint32_t)0x00000002) /*!< MULTI bit 1 */
#define ADC12_CCR_MULTI_2           ((uint32_t)0x00000004) /*!< MULTI bit 2 */
#define ADC12_CCR_MULTI_3           ((uint32_t)0x00000008) /*!< MULTI bit 3 */
#define ADC12_CCR_MULTI_4           ((uint32_t)0x00000010) /*!< MULTI bit 4 */
#define ADC12_CCR_DELAY             ((uint32_t)0x00000F00) /*!< Delay between 2 sampling phases */
#define ADC12_CCR_DELAY_0           ((uint32_t)0x00000100) /*!< DELAY bit 0 */
#define ADC12_CCR_DELAY_1           ((uint32_t)0x00000200) /*!< DELAY bit 1 */
#define ADC12_CCR_DELAY_2           ((uint32_t)0x00000400) /*!< DELAY bit 2 */
#define ADC12_CCR_DELAY_3           ((uint32_t)0x00000800) /*!< DELAY bit 3 */
#define ADC12_CCR_DMACFG            ((uint32_t)0x00002000) /*!< DMA configuration for multi-ADC mode */
#define ADC12_CCR_MDMA              ((uint32_t)0x0000C000) /*!< DMA mode for multi-ADC mode */
#define ADC12_CCR_MDMA_0            ((uint32_t)0x00004000) /*!< MDMA bit 0 */
#define ADC12_CCR_MDMA_1            ((uint32_t)0x00008000) /*!< MDMA bit 1 */
#define ADC12_CCR_CKMODE            ((uint32_t)0x00030000) /*!< ADC clock mode */
#define ADC12_CCR_CKMODE_0          ((uint32_t)0x00010000) /*!< CKMODE bit 0 */
#define ADC12_CCR_CKMODE_1          ((uint32_t)0x00020000) /*!< CKMODE bit 1 */
#define ADC12_CCR_VREFEN            ((uint32_t)0x00400000) /*!< VREFINT enable */
#define ADC12_CCR_TSEN              ((uint32_t)0x00800000) /*!< Temperature sensor enable */
#define ADC12_CCR_VBATEN            ((uint32_t)0x01000000) /*!< VBAT enable */

/********************  Bit definition for ADC_CDR register  ********************/
#define ADC12_CDR_RDATA_MST         ((uint32_t)0x0000FFFF) /*!< Regular Data of the master ADC */
#define ADC12_CDR_RDATA_MST_0       ((uint32_t)0x00000001) /*!< RDATA_MST bit 0 */
#define ADC12_CDR_RDATA_MST_1       ((uint32_t)0x00000002) /*!< RDATA_MST bit 1 */
#define ADC12_CDR_RDATA_MST_2       ((uint32_t)0x00000004) /*!< RDATA_MST bit 2 */
#define ADC12_CDR_RDATA_MST_3       ((uint32_t)0x00000008) /*!< RDATA_MST bit 3 */
#define ADC12_CDR_RDATA_MST_4       ((uint32_t)0x00000010) /*!< RDATA_MST bit 4 */
#define ADC12_CDR_RDATA_MST_5       ((uint32_t)0x00000020) /*!< RDATA_MST bit 5 */
#define ADC12_CDR_RDATA_MST_6       ((uint32_t)0x00000040) /*!< RDATA_MST bit 6 */
#define ADC12_CDR_RDATA_MST_7       ((uint32_t)0x00000080) /*!< RDATA_MST bit 7 */
#define ADC12_CDR_RDATA_MST_8       ((uint32_t)0x00000100) /*!< RDATA_MST bit 8 */
#define ADC12_CDR_RDATA_MST_9       ((uint32_t)0x00000200) /*!< RDATA_MST bit 9 */
#define ADC12_CDR_RDATA_MST_10      ((uint32_t)0x00000400) /*!< RDATA_MST bit 10 */
#define ADC12_CDR_RDATA_MST_11      ((uint32_t)0x00000800) /*!< RDATA_MST bit 11 */
#define ADC12_CDR_RDATA_MST_12      ((uint32_t)0x00001000) /*!< RDATA_MST bit 12 */
#define ADC12_CDR_RDATA_MST_13      ((uint32_t)0x00002000) /*!< RDATA_MST bit 13 */
#define ADC12_CDR_RDATA_MST_14      ((uint32_t)0x00004000) /*!< RDATA_MST bit 14 */
#define ADC12_CDR_RDATA_MST_15      ((uint32_t)0x00008000) /*!< RDATA_MST bit 15 */

#define ADC12_CDR_RDATA_SLV         ((uint32_t)0xFFFF0000) /*!< Regular Data of the master ADC */
#define ADC12_CDR_RDATA_SLV_0       ((uint32_t)0x00010000) /*!< RDATA_SLV bit 0 */
#define ADC12_CDR_RDATA_SLV_1       ((uint32_t)0x00020000) /*!< RDATA_SLV bit 1 */
#define ADC12_CDR_RDATA_SLV_2       ((uint32_t)0x00040000) /*!< RDATA_SLV bit 2 */
#define ADC12_CDR_RDATA_SLV_3       ((uint32_t)0x00080000) /*!< RDATA_SLV bit 3 */
#define ADC12_CDR_RDATA_SLV_4       ((uint32_t)0x00100000) /*!< RDATA_SLV bit 4 */
#define ADC12_CDR_RDATA_SLV_5       ((uint32_t)0x00200000) /*!< RDATA_SLV bit 5 */
#define ADC12_CDR_RDATA_SLV_6       ((uint32_t)0x00400000) /*!< RDATA_SLV bit 6 */
#define ADC12_CDR_RDATA_SLV_7       ((uint32_t)0x00800000) /*!< RDATA_SLV bit 7 */
#define ADC12_CDR_RDATA_SLV_8       ((uint32_t)0x01000000) /*!< RDATA_SLV bit 8 */
#define ADC12_CDR_RDATA_SLV_9       ((uint32_t)0x02000000) /*!< RDATA_SLV bit 9 */
#define ADC12_CDR_RDATA_SLV_10      ((uint32_t)0x04000000) /*!< RDATA_SLV bit 10 */
#define ADC12_CDR_RDATA_SLV_11      ((uint32_t)0x08000000) /*!< RDATA_SLV bit 11 */
#define ADC12_CDR_RDATA_SLV_12      ((uint32_t)0x10000000) /*!< RDATA_SLV bit 12 */
#define ADC12_CDR_RDATA_SLV_13      ((uint32_t)0x20000000) /*!< RDATA_SLV bit 13 */
#define ADC12_CDR_RDATA_SLV_14      ((uint32_t)0x40000000) /*!< RDATA_SLV bit 14 */
#define ADC12_CDR_RDATA_SLV_15      ((uint32_t)0x80000000) /*!< RDATA_SLV bit 15 */

/******************************************************************************/
/*                                                                            */
/*                      Analog Comparators (COMP)                             */
/*                                                                            */
/******************************************************************************/

/**********************  Bit definition for COMP1_CSR register  ***************/
#define COMP1_CSR_COMP1EN               ((uint32_t)0x00000001) /*!< COMP1 enable */
#define COMP1_CSR_COMP1SW1              ((uint32_t)0x00000002) /*!< COMP1 SW1 switch control */
#define COMP1_CSR_COMP1MODE             ((uint32_t)0x0000000C) /*!< COMP1 power mode */
#define COMP1_CSR_COMP1MODE_0           ((uint32_t)0x00000004) /*!< COMP1 power mode bit 0 */
#define COMP1_CSR_COMP1MODE_1           ((uint32_t)0x00000008) /*!< COMP1 power mode bit 1 */
#define COMP1_CSR_COMP1INSEL            ((uint32_t)0x00000070) /*!< COMP1 inverting input select */
#define COMP1_CSR_COMP1INSEL_0          ((uint32_t)0x00000010) /*!< COMP1 inverting input select bit 0 */
#define COMP1_CSR_COMP1INSEL_1          ((uint32_t)0x00000020) /*!< COMP1 inverting input select bit 1 */
#define COMP1_CSR_COMP1INSEL_2          ((uint32_t)0x00000040) /*!< COMP1 inverting input select bit 2 */
#define COMP1_CSR_COMP1NONINSEL         ((uint32_t)0x00000080) /*!< COMP1 non inverting input select */
#define COMP1_CSR_COMP1OUTSEL           ((uint32_t)0x00003C00) /*!< COMP1 output select */
#define COMP1_CSR_COMP1OUTSEL_0         ((uint32_t)0x00000400) /*!< COMP1 output select bit 0 */
#define COMP1_CSR_COMP1OUTSEL_1         ((uint32_t)0x00000800) /*!< COMP1 output select bit 1 */
#define COMP1_CSR_COMP1OUTSEL_2         ((uint32_t)0x00001000) /*!< COMP1 output select bit 2 */
#define COMP1_CSR_COMP1OUTSEL_3         ((uint32_t)0x00002000) /*!< COMP1 output select bit 3 */
#define COMP1_CSR_COMP1POL              ((uint32_t)0x00008000) /*!< COMP1 output polarity */
#define COMP1_CSR_COMP1HYST             ((uint32_t)0x00030000) /*!< COMP1 hysteresis */
#define COMP1_CSR_COMP1HYST_0           ((uint32_t)0x00010000) /*!< COMP1 hysteresis bit 0 */
#define COMP1_CSR_COMP1HYST_1           ((uint32_t)0x00020000) /*!< COMP1 hysteresis bit 1 */
#define COMP1_CSR_COMP1BLANKING         ((uint32_t)0x000C0000) /*!< COMP1 blanking */
#define COMP1_CSR_COMP1BLANKING_0       ((uint32_t)0x00040000) /*!< COMP1 blanking bit 0 */
#define COMP1_CSR_COMP1BLANKING_1       ((uint32_t)0x00080000) /*!< COMP1 blanking bit 1 */
#define COMP1_CSR_COMP1BLANKING_2       ((uint32_t)0x00100000) /*!< COMP1 blanking bit 2 */
#define COMP1_CSR_COMP1OUT              ((uint32_t)0x40000000) /*!< COMP1 output level */
#define COMP1_CSR_COMP1LOCK             ((uint32_t)0x80000000) /*!< COMP1 lock */

/**********************  Bit definition for COMP2_CSR register  ***************/
#define COMP2_CSR_COMP2EN               ((uint32_t)0x00000001) /*!< COMP2 enable */
#define COMP2_CSR_COMP2MODE             ((uint32_t)0x0000000C) /*!< COMP2 power mode */
#define COMP2_CSR_COMP2MODE_0           ((uint32_t)0x00000004) /*!< COMP2 power mode bit 0 */
#define COMP2_CSR_COMP2MODE_1           ((uint32_t)0x00000008) /*!< COMP2 power mode bit 1 */
#define COMP2_CSR_COMP2INSEL            ((uint32_t)0x00000070) /*!< COMP2 inverting input select */
#define COMP2_CSR_COMP2INSEL_0          ((uint32_t)0x00000010) /*!< COMP2 inverting input select bit 0 */
#define COMP2_CSR_COMP2INSEL_1          ((uint32_t)0x00000020) /*!< COMP2 inverting input select bit 1 */
#define COMP2_CSR_COMP2INSEL_2          ((uint32_t)0x00000040) /*!< COMP2 inverting input select bit 2 */
#define COMP2_CSR_COMP2NONINSEL         ((uint32_t)0x00000080) /*!< COMP2 non inverting input select */
#define COMP2_CSR_COMP2WNDWEN           ((uint32_t)0x00000200) /*!< COMP2 window mode enable */
#define COMP2_CSR_COMP2OUTSEL           ((uint32_t)0x00003C00) /*!< COMP2 output select */
#define COMP2_CSR_COMP2OUTSEL_0         ((uint32_t)0x00000400) /*!< COMP2 output select bit 0 */
#define COMP2_CSR_COMP2OUTSEL_1         ((uint32_t)0x00000800) /*!< COMP2 output select bit 1 */
#define COMP2_CSR_COMP2OUTSEL_2         ((uint32_t)0x00001000) /*!< COMP2 output select bit 2 */
#define COMP2_CSR_COMP2OUTSEL_3         ((uint32_t)0x00002000) /*!< COMP2 output select bit 3 */
#define COMP2_CSR_COMP2POL              ((uint32_t)0x00008000) /*!< COMP2 output polarity */
#define COMP2_CSR_COMP2HYST             ((uint32_t)0x00030000) /*!< COMP2 hysteresis */
#define COMP2_CSR_COMP2HYST_0           ((uint32_t)0x00010000) /*!< COMP2 hysteresis bit 0 */
#define COMP2_CSR_COMP2HYST_1           ((uint32_t)0x00020000) /*!< COMP2 hysteresis bit 1 */
#define COMP2_CSR_COMP2BLANKING         ((uint32_t)0x000C0000) /*!< COMP2 blanking */
#define COMP2_CSR_COMP2BLANKING_0       ((uint32_t)0x00040000) /*!< COMP2 blanking bit 0 */
#define COMP2_CSR_COMP2BLANKING_1       ((uint32_t)0x00080000) /*!< COMP2 blanking bit 1 */
#define COMP2_CSR_COMP2BLANKING_2       ((uint32_t)0x00100000) /*!< COMP2 blanking bit 2 */
#define COMP2_CSR_COMP2OUT              ((uint32_t)0x40000000) /*!< COMP2 output level */
#define COMP2_CSR_COMP2LOCK             ((uint32_t)0x80000000) /*!< COMP2 lock */

/**********************  Bit definition for COMP4_CSR register  ***************/
#define COMP4_CSR_COMP4EN               ((uint32_t)0x00000001) /*!< COMP4 enable */
#define COMP4_CSR_COMP4MODE             ((uint32_t)0x0000000C) /*!< COMP4 power mode */
#define COMP4_CSR_COMP4MODE_0           ((uint32_t)0x00000004) /*!< COMP4 power mode bit 0 */
#define COMP4_CSR_COMP4MODE_1           ((uint32_t)0x00000008) /*!< COMP4 power mode bit 1 */
#define COMP4_CSR_COMP4INSEL            ((uint32_t)0x00000070) /*!< COMP4 inverting input select */
#define COMP4_CSR_COMP4INSEL_0          ((uint32_t)0x00000010) /*!< COMP4 inverting input select bit 0 */
#define COMP4_CSR_COMP4INSEL_1          ((uint32_t)0x00000020) /*!< COMP4 inverting input select bit 1 */
#define COMP4_CSR_COMP4INSEL_2          ((uint32_t)0x00000040) /*!< COMP4 inverting input select bit 2 */
#define COMP4_CSR_COMP4NONINSEL         ((uint32_t)0x00000080) /*!< COMP4 non inverting input select */
#define COMP4_CSR_COMP4WNDWEN           ((uint32_t)0x00000200) /*!< COMP4 window mode enable */
#define COMP4_CSR_COMP4OUTSEL           ((uint32_t)0x00003C00) /*!< COMP4 output select */
#define COMP4_CSR_COMP4OUTSEL_0         ((uint32_t)0x00000400) /*!< COMP4 output select bit 0 */
#define COMP4_CSR_COMP4OUTSEL_1         ((uint32_t)0x00000800) /*!< COMP4 output select bit 1 */
#define COMP4_CSR_COMP4OUTSEL_2         ((uint32_t)0x00001000) /*!< COMP4 output select bit 2 */
#define COMP4_CSR_COMP4OUTSEL_3         ((uint32_t)0x00002000) /*!< COMP4 output select bit 3 */
#define COMP4_CSR_COMP4POL              ((uint32_t)0x00008000) /*!< COMP4 output polarity */
#define COMP4_CSR_COMP4HYST             ((uint32_t)0x00030000) /*!< COMP4 hysteresis */
#define COMP4_CSR_COMP4HYST_0           ((uint32_t)0x00010000) /*!< COMP4 hysteresis bit 0 */
#define COMP4_CSR_COMP4HYST_1           ((uint32_t)0x00020000) /*!< COMP4 hysteresis bit 1 */
#define COMP4_CSR_COMP4BLANKING         ((uint32_t)0x000C0000) /*!< COMP4 blanking */
#define COMP4_CSR_COMP4BLANKING_0       ((uint32_t)0x00040000) /*!< COMP4 blanking bit 0 */
#define COMP4_CSR_COMP4BLANKING_1       ((uint32_t)0x00080000) /*!< COMP4 blanking bit 1 */
#define COMP4_CSR_COMP4BLANKING_2       ((uint32_t)0x00100000) /*!< COMP4 blanking bit 2 */
#define COMP4_CSR_COMP4OUT              ((uint32_t)0x40000000) /*!< COMP4 output level */
#define COMP4_CSR_COMP4LOCK             ((uint32_t)0x80000000) /*!< COMP4 lock */

/**********************  Bit definition for COMP6_CSR register  ***************/
#define COMP6_CSR_COMP6EN               ((uint32_t)0x00000001) /*!< COMP6 enable */
#define COMP6_CSR_COMP6MODE             ((uint32_t)0x0000000C) /*!< COMP6 power mode */
#define COMP6_CSR_COMP6MODE_0           ((uint32_t)0x00000004) /*!< COMP6 power mode bit 0 */
#define COMP6_CSR_COMP6MODE_1           ((uint32_t)0x00000008) /*!< COMP6 power mode bit 1 */
#define COMP6_CSR_COMP6INSEL            ((uint32_t)0x00000070) /*!< COMP6 inverting input select */
#define COMP6_CSR_COMP6INSEL_0          ((uint32_t)0x00000010) /*!< COMP6 inverting input select bit 0 */
#define COMP6_CSR_COMP6INSEL_1          ((uint32_t)0x00000020) /*!< COMP6 inverting input select bit 1 */
#define COMP6_CSR_COMP6INSEL_2          ((uint32_t)0x00000040) /*!< COMP6 inverting input select bit 2 */
#define COMP6_CSR_COMP6NONINSEL         ((uint32_t)0x00000080) /*!< COMP6 non inverting input select */
#define COMP6_CSR_COMP6WNDWEN           ((uint32_t)0x00000200) /*!< COMP6 window mode enable */
#define COMP6_CSR_COMP6OUTSEL           ((uint32_t)0x00003C00) /*!< COMP6 output select */
#define COMP6_CSR_COMP6OUTSEL_0         ((uint32_t)0x00000400) /*!< COMP6 output select bit 0 */
#define COMP6_CSR_COMP6OUTSEL_1         ((uint32_t)0x00000800) /*!< COMP6 output select bit 1 */
#define COMP6_CSR_COMP6OUTSEL_2         ((uint32_t)0x00001000) /*!< COMP6 output select bit 2 */
#define COMP6_CSR_COMP6OUTSEL_3         ((uint32_t)0x00002000) /*!< COMP6 output select bit 3 */
#define COMP6_CSR_COMP6POL              ((uint32_t)0x00008000) /*!< COMP6 output polarity */
#define COMP6_CSR_COMP6HYST             ((uint32_t)0x00030000) /*!< COMP6 hysteresis */
#define COMP6_CSR_COMP6HYST_0           ((uint32_t)0x00010000) /*!< COMP6 hysteresis bit 0 */
#define COMP6_CSR_COMP6HYST_1           ((uint32_t)0x00020000) /*!< COMP6 hysteresis bit 1 */
#define COMP6_CSR_COMP6BLANKING         ((uint32_t)0x000C0000) /*!< COMP6 blanking */
#define COMP6_CSR_COMP6BLANKING_0       ((uint32_t)0x00040000) /*!< COMP6 blanking bit 0 */
#define COMP6_CSR_COMP6BLANKING_1       ((uint32_t)0x00080000) /*!< COMP6 blanking bit 1 */
#define COMP6_CSR_COMP6BLANKING_2       ((uint32_t)0x00100000) /*!< COMP6 blanking bit 2 */
#define COMP6_CSR_COMP6OUT              ((uint32_t)0x40000000) /*!< COMP6 output level */
#define COMP6_CSR_COMP6LOCK             ((uint32_t)0x80000000) /*!< COMP6 lock */

/**********************  Bit definition for COMP_CSR register  ****************/
#define COMP_CSR_COMPxEN               ((uint32_t)0x00000001) /*!< COMPx enable */
#define COMP_CSR_COMP1SW1              ((uint32_t)0x00000002) /*!< COMP1 SW1 switch control */
#define COMP_CSR_COMPxMODE             ((uint32_t)0x0000000C) /*!< COMPx power mode */
#define COMP_CSR_COMPxMODE_0           ((uint32_t)0x00000004) /*!< COMPx power mode bit 0 */
#define COMP_CSR_COMPxMODE_1           ((uint32_t)0x00000008) /*!< COMPx power mode bit 1 */
#define COMP_CSR_COMPxINSEL            ((uint32_t)0x00000070) /*!< COMPx inverting input select */
#define COMP_CSR_COMPxINSEL_0          ((uint32_t)0x00000010) /*!< COMPx inverting input select bit 0 */
#define COMP_CSR_COMPxINSEL_1          ((uint32_t)0x00000020) /*!< COMPx inverting input select bit 1 */
#define COMP_CSR_COMPxINSEL_2          ((uint32_t)0x00000040) /*!< COMPx inverting input select bit 2 */
#define COMP_CSR_COMPxNONINSEL         ((uint32_t)0x00000080) /*!< COMPx non inverting input select */
#define COMP_CSR_COMPxWNDWEN           ((uint32_t)0x00000200) /*!< COMPx window mode enable */
#define COMP_CSR_COMPxOUTSEL           ((uint32_t)0x00003C00) /*!< COMPx output select */
#define COMP_CSR_COMPxOUTSEL_0         ((uint32_t)0x00000400) /*!< COMPx output select bit 0 */
#define COMP_CSR_COMPxOUTSEL_1         ((uint32_t)0x00000800) /*!< COMPx output select bit 1 */
#define COMP_CSR_COMPxOUTSEL_2         ((uint32_t)0x00001000) /*!< COMPx output select bit 2 */
#define COMP_CSR_COMPxOUTSEL_3         ((uint32_t)0x00002000) /*!< COMPx output select bit 3 */
#define COMP_CSR_COMPxPOL              ((uint32_t)0x00008000) /*!< COMPx output polarity */
#define COMP_CSR_COMPxHYST             ((uint32_t)0x00030000) /*!< COMPx hysteresis */
#define COMP_CSR_COMPxHYST_0           ((uint32_t)0x00010000) /*!< COMPx hysteresis bit 0 */
#define COMP_CSR_COMPxHYST_1           ((uint32_t)0x00020000) /*!< COMPx hysteresis bit 1 */
#define COMP_CSR_COMPxBLANKING         ((uint32_t)0x000C0000) /*!< COMPx blanking */
#define COMP_CSR_COMPxBLANKING_0       ((uint32_t)0x00040000) /*!< COMPx blanking bit 0 */
#define COMP_CSR_COMPxBLANKING_1       ((uint32_t)0x00080000) /*!< COMPx blanking bit 1 */
#define COMP_CSR_COMPxBLANKING_2       ((uint32_t)0x00100000) /*!< COMPx blanking bit 2 */
#define COMP_CSR_COMPxOUT              ((uint32_t)0x40000000) /*!< COMPx output level */
#define COMP_CSR_COMPxLOCK             ((uint32_t)0x80000000) /*!< COMPx lock */

/******************************************************************************/
/*                                                                            */
/*                     Operational Amplifier (OPAMP)                          */
/*                                                                            */
/******************************************************************************/
/*********************  Bit definition for OPAMP1_CSR register  ***************/
#define OPAMP1_CSR_OPAMP1EN               ((uint32_t)0x00000001) /*!< OPAMP1 enable */
#define OPAMP1_CSR_FORCEVP                ((uint32_t)0x00000002) /*!< Connect the internal references to the plus input of the OPAMPX */
#define OPAMP1_CSR_VPSEL                  ((uint32_t)0x0000000C) /*!< Non inverting input selection */
#define OPAMP1_CSR_VPSEL_0                ((uint32_t)0x00000004) /*!< Bit 0 */
#define OPAMP1_CSR_VPSEL_1                ((uint32_t)0x00000008) /*!< Bit 1 */
#define OPAMP1_CSR_VMSEL                  ((uint32_t)0x00000060) /*!< Inverting input selection */
#define OPAMP1_CSR_VMSEL_0                ((uint32_t)0x00000020) /*!< Bit 0 */
#define OPAMP1_CSR_VMSEL_1                ((uint32_t)0x00000040) /*!< Bit 1 */
#define OPAMP1_CSR_TCMEN                  ((uint32_t)0x00000080) /*!< Timer-Controlled Mux mode enable */
#define OPAMP1_CSR_VMSSEL                 ((uint32_t)0x00000100) /*!< Inverting input secondary selection */
#define OPAMP1_CSR_VPSSEL                 ((uint32_t)0x00000600) /*!< Non inverting input secondary selection */
#define OPAMP1_CSR_VPSSEL_0               ((uint32_t)0x00000200) /*!< Bit 0 */
#define OPAMP1_CSR_VPSSEL_1               ((uint32_t)0x00000400) /*!< Bit 1 */
#define OPAMP1_CSR_CALON                  ((uint32_t)0x00000800) /*!< Calibration mode enable */
#define OPAMP1_CSR_CALSEL                 ((uint32_t)0x00003000) /*!< Calibration selection */
#define OPAMP1_CSR_CALSEL_0               ((uint32_t)0x00001000) /*!< Bit 0 */
#define OPAMP1_CSR_CALSEL_1               ((uint32_t)0x00002000) /*!< Bit 1 */
#define OPAMP1_CSR_PGGAIN                 ((uint32_t)0x0003C000) /*!< Gain in PGA mode */
#define OPAMP1_CSR_PGGAIN_0               ((uint32_t)0x00004000) /*!< Bit 0 */
#define OPAMP1_CSR_PGGAIN_1               ((uint32_t)0x00008000) /*!< Bit 1 */
#define OPAMP1_CSR_PGGAIN_2               ((uint32_t)0x00010000) /*!< Bit 2 */
#define OPAMP1_CSR_PGGAIN_3               ((uint32_t)0x00020000) /*!< Bit 3 */
#define OPAMP1_CSR_USERTRIM               ((uint32_t)0x00040000) /*!< User trimming enable */
#define OPAMP1_CSR_TRIMOFFSETP            ((uint32_t)0x00F80000) /*!< Offset trimming value (PMOS) */
#define OPAMP1_CSR_TRIMOFFSETN            ((uint32_t)0x1F000000) /*!< Offset trimming value (NMOS) */
#define OPAMP1_CSR_TSTREF                 ((uint32_t)0x20000000) /*!< It enables the switch to put out the internal reference */
#define OPAMP1_CSR_OUTCAL                 ((uint32_t)0x40000000) /*!< OPAMP ouput status flag */
#define OPAMP1_CSR_LOCK                   ((uint32_t)0x80000000) /*!< OPAMP lock */

/*********************  Bit definition for OPAMP2_CSR register  ***************/
#define OPAMP2_CSR_OPAMP2EN               ((uint32_t)0x00000001) /*!< OPAMP2 enable */
#define OPAMP2_CSR_FORCEVP                ((uint32_t)0x00000002) /*!< Connect the internal references to the plus input of the OPAMPX */
#define OPAMP2_CSR_VPSEL                  ((uint32_t)0x0000000C) /*!< Non inverting input selection */
#define OPAMP2_CSR_VPSEL_0                ((uint32_t)0x00000004) /*!< Bit 0 */
#define OPAMP2_CSR_VPSEL_1                ((uint32_t)0x00000008) /*!< Bit 1 */
#define OPAMP2_CSR_VMSEL                  ((uint32_t)0x00000060) /*!< Inverting input selection */
#define OPAMP2_CSR_VMSEL_0                ((uint32_t)0x00000020) /*!< Bit 0 */
#define OPAMP2_CSR_VMSEL_1                ((uint32_t)0x00000040) /*!< Bit 1 */
#define OPAMP2_CSR_TCMEN                  ((uint32_t)0x00000080) /*!< Timer-Controlled Mux mode enable */
#define OPAMP2_CSR_VMSSEL                 ((uint32_t)0x00000100) /*!< Inverting input secondary selection */
#define OPAMP2_CSR_VPSSEL                 ((uint32_t)0x00000600) /*!< Non inverting input secondary selection */
#define OPAMP2_CSR_VPSSEL_0               ((uint32_t)0x00000200) /*!< Bit 0 */
#define OPAMP2_CSR_VPSSEL_1               ((uint32_t)0x00000400) /*!< Bit 1 */
#define OPAMP2_CSR_CALON                  ((uint32_t)0x00000800) /*!< Calibration mode enable */
#define OPAMP2_CSR_CALSEL                 ((uint32_t)0x00003000) /*!< Calibration selection */
#define OPAMP2_CSR_CALSEL_0               ((uint32_t)0x00001000) /*!< Bit 0 */
#define OPAMP2_CSR_CALSEL_1               ((uint32_t)0x00002000) /*!< Bit 1 */
#define OPAMP2_CSR_PGGAIN                 ((uint32_t)0x0003C000) /*!< Gain in PGA mode */
#define OPAMP2_CSR_PGGAIN_0               ((uint32_t)0x00004000) /*!< Bit 0 */
#define OPAMP2_CSR_PGGAIN_1               ((uint32_t)0x00008000) /*!< Bit 1 */
#define OPAMP2_CSR_PGGAIN_2               ((uint32_t)0x00010000) /*!< Bit 2 */
#define OPAMP2_CSR_PGGAIN_3               ((uint32_t)0x00020000) /*!< Bit 3 */
#define OPAMP2_CSR_USERTRIM               ((uint32_t)0x00040000) /*!< User trimming enable */
#define OPAMP2_CSR_TRIMOFFSETP            ((uint32_t)0x00F80000) /*!< Offset trimming value (PMOS) */
#define OPAMP2_CSR_TRIMOFFSETN            ((uint32_t)0x1F000000) /*!< Offset trimming value (NMOS) */
#define OPAMP2_CSR_TSTREF                 ((uint32_t)0x20000000) /*!< It enables the switch to put out the internal reference */
#define OPAMP2_CSR_OUTCAL                 ((uint32_t)0x40000000) /*!< OPAMP ouput status flag */
#define OPAMP2_CSR_LOCK                   ((uint32_t)0x80000000) /*!< OPAMP lock */

/*********************  Bit definition for OPAMPx_CSR register  ***************/
#define OPAMP_CSR_OPAMPxEN               ((uint32_t)0x00000001) /*!< OPAMP enable */
#define OPAMP_CSR_FORCEVP                ((uint32_t)0x00000002) /*!< Connect the internal references to the plus input of the OPAMPX */
#define OPAMP_CSR_VPSEL                  ((uint32_t)0x0000000C) /*!< Non inverting input selection */
#define OPAMP_CSR_VPSEL_0                ((uint32_t)0x00000004) /*!< Bit 0 */
#define OPAMP_CSR_VPSEL_1                ((uint32_t)0x00000008) /*!< Bit 1 */
#define OPAMP_CSR_VMSEL                  ((uint32_t)0x00000060) /*!< Inverting input selection */
#define OPAMP_CSR_VMSEL_0                ((uint32_t)0x00000020) /*!< Bit 0 */
#define OPAMP_CSR_VMSEL_1                ((uint32_t)0x00000040) /*!< Bit 1 */
#define OPAMP_CSR_TCMEN                  ((uint32_t)0x00000080) /*!< Timer-Controlled Mux mode enable */
#define OPAMP_CSR_VMSSEL                 ((uint32_t)0x00000100) /*!< Inverting input secondary selection */
#define OPAMP_CSR_VPSSEL                 ((uint32_t)0x00000600) /*!< Non inverting input secondary selection */
#define OPAMP_CSR_VPSSEL_0               ((uint32_t)0x00000200) /*!< Bit 0 */
#define OPAMP_CSR_VPSSEL_1               ((uint32_t)0x00000400) /*!< Bit 1 */
#define OPAMP_CSR_CALON                  ((uint32_t)0x00000800) /*!< Calibration mode enable */
#define OPAMP_CSR_CALSEL                 ((uint32_t)0x00003000) /*!< Calibration selection */
#define OPAMP_CSR_CALSEL_0               ((uint32_t)0x00001000) /*!< Bit 0 */
#define OPAMP_CSR_CALSEL_1               ((uint32_t)0x00002000) /*!< Bit 1 */
#define OPAMP_CSR_PGGAIN                 ((uint32_t)0x0003C000) /*!< Gain in PGA mode */
#define OPAMP_CSR_PGGAIN_0               ((uint32_t)0x00004000) /*!< Bit 0 */
#define OPAMP_CSR_PGGAIN_1               ((uint32_t)0x00008000) /*!< Bit 1 */
#define OPAMP_CSR_PGGAIN_2               ((uint32_t)0x00010000) /*!< Bit 2 */
#define OPAMP_CSR_PGGAIN_3               ((uint32_t)0x00020000) /*!< Bit 3 */
#define OPAMP_CSR_USERTRIM               ((uint32_t)0x00040000) /*!< User trimming enable */
#define OPAMP_CSR_TRIMOFFSETP            ((uint32_t)0x00F80000) /*!< Offset trimming value (PMOS) */
#define OPAMP_CSR_TRIMOFFSETN            ((uint32_t)0x1F000000) /*!< Offset trimming value (NMOS) */
#define OPAMP_CSR_TSTREF                 ((uint32_t)0x20000000) /*!< It enables the switch to put out the internal reference */
#define OPAMP_CSR_OUTCAL                 ((uint32_t)0x40000000) /*!< OPAMP ouput status flag */
#define OPAMP_CSR_LOCK                   ((uint32_t)0x80000000) /*!< OPAMP lock */

/******************************************************************************/
/*                                                                            */
/*                   Controller Area Network (CAN )                           */
/*                                                                            */
/******************************************************************************/
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

/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        ((uint32_t)0x00000001)        /*!<Initialization Acknowledge */
#define  CAN_MSR_SLAK                        ((uint32_t)0x00000002)        /*!<Sleep Acknowledge */
#define  CAN_MSR_ERRI                        ((uint32_t)0x00000004)        /*!<Error Interrupt */
#define  CAN_MSR_WKUI                        ((uint32_t)0x00000008)        /*!<Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       ((uint32_t)0x00000010)        /*!<Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         ((uint32_t)0x00000100)        /*!<Transmit Mode */
#define  CAN_MSR_RXM                         ((uint32_t)0x00000200)        /*!<Receive Mode */
#define  CAN_MSR_SAMP                        ((uint32_t)0x00000400)        /*!<Last Sample Point */
#define  CAN_MSR_RX                          ((uint32_t)0x00000800)        /*!<CAN Rx Signal */

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
#define  CAN_RF0R_FMP0                       ((uint32_t)0x00000003)        /*!<FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      ((uint32_t)0x00000008)        /*!<FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      ((uint32_t)0x00000010)        /*!<FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      ((uint32_t)0x00000020)        /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       ((uint32_t)0x00000003)        /*!<FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      ((uint32_t)0x00000008)        /*!<FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      ((uint32_t)0x00000010)        /*!<FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      ((uint32_t)0x00000020)        /*!<Release FIFO 1 Output Mailbox */

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
#define  CAN_BTR_TS1_0                       ((uint32_t)0x00010000)        /*!<Time Segment 1 (Bit 0) */
#define  CAN_BTR_TS1_1                       ((uint32_t)0x00020000)        /*!<Time Segment 1 (Bit 1) */
#define  CAN_BTR_TS1_2                       ((uint32_t)0x00040000)        /*!<Time Segment 1 (Bit 2) */
#define  CAN_BTR_TS1_3                       ((uint32_t)0x00080000)        /*!<Time Segment 1 (Bit 3) */
#define  CAN_BTR_TS2                         ((uint32_t)0x00700000)        /*!<Time Segment 2 */
#define  CAN_BTR_TS2_0                       ((uint32_t)0x00100000)        /*!<Time Segment 2 (Bit 0) */
#define  CAN_BTR_TS2_1                       ((uint32_t)0x00200000)        /*!<Time Segment 2 (Bit 1) */
#define  CAN_BTR_TS2_2                       ((uint32_t)0x00400000)        /*!<Time Segment 2 (Bit 2) */
#define  CAN_BTR_SJW                         ((uint32_t)0x03000000)        /*!<Resynchronization Jump Width */
#define  CAN_BTR_SJW_0                       ((uint32_t)0x01000000)        /*!<Resynchronization Jump Width (Bit 0) */
#define  CAN_BTR_SJW_1                       ((uint32_t)0x02000000)        /*!<Resynchronization Jump Width (Bit 1) */
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
#define  CAN_FMR_FINIT                       ((uint32_t)0x00000001)        /*!<Filter Init Mode */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        ((uint32_t)0x00003FFF)        /*!<Filter Mode */
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

/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        ((uint32_t)0x00003FFF)        /*!<Filter Scale Configuration */
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

/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                       ((uint32_t)0x00003FFF)        /*!<Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                      ((uint32_t)0x00000001)        /*!<Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1R_FFA1                      ((uint32_t)0x00000002)        /*!<Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1R_FFA2                      ((uint32_t)0x00000004)        /*!<Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1R_FFA3                      ((uint32_t)0x00000008)        /*!<Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1R_FFA4                      ((uint32_t)0x00000010)        /*!<Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1R_FFA5                      ((uint32_t)0x00000020)        /*!<Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1R_FFA6                      ((uint32_t)0x00000040)        /*!<Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1R_FFA7                      ((uint32_t)0x00000080)        /*!<Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1R_FFA8                      ((uint32_t)0x00000100)        /*!<Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1R_FFA9                      ((uint32_t)0x00000200)        /*!<Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1R_FFA10                     ((uint32_t)0x00000400)        /*!<Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1R_FFA11                     ((uint32_t)0x00000800)        /*!<Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1R_FFA12                     ((uint32_t)0x00001000)        /*!<Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1R_FFA13                     ((uint32_t)0x00002000)        /*!<Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                       ((uint32_t)0x00003FFF)        /*!<Filter Active */
#define  CAN_FA1R_FACT0                      ((uint32_t)0x00000001)        /*!<Filter 0 Active */
#define  CAN_FA1R_FACT1                      ((uint32_t)0x00000002)        /*!<Filter 1 Active */
#define  CAN_FA1R_FACT2                      ((uint32_t)0x00000004)        /*!<Filter 2 Active */
#define  CAN_FA1R_FACT3                      ((uint32_t)0x00000008)        /*!<Filter 3 Active */
#define  CAN_FA1R_FACT4                      ((uint32_t)0x00000010)        /*!<Filter 4 Active */
#define  CAN_FA1R_FACT5                      ((uint32_t)0x00000020)        /*!<Filter 5 Active */
#define  CAN_FA1R_FACT6                      ((uint32_t)0x00000040)        /*!<Filter 6 Active */
#define  CAN_FA1R_FACT7                      ((uint32_t)0x00000080)        /*!<Filter 7 Active */
#define  CAN_FA1R_FACT8                      ((uint32_t)0x00000100)        /*!<Filter 8 Active */
#define  CAN_FA1R_FACT9                      ((uint32_t)0x00000200)        /*!<Filter 9 Active */
#define  CAN_FA1R_FACT10                     ((uint32_t)0x00000400)        /*!<Filter 10 Active */
#define  CAN_FA1R_FACT11                     ((uint32_t)0x00000800)        /*!<Filter 11 Active */
#define  CAN_FA1R_FACT12                     ((uint32_t)0x00001000)        /*!<Filter 12 Active */
#define  CAN_FA1R_FACT13                     ((uint32_t)0x00002000)        /*!<Filter 13 Active */

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
/*                     CRC calculation unit (CRC)                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((uint32_t)0xFFFFFFFF) /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((uint8_t)0xFF)       /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((uint32_t)0x00000001) /*!< RESET the CRC computation unit bit */
#define  CRC_CR_POLYSIZE                     ((uint32_t)0x00000018) /*!< Polynomial size bits */
#define  CRC_CR_POLYSIZE_0                   ((uint32_t)0x00000008) /*!< Polynomial size bit 0 */
#define  CRC_CR_POLYSIZE_1                   ((uint32_t)0x00000010) /*!< Polynomial size bit 1 */
#define  CRC_CR_REV_IN                       ((uint32_t)0x00000060) /*!< REV_IN Reverse Input Data bits */
#define  CRC_CR_REV_IN_0                     ((uint32_t)0x00000020) /*!< Bit 0 */
#define  CRC_CR_REV_IN_1                     ((uint32_t)0x00000040) /*!< Bit 1 */
#define  CRC_CR_REV_OUT                      ((uint32_t)0x00000080) /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
#define  CRC_INIT_INIT                       ((uint32_t)0xFFFFFFFF) /*!< Initial CRC value bits */

/*******************  Bit definition for CRC_POL register  ********************/
#define  CRC_POL_POL                         ((uint32_t)0xFFFFFFFF) /*!< Coefficients of the polynomial */

/******************************************************************************/
/*                                                                            */
/*                 Digital to Analog Converter (DAC)                          */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((uint32_t)0x00000001)        /*!< DAC channel1 enable */
#define  DAC_CR_BOFF1                        ((uint32_t)0x00000002)        /*!< DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         ((uint32_t)0x00000004)        /*!< DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((uint32_t)0x00000038)        /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  DAC_CR_TSEL1_1                      ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  DAC_CR_TSEL1_2                      ((uint32_t)0x00000020)        /*!< Bit 2 */

#define  DAC_CR_WAVE1                        ((uint32_t)0x000000C0)        /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  DAC_CR_WAVE1_1                      ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  DAC_CR_MAMP1                        ((uint32_t)0x00000F00)        /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  DAC_CR_MAMP1_1                      ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  DAC_CR_MAMP1_2                      ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  DAC_CR_MAMP1_3                      ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  DAC_CR_DMAEN1                       ((uint32_t)0x00001000)        /*!< DAC channel1 DMA enable */
#define  DAC_CR_DMAUDRIE1                    ((uint32_t)0x00002000)        /*!< DAC channel1 DMA underrun IT enable */ 
/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((uint32_t)0x00000001)        /*!< DAC channel1 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((uint32_t)0x00000FFF)        /*!< DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((uint32_t)0x0000FFF0)        /*!< DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((uint32_t)0x000000FF)        /*!< DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((uint32_t)0x00000FFF)        /*!< DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((uint32_t)0x0000FFF0)        /*!< DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((uint32_t)0x000000FF)        /*!< DAC channel1 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((uint32_t)0x00000FFF)        /*!< DAC channel1 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((uint32_t)0x00002000)        /*!< DAC channel1 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU (DBGMCU)                         */
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
#define  DBGMCU_APB1_FZ_DBG_TIM2_STOP        ((uint32_t)0x00000001)
#define  DBGMCU_APB1_FZ_DBG_TIM3_STOP        ((uint32_t)0x00000002)
#define  DBGMCU_APB1_FZ_DBG_TIM4_STOP        ((uint32_t)0x00000004)
#define  DBGMCU_APB1_FZ_DBG_TIM6_STOP        ((uint32_t)0x00000010)
#define  DBGMCU_APB1_FZ_DBG_RTC_STOP         ((uint32_t)0x00000400)
#define  DBGMCU_APB1_FZ_DBG_WWDG_STOP        ((uint32_t)0x00000800)
#define  DBGMCU_APB1_FZ_DBG_IWDG_STOP        ((uint32_t)0x00001000)
#define  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT   ((uint32_t)0x00200000)
#define  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT   ((uint32_t)0x00400000)
#define  DBGMCU_APB1_FZ_DBG_CAN_STOP         ((uint32_t)0x02000000)

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define  DBGMCU_APB2_FZ_DBG_TIM1_STOP        ((uint32_t)0x00000001)
#define  DBGMCU_APB2_FZ_DBG_TIM15_STOP       ((uint32_t)0x00000004)
#define  DBGMCU_APB2_FZ_DBG_TIM16_STOP       ((uint32_t)0x00000008)
#define  DBGMCU_APB2_FZ_DBG_TIM17_STOP       ((uint32_t)0x00000010)

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller (DMA)                           */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for DMA_ISR register  ********************/
#define  DMA_ISR_GIF1                        ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt flag */
#define  DMA_ISR_TCIF1                       ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete flag */
#define  DMA_ISR_HTIF1                       ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer flag */
#define  DMA_ISR_TEIF1                       ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error flag */
#define  DMA_ISR_GIF2                        ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt flag */
#define  DMA_ISR_TCIF2                       ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete flag */
#define  DMA_ISR_HTIF2                       ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer flag */
#define  DMA_ISR_TEIF2                       ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error flag */
#define  DMA_ISR_GIF3                        ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt flag */
#define  DMA_ISR_TCIF3                       ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete flag */
#define  DMA_ISR_HTIF3                       ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer flag */
#define  DMA_ISR_TEIF3                       ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error flag */
#define  DMA_ISR_GIF4                        ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt flag */
#define  DMA_ISR_TCIF4                       ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete flag */
#define  DMA_ISR_HTIF4                       ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer flag */
#define  DMA_ISR_TEIF4                       ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error flag */
#define  DMA_ISR_GIF5                        ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt flag */
#define  DMA_ISR_TCIF5                       ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete flag */
#define  DMA_ISR_HTIF5                       ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer flag */
#define  DMA_ISR_TEIF5                       ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error flag */
#define  DMA_ISR_GIF6                        ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt flag */
#define  DMA_ISR_TCIF6                       ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete flag */
#define  DMA_ISR_HTIF6                       ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer flag */
#define  DMA_ISR_TEIF6                       ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error flag */
#define  DMA_ISR_GIF7                        ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt flag */
#define  DMA_ISR_TCIF7                       ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete flag */
#define  DMA_ISR_HTIF7                       ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer flag */
#define  DMA_ISR_TEIF7                       ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define  DMA_IFCR_CGIF1                      ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt clear */
#define  DMA_IFCR_CTCIF1                     ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete clear */
#define  DMA_IFCR_CHTIF1                     ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer clear */
#define  DMA_IFCR_CTEIF1                     ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error clear */
#define  DMA_IFCR_CGIF2                      ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt clear */
#define  DMA_IFCR_CTCIF2                     ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete clear */
#define  DMA_IFCR_CHTIF2                     ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer clear */
#define  DMA_IFCR_CTEIF2                     ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error clear */
#define  DMA_IFCR_CGIF3                      ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt clear */
#define  DMA_IFCR_CTCIF3                     ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete clear */
#define  DMA_IFCR_CHTIF3                     ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer clear */
#define  DMA_IFCR_CTEIF3                     ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error clear */
#define  DMA_IFCR_CGIF4                      ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt clear */
#define  DMA_IFCR_CTCIF4                     ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete clear */
#define  DMA_IFCR_CHTIF4                     ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer clear */
#define  DMA_IFCR_CTEIF4                     ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error clear */
#define  DMA_IFCR_CGIF5                      ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt clear */
#define  DMA_IFCR_CTCIF5                     ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete clear */
#define  DMA_IFCR_CHTIF5                     ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer clear */
#define  DMA_IFCR_CTEIF5                     ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error clear */
#define  DMA_IFCR_CGIF6                      ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt clear */
#define  DMA_IFCR_CTCIF6                     ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete clear */
#define  DMA_IFCR_CHTIF6                     ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer clear */
#define  DMA_IFCR_CTEIF6                     ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error clear */
#define  DMA_IFCR_CGIF7                      ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt clear */
#define  DMA_IFCR_CTCIF7                     ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete clear */
#define  DMA_IFCR_CHTIF7                     ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer clear */
#define  DMA_IFCR_CTEIF7                     ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR register  ********************/
#define  DMA_CCR_EN                          ((uint32_t)0x00000001)        /*!< Channel enable                      */
#define  DMA_CCR_TCIE                        ((uint32_t)0x00000002)        /*!< Transfer complete interrupt enable  */
#define  DMA_CCR_HTIE                        ((uint32_t)0x00000004)        /*!< Half Transfer interrupt enable      */
#define  DMA_CCR_TEIE                        ((uint32_t)0x00000008)        /*!< Transfer error interrupt enable     */
#define  DMA_CCR_DIR                         ((uint32_t)0x00000010)        /*!< Data transfer direction             */
#define  DMA_CCR_CIRC                        ((uint32_t)0x00000020)        /*!< Circular mode                       */
#define  DMA_CCR_PINC                        ((uint32_t)0x00000040)        /*!< Peripheral increment mode           */
#define  DMA_CCR_MINC                        ((uint32_t)0x00000080)        /*!< Memory increment mode               */

#define  DMA_CCR_PSIZE                       ((uint32_t)0x00000300)        /*!< PSIZE[1:0] bits (Peripheral size)   */
#define  DMA_CCR_PSIZE_0                     ((uint32_t)0x00000100)        /*!< Bit 0                               */
#define  DMA_CCR_PSIZE_1                     ((uint32_t)0x00000200)        /*!< Bit 1                               */

#define  DMA_CCR_MSIZE                       ((uint32_t)0x00000C00)        /*!< MSIZE[1:0] bits (Memory size)       */
#define  DMA_CCR_MSIZE_0                     ((uint32_t)0x00000400)        /*!< Bit 0                               */
#define  DMA_CCR_MSIZE_1                     ((uint32_t)0x00000800)        /*!< Bit 1                               */

#define  DMA_CCR_PL                          ((uint32_t)0x00003000)        /*!< PL[1:0] bits(Channel Priority level)*/
#define  DMA_CCR_PL_0                        ((uint32_t)0x00001000)        /*!< Bit 0                               */
#define  DMA_CCR_PL_1                        ((uint32_t)0x00002000)        /*!< Bit 1                               */

#define  DMA_CCR_MEM2MEM                     ((uint32_t)0x00004000)        /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
#define  DMA_CNDTR_NDT                       ((uint32_t)0x0000FFFF)        /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
#define  DMA_CPAR_PA                         ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
#define  DMA_CMAR_MA                         ((uint32_t)0xFFFFFFFF)        /*!< Memory Address                      */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller (EXTI)              */
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
#define  EXTI_IMR_MR23                       ((uint32_t)0x00800000)        /*!< Interrupt Mask on line 23 */
#define  EXTI_IMR_MR24                       ((uint32_t)0x01000000)        /*!< Interrupt Mask on line 24 */
#define  EXTI_IMR_MR25                       ((uint32_t)0x02000000)        /*!< Interrupt Mask on line 25 */
#define  EXTI_IMR_MR26                       ((uint32_t)0x04000000)        /*!< Interrupt Mask on line 26 */
#define  EXTI_IMR_MR27                       ((uint32_t)0x08000000)        /*!< Interrupt Mask on line 27 */
#define  EXTI_IMR_MR28                       ((uint32_t)0x10000000)        /*!< Interrupt Mask on line 28 */
#define  EXTI_IMR_MR29                       ((uint32_t)0x20000000)        /*!< Interrupt Mask on line 29 */
#define  EXTI_IMR_MR30                       ((uint32_t)0x40000000)        /*!< Interrupt Mask on line 30 */
#define  EXTI_IMR_MR31                       ((uint32_t)0x80000000)        /*!< Interrupt Mask on line 31 */

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
#define  EXTI_EMR_MR23                       ((uint32_t)0x00800000)        /*!< Event Mask on line 23 */
#define  EXTI_EMR_MR24                       ((uint32_t)0x01000000)        /*!< Event Mask on line 24 */
#define  EXTI_EMR_MR25                       ((uint32_t)0x02000000)        /*!< Event Mask on line 25 */
#define  EXTI_EMR_MR26                       ((uint32_t)0x04000000)        /*!< Event Mask on line 26 */
#define  EXTI_EMR_MR27                       ((uint32_t)0x08000000)        /*!< Event Mask on line 27 */
#define  EXTI_EMR_MR28                       ((uint32_t)0x10000000)        /*!< Event Mask on line 28 */
#define  EXTI_EMR_MR29                       ((uint32_t)0x20000000)        /*!< Event Mask on line 29 */
#define  EXTI_EMR_MR30                       ((uint32_t)0x40000000)        /*!< Event Mask on line 30 */
#define  EXTI_EMR_MR31                       ((uint32_t)0x80000000)        /*!< Event Mask on line 31 */

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
#define  EXTI_RTSR_TR29                      ((uint32_t)0x20000000)        /*!< Rising trigger event configuration bit of line 29 */
#define  EXTI_RTSR_TR30                      ((uint32_t)0x40000000)        /*!< Rising trigger event configuration bit of line 30 */
#define  EXTI_RTSR_TR31                      ((uint32_t)0x80000000)        /*!< Rising trigger event configuration bit of line 31 */

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
#define  EXTI_FTSR_TR29                      ((uint32_t)0x20000000)        /*!< Falling trigger event configuration bit of line 29 */
#define  EXTI_FTSR_TR30                      ((uint32_t)0x40000000)        /*!< Falling trigger event configuration bit of line 30 */
#define  EXTI_FTSR_TR31                      ((uint32_t)0x80000000)        /*!< Falling trigger event configuration bit of line 31 */

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
#define  EXTI_SWIER_SWIER29                  ((uint32_t)0x20000000)        /*!< Software Interrupt on line 29 */
#define  EXTI_SWIER_SWIER30                  ((uint32_t)0x40000000)        /*!< Software Interrupt on line 30 */
#define  EXTI_SWIER_SWIER31                  ((uint32_t)0x80000000)        /*!< Software Interrupt on line 31 */

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
#define  EXTI_PR_PR29                        ((uint32_t)0x20000000)        /*!< Pending bit for line 29 */
#define  EXTI_PR_PR30                        ((uint32_t)0x40000000)        /*!< Pending bit for line 30 */
#define  EXTI_PR_PR31                        ((uint32_t)0x80000000)        /*!< Pending bit for line 31 */

/*******************  Bit definition for EXTI_IMR2 register  ******************/
#define  EXTI_IMR2_MR32                      ((uint32_t)0x00000001)        /*!< Interrupt Mask on line 32 */
#define  EXTI_IMR2_MR33                      ((uint32_t)0x00000002)        /*!< Interrupt Mask on line 33 */
#define  EXTI_IMR2_MR34                      ((uint32_t)0x00000004)        /*!< Interrupt Mask on line 34 */
#define  EXTI_IMR2_MR35                      ((uint32_t)0x00000008)        /*!< Interrupt Mask on line 35 */

/*******************  Bit definition for EXTI_EMR2 ****************************/
#define  EXTI_EMR2_MR32                      ((uint32_t)0x00000001)        /*!< Event Mask on line 32 */
#define  EXTI_EMR2_MR33                      ((uint32_t)0x00000002)        /*!< Event Mask on line 33 */
#define  EXTI_EMR2_MR34                      ((uint32_t)0x00000004)        /*!< Event Mask on line 34 */
#define  EXTI_EMR2_MR35                      ((uint32_t)0x00000008)        /*!< Event Mask on line 34 */

/******************  Bit definition for EXTI_RTSR2 register ********************/
#define  EXTI_RTSR2_TR32                     ((uint32_t)0x00000001)         /*!< Rising trigger event configuration bit of line 32 */
#define  EXTI_RTSR2_TR33                     ((uint32_t)0x00000002)         /*!< Rising trigger event configuration bit of line 33 */

/******************  Bit definition for EXTI_FTSR2 register  ******************/
#define  EXTI_FTSR2_TR32                     ((uint32_t)0x00000001)        /*!< Falling trigger event configuration bit of line 32 */
#define  EXTI_FTSR2_TR33                     ((uint32_t)0x00000002)        /*!< Falling trigger event configuration bit of line 33 */

/******************  Bit definition for EXTI_SWIER2 register  *****************/
#define  EXTI_SWIER2_SWIER32                 ((uint32_t)0x00000001)        /*!< Software Interrupt on line 32 */
#define  EXTI_SWIER2_SWIER33                 ((uint32_t)0x00000002)        /*!< Software Interrupt on line 33 */

/*******************  Bit definition for EXTI_PR2 register  *******************/
#define  EXTI_PR2_PR32                       ((uint32_t)0x00000001)        /*!< Pending bit for line 32 */
#define  EXTI_PR2_PR33                       ((uint32_t)0x00000002)        /*!< Pending bit for line 33 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for FLASH_ACR register  ******************/
#define  FLASH_ACR_LATENCY                   ((uint32_t)0x00000007)        /*!< LATENCY[2:0] bits (Latency) */
#define  FLASH_ACR_LATENCY_0                 ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  FLASH_ACR_LATENCY_1                 ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  FLASH_ACR_LATENCY_2                 ((uint32_t)0x00000004)        /*!< Bit 2 */

#define  FLASH_ACR_HLFCYA                    ((uint32_t)0x00000008)        /*!< Flash Half Cycle Access Enable */
#define  FLASH_ACR_PRFTBE                    ((uint32_t)0x00000010)        /*!< Prefetch Buffer Enable */
#define  FLASH_ACR_PRFTBS                    ((uint32_t)0x00000020)        /*!< Prefetch Buffer Status */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define  FLASH_KEYR_FKEYR                    ((uint32_t)0xFFFFFFFF)        /*!< FPEC Key */

#define  RDP_KEY                             ((uint32_t)0x000000A5)        /*!< RDP Key */
#define  FLASH_KEY1                          ((uint32_t)0x45670123)        /*!< FPEC Key1 */
#define  FLASH_KEY2                          ((uint32_t)0xCDEF89AB)        /*!< FPEC Key2 */

/*****************  Bit definition for FLASH_OPTKEYR register  ****************/
#define  FLASH_OPTKEYR_OPTKEYR               ((uint32_t)0xFFFFFFFF)        /*!< Option Byte Key */

#define  FLASH_OPTKEY1                       FLASH_KEY1                    /*!< Option Byte Key1 */
#define  FLASH_OPTKEY2                       FLASH_KEY2                    /*!< Option Byte Key2 */

/******************  Bit definition for FLASH_SR register  *******************/
#define  FLASH_SR_BSY                        ((uint32_t)0x00000001)        /*!< Busy */
#define  FLASH_SR_PGERR                      ((uint32_t)0x00000004)        /*!< Programming Error */
#define  FLASH_SR_WRPERR                     ((uint32_t)0x00000010)        /*!< Write Protection Error */
#define  FLASH_SR_EOP                        ((uint32_t)0x00000020)        /*!< End of operation */

/*******************  Bit definition for FLASH_CR register  *******************/
#define  FLASH_CR_PG                         ((uint32_t)0x00000001)        /*!< Programming */
#define  FLASH_CR_PER                        ((uint32_t)0x00000002)        /*!< Page Erase */
#define  FLASH_CR_MER                        ((uint32_t)0x00000004)        /*!< Mass Erase */
#define  FLASH_CR_OPTPG                      ((uint32_t)0x00000010)        /*!< Option Byte Programming */
#define  FLASH_CR_OPTER                      ((uint32_t)0x00000020)        /*!< Option Byte Erase */
#define  FLASH_CR_STRT                       ((uint32_t)0x00000040)        /*!< Start */
#define  FLASH_CR_LOCK                       ((uint32_t)0x00000080)        /*!< Lock */
#define  FLASH_CR_OPTWRE                     ((uint32_t)0x00000200)        /*!< Option Bytes Write Enable */
#define  FLASH_CR_ERRIE                      ((uint32_t)0x00000400)        /*!< Error Interrupt Enable */
#define  FLASH_CR_EOPIE                      ((uint32_t)0x00001000)        /*!< End of operation interrupt enable */
#define  FLASH_CR_OBL_LAUNCH                 ((uint32_t)0x00002000)        /*!< OptionBytes Loader Launch */

/*******************  Bit definition for FLASH_AR register  *******************/
#define  FLASH_AR_FAR                        ((uint32_t)0xFFFFFFFF)        /*!< Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define  FLASH_OBR_OPTERR                    ((uint32_t)0x00000001)        /*!< Option Byte Error */
#define  FLASH_OBR_RDPRT                     ((uint32_t)0x00000006)        /*!< Read protection */
#define  FLASH_OBR_RDPRT_1                   ((uint32_t)0x00000002)        /*!< Read protection Level 1 */
#define  FLASH_OBR_RDPRT_2                   ((uint32_t)0x00000006)        /*!< Read protection Level 2 */

#define  FLASH_OBR_USER                      ((uint32_t)0x00007700)        /*!< User Option Bytes */
#define  FLASH_OBR_IWDG_SW                   ((uint32_t)0x00000100)        /*!< IWDG SW */
#define  FLASH_OBR_nRST_STOP                 ((uint32_t)0x00000200)        /*!< nRST_STOP */
#define  FLASH_OBR_nRST_STDBY                ((uint32_t)0x00000400)        /*!< nRST_STDBY */
#define  FLASH_OBR_nBOOT1                    ((uint32_t)0x00001000)        /*!< nBOOT1 */
#define  FLASH_OBR_VDDA_MONITOR              ((uint32_t)0x00002000)        /*!< VDDA_MONITOR */
#define  FLASH_OBR_SRAM_PE                   ((uint32_t)0x00004000)        /*!< SRAM_PE */
#define  FLASH_OBR_DATA0                     ((uint32_t)0x00FF0000) /*!< Data0 */
#define  FLASH_OBR_DATA1                     ((uint32_t)0xFF000000) /*!< Data1 */

/******************  Bit definition for FLASH_WRPR register  ******************/
#define  FLASH_WRPR_WRP                        ((uint32_t)0xFFFFFFFF)      /*!< Write Protect */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for OB_RDP register  **********************/
#define  OB_RDP_RDP                          ((uint32_t)0x000000FF)        /*!< Read protection option byte */
#define  OB_RDP_nRDP                         ((uint32_t)0x0000FF00)        /*!< Read protection complemented option byte */

/******************  Bit definition for OB_USER register  *********************/
#define  OB_USER_USER                        ((uint32_t)0x00FF0000)        /*!< User option byte */
#define  OB_USER_nUSER                       ((uint32_t)0xFF000000)        /*!< User complemented option byte */

/******************  Bit definition for FLASH_WRP0 register  ******************/
#define  OB_WRP0_WRP0                        ((uint32_t)0x000000FF)        /*!< Flash memory write protection option bytes */
#define  OB_WRP0_nWRP0                       ((uint32_t)0x0000FF00)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP1 register  ******************/
#define  OB_WRP1_WRP1                        ((uint32_t)0x00FF0000)        /*!< Flash memory write protection option bytes */
#define  OB_WRP1_nWRP1                       ((uint32_t)0xFF000000)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP2 register  ******************/
#define  OB_WRP2_WRP2                        ((uint32_t)0x000000FF)        /*!< Flash memory write protection option bytes */
#define  OB_WRP2_nWRP2                       ((uint32_t)0x0000FF00)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRP3 register  ******************/
#define  OB_WRP3_WRP3                        ((uint32_t)0x00FF0000)        /*!< Flash memory write protection option bytes */
#define  OB_WRP3_nWRP3                       ((uint32_t)0xFF000000)        /*!< Flash memory write protection complemented option bytes */

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O (GPIO)                      */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0          ((uint32_t)0x00000003)
#define GPIO_MODER_MODER0_0        ((uint32_t)0x00000001)
#define GPIO_MODER_MODER0_1        ((uint32_t)0x00000002)
#define GPIO_MODER_MODER1          ((uint32_t)0x0000000C)
#define GPIO_MODER_MODER1_0        ((uint32_t)0x00000004)
#define GPIO_MODER_MODER1_1        ((uint32_t)0x00000008)
#define GPIO_MODER_MODER2          ((uint32_t)0x00000030)
#define GPIO_MODER_MODER2_0        ((uint32_t)0x00000010)
#define GPIO_MODER_MODER2_1        ((uint32_t)0x00000020)
#define GPIO_MODER_MODER3          ((uint32_t)0x000000C0)
#define GPIO_MODER_MODER3_0        ((uint32_t)0x00000040)
#define GPIO_MODER_MODER3_1        ((uint32_t)0x00000080)
#define GPIO_MODER_MODER4          ((uint32_t)0x00000300)
#define GPIO_MODER_MODER4_0        ((uint32_t)0x00000100)
#define GPIO_MODER_MODER4_1        ((uint32_t)0x00000200)
#define GPIO_MODER_MODER5          ((uint32_t)0x00000C00)
#define GPIO_MODER_MODER5_0        ((uint32_t)0x00000400)
#define GPIO_MODER_MODER5_1        ((uint32_t)0x00000800)
#define GPIO_MODER_MODER6          ((uint32_t)0x00003000)
#define GPIO_MODER_MODER6_0        ((uint32_t)0x00001000)
#define GPIO_MODER_MODER6_1        ((uint32_t)0x00002000)
#define GPIO_MODER_MODER7          ((uint32_t)0x0000C000)
#define GPIO_MODER_MODER7_0        ((uint32_t)0x00004000)
#define GPIO_MODER_MODER7_1        ((uint32_t)0x00008000)
#define GPIO_MODER_MODER8          ((uint32_t)0x00030000)
#define GPIO_MODER_MODER8_0        ((uint32_t)0x00010000)
#define GPIO_MODER_MODER8_1        ((uint32_t)0x00020000)
#define GPIO_MODER_MODER9          ((uint32_t)0x000C0000)
#define GPIO_MODER_MODER9_0        ((uint32_t)0x00040000)
#define GPIO_MODER_MODER9_1        ((uint32_t)0x00080000)
#define GPIO_MODER_MODER10         ((uint32_t)0x00300000)
#define GPIO_MODER_MODER10_0       ((uint32_t)0x00100000)
#define GPIO_MODER_MODER10_1       ((uint32_t)0x00200000)
#define GPIO_MODER_MODER11         ((uint32_t)0x00C00000)
#define GPIO_MODER_MODER11_0       ((uint32_t)0x00400000)
#define GPIO_MODER_MODER11_1       ((uint32_t)0x00800000)
#define GPIO_MODER_MODER12         ((uint32_t)0x03000000)
#define GPIO_MODER_MODER12_0       ((uint32_t)0x01000000)
#define GPIO_MODER_MODER12_1       ((uint32_t)0x02000000)
#define GPIO_MODER_MODER13         ((uint32_t)0x0C000000)
#define GPIO_MODER_MODER13_0       ((uint32_t)0x04000000)
#define GPIO_MODER_MODER13_1       ((uint32_t)0x08000000)
#define GPIO_MODER_MODER14         ((uint32_t)0x30000000)
#define GPIO_MODER_MODER14_0       ((uint32_t)0x10000000)
#define GPIO_MODER_MODER14_1       ((uint32_t)0x20000000)
#define GPIO_MODER_MODER15         ((uint32_t)0xC0000000)
#define GPIO_MODER_MODER15_0       ((uint32_t)0x40000000)
#define GPIO_MODER_MODER15_1       ((uint32_t)0x80000000)

/******************  Bit definition for GPIO_OTYPER register  *****************/
#define GPIO_OTYPER_OT_0           ((uint32_t)0x00000001)
#define GPIO_OTYPER_OT_1           ((uint32_t)0x00000002)
#define GPIO_OTYPER_OT_2           ((uint32_t)0x00000004)
#define GPIO_OTYPER_OT_3           ((uint32_t)0x00000008)
#define GPIO_OTYPER_OT_4           ((uint32_t)0x00000010)
#define GPIO_OTYPER_OT_5           ((uint32_t)0x00000020)
#define GPIO_OTYPER_OT_6           ((uint32_t)0x00000040)
#define GPIO_OTYPER_OT_7           ((uint32_t)0x00000080)
#define GPIO_OTYPER_OT_8           ((uint32_t)0x00000100)
#define GPIO_OTYPER_OT_9           ((uint32_t)0x00000200)
#define GPIO_OTYPER_OT_10          ((uint32_t)0x00000400)
#define GPIO_OTYPER_OT_11          ((uint32_t)0x00000800)
#define GPIO_OTYPER_OT_12          ((uint32_t)0x00001000)
#define GPIO_OTYPER_OT_13          ((uint32_t)0x00002000)
#define GPIO_OTYPER_OT_14          ((uint32_t)0x00004000)
#define GPIO_OTYPER_OT_15          ((uint32_t)0x00008000)

/****************  Bit definition for GPIO_OSPEEDR register  ******************/
#define GPIO_OSPEEDER_OSPEEDR0     ((uint32_t)0x00000003)
#define GPIO_OSPEEDER_OSPEEDR0_0   ((uint32_t)0x00000001)
#define GPIO_OSPEEDER_OSPEEDR0_1   ((uint32_t)0x00000002)
#define GPIO_OSPEEDER_OSPEEDR1     ((uint32_t)0x0000000C)
#define GPIO_OSPEEDER_OSPEEDR1_0   ((uint32_t)0x00000004)
#define GPIO_OSPEEDER_OSPEEDR1_1   ((uint32_t)0x00000008)
#define GPIO_OSPEEDER_OSPEEDR2     ((uint32_t)0x00000030)
#define GPIO_OSPEEDER_OSPEEDR2_0   ((uint32_t)0x00000010)
#define GPIO_OSPEEDER_OSPEEDR2_1   ((uint32_t)0x00000020)
#define GPIO_OSPEEDER_OSPEEDR3     ((uint32_t)0x000000C0)
#define GPIO_OSPEEDER_OSPEEDR3_0   ((uint32_t)0x00000040)
#define GPIO_OSPEEDER_OSPEEDR3_1   ((uint32_t)0x00000080)
#define GPIO_OSPEEDER_OSPEEDR4     ((uint32_t)0x00000300)
#define GPIO_OSPEEDER_OSPEEDR4_0   ((uint32_t)0x00000100)
#define GPIO_OSPEEDER_OSPEEDR4_1   ((uint32_t)0x00000200)
#define GPIO_OSPEEDER_OSPEEDR5     ((uint32_t)0x00000C00)
#define GPIO_OSPEEDER_OSPEEDR5_0   ((uint32_t)0x00000400)
#define GPIO_OSPEEDER_OSPEEDR5_1   ((uint32_t)0x00000800)
#define GPIO_OSPEEDER_OSPEEDR6     ((uint32_t)0x00003000)
#define GPIO_OSPEEDER_OSPEEDR6_0   ((uint32_t)0x00001000)
#define GPIO_OSPEEDER_OSPEEDR6_1   ((uint32_t)0x00002000)
#define GPIO_OSPEEDER_OSPEEDR7     ((uint32_t)0x0000C000)
#define GPIO_OSPEEDER_OSPEEDR7_0   ((uint32_t)0x00004000)
#define GPIO_OSPEEDER_OSPEEDR7_1   ((uint32_t)0x00008000)
#define GPIO_OSPEEDER_OSPEEDR8     ((uint32_t)0x00030000)
#define GPIO_OSPEEDER_OSPEEDR8_0   ((uint32_t)0x00010000)
#define GPIO_OSPEEDER_OSPEEDR8_1   ((uint32_t)0x00020000)
#define GPIO_OSPEEDER_OSPEEDR9     ((uint32_t)0x000C0000)
#define GPIO_OSPEEDER_OSPEEDR9_0   ((uint32_t)0x00040000)
#define GPIO_OSPEEDER_OSPEEDR9_1   ((uint32_t)0x00080000)
#define GPIO_OSPEEDER_OSPEEDR10    ((uint32_t)0x00300000)
#define GPIO_OSPEEDER_OSPEEDR10_0  ((uint32_t)0x00100000)
#define GPIO_OSPEEDER_OSPEEDR10_1  ((uint32_t)0x00200000)
#define GPIO_OSPEEDER_OSPEEDR11    ((uint32_t)0x00C00000)
#define GPIO_OSPEEDER_OSPEEDR11_0  ((uint32_t)0x00400000)
#define GPIO_OSPEEDER_OSPEEDR11_1  ((uint32_t)0x00800000)
#define GPIO_OSPEEDER_OSPEEDR12    ((uint32_t)0x03000000)
#define GPIO_OSPEEDER_OSPEEDR12_0  ((uint32_t)0x01000000)
#define GPIO_OSPEEDER_OSPEEDR12_1  ((uint32_t)0x02000000)
#define GPIO_OSPEEDER_OSPEEDR13    ((uint32_t)0x0C000000)
#define GPIO_OSPEEDER_OSPEEDR13_0  ((uint32_t)0x04000000)
#define GPIO_OSPEEDER_OSPEEDR13_1  ((uint32_t)0x08000000)
#define GPIO_OSPEEDER_OSPEEDR14    ((uint32_t)0x30000000)
#define GPIO_OSPEEDER_OSPEEDR14_0  ((uint32_t)0x10000000)
#define GPIO_OSPEEDER_OSPEEDR14_1  ((uint32_t)0x20000000)
#define GPIO_OSPEEDER_OSPEEDR15    ((uint32_t)0xC0000000)
#define GPIO_OSPEEDER_OSPEEDR15_0  ((uint32_t)0x40000000)
#define GPIO_OSPEEDER_OSPEEDR15_1  ((uint32_t)0x80000000)

/*******************  Bit definition for GPIO_PUPDR register ******************/
#define GPIO_PUPDR_PUPDR0          ((uint32_t)0x00000003)
#define GPIO_PUPDR_PUPDR0_0        ((uint32_t)0x00000001)
#define GPIO_PUPDR_PUPDR0_1        ((uint32_t)0x00000002)
#define GPIO_PUPDR_PUPDR1          ((uint32_t)0x0000000C)
#define GPIO_PUPDR_PUPDR1_0        ((uint32_t)0x00000004)
#define GPIO_PUPDR_PUPDR1_1        ((uint32_t)0x00000008)
#define GPIO_PUPDR_PUPDR2          ((uint32_t)0x00000030)
#define GPIO_PUPDR_PUPDR2_0        ((uint32_t)0x00000010)
#define GPIO_PUPDR_PUPDR2_1        ((uint32_t)0x00000020)
#define GPIO_PUPDR_PUPDR3          ((uint32_t)0x000000C0)
#define GPIO_PUPDR_PUPDR3_0        ((uint32_t)0x00000040)
#define GPIO_PUPDR_PUPDR3_1        ((uint32_t)0x00000080)
#define GPIO_PUPDR_PUPDR4          ((uint32_t)0x00000300)
#define GPIO_PUPDR_PUPDR4_0        ((uint32_t)0x00000100)
#define GPIO_PUPDR_PUPDR4_1        ((uint32_t)0x00000200)
#define GPIO_PUPDR_PUPDR5          ((uint32_t)0x00000C00)
#define GPIO_PUPDR_PUPDR5_0        ((uint32_t)0x00000400)
#define GPIO_PUPDR_PUPDR5_1        ((uint32_t)0x00000800)
#define GPIO_PUPDR_PUPDR6          ((uint32_t)0x00003000)
#define GPIO_PUPDR_PUPDR6_0        ((uint32_t)0x00001000)
#define GPIO_PUPDR_PUPDR6_1        ((uint32_t)0x00002000)
#define GPIO_PUPDR_PUPDR7          ((uint32_t)0x0000C000)
#define GPIO_PUPDR_PUPDR7_0        ((uint32_t)0x00004000)
#define GPIO_PUPDR_PUPDR7_1        ((uint32_t)0x00008000)
#define GPIO_PUPDR_PUPDR8          ((uint32_t)0x00030000)
#define GPIO_PUPDR_PUPDR8_0        ((uint32_t)0x00010000)
#define GPIO_PUPDR_PUPDR8_1        ((uint32_t)0x00020000)
#define GPIO_PUPDR_PUPDR9          ((uint32_t)0x000C0000)
#define GPIO_PUPDR_PUPDR9_0        ((uint32_t)0x00040000)
#define GPIO_PUPDR_PUPDR9_1        ((uint32_t)0x00080000)
#define GPIO_PUPDR_PUPDR10         ((uint32_t)0x00300000)
#define GPIO_PUPDR_PUPDR10_0       ((uint32_t)0x00100000)
#define GPIO_PUPDR_PUPDR10_1       ((uint32_t)0x00200000)
#define GPIO_PUPDR_PUPDR11         ((uint32_t)0x00C00000)
#define GPIO_PUPDR_PUPDR11_0       ((uint32_t)0x00400000)
#define GPIO_PUPDR_PUPDR11_1       ((uint32_t)0x00800000)
#define GPIO_PUPDR_PUPDR12         ((uint32_t)0x03000000)
#define GPIO_PUPDR_PUPDR12_0       ((uint32_t)0x01000000)
#define GPIO_PUPDR_PUPDR12_1       ((uint32_t)0x02000000)
#define GPIO_PUPDR_PUPDR13         ((uint32_t)0x0C000000)
#define GPIO_PUPDR_PUPDR13_0       ((uint32_t)0x04000000)
#define GPIO_PUPDR_PUPDR13_1       ((uint32_t)0x08000000)
#define GPIO_PUPDR_PUPDR14         ((uint32_t)0x30000000)
#define GPIO_PUPDR_PUPDR14_0       ((uint32_t)0x10000000)
#define GPIO_PUPDR_PUPDR14_1       ((uint32_t)0x20000000)
#define GPIO_PUPDR_PUPDR15         ((uint32_t)0xC0000000)
#define GPIO_PUPDR_PUPDR15_0       ((uint32_t)0x40000000)
#define GPIO_PUPDR_PUPDR15_1       ((uint32_t)0x80000000)

/*******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_0                 ((uint32_t)0x00000001)
#define GPIO_IDR_1                 ((uint32_t)0x00000002)
#define GPIO_IDR_2                 ((uint32_t)0x00000004)
#define GPIO_IDR_3                 ((uint32_t)0x00000008)
#define GPIO_IDR_4                 ((uint32_t)0x00000010)
#define GPIO_IDR_5                 ((uint32_t)0x00000020)
#define GPIO_IDR_6                 ((uint32_t)0x00000040)
#define GPIO_IDR_7                 ((uint32_t)0x00000080)
#define GPIO_IDR_8                 ((uint32_t)0x00000100)
#define GPIO_IDR_9                 ((uint32_t)0x00000200)
#define GPIO_IDR_10                ((uint32_t)0x00000400)
#define GPIO_IDR_11                ((uint32_t)0x00000800)
#define GPIO_IDR_12                ((uint32_t)0x00001000)
#define GPIO_IDR_13                ((uint32_t)0x00002000)
#define GPIO_IDR_14                ((uint32_t)0x00004000)
#define GPIO_IDR_15                ((uint32_t)0x00008000)

/******************  Bit definition for GPIO_ODR register  ********************/
#define GPIO_ODR_0                 ((uint32_t)0x00000001)
#define GPIO_ODR_1                 ((uint32_t)0x00000002)
#define GPIO_ODR_2                 ((uint32_t)0x00000004)
#define GPIO_ODR_3                 ((uint32_t)0x00000008)
#define GPIO_ODR_4                 ((uint32_t)0x00000010)
#define GPIO_ODR_5                 ((uint32_t)0x00000020)
#define GPIO_ODR_6                 ((uint32_t)0x00000040)
#define GPIO_ODR_7                 ((uint32_t)0x00000080)
#define GPIO_ODR_8                 ((uint32_t)0x00000100)
#define GPIO_ODR_9                 ((uint32_t)0x00000200)
#define GPIO_ODR_10                ((uint32_t)0x00000400)
#define GPIO_ODR_11                ((uint32_t)0x00000800)
#define GPIO_ODR_12                ((uint32_t)0x00001000)
#define GPIO_ODR_13                ((uint32_t)0x00002000)
#define GPIO_ODR_14                ((uint32_t)0x00004000)
#define GPIO_ODR_15                ((uint32_t)0x00008000)

/****************** Bit definition for GPIO_BSRR register  ********************/
#define GPIO_BSRR_BS_0             ((uint32_t)0x00000001)
#define GPIO_BSRR_BS_1             ((uint32_t)0x00000002)
#define GPIO_BSRR_BS_2             ((uint32_t)0x00000004)
#define GPIO_BSRR_BS_3             ((uint32_t)0x00000008)
#define GPIO_BSRR_BS_4             ((uint32_t)0x00000010)
#define GPIO_BSRR_BS_5             ((uint32_t)0x00000020)
#define GPIO_BSRR_BS_6             ((uint32_t)0x00000040)
#define GPIO_BSRR_BS_7             ((uint32_t)0x00000080)
#define GPIO_BSRR_BS_8             ((uint32_t)0x00000100)
#define GPIO_BSRR_BS_9             ((uint32_t)0x00000200)
#define GPIO_BSRR_BS_10            ((uint32_t)0x00000400)
#define GPIO_BSRR_BS_11            ((uint32_t)0x00000800)
#define GPIO_BSRR_BS_12            ((uint32_t)0x00001000)
#define GPIO_BSRR_BS_13            ((uint32_t)0x00002000)
#define GPIO_BSRR_BS_14            ((uint32_t)0x00004000)
#define GPIO_BSRR_BS_15            ((uint32_t)0x00008000)
#define GPIO_BSRR_BR_0             ((uint32_t)0x00010000)
#define GPIO_BSRR_BR_1             ((uint32_t)0x00020000)
#define GPIO_BSRR_BR_2             ((uint32_t)0x00040000)
#define GPIO_BSRR_BR_3             ((uint32_t)0x00080000)
#define GPIO_BSRR_BR_4             ((uint32_t)0x00100000)
#define GPIO_BSRR_BR_5             ((uint32_t)0x00200000)
#define GPIO_BSRR_BR_6             ((uint32_t)0x00400000)
#define GPIO_BSRR_BR_7             ((uint32_t)0x00800000)
#define GPIO_BSRR_BR_8             ((uint32_t)0x01000000)
#define GPIO_BSRR_BR_9             ((uint32_t)0x02000000)
#define GPIO_BSRR_BR_10            ((uint32_t)0x04000000)
#define GPIO_BSRR_BR_11            ((uint32_t)0x08000000)
#define GPIO_BSRR_BR_12            ((uint32_t)0x10000000)
#define GPIO_BSRR_BR_13            ((uint32_t)0x20000000)
#define GPIO_BSRR_BR_14            ((uint32_t)0x40000000)
#define GPIO_BSRR_BR_15            ((uint32_t)0x80000000)

/****************** Bit definition for GPIO_LCKR register  ********************/
#define GPIO_LCKR_LCK0             ((uint32_t)0x00000001)
#define GPIO_LCKR_LCK1             ((uint32_t)0x00000002)
#define GPIO_LCKR_LCK2             ((uint32_t)0x00000004)
#define GPIO_LCKR_LCK3             ((uint32_t)0x00000008)
#define GPIO_LCKR_LCK4             ((uint32_t)0x00000010)
#define GPIO_LCKR_LCK5             ((uint32_t)0x00000020)
#define GPIO_LCKR_LCK6             ((uint32_t)0x00000040)
#define GPIO_LCKR_LCK7             ((uint32_t)0x00000080)
#define GPIO_LCKR_LCK8             ((uint32_t)0x00000100)
#define GPIO_LCKR_LCK9             ((uint32_t)0x00000200)
#define GPIO_LCKR_LCK10            ((uint32_t)0x00000400)
#define GPIO_LCKR_LCK11            ((uint32_t)0x00000800)
#define GPIO_LCKR_LCK12            ((uint32_t)0x00001000)
#define GPIO_LCKR_LCK13            ((uint32_t)0x00002000)
#define GPIO_LCKR_LCK14            ((uint32_t)0x00004000)
#define GPIO_LCKR_LCK15            ((uint32_t)0x00008000)
#define GPIO_LCKR_LCKK             ((uint32_t)0x00010000)

/****************** Bit definition for GPIO_AFRL register  ********************/
#define GPIO_AFRL_AFRL0            ((uint32_t)0x0000000F)
#define GPIO_AFRL_AFRL1            ((uint32_t)0x000000F0)
#define GPIO_AFRL_AFRL2            ((uint32_t)0x00000F00)
#define GPIO_AFRL_AFRL3            ((uint32_t)0x0000F000)
#define GPIO_AFRL_AFRL4            ((uint32_t)0x000F0000)
#define GPIO_AFRL_AFRL5            ((uint32_t)0x00F00000)
#define GPIO_AFRL_AFRL6            ((uint32_t)0x0F000000)
#define GPIO_AFRL_AFRL7            ((uint32_t)0xF0000000)

/****************** Bit definition for GPIO_AFRH register  ********************/
#define GPIO_AFRH_AFRH0            ((uint32_t)0x0000000F)
#define GPIO_AFRH_AFRH1            ((uint32_t)0x000000F0)
#define GPIO_AFRH_AFRH2            ((uint32_t)0x00000F00)
#define GPIO_AFRH_AFRH3            ((uint32_t)0x0000F000)
#define GPIO_AFRH_AFRH4            ((uint32_t)0x000F0000)
#define GPIO_AFRH_AFRH5            ((uint32_t)0x00F00000)
#define GPIO_AFRH_AFRH6            ((uint32_t)0x0F000000)
#define GPIO_AFRH_AFRH7            ((uint32_t)0xF0000000)

/****************** Bit definition for GPIO_BRR register  *********************/
#define GPIO_BRR_BR_0              ((uint32_t)0x00000001)
#define GPIO_BRR_BR_1              ((uint32_t)0x00000002)
#define GPIO_BRR_BR_2              ((uint32_t)0x00000004)
#define GPIO_BRR_BR_3              ((uint32_t)0x00000008)
#define GPIO_BRR_BR_4              ((uint32_t)0x00000010)
#define GPIO_BRR_BR_5              ((uint32_t)0x00000020)
#define GPIO_BRR_BR_6              ((uint32_t)0x00000040)
#define GPIO_BRR_BR_7              ((uint32_t)0x00000080)
#define GPIO_BRR_BR_8              ((uint32_t)0x00000100)
#define GPIO_BRR_BR_9              ((uint32_t)0x00000200)
#define GPIO_BRR_BR_10             ((uint32_t)0x00000400)
#define GPIO_BRR_BR_11             ((uint32_t)0x00000800)
#define GPIO_BRR_BR_12             ((uint32_t)0x00001000)
#define GPIO_BRR_BR_13             ((uint32_t)0x00002000)
#define GPIO_BRR_BR_14             ((uint32_t)0x00004000)
#define GPIO_BRR_BR_15             ((uint32_t)0x00008000)

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface (I2C)              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
#define  I2C_CR1_PE                          ((uint32_t)0x00000001)        /*!< Peripheral enable */
#define  I2C_CR1_TXIE                        ((uint32_t)0x00000002)        /*!< TX interrupt enable */
#define  I2C_CR1_RXIE                        ((uint32_t)0x00000004)        /*!< RX interrupt enable */
#define  I2C_CR1_ADDRIE                      ((uint32_t)0x00000008)        /*!< Address match interrupt enable */
#define  I2C_CR1_NACKIE                      ((uint32_t)0x00000010)        /*!< NACK received interrupt enable */
#define  I2C_CR1_STOPIE                      ((uint32_t)0x00000020)        /*!< STOP detection interrupt enable */
#define  I2C_CR1_TCIE                        ((uint32_t)0x00000040)        /*!< Transfer complete interrupt enable */
#define  I2C_CR1_ERRIE                       ((uint32_t)0x00000080)        /*!< Errors interrupt enable */
#define  I2C_CR1_DNF                         ((uint32_t)0x00000F00)        /*!< Digital noise filter */
#define  I2C_CR1_ANFOFF                      ((uint32_t)0x00001000)        /*!< Analog noise filter OFF */
#define  I2C_CR1_SWRST                       ((uint32_t)0x00002000)        /*!< Software reset */
#define  I2C_CR1_TXDMAEN                     ((uint32_t)0x00004000)        /*!< DMA transmission requests enable */
#define  I2C_CR1_RXDMAEN                     ((uint32_t)0x00008000)        /*!< DMA reception requests enable */
#define  I2C_CR1_SBC                         ((uint32_t)0x00010000)        /*!< Slave byte control */
#define  I2C_CR1_NOSTRETCH                   ((uint32_t)0x00020000)        /*!< Clock stretching disable */
#define  I2C_CR1_WUPEN                       ((uint32_t)0x00040000)        /*!< Wakeup from STOP enable */
#define  I2C_CR1_GCEN                        ((uint32_t)0x00080000)        /*!< General call enable */
#define  I2C_CR1_SMBHEN                      ((uint32_t)0x00100000)        /*!< SMBus host address enable */
#define  I2C_CR1_SMBDEN                      ((uint32_t)0x00200000)        /*!< SMBus device default address enable */
#define  I2C_CR1_ALERTEN                     ((uint32_t)0x00400000)        /*!< SMBus alert enable */
#define  I2C_CR1_PECEN                       ((uint32_t)0x00800000)        /*!< PEC enable */

/******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_SADD                        ((uint32_t)0x000003FF)        /*!< Slave address (master mode) */
#define  I2C_CR2_RD_WRN                      ((uint32_t)0x00000400)        /*!< Transfer direction (master mode) */
#define  I2C_CR2_ADD10                       ((uint32_t)0x00000800)        /*!< 10-bit addressing mode (master mode) */
#define  I2C_CR2_HEAD10R                     ((uint32_t)0x00001000)        /*!< 10-bit address header only read direction (master mode) */
#define  I2C_CR2_START                       ((uint32_t)0x00002000)        /*!< START generation */
#define  I2C_CR2_STOP                        ((uint32_t)0x00004000)        /*!< STOP generation (master mode) */
#define  I2C_CR2_NACK                        ((uint32_t)0x00008000)        /*!< NACK generation (slave mode) */
#define  I2C_CR2_NBYTES                      ((uint32_t)0x00FF0000)        /*!< Number of bytes */
#define  I2C_CR2_RELOAD                      ((uint32_t)0x01000000)        /*!< NBYTES reload mode */
#define  I2C_CR2_AUTOEND                     ((uint32_t)0x02000000)        /*!< Automatic end mode (master mode) */
#define  I2C_CR2_PECBYTE                     ((uint32_t)0x04000000)        /*!< Packet error checking byte */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define  I2C_OAR1_OA1                        ((uint32_t)0x000003FF)        /*!< Interface own address 1 */
#define  I2C_OAR1_OA1MODE                    ((uint32_t)0x00000400)        /*!< Own address 1 10-bit mode */
#define  I2C_OAR1_OA1EN                      ((uint32_t)0x00008000)        /*!< Own address 1 enable */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_OA2                        ((uint32_t)0x000000FE)        /*!< Interface own address 2                        */
#define  I2C_OAR2_OA2MSK                     ((uint32_t)0x00000700)        /*!< Own address 2 masks                            */
#define  I2C_OAR2_OA2NOMASK                  ((uint32_t)0x00000000)        /*!< No mask                                        */
#define  I2C_OAR2_OA2MASK01                  ((uint32_t)0x00000100)        /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define  I2C_OAR2_OA2MASK02                  ((uint32_t)0x00000200)        /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define  I2C_OAR2_OA2MASK03                  ((uint32_t)0x00000300)        /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define  I2C_OAR2_OA2MASK04                  ((uint32_t)0x00000400)        /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define  I2C_OAR2_OA2MASK05                  ((uint32_t)0x00000500)        /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define  I2C_OAR2_OA2MASK06                  ((uint32_t)0x00000600)        /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define  I2C_OAR2_OA2MASK07                  ((uint32_t)0x00000700)        /*!< OA2[7:1] is masked, No comparison is done      */
#define  I2C_OAR2_OA2EN                      ((uint32_t)0x00008000)        /*!< Own address 2 enable                           */

/*******************  Bit definition for I2C_TIMINGR register *****************/
#define  I2C_TIMINGR_SCLL                    ((uint32_t)0x000000FF)        /*!< SCL low period (master mode) */
#define  I2C_TIMINGR_SCLH                    ((uint32_t)0x0000FF00)        /*!< SCL high period (master mode) */
#define  I2C_TIMINGR_SDADEL                  ((uint32_t)0x000F0000)        /*!< Data hold time */
#define  I2C_TIMINGR_SCLDEL                  ((uint32_t)0x00F00000)        /*!< Data setup time */
#define  I2C_TIMINGR_PRESC                   ((uint32_t)0xF0000000)        /*!< Timings prescaler */

/******************* Bit definition for I2C_TIMEOUTR register *****************/
#define  I2C_TIMEOUTR_TIMEOUTA               ((uint32_t)0x00000FFF)        /*!< Bus timeout A */
#define  I2C_TIMEOUTR_TIDLE                  ((uint32_t)0x00001000)        /*!< Idle clock timeout detection */
#define  I2C_TIMEOUTR_TIMOUTEN               ((uint32_t)0x00008000)        /*!< Clock timeout enable */
#define  I2C_TIMEOUTR_TIMEOUTB               ((uint32_t)0x0FFF0000)        /*!< Bus timeout B*/
#define  I2C_TIMEOUTR_TEXTEN                 ((uint32_t)0x80000000)        /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  *********************/
#define  I2C_ISR_TXE                         ((uint32_t)0x00000001)        /*!< Transmit data register empty */
#define  I2C_ISR_TXIS                        ((uint32_t)0x00000002)        /*!< Transmit interrupt status */
#define  I2C_ISR_RXNE                        ((uint32_t)0x00000004)        /*!< Receive data register not empty */
#define  I2C_ISR_ADDR                        ((uint32_t)0x00000008)        /*!< Address matched (slave mode)*/
#define  I2C_ISR_NACKF                       ((uint32_t)0x00000010)        /*!< NACK received flag */
#define  I2C_ISR_STOPF                       ((uint32_t)0x00000020)        /*!< STOP detection flag */
#define  I2C_ISR_TC                          ((uint32_t)0x00000040)        /*!< Transfer complete (master mode) */
#define  I2C_ISR_TCR                         ((uint32_t)0x00000080)        /*!< Transfer complete reload */
#define  I2C_ISR_BERR                        ((uint32_t)0x00000100)        /*!< Bus error */
#define  I2C_ISR_ARLO                        ((uint32_t)0x00000200)        /*!< Arbitration lost */
#define  I2C_ISR_OVR                         ((uint32_t)0x00000400)        /*!< Overrun/Underrun */
#define  I2C_ISR_PECERR                      ((uint32_t)0x00000800)        /*!< PEC error in reception */
#define  I2C_ISR_TIMEOUT                     ((uint32_t)0x00001000)        /*!< Timeout or Tlow detection flag */
#define  I2C_ISR_ALERT                       ((uint32_t)0x00002000)        /*!< SMBus alert */
#define  I2C_ISR_BUSY                        ((uint32_t)0x00008000)        /*!< Bus busy */
#define  I2C_ISR_DIR                         ((uint32_t)0x00010000)        /*!< Transfer direction (slave mode) */
#define  I2C_ISR_ADDCODE                     ((uint32_t)0x00FE0000)        /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  *********************/
#define  I2C_ICR_ADDRCF                      ((uint32_t)0x00000008)        /*!< Address matched clear flag */
#define  I2C_ICR_NACKCF                      ((uint32_t)0x00000010)        /*!< NACK clear flag */
#define  I2C_ICR_STOPCF                      ((uint32_t)0x00000020)        /*!< STOP detection clear flag */
#define  I2C_ICR_BERRCF                      ((uint32_t)0x00000100)        /*!< Bus error clear flag */
#define  I2C_ICR_ARLOCF                      ((uint32_t)0x00000200)        /*!< Arbitration lost clear flag */
#define  I2C_ICR_OVRCF                       ((uint32_t)0x00000400)        /*!< Overrun/Underrun clear flag */
#define  I2C_ICR_PECCF                       ((uint32_t)0x00000800)        /*!< PAC error clear flag */
#define  I2C_ICR_TIMOUTCF                    ((uint32_t)0x00001000)        /*!< Timeout clear flag */
#define  I2C_ICR_ALERTCF                     ((uint32_t)0x00002000)        /*!< Alert clear flag */

/******************  Bit definition for I2C_PECR register  ********************/
#define  I2C_PECR_PEC                        ((uint32_t)0x000000FF)        /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define  I2C_RXDR_RXDATA                     ((uint32_t)0x000000FF)        /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
#define  I2C_TXDR_TXDATA                     ((uint32_t)0x000000FF)        /*!< 8-bit transmit data */


/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG (IWDG)                      */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((uint32_t)0x0000FFFF)        /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((uint32_t)0x00000007)        /*!< PR[2:0] (Prescaler divider) */
#define  IWDG_PR_PR_0                        ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  IWDG_PR_PR_1                        ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  IWDG_PR_PR_2                        ((uint32_t)0x00000004)        /*!< Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((uint32_t)0x00000FFF)        /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((uint32_t)0x00000001)        /*!< Watchdog prescaler value update */
#define  IWDG_SR_RVU                         ((uint32_t)0x00000002)        /*!< Watchdog counter reload value update */
#define  IWDG_SR_WVU                         ((uint32_t)0x00000004)        /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_WINR_WIN                       ((uint32_t)0x00000FFF)        /*!< Watchdog counter window value */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((uint32_t)0x00000001)     /*!< Low-power Deepsleep */
#define  PWR_CR_PDDS                         ((uint32_t)0x00000002)     /*!< Power Down Deepsleep */
#define  PWR_CR_CWUF                         ((uint32_t)0x00000004)     /*!< Clear Wakeup Flag */
#define  PWR_CR_CSBF                         ((uint32_t)0x00000008)     /*!< Clear Standby Flag */
#define  PWR_CR_PVDE                         ((uint32_t)0x00000010)     /*!< Power Voltage Detector Enable */

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

#define  PWR_CR_DBP                          ((uint32_t)0x00000100)     /*!< Disable Backup Domain write protection */

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((uint32_t)0x00000001)     /*!< Wakeup Flag */
#define  PWR_CSR_SBF                         ((uint32_t)0x00000002)     /*!< Standby Flag */
#define  PWR_CSR_PVDO                        ((uint32_t)0x00000004)     /*!< PVD Output */
#define  PWR_CSR_VREFINTRDYF                 ((uint32_t)0x00000008)     /*!< Internal voltage reference (VREFINT) ready flag */

#define  PWR_CSR_EWUP1                       ((uint32_t)0x00000100)     /*!< Enable WKUP pin 1 */
#define  PWR_CSR_EWUP2                       ((uint32_t)0x00000200)     /*!< Enable WKUP pin 2 */
#define  PWR_CSR_EWUP3                       ((uint32_t)0x00000400)     /*!< Enable WKUP pin 3 */

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
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00000400)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00000500)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00000600)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00000700)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00000800)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00001000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00002000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00002000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x00002800)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x00003000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x00003800)        /*!< HCLK divided by 16 */

#define  RCC_CFGR_PLLSRC                     ((uint32_t)0x00010000)        /*!< PLL entry clock source */
#define  RCC_CFGR_PLLSRC_HSI_DIV2            ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define  RCC_CFGR_PLLSRC_HSE_PREDIV          ((uint32_t)0x00010000)        /*!< HSE/PREDIV clock selected as PLL entry clock source */

#define  RCC_CFGR_PLLXTPRE                   ((uint32_t)0x00020000)        /*!< HSE divider for PLL entry */
#define  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV1   ((uint32_t)0x00000000)        /*!< HSE/PREDIV clock not divided for PLL entry */
#define  RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2   ((uint32_t)0x00020000)        /*!< HSE/PREDIV clock divided by 2 for PLL entry */

/*!< PLLMUL configuration */
#define  RCC_CFGR_PLLMUL                     ((uint32_t)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMUL_0                   ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR_PLLMUL_1                   ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  RCC_CFGR_PLLMUL_2                   ((uint32_t)0x00100000)        /*!< Bit 2 */
#define  RCC_CFGR_PLLMUL_3                   ((uint32_t)0x00200000)        /*!< Bit 3 */

#define  RCC_CFGR_PLLMUL2                    ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
#define  RCC_CFGR_PLLMUL3                    ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
#define  RCC_CFGR_PLLMUL4                    ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
#define  RCC_CFGR_PLLMUL5                    ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
#define  RCC_CFGR_PLLMUL6                    ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
#define  RCC_CFGR_PLLMUL7                    ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
#define  RCC_CFGR_PLLMUL8                    ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
#define  RCC_CFGR_PLLMUL9                    ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
#define  RCC_CFGR_PLLMUL10                   ((uint32_t)0x00200000)        /*!< PLL input clock10 */
#define  RCC_CFGR_PLLMUL11                   ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
#define  RCC_CFGR_PLLMUL12                   ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
#define  RCC_CFGR_PLLMUL13                   ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
#define  RCC_CFGR_PLLMUL14                   ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
#define  RCC_CFGR_PLLMUL15                   ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
#define  RCC_CFGR_PLLMUL16                   ((uint32_t)0x00380000)        /*!< PLL input clock*16 */

/*!< USB configuration */
#define  RCC_CFGR_USBPRE                     ((uint32_t)0x00400000)        /*!< USB prescaler */

#define  RCC_CFGR_USBPRE_DIV1_5              ((uint32_t)0x00000000)        /*!< USB prescaler is PLL clock divided by 1.5 */
#define  RCC_CFGR_USBPRE_DIV1                ((uint32_t)0x00400000)        /*!< USB prescaler is PLL clock divided by 1 */

/*!< I2S configuration */
#define  RCC_CFGR_I2SSRC                     ((uint32_t)0x00800000)        /*!< I2S external clock source selection */

#define  RCC_CFGR_I2SSRC_SYSCLK              ((uint32_t)0x00000000)        /*!< System clock selected as I2S clock source */
#define  RCC_CFGR_I2SSRC_EXT                 ((uint32_t)0x00800000)        /*!< External clock selected as I2S clock source */

/*!< MCO configuration */
#define  RCC_CFGR_MCO                        ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define  RCC_CFGR_MCO_0                      ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  RCC_CFGR_MCO_1                      ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  RCC_CFGR_MCO_2                      ((uint32_t)0x04000000)        /*!< Bit 2 */

#define  RCC_CFGR_MCO_NOCLOCK                ((uint32_t)0x00000000)        /*!< No clock */
#define  RCC_CFGR_MCO_LSI                    ((uint32_t)0x02000000)        /*!< LSI clock selected as MCO source */
#define  RCC_CFGR_MCO_LSE                    ((uint32_t)0x03000000)        /*!< LSE clock selected as MCO source */
#define  RCC_CFGR_MCO_SYSCLK                 ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
#define  RCC_CFGR_MCO_HSI                    ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
#define  RCC_CFGR_MCO_HSE                    ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
#define  RCC_CFGR_MCO_PLL                    ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */

#define  RCC_CFGR_MCOF                       ((uint32_t)0x10000000)        /*!< Microcontroller Clock Output Flag */
#define  RCC_CFGR_PLLNODIV                   ((uint32_t)0x80000000)        /*!< Do not divide PLL to MCO */

/*********************  Bit definition for RCC_CIR register  ********************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001)        /*!< LSI Ready Interrupt flag */
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002)        /*!< LSE Ready Interrupt flag */
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004)        /*!< HSI Ready Interrupt flag */
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008)        /*!< HSE Ready Interrupt flag */
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010)        /*!< PLL Ready Interrupt flag */
#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080)        /*!< Clock Security System Interrupt flag */
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100)        /*!< LSI Ready Interrupt Enable */
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200)        /*!< LSE Ready Interrupt Enable */
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400)        /*!< HSI Ready Interrupt Enable */
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800)        /*!< HSE Ready Interrupt Enable */
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000)        /*!< PLL Ready Interrupt Enable */
#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000)        /*!< LSI Ready Interrupt Clear */
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000)        /*!< LSE Ready Interrupt Clear */
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000)        /*!< HSI Ready Interrupt Clear */
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000)        /*!< HSE Ready Interrupt Clear */
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000)        /*!< PLL Ready Interrupt Clear */
#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000)        /*!< Clock Security System Interrupt Clear */

/******************  Bit definition for RCC_APB2RSTR register  *****************/
#define  RCC_APB2RSTR_SYSCFGRST              ((uint32_t)0x00000001)        /*!< SYSCFG reset */
#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000800)        /*!< TIM1 reset */
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000)        /*!< SPI1 reset */
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00004000)        /*!< USART1 reset */
#define  RCC_APB2RSTR_TIM15RST               ((uint32_t)0x00010000)        /*!< TIM15 reset */
#define  RCC_APB2RSTR_TIM16RST               ((uint32_t)0x00020000)        /*!< TIM16 reset */
#define  RCC_APB2RSTR_TIM17RST               ((uint32_t)0x00040000)        /*!< TIM17 reset */

/******************  Bit definition for RCC_APB1RSTR register  ******************/
#define  RCC_APB1RSTR_TIM2RST                ((uint32_t)0x00000001)        /*!< Timer 2 reset */
#define  RCC_APB1RSTR_TIM3RST                ((uint32_t)0x00000002)        /*!< Timer 3 reset */
#define  RCC_APB1RSTR_TIM4RST                ((uint32_t)0x00000004)        /*!< Timer 4 reset */
#define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)        /*!< Timer 6 reset */
#define  RCC_APB1RSTR_WWDGRST                ((uint32_t)0x00000800)        /*!< Window Watchdog reset */
#define  RCC_APB1RSTR_SPI2RST                ((uint32_t)0x00004000)        /*!< SPI2 reset */
#define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00008000)        /*!< SPI3 reset */
#define  RCC_APB1RSTR_USART2RST              ((uint32_t)0x00020000)        /*!< USART 2 reset */
#define  RCC_APB1RSTR_USART3RST              ((uint32_t)0x00040000)        /*!< USART 3 reset */
#define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)        /*!< UART 4 reset */
#define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)        /*!< UART 5 reset */
#define  RCC_APB1RSTR_I2C1RST                ((uint32_t)0x00200000)        /*!< I2C 1 reset */
#define  RCC_APB1RSTR_I2C2RST                ((uint32_t)0x00400000)        /*!< I2C 2 reset */
#define  RCC_APB1RSTR_USBRST                 ((uint32_t)0x00800000)        /*!< USB reset */
#define  RCC_APB1RSTR_CANRST                 ((uint32_t)0x02000000)        /*!< CAN reset */
#define  RCC_APB1RSTR_PWRRST                 ((uint32_t)0x10000000)        /*!< PWR reset */
#define  RCC_APB1RSTR_DAC1RST                ((uint32_t)0x20000000)        /*!< DAC 1 reset */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define  RCC_AHBENR_DMA1EN                   ((uint32_t)0x00000001)        /*!< DMA1 clock enable */
#define  RCC_AHBENR_DMA2EN                   ((uint32_t)0x00000002)        /*!< DMA2 clock enable */
#define  RCC_AHBENR_SRAMEN                   ((uint32_t)0x00000004)        /*!< SRAM interface clock enable */
#define  RCC_AHBENR_FLITFEN                  ((uint32_t)0x00000010)        /*!< FLITF clock enable */
#define  RCC_AHBENR_CRCEN                    ((uint32_t)0x00000040)        /*!< CRC clock enable */
#define  RCC_AHBENR_GPIOAEN                  ((uint32_t)0x00020000)        /*!< GPIOA clock enable */
#define  RCC_AHBENR_GPIOBEN                  ((uint32_t)0x00040000)        /*!< GPIOB clock enable */
#define  RCC_AHBENR_GPIOCEN                  ((uint32_t)0x00080000)        /*!< GPIOC clock enable */
#define  RCC_AHBENR_GPIODEN                  ((uint32_t)0x00100000)        /*!< GPIOD clock enable */
#define  RCC_AHBENR_GPIOEEN                  ((uint32_t)0x00200000)        /*!< GPIOE clock enable */
#define  RCC_AHBENR_GPIOFEN                  ((uint32_t)0x00400000)        /*!< GPIOF clock enable */
#define  RCC_AHBENR_TSCEN                     ((uint32_t)0x01000000)       /*!< TS clock enable */
#define  RCC_AHBENR_ADC12EN                  ((uint32_t)0x10000000)        /*!< ADC1/ ADC2 clock enable */

/*****************  Bit definition for RCC_APB2ENR register  ******************/
#define  RCC_APB2ENR_SYSCFGEN                ((uint32_t)0x00000001)        /*!< SYSCFG clock enable */
#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000800)        /*!< TIM1 clock enable */
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000)        /*!< SPI1 clock enable */
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00004000)        /*!< USART1 clock enable */
#define  RCC_APB2ENR_TIM15EN                 ((uint32_t)0x00010000)        /*!< TIM15 clock enable */
#define  RCC_APB2ENR_TIM16EN                 ((uint32_t)0x00020000)        /*!< TIM16 clock enable */
#define  RCC_APB2ENR_TIM17EN                 ((uint32_t)0x00040000)        /*!< TIM17 clock enable */

/******************  Bit definition for RCC_APB1ENR register  ******************/
#define  RCC_APB1ENR_TIM2EN                  ((uint32_t)0x00000001)        /*!< Timer 2 clock enable */
#define  RCC_APB1ENR_TIM3EN                  ((uint32_t)0x00000002)        /*!< Timer 3 clock enable */
#define  RCC_APB1ENR_TIM4EN                  ((uint32_t)0x00000004)        /*!< Timer 4 clock enable */
#define  RCC_APB1ENR_TIM6EN                  ((uint32_t)0x00000010)        /*!< Timer 6 clock enable */
#define  RCC_APB1ENR_WWDGEN                  ((uint32_t)0x00000800)        /*!< Window Watchdog clock enable */
#define  RCC_APB1ENR_SPI2EN                  ((uint32_t)0x00004000)        /*!< SPI2 clock enable */
#define  RCC_APB1ENR_SPI3EN                  ((uint32_t)0x00008000)        /*!< SPI3 clock enable */
#define  RCC_APB1ENR_USART2EN                ((uint32_t)0x00020000)        /*!< USART 2 clock enable */
#define  RCC_APB1ENR_USART3EN                ((uint32_t)0x00040000)        /*!< USART 3 clock enable */
#define  RCC_APB1ENR_UART4EN                 ((uint32_t)0x00080000)        /*!< UART 4 clock enable */
#define  RCC_APB1ENR_UART5EN                 ((uint32_t)0x00100000)        /*!< UART 5 clock enable */
#define  RCC_APB1ENR_I2C1EN                  ((uint32_t)0x00200000)        /*!< I2C 1 clock enable */
#define  RCC_APB1ENR_I2C2EN                  ((uint32_t)0x00400000)        /*!< I2C 2 clock enable */
#define  RCC_APB1ENR_USBEN                   ((uint32_t)0x00800000)        /*!< USB clock enable */
#define  RCC_APB1ENR_CANEN                   ((uint32_t)0x02000000)        /*!< CAN clock enable */
#define  RCC_APB1ENR_PWREN                   ((uint32_t)0x10000000)        /*!< PWR clock enable */
#define  RCC_APB1ENR_DAC1EN                  ((uint32_t)0x20000000)        /*!< DAC 1 clock enable */

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSE                        ((uint32_t)0x00000007)        /*!< External Low Speed oscillator [2:0] bits */
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001)        /*!< External Low Speed oscillator enable */
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002)        /*!< External Low Speed oscillator Ready */
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004)        /*!< External Low Speed oscillator Bypass */

#define  RCC_BDCR_LSEDRV                     ((uint32_t)0x00000018)        /*!< LSEDRV[1:0] bits (LSE Osc. drive capability) */
#define  RCC_BDCR_LSEDRV_0                   ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  RCC_BDCR_LSEDRV_1                   ((uint32_t)0x00000010)        /*!< Bit 1 */

#define  RCC_BDCR_RTCSEL                     ((uint32_t)0x00000300)        /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_BDCR_RTCSEL_0                   ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_BDCR_RTCSEL_1                   ((uint32_t)0x00000200)        /*!< Bit 1 */

/*!< RTC configuration */
#define  RCC_BDCR_RTCSEL_NOCLOCK             ((uint32_t)0x00000000)        /*!< No clock */
#define  RCC_BDCR_RTCSEL_LSE                 ((uint32_t)0x00000100)        /*!< LSE oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_LSI                 ((uint32_t)0x00000200)        /*!< LSI oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_HSE                 ((uint32_t)0x00000300)        /*!< HSE oscillator clock divided by 32 used as RTC clock */

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000)        /*!< RTC clock enable */
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000)        /*!< Backup domain software reset  */

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)        /*!< Internal Low Speed oscillator enable */
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)        /*!< Internal Low Speed oscillator Ready */
#define  RCC_CSR_V18PWRRSTF                  ((uint32_t)0x00800000)        /*!< V1.8 power domain reset flag */
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)        /*!< Remove reset flag */
#define  RCC_CSR_OBLRSTF                     ((uint32_t)0x02000000)        /*!< OBL reset flag */
#define  RCC_CSR_PINRSTF                     ((uint32_t)0x04000000)        /*!< PIN reset flag */
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000)        /*!< POR/PDR reset flag */
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)        /*!< Software Reset flag */
#define  RCC_CSR_IWDGRSTF                    ((uint32_t)0x20000000)        /*!< Independent Watchdog reset flag */
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)        /*!< Window watchdog reset flag */
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)        /*!< Low-Power reset flag */

/*******************  Bit definition for RCC_AHBRSTR register  ****************/
#define  RCC_AHBRSTR_GPIOARST                ((uint32_t)0x00020000)         /*!< GPIOA reset */
#define  RCC_AHBRSTR_GPIOBRST                ((uint32_t)0x00040000)         /*!< GPIOB reset */
#define  RCC_AHBRSTR_GPIOCRST                ((uint32_t)0x00080000)         /*!< GPIOC reset */
#define  RCC_AHBRSTR_GPIODRST                ((uint32_t)0x00100000)         /*!< GPIOD reset */
#define  RCC_AHBRSTR_GPIOERST                ((uint32_t)0x00200000)         /*!< GPIOE reset */
#define  RCC_AHBRSTR_GPIOFRST                ((uint32_t)0x00400000)         /*!< GPIOF reset */
#define  RCC_AHBRSTR_TSCRST                  ((uint32_t)0x01000000)         /*!< TSC reset */
#define  RCC_AHBRSTR_ADC12RST                ((uint32_t)0x10000000)         /*!< ADC1 & ADC2 reset */

/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV configuration */
#define  RCC_CFGR2_PREDIV                    ((uint32_t)0x0000000F)        /*!< PREDIV[3:0] bits */
#define  RCC_CFGR2_PREDIV_0                  ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR2_PREDIV_1                  ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  RCC_CFGR2_PREDIV_2                  ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  RCC_CFGR2_PREDIV_3                  ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  RCC_CFGR2_PREDIV_DIV1               ((uint32_t)0x00000000)        /*!< PREDIV input clock not divided */
#define  RCC_CFGR2_PREDIV_DIV2               ((uint32_t)0x00000001)        /*!< PREDIV input clock divided by 2 */
#define  RCC_CFGR2_PREDIV_DIV3               ((uint32_t)0x00000002)        /*!< PREDIV input clock divided by 3 */
#define  RCC_CFGR2_PREDIV_DIV4               ((uint32_t)0x00000003)        /*!< PREDIV input clock divided by 4 */
#define  RCC_CFGR2_PREDIV_DIV5               ((uint32_t)0x00000004)        /*!< PREDIV input clock divided by 5 */
#define  RCC_CFGR2_PREDIV_DIV6               ((uint32_t)0x00000005)        /*!< PREDIV input clock divided by 6 */
#define  RCC_CFGR2_PREDIV_DIV7               ((uint32_t)0x00000006)        /*!< PREDIV input clock divided by 7 */
#define  RCC_CFGR2_PREDIV_DIV8               ((uint32_t)0x00000007)        /*!< PREDIV input clock divided by 8 */
#define  RCC_CFGR2_PREDIV_DIV9               ((uint32_t)0x00000008)        /*!< PREDIV input clock divided by 9 */
#define  RCC_CFGR2_PREDIV_DIV10              ((uint32_t)0x00000009)        /*!< PREDIV input clock divided by 10 */
#define  RCC_CFGR2_PREDIV_DIV11              ((uint32_t)0x0000000A)        /*!< PREDIV input clock divided by 11 */
#define  RCC_CFGR2_PREDIV_DIV12              ((uint32_t)0x0000000B)        /*!< PREDIV input clock divided by 12 */
#define  RCC_CFGR2_PREDIV_DIV13              ((uint32_t)0x0000000C)        /*!< PREDIV input clock divided by 13 */
#define  RCC_CFGR2_PREDIV_DIV14              ((uint32_t)0x0000000D)        /*!< PREDIV input clock divided by 14 */
#define  RCC_CFGR2_PREDIV_DIV15              ((uint32_t)0x0000000E)        /*!< PREDIV input clock divided by 15 */
#define  RCC_CFGR2_PREDIV_DIV16              ((uint32_t)0x0000000F)        /*!< PREDIV input clock divided by 16 */

/*!< ADCPRE12 configuration */
#define  RCC_CFGR2_ADCPRE12                  ((uint32_t)0x000001F0)        /*!< ADCPRE12[8:4] bits */
#define  RCC_CFGR2_ADCPRE12_0                ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR2_ADCPRE12_1                ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR2_ADCPRE12_2                ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR2_ADCPRE12_3                ((uint32_t)0x00000080)        /*!< Bit 3 */
#define  RCC_CFGR2_ADCPRE12_4                ((uint32_t)0x00000100)        /*!< Bit 4 */

#define  RCC_CFGR2_ADCPRE12_NO               ((uint32_t)0x00000000)        /*!< ADC12 clock disabled, ADC12 can use AHB clock */
#define  RCC_CFGR2_ADCPRE12_DIV1             ((uint32_t)0x00000100)        /*!< ADC12 PLL clock divided by 1 */
#define  RCC_CFGR2_ADCPRE12_DIV2             ((uint32_t)0x00000110)        /*!< ADC12 PLL clock divided by 2 */
#define  RCC_CFGR2_ADCPRE12_DIV4             ((uint32_t)0x00000120)        /*!< ADC12 PLL clock divided by 4 */
#define  RCC_CFGR2_ADCPRE12_DIV6             ((uint32_t)0x00000130)        /*!< ADC12 PLL clock divided by 6 */
#define  RCC_CFGR2_ADCPRE12_DIV8             ((uint32_t)0x00000140)        /*!< ADC12 PLL clock divided by 8 */
#define  RCC_CFGR2_ADCPRE12_DIV10            ((uint32_t)0x00000150)        /*!< ADC12 PLL clock divided by 10 */
#define  RCC_CFGR2_ADCPRE12_DIV12            ((uint32_t)0x00000160)        /*!< ADC12 PLL clock divided by 12 */
#define  RCC_CFGR2_ADCPRE12_DIV16            ((uint32_t)0x00000170)        /*!< ADC12 PLL clock divided by 16 */
#define  RCC_CFGR2_ADCPRE12_DIV32            ((uint32_t)0x00000180)        /*!< ADC12 PLL clock divided by 32 */
#define  RCC_CFGR2_ADCPRE12_DIV64            ((uint32_t)0x00000190)        /*!< ADC12 PLL clock divided by 64 */
#define  RCC_CFGR2_ADCPRE12_DIV128           ((uint32_t)0x000001A0)        /*!< ADC12 PLL clock divided by 128 */
#define  RCC_CFGR2_ADCPRE12_DIV256           ((uint32_t)0x000001B0)        /*!< ADC12 PLL clock divided by 256 */

/*******************  Bit definition for RCC_CFGR3 register  ******************/
#define  RCC_CFGR3_USART1SW                  ((uint32_t)0x00000003)        /*!< USART1SW[1:0] bits */
#define  RCC_CFGR3_USART1SW_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR3_USART1SW_1                ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR3_USART1SW_PCLK2            ((uint32_t)0x00000000)        /*!< PCLK2 clock used as USART1 clock source */
#define  RCC_CFGR3_USART1SW_SYSCLK           ((uint32_t)0x00000001)        /*!< System clock selected as USART1 clock source */
#define  RCC_CFGR3_USART1SW_LSE              ((uint32_t)0x00000002)        /*!< LSE oscillator clock used as USART1 clock source */
#define  RCC_CFGR3_USART1SW_HSI              ((uint32_t)0x00000003)        /*!< HSI oscillator clock used as USART1 clock source */

#define  RCC_CFGR3_I2CSW                     ((uint32_t)0x00000030)        /*!< I2CSW bits */
#define  RCC_CFGR3_I2C1SW                    ((uint32_t)0x00000010)        /*!< I2C1SW bits */
#define  RCC_CFGR3_I2C2SW                    ((uint32_t)0x00000020)        /*!< I2C2SW bits */

#define  RCC_CFGR3_I2C1SW_HSI                ((uint32_t)0x00000000)        /*!< HSI oscillator clock used as I2C1 clock source */
#define  RCC_CFGR3_I2C1SW_SYSCLK             ((uint32_t)0x00000010)        /*!< System clock selected as I2C1 clock source */
#define  RCC_CFGR3_I2C2SW_HSI                ((uint32_t)0x00000000)        /*!< HSI oscillator clock used as I2C2 clock source */
#define  RCC_CFGR3_I2C2SW_SYSCLK             ((uint32_t)0x00000020)        /*!< System clock selected as I2C2 clock source */
#define  RCC_CFGR3_TIMSW                     ((uint32_t)0x00000300)        /*!< TIMSW bits */
#define  RCC_CFGR3_TIM1SW                    ((uint32_t)0x00000100)        /*!< TIM1SW bits */
#define  RCC_CFGR3_TIM1SW_HCLK               ((uint32_t)0x00000000)        /*!< HCLK used as TIM1 clock source */
#define  RCC_CFGR3_TIM1SW_PLL                ((uint32_t)0x00000100)        /*!< PLL clock used as TIM1 clock source */

#define  RCC_CFGR3_USART2SW                  ((uint32_t)0x00030000)        /*!< USART2SW[1:0] bits */
#define  RCC_CFGR3_USART2SW_0                ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  RCC_CFGR3_USART2SW_1                ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  RCC_CFGR3_USART2SW_PCLK             ((uint32_t)0x00000000)        /*!< PCLK1 clock used as USART2 clock source */
#define  RCC_CFGR3_USART2SW_SYSCLK           ((uint32_t)0x00010000)        /*!< System clock selected as USART2 clock source */
#define  RCC_CFGR3_USART2SW_LSE              ((uint32_t)0x00020000)        /*!< LSE oscillator clock used as USART2 clock source */
#define  RCC_CFGR3_USART2SW_HSI              ((uint32_t)0x00030000)        /*!< HSI oscillator clock used as USART2 clock source */

#define  RCC_CFGR3_USART3SW                  ((uint32_t)0x000C0000)        /*!< USART3SW[1:0] bits */
#define  RCC_CFGR3_USART3SW_0                ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR3_USART3SW_1                ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  RCC_CFGR3_USART3SW_PCLK             ((uint32_t)0x00000000)        /*!< PCLK1 clock used as USART3 clock source */
#define  RCC_CFGR3_USART3SW_SYSCLK           ((uint32_t)0x00040000)        /*!< System clock selected as USART3 clock source */
#define  RCC_CFGR3_USART3SW_LSE              ((uint32_t)0x00080000)        /*!< LSE oscillator clock used as USART3 clock source */
#define  RCC_CFGR3_USART3SW_HSI              ((uint32_t)0x000C0000)        /*!< HSI oscillator clock used as USART3 clock source */

#define  RCC_CFGR3_UART4SW                   ((uint32_t)0x00300000)        /*!< UART4SW[1:0] bits */
#define  RCC_CFGR3_UART4SW_0                 ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  RCC_CFGR3_UART4SW_1                 ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  RCC_CFGR3_UART4SW_PCLK              ((uint32_t)0x00000000)        /*!< PCLK1 clock used as UART4 clock source */
#define  RCC_CFGR3_UART4SW_SYSCLK            ((uint32_t)0x00100000)        /*!< System clock selected as UART4 clock source */
#define  RCC_CFGR3_UART4SW_LSE               ((uint32_t)0x00200000)        /*!< LSE oscillator clock used as UART4 clock source */
#define  RCC_CFGR3_UART4SW_HSI               ((uint32_t)0x00300000)        /*!< HSI oscillator clock used as UART4 clock source */

#define  RCC_CFGR3_UART5SW                   ((uint32_t)0x00C00000)        /*!< UART5SW[1:0] bits */
#define  RCC_CFGR3_UART5SW_0                 ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  RCC_CFGR3_UART5SW_1                 ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  RCC_CFGR3_UART5SW_PCLK              ((uint32_t)0x00000000)        /*!< PCLK1 clock used as UART5 clock source */
#define  RCC_CFGR3_UART5SW_SYSCLK            ((uint32_t)0x00400000)        /*!< System clock selected as UART5 clock source */
#define  RCC_CFGR3_UART5SW_LSE               ((uint32_t)0x00800000)        /*!< LSE oscillator clock used as UART5 clock source */
#define  RCC_CFGR3_UART5SW_HSI               ((uint32_t)0x00C00000)        /*!< HSI oscillator clock used as UART5 clock source */

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
#define RTC_ISR_TAMP3F                       ((uint32_t)0x00008000)
#define RTC_ISR_TAMP2F                       ((uint32_t)0x00004000)
#define RTC_ISR_TAMP1F                       ((uint32_t)0x00002000)
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
#define RTC_TAFCR_TAMP3TRG                   ((uint32_t)0x00000040)
#define RTC_TAFCR_TAMP3E                     ((uint32_t)0x00000020)
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

/******************** Number of backup registers ******************************/
#define RTC_BKP_NUMBER                       16

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface (SPI)                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((uint32_t)0x00000001)        /*!< Clock Phase */
#define  SPI_CR1_CPOL                        ((uint32_t)0x00000002)        /*!< Clock Polarity */
#define  SPI_CR1_MSTR                        ((uint32_t)0x00000004)        /*!< Master Selection */
#define  SPI_CR1_BR                          ((uint32_t)0x00000038)        /*!< BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  SPI_CR1_BR_1                        ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  SPI_CR1_BR_2                        ((uint32_t)0x00000020)        /*!< Bit 2 */
#define  SPI_CR1_SPE                         ((uint32_t)0x00000040)        /*!< SPI Enable */
#define  SPI_CR1_LSBFIRST                    ((uint32_t)0x00000080)        /*!< Frame Format */
#define  SPI_CR1_SSI                         ((uint32_t)0x00000100)        /*!< Internal slave select */
#define  SPI_CR1_SSM                         ((uint32_t)0x00000200)        /*!< Software slave management */
#define  SPI_CR1_RXONLY                      ((uint32_t)0x00000400)        /*!< Receive only */
#define  SPI_CR1_CRCL                        ((uint32_t)0x00000800)        /*!< CRC Length */
#define  SPI_CR1_CRCNEXT                     ((uint32_t)0x00001000)        /*!< Transmit CRC next */
#define  SPI_CR1_CRCEN                       ((uint32_t)0x00002000)        /*!< Hardware CRC calculation enable */
#define  SPI_CR1_BIDIOE                      ((uint32_t)0x00004000)        /*!< Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((uint32_t)0x00008000)        /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((uint32_t)0x00000001)        /*!< Rx Buffer DMA Enable */
#define  SPI_CR2_TXDMAEN                     ((uint32_t)0x00000002)        /*!< Tx Buffer DMA Enable */
#define  SPI_CR2_SSOE                        ((uint32_t)0x00000004)        /*!< SS Output Enable */
#define  SPI_CR2_NSSP                        ((uint32_t)0x00000008)        /*!< NSS pulse management Enable */
#define  SPI_CR2_FRF                         ((uint32_t)0x00000010)        /*!< Frame Format Enable */
#define  SPI_CR2_ERRIE                       ((uint32_t)0x00000020)        /*!< Error Interrupt Enable */
#define  SPI_CR2_RXNEIE                      ((uint32_t)0x00000040)        /*!< RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((uint32_t)0x00000080)        /*!< Tx buffer Empty Interrupt Enable */
#define  SPI_CR2_DS                          ((uint32_t)0x00000F00)        /*!< DS[3:0] Data Size */
#define  SPI_CR2_DS_0                        ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  SPI_CR2_DS_1                        ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  SPI_CR2_DS_2                        ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  SPI_CR2_DS_3                        ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  SPI_CR2_FRXTH                       ((uint32_t)0x00001000)        /*!< FIFO reception Threshold */
#define  SPI_CR2_LDMARX                      ((uint32_t)0x00002000)        /*!< Last DMA transfer for reception */
#define  SPI_CR2_LDMATX                      ((uint32_t)0x00004000)        /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((uint32_t)0x00000001)        /*!< Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((uint32_t)0x00000002)        /*!< Transmit buffer Empty */
#define  SPI_SR_CHSIDE                       ((uint32_t)0x00000004)        /*!< Channel side */
#define  SPI_SR_UDR                          ((uint32_t)0x00000008)        /*!< Underrun flag */
#define  SPI_SR_CRCERR                       ((uint32_t)0x00000010)        /*!< CRC Error flag */
#define  SPI_SR_MODF                         ((uint32_t)0x00000020)        /*!< Mode fault */
#define  SPI_SR_OVR                          ((uint32_t)0x00000040)        /*!< Overrun flag */
#define  SPI_SR_BSY                          ((uint32_t)0x00000080)        /*!< Busy flag */
#define  SPI_SR_FRE                          ((uint32_t)0x00000100)        /*!< TI frame format error */
#define  SPI_SR_FRLVL                        ((uint32_t)0x00000600)        /*!< FIFO Reception Level */
#define  SPI_SR_FRLVL_0                      ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  SPI_SR_FRLVL_1                      ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  SPI_SR_FTLVL                        ((uint32_t)0x00001800)        /*!< FIFO Transmission Level */
#define  SPI_SR_FTLVL_0                      ((uint32_t)0x00000800)        /*!< Bit 0 */
#define  SPI_SR_FTLVL_1                      ((uint32_t)0x00001000)        /*!< Bit 1 */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((uint32_t)0x0000FFFF)        /*!< Data Register */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((uint32_t)0x0000FFFF)        /*!< CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((uint32_t)0x0000FFFF)        /*!< Rx CRC Register */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((uint32_t)0x0000FFFF)        /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((uint32_t)0x00000001)        /*!<Channel length (number of bits per audio channel) */
#define  SPI_I2SCFGR_DATLEN                  ((uint32_t)0x00000006)        /*!<DATLEN[1:0] bits (Data length to be transferred) */
#define  SPI_I2SCFGR_DATLEN_0                ((uint32_t)0x00000002)        /*!<Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((uint32_t)0x00000004)        /*!<Bit 1 */
#define  SPI_I2SCFGR_CKPOL                   ((uint32_t)0x00000008)        /*!<steady state clock polarity */
#define  SPI_I2SCFGR_I2SSTD                  ((uint32_t)0x00000030)        /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((uint32_t)0x00000010)        /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((uint32_t)0x00000020)        /*!<Bit 1 */
#define  SPI_I2SCFGR_PCMSYNC                 ((uint32_t)0x00000080)        /*!<PCM frame synchronization */
#define  SPI_I2SCFGR_I2SCFG                  ((uint32_t)0x00000300)        /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((uint32_t)0x00000100)        /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((uint32_t)0x00000200)        /*!<Bit 1 */
#define  SPI_I2SCFGR_I2SE                    ((uint32_t)0x00000400)        /*!<I2S Enable */
#define  SPI_I2SCFGR_I2SMOD                  ((uint32_t)0x00000800)        /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((uint32_t)0x000000FF)        /*!<I2S Linear prescaler */
#define  SPI_I2SPR_ODD                       ((uint32_t)0x00000100)        /*!<Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((uint32_t)0x00000200)        /*!<Master Clock Output Enable */

/******************************************************************************/
/*                                                                            */
/*                        System Configuration(SYSCFG)                        */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for SYSCFG_CFGR1 register  ****************/
#define SYSCFG_CFGR1_MEM_MODE               ((uint32_t)0x00000003) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_CFGR1_MEM_MODE_0             ((uint32_t)0x00000001) /*!< Bit 0 */
#define SYSCFG_CFGR1_MEM_MODE_1             ((uint32_t)0x00000002) /*!< Bit 1 */
#define SYSCFG_CFGR1_USB_IT_RMP             ((uint32_t)0x00000020) /*!< USB interrupt remap */
#define SYSCFG_CFGR1_TIM1_ITR3_RMP          ((uint32_t)0x00000040) /*!< Timer 1 ITR3 selection */
#define SYSCFG_CFGR1_DAC1_TRIG1_RMP         ((uint32_t)0x00000080) /*!< DAC1 Trigger1 remap */
#define SYSCFG_CFGR1_DMA_RMP                ((uint32_t)0x00003900) /*!< DMA remap mask */
#define SYSCFG_CFGR1_ADC24_DMA_RMP          ((uint32_t)0x00000100) /*!< ADC2 and ADC4 DMA remap */
#define SYSCFG_CFGR1_TIM16_DMA_RMP          ((uint32_t)0x00000800) /*!< Timer 16 DMA remap */
#define SYSCFG_CFGR1_TIM17_DMA_RMP          ((uint32_t)0x00001000) /*!< Timer 17 DMA remap */
#define SYSCFG_CFGR1_TIM6DAC1Ch1_DMA_RMP    ((uint32_t)0x00002000) /*!< Timer 6 / DAC1 Ch1 DMA remap */
#define SYSCFG_CFGR1_I2C_PB6_FMP            ((uint32_t)0x00010000) /*!< I2C PB6 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB7_FMP            ((uint32_t)0x00020000) /*!< I2C PB7 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB8_FMP            ((uint32_t)0x00040000) /*!< I2C PB8 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB9_FMP            ((uint32_t)0x00080000) /*!< I2C PB9 Fast mode plus */
#define SYSCFG_CFGR1_I2C1_FMP               ((uint32_t)0x00100000) /*!< I2C1 Fast mode plus */
#define SYSCFG_CFGR1_I2C2_FMP               ((uint32_t)0x00200000) /*!< I2C2 Fast mode plus */
#define SYSCFG_CFGR1_ENCODER_MODE           ((uint32_t)0x00C00000) /*!< Encoder Mode */
#define SYSCFG_CFGR1_ENCODER_MODE_0         ((uint32_t)0x00400000) /*!< Encoder Mode 0 */
#define SYSCFG_CFGR1_ENCODER_MODE_1         ((uint32_t)0x00800000) /*!< Encoder Mode 1 */
#define SYSCFG_CFGR1_FPU_IE                 ((uint32_t)0xFC000000) /*!< Floating Point Unit Interrupt Enable */
#define SYSCFG_CFGR1_FPU_IE_0               ((uint32_t)0x04000000) /*!< Floating Point Unit Interrupt Enable 0 */
#define SYSCFG_CFGR1_FPU_IE_1               ((uint32_t)0x08000000) /*!< Floating Point Unit Interrupt Enable 1 */
#define SYSCFG_CFGR1_FPU_IE_2               ((uint32_t)0x10000000) /*!< Floating Point Unit Interrupt Enable 2 */
#define SYSCFG_CFGR1_FPU_IE_3               ((uint32_t)0x20000000) /*!< Floating Point Unit Interrupt Enable 3 */
#define SYSCFG_CFGR1_FPU_IE_4               ((uint32_t)0x40000000) /*!< Floating Point Unit Interrupt Enable 4 */
#define SYSCFG_CFGR1_FPU_IE_5               ((uint32_t)0x80000000) /*!< Floating Point Unit Interrupt Enable 5 */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            ((uint32_t)0x0000000F) /*!< EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            ((uint32_t)0x000000F0) /*!< EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            ((uint32_t)0x00000F00) /*!< EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            ((uint32_t)0x0000F000) /*!< EXTI 3 configuration */

/*!<*
  * @brief  EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA         ((uint32_t)0x00000000) /*!< PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         ((uint32_t)0x00000001) /*!< PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         ((uint32_t)0x00000002) /*!< PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD         ((uint32_t)0x00000003) /*!< PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE         ((uint32_t)0x00000004) /*!< PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF         ((uint32_t)0x00000005) /*!< PF[0] pin */

/*!<*
  * @brief  EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA         ((uint32_t)0x00000000) /*!< PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         ((uint32_t)0x00000010) /*!< PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         ((uint32_t)0x00000020) /*!< PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD         ((uint32_t)0x00000030) /*!< PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE         ((uint32_t)0x00000040) /*!< PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF         ((uint32_t)0x00000050) /*!< PF[1] pin */

/*!<*
  * @brief  EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA         ((uint32_t)0x00000000) /*!< PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         ((uint32_t)0x00000100) /*!< PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         ((uint32_t)0x00000200) /*!< PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD         ((uint32_t)0x00000300) /*!< PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE         ((uint32_t)0x00000400) /*!< PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF         ((uint32_t)0x00000500) /*!< PF[2] pin */

/*!<*
  * @brief  EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA         ((uint32_t)0x00000000) /*!< PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         ((uint32_t)0x00001000) /*!< PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         ((uint32_t)0x00002000) /*!< PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD         ((uint32_t)0x00003000) /*!< PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE         ((uint32_t)0x00004000) /*!< PE[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            ((uint32_t)0x0000000F) /*!< EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            ((uint32_t)0x000000F0) /*!< EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            ((uint32_t)0x00000F00) /*!< EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            ((uint32_t)0x0000F000) /*!< EXTI 7 configuration */

/*!<*
  * @brief  EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA         ((uint32_t)0x00000000) /*!< PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         ((uint32_t)0x00000001) /*!< PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         ((uint32_t)0x00000002) /*!< PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD         ((uint32_t)0x00000003) /*!< PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE         ((uint32_t)0x00000004) /*!< PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF         ((uint32_t)0x00000005) /*!< PF[4] pin */

/*!<*
  * @brief  EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA         ((uint32_t)0x00000000) /*!< PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         ((uint32_t)0x00000010) /*!< PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         ((uint32_t)0x00000020) /*!< PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD         ((uint32_t)0x00000030) /*!< PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE         ((uint32_t)0x00000040) /*!< PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF         ((uint32_t)0x00000050) /*!< PF[5] pin */

/*!<*
  * @brief  EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA         ((uint32_t)0x00000000) /*!< PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         ((uint32_t)0x00000100) /*!< PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         ((uint32_t)0x00000200) /*!< PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD         ((uint32_t)0x00000300) /*!< PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE         ((uint32_t)0x00000400) /*!< PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF         ((uint32_t)0x00000500) /*!< PF[6] pin */

/*!<*
  * @brief  EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA         ((uint32_t)0x00000000) /*!< PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         ((uint32_t)0x00001000) /*!< PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         ((uint32_t)0x00002000) /*!< PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD         ((uint32_t)0x00003000) /*!< PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE         ((uint32_t)0x00004000) /*!< PE[7] pin */

/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            ((uint32_t)0x0000000F) /*!< EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            ((uint32_t)0x000000F0) /*!< EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           ((uint32_t)0x00000F00) /*!< EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           ((uint32_t)0x0000F000) /*!< EXTI 11 configuration */

/*!<*
  * @brief  EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA         ((uint32_t)0x00000000) /*!< PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         ((uint32_t)0x00000001) /*!< PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         ((uint32_t)0x00000002) /*!< PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD         ((uint32_t)0x00000003) /*!< PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE         ((uint32_t)0x00000004) /*!< PE[8] pin */

/*!<*
  * @brief  EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA         ((uint32_t)0x00000000) /*!< PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         ((uint32_t)0x00000010) /*!< PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         ((uint32_t)0x00000020) /*!< PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD         ((uint32_t)0x00000030) /*!< PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE         ((uint32_t)0x00000040) /*!< PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF         ((uint32_t)0x00000050) /*!< PF[9] pin */

/*!<*
  * @brief  EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA        ((uint32_t)0x00000000) /*!< PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        ((uint32_t)0x00000100) /*!< PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        ((uint32_t)0x00000200) /*!< PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD        ((uint32_t)0x00000300) /*!< PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE        ((uint32_t)0x00000400) /*!< PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF        ((uint32_t)0x00000500) /*!< PF[10] pin */

/*!<*
  * @brief  EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA        ((uint32_t)0x00000000) /*!< PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        ((uint32_t)0x00001000) /*!< PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        ((uint32_t)0x00002000) /*!< PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD        ((uint32_t)0x00003000) /*!< PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE        ((uint32_t)0x00004000) /*!< PE[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  *****************/
#define SYSCFG_EXTICR4_EXTI12           ((uint32_t)0x0000000F) /*!< EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           ((uint32_t)0x000000F0) /*!< EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           ((uint32_t)0x00000F00) /*!< EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           ((uint32_t)0x0000F000) /*!< EXTI 15 configuration */

/*!<*
  * @brief  EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA        ((uint32_t)0x00000000) /*!< PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        ((uint32_t)0x00000001) /*!< PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        ((uint32_t)0x00000002) /*!< PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD        ((uint32_t)0x00000003) /*!< PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE        ((uint32_t)0x00000004) /*!< PE[12] pin */

/*!<*
  * @brief  EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA        ((uint32_t)0x00000000) /*!< PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        ((uint32_t)0x00000010) /*!< PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        ((uint32_t)0x00000020) /*!< PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD        ((uint32_t)0x00000030) /*!< PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE        ((uint32_t)0x00000040) /*!< PE[13] pin */

/*!<*
  * @brief  EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA        ((uint32_t)0x00000000) /*!< PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        ((uint32_t)0x00000100) /*!< PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        ((uint32_t)0x00000200) /*!< PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD        ((uint32_t)0x00000300) /*!< PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE        ((uint32_t)0x00000400) /*!< PE[14] pin */

/*!<*
  * @brief  EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA        ((uint32_t)0x00000000) /*!< PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        ((uint32_t)0x00001000) /*!< PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        ((uint32_t)0x00002000) /*!< PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD        ((uint32_t)0x00003000) /*!< PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE        ((uint32_t)0x00004000) /*!< PE[15] pin */

/*****************  Bit definition for SYSCFG_CFGR2 register  ****************/
#define SYSCFG_CFGR2_LOCKUP_LOCK               ((uint32_t)0x00000001) /*!< Enables and locks the LOCKUP (Hardfault) output of CortexM4 with Break Input of TIMx */
#define SYSCFG_CFGR2_SRAM_PARITY_LOCK          ((uint32_t)0x00000002) /*!< Enables and locks the SRAM_PARITY error signal with Break Input of TIMx */
#define SYSCFG_CFGR2_PVD_LOCK                  ((uint32_t)0x00000004) /*!< Enables and locks the PVD connection with TIMx Break Input, as well as the PVDE and PLS[2:0] in the PWR_CR register */
#define SYSCFG_CFGR2_BYP_ADDR_PAR              ((uint32_t)0x00000010) /*!< Disables the adddress parity check on RAM */
#define SYSCFG_CFGR2_SRAM_PE                   ((uint32_t)0x00000100) /*!< SRAM Parity error flag */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         ((uint32_t)0x00000001)            /*!<Counter enable */
#define  TIM_CR1_UDIS                        ((uint32_t)0x00000002)            /*!<Update disable */
#define  TIM_CR1_URS                         ((uint32_t)0x00000004)            /*!<Update request source */
#define  TIM_CR1_OPM                         ((uint32_t)0x00000008)            /*!<One pulse mode */
#define  TIM_CR1_DIR                         ((uint32_t)0x00000010)            /*!<Direction */

#define  TIM_CR1_CMS                         ((uint32_t)0x00000060)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       ((uint32_t)0x00000020)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1                       ((uint32_t)0x00000040)            /*!<Bit 1 */

#define  TIM_CR1_ARPE                        ((uint32_t)0x00000080)            /*!<Auto-reload preload enable */

#define  TIM_CR1_CKD                         ((uint32_t)0x00000300)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1                       ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_CR1_UIFREMAP                    ((uint32_t)0x00000800)            /*!<Update interrupt flag remap */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        ((uint32_t)0x00000001)            /*!<Capture/Compare Preloaded Control */
#define  TIM_CR2_CCUS                        ((uint32_t)0x00000004)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        ((uint32_t)0x00000008)            /*!<Capture/Compare DMA Selection */

#define  TIM_CR2_MMS                         ((uint32_t)0x00000070)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1                       ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2                       ((uint32_t)0x00000040)            /*!<Bit 2 */

#define  TIM_CR2_TI1S                        ((uint32_t)0x00000080)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1                        ((uint32_t)0x00000100)            /*!<Output Idle state 1 (OC1 output) */
#define  TIM_CR2_OIS1N                       ((uint32_t)0x00000200)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        ((uint32_t)0x00000400)            /*!<Output Idle state 2 (OC2 output) */
#define  TIM_CR2_OIS2N                       ((uint32_t)0x00000800)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        ((uint32_t)0x00001000)            /*!<Output Idle state 3 (OC3 output) */
#define  TIM_CR2_OIS3N                       ((uint32_t)0x00002000)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        ((uint32_t)0x00004000)            /*!<Output Idle state 4 (OC4 output) */
#define  TIM_CR2_OIS5                        ((uint32_t)0x00010000)            /*!<Output Idle state 4 (OC4 output) */
#define  TIM_CR2_OIS6                        ((uint32_t)0x00040000)            /*!<Output Idle state 4 (OC4 output) */

#define  TIM_CR2_MMS2                        ((uint32_t)0x00F00000)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS2_0                      ((uint32_t)0x00100000)            /*!<Bit 0 */
#define  TIM_CR2_MMS2_1                      ((uint32_t)0x00200000)            /*!<Bit 1 */
#define  TIM_CR2_MMS2_2                      ((uint32_t)0x00400000)            /*!<Bit 2 */
#define  TIM_CR2_MMS2_3                      ((uint32_t)0x00800000)            /*!<Bit 2 */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        ((uint32_t)0x00010007)            /*!<SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMCR_SMS_0                      ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1                      ((uint32_t)0x00000002)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2                      ((uint32_t)0x00000004)            /*!<Bit 2 */
#define  TIM_SMCR_SMS_3                      ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_SMCR_OCCS                       ((uint32_t)0x00000008)            /*!< OCREF clear selection */

#define  TIM_SMCR_TS                         ((uint32_t)0x00000070)            /*!<TS[2:0] bits (Trigger selection) */
#define  TIM_SMCR_TS_0                       ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1                       ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2                       ((uint32_t)0x00000040)            /*!<Bit 2 */

#define  TIM_SMCR_MSM                        ((uint32_t)0x00000080)            /*!<Master/slave mode */

#define  TIM_SMCR_ETF                        ((uint32_t)0x00000F00)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1                      ((uint32_t)0x00000200)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2                      ((uint32_t)0x00000400)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3                      ((uint32_t)0x00000800)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS                       ((uint32_t)0x00003000)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1                     ((uint32_t)0x00002000)            /*!<Bit 1 */

#define  TIM_SMCR_ECE                        ((uint32_t)0x00004000)            /*!<External clock enable */
#define  TIM_SMCR_ETP                        ((uint32_t)0x00008000)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        ((uint32_t)0x00000001)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE                      ((uint32_t)0x00000002)            /*!<Capture/Compare 1 interrupt enable */
#define  TIM_DIER_CC2IE                      ((uint32_t)0x00000004)            /*!<Capture/Compare 2 interrupt enable */
#define  TIM_DIER_CC3IE                      ((uint32_t)0x00000008)            /*!<Capture/Compare 3 interrupt enable */
#define  TIM_DIER_CC4IE                      ((uint32_t)0x00000010)            /*!<Capture/Compare 4 interrupt enable */
#define  TIM_DIER_COMIE                      ((uint32_t)0x00000020)            /*!<COM interrupt enable */
#define  TIM_DIER_TIE                        ((uint32_t)0x00000040)            /*!<Trigger interrupt enable */
#define  TIM_DIER_BIE                        ((uint32_t)0x00000080)            /*!<Break interrupt enable */
#define  TIM_DIER_UDE                        ((uint32_t)0x00000100)            /*!<Update DMA request enable */
#define  TIM_DIER_CC1DE                      ((uint32_t)0x00000200)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      ((uint32_t)0x00000400)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      ((uint32_t)0x00000800)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      ((uint32_t)0x00001000)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      ((uint32_t)0x00002000)            /*!<COM DMA request enable */
#define  TIM_DIER_TDE                        ((uint32_t)0x00004000)            /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          ((uint32_t)0x00000001)            /*!<Update interrupt Flag */
#define  TIM_SR_CC1IF                        ((uint32_t)0x00000002)            /*!<Capture/Compare 1 interrupt Flag */
#define  TIM_SR_CC2IF                        ((uint32_t)0x00000004)            /*!<Capture/Compare 2 interrupt Flag */
#define  TIM_SR_CC3IF                        ((uint32_t)0x00000008)            /*!<Capture/Compare 3 interrupt Flag */
#define  TIM_SR_CC4IF                        ((uint32_t)0x00000010)            /*!<Capture/Compare 4 interrupt Flag */
#define  TIM_SR_COMIF                        ((uint32_t)0x00000020)            /*!<COM interrupt Flag */
#define  TIM_SR_TIF                          ((uint32_t)0x00000040)            /*!<Trigger interrupt Flag */
#define  TIM_SR_BIF                          ((uint32_t)0x00000080)            /*!<Break interrupt Flag */
#define  TIM_SR_B2IF                         ((uint32_t)0x00000100)            /*!<Break2 interrupt Flag */
#define  TIM_SR_CC1OF                        ((uint32_t)0x00000200)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        ((uint32_t)0x00000400)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        ((uint32_t)0x00000800)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        ((uint32_t)0x00001000)            /*!<Capture/Compare 4 Overcapture Flag */
#define  TIM_SR_CC5IF                        ((uint32_t)0x00010000)            /*!<Capture/Compare 5 interrupt Flag */
#define  TIM_SR_CC6IF                        ((uint32_t)0x00020000)            /*!<Capture/Compare 6 interrupt Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          ((uint32_t)0x00000001)               /*!<Update Generation */
#define  TIM_EGR_CC1G                        ((uint32_t)0x00000002)               /*!<Capture/Compare 1 Generation */
#define  TIM_EGR_CC2G                        ((uint32_t)0x00000004)               /*!<Capture/Compare 2 Generation */
#define  TIM_EGR_CC3G                        ((uint32_t)0x00000008)               /*!<Capture/Compare 3 Generation */
#define  TIM_EGR_CC4G                        ((uint32_t)0x00000010)               /*!<Capture/Compare 4 Generation */
#define  TIM_EGR_COMG                        ((uint32_t)0x00000020)               /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          ((uint32_t)0x00000040)               /*!<Trigger Generation */
#define  TIM_EGR_BG                          ((uint32_t)0x00000080)               /*!<Break Generation */
#define  TIM_EGR_B2G                         ((uint32_t)0x00000100)               /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      ((uint32_t)0x00000003)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1                    ((uint32_t)0x00000002)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE                     ((uint32_t)0x00000004)            /*!<Output Compare 1 Fast enable */
#define  TIM_CCMR1_OC1PE                     ((uint32_t)0x00000008)            /*!<Output Compare 1 Preload enable */

#define  TIM_CCMR1_OC1M                      ((uint32_t)0x00010070)            /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_CCMR1_OC1M_0                    ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1                    ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2                    ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR1_OC1M_3                    ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_CCMR1_OC1CE                     ((uint32_t)0x00000080)            /*!<Output Compare 1Clear Enable */

#define  TIM_CCMR1_CC2S                      ((uint32_t)0x00000300)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1                    ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE                     ((uint32_t)0x00000400)            /*!<Output Compare 2 Fast enable */
#define  TIM_CCMR1_OC2PE                     ((uint32_t)0x00000800)            /*!<Output Compare 2 Preload enable */

#define  TIM_CCMR1_OC2M                      ((uint32_t)0x01007000)            /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_CCMR1_OC2M_0                    ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1                    ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2                    ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR1_OC2M_3                    ((uint32_t)0x01000000)            /*!<Bit 3 */

#define  TIM_CCMR1_OC2CE                     ((uint32_t)0x00008000)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    ((uint32_t)0x0000000C)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  ((uint32_t)0x00000004)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  ((uint32_t)0x00000008)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F                      ((uint32_t)0x000000F0)            /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_CCMR1_IC1F_0                    ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1                    ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2                    ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3                    ((uint32_t)0x00000080)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC                    ((uint32_t)0x00000C00)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_CCMR1_IC2PSC_0                  ((uint32_t)0x00000400)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  ((uint32_t)0x00000800)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F                      ((uint32_t)0x0000F000)            /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_CCMR1_IC2F_0                    ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1                    ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2                    ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3                    ((uint32_t)0x00008000)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      ((uint32_t)0x00000003)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CCMR2_CC3S_0                    ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1                    ((uint32_t)0x00000002)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE                     ((uint32_t)0x00000004)            /*!<Output Compare 3 Fast enable */
#define  TIM_CCMR2_OC3PE                     ((uint32_t)0x00000008)            /*!<Output Compare 3 Preload enable */

#define  TIM_CCMR2_OC3M                      ((uint32_t)0x00010070)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1                    ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2                    ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR2_OC3M_3                    ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_CCMR2_OC3CE                     ((uint32_t)0x00000080)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      ((uint32_t)0x00000300)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1                    ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE                     ((uint32_t)0x00000400)            /*!<Output Compare 4 Fast enable */
#define  TIM_CCMR2_OC4PE                     ((uint32_t)0x00000800)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      ((uint32_t)0x01007000)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1                    ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2                    ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR2_OC4M_3                    ((uint32_t)0x01000000)            /*!<Bit 3 */

#define  TIM_CCMR2_OC4CE                     ((uint32_t)0x00008000)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    ((uint32_t)0x00000000000C)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  ((uint32_t)0x000000000004)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  ((uint32_t)0x000000000008)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F                      ((uint32_t)0x0000000000F0)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    ((uint32_t)0x000000000010)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1                    ((uint32_t)0x000000000020)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2                    ((uint32_t)0x000000000040)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3                    ((uint32_t)0x000000000080)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC                    ((uint32_t)0x000000000C00)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  ((uint32_t)0x000000000400)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  ((uint32_t)0x000000000800)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F                      ((uint32_t)0x00000000F000)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    ((uint32_t)0x000000001000)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1                    ((uint32_t)0x000000002000)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2                    ((uint32_t)0x000000004000)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3                    ((uint32_t)0x000000008000)            /*!<Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       ((uint32_t)0x00000001)            /*!<Capture/Compare 1 output enable */
#define  TIM_CCER_CC1P                       ((uint32_t)0x00000002)            /*!<Capture/Compare 1 output Polarity */
#define  TIM_CCER_CC1NE                      ((uint32_t)0x00000004)            /*!<Capture/Compare 1 Complementary output enable */
#define  TIM_CCER_CC1NP                      ((uint32_t)0x00000008)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       ((uint32_t)0x00000010)            /*!<Capture/Compare 2 output enable */
#define  TIM_CCER_CC2P                       ((uint32_t)0x00000020)            /*!<Capture/Compare 2 output Polarity */
#define  TIM_CCER_CC2NE                      ((uint32_t)0x00000040)            /*!<Capture/Compare 2 Complementary output enable */
#define  TIM_CCER_CC2NP                      ((uint32_t)0x00000080)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       ((uint32_t)0x00000100)            /*!<Capture/Compare 3 output enable */
#define  TIM_CCER_CC3P                       ((uint32_t)0x00000200)            /*!<Capture/Compare 3 output Polarity */
#define  TIM_CCER_CC3NE                      ((uint32_t)0x00000400)            /*!<Capture/Compare 3 Complementary output enable */
#define  TIM_CCER_CC3NP                      ((uint32_t)0x00000800)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       ((uint32_t)0x00001000)            /*!<Capture/Compare 4 output enable */
#define  TIM_CCER_CC4P                       ((uint32_t)0x00002000)            /*!<Capture/Compare 4 output Polarity */
#define  TIM_CCER_CC4NP                      ((uint32_t)0x00008000)            /*!<Capture/Compare 4 Complementary output Polarity */
#define  TIM_CCER_CC5E                       ((uint32_t)0x00010000)            /*!<Capture/Compare 5 output enable */
#define  TIM_CCER_CC5P                       ((uint32_t)0x00020000)            /*!<Capture/Compare 5 output Polarity */
#define  TIM_CCER_CC6E                       ((uint32_t)0x00100000)            /*!<Capture/Compare 6 output enable */
#define  TIM_CCER_CC6P                       ((uint32_t)0x00200000)            /*!<Capture/Compare 6 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         ((uint32_t)0xFFFFFFFF)            /*!<Counter Value */
#define  TIM_CNT_UIFCPY                      ((uint32_t)0x80000000)            /*!<Update interrupt flag copy */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         ((uint32_t)0x0000FFFF)            /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         ((uint32_t)0xFFFFFFFF)            /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         ((uint32_t)0x0000FFFF)            /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_CCR5 register  *******************/
#define  TIM_CCR5_CCR5                       ((uint32_t)0xFFFFFFFF)        /*!<Capture/Compare 5 Value */
#define  TIM_CCR5_GC5C1                      ((uint32_t)0x20000000)        /*!<Group Channel 5 and Channel 1 */
#define  TIM_CCR5_GC5C2                      ((uint32_t)0x40000000)        /*!<Group Channel 5 and Channel 2 */
#define  TIM_CCR5_GC5C3                      ((uint32_t)0x80000000)        /*!<Group Channel 5 and Channel 3 */

/*******************  Bit definition for TIM_CCR6 register  *******************/
#define  TIM_CCR6_CCR6                       ((uint32_t)0x0000FFFF)            /*!<Capture/Compare 6 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        ((uint32_t)0x000000FF)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1                      ((uint32_t)0x00000002)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2                      ((uint32_t)0x00000004)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3                      ((uint32_t)0x00000008)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4                      ((uint32_t)0x00000010)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5                      ((uint32_t)0x00000020)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6                      ((uint32_t)0x00000040)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7                      ((uint32_t)0x00000080)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK                       ((uint32_t)0x00000300)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1                     ((uint32_t)0x00000200)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI                       ((uint32_t)0x00000400)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       ((uint32_t)0x00000800)            /*!<Off-State Selection for Run mode */
#define  TIM_BDTR_BKE                        ((uint32_t)0x00001000)            /*!<Break enable for Break1 */
#define  TIM_BDTR_BKP                        ((uint32_t)0x00002000)            /*!<Break Polarity for Break1 */
#define  TIM_BDTR_AOE                        ((uint32_t)0x00004000)            /*!<Automatic Output enable */
#define  TIM_BDTR_MOE                        ((uint32_t)0x00008000)            /*!<Main Output enable */

#define  TIM_BDTR_BKF                        ((uint32_t)0x000F0000)            /*!<Break Filter for Break1 */
#define  TIM_BDTR_BK2F                       ((uint32_t)0x00F00000)            /*!<Break Filter for Break2 */

#define  TIM_BDTR_BK2E                       ((uint32_t)0x01000000)            /*!<Break enable for Break2 */
#define  TIM_BDTR_BK2P                       ((uint32_t)0x02000000)            /*!<Break Polarity for Break2 */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         ((uint32_t)0x0000001F)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       ((uint32_t)0x00000001)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1                       ((uint32_t)0x00000002)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2                       ((uint32_t)0x00000004)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3                       ((uint32_t)0x00000008)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4                       ((uint32_t)0x00000010)            /*!<Bit 4 */

#define  TIM_DCR_DBL                         ((uint32_t)0x00001F00)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       ((uint32_t)0x00000100)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1                       ((uint32_t)0x00000200)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2                       ((uint32_t)0x00000400)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3                       ((uint32_t)0x00000800)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4                       ((uint32_t)0x00001000)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       ((uint32_t)0x0000FFFF)            /*!<DMA register for burst accesses */

/*******************  Bit definition for TIM16_OR register  *********************/
#define TIM16_OR_TI1_RMP                     ((uint32_t)0x000000C0)            /*!<TI1_RMP[1:0] bits (TIM16 Input 1 remap) */
#define TIM16_OR_TI1_RMP_0                   ((uint32_t)0x00000040)            /*!<Bit 0 */
#define TIM16_OR_TI1_RMP_1                   ((uint32_t)0x00000080)            /*!<Bit 1 */

/*******************  Bit definition for TIM1_OR register  *********************/
#define TIM1_OR_ETR_RMP                      ((uint32_t)0x0000000F)            /*!<ETR_RMP[3:0] bits (TIM1 ETR remap) */
#define TIM1_OR_ETR_RMP_0                    ((uint32_t)0x00000001)            /*!<Bit 0 */
#define TIM1_OR_ETR_RMP_1                    ((uint32_t)0x00000002)            /*!<Bit 1 */
#define TIM1_OR_ETR_RMP_2                    ((uint32_t)0x00000004)            /*!<Bit 2 */
#define TIM1_OR_ETR_RMP_3                    ((uint32_t)0x00000008)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR3 register  *******************/
#define  TIM_CCMR3_OC5FE                     ((uint32_t)0x00000004)            /*!<Output Compare 5 Fast enable */
#define  TIM_CCMR3_OC5PE                     ((uint32_t)0x00000008)            /*!<Output Compare 5 Preload enable */

#define  TIM_CCMR3_OC5M                      ((uint32_t)0x00010070)            /*!<OC5M[2:0] bits (Output Compare 5 Mode) */
#define  TIM_CCMR3_OC5M_0                    ((uint32_t)0x00000010)            /*!<Bit 0 */
#define  TIM_CCMR3_OC5M_1                    ((uint32_t)0x00000020)            /*!<Bit 1 */
#define  TIM_CCMR3_OC5M_2                    ((uint32_t)0x00000040)            /*!<Bit 2 */
#define  TIM_CCMR3_OC5M_3                    ((uint32_t)0x00010000)            /*!<Bit 3 */

#define  TIM_CCMR3_OC5CE                     ((uint32_t)0x00000080)            /*!<Output Compare 5 Clear Enable */

#define  TIM_CCMR3_OC6FE                     ((uint32_t)0x00000400)            /*!<Output Compare 6 Fast enable */
#define  TIM_CCMR3_OC6PE                     ((uint32_t)0x00000800)            /*!<Output Compare 6 Preload enable */

#define  TIM_CCMR3_OC6M                      ((uint32_t)0x01007000)            /*!<OC6M[2:0] bits (Output Compare 6 Mode) */
#define  TIM_CCMR3_OC6M_0                    ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TIM_CCMR3_OC6M_1                    ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TIM_CCMR3_OC6M_2                    ((uint32_t)0x00004000)            /*!<Bit 2 */
#define  TIM_CCMR3_OC6M_3                    ((uint32_t)0x01000000)            /*!<Bit 3 */

#define  TIM_CCMR3_OC6CE                     ((uint32_t)0x00008000)            /*!<Output Compare 6 Clear Enable */

/******************************************************************************/
/*                                                                            */
/*                          Touch Sensing Controller (TSC)                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TSC_CR register  *********************/
#define  TSC_CR_TSCE                         ((uint32_t)0x00000001)            /*!<Touch sensing controller enable */
#define  TSC_CR_START                        ((uint32_t)0x00000002)            /*!<Start acquisition */
#define  TSC_CR_AM                           ((uint32_t)0x00000004)            /*!<Acquisition mode */
#define  TSC_CR_SYNCPOL                      ((uint32_t)0x00000008)            /*!<Synchronization pin polarity */
#define  TSC_CR_IODEF                        ((uint32_t)0x00000010)            /*!<IO default mode */

#define  TSC_CR_MCV                          ((uint32_t)0x000000E0)            /*!<MCV[2:0] bits (Max Count Value) */
#define  TSC_CR_MCV_0                        ((uint32_t)0x00000020)            /*!<Bit 0 */
#define  TSC_CR_MCV_1                        ((uint32_t)0x00000040)            /*!<Bit 1 */
#define  TSC_CR_MCV_2                        ((uint32_t)0x00000080)            /*!<Bit 2 */

#define  TSC_CR_PGPSC                        ((uint32_t)0x00007000)            /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
#define  TSC_CR_PGPSC_0                      ((uint32_t)0x00001000)            /*!<Bit 0 */
#define  TSC_CR_PGPSC_1                      ((uint32_t)0x00002000)            /*!<Bit 1 */
#define  TSC_CR_PGPSC_2                      ((uint32_t)0x00004000)            /*!<Bit 2 */

#define  TSC_CR_SSPSC                        ((uint32_t)0x00008000)            /*!<Spread Spectrum Prescaler */
#define  TSC_CR_SSE                          ((uint32_t)0x00010000)            /*!<Spread Spectrum Enable */

#define  TSC_CR_SSD                          ((uint32_t)0x00FE0000)            /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
#define  TSC_CR_SSD_0                        ((uint32_t)0x00020000)            /*!<Bit 0 */
#define  TSC_CR_SSD_1                        ((uint32_t)0x00040000)            /*!<Bit 1 */
#define  TSC_CR_SSD_2                        ((uint32_t)0x00080000)            /*!<Bit 2 */
#define  TSC_CR_SSD_3                        ((uint32_t)0x00100000)            /*!<Bit 3 */
#define  TSC_CR_SSD_4                        ((uint32_t)0x00200000)            /*!<Bit 4 */
#define  TSC_CR_SSD_5                        ((uint32_t)0x00400000)            /*!<Bit 5 */
#define  TSC_CR_SSD_6                        ((uint32_t)0x00800000)            /*!<Bit 6 */

#define  TSC_CR_CTPL                         ((uint32_t)0x0F000000)            /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
#define  TSC_CR_CTPL_0                       ((uint32_t)0x01000000)            /*!<Bit 0 */
#define  TSC_CR_CTPL_1                       ((uint32_t)0x02000000)            /*!<Bit 1 */
#define  TSC_CR_CTPL_2                       ((uint32_t)0x04000000)            /*!<Bit 2 */
#define  TSC_CR_CTPL_3                       ((uint32_t)0x08000000)            /*!<Bit 3 */

#define  TSC_CR_CTPH                         ((uint32_t)0xF0000000)            /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
#define  TSC_CR_CTPH_0                       ((uint32_t)0x10000000)            /*!<Bit 0 */
#define  TSC_CR_CTPH_1                       ((uint32_t)0x20000000)            /*!<Bit 1 */
#define  TSC_CR_CTPH_2                       ((uint32_t)0x40000000)            /*!<Bit 2 */
#define  TSC_CR_CTPH_3                       ((uint32_t)0x80000000)            /*!<Bit 3 */

/*******************  Bit definition for TSC_IER register  ********************/
#define  TSC_IER_EOAIE                       ((uint32_t)0x00000001)            /*!<End of acquisition interrupt enable */
#define  TSC_IER_MCEIE                       ((uint32_t)0x00000002)            /*!<Max count error interrupt enable */

/*******************  Bit definition for TSC_ICR register  ********************/
#define  TSC_ICR_EOAIC                       ((uint32_t)0x00000001)            /*!<End of acquisition interrupt clear */
#define  TSC_ICR_MCEIC                       ((uint32_t)0x00000002)            /*!<Max count error interrupt clear */

/*******************  Bit definition for TSC_ISR register  ********************/
#define  TSC_ISR_EOAF                        ((uint32_t)0x00000001)            /*!<End of acquisition flag */
#define  TSC_ISR_MCEF                        ((uint32_t)0x00000002)            /*!<Max count error flag */

/*******************  Bit definition for TSC_IOHCR register  ******************/
#define  TSC_IOHCR_G1_IO1                    ((uint32_t)0x00000001)            /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G1_IO2                    ((uint32_t)0x00000002)            /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G1_IO3                    ((uint32_t)0x00000004)            /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G1_IO4                    ((uint32_t)0x00000008)            /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G2_IO1                    ((uint32_t)0x00000010)            /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G2_IO2                    ((uint32_t)0x00000020)            /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G2_IO3                    ((uint32_t)0x00000040)            /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G2_IO4                    ((uint32_t)0x00000080)            /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G3_IO1                    ((uint32_t)0x00000100)            /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G3_IO2                    ((uint32_t)0x00000200)            /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G3_IO3                    ((uint32_t)0x00000400)            /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G3_IO4                    ((uint32_t)0x00000800)            /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G4_IO1                    ((uint32_t)0x00001000)            /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G4_IO2                    ((uint32_t)0x00002000)            /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G4_IO3                    ((uint32_t)0x00004000)            /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G4_IO4                    ((uint32_t)0x00008000)            /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G5_IO1                    ((uint32_t)0x00010000)            /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G5_IO2                    ((uint32_t)0x00020000)            /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G5_IO3                    ((uint32_t)0x00040000)            /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G5_IO4                    ((uint32_t)0x00080000)            /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G6_IO1                    ((uint32_t)0x00100000)            /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G6_IO2                    ((uint32_t)0x00200000)            /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G6_IO3                    ((uint32_t)0x00400000)            /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G6_IO4                    ((uint32_t)0x00800000)            /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G7_IO1                    ((uint32_t)0x01000000)            /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G7_IO2                    ((uint32_t)0x02000000)            /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G7_IO3                    ((uint32_t)0x04000000)            /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G7_IO4                    ((uint32_t)0x08000000)            /*!<GROUP7_IO4 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G8_IO1                    ((uint32_t)0x10000000)            /*!<GROUP8_IO1 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G8_IO2                    ((uint32_t)0x20000000)            /*!<GROUP8_IO2 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G8_IO3                    ((uint32_t)0x40000000)            /*!<GROUP8_IO3 schmitt trigger hysteresis mode */
#define  TSC_IOHCR_G8_IO4                    ((uint32_t)0x80000000)            /*!<GROUP8_IO4 schmitt trigger hysteresis mode */

/*******************  Bit definition for TSC_IOASCR register  *****************/
#define  TSC_IOASCR_G1_IO1                   ((uint32_t)0x00000001)            /*!<GROUP1_IO1 analog switch enable */
#define  TSC_IOASCR_G1_IO2                   ((uint32_t)0x00000002)            /*!<GROUP1_IO2 analog switch enable */
#define  TSC_IOASCR_G1_IO3                   ((uint32_t)0x00000004)            /*!<GROUP1_IO3 analog switch enable */
#define  TSC_IOASCR_G1_IO4                   ((uint32_t)0x00000008)            /*!<GROUP1_IO4 analog switch enable */
#define  TSC_IOASCR_G2_IO1                   ((uint32_t)0x00000010)            /*!<GROUP2_IO1 analog switch enable */
#define  TSC_IOASCR_G2_IO2                   ((uint32_t)0x00000020)            /*!<GROUP2_IO2 analog switch enable */
#define  TSC_IOASCR_G2_IO3                   ((uint32_t)0x00000040)            /*!<GROUP2_IO3 analog switch enable */
#define  TSC_IOASCR_G2_IO4                   ((uint32_t)0x00000080)            /*!<GROUP2_IO4 analog switch enable */
#define  TSC_IOASCR_G3_IO1                   ((uint32_t)0x00000100)            /*!<GROUP3_IO1 analog switch enable */
#define  TSC_IOASCR_G3_IO2                   ((uint32_t)0x00000200)            /*!<GROUP3_IO2 analog switch enable */
#define  TSC_IOASCR_G3_IO3                   ((uint32_t)0x00000400)            /*!<GROUP3_IO3 analog switch enable */
#define  TSC_IOASCR_G3_IO4                   ((uint32_t)0x00000800)            /*!<GROUP3_IO4 analog switch enable */
#define  TSC_IOASCR_G4_IO1                   ((uint32_t)0x00001000)            /*!<GROUP4_IO1 analog switch enable */
#define  TSC_IOASCR_G4_IO2                   ((uint32_t)0x00002000)            /*!<GROUP4_IO2 analog switch enable */
#define  TSC_IOASCR_G4_IO3                   ((uint32_t)0x00004000)            /*!<GROUP4_IO3 analog switch enable */
#define  TSC_IOASCR_G4_IO4                   ((uint32_t)0x00008000)            /*!<GROUP4_IO4 analog switch enable */
#define  TSC_IOASCR_G5_IO1                   ((uint32_t)0x00010000)            /*!<GROUP5_IO1 analog switch enable */
#define  TSC_IOASCR_G5_IO2                   ((uint32_t)0x00020000)            /*!<GROUP5_IO2 analog switch enable */
#define  TSC_IOASCR_G5_IO3                   ((uint32_t)0x00040000)            /*!<GROUP5_IO3 analog switch enable */
#define  TSC_IOASCR_G5_IO4                   ((uint32_t)0x00080000)            /*!<GROUP5_IO4 analog switch enable */
#define  TSC_IOASCR_G6_IO1                   ((uint32_t)0x00100000)            /*!<GROUP6_IO1 analog switch enable */
#define  TSC_IOASCR_G6_IO2                   ((uint32_t)0x00200000)            /*!<GROUP6_IO2 analog switch enable */
#define  TSC_IOASCR_G6_IO3                   ((uint32_t)0x00400000)            /*!<GROUP6_IO3 analog switch enable */
#define  TSC_IOASCR_G6_IO4                   ((uint32_t)0x00800000)            /*!<GROUP6_IO4 analog switch enable */
#define  TSC_IOASCR_G7_IO1                   ((uint32_t)0x01000000)            /*!<GROUP7_IO1 analog switch enable */
#define  TSC_IOASCR_G7_IO2                   ((uint32_t)0x02000000)            /*!<GROUP7_IO2 analog switch enable */
#define  TSC_IOASCR_G7_IO3                   ((uint32_t)0x04000000)            /*!<GROUP7_IO3 analog switch enable */
#define  TSC_IOASCR_G7_IO4                   ((uint32_t)0x08000000)            /*!<GROUP7_IO4 analog switch enable */
#define  TSC_IOASCR_G8_IO1                   ((uint32_t)0x10000000)            /*!<GROUP8_IO1 analog switch enable */
#define  TSC_IOASCR_G8_IO2                   ((uint32_t)0x20000000)            /*!<GROUP8_IO2 analog switch enable */
#define  TSC_IOASCR_G8_IO3                   ((uint32_t)0x40000000)            /*!<GROUP8_IO3 analog switch enable */
#define  TSC_IOASCR_G8_IO4                   ((uint32_t)0x80000000)            /*!<GROUP8_IO4 analog switch enable */

/*******************  Bit definition for TSC_IOSCR register  ******************/
#define  TSC_IOSCR_G1_IO1                    ((uint32_t)0x00000001)            /*!<GROUP1_IO1 sampling mode */
#define  TSC_IOSCR_G1_IO2                    ((uint32_t)0x00000002)            /*!<GROUP1_IO2 sampling mode */
#define  TSC_IOSCR_G1_IO3                    ((uint32_t)0x00000004)            /*!<GROUP1_IO3 sampling mode */
#define  TSC_IOSCR_G1_IO4                    ((uint32_t)0x00000008)            /*!<GROUP1_IO4 sampling mode */
#define  TSC_IOSCR_G2_IO1                    ((uint32_t)0x00000010)            /*!<GROUP2_IO1 sampling mode */
#define  TSC_IOSCR_G2_IO2                    ((uint32_t)0x00000020)            /*!<GROUP2_IO2 sampling mode */
#define  TSC_IOSCR_G2_IO3                    ((uint32_t)0x00000040)            /*!<GROUP2_IO3 sampling mode */
#define  TSC_IOSCR_G2_IO4                    ((uint32_t)0x00000080)            /*!<GROUP2_IO4 sampling mode */
#define  TSC_IOSCR_G3_IO1                    ((uint32_t)0x00000100)            /*!<GROUP3_IO1 sampling mode */
#define  TSC_IOSCR_G3_IO2                    ((uint32_t)0x00000200)            /*!<GROUP3_IO2 sampling mode */
#define  TSC_IOSCR_G3_IO3                    ((uint32_t)0x00000400)            /*!<GROUP3_IO3 sampling mode */
#define  TSC_IOSCR_G3_IO4                    ((uint32_t)0x00000800)            /*!<GROUP3_IO4 sampling mode */
#define  TSC_IOSCR_G4_IO1                    ((uint32_t)0x00001000)            /*!<GROUP4_IO1 sampling mode */
#define  TSC_IOSCR_G4_IO2                    ((uint32_t)0x00002000)            /*!<GROUP4_IO2 sampling mode */
#define  TSC_IOSCR_G4_IO3                    ((uint32_t)0x00004000)            /*!<GROUP4_IO3 sampling mode */
#define  TSC_IOSCR_G4_IO4                    ((uint32_t)0x00008000)            /*!<GROUP4_IO4 sampling mode */
#define  TSC_IOSCR_G5_IO1                    ((uint32_t)0x00010000)            /*!<GROUP5_IO1 sampling mode */
#define  TSC_IOSCR_G5_IO2                    ((uint32_t)0x00020000)            /*!<GROUP5_IO2 sampling mode */
#define  TSC_IOSCR_G5_IO3                    ((uint32_t)0x00040000)            /*!<GROUP5_IO3 sampling mode */
#define  TSC_IOSCR_G5_IO4                    ((uint32_t)0x00080000)            /*!<GROUP5_IO4 sampling mode */
#define  TSC_IOSCR_G6_IO1                    ((uint32_t)0x00100000)            /*!<GROUP6_IO1 sampling mode */
#define  TSC_IOSCR_G6_IO2                    ((uint32_t)0x00200000)            /*!<GROUP6_IO2 sampling mode */
#define  TSC_IOSCR_G6_IO3                    ((uint32_t)0x00400000)            /*!<GROUP6_IO3 sampling mode */
#define  TSC_IOSCR_G6_IO4                    ((uint32_t)0x00800000)            /*!<GROUP6_IO4 sampling mode */
#define  TSC_IOSCR_G7_IO1                    ((uint32_t)0x01000000)            /*!<GROUP7_IO1 sampling mode */
#define  TSC_IOSCR_G7_IO2                    ((uint32_t)0x02000000)            /*!<GROUP7_IO2 sampling mode */
#define  TSC_IOSCR_G7_IO3                    ((uint32_t)0x04000000)            /*!<GROUP7_IO3 sampling mode */
#define  TSC_IOSCR_G7_IO4                    ((uint32_t)0x08000000)            /*!<GROUP7_IO4 sampling mode */
#define  TSC_IOSCR_G8_IO1                    ((uint32_t)0x10000000)            /*!<GROUP8_IO1 sampling mode */
#define  TSC_IOSCR_G8_IO2                    ((uint32_t)0x20000000)            /*!<GROUP8_IO2 sampling mode */
#define  TSC_IOSCR_G8_IO3                    ((uint32_t)0x40000000)            /*!<GROUP8_IO3 sampling mode */
#define  TSC_IOSCR_G8_IO4                    ((uint32_t)0x80000000)            /*!<GROUP8_IO4 sampling mode */

/*******************  Bit definition for TSC_IOCCR register  ******************/
#define  TSC_IOCCR_G1_IO1                    ((uint32_t)0x00000001)            /*!<GROUP1_IO1 channel mode */
#define  TSC_IOCCR_G1_IO2                    ((uint32_t)0x00000002)            /*!<GROUP1_IO2 channel mode */
#define  TSC_IOCCR_G1_IO3                    ((uint32_t)0x00000004)            /*!<GROUP1_IO3 channel mode */
#define  TSC_IOCCR_G1_IO4                    ((uint32_t)0x00000008)            /*!<GROUP1_IO4 channel mode */
#define  TSC_IOCCR_G2_IO1                    ((uint32_t)0x00000010)            /*!<GROUP2_IO1 channel mode */
#define  TSC_IOCCR_G2_IO2                    ((uint32_t)0x00000020)            /*!<GROUP2_IO2 channel mode */
#define  TSC_IOCCR_G2_IO3                    ((uint32_t)0x00000040)            /*!<GROUP2_IO3 channel mode */
#define  TSC_IOCCR_G2_IO4                    ((uint32_t)0x00000080)            /*!<GROUP2_IO4 channel mode */
#define  TSC_IOCCR_G3_IO1                    ((uint32_t)0x00000100)            /*!<GROUP3_IO1 channel mode */
#define  TSC_IOCCR_G3_IO2                    ((uint32_t)0x00000200)            /*!<GROUP3_IO2 channel mode */
#define  TSC_IOCCR_G3_IO3                    ((uint32_t)0x00000400)            /*!<GROUP3_IO3 channel mode */
#define  TSC_IOCCR_G3_IO4                    ((uint32_t)0x00000800)            /*!<GROUP3_IO4 channel mode */
#define  TSC_IOCCR_G4_IO1                    ((uint32_t)0x00001000)            /*!<GROUP4_IO1 channel mode */
#define  TSC_IOCCR_G4_IO2                    ((uint32_t)0x00002000)            /*!<GROUP4_IO2 channel mode */
#define  TSC_IOCCR_G4_IO3                    ((uint32_t)0x00004000)            /*!<GROUP4_IO3 channel mode */
#define  TSC_IOCCR_G4_IO4                    ((uint32_t)0x00008000)            /*!<GROUP4_IO4 channel mode */
#define  TSC_IOCCR_G5_IO1                    ((uint32_t)0x00010000)            /*!<GROUP5_IO1 channel mode */
#define  TSC_IOCCR_G5_IO2                    ((uint32_t)0x00020000)            /*!<GROUP5_IO2 channel mode */
#define  TSC_IOCCR_G5_IO3                    ((uint32_t)0x00040000)            /*!<GROUP5_IO3 channel mode */
#define  TSC_IOCCR_G5_IO4                    ((uint32_t)0x00080000)            /*!<GROUP5_IO4 channel mode */
#define  TSC_IOCCR_G6_IO1                    ((uint32_t)0x00100000)            /*!<GROUP6_IO1 channel mode */
#define  TSC_IOCCR_G6_IO2                    ((uint32_t)0x00200000)            /*!<GROUP6_IO2 channel mode */
#define  TSC_IOCCR_G6_IO3                    ((uint32_t)0x00400000)            /*!<GROUP6_IO3 channel mode */
#define  TSC_IOCCR_G6_IO4                    ((uint32_t)0x00800000)            /*!<GROUP6_IO4 channel mode */
#define  TSC_IOCCR_G7_IO1                    ((uint32_t)0x01000000)            /*!<GROUP7_IO1 channel mode */
#define  TSC_IOCCR_G7_IO2                    ((uint32_t)0x02000000)            /*!<GROUP7_IO2 channel mode */
#define  TSC_IOCCR_G7_IO3                    ((uint32_t)0x04000000)            /*!<GROUP7_IO3 channel mode */
#define  TSC_IOCCR_G7_IO4                    ((uint32_t)0x08000000)            /*!<GROUP7_IO4 channel mode */
#define  TSC_IOCCR_G8_IO1                    ((uint32_t)0x10000000)            /*!<GROUP8_IO1 channel mode */
#define  TSC_IOCCR_G8_IO2                    ((uint32_t)0x20000000)            /*!<GROUP8_IO2 channel mode */
#define  TSC_IOCCR_G8_IO3                    ((uint32_t)0x40000000)            /*!<GROUP8_IO3 channel mode */
#define  TSC_IOCCR_G8_IO4                    ((uint32_t)0x80000000)            /*!<GROUP8_IO4 channel mode */

/*******************  Bit definition for TSC_IOGCSR register  *****************/
#define  TSC_IOGCSR_G1E                      ((uint32_t)0x00000001)            /*!<Analog IO GROUP1 enable */
#define  TSC_IOGCSR_G2E                      ((uint32_t)0x00000002)            /*!<Analog IO GROUP2 enable */
#define  TSC_IOGCSR_G3E                      ((uint32_t)0x00000004)            /*!<Analog IO GROUP3 enable */
#define  TSC_IOGCSR_G4E                      ((uint32_t)0x00000008)            /*!<Analog IO GROUP4 enable */
#define  TSC_IOGCSR_G5E                      ((uint32_t)0x00000010)            /*!<Analog IO GROUP5 enable */
#define  TSC_IOGCSR_G6E                      ((uint32_t)0x00000020)            /*!<Analog IO GROUP6 enable */
#define  TSC_IOGCSR_G7E                      ((uint32_t)0x00000040)            /*!<Analog IO GROUP7 enable */
#define  TSC_IOGCSR_G8E                      ((uint32_t)0x00000080)            /*!<Analog IO GROUP8 enable */
#define  TSC_IOGCSR_G1S                      ((uint32_t)0x00010000)            /*!<Analog IO GROUP1 status */
#define  TSC_IOGCSR_G2S                      ((uint32_t)0x00020000)            /*!<Analog IO GROUP2 status */
#define  TSC_IOGCSR_G3S                      ((uint32_t)0x00040000)            /*!<Analog IO GROUP3 status */
#define  TSC_IOGCSR_G4S                      ((uint32_t)0x00080000)            /*!<Analog IO GROUP4 status */
#define  TSC_IOGCSR_G5S                      ((uint32_t)0x00100000)            /*!<Analog IO GROUP5 status */
#define  TSC_IOGCSR_G6S                      ((uint32_t)0x00200000)            /*!<Analog IO GROUP6 status */
#define  TSC_IOGCSR_G7S                      ((uint32_t)0x00400000)            /*!<Analog IO GROUP7 status */
#define  TSC_IOGCSR_G8S                      ((uint32_t)0x00800000)            /*!<Analog IO GROUP8 status */

/*******************  Bit definition for TSC_IOGXCR register  *****************/
#define  TSC_IOGXCR_CNT                      ((uint32_t)0x00003FFF)            /*!<CNT[13:0] bits (Counter value) */

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_UE                        ((uint32_t)0x00000001)            /*!< USART Enable */
#define  USART_CR1_UESM                      ((uint32_t)0x00000002)            /*!< USART Enable in STOP Mode */
#define  USART_CR1_RE                        ((uint32_t)0x00000004)            /*!< Receiver Enable */
#define  USART_CR1_TE                        ((uint32_t)0x00000008)            /*!< Transmitter Enable */
#define  USART_CR1_IDLEIE                    ((uint32_t)0x00000010)            /*!< IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    ((uint32_t)0x00000020)            /*!< RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      ((uint32_t)0x00000040)            /*!< Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((uint32_t)0x00000080)            /*!< TXE Interrupt Enable */
#define  USART_CR1_PEIE                      ((uint32_t)0x00000100)            /*!< PE Interrupt Enable */
#define  USART_CR1_PS                        ((uint32_t)0x00000200)            /*!< Parity Selection */
#define  USART_CR1_PCE                       ((uint32_t)0x00000400)            /*!< Parity Control Enable */
#define  USART_CR1_WAKE                      ((uint32_t)0x00000800)            /*!< Receiver Wakeup method */
#define  USART_CR1_M                         ((uint32_t)0x00001000)            /*!< Word length */
#define  USART_CR1_M0                        ((uint32_t)0x00001000)            /*!< SmartCard Word length */
#define  USART_CR1_MME                       ((uint32_t)0x00002000)            /*!< Mute Mode Enable */
#define  USART_CR1_CMIE                      ((uint32_t)0x00004000)            /*!< Character match interrupt enable */
#define  USART_CR1_OVER8                     ((uint32_t)0x00008000)            /*!< Oversampling by 8-bit or 16-bit mode */
#define  USART_CR1_DEDT                      ((uint32_t)0x001F0000)            /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define  USART_CR1_DEDT_0                    ((uint32_t)0x00010000)            /*!< Bit 0 */
#define  USART_CR1_DEDT_1                    ((uint32_t)0x00020000)            /*!< Bit 1 */
#define  USART_CR1_DEDT_2                    ((uint32_t)0x00040000)            /*!< Bit 2 */
#define  USART_CR1_DEDT_3                    ((uint32_t)0x00080000)            /*!< Bit 3 */
#define  USART_CR1_DEDT_4                    ((uint32_t)0x00100000)            /*!< Bit 4 */
#define  USART_CR1_DEAT                      ((uint32_t)0x03E00000)            /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define  USART_CR1_DEAT_0                    ((uint32_t)0x00200000)            /*!< Bit 0 */
#define  USART_CR1_DEAT_1                    ((uint32_t)0x00400000)            /*!< Bit 1 */
#define  USART_CR1_DEAT_2                    ((uint32_t)0x00800000)            /*!< Bit 2 */
#define  USART_CR1_DEAT_3                    ((uint32_t)0x01000000)            /*!< Bit 3 */
#define  USART_CR1_DEAT_4                    ((uint32_t)0x02000000)            /*!< Bit 4 */
#define  USART_CR1_RTOIE                     ((uint32_t)0x04000000)            /*!< Receive Time Out interrupt enable */
#define  USART_CR1_EOBIE                     ((uint32_t)0x08000000)            /*!< End of Block interrupt enable */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADDM7                     ((uint32_t)0x00000010)            /*!< 7-bit or 4-bit Address Detection */
#define  USART_CR2_LBDL                      ((uint32_t)0x00000020)            /*!< LIN Break Detection Length */
#define  USART_CR2_LBDIE                     ((uint32_t)0x00000040)            /*!< LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((uint32_t)0x00000100)            /*!< Last Bit Clock pulse */
#define  USART_CR2_CPHA                      ((uint32_t)0x00000200)            /*!< Clock Phase */
#define  USART_CR2_CPOL                      ((uint32_t)0x00000400)            /*!< Clock Polarity */
#define  USART_CR2_CLKEN                     ((uint32_t)0x00000800)            /*!< Clock Enable */
#define  USART_CR2_STOP                      ((uint32_t)0x00003000)            /*!< STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((uint32_t)0x00001000)            /*!< Bit 0 */
#define  USART_CR2_STOP_1                    ((uint32_t)0x00002000)            /*!< Bit 1 */
#define  USART_CR2_LINEN                     ((uint32_t)0x00004000)            /*!< LIN mode enable */
#define  USART_CR2_SWAP                      ((uint32_t)0x00008000)            /*!< SWAP TX/RX pins */
#define  USART_CR2_RXINV                     ((uint32_t)0x00010000)            /*!< RX pin active level inversion */
#define  USART_CR2_TXINV                     ((uint32_t)0x00020000)            /*!< TX pin active level inversion */
#define  USART_CR2_DATAINV                   ((uint32_t)0x00040000)            /*!< Binary data inversion */
#define  USART_CR2_MSBFIRST                  ((uint32_t)0x00080000)            /*!< Most Significant Bit First */
#define  USART_CR2_ABREN                     ((uint32_t)0x00100000)            /*!< Auto Baud-Rate Enable*/
#define  USART_CR2_ABRMODE                   ((uint32_t)0x00600000)            /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define  USART_CR2_ABRMODE_0                 ((uint32_t)0x00200000)            /*!< Bit 0 */
#define  USART_CR2_ABRMODE_1                 ((uint32_t)0x00400000)            /*!< Bit 1 */
#define  USART_CR2_RTOEN                     ((uint32_t)0x00800000)            /*!< Receiver Time-Out enable */
#define  USART_CR2_ADD                       ((uint32_t)0xFF000000)            /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((uint32_t)0x00000001)            /*!< Error Interrupt Enable */
#define  USART_CR3_IREN                      ((uint32_t)0x00000002)            /*!< IrDA mode Enable */
#define  USART_CR3_IRLP                      ((uint32_t)0x00000004)            /*!< IrDA Low-Power */
#define  USART_CR3_HDSEL                     ((uint32_t)0x00000008)            /*!< Half-Duplex Selection */
#define  USART_CR3_NACK                      ((uint32_t)0x00000010)            /*!< SmartCard NACK enable */
#define  USART_CR3_SCEN                      ((uint32_t)0x00000020)            /*!< SmartCard mode enable */
#define  USART_CR3_DMAR                      ((uint32_t)0x00000040)            /*!< DMA Enable Receiver */
#define  USART_CR3_DMAT                      ((uint32_t)0x00000080)            /*!< DMA Enable Transmitter */
#define  USART_CR3_RTSE                      ((uint32_t)0x00000100)            /*!< RTS Enable */
#define  USART_CR3_CTSE                      ((uint32_t)0x00000200)            /*!< CTS Enable */
#define  USART_CR3_CTSIE                     ((uint32_t)0x00000400)            /*!< CTS Interrupt Enable */
#define  USART_CR3_ONEBIT                    ((uint32_t)0x00000800)            /*!< One sample bit method enable */
#define  USART_CR3_OVRDIS                    ((uint32_t)0x00001000)            /*!< Overrun Disable */
#define  USART_CR3_DDRE                      ((uint32_t)0x00002000)            /*!< DMA Disable on Reception Error */
#define  USART_CR3_DEM                       ((uint32_t)0x00004000)            /*!< Driver Enable Mode */
#define  USART_CR3_DEP                       ((uint32_t)0x00008000)            /*!< Driver Enable Polarity Selection */
#define  USART_CR3_SCARCNT                   ((uint32_t)0x000E0000)            /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
#define  USART_CR3_SCARCNT_0                 ((uint32_t)0x00020000)            /*!< Bit 0 */
#define  USART_CR3_SCARCNT_1                 ((uint32_t)0x00040000)            /*!< Bit 1 */
#define  USART_CR3_SCARCNT_2                 ((uint32_t)0x00080000)            /*!< Bit 2 */
#define  USART_CR3_WUS                       ((uint32_t)0x00300000)            /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
#define  USART_CR3_WUS_0                     ((uint32_t)0x00100000)            /*!< Bit 0 */
#define  USART_CR3_WUS_1                     ((uint32_t)0x00200000)            /*!< Bit 1 */
#define  USART_CR3_WUFIE                     ((uint32_t)0x00400000)            /*!< Wake Up Interrupt Enable */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_FRACTION              ((uint32_t)0x0000000F)            /*!< Fraction of USARTDIV */
#define  USART_BRR_DIV_MANTISSA              ((uint32_t)0x0000FFF0)            /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((uint32_t)0x000000FF)            /*!< PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_GT                       ((uint32_t)0x0000FF00)            /*!< GT[7:0] bits (Guard time value) */


/*******************  Bit definition for USART_RTOR register  *****************/
#define  USART_RTOR_RTO                      ((uint32_t)0x00FFFFFF)            /*!< Receiver Time Out Value */
#define  USART_RTOR_BLEN                     ((uint32_t)0xFF000000)            /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define  USART_RQR_ABRRQ                     ((uint32_t)0x00000001)            /*!< Auto-Baud Rate Request */
#define  USART_RQR_SBKRQ                     ((uint32_t)0x00000002)            /*!< Send Break Request */
#define  USART_RQR_MMRQ                      ((uint32_t)0x00000004)            /*!< Mute Mode Request */
#define  USART_RQR_RXFRQ                     ((uint32_t)0x00000008)            /*!< Receive Data flush Request */
#define  USART_RQR_TXFRQ                     ((uint32_t)0x00000010)            /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define  USART_ISR_PE                        ((uint32_t)0x00000001)            /*!< Parity Error */
#define  USART_ISR_FE                        ((uint32_t)0x00000002)            /*!< Framing Error */
#define  USART_ISR_NE                        ((uint32_t)0x00000004)            /*!< Noise detected Flag */
#define  USART_ISR_ORE                       ((uint32_t)0x00000008)            /*!< OverRun Error */
#define  USART_ISR_IDLE                      ((uint32_t)0x00000010)            /*!< IDLE line detected */
#define  USART_ISR_RXNE                      ((uint32_t)0x00000020)            /*!< Read Data Register Not Empty */
#define  USART_ISR_TC                        ((uint32_t)0x00000040)            /*!< Transmission Complete */
#define  USART_ISR_TXE                       ((uint32_t)0x00000080)            /*!< Transmit Data Register Empty */
#define  USART_ISR_LBDF                      ((uint32_t)0x00000100)            /*!< LIN Break Detection Flag */
#define  USART_ISR_CTSIF                     ((uint32_t)0x00000200)            /*!< CTS interrupt flag */
#define  USART_ISR_CTS                       ((uint32_t)0x00000400)            /*!< CTS flag */
#define  USART_ISR_RTOF                      ((uint32_t)0x00000800)            /*!< Receiver Time Out */
#define  USART_ISR_EOBF                      ((uint32_t)0x00001000)            /*!< End Of Block Flag */
#define  USART_ISR_ABRE                      ((uint32_t)0x00004000)            /*!< Auto-Baud Rate Error */
#define  USART_ISR_ABRF                      ((uint32_t)0x00008000)            /*!< Auto-Baud Rate Flag */
#define  USART_ISR_BUSY                      ((uint32_t)0x00010000)            /*!< Busy Flag */
#define  USART_ISR_CMF                       ((uint32_t)0x00020000)            /*!< Character Match Flag */
#define  USART_ISR_SBKF                      ((uint32_t)0x00040000)            /*!< Send Break Flag */
#define  USART_ISR_RWU                       ((uint32_t)0x00080000)            /*!< Receive Wake Up from mute mode Flag */
#define  USART_ISR_WUF                       ((uint32_t)0x00100000)            /*!< Wake Up from stop mode Flag */
#define  USART_ISR_TEACK                     ((uint32_t)0x00200000)            /*!< Transmit Enable Acknowledge Flag */
#define  USART_ISR_REACK                     ((uint32_t)0x00400000)            /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define  USART_ICR_PECF                      ((uint32_t)0x00000001)            /*!< Parity Error Clear Flag */
#define  USART_ICR_FECF                      ((uint32_t)0x00000002)            /*!< Framing Error Clear Flag */
#define  USART_ICR_NCF                       ((uint32_t)0x00000004)            /*!< Noise detected Clear Flag */
#define  USART_ICR_ORECF                     ((uint32_t)0x00000008)            /*!< OverRun Error Clear Flag */
#define  USART_ICR_IDLECF                    ((uint32_t)0x00000010)            /*!< IDLE line detected Clear Flag */
#define  USART_ICR_TCCF                      ((uint32_t)0x00000040)            /*!< Transmission Complete Clear Flag */
#define  USART_ICR_LBDCF                     ((uint32_t)0x00000100)            /*!< LIN Break Detection Clear Flag */
#define  USART_ICR_CTSCF                     ((uint32_t)0x00000200)            /*!< CTS Interrupt Clear Flag */
#define  USART_ICR_RTOCF                     ((uint32_t)0x00000800)            /*!< Receiver Time Out Clear Flag */
#define  USART_ICR_EOBCF                     ((uint32_t)0x00001000)            /*!< End Of Block Clear Flag */
#define  USART_ICR_CMCF                      ((uint32_t)0x00020000)            /*!< Character Match Clear Flag */
#define  USART_ICR_WUCF                      ((uint32_t)0x00100000)            /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define  USART_RDR_RDR                       ((uint32_t)0x000001FF)            /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define  USART_TDR_TDR                       ((uint32_t)0x000001FF)            /*!< TDR[8:0] bits (Transmit Data value) */

/******************************************************************************/
/*                                                                            */
/*                         USB Device General registers                       */
/*                                                                            */
/******************************************************************************/
#define USB_CNTR                             (USB_BASE + 0x40)             /*!< Control register */
#define USB_ISTR                             (USB_BASE + 0x44)             /*!< Interrupt status register */
#define USB_FNR                              (USB_BASE + 0x48)             /*!< Frame number register */
#define USB_DADDR                            (USB_BASE + 0x4C)             /*!< Device address register */
#define USB_BTABLE                           (USB_BASE + 0x50)             /*!< Buffer Table address register */

/****************************  ISTR interrupt events  *************************/
#define USB_ISTR_CTR                         ((uint16_t)0x8000)             /*!< Correct TRansfer (clear-only bit) */
#define USB_ISTR_PMAOVR                      ((uint16_t)0x4000)             /*!< DMA OVeR/underrun (clear-only bit) */
#define USB_ISTR_ERR                         ((uint16_t)0x2000)             /*!< ERRor (clear-only bit) */
#define USB_ISTR_WKUP                        ((uint16_t)0x1000)             /*!< WaKe UP (clear-only bit) */
#define USB_ISTR_SUSP                        ((uint16_t)0x0800)             /*!< SUSPend (clear-only bit) */
#define USB_ISTR_RESET                       ((uint16_t)0x0400)             /*!< RESET (clear-only bit) */
#define USB_ISTR_SOF                         ((uint16_t)0x0200)             /*!< Start Of Frame (clear-only bit) */
#define USB_ISTR_ESOF                        ((uint16_t)0x0100)             /*!< Expected Start Of Frame (clear-only bit) */
#define USB_ISTR_DIR                         ((uint16_t)0x0010)             /*!< DIRection of transaction (read-only bit)  */
#define USB_ISTR_EP_ID                       ((uint16_t)0x000F)             /*!< EndPoint IDentifier (read-only bit)  */

#define USB_CLR_CTR                          (~USB_ISTR_CTR)             /*!< clear Correct TRansfer bit */
#define USB_CLR_PMAOVR                       (~USB_ISTR_PMAOVR)          /*!< clear DMA OVeR/underrun bit*/
#define USB_CLR_ERR                          (~USB_ISTR_ERR)             /*!< clear ERRor bit */
#define USB_CLR_WKUP                         (~USB_ISTR_WKUP)            /*!< clear WaKe UP bit */
#define USB_CLR_SUSP                         (~USB_ISTR_SUSP)            /*!< clear SUSPend bit */
#define USB_CLR_RESET                        (~USB_ISTR_RESET)           /*!< clear RESET bit */
#define USB_CLR_SOF                          (~USB_ISTR_SOF)             /*!< clear Start Of Frame bit */
#define USB_CLR_ESOF                         (~USB_ISTR_ESOF)            /*!< clear Expected Start Of Frame bit */

/*************************  CNTR control register bits definitions  ***********/
#define USB_CNTR_CTRM                        ((uint16_t)0x8000)             /*!< Correct TRansfer Mask */
#define USB_CNTR_PMAOVR                      ((uint16_t)0x4000)             /*!< DMA OVeR/underrun Mask */
#define USB_CNTR_ERRM                        ((uint16_t)0x2000)             /*!< ERRor Mask */
#define USB_CNTR_WKUPM                       ((uint16_t)0x1000)             /*!< WaKe UP Mask */
#define USB_CNTR_SUSPM                       ((uint16_t)0x0800)             /*!< SUSPend Mask */
#define USB_CNTR_RESETM                      ((uint16_t)0x0400)             /*!< RESET Mask   */
#define USB_CNTR_SOFM                        ((uint16_t)0x0200)             /*!< Start Of Frame Mask */
#define USB_CNTR_ESOFM                       ((uint16_t)0x0100)             /*!< Expected Start Of Frame Mask */
#define USB_CNTR_RESUME                      ((uint16_t)0x0010)             /*!< RESUME request */
#define USB_CNTR_FSUSP                       ((uint16_t)0x0008)             /*!< Force SUSPend */
#define USB_CNTR_LPMODE                      ((uint16_t)0x0004)             /*!< Low-power MODE */
#define USB_CNTR_PDWN                        ((uint16_t)0x0002)             /*!< Power DoWN */
#define USB_CNTR_FRES                        ((uint16_t)0x0001)             /*!< Force USB RESet */

/********************  FNR Frame Number Register bit definitions   ************/
#define USB_FNR_RXDP                         ((uint16_t)0x8000)             /*!< status of D+ data line */
#define USB_FNR_RXDM                         ((uint16_t)0x4000)             /*!< status of D- data line */
#define USB_FNR_LCK                          ((uint16_t)0x2000)             /*!< LoCKed */
#define USB_FNR_LSOF                         ((uint16_t)0x1800)             /*!< Lost SOF */
#define USB_FNR_FN                           ((uint16_t)0x07FF)             /*!< Frame Number */

/********************  DADDR Device ADDRess bit definitions    ****************/
#define USB_DADDR_EF                         ((uint8_t)0x80)                /*!< USB device address Enable Function */
#define USB_DADDR_ADD                        ((uint8_t)0x7F)                /*!< USB device address */

/******************************  Endpoint register    *************************/
#define USB_EP0R                             USB_BASE                    /*!< endpoint 0 register address */
#define USB_EP1R                             (USB_BASE + 0x04)           /*!< endpoint 1 register address */
#define USB_EP2R                             (USB_BASE + 0x08)           /*!< endpoint 2 register address */
#define USB_EP3R                             (USB_BASE + 0x0C)           /*!< endpoint 3 register address */
#define USB_EP4R                             (USB_BASE + 0x10)           /*!< endpoint 4 register address */
#define USB_EP5R                             (USB_BASE + 0x14)           /*!< endpoint 5 register address */
#define USB_EP6R                             (USB_BASE + 0x18)           /*!< endpoint 6 register address */
#define USB_EP7R                             (USB_BASE + 0x1C)           /*!< endpoint 7 register address */
/* bit positions */ 
#define USB_EP_CTR_RX                        ((uint16_t)0x8000)             /*!<  EndPoint Correct TRansfer RX */
#define USB_EP_DTOG_RX                       ((uint16_t)0x4000)             /*!<  EndPoint Data TOGGLE RX */
#define USB_EPRX_STAT                        ((uint16_t)0x3000)             /*!<  EndPoint RX STATus bit field */
#define USB_EP_SETUP                         ((uint16_t)0x0800)             /*!<  EndPoint SETUP */
#define USB_EP_T_FIELD                       ((uint16_t)0x0600)             /*!<  EndPoint TYPE */
#define USB_EP_KIND                          ((uint16_t)0x0100)             /*!<  EndPoint KIND */
#define USB_EP_CTR_TX                        ((uint16_t)0x0080)             /*!<  EndPoint Correct TRansfer TX */
#define USB_EP_DTOG_TX                       ((uint16_t)0x0040)             /*!<  EndPoint Data TOGGLE TX */
#define USB_EPTX_STAT                        ((uint16_t)0x0030)             /*!<  EndPoint TX STATus bit field */
#define USB_EPADDR_FIELD                     ((uint16_t)0x000F)             /*!<  EndPoint ADDRess FIELD */

/* EndPoint REGister MASK (no toggle fields) */
#define USB_EPREG_MASK     (USB_EP_CTR_RX|USB_EP_SETUP|USB_EP_T_FIELD|USB_EP_KIND|USB_EP_CTR_TX|USB_EPADDR_FIELD)
                                                                               /*!< EP_TYPE[1:0] EndPoint TYPE */
#define USB_EP_TYPE_MASK                     ((uint16_t)0x0600)             /*!< EndPoint TYPE Mask */
#define USB_EP_BULK                          ((uint16_t)0x0000)             /*!< EndPoint BULK */
#define USB_EP_CONTROL                       ((uint16_t)0x0200)             /*!< EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS                   ((uint16_t)0x0400)             /*!< EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT                     ((uint16_t)0x0600)             /*!< EndPoint INTERRUPT */
#define USB_EP_T_MASK      (~USB_EP_T_FIELD & USB_EPREG_MASK)
                                                                 
#define USB_EPKIND_MASK    (~USB_EP_KIND & USB_EPREG_MASK)            /*!< EP_KIND EndPoint KIND */
                                                                               /*!< STAT_TX[1:0] STATus for TX transfer */
#define USB_EP_TX_DIS                        ((uint16_t)0x0000)             /*!< EndPoint TX DISabled */
#define USB_EP_TX_STALL                      ((uint16_t)0x0010)             /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK                        ((uint16_t)0x0020)             /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID                      ((uint16_t)0x0030)             /*!< EndPoint TX VALID */
#define USB_EPTX_DTOG1                       ((uint16_t)0x0010)             /*!< EndPoint TX Data TOGgle bit1 */
#define USB_EPTX_DTOG2                       ((uint16_t)0x0020)             /*!< EndPoint TX Data TOGgle bit2 */
#define USB_EPTX_DTOGMASK  (USB_EPTX_STAT|USB_EPREG_MASK)
                                                                               /*!< STAT_RX[1:0] STATus for RX transfer */
#define USB_EP_RX_DIS                        ((uint16_t)0x0000)             /*!< EndPoint RX DISabled */
#define USB_EP_RX_STALL                      ((uint16_t)0x1000)             /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK                        ((uint16_t)0x2000)             /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID                      ((uint16_t)0x3000)             /*!< EndPoint RX VALID */
#define USB_EPRX_DTOG1                       ((uint16_t)0x1000)             /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOG2                       ((uint16_t)0x2000)             /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOGMASK  (USB_EPRX_STAT|USB_EPREG_MASK)

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((uint32_t)0x0000007F)        /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  WWDG_CR_T1                          ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  WWDG_CR_T2                          ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  WWDG_CR_T3                          ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  WWDG_CR_T4                          ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  WWDG_CR_T5                          ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  WWDG_CR_T6                          ((uint32_t)0x00000040)        /*!<Bit 6 */

#define  WWDG_CR_WDGA                        ((uint32_t)0x00000080)        /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((uint32_t)0x0000007F)        /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         ((uint32_t)0x00000001)        /*!<Bit 0 */
#define  WWDG_CFR_W1                         ((uint32_t)0x00000002)        /*!<Bit 1 */
#define  WWDG_CFR_W2                         ((uint32_t)0x00000004)        /*!<Bit 2 */
#define  WWDG_CFR_W3                         ((uint32_t)0x00000008)        /*!<Bit 3 */
#define  WWDG_CFR_W4                         ((uint32_t)0x00000010)        /*!<Bit 4 */
#define  WWDG_CFR_W5                         ((uint32_t)0x00000020)        /*!<Bit 5 */
#define  WWDG_CFR_W6                         ((uint32_t)0x00000040)        /*!<Bit 6 */

#define  WWDG_CFR_WDGTB                      ((uint32_t)0x00000180)        /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     ((uint32_t)0x00000080)        /*!<Bit 0 */
#define  WWDG_CFR_WDGTB1                     ((uint32_t)0x00000100)        /*!<Bit 1 */

#define  WWDG_CFR_EWI                        ((uint32_t)0x00000200)        /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((uint32_t)0x00000001)        /*!<Early Wakeup Interrupt Flag */

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
#define IS_ADC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == ADC1) || \
                                       ((INSTANCE) == ADC2))
                                       
#define IS_ADC_MULTIMODE_MASTER_INSTANCE(INSTANCE) (((INSTANCE) == ADC1))

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC1_2_COMMON)
/****************************** CAN Instances *********************************/
#define IS_CAN_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CAN)

/****************************** COMP Instances ********************************/
#define IS_COMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == COMP1) || \
                                        ((INSTANCE) == COMP2) || \
                                        ((INSTANCE) == COMP4) || \
                                        ((INSTANCE) == COMP6))

/******************** COMP Instances with switch on DAC1 Channel1 output ******/
#define IS_COMP_DAC1SWITCH_INSTANCE(INSTANCE) ((INSTANCE) == COMP1)

/******************** COMP Instances with window mode capability **************/
#define IS_COMP_WINDOWMODE_INSTANCE(INSTANCE) (((INSTANCE) == COMP2) || \
                                               ((INSTANCE) == COMP4) || \
                                               ((INSTANCE) == COMP6))

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
                                       ((INSTANCE) == DMA1_Channel7) || \
                                       ((INSTANCE) == DMA2_Channel1) || \
                                       ((INSTANCE) == DMA2_Channel2) || \
                                       ((INSTANCE) == DMA2_Channel3) || \
                                       ((INSTANCE) == DMA2_Channel4) || \
                                       ((INSTANCE) == DMA2_Channel5))

/****************************** GPIO Instances ********************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE)  (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOC) || \
                                         ((INSTANCE) == GPIOD) || \
                                         ((INSTANCE) == GPIOE) || \
                                         ((INSTANCE) == GPIOF))

#define IS_GPIO_AF_INSTANCE(INSTANCE)   (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOC) || \
                                         ((INSTANCE) == GPIOD) || \
                                         ((INSTANCE) == GPIOE) || \
                                         ((INSTANCE) == GPIOF))

#define IS_GPIO_LOCK_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                         ((INSTANCE) == GPIOB) || \
                                         ((INSTANCE) == GPIOD))

/****************************** I2C Instances *********************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2))

/****************************** I2S Instances *********************************/
#define IS_I2S_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/****************************** OPAMP Instances *******************************/
#define IS_OPAMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == OPAMP1) || \
                                         ((INSTANCE) == OPAMP2))

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/****************************** SMBUS Instances *******************************/
#define IS_SMBUS_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                         ((INSTANCE) == I2C2))

/****************************** SPI Instances *********************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/******************* TIM Instances : All supported instances ******************/
#define IS_TIM_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/******************* TIM Instances : at least 1 capture/compare channel *******/
#define IS_TIM_CC1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : at least 2 capture/compare channels *******/
#define IS_TIM_CC2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : at least 3 capture/compare channels *******/
#define IS_TIM_CC3_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4))

/****************** TIM Instances : at least 4 capture/compare channels *******/
#define IS_TIM_CC4_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4))

/****************** TIM Instances : at least 5 capture/compare channels *******/
#define IS_TIM_CC5_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/****************** TIM Instances : at least 6 capture/compare channels *******/
#define IS_TIM_CC6_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)
    
/************************** TIM Instances : Advanced-control timers ***********/

/****************** TIM Instances : supporting clock selection ****************/
#define IS_TIM_CLOCK_SELECT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting external clock mode 1 for ETRF input */
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4))

/****************** TIM Instances : supporting external clock mode 2 **********/
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4))

/****************** TIM Instances : supporting external clock mode 1 for TIX inputs*/
#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting internal trigger inputs(ITRX) *******/
#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting OCxREF clear *******************/
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4))

/****************** TIM Instances : supporting encoder interface **************/
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4))

/****************** TIM Instances : supporting Hall interface *****************/
#define IS_TIM_HALL_INTERFACE_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting input XOR function *************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting master mode ********************/
#define IS_TIM_MASTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting slave mode *********************/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting synchronization ****************/
#define IS_TIM_SYNCHRO_INSTANCE(INSTANCE)\
    (((INSTANCE) == TIM1)    || \
     ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
     ((INSTANCE) == TIM6)    || \
     ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting 32 bits counter ****************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)\
    ((INSTANCE) == TIM2)

/****************** TIM Instances : supporting DMA burst **********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE)\
    (((INSTANCE) == TIM1)    || \
     ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
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
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4))

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
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting 2 break inputs *****************/
#define IS_TIM_BKIN2_INSTANCE(INSTANCE)\
  ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting ADC triggering through TRGO2 ***/
#define IS_TIM_TRGO2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1))

/****************** TIM Instances : supporting DMA generation on Update events*/
#define IS_TIM_DMA_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM15)   || \
   ((INSTANCE) == TIM16)   || \
   ((INSTANCE) == TIM17))

/****************** TIM Instances : supporting DMA generation on Capture/Compare events */
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
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
  (((INSTANCE) == TIM1)    || \
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
#define IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                            ((INSTANCE) == USART2) || \
                                                            ((INSTANCE) == USART3))

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                      ((INSTANCE) == USART2) || \
                                      ((INSTANCE) == USART3) || \
                                      ((INSTANCE) == UART4)  || \
                                      ((INSTANCE) == UART5))
                                      
/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                 ((INSTANCE) == USART2) || \
                                                 ((INSTANCE) == USART3) || \
                                                 ((INSTANCE) == UART4)  || \
                                                 ((INSTANCE) == UART5))                                       
                                      
/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                          ((INSTANCE) == USART2) || \
                                          ((INSTANCE) == USART3) || \
                                          ((INSTANCE) == UART4)  || \
                                          ((INSTANCE) == UART5))
                                          
/******************** UART Instances : Wake-up from Stop mode **********************/
#define IS_UART_WAKEUP_FROMSTOP_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                      ((INSTANCE) == USART2) || \
                                                      ((INSTANCE) == USART3) || \
                                                      ((INSTANCE) == UART4)  || \
                                                      ((INSTANCE) == UART5))

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART3))

/****************** UART Instances : Auto Baud Rate detection *****************/
#define IS_UART_AUTOBAUDRATE_DETECTION_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                           ((INSTANCE) == USART2) || \
                                                           ((INSTANCE) == USART3))

/****************** UART Instances : Driver Enable ****************************/
#define IS_UART_DRIVER_ENABLE_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                  ((INSTANCE) == USART2) || \
                                                  ((INSTANCE) == USART3))

/********************* UART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2) || \
                                         ((INSTANCE) == USART3))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3) || \
                                    ((INSTANCE) == UART4)  || \
                                    ((INSTANCE) == UART5))

/******************** UART Instances : Support of continuous communication using DMA ****/
#define IS_UART_DMA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                        ((INSTANCE) == USART2) || \
                                        ((INSTANCE) == USART3) || \
                                        ((INSTANCE) == UART4))

/****************************** USB Instances *********************************/
#define IS_USB_ALL_INSTANCE(INSTANCE) ((INSTANCE) == USB)

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
#define ADC1_IRQn       ADC1_2_IRQn
#define COMP1_2_3_IRQn  COMP1_2_IRQn
#define COMP2_IRQn      COMP1_2_IRQn
#define COMP_IRQn       COMP1_2_IRQn
#define COMP4_5_6_IRQn  COMP4_6_IRQn
#define TIM15_IRQn      TIM1_BRK_TIM15_IRQn
#define TIM18_DAC2_IRQn TIM1_CC_IRQn
#define TIM17_IRQn      TIM1_TRG_COM_TIM17_IRQn
#define TIM16_IRQn      TIM1_UP_TIM16_IRQn
#define TIM6_DAC1_IRQn  TIM6_DAC_IRQn
#define CEC_IRQn        USBWakeUp_IRQn
#define USBWakeUp_IRQn  USBWakeUp_RMP_IRQn
#define CAN_TX_IRQn     USB_HP_CAN_TX_IRQn
#define CAN_RX0_IRQn    USB_LP_CAN_RX0_IRQn


/* Aliases for __IRQHandler */
#define ADC1_IRQHandler       ADC1_2_IRQHandler
#define COMP1_2_3_IRQHandler  COMP1_2_IRQHandler
#define COMP2_IRQHandler      COMP1_2_IRQHandler
#define COMP_IRQHandler       COMP1_2_IRQHandler
#define COMP4_5_6_IRQHandler  COMP4_6_IRQHandler
#define TIM15_IRQHandler      TIM1_BRK_TIM15_IRQHandler
#define TIM18_DAC2_IRQHandler TIM1_CC_IRQHandler
#define TIM17_IRQHandler      TIM1_TRG_COM_TIM17_IRQHandler
#define TIM16_IRQHandler      TIM1_UP_TIM16_IRQHandler
#define TIM6_DAC1_IRQHandler  TIM6_DAC_IRQHandler
#define CEC_IRQHandler        USBWakeUp_IRQHandler
#define USBWakeUp_IRQHandler  USBWakeUp_RMP_IRQHandler
#define CAN_TX_IRQHandler     USB_HP_CAN_TX_IRQHandler
#define CAN_RX0_IRQHandler    USB_LP_CAN_RX0_IRQHandler


/** @defgroup ADC_Internal_Channels ADC Internal Channels
  * @{
  */

/* Note: Vopamp1, TempSensor and Vbat internal channels available on ADC1 only */
#define ADC_CHANNEL_VOPAMP1     15
#define ADC_CHANNEL_TEMPSENSOR  16
#define ADC_CHANNEL_VBAT        17

/* Note: Vopamp2/3/4 internal channels available on ADC2/3/4 respectively     */
#define ADC_CHANNEL_VOPAMP2     17
#define ADC_CHANNEL_VOPAMP3     17
#define ADC_CHANNEL_VOPAMP4     17

/* Note: VrefInt internal channels available on all ADCs, but only            */
/*       one ADC is allowed to be connected to VrefInt at the same time.      */
#define ADC_CHANNEL_VREFINT     18

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

#define ADC_TEMPSENSOR       ((ADC_TempSensorCalibrationTypeDef *)((uint32_t)0x1FFFF7B8))

#define ADC_VREFINT_CAL      (*((__I uint16_t *)((uint32_t)0x1FFFF7BA)))

/**
  * @}
  */

/** @defgroup Unique_Device_ID Unique Device ID
  * @brief    Use the macro as it was defined as: uint32_t DEVICE_ID_REG[3]
  * @{
  */

#define DEVICE_ID_REG        ((__I uint32_t *)((uint32_t)0x1FFFF7AC))

/**
  * @}
  */

/** @defgroup Device_Flash_Size Device Flash Memory Size in kB
  * @{
  */

#define DEVICE_FLASH_SIZE_KB (*((__I uint16_t *)((uint32_t)0x1FFFF7CC)))

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
#define GPIO_TIM3_AF2          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_TIM4_AF2          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_TIM15_AF2         ((uint8_t)0x02)  /* TIM15 Alternate Function mapping */
#define GPIO_COMP1_AF2         ((uint8_t)0x02)  /* COMP1 Alternate Function mapping */

/* AF 3 selection */
#define GPIO_TSC_AF3           ((uint8_t)0x03)  /* TSC Alternate Function mapping  */
#define GPIO_TIM15_AF3         ((uint8_t)0x03)  /* TIM15 Alternate Function mapping */

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
#define GPIO_I2S_AF5           ((uint8_t)0x05)  /* I2S Alternate Function mapping */
#define GPIO_I2S2ext_AF5       ((uint8_t)0x05)  /* I2S2ext Alternate Function mapping */
#define GPIO_IR_AF5            ((uint8_t)0x05)  /* IR Alternate Function mapping */
#define GPIO_UART4_AF5         ((uint8_t)0x05)  /* UART4 Alternate Function mapping */
#define GPIO_UART5_AF5         ((uint8_t)0x05)  /* UART5 Alternate Function mapping */

/* AF 6 selection */
#define GPIO_SPI2_AF6          ((uint8_t)0x06)  /* SPI2/I2S2 Alternate Function mapping */
#define GPIO_SPI3_AF6          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping */
#define GPIO_I2S3ext_AF6       ((uint8_t)0x06)  /* I2S3ext Alternate Function mapping */
#define GPIO_TIM1_AF6          ((uint8_t)0x06)  /* TIM1 Alternate Function mapping */
#define GPIO_IR_AF6            ((uint8_t)0x06)  /* IR Alternate Function mapping */

/* AF 7 selection */
#define GPIO_USART1_AF7        ((uint8_t)0x07)  /* USART1 Alternate Function mapping  */
#define GPIO_USART2_AF7        ((uint8_t)0x07)  /* USART2 Alternate Function mapping  */
#define GPIO_USART3_AF7        ((uint8_t)0x07)  /* USART3 Alternate Function mapping  */
#define GPIO_COMP6_AF7         ((uint8_t)0x07)  /* COMP6 Alternate Function mapping  */
#define GPIO_CAN_AF7           ((uint8_t)0x07)  /* CAN Alternate Function mapping  */

/* AF 8 selection */
#define GPIO_COMP1_AF8         ((uint8_t)0x08)  /* COMP1 Alternate Function mapping  */
#define GPIO_COMP2_AF8         ((uint8_t)0x08)  /* COMP2 Alternate Function mapping  */
#define GPIO_COMP4_AF8         ((uint8_t)0x08)  /* COMP4 Alternate Function mapping  */
#define GPIO_COMP6_AF8         ((uint8_t)0x08)  /* COMP6 Alternate Function mapping  */

/* AF 9 selection */
#define GPIO_CAN_AF9           ((uint8_t)0x09)  /* CAN Alternate Function mapping  */
#define GPIO_TIM1_AF9          ((uint8_t)0x09)  /* TIM1 Alternate Function mapping */
#define GPIO_TIM15_AF9         ((uint8_t)0x09)  /* TIM15 Alternate Function mapping */

/* AF 10 selection */
#define GPIO_TIM2_AF10         ((uint8_t)0x0A)  /* TIM2 Alternate Function mapping */
#define GPIO_TIM3_AF10         ((uint8_t)0x0A)  /* TIM3 Alternate Function mapping */
#define GPIO_TIM4_AF10         ((uint8_t)0x0A)  /* TIM4 Alternate Function mapping */
#define GPIO_TIM17_AF10        ((uint8_t)0x0A)  /* TIM17 Alternate Function mapping */

/* AF 11 selection */
#define GPIO_TIM1_AF11         ((uint8_t)0x0B)  /* TIM1 Alternate Function mapping */

/* AF 12 selection */
#define GPIO_TIM1_AF12         ((uint8_t)0x0C)  /* TIM1 Alternate Function mapping */

/* AF 14 selection */
#define GPIO_USB_AF14          ((uint8_t)0x0E)  /* USB Alternate Function mapping */

/* AF 15 selection */
#define GPIO_EVENTOUT_AF15     ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */

/**
  * @}
  */

/** @defgroup RCC_Internal_Oscillators RCC Internal Oscillators
  * @{
  */

#define HSI_VALUE 8000000  /* Value of the internal high speed oscillator in Hz */

#define LSI_VALUE 40000    /* Approximate value of the internal low speed oscillator in Hz */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32F302xC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
