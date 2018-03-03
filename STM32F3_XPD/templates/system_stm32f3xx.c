/**
  ******************************************************************************
  * @file    system_stm32f3xx.c
  * @author  Benedek Kupper
  * @version 0.2
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers template
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
#include <xpd_flash.h>
#include <xpd_rcc.h>
#include <xpd_utils.h>

#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field.
                                  This value must be a multiple of 0x200. */
#endif

/** @brief Global variable used to store the actual system clock frequency [Hz] */
uint32_t SystemCoreClock;

/**
 * @brief  Setup the microcontroller system.
 *         Initialize the default HSI clock source,
 *         vector table location
 *         and the PLL configuration is reset.
 */
void SystemInit(void)
{
    /* Reset all peripherals */
    XPD_vDeinit();

    /* Reset the RCC clock configuration to the default reset state
     * Sets SystemCoreClock value */
    RCC_vDeinit();

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    /* FPU settings: if used, set CP10 and CP11 Full Access */
    SCB->CPACR.b.CP10 = 3;
    SCB->CPACR.b.CP11 = 3;
#endif
    /* Configure Interrupt vector table location */
#ifdef VECT_TAB_SRAM
    SCB->VTOR.w = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
    SCB->VTOR.w = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

    /* initialize XPD services */
    XPD_vInit();

    /* TODO Configure system memory options */
    FLASH_vPrefetchBuffer(ENABLE);
}
