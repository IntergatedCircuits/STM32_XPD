/**
  ******************************************************************************
  * @file    xpd_syscfg.h
  * @author  Benedek Kupper
  * @version 0.1
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers System Configuration Module
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
#ifndef SYSCFG_REG_BIT

#ifdef SYSCFG_BB
#define         SYSCFG_REG_BIT(REG_NAME, BIT_NAME)          \
    (SYSCFG_BB->REG_NAME.BIT_NAME)
#else
#define         SYSCFG_REG_BIT(REG_NAME, BIT_NAME)          \
    (SYSCFG->REG_NAME.b.BIT_NAME)
#endif /* SYSCFG_BB */

#define __MEM_RMP_FLASH         0
#define __MEM_RMP_ROM           1
#define __MEM_RMP_SRAM          3
#define __MEM_RMP_FMC           2
#define __MEM_RMP_QUADSPI       6
/**
 * @brief Maps the selected memory also starting from 0 address
 * @param MEMORY: The system memory to map
 *          @arg FLASH   Main flash
 *          @arg ROM     System memory
 *          @arg SRAM    Embedded SRAM
 *          @arg FMC     FMC bank 1 (NOR/PSRAM 1 and 2)
 *          @arg QUADSPI QUADSPI memory
 */
#define         SYSTEM_MEMORY_REMAP(MEMORY)                 \
    (SYSCFG->MEMRMP.b.MEM_MODE = __MEM_RMP_##MEMORY)

#endif /* SYSCFG_REG_BIT */

