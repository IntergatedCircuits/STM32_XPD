/**
  ******************************************************************************
  * @file    system_stm32f0xx.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-06-02
  * @brief   STM32 eXtensible Peripheral Drivers System initialization template
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f0xx_system
  * @{
  */

/** @addtogroup STM32F0xx_System_Private_Includes
  * @{
  */

#include "xpd_flash.h"
#include "xpd_rcc.h"
#include "xpd_utils.h"

/**
  * @}
  */

/** @addtogroup STM32F0xx_System_Private_Variables
  * @{
  */

/** @brief Global variable used to store the actual system clock frequency [Hz] */
uint32_t SystemCoreClock;

/** @brief Global variable used to store the last MCU reset reason */
RCC_ResetSourceType ResetSource;

/**
  * @}
  */

/** @addtogroup STM32F0xx_System_Private_Functions
  * @{
  */

/**
 * @brief  Setup the microcontroller system.
 *         Initialize the default HSI clock source, vector table location and the PLL configuration is reset.
 */
void SystemInit(void)
{
    /* Reset all peripherals */
    XPD_Deinit();

    /* Reset the RCC clock configuration to the default reset state */
    XPD_RCC_Deinit();

    /* initialize XPD services */
    XPD_Init();

    /* Read reset source to global variable */
    ResetSource = XPD_RCC_GetResetSource(TRUE);

    /* TODO Configure system memory options */
    XPD_FLASH_PrefetchBufferCtrl(ENABLE);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
