/**
  ******************************************************************************
  * @file    xpd_flash.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-01
  * @brief   STM32 eXtensible Peripheral Drivers Flash Module
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
#include "xpd_flash.h"

/** @addtogroup FLASH
 * @{ */

/** @defgroup FLASH_Exported_Functions FLASH Exported Functions
 * @{ */

/**
 * @brief Sets the flash memory access latency (in clock cycles).
 * @param Latency: the flash access latency
 */
void XPD_FLASH_SetLatency(uint8_t Latency)
{
    FLASH->ACR.b.LATENCY = Latency;
}

/**
 * @brief Gets the flash memory access latency (in clock cycles).
 * @return The flash access latency
 */
uint8_t XPD_FLASH_GetLatency(void)
{
    return FLASH->ACR.b.LATENCY;
}

/**
 * @brief Enables the prefetch buffer.
 */
void XPD_FLASH_PrefetchBufferEnable(void)
{
    FLASH_REG_BIT(ACR,PRFTEN) = 1;
}

/**
 * @brief Disables the prefetch buffer.
 */
void XPD_FLASH_PrefetchBufferDisable(void)
{
    FLASH_REG_BIT(ACR,PRFTEN) = 0;
}

/**
 * @brief Enables the instruction cache.
 */
void XPD_FLASH_InstCacheEnable(void)
{
    FLASH_REG_BIT(ACR,ICEN) = 1;
}

/**
 * @brief Disables the instruction cache.
 */
void XPD_FLASH_InstCacheDisable(void)
{
    FLASH_REG_BIT(ACR,ICEN) = 0;
}

/**
 * @brief Enables the data cache.
 */
void XPD_FLASH_DataCacheEnable(void)
{
    FLASH_REG_BIT(ACR,DCEN) = 1;
}

/**
 * @brief Disables the data cache.
 */
void XPD_FLASH_DataCacheDisable(void)
{
    FLASH_REG_BIT(ACR,DCEN) = 0;
}

/** @} */

/** @} */
