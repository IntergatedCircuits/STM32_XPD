/**
  ******************************************************************************
  * @file    xpd_flash.h
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
#ifndef XPD_FLASH_H_
#define XPD_FLASH_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @defgroup FLASH
 * @{ */

#ifdef FLASH_BB
#define FLASH_REG_BIT(REG_NAME, BIT_NAME) (FLASH_BB->REG_NAME.BIT_NAME)
#else
#define FLASH_REG_BIT(REG_NAME, BIT_NAME) (FLASH->REG_NAME.b.BIT_NAME)
#endif

/** @defgroup FLASH_Exported_Functions FLASH Exported Functions
 * @{ */

/**
 * @brief Sets the flash memory access latency (in clock cycles).
 * @param Latency: the flash access latency [0 .. 15]
 */
__STATIC_INLINE void XPD_FLASH_SetLatency(uint8_t Latency)
{
    FLASH->ACR.b.LATENCY = Latency;
}

/**
 * @brief Gets the flash memory access latency (in clock cycles).
 * @return The flash access latency
 */
__STATIC_INLINE uint8_t XPD_FLASH_GetLatency(void)
{
    return FLASH->ACR.b.LATENCY;
}

#ifdef FLASH_ACR_PRFTEN
/**
 * @brief Sets the new state for the flash prefetch buffer.
 * @param NewState: the new state
 */
__STATIC_INLINE void XPD_FLASH_PrefetchBufferCtrl(FunctionalState NewState)
{
    FLASH_REG_BIT(ACR,PRFTEN) = NewState;
}
#endif

#ifdef FLASH_ACR_ICEN
/**
 * @brief Sets the new state for the instruction cache.
 * @param NewState: the new state
 */
__STATIC_INLINE void XPD_FLASH_InstCacheCtrl(FunctionalState NewState)
{
    FLASH_REG_BIT(ACR,ICEN) = NewState;
}
#endif

#ifdef FLASH_ACR_DCEN
/**
 * @brief Sets the new state for the data cache.
 * @param NewState: the new state
 */
__STATIC_INLINE void XPD_FLASH_DataCacheCtrl(FunctionalState NewState)
{
    FLASH_REG_BIT(ACR,DCEN) = NewState;
}
#endif

/** @} */

/** @} */

#endif /* XPD_FLASH_H_ */
