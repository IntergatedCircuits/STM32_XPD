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

/** @addtogroup FLASH_Exported_Functions
 * @{ */
void        XPD_FLASH_SetLatency            (uint8_t Latency);
uint8_t     XPD_FLASH_GetLatency            (void);

void        XPD_FLASH_PrefetchBufferEnable  (void);
void        XPD_FLASH_PrefetchBufferDisable (void);

void        XPD_FLASH_InstCacheEnable       (void);
void        XPD_FLASH_InstCacheDisable      (void);

void        XPD_FLASH_DataCacheEnable       (void);
void        XPD_FLASH_DataCacheDisable      (void);
/** @} */

#ifdef FLASH_BB
#define FLASH_REG_BIT(REG_NAME, BIT_NAME) (FLASH_BB->REG_NAME.BIT_NAME)
#else
#define FLASH_REG_BIT(REG_NAME, BIT_NAME) (FLASH->REG_NAME.b.BIT_NAME)
#endif

/** @} */

#endif /* XPD_FLASH_H_ */
