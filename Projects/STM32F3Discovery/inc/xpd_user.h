/**
  ******************************************************************************
  * @file    xpd.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-18
  * @brief   STM32 eXtensible Peripheral Drivers Main Header
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
#ifndef XPD_USER_H_
#define XPD_USER_H_

#include "xpd_adc.h"
#include "xpd_core.h"
#include "xpd_dma.h"
#include "xpd_exti.h"
#include "xpd_flash.h"
#include "xpd_gpio.h"
#include "xpd_pwr.h"
#include "xpd_rcc.h"
#include "xpd_spi.h"
#include "xpd_utils.h"

extern void ClockConfiguration(void);

#endif /* XPD_USER_H_ */
