/**
  ******************************************************************************
  * @file    xpd_config.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-01
  * @brief   STM32 eXtensible Peripheral Drivers Configuration Header
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
#ifndef XPD_CONFIG_H_
#define XPD_CONFIG_H_

/* TODO step 1: specify device header */
#include "stm32f303xc.h"

/* TODO step 2: specify startup configuration */
#define NVIC_PRIOGROUP_SELECT       NVIC_PRIOGROUP_0PRE_4SUB
#define PREFETCH_ENABLE             1
#define VDD_VALUE                   3300 /* Value of VDD in mV */

/* TODO step 3: specify used XPD modules */
#define USE_XPD_ADC
#define USE_XPD_SPI
#define USE_XPD_TIM


/* TODO step 4: specify oscillator parameters */
#ifndef HSI_STARTUP_TIMEOUT
#define HSI_STARTUP_TIMEOUT 5000 /* Timeout for HSI startup in ms */
#endif


/* TODO step 5: specify vector table location */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x0 /* Vector Table base offset field. This value must be a multiple of 0x200. */

#endif /* XPD_CONFIG_H_ */
