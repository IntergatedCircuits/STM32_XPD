/**
  ******************************************************************************
  * @file    core_cmBitband.h
  * @author  Benedek Kupper
  * @version V1.1
  * @date    17-November-2015
  * @brief   Cortex M cores Bit-Band Memory Region Access Header File.
  *
  *          This file provides macros and inline functions to
  *          utilize bit-banding (implemented in Cortex M3 and up).
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
  /* ----------------------------------------------------------------------
 * Copyright (C) 2015 Benedek Kupper. All rights reserved.
 *
 * Date:        2015.11.17.
 * Revision:    v1.1
 *
 * Project:     STM32_XPD
 * Title:       core_cmBitband.h
 *
 * Description: This file provides macros and inline functions to
 * 				utilize bit-banding (implemented in Cortex M3 and up).
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

 *  You should have received a copy of the GNU General Public License
 *  along with STM32_XPD.  If not, see <http://www.gnu.org/licenses/>.
 * -------------------------------------------------------------------- */
#ifndef CORE_CMBITBAND_H_
#define CORE_CMBITBAND_H_

#ifdef __cplusplus
extern "C"
{
#endif

#if (__CORTEX_M >= 3)
/** \brief  Peripheral bit-band alias address generation macro

 This macro maps a given peripheral address to its bit-band alias address.
 It is intended for fixed address peripheral addresses which are known at compile-time,
 so global pointers can be initialized with it.

 \param		Address		The fixed peripheral address to remap.

 \return				The bit-band alias address.

 \note		The input address must be within the valid peripheral bit-band memory range:
 0x40000000 - 0x400FFFFF
 */
#define PERIPH_BB(Address)		((void *)(0x42000000 | (((uint32_t)(Address) & 0x000FFFFF) << 5)))

/** \brief  SRAM bit-band alias address generation function

 This function returns bit-band alias address created from a given SRAM address.
 It is intended for SRAM addresses which are only known at link-time,
 therefore it must be called in run-time.

 \param		Address		The SRAM address to remap.

 \return				The bit-band alias address.

 \note     The input address must be within one of the valid SRAM bit-band memory range:
 0x20000000 - 0x200FFFFF
 (Practically used SRAM addresses generally fall into this range.)
 */
__STATIC_INLINE uint32_t * SRAM_BB(uint32_t * Address)
{
	return ((uint32_t *)(0x22000000 | (((uint32_t) (Address) & 0x000FFFFF) << 5)));
}

#endif

#ifdef __cplusplus
}
#endif
#endif /* CORE_CMBITBAND_H_ */
