/**
  ******************************************************************************
  * @file    xpd_syscfg.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-02-07
  * @brief   STM32 eXtensible Peripheral Drivers System Configuration Module
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
#ifndef SYSCFG_REG_BIT

#ifdef SYSCFG_BB
#define SYSCFG_REG_BIT(_REG_NAME_, _BIT_NAME_) (SYSCFG_BB->_REG_NAME_._BIT_NAME_)
#else
#define SYSCFG_REG_BIT(_REG_NAME_, _BIT_NAME_) (SYSCFG->_REG_NAME_.b._BIT_NAME_)
#endif /* SYSCFG_BB */

#endif /* SYSCFG_REG_BIT */

