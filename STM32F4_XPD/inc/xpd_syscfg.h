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
#define SYSCFG_REG_BIT(_REG_NAME_, _BIT_NAME_) (SYSCFG_BB->_REG_NAME_._BIT_NAME_)
#else
#define SYSCFG_REG_BIT(_REG_NAME_, _BIT_NAME_) (SYSCFG->_REG_NAME_.b._BIT_NAME_)
#endif /* SYSCFG_BB */

#endif /* SYSCFG_REG_BIT */

