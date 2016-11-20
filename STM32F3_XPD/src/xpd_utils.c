/**
  ******************************************************************************
  * @file    xpd_utils.c
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2016-01-16
  * @brief   STM32 eXtensible Peripheral Drivers Utilities Module
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
#include "xpd_utils.h"
#include "xpd_rcc.h"
#include "xpd_dbg.h"
#include "xpd_core.h"
#include "xpd_flash.h"

extern uint32_t SystemCoreClock;

static volatile uint32_t msTick = 0;

XPD_CallbacksType XPD_Callbacks = { NULL };

/** @addtogroup XPD_Utils
 * @{ */

/** @defgroup XPD_Exported_Functions XPD Exported Functions
 * @{ */

/** @defgroup XPD_Exported_Functions_Timer XPD Timer Handling Functions
 *  @brief    XPD Utilities millisecond timer handling functions
 * @{
 */

/**
 * @brief Millisecond timer incrementer utility. [overrideable]
 */
__weak void XPD_IncTimer(void)
{
    msTick++;
}

/**
 * @brief Millisecond timer reader utility. [overrideable]
 * @return System time in milliseconds
 */
__weak uint32_t XPD_GetTimer(void)
{
    return msTick;
}

/**
 * @brief Millisecond timer interrupt disabler utility. [overrideable]
 */
__weak void XPD_SuspendTimer(void)
{
    XPD_SysTick_DisableIT();
}

/**
 * @brief Millisecond timer interrupt enabler utility. [overrideable]
 */
__weak void XPD_ResumeTimer(void)
{
    XPD_SysTick_EnableIT();
}

/**
 * @brief Millisecond timer initializer utility. [overrideable]
 */
__weak void XPD_InitTimer(void)
{
    /* set the SysTick IRQ priority */
    XPD_NVIC_SetPriorityConfig(SysTick_IRQn, 0, 0);

    /* enable systick and configure 1ms tick */
    XPD_SysTick_Init(SystemCoreClock / 1000, SYSTICK_CLOCKSOURCE_HCLK);
    XPD_SysTick_Start_IT();
}

/**
 * @brief Inserts code delay of the specified time in milliseconds.
 * @param milliseconds: the desired delay in ms
 */
__weak void XPD_Delay_ms(uint32_t milliseconds)
{
    uint32_t starttime = XPD_GetTimer();
    while ((XPD_GetTimer() - starttime) < milliseconds);
}

/**
 * @brief Inserts code delay of the specified time in microseconds.
 * @param microseconds: the desired delay in us
 */
__weak void XPD_Delay_us(uint32_t microseconds)
{
    microseconds *= SystemCoreClock / 1000000;
    while (microseconds != 0)
    {
        microseconds--;
    }
}

/**
 * @brief Waits until the masked value read from address matches the input match, or until times out. [overrideable]
 * @param varAddress: the word address that needs to be monitored
 * @param bitSelector: a bit mask that selects which bits should be considered
 * @param match: the expected value to wait for
 * @param mstimeout: pointer to the timeout in ms
 * @return TIMEOUT if timed out, or OK if match occurred within the deadline
 */
__weak XPD_ReturnType XPD_WaitForMatch(volatile uint32_t * varAddress, uint32_t bitSelector, uint32_t match, uint32_t * mstimeout)
{
    XPD_ReturnType result = XPD_OK;
    uint32_t starttime = XPD_GetTimer();
    uint32_t timeout = *mstimeout;
    uint32_t acttime = starttime;

    while((*varAddress & bitSelector) != match)
    {
        if((timeout != XPD_NO_TIMEOUT) && ((XPD_GetTimer() - starttime) > timeout))
        {
            result = XPD_TIMEOUT;
            break;
        }
        if (acttime != XPD_GetTimer())
        {
            (*mstimeout)--;
            acttime = XPD_GetTimer();
        }
    }
    return result;
}

/**
 * @brief Waits until the masked value read from address differs from the input match, or until times out. [overrideable]
 * @param varAddress: the word address that needs to be monitored
 * @param bitSelector: a bit mask that selects which bits should be considered
 * @param match: the initial value that needs to differ
 * @param mstimeout: pointer to the timeout in ms
 * @return TIMEOUT if timed out, or OK if match occurred within the deadline
 */
__weak XPD_ReturnType XPD_WaitForDiff(volatile uint32_t * varAddress, uint32_t bitSelector, uint32_t match, uint32_t * mstimeout)
{
    XPD_ReturnType result = XPD_OK;
    uint32_t starttime = XPD_GetTimer();
    uint32_t timeout = *mstimeout;
    uint32_t acttime = starttime;

    while((*varAddress & bitSelector) == match)
    {
        if((timeout != XPD_NO_TIMEOUT) && ((XPD_GetTimer() - starttime) > timeout))
        {
            result = XPD_TIMEOUT;
            break;
        }
        if (acttime != XPD_GetTimer())
        {
            (*mstimeout)--;
            acttime = XPD_GetTimer();
        }
    }
    return result;
}
/** @} */

/** @defgroup XPD_Exported_Functions_IRQ XPD Interrupt Handling Functions
 *  @brief    XPD Utilities interrupt handlers
 * @{
 */

/**
 * @brief SysTick interrupt handler. Manages millisecond timer and provides callback.
 */
void XPD_SysTick_IRQHandler(void)
{
    XPD_IncTimer();

    XPD_SAFE_CALLBACK(XPD_Callbacks.Tick,);
}

/** @} */

/** @defgroup XPD_Exported_Functions_Init XPD Startup and Shutdown Functions
 *  @brief    XPD Utilities startup and shutdown functions
 * @{
 */

/**
 * @brief Initializes the basic services of the device:
 *        @arg Memory access
 *        @arg Timer utility
 *        @arg Interrupt priority group selection
 *        @arg Enable GPIO clocks
 */
void XPD_Init(void)
{
#if (INSTRUCTION_CACHE_ENABLE != 0)
    XPD_FLASH_InstCacheCtrl(ENABLE);
#endif

#if (DATA_CACHE_ENABLE != 0)
    XPD_FLASH_DataCacheCtrl(ENABLE);
#endif

#if (PREFETCH_ENABLE != 0)
    XPD_FLASH_PrefetchBufferCtrl(ENABLE);
#endif

    /* Set Interrupt Group Priority */
    XPD_NVIC_SetPriorityGroup(NVIC_PRIOGROUP_SELECT);

    /* Configure systick timer */
    XPD_InitTimer();

    /* enable clock for PWR */
    XPD_PWR_ClockCtrl(ENABLE);
}

/**
 * @brief Deinitializes the basic services of the device:
 *        @arg Timer utility
 *        @arg Resets all peripherals
 */
void XPD_Deinit(void)
{
    XPD_SuspendTimer();

    /* Reset of all peripherals */
    XPD_RCC_ResetAHB();
    XPD_RCC_ResetAPB1();
    XPD_RCC_ResetAPB2();
}

/** @} */

/** @} */

/** @} */
