/**
  ******************************************************************************
  * @file    xpd_utils.c
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers Utilities
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
#include <xpd_utils.h>
#include <xpd_rcc.h>
#include <xpd_core.h>

extern uint32_t SystemCoreClock;

/** @addtogroup XPD_Utils
 * @{ */

/** @defgroup XPD_Private_Functions XPD Private Functions
 * @{ */

/**
 * @brief Millisecond timer initializer utility.
 * @param ulCoreFreq_Hz: the new core frequency in Hz
 */
static void prvInitTimer(uint32_t ulCoreFreq_Hz)
{
    /* Enable SysTick and configure 1ms tick */
    (void)SysTick_eInit(SYSTICK_CLOCKSOURCE_HCLK, ulCoreFreq_Hz / 1000);
    SysTick_vStart();
}

/**
 * @brief Inserts code delay of the specified time in milliseconds.
 * @note  The milliseconds based waiting utilities shall not be used concurrently.
 *        The time of the preempted waiters do not elapse.
 * @param ulMilliseconds: the desired delay in ms
 */
static void prvDelay_ms(uint32_t ulMilliseconds)
{
    /* Initially clear flag */
    (void) SysTick->CTRL.b.COUNTFLAG;
    while (ulMilliseconds != 0)
    {
        /* COUNTFLAG returns 1 if timer counted to 0 since the last flag read */
        ulMilliseconds -= SysTick->CTRL.b.COUNTFLAG;
    }
}

/**
 * @brief Waits until the masked value read from address matches the input ulMatch, or until times out.
 * @note  The milliseconds based waiting utilities shall not be used concurrently.
 *        The time of the preempted waiters do not elapse.
 * @param pulVarAddress: the word address that needs to be monitored
 * @param ulBitSelector: a bit mask that selects which bits should be considered
 * @param ulMatch: the expected value to wait for
 * @param pulTimeout: pointer to the timeout in ms
 * @return TIMEOUT if timed out, or OK if ulMatch occurred within the deadline
 */
static XPD_ReturnType prvWaitForMatch(
        volatile uint32_t * pulVarAddress,
        uint32_t            ulBitSelector,
        uint32_t            ulMatch,
        uint32_t *          pulTimeout)
{
    XPD_ReturnType eResult = XPD_OK;

    /* Initially clear flag */
    (void) SysTick->CTRL.b.COUNTFLAG;

    while ((*pulVarAddress & ulBitSelector) != ulMatch)
    {
        if (*pulTimeout == 0)
        {
            eResult = XPD_TIMEOUT;
            break;
        }
        /* COUNTFLAG returns 1 if timer counted to 0 since the last flag read */
        *pulTimeout -= SysTick->CTRL.b.COUNTFLAG;
    }
    return eResult;
}

/**
 * @brief Waits until the masked value read from address differs from the input ulMatch, or until times out. [overrideable]
 * @note  The milliseconds based waiting utilities shall not be used concurrently.
 *        The time of the preempted waiters do not elapse.
 * @param pulVarAddress: the word address that needs to be monitored
 * @param ulBitSelector: a bit mask that selects which bits should be considered
 * @param ulMatch: the initial value that needs to differ
 * @param pulTimeout: pointer to the timeout in ms
 * @return TIMEOUT if timed out, or OK if ulMatch occurred within the deadline
 */
static XPD_ReturnType prvWaitForDiff(
        volatile uint32_t * pulVarAddress,
        uint32_t            ulBitSelector,
        uint32_t            ulMatch,
        uint32_t *          pulTimeout)
{
    XPD_ReturnType eResult = XPD_OK;

    /* Initially clear flag */
    (void) SysTick->CTRL.b.COUNTFLAG;

    while ((*pulVarAddress & ulBitSelector) == ulMatch)
    {
        if (*pulTimeout == 0)
        {
            eResult = XPD_TIMEOUT;
            break;
        }
        /* COUNTFLAG returns 1 if timer counted to 0 since the last flag read */
        *pulTimeout -= SysTick->CTRL.b.COUNTFLAG;
    }
    return eResult;
}

/** @} */

static const XPD_TimeServiceType xpd_xTimeService = {
        .Init           = prvInitTimer,
        .Block_ms       = prvDelay_ms,
        .MatchBlock_ms  = prvWaitForMatch,
        .DiffBlock_ms   = prvWaitForDiff,

}, *xpd_pxTimeService   = &xpd_xTimeService;

/** @defgroup XPD_Exported_Functions XPD Exported Functions
 * @{ */

/** @defgroup XPD_Exported_Functions_Timer XPD Timer Handling Functions
 *  @brief    XPD Utilities time service functions
 * @{
 */

/**
 * @brief  Returns the currently used time service.
 * @return Reference of the current time service
 */
const XPD_TimeServiceType* XPD_pxTimeService(void)
{
    return xpd_pxTimeService;
}

/**
 * @brief Sets the new system time service.
 * @param pxTimeService: reference to the time service implementation
 */
void XPD_vSetTimeService(const XPD_TimeServiceType* pxTimeService)
{
    xpd_pxTimeService = pxTimeService;
}

/**
 * @brief Resets the system time service to the XPD default and initializes it.
 */
void XPD_vResetTimeService(void)
{
    xpd_pxTimeService = &xpd_xTimeService;
    prvInitTimer(SystemCoreClock);
}

/**
 * @brief Inserts code delay of the specified time in microseconds.
 * @param ulMicroseconds: the desired delay in us
 */
__weak void XPD_vDelay_us(uint32_t ulMicroseconds)
{
    ulMicroseconds *= SystemCoreClock / 1000000;
    while (ulMicroseconds != 0)
    {
        ulMicroseconds--;
    }
}

/** @} */

/** @defgroup XPD_Exported_Functions_Stream XPD Data Stream Handling Functions
 *  @brief    XPD Utilities data stream handlers
 * @{
 */

/**
 * @brief Reads new register data to the stream and updates its context.
 * @param pulReg: pointer to the register to read from
 * @param pxStream: pointer to the destination stream
 */
void XPD_vReadToStream(const uint32_t * pulReg, DataStreamType * pxStream)
{
    /* Different size of data transferred */
    switch (pxStream->size)
    {
        case 1:
            *((uint8_t*) pxStream->buffer) = *((const uint8_t  *)pulReg);
            break;
        case 2:
            *((uint16_t*)pxStream->buffer) = *((const uint16_t *)pulReg);
            break;
        default:
            *((uint32_t*)pxStream->buffer) = *((const uint32_t *)pulReg);
            break;
    }
    /* Stream context update */
    pxStream->buffer += pxStream->size;
    pxStream->length--;
}

/**
 * @brief Writes a new stream data element to the register and updates the stream context.
 * @param pulReg: pointer to the register to write to
 * @param pxStream: pointer to the source stream
 */
void XPD_vWriteFromStream(uint32_t * pulReg, DataStreamType * pxStream)
{
    /* Different size of data transferred */
    switch (pxStream->size)
    {
        case 1:
            *((uint8_t  *)pulReg) = *((uint8_t*) pxStream->buffer);
            break;
        case 2:
            *((uint16_t *)pulReg) = *((uint16_t*)pxStream->buffer);
            break;
        default:
            *((uint32_t *)pulReg) = *((uint32_t*)pxStream->buffer);
            break;
    }
    /* Stream context update */
    pxStream->buffer += pxStream->size;
    pxStream->length--;
}

/** @} */

/** @defgroup XPD_Exported_Functions_Init XPD Startup and Shutdown Functions
 *  @brief    XPD Utilities startup and shutdown functions
 * @{
 */

/**
 * @brief Initializes the necessary utilities used by XPD drivers:
 *        @arg System Timer utility
 *        @arg Enable PWR and SYSCFG clocks
 */
void XPD_vInit(void)
{
    /* Configure systick timer */
    XPD_vInitTimer(SystemCoreClock);

    /* Enable clock for PWR */
    RCC_vClockEnable(RCC_POS_PWR);

    /* Enable SYSCFG clock  */
    RCC_vClockEnable(RCC_POS_SYSCFG);
}

/**
 * @brief Deinitializes the basic services of the device:
 *        @arg Resets all peripherals
 */
void XPD_vDeinit(void)
{
    /* Reset of all peripherals */
#if defined(AHBPERIPH_BASE) || defined(RCC_AHBRSTR_GPIOARST)
    RCC_vResetAHB();
#else
#if defined(AHB1PERIPH_BASE)
    RCC_vResetAHB1();
#endif
#if defined(AHB2PERIPH_BASE)
    RCC_vResetAHB2();
#endif
#if defined(AHB3PERIPH_BASE) || defined(RCC_AHB3RSTR_FSMCRST) || defined(RCC_AHB3RSTR_FMCRST)
    RCC_vResetAHB3();
#endif
#endif

#if defined(APBPERIPH_BASE)
    RCC_vResetAPB();
#else
#if defined(APB1PERIPH_BASE)
    RCC_vResetAPB1();
#endif
#if defined(APB2PERIPH_BASE)
    RCC_vResetAPB2();
#endif
#endif
}

/**
 * @brief Resets the MCU peripherals to their startup state and
 *        boots to the application at the specified address
 * @param pvStartAddress: The address of the application to be started
 * @note  Before starting the new application the user shall ensure
 *        the integrity of the program. This should include:
 *        @arg Checking address for valid RAM pointer
 *        @arg Checking address+4 (first PC value) for odd value
 */
void XPD_vBootTo(void * pvStartAddress)
{
    void (*pxStartApplication)(void) = *((const void **)(pvStartAddress + 4));

    /* Reset clock configuration */
    RCC_vDeinit();
    /* Reset all peripherals */
    XPD_vDeinit();
    /* Disable SysTick interrupt as well */
    SysTick_vStop_IT();

    /* Set the main stack pointer */
    __set_MSP(*((const uint32_t *)pvStartAddress));

    /* Jump to application */
    pxStartApplication();
}

/** @} */

/** @} */

/** @} */
