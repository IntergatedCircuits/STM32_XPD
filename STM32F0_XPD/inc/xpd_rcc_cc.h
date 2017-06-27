/**
  ******************************************************************************
  * @file    xpd_rcc_cc.h
  * @author  Benedek Kupper
  * @version V0.2
  * @date    2017-05-09
  * @brief   STM32 eXtensible Peripheral Drivers RCC Core Clocks Module
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
#ifndef __XPD_RCC_CC_H_
#define __XPD_RCC_CC_H_

#include "xpd_common.h"
#include "xpd_config.h"

/** @addtogroup RCC
 * @{ */

/** @defgroup RCC_Core_Clocks RCC Core
 * @{ */

/** @defgroup RCC_Core_Exported_Types RCC Core Exported Types
 * @{ */

/** @brief RCC oscillator types */
typedef enum
{
    HSI    = 0, /*!< High speed internal oscillator */
#ifdef HSE_VALUE
    HSE    = 1, /*!< High speed external oscillator */
#endif
    PLL    = 2, /*!< Phase locked loop */
#ifdef RCC_HSI48_SUPPORT
    HSI48  = 3, /*!< 48 MHz internal oscillator */
#endif
    LSI    = 4, /*!< Low speed internal oscillator */
#ifdef LSE_VALUE
    LSE    = 5, /*!< Low speed external oscillator */
#endif
    HSI14  = 6, /*!< 14 MHz internal oscillator */
    NO_OSC = -1,/*!< No oscillator available */
}RCC_OscType;

/** @brief RCC oscillator state types */
typedef enum
{
    OSC_OFF    = 0, /*!< Oscillator OFF state */
    OSC_ON     = 1, /*!< Oscillator ON state (internal/external resonator) */
    OSC_BYPASS = 3  /*!< External oscillator BYPASS state (external clock source) */
}RCC_OscStateType;

/** @brief PLL setup structure */
typedef struct
{
    uint8_t          Multiplier; /*!< PLL multiplier value [2..16] */
    FunctionalState  State;      /*!< PLL state */
    RCC_OscType      Source;     /*!< PLL input source selection. Permitted values:
                                      @arg @ref RCC_OscType::HSI
                                      @arg @ref RCC_OscType::HSE */
    uint8_t          Predivider; /*!< PLL predivider value [1..16]
                                      @note In some cases PREDIV is only used for HSE */
}RCC_PLL_InitType;

/** @brief RCC core clock types */
typedef enum
{
    NO_CLOCK = 0, /*!< No core clock is selected */
    SYSCLK   = 2, /*!< System root clock */
    HCLK     = 1, /*!< Core, AHB bus clock */
    PCLK1    = 4, /*!< APB1 bus clock */
}RCC_ClockType;

/** @brief RCC master clock output 1 clock source types */
typedef enum
{
    MCO1_CLOCKSOURCE_NOCLOCK     = 0, /*!< MCO clock disabled */
    MCO1_CLOCKSOURCE_HSI14       = 1, /*!< HSI14 clock source */
    MCO1_CLOCKSOURCE_LSI         = 2, /*!< LSI clock source */
#ifdef LSE_VALUE
    MCO1_CLOCKSOURCE_LSE         = 3, /*!< LSE clock source */
#endif
    MCO1_CLOCKSOURCE_SYSCLK      = 4,
    MCO1_CLOCKSOURCE_HSI         = 5, /*!< HSI clock source */
#ifdef HSE_VALUE
    MCO1_CLOCKSOURCE_HSE         = 6, /*!< HSE clock source */
#endif
    MCO1_CLOCKSOURCE_PLLCLK_DIV2 = 7, /*!< PLL / 2 clock source */
#ifdef RCC_HSI48_SUPPORT
    MCO1_CLOCKSOURCE_HSI48       = 8, /*!< HSE clock source */
#endif
#ifdef RCC_CFGR_PLLNODIV
    MCO1_CLOCKSOURCE_PLLCLK      = 0X17, /*!< PLL clock source */
#endif
}RCC_MCO1_ClockSourceType;

/** @brief RCC reset source types */
typedef enum
{
    RESET_SOURCE_UNKNOWN      = 0x000, /*!< Reset source unknown */
    RESET_SOURCE_LOWPOWER     = 0x800, /*!< Low-power management reset occurred */
    RESET_SOURCE_WWDG         = 0x400, /*!< Window watchdog reset occurred */
    RESET_SOURCE_IWDG         = 0x200, /*!< Independent watchdog reset from VDD domain occurred */
    RESET_SOURCE_SOFTWARE     = 0x100, /*!< Software reset occurred */
    RESET_SOURCE_POWERON      = 0x080, /*!< PowerOnReset / PowerDownReset occurred */
    RESET_SOURCE_NRST         = 0x040, /*!< NRST pin triggered occurred */
#ifdef RCC_CSR_OBLRSTF
    RESET_SOURCE_OB_LOADER    = 0x020, /*!< Option byte loader reset occurred */
#endif
#ifdef RCC_CSR_V18PWRRSTF
    RESET_SOURCE_POWERON_1p8V = 0x008, /*!< 1.8 V domain reset occurred */
#endif
}RCC_ResetSourceType;

/** @brief RCC callbacks container structure */
typedef struct {
    XPD_SimpleCallbackType OscReady; /*!< Oscillator ready callback */
    XPD_SimpleCallbackType CSS;      /*!< Clock Security System callback */
#ifdef RCC_HSI48_SUPPORT
    struct {
        XPD_SimpleCallbackType SyncSuccess;  /*!< SYNC OK flag requested interrupt */
        XPD_SimpleCallbackType SyncWarning;  /*!< SYNC WARN flag requested interrupt */
        XPD_SimpleCallbackType SyncExpected; /*!< ESYNC flag requested interrupt */
        XPD_SimpleCallbackType SyncError;    /*!< a SYNC error flag requested interrupt */
    }HSI48;
#endif
}XPD_RCC_CallbacksType;

/** @} */

/** @defgroup RCC_Core_Exported_Variables RCC Core Exported Variables
 * @{ */

/** @brief RCC callbacks container struct */
extern XPD_RCC_CallbacksType XPD_RCC_Callbacks;

/** @} */

/** @defgroup RCC_Core_Exported_Macros RCC Core Exported Macros
 * @{ */

/**
 * @brief  Enable the specified RCC interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 */
#define             XPD_RCC_EnableIT(IT_NAME)        \
    (RCC_REG_BIT(CIR,IT_NAME##IE) = 1)

/**
 * @brief  Enable the specified RCC interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 */
#define             XPD_RCC_DisableIT(IT_NAME)       \
    (RCC_REG_BIT(CIR,IT_NAME##IE) = 0)

/**
 * @brief  Get the specified RCC flag.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 *            @arg CSS:        Clock Security System
 */
#define             XPD_RCC_GetFlag(FLAG_NAME)       \
    (RCC_REG_BIT(CIR,FLAG_NAME##F))

/**
 * @brief  Clear the specified RCC flag.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 *            @arg CSS:        Clock Security System
 */
#define             XPD_RCC_ClearFlag(FLAG_NAME)     \
    (RCC_REG_BIT(CIR,FLAG_NAME##C) = 1)

/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions
 * @{ */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Oscillators
 * @{ */
XPD_ReturnType      XPD_RCC_HSIConfig           (FunctionalState NewState);
#ifdef RCC_HSI48_SUPPORT
XPD_ReturnType      XPD_RCC_HSI48Config         (FunctionalState NewState);
#endif
XPD_ReturnType      XPD_RCC_LSIConfig           (FunctionalState NewState);
XPD_ReturnType      XPD_RCC_PLLConfig           (const RCC_PLL_InitType * Config);
#ifdef HSE_VALUE
XPD_ReturnType      XPD_RCC_HSEConfig           (RCC_OscStateType NewState);
#endif
#ifdef LSE_VALUE
XPD_ReturnType      XPD_RCC_LSEConfig           (RCC_OscStateType NewState);
#endif

uint32_t            XPD_RCC_GetOscFreq          (RCC_OscType Oscillator);

RCC_OscType         XPD_RCC_GetSYSCLKSource     (void);
RCC_OscType         XPD_RCC_GetPLLSource        (void);

void                XPD_RCC_IRQHandler          (void);
RCC_OscType         XPD_RCC_GetReadyOsc         (void);
/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Clocks
 * @{ */
XPD_ReturnType      XPD_RCC_HCLKConfig          (RCC_OscType SYSCLK_Source, ClockDividerType HCLK_Divider,
                                                 uint8_t FlashLatency);
void                XPD_RCC_PCLKConfig          (RCC_ClockType PCLKx, ClockDividerType PCLK_Divider);
uint32_t            XPD_RCC_GetClockFreq        (RCC_ClockType SelectedClock);
/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_MCO
 * @{ */
void                XPD_RCC_MCO_Init            (uint8_t MCOx, RCC_MCO1_ClockSourceType MCOSource,
                                                 ClockDividerType MCODiv);
void                XPD_RCC_MCO_Deinit          (uint8_t MCOx);
/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_CSS RCC Clock Security System
 *  @brief    RCC Clock Security System
 * @{
 */

/**
 * @brief RCC interrupt handler that provides Clock Security System callback.
 */
__STATIC_INLINE void XPD_NMI_IRQHandler(void)
{
    /* Check RCC CSSF flag  */
    if (XPD_RCC_GetFlag(CSS) != 0)
    {
        /* Clear RCC CSS pending bit */
        XPD_RCC_ClearFlag(CSS);

        /* RCC Clock Security System interrupt user callback */
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.CSS,);
    }
}

/**
 * @brief Enables or disables the Clock Security System
 * @param NewState: the CSS activation
 */
__STATIC_INLINE void XPD_RCC_CSS(FunctionalState NewState)
{
    RCC_REG_BIT(CR,CSSON) = NewState;
}

/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Reset
 * @{ */
void                XPD_RCC_Deinit              (void);

void                XPD_RCC_ResetAHB            (void);
void                XPD_RCC_ResetAPB            (void);

RCC_ResetSourceType XPD_RCC_GetResetSource      (boolean_t Destructive);
/** @} */

/** @} */

/** @} */

/** @} */

#endif /* __XPD_RCC_CC_H_ */
