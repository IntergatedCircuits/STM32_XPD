/**
  ******************************************************************************
  * @file    xpd_rcc_cc.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2015-12-30
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
#ifndef XPD_RCC_CC_H_
#define XPD_RCC_CC_H_

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
#ifdef RCC_CFGR_SWS_PLLR
    PLLR   = 3, /*!< Secondary phase locked loop */
#endif
    LSI    = 4, /*!< Low speed internal oscillator */
#ifdef LSE_VALUE
    LSE    = 5, /*!< Low speed external oscillator */
#endif
    NO_OSC = -1,/*!< No oscillator available */
}RCC_OscType;

/** @brief RCC oscillator state types */
typedef enum
{
    OSC_OFF    = 0, /*!< Oscillator OFF state */
    OSC_ON     = 1, /*!< Oscillator ON state (internal/external resonator) */
    OSC_BYPASS = 3  /*!< External oscillator BYPASS state (external clock source) */
}RCC_OscStateType;

#ifdef HSE_VALUE
/** @brief HSE setup structure */
typedef struct
{
    RCC_OscStateType State; /*!< HSE state */
}RCC_HSE_InitType;
#endif

/** @brief HSI setup structure */
typedef struct
{
    uint8_t          CalibrationValue; /*!< HSI calibration value [0..31] (default is 16) */
    RCC_OscStateType State;            /*!< HSI state */
}RCC_HSI_InitType;

/** @brief PLL setup structure */
typedef struct
{
    uint8_t  M; /*!< Division factor for PLL VCO input. Permitted values: @arg 0 .. 63 */
    uint16_t N; /*!< Multiplication factor for PLL VCO input. Permitted values: @arg 50 .. 432
                     @arg (for STM32F411xE: 192 .. 432) */
    uint8_t  P; /*!< Division factor for main system clock.
                     Permitted values: @arg 2 @arg 4 @arg 6 @arg 8 */
    uint8_t  Q; /*!< Division factor for OTG FS, SDIO and RNG.
                     Permitted values: @arg 4 .. 15 */
#ifdef RCC_PLLCFGR_PLLR
    uint32_t R; /*!< PLL division factor for I2S, SAI, SYSTEM, SPDIFRX clocks.
                     Permitted values: @arg 2 .. 7 */
#endif
    RCC_OscStateType State;  /*!< PLL state */
    RCC_OscType      Source; /*!< PLL input source selection. Permitted values:
                                  @arg @ref RCC_OscType::HSI
                                  @arg @ref RCC_OscType::HSE */
}RCC_PLL_InitType;

/** @brief RCC core clock types */
typedef enum
{
    NO_CLOCK = 0, /*!< No core clock is selected */
    SYSCLK   = 2, /*!< System root clock */
    HCLK     = 1, /*!< Core, AHB bus clock */
    PCLK1    = 4, /*!< APB1 bus clock */
    PCLK2    = 8  /*!< APB2 bus clock */
}RCC_ClockType;

/** @brief RCC master clock output 1 clock source types */
typedef enum
{
    MCO1_CLOCKSOURCE_HSI = 0, /*!< HSI clock source */
#ifdef LSE_VALUE
    MCO1_CLOCKSOURCE_LSE = 1, /*!< LSE clock source */
#endif
#ifdef HSE_VALUE
    MCO1_CLOCKSOURCE_HSE = 2, /*!< HSE clock source */
#endif
    MCO1_CLOCKSOURCE_PLL = 3  /*!< PLL clock source */
}RCC_MCO1_ClockSourceType;

#ifdef RCC_CFGR_MCO2
/** @brief RCC master clock output 2 clock source types */
typedef enum
{
    MCO2_CLOCKSOURCE_SYSCLK    = 0, /*!< System clock source */
    MCO2_CLOCKSOURCE_PLLI2SCLK = 1, /*!< PLL I2S clock source */
#ifdef HSE_VALUE
    MCO2_CLOCKSOURCE_HSE       = 2, /*!< HSE clock source */
#endif
    MCO2_CLOCKSOURCE_PLL       = 3  /*!< PLL clock source */
}RCC_MCO2_ClockSourceType;
#endif

/** @brief RCC callbacks container structure */
typedef struct {
    XPD_SimpleCallbackType OscReady; /*!< Oscillator ready callback */
    XPD_SimpleCallbackType CSS;      /*!< Clock Security System callback */
} XPD_RCC_CallbacksType;
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
 *            @arg PLLI2SRDY:  Phase Locked Loop for I2S ready
 *            @arg PLLSAIRDY:  Phase Locked Loop for SAI ready
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
 *            @arg PLLI2SRDY:  Phase Locked Loop for I2S ready
 *            @arg PLLSAIRDY:  Phase Locked Loop for SAI ready
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
 *            @arg PLLI2SRDY:  Phase Locked Loop for I2S ready
 *            @arg PLLSAIRDY:  Phase Locked Loop for SAI ready
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
 *            @arg PLLI2SRDY:  Phase Locked Loop for I2S ready
 *            @arg PLLSAIRDY:  Phase Locked Loop for SAI ready
 *            @arg CSS:        Clock Security System
 */
#define             XPD_RCC_ClearFlag(FLAG_NAME)     \
    (RCC_REG_BIT(CIR,FLAG_NAME##C) = 1)

/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions
 * @{ */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Oscillators
 * @{ */
XPD_ReturnType      XPD_RCC_HSIConfig           (const RCC_HSI_InitType * Config);
XPD_ReturnType      XPD_RCC_LSIConfig           (RCC_OscStateType NewState);
XPD_ReturnType      XPD_RCC_PLLConfig           (const RCC_PLL_InitType * Config);
#ifdef HSE_VALUE
XPD_ReturnType      XPD_RCC_HSEConfig           (const RCC_HSE_InitType * Config);
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

/** @addtogroup RCC_Core_Clocks_Exported_Functions_CSS
 * @{ */
void                XPD_RCC_EnableCSS           (void);
void                XPD_RCC_DisableCSS          (void);

void                XPD_NMI_IRQHandler          (void);
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
void                XPD_RCC_MCOConfig           (uint8_t MCOx, uint8_t MCOSource, ClockDividerType MCODiv);
#ifdef RCC_CFGR_MCO1EN
void                XPD_RCC_EnableMCO           (uint8_t MCOx);
void                XPD_RCC_DisableMCO          (uint8_t MCOx);
#endif
/** @} */
/** @addtogroup RCC_Core_Clocks_Exported_Functions_Reset
 * @{ */
void                XPD_RCC_Deinit              (void);

void                XPD_RCC_ResetAHB1           (void);
void                XPD_RCC_ResetAHB2           (void);
void                XPD_RCC_ResetAHB3           (void);
void                XPD_RCC_ResetAPB1           (void);
void                XPD_RCC_ResetAPB2           (void);
/** @} */

/** @} */

/** @} */

/** @} */

#endif /* XPD_RCC_CC_H_ */
