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
    MSI    = 0, /*!< Multiple speed internal oscillator */
    HSI    = 1, /*!< High speed internal oscillator */
#ifdef HSE_VALUE
    HSE    = 2, /*!< High speed external oscillator */
#endif
    PLL    = 3, /*!< Phase locked loop */
    LSI    = 4, /*!< Low speed internal oscillator */
#ifdef LSE_VALUE
    LSE    = 5, /*!< Low speed external oscillator */
#endif
#ifdef RCC_HSI48_SUPPORT
    HSI48  = 6, /*!< 48 MHz internal oscillator */
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

/** @brief MSI oscillator clock range selection */
typedef enum
{
    MSI_100kHz = 0,  /*!< MSI at 100kHz */
    MSI_200kHz = 1,  /*!< MSI at 200kHz */
    MSI_400kHz = 2,  /*!< MSI at 400kHz */
    MSI_800kHz = 3,  /*!< MSI at 800kHz */
    MSI_1MHz   = 4,  /*!< MSI at 1MHz */
    MSI_2MHz   = 5,  /*!< MSI at 2MHz */
    MSI_4MHz   = 6,  /*!< MSI at 4MHz */
    MSI_8MHz   = 7,  /*!< MSI at 8MHz */
    MSI_16MHz  = 8,  /*!< MSI at 16MHz */
    MSI_24MHz  = 9,  /*!< MSI at 24MHz */
    MSI_32MHz  = 10, /*!< MSI at 32MHz */
    MSI_48MHz  = 11, /*!< MSI at 48MHz */
}RCC_MSIFreqType;

/** @brief MSI setup structure */
typedef struct
{
    RCC_MSIFreqType  ClockFreq;        /*!< MSI clock frequency */
    uint8_t          CalibrationValue; /*!< MSI calibration value [0..31] (default is 16) */
    RCC_OscStateType State;            /*!< MSI state */
}RCC_MSI_InitType;

/** @brief HSI setup structure */
typedef struct
{
    uint8_t          CalibrationValue; /*!< HSI calibration value [0..31] (default is 16) */
    RCC_OscStateType State;            /*!< HSI state */
}RCC_HSI_InitType;

/** @brief PLL setup structure */
typedef struct
{
    uint8_t  M; /*!< Common PLL predivider, only set for main PLL. Permitted values: @arg 1 .. 8 */
    uint16_t N; /*!< Multiplication factor for PLL. Permitted values: @arg 8 .. 86 */
    uint8_t  R; /*!< Division factor for main system clock.
                     Permitted values: @arg 2 @arg 4 @arg 6 @arg 8 */
    uint8_t  Q; /*!< Division factor for OTG FS.
                     Permitted values: @arg 2 @arg 4 @arg 6 @arg 8 */
    uint32_t P; /*!< PLL division factor for  SAI clocks.
                     Permitted values for advanced devices: @arg 2 .. 31
                     Permitted values otherwise: @arg 7 @arg 17 */
    RCC_OscStateType State;  /*!< PLL state */
    RCC_OscType      Source; /*!< PLL input source selection. Permitted values:
                                  @arg @ref RCC_OscType::HSI
                                  @arg @ref RCC_OscType::HSE
                                  @arg @ref RCC_OscType::MSI */
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
    MCO1_CLOCKSOURCE_NONE   = 0, /*!< no clock source */
    MCO1_CLOCKSOURCE_SYSCLK = 1, /*!< SYSCLK clock source */
    MCO1_CLOCKSOURCE_MSI    = 2, /*!< MSI clock source */
    MCO1_CLOCKSOURCE_HSI    = 3, /*!< HSI clock source */
#ifdef HSE_VALUE
    MCO1_CLOCKSOURCE_HSE    = 4, /*!< HSE clock source */
#endif
    MCO1_CLOCKSOURCE_PLL    = 5, /*!< PLL clock source */
    MCO1_CLOCKSOURCE_LSI    = 6, /*!< LSI clock source */
#ifdef LSE_VALUE
    MCO1_CLOCKSOURCE_LSE    = 7, /*!< LSE clock source */
#endif
#ifdef RCC_HSI48_SUPPORT
    MCO1_CLOCKSOURCE_HSI48  = 8, /*!< HSI48 clock source */
#endif
}RCC_MCO1_ClockSourceType;

/** @brief RCC low-speed clock output clock source types */
typedef enum
{
    LSCO_CLOCKSOURCE_NONE   = 0, /*!< no clock source */
    LSCO_CLOCKSOURCE_LSI    = 1, /*!< LSI clock source */
#ifdef LSE_VALUE
    LSCO_CLOCKSOURCE_LSE    = 3, /*!< LSE clock source */
#endif
}RCC_LSCO_ClockSourceType;

/** @brief RCC callbacks container structure */
typedef struct {
    XPD_SimpleCallbackType OscReady; /*!< Oscillator ready callback */
    XPD_SimpleCallbackType CSS;      /*!< Clock Security System callback */
    XPD_SimpleCallbackType LS_CSS;   /*!< LSE Dedicated Clock Security System callback */
} XPD_RCC_CallbacksType;
/** @} */

/** @defgroup RCC_Core_Exported_Variables RCC Core Exported Variables
 * @{ */

/** @brief RCC callbacks container struct */
extern XPD_RCC_CallbacksType XPD_RCC_Callbacks;

/** @} */

/** @defgroup RCC_Core_Exported_Macros RCC Core Exported Macros
 * @{ */

/** @brief Default HSI calibration value */
#define HSI_CALIBRATION_DEFAULT_VALUE   0x10

/** @brief Default HSI calibration value */
#define HSI48_CALIBRATION_DEFAULT_VALUE 0x10

/** @brief Default MSI calibration value */
#define MSI_CALIBRATION_DEFAULT_VALUE   0x10

/**
 * @brief  Enable the specified RCC interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg MSIRDY:     Medium Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 *            @arg PLLSAI1RDY: Phase Locked Loop for SAI1 ready
 *            @arg PLLSAI2RDY: Phase Locked Loop for SAI2 ready
 */
#define             XPD_RCC_EnableIT(IT_NAME)        \
    (RCC_REG_BIT(CIER,IT_NAME##IE) = 1)

/**
 * @brief  Enable the specified RCC interrupt.
 * @param  IT_NAME: specifies the interrupt to enable.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg MSIRDY:     Medium Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 *            @arg PLLSAI1RDY: Phase Locked Loop for SAI1 ready
 *            @arg PLLSAI2RDY: Phase Locked Loop for SAI2 ready
 */
#define             XPD_RCC_DisableIT(IT_NAME)       \
    (RCC_REG_BIT(CIER,IT_NAME##IE) = 0)

/**
 * @brief  Get the specified RCC flag.
 * @param  FLAG_NAME: specifies the flag to return.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg MSIRDY:     Medium Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 *            @arg PLLSAI1RDY: Phase Locked Loop for SAI1 ready
 *            @arg PLLSAI2RDY: Phase Locked Loop for SAI2 ready
 *            @arg CSS:        Clock Security System
 */
#define             XPD_RCC_GetFlag(FLAG_NAME)       \
    (RCC_REG_BIT(CIFR,FLAG_NAME##F))

/**
 * @brief  Clear the specified RCC flag.
 * @param  FLAG_NAME: specifies the flag to clear.
 *         This parameter can be one of the following values:
 *            @arg LSIRDY:     Low Speed Internal oscillator ready
 *            @arg LSERDY:     Low Speed Internal oscillator ready
 *            @arg MSIRDY:     Medium Speed Internal oscillator ready
 *            @arg HSIRDY:     Low Speed Internal oscillator ready
 *            @arg HSERDY:     Low Speed Internal oscillator ready
 *            @arg PLLRDY:     Phase Locked Loop ready
 *            @arg PLLSAI1RDY: Phase Locked Loop for SAI1 ready
 *            @arg PLLSAI2RDY: Phase Locked Loop for SAI2 ready
 *            @arg CSS:        Clock Security System
 */
#define             XPD_RCC_ClearFlag(FLAG_NAME)     \
    (RCC_REG_BIT(CICR,FLAG_NAME##C) = 1)

/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions
 * @{ */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Oscillators
 * @{ */
XPD_ReturnType      XPD_RCC_MSIConfig           (const RCC_MSI_InitType * Config);
XPD_ReturnType      XPD_RCC_HSIConfig           (const RCC_HSI_InitType * Config);
XPD_ReturnType      XPD_RCC_LSIConfig           (RCC_OscStateType NewState);
XPD_ReturnType      XPD_RCC_PLLConfig           (const RCC_PLL_InitType * Config);

XPD_ReturnType      XPD_RCC_PLLSAI1Config       (const RCC_PLL_InitType * Config);
#ifdef RCC_PLLSAI2_SUPPORT
XPD_ReturnType      XPD_RCC_PLLSAI2Config       (const RCC_PLL_InitType * Config);
#endif
#ifdef HSE_VALUE
XPD_ReturnType      XPD_RCC_HSEConfig           (RCC_OscStateType NewState);
#endif
#ifdef LSE_VALUE
XPD_ReturnType      XPD_RCC_LSEConfig           (RCC_OscStateType NewState);
#endif
#ifdef RCC_HSI48_SUPPORT
XPD_ReturnType      XPD_RCC_HSI48Config         (RCC_OscStateType NewState);
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

void                XPD_RCC_LSCO_Init           (RCC_LSCO_ClockSourceType LSCOSource);
void                XPD_RCC_LSCO_Deinit         (void);
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
    /* Check RCC LSECSSF flag  */
    if (XPD_RCC_GetFlag(LSECSS) != 0)
    {
        /* Clear RCC LSECSS pending bit */
        XPD_RCC_ClearFlag(LSECSS);

        /* RCC Low Speed Clock Security System interrupt user callback */
        XPD_SAFE_CALLBACK(XPD_RCC_Callbacks.LS_CSS,);
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
