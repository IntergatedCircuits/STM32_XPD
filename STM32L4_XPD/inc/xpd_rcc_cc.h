/**
  ******************************************************************************
  * @file    xpd_rcc_cc.h
  * @author  Benedek Kupper
  * @version 0.3
  * @date    2018-01-28
  * @brief   STM32 eXtensible Peripheral Drivers RCC Core Clocks Module
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
#ifndef __XPD_RCC_CC_H_
#define __XPD_RCC_CC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <xpd_common.h>

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
#ifdef HSE_VALUE_Hz
    HSE    = 2, /*!< High speed external oscillator */
#endif
    PLL    = 3, /*!< Phase locked loop */
    LSI    = 4, /*!< Low speed internal oscillator */
#ifdef LSE_VALUE_Hz
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
    FunctionalState  State;            /*!< MSI state */
    RCC_MSIFreqType  ClockFreq;        /*!< MSI clock frequency */
}RCC_MSI_InitType;

/** @brief PLL setup structure */
typedef struct
{
    uint8_t  M; /*!< Common PLL predivider, only set for main PLL [1 .. 8] */
    uint16_t N; /*!< Multiplication factor for PLL [8 .. 86] */
    uint8_t  R; /*!< Division factor for main system clock.
                     Permitted values: 2, 4, 6, 8 */
    uint8_t  Q; /*!< Division factor for CLK48 input.
                     Permitted values: 2, 4, 6, 8 */
    uint8_t  P; /*!< PLL division factor for  SAI clocks.
                     @arg for advanced devices: [2 .. 31]
                     @arg otherwise: 7, 17 */
    FunctionalState  State;  /*!< PLL state */
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

/** @brief RCC reset source types */
typedef enum
{
    RESET_SOURCE_UNKNOWN   = 0x00, /*!< Reset source unknown */
    RESET_SOURCE_LOWPOWER  = 0x80, /*!< Illegal Low power mode entry caused reset occurred */
    RESET_SOURCE_WWDG      = 0x40, /*!< Window watchdog reset occurred */
    RESET_SOURCE_IWDG      = 0x20, /*!< Independent watchdog reset occurred */
    RESET_SOURCE_SOFTWARE  = 0x10, /*!< Software reset occurred */
    RESET_SOURCE_BROWNOUT  = 0x08, /*!< BrownOutReset occurred */
    RESET_SOURCE_NRST      = 0x04, /*!< NRST pin triggered occurred */
#ifdef RCC_CSR_OBLRSTF
    RESET_SOURCE_OB_LOADER = 0x02, /*!< Option byte loader reset occurred */
#endif
#ifdef RCC_CSR_FWRSTF
    RESET_SOURCE_FIREWALL  = 0x01, /*!< Firewall initiated reset occurred */
#endif
}RCC_ResetSourceType;

/** @brief RCC callbacks container structure */
typedef struct {
    XPD_SimpleCallbackType OscReady; /*!< Oscillator ready callback */
    XPD_SimpleCallbackType CSS;      /*!< Clock Security System callback */
    XPD_SimpleCallbackType LS_CSS;   /*!< LSE Dedicated Clock Security System callback */
}RCC_CallbacksType;

/** @} */

/** @defgroup RCC_Core_Exported_Variables RCC Core Exported Variables
 * @{ */

/** @brief RCC callbacks container struct */
extern RCC_CallbacksType RCC_xCallbacks;

/** @} */

/** @defgroup RCC_Core_Exported_Macros RCC Core Exported Macros
 * @{ */

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
#define             RCC_IT_ENABLE(IT_NAME)              \
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
#define             RCC_IT_DISABLE(IT_NAME)             \
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
#define             RCC_FLAG_STATUS(FLAG_NAME)          \
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
#define             RCC_FLAG_CLEAR(FLAG_NAME)           \
    (RCC_REG_BIT(CICR,FLAG_NAME##C) = 1)

/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions
 * @{ */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Oscillators
 * @{ */
XPD_ReturnType      RCC_eMSI_Config         (const RCC_MSI_InitType * pxConfig);

XPD_ReturnType      RCC_eHSI_Enable         (void);
XPD_ReturnType      RCC_eHSI_Disable        (void);

XPD_ReturnType      RCC_eLSI_Enable         (void);
XPD_ReturnType      RCC_eLSI_Disable        (void);
#ifdef RCC_HSI48_SUPPORT
XPD_ReturnType      RCC_eHSI48_Enable       (void);
XPD_ReturnType      RCC_eHSI48_Disable      (void);
#endif

#ifdef HSE_VALUE_Hz
XPD_ReturnType      RCC_eHSE_Config         (RCC_OscStateType eOscState);
#endif
#ifdef LSE_VALUE_Hz
XPD_ReturnType      RCC_eLSE_Config         (RCC_OscStateType eOscState);
#endif

XPD_ReturnType      RCC_ePLL_Config         (const RCC_PLL_InitType * pxConfig);
uint32_t            RCC_ulOscFreq_Hz        (RCC_OscType eOscillator);

XPD_ReturnType      RCC_ePLLSAI1_Config     (const RCC_PLL_InitType * pxConfig);
#ifdef RCC_PLLSAI2_SUPPORT
XPD_ReturnType      RCC_ePLLSAI2_Config     (const RCC_PLL_InitType * pxConfig);
#endif

void                RCC_vIRQHandler         (void);
RCC_OscType         RCC_eGetReadyOsc        (void);
/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Clocks
 * @{ */
XPD_ReturnType      RCC_eHCLK_Config        (RCC_OscType eSYSCLK_Source, ClockDividerType eHCLK_Divider,
                                             uint8_t ucFlashLatency);
void                RCC_vPCLK1_Config       (ClockDividerType ePCLK_Divider);
void                RCC_vPCLK2_Config       (ClockDividerType ePCLK_Divider);
uint32_t            RCC_ulClockFreq_Hz      (RCC_ClockType eSelectedClock);
/** @} */

/** @defgroup RCC_Core_Clocks_Exported_Functions_CSS RCC Clock Security System
 *  @brief    RCC Clock Security System
 * @{
 */

/**
 * @brief RCC NMI interrupt handler that provides Clock Security System callback.
 */
__STATIC_INLINE void CSS_vIRQHandler(void)
{
    /* Check RCC CSSF flag  */
    if (RCC_FLAG_STATUS(CSS) != 0)
    {
        /* Clear RCC CSS pending bit */
        RCC_FLAG_CLEAR(CSS);

        /* RCC Clock Security System interrupt user callback */
        XPD_SAFE_CALLBACK(RCC_xCallbacks.CSS,);
    }
    /* Check RCC LSECSSF flag  */
    if (RCC_FLAG_STATUS(LSECSS) != 0)
    {
        /* Clear RCC LSECSS pending bit */
        RCC_FLAG_CLEAR(LSECSS);

        /* RCC Low Speed Clock Security System interrupt user callback */
        XPD_SAFE_CALLBACK(RCC_xCallbacks.LS_CSS,);
    }
}

/**
 * @brief Enables or disables the Clock Security System
 * @param eNewState: the CSS activation
 */
__STATIC_INLINE void RCC_vCSS(FunctionalState eNewState)
{
    RCC_REG_BIT(CR,CSSON) = eNewState;
}

/**
 * @brief Enables or disables the Low Speed Clock Security System
 * @param eNewState: the CSS activation
 * @note  Enable only after LSERDY and LSIRDY are set and the RTC clock is configured.
 * @note  Disable only after a LSE failure is detected.
 */
__STATIC_INLINE void RCC_vLS_CSS(FunctionalState eNewState)
{
    RCC_REG_BIT(BDCR,LSECSSON) = eNewState;
}

/** @} */

/** @addtogroup RCC_Core_Clocks_Exported_Functions_Reset
 * @{ */
void                RCC_vDeinit             (void);

void                RCC_vResetAHB1          (void);
void                RCC_vResetAHB2          (void);
void                RCC_vResetAHB3          (void);
void                RCC_vResetAPB1          (void);
void                RCC_vResetAPB2          (void);

/**
 * @brief Reads the reset source flags.
 * @return The RCC peripheral determined reset source
 */
__STATIC_INLINE RCC_ResetSourceType RCC_eGetResetSource(void)
{
    return (RCC->CSR.w & (~RCC_CSR_RMVF)) >> 24;
}

/**
 * @brief Clears the reset source flags from RCC.
 */
__STATIC_INLINE void RCC_vClearResetSource(void)
{
    RCC_REG_BIT(CSR,RMVF) = 1;
}

/** @} */

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __XPD_RCC_CC_H_ */
