#include "xpd_rcc.h"

/** @addtogroup RCC
 * @{ */

/** @addtogroup RCC_Peripheral_Control
 * @{ */

/** @addtogroup RCC_Generated_Functions
 * @{ */
#ifdef RCC_AHBENR_ADC1EN
/** @brief Sets the new clock state of the ADC1 peripheral. */
void XPD_ADC1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,ADC1EN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_ADC1RST
/** @brief Forces and releases a reset on the ADC1 peripheral. */
void XPD_ADC1_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,ADC1RST) = 1;
    RCC_REG_BIT(AHBRSTR,ADC1RST) = 0;
}
#endif
#ifdef RCC_AHBENR_ADC12EN
/** @brief Sets the new clock state of the ADC12 peripheral. */
void XPD_ADC12_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,ADC12EN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_ADC12RST
/** @brief Forces and releases a reset on the ADC12 peripheral. */
void XPD_ADC12_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,ADC12RST) = 1;
    RCC_REG_BIT(AHBRSTR,ADC12RST) = 0;
}
#endif
#ifdef RCC_AHBENR_ADC34EN
/** @brief Sets the new clock state of the ADC34 peripheral. */
void XPD_ADC34_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,ADC34EN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_ADC34RST
/** @brief Forces and releases a reset on the ADC34 peripheral. */
void XPD_ADC34_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,ADC34RST) = 1;
    RCC_REG_BIT(AHBRSTR,ADC34RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_CANEN
/** @brief Sets the new clock state of the CAN peripheral. */
void XPD_CAN_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CANEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_CANRST
/** @brief Forces and releases a reset on the CAN peripheral. */
void XPD_CAN_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CANRST) = 1;
    RCC_REG_BIT(APB1RSTR,CANRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_CECEN
/** @brief Sets the new clock state of the CEC peripheral. */
void XPD_CEC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,CECEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_CECRST
/** @brief Forces and releases a reset on the CEC peripheral. */
void XPD_CEC_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,CECRST) = 1;
    RCC_REG_BIT(APB1RSTR,CECRST) = 0;
}
#endif
#ifdef RCC_AHBENR_CRCEN
/** @brief Sets the new clock state of the CRC peripheral. */
void XPD_CRC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,CRCEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_APB1ENR_DAC1EN
/** @brief Sets the new clock state of the DAC1 peripheral. */
void XPD_DAC1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,DAC1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_DAC1RST
/** @brief Forces and releases a reset on the DAC1 peripheral. */
void XPD_DAC1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,DAC1RST) = 1;
    RCC_REG_BIT(APB1RSTR,DAC1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_DAC2EN
/** @brief Sets the new clock state of the DAC2 peripheral. */
void XPD_DAC2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,DAC2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_DAC2RST
/** @brief Forces and releases a reset on the DAC2 peripheral. */
void XPD_DAC2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,DAC2RST) = 1;
    RCC_REG_BIT(APB1RSTR,DAC2RST) = 0;
}
#endif
#ifdef RCC_AHBENR_DMA1EN
/** @brief Sets the new clock state of the DMA1 peripheral. */
void XPD_DMA1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,DMA1EN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBENR_DMA2EN
/** @brief Sets the new clock state of the DMA2 peripheral. */
void XPD_DMA2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,DMA2EN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBENR_FLITFEN
/** @brief Sets the new clock state of the FLITF peripheral. */
void XPD_FLITF_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,FLITFEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBENR_FMCEN
/** @brief Sets the new clock state of the FMC peripheral. */
void XPD_FMC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,FMCEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_FMCRST
/** @brief Forces and releases a reset on the FMC peripheral. */
void XPD_FMC_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,FMCRST) = 1;
    RCC_REG_BIT(AHBRSTR,FMCRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOAEN
/** @brief Sets the new clock state of the GPIOA peripheral. */
void XPD_GPIOA_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOAEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOARST
/** @brief Forces and releases a reset on the GPIOA peripheral. */
void XPD_GPIOA_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOARST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOARST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOBEN
/** @brief Sets the new clock state of the GPIOB peripheral. */
void XPD_GPIOB_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOBEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOBRST
/** @brief Forces and releases a reset on the GPIOB peripheral. */
void XPD_GPIOB_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOBRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOBRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOCEN
/** @brief Sets the new clock state of the GPIOC peripheral. */
void XPD_GPIOC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOCEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOCRST
/** @brief Forces and releases a reset on the GPIOC peripheral. */
void XPD_GPIOC_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOCRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOCRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIODEN
/** @brief Sets the new clock state of the GPIOD peripheral. */
void XPD_GPIOD_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIODEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIODRST
/** @brief Forces and releases a reset on the GPIOD peripheral. */
void XPD_GPIOD_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIODRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIODRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOEEN
/** @brief Sets the new clock state of the GPIOE peripheral. */
void XPD_GPIOE_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOEEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOERST
/** @brief Forces and releases a reset on the GPIOE peripheral. */
void XPD_GPIOE_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOERST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOERST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOFEN
/** @brief Sets the new clock state of the GPIOF peripheral. */
void XPD_GPIOF_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOFEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOFRST
/** @brief Forces and releases a reset on the GPIOF peripheral. */
void XPD_GPIOF_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOFRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOFRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOGEN
/** @brief Sets the new clock state of the GPIOG peripheral. */
void XPD_GPIOG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOGEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOGRST
/** @brief Forces and releases a reset on the GPIOG peripheral. */
void XPD_GPIOG_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOGRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOGRST) = 0;
}
#endif
#ifdef RCC_AHBENR_GPIOHEN
/** @brief Sets the new clock state of the GPIOH peripheral. */
void XPD_GPIOH_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,GPIOHEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_GPIOHRST
/** @brief Forces and releases a reset on the GPIOH peripheral. */
void XPD_GPIOH_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,GPIOHRST) = 1;
    RCC_REG_BIT(AHBRSTR,GPIOHRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_HRTIM1EN
/** @brief Sets the new clock state of the HRTIM1 peripheral. */
void XPD_HRTIM1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,HRTIM1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_HRTIM1RST
/** @brief Forces and releases a reset on the HRTIM1 peripheral. */
void XPD_HRTIM1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,HRTIM1RST) = 1;
    RCC_REG_BIT(APB2RSTR,HRTIM1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C1EN
/** @brief Sets the new clock state of the I2C1 peripheral. */
void XPD_I2C1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C1EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_I2C1RST
/** @brief Forces and releases a reset on the I2C1 peripheral. */
void XPD_I2C1_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C2EN
/** @brief Sets the new clock state of the I2C2 peripheral. */
void XPD_I2C2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_I2C2RST
/** @brief Forces and releases a reset on the I2C2 peripheral. */
void XPD_I2C2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_I2C3EN
/** @brief Sets the new clock state of the I2C3 peripheral. */
void XPD_I2C3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,I2C3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_I2C3RST
/** @brief Forces and releases a reset on the I2C3 peripheral. */
void XPD_I2C3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 1;
    RCC_REG_BIT(APB1RSTR,I2C3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_PWREN
/** @brief Sets the new clock state of the PWR peripheral. */
void XPD_PWR_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,PWREN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_PWRRST
/** @brief Forces and releases a reset on the PWR peripheral. */
void XPD_PWR_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,PWRRST) = 1;
    RCC_REG_BIT(APB1RSTR,PWRRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SDADC1EN
/** @brief Sets the new clock state of the SDADC1 peripheral. */
void XPD_SDADC1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SDADC1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_SDADC1RST
/** @brief Forces and releases a reset on the SDADC1 peripheral. */
void XPD_SDADC1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SDADC1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SDADC1RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SDADC2EN
/** @brief Sets the new clock state of the SDADC2 peripheral. */
void XPD_SDADC2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SDADC2EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_SDADC2RST
/** @brief Forces and releases a reset on the SDADC2 peripheral. */
void XPD_SDADC2_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SDADC2RST) = 1;
    RCC_REG_BIT(APB2RSTR,SDADC2RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SDADC3EN
/** @brief Sets the new clock state of the SDADC3 peripheral. */
void XPD_SDADC3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SDADC3EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_SDADC3RST
/** @brief Forces and releases a reset on the SDADC3 peripheral. */
void XPD_SDADC3_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SDADC3RST) = 1;
    RCC_REG_BIT(APB2RSTR,SDADC3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI1EN
/** @brief Sets the new clock state of the SPI1 peripheral. */
void XPD_SPI1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SPI1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_SPI1RST
/** @brief Forces and releases a reset on the SPI1 peripheral. */
void XPD_SPI1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPI2EN
/** @brief Sets the new clock state of the SPI2 peripheral. */
void XPD_SPI2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,SPI2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_SPI2RST
/** @brief Forces and releases a reset on the SPI2 peripheral. */
void XPD_SPI2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_SPI3EN
/** @brief Sets the new clock state of the SPI3 peripheral. */
void XPD_SPI3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,SPI3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_SPI3RST
/** @brief Forces and releases a reset on the SPI3 peripheral. */
void XPD_SPI3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 1;
    RCC_REG_BIT(APB1RSTR,SPI3RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_SPI4EN
/** @brief Sets the new clock state of the SPI4 peripheral. */
void XPD_SPI4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SPI4EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_SPI4RST
/** @brief Forces and releases a reset on the SPI4 peripheral. */
void XPD_SPI4_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 1;
    RCC_REG_BIT(APB2RSTR,SPI4RST) = 0;
}
#endif
#ifdef RCC_AHBENR_SRAMEN
/** @brief Sets the new clock state of the SRAM peripheral. */
void XPD_SRAM_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,SRAMEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
/** @brief Sets the new clock state of the SYSCFG peripheral. */
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,SYSCFGEN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
/** @brief Forces and releases a reset on the SYSCFG peripheral. */
void XPD_SYSCFG_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 1;
    RCC_REG_BIT(APB2RSTR,SYSCFGRST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM1EN
/** @brief Sets the new clock state of the TIM1 peripheral. */
void XPD_TIM1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_TIM1RST
/** @brief Forces and releases a reset on the TIM1 peripheral. */
void XPD_TIM1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM12EN
/** @brief Sets the new clock state of the TIM12 peripheral. */
void XPD_TIM12_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM12EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM12RST
/** @brief Forces and releases a reset on the TIM12 peripheral. */
void XPD_TIM12_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM12RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM13EN
/** @brief Sets the new clock state of the TIM13 peripheral. */
void XPD_TIM13_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM13EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM13RST
/** @brief Forces and releases a reset on the TIM13 peripheral. */
void XPD_TIM13_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM13RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM14EN
/** @brief Sets the new clock state of the TIM14 peripheral. */
void XPD_TIM14_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM14EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM14RST
/** @brief Forces and releases a reset on the TIM14 peripheral. */
void XPD_TIM14_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM14RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM15EN
/** @brief Sets the new clock state of the TIM15 peripheral. */
void XPD_TIM15_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM15EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_TIM15RST
/** @brief Forces and releases a reset on the TIM15 peripheral. */
void XPD_TIM15_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM15RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM15RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM16EN
/** @brief Sets the new clock state of the TIM16 peripheral. */
void XPD_TIM16_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM16EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_TIM16RST
/** @brief Forces and releases a reset on the TIM16 peripheral. */
void XPD_TIM16_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM16RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM16RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM17EN
/** @brief Sets the new clock state of the TIM17 peripheral. */
void XPD_TIM17_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM17EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_TIM17RST
/** @brief Forces and releases a reset on the TIM17 peripheral. */
void XPD_TIM17_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM17RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM17RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM18EN
/** @brief Sets the new clock state of the TIM18 peripheral. */
void XPD_TIM18_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM18EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM18RST
/** @brief Forces and releases a reset on the TIM18 peripheral. */
void XPD_TIM18_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM18RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM18RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM19EN
/** @brief Sets the new clock state of the TIM19 peripheral. */
void XPD_TIM19_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM19EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_TIM19RST
/** @brief Forces and releases a reset on the TIM19 peripheral. */
void XPD_TIM19_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM19RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM19RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM2EN
/** @brief Sets the new clock state of the TIM2 peripheral. */
void XPD_TIM2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM2RST
/** @brief Forces and releases a reset on the TIM2 peripheral. */
void XPD_TIM2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM2RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM20EN
/** @brief Sets the new clock state of the TIM20 peripheral. */
void XPD_TIM20_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM20EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_TIM20RST
/** @brief Forces and releases a reset on the TIM20 peripheral. */
void XPD_TIM20_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM20RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM20RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM3EN
/** @brief Sets the new clock state of the TIM3 peripheral. */
void XPD_TIM3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM3RST
/** @brief Forces and releases a reset on the TIM3 peripheral. */
void XPD_TIM3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM4EN
/** @brief Sets the new clock state of the TIM4 peripheral. */
void XPD_TIM4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM4RST
/** @brief Forces and releases a reset on the TIM4 peripheral. */
void XPD_TIM4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM5EN
/** @brief Sets the new clock state of the TIM5 peripheral. */
void XPD_TIM5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM5EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM5RST
/** @brief Forces and releases a reset on the TIM5 peripheral. */
void XPD_TIM5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM5RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM6EN
/** @brief Sets the new clock state of the TIM6 peripheral. */
void XPD_TIM6_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM6EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM6RST
/** @brief Forces and releases a reset on the TIM6 peripheral. */
void XPD_TIM6_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM6RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_TIM7EN
/** @brief Sets the new clock state of the TIM7 peripheral. */
void XPD_TIM7_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,TIM7EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_TIM7RST
/** @brief Forces and releases a reset on the TIM7 peripheral. */
void XPD_TIM7_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 1;
    RCC_REG_BIT(APB1RSTR,TIM7RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_TIM8EN
/** @brief Sets the new clock state of the TIM8 peripheral. */
void XPD_TIM8_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,TIM8EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_TIM8RST
/** @brief Forces and releases a reset on the TIM8 peripheral. */
void XPD_TIM8_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 1;
    RCC_REG_BIT(APB2RSTR,TIM8RST) = 0;
}
#endif
#ifdef RCC_AHBENR_TSCEN
/** @brief Sets the new clock state of the TSC peripheral. */
void XPD_TSC_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(AHBENR,TSCEN) = NewState;
    NewState = (FunctionalState)RCC->AHBENR.w;
}
#endif
#ifdef RCC_AHBRSTR_TSCRST
/** @brief Forces and releases a reset on the TSC peripheral. */
void XPD_TSC_Reset(void)
{
    RCC_REG_BIT(AHBRSTR,TSCRST) = 1;
    RCC_REG_BIT(AHBRSTR,TSCRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART4EN
/** @brief Sets the new clock state of the UART4 peripheral. */
void XPD_UART4_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,UART4EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_UART4RST
/** @brief Forces and releases a reset on the UART4 peripheral. */
void XPD_UART4_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART4RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART4RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_UART5EN
/** @brief Sets the new clock state of the UART5 peripheral. */
void XPD_UART5_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,UART5EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_UART5RST
/** @brief Forces and releases a reset on the UART5 peripheral. */
void XPD_UART5_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,UART5RST) = 1;
    RCC_REG_BIT(APB1RSTR,UART5RST) = 0;
}
#endif
#ifdef RCC_APB2ENR_USART1EN
/** @brief Sets the new clock state of the USART1 peripheral. */
void XPD_USART1_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB2ENR,USART1EN) = NewState;
    NewState = (FunctionalState)RCC->APB2ENR.w;
}
#endif
#ifdef RCC_APB2RSTR_USART1RST
/** @brief Forces and releases a reset on the USART1 peripheral. */
void XPD_USART1_Reset(void)
{
    RCC_REG_BIT(APB2RSTR,USART1RST) = 1;
    RCC_REG_BIT(APB2RSTR,USART1RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART2EN
/** @brief Sets the new clock state of the USART2 peripheral. */
void XPD_USART2_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART2EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USART2RST
/** @brief Forces and releases a reset on the USART2 peripheral. */
void XPD_USART2_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART2RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART2RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USART3EN
/** @brief Sets the new clock state of the USART3 peripheral. */
void XPD_USART3_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USART3EN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USART3RST
/** @brief Forces and releases a reset on the USART3 peripheral. */
void XPD_USART3_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USART3RST) = 1;
    RCC_REG_BIT(APB1RSTR,USART3RST) = 0;
}
#endif
#ifdef RCC_APB1ENR_USBEN
/** @brief Sets the new clock state of the USB peripheral. */
void XPD_USB_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,USBEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_USBRST
/** @brief Forces and releases a reset on the USB peripheral. */
void XPD_USB_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,USBRST) = 1;
    RCC_REG_BIT(APB1RSTR,USBRST) = 0;
}
#endif
#ifdef RCC_APB1ENR_WWDGEN
/** @brief Sets the new clock state of the WWDG peripheral. */
void XPD_WWDG_ClockCtrl(FunctionalState NewState)
{
    RCC_REG_BIT(APB1ENR,WWDGEN) = NewState;
    NewState = (FunctionalState)RCC->APB1ENR.w;
}
#endif
#ifdef RCC_APB1RSTR_WWDGRST
/** @brief Forces and releases a reset on the WWDG peripheral. */
void XPD_WWDG_Reset(void)
{
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 1;
    RCC_REG_BIT(APB1RSTR,WWDGRST) = 0;
}
#endif

/** @} */

/** @} */

/** @} */
