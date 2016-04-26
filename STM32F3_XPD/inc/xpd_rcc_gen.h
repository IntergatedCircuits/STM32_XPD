#ifndef XPD_RCC_GEN_H_
#define XPD_RCC_GEN_H_
#include "xpd_rcc.h"


/** @addtogroup RCC
 * @{ */

/** @defgroup RCC_Peripheral_Control RCC Peripheral Control
 * @{ */

/** @defgroup RCC_Generated_Functions RCC Generated Functions
 * @{ */
#ifdef RCC_AHBENR_ADC1EN
void XPD_ADC1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_ADC1RST
void XPD_ADC1_Reset(void);
#endif
#ifdef RCC_AHBENR_ADC12EN
void XPD_ADC12_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_ADC12RST
void XPD_ADC12_Reset(void);
#endif
#ifdef RCC_AHBENR_ADC34EN
void XPD_ADC34_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_ADC34RST
void XPD_ADC34_Reset(void);
#endif
#ifdef RCC_APB1ENR_CANEN
void XPD_CAN_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CANRST
void XPD_CAN_Reset(void);
#endif
#ifdef RCC_APB1ENR_CECEN
void XPD_CEC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_CECRST
void XPD_CEC_Reset(void);
#endif
#ifdef RCC_AHBENR_CRCEN
void XPD_CRC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1ENR_DAC1EN
void XPD_DAC1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_DAC1RST
void XPD_DAC1_Reset(void);
#endif
#ifdef RCC_APB1ENR_DAC2EN
void XPD_DAC2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_DAC2RST
void XPD_DAC2_Reset(void);
#endif
#ifdef RCC_AHBENR_DMA1EN
void XPD_DMA1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBENR_DMA2EN
void XPD_DMA2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBENR_FLITFEN
void XPD_FLITF_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBENR_FMCEN
void XPD_FMC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_FMCRST
void XPD_FMC_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOAEN
void XPD_GPIOA_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOARST
void XPD_GPIOA_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOBEN
void XPD_GPIOB_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOBRST
void XPD_GPIOB_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOCEN
void XPD_GPIOC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOCRST
void XPD_GPIOC_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIODEN
void XPD_GPIOD_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIODRST
void XPD_GPIOD_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOEEN
void XPD_GPIOE_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOERST
void XPD_GPIOE_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOFEN
void XPD_GPIOF_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOFRST
void XPD_GPIOF_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOGEN
void XPD_GPIOG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOGRST
void XPD_GPIOG_Reset(void);
#endif
#ifdef RCC_AHBENR_GPIOHEN
void XPD_GPIOH_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_GPIOHRST
void XPD_GPIOH_Reset(void);
#endif
#ifdef RCC_APB2ENR_HRTIM1EN
void XPD_HRTIM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_HRTIM1RST
void XPD_HRTIM1_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C1EN
void XPD_I2C1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C1RST
void XPD_I2C1_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C2EN
void XPD_I2C2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C2RST
void XPD_I2C2_Reset(void);
#endif
#ifdef RCC_APB1ENR_I2C3EN
void XPD_I2C3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_I2C3RST
void XPD_I2C3_Reset(void);
#endif
#ifdef RCC_APB1ENR_PWREN
void XPD_PWR_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_PWRRST
void XPD_PWR_Reset(void);
#endif
#ifdef RCC_APB2ENR_SDADC1EN
void XPD_SDADC1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SDADC1RST
void XPD_SDADC1_Reset(void);
#endif
#ifdef RCC_APB2ENR_SDADC2EN
void XPD_SDADC2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SDADC2RST
void XPD_SDADC2_Reset(void);
#endif
#ifdef RCC_APB2ENR_SDADC3EN
void XPD_SDADC3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SDADC3RST
void XPD_SDADC3_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI1EN
void XPD_SPI1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI1RST
void XPD_SPI1_Reset(void);
#endif
#ifdef RCC_APB1ENR_SPI2EN
void XPD_SPI2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_SPI2RST
void XPD_SPI2_Reset(void);
#endif
#ifdef RCC_APB1ENR_SPI3EN
void XPD_SPI3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_SPI3RST
void XPD_SPI3_Reset(void);
#endif
#ifdef RCC_APB2ENR_SPI4EN
void XPD_SPI4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SPI4RST
void XPD_SPI4_Reset(void);
#endif
#ifdef RCC_AHBENR_SRAMEN
void XPD_SRAM_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2ENR_SYSCFGEN
void XPD_SYSCFG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_SYSCFGRST
void XPD_SYSCFG_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM1EN
void XPD_TIM1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM1RST
void XPD_TIM1_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM12EN
void XPD_TIM12_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM12RST
void XPD_TIM12_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM13EN
void XPD_TIM13_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM13RST
void XPD_TIM13_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM14EN
void XPD_TIM14_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM14RST
void XPD_TIM14_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM15EN
void XPD_TIM15_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM15RST
void XPD_TIM15_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM16EN
void XPD_TIM16_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM16RST
void XPD_TIM16_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM17EN
void XPD_TIM17_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM17RST
void XPD_TIM17_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM18EN
void XPD_TIM18_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM18RST
void XPD_TIM18_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM19EN
void XPD_TIM19_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM19RST
void XPD_TIM19_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM2EN
void XPD_TIM2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM2RST
void XPD_TIM2_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM20EN
void XPD_TIM20_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM20RST
void XPD_TIM20_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM3EN
void XPD_TIM3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM3RST
void XPD_TIM3_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM4EN
void XPD_TIM4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM4RST
void XPD_TIM4_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM5EN
void XPD_TIM5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM5RST
void XPD_TIM5_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM6EN
void XPD_TIM6_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM6RST
void XPD_TIM6_Reset(void);
#endif
#ifdef RCC_APB1ENR_TIM7EN
void XPD_TIM7_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_TIM7RST
void XPD_TIM7_Reset(void);
#endif
#ifdef RCC_APB2ENR_TIM8EN
void XPD_TIM8_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_TIM8RST
void XPD_TIM8_Reset(void);
#endif
#ifdef RCC_AHBENR_TSCEN
void XPD_TSC_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_AHBRSTR_TSCRST
void XPD_TSC_Reset(void);
#endif
#ifdef RCC_APB1ENR_UART4EN
void XPD_UART4_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_UART4RST
void XPD_UART4_Reset(void);
#endif
#ifdef RCC_APB1ENR_UART5EN
void XPD_UART5_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_UART5RST
void XPD_UART5_Reset(void);
#endif
#ifdef RCC_APB2ENR_USART1EN
void XPD_USART1_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB2RSTR_USART1RST
void XPD_USART1_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART2EN
void XPD_USART2_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART2RST
void XPD_USART2_Reset(void);
#endif
#ifdef RCC_APB1ENR_USART3EN
void XPD_USART3_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USART3RST
void XPD_USART3_Reset(void);
#endif
#ifdef RCC_APB1ENR_USBEN
void XPD_USB_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_USBRST
void XPD_USB_Reset(void);
#endif
#ifdef RCC_APB1ENR_WWDGEN
void XPD_WWDG_ClockCtrl(FunctionalState NewState);
#endif
#ifdef RCC_APB1RSTR_WWDGRST
void XPD_WWDG_Reset(void);
#endif

/** @} */

/** @} */

/** @} */

#endif /* XPD_RCC_GEN_H_ */
