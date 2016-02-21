# STM32 eXtensible Peripheral Drivers

This project aims to provide a convenient and efficient C programming toolset and API to handle STM32 series microcontroller peripherals.

## 1 Peripheral structures with bit fields

All peripheral mapping structures are expanded with the bit maps of each register using a custom-made code generator. (The results are found [here](/CMSIS/Include) for the core registers and in the CMSIS/Device/ST/STM32__xx/Include folders for the peripheral registers.) The new core and device headers are incompatible with the old drivers, hence the necessity for new peripheral driver development.

The main advantage of the bit fields is that the whole operation of setting certain bits is much more simplified on the c code level. Instead of
```C
XXX->REG &= ~XXX_REG_FIELD_Msk;
XXX->REG |= newValue << XXX_REG_FIELD_Pos;
```
one can write 
```C
XXX->REG.b.FIELD = newValue;
```
and the compiler will produce the necessary operations that are explicitly written in the previous example. As the different bit shifts are no longer necessary, the code requires less symbols.

## 2 Full bit-band support
The Cortex M3, M4 and M7 cores enable the first 1MB of the SRAM and peripheral address space to be addressed bitwise, using a bit remapping address space called the alias region. A detailed explanation on the subject is available [here](/BitBanding.md). The project provides structures that make it possible to execute atomic bit reading and writing operations on the peripherals that are in the specified address range. The address transformation to bit-band alias is done in compile-time if the peripheral is given in compile-time. Therefore peripheral types with a single instance have a direct bit-band address, e.g.
```C
#define RCC_BB                    ((RCC_BitBand_TypeDef *) PERIPH_BB(RCC_BASE))
```
The XPD drivers use the following macro to switch between bit-banding and regular bit manipulation:
```C
#ifdef RCC_BB
#define RCC_REG_BIT(REG_NAME, BIT_NAME) (RCC_BB->REG_NAME.BIT_NAME)
#else
#define RCC_REG_BIT(REG_NAME, BIT_NAME) (RCC->REG_NAME.b.BIT_NAME)
#endif
```
Peripheral types which have multiple instances are accessed through *handles*. If bit-banding addressing is possible, a bit-band alias address field is added to this structure, and the address is calculated in the Init() function of the driver. A very similar macro is used for switching between addressing modes as the above one:
```C
#ifdef TIM_BB
#define TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#else
#define TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)
#endif
```
## 3 Run-time configurable callback architecture
The XPD drivers use function pointers to provide callback services to the application. These are either located in global variables or in the handle structure.

Feedback
========
This project is in development phase, so if you find any bugs, have any questions or constructive ideas please don't be afraid to share them here.
