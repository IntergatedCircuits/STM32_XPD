# STM32 eXtensible Peripheral Drivers

This project aims to provide a convenient and efficient C programming toolset and API to handle Cortex M - and more specifically, STM32 - series microcontrollers. The main benefits of this library that it provides bit-fields for all registers and bit-band alias mapped fields for affected registers of the controller. The peripheral drivers are designed for maximum portability within the STM32 device family.

#### Table of contents

<!--[TOC]-->

1. [Project scope](#scope)
2. [Benefits of bit fields](#fields)
3. [Benefits of bit-band support](#bitband)
4. [XPD User's Guide](#xpd_guide)
5. [Contact and Feedback](#feedback)

## Project scope <a name="scope"></a>

The core motivation of this library is to make register manipulation as straightforward and efficient as possible. As the CMSIS standard requires only the register level specification for peripherals, those structures needed to be replaced in their original location. Therefore the CMSIS folder of this repository should be considered as a partial replacement (upgrade) of the existing one.
The affected files are the following:
- The Include/core_cm[X].h headers contain core registers which have their bit fields added.
- The Include/core_cmBitband.h header introduces bit-band alias address calculating macros.
- The Device/ST/STM32[XX]XX/Include/stm32[xxxxxx].h headers contain peripheral registers which have their bit fields added, and bit-band alias accessible peripherals also have their register maps created for the alias addressing mode.

As these changes make all existing peripheral drivers incompatible, a new, extensible driver development begun for the preferred target controllers. The development is currently only a free-time hobby, but it can be extended with joint effort, please contact me if you are interested.


## Benefits of bit fields <a name="fields"></a>

The C programming language enables the use of bit fields in the following general syntax:
>```C
typedef union {
    struct {
        uint32_t EN : 1; /* Enable bit             */
        uint32_t IG : 1; /* Interrupt generate bit */
        uint32_t _reserved0 : 30;
    } b;        /* Structure used for bit  access  */
    uint32_t w; /* Type      used for word access  */
} REG_Type;
```

The bit fields of the structure are accessed the same way as of any other structure, it is the compiler which applies the necessary masking, shifting and setting operations. Therefore a big deal of programmer effort can be saved by using the introduced bit fields, as these compiler actions are otherwise required to be explicitly written in the source code. Examples for both read and write operations follow.

### Bit field write
Using word access: 
>```C
REG.w &= ~REG_IG_Msk;
REG.w |= newValue << REG_IG_Pos;
```

Using bit access:
>```C
REG.b.IG = newValue;
```

### Bit field read
Using word access:
>```C
newValue = (REG.w & REG_IG_Msk) >> REG_IG_Pos;
```

Using bit access:
>```C
newValue = REG.b.IG;
```

As these examples clearly demonstrate, bit field access is highly beneficial for not only the development of device code, but also for code review, maintenance and compatibility checks.

## Benefits of bit-band support <a name="bitband"></a>

The Cortex M3, M4 and M7 cores enable the first 1MB of the SRAM and peripheral address space to be addressed bitwise, using a bit remapping address space called the *alias region*. A complete explanation on the subject is available [here](/BitBanding.md). This project provides structures that make it possible to execute atomic bit reading and writing operations on the peripherals that are in the specified address region. The address transformation to bit-band alias can done in compile-time as long as the peripheral (address) is fixed. Therefore peripheral types with a single instance have a direct bit-band address, e.g.
>```C
#define RCC_BB    ((RCC_BitBand_TypeDef *) PERIPH_BB(RCC_BASE))
```

The XPD drivers use the following macro to switch between bit-banding and regular bit manipulation:
>```C
#ifdef RCC_BB
#define RCC_REG_BIT(REG_NAME, BIT_NAME) (RCC_BB->REG_NAME.BIT_NAME)
#else
#define RCC_REG_BIT(REG_NAME, BIT_NAME) (RCC->REG_NAME.b.BIT_NAME)
#endif
```

Peripheral types which have multiple instances are accessed through *handles*. If bit-banding addressing is possible, a bit-band alias address field is added to this structure, and the address is calculated in the Init() function of the driver. A very similar macro is used for switching between addressing modes as the above one:
>```C
#ifdef TIM_BB
#define TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst_BB->REG_NAME.BIT_NAME)
#else
#define TIM_REG_BIT(HANDLE, REG_NAME, BIT_NAME) ((HANDLE)->Inst->REG_NAME.b.BIT_NAME)
#endif
```

## XPD User's Guide <a name="xpd_guide"></a>

A brief description is provided below to familiarize the user with the concepts and ease the use of the XPD driver library.

### Main concept

The XPD library was created to utilize the new features introduced in the CMSIS layer and to create a library with maximum achievable portability through the STM32 family. This library - much like the *STM32 HAL* drivers - uses handles to manage peripherals. But unlike the HAL drivers, the existence of XPD handles are motivated by different reasons:

1. Contain addresses for both word and bit-band access.
2. Contain the instance-specific function pointers that are called by the drivers to manage external reactions to peripheral events. The user code can subscribe to these callbacks with individual functions in order to handle each event individually.
3. Contain the necessary interrupt and DMA management information of the peripheral.

Apart from a core set of peripherals (DMA, GPIO, RCC, etc.), the user can configure which peripheral modules shall be compiled. The driver functions are available in the XPD_[ ] namespace, while the data types are in the peripherals' respective namespaces. The whole library is Doxygen documented, the resulting documentation can be found in each XPD folder.

### Getting started

The first step is to get a copy of the CMSIS library (preferably from the STM32Cube), and overwrite it with the files of this CMSIS folder. The XPD drivers are standalone and (supposed to be) device dependent, so only a project-specific **xpd_config.h** file has be created. This header specifies - among other things - which device header and XPD modules shall be used and what are the external oscillator values and startup configuration settings.

After the XPD configuration, the user code development can begin. The Projects folder contains example codes which demonstrate the basic API usage of the library.

### Hints

1. To ensure safe operation make sure that all the handles are created using their initializer macro: `XXX_HandleType hxxx = NEW_XXX_HANDLE(XXX2, xxx2_init_fn, xxx2_deinit_fn);`

* * *

## Contact and Feedback <a name="feedback"></a>

This project is in development phase, so if you find any bugs, have any questions or constructive ideas please don't be afraid to share them here.
The CMSIS device headers are result of a custom code generator with some necessary touchups, however the publishing of further device headers will only continue on request.