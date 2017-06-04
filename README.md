# Peripheral Drivers for STM32 series MCUs

This project aims to provide an efficient peripheral library for the STM32 series microcontrollers with the potential ability to replace STM32 HAL drivers. The following features are the key to achieving this goal: a straightforward common API with high level of hardware abstraction and code portability, and a comprehensible and lightweight driver code.

## Project overview

The XPD drivers provide a hardware abstraction layer for the MCU peripherals, leaving the user to simply provide configuration sets, I/O data and callbacks to its API for complex device operations.

The XPD is currently under development, therefore only a subset of modules and devices are supported. A certain MCU is supported if its device descriptor header can be found within [CMSIS/Device/ST](https://github.com/IntergatedCircuits/STM32_XPD/tree/master/CMSIS/Device/ST)/STM32..xx/Include. For the list of supported peripheral modules please check the corresponding XPD library.

The project redefines all peripheral layout structures to include bit field definitions, and also introduces bit-band alias peripheral structures, therefore the CMSIS library contents are incompatible with the official release. The XPD library is heavily built on these attributes, therefore it must be used with the CMSIS of the same revision.

## Documentation

The project is well structured and doxygen documented, therefore offering easy understandability and navigation. The [XPD Wiki](https://github.com/IntergatedCircuits/STM32_XPD/wiki) offers a Beginner's Guide as well as detailed explanation of the unique concepts applied in the library.

## Feedback

The CMSIS device descriptors are result of a custom code generator with some manual touchups, therefore certain bit fields might have allocated incorrectly. Generally only the XPD supported peripherals' fields can be relied upon. Furthermore, part of the XPD API itself is not thoroughly tested. Should you find any bugs, have any questions or constructive ideas please don't be afraid to contact the author or [open an issue](https://github.com/IntergatedCircuits/STM32_XPD/issues/new).
