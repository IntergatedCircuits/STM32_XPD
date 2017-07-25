# USB Virtual COM Port Project

This project implements an USB to Serial communication channel by implementing the *Communication Device Class* (CDC), which provides an interface to configure and transfer byte stream through one of the device's UART peripherals.

The project is provided with a number of Board Support Packages, which extend the applicability of the project over the whole supported product range.

## Project structure

The project uses the XPD drivers that belong to the selected STM32 line (e.g. for the STM32F4-Discovery the [STM32F4_XPD](https://github.com/IntergatedCircuits/STM32_XPD/tree/master/STM32F4_XPD) and its dependencies in [CMSIS](https://github.com/IntergatedCircuits/STM32_XPD/tree/master/CMSIS)). The STMicroelectronics provided (but slightly improved) [STM32_USB_Device_Library](https://github.com/IntergatedCircuits/STM32_XPD/tree/master/Middlewares/STM32_USB_Device_Library) is used as the USB Core and CDC class stack.

The **App** folder contains the project-specific USB configuration, as well as the main Serial interfacing implementation (*usb_cdc_if.c*). The data from USB is stored in a two-paged buffer, flow-control ensures that all data is sent over UART. The received UART data is put in a circular buffer and consumed in every ms. DMA is used in both directions to establish fast data transfer between the memory buffers and the UART peripheral. The transfer speed is fast enough to handle MBaud serial communication.

The **BSP_{X}** folders each contain a Board Support Package which make the project out-of-the-box evaluatable for the given STM32 kit. This code also gives insight into how to efficiently separate the device-specific configuration from the application-specific peripheral handling.

## Drivers and Software

The USB Serial interface is supported natively on most operating systems, therefore the programmed device should be recognized as a Serial COM port when connected. In case the device is not discovered properly the [STM32 Virtual COM Port Driver](http://www.st.com/en/development-tools/stsw-stm32102.html) is recommended for installation.