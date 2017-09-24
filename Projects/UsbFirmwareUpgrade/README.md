# USB Firmware Upgrade Project

This project implements a firmware upgrade (bootloader) solution using the USB *Device Firmware Upgrade Class* (DFU), which provides an interface to up- and download (executable) images to the device memory. The project implements the same DFU functionality as the **System Memory Boot mode** permanently programmed in all USB capable STM32 devices, therefore the following guide applies to DFU firmware upgrade in general for the STM32 family.

The project is provided with a number of Board Support Packages, which extend the applicability of the project over the whole supported product range.

## Project structure

The project uses the XPD drivers that belong to the selected STM32 line (e.g. for the STM32F3-Discovery the [STM32F3_XPD](https://github.com/IntergatedCircuits/STM32_XPD/tree/master/STM32F3_XPD) and its dependencies in [CMSIS](https://github.com/IntergatedCircuits/STM32_XPD/tree/master/CMSIS)). The STMicroelectronics provided (but slightly improved) [STM32_USB_Device_Library](https://github.com/IntergatedCircuits/STM32_XPD/tree/master/Middlewares/STM32_USB_Device_Library) is used as the USB Core and DFU class stack.

The **App** folder contains the project-specific USB configuration, as well as the embedded flash media interface (*usb_dfu_flash.c*).

The **BSP_{X}** folders each contain a Board Support Package which make the project out-of-the-box evaluatable for the given STM32 kit. This code also gives insight into how to efficiently separate the device-specific configuration from the application-specific peripheral handling.

## Drivers and Software

The STM32 devices DFU driver package – the [DfuSe](http://www.st.com/en/development-tools/stsw-stm32080.html) – contains not only the necessary drivers, but also a freely modifiable software toolkit for simple device flashing using the USB interface. This includes DFU file generator and DFU flasher GUI and command line applications.

### Firmware Upgrade Procedure

1. Start the *DfuSeDemo.exe* application. It automatically detects when a DFU device is connected to the computer, and displays device information.
2. Open the DFU File Manager (*DfuFileMgr.exe*) application and select **GENERATE**.
3. The device data shall be copied from the device information displayed on the DfuSeDemo as well as the *Target ID*. The result of a built firmware should be *.hex* or *.bin* file, which have to be injected here. A descriptive *Target Name* can also be specified.
4. Press *Generate* and save the *.dfu* file.
5. In the DfuSeDemo, choose the previously created *.dfu* file in the *Upgrade or Verify Action* section, then press **Upgrade**, and wait for the procedure to be completed.