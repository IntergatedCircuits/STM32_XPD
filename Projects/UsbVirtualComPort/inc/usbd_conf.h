/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @author  Benedek Kupper
  * @version V0.1
  * @date    2017-03-22
  * @brief   STM32 eXtensible Peripheral Drivers USB Device Module
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
#ifndef __USBD_CONF_H_
#define __USBD_CONF_H_

#if (USBD_DEBUG_LEVEL > 0)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif
#include "xpd_usb.h"
#include "xpd_utils.h"
#include "usbd_def.h"

/** @addtogroup USBD_OTG_DRIVER
  * @{ */

/** @defgroup USBD_CONF
  * @brief usb otg low level driver configuration file
  * @{ */

/** @defgroup USBD_CONF_Exported_Defines
  * @{ */

#define USBD_MAX_NUM_INTERFACES             1
#define USBD_MAX_NUM_CONFIGURATION          1
#define USBD_MAX_STR_DESC_SIZ               0x100
#define USBD_SUPPORT_USER_STRING            0
#define USBD_SELF_POWERED                   0
#define USBD_MAX_POWER_mA                   100
#define USBD_DEBUG_LEVEL                    0
#define USBD_CDC_INTERVAL                   1000
#define MAX_STATIC_ALLOC_SIZE               1000

/* #define for FS and HS identification */
#define DEVICE_FS       0
#ifdef USB_OTG_HS
#define DEVICE_HS       1
#endif

/** @defgroup USBD_Exported_Macros
  * @{ */

/* Memory management macros */
#define USBD_malloc(SIZE)       (USBD_static_malloc(SIZE))
#define USBD_free(PTR)          ((void)0)

#define USBD_Delay(MS)                          \
    (XPD_Delay_ms(MS))

#define USBD_LL_DeInit(PDEV)                    \
    (XPD_USB_Deinit((PDEV)->pData), USBD_OK)

#define USBD_LL_Start(PDEV)                     \
    (XPD_USB_Start((PDEV)->pData), USBD_OK)

#define USBD_LL_Stop(PDEV)                      \
    (XPD_USB_Stop((PDEV)->pData), USBD_OK)

#define USBD_LL_OpenEP(PDEV, EA, TYPE, MPS)     \
    (XPD_USB_EP_Open((PDEV)->pData, EA, TYPE, MPS), USBD_OK)

#define USBD_LL_CloseEP(PDEV, EA)               \
    (XPD_USB_EP_Close((PDEV)->pData, EA), USBD_OK)

#define USBD_LL_FlushEP(PDEV, EA)               \
    (XPD_USB_EP_Flush((PDEV)->pData, EA), USBD_OK)

#define USBD_LL_StallEP(PDEV, EA)               \
    (XPD_USB_EP_SetStall((PDEV)->pData, EA), USBD_OK)

#define USBD_LL_IsStallEP(PDEV, EA)             \
    (((EA) > 0x7F) ? ((USB_HandleType*)(PDEV)->pData)->EP.IN[(EA) & 0x7F].Stalled \
                   : ((USB_HandleType*)(PDEV)->pData)->EP.OUT[EA].Stalled )

#define USBD_LL_ClearStallEP(PDEV, EA)          \
    (XPD_USB_EP_ClearStall((PDEV)->pData, EA), USBD_OK)

#define USBD_LL_SetUSBAddress(PDEV, AD)         \
    (XPD_USB_SetAddress((PDEV)->pData, AD), USBD_OK)

#define USBD_LL_Transmit(PDEV, EA, BUF, SIZE)   \
    (XPD_USB_EP_Transmit((PDEV)->pData, EA, BUF, SIZE), USBD_OK)

#define USBD_LL_PrepareReceive(PDEV, EA, BUF, SIZE) \
    (XPD_USB_EP_Receive((PDEV)->pData, EA, BUF, SIZE), USBD_OK)

#define USBD_LL_GetRxDataSize(PDEV, EA)         \
    ((uint32_t)XPD_USB_EP_GetRxCount((PDEV)->pData, EA))

/* For footprint reasons and since only one allocation is handled in the CDC class
   driver, the malloc/free is changed into a static allocation method */
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

/* DEBUG macros */
#if (USBD_DEBUG_LEVEL > 0)
#define  USBD_UsrLog(...)   printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_UsrLog(...)
#endif


#if (USBD_DEBUG_LEVEL > 1)

#define  USBD_ErrLog(...)   printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_ErrLog(...)
#endif


#if (USBD_DEBUG_LEVEL > 2)
#define  USBD_DbgLog(...)   printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBD_DbgLog(...)
#endif

#endif /* __USBD_CONF_H_ */
