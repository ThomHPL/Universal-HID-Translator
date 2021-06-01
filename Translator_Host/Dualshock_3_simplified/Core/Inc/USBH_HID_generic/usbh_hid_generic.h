/* Copyright (C) 2021 Thomas Herpoel - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * You should have received a copy of the GNU GPLv3 license with
 * this file. If not, please visit: https://www.gnu.org/licenses/gpl-3.0.fr.html
 */


#ifndef __USBH_HID_GENERIC_H
#define __USBH_HID_GENERIC_H

#include <USBH_HID_generic/usbh_hid.h>

typedef struct {
  char name[32];
  uint16_t vid;
  uint16_t pid;
  USBH_StatusTypeDef(*DeviceSpecificInit)(struct _USBH_HandleTypeDef *phost);
} USBH_HID_DriverTypeDef;

USBH_StatusTypeDef USBH_HID_GenericInit(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef USBH_HID_GetRawReport(USBH_HandleTypeDef *phost, uint8_t* report);
USBH_StatusTypeDef USBH_HID_DeviceSpecificInit(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef USBH_HID_GenericRegisterDriver(USBH_HID_DriverTypeDef driver);

#endif
