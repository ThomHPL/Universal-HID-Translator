/* Copyright (C) 2021 Thomas Herpoel - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * You should have received a copy of the GNU GPLv3 license with
 * this file. If not, please visit: https://www.gnu.org/licenses/gpl-3.0.fr.html
 */


#include "USBH_HID_generic/usbh_hid_generic.h"
#include "USBH_HID_generic/usbh_hid_parser.h"

// buffer to store the last received report
uint8_t	rx_report_buf[128];

const USBH_HID_DriverTypeDef  null_Driver =
{
		"null driver",
		0x0000,
		0x0000,
		NULL,
};

// TODO: use a linked list to store the registered drivers?
#define MAX_DRIVERS 4 							// max number of registered drivers
USBH_HID_DriverTypeDef drivers[MAX_DRIVERS]; 	// array containing the drivers
USBH_HID_DriverTypeDef current_driver;			// currently used driver (for the currently connected device)
uint8_t n_drivers = 0;							// current amount of registered drivers

/**
 * @brief  USBH_HID_GenericRegisterDriver
 *         The function registers a specific device driver to be used when the corresponding device is connected.
 *         It is added to an array of drivers available.
 * @param  driver: driver to be loaded
 * @retval USBH Status
 */
USBH_StatusTypeDef USBH_HID_GenericRegisterDriver(USBH_HID_DriverTypeDef driver)
{
	if (n_drivers == (MAX_DRIVERS-1))
	{
		return USBH_FAIL;
	}

	drivers[n_drivers] = driver;
	n_drivers += 1;
	return USBH_OK;
}

/**
 * @brief  USBH_HID_GenericInit
 *         The function init the HID generic device.
 * @param  phost: Host handle
 * @retval USBH Status
 */
USBH_StatusTypeDef USBH_HID_GenericInit(USBH_HandleTypeDef *phost)
{
	uint32_t i;
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

	for (i = 0U; i < (sizeof(rx_report_buf) / sizeof(uint8_t)); i++)
	{
		rx_report_buf[i] = 0U;
	}

	if (HID_Handle->length > sizeof(rx_report_buf))
	{
		HID_Handle->length = sizeof(rx_report_buf);
	}

	HID_Handle->pData = (uint8_t *)(void *)rx_report_buf;

	USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, 2 * sizeof(rx_report_buf));

	USBH_StatusTypeDef status = USBH_NOT_SUPPORTED;
	// loads the null driver
	current_driver = null_Driver;
	// looks for a compatible driver amongst the registered drivers
	for(uint8_t i = 0; i < n_drivers; i++)
	{
		if((phost->device.DevDesc.idVendor == drivers[i].vid) && (phost->device.DevDesc.idProduct == drivers[i].pid))
		{
			current_driver = drivers[i];
			status = current_driver.DeviceSpecificInit(phost);
		}
		else
		{
			status = USBH_NOT_SUPPORTED;
		}
	}

	return status;
	//return USBH_HID_DeviceSpecificInit(phost);
}

/**
 * @brief  USBH_HID_GetRawReport
 *         The function gives the next raw HID report in the fifo
 * @param  phost: Host handle
 * @param  report: pointer to the raw report
 *
 * @retval USBH status
 */
USBH_StatusTypeDef USBH_HID_GetRawReport(USBH_HandleTypeDef *phost, uint8_t* report)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

	if (HID_Handle->length == 0U)
	{
		return USBH_FAIL;
	}
	/*Fill report */
	if(USBH_HID_FifoRead(&HID_Handle->fifo, report, HID_Handle->length) !=  HID_Handle->length)
	{
		return USBH_FAIL;
	}
	return USBH_OK;
}

//__weak USBH_StatusTypeDef USBH_HID_DeviceSpecificInit(USBH_HandleTypeDef *phost)
//{
//	/* Prevent unused argument(s) compilation warning */
//	UNUSED(phost);
//	return USBH_OK;
//}
