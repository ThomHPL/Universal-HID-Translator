/* Copyright (C) 2021 Thomas Herpoel - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * You should have received a copy of the GNU GPLv3 license with
 * this file. If not, please visit: https://www.gnu.org/licenses/gpl-3.0.fr.html
 */

#include "USBH_HID_generic/usbh_hid.h"
#include "USBH_HID_generic/usbh_hid_generic.h"

#include "Dualshock3_driver.h"

static USBH_StatusTypeDef Dualshock3Init(USBH_HandleTypeDef *phost);

USBH_HID_DriverTypeDef  dualshock3_Driver =
{
		"Dualshock 3",
		0x054C,
		0x0268,
		Dualshock3Init,
};

DS3_report ds3report;

/**
 * @brief  	Dualshock 3 init: this function sets features in the device to start the HID reporting
 * @param  	phost: pointer the the usbh handler
 * @retval	usbh status
 */
static USBH_StatusTypeDef Dualshock3Init(USBH_HandleTypeDef *phost)
{
	USBH_StatusTypeDef status = USBH_BUSY;
	int i = 0;
	while((status != USBH_OK )||(i>=5))
	{
		if(i>=5)
		{
			return USBH_BUSY;
		}
		uint8_t magic[] = {0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0xA0, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		status = USBH_HID_SetReport(phost, 0x03, 0xEF, magic, 48);
	}

	status = USBH_BUSY;
	i = 0;
	while((status != USBH_OK )||(i>=5))
	{
		if(i>=5)
		{
			return USBH_BUSY;
		}
		uint8_t magic[] = {0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0xB0, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		status = USBH_HID_SetReport(phost, 0x03, 0xEF, magic, 48);
	}
	status = USBH_BUSY;
	i = 0;
	while((status != USBH_OK )||(i>=5))
	{
		if(i>=5)
		{
			return USBH_BUSY;
		}
		uint8_t magic[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		status = USBH_HID_SetReport(phost, 0x02, 0x01, magic, 48);
	}

	status = USBH_BUSY;
	i = 0;
	while((status != USBH_OK )||(i>=5))
	{
		if(i>=5)
		{
			return USBH_BUSY;
		}
		uint8_t magic[] = {0x42, 0x0c, 0x00, 0x00};
		status = USBH_HID_SetReport(phost, 0x03, 0xF4, magic, 4);
	}

	status = USBH_BUSY;
	i = 0;
	while((status != USBH_OK )||(i>=5))
	{
		if(i>=5)
		{
			return USBH_BUSY;
		}
		uint8_t magic[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x02, 0xff, 0x27, 0x10, 0x00, 0x32, 0xff,
				0x27, 0x10, 0x00, 0x32, 0xff, 0x27, 0x10, 0x00,
				0x32, 0xff, 0x27, 0x10, 0x00, 0x32, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		};
		status = USBH_HID_SetReport(phost, 0x02, 0x01, magic, 48);
	}
	Dualshock3_connected_CB();
	return status;
}

/**
 * @brief  	This function decodes the raw 10 bits from an accelerometer or
 * 			magnetometer value into a signed 16 bits integer
 * @param  	raw_HI: 2 highest bits of the value
 * @param  	raw_LO: 8 lowest bits of the value
 * @retval	decoded value int a signed 16 bits integer
 */
int16_t decodeRawData(uint8_t raw_HI, uint8_t raw_LO)
{
	return (((uint16_t)raw_HI<<8) | raw_LO) - 512;
}

/**
 * Callback from usbh_hid library, called when an HID report is added to the fifo
 */
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
	USBH_HID_GetRawReport(phost, (uint8_t*)&ds3report);
	Dualshock3_newReport_CB(&ds3report);
}

/**
 * @brief	connected callback: called when a dualshock 3 has been configured and connected
 * @attention	Not yet implemented!
 */
__weak void Dualshock3_connected_CB(void)
{
}

/**
 * @brief  	new report callback: called when a new report was received
 * @param  	report: pointer to a DS3_report data struct containing the received report
 */
__weak void Dualshock3_newReport_CB(DS3_report* report)
{
	UNUSED(report);
}
