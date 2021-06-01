/* Copyright (C) 2021 Thomas Herpoel - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * You should have received a copy of the GNU GPLv3 license with
 * this file. If not, please visit: https://www.gnu.org/licenses/gpl-3.0.fr.html
 */

#ifndef DUALSHOCK3_DRIVER
#define DUALSHOCK3_DRIVER

extern USBH_HID_DriverTypeDef  dualshock3_Driver;

typedef struct {
	uint8_t reportID;
	uint8_t reserved1;

	uint8_t Select 	: 1; //LSb
	uint8_t L3 		: 1;
	uint8_t R3 		: 1;
	uint8_t Start 	: 1;
	uint8_t DUp 	: 1;
	uint8_t DRight 	: 1;
	uint8_t DDown 	: 1;
	uint8_t DLeft 	: 1; //MSb

	uint8_t L2 		: 1;
	uint8_t R2 		: 1;
	uint8_t L1 		: 1;
	uint8_t R1 		: 1;
	uint8_t Triangle : 1;
	uint8_t Circle 	: 1;
	uint8_t Cross 	: 1;
	uint8_t Square 	: 1;

	uint8_t PS 		: 1;
	uint8_t unused0	: 7;

	uint8_t byte5;

	uint8_t LeftStickX; 	// top left is 0
	uint8_t LeftStickY;
	uint8_t RightStickX;
	uint8_t RightStickY;

	uint32_t byte10_11 : 16;

	uint8_t DUpForce;
	uint8_t DRightForce;
	uint8_t DDownForce;
	uint8_t DLeftForce;

	uint8_t L2Force;
	uint8_t R2Force;
	uint8_t L1Force;
	uint8_t R1Force;
	uint8_t TriangleForce;
	uint8_t CircleForce;
	uint8_t CrossForce;
	uint8_t SquareForce;

	uint64_t byte24_31;
	uint64_t byte32_39;
	uint8_t byte40;

	uint8_t AccX_Raw_HI;	// 10 bits of data, unsigned, little endian
	uint8_t AccX_Raw_LO;	// centered around 512 (1024/2)
	uint8_t AccY_Raw_HI;
	uint8_t AccY_Raw_LO;
	uint8_t AccZ_Raw_HI;
	uint8_t AccZ_Raw_LO;
	uint8_t Gyro_Raw_HI;	// sense rotation around Z axis?
	uint8_t Gyro_Raw_LO;

} DS3_report;

int16_t decodeRawData(uint8_t raw_HI, uint8_t raw_LO);

void Dualshock3_connected_CB(void);
void Dualshock3_newReport_CB(DS3_report* report);

#endif
