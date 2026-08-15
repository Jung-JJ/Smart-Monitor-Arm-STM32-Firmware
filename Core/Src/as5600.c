/*
 * as5600.c
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#include "as5600.h"

#include <stddef.h>

#include "i2c.h"

#define AS5600_I2C_ADDRESS (0x36U << 1U)

#define AS5600_RAW_ANGLE_REG     0x0CU

#define AS5600_RAW_ANGLE_MASK    0x0FFFU

#define AS5600_I2C_TIMEOUT_MS    100U
volatile uint16_t debugAS5600RawAngle = 0U;
AS5600_Status_t AS5600_ReadRawAngle(uint16_t *raw_angle){
	uint8_t rx_Data[2];
	HAL_StatusTypeDef hal_status;

	if(raw_angle == NULL){
		return AS5600_ERROR_NULL_POINTER;
	}

	hal_status = HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDRESS, AS5600_RAW_ANGLE_REG, I2C_MEMADD_SIZE_8BIT,
			rx_Data, sizeof(rx_Data), AS5600_I2C_TIMEOUT_MS);

	if(hal_status != HAL_OK){
		return AS5600_ERROR_I2C;
	}

	*raw_angle = (((uint16_t)rx_Data[0]<<8U)|((uint16_t)rx_Data[1])) & AS5600_RAW_ANGLE_MASK;
	debugAS5600RawAngle = *raw_angle;
	return AS5600_OK;
}

AS5600_Status_t AS5600_ReadAngleDeg(float *angle_deg){
	uint16_t raw_angle;
	AS5600_Status_t status;

	if(angle_deg == NULL){
		return AS5600_ERROR_NULL_POINTER;
	}

	status = AS5600_ReadRawAngle(&raw_angle);

	if(status != AS5600_OK){
		return status;
	}

	*angle_deg = ((float)raw_angle * 360.0f)/ 4096.0f;

	return AS5600_OK;
}

