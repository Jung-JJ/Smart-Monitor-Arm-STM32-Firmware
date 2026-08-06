/*
 * as5600.h
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include <stdint.h>


typedef enum{
	AS5600_OK = 0U,
	AS5600_ERROR_NULL_POINTER,
	AS5600_ERROR_I2C
}AS5600_Status_t;


AS5600_Status_t AS5600_ReadRawAngle(uint16_t *raw_angle);

AS5600_Status_t AS5600_ReadAngleDeg(float *angle_deg);

#endif /* INC_AS5600_H_ */
