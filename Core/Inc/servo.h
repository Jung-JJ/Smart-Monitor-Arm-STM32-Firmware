/*
 * servo.h
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>

typedef enum
{
    SERVO_CHANNEL_THETA2 = 0,
    SERVO_CHANNEL_THETA3
} ServoChannel_t;

typedef enum
{
    SERVO_OK = 0,
    SERVO_ERROR_INVALID_CHANNEL,
    SERVO_ERROR_INVALID_ANGLE,
    SERVO_ERROR_HAL
} ServoStatus_t;

ServoStatus_t Servo_Init(void);

ServoStatus_t Servo_SetAngle(ServoChannel_t channel,
                             float angle_deg);

uint16_t Servo_AngleToPulse(float angle_deg);

#endif /* INC_SERVO_H_ */
