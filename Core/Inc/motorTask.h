/*
 * motorTask.h
 *
 *  Created on: 2026. 8. 2.
 *      Author: wowns
 */

#ifndef INC_MOTOR_TASK_H_
#define INC_MOTOR_TASK_H_

#include <stdint.h>

typedef enum
{
    MOTOR_ERROR_NONE            = 0x00,

    MOTOR_ERROR_INVALID_TARGET  = 0x01,
    MOTOR_ERROR_INVALID_AXIS    = 0x02,
    MOTOR_ERROR_ANGLE_LIMIT     = 0x03,

    MOTOR_ERROR_STEPPER_BUSY    = 0x04,
    MOTOR_ERROR_STEPPER         = 0x05,
    MOTOR_ERROR_SERVO2          = 0x06,
    MOTOR_ERROR_SERVO3          = 0x07,

    MOTOR_ERROR_HOME_NOT_SET    = 0x08,

    MOTOR_ERROR_INIT            = 0x09,
    MOTOR_ERROR_QUEUE           = 0x0A,
    MOTOR_ERROR_INVALID_COMMAND = 0x0B,

	MOTOR_ERROR_COMM_LOST       = 0x0C,
	MOTOR_ERROR_ENCODER

} MotorError_t;

typedef enum
{
    MOTOR_STATE_INIT = 0,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_ERROR
} MotorState_t;


typedef enum
{
	MOTOR_STATUS_COMMAND_DONE = 0,
    MOTOR_STATUS_ERROR,
} MotorStatus_t;

typedef struct
{
    MotorStatus_t status;
    MotorError_t error;

    uint8_t command_type;
} MotorStatusMessage_t;

extern volatile MotorState_t motorState;

void Motor_GetCommandedAngles(float *theta1Deg,
                              float *theta2Deg,
                              float *theta3Deg);

void StartMotorTask(void *argument);

#endif /* INC_MOTOR_TASK_H_ */
