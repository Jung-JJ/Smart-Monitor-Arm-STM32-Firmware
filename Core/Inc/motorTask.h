/*
 * motorTask.h
 *
 *  Created on: 2026. 8. 2.
 *      Author: wowns
 */

#ifndef INC_MOTOR_TASK_H_
#define INC_MOTOR_TASK_H_

typedef enum
{
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_MOVING,
    MOTOR_STATE_ERROR
} MotorState_t;

typedef enum
{
    MOTOR_STATUS_MOVE_DONE = 0,
    MOTOR_STATUS_ERROR
} MotorStatus_t;



void StartMotorTask(void *argument);

#endif /* INC_MOTOR_TASK_H_ */
