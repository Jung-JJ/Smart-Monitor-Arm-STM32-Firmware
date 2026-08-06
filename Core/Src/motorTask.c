/*
 * motorTask.c
 *
 *  Created on: 2026. 8. 2.
 *      Author: wowns
 */


#include "motorTask.h"
#include "stepper.h"
#include "cmsis_os.h"
#include "motorCommand.h"
#include "servo.h"

extern osMessageQueueId_t motorCommandQueueHandle;
extern osMessageQueueId_t motorStatusQueueHandle;


#define THETA1_MIN_X10   (-900)
#define THETA1_MAX_X10    900

#define THETA2_MIN_X10   (-1350)
#define THETA2_MAX_X10   1350

#define THETA3_MIN_X10   (-1350)
#define THETA3_MAX_X10   1350

/* 디버깅용 변수 */
volatile int16_t debugTheta1 = 0;
volatile int16_t debugTheta2 = 0;
volatile int16_t debugTheta3 = 0;

volatile float debugStepperMoveAngleDeg = 0.0f;

volatile MotorState_t motorState = MOTOR_STATE_IDLE;

volatile StepperStatus_t debugStepperStatus = STEPPER_OK;
volatile ServoStatus_t debugServo2Status = SERVO_OK;
volatile ServoStatus_t debugServo3Status = SERVO_OK;

static uint8_t Motor_CheckTarget(const MotorCommand_t *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    if ((command->theta1_x10 < THETA1_MIN_X10) || (command->theta1_x10 > THETA1_MAX_X10))
    {
        return 0U;
    }

    if ((command->theta2_x10 < THETA2_MIN_X10) || (command->theta2_x10 > THETA2_MAX_X10))
    {
        return 0U;
    }

    if ((command->theta3_x10 < THETA3_MIN_X10) || (command->theta3_x10 > THETA3_MAX_X10))
    {
        return 0U;
    }

    return 1U;
}

void StartMotorTask(void *argument)
{
    MotorCommand_t command;
    float theta1Deg;
    float theta2Deg;
    float theta3Deg;

    (void)argument;
    Stepper_Init();

    if (Servo_Init() != SERVO_OK)
    {
        motorState = MOTOR_STATE_ERROR;
    }
    else
    {
        motorState = MOTOR_STATE_IDLE;
    }

    motorState = MOTOR_STATE_IDLE;

    for (;;)
    {
        if (osMessageQueueGet(motorCommandQueueHandle,
                              &command,
                              NULL,
                              osWaitForever) != osOK)
        {
           motorState = MOTOR_STATE_ERROR;
           continue;
        }

        switch(command.type){
        case MOTOR_COMMAND_SET_TARGET:
        	if(Motor_CheckTarget(&command) == 0U){
        		motorState = MOTOR_STATE_ERROR;
        		break;
        	}

        	if (Stepper_IsBusy() != 0U){
        		debugStepperStatus = STEPPER_ERROR_BUSY;
        	    motorState = MOTOR_STATE_ERROR;
        	    break;
        	}

        	debugTheta1 = command.theta1_x10;
        	debugTheta2 = command.theta2_x10;
        	debugTheta3 = command.theta3_x10;

        	theta2Deg = (float)command.theta2_x10 / 10.0f;
        	theta3Deg = (float)command.theta3_x10 / 10.0f;
        	theta1Deg = (float)command.theta1_x10 / 10.0f;

        	debugStepperMoveAngleDeg = theta1Deg;

        	if(theta1Deg != 0.0f){
        		debugStepperStatus = Stepper_MoveRelative(theta1Deg);
        		if(debugStepperStatus != STEPPER_OK){
        			motorState = MOTOR_STATE_ERROR;
        			break;
        		}
        	}
        	else{
        		debugStepperStatus = STEPPER_OK;
        	}

        	debugServo2Status = Servo_SetAngle(SERVO_CHANNEL_THETA2, theta2Deg);

        	if(debugServo2Status != SERVO_OK){
        		motorState = MOTOR_STATE_ERROR;
        		break;
        	}

        	debugServo3Status = Servo_SetAngle(SERVO_CHANNEL_THETA3,theta3Deg);

        	if(debugServo3Status != SERVO_OK){
        		motorState = MOTOR_STATE_ERROR;
        		break;
        	}

        	while(Stepper_IsBusy() != 0U){
        		osDelay(1U);
        	}

        	motorState = MOTOR_STATE_IDLE;
        	MotorState_t status = MOTOR_STATUS_MOVE_DONE;

        	osMessageQueuePut(motorStatusQueueHandle, &status, 0U, 0U);

        	break;
        case MOTOR_COMMAND_SET_HOME:
        	//여기에는 통신으로 받아온 각도를 넣어야함.
        	motorState = MOTOR_STATE_IDLE;

        	break;

        case MOTOR_COMMAND_MOVE_HOME:
        	//여기에는 우리가 가져온 각도에 가라고 명령.
        	motorState = MOTOR_STATE_IDLE;

        	break;

        default:
        	motorState = MOTOR_STATE_ERROR;

        	break;
        }
    }
}
