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


static float commandedTheta1Deg = 0.0f; //엔코더 달면 제거
static float commandedTheta2Deg = 0.0f;
static float commandedTheta3Deg = 0.0f;

static float homeTheta1Deg = 0.0f;
static float homeTheta2Deg = 0.0f;
static float homeTheta3Deg = 0.0f;

static uint8_t homeValid = 0U;

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
void Motor_GetCommandedAngles(float *theta1Deg,
                              float *theta2Deg,
                              float *theta3Deg)
{
    if (theta1Deg != NULL)
    {
        *theta1Deg = commandedTheta1Deg;
    }

    if (theta2Deg != NULL)
    {
        *theta2Deg = commandedTheta2Deg;
    }

    if (theta3Deg != NULL)
    {
        *theta3Deg = commandedTheta3Deg;
    }
}
void StartMotorTask(void *argument)
{
    MotorCommand_t command;
    float theta1Deg;
    float theta2Deg;
    float theta3Deg;
    float theta1MoveDeg;

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
			{
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

				theta1MoveDeg = theta1Deg - commandedTheta1Deg;

				debugStepperMoveAngleDeg = theta1MoveDeg;

				if(theta1MoveDeg != 0.0f){

					debugStepperStatus = Stepper_MoveRelative(theta1MoveDeg);

					if(debugStepperStatus != STEPPER_OK){
						motorState = MOTOR_STATE_ERROR;
						break;
					}

					motorState = MOTOR_STATE_MOVING;
				}

				else{
					debugStepperStatus = STEPPER_OK;
				}

				debugServo2Status = Servo_SetAngle(SERVO_CHANNEL_THETA2, theta2Deg);

				if(debugServo2Status != SERVO_OK){
					motorState = MOTOR_STATE_ERROR;
					break;
				}
				commandedTheta2Deg = theta2Deg;

				debugServo3Status = Servo_SetAngle(SERVO_CHANNEL_THETA3,theta3Deg);

				if(debugServo3Status != SERVO_OK){
					motorState = MOTOR_STATE_ERROR;
					break;
				}

				commandedTheta3Deg = theta3Deg;

				while(Stepper_IsBusy() != 0U){
					osDelay(1U);
				}

				commandedTheta1Deg = theta1Deg;

				motorState = MOTOR_STATE_IDLE;
				MotorStatus_t status = MOTOR_STATUS_MOVE_DONE;


				if (osMessageQueuePut(motorStatusQueueHandle, &status, 0U, 0U) != osOK){
				    motorState = MOTOR_STATE_ERROR;
				}

				break;
			}

			case MOTOR_COMMAND_SET_HOME:
			{
				MotorStatus_t status;

				homeTheta1Deg = commandedTheta1Deg;
				homeTheta2Deg = commandedTheta2Deg;
				homeTheta3Deg = commandedTheta3Deg;

				homeValid = 1U;

				motorState =  MOTOR_STATE_IDLE;
				status = MOTOR_STATUS_MOVE_DONE;
				(void)osMessageQueuePut(motorStatusQueueHandle, &status, 0U, 0U);

				break;
			}

			case MOTOR_COMMAND_MOVE_HOME:
			{
				float theta1MoveDeg;
			    MotorStatus_t status;

				if(homeValid == 0U){
					motorState = MOTOR_STATE_ERROR;
					break;
				}
				if(Stepper_IsBusy() != 0U){
					debugStepperStatus = STEPPER_ERROR_BUSY;
			        motorState = MOTOR_STATE_ERROR;
			        break;
				}

				theta1MoveDeg =homeTheta1Deg - commandedTheta1Deg;

				if (theta1MoveDeg != 0.0f)
				{
					debugStepperStatus =
						Stepper_MoveRelative(theta1MoveDeg);

					if (debugStepperStatus != STEPPER_OK)
					{
						motorState = MOTOR_STATE_ERROR;
						break;
					}

					motorState = MOTOR_STATE_MOVING;
				}

				debugServo2Status = Servo_SetAngle(SERVO_CHANNEL_THETA2, homeTheta2Deg);

				 if (debugServo2Status != SERVO_OK){
					 motorState = MOTOR_STATE_ERROR;
				     break;
				 }

				debugServo3Status = Servo_SetAngle(SERVO_CHANNEL_THETA3, homeTheta3Deg);

				if(debugServo3Status != SERVO_OK){
					 motorState = MOTOR_STATE_ERROR;
				     break;
				}

				while(Stepper_IsBusy() != 0U){
					osDelay(1);
				}

				commandedTheta1Deg = homeTheta1Deg;
				commandedTheta2Deg = homeTheta2Deg;
				commandedTheta3Deg = homeTheta3Deg;

				motorState = MOTOR_STATE_IDLE;

			    status = MOTOR_STATUS_MOVE_DONE;

			    if (osMessageQueuePut(motorStatusQueueHandle, &status, 0U, 0U) != osOK)
			    {
			        motorState = MOTOR_STATE_ERROR;
			    }

				break;
			}

			case MOTOR_COMMAND_JOG:
			{
				float deltaDeg;
				float targetDeg;
				MotorStatus_t status;

				deltaDeg = (float)command.delta_x10 / 10.0f;

				if (deltaDeg == 0.0f){
				        motorState = MOTOR_STATE_IDLE;
				        break;
				}

				switch (command.axis){
					case MOTOR_AXIS_THETA1:
						if (Stepper_IsBusy() != 0U){
							debugStepperStatus = STEPPER_ERROR_BUSY;
						    motorState = MOTOR_STATE_ERROR;
						    break;
						}
						debugStepperStatus = Stepper_MoveRelative(deltaDeg);
						if(debugStepperStatus != STEPPER_OK){
							motorState = MOTOR_STATE_ERROR;
							break;
						}

						motorState = MOTOR_STATE_MOVING;

						while(Stepper_IsBusy()!= 0U){
							osDelay(1U);
						}
					    commandedTheta1Deg += deltaDeg;
						motorState = MOTOR_STATE_IDLE;

						break;

					case MOTOR_AXIS_THETA2:
						targetDeg = commandedTheta2Deg + deltaDeg;
						debugServo2Status = Servo_SetAngle(SERVO_CHANNEL_THETA2, targetDeg);

						if(debugServo2Status != SERVO_OK){
							motorState = MOTOR_STATE_ERROR;
							break;
						}

						commandedTheta2Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;
						break;

					case MOTOR_AXIS_THETA3:
						targetDeg = commandedTheta3Deg + deltaDeg;
						debugServo3Status = Servo_SetAngle(SERVO_CHANNEL_THETA3, targetDeg);

						if(debugServo3Status != SERVO_OK){
							motorState = MOTOR_STATE_ERROR;
							break;
						}

						commandedTheta3Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;
						break;

					default:
						motorState = MOTOR_STATE_ERROR;
						break;
				}

				if (motorState == MOTOR_STATE_IDLE){
					status = MOTOR_STATUS_MOVE_DONE;

					if (osMessageQueuePut(motorStatusQueueHandle, &status, 0U, 0U) != osOK){
						motorState = MOTOR_STATE_ERROR;
					}

				}
				break;
			}

			default:
				motorState = MOTOR_STATE_ERROR;

				break;
        }
    }
}
