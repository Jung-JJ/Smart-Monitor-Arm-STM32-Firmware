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

#define MOTOR_INIT_THETA1_DEG    10.0f
#define MOTOR_INIT_THETA2_DEG    10.0f
#define MOTOR_INIT_THETA3_DEG    10.0f

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

static uint8_t Motor_Init(void)
{
    Stepper_Init();

    if (Servo_Init() != SERVO_OK)
    {
        return 0U;
    }

    if (Servo_SetAngle(SERVO_CHANNEL_THETA2, MOTOR_INIT_THETA2_DEG) != SERVO_OK)
    {
        return 0U;
    }

    if (Servo_SetAngle(SERVO_CHANNEL_THETA3, MOTOR_INIT_THETA3_DEG) != SERVO_OK)
    {
        return 0U;
    }

    //주의할점 지금 엔코더가 없으니 시작 각도는 안 움직이게 할꺼임.
    commandedTheta1Deg = MOTOR_INIT_THETA1_DEG;
    commandedTheta2Deg = MOTOR_INIT_THETA2_DEG;
    commandedTheta3Deg = MOTOR_INIT_THETA3_DEG;

    return 1U;
}

static uint8_t Motor_IsAngleInRange(MotorAxis_t axis, float angleDeg)
{
    switch (axis)
    {
        case MOTOR_AXIS_THETA1:
            return (angleDeg >= ((float)THETA1_MIN_X10 / 10.0f)) && (angleDeg <= ((float)THETA1_MAX_X10 / 10.0f));

        case MOTOR_AXIS_THETA2:
            return (angleDeg >= ((float)THETA2_MIN_X10 / 10.0f)) && (angleDeg <= ((float)THETA2_MAX_X10 / 10.0f));

        case MOTOR_AXIS_THETA3:
            return (angleDeg >= ((float)THETA3_MIN_X10 / 10.0f)) && (angleDeg <= ((float)THETA3_MAX_X10 / 10.0f));

        default:
            return 0U;
    }
}

static void Motor_ReportError(MotorError_t error)
{
	MotorStatusMessage_t message;

	motorState = MOTOR_STATE_ERROR;

	message.status = MOTOR_STATUS_ERROR;
	message.error = error;
	message.command_type = 0U;

    (void)osMessageQueuePut(motorStatusQueueHandle, &message, 0U, 0U);
}

static void Motor_ReportCommandDone(uint8_t commandType)
{
    MotorStatusMessage_t message;

    message.status = MOTOR_STATUS_COMMAND_DONE;
    message.error = MOTOR_ERROR_NONE;
    message.command_type = commandType;

    if (osMessageQueuePut(motorStatusQueueHandle, &message, 0U, 0U) != osOK){
        motorState = MOTOR_STATE_ERROR;
    }
}

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

    motorState = MOTOR_STATE_INIT;

    if (Motor_Init() == 0U)
    {
    	Motor_ReportError(MOTOR_ERROR_INIT);
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
           Motor_ReportError(MOTOR_ERROR_QUEUE);
           continue;
        }

        switch(command.type){
			case MOTOR_COMMAND_SET_TARGET:
			{
				if(Motor_CheckTarget(&command) == 0U){
					Motor_ReportError(MOTOR_ERROR_INVALID_TARGET);
					break;
				}

				if (Stepper_IsBusy() != 0U){
				    debugStepperStatus = STEPPER_ERROR_BUSY;
				    Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
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
						Motor_ReportError(MOTOR_ERROR_STEPPER);
						break;
					}

					motorState = MOTOR_STATE_MOVING;
				}

				else{
					debugStepperStatus = STEPPER_OK;
				}

				debugServo2Status = Servo_SetAngle(SERVO_CHANNEL_THETA2, theta2Deg);

				if(debugServo2Status != SERVO_OK){
					Motor_ReportError(MOTOR_ERROR_SERVO2);
					break;
				}
				commandedTheta2Deg = theta2Deg;

				debugServo3Status = Servo_SetAngle(SERVO_CHANNEL_THETA3,theta3Deg);

				if(debugServo3Status != SERVO_OK){
					Motor_ReportError(MOTOR_ERROR_SERVO3);
					break;
				}

				commandedTheta3Deg = theta3Deg;

				while(Stepper_IsBusy() != 0U){
					osDelay(1U);
				}

				commandedTheta1Deg = theta1Deg;
				motorState = MOTOR_STATE_IDLE;

				Motor_ReportCommandDone((uint8_t)command.type);

				break;
			}

			case MOTOR_COMMAND_SET_HOME:
			{

				homeTheta1Deg = commandedTheta1Deg;
				homeTheta2Deg = commandedTheta2Deg;
				homeTheta3Deg = commandedTheta3Deg;

				homeValid = 1U;

				motorState =  MOTOR_STATE_IDLE;

				Motor_ReportCommandDone((uint8_t)command.type);

				break;
			}

			case MOTOR_COMMAND_MOVE_HOME:
			{
				float theta1MoveDeg;

				if(homeValid == 0U){
					Motor_ReportError(MOTOR_ERROR_HOME_NOT_SET);
					break;
				}
				if(Stepper_IsBusy() != 0U){
					debugStepperStatus = STEPPER_ERROR_BUSY;
					Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
			        break;
				}

				theta1MoveDeg =homeTheta1Deg - commandedTheta1Deg;

				if (theta1MoveDeg != 0.0f)
				{
					debugStepperStatus = Stepper_MoveRelative(theta1MoveDeg);

					if (debugStepperStatus != STEPPER_OK)
					{
						Motor_ReportError(MOTOR_ERROR_STEPPER);
						break;
					}

					motorState = MOTOR_STATE_MOVING;
				}

				debugServo2Status = Servo_SetAngle(SERVO_CHANNEL_THETA2, homeTheta2Deg);

				 if (debugServo2Status != SERVO_OK){
					 Motor_ReportError(MOTOR_ERROR_SERVO2);
				     break;
				 }

				debugServo3Status = Servo_SetAngle(SERVO_CHANNEL_THETA3, homeTheta3Deg);

				if(debugServo3Status != SERVO_OK){
					 Motor_ReportError(MOTOR_ERROR_SERVO3);
				     break;
				}

				while(Stepper_IsBusy() != 0U){
					osDelay(1U);
				}

				commandedTheta1Deg = homeTheta1Deg;
				commandedTheta2Deg = homeTheta2Deg;
				commandedTheta3Deg = homeTheta3Deg;

				motorState = MOTOR_STATE_IDLE;

				Motor_ReportCommandDone((uint8_t)command.type);

				break;
			}

			case MOTOR_COMMAND_JOG:
			{
				float deltaDeg;
				float targetDeg;

				deltaDeg = (float)command.delta_x10 / 10.0f;

				if (deltaDeg == 0.0f){
				        motorState = MOTOR_STATE_IDLE;
				        Motor_ReportCommandDone((uint8_t)command.type);
				        break;
				}

				switch (command.axis){
					case MOTOR_AXIS_THETA1:

						targetDeg = commandedTheta1Deg + deltaDeg;

						if (Motor_IsAngleInRange(command.axis, targetDeg) == 0U){
							Motor_ReportError(MOTOR_ERROR_ANGLE_LIMIT);
							break;
						}

						if (Stepper_IsBusy() != 0U){
							debugStepperStatus = STEPPER_ERROR_BUSY;
							Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
						    break;
						}
						debugStepperStatus = Stepper_MoveRelative(deltaDeg);
						if(debugStepperStatus != STEPPER_OK){
							Motor_ReportError(MOTOR_ERROR_STEPPER);
							break;
						}

						motorState = MOTOR_STATE_MOVING;

						while(Stepper_IsBusy()!= 0U){
							osDelay(1U);
						}
					    commandedTheta1Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;

						break;

					case MOTOR_AXIS_THETA2:

						targetDeg = commandedTheta2Deg + deltaDeg;

						if (Motor_IsAngleInRange(command.axis, targetDeg) == 0U){
							Motor_ReportError(MOTOR_ERROR_ANGLE_LIMIT);
							break;
						}

						debugServo2Status = Servo_SetAngle(SERVO_CHANNEL_THETA2, targetDeg);

						if(debugServo2Status != SERVO_OK){
							Motor_ReportError(MOTOR_ERROR_SERVO2);
							break;
						}

						commandedTheta2Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;
						break;

					case MOTOR_AXIS_THETA3:

						targetDeg = commandedTheta3Deg + deltaDeg;

						if (Motor_IsAngleInRange(command.axis, targetDeg) == 0U){
							Motor_ReportError(MOTOR_ERROR_ANGLE_LIMIT);
							break;
						}

						debugServo3Status = Servo_SetAngle(SERVO_CHANNEL_THETA3, targetDeg);

						if(debugServo3Status != SERVO_OK){
							Motor_ReportError(MOTOR_ERROR_SERVO3);
							break;
						}

						commandedTheta3Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;
						break;

					default:
						Motor_ReportError(MOTOR_ERROR_INVALID_AXIS);
						break;
				}

				if (motorState == MOTOR_STATE_IDLE){
				    Motor_ReportCommandDone((uint8_t)command.type);
				}

				break;
			}

			default:
				Motor_ReportError(MOTOR_ERROR_INVALID_COMMAND);

				break;
        }
    }
}
