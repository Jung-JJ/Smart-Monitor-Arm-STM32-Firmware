/*
 * encoderTask.c
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#include "encoderTask.h"

#include "cmsis_os.h"
#include "as5600.h"
#include "i2c.h"


#define ENCODER_TASK_PERIOD_MS    20U

#define THETA1_ZERO_OFFSET_DEG    78.75f //측정 후 병경

volatile float currentEncoderAbsoluteAngleDeg = 0.0f;
volatile float currentEncoderAngleDeg = 0.0f;

volatile AS5600_Status_t encoderStatus = AS5600_ERROR_I2C;
volatile uint32_t encoderReadCount = 0U;
volatile uint32_t encoderErrorCount = 0U;

volatile uint32_t encoderHalError = HAL_I2C_ERROR_NONE;
volatile HAL_I2C_StateTypeDef encoderI2cState;

static float Encoder_CalculateRelativeAngle(float absoluteAngleDeg)
{
    float relativeAngleDeg;

    relativeAngleDeg =
        absoluteAngleDeg - THETA1_ZERO_OFFSET_DEG;

    if (relativeAngleDeg > 180.0f)
    {
        relativeAngleDeg -= 360.0f;
    }
    else if (relativeAngleDeg < -180.0f)
    {
        relativeAngleDeg += 360.0f;
    }
    return relativeAngleDeg;
}

void StartEncoderTask(void *argument)
{
	float absoluteAngleDeg;
    AS5600_Status_t status;

    (void)argument;

    for (;;)
    {
        status = AS5600_ReadAngleDeg(&absoluteAngleDeg);

        encoderStatus = status;

        if (status == AS5600_OK)
        {
        	currentEncoderAbsoluteAngleDeg = absoluteAngleDeg; //절대각
            currentEncoderAngleDeg = Encoder_CalculateRelativeAngle(absoluteAngleDeg); //상대각 (매핑)
            encoderReadCount++;
        }
        else
        {
            encoderErrorCount++;
            encoderHalError = HAL_I2C_GetError(&hi2c1);
            encoderI2cState = HAL_I2C_GetState(&hi2c1);
        }

        osDelay(ENCODER_TASK_PERIOD_MS);
    }
}
