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

#include <stdio.h>
#include <string.h>
#include "usart.h"
char uart_buf[64];
int uart_len;

#define ENCODER_TASK_PERIOD_MS    20U

volatile float currentEncoderAngleDeg = 0.0f;
volatile AS5600_Status_t encoderStatus = AS5600_ERROR_I2C;
volatile uint32_t encoderReadCount = 0U;
volatile uint32_t encoderErrorCount = 0U;

volatile uint32_t encoderHalError = HAL_I2C_ERROR_NONE;
volatile HAL_I2C_StateTypeDef encoderI2cState;

void StartEncoderTask(void *argument)
{
    float angle_deg;
    AS5600_Status_t status;

    (void)argument;

    for (;;)
    {
        status = AS5600_ReadAngleDeg(&angle_deg);

        encoderStatus = status;

        if (status == AS5600_OK)
        {
            currentEncoderAngleDeg = angle_deg;
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
