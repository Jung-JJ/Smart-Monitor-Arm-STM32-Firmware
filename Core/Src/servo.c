/*
 * servo.c
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */


#include "servo.h"

#include "tim.h"

#define SERVO_MIN_ANGLE_DEG      (-135.0f)
#define SERVO_MAX_ANGLE_DEG      135.0f

#define SERVO_MIN_PULSE_US       500U
#define SERVO_NEUTRAL_PULSE_US   1500U
#define SERVO_MAX_PULSE_US       2500U

#define SERVO_THETA2_CHANNEL     TIM_CHANNEL_1
#define SERVO_THETA3_CHANNEL     TIM_CHANNEL_2

uint16_t Servo_AngleToPulse(float angle_deg)
{
    float real_angle_deg;
	float pulse_us;

    if (angle_deg <= SERVO_MIN_ANGLE_DEG)
    {
        return SERVO_MIN_PULSE_US;
    }

    if (angle_deg >= SERVO_MAX_ANGLE_DEG)
    {
        return SERVO_MAX_PULSE_US;
    }

    real_angle_deg = 135.0f + angle_deg;

    pulse_us = 500.0f + (real_angle_deg / 270.0f)*2000.0f;

    return (uint16_t)(pulse_us + 0.5f);
}

ServoStatus_t Servo_Init(void)
{
    HAL_StatusTypeDef status;

    __HAL_TIM_SET_COMPARE(&htim4,
                          SERVO_THETA2_CHANNEL,
                          SERVO_NEUTRAL_PULSE_US);

    __HAL_TIM_SET_COMPARE(&htim4,
                          SERVO_THETA3_CHANNEL,
                          SERVO_NEUTRAL_PULSE_US);

    status =
        HAL_TIM_PWM_Start(&htim4,
                          SERVO_THETA2_CHANNEL);

    if (status != HAL_OK)
    {
        return SERVO_ERROR_HAL;
    }

    status =
        HAL_TIM_PWM_Start(&htim4,
                          SERVO_THETA3_CHANNEL);

    if (status != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(&htim4,
                               SERVO_THETA2_CHANNEL);

        return SERVO_ERROR_HAL;
    }

    return SERVO_OK;
}

ServoStatus_t Servo_SetAngle(ServoChannel_t channel,
                             float angle_deg)
{
    uint16_t pulse_us;
    uint32_t timer_channel;

    if ((angle_deg < SERVO_MIN_ANGLE_DEG) ||
        (angle_deg > SERVO_MAX_ANGLE_DEG))
    {
        return SERVO_ERROR_INVALID_ANGLE;
    }

    if (channel == SERVO_CHANNEL_THETA2)
    {
        timer_channel = SERVO_THETA2_CHANNEL;
    }
    else if (channel == SERVO_CHANNEL_THETA3)
    {
        timer_channel = SERVO_THETA3_CHANNEL;
    }
    else
    {
        return SERVO_ERROR_INVALID_CHANNEL;
    }

    pulse_us = Servo_AngleToPulse(angle_deg);

    __HAL_TIM_SET_COMPARE(&htim4,
                          timer_channel,
                          pulse_us);

    return SERVO_OK;
}
