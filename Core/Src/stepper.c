#include "stepper.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "tim.h"

/*
 * 기존 하드웨어 테스트에서 사용한 논리 레벨.
 * 실제 TB6600 배선 방식에 따라 반대로 바뀔 수 있다.
 */
#define STEPPER_DIR_CW_LEVEL         GPIO_PIN_RESET //실험을 통해 반전입력인걸 알음.
#define STEPPER_DIR_CCW_LEVEL        GPIO_PIN_SET

#define STEPPER_ENABLE_LEVEL         GPIO_PIN_RESET
#define STEPPER_DISABLE_LEVEL        GPIO_PIN_SET

/*
 * 1.8도 스텝모터:
 * 360 / 1.8 = 200 full steps/rev
 *
 * TB6600 마이크로스텝 설정:
 * 1/8 microstep
 *
 * 200 × 8 = 1600 pulse/rev
 */
#define STEPPER_FULL_STEPS_PER_REV   200U
#define STEPPER_MICROSTEP_DIVISION   8U
#define STEPPER_GEAR_RATIO           40U
#define STEPPER_PULSES_PER_OUTPUT_REV		 (STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEP_DIVISION * STEPPER_GEAR_RATIO)

/*
 * TIM3 설정
 *
 * Timer clock = 90 MHz
 * PSC = 89
 * Counter clock = 1 MHz
 *
 * ARR = 1999
 * PWM frequency = 1 MHz / 2000 = 500 Hz
 *
 * CCR = 1000
 * Duty = 약 50%
 */
#define STEPPER_TIMER_CHANNEL        TIM_CHANNEL_1
#define STEPPER_PWM_COMPARE          703U

static volatile uint32_t stepCount = 0U;
static volatile uint32_t targetStepCount = 0U;
static volatile uint8_t stepperBusy = 0U;

static void Stepper_SetDirection(StepperDirection_t direction)
{
    GPIO_PinState gpioLevel;

    if (direction == STEPPER_DIRECTION_CW)
    {
        gpioLevel = STEPPER_DIR_CW_LEVEL;
    }
    else
    {
        gpioLevel = STEPPER_DIR_CCW_LEVEL;
    }

    HAL_GPIO_WritePin(DIR_GPIO_Port,
                      DIR_Pin,
                      gpioLevel);
}

void Stepper_Enable(void)
{
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, STEPPER_ENABLE_LEVEL);
}

void Stepper_Disable(void)
{
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, STEPPER_DISABLE_LEVEL);
}

uint32_t Stepper_AngleToSteps(float angle_deg)
{
    float calculatedSteps;

    if (angle_deg <= 0.0f)
    {
        return 0U;
    }

    calculatedSteps =
        (angle_deg * (float)STEPPER_PULSES_PER_OUTPUT_REV) /
        360.0f;

    /* 가장 가까운 정수 펄스 수로 반올림 */
    return (uint32_t)(calculatedSteps + 0.5f);
}


void Stepper_HandlePulseFinished(void)
{
    if (stepperBusy == 0U)
    {
        return;
    }

    stepCount++;

    if (stepCount >= targetStepCount)
    {
        (void)HAL_TIM_PWM_Stop_IT(&htim3,
                                  STEPPER_TIMER_CHANNEL);

        __HAL_TIM_SET_COMPARE(&htim3, STEPPER_TIMER_CHANNEL, 0U);

        Stepper_Disable();

        stepCount = 0U;
        targetStepCount = 0U;
        stepperBusy = 0U;
    }
}

StepperStatus_t Stepper_MoveRelative(float angle_deg)
{
    uint32_t requestedSteps;
    StepperDirection_t direction;
    HAL_StatusTypeDef halStatus;

    if (stepperBusy != 0U)
    {
        return STEPPER_ERROR_BUSY;
    }

    if (angle_deg == 0.0f)
    {
        return STEPPER_ERROR_INVALID_ANGLE;
    }

    if (angle_deg > 0.0f)
    {
        direction = STEPPER_DIRECTION_CW;
    }
    else
    {
        direction = STEPPER_DIRECTION_CCW;

        angle_deg = -angle_deg;
    }

    requestedSteps = Stepper_AngleToSteps(angle_deg);

    if (requestedSteps == 0U){
        return STEPPER_ERROR_INVALID_ANGLE;
    }

    Stepper_SetDirection(direction);

    Stepper_Enable();

    stepCount = 0U;
    targetStepCount = requestedSteps;
    stepperBusy = 1U;

    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    __HAL_TIM_SET_COMPARE(&htim3, STEPPER_TIMER_CHANNEL, STEPPER_PWM_COMPARE);

    halStatus = HAL_TIM_PWM_Start_IT(&htim3, STEPPER_TIMER_CHANNEL);

    if (halStatus != HAL_OK)
    {
    	__HAL_TIM_SET_COMPARE(&htim3, STEPPER_TIMER_CHANNEL, 0U);
        stepCount = 0U;
        targetStepCount = 0U;
        stepperBusy = 0U;

        return STEPPER_ERROR_HAL;
    }

    return STEPPER_OK;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM3) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        Stepper_HandlePulseFinished();
    }
}

void Stepper_Init(void)
{
    Stepper_Disable();

    stepCount = 0U;
    targetStepCount = 0U;
    stepperBusy = 0U;
}

uint8_t Stepper_IsBusy(void)
{
    return stepperBusy;
}

uint32_t Stepper_GetCurrentStepCount(void)
{
    return stepCount;
}

uint32_t Stepper_GetTargetStepCount(void)
{
    return targetStepCount;
}
