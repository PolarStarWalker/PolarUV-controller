#include "MotorsStruct.hpp"

#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/**********************DATA****************************/
static MotorsStruct Motors;
static TimersStruct Timers;

static MotorMessageStruct MotorMessage;


/*********************FUNCRIONS************************/
extern "C" {

#include "Motors.hpp"

void SetUpTimers() {

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_Base_Start_IT(&htim5);
}

struct MotorMessageStruct *GetMotorsStructBuffer() {
    return &MotorMessage;
}

size_t GetMotorsMessageSize() {
    return MotorMessageStruct::MotorsMessageLen;
}

struct MotorsStruct *Parse(const struct MotorMessageStruct* motorsMessage) {
    return motorsMessage->Parse(&Motors);
}

void SetMotors(const MotorsStruct *motors) {
    Timers.SetPWM(motors);
}

void SetPWM(const MotorsStruct *motors) {
    Timers.SetMotors(motors);
}

void StopMotors() {
    Timers.StopMotors();
}

void StopPWM() {
    Timers.StopPWM();
}
}