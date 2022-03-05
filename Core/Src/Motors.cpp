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

struct MotorMessageStruct *GetMotorsMessageBuffer() {
    return &MotorMessage;
}

size_t GetMotorsMessageSize() {
    return MotorMessageStruct::MotorsMessageLen;
}

struct MotorsStruct *GetMotorsStruct(){
    return &Motors;
}

size_t Parse(const struct MotorMessageStruct* motorsMessage, struct MotorsStruct* motorsStruct) {
    return motorsMessage->Parse(motorsStruct);
}

void CleanMotorsStruct(struct MotorsStruct* motorsStruct){
    for(size_t i =0; i < MotorsSize; ++i)
        motorsStruct->PacketArray[i] = 1000;

    for(size_t i =0; i < PWMSize; ++i)
        motorsStruct->PWM[i] = 0;

}

void SetMotors(const MotorsStruct *motors) {
    Timers.SetMotors(motors);
}

void SetPWM(const MotorsStruct *motors) {
    Timers.SetPWM(motors);
}
}