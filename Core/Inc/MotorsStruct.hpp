#ifndef CONTROLLER_MOTORSSTRUCT_HPP
#define CONTROLLER_MOTORSSTRUCT_HPP

#include "tim.h"
#include <array>
#include <cstring>

constexpr uint32_t neutral = 1000;

constexpr size_t MotorsSize = 12;
constexpr size_t PWMSize = 4;

template<typename Type>
using MotorsArray = std::array<Type, MotorsSize>;

template<typename Type>
using PWMArray = std::array<Type, PWMSize>;

struct MotorsStruct {

    enum DShotMode : int8_t {
        DShot150 = 1,
        DShot300 = 2,
        DShot600 = 4,
        DShot1200 = 8
    };

    MotorsArray<uint16_t> PacketArray = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

    PWMArray<uint16_t> PWM = {};

    DShotMode TimerPrescaler = DShot300;

};


struct MotorMessageStruct{

    static constexpr size_t MotorsStructLen = sizeof(MotorsStruct);
    static constexpr size_t MotorsMessageLen = 2 * (sizeof(MotorsStruct) + 4);

    inline MotorsStruct* Parse(MotorsStruct* motors)  const {

        for (size_t i = 0; i < MotorsMessageLen; i++) {
            if (buffer_[i] == 's' && buffer_[i + 1] == 's' && buffer_[i + 2] == 's' && buffer_[i + 3] == 's') {
                memcpy(motors, &buffer_[i + 4], MotorsStructLen);
                return motors;
            }
        }

        return nullptr;
    }

private:
    std::array<uint8_t, MotorsMessageLen> buffer_;


};


struct TimersStruct {

    static constexpr float ACCELERATION_VALUE = 0.01;

    inline void SetMotors(const MotorsStruct *motors) {
        for (size_t i = 0; i < MotorsSize; i++)
            *(MotorTimers[i]) = CalculateTickValue(motors->PacketArray[i]);
    }

    inline void SetPWM(const MotorsStruct *motors) {
        for (size_t i = 0; i < PWMSize; i++)
            *(PWMTimers[i]) = motors->PWM[i];
    }

    inline void StopMotors(){
        for (size_t i = 0; i < MotorsSize; i++)
            *(MotorTimers[i]) = CalculateTickValue(neutral);
    }

    inline void StopPWM(){
        for (size_t i = 0; i < PWMSize; i++)
            *(PWMTimers[i]) = 0;
    }

    static inline uint32_t CalculateTickValue(uint32_t gas) {
        return gas * 6 + 9000;
    }

private:

    MotorsArray<__IO uint32_t *> MotorTimers{&(TIM1->CCR1), &(TIM1->CCR2), &(TIM1->CCR3), &(TIM1->CCR4),
                                             &(TIM3->CCR1), &(TIM3->CCR2), &(TIM3->CCR3), &(TIM3->CCR4),
                                             &(TIM4->CCR1), &(TIM4->CCR2), &(TIM4->CCR3), &(TIM4->CCR4)};

    MotorsArray<float> Velocity{};
    MotorsArray<float> Acceleration{};

    PWMArray<__IO uint32_t *> PWMTimers{&(TIM2->CCR1), &(TIM2->CCR2), &(TIM2->CCR3), &(TIM2->CCR4)};
};

#endif
