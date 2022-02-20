#ifndef CONTROLLER_MOTORS_HPP
#define CONTROLLER_MOTORS_HPP

#include <stdint.h>
#include <stddef.h>

struct MotorsStruct;

struct MotorMessageStruct;

void SetUpTimers();

struct MotorMessageStruct* GetMotorsStructBuffer();

struct MotorsStruct *Parse(const struct MotorMessageStruct* motorsMessage);

void SetMotors(const struct MotorsStruct* motors);

void SetPWM(const struct MotorsStruct* motors);

void StopMotors();

void StopPWM();

size_t GetMotorsMessageSize();

#endif
