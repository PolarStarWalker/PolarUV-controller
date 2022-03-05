#ifndef CONTROLLER_MOTORS_HPP
#define CONTROLLER_MOTORS_HPP

#include <stdint.h>
#include <stddef.h>

struct MotorsStruct;

struct MotorMessageStruct;

void SetUpTimers();

struct MotorMessageStruct* GetMotorsMessageBuffer();
size_t GetMotorsMessageSize();

struct MotorsStruct *GetMotorsStruct();

size_t Parse(const struct MotorMessageStruct* motorsMessage, struct MotorsStruct* motorsStruct);

void SetMotors(const struct MotorsStruct* motors);

void SetPWM(const struct MotorsStruct* motors);

void CleanMotorsStruct(struct MotorsStruct* motorsStruct);

#endif
