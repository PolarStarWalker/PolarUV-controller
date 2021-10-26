#ifndef ROBOT_MOTORSSTRUCT_HPP
#define ROBOT_MOTORSSTRUCT_HPP
//#include <iostream>

enum DShotMode : int8_t {
    DShot150 = 1,
    DShot300 = 2,
    DShot600 = 4,
    DShot1200 = 8
};

struct MotorsStruct {
    uint16_t PacketArray[12] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};

    uint16_t PWM[4] = {};

    DShotMode TimerPrescaler = DShot300;
};

extern MotorsStruct MotorsStructData;
constexpr size_t MotorsStructArrayLength = sizeof(MotorsStructData.PacketArray);
constexpr size_t MotorsStructLen = sizeof(MotorsStructData);
constexpr size_t MotorsStructLenMessage = MotorsStructLen + 4;

//std::ostream &operator<<(std::ostream &s, const MotorsStruct &c);
#endif
