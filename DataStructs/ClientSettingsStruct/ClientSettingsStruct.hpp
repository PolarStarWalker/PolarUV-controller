#ifndef CLIENT_SETTINGSSTRUCT_HPP
#define CLIENT_SETTINGSSTRUCT_HPP

#include <fstream>
#include <cstring>
#include <QString>

enum RobotAnalogActions : int8_t {
    NoAnalogAction = 0,
    MoveX = 1,
    MoveY = 2,
    MoveZ = 3,
    RotateX = 4,
    RotateY = 5,
    RotateZ = 6,
    GrabHand = 7,
    RotateHand = 8,
};

enum RobotDiscreteActions : int8_t {
    NoDiscreteAction = 0,
    TurnOn = 1,
    TurnOff = 2
};

class ClientSettingsStruct {
public:
    ClientSettingsStruct();

    ~ClientSettingsStruct();

    void Save();

    bool Load();

    int32_t gamepadID;
    QChar serverIP[16];

    int8_t dPadXActionID;
    int8_t dPadYActionID;
    int8_t leftStickXActionID;
    int8_t leftStickYActionID;
    int8_t rightStickXActionID;
    int8_t rightStickYActionID;
    int8_t leftShoulderActionID;
    int8_t rightShoulderActionID;
    int8_t leftStickPressActionID;
    int8_t rightStickPressActionID;

    int8_t rectangleActionID;
    int8_t triangleActionID;
    int8_t circleActionID;
    int8_t crossActionID;
    int8_t startActionID;
    int8_t backActionID;

    bool dPadXInverted;
    bool dPadYInverted;
    bool leftStickXInverted;
    bool leftStickYInverted;
    bool rightStickXInverted;
    bool rightStickYInverted;
    bool leftShoulderInverted;
    bool rightShoulderInverted;


};

extern ClientSettingsStruct ClientSettingsStructData;
constexpr size_t ClientSettingsStructLen = sizeof(ClientSettingsStructData);
#endif
