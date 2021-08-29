#include "./ClientSettingsStruct/ClientSettingsStruct.hpp"

ClientSettingsStruct::ClientSettingsStruct() {

    std::memset(this->serverIP, 0, sizeof(this->serverIP));
    this->gamepadID = -1;

    this->dPadXActionID = RotateHand;
    this->dPadYActionID = GrabHand;
    this->leftStickXActionID = MoveY;
    this->leftStickYActionID = MoveZ;
    this->rightStickXActionID = RotateZ;
    this->rightStickYActionID = MoveX;
    this->leftShoulderActionID = RotateX;
    this->rightShoulderActionID = RotateY;

    this->leftStickPressActionID = NoDiscreteAction;
    this->rightStickPressActionID = NoDiscreteAction;
    this->triangleActionID = NoDiscreteAction;
    this->crossActionID = NoDiscreteAction;
    this->rectangleActionID = NoDiscreteAction;
    this->circleActionID = NoDiscreteAction;
    this->startActionID = TurnOn;
    this->backActionID = TurnOff;

    this->dPadXInverted = false;
    this->dPadYInverted = false;
    this->leftStickXInverted = false;
    this->leftStickYInverted = false;
    this->rightStickXInverted = false;
    this->rightStickYInverted = false;
    this->leftShoulderInverted = false;
    this->rightShoulderInverted = false;

}

ClientSettingsStruct::~ClientSettingsStruct() = default;

void ClientSettingsStruct::Save() {
    std::ofstream file("settings", std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);
    file.write((char *) this, ClientSettingsStructLen);
    file.close();
}

bool ClientSettingsStruct::Load() {
    std::ifstream file("settings", std::ios_base::in | std::ios_base::binary);
    if (!file) return false;
    file.read((char *) this, ClientSettingsStructLen);
    file.close();
    return true;
}


