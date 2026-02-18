#include "subsystems/Turret/GeneralEncoder.h"
#include "subsystems/Turret/TurretCancoderIO.h"
#include "subsystems/Turret/TurretThroughboreIO.h"
#include "utilities/BotVars.h"

GeneralEncoder::GeneralEncoder(int encoderCanID) {
    if(BotVars::GetRobot() == BotVars::PRACTICE) {
        _io = std::make_unique<TurretThroughboreIO>(encoderCanID);
    }
    else {
        _io = std::make_unique<TurretCancoderIO>(encoderCanID);
    }

    ConfigEncoder();
}

void GeneralEncoder::ConfigEncoder() {
    _io->ConfigEncoder();
}

units::degree_t GeneralEncoder::GetPosition() {
    return _io->GetPosition();
}