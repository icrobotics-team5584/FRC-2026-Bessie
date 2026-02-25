#include "subsystems/Turret/TurretThroughboreIO.h"
#include "subsystems/Turret/TurretEncoderConfig.h"
#include "utilities/Logger.h"

TurretThroughboreIO::TurretThroughboreIO(int encoder1CanID, int encoder2CanID) : _encoder1(encoder1CanID), _encoder2(encoder2CanID) {}

void TurretThroughboreIO::ConfigEncoder() {
    _encoder1.SetAssumedFrequency(TurretThroughboreConfig::ENCODER_FREQUENCY);
    _encoder2.SetAssumedFrequency(TurretThroughboreConfig::ENCODER_FREQUENCY);
}

units::degree_t TurretThroughboreIO::GetEncoder1Degrees() {
    return (_encoder1.Get() - TurretThroughboreConfig::encoder1ZeroOffset)*360_deg;
}

units::degree_t TurretThroughboreIO::GetEncoder2Degrees() {
    return (_encoder2.Get() - TurretThroughboreConfig::encoder2ZeroOffset)*360_deg;
}

bool TurretThroughboreIO::IsConnected() {
    return _encoder1.IsConnected() && _encoder2.IsConnected();
}