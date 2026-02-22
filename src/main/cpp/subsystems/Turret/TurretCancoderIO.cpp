#include "subsystems/Turret/TurretCancoderIO.h"
#include "subsystems/Turret/TurretEncoderConfig.h"
#include "utilities/Logger.h"

TurretCancoderIO::TurretCancoderIO(int encoder1CanID, int encoder2CanID) : _encoder1(encoder1CanID), _encoder2(encoder2CanID) {}

void TurretCancoderIO::ConfigEncoder() {

}

units::degree_t TurretCancoderIO::GetEncoder1Degrees() {
    return (_encoder1.GetPosition().GetValueAsDouble() - TurretThroughboreConfig::encoder1ZeroOffset)*360_deg;
}

units::degree_t TurretCancoderIO::GetEncoder2Degrees() {
    return (_encoder2.GetPosition().GetValueAsDouble() - TurretThroughboreConfig::encoder2ZeroOffset)*360_deg;
}

bool TurretCancoderIO::IsConnected() {
    return _encoder1.IsConnected() && _encoder2.IsConnected();
}