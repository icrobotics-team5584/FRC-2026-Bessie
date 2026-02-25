#include "subsystems/Turret/TurretCancoderIO.h"
#include "subsystems/Turret/TurretEncoderConfig.h"
#include "utilities/Logger.h"

TurretCancoderIO::TurretCancoderIO(int encoder1ID, int encoder2ID) : _encoder1(encoder1ID), _encoder2(encoder2ID) {}

void TurretCancoderIO::ConfigEncoder() {

}

units::degree_t TurretCancoderIO::GetEncoder1Degrees() {
    return _encoder1.GetPosition().GetValue() - TurretThroughboreConfig::encoder1ZeroOffset;
}

units::degree_t TurretCancoderIO::GetEncoder2Degrees() {
    return _encoder2.GetPosition().GetValue() - TurretThroughboreConfig::encoder2ZeroOffset;
}

bool TurretCancoderIO::IsConnected() {
    return _encoder1.IsConnected() && _encoder2.IsConnected();
}