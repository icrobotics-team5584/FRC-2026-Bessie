#include "subsystems/Turret/TurretThroughboreIO.h"
#include "subsystems/Turret/TurretEncoderConfig.h"
#include "utilities/Logger.h"

TurretThroughboreIO::TurretThroughboreIO(int encoderCanID) : _encoder(encoderCanID) {}

void TurretThroughboreIO::ConfigEncoder() {
    _encoder.SetAssumedFrequency(Throughbore::ENCODER_FREQUENCY);
}

double TurretThroughboreIO::GetPosition() {
    return _encoder.Get();
}

bool TurretThroughboreIO::IsConnected() {
    return _encoder.IsConnected();
}