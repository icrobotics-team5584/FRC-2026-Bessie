#include "subsystems/Turret/TurretCancoderIO.h"
#include "subsystems/Turret/TurretEncoderConfig.h"
#include "utilities/Logger.h"

TurretCancoderIO::TurretCancoderIO(int encoderCanID) : _encoder(encoderCanID) {}

void TurretCancoderIO::ConfigEncoder() {

}

double TurretCancoderIO::GetPosition() {
    return _encoder.GetPosition().GetValueAsDouble();
}

bool TurretCancoderIO::IsConnected() {
    return _encoder.IsConnected();
}