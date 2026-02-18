#include "subsystems/Hood/NeoIO.h"
#include "subsystems/Hood/HoodMotorConfig.h"

NeoIO::NeoIO(int motorCanID) : _motor(motorCanID) {}

void NeoIO::ConfigMotor() {
    rev::spark::SparkBaseConfig _config;
    _config.encoder.PositionConversionFactor(1/NeoMotorConfig::GEAR_RATIO);
    _config.encoder.VelocityConversionFactor(1/NeoMotorConfig::GEAR_RATIO);
    _config.closedLoop.Pid(
        NeoMotorConfig::P,
        NeoMotorConfig::I,
        NeoMotorConfig::D);
    _config.closedLoop.feedForward.kS(NeoMotorConfig::S);
    _config.SmartCurrentLimit(NeoMotorConfig::CURRENT_LIMT);
    _config.Inverted(NeoMotorConfig::INVERTED);
    _config.SetIdleMode(NeoMotorConfig::IDLE_MODE);
    _motor.OverwriteConfig(_config);
}

void NeoIO::SetVoltage(units::volt_t voltage) {
    _motor.SetVoltage(voltage);
}

void NeoIO::SetPositionTarget(units::degree_t target) {
    _motor.SetPositionTarget(target);
}

void NeoIO::SetBrakeMode(bool isBreakModeOn) {
    rev::spark::SparkBaseConfig _brakeModeConfig;
    if (isBreakModeOn) {
        _brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    } else {
        _brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    }
    _motor.AdjustConfigNoPersist(_brakeModeConfig);
}

void NeoIO::StartLoggingMotor(std::string keyName) {
    Logger::Log(keyName, &_motor);
}

units::degree_t NeoIO::GetPosition() {
    return _motor.GetPosition();
}

units::degree_t NeoIO::GetPositionTarget() {
    return _motor.GetPositionTarget();
}

units::ampere_t NeoIO::GetCurrent() {
    return _motor.GetStatorCurrent();
}

units::volt_t NeoIO::GetVoltage() {
    return _motor.GetMotorVoltage();
}