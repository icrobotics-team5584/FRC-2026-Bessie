#include "subsystems/Hood/HoodNeoIO.h"
#include "subsystems/Hood/HoodMotorConfig.h"

HoodNeoIO::HoodNeoIO(int motorCanID) : _motor(motorCanID) {}

void HoodNeoIO::ConfigMotor() {
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

void HoodNeoIO::SetVoltage(units::volt_t voltage) {
    _motor.SetVoltage(voltage);
}

void HoodNeoIO::SetPositionTarget(units::degree_t target) {
    _motor.SetPositionTarget(target);
}

void HoodNeoIO::SetBrakeMode(bool isBreakModeOn) {
    rev::spark::SparkBaseConfig _brakeModeConfig;
    if (isBreakModeOn) {
        _brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    } else {
        _brakeModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    }
    _motor.AdjustConfigNoPersist(_brakeModeConfig);
}

void HoodNeoIO::StartLoggingMotor(std::string keyName) {
    Logger::Log(keyName, &_motor);
}

void HoodNeoIO::StopMotor() {
    _motor.Set(0);
}

units::degree_t HoodNeoIO::GetPosition() {
    return _motor.GetPosition();
}

units::degree_t HoodNeoIO::GetPositionTarget() {
    return _motor.GetPositionTarget();
}

units::ampere_t HoodNeoIO::GetCurrent() {
    return _motor.GetStatorCurrent();
}

units::volt_t HoodNeoIO::GetVoltage() {
    return _motor.GetMotorVoltage();
}