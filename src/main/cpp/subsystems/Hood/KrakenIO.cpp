#include "subsystems/Hood/KrakenIO.h"
#include "subsystems/Hood/HoodMotorConfig.h"

KrakenIO::KrakenIO(int motorCanID) : _motor(motorCanID) {}

void KrakenIO::ConfigMotor() {
    ctre::phoenix6::configs::TalonFXConfiguration config;
    config.Feedback.SensorToMechanismRatio = KrakenMotorConfig::GEAR_RATIO;
    config.Slot0.kP = KrakenMotorConfig::P;
    config.Slot0.kI = KrakenMotorConfig::I;
    config.Slot0.kD = KrakenMotorConfig::D;
    config.MotorOutput.Inverted = KrakenMotorConfig::INVERTED;
    config.MotorOutput.NeutralMode = KrakenMotorConfig::IDLE_MODE;

    config.CurrentLimits.SupplyCurrentLimitEnable = KrakenMotorConfig::ENABLE_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = KrakenMotorConfig::SUPPLY_CURRENT_LIMT;
    config.CurrentLimits.SupplyCurrentLowerLimit = KrakenMotorConfig::SUPPLY_CURRENT_LOWER_LIMT;
    config.CurrentLimits.SupplyCurrentLowerTime = KrakenMotorConfig::SUPPLY_CURRENT_LOWER_LIMT;
    
    config.CurrentLimits.StatorCurrentLimitEnable = KrakenMotorConfig::ENABLE_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimit = KrakenMotorConfig::STATOR_CURRENT_LIMIT;
    
    _motor.GetConfigurator().Apply(config);
}

void KrakenIO::SetVoltage(units::volt_t voltage) {
    _motor.SetControl(ctre::phoenix6::controls::VoltageOut{voltage});
}

void KrakenIO::SetPositionTarget(units::degree_t target) {
    _desiredAngle = target;
    _motor.SetControl(ctre::phoenix6::controls::PositionVoltage(_desiredAngle)
        .WithEnableFOC(KrakenMotorConfig::ENABLE_FOC));
}

void KrakenIO::SetBrakeMode(bool isBreakModeOn) {
    if (isBreakModeOn) {
        _motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    } else {
        _motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    }
}

void KrakenIO::StartLoggingMotor(std::string keyName) {
    Logger::LogFalcon(keyName, _motor);
}

units::degree_t KrakenIO::GetPosition() {
    return _motor.GetPosition().GetValue();
}

units::degree_t KrakenIO::GetPositionTarget() {
    return _desiredAngle;
}

units::ampere_t KrakenIO::GetCurrent() {
    return _motor.GetStatorCurrent().GetValue();
}

units::volt_t KrakenIO::GetVoltage() {
    return _motor.GetMotorVoltage().GetValue();
}
