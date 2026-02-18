#include "subsystems/Hood/GeneralMotor.h"
#include "subsystems/Hood/HoodKrakenIO.h"
#include "subsystems/Hood/HoodNeoIO.h"
#include "utilities/BotVars.h"

GeneralMotor::GeneralMotor(int motorCanID) {
    if (BotVars::GetRobot() == BotVars::PRACTICE){
    _io = std::make_unique<HoodNeoIO>(motorCanID);
  }
  else {
    _io = std::make_unique<HoodKrakenIO>(motorCanID);
  }

  ConfigMotor();
}

void GeneralMotor::ConfigMotor() {
    _io->ConfigMotor();
}
void GeneralMotor::SetVoltage(units::volt_t voltage) {
    _io->SetVoltage(voltage);
}
void GeneralMotor::SetPosition(units::degree_t pos) {
    _io->SetPosition(pos);
}
void GeneralMotor::SetPositionTarget(units::degree_t target) {
    _io->SetPositionTarget(target);
}
void GeneralMotor::SetBrakeMode(bool isBreakModeOn) {
    _io->SetBrakeMode(isBreakModeOn);
}
void GeneralMotor::StartLoggingMotor(std::string keyName) {
    _io->StartLoggingMotor(keyName);
}
void GeneralMotor::StopMotor() {
    _io->StopMotor();
}
units::degree_t GeneralMotor::GetPosition() {
    return _io->GetPosition();
}
units::degree_t GeneralMotor::GetPositionTarget() {
    return _io->GetPositionTarget();
}
units::degree_t GeneralMotor::GetPositionError() {
    return _io->GetPositionError();
}
units::ampere_t GeneralMotor::GetCurrent() {
    return _io->GetCurrent();
}
units::volt_t GeneralMotor::GetVoltage() {
    return _io->GetVoltage();
}