#include "utilities/BotVars.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>

#include <iostream>

BotVars::Robot BotVars::DetermineRobot() {
  std::string serial = "";
  if (frc::RobotBase::IsSimulation()) {
    serial = "Simulation";
  } else {
    serial = frc::RobotController::GetSerialNumber();
  }
  frc::SmartDashboard::PutString("botVars/Detected serial", serial);
  frc::SmartDashboard::PutString("botVars/comp bot serial", COMP_BOT_SERIAL);
  frc::SmartDashboard::PutString("botVars/prac bot serial", PRACTICE_BOT_SERIAL);

  if (serial == COMP_BOT_SERIAL) {
    frc::SmartDashboard::PutString("botVars/Active robot", "COMP");
    return Robot::COMP;
  } else if (serial == PRACTICE_BOT_SERIAL) {
    frc::SmartDashboard::PutString("botVars/Active robot", "PRACTICE");
    return Robot::PRACTICE;
  }

  frc::SmartDashboard::PutString(
    "botVars/Active robot", "ERROR! Could not match serial. Defaulting to COMP.");
  return Robot::COMP;
}

BotVars::Robot BotVars::GetRobot() {
  static Robot activeRobot = DetermineRobot();
  return activeRobot;
}
