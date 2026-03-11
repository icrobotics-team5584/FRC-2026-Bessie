// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"
#include "utilities/ICSparkFlex.h"

#include <frc/Alert.h>
#include <frc/DigitalInput.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/config/SparkFlexConfig.h>

#include "Constants.h"
#include "frc2/command/Commands.h"

class SubFeeder : public frc2::SubsystemBase {
 public:
  static SubFeeder& GetInstance() {
    static SubFeeder instance;
    return instance;
  }
  SubFeeder();

  frc2::CommandPtr Feed();
  frc2::CommandPtr FeederOn();
  frc2::CommandPtr FeederOff();

  void Periodic() override;

  void SimulationPeriodic() override;

 private:
  ICSparkFlex _feederMotor{canid::FEEDER};
  rev::spark::SparkFlexConfig _feederMotorConfig;

  std::shared_ptr<AlertController::AlertConfig> _feederAlertConfig;

  // Simulation components
  static constexpr double GEARING = 1.0;
  static constexpr units::kilogram_square_meter_t MOI = 0.0000001_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NEO(1);
  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _sim{_flywheelSystem, MOTOR_MODEL};
};
