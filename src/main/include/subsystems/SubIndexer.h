// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/AlertController.h"
#include "utilities/ICSparkFlex.h"

#include <frc/Alert.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>

#include <Constants.h>

#include "frc2/command/Commands.h"
#include "rev/config/SparkFlexConfig.h"

class SubIndexer : public frc2::SubsystemBase {
 public:
  static SubIndexer& GetInstance() {
    static SubIndexer instance;
    return instance;
  }
  SubIndexer();

  frc2::CommandPtr IndexerOn();
  frc2::CommandPtr IndexerOff();

  frc2::CommandPtr Index();
  frc2::CommandPtr IndexBackwards();
  frc2::CommandPtr StopIndex();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ICSparkFlex _indexerMotor{canid::INDEXER};
  rev::spark::SparkFlexConfig _indexerMotorConfig;

  std::shared_ptr<AlertController::AlertConfig> _indexerAlertConfig;

  // Simulation components
  static constexpr double GEARING = 1.0;
  static constexpr units::kilogram_square_meter_t MOI = 0.0000001_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _sim{_flywheelSystem, MOTOR_MODEL};
};
