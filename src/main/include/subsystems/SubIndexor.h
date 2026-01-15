// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"

#include <frc/Alert.h>
#include <frc/Timer.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>

#include <Constants.h>

#include "frc2/command/Commands.h"
#include "rev/config/SparkFlexConfig.h"

class SubIndexor : public frc2::SubsystemBase {
 public:
  static SubIndexor& GetInstance() {
    static SubIndexor instance;
    return instance;
  }
  SubIndexor();

  frc2::CommandPtr IndexorOn();
  frc2::CommandPtr IndexorOff();

  void CurrentHighTimer();

  frc::Alert indexorCurrentAlert{"Indexor Motor Overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert highTempuratureAlert{
    "Indexor Motor High Temperature!", frc::Alert::AlertType::kWarning};
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ICSparkFlex _indexorMotor{canid::INDEXOR};
  rev::spark::SparkFlexConfig _indexorMotorConfig;

  frc::Timer _indexorHighCurrentTimer;

  // Simulation components
  static constexpr double GEARING = 1.0;
  static constexpr units::kilogram_square_meter_t MOI = 0.02_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _sim{_flywheelSystem, MOTOR_MODEL};
};
