// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

class SubTurret : public frc2::SubsystemBase {
 public:
  SubTurret();
  static SubTurret& GetInstance() {
    static SubTurret inst;
    return inst;
  }

  void SimulationPeriodic();

  double GetTurretAngle(double encoder1, double encoder2);
  frc2::CommandPtr SetTurretAngle(units::degree_t angle);



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  ICSparkMax _turretMotor{canid::TURRET_MOTOR, 30_A};
  rev::spark::SparkBaseConfig _turretMotorConfig;

  double P = 0;
  double I = 0;
  double D = 0;
  
  static constexpr double ENCODER1_RATIO = 20.0/115; //20/115
  static constexpr double ENCODER2_RATIO = 22.0/115; //22/115
  static constexpr double GEAR_RATIO = 1.0/46; //1/46
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
