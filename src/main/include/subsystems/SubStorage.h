// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Alert.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include <utilities/IcSparkFlex.h>

#include "Constants.h"
#include "frc2/command/Commands.h"
#include "rev/config/SparkFlexConfig.h"

class SubStorage : public frc2::SubsystemBase {
 public:
  static SubStorage& GetInstance() {
    static SubStorage instance;
    return instance;
  }
  SubStorage();

  frc2::CommandPtr StorageOn();
  frc2::CommandPtr StorageOff();

  frc2::Trigger StorageEnterTrigger();

  bool IsFull();
  bool IsEmpty();
  bool CheckSensorHigher();
  bool CheckSensorLower();

  void CurrentHighTimer();

  frc::Alert storageCurrentAlert{"Storage Motor Overcurrent!", frc::Alert::AlertType::kWarning};
  frc::Alert highTemperatureAlert{
    "Storage Motor High Temperature!", frc::Alert::AlertType::kWarning};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  int fuelCount = 0;
  frc::Timer _storageHighCurrentTimer;
  ICSparkFlex _storageMotor{canid::STORAGE};
  rev::spark::SparkFlexConfig _storageMotorConfig;

  frc::DigitalInput _storagFullSensor{dio::STORAGE_SENSOR_FULL};
  frc::DigitalInput _storageEmptySensor{dio::STORAGE_SENSOR_EMPTY};
  frc::DigitalInput _storageEnterSensor{dio::STORAGE_SENSOR_ENTER};

  static constexpr double GEARING = 1.0;
  static constexpr units::kilogram_square_meter_t MOI =  0.0000001_kg_sq_m;
  static constexpr frc::DCMotor MOTOR_MODEL = frc::DCMotor::NeoVortex();
  frc::LinearSystem<1, 1, 1> _flywheelSystem =
    frc::LinearSystemId::FlywheelSystem(MOTOR_MODEL, MOI, GEARING);
  frc::sim::FlywheelSim _sim{_flywheelSystem, MOTOR_MODEL};
};
