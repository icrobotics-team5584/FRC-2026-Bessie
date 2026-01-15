// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexor.h"
#include <units/current.h>
#include <utilities/Logger.h>

#include <frc/Alert.h>
#include <frc/smartdashboard/SmartDashboard.h>

SubIndexor::SubIndexor() {
    _indexorMotorConfig.SmartCurrentLimit(60);
    _indexorMotor.OverwriteConfig(_indexorMotorConfig);
     Logger::Log("Indexor/Indexor Motor", &_indexorMotor);
}

frc2::CommandPtr SubIndexor::IndexorOn() {
  return StartEnd([this] { _indexorMotor.Set(1); }, [this] { _indexorMotor.Set(0); });
};

frc2::CommandPtr SubIndexor::IndexorOff() {
  return RunOnce([this] { _indexorMotor.Set(0); });
};

void SubIndexor::CurrentHighTimer() {
  _indexorHighCurrentTimer.Start();

  if (_indexorHighCurrentTimer.Get() > 3_s) {
    indexorCurrentAlert.Set(true);
  }
};

// This method will be called once per scheduler run
void SubIndexor::Periodic() {
   units::ampere_t current = _indexorMotor.GetOutputCurrent() * 1_A;
  frc::SmartDashboard::PutNumber("Indexor/Current", current.value());
  if (current > 20_A) {
    SubIndexor::CurrentHighTimer();
  } else {
    indexorCurrentAlert.Set(false);
    _indexorHighCurrentTimer.Reset();
  }

  units::celsius_t temperature = _indexorMotor.GetTemperature();
  frc::SmartDashboard::PutNumber("Indexor/Temperature", temperature.value());
  if (temperature > 60_degC) {
    highTempuratureAlert.Set(true);
  } else {
    highTempuratureAlert.Set(false);
  }
}

void SubIndexor::SimulationPeriodic() {
  _sim.SetInputVoltage(_indexorMotor.CalcSimVoltage());
  _sim.Update(20_ms);
  _indexorMotor.IterateSim(_sim.GetAngularVelocity());
}
