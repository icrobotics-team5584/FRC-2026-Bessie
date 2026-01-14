// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIndexor.h"
#include <units/current.h>
#include <utilities/Logger.h>
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

// This method will be called once per scheduler run
void SubIndexor::Periodic() {}
