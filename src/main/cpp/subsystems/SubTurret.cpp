// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubTurret.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/Logger.h"
#include "utilities/PoseHandler.h"
#include <frc/RobotBase.h>

SubTurret::SubTurret() {
    _turretMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO);
    _turretMotorConfig.encoder.VelocityConversionFactor(1/GEAR_RATIO);
    _turretMotorConfig.closedLoop.Pid(P, I, D);
    _turretMotorConfig.closedLoop.MaxOutput(1.0);
    _turretMotorConfig.closedLoop.MinOutput(-1.0);
    _turretMotorConfig.closedLoop.IMaxAccum(0.05);
    _turretMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    _turretMotorConfig.SmartCurrentLimit(30);
    _turretMotorConfig.softLimit.ForwardSoftLimit(POS_LIMIT.convert<units::turns>().value());
    _turretMotorConfig.softLimit.ForwardSoftLimitEnabled(true);
    _turretMotorConfig.softLimit.ReverseSoftLimit(NEG_LIMIT.convert<units::turns>().value());
    _turretMotorConfig.softLimit.ReverseSoftLimitEnabled(true);
    _turretMotor.OverwriteConfig(_turretMotorConfig);

    _turretEncoder1.SetAssumedFrequency(ENCODER_FREQUENCY);
    _turretEncoder2.SetAssumedFrequency(ENCODER_FREQUENCY);

    frc::SmartDashboard::PutData("Turret/Motor", &_turretMotor);
    frc::SmartDashboard::PutData("Turret/mech2dDisplay", &_turretMech);
}

// This method will be called once per scheduler run
void SubTurret::Periodic() {
    auto loopStart = frc::GetTime();
    units::celsius_t turretTemperature = _turretMotor.GetTemperature();
    units::ampere_t turretCurrent = _turretMotor.GetStatorCurrent();

    AlertController::UpdateTemperatureAlert(_turretAlertConfig, turretTemperature);
    AlertController::UpdateCurrentAlert(_turretAlertConfig, turretCurrent);

    if(_hasZeroed == false && _turretEncoder1.IsConnected() && _turretEncoder2.IsConnected()) {
        units::degree_t motorPosition = _turretMotor.GetPosition();
        units::degree_t crtPosition = GetTurretAngleCRT();
        Logger::Log("Turret/reset/motorPosition", motorPosition);
        Logger::Log("Turret/reset/crtPosition", crtPosition);

        bool CRTSameAsMotor = (units::math::abs(motorPosition - crtPosition) < 0.5_deg);
        Logger::Log("Turret/reset/CRTSameAsMotor", CRTSameAsMotor);

        if(CRTSameAsMotor){
            _hasZeroed = true;
        }
        if(!CRTSameAsMotor){
            _hasZeroed = false;
            ZeroTurret();
        }
    }

    _turretMechCircle.SetAngle(_turretMotor.GetPosition());

    Logger::Log("Turret/Field Relative Turret Angle", GetFieldRelativeTurretAngle());
    Logger::Log("Turret/CRT Positiion", GetTurretAngleCRT());
    Logger::Log("Turret/Encoder/Encoder1", _turretEncoder1.Get());
    Logger::Log("Turret/Encoder/Encoder2", _turretEncoder2.Get());
    Logger::Log("Turret/Encoder/ZeroedEncoder1", getEncoder1Degrees());
    Logger::Log("Turret/Encoder/ZeroedEncoder2", getEncoder2Degrees());
    Logger::Log("Turret/Encoder/e1init", encoder1ZeroOffset);
    Logger::Log("Turret/Encoder/e2init", encoder2ZeroOffset);
    Logger::Log("Turret/hasReset", _hasZeroed);
    Logger::Log("Turret/IsAtTarget", IsAtTarget());

    Logger::Log("Turret/Encoder/Encoder1IsConnected", _turretEncoder1.IsConnected());
    Logger::Log("Turret/Encoder/Encoder2IsConnected", _turretEncoder2.IsConnected());
    Logger::Log("Turret/Encoder/Encoder1Frequency", _turretEncoder1.GetFrequency());
    Logger::Log("Turret/Encoder/Encoder2Frequency", _turretEncoder2.GetFrequency());

    Logger::Log("Turret/Loop Time", (frc::GetTime() - loopStart));
    _turretPos.AddSample(frc::Timer::GetFPGATimestamp(), CalcOptimisedTurretAngle(_turretMotor.GetPosition()));
}

void SubTurret::SimulationPeriodic() {
    _turretSim.SetInputVoltage(_turretMotor.CalcSimVoltage());
    _turretSim.Update(20_ms);
    _turretMotor.IterateSim(_turretSim.GetAngularVelocity(), _turretSim.GetAngularPosition());
}

units::degree_t SubTurret::GetTurretAngleCRT() {

    if(frc::RobotBase::IsSimulation()) {
        return _turretMotor.GetPosition();
    }

    // get encoder values and difference
    double e1deg = getEncoder1Degrees().value();
    double e2deg = getEncoder2Degrees().value();
    double difference = e2deg - e1deg;

    // clamp difference
    if(difference > 180) {
        difference -= 360;
    } else if(difference < -180) {
        difference += 360;
    } 

    // find slope and multiply to difference 
    // (converting from encoder difference to turret degrees)
    static double SLOPE = (E2_TEETH * E1_TEETH) / (BIG_TEETH);
    difference *= SLOPE;

    // estimate encoder 1 rotation count
    // (solve for encoder 1 rotations)
    double e1rotations = (difference * BIG_TEETH / E1_TEETH) / 360.0;
    double e1rotations_floored = floor(e1rotations);

    // solve for turret angle with encoder 1
    double turretAngle = (
        (e1rotations_floored * 360.0 + e1deg) *
        (E1_TEETH / BIG_TEETH)
    );

    // resolve ambiguity (when encoders are the same again)
    double period = (E1_TEETH / BIG_TEETH) * 360.0;

    if(turretAngle - difference < -period / 2) {
        turretAngle += period;
    } else if(turretAngle - difference > period / 2) {
        turretAngle -= period;
    }

    // Move the zero angle to point at the robot's front (Intake) 
    return (turretAngle*1_deg) - turretZeroOffset;
}

units::degree_t SubTurret::GetTurretAngle() {
    return _turretMotor.GetPosition();
}

units::degree_t SubTurret::GetTurretAngleAtTime(units::second_t time) {
    auto sample = _turretPos.Sample(time);
    if (sample.has_value()) {
        return sample.value();
    } else {
        return GetTurretAngle();
    }
}

frc2::CommandPtr SubTurret::SetTurretTargetAngle(std::function<units::degree_t()> angle, std::function<units::degrees_per_second_t()> robotAngVel){
    return Run([this, angle, robotAngVel] {
        units::degrees_per_second_t nextVel = -robotAngVel(); 
        // we want the turret to negate the robot rotation, hence the negative

        units::volt_t rotationFeedforward = _robotRotVelFF.Calculate(nextVel);

        Logger::Log("Turret/RobotRotFF/ffVolts", rotationFeedforward);
        Logger::Log("Turret/RobotRotFF/DrivebaseRotVel", nextVel);
        _turretMotor.SetPositionTarget(CalcOptimisedTurretAngle(angle()), rotationFeedforward);
    });
}

units::degree_t SubTurret::CalcOptimisedTurretAngle(units::degree_t angle) {
    units::degree_t currentAngle = SubTurret::GetInstance().GetTurretAngle();
    Logger::Log("Turret/CalcOptimisedTurretAngle/CurrentAngle", currentAngle);
    Logger::Log("Turret/CalcOptimisedTurretAngle/angle(input)", angle);

    //limit target angle to limits
    if(angle > POS_LIMIT) { angle -= 360_deg;}
    if(angle < NEG_LIMIT) { angle += 360_deg;}

    units::degree_t closestOffset = angle - currentAngle;

    // limit to +- 180 deg
    if(closestOffset > 180_deg) {
        closestOffset -= 360_deg;
    }
    if(closestOffset < -180_deg) {
        closestOffset += 360_deg;
    }
    
    Logger::Log("Turret/CalcOptimisedTurretAngle/closestOffset", closestOffset);

    units::degree_t newTarget = currentAngle + closestOffset;

    // clamp target to limits
    if(newTarget > POS_LIMIT) {
        newTarget -= 360_deg;
    }

    if(newTarget < NEG_LIMIT) {
        newTarget += 360_deg;
    }

    Logger::Log("Turret/CalcOptimisedTurretAngle/newTarget(final output)", newTarget);
    return newTarget;
}

void SubTurret::SetTurretAngle(units::degree_t angle) {
    _turretMotor.SetPosition(angle);
}

void SubTurret::ZeroTurret() {
    SetTurretAngle(GetTurretAngleCRT());
    _turretMotor.SetPositionTarget(GetTurretAngleCRT());
}

frc2::CommandPtr SubTurret::ZeroTurretCmd() {
    return RunOnce( [this] {
        ZeroTurret();
    });
}

units::degree_t SubTurret::getEncoder1Degrees() {
    return (_turretEncoder1.Get()-encoder1ZeroOffset)*360_deg;
}

units::degree_t SubTurret::getEncoder2Degrees() {
    return (_turretEncoder2.Get()-encoder2ZeroOffset)*360_deg;
}

bool SubTurret::IsAtTarget() {
    return units::math::abs(_turretMotor.GetPosError()) < TOLARANCE;
}

units::degree_t SubTurret::GetFieldRelativeTurretAngle() {
    auto robot = PoseHandler::GetInstance().GetPose();
    return robot.Rotation().Degrees() + GetTurretAngle();
}

void SubTurret::SetBrakeMode(bool brakeMode){
    rev::spark::SparkBaseConfig _neutralModeConfig;
    if (brakeMode == true) {
        _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        _turretMotor.AdjustConfigNoPersist(_neutralModeConfig);
    } else if (brakeMode == false) {
        _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
        _turretMotor.AdjustConfigNoPersist(_neutralModeConfig);
    }
}