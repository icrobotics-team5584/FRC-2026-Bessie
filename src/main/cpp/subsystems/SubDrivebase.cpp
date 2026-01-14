#include <frc/RobotBase.h>
#include "subsystems/SubDrivebase.h"
#include "utilities/PoseHandler.h"
#include "utilities/Logger.h"

SubDrivebase::SubDrivebase() {
  ctre::phoenix6::configs::Pigeon2Configuration gyroConfig;
  gyroConfig.MountPose.MountPosePitch = 0_deg;
  gyroConfig.MountPose.MountPoseRoll = 0_deg;
  gyroConfig.MountPose.MountPoseYaw = 0_deg;
  _gyro.GetConfigurator().Apply(gyroConfig);
}

void SubDrivebase::Periodic() {
  auto loopstart = frc::GetTime();
  LogDrivebaseStates();
  UpdateOdometry();
  Logger::Log("Drivebase/loop time (sec)", (frc::GetTime() - loopstart));
}

void SubDrivebase::SimulationPeriodic() {
  _frontLeft.UpdateSim(20_ms);
  _frontRight.UpdateSim(20_ms);
  _backLeft.UpdateSim(20_ms);
  _backRight.UpdateSim(20_ms);

  auto rotSpeed = _kinematics
                      .ToChassisSpeeds(_frontLeft.GetState(), _frontRight.GetState(),
                                       _backLeft.GetState(), _backRight.GetState()).omega;
  units::radian_t changeInRot = rotSpeed * 20_ms;
  units::degree_t newHeading = GetGyroAngle().RotateBy(changeInRot).Degrees();
  _gyro.SetYaw(newHeading);

  wpi::array<frc::SwerveModulePosition, 4U> states = {
    _frontLeft.GetPosition(),
    _frontRight.GetPosition(),
    _backLeft.GetPosition(),
    _backRight.GetPosition()
  };

  PoseHandler::GetInstance().UpdateSim(GetGyroAngle(), states, true, PoseHandler::GetInstance().GetPose().Rotation());
}

// Commands

void SubDrivebase::LogDrivebaseStates() {
  Logger::Log("Drivebase/GyroAngle/Roll", SubDrivebase::GetInstance().GetRoll().value());
  Logger::Log("Drivebase/GyroAngle/Pitch", SubDrivebase::GetInstance().GetPitch().value());
  Logger::Log("Drivebase/Coast Button", CheckCoastButton().Get());

  Logger::Log("Drivebase/velocity", GetVelocity());
  Logger::Log("Drivebase/Internal Encoder Swerve States",
              wpi::array{_frontLeft.GetState(), _frontRight.GetState(), _backLeft.GetState(),
                         _backRight.GetState()});
  Logger::Log("Drivebase/CANCoder Swerve States",
              wpi::array{_frontLeft.GetCANCoderState(), _frontRight.GetCANCoderState(),
                         _backLeft.GetCANCoderState(), _backRight.GetCANCoderState()});
  Logger::Log("Drivebase/Pigeon raw angle", _gyro.GetYaw().GetValue().value());
  Logger::Log("Drivebase/Pigeon raw Rotation2d", _gyro.GetRotation2d().Degrees());

  units::turn_t flRotations = _frontLeft.GetDrivenRotations();
  units::turn_t frRotations = _frontRight.GetDrivenRotations();
  units::turn_t blRotations = _backLeft.GetDrivenRotations();
  units::turn_t brRotations = _backRight.GetDrivenRotations();

  Logger::Log("Drivebase/DistanceDrivenRotations/fl", flRotations);
  Logger::Log("Drivebase/DistanceDrivenRotations/fr", frRotations);
  Logger::Log("Drivebase/DistanceDrivenRotations/bl", blRotations);
  Logger::Log("Drivebase/DistanceDrivenRotations/br", brRotations);

  Logger::Log("Drivebase/DistanceDriven/fl",
              (flRotations) * (0.04121451348939883 * 2 * std::numbers::pi));
  Logger::Log("Drivebase/DistanceDriven/fr",
              (frRotations) * (0.04121451348939883 * 2 * std::numbers::pi));
  Logger::Log("Drivebase/DistanceDriven/bl",
              (blRotations) * (0.04121451348939883 * 2 * std::numbers::pi));
  Logger::Log("Drivebase/DistanceDriven/br",
              (brRotations) * (0.04121451348939883 * 2 * std::numbers::pi));

  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();
} 

void SubDrivebase::UpdateOdometry() {
  wpi::array<frc::SwerveModulePosition, 4U> states = {
    _frontLeft.GetPosition(),
    _frontRight.GetPosition(),
    _backLeft.GetPosition(),
    _backRight.GetPosition()
  };

  if (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) ==
      frc::DriverStation::Alliance::kBlue) {
    PoseHandler::GetInstance().Update(GetGyroAngle(), states);
  } else {
    PoseHandler::GetInstance().Update(GetGyroAngle() - 180_deg, states);
  }
}

frc2::CommandPtr SubDrivebase::DriveToPose(std::function<frc::Pose2d()> pose, double speedScaling = 1) {
  return Drive(([this,pose,speedScaling]{ return CalcDriveToPoseSpeeds(pose()) * speedScaling;}), true)
  .Until([this,pose] {return IsAtPose(pose());});
} 

void SubDrivebase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();

  _frontLeft.ConfigTurnMotor();
  _frontRight.ConfigTurnMotor();
  _backLeft.ConfigTurnMotor();
  _backRight.ConfigTurnMotor();
}

void SubDrivebase::SetPose(frc::Pose2d pose) {
  wpi::array<frc::SwerveModulePosition, 4U> states = {
    _frontLeft.GetPosition(),
    _frontRight.GetPosition(),
    _backLeft.GetPosition(),
    _backRight.GetPosition()
  };

  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
      frc::DriverStation::Alliance::kBlue) {
    ResetGyroHeading(pose.Rotation().Degrees());
  } else {
    ResetGyroHeading(pose.Rotation().Degrees() - 180_deg);
  }

  PoseHandler::GetInstance().Update(pose.Rotation(), states);
  PoseHandler::GetInstance().UpdateSim(pose.Rotation(), states);
}

bool SubDrivebase::IsAtPose(frc::Pose2d pose) {
  auto currentPose = PoseHandler::GetInstance().GetPose();
  auto rotError = GetAllianceRelativeGyroAngle() - pose.Rotation();
  auto posError = currentPose.Translation().Distance(pose.Translation());
  Logger::FieldDisplay::GetInstance().DisplayPose("current pose", currentPose);
  Logger::FieldDisplay::GetInstance().DisplayPose("target pose", pose);

  frc::SmartDashboard::PutNumber("Drivebase/rotError",
                                 rotError.Degrees().value());
  frc::SmartDashboard::PutNumber("Drivebase/posError", 
                                posError.value());

  frc::SmartDashboard::PutBoolean("Drivebase/IsAtPose",
                                  units::math::abs(rotError.Degrees()) < 2_deg && posError < 2_cm);

  if (units::math::abs(rotError.Degrees()) < 2_deg && posError < 2_cm) { 
    return true;
  } else {
    return false;
  }
}

frc2::CommandPtr SubDrivebase::SyncSensor() {
  return RunOnce([this] {SyncSensors();});
}

void SubDrivebase::ResetGyroHeading(units::degree_t startingAngle) {
  _gyro.SetYaw(startingAngle);
}

frc2::CommandPtr SubDrivebase::ResetGyroCmd() {
  return RunOnce([this] { ResetGyroHeading(0_deg); });
}

void SubDrivebase::SetBrakeMode(bool mode) {
  _frontLeft.SetBreakMode(mode);
  _frontRight.SetBreakMode(mode);
  _backLeft.SetBreakMode(mode);
  _backRight.SetBreakMode(mode);
}

void SubDrivebase::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                         units::turns_per_second_t rot, bool fieldRelative,
                         std::optional<std::array<units::newton_t, 4>> xForceFeedforwards,
                         std::optional<std::array<units::newton_t, 4>> yForceFeedforwards)
{
    // Optionally convert speeds to field relative
  auto speeds = fieldRelative
                    ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetGyroAngle())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  // Discretize to get rid of translational drift while rotating
  speeds = frc::ChassisSpeeds::Discretize(speeds, 60_ms);

  // Get states of all swerve modules
  auto states = _kinematics.ToSwerveModuleStates(speeds);

  // Set speed limit and apply speed limit to all modules
  _kinematics.DesaturateWheelSpeeds(
      &states,
      frc::SmartDashboard::GetNumber("Drivebase/Config/Max Velocity", DrivebaseConfig::MAX_VELOCITY.value()) *
          1_mps);

  // Extract force feedforwards
  std::array<units::newton_t, 4> defaults{0_N, 0_N, 0_N, 0_N};
  auto [flXForce, frXForce, blXForce, brXForce] = xForceFeedforwards.value_or(defaults);
  auto [flYForce, frYForce, blYForce, brYForce] = yForceFeedforwards.value_or(defaults);

  // Setting modules from aquired states
  Logger::Log("Drivebase/Desired Swerve States", states);
  auto [fl, fr, bl, br] = states;
  _frontLeft.SetDesiredState(fl, flXForce, flYForce);
  _frontRight.SetDesiredState(fr, frXForce, frYForce);
  _backLeft.SetDesiredState(bl, blXForce, blYForce);
  _backRight.SetDesiredState(br, brXForce, brYForce);
}

frc2::CommandPtr SubDrivebase::Drive(std::function<frc::ChassisSpeeds()> speeds, bool fieldOriented)
{
    return Run([this, speeds, fieldOriented] {
        auto speedVal = speeds();
        Drive(speedVal.vx, speedVal.vy, speedVal.omega, fieldOriented);
    }).FinallyDo([this] { Drive(0_mps,0_mps,0_deg_per_s, false); });
}

// Getters & calculations

frc::Rotation2d SubDrivebase::GetGyroAngle(bool allianceRelated) { 
  auto alliance = frc::DriverStation::GetAlliance();
  if (!allianceRelated ||
    alliance.value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kBlue) {
    return _gyro.GetRotation2d();
  } else {
    return _gyro.GetRotation2d() - 180_deg;
  }
}

units::degree_t SubDrivebase::GetPitch() {
  return (_gyro.GetPitch().GetValue());
}

units::degree_t SubDrivebase::GetRoll() {
  return (_gyro.GetRoll().GetValue());
}

units::meters_per_second_t SubDrivebase::GetVelocity() {
  // Use pythag to find velocity from x and y components
  auto speeds = _kinematics.ToChassisSpeeds(_frontLeft.GetState(), _frontRight.GetState(),
                                            _backLeft.GetState(), _backRight.GetState());
  namespace m = units::math;
  Logger::Log("Drivebase/velocity/vx", speeds.vx);
  Logger::Log("Drivebase/velocity/vy", speeds.vy);
  return m::sqrt(m::pow<2>(speeds.vx) + m::pow<2>(speeds.vy));
}

frc2::Trigger SubDrivebase::CheckCoastButton() {
  return frc2::Trigger{[this] { return !_toggleBrakeCoast.Get(); }};
}

units::turns_per_second_t SubDrivebase::CalcRotateSpeed(units::turn_t rotationError) {
  auto omega = _teleopRotationController.Calculate(rotationError, 0_deg) * 1_rad_per_s;
  omega = units::math::min(omega, DrivebaseConfig::MAX_ANGULAR_VELOCITY);
  omega = units::math::max(omega, -DrivebaseConfig::MAX_ANGULAR_VELOCITY);
  return omega;
}

frc::ChassisSpeeds SubDrivebase::CalcDriveToPoseSpeeds(frc::Pose2d targetPose) {
  double targetXMeters = targetPose.X().value();
  double targetYMeters = targetPose.Y().value();
  units::turn_t targetRotation = targetPose.Rotation().Radians();
  frc::Pose2d currentPosition = PoseHandler::GetInstance().GetPose();
  double currentXMeters = currentPosition.X().value();
  double currentYMeters = currentPosition.Y().value();
  units::turn_t currentRotation = GetGyroAngle(true).Degrees();

  // Use PID controllers to calculate speeds
  auto xSpeed = _teleopTranslationController.Calculate(currentXMeters, targetXMeters) * 1_mps;
  auto ySpeed = _teleopTranslationController.Calculate(currentYMeters, targetYMeters) * 1_mps;
  auto rSpeed = CalcRotateSpeed(currentRotation - targetRotation);

  // Clamp to max velocity
  xSpeed = units::math::min(xSpeed, DrivebaseConfig::MAX_DRIVE_TO_POSE_VELOCITY);
  xSpeed = units::math::max(xSpeed, -DrivebaseConfig::MAX_DRIVE_TO_POSE_VELOCITY);
  ySpeed = units::math::min(ySpeed, DrivebaseConfig::MAX_DRIVE_TO_POSE_VELOCITY);
  ySpeed = units::math::max(ySpeed, -DrivebaseConfig::MAX_DRIVE_TO_POSE_VELOCITY);

  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    xSpeed *= -1;
    ySpeed *= -1;
  }

  Logger::Log("CalcDriveLogs/xSpeed", -xSpeed.value());
  Logger::Log("CalcDriveLogs/ySpeed", ySpeed.value());
  Logger::Log("CalcDriveLogs/rSpeed", rSpeed.value());
  Logger::Log("CalcDriveLogs/targetXMeters", targetXMeters);
  Logger::Log("CalcDriveLogs/targetYMeters", targetYMeters);
  Logger::Log("CalcDriveLogs/currentXMeters", currentXMeters);
  Logger::Log("CalcDriveLogs/currentYMeters", currentYMeters);
  Logger::Log("CalcDriveLogs/currentRotation", currentRotation.value());
  return frc::ChassisSpeeds{xSpeed, ySpeed, rSpeed};
}

frc::ChassisSpeeds SubDrivebase::CalcJoystickSpeeds(frc2::CommandXboxController& controller) {
  std::string configPath = "Drivebase/Config/";
  auto deadband = Logger::Tune(configPath + "Joystick Deadband", DrivebaseConfig::JOYSTICK_DEADBAND);
  auto maxVelocity = Logger::Tune(configPath + "Max Velocity", DrivebaseConfig::MAX_VELOCITY);
  auto maxAngularVelocity = Logger::Tune(configPath + "Max Angular Velocity", DrivebaseConfig::MAX_ANGULAR_VELOCITY);
  auto maxJoystickAccel = Logger::Tune(configPath + "Max Joystick Accel", DrivebaseConfig::MAX_JOYSTICK_ACCEL);
  auto maxAngularJoystickAccel =
      Logger::Tune(configPath + "Max Joystick Angular Accel", DrivebaseConfig::MAX_ANGULAR_JOYSTICK_ACCEL);
  auto translationScaling =
      Logger::Tune(configPath + "Translation Scaling", DrivebaseConfig::TRANSLATION_SCALING);
  auto rotationScaling = Logger::Tune(configPath + "Rotation Scaling", DrivebaseConfig::ROTATION_SCALING);

  // Recreate slew rate limiters if limits have changed
  if (maxJoystickAccel != DrivebaseConfig::MAX_JOYSTICK_ACCEL) {
    _xStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _yStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _tunedMaxJoystickAccel = maxJoystickAccel;
  }
  if (maxAngularJoystickAccel != _tunedMaxAngularJoystickAccel) {
    _rotStickLimiter = frc::SlewRateLimiter<units::scalar>{maxAngularJoystickAccel / 1_s};
    _tunedMaxAngularJoystickAccel = maxAngularJoystickAccel;
  }

  // Apply deadbands
  double rawTranslationY = frc::ApplyDeadband(-controller.GetLeftY(), deadband);
  double rawTranslationX = frc::ApplyDeadband(-controller.GetLeftX(), deadband);
  double rawRotation = frc::ApplyDeadband(-controller.GetRightX(), deadband);

  // Convert cartesian (x, y) translation stick coordinates to polar (R, theta) and scale R-value
  double rawTranslationR = std::min(1.0, sqrt(pow(rawTranslationX, 2) + pow(rawTranslationY, 2)));
  double translationTheta = atan2(rawTranslationY, rawTranslationX);
  double scaledTranslationR = pow(rawTranslationR, translationScaling);

  // Convert polar coordinates (with scaled R-value) back to cartesian; scale rotation as well
  double scaledTranslationY = scaledTranslationR * sin(translationTheta);
  double scaledTranslationX = scaledTranslationR * cos(translationTheta);

  double scaledRotation;
  if (rawRotation >= 0) {
    scaledRotation = pow(rawRotation, rotationScaling);
  } else {
    scaledRotation = std::copysign(pow(abs(rawRotation), rotationScaling), rawRotation);
  }

  // Apply joystick rate limits and calculate speed
  auto forwardSpeed = _yStickLimiter.Calculate(scaledTranslationY) * maxVelocity;
  auto sidewaysSpeed = _xStickLimiter.Calculate(scaledTranslationX) * maxVelocity;
  auto rotationSpeed = _rotStickLimiter.Calculate(scaledRotation) * maxAngularVelocity;

  // Logger things
  Logger::Log("Drivebase/Joystick Scaling/rawTranslationY", rawTranslationY);
  Logger::Log("Drivebase/Joystick Scaling/rawTranslationX", rawTranslationX);
  Logger::Log("Drivebase/Joystick Scaling/rawTranslationR", rawTranslationR);
  Logger::Log(
      "Drivebase/Joystick Scaling/translationTheta (degrees)",
      translationTheta *
          (180 / std::numbers::pi));  // Multiply by 180/pi to convert radians to degrees
  Logger::Log("Drivebase/Joystick Scaling/scaledTranslationR",
                                 scaledTranslationR);
  Logger::Log("Drivebase/Joystick Scaling/scaledTranslationY",
                                 scaledTranslationY);
  Logger::Log("Drivebase/Joystick Scaling/scaledTranslationX",
                                 scaledTranslationX);
  Logger::Log("Drivebase/Joystick Scaling/rawRotation", rawRotation);
  Logger::Log("Drivebase/Joystick Scaling/scaledRotation", scaledRotation);

  return frc::ChassisSpeeds{forwardSpeed, sidewaysSpeed, rotationSpeed};
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController& controller, bool fieldOriented, double speedScale) {
  return Drive([this, speedScale, &controller] {
    auto speeds = CalcJoystickSpeeds(controller);
    speeds.vx = std::clamp(speeds.vx * speedScale, -DrivebaseConfig::MAX_VELOCITY, DrivebaseConfig::MAX_VELOCITY);
    speeds.vy = std::clamp(speeds.vy * speedScale, -DrivebaseConfig::MAX_VELOCITY, DrivebaseConfig::MAX_VELOCITY);
    return frc::ChassisSpeeds{speeds.vx, speeds.vy, speeds.omega};
  }, fieldOriented);
}

// Special

frc2::CommandPtr SubDrivebase::CharacteriseWheels() {
  static units::radian_t prevGyroAngle = 0_rad;
  static units::radian_t gyroAccumulator = 0_rad;
  static units::radian_t FRinitialWheelDistance = 0_rad;
  static units::radian_t FLinitialWheelDistance = 0_rad;
  static units::radian_t BRinitialWheelDistance = 0_rad;
  static units::radian_t BLinitialWheelDistance = 0_rad;
  static auto limiter = frc::SlewRateLimiter<units::degrees_per_second>{240_deg_per_s / 10_s};
  static units::meter_t drivebaseRadius = DrivebaseConfig::FL_POSITION.Norm();

  return RunOnce([this] {
    prevGyroAngle = GetGyroAngle().Radians();
    gyroAccumulator = 0_rad;
    FRinitialWheelDistance = _frontRight.GetDrivenRotations();
    FLinitialWheelDistance = _frontLeft.GetDrivenRotations();
    BRinitialWheelDistance = _backRight.GetDrivenRotations();
    BLinitialWheelDistance = _backLeft.GetDrivenRotations();
    limiter.Reset(0_deg_per_s);
    Logger::Log("Drivebase/WheelCharacterisation/DrivebaseRadius", drivebaseRadius);
  })
  .AndThen(Drive([] {
    auto speed = limiter.Calculate(100_deg_per_s);
    return frc::ChassisSpeeds{0_mps, 0_mps, speed};
  }, false))
  .AlongWith(frc2::cmd::Wait(1_s).AndThen(frc2::cmd::Run([this] {
    // units::radian_t curGyroAngle = GetHeading().Radians(); using GetGyroAngle() instead
    units::radian_t curGyroAngle = GetGyroAngle().Radians();
    gyroAccumulator = gyroAccumulator + frc::AngleModulus((prevGyroAngle - curGyroAngle));
    prevGyroAngle = curGyroAngle;
    Logger::Log("Drivebase/WheelCharacterisation/GyroAccum", gyroAccumulator);
    Logger::Log("Drivebase/WheelCharacterisation/GyroCur", curGyroAngle);
    Logger::Log("Drivebase/WheelCharacterisation/GyroPrev", prevGyroAngle);

    units::radian_t FRfinalWheelDistance = _frontRight.GetDrivenRotations();
    units::radian_t FLfinalWheelDistance = _frontLeft.GetDrivenRotations();
    units::radian_t BRfinalWheelDistance = _backRight.GetDrivenRotations();
    units::radian_t BLfinalWheelDistance = _backLeft.GetDrivenRotations();

    units::radian_t FRdelta = units::math::abs(FRfinalWheelDistance - FRinitialWheelDistance);
    units::radian_t FLdelta = units::math::abs(FLfinalWheelDistance - FLinitialWheelDistance);
    units::radian_t BRdelta = units::math::abs(BRfinalWheelDistance - BRinitialWheelDistance);
    units::radian_t BLdelta = units::math::abs(BLfinalWheelDistance - BLinitialWheelDistance);

    units::radian_t avgWheelDelta = (FRdelta + FLdelta + BRdelta + BLdelta) / 4.0;
    units::meter_t calcedWheelRadius = ((gyroAccumulator * drivebaseRadius) / avgWheelDelta);

    Logger::Log("Drivebase/WheelCharacterisation/CalcedWheelRadius", calcedWheelRadius);
    Logger::Log("Drivebase/WheelCharacterisation/WheelDistance", avgWheelDelta);

    // Logger::Log("Drivebase/WheelCharacterisation/FLinitialWheelDistance", FLinitialWheelDistance);
    // Logger::Log("Drivebase/WheelCharacterisation/FRinitialWheelDistance", FRinitialWheelDistance);
    // Logger::Log("Drivebase/WheelCharacterisation/BLinitialWheelDistance", BLinitialWheelDistance);
    // Logger::Log("Drivebase/WheelCharacterisation/BRinitialWheelDistance", BRinitialWheelDistance);

    // Logger::Log("Drivebase/WheelCharacterisation/FLfinalWheelDistance", FLfinalWheelDistance);
    // Logger::Log("Drivebase/WheelCharacterisation/FRfinalWheelDistance", FRfinalWheelDistance);
    // Logger::Log("Drivebase/WheelCharacterisation/BLfinalWheelDistance", BLfinalWheelDistance);
    // Logger::Log("Drivebase/WheelCharacterisation/BRfinalWheelDistance", BRfinalWheelDistance);

    Logger::Log("Drivebase/WheelCharacterisation/FLdelta", FLdelta);
    Logger::Log("Drivebase/WheelCharacterisation/FRdelta", FRdelta);
    Logger::Log("Drivebase/WheelCharacterisation/BLdelta", BLdelta);
    Logger::Log("Drivebase/WheelCharacterisation/BRdelta", BRdelta);
  })));
}

frc::Rotation2d SubDrivebase::GetAllianceRelativeGyroAngle() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.value_or(frc::DriverStation::Alliance::kBlue) ==
      frc::DriverStation::Alliance::kBlue) {
    return _gyro.GetRotation2d();
  } else {
    return _gyro.GetRotation2d() - 180_deg;
  }
}