#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/velocity.h>
#include <pathplanner/lib/config/PIDConstants.h>

namespace DrivebaseConfig {
  // Drive controls
  constexpr units::meters_per_second_t MAX_VELOCITY = 2_mps;
  constexpr units::meters_per_second_squared_t MAX_ACCEL = 20_mps_sq;
  constexpr units::meters_per_second_t MAX_DRIVE_TO_POSE_VELOCITY = 2_mps;
  constexpr units::turns_per_second_t MAX_ANGULAR_VELOCITY = 300_deg_per_s;

  static constexpr units::meters_per_second_squared_t MAX_P2P_ACCEL = 3_mps_sq;
  static constexpr units::turns_per_second_squared_t MAX_P2P_ANGULAR_ACCEL = 3_tr_per_s_sq;

  static constexpr units::turns_per_second_squared_t MAX_ANGULAR_ACCEL{std::numbers::pi};

  static constexpr double MAX_JOYSTICK_ACCEL = 5;
  static constexpr double MAX_ANGULAR_JOYSTICK_ACCEL = 3;
  static constexpr double JOYSTICK_DEADBAND = 0.08;
  static constexpr double TRANSLATION_SCALING = 2;  // Set to 1 for linear scaling
  static constexpr double ROTATION_SCALING = 1;     // Set to 1 for linear scaling

  // Swerve config
  constexpr frc::Translation2d FL_POSITION{+0.281_m, +0.281_m};
  constexpr frc::Translation2d FR_POSITION{+0.281_m, -0.281_m};
  constexpr frc::Translation2d BL_POSITION{-0.281_m, +0.281_m};
  constexpr frc::Translation2d BR_POSITION{-0.281_m, -0.281_m};

  const units::turn_t FRONT_RIGHT_MAG_OFFSET = -0.446044921875_tr;
  const units::turn_t FRONT_LEFT_MAG_OFFSET = -0.26611328125_tr;
  const units::turn_t BACK_RIGHT_MAG_OFFSET = -0.675048828125_tr;
  const units::turn_t BACK_LEFT_MAG_OFFSET = -0.824951171875_tr;

  const frc::ProfiledPIDController<units::meters> TELE_TRANSLATION_PID{2.5, 0, 0, {MAX_VELOCITY, MAX_ACCEL}};
  const frc::ProfiledPIDController<units::radian> TELE_ROTATION_PID{2.0, 0, 0, {MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL}};

  const pathplanner::PIDConstants AUTO_TRANSLATION_PID{3.2, 0.0, 0.3};
  const pathplanner::PIDConstants AUTO_ROTATION_PID{1.5, 0.0, 0.0};
}