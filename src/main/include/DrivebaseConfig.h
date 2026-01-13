#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/velocity.h>

namespace DrivebaseConfig {
  // Drive controls
  constexpr units::meters_per_second_t MAX_VELOCITY = 5_mps;
  constexpr units::meters_per_second_t MAX_DRIVE_TO_POSE_VELOCITY = 1_mps;
  constexpr units::turns_per_second_t MAX_ANGULAR_VELOCITY = 290_deg_per_s;  // CHANGE TO 720\[]

  static constexpr units::turns_per_second_squared_t MAX_ANG_ACCEL{std::numbers::pi};

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

  const units::turn_t FRONT_RIGHT_MAG_OFFSET = -0.37451171875_tr;
  const units::turn_t FRONT_LEFT_MAG_OFFSET = -0.943848_tr;
  const units::turn_t BACK_RIGHT_MAG_OFFSET = -0.46435546875_tr;
  const units::turn_t BACK_LEFT_MAG_OFFSET = -0.353515625_tr;

  const frc::PIDController TELE_TRANSLATION_PID{7.0, 0.0, 0.0};
  const frc::ProfiledPIDController<units::radian> TELE_ROTATION_PID{
              3.0, 0, 0, {MAX_ANGULAR_VELOCITY, MAX_ANG_ACCEL}};
}