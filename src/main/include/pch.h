#pragma once

// Precompiled header for stable vendor/WPILib headers.
// Only include headers here that:
//   1. Come from external libraries (WPILib, vendor deps)
//   2. Rarely change between builds
//   3. Are used across many .cpp files
// NEVER include project headers here.

// WPILib command framework
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

// Units (used everywhere)
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/current.h>
#include <units/mass.h>
#include <units/temperature.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/torque.h>

// Geometry (used everywhere)
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/kinematics/SwerveModuleState.h>

// Common WPILib
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
