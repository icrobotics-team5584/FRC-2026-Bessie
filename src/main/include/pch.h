#pragma once

// Precompiled header for stable vendor/WPILib headers.
// Only include headers here that:
//   1. Come from external libraries (WPILib, vendor deps)
//   2. Rarely change between builds
//   3. Are used across many .cpp files
// NEVER include project headers here.
//
// NOTE: The roboRIO cross-compiler (arm-frc2026-linux-gnueabi-g++) has
// limited address space for PCH files on Windows. WPILib headers like
// CommandPtr.h, SmartDashboard.h, and geometry headers exceed this
// limit. Only lightweight <units/*> headers are safe here.

// Units (used everywhere — lightweight template headers)
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
