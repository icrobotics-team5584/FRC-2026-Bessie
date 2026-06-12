// Smoke tests proving the test toolchain works: gtest runs, the HAL sim
// clock can be stepped deterministically, and the executable's working
// directory is the project root (required before any test touches
// SubVision, which loads the AprilTag layout from src/main/deploy).

#include <frc/Timer.h>
#include <frc/simulation/SimHooks.h>

#include <filesystem>
#include <units/time.h>

#include "gtest/gtest.h"

TEST(BuildSanity, GTestRuns) {
  EXPECT_TRUE(true);
}

TEST(BuildSanity, HalSimTimeStepping) {
  frc::sim::PauseTiming();
  units::second_t start = frc::Timer::GetFPGATimestamp();
  frc::sim::StepTiming(20_ms);
  units::second_t end = frc::Timer::GetFPGATimestamp();
  frc::sim::ResumeTiming();

  EXPECT_NEAR((end - start).value(), 0.020, 1e-6);
}

TEST(BuildSanity, WorkingDirIsProjectRoot) {
  EXPECT_TRUE(std::filesystem::exists("src/main/deploy/2026-rebuilt.json"))
    << "Test executable must run with working directory = project root "
       "(anchored by ChdirToProjectRoot in src/test/cpp/main.cpp). Without "
       "it, SubVision's constructor throws on the missing AprilTag layout "
       "file. Current dir: "
    << std::filesystem::current_path();
}
