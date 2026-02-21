#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose2d.h>

namespace fieldpos {
constexpr frc::Translation2d BLUE_ALLIANCE_ZONE_TOP_RIGHT{4_m, 8_m};
constexpr frc::Translation2d BLUE_ALLIANCE_ZONE_BOTTOM_LEFT{0_m, 0_m};

constexpr frc::Translation2d TOP_PASSING_ZONE_TOP_RIGHT{16.5_m, 8_m};
constexpr frc::Translation2d TOP_PASSING_ZONE_BOTTOM_LEFT{4_m, 4_m};

constexpr frc::Translation2d BOTTOM_PASSING_ZONE_TOP_RIGHT{16.5_m, 4_m};
constexpr frc::Translation2d BOTTOM_PASSING_ZONE_BOTTOM_LEFT{4_m, 0_m};

constexpr frc::Translation3d HUB_POSITION{4.625_m, 4.018_m, 0_m};

constexpr frc::Translation3d TOP_ALLIANCE_ZONE_POSITION{2.5_m, 5.5_m, 0_m};
constexpr frc::Translation3d BOTTOM_ALLIANCE_ZONE_POSITION{2.5_m, 2.75_m, 0_m};

//auton positions (assuming robot with bumpers is 91cm by 91cm; blue alliance only)
constexpr frc::Pose2d NEUTRAL_IN_LEFT{7.80_m, 7.00_m, 270_deg};
constexpr frc::Pose2d NEUTRAL_END_LEFT{7.80_m, 4.75_m, 270_deg};
constexpr frc::Pose2d NEUTRAL_IN_RIGHT{7.80_m, 1.07_m, 90_deg};
constexpr frc::Pose2d NEUTRAL_END_RIGHT{7.80_m, 3.32_m, 90_deg};

constexpr frc::Pose2d START_BUMP_LEFT{3.58_m, 5.80_m, 0_deg};
constexpr frc::Pose2d START_TRENCH_LEFT{3.58_m, 7.50_m, 0_deg};
constexpr frc::Pose2d START_BUMP_RIGHT{3.58_m, 2.27_m, 0_deg};
constexpr frc::Pose2d START_TRENCH_RIGHT{3.58_m, 0.57_m, 0_deg};

constexpr frc::Pose2d DEPOT_IN{1.33_m, 5.96_m, 180_deg};
constexpr frc::Pose2d DEPOT_END{0.75_m, 5.96_m, 180_deg};
constexpr frc::Pose2d OUTPOST{0.60_m, 0.67_m, 0_deg};
constexpr frc::Pose2d TOWER{1.65_m, 3.75_m, 0_deg};
}  // namespace fieldpos