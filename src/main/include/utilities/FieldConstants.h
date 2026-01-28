#pragma once
#include <frc/geometry/Pose2d.h>

/*
All poses are are of Pose2d. Alls poses are written under the assumption that
the blue alliances rightmost corner is 0,0
*/

namespace fieldpos {
constexpr frc::Translation2d BLUE_ALLIANCE_ZONE_TOP_RIGHT{4_m, 8_m};
constexpr frc::Translation2d BLUE_ALLIANCE_ZONE_BOTTOM_LEFT{0_m, 0_m};

constexpr frc::Translation2d TOP_PASSING_ZONE_TOP_RIGHT{16.5_m, 8_m};
constexpr frc::Translation2d TOP_PASSING_ZONE_BOTTOM_LEFT{4_m, 4_m};

constexpr frc::Translation2d BOTTOM_PASSING_ZONE_TOP_RIGHT{16.5_m, 4_m};
constexpr frc::Translation2d BOTTOM_PASSING_ZONE_BOTTOM_LEFT{4_m, 0_m};

constexpr frc::Translation3d HUB_POSITION{4_m, 4_m, 0_m};

constexpr frc::Translation3d TOP_ALLIANCE_ZONE_POSITION{2.5_m, 5.5_m, 0_m};
constexpr frc::Translation3d BOTTOM_ALLIANCE_ZONE_POSITION{2.5_m, 2.75_m, 0_m};

} 