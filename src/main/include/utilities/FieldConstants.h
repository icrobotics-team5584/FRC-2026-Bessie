#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>

/* All poses are written with the assumption that the bottom, left pose from a
 * bird's eye perspective is 0_m,0_m. All measurements are taken from the WELDED 
 * reference dimensions converted from inches to meters:
 * https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dwg-complete.pdf
 */

namespace fieldpos {
    typedef struct Zone {
        frc::Translation2d TOP_RIGHT;
        frc::Translation2d BOTTOM_LEFT;
    } Zone;

    /* CONSTANTS */
    constexpr Zone BLUE_ALLIANCE{
        .TOP_RIGHT = {3.965_m, 8.069_m},
        .BOTTOM_LEFT = {0_m, 0_m}
    };

    constexpr Zone NEUTRAL_ZONE{
        .TOP_RIGHT = {12.563_m, 8.069_m},
        .BOTTOM_LEFT = {3.965_m, 0_m}
    };

    int test[4] = { [3] = 3};

    constexpr frc::Translation2d HUB_POSITION{4.626_m, 4.034_m};

constexpr frc::Translation2d TOP_PASSING_ZONE_TOP_RIGHT{16.5_m, 8_m};
constexpr frc::Translation2d TOP_PASSING_ZONE_BOTTOM_LEFT{4_m, 4_m};

constexpr frc::Translation2d BOTTOM_PASSING_ZONE_TOP_RIGHT{16.5_m, 4_m};
constexpr frc::Translation2d BOTTOM_PASSING_ZONE_BOTTOM_LEFT{4_m, 0_m};


constexpr frc::Translation3d TOP_ALLIANCE_ZONE_POSITION{2.5_m, 5.5_m, 0_m};
constexpr frc::Translation3d BOTTOM_ALLIANCE_ZONE_POSITION{2.5_m, 2.75_m, 0_m};

}  