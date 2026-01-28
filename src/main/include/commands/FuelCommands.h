#pragma once
#include <frc2/command/Command.h>
#include <frc/geometry/Translation2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();

frc2::CommandPtr StationaryAimAt(frc::Translation2d target);
frc2::CommandPtr StationaryShootWhenReady();
}
