#pragma once
#include <frc2/command/Command.h>
#include <frc/geometry/Translation2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();

frc2::CommandPtr StationaryShootAt(frc::Translation2d target);
}
