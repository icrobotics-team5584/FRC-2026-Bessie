#pragma once
#include <frc2/command/Command.h>
#include <frc/geometry/Pose2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();

frc2::CommandPtr StationaryShootAt(frc::Pose2d target);
}
