#pragma once
#include <frc2/command/Command.h>
#include <frc/geometry/Translation2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();

frc2::CommandPtr StationaryShootAt(frc::Translation2d target);

frc2::CommandPtr ShootWhenReady();
frc2::CommandPtr AimOnTheMove();
frc2::CommandPtr ShootOnTheMove();

bool SubSystemsOnTarget();
}
