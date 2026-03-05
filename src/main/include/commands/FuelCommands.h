#pragma once
#include <frc2/command/Command.h>
#include <frc/geometry/Translation2d.h>

namespace cmd {
frc2::CommandPtr IntakeSequence();
frc2::CommandPtr OuttakeSequence();

frc2::CommandPtr StationaryShootAt(frc::Translation2d target);
frc2::CommandPtr BackupShoot();

frc2::CommandPtr ShootWhenReady();
frc2::CommandPtr AimOnTheMove();
frc2::CommandPtr ShootOnTheMove();
frc2::CommandPtr ToggleBrakeCoast();
frc2::CommandPtr EjectFuel();
frc2::CommandPtr DisableAllOverrides();
frc2::CommandPtr ForceShoot();

bool IsReadyToShoot();
}
