#include "commands/FuelCommands.h"

#include "subsystems/SubIntake.h"
#include "subsystems/SubStorage.h"

namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubIntake::GetInstance()
    .IntakeOn()
    .AlongWith(SubStorage::GetInstance().StorageOn())
    .Until([] { return SubStorage::GetInstance().IsFull(); });
}
}  // namespace cmd