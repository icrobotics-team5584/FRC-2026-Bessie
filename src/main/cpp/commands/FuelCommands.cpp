#include "commands/FuelCommands.h"

#include "subsystems/SubDeploy.h"
#include "subsystems/SubIndexer.h"
#include "subsystems/SubIntake.h"
namespace cmd {
frc2::CommandPtr IntakeSequence() {
  return SubDeploy::GetInstance()
    .DeployIntake()
    .AlongWith(SubIntake::GetInstance().IntakeOn())
    .AlongWith(SubIndexer::GetInstance().IndexerOn());
}

}  // namespace cmd