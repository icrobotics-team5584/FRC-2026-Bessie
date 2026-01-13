#include "commands/AutonCommands.h"

namespace cmd {
    frc2::CommandPtr DefaultAuton() {
        return frc2::cmd::Print("Default Auton");
    }
}