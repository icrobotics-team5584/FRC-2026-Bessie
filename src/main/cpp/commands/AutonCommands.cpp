#include "commands/AutonCommands.h"
#include "commands/FuelCommands.h"
#include "subsystems/SubDrivebase.h"

namespace cmd {
    frc2::CommandPtr DefaultAuton() {
        return frc2::cmd::Print("Default Auton");
    }


    //SOTM Test cases WAITING FOR AUTON PR
    frc2::CommandPtr DriveStraightWhileShooting(){
        return frc2::cmd::Sequence(
        
        ).AlongWith(cmd::ShootOnTheMove());
    }


    frc2::CommandPtr TEMPLATE(){
        return frc2::cmd::Sequence(


        ).AlongWith(cmd::ShootOnTheMove());
    }
}