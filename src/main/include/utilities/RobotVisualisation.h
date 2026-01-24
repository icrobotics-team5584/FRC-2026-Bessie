#pragma once

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include "utilities/MechanismCircle2d.h"
#include "utilities/Logger.h"


class RobotVisualisation {
    public:
    RobotVisualisation(){
        Logger::Log("RobotVisualisation", &_display);
    }
   static RobotVisualisation& GetInstance() {
    static RobotVisualisation intstance;
    return intstance;
   }

   frc::Mechanism2d _display{0.25,0.25};
   frc::MechanismRoot2d* _deployRoot = _display.GetRoot("Deploy Root", 0.05, 0);

   frc::MechanismLigament2d* _deployLigament =
    _deployRoot->Append<frc::MechanismLigament2d>("Deploy", 0.1, 90_deg);
    MechanismCircle2d _intakeWheel{_deployLigament, "IntakeWheel", 0.01, 90_deg};

};