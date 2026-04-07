#include "DrivebaseConfig.h"
#include "utilities/BotVars.h"

namespace DrivebaseConfig {
  const units::turn_t FRONT_RIGHT_MAG_OFFSET = BotVars::Choose(-0.94384765625_tr - 0.25_tr, -0.446044921875_tr);
  const units::turn_t FRONT_LEFT_MAG_OFFSET = BotVars::Choose(-0.37451171875_tr + 0.25_tr, -0.26611328125_tr);
  const units::turn_t BACK_RIGHT_MAG_OFFSET = BotVars::Choose(-0.353515625_tr + 0.25_tr, -0.675048828125_tr);
  const units::turn_t BACK_LEFT_MAG_OFFSET = BotVars::Choose(-0.464111328125_tr - 0.25_tr, -0.824951171875_tr);

  const frc::PIDController P2P_TRANSLATION_PID{5, 0, 0};
  const frc::PIDController P2P_ROTATION_PID{33, 0, 0};
}
