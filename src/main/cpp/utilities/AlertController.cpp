#include "utilities/AlertController.h"

namespace AlertController {

// creates an alert config from the alerts and a timer and is used in updating alerts
motorAlertConfig AlertController::Config(frc::Alert temperatureAlert, frc::Alert currentAlert,
  frc::Timer highCurrentTimer, units::celsius_t maxDegrees, units::ampere_t maxCurrent) {
  return {temperatureAlert, currentAlert, highCurrentTimer, maxDegrees, maxCurrent};
}
// updates the alert for the temperature. automatically updates to true or false
void UpdateTemperatureAlert(motorAlertConfig& config, units::celsius_t motorTemperature) {
  if (motorTemperature > config.maxDegrees) {
    config.temperatureAlert.Set(true);
  } else {
    config.temperatureAlert.Set(false);
  }
}
// updates the alert for the current. automatically updates to true if has been over 3s or false.
void AlertController::UpdateCurrentAlert(motorAlertConfig& config, units::ampere_t motorCurrent) {
  if (motorCurrent > config.maxCurrent) {
    config.highCurrentTimer.Start();
    if (config.highCurrentTimer.Get() > 3_s) {
      config.currentAlert.Set(true);
    }

  } else {
    config.currentAlert.Set(false);
    config.highCurrentTimer.Reset();
  }
}

}  // namespace AlertController