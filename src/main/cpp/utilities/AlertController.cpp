#include "utilities/AlertController.h"

#include "utilities/Logger.h"

namespace AlertController {

// creates an alert config used in the following functions
motorAlertConfig Config(frc::Alert temperatureAlert, frc::Alert currentAlert,
  frc::Alert recordedTemperatureAlert, frc::Alert recordedCurrentAlert, frc::Timer highCurrentTimer,
  units::celsius_t maxDegrees, units::ampere_t maxCurrent) {
  return {temperatureAlert, currentAlert, recordedCurrentAlert, recordedTemperatureAlert,
    highCurrentTimer, maxDegrees, maxCurrent};
}
// updates the alert for the temperature. automatically updates to true or false
void UpdateTemperatureAlert(motorAlertConfig& config, units::celsius_t motorTemperature) {
  if (motorTemperature > config.maxDegrees) {
    config.temperatureAlert.Set(true);
    config.recordedTemperatureAlert.Set(true);
  } else {
    config.temperatureAlert.Set(false);
  }
}
// updates the alert for the current. automatically updates to true if has been over 3s or false.
void UpdateCurrentAlert(motorAlertConfig& config, units::ampere_t motorCurrent) {
  if (motorCurrent > config.maxCurrent) {
    config.highCurrentTimer.Start();
    if (config.highCurrentTimer.Get() > 3_s) {
      config.currentAlert.Set(true);
      config.recordedCurrentAlert.Set(true);
    }

  } else {
    config.currentAlert.Set(false);
    config.highCurrentTimer.Reset();
  }
}

}  // namespace AlertController