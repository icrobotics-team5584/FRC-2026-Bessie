#pragma once
#include <frc/Alert.h>
#include <frc/Timer.h>

#include <units/current.h>
#include <units/temperature.h>

struct motorAlertConfig {
  frc::Alert temperatureAlert;
  frc::Alert currentAlert;
  frc::Timer highCurrentTimer;

  units::celsius_t maxDegrees;
  units::ampere_t maxCurrent;
};

namespace AlertController {

// updates the alert for the temperature. automatically updates to true or false
void UpdateTemperatureAlert(motorAlertConfig& config, units::celsius_t motorTemperature);

// updates the alert for the current. automatically updates to true if has been over 3s or false.
void UpdateCurrentAlert(motorAlertConfig& config, units::ampere_t motorCurrent);
};  // namespace AlertController
