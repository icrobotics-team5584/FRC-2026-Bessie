#include "utilities/AlertController.h"

#include <unordered_map>
namespace AlertController {
std::vector<std::weak_ptr<AlertConfig>> configList;

void RegisterAlertConfig(std::weak_ptr<AlertConfig> config) {
  configList.emplace_back(config);
}

void UpdateAllAlerts(AlertConfig& config, MotorTelemetryConfig& telemetryConfig) {
  UpdateTemperatureAlert(config, telemetryConfig);
  UpdateCurrentAlert(config, telemetryConfig);
}

void UpdateTemperatureAlert(AlertConfig& config, MotorTelemetryConfig& telemetryConfig) {
  if (telemetryConfig.motorTemperature > config.maxDegrees && !config.hasReachedHighTemperature) {
    config.responsiveHighTemperatureAlert.Set(true);

    config.tempuratureReachedCount += 1;

    OverideStaticAlerts(config);
    recordLastMotorTemperature(config, telemetryConfig);

    config.staticHighTemperatureAlert.Set(true);
    config.hasReachedHighTemperature = true;
  } else if (telemetryConfig.motorTemperature <= config.maxDegrees) {
    config.responsiveHighTemperatureAlert.Set(false);
    config.hasReachedHighTemperature = false;
  }
}

void UpdateCurrentAlert(AlertConfig& config, MotorTelemetryConfig& telemetryConfig) {
  if (telemetryConfig.motorCurrent > config.maxCurrent && !config.hasReachedHighCurrent) {
    config.highCurrentTimer.Start();
    if (config.highCurrentTimer.Get() > 3_s) {
      config.responsiveHighCurrentAlert.Set(true);

      config.currentReachedCount += 1;

      OverideStaticAlerts(config);
      recordLastMotorCurrent(config, telemetryConfig);

      config.staticHighCurrentAlert.Set(true);
      config.hasReachedHighCurrent = true;
    }

  } else if (telemetryConfig.motorCurrent <= config.maxCurrent) {
    config.responsiveHighCurrentAlert.Set(false);
    config.highCurrentTimer.Reset();
    config.hasReachedHighCurrent = false;
  }
}

void recordLastMotorTemperature(AlertConfig& config, MotorTelemetryConfig& telemetryConfig) {
  config.lastRecordedHighTemperatureAlertInfo.SetText(
    config.motorString + " Last Recorded High Temperature: " +
    std::to_string((int)telemetryConfig.motorTemperature) + "_DegC");

  config.lastRecordedHighTemperatureAlertInfo.Set(true);
}
void recordLastMotorCurrent(AlertConfig& config, MotorTelemetryConfig& telemetryConfig) {
  config.lastRecordedHighTemperatureAlertInfo.SetText(
    config.motorString +
    " Last Recorded High Current: " + std::to_string((int)telemetryConfig.motorCurrent) + "_A");

  config.lastRecordedHighTemperatureAlertInfo.Set(true);
}

void OverideStaticAlerts(AlertConfig& config) {
  config.staticHighTemperatureAlert.SetText(
    config.motorString +
    " MAX TEMPERATURE REACHED: " + std::to_string(config.tempuratureReachedCount) + " times");

  config.staticHighCurrentAlert.SetText(config.motorString + " MAX CURRENT REACHED: " +
                                        std::to_string(config.currentReachedCount) + " times");
}

frc2::CommandPtr ForceRemoveAllAlerts() {
  return frc2::cmd::RunOnce([] {
    for (std::weak_ptr<AlertConfig>& configWeak : configList) {
      if (auto config = configWeak.lock()) {  // lock once
        config->responsiveHighTemperatureAlert.Set(false);
        config->responsiveHighCurrentAlert.Set(false);
        config->staticHighTemperatureAlert.Set(false);
        config->staticHighCurrentAlert.Set(false);

        config->lastRecordedHighTemperatureAlertInfo.Set(false);
        config->lastRecordedHighCurrentAlertInfo.Set(false);
        config->highCurrentTimer.Reset();
      }
    }
  })
    .WithName("Force Remove All Alerts")
    .IgnoringDisable(true);
}

}  // namespace AlertController