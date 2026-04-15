#pragma once
#include <frc/Alert.h>
#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <units/current.h>
#include <units/temperature.h>

namespace AlertController {

struct AlertConfig {
  std::string motorString;

  units::celsius_t maxDegrees;
  units::ampere_t maxCurrent;

  frc::Alert responsiveHighTemperatureAlert;
  frc::Alert responsiveHighCurrentAlert;

  frc::Alert staticHighTemperatureAlert;
  frc::Alert staticHighCurrentAlert;

  frc::Alert lastRecordedHighTemperatureAlertInfo;
  frc::Alert lastRecordedHighCurrentAlertInfo;

  frc::Timer highCurrentTimer;

  int tempuratureReachedCount = 0;
  int currentReachedCount = 0;

  bool hasReachedHighTemperature = false;
  bool hasReachedHighCurrent = false;

  AlertConfig(
    std::string motorName, units::celsius_t motorMaxTemperature, units::ampere_t motorMaxCurrent)

    : responsiveHighTemperatureAlert(
        motorName + " TEMPURATURE HIGH!", frc::Alert::AlertType::kWarning),

      responsiveHighCurrentAlert(motorName + " CURRENT HIGH!", frc::Alert::AlertType::kWarning),

      staticHighTemperatureAlert(motorName + " MAX TEMPERATURE REACHED: " +  // PlaceHolder Alert
                                   " 0 " + " times",
        frc::Alert::AlertType::kWarning),

      staticHighCurrentAlert(
        motorName + " MAX CURRENT REACHED: " + " 0 " + " times",  // PlaceHolder Alert
        frc::Alert::AlertType::kWarning),

      lastRecordedHighTemperatureAlertInfo(
        motorName + " Last Recorded High Temperature: " + " 0_degC",
        frc::Alert::AlertType::kInfo),  // PlaceHolder Alert
      lastRecordedHighCurrentAlertInfo(motorName + " Last Recorded High Current: " + " 0_A",
        frc::Alert::AlertType::kInfo),  // Placeholder Alert

      highCurrentTimer() {
    maxDegrees = motorMaxTemperature;
    maxCurrent = motorMaxCurrent;
    motorString = motorName;
  }
};

struct MotorTelemetry {
  units::celsius_t motorTemperature;
  units::ampere_t motorCurrent;
};
void RegisterAlertConfig(std::weak_ptr<AlertConfig> config);

void UpdateAllAlerts(AlertConfig& config, MotorTelemetry& telemetryConfig);

void UpdateTemperatureAlert(AlertConfig& config, MotorTelemetry& telemetryConfig);
void UpdateCurrentAlert(AlertConfig& config, MotorTelemetry& telemetryConfig);

void recordLastMotorTemperature(AlertConfig& config, MotorTelemetry& telemetryConfig);
void recordLastMotorCurrent(AlertConfig& config, MotorTelemetry& telemetryConfig);

void OverrideStaticAlerts(AlertConfig& config);

frc2::CommandPtr ForceRemoveAllAlerts();

};  // namespace AlertController
