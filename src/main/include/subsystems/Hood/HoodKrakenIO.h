#include "MotorIO.h"

class HoodKrakenIO : public MotorIO {
    public:
        HoodKrakenIO(int motorCanID);
        void ConfigMotor() override;
        void SetVoltage(units::volt_t voltage) override;
        void SetPosition(units::degree_t pos) override;
        void SetPositionTarget(units::degree_t target) override;
        void SetBrakeMode(bool isBreakModeOn) override;
        void StartLoggingMotor(std::string keyName) override;
        void StopMotor() override;
        units::degree_t GetPosition() override;
        units::degree_t GetPositionTarget() override;
        units::degree_t GetPositionError() override;
        units::ampere_t GetCurrent() override;
        units::volt_t GetVoltage() override;    
    private:
        ctre::phoenix6::hardware::TalonFX _motor;
        
        units::degree_t _desiredAngle;
};