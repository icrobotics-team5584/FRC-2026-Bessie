#include "MotorIO.h"

class NeoIO : public MotorIO {
    public:
        NeoIO(int motorCanID);
        void ConfigMotor() override;
        void SetVoltage(units::volt_t voltage) override;
        void SetPositionTarget(units::degree_t target) override;
        void SetBrakeMode(bool isBreakModeOn) override;
        void StartLoggingMotor(std::string keyName) override;
        units::degree_t GetPosition() override;
        units::degree_t GetPositionTarget() override;
        units::ampere_t GetCurrent() override;
        units::volt_t GetVoltage() override;    
    private:        
        ICSparkMax _motor;
};