#include "TurretEncoderIO.h"

class TurretCancoderIO : public TurretEncoderIO {
    public:
        TurretCancoderIO(int encoderCanID);
        void ConfigEncoder();
        double GetPosition();
        bool IsConnected();
    private:
        ctre::phoenix6::hardware::CANcoder _encoder;
        ctre::phoenix6::configs::CANcoderConfiguration _encoderConfig;
};