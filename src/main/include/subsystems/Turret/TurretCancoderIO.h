#include "EncoderIO.h"

class TurretCancoderIO : public EncoderIO {
    public:
        TurretCancoderIO(int encoderCanID);
        void ConfigEncoder();
        units::degree_t GetPosition();
    private:
        ctre::phoenix6::hardware::CANcoder _encoder;
        ctre::phoenix6::configs::CANcoderConfiguration _encoderConfig;
};