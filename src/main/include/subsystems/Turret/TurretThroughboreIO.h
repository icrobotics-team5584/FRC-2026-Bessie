#include "EncoderIO.h"

class TurretThroughboreIO : public EncoderIO {
    public:
        TurretThroughboreIO(int encoderCanID);
        void ConfigEncoder();
        units::degree_t GetPosition();
    private:
        frc::DutyCycleEncoder _encoder;
};