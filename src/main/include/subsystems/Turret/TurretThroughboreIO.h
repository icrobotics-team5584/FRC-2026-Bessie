#include "TurretEncoderIO.h"

class TurretThroughboreIO : public TurretEncoderIO {
    public:
        TurretThroughboreIO(int encoderCanID);
        void ConfigEncoder();
        double GetPosition();
        bool IsConnected();
    private:
        frc::DutyCycleEncoder _encoder;
};