#pragma once

class TurretEncoderIO {
    public:
        virtual void ConfigEncoder() = 0;
        virtual double GetPosition() = 0;
        virtual bool IsConnected() = 0;
};