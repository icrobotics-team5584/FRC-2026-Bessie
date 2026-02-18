#include "EncoderIO.h"

class GeneralEncoder {
    public:
        GeneralEncoder(int encoderCanID);
        void ConfigEncoder();
        units::degree_t GetPosition();

    private:
        std::unique_ptr<EncoderIO> _io;
};