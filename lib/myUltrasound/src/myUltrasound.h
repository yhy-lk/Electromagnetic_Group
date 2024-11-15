#include <Arduino.h>

class myUltrasound {
public:
    myUltrasound();
    myUltrasound(int32_t boards, int TX, int RX);

    bool Init();

    void autoComputeDistance();

    void ctrlComputeDistance();

    int16_t getDistance() const;

    const byte* getBuf() const;

private:
    int16_t distance;
    int TX, RX;
    uint32_t boards;
    byte buf[10];
};

