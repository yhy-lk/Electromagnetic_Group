#include <Arduino.h>

class myUltrasound {
public:
    myUltrasound();
    myUltrasound(int32_t boards, int TX, int RX);

    // 初始化函数，传入参数为波特率
    bool Init();

    // 自动计算距离的超声波模块，实机实际用的计算函数，推荐使用
    void autoComputeDistance();

    // 受控计算距离的超声波模块，因为仅测试使用，未经优化，慎用
    void ctrlComputeDistance();

    // 得到测量的距离，如果是-1，代表测量错误
    int16_t getDistance();

    byte* getBuf();

private:
    int16_t distance;
    int TX, RX;
    uint32_t boards;
    byte buf[10];
};

