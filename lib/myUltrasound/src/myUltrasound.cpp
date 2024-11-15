#include <myUltrasound.h>

myUltrasound::myUltrasound() {}

myUltrasound::myUltrasound(int32_t boards, int RX, int TX) {
    this->distance = -1;
    this->RX = RX;
    this->TX = TX;
    this->boards = boards;
    this->buf[4] = {0};
}

// 初始化函数，传入参数为波特率
bool myUltrasound::Init() {
    Serial1.setPins(RX, TX);
    Serial1.begin(boards);
    return true;
}

// 自动计算距离的超声波模块，实机实际用的计算函数，推荐使用
void myUltrasound::autoComputeDistance() {
    uint8_t buffer[4];
    if (Serial1.available() >= 4) {
        for (int i = 0; i < 4; i++) {
            buffer[i] = Serial1.read();
            buf[i] = buffer[i];
        }
        if (buffer[0] == 0xFF) {                                             // 检查帧头
            uint16_t rawDistance = (buffer[1] << 8) + buffer[2];             // 计算距离
            uint8_t sum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;        // 计算校验和
            if (sum == buffer[3]) {                                          // 校验
                distance = rawDistance;
            }else {
                distance = -1;
            }
        }else {
            while(Serial1.available()) Serial1.read();
        }
    }
}

// 受控计算距离的超声波模块，因为仅测试使用，未经优化，慎用
void myUltrasound::ctrlComputeDistance() {
    digitalWrite(TX, LOW);
    delayMicroseconds(500);
    digitalWrite(TX, HIGH);
    uint8_t buffer[4];
    if (Serial1.available() >= 4) {
        for (int i = 0; i < 4; i++) {
            buffer[i] = Serial1.read();
            buf[i] = buffer[i];
        }
        if (buffer[0] == 0xFF) {                                             // 检查帧头
            uint16_t rawDistance = (buffer[1] << 8) + buffer[2];             // 计算距离
            uint8_t sum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;        // 计算校验和
            if (sum == buffer[3]) {                                          // 校验
                distance = rawDistance;
            }else {
                distance = -1;
            }
        }        
    }
}

// 得到测量的距离，如果是-1，代表测量错误
int16_t myUltrasound::getDistance() {
    return distance;
}

byte* myUltrasound::getBuf() {
    return buf;
}