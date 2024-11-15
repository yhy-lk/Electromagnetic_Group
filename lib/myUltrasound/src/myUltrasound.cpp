//=============================================================================================
//
// Date			Author          Notes
// 15/11/2024	YHY             Initial release
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// 头文件
#include <myUltrasound.h>

//--------------------------------------------------------------------------------------------
/**
 * @brief 无参构造函数
 *        @note
 *          - 实际不使用，仅供测试代码BUG
 * 
 * @param 无
 * 
 * @return  
 *     - 无
 */myUltrasound::myUltrasound() {}



//--------------------------------------------------------------------------------------------
/**
 * @brief 有参构造函数
 * 
 * @param boards UART波特率
 * @param RX UART的RX
 * @param TX UART的TX
 *
 * @return  
 *     - 无
 */
myUltrasound::myUltrasound(int32_t boards, int RX, int TX) {
    this->distance = -1;
    this->RX = RX;
    this->TX = TX;
    this->boards = boards;
    this->buf[4] = {0};
}


//--------------------------------------------------------------------------------------------
/**
 * @brief 超声波模块初始化
 *        @note
 *          - 这个函数会利用构造函数传入的参数初始化UART
 * 
 * @param 无
 *
 * @return  
 *     - 无
 */
bool myUltrasound::Init() {
    Serial1.setPins(RX, TX);
    Serial1.begin(boards);
    return true;
}


//--------------------------------------------------------------------------------------------
/**
 * @brief 自动计算距离
 *        @note
 *          - 这个函数会根据协议自动计算距离
 * 
 * @param 无
 *
 * @return  
 *     - 无
 */
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
            while(Serial1.available()) Serial1.read();                       // 帧头对不上，丢弃该帧
        }
    }
}


//--------------------------------------------------------------------------------------------
/**
 * @brief 受控计算距离
 *        @note
 *          - 这个函数会根据协议受控计算距离
 * 
 * @param 无
 *
 * @return  
 *     - 无
 */
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
        }else {
            while(Serial1.available()) Serial1.read();                       // 帧头对不上，丢弃该帧
        }        
    }
}


//--------------------------------------------------------------------------------------------
/**
 * @brief 得到距离
 * 
 * @param 无
 *
 * @return  
 *     - 超声波模块测量的距离
 */
int16_t myUltrasound::getDistance() const{
    return distance;
}


//--------------------------------------------------------------------------------------------
/**
 * @brief 得到最后一次数据包
 *        @note
 *          - 这个函数仅供测试使用，因为它返回了私有成员的地址，这违反了封装性原则
 * 
 * @param 无
 *
 * @return  
 *     - 数据包头指针
 */
const byte* myUltrasound::getBuf() const {
    return buf;
}