#include <myOdometry.h>

#define EncoderPCNT


/*

*/
myOdometer::myOdometer() {}

myOdometer::myOdometer(uint16_t leftEncoderPin, uint16_t rightEncoderPin, uint16_t pulsesPerRevolution, float wheelRadius, int reductionRatio, float wheelSpacing, float frequency) {
    myOdometer(leftEncoderPin, rightEncoderPin, 1, 1, pulsesPerRevolution, wheelRadius, reductionRatio, wheelSpacing, frequency);
}

myOdometer::myOdometer(uint16_t leftEncoderPin, uint16_t rightEncoderPin, uint16_t leftTurnPin, uint16_t rightTurnPin, uint16_t pulsesPerRevolution, float wheelRadius, int reductionRatio, float wheelSpacing, float frequency) {
    this->leftEncoderPin = leftEncoderPin;
    this->rightEncoderPin = rightEncoderPin;

    this->leftTurnPin = leftTurnPin;
    this->rightTurnPin = rightTurnPin;

    this->leftLastPulseTime = 0;
    this->leftPulsePeriod = 0;

    this->rightLastPulseTime = 0;
    this->rightPulsePeriod = 0;
    
    this->leftWheelVelocity = 0.0f;
    this->rightWheelVelocity = 0.0f;

    this->pulsesPerRevolution = pulsesPerRevolution;

    this->wheelCircumference = 2 * PI * wheelRadius;

    this->reductionRatio = reductionRatio;

    this->wheelSpacing = wheelSpacing;

    this->frequency = frequency;

    this->position.x = 0.0f;
    this->position.y = 0.0f;
    this->position.yaw = 0.0f;
}

bool myOdometer::Init() {

#if defined(EncoderISR)

    attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

#elif defined(EncoderPCNT)

    setupPcntUnit(PCNT_UNIT_0, leftEncoderPin);
    setupPcntUnit(PCNT_UNIT_1, rightEncoderPin);

#endif
    return true;
}

// 配置 PCNT 单元的函数
void myOdometer::setupPcntUnit(pcnt_unit_t unit, int pulse_gpio) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pulse_gpio,   // 输入脉冲信号的 GPIO 引脚
        .ctrl_gpio_num = -1,            // 不需要控制引脚
        .lctrl_mode = PCNT_MODE_KEEP,   // 控制模式保持
        .hctrl_mode = PCNT_MODE_KEEP,   // 控制模式保持
        .pos_mode = PCNT_COUNT_INC,     // 上升沿计数增加
        .neg_mode = PCNT_COUNT_DIS,     // 下降沿不计数
        .counter_h_lim = 1000,          // 设置高阈值
        .counter_l_lim = -1000,          // 设置低阈值
        .unit = unit,                   // 指定 PCNT 单元
        .channel = PCNT_CHANNEL_0      // 使用通道 0
    };

    // 配置 PCNT 单元
    pcnt_unit_config(&pcnt_config);

    // 清除计数器值
    pcnt_counter_clear(unit);

    // 启动计数器
    pcnt_counter_resume(unit);
}

void myOdometer::resetVelocity() {
    leftWheelVelocity = 0.0f;
    rightWheelVelocity = 0.0f;
}

float myOdometer::getLeftWheelVelocity() const {
    return leftWheelVelocity;
}

float myOdometer::getRightWheelVelocity() const {
    return rightWheelVelocity;
}


void myOdometer::update() {
    int16_t leftCnt = 0;
    int16_t rightCnt = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &leftCnt);
    pcnt_get_counter_value(PCNT_UNIT_1, &rightCnt);
    // 清除计数器值
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    leftWheelVelocity = (leftCnt * wheelCircumference * frequency) / (float)(pulsesPerRevolution * reductionRatio);
    leftWheelVelocity *= 2 * digitalRead(leftTurnPin) - 1;
    rightWheelVelocity = (rightCnt * wheelCircumference * frequency) / (float)(pulsesPerRevolution * reductionRatio);
    rightWheelVelocity *= 1 - 2 * digitalRead(rightTurnPin);
    float wheelVelocity = (leftWheelVelocity + rightWheelVelocity) / 2.0f;
    float angularVelocity = (leftWheelVelocity - rightWheelVelocity) / wheelSpacing;
    position.x += wheelVelocity * cos(position.yaw) * (1.0f / frequency);
    position.y += wheelVelocity * sin(position.yaw) * (1.0f / frequency);
    position.yaw += angularVelocity * RAD_TO_DEG * (1.0f / frequency);
}

// 编码器中断服务程序（ISR）
void myOdometer::leftEncoderISR() {
    unsigned long currentTime = micros();  // 获取当前时间
    leftPulsePeriod = currentTime - leftLastPulseTime; // 计算脉冲间隔
    leftWheelVelocity = (wheelCircumference * 1000000) / (float)(pulsesPerRevolution * leftPulsePeriod * reductionRatio);
    if(!digitalRead(leftTurnPin)) leftWheelVelocity *= -1;
    leftLastPulseTime = currentTime;  // 更新上次脉冲时间
}

// 编码器中断服务程序（ISR）
void myOdometer::rightEncoderISR() {
    unsigned long currentTime = micros();  // 获取当前时间
    rightPulsePeriod = currentTime - rightLastPulseTime; // 计算脉冲间隔
    rightWheelVelocity = (wheelCircumference * 1000000) / (float)(pulsesPerRevolution * rightPulsePeriod * reductionRatio);
    if(!digitalRead(rightTurnPin)) rightWheelVelocity *= -1;
    rightLastPulseTime = currentTime;  // 更新上次脉冲时间
}


float myOdometer::getPositionX() const {
    return position.x;
}

float myOdometer::getPositionY() const {
    return position.y;
}

float myOdometer::getPositionYaw() const {
    return position.yaw;
}

myOdometer::~myOdometer() {}

uint16_t myOdometer::leftTurnPin = 0;
uint16_t myOdometer::rightTurnPin = 0;

unsigned volatile long myOdometer::leftLastPulseTime = 0;           // 左轮上一次脉冲时间
unsigned long myOdometer::leftPulsePeriod = 0;             // 左轮脉冲周期

unsigned volatile long myOdometer::rightLastPulseTime = 0;          // 右轮上一次脉冲时间
unsigned long myOdometer::rightPulsePeriod = 0;            // 右轮脉冲周期

volatile float myOdometer::leftWheelVelocity = 0.0f;                      // 左轮定义转速变量（米每秒）
volatile float myOdometer::rightWheelVelocity = 0.0f;                     // 右轮定义转速变量（米每秒）

uint16_t myOdometer::pulsesPerRevolution = 12;           // 每圈脉冲数（编码器的分辨率）

float myOdometer::wheelCircumference = 0.5f;                  // 轮子周长（根据具体轮子尺寸设置）

int myOdometer::reductionRatio = 40;                        // 电机减速比