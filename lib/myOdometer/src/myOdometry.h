#include <Arduino.h>
#include "driver/pcnt.h"

class myOdometer {
public:
    myOdometer();
    myOdometer(uint16_t leftEncoderPin, uint16_t rightEncoderPin, uint16_t pulsesPerRevolution, float wheelRadius, int reductionRatio, float wheelSpacing, float frequency);
    myOdometer(uint16_t leftEncoderPin, uint16_t rightEncoderPin, uint16_t leftTurnPin, uint16_t rightTurnPin, uint16_t pulsesPerRevolution, float wheelRadius, int reductionRatio, float wheelSpacing, float frequency);
    
    bool Init();

    void update();

    float getLeftWheelVelocity() const;
    float getRightWheelVelocity() const;

    float getPositionX() const;

    float getPositionY() const;

    float getPositionYaw() const;

    ~myOdometer();

    static unsigned long leftPulsePeriod;             // 左轮脉冲周期
    static unsigned long rightPulsePeriod;            // 右轮脉冲周期
    uint32_t currentPulsePeriod = INT32_MAX;
    
private:
    void setupPcntUnit(pcnt_unit_t unit, int pulse_gpio);
    static void leftEncoderISR();
    static void rightEncoderISR();
    void resetVelocity();
    void resetPulsePeriod();

    uint16_t leftEncoderPin;                    // 左轮编码器的信号引脚，使用外部中断
    uint16_t rightEncoderPin;                    // 右轮编码器的信号引脚，使用外部中断

    static uint16_t leftTurnPin;
    static uint16_t rightTurnPin;

    static volatile unsigned long leftLastPulseTime;           // 左轮上一次脉冲时间

    static volatile unsigned long rightLastPulseTime;          // 右轮上一次脉冲时间

    static volatile float leftWheelVelocity;                      // 左轮定义转速变量（米每秒）
    static volatile float rightWheelVelocity;                     // 右轮定义转速变量（米每秒）

    static uint16_t pulsesPerRevolution;           // 每圈脉冲数（编码器的分辨率）

    static float wheelCircumference;                  // 轮子周长（根据具体轮子尺寸设置）

    static int reductionRatio;                        // 电机减速比

    float wheelSpacing;                          // 两轮间距

    float frequency;

    typedef struct Position {
        float x, y;
        float yaw;
    } Position;

    Position position;
};