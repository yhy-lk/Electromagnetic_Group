#include <Arduino.h>
#include <myDiffControl.h>
#include <myOdometry.h>

class myControlSystem : public DiffControl, public myOdometer {
public:
    myControlSystem();
    myControlSystem(uint16_t leftEncoderPin, uint16_t rightEncoderPin, uint16_t pulsesPerRevolution, 
                    float wheelRadius, int reductionRatio, float wheelSpacing,
                    uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM, uint16_t leftBPAKE, uint16_t rightBPAKE,
                    float Kp, float Ki, float Kd, float frequency);

    void Init();

    void setTargetVel(float leftVel, float rightVel);

    void setPID(float kp, float ki, float kd);

    ~myControlSystem();

    // 接口函数
    float getLeftOutput() const;
    float getRightOutput() const;

    float getLeftTargetVel() const;
    float getRightTargetVel() const;
    float getRightKp() const;
    float getRightKi() const;
    float getRightKd() const;

    float getPIDKp() const;
    float getPIDKi() const;
    float getPIDKd() const;

    uint16_t getLeftPWM() const;
    uint16_t getRightPWM() const;

private:

    // 接口函数
    float getLeftKp() const;
    float getLeftKi() const;
    float getLeftKd() const;

    inline int sign(float x) const;
};

