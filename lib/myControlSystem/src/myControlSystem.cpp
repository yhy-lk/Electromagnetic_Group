#include <myControlSystem.h>
#include <PID_v1.h>


//--------------------------------------------------------------------------------------------
// 左轮变量声明
float leftCurrVel;
float leftOutput;
float leftTargetVel;
float leftKp, leftKi, leftKd;
uint16_t leftPWM;

//--------------------------------------------------------------------------------------------
// 右轮变量声明
float rightCurrVel;
float rightOutput;
float rightTargetVel;
float rightKp, rightKi, rightKd;
uint16_t rightPWM;


//--------------------------------------------------------------------------------------------
// 实例化两边车轮速度控制PID类
PID leftPID = PID(&leftCurrVel, &leftOutput, &leftTargetVel, leftKp, leftKi, leftKd, DIRECT);
PID rightPID = PID(&rightCurrVel, &rightOutput, &rightTargetVel, rightKp, rightKi, rightKd, DIRECT);


//--------------------------------------------------------------------------------------------
// 无参构造函数，这里不使用，仅我调试代码BUG时使用
myControlSystem::myControlSystem() {}


//--------------------------------------------------------------------------------------------
/**
 * @brief 有参构造函数，实际使用构造函数
 *        @note
 *        这个构造函数会同时构造父类DiffControl和myOdometer，同时初始化PID的两个实例对象leftPID和rightPID
 *
 * @param leftEncoderPin 左轮编码器的GPIO
 * @param rightEncoderPin 右轮编码器的GPIO
 * @param pulsesPerRevolution 编码器分辨率，即旋转一周产生多少个脉冲
 * @param wheelRadius 车轮半径，通过电机角速度计算电机线速度需要用到
 * @param reductionRatio 电机的减速比，它的目的是增大电机扭矩
 * @param wheelSpacing 两轮间距，用于计算机器人的旋转角速度，旋转角速度则用于计算偏航角，偏航角也可由IMU计算得到
 * @param leftCW 左边电机的转向GPIO
 * @param leftPWM 左边电机的控速GPIO
 * @param rightCW 右边电机的转向GPIO
 * @param rightPWM 右边电机的控速GPIO
 * @param leftBPAKE 左边电机的刹车GPIO
 * @param rightBPAKE 右边电机的刹车GPIO
 * @param Kp PID的比例项，这里左轮和右轮默认使用同一个Kp
 * @param Ki PID的积分项，这里左轮和右轮默认使用同一个Ki
 * @param Kd PID的微分项，这里左轮和右轮默认使用同一个Kd
 * @param frequency 统一使用的频率，测周法获取电机速度的频率，对速度积分里程计定位的频率，PID的计算频率等
 *
 * @return
 *     - 无
 */
myControlSystem::myControlSystem(uint16_t leftEncoderPin, uint16_t rightEncoderPin, uint16_t pulsesPerRevolution, 
                    float wheelRadius, int reductionRatio, float wheelSpacing,
                    uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM, uint16_t leftBPAKE, uint16_t rightBPAKE,
                    float Kp, float Ki, float Kd, float frequency)
                    : DiffControl(leftCW, leftPWM, rightCW, rightPWM, leftBPAKE, rightBPAKE),
                      myOdometer(leftEncoderPin, rightEncoderPin, leftCW, rightCW, pulsesPerRevolution, wheelRadius, reductionRatio, wheelSpacing, frequency) {

                        leftCurrVel = myOdometer::getLeftWheelVelocity(); leftOutput = 0.0f; leftTargetVel = 0.0f;
                        leftKp = Kp; leftKi = Ki; leftKd = Kd;
                        leftPID.SetMode(AUTOMATIC);
                        leftPID.SetOutputLimits(-255.0f, 255.0f);
                        leftPID.SetSampleTime(frequency);
                        leftPID.SetTunings(leftKp, leftKi, leftKd);

                        rightCurrVel = myOdometer::getRightWheelVelocity(); rightOutput = 0.0f; rightTargetVel = 0.0f;
                        rightKp = Kp; rightKi = Ki; rightKd = Kd;
                        rightPID.SetMode(AUTOMATIC);
                        rightPID.SetOutputLimits(-255.0f, 255.0f);
                        rightPID.SetSampleTime(frequency);
                        rightPID.SetTunings(rightKp, rightKi, rightKd);

                    }

//--------------------------------------------------------------------------------------------
// 初始化差速控制类和里程计定位类
void myControlSystem::Init() {
    DiffControl::Init();
    myOdometer::Init();
}

//--------------------------------------------------------------------------------------------
/**
 * @brief 控制电机达到目标速度的函数
 *        @note
 *          - 这个函数会自动获取电机当前转速，根据与目标速度的差值，让PID控制器计算输出，然后发布速度命令，控制电机达到目标速度
 *          - 同时它也会自动计算定位，无需重复调用myOdometer::update()
 * 
 * @param leftVel 左边电机的目标速度
 * @param rightVel 右边电机的目标速度
 *
 * @return
 *     - 无
 */
void myControlSystem::setTargetVel(float leftVel, float rightVel) {
    myOdometer::update();
    leftTargetVel = leftVel;
    leftCurrVel = myOdometer::getLeftWheelVelocity();

    rightTargetVel = rightVel;
    rightCurrVel = myOdometer::getRightWheelVelocity();

    leftPID.Compute();
    rightPID.Compute();

    leftPWM = min(255, max(0, leftPWM + (int16_t)(leftOutput) * sign(leftTargetVel)));
    rightPWM = min(255, max(0, rightPWM + (int16_t)(rightOutput) * sign(rightTargetVel)));

    DiffControl::userDefined(leftTargetVel > 0, leftPWM, rightTargetVel < 0, rightPWM);
}

//--------------------------------------------------------------------------------------------
/**
 * @brief 动态调整PID参数，可以在运行时实时修改
 * 
 * @param Kp PID的比例项，这里左轮和右轮默认使用同一个Kp
 * @param Ki PID的积分项，这里左轮和右轮默认使用同一个Ki
 * @param Kd PID的微分项，这里左轮和右轮默认使用同一个Kd
 *
 * @return
 *     - 无
 */

void myControlSystem::setPID(float Kp, float Ki, float Kd) {
    leftKp = Kp; leftKi = Ki; leftKd = Kd;
    rightKp = Kp; rightKi = Ki; rightKd = Kd;
    leftPID.SetTunings(leftKp, leftKi, leftKd);
    rightPID.SetTunings(rightKp, rightKi, rightKd);
}

//--------------------------------------------------------------------------------------------
// 析构函数
myControlSystem::~myControlSystem() {}


//--------------------------------------------------------------------------------------------
// 访问私有成员的接口函数
float myControlSystem::getLeftTargetVel() const {
    return leftTargetVel;
}

float myControlSystem::getRightTargetVel() const {
    return rightTargetVel;
}

float myControlSystem::getLeftKp() const {
    return leftKp;
}
float myControlSystem::getLeftKi() const {
    return leftKi;
}
float myControlSystem::getLeftKd() const {
    return leftKd;
}

float myControlSystem::getRightKp() const {
    return rightKp;
}
float myControlSystem::getRightKi() const {
    return rightKi;
}
float myControlSystem::getRightKd() const {
    return rightKd;
}

float myControlSystem::getLeftOutput() const {
    return leftOutput;
}

float myControlSystem::getRightOutput() const {
    return rightOutput;
}


float myControlSystem::getPIDKp() const {
    return leftPID.GetKp();
}
float myControlSystem::getPIDKi() const {
    return leftPID.GetKi();
}
float myControlSystem::getPIDKd() const {
    return leftPID.GetKd();
}

uint16_t myControlSystem::getLeftPWM() const {
    return leftPWM;
}

uint16_t myControlSystem::getRightPWM() const {
    return rightPWM;
}


//--------------------------------------------------------------------------------------------
/**
 * @brief 用于计算浮点数类型变量的符号
 * 
 * @param x 计算对象
 *
 * @return
 *     - 1 代表正数
 *     - 0 代表计算对象为0
 *     - -1 代表负数
 */
inline int myControlSystem::sign(float x) const {
    return (x > 0) - (x < 0);
}

