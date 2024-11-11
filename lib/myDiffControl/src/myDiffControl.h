#include <Arduino.h>

class DiffControl {
public:
    DiffControl() {}
    // 差速控制类构造函数，传入参数分别是
    DiffControl(uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM, uint16_t leftBPAKE, uint16_t rightBPAKE);
    
    // 设置电机控制引脚为输出模式
    bool Init();

    // 前进函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
    void goForward(uint32_t PWM, uint32_t time);

    // 前进函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
    void goForward(uint32_t PWM);

    // 倒车函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
    void goBack(uint32_t PWM, uint32_t time);

    // 倒车函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
    void goBack(uint32_t PWM);

    // 左转函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
    void turnLeft(uint32_t PWM, uint32_t time);

    // 左转函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
    void turnLeft(uint32_t PWM);

    //右转函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
    void turnRight(uint32_t PWM, uint32_t time);

    //右转函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
    void turnRight(uint32_t PWM);

    // 停车函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
    void stop(uint32_t time);

    // 停车函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
    void stop();

    // 刹车函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
    void brake(uint32_t time);

    // 刹车函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
    void brake();

    // 用户自定义函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
    void userDefined(uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM, uint32_t time);

    // 用户自定义函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
    void userDefined(uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM);

    // 析构函数
    ~DiffControl() {}
private:
    uint16_t PWM, time;        //PWM控制占空比，time是持续时间
    uint16_t LC, LP, RC, RP;   //分别是左轮转向，左轮速度，右轮转向，右轮速度
    uint16_t LBPAKE, RBPAKE;   //分别是左轮的急停刹车和右轮的急停刹车
};

// 转向标志
enum turn {
    LEFT,
    RIGHT,
    beEnd,
    NO
};
