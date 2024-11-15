#include <myDiffControl.h>

DiffControl::DiffControl(uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM, uint16_t leftBPAKE, uint16_t rightBPAKE): LC(leftCW), LP(leftPWM), RC(rightCW), RP(rightPWM), LBPAKE(leftBPAKE), RBPAKE(rightBPAKE) {}

// 设置电机控制引脚为输出模式
bool DiffControl::Init() {
    pinMode(LC, OUTPUT);
    pinMode(RC, OUTPUT);
    pinMode(LBPAKE, OUTPUT);
    pinMode(RBPAKE, OUTPUT);
    ledcSetup(0, 1000, 8);
    ledcSetup(1, 1000, 8);
    ledcAttachPin(LP, 0);
    ledcAttachPin(RP, 1);
    ledcWrite(0, 255);
    ledcWrite(1, 255);
    return true;
}

// 前进函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
void DiffControl::goForward(uint32_t PWM, uint32_t time) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, HIGH);
    digitalWrite(RC, LOW);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
    delay(time);
}

// 前进函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
void DiffControl::goForward(uint32_t PWM) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, HIGH);
    digitalWrite(RC, LOW);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
}
   

// 倒车函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
void DiffControl::goBack(uint32_t PWM, uint32_t time) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, LOW);
    digitalWrite(RC, HIGH);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
    delay(time);
}

// 倒车函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
void DiffControl::goBack(uint32_t PWM) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, LOW);
    digitalWrite(RC, HIGH);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
}

// 左转函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
void DiffControl::turnLeft(uint32_t PWM, uint32_t time) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, LOW);
    digitalWrite(RC, LOW);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
    delay(time);
}

// 左转函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
void DiffControl::turnLeft(uint32_t PWM) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, LOW);
    digitalWrite(RC, LOW);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
}

//右转函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
void DiffControl::turnRight(uint32_t PWM, uint32_t time) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, HIGH);
    digitalWrite(RC, HIGH);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
    delay(time);
}

//右转函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
void DiffControl::turnRight(uint32_t PWM) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, HIGH);
    digitalWrite(RC, HIGH);
    ledcWrite(0, 255 - PWM);
    ledcWrite(1, 255 - PWM);
}

// 停车函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
void DiffControl::stop(uint32_t time) {
    ledcWrite(0, 255);
    ledcWrite(1, 255);
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    delay(time);
}

// 停车函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
void DiffControl::stop() {
    ledcWrite(0, 255);
    ledcWrite(1, 255);
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
}

// 刹车函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
void DiffControl::brake(uint32_t time) {
    ledcWrite(0, 255);
    ledcWrite(1, 255);
    digitalWrite(LBPAKE, LOW);
    digitalWrite(RBPAKE, LOW);
    delay(time);
}

// 刹车函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
void DiffControl::brake() {
    ledcWrite(0, 255);
    ledcWrite(1, 255);
    digitalWrite(LBPAKE, LOW);
    digitalWrite(RBPAKE, LOW);
}


// 用户自定义函数,带有delay，如果使用了freeRTOS等实时操作系统，请慎用此函数
void DiffControl::userDefined(uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM, uint32_t time) {

    delay(time);
}

// 用户自定义函数,不带delay，如果使用了freeRTOS等实时操作系统，请使用此函数
void DiffControl::userDefined(uint16_t leftCW, uint16_t leftPWM, uint16_t rightCW, uint16_t rightPWM) {
    digitalWrite(LBPAKE, HIGH);
    digitalWrite(RBPAKE, HIGH);
    digitalWrite(LC, leftCW);
    digitalWrite(RC, rightCW);
    ledcWrite(0, 255 - leftPWM);
    ledcWrite(1, 255 - rightPWM);
}
