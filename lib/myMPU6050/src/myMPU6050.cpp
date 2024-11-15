#include "myMPU6050.h"

TwoWire WireMPU6050 = TwoWire(0);

myMPU6050::myMPU6050() {}

myMPU6050::myMPU6050(int sda, int scl, float frequency) {
    this->sda = sda;
    this->scl = scl;
    this->frequency = frequency;
}

// IMU初始化,传入参数为IIC_6050的地址
bool myMPU6050::Init() {
    WireMPU6050.setPins(sda, scl);
    filter.begin(frequency);
    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &WireMPU6050, 0)) {
        return false;
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
    return true;

}

// 设置MPU6050数据更新频率
void myMPU6050::begin(float frequency) {
    this->frequency = frequency;
    filter.begin(frequency);
}

// 得到静止状态下的偏移量
void myMPU6050::getOffset() {
    for(int i = 0; i < 100; i++) {
        mpu.getEvent(&a, &g, &temp);
        MPU6050ERROR[0] += a.acceleration.x;
        MPU6050ERROR[1] += a.acceleration.y;
        MPU6050ERROR[2] += a.acceleration.z - 9.8;
        MPU6050ERROR[3] += g.gyro.x;
        MPU6050ERROR[4] += g.gyro.y;
        MPU6050ERROR[5] += g.gyro.z;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for(int i = 0; i < 6; ++i) {
        MPU6050ERROR[i] /= 100.0;
    }
}

// 得到滤波之后的准确数值
void myMPU6050::dataGetAndFilter() {
    mpu.getEvent(&a, &g, &temp);
    ax = a.acceleration.x - MPU6050ERROR[0];
    ay = a.acceleration.y - MPU6050ERROR[1];
    az = a.acceleration.z - MPU6050ERROR[2];
    gx = (g.gyro.x - MPU6050ERROR[3]) * RAD_TO_DEG;
    gy = (g.gyro.y - MPU6050ERROR[4]) * RAD_TO_DEG;
    gz = (g.gyro.z - MPU6050ERROR[5]) * RAD_TO_DEG;
    temperature = temp.temperature;
    if(gx < 0.0015f && gx > -0.0015f) gx = 0.0f;
    if(gy < 0.0015f && gy > -0.0015f) gy = 0.0f;
    if(gz < 0.0015f && gz > -0.0015f) gz = 0.0f;
} 

myMPU6050::~myMPU6050() {}

// 计算欧拉角
void myMPU6050::IMUupdate() {
    dataGetAndFilter();
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    Yaw = filter.getYaw();
    Roll = filter.getRoll();
    Pitch = filter.getPitch();
}

// 拿到yaw角的值
double myMPU6050::getYaw() const {
    return Yaw;
}

// 拿到pitch角的值
double myMPU6050::getPitch() const { 
    return Pitch;
}

// 拿到Roll角的值 
double myMPU6050::getRoll() const {
    return Roll;
}


// 直接对z轴角速度积分得到的Yaw，仅供测试使用
void myMPU6050::updataMyYaw() {
    MyYaw += gz / frequency;
}

// 得到z轴角速度积分得到的Yaw，仅供测试使用
double myMPU6050::GetMyYaw() {
    return MyYaw;
}

// 加速度二重积分得位移，仅供测试使用
void myMPU6050::calculateDisplacement() {
    vx += ax / frequency;
    vy += ay / frequency;
    vz += (az - 9.8) / frequency;

    sx += vx / frequency;
    sy += vy / frequency;
    sz += vz / frequency;
}
