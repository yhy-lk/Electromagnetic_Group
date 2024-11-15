#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

class myMPU6050 {
public:
    // 默认构造函数
    myMPU6050();
    myMPU6050(int sda, int scl, float frequency);

    bool Init();

    void begin(float frequency);

    void getOffset();

    // 得到滤波之后的准确数值
    void dataGetAndFilter();

    // 计算欧拉角
    void IMUupdate();

    // 拿到yaw角的值
    double getYaw() const;

    // 拿到pitch角的值
    double getPitch() const;

    // 拿到Roll角的值 
    double getRoll() const;

    // 析构函数
    ~myMPU6050();
private:

    void updataMyYaw();
    double GetMyYaw();
    void calculateDisplacement();
    int sda, scl;
    sensors_event_t a, g, temp;
    double ax, ay, az, gx, gy, gz, temperature;
    double MyYaw;
    double vx, vy, vz, sx, sy, sz;
    double YawErrorK;
    float frequency = 100.0f;
    double MPU6050ERROR[6] = {0.0};
    double Yaw = 0.0, Pitch = 0.0, Roll = 0.0;                     // 偏航角，俯仰角，翻滚角
    
    Adafruit_MPU6050 mpu;
    Madgwick filter;
};

// MPU6050的一些参数
typedef struct {
    unsigned int MPU6050dt;            // 记录MPU6050的数据更新时间间隔，这里为10ms
    unsigned long preMPU6050Millis;    // 记录上一次MPU6050的数据更新时间
    unsigned int firstGetYawTime;      // 记录开始进行MPU6050的Yaw数据滤波优化时间
    unsigned long previousprintMillis; // 记录上一次打印数据时的时间

} globalMPU6050Params;
