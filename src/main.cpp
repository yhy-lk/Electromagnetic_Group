
#include <main.h>


void setup() {
    Serial.begin(115200);


    xSemaphoreVel = xSemaphoreCreateMutex();
    if ( ( xSemaphoreVel ) != NULL )
        xSemaphoreGive( ( xSemaphoreVel ) );

    xTaskCreatePinnedToCore(TaskControl, "TaskControl", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(blinkerLoopTask, "blinkerLoopTask", 8192, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
    // xTaskCreatePinnedToCore(TaskMPU6050, "TaskMPU6050", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    // xTaskCreatePinnedToCore(TaskUltrasound, "TaskUltrasound", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    // xTaskCreatePinnedToCore(TaskTest01, "TaskTest01", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
}

void loop() {

//--------------------------------------------------------------------------------------------
// control
    Serial.print("TargetVel = ");
    Serial.print(Vel);
    Serial.println(" m/s");
    Serial.print("leftVel = ");
    Serial.print(control.myOdometer::getLeftWheelVelocity());
    Serial.print(" m/s");
    Serial.print(", rightVel = ");
    Serial.print(control.myOdometer::getRightWheelVelocity());
    Serial.println(" m/s");
    Serial.print("leftTargetVel = ");
    Serial.print(control.getLeftTargetVel());
    Serial.print(" m/s");
    Serial.print(", rightTargetVel = ");
    Serial.print(control.getRightTargetVel());
    Serial.println(" m/s");
    Serial.print("leftOutPut = ");
    Serial.print(control.getLeftOutput());
    Serial.print(", rightOutPut = ");
    Serial.println(control.getRightOutput());
    Serial.print("leftPWM = ");
    Serial.print(control.getLeftPWM());
    Serial.print(", rightPWM = ");
    Serial.println(control.getRightPWM());
    Serial.print("Kp = ");
    Serial.print(control.getPIDKp());
    Serial.print(", Ki = ");
    Serial.print(control.getPIDKi());
    Serial.print(", Kd = ");
    Serial.println(control.getPIDKd());

//--------------------------------------------------------------------------------------------
// MPU6050
    // Serial.print("Yaw = ");
    // Serial.print(IMU.getYaw());
    // Serial.print(" degree, ");
    // Serial.print("Roll = ");
    // Serial.print(IMU.getRoll());
    // Serial.print(" degree, ");
    // Serial.print("Pitch = ");
    // Serial.print(IMU.getPitch());
    // Serial.println(" degree");


//--------------------------------------------------------------------------------------------
// Ultrasound
    // Serial.print("distance: ");
    // Serial.print(ultrasound.getDistance());
    // Serial.println(" mm");

    // const byte* buf = ultrasound.getBuf();
    // for(int i = 0; i < 4; ++i) {
    //     Serial.print(*(buf++));
    //     Serial.print(" ");
    // }
    // Serial.println();

    
//--------------------------------------------------------------------------------------------
// 500ms打印一次
    vTaskDelay(pdMS_TO_TICKS(500));
}

