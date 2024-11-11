/*
    control.Init()的调用必须在Blinker.begin()之前，具体原因未知，如果不遵守这个规则，单片机会一直尝试复位
*/

#include <main.h>


void setup() {
    Serial.begin(9600);
    control.Init();
    BLINKER_DEBUG.stream(Serial);
    Serial.println("Create1!");
    Blinker.begin();
    Serial.println("Create2!");
    Serial.println("Create3!");

    Slider1.attach(slider1_callback);
    
    xSemaphoreVel = xSemaphoreCreateMutex();
    if ( ( xSemaphoreVel ) != NULL )
         xSemaphoreGive( ( xSemaphoreVel ) );

    xTaskCreatePinnedToCore(TaskControl, "TaskControl", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(blinkerLoopTask, "blinkerLoopTask", 8192, NULL, 3, NULL, ARDUINO_RUNNING_CORE);
    // xTaskCreatePinnedToCore(TaskTest01, "TaskTest01", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
}

void loop() {
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
    // Serial.print("Kp = ");
    // Serial.print(control.getPIDKp());
    // Serial.print(", Ki = ");
    // Serial.print(control.getPIDKi());
    // Serial.print(", Kd = ");
    // Serial.println(control.getPIDKd());
    vTaskDelay(pdMS_TO_TICKS(500));
}

