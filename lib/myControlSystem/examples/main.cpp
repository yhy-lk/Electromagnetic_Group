#include <myControlSystem.h>

myControlSystem control = myControlSystem(14, 38, 12, 0.2, 40, 0.3, 35, 36, 39, 40, 13, 41, 3, 0.3, 0.2, 100);

volatile float Vel = 0.0f;

void TaskControl(void *pvParameters) {
    ( void ) pvParameters;
    control.Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        for(int i = -300; i <= 300; i += 50) {
            Vel = (float)i / 100.0f;
            for(int j = 0; j < 50; ++j) {
                control.setTargetVel(Vel, Vel);
                vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    xTaskCreatePinnedToCore(TaskControl, "TaskControl", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
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
    // 500ms打印一次
    vTaskDelay(pdMS_TO_TICKS(500));
}

