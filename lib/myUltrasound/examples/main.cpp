

#include <myUltrasound.h>

myUltrasound ultrasound =  myUltrasound(115200, 11, 12);

void TaskUltrasound(void *pvParameters) {
    ( void ) pvParameters;

    // 超声波模块初始化
    ultrasound.Init();
    vTaskDelay(pdMS_TO_TICKS(10));
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        // 自动计算距离
        ultrasound.autoComputeDistance();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}


void setup() {
    Serial.begin(115200);

    // 在ESP32的1核上运行此任务
    xTaskCreatePinnedToCore(TaskUltrasound, "TaskUltrasound", 8192, NULL, 2, NULL, ARDUINO_RUNNING_CORE);
}

void loop() {
//--------------------------------------------------------------------------------------------
    // Ultrasound
    Serial.print("distance: ");
    Serial.print(ultrasound.getDistance());
    Serial.println(" mm");

    const byte* buf = ultrasound.getBuf();
    for(int i = 0; i < 4; ++i) {
        Serial.print(*(buf++));
        Serial.print(" ");
    }
    Serial.println();

//--------------------------------------------------------------------------------------------
    // 500ms打印一次
    vTaskDelay(pdMS_TO_TICKS(500));
}

