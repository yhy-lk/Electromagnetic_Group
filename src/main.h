#define BLINKER_BLE

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Blinker.h>
#include <myControlSystem.h>
#include <myMPU6050.h>
#include <myUltrasound.h>

myUltrasound ultrasound =  myUltrasound(115200, 11, 12);
myMPU6050 IMU = myMPU6050(42, 45, 100.0f);
myControlSystem control = myControlSystem(14, 38, 12, 0.2, 40, 0.3, 35, 36, 39, 40, 13, 41, 3, 0.3, 0.2, 100);
// myOdometer odometer = myOdometer(0, 38, 35, 39, 12, 0.5, 40, 0.5);

#define velSlider "velSlider1"
BlinkerSlider Slider1(velSlider);

volatile float Vel = 0.0f;
volatile float testi = 0.0f;

SemaphoreHandle_t xSemaphoreVel;

void slider1_callback(int32_t value)
{
    if (xSemaphoreTake(xSemaphoreVel, (TickType_t) 1) == pdTRUE) {
        Vel = (float)value / 100.0f;
        xSemaphoreGive( xSemaphoreVel );
    }
    BLINKER_LOG("get slider value: ", value);
}

void blinkerLoopTask(void *pvParameters)
{
    ( void ) pvParameters;
    vTaskDelay(pdMS_TO_TICKS(1000));
    BLINKER_DEBUG.stream(Serial);
    Slider1.attach(slider1_callback);
    Blinker.begin();
    for(;;) {
        Blinker.run();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void TaskControl(void *pvParameters) {
    ( void ) pvParameters;
    control.Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        if (xSemaphoreTake(xSemaphoreVel, (TickType_t) 1) == pdTRUE) {
            control.setTargetVel(Vel, Vel);
            xSemaphoreGive( xSemaphoreVel );
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }

}

void TaskTest01(void *pvParameters) {
    ( void ) pvParameters;
    for(;;) {
        control.DiffControl::goForward(150);
        vTaskDelay(pdMS_TO_TICKS(5000));
        control.DiffControl::turnLeft(150);
        vTaskDelay(pdMS_TO_TICKS(5000));
        control.DiffControl::turnRight(150);
        vTaskDelay(pdMS_TO_TICKS(5000));
        control.DiffControl::goBack(150);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void TaskMPU6050(void *pvParameters) {
    ( void ) pvParameters;
    vTaskDelay(pdMS_TO_TICKS(1000));
    IMU.Init();
    IMU.getOffset();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        IMU.IMUupdate();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void TaskUltrasound(void *pvParameters) {
    ( void ) pvParameters;
    ultrasound.Init();
    vTaskDelay(pdMS_TO_TICKS(10));
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        ultrasound.autoComputeDistance();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}


