#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "driver/i2c.h"
#include "sht31.h"

TaskHandle_t TASK_READ_SHT31 = NULL;
TaskHandle_t TASK_READ_TEMP = NULL;
TaskHandle_t TASK_READ_HUMI = NULL;

static const char* TAG = "freeRTOS";
float t, h;

void Get_SHT31(void * params)
{
    while(1)
    {
        if(sht31_readTempHum())
        {
            t = sht31_GetTemperature();
            h = sht31_GetHumidity();
        }
        else
        {
            ESP_LOGE(TAG, "Read SHT31 failed");
        }
       
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void Get_Temp(void * params)
{
    while(1)
    {
        printf("Temp: %2.2f \n", t);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void Get_Humi(void * params)
{
    while(1)
    {
        printf("Humi: %2.2f \n", h);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    sht31_init();
    xTaskCreate(&Get_SHT31, "Get_SHT31", 2048, NULL, 1, &TASK_READ_SHT31);
    xTaskCreate(&Get_Temp, "Get_Temp", 2048, NULL, 1, &TASK_READ_TEMP);
    xTaskCreate(&Get_Humi, "Get_Humi", 2048, NULL, 1, &TASK_READ_HUMI);
}
