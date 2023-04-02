#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

TaskHandle_t myTASK_FIRST = NULL
TaskHandle_t myTASK_SECOND = NULL
TaskHandle_t myTASK_THIRD = NULL

static const char* TAG = "freeRTOS";
EventGroupHandle_t event_group;

const int got_temp = BIT0;
const int got_humi = BIT1;

/*
* Task 1 get temperature with cycle 1s/lan
*/
void Get_Temp(void *params)
{
    while (true)    
    {
        xEventGroupSetBits(event_group, got_temp);
        // ESP_LOGE(TAG, "temperature value received");
        printf("task 1 get temp\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }    
}

/*
* Task 2 get humidity with cycle 2s/lan
*/
void Get_Humi(void *params)
{
    while (true)
    {
        xEventGroupSetBits(event_group, got_humi);
        // ESP_LOGE(TAG, "Humidity value received");
        printf("task 2 get humi\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }   
}

/*
* Task 3 get "print = hello" with cycle 500ms/lan
* vTaskDelay();la mo ham duoc cung cap boi he dieu hanh, 
    duoc su dung de dat mot task vao trang thai "BLOCKED"
*
*/
void Get_Start(void *params)
{
    while (true)
    {
        // ESP_LOGE(TAG, "Humidity value received");
        printf("task 3 hello world!!!\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }   
}

void Sender(void *params)
{
    while (true)
    {
        xEventGroupWaitBits(event_group, got_temp | got_humi, true, true, portMAX_DELAY);
        printf("Sender: Recived Temperature and Humidity\n");
    }
    
}

void app_main(void)
{
    ESP_LOGI(TAG, "----------FreeRTOS Ready----------");
    event_group = xEventGroupCreate();
    xTaskCreate(&Get_Temp, "Get_Temp", 2048, NULL, 1, myTASK_FIRST);
    xTaskCreate(&Get_Humi, "Get_Humi", 2048, NULL, 1, myTASK_SECOND);
    xTaskCreate(&Get_Start, "Get_start", 2048, NULL, 1, myTASK_THIRD);
    xTaskCreate(&Sender, "Sender", 2048, NULL, 1, NULL);
}
