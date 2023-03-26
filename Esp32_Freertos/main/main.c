/*
@Author: duchien
@Date: 2023-03-25
@Last Modified by: duchien
@Last Modified time: 2023-03-25
*/

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "linear_regression.h"
#include "driver/gpio.h"

#define ESP_INR_FLAG_DEFAULT 0
#define LED_PIN 2
#define PUSH_BUTTON_PIN 0

TaskHandle_t ISR = NULL;
TaskFunction_t myTaskHandle_1 = NULL;
TaskFunction_t myTaskHandle_2 = NULL;



void IRAM_ATTR button_isr_handler(void * arg)
{
    xTaskResumeFromISR(ISR);
}

void interrupt_task(void *arg)
{
    bool led_status = false;
    while(1)
    {
        vTaskSuspend(NULL);
        led_status = !led_status;
        gpio_set_level(LED_PIN, led_status);
        printf("Button pressed!\n");
    }
}

// void Init_GPIO(void)
// {
//     gpio_pad_select_gpio(PUSH_BUTTON_PIN);
//     gpio_pad_select_gpio(LED_PIN);
//     gpio_set_direction(PUSH_BUTTON_PIN, GPIO_MODE_INPUT);
//     gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

//     gpio_set_intr_type(PUSH_BUTTON_PIN, GPIO_INTR_POSEDGE);
//     gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);
//     gpio_isr_handler_add(PUSH_BUTTON_PIN, button_isr_handler, NULL);
// }

void Demo_Task_1(void *arg)
{
    int count = 0;
    while (1)
    {
        count++;
        printf("Demo task 1: %d @@\n", count);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if(count == 5)
        {
            // vTaskDelete(myTaskHandle_2);    // Delete task 2
            vTaskSuspend(myTaskHandle_2);       // Pause task 2
            printf("Demo 2 is Suspend!!\n");
        }
        else if( count == 8)
        {
            vTaskResume(myTaskHandle_2);    // Continue task 2
            printf("Demo 2 is Resume!!\n");
        }
        else if( count == 13)
        {
            vTaskDelete(myTaskHandle_2);
            printf("Demo 2 is Delete!!\n");

        }
    }   
}

void Demo_Task_2(void *arg)
{
    // for(int i = 0; i < 15; i++)
    // {
    //     printf("task 2 count: %d\n", i);
    //     vTaskDelay(500 / portTICK_PERIOD_MS);
    // }
    int cnt = 0;
    while (1)
    {
        printf("count: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }   
}

void app_main(void)
{
    gpio_pad_select_gpio(PUSH_BUTTON_PIN);
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(PUSH_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_set_intr_type(PUSH_BUTTON_PIN, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);
    gpio_isr_handler_add(PUSH_BUTTON_PIN, button_isr_handler, NULL);

    // Init_GPIO();
    xTaskCreate(interrupt_task, "interrupt_task", 4096, NULL, 10, &ISR);
    // xTaskCreate(Demo_Task_1, "demo task 1", 4096, NULL, 10, &myTaskHandle_1);
    xTaskCreate(Demo_Task_2, "demo task 2", 4096, NULL, 10, &myTaskHandle_2);
    // xTaskCreatePinnedToCore(Demo_Task_2, "demo task 2", 4096, NULL, 10, &myTaskHandle_2, 1);
}
