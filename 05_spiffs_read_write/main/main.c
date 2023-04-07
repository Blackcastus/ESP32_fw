#include <stdio.h>
#include <stdint.h>
#include "string.h"
#include "cJSON.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
// #include "config.h"
#include "ntt.h"

#define TAG "spiffs"

typedef struct
{
    // char name;
    uint8_t ssid;
    // int age;
    // float height;
} Person_t;

Person_t p;

void app_main(void)
{
    Ntt_Init();
    Ntt_save();
    // CFG_Init();
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // CFG_Save();
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // CFG_Load();
    // size_t size = sizeof(p);
    // printf("Size of struct Person: %zu bytes\n", size);

}