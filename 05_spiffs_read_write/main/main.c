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
#include "config.h"

static const char *TAG = "main";

int app_main(void)
{
    esp_err_t err;
    err = CFG_Spiffs_Init();
    if(err != ESP_OK)
        ESP_LOGE(TAG, "%d", err);
    
}
