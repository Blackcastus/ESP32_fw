#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "max31865.h"

#define RTD_STANDARD MAX31865_ITS90   // ITS-90
#define RTD_NOMINAL 100               // 100 Ohm, PT100
#define RTD_REFERENCE 390.0             // Rref = 430 Ohm
#define RTD_CONNECTION MAX31865_2WIRE // 2-wire connection configuration
#define MAX_SPI_MOSI 13
#define MAX_SPI_MISO 12
#define MAX_SPI_CLK 14
#define MAX_SPI_CS 33
#define MAX_SPI_DRYPIN -1               // NO CONNECT
static const char *TAG = "MAX31865";

static max31865_config_t max_config = {
    .v_bias = true,
    .filter = MAX31865_FILTER_50HZ,
    .mode = MAX31865_MODE_SINGLE,
    .connection = RTD_CONNECTION            // config mode 2 wire
};

static float Temp_Val;

void MAX_Init()
{
    max31865_t dev = {
        .standard = RTD_STANDARD,
        .r_ref = RTD_REFERENCE,
        .rtd_nominal = RTD_NOMINAL,
    };

    Max31865_Init(&dev, SPI2_HOST, MAX_SPI_MISO, MAX_SPI_MOSI, MAX_SPI_CLK, MAX_SPI_CS, MAX_SPI_DRYPIN);
    ESP_ERROR_CHECK(Max31865_SetConfig);
}

void app_main(void)
{
    // Initialize NVS
    // esp_err_t err = nvs_flash_init();
    // if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     err = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "READY");

    vTaskDelay(2000/portTICK_PERIOD_MS);

    // MAX_Init();
    max31865_t dev = {
        .standard = RTD_STANDARD,
        .r_ref = RTD_REFERENCE,
        .rtd_nominal = RTD_NOMINAL,
    };

    Max31865_Init(&dev, SPI2_HOST, MAX_SPI_MISO, MAX_SPI_MOSI, MAX_SPI_CLK, MAX_SPI_CS, MAX_SPI_DRYPIN);
    ESP_ERROR_CHECK(Max31865_SetConfig(&dev, &max_config));
    ESP_LOGI(TAG, "Init Max31865 OK");

    while(1)
    {
        // ESP_LOGI(TAG, "READY");
        // vTaskDelay(1000/portTICK_PERIOD_MS);

        esp_err_t res = Max31865_Measure(&dev, &Temp_Val);
        if(res != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to measure: %d (%s)", res, esp_err_to_name(res));
            /* status sensor */
        }
        else
        {
            printf("Value temp: %2.2f\n", Temp_Val);
        }
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}
