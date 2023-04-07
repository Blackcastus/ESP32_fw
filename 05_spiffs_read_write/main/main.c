#include <stdio.h>
#include <stdint.h>
#include "cJSON.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#define TAG "spiffs"
#define SPIFFS_PATH "/spiffs/data.txt"

void get_spiffs(void)
{
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&config);

    FILE *file = fopen(SPIFFS_PATH, "r");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "File does not exist!");
    }
    else
    {
        // Lấy kích thước tệp
        fseek(file, 0, SEEK_END);
        long size = ftell(file);
        fseek(file, 0, SEEK_SET);
        printf("size file: %ld\n", size);

        char buffer[size + 10];
        fread(buffer, 1, size, file);
        fclose(file);
        // Chuyển đổi chuỗi JSON thành đối tượng JSON
        cJSON *json = cJSON_Parse(buffer);
        char *data = cJSON_Print(json);
        // In ra các giá trị của đối tượng JSON
        printf("%s\n", data);
        cJSON *passObject = cJSON_GetObjectItem(json, "pass");
        char *pass = passObject->valuestring;

        printf("pass: %s\n", pass);
        cJSON_Delete(json);

    }
    esp_vfs_spiffs_unregister(NULL);
}

void save_spiffs(void)
{
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&config);

    // ESP_LOGE(TAG, "Creating New file: data.txt");
    FILE *f = fopen(SPIFFS_PATH, "w");
    if (f == NULL)
    {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return;
    }
    // open file oke
    ESP_LOGE(TAG, "Writing data to file: data.txt");
    cJSON *root = cJSON_CreateObject();                                       // cap phat o nho nho cho con tro root va gan o nho do bang 1 chuoi json co dang cap key: value {"key":"value", "key":"value"}
    cJSON_AddItemToObject(root, "ssid", cJSON_CreateString("Black_life"));    // add cap key - value vao chuoi json moi duoc tao
    cJSON_AddItemToObject(root, "pass", cJSON_CreateString("Inlove_s2@123")); // add cap key - value vao chuoi json moi duoc tao

    char *jsonString = cJSON_Print(root); // Chuyển đổi đối tượng JSON thành chuỗi JSON và ghi vào tệp
    fwrite(jsonString, sizeof(jsonString), 1, f);
    fclose(f);
    ESP_LOGI(TAG, "File written");
    cJSON_Delete(root);
    free(jsonString); // giải phóng bộ nhớ jsonString
}

void app_main(void)
{
    esp_err_t err;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // get_spiffs();
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // save_spiffs();
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    get_spiffs();

    // cJSON *root = cJSON_CreateObject();
    // cJSON_AddItemToObject(root, "ssid", cJSON_CreateString("Black_life"));
    // cJSON_AddItemToObject(root, "pass", cJSON_CreateString("Inlove_s2@123"));

    // char *jsonString = cJSON_Print(root);
    // printf("%s\n", jsonString);

    // cJSON *passObject = cJSON_GetObjectItem(root, "pass");
    // char *pass = passObject->valuestring;

    // printf("pass: %s\n", pass);

    // cJSON_Delete(root);
}