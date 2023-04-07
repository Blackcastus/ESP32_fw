/*
 * config.h
 *  save data system
 *  Created on: 2023.07.04.
 *  Modifier on: 2023.07.04.
 *      Author: DucHien
 */

#include "config.h"

#define SPIFFS_PATH "/spiffs/hello.txt"

static const char *TAG = "config data";

void CFG_Init(void)
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
}

void CFG_Save(void)
{
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&config);

    // ESP_LOGE(TAG, "Creating/open New file: data.txt");
    FILE *f = fopen(SPIFFS_PATH, "w");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    // open file oke
    ESP_LOGE(TAG, "Writing data to file");

    char *json = "{\"name\": \"John\", \"age\": 30, \"isStudent\": true}";

    
    size_t len = strlen(json);

    char *data = malloc(len + 1); // Cần cấp phát thêm 1 byte cho ký tự NULL
    memset(data, 0, len + 1);
    strcpy(data, json);

    // printf("%s\n", json);

    fprintf(f, data);

    fclose(f); // đóng file

    ESP_LOGI(TAG, "File written");
    // giải phóng bộ nhớ
    free(data);
}

void CFG_Load(void)
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

        char buffer[size +1];

        while (fgets(buffer, sizeof(buffer), file) != NULL)
        {
            printf("%s\n", buffer);
        }
        fclose(file);

        // Chuyển đổi chuỗi JSON thành đối tượng JSON
        cJSON *jsondata = cJSON_Parse(buffer);
        cJSON *nameObject = cJSON_GetObjectItem(jsondata, "name");
        char *name = nameObject->valuestring;

        printf("name: %s\n", name);

        cJSON_Delete(jsondata);
    }
    esp_vfs_spiffs_unregister(NULL);
}