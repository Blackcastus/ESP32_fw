/*
 * config.h
 *  save data system
 *  Created on: 2023.07.04.
 *  Modifier on: 2023.10.04.
 *      Author: DucHien
 */

#include "config.h"

#define SPIFFS_PATH "/spiffs/hello.txt"
static const char *TAG = "Read_Write_Data";

CONFIG_t sys_cfg;

esp_err_t CFG_Spiffs_Init(void)
{
    esp_err_t err;
    err = nvs_flash_init();
    ESP_LOGE(TAG, "%d", err);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&config);
    
    CFG_Spiffs_Load();

    if(sys_cfg.cfg_holder != CFG_HOLDER)
    {
        sys_cfg.cfg_holder = CFG_HOLDER;
        strcpy(sys_cfg.ssid, "PhuongHai");
        strcpy(sys_cfg.pass, "phuongHai123");
        CFG_Spiffs_Save();
    }
    

    return err;

}
Error_Spiffs_t CFG_Spiffs_Save(void)
{

    FILE *file;

    // open file
    file = fopen(SPIFFS_PATH, "wb");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ERR_OPEN_FILE;
    }
    ESP_LOGI(TAG, "Writing data to file");

    fwrite(&sys_cfg, sizeof(CONFIG_t), 1, file);
    fclose(file); // dùng để đóng file được mở ra, cũng như giải phóng bộ nhớ chương trình.

    return SPIFFS_OK;
}

Error_Spiffs_t CFG_Spiffs_Load(void)
{
    FILE *file = fopen(SPIFFS_PATH, "rb");
    if (!file)
    {
        ESP_LOGE(TAG, "File does not exist!");
        return ERR_OPEN_FILE;
    }

    fseek(file, 0, SEEK_END); // di chuyển con trỏ tới vị trí CUỐI file
    long size = ftell(file);  // trả về vị trí hiện tại của con trỏ file đó chính là kích thước file
    fseek(file, 0, SEEK_SET); // di chuyển con trỏ tới vị trí ĐẦU file
    printf("size file: %ld\n", size);

    if(size == 0)
    {
        ESP_LOGE(TAG, "Data NULL");
        return ERR_NODATA;
    }

    fread(&sys_cfg, sizeof(CONFIG_t), 1, file); // EOF ký tự kết thúc file

    printf("ssid: %s\n", sys_cfg.ssid);
    printf("pass: %s\n", sys_cfg.pass);

    fclose(file);
    return SPIFFS_OK;
}