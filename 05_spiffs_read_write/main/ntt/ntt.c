#include "ntt.h"

#define SPIFFS_PATH "/spiffs/ntt.txt"
static const char *TAG = "ntt config";
void Ntt_Init()
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

    err = Ntt_load();
    if (err != ESP_OK)
        ESP_LOGI(TAG, "Error (%d) reading data from NVS!", err);

    if (sys_cfg.cfg_holder != CFG_HOLDER)
    {
        ESP_LOGI(TAG, "Add default config");
        memset(&sys_cfg, 0x00, sizeof sys_cfg);
        sys_cfg.cfg_holder = CFG_HOLDER;
        sys_cfg.ssid = "duchien";
        sys_cfg.pass = "1234";
        err = Ntt_save();
        if (err != ESP_OK)
            ESP_LOGI(TAG, "Error (%d) saving data to NVS!", err);
    }
}

esp_err_t Ntt_save(void)
{
    esp_err_t err;

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
        return 1;
    }

    // Lấy kích thước tệp
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    // Read the size of memory spiffs
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS

    char buffer[size +1];
    if(size > 0)
    {
        while (fgets(buffer, sizeof(buffer), f) != NULL)
        {
            printf("%s\n", buffer);
        }
    }
    
    // ESP_LOGE(TAG, "Writing data to file");
    required_size = sizeof(sys_cfg);
    uint32_t *dataSave = malloc(sizeof(sys_cfg));
    memcpy(dataSave, &sysCfg, sizeof(sysCfg));
    fprintf(f,"%d\n", dataSave);

    fclose(f); // đóng file

    return 0;
}
esp_err_t Ntt_load()
{
    return 0;
}