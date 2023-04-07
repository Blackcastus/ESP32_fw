#ifndef _NTT_H_
#define _NTT_H_
#include <stdio.h>
#include <stdint.h>
#include <esp_log.h>
#include "string.h"
#include "cJSON.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_system.h"

#define CFG_HOLDER  					  0x00FF55AB

typedef struct
{
    uint32_t 		cfg_holder;
    char            *ssid;
    char            *pass;

}NTT_t;

NTT_t sys_cfg;

void Ntt_Init();
esp_err_t Ntt_save();
esp_err_t Ntt_load();

#endif