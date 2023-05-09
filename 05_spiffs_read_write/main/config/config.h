/*
 * config.h
 *
 *  Created on: 2023.07.04.
 *  Modifier on: 2023.07.04.
 *      Author: DucHien
 */
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <esp_log.h>
#include "string.h"
#include "cJSON.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_system.h"

#define CFG_HOLDER 0x00FF55AB

typedef struct
{
    char ssid[20];  // ssid wifi
    char pass[20];      // password wifi
    uint32_t cfg_holder;
}CONFIG_t;


typedef enum
{
    SPIFFS_OK = 0,
    ERR_OPEN_FILE,
    ERR_WRITE_TO_FILE,
    ERR_READ_TO_DIE,
    ERR_NODATA
} Error_Spiffs_t;

extern CONFIG_t sys_cfg;

esp_err_t CFG_Spiffs_Init(void);
Error_Spiffs_t CFG_Spiffs_Save(void);
Error_Spiffs_t CFG_Spiffs_Load(void);

#endif /* _COMPONENT_CONFIG_H_ */