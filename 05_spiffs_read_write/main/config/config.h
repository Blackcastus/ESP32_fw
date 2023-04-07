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
#include <esp_log.h>
#include "string.h"
#include "cJSON.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_system.h"

typedef struct
{
    char *ssid;
    char *pass;
}CONFIG_t;


void CFG_Init(void);
void CFG_Save(void);
void CFG_Load(void);

#endif /* _COMPONENT_CONFIG_H_ */