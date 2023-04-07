/*
 * sht3x.h
 *
 *  Created on: 2023.05.04.
 *  Modifier on: 2023.07.04.
 *      Author: DucHien
 */

#include <stdbool.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include <driver/i2c.h>
#include <esp_log.h>

#ifndef __SHT3X_H_
#define __SHT3X_H_

#define SHT31_NUM 				I2C_NUM_0
#define SHT31_SCL_IO    		22	    /*!< gpio number for I2C master clock */
#define SHT31_SDA_IO    	    21    /*!< gpio number for I2C master data  */
#define SHT31_FREQ_HZ    		100000     /*!< I2C master clock frequency */
#define SHT31_TX_BUF_DISABLE   	0   /*!< I2C master do not need buffer */
#define SHT31_RX_BUF_DISABLE   	0   /*!< I2C master do not need buffer */
#define SHT31_ADDRESS 			0x44
#define SHT31_SOFTRESET			0x30A2
#define SHT31_ACK_CHECK_EN   	0x1     /*!< I2C master will check ack from slave*/
#define SHT31_ACK_CHECK_DIS  	0x0     /*!< I2C master will not check ack from slave */
#define SHT31_ACK_VAL    0x0         /*!< I2C ack value */
#define SHT31_NACK_VAL   0x1         /*!< I2C nack value */

typedef struct
{
    bool state;         //state = 0 -> connect, state = 1 -> no connect
    float temp;         //value temp
    float humi;         //value humi
}SHT3x_t;

void sht31_init(); // init i2c port
bool sht31_readTempHum(SHT3x_t *sht3x);

#endif /* _COMPONENT_SHT31_H_ */
