/*
 * sht3x.c
 *
 *  Created on: 2023.05.04.
 *  Modifier on: 2023.07.04.
 *      Author: DucHien
 */

#include "sht3x.h"

uint8_t sht31_crc8(const uint8_t *data, int len);
esp_err_t sht31_reset();

static const char* TAG = "duchien";
// static float humidity, temp;

esp_err_t sht31_reset() {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_WRITE,
	SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x30, SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xA2, SHT31_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(SHT31_NUM, cmd,
			1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) {
		return ESP_FAIL;
	}
	return ESP_OK;
}

void sht31_init() {
	int i2c_master_port = SHT31_NUM;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 21,
		.scl_io_num = 22,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = SHT31_FREQ_HZ,
	};
	
	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0));
	vTaskDelay(200 / portTICK_PERIOD_MS);

	ESP_ERROR_CHECK(sht31_reset());

	ESP_LOGI(TAG, "sht31 init oke");
}

// float sht31_GetTemperature() {

// 	return temp;
// }

// float sht31_GetHumidity() {

// 	return humidity;
// }

bool sht31_readTempHum(SHT3x_t *sht3x) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_WRITE,
	SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x24, SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x00, SHT31_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(SHT31_NUM, cmd,
			1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) {
		ESP_LOGE(TAG, "0x2400 Failed");
		return false;
	}

	uint8_t readbuffer[6];

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_READ,
	SHT31_ACK_CHECK_EN);

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 1, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 2, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 3, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 4, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 5, SHT31_NACK_VAL));

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(SHT31_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (ret == ESP_FAIL) {
		sht3x->state = 1;
		ESP_LOGE(TAG, "reading Failed");
		return 1;
	}
	uint16_t ST, SRH;
	ST = readbuffer[0];
	ST <<= 8;
	ST |= readbuffer[1];

	if (readbuffer[2] != sht31_crc8(readbuffer, 2)) {
		sht3x->state = 1;
		ESP_LOGE(TAG, "crc8 : 0 Failed");
		return 1;
	}

	SRH = readbuffer[3];
	SRH <<= 8;
	SRH |= readbuffer[4];

	if (readbuffer[5] != sht31_crc8(readbuffer + 3, 2)) {
		sht3x->state = 1;
		ESP_LOGE(TAG, "crc8 : 1 Failed");
		return 1;
	}

	double stemp = ST;
	stemp *= 175;
	stemp /= 0xffff;
	stemp = -45 + stemp;
	sht3x->temp = stemp;

	double shum = SRH;
	shum *= 100;
	shum /= 0xFFFF;

	// humidity = shum;
	// printf("%2.2f\n", shum);
	sht3x->humi = shum;
	sht3x->state = 0;

	return 0;
}

uint8_t sht31_crc8(const uint8_t *data, int len) 
{
	/*
	 *
	 * CRC-8 formula from page 14 of SHT spec pdf
	 *
	 * Test data 0xBE, 0xEF should yield 0x92
	 *
	 * Initialization data 0xFF
	 * Polynomial 0x31 (x8 + x5 +x4 +1)
	 * Final XOR 0x00
	 */

	const uint8_t POLYNOMIAL = 0x31;
	uint8_t crc = 0xFF;

	for (int j = len; j; --j) {
		crc ^= *data++;

		for (int i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
		}
	}
	return crc;
}
