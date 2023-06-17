#include "max31865.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

static const char *TAG = "max31865";
#define CLOCK_SPEED_HZ 	(1000000)	//1MHz

typedef struct 
{
	float a, b;
} rtd_coeff_t;

static const rtd_coeff_t rtd_coeff[] = {
	[MAX31865_ITS90]         = { .a = 3.9083e-3f, .b = -5.775e-7f },
    [MAX31865_DIN43760]      = { .a = 3.9848e-3f, .b = -5.8019e-7f },
    [MAX31865_US_INDUSTRIAL] = { .a = 3.9692e-3f, .b = -5.8495e-7f },
};

#define CHECK(x) do {esp_err_t __; if((__ = x) != ESP_OK) return __; } while(0)
#define CHECK_ARG(VAL)	do {if(!(VAL)) return ESP_ERR_INVALID_ARG; } while(0)

static esp_err_t write_reg_8(max31865_t *dev, uint8_t reg, uint8_t val)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(spi_transaction_t));
	uint8_t tx[] = {reg | 0x80, val};

	t.tx_buffer = tx;
	t.length = sizeof(tx) * 8;

	return spi_device_transmit(dev->spi_dev, &t);
}

static esp_err_t read_reg_8(max31865_t *dev, uint8_t reg, uint8_t *val)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(spi_transaction_t));

	uint8_t tx[] = {reg, 0};
	uint8_t rx[sizeof(tx)];

	t.tx_buffer = tx;
	t.rx_buffer = rx;
	t.length = sizeof(tx) * 8;

	CHECK(spi_device_transmit(dev->spi_dev, &t));
	*val = rx[1];
	return ESP_OK;
}

static esp_err_t read_reg_16(max31865_t *dev, uint8_t reg, uint16_t *val)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(spi_transaction_t));

	uint8_t tx[] = {reg, 0, 0};
	uint8_t rx[sizeof(tx)];

	t.tx_buffer = tx;
	t.rx_buffer = rx;
	t.length = sizeof(tx) * 8;
	CHECK(spi_device_transmit(dev->spi_dev, &t));

	*val = (uint16_t)rx[1] << 8;
	*val |= rx[2];
	return ESP_OK;
}

// esp_err_t writeSPI(uint8_t addr, uint8_t *data, size_t size)
// {
// 	assert(size <= 4);	//we're using the transaction buffers
// 	spi_transaction_t transaction = {};
// 	transaction.length = CHAR_BIT * size;
// 	transaction.rxlength = 0;
// 	transaction.addr = addr | MAX31865_REG_WRITE_OFFSET;
// 	transaction.flags = SPI_TRANS_USE_TXDATA;
// 	memcpy(transaction.tx_data, data, size);
// 	gpio_set_level((gpio_num_t)cs, 0);
// 	esp_err_t err = spi_device_polling_transmit(deviceHandle, &transaction);
// 	gpio_set_level((gpio_num_t)cs, 1);
// 	return err;
// }

// esp_err_t readSPI(uint8_t addr, uint8_t *result, size_t size)
// {
// 	assert(size <= 4); //we're using the transaction buffers
// 	spi_transaction_t transaction = {};
// 	transaction.length = 0;
// 	transaction.rxlength = CHAR_BIT * size;
// 	transaction.addr = addr & (MAX31865_REG_WRITE_OFFSET - 1);
// 	transaction.flags = SPI_TRANS_USE_RXDATA;
// 	gpio_set_level((gpio_num_t)cs, 0);
// 	esp_err_t err = spi_device_polling_transmit(deviceHandle, &transaction);
// 	gpio_set_level((gpio_num_t)cs, 1);
// 	if(err != ESP_OK) {
// 		ESP_LOGE(TAG, "Error sending SPI transaction: %s", esp_err_to_name(err));
// 		return err;
// 	}
// 	printf("trans: %d\n", *transaction.rx_data);
// 	memcpy(result, transaction.rx_data, size);
// 	return ESP_OK;
// }

const char *errorToString(max31865_error_t error)
{
	switch(error)
	{
		case NoError:
			return "No error";
		case Voltage:
			return "Over/under Voltage fault";
		case RTDInLow:
			return "RTDIN- < 0.85*VBIAS (FORCE-open)";
		case RefLow:
			return "REFIN- < 0.85*VBIAS (FORCE-open)";
		case RefHigh:
			return "REFIN- > 0.85*VBIAS";
		case RTDLow:
			return "RTD below low threshold";
		case RTDHigh:
			return "RTD above high threshold";
	}
	return "";
}

void IRAM_ATTR drdyInterruptHandler(void *arg)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR((SemaphoreHandle_t) arg, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}
}

esp_err_t Max31865_Init(max31865_t *dev, spi_host_device_t host, int misoPin, int mosiPin, int sckPin, int csPin, int drdyPin)
{
	CHECK_ARG(dev);
	memset(&dev->spi_cfg, 0, sizeof(dev->spi_cfg));
	dev->miso = misoPin;
	dev->mosi = mosiPin;
	dev->sck = sckPin;
	dev->cs = csPin;
	dev->drdy = drdyPin;
	dev->host = host;

	if(drdyPin > -1) {
		gpio_config_t gpioConfig = {};
		gpioConfig.intr_type = GPIO_INTR_NEGEDGE;
		gpioConfig.mode = GPIO_MODE_INPUT;
		gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
		gpioConfig.pin_bit_mask = 1ULL << drdyPin;
		gpio_config(&gpioConfig);

		dev->drdySemaphore = xSemaphoreCreateBinary();
		// There won't be a negative edge interrupt if it's already low
		if(gpio_get_level((gpio_num_t)drdyPin) == 0) {
			xSemaphoreGive(dev->drdySemaphore);
		}
		gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
		gpio_isr_handler_add((gpio_num_t)drdyPin, &drdyInterruptHandler, dev->drdySemaphore);
	}

	spi_bus_config_t busConfig = {
		.miso_io_num = dev->miso,
		.mosi_io_num = dev->mosi,
		.sclk_io_num = dev->sck,
		.quadhd_io_num = -1,
		.quadwp_io_num = -1,
	};
	esp_err_t err = spi_bus_initialize(host, &busConfig, 0);
	//INVALID_STATE means the host is already in use - that's OK
	if(err == ESP_ERR_INVALID_STATE) {
		ESP_LOGD(TAG, "SPI bus already initialized");
	} else if(err != ESP_OK) {
		ESP_LOGE(TAG, "Error initialising SPI bus: %s", esp_err_to_name(err));
		return err;
	}
	dev->spi_cfg.spics_io_num = dev->cs; 
	dev->spi_cfg.clock_speed_hz = CLOCK_SPEED_HZ;
	dev->spi_cfg.mode = 1;
	//dev->spi_cfg.address_bits = CHAR_BIT;
	//dev->spi_cfg.command_bits = 0;
	//dev->spi_cfg.flags = SPI_DEVICE_HALFDUPLEX;
	dev->spi_cfg.queue_size = 1;
	dev->spi_cfg.cs_ena_pretrans = 1;
	err = spi_bus_add_device(host, &dev->spi_cfg, &dev->spi_dev);
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));
		return err;
	}
	return err;
}

esp_err_t Max31865_DeInit(max31865_t *dev)
{
	spi_bus_remove_device(dev->spi_dev);
	esp_err_t err = spi_bus_free(dev->host);
	if(err == ESP_OK) {
		gpio_uninstall_isr_service();
	} else if(err == ESP_ERR_INVALID_STATE) {
		ESP_LOGD(TAG, "Devices still attached; not freeing the bus");
	} else {
		ESP_LOGE(TAG, "Error freeing bus: %s", esp_err_to_name(err));
	}
	return ESP_OK;
}

esp_err_t Max31865_SetConfig(max31865_t *dev, const max31865_config_t *config)
{
	CHECK_ARG(dev && config);
	uint8_t val;
	CHECK(read_reg_8(dev, MAX31865_CONFIG_REG, &val));

	val &= ~(MAX31865_CONFIG_CONVERSIONMODE_BIT | MAX31865_CONFIG_NWIRES_BIT | MAX31865_CONFIG_VBIAS_BIT | MAX31865_CONFIG_FILTER_BIT);

	val |= config->mode == MAX31865_MODE_AUTO ? MAX31865_CONFIG_CONVERSIONMODE_BIT : 0;
	val |= config->connection == MAX31865_3WIRE ? MAX31865_CONFIG_NWIRES_BIT : 0;
	val |= config->v_bias ? MAX31865_CONFIG_VBIAS_BIT : 0;
	val |= config->filter == MAX31865_FILTER_50HZ ? MAX31865_CONFIG_FILTER_BIT : 0;

	CHECK(write_reg_8(dev, MAX31865_CONFIG_REG, val));

	return ESP_OK;
}

esp_err_t Max31865_GetConfig(max31865_t *dev, max31865_config_t *config)
{

	CHECK_ARG(dev && config);
	uint8_t val;

	CHECK(read_reg_8(dev, MAX31865_CONFIG_REG, &val));
	config->filter = val & MAX31865_CONFIG_FILTER_BIT ? MAX31865_FILTER_50HZ : MAX31865_FILTER_60HZ;
	config->v_bias = val & MAX31865_CONFIG_VBIAS_BIT ? 1 : 0;
	config->mode = val & MAX31865_CONFIG_CONVERSIONMODE_BIT ? MAX31865_MODE_AUTO : MAX31865_MODE_SINGLE;
	config->connection = val & MAX31865_CONFIG_NWIRES_BIT ? MAX31865_3WIRE : MAX31865_2WIRE;

	return ESP_OK;
}

esp_err_t Max31865_Start_Measurement(max31865_t *dev)
{
	CHECK_ARG(dev);

	uint8_t val;
	CHECK(read_reg_8(dev, MAX31865_CONFIG_REG, &val));
	val |= MAX31865_CONFIG_1SHOT_BIT;
	CHECK(write_reg_8(dev, MAX31865_CONFIG_REG, val));
	return ESP_OK;
}

esp_err_t Max31865_Read_Raw(max31865_t *dev, uint16_t *raw, bool *fault)
{
	CHECK_ARG(dev && raw && fault);

	CHECK(read_reg_16(dev, MAX31865_RTD_REG, raw));
	*fault = *raw & 1;
	*raw >>= 1;
	return ESP_OK;
}

esp_err_t Max31865_Read_Temperature(max31865_t *dev, float *temp)
{
	CHECK_ARG(dev && temp && dev->standard <= MAX31865_US_INDUSTRIAL);
	uint16_t raw;
	bool fault;
	CHECK(Max31865_Read_Raw(dev, &raw, &fault));
	if(fault)
	{
		ESP_LOGE(TAG, "[CS %d] Fault detected", dev->spi_cfg.spics_io_num);
		return ESP_FAIL;
	}
	float r_rtd = raw * dev->r_ref/32768;
	ESP_LOGD(TAG, "[CS %d] RTD resistance: %.8f", dev->spi_cfg.spics_io_num, r_rtd);

	const rtd_coeff_t *c = rtd_coeff + dev->standard;

	*temp = (sqrtf((c->a * c->a - (4 * c->b)) + (4 * c->b / dev->rtd_nominal * r_rtd)) - c->a) / (2 * c->b);

    if (*temp >= 0)
        return ESP_OK;

    // below zero
    r_rtd = r_rtd / dev->rtd_nominal * 100; // normalize to 100 Ohm

    float rpoly = r_rtd;

    *temp = -242.02;
    *temp += 2.2228 * rpoly;
    rpoly *= r_rtd; // square
    *temp += 2.5859e-3 * rpoly;
    rpoly *= r_rtd; // ^3
    *temp -= 4.8260e-6 * rpoly;
    rpoly *= r_rtd; // ^4
    *temp -= 2.8183e-8 * rpoly;
    rpoly *= r_rtd; // ^5
    *temp += 1.5243e-10 * rpoly;

    return ESP_OK;
}

esp_err_t Max31865_Measure(max31865_t *dev, float *temp)
{
    CHECK(Max31865_Start_Measurement(dev));
    vTaskDelay(pdMS_TO_TICKS(70));
    return Max31865_Read_Temperature(dev, temp);
}


// esp_err_t Max31865_SetRTDThresholds(uint16_t min, uint16_t max)
// {
// 	assert((min < (1 << 15)) && (max < (1 << 15)));
// 	uint8_t thresholds[4];
// 	thresholds[0] = (uint8_t)((max << 1) >> CHAR_BIT);
// 	thresholds[1] = (uint8_t)(max << 1);
// 	thresholds[2] = (uint8_t)((min << 1) >> CHAR_BIT);
// 	thresholds[3] = (uint8_t)(min << 1);
// 	return writeSPI(MAX31865_HIGH_FAULT_REG, thresholds, sizeof(thresholds));
// }

esp_err_t Max31865_ClearFault(max31865_t *dev)
{
	CHECK_ARG(dev);
	uint8_t val;
	CHECK(read_reg_8(dev, MAX31865_CONFIG_REG, &val));
	val &= ~(MAX31865_CONFIG_1SHOT_BIT | MAX31865_CONFIG_FAULT_D2 | MAX31865_CONFIG_FAULT_D3);
	val |= MAX31865_CONFIG_FAULT_CLEAR;
	CHECK(write_reg_8(dev, MAX31865_CONFIG_REG, val));
	//esp_err_t err = readSPI(MAX31865_CONFIG_REG, &configByte, 1);
	// if(err != ESP_OK) {
	// 	ESP_LOGE(TAG, "Error reading config: %s", esp_err_to_name(err));
	// 	return err;
	// }
	// configByte |= 1U << MAX31865_CONFIG_FAULTSTATUS_BIT;
	// return writeSPI(MAX31865_CONFIG_REG, &configByte, 1);
	return ESP_OK;
}

// esp_err_t Max31865_ReadFaultStatus(max31865_error_t *fault)
// {
// 	*fault = NoError;
// 	uint8_t faultByte = 0;
// 	esp_err_t err = readSPI(MAX31865_FAULT_STATUS_REG, &faultByte, 1);
// 	if(err != ESP_OK) {
// 		ESP_LOGE(TAG, "Error reading fault status: %s", esp_err_to_name(err));
// 		return err;
// 	}
// 	if(faultByte != 0) {
// 		*fault = (Max31865Error)(CHAR_BIT * sizeof(unsigned int) - 1 - __builtin_clz(faultByte));
// 	}
// 	return Max31865_ClearFault();
// }

// esp_err_t Max31865_GetRTD(uint16_t *rtd, max31865_error_t *fault)
// {
// 	max31865_config_t oldConfig = chipConfig;
// 	bool restoreConfig = false;
// 	if(!chipConfig.vbias) {
// 		restoreConfig = true;
// 		chipConfig.vbias = true;
// 		esp_err_t err = Max31865_SetConfig(chipConfig);
// 		if(err != ESP_OK) {
// 			ESP_LOGE(TAG, "Error setting config: %s", esp_err_to_name(err));
// 			return err;
// 		}
// 		vTaskDelay(pdMS_TO_TICKS(10));
// 	}
// 	if(!chipConfig.autoConversion) {
// 		restoreConfig = true;
// 		uint8_t configByte = 0;
// 		esp_err_t err = readSPI(MAX31865_CONFIG_REG, &configByte, 1);
// 		if(err != ESP_OK) {
// 			ESP_LOGE(TAG, "Error reading config: %s", esp_err_to_name(err));
// 			return err;
// 		}
// 		configByte |= 1U << MAX31865_CONFIG_1SHOT_BIT;
// 		err = writeSPI(MAX31865_CONFIG_REG, &configByte, 1);
// 		if(err != ESP_OK){
// 			ESP_LOGE(TAG, "Error writing config: %s", esp_err_to_name(err));
// 			return err;
// 		}
// 		vTaskDelay(pdMS_TO_TICKS(65));
// 	} else if(drdy > -1) {
// 		xSemaphoreTake(drdySemaphore, portMAX_DELAY);
// 	}
// 	uint8_t rtdByte[2];
// 	esp_err_t err = readSPI(MAX31865_RTD_REG, rtdByte, 2);
// 	printf("rtdByte[0]: %d  \n", rtdByte[0]);
// 	printf("rtdByte[1]: %d  \n", rtdByte[1]);
// 	if(err != ESP_OK) {
// 		ESP_LOGE(TAG, "Error reading RTD: %s", esp_err_to_name(err));
// 		return err;
// 	}
// 	if((bool)(rtdByte[1] & 1U)) {
// 		*rtd = 0;
// 		if(fault == NULL) {
// 			int tmp = NoError;
// 			fault = &tmp;
// 		}
// 		Max31865_ReadFaultStatus(fault);
// 		ESP_LOGW(TAG, "Sensor fault detected: %s", errorToString(*fault));
// 		return ESP_ERR_INVALID_RESPONSE;
// 	}
// 	*rtd = rtdByte[0] << CHAR_BIT;
// 	*rtd |= rtdByte[1];
// 	*rtd >>= 1U;
// 	return restoreConfig ? Max31865_SetConfig(oldConfig):ESP_OK;
// }
