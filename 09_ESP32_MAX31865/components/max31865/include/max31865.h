#ifndef ESP32_MAX31865_H
#define ESP32_MAX31865_H

#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define MAX31865_CONFIG_REG         (0x00)
#define MAX31865_RTD_REG        (0x01)
#define MAX31865_HIGH_FAULT_REG (0x03)
#define MAX31865_LOW_FAULT_REG  (0x05)
#define MAX31865_FAULT_STATUS_REG   (0x07)

#define MAX31865_CONFIG_FILTER_BIT 		BIT(0)
#define MAX31865_CONFIG_FAULT_CLEAR 	BIT(1)
#define MAX31865_CONFIG_FAULT_D2		BIT(2)
#define MAX31865_CONFIG_FAULT_D3		BIT(3)
#define MAX31865_CONFIG_NWIRES_BIT		BIT(4)
#define MAX31865_CONFIG_1SHOT_BIT		BIT(5)
#define MAX31865_CONFIG_CONVERSIONMODE_BIT	BIT(6)
#define MAX31865_CONFIG_VBIAS_BIT		BIT(7)

#define MASK_CONFIG_FAULT (BIT_CONFIG_FAULT_D2 | BIT_CONFIG_FAULT_D3)


/* Conversion mode*/
typedef enum
{
	MAX31865_MODE_SINGLE = 0, /**< Single consersion mode, default */
	MAX31865_MODE_AUTO 		  /**< Automatic conversion mode at 50/60Hz rate*/
} max31865_mode_t;

/* Notch frequencies for the noise rejection filter */
typedef enum
{
	MAX31865_FILTER_60HZ = 0, /**< 60Hz*/
	MAX31865_FILTER_50HZ	  /**< 50Hz*/
} max31865_filter_t;

/* Connection type */
typedef enum
{
	MAX31865_2WIRE = 0,	/**< 2 wires*/
	MAX31865_3WIRE,		/**< 3 wires*/
	MAX31865_4WIRE		/**< 4 wires*/
} max31865_connection_type_t;

typedef enum
{
	NoAction = 0x00,
	AutoDelay = 0x01,
	ManualDelayCycle1 = 0x02,
	ManualDelayCycle2 = 0x03
} max31865_fault_detection;

typedef enum
{
	NoError = 0,
	Voltage = 2,
	RTDInLow,
	RefLow,
	RefHigh,
	RTDLow,
	RTDHigh
} max31865_error_t;

/*Temperature scale standard */
typedef enum {
	MAX31865_ITS90 = 0, /**< ITS-90 */
	MAX31865_DIN43760, /**< DIN43760 */
	MAX31865_US_INDUSTRIAL /**< US INDUSTRIAL*/
} max31865_standard_t;

typedef struct
{
	max31865_mode_t mode;
	max31865_connection_type_t connection;
	bool v_bias;
	max31865_filter_t filter;
} max31865_config_t;

/* Device descriptor */
typedef struct
{	
	int miso, mosi, sck, cs, drdy;			/*< Max31865 connect pin*/
	spi_device_interface_config_t spi_cfg; 	/*< SPI device configuration */
	spi_device_handle_t spi_dev; 			/*< SPI device handler */
	spi_host_device_t host;					/*< SPI host*/
	SemaphoreHandle_t drdySemaphore;
	max31865_standard_t standard; 			/*< Temperature scale standard */
	float r_ref;							/*< Reference resistor value, Ohms */
	float rtd_nominal;						/*< RTD nominal resistance at 0 deg. C, Ohms (PT100 - 100 Ohms, PT1000 - 1000 Ohms) */
} max31865_t;

float RTDtoTemperature(uint16_t rtd, max31865_config_t rtdConfig);
uint16_t temperatureToRTD(float temperature, max31865_config_t rtdConfig);

esp_err_t Max31865_Init(max31865_t *dev,  spi_host_device_t host, int misoPin, int mosiPin, int sckPin, int csPin, int drdyPin);
esp_err_t Max31865_DeInit(max31865_t *dev);

esp_err_t Max31865_SetConfig(max31865_t *dev, const max31865_config_t *config);
esp_err_t Max31865_GetConfig(max31865_t *dev, max31865_config_t *config);
esp_err_t Max31865_Start_Measurement(max31865_t *dev);
esp_err_t Max31865_Measure(max31865_t *dev, float *temp);
esp_err_t Max31865_Read_Raw(max31865_t *dev, uint16_t *raw, bool *fault);
esp_err_t Max31865_Read_Temperature(max31865_t *dev, float *temp);

#endif