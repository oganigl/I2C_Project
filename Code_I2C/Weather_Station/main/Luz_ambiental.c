
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "Luz_ambiental.h"
#include "driver/i2c.h"

#define VEML7700_I2C_ADDR 0x10 /*!< Sensor slave I2C address */

#define VEML7700_ALS_CONFIG 0x00        /*!< Light configuration register */
#define VEML7700_ALS_THREHOLD_HIGH 0x01 /*!< Light high threshold for irq */
#define VEML7700_ALS_THREHOLD_LOW 0x02  /*!< Light low threshold for irq */
#define VEML7700_ALS_POWER_SAVE 0x03    /*!< Power save register */
#define VEML7700_ALS_DATA 0x04          /*!< The light data output */
#define VEML7700_WHITE_DATA 0x05        /*!< The white light data output */
#define VEML7700_INTERRUPTSTATUS 0x06   /*!< What IRQ (if any) */

#define VEML7700_INTERRUPT_HIGH 0x4000  /*!< Interrupt status for high threshold */
#define VEML7700_INTERRUPT_LOW 0x8000   /*!< Interrupt status for low threshold */

#define VEML7700_GAIN_2 0x01            /*!< ALS gain 2x */
#define VEML7700_GAIN_1 0x00            /*!< ALS gain 1x */
#define VEML7700_GAIN_1_8 0x02          /*!< ALS gain 1/8x */
#define VEML7700_GAIN_1_4 0x03          /*!< ALS gain 1/4x */

#define VEML7700_IT_800MS 0x03          /*!< ALS intetgration time 800ms */
#define VEML7700_IT_400MS 0x02          /*!< ALS intetgration time 400ms */
#define VEML7700_IT_200MS 0x01          /*!< ALS intetgration time 200ms */
#define VEML7700_IT_100MS 0x00          /*!< ALS intetgration time 100ms */
#define VEML7700_IT_50MS 0x08           /*!< ALS intetgration time 50ms */
#define VEML7700_IT_25MS 0x0C           /*!< ALS intetgration time 25ms */

#define VEML7700_PERS_1 0x00            /*!< ALS irq persisance 1 sample */
#define VEML7700_PERS_2 0x01            /*!< ALS irq persisance 2 samples */
#define VEML7700_PERS_4 0x02            /*!< ALS irq persisance 4 samples */
#define VEML7700_PERS_8 0x03            /*!< ALS irq persisance 8 samples */

#define VEML7700_POWERSAVE_MODE1 0x00   /*!< Power saving mode 1 */
#define VEML7700_POWERSAVE_MODE2 0x01   /*!< Power saving mode 2 */
#define VEML7700_POWERSAVE_MODE3 0x02   /*!< Power saving mode 3 */
#define VEML7700_POWERSAVE_MODE4 0x03   /*!< Power saving mode 4 */

#define VEML7700_GAIN_OPTIONS_COUNT 4	/*!< Possible gain values count */
#define VEML7700_IT_OPTIONS_COUNT 6		/*!< Possible integration time values count */

#define LUX_FC_COEFFICIENT 0.092903     /*!< Multiplier coefficient for lux-fc conversion */
#define FC_LUX_COEFFICIENT 10.7639      /*!< Multiplier coefficient for fc-lux conversion */

#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

typedef struct
{
    uint16_t gain;				/*!< Ganancia del sensor*/
	uint16_t integration_time;	/*!< Sensor integration time configuration */
	uint16_t persistance;		/*!< Last result persistance on-sensor configuration */
	uint16_t interrupt_enable;	/*!< Enable/disable interrupts */
	uint16_t shutdown;			/*!< Shutdown command configuration */
    float resolution;			/*!< Current resolution and multiplier */
    uint32_t maximum_lux;		/*!< Current maximum lux limit */
}Config_veml7700;

//Variables globales
Config_veml7700 sensor_luz;
static const char* VEML7700_TAG = "VEML7700";
const uint8_t veml7700_slave_address = VEML7700_I2C_ADDR;
uint8_t command_config=VEML7700_ALS_CONFIG;
uint8_t command_read=VEML7700_ALS_DATA;
uint8_t master_num=I2C_MASTER_NUM;

esp_err_t config_veml7700()
{
    esp_err_t error=0;
    //elijo la configuración que quiera la resolución y maxima luz dependen de ganancia y integration_time (esta en una tabla del datasheet)
    sensor_luz.gain=VEML7700_GAIN_1_8;
    sensor_luz.integration_time=VEML7700_IT_100MS;
    sensor_luz.persistance=VEML7700_PERS_1;
    sensor_luz.interrupt_enable=false;
    sensor_luz.shutdown=VEML7700_POWERSAVE_MODE1;
    sensor_luz.resolution=0.5376;
    sensor_luz.maximum_lux=35232;
    
    //METO los datos del sensor que voy a enviar para configurarlo
    uint16_t config_data =(
        (sensor_luz.gain<<11) |
        (sensor_luz.integration_time<<6) |
        (sensor_luz.persistance<<4) |
        (sensor_luz.interrupt_enable<<1) |
        (sensor_luz.shutdown<<0)
    );

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	error+=i2c_master_start(cmd);
	error+=i2c_master_write_byte(cmd, (veml7700_slave_address << 1) | I2C_MASTER_WRITE, true);
	error+=i2c_master_write_byte(cmd,command_config, true);

	uint8_t write_data[2];
	write_data[0] = config_data&0xff;
	write_data[1] = (config_data>>8)&0xff;
	error+=i2c_master_write(cmd, write_data, 2, true);
	
	i2c_master_stop(cmd);

	error+= i2c_master_cmd_begin(master_num, cmd, 1000 / portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);
    return ESP_OK;
}
esp_err_t veml7700_read_als_lux(float *lux)
{
    esp_err_t error=0;
    uint16_t valor_leido;
    uint8_t read_data[2];//guardamos lo que lea
    //lectura en i2c
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	error+=i2c_master_start(cmd);
	error+=i2c_master_write_byte(cmd, (veml7700_slave_address << 1) | I2C_MASTER_WRITE, true);
	error+=i2c_master_write_byte(cmd, command_read, true);

	error+=i2c_master_start(cmd);
	error+=i2c_master_write_byte(cmd, (veml7700_slave_address << 1) | I2C_MASTER_READ, true);
	
	
	error+=i2c_master_read(cmd, read_data, 2, I2C_MASTER_LAST_NACK);
	error+=i2c_master_stop(cmd);

	error+= i2c_master_cmd_begin(master_num, cmd, 2000 / portTICK_PERIOD_MS);

	valor_leido = read_data[0] | (read_data[1]<<8);
	i2c_cmd_link_delete(cmd);
    if(error!=0)
    {
        ESP_LOGW(VEML7700_TAG, "Problema al leer los datos");
		return ESP_OK;
    }
    *lux=valor_leido * sensor_luz.resolution;
    return error;
}