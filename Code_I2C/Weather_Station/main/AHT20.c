#include "AHT20.h"

#define ADDRS_AHT20 0x38

static float temperature_in_degrees, humidity_percent;

static uint32_t temperature_byte, humidity_byte;

void get_temperature_humidity(float *temp, float *hum)
{
    *temp = temperature_in_degrees;
    *hum = humidity_percent;

}
static void calculate_measure()
{
    temperature_in_degrees = (float)temperature_byte * 200.0F / 1048576.0F - 50;
    humidity_percent = humidity_byte * 100.0F / 1048576.0F;
}

void get_AHT20_packet()
{
    uint8_t buffer[7] ={0, 0, 0, 0, 0, 0, 0};
    esp_err_t err=0;
    //Escribir
    uint8_t command[3] = {0xAC, 0x33, 0x00};

    err+=i2c_master_write_to_device(I2C_MASTER_NUM, ADDRS_AHT20, 
                                command,
                                3,
                                TIME_OUT / portTICK_PERIOD_MS);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADDRS_AHT20 << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);


    humidity_byte = ((buffer[1] << 16) | (buffer[2] << 8) | (buffer[3])) >> 4;
    temperature_byte= ((buffer[3] & 0x0F) << 16) | (buffer[4] << 8) | (buffer[5]);
    calculate_measure();

}