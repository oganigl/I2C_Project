
#include "BMP280.h"



static int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint16_t dig_T1, dig_P1;

static int32_t adc_P, adc_T;

static float pressure_in_pascal, temperature_in_degrees;

static esp_err_t is_not_init = 1;

//inicialización de valores por defecto, leidos una sola vez y siempre iguales
static esp_err_t get_calibration_data()
{
    esp_err_t err = 0;
    uint8_t data [26] = {};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    err +=i2c_master_start(cmd);
    err +=i2c_master_write_byte(cmd, ADDRS_BMP280 << 1 | I2C_MASTER_WRITE, true);
    err +=i2c_master_write_byte(cmd, ADDRS_CALIBRATION, true);
     err +=i2c_master_start(cmd);
    err +=i2c_master_write_byte(cmd, ADDRS_BMP280 << 1 | I2C_MASTER_READ, true);
    err+= i2c_master_read(cmd, data, 26, I2C_MASTER_LAST_NACK);
    err+=i2c_master_stop(cmd);
    err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];

    return err;

} 

//establece el byte de configuración
static esp_err_t set_config_byte()
{
    esp_err_t err = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    err +=i2c_master_start(cmd);
    err +=i2c_master_write_byte(cmd, ADDRS_BMP280 << 1 | I2C_MASTER_WRITE, true);
    err +=i2c_master_write_byte(cmd, ADDRS_CONFIG, true);
    err+=i2c_master_write_byte(cmd, 0xA0, true); //A0 es el valor para la configuración por defecto
    err+=i2c_master_stop(cmd);
    err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

//inicia el sensor BMP280 escribe bytes de configuración obtiene valores de calibración
void init_BMP280()
{
    esp_err_t err=0;
    err=set_config_byte();
    if(err!=ESP_OK)
    {
        //printf("Error en config");
        return;
    }
    err=get_calibration_data();
    if(err!=ESP_OK)
    {
        //printf("Error en get_calibration");
        return;
    }
    is_not_init=err;
}


static void calculate_measure()
{
    int32_t var1, var2, T, t_fine;
    int64_t var3, var4, P;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    var3 = ((int64_t)t_fine) - 128000;
    var4 = var3 * var3 * (int64_t)dig_P6;
    var4 = var4 + ((var3*(int64_t)dig_P5)<<17);
    var4 = var4 + (((int64_t)dig_P4)<<35);
    var3 = ((var3 * var3 * (int64_t)dig_P3)>>8) + ((var3 * (int64_t)dig_P2)<<12);
    var3 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
    if (var3 == 0)
    {
    return; // avoid exception caused by division by zero
    }
    P = 1048576-adc_P;
    P = (((P<<31)-var4)*3125)/var3;
    var1 = (((int64_t)dig_P9) * (P>>13) * (P>>13)) >> 25;
    var4 = (((int64_t)dig_P8) * P) >> 19;
    P = ((P + var1 + var4) >> 8) + (((int64_t)dig_P7)<<4);
    pressure_in_pascal = (P/256.0)/100.0f;
    temperature_in_degrees = T/100.0;
    //printf("Presion: %f Pa Temperatura: %f ºC", pressure_in_pascal, temperature_in_degrees);
}

//copia el valor de la presion
void get_pressure(float *press)
{
    *press = pressure_in_pascal;
}


//Establece la comunicación para configurar en un modo forzado
static esp_err_t write_command_bytes()
{
    uint8_t err = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    err +=i2c_master_start(cmd);
    err +=i2c_master_write_byte(cmd, ADDRS_BMP280 << 1 | I2C_MASTER_WRITE, true);
    err +=i2c_master_write_byte(cmd, ADDRS_CRTL, true);
    err+=i2c_master_write_byte(cmd, 0X25, true);
    err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS);
    err+=i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
    return err;
}

//Se comunica con i2c para leer los bytes del sensor en el registro
static esp_err_t read_data_bytes()
{
    esp_err_t err=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    uint8_t buffer_read[6] = {0, 0, 0, 0, 0, 0};
    err +=i2c_master_start(cmd);
    err +=i2c_master_write_byte(cmd, ADDRS_BMP280 << 1 | I2C_MASTER_WRITE, true);
    err +=i2c_master_write_byte(cmd, ADDRS_READ, true);
    err +=i2c_master_start(cmd);
    err +=i2c_master_write_byte(cmd, ADDRS_BMP280 << 1 | I2C_MASTER_READ, true);
    err+= i2c_master_read(cmd, buffer_read, 6, I2C_MASTER_LAST_NACK);
    err+=i2c_master_stop(cmd);
    err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    
    adc_P = buffer_read[0] << 12 | buffer_read[1]<<4 | buffer_read[2]>>4;
    adc_T = buffer_read[3] << 12 | buffer_read[4]<<4 | buffer_read[5]>>4;
    return err;
}


//obtiene la medida, primero manda los bytes de configuración para forzado y despues lee los bytes y calcula las medidas
void get_BMP280_packet()
{
    if(is_not_init==ESP_OK)
    {
        esp_err_t err=0;
    
        err+=write_command_bytes();
        if(err!=ESP_OK){
           // printf("Error en write\n");
            return;
        }
        //printf("Write correcto\n");
        err+=read_data_bytes();    //uint8_t read_addrs[1]
        if(err){
            //printf("Error en read\n");
            return;
        }
        //printf("Read correcto\n");
        calculate_measure();
    }
    else
    init_BMP280();
}



