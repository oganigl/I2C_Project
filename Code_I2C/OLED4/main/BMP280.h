#ifndef BMP280_H
#define BMP280_H

#include "i2c_new_bus.h"

#define ADDRS_BMP280 0x76
#define ADDRS_CONFIG 0xF5
#define ADDRS_CRTL 0xF4
#define ADDRS_READ 0xF7
#define ADDRS_CALIBRATION 0x88


void get_BMP280_packet(void);
void init_BMP280(void);
void get_pressure(float *press);


#endif