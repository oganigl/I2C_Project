#ifndef AHT20_H
#define AHT20_H

#include "i2c_new_bus.h"

void get_AHT20_packet();
void get_temperature_humidity(float *temp, float *hum);

#endif