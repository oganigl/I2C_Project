#ifndef VEML7700_H
#define VEML7700_H

#include "esp_err.h"

esp_err_t config_veml7700(void);
esp_err_t veml7700_read_als_lux(float *lux);

#endif