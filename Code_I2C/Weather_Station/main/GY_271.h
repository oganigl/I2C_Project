#ifndef GY_271
#define GY_271
#include <limits.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "i2c_new_bus.h"



void init_GY_271();
float read_data_bytes();
void calculo_offset();

//void calculate_measure();


#endif