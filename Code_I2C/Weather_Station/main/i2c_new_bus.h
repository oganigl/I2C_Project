
#include "stdio.h"
#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifndef I2C_BUS_H
#define I2C_BUS_H

#define MODE I2C_MODE_MASTER
#define GPIO_SDA GPIO_NUM_11
#define GPIO_CLK GPIO_NUM_10
#define I2C_MASTER_FREQ_HZ          100000    
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define TIME_OUT                    1000                        //in miliseconds
#define MAX_SLAVES                  5

void i2c_init(void);
#endif