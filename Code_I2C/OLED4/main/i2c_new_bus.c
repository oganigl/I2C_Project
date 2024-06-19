#include "i2c_new_bus.h"

void i2c_init()
{
    i2c_config_t config = {
        .mode = MODE,
        .sda_io_num = GPIO_SDA,
        .scl_io_num = GPIO_CLK,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        
        #if MODE ==  I2C_MODE_MASTER
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        #else
        config.slave.addr_10bit_en = ;
        config.slave.slave_addr = ;
        config.slave.maximun_speed = ;
        #endif
    };

    i2c_param_config(I2C_MASTER_NUM, &config);
    i2c_driver_install(I2C_MASTER_NUM, config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
