
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_vendor.h"

#ifndef OLED_H
#define OLED_H
extern portMUX_TYPE mutex_temperatura;
extern portMUX_TYPE mutex_humedad;
extern portMUX_TYPE mutex_luz;
extern portMUX_TYPE mutex_heading;
extern portMUX_TYPE mutex_presion;
extern portMUX_TYPE mutex_modo_pantalla;
void example_lvgl_demo_ui();
void configurar();
void imprimir_Oled();
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
#endif