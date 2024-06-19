#include "Oled.h"

static const char *TAG = "example";

#define I2C_HOST  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8
#define THERMOMETER_WIDTH   20
#define THERMOMETER_HEIGHT  60

portMUX_TYPE mutex_temperatura = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mutex_humedad = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mutex_luz = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mutex_heading = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mutex_presion = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mutex_modo_pantalla = portMUX_INITIALIZER_UNLOCKED;

lv_disp_t *disp;
extern float hum, temp, press, lux_als, heading;
extern uint8_t modo_pantalla;
lv_obj_t *scr, *label, *label2, *label3, *label4, *label5, *label6, *label7, *label8, *label9, *label10;

uint8_t pantalla_mode;

void example_lvgl_demo_ui() {
    static char buffer_hum[25];
    static char buffer_temp[25];
    static char buffer_lux_als[25];
    static char buffer_press[25];
    static char buffer_grados[25];
    static char buffer_temp_unic[25];
    static char buffer_lux_als_unic[25];
    static char buffer_press_unic[25];
    static char buffer_grados_unic[25];
    static char buffer_hum_unic[25]; 
    static char buffer_lux_mensaje[35];
    float humedad, temperatura, iluminacion, presion, brujula;

    portENTER_CRITICAL(&mutex_temperatura);
    temperatura = temp;
    portEXIT_CRITICAL(&mutex_temperatura);

    portENTER_CRITICAL(&mutex_humedad);
    humedad = hum;
    portEXIT_CRITICAL(&mutex_humedad);

    portENTER_CRITICAL(&mutex_luz);
    iluminacion = lux_als;
    portEXIT_CRITICAL(&mutex_luz);

    portENTER_CRITICAL(&mutex_heading);
    brujula = heading;
    portEXIT_CRITICAL(&mutex_heading);

    portENTER_CRITICAL(&mutex_presion);
    presion = press;
    portEXIT_CRITICAL(&mutex_presion);

    portENTER_CRITICAL(&mutex_modo_pantalla);
    pantalla_mode = modo_pantalla;
    portEXIT_CRITICAL(&mutex_modo_pantalla);

    // Usa snprintf para evitar desbordamiento de buffer
    snprintf(buffer_hum, sizeof(buffer_hum), "Hum: %0.2f", humedad);
    snprintf(buffer_temp, sizeof(buffer_temp), "Temp: %0.2f", temperatura);
    snprintf(buffer_lux_als, sizeof(buffer_lux_als), "Lux: %0.2f", iluminacion);
    snprintf(buffer_press, sizeof(buffer_press), "Press: %0.2f", presion);
    snprintf(buffer_grados, sizeof(buffer_grados), "Grad: %0.2f", brujula);
    snprintf(buffer_temp_unic, sizeof(buffer_temp_unic), "Temperatura:\n %0.2f", temperatura);
    snprintf(buffer_hum_unic, sizeof(buffer_hum_unic), "Humedad:\n %0.2f", humedad);
    snprintf(buffer_grados_unic, sizeof(buffer_grados_unic), "Orientacion:\n  %0.2f", brujula);
    snprintf(buffer_press_unic, sizeof(buffer_press_unic), "Presion: \n %0.2f", presion);
    snprintf(buffer_lux_als_unic, sizeof(buffer_lux_als_unic), "Iluminacion:\n %0.2f", iluminacion);
    if(iluminacion <50){
              snprintf(buffer_lux_mensaje, sizeof(buffer_lux_mensaje), "Bueno me voy\n   a dormir");
    }
    else if (iluminacion>=30 && iluminacion <150) {
        snprintf(buffer_lux_mensaje, sizeof(buffer_lux_mensaje), "Empieza a \n   oscurecer");
    } else if (iluminacion >= 150 && iluminacion<700) {
       snprintf(buffer_lux_mensaje, sizeof(buffer_lux_mensaje), "Dia Normalito");
    }
    else if(iluminacion >= 700 && iluminacion<1300){
        snprintf(buffer_lux_mensaje, sizeof(buffer_lux_mensaje), "Ha salido \nLorenzo");
    }
    else {
        snprintf(buffer_lux_mensaje, sizeof(buffer_lux_mensaje), "que me quemo,\ncuanta luz");
    }
    const char* const_hum = buffer_hum;
    const char* const_temp = buffer_temp;
    const char* const_lux_als = buffer_lux_als;
    const char* const_press = buffer_press;
    const char* const_grados = buffer_grados;
    const char* const_temp_unic = buffer_temp_unic;
    const char* const_hum_unic = buffer_hum_unic;
    const char* const_press_unic = buffer_press_unic;
    const char* const_lux_als_unic = buffer_lux_als_unic;
    const char* const_grados_unic = buffer_grados_unic;
    const char* const_lux_mensaje=buffer_lux_mensaje;
    scr = lv_disp_get_scr_act(disp);
    switch (pantalla_mode) {
        case 0:
            label = lv_label_create(scr);
            lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label, const_temp);
            /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
            lv_obj_set_width(label, disp->driver->hor_res);
            lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);

            label2 = lv_label_create(scr);
            lv_label_set_long_mode(label2, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label2, const_hum);
            lv_obj_set_width(label2, disp->driver->hor_res);
            lv_obj_align(label2, LV_ALIGN_TOP_MID, 0, 12);

            label3 = lv_label_create(scr);
            lv_label_set_long_mode(label3, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label3, const_lux_als);
            lv_obj_set_width(label3, disp->driver->hor_res);
            lv_obj_align(label3, LV_ALIGN_TOP_MID, 0, 23);

            label4 = lv_label_create(scr);
            lv_label_set_long_mode(label4, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label4, const_press);
            lv_obj_set_width(label4, disp->driver->hor_res);
            lv_obj_align(label4, LV_ALIGN_TOP_MID, 0, 34);

            label5 = lv_label_create(scr);
            lv_label_set_long_mode(label5, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label5, const_grados);
            lv_obj_set_width(label5, disp->driver->hor_res);
            lv_obj_align(label5, LV_ALIGN_TOP_MID, 0, 45);
            break;
        case 1:
            label6 = lv_label_create(scr);
            lv_label_set_long_mode(label6, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label6, const_temp_unic);
            lv_obj_set_width(label6, disp->driver->hor_res);
            lv_obj_align(label6, LV_ALIGN_TOP_MID, 0, 7);
            break;
        case 2:
            label7 = lv_label_create(scr);
            lv_label_set_long_mode(label7, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label7, const_hum_unic);
            lv_obj_set_width(label7, disp->driver->hor_res);
            lv_obj_align(label7, LV_ALIGN_TOP_MID, 0, 7);
            break;
        case 3:
            label8 = lv_label_create(scr);
            lv_label_set_long_mode(label8, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label8, const_grados_unic);
            lv_obj_set_width(label8, disp->driver->hor_res);
            lv_obj_align(label8, LV_ALIGN_TOP_MID, 0, 7);
            break;
        case 4:
            label9 = lv_label_create(scr);
            lv_label_set_long_mode(label9, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label9, const_lux_als_unic);
            lv_obj_set_width(label9, disp->driver->hor_res);
            lv_obj_align(label9, LV_ALIGN_TOP_MID, 0, 2);
            label2 = lv_label_create(scr);
           lv_label_set_long_mode(label2, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label2, const_lux_mensaje);
            lv_obj_set_width(label2, disp->driver->hor_res);
            lv_obj_align(label2, LV_ALIGN_TOP_MID, 0, 30);
            break;
        case 5:
            label10 = lv_label_create(scr);
            lv_label_set_long_mode(label10, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label10, const_press_unic);
            lv_obj_set_width(label10, disp->driver->hor_res);
            lv_obj_align(label10, LV_ALIGN_TOP_MID, 0, 7);
            break;
    }
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_t *display = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(display);
    return false;
}

void configurar() {
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
}

void imprimir_Oled() {
    example_lvgl_demo_ui();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    lv_obj_clean(lv_scr_act());
}


