#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "i2c_new_bus.h"
#include "Luz_ambiental.h"
#include "BMP280.h"
#include "AHT20.h"
#include "Oled.h"
#include "GY_271.h"
#include "Bluetooth.h"


float lux_als,press,hum,temp, heading;
uint8_t modo_pantalla;
void oled(void *pvParameters)
{
    for(;;)
    {
        imprimir_Oled();
    }
    
}
void task_veml7700_read(void *ignore)
{ 
	esp_err_t init_result = config_veml7700();
	float lux;
	if (init_result != ESP_OK) 
    {
		printf("Failed to initialize. Result: %d\n", init_result);
		return;
	}
	for(;;) {
		// Read the ALS data
		ESP_ERROR_CHECK(veml7700_read_als_lux(&lux));
		portENTER_CRITICAL(&mutex_luz);
		lux_als=lux;
		portEXIT_CRITICAL(&mutex_luz);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void presion()
{
    float presion;
    for(;;)
    {
        get_BMP280_packet();
        get_pressure(&presion);

        portENTER_CRITICAL(&mutex_presion);
        press = presion;
		portEXIT_CRITICAL(&mutex_presion);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}

void temp_hum(void *pvParameters)
{
    float temperatura, humedad;
    for(;;)
    {
        
        get_AHT20_packet();
        get_temperature_humidity(&temperatura, &humedad);

        portENTER_CRITICAL(&mutex_temperatura);
		temp = temperatura;
        portEXIT_CRITICAL(&mutex_temperatura);

        portENTER_CRITICAL(&mutex_humedad);
        hum = humedad;
        portEXIT_CRITICAL(&mutex_humedad);
        
       vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void brujula(void *pvParameters)
{
    float valor_brujula;
    calculo_offset(); //calcula el offset de la brujula
    for(;;)
    {
        valor_brujula=read_data_bytes();//lee la brujula
    portENTER_CRITICAL(&mutex_heading);
     heading=valor_brujula;
    portEXIT_CRITICAL(&mutex_heading);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void app_main(void)
{
    modo_pantalla=0; 
    iniciar_bluetooth();
    i2c_init();
   
    init_GY_271(); //inicia brujula, tarda un ratillo en calibrar

   init_BMP280();
    configurar();
    esp_err_t error = config_veml7700();
   xTaskCreate(&task_veml7700_read, "task_read_veml7700",  2048, NULL, 2, NULL);
   xTaskCreatePinnedToCore(&oled,"oled",4096,NULL,2,NULL, 1);
    
    xTaskCreate(&temp_hum,"temp_hum",2048,NULL,2,NULL);
   xTaskCreate(&presion,"presion",2048,NULL,2,NULL);
    xTaskCreate(&brujula,"brujula",2048,NULL,2,NULL);

}