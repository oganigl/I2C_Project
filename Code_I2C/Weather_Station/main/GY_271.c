
#include "GY_271.h"



/* Dirección por defecto del sensor magnético */
#define QMC5883L_ADDR 0x0D

/* Numeros de registro*/
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Valores de bit para el registro STATUS */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Valores de muestreo para el registro CONFIG */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Valores de rango para el registro CONFIG */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Valores de modo para el registro CONFIG */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001

//defino el declination angle que está en mi ubicación, een este caso la UPV

#define DECLINATION_ANGLE_DEGREES 1 // Declination angle en grados
#define DECLINATION_ANGLE_MINUTES 16 // Declination angle en minutos



/* m_PI no me dejaba usarlo y lo defino */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif


//defino las variables que uso para poner los bits crudos
int16_t xbytes, ybytes, zbytes;
int16_t x_offset ,y_offset ,z_offset;
//defino las variables que uso para poner el offset
int16_t x_max = INT16_MIN, x_min = INT16_MAX;
int16_t y_max = INT16_MIN, y_min = INT16_MAX;
int16_t z_max = INT16_MIN, z_min = INT16_MAX;


static esp_err_t is_not_init = 1;
static esp_err_t is_not_ready = 1;

uint8_t addrs_read [1] = {QMC5883L_X_LSB};     //borrar si no se usa


//pongo la variable del heading, dirección a la que apunto
extern float heading;


float read_data_bytes();


//configuramos el chip
static esp_err_t set_config_byte()
{
    esp_err_t err = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //crear handle

    //procedimiento de escritura

    i2c_master_start(cmd); //start
    err += i2c_master_write_byte(cmd, QMC5883L_ADDR << 1 | I2C_MASTER_WRITE, true); //escribo dirreccion del chip con bit escritura
    err += i2c_master_write_byte(cmd, QMC5883L_CONFIG2, true); //envio registro donde voy a escrivir 
    err+=i2c_master_write_byte(cmd, 0x41, true); //escribo 0100 0001 para poner la configuración deseada
    i2c_master_stop(cmd); //hago el stop
    err += i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS); //esto le envía toda la secuencia de antes
    i2c_cmd_link_delete(cmd); //borro handle

    
    // hago un or con los defines y me creo un uint8 que tiene los bits que quiero modificar
    uint8_t controlregister1 = QMC5883L_CONFIG_OS256 | QMC5883L_CONFIG_2GAUSS | QMC5883L_CONFIG_100HZ | QMC5883L_CONFIG_CONT;
    //                         muestreo                sensibilidad             frecuencia              ponerlo en modo continuo



    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();// creamos otro handle
    err +=i2c_master_start(cmd2);// start
    err +=i2c_master_write_byte(cmd2, QMC5883L_ADDR << 1 | I2C_MASTER_WRITE, true);//direccion del sensor
    err +=i2c_master_write_byte(cmd2, QMC5883L_CONFIG, true); //registro donde quiero escribir
    err+=i2c_master_write_byte(cmd2, controlregister1, true);//lo que voy a escribir
    err+=i2c_master_stop(cmd2);// stop
    err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd2, TIME_OUT / portTICK_PERIOD_MS);// envia secuencia
    i2c_cmd_link_delete(cmd2);// borro handle

    return err;
}


//inicialización 

void init_GY_271()
{   
    esp_err_t err=0;
    err=set_config_byte(); //vemos si da error de configuracion

    if(err!=ESP_OK)
    {
        printf("Error en config\n");
        return;
    }
    is_not_init=err;
}

// esta función lee el bit 0 del registro 06, que nos confirma que hay valores nuevos para leer
static uint8_t check_status()
{
    esp_err_t err=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //creo handle
    uint8_t data[1] = {0}; //creo buffer para guardar dato que leo

    //inicio de procedimiento de lectura (que es como hacer un write sin hacer un stop hasta acabar de leer)

    err +=i2c_master_start(cmd); //start
    err +=i2c_master_write_byte(cmd, QMC5883L_ADDR << 1 | I2C_MASTER_WRITE, true);// direccion del sensor
    err +=i2c_master_write_byte(cmd, QMC5883L_STATUS, true); // registro del que quiero leer
    err +=i2c_master_start(cmd); //start

    err +=i2c_master_write_byte(cmd, QMC5883L_ADDR << 1 | I2C_MASTER_READ, true);// direccion del sensor otra vez
    err+= i2c_master_read(cmd, data, 1, I2C_MASTER_LAST_NACK);//leemos del registro y ponemos en data
    err+=i2c_master_stop(cmd);//stop
    err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS);//envio todo
    i2c_cmd_link_delete(cmd);//borro handle

    data[0] = data[0] << 7; //desplazo 7, si el bit leido es cero data es 0, y si es uno, es mayor que 0
    return data[0]; //devolvemos true o false
}

//calculamos el offset
void calculo_offset()
{

    //bucle de 1000 iteraciones donde voy viendo el maximo y el minimo de las muestras cogidas
    
    for (int i = 0; i < 1000; i++)
    {
        read_data_bytes();
        if (xbytes > x_max) x_max = xbytes;
        if (xbytes < x_min) x_min = xbytes;
        if (ybytes > y_max) y_max = ybytes;
        if (ybytes < y_min) y_min = ybytes;
        if (zbytes > z_max) z_max = zbytes;
        if (zbytes < z_min) z_min = zbytes;

         /*printf("%d \n", (int)xbytes);
         printf("%d \n", (int)ybytes);
         printf("%d \n", (int)zbytes);*/
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    //calculo el offset como la media del maximo y el minimo
    x_offset = (x_max + x_min) / 2;
    y_offset = (y_max + y_min) / 2;
    z_offset = (z_max + z_min) / 2;


    printf( "Offset X: %d, Offset Y: %d, Offset Z: %d", x_offset, y_offset, z_offset); 
}

// aqui ya vamos a leer los datos
float read_data_bytes()
{
    esp_err_t err=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); //creamos handle
    uint8_t buffer_read[6] = {0, 0, 0, 0, 0, 0}; //creamos el buffer para la info de x,y,z . 2 bytes para cada una

    if (check_status() > 0) //datos para leer?
    {
        //READ
        err +=i2c_master_start(cmd); //start
        err +=i2c_master_write_byte(cmd, QMC5883L_ADDR << 1 | I2C_MASTER_WRITE, true);// direccion sensor
        err +=i2c_master_write_byte(cmd, QMC5883L_X_LSB, true);//registro leer
        err +=i2c_master_start(cmd);//start

        err +=i2c_master_write_byte(cmd, QMC5883L_ADDR << 1 | I2C_MASTER_READ, true);// direccion sensor
        err+= i2c_master_read(cmd, buffer_read, 6, I2C_MASTER_LAST_NACK); //leo y guardo en buffer
        err+=i2c_master_stop(cmd);//stop
        err+=i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, TIME_OUT / portTICK_PERIOD_MS);// enviamos todo
        i2c_cmd_link_delete(cmd); //borramos handle

    }

    
    //datos leidos ordenamos y ponemos en variables
    xbytes = buffer_read[1] << 8 | buffer_read[0] ;
    ybytes = buffer_read[3] << 8 | buffer_read[2] ;
    zbytes = buffer_read[5] << 8 | buffer_read[4] ;
    
    //le restamos el offset (esto según internet hay que hacerlo para mejor calibración)
    xbytes -= x_offset ;
    ybytes -= y_offset;
    
    
    // tengo que ponerlo en un int, si no no me funciona 
    int fx_bits = xbytes; // Valor en bits para la componente x
    int fy_bits = ybytes; // Valor en bits para la componente y

    // Convertir bits a grados
    double angulo_rad = atan2((double)fy_bits, (double)fx_bits); // Calcular ángulo en radianes
    double angulo_grados = angulo_rad * (180.0 / M_PI); // Convertir a grados

    // Ajustar el ángulo para que esté en el rango de 0 a 360 grados
    if (angulo_grados < 0) {
        angulo_grados += 360.0;
    }
     return angulo_grados;
    
    //printf("%f \n", heading);
 
    return err;
}




