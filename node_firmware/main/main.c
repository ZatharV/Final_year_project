#include <stdio.h>
#include <stdlib.h>
#include "ads1115.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "esp_adc/adc_continuous.h"
// #include "esp_adc/adc_cali.h"
// #include "esp_adc/adc_cali_scheme.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_private/esp_clk.h"
#include "driver/gpio.h"
#include <math.h>
#include "esp_timer.h"
#include "mq2.h"
#include "actuator.h"


// typedef struct 
// {
//     float sensor_data1;
//     float sensor_data2;
//     float sensor_data3;
//     float sensor_data4;
//     float sensor_data5;
//     //float sensor_data1;

// }payload_t;
// //static const char* TAG = "MyModule";

// payload_t payload;

// In main.c
// In ads1115.h



// In main.c
// payload_t payload;


void app_main(void)
{
    
    adc_init();
    battery_init();
    actuator_init();     

    // ESP_LOGI(TAG, "The ads sensor value 1 is %f", payload.sensor_data1);
    // ESP_LOGI(TAG, "The ads sensor value 2 is %f", payload.sensor_data1);
    // ESP_LOGI(TAG, "The ads sensor value 3 is %f", payload.sensor_data1);
    // ESP_LOGI(TAG, "The ads sensor value 3 is %f", payload.sensor_data1);
    
}

