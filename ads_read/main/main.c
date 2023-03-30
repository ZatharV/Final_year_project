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
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include <math.h>
#include "esp_timer.h"


typedef struct 
{
    float sensor_data1;
    float sensor_data2;
    float sensor_data3;
    float sensor_data4;
    float sensor_data5;
    //float sensor_data1;

}payload_t;

payload_t payload;

void app_main(void)
{
    
    adc_init();
    
    
        
    
}

