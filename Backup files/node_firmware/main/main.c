#include <stdio.h>
#include <stdlib.h>
#include "ads1115.h"
#include "esp_log.h"
#include "mq2.h"
#include "actuator.h"
#include "mqtt_c.h"
#include "wifi_station.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"




void app_main(void)
{
    wifi_station_init();
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    adc_init();
    battery_init();
    actuator_init(); 
    mqtt_init();  


    
    
}

