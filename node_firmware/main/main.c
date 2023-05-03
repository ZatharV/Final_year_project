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


static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing wifi");
    blink_led();
    wifi_station_init();
    blink_led();
    if (MY_FLAG)
    {
        blink_led();
        printf("Connected now wifi initializing sensors... ");

        adc_init();
        battery_init();
        mqtt_init();
    }

    ESP_LOGE(TAG, "Failed to connect to wifi maximum tries exceeded! ");
}

