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




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define LED_PIN 2
#define MOTOR_PIN 4
#define BUZZER 33

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
// static const char *TAG = "i2c-simple-example";

static const char *TAG = "ADC EXAMPLE";
static esp_adc_cal_characteristics_t adc1_chars;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(MOTOR_PIN);
    gpio_set_direction(MOTOR_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    int ON = 0;
    while(true)
    {
        ON = !ON;
        gpio_set_level(LED_PIN, ON);
        gpio_set_level(MOTOR_PIN, ON);
        gpio_set_level(BUZZER, ON);
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    uint32_t voltage;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11));
    while (1) 
    {
        voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_6), &adc1_chars);
        ESP_LOGI(TAG, "ADC1_CHANNEL_6: %u mV", (unsigned int)voltage);
        vTaskDelay(pdMS_TO_TICKS(5000));

    }

    //////////////////////////////////////////////////////////////////
    
        

    
    
}

