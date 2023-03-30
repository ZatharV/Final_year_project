#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
//#include "esp_adc/adc_cali.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mq2.h"


#define STACK_SIZE 2048

static const char *TAG = "Bat";
static esp_adc_cal_characteristics_t adc_chars; 
static float voltage;
static float battery_voltage;


static void battery_setup()
{
    esp_err_t err;
    err = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (err)
    {
        ESP_LOGE(TAG,"The calibration mode is not supported in eFuse: %s",esp_err_to_name(err));
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6 , ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_DEFAULT_VREF, &adc_chars);
    }
    else
    {
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6 , ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_EFUSE_VREF, &adc_chars);
		ESP_LOGE(TAG,"The calibration mode is supported in eFuse ");
    }
	ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
	ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_6));
}

static void battery_get_voltage_task()
{
    while (1) 
    {
        voltage = (float)esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_6), &adc_chars);
        battery_voltage = (float)((voltage * SCALING_FACTOR) / 1000.0f);
        ESP_LOGI(TAG,"bat is %.2f FB is %.2f", battery_voltage, voltage);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void battery_init(void)
{
    battery_setup();
    xTaskCreate((void*)battery_get_voltage_task, "battery get voltage task", STACK_SIZE, NULL, 2, NULL); 
}




