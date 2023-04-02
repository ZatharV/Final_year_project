#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mq2.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "ads1115.h"


#define STACK_SIZE 2048

static const char *TAG = "MQ2";
static esp_adc_cal_characteristics_t adc_chars; 
static float voltage;
// payload_t payload;


static void battery_setup()
{
    esp_err_t err;
    err = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (err)
    {
        ESP_LOGE(TAG,"The calibration mode is not supported in eFuse: %s",esp_err_to_name(err));
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11 , ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_DEFAULT_VREF, &adc_chars);
    }
    else
    {
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11 , ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_EFUSE_VREF, &adc_chars);
		ESP_LOGE(TAG,"The calibration mode is supported in eFuse ");
    }
	ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
	ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11));
}

static void battery_get_voltage_task(void* pvParameters)
{
    while (1) 
    {
        voltage = (float)esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_6), &adc_chars);
        ((payload_t*)pvParameters)->sensor_data5=voltage;
        ESP_LOGI(TAG,"voltage from mq2 is %.2f", payload.sensor_data5);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void battery_init(void)
{
    battery_setup();
    xTaskCreate((void*)battery_get_voltage_task, "battery get voltage task", STACK_SIZE, (void*)&payload, 2, NULL); 
}




