#include <stdio.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mq2.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "ads1115.h"
#include "esp_efuse.h"
#include "esp_adc_cal.h"


#define STACK_SIZE 2048

static const char *TAG = "MQ2";
static esp_adc_cal_characteristics_t adc_chars; 
static float voltage;
adc_cali_handle_t cali_handle;
// payload_t payload;


static void battery_setup()
{
    
    esp_err_t err;

    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");

    // Configure calibration parameters
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_WIDTH_BIT_12,
    };

    // Create a new calibration scheme
    err = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to create calibration scheme: %s", esp_err_to_name(err));
        return;
    }
    
    err = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (err)
    {

        ESP_LOGE(TAG,"The calibration mode is not supported in eFuse: %s",esp_err_to_name(err));
        //adc_cali_handle_t cali_handle = adc_cali_create(ADC_UNIT_1, ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_DEFAULT_VREF, "DEFAULT_CALIBRATION");
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11 , ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_DEFAULT_VREF, &adc_chars);
    }
    else
    {
        //adc_cali_handle_t cali_handle = adc_cali_create(ADC_UNIT_1, ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_EFUSE_VREF, "DEFAULT_CALIBRATION");
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
        //esp_adc_cal_raw_to_voltage(adc_chars, adc1_get_raw(ADC1_CHANNEL_6), &adc_chars);
        //adc_cali_handle_t cali_handle = adc_cali_create(ESP_ADC_CAL_VAL_EFUSE_VREF);
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc1_get_raw(ADC1_CHANNEL_6), (int*)&voltage));

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




