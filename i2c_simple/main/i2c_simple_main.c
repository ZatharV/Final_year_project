#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_cali.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


#define ADC1_CH4 34
static const char *TAG = "Battery Voltage";
static esp_adc_cal_characteristics_t adc_chars; 
static double voltage;
static double battery_voltage;


static void battery_setup()
{
    esp_err_t err;
    //esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t)); //is the characteristics needed to be static? 

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
    // ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    //ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CH4, ADC_ATTEN_DB_11));

}

static void battery_get_voltage_task()
{
    while (1) 
    {
        voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_6), &adc_chars);
        //battery_voltage = (voltage*1538)/562;
		ESP_LOGI(TAG, "Voltage : %f ", voltage);
        //ESP_LOGI(TAG, "battery voltage : %f V", battery_voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

void battery_init()
{
    battery_setup();
    xTaskCreate((void*)battery_get_voltage_task, "battery get voltage task", 2048, NULL, 4, NULL); //WHAT PRIORITY NNEDED
}

void app_main(void){
    battery_init();
}