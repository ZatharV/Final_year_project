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

#define I2C_MASTER_NUM              0
#define I2C_MASTER_SCL_IO           22      
#define I2C_MASTER_SDA_IO           21     
#define I2C_MASTER_NUM              0                          
#define I2C_MASTER_FREQ_HZ          100000                                          
#define I2C_MASTER_TIMEOUT_MS       1000

#define I2C_MASTER_TX_BUF_DISABLE   0                          
#define I2C_MASTER_RX_BUF_DISABLE   0                          

#define ADS1115_SLAVE_ADDR                 0x48    

int16_t value;
int16_t value2;
int16_t value3;
int16_t value4;
// static const char *TAG = "i2c-simple-example";

static const char *TAG = "ADC EXAMPLE";
static esp_adc_cal_characteristics_t adc1_chars;

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void app_main(void)
{
    
    
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

     
    ESP_LOGI(TAG, "I2C successfully");

    // ads1115_t ads2 = ads1115_config(I2C_NUM_0, 0x48);
    
    // ads1115_set_mux(&ads2, ADS1115_MUX_1_GND);
    // ads1115_set_pga(&ads2,  ADS1115_FSR_2_048); 
    // ads1115_set_mode(&ads2, ADS1115_MODE_CONTINUOUS); 
    // ads1115_set_sps(&ads2, ADS1115_SPS_128); 
    // ads1115_set_max_ticks(&ads2, 1000); 
    // ESP_LOGI(TAG, "I2C successfully");

    // uint32_t voltage;
    // esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    // ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    // ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11));
    // while (1) 
    // {
    //     voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_6), &adc1_chars);
    //     ESP_LOGI(TAG, "ADC1_CHANNEL_6: %u mV", (unsigned int)voltage);
    //     vTaskDelay(pdMS_TO_TICKS(5000));

    // }

    double vol0;
    double vol1;
    double vol2;
    double vol3;
    ads1115_t humi = ads1115_config(I2C_NUM_0, 0x48);
    
    ads1115_t ldr = ads1115_config(I2C_NUM_0, 0x48);
    
    ads1115_t pt = ads1115_config(I2C_NUM_0, 0x48);
    
    ads1115_t bat = ads1115_config(I2C_NUM_0, 0x48);
    
    while (1)
    {
        //vTaskDelay(pdMS_TO_TICKS(5000));
        // ads1115_set_mux(&ads, ADS1115_MUX_0_GND);
        //ads1115_t ads = ads1115_config(I2C_NUM_0, 0x48);
        // ads1115_set_mux(&ads, ADS1115_MUX_0_GND);
        // ads1115_set_pga(&ads,  ADS1115_FSR_2_048); 
        // ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS); 
        // ads1115_set_sps(&ads, ADS1115_SPS_128); 
        // ads1115_set_max_ticks(&ads, 1000);
        //value2 = ads1115_get_raw(&ads);
        ads1115_set_mux(&humi, ADS1115_MUX_0_GND);
        vol0 = ads1115_get_voltage(&humi);
        //ESP_LOGI(TAG, "Raw A0  is : %d", value2);
        ESP_LOGI(TAG, "Voltage at A0  is : %f", vol0);
        vTaskDelay(pdMS_TO_TICKS(5000));

        
        ads1115_set_mux(&ldr, ADS1115_MUX_1_GND);
        vol1 = ads1115_get_voltage(&ldr);
        //ESP_LOGI(TAG, "Raw A0  is : %d", value2);
        ESP_LOGI(TAG, "Voltage at A1  is : %f", vol1);
        vTaskDelay(pdMS_TO_TICKS(5000));

        ads1115_set_mux(&pt, ADS1115_MUX_2_GND);
        vol2 = ads1115_get_voltage(&pt);
        //ESP_LOGI(TAG, "Raw A0  is : %d", value2);
        ESP_LOGI(TAG, "Voltage at A2  is : %f", vol2);
        vTaskDelay(pdMS_TO_TICKS(5000));

        ads1115_set_mux(&bat, ADS1115_MUX_3_GND);
        vol3 = ads1115_get_voltage(&bat);
        //ESP_LOGI(TAG, "Raw A0  is : %d", value2);
        ESP_LOGI(TAG, "Voltage at A3  is : %f", vol3);
        vTaskDelay(pdMS_TO_TICKS(5000));


        // ads1115_t ads1 = ads1115_config(I2C_NUM_0, 0x48);
        // ads1115_set_mux(&ads1, ADS1115_MUX_1_GND);
        // ads1115_set_pga(&ads1,  ADS1115_FSR_2_048); 
        // ads1115_set_mode(&ads1, ADS1115_MODE_CONTINUOUS); 
        // ads1115_set_sps(&ads1, ADS1115_SPS_128); 
        // ads1115_set_max_ticks(&ads1, 1000);
        // value = ads1115_get_raw(&ads1);
        // double vol = ads1115_get_voltage(&ads1);
        // ESP_LOGI(TAG, "Voltage at A1  is : %f", vol);
        // ESP_LOGI(TAG, "Raw A1  is : %d", value);

        // vTaskDelay(pdMS_TO_TICKS(5000));
        // ads1115_t ads2 = ads1115_config(I2C_NUM_0, 0x48);
        // ads1115_set_mux(&ads2, ADS1115_MUX_2_GND);
        // ads1115_set_pga(&ads2,  ADS1115_FSR_2_048); 
        // ads1115_set_mode(&ads2, ADS1115_MODE_CONTINUOUS); 
        // ads1115_set_sps(&ads2, ADS1115_SPS_128); 
        // ads1115_set_max_ticks(&ads2, 1000);
        // value3 = ads1115_get_raw(&ads2);
        // double vol3 = ads1115_get_voltage(&ads2);
        // ESP_LOGI(TAG, "Voltage at A2  is : %f", vol3);
        // ESP_LOGI(TAG, "Raw A2  is : %d", value3);

        // vTaskDelay(pdMS_TO_TICKS(5000));
        // ads1115_t ads3 = ads1115_config(I2C_NUM_0, 0x48);
        // ads1115_set_mux(&ads3, ADS1115_MUX_2_GND);
        // ads1115_set_pga(&ads3,  ADS1115_FSR_2_048); 
        // ads1115_set_mode(&ads3, ADS1115_MODE_CONTINUOUS); 
        // ads1115_set_sps(&ads3, ADS1115_SPS_128); 
        // ads1115_set_max_ticks(&ads3, 1000);
        // value4 = ads1115_get_raw(&ads3);
        // double vol4 = ads1115_get_voltage(&ads3);
        // ESP_LOGI(TAG, "Voltage at A3  is : %f", vol4);
        // ESP_LOGI(TAG, "Raw A3  is : %d", value4);
        // double r = (vol * 2.7 / 1.5 - 0.330) *23.1729 + 10;
        // ESP_LOGI(TAG, "Humidity value  is : %f", vol);

       

    }
        

    
    
}
