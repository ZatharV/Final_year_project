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
double r;

///////////////////////////////////////////////////////////
// static const char *TAG = "i2c-simple-example";

static const char *TAG = "ADC EXAMPLE";
static esp_adc_cal_characteristics_t adc1_chars;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
    
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ads1115_t ads = ads1115_config(I2C_NUM_0, 0x48);
    
    ads1115_set_mux(&ads, ADS1115_MUX_0_GND);
    ads1115_set_pga(&ads,  ADS1115_FSR_2_048); 
    ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS); 
    ads1115_set_sps(&ads, ADS1115_SPS_128); 
    ads1115_set_max_ticks(&ads, 1000); 
    ESP_LOGI(TAG, "I2C successfully");
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
    while (1)
    {
        value = ads1115_get_raw(&ads);
        ESP_LOGI(TAG, "count  is : %d", value);
        double vol = ads1115_get_voltage(&ads);
        r = (vol * 2.7 / 1.5 - 0.330) *23.1729 + 10;
        ESP_LOGI(TAG, "Humidity value  is : %f", vol);

    }
        

    
    
}

