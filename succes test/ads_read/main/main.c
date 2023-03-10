#include <stdio.h>
#include "ads1115.h"

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
static const char *TAG = "i2c-simple-example";

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

    ads1115_t ads = ads1115_config(I2C_NUM_0, 0x48);
    
    ads1115_set_mux(&ads, ADS1115_MUX_0_GND);
    ads1115_set_pga(&ads,  ADS1115_FSR_2_048); 
    ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS); 
    ads1115_set_sps(&ads, ADS1115_SPS_128); 
    ads1115_set_max_ticks(&ads, 1000); 


    ESP_LOGI(TAG, "I2C successfully");

    while (1)
    {
        value = ads1115_get_raw(&ads);
        ESP_LOGI(TAG, "count  is : %d", value);
        double vol = ads1115_get_voltage(&ads);
        double r = (vol * 2.7 / 1.5 - 0.330) *23.1729 + 10;
        ESP_LOGI(TAG, "Humidity value  is : %f", vol);

    }
        

    
    
}
