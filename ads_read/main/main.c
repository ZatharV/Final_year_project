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
//////////////////////////////////////////////////////
// int pin = 19;//DSM501A 
// unsigned long duration;
// unsigned long starttime;
// unsigned long endtime;
// unsigned long sampletime_ms = 30000000;
// unsigned long lowpulseoccupancy = 0;
// float ratio = 0;
// float concentration = 0;
// #define DSM_PIN  19

// static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
// {
//     static uint32_t cap_val_begin_of_sample = 0;
//     static uint32_t cap_val_end_of_sample = 0;
//     TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
//     BaseType_t high_task_wakeup = pdFALSE;

//     //calculate the interval in the ISR,
//     //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
//     if (edata->cap_edge == MCPWM_CAP_EDGE_NEG) {
//         // store the timestamp when pos edge is detected
//         cap_val_begin_of_sample = edata->cap_value;
//         cap_val_end_of_sample = cap_val_begin_of_sample;
//     } else {
//         cap_val_end_of_sample = edata->cap_value;
//         uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

//         // notify the task to calculate the distance
//         xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
//     }

//     return high_task_wakeup == pdTRUE;
// }


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
    // ESP_LOGI(TAG, "Install capture timer");
    // mcpwm_cap_timer_handle_t cap_timer = NULL;
    // mcpwm_capture_timer_config_t cap_conf = {
    //     .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    //     .group_id = 0,
    // };
    // ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    // ESP_LOGI(TAG, "Install capture channel");
    // mcpwm_cap_channel_handle_t cap_chan = NULL;
    // mcpwm_capture_channel_config_t cap_ch_conf = {
    //     .gpio_num = DSM_PIN,
    //     .prescale = 1,
    //     // capture on both edge
    //     .flags.neg_edge = true,
    //     .flags.pos_edge = true,
    //     // pull up internally
    //     .flags.pull_up = true,
    // };
    // ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    // ESP_LOGI(TAG, "Register capture callback");
    // TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    // mcpwm_capture_event_callbacks_t cbs = {
    //     .on_cap = hc_sr04_echo_callback,
    // };
    // ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    // ESP_LOGI(TAG, "Enable capture channel");
    // ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    // // ESP_LOGI(TAG, "Configure Trig pin");
    // // gpio_config_t io_conf = {
    // //     .mode = GPIO_MODE_OUTPUT,
    // //     .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    // // };
    // // ESP_ERROR_CHECK(gpio_config(&io_conf));
    // // // drive low by default
    // // ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    // ESP_LOGI(TAG, "Enable and start capture timer");
    // ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    // ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    // uint32_t tof_ticks;
    // starttime = esp_timer_get_time();

    // while (1) {
    //     // trigger the sensor to start a new sample
    //     // gen_trig_output();
    //     // wait for echo done signal
    //     if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
    //         float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
    //         if (pulse_width_us > 35000) {
    //             // out of range
    //             continue;
    //         }
    //         // convert the pulse width into measure distance
    //         float distance = (float) pulse_width_us/1000;
    //         ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);

    //         lowpulseoccupancy += distance;
    //         endtime =  esp_timer_get_time();
    //         if ((endtime-starttime) > sampletime_ms)
    //         {
    //             ratio = (lowpulseoccupancy-endtime+starttime + sampletime_ms)/(sampletime_ms*10.0);  // Integer percentage 0=>100
    //             concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    //             ESP_LOGI(TAG, "Concentration: %f", concentration);
    //             lowpulseoccupancy = 0;
    //             starttime = esp_timer_get_time();
    //         }
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     }
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

