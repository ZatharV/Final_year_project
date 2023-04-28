#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/pcnt.h"
#include <math.h>
#include "esp_timer.h"

#define PIN 34 // DSM501A input D8
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      1000
#define PCNT_THRESH1_VAL    500
#define PCNT_THRESH0_VAL   -500
#define PCNT_INPUT_SIG_IO   PIN

static const char *TAG = "DSM501A";

unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 30000;
int16_t lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

void pcnt_init()
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = 0,
        .unit = PCNT_TEST_UNIT,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);
    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

void app_main()
{
    pcnt_init();
    ESP_LOGI(TAG, "DSM501A started");

    while(1) {
        pcnt_get_counter_value(PCNT_TEST_UNIT, &lowpulseoccupancy);
        pcnt_counter_clear(PCNT_TEST_UNIT);

        endtime = xTaskGetTickCount();
        if ((endtime-starttime) > pdMS_TO_TICKS(sampletime_ms))
        {
            ratio = (lowpulseoccupancy + sampletime_ms) / (sampletime_ms * 10.0);  // Integer percentage 0=>100
            concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using
            ESP_LOGI(TAG, "lowpulseoccupancy: %d, ratio: %f, DSM501A: %f", lowpulseoccupancy, ratio, concentration);
        lowpulseoccupancy = 0;
        starttime = xTaskGetTickCount();
        }
    vTaskDelay(pdMS_TO_TICKS(100)); // add some delay to reduce CPU usage
    }
}
