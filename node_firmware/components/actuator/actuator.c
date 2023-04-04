#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "actuator.h"

#define LED_PIN 2
#define MOTOR_PIN 4
#define BUZZER 33
static const char *TAG = "ACTUATORS";

static void actuator_setup(void)
{
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(MOTOR_PIN);
    gpio_set_direction(MOTOR_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
}


static void actuator_task(void)
{
    static int ON = 0;
    while(true)
    {
        ON = !ON;
        gpio_set_level(LED_PIN, ON);
        gpio_set_level(MOTOR_PIN, ON);
        gpio_set_level(BUZZER, ON);
        vTaskDelay(1000/ portTICK_PERIOD_MS);
        ESP_LOGI(TAG,"RUNNING MTOR LED BUZZER");

    }
    
}

void actuator_init(void)
{
    actuator_setup();
    xTaskCreate((void*)actuator_task, "run the actuators", 2048, NULL, 2, NULL); 

}



    