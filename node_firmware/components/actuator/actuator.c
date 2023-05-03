#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "actuator.h"
#include "ads1115.h"

#define LED_PIN 12
#define MOTOR_PIN 4
#define BUZZER 33
static const char *TAG = "ACTUATORS";

static void actuator_setup(void)
{
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(MOTOR_PIN);
    gpio_set_direction(MOTOR_PIN, GPIO_MODE_OUTPUT);

    esp_rom_gpio_pad_select_gpio(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
}

void blink_led(void)
{
    ESP_LOGI(TAG, "Blinking LED...");
    actuator_setup();
    int ON = 0;
    for (int i=0; i<6 ; i++)
    {
        ON = !ON;
        gpio_set_level(LED_PIN, ON);
        vTaskDelay(500/ portTICK_PERIOD_MS);

    }
}

void buzzer_on(void)
{
    actuator_setup();
    gpio_set_level(BUZZER, 1);
    ESP_LOGI(TAG, "BUZZER IS ON");
}

void buzzer_off(void)
{
    actuator_setup();
    gpio_set_level(BUZZER, 0);
    ESP_LOGI(TAG, "BUZZER IS OFF");
}

void motor_on(void)
{
    actuator_setup();
    gpio_set_level(MOTOR_PIN, 1);
    ESP_LOGI(TAG, "MOTOR IS ON");
}    

void motor_off(void)
{
    actuator_setup();
    gpio_set_level(MOTOR_PIN, 0);
    ESP_LOGI(TAG, "MOTOR IS OFF");
}
