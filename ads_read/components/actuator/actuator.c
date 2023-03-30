#include "ads1115.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define LED_PIN 2
#define MOTOR_PIN 4
#define BUZZER 33

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

    