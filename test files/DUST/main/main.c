#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "math.h"
#include "esp_timer.h"

#define dust_gpio 32
#define timeout1 2e4
#define pulseIn(pin,status) pulseInX(pin, status, timeout1)

unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_us = 30e6;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;


int s_led_state = 0;



unsigned long pulseInX(uint8_t pin, uint8_t status, unsigned long timeout)
{
    int start_time = esp_timer_get_time();
    while (gpio_get_level(pin) != status)
    {
        if (esp_timer_get_time() > start_time + timeout)
        {
            return timeout;
        }
    }
    start_time = esp_timer_get_time();
    while (gpio_get_level(pin) == status)
    {
        if (esp_timer_get_time() > start_time + timeout)
        {
            return timeout;
        }
    }
    return (esp_timer_get_time() - start_time);
}



static void configure_led(void)
{
    // gpio_reset_pin(dust_gpio);
    // /* Set the GPIO as a push/pull output */
    // gpio_set_direction(dust_gpio, GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(dust_gpio);
    gpio_set_direction(dust_gpio, GPIO_MODE_INPUT);
}

void get_dust(void)
{
    while (1) {
        duration = pulseIn(dust_gpio, 0);
        lowpulseoccupancy += duration;
        endtime = esp_timer_get_time();
        if ((endtime-starttime) > sampletime_us)
        {
            ratio = (lowpulseoccupancy)/((endtime-starttime)*10.0);  // Integer percentage 0=>100
            concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
            printf("lowpulseoccupancy: %ld\tratio: %f\tDSM501A: %f\r\n", lowpulseoccupancy,ratio, concentration);
            lowpulseoccupancy = 0;
            starttime = esp_timer_get_time();
        }   
    }
}
void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    printf("Turning on Dust sensor\n");
    xTaskCreate((void*)get_dust, "dust get voltage task", 4096, NULL, 2, NULL);
    
}