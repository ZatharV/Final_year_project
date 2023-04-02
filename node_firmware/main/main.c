#include <stdio.h>
#include <stdlib.h>
#include "ads1115.h"
#include "esp_log.h"
#include "mq2.h"
#include "actuator.h"




void app_main(void)
{
    
    adc_init();
    battery_init();
    actuator_init();     

    
    
}

