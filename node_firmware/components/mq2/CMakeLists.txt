idf_component_register(SRCS "mq2.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       REQUIRES freertos
                       REQUIRES esp_timer
                       REQUIRES ads1115
                       REQUIRES esp_adc
                       REQUIRES efuse
                       REQUIRES actuator
                       )