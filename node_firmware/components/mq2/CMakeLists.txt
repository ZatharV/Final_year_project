idf_component_register(SRCS "mq2.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       REQUIRES freertos
                       REQUIRES log
                       REQUIRES esp_timer
                       REQUIRES esp_adc
                       
                       )
