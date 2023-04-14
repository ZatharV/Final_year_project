idf_component_register(SRCS "mqtt.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver
                       REQUIRES freertos
                       REQUIRES log
                       REQUIRES esp_timer
                       REQUIRES ads1115
                       REQUIRES mqtt
                       REQUIRES lwip
                       REQUIRES esp_netif
                       REQUIRES esp_event
                       REQUIRES nvs_flash
                       REQUIRES esp_system
                       REQUIRES esp_wifi
                       REQUIRES espmqtt
                       REQUIRES mqtt
                       )
