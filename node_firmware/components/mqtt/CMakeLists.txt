idf_component_register(SRCS "mqtt_c.c"
                       INCLUDE_DIRS "include"
                                    "D:/Espressif/frameworks/esp-idf-v5.0.1/components/mqtt/esp-mqtt/include"
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
                       REQUIRES actuator
                      )
