#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"

#include "mqtt_client.h"
#include "mqtt_c.h"


#include "ads1115.h"

static const char *TAG = "MQTT_TCP";

char sensor_char1[20];
char sensor_char2[20];
char sensor_char3[20];
char sensor_char4[20];
char sensor_char5[20];
char sensor_char6[20];
char sensor_char7[20];
char sensor_char8[20];


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        vTaskDelay(100 / portTICK_PERIOD_MS);

        while (1)
        {
        //esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t) params;
            if(MQTT_EVENT_CONNECTED)
                    {
                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char1, "%g", payload.sensor_data1);
                        esp_mqtt_client_publish(client, "/topic/qos1", sensor_char1, 0, 1, 0);
                        printf("Data sent1: %s", sensor_char1 );

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char2, "%g", payload.sensor_data2);
                        esp_mqtt_client_publish(client, "/topic/humidity", sensor_char2, 0, 1, 0);
                        printf("Data sent2: %s", sensor_char2 );

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char3, "%g", payload.sensor_data3);
                        esp_mqtt_client_publish(client, "/topic/qos1", sensor_char3, 0, 1, 0);
                        printf("Data sent3: %s", sensor_char3);

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char4, "%g", payload.sensor_data4);
                        esp_mqtt_client_publish(client, "/topic/qos1", sensor_char4, 0, 1, 0);
                        printf("Data sent4: %s", sensor_char4);

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char5, "%g", payload.sensor_data5);
                        esp_mqtt_client_publish(client, "/topic/qos1", sensor_char5, 0, 1, 0);
                        printf("Data sent5: %s", sensor_char5);
                        vTaskDelay(100 / portTICK_PERIOD_MS);

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char6, "%g", payload.sensor_data6);
                        esp_mqtt_client_publish(client, "/topic/qos1", sensor_char6, 0, 1, 0);
                        printf("Data sent6: %s", sensor_char6);
                        vTaskDelay(100 / portTICK_PERIOD_MS);

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char7, "%g", payload.sensor_data7);
                        esp_mqtt_client_publish(client, "/topic/qos1", sensor_char7, 0, 1, 0);
                        printf("Data sent7: %s", sensor_char7);
                        vTaskDelay(100 / portTICK_PERIOD_MS);

                        vTaskDelay(500 / portTICK_PERIOD_MS);
                        sprintf(sensor_char8, "%g", payload.sensor_data8);
                        esp_mqtt_client_publish(client, "/topic/qos1", sensor_char8, 0, 1, 0);
                        printf("Data sent8: %s", sensor_char8);
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                        //esp_mqtt_client_subscribe(client, "/topic/qos0", 1);
                    }
        }
        
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


// void uplink_msg(void *params)
// {
//     while (1)
//     {
//         esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t) params;
//         if(MQTT_EVENT_CONNECTED)
//                 {
//                     vTaskDelay(500 / portTICK_PERIOD_MS);
//                     sprintf(sensor_char1, "%g", payload.sensor_data1);
//                     esp_mqtt_client_publish(client, "/topic/qos1", sensor_char1, 0, 1, 0);
//                     printf("Data sent1: %s", sensor_char1 );

//                     vTaskDelay(500 / portTICK_PERIOD_MS);
//                     sprintf(sensor_char2, "%g", payload.sensor_data2);
//                     esp_mqtt_client_publish(client, "/topic/humidity", sensor_char2, 0, 1, 0);
//                     printf("Data sent2: %s", sensor_char2 );

//                     vTaskDelay(500 / portTICK_PERIOD_MS);
//                     sprintf(sensor_char3, "%g", payload.sensor_data3);
//                     esp_mqtt_client_publish(client, "/topic/qos1", sensor_char3, 0, 1, 0);
//                     printf("Data sent3: %s", sensor_char3);

//                     vTaskDelay(500 / portTICK_PERIOD_MS);
//                     sprintf(sensor_char4, "%g", payload.sensor_data4);
//                     esp_mqtt_client_publish(client, "/topic/qos1", sensor_char4, 0, 1, 0);
//                     printf("Data sent4: %s", sensor_char4);

//                     vTaskDelay(500 / portTICK_PERIOD_MS);
//                     sprintf(sensor_char5, "%g", payload.sensor_data5);
//                     esp_mqtt_client_publish(client, "/topic/qos1", sensor_char5, 0, 1, 0);
//                     printf("Data sent4: %s", sensor_char5);
//                     vTaskDelay(10000 / portTICK_PERIOD_MS);


//                     //esp_mqtt_client_subscribe(client, "/topic/qos0", 1);
//                 }
//     }
// }

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.hivemq.com:1883",
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    //xTaskCreate((void*)uplink_msg, "mqtt push msgfunction", 4096, (void*)&client, 1, NULL);
}



void mqtt_init(void)
{
    

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    printf("WIFI was initiated so sending mqtt data...........\n");

    mqtt_app_start();
    
    
}


