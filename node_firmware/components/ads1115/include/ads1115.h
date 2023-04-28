#ifdef __cplusplus
extern "C" {
#endif

#ifndef ADS1115_H
#define ADS1115_H

#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"



typedef struct{
    float sensor_data1;
    float sensor_data2;
    float sensor_data3;
    float sensor_data4;
    float sensor_data5;
    float sensor_data6;
    float sensor_data7;
    float sensor_data8;

} payload_t;



extern payload_t payload;

typedef enum { 
  ADS1115_CONVERSION_REGISTER_ADDR = 0,
  ADS1115_CONFIG_REGISTER_ADDR,
  ADS1115_LO_THRESH_REGISTER_ADDR,
  ADS1115_HI_THRESH_REGISTER_ADDR,
  ADS1115_MAX_REGISTER_ADDR
} ads1115_register_addresses_t;

typedef enum { 
  ADS1115_MUX_0_1 = 0,
  ADS1115_MUX_0_3,
  ADS1115_MUX_1_3,
  ADS1115_MUX_2_3,
  ADS1115_MUX_0_GND,
  ADS1115_MUX_1_GND,
  ADS1115_MUX_2_GND,
  ADS1115_MUX_3_GND,
} ads1115_mux_t;

typedef enum { 
  ADS1115_FSR_6_144 = 0,
  ADS1115_FSR_4_096,
  ADS1115_FSR_2_048,
  ADS1115_FSR_1_024,
  ADS1115_FSR_0_512,
  ADS1115_FSR_0_256,
} ads1115_fsr_t;

typedef enum { 
  ADS1115_SPS_8 = 0,
  ADS1115_SPS_16,
  ADS1115_SPS_32,
  ADS1115_SPS_64,
  ADS1115_SPS_128,
  ADS1115_SPS_250,
  ADS1115_SPS_475,
  ADS1115_SPS_860
} ads1115_sps_t;

typedef enum {
  ADS1115_MODE_CONTINUOUS = 0,
  ADS1115_MODE_SINGLE
} ads1115_mode_t;

typedef union { // configuration register
  struct {
    uint16_t COMP_QUE:2;  
    uint16_t COMP_LAT:1;  
    uint16_t COMP_POL:1;  
    uint16_t COMP_MODE:1; 
    uint16_t DR:3;        
    uint16_t MODE:1;      
    uint16_t PGA:3;       
    uint16_t MUX:3;       
    uint16_t OS:1;        
  } bit;
  uint16_t reg;
} ADS1115_CONFIG_REGISTER_Type;

typedef struct {
  bool in_use; 
  gpio_num_t pin; 
  QueueHandle_t gpio_evt_queue; 
} ads1115_rdy_pin_t;

typedef struct {
  ADS1115_CONFIG_REGISTER_Type config;
  i2c_port_t i2c_port;
  int address;
  ads1115_rdy_pin_t rdy_pin;
  ads1115_register_addresses_t last_reg; 
  bool changed; 
  TickType_t max_ticks; 
} ads1115_t;

// initialize device
ads1115_t ads1115_config(i2c_port_t i2c_port, uint8_t address); // set up configuration

// set configuration
void ads1115_set_rdy_pin(ads1115_t* ads, gpio_num_t gpio); // set up data-ready pin
void ads1115_set_mux(ads1115_t* ads, ads1115_mux_t mux); // set multiplexer
void ads1115_set_pga(ads1115_t* ads, ads1115_fsr_t fsr); // set fsr
void ads1115_set_mode(ads1115_t* ads, ads1115_mode_t mode); // set read mode
void ads1115_set_sps(ads1115_t* ads, ads1115_sps_t sps); // set sampling speed
void ads1115_set_max_ticks(ads1115_t* ads, TickType_t max_ticks); // maximum wait ticks for i2c bus

int16_t ads1115_get_raw(ads1115_t* ads); // get voltage in bits
double ads1115_get_voltage(ads1115_t* ads); // get voltage in volts
void adc_init(void);


#endif // ifdef ADS1115_H

#ifdef __cplusplus
}
#endif
