#include "ads1115.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "actuator.h"
#include "math.h"

#define I2C_MASTER_NUM              0
#define I2C_MASTER_SCL_IO           22      
#define I2C_MASTER_SDA_IO           21     
#define I2C_MASTER_NUM              0                          
#define I2C_MASTER_FREQ_HZ          100000                                          
#define I2C_MASTER_TIMEOUT_MS       1000

#define I2C_MASTER_TX_BUF_DISABLE   0                          
#define I2C_MASTER_RX_BUF_DISABLE   0                          

#define ADS1115_SLAVE_ADDR                 0x48    
#define DUMMY 23
#define LUX_CALC_SCALAR           12518931
#define LUX_CALC_EXPONENT         -1.405

float r0;
float r1;
float r2;
float r3;
int res;


static const char* TAG = "ADS1115";

payload_t payload;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  const bool ret = 1; 
  QueueHandle_t gpio_evt_queue = (QueueHandle_t)arg; 
  xQueueSendFromISR(gpio_evt_queue, &ret, NULL);
}

static esp_err_t ads1115_write_register(ads1115_t* ads, ads1115_register_addresses_t reg, uint16_t data) {
  i2c_cmd_handle_t cmd;
  esp_err_t ret;
  uint8_t out[2];

  out[0] = data >> 8; 
  out[1] = data & 0xFF; 
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); 
  i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_WRITE,1); 
  i2c_master_write_byte(cmd,reg,1); 
  i2c_master_write(cmd,out,2,1); 
  i2c_master_stop(cmd); 
  ret = i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks); 
  i2c_cmd_link_delete(cmd);
  ads->last_reg = reg; 
  return ret;
}

static esp_err_t ads1115_read_register(ads1115_t* ads, ads1115_register_addresses_t reg, uint8_t* data, uint8_t len) {
  i2c_cmd_handle_t cmd;
  esp_err_t ret;

  if(ads->last_reg != reg) {
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_WRITE,1);
    i2c_master_write_byte(cmd,reg,1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks);
    i2c_cmd_link_delete(cmd);
    ads->last_reg = reg;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); 
  i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_READ,1);
  i2c_master_read(cmd, data, len, 0); 
  i2c_master_stop(cmd); 
  ret = i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks); 
  i2c_cmd_link_delete(cmd);
  return ret;
}

ads1115_t ads1115_config(i2c_port_t i2c_port, uint8_t address) {
  ads1115_t ads; // setup configuration with default values
  ads.config.bit.OS = 1; // always start conversion
  ads.config.bit.MUX = ADS1115_MUX_0_GND;
  ads.config.bit.PGA = ADS1115_FSR_2_048;
  ads.config.bit.MODE = ADS1115_MODE_CONTINUOUS;
  ads.config.bit.DR = ADS1115_SPS_860;
  ads.config.bit.COMP_MODE = 0;
  ads.config.bit.COMP_POL = 0;
  ads.config.bit.COMP_LAT = 0;
  ads.config.bit.COMP_QUE = 0b11;

  ads.i2c_port = i2c_port; // save i2c port
  ads.address = address; // save i2c address
  ads.rdy_pin.in_use = 0; // state that rdy_pin not used
  ads.last_reg = ADS1115_MAX_REGISTER_ADDR; // say that we accessed invalid register last
  ads.changed = 1; // say we changed the configuration
  ads.max_ticks = 10/portTICK_PERIOD_MS;
  return ads; // return the completed configuration
}


void ads1115_set_mux(ads1115_t* ads, ads1115_mux_t mux) {
  ads->config.bit.MUX = mux;
  ads->changed = 1;
}

void ads1115_set_rdy_pin(ads1115_t* ads, gpio_num_t gpio) {
  const static char* TAG = "ads1115_set_rdy_pin";
  gpio_config_t io_conf;
  esp_err_t err;

  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask = 1<<gpio;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  io_conf.pull_down_en = 0;
  gpio_config(&io_conf); 

  ads->rdy_pin.gpio_evt_queue = xQueueCreate(1, sizeof(bool));
  gpio_install_isr_service(0);

  ads->rdy_pin.in_use = 1;
  ads->rdy_pin.pin = gpio;
  ads->config.bit.COMP_QUE = 0b00; 
  ads->changed = 1;

  err = ads1115_write_register(ads, ADS1115_LO_THRESH_REGISTER_ADDR,0); 
  if(err) ESP_LOGE(TAG,"could not set low threshold: %s",esp_err_to_name(err));
  err = ads1115_write_register(ads, ADS1115_HI_THRESH_REGISTER_ADDR,0xFFFF); 
  if(err) ESP_LOGE(TAG,"could not set high threshold: %s",esp_err_to_name(err));
}

void ads1115_set_pga(ads1115_t* ads, ads1115_fsr_t fsr) {
  ads->config.bit.PGA = fsr;
  ads->changed = 1;
}

void ads1115_set_mode(ads1115_t* ads, ads1115_mode_t mode) {
  ads->config.bit.MODE = mode;
  ads->changed = 1;
}

void ads1115_set_sps(ads1115_t* ads, ads1115_sps_t sps) {
  ads->config.bit.DR = sps;
  ads->changed = 1;
}

void ads1115_set_max_ticks(ads1115_t* ads, TickType_t max_ticks) {
  ads->max_ticks = max_ticks;
}

int16_t ads1115_get_raw(ads1115_t* ads) {
  const static char* TAG = "ads1115_get_raw";
  const static uint16_t sps[] = {8,16,32,64,128,250,475,860};
  const static uint8_t len = 2;
  uint8_t data[2];
  esp_err_t err;
  bool tmp; 

  if(ads->rdy_pin.in_use) {
    gpio_isr_handler_add(ads->rdy_pin.pin, gpio_isr_handler, (void*)ads->rdy_pin.gpio_evt_queue);
    xQueueReset((QueueHandle_t)ads->rdy_pin.gpio_evt_queue);
  }
  
  if((ads->config.bit.MODE==ADS1115_MODE_SINGLE) || (ads->changed)) { 
    err = ads1115_write_register(ads, ADS1115_CONFIG_REGISTER_ADDR, ads->config.reg);
    if(err) {
      ESP_LOGE(TAG,"could not write to device: %s",esp_err_to_name(err));
      if(ads->rdy_pin.in_use) {
        gpio_isr_handler_remove(ads->rdy_pin.pin);
        xQueueReset((QueueHandle_t)ads->rdy_pin.gpio_evt_queue);
      }
      return 0;
    }
    ads->changed = 0; 
  }

  if(ads->rdy_pin.in_use) {
    xQueueReceive((QueueHandle_t)ads->rdy_pin.gpio_evt_queue, &tmp, portMAX_DELAY);
    gpio_isr_handler_remove(ads->rdy_pin.pin);
  }
  else {
    
    vTaskDelay((((1000/sps[ads->config.bit.DR]) + 1) / portTICK_PERIOD_MS)+1);
  }

  err = ads1115_read_register(ads, ADS1115_CONVERSION_REGISTER_ADDR, data, len);
  if(err) {
    ESP_LOGE(TAG,"could not read from device: %s",esp_err_to_name(err));
    return 0;
  }
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

double ads1115_get_voltage(ads1115_t* ads) {
  const double fsr[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
  const int16_t bits = (1L<<15)-1;
  int16_t raw;

  raw = ads1115_get_raw(ads);
  return (double)raw * fsr[ads->config.bit.PGA] / (double)bits;
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void adc_log(void* pvParameters)
{
  // esp_rom_gpio_pad_select_gpio(DUMMY);
  // gpio_set_direction(DUMMY, GPIO_MODE_INPUT);
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized successfully");
  ESP_LOGI(TAG, "Initalization structures completed");

  bool myflag1 = true;
  bool myflag2 = true;
  double vol0;
  double vol1;
  double vol2;
  double vol3;
  
 
  while (1)
    {
        // 0 channel-hcpv, 1channel -ldr, 2 pt100, a3- battery level
        ads1115_t humi = ads1115_config(I2C_NUM_0, 0x48);
        ads1115_t ldr = ads1115_config(I2C_NUM_0, 0x48);
        ads1115_t pt = ads1115_config(I2C_NUM_0, 0x48);
        ads1115_t bat = ads1115_config(I2C_NUM_0, 0x48);
  
        vol0 = ads1115_get_voltage(&humi);
        ESP_LOGI(TAG, "Voltage at A0  is : %f", vol0);
        r0 = (vol0 * 2.7 / 1.5 - 0.330) *23.1729 + 10;
        ((payload_t*)pvParameters)->sensor_data2 = r0;
        ESP_LOGI(TAG, "Humidity value  is : %f", r0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        

        ads1115_set_mux(&ldr, ADS1115_MUX_1_GND);
        vol1 = ads1115_get_voltage(&ldr);
        ESP_LOGI(TAG, "Voltage at A1  is : %f", vol1);
        ((payload_t*)pvParameters)->sensor_data4 = r1;
        res = vol1*5e4/(10-5*vol1);
        r1 = LUX_CALC_SCALAR * pow(res, LUX_CALC_EXPONENT);
        ESP_LOGI(TAG, "Light intensity value inside factory is : %f", r1);

        vTaskDelay(pdMS_TO_TICKS(1000));

        
        ads1115_set_mux(&pt, ADS1115_MUX_2_GND);
        vol2 = ads1115_get_voltage(&pt);
        ESP_LOGI(TAG, "Voltage at A2  is : %f", vol2);
        r2 = (329.537 * vol2 - 186.935)/ 385 * 100;
        ((payload_t*)pvParameters)->sensor_data1 = r2;
        ESP_LOGI(TAG, "Temperature value  is : %f", r2);
        vTaskDelay(pdMS_TO_TICKS(1000));
         if (r2 > 80)
        {
          myflag2 = true;
          if (myflag1 == true)
          {
            buzzer_on();
            myflag1 = false;
          }
          
        
        }
        else
        {
          myflag1 = true;
          if (myflag2 == true)
          {
            buzzer_off();
            myflag2 = false;
          } 
        }

        
        ads1115_set_mux(&bat, ADS1115_MUX_3_GND);
        vol3 = ads1115_get_voltage(&bat);
        ESP_LOGI(TAG, "Voltage at A3  is : %f", vol3);
        // r3 = ((((133.21 * vol3) + 1491.05) / 32830.21) * 241);
        r3 = ((12.47-11.04)/(1.906-0.402))*(vol3 - 0.402) + 11.04;
        ((payload_t*)pvParameters)->sensor_data3 = r3;
        ESP_LOGI(TAG, "Battery voltage value  is : %f", r3);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

void adc_init(void)
{
    xTaskCreate((void*)adc_log, "ads1115 get voltage task", 4096, (void*)&payload, 2, NULL); 
}