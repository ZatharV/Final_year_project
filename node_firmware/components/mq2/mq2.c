#include <stdio.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mq2.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "ads1115.h"
#include "esp_efuse.h"
#include "esp_adc_cal.h"
#include <math.h>
#include "actuator.h"


#define STACK_SIZE 2048

static const char *TAG = "MQ2";
static esp_adc_cal_characteristics_t adc_chars; 
static float voltage;
adc_cali_handle_t cali_handle;


#define         MQ_PIN                       ADC1_CHANNEL_6     
#define         RL_VALUE                     (5)     
#define         RO_CLEAN_AIR_FACTOR          (9.83)                                                       
#define         CALIBARAION_SAMPLE_TIMES     (50)   
#define         CALIBRATION_SAMPLE_INTERVAL  (500)                                                        
#define         READ_SAMPLE_INTERVAL         (50)    
#define         READ_SAMPLE_TIMES            (5)                                                         
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

float LPGCurve[3] = {2.3,0.21,-0.47};                                                                                                                                                          
float COCurve[3] = {2.3,0.72,-0.34};                                                                                                                                                                
float SmokeCurve[3] = {2.3,0.53,-0.44};    
                                                    
                                                    
                                                                                                       
float Ro = 10;   

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            
    val += MQResistanceCalculation(adc1_get_raw(ADC1_CHANNEL_6));
    vTaskDelay(pdMS_TO_TICKS(CALIBRATION_SAMPLE_INTERVAL));
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   

  val = val/RO_CLEAN_AIR_FACTOR;                       
                                                         

  return val; 
}


float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(adc1_get_raw(ADC1_CHANNEL_6));
    vTaskDelay(pdMS_TO_TICKS(READ_SAMPLE_INTERVAL));
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}


int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}





///////////////////////////////////////////////////////////////////////////////////////////////
static void battery_setup()
{
    
    esp_err_t err;

    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");

   
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_WIDTH_BIT_12,
    };

    
    err = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to create calibration scheme: %s", esp_err_to_name(err));
        return;
    }
    
    err = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (err)
    {

        ESP_LOGE(TAG,"The calibration mode is not supported in eFuse: %s",esp_err_to_name(err));
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11 , ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_DEFAULT_VREF, &adc_chars);
    }
    else
    {
        
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11 , ADC_WIDTH_BIT_12, ESP_ADC_CAL_VAL_EFUSE_VREF, &adc_chars);
		ESP_LOGE(TAG,"The calibration mode is supported in eFuse ");
    }
	ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
	ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11));

  Ro = MQCalibration(MQ_PIN);
}

static void battery_get_voltage_task(void* pvParameters)
{
    bool myflag1 = true;
    bool myflag2 = true;
    while (1) 
    {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, adc1_get_raw(ADC1_CHANNEL_6), (int*)&voltage));

        int16_t lpg = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
        int16_t co = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
        int16_t smoke =  MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
        ESP_LOGI(TAG,"lpg in ppm: %d\n", MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG));
        ESP_LOGI(TAG,"CO in ppm: %d\n", MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO));
        ESP_LOGI(TAG,"gas smoke in ppm:  %d\n", MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE));
        ESP_LOGI(TAG,"Ananlog RAW READING FROM MQ2 : %d\n",adc1_get_raw(ADC1_CHANNEL_6));
        ((payload_t*)pvParameters)->sensor_data5=(float)lpg;
        ((payload_t*)pvParameters)->sensor_data6=(float)co;
        ((payload_t*)pvParameters)->sensor_data7=(float)smoke;
        if (smoke > 100)
        {
          myflag2 = true;
          if (myflag1 == true)
          {
            motor_on();
            ((payload_t*)pvParameters)->sensor_data8 = 1;
            myflag1 = false;
          }
          
        
        }
        else
        {
          myflag1 = true;
          if (myflag2 == true)
          {
            motor_off(); 
            ((payload_t*)pvParameters)->sensor_data8 = 0;
            myflag2 = false;
          } 
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void battery_init(void)
{
    battery_setup();
    xTaskCreate((void*)battery_get_voltage_task, "battery get voltage task", STACK_SIZE, (void*)&payload, 2, NULL); 
}




