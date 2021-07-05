#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "Arduino.h"
#include <arduinoFFT.h>
//#include <FastLED.h>
#include "esp32-hal.h"
#include "BluetoothA2DPSink.h"
#include <math.h>   


#define SAMPLES         1024          // Must be a power of 2
#define SAMPLING_FREQ   40000         // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define NUM_BANDS       10            // To change this, you will need to change the bunch of if statements describing the mapping from bins to bands

#define LEDs 200
#define NR_OF_ALL_BITS 24*LEDs
#define KB_TASK_STACK (4*1024)
#define KB_PRIORITY 10

BluetoothA2DPSink a2dp_sink;

int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int PINS [NUM_BANDS] = {19,18,5,4,16,17};

double vReal[SAMPLES];
double vImag[SAMPLES];

arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);



int cnt = 0;
uint8_t got_data;


rmt_data_t led_data[NR_OF_ALL_BITS];
rmt_data_t led_null_data[NR_OF_ALL_BITS];
rmt_obj_t* rmt_send= {NULL};
float realTick = {0};




void read_data_stream(const uint8_t *data, uint32_t length){  
 
  if (cnt == 2) {    
    for (int i = 0; i < SAMPLES; i++){     
      vReal[i] =  data[i];
      vImag[i] = 0;
    }
    got_data = 1;  
    cnt = 0;
  }  
  cnt++;
  
}


xTaskHandle LEDTaskHandle;

void setup() {

  Serial.begin(500000);
  printf("ZONT [2.0]\n");

  
  if ((rmt_send = rmtInit(19, true, RMT_MEM_64)) == NULL)//<-------------------------- PIN
    Serial.println("init sender failed\n");    

  realTick = rmtSetTick(rmt_send, 100);
  
  static const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = 44100, // corrected by info from bluetooth
        .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
  };
  
  got_data = 0;
  
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_stream_reader(read_data_stream);
  a2dp_sink.start("MyMusic");


  xTaskCreatePinnedToCore(
                  leds,     /* Function to implement the task */
                  "LED",    /* Name of the task */
                  KB_TASK_STACK,  /* Stack size in words */
                  NULL,           /* Task input parameter */
                  (configMAX_PRIORITIES - 1), /* Priority of the task */
                  &LEDTaskHandle,     /* Task handle. */
                  1);       /* Core where the task should run */

  uint8_t dummyColor = 0;
  int i = 0;
  
  for (int y = 0; y < LEDs; y++)  {    
    for (int col=0; col<3; col++ ) {
      for (int bit=0; bit<8; bit++){
          if ( (dummyColor & (1<<(7-bit)))) {  
              led_null_data[i].level0 = 1;
              led_null_data[i].duration0 = 8;
              led_null_data[i].level1 = 0;
              led_null_data[i].duration1 = 4;
          } else {
              led_null_data[i].level0 = 1;
              led_null_data[i].duration0 = 4;
              led_null_data[i].level1 = 0;
              led_null_data[i].duration1 = 8;
          }
          i++;
      }
    }
  }
  
}


void loop() {  
   vTaskDelay(50 / portTICK_RATE_MS); 
}




void leds (void *pvParameters){
  
  int result;
  
  for (;;){  
      
    if (got_data){

      result = 0;
            
      // Reset bandValues[]
      for (int i = 0; i<NUM_BANDS; i++){
        bandValues[i] = 0;
      }
      
      got_data = false;
      
      FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(FFT_FORWARD);
      double sig = FFT.MajorPeak();
      
      if (sig > 8000) 
      {
         rmtWrite(rmt_send, led_null_data, NR_OF_ALL_BITS);
      }
      else
      {
        float coef = sqrt(sig)*0.01;
        int sect = random(4); 
        int offset = random(50);  
        uint8_t brightness =  abs(100 * coef);
        int r = abs(50 * coef);

        uint8_t startSeg  = offset + 50 * sect;
        uint8_t endSeg = offset + 50 * sect + r;
        
        printf("coef %f\tbrightness %d\tstartSeg %d\tendSeg%d\n",coef ,brightness,  startSeg,  endSeg);
        
        rewriteLEDS (brightness, startSeg,endSeg);
        
      }
    } 
    vTaskDelay(1 / portTICK_RATE_MS); 
  }
}      

      


void rewriteLEDS (uint8_t brightness, uint8_t startSeg, uint8_t endSeg){
  
  uint8_t dummyColor;  // RGB value
  int led, col, bit;
  int i=0;
  
  for (int y = 0; y < LEDs; y++)  {     
     
    if ((y >=  startSeg) && (y < endSeg))
      dummyColor = brightness;    
    else    
      dummyColor= 0;
            
       
    for (col=0; col<3; col++ ) {
      for (bit=0; bit<8; bit++){
          if ( (dummyColor & (1<<(7-bit)))) {  
              led_data[i].level0 = 1;
              led_data[i].duration0 = 8;
              led_data[i].level1 = 0;
              led_data[i].duration1 = 4;
          } else {
              led_data[i].level0 = 1;
              led_data[i].duration0 = 4;
              led_data[i].level1 = 0;
              led_data[i].duration1 = 8;
          }
          i++;
      }
    }
  }
  
  rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
  
}

