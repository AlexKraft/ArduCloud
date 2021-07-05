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
#define NOISE           500           // Used as a crude noise filter, values below this are ignored

#define AMPLITUDE       1000          // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.
#define TOP 200
#define NR_OF_ALL_BITS 24*TOP
#define KB_TASK_STACK (4*1024)
#define KB_PRIORITY 10

BluetoothA2DPSink a2dp_sink;

int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int oldBarHeights[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte peak[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};              // The length of these arrays must be >= NUM_BANDS

int PINS [NUM_BANDS] = {19,18,5,4,16,17};

double vReal[SAMPLES];
double vImag[SAMPLES];

arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);

uint8_t got_data;

byte brightness = 30;
uint8_t color[] =  { 0x55, 0x11, 0x77 };  // RGB value
typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;
typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;


rmt_data_t led_data[NR_OF_ALL_BITS];
rmt_data_t led_null_data[NR_OF_ALL_BITS];
rmt_obj_t* rmt_send= {NULL};
float realTick = {0};

int cnt = 0;
uint8_t colorTimer = 50;



void read_data_stream(const uint8_t *data, uint32_t length){  
 
  if (cnt == 2) {    
    for (int i = 0; i < SAMPLES; i++){     
      vReal[i] =  data[i];
      vImag[i] = 0;
    }
    got_data = 1;  
    cnt = 0;
//    printf("got_data\n");
  }  
  cnt++;
  
}


//  Serial.printf("Got data len %d\nCopied with core %d\n SET the flag to %d\n",length,xPortGetCoreID(),got_data);




/*
PINS TO CONNECT ws2812
*/
// #define STRIP_0 19
// #define STRIP_1 18
// #define STRIP_2 5
// #define STRIP_3 17
// #define STRIP_4 16
// #define STRIP_5 4
xTaskHandle LEDTaskHandle;

void setup() {

  Serial.begin(500000);
  printf("ZONT [2.0]\n");

  
//  for (int i = 0; i <= NUM_BANDS; i ++ )
//  {
//    if ((rmt_send[i] = rmtInit(PINS[i], true, RMT_MEM_64)) == NULL)
//        Serial.println("init sender failed\n");
//  }
  if ((rmt_send = rmtInit(19, true, RMT_MEM_64)) == NULL)//<-------------------------- PIN
    Serial.println("init sender failed\n");
    
//  for (int i = 0; i < NUM_BANDS; i++){
//    realTick[i] = rmtSetTick(rmt_send[i], 100);
//  }

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
  for (int y = 0; y < TOP; y++)  {    
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

  HsvColor hsv;    
  RgbColor ReGB;

  hsv.s = 255;
  hsv.v = brightness;
  char k = 0;

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
//        printf("\t%d\t%d\n", 0,0);
      }
      else
      {
//        printf("%f1",sig);
        float coef = sqrt(sig)*0.01;
        int sect = random(4); 
        int offset = random(50);  
        uint8_t brightness =  abs(100 * coef);
        int r = abs(50 * coef);

        uint8_t startSeg  = offset + 50 * sect;
        uint8_t endSeg = offset + 50 * sect + r;
//        printf("\t%d\t%d\n",r,brightness);
//        printf("coef %f",coef);
        printf("coef %f\tbrightness %d\tstartSeg %d\tendSeg%d\n",coef ,brightness,  startSeg,  endSeg);
        rewriteLEDS (brightness, startSeg,endSeg);
        
      }


//      printf("\n");
      
    } 
    vTaskDelay(1 / portTICK_RATE_MS); 
  }
}      

      


void rewriteLEDS (uint8_t brightness, uint8_t startSeg, uint8_t endSeg){
  
  uint8_t dummyColor;  // RGB value
  int led, col, bit;
  int i=0;
  
  for (int y = 0; y < TOP; y++)  {     
     
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


RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;
    
    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6; 

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}
