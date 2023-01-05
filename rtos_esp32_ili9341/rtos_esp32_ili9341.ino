//////////////////////////////////////////////////////////////////////
#include "logo.h"
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <TJpg_Decoder.h>
#include "SPI.h"
#include <TFT_eSPI.h>              // Hardware-specific library
#define BLYNK_TEMPLATE_ID "TMPLU2k8lvQS"
#define BLYNK_DEVICE_NAME "Pulse oksimetre "
//#define BLYNK_AUTH_TOKEN "sPWQBJKfUEpZOvJV5th-pFayCLgAVY6E"
#define BLYNK_PRINT Serial
#include <Blynk.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
 
#define REPORTING_PERIOD_MS 1000
 
char auth[] = "sPWQBJKfUEpZOvJV5th-pFayCLgAVY6E";             // You should get Auth Token in the Blynk App.
char ssid[] = "SUPERONLINE_WiFi_C1A";                                     // Your WiFi credentials.
char pass[] = "ALPMVH9LEWVW";

uint32_t tsLastReport = 0;

  uint16_t  dmaBuffer1[16*16]; // Toggle buffer for 16*16 MCU block, 512bytes
  uint16_t  dmaBuffer2[16*16]; // Toggle buffer for 16*16 MCU block, 512bytes
  uint16_t* dmaBufferPtr = dmaBuffer1;
  bool dmaBufferSel = 0;
  volatile int analogValue,*heartratePtr,*spo2Ptr,*beatPtr;
  volatile int heartrate_,spo2_,beat_;
  volatile uint32_t beat_prev;
  volatile uint16_t xPos;
  volatile uint32_t Time1;
  volatile bool Server_Status=true;
  volatile bool *Server_Status_Ptr;

#define SCREEN_WIDTH 320 //display width, in pixels
#define SCREEN_HEIGHT 128 //display height, in pixels
#define USE_DMA
#define CONFIG_FREERTOS_UNICORE

#define SAMPLING_RATE                       MAX30100_SAMPRATE_1000HZ
#define IR_LED_CURRENT                      MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT                     MAX30100_LED_CURR_27_1MA

// The pulse width of the LEDs driving determines the resolution of
// the ADC (which is a Sigma-Delta).
// set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
#define PULSE_WIDTH                         MAX30100_SPC_PW_1600US_16BITS
#define HIGHRES_MODE                        true

PulseOximeter pox;

TaskHandle_t AnalogReadA3;
TaskHandle_t Task2;
TaskHandle_t blynkSend;

void TaskAnalogReadA3code( void *pvParameters );

const int MIN_ANALOG_INPUT = 0;
const int MAX_ANALOG_INPUT = 1023;
const int DELAY_LOOP_MS = 5; // change to slow down how often to read and graph value

int _circularBuffer[SCREEN_WIDTH]; //fast way to store values 
int _curWriteIndex = 0; // tracks where we are in the circular buffer

// for tracking fps
float _fps = 0;
unsigned long _frameCount = 0;
unsigned long _fpsStartTimeStamp = 0;

// status bar
boolean _drawStatusBar = true; // change to show/hide status bar
int _graphHeight = SCREEN_HEIGHT;
bool screen_overflow=false;

TFT_eSPI tft = TFT_eSPI();         // Invoke custom library
MAX30100 sensor;
///////////////////////////////////////////////////////////////////////

boolean toggle = false;

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap){
  if ( y >= tft.height() ) return 0;
  if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
  else dmaBufferPtr = dmaBuffer1;
  dmaBufferSel = !dmaBufferSel;
  tft.pushImageDMA(x, y, w, h, bitmap, dmaBufferPtr); 
  //tft.pushImage(x, y, w, h, bitmap); 
  return 1;
}

void drawStatusBar(int analogValue ,int xpos_){
   if(analogValue>1000){
      tft.drawNumber(analogValue,xpos_,10,4);
   }else if(analogValue>100){
      tft.drawNumber(analogValue,xpos_,10,4);
   }else if(analogValue>10){
      tft.drawNumber(analogValue,xpos_,10,4);
      tft.drawString("   ",xpos_+28,10,4);//yazıyı temizle
   }else if(analogValue>0){
      tft.drawNumber(analogValue,xpos_,10,4);
      tft.drawString("    ",xpos_+14,10,4);//yazıyı temizle
   }else{
      tft.drawString("0      ",xpos_,10,4);//yazıyı temizle 
      //analogValue=0;
   }
}

void drawLine(int _xPos, int analogVal){
   tft.drawFastVLine(_xPos,112,210-analogVal,TFT_BLACK);//alt cizgileri temizle
   tft.drawFastVLine(_xPos,240-analogVal,analogVal,TFT_GREEN);
}

void drawGraph(int val_){

  // //////////////serial.println(val_);
   val_=val_+360;
 if (val_<1408 && val_>0) {
     val_=val_/11;
  }else{
     val_=36;
 }
    
 if( xPos >= SCREEN_WIDTH){
    xPos = 0;
  }

    xPos++;
    drawLine(xPos, val_);
    tft.drawFastVLine(xPos+1,240,128,TFT_BLACK);
}

// define two tasks for Blink & AnalogRead
void TaskBlinkcode( void *pvParameters );
void TaskAnalogReadA3code( void *pvParameters );
void TaskblynkSend( void *pvParameters );

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(17, OUTPUT);

  Serial.begin(115200);
  analogReadResolution(12);

  analogValue=0;
  // Initialise the TFT
  tft.begin();
  tft.setRotation(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.initDMA();

  tft.setCursor(0, 0, 4);


  TJpgDec.setJpgScale(1);
  tft.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  tft.startWrite();
  TJpgDec.drawJpg(0, 0, kou_logo, sizeof(kou_logo));// jpeg coz ve dma logo çizdir
  tft.endWrite();

  delay(1000);

  TJpgDec.setJpgScale(1);
  tft.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  TJpgDec.drawJpg(0, 0, heart_logo, sizeof(heart_logo));//dma logo çizdir
  tft.endWrite();

  TJpgDec.setJpgScale(1);
  tft.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  tft.startWrite();
  TJpgDec.drawJpg(150, 0, spo2_logo, sizeof(spo2_logo));//dma logo çizdir
  tft.endWrite();

  TJpgDec.setJpgScale(1);
  tft.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  tft.startWrite();
  TJpgDec.drawJpg(16, 50, wifi_logo, sizeof(wifi_logo));//dma logo çizdir
  tft.endWrite();

  tft.drawString("BPM:",48,10,4);
  tft.drawString("SpO2%:",200,10,4);
  
    digitalWrite(17, HIGH);
    delay(100);
    digitalWrite(17, LOW);
   

  xTaskCreatePinnedToCore(
    TaskblynkSend
    ,  "blynkSend"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &blynkSend 
    ,  0); /* pin task to core 1 */


  vTaskDelay(50);
  
  // // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskAnalogReadA3code
    ,  "AnalogReadA3"   // A name just for humans
    ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &AnalogReadA3 
    ,  1); /* pin task to core 1 */

      // // Now set up two tasks to run independently.

   vTaskDelay(50);
      //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
   xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
     10000,       /* Stack size of task */
     NULL,        /* parameter of the task */
     2,           /* priority of the task */
     &Task2,      /* Task handle to keep track of created task */
     0);          /* pin task to core 0 */

    


  

}

void loop()
{
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void TaskAnalogReadA3code(void *pvParameters)  // This is a task.
{

    //////////////serial.print("Initializing pulse oximeter..");
    if (!pox.begin()) {
        //////////////serial.println("FAILED");
        for(;;);
    } else {
        //////////////serial.println("SUCCESS");
    }
    pox.setMode(MAX30100_MODE_SPO2_HR);
    pox.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT);
    pox.setLedsPulseWidth(PULSE_WIDTH);
    pox.setSamplingRate(SAMPLING_RATE);
    pox.setHighresModeEnabled(HIGHRES_MODE);

  for(;;)
  {
        pox.update();

        heartrate_=pox.getHeartRate();
        if(heartrate_==0 || heartrate_>120){
          digitalWrite(17, LOW);
          heartrate_=0;
        }
        spo2_=pox.getSpO2();
        if(spo2_>100 || spo2_<50){
          spo2_=0;
        }

        beat_=pox.getbeat();
        ////////////////serial.println(beat_);

        if(heartrate_>0 && heartrate_<120 && spo2_!=0)
         if(millis()-Time1>=10000/heartrate_){
          toggle = !toggle;
          digitalWrite(17, toggle);
          Time1=millis();
         }

        beatPtr=&beat_;
        heartratePtr=&heartrate_;
        spo2Ptr=&spo2_;
        vTaskDelay(1);
  }
}

void Task2code( void * pvParameters )
{
  //////////////serial.println("Task2code basladi!!!");
  tft.drawString("Sunucuya Baglaniyor...",48,50,2);
  for(;;)
  {
    drawGraph(*beatPtr);
    drawStatusBar(*heartratePtr,108);
    drawStatusBar(*spo2Ptr,290);
    vTaskDelay(3);
  }
}

void TaskblynkSend( void * pvParameters )
{
  Blynk.begin(auth, ssid, pass);
  Server_Status_Ptr=&Server_Status;
  tft.drawString("Sunucuya Baglandi!!    ",48,50,2);
  vTaskDelay(1);
  for(;;)
  {
    Blynk.run();
    Blynk.virtualWrite(V7, *heartratePtr);
    Blynk.virtualWrite(V8, *spo2Ptr);
    vTaskDelay(1000);
  }
}
