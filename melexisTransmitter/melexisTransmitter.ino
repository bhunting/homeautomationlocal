/*

  
*/

#include <Arduino.h>
#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#include <Wire.h>
#include <Adafruit_MLX90614.h>

void esplink_setup1();
void esplink_setup2();
void esplink_loop();
void esplink_update(int changeFlag);
bool check_esp_link_sync();


Adafruit_MLX90614 mlx = Adafruit_MLX90614();
U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8g(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);

//------------------------------------------------------------------------------
// Interval between data records in milliseconds.
const long ESPSYNC_INTERVAL_MS        = 10000; // 1 seconds interval to attempt esp-link re-sync if not connected
const long ESPDATA_INTERVAL_MS        = 10000; // 10 seconds interval to push data to esp-link, even if no change
const long DISPLAY_UPDATE_INTERVAL_MS = 1000;  // 1 second interval to update displayed value
const long DISPLAY_BLANK_INTERVAL_MS  = 30000; // 30 seconds of no activity then blank screen
const long SAMPLE_INTERVAL_MS         = 100;   // 0.1 second read temp sensor 10 times per second

//==============================================================================
bool espSyncOk = false;
long timerNow = millis(); // init timer to reasonable value
long espSyncTimeout = timerNow;   // delay for re-sync check to esp-link
long espPushTimeout = timerNow;   // delay for push data to esp-link
long displayTimeout = timerNow;   // delay for updating display
long tempReadTimeout = timerNow;  // delay for reading melexis 
long displayBlankTimeout = timerNow;

const uint8_t tempAverageSize = 10;
double tempAverageArray[tempAverageSize] = {0};
double tempAverage = 0;
double tempAveragePrevious = 0;
uint8_t tempAveIndex = 0;

bool dataChangeFlag = false;

void pre(void)
{
  u8g.setFont(u8x8_font_amstrad_cpc_extended_f);    
  u8g.clear();

  if( espSyncOk )
  {
    u8g.noInverse();
    u8g.print(" ESPSYNC OK ");
  }
  else
  {
    u8g.inverse();
    u8g.print(" ESPSYNC FAIL ");
  }

  u8g.setFont(u8x8_font_chroma48medium8_r);  
  u8g.noInverse();
  //u8g.setCursor(0,1);
  u8g.drawString(0, 1, "Melexis ESP-LINK");
}

void setup(void) 
{
  Serial.begin(19200); // required for esp-link comms
  //Serial.println("MLX90614 MQTT PUB");
  esplink_setup1();
  mlx.begin();  

  // screen setup, rotation and font
  u8g.begin();
  //u8g.setRot270();
  //u8g.setFont(u8x8_font_inb33_3x6_n);
  //u8g.setFont(u8x8_font_px437wyse700b_2x2_r);

  // preset average temperature array
  for( int i = 0; i < tempAverageSize; i++)
  {
        tempAverageArray[i] = mlx.readObjectTempF();
        delay(10);
  }
}

void loop(void) 
{
  static int i = 0;

  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); 
  //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  //Serial.println();

  timerNow = millis(); // capture timer at this loop

    if ( (timerNow - tempReadTimeout) > SAMPLE_INTERVAL_MS ) // update temp reading interval
    {
      tempAverageArray[tempAveIndex] = mlx.readObjectTempF();
      tempAveIndex++; // point to next available slot in the average array
      if( tempAveIndex >= tempAverageSize) // if pointing to past array then reset to start of array
      {
        tempAveIndex = 0;
      }
      tempReadTimeout = timerNow;
    }

    // on human readable timescale average data and send to display and esp link
    if ( (timerNow - displayTimeout) > DISPLAY_UPDATE_INTERVAL_MS ) // update display interval
    {
      for( int i = 0; i < tempAverageSize; i++)
      {
        tempAverage += tempAverageArray[i];
      }
      tempAverage /= tempAverageSize;

      // if the data has changed by more than 0.5 degrees then push the data
      if( abs(tempAverage - tempAveragePrevious) > 0.5 )
      {
        dataChangeFlag = true;
        tempAveragePrevious = tempAverage;
      }

      //Serial.print("Average temp = ");
      //Serial.print(tempAverage);
      //Serial.println();

      pre();
      u8g.setFont(u8x8_font_inb33_3x6_n);
      for(i = 0; i < 100; i++ )
      {
        u8g.setCursor( 0, 2 );
        u8g.print( tempAverage );
      }
      displayTimeout = timerNow;
    }

  // if esp synced with mqtt broker then send data
  if( espSyncOk )
  {
    esplink_loop(); // run esp.process to drive esp mqtt link

    if ( ((timerNow - espPushTimeout) > ESPDATA_INTERVAL_MS) ||
          dataChangeFlag ) // push esp data interval
    {
      esplink_update(1);
      espPushTimeout = timerNow;
    }
    //esplink_update(doorChange);
  }
  else // sync not ok so try to sync
  {
    if ( (timerNow - espSyncTimeout) > ESPSYNC_INTERVAL_MS ) // check about once per second until synced
    {
      if( (espSyncOk = check_esp_link_sync()) )
      {
        esplink_setup2(); // finish esp link setup once synced
      }
      espSyncTimeout = timerNow;
    }
  }


}
