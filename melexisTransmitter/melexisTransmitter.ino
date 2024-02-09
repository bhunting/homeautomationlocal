/*
 * MQTT publisher over esp-link external using an ESP-01 of Melexis non-contact 
 * temperature sensor.
 * Uses Arduino pro mini, 0.96 OLED, and Melexis MLX90612 sensor.
 * External ESP-01 connected over serial io at 19200 due to ESP-01
 * hanging off of jumper wires.  Faster baud rates result in comm errors
 * visible in the ESP-LINK console when browsing to the ESP-01 running ESP-LINK.
*/

#include <Arduino.h>
#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#include <Wire.h>
#include <Adafruit_MLX90614.h>

/* functions to setup, drive the MQTT loop, and send updates over MQTT */
void esplink_setup1();
void esplink_setup2();
void esplink_loop();
void esplink_update(int changeFlag);
bool check_esp_link_sync();

/* Melexis sensor and OLED display objects */
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8g(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);

//------------------------------------------------------------------------------
// Interval between various events in milliseconds.
const unsigned long ESPSYNC_INTERVAL_MS        = 10000; // 10 seconds interval to attempt esp-link re-sync if not connected
const unsigned long ESPDATA_INTERVAL_MS        = 10000; // 10 seconds interval to push data to esp-link, even if no change
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 1000;  // 1 second interval to update displayed value
const unsigned long DISPLAY_BLANK_INTERVAL_MS  = 30000; // 30 seconds of no activity then blank screen
const unsigned long SAMPLE_INTERVAL_MS         = 100;   // 0.1 second read temp sensor 10 times per second
const unsigned long BUTTON_DEBOUNCE_DELAY_MS   = 50;    // 50 millisecond button press debounce delay

//==============================================================================
bool espSyncOk = false;
unsigned long timerNow = millis(); // init timer to reasonable value
unsigned long espSyncTimeout = timerNow;   // delay for re-sync check to esp-link
unsigned long espPushTimeout = timerNow;   // delay for push data to esp-link
unsigned long displayTimeout = timerNow;   // delay for updating display
unsigned long tempReadTimeout = timerNow;  // delay for reading melexis 
unsigned long displayBlankTimeout = timerNow;

// ======= capture multiple sensor readings and average to avoid random spikes in reading
const uint8_t tempAverageSize = 10;
float tempAverageArray[tempAverageSize] = {0};
float tempAverage = 0;
float tempAveragePrevious = 0;
uint8_t tempAveIndex = 0;

// ======== flag to indicate a data change that should be sent to MQTT
bool dataChangeFlag = false;

// ======== button variables for waking up display
static const int buttonOnePin = A2;
static const int buttonTwoPin = A3;
int buttonOneState;             // the current reading from the input pin
int lastButtonOneState = HIGH;   // the previous reading from the input pin
int buttonTwoState;             // the current reading from the input pin
int lastButtonTwoState = HIGH;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTimeButtonOne = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTimeButtonTwo = 0;  // the last time the output pin was toggled

// =============== local functions ============================================
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
  pinMode(buttonOnePin, INPUT_PULLUP);
  pinMode(buttonTwoPin, INPUT_PULLUP);
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
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); 
  //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF()); 
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  //Serial.println();

  timerNow = millis(); // capture timer at this loop

  if ( (timerNow - tempReadTimeout) > SAMPLE_INTERVAL_MS ) // update temp reading interval
  {
    //Serial.print("x");
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
    //Serial.print("y");
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
    u8g.setCursor( 0, 2 );
    u8g.print( tempAverage );

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
      dataChangeFlag = false; // reset flag when data has been sent
      espPushTimeout = timerNow;
    }
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

  // ======= check for blanking display
  if ( (timerNow - displayBlankTimeout) >= DISPLAY_BLANK_INTERVAL_MS )
  {
    displayBlankTimeout = timerNow;
    u8g.setPowerSave( 1 );
  }

  // ----------- read physical buttons --------------------------
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonOnePin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonOneState) 
  {
    // reset the debouncing timer
    //Serial.println("button state changed");
    lastDebounceTimeButtonOne = millis();
  }

  if ((millis() - lastDebounceTimeButtonOne) > BUTTON_DEBOUNCE_DELAY_MS) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (reading != buttonOneState) 
    {
      buttonOneState = reading;
      // only enable OLED if the new button state is LOW
      if (buttonOneState == LOW) 
      {
        //Serial.println("button low");
        u8g.setPowerSave( 0 );
        displayBlankTimeout = millis(); // button was pressed so reset display blank timeout timer
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonOneState = reading;

}
