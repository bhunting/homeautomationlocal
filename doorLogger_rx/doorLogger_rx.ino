/*
 Load this onto OLED ProMini
 Run doorLogger on other node to send door status changes to this node
 */

/**
 * Simplest possible example of using RF24Network,
 *
 * RECEIVER NODE
 * Listens for messages from the transmitter and prints them out.
 */

#include <SPI.h>
#include <U8x8lib.h>  // OLED display
#include "LPD8806.h"  // RGB LED strand

#define IDX_FRONT     (0)
#define IDX_BACK      (1)
#define IDX_MUDROOM   (2)
#define IDX_PIR       (3)
#define NUM_DOORS     (4)

/***********************************************************************/
// Arrays used for OLED display positions and values
// 12345678901234567890123456789012   array count
// 01234567890123456789012345678901   array index position
// YYMMDD,HHMMSS,F,M,B,P,C
static char doorStatusString[32];
static const int frontDoorStringPos = 14;
static const int mudroomDoorStringPos = 16;
static const int backDoorStringPos = 18;
static const int pirDoorStringPos = 20;

const int doorChangeArrayRows = 5;
const int doorChangeArrayCols = 24;
static char doorChangeArray[doorChangeArrayRows][doorChangeArrayCols];

/***********************************************************************/
// RGB LED strand
// Number of RGB LEDs in strand:
int nLEDs = 4;
// Chose 2 pins for output; can be any valid output pins:
int dataPin  = 3; 
int clockPin = 4;
// First parameter is the number of LEDs in the strand. 
// Next two parameters are SPI data and clock pins:
LPD8806 strip = LPD8806(nLEDs, dataPin, clockPin);

/***********************************************************************/
static U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);
static const unsigned long display_update_interval = 1000; // ms       // Delay.
static unsigned long last_time_display_update;
static const unsigned long displayOffTimeout = 60000; // 60000 ms = 60 sec
static unsigned long last_displayOff_time;

/***********************************************************************/
// General variables
static const int numDoors = 4;
static int doorStatusArray[ numDoors ];
static int commOK = 1;
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
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

bool espSyncOk = false;
long espTimeout;
long timerNow;

/***********************************************************************/
void setup(void)
{
  Serial.begin(19200);

  pinMode(7, OUTPUT);     // OLED CS
  digitalWrite(7, HIGH);  // de-select OLED CS  
  pinMode(8, OUTPUT);     // OLED DC
  digitalWrite(8, HIGH);  // set OLED DC

  pinMode(buttonOnePin, INPUT_PULLUP);
  pinMode(buttonTwoPin, INPUT_PULLUP);

  SPI.begin();
  /* U8g2 Project: SSD1306 Test Board */
  u8x8.begin();
  u8x8.setPowerSave(0);

  // Start up the LED strip
  strip.begin();
  // Update the strip, to start they are all 'off'
  strip.show();  

  // esp-link
  esplink_setup1();
  espTimeout = millis();
}

void loop(void)
{
    // update doorStatusArray
    //doorStatusArray[ 0 ] = doorStatusString[ frontDoorStringPos ] - '0';
    //doorStatusArray[ 1 ] = doorStatusString[ mudroomDoorStringPos ] - '0';
    //doorStatusArray[ 2 ] = doorStatusString[ backDoorStringPos ] - '0';
    //doorStatusArray[ 3 ] = doorStatusString[ pirDoorStringPos ] - '0';

  // attempt to sync and finish initializing esp link
  timerNow = millis(); // capture timer at this loop
  if( espSyncOk )
  {
    esplink_loop();
  }
  else // sync not ok so try to sync
  {
    if ( (timerNow - espTimeout) > 1000 ) // check about once per second until synced
    {
      if( espSyncOk = check_esp_link_sync() )
      {
        Serial.println("Call setup2");
        esplink_setup2(); // finish esp link setup once synced
      }
      espTimeout = timerNow;
    }
  }

  static int clearSent = 0;  
  unsigned long now = millis(); // Update display and LED string once ever n-millisecs
  if ( (now - last_time_display_update) >= display_update_interval )
  {
    last_time_display_update = now;

    // Update OLED display
    switch( clearSent )
    {
      case 0:
      u8x8.drawString(14,0, " *");
      clearSent = 1;
      break;

      case 1:
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(0,0, &doorStatusString[7]);
      u8x8.drawString(14,0, "  ");
      u8x8.drawString(0,2, &doorChangeArray[0][7]);
      u8x8.drawString(0,3, &doorChangeArray[1][7]);
      u8x8.drawString(0,4, &doorChangeArray[2][7]);
      u8x8.drawString(0,5, &doorChangeArray[3][7]);
      u8x8.drawString(0,6, &doorChangeArray[4][7]);
      clearSent = 0;
      break;

      default:
      clearSent = 0;
      break;
    }

    // Update LED strip
    if( commOK )
    {
      strip.setPixelColor( 0, (doorStatusArray[0]?100:0), (doorStatusArray[0]?0:100), 0);
      strip.setPixelColor( 1, (doorStatusArray[1]?100:0), (doorStatusArray[1]?0:100), 0);
      strip.setPixelColor( 2, (doorStatusArray[2]?100:0), (doorStatusArray[2]?0:100), 0);
      strip.setPixelColor( 3, (doorStatusArray[3]?100:0), (doorStatusArray[3]?0:100), 0); // PIR
    }
    else
    {
      strip.setPixelColor( 0, 100, 100, 0);
      strip.setPixelColor( 1, 100, 100, 0);
      strip.setPixelColor( 2, 100, 100, 0);
      strip.setPixelColor( 3, 100, 100, 0); // PIR
    }
    strip.show();
  }

  if ( (now - last_displayOff_time) >= displayOffTimeout )
  {
    last_displayOff_time = now;
    u8x8.setPowerSave( 1 );
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
    lastDebounceTimeButtonOne = millis();
  }

  if ((millis() - lastDebounceTimeButtonOne) > debounceDelay) 
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
        u8x8.setPowerSave( 0 );
        last_displayOff_time = millis();
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonOneState = reading;
}
