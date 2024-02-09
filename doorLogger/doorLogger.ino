/*
 * monitor door status switches
 * Print updates on LCD
 */
 
#include <Wire.h>       // support for LCD backpack and RTC
#include <LiquidTWI2.h> // LCD backpack libraries
#include <TimeLib.h>    // various time data
#include <DS1307RTC.h>  // real time clock hardware

//#define SERIAL_ON

// Pin numbers used for door switch sense inputs
const int frontDoorPin   = 7;   // the number of the pushbutton pin
const int mudroomDoorPin = A3;  // the number of the pushbutton pin
const int backDoorPin    = A2;  // the number of the pushbutton pin
const int pirPin         = A1;  // the number of the pushbutton pin
const int redLED         = 10;
const int greenLED       = 3;

// buzzer hardware pins
const int buzzerPin = 5;
const int buzzerPinLo = 4;
const int buzzerOnDelay = 50;

// Notes and their frequencies for creating buzzer tones
const int C = 1046;
const int D = 1175;
const int E = 1319;
const int F = 1397;
const int G = 1568;
const int A = 1760;
const int B = 1976;
const int C1 = 2093;
const int D1 = 2349;

//==============================================================================
// LCD display object
// Connect via i2c, address 0x20 (A0-A2 not jumpered)
LiquidTWI2 lcd(0x20);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void updateDisplay( int frontDoorState, int mudroomDoorState, int backDoorState, int pirState ) 
{
  tmElements_t tm;
  char timedatebuffer[17];

  // Interval is how long we wait to update display
  static unsigned long const displayInterval=1000;

  // Tracks the time since last event fired
  static unsigned long previousDisplayMillis=0;
  
  // Get snapshot of time
  unsigned long currentMillis = millis();

  // Check for update display
  if ((unsigned long)(currentMillis - previousDisplayMillis) >= displayInterval) 
  {
    // Use the snapshot to set track time until next event
    previousDisplayMillis = currentMillis;

    // It's time to do something!
    if (RTC.read(tm)) 
    {
      sprintf(timedatebuffer, "%02d/%02d/%02d FMBP", 
      tm.Month,
      tm.Day,
      tmYearToY2k(tm.Year) );
      
      // set the cursor to column 0, line 1
      lcd.setCursor(0, 0);
      lcd.print(timedatebuffer);

      sprintf(timedatebuffer, "%02d:%02d:%02d %1u%1u%1u%1u", 
      tm.Hour,
      tm.Minute,
      tm.Second,
      frontDoorState,
      mudroomDoorState,
      backDoorState,
      pirState);
      
      // (note: line 1 is the second row, since counting begins with 0):
      lcd.setCursor(0, 1);
      lcd.print(timedatebuffer);
    } 
    else 
    {
      if (RTC.chipPresent()) 
      {
        lcd.print(F("RTC Stopped"));
      } 
      else 
      {
        lcd.print(F("RTC ERROR"));
      }
    }    
  }
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
static unsigned char frontDoorState   = 0; // the current reading from the input pin
static unsigned char mudroomDoorState = 0; // the current reading from the input pin
static unsigned char backDoorState    = 0; // the current reading from the input pin
static unsigned char pirState         = 0; // the current reading from the input pin
//------------------------------------------------------------------------------
int updateDoorSwitchState() 
{
  static unsigned char lastFrontDoorReading = LOW;    // the previous reading from the input pin
  static unsigned long frontDoorChangeMillis = 0;     // time reading when last input change
  static unsigned char lastMudroomDoorReading = LOW;  // the previous reading from the input pin
  static unsigned long mudroomDoorChangeMillis = 0;   // time reading when last input change
  static unsigned char lastBackDoorReading = LOW;     // the previous reading from the input pin
  static unsigned long backDoorChangeMillis = 0;      // time reading when last input change
  static unsigned char lastPirReading = LOW;          // the previous reading from the input pin
  static unsigned long pirChangeMillis = 0;           // time reading when last input change

  unsigned char doorChangeFlag = 0;
  // the door switch reading debounce time
  static unsigned long debounceDelay = 50;

  // Tracks the time since last event fired
  static unsigned long previousDebounceMillis = 0;  // the last time the output pin was toggled

  unsigned char frontDoorReadPin   = digitalRead(frontDoorPin);
  unsigned char mudroomDoorReadPin = digitalRead(mudroomDoorPin);
  unsigned char backDoorReadPin    = digitalRead(backDoorPin);
  unsigned char pirReadPin         = digitalRead(pirPin);
  
  // Get snapshot of time
  unsigned long currentMillis = millis();

  // If the switch changed, due to noise or pressing:
  if (frontDoorReadPin != lastFrontDoorReading) 
  {
    // reset the debouncing timer
    frontDoorChangeMillis = currentMillis;
  }

  // If the switch changed, due to noise or pressing:
  if (mudroomDoorReadPin != lastMudroomDoorReading) 
  {
    // reset the debouncing timer
    mudroomDoorChangeMillis = currentMillis;
  }

  // If the switch changed, due to noise or pressing:
  if (backDoorReadPin != lastBackDoorReading) 
  {
    // reset the debouncing timer
    backDoorChangeMillis = currentMillis;
  }

  // If the switch changed, due to noise or pressing:
  if (pirReadPin != lastPirReading) 
  {
    // reset the debouncing timer
    pirChangeMillis = currentMillis;
  }

  if ((currentMillis - frontDoorChangeMillis) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    if (frontDoorReadPin != frontDoorState) 
    {
      frontDoorState = frontDoorReadPin;
      doorChangeFlag = 1;
    }
  }

  if ((currentMillis - mudroomDoorChangeMillis) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    if (mudroomDoorReadPin != mudroomDoorState) 
    {
      mudroomDoorState = mudroomDoorReadPin;
      doorChangeFlag = 1;
    }
  }

  if ((currentMillis - backDoorChangeMillis) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    if (backDoorReadPin != backDoorState) 
    {
      backDoorState = backDoorReadPin;
      doorChangeFlag = 1;
    }
  }

  if ((currentMillis - pirChangeMillis) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    if (pirReadPin != pirState) 
    {
      pirState = pirReadPin;
      doorChangeFlag = 1;
    }
  }

  lastFrontDoorReading   = frontDoorReadPin;
  lastMudroomDoorReading = mudroomDoorReadPin;
  lastBackDoorReading    = backDoorReadPin;
  lastPirReading         = pirReadPin;

  return( doorChangeFlag );
}

//------------------------------------------------------------------------------
// Interval between data records in milliseconds.
// The interval must be greater than the maximum SD write latency plus the
// time to acquire and write data to the SD to avoid overrun errors.
// Run the bench example to check the quality of your SD card.
const uint32_t SAMPLE_INTERVAL_MS = 1000;


//==============================================================================
bool espSyncOk = false;
long espTimeout;
long timerNow;

//------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200); // required for esp-link comms

  pinMode(frontDoorPin, INPUT_PULLUP);
  pinMode(mudroomDoorPin, INPUT_PULLUP);
  pinMode(backDoorPin, INPUT_PULLUP);
  pinMode(pirPin, INPUT);

  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  // Set the buzzer pin as an OUTPUT, set ground side of buzzer to low
  pinMode(buzzerPin, OUTPUT);
  pinMode(buzzerPinLo, OUTPUT);
  digitalWrite(buzzerPinLo, LOW);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // set the LCD type
  //lcd.setMCPType(LTI_TYPE_MCP23017); 
  lcd.setMCPType(LTI_TYPE_MCP23008); 
  lcd.begin(16, 2); // set up the LCD's number of rows and columns
  lcd.setBacklight(HIGH);

  esplink_setup1();
}
//------------------------------------------------------------------------------
void loop() 
{
  int doorChange = updateDoorSwitchState();
  updateDisplay(frontDoorState, mudroomDoorState, backDoorState, pirState);

  if( frontDoorState || mudroomDoorState || backDoorState || pirState )
  {
    digitalWrite( redLED, HIGH );
    digitalWrite( greenLED, LOW );
  }
  else
  {
    digitalWrite( redLED, LOW );
    digitalWrite( greenLED, HIGH );
  }

  timerNow = millis(); // capture timer at this loop

  // attempt to sync and finish initializing esp link
  if( espSyncOk )
  {
    esplink_loop();
    esplink_update(doorChange);
  }
  else // sync not ok so try to sync
  {
    if ( (timerNow - espTimeout) > 1000 ) // check about once per second until synced
    {
      if( espSyncOk = check_esp_link_sync() )
      {
        esplink_setup2(); // finish esp link setup once synced
      }
      espTimeout = timerNow;
    }
  }
}
