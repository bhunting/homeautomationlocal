/*
 * MQTT publisher over esp-link external using an ESP-01 of DS18B20 temperature sensor.
 * Uses Arduino pro mini, 0.96 OLED, and DS18B20 sensor.
 * External ESP-01 connected over serial io at 19200 due to ESP-01
 * hanging off of jumper wires.  Faster baud rates result in comm errors
 * visible in the ESP-LINK console when browsing to the ESP-01 running ESP-LINK.
*/

#include <Arduino.h>
#include <U8x8lib.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Dallas Temp Sensor data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//-------------------------------------------------------------
// OLED Display
static U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);


/* functions to setup, drive the MQTT loop, and send updates over MQTT */
void esplink_setup1();
void esplink_setup2();
void esplink_loop();
void esplink_update(int changeFlag);
bool check_esp_link_sync();


// Running timers for various events
unsigned long timerNow = millis(); // init timer to reasonable value

//------------------------------------------------------------------------------
// Timeouts and Timers for ESP-LINK MQTT / WIFI updates
static const unsigned long ESPSYNC_INTERVAL_MS        = 10000; // 10 seconds interval to attempt esp-link re-sync if not connected
static const unsigned long ESPDATA_INTERVAL_MS        = 10000; // 10 seconds interval to push data to esp-link, even if no change

bool espSyncOk = false;
unsigned long espSyncTimeout = timerNow;   // delay for re-sync check to esp-link
unsigned long espPushTimeout = timerNow;   // delay for push data to esp-link

//----------------------------------------------------------------------------
// Timeouts and Timers for Display Update and Blanking
static const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 1000;       // 1 second interval to update displayed value
static const unsigned long DISPLAY_BLANK_INTERVAL_MS  = 30000;      // 30 seconds of no activity then blank screen

static bool displayON                                 = true;
static unsigned long last_displayOff_time             = timerNow;   // running timeout for blanking display
static unsigned long displayTimeout                   = timerNow;   // delay for updating display

//----------------------------------------------------------------------------
// Timeouts and Timers for sensor readings
static const unsigned long SAMPLE_INTERVAL_MS         = 1000;       // 1 second read temp sensor interval
unsigned long tempReadTimeout                         = timerNow;   // delay for reading temperature sensor 


unsigned short alarmTime_ON                           = 500;        // alarm ON for 0.5 second
unsigned short alarmTime_OFF                          = 1000;       // alarm OFF for 1 second
unsigned long alarmTimer                              = timerNow;   // alarm buzzer ON / OFF timer

unsigned short alarmTime_DELAY_ON                     = 10000;      // delay time for temp above alarm before sounding alarm
unsigned long alarmTimer_DELAY                        = timerNow;   // 

float alarmTemperature                                = 5.0;        // 5 degrees F (32-5 or 27 degrees below freezing)
float alarmTemperatureError                           = -20.0;      // if freezer sensor ever reads below -20 then there is an error in the reading

// ======= capture multiple sensor readings and average to avoid random spikes in reading
const uint8_t tempAverageSize           = 10;
float tempAverageArray[tempAverageSize] = {0};
float tempAverage                       = 0;
float tempAveragePrevious               = 0;
uint8_t tempAveIndex                    = 0;
bool dataChangeFlag                     = false;    // flag to indicate a data change that should be sent to MQTT

//----------------------------------------------------------------------------
// Timeouts and Timers for button readings
static const unsigned long BUTTON_DEBOUNCE_DELAY_MS = 50;   // 50 millisecond button press debounce delay
static const int buttonOnePin                       = A2;   // physical pins on the board
static const int buttonTwoPin                       = A3;
int buttonOneState;                                         // the current reading from the input pin
int lastButtonOneState                              = HIGH; // the previous reading from the input pin
int buttonTwoState;                                         // the current reading from the input pin
int lastButtonTwoState                              = HIGH; // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTimeButtonOne             = 0;    // the last time the output pin was toggled
unsigned long lastDebounceTimeButtonTwo             = 0;    // the last time the output pin was toggled
unsigned long debounceDelay                         = 50;   // the debounce time; increase if the output flickers

// =============== local functions ============================================
void pre(void)
{
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);    
  u8x8.clear();

  if( espSyncOk )
  {
    u8x8.noInverse();
    u8x8.print(" ESPSYNC OK ");
  }
  else
  {
    u8x8.inverse();
    u8x8.print(" ESPSYNC FAIL ");
  }

  u8x8.setFont(u8x8_font_chroma48medium8_r);  
  u8x8.noInverse();
  //u8x8.setCursor(0,1);
  u8x8.drawString(0, 1, "Freezer ESP-LINK");
}

//----- LED and BUZZER management functions
#define LED_RED_PIN (9)
#define LED_GRN_PIN (10)
#define BZR_PIN (2)

// ALARM STATES
// 0 - ALARM OFF
// 1 - ALARM TOGGLE ON
// 2 - ALARM TOGGLE OFF
// 3 - ALARM DELAY
typedef enum ALARM_STATES
{
    ALARM_IDLE,
    ALARM_ON,
    ALARM_OFF,
    ALARM_DELAY
} alarm_states_t;

void checkSetAlarm(float temperature)
{
  static alarm_states_t alarmSTATE = ALARM_IDLE;

  // if temperature is COLD / GOOD then turn GRN LED ON and RED LED and BUZZER OFF
  if((temperature < alarmTemperature) && (temperature > alarmTemperatureError))
  {
    alarmSTATE = ALARM_IDLE;          // ALARM IDLE state
    digitalWrite(LED_GRN_PIN, LOW);   // turn the GRN LED ON. LED active LOW
    digitalWrite(LED_RED_PIN, HIGH);  // turn the RED LED OFF.  LED active LOW
    digitalWrite(BZR_PIN, HIGH);      // turn the BUZZER OFF.  BUZZER active LOW
  }
  else
  {
    timerNow = millis(); // Capture time now
    // Else Temperature is above alarm temperature so take action
    switch( alarmSTATE )
    {
      case ALARM_IDLE: // ALARM IDLE
      // if previously in temperature OK/IDLE state, alarmSTATE IDLE, then
      // check for temperature is HOT or ERROR reading then transition to delay state 
      if((temperature > alarmTemperature) || (temperature < alarmTemperatureError)) // if temperature is above alarm temperature or below error temperature then transition to alarm state
      {
        alarmTimer_DELAY = timerNow + alarmTime_DELAY_ON; // set delay time to now plus delay wait time
        alarmSTATE = ALARM_DELAY; // ALARM DELAY, we were in alarm IDLE state but transitioning to OVER TEMP STATE, so DELAY first
      }
      break;

      case ALARM_ON: // ALARM TOGGLE ON
      digitalWrite(LED_GRN_PIN, HIGH);  // turn the GRN LED OFF. LED active LOW
      digitalWrite(LED_RED_PIN, LOW);   // turn the RED LED ON.  LED active LOW
      digitalWrite(BZR_PIN, LOW);       // turn the BUZZER ON.  BUZZER active LOW
      if( timerNow > alarmTimer )
      {
        // if we have stayed in ALARM ON state for longer than timeout then transition to alarm OFF
        alarmTimer = timerNow + alarmTime_OFF;
        alarmSTATE = ALARM_OFF;
      }
      break;

      case ALARM_OFF: // ALARM TOGGLE OFF
      digitalWrite(LED_GRN_PIN, HIGH);  // turn the GRN LED OFF. LED active LOW
      digitalWrite(LED_RED_PIN, HIGH);  // turn the RED LED OFF.  LED active LOW
      digitalWrite(BZR_PIN, HIGH);      // turn the BUZZER OFF.  BUZZER active LOW
      if( timerNow > alarmTimer )
      {
        // if we have stayed in ALARM OFF state for longer than timeout then transition to alarm ON
        alarmTimer = timerNow + alarmTime_ON;
        alarmSTATE = ALARM_ON;
      }
      break;

      case ALARM_DELAY: // ALARM DELAY
      if( timerNow > alarmTimer_DELAY )
      {
        // if we have stayed in delay state for longer than delay timeout then transition to alarm ON
        alarmTimer = timerNow + alarmTime_ON;
        alarmSTATE = ALARM_ON;
      }
      break;

      // Unknown state so transition to alarmSTATE Toggle ON to indicate an error
      default:
      alarmSTATE = ALARM_ON;
      break;
    }
  }
}

void setup(void) 
{
  // RED / GRN LEDs and Buzzer are Active LOW.  Setting output low with enable LED / Buzzer
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GRN_PIN, OUTPUT);
  pinMode(BZR_PIN, OUTPUT);

  digitalWrite(LED_RED_PIN, HIGH);  // turn the LED OFF.  LED active LOW
  digitalWrite(LED_GRN_PIN, HIGH);  // turn the LED OFF.  LED active LOW
  digitalWrite(BZR_PIN, HIGH);      // turn the BZR OFF.  BZR active LOW

  Serial.begin(19200); // required for esp-link comms
  pinMode(buttonOnePin, INPUT_PULLUP);
  pinMode(buttonTwoPin, INPUT_PULLUP);
  esplink_setup1();
  sensors.begin();

  // screen setup, rotation and font
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setPowerSave(0);

  //u8x8.setRot270();
  //u8x8.setFont(u8x8_font_inb33_3x6_n);
  //u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);

  // preset average temperature array
  for( int i = 0; i < tempAverageSize; i++)
  {
      // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
      sensors.requestTemperatures(); // Send the command to get temperatures
      // Use the function ByIndex to get the temperature from the first sensor only.
      //float tempC = sensors.getTempCByIndex(0);
      float tempF = sensors.getTempFByIndex(0);
 
    // Check if reading was successful
    if(tempF != DEVICE_DISCONNECTED_C) 
    {
        tempAverageArray[i] = tempF;
    }
    else
    {
      // Print ERROR on OLED DISPLAY
        tempAverageArray[i] = 0;
    }

    delay(100);
  }
}

void loop(void) 
{
  timerNow = millis(); // capture timer at this loop
  float tempF = sensors.getTempFByIndex(0); // init to something reasonable

  if ( (timerNow - tempReadTimeout) > SAMPLE_INTERVAL_MS ) // update temp reading interval
  {
    // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
    sensors.requestTemperatures(); // Send the command to get temperatures
    // Use the function ByIndex to get the temperature from the first sensor only.
    // tempC = sensors.getTempCByIndex(0);
    tempF = sensors.getTempFByIndex(0); // update temp from sensor
        // Check if reading was successful
    if(tempF != DEVICE_DISCONNECTED_C) 
    {
        tempAverageArray[tempAveIndex] = tempF;
    }
    else
    {
      // Print ERROR on OLED DISPLAY
        tempAverageArray[tempAveIndex] = 0;
    }
    tempAveIndex++; // point to next available slot in the average array
    if( tempAveIndex >= tempAverageSize) // if pointing to past array then reset to start of array
    {
      tempAveIndex = 0;
    }
    tempReadTimeout = timerNow;
  }

  checkSetAlarm( tempF );

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

    pre();
    u8x8.setFont(u8x8_font_inb21_2x4_n);
    u8x8.setCursor( 0, 2 );
    u8x8.print( tempAverage );

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
  if ( (timerNow - last_displayOff_time) >= DISPLAY_BLANK_INTERVAL_MS )
  {
    last_displayOff_time = timerNow;
    u8x8.setPowerSave( 1 );
    displayON = false;
  }

  // ----------- read physical buttons --------------------------
  // read the state of the switch into a local variable:
  int buttonOneValue = digitalRead(buttonOnePin);
  int buttonTwoValue = digitalRead(buttonTwoPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  timerNow = millis();  // refresh timer value for button checks
  // If the switch changed, due to noise or pressing:
  if ((buttonOneValue != lastButtonOneState) || (buttonTwoValue != lastButtonTwoState))
  {
    // reset the debouncing timer
    lastDebounceTimeButtonOne = timerNow;
    lastDebounceTimeButtonTwo = timerNow;
  }

  if (((timerNow - lastDebounceTimeButtonOne) > debounceDelay) || ((timerNow - lastDebounceTimeButtonTwo) > debounceDelay))
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if ((buttonOneValue != buttonOneState) || (buttonTwoValue != buttonTwoState))
    {
      buttonOneState = buttonOneValue;
      buttonTwoState = buttonTwoValue;
      // only enable OLED if the new button state is LOW
      if ((buttonOneState == LOW) || (buttonTwoState == LOW))
      {
        u8x8.setPowerSave( 0 );           // enable display
        displayON = true;
        last_displayOff_time = timerNow;  // reset display blanking timeout to button press time
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonOneState = buttonOneValue;
  lastButtonTwoState = buttonTwoValue;

}
