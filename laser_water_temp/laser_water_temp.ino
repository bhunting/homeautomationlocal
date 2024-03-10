/*****************
 * Read and display temp from Dallas One Wire temp sensor to OLED.
 * Used for Laser Cutter Water bath monitor.
 * Uses Dallas Temp Sensor purchased from Amazon.
 * Weewooday DS18B20 1 m/ 3.2 Ft Digital Temperature Thermal Cable Waterproof Probe Sensor -55°C to +125°C
 */

// Include the libraries we need
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

/***********************************************************************/
static U8X8_SSD1306_128X64_NONAME_4W_HW_SPI u8x8(/* cs=*/ 7, /* dc=*/ 8, /* reset=*/ 6);
static const unsigned long display_update_interval = 1000;  // set timeout for updating display, 1000 ms = 1 second
static unsigned long last_time_display_update;        // running timeout for updating display
static const unsigned long displayOffTimeout = 300000;// set timeout for blanking display 60000 ms = 60 sec, 300000 ms = 5 minutes
static unsigned long last_displayOff_time;            // running timeout for blanking display
static bool displayON = true;

/***************** BUTTON SUPPORT ***************/
static const int buttonOnePin = A2;   // physical pins on the board
static const int buttonTwoPin = A3;
int buttonOneState;                   // the current reading from the input pin
int lastButtonOneState = HIGH;        // the previous reading from the input pin
int buttonTwoState;                   // the current reading from the input pin
int lastButtonTwoState = HIGH;        // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTimeButtonOne = 0;  // the last time the output pin was toggled
unsigned long lastDebounceTimeButtonTwo = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;             // the debounce time; increase if the output flickers

unsigned long timerNow;               // capture time now for checking against timeouts for display

/*
 * The setup function. We only start the sensors here
 */
void setup(void)
{
  // start serial port
  Serial.begin(115200);
  Serial.println("Dallas Temperature IC Control Library Demo");

  pinMode(buttonOnePin, INPUT_PULLUP);
  pinMode(buttonTwoPin, INPUT_PULLUP);

  // Start up the library
  sensors.begin();
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setPowerSave(0);
}

/*
 * Main function, get and show the temperature
 */
void loop(void)
{ 
  // take action every N milliseconds for display
  timerNow = millis(); // capture timer at this loop
  // check to see if enough time has elapsed to update display
  if ( displayON && (timerNow - last_time_display_update) >= display_update_interval )
  {
    last_time_display_update = timerNow; // reset time of this update

    // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
    //Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    //Serial.println("DONE");
    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    //float tempC = sensors.getTempCByIndex(0);
    float tempF = sensors.getTempFByIndex(0);
    char buffer[8];
 
    // Check if reading was successful
    if(tempF != DEVICE_DISCONNECTED_C) 
    {
      Serial.print("Temperature for the device 1 (index 0) is: ");
      //Serial.println(tempC);
      Serial.println(tempF);

      Serial.println((unsigned int)(tempF));
      Serial.println((unsigned long)(tempF*1000));
      Serial.println((unsigned int)(tempF*100)%100);
      
      sprintf(buffer, "%d.%d", (unsigned int)(tempF), (unsigned int)(tempF*100)%100);
      Serial.println(buffer);

      u8x8.clearDisplay();
      //u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
      u8x8.drawString(0,2,buffer);
      if( tempF > 65.0)
      {
        u8x8.setInverseFont(1);
        u8x8.drawString(0,6,"OVERTEMP");
        u8x8.setInverseFont(0);
      }

      //u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
      //u8x8.drawString(0, 2, tempF);
      u8x8.refreshDisplay();    // only required for SSD1606/7  
    } 
    else
    {
      Serial.println("Error: Could not read temperature data");
      u8x8.drawString(0,6,"  ERROR  ");
    }
  }

  // Check display blank timeout
  if ( (timerNow - last_displayOff_time) >= displayOffTimeout )
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
