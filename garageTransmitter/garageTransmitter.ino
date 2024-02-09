/*
  garageTransmitter.ino derived from SimpleMQTTClient.ino
  Code to run on a WEMOS D1 mini clone to monitor the switches for the garage doors
  and transmit MQTT messages to the home RASPI MQTT broker on door switch state changes.
  This code monitors the garage door switches and sends the door state to the MQTT broker.
  This code runs in the WEMOS D1 that is in the garage wired to the garage door closed switches.
  There is a single RGB LED shield on the board stack to indicate RED or GREEN if any garage
  doors are open (RED) or if all garage doors are closed (GREEN).
  https://www.wemos.cc/en/latest/d1/d1_mini.html
*/

/* From https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
 * 
 * Label  GPIO	  Input	        Output	              Notes
 * D0	    GPIO16	no interrupt	no PWM or I2C support	HIGH at boot used to wake up from deep sleep
 * D1	    GPIO5	  OK	          OK	                  often used as SCL (I2C)
 * D2	    GPIO4	  OK	          OK	                  often used as SDA (I2C)
 * D3	    GPIO0	  pulled up	    OK	                  connected to FLASH button, boot fails if pulled LOW
 * D4	    GPIO2	  pulled up	    OK	                  HIGH at boot connected to on-board LED, boot fails if pulled LOW
 * D5	    GPIO14	OK	          OK	                  SPI (SCLK)
 * D6	    GPIO12	OK	          OK	                  SPI (MISO)
 * D7	    GPIO13	OK	          OK	                  SPI (MOSI)
 * D8     GPIO15	pulled to GND	OK	                  SPI (CS) Boot fails if pulled HIGH
 * RX	    GPIO3	  OK	          RX pin	              HIGH at boot
 * TX	    GPIO1	  TX pin	      OK	                  HIGH at boot debug output at boot, boot fails if pulled LOW
 * A0	    ADC0	  Analog Input	X	
 * 
*/
#include "E:/dev/sw/credentials.h"

#include "EspMQTTClient.h"

EspMQTTClient client(
  _MY_WIFI_SSID,
  _MY_WIFI_PWD,
  _MY_MQTT_SERVER,// MQTT Broker server ip
  _MY_MQTT_NAME,  // Can be omitted if not needed
  _MY_MQTT_PWD,   // Can be omitted if not needed
  "garagedoors",  // Client name that uniquely identify your device
  1883            // The MQTT port, default to 1883. this line can be omitted
);

#include <Adafruit_NeoPixel.h>
#define LED_PIN (D2)
#define LED_NUM (1)

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

// number of LEDs and 3 values for R, G, B
uint8_t ledArray[LED_NUM][3] = {0};

// PIN defines for door sense switches
#define PIN_WEST    (D5)
#define PIN_MIDDLE  (D6)
#define PIN_EAST    (D7)

long mqttUpdateTimeout; // periodic mqtt update timer
long timerNow;

// variables for reading switch states each loop
int   switch_now_WEST = 0;
int   switch_now_MIDDLE = 0;
int   switch_now_EAST = 0;

// previous readding of switch states for debouncing compared to most recent reading
int   switch_previous_WEST = 0;
int   switch_previous_MIDDLE = 0;
int   switch_previous_EAST = 0;

// last state after a debounce
int   switch_state_WEST = 0;
int   switch_state_MIDDLE = 0;
int   switch_state_EAST = 0;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void led_set(uint8_t ledvalues[LED_NUM][3]) 
{
  for (int i = 0; i < LED_NUM; i++) 
  {
    leds.setPixelColor(i, leds.Color(ledvalues[i][0], ledvalues[i][1], ledvalues[i][2]));
    leds.show();
  }
}

void setup()
{
  Serial.begin(115200);

  // Optional functionalities of EspMQTTClient
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA(); // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.enableLastWillMessage("/espdoorled/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true

  leds.begin(); // This initializes the NeoPixel library.
  led_set(ledArray);

  pinMode(PIN_WEST, INPUT_PULLUP);
  pinMode(PIN_MIDDLE, INPUT_PULLUP);
  pinMode(PIN_EAST, INPUT_PULLUP);

  mqttUpdateTimeout = millis();
}

#define setLedArray(index, R, G, B) {ledArray[index][0]=R;ledArray[index][1]=G;ledArray[index][2]=B;}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
//  client.subscribe("/garagedoors/UPDATE", [](const String & payload) 
//  {
//    //Serial.println(payload);
//    if( payload == "0")
//    {
//      // do something
//    }
//    else
//    {
//      // do something else
//    }
//  });


  // Publish a message to indicate garage door sensors online
  client.publish("/garagedoors/ONLINE", "1"); // You can activate the retain flag by setting the third parameter to true

//  // Execute delayed instructions
//  client.executeDelayed(5 * 1000, []() 
//  {
//    client.publish("/garagedoors/DELAYEDUPDATE", "This is a message sent 5 seconds later");
//  });
}

void publishDoorState(bool WEST, bool MIDDLE, bool EAST)
{
  client.publish("/garagedoors/WEST",   (WEST?"1":"0"));   // You can activate the retain flag by setting the third parameter to true
  client.publish("/garagedoors/MIDDLE", (MIDDLE?"1":"0")); // You can activate the retain flag by setting the third parameter to true
  client.publish("/garagedoors/EAST",   (EAST?"1":"0"));   // You can activate the retain flag by setting the third parameter to true
}

void loop()
{
  client.loop();

  // Set flag to not send an immediate mqtt update. Check and update based on switch changes
  bool sendMqttUpdate = false;  

  // Read door switches
  switch_now_WEST   = digitalRead(PIN_WEST);
  switch_now_MIDDLE = digitalRead(PIN_MIDDLE);
  switch_now_EAST   = digitalRead(PIN_EAST);

  // check to see if any door switches have changed state and debounce timer expired
  // check and debounce all switches together
  if ((switch_now_WEST   != switch_previous_WEST)   ||
      (switch_now_MIDDLE != switch_previous_MIDDLE) ||
      (switch_now_EAST   != switch_previous_EAST))
  {
    // reset the debouncing timer if any switches have changed
    lastDebounceTime = millis();
  }

  // update previous loop readings to look for a change next loop
  switch_previous_WEST   = switch_now_WEST;
  switch_previous_MIDDLE = switch_now_MIDDLE;
  switch_previous_EAST   = switch_now_EAST;

  // check if debounce delay is exceeded
  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    // switch state has not changed in at least debounce time
    // so take this switch reading as the actual new state:
    // Now check to see if the new switch state is different from the last debounced state
    if ((switch_now_WEST   != switch_state_WEST)   ||
        (switch_now_MIDDLE != switch_state_MIDDLE) ||
        (switch_now_EAST   != switch_state_EAST))
    {
      // debounce time expired and switch state has changed so update switches
      switch_state_WEST   = switch_now_WEST;
      switch_state_MIDDLE = switch_now_MIDDLE;
      switch_state_EAST   = switch_now_EAST;

      // set flag to update mqtt
      sendMqttUpdate = true;
    }
  }

  timerNow = millis(); // read system timer
  // update mqtt broker about every 15 seconds to keep in sync
  if( ((timerNow - mqttUpdateTimeout) > 15000) || sendMqttUpdate )
  {
    bool WEST   = (switch_state_WEST  ?HIGH:LOW);
    bool MIDDLE = (switch_state_MIDDLE?HIGH:LOW);
    bool EAST   = (switch_state_EAST  ?HIGH:LOW);

    // if any doors switch state is HIGH then turn LED RED
    if(WEST || MIDDLE || EAST)
    {
      setLedArray(0, 255, 0, 0);
    }
    else // if no doors open set LED to GREEN
    {
      setLedArray(0, 0, 255, 0);
    }    
    led_set(ledArray); // update LED
    publishDoorState(WEST, MIDDLE, EAST); // UPDATE MQTT

    // reset various timers and flags
    mqttUpdateTimeout = timerNow;
    sendMqttUpdate = false;
  }
}
