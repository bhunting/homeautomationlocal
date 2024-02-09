/*
  WEMOS D1 mini clone with WEMOS 7 pixel RGB_LED shield.
  Used to display the 7 door states for the 3 house doors, the 3 garage doors, and the TV PIR sensor.
  This code subscribes to the RASPI MQTT broker to listen to the door messages and display the door 
  status on the LEDs.  This code does not monitor the actual door sensors.  This code only subscribes 
  to MQTT messages and displayes on the LEDs the status of the messages.
  See for docs on the RGB shield https://www.wemos.cc/en/latest/d1_mini_shield/rgb_led.html
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
  _MY_MQTT_SERVER,  // MQTT Broker server ip
  _MY_MQTT_NAME,   // Can be omitted if not needed
  _MY_MQTT_PWD,   // Can be omitted if not needed
  "espdoorled",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

#include <Adafruit_NeoPixel.h>
#define PIN   D4
#define LED_NUM 7
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_NUM, PIN, NEO_GRB + NEO_KHZ800);

// number of LEDs and 3 values for R, G, B
uint8_t ledArray[LED_NUM][3] = {0};

// LED array index for each door
#define IDX_FRONT     (0)
#define IDX_BACK      (1)
#define IDX_MUDROOM   (2)
#define IDX_PIR       (3)
#define IDX_GAR_WEST  (4)
#define IDX_GAR_MID   (5)
#define IDX_GAR_EAST  (6)

long ledTimeout; // LED update timer
long timerNow;

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
  ledTimeout = millis();
}

#define setLedArray(index, R, G, B) {ledArray[index][0]=R;ledArray[index][1]=G;ledArray[index][2]=B;}
static bool doorChange = false;

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
  client.subscribe("/doorlogger/FRONT", [](const String & payload) 
  {
    //Serial.println(payload);
    if( payload == "0")
    {
      setLedArray(IDX_FRONT,0,50,0);
    }
    else
    {
      setLedArray(IDX_FRONT,50,0,0);
    }
    doorChange = true;
  });

  client.subscribe("/doorlogger/BACK", [](const String & payload) 
  {
    //Serial.println(payload);
    if( payload == "0")
    {
      setLedArray(IDX_BACK,0,50,0);
    }
    else
    {
      setLedArray(IDX_BACK,50,0,0);
    }
    doorChange = true;
  });

  client.subscribe("/doorlogger/MUDROOM", [](const String & payload) 
  {
    //Serial.println(payload);
    if( payload == "0")
    {
      setLedArray(IDX_MUDROOM,0,50,0);
    }
    else
    {
      setLedArray(IDX_MUDROOM,50,0,0);
    }
    doorChange = true;
  });

  client.subscribe("/doorlogger/PIR", [](const String & payload) 
  {
    //Serial.println(payload);
    if( payload == "0")
    {
      setLedArray(IDX_PIR,0,50,0);
    }
    else
    {
      setLedArray(IDX_PIR,50,0,0);
    }
    doorChange = true;
  });

  client.subscribe("/garagedoors/WEST", [](const String & payload) 
  {
    //Serial.println(payload);
    if( payload == "0")
    {
      setLedArray(IDX_GAR_WEST,0,50,0);
    }
    else
    {
      setLedArray(IDX_GAR_WEST,50,0,0);
    }
    doorChange = true;
  });

  client.subscribe("/garagedoors/MIDDLE", [](const String & payload) 
  {
    //Serial.println(payload);
    if( payload == "0")
    {
      setLedArray(IDX_GAR_MID,0,50,0);
    }
    else
    {
      setLedArray(IDX_GAR_MID,50,0,0);
    }
    doorChange = true;
  });

  client.subscribe("/garagedoors/EAST", [](const String & payload) 
  {
    //Serial.println(payload);
    if( payload == "0")
    {
      setLedArray(IDX_GAR_EAST,0,50,0);
    }
    else
    {
      setLedArray(IDX_GAR_EAST,50,0,0);
    }
    doorChange = true;
  });


//  // Publish a message to "mytopic/test"
//  client.publish("mytopic/test", "This is a message"); // You can activate the retain flag by setting the third parameter to true

//  // Execute delayed instructions
//  client.executeDelayed(5 * 1000, []() 
//  {
//    client.publish("mytopic/wildcardtest/test123", "This is a message sent 5 seconds later");
//  });
}

void loop()
{
  client.loop();

  timerNow = millis(); // capture timer at this loop
  if( ((timerNow - ledTimeout) > 5000) || doorChange )
  {
    led_set(ledArray);
    ledTimeout = timerNow;
    doorChange = false;
  }
}
