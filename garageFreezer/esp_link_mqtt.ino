/**
 * Simple example to demo the esp-link MQTT client
 */

#include <ELClient.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>

// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
//HardwareSerial Serial2(USART2);   // PA3  (RX)  PA2  (TX)

ELClient esp(&Serial, &Serial);

// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins
void wifiCb(void* response) 
{
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) 
  {
    uint8_t status;
    res->popArg(&status, 1);

    if(status == STATION_GOT_IP) 
    {
      Serial.println("WIFI CONNECTED");
    } else 
    {
      Serial.print("WIFI NOT READY: ");
      Serial.println(status);
    }
  }
}

bool connected = false;
static uint32_t timeoutOne;

void esplink_loop() 
{
  esp.Process();

//  uint32_t now = millis();
//  char buf[12];
//  if (connected && (now - timeoutOne) > 10000) 
//  {
//    //itoa(frontDoorState, buf, 10);
//    mqtt.publish("/garage/freezer/TEMP", buf);
//
//    timeoutOne = now;
//  }
}

void esplink_update(int changeFlag) 
{
  esp.Process();
  char buf[12];

  if (connected && (0 != changeFlag)) 
  {
    dtostrf(tempAverage, 7, 2, buf); // size for -199.99
    mqtt.publish("/garage/freezer/TEMP", buf);
  }
}

// Callback when MQTT is connected
void mqttConnected(void* response) 
{
  Serial.println("MQTT connected!");
  mqtt.subscribe("/garage/freezer/GET");
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) 
{
  Serial.println("MQTT disconnected");
  connected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) 
{
  ELClientResponse *res = (ELClientResponse *)response;

  String topic = res->popString();
  //Serial.print("Received: topic=");
  //Serial.println(topic);

  String data = res->popString();
  //Serial.print("data=");
  //Serial.println(data);

  if( topic == "/garage/freezer/GET")
  {
    esplink_update(1);
  }
}

void mqttPublished(void* response) 
{
  //Serial.println("MQTT published");
}

bool check_esp_link_sync()
{
  return esp.Sync();
}

void esplink_setup1() 
{
  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
}

void esplink_setup2() 
{
  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();

  //Serial.println("EL-MQTT ready");
  timeoutOne = millis();  
}

