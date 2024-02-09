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

bool connected;

// Callback when MQTT is connected
void mqttConnected(void* response) 
{
  Serial.println("MQTT connected!");
  mqtt.subscribe("/doorlogger/FRONT");
  mqtt.subscribe("/doorlogger/BACK");
  mqtt.subscribe("/doorlogger/MUDROOM");
  mqtt.subscribe("/doorlogger/PIR");
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

  Serial.print("Received: topic=");
  String topic = res->popString();
  Serial.println(topic);

  Serial.print("data=");
  String data = res->popString();
  Serial.println(data);

  if( topic == "/doorlogger/FRONT")
  {
    doorStatusArray[IDX_FRONT] = (data == "0"?0:1);
  }
  if( topic == "/doorlogger/BACK")
  {
    doorStatusArray[IDX_BACK] = (data == "0"?0:1);
  }
  if( topic == "/doorlogger/MUDROOM")
  {
    doorStatusArray[IDX_MUDROOM] = (data == "0"?0:1);
  }
  if( topic == "/doorlogger/PIR")
  {
    doorStatusArray[IDX_PIR] = (data == "0"?0:1);
  }
}

void mqttPublished(void* response) 
{
  Serial.println("MQTT published");
}

void esplink_setup1() 
{
  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
}

bool check_esp_link_sync()
{
  Serial.println("Called link_sync");

  return esp.Sync();
}

static uint32_t timeoutOne;

void esplink_setup2() 
{
  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();
  Serial.println("EL-MQTT ready");
}

void esplink_loop() 
{
  esp.Process();
}

