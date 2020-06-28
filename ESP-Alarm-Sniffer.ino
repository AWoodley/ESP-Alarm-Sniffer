/*
*/

#define PubSubClient_MAX_PACKET_SIZE 512;

//#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include "secrets.h"

void command(char *topic, byte *payload, unsigned int length);
ICACHE_RAM_ATTR void alarmed();
ICACHE_RAM_ATTR void clockIn();
ICACHE_RAM_ATTR void cleared();


const int ON = 1;
const int OFF = 0;

const int armPin = D8;
const int alarmPin = D1;
const int clockPin = D5;
const int dataPin = D7;

const String wifiSSID = SECRETS_WIFI_SSID;
const String wifiPassword = SECRETS_WIFI_PASSWORD;

IPAddress staticIP(192, 168, 29, 181);
IPAddress gateway(192, 168, 29, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 29, 254);

const String mqttServer = "192.168.29.2";
const String mqttUser = SECRETS_MQTT_USERNAME;
const String mqttPassword = SECRETS_MQTT_PASSWORD;
const String mqttID = "Alarm3";

#define mqttDiscoveryPrefix "homeassistant"
#define mqttHomePrefix "home"
#define mqttAlarm "alarm"
#define mqttAlarmZone "zones"

const String DISARM = "DISARM";
const String ARM_AWAY = "ARM_AWAY";

const long update_interval = 60000; // 60 seconds

const long debounce = 25; // 25uSec
const long clock_time = 10000; // 10milliSec
const long ON_LED_time = 2000; // 2sec
const long alarm_flash_time = 2000; // 2sec

// And a sensor for WiFi signal strength
const String mqttDiscoSignalStateTopic = mqttHomePrefix "/sensor/" mqttAlarm "-signal/state";
const String mqttDiscoSignalConfigTopic = mqttDiscoveryPrefix "/sensor/" mqttAlarm "-signal/config";
// And a sensor for device uptime
const String mqttDiscoUptimeStateTopic = mqttHomePrefix "/sensor/" mqttAlarm "-uptime/state";
const String mqttDiscoUptimeConfigTopic = mqttDiscoveryPrefix "/sensor/" mqttAlarm "-uptime/config";

// The strings below will spill over the PubSubClient_MAX_PACKET_SIZE 128
// You'll need to manually set MQTT_MAX_PACKET_SIZE in PubSubClient.h to 512
const String mqttDiscoSignalConfigPayload = "{\"name\": \"" mqttAlarm "-signal\", \"state_topic\": \"" + mqttDiscoSignalStateTopic + "\", \"unit_of_measurement\": \"dBm\", \"value_template\": \"{{ value }}\"}";
const String mqttDiscoUptimeConfigPayload = "{\"name\": \"" mqttAlarm "-uptime\", \"state_topic\": \"" + mqttDiscoUptimeStateTopic + "\", \"unit_of_measurement\": \"msec\", \"value_template\": \"{{ value }}\"}";

const String mqttDiscoAlarmStateTopic = mqttHomePrefix "/" mqttAlarm;
// const String mqttDiscoAlarmConfigTopic = mqttDiscoveryPrefix "/" mqttAlarm "/config" ;
const String mqttDiscoAlarmCommandTopic = mqttHomePrefix "/" mqttAlarm "/set";
const String mqttDiscoAlarmZonesStateTopic = mqttHomePrefix "/" mqttAlarm "/zones";

// const String mqttDiscoAlarmConfigPaylaoad = "{\"name:\" \"" mqttAlarm "\", \"state_topic\": \"" + mqttDiscoAlarmStateTopic + "\", "

namespace Alarm
{
    enum State
    {
        UNSET = 0,
        DISARMED,
        PENDING,
        ARMED_AWAY,
        ARMED_HOME,
        TRIGGERED
    };
    const String payload[] = {"", "disarmed", "pending", "armed_away", "armed_home", "triggered"};
    const String stateTopic = mqttHomePrefix "/" mqttAlarm;
    const String commandTopic = mqttHomePrefix "/" mqttAlarm "/set";
    // const String configTopic = mqttDiscoveryPrefix "/" mqttAlarm "/config";
    // const String availabilityTopic = mqttDiscoveryPrefix "/" mqttAlarm "/status";
    const int LEDMask = 1024;
    State state = UNSET;
    State requested = UNSET;
} // namespace Alarm

namespace Zone
{
    enum State
    {
        UNSET = 0,
        ON,
        OFF
    };
    const String payload[] = { "", "ON", "OFF"};
    const String stateTopic = mqttHomePrefix "/" mqttAlarm "/" mqttAlarmZone;
    State state = UNSET;
}

const int NumZones = 8;

Zone::State ZoneStates[NumZones];

//namespace FrontDoor
//{
//    enum State
//    {
//        UNSET = 0,
//        OPEN,
//        CLOSED
//    };
//    const String payload[] = {"", "ON", "OFF"};
//    const String stateTopic = mqttDiscoveryPrefix "/entry/door";
//    const int LEDMask = 1;
//    State state = UNSET;
//    
//} // namespace FrontDoor
//
//namespace LaundryDoor
//{
//    enum State
//    {
//        UNSET = 0,
//        OPEN,
//        CLOSED
//    };
//    const String payload[] = {"", "ON", "OFF"};
//    const String stateTopic = mqttDiscoveryPrefix "/laundry/door";
//    const int LEDMask = 32;
//    State state = UNSET;
//} // namespace LaundryDoor

// Set the signal strength and uptime reporting interval in milliseconds
const unsigned long reportInterval = 5000;
unsigned long reportTimer = millis();

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

long lastMsg = 0;
char msg[50];
int value = 0;

int temp_data = 0;
int shift = 0;

int data = 0;

long last_clock = 0;
long last_watchdog = 0;

bool ON_LED_last_state = 0;
long ON_LED_timer = 0;

long alarm_cleared_timer = 0;

char bits_str[17];
char status_str[100];

long last_update = 0;

bool state_changed = false;

void setup_wifi()
{
//    Serial.print("setup_wifi() ");

    delay(10);

    WiFi.config(staticIP, dns, gateway, subnet);
    WiFi.begin(wifiSSID, wifiPassword);
    WiFi.mode(WIFI_STA);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
//    Serial.println("done");
}

void reconnect()
{
//    Serial.print("reconnect() ");
    // Loop until we're reconnected
    while (!mqttClient.connected())
    {
        if (mqttClient.connect(mqttID.c_str(), mqttUser.c_str(), mqttPassword.c_str()))
        {
            mqttClient.subscribe(Alarm::commandTopic.c_str());
        }
        else
        {
            delay(5000);
        }
    }
    
//    Serial.println("done");
}

void arm()
{
    EEPROM.write(0, Alarm::state); // Write state to EEPROM
    EEPROM.commit();
    digitalWrite(armPin, LOW);
    delay(500);
    digitalWrite(armPin, HIGH);
    Alarm::requested = Alarm::ARMED_AWAY;
}

void disarm()
{
    EEPROM.write(0, Alarm::state); // Write state to EEPROM
    EEPROM.commit();
    digitalWrite(armPin, HIGH);
    delay(500);
    digitalWrite(armPin, LOW);
    Alarm::requested = Alarm::DISARMED;
}

void command(char *topic, byte *payload, unsigned int length)
{
    if (strlen(DISARM.c_str()) == length && memcmp(payload, DISARM.c_str(), length) == 0)
    {
        disarm();
    }
    else if (strlen(ARM_AWAY.c_str()) == length && memcmp(payload, ARM_AWAY.c_str(), length) == 0)
    {
        arm();
    }
}

void checkAlarm()
{
    if (Alarm::state == Alarm::TRIGGERED && 
        digitalRead(alarmPin) == LOW &&
        millis() - alarm_cleared_timer > alarm_flash_time)
    {
      Alarm::state = Alarm::requested;
      state_changed = true;
    }
}

void alarmed()
{
    Alarm::state = Alarm::TRIGGERED;
    state_changed = true;
    attachInterrupt(digitalPinToInterrupt(alarmPin), cleared, FALLING);
}

void cleared()
{
  alarm_cleared_timer = millis();
  attachInterrupt(digitalPinToInterrupt(alarmPin), alarmed, RISING);
}

void clockIn() {
  long now = micros();
  long delta = now - last_clock;
  if (delta > debounce) {
    if (delta > clock_time) {
      shift = 0;
      temp_data = 0;
    }
    last_clock = now;
    temp_data = temp_data | (digitalRead(dataPin) << shift);
    shift++;
    if (shift == 16) {
      data = temp_data;
//      client.publish(test_topic, String(data).c_str(), false);
    }
  }
}

void onLED()
{
    if (Alarm::state != Alarm::TRIGGERED)
    {
        if (data & Alarm::LEDMask)
        { //ON LED on
            if (ON_LED_last_state == OFF)
            {                            // LED was OFF previous
                ON_LED_timer = millis(); // start timer
                ON_LED_last_state = ON;  // set new state
            }
            else
            { // LED was ON previous
                if (millis() - ON_LED_timer < ON_LED_time)
                { // on less than 2 seconds
                    if (Alarm::state != Alarm::PENDING)
                    { // longer than update interval
                        Alarm::state = Alarm::PENDING;
                        state_changed = true;
                    }
                }
                else
                { //on more than 2 seconds
                    if (Alarm::state != Alarm::ARMED_AWAY)
                    { // longer than update interval
                        Alarm::state = Alarm::ARMED_AWAY;
                        state_changed = true;
                    }
                }
            }
        }
        else
        { //ON LED off
            if (ON_LED_last_state == ON)
            { //ON LED was ON previous
                ON_LED_timer = millis();
                ON_LED_last_state = OFF;
            }
            else
            { // LED was OFF previous
                if (millis() - ON_LED_timer > ON_LED_time)
                { // off more than 2 seconds
                    if (Alarm::state != Alarm::DISARMED)
                    { // longer than update interval
                        Alarm::state = Alarm::DISARMED;
                        state_changed = true;
                    }
                }
            }
        }
    }
}

void zones()
{
    int zone = 0;
    int LEDMask = 0;
    if (Alarm::state != Alarm::TRIGGERED) // Quick'n'dirty to avoid flickering zones when alarming
    {
      for (zone = 0; zone < NumZones; zone++)
      {
          LEDMask = 1 << zone;
          if (data & LEDMask)
          {
              if (ZoneStates[zone] != Zone::ON)
              {
                  ZoneStates[zone] = Zone::ON;
                  state_changed = true;
              }
          }
          else
          {
              if (ZoneStates[zone] != Zone::OFF)
              {
                  ZoneStates[zone] = Zone::OFF;
                  state_changed = true;
              }
          }
      }
   }
}

// void frontDoorLED()
// {
//     if (data & FrontDoor::LEDMask)
//     { //Door LED on
//         if (FrontDoor::state != FrontDoor::OPEN)
//         { // state changed
//             FrontDoor::state = FrontDoor::OPEN;
//             state_changed = true;
//         }
//     }
//     else
//     { //Door LED off
//         if (FrontDoor::state != FrontDoor::CLOSED)
//         { // state changed
//             FrontDoor::state = FrontDoor::CLOSED;
//             state_changed = true;
//         }
//     }
// }

// void laundryDoorLED()
// {
//     if (data & LaundryDoor::LEDMask)
//     { //Door LED on
//         if (LaundryDoor::state != LaundryDoor::OPEN)
//         { // state changed
//             LaundryDoor::state = LaundryDoor::OPEN;
//             state_changed = true;
//         }
//     }
//     else
//     { //Door LED off
//         if (LaundryDoor::state != LaundryDoor::CLOSED)
//         { // state changed
//             LaundryDoor::state = LaundryDoor::CLOSED;
//             state_changed = true;
//         }
//     }
// }

// void watchdog() {
//   unsigned long currentMillis = millis();
//   if (currentMillis - last_watchdog >= watchdog_interval) {
//     char watchdog_topic[strlen(watchdog_topic_base) + 16];
//     strcpy(watchdog_topic, watchdog_topic_base);
//     strcat(watchdog_topic, WiFi.localIP().toString().c_str());

//     mqttClient.publish(watchdog_topic, String(currentMillis).c_str(), false);
//     last_watchdog = currentMillis;
//   }
// }

void setup()
{
    Serial.begin(115200);

//    Serial.print("setup() ");
    EEPROM.begin(512);              // Begin eeprom to store on/off state
    pinMode(armPin, OUTPUT);         // Initialize the relay pin as an output
    pinMode(alarmPin, INPUT_PULLUP); // Initialize the relay pin as an output
    pinMode(clockPin, INPUT_PULLUP); // Initialize the relay pin as an output
    pinMode(dataPin, INPUT_PULLUP); // Initialize the relay pin as an output
    Alarm::requested = (Alarm::State)EEPROM.read(0);
    if (Alarm::requested == Alarm::ARMED_AWAY)
    {
        arm();
    }
    else
    {
        disarm();
    }

    attachInterrupt(digitalPinToInterrupt(clockPin), clockIn, FALLING);
    attachInterrupt(digitalPinToInterrupt(alarmPin), alarmed, RISING);
//    attachInterrupt(digitalPinToInterrupt(alarmPin), cleared, FALLING);

    setup_wifi(); // Connect to wifi
    ArduinoOTA.begin();
    mqttClient.setServer(mqttServer.c_str(), 1883);
    mqttClient.setCallback(command);
    
//    Serial.println("done");
}

void loop()
{
    if (!mqttClient.connected())
    {
        reconnect();
    }

    onLED();

    zones();

    checkAlarm();

    if (state_changed || millis() - last_update > update_interval) {
//        mqttClient.publish("test/test", String(data).c_str(), false);
        mqttClient.publish(Alarm::stateTopic.c_str(), Alarm::payload[Alarm::state].c_str(), false);
        int zone = 0;
        for (zone = 0; zone < NumZones; zone++)
        {
            String zoneTopic = Zone::stateTopic + "/" + String(zone).c_str();
            mqttClient.publish(zoneTopic.c_str(), Zone::payload[ZoneStates[zone]].c_str(), false);
        }

        last_update = millis();
        state_changed = false;
//        mqttClient.publish("test/alarm", String(digitalRead(alarmPin)).c_str(), false);
//        mqttClient.publish("test/alarm", String(Alarm::state).c_str(), false);
    }

    ArduinoOTA.handle();

    mqttClient.loop();
//    Serial.println("loop()");
}
