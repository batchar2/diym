#pragma once

#include <Arduino.h>

#include <WiFiEsp.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#ifndef HAVE_HWesp_serial
  #include <SoftwareSerial.h>
#endif


#define STATUS_OK               (0)
#define STATUS_ERR              (1)

// Дебаг через программный UART
extern SoftwareSerial SerialDebug;
// Сотояние соединения с WiFi
extern int WiFiStatus;

extern PubSubClient MQTTClient;

#define DEBUG(m)                    (SerialDebug.println(m))
//#define DEBUG(m)                    (Serial.println(m))

#define DEBUG_SERIAL_BOUDRATE       9600
#define ESP_SERIAL_BOUDRATE         9600

#define WIFI_SSID                   "TP-LINK_664E"
#define WIFI_PASS                   "31238342"

#define MQTT_SERVER                 "192.168.0.102"
#define MQTT_PORT                   1883

#define MQTT_USERNAME               "domophone"
#define MQTT_PASSWORD               "domophone"


#define RX_DBG_PIN                  7
#define TX_DBG_PIN                  6


// Подключились к Wifi
#define LED_CONNECTION_WIFI         13

extern SoftwareSerial SerialESP;


extern int connect_wifi();
extern void callback_mqtt(char* topic, byte* payload, unsigned int length);
extern void reconnect_mqtt();
