#include <WiFiEsp.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>


#include "description.h"

SoftwareSerial SerialDebug(RX_DBG_PIN, TX_DBG_PIN); // RX, TX

// Статус соединения с сервером
int WiFiStatus = WL_IDLE_STATUS;

// Serial-client для соединения с ESP8266
WiFiEspClient EspClient;
// Публикатор-подписчик на базе MQTT протокола
PubSubClient MQTTClient(EspClient);


void setup()
{
    Serial.begin(ESP_SERIAL_BOUDRATE);
    SerialDebug.begin(DEBUG_SERIAL_BOUDRATE);

    pinMode(LED_CONNECTION_WIFI, OUTPUT);
    // Подключаемся к WiFi
    while (connect_wifi() != STATUS_OK);

    DEBUG("Starting connection to server...");
    MQTTClient.setServer(MQTT_SERVER, MQTT_PORT);
    MQTTClient.setCallback(callback_mqtt);
}


long lastMsg = 0;
int value = 0;
char msg[50];

void loop()
{
    if (!MQTTClient.connected()) {
        reconnect_mqtt();
    }
    MQTTClient.loop();


    //long now = millis();
    //if (now - lastMsg > 10000) {
    //    lastMsg = now;
    //    ++value;
    //    snprintf (msg, 75, "#%ld - hello", value);
    //    DEBUG("Publish message: ");
    //    DEBUG(msg);
    //    MQTTClient.publish("outTopic", msg, true);
    //}

}



/*
 WiFiEsp example: WebClient
 This sketch connects to google website using an ESP8266 module to
 perform a simple web search.
 For more details see: http://yaab-arduino.blogspot.com/p/wifiesp-example-client.html
*/
/*


#include "WiFiEsp.h"
#include "description.h"
#include <PubSubClient.h>


#define TX_ESP_PIN          6
#define RX_ESP_PIN          7
// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(RX_ESP_PIN, TX_ESP_PIN); // RX, TX
#endif

//char ssid[] = WIFI_SSID;            // your network SSID (name)
//char pass[] = "12345678";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char mqtt_server[] = "192.168.0.102";

// Initialize the Ethernet client object
WiFiEspClient espClient;

PubSubClient client(espClient);


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(13, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(13, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}



void setup()
{
  pinMode(13, OUTPUT);

  // initialize serial for debugging
  Serial.begin(9600);
  // initialize serial for ESP module
  Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");

  //printWifiStatus();

  Serial.println();
  Serial.println("Starting connection to server...");
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    char client_id[50] = {0};
    sprintf(client_id, "arduino-%d", random(0xffff));

    if (client.connect(client_id, "domophone", "domophone")) {
      Serial.println("connected");
      Serial.println(client_id);
      // Once connected, publish an announcement...
      client.publish("/dm/status", "hello world", true);
      // ... and resubscribe
      client.subscribe("/dm/settings");

      Serial.println("connect settings");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

long lastMsg = 0;
int value = 0;
char msg[50];

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 75, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg, true);
  }

}


*/






































/*

#include "description.h"


// Регистрация serial, который будет отвечать за связь с ESP8266
SoftwareSerial SerialESP(RX_ESP_PIN, TX_ESP_PIN);
WiFiEspClient espClient;

PubSubClient client(espClient);

int status = 0;

void connect_wifi()
{
  WiFi.init(&SerialESP);
  if (WiFi.status() == WL_NO_SHIELD) {
    DEBUG("WiFi shield not present");
    while (true);
  }
  while ( status != WL_CONNECTED) {
    DEBUG("Attempting to connect to WPA SSID: ");
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  DEBUG("You're connected to the network");
  // TODO зажечь светодиод!
  digitalWrite(LED_BUILTIN, HIGH);
}



void setup()
{
  Serial.begin(DEBUG_SERIAL_BOUDRATE);
  SerialESP.begin(ESP_SERIAL_BOUDRATE);

  connect_wifi();

  // initialize LED digital pin as an output.
  pinMode(LED_CONNECTION_WIFI, OUTPUT);


  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  //digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  //delay(1000);
  // turn the LED off by making the voltage LOW
  //digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  //delay(1000);

}

*/



















/*
Basic ESP8266 MQTT example

This sketch demonstrates the capabilities of the pubsub library in combination
with the ESP8266 board/library.

It connects to an MQTT server then:
- publishes "hello world" to the topic "outTopic" every two seconds
- subscribes to the topic "inTopic", printing out any messages
  it receives. NB - it assumes the received payloads are strings not binary
- If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
  else switch it off

It will reconnect to the server if the connection is lost using a blocking
reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
achieve the same result without blocking the main loop.

To install the ESP8266 board, (using Arduino 1.6.4+):
- Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
     http://arduino.esp8266.com/stable/package_esp8266com_index.json
- Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
- Select your ESP8266 in "Tools -> Board"



#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.

const char* ssid = "........";
const char* password = "........";
const char* mqtt_server = "broker.mqtt-dashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
Serial.begin(115200);
setup_wifi();
client.setServer(mqtt_server, 1883);
client.setCallback(callback);
}

void setup_wifi() {

delay(10);
// We start by connecting to a WiFi network
Serial.println();
Serial.print("Connecting to ");
Serial.println(ssid);

WiFi.begin(ssid, password);

while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
}

Serial.println("");
Serial.println("WiFi connected");
Serial.println("IP address: ");
Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
Serial.print("Message arrived [");
Serial.print(topic);
Serial.print("] ");
for (int i = 0; i < length; i++) {
  Serial.print((char)payload[i]);
}
Serial.println();

// Switch on the LED if an 1 was received as first character
if ((char)payload[0] == '1') {
  digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
  // but actually the LED is on; this is because
  // it is acive low on the ESP-01)
} else {
  digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
}

}

void reconnect() {
// Loop until we're reconnected
while (!client.connected()) {
  Serial.print("Attempting MQTT connection...");
  // Attempt to connect
  if (client.connect("ESP8266Client")) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    client.publish("outTopic", "hello world");
    // ... and resubscribe
    client.subscribe("inTopic");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
}
}
void loop() {

if (!client.connected()) {
  reconnect();
}
client.loop();

long now = millis();
if (now - lastMsg > 2000) {
  lastMsg = now;
  ++value;
  snprintf (msg, 75, "hello world #%ld", value);
  Serial.print("Publish message: ");
  Serial.println(msg);
  client.publish("outTopic", msg);
}
}
*/
