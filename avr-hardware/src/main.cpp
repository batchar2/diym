
//#include <UIPEthernet.h>
#include <PubSubClient.h>

#include "description.h"

EthernetClient ethClient;
PubSubClient mqttClient;

extern void call_detect();

//uint8_t adc_buffer[ADC_BUFFER_SIZE] = {0};
uint8_t adc_buffer_length = 0;

//uint8_t dac_buffer[DAC_BUFFER_SIZE] = {0};
//uint8_t dac_buffer_length = 0;
// СОСТОЯНИЕ устойства. Устройиство может быть только в определенном состоянии
// Переключение осуществляется команддой сервера, либо началом/окончание вызова
uint8_t dv_state = DV_STATE_INIT;

// Отлов прерываний от АЦП
ISR(ADC_vect)
{
    static uint8_t value = 0;
    if (dv_state == DV_STATE_UP_PHONE) {
        //DEBUG("ISR");
        if (adc_buffer_length < ADC_BUFFER_SIZE) {
            value = ADCH;  // read 8 bit value from ADC
            //adc_buffer[adc_buffer_length++] = value;
        } else {
            //DEBUG("ISR");
            //mqttClient.publish("/dm/adc_sample", adc_buffer, adc_buffer_length, true);
            adc_buffer_length = 0;
        }
    }
}

void publish_topic(char* topic, char *value)
{
    if (mqttClient.state() != MQTT_CONNECTED){
        reconnect_mqtt();
    }
    mqttClient.publish(topic, value, true);
}

/*
void dvPinOut(int pin)
{
    DDRB |= (1<<pin);       // устанавливаем вывод PB5 как выход
}

void dvPinLow(int pin)
{
    PORTB &= ~(1<<pin);
}

void dvPinHigh(int pin)
{
    PORTB |= (1<<pin);
}
*/

void setup()
{
    Serial.begin(DEBUG_SERIAL_BOUDRATE);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
    }

//    dvPinOut(LED_MQTT_RECIVE);
//    dvPinOut(CALL_DETECT_PIN);
    pinMode(LED_MQTT_RECIVE, OUTPUT);
    pinMode(CALL_DETECT_PIN, INPUT);

    DEBUG(F("Start device"));

    attachInterrupt (0, call_detect, CHANGE);

    DEBUG(F("Settings mqtt"));
    // setup mqtt client
    mqttClient.setClient(ethClient);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(callback_mqtt);

    // Настраиваю прерывания для ADC
    ADCSRA = 0;             // clear ADCSRA register
    ADCSRB = 0;             // clear ADCSRB register
    ADMUX |= (0 & 0x07);    // set A0 analog input pin
    ADMUX |= (1 << REFS0);  // set reference voltage
    ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 9.6KHz

    ADCSRA |= (1 << ADATE); // enable auto trigger
    ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
    ADCSRA |= (1 << ADEN);  // enable ADC
    ADCSRA |= (1 << ADSC);  // start ADC measurements
}

static const char STATE_NAME_CALL[] = "wait_call";
static const char STATE_NAME_SEND_NOTIFY_CALL[] = "call";
static const char STATE_NAME_WAIT_USER[] = "wait_user";
static const char STATE_NAME_UP_PHONE[] = "up_phone";
static const char STATE_NAME_DOWN_PHONE[] = "down_phone";
static const char STATE_NAME_OPEN_DOR[] = "open_dor";

void change_state_device()
{
    switch(dv_state) {
        case DV_STATE_WAIT:
            // Привожу все пины в исходное СОСТОЯНИЕ
            //dvPinLow(LED_MQTT_RECIVE);
            digitalWrite(LED_MQTT_RECIVE, LOW);
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_CALL);
            break;
        // Отправка уведомления пользователю о вызове
        case DV_STATE_CALL_SEND_NOTIFY:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_SEND_NOTIFY_CALL);
            break;
        // Пользователь подтвердил получение уведомления
        case DV_STATE_WAIT_USER_ACTION:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_WAIT_USER);
            break;
        // Пользователь снял трубку
        case DV_STATE_UP_PHONE:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_UP_PHONE);
            //dvPinHigh(LED_MQTT_RECIVE);
            digitalWrite(LED_MQTT_RECIVE, HIGH);
            break;
        // Пользователь кладет трубку
        case DV_STATE_DOWN_PHONE:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_DOWN_PHONE);
            //dvPinLow(LED_MQTT_RECIVE);
            digitalWrite(LED_MQTT_RECIVE, LOW);
            break;
        case DV_STATE_OPEN_DOR:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_OPEN_DOR);
            break;
    }
}

void loop()
{
    static long previous_time = 0;
    //DEBUG("LOOP");
    if (mqttClient.state() != MQTT_CONNECTED) {
        dv_state = DV_STATE_INIT;
        reconnect_mqtt();
        dv_state = DV_STATE_WAIT;

        call_detect();
    }
    mqttClient.loop();


    long current_time = millis();
    if (current_time - previous_time > INTERVAL_STATE_CHANGE) {
        previous_time = current_time;
        change_state_device();
    }



}

void call_detect()
{
    uint8_t is_call = 0;
    // Смотрим, что событие произошло на интересующем нас пине
    if ((is_call=digitalRead(CALL_DETECT_PIN)) > 0) {
        // Если вызов только начался
        if (dv_state == DV_STATE_WAIT) {
            DEBUG(F("SET STATE CALL"));
            dv_state = DV_STATE_CALL_SEND_NOTIFY;
        }
    // Если вызов закончился
    } else {
        dv_state = DV_STATE_WAIT;
    }
}






/*

#include <Arduino_FreeRTOS.h>

// Определим две задачи для алгоритмов Blink и AnalogRead:
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );

// Функция setup запустится один раз, когда Вы нажмете кнопку
// сброса или подадите питание на плату.
void setup() {
    Serial.begin(115200);
   // Теперь создадим две задачи, чтобы они работали независимо
   // друг от друга:
   xTaskCreate(
      TaskBlink
      ,  (const portCHAR *)"Blink"  // Это просто любое имя, удобное
                                    // для чтения человеком.
      ,  128                        // Размер стека задачи
      ,  NULL
      ,  2                          // Приоритет задачи.
      ,  NULL );

   xTaskCreate(
      TaskAnalogRead
      ,  (const portCHAR *) "AnalogRead"
      ,  128                        // Этот размер стека может быть проверен
                                    // и подстроен путем чтения Highwater.
      ,  NULL
      ,  1                          // Приоритет задачи.
      ,  NULL );

   // Теперь автоматически и неявно для пользователя запустится scheduler,
   // который возьмет на себя управление планированием запуска отдельных задач.
}

void loop()
{
}


void TaskBlink(void *pvParameters)
{
  (void) pvParameters;

   // Инициализация цифрового вывода 13 в режиме выхода.
   pinMode(8, OUTPUT);

   for (;;) // A Task shall never return or exit.
   {
      digitalWrite(8, HIGH);    // включение светодиода LED
      vTaskDelay( 1000 / portTICK_PERIOD_MS ); // ожидание в 1 секунду
      digitalWrite(8, LOW);     // включение светодиода LED
      vTaskDelay( 1000 / portTICK_PERIOD_MS ); // ожидание в 1 секунду
   }
}

void TaskAnalogRead(void *pvParameters)
{
  (void) pvParameters;

   // Инициализация последовательного обмена данными на скорости
   // 9600 бит в секунду:


   for (;;)
   {
      // Чтение входа аналогового вывода 0:
      int sensorValue = analogRead(A0);
      // Вывод на печать прочитанного значения:
      DEBUG(sensorValue);
      // Задержка в 1 тик (15 мс) между чтениями, для стабильности:
      vTaskDelay(1);
   }
}
*/
























/*
#include "description.h"
#define CLIENT_ID       "UnoMQTT"
#define INTERVAL        10 // 3 sec delay between publishing
//#define DHTPIN          3
//#define DHTTYPE         DHT11
bool statusKD=HIGH;//living room door
bool statusBD=HIGH;//front door
bool statusGD=HIGH;//garage door
int lichtstatus;
uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

EthernetClient ethClient;
PubSubClient mqttClient;
//DHT dht(DHTPIN, DHTTYPE);

long previousMillis;

void setup() {
pinMode(4,INPUT_PULLUP);
pinMode(5,INPUT_PULLUP);
pinMode(6,INPUT_PULLUP);
  // setup serial communication
  Serial.begin(115200);

  DEBUG("START");
  // setup ethernet communication using DHCP
  if(Ethernet.begin(mac) == 0) {
    DEBUG(F("Ethernet configuration using DHCP failed"));
    for(;;);
  }
  // setup mqtt client
  mqttClient.setClient(ethClient);
  mqttClient.setServer("192.168.0.102", 1883);
  //mqttClient.setServer("192.168.1.xxx",1883); //for using local broker
  //mqttClient.setServer("broker.hivemq.com",1883);
  DEBUG(F("MQTT client configured"));
  mqttClient.connect(CLIENT_ID, "domophone", "domophone");
  // setup DHT sensor
  //dht.begin();
  previousMillis = millis();
}
void sendData() ;
void loop() {
  statusBD=digitalRead(4);
  statusGD=digitalRead(5);
  statusKD=digitalRead(6);
  lichtstatus = analogRead(A0);
  // check interval
  if(millis() - previousMillis > INTERVAL) {
    sendData();
    previousMillis = millis();
  }
  mqttClient.loop();
}

void sendData() {
  char msgBuffer[20] = "HELLO!!!!!";
  float h=100;//dht.readHumidity();
  float t = -100;//dht.readTemperature();
  //if(mqttClient.connect(CLIENT_ID, "domophone", "domophone")) {
   mqttClient.publish("hal/temp", dtostrf(t, 6, 2, msgBuffer));
   mqttClient.publish("hal/humid", dtostrf(h, 6, 2, msgBuffer));
   mqttClient.publish("hal/door", (statusBD == HIGH) ? "OPEN" : "DICHT");
   mqttClient.publish("hal/garage",(statusGD == HIGH) ? "OPEN" : "DICHT");
   mqttClient.publish("hal/kamer",(statusKD == HIGH) ? "OPEN" : "DICHT");
   mqttClient.publish("hal/licht", dtostrf(lichtstatus, 4, 0, msgBuffer));


   DEBUG("SEND");
 //hal=hallway, DICHT=Closed, kamer=room, licht=light
 //}
}
*/



































/*
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
    pinMode(LED_CONECTION_MQTT, OUTPUT);
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


    long now = millis();
    if (now - lastMsg > 1000) {
        lastMsg = now;
        ++value;
        snprintf (msg, 75, "#%ld - hello datahello datahello ", value);
        DEBUG("Publish message: ");
        //DEBUG(msg);
        MQTTClient.publish("outTopic", msg, true);
    }

}


*/
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
  DEBUG();

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
    DEBUG("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    DEBUG(WIFI_SSID);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  // you're connected now, so print out the data
  DEBUG("You're connected to the network");

  //printWifiStatus();

  DEBUG();
  DEBUG("Starting connection to server...");
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
      DEBUG("connected");
      DEBUG(client_id);
      // Once connected, publish an announcement...
      client.publish("/dm/status", "hello world", true);
      // ... and resubscribe
      client.subscribe("/dm/settings");

      DEBUG("connect settings");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      DEBUG(" try again in 5 seconds");
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
    DEBUG(msg);
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
DEBUG();
Serial.print("Connecting to ");
DEBUG(ssid);

WiFi.begin(ssid, password);

while (WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
}

DEBUG("");
DEBUG("WiFi connected");
DEBUG("IP address: ");
DEBUG(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
Serial.print("Message arrived [");
Serial.print(topic);
Serial.print("] ");
for (int i = 0; i < length; i++) {
  Serial.print((char)payload[i]);
}
DEBUG();

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
    DEBUG("connected");
    // Once connected, publish an announcement...
    client.publish("outTopic", "hello world");
    // ... and resubscribe
    client.subscribe("inTopic");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    DEBUG(" try again in 5 seconds");
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
  DEBUG(msg);
  client.publish("outTopic", msg);
}
}
*/
