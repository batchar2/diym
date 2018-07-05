#pragma once
#include <Arduino.h>

// Состояние работы устройства
extern uint8_t dv_state;
extern uint8_t dac_buffer[];
extern uint8_t volatile dac_buffer_length;

#define INTERVAL_STATE_CHANGE           1000

#define TRUE                            1
#define FALSE                           0

#define STATUS_OK                       0
#define STATUS_ERR                      -1

#define ADC_BUFFER_SIZE                 120
#define DAC_BUFFER_SIZE                 120

// Перечислены основные состояния трубки
// Трубка сама ничего не решает, принятие решения происходит на сервере

// Подготовка
#define DV_STATE_INIT                   -1
// Ожидание
#define DV_STATE_WAIT                   0
// Пошел вызов, ждем действия пользователя
#define DV_STATE_CALL_SEND_NOTIFY       1
// Пользователь отвечает - уведомление о вызове принято
#define DV_STATE_WAIT_USER_ACTION       2
// Снять турбку, пошел режим разговора
#define DV_STATE_UP_PHONE               3
// открыть дверь
#define DV_STATE_OPEN_DOR               4
// Положить трубку
#define DV_STATE_DOWN_PHONE             5

#define DEBUG(m)                        (Serial.println(m))

#define DEBUG_SERIAL_BOUDRATE           115200

// У принимаемого аудио частота дискретизации
#define RECV_AUDIO_SAMPLE_RATE          (8000)

// Мозгоебучий баг! Время жизни соединения
//#define MQTT_KEEPALIVE                  65000
#define MQTT_SERVER                     "192.168.0.102"
#define MQTT_PORT                       1883

#define MQTT_USERNAME                   "domophone"
#define MQTT_PASSWORD                   "domophone"
#define MQTT_QOS                        (0)
//#define MQTT_MAX_PACKET_SIZE            250
#define MQTT_MAX_PAYLOAD_LENGTH         180

// Топик, в который публикуется изменение состояния устройства
#define MQTT_TOPIC_PUB_DEVICE_STATE     "/dm/state/device"
// Топик, для управления устройством
#define MQTT_TOPIC_SUB_CONTROL_STATE    "/dm/state/control"
// Устройство публикует аудиосэмплы в этот топик
#define MQTT_TOPIC_PUB_DEVICE_VOICE     "/dm/state/device_voice"
// Устройство вычитывает аудиосэмплы опубликованные в этом топике программой управления
#define MQTT_TOPIC_SUB_CONTROL_VOICE    "/dm/state/control_voice"

// Взять трбку (реле)
#define PIN_OUT_OPEN_PHONE              8
// Открыть дверь (Реле)
#define PIN_OUT_OPEN_DOR                7
// Определение вызова
#define PIN_IN_IS_CALL                  12
// ШИМ-звук
//#define PIN_OUT_PLAY_AUDIO              5



#define ESP_SERIAL_BOUDRATE             230400

#define WIFI_SSID                       "TP-LINK_664E"
#define WIFI_PASS                       "31238342"



extern int connect_ethernet();
//extern int connect_wifi();
extern void callback_mqtt(char* topic, byte* payload, unsigned int length);
extern void reconnect_mqtt();
extern void restart_net();
extern void call_detect();

//#include <UIPEthernet.h>

#include <WiFiEsp.h>
#include <PubSubClient.h>

//extern EthernetClient       ethClient;
extern PubSubClient  mqttClient;
extern int dvWiFiStatus;
extern WiFiEspClient espSerialClient;
