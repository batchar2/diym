#include "description.h"


// 1793
// 1727
// 1723
// 1717
// 1707
// 1633
// 1729
// 1713
// 11684

// Пользователь дал указание - я получил вызов, будем решать что с ним делать
#define MQTT_ANSWER_USER_NOTIFY             "user_notify"
// Пользователь дал указание - поднять трубку
#define MQTT_ANSWER_UP_PHONE                "up_phone"
// Пользовтаель дал указаление - положить трубку
#define MQTT_ANSWER_DOWN_PHONE              "down_phone"
// Пользователь желает открыть дверь
#define MQTT_ANSWER_OPEN_DOR                "open_dor"

const uint8_t PROGMEM mac[6] = {0x51, 0x77, 0x02, 0x03, 0x04, 0x05};

/*
void restart_net()
{
    DEBUG(F("recon net"));
    // setup ethernet communication using DHCP
    while (Ethernet.begin(mac) == 0) {
        DEBUG(F("DHCP failed"));
    }
    DEBUG(F("recon net success!"));
}
*/
void callback_mqtt(char* topic, byte* payload, unsigned int length)
{
    DEBUG(F(">>>"));
    if (length >= MQTT_MAX_PAYLOAD_LENGTH) {
        return;
    }
    //return;
    //DEBUG(data);
    // отлов событий от топика, через который осуществляется управление
    if (strstr(MQTT_TOPIC_SUB_CONTROL_STATE, topic) != NULL) {
        // Тут как правило маленькие данные, для экономии памяти пришлось пойти на это
        char data[length] = {0};
        memcpy(data, payload, length);
        data[length] = '\0';
        // Пользователь получил уведомление о вызове
        if (strstr(MQTT_ANSWER_USER_NOTIFY, data) != NULL) {
            dv_state = DV_STATE_WAIT_USER_ACTION;
        // Пользователь желает взять трубку
        } else if (strstr(MQTT_ANSWER_UP_PHONE, data) != NULL) {
            dv_state = DV_STATE_UP_PHONE;
        // Пользовтаель желает положить трубку
        } else if (strstr(MQTT_ANSWER_DOWN_PHONE, data) != NULL) {
            dv_state = DV_STATE_DOWN_PHONE;
        // Пользователь желает открыть дверь
        } else if (strstr(MQTT_ANSWER_OPEN_DOR, data) != NULL) {
            dv_state = DV_STATE_OPEN_DOR;
        }
    // Пришел звук
    } else if (strstr(MQTT_TOPIC_SUB_CONTROL_VOICE, topic) != NULL) {
        cli();
        dac_buffer_length = length;
        memcpy(dac_buffer, payload, length);
        sei();
    }
}

void reconnect_mqtt()
{
    // Отписываюсь от топика
    //restart_net();
    while (!mqttClient.connected()) {
        DEBUG(F("recon MQTT"));
        char client_id[15] = {0};
        sprintf(client_id, "a-%d", random(0xffff));

        if (mqttClient.connect(client_id, MQTT_USERNAME, MQTT_PASSWORD)) {
            DEBUG(F("connected MQTT server....."));
            mqttClient.subscribe(MQTT_TOPIC_SUB_CONTROL_STATE, MQTT_QOS);
            //mqttClient.subscribe(MQTT_TOPIC_SUB_CONTROL_STATE, MQTT_QOS);
            mqttClient.subscribe(MQTT_TOPIC_SUB_CONTROL_VOICE);
        } else {
            Serial.print(F("failed connect"));
        }
    }
    //call_detect();
}
