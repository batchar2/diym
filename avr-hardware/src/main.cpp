
//#include <UIPEthernet.h>
//#include <PubSubClient.h>

#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "description.h"

//EthernetClient ethClient;
WiFiEspClient espSerialClient;
PubSubClient mqttClient(espSerialClient);

static uint8_t adc_buffer[ADC_BUFFER_SIZE] = {0};
static volatile uint8_t adc_buffer_length = 0;

uint8_t dac_buffer[DAC_BUFFER_SIZE] = {0};
uint8_t volatile dac_buffer_length = 0;

// СОСТОЯНИЕ устойства. Устройиство может быть только в определенном состоянии
// Переключение осуществляется команддой сервера, либо началом/окончание вызова
uint8_t dv_state = DV_STATE_INIT;
int dvWiFiStatus = WL_IDLE_STATUS;



/*
// Отлов прерываний от АЦП
ISR(ADC_vect)
{
    static uint8_t value = 0;
    if (dv_state == DV_STATE_UP_PHONE) {
        if (adc_buffer_length < ADC_BUFFER_SIZE) {
            value = ADCH;  // read 8 bit value from ADC
            adc_buffer[adc_buffer_length++] = value;
        }
    }
}
ISR(TIMER0_COMPA_vect)
{
    static long previous_time = 0;
    static int count = 0;
    if (dv_state != DV_STATE_WAIT && dv_state != DV_STATE_INIT && dv_state != DV_STATE_CALL_SEND_NOTIFY) {
        if (dac_buffer_length < DAC_BUFFER_SIZE) {
            //OCR1A = 0;
            //OCR1C = 12;
            //OCR1A = dac_buffer[dac_buffer_length++];
            //OCR1B = dac_buffer[dac_buffer_length++];
            //OCR1A = dac_buffer[dac_buffer_length++];
        }
    }
    long current_time = millis();
    if (current_time - previous_time > INTERVAL_STATE_CHANGE) {
        Serial.print("ISR >>> ");
        Serial.println(count);
        count = 0;
    }
    previous_time = current_time;
    count += 1;
    Serial.println("+");
}
*/

static void adc_init()
{
    // Настройки АЦП
    // Мультиплексирование АЦП
    ADMUX = 0;
    ADMUX |= (1 << REFS0); // опорное напряжение 5в (AVCC — источник опорного напряжения)
    ADMUX |= (1 << ADLAR); // выравниевание по левому краю
    ADMUX |= 0;//;(0 & 0x07); // Вход - A5 пин ардуинки
    // Регистр А статуса и управления АЦП
    // ADCSRA - используется для настройки работы модуля АЦП
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 9.6KHz
    ADCSRA |= (1 << ADATE); // enable auto trigger
    ADCSRA |= (1 << ADEN);  // Активирую АЦП
    ADCSRA |= (1 << ADIE);  // Разрешаю прерывания АЦП
    ADCSRA |= (1 << ADSC);  // Стартую АЦП
}


/*
static void dac_init()
{
    // http://reso-nance.org/wiki/logiciels/arduino-timer/accueil
    // http://www.instructables.com/id/Arduino-Timer-Interrupts/
    // https://toster.ru/q/409789
    // http://extremeelectronics.co.in/avr-tutorials/sound-generation-by-avr-micro-tutorial-i/
    //TCCR0B = 0b01011000 | 0x02;
    TCCR0B = TCCR0B & 0b11111000; //| 0x02;

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK0 |= _BV(OCIE1A);

    OCR1A = F_CPU / RECV_AUDIO_SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK0 |= _BV(OCIE1A);

    //TIMSK1 |= _BV(OCIE1A);
    //orig --> TCCR0B = TCCR0B & 0b11111000 | 0x02;



    //OCR0A = 0;
    //OCR1A = 1;
    //OCR0 = 1;
    //OCA1 = 1;
    //TCCR1A = (1 << WGM00) | (1 << WGM00) |
    //TCCR0 |= (1<<WGM00)|(1<<WGM01)|(1<<COM01)|(1<<CS00);
    //TCCRC0C = 1;

    //TCCR1A |= 0x08;

    //TCCR1A = F_CPU / RECV_AUDIO_SAMPLE_RATE;
    //TCCR1B =  F_CPU / RECV_AUDIO_SAMPLE_RATE;//249;
    Добрый день! Можете разместить запись?
}
*/

int connect_wifi()
{
    // initialize ESP module
    WiFi.init(&Serial1);
    //WiFi.init(&SerialDebug);

    // check for the presence of the shield
    if (WiFi.status() == WL_NO_SHIELD) {
        DEBUG("WiFi shield not present");
        // don't continue
        return STATUS_ERR;
    }

    // attempt to connect to WiFi network
    while (dvWiFiStatus != WL_CONNECTED) {
        DEBUG("Attempting to connect to WPA SSID: ");
        DEBUG(WIFI_SSID);
        // Connect to WPA/WPA2 network
        dvWiFiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
        // you're connected now, so print out the data
        DEBUG("You're connected to the network");
    }

    // Сообщаю индикацией о том, что соединение установлено
    digitalWrite(13, HIGH);

    return STATUS_OK;
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);

    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
    }
    delay(2000);


    //Serial.begin(115200);
    //Serial1.begin(115200);

    //while (connect_wifi() != STATUS_OK);

    // initialize serial for ESP module
    //esp_serial.begin(9600);
    // initialize ESP module
    bool is_init= false;
    while (!is_init) {
        WiFi.init(&Serial1);
        // check for the presence of the shield
        if (WiFi.status() == WL_NO_SHIELD) {
            Serial.println("WiFi shield not present");
            // don't continue
            //while (true);
            continue;
        }

        // attempt to connect to WiFi network
        while ( dvWiFiStatus != WL_CONNECTED) {
            Serial.print("Attempting to connect to WPA SSID: ");
            // Serial.println(ssid);
            // Connect to WPA/WPA2 network
            dvWiFiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
            is_init = true;
            //break;
        }
    }




    pinMode(PIN_OUT_OPEN_DOR, OUTPUT);
    pinMode(PIN_OUT_OPEN_PHONE, OUTPUT);
    pinMode(PIN_IN_IS_CALL, INPUT);
    //pinMode(PIN_OUT_PLAY_AUDIO, OUTPUT);

    DEBUG(F("Start device"));

    //cli();
    //adc_init();
    //dac_init();
    //sei();



    DEBUG(F("Settings mqtt"));
    // setup mqtt client
    //mqttClient.setClient(ethClient);
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(callback_mqtt);

    //restart_net();
}

static const char STATE_NAME_CALL[] = "wait_call";
static const char STATE_NAME_SEND_NOTIFY_CALL[] = "call";
static const char STATE_NAME_WAIT_USER[] = "wait_user";
static const char STATE_NAME_UP_PHONE[] = "up_phone";
static const char STATE_NAME_DOWN_PHONE[] = "down_phone";
static const char STATE_NAME_OPEN_DOR[] = "open_dor";

void publish_topic(char* topic, char *value)
{
    if (mqttClient.state() != MQTT_CONNECTED){
        reconnect_mqtt();
    }
    mqttClient.publish(topic, value, true);
}

void change_state_device()
{
    //return;
    switch(dv_state) {
        case DV_STATE_WAIT:
            //DEBUG("DV_STATE_WAIT");
            // Привожу все пины в исходное СОСТОЯНИЕ
            digitalWrite(PIN_OUT_OPEN_DOR, LOW);
            digitalWrite(PIN_OUT_OPEN_PHONE, LOW);
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_CALL);
            break;
        // Отправка уведомления пользователю о вызове
        case DV_STATE_CALL_SEND_NOTIFY:
            //DEBUG("DV_STATE_CALL_SEND_NOTIFY");
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_SEND_NOTIFY_CALL);
            break;
        // Пользователь подтвердил получение уведомления
        case DV_STATE_WAIT_USER_ACTION:
            //DEBUG("DV_STATE_WAIT_USER_ACTION");
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_WAIT_USER);
            break;
        // Пользователь снял трубку
        case DV_STATE_UP_PHONE:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_UP_PHONE);
            //DEBUG("DV_STATE_UP_PHONE");
            digitalWrite(PIN_OUT_OPEN_PHONE, HIGH);
            break;
        // Пользователь кладет трубку
        case DV_STATE_DOWN_PHONE:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_DOWN_PHONE);
            digitalWrite(PIN_OUT_OPEN_PHONE, LOW);
            //DEBUG("DV_STATE_DOWN_PHONE");
            break;
        case DV_STATE_OPEN_DOR:
            publish_topic(MQTT_TOPIC_PUB_DEVICE_STATE, STATE_NAME_OPEN_DOR);
            digitalWrite(PIN_OUT_OPEN_DOR, HIGH);
            //DEBUG("DV_STATE_OPEN_DOR");
            break;
    }
}

void send_audio_sample()
{
    static uint8_t adc_buffer_copy[ADC_BUFFER_SIZE] = {0};
    if (adc_buffer_length == ADC_BUFFER_SIZE)
    {
        memcpy(adc_buffer_copy, adc_buffer, ADC_BUFFER_SIZE);
        adc_buffer_length = 0;
        mqttClient.publish(MQTT_TOPIC_PUB_DEVICE_VOICE, adc_buffer_copy, ADC_BUFFER_SIZE);
    }
}

void loop()
{
    static long previous_time = 0;
    if (mqttClient.state() != MQTT_CONNECTED) {
        dv_state = DV_STATE_INIT;
        reconnect_mqtt();
        dv_state = DV_STATE_WAIT;
    }

    // Если устройство находится в одном из состояний нужных, считываем данные и отправляем
    //if (dv_state == DV_STATE_UP_PHONE || dv_state == DV_STATE_OPEN_DOR) {
    //    send_audio_sample();
    //}

    long current_time = millis();
    //if (current_time - previous_time >= 20) {
    if (current_time - previous_time >= INTERVAL_STATE_CHANGE) {
        //Serial.print(current_time);
        //Serial.print("   ");
        //Serial.print(previous_time);
        //Serial.println("");
        // детектирую попытку вызова
        call_detect();
        change_state_device();
        previous_time = current_time;
        //DEBUG(current_time);
        //DEBUG(previous_time);
    }
    mqttClient.loop();

}

void call_detect()
{
    uint8_t is_call = 0;
    // Смотрим, что событие произошло на интересующем нас пине
    if ((is_call=digitalRead(PIN_IN_IS_CALL)) == 0) {
        // Если вызов только начался
        if (dv_state == DV_STATE_WAIT) {
            dv_state = DV_STATE_CALL_SEND_NOTIFY;
        }
    // Если вызов закончился
    } else {
        dv_state = DV_STATE_WAIT;
    }
}
