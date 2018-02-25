#include "description.h"

void callback_mqtt(char* topic, byte* payload, unsigned int length)
{
    DEBUG("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Message arrived [");
    DEBUG(topic);
    DEBUG("] ");

    for (int i = 0; i < length; i++) {
        SerialDebug.print((char)payload[i]);
    }
    DEBUG(" ");
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
        digitalWrite(13, LOW);   // Turn the LED on (Note that LOW is the voltage level
    } else {
        digitalWrite(13, HIGH);  // Turn the LED off by making the voltage HIGH
    }
}



void reconnect_mqtt()
{
    // Loop until we're reconnected
    while (!MQTTClient.connected()) {
        DEBUG("Attempting MQTT connection...");

        //String clientId = "ESP8266Client-";
        //clientId += String(random(0xffff), HEX);
        char client_id[50] = {0};
        sprintf(client_id, "arduino-%d", random(0xffff));

        if (MQTTClient.connect(client_id, MQTT_USERNAME, MQTT_PASSWORD)) {
            DEBUG("connected");
            //DEBUG(client_id);
            // Once connected, publish an announcement...
            MQTTClient.publish("/dm/status", "hello world", true);
            MQTTClient.subscribe("/dm/settings");
            //DEBUG("connect settings");
        } else {
            DEBUG("failed, rc=");
            DEBUG(MQTTClient.state());
            DEBUG(" try again in 0.2 seconds");
            // Wait 5 seconds before retrying
            delay(200);
        }
    }
}
