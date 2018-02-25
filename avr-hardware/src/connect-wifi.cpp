#include <WiFiEsp.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#include "description.h"

int connect_wifi()
{
    // initialize ESP module
    WiFi.init(&Serial);
    //WiFi.init(&SerialDebug);

    // check for the presence of the shield
    if (WiFi.status() == WL_NO_SHIELD) {
        DEBUG("WiFi shield not present");
        // don't continue
        return STATUS_ERR;
    }

    // attempt to connect to WiFi network
    while (WiFiStatus != WL_CONNECTED) {
        DEBUG("Attempting to connect to WPA SSID: ");
        DEBUG(WIFI_SSID);
        // Connect to WPA/WPA2 network
        WiFiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
        // you're connected now, so print out the data
        DEBUG("You're connected to the network");
    }

    // Сообщаю индикацией о том, что соединение установлено
    digitalWrite(LED_CONNECTION_WIFI, HIGH);

    return STATUS_OK;
}
