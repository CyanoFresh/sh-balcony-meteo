#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config.h"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

Ticker sendTimer;

Adafruit_BME280 bme;  // I2C

uint8_t currentReading = 0; // max 255
int16_t temperatureSum = 0;    // max 65 536 / 2
uint16_t humiditySum = 0;    // max 65 535
uint32_t pressureSum = 0;   // max ‭4 294 967 296‬

void readData() {
    temperatureSum += (int16_t) (bme.readTemperature() * 10);
    humiditySum += (uint16_t) bme.readHumidity();
    pressureSum += (uint32_t) (bme.readPressure());
    currentReading++;

    if (currentReading == config::READINGS_COUNT) {
        if (mqttClient.connected()) {
            mqttClient.publish("variable/balcony-air_temperature", 0, false,
                               String(temperatureSum / (config::READINGS_COUNT * 10.0), 1).c_str());
            mqttClient.publish("variable/balcony-air_humidity", 0, false,
                               String(humiditySum / config::READINGS_COUNT).c_str());
            mqttClient.publish("variable/balcony-air_pressure", 0, false,
                               String(pressureSum / (config::READINGS_COUNT * 100.0), 1).c_str());

            Serial.println(F("Data sent"));
        }

        currentReading = 0;
        temperatureSum = 0;
        pressureSum = 0;
        humiditySum = 0;
    }
}

void connectToWifi() {
    Serial.println(F("Connecting to Wi-Fi..."));
    WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);
}

void connectToMqtt() {
    Serial.println(F("Connecting to MQTT..."));
    mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP &) {
    connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event) {
    Serial.print(F("Disconnected from Wi-Fi: "));
    Serial.println(event.reason);
    digitalWrite(LED_BUILTIN, LOW);

    mqttReconnectTimer.detach();
    wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool) {
    Serial.println(F("Connected to MQTT."));
    digitalWrite(LED_BUILTIN, HIGH);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.print(F("Disconnected from MQTT. Reason: "));
    Serial.println((int) reason);
    digitalWrite(LED_BUILTIN, LOW);

    if (WiFi.isConnected()) {
        mqttReconnectTimer.once(2, connectToMqtt);
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    Serial.println();
    Serial.println();

    while (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor");
        delay(2000);
    }

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(config::MQTT_HOST, config::MQTT_PORT);
    mqttClient.setClientId(config::MQTT_ID);
    mqttClient.setCredentials("device", config::MQTT_PASSWORD);

    connectToWifi();

    sendTimer.attach(config::SENSOR_READ_INTERVAL, readData);
}

void loop() {}