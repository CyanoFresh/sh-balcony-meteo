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

void connectToWifi() {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);
}

void connectToMqtt() {
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP &event) {
    Serial.println("Connected to Wi-Fi.");

    connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event) {
    Serial.print("Disconnected from Wi-Fi. Reason: ");
    Serial.println(event.reason);
    digitalWrite(LED_BUILTIN, HIGH);

    mqttReconnectTimer.detach();
    wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool) {
    Serial.println("Connected to MQTT.");
    digitalWrite(LED_BUILTIN, LOW);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason) {
    Serial.println("Disconnected from MQTT.");
    digitalWrite(LED_BUILTIN, HIGH);

    if (WiFi.isConnected()) {
        mqttReconnectTimer.once(2, connectToMqtt);
    }
}

uint8_t currentReading = 0;
uint16_t temperatureSum = 0;
uint16_t humiditySum = 0;
uint32_t pressureSum = 0;

void readData() {
    temperatureSum += (uint16_t) (bme.readTemperature() * 10);
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

            Serial.println("Data sent");
        }

        currentReading = 0;
        temperatureSum = 0;
        pressureSum = 0;
        humiditySum = 0;
    }
}

void setup() {
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