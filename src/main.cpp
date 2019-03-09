#include <AsyncMqttClient.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <Wire.h>
#include "SparkFunBME280.h"

#define WIFI_SSID "Solomaha"
#define WIFI_PASSWORD "solomakha21"

#define MQTT_HOST IPAddress(192, 168, 1, 230)
#define MQTT_PORT 1883
#define MQTT_ID "balcony-meteo"
#define MQTT_PASSWORD "gfbnjdfgshbnmsdgbnhjmksgdfsdfgdhfdhgf"

#define READINGS_COUNT 3

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

BME280 bme;  // I2C
Ticker sendTimer;

int currentReading = 0;
double temperatureSum = 0;
int humiditySum = 0;
double pressureSum = 0;

void sendData() {
    temperatureSum += bme.readTempC();
    humiditySum += (int)bme.readFloatHumidity();
    pressureSum += bme.readFloatPressure() / 100.0;

    currentReading += 1;

    if (currentReading == READINGS_COUNT) {
        mqttClient.publish("variable/balcony-air_temperature", 0, false, String(temperatureSum / READINGS_COUNT, 1).c_str());
        mqttClient.publish("variable/balcony-air_humidity", 0, false, String(humiditySum / READINGS_COUNT).c_str());
        mqttClient.publish("variable/balcony-air_pressure", 0, false, String(pressureSum / READINGS_COUNT, 1).c_str());

        Serial.println("Data sent");

        currentReading = 0;
        temperatureSum = 0;
        humiditySum = 0;
        pressureSum = 0;
    }
}

void connectToWifi() {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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
    Serial.println("Disconnected from Wi-Fi.");
    Serial.println(event.reason);

    mqttReconnectTimer.detach();
    sendTimer.detach();

    wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool) {
    Serial.println("Connected to MQTT.");

    // Send initial state
    sendData();
    sendTimer.attach(60, sendData);

    // Subscribe
    mqttClient.subscribe("device/balcony-meteo", 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.print("Disconnected from MQTT. Reason: ");
    Serial.println((int) reason);

    sendTimer.detach();

    if (WiFi.isConnected()) {
        mqttReconnectTimer.once(2, connectToMqtt);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println();

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setClientId(MQTT_ID);
    mqttClient.setCredentials("device", MQTT_PASSWORD);

    Wire.begin();
    bme.setI2CAddress(0x76);
    if (!bme.beginI2C()) Serial.println("bme connect failed");

    connectToWifi();
}

void loop() {
}