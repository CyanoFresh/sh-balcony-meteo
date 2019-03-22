#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>

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

Ticker sendTimer;

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
    digitalWrite(LED_BUILTIN, LOW);

    connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event) {
    Serial.print("Disconnected from Wi-Fi. Reason: ");
    Serial.println(event.reason);
    digitalWrite(LED_BUILTIN, HIGH);

    mqttReconnectTimer.detach();
    wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
    Serial.println("Connected to MQTT.");
    digitalWrite(LED_BUILTIN, LOW);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("Disconnected from MQTT.");
    digitalWrite(LED_BUILTIN, HIGH);

    if (WiFi.isConnected()) {
        mqttReconnectTimer.once(2, connectToMqtt);
    }
}

uint8_t currentReading = 0;
uint16_t temperatureSum = 0;
uint16_t humiditySum = 0;
uint16_t pressureSum = 0;

Adafruit_BME280 bme;  // I2C

void readData() {
    temperatureSum += (uint16_t) (bme.readTemperature() * 10);
    humiditySum += (uint16_t) bme.readHumidity();
    pressureSum += (uint16_t) (bme.readPressure() * 10);
    currentReading++;

    if (currentReading == READINGS_COUNT) {
        if (mqttClient.connected()) {
            mqttClient.publish("variable/balcony-air_temperature", 0, false,
                               String(temperatureSum / (READINGS_COUNT * 10.0), 1).c_str());
            mqttClient.publish("variable/balcony-air_humidity", 0, false, String(humiditySum / READINGS_COUNT).c_str());
            mqttClient.publish("variable/balcony-air_pressure", 0, false,
                               String(pressureSum / (READINGS_COUNT * 10.0), 1).c_str());

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

    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        return;
    }

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setClientId(MQTT_ID);
    mqttClient.setCredentials("device", MQTT_PASSWORD);

    sendTimer.attach(60, readData);

    connectToWifi();
}

void loop() {}