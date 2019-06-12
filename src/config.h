#ifndef ROOM1_SECONDARY_LIGHT_CONFIG_H
#define ROOM1_SECONDARY_LIGHT_CONFIG_H

#include <Arduino.h>

namespace config {
    const char WIFI_SSID[] = "SmartHome";
    const char WIFI_PASSWORD[] = "123456789";

//    const auto MQTT_HOST = IPAddress(192, 168, 1, 230);
    const auto MQTT_HOST = IPAddress(176, 36, 198, 196);
    const uint16_t MQTT_PORT = 1883;
    const char MQTT_ID[] = "balcony-meteo";
    const char MQTT_PASSWORD[] = "gfbnjdfgshbnmsdgbnhjmksgdfsdfgdhfdhgf";

    const uint8_t SENSOR_READ_INTERVAL = 10;
    const uint8_t READINGS_COUNT = 3;
}

#endif
