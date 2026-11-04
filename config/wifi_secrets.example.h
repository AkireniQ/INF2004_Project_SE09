/* Copy to wifi_secrets.h locally. Never commit real credentials. */
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H
#define WIFI_SSID "YOUR_2_4_GHZ_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_IP "192.168.1.100" /* numeric IPv4 on the same LAN */
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "se09-car"
/* Optional override: #define MQTT_TOPIC_ROOT "robot/se09-car/v2" */
#endif
