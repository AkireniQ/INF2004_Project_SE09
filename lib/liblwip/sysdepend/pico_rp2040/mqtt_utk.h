/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h) and, once
 * connected, a periodic heartbeat publish on MQTT_TOPIC. Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
/* Wi-Fi service context ONLY. Call every service iteration, including link down.
 * wifi_up means CYW43_LINK_UP (associated with an IP address). */
void mqtt_utk_poll(int wifi_up);
/* Compatibility entry point; service polling is still required. */
void mqtt_utk_start(void);
#endif

