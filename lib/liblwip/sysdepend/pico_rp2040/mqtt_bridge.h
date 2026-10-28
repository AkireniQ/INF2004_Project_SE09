/* Fixed-size, copied task messages. No lwIP or SDK types in this interface. */
#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#define MQTT_BRIDGE_PAYLOAD_MAX 32u
#define MQTT_BRIDGE_FRAME_MAX (1u + MQTT_BRIDGE_PAYLOAD_MAX)
#define MQTT_BRIDGE_QUEUE_BYTES 256u
/* Topic IDs are the minimum-bit representation of the fixed topic dictionary.
 * The first byte uses three bits, bits 3..7 must be zero. */
enum {
    MQTT_ROUTE_STATUS, MQTT_ROUTE_MOTION, MQTT_ROUTE_LINE,
    MQTT_ROUTE_TERRAIN, MQTT_ROUTE_OBSTACLE, MQTT_ROUTE_VEHICLE,
    MQTT_ROUTE_ACK, MQTT_ROUTE_COMMAND,
    MQTT_ROUTE_COUNT
};
#define MQTT_LINK_WIFI 1u
#define MQTT_LINK_BROKER 2u
#define MQTT_LINK_SUBSCRIBED 4u
#define MQTT_LINK_TX_DROP 8u
#define MQTT_LINK_RX_DROP 16u

/* Task context only; returns 0 on success or a negative kernel error.
 * init is called once by subsystem1_init before application tasks start.
 * All sends and receives use TMO_POL, including the Wi-Fi service side.
 * A receive returns a frame byte count (>0) or a negative error.
 * Frames: topic ID byte followed by 1..32 binary payload bytes.
 * Full: reject newest. Oversize/invalid: reject whole message, never truncate. */
int mqtt_bridge_init(void);
int mqtt_bridge_ready(void);
int mqtt_bridge_send(unsigned route, const unsigned char *data, unsigned length);
int mqtt_bridge_receive(unsigned char frame[MQTT_BRIDGE_FRAME_MAX]);
int mqtt_bridge_service_take(unsigned char frame[MQTT_BRIDGE_FRAME_MAX]);
int mqtt_bridge_service_deliver(unsigned route, const unsigned char *data, unsigned length);
/* Only the Wi-Fi service writes link flags; acquire/release snapshot for readers. */
void mqtt_bridge_service_status(unsigned flags);
unsigned mqtt_bridge_status(void);
#endif
