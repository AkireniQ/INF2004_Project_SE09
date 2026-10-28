/* All lwIP/MQTT calls run exclusively in cyw43_utk_task (NO_SYS=1).
 * Application tasks exchange COPIED, bounded messages through mqtt_bridge. */
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "mqtt_bridge.h"
#include <string.h>

#ifndef MQTT_TOPIC_ROOT
#define MQTT_TOPIC_ROOT "robot/" MQTT_CLIENT_ID "/v2"
#endif
#define RETRY_MS 3000u
#define HEARTBEAT_MS 5000u
#define PENDING_MS 5000u
#define TOPIC_MAX 64u
#define TX_BUDGET 2u

static const char * const topics[MQTT_ROUTE_COUNT] = {
    MQTT_TOPIC_ROOT "/status", MQTT_TOPIC_ROOT "/motion",
    MQTT_TOPIC_ROOT "/line", MQTT_TOPIC_ROOT "/terrain",
    MQTT_TOPIC_ROOT "/obstacle", MQTT_TOPIC_ROOT "/vehicle",
    MQTT_TOPIC_ROOT "/ack", MQTT_TOPIC_ROOT "/command"
};
/* Fail compilation rather than silently truncate a configured topic. */
typedef char topic_must_fit[(sizeof(MQTT_TOPIC_ROOT "/obstacle") <= TOPIC_MAX) ? 1 : -1];
static mqtt_client_t *client;
static ip_addr_t broker;
static u8_t connecting;
static u8_t subscribing;
static unsigned char flags;
static u32_t retry_at;
static u32_t subscribe_at;
static u32_t heartbeat_at;
static unsigned char pending[MQTT_BRIDGE_FRAME_MAX];
static int pending_size;
static u32_t pending_at;
static char rx_topic[TOPIC_MAX];
static unsigned char rx_data[MQTT_BRIDGE_PAYLOAD_MAX];
static u32_t rx_expected;
static unsigned rx_size;
static u8_t rx_valid;

static int due(u32_t now, u32_t deadline)
{
    return (s32_t)(now - deadline) >= 0;
}

static void incoming_publish_cb(void *arg, const char *topic, u32_t total)
{
    unsigned i = 0;
    (void)arg;
    /* Topic and fragments belong to lwIP only until each callback returns. */
    while (i < TOPIC_MAX - 1u && topic[i] != '\0') {
        rx_topic[i] = topic[i];
        ++i;
    }
    rx_topic[i] = '\0';
    rx_size = 0;
    rx_expected = total;
    rx_valid = (topic[i] == '\0' && total > 0u &&
                total <= MQTT_BRIDGE_PAYLOAD_MAX &&
                strcmp(rx_topic, topics[MQTT_ROUTE_COMMAND]) == 0);
    if (!rx_valid) { flags |= MQTT_LINK_RX_DROP; }
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t length, u8_t last)
{
    (void)arg;
    if (rx_valid) {
        if ((unsigned)length > rx_expected - rx_size) {
            rx_valid = 0;
        } else {
            memcpy(rx_data + rx_size, data, length);
            rx_size += length;
        }
    }
    if ((last & MQTT_DATA_FLAG_LAST) != 0u) {
        if (!rx_valid || rx_size != rx_expected ||
            mqtt_bridge_service_deliver(MQTT_ROUTE_COMMAND, rx_data, rx_size) < 0) {
            flags |= MQTT_LINK_RX_DROP;
        }
        rx_valid = 0;
    }
}

static void subscribe_done_cb(void *arg, err_t result)
{
    (void)arg;
    subscribing = 0;
    if (result == ERR_OK && (flags & MQTT_LINK_BROKER) != 0u) {
        flags |= MQTT_LINK_SUBSCRIBED;
    } else {
        flags &= ~MQTT_LINK_SUBSCRIBED;
        subscribe_at = sys_now() + RETRY_MS;
    }
}

static void publish_done_cb(void *arg, err_t result)
{
    (void)arg;
    if (result != ERR_OK) { flags |= MQTT_LINK_TX_DROP; }
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
    (void)c;
    (void)arg;
    connecting = 0;
    subscribing = 0;
    rx_valid = 0;
    flags &= ~(MQTT_LINK_BROKER | MQTT_LINK_SUBSCRIBED);
    if (status == MQTT_CONNECT_ACCEPTED) {
        flags |= MQTT_LINK_BROKER;
        mqtt_set_inpub_callback(client, incoming_publish_cb, incoming_data_cb, 0);
        subscribe_at = sys_now(); /* Restore subscription on EVERY reconnect. */
        heartbeat_at = sys_now();
    } else {
        retry_at = sys_now() + RETRY_MS;
        if (pending_size != 0) { flags |= MQTT_LINK_TX_DROP; }
        pending_size = 0;
    }
}

static void connect_client(u32_t now)
{
    struct mqtt_connect_client_info_t info = {0};
    err_t result;
    retry_at = now + RETRY_MS;
    if (client == 0) { client = mqtt_client_new(); }
    if (client == 0) { return; }
    if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker))) { return; }
    IP_SET_TYPE(&broker, IPADDR_TYPE_V4);
    info.client_id = MQTT_CLIENT_ID;
    info.keep_alive = 60;
    info.will_topic = topics[MQTT_ROUTE_STATUS];
    info.will_msg = "\0"; /* One binary status byte: offline. */
    info.will_msg_len = 1;
    info.will_retain = 1;
    connecting = 1;
    result = mqtt_client_connect(client, &broker, MQTT_BROKER_PORT,
                                 connection_cb, 0, &info);
    if (result != ERR_OK) { connecting = 0; }
}

void mqtt_utk_poll(int wifi_up)
{
    u32_t now = sys_now();
    unsigned budget;
    if (wifi_up) { flags |= MQTT_LINK_WIFI; }
    else {
        if (client != 0 && (connecting || (flags & MQTT_LINK_BROKER) != 0u)) {
            mqtt_disconnect(client);
        }
        connecting = 0;
        subscribing = 0;
        rx_valid = 0;
        flags &= ~(MQTT_LINK_WIFI | MQTT_LINK_BROKER | MQTT_LINK_SUBSCRIBED);
        retry_at = now + RETRY_MS;
    }
    /* The Wi-Fi task can run before usermain creates both message buffers. */
    if (!mqtt_bridge_ready()) { return; }
    if (wifi_up && !connecting && (flags & MQTT_LINK_BROKER) == 0u && due(now, retry_at)) {
        connect_client(now);
    }
    if ((flags & MQTT_LINK_BROKER) != 0u) {
        if ((flags & MQTT_LINK_SUBSCRIBED) == 0u && !subscribing && due(now, subscribe_at)) {
            err_t result;
            subscribing = 1;
            subscribe_at = now + RETRY_MS;
            result = mqtt_subscribe(client, topics[MQTT_ROUTE_COMMAND], 0, subscribe_done_cb, 0);
            if (result != ERR_OK) { subscribing = 0; }
        }
        if (due(now, heartbeat_at)) {
            unsigned char status = (unsigned char)flags;
            heartbeat_at = now + HEARTBEAT_MS;
            if (mqtt_publish(client, topics[MQTT_ROUTE_STATUS], &status, 1,
                             0, 1, publish_done_cb, 0) != ERR_OK) {
                flags |= MQTT_LINK_TX_DROP;
            }
        }
    }
    /* Fixed work budget: a busy producer cannot starve radio/TCP polling.
     * Offline telemetry is discarded; ERR_MEM retains one frame for <=5s. */
    for (budget = 0; budget < TX_BUDGET; ++budget) {
        err_t result;
        if (pending_size == 0) {
            pending_size = mqtt_bridge_service_take(pending);
            pending_at = now;
            if (pending_size < 0) { pending_size = 0; break; }
        }
        if (pending_size < 2 || pending_size > (int)MQTT_BRIDGE_FRAME_MAX ||
            pending[0] >= MQTT_ROUTE_COMMAND || (flags & MQTT_LINK_BROKER) == 0u ||
            due(now, pending_at + PENDING_MS)) {
            flags |= MQTT_LINK_TX_DROP;
            pending_size = 0;
            continue;
        }
        result = mqtt_publish(client, topics[pending[0]], pending + 1,
                              (u16_t)(pending_size - 1), 0, 0, publish_done_cb, 0);
        if (result == ERR_MEM) { break; }
        if (result != ERR_OK) { flags |= MQTT_LINK_TX_DROP; }
        pending_size = 0;
    }
    mqtt_bridge_service_status(flags);
}

void mqtt_utk_start(void)
{
    mqtt_utk_poll(1);
}
