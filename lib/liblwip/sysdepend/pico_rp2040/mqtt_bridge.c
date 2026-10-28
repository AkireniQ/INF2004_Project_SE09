/* Kernel-facing half: kept apart from lwIP to avoid SDK/T-Kernel size_t clash. */
#include <tk/tkernel.h>
#include "mqtt_bridge.h"

static ID tx_buffer;
static ID rx_buffer;
static unsigned char ready;
static unsigned char link_flags;

int mqtt_bridge_ready(void)
{
    return (int)__atomic_load_n(&ready, __ATOMIC_ACQUIRE);
}

int mqtt_bridge_init(void)
{
    T_CMBF config = {0};
    ID tx;
    ID rx;
    if (mqtt_bridge_ready()) { return E_OK; }
    config.mbfatr = TA_TFIFO;
    config.bufsz = MQTT_BRIDGE_QUEUE_BYTES;
    config.maxmsz = MQTT_BRIDGE_FRAME_MAX;
    tx = tk_cre_mbf(&config);
    if (tx < E_OK) { return tx; }
    rx = tk_cre_mbf(&config);
    if (rx < E_OK) {
        (void)tk_del_mbf(tx);
        return rx;
    }
    tx_buffer = tx;
    rx_buffer = rx;
    /* Publish both immutable handles together after successful creation. */
    __atomic_store_n(&ready, 1u, __ATOMIC_RELEASE);
    return E_OK;
}

static int send_frame(ID buffer, unsigned route, const unsigned char *data,
                      unsigned length)
{
    unsigned char frame[MQTT_BRIDGE_FRAME_MAX];
    unsigned i;
    if (route >= MQTT_ROUTE_COUNT || data == 0 || length == 0u ||
        length > MQTT_BRIDGE_PAYLOAD_MAX) { return E_PAR; }
    frame[0] = (unsigned char)route;
    for (i = 0; i < length; ++i) { frame[i + 1u] = data[i]; }
    return tk_snd_mbf(buffer, frame, (INT)(length + 1u), TMO_POL);
}

int mqtt_bridge_send(unsigned route, const unsigned char *data, unsigned length)
{
    if (!mqtt_bridge_ready()) { return E_OBJ; }
    if (route >= MQTT_ROUTE_COMMAND) { return E_PAR; }
    return send_frame(tx_buffer, route, data, length);
}

int mqtt_bridge_receive(unsigned char frame[MQTT_BRIDGE_FRAME_MAX])
{
    if (!mqtt_bridge_ready()) { return E_OBJ; }
    if (frame == 0) { return E_PAR; }
    return tk_rcv_mbf(rx_buffer, frame, TMO_POL);
}

int mqtt_bridge_service_take(unsigned char frame[MQTT_BRIDGE_FRAME_MAX])
{
    if (!mqtt_bridge_ready()) { return E_OBJ; }
    if (frame == 0) { return E_PAR; }
    return tk_rcv_mbf(tx_buffer, frame, TMO_POL);
}

int mqtt_bridge_service_deliver(unsigned route, const unsigned char *data, unsigned length)
{
    if (!mqtt_bridge_ready()) { return E_OBJ; }
    if (route != MQTT_ROUTE_COMMAND) { return E_PAR; }
    return send_frame(rx_buffer, route, data, length);
}

void mqtt_bridge_service_status(unsigned flags)
{
    __atomic_store_n(&link_flags, flags & 31u, __ATOMIC_RELEASE);
}

unsigned mqtt_bridge_status(void)
{
    return __atomic_load_n(&link_flags, __ATOMIC_ACQUIRE);
}
