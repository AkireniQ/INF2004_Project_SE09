#include "s1_comm.h"
#include "../subsystem2/subsystem2.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h"

static ER publish(unsigned route, const unsigned char *data, unsigned length)
{
#if TM_MQTT
    return (ER)mqtt_bridge_send(route, data, length);
#else
    (void)route; (void)data; (void)length;
    return E_NOSPT;
#endif
}

ER s1_publish_vehicle(unsigned state, unsigned fault_mask)
{
    unsigned char data;
    if (state > 7u || fault_mask > 31u) { return E_PAR; }
    data = (unsigned char)(state | (fault_mask << 3u));
    return publish(MQTT_ROUTE_VEHICLE, &data, 1u);
}

ER s1_publish_line(unsigned ir_bits, unsigned barcode)
{
    unsigned char data[1];
    if (!s1_wire_line(data, ir_bits, barcode)) { return E_PAR; }
    return publish(MQTT_ROUTE_LINE, data, sizeof(data));
}

ER s1_publish_terrain(unsigned height_mm, unsigned phase)
{
    unsigned char data[2];
    if (!s1_wire_terrain(data, height_mm, phase)) { return E_PAR; }
    return publish(MQTT_ROUTE_TERRAIN, data, sizeof(data));
}

ER s1_publish_obstacle(unsigned nearest_mm, unsigned width_mm,
                       unsigned left_mm, unsigned right_mm)
{
    unsigned char data[6];
    if (!s1_wire_obstacle(data, nearest_mm, width_mm, left_mm, right_mm)) { return E_PAR; }
    return publish(MQTT_ROUTE_OBSTACLE, data, sizeof(data));
}

unsigned s1_connection_status(void)
{
#if TM_MQTT
    return mqtt_bridge_status();
#else
    return 0;
#endif
}

#if TM_MQTT
/* Private to the command task: no cross-task packed read-modify-write. */
typedef struct {
    unsigned char phase : 2;   /* 0 stopped, 1 waiting for owner, 2 running */
    unsigned char refresh : 3; /* 0..4 service ticks; renew every ~200ms */
} S1_CONTROL;
_Static_assert(sizeof(S1_CONTROL) == 1u, "command control must fit one byte");

static void stop_requested_motion(S1_CONTROL *control)
{
    motion_stop();
    control->phase = 0;
    control->refresh = 0;
}

static int request_start(S1_CONTROL *control)
{
    S2_STATUS status;
    if ((s1_connection_status() & 7u) != 7u || !motion_get_status(&status) ||
        s2_get_bits(&status, 5u, 4u) != 0u) { return 0; }
    if (control->phase != 0u) { return 1; } /* Duplicate START is idempotent. */
    if (motion_busy()) { return 0; }
    /* Buddy 2 owns pins/PID/encoders. Start at speed level 1 (25% target RPM).
     * If either queue operation fails, the priority stop cancels both. */
    if (!motion_set_speed(1u) || !motion_forward()) {
        stop_requested_motion(control);
        return 0;
    }
    control->phase = 1;
    control->refresh = 0;
    return 1;
}

static void maintain_requested_motion(S1_CONTROL *control)
{
    S2_STATUS status;
    if (control->phase == 0u) { return; }
    if ((s1_connection_status() & 7u) != 7u || !motion_get_status(&status) ||
        s2_get_bits(&status, 5u, 4u) != 0u) {
        stop_requested_motion(control);
        return;
    }
    if (++control->refresh < 4u) { return; }
    control->refresh = 0;
    /* A queued START can be rejected by the owner (e.g. settling or stale).
     * Never keep renewing or automatically retry after such a rejection. */
    if (s2_get_bits(&status, 0u, 2u) != 1u || !motion_set_speed(1u)) {
        stop_requested_motion(control);
        return;
    }
    control->phase = 2;
}
#endif

void s1_comm_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
#if TM_MQTT
    unsigned char frame[MQTT_BRIDGE_FRAME_MAX];
    S1_CONTROL control = {0};
    for (;;) {
        unsigned budget;
        unsigned char stop_seen = 0;
        maintain_requested_motion(&control);
        for (budget = 0; budget < 4u; ++budget) {
            unsigned op;
            unsigned char ack;
            int length = mqtt_bridge_receive(frame);
            if (length < 0) { break; }
            if (length < 2 || frame[0] != MQTT_ROUTE_COMMAND) { continue; }
            if (!s1_command_decode(frame + 1, (unsigned)length - 1u, &op)) {
                ack = (unsigned char)((frame[1] & 1u) | 2u);
            } else if (op == S1_CMD_STOP) {
                stop_requested_motion(&control);
                stop_seen = 1;
                ack = S1_CMD_STOP;
            } else {
                /* STOP takes priority over any START in the same drained batch. */
                ack = (unsigned char)(S1_CMD_START |
                    ((stop_seen || !request_start(&control)) ? 2u : 0u));
            }
            (void)publish(MQTT_ROUTE_ACK, &ack, 1u);
        }
        /* Heartbeat is emitted by the Wi-Fi service. No sensor collection or
         * automatic motion telemetry is required for this command-only demo. */
        (void)tk_dly_tsk(50);
    }
#endif
}
