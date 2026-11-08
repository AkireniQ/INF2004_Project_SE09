#include "s1_comm.h"
#include "../vehicle_controller/vehicle_controller.h"
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

void s1_comm_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
#if TM_MQTT
    unsigned char frame[MQTT_BRIDGE_FRAME_MAX];
    uint32_t telemetry_at=0;
    for (;;) {
        unsigned budget;
        unsigned char stop_seen = 0;

        for (budget = 0; budget < 4u; ++budget) {
            unsigned op;
            unsigned char ack;
            int length = mqtt_bridge_receive(frame);
            if (length < 0) { break; }
            if (length < 2 || frame[0] != MQTT_ROUTE_COMMAND) { continue; }
            if (!s1_command_decode(frame + 1, (unsigned)length - 1u, &op)) {
                ack = (unsigned char)((frame[1] & 1u) | 2u);
            } else if (op == S1_CMD_STOP) {
                vehicle_request_stop();
                stop_seen = 1;
                ack = S1_CMD_STOP;
            } else if(op==S1_CMD_RESET){
                ack=(unsigned char)(4u|((stop_seen||!vehicle_request_reset())?2u:0u));
            } else {
                /* STOP takes priority over any START in the same drained batch. */
                ack = (unsigned char)(S1_CMD_START |
                    ((stop_seen || !vehicle_request_start()) ? 2u : 0u));
            }
            (void)publish(MQTT_ROUTE_ACK, &ack, 1u);
        }
        SYSTIM time;tk_get_tim(&time);
        if((uint32_t)(time.lo-telemetry_at)>=200u){
            telemetry_at=time.lo;
            s1_publish_telemetry();
        }
        (void)tk_dly_tsk(50);
    }
#endif
}
