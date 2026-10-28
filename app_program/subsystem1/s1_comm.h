#ifndef S1_COMM_H
#define S1_COMM_H
#include "../app.h"
#include "s1_wire.h"
/* Task-context APIs. E_OK means copied to TX, NOT acknowledged by broker.
 * E_TMOUT means full; E_PAR means invalid/out of range; E_NOSPT without MQTT.
 * Callers own vehicle/sensor state and pass a coherent snapshot. */
ER s1_publish_vehicle(unsigned state, unsigned fault_mask);
ER s1_publish_line(unsigned ir_bits, unsigned barcode); /* 0 none, 1..4 A..D */
ER s1_publish_terrain(unsigned height_mm, unsigned phase); /* phases 0..5 */
ER s1_publish_obstacle(unsigned nearest_mm, unsigned width_mm,
                       unsigned left_mm, unsigned right_mm);
unsigned s1_connection_status(void); /* bits: WiFi, broker, subscribed, TX drop, RX drop */
void s1_comm_task(INT stacd, void *exinf);
#endif
