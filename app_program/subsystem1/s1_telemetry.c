#include "s1_comm.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h"
ER s1_publish_motion(void)
{
#if TM_MQTT
    S2_STATUS s;
    if(!motion_get_status(&s))return E_OBJ;
    /* The first 41 bits contain mode, faults, RPM and PWM. Progress is local. */
    s.bytes[5] &= 1u;
    return (ER)mqtt_bridge_send(MQTT_ROUTE_MOTION, s.bytes, 6u);
#else
    return E_NOSPT;
#endif
}
#if TM_MQTT
static void account(ER result)
{ if(result!=E_OK)vehicle_record_telemetry_drop(); }
#endif
#if TM_MQTT
static void publish_controller(const VEHICLE_SNAPSHOT *s)
{
    unsigned char bytes[13];
    bytes[0]=3;bytes[1]=s->state;bytes[2]=s->reason;bytes[3]=s->start_block;
    bytes[4]=(s->motor_ready?1u:0u)|(s->start_pending?16u:0u)|
             (s->stop_pending?32u:0u)|(s->reset_pending?128u:0u);
    bytes[5]=s->motor_faults;
    bytes[6]=s->last_start_reject;
    for(unsigned i=0;i<4;i++)bytes[7+i]=(s->timing_fault_gap_us>>(8*i))&255;
    bytes[11]=s->control_gap_limit_us&255;bytes[12]=s->control_gap_limit_us>>8;
    account((ER)mqtt_bridge_send(MQTT_ROUTE_CONTROLLER,bytes,sizeof(bytes)));
}

static void publish_obstacle(const VEHICLE_SNAPSHOT *s)
{
    static uint32_t last_at;
    static S1_OBSTACLE_STATUS last_status;
    static unsigned last_width;
    S1_OBSTACLE_STATUS status = S1_OBSTACLE_UNKNOWN;
    if (s->obstacle_blocked || s->state == VC_AVOIDING || s->state == VC_RECOVER_LINE)
        status = S1_OBSTACLE_DETECTED;
    else if (s5_range_available(s->obstacle_forward_mm))
        status = S1_OBSTACLE_CLEAR;
    unsigned width = s->obstacle_profile ? s->width_mm : S1_WIDTH_UNAVAILABLE;
    if (status == last_status && width == last_width && s->timestamp_ms - last_at < 1000u)
        return;
    ER result = s1_publish_obstacle(status, width);
    account(result);
    if (result == E_OK) {
        last_at = s->timestamp_ms;
        last_status = status;
        last_width = width;
    }
}
#endif
void s1_publish_telemetry(void)
{
#if TM_MQTT
    VEHICLE_SNAPSHOT s;
    if((s1_connection_status()&3u)!=3u || !vehicle_get_snapshot(&s))return;
    unsigned faults=s.motor_faults?S1_FAULT_MOTOR:0;
    if(s.motor_faults&1u)faults|=S1_FAULT_ENCODER;
    if(s.reason==VC_REASON_LINK)faults|=S1_FAULT_WIFI;
    if(s.obstacle_no_echo)faults|=S1_FAULT_NO_ECHO;
    account(s1_publish_vehicle(s.state,faults));
    publish_controller(&s);
    /* No fabricated zero measurements during startup or absent subsystems. */
    if(motion_ready())account(s1_publish_motion());
    if(s.line_valid && (uint32_t)(s.timestamp_ms-s.line_timestamp_ms)<=VC_LINE_MAX_AGE_MS){
        unsigned ir=(s.l>=600?1u:0u)|(s.r>=600?4u:0u);
        account(s1_publish_line(ir,s.barcode));
    }
    if(s.terrain_valid && (uint32_t)(s.timestamp_ms-s.terrain_timestamp_ms)<=VC_SENSOR_MAX_AGE_MS)
        account(s1_publish_terrain(s.height_mm));
    publish_obstacle(&s);
#endif
}
