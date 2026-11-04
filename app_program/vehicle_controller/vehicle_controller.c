#include "vehicle_controller.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/motion_core.h"
#include "../subsystem2/motion_config.h"
#include "../subsystem1/s1_comm.h"
#ifndef VC_AUTO_START
#if TM_MQTT
#define VC_AUTO_START 0
#else
#define VC_AUTO_START 1
#endif
#endif
#define VC_REFRESH_MS 100u
static ID guard;
static VEHICLE_SNAPSHOT data;
static VC_MODEL model;
static BOOL start_pending,stop_pending,auto_pending,reset_pending;
static uint32_t reset_refreshed;
static uint32_t now_ms(void) { SYSTIM t;tk_get_tim(&t);return t.lo; }
static BOOL lock(void) { return guard>0 && tk_loc_mtx(guard,TMO_FEVR)>=E_OK; }
static void unlock(void) { (void)tk_unl_mtx(guard); }
ER vehicle_init(void)
{
    T_CMTX m={0};m.mtxatr=TA_INHERIT;
    guard=tk_cre_mtx(&m);
    if(guard<E_OK)return guard;
    data=(VEHICLE_SNAPSHOT){0};model=(VC_MODEL){0};
    start_pending=stop_pending=reset_pending=FALSE;auto_pending=VC_AUTO_START;
    return E_OK;
}
void vehicle_submit_line(INT l,INT c,INT r,INT left,INT right,BOOL stop,unsigned barcode)
{
    if(l<0||l>1000||c<0||c>1000||r<0||r>1000||left<0||left>126||right<0||right>126||
       (left&1)||(right&1)||barcode>4){vehicle_invalidate_line();return;}
    uint32_t now=now_ms();
    if(!lock())return;
    data.l=l;data.c=c;data.r=r;data.left_rpm=left;data.right_rpm=right;
    data.barcode=barcode;data.line_stop=stop;data.line_valid=TRUE;data.line_timestamp_ms=now;
    /* A stop recommendation must survive a newer non-stop sensor sample. */
    if(stop){stop_pending=TRUE;start_pending=FALSE;auto_pending=FALSE;}
    unlock();
}
void vehicle_invalidate_line(void)
{ if(lock()){data.line_valid=FALSE;unlock();} }
BOOL vehicle_request_reset(void)
{
    S2_STATUS motor={{0}};
    if(!motion_get_status(&motor) || s2_get_bits(&motor,0,2)!=0)return FALSE;
    if(!lock())return FALSE;
    BOOL ok=!model.running && !start_pending && !reset_pending && s2_get_bits(&motor,5,4)!=0;
    if(ok){reset_pending=TRUE;reset_refreshed=now_ms()-VC_REFRESH_MS;auto_pending=FALSE;}
    unlock();return ok;
}
void vehicle_request_stop(void)
{ if(lock()){stop_pending=TRUE;start_pending=reset_pending=FALSE;auto_pending=FALSE;unlock();} }
static VC_START_BLOCK start_block(BOOL ready,BOOL linked,unsigned faults,uint32_t now)
{
    if(faults)return VC_START_MOTOR;
    if(!linked)return VC_START_LINK;
    if(!ready)return VC_START_STARTUP;
    if(!data.line_valid)return VC_START_NO_LINE;
    if(now-data.line_timestamp_ms>VC_LINE_MAX_AGE_MS)return VC_START_OLD_LINE;
    if(data.line_stop)return VC_START_LINE_STOP;
    if(data.obstacle_blocked)return VC_START_OBSTACLE;
    if(stop_pending || reset_pending)return VC_START_STOP_PENDING;
    return VC_START_READY;
}
BOOL vehicle_request_start(void)
{
    S2_STATUS s={{0}};
    BOOL ready=motion_get_status(&s);
    BOOL linked=TRUE;
#if TM_MQTT
    linked=(s1_connection_status()&7u)==7u;
#endif
    if(!lock())return FALSE;
    uint32_t now=now_ms();
    VC_START_BLOCK block=start_block(ready,linked,s2_get_bits(&s,5,4),now);
    data.last_start_reject=block;
    BOOL ok=block==VC_START_READY;
    if(ok){start_pending=TRUE;auto_pending=FALSE;}
    unlock();
    if(!ok)tm_printf((UB*)"[VEHICLE] START rejected: blocker=%u (1=startup 2=link 3=motor 4=stop 5=no-ADC 6=stale-ADC 7=junction 8=obstacle)\n",(unsigned)block);
    return ok;
}
BOOL vehicle_submit_terrain(unsigned height,unsigned phase)
{
    if(height>2000||phase>5||!lock())return FALSE;
    data.height_mm=height;data.terrain_phase=phase;
    data.terrain_timestamp_ms=now_ms();data.terrain_valid=TRUE;unlock();return TRUE;
}
BOOL vehicle_submit_obstacle(unsigned near,unsigned width,unsigned left,unsigned right,BOOL blocked)
{
    if(near>4000||width>2000||left>4000||right>4000||!lock())return FALSE;
    data.near_mm=near;data.width_mm=width;data.clear_left_mm=left;data.clear_right_mm=right;
    data.obstacle_timestamp_ms=now_ms();data.obstacle_valid=TRUE;
    /* A stale blocked report remains blocked until explicitly cleared. */
    data.obstacle_blocked=blocked;
    if(blocked){stop_pending=TRUE;start_pending=FALSE;auto_pending=FALSE;}
    unlock();return TRUE;
}
BOOL vehicle_get_snapshot(VEHICLE_SNAPSHOT *out)
{
    if(!out)return FALSE;
    S2_STATUS motor={{0}};BOOL ready=motion_get_status(&motor),linked=TRUE;
#if TM_MQTT
    linked=(s1_connection_status()&7u)==7u;
#endif
    if(!lock())return FALSE;
    *out=data;out->timestamp_ms=now_ms();out->motor_ready=ready;out->link_ready=linked;
    out->start_pending=start_pending;out->stop_pending=stop_pending;out->reset_pending=reset_pending;
    out->motor_faults=s2_get_bits(&motor,5,4);
    out->timing_fault_gap_us=s2_core_timing_fault_gap_us();
    out->control_gap_limit_us=MAX_CONTROL_GAP_US;
    out->start_block=start_block(ready,linked,out->motor_faults,out->timestamp_ms);
    unlock();return TRUE;
}
BOOL vehicle_is_following(void)
{ if(!lock())return FALSE;BOOL following=model.running && model.state==VC_FOLLOW_LINE;unlock();return following; }
void vehicle_record_telemetry_drop(void)
{ if(lock()){if(data.telemetry_drops!=0xffffffffu)data.telemetry_drops++;unlock();} }
static BOOL moving;
static unsigned last_l=127,last_r=127;
static uint32_t refreshed;
/* Owner-only operation, separated from pacing for deterministic host tests. */
static void vehicle_service_once(void)
{
        S2_STATUS status={{0}};BOOL ready=motion_get_status(&status);
        uint32_t now=now_ms();BOOL linked=TRUE;
#if TM_MQTT
        linked=(s1_connection_status()&7u)==7u;
#endif
        if(lock()){
            now=now_ms(); /* sample after acquiring the input snapshot */
            BOOL reset=reset_pending;
            VC_INPUT in={0};VC_OUTPUT out;
            in.ready=ready;in.motor_faults=s2_get_bits(&status,5,4);
            in.link_required=TM_MQTT;in.link_ok=linked;
            in.line_stop=data.line_stop;in.line_valid=data.line_valid;in.line_age_ms=now-data.line_timestamp_ms;
            in.left_rpm=data.left_rpm;in.right_rpm=data.right_rpm;
            in.obstacle_blocked=data.obstacle_blocked;
            in.stop=stop_pending;in.start=start_pending;
            if(auto_pending && ready && in.line_valid && in.line_age_ms<=VC_LINE_MAX_AGE_MS){
                in.start=TRUE;auto_pending=FALSE;
            }
            start_pending=stop_pending=FALSE;
            vc_core_step(&model,&in,&out);
            data.state=model.state;data.reason=model.reason;data.motor_faults=in.motor_faults;
            unlock();
            if(reset && out.stop){
                if(!in.motor_faults){
                    if(lock()){reset_pending=FALSE;model=(VC_MODEL){0};data.state=VC_IDLE;data.reason=VC_REASON_NONE;unlock();}
                }else if((uint32_t)(now-reset_refreshed)>=VC_REFRESH_MS){
                    if(motion_clear_faults())reset_refreshed=now;
                }
            }
            if(out.stop){
                if(moving){motion_stop();moving=FALSE;last_l=last_r=127;}
            }else if(out.left_rpm!=last_l||out.right_rpm!=last_r||now-refreshed>=VC_REFRESH_MS){
                if(motion_set_wheel_rpm(out.left_rpm,out.right_rpm)){
                    moving=TRUE;last_l=out.left_rpm;last_r=out.right_rpm;refreshed=now;
                }
            }
        }
}
static void vehicle_task(INT stacd,void *exinf)
{
    (void)stacd;(void)exinf;
    moving=FALSE;last_l=last_r=127;refreshed=0;motion_stop();
    for(;;){vehicle_service_once();(void)tk_dly_tsk(10);}
}
ER vehicle_start_task(void)
{ ID id=app_start_task((FP)vehicle_task,15,TP_PRC1,"vehicle_controller");return id<E_OK?id:E_OK; }
