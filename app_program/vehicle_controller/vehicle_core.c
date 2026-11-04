#include "vehicle_core.h"
void vc_core_step(VC_MODEL *m, const VC_INPUT *in, VC_OUTPUT *out)
{
    *out=(VC_OUTPUT){.stop=true};
    if(in->stop) { m->running=false; m->state=VC_IDLE; m->reason=VC_REASON_STOP; return; }
    if(in->motor_faults) {
        m->running=false; m->state=VC_FAULT; m->reason=VC_REASON_MOTOR; return;
    }
    bool fresh=in->line_valid && in->line_age_ms<=VC_LINE_MAX_AGE_MS;
    bool linked=!in->link_required || in->link_ok;
    if(in->start && in->ready && fresh && !in->line_stop && linked && !in->obstacle_blocked) {
        m->running=true; m->reason=VC_REASON_NONE;
    }
    if(!m->running)return;
    if(!in->ready || !fresh || !linked || in->obstacle_blocked || in->line_stop) {
        m->running=false;
        m->state=(in->line_stop || in->obstacle_blocked)?VC_IDLE:VC_FAULT;
        m->reason=in->obstacle_blocked?VC_REASON_OBSTACLE:
                  !linked?VC_REASON_LINK:!in->ready?VC_REASON_MOTOR:
                  !fresh?VC_REASON_STALE_LINE:VC_REASON_LINE_STOP;
        return;
    }
    m->state=VC_FOLLOW_LINE; m->reason=VC_REASON_NONE;
    out->stop=false;out->left_rpm=in->left_rpm;out->right_rpm=in->right_rpm;
}
