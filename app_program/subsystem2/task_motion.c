#include "subsystem2.h"
#include "motion_config.h"
#include "motion_core.h"
#include "motion_transport.h"
#include "motion_hw.h"

/* A relative tk_dly_tsk(10) in this port adds a 10ms guard tick and
 * can sleep 20ms. Use the kernel's phase-preserving 10ms cyclic releases.
 * Callback does no motor work; it only wakes the pinned motion owner. */
static ID owner_id;
static void control_release(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(owner_id);
}
static bool execute(uint16_t word)
{
    unsigned op=word&15u,arg=word>>4;
    switch(op){
    case S2_SPEED:return s2_core_set_speed(arg);
    case S2_INC:return s2_core_increase_speed();
    case S2_DEC:return s2_core_decrease_speed();
    case S2_FORWARD:return s2_core_forward();
    case S2_BACKWARD:return s2_core_backward();
    case S2_WHEELS:return s2_core_set_wheel_levels(arg&7u,(arg>>3)&7u);
    case S2_MOVE_F:return s2_core_move_forward_mm(arg);
    case S2_MOVE_B:return s2_core_move_backward_mm(arg);
    case S2_LEFT:return s2_core_turn_left(arg+1);
    case S2_RIGHT:return s2_core_turn_right(arg+1);
    case S2_CLEAR:return s2_core_clear_faults();
    default:return false;
    }
}
EXPORT void motion_control_task(INT stacd,void *exinf)
{
    (void)stacd;(void)exinf;
    /* PWM, encoder callbacks and all motor output writes owned by this pinned task. */
    s2_core_init();
    owner_id=tk_get_tid();
    T_CCYC cyc={0};
    cyc.cycatr=TA_HLNG|TA_STA;
    cyc.cychdr=(FP)control_release;
    cyc.cyctim=S2_PERIOD_MS;
    cyc.cycphs=S2_PERIOD_MS;
    ID cyclic_id=tk_cre_cyc(&cyc);
    if(cyclic_id<E_OK){
        s2_core_stop();
        tm_printf((UB*)"Motion periodic wakeup creation failed: %d\n",cyclic_id);
        tk_ext_tsk();return;
    }
    uint32_t boot=s2_now_ms(),last_command=boot;
    /* Boot is already stopped; no reversal has occurred. Arm the settling
       guard only after stopping actual motion, using the same clock as startup. */
    uint32_t stopped_at_ms=boot;
    bool settling_active=false;
    bool was_busy=false;
    while(1){
        ER wake=tk_slp_tsk(TMO_FEVR);
        if(wake<E_OK)continue;
        /* Discard already accumulated wakeups; never run catch-up PID bursts.
           A missed release while moving still trips the unchanged 11ms guard. */
        (void)tk_can_wup(owner_id);
        uint32_t now=s2_now_ms();
        bool ready=(uint32_t)(now-boot)>=S2_STARTUP_DELAY_MS;
        bool zero=false;
        bool stopped=s2_take_stop(&zero);
        if(stopped){
            bool was_moving=s2_core_busy();
            s2_core_stop();if(zero)s2_core_set_speed(0);
            if(was_moving){stopped_at_ms=s2_now_ms();settling_active=true;}
        }
        if(settling_active && (uint32_t)(s2_now_ms()-stopped_at_ms)>=
                              (S2_DIRECTION_SETTLE_US+999u)/1000u)settling_active=false;
        uint16_t word;uint32_t issued;
        bool have=false,accepted=false;
        unsigned reason=S2_REPLY_NONE;uint32_t age=0;
        if(ready && !stopped && (have=s2_take_request(&word,&issued))){
            unsigned op=word&15u;
            /* Timestamp after taking the request: a pre-dequeue loop timestamp
               can be older than a command enqueued while the owner was blocked. */
            uint32_t received=s2_now_ms();age=(uint32_t)(received-issued);
            bool fresh=age<=S2_COMMAND_TTL_MS;
            bool starts=(op==S2_FORWARD||op==S2_BACKWARD||op==S2_WHEELS||op==S2_MOVE_F||op==S2_MOVE_B||op==S2_LEFT||op==S2_RIGHT);
            bool settled=!starts || s2_core_busy() || !settling_active;
            /* Requests during settling are rejected, never silently replayed later. */
            if(!fresh)reason=S2_REPLY_STALE;
            else if(!settled)reason=S2_REPLY_SETTLING;
            else {
                accepted=execute(word);
                if(accepted){reason=S2_REPLY_OK;last_command=received;}
                else if(s2_core_faults())reason=S2_REPLY_FAULT;
                else if(starts && s2_core_busy())reason=S2_REPLY_BUSY;
                else {
                    S2_STATUS current;s2_core_snapshot(&current);
                    reason=starts && s2_get_bits(&current,2,3)==0?
                           S2_REPLY_ZERO_SPEED:S2_REPLY_INVALID;
                }
            }
        }
        if(!ready)s2_core_stop();
        s2_core_service();
        S2_STATUS snapshot;s2_core_snapshot(&snapshot);
        now=s2_now_ms();
        if(s2_get_bits(&snapshot,0,2)==1 && (uint32_t)(now-last_command)>S2_COMMAND_TIMEOUT_MS){
            /* TODO TEAM: agree velocity freshness policy; reissue wheel/speed requests.
               Stop cancels rather than resumes. Latch TIMEOUT rather than coast forever. */
            s2_core_timeout();s2_core_snapshot(&snapshot);
        }
        bool busy=s2_core_busy();
        if(was_busy && !busy && !stopped){
            stopped_at_ms=s2_now_ms();settling_active=true;
        }
        was_busy=busy;
        if(s2_core_faults())s2_discard_queue();
        s2_publish(&snapshot,ready);
        if(have)s2_record_owner_result(word,age,reason,accepted);
    }
}
