#include "motion_core.h"
#include "motion_config.h"
#include "motion_crawl.h"
#include "motion_hw.h"
#include "motion_transport.h"
#include "../subsystem3/line_steering.h"

#define WHEEL_CIRCUMFERENCE_UM ((uint32_t)((uint64_t)WHEEL_DIAMETER_MM*3141593u/1000u))
#define MANEUVER_PIVOT_RPM 24u
#define ARC_DIAMETER_MM (2u * S2_ARC_RADIUS_MM)
#define ARC_INNER_RPM ((S3_CRUISE_RPM * (ARC_DIAMETER_MM - EFFECTIVE_TRACK_MM) + S2_ARC_RADIUS_MM) / ARC_DIAMETER_MM)
#define ARC_OUTSIDE_RPM (2u * S3_CRUISE_RPM - ARC_INNER_RPM)
#define CRAWL_ASSIST_MIN_RPM 6u

_Static_assert(ARC_DIAMETER_MM > EFFECTIVE_TRACK_MM, "Both wheels must roll forward on a detour arc");
_Static_assert(S3_CRUISE_RPM <= S2_CRAWL_MAX_RPM && ARC_INNER_RPM > 3u,
               "Crawl speed must leave room for arc wheel trim");

enum { IDLE, VELOCITY, DISTANCE, TURN };
enum { FAULT_COUNTS=1, FAULT_TIMING=2, FAULT_STALL=4, FAULT_TIMEOUT=8 };
/* Bit positions: L count 0..4; R count 5..9; overflow flags 10,11.
   Saturating 5-bit interval counters, atomically drained every control cycle.
   Two unused bits in the 16-bit container are padding, not application fields. */
static volatile uint16_t encoder_word;
/* ISR and owner both pinned to physical core 1. Local IRQ masking is sufficient. */
static struct {
    unsigned mode:2;
    unsigned level:3;
    unsigned left_rpm:8;
    unsigned right_rpm:8;
    unsigned left_reverse:1;
    unsigned right_reverse:1;
    unsigned arc:1;
    unsigned crawl_pivot:1;
    unsigned faults:4;
} state;
/* Explicit initial bounds: RPM 0..300 (9); errors -300..240 (10 signed);
   integral -2000..2000 (12 signed); PWM 0..100 (7);
   bounded movement target/progress 0..8191 (13). */
static struct wheel_data {
    unsigned rpm:9;
    signed integral:12;
    signed previous_error:10;
    unsigned duty:7;
    unsigned target:13;
    unsigned progress:13;
} wheels[2];
/* Pico hardware timestamps use the SDK's native 64-bit representation.
   They are infrastructure, not compact application messages. See README scope note. */
static uint64_t last_tick, command_start;
static uint32_t timing_fault_gap_us; /* Diagnostic only; latched until explicit clear. */
static uint64_t stall_start[2];
static S2_CRAWL crawl;
static bool crawling;
static unsigned cpr(unsigned w) { return w ? RIGHT_CPR_X10 : LEFT_CPR_X10; }
static int clamp(int v, int lo, int hi) { return v<lo?lo:(v>hi?hi:v); }
void subsystem2_encoder_gpio_irq(unsigned gpio, uint32_t events)
{
    if (!(events & S2_GPIO_EDGE_RISE)) return;
    unsigned shift;
    if (gpio == LEFT_ENC_A) shift=0;
    else if (gpio == RIGHT_ENC_A) shift=5;
    else return;
    UW saved;DI(saved);
    uint16_t word=encoder_word;
    unsigned n=(word>>shift)&31u;
    if (n == 31u) word |= (uint16_t)(1u << (shift ? 11 : 10));
    else word=(uint16_t)((word & ~(31u<<shift)) | ((n+1u)<<shift));
    encoder_word=word;
    EI(saved);
}
static uint16_t drain_encoder(void)
{
    UW saved;DI(saved);
    uint16_t result=encoder_word;
    encoder_word=0;
    EI(saved);
    return result;
}

static S2_TRAVEL travel;

S2_TRAVEL s2_core_travel(void) { return travel; }
static void drive_ticks(unsigned w, unsigned ticks)
{
    unsigned a=w?RIGHT_IN_A:LEFT_IN_A, b=w?RIGHT_IN_B:LEFT_IN_B;
    bool reverse=w?state.right_reverse:state.left_reverse;
    reverse ^= w?RIGHT_MOTOR_INVERT:LEFT_MOTOR_INVERT;
    /* Clear both before selecting the active input. Zero duty = coast. */
    s2_hw_pwm_write(a,0);
    s2_hw_pwm_write(b,0);
    if (ticks) s2_hw_pwm_write(reverse?b:a, (uint16_t)ticks);
    wheels[w].duty=(ticks+5u)/10u;
}
static void drive(unsigned w, unsigned percent)
{
    drive_ticks(w,percent*10u);
}
/* Robo Pico vendor demo uses throttle=0 (both inputs high) for braking.
 * Apply only to a zero-target wheel during forward velocity steering.
 * Full stops/faults still use the established coast path in s2_core_stop.
 */
static void hold_pivot_wheel(unsigned w)
{
#if S2_BRAKE_ZERO_WHEEL
    unsigned a=w?RIGHT_IN_A:LEFT_IN_A, b=w?RIGHT_IN_B:LEFT_IN_B;
    /* TOP=999, CC=1000 holds both inputs continuously high. */
    s2_hw_pwm_write(a,1000);
    s2_hw_pwm_write(b,1000);
    wheels[w].duty=0; /* zero propulsion duty; electrical brake is active */
#else
    drive(w,0);
#endif
}
void s2_core_stop(void)
{
    drive(0,0); drive(1,0);
    state.mode=IDLE;
    state.arc=false;
    state.crawl_pivot=false;
    crawling=false;
    for(unsigned i=0;i<2;i++) {
        wheels[i].integral=0;
        wheels[i].previous_error=0;
        stall_start[i]=0;
    }
}
static void fail(unsigned fault) { s2_core_stop(); state.faults |= fault; }
void s2_core_init(void)
{
    state=(__typeof__(state)){0};
    travel=(S2_TRAVEL){0};
    for(unsigned i=0;i<2;i++)wheels[i]=(struct wheel_data){0};
    timing_fault_gap_us=0;
    state.level=1; /* 25% default target speed. */
    s2_hw_pwm_init();
    s2_core_stop();
    encoder_word=0;
    if(s2_hw_encoder_init()<E_OK) state.faults |= FAULT_COUNTS;
    /* x1 A-rising counting; B inputs reserved, not quadrature direction decoding. */
    last_tick=s2_hw_time_us();
}
static bool start(unsigned mode, bool lr, bool rr, unsigned lt, unsigned rt)
{
    if(state.faults || s2_core_busy() || !state.level || lt>8191 || rt>8191) return false;
    /* Stop -> 200 ms settle -> service -> new command. Caller must provide settling. */
    drain_encoder();
    state.mode=mode; state.left_reverse=lr; state.right_reverse=rr;
    state.left_rpm=state.right_rpm=state.level*MAX_RPM/4u;
    for(unsigned i=0;i<2;i++) {
        wheels[i].integral=0; wheels[i].previous_error=0;
        wheels[i].target=0; wheels[i].progress=0; wheels[i].duty=0;
    }
    wheels[0].target=lt; wheels[1].target=rt;
    command_start=last_tick=s2_hw_time_us();
    stall_start[0]=stall_start[1]=0;
    return true;
}
bool s2_core_set_speed(unsigned level)
{
    if(level>4 || state.faults) return false;
    state.level=level;
    if(!level) s2_core_stop();
    else if(state.mode==VELOCITY) state.left_rpm=state.right_rpm=level*MAX_RPM/4u;
    /* During a bounded move, changes take effect on the NEXT command. */
    return true;
}
bool s2_core_increase_speed(void) { return s2_core_set_speed(state.level<4?state.level+1:4); }
bool s2_core_decrease_speed(void) { return s2_core_set_speed(state.level?state.level-1:0); }
bool s2_core_forward(void) { return start(VELOCITY,false,false,0,0); }
bool s2_core_backward(void) { return start(VELOCITY,true,true,0,0); }
bool s2_core_set_wheel_levels(unsigned left,unsigned right)
{
    if(left>4 || right>4) return false;
    return s2_core_set_wheel_rpm(left*MAX_RPM/4u,right*MAX_RPM/4u);
}
bool s2_core_set_wheel_rpm(unsigned left,unsigned right)
{
    if(left>MAX_RPM || right>MAX_RPM || state.faults) return false;
    if(!left && !right) { s2_core_stop(); return true; }
    if(state.mode!=IDLE && state.mode!=VELOCITY) return false;
    if(state.mode==VELOCITY && (state.left_reverse || state.right_reverse)) return false;
    if(state.mode==IDLE) {
        unsigned selected=state.level;
        if(!selected) state.level=1; /* Independent wheel requests do not need a global speed. */
        bool accepted=start(VELOCITY,false,false,0,0);
        state.level=selected;
        if(!accepted) return false;
    }
    if (!state.left_rpm && left) { wheels[0].integral=0; wheels[0].previous_error=0; }
    if (!state.right_rpm && right) { wheels[1].integral=0; wheels[1].previous_error=0; }
    state.left_rpm=left; state.right_rpm=right;
    return true;
}
static unsigned distance_counts(unsigned mm,unsigned w)
{
    /* circumference in micrometres, pi approximated as 3.141593 */
    uint32_t circumference_um=WHEEL_CIRCUMFERENCE_UM;
    return (unsigned)(((uint64_t)mm*cpr(w)*100u + circumference_um/2)/circumference_um);
}
static bool move_mm(unsigned mm,bool reverse)
{
    if(mm>MAX_DISTANCE_MM) return false;
    if(!mm) return !state.faults;
    return start(DISTANCE,reverse,reverse,distance_counts(mm,0),distance_counts(mm,1));
}
bool s2_core_move_forward_mm(unsigned mm) { return move_mm(mm,false); }
bool s2_core_move_backward_mm(unsigned mm) { return move_mm(mm,true); }
static unsigned turn_counts(unsigned degrees,unsigned wheel)
{
    uint64_t numerator=(uint64_t)EFFECTIVE_TRACK_MM*degrees*cpr(wheel);
    uint32_t denominator=360u*WHEEL_DIAMETER_MM*10u;
    return (unsigned)((numerator+denominator/2)/denominator);
}
static bool pivot(unsigned degrees, bool right)
{
    if (!degrees || degrees > 360u) return false;
    return start(TURN, !right, right, turn_counts(degrees, 0), turn_counts(degrees, 1));
}
static bool turn(unsigned steps,bool right)
{
    return steps >= 1u && steps <= 8u && pivot(steps * 45u, right);
}
bool s2_core_turn_left(unsigned steps) { return turn(steps,false); }
bool s2_core_turn_right(unsigned steps) { return turn(steps,true); }

bool s2_core_crawl_pivot(unsigned degrees, bool right)
{
    unsigned level = state.level;
    if (!level) state.level = 1;
    bool accepted = pivot(degrees, right);
    state.level = level;
    if (accepted) {
        state.crawl_pivot = true;
        state.left_rpm = state.right_rpm = S3_CRUISE_RPM;
    }
    return accepted;
}

bool s2_core_pivot_reached(unsigned degrees, const S2_STATUS *status)
{
    return degrees && degrees <= 360u &&
           s2_get_bits(status, 41, 13) >= turn_counts(degrees, 0) &&
           s2_get_bits(status, 54, 13) >= turn_counts(degrees, 1);
}

bool s2_core_maneuver(unsigned arg)
{
    bool accepted;
    unsigned level=state.level;
    if(!level)state.level=1;
    if ((arg & ~1u)==S2_MANEUVER_TURN) {
        accepted=turn(2,(arg&1u)!=0);
    } else {
        bool reverse=arg>S2_MANEUVER_REVERSE_BASE;
        unsigned mm=reverse?arg-S2_MANEUVER_REVERSE_BASE:arg;
        accepted=mm && mm<=MAX_DISTANCE_MM && move_mm(mm,reverse);
    }
    state.level=level;
    if(accepted)state.left_rpm=state.right_rpm=state.mode==DISTANCE?S3_CRUISE_RPM:MANEUVER_PIVOT_RPM;
    return accepted;
}

static unsigned arc_counts(bool inside,unsigned wheel)
{
    /* Wheel revolutions = arc fraction * path diameter / wheel diameter. */
    unsigned diameter=2u*S2_ARC_RADIUS_MM;
    unsigned path_diameter=inside?diameter-EFFECTIVE_TRACK_MM:diameter+EFFECTIVE_TRACK_MM;
    unsigned denominator=360u*WHEEL_DIAMETER_MM*10u;
    return (path_diameter*cpr(wheel)*S2_ARC_ANGLE_DEG+denominator/2u)/denominator;
}

bool s2_core_arc(bool right)
{
    unsigned level=state.level;
    if(!level)state.level=1;
    bool accepted=start(TURN,false,false,arc_counts(!right,0),arc_counts(right,1));
    state.level=level;
    if(!accepted)return false;
    state.arc=true;
    state.left_rpm=right?ARC_OUTSIDE_RPM:ARC_INNER_RPM;
    state.right_rpm=right?ARC_INNER_RPM:ARC_OUTSIDE_RPM;
    return true;
}

bool s2_core_arc_finished(bool right,const S2_STATUS *status)
{
    return s2_get_bits(status,0,2)==IDLE && !s2_get_bits(status,5,4) &&
           s2_get_bits(status,41,13)>=arc_counts(!right,0) &&
           s2_get_bits(status,54,13)>=arc_counts(right,1);
}

bool s2_core_distance_reached(unsigned mm,const S2_STATUS *status)
{
    return s2_get_bits(status,41,13)>=distance_counts(mm,0) &&
           s2_get_bits(status,54,13)>=distance_counts(mm,1);
}

unsigned s2_core_distance_travelled_mm(const S2_STATUS *status)
{
    unsigned left=(unsigned)((uint64_t)s2_get_bits(status,41,13)*WHEEL_CIRCUMFERENCE_UM/(cpr(0)*100u));
    unsigned right=(unsigned)((uint64_t)s2_get_bits(status,54,13)*WHEEL_CIRCUMFERENCE_UM/(cpr(1)*100u));
    return left<right?left:right;
}

bool s2_core_maneuver_finished(unsigned arg,const S2_STATUS *status)
{
    if(s2_get_bits(status,0,2)!=IDLE || s2_get_bits(status,5,4))return false;
    if((arg&~1u)==S2_MANEUVER_TURN)
        return s2_core_pivot_reached(90, status);
    unsigned mm=arg>S2_MANEUVER_REVERSE_BASE?arg-S2_MANEUVER_REVERSE_BASE:arg;
    return mm && mm<=MAX_DISTANCE_MM && s2_core_distance_reached(mm,status);
}
bool s2_core_busy(void) { return state.mode!=IDLE; }
unsigned s2_core_faults(void) { return state.faults; }
bool s2_core_clear_faults(void)
{
    if(s2_core_busy()) return false;
    drain_encoder(); state.faults=0; timing_fault_gap_us=0; last_tick=s2_hw_time_us();
    return true;
}
static unsigned pid(unsigned w,unsigned target)
{
    int error=(int)target-(int)wheels[w].rpm;
    int derivative=error-wheels[w].previous_error;
    int integral=clamp(wheels[w].integral+error,-INTEGRAL_LIMIT,INTEGRAL_LIMIT);
    int base=(int)(target*100u/MAX_RPM);
    int ki_den=target<60u?KI_LOW_SPEED_DEN:KI_DEN;
    int output=base+error*KP_NUM/KP_DEN+integral/ki_den+derivative*KD_NUM/KD_DEN;
    /* Conditional integration avoids pushing further into output saturation. */
    if(!((output>100 && error>0)||(output<0 && error<0))) wheels[w].integral=integral;
    wheels[w].previous_error=error;
    return (unsigned)clamp(output,0,100);
}
void s2_core_service(void)
{
    uint64_t now=s2_hw_time_us(), elapsed=now-last_tick;
    /* RTOS owner calls once per absolute 10 ms release; use actual elapsed time. */
    if(!elapsed) return;
    uint16_t sample=drain_encoder();
    travel.left += sample & 31u;
    travel.right += (sample >> 5) & 31u;
    last_tick=now;
    bool moving=s2_core_busy();
    if(moving && (sample & (3u<<10))) { fail(FAULT_COUNTS); return; }
    if(moving && elapsed>MAX_CONTROL_GAP_US) {
        timing_fault_gap_us=elapsed>0xffffffffu?0xffffffffu:(uint32_t)elapsed;
        fail(FAULT_TIMING); return;
    }
    if(moving && state.mode!=VELOCITY && now-command_start>MOVE_TIMEOUT_US) { fail(FAULT_TIMEOUT); return; }
    bool slow_targets=state.left_rpm<=S2_CRAWL_MAX_RPM && state.right_rpm<=S2_CRAWL_MAX_RPM;
    bool slow_forward=slow_targets && state.mode==VELOCITY && !state.left_reverse && !state.right_reverse;
    bool slow_bounded=slow_targets && (state.mode==DISTANCE || state.crawl_pivot);
    bool low_speed=slow_forward || slow_bounded || state.arc;
    if(low_speed && !crawling) { crawl=(S2_CRAWL){0}; }
    crawling=low_speed;
    uint32_t crawl_rpm[2]={0};
    if(crawling) { s2_crawl_sample(&crawl,(uint16_t)elapsed,sample,crawl_rpm); }
    bool both_driving=slow_forward || (slow_bounded && wheels[0].progress<wheels[0].target &&
                                      wheels[1].progress<wheels[1].target);
    bool allow_assist = both_driving && (slow_forward || state.crawl_pivot) &&
                        state.left_rpm >= CRAWL_ASSIST_MIN_RPM && state.right_rpm >= CRAWL_ASSIST_MIN_RPM;
    bool match_wheels=both_driving && state.left_rpm==state.right_rpm && state.left_rpm!=0u;
    int arc_trim=0;
    if(state.arc) {
        /* Correct the fraction of each wheel's arc completed. This keeps the
           turn on its radius when one wheel needs more torque to start. */
        int left=(int)(wheels[0].progress*100u/wheels[0].target);
        int right=(int)(wheels[1].progress*100u/wheels[1].target);
        arc_trim=clamp((right-left)/2,-3,3);
    }

    for(unsigned w=0;w<2;w++) {
        unsigned n=(sample>>(w?5:0))&31u;
        unsigned rpm=(unsigned)((uint64_t)n*600000000u/(cpr(w)*elapsed));
        if(rpm>300 && moving) { fail(FAULT_COUNTS); return; }
        if(crawling) {
            wheels[w].rpm=(crawl_rpm[w]+S2_RPM_SCALE/2u)/S2_RPM_SCALE;
        } else {
            wheels[w].rpm=(3u*wheels[w].rpm+(rpm>300?300:rpm)+2u)/4u;
        }
        if(!moving) continue;
        if(state.mode==DISTANCE || state.mode==TURN) {
            unsigned p=wheels[w].progress+n;
            if(p>8191) { fail(FAULT_COUNTS); return; }
            wheels[w].progress=p;
            if(p>=wheels[w].target) { drive(w,0); continue; }
        }
        unsigned target=w?state.right_rpm:state.left_rpm;
        if(state.arc)target=(unsigned)((int)target+(w?-arc_trim:arc_trim));
        if(!target) {
            if(state.mode==VELOCITY && (state.left_rpm || state.right_rpm)) {
                hold_pivot_wheel(w);
            } else { drive(w,0); }
            wheels[w].integral=0; wheels[w].previous_error=0;
            crawl.assist_ticks[w] = 0;
            stall_start[w]=0;
            continue;
        }
        /* Reduce speed near a distance/turn target; coast overshoot still needs calibration. */
        if(state.mode!=VELOCITY && wheels[w].target-wheels[w].progress<80 && target>60) target=60;
        unsigned ticks=crawling ? s2_crawl_duty(&crawl, w, target, crawl_rpm, match_wheels, allow_assist)
                                : 10u*pid(w,target);
        if(n || ticks<200u) stall_start[w]=now;
        if(!stall_start[w]) stall_start[w]=now;
        if(now-stall_start[w]>STALL_US) { fail(FAULT_STALL); return; }
        drive_ticks(w,ticks);
    }
    if(moving && state.mode!=VELOCITY &&
       wheels[0].progress>=wheels[0].target && wheels[1].progress>=wheels[1].target) s2_core_stop();
}


void s2_core_snapshot(S2_STATUS *out)
{
    *out=(S2_STATUS){{0}};
    s2_put_bits(out,0,9,state.mode | (state.level<<2) | (state.faults<<5));
    s2_put_bits(out,9,9,wheels[0].rpm); s2_put_bits(out,18,9,wheels[1].rpm);
    s2_put_bits(out,27,7,wheels[0].duty); s2_put_bits(out,34,7,wheels[1].duty);
    s2_put_bits(out,41,13,wheels[0].progress); s2_put_bits(out,54,13,wheels[1].progress);
}

void s2_core_timeout(void) { fail(FAULT_TIMEOUT); }

uint32_t s2_core_timing_fault_gap_us(void) { return timing_fault_gap_us; }
