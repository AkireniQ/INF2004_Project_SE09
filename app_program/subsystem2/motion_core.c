#include "motion_core.h"
#include "motion_config.h"
#include "motion_hw.h"

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
    unsigned left_level:3;
    unsigned right_level:3;
    unsigned left_reverse:1;
    unsigned right_reverse:1;
    unsigned faults:4;
} state;
/* Explicit initial bounds: RPM 0..300 (9); errors -240..240 (9 signed);
   integral -2000..2000 (12 signed); PWM 0..100 (7);
   bounded movement target/progress 0..8191 (13). */
static struct wheel_data {
    unsigned rpm:9;
    signed integral:12;
    signed previous_error:9;
    unsigned duty:7;
    unsigned target:13;
    unsigned progress:13;
} wheels[2];
/* Pico hardware timestamps use the SDK's native 64-bit representation.
   They are infrastructure, not compact application messages. See README scope note. */
static uint64_t last_tick, command_start;
static uint32_t timing_fault_gap_us; /* Diagnostic only; latched until explicit clear. */
static uint64_t stall_start[2];
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
static void drive(unsigned w, unsigned percent)
{
    unsigned a=w?RIGHT_IN_A:LEFT_IN_A, b=w?RIGHT_IN_B:LEFT_IN_B;
    bool reverse=w?state.right_reverse:state.left_reverse;
    reverse ^= w?RIGHT_MOTOR_INVERT:LEFT_MOTOR_INVERT;
    /* Clear both before selecting the active input. Zero duty = coast. */
    s2_hw_pwm_write(a,0);
    s2_hw_pwm_write(b,0);
    if (percent) s2_hw_pwm_write(reverse?b:a, (uint16_t)(percent*10u));
    wheels[w].duty=percent;
}
void s2_core_stop(void)
{
    drive(0,0); drive(1,0);
    state.mode=IDLE;
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
    state.left_level=state.right_level=state.level;
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
    else if(state.mode==VELOCITY) state.left_level=state.right_level=level;
    /* During a bounded move, changes take effect on the NEXT command. */
    return true;
}
bool s2_core_increase_speed(void) { return s2_core_set_speed(state.level<4?state.level+1:4); }
bool s2_core_decrease_speed(void) { return s2_core_set_speed(state.level?state.level-1:0); }
bool s2_core_forward(void) { return start(VELOCITY,false,false,0,0); }
bool s2_core_backward(void) { return start(VELOCITY,true,true,0,0); }
bool s2_core_set_wheel_levels(unsigned left,unsigned right)
{
    if(left>4 || right>4 || state.faults) return false;
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
    state.left_level=left; state.right_level=right;
    return true;
}
static unsigned distance_counts(unsigned mm,unsigned w)
{
    /* circumference in micrometres, pi approximated as 3.141593 */
    uint32_t circumference_um=(uint32_t)((uint64_t)WHEEL_DIAMETER_MM*3141593u/1000u);
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
static bool turn(unsigned steps,bool right)
{
    if(steps<1 || steps>8) return false;
    unsigned target[2];
    for(unsigned i=0;i<2;i++) {
        uint64_t numerator=(uint64_t)EFFECTIVE_TRACK_MM*steps*45u*cpr(i);
        uint32_t denominator=360u*WHEEL_DIAMETER_MM*10u;
        target[i]=(unsigned)((numerator+denominator/2)/denominator);
    }
    if(!start(TURN,!right,right,target[0],target[1])) return false;
    return true;
}
bool s2_core_turn_left(unsigned steps) { return turn(steps,false); }
bool s2_core_turn_right(unsigned steps) { return turn(steps,true); }
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
    int output=base+error*KP_NUM/KP_DEN+integral/KI_DEN+derivative*KD_NUM/KD_DEN;
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
    last_tick=now;
    bool moving=s2_core_busy();
    if(moving && (sample & (3u<<10))) { fail(FAULT_COUNTS); return; }
    if(moving && elapsed>MAX_CONTROL_GAP_US) {
        timing_fault_gap_us=elapsed>0xffffffffu?0xffffffffu:(uint32_t)elapsed;
        fail(FAULT_TIMING); return;
    }
    if(moving && state.mode!=VELOCITY && now-command_start>MOVE_TIMEOUT_US) { fail(FAULT_TIMEOUT); return; }
    for(unsigned w=0;w<2;w++) {
        unsigned n=(sample>>(w?5:0))&31u;
        unsigned rpm=(unsigned)((uint64_t)n*600000000u/(cpr(w)*elapsed));
        if(rpm>300 && moving) { fail(FAULT_COUNTS); return; }
        wheels[w].rpm=(3u*wheels[w].rpm+(rpm>300?300:rpm)+2u)/4u;
        if(!moving) continue;
        if(state.mode==DISTANCE || state.mode==TURN) {
            unsigned p=wheels[w].progress+n;
            if(p>8191) { fail(FAULT_COUNTS); return; }
            wheels[w].progress=p;
            if(p>=wheels[w].target) { drive(w,0); continue; }
        }
        unsigned level=w?state.right_level:state.left_level;
        if(!level) { drive(w,0); wheels[w].integral=0; stall_start[w]=0; continue; }
        unsigned target=level*MAX_RPM/4u;
        /* Reduce speed near a distance/turn target; coast overshoot still needs calibration. */
        if(state.mode!=VELOCITY && wheels[w].target-wheels[w].progress<80 && target>60) target=60;
        unsigned duty=pid(w,target);
        if(n || duty<20) stall_start[w]=now;
        if(!stall_start[w]) stall_start[w]=now;
        if(now-stall_start[w]>STALL_US) { fail(FAULT_STALL); return; }
        drive(w,duty);
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
