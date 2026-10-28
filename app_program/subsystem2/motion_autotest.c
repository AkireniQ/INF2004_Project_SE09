/* Automatic motor test producer. Never writes motor pins; uses public motion APIs.
 * Excluded logically with MOTION_AUTOTEST=0; other subsystems must not produce commands
 * while this test is enabled. It runs once per reset, then stays stopped. */
#include "subsystem2.h"
#include "motion_config.h"
#include "motion_core.h" /* Read-only timing fault diagnostic. */
#include "motion_transport.h"
#include "motion_hw.h" /* Startup clock validation; no output writes. */
extern bool subsystem2_run_queue_tests(void);
enum { TEST_SPEED, TEST_MOVE, TEST_LEFT, TEST_RIGHT, TEST_FORWARD };
static const char *test_command_name(unsigned op)
{
    switch(op){case TEST_SPEED:return "set_speed";case TEST_MOVE:return "move_forward_mm";
      case TEST_LEFT:return "turn_left";case TEST_RIGHT:return "turn_right";
      case TEST_FORWARD:return "forward";default:return "unknown";}
}
static const char *reply_reason(unsigned r)
{
    switch(r){case S2_REPLY_OK:return "accepted";case S2_REPLY_STALE:return "stale";
      case S2_REPLY_SETTLING:return "settling";case S2_REPLY_FAULT:return "motor fault";
      case S2_REPLY_BUSY:return "busy";case S2_REPLY_ZERO_SPEED:return "zero speed";
      default:return "invalid/unavailable";}
}
static bool command(unsigned op,unsigned arg)
{
    uint32_t old_seq=0,seq=0;bool result=false,queued=false;
    if(!motion_last_result(&old_seq,&result)){
        tm_printf((UB*)"COMMAND FAIL %s(%u): not ready before submit\n",test_command_name(op),arg);return false;
    }
    switch(op){
      case TEST_SPEED:queued=motion_set_speed(arg);break;
      case TEST_MOVE:queued=motion_move_forward_mm(arg);break;
      case TEST_LEFT:queued=motion_turn_left(arg);break;
      case TEST_RIGHT:queued=motion_turn_right(arg);break;
      case TEST_FORWARD:queued=motion_forward();break;
    }
    if(!queued){tm_printf((UB*)"COMMAND FAIL %s(%u): enqueue rejected ready=%u busy=%u faults=%u\n",
      test_command_name(op),arg,(unsigned)motion_ready(),(unsigned)motion_busy(),motion_faults());return false;}
    uint32_t start=s2_now_ms();
    do {
        tk_dly_tsk(10);
        if(!motion_last_result(&seq,&result)){
            tm_printf((UB*)"COMMAND FAIL %s(%u): readiness lost waiting for acknowledgement\n",test_command_name(op),arg);return false;
        }
        if(seq!=old_seq){
            /* Only failure logging here: never print from the 10ms owner loop. */
            if(!result || motion_faults()){
                S2_REPLY reply={0};motion_get_reply(&reply);
                tm_printf((UB*)"COMMAND FAIL %s(%u): reason=%s age=%u ms reply_seq=%u expected_after=%u faults=%u\n",
                  test_command_name(op),arg,reply_reason(reply.reason),(unsigned)reply.age_ms,
                  (unsigned)reply.sequence,(unsigned)old_seq,motion_faults());
            }
            return result&&!motion_faults();
        }
    }while((uint32_t)(s2_now_ms()-start)<500);
    tm_printf((UB*)"COMMAND FAIL %s(%u): acknowledgement timeout seq=%u unchanged_from=%u ready=%u busy=%u faults=%u\n",
      test_command_name(op),arg,(unsigned)seq,(unsigned)old_seq,(unsigned)motion_ready(),(unsigned)motion_busy(),motion_faults());
    return false;
}
static void report(const char *name)
{
    S2_STATUS s={{0}};motion_get_status(&s);
    tm_printf((UB*)"%s: rpm L=%u R=%u PWM L=%u R=%u progress L=%u R=%u faults=%u\n",
      name,s2_get_bits(&s,9,9),s2_get_bits(&s,18,9),s2_get_bits(&s,27,7),
      s2_get_bits(&s,34,7),s2_get_bits(&s,41,13),s2_get_bits(&s,54,13),motion_faults());
    if(motion_faults()&2u)tm_printf((UB*)"Timing fault: observed gap=%u us; limit=%u us\n",
      (unsigned)s2_core_timing_fault_gap_us(),(unsigned)MAX_CONTROL_GAP_US);
}
static bool stopped(void)
{
    uint32_t start=s2_now_ms();motion_stop();
    while(motion_busy()){
        if((uint32_t)(s2_now_ms()-start)>100)return false;
        tk_dly_tsk(10);
    }
    tm_printf((UB*)"Stop observed after %u ms (kernel time resolution 10 ms)\n",(unsigned)(s2_now_ms()-start));
    tk_dly_tsk(500); /* Exceeds owner's 200 ms reversal settling guard. */
    return motion_faults()==0;
}
static bool finish_move(const char *name)
{
    uint32_t start=s2_now_ms();
    while(motion_busy()){
        if(motion_faults()||(uint32_t)(s2_now_ms()-start)>15000){
            tm_printf((UB*)"MOVE FAIL %s: timeout_or_fault elapsed=%u ms faults=%u\n",name,(unsigned)(s2_now_ms()-start),motion_faults());
            motion_stop();return false;
        }
        tk_dly_tsk(20);
    }
    report(name);tk_dly_tsk(500);
    return motion_faults()==0;
}
static bool velocity(unsigned level)
{
    if(!command(TEST_SPEED,level)||!command(TEST_FORWARD,0))return false;
    uint32_t start=s2_now_ms(),refreshed=start,logged=start;
    while((uint32_t)(s2_now_ms()-start)<2000){
        tk_dly_tsk(20);
        if(motion_faults())return false;
        uint32_t now=s2_now_ms();
        /* Velocity watchdog needs fresh requests. Do not repeatedly call forward:
           owner already executes it; refresh the selected speed instead. */
        if((uint32_t)(now-refreshed)>=200){if(!command(TEST_SPEED,level))return false;refreshed=now;}
        if((uint32_t)(now-logged)>=500){report(level==1?"25% target":"50% target");logged=now;}
    }
    return stopped();
}
static void test_task(INT stacd,void *exinf)
{
    (void)stacd;(void)exinf;
    bool ok=subsystem2_run_queue_tests(); /* No motor task exists yet. */
    if(!ok){tk_dly_tsk(5000);tm_printf((UB*)"FAIL: queue/priority checks. Motors were not started.\n");tk_ext_tsk();return;}
    ER er=subsystem2_start();
    if(er<E_OK){tm_printf((UB*)"FAIL starting owner: %d\n",er);tk_ext_tsk();return;}
    uint32_t start=s2_now_ms();
    uint64_t clock_start_us=s2_hw_time_us();
    while(!motion_ready()){
        if((uint32_t)(s2_now_ms()-start)>7000){tm_printf((UB*)"FAIL waiting for motion ready\n");tk_ext_tsk();return;}
        tk_dly_tsk(20);
    }
    uint32_t clock_ms=(uint32_t)(s2_now_ms()-start);
    uint64_t clock_us=s2_hw_time_us()-clock_start_us;
    tm_printf((UB*)"CLOCK CHECK kernel=%u ms hardware=%u us\n",(unsigned)clock_ms,(unsigned)clock_us);
    /* Compare elapsed durations, not absolute origins. Allow scheduling/tick error.
       Reject a frozen/wrong-rate hardware timer before allowing motor output. */
    if(clock_ms<1000u || clock_us<(uint64_t)clock_ms*800u ||
                         clock_us>(uint64_t)clock_ms*1200u){
        tm_printf((UB*)"CLOCK CHECK FAILED: hardware timer and RTOS disagree; no movement requested\n");
        motion_stop();tk_ext_tsk();return;
    }
    tm_printf((UB*)"QUEUE: 16 checks passed. STARTING AUTOMATIC MOVEMENT.\n");
    if(motion_faults())goto failed;
    tm_printf((UB*)"1: forward 200 mm at 25%% target\n");
    if(!command(TEST_SPEED,1)||!command(TEST_MOVE,200)||!finish_move("200 mm"))goto failed;
    tm_printf((UB*)"2: left 45 degrees, then right 45 degrees\n");
    if(!command(TEST_LEFT,1)||!finish_move("left45"))goto failed;
    if(!command(TEST_RIGHT,1)||!finish_move("right45"))goto failed;
    tm_printf((UB*)"3: forward at 25%% target for 2 seconds\n");
    if(!velocity(1))goto failed;
    tm_printf((UB*)"4: forward at 50%% target for 2 seconds\n");
    if(!velocity(2))goto failed;
    tm_printf((UB*)"MOTION AUTOTEST COMPLETE. Verify physical distance/angles yourself. Reset repeats.\n");
    tk_ext_tsk();return;
failed:
    motion_stop();tk_dly_tsk(30);report("AUTOTEST FAILED - stopped; no automatic fault clear");
    tk_ext_tsk();
}
ER subsystem2_autotest_start(void)
{
    ID id=app_start_task((FP)test_task,16,TP_PRC2,"motion_autotest");
    return id<E_OK?(ER)id:E_OK;
}
