/* Dedicated micro T-Kernel queue/priority test. See README.md. */
#include "subsystem2.h"
#include "motion_config.h"
#include "motion_transport.h" /* Intentional white-box access for isolated queue checks. */



/* TODO TEAM: unused priorities supported by this kernel; lower number = higher priority.
   All three tasks MUST be pinned to the SAME core for the priority-order assertion. */
#ifndef S2_SELFTEST_PRIORITY
#define S2_SELFTEST_PRIORITY 16
#endif
#ifndef S2_SELFTEST_HIGH_PRIORITY
#define S2_SELFTEST_HIGH_PRIORITY 14
#endif
#ifndef S2_SELFTEST_LOW_PRIORITY
#define S2_SELFTEST_LOW_PRIORITY 15
#endif
#ifndef S2_SELFTEST_AFFINITY
#define S2_SELFTEST_AFFINITY S2_TASK_AFFINITY
#endif
#ifndef S2_SELFTEST_EXPECTED_PRC
#define S2_SELFTEST_EXPECTED_PRC 2 /* TODO TEAM: TP_PRC2 example uses processor2. */
#endif
_Static_assert(S2_SELFTEST_HIGH_PRIORITY < S2_SELFTEST_LOW_PRIORITY,"high probe must have higher precedence");
_Static_assert(S2_SELFTEST_LOW_PRIORITY < S2_SELFTEST_PRIORITY,"coordinator must have lower precedence than probes");

static unsigned passed,failed;
static ID gate_id,ready_id,done_id;
static volatile unsigned order_count,order[2],probe_core[2];

static void check(const char *name,bool ok)
{
    if(ok)passed++;else failed++;
    tm_printf((UB*)"[%s] %s\n",ok?"PASS":"FAIL",name);
}
static void empty_transport(void)
{
    bool zero;
    (void)s2_take_stop(&zero);
    s2_discard_queue();
    s2_record_result(false); /* Clear any test request marked in processing. */
}
static void transport_tests(void)
{
    S2_STATUS fake={{0}};
    empty_transport();
    s2_publish(&fake,false);
    check("Q1 commands rejected before ready",!motion_forward());
    s2_publish(&fake,true); /* Fake readiness only; NO motor task or outputs involved. */

    const unsigned levels[]={1,2,3};
    bool ok=true;
    for(unsigned i=0;i<3;i++)ok=motion_set_speed(levels[i])&&ok;
    for(unsigned i=0;i<3;i++){
        uint16_t word=0;uint32_t issued=0;
        bool got=s2_take_request(&word,&issued);
        ok=got && (word&15u)==S2_SPEED && (word>>4)==levels[i] && ok;
        s2_record_result(got);
    }
    check("Q2 FIFO command order 1,2,3 preserved",ok);
    empty_transport();

    ok=true;
    for(unsigned i=0;i<S2_QUEUE_LENGTH;i++)ok=motion_set_speed(1)&&ok;
    check("Q3 queue capacity accepts exactly configured slots",ok);
    check("Q4 full queue rejects extra command",!motion_forward());
    check("Q5 busy includes queued commands",motion_busy());

    motion_stop(); /* Must work even though queue is full. */
    check("Q6 pending stop blocks new movement submission",!motion_forward());
    bool zero=true;
    check("Q7 stop latch present, retains selected speed",s2_take_stop(&zero)&&!zero);
    uint16_t word;uint32_t issued;
    check("Q8 stop purges old queued movements",!s2_take_request(&word,&issued));
    empty_transport();
    check("Q9 drained transport not busy",!motion_busy());

    bool queued=motion_set_speed(0);
    check("Q10 speed zero uses priority stop",queued&&s2_take_stop(&zero)&&zero);
    empty_transport();

    ok=true;
    for(unsigned n=1;n<=8;n++){
        bool queued_turn=motion_turn_left(n);
        bool got=s2_take_request(&word,&issued);
        ok=queued_turn&&got&&(word&15u)==S2_LEFT&&(word>>4)==n-1&&ok;
        s2_record_result(got);
    }
    check("Q11 eight angles fit three-bit payload",ok);
    check("Q12 invalid inputs rejected",!motion_set_speed(5)&&!motion_turn_left(0)&&!motion_turn_right(9)&&!motion_move_forward_mm(MAX_DISTANCE_MM+1));
    empty_transport();

    bool fresh=motion_forward()&&s2_take_request(&word,&issued);
    fresh=fresh&&(uint32_t)(s2_now_ms()-issued)<=S2_COMMAND_TTL_MS;
    check("Q13 fresh timestamp meets owner age predicate",fresh);
    s2_record_result(false);
    bool stale=motion_forward();
    tk_dly_tsk(S2_COMMAND_TTL_MS+20u);
    stale=stale&&s2_take_request(&word,&issued);
    stale=stale&&(uint32_t)(s2_now_ms()-issued)>S2_COMMAND_TTL_MS;
    check("Q14 delayed timestamp fails owner age predicate",stale);
    /* White-box predicate only: owner task is NOT running to reject it end to end. */
    s2_record_result(false);empty_transport();

    s2_put_bits(&fake,9,9,123);s2_put_bits(&fake,18,9,124);
    s2_publish(&fake,true);
    S2_STATUS readback;
    check("Q15 status snapshot round trip",motion_get_status(&readback)&&s2_get_bits(&readback,9,9)==123&&s2_get_bits(&readback,18,9)==124);
    fake=(S2_STATUS){{0}};s2_publish(&fake,false); /* Leave module NOT ready. */
}
static void probe(unsigned tag)
{
    if(tk_sig_sem(ready_id,1)<E_OK){tk_ext_tsk();return;}
    if(tk_wai_sem(gate_id,1,TMO_FEVR)>=E_OK){
        /* Same core + distinct priorities: no sleep/kernel calls while recording order.
           Volatile is visibility only; this is NOT an SMP shared-buffer stress test. */
        unsigned i=order_count;
        if(i<2){order[i]=tag;
#if TK_SUPPORT_SMP
            probe_core[i]=(unsigned)tk_get_prc();
#else
            probe_core[i]=1;
#endif
            order_count=i+1;
        }
        (void)tk_sig_sem(done_id,1);
    }
    tk_ext_tsk();
}
static void high_probe(INT stacd,void *exinf){(void)stacd;(void)exinf;probe(1);}
static void low_probe(INT stacd,void *exinf){(void)stacd;(void)exinf;probe(2);}
static ID create_sem(void)
{
    T_CSEM c={0};c.sematr=TA_TPRI;c.isemcnt=0;c.maxsem=2;
    return tk_cre_sem(&c);
}
static void priority_test(void)
{
    gate_id=create_sem();ready_id=create_sem();done_id=create_sem();
    ID low=-1,high=-1;
    bool complete=false;
    if(gate_id<E_OK||ready_id<E_OK||done_id<E_OK){check("P0 create priority-test semaphores",false);goto cleanup;}
    order_count=0;order[0]=order[1]=0;probe_core[0]=probe_core[1]=0;
    /* Lower probe starts first but blocks at gate. Higher probe then also blocks. */
    low=app_start_task((FP)low_probe,S2_SELFTEST_LOW_PRIORITY,S2_SELFTEST_AFFINITY,"s2_probe_low");
    high=app_start_task((FP)high_probe,S2_SELFTEST_HIGH_PRIORITY,S2_SELFTEST_AFFINITY,"s2_probe_high");
    if(low<E_OK||high<E_OK){check("P0 create both probe tasks",false);goto cleanup;}
    if(tk_wai_sem(ready_id,2,1000)<E_OK){check("P0 both probes reached gate",false);goto cleanup;}
    /* Coordinator priority16 is lower than probes14/15. Both get a token in this
       priority-ordered semaphore release. Highest-priority waiter should win. */
    if(tk_sig_sem(gate_id,2)<E_OK||tk_wai_sem(done_id,2,1000)<E_OK){check("P0 probes finished",false);goto cleanup;}
    complete=true;
#if TK_SUPPORT_SMP
    bool same_core=probe_core[0]==S2_SELFTEST_EXPECTED_PRC&&probe_core[1]==S2_SELFTEST_EXPECTED_PRC;
#else
    bool same_core=true;
#endif
    tm_printf((UB*)"Priority probe order=%u,%u; processors=%u,%u; expected high=1 then low=2\n",order[0],order[1],probe_core[0],probe_core[1]);
    check("P1 same-core task priority order",same_core&&order_count==2&&order[0]==1&&order[1]==2);
cleanup:
    /* Probe priorities exceed coordinator: after signaling done they exit before it runs.
       On failure, terminate only tasks CREATED BY THIS SELFTEST before freeing resources. */
    if(low>=E_OK){if(!complete)(void)tk_ter_tsk(low);(void)tk_del_tsk(low);}
    if(high>=E_OK){if(!complete)(void)tk_ter_tsk(high);(void)tk_del_tsk(high);}
    if(gate_id>=E_OK)(void)tk_del_sem(gate_id);
    if(ready_id>=E_OK)(void)tk_del_sem(ready_id);
    if(done_id>=E_OK)(void)tk_del_sem(done_id);
}
/* Call only from the isolated test driver (priority16 on TP_PRC2), after
 * subsystem2_init and BEFORE subsystem2_start. Do not run beside command producers. */
bool subsystem2_run_queue_tests(void)
{
    passed=failed=0;
    transport_tests();priority_test();
    tm_printf((UB*)"QUEUE SUMMARY PASS=%u FAIL=%u\n",passed,failed);
    return passed==16 && failed==0;
}
