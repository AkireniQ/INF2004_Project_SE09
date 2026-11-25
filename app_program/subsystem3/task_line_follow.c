/*
 * task_line_follow.c: line following (Buddy 3).
 * line_sense()  reads the 3 sensors -> darkness 0..1000
 * line_decide() PURE logic: darkness in, wheel RPM out (easy to unit-test)
 * line_follow_task() runs both every LINE_PERIOD_MS and drives Motion Control
 */
#include "subsystem3.h"
#include "barcode.h"
#include "../vehicle_controller/vehicle_controller.h"

enum { S3_WAIT_TICK, S3_READ_ADC, S3_UPDATE_STEERING, S3_SUBMIT_LINE };
LOCAL ID owner_id;
LOCAL unsigned sampler_stage;

EXPORT void s3_log_sampler_state(void)
{
    static const char *const stages[] = {
        "waiting for timer", "reading ADC", "updating steering", "submitting IR sample",
    };
    ID task = __atomic_load_n(&owner_id, __ATOMIC_ACQUIRE);
    unsigned stage = __atomic_load_n(&sampler_stage, __ATOMIC_RELAXED);
    T_RTSK state = {0};
    ER result = task > 0 ? tk_ref_tsk(task, &state) : E_NOEXS;
    tm_printf((UB *)"[S3] IR sampler: %s, task state=%u, wait=0x%x, object=%d, status=%d\n",
              stages[stage], state.tskstat, (unsigned)state.tskwait, state.wid, result);
}


/* Calibration: raw ADC on white paper and on the black line (L, C, R).
   Measure on your track before tuning the line thresholds. */
LOCAL const INT white_raw[3] = { 130, 130, 130 };
LOCAL const INT black_raw[3] = { 3200, 3200, 3200 };


LOCAL INT darkness(INT raw, INT white, INT black)
{
    INT d;
    if (black <= white) { return 0; }
    d = ((raw - white) * 1000) / (black - white);
    if (d < 0)    { d = 0; }
    if (d > 1000) { d = 1000; }
    return d;
}


/* Only the outer sensors are required for steering and START. */
EXPORT ER line_sense(INT *left, INT *center, INT *right)
{
    INT left_raw = ir_read(0);
    INT right_raw = ir_read(2);
    if (left_raw < 0 || right_raw < 0) { return E_IO; }
    *left = darkness(left_raw, white_raw[0], black_raw[0]);
    *right = darkness(right_raw, white_raw[2], black_raw[2]);
    INT barcode_raw = ir_read(1);
    *center = barcode_raw < 0 ? -1 : darkness(barcode_raw, white_raw[1], black_raw[1]);
    return E_OK;
}

LOCAL void tick_release(void *exinf)
{
    (void)exinf;
    ID task = __atomic_load_n(&owner_id, __ATOMIC_ACQUIRE);
    (void)tk_wup_tsk(task);
}

EXPORT void line_follow_task(INT stacd, void *exinf)
{
    LF_STATE steering = {0};
    SYSTIM time;
    BOOL was_following = FALSE;
    BOOL reported_missed_tick = FALSE;
    T_CCYC cyc = {0};

    (void)stacd;
    (void)exinf;
    ID task = tk_get_tid();
    __atomic_store_n(&owner_id, task, __ATOMIC_RELEASE);
    cyc.cycatr = TA_HLNG | TA_STA;
    cyc.cychdr = (FP)tick_release;
    cyc.cyctim = LINE_PERIOD_MS;
    cyc.cycphs = LINE_PERIOD_MS;
    ID timer = tk_cre_cyc(&cyc);
    if (timer < E_OK)
        tm_printf((UB *)"[S3] line timer creation failed (%d); using timed wakeups\n", timer);

    for (;;) {
        __atomic_store_n(&sampler_stage, S3_WAIT_TICK, __ATOMIC_RELAXED);
        /* The cyclic interrupt sets the normal cadence. A missed release
           must not strand the task in sleep with an ageing IR sample. */
        ER wake = tk_slp_tsk(2 * LINE_PERIOD_MS);
        if (wake == E_TMOUT && timer >= E_OK && !reported_missed_tick) {
            tm_printf((UB *)"[S3] missed line timer wakeup; retrying ADC sampling\n");
            reported_missed_tick = TRUE;
        } else if (wake < E_OK && wake != E_TMOUT) {
            vehicle_invalidate_line();
            (void)tk_dly_tsk(LINE_PERIOD_MS);
            continue;
        }
        (void)tk_can_wup(task);
        INT left, center, right;
        __atomic_store_n(&sampler_stage, S3_READ_ADC, __ATOMIC_RELAXED);
        ER sample = line_sense(&left, &center, &right);
        if (sample == E_IO) {
            /* Retry the whole sample once so both tracking readings are fresh.
               The controller still times out the previous sample independently. */
            sample = line_sense(&left, &center, &right);
        }
        if (sample < E_OK) {
            vehicle_invalidate_line();
            steering = (LF_STATE){0};
            barcode_reset();
            continue;
        }
        (void)tk_get_otm(&time);

        __atomic_store_n(&sampler_stage, S3_UPDATE_STEERING, __ATOMIC_RELAXED);
        BOOL following = vehicle_is_following();
        if (!following || !was_following) steering = (LF_STATE){0};
        if (!following || !was_following || center < 0) barcode_reset();
        char letter = 0;
        if (following && center >= 0) {
            letter = barcode_feed(center, time.lo);
        }
        was_following = following;

        LF_CMD command;
        line_decide(left, center, right, &steering, &command);
        __atomic_store_n(&sampler_stage, S3_SUBMIT_LINE, __ATOMIC_RELAXED);
        vehicle_submit_line(left, center < 0 ? 0 : center, right,
                            command.left, command.right, command.stop,
                            letter ? (unsigned)(letter - 'A' + 1) : 0u);
    }
}
