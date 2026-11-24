#include "subsystem3.h"

#if APP_IR_BENCH
#define IR_REPORT_MS 250u

typedef struct {
    INT raw[3];
    unsigned errors[3];
    unsigned scans;
    UW sampled_at;
} IR_BENCH_SAMPLE;

LOCAL IR_BENCH_SAMPLE latest;
LOCAL ID sample_guard;
LOCAL unsigned reading_gpio;

LOCAL void ir_bench_sample_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    IR_BENCH_SAMPLE sample = {0};

    for (;;) {
        for (INT channel = 0; channel < 3; ++channel) {
            __atomic_store_n(&reading_gpio, 26u + (unsigned)channel, __ATOMIC_RELAXED);
            sample.raw[channel] = ir_read(channel);
            if (sample.raw[channel] < 0) ++sample.errors[channel];
        }
        __atomic_store_n(&reading_gpio, 0u, __ATOMIC_RELAXED);
        SYSTIM now;
        (void)tk_get_otm(&now);
        sample.sampled_at = now.lo;
        ++sample.scans;

        if (tk_loc_mtx(sample_guard, TMO_FEVR) == E_OK) {
            latest = sample;
            (void)tk_unl_mtx(sample_guard);
        }
        (void)tk_dly_tsk(LINE_PERIOD_MS);
    }
}

EXPORT void ir_bench_run(void)
{
    T_CMTX mutex = {.mtxatr = TA_INHERIT};
    sample_guard = tk_cre_mtx(&mutex);
    CHECK(sample_guard, "IR bench sample mutex");
    if (sample_guard < E_OK) return;

    ID task = app_start_task((FP)ir_bench_sample_task, S3_PRI_LINE, TP_PRC1, "ir_bench");
    if (task < E_OK) return;
    tm_printf((UB *)"[IR] Sensor bench. Raw ADC 0..4095; -1 means read error.\n");

    /* The reporter can still show scan age if the sampler is waiting on ADC.
       The sample mutex is held only while copying data. */
    for (;;) {
        IR_BENCH_SAMPLE sample = {0};
        if (tk_loc_mtx(sample_guard, TMO_FEVR) == E_OK) {
            sample = latest;
            (void)tk_unl_mtx(sample_guard);
        }
        SYSTIM now;
        (void)tk_get_otm(&now);
        unsigned active = __atomic_load_n(&reading_gpio, __ATOMIC_RELAXED);
        if (sample.scans) {
            tm_printf((UB *)"[IR] scan=%u age=%u ms L(GP26)=%d C(GP27)=%d R(GP28)=%d "
                            "errors L/C/R=%u/%u/%u adc_gp=%u\n",
                      sample.scans, (unsigned)(now.lo - sample.sampled_at),
                      sample.raw[0], sample.raw[1], sample.raw[2],
                      sample.errors[0], sample.errors[1], sample.errors[2], active);
        } else {
            tm_printf((UB *)"[IR] Waiting for first ADC scan (adc_gp=%u)\n", active);
        }
        (void)tk_dly_tsk(IR_REPORT_MS);
    }
}
#endif
