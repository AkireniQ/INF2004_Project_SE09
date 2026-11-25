#include "subsystem3.h"
#include "barcode.h"

#define BARCODE_REPORT_MS 2000u

typedef struct {
    UW bars, starting_bars, adc_errors, gaps, max_gap_ms, width_rejects;
    UW sampled_at, reported_at;
    uint16_t min_adc, max_adc;
    uint8_t max_elements;
    bool following, has_adc;
    char letter;
} BARCODE_REPORT;

/* Both tasks are pinned to core 0. Copy with local interrupts masked, then
   release the copy before doing any USB console I/O. */
LOCAL BARCODE_REPORT latest;
LOCAL UW report_id;
LOCAL ID log_task;

LOCAL void publish_report(const BARCODE_REPORT *report)
{
    UW saved;
    DI(saved);
    latest = *report;
    ++report_id;
    ID task = log_task;
    EI(saved);
    if (task > 0) (void)tk_wup_tsk(task);
}

EXPORT void s3_record_barcode_sample(BOOL following, INT adc, UW now_ms, char letter)
{
    LOCAL BARCODE_REPORT report;
    if (!following) {
        if (report.following) {
            report.following = false;
            publish_report(&report);
        }
        return;
    }

    bool first = !report.following;
    if (first) report = (BARCODE_REPORT){.following = true, .min_adc = 4095};
    if (adc < 0) {
        ++report.adc_errors;
    } else {
        if (report.has_adc) {
            UW gap = now_ms - report.sampled_at;
            if (gap > report.max_gap_ms) report.max_gap_ms = gap;
            if (gap > BARCODE_MAX_SAMPLE_GAP_MS) ++report.gaps;
        }
        report.has_adc = true;
        report.sampled_at = now_ms;
        if (adc < report.min_adc) report.min_adc = (uint16_t)adc;
        if (adc > report.max_adc) report.max_adc = (uint16_t)adc;
    }

    BARCODE_PROGRESS progress = barcode_progress();
    bool rejected = !first && progress.width_rejects != report.width_rejects;
    report.width_rejects = progress.width_rejects;
    if (first) report.starting_bars = progress.bars;
    report.bars = progress.bars - report.starting_bars;
    unsigned elements = letter ? BARCODE_FRAME_ELEMENTS : progress.elements;
    if (elements > report.max_elements) report.max_elements = (uint8_t)elements;
    if (letter) report.letter = letter;
    if (first || letter || rejected || now_ms - report.reported_at >= BARCODE_REPORT_MS) {
        report.reported_at = now_ms;
        publish_report(&report);
    }
}

EXPORT void s3_barcode_log_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    ID task = tk_get_tid();
    UW saved;
    DI(saved);
    log_task = task;
    EI(saved);

    UW seen = 0;
    for (;;) {
        BARCODE_REPORT report;
        DI(saved);
        UW next = report_id;
        if (next != seen) report = latest;
        EI(saved);
        if (next != seen) {
            seen = next;
            tm_printf((UB *)"[BC] %s GP27=%d..%d bars=%u window=%u adc_errors=%u "
                            "sample_gaps=%u max_gap=%u ms width_rejects=%u decoded=%c\n",
                      report.following ? "following" : "paused",
                      report.has_adc ? (INT)report.min_adc : -1,
                      report.has_adc ? (INT)report.max_adc : -1,
                      (unsigned)report.bars, (unsigned)report.max_elements,
                      (unsigned)report.adc_errors, (unsigned)report.gaps,
                      (unsigned)report.max_gap_ms, (unsigned)report.width_rejects,
                      report.letter ? report.letter : '-');
        }
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
