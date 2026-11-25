#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdbool.h>
#include <stdint.h>
#include "s5_profile.h"

#define S5_RANGE_MAX_AGE_MS 300u
#define S5_SLOW_RANGE_MM 500u
#define S5_MANEUVER_STOP_MM 150u
#define S5_PASS_BODY_ALLOWANCE_MM 160u
#ifndef S5_PASS_DEPTH_PERCENT
#define S5_PASS_DEPTH_PERCENT 55u
#endif
#ifndef S5_PASS_UNCERTAIN_DEPTH_MM
#define S5_PASS_UNCERTAIN_DEPTH_MM 130u
#endif
#define S5_MAX_PASS_MM 720u
#define S5_APPROACH_RPM 20u
#define S5_CLEARANCE_BEARING_DEG 45u
#define S5_OUTWARD_CLEARANCE_BEARING_DEG 60u
#ifndef S5_CHASSIS_WIDTH_MM
#define S5_CHASSIS_WIDTH_MM 160u /* Verify the widest point, including tyres. */
#endif
#define S5_SIDE_GAP_MM 55u
#define S5_SIDE_ALLOWANCE_MM ((S5_CHASSIS_WIDTH_MM + 1u) / 2u + S5_SIDE_GAP_MM)
#ifndef S5_IR_TO_AXLE_MM
#define S5_IR_TO_AXLE_MM 130u
#endif
#ifndef S5_OUTER_IR_SPACING_MM
#define S5_OUTER_IR_SPACING_MM 30u
#endif
#define S5_UNCERTAIN_EDGE_MM 25u
#define S5_REVERSE_MM 30u
#define S5_LONG_REVERSE_MM 60u
#define S5_SEARCH_MARGIN_MM 40u
#define S5_SEARCH_EXTENSION_MM 60u
#define S5_MAX_LEG_MM 900u
#ifndef S5_TAPE_WIDTH_MM
#define S5_TAPE_WIDTH_MM 18u
#endif

typedef enum {
    S5_OFF, S5_PROFILE_PATH, S5_TURN_OUT, S5_SHIFT_OUT, S5_ALIGN_OUTSIDE,
    S5_PASS, S5_TURN_IN, S5_FIND_LINE, S5_CENTER_LINE, S5_ALIGN_LINE,
    /* Keep the phase numbers stable for MQTT diagnostics. */
    S5_COMPLETE = 11, S5_FAILED, S5_CHECK_LINE, S5_BACK_UP, S5_WAIT_PATH
} S5_PHASE;
typedef enum { S5_NO_SIDE, S5_LEFT, S5_RIGHT } S5_SIDE;

typedef enum {
    S5_WAIT, S5_STOP, S5_ADVANCE, S5_REVERSE, S5_RESCAN,
    S5_ARC_LEFT, S5_ARC_RIGHT
} S5_ACTION;
typedef enum { S5_MOTION_NONE, S5_MOTION_QUEUED, S5_MOTION_PENDING, S5_MOTION_DONE, S5_MOTION_FAILED } S5_MOTION;
typedef enum {
    S5_FAIL_NONE, S5_FAIL_SCAN_TIMEOUT, S5_FAIL_TARGET, S5_FAIL_WIDTH, S5_FAIL_ROOM,
    S5_FAIL_SIDE, S5_FAIL_ECHO, S5_FAIL_CLOSE, S5_FAIL_MOTION, S5_FAIL_LINE,
    S5_FAIL_STOP, S5_FAIL_INTERLOCK, S5_FAIL_PHASE_TIMEOUT,
    S5_FAIL_LEFT_LIMIT, S5_FAIL_RIGHT_LIMIT, S5_FAIL_BOTH_LIMITS
} S5_FAILURE;

typedef struct {
    uint32_t now, range_at, profile_at, line_at;
    S5_PROFILE profile;
    unsigned range_mm : 12;
    unsigned search_progress_mm : 10;
    unsigned motion : 3;
    bool stopped : 1, range_valid : 1, profile_valid : 1;
    bool line_hit : 1, line_junction : 1, line_centered : 1, can_retrace : 1;
    bool near_original_line : 1, can_retrace_far : 1, line_center_seen : 1, retry_scan : 1;
    bool left_on_tape : 1, right_on_tape : 1;
} S5_AVOID_INPUT;
typedef struct {
    unsigned entry_mm:10, samples:2;
} S5_TAPE_EDGE;

typedef struct {
    uint32_t phase_at, last_line_at, last_range_at;
    unsigned phase:5, side:2, line_samples:2, reverse_available:1;
    unsigned reverse_long:1, scan_retried:1, scan_after_retrace:1, failure:4, failed_phase:5;
    unsigned offset_mm:10, diagonal_mm:10, pass_mm:10, search_mm:10, search_after_mm:10;
    unsigned align_at_mm:10, search_progress_mm:10;
    unsigned blocked_range_mm:12, range_seen:1, search_exhausted:1;
    S5_TAPE_EDGE left_edge, right_edge;
} S5_AVOID;
typedef struct {
    uint16_t action : 4;
    uint16_t distance_mm : 10;
} S5_AVOID_OUTPUT;

typedef struct {
    unsigned side:2, failure:4;
    unsigned offset_mm:9;
    unsigned probe_left:1, probe_right:1;
} S5_BYPASS;

S5_BYPASS s5_plan_bypass(const S5_PROFILE *profile);
S5_BYPASS s5_plan_bypass_at_range(const S5_PROFILE *profile, unsigned limit_mm);
unsigned s5_scan_range_limit(const S5_AVOID *state);
unsigned s5_planned_pass_mm(const S5_PROFILE *profile, unsigned forward_mm);
bool s5_avoid_executing(S5_PHASE phase);
void s5_avoid_begin(S5_AVOID *state, uint32_t now);
void s5_avoid_abort(S5_AVOID *state, S5_FAILURE reason, uint32_t now);
S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in);
#endif
